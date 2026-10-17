/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-devkit/src/esp32s3_bringup.c
 *
 * M5 Cardputer ADV — Hacker Green UI   (low-RAM + Wi-Fi edition)
 *
 *   Fixed phosphor-green theme (#00FF41).
 *
 *   RAM model  (what changed vs. the first version)
 *   ---------
 *   • No full framebuffer any more.  Every screen is a "compose" callback
 *     that draws into a 240 x 16 px BAND buffer (7,680 B).  gfx_render()
 *     runs the callback once per band and pushes each band to the ST7789
 *     with ONE putarea() SPI transfer.  The old 64,800 B g_fb is gone.
 *   • Scrollback stores characters only.  The old colour plane
 *     (200 x 30 x 2 = 12,000 B) always held COL_FG, so it was dropped.
 *   • File manager no longer caches 64 directory entries (≈2.7 KB); it
 *     re-reads the directory and keeps only the 6 visible names.
 *   • The Wi-Fi app's state (scan list, password) lives on the heap only
 *     while the app is open and is wiped + freed when you leave it.
 *   • Battery ADC is read once per tick into a cache instead of on every
 *     status-bar draw.
 *
 *   Static .bss (approx.)          before      after
 *     framebuffer / band          64,800      7,680
 *     SPI byte-swap line buffer      480          0
 *     scrollback chars             6,000      6,000
 *     scrollback colours          12,000          0
 *     file-manager cache           2,752       ~300
 *     ring buffers                 1,024        768
 *     kthread stacks (heap)        8,192      6,656
 *                                ~95 KB     ~21 KB
 *
 *   CPU / Wi-Fi friendliness
 *   ------------------------
 *   • Keyboard thread sleeps (usleep) instead of busy-waiting (up_mdelay).
 *   • Soft-I2C no longer disables interrupts for a whole transaction;
 *     I2C tolerates a stretched SCL, Wi-Fi does not tolerate IRQ-off time.
 *   • A newline now costs 8 SPI transfers instead of 119.
 *
 *   Key routing
 *   -----------
 *   MENU / ScreenTest / Files / Wi-Fi list : i ; = UP, k . = DOWN (no fn)
 *   TERMINAL : fn+UP / fn+DOWN scroll back, other arrows → VT100 to shell
 *   OPT      : back to menu from anywhere
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/kthread.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/lcd_dev.h>
#include <nuttx/video/fb.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#ifdef CONFIG_SYSLOG
#  include <nuttx/syslog/syslog.h>
#endif

#include <arch/board/board.h>
#include "espressif/esp_gpio.h"
#include "esp32s3_start.h"

#if defined(CONFIG_ESPRESSIF_ADC) || defined(CONFIG_ESP32S3_ADC)
#  include "esp32s3_board_adc.h"
#endif

#ifdef CONFIG_ESPRESSIF_WIFI
#  include "esp32s3_board_wlan.h"
#endif

#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <poll.h>
#include <time.h>

/* Wi-Fi UI is built only when the network stack and the wireless ioctls
 * are available.  Everything else works without it.
 */

#if defined(CONFIG_NET) && defined(CONFIG_NETDEV_WIRELESS_IOCTL) && \
    (defined(CONFIG_ESPRESSIF_WIFI) || defined(CONFIG_ESP32S3_WIFI))
#  define CARDPUTER_WIFI 1
#  include <sys/socket.h>
#  include <net/if.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <nuttx/net/net.h>
#  include <nuttx/wireless/wireless.h>
#  ifdef CONFIG_LIBC_NETDB
#    include <netdb.h>
#  endif
#endif

/* Saved Wi-Fi networks live in a raw 4 KB flash sector (no filesystem, so
 * no FS caches in RAM).  Needs the ESP SPI-flash MTD driver; disabled for
 * MCUboot images.  With a partition table (legacy IDF format, e.g. when
 * installed by M5 Launcher) only a sector that no partition claims is used.
 */

#if defined(CARDPUTER_WIFI) && defined(CONFIG_ESPRESSIF_MTD) && \
    defined(CONFIG_MTD_PARTITION) && !defined(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT)
#  define CARDPUTER_WIFI_STORE 1
#  include <nuttx/mtd/mtd.h>
FAR struct mtd_dev_s *esp_spiflash_alloc_mtdpart(uint32_t mtd_offset,
                                                 uint32_t mtd_size);
#endif

/* Boot trace.  Every step is listed on the LCD boot screen (all builds) so
 * a boot that stops shows *where* it stops with no PC attached; debug
 * builds also send it to syslog.
 */

static void boot_step(FAR const char *fmt, ...)
  __attribute__((format(printf, 1, 2)));

#ifdef CONFIG_DEBUG_FEATURES
#  define BTRACE(...) \
     do { syslog(LOG_ERR, "cardputer: " __VA_ARGS__); \
          boot_step(__VA_ARGS__); } while (0)
#else
#  define BTRACE(...) boot_step(__VA_ARGS__)
#endif

/* Debug builds: a crash is drawn on the LCD (bit-banged, so it works even
 * when the crash happened inside the SPI/LCD driver or before it started).
 */

#ifdef CONFIG_DEBUG_FEATURES
#  define CARDPUTER_PANIC_SCREEN 1
#  include <nuttx/notifier.h>
#  include <nuttx/panic_notifier.h>
#  include <nuttx/sched.h>
#endif

/****************************************************************************
 * Tunables  (override with -D or edit here)
 ****************************************************************************/

#ifndef CARDPUTER_SCROLLBACK_LINES
#  define CARDPUTER_SCROLLBACK_LINES 200      /* 30 B per line           */
#endif

#ifndef CARDPUTER_LCDCONS_STACK
#  define CARDPUTER_LCDCONS_STACK    3072     /* was 4096                */
#endif

#ifndef CARDPUTER_KBD_STACK
#  define CARDPUTER_KBD_STACK        4096     /* Wi-Fi scan + DNS for NTP */
#endif

#define KBD_RING_SIZE   256    /* keys + injected Wi-Fi commands          */
#define OUT_RING_SIZE   512    /* shell output burst buffer               */

/****************************************************************************
 * External LCD driver symbols
 ****************************************************************************/

FAR struct lcd_dev_s *st7789_lcdinitialize(FAR struct spi_dev_s *spi);

#ifdef CONFIG_ESP32S3_SPIFLASH
int board_spiflash_init(void);
#endif

#ifdef CONFIG_ESPRESSIF_HR_TIMER
int esp_hr_timer_init(void);   /* arch/xtensa/src/common/espressif */
#endif

/****************************************************************************
 * TCA8418 / keyboard constants
 ****************************************************************************/

#define TCA8418_ADDR   0x34
#define PIN_SDA        8
#define PIN_SCL        9

#define KEY_LEFT_CTRL  0x80
#define KEY_LEFT_SHIFT 0x81
#define KEY_LEFT_ALT   0x82
#define KEY_UP         0x83
#define KEY_DOWN       0x84

#define KEY_FN         0xff
#define KEY_OPT        0x00

#define KEY_BACKSPACE  0x7f
#define KEY_TAB        0x09
#define KEY_ENTER      0x0d

/****************************************************************************
 * LCD geometry / text grid
 ****************************************************************************/

#define LCD_W     240
#define LCD_H     135
#define FONT_W    8
#define FONT_H    16
#define COLS      (LCD_W / FONT_W)        /* 30 */
#define ROWS      (LCD_H / FONT_H)        /*  8 */

#define BAR_ROW   0
#define BAR_H     FONT_H
#define TOP_ROW   1
#define BOT_ROW   (ROWS - 1)
#define TERM_ROWS (ROWS - TOP_ROW)        /* 7 visible terminal rows */

#define TERM_Y0   (TOP_ROW * FONT_H)
#define TERM_Y1   (LCD_H - 1)

/****************************************************************************
 * Colour palette — fixed hacker green
 ****************************************************************************/

#define COL_ACCENT   0x07E8u
#define COL_MARKER   0x4FEAu
#define COL_SEL_FG   0xFFFFu
#define COL_FG       0xAFF5u
#define COL_DIM      0x0360u
#define COL_BG       0x0040u
#define COL_BG2      0x0060u
#define COL_BAR_BG   0x0040u
#define COL_SEL_BG   0x0120u
#define COL_SEL_BG2  0x0200u
#define COL_OK       0x07E8u
#define COL_WARN     0x4FEAu
#define COL_ERR      0xAFF5u

#define RGB_RED      0xF800u
#define RGB_GREEN    0x07E0u
#define RGB_BLUE     0x001Fu
#define RGB_YELLOW   0xFFE0u
#define RGB_CYAN     0x07FFu
#define RGB_MAGENTA  0xF81Fu
#define RGB_WHITE    0xFFFFu
#define RGB_BLACK    0x0000u

/****************************************************************************
 * Embedded font (JetBrains Mono 8×16) — const, lives in flash (DROM)
 ****************************************************************************/

#define GFX_FONT_FIRST 32
#define GFX_FONT_LAST  126
#define GFX_FONT_W     8
#define GFX_FONT_H     16
static const uint8_t g_font8x16[(126-32+1)*16] =
{
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,0x00,0x18,0x00,
  0x00,0x00,0x00,0x00,0x00,0x24,0x24,0x24,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x12,0x12,0x7F,0x26,0x24,0x24,0xFE,0x64,0x44,0x4C,0x00,
  0x00,0x00,0x08,0x08,0x1C,0x7E,0x4A,0x48,0x78,0x3C,0x0E,0x0A,0x4A,0x6E,0x3C,0x08,
  0x00,0x00,0x00,0x00,0x00,0x79,0x59,0x4A,0x7E,0x04,0x0F,0x1D,0x34,0x25,0x43,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3C,0x24,0x20,0x30,0x30,0x5A,0x4E,0x44,0x6E,0x7A,0x00,
  0x00,0x00,0x00,0x00,0x00,0x08,0x08,0x08,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x06,0x0C,0x18,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x18,0x08,0x0E,
  0x00,0x00,0x00,0x20,0x38,0x08,0x0C,0x04,0x04,0x04,0x04,0x04,0x04,0x0C,0x18,0x30,
  0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x08,0x6B,0x7E,0x18,0x34,0x26,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x08,0x08,0x7F,0x08,0x08,0x08,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x18,0x18,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,
  0x00,0x00,0x00,0x02,0x02,0x06,0x04,0x0C,0x0C,0x08,0x18,0x10,0x10,0x30,0x20,0x20,
  0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x42,0x42,0x52,0x42,0x42,0x42,0x66,0x3C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x18,0x78,0x48,0x08,0x08,0x08,0x08,0x08,0x08,0x7E,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x42,0x02,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00,
  0x00,0x00,0x00,0x00,0x00,0x7C,0x04,0x18,0x1C,0x06,0x02,0x02,0x42,0x66,0x3C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x04,0x0C,0x18,0x10,0x22,0x62,0x42,0x7E,0x02,0x02,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3F,0x20,0x20,0x3E,0x33,0x01,0x01,0x01,0x33,0x1E,0x00,
  0x00,0x00,0x00,0x00,0x00,0x08,0x10,0x30,0x3C,0x66,0x42,0x42,0x42,0x66,0x3C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x7E,0x42,0x46,0x04,0x04,0x0C,0x08,0x18,0x10,0x30,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x42,0x66,0x3C,0x3C,0x46,0x42,0x66,0x3C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x42,0x42,0x66,0x3E,0x0C,0x08,0x18,0x10,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x18,0x18,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x18,0x18,0x10,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x1C,0x70,0x60,0x38,0x0E,0x02,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x38,0x0E,0x06,0x1C,0x30,0x40,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x78,0x0C,0x04,0x0C,0x18,0x10,0x10,0x00,0x00,0x30,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3E,0x63,0x41,0x4F,0x49,0x49,0x49,0x49,0x4B,0x4F,0x60,
  0x00,0x00,0x00,0x00,0x00,0x18,0x1C,0x1C,0x34,0x24,0x26,0x3E,0x62,0x42,0x43,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3E,0x23,0x21,0x23,0x3E,0x23,0x21,0x21,0x23,0x3E,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x42,0x40,0x40,0x40,0x40,0x42,0x66,0x3C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x7C,0x46,0x42,0x42,0x42,0x42,0x42,0x42,0x46,0x7C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3F,0x20,0x20,0x20,0x3E,0x20,0x20,0x20,0x20,0x3F,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3F,0x20,0x20,0x20,0x3F,0x20,0x20,0x20,0x20,0x20,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x42,0x40,0x40,0x4E,0x42,0x42,0x66,0x3C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x21,0x21,0x21,0x21,0x3F,0x21,0x21,0x21,0x21,0x21,0x00,
  0x00,0x00,0x00,0x00,0x00,0x7C,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x1E,0x02,0x02,0x02,0x02,0x02,0x02,0x42,0x66,0x3C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x46,0x44,0x4C,0x48,0x78,0x48,0x4C,0x44,0x46,0x42,0x00,
  0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x3F,0x00,
  0x00,0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x5A,0x5A,0x5A,0x42,0x42,0x42,0x42,0x00,
  0x00,0x00,0x00,0x00,0x00,0x62,0x62,0x72,0x52,0x52,0x5A,0x4A,0x4E,0x46,0x46,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x42,0x42,0x42,0x42,0x42,0x42,0x66,0x3C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x7C,0x46,0x42,0x42,0x46,0x7C,0x40,0x40,0x40,0x40,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x42,0x42,0x42,0x42,0x42,0x42,0x66,0x3C,0x0C,
  0x00,0x00,0x00,0x00,0x00,0x7C,0x46,0x42,0x46,0x7C,0x48,0x4C,0x44,0x46,0x46,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x42,0x40,0x38,0x0C,0x06,0x42,0x66,0x3C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x7F,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,
  0x00,0x00,0x00,0x00,0x00,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x66,0x3C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x43,0x42,0x62,0x26,0x26,0x24,0x34,0x1C,0x1C,0x18,0x00,
  0x00,0x00,0x00,0x00,0x00,0xD9,0xD9,0x59,0x5B,0x53,0x56,0x76,0x66,0x66,0x66,0x00,
  0x00,0x00,0x00,0x00,0x00,0x62,0x66,0x34,0x1C,0x18,0x18,0x1C,0x24,0x66,0x43,0x00,
  0x00,0x00,0x00,0x00,0x00,0x41,0x62,0x22,0x34,0x1C,0x18,0x08,0x08,0x08,0x08,0x00,
  0x00,0x00,0x00,0x00,0x00,0x7E,0x06,0x04,0x0C,0x08,0x10,0x30,0x20,0x60,0x7E,0x00,
  0x00,0x00,0x00,0x1C,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x1C,
  0x00,0x00,0x00,0x40,0x60,0x20,0x30,0x30,0x10,0x18,0x08,0x08,0x0C,0x04,0x04,0x06,
  0x00,0x00,0x00,0x38,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x38,
  0x00,0x00,0x00,0x00,0x08,0x18,0x14,0x24,0x26,0x62,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,
  0x00,0x00,0x00,0x00,0x10,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x02,0x3E,0x62,0x42,0x66,0x3A,0x00,
  0x00,0x00,0x00,0x00,0x00,0x40,0x40,0x7C,0x66,0x42,0x42,0x42,0x42,0x66,0x5C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x42,0x40,0x40,0x42,0x66,0x3C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x02,0x02,0x3E,0x66,0x42,0x42,0x42,0x42,0x66,0x3A,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x42,0x7E,0x40,0x40,0x66,0x3C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x1F,0x10,0x10,0x7F,0x10,0x10,0x10,0x10,0x10,0x10,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3A,0x66,0x42,0x42,0x42,0x42,0x66,0x3A,0x02,
  0x00,0x00,0x00,0x00,0x00,0x40,0x40,0x5C,0x66,0x42,0x42,0x42,0x42,0x42,0x42,0x00,
  0x00,0x00,0x00,0x00,0x08,0x08,0x00,0x78,0x08,0x08,0x08,0x08,0x08,0x08,0x7F,0x00,
  0x00,0x00,0x00,0x00,0x0C,0x0C,0x00,0x7C,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,
  0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x23,0x22,0x24,0x3C,0x24,0x26,0x22,0x23,0x00,
  0x00,0x00,0x00,0x00,0x00,0xF0,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x1E,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x5E,0x4A,0x4A,0x4A,0x4A,0x4A,0x4A,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5C,0x66,0x42,0x42,0x42,0x42,0x42,0x42,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x42,0x42,0x42,0x42,0x66,0x3C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5C,0x66,0x42,0x42,0x42,0x42,0x66,0x7C,0x40,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3A,0x66,0x42,0x42,0x42,0x42,0x66,0x3E,0x02,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2E,0x33,0x21,0x20,0x20,0x20,0x20,0x20,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1E,0x33,0x20,0x3C,0x0F,0x01,0x23,0x3E,0x00,
  0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x7F,0x10,0x10,0x10,0x10,0x10,0x10,0x1F,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x42,0x42,0x42,0x42,0x42,0x42,0x66,0x3C,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x43,0x62,0x62,0x26,0x34,0x34,0x1C,0x18,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD9,0x59,0x5B,0x5A,0x56,0x76,0x66,0x26,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x62,0x26,0x3C,0x18,0x18,0x34,0x26,0x62,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x43,0x62,0x26,0x24,0x34,0x1C,0x18,0x18,0x18,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x06,0x0C,0x08,0x10,0x30,0x60,0x7E,0x00,
  0x00,0x00,0x00,0x06,0x0C,0x08,0x08,0x08,0x18,0x70,0x18,0x08,0x08,0x08,0x08,0x0E,
  0x00,0x00,0x00,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
  0x00,0x00,0x00,0x60,0x30,0x10,0x10,0x10,0x10,0x1E,0x18,0x10,0x10,0x10,0x30,0x60,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x72,0x5A,0x4E,0x00,0x00,0x00,0x00,0x00,
};

/****************************************************************************
 * Graphics library — banded RGB565 renderer
 *
 *   g_band holds BAND_H pixel rows [g_by0 .. g_by1] of the screen.
 *   Every primitive clips against that window, so a compose callback can
 *   draw the WHOLE screen and only the part inside the band lands in RAM.
 *   Outside gfx_render() the window is empty and all drawing is a no-op.
 ****************************************************************************/

#define BAND_H       16
#define BAND_PIXELS  (LCD_W * BAND_H)

#ifndef GFX_SWAP_BYTES
#  define GFX_SWAP_BYTES 1
#endif

typedef void (*compose_t)(void);

static struct lcd_dev_s        *g_lcddev = NULL;
static struct lcd_planeinfo_s   g_pinfo;
static bool                     g_gfx_ready = false;
static uint16_t                 g_band[BAND_PIXELS] aligned_data(4);
static int                      g_by0 = 0;     /* band top    (inclusive) */
static int                      g_by1 = -1;    /* band bottom (inclusive) */
static mutex_t                  g_con_mutex = NXMUTEX_INITIALIZER;

static int gfx_init(void)
{
  if (g_lcddev == NULL) return -ENODEV;
  memset(&g_pinfo, 0, sizeof(g_pinfo));
  if (g_lcddev->getplaneinfo(g_lcddev, 0, &g_pinfo) < 0) return -EIO;
  g_gfx_ready = (g_pinfo.putarea != NULL || g_pinfo.putrun != NULL);
  return g_gfx_ready ? OK : -ENOSYS;
}

static inline FAR uint16_t *gfx_rowp(int y)
{
  return &g_band[(y - g_by0) * LCD_W];
}

/* Clip a [*ya, *yb] row range to the current band. */

static inline bool gfx_clip_y(FAR int *ya, FAR int *yb)
{
  if (*ya < g_by0) *ya = g_by0;
  if (*yb > g_by1) *yb = g_by1;
  return *ya <= *yb;
}

static inline bool gfx_band_hits(int y, int h)
{
  return (y <= g_by1) && (y + h - 1 >= g_by0);
}

/* Push band rows [y0 .. y0+h-1] to the panel.  Bytes are swapped in place:
 * the band is recomposed from scratch before the next flush anyway.
 */

static void gfx_band_flush(int y0, int h)
{
  FAR struct lcd_dev_s *dev = g_pinfo.dev ? g_pinfo.dev : g_lcddev;
  int n = h * LCD_W;

#if GFX_SWAP_BYTES
  for (int i = 0; i < n; i++)
    {
      uint16_t v = g_band[i];
      g_band[i] = (uint16_t)((v >> 8) | (v << 8));
    }
#else
  UNUSED(n);
#endif

  if (g_pinfo.putarea != NULL)
    {
      /* stride == row size → the ST7789 driver does ONE SPI transfer */

      g_pinfo.putarea(dev, y0, y0 + h - 1, 0, LCD_W - 1,
                      (FAR const uint8_t *)g_band, LCD_W * 2);
    }
  else
    {
      for (int r = 0; r < h; r++)
        g_pinfo.putrun(dev, y0 + r, 0,
                       (FAR const uint8_t *)&g_band[r * LCD_W], LCD_W);
    }
}

/* Draw screen rows [y0..y1] with `fn` and send them to the LCD.
 * Caller must hold g_con_mutex (the band buffer is shared).
 */

static void gfx_render(compose_t fn, int y0, int y1)
{
  if (!g_gfx_ready) return;
  if (y0 < 0) y0 = 0;
  if (y1 > LCD_H - 1) y1 = LCD_H - 1;

  for (int y = y0; y <= y1; y += BAND_H)
    {
      int h = y1 - y + 1;
      if (h > BAND_H) h = BAND_H;

      g_by0 = y;
      g_by1 = y + h - 1;

      for (int i = 0; i < h * LCD_W; i++) g_band[i] = COL_BG;
      fn();
      gfx_band_flush(y, h);
    }

  g_by0 = 0;
  g_by1 = -1;          /* stray draws outside a render become no-ops */
}

static inline void gfx_render_all(compose_t fn)
{
  gfx_render(fn, 0, LCD_H - 1);
}

static void compose_nothing(void) { }

static inline void gfx_px(int x, int y, uint16_t c)
{
  if ((unsigned)x < LCD_W && y >= g_by0 && y <= g_by1)
    gfx_rowp(y)[x] = c;
}

static void gfx_hline(int x, int y, int w, uint16_t c)
{
  if (y < g_by0 || y > g_by1 || w <= 0) return;
  if (x < 0) { w += x; x = 0; }
  if (x + w > LCD_W) w = LCD_W - x;
  if (w <= 0) return;
  FAR uint16_t *p = gfx_rowp(y) + x;
  while (w-- > 0) *p++ = c;
}

static void gfx_vline(int x, int y, int h, uint16_t c)
{
  if ((unsigned)x >= LCD_W || h <= 0) return;
  int ya = y, yb = y + h - 1;
  if (!gfx_clip_y(&ya, &yb)) return;
  for (int yy = ya; yy <= yb; yy++) gfx_rowp(yy)[x] = c;
}

static void gfx_fill_rect(int x, int y, int w, int h, uint16_t c)
{
  int ya = y, yb = y + h - 1;
  if (!gfx_clip_y(&ya, &yb)) return;
  for (int yy = ya; yy <= yb; yy++) gfx_hline(x, yy, w, c);
}

static void gfx_clear(uint16_t c)
{
  int n = (g_by1 - g_by0 + 1) * LCD_W;
  for (int i = 0; i < n; i++) g_band[i] = c;
}

static void gfx_line(int x0, int y0, int x1, int y1, uint16_t c)
{
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (;;)
    {
      gfx_px(x0, y0, c);
      if (x0 == x1 && y0 == y1) break;
      int e2 = 2 * err;
      if (e2 >= dy) { err += dy; x0 += sx; }
      if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static int isqrt32(int v)
{
  if (v <= 0) return 0;
  int x = v, y = (x + 1) / 2;
  while (y < x) { x = y; y = (x + v / x) / 2; }
  return x;
}

/* Row index range of a shape [y, y+h) that falls inside the band. */

static inline bool gfx_rows(int y, int h, FAR int *j0, FAR int *j1)
{
  *j0 = g_by0 - y; if (*j0 < 0) *j0 = 0;
  *j1 = g_by1 - y; if (*j1 > h - 1) *j1 = h - 1;
  return *j0 <= *j1;
}

static inline int gfx_round_inset(int j, int h, int r)
{
  int dy = -1;
  if (j < r)           dy = r - j;
  else if (j >= h - r) dy = r - (h - 1 - j);
  return (dy >= 0) ? r - isqrt32(r * r - dy * dy) : 0;
}

static void gfx_fill_round(int x, int y, int w, int h, int r, uint16_t c)
{
  int j0, j1;
  if (w <= 0 || h <= 0 || !gfx_rows(y, h, &j0, &j1)) return;
  if (r < 0) r = 0;
  if (r > w / 2) r = w / 2;
  if (r > h / 2) r = h / 2;
  for (int j = j0; j <= j1; j++)
    {
      int inset = gfx_round_inset(j, h, r);
      gfx_hline(x + inset, y + j, w - 2 * inset, c);
    }
}

static void gfx_round(int x, int y, int w, int h, int r, uint16_t c)
{
  if (w <= 0 || h <= 0 || !gfx_band_hits(y, h)) return;
  if (r < 0) r = 0;
  if (r > w / 2) r = w / 2;
  if (r > h / 2) r = h / 2;
  gfx_hline(x + r, y,         w - 2 * r, c);
  gfx_hline(x + r, y + h - 1, w - 2 * r, c);
  gfx_vline(x,         y + r, h - 2 * r, c);
  gfx_vline(x + w - 1, y + r, h - 2 * r, c);
  int f = 1 - r, ddx = 1, ddy = -2 * r, px = 0, py = r;
  int x0 = x + r, y0 = y + r, x1 = x + w - 1 - r, y1 = y + h - 1 - r;
  while (px <= py)
    {
      gfx_px(x1+px,y1+py,c); gfx_px(x0-px,y1+py,c);
      gfx_px(x1+px,y0-py,c); gfx_px(x0-px,y0-py,c);
      gfx_px(x1+py,y1+px,c); gfx_px(x0-py,y1+px,c);
      gfx_px(x1+py,y0-px,c); gfx_px(x0-py,y0-px,c);
      if (f >= 0) { py--; ddy += 2; f += ddy; }
      px++; ddx += 2; f += ddx;
    }
}

static void gfx_fill_circle(int cx, int cy, int r, uint16_t c)
{
  int d0 = -r, d1 = r;
  if (cy + d0 < g_by0) d0 = g_by0 - cy;
  if (cy + d1 > g_by1) d1 = g_by1 - cy;
  for (int dy = d0; dy <= d1; dy++)
    {
      int half = isqrt32(r * r - dy * dy);
      gfx_hline(cx - half, cy + dy, 2 * half + 1, c);
    }
}

static inline uint16_t gfx_blend565(uint16_t a, uint16_t b, int t)
{
  int ar=(a>>11)&0x1F, ag=(a>>5)&0x3F, ab=a&0x1F;
  int br=(b>>11)&0x1F, bg=(b>>5)&0x3F, bb=b&0x1F;
  int r=ar+(br-ar)*t/32, g=ag+(bg-ag)*t/32, bl=ab+(bb-ab)*t/32;
  return (uint16_t)((r<<11)|(g<<5)|bl);
}

static void gfx_vgrad(int x, int y, int w, int h, uint16_t c0, uint16_t c1)
{
  int j0, j1;
  if (h <= 0 || !gfx_rows(y, h, &j0, &j1)) return;
  for (int j = j0; j <= j1; j++)
    {
      int t = (h > 1) ? (j * 32 / (h - 1)) : 0;
      gfx_hline(x, y + j, w, gfx_blend565(c0, c1, t));
    }
}

static void gfx_fill_round_grad(int x, int y, int w, int h, int r,
                                uint16_t c0, uint16_t c1)
{
  int j0, j1;
  if (w <= 0 || h <= 0 || !gfx_rows(y, h, &j0, &j1)) return;
  if (r < 0) r = 0;
  if (r > w / 2) r = w / 2;
  if (r > h / 2) r = h / 2;
  for (int j = j0; j <= j1; j++)
    {
      int inset = gfx_round_inset(j, h, r);
      int t = (h > 1) ? (j * 32 / (h - 1)) : 0;
      gfx_hline(x+inset, y+j, w-2*inset, gfx_blend565(c0, c1, t));
    }
}

static void gfx_glyph(int x, int y, char ch, uint16_t fg)
{
  unsigned char u = (unsigned char)ch;
  int j0, j1;
  if (u < GFX_FONT_FIRST || u > GFX_FONT_LAST) return;
  if (!gfx_rows(y, GFX_FONT_H, &j0, &j1)) return;
  FAR const uint8_t *g = &g_font8x16[(u - GFX_FONT_FIRST) * GFX_FONT_H];
  for (int row = j0; row <= j1; row++)
    {
      uint8_t bits = g[row];
      if (!bits) continue;
      FAR uint16_t *line = gfx_rowp(y + row);
      for (int col = 0; col < GFX_FONT_W; col++)
        if (bits & (0x80 >> col))
          {
            int px = x + col;
            if ((unsigned)px < LCD_W) line[px] = fg;
          }
    }
}

static void gtext(int x, int y, FAR const char *s, uint16_t fg)
{
  if (!gfx_band_hits(y, GFX_FONT_H)) return;      /* cheap reject */
  for (; *s && x < LCD_W; s++) { gfx_glyph(x, y, *s, fg); x += GFX_FONT_W; }
}

static void gtext_center(int cx, int y, FAR const char *s, uint16_t fg)
{
  gtext(cx - (int)strlen(s) * GFX_FONT_W / 2, y, s, fg);
}

static void gtext_right(int rx, int y, FAR const char *s, uint16_t fg)
{
  gtext(rx - (int)strlen(s) * GFX_FONT_W, y, s, fg);
}

/****************************************************************************
 * Boot screen
 ****************************************************************************/

#define BOOT_LINES (ROWS - 1)

static char          g_boot_log[BOOT_LINES][COLS + 1];
static int           g_boot_n;
static volatile bool g_boot_screen = true;   /* until the menu takes over */

static void compose_boot(void)
{
  gfx_fill_rect(0, 0, LCD_W, BAR_H, COL_SEL_BG);
  gtext(4, 0, "NuttX boot", COL_SEL_FG);
  for (int i = 0; i < g_boot_n; i++)
    gtext(0, (i + 1) * FONT_H, g_boot_log[i],
          i == g_boot_n - 1 ? COL_SEL_FG : COL_FG);
}

static void boot_step(FAR const char *fmt, ...)
{
  char line[COLS + 1];
  va_list ap;
  size_t n;

  if (!g_boot_screen) return;

  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  n = strlen(line);
  while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r'))
    line[--n] = '\0';

  nxmutex_lock(&g_con_mutex);
  if (g_boot_screen)
    {
      if (g_boot_n == BOOT_LINES)
        {
          memmove(g_boot_log[0], g_boot_log[1],
                  sizeof(g_boot_log[0]) * (BOOT_LINES - 1));
          g_boot_n--;
        }
      snprintf(g_boot_log[g_boot_n++], sizeof(g_boot_log[0]), "%s", line);
      gfx_render_all(compose_boot);          /* no-op until the LCD is up */
    }
  nxmutex_unlock(&g_con_mutex);
}

/* The menu is about to be drawn: stop drawing boot steps. */

static void boot_screen_end(void)
{
  nxmutex_lock(&g_con_mutex);
  g_boot_screen = false;
  nxmutex_unlock(&g_con_mutex);
}

/****************************************************************************
 * Bit-banged LCD
 *
 *   Drives the ST7789 through plain GPIOs, independent of NuttX's SPI and
 *   LCD drivers and of the OS itself.  Used before the OS starts (to prove
 *   that NuttX runs at all) and from the panic handler.  Each use takes the
 *   pins away from SPI2; board_lcd_initialize() takes them back.
 ****************************************************************************/

#define PIN_LCD_RST   33
#define PIN_LCD_DC    34
#define PIN_LCD_MOSI  35
#define PIN_LCD_SCLK  36
#define PIN_LCD_CS    37
#define PIN_LCD_BL    38

#ifndef SIG_GPIO_OUT_IDX
#  define SIG_GPIO_OUT_IDX 256               /* ESP32-S3 "simple GPIO" */
#endif

#if defined(CONFIG_LCD_LANDSCAPE)
#  define BB_MADCTL 0x70
#  define BB_XOFF   CONFIG_LCD_ST7789_YOFFSET /* the driver swaps them too */
#  define BB_YOFF   CONFIG_LCD_ST7789_XOFFSET
#elif defined(CONFIG_LCD_RLANDSCAPE)
#  define BB_MADCTL 0xa0
#  define BB_XOFF   CONFIG_LCD_ST7789_YOFFSET
#  define BB_YOFF   CONFIG_LCD_ST7789_XOFFSET
#else
#  define BB_MADCTL 0x70
#  define BB_XOFF   40
#  define BB_YOFF   53
#endif

static void bb_byte(uint8_t b)
{
  for (int i = 0; i < 8; i++, b <<= 1)
    {
      esp_gpiowrite(PIN_LCD_MOSI, (b & 0x80) != 0);
      esp_gpiowrite(PIN_LCD_SCLK, 1);
      esp_gpiowrite(PIN_LCD_SCLK, 0);
    }
}

static void bb_cmd(uint8_t cmd, FAR const uint8_t *data, int n)
{
  esp_gpiowrite(PIN_LCD_CS, 0);
  esp_gpiowrite(PIN_LCD_DC, 0);
  bb_byte(cmd);
  esp_gpiowrite(PIN_LCD_DC, 1);
  for (int i = 0; i < n; i++) bb_byte(data[i]);
  esp_gpiowrite(PIN_LCD_CS, 1);
}

/* Grab the LCD pins and (re)initialise the panel.  hw_reset pulses RST,
 * which is safe before the OS runs; in a panic we keep the panel state.
 */

static void bb_lcd_begin(bool hw_reset)
{
  static const uint8_t pins[] =
  {
    PIN_LCD_RST, PIN_LCD_DC, PIN_LCD_MOSI, PIN_LCD_SCLK, PIN_LCD_CS,
    PIN_LCD_BL
  };
  uint8_t v;

  for (size_t i = 0; i < sizeof(pins); i++)
    {
      esp_configgpio(pins[i], OUTPUT);
      esp_gpio_matrix_out(pins[i], SIG_GPIO_OUT_IDX, false, false);
    }

  esp_gpiowrite(PIN_LCD_CS, 1);
  esp_gpiowrite(PIN_LCD_SCLK, 0);
  esp_gpiowrite(PIN_LCD_RST, 1);
  esp_gpiowrite(PIN_LCD_BL, 1);
  if (hw_reset)
    {
      esp_gpiowrite(PIN_LCD_RST, 0); up_mdelay(10);
      esp_gpiowrite(PIN_LCD_RST, 1); up_mdelay(120);
    }

  bb_cmd(0x11, NULL, 0); up_mdelay(120);    /* SLPOUT */
  v = 0x55;      bb_cmd(0x3a, &v, 1);       /* COLMOD 16 bpp */
  v = BB_MADCTL; bb_cmd(0x36, &v, 1);       /* MADCTL, same as the driver */
#ifdef CONFIG_LCD_ST7789_INVCOLOR
  bb_cmd(0x21, NULL, 0);                    /* INVON */
#else
  bb_cmd(0x20, NULL, 0);                    /* INVOFF */
#endif
  bb_cmd(0x13, NULL, 0);                    /* NORON */
  bb_cmd(0x29, NULL, 0);                    /* DISPON */
}

/* Render a whole frame with `fn` through the band buffer, bit-banged. */

static void bb_frame(compose_t fn)
{
  const uint8_t ca[4] = { (BB_XOFF) >> 8, (BB_XOFF) & 0xff,
                          (BB_XOFF + LCD_W - 1) >> 8,
                          (BB_XOFF + LCD_W - 1) & 0xff };
  const uint8_t ra[4] = { (BB_YOFF) >> 8, (BB_YOFF) & 0xff,
                          (BB_YOFF + LCD_H - 1) >> 8,
                          (BB_YOFF + LCD_H - 1) & 0xff };

  bb_cmd(0x2a, ca, 4);                      /* CASET */
  bb_cmd(0x2b, ra, 4);                      /* RASET */

  esp_gpiowrite(PIN_LCD_CS, 0);
  esp_gpiowrite(PIN_LCD_DC, 0);
  bb_byte(0x2c);                            /* RAMWR */
  esp_gpiowrite(PIN_LCD_DC, 1);

  for (int y = 0; y < LCD_H; y += BAND_H)
    {
      int h = LCD_H - y < BAND_H ? LCD_H - y : BAND_H;

      g_by0 = y;
      g_by1 = y + h - 1;
      memset(g_band, 0, sizeof(g_band));
      fn();
      for (int i = 0; i < h * LCD_W; i++)
        {
          bb_byte((uint8_t)(g_band[i] >> 8));
          bb_byte((uint8_t)g_band[i]);
        }
    }

  esp_gpiowrite(PIN_LCD_CS, 1);
  g_by0 = 0;
  g_by1 = -1;
}

/* 8x8 text: the 8x16 font squashed by OR-ing row pairs (still readable),
 * 30 columns x 16 lines.
 */

static void gtext_small(int x, int y, FAR const char *s, uint16_t fg)
{
  if (!gfx_band_hits(y, 8)) return;
  for (; *s && x < LCD_W; s++, x += 8)
    {
      unsigned char u = (unsigned char)*s;
      if (u < GFX_FONT_FIRST || u > GFX_FONT_LAST) continue;
      FAR const uint8_t *g = &g_font8x16[(u - GFX_FONT_FIRST) * GFX_FONT_H];
      for (int r = 0; r < 8; r++)
        {
          int py = y + r;
          uint8_t bits = (uint8_t)(g[2 * r] | g[2 * r + 1]);
          if (!bits || py < g_by0 || py > g_by1) continue;
          FAR uint16_t *line = gfx_rowp(py);
          for (int c = 0; c < 8; c++)
            if ((bits & (0x80 >> c)) && (unsigned)(x + c) < LCD_W)
              line[x + c] = fg;
        }
    }
}

/* Very first thing NuttX does (called from esp32s3_board_initialize(),
 * before the OS starts): light the backlight and draw a frame.  If this
 * screen does not appear, NuttX never got control from the bootloader.
 */

static void compose_early(void)
{
  gfx_clear(0x001f);                        /* blue */
  gtext(8, 40, "NuttX started", 0xffff);
  gtext(8, 64, "(early init)", 0xffff);
}

void cardputer_early_init(void)
{
#ifdef CONFIG_DEBUG_FEATURES
  bb_lcd_begin(true);
  bb_frame(compose_early);
#else
  esp_configgpio(PIN_LCD_BL, OUTPUT);
  esp_gpio_matrix_out(PIN_LCD_BL, SIG_GPIO_OUT_IDX, false, false);
  esp_gpiowrite(PIN_LCD_BL, 1);
#endif
}

/****************************************************************************
 * Panic screen (debug builds)
 *
 *   The last syslog lines are kept in a small ring; on a panic they are
 *   drawn in red together with the task, file:line and PC.
 ****************************************************************************/

#ifdef CARDPUTER_PANIC_SCREEN
#define TAIL_LINES 8
#define PANIC_LINES 16

static char g_tail[TAIL_LINES][COLS + 1];
static int  g_tail_head;                   /* line being written */
static int  g_tail_col;
static char g_panic_txt[PANIC_LINES][COLS + 1];
static int  g_panic_n;
static bool g_panic_drawn;

static void tail_putc(int c)
{
  if (c == '\r') return;
  if (c == '\n' || g_tail_col >= COLS)
    {
      g_tail_head = (g_tail_head + 1) % TAIL_LINES;
      g_tail_col = 0;
      g_tail[g_tail_head][0] = '\0';
      if (c == '\n') return;
    }
  g_tail[g_tail_head][g_tail_col++] = (char)c;
  g_tail[g_tail_head][g_tail_col] = '\0';
}

static void panic_line(FAR const char *fmt, ...)
  __attribute__((format(printf, 1, 2)));
static void panic_line(FAR const char *fmt, ...)
{
  va_list ap;
  if (g_panic_n >= PANIC_LINES) return;
  va_start(ap, fmt);
  vsnprintf(g_panic_txt[g_panic_n++], COLS + 1, fmt, ap);
  va_end(ap);
}

static void compose_panic(void)
{
  gfx_clear(0xa000);                        /* dark red */
  for (int i = 0; i < g_panic_n; i++)
    gtext_small(0, i * 8 + 2, g_panic_txt[i], i == 0 ? 0xffe0 : 0xffff);
}

static int panic_screen_cb(FAR struct notifier_block *nb,
                           unsigned long action, FAR void *data)
{
  FAR struct panic_notifier_s *p = data;
  FAR const char *f;

  UNUSED(nb);
  if (g_panic_drawn || action == PANIC_KERNEL_FINAL || p == NULL)
    return 0;
  g_panic_drawn = true;

  f = p->filename ? strrchr(p->filename, '/') : NULL;
  f = f ? f + 1 : (p->filename ? p->filename : "?");

  panic_line("*** CRASH *** photo this");
#if CONFIG_TASK_NAME_SIZE > 0
  panic_line("task %s", p->rtcb ? p->rtcb->name : "?");
#endif
  panic_line("%s:%d", f, p->linenum);
  if (p->msg) panic_line("%s", p->msg);
  if (p->regs)
    {
      FAR uint32_t *r = p->regs;
      panic_line("PC %08" PRIx32 " A0 %08" PRIx32, r[REG_PC], r[REG_A0]);
      panic_line("EXC %" PRIu32 " VADDR %08" PRIx32,
                 r[REG_EXCCAUSE], r[REG_EXCVADDR]);
    }
  if (g_boot_n > 0)
    panic_line("step: %s", g_boot_log[g_boot_n - 1]);
  panic_line("--- last log ---");
  for (int i = 1; i <= TAIL_LINES; i++)
    {
      FAR const char *t = g_tail[(g_tail_head + i) % TAIL_LINES];
      if (t[0]) panic_line("%s", t);
    }

  bb_lcd_begin(false);
  bb_frame(compose_panic);
  return 0;
}

static struct notifier_block g_panic_nb =
{
  .notifier_call = panic_screen_cb,
};
#endif /* CARDPUTER_PANIC_SCREEN */

/****************************************************************************
 * Vector app icons
 ****************************************************************************/

static void icon_screen(int x, int y, uint16_t a)
{
  gfx_round(x, y+1, 15, 10, 2, a);
  gfx_fill_rect(x+5, y+11, 5, 2, a);
  gfx_fill_rect(x+2, y+13, 11, 1, a);
}
static void icon_keyboard(int x, int y, uint16_t a)
{
  gfx_round(x, y+2, 15, 11, 2, a);
  for (int r=0;r<2;r++) for (int c=0;c<4;c++)
    gfx_fill_rect(x+3+c*3, y+5+r*3, 1, 1, a);
}
static void icon_terminal(int x, int y, uint16_t a)
{
  gfx_round(x, y+1, 15, 13, 2, a);
  gfx_line(x+4, y+5, x+7, y+7, a);
  gfx_line(x+7, y+7, x+4, y+9, a);
  gfx_fill_rect(x+8, y+9, 4, 1, a);
}
static void icon_files(int x, int y, uint16_t a)
{
  gfx_fill_round(x, y+2, 6, 2, 1, a);
  gfx_round(x, y+3, 15, 10, 2, a);
}
static void icon_battery(int x, int y, uint16_t a)
{
  gfx_round(x, y+3, 13, 8, 2, a);
  gfx_fill_rect(x+13, y+5, 2, 4, a);
  gfx_fill_rect(x+2, y+5, 6, 4, a);
}
static void icon_clock(int x, int y, uint16_t a)
{
  gfx_round(x, y+1, 14, 14, 7, a);
  gfx_line(x+7, y+8, x+7, y+4, a);
  gfx_line(x+7, y+8, x+10, y+10, a);
}

/* 2x scaled glyph (16 x 32) for the big clock digits */

static void gfx_glyph2x(int x, int y, char ch, uint16_t fg)
{
  unsigned char u = (unsigned char)ch;
  if (u < GFX_FONT_FIRST || u > GFX_FONT_LAST) return;
  if (!gfx_band_hits(y, GFX_FONT_H * 2)) return;
  FAR const uint8_t *g = &g_font8x16[(u - GFX_FONT_FIRST) * GFX_FONT_H];
  for (int row = 0; row < GFX_FONT_H; row++)
    {
      int py = y + row * 2;
      uint8_t bits = g[row];
      if (!bits || py > g_by1 || py + 1 < g_by0) continue;
      for (int col = 0; col < GFX_FONT_W; col++)
        if (bits & (0x80 >> col)) gfx_fill_rect(x + col * 2, py, 2, 2, fg);
    }
}

static void gtext2x_center(int cx, int y, FAR const char *s, uint16_t fg)
{
  int x = cx - (int)strlen(s) * GFX_FONT_W;
  for (; *s; s++, x += GFX_FONT_W * 2) gfx_glyph2x(x, y, *s, fg);
}

/* Wi-Fi fan: a dot plus `bars` arcs (0..3) opening upward. */

static void gfx_wifi_glyph(int cx, int cy, int bars, uint16_t on,
                           uint16_t off)
{
  gfx_fill_circle(cx, cy, 1, on);
  for (int i = 1; i <= 3; i++)
    {
      int r = 1 + i * 3;
      uint16_t c = (i <= bars) ? on : off;
      for (int dx = -r; dx <= r; dx++)
        {
          if (dx * dx * 2 > r * r) continue;          /* ±45° fan */
          int dy = isqrt32(r * r - dx * dx);
          gfx_px(cx + dx, cy - dy, c);
          gfx_px(cx + dx, cy - dy + 1, c);
        }
    }
}

#ifdef CARDPUTER_WIFI
static void icon_wifi(int x, int y, uint16_t a)
{
  gfx_wifi_glyph(x + 7, y + 12, 3, a, a);
}
#endif

/****************************************************************************
 * Ring-buffer character device
 ****************************************************************************/

struct ringdev_s
{
  FAR char      *buf;
  uint16_t       size;
  volatile int   head;
  volatile int   tail;
  sem_t          data_sem;
  sem_t          space_sem;
  FAR struct pollfd *fds_pollin;
};

static char             g_kbd_buf[KBD_RING_SIZE];
static char             g_out_buf[OUT_RING_SIZE];
static struct ringdev_s g_kbd_ring;
static struct ringdev_s g_out_ring;
static volatile bool    g_out_ring_ready = false;

static void ringdev_init(FAR struct ringdev_s *r, FAR char *buf,
                         uint16_t size)
{
  r->buf = buf; r->size = size;
  r->head = 0; r->tail = 0; r->fds_pollin = NULL;
  nxsem_init(&r->data_sem,  0, 0);
  nxsem_init(&r->space_sem, 0, size);
}

static inline void ringdev_store(FAR struct ringdev_s *r, char c)
{
  r->buf[r->head] = c;
  r->head = (r->head + 1) % r->size;
  nxsem_post(&r->data_sem);
  if (r->fds_pollin != NULL) poll_notify(&r->fds_pollin, 1, POLLIN);
}

static inline char ringdev_take(FAR struct ringdev_s *r)
{
  char c = r->buf[r->tail];
  r->tail = (r->tail + 1) % r->size;
  nxsem_post(&r->space_sem);
  return c;
}

static void ringdev_putc(FAR struct ringdev_s *r, char c)
{
  int ret;
  do { ret = nxsem_wait(&r->space_sem); } while (ret == -EINTR);
  ringdev_store(r, c);
}

static void ringdev_tryputc(FAR struct ringdev_s *r, char c)
{
  if (nxsem_trywait(&r->space_sem) < 0) return;
  ringdev_store(r, c);
}

static void ringdev_puts(FAR struct ringdev_s *r, FAR const char *s)
{
  for (; *s; s++) ringdev_putc(r, *s);
}

static char ringdev_getc(FAR struct ringdev_s *r)
{
  int ret;
  do { ret = nxsem_wait(&r->data_sem); } while (ret == -EINTR);
  return ringdev_take(r);
}

static int ringdev_open (FAR struct file *f) { UNUSED(f); return OK; }
static int ringdev_close(FAR struct file *f) { UNUSED(f); return OK; }

static ssize_t ringdev_read(FAR struct file *filep, FAR char *buf, size_t len)
{
  FAR struct ringdev_s *r = filep->f_inode->i_private;
  int ret;
  if (len == 0) return 0;
  if (filep->f_oflags & O_NONBLOCK)
    {
      if (nxsem_trywait(&r->data_sem) < 0) return -EAGAIN;
    }
  else
    {
      do { ret = nxsem_wait(&r->data_sem); } while (ret == -EINTR);
      if (ret < 0) return (ssize_t)ret;
    }
  buf[0] = ringdev_take(r);
  ssize_t n = 1;
  while (n < (ssize_t)len && nxsem_trywait(&r->data_sem) >= 0)
    buf[n++] = ringdev_take(r);
  return n;
}

static ssize_t ringdev_kbd_write(FAR struct file *filep,
                                 FAR const char *buf, size_t len)
{
  UNUSED(filep);
  for (size_t i = 0; i < len; i++) ringdev_tryputc(&g_out_ring, buf[i]);
  return (ssize_t)len;
}

static ssize_t ringdev_write(FAR struct file *filep,
                             FAR const char *buf, size_t len)
{
  FAR struct ringdev_s *r = filep->f_inode->i_private;
  for (size_t i = 0; i < len; i++) ringdev_putc(r, buf[i]);
  return (ssize_t)len;
}

static ssize_t ringdev_out_read(FAR struct file *f, FAR char *b, size_t l)
{ UNUSED(f); UNUSED(b); UNUSED(l); return -ENOSYS; }

static int ringdev_kbd_poll(FAR struct file *filep,
                            FAR struct pollfd *fds, bool setup)
{
  FAR struct ringdev_s *r = filep->f_inode->i_private;
  int val = 0;
  if (setup)
    {
      nxsem_get_value(&r->data_sem, &val);
      if (val > 0) poll_notify(&fds, 1, POLLIN);
      else r->fds_pollin = fds;
    }
  else { if (r->fds_pollin == fds) r->fds_pollin = NULL; }
  return OK;
}

static int ringdev_out_poll(FAR struct file *filep,
                            FAR struct pollfd *fds, bool setup)
{
  UNUSED(filep);
  if (setup) poll_notify(&fds, 1, POLLOUT);
  return OK;
}

static struct termios g_kbd_termios =
{
  .c_iflag = ICRNL,
  .c_oflag = OPOST | ONLCR,
  .c_cflag = CS8 | CREAD | B115200,
  .c_lflag = 0,
  .c_cc    = { [VMIN]=1, [VTIME]=0, [VERASE]='\x7f' }
};

static int ringdev_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  UNUSED(filep);
  if (cmd == TCGETS)
    {
      FAR struct termios *t = (FAR struct termios *)(uintptr_t)arg;
      if (!t) return -EINVAL;
      memcpy(t, &g_kbd_termios, sizeof(*t)); return OK;
    }
  if (cmd == TCSETS || cmd == TCSETSW || cmd == TCSETSF)
    {
      FAR struct termios *t = (FAR struct termios *)(uintptr_t)arg;
      if (!t) return -EINVAL;
      memcpy(&g_kbd_termios, t, sizeof(*t)); return OK;
    }
  if (cmd == TIOCGWINSZ)
    {
      FAR struct winsize *w = (FAR struct winsize *)(uintptr_t)arg;
      if (!w) return -EINVAL;
      w->ws_col = COLS; w->ws_row = TERM_ROWS; return OK;
    }
  return -ENOTTY;
}

static const struct file_operations g_kbd_ops =
{
  .open  = ringdev_open,
  .close = ringdev_close,
  .read  = ringdev_read,
  .write = ringdev_kbd_write,
  .ioctl = ringdev_ioctl,
  .poll  = ringdev_kbd_poll,
};

static const struct file_operations g_out_ops =
{
  .open  = ringdev_open,
  .close = ringdev_close,
  .read  = ringdev_out_read,
  .write = ringdev_write,
  .ioctl = ringdev_ioctl,
  .poll  = ringdev_out_poll,
};

/****************************************************************************
 * Syslog → LCD
 ****************************************************************************/

/* Debug builds: mirror syslog -- including panic/assert dumps, which use
 * the polled "force" path -- to the USB cable (USB-Serial-JTAG), so a crash
 * can be read on a PC with any serial terminal even when the console UART
 * (GPIO43/44) is not wired out.  Never blocks: if no host reads the port
 * the output is dropped.
 */

#if defined(CONFIG_SYSLOG) && defined(CONFIG_DEBUG_FEATURES)
#  define CARDPUTER_USB_SYSLOG 1
#  define USJ_EP1_REG       0x60038000u
#  define USJ_EP1_CONF_REG  0x60038004u
#  define USJ_WR_DONE       (1u << 0)
#  define USJ_IN_FREE       (1u << 1)

static volatile bool g_usj_dead;

static int usj_syslog_putc(FAR struct syslog_channel_s *ch, int c)
{
  UNUSED(ch);
#ifdef CARDPUTER_PANIC_SCREEN
  tail_putc(c);
#endif
  if (g_usj_dead) return c;
  for (int i = 0; i < 200000; i++)
    {
      if (*(volatile uint32_t *)USJ_EP1_CONF_REG & USJ_IN_FREE)
        {
          *(volatile uint32_t *)USJ_EP1_REG = (uint32_t)(c & 0xff);
          *(volatile uint32_t *)USJ_EP1_CONF_REG = USJ_WR_DONE;
          return c;
        }
    }
  g_usj_dead = true;                         /* nobody listening */
  return c;
}

static int usj_syslog_flush(FAR struct syslog_channel_s *ch)
{
  UNUSED(ch);
  *(volatile uint32_t *)USJ_EP1_CONF_REG = USJ_WR_DONE;
  return OK;
}

static const struct syslog_channel_ops_s g_usj_syslog_ops =
{
  .sc_putc  = usj_syslog_putc,
  .sc_force = usj_syslog_putc,
  .sc_flush = usj_syslog_flush,
};
static struct syslog_channel_s g_usj_syslog_channel =
{
  .sc_ops = &g_usj_syslog_ops,
};
#endif

#ifdef CONFIG_SYSLOG
static int lcd_syslog_putc(FAR struct syslog_channel_s *ch, int c)
{
  UNUSED(ch);
  if (g_out_ring_ready) ringdev_tryputc(&g_out_ring, (char)c);
  return c;
}
static int lcd_syslog_flush(FAR struct syslog_channel_s *ch)
{ UNUSED(ch); return OK; }

static const struct syslog_channel_ops_s g_lcd_syslog_ops =
{
  .sc_putc  = lcd_syslog_putc,
  .sc_force = lcd_syslog_putc,
  .sc_flush = lcd_syslog_flush,
};
static struct syslog_channel_s g_lcd_syslog_channel =
{
  .sc_ops = &g_lcd_syslog_ops,
};
#endif

/****************************************************************************
 * Key map
 ****************************************************************************/

struct KeyValue_t { char value_first; char value_second; };

static const struct KeyValue_t g_key_map[4][14] =
{
  { {'`','~'},{'1','!'},{'2','@'},{'3','#'},{'4','$'},{'5','%'},
    {'6','^'},{'7','&'},{'8','*'},{'9','('},{'0',')'},
    {'-','_'},{'=','+'},{ KEY_BACKSPACE,KEY_BACKSPACE } },
  { {KEY_TAB,KEY_TAB},{'q','Q'},{'w','W'},{'e','E'},{'r','R'},
    {'t','T'},{'y','Y'},{'u','U'},{'i','I'},{'o','O'},{'p','P'},
    {'[','{'},{']','}'},{'\\','|'} },
  { {(char)KEY_FN,(char)KEY_FN},{(char)KEY_LEFT_SHIFT,(char)KEY_LEFT_SHIFT},
    {'a','A'},{'s','S'},{'d','D'},{'f','F'},{'g','G'},
    {'h','H'},{'j','J'},{'k','K'},{'l','L'},
    {';',':'},{'\'','\"'},{KEY_ENTER,KEY_ENTER} },
  { {(char)KEY_LEFT_CTRL,(char)KEY_LEFT_CTRL},{(char)KEY_OPT,(char)KEY_OPT},
    {(char)KEY_LEFT_ALT,(char)KEY_LEFT_ALT},{'z','Z'},{'x','X'},{'c','C'},
    {'v','V'},{'b','B'},{'n','N'},{'m','M'},{',','<'},{'.','>'},{'/','?'},
    {' ',' '} }
};

/****************************************************************************
 * Soft I2C + TCA8418
 *
 *   Only the kbd thread touches these pins, so no lock is needed.  The old
 *   version wrapped every transaction in enter_critical_section(), which
 *   kept interrupts off for ~300 µs every 20 ms — bad for the Wi-Fi MAC.
 *   I2C is fully synchronous (the master owns SCL), so being preempted
 *   mid-transfer only stretches the clock; the TCA8418 doesn't care.
 ****************************************************************************/

static void soft_i2c_delay(void) { up_udelay(4); }

static void soft_i2c_start(void)
{
  esp_configgpio(PIN_SDA,OUTPUT); esp_gpiowrite(PIN_SDA,1);
  esp_configgpio(PIN_SCL,OUTPUT); esp_gpiowrite(PIN_SCL,1);
  soft_i2c_delay(); esp_gpiowrite(PIN_SDA,0);
  soft_i2c_delay(); esp_gpiowrite(PIN_SCL,0);
}
static void soft_i2c_stop(void)
{
  esp_configgpio(PIN_SDA,OUTPUT); esp_gpiowrite(PIN_SDA,0);
  esp_configgpio(PIN_SCL,OUTPUT); esp_gpiowrite(PIN_SCL,1);
  soft_i2c_delay(); esp_gpiowrite(PIN_SDA,1);
  soft_i2c_delay();
}
static bool soft_i2c_write_byte(uint8_t byte)
{
  esp_configgpio(PIN_SDA,OUTPUT);
  for (int i=0;i<8;i++)
    {
      esp_gpiowrite(PIN_SDA,(byte&0x80)?1:0); byte<<=1;
      soft_i2c_delay(); esp_gpiowrite(PIN_SCL,1);
      soft_i2c_delay(); esp_gpiowrite(PIN_SCL,0);
    }
  esp_configgpio(PIN_SDA,INPUT|PULLUP);
  soft_i2c_delay(); esp_gpiowrite(PIN_SCL,1); soft_i2c_delay();
  bool ack=(esp_gpioread(PIN_SDA)==0);
  esp_gpiowrite(PIN_SCL,0);
  return ack;
}
static uint8_t soft_i2c_read_byte(bool ack)
{
  uint8_t byte=0;
  esp_configgpio(PIN_SDA,INPUT|PULLUP);
  for (int i=0;i<8;i++)
    {
      byte<<=1;
      soft_i2c_delay(); esp_gpiowrite(PIN_SCL,1); soft_i2c_delay();
      if (esp_gpioread(PIN_SDA)) byte|=1;
      esp_gpiowrite(PIN_SCL,0);
    }
  esp_configgpio(PIN_SDA,OUTPUT); esp_gpiowrite(PIN_SDA,ack?0:1);
  soft_i2c_delay(); esp_gpiowrite(PIN_SCL,1);
  soft_i2c_delay(); esp_gpiowrite(PIN_SCL,0);
  return byte;
}
/* Free a bus left mid-transfer (e.g. by the firmware that rebooted into
 * us): clock SCL until the slave releases SDA, then send a STOP.
 */

static void soft_i2c_recover(void)
{
  esp_configgpio(PIN_SDA,INPUT|PULLUP);
  esp_configgpio(PIN_SCL,OUTPUT); esp_gpiowrite(PIN_SCL,1);
  soft_i2c_delay();
  for (int i=0;i<9;i++)
    {
      esp_gpiowrite(PIN_SCL,0); soft_i2c_delay();
      esp_gpiowrite(PIN_SCL,1); soft_i2c_delay();
    }
  soft_i2c_stop();
}
static bool soft_tca8418_write(uint8_t reg, uint8_t val)
{
  bool ok;
  soft_i2c_start();
  ok = soft_i2c_write_byte(TCA8418_ADDR<<1) &&
       soft_i2c_write_byte(reg) &&
       soft_i2c_write_byte(val);
  soft_i2c_stop();
  return ok;
}
static bool soft_tca8418_read(uint8_t reg, FAR uint8_t *val)
{
  soft_i2c_start();
  if (!soft_i2c_write_byte(TCA8418_ADDR<<1) || !soft_i2c_write_byte(reg))
    { soft_i2c_stop(); return false; }
  soft_i2c_start();
  if (!soft_i2c_write_byte((TCA8418_ADDR<<1)|1))
    { soft_i2c_stop(); return false; }
  *val=soft_i2c_read_byte(false);
  soft_i2c_stop();
  return true;
}

/****************************************************************************
 * UI state
 ****************************************************************************/

enum ui_mode_e
{
  UI_MODE_MENU = 0,
  UI_MODE_TERMINAL,
  UI_MODE_SCREENTEST,
  UI_MODE_KEYBOARDTEST,
  UI_MODE_FILEMANAGER,
  UI_MODE_BATTERY,
  UI_MODE_WIFI,
  UI_MODE_CLOCK,
  UI_MODE_COUNT
};

static const char * const g_mode_title[UI_MODE_COUNT] =
{
  "Cardputer", "Terminal", "ScreenTest", "Keyboard", "Files", "Battery",
  "Wi-Fi", "Clock"
};

static volatile int g_ui_mode = UI_MODE_MENU;

/* board_wlan_init() result, shown by the Wi-Fi app (1 = never called) */

#if defined(CONFIG_ESPRESSIF_WIFI) || defined(CONFIG_ESP32S3_WIFI)
static int g_wlan_init_ret = 1;
#endif
enum { NAV_NONE=0, NAV_UP, NAV_DOWN, NAV_LEFT, NAV_RIGHT };

struct menu_item_s
{
  void (*icon)(int, int, uint16_t);
  FAR const char *label;
  uint8_t mode;
};

static const struct menu_item_s g_menu_items[] =
{
  { icon_screen,   "ScreenTest", UI_MODE_SCREENTEST   },
  { icon_keyboard, "Keyboard",   UI_MODE_KEYBOARDTEST },
  { icon_terminal, "Terminal",   UI_MODE_TERMINAL     },
  { icon_files,    "Files",      UI_MODE_FILEMANAGER  },
  { icon_battery,  "Battery",    UI_MODE_BATTERY      },
#ifdef CARDPUTER_WIFI
  { icon_wifi,     "Wi-Fi",      UI_MODE_WIFI         },
#endif
  { icon_clock,    "World Clock", UI_MODE_CLOCK       },
};
#define MENU_COUNT ((int)(sizeof(g_menu_items)/sizeof(g_menu_items[0])))
static int g_menu_sel = 0;

/* ScreenTest */
static const struct { FAR const char *name; uint16_t color; } g_st_entries[] =
{
  {"WHITE",RGB_WHITE},{"RED",RGB_RED},{"GREEN",RGB_GREEN},
  {"BLUE",RGB_BLUE},{"YELLOW",RGB_YELLOW},{"CYAN",RGB_CYAN},
  {"MAGENTA",RGB_MAGENTA},
};
#define ST_COLOR_COUNT ((int)(sizeof(g_st_entries)/sizeof(g_st_entries[0])))
static int g_st_color_idx=0;

/****************************************************************************
 * Battery (ADC read into a cache; drawing never touches the ADC)
 ****************************************************************************/

#define BAT_ADC_DEVPATH "/dev/adc0"
#define BAT_ADC_CHANNEL 9
#define BAT_DIVIDER     2

struct bat_read_s { int mv,err,nsamp,ch0,raw0; };
static struct bat_read_s g_bat = { -1, 0, 0, -1, 0 };

static void battery_read(FAR struct bat_read_s *o)
{
  o->mv=-1; o->err=0; o->nsamp=0; o->ch0=-1; o->raw0=0;
  int fd=open(BAT_ADC_DEVPATH,O_RDONLY);
  if (fd<0) { o->err=errno; return; }
  if (ioctl(fd,ANIOC_TRIGGER,0)<0) { o->err=errno; close(fd); return; }
  struct adc_msg_s s[8];
  ssize_t n=read(fd,s,sizeof(s));
  if (n<0) { o->err=errno; close(fd); return; }
  int cnt=(int)(n/(ssize_t)sizeof(struct adc_msg_s));
  o->nsamp=cnt;
  if (cnt>0) { o->ch0=s[0].am_channel; o->raw0=(int)s[0].am_data; }
  for (int i=0;i<cnt;i++)
    if (s[i].am_channel==BAT_ADC_CHANNEL)
      { o->mv=(int)s[i].am_data*BAT_DIVIDER; break; }
  if (o->mv<0 && cnt>0) o->mv=(int)s[0].am_data*BAT_DIVIDER;
  close(fd);
}

static void battery_refresh(void)
{
  struct bat_read_s r;
  battery_read(&r);
  g_bat = r;
}

static int battery_percent(int mv)
{
  static const struct { int16_t mv; int8_t pct; } lut[]=
  { {4060,100},{3930,92},{3880,85},{3790,70},{3690,55},
    {3600,40},{3500,28},{3400,15},{3310,5},{3170,0} };
  const int n=(int)(sizeof(lut)/sizeof(lut[0]));
  if (mv>=lut[0].mv)     return 100;
  if (mv<=lut[n-1].mv)   return 0;
  for (int i=0;i<n-1;i++)
    if (mv<=lut[i].mv && mv>=lut[i+1].mv)
      return lut[i+1].pct+(mv-lut[i+1].mv)*(lut[i].pct-lut[i+1].pct)/(lut[i].mv-lut[i+1].mv);
  return 0;
}

static inline uint16_t battery_color(int pct)
{
  return (pct<=20)?COL_ERR:(pct<=50)?COL_WARN:COL_OK;
}

/****************************************************************************
 * World time core  (city table lives in flash; ~6 bytes of RAM state)
 *
 *   Fixed UTC offsets plus the four DST rules that cover the listed cities.
 *   All maths is done on UTC seconds, so it is independent of the NuttX
 *   TZ database (none is configured).
 ****************************************************************************/

enum { DST_NONE = 0, DST_EU, DST_US, DST_AU, DST_NZ };

struct city_s
{
  FAR const char *name;
  int32_t  std;            /* standard-time UTC offset, seconds */
  uint8_t  dst;            /* DST_* rule */
};

static const struct city_s g_cities[] =
{
  { "Taipei",       8 * 3600,           DST_NONE },
  { "Tokyo",        9 * 3600,           DST_NONE },
  { "Singapore",    8 * 3600,           DST_NONE },
  { "Bangkok",      7 * 3600,           DST_NONE },
  { "New Delhi",    5 * 3600 + 1800,    DST_NONE },
  { "Dubai",        4 * 3600,           DST_NONE },
  { "Moscow",       3 * 3600,           DST_NONE },
  { "Berlin",       1 * 3600,           DST_EU   },
  { "London",       0,                  DST_EU   },
  { "UTC",          0,                  DST_NONE },
  { "New York",    -5 * 3600,           DST_US   },
  { "Chicago",     -6 * 3600,           DST_US   },
  { "Los Angeles", -8 * 3600,           DST_US   },
  { "Honolulu",   -10 * 3600,           DST_NONE },
  { "Sydney",      10 * 3600,           DST_AU   },
  { "Auckland",    12 * 3600,           DST_NZ   },
};
#define CITY_COUNT ((int)(sizeof(g_cities) / sizeof(g_cities[0])))

static uint8_t g_home_city = 0;               /* Taipei */

/* Howard Hinnant's days_from_civil / civil_from_days (proleptic Gregorian) */

static int32_t days_from_civil(int y, int m, int d)
{
  y -= m <= 2;
  int32_t era = (y >= 0 ? y : y - 399) / 400;
  int yoe = y - era * 400;
  int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

static void civil_from_days(int32_t z, FAR int *y, FAR int *m, FAR int *d)
{
  z += 719468;
  int32_t era = (z >= 0 ? z : z - 146096) / 146097;
  int doe = (int)(z - era * 146097);
  int yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  int doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  int mp  = (5 * doy + 2) / 153;
  *d = doy - (153 * mp + 2) / 5 + 1;
  *m = mp < 10 ? mp + 3 : mp - 9;
  *y = yoe + era * 400 + (*m <= 2);
}

static int weekday_of(int32_t days)            /* 0 = Sunday */
{
  return (int)((days % 7 + 11) % 7);
}

static int last_mday(int y, int m)
{
  return (int)(days_from_civil(m == 12 ? y + 1 : y, m == 12 ? 1 : m + 1, 1) -
               days_from_civil(y, m, 1));
}

/* Day of month of the n-th Sunday (n = 1..4) or the last one (n = 5). */

static int nth_sunday(int y, int m, int n)
{
  if (n < 5)
    {
      int wd = weekday_of(days_from_civil(y, m, 1));
      return 1 + (7 - wd) % 7 + (n - 1) * 7;
    }

  int ld = last_mday(y, m);
  return ld - weekday_of(days_from_civil(y, m, ld));
}

static int64_t utc_at(int y, int m, int d, int hh)
{
  return (int64_t)days_from_civil(y, m, d) * 86400 + hh * 3600;
}

static bool city_in_dst(FAR const struct city_s *c, int64_t t)
{
  int y, m, d;
  int64_t s, e;
  int32_t sd = c->std;

  if (c->dst == DST_NONE) return false;
  civil_from_days((int32_t)((t + sd) / 86400), &y, &m, &d);

  switch (c->dst)
    {
      case DST_EU:   /* last Sun Mar 01:00 UTC .. last Sun Oct 01:00 UTC */
        s = utc_at(y, 3, nth_sunday(y, 3, 5), 1);
        e = utc_at(y, 10, nth_sunday(y, 10, 5), 1);
        return t >= s && t < e;

      case DST_US:   /* 2nd Sun Mar 02:00 local .. 1st Sun Nov 02:00 local */
        s = utc_at(y, 3, nth_sunday(y, 3, 2), 2) - sd;
        e = utc_at(y, 11, nth_sunday(y, 11, 1), 2) - (sd + 3600);
        return t >= s && t < e;

      case DST_AU:   /* 1st Sun Oct 02:00 .. 1st Sun Apr 03:00 (south) */
        e = utc_at(y, 4, nth_sunday(y, 4, 1), 3) - (sd + 3600);
        s = utc_at(y, 10, nth_sunday(y, 10, 1), 2) - sd;
        return t < e || t >= s;

      case DST_NZ:   /* last Sun Sep 02:00 .. 1st Sun Apr 03:00 (south) */
        e = utc_at(y, 4, nth_sunday(y, 4, 1), 3) - (sd + 3600);
        s = utc_at(y, 9, nth_sunday(y, 9, 5), 2) - sd;
        return t < e || t >= s;

      default:
        return false;
    }
}

static int32_t city_offset(int idx, int64_t t)
{
  FAR const struct city_s *c = &g_cities[idx];
  return c->std + (city_in_dst(c, t) ? 3600 : 0);
}

struct ltime_s { int y, mo, d, h, mi, s, wday; int32_t days; };

static void city_localtime(int idx, int64_t t, FAR struct ltime_s *lt)
{
  int64_t l = t + city_offset(idx, t);
  int32_t days = (int32_t)(l >= 0 ? l / 86400 : (l - 86399) / 86400);
  int32_t sec  = (int32_t)(l - (int64_t)days * 86400);
  civil_from_days(days, &lt->y, &lt->mo, &lt->d);
  lt->days = days;
  lt->wday = weekday_of(days);
  lt->h = sec / 3600; lt->mi = (sec / 60) % 60; lt->s = sec % 60;
}

/* Current UTC seconds, or -1 if the clock has never been set (it starts at
 * CONFIG_START_YEAR after boot because there is no RTC).
 */

static int64_t utc_now(void)
{
  struct timespec ts;
  if (clock_gettime(CLOCK_REALTIME, &ts) < 0) return -1;
  return (ts.tv_sec >= 1704067200) ? (int64_t)ts.tv_sec : -1;   /* >= 2024 */
}

static FAR const char * const g_wday[7] =
{ "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

/****************************************************************************
 * Status bar
 ****************************************************************************/

static void statusbar_compose(void)
{
  if (g_by0 > BAR_H - 1) return;              /* band is below the bar */

  gfx_fill_rect(0, 0, LCD_W, BAR_H, COL_BAR_BG);
  gfx_hline(0, BAR_H-1, LCD_W, COL_SEL_BG);

  int mode = g_ui_mode;
  gtext(6, 0, g_mode_title[(mode >= 0 && mode < UI_MODE_COUNT) ? mode : 0],
        COL_ACCENT);

  int64_t now = utc_now();
  if (now >= 0 && mode != UI_MODE_CLOCK)
    {
      struct ltime_s lt;
      char tbuf[8];
      city_localtime(g_home_city, now, &lt);
      snprintf(tbuf, sizeof(tbuf), "%02d:%02d", lt.h, lt.mi);
      gtext(100, 0, tbuf, COL_FG);
    }

  int bx=LCD_W-30, by=3, bw=24, bh=11;
  int mv=g_bat.mv;
  gfx_round(bx,by,bw,bh,3,COL_DIM);
  gfx_fill_rect(bx+bw,by+3,2,5,COL_DIM);
  if (mv<0)
    {
      gtext_right(bx-4, 0, "N/A", COL_DIM);
    }
  else
    {
      int pct=battery_percent(mv);
      uint16_t col=battery_color(pct);
      int fillw=pct*(bw-6)/100;
      if (fillw>0) gfx_fill_round(bx+3,by+3,fillw,bh-6,1,col);
      char pbuf[16]; snprintf(pbuf,sizeof(pbuf),"%d%%",pct);
      gtext_right(bx-5, 0, pbuf, col);
    }
}

/****************************************************************************
 * Scrollback buffer  (characters only — 30 B per line)
 *
 *   g_sb[slot][col]   char   (slot = abs_line % SCROLLBACK_LINES)
 *   g_sb_total        absolute index of current (live) line
 *   g_sb_col          write column on the live line
 *   g_scroll_offset   0 = live view; N = scrolled back N lines
 ****************************************************************************/

#define SCROLLBACK_LINES  CARDPUTER_SCROLLBACK_LINES
#define SB_SLOT(l)        ((l) % SCROLLBACK_LINES)
#define SB_CH(l,c)        g_sb[SB_SLOT(l)][(c)]

static char     g_sb[SCROLLBACK_LINES][COLS];
static int      g_sb_total       = 0;
static int      g_sb_col         = 0;
static int      g_scroll_offset  = 0;

static inline void sb_clear_line(int abs_line)
{
  memset(g_sb[SB_SLOT(abs_line)], ' ', COLS);
}

/* Draw one view row (0-based inside the terminal area). */

static void sb_paint_row(int view_row)
{
  int y = (TOP_ROW + view_row) * FONT_H;
  if (!gfx_band_hits(y, FONT_H)) return;

  gfx_fill_rect(0, y, LCD_W, FONT_H, COL_BG);

  int abs_line = g_sb_total - g_scroll_offset - (TERM_ROWS - 1) + view_row;
  if (abs_line < 0 || abs_line > g_sb_total) return;

  FAR const char *line = g_sb[SB_SLOT(abs_line)];
  for (int c = 0; c < COLS; c++)
    if (line[c] != ' ') gfx_glyph(c * FONT_W, y, line[c], COL_FG);
}

static void term_rows_compose(void)
{
  for (int r = 0; r < TERM_ROWS; r++) sb_paint_row(r);
}

static void term_compose(void)
{
  statusbar_compose();
  if (g_scroll_offset > 0 && g_by0 < BAR_H)
    {
      gfx_fill_rect(LCD_W/2, 0, LCD_W/2, BAR_H, COL_BAR_BG);
      gfx_hline(LCD_W/2, BAR_H-1, LCD_W/2, COL_SEL_BG);
      char sbuf[16];
      snprintf(sbuf, sizeof(sbuf), "^%d", g_scroll_offset);
      gtext_right(LCD_W - 2, 0, sbuf, COL_WARN);
    }
  term_rows_compose();
}

/* Live line is always the bottom view row while g_scroll_offset == 0 */

#define LIVE_Y   ((TOP_ROW + TERM_ROWS - 1) * FONT_H)

static inline void term_flush_live_locked(void)
{
  gfx_render(term_rows_compose, LIVE_Y, LIVE_Y + FONT_H - 1);
}

static inline void term_flush_band_locked(void)
{
  gfx_render(term_rows_compose, TERM_Y0, TERM_Y1);
}

static void sb_newline_locked(void)
{
  g_sb_total++;
  g_sb_col = 0;
  sb_clear_line(g_sb_total);
}

static int sb_max_offset(void)
{
  int avail = g_sb_total;
  if (avail > SCROLLBACK_LINES - 1) avail = SCROLLBACK_LINES - 1;
  int m = avail - TERM_ROWS + 1;
  return (m < 0) ? 0 : m;
}

/* Full terminal repaint — scrollback view changes only. */

static void term_repaint_locked(void)
{
  gfx_render_all(term_compose);
}

/****************************************************************************
 * con_putc — hot path character output
 *
 *   printable / '\r' / '\b'  → 1 band  (16 px rows, one SPI transfer)
 *   '\n' / wrap              → terminal area (8 SPI transfers)
 *   While scrolled back the store is updated silently.
 ****************************************************************************/

static void con_putc(char c)
{
  nxmutex_lock(&g_con_mutex);

  if (c == '\r')
    {
      g_sb_col = 0;
    }
  else if (c == '\b')
    {
      if (g_sb_col > 0)
        {
          g_sb_col--;
          SB_CH(g_sb_total, g_sb_col) = ' ';
        }
      if (g_scroll_offset == 0) term_flush_live_locked();
    }
  else if (c == '\n')
    {
      sb_newline_locked();
      if (g_scroll_offset == 0) term_flush_band_locked();
    }
  else if (c >= 0x20 && (unsigned char)c < 0x80)
    {
      if (g_sb_col >= COLS)
        {
          sb_newline_locked();
          if (g_scroll_offset == 0) term_flush_band_locked();
        }

      SB_CH(g_sb_total, g_sb_col) = c;
      g_sb_col++;

      if (g_scroll_offset == 0) term_flush_live_locked();
    }

  nxmutex_unlock(&g_con_mutex);
}

/****************************************************************************
 * con_clear / con_clear_full
 ****************************************************************************/

static void sb_reset_locked(void)
{
  g_sb_total = 0; g_sb_col = 0; g_scroll_offset = 0;
  memset(g_sb, ' ', sizeof(g_sb));
}

static void con_clear(void)
{
  nxmutex_lock(&g_con_mutex);
  sb_reset_locked();
  term_flush_band_locked();
  nxmutex_unlock(&g_con_mutex);
}

static void con_clear_full(void)
{
  nxmutex_lock(&g_con_mutex);
  sb_reset_locked();
  gfx_render_all(compose_nothing);
  nxmutex_unlock(&g_con_mutex);
}

/****************************************************************************
 * Scrollback scroll helpers (kbd thread)
 ****************************************************************************/

static void term_scroll_up(void)
{
  nxmutex_lock(&g_con_mutex);
  if (g_scroll_offset < sb_max_offset())
    {
      g_scroll_offset++;
      term_repaint_locked();
    }
  nxmutex_unlock(&g_con_mutex);
}

static void term_scroll_down(void)
{
  nxmutex_lock(&g_con_mutex);
  if (g_scroll_offset > 0)
    {
      g_scroll_offset--;
      term_repaint_locked();
    }
  nxmutex_unlock(&g_con_mutex);
}

static bool term_snap_live(void)
{
  if (g_scroll_offset == 0) return false;
  nxmutex_lock(&g_con_mutex);
  g_scroll_offset = 0;
  term_repaint_locked();
  nxmutex_unlock(&g_con_mutex);
  return true;
}

/****************************************************************************
 * Shared background / footer helpers
 ****************************************************************************/

static void bg_compose(void)
{
  gfx_vgrad(0, 0, LCD_W, LCD_H, COL_BG, COL_BG2);
}

static void footer_compose(FAR const char *text)
{
  int y=119;
  if (!gfx_band_hits(y, FONT_H)) return;
  gfx_fill_circle(8, y+7, 2, COL_ACCENT);
  gtext(14, y, text, COL_DIM);
}

/* Render a full app screen with the mutex held. */

static void ui_render(compose_t fn)
{
  nxmutex_lock(&g_con_mutex);
  gfx_render_all(fn);
  nxmutex_unlock(&g_con_mutex);
}

static void ui_render_rows(compose_t fn, int y0, int y1)
{
  nxmutex_lock(&g_con_mutex);
  gfx_render(fn, y0, y1);
  nxmutex_unlock(&g_con_mutex);
}

/****************************************************************************
 * Main menu
 ****************************************************************************/

#define MENU_Y0   18
#define MENU_STEP 16
#define MENU_IH   16

static void menu_compose(void)
{
  bg_compose();
  statusbar_compose();
  for (int i=0; i<MENU_COUNT; i++)
    {
      int y=MENU_Y0+i*MENU_STEP;
      if (!gfx_band_hits(y, MENU_IH)) continue;
      bool sel=(i==g_menu_sel);
      FAR const struct menu_item_s *m=&g_menu_items[i];
      if (sel)
        {
          gfx_fill_round_grad(4,y,232,MENU_IH,7,COL_SEL_BG2,COL_SEL_BG);
          gfx_fill_round(7,y+3,6,MENU_IH-6,2,
                         gfx_blend565(COL_SEL_BG,COL_MARKER,16));
          gfx_fill_round(8,y+3,4,MENU_IH-6,2,COL_MARKER);
          m->icon(20,y+1,COL_ACCENT);
          gtext(40,y,m->label,COL_SEL_FG);
        }
      else
        {
          m->icon(20,y+1,COL_DIM);
          gtext(40,y,m->label,COL_FG);
        }
    }
  if (MENU_Y0 + MENU_COUNT * MENU_STEP <= 119)     /* room for a footer? */
    footer_compose("UP/DN   ENTER   OPT=menu");
}

static void menu_render(void)       { ui_render(menu_compose); }
static void statusbar_render(void)  { ui_render_rows(statusbar_compose, 0, BAR_H-1); }

/****************************************************************************
 * ScreenTest
 ****************************************************************************/

static void screentest_compose(void)
{
  uint16_t color=g_st_entries[g_st_color_idx].color;
  gfx_clear(color);
  int cw=150,ch=56,cx=(LCD_W-cw)/2,cy=(LCD_H-ch)/2;
  gfx_fill_round(cx,cy,cw,ch,10,COL_BG);
  gfx_round(cx,cy,cw,ch,10,COL_ACCENT);
  gtext_center(LCD_W/2,cy+10,g_st_entries[g_st_color_idx].name,color);
  char idx[16]; snprintf(idx,sizeof(idx),"%d / %d",g_st_color_idx+1,ST_COLOR_COUNT);
  gtext_center(LCD_W/2,cy+30,idx,COL_FG);
  gtext_center(LCD_W/2,LCD_H-16,"UP/DN=colour   OPT=back",COL_DIM);
}

static void screentest_handle(int navkey)
{
  if      (navkey==NAV_UP)   g_st_color_idx=(g_st_color_idx+ST_COLOR_COUNT-1)%ST_COLOR_COUNT;
  else if (navkey==NAV_DOWN) g_st_color_idx=(g_st_color_idx+1)%ST_COLOR_COUNT;
  else return;
  ui_render(screentest_compose);
}

/****************************************************************************
 * Keyboard test
 ****************************************************************************/

static struct
{
  bool    has_key, sh, ct, fn;
  char    ch;
  uint8_t raw;
  uint8_t nav;
} g_kt;

static void kt_chip(int x,int y,FAR const char *s,bool on,uint16_t onc)
{
  if (on) { gfx_fill_round(x,y,26,16,4,onc); gtext(x+5,y,s,COL_BG); }
  else    { gfx_round(x,y,26,16,4,COL_DIM);  gtext(x+5,y,s,COL_DIM); }
}

static void keyboardtest_compose(void)
{
  bg_compose(); statusbar_compose();
  gfx_fill_round_grad(8,22,224,20,6,COL_SEL_BG2,COL_SEL_BG);
  gtext(16,24,"Keyboard Test",COL_ACCENT);
  gfx_fill_round(8,48,224,26,6,COL_SEL_BG);

  if (!g_kt.has_key)
    {
      gtext_center(LCD_W/2,53,"Press any key ...",COL_DIM);
      gtext(16,80,"scan  ---",COL_DIM);
    }
  else
    {
      char key_str[40];
      FAR const char *name=NULL;
      char ch=g_kt.ch;
      if      (g_kt.nav==NAV_UP)    name="UP";
      else if (g_kt.nav==NAV_DOWN)  name="DOWN";
      else if (g_kt.nav==NAV_LEFT)  name="LEFT";
      else if (g_kt.nav==NAV_RIGHT) name="RIGHT";

      if (name)                         snprintf(key_str,sizeof(key_str),"Key:  %s",name);
      else if (ch=='\r')                snprintf(key_str,sizeof(key_str),"Key:  ENTER");
      else if (ch=='\t')                snprintf(key_str,sizeof(key_str),"Key:  TAB");
      else if (ch==(char)KEY_BACKSPACE) snprintf(key_str,sizeof(key_str),"Key:  BACKSPACE");
      else if (ch>=0x20&&(uint8_t)ch<0x7f)
        snprintf(key_str,sizeof(key_str),"Key:  '%c'  (0x%02X)",ch,(uint8_t)ch);
      else snprintf(key_str,sizeof(key_str),"Code: 0x%02X",(uint8_t)ch);

      uint16_t kcol=(ch>=0x20&&(uint8_t)ch<0x7f&&g_kt.nav==NAV_NONE)?COL_SEL_FG:COL_WARN;
      gtext(16,53,key_str,kcol);

      char scan_str[24]; snprintf(scan_str,sizeof(scan_str),"scan  0x%02X",g_kt.raw);
      gtext(16,80,scan_str,COL_FG);
    }

  kt_chip(16,98,"SH",g_kt.sh,COL_WARN);
  kt_chip(48,98,"CT",g_kt.ct,COL_ACCENT);
  kt_chip(80,98,"FN",g_kt.fn,COL_FG);
  footer_compose("Key shown above   OPT=back");
}

static void keyboardtest_enter(void)
{
  memset(&g_kt, 0, sizeof(g_kt));
  ui_render(keyboardtest_compose);
}

static void keyboardtest_show(char ch,uint8_t raw,int navkey,
                              bool sh,bool ct,bool fn)
{
  g_kt.has_key=true; g_kt.ch=ch; g_kt.raw=raw; g_kt.nav=(uint8_t)navkey;
  g_kt.sh=sh; g_kt.ct=ct; g_kt.fn=fn;
  ui_render_rows(keyboardtest_compose, 48, 113);   /* only the key area */
}

/****************************************************************************
 * FileManager  (streams the directory; keeps only the visible window)
 ****************************************************************************/

#define FM_ROW_H    15
#define FM_Y0       38
#define FM_VISIBLE  6
#define FM_SHOW_LEN 26                 /* (240 - 30) / 8 columns */

struct fm_vis_s { char name[FM_SHOW_LEN + 1]; bool is_dir; };

static struct fm_vis_s g_fm_vis[FM_VISIBLE];
static int  g_fm_count=0, g_fm_sel=0, g_fm_top=0;
static char g_fm_path[128]="/";

static inline bool fm_has_parent(void) { return strcmp(g_fm_path, "/") != 0; }

static inline bool fm_skip(FAR const struct dirent *e)
{
  return strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0;
}

static void fm_dir_icon(int x,int y,uint16_t a)
{
  gfx_fill_round(x,y+1,4,2,1,a); gfx_round(x,y+2,11,8,2,a);
}
static void fm_file_icon(int x,int y,uint16_t a)
{
  gfx_round(x+1,y+1,8,10,1,a); gfx_line(x+6,y+1,x+8,y+3,a);
}

static void fm_load(void)
{
  g_fm_count = fm_has_parent() ? 1 : 0;
  g_fm_sel = 0; g_fm_top = 0;
  FAR DIR *d = opendir(g_fm_path);
  if (d)
    {
      FAR struct dirent *e;
      while ((e = readdir(d)) != NULL)
        if (!fm_skip(e)) g_fm_count++;
      closedir(d);
    }
}

/* Fill g_fm_vis[] with entries [g_fm_top .. g_fm_top + FM_VISIBLE). */

static void fm_fill_visible(void)
{
  int hp = fm_has_parent() ? 1 : 0;
  memset(g_fm_vis, 0, sizeof(g_fm_vis));

  if (hp && g_fm_top == 0)
    { strlcpy(g_fm_vis[0].name, "..", sizeof(g_fm_vis[0].name)); g_fm_vis[0].is_dir = true; }

  int first = g_fm_top - hp;          /* dir index shown in slot 0 */
  FAR DIR *d = opendir(g_fm_path);
  if (!d) return;
  FAR struct dirent *e;
  int n = 0;
  while ((e = readdir(d)) != NULL)
    {
      if (fm_skip(e)) continue;
      int slot = n - first;
      if (slot >= FM_VISIBLE) break;
      if (slot >= 0)
        {
          strlcpy(g_fm_vis[slot].name, e->d_name, sizeof(g_fm_vis[slot].name));
          g_fm_vis[slot].is_dir = (e->d_type == DT_DIR);
        }
      n++;
    }
  closedir(d);
}

static bool fm_join(FAR const char *name)
{
  if (strcmp(name,"..")==0)
    {
      FAR char *s=strrchr(g_fm_path,'/');
      if (s) { if (s==g_fm_path) g_fm_path[1]='\0'; else *s='\0'; }
      return true;
    }
  size_t len=strlen(g_fm_path);
  bool slash = (len>0 && g_fm_path[len-1]=='/');
  if (len + (slash ? 0 : 1) + strlen(name) + 1 > sizeof(g_fm_path)) return false;
  snprintf(g_fm_path+len,sizeof(g_fm_path)-len, slash ? "%s" : "/%s", name);
  return true;
}

/* Enter the selected entry if it is a directory (re-reads its full name). */

static bool fm_open_selected(void)
{
  int hp = fm_has_parent() ? 1 : 0;
  if (hp && g_fm_sel == 0) return fm_join("..");

  int want = g_fm_sel - hp, n = 0;
  bool ok = false;
  FAR DIR *d = opendir(g_fm_path);
  if (!d) return false;
  FAR struct dirent *e;
  while ((e = readdir(d)) != NULL)
    {
      if (fm_skip(e)) continue;
      if (n++ == want)
        {
          if (e->d_type == DT_DIR) ok = fm_join(e->d_name);
          break;
        }
    }
  closedir(d);
  return ok;
}

static void fm_compose(void)
{
  bg_compose(); statusbar_compose();
  gfx_fill_round(6,20,228,14,4,COL_SEL_BG);
  char pbuf[28]; snprintf(pbuf,sizeof(pbuf),"%.26s",g_fm_path);
  gtext(12,19,pbuf,COL_ACCENT);
  if (g_fm_count>FM_VISIBLE)
    {
      char sc[24]; snprintf(sc,sizeof(sc),"%d/%d",g_fm_sel+1,g_fm_count);
      gtext_right(LCD_W-10,19,sc,COL_DIM);
    }
  for (int i=0;i<FM_VISIBLE;i++)
    {
      int idx=g_fm_top+i, y=FM_Y0+i*FM_ROW_H;
      if (idx>=g_fm_count || !gfx_band_hits(y, FM_ROW_H + 1)) continue;
      bool sel=(idx==g_fm_sel), is_dir=g_fm_vis[i].is_dir;
      uint16_t icol=is_dir?COL_ACCENT:COL_DIM;
      if (sel) { gfx_fill_round(4,y,232,FM_ROW_H-1,5,COL_SEL_BG);
                 gfx_fill_round(6,y+2,3,FM_ROW_H-5,1,COL_MARKER); }
      if (is_dir) fm_dir_icon(13,y+1,icol);
      else        fm_file_icon(13,y+1,icol);
      gtext(30,y,g_fm_vis[i].name,sel?COL_SEL_FG:(is_dir?COL_FG:COL_DIM));
    }
}

static void fm_render(void)
{
  int old_top = g_fm_top;
  if (g_fm_sel<g_fm_top) g_fm_top=g_fm_sel;
  if (g_fm_sel>=g_fm_top+FM_VISIBLE) g_fm_top=g_fm_sel-FM_VISIBLE+1;
  if (old_top != g_fm_top) fm_fill_visible();     /* re-read only on scroll */
  ui_render(fm_compose);
}

static void fm_reload_render(void)
{
  fm_load();
  fm_fill_visible();
  ui_render(fm_compose);
}

static void filemanager_enter(void)
{
  snprintf(g_fm_path,sizeof(g_fm_path),"/");
  fm_reload_render();
}

static void filemanager_handle(int navkey,char ch)
{
  if (navkey==NAV_UP)
    { if (g_fm_count) { g_fm_sel=(g_fm_sel+g_fm_count-1)%g_fm_count; fm_render(); } }
  else if (navkey==NAV_DOWN)
    { if (g_fm_count) { g_fm_sel=(g_fm_sel+1)%g_fm_count; fm_render(); } }
  else if ((ch=='\r'||ch=='\n') && g_fm_count)
    { if (fm_open_selected()) fm_reload_render(); }
}

/****************************************************************************
 * Battery app
 ****************************************************************************/

static void battery_compose(void)
{
  struct bat_read_s r = g_bat;
  bg_compose(); statusbar_compose();
  if (r.mv<0)
    {
      gfx_fill_round(20,40,200,56,10,COL_SEL_BG);
      gfx_round(20,40,200,56,10,COL_ERR);
      gtext_center(LCD_W/2,50,"Battery read FAILED",COL_ERR);
      char line[40];
      if (r.err)
        snprintf(line,sizeof(line),"errno %d  (%s)",r.err,
                 r.err==ENOENT?"/dev/adc0 missing":"open/read err");
      else
        snprintf(line,sizeof(line),"got %d ch, ch0=%d raw=%d",r.nsamp,r.ch0,r.raw0);
      gtext_center(LCD_W/2,70,line,COL_DIM);
    }
  else
    {
      int pct=battery_percent(r.mv);
      uint16_t col=battery_color(pct);
      int bx=30,by=36,bw=150,bh=48;
      gfx_round(bx,by,bw,bh,8,col);
      gfx_round(bx+1,by+1,bw-2,bh-2,7,col);
      gfx_fill_round(bx+bw,by+15,9,18,3,col);
      int fillw=pct*(bw-12)/100;
      if (fillw>0)
        gfx_fill_round_grad(bx+6,by+6,fillw,bh-12,4,
                            gfx_blend565(col,RGB_WHITE,10),col);
      char big[16]; snprintf(big,sizeof(big),"%d%%",pct);
      gtext_center(bx+bw/2,by+bh/2-8,big,COL_SEL_FG);
      char line[40];
      snprintf(line,sizeof(line),"Voltage  %d mV",r.mv);
      gtext(30,94,line,COL_FG);
      FAR const char *state=(pct>=95)?"Full":(pct<=20)?"Low - charge soon":"OK";
      snprintf(line,sizeof(line),"Status   %s",state);
      gtext(30,110,line,col);
    }
  footer_compose("auto-refresh   OPT=back");
}

static void battery_render(void)
{
  battery_refresh();
  ui_render(battery_compose);
}

/****************************************************************************
 * Wi-Fi app
 *
 *   LIST  : link status card + scan results (strongest first)
 *           s = scan   r = refresh status   ENTER = join   UP/DN = select
 *   PASS  : type the passphrase   TAB = show/hide   ENTER = connect
 *           fn+LEFT = back to the list   (empty passphrase = open network)
 *
 *   Scan, status and join use the wireless ioctls directly from the kernel
 *   (psock_*: no fd, no libc socket table) — the same requests `wapi scan`,
 *   `wapi psk` and `wapi essid` send, so the passphrase never passes
 *   through the shell.  After association the terminal opens and runs
 *   `renew wlan0` (stock DHCP client) so you can watch the lease arrive.
 *
 *   All state lives in one heap block that exists only while the app is
 *   open (≈460 B) and is zeroed before it is freed.
 ****************************************************************************/

#ifdef CARDPUTER_WIFI

#define WIFI_IFNAME      "wlan0"
#define WIFI_MAX_AP      8
#define WIFI_VISIBLE     4
#define WIFI_LIST_Y0     54
#define WIFI_ROW_H       15
#define WIFI_PASS_MAX    63
#define WIFI_STATUS_MS   3000

enum { WIFI_ST_LIST = 0, WIFI_ST_PASS };

struct wifi_ap_s
{
  char    ssid[IW_ESSID_MAX_SIZE + 1];
  int8_t  rssi;
  uint8_t chan;
  uint8_t saved;                             /* passphrase in flash */
};

struct wifi_ui_s
{
  struct wifi_ap_s ap[WIFI_MAX_AP];
  int8_t  nap, sel, top, state;
  bool    show_pass, scanned, up, running;
  int8_t  rssi;                              /* dBm, 0 = unknown */
  uint8_t passlen;
  char    pass[WIFI_PASS_MAX + 1];
  char    cur_ssid[IW_ESSID_MAX_SIZE + 1];
  char    ip[16];
  char    msg[30];                           /* one-line notice */
};

static FAR struct wifi_ui_s *g_wifi = NULL;

static void wifi_free(void)
{
  FAR struct wifi_ui_s *w = g_wifi;
  if (w == NULL) return;
  g_wifi = NULL;
  memset(w, 0, sizeof(*w));                 /* wipe the passphrase */
  kmm_free(w);
}

static int wifi_sock(FAR struct socket *s)
{
  int ret = psock_socket(AF_INET, SOCK_DGRAM, 0, s);
  if (ret < 0) ret = psock_socket(AF_INET, SOCK_STREAM, 0, s);
  return ret;
}

static inline void wifi_ifname(FAR char *dst)
{
  strlcpy(dst, WIFI_IFNAME, IFNAMSIZ);
}

static int8_t rssi_from_dbm(int v)
{
  if (v > 0) v = -v;            /* ESP driver reports -(rssi) */
  if (v < -127) v = -127;
  return (int8_t)v;
}

static int rssi_bars(int rssi)
{
  if (rssi == 0)   return 0;
  if (rssi >= -60) return 3;
  if (rssi >= -72) return 2;
  if (rssi >= -85) return 1;
  return 0;
}

static void wifi_status_refresh(FAR struct wifi_ui_s *w)
{
  struct socket s;
  struct ifreq  ifr;
  struct iwreq  iwr;

  w->up = w->running = false;
  w->cur_ssid[0] = '\0';
  w->rssi = 0;
  strlcpy(w->ip, "-", sizeof(w->ip));

  if (wifi_sock(&s) < 0)
    {
      strlcpy(w->msg, "socket() failed", sizeof(w->msg));
      return;
    }

  memset(&ifr, 0, sizeof(ifr));
  wifi_ifname(ifr.ifr_name);
  if (psock_ioctl(&s, SIOCGIFFLAGS, (unsigned long)(uintptr_t)&ifr) >= 0)
    {
      w->up      = (ifr.ifr_flags & IFF_UP) != 0;
      w->running = (ifr.ifr_flags & IFF_RUNNING) != 0;
    }
  else
    {
      if (g_wlan_init_ret < 0)
        snprintf(w->msg, sizeof(w->msg), "wifi init err %d", g_wlan_init_ret);
      else if (g_wlan_init_ret == 1)
        strlcpy(w->msg, "board_wlan_init not run", sizeof(w->msg));
      else
        strlcpy(w->msg, "no " WIFI_IFNAME " device", sizeof(w->msg));
    }

  memset(&ifr, 0, sizeof(ifr));
  wifi_ifname(ifr.ifr_name);
  if (psock_ioctl(&s, SIOCGIFADDR, (unsigned long)(uintptr_t)&ifr) >= 0)
    {
      FAR struct sockaddr_in *sin = (FAR struct sockaddr_in *)&ifr.ifr_addr;
      uint32_t a = ntohl(sin->sin_addr.s_addr);
      if (a != 0)
        snprintf(w->ip, sizeof(w->ip), "%u.%u.%u.%u",
                 (unsigned)(a >> 24) & 0xff, (unsigned)(a >> 16) & 0xff,
                 (unsigned)(a >> 8) & 0xff,  (unsigned)a & 0xff);
    }

  if (w->running)
    {
      memset(&iwr, 0, sizeof(iwr));
      wifi_ifname(iwr.ifr_name);
      iwr.u.essid.pointer = w->cur_ssid;
      iwr.u.essid.length  = IW_ESSID_MAX_SIZE;
      memset(w->cur_ssid, 0, sizeof(w->cur_ssid));
      if (psock_ioctl(&s, SIOCGIWESSID, (unsigned long)(uintptr_t)&iwr) < 0)
        w->cur_ssid[0] = '\0';
      w->cur_ssid[IW_ESSID_MAX_SIZE] = '\0';

      memset(&iwr, 0, sizeof(iwr));
      wifi_ifname(iwr.ifr_name);
      if (psock_ioctl(&s, SIOCGIWSENS, (unsigned long)(uintptr_t)&iwr) >= 0)
        w->rssi = rssi_from_dbm(iwr.u.sens.value);
    }

  psock_close(&s);
}

/* Insert keeping the list sorted by RSSI and free of duplicate SSIDs. */

static void wifi_add_ap(FAR struct wifi_ui_s *w,
                        FAR const struct wifi_ap_s *ap)
{
  int i;
  if (ap->ssid[0] == '\0') return;                    /* hidden network */

  for (i = 0; i < w->nap; i++)
    {
      if (strcmp(w->ap[i].ssid, ap->ssid) == 0)
        {
          if (ap->rssi <= w->ap[i].rssi) return;      /* keep stronger */
          memmove(&w->ap[i], &w->ap[i + 1],
                  (w->nap - i - 1) * sizeof(w->ap[0]));
          w->nap--;
          break;
        }
    }

  i = 0;
  while (i < w->nap && w->ap[i].rssi >= ap->rssi) i++;
  if (i >= WIFI_MAX_AP) return;

  int n = w->nap < WIFI_MAX_AP ? w->nap : WIFI_MAX_AP - 1;
  memmove(&w->ap[i + 1], &w->ap[i], (n - i) * sizeof(w->ap[0]));
  w->ap[i] = *ap;
  if (w->nap < WIFI_MAX_AP) w->nap++;
}

/* Walk the iw_event stream returned by SIOCGIWSCAN. */

static void wifi_parse_scan(FAR struct wifi_ui_s *w,
                            FAR const uint8_t *buf, size_t len)
{
  const size_t hdr = offsetof(struct iw_event, u);
  struct wifi_ap_s cur;
  bool have = false;
  size_t off = 0;

  memset(&cur, 0, sizeof(cur));
  while (off + hdr <= len)
    {
      struct iw_event iwe;
      size_t avail = len - off;

      memset(&iwe, 0, sizeof(iwe));
      memcpy(&iwe, buf + off, avail < sizeof(iwe) ? avail : sizeof(iwe));
      if (iwe.len < hdr || iwe.len > avail) break;

      switch (iwe.cmd)
        {
          case SIOCGIWAP:                       /* start of a new BSS */
            if (have) wifi_add_ap(w, &cur);
            memset(&cur, 0, sizeof(cur));
            have = true;
            break;

          case SIOCGIWESSID:
            {
              size_t l = iwe.u.essid.length;
              size_t p = off + hdr + (size_t)(uintptr_t)iwe.u.essid.pointer;
              if (l > IW_ESSID_MAX_SIZE) l = IW_ESSID_MAX_SIZE;
              if (p + l <= len)
                {
                  memcpy(cur.ssid, buf + p, l);
                  cur.ssid[l] = '\0';
                }
            }
            break;

          case IWEVQUAL:
            cur.rssi = (int8_t)iwe.u.qual.level;
            break;

          case SIOCGIWFREQ:
            if (iwe.u.freq.e == 0 && iwe.u.freq.m > 0 && iwe.u.freq.m < 200)
              cur.chan = (uint8_t)iwe.u.freq.m;
            break;

          default:
            break;
        }

      off += iwe.len;
    }

  if (have) wifi_add_ap(w, &cur);
}

static int wifi_do_scan(FAR struct wifi_ui_s *w)
{
  struct socket s;
  struct iwreq  iwr;
  FAR uint8_t  *buf = NULL;
  uint16_t      size = 0;
  int           ret;

  w->nap = 0; w->sel = 0; w->top = 0;

  ret = wifi_sock(&s);
  if (ret < 0) return ret;

  memset(&iwr, 0, sizeof(iwr));
  wifi_ifname(iwr.ifr_name);
  ret = psock_ioctl(&s, SIOCSIWSCAN, (unsigned long)(uintptr_t)&iwr);

  /* The driver blocks until the scan is done.  A NULL buffer asks for the
   * result size (-E2BIG); the second call copies and frees the driver's
   * copy.  -EAGAIN (other drivers) means "not done yet".
   */

  for (int tries = 0; ret >= 0 && tries < 20; tries++)
    {
      memset(&iwr, 0, sizeof(iwr));
      wifi_ifname(iwr.ifr_name);
      iwr.u.data.pointer = buf;
      iwr.u.data.length  = size;
      ret = psock_ioctl(&s, SIOCGIWSCAN, (unsigned long)(uintptr_t)&iwr);

      if (ret == -EAGAIN)
        {
          ret = 0;
          usleep(250 * 1000);
          continue;
        }

      if (ret == -E2BIG && buf == NULL)
        {
          size = iwr.u.data.length ? iwr.u.data.length : 2048;
          buf  = kmm_malloc(size);
          ret  = buf ? 0 : -ENOMEM;
          continue;
        }

      if (ret >= 0 && buf != NULL)
        wifi_parse_scan(w, buf, iwr.u.data.length);
      break;
    }

  if (buf) kmm_free(buf);
  psock_close(&s);
  w->scanned = true;
  return ret < 0 ? ret : w->nap;
}

/* --- saved networks (flash) --------------------------------------------- */

#ifdef CARDPUTER_WIFI_STORE

#if defined(CONFIG_ESP32S3_FLASH_32M) || defined(CONFIG_ESPRESSIF_FLASH_32M)
#  define WSTORE_FLASH_BYTES 0x2000000
#elif defined(CONFIG_ESP32S3_FLASH_16M) || defined(CONFIG_ESPRESSIF_FLASH_16M)
#  define WSTORE_FLASH_BYTES 0x1000000
#elif defined(CONFIG_ESP32S3_FLASH_8M) || defined(CONFIG_ESPRESSIF_FLASH_8M)
#  define WSTORE_FLASH_BYTES 0x800000
#else
#  define WSTORE_FLASH_BYTES 0x400000
#endif

#ifdef CONFIG_ESP32S3_PARTITION_TABLE_OFFSET
#  define WSTORE_PT_OFFSET CONFIG_ESP32S3_PARTITION_TABLE_OFFSET
#else
#  define WSTORE_PT_OFFSET 0x8000
#endif
#define WSTORE_PT_BYTES  0xc00                /* 96 entries x 32 B */

#define WSTORE_SIZE    0x1000
#define WSTORE_MAGIC   0x46575043u            /* "CPWF" */
#define WSTORE_VERSION 1
#define WSTORE_MAX     4
#define WSTORE_IMG     512                    /* bytes read/written */

struct wifi_cred_s
{
  char    ssid[IW_ESSID_MAX_SIZE + 1];
  uint8_t passlen;
  char    pass[WIFI_PASS_MAX + 1];
};

struct wifi_store_s
{
  uint32_t magic;
  uint16_t version;
  uint16_t count;
  struct wifi_cred_s net[WSTORE_MAX];       /* [0] = most recently used */
  uint32_t crc;
};

static FAR struct mtd_dev_s *g_wstore_mtd;
static uint32_t g_wstore_off;                /* chosen sector, 1 = none */

static uint32_t wstore_crc(FAR const void *data, size_t len)
{
  FAR const uint8_t *p = data;
  uint32_t crc = 0xffffffffu;
  while (len--)
    {
      crc ^= *p++;
      for (int k = 0; k < 8; k++)
        crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1)));
    }
  return ~crc;
}

/* Pick the flash sector for the store.
 *
 *   No partition table at 0x8000 (simple-boot image: our own code sits
 *   there) -> the last sector of the chip.
 *   Partition table present (IDF bootloader / M5 Launcher, which hands the
 *   free space to other apps and their data partitions) -> the highest
 *   sector above 0x10000 that no partition covers, or none at all.
 *   A layout change made later by Launcher can at worst overwrite the
 *   store, which the CRC then rejects -- it never makes us write into
 *   someone else's partition, because the table is checked every boot.
 */

static uint32_t wstore_pick_offset(void)
{
  FAR struct mtd_dev_s *pt;
  struct mtd_geometry_s geo;
  uint32_t off[24], len[24];
  FAR uint8_t *tbl;
  int n = 0;

  pt = esp_spiflash_alloc_mtdpart(WSTORE_PT_OFFSET, 0x1000);
  memset(&geo, 0, sizeof(geo));
  if (pt == NULL || MTD_IOCTL(pt, MTDIOC_GEOMETRY,
                              (unsigned long)(uintptr_t)&geo) < 0 ||
      geo.blocksize == 0 || (tbl = kmm_malloc(WSTORE_PT_BYTES)) == NULL)
    return 1;

  if (MTD_BREAD(pt, 0, WSTORE_PT_BYTES / geo.blocksize, tbl) ==
      (ssize_t)(WSTORE_PT_BYTES / geo.blocksize))
    {
      for (int i = 0; i < WSTORE_PT_BYTES && n < 24; i += 32)
        {
          FAR const uint8_t *e = tbl + i;
          if (e[0] != 0xaa || e[1] != 0x50) break;     /* end / MD5 / none */
          off[n] = e[4] | (e[5] << 8) | (e[6] << 16) | ((uint32_t)e[7] << 24);
          len[n] = e[8] | (e[9] << 8) | (e[10] << 16) | ((uint32_t)e[11] << 24);
          n++;
        }
    }

  kmm_free(tbl);
  if (n == 0) return WSTORE_FLASH_BYTES - WSTORE_SIZE;

  for (uint32_t sec = WSTORE_FLASH_BYTES - WSTORE_SIZE; sec >= 0x10000;
       sec -= WSTORE_SIZE)
    {
      bool used = false;
      for (int i = 0; i < n && !used; i++)
        used = sec < off[i] + len[i] && sec + WSTORE_SIZE > off[i];
      if (!used) return sec;
    }

  return 1;                                    /* every sector is taken */
}

/* Returns the MTD and the block count covering WSTORE_IMG bytes. */

static FAR struct mtd_dev_s *wstore_open(FAR size_t *nblk, FAR size_t *img)
{
  struct mtd_geometry_s geo;

  if (g_wstore_off == 0)
    {
      g_wstore_off = wstore_pick_offset();
      if (g_wstore_off != 1)
        {
          g_wstore_mtd = esp_spiflash_alloc_mtdpart(g_wstore_off, WSTORE_SIZE);
          syslog(LOG_INFO, "wifi: credential store at 0x%" PRIx32 "\n",
                 g_wstore_off);
        }
      else
        {
          syslog(LOG_WARNING, "wifi: no free flash sector, not saving\n");
        }
    }

  if (g_wstore_mtd == NULL) return NULL;

  memset(&geo, 0, sizeof(geo));
  if (MTD_IOCTL(g_wstore_mtd, MTDIOC_GEOMETRY,
                (unsigned long)(uintptr_t)&geo) < 0 || geo.blocksize == 0)
    return NULL;

  *nblk = (WSTORE_IMG + geo.blocksize - 1) / geo.blocksize;
  *img  = *nblk * geo.blocksize;
  return g_wstore_mtd;
}

/* Load into *st (heap, caller frees).  Returns the entry count, 0 when the
 * sector is blank or fails its checks.
 */

static int wstore_load(FAR struct wifi_store_s *st)
{
  size_t nblk, isz;
  FAR struct mtd_dev_s *mtd = wstore_open(&nblk, &isz);
  FAR uint8_t *img;
  ssize_t n;

  memset(st, 0, sizeof(*st));
  if (mtd == NULL || (img = kmm_malloc(isz)) == NULL) return 0;

  n = MTD_BREAD(mtd, 0, nblk, img);
  if (n == (ssize_t)nblk) memcpy(st, img, sizeof(*st));
  memset(img, 0, isz);
  kmm_free(img);

  if (st->magic != WSTORE_MAGIC || st->version != WSTORE_VERSION ||
      st->count > WSTORE_MAX ||
      st->crc != wstore_crc(st, offsetof(struct wifi_store_s, crc)))
    {
      memset(st, 0, sizeof(*st));
      return 0;
    }

  for (int i = 0; i < st->count; i++)          /* never trust lengths */
    {
      st->net[i].ssid[IW_ESSID_MAX_SIZE] = '\0';
      if (st->net[i].passlen > WIFI_PASS_MAX) st->net[i].passlen = 0;
      st->net[i].pass[st->net[i].passlen] = '\0';
    }

  return st->count;
}

static int wstore_save(FAR struct wifi_store_s *st)
{
  size_t nblk, isz;
  FAR struct mtd_dev_s *mtd = wstore_open(&nblk, &isz);
  FAR uint8_t *img;
  ssize_t n;
  int ret;

  if (mtd == NULL) return g_wstore_off == 1 ? -ENOSPC : -ENODEV;
  if ((img = kmm_malloc(isz)) == NULL) return -ENOMEM;

  st->magic   = WSTORE_MAGIC;
  st->version = WSTORE_VERSION;
  st->crc     = wstore_crc(st, offsetof(struct wifi_store_s, crc));

  memset(img, 0xff, isz);
  memcpy(img, st, sizeof(*st));

  ret = MTD_ERASE(mtd, 0, 1);
  if (ret >= 0)
    {
      n = MTD_BWRITE(mtd, 0, nblk, img);
      ret = (n == (ssize_t)nblk) ? OK : (n < 0 ? (int)n : -EIO);
    }

  memset(img, 0, isz);
  kmm_free(img);
  return ret;
}

/* Copy the saved passphrase for `ssid` into pass[WIFI_PASS_MAX+1]. */

static bool wstore_find(FAR const char *ssid, FAR char *pass,
                        FAR uint8_t *plen)
{
  FAR struct wifi_store_s *st = kmm_malloc(sizeof(*st));
  bool found = false;

  if (st == NULL) return false;
  wstore_load(st);
  for (int i = 0; i < st->count && !found; i++)
    if (strcmp(st->net[i].ssid, ssid) == 0)
      {
        memcpy(pass, st->net[i].pass, WIFI_PASS_MAX + 1);
        *plen = st->net[i].passlen;
        found = true;
      }

  memset(st, 0, sizeof(*st));
  kmm_free(st);
  return found;
}

/* Put ssid/pass first; keep the other most recent ones.  Writes flash only
 * when something actually changed (flash sectors wear out).
 */

static int wstore_remember(FAR const char *ssid, FAR const char *pass,
                           size_t plen)
{
  FAR struct wifi_store_s *st = kmm_malloc(sizeof(*st));
  struct wifi_cred_s cur;
  int ret = OK;

  if (st == NULL) return -ENOMEM;
  wstore_load(st);

  if (st->count > 0 && strcmp(st->net[0].ssid, ssid) == 0 &&
      st->net[0].passlen == plen && memcmp(st->net[0].pass, pass, plen) == 0)
    goto out;                                 /* already first, unchanged */

  memset(&cur, 0, sizeof(cur));
  strlcpy(cur.ssid, ssid, sizeof(cur.ssid));
  cur.passlen = (uint8_t)plen;
  memcpy(cur.pass, pass, plen);

  int n = 0;
  struct wifi_cred_s keep[WSTORE_MAX];
  for (int i = 0; i < st->count && n < WSTORE_MAX - 1; i++)
    if (strcmp(st->net[i].ssid, ssid) != 0) keep[n++] = st->net[i];

  st->net[0] = cur;
  for (int i = 0; i < n; i++) st->net[i + 1] = keep[i];
  for (int i = n + 1; i < WSTORE_MAX; i++) memset(&st->net[i], 0, sizeof(st->net[i]));
  st->count = (uint16_t)(n + 1);
  ret = wstore_save(st);

  memset(keep, 0, sizeof(keep));
  memset(&cur, 0, sizeof(cur));
out:
  memset(st, 0, sizeof(*st));
  kmm_free(st);
  return ret;
}

static int wstore_forget(FAR const char *ssid)
{
  FAR struct wifi_store_s *st = kmm_malloc(sizeof(*st));
  int ret = -ENOENT, n = 0;

  if (st == NULL) return -ENOMEM;
  wstore_load(st);
  for (int i = 0; i < st->count; i++)
    if (strcmp(st->net[i].ssid, ssid) != 0) st->net[n++] = st->net[i];

  if (n != st->count)
    {
      for (int i = n; i < WSTORE_MAX; i++) memset(&st->net[i], 0, sizeof(st->net[i]));
      st->count = (uint16_t)n;
      ret = wstore_save(st);
    }

  memset(st, 0, sizeof(*st));
  kmm_free(st);
  return ret;
}

static void wifi_mark_saved(FAR struct wifi_ui_s *w)
{
  FAR struct wifi_store_s *st = kmm_malloc(sizeof(*st));
  if (st == NULL) return;
  wstore_load(st);
  for (int a = 0; a < w->nap; a++)
    {
      w->ap[a].saved = 0;
      for (int i = 0; i < st->count; i++)
        if (strcmp(w->ap[a].ssid, st->net[i].ssid) == 0) w->ap[a].saved = 1;
    }
  memset(st, 0, sizeof(*st));
  kmm_free(st);
}

#else  /* !CARDPUTER_WIFI_STORE */

static bool wstore_find(FAR const char *s, FAR char *p, FAR uint8_t *l)
{ UNUSED(s); UNUSED(p); UNUSED(l); return false; }
static int  wstore_remember(FAR const char *s, FAR const char *p, size_t l)
{ UNUSED(s); UNUSED(p); UNUSED(l); return -ENOSYS; }
static int  wstore_forget(FAR const char *s) { UNUSED(s); return -ENOSYS; }
static void wifi_mark_saved(FAR struct wifi_ui_s *w) { UNUSED(w); }

#endif /* CARDPUTER_WIFI_STORE */

/* --- drawing ------------------------------------------------------------ */

static void wifi_list_compose(FAR struct wifi_ui_s *w)
{
  /* Status card */

  gfx_fill_round(6, 19, 228, 33, 6, COL_SEL_BG);
  gfx_wifi_glyph(220, 44, w->running ? rssi_bars(w->rssi) : 0,
                 COL_ACCENT, COL_DIM);

  char line[32];
  if (w->running)
    {
      snprintf(line, sizeof(line), "%.24s",
               w->cur_ssid[0] ? w->cur_ssid : "(connected)");
      gtext(12, 19, line, COL_SEL_FG);
      if (w->rssi)
        snprintf(line, sizeof(line), "%-15s %ddBm", w->ip, w->rssi);
      else
        snprintf(line, sizeof(line), "%s", w->ip);
      gtext(12, 35, line, COL_FG);
    }
  else
    {
      gtext(12, 19, w->up ? "Not connected" : WIFI_IFNAME " is down",
            COL_WARN);
      gtext(12, 35, w->msg[0] ? w->msg : "s = scan networks", COL_DIM);
    }

  /* Scan results */

  if (w->nap == 0)
    {
      gtext_center(LCD_W / 2, 70,
                   w->scanned ? "No networks found" : "Press s to scan",
                   COL_DIM);
    }

  for (int i = 0; i < WIFI_VISIBLE; i++)
    {
      int idx = w->top + i, y = WIFI_LIST_Y0 + i * WIFI_ROW_H;
      if (idx >= w->nap || !gfx_band_hits(y, WIFI_ROW_H + 1)) continue;
      FAR const struct wifi_ap_s *ap = &w->ap[idx];
      bool sel = (idx == w->sel);
      if (sel)
        {
          gfx_fill_round(4, y, 232, WIFI_ROW_H - 1, 5, COL_SEL_BG);
          gfx_fill_round(6, y + 2, 3, WIFI_ROW_H - 5, 1, COL_MARKER);
        }
      gfx_wifi_glyph(19, y + 12, rssi_bars(ap->rssi),
                     sel ? COL_ACCENT : COL_FG, COL_DIM);
      snprintf(line, sizeof(line), "%.18s", ap->ssid);
      gtext(32, y, line, sel ? COL_SEL_FG : COL_FG);
      if (ap->saved) gfx_fill_circle(186, y + 8, 2, COL_ACCENT);   /* saved */
      snprintf(line, sizeof(line), "%d", ap->rssi);
      gtext_right(LCD_W - 10, y, line, COL_DIM);
    }

  if (w->nap > WIFI_VISIBLE)
    {
      snprintf(line, sizeof(line), "%d/%d", w->sel + 1, w->nap);
      gtext_right(LCD_W - 6, 119, line, COL_DIM);
    }

  footer_compose("s=scan ok=join f=forget");
}

static void wifi_pass_compose(FAR struct wifi_ui_s *w)
{
  char line[40];
  FAR const struct wifi_ap_s *ap = &w->ap[w->sel];

  gfx_fill_round_grad(8, 20, 224, 20, 6, COL_SEL_BG2, COL_SEL_BG);
  snprintf(line, sizeof(line), "Join %.22s", ap->ssid);
  gtext(16, 22, line, COL_ACCENT);

  gtext(12, 46, "Passphrase", COL_DIM);
  gfx_fill_round(8, 62, 224, 22, 6, COL_SEL_BG);
  gfx_round(8, 62, 224, 22, 6, COL_ACCENT);

  /* Show the tail of the passphrase that fits (26 columns) */

  int vis = w->passlen > 25 ? 25 : w->passlen;
  int from = w->passlen - vis;
  for (int i = 0; i < vis; i++)
    {
      char c = w->show_pass ? w->pass[from + i] : '*';
      if (!w->show_pass && i == vis - 1) c = w->pass[from + i];  /* last key */
      gfx_glyph(16 + i * FONT_W, 65, c, COL_SEL_FG);
    }
  gfx_fill_rect(16 + vis * FONT_W, 66, 2, 14, COL_ACCENT);      /* cursor */

  snprintf(line, sizeof(line), "%d chars  %s", w->passlen,
           w->passlen == 0 ? "(open net)" :
           w->passlen < 8  ? "(min 8)"    : "");
  gtext(12, 90, line, (w->passlen && w->passlen < 8) ? COL_WARN : COL_DIM);
  if (w->msg[0]) gtext(12, 104, w->msg, COL_WARN);

  footer_compose("TAB=show ENTER=go fn+j=back");
}

static void wifi_compose(void)
{
  FAR struct wifi_ui_s *w = g_wifi;
  bg_compose(); statusbar_compose();

  if (w == NULL)
    {
      gtext_center(LCD_W / 2, 60, "Out of memory", COL_ERR);
      footer_compose("OPT=back");
      return;
    }

  if (w->state == WIFI_ST_PASS) wifi_pass_compose(w);
  else                          wifi_list_compose(w);
}

static void wifi_scanning_compose(void)
{
  bg_compose(); statusbar_compose();
  gfx_fill_round(30, 48, 180, 40, 10, COL_SEL_BG);
  gfx_round(30, 48, 180, 40, 10, COL_ACCENT);
  gfx_wifi_glyph(52, 76, 3, COL_ACCENT, COL_DIM);
  gtext(72, 60, "Scanning...", COL_SEL_FG);
}

static void wifi_render(void) { ui_render(wifi_compose); }

/* --- actions ------------------------------------------------------------ */

/* Bring wlan0 up if NSH's netinit didn't; report the driver's error. */

static int wifi_set_up(void)
{
  struct socket s;
  struct ifreq  ifr;
  int ret;

  ret = wifi_sock(&s);
  if (ret < 0) return ret;
  memset(&ifr, 0, sizeof(ifr));
  wifi_ifname(ifr.ifr_name);
  ret = psock_ioctl(&s, SIOCGIFFLAGS, (unsigned long)(uintptr_t)&ifr);
  if (ret >= 0 && !(ifr.ifr_flags & IFF_UP))
    {
      ifr.ifr_flags |= IFF_UP;
      ret = psock_ioctl(&s, SIOCSIFFLAGS, (unsigned long)(uintptr_t)&ifr);
    }
  psock_close(&s);
  return ret;
}

static void wifi_ifup(FAR struct wifi_ui_s *w)
{
  if (w->up) return;
  int ret = wifi_set_up();
  if (ret < 0) snprintf(w->msg, sizeof(w->msg), "ifup err %d", ret);
  wifi_status_refresh(w);
}

static void wifi_enter(void)
{
  if (g_wifi == NULL) g_wifi = kmm_zalloc(sizeof(struct wifi_ui_s));
  if (g_wifi != NULL)
    {
      wifi_status_refresh(g_wifi);
      if (!g_wifi->up && g_wlan_init_ret == 0) wifi_ifup(g_wifi);
    }
  wifi_render();
}

static void wifi_scan(FAR struct wifi_ui_s *w)
{
  ui_render(wifi_scanning_compose);
  int ret = wifi_do_scan(w);
  if (ret < 0) snprintf(w->msg, sizeof(w->msg), "scan failed (%d)", ret);
  else         w->msg[0] = '\0';
  wifi_mark_saved(w);
  wifi_status_refresh(w);
  wifi_render();
}

static void kbd_echo_str(FAR const char *s);
static void ui_enter_mode(int mode);

static int wifi_set_auth(FAR struct socket *s, int idx, uint32_t value)
{
  struct iwreq iwr;
  memset(&iwr, 0, sizeof(iwr));
  wifi_ifname(iwr.ifr_name);
  iwr.u.param.flags = idx & IW_AUTH_INDEX;
  iwr.u.param.value = value;
  int ret = psock_ioctl(s, SIOCSIWAUTH, (unsigned long)(uintptr_t)&iwr);
  return ret == -EOPNOTSUPP ? 0 : ret;
}

/* The same three steps `wapi psk` + `wapi essid ... 1` perform, done in
 * the kernel so the passphrase never goes through the shell (no quoting
 * problems, no LINE_MAX limit, nothing in NSH's line buffer).
 * Blocks until associated or CONFIG_ESPRESSIF_WIFI_CONNECT_TIMEOUT.
 */

static int wifi_join(FAR const char *ssid, FAR const char *pass, size_t plen)
{
  uint8_t kbuf[sizeof(struct iw_encode_ext) + WIFI_PASS_MAX] aligned_data(4);
  FAR struct iw_encode_ext *ext = (FAR struct iw_encode_ext *)kbuf;
  struct socket s;
  struct iwreq  iwr;
  int ret;

  ret = wifi_sock(&s);
  if (ret < 0) return ret;

  /* WPA2 is a *minimum*: WPA2/WPA3 mixed and WPA3-SAE APs also match. */

  ret = wifi_set_auth(&s, IW_AUTH_WPA_VERSION,
                      plen ? IW_AUTH_WPA_VERSION_WPA2
                           : IW_AUTH_WPA_VERSION_DISABLED);
  if (ret >= 0)
    ret = wifi_set_auth(&s, IW_AUTH_CIPHER_PAIRWISE,
                        plen ? IW_AUTH_CIPHER_CCMP : IW_AUTH_CIPHER_NONE);

  if (ret >= 0)
    {
      memset(kbuf, 0, sizeof(kbuf));
      ext->alg     = plen ? IW_ENCODE_ALG_CCMP : IW_ENCODE_ALG_NONE;
      ext->key_len = (uint16_t)plen;
      memcpy(ext->key, pass, plen);

      memset(&iwr, 0, sizeof(iwr));
      wifi_ifname(iwr.ifr_name);
      iwr.u.encoding.pointer = ext;
      iwr.u.encoding.length  = (uint16_t)(sizeof(*ext) + plen);
      ret = psock_ioctl(&s, SIOCSIWENCODEEXT, (unsigned long)(uintptr_t)&iwr);
      memset(kbuf, 0, sizeof(kbuf));                  /* wipe key copy */
    }

  if (ret >= 0)
    {
      memset(&iwr, 0, sizeof(iwr));
      wifi_ifname(iwr.ifr_name);
      iwr.u.essid.pointer = (FAR void *)ssid;
      iwr.u.essid.length  = (uint16_t)strlen(ssid);
      iwr.u.essid.flags   = IW_ESSID_ON;              /* set + connect */
      ret = psock_ioctl(&s, SIOCSIWESSID, (unsigned long)(uintptr_t)&iwr);
    }

  psock_close(&s);
  return ret;
}

static char g_wifi_join_ssid[24];

static void wifi_connecting_compose(void)
{
  bg_compose(); statusbar_compose();
  gfx_fill_round(20, 44, 200, 48, 10, COL_SEL_BG);
  gfx_round(20, 44, 200, 48, 10, COL_ACCENT);
  gfx_wifi_glyph(42, 76, 3, COL_ACCENT, COL_DIM);
  gtext(62, 50, "Connecting", COL_SEL_FG);
  gtext(62, 68, g_wifi_join_ssid, COL_FG);
}

/* Join the selected AP with w->pass.  from_saved: the passphrase came from
 * flash -- on failure drop into the prompt so it can be retyped.
 * On success the passphrase is (re)saved, then DHCP runs in the terminal.
 */

static void wifi_connect(FAR struct wifi_ui_s *w, bool from_saved)
{
  FAR struct wifi_ap_s *ap = &w->ap[w->sel];
  char line[64];
  int  ret;

  w->msg[0] = '\0';
  if (w->passlen > 0 && w->passlen < 8)
    {
      strlcpy(w->msg, "need 8..63 chars", sizeof(w->msg));
      wifi_render();
      return;
    }

  snprintf(g_wifi_join_ssid, sizeof(g_wifi_join_ssid), "%.22s", ap->ssid);
  ui_render(wifi_connecting_compose);

  int sret = 0;
  ret = wifi_join(ap->ssid, w->pass, w->passlen);
  if (ret >= 0 && !from_saved)
    {
      sret = wstore_remember(ap->ssid, w->pass, w->passlen);
      if (sret < 0) syslog(LOG_WARNING, "wifi: save failed %d\n", sret);
    }

  memset(w->pass, 0, sizeof(w->pass));
  w->passlen = 0;

  if (ret < 0)
    {
      wifi_status_refresh(w);
      if (from_saved)
        {
          w->state = WIFI_ST_PASS;              /* saved key rejected */
          w->show_pass = false;
          snprintf(w->msg, sizeof(w->msg), "saved key failed %d", ret);
        }
      else
        {
          w->state = WIFI_ST_LIST;
          snprintf(w->msg, sizeof(w->msg), "join failed (%d)", ret);
        }
      wifi_render();
      return;
    }

  /* Associated.  Get an address with the stock DHCP client in the
   * terminal so the user sees the result.  No secrets are typed there.
   */

  snprintf(line, sizeof(line), sret < 0 ? "[wifi] joined %.22s (not saved)\r\n"
                                        : "[wifi] joined %.22s\r\n", ap->ssid);
  wifi_free();                                   /* w, ap invalid now */
  ui_enter_mode(UI_MODE_TERMINAL);
  kbd_echo_str(line);
  ringdev_puts(&g_kbd_ring, "renew " WIFI_IFNAME "\n");
}

static void wifi_handle(int navkey, char ch, char base)
{
  FAR struct wifi_ui_s *w = g_wifi;
  if (w == NULL) return;

  if (w->state == WIFI_ST_LIST)
    {
      if (navkey == NAV_NONE && (base == 'i' || base == ';')) navkey = NAV_UP;
      if (navkey == NAV_NONE && (base == 'k' || base == '.')) navkey = NAV_DOWN;

      if (navkey == NAV_UP && w->nap)
        w->sel = (w->sel + w->nap - 1) % w->nap;
      else if (navkey == NAV_DOWN && w->nap)
        w->sel = (w->sel + 1) % w->nap;
      else if (ch == 's' || ch == 'S')
        { wifi_scan(w); return; }
      else if (ch == 'r' || ch == 'R')
        wifi_status_refresh(w);
      else if ((ch == '\r' || ch == '\n') && w->nap)
        {
          w->show_pass = false; w->msg[0] = '\0';
          if (w->ap[w->sel].saved &&
              wstore_find(w->ap[w->sel].ssid, w->pass, &w->passlen))
            {
              wifi_connect(w, true);            /* no prompt needed */
              return;
            }

          w->state = WIFI_ST_PASS;
          w->passlen = 0; w->pass[0] = '\0';
        }
      else if ((ch == 'f' || ch == 'F') && w->nap && w->ap[w->sel].saved)
        {
          int ret = wstore_forget(w->ap[w->sel].ssid);
          if (ret >= 0)
            {
              w->ap[w->sel].saved = 0;
              snprintf(w->msg, sizeof(w->msg), "forgot %.16s",
                       w->ap[w->sel].ssid);
            }
          else
            {
              snprintf(w->msg, sizeof(w->msg), "forget err %d", ret);
            }
        }
      else
        return;

      if (w->sel < w->top) w->top = w->sel;
      if (w->sel >= w->top + WIFI_VISIBLE) w->top = w->sel - WIFI_VISIBLE + 1;
      wifi_render();
      return;
    }

  /* WIFI_ST_PASS */

  if (navkey == NAV_LEFT)
    {
      memset(w->pass, 0, sizeof(w->pass));
      w->passlen = 0; w->state = WIFI_ST_LIST; w->msg[0] = '\0';
    }
  else if (navkey != NAV_NONE)
    {
      return;
    }
  else if (ch == '\r' || ch == '\n')
    {
      wifi_connect(w, false);
      return;
    }
  else if (ch == (char)KEY_BACKSPACE)
    {
      if (w->passlen) w->pass[--w->passlen] = '\0';
    }
  else if (ch == '\t')
    {
      w->show_pass = !w->show_pass;
    }
  else if (ch >= 0x20 && (uint8_t)ch < 0x7f && w->passlen < WIFI_PASS_MAX)
    {
      w->pass[w->passlen++] = ch;
      w->pass[w->passlen] = '\0';
    }
  else
    {
      return;
    }

  ui_render_rows(wifi_compose, 46, 118);
}

static void wifi_tick(void)
{
  FAR struct wifi_ui_s *w = g_wifi;
  if (w == NULL || w->state != WIFI_ST_LIST) return;
  wifi_status_refresh(w);
  ui_render_rows(wifi_compose, 16, 51);                /* status card */
}

#endif /* CARDPUTER_WIFI */

/****************************************************************************
 * World Clock app + NTP
 *
 *   Big HH:MM:SS of the home city, date, then every city in g_cities[].
 *   UP/DN = select   ENTER = make it the home city   n = sync with NTP
 *
 *   NTP is a single SNTP request over a kernel UDP socket (no ntpc daemon,
 *   no extra task, ~100 B of stack).  pool.ntp.org is tried via DNS first,
 *   then two anycast IPs in case DNS isn't configured yet.  The result is
 *   written with clock_settime(CLOCK_REALTIME); every app, including NSH's
 *   `date`, then sees the correct time until reboot.
 ****************************************************************************/

#define CLK_LIST_Y0    70
#define CLK_ROW_H      16
#define CLK_VISIBLE    3

static uint8_t g_clk_sel = 0, g_clk_top = 0;
static int     g_ntp_state = 1;     /* 1 never, 0 ok, 2 running, <0 error */

#ifdef CARDPUTER_WIFI

#define NTP_UNIX_OFFSET 2208988800u          /* 1900 -> 1970 */

static uint32_t be32(FAR const uint8_t *p)
{
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
         ((uint32_t)p[2] << 8) | p[3];
}

/* Parse a 48-byte SNTP server reply into Unix time. */

static int ntp_parse(FAR const uint8_t *pkt, size_t len,
                     FAR struct timespec *ts)
{
  if (len < 48) return -EMSGSIZE;
  if ((pkt[0] & 7) != 4 || pkt[1] == 0 || pkt[1] > 15) return -EPROTO;
  uint32_t sec  = be32(pkt + 40);              /* transmit timestamp */
  uint32_t frac = be32(pkt + 44);
  if (sec < NTP_UNIX_OFFSET) return -ERANGE;
  ts->tv_sec  = (time_t)(sec - NTP_UNIX_OFFSET);
  ts->tv_nsec = (long)(((uint64_t)frac * 1000000000u) >> 32);
  return OK;
}

static int ntp_query(FAR const struct sockaddr_in *sa,
                     FAR struct timespec *ts)
{
  struct socket s;
  struct timeval tv = { 2, 0 };
  uint8_t pkt[48];
  ssize_t n;
  int ret;

  ret = psock_socket(AF_INET, SOCK_DGRAM, 0, &s);
  if (ret < 0) return ret;

  psock_setsockopt(&s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  memset(pkt, 0, sizeof(pkt));
  pkt[0] = 0x23;                               /* LI 0, VN 4, mode 3 */
  n = psock_sendto(&s, pkt, sizeof(pkt), 0,
                   (FAR const struct sockaddr *)sa, sizeof(*sa));
  if (n >= 0)
    n = psock_recvfrom(&s, pkt, sizeof(pkt), 0, NULL, NULL);

  psock_close(&s);
  if (n < 0) return (int)n;
  return ntp_parse(pkt, (size_t)n, ts);
}

static int ntp_sync(void)
{
  static const uint32_t fallback[] =
  {
    0xd8ef2300u,          /* time.google.com     216.239.35.0  */
    0xa29fc801u,          /* time.cloudflare.com 162.159.200.1 */
  };

  struct sockaddr_in sa;
  struct timespec ts;
  int ret = -EHOSTUNREACH;

  memset(&sa, 0, sizeof(sa));
  sa.sin_family = AF_INET;
  sa.sin_port   = htons(123);

#ifdef CONFIG_LIBC_NETDB
  {
    struct addrinfo hints;
    FAR struct addrinfo *res = NULL;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo("pool.ntp.org", NULL, &hints, &res) == 0 && res != NULL)
      {
        sa.sin_addr = ((FAR struct sockaddr_in *)res->ai_addr)->sin_addr;
        ret = ntp_query(&sa, &ts);
        freeaddrinfo(res);
      }
  }
#endif

  for (size_t i = 0; ret < 0 && i < sizeof(fallback) / sizeof(fallback[0]);
       i++)
    {
      sa.sin_addr.s_addr = htonl(fallback[i]);
      ret = ntp_query(&sa, &ts);
    }

  if (ret >= 0 && clock_settime(CLOCK_REALTIME, &ts) < 0) ret = -errno;
  return ret;
}

static bool net_link_running(void)
{
  struct socket s;
  struct ifreq ifr;
  bool up = false;

  if (wifi_sock(&s) < 0) return false;
  memset(&ifr, 0, sizeof(ifr));
  wifi_ifname(ifr.ifr_name);
  if (psock_ioctl(&s, SIOCGIFFLAGS, (unsigned long)(uintptr_t)&ifr) >= 0)
    up = (ifr.ifr_flags & (IFF_UP | IFF_RUNNING)) == (IFF_UP | IFF_RUNNING);
  psock_close(&s);
  return up;
}

#ifdef CARDPUTER_WIFI_STORE

/* Boot-time auto connect.  Runs once in its own short-lived kernel thread
 * (stack freed when it returns), so the keyboard stays responsive:
 *   1. try the saved networks, most recently used first
 *   2. ask NSH's stock DHCP client for a lease (`renew wlan0`)
 *   3. once an address arrives, set the clock over NTP
 */

#ifndef CARDPUTER_WIFI_AUTO_DELAY_MS
#  define CARDPUTER_WIFI_AUTO_DELAY_MS 3000  /* let NSH + netinit start */
#endif

static bool wifi_has_lease(void)
{
  struct socket s;
  struct ifreq ifr;
  uint32_t a = 0;

  if (!net_link_running() || wifi_sock(&s) < 0) return false;
  memset(&ifr, 0, sizeof(ifr));
  wifi_ifname(ifr.ifr_name);
  if (psock_ioctl(&s, SIOCGIFADDR, (unsigned long)(uintptr_t)&ifr) >= 0)
    a = ntohl(((FAR struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr);
  psock_close(&s);

#if defined(CONFIG_NETINIT_NETLOCAL) && defined(CONFIG_NETINIT_IPADDR)
  if (a == (uint32_t)CONFIG_NETINIT_IPADDR) return false;  /* static default */
#endif
  return a != 0;
}

static int wifi_auto_thread(int argc, FAR char *argv[])
{
  FAR struct wifi_store_s *st;
  int ret = -ENOENT;

  UNUSED(argc); UNUSED(argv);
  usleep(CARDPUTER_WIFI_AUTO_DELAY_MS * 1000);

  st = kmm_malloc(sizeof(*st));
  if (st == NULL) return -ENOMEM;

  if (wstore_load(st) > 0)
    {
      wifi_set_up();
      for (int i = 0; i < st->count && ret < 0; i++)
        {
          ret = wifi_join(st->net[i].ssid, st->net[i].pass,
                          st->net[i].passlen);
          syslog(LOG_INFO, "wifi: auto-join %s: %d\n", st->net[i].ssid, ret);
        }
    }

  memset(st, 0, sizeof(*st));
  kmm_free(st);
  if (ret < 0) return ret;

  ringdev_puts(&g_kbd_ring, "renew " WIFI_IFNAME "\n");

  for (int t = 0; t < 40; t++)                  /* up to 20 s for a lease */
    {
      usleep(500 * 1000);
      if (wifi_has_lease())
        {
          g_ntp_state = 2;
          g_ntp_state = ntp_sync() >= 0 ? 0 : -EIO;
          break;
        }
    }

  return OK;
}

#endif /* CARDPUTER_WIFI_STORE */

#endif /* CARDPUTER_WIFI */

static void clock_compose(void)
{
  int64_t now = utc_now();
  struct ltime_s home, lt;
  char line[32];

  bg_compose(); statusbar_compose();

  if (now >= 0)
    {
      city_localtime(g_home_city, now, &home);
      snprintf(line, sizeof(line), "%02d:%02d:%02d", home.h, home.mi, home.s);
      gtext2x_center(LCD_W / 2, 18, line, COL_SEL_FG);
      snprintf(line, sizeof(line), "%s %04d-%02d-%02d",
               g_wday[home.wday], home.y, home.mo, home.d);
      gtext(8, 52, line, COL_FG);
    }
  else
    {
      gtext2x_center(LCD_W / 2, 18, "--:--:--", COL_DIM);
      gtext(8, 52, "time not set", COL_DIM);
    }
  gtext_right(LCD_W - 8, 52, g_cities[g_home_city].name, COL_ACCENT);
  gfx_hline(4, 68, LCD_W - 8, COL_SEL_BG);

  for (int i = 0; i < CLK_VISIBLE; i++)
    {
      int idx = g_clk_top + i, y = CLK_LIST_Y0 + i * CLK_ROW_H;
      if (idx >= CITY_COUNT || !gfx_band_hits(y, CLK_ROW_H)) continue;
      bool sel = (idx == g_clk_sel);
      if (sel)
        {
          gfx_fill_round(4, y, 232, CLK_ROW_H - 1, 5, COL_SEL_BG);
          gfx_fill_round(6, y + 2, 3, CLK_ROW_H - 5, 1, COL_MARKER);
        }
      uint16_t fg = sel ? COL_SEL_FG : COL_FG;
      gtext(14, y, g_cities[idx].name, idx == g_home_city ? COL_ACCENT : fg);

      if (now < 0)
        {
          gtext(110, y, "--:--", COL_DIM);
          continue;
        }

      city_localtime(idx, now, &lt);
      snprintf(line, sizeof(line), "%02d:%02d", lt.h, lt.mi);
      gtext(110, y, line, fg);
      if (lt.days != home.days) gtext(154, y, g_wday[lt.wday], COL_DIM);

      int32_t diff = city_offset(idx, now) - city_offset(g_home_city, now);
      int32_t ad = diff < 0 ? -diff : diff;
      if (diff == 0)
        strlcpy(line, "0", sizeof(line));
      else if (ad % 3600)
        snprintf(line, sizeof(line), "%c%d:%02d", diff < 0 ? '-' : '+',
                 (int)(ad / 3600), (int)(ad % 3600 / 60));
      else
        snprintf(line, sizeof(line), "%+dh", (int)(diff / 3600));
      gtext_right(LCD_W - 8, y, line, COL_DIM);
    }

  FAR const char *st =
      g_ntp_state == 0 ? "NTP ok"  :
      g_ntp_state == 2 ? "sync.."  :
      g_ntp_state == 1 ? "no NTP"  :
      g_ntp_state == -ENETDOWN ? "no WiFi" : "NTP err";
  footer_compose("n=sync ok=home");
  gtext_right(LCD_W - 6, 119, st, g_ntp_state == 0 ? COL_OK : COL_WARN);
}

static void clock_render(void) { ui_render(clock_compose); }

static void clock_sync(void)
{
#ifdef CARDPUTER_WIFI
  if (!net_link_running())
    {
      g_ntp_state = -ENETDOWN;
      clock_render();
      return;
    }

  g_ntp_state = 2;
  clock_render();
  g_ntp_state = ntp_sync() >= 0 ? 0 : -EIO;
#else
  g_ntp_state = -ENOSYS;
#endif
  clock_render();
}

static void clock_enter(void)
{
  clock_render();
  if (utc_now() < 0) clock_sync();          /* first visit: get the time */
}

static void clock_handle(int navkey, char ch)
{
  if (navkey == NAV_UP)
    g_clk_sel = (uint8_t)((g_clk_sel + CITY_COUNT - 1) % CITY_COUNT);
  else if (navkey == NAV_DOWN)
    g_clk_sel = (uint8_t)((g_clk_sel + 1) % CITY_COUNT);
  else if (ch == '\r' || ch == '\n')
    g_home_city = g_clk_sel;
  else if (ch == 'n' || ch == 'N')
    { clock_sync(); return; }
  else
    return;

  if (g_clk_sel < g_clk_top) g_clk_top = g_clk_sel;
  if (g_clk_sel >= g_clk_top + CLK_VISIBLE)
    g_clk_top = (uint8_t)(g_clk_sel - CLK_VISIBLE + 1);
  clock_render();
}

/****************************************************************************
 * Mode switch
 ****************************************************************************/

static void ui_leave_mode(int mode)
{
#ifdef CARDPUTER_WIFI
  if (mode == UI_MODE_WIFI) wifi_free();
#else
  UNUSED(mode);
#endif
}

static void ui_enter_mode(int mode)
{
  ui_leave_mode(g_ui_mode);
  g_ui_mode = mode;
  switch (mode)
    {
      case UI_MODE_SCREENTEST:
        g_st_color_idx=0; ui_render(screentest_compose); break;
      case UI_MODE_KEYBOARDTEST:
        keyboardtest_enter(); break;
      case UI_MODE_FILEMANAGER:
        filemanager_enter(); break;
      case UI_MODE_BATTERY:
        battery_render(); break;
#ifdef CARDPUTER_WIFI
      case UI_MODE_WIFI:
        wifi_enter(); break;
#endif
      case UI_MODE_CLOCK:
        clock_enter(); break;
      case UI_MODE_TERMINAL:
        con_clear(); statusbar_render();
        ringdev_putc(&g_kbd_ring,'\n'); break;
      default: break;
    }
}

static void ui_back_to_menu(void)
{
  ui_leave_mode(g_ui_mode);
  g_ui_mode = UI_MODE_MENU;
  g_scroll_offset = 0;
  menu_render();
}

static void menu_handle(int navkey, char ch)
{
  if      (navkey==NAV_UP)   { g_menu_sel=(g_menu_sel+MENU_COUNT-1)%MENU_COUNT; menu_render(); }
  else if (navkey==NAV_DOWN) { g_menu_sel=(g_menu_sel+1)%MENU_COUNT; menu_render(); }
  else if (ch=='\r'||ch=='\n') ui_enter_mode(g_menu_items[g_menu_sel].mode);
}

/****************************************************************************
 * kbd_nsh_thread
 ****************************************************************************/

static void kbd_send_vt100(FAR const char *seq) { ringdev_puts(&g_kbd_ring, seq); }
static void kbd_echo(char c) { ringdev_tryputc(&g_out_ring,c); }
static void kbd_echo_str(FAR const char *s) { for (; *s; s++) kbd_echo(*s); }

static inline uint32_t uptime_ms(void)
{
  return (uint32_t)TICK2MSEC(clock_systime_ticks());
}

static inline int nav_from_base(char base)
{
  if (base=='i'||base==';') return NAV_UP;
  if (base=='k'||base=='.') return NAV_DOWN;
  return NAV_NONE;
}

static int kbd_nsh_thread(int argc, FAR char *argv[])
{
  uint8_t reg_val;
  bool shift_pressed=false, ctrl_pressed=false, fn_pressed=false;

#define KBD_COOLDOWN_MS 100
#define KBD_POLL_US     20000
  uint32_t last_key_ms=0;
  uint8_t  last_key_raw=0;
  uint32_t bar_last_ms=0, bat_last_ms=0, wifi_last_ms=0, clk_last_ms=0;

  UNUSED(argc); UNUSED(argv);

  esp_configgpio(44,OUTPUT); esp_gpiowrite(44,1);
  esp_configgpio(11,INPUT|PULLUP);
  usleep(300 * 1000);

  /* Keep trying: giving up here left the keyboard dead for good. */

  for (int tries = 0; ; tries++)
    {
      soft_i2c_recover();
      if (soft_tca8418_write(0x01,0x80)) break;
      if (tries == 0) BTRACE("kbd: no TCA8418, retrying\n");
      usleep(500 * 1000);
    }
  BTRACE("kbd: TCA8418 ok\n");

  /* Another firmware (M5 Launcher) may have configured the chip; it keeps
   * its registers across our reset.  Return the GPIO/event registers to
   * their power-on values (0) before applying our setup.
   */

  for (uint8_t r=0x1A;r<=0x2E;r++) soft_tca8418_write(r,0x00);
  for (int i=0;i<16;i++)
    {
      if (!soft_tca8418_read(0x03,&reg_val) || (reg_val&0x0F)==0) break;
      soft_tca8418_read(0x04,&reg_val);
    }
  soft_tca8418_write(0x02,0x1F);

  soft_tca8418_write(0x11,0xFF); soft_tca8418_write(0x12,0xFF);
  soft_tca8418_write(0x13,0x03); soft_tca8418_write(0x1D,0xFF);
  soft_tca8418_write(0x1E,0xFF); soft_tca8418_write(0x29,0xFF);
  soft_tca8418_write(0x2A,0xFF); soft_tca8418_write(0x2B,0x03);
  soft_tca8418_read(0x04,&reg_val);
  soft_tca8418_read(0x04,&reg_val);
  soft_tca8418_write(0x01,0x21);
  soft_tca8418_write(0x02,0x0F);

  while (1)
    {
      uint32_t now = uptime_ms();

      if (soft_tca8418_read(0x03,&reg_val))
        {
          uint8_t event_count = reg_val & 0x0F;

          for (; event_count > 0; event_count--)
            {
              if (!soft_tca8418_read(0x04,&reg_val)) continue;

              bool    is_pressed = (reg_val & 0x80) != 0;
              uint8_t raw_id     = reg_val & 0x7F;
              if (raw_id==0) continue;

              uint8_t buf_id=raw_id-1;
              uint8_t raw_row=buf_id/10, raw_col=buf_id%10;
              uint8_t mapped_col=(uint8_t)(raw_row<<1);
              if (raw_col>3) mapped_col++;
              uint8_t mapped_row=(uint8_t)((raw_col+4)&3);
              if (mapped_col>=14||mapped_row>=4) continue;

              FAR const struct KeyValue_t *kv=&g_key_map[mapped_row][mapped_col];
              char    base=kv->value_first;
              uint8_t id=(uint8_t)base;

              /* --- Modifier latches --- */
              if (id==KEY_LEFT_SHIFT) { shift_pressed=is_pressed; continue; }
              if (id==KEY_LEFT_CTRL)  { ctrl_pressed =is_pressed; continue; }
              if (id==KEY_FN)         { fn_pressed   =is_pressed; continue; }
              if (id==(uint8_t)KEY_OPT) { if (is_pressed) ui_back_to_menu(); continue; }
              if (id==KEY_LEFT_ALT || !is_pressed) continue;

              /* --- Cooldown (same key within 100 ms) --- */
              if (raw_id==last_key_raw && (now-last_key_ms)<KBD_COOLDOWN_MS)
                continue;
              last_key_ms=now; last_key_raw=raw_id;

              /* --- Decode navkey (fn layer) and/or character --- */
              int  navkey = NAV_NONE;
              char ch     = 0;

              if (fn_pressed)
                {
                  switch (base)
                    {
                      case 'i': case ';': navkey=NAV_UP;    break;
                      case 'k': case '.': navkey=NAV_DOWN;  break;
                      case 'j':           navkey=NAV_LEFT;  break;
                      case 'l':           navkey=NAV_RIGHT; break;
                      default: break;
                    }
                }

              if (navkey==NAV_NONE)
                {
                  if (ctrl_pressed)
                    ch=(base>='a'&&base<='z') ? (char)(base-'a'+1) : base;
                  else
                    ch=shift_pressed ? kv->value_second : base;
                  if ((uint8_t)ch>=0x80) continue;
                }

              /* --- Mode dispatch --- */
              switch (g_ui_mode)
                {
                  case UI_MODE_MENU:
                    if (navkey==NAV_NONE) navkey=nav_from_base(base);
                    menu_handle(navkey, ch);
                    break;

                  case UI_MODE_TERMINAL:
                    if (fn_pressed && navkey==NAV_UP)        term_scroll_up();
                    else if (fn_pressed && navkey==NAV_DOWN) term_scroll_down();
                    else
                      {
                        term_snap_live();
                        if      (navkey==NAV_UP)    kbd_send_vt100("\x1b[A");
                        else if (navkey==NAV_DOWN)  kbd_send_vt100("\x1b[B");
                        else if (navkey==NAV_LEFT)  kbd_send_vt100("\x1b[D");
                        else if (navkey==NAV_RIGHT) kbd_send_vt100("\x1b[C");
                        else if (ch=='\r')
                          { kbd_echo('\r'); kbd_echo('\n'); ringdev_putc(&g_kbd_ring,'\n'); }
                        else if (ch==(char)KEY_BACKSPACE || ch=='\t' ||
                                 ((uint8_t)ch<0x20 && ch!=0))
                          ringdev_putc(&g_kbd_ring,ch);
                        else if (ch!=0)
                          { kbd_echo(ch); ringdev_putc(&g_kbd_ring,ch); }
                      }
                    break;

                  case UI_MODE_SCREENTEST:
                    if (navkey==NAV_NONE) navkey=nav_from_base(base);
                    screentest_handle(navkey);
                    break;

                  case UI_MODE_KEYBOARDTEST:
                    keyboardtest_show(ch,raw_id,navkey,
                                      shift_pressed,ctrl_pressed,fn_pressed);
                    break;

                  case UI_MODE_FILEMANAGER:
                    if (navkey==NAV_NONE) navkey=nav_from_base(base);
                    filemanager_handle(navkey,ch);
                    break;

#ifdef CARDPUTER_WIFI
                  case UI_MODE_WIFI:
                    wifi_handle(navkey, ch, base);
                    break;
#endif

                  case UI_MODE_CLOCK:
                    if (navkey==NAV_NONE) navkey=nav_from_base(base);
                    clock_handle(navkey, ch);
                    break;

                  case UI_MODE_BATTERY:
                  default: break;
                }
            }

          soft_tca8418_write(0x02,0x0F);
        }

      /* --- Periodic work --- */

      now = uptime_ms();

      if ((now-bar_last_ms) >= 2000)
        {
          bar_last_ms=now;
          battery_refresh();
          if (g_ui_mode != UI_MODE_SCREENTEST && g_scroll_offset == 0)
            statusbar_render();
        }

      if (g_ui_mode==UI_MODE_BATTERY && (now-bat_last_ms) >= 1000)
        { bat_last_ms=now; battery_render(); }

      if (g_ui_mode==UI_MODE_CLOCK && (now-clk_last_ms) >= 1000)
        { clk_last_ms=now; ui_render_rows(clock_compose, 16, 134); }

#ifdef CARDPUTER_WIFI
      if (g_ui_mode==UI_MODE_WIFI && (now-wifi_last_ms) >= WIFI_STATUS_MS)
        { wifi_last_ms=now; wifi_tick(); }
#else
      UNUSED(wifi_last_ms);
#endif

      usleep(KBD_POLL_US);       /* sleep — do NOT busy-wait (up_mdelay) */
    }

  return OK;
}

/****************************************************************************
 * lcd_cons_thread — VT100 parser → scrollback
 *
 *   ESC [ n J / H  erase display     ESC [ n K  erase to end of line
 *   ESC [ n A / B  cursor up / down  ESC [ n C / D  cursor right / left
 *   0x0C (FF)      clear screen      ESC [ … m  SGR (ignored)
 ****************************************************************************/

#define ANSI_ESC     0x1b
#define ANSI_BUF_MAX 16

static int lcd_cons_thread(int argc, FAR char *argv[])
{
  uint8_t ansi_buf[ANSI_BUF_MAX];
  int     ansi_len  = 0;
  bool    in_escape = false;

  UNUSED(argc); UNUSED(argv);

  BTRACE("lcd_cons: first frame\n");
#ifdef CONFIG_DEBUG_FEATURES
  usleep(3000 * 1000);                       /* leave the boot log readable */
#endif
  boot_screen_end();
  battery_refresh();
  con_clear_full();
  menu_render();
  BTRACE("lcd_cons: menu drawn\n");

  while (1)
    {
      char c = ringdev_getc(&g_out_ring);

      if (g_ui_mode != UI_MODE_TERMINAL)
        { in_escape=false; ansi_len=0; continue; }

      if (in_escape)
        {
          if (ansi_len < ANSI_BUF_MAX-1) ansi_buf[ansi_len++]=(uint8_t)c;
          if ((c>='A'&&c<='Z')||(c>='a'&&c<='z')||c=='~')
            {
              in_escape=false;
              int arg=0;
              for (int i=1;i<ansi_len-1;i++)
                if (ansi_buf[i]>='0'&&ansi_buf[i]<='9')
                  arg=arg*10+(ansi_buf[i]-'0');
              if (arg==0) arg=1;

              switch (c)
                {
                  case 'J': case 'H':
                    con_clear(); break;

                  case 'K':
                    nxmutex_lock(&g_con_mutex);
                    if (g_sb_col < COLS)
                      memset(&SB_CH(g_sb_total, g_sb_col), ' ', COLS - g_sb_col);
                    if (g_scroll_offset==0) term_flush_live_locked();
                    nxmutex_unlock(&g_con_mutex);
                    break;

                  case 'D':
                    nxmutex_lock(&g_con_mutex);
                    g_sb_col = (g_sb_col > arg) ? g_sb_col - arg : 0;
                    nxmutex_unlock(&g_con_mutex);
                    break;

                  case 'C':
                    nxmutex_lock(&g_con_mutex);
                    g_sb_col += arg;
                    if (g_sb_col > COLS-1) g_sb_col = COLS-1;
                    nxmutex_unlock(&g_con_mutex);
                    break;

                  case 'A':
                    nxmutex_lock(&g_con_mutex);
                    g_sb_total = (g_sb_total > arg) ? g_sb_total - arg : 0;
                    g_sb_col=0;
                    nxmutex_unlock(&g_con_mutex);
                    break;

                  case 'B':
                    nxmutex_lock(&g_con_mutex);
                    for (int i=0;i<arg;i++) sb_newline_locked();
                    nxmutex_unlock(&g_con_mutex);
                    break;

                  default: break;          /* 'm' (SGR) and others */
                }
              ansi_len=0;
            }
          continue;
        }

      if (c==ANSI_ESC)
        { in_escape=true; ansi_len=0; ansi_buf[ansi_len++]=(uint8_t)c; continue; }
      if (c==0x0c) { con_clear(); continue; }

      con_putc(c);
    }

  return OK;
}

/****************************************************************************
 * board_reclaim_gpios
 *
 *   When another firmware starts us with a software reset -- M5 Launcher
 *   does exactly that -- the GPIO matrix is NOT reset.  Launcher leaves:
 *     GPIO38  backlight on LEDC PWM, duty 0 (it dims to 0 before rebooting)
 *     GPIO8/9 keyboard bus on the hardware I2C peripheral (Wire)
 *     GPIO11  a falling-edge GPIO interrupt (TCA8418 INT)
 *   esp_configgpio() selects the GPIO function but keeps the old output
 *   signal, so esp_gpiowrite() would have no effect: black screen, dead
 *   keyboard, while NuttX itself runs fine.  Route every pin we drive by
 *   hand back to the plain GPIO output register and drop the interrupt.
 ****************************************************************************/

#ifndef SIG_GPIO_OUT_IDX
#  define SIG_GPIO_OUT_IDX 256               /* ESP32-S3 "simple GPIO" */
#endif

static void board_reclaim_gpios(void)
{
  static const uint8_t pins[] = { 8, 9, 11, 33, 34, 38, 44 };

  for (size_t i = 0; i < sizeof(pins); i++)
    esp_gpio_matrix_out(pins[i], SIG_GPIO_OUT_IDX, false, false);

  esp_configgpio(11, INPUT | PULLUP);        /* also clears the INT type */
}

/****************************************************************************
 * board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  extern FAR struct spi_dev_s *esp32s3_spibus_initialize(int port);
  FAR struct spi_dev_s *spi;
  if (g_lcddev) return OK;

  esp_configgpio(38,OUTPUT); esp_gpiowrite(38,1);
  esp_configgpio(33,OUTPUT); esp_gpiowrite(33,0);
  up_mdelay(50); esp_gpiowrite(33,1); up_mdelay(120);
  esp_configgpio(34,OUTPUT); esp_gpiowrite(34,1);

  spi=esp32s3_spibus_initialize(2);
  if (!spi) return -ENODEV;

  g_lcddev=st7789_lcdinitialize(spi);
  if (!g_lcddev) return -ENODEV;

  if (g_lcddev->setpower)
    g_lcddev->setpower(g_lcddev,CONFIG_LCD_MAXPOWER);

  if (gfx_init()<0)
    {
      syslog(LOG_ERR,"gfx_init failed: no putarea/putrun on LCD plane\n");
      return -ENOSYS;
    }
  return OK;
}

/****************************************************************************
 * esp32s3_bringup
 ****************************************************************************/

int esp32s3_bringup(void)
{
  int ret;
#if defined(CONFIG_LCD)
  int lcd_ret;
#endif

#ifdef CARDPUTER_USB_SYSLOG
  syslog_channel_register(&g_usj_syslog_channel);  /* needs MAX_CHANNELS 4 */
#endif
#ifdef CARDPUTER_PANIC_SCREEN
  panic_notifier_chain_register(&g_panic_nb);
#endif
  BTRACE("bringup start\n");
  board_reclaim_gpios();                     /* before any esp_gpiowrite() */
  BTRACE("gpios reclaimed\n");

#ifdef CONFIG_FS_PROCFS
  nx_mount(NULL,"/proc","procfs",0,NULL);
#endif

  /* LCD first, so every later step shows up on the boot screen. */

#if defined(CONFIG_LCD)
  lcd_ret=board_lcd_initialize();
  BTRACE("lcd init %d\n", lcd_ret);
#endif

#ifdef CONFIG_ESP32S3_SPIFLASH
  BTRACE("spiflash init...\n");
  ret=board_spiflash_init();
  if (ret<0) syslog(LOG_ERR,"board_spiflash_init failed: %d\n",ret);
#endif

#ifdef CONFIG_ESPRESSIF_HR_TIMER
  /* The Wi-Fi PHY creates esp_timers on first RF enable; they fail with
   * ESP_ERR_INVALID_STATE (259) unless the HR timer subsystem is up.
   * NuttX before 2026-08 (commit 3d30272ce) leaves this to the board.
   * Idempotent, so harmless on newer trees that also call it.
   */

  ret=esp_hr_timer_init();
  if (ret<0) syslog(LOG_ERR,"esp_hr_timer_init failed: %d\n",ret);
#endif

#ifdef CONFIG_ESPRESSIF_WIFI
  /* esp_wifi_init() wants a large contiguous heap block: still early, the
   * LCD driver above only allocated a few bytes.
   */

  BTRACE("wifi init...\n");
  ret=board_wlan_init();
  g_wlan_init_ret=ret;
  BTRACE("wifi init %d\n", ret);
  if (ret<0) syslog(LOG_ERR,"board_wlan_init failed: %d\n",ret);
#endif

#if defined(CONFIG_LCD)
  if (lcd_ret<0) return lcd_ret;

  ringdev_init(&g_kbd_ring, g_kbd_buf, sizeof(g_kbd_buf));
  ringdev_init(&g_out_ring, g_out_buf, sizeof(g_out_buf));
  g_out_ring_ready=true;

#ifdef CONFIG_SYSLOG
  /* Needs CONFIG_SYSLOG_MAX_CHANNELS >= 2 to coexist with the console. */

  if (syslog_channel_register(&g_lcd_syslog_channel) < 0)
    syslog(LOG_WARNING,"LCD syslog mirror not registered\n");
#endif

#if defined(CONFIG_ESPRESSIF_ADC)||defined(CONFIG_ESP32S3_ADC)
  BTRACE("adc init...\n");
  ret=board_adc_init();
  if (ret<0) syslog(LOG_ERR,"board_adc_init failed: %d\n",ret);
#endif

  ret=register_driver("/dev/cardputer_kbd",&g_kbd_ops,0666,&g_kbd_ring);
  if (ret<0) return ret;
  ret=register_driver("/dev/cardputer_out",&g_out_ops,0666,&g_out_ring);
  if (ret<0) return ret;

  kthread_create("lcd_cons",110,CARDPUTER_LCDCONS_STACK,lcd_cons_thread,NULL);
  kthread_create("kbd_nsh", 100,CARDPUTER_KBD_STACK,   kbd_nsh_thread, NULL);
  BTRACE("threads started\n");

#ifdef CARDPUTER_WIFI_STORE
  /* One-shot: reconnect to a saved network, DHCP, NTP; then exits. */

  if (g_wlan_init_ret == 0)
    kthread_create("wifi_auto", 100, 4096, wifi_auto_thread, NULL);
#endif
#endif

  UNUSED(ret);
  return OK;
}

struct lcd_dev_s *board_lcd_getdev(int lcddev) { UNUSED(lcddev); return g_lcddev; }
int up_fbinitialize(int display)               { UNUSED(display); return OK; }
