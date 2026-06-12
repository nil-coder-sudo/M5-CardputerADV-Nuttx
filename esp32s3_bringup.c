/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-devkit/src/esp32s3_bringup.c
 *
 * M5 Cardputer — Hacker Green UI
 *
 *   Fixed phosphor-green theme (#00FF41).
 *
 *   Performance model
 *   -----------------
 *   Normal terminal output (con_putc):
 *     • Mutates the scrollback store.
 *     • Flushes only the single dirty row via gfx_flush_band()  ← fast path
 *     • gfx_present() (full-screen SPI) is NEVER called on the hot path.
 *
 *   Scroll repaint (term_repaint_locked):
 *     • Redraws all TERM_ROWS into the framebuffer then calls gfx_present().
 *     • Only triggered by fn+UP / fn+DOWN keypresses.
 *
 *   Newline / scroll-up (sb_scroll_screen_locked):
 *     • Blits TERM_ROWS rows then calls gfx_flush_band(terminal area).
 *     • Does NOT call gfx_present().
 *
 *   Key routing
 *   -----------
 *   UI_MODE_MENU
 *     UP / DOWN          → menu navigation  (NO fn required)
 *     fn + UP/DOWN       → same (fn is irrelevant in menu)
 *
 *   UI_MODE_TERMINAL
 *     fn + UP            → scroll back  (term_scroll_up)
 *     fn + DOWN          → scroll forward (term_scroll_down)
 *     UP/DOWN/LEFT/RIGHT → VT100 cursor escape sent to shell
 *     any other key      → snap to live, then route to shell
 *
 *   Scrollback
 *   ----------
 *   SCROLLBACK_LINES ring-buffer of char+colour rows.
 *   g_scroll_offset == 0  →  live view (normal).
 *   g_scroll_offset >  0  →  scrolled back; status bar shows "↑N".
 *   Any non-scroll key in terminal snaps back to live view first.
 *
 *   RAM: 240×135×2 = 64,800 B framebuffer +
 *        SCROLLBACK_LINES × COLS × 3 B history (200 × 30 × 3 = 18,000 B).
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/kthread.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/mutex.h>
#include <nuttx/irq.h>
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

#ifdef CONFIG_ESP32S3_TIMER
#  include "esp32s3_board_tim.h"
#endif

#if defined(CONFIG_ESPRESSIF_ADC) || defined(CONFIG_ESP32S3_ADC)
#  include "esp32s3_board_adc.h"
#endif

#include <fcntl.h>
#include <unistd.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <poll.h>

/****************************************************************************
 * External LCD driver symbols
 ****************************************************************************/

FAR struct lcd_dev_s *st7789_lcdinitialize(FAR struct spi_dev_s *spi);

#ifdef CONFIG_ESP32S3_SPIFLASH
int board_spiflash_init(void);
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

/* Pixel bands */
#define TERM_Y0   (TOP_ROW * FONT_H)
#define TERM_Y1   (LCD_H - 1)

/****************************************************************************
 * Colour palette — fixed hacker green
 ****************************************************************************/

#define COL_PROMPT   COL_ACCENT

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
 * Embedded font (JetBrains Mono 8×16)
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
 * Graphics library — RGB565 framebuffer
 ****************************************************************************/

static struct lcd_dev_s        *g_lcddev = NULL;
static struct lcd_planeinfo_s   g_pinfo;
static bool                     g_gfx_ready = false;
static uint16_t                 g_fb[LCD_W * LCD_H];
static mutex_t                  g_con_mutex = NXMUTEX_INITIALIZER;

static int gfx_init(void)
{
  if (g_lcddev == NULL) return -ENODEV;
  memset(&g_pinfo, 0, sizeof(g_pinfo));
  if (g_lcddev->getplaneinfo(g_lcddev, 0, &g_pinfo) < 0) return -EIO;
  g_gfx_ready = (g_pinfo.putarea != NULL || g_pinfo.putrun != NULL);
  return g_gfx_ready ? OK : -ENOSYS;
}

#ifndef GFX_SWAP_BYTES
#  define GFX_SWAP_BYTES 1
#endif

static uint16_t g_linebuf[LCD_W];

/* Flush pixel rows [y0..y1] inclusive to LCD over SPI.
   This is the ONLY path that writes to the physical display. */
static void gfx_flush_band(int y0, int y1)
{
  if (!g_gfx_ready) return;
  if (y0 < 0) y0 = 0;
  if (y1 > LCD_H - 1) y1 = LCD_H - 1;
  if (y1 < y0) return;

  FAR struct lcd_dev_s *dev = g_pinfo.dev ? g_pinfo.dev : g_lcddev;
  for (int row = y0; row <= y1; row++)
    {
      FAR const uint16_t *src = &g_fb[row * LCD_W];
#if GFX_SWAP_BYTES
      for (int x = 0; x < LCD_W; x++)
        {
          uint16_t v = src[x];
          g_linebuf[x] = (uint16_t)((v >> 8) | (v << 8));
        }
      FAR const uint8_t *p = (FAR const uint8_t *)g_linebuf;
#else
      FAR const uint8_t *p = (FAR const uint8_t *)src;
#endif
      if (g_pinfo.putarea != NULL)
        g_pinfo.putarea(dev, row, row, 0, LCD_W - 1, p, LCD_W * 2);
      else
        g_pinfo.putrun(dev, row, 0, p, LCD_W);
    }
}

/* Full-screen flush (menu, splash, scroll repaint) */
static inline void gfx_present(void) { gfx_flush_band(0, LCD_H - 1); }

/* Flush one terminal text row (16 pixel lines) */
static inline void gfx_flush_row(int text_row)
{
  int y = text_row * FONT_H;
  gfx_flush_band(y, y + FONT_H - 1);
}

static inline void gfx_px(int x, int y, uint16_t c)
{
  if ((unsigned)x < LCD_W && (unsigned)y < LCD_H) g_fb[y * LCD_W + x] = c;
}

static void gfx_hline(int x, int y, int w, uint16_t c)
{
  if (y < 0 || y >= LCD_H || w <= 0) return;
  if (x < 0) { w += x; x = 0; }
  if (x + w > LCD_W) w = LCD_W - x;
  if (w <= 0) return;
  uint16_t *p = &g_fb[y * LCD_W + x];
  while (w-- > 0) *p++ = c;
}

static void gfx_vline(int x, int y, int h, uint16_t c)
{
  if (x < 0 || x >= LCD_W || h <= 0) return;
  if (y < 0) { h += y; y = 0; }
  if (y + h > LCD_H) h = LCD_H - y;
  for (int i = 0; i < h; i++) g_fb[(y + i) * LCD_W + x] = c;
}

static void gfx_fill_rect(int x, int y, int w, int h, uint16_t c)
{
  for (int j = 0; j < h; j++) gfx_hline(x, y + j, w, c);
}

static void gfx_rect(int x, int y, int w, int h, uint16_t c)
{
  gfx_hline(x, y, w, c); gfx_hline(x, y + h - 1, w, c);
  gfx_vline(x, y, h, c); gfx_vline(x + w - 1, y, h, c);
}

static void gfx_clear(uint16_t c)
{
  for (int i = 0; i < LCD_W * LCD_H; i++) g_fb[i] = c;
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

static void gfx_fill_round(int x, int y, int w, int h, int r, uint16_t c)
{
  if (w <= 0 || h <= 0) return;
  if (r < 0) r = 0;
  if (r > w / 2) r = w / 2;
  if (r > h / 2) r = h / 2;
  for (int j = 0; j < h; j++)
    {
      int dy = -1;
      if (j < r)           dy = r - j;
      else if (j >= h - r) dy = r - (h - 1 - j);
      int inset = 0;
      if (dy >= 0) inset = r - isqrt32(r * r - dy * dy);
      gfx_hline(x + inset, y + j, w - 2 * inset, c);
    }
}

static void gfx_round(int x, int y, int w, int h, int r, uint16_t c)
{
  if (w <= 0 || h <= 0) return;
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
  for (int dy = -r; dy <= r; dy++)
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
  if (h <= 0) return;
  for (int j = 0; j < h; j++)
    {
      int t = (h > 1) ? (j * 32 / (h - 1)) : 0;
      gfx_hline(x, y + j, w, gfx_blend565(c0, c1, t));
    }
}

static void gfx_fill_round_grad(int x, int y, int w, int h, int r,
                                uint16_t c0, uint16_t c1)
{
  if (w <= 0 || h <= 0) return;
  if (r < 0) r = 0;
  if (r > w / 2) r = w / 2;
  if (r > h / 2) r = h / 2;
  for (int j = 0; j < h; j++)
    {
      int dy = -1;
      if (j < r)           dy = r - j;
      else if (j >= h - r) dy = r - (h - 1 - j);
      int inset = 0;
      if (dy >= 0) inset = r - isqrt32(r * r - dy * dy);
      int t = (h > 1) ? (j * 32 / (h - 1)) : 0;
      gfx_hline(x+inset, y+j, w-2*inset, gfx_blend565(c0, c1, t));
    }
}

static void gfx_glyph(int x, int y, char ch, uint16_t fg)
{
  unsigned char u = (unsigned char)ch;
  if (u < GFX_FONT_FIRST || u > GFX_FONT_LAST) return;
  const uint8_t *g = &g_font8x16[(u - GFX_FONT_FIRST) * GFX_FONT_H];
  for (int row = 0; row < GFX_FONT_H; row++)
    {
      uint8_t bits = g[row];
      if (!bits) continue;
      int py = y + row;
      if ((unsigned)py >= LCD_H) continue;
      uint16_t *line = &g_fb[py * LCD_W];
      for (int col = 0; col < GFX_FONT_W; col++)
        if (bits & (0x80 >> col))
          {
            int px = x + col;
            if ((unsigned)px < LCD_W) line[px] = fg;
          }
    }
}

static void gtext(int x, int y, const char *s, uint16_t fg)
{
  for (; *s; s++) { gfx_glyph(x, y, *s, fg); x += GFX_FONT_W; }
}

static void gtext_center(int cx, int y, const char *s, uint16_t fg)
{
  gtext(cx - (int)strlen(s) * GFX_FONT_W / 2, y, s, fg);
}

static void gtext_right(int rx, int y, const char *s, uint16_t fg)
{
  gtext(rx - (int)strlen(s) * GFX_FONT_W, y, s, fg);
}

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

/****************************************************************************
 * Ring-buffer character device
 ****************************************************************************/

#define RINGBUF_SIZE 512

struct ringdev_s
{
  char          buf[RINGBUF_SIZE];
  volatile int  head;
  volatile int  tail;
  sem_t         data_sem;
  sem_t         space_sem;
  FAR struct pollfd *fds_pollin;
};

static struct ringdev_s g_kbd_ring;
static struct ringdev_s g_out_ring;
static volatile bool    g_out_ring_ready = false;

static void ringdev_init(FAR struct ringdev_s *r)
{
  memset(r->buf, 0, sizeof(r->buf));
  r->head = 0; r->tail = 0; r->fds_pollin = NULL;
  nxsem_init(&r->data_sem,  0, 0);
  nxsem_init(&r->space_sem, 0, RINGBUF_SIZE);
}

static void ringdev_putc(FAR struct ringdev_s *r, char c)
{
  int ret;
  do { ret = nxsem_wait(&r->space_sem); } while (ret == -EINTR);
  r->buf[r->head] = c;
  r->head = (r->head + 1) % RINGBUF_SIZE;
  nxsem_post(&r->data_sem);
  if (r->fds_pollin != NULL) poll_notify(&r->fds_pollin, 1, POLLIN);
}

static void ringdev_tryputc(FAR struct ringdev_s *r, char c)
{
  if (nxsem_trywait(&r->space_sem) < 0) return;
  r->buf[r->head] = c;
  r->head = (r->head + 1) % RINGBUF_SIZE;
  nxsem_post(&r->data_sem);
  if (r->fds_pollin != NULL) poll_notify(&r->fds_pollin, 1, POLLIN);
}

static char ringdev_getc(FAR struct ringdev_s *r)
{
  int ret;
  do { ret = nxsem_wait(&r->data_sem); } while (ret == -EINTR);
  char c = r->buf[r->tail];
  r->tail = (r->tail + 1) % RINGBUF_SIZE;
  nxsem_post(&r->space_sem);
  return c;
}

static int ringdev_open (FAR struct file *f) { return OK; }
static int ringdev_close(FAR struct file *f) { return OK; }

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
  buf[0] = r->buf[r->tail];
  r->tail = (r->tail + 1) % RINGBUF_SIZE;
  nxsem_post(&r->space_sem);
  ssize_t n = 1;
  while (n < (ssize_t)len)
    {
      if (nxsem_trywait(&r->data_sem) < 0) break;
      buf[n] = r->buf[r->tail];
      r->tail = (r->tail + 1) % RINGBUF_SIZE;
      nxsem_post(&r->space_sem);
      n++;
    }
  return n;
}

static ssize_t ringdev_kbd_write(FAR struct file *filep,
                                 FAR const char *buf, size_t len)
{
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
{ ringdev_open, ringdev_close, ringdev_read, ringdev_kbd_write,
  NULL, ringdev_ioctl, NULL, NULL, ringdev_kbd_poll };

static const struct file_operations g_out_ops =
{ ringdev_open, ringdev_close, ringdev_out_read, ringdev_write,
  NULL, ringdev_ioctl, NULL, NULL, ringdev_out_poll };

/****************************************************************************
 * Syslog → LCD
 ****************************************************************************/

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
{ lcd_syslog_putc, lcd_syslog_putc, lcd_syslog_flush, NULL };
static struct syslog_channel_s g_lcd_syslog_channel = { &g_lcd_syslog_ops };
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
  { {KEY_FN,KEY_FN},{KEY_LEFT_SHIFT,KEY_LEFT_SHIFT},
    {'a','A'},{'s','S'},{'d','D'},{'f','F'},{'g','G'},
    {'h','H'},{'j','J'},{'k','K'},{'l','L'},
    {';',':'},{'\'','\"'},{KEY_ENTER,KEY_ENTER} },
  { {KEY_LEFT_CTRL,KEY_LEFT_CTRL},{(char)KEY_OPT,(char)KEY_OPT},
    {KEY_LEFT_ALT,KEY_LEFT_ALT},{'z','Z'},{'x','X'},{'c','C'},
    {'v','V'},{'b','B'},{'n','N'},{'m','M'},{',','<'},{'.','>'},{'/','?'},
    {' ',' '} }
};

/****************************************************************************
 * Soft I2C + TCA8418
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
static bool soft_tca8418_write(uint8_t reg, uint8_t val)
{
  irqstate_t f=enter_critical_section();
  soft_i2c_start();
  if (!soft_i2c_write_byte(TCA8418_ADDR<<1))
    { soft_i2c_stop(); leave_critical_section(f); return false; }
  if (!soft_i2c_write_byte(reg))
    { soft_i2c_stop(); leave_critical_section(f); return false; }
  if (!soft_i2c_write_byte(val))
    { soft_i2c_stop(); leave_critical_section(f); return false; }
  soft_i2c_stop(); leave_critical_section(f); return true;
}
static bool soft_tca8418_read(uint8_t reg, uint8_t *val)
{
  irqstate_t f=enter_critical_section();
  soft_i2c_start();
  if (!soft_i2c_write_byte(TCA8418_ADDR<<1))
    { soft_i2c_stop(); leave_critical_section(f); return false; }
  if (!soft_i2c_write_byte(reg))
    { soft_i2c_stop(); leave_critical_section(f); return false; }
  soft_i2c_start();
  if (!soft_i2c_write_byte((TCA8418_ADDR<<1)|1))
    { soft_i2c_stop(); leave_critical_section(f); return false; }
  *val=soft_i2c_read_byte(false);
  soft_i2c_stop(); leave_critical_section(f); return true;
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
  UI_MODE_BATTERY
};

static volatile int g_ui_mode = UI_MODE_MENU;
enum { NAV_NONE=0, NAV_UP, NAV_DOWN, NAV_LEFT, NAV_RIGHT };

struct menu_item_s { void (*icon)(int,int,uint16_t); const char *label; };
static const struct menu_item_s g_menu_items[] =
{
  { icon_screen,   "ScreenTest" },
  { icon_keyboard, "Keyboard"   },
  { icon_terminal, "Terminal"   },
  { icon_files,    "Files"      },
  { icon_battery,  "Battery"    },
};
#define MENU_COUNT ((int)(sizeof(g_menu_items)/sizeof(g_menu_items[0])))
static int g_menu_sel = 0;

/* FileManager */
#define FM_MAX_ENTRIES 64
#define FM_NAME_LEN    40
#define FM_VISIBLE     6
struct fm_entry_s { char name[FM_NAME_LEN]; bool is_dir; };
static struct fm_entry_s g_fm_entries[FM_MAX_ENTRIES];
static int  g_fm_count=0, g_fm_sel=0, g_fm_top=0;
static char g_fm_path[128]="/";

/* ScreenTest */
static const struct { const char *name; uint16_t color; } g_st_entries[] =
{
  {"WHITE",RGB_WHITE},{"RED",RGB_RED},{"GREEN",RGB_GREEN},
  {"BLUE",RGB_BLUE},{"YELLOW",RGB_YELLOW},{"CYAN",RGB_CYAN},
  {"MAGENTA",RGB_MAGENTA},
};
#define ST_COLOR_COUNT ((int)(sizeof(g_st_entries)/sizeof(g_st_entries[0])))
static int g_st_color_idx=0;

/* Battery */
#define BAT_ADC_DEVPATH "/dev/adc0"
#define BAT_ADC_CHANNEL 9
#define BAT_DIVIDER     2

struct bat_read_s { int mv,err,nsamp,ch0,raw0; };

static void battery_read(struct bat_read_s *o)
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
static int battery_mv(void) { struct bat_read_s r; battery_read(&r); return r.mv; }
static int battery_percent(int mv)
{
  static const struct { int mv,pct; } lut[]=
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

/****************************************************************************
 * Status bar  (compose into framebuffer only — caller does flush)
 ****************************************************************************/

static void statusbar_compose(void)
{
  gfx_fill_rect(0, 0, LCD_W, BAR_H, COL_BAR_BG);
  gfx_hline(0, BAR_H-1, LCD_W, COL_SEL_BG);

  const char *title;
  switch (g_ui_mode)
    {
      case UI_MODE_TERMINAL:     title="Terminal";   break;
      case UI_MODE_SCREENTEST:   title="ScreenTest"; break;
      case UI_MODE_KEYBOARDTEST: title="Keyboard";   break;
      case UI_MODE_FILEMANAGER:  title="Files";      break;
      case UI_MODE_BATTERY:      title="Battery";    break;
      default:                   title="Cardputer";  break;
    }
  gtext(6, 0, title, COL_ACCENT);

  int bx=LCD_W-30, by=3, bw=24, bh=11;
  int mv=battery_mv();
  if (mv<0)
    {
      gfx_round(bx,by,bw,bh,3,COL_DIM);
      gfx_fill_rect(bx+bw,by+3,2,5,COL_DIM);
      gtext_right(bx-4, 0, "N/A", COL_DIM);
    }
  else
    {
      int pct=battery_percent(mv);
      uint16_t col=(pct<=20)?COL_ERR:(pct<=50)?COL_WARN:COL_OK;
      gfx_round(bx,by,bw,bh,3,COL_DIM);
      gfx_fill_rect(bx+bw,by+3,2,5,COL_DIM);
      int fillw=pct*(bw-6)/100;
      if (fillw>0) gfx_fill_round(bx+3,by+3,fillw,bh-6,1,col);
      char pbuf[8]; snprintf(pbuf,sizeof(pbuf),"%d%%",pct);
      gtext_right(bx-5, 0, pbuf, col);
    }
}

/****************************************************************************
 * Scrollback buffer
 *
 * Layout
 * ------
 *   g_sb_lines[slot][col]   char   (slot = abs_line % SCROLLBACK_LINES)
 *   g_sb_colors[slot][col]  colour
 *   g_sb_total              absolute index of current (live) line
 *   g_sb_col                write column on the live line
 *   g_scroll_offset         0 = live view; N = scrolled back N lines
 *
 * Flush discipline
 * ----------------
 *   sb_paint_row_locked(text_row)
 *       Redraws one text_row of the *view* into the framebuffer.
 *       Does NOT touch the SPI bus.
 *
 *   con_putc()
 *       Printable char  → sb_putch → sb_paint_row_locked(live row)
 *                        → gfx_flush_row(live_text_row)    ← 1 row SPI
 *       '\n'            → sb_newline_locked then full-band repaint
 *                        (sb_scroll_screen_locked).  The view is bottom-
 *                        anchored, so every newline shifts ALL rows up by
 *                        one and the whole terminal band must be redrawn.
 *       '\b' '\r'       → sb_paint_row_locked + gfx_flush_row
 *       Suppressed when g_scroll_offset > 0 (user is reading history).
 *
 *   term_repaint_locked()   (fn+UP/DN only)
 *       Repaints all TERM_ROWS + status indicator → gfx_present()
 ****************************************************************************/

#define SCROLLBACK_LINES  200
#define SB_SLOT(l)        ((l) % SCROLLBACK_LINES)
#define SB_CH(l,c)        g_sb_lines [SB_SLOT(l)][(c)]
#define SB_COL(l,c)       g_sb_colors[SB_SLOT(l)][(c)]

static char     g_sb_lines [SCROLLBACK_LINES][COLS];
static uint16_t g_sb_colors[SCROLLBACK_LINES][COLS];
static int      g_sb_total       = 0;
static int      g_sb_col         = 0;
static int      g_scroll_offset  = 0;

static void sb_clear_slot(int slot)
{
  memset(g_sb_lines[slot], ' ', COLS);
  for (int c=0; c<COLS; c++) g_sb_colors[slot][c] = COL_FG;
}

static void sb_clear_line(int abs_line)
{
  sb_clear_slot(SB_SLOT(abs_line));
}

/* Paint one view row (0-based within terminal area) into framebuffer.
   Does NOT flush to SPI. */
static void sb_paint_row_locked(int view_row)
{
  /* Which absolute scrollback line maps to this view row? */
  int first     = g_sb_total - g_scroll_offset - (TERM_ROWS - 1);
  int abs_line  = first + view_row;
  int text_row  = TOP_ROW + view_row;
  int y         = text_row * FONT_H;

  gfx_fill_rect(0, y, LCD_W, FONT_H, COL_BG);
  if (abs_line < 0 || abs_line > g_sb_total) return;

  for (int c=0; c<COLS; c++)
    {
      char     ch  = SB_CH (abs_line, c);
      uint16_t col = SB_COL(abs_line, c);
      if (ch != ' ') gfx_glyph(c * FONT_W, y, ch, col);
    }
}

/* The live line is always view_row = TERM_ROWS-1 when scroll_offset==0 */
static inline int sb_live_view_row(void) { return TERM_ROWS - 1; }
static inline int sb_live_text_row(void) { return TOP_ROW + sb_live_view_row(); }

/* Scroll the screen up one line: repaint all rows, flush terminal band. */
static void sb_scroll_screen_locked(void)
{
  for (int r=0; r<TERM_ROWS; r++) sb_paint_row_locked(r);
  gfx_flush_band(TERM_Y0, TERM_Y1);
}

/* Commit live line, open next. */
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

/****************************************************************************
 * Full terminal repaint — used ONLY for scrollback view changes.
 * Redraws everything then calls gfx_present() (full SPI).
 * Must be called with g_con_mutex held.
 ****************************************************************************/

static void term_repaint_locked(void)
{
  /* Redraw status bar (scroll indicator or normal) */
  statusbar_compose();
  if (g_scroll_offset > 0)
    {
      /* Overwrite right half of bar with scroll indicator */
      gfx_fill_rect(LCD_W/2, 0, LCD_W/2, BAR_H, COL_BAR_BG);
      char sbuf[16];
      snprintf(sbuf, sizeof(sbuf), "\x18%d", g_scroll_offset); /* ↑N */
      gtext_right(LCD_W - 2, 0, sbuf, COL_WARN);
    }

  /* Repaint all terminal rows */
  for (int r=0; r<TERM_ROWS; r++) sb_paint_row_locked(r);

  gfx_present();   /* full SPI — acceptable for scroll key events */
}

/****************************************************************************
 * con_putc — hot path character output
 *
 * Performance guarantee: every printable character causes at most ONE
 * gfx_flush_row() call (16 pixel rows of SPI).  A newline redraws the whole
 * terminal band via sb_scroll_screen_locked() (one gfx_flush_band of the
 * terminal area) because the view is bottom-anchored: advancing g_sb_total
 * shifts EVERY visible row up by one, so they must all be repainted.
 * gfx_present() is NEVER called from this function.
 *
 * While scrolled (g_scroll_offset > 0) the scrollback store is updated
 * silently; the screen is NOT refreshed so the user's scroll position is
 * preserved.
 ****************************************************************************/

static void con_putc(char c, uint16_t color)
{
  nxmutex_lock(&g_con_mutex);

  if (c == '\r')
    {
      g_sb_col = 0;
      if (g_scroll_offset == 0)
        {
          sb_paint_row_locked(sb_live_view_row());
          gfx_flush_row(sb_live_text_row());
        }
    }
  else if (c == '\b')
    {
      if (g_sb_col > 0)
        {
          g_sb_col--;
          SB_CH (g_sb_total, g_sb_col) = ' ';
          SB_COL(g_sb_total, g_sb_col) = COL_FG;
        }
      if (g_scroll_offset == 0)
        {
          sb_paint_row_locked(sb_live_view_row());
          gfx_flush_row(sb_live_text_row());
        }
    }
  else if (c == '\n')
    {
      /* Advance to a fresh live line.  Because the view is bottom-anchored
       * (the live line is always painted at the bottom terminal row), every
       * newline shifts ALL visible rows up by one — so the whole terminal
       * band must be repainted, not just the new bottom row.
       *
       * Painting only the bottom row (the old "at_bottom" fast path) was the
       * cause of output appearing to "jump in place": each printed line was
       * drawn at the bottom, then erased by the next newline without the
       * earlier lines ever being shifted up into view.  They only became
       * visible later, once enough lines accumulated to trigger a full
       * repaint — hence "the second ls makes both appear".
       */
      sb_newline_locked();
      if (g_scroll_offset == 0)
        sb_scroll_screen_locked();
      /* if scrolled: silently update store, no screen change */
    }
  else if (c >= 0x20 && (unsigned char)c < 0x80)
    {
      if (g_sb_col >= COLS)
        {
          /* Soft-wrap: commit line, open next */
          sb_newline_locked();
          if (g_scroll_offset == 0)
            sb_scroll_screen_locked();
        }

      SB_CH (g_sb_total, g_sb_col) = c;
      SB_COL(g_sb_total, g_sb_col) = color;
      g_sb_col++;

      if (g_scroll_offset == 0)
        {
          sb_paint_row_locked(sb_live_view_row());
          gfx_flush_row(sb_live_text_row());
        }
    }

  nxmutex_unlock(&g_con_mutex);
}

/****************************************************************************
 * con_clear / con_clear_full
 ****************************************************************************/

static void con_clear_locked(void)
{
  g_sb_total      = 0;
  g_sb_col        = 0;
  g_scroll_offset = 0;
  for (int s=0; s<SCROLLBACK_LINES; s++) sb_clear_slot(s);
  gfx_fill_rect(0, TERM_Y0, LCD_W, LCD_H - TERM_Y0, COL_BG);
  gfx_flush_band(TERM_Y0, TERM_Y1);
}

static void con_clear(void)
{
  nxmutex_lock(&g_con_mutex);
  con_clear_locked();
  nxmutex_unlock(&g_con_mutex);
}

static void con_clear_full(void)
{
  nxmutex_lock(&g_con_mutex);
  g_sb_total=0; g_sb_col=0; g_scroll_offset=0;
  for (int s=0;s<SCROLLBACK_LINES;s++) sb_clear_slot(s);
  gfx_clear(COL_BG);
  gfx_present();
  nxmutex_unlock(&g_con_mutex);
}

/****************************************************************************
 * Scrollback scroll helpers (called from kbd thread — acquire mutex here)
 ****************************************************************************/

#define SCROLL_STEP 1

static void term_scroll_up(void)
{
  nxmutex_lock(&g_con_mutex);
  int max = sb_max_offset();
  if (g_scroll_offset < max)
    {
      g_scroll_offset += SCROLL_STEP;
      if (g_scroll_offset > max) g_scroll_offset = max;
      term_repaint_locked();
    }
  nxmutex_unlock(&g_con_mutex);
}

static void term_scroll_down(void)
{
  nxmutex_lock(&g_con_mutex);
  if (g_scroll_offset > 0)
    {
      g_scroll_offset -= SCROLL_STEP;
      if (g_scroll_offset < 0) g_scroll_offset = 0;
      term_repaint_locked();
    }
  nxmutex_unlock(&g_con_mutex);
}

/* Snap back to live. Returns true if we were scrolled (caller may want to
   eat the keypress that caused the snap). */
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
static void footer_compose(const char *text)
{
  int y=119;
  gfx_fill_circle(8, y+7, 2, COL_ACCENT);
  gtext(14, y, text, COL_DIM);
}

/****************************************************************************
 * Main menu
 ****************************************************************************/

#define MENU_Y0   20
#define MENU_STEP 16
#define MENU_IH   16

static void menu_compose(void)
{
  bg_compose();
  statusbar_compose();
  for (int i=0; i<MENU_COUNT; i++)
    {
      int y=MENU_Y0+i*MENU_STEP;
      bool sel=(i==g_menu_sel);
      const struct menu_item_s *m=&g_menu_items[i];
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
  footer_compose("UP/DN   ENTER   OPT=menu");
}

static void menu_render(void)
{
  nxmutex_lock(&g_con_mutex);
  menu_compose(); gfx_present();
  nxmutex_unlock(&g_con_mutex);
}

static void statusbar_render(void)
{
  nxmutex_lock(&g_con_mutex);
  statusbar_compose();
  gfx_flush_band(0, BAR_H-1);
  nxmutex_unlock(&g_con_mutex);
}

/****************************************************************************
 * ScreenTest
 ****************************************************************************/

static void screentest_render(void)
{
  uint16_t color=g_st_entries[g_st_color_idx].color;
  const char *name=g_st_entries[g_st_color_idx].name;
  nxmutex_lock(&g_con_mutex);
  gfx_clear(color);
  int cw=150,ch=56,cx=(LCD_W-cw)/2,cy=(LCD_H-ch)/2;
  gfx_fill_round(cx,cy,cw,ch,10,COL_BG);
  gfx_round(cx,cy,cw,ch,10,COL_ACCENT);
  gtext_center(LCD_W/2,cy+10,name,color);
  char idx[16]; snprintf(idx,sizeof(idx),"%d / %d",g_st_color_idx+1,ST_COLOR_COUNT);
  gtext_center(LCD_W/2,cy+30,idx,COL_FG);
  gtext_center(LCD_W/2,LCD_H-16,"UP/DN=colour   OPT=back",COL_DIM);
  gfx_present();
  nxmutex_unlock(&g_con_mutex);
}

static void screentest_handle(int navkey)
{
  if      (navkey==NAV_UP)   g_st_color_idx=(g_st_color_idx+ST_COLOR_COUNT-1)%ST_COLOR_COUNT;
  else if (navkey==NAV_DOWN) g_st_color_idx=(g_st_color_idx+1)%ST_COLOR_COUNT;
  else return;
  screentest_render();
}

/****************************************************************************
 * Keyboard test
 ****************************************************************************/

static void kt_compose_base(void)
{
  bg_compose(); statusbar_compose();
  gfx_fill_round_grad(8,22,224,20,6,COL_SEL_BG2,COL_SEL_BG);
  gtext(16,24,"Keyboard Test",COL_ACCENT);
}
static void kt_chip(int x,int y,const char *s,bool on,uint16_t onc)
{
  if (on) { gfx_fill_round(x,y,26,16,4,onc); gtext(x+5,y,s,COL_BG); }
  else    { gfx_round(x,y,26,16,4,COL_DIM);  gtext(x+5,y,s,COL_DIM); }
}
static void keyboardtest_enter(void)
{
  nxmutex_lock(&g_con_mutex);
  kt_compose_base();
  gfx_fill_round(8,48,224,26,6,COL_SEL_BG);
  gtext_center(LCD_W/2,53,"Press any key ...",COL_DIM);
  gtext(16,80,"scan  ---",COL_DIM);
  kt_chip(16,98,"SH",false,COL_WARN);
  kt_chip(48,98,"CT",false,COL_ACCENT);
  kt_chip(80,98,"FN",false,COL_FG);
  footer_compose("Key shown above   OPT=back");
  gfx_present();
  nxmutex_unlock(&g_con_mutex);
}
static void keyboardtest_show(char ch,uint8_t raw,int navkey,
                              bool sh,bool ct,bool fn)
{
  char key_str[40];
  const char *name=NULL;
  if      (navkey==NAV_UP)    name="UP";
  else if (navkey==NAV_DOWN)  name="DOWN";
  else if (navkey==NAV_LEFT)  name="LEFT";
  else if (navkey==NAV_RIGHT) name="RIGHT";

  if (name)                           snprintf(key_str,sizeof(key_str),"Key:  %s",name);
  else if (ch=='\r')                  snprintf(key_str,sizeof(key_str),"Key:  ENTER");
  else if (ch=='\t')                  snprintf(key_str,sizeof(key_str),"Key:  TAB");
  else if (ch==(char)KEY_BACKSPACE)   snprintf(key_str,sizeof(key_str),"Key:  BACKSPACE");
  else if (ch>=0x20&&(uint8_t)ch<0x7f)
    snprintf(key_str,sizeof(key_str),"Key:  '%c'  (0x%02X)",ch,(uint8_t)ch);
  else snprintf(key_str,sizeof(key_str),"Code: 0x%02X",(uint8_t)ch);

  char scan_str[24]; snprintf(scan_str,sizeof(scan_str),"scan  0x%02X",raw);

  nxmutex_lock(&g_con_mutex);
  uint16_t kcol=(ch>=0x20&&(uint8_t)ch<0x7f&&navkey==NAV_NONE)?COL_SEL_FG:COL_WARN;
  gfx_fill_round(8,48,224,26,6,COL_SEL_BG);
  gtext(16,53,key_str,kcol);
  gfx_fill_rect(8,80,224,16,COL_BG);
  gtext(16,80,scan_str,COL_FG);
  kt_chip(16,98,"SH",sh,COL_WARN);
  kt_chip(48,98,"CT",ct,COL_ACCENT);
  kt_chip(80,98,"FN",fn,COL_FG);
  gfx_flush_band(48,113);
  nxmutex_unlock(&g_con_mutex);
}

/****************************************************************************
 * FileManager
 ****************************************************************************/

#define FM_ROW_H 15
#define FM_Y0    38

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
  g_fm_count=0; g_fm_sel=0; g_fm_top=0;
  if (strcmp(g_fm_path,"/")!=0)
    {
      snprintf(g_fm_entries[0].name,FM_NAME_LEN,"..");
      g_fm_entries[0].is_dir=true; g_fm_count=1;
    }
  DIR *d=opendir(g_fm_path);
  if (d)
    {
      struct dirent *e;
      while ((e=readdir(d))!=NULL && g_fm_count<FM_MAX_ENTRIES)
        {
          snprintf(g_fm_entries[g_fm_count].name,FM_NAME_LEN,"%.*s",FM_NAME_LEN-1,e->d_name);
          g_fm_entries[g_fm_count].is_dir=(e->d_type==DT_DIR);
          g_fm_count++;
        }
      closedir(d);
    }
}
static void fm_join(const char *name)
{
  if (strcmp(name,"..")==0)
    {
      char *s=strrchr(g_fm_path,'/');
      if (s) { if (s==g_fm_path) g_fm_path[1]='\0'; else *s='\0'; }
      return;
    }
  size_t len=strlen(g_fm_path);
  if (len>0 && g_fm_path[len-1]=='/')
    snprintf(g_fm_path+len,sizeof(g_fm_path)-len,"%s",name);
  else
    snprintf(g_fm_path+len,sizeof(g_fm_path)-len,"/%s",name);
}
static void fm_render(void)
{
  nxmutex_lock(&g_con_mutex);
  bg_compose(); statusbar_compose();
  gfx_fill_round(6,20,228,14,4,COL_SEL_BG);
  char pbuf[28]; snprintf(pbuf,sizeof(pbuf),"%.26s",g_fm_path);
  gtext(12,19,pbuf,COL_ACCENT);
  if (g_fm_count>FM_VISIBLE)
    {
      char sc[10]; snprintf(sc,sizeof(sc),"%d/%d",g_fm_sel+1,g_fm_count);
      gtext_right(LCD_W-10,19,sc,COL_DIM);
    }
  if (g_fm_sel<g_fm_top) g_fm_top=g_fm_sel;
  if (g_fm_sel>=g_fm_top+FM_VISIBLE) g_fm_top=g_fm_sel-FM_VISIBLE+1;
  for (int i=0;i<FM_VISIBLE;i++)
    {
      int idx=g_fm_top+i, y=FM_Y0+i*FM_ROW_H;
      if (idx>=g_fm_count) continue;
      bool sel=(idx==g_fm_sel), is_dir=g_fm_entries[idx].is_dir;
      const char *nm=g_fm_entries[idx].name;
      uint16_t icol=is_dir?COL_ACCENT:COL_DIM;
      if (sel) { gfx_fill_round(4,y,232,FM_ROW_H-1,5,COL_SEL_BG);
                 gfx_fill_round(6,y+2,3,FM_ROW_H-5,1,COL_MARKER); }
      if (is_dir) fm_dir_icon(13,y+1,icol);
      else        fm_file_icon(13,y+1,icol);
      gtext(30,y,nm,sel?COL_SEL_FG:(is_dir?COL_FG:COL_DIM));
    }
  gfx_present();
  nxmutex_unlock(&g_con_mutex);
}
static void filemanager_enter(void)
{
  snprintf(g_fm_path,sizeof(g_fm_path),"/"); fm_load(); fm_render();
}
static void filemanager_handle(int navkey,char ch)
{
  if      (navkey==NAV_UP)   { if (g_fm_count) g_fm_sel=(g_fm_sel+g_fm_count-1)%g_fm_count; fm_render(); }
  else if (navkey==NAV_DOWN) { if (g_fm_count) g_fm_sel=(g_fm_sel+1)%g_fm_count; fm_render(); }
  else if (ch=='\r'||ch=='\n')
    {
      if (!g_fm_count) return;
      if (g_fm_entries[g_fm_sel].is_dir)
        { fm_join(g_fm_entries[g_fm_sel].name); fm_load(); fm_render(); }
    }
}

/****************************************************************************
 * Battery app
 ****************************************************************************/

static void battery_render(void)
{
  struct bat_read_s r; battery_read(&r);
  nxmutex_lock(&g_con_mutex);
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
      uint16_t col=(pct<=20)?COL_ERR:(pct<=50)?COL_WARN:COL_OK;
      int bx=30,by=36,bw=150,bh=48;
      gfx_round(bx,by,bw,bh,8,col);
      gfx_round(bx+1,by+1,bw-2,bh-2,7,col);
      gfx_fill_round(bx+bw,by+15,9,18,3,col);
      int fillw=pct*(bw-12)/100;
      if (fillw>0)
        gfx_fill_round_grad(bx+6,by+6,fillw,bh-12,4,
                            gfx_blend565(col,RGB_WHITE,10),col);
      char big[8]; snprintf(big,sizeof(big),"%d%%",pct);
      gtext_center(bx+bw/2,by+bh/2-8,big,COL_SEL_FG);
      char line[40];
      snprintf(line,sizeof(line),"Voltage  %d mV",r.mv);
      gtext(30,94,line,COL_FG);
      const char *state=(pct>=95)?"Full":(pct<=20)?"Low - charge soon":"OK";
      snprintf(line,sizeof(line),"Status   %s",state);
      gtext(30,110,line,col);
    }
  footer_compose("auto-refresh   OPT=back");
  gfx_present();
  nxmutex_unlock(&g_con_mutex);
}
static void battery_enter(void) { battery_render(); }

/****************************************************************************
 * Mode switch
 ****************************************************************************/

static void ui_enter_mode(int mode)
{
  g_ui_mode = mode;
  switch (mode)
    {
      case UI_MODE_SCREENTEST:
        g_st_color_idx=0; screentest_render(); break;
      case UI_MODE_KEYBOARDTEST:
        keyboardtest_enter(); break;
      case UI_MODE_FILEMANAGER:
        filemanager_enter(); break;
      case UI_MODE_BATTERY:
        battery_enter(); break;
      case UI_MODE_TERMINAL:
        con_clear(); statusbar_render();
        ringdev_putc(&g_kbd_ring,'\n'); break;
      default: break;
    }
}

static void ui_back_to_menu(void)
{
  g_ui_mode = UI_MODE_MENU;
  g_scroll_offset = 0;
  menu_render();
}

static void menu_handle(int navkey, char ch)
{
  if      (navkey==NAV_UP)         { g_menu_sel=(g_menu_sel+MENU_COUNT-1)%MENU_COUNT; menu_render(); }
  else if (navkey==NAV_DOWN)       { g_menu_sel=(g_menu_sel+1)%MENU_COUNT; menu_render(); }
  else if (ch=='\r'||ch=='\n')
    {
      switch (g_menu_sel)
        {
          case 0: ui_enter_mode(UI_MODE_SCREENTEST);   break;
          case 1: ui_enter_mode(UI_MODE_KEYBOARDTEST); break;
          case 2: ui_enter_mode(UI_MODE_TERMINAL);     break;
          case 3: ui_enter_mode(UI_MODE_FILEMANAGER);  break;
          case 4: ui_enter_mode(UI_MODE_BATTERY);      break;
          default: break;
        }
    }
}

/****************************************************************************
 * kbd_nsh_thread
 *
 * Key decode rules
 * ----------------
 * Step 1 – modifier latches (shift/ctrl/fn) are processed for every event
 *          regardless of mode.
 *
 * Step 2 – navkey decode:
 *   fn pressed  →  i/;=UP  k/.=DOWN  j=LEFT  l=RIGHT
 *   fn absent   →  navkey stays NAV_NONE; ch is decoded from key map
 *
 * Step 3 – mode dispatch:
 *
 *   UI_MODE_MENU
 *       navkey UP/DOWN   → menu navigate  (works with OR without fn)
 *       ch ENTER         → select
 *
 *   UI_MODE_TERMINAL
 *       fn+UP            → term_scroll_up()
 *       fn+DOWN          → term_scroll_down()
 *       UP/DOWN/L/R      → VT100 escape (snap first)
 *       ch               → snap + send to shell
 *
 *   UI_MODE_SCREENTEST
 *       navkey UP/DOWN   → cycle colours (no fn needed, matches original)
 *
 *   UI_MODE_KEYBOARDTEST / FILEMANAGER / BATTERY
 *       unchanged
 ****************************************************************************/

static void kbd_send_vt100(const char *seq)
{ for (;*seq;seq++) ringdev_putc(&g_kbd_ring,*seq); }

static void kbd_echo(char c) { ringdev_tryputc(&g_out_ring,c); }

static int kbd_nsh_thread(int argc, FAR char *argv[])
{
  uint8_t reg_val;
  bool shift_pressed=false, ctrl_pressed=false, fn_pressed=false;

#define KBD_COOLDOWN_MS 100
  uint32_t last_key_ms=0, g_uptime_ms=0;
  uint8_t  last_key_raw=0;
  uint32_t bar_last_ms=0, bat_last_ms=0;

  esp_configgpio(44,OUTPUT); esp_gpiowrite(44,1);
  esp_configgpio(11,INPUT|PULLUP);
  up_mdelay(300);

  if (!soft_tca8418_write(0x01,0x80)) return -ENODEV;
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
      g_uptime_ms += 20;

      if (soft_tca8418_read(0x03,&reg_val))
        {
          uint8_t event_count = reg_val & 0x0F;

          while (event_count > 0)
            {
              if (!soft_tca8418_read(0x04,&reg_val))
                { event_count--; continue; }

              bool    is_pressed = (reg_val & 0x80) != 0;
              uint8_t raw_id     = reg_val & 0x7F;
              if (raw_id==0) { event_count--; continue; }

              uint8_t buf_id=raw_id-1, raw_row=0, raw_col=buf_id;
              while (raw_col>=10) { raw_col-=10; raw_row++; }
              uint8_t mapped_col=(uint8_t)(raw_row<<1);
              if (raw_col>3) mapped_col++;
              uint8_t mapped_row=(uint8_t)((raw_col+4)&3);
              if (mapped_col>=14||mapped_row>=4) { event_count--; continue; }

              uint8_t id=(uint8_t)g_key_map[mapped_row][mapped_col].value_first;

              /* --- Modifier latches --- */
              if (id==KEY_LEFT_SHIFT) { shift_pressed=is_pressed; event_count--; continue; }
              if (id==KEY_LEFT_CTRL)  { ctrl_pressed =is_pressed; event_count--; continue; }
              if (id==KEY_FN)         { fn_pressed   =is_pressed; event_count--; continue; }
              if (id==(uint8_t)KEY_OPT) { if (is_pressed) ui_back_to_menu(); event_count--; continue; }
              if (id==KEY_LEFT_ALT)   { event_count--; continue; }
              if (!is_pressed)        { event_count--; continue; }

              /* --- Cooldown --- */
              if (raw_id==last_key_raw &&
                  (g_uptime_ms-last_key_ms)<KBD_COOLDOWN_MS)
                { event_count--; continue; }
              last_key_ms=g_uptime_ms; last_key_raw=raw_id;

              /* --- Decode navkey (fn-layer) and/or character --- */
              int  navkey = NAV_NONE;
              char ch     = 0;

              if (fn_pressed)
                {
                  /* fn held: decode navigation layer */
                  char base = g_key_map[mapped_row][mapped_col].value_first;
                  switch (base)
                    {
                      case 'i': case ';': navkey=NAV_UP;    break;
                      case 'k': case '.': navkey=NAV_DOWN;  break;
                      case 'j':           navkey=NAV_LEFT;  break;
                      case 'l':           navkey=NAV_RIGHT; break;
                      default:
                        /* fn + non-arrow: decode as character normally */
                        if (ctrl_pressed)
                          {
                            if (base>='a'&&base<='z') ch=(char)(base-'a'+1);
                            else if (base>='A'&&base<='Z') ch=(char)(base-'A'+1);
                            else ch=base;
                          }
                        else
                          ch=shift_pressed
                             ? g_key_map[mapped_row][mapped_col].value_second
                             : base;
                        if ((uint8_t)ch>=0x80) { event_count--; continue; }
                        break;
                    }
                }
              else
                {
                  /* No fn: decode character layer */
                  if (ctrl_pressed)
                    {
                      char base=g_key_map[mapped_row][mapped_col].value_first;
                      if (base>='a'&&base<='z') ch=(char)(base-'a'+1);
                      else if (base>='A'&&base<='Z') ch=(char)(base-'A'+1);
                      else ch=base;
                    }
                  else
                    ch=shift_pressed
                       ? g_key_map[mapped_row][mapped_col].value_second
                       : g_key_map[mapped_row][mapped_col].value_first;
                  if ((uint8_t)ch>=0x80) { event_count--; continue; }
                }

              /* --- Mode dispatch --- */
              switch (g_ui_mode)
                {
                  /* ---- MENU ---- */
                  case UI_MODE_MENU:
                    /*
                     * UP/DOWN work with or without fn.
                     * navkey is set by both fn-layer and (when fn absent) we
                     * synthesise it from ch below so the branch is uniform.
                     */
                    if (navkey==NAV_NONE)
                      {
                        /* no fn: derive navkey from raw id for the two
                           physical arrow positions (i/k when fn not held) */
                        char base=g_key_map[mapped_row][mapped_col].value_first;
                        if (base=='i'||base==';') navkey=NAV_UP;
                        else if (base=='k'||base=='.') navkey=NAV_DOWN;
                      }
                    menu_handle(navkey, ch);
                    break;

                  /* ---- TERMINAL ---- */
                  case UI_MODE_TERMINAL:
                    if (fn_pressed && navkey==NAV_UP)
                      { term_scroll_up(); }
                    else if (fn_pressed && navkey==NAV_DOWN)
                      { term_scroll_down(); }
                    else
                      {
                        term_snap_live();   /* snap on any non-scroll key */
                        if      (navkey==NAV_UP)    kbd_send_vt100("\x1b[A");
                        else if (navkey==NAV_DOWN)  kbd_send_vt100("\x1b[B");
                        else if (navkey==NAV_LEFT)  kbd_send_vt100("\x1b[D");
                        else if (navkey==NAV_RIGHT) kbd_send_vt100("\x1b[C");
                        else if (ch=='\r'||ch==KEY_ENTER)
                          { kbd_echo('\r'); kbd_echo('\n'); ringdev_putc(&g_kbd_ring,'\n'); }
                        else if (ch==KEY_BACKSPACE)
                          ringdev_putc(&g_kbd_ring,KEY_BACKSPACE);
                        else if (ch=='\t')
                          ringdev_putc(&g_kbd_ring,ch);
                        else if ((uint8_t)ch<0x20&&ch!=0)
                          ringdev_putc(&g_kbd_ring,ch);
                        else if (ch!=0)
                          { kbd_echo(ch); ringdev_putc(&g_kbd_ring,ch); }
                      }
                    break;

                  /* ---- SCREENTEST ---- */
                  case UI_MODE_SCREENTEST:
                    /* UP/DOWN work without fn (navkey may be NAV_NONE if fn absent) */
                    if (navkey==NAV_NONE)
                      {
                        char base=g_key_map[mapped_row][mapped_col].value_first;
                        if (base=='i'||base==';') navkey=NAV_UP;
                        else if (base=='k'||base=='.') navkey=NAV_DOWN;
                      }
                    screentest_handle(navkey);
                    break;

                  /* ---- KEYBOARD TEST ---- */
                  case UI_MODE_KEYBOARDTEST:
                    keyboardtest_show(ch,raw_id,navkey,
                                      shift_pressed,ctrl_pressed,fn_pressed);
                    break;

                  /* ---- FILE MANAGER ---- */
                  case UI_MODE_FILEMANAGER:
                    if (navkey==NAV_NONE)
                      {
                        char base=g_key_map[mapped_row][mapped_col].value_first;
                        if (base=='i'||base==';') navkey=NAV_UP;
                        else if (base=='k'||base=='.') navkey=NAV_DOWN;
                      }
                    filemanager_handle(navkey,ch);
                    break;

                  case UI_MODE_BATTERY:
                  default: break;
                }

              event_count--;
            }

          soft_tca8418_write(0x02,0x0F);
        }

      /* Periodic status bar tick — skip while scroll indicator is shown */
      if (g_ui_mode != UI_MODE_SCREENTEST &&
          g_scroll_offset == 0 &&
          (g_uptime_ms-bar_last_ms) >= 2000)
        { bar_last_ms=g_uptime_ms; statusbar_render(); }

      if (g_ui_mode==UI_MODE_BATTERY &&
          (g_uptime_ms-bat_last_ms) >= 1000)
        { bat_last_ms=g_uptime_ms; battery_render(); }

      up_mdelay(20);
    }

  return OK;
}

/****************************************************************************
 * lcd_cons_thread — VT100 parser → scrollback
 *
 * Supported escape sequences:
 *   ESC [ n J   erase display
 *   ESC [ n H   cursor home (treated as erase)
 *   ESC [ n K   erase to end of line
 *   ESC [ n A   cursor up   n  (moves scrollback write-pointer)
 *   ESC [ n B   cursor down n
 *   ESC [ n C   cursor right n (column move)
 *   ESC [ n D   cursor left  n
 *   0x0C (FF)   clear screen
 *
 * Colour SGR sequences (ESC [ … m) are silently consumed.
 ****************************************************************************/

#define ANSI_ESC     0x1b
#define ANSI_BUF_MAX 16

static int lcd_cons_thread(int argc, FAR char *argv[])
{
  uint8_t ansi_buf[ANSI_BUF_MAX];
  int     ansi_len  = 0;
  bool    in_escape = false;

  con_clear_full();
  menu_render();

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
                    for (int col=g_sb_col; col<COLS; col++)
                      { SB_CH(g_sb_total,col)=' '; SB_COL(g_sb_total,col)=COL_FG; }
                    if (g_scroll_offset==0)
                      {
                        sb_paint_row_locked(sb_live_view_row());
                        gfx_flush_row(sb_live_text_row());
                      }
                    nxmutex_unlock(&g_con_mutex);
                    break;

                  case 'D':   /* cursor left */
                    nxmutex_lock(&g_con_mutex);
                    for (int i=0;i<arg;i++) if (g_sb_col>0) g_sb_col--;
                    nxmutex_unlock(&g_con_mutex);
                    break;

                  case 'C':   /* cursor right */
                    nxmutex_lock(&g_con_mutex);
                    for (int i=0;i<arg;i++) if (g_sb_col<COLS-1) g_sb_col++;
                    nxmutex_unlock(&g_con_mutex);
                    break;

                  case 'A':   /* cursor up — retract write pointer */
                    nxmutex_lock(&g_con_mutex);
                    for (int i=0;i<arg;i++) if (g_sb_total>0) g_sb_total--;
                    g_sb_col=0;
                    nxmutex_unlock(&g_con_mutex);
                    break;

                  case 'B':   /* cursor down */
                    nxmutex_lock(&g_con_mutex);
                    for (int i=0;i<arg;i++)
                      { sb_newline_locked(); }
                    nxmutex_unlock(&g_con_mutex);
                    break;

                  case 'm':   /* SGR colour — ignore */
                    break;

                  default: break;
                }
              ansi_len=0;
            }
          continue;
        }

      if (c==ANSI_ESC)
        { in_escape=true; ansi_len=0; ansi_buf[ansi_len++]=(uint8_t)c; continue; }
      if (c==0x0c) { con_clear(); continue; }

      con_putc(c, COL_FG);
    }

  return OK;
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

#ifdef CONFIG_FS_PROCFS
  nx_mount(NULL,"/proc","procfs",0,NULL);
#endif

#ifdef CONFIG_ESP32S3_SPIFLASH
  ret=board_spiflash_init();
  if (ret<0) syslog(LOG_ERR,"board_spiflash_init failed: %d\n",ret);
#endif

#if defined(CONFIG_LCD)
  ret=board_lcd_initialize();
  if (ret<0) return ret;

  ringdev_init(&g_kbd_ring);
  ringdev_init(&g_out_ring);
  g_out_ring_ready=true;

#ifdef CONFIG_SYSLOG
  syslog_channel_register(&g_lcd_syslog_channel);
#endif

#if defined(CONFIG_ESPRESSIF_ADC)||defined(CONFIG_ESP32S3_ADC)
  ret=board_adc_init();
  if (ret<0) syslog(LOG_ERR,"board_adc_init failed: %d\n",ret);
#endif

  ret=register_driver("/dev/cardputer_kbd",&g_kbd_ops,0666,&g_kbd_ring);
  if (ret<0) return ret;
  ret=register_driver("/dev/cardputer_out",&g_out_ops,0666,&g_out_ring);
  if (ret<0) return ret;

  kthread_create("lcd_cons",110,4096,lcd_cons_thread,NULL);
  kthread_create("kbd_nsh", 100,4096,kbd_nsh_thread, NULL);
#endif

  UNUSED(ret);
  return OK;
}

struct lcd_dev_s *board_lcd_getdev(int lcddev) { return g_lcddev; }
int up_fbinitialize(int display)               { return OK; }