# Cardputer ADV — Hacker Green UI

A NuttX firmware UI for the **M5 Cardputer ADV (ESP32‑S3)** with a fixed
phosphor‑green theme (`#00FF41`). It provides a small launcher menu and a set
of built‑in apps: a screen tester, a keyboard tester, an interactive NSH
terminal with scrollback, a file manager, a battery monitor, and a Wi‑Fi
manager (scan / join / DHCP). The UI runs without a full framebuffer, so the
whole firmware leaves ~70 KB more heap for Wi‑Fi than earlier versions.

> **Board:** this firmware targets the **Cardputer ADV** (the "Advance"
> version), built on the **Stamp‑S3A** core module (ESP32‑S3FN8) — not the
> original Cardputer. The ADV keeps the same 1.14" LCD and 56‑key keyboard but
> adds a larger 1750 mAh battery, a BMI270 IMU, ES8311 audio, an IR emitter,
> and a microSD slot.

**Maker:** Nelson

---

## Table of Contents

1. [Hardware](#hardware)
2. [Features](#features)
3. [Building & Flashing](#building--flashing)
4. [Powering On](#powering-on)
5. [Controls & Key Map](#controls--key-map)
6. [Menu Options](#menu-options)
7. [Using the Terminal](#using-the-terminal)
8. [Troubleshooting](#troubleshooting)
9. [Technical Notes](#technical-notes)
10. [License & Credits](#license--credits)

---

## Hardware

| Component   | Detail                                                              |
|-------------|---------------------------------------------------------------------|
| Core module | **Stamp‑S3A** (ESP32‑S3FN8, dual‑core Xtensa LX7 @ 240 MHz)         |
| Display     | ST7789, 1.14" IPS, 240 × 135 px, RGB565, SPI bus 2                  |
| Keyboard    | 56‑key (4 × 14 matrix) via **TCA8418** controller on **software I²C** (SDA = GPIO8, SCL = GPIO9) |
| Battery     | 1750 mAh Li‑ion, side **ON/OFF** switch; read via `/dev/adc0`, channel 9, ×2 divider |
| RTOS        | Apache NuttX                                                         |

**ADV‑only peripherals** (present on the board but **not used** by this
firmware yet): BMI270 6‑axis IMU, ES8311 audio codec + 1 W speaker + 3.5 mm
jack + MEMS microphone, IR emitter (GPIO44), microSD slot, HY2.0‑4P Grove port,
and the EXT 2.54‑14P expansion header. They're free for you to wire up.

The UI uses an 8 × 16 embedded font (30 columns × 8 rows of text), with the
top row reserved for the status bar and the lower 7 rows for app/terminal
content.

---

## Features

- **Launcher menu** with seven built‑in apps.
- **World Clock** — 16 cities with automatic daylight‑saving time, synced
  over NTP; the current time also appears in the status bar.
- **Wi‑Fi app** — scan, pick a network, type the passphrase on the
  keyboard, join, and get a DHCP lease. Status card shows SSID, IP and RSSI.
- **Low‑RAM banded renderer** — screens are drawn 16 pixel rows at a time
  into a 7.6 KB buffer instead of a 64.8 KB framebuffer.
- **Live status bar** showing the current app title and a battery gauge
  (percentage + colour state), refreshed roughly every 2 seconds.
- **NSH terminal** wired to a ring‑buffer character device, with a
  **200‑line scrollback** history you can page through.
- **VT100 handling** for the shell (cursor moves, erase line/screen, color SGR
  sequences are consumed).
- **One SPI transfer per 16‑row band**: a typed character costs one
  transfer, a newline eight.
- **Self‑contained vector graphics** (rounded rects, gradients, icons) — no
  external image assets.

---

## Building & Flashing

> These steps follow the standard NuttX ESP32‑S3 build flow. Adjust the board
> config name and serial port to match your setup.

### 1. Prerequisites

- A working **NuttX + nuttx‑apps** checkout.
- The **ESP32‑S3 Xtensa toolchain** on your `PATH`.
- **`esptool`** (`pip install esptool`).
- USB‑C cable connected to the Cardputer ADV, and the side switch set to **ON**.

### 2. Place the board file

This firmware lives in the board bring‑up source:

```
nuttx/boards/xtensa/esp32s3/esp32s3-devkit/src/esp32s3_bringup.c
```

In the same directory, `esp32s3-devkit.h` must use the Cardputer's LCD
pins (the devkit default is the ST7735 wiring, DC = 14, which leaves the
screen black because every command reaches the panel as data):

```c
#define GPIO_LCD_DC         34
#define GPIO_LCD_RST        33
```

The bring-up refuses to compile with any other DC pin.

Make sure your build is configured so that `CONFIG_LCD`, the ST7789 driver,
the ADC, and SPIFLASH (if used) are enabled. For Wi‑Fi the bring‑up calls
`board_wlan_init()` whenever `CONFIG_ESPRESSIF_WIFI` is set. The bring‑up code registers two
character devices — `/dev/cardputer_kbd` and `/dev/cardputer_out` — and
launches the `lcd_cons` and `kbd_nsh` kernel threads.

### 3. Configure

```bash
cd nuttx
./tools/configure.sh esp32s3-devkit:nsh
cp ../M5-CardputerADV-Nuttx/.config .config   # the tuned config from this repo
make olddefconfig                              # let Kconfig fill in defaults
```

The repo's `.config` is already tuned for Wi‑Fi on a no‑PSRAM board. If you
keep your own config, apply the same changes with:

```bash
python3 ../M5-CardputerADV-Nuttx/tune_config.py .config && make olddefconfig
```

See the comments in `tune_config.py` for what each setting does.

### 4. Build

```bash
make -j
```

This produces `nuttx.bin` (plus the bootloader and partition‑table images).

### 5. Flash

Put the Cardputer into download mode if needed (hold the **G0** button while
tapping reset), then:

```bash
make flash ESPTOOL_PORT=/dev/ttyACM0 ESPTOOL_BAUD=921600
```

> On Linux the port is usually `/dev/ttyACM0` or `/dev/ttyUSB0`; on macOS it
> looks like `/dev/cu.usbmodemXXXX`. The ESP32‑S3's native USB normally allows
> auto‑reset, so manual download mode is often unnecessary.

### 5b. (Alternative) Install through M5 Launcher

[M5 Launcher](https://github.com/bmorcelli/Launcher) installs app images into
its own OTA partitions and starts them with the ESP‑IDF 2nd‑stage
bootloader, so the image must be built in **legacy IDF format** (the default
"simple boot" image only runs when flashed at `0x0`):

```bash
# 1. fix NuttX's legacy linker script (once per NuttX tree, see below)
python3 ../M5-CardputerADV-Nuttx/fix_legacy_ld.py .

# 2. menuconfig: System Type → Bootloader and Image Configuration →
#                [*] Enable legacy IDF bootloader format
#    or:
echo CONFIG_ESP32S3_APP_FORMAT_LEGACY=y >> .config
make olddefconfig && make -j
```

**Why step 1:** since the 2026‑02 ESP‑IDF HAL update, NuttX's
`legacy_sections.ld` has no rule for the HAL's `.rtc.force_slow.*` data, so
the linker drops it into the flash address range outside the image. The
first write to it faults right after the bootloader starts NuttX, so the
screen stays dark. `fix_legacy_ld.py` adds the missing rules (it is a no‑op
on a tree that already has them), and `mk_launcher_bin.py` refuses to wrap an
image that still has the problem.

Then wrap it with the partition table Launcher expects (app at `0x10000`):

```bash
python3 ../M5-CardputerADV-Nuttx/mk_launcher_bin.py nuttx.bin   # -> nuttx-launcher.bin
```

Copy `nuttx-launcher.bin` to the SD card and install it from Launcher's
SD menu. The script refuses simple-boot images, which cannot run from an
OTA partition. **Do not run `make flash` in this mode** — it writes NuttX's own
bootloader and partition table and erases Launcher.

**Boot behaviour under Launcher** (from Launcher's source and its
bootloader): a power‑on reset or the reset button always starts Launcher; it
then starts the installed app with a software reset. That software reset does
not clear the ESP32‑S3 GPIO matrix, so Launcher's backlight PWM (GPIO38, set
to 0 % just before the reboot) and its hardware‑I2C keyboard pins (GPIO8/9)
would stay attached. The bring‑up takes these pins back first thing
(`board_reclaim_gpios()`), otherwise NuttX runs with a black screen and a
dead keyboard.

Under Launcher the saved Wi‑Fi networks go to the highest flash sector that
no partition uses; if Launcher has handed out the whole flash, networks are
not saved and the terminal says `(not saved)`.

**Required for Launcher: the early-init hook.** Add one line to
`esp32s3_board_initialize()` in
`boards/xtensa/esp32s3/esp32s3-devkit/src/esp32s3_boot.c`:

```c
void esp32s3_board_initialize(void)
{
  extern void cardputer_early_init(void);
  cardputer_early_init();
}
```

Launcher starts the app with `esp_restart()`, which resets the CPU but not
the interrupt matrix, the GPIO interrupt settings or most peripherals
(SPI2, I2C, LEDC, ADC ...). ESP-IDF apps clear these at start-up; NuttX does
not, so interrupt sources Launcher left routed to CPU interrupts (for
example its GPIO11 keyboard interrupt) fire into whatever NuttX later puts
on those CPU interrupts, and the system hangs with no message right after
it starts. `cardputer_early_init()` clears the interrupt matrix and GPIO
interrupts and resets those peripherals before the OS starts. It is
harmless when flashed normally.

`fix_legacy_ld.py` also brings `legacy_sections.ld` up to date with the
simple-boot script: since the HAL update the code that suspends the flash
cache (`esp_cache_utils`, `mmu_hal`, `mspi_timing_*`, ...) moved to new
object files that only the simple-boot script places in IRAM. In legacy
builds it stayed in flash, so the first flash access (reading the flash ID
in `init_flash`) suspended the cache from code behind that cache and the
CPU stalled -- or not, depending on what happened to be cached, which is
why one build hung and another booted. Costs ~8 KB of IRAM, the same as
simple boot.

`fix_legacy_ld.py` also patches NuttX's `esp32s3_wdt_early_deinit()`:
`esp_restart()` arms the RTC watchdog (about one second) and write-protects
it; NuttX cleared the enable bit without unlocking first, so the chip reset
about a second after NuttX started (the early-init screen, then a reboot).
`cardputer_early_init()` turns the watchdogs off as well, and in debug
builds the early-init screen shows the last reset cause (12 = software
restart from Launcher, 9/16 = RTC watchdog).

**Finding out where a boot stops (no PC needed).** Every bring-up step is
listed on a *NuttX boot* screen until the menu appears, so a boot that hangs
shows the last step reached. With the hook above (any build) you also get:

| What the screen shows | Meaning |
|---|---|
| stays completely dark | NuttX never got control: install/bootloader problem |
| blue **NuttX started (early init)** only | crashed or hung between early init and the bring-up |
| green **NuttX boot** list, stops at a line | hung in that step |
| red **\*\*\* CRASH \*\*\*** screen | a crash: task, file:line, PC and the last log lines |

The early screen and the crash screen drive the LCD with bit-banged GPIOs,
so they work even when the SPI/LCD drivers are not up or are what crashed.
`CONFIG_DEBUG_FEATURES=y` adds a 3 s pause on the boot list, mirrors syslog
to the USB cable and puts the last log lines on the crash screen.

### 6. (Optional) Serial monitor

A serial console is handy for debugging the boot sequence:

```bash
make monitor ESPTOOL_PORT=/dev/ttyACM0
```

---

## Powering On

After flashing, slide the **ON/OFF switch** on the side of the Cardputer ADV to
**ON** (or press reset), then:

1. The display initialises and clears to the dark green background.
2. The **launcher menu** appears with the status bar at the top.
3. Use the keys below to navigate.

---

## Controls & Key Map

The Cardputer keyboard has a **fn** modifier used to reach the arrow layer, a
**shift** key, a **ctrl** key, and an **OPT** key used as a universal "back".

### Universal

| Key        | Action                                            |
|------------|---------------------------------------------------|
| **OPT**    | Return to the launcher menu from any app          |
| **ENTER**  | Select / confirm                                  |

### Arrow / navigation layer (hold **fn**)

| Combo      | Direction |
|------------|-----------|
| `fn` + `i` | **UP**    |
| `fn` + `k` | **DOWN**  |
| `fn` + `j` | **LEFT**  |
| `fn` + `l` | **RIGHT** |

> In the **menu**, **ScreenTest**, and **Files** apps, `i` / `;` also act as
> UP and `k` / `.` also act as DOWN **without** holding `fn`, so simple
> up/down browsing is one‑handed. In the **Terminal**, arrows always require
> `fn` so that normal typing isn't intercepted.

---

## Menu Options

Navigate the list with UP/DOWN and press **ENTER** to open an app. **OPT**
always brings you back here.

### 1. ScreenTest

Cycles through full‑screen solid colours to check the panel for dead pixels or
tint issues. A centered card shows the colour name and its index.

- **UP / DOWN** — previous / next colour
- Colours: WHITE, RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA
- **OPT** — back to menu

### 2. Keyboard

A live key inspector. Press any key and it shows the decoded character (or
named key such as ENTER / TAB / BACKSPACE / arrow), the raw scan code, and the
state of the **SH** (shift), **CT** (ctrl), and **FN** modifier chips.

- Useful for verifying the key matrix and modifier latching.
- **OPT** — back to menu

### 3. Terminal

An interactive **NSH** shell rendered on the LCD with command history and
scrollback. See [Using the Terminal](#using-the-terminal) below.

### 4. Files

A simple file browser starting at root (`/`).

- **UP / DOWN** — move the selection
- **ENTER** — open the highlighted directory (`..` goes up one level)
- Directories and files use distinct icons; a counter (e.g. `3/12`) appears
  when the listing is longer than the visible window.
- **OPT** — back to menu

### 5. Battery

Reads the battery voltage from the ADC and shows a large percentage gauge,
the raw voltage in millivolts, and a status line (Full / OK / Low — charge
soon). The view auto‑refreshes about once per second.

- If the ADC device is missing or unreadable, the app shows a diagnostic with
  the `errno` and sample info instead of crashing.
- **OPT** — back to menu

### 6. Wi‑Fi

The top card shows the current link (SSID, IP address, RSSI and a signal
icon); it refreshes every 3 seconds. Below it is the scan list — up to 8
networks, strongest first, duplicates and hidden SSIDs removed.

| Key | Action |
|-----|--------|
| **s** | Scan for networks (takes a couple of seconds) |
| **r** | Refresh link status |
| **UP / DOWN** | Select a network (`i`/`;` and `k`/`.` work without `fn`) |
| **ENTER** | Join: saved networks connect at once, others open the passphrase prompt |
| **f** | Forget the selected saved network |

In the passphrase prompt:

| Key | Action |
|-----|--------|
| any printable key | Type (8–63 characters; leave empty for an open network) |
| **TAB** | Show / hide the passphrase |
| **BACKSPACE** | Delete |
| **ENTER** | Connect |
| `fn` + `j` | Back to the list |

**Saved networks.** After a successful join the passphrase is stored in the
last 4 KB sector of the SPI flash (up to 4 networks, most recent first,
CRC‑checked; a failed join is never saved). Saved networks show a dot in the
scan list. At boot a one‑shot background task tries the saved networks,
runs `renew wlan0` and, once it has an address, sets the clock over NTP —
so after a reboot Wi‑Fi and the time come back by themselves. If a saved
passphrase stops working, the prompt opens so you can type the new one.
Passphrases are stored in plain text: anyone with the device and `esptool`
can read them back.

On success the firmware switches to the **Terminal** and runs
`renew wlan0`, so you can watch the DHCP lease arrive. The passphrase is
handed straight to the driver with ioctls — it never goes through the shell
and is wiped from memory afterwards. The app's state (~430 B) only exists
on the heap while the app is open.

### 7. World Clock

Shows the home city's time in large digits, the date, and a scrollable list
of 16 cities (Taipei, Tokyo, Singapore, Bangkok, New Delhi, Dubai, Moscow,
Berlin, London, UTC, New York, Chicago, Los Angeles, Honolulu, Sydney,
Auckland) with each city's time and its offset from home. Daylight‑saving
time is applied automatically for Europe, the US, Australia and New Zealand.

| Key | Action |
|-----|--------|
| **UP / DOWN** | Move through the city list |
| **ENTER** | Make the selected city the home city |
| **n** | Sync with NTP now |

The Cardputer has no battery‑backed clock, so the time is unknown after
boot. The first time you open World Clock with Wi‑Fi connected it syncs
automatically (`pool.ntp.org`, falling back to `time.google.com` and
`time.cloudflare.com`). Once synced, the status bar shows `HH:MM` and NSH's
`date` command shows the correct time until the next reboot. The footer shows
`NTP ok`, `sync..`, `no WiFi` or `NTP err`.

### Wi‑Fi from the terminal

You can also manage Wi‑Fi from the terminal with the stock tools:

```
wapi scan wlan0
wapi psk wlan0 <passphrase> 3
wapi essid wlan0 <ssid> 1
renew wlan0
ifconfig
ping 8.8.8.8
```

---

## Using the Terminal

The Terminal app connects the on‑screen display to the NSH shell. Type
commands and press **ENTER** to run them.

### Scrollback

The firmware keeps the last **200 lines** of output. The bottom of the screen
is the live prompt; older output scrolls up off the top as new lines arrive.

| Combo         | Action                                              |
|---------------|-----------------------------------------------------|
| `fn` + `UP`   | Scroll **back** into history                        |
| `fn` + `DOWN` | Scroll **forward** toward the live view             |

While you are scrolled back, the status bar shows a `↑N` indicator (where `N`
is how many lines you are above the live view). New output keeps accumulating
in the background **without** disturbing your scroll position. Pressing **any
non‑scroll key** snaps you back to the live view first, then routes that key to
the shell.

### Shell keys

- **Arrow keys** (`fn` + `i/j/k/l`) send VT100 cursor escapes to the shell, so
  command‑line editing and history recall work as expected.
- **BACKSPACE**, **TAB**, and **Ctrl‑combos** (e.g. `Ctrl‑C`) are passed
  through to NSH.

### Layout note

The terminal is **bottom‑anchored**: when you first enter it, the prompt
appears near the bottom of the screen and output fills upward until the screen
is full, after which it scrolls. This is intentional, not a glitch.

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| Blank / black screen | Check `CONFIG_LCD` and the ST7789 driver are enabled; verify SPI bus 2 wiring and the reset/backlight GPIOs in `board_lcd_initialize`. |
| `gfx_init failed` in the log | The LCD plane exposes neither `putarea` nor `putrun`; confirm the ST7789 driver build. |
| No key input | The TCA8418 isn't answering on GPIO8/9. The boot screen shows `kbd: no TCA8418, retrying`; the driver keeps retrying (with an I²C bus recovery each time). |
| Battery shows **N/A** / FAILED | `/dev/adc0` is missing or ADC isn't enabled — enable the ESP32‑S3 ADC and re‑flash. |
| Typing feels laggy / repeats | A ~100 ms per‑key cooldown debounces the matrix; this is expected, not a fault. |
| Flash fails to connect | Hold **G0** during reset to force download mode, lower the baud rate, or try a different USB cable/port. |
| Wi‑Fi app says `wlan0 is down` / no device | `CONFIG_ESPRESSIF_WIFI` is off, or `board_wlan_init` failed — check the serial log. |
| `join failed (-110)` | Timed out: wrong passphrase or AP out of range. Raise `CONFIG_ESPRESSIF_WIFI_CONNECT_TIMEOUT` for slow APs. |
| Joined but no IP | Check the `renew wlan0` output in the terminal; run it again if the AP was slow. |
| Garbage on screen after the update | The SPI clock is now 40 MHz. Set `CONFIG_LCD_ST7789_FREQUENCY=20000000` and rebuild. |
| Random crashes after shrinking stacks | Run `ps` (stack coloration is on) and raise `CARDPUTER_LCDCONS_STACK` / `CARDPUTER_KBD_STACK` at the top of `esp32s3_bringup.c`. |

---

## Technical Notes

- **Banded renderer:** no framebuffer. Every screen is a *compose*
  callback; `gfx_render()` runs it once per 16‑row band into a
  240 × 16 × 2 = 7,680‑byte buffer and pushes each band with one `putarea()`.
  Drawing primitives clip to the current band, so app code just draws the
  whole screen.
- **Scrollback store:** 200 × 30 = 6,000 bytes (characters only — the old
  colour plane always held the same colour).
- **File manager:** re‑reads the directory and keeps only the 6 visible
  names, instead of caching 64 entries.
- **RAM, measured on the linked firmware** (NuttX master, Wi‑Fi on):

  | | old UI | this version |
  |---|---:|---:|
  | UI static RAM (`.bss`/`.data`) | 88.8 KB | 16.6 KB |
  | Whole‑image DRAM used | 184.3 KB | 117.5 KB |
  | Wi‑Fi static RX buffers (heap) | 16 KB | 6.4 KB |
  | UI thread stacks (heap) | 8 KB | 6.5 KB |

  Net: roughly **78 KB more free heap** with Wi‑Fi running. The tuned
  config deliberately *adds* ~8 KB of IOBs so full 1500‑byte Wi‑Fi frames
  can be received at all (the old 8 × 196 B pool could not hold one).
- **Display flush discipline:**
  - Printable characters flush only the live text row (one band).
  - Newlines repaint the terminal area (8 bands) because the view is
    bottom‑anchored — every newline shifts every visible row up by one.
  - Full‑screen renders are reserved for menus, app screens, and scrollback
    repaints.
- **No busy‑waiting:** the keyboard thread sleeps between 20 ms polls, and
  the soft‑I²C no longer disables interrupts (which starved the Wi‑Fi MAC).
- **Threads:** `lcd_cons` (VT100 parser → scrollback renderer, 3 KB stack)
  and `kbd_nsh` (keyboard scan → shell input / UI dispatch, Wi‑Fi ioctls,
  3.5 KB stack).
- **Devices:** `/dev/cardputer_kbd` (shell I/O) and `/dev/cardputer_out`
  (display output ring). Syslog is mirrored to the screen when enabled.

---

## License & Credits

**Maker:** Nelson

Built on **Apache NuttX** for the **M5 Cardputer ADV (ESP32‑S3 / Stamp‑S3A)**.
Embedded font is JetBrains Mono (8 × 16). Use, modify, and share at your own
discretion; verify hardware pin assignments against your own board before
flashing.
