# Cardputer ADV — Hacker Green UI

A NuttX firmware UI for the **M5 Cardputer ADV (ESP32‑S3)** with a fixed
phosphor‑green theme (`#00FF41`). It provides a small launcher menu and a set
of built‑in apps: a screen tester, a keyboard tester, an interactive NSH
terminal with scrollback, a file manager, and a battery monitor.

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

- **Launcher menu** with five built‑in apps.
- **Live status bar** showing the current app title and a battery gauge
  (percentage + colour state), refreshed roughly every 2 seconds.
- **NSH terminal** wired to a ring‑buffer character device, with a
  **200‑line scrollback** history you can page through.
- **VT100 handling** for the shell (cursor moves, erase line/screen, color SGR
  sequences are consumed).
- **Fast single‑row screen flushes** on the hot path for snappy typing, with
  full‑band repaints only on newline/scroll.
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

Make sure your build is configured so that `CONFIG_LCD`, the ST7789 driver,
the ADC, and SPIFLASH (if used) are enabled. The bring‑up code registers two
character devices — `/dev/cardputer_kbd` and `/dev/cardputer_out` — and
launches the `lcd_cons` and `kbd_nsh` kernel threads.

### 3. Configure

```bash
cd nuttx
./tools/configure.sh esp32s3-devkit:nsh
make menuconfig   # enable LCD / ST7789 / ADC / SPIFLASH as needed
```

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
| No key input | The TCA8418 didn't initialise (software I²C on GPIO8/9). Check the `kbd_nsh` thread returned `-ENODEV` in the serial log. |
| Battery shows **N/A** / FAILED | `/dev/adc0` is missing or ADC isn't enabled — enable the ESP32‑S3 ADC and re‑flash. |
| Typing feels laggy / repeats | A ~100 ms per‑key cooldown debounces the matrix; this is expected, not a fault. |
| Flash fails to connect | Hold **G0** during reset to force download mode, lower the baud rate, or try a different USB cable/port. |

---

## Technical Notes

- **Framebuffer:** 240 × 135 × 2 = 64,800 bytes (full RGB565 frame held in RAM).
- **Scrollback store:** 200 × 30 × 3 = 18,000 bytes of char + colour history.
- **Display flush discipline:**
  - Printable characters flush only the single dirty text row (fast path).
  - Newlines repaint the whole terminal band (all 7 rows) because the view is
    bottom‑anchored — every newline shifts every visible row up by one.
  - Full‑screen presents are reserved for menus, app screens, and scrollback
    repaints.
- **Threads:** `lcd_cons` (VT100 parser → scrollback renderer) and `kbd_nsh`
  (keyboard scan → shell input / UI dispatch).
- **Devices:** `/dev/cardputer_kbd` (shell I/O) and `/dev/cardputer_out`
  (display output ring). Syslog is mirrored to the screen when enabled.

---

## License & Credits

**Maker:** Nelson

Built on **Apache NuttX** for the **M5 Cardputer ADV (ESP32‑S3 / Stamp‑S3A)**.
Embedded font is JetBrains Mono (8 × 16). Use, modify, and share at your own
discretion; verify hardware pin assignments against your own board before
flashing.
