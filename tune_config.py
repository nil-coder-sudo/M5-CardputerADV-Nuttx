#!/usr/bin/env python3
"""Apply the low-RAM Wi-Fi tuning to a NuttX .config.

usage: python3 tune_config.py [path/to/.config]      (default: ./.config)

Edits the file in place (a .config.orig backup is written first), then run
`make olddefconfig` in the nuttx tree so Kconfig re-validates everything.
"""
import re
import shutil
import sys

# name -> value   (value None = "is not set")
TUNE = {
    # --- Wi-Fi driver buffers -------------------------------------------
    # Each static RX buffer is ~1.6 KB allocated forever at esp_wifi_init().
    "ESPRESSIF_WIFI_STATIC_RX_BUFFER_NUM": "4",      # was 10  (-9.6 KB)
    "ESPRESSIF_WIFI_DYNAMIC_RX_BUFFER_NUM": "16",    # was 32  (peak cap)
    "ESPRESSIF_WIFI_DYNAMIC_TX_BUFFER_NUM": "16",    # was 32  (peak cap)
    "ESPRESSIF_WIFI_RX_MGMT_BUF_NUM_DEF": "3",       # was 5
    # Keep AMPDU TX enabled: NuttX's esp_wifi_api.c reads
    # CONFIG_ESPRESSIF_WIFI_TX_BA_WIN unconditionally (and feeds it into
    # rx_ba_win), so turning AMPDU TX off breaks the build.  Shrink both
    # windows instead; keep them <= STATIC_RX_BUFFER_NUM.
    "ESPRESSIF_WIFI_RX_BA_WIN": "4",                 # was 6
    "ESPRESSIF_WIFI_TX_BA_WIN": "4",                 # was 6
    "ESPRESSIF_WIFI_SCAN_RESULT_SIZE": "2048",       # was 4096 (transient)
    "WIRELESS_WAPI_SCAN_MAX_DATA": "2048",
    "ESPRESSIF_WIFI_ENABLE_SAE_PK": None,            # rarely used WPA3 bits
    "ESPRESSIF_WIFI_ENABLE_WPA3_OWE_STA": None,

    # --- Network stack ----------------------------------------------------
    # 590 drops every full-size (1500 B) Wi-Fi frame.  Required for Wi-Fi.
    "NET_ETH_PKTSIZE": "1514",
    # 8 x 196 B could not even hold one 1514 B frame.
    "IOB_BUFSIZE": "400",
    "IOB_NBUFFERS": "24",                            # ~10 KB total
    "IOB_NCHAINS": "12",
    "NET_TCP_PREALLOC_CONNS": "4",                   # was 8
    "NET_UDP_PREALLOC_CONNS": "4",                   # was 8
    "NET_ICMP_PREALLOC_CONNS": "2",                  # was 4
    "NET_TCPBACKLOG_CONNS": "2",                     # was 8
    "NET_MAX_LISTENPORTS": "4",                      # was 20
    "NET_PREALLOC_DEVIF_CALLBACKS": "8",             # was 16
    "NET_ARPTAB_SIZE": "8",                          # was 16
    "NETDB_DNSCLIENT_ENTRIES": "4",                  # was 8
    "NETDB_DNSCLIENT_NAMESIZE": "64",                # was 256 (-2 KB cache)

    # --- System -----------------------------------------------------------
    # REQUIRED for Wi-Fi: the ESP Wi-Fi adapter creates recursive mutexes;
    # without this pthread_mutexattr_settype() returns ENOSYS and
    # esp_wifi_init() fails with ESP_ERR_NO_MEM (board_wlan_init: -12).
    "PTHREAD_MUTEX_TYPES": "y",
    # REQUIRED for Wi-Fi: without it netdev_register() rejects the
    # NET_LL_IEEE80211 link type ("Unrecognized link type: 5", -22) and
    # wlan0 is never created.  Also makes netinit use wlan0, not eth0.
    "DRIVERS_IEEE80211": "y",
    "SYSLOG_MAX_CHANNELS": "3",      # console + crash-screen tail + LCD mirror (debug: 4, +USB)
    "TASK_NAME_SIZE": "15",          # was 31 (per task)
    "STACK_COLORATION": "y",         # `ps` shows real stack use -> tune
    "SYSTEM_PING": "y",              # quick Wi-Fi sanity check

    # --- Display ----------------------------------------------------------
    # 1 MHz made a full redraw take ~0.5 s.  The panel runs fine at 40 MHz
    # on the Cardputer; drop to 20000000 if you ever see corruption.
    "LCD_ST7789_FREQUENCY": "40000000",
    # Cardputer panel with the stock NuttX ST7789 driver (verified on the
    # device): the driver swaps X/Y offsets in landscape, and the panel
    # needs display inversion on.
    "LCD_ST7789_XOFFSET": "53",
    "LCD_ST7789_YOFFSET": "40",
    "LCD_ST7789_INVCOLOR": "y",
}


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else ".config"
    shutil.copyfile(path, path + ".orig")
    lines = open(path).read().split("\n")
    seen = set()
    pat = re.compile(r"^(?:# )?CONFIG_([A-Za-z0-9_]+)(?:=| is not set)")
    for i, line in enumerate(lines):
        m = pat.match(line)
        if not m or m.group(1) not in TUNE:
            continue
        name = m.group(1)
        val = TUNE[name]
        lines[i] = (f"# CONFIG_{name} is not set" if val is None
                    else f"CONFIG_{name}={val}")
        seen.add(name)
    for name, val in TUNE.items():
        if name not in seen and val is not None:
            lines.append(f"CONFIG_{name}={val}")
    open(path, "w").write("\n".join(lines))
    print(f"tuned {len(TUNE)} options in {path} (backup: {path}.orig)")


if __name__ == "__main__":
    main()
