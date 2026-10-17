#!/usr/bin/env python3
"""Fix the ESP32-S3 legacy (IDF-bootloader) linker script in a NuttX tree.

Since the 2026-02 ESP-IDF HAL update, sleep_modes.c (and others) put data in
.rtc.force_slow.* / .rtc.force_fast.* sections.  esp32s3_sections.ld (simple
boot) places them in RTC memory, but legacy_sections.ld has no rule for them,
so the linker drops them as orphans into the flash DROM range, outside the
image.  The first write faults (EXCCAUSE 15) right after the bootloader
jumps to NuttX -- which is why a legacy image "does nothing" under M5
Launcher.

usage:  python3 fix_legacy_ld.py ~/nuttxspace/nuttx
"""
import re
import sys
from pathlib import Path

FORCE_FAST = """  /* RTC_FAST_ATTR data (ESP-IDF HAL); see fix_legacy_ld.py */

  .rtc.force_fast :
  {
    . = ALIGN(4);
    _rtc_force_fast_start = ABSOLUTE(.);
    *(.rtc.force_fast .rtc.force_fast.*)
    . = ALIGN(4);
    _rtc_force_fast_end = ABSOLUTE(.);
  } > rtc_slow_seg

"""


def main():
    top = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    ld = top / "boards/xtensa/esp32s3/common/scripts/legacy_sections.ld"
    if not ld.exists():
        sys.exit(f"error: {ld} not found (pass the nuttx directory)")
    s = ld.read_text()
    if "rtc.force_slow" in s:
        print(f"{ld}: already fixed")
        return

    m = re.search(r"^([ \t]*)\.rtc\.data\s*:\s*\n[ \t]*\{[ \t]*\n", s, re.M)
    if not m:
        sys.exit("error: could not find the .rtc.data section; edit by hand")

    body_indent = m.group(1) + "  "
    insert_at = m.end()
    nxt = re.match(r"[ \t]*\. = ALIGN\(4\);[ \t]*\n", s[insert_at:])
    if nxt:
        insert_at += nxt.end()

    s = (s[:m.start()] + ("" if ".rtc.force_fast" in s else FORCE_FAST) +
         s[m.start():insert_at] +
         f"{body_indent}*(.rtc.force_slow .rtc.force_slow.*)\n" +
         s[insert_at:])
    ld.with_suffix(".ld.orig").write_text(ld.read_text())
    ld.write_text(s)
    print(f"{ld}: fixed (backup: legacy_sections.ld.orig).  Rebuild with "
          f"'make -j'.")


if __name__ == "__main__":
    main()
