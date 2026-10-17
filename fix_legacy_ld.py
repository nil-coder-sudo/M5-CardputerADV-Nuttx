#!/usr/bin/env python3
"""Fix the ESP32-S3 legacy (IDF-bootloader) linker script in a NuttX tree.

Since the 2026-02 ESP-IDF HAL update, sleep_modes.c (and others) put data in
.rtc.force_slow.* / .rtc.force_fast.* sections.  esp32s3_sections.ld (simple
boot) places them in RTC memory, but legacy_sections.ld has no rule for them,
so the linker drops them as orphans into the flash DROM range, outside the
image.  The first write faults (EXCCAUSE 15) right after the bootloader
jumps to NuttX -- which is why a legacy image "does nothing" under M5
Launcher.

It also fixes esp32s3_wdt_early_deinit() (arch/xtensa/src/esp32s3/
esp32s3_wdt.c).  The IDF bootloader -- and esp_restart() in the firmware
that started us, e.g. M5 Launcher -- leaves the RTC watchdog running and
write-protected.  NuttX clears the enable bit without writing the unlock
key first, so the watchdog keeps running and resets the chip about a second
after NuttX starts.

Third, the same HAL update renamed and split the objects that must run
from IRAM while the flash cache is off (esp_cache_utils, mmu_hal,
mspi_timing_*, ...).  esp32s3_sections.ld (simple boot) was updated,
legacy_sections.ld was not: e.g. esp_cache_suspend_ext_mem_cache() stays
in flash, and the first flash operation (reading the flash ID in
init_flash) suspends the cache from code that lives behind that cache.
Whether the CPU then stalls depends on what happens to be cached, which is
why one build hung and another did not.  The missing IRAM/DRAM placement
rules are copied from esp32s3_sections.ld.

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


WDT_FIXED = """void esp32s3_wdt_early_deinit(void)
{
  uint32_t regval;

  /* Unlock first: the bootloader / esp_restart() leave RWDT write-protected
   * (see fix_legacy_ld.py in M5-CardputerADV-Nuttx).
   */

  putreg32(0x50d83aa1, RTC_CNTL_WDTWPROTECT_REG);
  regval  = getreg32(RTC_CNTL_WDTCONFIG0_REG);
  regval &= ~(RTC_CNTL_WDT_EN | (1 << 12));   /* EN, FLASHBOOT_MOD_EN */
  putreg32(regval, RTC_CNTL_WDTCONFIG0_REG);
  putreg32(0, RTC_CNTL_WDTWPROTECT_REG);
}"""


def fix_wdt(top):
    src = top / "arch/xtensa/src/esp32s3/esp32s3_wdt.c"
    if not src.exists():
        print(f"{src}: not found, skipped")
        return
    s = src.read_text()
    if "0x50d83aa1, RTC_CNTL_WDTWPROTECT_REG" in s:
        print(f"{src}: already fixed")
        return
    m = re.search(r"void esp32s3_wdt_early_deinit\(void\)\n\{.*?\n\}", s, re.S)
    if not m:
        print(f"{src}: esp32s3_wdt_early_deinit() not found, skipped")
        return
    src.with_suffix(".c.orig").write_text(s)
    src.write_text(s[:m.start()] + WDT_FIXED + s[m.end():])
    print(f"{src}: fixed (backup: esp32s3_wdt.c.orig)")


SYNC_TAG = "/* rules synced from esp32s3_sections.ld by fix_legacy_ld.py */"


def section_rules(text, name):
    """Unconditional input-section rules of output section `name`."""
    m = re.search(r"^  " + re.escape(name) + r"\b[^\n]*\n  \{\n(.*?)^  \}",
                  text, re.M | re.S)
    if not m:
        return None, None
    rules, depth = [], 0
    for line in m.group(1).split("\n"):
        st = line.strip()
        if st.startswith("#if"):
            depth += 1
        elif st.startswith("#endif"):
            depth -= 1
        elif depth == 0 and re.match(r"(ARCHLIB|\*lib)[^ ]*:", st) and \
                st.endswith(")"):
            rules.append(st)
    return rules, m


def sync_rules(top, ld, s):
    simple = (top / "boards/xtensa/esp32s3/common/scripts/"
              "esp32s3_sections.ld").read_text()
    if SYNC_TAG in s:
        return s, 0
    added = 0
    for name, anchor in ((".iram0.text", "    *(.wifirxiram"),
                         (".dram0.data", "    _edata = ABSOLUTE(.);")):
        want, _ = section_rules(simple, name)
        have, m = section_rules(s, name)
        if not want or have is None:
            sys.exit(f"error: no {name} section; edit {ld} by hand")
        norm = {" ".join(r.split()) for r in have}
        new = [r for r in want if " ".join(r.split()) not in norm]
        body = m.group(1)
        i = body.find(anchor)
        if i < 0:
            sys.exit(f"error: anchor for {name} not found; edit {ld} by hand")
        block = ("    " + SYNC_TAG + "\n" +
                 "".join(f"    {r}\n" for r in new) + "\n")
        body = body[:i] + block + body[i:]
        s = s[:m.start(1)] + body + s[m.end(1):]
        added += len(new)
    return s, added


def main():
    top = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    fix_wdt(top)
    ld = top / "boards/xtensa/esp32s3/common/scripts/legacy_sections.ld"
    if not ld.exists():
        sys.exit(f"error: {ld} not found (pass the nuttx directory)")
    s = ld.read_text()
    orig = s
    s, added = sync_rules(top, ld, s)
    if added:
        print(f"{ld}: {added} IRAM/DRAM placement rules added")
    if "rtc.force_slow" in s:
        if s != orig:
            if not ld.with_suffix(".ld.orig").exists():
                ld.with_suffix(".ld.orig").write_text(orig)
            ld.write_text(s)
        else:
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
    if not ld.with_suffix(".ld.orig").exists():
        ld.with_suffix(".ld.orig").write_text(orig)
    ld.write_text(s)
    print(f"{ld}: fixed (backup: legacy_sections.ld.orig).  Rebuild with "
          f"'make -j'.")


if __name__ == "__main__":
    main()
