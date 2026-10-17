#!/usr/bin/env python3
"""Wrap a legacy-format NuttX image into an M5 Launcher installable .bin.

Layout of the output (same as the merged images most Launcher firmwares ship):

    0x0000   bootloader-esp32s3.bin if given with -b, else 0xFF
    0x8000   partition table: nvs (data) + factory app at 0x10000
    0x10000  nuttx.bin (ESP-IDF app image, built with
             CONFIG_ESP32S3_APP_FORMAT_LEGACY=y)

Launcher reads the partition table at 0x8000, takes the app from 0x10000
and installs it into one of its own OTA partitions; it ignores 0x0-0x8000.

usage:
    python3 mk_launcher_bin.py nuttx.bin                 # -> nuttx-launcher.bin
    python3 mk_launcher_bin.py nuttx.bin -o cardputer.bin
    python3 mk_launcher_bin.py nuttx.bin -b bootloader-esp32s3.bin
"""
import argparse
import hashlib
import os
import struct
import sys

PT_OFFSET = 0x8000
APP_OFFSET = 0x10000
ALIGN = 0x10000


def pt_entry(ptype, subtype, offset, size, label, flags=0):
    lbl = label.encode()[:16].ljust(16, b"\0")
    return struct.pack("<2sBBII16sI", b"\xaa\x50", ptype, subtype,
                       offset, size, lbl, flags)


def check_app(img):
    if len(img) < 24 or img[0] != 0xE9:
        sys.exit("error: not an ESP app image (first byte must be 0xE9)")
    nseg = img[1]
    chip_id = struct.unpack_from("<H", img, 12)[0]
    if chip_id != 9:
        sys.exit(f"error: image is for chip id {chip_id}, not ESP32-S3 (9)")
    # A simple-boot image keeps code in RAM segments only; the IDF
    # bootloader used by Launcher needs flash-mapped (IROM/DROM) segments.
    pos, mapped = 24, False
    for _ in range(nseg):
        addr, size = struct.unpack_from("<II", img, pos)
        if 0x3C000000 <= addr < 0x3E000000 or 0x42000000 <= addr < 0x44000000:
            mapped = True
        pos += 8 + size
    if not mapped:
        sys.exit("error: no flash-mapped segments -- this is a simple-boot "
                 "image.  Rebuild with CONFIG_ESP32S3_APP_FORMAT_LEGACY=y")
    return nseg


def check_elf(path):
    """Refuse images whose ELF has orphan .rtc.force_* output sections:
    those crash at boot (see fix_legacy_ld.py)."""
    try:
        e = open(path, "rb").read()
    except OSError:
        return
    if e[:4] != b"\x7fELF" or e[4] != 1:
        return
    shoff, = struct.unpack_from("<I", e, 0x20)
    shentsize, shnum, shstrndx = struct.unpack_from("<HHH", e, 0x2e)
    stro = struct.unpack_from("<I", e, shoff + shstrndx * shentsize + 16)[0]
    bad = []
    for i in range(shnum):
        nm, = struct.unpack_from("<I", e, shoff + i * shentsize)
        name = e[stro + nm:e.index(b"\0", stro + nm)].decode()
        addr, = struct.unpack_from("<I", e, shoff + i * shentsize + 12)
        if name.startswith((".rtc.force_slow.", ".rtc.force_fast.")) or (
                name.startswith(".rtc.force_") and addr < 0x50000000):
            bad.append(f"{name}@0x{addr:x}")
    if bad:
        sys.exit("error: " + ", ".join(bad) + " is an orphan section in "
                 f"{path}; this image crashes at boot.\n"
                 "       run: python3 fix_legacy_ld.py <nuttx dir>  and rebuild")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("app", help="nuttx.bin built in legacy IDF format")
    ap.add_argument("-o", "--output", default="nuttx-launcher.bin")
    ap.add_argument("-b", "--bootloader", help="optional bootloader for 0x0")
    args = ap.parse_args()

    app = open(args.app, "rb").read()
    nseg = check_app(app)
    elf = os.path.join(os.path.dirname(os.path.abspath(args.app)), "nuttx")
    check_elf(elf)
    app_part = (len(app) + ALIGN - 1) // ALIGN * ALIGN

    table = (pt_entry(0x01, 0x02, 0x9000, 0x5000, "nvs") +
             pt_entry(0x00, 0x00, APP_OFFSET, app_part, "factory"))
    table += b"\xeb\xeb" + b"\xff" * 14 + hashlib.md5(table).digest()

    out = bytearray(b"\xff" * (APP_OFFSET + len(app)))
    if args.bootloader:
        boot = open(args.bootloader, "rb").read()
        if len(boot) > PT_OFFSET:
            sys.exit("error: bootloader is larger than 0x8000")
        out[0:len(boot)] = boot
    out[PT_OFFSET:PT_OFFSET + len(table)] = table
    out[APP_OFFSET:] = app
    open(args.output, "wb").write(out)

    print(f"{args.output}: {len(out)} bytes")
    print(f"  0x{PT_OFFSET:05x} partition table (nvs, factory 0x{APP_OFFSET:x}"
          f" size 0x{app_part:x})")
    print(f"  0x{APP_OFFSET:05x} app image {len(app)} bytes, {nseg} segments")
    if not args.bootloader:
        print("  note: no bootloader at 0x0 -- install with Launcher only, "
              "do not flash this file at 0x0 with esptool")


if __name__ == "__main__":
    main()
