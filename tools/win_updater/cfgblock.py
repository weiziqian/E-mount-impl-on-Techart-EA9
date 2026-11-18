#!/usr/bin/env python3
"""
cfgblock.py -- the lens configuration block in the release image.

The firmware keeps its configuration in one struct, struct lens_cfg in
src/lens_cfg.h, starting with the magic "CfgAnchor\\0".  The updater finds it
by that magic and overwrites it; this script is the release-time check that it
can, and the reference for the encoding (pack() must match Encode() in
src/LensConfig.cs).

  cfgblock.py IMAGE             check IMAGE and print its block
  cfgblock.py IMAGE --out COPY  also write IMAGE to COPY once it passes

Checks: the magic occurs exactly once, the block fits in the image, and it
parses (hardware version 1 or 2, pad 0, every name NUL-terminated, at least
one slot set).
"""

import argparse
import struct
import sys

MAGIC = b"CfgAnchor\0"
SLOTS = 8
NAME_FIELD = 64
HEADER = 12
SLOT_SIZE = 2 + NAME_FIELD
SIZE = HEADER + SLOTS * SLOT_SIZE          # 540


def pack(hw_version, slots):
    """slots: 8 (focal_mm, name) pairs, focal 0 / name '' when unset."""
    b = bytearray(MAGIC) + bytes((hw_version, 0))
    for mm, name in slots:
        raw = name.encode("ascii")
        assert len(raw) < NAME_FIELD
        b += struct.pack("<H", mm) + raw + b"\0" * (NAME_FIELD - len(raw))
    assert len(b) == SIZE
    return bytes(b)


def find(image):
    at = image.find(MAGIC)
    if at < 0:
        sys.exit("no %r in the image" % MAGIC)
    if image.find(MAGIC, at + 1) >= 0:
        sys.exit("%r occurs more than once in the image" % MAGIC)
    if at + SIZE > len(image):
        sys.exit("the block at 0x%x runs past the end of the image" % at)
    return at


def parse(block):
    hw, pad = block[10], block[11]
    if hw not in (1, 2) or pad != 0:
        sys.exit("bad header: hw_version %d, pad %d" % (hw, pad))
    slots = []
    for i in range(SLOTS):
        o = HEADER + i * SLOT_SIZE
        mm, = struct.unpack_from("<H", block, o)
        name = block[o + 2:o + 2 + NAME_FIELD]
        if name[-1] != 0:
            sys.exit("slot %d: name not NUL-terminated" % i)
        slots.append((mm, name.split(b"\0")[0].decode("ascii")))
    if not any(mm for mm, _ in slots):
        sys.exit("no slot is set")
    return hw, slots


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image")
    ap.add_argument("--out", help="write the image here once it passes")
    args = ap.parse_args()

    image = open(args.image, "rb").read()
    at = find(image)
    hw, slots = parse(image[at:at + SIZE])
    if pack(hw, slots) != image[at:at + SIZE]:
        sys.exit("the block does not round-trip (bytes after a name's NUL?)")
    print("lens configuration block at 0x%05x, hardware version %d" % (at, hw))
    for i, (mm, name) in enumerate(slots):
        print("  slot %d  %4d mm  %s" % (i, mm, name))
    if args.out:
        with open(args.out, "wb") as f:
            f.write(image)
        print("wrote %s" % args.out)


if __name__ == "__main__":
    main()
