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
one slot set), and every slot's optical file is exactly what gen_optical.py
builds for its focal length and exit pupil (the exit pupil is read back from
type 0), all zero for an unset slot.
"""

import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import gen_optical  # noqa: E402

MAGIC = b"CfgAnchor\0"
SLOTS = 8
NAME_FIELD = 64
HEADER = 12
OPTICAL = 296
SLOT_SIZE = 2 + NAME_FIELD + OPTICAL       # 362
SIZE = HEADER + SLOTS * SLOT_SIZE          # 2908


def pack(hw_version, slots):
    """slots: 8 (focal_mm, name, xpd_mm) triples, (0, '', 0) when unset."""
    b = bytearray(MAGIC) + bytes((hw_version, 0))
    for mm, name, xpd in slots:
        raw = name.encode("ascii")
        assert len(raw) < NAME_FIELD
        b += struct.pack("<H", mm) + raw + b"\0" * (NAME_FIELD - len(raw))
        b += gen_optical.build(mm, xpd) if mm else bytes(OPTICAL)
    assert len(b) == SIZE
    return bytes(b)


def xpd_of(optical):
    """The exit pupil an optical file was built with: type 0 at infinity / 15."""
    # LF header 16, ED chunk header 8, ED header 8, block header 10, closest row 12
    t0_inf = struct.unpack_from("<H", optical, 16 + 8 + 8 + 10 + 12)[0]
    return round(t0_inf / 15)


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
        opt = block[o + 2 + NAME_FIELD:o + SLOT_SIZE]
        if not mm:
            if any(opt):
                sys.exit("slot %d: not set but its optical file is not zero" % i)
            xpd = 0
        else:
            if opt[:2] != b"LF":
                sys.exit("slot %d: optical file does not start LF" % i)
            xpd = xpd_of(opt)
            if xpd not in gen_optical.XPD_CHOICES:
                sys.exit("slot %d: exit pupil %d mm is not one of %s" % (i, xpd, gen_optical.XPD_CHOICES))
            if gen_optical.build(mm, xpd) != opt:
                sys.exit("slot %d: optical file differs from gen_optical.build(%d, %d)" % (i, mm, xpd))
        slots.append((mm, name.split(b"\0")[0].decode("ascii"), xpd))
    if not any(mm for mm, _, _ in slots):
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
    for i, (mm, name, xpd) in enumerate(slots):
        print("  slot %d  %4d mm  exit pupil %2d mm  %s" % (i, mm, xpd, name))
    if args.out:
        with open(args.out, "wb") as f:
            f.write(image)
        print("wrote %s" % args.out)


if __name__ == "__main__":
    main()
