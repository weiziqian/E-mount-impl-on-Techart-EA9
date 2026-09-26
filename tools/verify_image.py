#!/usr/bin/env python3
"""verify_image.py -- static checks on a rebuilt LM-EA9 app image.

Everything here is checkable without hardware.  The point is to catch a bad
link (wrong base, wrong stack, vectors outside the image) before anything is
flashed, and to show the rebuilt image's shape next to the stock one.

  verify_image.py <image.bin> [stock.bin]
"""
import struct
import sys

BASE = 0x5000
FLASH_END = 0x20000          # SAMD21E17A, 128 KB (EA9.md 11.7)
RAM_LO, RAM_HI = 0x20000000, 0x20004000   # 16 KB

EXC = {0: "SP", 1: "Reset", 2: "NMI", 3: "HardFault", 11: "SVC", 14: "PendSV",
       15: "SysTick"}
IRQ = {0: "EIC(4)", 5: "NVMCTRL", 6: "DMAC", 10: "SERCOM1", 15: "TCC0",
       16: "TCC1", 19: "TC4"}


NVEC = 44        # 16 Cortex-M0+ exceptions + 28 SAM D21 IRQ lines; code starts
                 # at 0x50b4 in the stock image, so anything past [43] is code.


def vectors(img):
    n = min(len(img) // 4, NVEC)
    return list(struct.unpack_from("<%dI" % n, img, 0))


def check(img, name):
    ok = True

    def bad(msg):
        nonlocal ok
        ok = False
        print(f"  FAIL  {msg}")

    print(f"{name}: {len(img)} bytes, loads at 0x{BASE:04x}-0x{BASE+len(img):05x}")
    if BASE + len(img) > FLASH_END:
        bad(f"image overruns flash end 0x{FLASH_END:05x}")

    v = vectors(img)
    sp, reset = v[0], v[1]
    print(f"  initial SP     0x{sp:08x}")
    if not (RAM_LO < sp <= RAM_HI):
        bad(f"SP outside SRAM 0x{RAM_LO:08x}-0x{RAM_HI:08x}")
    if sp % 8:
        bad("SP not 8-byte aligned")

    print(f"  reset vector   0x{reset:08x}")
    if not reset & 1:
        bad("reset vector has no Thumb bit -- the CPU will fault immediately")
    if not (BASE <= (reset & ~1) < BASE + len(img)):
        bad("reset vector points outside the image")

    stray = [(i, x) for i, x in enumerate(v)
             if x and i != 0 and not (BASE <= (x & ~1) < BASE + len(img))]
    if stray:
        bad("vector entries pointing outside the image: "
            + ", ".join(f"[{i}]=0x{x:08x}" for i, x in stray))
    else:
        print(f"  all {sum(1 for x in v[1:] if x)} populated vectors land inside the image")

    dflt = v[3]
    live = {i: x for i, x in enumerate(v) if i > 1 and x and x != dflt}
    print(f"  default handler 0x{dflt:08x}; {len(live)} vectors differ from it:")
    for i, x in sorted(live.items()):
        who = EXC.get(i) or (f"IRQ{i-16} {IRQ.get(i-16,'')}".strip() if i >= 16 else "")
        print(f"    [{i:2d}] 0x{x:08x}  {who}")
    return ok


def main():
    img = open(sys.argv[1], "rb").read()
    ok = check(img, sys.argv[1])
    if len(sys.argv) > 2:
        print()
        stock = open(sys.argv[2], "rb").read()
        check(stock, sys.argv[2])
        print("\ncomparison")
        a, b = vectors(img), vectors(stock)
        print(f"  stock SP 0x{b[0]:08x}   rebuild SP 0x{a[0]:08x}"
              f"   {'(same)' if a[0]==b[0] else '(differ -- expected until .bss matches)'}")
    print("\nRESULT:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
