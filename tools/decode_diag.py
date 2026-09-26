#!/usr/bin/env python3
"""decode_diag.py -- read the flash log left by the diagnostic build.

  ea9flash.py --dump 0x1f000:0x400 -o dumps/diag.bin
  decode_diag.py dumps/diag.bin

16 pages, all erased at boot and written one at a time as the run progresses,
so THE LAST PAGE PRESENT IS WHERE IT STOPPED.  Every page carries a format word;
pages from an older build are ignored rather than decoded as garbage -- an
earlier version of this script decoded stale rows and reported movement that
never happened.
"""
import struct
import sys

MAGIC, FORMAT = 0x39414544, 4
PAGE = 64
blob = open(sys.argv[1] if len(sys.argv) > 1 else "dumps/diag.bin", "rb").read()

RCAUSE = [(0, "POR (power-on)"), (1, "BOD12"), (2, "BOD33 (brown-out)"),
          (4, "EXT"), (5, "WDT"), (6, "SYST")]
NAME = {0x1: "WO0 alone", 0x2: "WO1 alone", 0x4: "WO2 alone", 0x8: "WO3 alone",
        0x9: "WO0+WO3 diagonal", 0x6: "WO1+WO2 diagonal",
        0x5: "WO0+WO2 (stock +)", 0xA: "WO1+WO3 (stock -)"}


def page(p):
    off = p * PAGE
    if off + PAGE > len(blob):
        return None
    w = list(struct.unpack_from("<16I", blob, off))
    return w if w[0] == MAGIC and w[1] == FORMAT else None


pages = [p for p in range(len(blob) // PAGE) if page(p)]
print(f"pages present: {pages or 'NONE'}")
if not pages:
    print("\n  Nothing from this build.  Either it never ran, or the dump still")
    print("  holds an older format (the format word guards against decoding it).")
    sys.exit(0)

w0 = page(0)
rc = w0[3]
why = ", ".join(n for b, n in RCAUSE if rc >> b & 1) or f"0x{rc:02x}"
print(f"\nboot #{w0[4]}   PM->RCAUSE = 0x{rc:02x} -> {why}")
print(f"PORT->IN at power-up = 0x{w0[2]:08x}")

band = None
if page(1):
    w = page(1)
    idle = [w[4 + i] for i in range(8)]
    band = max(idle) - min(idle)
    print(f"TCC0 CTRLA = 0x{w[2]:08x}   PER = {w[3]}   pulse duty = {w[12]}")
    print(f"idle angles {idle}   noise band {band}")

thr = (band if band is not None else 4) + 4
print(f"\nmovement threshold: > {thr} counts\n")

# round 7: a page every 200 ms, motor untouched
if page(2) is not None and page(2)[2] == 2:
    print("  tick   millis   angle   PORT->IN")
    last_t = 0
    for p in range(1, 16):
        w = page(p)
        if w is None:
            break
        last_t = w[3]
        print(f"   {w[2]:2d}   {w[3]:6d}   {w[4]:5d}   0x{w[5]:08x}")
    print()
    if page(15) is not None:
        print("  survived the whole 3 s with NO motor activity.")
        print("  => the body is not cutting power on a timer; the drive is implicated.")
    else:
        print(f"  DIED at about {last_t} ms, with the motor never driven at all.")
        print("  => the body cuts power on a schedule.  Every 'which pattern killed")
        print("     it' result from rounds 4-6 was an artefact of that timer.")
    raise SystemExit(0)

done = page(10)
print()
if done:
    print(f"  ran to completion; final angle {done[3]}")
elif last is None:
    print("  died before the first pattern")
else:
    print(f"  STOPPED after page {last} -- page {last+1} is what killed it")
