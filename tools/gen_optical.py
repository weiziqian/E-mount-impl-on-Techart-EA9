#!/usr/bin/env python3
"""Generate a message 0x3B optical file (LF) for a given focal length and exit pupil.

Same layout and size (296 bytes) as optical_file[] in src/optical_file.c, which
is the Yongnuo 50 DF file trimmed to one aperture (tools/mk_optical.py): one
zoom point, 8 focus points, aperture 0 plus the end marker, two focus rows
(index 0 = closest, index 7 = infinity) per block.  Only the values change:

  focus axis   index 7 = 0x700 (infinity); indices 0..6 at extensions
               4.5 mm * (7-i)/7, so index 0 is the closest focus at full
               extension.  Distance u = f*(f+x)/x and the code
               384 + 64*log2(u m) are computed exactly as src/focus_dist.c
               does, so the axis lines up with what the adapter reports.
               Extension steps are equal, so the body's index-linear fill of
               indices 1..6 reproduces type 0 exactly.
  type 0       all six columns: infinity row = XPD * 15, closest row =
               (XPD + 4.5) * 15 (XPD in mm; i.e. 0.1 mm units x 1.5).
  type 1       column 0 = f/1.8 (= LC[3]); column 5 = f/1.8 * (6 - 0.06*f),
               with f clamped to 30..70 mm for this formula only; columns 1..4 follow the shape of
               the Yongnuo 50 DF infinity row, in counts (256 per stop).
               No bellows factor: closest row = infinity row.
  type 2       same, scaled so its rise keeps the Yongnuo ratio to type 1.
  end-marker   aperture 2645: type 0 as above, type 1/2 the Yongnuo values.
  LC           [1, 848, 1008, LC3, 3195, 0]: LC[3] = f/1.8, the rest Yongnuo
               (meaning UNKNOWN, msg_0x3B.md 1.5).
  check word   XOR of every u32 LE word before it.

    gen_optical.py --focal 50 --xpd 48.3              table + C array
    gen_optical.py --focal 50                         XPD chosen by auto_xpd()
    gen_optical.py --focal 35 --xpd 34 --bin out.bin
    gen_optical.py --focal 50 --xpd 48.3 --name optical_file_50 > f50.inc
    gen_optical.py --focal 50 --xpd 48.3 --write src/optical_file.c
    gen_optical.py --defaults src/lens_cfg_optical.h  lens_cfg.c's slot defaults
    gen_optical.py --selftest

The Windows updater builds the same bytes (tools/win_updater/src/OpticalFile.cs);
cfgblock.py checks every slot of the release image against build() here.
"""
import argparse
import math
import re
import struct
import sys

TRAVEL_UM = 4500                 # src/focus_dist.h FOCUS_TRAVEL_UM
NF = 8
INF = 0x700
AP_END = 2645
LC3 = round(512 * math.log2(1.8))            # 434 = f/1.8
LC_REST = (1, 848, 1008, None, 3195, 0)
MODEL = 0x9140
VERSION = 0x0b

# Yongnuo 50 DF, aperture 0, infinity row (what the adapter has been sending).
YN_T1 = (382, 440, 528, 645, 803, 1008)
YN_T2 = (382, 384, 397, 426, 473, 541)
YN_END_T1 = ((2646,) * 6, (2572,) * 6)       # (closest, infinity) rows
YN_END_T2 = YN_END_T1
SHAPE1 = [(v - YN_T1[0]) / (YN_T1[5] - YN_T1[0]) for v in YN_T1]
SHAPE2 = [(v - YN_T2[0]) / (YN_T1[5] - YN_T1[0]) for v in YN_T2]


def log2_q6(x):
    """src/focus_dist.c focus_log2_q6, bit for bit."""
    if x == 0:
        return 0
    e = x.bit_length() - 1
    m = x << (14 - e) if e <= 14 else x >> (e - 14)
    frac = 0
    for _ in range(7):
        m = (m * m) >> 14
        frac <<= 1
        if m >= (2 << 14):
            m >>= 1
            frac |= 1
    return ((e << 7) + frac + 1) >> 1


def dist_code(x_um, focal_mm10):
    """src/focus_dist.c focus_distance_code, from an extension in um."""
    if x_um == 0:
        return INF
    f_mm, f_um = focal_mm10 // 10, focal_mm10 * 100
    d_mm = f_mm * (f_um + x_um) // x_um
    code = 384 + log2_q6(d_mm) - 638
    return min(max(code, 0), INF)


def focus_axis(focal_mm10):
    xs = [(TRAVEL_UM * (NF - 1 - i) + (NF - 1) // 2) // (NF - 1) for i in range(NF)]
    return [dist_code(x, focal_mm10) for x in xs], xs


VIG_MIN, VIG_MAX = 30, 70       # the edge formula's focal range; clamped outside it
XPD_CHOICES = (25, 30, 35, 40, 45, 50, 55, 60, 70)   # the updater's drop-down, mm


def auto_xpd(focal):
    """The exit pupil the updater preselects for a focal length."""
    for limit, xpd in ((30, 30), (35, 35), (40, 40), (50, 45), (60, 50)):
        if focal <= limit:
            return xpd
    return 60


def rows(focal, xpd):
    t0_inf = [round(xpd * 15)] * 6
    t0_near = [round((xpd + TRAVEL_UM / 1000) * 15)] * 6
    k = 6 - 0.06 * min(max(focal, VIG_MIN), VIG_MAX)
    rise = 512 * math.log2(k)
    t1 = [round(LC3 + rise * s) for s in SHAPE1]
    t2 = [round(LC3 + rise * s) for s in SHAPE2]
    return t0_near, t0_inf, t1, t2, k


def build(focal, xpd):
    fm10 = round(focal * 10)
    axis, _ = focus_axis(fm10)
    t0n, t0i, t1, t2, _ = rows(focal, xpd)

    def block(near, inf):
        return struct.pack('<5H', 1, 2, 0, 0, NF - 1) + struct.pack('<12H', *near, *inf)

    ed = struct.pack('<4H', 6, 1, NF, 2)
    ed += block(t0n, t0i) + block(t1, t1) + block(t2, t2)
    ed += block(t0n, t0i) + block(*YN_END_T1) + block(*YN_END_T2)
    cs = struct.pack('<4H', 1, NF, 2, 0) + struct.pack('<%dH' % NF, *axis) + struct.pack('<2H', 0, AP_END)
    lc = struct.pack('<6H', *(LC3 if v is None else v for v in LC_REST))
    body = b''.join(t + struct.pack('<HI', 8, len(d)) + d
                    for t, d in ((b'ED', ed), (b'CS', cs), (b'LC', lc)))
    f = b'LF' + struct.pack('<HIII', 16, len(body) + 4, VERSION, MODEL) + body
    return f + struct.pack('<I', check_word(f))


def check_word(b):
    x = 0
    for w in struct.unpack('<%dI' % (len(b) // 4), b):
        x ^= w
    return x


def c_array(f):
    return '\n'.join('\t/* +0x%03x */ %s,' % (i, ', '.join('0x%02x' % b for b in f[i:i + 12]))
                     for i in range(0, len(f), 12))


def show(focal, xpd):
    fm10 = round(focal * 10)
    axis, xs = focus_axis(fm10)
    t0n, t0i, t1, t2, k = rows(focal, xpd)
    print('focal %.1f mm, XPD %.1f mm (closest %.1f mm)' % (focal, xpd, xpd + TRAVEL_UM / 1000))
    print('focus axis (index: extension -> code = distance):')
    for i, (x, c) in enumerate(zip(xs, axis)):
        d = 'inf' if c >= INF else '%.3f m' % 2 ** ((c - 384) / 64)
        print('  %d: %5.3f mm -> %4d = %s' % (i, x / 1000, c, d))
    print('type 0  closest %s\n        inf     %s' % (t0n, t0i))
    print('type 1  %s   f/%.2f -> f/%.2f (x%.2f)' % (t1, 2 ** (t1[0] / 512), 2 ** (t1[5] / 512), k))
    print('type 2  %s' % t2)
    print('LC[3] %d = f/%.3f' % (LC3, 2 ** (LC3 / 512)))


# The focal lengths of lens_cfg.c's default slots.  Keep in step with it.
DEFAULT_FOCALS = (25, 28, 32, 35, 40, 45, 50, 56)


def write_defaults(path):
    out = ['/* lens_cfg_optical.h -- the optical file of each default slot in lens_cfg.c.',
           ' * GENERATED by tools/gen_optical.py --defaults; do not edit.  Each is',
           ' * build(focal, auto_xpd(focal)): what the updater writes for that focal',
           ' * length with the exit pupil it preselects. */',
           '#ifndef LENS_CFG_OPTICAL_H', '#define LENS_CFG_OPTICAL_H', '']
    for fl in DEFAULT_FOCALS:
        x = auto_xpd(fl)
        f = build(fl, x)
        out.append('/* %d mm, exit pupil %d mm */' % (fl, x))
        out.append('#define LENS_CFG_OPTICAL_%dMM { \\' % fl)
        for i in range(0, len(f), 12):
            out.append('\t%s, \\' % ', '.join('0x%02x' % b for b in f[i:i + 12]))
        out.append('}')
        out.append('')
    out.append('#endif /* LENS_CFG_OPTICAL_H */')
    open(path, 'w').write('\n'.join(out) + '\n')


def current_file():
    import os
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'optical_file.c')
    s = open(p).read().split('optical_file[OPTICAL_FILE_LEN] = {')[1].split('};')[0]
    s = re.sub(r'/\*.*?\*/', '', s)
    return bytes(int(x, 16) for x in re.findall(r'0x[0-9a-fA-F]{2}', s))


def selftest():
    import os
    old = current_file()
    new = build(50, 48.3)
    assert len(new) == len(old) == 296, (len(new), len(old))
    # Layout: every byte that is not a value (axis, row, LC[3], check word) must match.
    vals = set()
    def mark(o, n):
        vals.update(range(o, o + 2 * n))
    p = 16 + 8 + 8
    for _ in range(6):
        mark(p + 10, 12); p += 34
    p += 8 + 8; mark(p, NF); p += 2 * NF + 4
    p += 8; mark(p + 6, 1)
    vals.update(range(len(old) - 4, len(old)))
    bad = [i for i in range(len(old)) if i not in vals and old[i] != new[i]]
    assert not bad, 'layout differs at %s' % bad
    assert check_word(new[:-4]) == struct.unpack_from('<I', new, len(new) - 4)[0]
    assert check_word(old[:-4]) == struct.unpack_from('<I', old, len(old) - 4)[0]
    # The distance code must equal the C implementation's for spot values.
    assert dist_code(4500, 500) == 384 + log2_q6(50 * 54500 // 4500) - 638
    print('selftest OK: 296 bytes, layout identical to src/optical_file.c, check word valid')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--focal', type=float, help='focal length, mm')
    ap.add_argument('--xpd', type=float, help='exit pupil distance from the sensor at infinity, mm '
                    '(default: auto_xpd)')
    ap.add_argument('--defaults', metavar='HEADER', help='write the slot defaults for src/lens_cfg.c')
    ap.add_argument('--bin', help='write the raw file here')
    ap.add_argument('--name', help='print a complete C array definition with this name')
    ap.add_argument('--write', metavar='OPTICAL_FILE_C', help='replace optical_file[] in this source')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest:
        selftest()
        return
    if a.defaults:
        write_defaults(a.defaults)
        return
    if a.focal is None:
        ap.error('--focal is required')
    if a.xpd is None:
        a.xpd = auto_xpd(a.focal)
    f = build(a.focal, a.xpd)
    if a.bin:
        open(a.bin, 'wb').write(f)
    if a.write:
        src = open(a.write).read()
        src, n = re.subn(r'(const uint8_t optical_file\[OPTICAL_FILE_LEN\] = \{\n).*?(\n\};)',
                         lambda m: m.group(1) + c_array(f) + m.group(2), src, count=1, flags=re.S)
        assert n == 1
        open(a.write, 'w').write(src)
    if a.name:
        print('/* gen_optical.py --focal %g --xpd %g */' % (a.focal, a.xpd))
        print('const uint8_t %s[%d] = {\n%s\n};' % (a.name, len(f), c_array(f)))
    elif not (a.bin or a.write):
        show(a.focal, a.xpd)
        print('%d bytes, check word %08x' % (len(f), check_word(f[:-4])))
        print(c_array(f))
    else:
        show(a.focal, a.xpd)


if __name__ == '__main__':
    main()
