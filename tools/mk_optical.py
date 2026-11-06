#!/usr/bin/env python3
"""Rebuild optical_file[] in src/optical_file.c from the Yongnuo 50 DF image.

The adapter declares one aperture, f/1.8, so only the Yongnuo file's first
aperture (0, wide open) and its closing end-marker entry are kept; the 22 in
between are dropped.  Every value is the DF's own.  The check word is then
recomputed: the XOR of every u32 LE word before it (msg_0x3B.md).

    mk_optical.py <50F1.8S_DF_v04.sec2_cortexm.bin> [src/optical_file.c]
"""
import re
import struct
import sys

DF_OFFSET = 0x10d8c          # 0x0801ad8c in the DF image, load base 0x0800a000
DF_LEN = 2584


def chunks(f):
    out, p, end = {}, 16, 16 + struct.unpack_from('<I', f, 4)[0] - 4
    while p < end:
        tag, hs, n = struct.unpack_from('<2sHI', f, p)
        out[tag] = f[p + hs:p + hs + n]
        p += hs + n
    return out


def ed_blocks(ed):
    nh, _, _, na = struct.unpack_from('<4H', ed, 0)
    p, blocks = 8, []
    for _ in range(3 * na):
        r = struct.unpack_from('<H', ed, p + 2)[0]
        n = 8 + 2 * (r - 1) + 2 * nh * r
        blocks.append(ed[p:p + n])
        p += n
    assert p == len(ed)
    return blocks


def check_word(b):
    x = 0
    for w in struct.unpack('<%dI' % (len(b) // 4), b):
        x ^= w
    return x


def trim(df):
    c = chunks(df)
    nh, zm, nf, na = struct.unpack_from('<4H', c[b'ED'], 0)
    blocks = ed_blocks(c[b'ED'])
    keep = (0, na - 1)
    ed = struct.pack('<4H', nh, zm, nf, len(keep))
    ed += b''.join(blocks[3 * a + t] for a in keep for t in range(3))
    cs = c[b'CS']
    one, cnf, cna, z = struct.unpack_from('<4H', cs, 0)
    assert (cnf, cna) == (nf, na)
    ap = struct.unpack_from('<%dH' % na, cs, 8 + 2 * nf)
    cs = (struct.pack('<4H', one, nf, len(keep), z) + cs[8:8 + 2 * nf]
          + struct.pack('<%dH' % len(keep), *(ap[a] for a in keep)))
    body = b''.join(t + struct.pack('<HI', 8, len(d)) + d
                    for t, d in ((b'ED', ed), (b'CS', cs), (b'LC', c[b'LC'])))
    f = df[0:4] + struct.pack('<I', len(body) + 4) + df[8:16] + body
    return f + struct.pack('<I', check_word(f))


def c_array(f):
    rows = []
    for i in range(0, len(f), 12):
        rows.append('\t/* +0x%03x */ %s,' % (
            i, ', '.join('0x%02x' % b for b in f[i:i + 12])))
    return '\n'.join(rows)


def main():
    df = open(sys.argv[1], 'rb').read()[DF_OFFSET:DF_OFFSET + DF_LEN]
    assert df[:4] == b'LF\x10\x00'
    f = trim(df)
    print('%d bytes, check word %08x' % (len(f), check_word(f[:-4])))
    if len(sys.argv) > 2:
        path = sys.argv[2]
        src = open(path).read()
        src, n = re.subn(
            r'(const uint8_t optical_file\[OPTICAL_FILE_LEN\] = \{\n).*?(\n\};)',
            lambda m: m.group(1) + c_array(f) + m.group(2), src, count=1,
            flags=re.S)
        assert n == 1
        open(path, 'w').write(src)
    else:
        print(c_array(f))


if __name__ == '__main__':
    main()
