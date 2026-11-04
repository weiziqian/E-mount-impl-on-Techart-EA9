#!/usr/bin/env python3
"""
locate.py -- find where the focal length and the lens name sit in the release
image, by building it three times and comparing.

  A  the repo as it is                        -> the image that ships
  B  a temporary copy, focal 137 mm, name 'Z' x 63
  C  a temporary copy, focal  85 mm, a shorter name

Every byte where A and B differ must be one of three encodings of the focal
length or the name field; anything else (code that changed, a shifted layout)
is an error.  Then A is patched to B's and C's values with the table just
found, and each result must equal the real build BYTE FOR BYTE -- so a missed
location, or one the compiler folded into code, fails here and not on a
camera.

The source is never edited: B and C are built in a scratch copy of src/ and
the Makefile.  Release configuration only.

  tools/win_updater/locate.py -o patchmap.json --bin ea9.bin

writes the patch table (and prints it), and with --bin a copy of image A --
the pair the updater embeds.  The updater applies it with
apply_patch() below; spec.md section 5 is the contract.
"""

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
IMAGE = "build/ea9-rebuild-release.bin"

NAME_LEN = 63                       # 0x3F pl[1..63]; pl[64] stays NUL
DEFAULT = (50, "TECHART LM-EA9")
PROBE_B = (137, "Z" * NAME_LEN)     # every byte differs from DEFAULT's
PROBE_C = (85, "Summicron-M 35mm f/2 ASPH")

# What the current source holds: em_focal_mm10 as u16 (the messages copy it at
# start-up, so it is the only copy) and the 0x3F name.  A different count
# means the source changed -- look at why before trusting the table.
EXPECT = {"u16le_mm10": 1, "bcd_mm": 0, "name": 1}


def bcd(n):
    return ((n // 10 % 10) << 4) | (n % 10)


def encode(kind, focal_mm, name):
    if kind == "u16le_mm10":
        v = focal_mm * 10
        return bytes((v & 0xFF, v >> 8))
    if kind == "bcd_mm":
        return bytes((bcd(focal_mm // 100), bcd(focal_mm)))
    if kind == "name":
        raw = name.encode("ascii")
        if len(raw) > NAME_LEN:
            raise ValueError("name longer than %d characters" % NAME_LEN)
        return raw + b"\0" * (NAME_LEN - len(raw))
    raise ValueError(kind)


def apply_patch(image, fields, focal_mm, name):
    out = bytearray(image)
    for f in fields:
        b = encode(f["kind"], focal_mm, name)
        out[f["offset"]:f["offset"] + len(b)] = b
    return bytes(out)


def make_release(tree):
    r = subprocess.run(["make", "release"], cwd=tree,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       universal_newlines=True)
    if r.returncode != 0:
        sys.exit("make release failed in %s:\n%s" % (tree, r.stdout[-3000:]))
    with open(os.path.join(tree, IMAGE), "rb") as f:
        return f.read()


def set_values(tree, focal_mm, name):
    """Edit the scratch copy: the focal #define and the 0x3F name bytes."""
    p = os.path.join(tree, "src", "emount.h")
    s = open(p).read()
    s, n = re.subn(r"#define EM_FOCAL_MM10 \d+u",
                   "#define EM_FOCAL_MM10 %du" % (focal_mm * 10), s)
    if n != 1:
        sys.exit("EM_FOCAL_MM10 not found exactly once in src/emount.h")
    open(p, "w").write(s)

    p = os.path.join(tree, "src", "emount_packets.c")
    s = open(p).read()
    m = re.search(r"(uint8_t em_msg_3F\[EA9_LEN_3F\] = \{)(.*?)(\};)", s, re.S)
    if not m:
        sys.exit("em_msg_3F not found in src/emount_packets.c")
    body = re.sub(r"/\*.*?\*/", "", m.group(2), flags=re.S)
    frame = [int(t, 16) for t in re.findall(r"0x[0-9a-fA-F]{2}", body)]
    frame[7:7 + NAME_LEN] = encode("name", 0, name)
    new = "\n\t" + ", ".join("0x%02x" % b for b in frame) + ",\n"
    s = s[:m.start(2)] + new + s[m.end(2):]
    open(p, "w").write(s)


def build_variant(focal_mm, name):
    tmp = tempfile.mkdtemp(prefix="ea9-locate-")
    try:
        shutil.copytree(os.path.join(REPO, "src"), os.path.join(tmp, "src"))
        shutil.copy(os.path.join(REPO, "Makefile"), tmp)
        set_values(tmp, focal_mm, name)
        return make_release(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def find_fields(a, b):
    if len(a) != len(b):
        sys.exit("A and B differ in length (%d vs %d): the values changed "
                 "the code layout, so the image cannot be patched" % (len(a), len(b)))
    diff = [i for i in range(len(a)) if a[i] != b[i]]
    runs = []
    for i in diff:
        if runs and i == runs[-1][1]:
            runs[-1][1] = i + 1
        else:
            runs.append([i, i + 1])

    want = {k: encode(k, *DEFAULT) for k in ("u16le_mm10", "bcd_mm")}
    probe = {k: encode(k, *PROBE_B) for k in ("u16le_mm10", "bcd_mm")}
    fields = []
    for lo, hi in runs:
        if hi - lo == NAME_LEN and b[lo:hi] == encode("name", *PROBE_B) \
                and a[lo:hi] == encode("name", *DEFAULT):
            fields.append({"offset": lo, "kind": "name", "len": NAME_LEN})
            continue
        if (hi - lo) % 2:
            sys.exit("unexplained difference at 0x%x..0x%x" % (lo, hi))
        for o in range(lo, hi, 2):
            for kind in ("u16le_mm10", "bcd_mm"):
                if a[o:o + 2] == want[kind] and b[o:o + 2] == probe[kind]:
                    fields.append({"offset": o, "kind": kind, "len": 2})
                    break
            else:
                sys.exit("unexplained difference at 0x%x: %s -> %s"
                         % (o, a[o:o + 2].hex(" "), b[o:o + 2].hex(" ")))
    return fields


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--out", required=True, help="patch table (JSON)")
    ap.add_argument("--bin", help="also write image A here")
    args = ap.parse_args()

    print("building A (repo) ...")
    a = make_release(REPO)
    print("building B (%d mm, %r) ..." % PROBE_B)
    b = build_variant(*PROBE_B)
    print("building C (%d mm, %r) ..." % PROBE_C)
    c = build_variant(*PROBE_C)

    fields = find_fields(a, b)
    count = {k: sum(f["kind"] == k for f in fields) for k in EXPECT}
    if count != EXPECT:
        sys.exit("found %s, expected %s -- the source changed; check why"
                 % (count, EXPECT))
    for f in fields:
        if a[f["offset"]:f["offset"] + f["len"]] != encode(f["kind"], *DEFAULT):
            sys.exit("A does not hold the default at 0x%x" % f["offset"])

    if apply_patch(a, fields, *PROBE_B) != b:
        sys.exit("patched A != B")
    if apply_patch(a, fields, *PROBE_C) != c:
        sys.exit("patched A != C: a location is missing or encoded differently")
    print("verified: A patched to B's and C's values equals the real builds")

    table = {
        "image": IMAGE,
        "image_len": len(a),
        "image_sha256": hashlib.sha256(a).hexdigest(),
        "default": {"focal_mm": DEFAULT[0], "name": DEFAULT[1]},
        "fields": sorted(fields, key=lambda f: f["offset"]),
    }
    with open(args.out, "w") as f:
        json.dump(table, f, indent=2)
        f.write("\n")
    for f in table["fields"]:
        print("  0x%05x  %-11s %s" % (f["offset"], f["kind"],
                                       a[f["offset"]:f["offset"] + f["len"]][:16].hex(" ")))
    print("wrote %s" % args.out)
    if args.bin:
        with open(args.bin, "wb") as f:
            f.write(a)
        print("wrote %s" % args.bin)


if __name__ == "__main__":
    main()
