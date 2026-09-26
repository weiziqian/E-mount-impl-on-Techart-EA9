#!/usr/bin/env python3
"""gen_packets.py -- lift the stored E-mount reply packets out of a stock image.

The LM-EA9 keeps all 23 of its outgoing frames as a pre-built table in `.data`
(CLAUDE.md, "The E-mount protocol tables").  The rebuild answers the body from
the same bytes, so this emits that whole `.data` initialiser plus an index of
{message id -> offset, length}.

Why the whole block and not just the frames: every address in EA9.md is quoted
as `.data+0xNNN`, and three of the init handlers write into a packet before
sending it (ids 0x1b, 0x34) or into a neighbouring slot.  Keeping the layout
intact keeps every one of those notes directly usable.

Lengths come from the handlers themselves, not from the stored length byte --
six frames are stored as runtime templates with a length byte of 0.  See
`decode_handlers()`.

Usage: gen_packets.py fw/EA9-VER-1-8-0.bin src/emount_packets.c src/emount_packets.h
"""
import struct
import sys

BASE = 0x5000            # app load address; file_offset = addr - BASE
SIDATA = 0x99D4          # .data initialiser in flash (reset handler literal pool)
DATA_LEN = 0x4F8         # _edata - _sdata
JUMP_TABLE = 0x95F8      # 91 entries, ids 0x00..0x5a (EA9.md 2.5)
IGNORE = 0x58BA          # the "no handler" exit
TRANSMIT = 0x51D5        # transmit(buf, len)

# Packet pointer slots that protocol_init (0x551c) fills in, so the handlers
# that reach their frame through RAM can still be resolved.  Slot address ->
# the .data address the slot is loaded with.
PTR_TARGET = {
    0x2000064C: 0x038,   # msg 0x01
    0x20000650: 0x064,   # msg 0x07
    0x20000670: 0x090,   # msg 0x0c
    0x2000068C: 0x09C,   # msg 0x0b
    0x20000648: 0x0A8,   # msg 0x0d
    0x2000065C: 0x0B4,   # msg 0x08
    0x20000664: 0x188,   # msg 0x09
    0x20000658: 0x19C,   # msg 0x10
}
PTR_TARGET = {slot: 0x20000000 + off for slot, off in PTR_TARGET.items()}

# The two status frames have no init handler -- they are sent unsolicited by the
# senders at 0x5238 / 0x54d8, whose lengths come from 0x20000654 / 0x20000655.
STATUS = {0x05: (0x1A8, 105), 0x06: (0x000, 48)}


RX_BUF = 0x20000694          # the inbound DMA buffer; buffer[0] is the 0xF0


def decode_handlers(img):
    """Interpret every live handler in the dispatch table.

    Returns {id: (data_offset, length, [fixups])}, where a fixup is
    (frame_offset, kind, value) with kind "imm" for a constant and "rx" for a
    byte copied out of the received frame.

    WHY THIS INTERPRETS RATHER THAN PATTERN-MATCHES.  The first version of this
    script read each handler only far enough to recover the (address, length)
    pair it hands to transmit, and ignored everything else in the body.  Three
    handlers edit their frame before sending it, and dropping those edits sent
    message 0x0c with payload[0] = 1 where the stock firmware always sends 0 --
    which was enough for a real body to stop the handshake dead.  A handler is
    twelve bytes; there is no excuse for reading only half of one.

    Values are tagged rather than raw so a store can say where its byte came
    from: ("imm", n) a constant, ("addr", a) an absolute address, ("rx", n) the
    received frame's byte n.
    """
    def w(a):
        return struct.unpack("<I", img[a - BASE:a - BASE + 4])[0]

    def h(a):
        return struct.unpack("<H", img[a - BASE:a - BASE + 2])[0]

    out = {}
    for mid in range(0x5B):
        target = w(JUMP_TABLE + 4 * mid)
        if target == IGNORE:
            continue

        reg = {}
        stores = []          # (absolute address, tagged value)
        length = None
        sent = False
        a = target

        for _ in range(24):
            op = h(a)
            a += 2
            rt, rn = op & 7, (op >> 3) & 7

            if op >> 11 == 0b01001:                            # ldr rT, [pc, #imm]
                reg[(op >> 8) & 7] = ("addr", w(((a + 2) & ~3) + (op & 0xFF) * 4))

            elif op & 0xF800 == 0x6800:                        # ldr rT, [rN, #imm]
                base = reg.get(rn)
                imm = ((op >> 6) & 0x1F) * 4
                reg[rt] = (("addr", PTR_TARGET[base[1] + imm])
                           if base and base[0] == "addr"
                           and base[1] + imm in PTR_TARGET else None)

            elif op & 0xF800 == 0x7800:                        # ldrb rT, [rN, #imm]
                base = reg.get(rn)
                imm = (op >> 6) & 0x1F
                reg[rt] = (("rx", imm)
                           if base and base[0] == "addr" and base[1] == RX_BUF
                           else None)

            elif op & 0xF800 == 0x7000:                        # strb rT, [rN, #imm]
                base = reg.get(rn)
                imm = (op >> 6) & 0x1F
                if base and base[0] == "addr" and reg.get(rt):
                    stores.append((base[1] + imm, reg[rt]))

            elif op >> 11 == 0b00100:                          # movs rD, #imm
                rd = (op >> 8) & 7
                reg[rd] = ("imm", op & 0xFF)
                if rd == 1:
                    length = op & 0xFF

            elif op >> 11 == 0b00110:                          # adds rD, #imm
                rd = (op >> 8) & 7
                v = reg.get(rd)
                if v and v[0] in ("addr", "imm"):
                    reg[rd] = (v[0], v[1] + (op & 0xFF))

            elif op >> 11 == 0b00111:                          # subs rD, #imm
                rd = (op >> 8) & 7
                v = reg.get(rd)
                if v and v[0] in ("addr", "imm"):
                    reg[rd] = (v[0], v[1] - (op & 0xFF))

            elif op & 0xFF87 == 0x4600:                        # mov rD, rM (high form)
                reg[op & 7] = reg.get((op >> 3) & 0xF)

            elif op & 0xFFC0 == 0x0000:                        # lsls rD, rM, #0 = movs
                reg[rt] = reg.get(rn)

            elif (op & 0xF800 == 0xE000 or op & 0xFF00 == 0xBD00
                  or op == 0x4770):
                # b <label>, pop {..,pc}, bx lr -- the handler is over.  Without
                # this the decoder runs off the end of a handler that sends
                # nothing (id 0x16) into the next one's body, and reports that
                # handler's frame under the wrong id.
                break

            elif op == 0x4798:                                 # blx r3
                r3 = reg.get(3)
                if r3 and r3[0] == "addr" and r3[1] == TRANSMIT:
                    sent = True
                    break
                # some other call (id 0x1b runs the focal-length procedure
                # first); it clobbers r0-r3, so stop trusting them
                for k in (0, 1, 2, 3):
                    reg.pop(k, None)
                reg[1] = ("imm", length) if length is not None else None

        frame = reg.get(0)
        if not sent or not frame or frame[0] != "addr" or length is None:
            out[mid] = None
            continue

        base = frame[1]
        fixups = [(addr - base, val[0], val[1]) for addr, val in stores
                  if base <= addr < base + length]
        out[mid] = (base - 0x20000000, length, fixups)
    return out


def main():
    img = open(sys.argv[1], "rb").read()
    data = img[SIDATA - BASE:SIDATA - BASE + DATA_LEN]

    handlers = decode_handlers(img)
    table = {mid: (off, ln, []) for mid, (off, ln) in STATUS.items()}
    for mid, v in sorted(handlers.items()):
        if v is not None:
            table[mid] = v
    fixups = [(mid, off, kind, val)
              for mid, (_, _, fx) in sorted(table.items())
              for off, kind, val in fx]

    # Sanity: every frame must start 0xF0 and carry its own id at offset 5.
    for mid, (off, ln, _) in sorted(table.items()):
        assert data[off] == 0xF0, f"id {mid:#04x}: no 0xF0 at .data+{off:#05x}"
        assert data[off + 5] == mid, \
            f"id {mid:#04x}: frame says id {data[off + 5]:#04x}"
        assert data[off + ln - 1] == 0x55, \
            f"id {mid:#04x}: length {ln} does not end on 0x55"

    with open(sys.argv[3], "w") as f:
        f.write(f"""/* emount_packets.h -- GENERATED by rebuild/tools/gen_packets.py, do not edit.
 * Source: {sys.argv[1]}, .data initialiser at flash {SIDATA:#07x}, {DATA_LEN} bytes.
 */
#ifndef EMOUNT_PACKETS_H
#define EMOUNT_PACKETS_H

#include <stdint.h>

#define EA9_DATA_LEN {DATA_LEN}
#define EA9_PACKET_COUNT {len(table)}

struct ea9_packet {{
\tuint8_t  id;
\tuint16_t offset;   /* into ea9_data */
\tuint16_t len;
}};

/* Live, writable: the init handlers for ids 0x1b and 0x34 edit their frame
 * before sending it, and the status frames are rewritten every body frame. */
extern uint8_t ea9_data[EA9_DATA_LEN];
extern const struct ea9_packet ea9_packets[EA9_PACKET_COUNT];

""")
        for mid, (off, ln, _) in sorted(table.items()):
            f.write(f"#define EA9_OFF_{mid:02X} {off:#05x}u\n")
            f.write(f"#define EA9_LEN_{mid:02X} {ln}u\n")
        f.write(f"""
/* Frame edits the stock handlers make before sending.  Three handlers do this
 * and none of them is optional -- see decode_handlers() in the generator for
 * what happened when they were dropped.  EA9_FIXUP_RX means the byte is copied
 * from the received frame at that offset. */
#define EA9_FIXUP_IMM 0
#define EA9_FIXUP_RX  1
#define EA9_FIXUP_COUNT {len(fixups)}

struct ea9_fixup {{
\tuint8_t id;
\tuint8_t off;      /* into the outgoing frame */
\tuint8_t kind;     /* EA9_FIXUP_IMM or EA9_FIXUP_RX */
\tuint8_t val;      /* the constant, or the offset into the received frame */
}};

extern const struct ea9_fixup ea9_fixups[EA9_FIXUP_COUNT];

#endif /* EMOUNT_PACKETS_H */
""")

    with open(sys.argv[2], "w") as f:
        f.write("/* emount_packets.c -- GENERATED by rebuild/tools/gen_packets.py, do not edit. */\n")
        f.write('#include "emount_packets.h"\n\n')
        f.write("uint8_t ea9_data[EA9_DATA_LEN] = {")
        for i, b in enumerate(data):
            if i % 12 == 0:
                f.write("\n\t")
            f.write(f"0x{b:02x}, ")
        f.write("\n};\n\n")
        f.write("const struct ea9_packet ea9_packets[EA9_PACKET_COUNT] = {\n")
        for mid, (off, ln, _) in sorted(table.items()):
            f.write(f"\t{{ 0x{mid:02x}, {off:#05x}, {ln:3d} }},\n")
        f.write("};\n\n")
        f.write("const struct ea9_fixup ea9_fixups[EA9_FIXUP_COUNT] = {\n")
        for mid, off, kind, val in fixups:
            k = "EA9_FIXUP_RX " if kind == "rx" else "EA9_FIXUP_IMM"
            f.write(f"\t{{ 0x{mid:02x}, {off:2d}, {k}, {val:#04x} }},\n")
        f.write("};\n")

    print(f"{len(table)} packets, {DATA_LEN} bytes of .data")
    for mid, (off, ln, fx) in sorted(table.items()):
        note = "" if mid in handlers else "   (status, sent unsolicited)"
        print(f"  id {mid:#04x}  .data+{off:#05x}  len {ln:3d}{note}")
        for off2, kind, val in fx:
            src = f"rx[{val}]" if kind == "rx" else f"{val:#04x}"
            was = data[off + off2]
            print(f"           frame[{off2}] := {src:<8} "
                  f"(stored {was:#04x})")
    silent = [m for m, v in handlers.items() if v is None]
    if silent:
        print("  answered without sending: " + ", ".join(f"{m:#04x}" for m in silent))


if __name__ == "__main__":
    main()
