#!/usr/bin/env python3
"""decode_proto_diag.py -- read the trail left by the protocol build.

  sudo tools/ea9flash.py --dump 0x16000:0x2100 -o dumps/proto.bin
  tools/decode_proto_diag.py dumps/proto.bin

The address was 0x1f000:0x900 here for a long time and has not been right
since the trail grew to four slots: it dumps erased flash, which decodes as a
run that never happened.

Thirty-two pages in eight rows per slot, four slots alternating by boot,
erased at boot and written one at a time, so THE LAST PAGE PRESENT IS WHERE IT
STOPPED.  Pages 0 and 1 are milestones written
before and after the bus comes up; pages 4..15 are timed snapshots that
straddle the two-second cutoff on both sides, which is what dates the power cut.

Every page carries its own magic and format word.  The motor build reuses these
same rows with a different magic, and an older run's pages are skipped rather
than decoded -- an earlier decoder did decode stale rows and reported movement
that never happened (NOTES.md §17).
"""
import struct
import sys

MAGIC = 0x544F5250
# Formats this decoder understands.  It must accept EVERY format it knows, not
# just the newest: gating acceptance on one value threw away a complete hardware
# run when the firmware went to format 2 and this constant did not -- the boot
# tally said the run had reached PA23 while its slot decoded as empty.
# Acceptance is a range; per-field meaning is gated separately, where it changed.
FORMATS = (1, 2, 3, 4, 5)
FORMAT_LATEST = max(FORMATS)
NORM = 0x4E4F524D
CHK  = 0x4348_4B21
CAP  = 0x43415000
MOTR = 0x4D4F5452
FOCS = 0x464F4353
FLST = 0x464C5354
M08  = 0x3830304D
TRAC = 0x54524143
DRY0, DRY1 = 0x30595244, 0x31595244
QUIET = 0x54495551
FAILP = 0x4C494146

# Only these are timed snapshots.  A WHITELIST, because the previous blacklist
# meant every page type added later was silently treated as a snapshot: the DRY
# marker page became the "last" one and the whole run summary was decoded out
# of it -- every milestone "never", reset cause 0x00, and a cutoff between 1 ms
# and 3000 ms for a run that reached 12 s.  Third time an unknown page has
# corrupted a report (NOTES.md §29).
SNAP_TAGS = (0x424F4F54, 0x494E4954, 0x534E4150)
TALLY_OFF, TALLY_SLOTS = 0x800, 64
PAGE = 64

TAGS = {0x424F4F54: "BOOT  app is executing",
        0x494E4954: "INIT  bus brought up, body pulsed its chip select",
        0x534E4150: "SNAP  timed snapshot",
        0x49444C47: "IDLG  the ids the body asked for, in order",
        0x464F4353: "FOCS  a focus move commanded by the body",
        0x46484953: "FHIS  focus history: the body's last targets and mode bytes",
        0x464C5354: "FLST  the LAST focus moves, written when the bus went quiet",
        0x4E4F524D: "NORM  normal-class traffic, by frame length",
        0x3830304D: "M008  the body's message 0x08 request, raw",
        0x43484B21: "CHK!  build consistency",
        0x43415000: "CAP   raw bytes of one receive window",
        0x4D4F5452: "MOTR  supply and encoder around one drive step",
        0x54524143: "TRAC  raw encoder across one drive pulse",
        0x30595244: "DRY0  build marker: motor drive ENABLED",
        0x31595244: "DRY1  build marker: motor drive SUPPRESSED (control)",
        0x54495551: "QUIT  the body stopped sending",
        0x4C494146: "FAIL  the handshake did not complete"}

# What the stock firmware does with each normal-class frame length (EA9.md 2.2).
# The whole point of the NORM page is to find out whether a modern body ever
# emits 27 or 36, which no capture in captures/ contains.
LENGTH_MEANING = {
    22: "frame clock -- clocks the stock's executor one step",
    23: "*** STOP / ABORT: motor_stop, event 0x1C ***",
    27: "*** ONE-STAGE MOVE: pl[14:16] -> target A, pl[16:18] -> target B ***",
    36: "*** TWO-STAGE MOVE: pl[21:23] -> target A, then target B ***",
    29: "body 0x03, seen on the A6000, not in the EA9's length table",
    32: "body 0x03, seen on the A6000, not in the EA9's length table",
}
COMMAND_LENGTHS = (27, 36)

# Indexed by servo_result.outcome.  ABORTED is the body's 23-byte stop command
# arriving mid-move: the target was deliberately NOT reached, so a decoder that
# printed it as a failure would report a working stop as a fault.
OUTCOMES = ("OK", "STALL", "TIMEOUT", "RUNAWAY", "WRONG_WAY", "ABORTED")

# enum em_focus_op -- which INSTRUCTION a leg of motion belonged to.  Before
# format 4 this byte was the channel a target arrived on (1 = msg 0x1B,
# 2 = msg 0x04), which is why the numbering starts where it does.
OPS = {0: "-", 1: "Move", 2: "Move/dist", 3: "Move/rel", 4: "Move/defocus",
       5: "Stop", 6: "Scan", 7: "Drive"}

# When each page is scheduled, so a missing page dates the power cut.
SCHEDULE = {2: 700, 3: 3000,
            4: 500, 5: 1000, 6: 1500, 7: 2000, 8: 2500, 9: 3000,
            10: 4000, 11: 5000, 12: 6000, 13: 7000, 14: 8500, 15: 10000}

RCAUSE = [(0, "POR"), (1, "BOD12"), (2, "BOD33 brown-out"),
          (4, "EXT"), (5, "WDT"), (6, "SYST")]

# The handshake the A6000 captures show, in order (CLAUDE.md / EA9.md 2.5).
# A newer body is entitled to ask for a different set -- the a9 II asks for
# 0x0c, which appears nowhere in that capture -- so treat this as a reference
# sequence, not a requirement.
HANDSHAKE = [0x01, 0x07, 0x0B, 0x08, 0x09, 0x0D, 0x10, 0x0A]


def summarise_motor(motr, dry=False):
    """The supply-and-encoder verdict.  Reachable from both paths through
    main(): an earlier version returned early when a dump had no timed
    snapshots, which suppressed this entirely on the very run it was written
    for -- the one where the motor finally moved."""
    if not motr:
        return
    if dry:
        print("\n--- CONTROL RUN: the coils were never energised ---")
    else:
        print("\n--- supply under motor load ---")
    base = None
    moved = False
    for p in sorted(motr):
        mw = motr[p]
        duty = mw[4] - (1 << 32) if mw[4] >> 31 else mw[4]
        eb = mw[8] - (1 << 32) if mw[8] >> 31 else mw[8]
        ea = mw[9] - (1 << 32) if mw[9] >> 31 else mw[9]
        sag = (mw[5] & 0xFFFF) - (mw[6] & 0xFFFF)
        if duty == 0:
            base = sag
        # The encoder jitters 2-3 counts standing still, so anything at or
        # under the servo's own tolerance is not movement.  A threshold of 2
        # reported "THE MOTOR MOVED" for a dry run whose largest reading was
        # 3 counts of noise.
        if abs(ea - eb) > 16:
            moved = True
        # No wrap caveat any more: these are abs_encoder_track() values, which
        # accumulate unwrapped.  The warning belonged to the open-loop build
        # that logged abs_encoder_position(), and printing it on a correct
        # 2002-count move was just noise.
        pct = (ea - eb) * 100.0 / 5952
        print(f"  commanded {duty:+5d} counts   supply dips {sag:+4d} mV "
              f"   encoder {ea - eb:+d}  ({pct:+.1f}% of travel)")
    print()
    if dry:
        print("  Nothing was driven, so the encoder figures above are the noise")
        print("  floor and the supply figures are the baseline for a run of the")
        print("  same shape WITHOUT motor current.  Compare a real run to these.")
        return
    if moved:
        print("  THE MOTOR MOVED.  Neither §17 explanation applies -- it was not")
        print("  current limiting and not a dead drive path.  What stopped it")
        print("  before was the 2 s cutoff itself: it never had time.")
    else:
        drive = [((motr[p][5] & 0xFFFF) - (motr[p][6] & 0xFFFF))
                 for p in motr if motr[p][4] != 0]
        worst = max(drive) if drive else 0
        if base is not None and worst - base > 50:
            print(f"  Nothing moved, and the rail sags {worst - base} mV more")
            print( "  under drive than at idle.  That is current being drawn --")
            print( "  so the drive IS reaching something, and the limit is power.")
        else:
            print("  Nothing moved and the rail did not sag beyond the idle")
            print("  baseline.  On the logic rail that is WEAK evidence for the")
            print("  drive not arriving -- the motor runs from a separate")
            print("  contact (vdd.c).  Check the boot tally for BOD33 before")
            print("  concluding anything.")


MARKS = {0xB0: "boot", 0xB2: "bus up", 0xB3: "handshake",
         0xB4: "0x16 acked", 0xB5: "PA23 driven", 0xB6: "NO BODY"}


def summarise_tally(blob, tally_off):
    """The boot tally: a milestone timeline that survives the trail slots.

    Only two boots fit in the trail, and the boots that fail are exactly the
    ones the next boot overwrites.  These marks are appended to a row that is
    never erased, so every boot leaves at least a trace of how far it got.
    """
    if len(blob) <= tally_off:
        return
    words = [struct.unpack_from("<I", blob, tally_off + 4 * i)[0]
             for i in range(TALLY_SLOTS)
             if tally_off + 4 * i + 4 <= len(blob)]
    if not words or words[0] == 0xFFFFFFFF:
        print("\n  boot tally row is erased -- this build did not write it")
        return

    stamp = None
    if (words[0] >> 24) == 0xB1:
        stamp = words[0] & 0xFFFFFF
    marks = [w for w in words[1:] if w != 0xFFFFFFFF]

    when = ""
    if stamp is not None:
        # An opaque build id now -- the low 24 bits of the build's epoch
        # second -- not a time of day.  Formatting it as a clock gave
        # "3195:25:39".
        when = f", build {stamp:#08x}"
    boots = {}
    for w in marks:
        code, idx, data = w >> 24, (w >> 8) & 0xFF, w & 0xFF
        boots.setdefault(idx, []).append((code, data))

    # An image from before the milestone marks writes only BOOT marks, and
    # calling every one of those "died before the bus came up" would be a
    # confident lie about a dump that simply cannot say.
    milestones = any(c != 0xB0 for got in boots.values() for c, _ in got)
    nobody = 0

    print(f"\n--- boot tally ({len(boots)} boot(s) since this image was "
          f"flashed{when}) ---")
    if not milestones:
        print("  (boot marks only -- this image predates the milestone marks,")
        print("   so how far each boot got cannot be told from here)")
    for idx in sorted(boots):
        got = boots[idx]
        names = []
        for code, data in got:
            n = MARKS.get(code, f"{code:#04x}")
            if code == 0xB0:
                rc = [x for b, x in RCAUSE if data & (1 << b)] or [f"{data:#04x}"]
                n += f" ({', '.join(rc)})"
            names.append(n)
        reached = {c for c, _ in got}
        if not milestones:
            print(f"  boot {idx:2d}   {' -> '.join(names)}")
            continue
        if 0xB3 in reached:
            verdict = "ran" if 0xB4 in reached else "ran, no shutdown request"
        elif 0xB2 in reached:
            verdict = "*** bus came up, handshake never completed ***"
        elif 0xB6 in reached:
            # Not a failure.  em_init waits for a chip select that a USB port
            # or a bench supply never raises, so the app sits there for as
            # long as it is powered.  Before this mark existed every flashing
            # and dumping session showed up as a dead boot.
            nobody += 1
            verdict = "no camera attached (flashing or bench power)"
        else:
            verdict = "*** died before the bus came up ***"
        print(f"  boot {idx:2d}   {' -> '.join(names)}")
        print(f"           {verdict}")

    real = len(boots) - nobody
    if nobody:
        print(f"\n  {nobody} of these {len(boots)} boots had no camera attached; "
              f"{real} did.")
    if real > 1:
        print(f"\n  {real} boots on a camera since flashing.  If the adapter was")
        print( "  mounted fewer times than that, the body is restarting it --")
        print( "  and the lines above say how far each attempt got.")
    elif real == 1 and not nobody:
        print("\n  One boot since flashing.")


def decode_ids(lo, hi):
    return [i for i in range(64)
            if ((lo >> i) & 1 if i < 32 else (hi >> (i - 32)) & 1)]


def decode_trail(blob, label):
    """Decode one trail slot.  Returns True if it held anything."""
    print(label)
    pages = {}
    for p in range(len(blob) // PAGE):
        w = list(struct.unpack_from("<16I", blob, p * PAGE))
        if w[0] == MAGIC and w[1] in FORMATS:
            pages[p] = w

    print(f"  pages present: {sorted(pages) if pages else 'NONE'}\n")
    if not pages:
        print("  Empty -- this slot holds no run from this build.\n")
        return False

    # DID THE APP REACH main()?
    #
    # This used to look for a BOOT page at page 0.  The start-up pages were
    # removed once start-up stopped being the question, and page 0 now holds a
    # focus record -- so the check fired on every slot of every healthy run and
    # said the app had crashed.  An instrument that cries wolf on a good run is
    # worse than no instrument.
    #
    # The build-consistency page is the honest marker: main() writes it
    # unconditionally, before anything that can fail.
    if not any(w[2] == CHK for w in pages.values()) \
       and not any(w[2] == MOTR for w in pages.values()):
        print("  WARNING: no build-consistency page.  The app did not reach"
              " main.\n")

    norms = {p: w for p, w in pages.items() if w[2] == NORM}
    chks  = {p: w for p, w in pages.items() if w[2] == CHK}
    caps  = {p: w for p, w in pages.items() if w[2] == CAP}
    motr  = {p: w for p, w in pages.items() if w[2] == MOTR}
    trac  = {p: w for p, w in pages.items() if w[2] == TRAC}
    quiet = {p: w for p, w in pages.items() if w[2] == QUIET}
    failp = {p: w for p, w in pages.items() if w[2] == FAILP}
    snaps = {p: w for p, w in pages.items() if w[2] in SNAP_TAGS}
    dry   = any(w[2] == DRY1 for w in pages.values())
    # Derived from TAGS, not hand-maintained.  A hand-written exclusion list
    # has now drifted three times -- every page type added since it was written
    # decoded correctly and was ALSO reported as unknown.
    unknown = {p: w[2] for p, w in pages.items()
               if w[2] not in SNAP_TAGS and w[2] not in TAGS}
    if unknown:
        print("  NOTE: page(s) with tags this decoder does not know: "
              + ", ".join(f"{p}:{t:#010x}" for p, t in sorted(unknown.items())))
        print("  They are ignored rather than guessed at.\n")
    last = snaps[max(snaps)] if snaps else None

    for p in sorted(pages):
        w = pages[p]
        tag = TAGS.get(w[2], f"?{w[2]:#010x}")
        if w[2] == 0x49444C47:
            n = min(w[3], 40)          # the log is capped at 10 words here
            ids = [(w[4 + i // 4] >> (8 * (i % 4))) & 0xFF for i in range(n)]
            print(f"page {p:2d}  {tag}")
            print(f"         {' '.join(f'{i:#04x}' for i in ids) or '(none)'}")
            offer = w[14] | (w[15] << 32)
            if offer:
                have = [m + 1 for m in range(64) if offer >> m & 1]
                raw = " ".join(f"{(offer >> (8 * k)) & 0xFF:02x}" for k in range(8))
                print(f"         body's capability offer: {raw}"
                      f"   ({len(have)} ids, up to {max(have):#04x})")
                for mid in (0x03, 0x04, 0x1B):
                    mark = "offered" if offer >> (mid - 1) & 1 else "NOT OFFERED"
                    print(f"           id {mid:#04x}: {mark}")
            continue
        if w[2] == TRAC:
            n, step = (w[3] >> 16) & 0xFFFF, w[3] & 0xFFFF
            raw = [(w[4 + i // 2] >> (16 * (i % 2))) & 0xFFFF for i in range(n)]
            # Unwrap between consecutive samples: the true step is the one of
            # (d, d-16384, d+16384) with the smallest magnitude.  That is only
            # ambiguous if the mechanism moved more than half a magnet turn
            # between two samples, which it cannot at these speeds.
            pos, acc = [0], 0
            for a, b in zip(raw, raw[1:]):
                d = (b - a) & 0x3FFF
                if d > 8192:
                    d -= 16384
                acc += d
                pos.append(acc)
            print(f"page {p:2d}  {tag}: step {step}, {n} samples")
            print(f"         raw:   {' '.join(f'{v:5d}' for v in raw[:8])} ...")
            print(f"         moved: {acc:+d} encoder counts "
                  f"({acc / 4:+.0f} protocol units), unwrapped")
            if len(pos) > 4:
                q = len(pos) // 4
                print(f"         quarters: {pos[q]:+d} {pos[2*q]:+d} "
                      f"{pos[3*q]:+d} {pos[-1]:+d}")
            continue
        if w[2] == 0x46484953:
            ring_n, t04n = w[4] >> 16, w[4] & 0xFFFF
            f1b, modes = w[5] >> 16, w[5] & 0xFFFF
            tgts = [(w[6 + i // 2] >> (16 * (i % 2))) & 0xFFFF for i in range(10)]
            # The ring is oldest-first only once it has wrapped; before that
            # the tail is still zero.
            order = (list(range(ring_n % 10, 10)) + list(range(ring_n % 10))
                     if ring_n > 10 else list(range(min(ring_n, 10))))
            print(f"page {p:2d}  {tag}  (written at {w[3]} ms)")
            print(f"         {t04n} targets on 0x04, {f1b} frames on 0x1B")
            shown = [("none" if tgts[i] == 0x7FFF else str(tgts[i])) for i in order]
            print(f"         last targets, oldest first: {' '.join(shown) or '(none)'}")
            for i in range(min(modes, 6)):
                v = (w[11 + i // 2] >> (16 * (i % 2))) & 0xFFFF
                mv, mc = v & 0xFF, v >> 8
                known = {0x09: "idle", 0x01: "focusing",
                         0x31: "A6000 -> native", 0x40: "focusing",
                         0x41: "focusing"}.get(mv, "*** UNSEEN BEFORE ***")
                print(f"         0x04 mode byte {mv:#04x} x{mc:<5d} {known}")
            continue
        if w[2] == FOCS:
            delta = w[4] - (1 << 32) if w[4] >> 31 else w[4]
            tgt, got = w[5] >> 16, w[5] & 0xFFFF
            t04 = w[6] >> 16
            moves, seen = w[14] & 0xFFFF, w[14] >> 16
            print(f"page {p:2d}  {tag}  FOCUS move #{moves} at t={w[3]} ms")
            print(f"         0x1B target {tgt}  ->  reported {got} on arrival"
                  f"   ({delta:+d} counts commanded)")
            outc = OUTCOMES
            oi = (w[12] >> 16) & 0xFF
            eb = w[8] - (1 << 32) if w[8] >> 31 else w[8]
            ea = w[9] - (1 << 32) if w[9] >> 31 else w[9]
            ms = w[12] & 0xFFFF
            moved = ea - eb
            rt = (w[15] >> 16) & 0xFF
            print(f"         encoder {eb} -> {ea}  ({moved:+d} counts), "
                  f"duty {w[7] & 0xFFFF} at breakaway, peak {w[7] >> 16}, "
                  f"{outc[oi] if oi < len(outc) else oi} in {ms} ms")
            # ERROR AND AVERAGE SPEED, computed rather than left to the reader.
            #
            # The error is what says whether the controller arrived; the
            # average speed is what says whether it took a sensible time to.
            # Both had to be worked out by hand from the af1 dump, and the
            # second one is where that run's finding was: a 69-count move
            # took 446 ms, LONGER than a 1660-count one, because every move
            # pays a fixed endgame in the dead zone whatever its length.
            if ms:
                print(f"         error {moved - delta:+d} counts, "
                      f"average {abs(moved) / ms:.2f} counts/ms "
                      f"(cruise allows 25)")
            # RETARGETS.  Recorded since format 3 and never printed.  Zero
            # across every move of the af1 run, which says this body does not
            # redirect in flight at all -- it waits, or it sends a Stop.  All
            # the mid-move retargeting machinery is therefore untouched by
            # this camera, and that is worth knowing before tuning any of it.
            print(f"         retargets {rt}")
            # w[6]'s low half-word was a second copy of em_t04_n until
            # format 3.  Reading it as a frame period on an older page would
            # print a plausible number built out of an unrelated counter, so
            # it is gated on the page's own format word.
            if w[1] >= 3:
                per, pern = (w[6] >> 8) & 0xFF, w[6] & 0xFF
                print("         body frame period "
                      + (f"{per} ms ({pern} samples accepted)" if pern
                         else "NOT MEASURED -- forecasts used the default"))
            # Format 3 moved the source code into the TOP byte to make room
            # for the retarget count; before that it was the high half-word.
            # Format 3 covers two packings: the first 3-format build still
            # wrote src in the high half-word, and only the retargeting build
            # moved it to the top byte.  Told apart by which field holds a
            # valid source code -- without this, a pre-retargeting page
            # decodes as "commanded via ?0, REDIRECTED 2x", which is both
            # wrong and believable.
            if w[1] >= 3 and (w[15] >> 24) in (1, 2, 3, 4, 5, 6, 7):
                srcn, retgt = w[15] >> 24, (w[15] >> 16) & 0xFF
            else:
                srcn, retgt = (w[15] >> 16) & 0xFF, None
            # Format 4 replaced the channel code with the INSTRUCTION: the
            # channel question is closed -- every target comes on 0x04 -- and
            # which record asked for a leg is the live one.
            if w[1] >= 4:
                src = OPS.get(srcn, f"?{srcn}")
            else:
                src = {1: "0x1B", 2: "0x04"}.get(srcn, f"?{srcn}")
            print(f"         {'instruction' if w[1] >= 4 else 'commanded via'}"
                  f" {src}   (0x1B frames seen: {seen},"
                  f" 0x04 targets: {w[15] & 0xFFFF}"
                  + (f", last {t04}" if w[15] & 0xFFFF else "") + ")")
            if retgt:
                print(f"         REDIRECTED {retgt}x mid-move -- the move that "
                      f"ran is not the one commanded above")
            continue
        if w[2] == FLST:
            OUT = OUTCOMES
            total, ringn = w[4] & 0xFFFF, w[4] >> 16
            print(f"page {p:2d}  {tag}  (written at {w[3]} ms)")
            print(f"         {total} focus move(s) this session; the last "
                  f"{min(ringn, 3)} of them, oldest first:")
            for k in range(3):
                a, d, t = w[5 + k * 3], w[6 + k * 3], w[7 + k * 3]
                if not (a or d or t):
                    continue
                oi, rg, ms = a >> 24, (a >> 16) & 0xFF, a & 0xFFFF
                delta = d - (1 << 32) if d >> 31 else d
                print(f"           target {t >> 16:5d} -> stopped at "
                      f"{t & 0xFFFF:5d}   ({delta:+6d} counts, "
                      f"{OUT[oi] if oi < len(OUT) else oi} in {ms} ms"
                      + (f", REDIRECTED {rg}x" if rg else "") + ")")
            src = (OPS.get(w[14] & 0xFF, f"?{w[14] & 0xFF}") if w[1] >= 4
                   else {1: "0x1B", 2: "0x04"}.get(w[14] & 0xFF,
                                                   f"?{w[14] & 0xFF}"))
            print(f"         newest commanded via {src};  0x1B frames seen "
                  f"{w[15] >> 16}, 0x04 targets {w[15] & 0xFFFF}")
            idle_stops = w[14] >> 24
            stops, aborts = (w[14] >> 16) & 0xFF, (w[14] >> 8) & 0xFF
            print(f"         {stops} stop command(s) from the body "
                  f"(8-bit, saturates; NORM has the full count): "
                  f"{aborts} cut a move short, {idle_stops} arrived with "
                  f"nothing running")
            continue
        if w[2] == MOTR:
            # w[4] is the COMMANDED DISPLACEMENT in counts, and w[7] is the
            # duty pair.  An earlier version printed w[4] as a duty percentage
            # and w[7] as "supply after", which showed the ramp duties 102..204
            # as millivolts.
            delta = w[4] - (1 << 32) if w[4] >> 31 else w[4]
            enc_b = w[8] - (1 << 32) if w[8] >> 31 else w[8]
            enc_a = w[9] - (1 << 32) if w[9] >> 31 else w[9]
            first, peak = w[7] & 0xFFFF, w[7] >> 16
            print(f"page {p:2d}  {tag}: step {w[15]}, commanded {delta:+d} counts "
                  f"({delta / 4:+.0f} protocol units) at t={w[3]} ms")
            for label, word in (("idle", w[5]), ("load", w[6])):
                print(f"         supply {label}  {word & 0xFFFF:4d} .. "
                      f"{word >> 16:4d} mV")
            print(f"         duty    moved at {first} ({first * 100 // 2560}%), "
                  f"peak {peak} ({peak * 100 // 2560}%) of 2560")
            outc = OUTCOMES
            oi = (w[12] >> 16) & 0xFF
            print(f"         outcome {outc[oi] if oi < len(outc) else oi} "
                  f"in {w[12] & 0xFFFF} ms")
            print(f"         encoder {enc_b} -> {enc_a}   "
                  f"({enc_a - enc_b:+d} counts)")
            print(f"         bus still alive: rx={w[10]} tx={w[11]} "
                  f"handshake={'yes' if w[12] else 'no'}")
            continue
        if w[2] == FAILP:
            print(f"page {p:2d}  {tag}  (written at {w[3]} ms)")
            print(f"         first VD edge      {w[4] or 'never'} ms"
                  f"   ({w[5]} edges total)")
            print(f"         body CS first high {(w[6] & 0xFFFF) or 'never'} ms"
                  f"   (window held open {w[6] >> 16} ms)")
            print(f"         bus init complete  {w[7] or 'never'} ms")
            print(f"         first frame        {w[8] or 'never'} ms")
            print(f"         last frame         {(w[9] & 0xFFFF) or 'never'} ms")
            print(f"         first BYTE          {(w[9] >> 16) or 'never'} ms"
                  f"   (vs CS high at {(w[6] & 0xFFFF) or '?'} ms)")
            print(f"         rx frames {w[10]}  tx frames {w[11]}  "
                  f"captured {w[14] >> 24}")
            print(f"         body CS edges      {w[12]}")
            total, inwin = w[13] & 0xFFFF, w[13] >> 16
            print(f"         raw bytes received {total}"
                  f"   ({inwin} of them inside the first window)")
            st, errs = (w[14] >> 8) & 0xFFFF, w[14] & 0xFF
            if w[14] >> 31:
                print("         a frame received during bring-up was RESCUED")
            st &= 0x7FFF
            bits = [n for b, n in ((0, "PERR"), (1, "FERR"), (2, "BUFOVF"),
                                   (3, "CTS"), (4, "ISF"), (5, "COLL"))
                    if st & (1 << b)]
            print(f"         UART errors        {errs}"
                  + (f"  ({', '.join(bits)})" if bits else ""))
            if w[12] <= 2 and total == 0:
                print("         -> the body pulsed its chip select once and then")
                print("            said nothing at all.  It is not us mis-framing;")
                print("            no bytes ever arrived.")
            elif inwin and not w[10]:
                print("         -> the body DID send inside its window and no frame")
                print("            parsed: baud, polarity or framing.")
            elif total and not inwin:
                print("         -> every byte arrived OUTSIDE the window.  The body")
                print("            sent nothing; that is the idle line, not traffic.")
            continue
        if w[2] == QUIET:
            print(f"page {p:2d}  {tag}: tick {w[4] & 0xFFFF} at t={w[3]} ms")
            print(f"         last frame at {w[7]} ms -- still alive "
                  f"{w[8]} ms after the bus went quiet")
            print(f"         rx={w[5]} tx={w[6]}  last id={w[12] >> 24:#04x} "
                  f"shutdown_acked={'yes' if (w[12] >> 8) & 1 else 'no'}")
            print(f"         encoder at {w[13] - (1 << 32) if w[13] >> 31 else w[13]} counts")
            print(f"         0x16 acked at {w[9] or 'never'} ms, "
                  f"PA23 driven at {w[15] or 'never'} ms")
            # w[14] was RCAUSE; it now carries the focus-channel tally, which
            # is the one number this build exists to produce.  Old dumps have
            # RCAUSE here and will read as implausibly large counts -- the
            # format word in w[1] is the guard against mixing them up.
            f1b, f04 = w[14] >> 16, w[14] & 0xFFFF
            verdict = ("body drove focus via 0x1B" if f1b else
                       "body sent NO 0x1B -- it uses 0x04" if f04 else
                       "no focus command on either channel")
            print(f"         focus channel: 0x1B x{f1b}, 0x04 x{f04}"
                  f"   <-- {verdict}")
            continue
        if w[2] in (DRY0, DRY1):
            print(f"page {p:2d}  {tag}")
            # The ASCII tag in w[5] is the discriminator, NOT the number in
            # w[4]: images older than the boot-regression bisect wrote neither,
            # and w[4] == 0 there means "field absent", not "variant 0".  Keying
            # off w[4] labelled those dumps as the no-shutdown arm, which is the
            # opposite of what they were.
            #
            # Only BOFF is built now; the rest are the bisect's arms
            # (NOTES.md §52-53), kept so its dumps still decode.  A historical
            # dump is not an error and must not be flagged as one.
            variants = {0x48534F4E: ("NOSH", 0, "shutdown path REMOVED -- no ack, park or PA23"),
                        0x54554853: ("SHUT", 1, "full path, bus LEFT ENABLED"),
                        0x3332414E: ("NA23", 2, "ack + park, NO PA23"),
                        0x46464F42: ("BOFF", 3, "full stock shutdown incl. bus off")}
            if w[5] == 0:
                print("         (built before the shutdown-variant marker existed)")
            elif w[5] in variants:
                name, expect, desc = variants[w[5]]
                note = "" if name == "BOFF" else "   [historical]"
                print(f"         {name}: {desc}{note}")
                if w[4] != expect:
                    print(f"         *** marker inconsistent: {name} but w[4]={w[4]},"
                          f" expected {expect}")
            else:
                print(f"         *** unrecognised variant tag {w[5]:#010x}")
            continue
        if w[2] == CAP:
            declared, got, mid = w[3] & 0xFF, (w[3] >> 8) & 0xFF, (w[3] >> 16) & 0xFF
            raw = bytes((w[4 + i // 4] >> (8 * (i % 4))) & 0xFF for i in range(48))
            flag = "" if declared == got else f"   <-- got {got}, not {declared}"
            print(f"page {p:2d}  {tag}: id {mid:#04x}, declared {declared}{flag}")
            body = raw[:got] if got <= 48 else raw
            for off in range(0, len(body), 16):
                chunk = body[off:off + 16]
                print(f"         +{off:02x}  {chunk.hex(' ')}")
            if got > declared and declared >= 6:
                tail = raw[declared:got]
                print(f"         the {len(tail)} byte(s) past the declared length: "
                      f"{tail.hex(' ')}")
                # The buffer is pre-filled with 0xA5 at the start of every
                # window, so a byte still reading 0xA5 was never delivered.
                poison = sum(1 for b in tail if b == 0xA5)
                if poison == len(tail):
                    print("         all still poison (0xa5): those bytes were NEVER")
                    print("         received -- the length counter is over-reporting.")
                elif poison:
                    print(f"         {poison} of {len(tail)} still poison: the window "
                          f"really did carry {len(tail) - poison} extra byte(s).")
                else:
                    print("         none is poison: the body really sent every one.")
                if len(tail) >= 6 and tail[0] == 0xF0:
                    print(f"         -- that is another frame: len {tail[1]}, "
                          f"class {tail[3]:#04x}, id {tail[5]:#04x}")
            continue
        if w[2] == CHK:
            print(f"page {p:2d}  {tag}")
            ok = "OK" if w[3] == w[4] else "*** MISMATCH ***"
            print(f"         trail capacity: main says {w[3]}, "
                  f"diag.c says {w[4]}   {ok}")
            if w[3] != w[4]:
                print("         Two objects were compiled against different "
                      "versions of diag.h.")
                print("         Rebuild from clean; the Makefile should be "
                      "emitting -MMD header deps.")
            print(f"         boot tally row at {w[5]:#07x}")
            if w[8]:
                print(f"         SOURCE ID {w[8]:#08x}   "
                      f"(same value = built from identical source)")
            if w[6] or w[7]:
                # board_init_pins, PLUS main's one addition: it drives PA16
                # high immediately afterwards, and trail_chk runs after that.
                # Expecting the bare board_init_pins value here reported every
                # single dump as "*** WRONG: PA16 ***".
                want = {7: 1, 10: 0, 11: 1, 16: 1, 23: 0}
                bad = [p2 for p2, lvl in want.items()
                       if not (w[6] >> p2) & 1 or ((w[7] >> p2) & 1) != lvl]
                names = " ".join(f"PA{p2:02d}" for p2 in sorted(want))
                print(f"         pin setup ({names}): "
                      + ("OK" if not bad else
                         "*** WRONG: " + " ".join(f"PA{p2:02d}" for p2 in bad)
                         + " -- board_init_pins did not run or did not take ***"))
            continue
        if w[2] == M08:
            n, on, ln = w[3] >> 16, (w[3] >> 8) & 0xFF, w[3] & 0xFF
            raw = bytes((w[4 + i // 4] >> (8 * (i % 4))) & 0xFF for i in range(ln))
            print(f"page {p:2d}  {tag}: {n} arrived, {ln} byte(s) captured")
            if not ln:
                print("         none seen -- the handshake never got this far")
                continue
            # The capture is bytes from the RECEIVE BUFFER, not a frame: the
            # window is usually longer than the frame in it, so split at the
            # declared length rather than showing one run of bytes that
            # invites reading the padding as payload.
            flen = (raw[1] | (raw[2] << 8)) if ln >= 3 else ln
            body = raw[:flen] if 0 < flen <= ln else raw
            for off in range(0, len(body), 16):
                print(f"         +{off:02x}  {body[off:off + 16].hex(' ')}")
            if 0 < flen < ln:
                tail = raw[flen:]
                print(f"         the frame is {flen} bytes; the window carried "
                      f"{ln - flen} more: {tail.hex(' ')}")
                if any(tail):
                    print("         (padding or the next frame -- not this one's"
                          " payload)")
            if flen >= 14:
                cs = sum(body[1:flen - 3]) & 0xFFFF
                got = body[flen - 3] | (body[flen - 2] << 8)
                print(f"         checksum {cs:#06x} vs {got:#06x}"
                      + ("   OK" if cs == got else "   *** MISMATCH ***")
                      + f",  payload {body[6:flen - 3].hex(' ')}")
            if ln >= 8:
                pl1 = raw[7]
                print(f"         pl[1] = {pl1:#04x}, bit 7 "
                      + ("SET"if pl1 & 0x80 else "CLEAR"))
                print("         -> message 0x28 pl[9..10] "
                      + ("carries the aperture" if on else "stays zero")
                      + ("   (the body asked for it)" if on
                         else "   (the body did not ask for it)"))
                if bool(pl1 & 0x80) == bool(on):
                    print("         *** the decoded bit and the flag disagree ***")
            continue
        if w[2] == NORM:
            used, overflow = (w[3] >> 16) & 0xFF, (w[3] >> 8) & 0xFF
            print(f"page {p:2d}  {tag}"
                  + ("   OVERFLOW: more kinds than slots" if overflow else ""))
            for i in range(min(used, 4)):
                ln, mid, cnt = w[6 + i] & 0xFF, (w[6 + i] >> 8) & 0xFF, w[6 + i] >> 16
                note = LENGTH_MEANING.get(ln, "not in the EA9's length table")
                print(f"         len {ln:3d}  id {mid:#04x}  x{cnt:<6d} {note}")
            # w[4] is the first four DISTINCT receive-window lengths, w[5]
            # packs (window kinds seen << 16 | truncated frames).  An earlier
            # version read both as if they were one "declared vs got" pair and
            # printed "196609 frame(s) shorter than declared" for a run with
            # exactly one truncation.
            wl = [(w[4] >> (8 * k)) & 0xFF for k in range((w[5] >> 16) & 0xFF)]
            if wl:
                print("         receive windows, by byte count: "
                      + ", ".join(str(v) for v in wl))
            if w[5] & 0xFFFF:
                print(f"         {w[5] & 0xFFFF} window(s) shorter than the "
                      f"frame length they declared")
            # The RECORDS inside those frames.  The length histogram can only
            # name the one-record forms; this is what the body actually asked
            # for, whatever it packed them into.
            # Format 4 moved the record counts into w[10..13]; before that
            # those words were the tail of the frame-length histogram, and
            # reading them as record counts on an old dump would invent
            # traffic that never happened.
            recs = [] if w[1] < 4 else [
                    ("Move   0x1D", w[10] >> 16), ("Stop   0x1C", w[10] & 0xFFFF),
                    ("Drive  0x3C", w[11] >> 16), ("Scan   0x1F", w[11] & 0xFFFF),
                    ("RowIdx 0x2F", w[12] >> 16), ("Query  0x22/0x2E", w[12] & 0xFFFF),
                    ("other tags", w[13] & 0xFFFF)]
            if recs and (any(n for _, n in recs) or (w[13] >> 16)):
                print("         message 0x04 records received:")
                for name, n in recs:
                    print(f"           {name:<18s} {n if n else '--':>6}")
                unk = w[13] >> 16
                print(f"           {'walk ended early':<18s} {unk if unk else '--':>6}"
                      + ("   <-- a tag whose size is unknown, or a"
                         " truncated record" if unk else ""))
            ev, stops = w[15] >> 16, w[15] & 0xFFFF
            print(f"         answered with {ev} tail frame(s); "
                  f"{stops} stop(s) reached the firmware")
            if w[1] >= 5:
                # Format 5 packs the two apertures the body asks for into
                # w[14]: 0x03's every-frame request and 0x1B's at the shutter.
                def fnum(v):
                    return f"{v} = f/{2 ** (((v - 4096) / 256.0) / 2):.2g}" if v else "--"
                print(f"         aperture asked for: message 0x03 "
                      f"{fnum(w[14] >> 16)}, message 0x1B {fnum(w[14] & 0xFFFF)}")
            elif w[1] == 4:
                n1b, req = w[14] >> 16, w[14] & 0xFFFF
                if n1b:
                    av = (req - 4096) / 256.0
                    print(f"         message 0x1B: {n1b} aperture command(s), "
                          f"last asked for {req} = f/{2 ** (av / 2):.2g}")
                else:
                    print("         message 0x1B: no aperture command at all")
            continue
        print(f"page {p:2d}  t={w[3]:6d} ms  boot#{w[4] & 0xffff}"
              f"/flash#{w[4] >> 16}  {tag}")
        print(f"         rx={w[5]:<6d} tx={w[6]:<6d}  "
              f"last id={w[14] >> 24:#04x} class={(w[14] >> 16) & 0xff:#04x} "
              f"bad={(w[14] >> 8) & 0xff}  handshake={'yes' if w[14] & 1 else 'no'}")

    # THE LIVE MEASUREMENTS COME FIRST, and do not depend on a snapshot page.
    #
    # The start-up and shut-down pages were removed once those questions were
    # closed, and the summaries below are about the protocol, not the boot --
    # so they must not sit behind a check for pages that no longer exist.
    if norms:
        nw = norms[max(norms)]
        used = (nw[3] >> 16) & 0xFF
        lens = {nw[6 + i] & 0xFF: nw[6 + i] >> 16 for i in range(min(used, 10))}
        print("\n--- normal-class traffic ---")
        print(f"  distinct (length, id) kinds        {used}")
        # The four inbound lengths the stock dispatches on, and which of
        # them this body actually emits.  Answer each one explicitly: a
        # length that is absent is a result, and printing nothing for it
        # reads as "not checked" rather than "not sent".
        for ln, what in ((22, "frame clock"),
                         (23, "stop / abort"),
                         (27, "one-stage move"),
                         (36, "two-stage move")):
            n = lens.get(ln)
            print(f"  length {ln:<3d} {what:<16s}"
                  + (f"   x{n}" if n else "   NOT SENT"))
        if 36 not in lens:
            print("  >> no two-stage command in this run.  The stock's 36-byte")
            print("     handler (EA9.md 3.3) is unreachable on this body, at")
            print("     least in whatever AF mode was used.")
        if 23 in lens and 27 in lens:
            print(f"  >> the body both commands and stops: {lens[27]} move(s),"
                  f" {lens[23]} stop(s).")

    if caps:
        print("\n--- captured windows ---")
        kinds = {}
        # NOT `w`: that name holds the last snapshot and the cutoff section
        # below still needs it.  Shadowing it here made the cutoff report a
        # capture header decoded as a timestamp -- the same shadowing bug as
        # the `d`/`blob` one in NOTES.md §17.
        for p in sorted(caps):
            cw = caps[p]
            kinds.setdefault((cw[3] & 0xFF, (cw[3] >> 16) & 0xFF), []).append(p)
        for (ln, mid), ps in sorted(kinds.items()):
            print(f"  len {ln:3d} id {mid:#04x}   pages {ps}")
        cmd = [(ln, mid, ps) for (ln, mid), ps in sorted(kinds.items())
               if ln in COMMAND_LENGTHS]
        for ln, mid, ps in cmd:
            a, b = (caps[ps[0]], caps[ps[1]]) if len(ps) > 1 else (None, None)
            if a and b:
                ra = bytes((a[4 + i // 4] >> (8 * (i % 4))) & 0xFF for i in range(48))
                rb = bytes((b[4 + i // 4] >> (8 * (i % 4))) & 0xFF for i in range(48))
                diff = [i for i in range(ln) if ra[i] != rb[i]]
                named = {4: "seq", ln - 3: "checksum", ln - 2: "checksum"}
                shown = ", ".join(f"{i}({named[i]})" if i in named else str(i)
                                  for i in diff)
                print(f"  >> the two length-{ln} captures differ at: {shown or 'nowhere'}")
                for off, label in ((20, "target A  pl[14:16]"),
                                   (22, "target B  pl[16:18]")):
                    if off + 1 < ln:
                        va = ra[off] | (ra[off + 1] << 8)
                        vb = rb[off] | (rb[off + 1] << 8)
                        def show(v):
                            if v == 0x7FFF:
                                return f"{v} (0x7fff, no-target sentinel)"
                            if 4144 <= v <= 5632:
                                return f"{v} (inside the advertised travel)"
                            return str(v)
                        print(f"     {label}: {show(va)}  /  {show(vb)}")


    if not last:
        # An image with no start-up snapshots: everything above still applies,
        # and the sections below are about how the boot went, which this dump
        # no longer records.
        summarise_motor(motr, dry)
        return True

    w = last
    print("\n--- what the run did ---")

    def when(v, what):
        print(f"  {what:<34} {'never' if not v else f'{v} ms'}")

    when(w[7],  "body chip select first high")
    when(w[8],  "bus init complete")
    when(w[9],  "first valid frame from the body")
    when(w[10], "message 0x0a answered (handshake)")
    when(w[11], "first status pair sent")

    ids = decode_ids(w[12], w[13])
    print(f"  init-class ids requested           "
          f"{' '.join(f'{i:#04x}' for i in ids) if ids else 'none'}")
    missing = [i for i in HANDSHAKE if i not in ids]
    if missing:
        print(f"  of the A6000 reference set, unseen "
              f"{' '.join(f'{i:#04x}' for i in missing)}")
    extra = [i for i in ids if i not in HANDSHAKE]
    if extra:
        print(f"  asked for, outside that set        "
              f"{' '.join(f'{i:#04x}' for i in extra)}")
    if ids and not last[14] & 1:
        print(f"  >> the body stopped after we answered {ids[-1]:#04x}.")
        print( "     That reply is the first thing to check against the stock")
        print( "     handler for that id.")

    # Format 1 put PM->RCAUSE in w[15]; format 2 puts the focus-channel tally
    # there.  Reading one as the other invents an observation -- a version-1
    # POR (1) renders as "0x04 x1" -- so the format word gates it.
    if w[1] >= 2:
        f1b, f04 = w[15] >> 16, w[15] & 0xFFFF
        verdict = ("body commanded focus via 0x1B" if f1b else
                   "body sent NO 0x1B -- it uses 0x04" if f04 else
                   "no focus command seen on either channel")
        print(f"  focus channel                      0x1B x{f1b}, 0x04 x{f04}")
        print(f"                                     <-- {verdict}")
    else:
        causes = [n for b, n in RCAUSE if w[15] & (1 << b)] or [f"{w[15]:#04x}"]
        print(f"  reset cause                        {', '.join(causes)}")

    summarise_motor(motr, dry)

    if quiet:
        last_q = quiet[max(quiet, key=lambda p: quiet[p][3])]
        print("\n--- what happened at power-off ---")
        print(f"  the body stopped sending at    {last_q[7]} ms")
        print(f"  we were still alive            {last_q[8]} ms later")
        acked = (last_q[12] >> 8) & 1
        print(f"  message 0x16 acknowledged      "
              f"{str(last_q[9]) + ' ms' if acked else 'NO'}")
        print(f"  PA23 driven high               "
              f"{str(last_q[15]) + ' ms' if last_q[15] else 'NO'}")
        # A quiet period is not necessarily the end of the session: the body
        # can go silent mid-run (mode change, sleep) and come back.  If any
        # page carries a LATER timestamp than this one, it did -- and calling
        # that "the body never asks the lens to shut down" is a wrong
        # conclusion drawn from a mid-session pause.  One log showed exactly
        # that: quiet at 49 s, then a park at 117 s.
        # ONLY pages whose w[3] is a timestamp.  TRAC packs (count << 16 |
        # step) there, IDLG packs a length, CHK packs a page count -- scanning
        # all of them reported "a page was written 1572864 ms in", which is
        # page 24's 24 << 16.  A field's meaning is per-tag, not global.
        TIMED = SNAP_TAGS + (QUIET, MOTR, FOCS, 0x46484953)
        later = max((v[3] for v in pages.values()
                     if v[2] in TIMED and v[3] and v[3] > last_q[3]), default=0)
        if later:
            print(f"  -> NOT the end of the session: the body came back, and a")
            print(f"     page was written {later} ms in, {later - last_q[3]} ms later.")
            print(f"     This was a mid-session pause, not a power-off.")
        elif not acked:
            print("  -> this body does not ask the lens to shut down; it just")
            print("     stops talking.  A park has to be triggered off the")
            print("     silence, and the number above is the whole budget.")

    print("\n--- the cutoff ---")
    written = max(snaps)
    # The next page that SHOULD have appeared: earliest scheduled time later
    # than the last one actually written.  Taking the lowest missing page
    # number instead gives times that run backwards, because the pages are not
    # numbered in time order (2 and 3 are the id logs, at 700 ms and 3000 ms).
    due_later = [p for p in SCHEDULE if p not in pages and SCHEDULE[p] > w[3]]
    nxt = min(due_later, key=lambda p: SCHEDULE[p]) if due_later else None
    print(f"  last page written                  page {written} at {w[3]} ms")
    if nxt is None:
        # The two builds use different snapshot schedules, so quote the
        # timestamp actually recorded, not a hardcoded duration.
        print(f"  every scheduled page present       ran to {w[3]} ms")
        print("\n  VERDICT: ran past the 2 s cutoff.  The responder is holding power.")
        return 0

    print(f"  first missing page                 page {nxt}, due at "
          f"{SCHEDULE[nxt]} ms")
    print(f"  so power was cut between           {w[3]} ms and {SCHEDULE[nxt]} ms")
    if SCHEDULE[nxt] <= 2500:
        print("\n  VERDICT: cut at the 2 s mark.  The body treated us as a silent")
        print("           lens -- the handshake did not complete.")
    else:
        print("\n  VERDICT: survived the 2 s cutoff, then stopped later.  That is a")
        print("           different failure: the body accepted us and then dropped")
        print("           us, so look at the status loop, not the handshake.")
    return True


# Trail slots of 2 KB, then the boot tally row.  The layout has grown -- one
# slot, then two, now four -- so the count is inferred from the dump size
# rather than hardcoded, and old dumps keep decoding.
SLOT_SIZE = 0x800


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "dumps/proto.bin"
    blob = open(path, "rb").read()
    slots = max(1, (len(blob) - 0x100) // SLOT_SIZE)

    print(f"{path}: {len(blob)} bytes, "
          f"{slots} trail slot(s) + tally\n")

    for k in range(slots):
        chunk = blob[k * SLOT_SIZE:(k + 1) * SLOT_SIZE]
        decode_trail(chunk, f"=== boot slot {k} (boots " +
                     ", ".join(str(k + n * slots) for n in range(3)) +
                     " ... since flashing) ===")
    summarise_tally(blob, SLOT_SIZE * slots)
    return 0


if __name__ == "__main__":
    sys.exit(main())
