/* emount.h -- minimal E-mount responder.
 *
 * Purpose, in one line: keep the body from cutting power.  A Sony body powers
 * down a lens that does not answer within about two seconds (EA9.md §11a),
 * which makes this the prerequisite for every other hardware experiment, not a
 * feature on top of them.
 */
#ifndef EMOUNT_H
#define EMOUNT_H

#include <stdint.h>

/* The BCD firmware version this adapter reports to the camera, in message
 * 0x07 payload[6].  Stock 1.8.0 sends 0x18.  It reads as BCD in the body's
 * Setup -> Version -> Lens page, which makes it a free telltale: the camera
 * shows the number, so this responder answered message 0x07 and the body
 * believed it.
 *
 * It also identifies WHICH build is on the adapter, which matters while the
 * protocol gaps are being closed one at a time:
 *
 *     0x20 = 2.00   the responder before any command-progress reporting
 *     0x21 = 2.01   message 0x06 payload[25], the command-progress ladder
 *     0x22 = 2.02   message 0x06's event appendix, 0x1D on arrival
 *     0x23 = 2.03   same, plus the normal-class histogram in the trail
 *     0x24 = 2.04   same; the histogram now covers the whole session
 *     0x25 = 2.05   the 23-byte STOP command is obeyed and acknowledged
 *     0x26 = 2.06   message 0x04 read as a RECORD STREAM: every instruction
 *                   the protocol defines, and the status fields that go with
 *                   them
 *     0x27 = 2.07   the record walk bounded by the FRAME, not the window
 *     0x28 = 2.08   same protocol; the trail can now tell a flashing session
 *                   apart from a boot the camera cut short
 *     0x29 = 2.09   the start-up and shut-down pages are gone; their sixteen
 *                   pages carry the transfer-window captures and nine more
 *                   focus moves
 *     0x30 = 3.00   the aperture declared as an aperture VALUE, f/2.0
 *     0x31 = 3.01   message 0x1B answered with our own aperture, and what it
 *                   asked for recorded
 *     0x32 = 3.02   pl[46] back to what the stock sends: equal to pl[44]
 *     0x33 = 3.03   the minimum back to f/90 -- TRIED, changed nothing, and
 *                   start-up hung; reverted
 *     0x34 = 3.04   the minimum back to f/2.0, i.e. 3.02's behaviour
 *     0x35 = 3.05   the aperture message 0x03 asks for, recorded
 *     0x36 = 3.06   message 0x28 payload[9..10] back to the stock's zero --
 *                   FIXED the a9 II re-metering after every exposure
 *     0x37 = 3.07   that field now follows the capability bit in the body's
 *                   message 0x08 request, and the request is captured
 *
 * Not to be confused with the bootloader's own version triplet, which is what
 * ea9flash.py --check reports and lives nowhere in the app image (CLAUDE.md,
 * "There are TWO independent version records"). */
#define EM_FW_VERSION_BCD   0x37u

/* Blocks until the body brings its chip select up and back down, then arms the
 * receiver.  Never returns on a bench supply with no body attached -- the same
 * as the stock firmware's protocol_init at 0x551c. */
void em_init(void);

/* Run pending work: replies to init-class requests and the periodic status
 * pair.  Call it from the main loop; it blocks only for as long as a frame
 * takes to shift out. */
void em_poll(void);

/* Message 0x16 is the body asking the lens to SHUT DOWN, not an autofocus
 * trigger as an earlier reading of this file guessed.  The stock sequence
 * (EA9.md §2.6) is: park, wait for arrival, echo the body's own frame back as
 * an acknowledgement, disable the bus, wait for the frame sync to stop, and
 * halt.  Ignoring it leaves the body waiting -- which is the shutdown hang. */
#define EM_ID_SHUTDOWN   0x16u

/* The first message of every handshake ever observed, on every body:
 * 0x01 -> 0x07 -> ...  So it is the session opener, and the only thing that
 * should bring the lens back after it has acknowledged a shutdown. */
#define EM_ID_SESSION    0x01u

/* The aperture command.  Init-class, an aperture in payload[0..1] on the
 * 256 x AV + 4096 scale, and the reply echoes it back.  A body sends it at the
 * shutter press.  An adapter with no iris has nothing to do with it, so it is
 * counted and answered, never acted on. */
#define EM_ID_APERTURE   0x1Bu

/* The lens ID we report in message 0x07 payload[9..10], u16 LE.
 *
 * Stock is 234 (0x00EA), in the legacy/A-mount range that every measured
 * ADAPTER sits in -- the two Viltrox EF adapters report 78.  Native E-mount
 * lenses are 0x8xxx and third-party natives sit in their own high block; both
 * Yongnuo lenses report 50496 (0xC540) despite being different focal lengths
 * and different formats.
 *
 * EXPERIMENT: the a9 II drives this adapter's focus through message 0x04 and
 * has never sent a 0x1B, while Yongnuo lenses are driven through 0x1B.  Neither
 * capability mask explains it -- the body offers all 64 ids and we advertise
 * 0x1B.  The lens ID is the next discriminator: if the body selects its focus
 * channel by ID space, this changes it.
 *
 * Note pl[0] is a SECOND discriminator on the same theme: 0x01 on all three
 * natives measured, 0x02 on all three adapters.  Left alone so this run has one
 * variable; it is the obvious follow-up if the ID alone changes nothing. */
#define EM_LENS_ID       50496u

extern volatile uint8_t  em_shutdown_acked;  /* 0x16 answered, status loop off */
extern volatile uint32_t em_t_shutdown;      /* ms when it was answered */
extern volatile uint32_t em_t_revived;       /* ms a new session reopened us */
extern volatile uint8_t  em_ignored_after_shutdown;  /* requests left unanswered */

/* Called on message 0x16 BEFORE the acknowledgement goes out, so the lens can
 * park the way the stock does.  Optional: builds with no motor register none.
 *
 * It must return promptly -- the body is waiting for the echo, and the echo is
 * what unblocks its power-off.  A park that stalls or times out must not stop
 * the acknowledgement, so the caller sends it either way. */
void em_set_park_handler(void (*park)(void));

/* Called AFTER the acknowledgement has gone out.  The stock does three more
 * things at this point -- disable the bus, wait for the frame sync to stop,
 * drive PA23 high -- and then halts (EA9.md §2.6).  Whatever of that a build
 * wants to reproduce goes here. */
void em_set_powerdown_handler(void (*fn)(void));

/* Turn the whole message-0x16 shutdown path off, so the responder behaves the
 * way it did before any of it existed: 0x16 is treated as an id with nothing
 * to send, exactly like the 72 ids the stock jump table ignores.
 *
 * This exists as a CONTROL.  The boot count went from one per mount to three
 * at the build that added PA23 and the power-down tail, and every attempt to
 * explain the startup failure from the receiver side has failed.  Holding
 * everything else constant and removing only the shutdown work is the way to
 * find out whether it is responsible (NOTES.md §45). */
void em_set_shutdown_enabled(uint8_t on);

/* Status frames only, no reply dispatch.  This is what a nested em_poll()
 * becomes: the park runs inside em_poll, and its servo loop pumps em_poll
 * again, which would otherwise re-enter the reply path and park recursively. */
void em_pump_status(void);

/* Focus position reported in message 0x06, in protocol units.  The two fields
 * are NOT duplicates: payload[20:22] is where focus is now, payload[2:4] is
 * where it will be one body frame from now.  Pass `ahead == pos` when nothing
 * is moving -- that is what a stationary native lens sends. */
void em_set_focus_position(int32_t pos, int32_t ahead);

/* Message 0x06 payload[0..1]: is focus moving, and which way.
 *
 * `moving` is 0 at rest.  `dir` is 0 stopped, 2 one way, 4 the other.  The
 * near-limit bits are derived from `pos`, so the caller passes the position it
 * has just reported rather than a flag it has to keep in step. */
void em_set_motion(uint8_t moving, uint8_t dir, int32_t pos);

/* Message 0x06 payload[32..38]: push one frame-to-frame position difference
 * into the seven-deep velocity history, newest last. */
void em_push_velocity(int32_t delta);

/* Message 0x05's focus reports: the subject distance as a distance code, the
 * same distance as a coarse signed byte, the coarse position, and whether the
 * lens is in motion (payload[22] bit 6). */
void em_set_subject_distance(uint16_t code, uint8_t coarse, uint8_t coarse_pos);
void em_set_in_motion(uint8_t moving);

/* Message 0x05 payload[4] is the APERTURE settle countdown -- how many more
 * status frames the iris needs to reach the aperture the body commanded
 * (msg_0x05.md).  It has nothing to do with focus.
 *
 * This adapter has no iris, so its aperture is never in transit and the field
 * is a constant 0, which is what every LM-EA9 ever shipped sends and what the
 * stored frame already holds.  Nothing writes it.
 */

/* ------------------------------------------------------------------------
 * Message 0x06 payload[25]: the SCAN progress byte.
 *
 * It belongs to the Scan instruction and to nothing else (autofocus.md 4.3):
 *
 *     0x10  scan accepted, the sweep leg is about to be launched
 *     0x20  approach finished, the sweep is under way
 *     0x30  sweep finished
 *     0x00  idle
 *
 * It stays 0x00 for Move, Stop and Drive.  An earlier build stepped it on
 * every move; that was read off one device's command state machine and is not
 * what the field means.
 *
 * Each value is held for at least one status frame, so a body sampling once
 * per frame cannot miss a step.
 */
#define EM_SCAN_IDLE      0x00u
#define EM_SCAN_ACCEPTED  0x10u
#define EM_SCAN_SWEEPING  0x20u
#define EM_SCAN_DONE      0x30u

/* Latch a value, cancelling any sequence still running. */
void em_set_scan_state(uint8_t state);

/* Walk the tail of the sequence: 0x30 on the next status frame, then 0x00.
 * Driven by the status pump, so the steps are spaced by the body's own frame
 * rate rather than by however long the caller takes. */
void em_scan_finish(void);

/* ------------------------------------------------------------------------
 * Message 0x06's EVENT APPENDIX -- the frame is variable-length.
 *
 * Idle it is 48 bytes.  To report an event the lens grows it to 50 and the two
 * extra bytes carry:
 *
 *     pl[39] = frame[0x2D]   the event code
 *     pl[40] = frame[0x2E]   its parameter, always 0 on this device
 *
 * In the 48-byte form those two offsets ARE the checksum, so the appendix
 * exists only while the frame is 50 bytes long -- there is no way to send an
 * event without growing the frame, and no way to grow the frame without
 * sending one.
 *
 * The stock holds the 50-byte form for exactly ONE transmission: its frame
 * state machine sends message 0x06 at state 5 and puts the length back at the
 * 5 -> 6 step immediately afterwards (`0x52c0`, `0x5274`).  That is not an
 * accident -- the checksum of a 50-byte frame lands on the staged event byte
 * itself, so a second frame sent without re-staging would carry rubbish.
 *
 * This is the channel a lens uses to say "the move you commanded is done".
 */
#define EM06_LEN_IDLE    48u
#define EM06_LEN_EVENT   50u
/* THE HARD CEILING on a 0x06 frame, and it is not a matter of taste.
 *
 * The frame is built in place in the stored packet table, where message 0x06
 * occupies .data+0x000 and packet 0x01 begins at .data+0x038 = 56.  A frame
 * longer than 56 bytes writes its tail over packet 0x01's header -- the
 * capability bitmap, the first thing a body ever asks for -- and the damage
 * would show up as a failed handshake on the NEXT session, nowhere near the
 * tail that caused it.
 *
 * 56 leaves 8 bytes of tail: a 2-byte event and two 3-byte query answers, or
 * a 4-byte Drive event and one answer.  Anything that does not fit waits for
 * the next frame rather than being dropped. */
#define EM06_LEN_MAX     56u

/* Event codes.  The stock stages 0x1D when a one-stage (27-byte) command
 * finishes, 0x1F for a two-stage (36-byte) one, and 0x1C to acknowledge a stop.
 * Only the first is implemented here -- 27 bytes is the form an a9 II sends.
 *
 * Caution: 0x1F is a real code here but is another vendor's "no event"
 * sentinel.  The two do not agree; there is no shared code table. */
#define EM_EVENT_MOVE_DONE  0x1Du
#define EM_EVENT_STOPPED    0x1Cu

/* Stage an event.  It goes out in the tail of the next message 0x06, which
 * grows the frame to carry it, and the frame returns to 48 bytes afterwards.
 *
 * ONE EVENT PER FRAME (autofocus.md 4.2): several waiting are sent one per
 * frame, in order.  Only the LATEST instruction is answered -- an instruction
 * a newer one replaced before it finished produces no event at all -- so the
 * queue is shallow by design; posting past its depth drops the newcomer
 * rather than losing one already promised.
 *
 * `param` is the event's second byte: 0x00 for Move and Scan, 0x01 for Stop,
 * and for Drive the top six bits of the Drive record's rec[1]. */
void em_post_event(uint8_t code, uint8_t param);

/* How many 50-byte frames have actually gone out. */
extern volatile uint16_t em_events_sent;

/* ------------------------------------------------------------------------
 * The STOP command: message 0x04's 23-byte form.
 *
 * The body sends it to call off a move in flight.  A stock lens stops the
 * motor and answers with two events on consecutive frames -- the code for the
 * command it just killed (0x1D for a one-stage move), then 0x1C for the stop.
 *
 * Observed 12 times in one 85 s session on an a9 II, so this is not a
 * theoretical form; it is the third command length that body actually uses.
 */
extern volatile uint16_t em_stop_n;  /* how many Stop records have arrived */

/* THE focal length, in mm x 10 -- the one place it is written down.
 *
 * The stock table is internally inconsistent: message 0x05 says 400 (40.0 mm,
 * inherited from the Canon EF 40mm f/2.8 STM descriptor TECHART cloned) while
 * message 0x28's focal pair says 500 (50.0 mm).  Both are sent, so the adapter
 * tells the body two different focal lengths depending on which message it
 * asks for.
 *
 * The 52.0 mm telltale did its job (NOTES.md §70): a photo came out of the a9
 * II tagged 52 mm, so the body reads its focal length from a field we write and
 * both messages now agree.  Back to 50.0 mm. */
#define EM_FOCAL_MM10 500u

/* Write the focal length into every field that carries it: message 0x05's pair,
 * message 0x28's pair, and message 0x06's focal-derived step size.  Called from
 * em_init; call it again to change focal length at runtime. */
void em_set_focal_length(uint16_t mm10);

/* The defocus scale this lens is currently advertising in message 0x06
 * payload[13..14] -- position counts per defocus unit.  Read back rather than
 * recomputed, so a Move in defocus units uses exactly the number the body was
 * given. */
uint16_t em_defocus_scale(void);

/* THE APERTURE this adapter declares -- maximum, minimum, and the one it is at
 * right now.  Three constants because the protocol carries three fields; one
 * number because an LM-EA9 has no iris.  It carries a manual Leica M lens whose
 * aperture ring it cannot read and cannot move, so the honest report is a fixed
 * aperture: wide open, permanently, and no range for the body to drive.
 *
 * ALL THREE ARE APERTURE VALUES -- the u16 `256 * AV + 4096` scale that
 * message 0x05 payload[0..1] and message 0x28 payload[9..10] carry directly
 * (aperture_value.md).  Read one straight off:
 *
 *     AV = (value - 4096) / 256        F = 2^(AV / 2)
 *     0x1200 = 4608 -> AV = 2 -> F = 2^1 = f/2.0
 *
 * The high byte alone is legible: 0x12 = 18, and 18 - 0x10 = 2 = AV, so the
 * whole stops read off the top byte and the low byte is the fraction of a
 * stop.  f/1.4 is 0x1100, f/2.8 is 0x1300, f/4 is 0x1400.
 *
 * The OTHER encoding, the Canon-convention descriptor of message 0x05
 * pl[44..59], is derived from these by em_aperture_to_descriptor() rather than
 * written out separately -- one number per fact, so the two encodings cannot
 * disagree.  It is the coarser of the two at 1/8 stop, so a value that is not
 * a multiple of 32 above 4096 rounds when it goes there.
 *
 * THE MINIMUM IS THE MAXIMUM.  An adapter with no iris has no range, and the
 * stock's f/90 is a fiction: a body may command anywhere in that range and the
 * transmission is f/2.0 regardless.
 *
 * Restoring f/90 was TRIED and changed nothing: the camera still re-metered
 * after every exposure, and start-up hung as well (NOTES.md 89).  So the
 * fiction buys nothing here, and the honest single point is what stands.
 *
 * pl[52] is therefore the ONE byte of this block that a stock LM-EA9 sends
 * differently.  Every other byte matches one, and the test says so against a
 * literal rather than against values derived from these constants. */
#define EM_APERTURE_MAX  0x1200u        /* f/2.0 -- message 0x05 pl[44], pl[51] */
#define EM_APERTURE_MIN  0x1200u        /* f/2.0 -- message 0x05 pl[52]         */
#define EM_APERTURE_NOW  0x1200u        /* f/2.0 -- 0x05 pl[0..3], 0x28 pl[9..10] */

/* The current aperture, as 256 x AV + 4096, into message 0x05 payload[0..1],
 * its second copy at [2..3], and message 0x1B's reply.
 *
 * NOT into message 0x28 payload[9..10]: a stock adapter leaves that zero and
 * its photographs are still tagged with the right aperture, so the field is
 * not what carries it. */
void em_set_aperture(uint16_t av);

/* The aperture the body last commanded on message 0x1B, and how many of those
 * frames have arrived (em_focus_n).  Recorded rather than obeyed: this device
 * has one aperture and cannot move it, so the only thing worth knowing is
 * whether a body ever asks for a different one. */
extern volatile uint16_t em_aperture_req;

/* ------------------------------------------------------------------------
 * MESSAGE 0x08's REQUEST -- captured whole, and the capability bit in it.
 *
 * The body sends it exactly once, sixth in the init handshake, in every
 * session ever recorded.  So the first one is the only one, and capturing it
 * is the whole of the observation.
 *
 * Bit 7 of its payload[1] decides whether the lens populates payload[9..10] of
 * message 0x28 -- and of message 0x35, which this device does not send.  The
 * bit is inverted: CLEAR means the body wants the field.  A lens latches it
 * when it answers 0x08 and applies it for the rest of the session.
 *
 * Default OFF, which is both the stock's behaviour and the one measured to be
 * correct: filling the field unasked made an a9 II re-meter after every
 * exposure (NOTES.md 92). */
#define EM_M08_LEN 24
extern volatile uint8_t  em_m08[EM_M08_LEN];   /* the raw request, truncated  */
extern volatile uint8_t  em_m08_len;           /* how much of it was captured */
extern volatile uint16_t em_m08_n;             /* how many arrived            */
extern volatile uint8_t  em_m08_aperture_on;   /* the decoded capability bit  */

/* Message 0x03 payload[5..6] is the aperture the body is ASKING for, sent
 * every frame of the normal loop (msg_0x03.md).  Nothing here can act on it --
 * there is no iris -- so it is recorded for the same reason 0x1B's is: to see
 * whether a body ever asks for an aperture this device does not have. */
extern volatile uint16_t em_m03_aperture;
extern volatile uint16_t em_m03_n;

/* The declared range into message 0x05 payload[44..59], both arguments on the
 * aperture-value scale and converted to descriptor bytes here.  The body reads
 * this block: an adapter that sends zeros here is shown as F1.0 and refused
 * autofocus (EA9.md §8). */
void em_set_aperture_range(uint16_t max_av, uint16_t min_av);

/* The two encodings, both ways.  A descriptor byte is 1/8 stop with 0x08 =
 * f/1.0; an aperture value is 256*AV + 4096. */
uint16_t em_aperture_from_descriptor(uint8_t v);
uint8_t  em_aperture_to_descriptor(uint16_t av);

/* The body's "no target" sentinel, on both channels.  Driving to it would
 * command a position far outside any travel a lens advertises. */
#define EM_NO_TARGET     0x7FFFu

/* ------------------------------------------------------------------------
 * FOCUS INSTRUCTIONS -- message 0x04's record stream.
 *
 * Message 0x04 is a 13-byte header followed by tagged records, each a tag byte
 * and a fixed number of operand bytes (autofocus.md 2).  The frame length is a
 * CONSEQUENCE of the records in it, not a code for them: keying on length
 * works only for the one-record forms and cannot see a frame carrying two.
 *
 * The tags this device understands, and their total sizes:
 *
 *     0x1C   1 B   Stop
 *     0x1D   5 B   Move        operand + mode byte
 *     0x1F  14 B   Scan        sweep a window at a chosen speed
 *     0x22   3 B   Query       position -> distance code
 *     0x2E   3 B   Query       distance code -> position
 *     0x2F   3 B   Row index   optical table, not focus
 *     0x34  24 B   unknown
 *     0x3C   8 B   Drive       run toward an end at a given speed
 *     0x4A  12 B   unknown
 *
 * Records carry no length field, so a receiver must know the size of every tag
 * it may meet.  One it does not know ends the walk -- it cannot be skipped,
 * because its size is exactly what is unknown.
 */
#define EM_REC_STOP      0x1Cu
#define EM_REC_MOVE      0x1Du
#define EM_REC_SCAN      0x1Fu
#define EM_REC_Q_POS2D   0x22u
#define EM_REC_Q_D2POS   0x2Eu
#define EM_REC_ROW_INDEX 0x2Fu
#define EM_REC_DRIVE     0x3Cu

/* What the body has asked for.  One instruction at a time: a new one replaces
 * an unread one, which is what the protocol says a new instruction does to a
 * running one (autofocus.md 3.1 rule 4). */
enum em_focus_op {
	EM_OP_NONE = 0,
	EM_OP_MOVE,        /* absolute, target in position units       */
	EM_OP_MOVE_DIST,   /* absolute, target named by distance code  */
	EM_OP_MOVE_REL,    /* relative, delta in position units        */
	EM_OP_MOVE_DEFOCUS,/* relative, delta in defocus units         */
	EM_OP_STOP,
	EM_OP_SCAN,
	EM_OP_DRIVE
};

struct em_focus_cmd {
	uint8_t  op;            /* enum em_focus_op */

	uint16_t target;        /* MOVE: position; MOVE_DIST: distance code */
	int16_t  delta;         /* MOVE_REL, MOVE_DEFOCUS                   */

	/* Scan.  a and b are the window's endpoints in position units, or its
	 * extents either side of centre in the centred form. */
	uint16_t scan_a, scan_b, scan_centre;
	uint8_t  scan_flags;    /* rec[1]: bit 0 centred form, bit 3 nearer end */
	uint8_t  scan_speed;    /* rec[4]: magnitude in the low bits           */

	int16_t  drive_vel;     /* Drive: signed velocity                      */
	uint8_t  drive_param;   /* Drive: rec[1], echoed in the event          */
};

/* Consume the pending instruction.  Returns 0 if none is pending. */
uint8_t em_take_focus_cmd(struct em_focus_cmd *out);

/* ------------------------------------------------------------------------
 * QUERIES.  0x22 and 0x2E ask the lens to convert between its own position
 * units and the distance code.  The answers ride in the tail of the next
 * message 0x06, as 3-byte blocks `tag lo hi`.
 *
 * Queued rather than answered here: the conversion belongs to whatever owns
 * the focus model, and this runs in the chip-select interrupt. */
#define EM_QUERY_Q 4
uint8_t em_take_query(uint8_t *tag, uint16_t *operand);
void    em_post_query_answer(uint8_t tag, uint16_t value);

/* Counters, so a dump can say which instructions a body actually uses. */
extern volatile uint16_t em_rec_n[7];    /* indexed by EM_REC_SLOT_* below */
extern volatile uint16_t em_rec_unknown; /* records whose tag ended the walk */
#define EM_REC_SLOT_STOP    0u
#define EM_REC_SLOT_MOVE    1u
#define EM_REC_SLOT_SCAN    2u
#define EM_REC_SLOT_DRIVE   3u
#define EM_REC_SLOT_Q       4u
#define EM_REC_SLOT_ROW     5u
#define EM_REC_SLOT_OTHER   6u
#define EM_REC_SLOT_N       7u

/* Focus-command observation.  em_focus_target / em_focus_pending carry the
 * target from message 0x04's 27-byte form; em_t04_n counts those frames and
 * em_focus_n counts message 0x1B aperture requests, so a single dump says
 * whether a body ever drove the aperture channel. */
extern volatile uint16_t em_focus_target;
extern volatile uint8_t  em_focus_pending;
extern volatile uint16_t em_focus_n;
extern volatile uint8_t  em_focus_src;

/* The body's own capability bitmap from its message 0x01 request: bit n means
 * it supports id n+1, little-endian.  All zero if it never sent one. */
extern volatile uint8_t  em_body_offer[8];

/* Message 0x04's mode byte, payload[10], as a small histogram.  It reads 0x09
 * while idle and 0x40/0x41/0x01 while focusing; a value seen only in one
 * shooting mode is the only handle we have on "what was the body doing". */
#define EM_MODE_SLOTS 6
extern volatile uint8_t  em_mode_val[EM_MODE_SLOTS];
extern volatile uint16_t em_mode_count[EM_MODE_SLOTS];
extern volatile uint8_t  em_mode_used;

/* The last targets the body sent, including the 0x7FFF no-target sentinel.
 * Only the first few moves get a diagnostic page, so this is what makes an
 * event late in a session -- a power-off, say -- reconstructable. */
#define EM_TGT_RING 10
extern volatile uint16_t em_tgt_ring[EM_TGT_RING];
extern volatile uint16_t em_tgt_ring_n;
extern volatile uint16_t em_t04_target;
extern volatile uint16_t em_t04_n;

/* Diagnostics.  There is no debugger on this part and the adapter only runs
 * on a camera, so these are the whole observation channel: main.c copies
 * them into the flash trail, which ea9flash.py --dump reads back afterwards.
 *
 * The milestones are timestamps rather than flags because "how far did it get"
 * and "when did it stop" are the two questions a two-second power cut leaves
 * behind, and a flag answers only the first. */
extern volatile uint32_t em_frames_rx;      /* frames seen from the body */
extern volatile uint32_t em_frames_tx;      /* frames we have sent */
extern volatile uint8_t  em_handshake_done; /* set once message 0x0a is answered */
extern volatile uint8_t  em_last_id;        /* last init-class id requested */
extern volatile uint8_t  em_last_class;     /* class byte of the last frame */
extern volatile uint8_t  em_bad_frames;     /* frames that failed the 0xF0 test */

extern volatile uint32_t em_t_cs_first_high; /* ms when PA02 first went high */
extern volatile uint32_t em_t_init_done;     /* ms when em_init returned */
extern volatile uint32_t em_t_first_frame;   /* ms of the first valid frame */
extern volatile uint32_t em_t_handshake;     /* ms message 0x0a was answered */
extern volatile uint32_t em_t_first_status;  /* ms of the first status pair */
extern volatile uint32_t em_t_last_frame;    /* ms of the most recent valid frame */

/* The body's frame sync, counted from the moment the EIC is armed.
 *
 * In a failing boot the body does not pulse its chip select for 4.5 s.  These
 * say whether it is otherwise alive during that silence -- a body that is
 * clocking VD but not talking is in a different state from one that is doing
 * nothing at all, and the trail could not tell them apart. */
/* Set when a frame that arrived during bring-up was recovered rather than
 * discarded.  Zero on a healthy boot. */
extern volatile uint8_t  em_init_frame_rescued;

/* em_rx_bytes sampled as the first window opens and again as it closes.
 *
 * "47 bytes arrived" cannot say whether they came from the body inside its
 * window or from a floating line before it -- and those call for opposite
 * fixes.  The difference of these two can. */
extern volatile uint32_t em_rx_bytes_at_open;
extern volatile uint32_t em_rx_bytes_at_close;
extern volatile uint32_t em_first_window_ms;  /* how long the body held it open */

extern volatile uint32_t em_cs_edges;   /* both edges of the body's chip select */
extern volatile uint32_t em_vd_edges;
extern volatile uint32_t em_t_first_vd;

/* The body's frame period, MEASURED from the frame-sync line rather than
 * assumed, and only updated from deltas inside the band below -- a lens that
 * scaled a forecast by a missed edge would hand the body a wildly wrong
 * number.  em_frame_period_n counts the accepted samples, so a dump says
 * whether the period was ever measured at all or is still the default. */
#define EM_FRAME_PERIOD_DEFAULT 17u   /* ~60 Hz, until measured */
#define EM_FRAME_PERIOD_MIN     10u   /* 100 Hz */
#define EM_FRAME_PERIOD_MAX     30u   /* 33 Hz  */
extern volatile uint16_t em_frame_period_ms;
extern volatile uint16_t em_frame_period_n;

/* Which init-class ids the body has asked for: bit n = id n.  The stock
 * handshake is 0x01 0x07 0x0b 0x08 0x09 0x0d 0x10 0x0a, so a mask short of
 * those says exactly where the body gave up. */
extern volatile uint32_t em_id_mask_lo;      /* ids 0x00..0x1f */
extern volatile uint32_t em_id_mask_hi;      /* ids 0x20..0x3f */

/* The ids in the order the body asked for them.  The mask says which; only the
 * order says where a handshake diverged from the reference one, and which of
 * our replies the body reacted badly to -- it is the last entry. */
#define EM_ID_LOG_LEN 48
extern volatile uint8_t  em_id_log[EM_ID_LOG_LEN];
extern volatile uint8_t  em_id_log_n;

/* Normal-class traffic, tallied by (declared length, message id).
 *
 * This is the one measurement that stands between here and the motor.  The
 * stock firmware dispatches focus commands on the frame's LENGTH, not its id
 * (EA9.md §2.2): 27 bytes carries two focus targets, 36 bytes carries one, and
 * 22 bytes is the frame clock.  Neither 27 nor 36 has ever been seen on any
 * body -- the whole captures/ corpus is an A6000, which sends only 22, 29 and
 * 32, none of which the EA9 treats as a command (EA9.md §2.4).
 *
 * So either a modern body does emit them, and this tally will show it, or the
 * EA9 takes focus commands by some route nobody has found. */
#define EM_NORM_SLOTS 14
extern volatile uint8_t  em_norm_len[EM_NORM_SLOTS];    /* frame[1] as declared */
extern volatile uint8_t  em_norm_id[EM_NORM_SLOTS];     /* frame[5] */
extern volatile uint16_t em_norm_count[EM_NORM_SLOTS];
extern volatile uint8_t  em_norm_used;
extern volatile uint8_t  em_norm_overflow;   /* more than EM_NORM_SLOTS kinds */

/* The body transmits a FIXED-SIZE window, not a frame-sized one: every window
 * measured on an a9 II delivers 32 bytes whatever the frame inside declares,
 * and the bytes past the frame are really sent (proved by pre-filling the
 * buffer with EM_RX_POISON -- none of it survived).  So "declared != received"
 * is the normal case and counting it was noise.
 *
 * What is worth counting is a window SHORTER than the frame it carries, which
 * would be genuine truncation, and the set of window sizes actually seen. */
extern volatile uint16_t em_truncated;       /* received < declared */
extern volatile uint8_t  em_window_len[4];   /* distinct window sizes seen */
extern volatile uint8_t  em_window_used;

/* Raw bytes of the first two windows of each distinct (length, id) kind.
 *
 * Counters have taken this as far as they can.  The tally says the body sends
 * 27-byte message 0x04 frames -- the length the stock firmware reads two focus
 * targets out of (EA9.md §2.2) -- and the motor path needs their contents, not
 * their count.  The same capture settles what the length-mismatch counter is
 * actually seeing, which no counter can. */
#define EM_CAP_SLOTS 8    /* pages 20..27; 28+ belong to the motor build */
#define EM_CAP_BYTES 48

struct em_capture {
	uint8_t declared;    /* frame[1] */
	uint8_t got;         /* bytes received in the window */
	uint8_t id;          /* frame[5] */
	uint8_t data[EM_CAP_BYTES];
};

extern volatile struct em_capture em_cap[EM_CAP_SLOTS];
extern volatile uint8_t em_cap_used;

/* Frames received BEFORE the handshake completes, in their own buffer.
 *
 * Separate from em_cap on purpose: the handshake is eleven frames and em_cap
 * has eight slots, so sharing meant a normal boot filled them all and the
 * normal-class capture the proto build exists to do never happened. */
#define EM_PRE_SLOTS 4
extern volatile struct em_capture em_pre[EM_PRE_SLOTS];
extern volatile uint8_t em_pre_used;

#endif /* EMOUNT_H */
