/* emount.c -- minimal E-mount responder.
 *
 * WHAT THIS REPRODUCES, and what it deliberately does not.
 *
 * The stock firmware runs two inbound paths selected by the frame's class byte
 * (EA9.md §2.5): init-class frames go to an id-keyed jump table that answers
 * with a stored packet, normal-class frames go to a length-keyed table that
 * takes focus commands.  On top of that sits a fourteen-state transmit phase
 * machine, a four-value phase interlock, and a deferred-call timer on TC4.
 *
 * This file implements the first path and the status loop, and nothing else:
 *
 *   answered      every init-class request, from the same stored packets
 *   sent          the message 0x05 / 0x06 status pair, on the body's frame sync
 *   NOT handled   focus commands -- the body cannot issue one to this adapter
 *                 on an A6000-class body anyway (EA9.md §2.4), and the motor is
 *                 a separate workstream that is blocked behind this file
 *
 * Three simplifications, each chosen with its risk understood:
 *
 * 1. NO PHASE INTERLOCK.  The stock gate at 0x2000051c[3] opens at reset,
 *    closes when message 0x0a is answered, and reopens on later exchanges
 *    (EA9.md §2.5, CORRECTION 2026-08-17).  Here init-class requests are always
 *    answered.  That is strictly more responsive than stock, never less.  The
 *    camera that §8.2 broke was broken by adding a handler for a message the
 *    body never asked for (0x35) with the stock's interlock removed, so it
 *    transmitted unasked.  Here a reply only ever answers a request, 0x35's
 *    included, so that failure mode is not in reach.
 *
 * 2. NO DEFERRED-CALL TIMER.  The stock 10 us / 100 us / 300 us gaps come from
 *    TC4 one-shots.  The gaps here are busy waits, which removes a timer, an
 *    interrupt and a callback slot.
 *
 * 3. BLOCKING TRANSMIT instead of DMA -- see em_uart.c.
 *
 * The chip-select timing around a frame is the part a body can notice, and is
 * set from native lenses rather than copied: EM_CS_HOLD_US after the last
 * bit, and at least EM_CS_LOW_MIN_US low between any two of our frames.
 */
#include "board.h"
#include "clock.h"
#include "delay.h"
#include "emount.h"
#include "diag.h"
#include "emount_packets.h"
#include "em_uart.h"
#include "em_eic.h"
#include "optical_file.h"

#define EM_CLASS_INIT     0x02u
#define EM_CLASS_NORMAL   0x01u
#define EM_CLASS_FILE     0x03u     /* 0x3A/0x3B, the paged file (msg_0x3B.md) */

#define EM_FRAME_START    0xF0u
#define EM_FRAME_END      0x55u
#define EM_HEADER_LEN     6        /* start, u16 length, class, seq, id */

/* Our chip select around a frame.
 *
 * EM_CS_HOLD_US: high this long after the last bit has left (em_uart_send
 * returns on TXC).  The stock holds 100 us (0x5188's defer(100, 0x5281));
 * native lenses on the A6000 drop it 14-57 us after their last RXD edge, the
 * Viltrox adapter after ~3.
 *
 * EM_CS_LOW_MIN_US: low at least this long between two of our frames,
 * whatever sends them -- the status pair, queued replies sent back to back,
 * a reply followed by the status pair.  Native lenses on the A6000 stay low
 * 470-620 us between frames at the shortest (one gap of 120-150 us per
 * capture).  This replaces the stock's 300 us between 0x05 and 0x06
 * (0x5280's defer(300, msg06_sender)), which covered only that pair.
 *
 * EM_LEAD_US is the defer(10, msg05_sender) the stock's main loop issues
 * after the frame sync. */
#define EM_CS_HOLD_US     20u
#define EM_CS_LOW_MIN_US  500u
#define EM_LEAD_US        10u

/* Frame offsets into the message 0x06 packet that carry the focus position:
 * payload[2..3] the target, payload[20..21] at the frame sync. */
#define EM06_POS_A        0x08
#define EM06_POS_B        0x1A
#define EM06_FORECAST_FRAMES 1   /* pl[2..3] runs this many frames ahead */

/* The travel limits the body clamps every target into: message 0x06
 * payload[7..8] and payload[9..10] = frame offsets 0x0D and 0x0F. */
#define EM06_LIMIT_LO     0x0D
#define EM06_LIMIT_HI     0x0F

/* Message 0x06's event appendix -- frame offsets 0x2D and 0x2E, which are
 * payload[39] and payload[40] in the 50-byte form and the CHECKSUM in the
 * 48-byte one.  The two bytes past the idle frame, .data[0x30] and [0x31],
 * are the gap between message 0x06 and message 0x01 in the stored table; the
 * stock uses the first of them as its staging byte for exactly this. */
#define EM06_EVENT_CODE   0x2D
#define EM06_EVENT_PARAM  0x2E

/* Message 0x06 payload[25] = frame offset 0x1F: the SCAN progress byte,
 * 0x10 -> 0x20 -> 0x30 -> 0x00.  Zero for every other instruction. */
#define EM06_SCAN_STATE   0x1F

/* Message 0x06 payload[23] = frame offset 0x1D: bit 6 flips once per body
 * cycle (message 0x03 pl[10..11]).  Meaning unknown. */
#define EM06_CYCLE        0x1D
#define EM06_CYCLE_BIT    0x40u

/* The current aperture, 256 x AV + 4096: message 0x05 payload[0..1] (fixed,
 * in the template -- there is no iris) and message 0x28 payload[9..10], which
 * follows it only when the body asks (set_28_aperture). */
#define EM05_AP_NOW_A     0x06
#define EM28_AP_NOW       0x0F

/* The focal length pair, mm x 10, u16 LE twice: message 0x05 payload[24..27]
 * and messages 0x28 / 0x35 payload[4..7].  Message 0x08 payload[16..17] holds
 * it in whole mm, BCD.  Written once by put_focal. */
#define EM05_FOCAL        0x1E
#define EM28_FOCAL        0x0A
#define EM08_FOCAL_BCD    0x16

/* Message 0x05 payload[4] = frame offset 10 is the APERTURE settle countdown.
 * Nothing writes it: this adapter has no iris, so its aperture is never in
 * transit and the stored 0x00 is already the right answer (emount.h).
 *
 * payload[8] -- the neighbouring state byte, whose bit 0 means "aperture at
 * rest" -- is left alone for the same reason.  The stored 0x07 has bit 0 set,
 * which is what a lens whose iris is always settled should say.
 */

/* Message 0x06 payload[13..14] = frame offsets 0x13/0x14: the DEFOCUS SCALE,
 * how many position counts one defocus unit is worth (autofocus.md 2.6).  It
 * is what Move mode 6 multiplies its operand by, and it depends on the
 * aperture -- roughly in proportion to the f-number, so it reads as a fixed
 * fraction of the depth of focus.
 *
 * Fixed, in the template (emount.h, em_defocus_scale). */
#define EM06_DEFOCUS      0x13

/* Message 0x06 payload[0] and payload[1]: the motion status and the direction
 * of travel (autofocus.md 4.1).
 *
 *   pl[0]  0x82 at rest, 0x02 moving, OR 0x10 when within NEAR_LIMIT counts
 *          of the lower travel limit and 0x08 when within it of the upper
 *   pl[1]  0x00 stopped, 0x02 one direction, 0x04 the other
 */
#define EM06_MOTION       0x06
#define EM06_DIRECTION    0x07
#define EM06_DIR_MIN      2      /* counts/frame that count as real motion */
#define EM06_AT_REST      0x82u
#define EM06_MOVING       0x02u
#define EM06_NEAR_LO      0x10u
#define EM06_NEAR_HI      0x08u
#define EM06_NEAR_LIMIT   0x1F

/* Message 0x06 payload[32..38] = frame offsets 0x26..0x2C: seven signed bytes,
 * the position trace within the last frame (em_sample_position). */
#define EM06_VEL_HIST     0x26
#define EM06_VEL_HIST_N   7

/* Message 0x05's focus reports (autofocus.md 4.4).  0x06 gives the servo view
 * in the lens's own counts; these give the same state in units the body can
 * use without knowing the lens. */
#define EM05_DISTANCE     0x1A          /* pl[20..21] distance code        */
#define EM05_FLAGS        0x1C          /* pl[22] bit 7 set, bit 6 = moving */
#define EM05_DIST_COARSE  0x1D          /* pl[23] ~ -32 x dioptres          */
#define EM05_POS_COARSE   0x3C          /* pl[54] 0x06 units from infinity / 256 */
#define EM05_MIN_FOCUS    0x22          /* pl[28..29] minimum focus distance */
#define EM05_RING_DIR     0x42          /* pl[60] focus ring direction      */

/* Messages 0x28 / 0x35 payload[0..3] repeat message 0x05 payload[20..23] for
 * the frame the body is exposing: the distance code, a zero byte, the coarse
 * distance (msg_0x35.md; Yongnuo copies them from its status snapshot, and the
 * A7 IV files payload[0..1] as the subject distance of the exposure). */
#define EM28_DISTANCE     0x06          /* pl[0..1] distance code           */
#define EM28_DIST_COARSE  0x09          /* pl[3]                            */
#define EM05_FLAG_REPORTS 0x80u
#define EM05_FLAG_MOVING  0x40u

/* Message 0x05 payload[77..80] = frame offsets 0x53..0x56: the row numbers of
 * slots C and D, then two bytes the body stores but does not act on.
 *
 * The body chooses the rows.  Its message 0x03 carries a 0x2F record, `2F
 * rowC rowD`, sent only if 0x01 declares 0x2F (bit 46).  A Sony lens puts
 * that pair into the 0x05 it sends two cycles later -- 421 of 421 frames
 * over three sessions (msg_0x03.md).  An A7 IV (0x3ab854) files slot C as
 * row pl[77], and slot D as row pl[78] if that differs; rows 0 and 1 are
 * discarded.
 *
 * Slots C and D stay zero: this adapter has no per-row data.  A zero row
 * decodes to a flat zero curve, which is what the Yongnuo 35 DA sends for
 * every requested row.
 *
 * pl[79] is 0x00, as on every device measured.  pl[80] is Yongnuo's
 * 14-entry lookup on pl[78], the table added in the 50 DF release that fixed
 * the A7M5.  NOT USED: the A7 IV only copies pl[79..82] into a log record
 * and never branches on them.  It is sent because Yongnuo and Sony send it,
 * not because any body is known to read it. */
#define EM05_ROW_INDEX    0x53
#define EM05_ROW_TBL_LO   0x07u
static const uint8_t g_row_tbl[14] = {
	0x06, 0x06, 0x0C, 0x0C, 0x12, 0x12, 0x0A, 0x0A,
	0x14, 0x14, 0x1E, 0x1E, 0x32, 0x32,
};

/* Where message 0x03's records begin: 6 frame-header bytes + the 20-byte
 * message header = pl[20], where the 0x2F record sits (msg_0x03.md). */
#define EM03_REC_START    26u

/* Message 0x08 payload[13] = frame offset 0x13: (v & 0xFE) == 0x80 declares
 * the 105-byte message 0x05, anything else the 117-byte one (msg_0x08.md). */
#define EM08_STATUS_FORM  0x13

/* The long message 0x05: the 105-byte frame with twelve more payload bytes
 * before the checksum -- payload 96..107, the rest of the slot C/D record the
 * short form cuts off.  This device has no table to put there, so zeros. */
#define EM05_LEN_LONG     117u

volatile uint32_t em_frames_rx;
volatile uint32_t em_frames_tx;
volatile uint8_t  em_handshake_done;
volatile uint8_t  em_last_id;
volatile uint8_t  em_shutdown_acked;
volatile uint32_t em_t_shutdown;
volatile uint32_t em_t_revived;
volatile uint8_t  em_ignored_after_shutdown;
volatile uint8_t  em_m16_pl0;
volatile uint16_t em_sleep_n;
volatile uint16_t em_wake_n;
volatile uint32_t em_t_wake;
volatile uint8_t  em_wake_first_id;
static volatile uint8_t g_woke;     /* woken, WAKE mark not yet written */
static void (*g_park)(void);
static void (*g_powerdown)(void);
static uint8_t g_shutdown_enabled = 1;
static volatile uint8_t g_in_poll;
volatile uint8_t  em_last_class;
volatile uint16_t em_focus_target;
volatile uint8_t  em_focus_pending;
volatile uint16_t em_focus_n;
volatile uint8_t  em_focus_src;
volatile uint16_t em_events_sent;
volatile uint16_t em_stop_n;
volatile uint16_t em_rec_n[EM_REC_SLOT_N];
volatile uint16_t em_rec_unknown;
volatile uint16_t em_move_mode_n[8];
volatile uint16_t em_move34_n;
volatile uint16_t em_scan_unit_n[2];
volatile uint16_t em_aperture_req;
volatile uint16_t em_m03_aperture;
volatile uint16_t em_m03_n;
volatile uint8_t  em_focus_mf;
static uint8_t    g_mf_run;      /* consecutive 0x81 frames, to EM_MF_FRAMES */
volatile uint16_t em_mf_edges;
volatile uint8_t  em_m08[EM_M08_LEN];
volatile uint8_t  em_m0a[EM_M0A_LEN];
volatile uint8_t  em_m0a_len;
volatile uint16_t em_m0a_n;
volatile uint8_t  em_status05_len = EA9_LEN_05;
volatile uint8_t  em_m08_len;
volatile uint16_t em_m08_n;
volatile uint8_t  em_m08_aperture_on;
static void set_28_aperture(void);

/* The pending instruction, and the query queue.  Both are written from the
 * chip-select interrupt and read from the main loop. */
static volatile struct em_focus_cmd g_cmd;
static volatile uint8_t  g_query_tag[EM_QUERY_Q];
static volatile uint16_t g_query_op[EM_QUERY_Q];
static volatile uint8_t  g_query_n;
volatile uint8_t  em_body_offer[8];
volatile uint8_t  em_mode_val[EM_MODE_SLOTS];
volatile uint16_t em_mode_count[EM_MODE_SLOTS];
volatile uint8_t  em_mode_used;
volatile uint16_t em_tgt_ring[EM_TGT_RING];
volatile uint16_t em_tgt_ring_n;
volatile uint16_t em_t04_target;
volatile uint16_t em_t04_n;
volatile uint8_t  em_bad_frames;

volatile uint32_t em_t_cs_first_high;
volatile uint32_t em_t_init_done;
volatile uint8_t  em_init_frame_rescued;
volatile uint32_t em_t_first_frame;
volatile uint32_t em_t_handshake;
volatile uint32_t em_t_first_status;
volatile uint32_t em_t_last_frame;
volatile uint32_t em_rx_bytes_at_close;
volatile uint32_t em_first_window_ms;
volatile uint32_t em_cs_edges;
volatile uint32_t em_vd_edges;
volatile uint32_t em_t_first_vd;
volatile uint16_t em_frame_period_ms = EM_FRAME_PERIOD_DEFAULT;
volatile uint16_t em_frame_period_n;
static volatile uint32_t g_t_prev_vd;
static volatile uint32_t g_t_vd_us;    /* micros() at the last frame sync */
static volatile uint32_t g_t_vd_prev_us;  /* ... and at the one before */
volatile uint32_t em_id_mask_lo;
volatile uint32_t em_id_mask_hi;
volatile uint8_t  em_id_log[EM_ID_LOG_LEN];
volatile uint8_t  em_id_log_n;
volatile uint8_t  em_norm_len[EM_NORM_SLOTS];
volatile uint8_t  em_norm_id[EM_NORM_SLOTS];
volatile uint16_t em_norm_count[EM_NORM_SLOTS];
volatile uint8_t  em_norm_used;
volatile uint8_t  em_norm_overflow;
volatile struct em_capture em_cap[EM_CAP_SLOTS];
volatile uint8_t  em_cap_used;
volatile struct em_capture em_pre[EM_PRE_SLOTS];
volatile uint8_t  em_pre_used;

/* The row index.  g_row_req is the body's latest request, written from the
 * chip-select interrupt; g_row_next is what the next message 0x05 carries,
 * latched from g_row_req at each status send -- which is what puts the two
 * cycles between a request and its answer.  Both start at 00 00, the
 * "nothing new" index a Sony lens opens a session with. */
static volatile uint8_t  g_row_req[2];
static uint8_t           g_row_next[2];
volatile uint16_t        em_row_req_n;

static volatile uint8_t  g_status_due;

/* The focus motor's state for message 0x06 payload[0..1] (em_set_motor):
 * 0 idle, +1 running toward higher positions, -1 toward lower.  g_motor_vd
 * is its value at the last frame sync. */
static volatile int8_t g_motor;
static volatile int8_t g_motor_vd;
static volatile uint8_t  g_reply_seq;

/* THE REQUEST QUEUE.  Every init-class request is queued whole as it is
 * parsed (in the chip-select interrupt) and answered in arrival order from
 * em_poll.  One slot used to do this, and a second request landing before the
 * first was answered overwrote it -- the first then never got a reply, which
 * a body treats as SYNC TIMEOUT.  An A7 IV can have two requests out at once
 * in a session: its async channel (0x3ae4fc) does not wait for the sync one.
 *
 * Two replies are built from the request's bytes: 0x0A echoes it whole, 0x34
 * copies two of them.  So each entry carries the request, up to 32 bytes --
 * the longest, the 25-byte 0x0A, fits.  g_req is the one being answered. */
#define EM_REQ_SNAP 32
#define EM_REQ_Q    8               /* a power of 2 */
static volatile uint8_t  g_rq[EM_REQ_Q][EM_REQ_SNAP];
static volatile uint8_t  g_rq_head;   /* written by the interrupt */
static volatile uint8_t  g_rq_tail;   /* written by em_poll */
volatile uint16_t        em_req_dropped;
volatile uint8_t         em_req_depth_max;
static uint8_t           g_req[EM_REQ_SNAP];

/* Queue one init-class request.  Interrupt context. */
static void req_push(const uint8_t *rx, uint16_t n)
{
	uint8_t head  = g_rq_head;
	uint8_t depth = (uint8_t)((head - g_rq_tail) & (EM_REQ_Q - 1u));
	uint16_t i;

	if (depth == EM_REQ_Q - 1u) {
		if (em_req_dropped != 0xFFFFu) {
			em_req_dropped++;
		}
		return;
	}
	for (i = 0; i < EM_REQ_SNAP; i++) {
		g_rq[head][i] = (i < n) ? rx[i] : 0;
	}
	g_rq_head = (uint8_t)((head + 1u) & (EM_REQ_Q - 1u));
	if (depth + 1u > em_req_depth_max) {
		em_req_depth_max = (uint8_t)(depth + 1u);
	}
}

/* Take the oldest into g_req; returns its id, or -1 if there is none. */
static int req_pop(void)
{
	uint8_t tail = g_rq_tail;
	uint16_t i;

	if (tail == g_rq_head) {
		return -1;
	}
	for (i = 0; i < EM_REQ_SNAP; i++) {
		g_req[i] = g_rq[tail][i];
	}
	g_rq_tail = (uint8_t)((tail + 1u) & (EM_REQ_Q - 1u));
	return g_req[5];
}

static void req_clear(void)
{
	g_rq_tail = g_rq_head;
}

static uint16_t clamp_travel(int32_t pos);
static void focus_cmd_set(const struct em_focus_cmd *c);
static void count16(volatile uint16_t *n);
static uint16_t g_pos_now;        /* last reported position */

/* MESSAGE 0x34: move to a position within a time limit (msg_0x34.md), as
 * Yongnuo 50 DF does it (FUN_0801620c).  The body sends it from its release
 * sequence, as a frame of its own:
 *
 *   pl[0..1]  target, position units
 *   pl[3]     bits 0-1 mode; other bits flags
 *   pl[4]     milliseconds to the body's deadline
 *
 * A target inside the travel is a position whatever the mode.  The move is
 * capped at what fits the time: Yongnuo takes pl[4] less 13 ms when it is
 * above 13, at 18 units per millisecond -- some three quarters of its top
 * speed.  Here the rate is set by the caller from this mechanism's cruise
 * speed (em_set_move34_speed).  A target outside the travel under a non-zero
 * mode is a distance code or a table value that Yongnuo converts; this does
 * not, and stays where it is.  The body's no-target 0x7FFF never moves it.
 * The move is SILENT: the reply is its answer, so no 1D event follows.
 *
 * Returns the position the lens is now going to -- the reply's pl[1..2]. */
static uint32_t g_move34_ups = 4687u;   /* 0.75 x 25 counts/ms, in units/s */

void em_set_move34_speed(uint32_t ups)
{
	g_move34_ups = ups;
}

static uint16_t timed_move34(void)
{
	uint16_t t    = (uint16_t)g_req[6] | ((uint16_t)g_req[7] << 8);
	uint8_t  mode = g_req[9];
	uint32_t ms   = g_req[10] > 13u ? g_req[10] - 13u : g_req[10];
	int32_t  here = g_pos_now;
	int32_t  step = (int32_t)(g_move34_ups * ms / 1000u);
	int32_t  to;
	struct em_focus_cmd c = { 0 };

	if (t == EM_NO_TARGET
	    || ((mode & 0x07u) && clamp_travel(t) != t)) {
		return (uint16_t)here;
	}
	to = clamp_travel(t);
	if (to - here > step) {
		to = here + step;
	} else if (here - to > step) {
		to = here - step;
	}
	if (to != here) {
		c.op     = EM_OP_MOVE;
		c.target = (uint16_t)to;
		c.silent = 1;
		focus_cmd_set(&c);
	}
	count16(&em_move34_n);
	return (uint16_t)to;
}

static const struct ea9_packet *find_packet(uint8_t id)
{
	for (unsigned i = 0; i < EA9_PACKET_COUNT; i++) {
		if (ea9_packets[i].id == id) {
			return &ea9_packets[i];
		}
	}
	return 0;
}

/* transmit(), 0x51d4: stamp the length, the terminator and a fresh checksum,
 * then shift the frame out with our chip select asserted.
 *
 * The checksum being recomputed on send is why the stored packets can be
 * edited freely -- six of the seventeen stored checksums are stale in every
 * shipped image and the adapter still works (CLAUDE.md, "Checksum -- SOLVED").
 */
static uint32_t g_cs_low_us;      /* micros() when our chip select last fell */

static void em_transmit(uint8_t *frame, uint16_t len)
{
	uint16_t sum  = 0;
	uint32_t low;

	frame[1]       = (uint8_t)len;
	frame[2]       = (uint8_t)(len >> 8);      /* 0x3B is 500 bytes */
	frame[len - 1] = EM_FRAME_END;

	/* sum over frame[1] .. frame[len-4], i.e. everything but the start
	 * byte, the checksum itself and the terminator. */
	for (uint16_t i = 1; i <= (uint16_t)(len - 4); i++) {
		sum += frame[i];
	}
	frame[len - 3] = (uint8_t)(sum & 0xFFu);
	frame[len - 2] = (uint8_t)(sum >> 8);

	/* The frame is ready; wait out what is left of the low time.  Time
	 * spent building it, and in interrupts, already counts. */
	low = micros() - g_cs_low_us;
	if (low < EM_CS_LOW_MIN_US) {
		delay_us(EM_CS_LOW_MIN_US - low);
	}

	PORT->Group[0].OUTSET.reg = (1u << EM_PIN_LENS_CS);
	em_uart_send(frame, len);

	delay_us(EM_CS_HOLD_US);
	PORT->Group[0].OUTCLR.reg = (1u << EM_PIN_LENS_CS);
	g_cs_low_us = micros();

	em_frames_tx++;
}

/* The shutdown acknowledgement: a fixed 10-byte frame, class 0x02, seq 0,
 * id 0x16, payload 00 -- Yongnuo's stored reply, byte for byte.
 *
 * The stock echoes the request instead.  Either works because the A7 IV only
 * looks for a reply with id 0x16 and reads nothing in it (msg_0x16.md), and a
 * fixed frame cannot carry anything of the request's back to the body.
 *
 * What releases the body is not this frame alone: after it, the body cuts
 * the motor supply, switches its own UART off and polls RXD every 5 ms until
 * it reads low.  The powerdown handler (main.c) is what gets RXD there.
 */
void em_set_park_handler(void (*park)(void))
{
	g_park = park;
}

void em_set_powerdown_handler(void (*fn)(void))
{
	g_powerdown = fn;
}

void em_set_shutdown_enabled(uint8_t on)
{
	g_shutdown_enabled = on;
}

static void em_shutdown_ack(void)
{
	uint8_t ack[EM_HEADER_LEN + 4] = {
		EM_FRAME_START, 0, 0, EM_CLASS_INIT, 0x00, EM_ID_SHUTDOWN, 0x00,
	};
	uint8_t already = em_shutdown_acked;

	/* Park first, acknowledge second -- the order the stock uses, and the
	 * only one that works: the body cuts the motor supply as soon as it
	 * has the reply.  A repeated 0x16 gets its reply -- the body may
	 * simply not have heard the first -- but the park and the power-down
	 * signal happen once. */
	if (!em_shutdown_acked && g_park) {
		g_park();
	}

	/* Unconditionally, even if the park stalled: a lens stuck mid-travel
	 * is a far smaller problem than a camera that will not switch off. */
	em_transmit(ack, sizeof(ack));

	em_shutdown_acked = 1;
	em_t_shutdown     = millis();

	if (!already && g_powerdown) {
		g_powerdown();
	}
}

/* The baud switch after the 0x0C reply is a chip-select handshake, not just a
 * register write.  Every A6000 capture (SELP1650, SEL55210, Viltrox) shows,
 * after the lens's 0x0C reply:
 *
 *   body CS  up  ~110-130 us after the lens CS dropped, no data
 *   lens CS  up   4-23 us later          -- "switching"
 *   body CS  down ~60-70 us after it rose
 *   lens CS  down 190-430 us after that  -- "ready at 1.5 Mbaud"
 *
 * Yongnuo does the same in code: wait for body CS high, raise its own CS,
 * reconfigure the UART, wait for body CS low, drop its CS.  The first 1.5 Mbaud
 * build switched without this pulse and an a9 II did not complete the
 * handshake. */
static void em_wait_body_cs(int level)
{
	uint32_t t;

	for (t = 0; t < EM_0C_WAIT_US; t++) {
		if (em_pin_level(EM_PIN_BODY_CS) == level) {
			return;
		}
		delay_us(1);
	}
}

static void em_baud_switch(void)
{
	/* A wait that times out is not fatal: the body follows the 01 either
	 * way, so the switch happens regardless. */
	em_wait_body_cs(1);
	PORT->Group[0].OUTSET.reg = (1u << EM_PIN_LENS_CS);
	em_uart_set_baud(EM_BAUD_FAST);
	em_wait_body_cs(0);
	delay_us(EM_0C_READY_US);
	PORT->Group[0].OUTCLR.reg = (1u << EM_PIN_LENS_CS);
}

static uint8_t          g_ready = 1;
static uint8_t          g_m10_owed;
volatile uint8_t        em_reset_wanted;

void em_hold_ready(void)
{
	g_ready = 0;
}

void em_set_ready(void)
{
	g_ready = 1;
}

static void em_send_id(uint8_t id)
{
	const struct ea9_packet *p = find_packet(id);
	uint8_t *frame;

	if (id == EM_ID_SHUTDOWN) {
		if (g_shutdown_enabled) {
			em_shutdown_ack();
		}
		return;
	}

	/* After acknowledging a shutdown, stay silent.
	 *
	 * The first version cleared the flag on ANY init-class request, which
	 * meant the lens could announce "done" and then start sending status
	 * frames again the moment the body said anything -- something the stock
	 * cannot do, because its bus is switched off by then.
	 *
	 * Only a fresh session opener revives us.  Message 0x01 is the first
	 * message of every handshake on every body in the corpus, so it is the
	 * one unambiguous "we are starting over".  Answering anything else
	 * would be answering the tail of a conversation that has ended.
	 *
	 * This is the reversible version of the stock's UART disable: the same
	 * silence from the body's point of view, but a lens that can still hear
	 * and can still be woken.
	 *
	 * Only reached with no power-down handler (the host tests).  The
	 * firmware's handler ends in em_sleep_until_wake, which clears the flag
	 * itself before the body's short wake init -- 0x0B first, no 0x01. */
	if (em_shutdown_acked) {
		if (id != EM_ID_SESSION) {
			em_ignored_after_shutdown++;
			return;
		}
		em_shutdown_acked = 0;
		em_t_revived      = millis();
	}

	/* Message 0x16 has a handler but no frame: it is a pure state poke that
	 * arms the body's AF request (0x59ae).  Anything else unknown is simply
	 * not ours to answer -- the stock jump table sends 72 of its 91 ids to
	 * the ignore exit. */
	/* Message 0x0A: echo the request exactly, as every lens measured does
	 * (msg_0x0A.md).  What it asked for -- the message 0x05 length -- was
	 * read when it arrived. */
	if (id == 0x0Au) {
		uint8_t  echo[EM_REQ_SNAP];
		uint16_t n = g_req[1];
		uint16_t i;

		if (n >= EM_HEADER_LEN + 3u && n <= EM_REQ_SNAP) {
			for (i = 0; i < n; i++) {
				echo[i] = g_req[i];
			}
			em_transmit(echo, n);
		}
		return;
	}

	/* Message 0x10: ready, or not yet (em_hold_ready, emount.h). */
	if (id == EM_ID_READY) {
		if (!g_ready) {
			g_m10_owed      = 1;
			em_reset_wanted = 1;
			return;
		}
		em_msg_10[6] = 0x00u;
		em_transmit(em_msg_10, EA9_LEN_10);
		return;
	}

	if (!p) {
		return;
	}
	frame = p->frame;

	/* The one reply with request-dependent content: message 0x34, the
	 * timed move, answers with where the lens is now going. */
	if (id == 0x34u) {
		uint16_t to = timed_move34();

		frame[6] = 0x00u;
		frame[7] = (uint8_t)to;
		frame[8] = (uint8_t)(to >> 8);
	}

	/* Message 0x0C: the template answers 01, and the switch to 1.5 Mbaud
	 * follows the reply.  Nothing switches back: a standby wake keeps the
	 * speed (em_sleep_until_wake), and a real power-off reboots at 750. */
	if (id == EM_ID_BAUD) {
		em_transmit(frame, p->len);
		em_baud_switch();
		return;
	}

	em_transmit(frame, p->len);
}

/* MESSAGES 0x3A AND 0x3B: the paged optical file, as Yongnuo 50 DF serves it
 * (FUN_080175e8, reached from its receive dispatcher on class 0x03 only).
 * The body selects a file with 0x3A pl[0] and reads it with 0x3B, one page
 * per request; the content is the DF's, cut to the f/1.8 entry and with its
 * check word recomputed (optical_file.c) -- 296 bytes, one page.
 *
 *   0x3A pl[0] = 00   select the LF file: reply 32 bytes, seq 0, pl[1..2] =
 *                     size + 16, pl[5..6] = pages = (size + 16) / 488 + 1
 *   0x3A pl[0] = 08   select the one-page object: reply pl[1] = 3a, pl[5] = 1
 *   0x3A otherwise    clear the selection, no reply
 *   0x3B pl[0] = n    page n (1-based, clamped to the page count): reply 500
 *                     bytes, pl[0..2] = 00 n <request pl[1]>, pl[3..] = the
 *                     next 488 bytes, zero past the end; nothing selected,
 *                     all zero.  The one-page object is a whole frame and is
 *                     sent whole, with its header rewritten.
 *
 * The request's two bytes are taken when it arrives (em_frame_in): this class
 * is not queued with the init requests, and arrives between the 60 Hz frames. */
#define EM_ID_FILE_SIZE   0x3Au
#define EM_ID_FILE_PAGE   0x3Bu
#define EM_3A_LEN         32u
#define EM_3B_LEN         500u
#define EM_3B_DATA        9u        /* frame offset of pl[3] */
#define EM_3B_CHUNK       488u

static volatile uint8_t g_file_pending;
static volatile uint8_t g_file_id, g_file_p0, g_file_p1;

static const uint8_t *g_file_ptr;   /* the selected object, or none */
static uint16_t g_file_size;
static uint16_t g_file_pages;
static uint16_t g_file_chunk;       /* 488, or 500 for a whole frame */

static uint8_t g_file_frame[EM_3B_LEN];

static void em_send_file(uint8_t id, uint8_t p0, uint8_t p1)
{
	uint8_t *f = g_file_frame;
	uint16_t k;

	for (k = 0; k < EM_3B_LEN; k++) {
		f[k] = 0;
	}

	if (id == EM_ID_FILE_SIZE) {
		if (p0 == 0x00u) {
			uint16_t total;

			g_file_ptr   = optical_file;
			g_file_size  = (uint16_t)(optical_file[4]
			                          | (optical_file[5] << 8)) + 16u;
			g_file_chunk = EM_3B_CHUNK;
			total        = (uint16_t)(g_file_size + 16u);
			g_file_pages = (uint16_t)(total / EM_3B_CHUNK + 1u);
			f[0] = EM_FRAME_START; f[3] = EM_CLASS_FILE;
			f[4] = 0x00u;            /* Yongnuo writes seq 0 here */
			f[5] = EM_ID_FILE_SIZE;
			f[7] = (uint8_t)total;  f[8]  = (uint8_t)(total >> 8);
			f[11] = (uint8_t)g_file_pages;
			f[12] = (uint8_t)(g_file_pages >> 8);
			em_transmit(f, EM_3A_LEN);
		} else if (p0 == 0x08u) {
			f[0] = EM_FRAME_START; f[3] = EM_CLASS_FILE;
			f[4] = g_reply_seq;
			f[5] = EM_ID_FILE_SIZE;
			f[7] = EM_ID_FILE_SIZE; f[11] = 0x01u;
			em_transmit(f, EM_3A_LEN);
			g_file_ptr   = optical_page08;
			g_file_size  = OPTICAL_PAGE08_LEN;
			g_file_pages = 1u;
			g_file_chunk = OPTICAL_PAGE08_LEN;
		} else {
			g_file_ptr   = 0;
			g_file_size  = 0;
			g_file_pages = 0;
			g_file_chunk = 0;
		}
		return;
	}

	if (g_file_ptr) {
		uint16_t idx = p0 ? (uint16_t)(p0 - 1u) : 0u;
		const uint8_t *src;

		if (idx >= g_file_pages) {
			idx = (uint16_t)(g_file_pages - 1u);
		}
		src = g_file_ptr + (uint32_t)g_file_chunk * idx;
		if (g_file_chunk == EM_3B_LEN) {
			for (k = 0; k < EM_3B_LEN; k++) {
				f[k] = src[k];
			}
		} else {
			int32_t  left = (int32_t)g_file_size
			                - (int32_t)g_file_chunk * idx;
			uint16_t n    = g_file_chunk;

			if (left < (int32_t)n) {
				n = left > 0 ? (uint16_t)left : 0u;
			}
			for (k = 0; k < n; k++) {
				f[EM_3B_DATA + k] = src[k];
			}
		}
	}
	f[0] = EM_FRAME_START; f[3] = EM_CLASS_FILE;
	f[4] = g_reply_seq;
	f[5] = EM_ID_FILE_PAGE;
	f[6] = 0x00u;
	f[7] = p0;
	f[8] = p1;
	em_transmit(f, EM_3B_LEN);
}

/* THE FOCUS INSTRUCTIONS, raw, into the capture slots: the first message
 * 0x04 of each distinct kind -- frame length, first record tag, and for a
 * Move its mode byte -- so a dump shows exactly which instruction and mode
 * the body uses.  Bare 22-byte frames carry no record and are skipped. */
static void em_cap_command(const uint8_t *rx, uint16_t n)
{
	uint8_t tag  = rx[19];
	uint8_t mode = (tag == 0x1Du && n > 23u) ? rx[23] : 0u;
	volatile struct em_capture *c;
	uint8_t j;
	uint16_t k;

	if (em_cap_used >= EM_CAP_SLOTS || rx[5] != 0x04u || rx[1] <= 22u) {
		return;
	}
	for (j = 0; j < em_cap_used; j++) {
		c = &em_cap[j];
		if (c->declared == rx[1] && c->data[19] == tag
		    && (tag != 0x1Du || c->data[23] == mode)) {
			return;
		}
	}
	c = &em_cap[em_cap_used];
	c->declared = rx[1];
	c->got      = (uint8_t)n;
	c->id       = rx[5];
	for (k = 0; k < EM_CAP_BYTES; k++) {
		c->data[k] = (k < n) ? rx[k] : 0;
	}
	em_cap_used++;
}

/* Where message 0x04's records begin: 6 frame-header bytes + the 13-byte
 * message header = pl[13]. */
#define EM04_REC_START   19u

/* Total size of a record, tag byte included, or 0 for a tag whose size is not
 * known.  A record carries no length field, so an unknown tag cannot be
 * skipped -- its size is precisely what is unknown -- and the walk has to
 * stop there rather than resynchronise on a byte that might be an operand. */
static uint8_t record_size(uint8_t tag)
{
	switch (tag) {
	case EM_REC_STOP:      return 1u;
	case EM_REC_MOVE:      return 5u;
	case EM_REC_SCAN:      return 14u;
	case EM_REC_Q_POS2D:   return 3u;
	case EM_REC_Q_D2POS:   return 3u;
	case EM_REC_ROW_INDEX: return 3u;
	case 0x34u:            return 24u;
	case EM_REC_DRIVE:     return 8u;
	case 0x4Au:            return 12u;
	default:               return 0u;
	}
}

static void rec_count(uint8_t slot)
{
	if (slot < EM_REC_SLOT_N && em_rec_n[slot] != 0xFFFFu) {
		em_rec_n[slot]++;
	}
}

static void count16(volatile uint16_t *n)
{
	if (*n != 0xFFFFu) {
		(*n)++;
	}
}

/* Publish one instruction.  A newer instruction REPLACES an unread one: the
 * protocol says a new one replaces a running one, so it certainly replaces one
 * that has not started (autofocus.md 3.1 rule 4). */
static void focus_cmd_set(const struct em_focus_cmd *c)
{
	g_cmd = *c;
	em_focus_pending = 1;
}

/* A 0x2F record: the body asks for a row of the optical table. */
static void row_request(const uint8_t *r)
{
	g_row_req[0] = r[1];
	g_row_req[1] = r[2];
	count16(&em_row_req_n);
}

/* Message 0x03's records.  Same encoding as message 0x04's, but the only
 * record seen in a 0x03 is the row index, so that is the only one acted on:
 * a Move or a Stop read out of a frame that has never carried one would drive
 * the motor on a guess.  Anything else ends the walk. */
static void em_parse_03_records(const uint8_t *rx, uint16_t end)
{
	uint16_t i = EM03_REC_START;

	while (i < end) {
		uint8_t size = record_size(rx[i]);

		if (size == 0u || (uint16_t)(i + size) > end) {
			return;
		}
		if (rx[i] == EM_REC_ROW_INDEX) {
			row_request(&rx[i]);
		}
		i += size;
	}
}

static void em_parse_records(const uint8_t *rx, uint16_t end)
{
	uint16_t i = EM04_REC_START;

	while (i < end) {
		uint8_t  tag  = rx[i];
		uint8_t  size = record_size(tag);
		const uint8_t *r = &rx[i];
		struct em_focus_cmd c;
		uint8_t  k;

		if (size == 0u || (uint16_t)(i + size) > end) {
			/* Either a tag whose size is unknown, or a record that
			 * runs past the end of the frame.  Both mean the rest
			 * of this payload cannot be read; count it and stop. */
			if (em_rec_unknown != 0xFFFFu) {
				em_rec_unknown++;
			}
			return;
		}
		i += size;

		for (k = 0; k < sizeof(c); k++) {
			((uint8_t *)&c)[k] = 0;
		}

		switch (tag) {
		case EM_REC_STOP:
			rec_count(EM_REC_SLOT_STOP);
			if (em_stop_n != 0xFFFFu) {
				em_stop_n++;
			}
			c.op = EM_OP_STOP;
			focus_cmd_set(&c);
			break;

		case EM_REC_MOVE: {
			/* rec[1..2] operand, rec[3] unknown, rec[4] mode.
			 * Bits 0-2 of the mode select; bit 3 is ignored. */
			uint16_t v = (uint16_t)r[1] | ((uint16_t)r[2] << 8);

			rec_count(EM_REC_SLOT_MOVE);
			em_tgt_ring[em_tgt_ring_n % EM_TGT_RING] = v;
			em_tgt_ring_n++;
			em_t04_target = v;
			if (em_t04_n != 0xFFFFu) {
				em_t04_n++;
			}

			count16(&em_move_mode_n[r[4] & 0x07u]);
			switch (r[4] & 0x07u) {
			case 0:
				/* 0x7FFF is the body's NO-TARGET sentinel.
				 *
				 * The protocol leaves the reading open -- it
				 * lies far above any lens's range, so a lens
				 * that clamps drives to its upper limit either
				 * way (autofocus.md 3.1).  This device does not
				 * clamp it: driving the helicoid to its close
				 * stop because the body had nothing to ask for
				 * is a full-travel slam on every idle frame
				 * that carries one, and an a9 II does send
				 * them. */
				if (v == EM_NO_TARGET) {
					c.op = EM_OP_NONE;
					break;
				}
				c.op     = EM_OP_MOVE;
				c.target = v;
				break;
			case 3:
				c.op     = EM_OP_MOVE_DIST;
				c.target = v;
				break;
			/* Modes 4 and 6: a zero delta is ignored, as
			 * Yongnuo does -- taken as a move to here, it
			 * would stop a leg that is still running. */
			case 4:
				c.op    = v ? EM_OP_MOVE_REL : EM_OP_NONE;
				c.delta = (int16_t)v;
				break;
			case 6:
				c.op    = v ? EM_OP_MOVE_DEFOCUS : EM_OP_NONE;
				c.delta = (int16_t)v;
				break;
			default:
				c.op = EM_OP_NONE;     /* ignored, as specified */
				break;
			}
			if (c.op != EM_OP_NONE) {
				focus_cmd_set(&c);
			}
			break;
		}

		case EM_REC_SCAN:
			/* rec[1] flags, rec[4] speed, rec[5..6] endpoint A,
			 * rec[7..8] endpoint B, rec[12..13] centre. */
			rec_count(EM_REC_SLOT_SCAN);
			count16(&em_scan_unit_n[(r[4] & 0x80u) ? 1 : 0]);
			c.op          = EM_OP_SCAN;
			c.scan_flags  = r[1];
			c.scan_speed  = r[4];
			c.scan_a      = (uint16_t)r[5] | ((uint16_t)r[6] << 8);
			c.scan_b      = (uint16_t)r[7] | ((uint16_t)r[8] << 8);
			c.scan_centre = (uint16_t)r[12] | ((uint16_t)r[13] << 8);
			focus_cmd_set(&c);
			break;

		case EM_REC_DRIVE:
			/* rec[1] parameter, rec[2..3] signed velocity. */
			rec_count(EM_REC_SLOT_DRIVE);
			c.op          = EM_OP_DRIVE;
			c.drive_param = r[1];
			c.drive_vel   = (int16_t)((uint16_t)r[2]
			                          | ((uint16_t)r[3] << 8));
			focus_cmd_set(&c);
			break;

		case EM_REC_Q_POS2D:
		case EM_REC_Q_D2POS:
			rec_count(EM_REC_SLOT_Q);
			if (g_query_n < EM_QUERY_Q) {
				g_query_tag[g_query_n] = tag;
				g_query_op[g_query_n] =
				        (uint16_t)r[1] | ((uint16_t)r[2] << 8);
				g_query_n++;
			}
			break;

		case EM_REC_ROW_INDEX:
			/* The optical table's row selector.  Never seen in a
			 * 0x04, but Yongnuo walks 0x03 and 0x04 with one
			 * parser, so it is honoured here too. */
			rec_count(EM_REC_SLOT_ROW);
			row_request(r);
			break;

		default:
			rec_count(EM_REC_SLOT_OTHER);
			break;
		}
	}
}

/* One whole frame from the body, checksum verified: rx[0] is its 0xF0 and n
 * its declared length.  Chip-select interrupt context. */
static void em_frame_in(const uint8_t *rx, uint16_t n)
{
	em_frames_rx++;
	em_last_class = rx[3];

	/* Until the handshake completes, keep every frame.  A boot whose
	 * handshake never completes is exactly the one whose frames nobody has
	 * ever seen, and its motor pages stay unused, so there is somewhere to
	 * put them. */
	if (!em_handshake_done && em_pre_used < EM_PRE_SLOTS) {
		volatile struct em_capture *c = &em_pre[em_pre_used];
		uint16_t k;

		c->declared = rx[1];
		c->got      = (uint8_t)n;
		c->id       = rx[5];
		for (k = 0; k < EM_CAP_BYTES; k++) {
			c->data[k] = (k < n) ? rx[k] : 0;
		}
		em_pre_used++;
	}

	if (!em_t_first_frame) {
		em_t_first_frame = millis();
	}
	em_t_last_frame = millis();

	/* The reply carries the body's sequence number plus one -- 0x53a4
	 * writes rx[4] + 1 into both status packets.  Held here and stamped at
	 * transmit time rather than written into the packet now: this runs in
	 * an interrupt and could otherwise land between em_transmit computing
	 * a checksum and sending the byte it covers. */
	g_reply_seq = (uint8_t)(rx[4] + 1);

	if (rx[3] != EM_CLASS_INIT) {
		/* Tally normal-class traffic by (length, id) -- see emount.h. */
		uint8_t i;

		for (i = 0; i < em_norm_used; i++) {
			if (em_norm_len[i] == rx[1] && em_norm_id[i] == rx[5]) {
				break;
			}
		}
		if (i < EM_NORM_SLOTS) {
			if (i == em_norm_used) {
				em_norm_len[i] = rx[1];
				em_norm_id[i]  = rx[5];
				em_norm_used++;
			}
			if (em_norm_count[i] != 0xFFFF) {
				em_norm_count[i]++;
			}

			em_cap_command(rx, n);
		} else {
			em_norm_overflow = 1;
		}
	}

	/* Message 0x1B, the aperture command: COUNTED, NOT OBEYED.
	 *
	 * Init-class, so it arrives here rather than on the per-frame path, and
	 * it carries an aperture in payload[0..1].  This adapter has no iris, so
	 * there is nothing to obey; the reply echoes the requester's own bytes
	 * back through the stored fixups.
	 *
	 * Counted because an a9 II sends exactly one per session, inside the
	 * burst of init-class requests that follows a shutter press
	 * (0x0a 0x0b 0x1b 0x28 0x19 0x19 0x28 0x0b 0x0a), and a body that ever
	 * drove the channel harder would show up here. */
	/* MESSAGE 0x08's REQUEST.
	 *
	 * Captured whole, because it is sent once per session and carries a
	 * capability bit nothing else does.  Bit 7 of payload[1] -- frame byte
	 * 7 -- gates message 0x28's payload[9..10], INVERTED: clear means the
	 * body wants the field.
	 *
	 * The aperture is re-applied on the spot so the decision takes effect
	 * without waiting for the next thing that happens to set it. */
	if (rx[3] == EM_CLASS_INIT && rx[5] == 0x08u && n >= 8) {
		uint16_t k;

		if (em_m08_n != 0xFFFFu) {
			em_m08_n++;
		}
		if (em_m08_len == 0) {
			for (k = 0; k < EM_M08_LEN && k < n; k++) {
				em_m08[k] = rx[k];
			}
			em_m08_len = (uint8_t)k;
		}
		em_m08_aperture_on = ((rx[7] & 0x80u) == 0u);
		set_28_aperture();
	}

	/* MESSAGE 0x0A's REQUEST: capture the first, and let every one pick
	 * the message 0x05 length (see emount.h). */
	if (rx[3] == EM_CLASS_INIT && rx[5] == 0x0Au && n >= 10) {
		uint16_t k;
		uint8_t  pl13 = em_msg_08[EM08_STATUS_FORM];

		if (em_m0a_n != 0xFFFFu) {
			em_m0a_n++;
		}
		if (em_m0a_len == 0) {
			for (k = 0; k < EM_M0A_LEN && (uint16_t)(6u + k + 3u) <= rx[1]; k++) {
				em_m0a[k] = rx[6 + k];
			}
			em_m0a_len = (uint8_t)k;
		}
		/* 00 00 asks for no fields at all -- the body sends it ahead of
		 * 0x16 (msg_0x0A.md) -- so it chooses no length either: the
		 * one in force stays. */
		if (rx[1] >= 11u && rx[6] == 0x00u && rx[7] == 0x00u) {
		} else if (rx[1] >= 11u && rx[6] == 0xFFu && rx[7] == 0x7Fu) {
			em_status05_len = EA9_LEN_05;
		} else if ((pl13 & 0xFEu) != 0x80u) {
			em_status05_len = EM05_LEN_LONG;
		}
	}

	if (rx[3] == EM_CLASS_INIT && rx[5] == EM_ID_APERTURE && n >= 11) {
		if (em_focus_n != 0xFFFFu) {
			em_focus_n++;
		}
		/* RECORD WHAT IT ASKED FOR.  The count alone cannot say whether
		 * a body ever commands an aperture this device does not have,
		 * which is the only reason the channel matters here. */
		em_aperture_req = (uint16_t)rx[6] | ((uint16_t)rx[7] << 8);
	}

	/* Message 0x04's MODE BYTE, payload[10] = frame offset 16, on EVERY
	 * frame -- not only the ones carrying records.
	 *
	 * Sampling only the command form was useless for its purpose: that form
	 * exists to carry an instruction, so the only values it can ever show
	 * are the focusing ones.  A histogram that can only observe one state
	 * cannot detect a change of state.
	 *
	 * 11 payload bytes need 6 header + 11 + 3 trailer = 20. */
	if (rx[3] != EM_CLASS_INIT && rx[5] == 0x04u && n >= 20) {
		uint8_t k;

		for (k = 0; k < em_mode_used; k++) {
			if (em_mode_val[k] == rx[16]) {
				break;
			}
		}
		if (k < EM_MODE_SLOTS) {
			if (k == em_mode_used) {
				em_mode_val[k] = rx[16];
				em_mode_used++;
			}
			if (em_mode_count[k] != 0xFFFFu) {
				em_mode_count[k]++;
			}
		}
	}

	/* MESSAGE 0x03's APERTURE REQUEST.
	 *
	 * 0x03 is body state and this device acts on none of it, but pl[5..6]
	 * is the aperture the body is asking for (msg_0x03.md) -- and an
	 * adapter with one fixed aperture wants to know if that is ever
	 * something other than the one it has.  Recorded, not obeyed, exactly
	 * as message 0x1B's is.
	 *
	 * 6 header + 7 payload bytes needs 6 + 7 + 3 = 16. */
	if (rx[3] != EM_CLASS_INIT && rx[5] == 0x03u && n >= 16) {
		em_m03_aperture = (uint16_t)rx[11] | ((uint16_t)rx[12] << 8);
		if (em_m03_n != 0xFFFFu) {
			em_m03_n++;
		}
	}

	/* MESSAGE 0x06 pl[23] bit 6 FOLLOWS THE BODY'S CYCLE.
	 *
	 * 0x03 pl[10] is the body's cycle length N and pl[11] the frame within
	 * it, 0..N-1.  Sony lenses flip pl[23] bit 6 on every frame where
	 * pl[11] returns to 0; with N = 1, where pl[11] is always 0, they flip
	 * every second frame instead.
	 *
	 * 6 header + 12 payload bytes needs 6 + 12 + 3 = 21. */
	if (rx[3] != EM_CLASS_INIT && rx[5] == 0x03u && n >= 21 && rx[17] == 0) {
		static uint8_t skip;

		if (rx[16] >= 2u || (skip ^= 1u) == 0u) {
			em_msg_06[EM06_CYCLE] ^= EM06_CYCLE_BIT;
		}
	}

	/* MESSAGE 0x03's RECORDS: the row index.  Only the 23-byte payload a
	 * Sony native gets carries one; the 20-byte form a body sends an
	 * adapter ends where the records would start.  Bounded by the frame's
	 * declared length, as message 0x04's are. */
	if (rx[3] != EM_CLASS_INIT && rx[5] == 0x03u && !em_shutdown_acked) {
		uint16_t end = (uint16_t)rx[1] | ((uint16_t)rx[2] << 8);

		if (end > n) {
			end = n;
		}
		if (end >= EM03_REC_START + 3u) {
			em_parse_03_records(rx, (uint16_t)(end - 3u));
		}
	}

	/* THE RECORD STREAM.
	 *
	 * 6 header bytes + a 13-byte message header take us to pl[13] = frame
	 * offset 19, where the records start, and they run to the checksum.
	 *
	 * BOUNDED BY THE FRAME'S OWN DECLARED LENGTH, not by how many bytes
	 * arrived.  The body PADS its chip-select windows -- 16, 32 and 48
	 * bytes are the only sizes ever measured -- so a 22-byte frame lands in
	 * a 32-byte window and `n` overshoots the frame by ten bytes of
	 * nothing.  Walking to `n` reads the checksum, the terminator and the
	 * padding as records, and the terminator 0x55 is a perfectly good
	 * operand byte (NOTES.md 82).
	 *
	 * Clamped to what actually arrived as well: a window shorter than the
	 * frame it declares cannot be trusted past its own end either. */
	if (rx[3] != EM_CLASS_INIT && rx[5] == 0x04u && !em_shutdown_acked) {
		uint16_t end = (uint16_t)rx[1] | ((uint16_t)rx[2] << 8);

		if (end > n) {
			end = n;
		}
		/* THE FOCUS MODE, pl[3].  It moves the motor -- the main loop
		 * returns to infinity on entering manual focus -- and nothing
		 * else on this path checks a checksum, so a frame whose
		 * checksum does not verify is skipped here rather than let one
		 * corrupted byte drive the helicoid.  Skipped, not counted as
		 * "not manual": it says nothing either way. */
		if (end >= 13u && end == ((uint16_t)rx[1] | ((uint16_t)rx[2] << 8))) {
			uint16_t sum = 0, k;

			for (k = 1; k <= (uint16_t)(end - 4u); k++) {
				sum += rx[k];
			}
			if (rx[end - 1] == EM_FRAME_END
			    && rx[end - 3] == (uint8_t)sum
			    && rx[end - 2] == (uint8_t)(sum >> 8)) {
				/* Manual focus only after EM_MF_FRAMES
				 * consecutive 0x81 frames; the first other
				 * frame leaves it at once (the stock's 0x5308). */
				if (rx[9] != EM04_MODE_MF) {
					g_mf_run    = 0;
					em_focus_mf = 0;
				} else if (g_mf_run < EM_MF_FRAMES
				           && ++g_mf_run == EM_MF_FRAMES) {
					em_focus_mf = 1;
					count16(&em_mf_edges);
				}
			}
		}
		if (end >= EM04_REC_START + 3u) {
			em_parse_records(rx, (uint16_t)(end - 3u));
		}
	}

	/* The body's capability offer, payload[0..7] of its message 0x01 -- a
	 * 64-bit little-endian bitmap where bit n means it supports id n+1.
	 *
	 * Captured because nothing in the corpus records an a9 II offer, and
	 * which ids a body admits to supporting bounds every other question
	 * about which channels it will use. */
	/* 6 header + 8 payload + 2 checksum + 1 terminator = 17.  A shorter
	 * guard (n >= 14) reads the terminator as offer[7]; the host test
	 * caught exactly that. */
	if (rx[3] == EM_CLASS_INIT && rx[5] == EM_ID_SESSION && n >= 17) {
		uint8_t k;

		for (k = 0; k < 8; k++) {
			em_body_offer[k] = rx[6 + k];
		}
	}

	if (rx[3] == EM_CLASS_FILE
	    && (rx[5] == EM_ID_FILE_SIZE || rx[5] == EM_ID_FILE_PAGE)) {
		g_file_id      = rx[5];
		g_file_p0      = (n >= 10u) ? rx[6] : 0u;
		g_file_p1      = (n >= 11u) ? rx[7] : 0u;
		g_file_pending = 1;
	}

	/* Message 0x16's pl[0], for the tally: 01 or 03 by the body's state. */
	if (rx[3] == EM_CLASS_INIT && rx[5] == EM_ID_SHUTDOWN && n >= 10) {
		em_m16_pl0 = rx[6];
	}

	if (rx[3] == EM_CLASS_INIT) {
		if (g_woke && !em_wake_first_id) {
			em_wake_first_id = rx[5];
		}
		em_last_id = rx[5];
		req_push(rx, n);
		if (rx[5] < 32) {
			em_id_mask_lo |= (1u << rx[5]);
		} else if (rx[5] < 64) {
			em_id_mask_hi |= (1u << (rx[5] - 32));
		}
		if (em_id_log_n < EM_ID_LOG_LEN) {
			em_id_log[em_id_log_n++] = rx[5];
		}
	}
}

/* THE FRAMER.  The receive ring holds bytes, not windows: a frame is wherever
 * an 0xF0 starts a run whose length field, 0x55 terminator and checksum all
 * agree.  This replaces "rising edge clears the buffer, falling edge parses
 * it", which needed both edges handled on time.  A CPU held up for a few
 * milliseconds -- a flash write right as the body's first request came in,
 * dump_fail_start.bin -- took the rising edge late, cleared the head of the
 * frame that had already arrived, and the body never got an answer.  Now a
 * late edge only means a late parse.
 *
 * It runs only with the body's chip select LOW -- on a falling edge, or by
 * hand in em_init after the bring-up window -- for two reasons.  The window is
 * over, so a frame that has not completed never will and its 0xF0 is not one.
 * And reading the DMA's position suspends the channel for a moment, which is
 * only harmless while the body sends nothing (em_uart.c).  A falling edge
 * handled so late that the next window has already opened finds the line
 * high and does nothing; that window's own fall parses both, one frame late.
 *
 * The body pads its windows (16, 32 or 48 bytes), so padding is skipped here
 * one byte at a time; a padding 0xF0 followed by bytes that pass all three
 * checks is the only way to mistake it for a frame.
 *
 * Checksum: the 16-bit sum of everything after the 0xF0 up to the checksum,
 * as on the lens's own frames.  Every body frame captured passes it. */
#define EM_FRAME_MIN      9u      /* header 6 + checksum 2 + terminator 1 */
#define EM_RX_SETTLE_US   30u     /* a last byte still in flight at the fall */
static uint8_t g_frame[EM_RX_BUF_LEN];
volatile uint16_t em_rx_skipped;  /* bytes that started no frame: padding */

static uint16_t em_rx_drain(void)
{
	uint16_t avail;
	uint16_t frames  = 0;
	uint8_t  settled = 0;

	if (em_pin_level(EM_PIN_BODY_CS)) {
		return 0;
	}
	avail = em_uart_rx_count();
	while (avail) {
		uint16_t len, k, sum;

		if (em_uart_rx_at(0) != EM_FRAME_START) {
			em_uart_rx_consume(1);
			avail--;
			count16(&em_rx_skipped);
			continue;
		}
		len = (avail >= 3u)
		      ? (uint16_t)(em_uart_rx_at(1) | ((uint16_t)em_uart_rx_at(2) << 8))
		      : 0u;
		if (avail < 3u || avail < len) {
			if (!settled) {
				/* The fall can beat the last byte by a few us.  If
				 * the next window has opened meanwhile, leave the
				 * tail for its fall rather than read mid-transfer. */
				settled = 1;
				delay_us(EM_RX_SETTLE_US);
				if (em_pin_level(EM_PIN_BODY_CS)) {
					break;
				}
				avail = em_uart_rx_count();
				continue;
			}
			if (avail < 3u) {
				em_uart_rx_consume(avail);
				break;
			}
		}
		if (len < EM_FRAME_MIN || len > EM_RX_BUF_LEN || avail < len
		    || em_uart_rx_at(len - 1u) != EM_FRAME_END) {
			goto not_a_frame;
		}
		for (sum = 0, k = 1; k <= (uint16_t)(len - 4u); k++) {
			sum = (uint16_t)(sum + em_uart_rx_at(k));
		}
		if (em_uart_rx_at(len - 3u) != (uint8_t)sum
		    || em_uart_rx_at(len - 2u) != (uint8_t)(sum >> 8)) {
			goto not_a_frame;
		}
		for (k = 0; k < len; k++) {
			g_frame[k] = em_uart_rx_at(k);
		}
		em_uart_rx_consume(len);
		avail = (uint16_t)(avail - len);
		em_frame_in(g_frame, len);
		frames++;
		continue;

not_a_frame:
		if (em_bad_frames != 0xFFu) {
			em_bad_frames++;
		}
		em_uart_rx_consume(1);
		avail--;
	}
	return frames;
}

/* The body's chip select, both edges (0x53a4).  The fall ends a window and
 * runs the framer; the rise needs nothing -- the DMA is already receiving. */
static void on_cs_edge(int level)
{
	em_cs_edges++;
	if (!level) {
		(void)em_rx_drain();
	}
}

/* The body's frame sync, rising edge (0x5120).  Once the handshake is over
 * this is what clocks the status pair at the body's frame rate. */
static void on_vd_edge(void)
{
	uint32_t now = millis();

	g_t_vd_prev_us = g_t_vd_us;
	g_t_vd_us      = micros();
	em_vd_edges++;
	g_motor_vd = g_motor;
	if (!em_t_first_vd) {
		em_t_first_vd = now;
	}

	/* Time the body's polling.  A native lens measures this rather than
	 * assuming it, and only accepts a delta inside the band that reads as a
	 * real frame rate -- so a missed edge, a burst of two, or the long gap
	 * across a shutdown cannot poison the period (msg_0x06.md).  The
	 * position trace's eighths are timed by this number. */
	if (g_t_prev_vd) {
		uint32_t d = now - g_t_prev_vd;

		if (d >= EM_FRAME_PERIOD_MIN && d <= EM_FRAME_PERIOD_MAX) {
			em_frame_period_ms = (uint16_t)d;
			em_frame_period_n++;
		}
	}
	g_t_prev_vd = now;

	if (em_handshake_done) {
		g_status_due = 1;
	}
}

/* Clamp into the travel the adapter advertises in message 0x06.  An
 * out-of-range position is worse than a stale one: the body would be told the
 * lens is somewhere its own limits say it cannot be. */
static uint16_t clamp_travel(int32_t pos)
{
	if (pos < ENC_TRAVEL_LO) {
		pos = ENC_TRAVEL_LO;
	} else if (pos > ENC_TRAVEL_HI) {
		pos = ENC_TRAVEL_HI;
	}
	return (uint16_t)pos;
}

static void put16(uint8_t *frame, uint16_t off, uint16_t v)
{
	frame[off]     = (uint8_t)(v & 0xFFu);
	frame[off + 1] = (uint8_t)(v >> 8);
}

void em_set_motor(int8_t dir)
{
	g_motor = dir;
}

/* Message 0x06 payload[32..38]: the position trace WITHIN the last frame, as
 * Yongnuo 50 DF builds it.  Its frame-sync handler (FUN_08015f80) latches the
 * position into L[0] and starts a timer that samples L[1..7] at every eighth
 * of the frame period (0x08016020: 40000 ticks at 20 ms frames, 33280 at
 * 16.7); the next frame sync takes L[8] and snapshots the lot, and the status
 * writer (FUN_080160a4) sends pl[32+k] = L[k+1] - L[k+2], k = 0..6 -- OLDER
 * minus newer, one eighth of a frame each.
 *
 * Here there is no timer: the encoder is read by the caller's loop, which
 * reads it anyway, and every reading goes into a time-stamped ring.  When a
 * frame sync has gone by, L[1..8] are INTERPOLATED out of the ring at the
 * exact instants -- the previous sync plus k eighths of the measured interval
 * between the two syncs, L[8] at the sync itself -- so neither a late loop
 * iteration nor a blocking step that skipped some eighths moves them.  The
 * body expects the frame-sync instant (msg_0x06.md): it files these points by
 * frame and lines them up with the exposure. */
static uint16_t g_pos_target;     /* where the running move is going */
static uint8_t  g_pos_set;        /* a position has been reported */
#define SUB_N 8
static uint16_t g_sub[SUB_N + 1];   /* L[8] in [0] once latched, L[1..8] */
static uint8_t  g_sub_valid;      /* g_sub[0] holds a real latch */
static uint32_t g_sub_vd;         /* em_vd_edges at that latch */
static int16_t  g_frame_disp;     /* L[0] minus the previous L[0], per frame */
static int32_t  g_v0_q8;          /* speed at the latch, units/ms x 256 */

/* The forecast's mechanism model (em_set_forecast_model), units/ms x 256. */
static uint32_t g_fc_cap_q8 = 1600u;   /* 25 counts/ms = 6.25 units/ms, cruise */
static uint32_t g_fc_sat_q8 = 2926u;   /* 45.7 counts/ms, duty 1280 */
static uint16_t g_fc_tau_ms = 26u;

void em_set_forecast_model(uint32_t v_cap_ups, uint32_t v_sat_ups,
                           uint16_t tau_ms)
{
	g_fc_cap_q8 = v_cap_ups * 256u / 1000u;
	g_fc_sat_q8 = v_sat_ups * 256u / 1000u;
	g_fc_tau_ms = tau_ms ? tau_ms : 1u;
}

/* THE RING.  One entry per encoder reading, (micros, position), newest at
 * g_ring_head - 1.  A reading closer than POS_RING_MIN_US to the entry before
 * the newest... more exactly: while the newest entry is closer than that to
 * the one before it, each new reading overwrites it.  So however fast the
 * loop reads, all neighbours but the newest are >= 0.625 ms apart and 64
 * entries span >= 39 ms -- past the longest frame latch_frame accepts (37.5 ms).  The first
 * reading after a frame sync is always added, so the sync has a reading on
 * each side.  A reading more than POS_RING_STALE_US after the newest clears
 * the ring first: entries that old say nothing about now, and past 2^31 us
 * the signed time comparisons would misread them.  64 is a power of two, so
 * indexing is a mask -- a Cortex-M0+ divides in software. */
#define POS_RING_N        64u
#define POS_RING_MASK     (POS_RING_N - 1u)
#define POS_RING_MIN_US   625u
#define POS_RING_STALE_US 1000000u
#define POS_EXTRAP_US     2000u    /* how far past the newest reading to project */
static uint32_t g_ring_t[POS_RING_N];
static uint16_t g_ring_p[POS_RING_N];
static uint8_t  g_ring_head;      /* next slot to write */
static uint8_t  g_ring_n;         /* entries held, 0..POS_RING_N */
static uint32_t g_ring_vd;        /* em_vd_edges when the newest was written */

static uint8_t ring_idx(uint8_t back)   /* back = 0: the newest; back < g_ring_n */
{
	return (uint8_t)((g_ring_head - 1u - back) & POS_RING_MASK);
}

static void ring_push(uint16_t p)
{
	uint32_t t  = micros();
	uint32_t vd = em_vd_edges;

	if (g_ring_n && t - g_ring_t[ring_idx(0)] > POS_RING_STALE_US) {
		g_ring_n = 0;
	}
	/* The newest entry is provisional while it is still within
	 * POS_RING_MIN_US of the one before it: overwrite it.  So every pair of
	 * neighbours but the newest is >= POS_RING_MIN_US apart. */
	if (g_ring_n >= 2 && vd == g_ring_vd
	    && g_ring_t[ring_idx(0)] - g_ring_t[ring_idx(1)] < POS_RING_MIN_US) {
		g_ring_t[ring_idx(0)] = t;          /* replace the newest */
		g_ring_p[ring_idx(0)] = p;
		return;
	}
	g_ring_t[g_ring_head] = t;
	g_ring_p[g_ring_head] = p;
	g_ring_head = (uint8_t)((g_ring_head + 1u) & POS_RING_MASK);
	if (g_ring_n < POS_RING_N) {
		g_ring_n++;
	}
	g_ring_vd = vd;
}

/* The position at instant `at`, given b: the newest entry at or before `at`
 * (b = 0: `at` is at or past the newest reading; b = g_ring_n: older than
 * every reading).  Between two readings: linear, rounded to nearest.  Older
 * than the ring: its oldest reading.  Past the newest: projected along the
 * last two readings for up to POS_EXTRAP_US, else held. */
static int32_t ring_interp(uint8_t b, uint32_t at)
{
	uint8_t i, j;
	int32_t span, into, d;

	if (b >= g_ring_n) {
		return g_ring_p[ring_idx((uint8_t)(g_ring_n - 1u))];
	}
	i = ring_idx(b);
	if (b == 0) {
		uint32_t past = at - g_ring_t[i];

		if (past == 0 || g_ring_n < 2 || past > POS_EXTRAP_US) {
			return g_ring_p[i];
		}
		j = ring_idx(1);
		span = (int32_t)(g_ring_t[i] - g_ring_t[j]);
		if (span <= 0) {
			return g_ring_p[i];
		}
		return (int32_t)g_ring_p[i]
		       + ((int32_t)g_ring_p[i] - (int32_t)g_ring_p[j])
		         * (int32_t)past / span;
	}
	j    = ring_idx((uint8_t)(b - 1u));     /* the next newer reading */
	span = (int32_t)(g_ring_t[j] - g_ring_t[i]);
	into = (int32_t)(at - g_ring_t[i]);
	d    = (int32_t)g_ring_p[j] - (int32_t)g_ring_p[i];
	if (span <= 0) {
		return g_ring_p[j];
	}
	return (int32_t)g_ring_p[i]
	       + (d * into + (d >= 0 ? span / 2 : -span / 2)) / span;
}

/* Step b back from where it is to the newest entry at or before `at`.  Called
 * with instants in DESCENDING order, so one walk of the ring serves all nine
 * points of a frame. */
static uint8_t ring_seek(uint8_t b, uint32_t at)
{
	while (b < g_ring_n && (int32_t)(g_ring_t[ring_idx(b)] - at) > 0) {
		b++;
	}
	return b;
}

static uint8_t sat8(int32_t v)
{
	if (v > 127) {
		v = 127;
	} else if (v < -128) {
		v = -128;
	}
	return (uint8_t)(int8_t)v;
}

/* A frame sync has gone by: interpolate L[1..8] for the frame that ended at
 * it, and derive the trace, the frame's displacement and the speed at the
 * sync from them. */
static void latch_frame(void)
{
	uint8_t *h = &em_msg_06[EM06_VEL_HIST];
	uint32_t t_now, t_prev, span, vd;
	int32_t  L[SUB_N + 1];
	uint8_t  k, b;
	int      span_ok, latch_ok, trace_ok;

	__disable_irq();
	vd     = em_vd_edges;
	t_now  = g_t_vd_us;
	t_prev = g_t_vd_prev_us;
	__enable_irq();

	span    = t_now - t_prev;
	span_ok = span >= EM_FRAME_PERIOD_MIN * 1000u * 3u / 4u
	          && span <= EM_FRAME_PERIOD_MAX * 1000u * 5u / 4u;
	/* A latch is real only with readings to take it from and a sync that
	 * was timed: the handshake counts syncs by polling, untimed. */
	latch_ok = g_ring_n != 0 && t_now != 0;
	trace_ok = latch_ok && span_ok && g_sub_valid;

	if (g_ring_n == 0) {
		for (k = 1; k <= SUB_N; k++) {
			L[k] = g_pos_now;
		}
	} else {
		b        = ring_seek(0, t_now);
		L[SUB_N] = clamp_travel(ring_interp(b, t_now));
		for (k = SUB_N - 1; k >= 1; k--) {
			uint32_t at = t_prev + span * k / SUB_N;

			if (!span_ok) {
				L[k] = L[SUB_N];
				continue;
			}
			b    = ring_seek(b, at);
			L[k] = clamp_travel(ring_interp(b, at));
		}
	}
	for (k = 0; k < EM06_VEL_HIST_N; k++) {
		h[k] = trace_ok ? sat8(L[k + 1] - L[k + 2]) : 0u;
	}
	/* The last frame's displacement -- per frame, if syncs went by
	 * unlatched. */
	g_frame_disp = latch_ok && g_sub_valid
	               ? (int16_t)((L[SUB_N] - (int32_t)g_sub[0])
	                           / (int32_t)(vd - g_sub_vd))
	               : 0;
	/* The speed at the sync, for the forecast: the last eighth's average,
	 * raised by the half-eighth of full-duty acceleration it lags behind --
	 * so it is not below the real speed. */
	{
		int32_t eighth_us = span_ok ? (int32_t)(span / SUB_N) : 0;
		int32_t dv = trace_ok ? L[SUB_N] - L[SUB_N - 1] : 0;
		int32_t v  = eighth_us > 0 ? dv * 256000 / eighth_us : 0;
		int32_t a  = (int32_t)g_fc_sat_q8 - (v < 0 ? -v : v);
		int32_t up = a > 0 && eighth_us > 0
		             ? a * eighth_us / (2000 * (int32_t)g_fc_tau_ms) : 0;

		g_v0_q8 = v > 0 ? v + up : v < 0 ? v - up : 0;
	}
	for (k = 1; k <= SUB_N; k++) {
		g_sub[k] = (uint16_t)L[k];
	}
	g_sub[0]    = (uint16_t)L[SUB_N];
	g_sub_valid = (uint8_t)latch_ok;
	g_sub_vd    = vd;
}

/* Every encoder reading the caller makes, into the ring; the first one after
 * a frame sync also latches that frame. */
void em_sample_position(int32_t pos)
{
	ring_push(clamp_travel(pos));
	if (g_sub_vd != em_vd_edges) {
		latch_frame();
	}
}

void em_set_subject_distance(uint16_t code, uint8_t coarse, uint8_t coarse_pos)
{
	put16(em_msg_05, EM05_DISTANCE, code);
	em_msg_05[EM05_DIST_COARSE] = coarse;
	em_msg_05[EM05_POS_COARSE]  = coarse_pos;
	put16(em_msg_28, EM28_DISTANCE, code);
	em_msg_28[EM28_DIST_COARSE] = coarse;
	put16(em_msg_35, EM28_DISTANCE, code);
	em_msg_35[EM28_DIST_COARSE] = coarse;
}

void em_set_min_focus(uint16_t code)
{
	put16(em_msg_05, EM05_MIN_FOCUS, code);
}

void em_set_in_motion(uint8_t moving)
{
	uint8_t v = em_msg_05[EM05_FLAGS];

	v |= EM05_FLAG_REPORTS;
	if (moving) {
		v |= EM05_FLAG_MOVING;
	} else {
		v = (uint8_t)(v & ~EM05_FLAG_MOVING);
	}
	em_msg_05[EM05_FLAGS] = v;
}

static uint16_t g_pos_rest;       /* pl[2..3] while at rest, taken once */
static uint8_t  g_pos_rest_held;  /* g_pos_rest has been taken */

void em_set_focus_position(int32_t pos, int32_t target)
{
	ring_push(clamp_travel(pos));
	g_pos_now    = clamp_travel(pos);
	g_pos_target = clamp_travel(target);
	g_pos_set    = 1;
}

/* How far focus can get toward the target, `left` units away, within
 * EM06_FORECAST_FRAMES frames of the latch.  An upper bound, not a
 * prediction.  The plant is tau * dv/dt + v = v_sat at full duty, so its
 * acceleration only falls as it speeds up: holding the latch's acceleration
 * a0 = (v_sat - u) / tau for the whole frame, then the move's speed cap,
 * gives a speed that is never below the real one -- and so a distance that is
 * never short of it.  Two pieces, both closed form (units x 256, ms):
 *
 *   cap not reached:  d = u*T + (v_sat - u) * T^2 / (2 tau)
 *   cap reached:      d = cap*T - (cap - u)^2 * tau / (2 (v_sat - u))
 *
 * the second being "accelerate to the cap, then cruise" with the time to the
 * cap, (cap - u) * tau / (v_sat - u), folded in.  Already at or above the cap
 * (or v_sat): u*T, no deceleration assumed.  Integer division only shrinks
 * what is subtracted, so rounding errs long.  Deceleration is not modelled:
 * near the target it reports the target early rather than late, and it never
 * passes the target.  Moving away from the target: 0 -- no forecast past
 * where it is. */
static int32_t forecast06(int32_t left, int8_t m)
{
	int32_t  dir  = left > 0 ? 1 : left < 0 ? -1 : 0;
	uint32_t room = (uint32_t)(left * dir) << 8;     /* x 256 */
	uint32_t t    = (uint32_t)em_frame_period_ms * EM06_FORECAST_FRAMES;
	uint32_t tau  = g_fc_tau_ms;
	uint32_t cap  = g_fc_cap_q8, sat = g_fc_sat_q8;
	uint32_t u, x;

	/* Moving away: driven away, or still travelling away -- in the last
	 * eighth or over the last frame (a reversal coasting the old way). */
	if (dir == 0 || m * dir < 0 || g_v0_q8 * dir < 0
	    || (int32_t)g_frame_disp * dir < 0) {
		return 0;
	}
	u = (uint32_t)(g_v0_q8 * dir);

	if (u >= cap || u >= sat) {
		x = u * t;
	} else if ((sat - u) * t <= (cap - u) * tau) {
		x = u * t + (sat - u) * t * t / (2u * tau);
	} else {
		x = cap * t - (cap - u) * (cap - u) * tau / (2u * (sat - u));
	}
	if (x > room) {
		x = room;
	}
	return dir * (int32_t)((x + 255u) >> 8);
}

/* Message 0x06 payload[0..3] and [20..21], written just before each send.
 * As Yongnuo 50 DF builds them (FUN_08016150, FUN_080160a4):
 *
 *   pl[20..21]  the position latched at the last frame sync
 *   pl[2..3]    that position plus a forecast: while the motor runs, the
 *               farthest focus can get toward the target in one frame
 *               (forecast06: the latch's full-duty acceleration held, up to
 *               the move's speed cap), never past the target, clamped into the advertised
 *               travel.  One frame because an A7 IV files pl[2..3] under
 *               the next frame (msg_0x06.md).  Moving away from the target,
 *               it is the position itself; at rest,
 *               the position at the first frame of rest, held there until
 *               the motor runs again -- steady, as Sony natives send it,
 *               where pl[20..21] jitters.  Yongnuo computes the
 *               displacement from its motion profile; this one is measured.
 *   pl[0..1]    02 02 / 02 04 if the motor was running at the frame sync
 *               or is running now (02 = toward higher positions), else
 *               82 00.  The direction is the one it actually moved in the
 *               last frame, so a reversal coasting the old way is reported
 *               as that until it turns; under EM06_DIR_MIN counts it is the
 *               commanded one; then OR 0x10 / 0x08 when the CURRENT position is
 *               within NEAR_LIMIT of the lower / upper limit */
static void status06_focus(void)
{
	uint16_t lo = (uint16_t)em_msg_06[EM06_LIMIT_LO]
	              | ((uint16_t)em_msg_06[EM06_LIMIT_LO + 1] << 8);
	uint16_t hi = (uint16_t)em_msg_06[EM06_LIMIT_HI]
	              | ((uint16_t)em_msg_06[EM06_LIMIT_HI + 1] << 8);
	int8_t   m  = g_motor_vd ? g_motor_vd : g_motor;
	uint8_t  v;

	if (!g_pos_set) {
		return;
	}
	/* A frame sync with no reading after it yet: latch now, projecting the
	 * last readings a little past the edge (ring_interp). */
	if (g_sub_vd != em_vd_edges) {
		latch_frame();
	}
	put16(em_msg_06, EM06_POS_B, g_sub[0]);
	{
		int32_t at   = g_sub[0];
		int32_t left = (int32_t)g_pos_target - at;
		int32_t d    = m ? forecast06(left, m) : 0;

		if (m) {
			g_pos_rest_held = 0;
			put16(em_msg_06, EM06_POS_A, clamp_travel(at + d));
		} else {
			/* At rest: the position on the first frame with no move
			 * running, then held, so encoder jitter does not reach
			 * pl[2..3].  The body takes 13 frames of an unchanged
			 * pl[2..3] as the end of a Drive (msg_0x06.md). */
			if (!g_pos_rest_held) {
				g_pos_rest      = (uint16_t)at;
				g_pos_rest_held = 1;
			}
			put16(em_msg_06, EM06_POS_A, g_pos_rest);
		}
	}

	v = m ? EM06_MOVING : EM06_AT_REST;
	if (g_pos_now <= lo + EM06_NEAR_LIMIT) {
		v |= EM06_NEAR_LO;
	}
	if ((int32_t)g_pos_now >= (int32_t)hi - EM06_NEAR_LIMIT) {
		v |= EM06_NEAR_HI;
	}
	em_msg_06[EM06_MOTION]    = v;
	if (m && (g_frame_disp >= EM06_DIR_MIN || g_frame_disp <= -EM06_DIR_MIN)) {
		m = g_frame_disp > 0 ? 1 : -1;
	}
	em_msg_06[EM06_DIRECTION] = m > 0 ? 0x02u : m < 0 ? 0x04u : 0x00u;
}


/* The scan-status queue (see emount.h): taken one value per status frame. */
#define SCAN_Q_N          4
#define SCAN_CLEAR        0xFFu   /* back to idle, and arm the event */
#define SCAN_EVENT_FRAMES 5
static uint8_t g_scan_q[SCAN_Q_N];
static uint8_t g_scan_qn;
static uint8_t g_scan_event_in;   /* frames until 1F 00; 0 = none pending */

static void scan_push(uint8_t v)
{
	if (g_scan_qn < SCAN_Q_N) {
		g_scan_q[g_scan_qn++] = v;
	}
}

void em_set_scan_state(uint8_t state)
{
	g_scan_qn       = 0;
	g_scan_event_in = 0;
	em_msg_06[EM06_SCAN_STATE] = state;
}

void em_scan_launch(void)
{
	scan_push(EM_SCAN_ACCEPTED);
	scan_push(EM_SCAN_SWEEPING);
}

/* Events waiting to go out, oldest first.  One per frame: a 50-byte frame's
 * checksum lands on the staging byte, so the stock reverts the length after
 * every single event frame and so does this.
 *
 * Two deep because a stop produces two -- the code for the command it killed,
 * then 0x1C -- on consecutive body frames, which is 32 ms apart and far longer
 * than anything else here takes. */
#define EM_EVENT_Q 2
static volatile uint8_t g_event_code[EM_EVENT_Q];
static volatile uint8_t g_event_param[EM_EVENT_Q];
static volatile uint8_t g_event_n;

void em_post_event(uint8_t code, uint8_t param)
{
	if (g_event_n < EM_EVENT_Q) {
		g_event_code[g_event_n]  = code;
		g_event_param[g_event_n] = param;
		g_event_n++;
	}
}

/* The answers waiting to ride in the next tail, already converted. */
static volatile uint8_t  g_ans_tag[EM_QUERY_Q];
static volatile uint16_t g_ans_val[EM_QUERY_Q];
static volatile uint8_t  g_ans_n;

void em_post_query_answer(uint8_t tag, uint16_t value)
{
	if (g_ans_n < EM_QUERY_Q) {
		g_ans_tag[g_ans_n] = tag;
		g_ans_val[g_ans_n] = value;
		g_ans_n++;
	}
}

void em_scan_finish(void)
{
	scan_push(EM_SCAN_DONE);
	scan_push(SCAN_CLEAR);
}

/* Once per status frame, before message 0x06 is built: count down a pending
 * 1F 00, then take the next queued value into payload[25].  An empty queue
 * leaves the last value standing. */
static void scan_state_frame(void)
{
	uint8_t v, k;

	if (g_scan_event_in && --g_scan_event_in == 0) {
		em_post_event(EM_REC_SCAN, 0x00u);
	}
	if (!g_scan_qn) {
		return;
	}
	v = g_scan_q[0];
	for (k = 1; k < g_scan_qn; k++) {
		g_scan_q[k - 1] = g_scan_q[k];
	}
	g_scan_qn--;
	if (v == SCAN_CLEAR) {
		v = EM_SCAN_IDLE;
		g_scan_event_in = SCAN_EVENT_FRAMES;
	}
	em_msg_06[EM06_SCAN_STATE] = v;
}

uint16_t em_defocus_scale(void)
{
	return (uint16_t)em_msg_06[EM06_DEFOCUS]
	       | ((uint16_t)em_msg_06[EM06_DEFOCUS + 1] << 8);
}

/* Messages 0x28 and 0x35 pl[9..10] carry the current aperture only when the
 * body's 0x08 request asks for it (emount.h); otherwise zero.  Filling it
 * unasked made an a9 II re-meter after every exposure. */
static void set_28_aperture(void)
{
	uint16_t av = (uint16_t)em_msg_05[EM05_AP_NOW_A]
	              | ((uint16_t)em_msg_05[EM05_AP_NOW_A + 1] << 8);

	put16(em_msg_28, EM28_AP_NOW, em_m08_aperture_on ? av : 0u);
	put16(em_msg_35, EM28_AP_NOW, em_m08_aperture_on ? av : 0u);
}

uint8_t em_take_focus_cmd(struct em_focus_cmd *out)
{
	uint8_t k;

	if (!em_focus_pending) {
		return 0;
	}
	/* The chip-select interrupt writes g_cmd; a frame landing mid-copy
	 * would hand over half of each instruction. */
	__disable_irq();
	em_focus_pending = 0;
	for (k = 0; k < sizeof(*out); k++) {
		((uint8_t *)out)[k] = ((const volatile uint8_t *)&g_cmd)[k];
	}
	__enable_irq();
	return 1;
}

uint8_t em_peek_focus_op(void)
{
	return em_focus_pending ? g_cmd.op : (uint8_t)EM_OP_NONE;
}

uint8_t em_take_query(uint8_t *tag, uint16_t *operand)
{
	uint8_t k;

	if (!g_query_n) {
		return 0;
	}
	/* The chip-select interrupt appends to this queue. */
	__disable_irq();
	*tag     = g_query_tag[0];
	*operand = g_query_op[0];
	for (k = 1; k < g_query_n; k++) {
		g_query_tag[k - 1] = g_query_tag[k];
		g_query_op[k - 1]  = g_query_op[k];
	}
	g_query_n--;
	__enable_irq();
	return 1;
}

/* 0..99 as two BCD digits, the encoding of message 0x08's nameplate. */
static uint8_t bcd2(uint16_t n)
{
	return (uint8_t)(((n / 10u % 10u) << 4) | (n % 10u));
}

/* The focal length, from em_focal_mm10 -- the one word the updater patches --
 * into every message that carries it.  The templates hold zeros there. */
static void put_focal(void)
{
	uint16_t mm10 = em_focal_mm10;
	uint16_t mm   = (uint16_t)((mm10 + 5u) / 10u);

	put16(em_msg_05, EM05_FOCAL,     mm10);
	put16(em_msg_05, EM05_FOCAL + 2, mm10);
	put16(em_msg_28, EM28_FOCAL,     mm10);
	put16(em_msg_28, EM28_FOCAL + 2, mm10);
	put16(em_msg_35, EM28_FOCAL,     mm10);
	put16(em_msg_35, EM28_FOCAL + 2, mm10);
	em_msg_08[EM08_FOCAL_BCD]     = bcd2(mm / 100u);
	em_msg_08[EM08_FOCAL_BCD + 1] = bcd2(mm);
}

void em_init(void)
{
	/* Fixed content is in the templates (emount_packets.c); the focal
	 * length is written here. */
	put_focal();

	/* protocol_init, 0x551c, replicated including the one non-obvious bit:
	 * our chip select goes high FIRST and stays high across the whole
	 * bring-up, only dropping 50 ms after the body's first pulse.  The pin
	 * is already an output driven low -- board_init_pins did that, and the
	 * caller must have run it. */
	PORT->Group[0].OUTSET.reg = (1u << EM_PIN_LENS_CS);

	/* PA02 and PA03 have to be readable before the EIC owns them. */
	PORT->Group[0].PINCFG[EM_PIN_BODY_CS].reg = PORT_PINCFG_INEN;
	PORT->Group[0].PINCFG[EM_PIN_BODY_VD].reg = PORT_PINCFG_INEN;

	/* LISTEN BEFORE THE FIRST WINDOW OPENS.
	 *
	 * This used to be five milliseconds AFTER the body's chip select went
	 * high, copied from the stock's protocol_init.  At 750 kbaud five
	 * milliseconds is about 375 byte times, so if the body put anything in
	 * that first window the receiver started in the middle of it and the
	 * buffer began mid-frame.  Two failing boots were measured receiving 47
	 * and 29 bytes that way, and the first attempt at rescuing them checked
	 * for 0xF0 at offset zero -- which mid-frame bytes can never satisfy, so
	 * it could not have fired (NOTES.md §42).
	 *
	 * The receiver only listens.  Enabling it early costs nothing and means
	 * no window is ever joined late.
	 *
	 * The stock's delay_ms(5) went with it: its only job was to hold the
	 * receiver off for five milliseconds, and there is no receiver left to
	 * hold off.  Recording the removal rather than letting a dropped stock
	 * step pass unmentioned. */
	em_uart_init();

	/* Poll the frame sync while waiting, because the EIC is not armed yet
	 * and this wait is where one kind of failing boot spends four and a half
	 * seconds.  Whether the body is clocking VD during that silence is the
	 * difference between "busy elsewhere" and "not there at all". */
	{
		int      last   = em_pin_level(EM_PIN_BODY_VD);
		uint32_t t0     = millis();
		uint8_t  marked = 0;

		while (!em_pin_level(EM_PIN_BODY_CS)) {
			int v = em_pin_level(EM_PIN_BODY_VD);

			if (v && !last) {
				em_vd_edges++;
				if (!em_t_first_vd) {
					em_t_first_vd = millis();
				}
			}
			last = v;

			/* NOBODY THERE.  This loop never returns on a bench
			 * supply or the USB flasher, and the trail cannot tell
			 * that apart from a camera that cut power early -- so
			 * every dump has carried dead-looking boots that were
			 * really just the flashing session.  A camera clocks
			 * its frame sync before raising chip select; a USB
			 * port does not, so 2 s of silence says so. */
			/* A diagnostic, not a decision.  em_init waits for
			 * the body's chip select forever, as the stock's
			 * protocol_init at 0x551c does -- but a boot on a bench
			 * supply or the USB flasher waits forever too, and in
			 * the trail that is byte-for-byte what a camera cutting
			 * power early looks like.  Every dump this project has
			 * taken carries at least one, because taking the dump
			 * is one.
			 *
			 * A camera clocks its frame sync before it raises chip
			 * select; a USB port does not.  So two seconds of
			 * silence with no VD edge at all says nobody is there.
			 *
			 * It does NOT give up.  An earlier version returned
			 * early here so that a bench session could get past
			 * it and drive the motor; that turned out to be the
			 * wrong place to drive it from, since USB does not
			 * power the motor and a bench supply would not be the
			 * one the mechanism sees in service.  The bail-out
			 * bought nothing and cost a way for a slow body to be
			 * read as absent. */
			if (!marked && em_vd_edges == 0
			    && (uint32_t)(millis() - t0) >= 2000u) {
				marked = 1;
				diag_tally_mark(DIAG_MARK_NOBODY, 0);
			}
		}
	}

	/* The window has just opened.  Nothing is read from the ring while it
	 * is: the DMA position is only read with the line low (em_uart.c).
	 * Whatever the idle line left in it is skipped by the framer. */
	em_t_cs_first_high = millis();

	while (em_pin_level(EM_PIN_BODY_CS)) {
	}
	(void)em_uart_rx_count();          /* brings em_rx_bytes up to date */
	em_rx_bytes_at_close = em_rx_bytes;  /* everything since the UART came up */
	em_first_window_ms   = millis() - em_t_cs_first_high;

	/* Whatever that window held: there was no edge interrupt to parse it --
	 * the EIC is not armed yet -- so run the framer by hand, BEFORE arming
	 * it, so the two never run at once.  A normal boot's first window is an
	 * empty sync pulse and this finds nothing.  Bytes that arrive after it
	 * stay in the ring for the next edge. */
	if (em_rx_drain()) {
		em_init_frame_rescued = 1;
	}

	em_eic_init(on_cs_edge, on_vd_edge);

	delay_ms(50);
	PORT->Group[0].OUTCLR.reg = (1u << EM_PIN_LENS_CS);
	em_t_init_done = millis();
}

/* Message 0x05 pl[77..80]: the row the body asked for two cycles ago, and its
 * table value.  The interrupt is masked for the copy so a request landing in
 * between cannot split the pair. */
static void status05_row_index(void)
{
	uint8_t tag = g_row_next[1];
	uint8_t k   = (uint8_t)(tag - EM05_ROW_TBL_LO);

	em_msg_05[EM05_ROW_INDEX]     = g_row_next[0];
	em_msg_05[EM05_ROW_INDEX + 1] = tag;
	em_msg_05[EM05_ROW_INDEX + 2] = 0x00;
	em_msg_05[EM05_ROW_INDEX + 3] = (k < sizeof(g_row_tbl)) ? g_row_tbl[k] : 0x00;

	__disable_irq();
	g_row_next[0] = g_row_req[0];
	g_row_next[1] = g_row_req[1];
	__enable_irq();
}

/* The status half of em_poll, on its own. */
void em_pump_status(void)
{
	if (!g_status_due || em_shutdown_acked) {
		return;
	}
	g_status_due = 0;

	em_msg_05[4] = g_reply_seq;
	em_msg_06[4] = g_reply_seq;
	status05_row_index();

	delay_us(EM_LEAD_US);
	if (em_status05_len == EM05_LEN_LONG) {
		/* Built aside, so the 105-byte form's checksum never lands in
		 * the long form's payload. */
		static uint8_t long05[EM05_LEN_LONG];
		uint8_t i;

		for (i = 0; i < EA9_LEN_05 - 3u; i++) {
			long05[i] = em_msg_05[i];
		}
		for (; i < EM05_LEN_LONG - 3u; i++) {
			long05[i] = 0;
		}
		em_transmit(long05, EM05_LEN_LONG);
	} else {
		em_transmit(em_msg_05, EA9_LEN_05);
	}

	/* THE TAIL.  Message 0x06 is a 43-byte core plus zero or more tail
	 * blocks starting at pl[39] = frame offset 0x2D, and the frame grows to
	 * fit them (msg_0x06.md).  At most one event per frame; the query
	 * answers from one message 0x04 all go in the next 0x06.
	 *
	 * In the 48-byte form pl[39..40] ARE the checksum, so there is no way
	 * to carry a block without growing the frame and no way to grow it
	 * without carrying one. */
	scan_state_frame();
	status06_focus();

	{
		uint16_t len = EM06_LEN_IDLE;
		uint8_t  k, sent_ans;

		if (g_event_n) {
			em_msg_06[len - 3] = g_event_code[0];
			em_msg_06[len - 2] = g_event_param[0];
			len += 2;
			/* Drive's event is four bytes, not two. */
			if (g_event_code[0] == EM_REC_DRIVE) {
				em_msg_06[len - 3] = 0;
				em_msg_06[len - 2] = 0;
				len += 2;
			}
		}
		for (k = 0; k < g_ans_n && len + 3u <= EM06_LEN_MAX; k++) {
			em_msg_06[len - 3] = g_ans_tag[k];
			em_msg_06[len - 2] =
			        (uint8_t)(g_ans_val[k] & 0xFFu);
			em_msg_06[len - 1] =
			        (uint8_t)(g_ans_val[k] >> 8);
			len += 3;
		}
		sent_ans = k;

		em_transmit(em_msg_06, len);

		/* Shift the event queue AFTER the frame has gone: whatever is
		 * behind it waits for the next one. */
		if (g_event_n) {
			g_event_code[0]  = g_event_code[1];
			g_event_param[0] = g_event_param[1];
			g_event_n--;
		}
		/* Whatever did not fit WAITS rather than being dropped.  The
		 * body asked a question; answering it a frame late is a worse
		 * answer than answering it now, and no answer at all is not an
		 * answer. */
		for (k = sent_ans; k < g_ans_n; k++) {
			g_ans_tag[k - sent_ans] = g_ans_tag[k];
			g_ans_val[k - sent_ans] = g_ans_val[k];
		}
		g_ans_n = (uint8_t)(g_ans_n - sent_ans);

		if (len != EM06_LEN_IDLE && em_events_sent != 0xFFFFu) {
			em_events_sent++;
		}
	}

	if (!em_t_first_status) {
		em_t_first_status = millis();
	}
}

void em_poll(void)
{
	/* Re-entrancy: the park handler runs from here and its servo loop pumps
	 * em_poll again.  Without this the nested call would dispatch the same
	 * pending reply and park recursively.  Keep feeding the body, do not
	 * re-enter the reply path. */
	if (g_in_poll) {
		em_pump_status();
		return;
	}
	g_in_poll = 1;

	for (;;) {
		int     got = req_pop();
		uint8_t id;

		if (got < 0) {
			break;
		}
		id = (uint8_t)got;
		em_send_id(id);

		/* Message 0x0a is the last of the handshake in every capture
		 * (0x01 -> 0x07 -> 0x0b -> 0x08 -> 0x09 -> 0x0d -> 0x10 ->
		 * 0x0a); answering it is what arms the status loop. */
		if (id == 0x0A) {
			em_handshake_done = 1;
			em_t_handshake    = millis();
			if (g_woke) {
				g_woke = 0;
				diag_tally_mark(DIAG_MARK_WAKE, em_wake_first_id);
			}
		}
	}

	if (g_m10_owed && g_ready) {
		g_m10_owed = 0;
		em_send_id(EM_ID_READY);
	}

	if (g_file_pending) {
		uint8_t id, p0, p1;

		__disable_irq();
		id = g_file_id; p0 = g_file_p0; p1 = g_file_p1;
		g_file_pending = 0;
		__enable_irq();
		if (!em_shutdown_acked) {
			em_send_file(id, p0, p1);
		}
	}

	em_pump_status();

	g_in_poll = 0;
}

/* THE STANDBY WAIT (emount.h).  Entered from the power-down handler, after the
 * 0x16 reply, with our UART off and PA00 driven low. */

/* The body drives its TXD low within one 5 ms poll of seeing our RXD low
 * (msg_0x16.md, power-off step 6).  Past this it is not coming, and the wait
 * goes on regardless: what matters is the chip select. */
#define EM_SLEEP_RX_LOW_MS  200u

/* Everything that belongs to one session and not to the lens.  What the long
 * init told the body -- capabilities, identity, the 0x08 request's aperture
 * bit, the speed -- stays, because the short init does not ask again.  The
 * position state stays too: the helicoid has not moved. */
static void em_session_reset(void)
{
	__disable_irq();
	req_clear();
	g_file_pending   = 0;
	g_status_due     = 0;
	em_focus_pending = 0;
	g_query_n        = 0;
	g_event_n        = 0;
	g_ans_n          = 0;
	g_row_req[0]     = 0;
	g_row_req[1]     = 0;
	g_row_next[0]    = 0;
	g_row_next[1]    = 0;
	g_t_prev_vd      = 0;    /* the gap across the standby is not a frame */
	__enable_irq();
	em_set_scan_state(0);
}

/* The wake pin handshake, from the moment the body's chip select is seen high.
 * Returns 0 if the body never brought it down again -- a glitch on an idle
 * line rather than a body -- and the caller goes back to waiting. */
static uint8_t em_wake_handshake(void)
{
	uint32_t t;

	PORT->Group[0].OUTSET.reg = (1u << EM_PIN_LENS_CS);
	/* Our TX, the body's RXD, idles high.  This flushes the receive ring --
	 * a DMA position read -- with the body's chip select HIGH, the one place
	 * that rule is bent: the line is up for the pin handshake, and the body
	 * sends no data until that handshake is over. */
	em_uart_resume();

	for (t = 0; t < EM_0C_WAIT_US; t++) {
		if (!em_pin_level(EM_PIN_BODY_CS)) {
			delay_us(EM_0C_READY_US);
			PORT->Group[0].OUTCLR.reg = (1u << EM_PIN_LENS_CS);
			return 1;
		}
		delay_us(1);
	}

	/* Not a wake: put the line back the way the body left it. */
	em_uart_disable();
	PORT->Group[0].OUTCLR.reg = (1u << EM_PIN_TX);
	PORT->Group[0].DIRSET.reg = (1u << EM_PIN_TX);
	PORT->Group[0].PINCFG[EM_PIN_TX].reg &= ~(uint8_t)PORT_PINCFG_PMUXEN;
	PORT->Group[0].OUTCLR.reg = (1u << EM_PIN_LENS_CS);
	return 0;
}

void em_sleep_until_wake(void)
{
	uint32_t t0;
	uint8_t  rx_low = 0;

	em_eic_pause();
	em_handshake_done = 0;
	g_status_due      = 0;
	count16(&em_sleep_n);

	t0 = millis();
	while ((uint32_t)(millis() - t0) < EM_SLEEP_RX_LOW_MS) {
		if (!em_pin_level(EM_PIN_RX)) {
			rx_low = 1;
			break;
		}
	}
	diag_tally_mark(DIAG_MARK_SLEEP, rx_low);

	/* Standby, or the last microseconds before the power goes.  A RISING
	 * edge, not a level: after a glitch that never fell the line may still
	 * be high, and its eventual fall must not read as a wake.  The low half
	 * is a short spin; the wait for the rise is real sleep -- an a9 II keeps
	 * the lens powered through a camera power-off (dump_wakeup.bin), so this
	 * wait can last as long as the camera stays off. */
	do {
		while (em_pin_level(EM_PIN_BODY_CS)) {
		}
		em_eic_sleep_until_cs_high();
	} while (!em_wake_handshake());

	em_session_reset();
	em_wake_first_id  = 0;
	g_woke            = 1;
	count16(&em_wake_n);
	em_t_wake         = millis();
	em_t_last_frame   = em_t_wake;   /* or the main loop reads the standby
	                                    as the bus going quiet */
	em_shutdown_acked = 0;
	em_eic_resume();
}
