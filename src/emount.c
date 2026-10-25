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
 *    body never asked for (0x35), not by answering one it did; no handler is
 *    added here, so that failure mode is not in reach.
 *
 * 2. NO DEFERRED-CALL TIMER.  The stock 10 us / 100 us / 300 us gaps come from
 *    TC4 one-shots.  They are busy waits here, which costs at most 410 us of a
 *    16.6 ms frame and removes a timer, an interrupt and a callback slot.
 *
 * 3. BLOCKING TRANSMIT instead of DMA -- see em_uart.c.
 *
 * The frame gaps ARE reproduced, because they are the part a body can notice:
 * chip select stays asserted for 100 us after the last bit, matching the
 * 0x5188 -> 0x5280 path, and message 0x06 follows 0x05 by 300 us.
 */
#include "board.h"
#include "clock.h"
#include "delay.h"
#include "emount.h"
#include "diag.h"
#include "emount_packets.h"
#include "em_uart.h"
#include "em_eic.h"

#define EM_CLASS_INIT     0x02u
#define EM_CLASS_NORMAL   0x01u

#define EM_FRAME_START    0xF0u
#define EM_FRAME_END      0x55u
#define EM_HEADER_LEN     6        /* start, u16 length, class, seq, id */

/* Recovered timings.  EM_CS_HOLD_US is 0x5188's defer(100, 0x5281): the
 * transmit-complete callback waits 100 us before 0x5280 drops the line.
 * EM_STATUS_GAP_US is 0x5280's defer(300, msg06_sender).  EM_LEAD_US is the
 * defer(10, msg05_sender) the main loop issues after the frame sync. */
#define EM_CS_HOLD_US     100u
#define EM_STATUS_GAP_US  300u
#define EM_LEAD_US        10u

/* Frame offsets into the message 0x06 packet that carry the focus position,
 * from the frame-sync callback at 0x5120.  Both get the same value; the stock
 * firmware sends no forecast in the first field (EA9.md §4.3). */
#define EM06_POS_A        0x08
#define EM06_POS_B        0x1A

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

/* The current aperture, 256 x AV + 4096: message 0x05 payload[0..1] (fixed,
 * in the template -- there is no iris) and message 0x28 payload[9..10], which
 * follows it only when the body asks (set_28_aperture). */
#define EM05_AP_NOW_A     0x06
#define EM28_AP_NOW       0x0F

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
#define EM05_POS_COARSE   0x3C          /* pl[54] counts from infinity / 256 */
#define EM05_RING_DIR     0x42          /* pl[60] focus ring direction      */
#define EM05_FLAG_REPORTS 0x80u
#define EM05_FLAG_MOVING  0x40u

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
volatile uint16_t em_aperture_req;
volatile uint16_t em_m03_aperture;
volatile uint16_t em_m03_n;
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
volatile uint32_t em_rx_bytes_at_open;
volatile uint32_t em_rx_bytes_at_close;
volatile uint32_t em_first_window_ms;
volatile uint32_t em_cs_edges;
volatile uint32_t em_vd_edges;
volatile uint32_t em_t_first_vd;
volatile uint16_t em_frame_period_ms = EM_FRAME_PERIOD_DEFAULT;
volatile uint16_t em_frame_period_n;
static volatile uint32_t g_t_prev_vd;
static volatile uint32_t g_t_vd_us;    /* micros() at the last frame sync */
volatile uint32_t em_id_mask_lo;
volatile uint32_t em_id_mask_hi;
volatile uint8_t  em_id_log[EM_ID_LOG_LEN];
volatile uint8_t  em_id_log_n;
volatile uint8_t  em_norm_len[EM_NORM_SLOTS];
volatile uint8_t  em_norm_id[EM_NORM_SLOTS];
volatile uint16_t em_norm_count[EM_NORM_SLOTS];
volatile uint8_t  em_norm_used;
volatile uint8_t  em_norm_overflow;
volatile uint16_t em_truncated;
volatile uint8_t  em_window_len[4];
volatile uint8_t  em_window_used;
volatile struct em_capture em_cap[EM_CAP_SLOTS];
volatile uint8_t  em_cap_used;
volatile struct em_capture em_pre[EM_PRE_SLOTS];
volatile uint8_t  em_pre_used;

static volatile uint8_t  g_reply_pending;
static volatile uint8_t  g_reply_id;
static volatile uint8_t  g_status_due;
static volatile uint8_t  g_reply_seq;

/* Three replies are built from the request: 0x0A and 0x16 echo it whole,
 * 0x34 copies two bytes of it.  The reply goes out from em_poll, by which time
 * the body may already have opened another window and reset the receive
 * buffer -- so the request is snapshotted here at parse time, in the
 * interrupt.  32 covers the longest of them, the 25-byte 0x0A. */
#define EM_REQ_SNAP 32
static volatile uint8_t  g_req[EM_REQ_SNAP];

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
static void em_transmit(uint8_t *frame, uint16_t len)
{
	uint16_t sum = 0;

	frame[1]       = (uint8_t)len;
	frame[len - 1] = EM_FRAME_END;

	/* sum over frame[1] .. frame[len-4], i.e. everything but the start
	 * byte, the checksum itself and the terminator. */
	for (uint16_t i = 1; i <= (uint16_t)(len - 4); i++) {
		sum += frame[i];
	}
	frame[len - 3] = (uint8_t)(sum & 0xFFu);
	frame[len - 2] = (uint8_t)(sum >> 8);

	PORT->Group[0].OUTSET.reg = (1u << EM_PIN_LENS_CS);
	em_uart_send(frame, len);

	delay_us(EM_CS_HOLD_US);
	PORT->Group[0].OUTCLR.reg = (1u << EM_PIN_LENS_CS);

	em_frames_tx++;
}

/* The shutdown acknowledgement: send the body's own frame straight back.
 *
 * That is literally what the stock does -- transmit(rx_buffer, rx[1]) -- so
 * the reply is the request with a recomputed checksum.
 *
 * What is NOT reproduced, and why:
 *   park     the stock moves to a stored position first.  We have no parking
 *            policy yet and inventing one would move the mechanism at shutdown
 *            for no reason we can justify.
 *   PA23     the stock drives it high here.  PA23 is also the bootloader's
 *            stay-in-bootloader input (EA9.md §11.4) and what driving it means
 *            at this point is not understood.  Not replicating an unexplained
 *            pin write on the shutdown path.
 *   halt     the stock disables the bus and sleeps.  Going quiet is enough for
 *            the body, and staying alive means a 0x16 that turns out not to be
 *            a shutdown cannot strand us -- the next init-class request revives
 *            the status loop.
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
	uint8_t echo[EM_REQ_SNAP];
	uint16_t n = g_req[1];
	uint16_t i;
	uint8_t already = em_shutdown_acked;

	/* Park first, acknowledge second -- the order the stock uses.  Snapshot
	 * the request before parking: the park takes long enough for the body
	 * to open another receive window and overwrite g_req. */
	if (n >= EM_HEADER_LEN && n <= EM_REQ_SNAP) {
		for (i = 0; i < n; i++) {
			echo[i] = g_req[i];
		}
	}

	/* A repeated 0x16 gets its echo -- the body may simply not have heard
	 * the first -- but the park and the power-down signal happen once. */
	if (!em_shutdown_acked && g_park) {
		g_park();
	}

	/* Unconditionally, even if the park stalled: the echo is what releases
	 * the body, and a lens stuck mid-travel is a far smaller problem than a
	 * camera that will not switch off. */
	if (n < EM_HEADER_LEN || n > EM_REQ_SNAP) {
		return;
	}
	em_transmit(echo, n);

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
	 * and can still be woken. */
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

	if (!p) {
		return;
	}
	frame = p->frame;

	/* The one reply with request-dependent content: message 0x34 echoes
	 * the request's pl[0..1] into its own pl[1..2]. */
	if (id == 0x34u) {
		frame[7] = g_req[6];
		frame[8] = g_req[7];
	}

	/* Message 0x0C: the template answers 01, and the switch to 1.5 Mbaud
	 * follows the reply.  Nothing switches back: the bus is disabled at
	 * shutdown and the next boot starts at 750. */
	if (id == EM_ID_BAUD) {
		em_transmit(frame, p->len);
		em_baud_switch();
		return;
	}

	em_transmit(frame, p->len);
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

/* Publish one instruction.  A newer instruction REPLACES an unread one: the
 * protocol says a new one replaces a running one, so it certainly replaces one
 * that has not started (autofocus.md 3.1 rule 4). */
static void focus_cmd_set(const struct em_focus_cmd *c)
{
	g_cmd = *c;
	em_focus_pending = 1;
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
			case 4:
				c.op    = EM_OP_MOVE_REL;
				c.delta = (int16_t)v;
				break;
			case 6:
				c.op    = EM_OP_MOVE_DEFOCUS;
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
			/* The optical table's row selector.  This adapter has
			 * no table, so there is nothing to select. */
			rec_count(EM_REC_SLOT_ROW);
			break;

		default:
			rec_count(EM_REC_SLOT_OTHER);
			break;
		}
	}
}

/* The body's chip select, both edges (0x53a4).  Rising arms the receiver;
 * falling means a whole frame has landed. */
static void on_cs_edge(int level)
{
	const uint8_t *rx;
	uint16_t n;

	em_cs_edges++;

	if (level) {
		em_uart_rx_reset();
		return;
	}

	rx = em_uart_rx_buf();
	n  = em_uart_rx_len();
	if (n < EM_HEADER_LEN || rx[0] != EM_FRAME_START) {
		if (n) {
			em_bad_frames++;
		}
		return;
	}
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

	for (uint16_t i = 0; i < EM_REQ_SNAP; i++) {
		g_req[i] = (i < n) ? rx[i] : 0;
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

	/* Genuine truncation only -- a window shorter than the frame in it. */
	if ((n & 0xFF) < rx[1]) {
		em_truncated++;
	}
	{
		uint8_t j;

		for (j = 0; j < em_window_used; j++) {
			if (em_window_len[j] == (uint8_t)n) {
				break;
			}
		}
		if (j == em_window_used && j < 4) {
			em_window_len[j] = (uint8_t)n;
			em_window_used++;
		}
	}

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
		if (rx[1] >= 11u && rx[6] == 0xFFu && rx[7] == 0x7Fu) {
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

	if (rx[3] == EM_CLASS_INIT) {
		em_last_id      = rx[5];
		g_reply_id      = rx[5];
		g_reply_pending = 1;
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

/* The body's frame sync, rising edge (0x5120).  Once the handshake is over
 * this is what clocks the status pair at the body's frame rate. */
static void on_vd_edge(void)
{
	uint32_t now = millis();

	g_t_vd_us = micros();
	em_vd_edges++;
	if (!em_t_first_vd) {
		em_t_first_vd = now;
	}

	/* Time the body's polling.  A native lens measures this rather than
	 * assuming it, and only accepts a delta inside the band that reads as a
	 * real frame rate -- so a missed edge, a burst of two, or the long gap
	 * across a shutdown cannot poison the period (msg_0x06.md).  Everything
	 * one frame ahead is scaled by this number. */
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

void em_set_motion(uint8_t moving, uint8_t dir, int32_t pos)
{
	uint8_t v = moving ? EM06_MOVING : EM06_AT_REST;
	uint16_t lo = (uint16_t)em_msg_06[EM06_LIMIT_LO]
	              | ((uint16_t)em_msg_06[EM06_LIMIT_LO + 1] << 8);
	uint16_t hi = (uint16_t)em_msg_06[EM06_LIMIT_HI]
	              | ((uint16_t)em_msg_06[EM06_LIMIT_HI + 1] << 8);

	/* Read the limits back out of the frame rather than from a constant:
	 * they are what the body clamps against, and a status byte derived from
	 * a different number would contradict the frame carrying it. */
	if (pos <= (int32_t)lo + EM06_NEAR_LIMIT) {
		v |= EM06_NEAR_LO;
	}
	if (pos >= (int32_t)hi - EM06_NEAR_LIMIT) {
		v |= EM06_NEAR_HI;
	}
	em_msg_06[EM06_MOTION]    = v;
	em_msg_06[EM06_DIRECTION] = dir;
}

/* Message 0x06 payload[32..38]: the position trace WITHIN the last frame, as
 * Yongnuo 50 DF builds it.  Its frame-sync handler (FUN_08015f80) latches the
 * position into L[0] and starts a timer that samples L[1..7] at every eighth
 * of the frame period (0x08016020: 40000 ticks at 20 ms frames, 33280 at
 * 16.7); the next frame sync takes L[8] and snapshots the lot, and the status
 * writer (FUN_080160a4) sends pl[32+k] = L[k+1] - L[k+2], k = 0..6 -- OLDER
 * minus newer, one eighth of a frame each.
 *
 * Here the samples are taken from the caller's loop, which reads the encoder
 * anyway, at the first call past each eighth: within one loop iteration of
 * the instant Yongnuo samples at. */
#define SUB_N 8
static uint16_t g_sub[SUB_N + 1];
static uint8_t  g_sub_idx;        /* next slot to fill, 1..SUB_N-1 */
static uint8_t  g_sub_valid;      /* g_sub[0] holds a real latch */
static uint32_t g_sub_vd;         /* em_vd_edges at that latch */

static uint8_t sat8(int32_t v)
{
	if (v > 127) {
		v = 127;
	} else if (v < -128) {
		v = -128;
	}
	return (uint8_t)(int8_t)v;
}

void em_sample_position(int32_t pos)
{
	uint32_t vd = em_vd_edges;
	uint16_t p  = clamp_travel(pos);

	if (vd != g_sub_vd) {
		uint8_t *h = &em_msg_06[EM06_VEL_HIST];
		uint8_t  k;

		/* A frame whose eighths were not all reached (a blocking step
		 * in between) has its missing ones read as where it ended. */
		while (g_sub_idx < SUB_N) {
			g_sub[g_sub_idx++] = p;
		}
		g_sub[SUB_N] = p;
		for (k = 0; k < EM06_VEL_HIST_N; k++) {
			h[k] = g_sub_valid
			       ? sat8((int32_t)g_sub[k + 1] - (int32_t)g_sub[k + 2])
			       : 0u;
		}
		g_sub[0]    = p;
		g_sub_idx   = 1;
		g_sub_valid = 1;
		g_sub_vd    = vd;
		return;
	}

	if (g_sub_idx < SUB_N && em_frame_period_ms) {
		uint32_t due = (uint32_t)g_sub_idx * em_frame_period_ms
		               * 1000u / SUB_N;

		if ((uint32_t)(micros() - g_t_vd_us) >= due) {
			g_sub[g_sub_idx++] = p;
		}
	}
}

void em_set_subject_distance(uint16_t code, uint8_t coarse, uint8_t coarse_pos)
{
	put16(em_msg_05, EM05_DISTANCE, code);
	em_msg_05[EM05_DIST_COARSE] = coarse;
	em_msg_05[EM05_POS_COARSE]  = coarse_pos;
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

void em_set_focus_position(int32_t pos, int32_t ahead)
{
	uint16_t now_v   = clamp_travel(pos);
	uint16_t ahead_v = clamp_travel(ahead);

	/* payload[2..3] is the position ONE FRAME AHEAD, payload[20..21] the
	 * position now.  The stock writes the same value to both -- it has no
	 * forecast to offer -- and so did this, until the caller was given one.
	 * Equal values remain correct and are what a stationary lens sends. */
	em_msg_06[EM06_POS_A]     = (uint8_t)(ahead_v & 0xFFu);
	em_msg_06[EM06_POS_A + 1] = (uint8_t)(ahead_v >> 8);
	em_msg_06[EM06_POS_B]     = (uint8_t)(now_v & 0xFFu);
	em_msg_06[EM06_POS_B + 1] = (uint8_t)(now_v >> 8);
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

/* Message 0x28 pl[9..10] carries the current aperture only when the body's
 * 0x08 request asks for it (emount.h); otherwise zero.  Filling it unasked
 * made an a9 II re-meter after every exposure. */
static void set_28_aperture(void)
{
	uint16_t av = (uint16_t)em_msg_05[EM05_AP_NOW_A]
	              | ((uint16_t)em_msg_05[EM05_AP_NOW_A + 1] << 8);

	put16(em_msg_28, EM28_AP_NOW, em_m08_aperture_on ? av : 0u);
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

void em_init(void)
{
	/* Fixed content is in the templates (emount_packets.c). */

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

	/* The window has just opened.  Drop anything picked up from the idle
	 * line beforehand, so what follows is this window and nothing else. */
	em_rx_bytes_at_open = em_rx_bytes;
	em_uart_rx_reset();
	em_t_cs_first_high = millis();

	while (em_pin_level(EM_PIN_BODY_CS)) {
	}
	em_rx_bytes_at_close = em_rx_bytes;
	em_first_window_ms   = millis() - em_t_cs_first_high;

	em_eic_init(on_cs_edge, on_vd_edge);

	/* Whatever that window held is a real frame, captured from its start.
	 * There was no falling edge to parse it -- the EIC was not armed yet --
	 * so run the falling-edge path by hand.  A normal boot's first window is
	 * an empty sync pulse and this does nothing. */
	if (em_uart_rx_len() >= EM_HEADER_LEN
	    && em_uart_rx_buf()[0] == EM_FRAME_START) {
		em_init_frame_rescued = 1;
		on_cs_edge(0);
	} else {
		em_uart_rx_reset();
	}

	delay_ms(50);
	PORT->Group[0].OUTCLR.reg = (1u << EM_PIN_LENS_CS);
	em_t_init_done = millis();
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
	delay_us(EM_STATUS_GAP_US);

	/* THE TAIL.  Message 0x06 is a 43-byte core plus zero or more tail
	 * blocks starting at pl[39] = frame offset 0x2D, and the frame grows to
	 * fit them (msg_0x06.md).  At most one event per frame; the query
	 * answers from one message 0x04 all go in the next 0x06.
	 *
	 * In the 48-byte form pl[39..40] ARE the checksum, so there is no way
	 * to carry a block without growing the frame and no way to grow it
	 * without carrying one. */
	scan_state_frame();

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

	if (g_reply_pending) {
		uint8_t id = g_reply_id;

		g_reply_pending = 0;
		em_send_id(id);

		/* Message 0x0a is the last of the handshake in every capture
		 * (0x01 -> 0x07 -> 0x0b -> 0x08 -> 0x09 -> 0x0d -> 0x10 ->
		 * 0x0a); answering it is what arms the status loop. */
		if (id == 0x0A) {
			em_handshake_done = 1;
			em_t_handshake    = millis();
		}
	}

	em_pump_status();

	g_in_poll = 0;
}
