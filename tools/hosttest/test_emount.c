/* test_emount.c -- run the real emount.c against scripted body traffic.
 *
 * This compiles src/emount.c and src/emount_packets.c unmodified and drives
 * them through the shim, so what is being tested is the code that ships, not
 * a model of it.  Cheap insurance: the alternative is discovering a wrong
 * checksum or a missed handshake id on a camera, where the only feedback is
 * that the body cut power.
 */
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "emount.h"
#include "emount_packets.h"

extern uint8_t  sent[][256];
extern uint16_t sent_len[];
extern int      sent_cs_asserted[];
extern int      sent_n;
extern int      sent_overflow;
extern int      host_pad_to;
extern uint32_t host_millis;
void host_body_frame(const uint8_t *frame, uint16_t len);
void host_frame_sync(void);
int  host_cs_released(void);

static int failures;

/* Stand-in for the motor build's park handler. */
static int park_calls;
static int park_sent_before;     /* frames already sent when the park ran */
static int park_pumps;           /* park calls em_poll, as the servo's pump does */
static int park_inject_0x16;     /* body sends another 0x16 while we park */
static uint16_t make_request(uint8_t *buf, uint8_t cls, uint8_t seq, uint8_t id,
                             uint8_t a0, uint8_t a1);

static void fake_park(void)
{
	park_calls++;
	park_sent_before = sent_n;

	/* A park takes ~150 ms, during which the body keeps sending -- about
	 * nine frames.  If one of them is another 0x16 and the nested em_poll
	 * dispatches it, the park recurses.  That is the case the re-entrancy
	 * guard exists for, and it only happens if the test delivers a frame. */
	if (park_inject_0x16) {
		uint8_t r2[64];
		uint16_t n2 = make_request(r2, 0x02, 0x99, EM_ID_SHUTDOWN, 0, 0);

		park_inject_0x16 = 0;
		host_body_frame(r2, n2);
	}
	while (park_pumps--) {
		em_poll();               /* must NOT re-enter the reply path */
	}
}


static void fail(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	fputs("FAIL: ", stdout);
	vprintf(fmt, ap);
	putchar('\n');
	va_end(ap);
	failures++;
}

static uint16_t checksum(const uint8_t *f, uint16_t len)
{
	uint16_t s = 0;
	for (uint16_t i = 1; i <= (uint16_t)(len - 4); i++) {
		s += f[i];
	}
	return s;
}

/* Build an init-class request for one message id, the shape a body sends.
 * a0/a1 land at payload[0]/payload[1] -- frame offsets 6 and 7, which are the
 * bytes the 0x1b and 0x34 handlers echo back. */
static uint16_t make_request(uint8_t *buf, uint8_t cls, uint8_t seq, uint8_t id,
                             uint8_t a0, uint8_t a1)
{
	uint16_t len = 10;
	memset(buf, 0, len);
	buf[0] = 0xF0;
	buf[1] = (uint8_t)len;
	buf[2] = 0;
	buf[3] = cls;
	buf[4] = seq;
	buf[5] = id;
	buf[6] = a0;
	buf[7] = a1;
	buf[len - 1] = 0x55;
	return len;
}

/* A message 0x1B focus command.  Built by hand rather than with
 * make_request(), which emits 10-byte frames: 0x1B needs TWO payload bytes, so
 * 6 header + 2 + 3 trailer = 11.  In a 10-byte frame, byte 7 is a checksum
 * byte, and reading it as the target's high byte is exactly the bug the length
 * guard exists to stop. */
static int aperture_cmd(uint8_t seq, uint16_t av)
{
	uint8_t f[32];
	uint16_t n = 11;
	int before = sent_n;

	memset(f, 0, sizeof(f));
	f[0] = 0xF0;
	f[1] = (uint8_t)n;
	f[3] = 0x02;
	f[4] = seq;
	f[5] = EM_ID_APERTURE;
	f[6] = (uint8_t)(av & 0xFF);
	f[7] = (uint8_t)(av >> 8);
	f[n - 1] = 0x55;
	host_body_frame(f, n);
	em_poll();
	return (sent_n == before + 1) ? before : -1;
}

/* Ask for one id and return the index of the frame it produced, or -1. */
static int request(uint8_t id, uint8_t seq, uint8_t a0, uint8_t a1)
{
	uint8_t req[64];
	int before = sent_n;
	uint16_t len = make_request(req, 0x02, seq, id, a0, a1);

	host_body_frame(req, len);
	em_poll();
	return (sent_n == before + 1) ? before : -1;
}

static void check_frame(int i, uint8_t id, const char *what)
{
	const uint8_t *f = sent[i];
	uint16_t len = sent_len[i];
	uint16_t cs;

	if (f[0] != 0xF0) {
		fail("%s: frame %d does not start 0xF0", what, i);
	}
	if (f[1] != (len & 0xFF)) {
		fail("%s: length byte %#04x but %d bytes sent", what, f[1], len);
	}
	if (f[5] != id) {
		fail("%s: expected id %#04x, got %#04x", what, id, f[5]);
	}
	if (f[len - 1] != 0x55) {
		fail("%s: frame does not end 0x55", what);
	}
	cs = checksum(f, len);
	if (f[len - 3] != (cs & 0xFF) || f[len - 2] != (cs >> 8)) {
		fail("%s: checksum %#06x but frame carries %#06x",
		     what, cs, f[len - 3] | (f[len - 2] << 8));
	}
	if (!sent_cs_asserted[i]) {
		fail("%s: sent with our chip select low", what);
	}
}

int main(void)
{
	static const uint8_t HANDSHAKE[] = { 0x01, 0x07, 0x0B, 0x08,
	                                     0x09, 0x0D, 0x10, 0x0A };
	uint8_t req[64];
	uint16_t len;
	int i;

	em_init();

	/* --- the version telltale ---------------------------------------- */
	if (ea9_data[EA9_OFF_07 + 12] != EM_FW_VERSION_BCD) {
		fail("em_init did not write the version byte");
	}
	{
		uint16_t lid = ea9_data[EA9_OFF_07 + 15]
		               | ((uint16_t)ea9_data[EA9_OFF_07 + 16] << 8);

		if (lid != EM_LENS_ID) {
			fail("lens ID is %u, expected %u", lid, EM_LENS_ID);
		}
	}

	/* --- the init handshake ------------------------------------------ */
	for (i = 0; i < (int)sizeof(HANDSHAKE); i++) {
		uint8_t id = HANDSHAKE[i];
		int before = sent_n;

		len = make_request(req, 0x02, (uint8_t)(0x40 + i), id, 0, 0);
		host_body_frame(req, len);
		em_poll();

		if (sent_n != before + 1) {
			fail("id %#04x: %d frames sent, expected 1",
			     id, sent_n - before);
			continue;
		}
		check_frame(before, id, "handshake");
		if (!host_cs_released()) {
			fail("id %#04x: chip select left asserted after the reply",
			     id);
		}
		if (sent_len[before] != ea9_packets[0].len && sent_len[before] < 10) {
			fail("id %#04x: implausible reply length %d", id,
			     sent_len[before]);
		}
	}
	if (!em_handshake_done) {
		fail("answering message 0x0a did not complete the handshake");
	}
	if (em_frames_rx != sizeof(HANDSHAKE)) {
		fail("counted %u frames, expected %u",
		     (unsigned)em_frames_rx, (unsigned)sizeof(HANDSHAKE));
	}

	/* The reply to 0x07 must carry the version we report, and its checksum
	 * must cover it -- this is the whole point of the build. */
	for (i = 0; i < sent_n; i++) {
		if (sent[i][5] == 0x07 && sent[i][12] != EM_FW_VERSION_BCD) {
			fail("the 0x07 reply carries version %#04x", sent[i][12]);
		}
	}

	/* --- the status pair --------------------------------------------- */
	{
		int before = sent_n;

		len = make_request(req, 0x01, 0x7E, 0x04, 0, 0);
		host_body_frame(req, len);
		em_poll();
		if (sent_n != before) {
			fail("a normal-class frame drew %d replies",
			     sent_n - before);
		}

		host_frame_sync();
		em_poll();
		if (sent_n != before + 2) {
			fail("frame sync sent %d frames, expected the 0x05/0x06 pair",
			     sent_n - before);
		} else {
			check_frame(before,     0x05, "status 0x05");
			check_frame(before + 1, 0x06, "status 0x06");
			if (sent[before][4] != 0x7F || sent[before + 1][4] != 0x7F) {
				fail("status seq is %#04x/%#04x, expected 0x7f "
				     "(body seq 0x7e + 1)",
				     sent[before][4], sent[before + 1][4]);
			}
			if (sent_len[before] != EA9_LEN_05
			    || sent_len[before + 1] != EA9_LEN_06) {
				fail("status lengths %d/%d, expected %d/%d",
				     sent_len[before], sent_len[before + 1],
				     EA9_LEN_05, EA9_LEN_06);
			}
		}
	}

	/* --- the frame edits the stock handlers make --------------------- */
	/* Driven straight off ea9_fixups so it cannot drift from the generator.
	 * This is the test that was missing when message 0x0c went out with the
	 * stored payload[0] = 1 and a real body stopped answering. */
	{
		const uint8_t A0 = 0x5A, A1 = 0xC3;
		unsigned k;
		int checked = 0;

		for (k = 0; k < EA9_FIXUP_COUNT; k++) {
			const struct ea9_fixup *f = &ea9_fixups[k];
			uint8_t want;
			int idx;

			/* Message 0x1B's RX fixups are DELIBERATELY not applied:
			 * its reply carries this device's own aperture, not the
			 * requester's.  Asserted on its own below, so exempting
			 * it here hides nothing. */
			if (f->id == EM_ID_APERTURE && f->kind == EA9_FIXUP_RX) {
				continue;
			}
			want = (f->kind == EA9_FIXUP_RX)
			       ? (f->val == 6 ? A0 : f->val == 7 ? A1 : 0)
			       : f->val;
			idx = request(f->id, 0x20, A0, A1);

			if (idx < 0) {
				fail("id %#04x: no reply, cannot check its fixups",
				     f->id);
				continue;
			}
			if (sent[idx][f->off] != want) {
				fail("id %#04x: frame[%d] is %#04x, the stock "
				     "handler writes %#04x",
				     f->id, f->off, sent[idx][f->off], want);
			}
			check_frame(idx, f->id, "fixup");   /* checksum must cover it */
			checked++;
		}
		if (!checked) {
			fail("no fixups were exercised -- the generator found none?");
		}
	}

	/* --- the status frames are never sent as stored ------------------- */
	{
		const uint8_t MAXD = em_aperture_to_descriptor(EM_APERTURE_MAX);
		const uint8_t MIND = em_aperture_to_descriptor(EM_APERTURE_MIN);
		const struct { uint16_t off; uint8_t want; } PATCH[] = {
			{ EA9_OFF_06 + 0x20, 0x6C },
			{ EA9_OFF_06 + 0x21, 0x02 }, { EA9_OFF_06 + 0x22, 0x06 },
			{ EA9_OFF_06 + 0x23, 0x20 },
			/* The three message 0x05 bytes the stock patches here
			 * are the aperture descriptor, and this build writes
			 * them from its own declaration instead -- asserted
			 * with the rest of the block further down. */
			{ EA9_OFF_05 + 0x32, MAXD },
			{ EA9_OFF_05 + 0x39, MAXD },
			{ EA9_OFF_05 + 0x3A, MIND },
			/* The focal-derived step, now from the ladder rather
			 * than a literal: 52 mm falls in the 50..69 band. */
			{ EA9_OFF_06 + 0x13, 0x08 },
		};
		unsigned k;

		for (k = 0; k < sizeof(PATCH) / sizeof(PATCH[0]); k++) {
			if (ea9_data[PATCH[k].off] != PATCH[k].want) {
				fail("status patch at .data+%#05x is %#04x, "
				     "expected %#04x", PATCH[k].off,
				     ea9_data[PATCH[k].off], PATCH[k].want);
			}
		}
	}

	/* --- the focus position pair -------------------------------------- */
	em_set_focus_position(5000, 5000);
	if (ea9_data[EA9_OFF_06 + 0x08] != (5000 & 0xFF)
	    || ea9_data[EA9_OFF_06 + 0x09] != (5000 >> 8)
	    || ea9_data[EA9_OFF_06 + 0x1A] != (5000 & 0xFF)
	    || ea9_data[EA9_OFF_06 + 0x1B] != (5000 >> 8)) {
		fail("em_set_focus_position did not write both fields");
	}

	/* The two fields are now/one-frame-ahead, NOT duplicates.  Written to
	 * the wrong way round they would still both be "populated", which is
	 * exactly the check the old test made and the reason it could not have
	 * caught a swap. */
	em_set_focus_position(5000, 5100);
	if (ea9_data[EA9_OFF_06 + 0x1A] != (5000 & 0xFF)
	    || ea9_data[EA9_OFF_06 + 0x1B] != (5000 >> 8)) {
		fail("pl[20:22] must carry the position NOW");
	}
	if (ea9_data[EA9_OFF_06 + 0x08] != (5100 & 0xFF)
	    || ea9_data[EA9_OFF_06 + 0x09] != (5100 >> 8)) {
		fail("pl[2:4] must carry the position one frame AHEAD");
	}

	em_set_focus_position(99999, 99999);
	if (ea9_data[EA9_OFF_06 + 0x08] != (ENC_TRAVEL_HI & 0xFF)
	    || ea9_data[EA9_OFF_06 + 0x1A] != (ENC_TRAVEL_HI & 0xFF)) {
		fail("em_set_focus_position did not clamp to the advertised travel");
	}
	/* The forecast is clamped on its own account: it is the one that runs
	 * ahead of the mechanism and so the one that can leave the travel. */
	em_set_focus_position(5000, 99999);
	if (ea9_data[EA9_OFF_06 + 0x08] != (ENC_TRAVEL_HI & 0xFF)
	    || ea9_data[EA9_OFF_06 + 0x09] != (ENC_TRAVEL_HI >> 8)) {
		fail("the forecast field was not clamped");
	}

	/* --- the focal length, from ONE variable --------------------------
	 * The stock table disagrees with itself here -- 400 in message 0x05,
	 * 500 in message 0x28 -- so the check that matters is that every field
	 * now carries the SAME number, not that any one of them is right. */
	{
		static const uint16_t FOCAL[] = {
			EA9_OFF_05 + 0x1E, EA9_OFF_05 + 0x20,
			EA9_OFF_28 + 0x0A, EA9_OFF_28 + 0x0C,
		};
		unsigned k;

		for (k = 0; k < sizeof(FOCAL) / sizeof(FOCAL[0]); k++) {
			uint16_t v = (uint16_t)ea9_data[FOCAL[k]]
			             | ((uint16_t)ea9_data[FOCAL[k] + 1] << 8);

			if (v != EM_FOCAL_MM10) {
				fail("focal field %u reads %u, expected %u",
				     k, v, EM_FOCAL_MM10);
			}
		}

		/* And it must actually be driven by the variable, not merely
		 * happen to agree with it once. */
		em_set_focal_length(1234);
		for (k = 0; k < sizeof(FOCAL) / sizeof(FOCAL[0]); k++) {
			uint16_t v = (uint16_t)ea9_data[FOCAL[k]]
			             | ((uint16_t)ea9_data[FOCAL[k] + 1] << 8);

			if (v != 1234) {
				fail("focal field %u did not follow "
				     "em_set_focal_length", k);
			}
		}

		/* The 0x567c ladder, at every boundary it has.  A step size
		 * that disagrees with the focal length in the same build is the
		 * kind of inconsistency this whole change exists to remove. */
		{
			static const struct { uint16_t mm10; uint8_t step; }
			LADDER[] = {
				{ 1234, 0x06 }, {  700, 0x06 }, {  699, 0x08 },
				{  520, 0x08 }, {  500, 0x08 }, {  499, 0x0A },
				{  400, 0x0A }, {  350, 0x0A }, {  349, 0x0E },
				{  240, 0x0E }, {  239, 0x10 }, {  160, 0x10 },
				{  159, 0x14 }, {  100, 0x14 },
			};
			unsigned j;

			for (j = 0; j < sizeof(LADDER) / sizeof(LADDER[0]); j++) {
				em_set_focal_length(LADDER[j].mm10);
				if (ea9_data[EA9_OFF_06 + 0x13] != LADDER[j].step) {
					fail("focal %u mm10 gave step %#04x, "
					     "expected %#04x", LADDER[j].mm10,
					     ea9_data[EA9_OFF_06 + 0x13],
					     LADDER[j].step);
				}
			}
		}
		em_set_focal_length(EM_FOCAL_MM10);
	}

	/* --- the optical rows, from three constants -----------------------
	 * The stock sends no grid at all (slot A is six zero bytes, which is
	 * not a row), a pupil row inherited from the Canon clone, and a slot B
	 * in message 0x28 that is the WRONG TYPE for that field.  Checked
	 * here: every field carries what its constant says, each field gets
	 * the row type that field is supposed to carry, and the fields FOLLOW
	 * the setter rather than happening to agree once. */
	{
		static const struct {
			uint16_t off;        /* into ea9_data           */
			uint8_t  type;       /* what bit 7 must say     */
			const char *name;
		} ROW[] = {
			{ EA9_OFF_05 + 0x26, 1, "message 0x05 slot A" },
			{ EA9_OFF_05 + 0x2C, 0, "message 0x05 slot B" },
			{ EA9_OFF_28 + 0x11, 1, "message 0x28 slot A" },
			{ EA9_OFF_28 + 0x17, 1, "message 0x28 slot B" },
		};
		static const uint8_t A[EM_ROW_BYTES]  = EM_SLOT_A;
		static const uint8_t B0[EM_ROW_BYTES] = EM_SLOT_B_TYPE0;
		static const uint8_t B1[EM_ROW_BYTES] = EM_SLOT_B_TYPE1;
		/* Distinct in every byte, so a field wired to the wrong source
		 * cannot pass by looking like its neighbour. */
		static const uint8_t OTHER_A[EM_ROW_BYTES] = {
			0xB1, 0x11, 0x22, 0x33, 0x44, 0x55
		};
		static const uint8_t OTHER_B0[EM_ROW_BYTES] = {
			0x24, 0x66, 0x77, 0x01, 0x02, 0x03
		};
		static const uint8_t OTHER_B1[EM_ROW_BYTES] = {
			0xD1, 0x88, 0x99, 0x04, 0x05, 0x06
		};
		const uint8_t *want[4], *then[4];
		unsigned r, k;

		want[0] = A;  want[1] = B0; want[2] = A;  want[3] = B1;
		then[0] = OTHER_A;  then[1] = OTHER_B0;
		then[2] = OTHER_A;  then[3] = OTHER_B1;

		for (r = 0; r < 4; r++) {
			for (k = 0; k < EM_ROW_BYTES; k++) {
				if (ea9_data[ROW[r].off + k] != want[r][k]) {
					fail("%s byte %u is %#04x, expected "
					     "%#04x", ROW[r].name, k,
					     ea9_data[ROW[r].off + k],
					     want[r][k]);
				}
			}
			/* Bit 7 of byte 0 is the row type.  Get it wrong and
			 * the body is handed the right numbers as the wrong
			 * physical quantity, which no value check catches. */
			if (!!(ea9_data[ROW[r].off] & 0x80) != ROW[r].type) {
				fail("%s is not a type %u row (byte 0 %#04x)",
				     ROW[r].name, ROW[r].type,
				     ea9_data[ROW[r].off]);
			}
		}

		em_set_optical_rows(OTHER_A, OTHER_B0, OTHER_B1);
		for (r = 0; r < 4; r++) {
			for (k = 0; k < EM_ROW_BYTES; k++) {
				if (ea9_data[ROW[r].off + k] != then[r][k]) {
					fail("%s did not follow "
					     "em_set_optical_rows (byte %u)",
					     ROW[r].name, k);
				}
			}
		}
		em_set_optical_rows(A, B0, B1);
	}

	/* Message 0x05 payload[4] is the APERTURE settle countdown -- how many
	 * status frames the iris still needs to reach the aperture the body
	 * commanded.  Not a focus field.  This adapter has no iris, so the
	 * stored 0 is already the right answer and nothing writes it.
	 */

	/* pl[8] bit 0 means "aperture at rest", and the stored 0x07 has it set,
	 * which is what a lens whose iris never moves should say. */
	if (ea9_data[EA9_OFF_05 + 0x0E] != 0x07) {
		fail("message 0x05 pl[8] was disturbed");
	}

	/* --- the normal-class tally -------------------------------------- */
	{
		uint8_t req[64];
		unsigned k;
		int seen22 = -1, seen27 = -1;

		/* Two kinds, repeated: the frame clock and a focus command. */
		for (k = 0; k < 5; k++) {
			uint16_t len = make_request(req, 0x01, (uint8_t)k, 0x04, 0, 0);
			host_body_frame(req, len);
			em_poll();
		}
		/* A 27-byte frame: the length the stock firmware treats as a
		 * two-target focus command (EA9.md 2.2), never yet captured. */
		{
			uint8_t cmd[27];
			memset(cmd, 0, sizeof(cmd));
			cmd[0] = 0xF0; cmd[1] = 27; cmd[3] = 0x01;
			cmd[4] = 0x33; cmd[5] = 0x03; cmd[26] = 0x55;
			host_body_frame(cmd, sizeof(cmd));
			em_poll();
		}

		for (k = 0; k < em_norm_used; k++) {
			if (em_norm_len[k] == 10) seen22 = (int)k;
			if (em_norm_len[k] == 27) seen27 = (int)k;
		}
		if (seen22 < 0 || em_norm_count[seen22] != 6) {
			fail("normal-class tally: 10-byte frames counted %d, expected 6",
			     seen22 < 0 ? -1 : em_norm_count[seen22]);
		}
		if (seen27 < 0 || em_norm_count[seen27] != 1) {
			fail("normal-class tally: the 27-byte frame was not counted");
		}
		if (em_norm_used != 2) {
			fail("normal-class tally: %u kinds, expected 2", em_norm_used);
		}
		if (em_truncated != 0) {
			fail("%u truncations on well-formed frames", em_truncated);
		}

		/* The first two windows of each kind must be captured whole --
		 * counters took this as far as they could, and the focus
		 * command's contents are what the motor path needs. */
		if (em_cap_used != 3) {
			fail("captured %u windows, expected 3 (two 10-byte, "
			     "one 27-byte)", em_cap_used);
		} else {
			if (em_cap[0].declared != 10 || em_cap[0].got != 10) {
				fail("capture 0: declared %u got %u, expected 10/10",
				     em_cap[0].declared, em_cap[0].got);
			}
			if (em_cap[2].declared != 27 || em_cap[2].id != 0x03) {
				fail("capture 2: declared %u id %#04x, expected 27/0x03",
				     em_cap[2].declared, em_cap[2].id);
			}
			if (em_cap[2].data[0] != 0xF0 || em_cap[2].data[4] != 0x33) {
				fail("capture 2 did not keep the raw bytes: "
				     "%#04x %#04x", em_cap[2].data[0],
				     em_cap[2].data[4]);
			}
		}
		/* A frame that declares more than it delivers must be caught. */
		{
			uint8_t trunc[12];
			memset(trunc, 0, sizeof(trunc));
			trunc[0] = 0xF0; trunc[1] = 40; trunc[3] = 0x01; trunc[5] = 0x03;
			host_body_frame(trunc, sizeof(trunc));
			em_poll();
			if (em_truncated != 1) {
				fail("a frame declaring 40 bytes but delivering 12 "
				     "was not flagged as truncated");
			}
		}
	}

	/* --- junk on the wire -------------------------------------------- */
	{
		int before = sent_n;
		uint8_t junk[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };

		host_body_frame(junk, sizeof(junk));
		em_poll();
		if (sent_n != before) {
			fail("a frame with no 0xF0 drew a reply");
		}
		if (em_bad_frames != 1) {
			fail("bad frame not counted (%u)", (unsigned)em_bad_frames);
		}
	}

	/* --- an id we have no packet for and no handler ------------------- */
	{
		int before = sent_n;

		len = make_request(req, 0x02, 0x11, 0x44, 0, 0); /* nothing at all */
		host_body_frame(req, len);
		em_poll();
		if (sent_n != before) {
			fail("an unknown id drew a reply");
		}
	}

	/* --- the park handler runs BEFORE the acknowledgement ------------- */
	/* The body is waiting for the echo, and the stock parks first, so the
	 * order is part of the contract.  The park also pumps em_poll from its
	 * servo loop, which must not re-enter the reply path and park again. */
	{
		int before = sent_n;

		em_set_park_handler(fake_park);
		park_calls = 0;
		park_pumps = 3;
		park_inject_0x16 = 1;
		len = make_request(req, 0x02, 0x50, EM_ID_SHUTDOWN, 0, 0);
		host_body_frame(req, len);
		em_poll();

		if (park_calls != 1) {
			fail("park handler ran %d times, expected exactly 1",
			     park_calls);
		}
		if (park_sent_before != before) {
			fail("the acknowledgement went out BEFORE the park");
		}
		if (sent_n != before + 1) {
			fail("0x16 with a park produced %d frames, expected 1",
			     sent_n - before);
		}
		em_set_park_handler(0);
		/* revive for the tests below -- only a session opener does it */
		len = make_request(req, 0x02, 0x51, EM_ID_SESSION, 0, 0);
		host_body_frame(req, len);
		em_poll();
	}

	/* --- a park that misbehaves must not block the acknowledgement ---- */
	{
		int before = sent_n;

		em_set_park_handler(fake_park);
		park_calls = 0;
		park_pumps = 0;
		len = make_request(req, 0x02, 0x52, EM_ID_SHUTDOWN, 0, 0);
		host_body_frame(req, len);
		em_poll();
		if (sent_n != before + 1) {
			fail("no acknowledgement after the park");
		}
		em_set_park_handler(0);
		len = make_request(req, 0x02, 0x53, EM_ID_SESSION, 0, 0);
		host_body_frame(req, len);
		em_poll();
	}

	/* --- message 0x16: the shutdown request --------------------------- */
	/* Not an AF trigger.  The stock parks, then echoes the body's own frame
	 * back as an acknowledgement, then goes quiet.  Ignoring it is what left
	 * a real body waiting at power-off. */
	{
		int before = sent_n;

		len = make_request(req, 0x02, 0x55, EM_ID_SHUTDOWN, 0xAB, 0xCD);
		host_body_frame(req, len);
		em_poll();

		if (sent_n != before + 1) {
			fail("message 0x16 drew %d replies, expected exactly one "
			     "(the echo)", sent_n - before);
		} else {
			const uint8_t *e = sent[before];

			if (sent_len[before] != len) {
				fail("0x16 echo is %d bytes, the request was %d",
				     sent_len[before], len);
			}
			if (e[3] != 0x02 || e[4] != 0x55 || e[5] != EM_ID_SHUTDOWN) {
				fail("0x16 echo is not the request: class %#04x "
				     "seq %#04x id %#04x", e[3], e[4], e[5]);
			}
			/* A 10-byte frame has exactly ONE payload byte: [6].
			 * [7] and [8] are the checksum, which em_transmit
			 * recomputes -- as the stock's own echo does. */
			if (e[6] != 0xAB) {
				fail("0x16 echo lost the payload byte: %#04x", e[6]);
			}
			check_frame(before, EM_ID_SHUTDOWN, "0x16 echo");
		}
		if (!em_shutdown_acked) {
			fail("0x16 did not set the shutdown flag");
		}

		/* and the status loop must stop: the stock takes its bus down. */
		before = sent_n;
		host_frame_sync();
		em_poll();
		if (sent_n != before) {
			fail("%d status frame(s) sent after acknowledging shutdown",
			     sent_n - before);
		}

		/* An ordinary request must NOT revive us, and must draw no reply.
		 * Announcing shutdown and then answering the tail of the old
		 * conversation is what the stock's switched-off bus prevents. */
		before = sent_n;
		len = make_request(req, 0x02, 0x56, 0x07, 0, 0);
		host_body_frame(req, len);
		em_poll();
		if (sent_n != before) {
			fail("id 0x07 drew a reply after shutdown was acknowledged");
		}
		if (!em_shutdown_acked) {
			fail("id 0x07 revived us; only a session opener should");
		}
		if (!em_ignored_after_shutdown) {
			fail("the ignored request was not counted");
		}

		/* A repeated 0x16 gets its echo -- the body may not have heard
		 * the first -- but must not park a second time. */
		before = sent_n;
		park_calls = 0;
		em_set_park_handler(fake_park);
		len = make_request(req, 0x02, 0x57, EM_ID_SHUTDOWN, 0, 0);
		host_body_frame(req, len);
		em_poll();
		if (sent_n != before + 1) {
			fail("a repeated 0x16 drew %d replies, expected 1",
			     sent_n - before);
		}
		if (park_calls != 0) {
			fail("a repeated 0x16 parked again");
		}
		em_set_park_handler(0);

		/* only the session opener brings us back */
		len = make_request(req, 0x02, 0x60, EM_ID_SESSION, 0, 0);
		host_body_frame(req, len);
		em_poll();
		if (em_shutdown_acked) {
			fail("a session opener did not clear the shutdown flag");
		}
		if (!em_t_revived) {
			fail("the revival was not timestamped");
		}
		before = sent_n;
		host_frame_sync();
		em_poll();
		if (sent_n != before + 2) {
			fail("the status loop did not resume after revival");
		}
	}

	/* --- the aperture request, message 0x1B --------------------------- */
	{
		struct em_focus_cmd cmd;
		int idx;

		/* The id sweep above already sent 0x1B frames, so start from a
		 * known state rather than asserting none arrived. */
		(void)em_take_focus_cmd(&cmd);
		em_focus_n = 0;
		if (em_take_focus_cmd(&cmd)) {
			fail("draining left an instruction pending");
		}

		/* A 0x1B carrying 4800 in payload[0..1]. */
		idx = aperture_cmd(0x70, 4800);
		if (idx < 0) {
			fail("message 0x1B drew no reply");
		} else {
			check_frame(idx, EM_ID_APERTURE, "aperture request");
		}
		if (em_focus_n != 1) {
			fail("0x1B not counted: em_focus_n = %u",
			     (unsigned)em_focus_n);
		}
		/* MESSAGE 0x1B MUST NOT DRIVE FOCUS.  It is an init-class
		 * aperture request, and this adapter has no iris. */
		if (em_take_focus_cmd(&cmd)) {
			fail("message 0x1B queued a focus instruction -- it is "
			     "an aperture request, not a focus command");
		}

		/* A 10-byte 0x1B carries no payload[0..1] at all; it must not
		 * even be counted as one that could. */
		em_focus_n = 0;
		request(EM_ID_APERTURE, 0x7A, 0xC0, 0x12);
		if (em_focus_n != 0) {
			fail("a 10-byte 0x1B was counted as carrying a target");
		}
		aperture_cmd(0x7B, 4800);          /* restore: 0x1B has been seen */

		/* The current aperture must reach every field that reports it:
		 * message 0x05's pair and message 0x28's copy.  A body that
		 * reads one at the shutter press and the other from the status
		 * loop must not be told two different numbers. */
		em_set_aperture(5000);
		if (ea9_data[EA9_OFF_05 + 6] != 0x88
		    || ea9_data[EA9_OFF_05 + 7] != 0x13
		    || ea9_data[EA9_OFF_05 + 8] != 0x88
		    || ea9_data[EA9_OFF_05 + 9] != 0x13) {
			fail("message 0x05 did not get both copies of the aperture");
		}
		/* Message 0x28 pl[9..10] FOLLOWS THE CAPABILITY BIT, and nothing
		 * else.  The handshake above sent a 0x08 whose pl[1] is zero --
		 * bit 7 clear -- so by now the body has asked for it, and the
		 * field must carry the aperture.  Both states are exercised
		 * properly further down; this is the consistency check. */
		{
			uint16_t got = ea9_data[EA9_OFF_28 + 0x0F]
			               | ((uint16_t)ea9_data[EA9_OFF_28 + 0x10] << 8);
			uint16_t want = em_m08_aperture_on ? 5000 : 0;

			if (got != want) {
				fail("0x28 pl[9..10] is %u with the capability "
				     "bit %s, expected %u",
				     got, em_m08_aperture_on ? "on" : "off", want);
			}
		}

		/* The conversion from the Canon-convention descriptor, checked
		 * against the three devices actually measured on the wire.  If
		 * this drifts, the adapter reports one aperture in pl[0..1] and
		 * a different one in pl[44] -- which is the inconsistency the
		 * stock firmware ships. */
		{
			static const struct { uint8_t v; uint16_t av; } AP[] = {
				{ 0x16, 4544 },   /* Viltrox + EF 50/1.8   f/1.834 */
				{ 0x20, 4864 },   /* Viltrox + EF-S 24/2.8 f/2.828 */
				{ 0x18, 4608 },   /* stock LM-EA9, at boot f/2.0   */
				/* What we declare -- the same f/1.834 the
				 * Viltrox sends, which is why the first row
				 * and this one now agree.  Both are here on
				 * purpose: one is a measured device, the other
				 * is a constant in this build, and the test
				 * is that the conversion serves both. */
				{ 0x16, EM_APERTURE_NOW },
			};
			unsigned k;

			for (k = 0; k < sizeof(AP) / sizeof(AP[0]); k++) {
				if (em_aperture_from_descriptor(AP[k].v) != AP[k].av) {
					fail("descriptor %#04x converted to %u, "
					     "expected %u", AP[k].v,
					     em_aperture_from_descriptor(AP[k].v),
					     AP[k].av);
				}
			}
		}

		/* THE TWO ENCODINGS ROUND-TRIP.
		 *
		 * The declarations are aperture values; the descriptor block
		 * is 1/8 stop.  Anything that lands on the coarser grid must
		 * come back unchanged, or the block would disagree with the
		 * live fields derived from the same constant. */
		{
			static const uint16_t ON_GRID[] = {
				4096, 4352, 4608, 4864, 5120, 5632, 6656
			};
			unsigned k;

			for (k = 0; k < sizeof(ON_GRID) / sizeof(ON_GRID[0]); k++) {
				uint8_t  d = em_aperture_to_descriptor(ON_GRID[k]);
				uint16_t v = em_aperture_from_descriptor(d);

				if (v != ON_GRID[k]) {
					fail("aperture %u -> descriptor %#04x -> %u",
					     ON_GRID[k], d, v);
				}
			}
			/* Off the grid it ROUNDS, and rounds to nearest --
			 * truncating would always report the lens slower than
			 * it is.  4608 + 16 is the midpoint; one count past it
			 * must round up. */
			if (em_aperture_to_descriptor(4608 + 17) != 0x19) {
				fail("an aperture just past the midpoint rounded "
				     "down (%#04x)",
				     em_aperture_to_descriptor(4608 + 17));
			}
			if (em_aperture_to_descriptor(4608 + 15) != 0x18) {
				fail("an aperture just below the midpoint rounded "
				     "up (%#04x)",
				     em_aperture_to_descriptor(4608 + 15));
			}
			/* Below the scale's floor it clamps rather than
			 * wrapping: 4096 is f/1.0 and there is nothing below
			 * it in an unsigned subtraction. */
			if (em_aperture_to_descriptor(0) != 0x08
			    || em_aperture_to_descriptor(4000) != 0x08) {
				fail("an aperture below f/1.0 did not clamp");
			}
		}

		/* THE ARGUMENT ORDER, pinned independently of the constants.
		 *
		 * This adapter declares the same aperture for both ends, so
		 * with the shipped values a swapped max/min is invisible --
		 * and a mutation that swapped them passed every other check
		 * here.  Call it with a range that is actually a range. */
		{
			em_set_aperture_range(0x1100u, 0x1600u);  /* f/1.4 .. f/8 */
			if (ea9_data[EA9_OFF_05 + 0x32] != 0x10) {
				fail("pl[44] took the minimum, not the maximum "
				     "(%#04x)", ea9_data[EA9_OFF_05 + 0x32]);
			}
			if (ea9_data[EA9_OFF_05 + 0x34] != 0x10) {
				fail("pl[46] did not follow the maximum (%#04x)",
				     ea9_data[EA9_OFF_05 + 0x34]);
			}
			if (ea9_data[EA9_OFF_05 + 0x3A] != 0x38) {
				fail("pl[52] took the maximum, not the minimum "
				     "(%#04x)", ea9_data[EA9_OFF_05 + 0x3A]);
			}
		}

		/* THE BLOCK MATCHES A STOCK LM-EA9 EXCEPT AT pl[52].
		 *
		 * The strongest statement available about a block whose fields
		 * are half unknown: not "our values are reasonable" but "they
		 * are the ones a shipped adapter sends".  The one field this
		 * build was ever free with is pl[46], and getting it wrong
		 * advertised an aperture the lens does not have.
		 *
		 * FOUR bytes are ours, and only those four: pl[44], pl[46] and
		 * pl[51] carry the declared maximum, which is f/1.8 here and
		 * f/2.0 on a stock unit, and pl[52] carries the declared
		 * minimum, which is the same f/1.8 rather than the stock's
		 * f/90 because this device has no iris.  Restoring f/90 was
		 * tried and changed nothing.
		 *
		 * The maximum moved to f/1.8 so the descriptor block and the
		 * slot B type 1 optical row describe one lens (emount.h).
		 *
		 * A future deviation has to edit this array, deliberately and
		 * with a reason. */
		{
			static const uint8_t STOCK[16] = {
				0x18, 0x00, 0x18, 0x00, 0xA0, 0x00, 0x00, 0x18,
				0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
			};
			/* The indices this build writes from its own
			 * declarations: pl[44], pl[46], pl[51] and pl[52]. */
			static const uint8_t OURS[16] = {
				1, 0, 1, 0, 0, 0, 0, 1,
				2, 0, 0, 0, 0, 0, 0, 0
			};
			const uint8_t MAXB = em_aperture_to_descriptor(EM_APERTURE_MAX);
			const uint8_t MINB = em_aperture_to_descriptor(EM_APERTURE_MIN);
			unsigned k, deviations = 0;

			em_set_aperture_range(EM_APERTURE_MAX, EM_APERTURE_MIN);
			for (k = 0; k < 16; k++) {
				uint8_t want = OURS[k] == 1 ? MAXB
				               : OURS[k] == 2 ? MINB
				               : STOCK[k];

				if (ea9_data[EA9_OFF_05 + 0x32 + k] != want) {
					fail("pl[%u] is %#04x, expected %#04x "
					     "(a stock LM-EA9 sends %#04x)",
					     44 + k,
					     ea9_data[EA9_OFF_05 + 0x32 + k],
					     want, STOCK[k]);
				}
				if (want != STOCK[k]) {
					deviations++;
				}
			}
			/* Every byte not in OURS matched a stock literal above,
			 * so the only way to deviate is through OURS -- and the
			 * exemption is not allowed to become vacuous either. */
			if (!deviations) {
				fail("test stale: nothing deviates from the "
				     "stock block any more, so the exemption "
				     "hides nothing and should go");
			}
		}

		/* The descriptor block, written from the three declarations.
		 * Checked against the live table, NOT by re-running em_init --
		 * that blocks on the body's chip select. */
		em_set_aperture_range(EM_APERTURE_MAX, EM_APERTURE_MIN);
		em_set_aperture(EM_APERTURE_NOW);
		{
			const uint8_t MAXD = em_aperture_to_descriptor(EM_APERTURE_MAX);
			const uint8_t MIND = em_aperture_to_descriptor(EM_APERTURE_MIN);
			const struct { uint16_t off; uint8_t want;
			               const char *what; } D[] = {
				{ 0x32, MAXD,     "pl[44] maximum" },
				{ 0x39, MAXD,     "pl[51] maximum copy" },
				{ 0x3A, MIND,     "pl[52] minimum" },
				/* pl[46] EQUALS the maximum.  The "one stop
				 * wider" relation is in the STORED descriptor;
				 * the boot patch moves pl[44] and not this, so
				 * what the stock sends has them equal.  Getting
				 * this wrong advertises an aperture the lens
				 * does not have. */
				{ 0x34, MAXD,     "pl[46] = the maximum" },
				{ 0x36, 0xA0,     "pl[48] constant" },
				{ 0x41, 0x01,     "pl[59] constant" },
			};
			unsigned k;
			uint16_t got, want;

			for (k = 0; k < sizeof(D) / sizeof(D[0]); k++) {
				if (ea9_data[EA9_OFF_05 + D[k].off] != D[k].want) {
					fail("descriptor %s is %#04x, expected %#04x",
					     D[k].what,
					     ea9_data[EA9_OFF_05 + D[k].off],
					     D[k].want);
				}
			}

			/* NEVER all zero.  A build that zeroed this block was
			 * displayed as F1.0 by an a9 II and refused autofocus
			 * altogether (EA9.md §8). */
			if (ea9_data[EA9_OFF_05 + 0x32] == 0
			    && ea9_data[EA9_OFF_05 + 0x39] == 0
			    && ea9_data[EA9_OFF_05 + 0x3A] == 0) {
				fail("the aperture descriptor is empty");
			}

			/* And the reported aperture must agree with the
			 * declaration -- now the same number, not a
			 * conversion of it. */
			got  = (uint16_t)ea9_data[EA9_OFF_05 + 6]
			       | ((uint16_t)ea9_data[EA9_OFF_05 + 7] << 8);
			want = EM_APERTURE_NOW;
			if (got != want) {
				fail("reported aperture %u disagrees with the "
				     "declaration %u", got, want);
			}
		}

		/* THE REPLY CARRIES OUR OWN APERTURE, NOT THE REQUESTER'S.
		 *
		 * The stock echoes; this device has no iris, so echoing an
		 * aperture it cannot produce would contradict message 0x05
		 * pl[0..1] in the same frame pair.  Ask for something we do not
		 * have and the answer must still be what we do have. */
		em_set_aperture(EM_APERTURE_NOW);
		idx = aperture_cmd(0x73, 4864);          /* f/2.83, not ours */
		if (idx >= 0) {
			const uint8_t *g = sent[idx];
			uint16_t a = g[6] | ((uint16_t)g[7] << 8);
			uint16_t b = g[8] | ((uint16_t)g[9] << 8);

			if (a != EM_APERTURE_NOW || b != EM_APERTURE_NOW) {
				fail("0x1B reply carried %u/%u -- it must report "
				     "our own %u, not the request's 4864",
				     a, b, EM_APERTURE_NOW);
			}
			check_frame(idx, EM_ID_APERTURE, "aperture reply");
		}
		/* And it must AGREE with message 0x05, which is the whole point
		 * of not echoing. */
		{
			uint16_t m5 = ea9_data[EA9_OFF_05 + 6]
			              | ((uint16_t)ea9_data[EA9_OFF_05 + 7] << 8);

			if (m5 != EM_APERTURE_NOW) {
				fail("message 0x05 says %u while 0x1B says %u",
				     m5, EM_APERTURE_NOW);
			}
		}
		/* The request is RECORDED even though it is not obeyed: a body
		 * that asks for an aperture we do not have is the one thing
		 * worth knowing about this channel. */
		if (em_aperture_req != 4864) {
			fail("the commanded aperture was not recorded (%u)",
			     em_aperture_req);
		}

	}

	/* --- message 0x04's RECORD STREAM --------------------------------- *
	 *
	 * The frame is a 13-byte header then tagged records, and its LENGTH is
	 * a consequence of the records in it.  Keying on length -- which an
	 * earlier build did -- can see a frame with one record and is blind to
	 * a frame with two.
	 */
	{
		struct em_focus_cmd cmd;
		uint8_t  f[64];
		uint16_t n;
		uint16_t clean_unknown;

		/* Build a 0x04 carrying the record bytes handed in. */
		#define REC_FRAME(...) do {                                   \
			static const uint8_t R[] = { __VA_ARGS__ };           \
			unsigned q;                                           \
			n = (uint16_t)(19 + sizeof(R) + 3);                   \
			memset(f, 0, sizeof(f));                              \
			f[0] = 0xF0; f[1] = (uint8_t)n; f[3] = 0x01;          \
			f[5] = 0x04;                                          \
			for (q = 0; q < sizeof(R); q++) { f[19 + q] = R[q]; } \
			f[n - 1] = 0x55;                                      \
			host_body_frame(f, n); em_poll();                     \
		} while (0)

		(void)em_take_focus_cmd(&cmd);

		/* EVERY FRAME IN THIS SECTION ARRIVES IN A PADDED WINDOW, the
		 * way a real body sends it: 16, 32 or 48 bytes whatever the
		 * frame's own length.  The walk must be bounded by the FRAME,
		 * not by the window -- reading to the end of the window takes
		 * in the checksum, the terminator and the padding, and 0x55 is
		 * a perfectly good operand byte. */
		host_pad_to = 48;
		clean_unknown = em_rec_unknown;

		/* MOVE, mode 0: absolute, the operand is the position. */
		REC_FRAME(0x1D, 0xEC, 0x12, 0x00, 0x00);
		if (!em_take_focus_cmd(&cmd)) {
			fail("a Move record queued no instruction");
		} else if (cmd.op != EM_OP_MOVE || cmd.target != 0x12EC) {
			fail("Move mode 0 decoded as op %u target %u",
			     cmd.op, cmd.target);
		}

		/* Mode 3: the operand is a distance code, not a position. */
		REC_FRAME(0x1D, 0x80, 0x01, 0x00, 0x03);
		if (!em_take_focus_cmd(&cmd) || cmd.op != EM_OP_MOVE_DIST
		    || cmd.target != 384) {
			fail("Move mode 3 (by distance) not decoded");
		}

		/* Mode 4: relative, and the operand is SIGNED. */
		REC_FRAME(0x1D, 0x9C, 0xFF, 0x00, 0x04);
		if (!em_take_focus_cmd(&cmd) || cmd.op != EM_OP_MOVE_REL
		    || cmd.delta != -100) {
			fail("Move mode 4 (relative) not decoded as -100");
		}

		/* Mode 6: relative in defocus units. */
		REC_FRAME(0x1D, 0x05, 0x00, 0x00, 0x06);
		if (!em_take_focus_cmd(&cmd) || cmd.op != EM_OP_MOVE_DEFOCUS
		    || cmd.delta != 5) {
			fail("Move mode 6 (defocus units) not decoded");
		}

		/* BIT 3 OF THE MODE BYTE IS IGNORED.  A build that masked with
		 * 0x0F instead of 0x07 would read mode 8 as a mode of its own
		 * and silently drop every Move carrying it. */
		REC_FRAME(0x1D, 0xEC, 0x12, 0x00, 0x08);
		if (!em_take_focus_cmd(&cmd) || cmd.op != EM_OP_MOVE) {
			fail("mode bit 3 was not ignored");
		}

		/* A mode with no meaning is IGNORED, not guessed at. */
		REC_FRAME(0x1D, 0xEC, 0x12, 0x00, 0x05);
		if (em_take_focus_cmd(&cmd)) {
			fail("an unknown Move mode was acted on");
		}

		/* THE NO-TARGET SENTINEL must never be driven to.  Clamping it
		 * into the travel turns every idle frame carrying one into a
		 * full-travel slam toward the close stop. */
		REC_FRAME(0x1D, 0xFF, 0x7F, 0x00, 0x00);
		if (em_take_focus_cmd(&cmd)) {
			fail("the 0x7FFF no-target sentinel queued a move");
		}

		/* STOP is a one-byte record, not a frame length. */
		REC_FRAME(0x1C);
		if (!em_take_focus_cmd(&cmd) || cmd.op != EM_OP_STOP) {
			fail("a Stop record was not decoded");
		}

		/* TWO RECORDS IN ONE FRAME -- the row index then a stop, which
		 * is a 26-byte frame the protocol lists.  The walker must skip
		 * the 3-byte row index and find the Stop behind it; a
		 * length-keyed reader sees a length it has never heard of. */
		REC_FRAME(0x2F, 0x09, 0x0A, 0x1C);
		if (!em_take_focus_cmd(&cmd) || cmd.op != EM_OP_STOP) {
			fail("a Stop behind a row-index record was missed");
		}

		/* SCAN, 14 bytes, from the record observed on a native lens. */
		REC_FRAME(0x1F, 0x02, 0x02, 0x83, 0x88, 0x3D, 0x49, 0xED,
		          0x47, 0x3F, 0x46, 0x03, 0x00, 0x00);
		if (!em_take_focus_cmd(&cmd)) {
			fail("a Scan record queued no instruction");
		} else if (cmd.op != EM_OP_SCAN || cmd.scan_a != 0x493D
		           || cmd.scan_b != 0x47ED || cmd.scan_speed != 0x88
		           || cmd.scan_flags != 0x02) {
			fail("Scan decoded as op %u a=%u b=%u speed=%#04x",
			     cmd.op, cmd.scan_a, cmd.scan_b, cmd.scan_speed);
		}

		/* A ROW INDEX THEN A SCAN -- the 39-byte form the protocol
		 * lists.  Two records, and the second only reachable if the
		 * first is sized right. */
		REC_FRAME(0x2F, 0x07, 0x08,
		          0x1F, 0x06, 0x02, 0x83, 0x98, 0x7D, 0x45, 0xF5,
		          0x43, 0x10, 0x52, 0x03, 0x00, 0x00);
		if (!em_take_focus_cmd(&cmd) || cmd.op != EM_OP_SCAN
		    || cmd.scan_a != 0x457D || cmd.scan_flags != 0x06) {
			fail("a Scan behind a row-index record was missed");
		}

		/* ...and a STOP behind a SCAN, which pins the Scan's own size
		 * from the other side: 13 bytes and the walk lands on an
		 * operand, 15 and it swallows the Stop. */
		REC_FRAME(0x1F, 0x02, 0x02, 0x83, 0x88, 0x3D, 0x49, 0xED,
		          0x47, 0x3F, 0x46, 0x03, 0x00, 0x00,
		          0x1C);
		if (!em_take_focus_cmd(&cmd) || cmd.op != EM_OP_STOP) {
			fail("a Stop behind a Scan was missed -- the Scan "
			     "record's size is wrong");
		}

		/* DRIVE, 8 bytes, with a signed velocity. */
		REC_FRAME(0x3C, 0x40, 0x00, 0xFF, 0, 0, 0, 0);
		if (!em_take_focus_cmd(&cmd) || cmd.op != EM_OP_DRIVE
		    || cmd.drive_vel != -256 || cmd.drive_param != 0x40) {
			fail("Drive not decoded");
		}

		/* A STOP BEHIND A DRIVE pins the Drive's size the same way. */
		REC_FRAME(0x3C, 0x40, 0x00, 0xFF, 0, 0, 0, 0, 0x1C);
		if (!em_take_focus_cmd(&cmd) || cmd.op != EM_OP_STOP) {
			fail("a Stop behind a Drive was missed -- the Drive "
			     "record's size is wrong");
		}

		/* QUERIES.  Both are 3 bytes and neither is an instruction:
		 * they must reach the query queue and leave focus alone. */
		(void)em_take_focus_cmd(&cmd);
		REC_FRAME(0x22, 0x34, 0x12, 0x2E, 0x80, 0x01);
		if (em_take_focus_cmd(&cmd)) {
			fail("a query queued a focus instruction");
		}
		{
			uint8_t  tag;
			uint16_t operand;

			if (!em_take_query(&tag, &operand) || tag != 0x22
			    || operand != 0x1234) {
				fail("the 0x22 query did not reach the queue");
			}
			if (!em_take_query(&tag, &operand) || tag != 0x2E
			    || operand != 0x0180) {
				fail("the 0x2E query did not reach the queue");
			}
			if (em_take_query(&tag, &operand)) {
				fail("a third query came out of a two-query frame");
			}
		}

		/* EVERY WELL-FORMED FRAME ABOVE MUST HAVE WALKED CLEANLY.
		 *
		 * This is what catches a wrong record SIZE.  Decoding the
		 * fields of a record is not enough: if its size is short, the
		 * walk lands on an operand byte, reads it as a tag and stops --
		 * and every assertion above still passes, because the record
		 * itself decoded fine.  A size that is too long swallows the
		 * record behind it just as silently. */
		if (em_rec_unknown != clean_unknown) {
			fail("%u well-formed record(s) ended the walk -- a "
			     "record size is wrong",
			     em_rec_unknown - clean_unknown);
		}

		/* A BARE 0x04 -- 13 bytes of header and nothing else -- is the
		 * idle form, and carries no instruction.
		 *
		 * This is the frame the body sends a thousand times a session,
		 * and in a padded window it is where the damage started: the
		 * walk began at pl[13], which in a 22-byte frame is already the
		 * checksum, and ran on into the padding. */
		{
			uint16_t before_unknown = em_rec_unknown;

			n = 22;
			memset(f, 0, sizeof(f));
			f[0] = 0xF0; f[1] = (uint8_t)n; f[3] = 0x01; f[5] = 0x04;
			f[n - 1] = 0x55;
			host_body_frame(f, n); em_poll();
			if (em_take_focus_cmd(&cmd)) {
				fail("the bare 22-byte 0x04 queued an instruction");
			}
			if (em_rec_unknown != before_unknown) {
				fail("the bare 22-byte 0x04 was read as a bad record");
			}
		}

		/* AN UNKNOWN TAG ENDS THE WALK.  It cannot be skipped -- its
		 * size is exactly what is unknown -- so anything behind it is
		 * unreachable, and pretending otherwise would act on operand
		 * bytes read as tags. */
		{
			uint16_t before_unknown = em_rec_unknown;

			REC_FRAME(0x77, 0x1C);
			if (em_take_focus_cmd(&cmd)) {
				fail("the walker resynchronised past an unknown tag");
			}
			if (em_rec_unknown == before_unknown) {
				fail("an unknown tag was not counted");
			}
		}

		/* A RECORD THAT RUNS PAST THE END OF THE FRAME is the same
		 * case: a truncated Move must not be read out of the checksum
		 * bytes behind it. */
		{
			uint16_t before_unknown = em_rec_unknown;

			n = 19 + 2 + 3;          /* a 5-byte Move, 2 bytes of it */
			memset(f, 0, sizeof(f));
			f[0] = 0xF0; f[1] = (uint8_t)n; f[3] = 0x01; f[5] = 0x04;
			f[19] = 0x1D; f[20] = 0xEC;
			f[n - 1] = 0x55;
			host_body_frame(f, n); em_poll();
			if (em_take_focus_cmd(&cmd)) {
				fail("a truncated Move was acted on");
			}
			if (em_rec_unknown == before_unknown) {
				fail("a truncated record was not counted");
			}
		}

		/* THE NEWEST INSTRUCTION WINS.  Two frames, no poll between
		 * them: the second replaces the first, because that is what a
		 * new instruction does to a running one. */
		REC_FRAME(0x1D, 0x00, 0x11, 0x00, 0x00);
		REC_FRAME(0x1C);
		if (!em_take_focus_cmd(&cmd) || cmd.op != EM_OP_STOP) {
			fail("a Stop did not supersede the Move before it");
		}
		REC_FRAME(0x1C);
		REC_FRAME(0x1D, 0x00, 0x11, 0x00, 0x00);
		if (!em_take_focus_cmd(&cmd) || cmd.op != EM_OP_MOVE) {
			fail("a Move did not supersede the Stop before it");
		}

		/* A SHORT WINDOW is the other direction: a frame that declares
		 * more than arrived must not be read past what arrived. */
		{
			uint16_t before_trunc = em_truncated;

			host_pad_to = 0;
			memset(f, 0, sizeof(f));
			f[0] = 0xF0; f[1] = 27; f[3] = 0x01; f[5] = 0x04;
			f[19] = 0x1D; f[20] = 0xEC; f[21] = 0x12;
			host_body_frame(f, 22);          /* 27 declared, 22 sent */
			em_poll();
			/* The Move's operand bytes never arrived, so there is
			 * no Move -- reading one would mean reading two bytes
			 * that were never sent. */
			if (em_take_focus_cmd(&cmd)) {
				fail("a frame shorter than it declared was read "
				     "past its own end");
			}
			if (em_truncated == before_trunc) {
				fail("the short window was not noticed at all");
			}
		}

		host_pad_to = 0;
		#undef REC_FRAME
	}

	/* --- the 0x06 tail: events and query answers ---------------------- */
	{
		int before;

		/* An idle frame has no tail at all: in the 48-byte form
		 * pl[39..40] ARE the checksum, so there is no way to carry a
		 * block without growing the frame. */
		before = sent_n;
		host_frame_sync();
		em_poll();
		if (sent_len[before + 1] != EM06_LEN_IDLE) {
			fail("an idle message 0x06 was %u bytes",
			     sent_len[before + 1]);
		}

		/* ONE EVENT -> a 50-byte frame carrying code and parameter. */
		em_post_event(EM_REC_MOVE, 0x00);
		before = sent_n;
		host_frame_sync();
		em_poll();
		{
			const uint8_t *g = sent[before + 1];
			uint16_t cs = 0, k;

			if (sent_len[before + 1] != EM06_LEN_EVENT) {
				fail("the event frame was %u bytes",
				     sent_len[before + 1]);
			}
			if (g[1] != EM06_LEN_EVENT) {
				fail("the event frame's length byte is %u", g[1]);
			}
			if (g[0x2D] != 0x1D || g[0x2E] != 0x00) {
				fail("the Move event carried %#04x %#04x",
				     g[0x2D], g[0x2E]);
			}
			/* The checksum and terminator have MOVED by two. */
			for (k = 1; k <= EM06_LEN_EVENT - 4; k++) {
				cs += g[k];
			}
			if (g[EM06_LEN_EVENT - 3] != (cs & 0xFF)
			    || g[EM06_LEN_EVENT - 2] != (cs >> 8)
			    || g[EM06_LEN_EVENT - 1] != 0x55) {
				fail("the event frame's trailer did not move");
			}
		}

		/* The STOP event's parameter is 0x01, not 0x00.  Checking the
		 * code alone would pass on a build that always sent 0. */
		em_post_event(EM_REC_STOP, 0x01);
		before = sent_n;
		host_frame_sync();
		em_poll();
		if (sent[before + 1][0x2D] != 0x1C
		    || sent[before + 1][0x2E] != 0x01) {
			fail("the Stop event carried %#04x %#04x",
			     sent[before + 1][0x2D], sent[before + 1][0x2E]);
		}

		/* A DRIVE event is FOUR bytes, not two. */
		em_post_event(EM_REC_DRIVE, 0x10);
		before = sent_n;
		host_frame_sync();
		em_poll();
		if (sent_len[before + 1] != 52) {
			fail("the Drive event frame was %u bytes, expected 52",
			     sent_len[before + 1]);
		}
		if (sent[before + 1][0x2D] != 0x3C
		    || sent[before + 1][0x2E] != 0x10
		    || sent[before + 1][0x2F] != 0x00
		    || sent[before + 1][0x30] != 0x00) {
			fail("the Drive event's 4-byte block is wrong");
		}

		/* ONE EVENT PER FRAME: two posted, two frames, in order. */
		em_post_event(EM_REC_MOVE, 0x00);
		em_post_event(EM_REC_STOP, 0x01);
		before = sent_n;
		host_frame_sync();
		em_poll();
		if (sent[before + 1][0x2D] != 0x1D) {
			fail("the first of two events was not the older one");
		}
		before = sent_n;
		host_frame_sync();
		em_poll();
		if (sent[before + 1][0x2D] != 0x1C) {
			fail("the second event did not follow on the next frame");
		}
		before = sent_n;
		host_frame_sync();
		em_poll();
		if (sent_len[before + 1] != EM06_LEN_IDLE) {
			fail("the event queue did not drain");
		}

		/* QUERY ANSWERS are 3-byte blocks, and several ride together. */
		em_post_query_answer(0x22, 0x0180);
		em_post_query_answer(0x2E, 0x1234);
		before = sent_n;
		host_frame_sync();
		em_poll();
		{
			const uint8_t *g = sent[before + 1];

			if (sent_len[before + 1] != EM06_LEN_IDLE + 6) {
				fail("two query answers made a %u-byte frame",
				     sent_len[before + 1]);
			}
			if (g[0x2D] != 0x22 || g[0x2E] != 0x80 || g[0x2F] != 0x01
			    || g[0x30] != 0x2E || g[0x31] != 0x34 || g[0x32] != 0x12) {
				fail("the query answers are not two 3-byte blocks");
			}
		}

		/* AN EVENT AND AN ANSWER IN THE SAME TAIL.  The event comes
		 * first, and the frame is long enough for both. */
		em_post_event(EM_REC_MOVE, 0x00);
		em_post_query_answer(0x22, 0x0700);
		before = sent_n;
		host_frame_sync();
		em_poll();
		if (sent_len[before + 1] != EM06_LEN_IDLE + 5
		    || sent[before + 1][0x2D] != 0x1D
		    || sent[before + 1][0x2F] != 0x22) {
			fail("an event and an answer did not share one tail");
		}

		/* THE TAIL MUST NOT RUN INTO PACKET 0x01.
		 *
		 * The frame is built in place, and packet 0x01 starts 56 bytes
		 * into the same table.  A tail that overran would corrupt the
		 * capability bitmap -- and the failure would surface as a
		 * broken handshake in some LATER session, nowhere near here.
		 * Four answers cannot fit; the ones that do not must WAIT. */
		{
			uint8_t saved[8];
			unsigned k;

			for (k = 0; k < 8; k++) {
				saved[k] = ea9_data[EA9_OFF_01 + k];
			}
			em_post_event(EM_REC_DRIVE, 0x10);
			em_post_query_answer(0x22, 0x0111);
			em_post_query_answer(0x22, 0x0222);
			em_post_query_answer(0x22, 0x0333);
			em_post_query_answer(0x22, 0x0444);

			before = sent_n;
			host_frame_sync();
			em_poll();
			if (sent_len[before + 1] > EM06_LEN_MAX) {
				fail("a %u-byte 0x06 overran its slot",
				     sent_len[before + 1]);
			}
			for (k = 0; k < 8; k++) {
				if (ea9_data[EA9_OFF_01 + k] != saved[k]) {
					fail("the tail wrote into packet 0x01 "
					     "at +%u", k);
					break;
				}
			}

			/* The answers that did not fit come out on the frames
			 * that follow, in order, none lost. */
			{
				uint16_t seen = 0;
				unsigned f;

				for (f = 0; f < 6; f++) {
					const uint8_t *g;
					uint16_t       q;

					before = sent_n;
					host_frame_sync();
					em_poll();
					g = sent[before + 1];
					for (q = 45; q + 2 < sent_len[before + 1] - 3 + 3; q += 3) {
						if (g[q] == 0x22) {
							seen++;
						}
					}
					if (sent_len[before + 1] == EM06_LEN_IDLE) {
						break;
					}
				}
				if (seen < 3) {
					fail("only %u of the 4 answers followed",
					     seen);
				}
			}
		}

		/* And the frame after a tail is a valid 48-byte frame again --
		 * a tail must not corrupt what follows it. */
		before = sent_n;
		host_frame_sync();
		em_poll();
		{
			const uint8_t *g = sent[before + 1];
			uint16_t cs = 0, k;

			if (sent_len[before + 1] != EM06_LEN_IDLE) {
				fail("the frame after a tail was %u bytes",
				     sent_len[before + 1]);
			}
			for (k = 1; k <= EM06_LEN_IDLE - 4; k++) {
				cs += g[k];
			}
			if (g[EM06_LEN_IDLE - 3] != (cs & 0xFF)
			    || g[EM06_LEN_IDLE - 2] != (cs >> 8)
			    || g[EM06_LEN_IDLE - 1] != 0x55) {
				fail("the frame after a tail has a bad trailer");
			}
		}
	}

	/* --- message 0x06 payload[25]: the SCAN progress byte ------------- *
	 *
	 * It belongs to the Scan instruction and to nothing else.  An earlier
	 * build stepped it on every move, which is not what the field means.
	 */
	{
		const uint16_t OFF = EA9_OFF_06 + 0x1F;
		int before;

		em_set_scan_state(EM_SCAN_IDLE);
		if (ea9_data[OFF] != EM_SCAN_IDLE) {
			fail("the scan byte did not latch idle");
		}

		em_set_scan_state(EM_SCAN_ACCEPTED);
		/* It must HOLD.  An approach leg runs for many frames, and a
		 * byte that decayed on its own would tell the body the sweep
		 * had finished while it had not started. */
		host_frame_sync(); em_poll();
		host_frame_sync(); em_poll();
		if (ea9_data[OFF] != EM_SCAN_ACCEPTED) {
			fail("the scan byte decayed to %#04x", ea9_data[OFF]);
		}

		em_set_scan_state(EM_SCAN_SWEEPING);
		em_scan_finish();
		if (ea9_data[OFF] != EM_SCAN_DONE) {
			fail("the sweep did not end at 0x30 (%#04x)",
			     ea9_data[OFF]);
		}

		/* The frame carrying 0x30 goes out BEFORE the byte moves on.
		 * Stepping before the transmit would pass every table-side
		 * check and still never put 0x30 on the wire. */
		before = sent_n;
		host_frame_sync();
		em_poll();
		if (sent[before + 1][0x1F] != EM_SCAN_DONE) {
			fail("the frame sent carried %#04x, expected 0x30",
			     sent[before + 1][0x1F]);
		}
		if (ea9_data[OFF] != EM_SCAN_IDLE) {
			fail("the scan byte did not return to idle");
		}

		/* And it stays idle: the sequence must not wrap. */
		host_frame_sync(); em_poll();
		host_frame_sync(); em_poll();
		if (ea9_data[OFF] != EM_SCAN_IDLE) {
			fail("the scan sequence restarted itself");
		}
	}

	/* --- message 0x06 payload[0..1]: motion and direction ------------- */
	{
		const uint16_t MOT = EA9_OFF_06 + 0x06;
		const uint16_t DIR = EA9_OFF_06 + 0x07;

		/* At rest, mid-travel: 0x82 and no direction. */
		em_set_motion(0, 0x00, 4800);
		if (ea9_data[MOT] != 0x82 || ea9_data[DIR] != 0x00) {
			fail("at rest reported %#04x %#04x",
			     ea9_data[MOT], ea9_data[DIR]);
		}
		em_set_motion(1, 0x02, 4800);
		if (ea9_data[MOT] != 0x02 || ea9_data[DIR] != 0x02) {
			fail("moving reported %#04x %#04x",
			     ea9_data[MOT], ea9_data[DIR]);
		}

		/* The near-limit bits are derived from the position against the
		 * limits IN THE FRAME, so they cannot contradict it. */
		em_set_motion(0, 0x00, 4144);
		if (!(ea9_data[MOT] & 0x10)) {
			fail("at the lower limit, bit 4 is clear (%#04x)",
			     ea9_data[MOT]);
		}
		em_set_motion(0, 0x00, 5632);
		if (!(ea9_data[MOT] & 0x08)) {
			fail("at the upper limit, bit 3 is clear (%#04x)",
			     ea9_data[MOT]);
		}
		em_set_motion(0, 0x00, 4800);
		if (ea9_data[MOT] & 0x18) {
			fail("mid-travel set a near-limit bit (%#04x)",
			     ea9_data[MOT]);
		}
	}

	/* --- message 0x06 payload[32..38]: the velocity history ----------- */
	{
		const uint8_t *h = &ea9_data[EA9_OFF_06 + 0x26];
		unsigned k;

		for (k = 0; k < 7; k++) {
			em_push_velocity(0);
		}
		em_push_velocity(5);
		if ((int8_t)h[6] != 5) {
			fail("the newest difference is not last (%d)",
			     (int8_t)h[6]);
		}
		em_push_velocity(-3);
		if ((int8_t)h[6] != -3 || (int8_t)h[5] != 5) {
			fail("the history did not shift");
		}
		/* It is SIGNED and it saturates rather than wrapping: a move
		 * faster than 127 counts per frame must not report a reversal. */
		em_push_velocity(10000);
		if ((int8_t)h[6] != 127) {
			fail("a fast frame wrapped to %d", (int8_t)h[6]);
		}
		em_push_velocity(-10000);
		if ((int8_t)h[6] != -128) {
			fail("a fast reverse frame wrapped to %d", (int8_t)h[6]);
		}
		for (k = 0; k < 7; k++) {
			em_push_velocity(0);
		}
	}

	/* --- message 0x05's focus reports --------------------------------- */
	{
		em_set_subject_distance(0x0700, 0xFF, 0x00);
		if (ea9_data[EA9_OFF_05 + 0x1A] != 0x00
		    || ea9_data[EA9_OFF_05 + 0x1B] != 0x07) {
			fail("the subject distance is not at pl[20..21]");
		}
		if (ea9_data[EA9_OFF_05 + 0x1D] != 0xFF) {
			fail("the coarse distance is not at pl[23]");
		}

		/* pl[22]: bit 7 always set on a lens that reports motion, bit 6
		 * only while something is moving.  Setting bit 6 must not
		 * disturb the per-lens bits below it. */
		ea9_data[EA9_OFF_05 + 0x1C] = 0x80;
		em_set_in_motion(1);
		if (ea9_data[EA9_OFF_05 + 0x1C] != 0xC0) {
			fail("in-motion did not set bit 6 (%#04x)",
			     ea9_data[EA9_OFF_05 + 0x1C]);
		}
		em_set_in_motion(0);
		if (ea9_data[EA9_OFF_05 + 0x1C] != 0x80) {
			fail("at rest did not clear bit 6 (%#04x)",
			     ea9_data[EA9_OFF_05 + 0x1C]);
		}
	}


	/* --- message 0x08's capability bit -------------------------------- *
	 *
	 * Bit 7 of the request's pl[1], INVERTED, decides whether message 0x28
	 * reports the aperture.  Both directions are tested, because the
	 * inversion is the kind of thing that is half-right: a build that read
	 * the bit straight would populate the field exactly when it must not.
	 */
	{
		uint8_t  f[24];
		uint16_t n = 17;          /* 6 header + 8 payload + 3 trailer */
		uint16_t before_n = em_m08_n;

		/* Bit 7 SET -> the body does NOT want it. */
		memset(f, 0, sizeof(f));
		f[0] = 0xF0; f[1] = (uint8_t)n; f[3] = 0x02; f[5] = 0x08;
		f[6] = 0xC2; f[7] = 0xE1;                    /* pl[1] bit 7 set */
		f[n - 1] = 0x55;
		host_body_frame(f, n);
		em_poll();

		if (em_m08_n != (uint16_t)(before_n + 1)) {
			fail("the 0x08 request was not counted");
		}
		if (em_m08_aperture_on) {
			fail("pl[1] bit 7 SET was read as \"the body wants it\"");
		}
		if (ea9_data[EA9_OFF_28 + 0x0F] || ea9_data[EA9_OFF_28 + 0x10]) {
			fail("0x28 pl[9..10] was filled although bit 7 was set");
		}

		/* Bit 7 CLEAR -> it does.  And the field must take effect at
		 * once: waiting for the next thing that happens to set the
		 * aperture would leave it zero for the rest of a session in
		 * which nothing else ever does. */
		f[4] = 0x01;
		f[7] = 0x61;                                 /* pl[1] bit 7 clear */
		host_body_frame(f, n);
		em_poll();

		if (!em_m08_aperture_on) {
			fail("pl[1] bit 7 CLEAR was not read as \"wanted\"");
		}
		{
			uint16_t got = ea9_data[EA9_OFF_28 + 0x0F]
			               | ((uint16_t)ea9_data[EA9_OFF_28 + 0x10] << 8);

			if (got != EM_APERTURE_NOW) {
				fail("0x28 pl[9..10] is %u, expected the current "
				     "aperture %u", got, EM_APERTURE_NOW);
			}
		}
		/* ...and it tracks a later aperture change, rather than being
		 * a one-shot copy made when the bit arrived. */
		em_set_aperture(0x1400u);
		if ((ea9_data[EA9_OFF_28 + 0x0F]
		     | ((uint16_t)ea9_data[EA9_OFF_28 + 0x10] << 8)) != 0x1400u) {
			fail("0x28 pl[9..10] did not follow a later aperture");
		}

		/* THE FIRST REQUEST IS THE ONE CAPTURED.  The body sends 0x08
		 * once per session, so the capture must be the handshake's --
		 * a 10-byte frame with pl[1] = 0 -- and neither of the two
		 * 17-byte ones sent just now may have overwritten it. */
		if (em_m08[0] != 0xF0 || em_m08[5] != 0x08) {
			fail("the captured bytes are not a 0x08 frame");
		}
		if (em_m08_len != 10 || em_m08[7] != 0x00) {
			fail("a later 0x08 overwrote the first: %u bytes, "
			     "pl[1] = %#04x", em_m08_len,
			     em_m08_len >= 8 ? em_m08[7] : 0);
		}
		/* ...while the FLAG does follow the latest, because it is a
		 * live setting and the capture is a record. */
		if (em_m08_n < 3) {
			fail("only %u 0x08 request(s) counted, expected 3",
			     em_m08_n);
		}

		/* Put it back the way the rest of the suite expects. */
		f[4] = 0x02;
		f[7] = 0xE1;
		host_body_frame(f, n);
		em_poll();
		em_set_aperture(EM_APERTURE_NOW);
	}

	/* --- the body's capability offer -------------------------------- */
	{
		uint8_t req2[64];
		uint16_t n2 = 17;         /* 6 header + 8 payload + 3 trailer */

		memset(req2, 0, sizeof(req2));
		req2[0] = 0xF0;
		req2[1] = (uint8_t)n2;
		req2[3] = 0x02;
		req2[5] = EM_ID_SESSION;
		req2[6] = 0xFF; req2[7] = 0xFF; req2[8] = 0xFF; req2[9] = 0xFF;
		req2[10] = 0xFF; req2[11] = 0xFF; req2[12] = 0xFF; req2[13] = 0x1F;
		req2[n2 - 1] = 0x55;

		/* A frame too short to hold 8 payload bytes must be REJECTED,
		 * not read into the terminator. */
		{
			uint8_t shortf[64];
			uint16_t ns = 14;

			memset(shortf, 0, sizeof(shortf));
			shortf[0] = 0xF0; shortf[1] = (uint8_t)ns; shortf[3] = 0x02;
			shortf[5] = EM_ID_SESSION;
			shortf[6] = 0xAA; shortf[ns - 1] = 0x55;
			host_body_frame(shortf, ns);
			em_poll();
			if (em_body_offer[0] == 0xAA) {
				fail("a 14-byte 0x01 was read as an 8-byte offer");
			}
		}
		host_body_frame(req2, n2);
		em_poll();

		if (em_body_offer[0] != 0xFF || em_body_offer[7] != 0x1F) {
			fail("the body's capability offer was not captured (%02x..%02x)",
			     em_body_offer[0], em_body_offer[7]);
		}
	}

	/* A full transmit log makes every `sent[before + 1]` read a stale
	 * frame, so it must be an error, not a silent truncation. */
	if (sent_overflow) {
		fail("the transmit log overflowed by %d frame(s) -- every index "
		     "into it after that point is stale", sent_overflow);
	}

	printf("%d frames sent, %u received, %d failure(s)\n",
	       sent_n, (unsigned)em_frames_rx, failures);
	return failures ? 1 : 0;
}
