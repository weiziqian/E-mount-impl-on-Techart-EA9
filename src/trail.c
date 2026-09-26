#include "board.h"
#include "delay.h"
#include "diag.h"
#include "emount.h"
#include "trail.h"

static uint32_t g_noinit_boot;
static uint32_t g_flash_boot;

void trail_set_boot(uint32_t noinit_boot, uint32_t flash_boot)
{
	g_noinit_boot = noinit_boot;
	g_flash_boot  = flash_boot;
}

static void zero(uint32_t *w)
{
	unsigned i;

	for (i = 0; i < 16; i++) {
		w[i] = 0;
	}
}


void trail_idlog(uint32_t page)
{
	uint32_t w[16];
	unsigned i;

	zero(w);
	w[0] = EM_DIAG_MAGIC;
	w[1] = EM_DIAG_FORMAT;
	w[2] = TAG_IDLOG;
	w[3] = em_id_log_n;
	for (i = 0; i < em_id_log_n && i < EM_ID_LOG_LEN && i / 4 + 4 < 14; i++) {
		w[4 + i / 4] |= (uint32_t)em_id_log[i] << (8 * (i % 4));
	}
	/* The body's capability offer, in the last two words.  The id log is
	 * capped at 48 entries = 12 words (4..15), so this would collide if the
	 * log ever filled; it is bounded to 10 words here instead. */
	for (i = 0; i < 4; i++) {
		w[14] |= (uint32_t)em_body_offer[i] << (8 * i);
		w[15] |= (uint32_t)em_body_offer[4 + i] << (8 * i);
	}
	diag_page(page, w, 16);
}

void trail_norm(uint32_t page)
{
	uint32_t w[16];
	unsigned i;

	zero(w);
	w[0] = EM_DIAG_MAGIC;
	w[1] = EM_DIAG_FORMAT;
	w[2] = TAG_NORM;
	w[3] = ((uint32_t)em_norm_used << 16) | ((uint32_t)em_norm_overflow << 8);
	w[4] = (uint32_t)em_window_len[0] | ((uint32_t)em_window_len[1] << 8)
	       | ((uint32_t)em_window_len[2] << 16) | ((uint32_t)em_window_len[3] << 24);
	w[5] = ((uint32_t)em_window_used << 16) | em_truncated;
	for (i = 0; i < em_norm_used && i < 4; i++) {
		w[6 + i] = ((uint32_t)em_norm_count[i] << 16)
		           | ((uint32_t)em_norm_id[i] << 8) | em_norm_len[i];
	}
	/* Four (length, id) kinds is what a body sends -- 0x03, and the 0x04
	 * forms -- so the histogram stops at w[9] and the rest of the page
	 * carries the record counts.
	 *
	 * WHICH RECORDS the body actually sent, which is the question the
	 * frame-length histogram above can only answer for the one-record
	 * forms.  Two per word, and the event count beside them -- "the body
	 * asked N times and we replied M times" is one fact, not two. */
	w[10] = ((uint32_t)em_rec_n[EM_REC_SLOT_MOVE] << 16)
	        | em_rec_n[EM_REC_SLOT_STOP];
	w[11] = ((uint32_t)em_rec_n[EM_REC_SLOT_DRIVE] << 16)
	        | em_rec_n[EM_REC_SLOT_SCAN];
	w[12] = ((uint32_t)em_rec_n[EM_REC_SLOT_ROW] << 16)
	        | em_rec_n[EM_REC_SLOT_Q];
	w[13] = ((uint32_t)em_rec_unknown << 16)
	        | em_rec_n[EM_REC_SLOT_OTHER];
	/* The aperture channel: how many 0x1B frames arrived and what the last
	 * one asked for.  Zero frames is itself a result -- a body with no
	 * aperture range to command appears to stop asking. */
	/* THE TWO APERTURES THE BODY ASKS FOR, in one word.
	 *
	 * Message 0x03 pl[5..6] carries one every frame; message 0x1B carries
	 * one at the shutter press, when it carries one at all.  Neither can be
	 * obeyed here, so what matters is only whether either is ever something
	 * other than the aperture this device has -- and that is two numbers,
	 * not two counts.  The 0x1B count is on the FLST page already. */
	w[14] = ((uint32_t)em_m03_aperture << 16) | em_aperture_req;
	w[15] = ((uint32_t)em_events_sent << 16) | em_stop_n;
	diag_page(page, w, 16);
}

/* The body's message 0x08 request, raw.
 *
 * One page for one frame, because that frame carries a capability bit nothing
 * else does and it arrives exactly once per session.  Everything else about
 * 0x08 is in the reply, which is a static blob we send unchanged. */
void trail_m08(uint32_t page)
{
	uint32_t w[16];
	unsigned i;

	zero(w);
	w[0] = EM_DIAG_MAGIC;
	w[1] = EM_DIAG_FORMAT;
	w[2] = TAG_M08;
	w[3] = ((uint32_t)em_m08_n << 16) | ((uint32_t)em_m08_aperture_on << 8)
	       | em_m08_len;
	for (i = 0; i < EM_M08_LEN && i < em_m08_len; i++) {
		w[4 + i / 4] |= (uint32_t)em_m08[i] << (8 * (i % 4));
	}
	diag_page(page, w, 16);
}

void trail_chk(uint32_t page)
{
	uint32_t w[16];

	zero(w);
	w[0] = EM_DIAG_MAGIC;
	w[1] = EM_DIAG_FORMAT;
	w[2] = TAG_CHK;
	w[3] = DIAG_PAGES;               /* as this translation unit sees it */
	w[4] = diag_page_capacity();     /* as diag.c sees it */
	w[5] = diag_slot_base();
	/* PORT->DIR, so a dump can PROVE the pins were configured.  A botched
	 * edit once removed the board_init_pins() call entirely and nothing in
	 * the image or the trail would have shown it (NOTES.md §49). */
	w[6] = PORT->Group[0].DIR.reg;
	w[7] = PORT->Group[0].OUT.reg;
	w[8] = diag_src_id();
	diag_page(page, w, 16);
}

static void cap_page(uint32_t page, volatile const struct em_capture *c)
{
	uint32_t w[16];
	unsigned i;

	zero(w);
	w[0] = EM_DIAG_MAGIC;
	w[1] = EM_DIAG_FORMAT;
	w[2] = TAG_CAP;
	w[3] = ((uint32_t)c->id << 16) | ((uint32_t)c->got << 8) | c->declared;
	for (i = 0; i < EM_CAP_BYTES; i++) {
		w[4 + i / 4] |= (uint32_t)c->data[i] << (8 * (i % 4));
	}
	diag_page(page, w, 16);
}

void trail_cap_pre(uint32_t page, unsigned slot)
{
	cap_page(page, &em_pre[slot]);
}

void trail_cap(uint32_t page, unsigned slot)
{
	uint32_t w[16];
	unsigned i;

	zero(w);
	w[0] = EM_DIAG_MAGIC;
	w[1] = EM_DIAG_FORMAT;
	w[2] = TAG_CAP;
	w[3] = ((uint32_t)em_cap[slot].id << 16)
	       | ((uint32_t)em_cap[slot].got << 8) | em_cap[slot].declared;
	for (i = 0; i < EM_CAP_BYTES; i++) {
		w[4 + i / 4] |= (uint32_t)em_cap[slot].data[i] << (8 * (i % 4));
	}
	diag_page(page, w, 16);
}

/* The focus history: the last targets the body sent and the mode bytes it sent
 * them with.  Written at shutdown, which is where the questions have been --
 * the per-move pages only cover the first few moves of a session. */
void trail_fhist(uint32_t page)
{
	uint32_t w[16];
	unsigned i;

	zero(w);
	w[0] = EM_DIAG_MAGIC;
	w[1] = EM_DIAG_FORMAT;
	w[2] = TAG_FHIS;
	w[3] = millis();
	w[4] = ((uint32_t)em_tgt_ring_n << 16) | em_t04_n;
	w[5] = ((uint32_t)em_focus_n << 16) | em_mode_used;

	/* 10 targets, two per word: w[6]..w[10] */
	for (i = 0; i < EM_TGT_RING; i++) {
		w[6 + i / 2] |= (uint32_t)em_tgt_ring[i] << (16 * (i % 2));
	}
	/* up to 6 (mode byte, count) pairs: w[11]..w[15], packed count<<8|val,
	 * two per word */
	for (i = 0; i < em_mode_used && i < EM_MODE_SLOTS && 11 + i / 2 < 16; i++) {
		uint32_t v = ((uint32_t)em_mode_count[i] << 8) | em_mode_val[i];

		w[11 + i / 2] |= (v & 0xFFFFu) << (16 * (i % 2));
	}
	diag_page(page, w, 16);
}
