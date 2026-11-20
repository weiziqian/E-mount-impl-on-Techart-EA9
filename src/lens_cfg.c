#include "lens_cfg.h"
#include "cfg.h"
#include "emount.h"
#include "emount_packets.h"
#include "lens_cfg_optical.h"
#include "optical_file.h"

_Static_assert(sizeof(struct lens_cfg) == 2908, "lens_cfg layout changed");
_Static_assert(LENS_CFG_OPTICAL_LEN == OPTICAL_FILE_LEN, "optical file size");

/* Message 0x3F's name field: pl[1..63], frame offsets 7..69; pl[64] stays NUL. */
#define EM3F_NAME     7u
#define EM3F_NAME_LEN (LENS_CFG_NAME_LEN - 1u)

/* The aperture that selects each entry, as aperture values (256 x AV + 4096)
 * on the third-stop series, f/2.5 to f/5.6.  The highest set entry's is also
 * the minimum aperture.  Matched exactly, so these are the values a body was
 * logged sending: two thirds of a stop is 0xAA, not 0xAB, as the stock's own
 * focal menu (0x56cc) also expects. */
static const uint16_t AP_SEL[LENS_CFG_N] = {
	0x12AAu,                /* f/2.5 */
	0x1300u,                /* f/2.8 */
	0x1355u,                /* f/3.2 */
	0x13AAu,                /* f/3.5 */
	0x1400u,                /* f/4   */
	0x1455u,                /* f/4.5 */
	0x14AAu,                /* f/5   */
	0x1500u,                /* f/5.6 */
};

/* Message 0x07's lens ID, pl[9..10] = frame offsets 15..16, u16 LE. */
#define EM07_LENS_ID 15u

static uint8_t          g_top;          /* highest entry that may be chosen */
static uint8_t          g_boot_idx;     /* the entry booted with */
static volatile uint8_t g_sel_idx;      /* the entry last chosen */
volatile uint8_t        lens_changed;

__attribute__((used, aligned(4)))
const volatile struct lens_cfg lens_cfg = {
	.magic      = LENS_CFG_MAGIC,
	.hw_version = 2,
	.lens       = {
		{ 25, "Community EA9 25MM", LENS_CFG_OPTICAL_25MM },
		{ 28, "Community EA9 28MM", LENS_CFG_OPTICAL_28MM },
		{ 32, "Community EA9 32MM", LENS_CFG_OPTICAL_32MM },
		{ 35, "Community EA9 35MM", LENS_CFG_OPTICAL_35MM },
		{ 40, "Community EA9 40MM", LENS_CFG_OPTICAL_40MM },
		{ 45, "Community EA9 45MM", LENS_CFG_OPTICAL_45MM },
		{ 50, "Community EA9 50MM", LENS_CFG_OPTICAL_50MM },
		{ 56, "Community EA9 56MM", LENS_CFG_OPTICAL_56MM },
	},
};

/* The lowest set entry; 0 if none is (the updater never writes that). */
static uint8_t first_set(void)
{
	uint8_t i;

	for (i = 0; i < LENS_CFG_N; i++) {
		if (lens_cfg.lens[i].focal_mm != 0) {
			return i;
		}
	}
	return 0;
}

uint8_t lens_cfg_hw_version(void)
{
	return lens_cfg.hw_version;
}

void lens_cfg_boot(void)
{
	uint8_t  top = 0;
	uint32_t idx = 0;

	for (top = LENS_CFG_N - 1u; top > 0; top--) {
		if (lens_cfg.lens[top].focal_mm != 0) {
			break;
		}
	}
	em_aperture_min = top ? AP_SEL[top] : EM_APERTURE_MIN;

	if (top != 0) {
		if (!cfg_read(CFG_TYPE_LENS, &idx) || idx >= LENS_CFG_N
		    || lens_cfg.lens[idx].focal_mm == 0) {
			idx = first_set();
			lens_cfg_select((uint8_t)idx);
		}
	}
	g_top      = top;
	g_boot_idx = (uint8_t)idx;
	g_sel_idx  = (uint8_t)idx;
	lens_cfg_apply((uint8_t)idx);
}

void lens_cfg_request(uint16_t av)
{
	uint8_t i;

	for (i = 0; i <= g_top; i++) {
		if (av == AP_SEL[i]) {
			break;
		}
	}
	if (i > g_top || i == g_sel_idx || lens_cfg.lens[i].focal_mm == 0) {
		return;
	}
	g_sel_idx    = i;
	lens_changed = (i != g_boot_idx);
}

void lens_cfg_commit(void)
{
	lens_cfg_select(g_sel_idx);
}

void lens_cfg_select(uint8_t idx)
{
	cfg_write(CFG_TYPE_LENS, idx);
}

void lens_cfg_apply(uint8_t idx)
{
	const volatile struct lens_cfg_lens *l = &lens_cfg.lens[idx];
	unsigned i;
	char c = 1;

	uint16_t id = (uint16_t)(em_msg_07[EM07_LENS_ID]
	                         | (em_msg_07[EM07_LENS_ID + 1u] << 8)) + idx;

	em_msg_07[EM07_LENS_ID]      = (uint8_t)id;
	em_msg_07[EM07_LENS_ID + 1u] = (uint8_t)(id >> 8);
	em_focal_mm10 = (uint16_t)(l->focal_mm * 10u);
	em_optical_file = (l->optical[0] == 'L' && l->optical[1] == 'F')
	                  ? l->optical : optical_file;
	for (i = 0; i < EM3F_NAME_LEN; i++) {
		if (c) {
			c = l->name[i];
		}
		em_msg_3F[EM3F_NAME + i] = (uint8_t)c;
	}
}
