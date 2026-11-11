#include "board.h"
#include "diag.h"
#include "nvm.h"

/* Above anything the linker allocates (bss ends well below this), and not
 * cleared by startup -- so it survives a warm reset but not a power cycle. */
#define NOINIT_MAGIC_ADDR   0x20003FF0u
#define NOINIT_COUNT_ADDR   0x20003FF4u
#define NOINIT_MAGIC        0x424F4F54u     /* "BOOT" */

uint32_t diag_src_id(void)
{
	return DIAG_SRC_ID;
}

uint32_t diag_page_capacity(void)
{
	return DIAG_PAGES;
}

/* Write one word into an erased page without disturbing the rest of it.
 *
 * Clearing the page buffer sets it to all-ones; storing one word leaves every
 * other byte 0xFF; and programming can only drive bits 1 -> 0, so those bytes
 * leave the existing flash content alone.  This is the standard flash-log
 * technique.  It does mean a page is programmed more than once between erases,
 * which Microchip documents as permitted for NVM but discourages as a habit --
 * 64 words over the life of a test session is well inside that.
 */
static void tally_write(uint32_t slot, uint32_t value)
{
	nvm_log_write(DIAG_TALLY_BASE + slot * 4u, &value, 1);
}

static uint8_t g_tally_boot;

void diag_tally_mark(uint8_t code, uint8_t data)
{
	volatile uint32_t *slots = (volatile uint32_t *)DIAG_TALLY_BASE;
	uint32_t k;

	for (k = 1; k < DIAG_TALLY_SLOTS; k++) {
		if (slots[k] == 0xFFFFFFFFu) {
			break;
		}
	}
	if (k == DIAG_TALLY_SLOTS) {
		return;      /* full: keep the history we have rather than wipe it */
	}
	tally_write(k, ((uint32_t)code << 24) | ((uint32_t)g_tally_boot << 8)
	               | data);
}

uint32_t diag_boot_tally(uint32_t rcause)
{
	uint32_t build_id = DIAG_BUILD_ID;

	volatile uint32_t *slots = (volatile uint32_t *)DIAG_TALLY_BASE;
	uint32_t stamp = DIAG_TALLY_STAMP | (build_id & 0x00FFFFFFu);
	uint32_t k, boots = 0;

	/* A different build -- or a blank row -- starts the tally over. */
	if (slots[0] != stamp) {
		nvm_log_erase_row(DIAG_TALLY_BASE);
		tally_write(0, stamp);
	}

	/* The boot index is how many BOOT marks are already there, so marks from
	 * other milestones do not shift it. */
	for (k = 1; k < DIAG_TALLY_SLOTS; k++) {
		if (slots[k] == 0xFFFFFFFFu) {
			break;
		}
		if ((slots[k] >> 24) == DIAG_MARK_BOOT) {
			boots++;
		}
	}
	if (k == DIAG_TALLY_SLOTS) {
		nvm_log_erase_row(DIAG_TALLY_BASE);
		tally_write(0, stamp);
		boots = 0;
	}

	g_tally_boot = (uint8_t)boots;
	diag_tally_mark(DIAG_MARK_BOOT, (uint8_t)rcause);
	return boots;
}

static uint32_t g_slot_base = DIAG_BASE;

uint32_t diag_slot_base(void)
{
	return g_slot_base;
}

uint32_t diag_begin(uint32_t boot_index)
{
	volatile uint32_t *magic = (volatile uint32_t *)NOINIT_MAGIC_ADDR;
	volatile uint32_t *count = (volatile uint32_t *)NOINIT_COUNT_ADDR;
	uint32_t n;

	if (*magic != NOINIT_MAGIC) {
		*magic = NOINIT_MAGIC;
		*count = 0;
	} else {
		*count = *count + 1;
	}
	/* Erase THIS boot's slot only.  Pages are written one at a time as the
	 * run progresses, so the last page present is where it stopped, and the
	 * other slot still holds the boot before this one. */
	g_slot_base = DIAG_BASE + (boot_index % DIAG_SLOTS) * DIAG_SLOT_SIZE;
	for (n = 0; n < DIAG_ROWS; n++) {
		nvm_log_erase_row(g_slot_base + n * DIAG_ROW_SIZE);
	}
	return *count;
}

void diag_page(uint32_t page, const uint32_t *w, uint32_t words)
{
	if (page >= DIAG_PAGES || words > DIAG_PAGE_SIZE / 4) {
		return;
	}
	nvm_log_write(g_slot_base + page * DIAG_PAGE_SIZE, w, words);
}
