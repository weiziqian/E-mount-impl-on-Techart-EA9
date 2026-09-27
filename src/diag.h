#ifndef DIAG_H
#define DIAG_H
#include <stdint.h>

/* EA9_NO_LOG -- the release configuration.
 *
 * Everything below writes to flash, and flash has a finite number of
 * erase/program cycles.  That is a fair price for a development instrument
 * used over a bench session; it is not a fair price for an adapter that sits
 * in a camera bag for years.  So the release build (`make release`) defines
 * EA9_NO_LOG, which turns this whole interface into inline no-ops and drops
 * diag.c and trail.c from the build entirely.  The callers keep their calls --
 * no #ifdef anywhere else in the tree -- and the compiler deletes the page
 * buffers they were filling, because nothing reads them any more.
 *
 * It is a compile-time removal rather than a runtime `if`, deliberately: a
 * runtime flag leaves the NVMCTRL command sequence in the image, and then
 * "does this build write flash?" is a question about control flow instead of
 * a question about symbols.  The Makefile checks the symbols. */

/* A progress trail in flash, so a run that produces no visible motion can still
 * be read back over USB afterwards.  FOUR slots of eight erased rows each at
 * DIAG_BASE, 64-byte pages; each page is written exactly once, so whichever
 * pages are present says how far the firmware got before it stopped.  Dump the
 * slots AND the tally row that follows them with
 *
 *   ea9flash.py --dump 0x16000:0x2100
 *
 * (0x16000 + 4 * 0x800 of slots, then 0x100 for the tally.)  The address here
 * used to say 0x1f000 and 0x800, from when there was one slot somewhere else;
 * following it dumps erased flash and decodes as an empty run. */

/* TWO trail slots, alternating by boot.
 *
 * diag_begin() used to erase everything on every boot, so the trail only ever
 * described the LAST boot.  When the body started power-cycling the adapter --
 * first boot failing, second one coming up without autofocus, reproducibly --
 * the evidence for the failing boot was erased by the boot that followed
 * (NOTES.md §27).  An instrument that destroys the thing it is there to record
 * is worse than no instrument.
 *
 * Boot N writes slot N % DIAG_SLOTS.
 *
 * Two slots was not enough.  With four boots in a session -- a failed one, a
 * good one, and a probe after each shutdown -- the odd boots shared a slot and
 * the failing boot was overwritten by the one two later.  Twice the evidence
 * that mattered was the evidence that got destroyed (NOTES.md §36, §37). */
#define DIAG_SLOT_SIZE  0x800u          /* 8 rows, 32 pages */
#define DIAG_SLOTS      4u
#define DIAG_BASE       0x00016000u
#define DIAG_ROW_BASE   DIAG_BASE       /* slot 0; diag_begin() re-bases */
#define DIAG_ROW_SIZE   256u
#define DIAG_ROWS       8u              /* 8 rows = 32 pages, erased together.
                                           Was 4; the normal-class tally needs
                                           room past the timed snapshots. */
#define DIAG_PAGE_SIZE  64u
#define DIAG_PAGES      (DIAG_ROWS * 4u)
#define DIAG_FORMAT     5u              /* bumped whenever the layout changes,
                                           so stale rows are never decoded */

/* Boot counter in RAM the startup code does not touch, so it survives a warm
 * reset.  If the device is brown-out resetting under motor load, successive
 * boots land in successive rows and the trail shows it. */
/* Erases THIS boot's slot only, leaving the previous boot's intact.  Pass the
 * boot index from diag_boot_tally(); returns the noinit boot counter. */
#ifdef EA9_NO_LOG
static inline uint32_t diag_begin(uint32_t boot_index)
{
	(void)boot_index;
	return 0;
}
static inline void diag_page(uint32_t page, const uint32_t *w, uint32_t words)
{
	(void)page; (void)w; (void)words;
}
#else
uint32_t diag_begin(uint32_t boot_index);
void     diag_page(uint32_t page, const uint32_t *w, uint32_t words);
#endif

/* DIAG_PAGES as diag.c was compiled with it.  Two objects can disagree about a
 * constant from this header if one of them was not rebuilt, and the link will
 * not say so -- that cost a camera run (NOTES.md §21).  The header
 * dependencies in the Makefile prevent it; this reports it if they ever fail
 * again, because a silent instrument is worse than none. */
#ifdef EA9_NO_LOG
static inline uint32_t diag_page_capacity(void) { return 0; }
static inline uint32_t diag_slot_base(void)     { return 0; }
#else
uint32_t diag_page_capacity(void);
uint32_t diag_slot_base(void);
#endif

/* A boot tally in a row diag_begin does NOT erase, so it survives the erase
 * that every boot performs.  The trail alone cannot distinguish "booted once"
 * from "booted ten times and this is the last" -- and the body power-cycling
 * the adapter looks exactly like a slow start from the outside.
 *
 * One 4-byte slot per boot, 64 slots in one row; when it fills, the row is
 * erased and counting restarts.  Returns the slot index, i.e. how many boots
 * have happened since the last wrap. */
/* Slot 0 holds a build stamp; boots start at slot 1.  Without it the tally
 * spans test sessions -- it survives a reflash, since flashing only writes the
 * app region -- and "2 boots" then means "one from the previous session and one
 * from this", which reads as the body restarting us when it did not.  A new
 * build stamps a new value and the row restarts. */
#define DIAG_TALLY_BASE   (DIAG_BASE + DIAG_SLOTS * DIAG_SLOT_SIZE)
#define DIAG_TALLY_SLOTS  64
#define DIAG_TALLY_STAMP  0xB1000000u

/* Identifies the build, so a freshly flashed image starts the tally over.
 *
 * It used to be __TIME__ of diag.c, which only changes when diag.c is
 * recompiled -- so every change that did not touch this file left the stamp
 * alone, the tally was never reset, and "boots since this image was flashed"
 * silently counted boots from previous flashes too.  Two runs' worth of boots
 * were read as one before that was noticed (NOTES.md §43).
 *
 * The Makefile now passes DIAG_BUILD_STAMP and forces this file to rebuild
 * every time, so it tracks the image rather than one translation unit. */
#ifdef DIAG_BUILD_STAMP
#define DIAG_BUILD_ID (DIAG_BUILD_STAMP & 0x00FFFFFF)
#else
#define DIAG_BUILD_ID ( ((__TIME__[0] - '0') * 10 + (__TIME__[1] - '0')) * 3600 \
                      + ((__TIME__[3] - '0') * 10 + (__TIME__[4] - '0')) * 60   \
                      + ((__TIME__[6] - '0') * 10 + (__TIME__[7] - '0')) )
#endif

/* The build id is diag.c's own business -- it is the file the Makefile forces
 * to rebuild so the stamp tracks the image.  Taking it as an argument meant it
 * was expanded in the CALLER, which is not force-rebuilt, so the forced
 * rebuild had no effect at all. */
#ifdef EA9_NO_LOG
static inline uint32_t diag_boot_tally(uint32_t rcause)
{
	(void)rcause;
	return 0;
}
#else
uint32_t diag_boot_tally(uint32_t rcause);
#endif

/* A milestone, appended to the tally row.  The row is never erased between
 * boots, so this is the only record that survives a boot being overwritten --
 * and the boots that fail are exactly the ones the two trail slots lose.
 *
 * Marks are one word: code, the boot index they belong to, and a byte of
 * payload.  A boot that only ever shows DIAG_MARK_BOOT died before the bus
 * came up; one that shows BOOT and BUSUP but no HANDSHAKE stalled in the
 * handshake; and so on. */
#define DIAG_MARK_BOOT      0xB0u   /* payload: RCAUSE                  */
#define DIAG_MARK_BUSUP     0xB2u   /* em_init returned                 */
#define DIAG_MARK_HANDSHAKE 0xB3u   /* message 0x0a answered            */
#define DIAG_MARK_SHUTREQ   0xB4u   /* message 0x16 acknowledged        */
#define DIAG_MARK_PA23      0xB5u   /* the power-down signal was driven */
/* NO BODY.  em_init blocks in a bare `while (!chip_select)` loop, so a boot on
 * a bench supply or on the USB flasher writes page 0 and then waits forever --
 * which in the trail is indistinguishable from a camera that cut power before
 * the bus came up.  Every dump this project has taken carries at least one of
 * these, and reading them as failures overstates the startup problem.
 *
 * A camera clocks its frame sync even before it raises chip select; a USB port
 * does not.  So a long wait with no frame sync at all says nobody is there.
 * Payload: the number of frame-sync edges seen, saturated at 255. */
#define DIAG_MARK_NOBODY    0xB6u

#ifdef EA9_NO_LOG
static inline void diag_tally_mark(uint8_t code, uint8_t data)
{
	(void)code; (void)data;
}
#else
void diag_tally_mark(uint8_t code, uint8_t data);
#endif

/* A hash of every source file, compiled in.  Two images with the same value
 * were built from identical source; two with different values were not --
 * which is the question "are these the same firmware?" and it was not
 * answerable before.  The build stamp cannot answer it: it changes on every
 * build by design, so the tally resets per flash. */
#ifndef DIAG_SRC_ID
#define DIAG_SRC_ID 0
#endif
#ifdef EA9_NO_LOG
static inline uint32_t diag_src_id(void) { return DIAG_SRC_ID; }
#else
uint32_t diag_src_id(void);
#endif

#endif
