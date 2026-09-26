/* trail.h -- the flash trail's page formats, shared by every build.
 *
 * These layouts are a contract with rebuild/tools/decode_proto_diag.py.  They
 * live in one place because two builds writing "the same" page format from two
 * copies of the code is exactly how a decoder ends up misreading one of them --
 * and this project has already lost a camera run to two objects disagreeing
 * about a constant (NOTES.md §21).
 */
#ifndef TRAIL_H
#define TRAIL_H

#include <stdint.h>

#define EM_DIAG_MAGIC   0x544F5250u   /* "PROT" */
/* Page format version, in w[1] of every page.
 *
 * 1 -> 2: the timed snapshot's w[15] changed from PM->RCAUSE to the
 * focus-channel tally.  The decoder must not read a version-1 snapshot as a
 * tally -- RCAUSE 1 (POR) renders as "0x04 x1", which is a fabricated
 * observation, and fabricated observations are worse than missing ones. */
#define EM_DIAG_FORMAT  5u

#define TAG_IDLOG       0x49444C47u
#define TAG_NORM        0x4E4F524Du
#define TAG_M08         0x3830304Du   /* "M008": the body's message 0x08 request */
#define TAG_CHK         0x43484B21u
#define TAG_CAP         0x43415000u
#define TAG_MOTR        0x4D4F5452u

/* Focus moves get their OWN tag rather than a flag inside a MOTR page.  The
 * first attempt marked them with w[15] == 1, which is also what the old
 * fixed-travel test wrote as its step number -- so a dump from the previous
 * build decoded as a focus move, complete with an invented 0x1B target.  A
 * discriminator has to be a value the other writer cannot produce. */
#define TAG_FOCS        0x464F4353u
#define TAG_FLST 0x464C5354u   /* the last few focus moves */
#define TAG_FHIS        0x46484953u    /* focus history: targets + mode bytes */
#define TAG_FAIL        0x4C494146u   /* "FAIL": the handshake did not complete */

/* Boot identity, carried in every page that has room for it. */
void trail_set_boot(uint32_t noinit_boot, uint32_t flash_boot);

void trail_idlog(uint32_t page);
void trail_norm(uint32_t page);
void trail_m08(uint32_t page);
void trail_fhist(uint32_t page);
void trail_chk(uint32_t page);
void trail_cap(uint32_t page, unsigned slot);
void trail_cap_pre(uint32_t page, unsigned slot);

#endif
