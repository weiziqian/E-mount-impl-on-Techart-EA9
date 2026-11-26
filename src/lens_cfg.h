/* lens_cfg.h -- the lens configuration block, and which entry is in use.
 *
 * ONE STRUCT IN THE IMAGE, found by its magic.  The updater searches the
 * release image for LENS_CFG_MAGIC and overwrites the block in place; nothing
 * else in the image needs patching.  Layout (2908 bytes, little-endian):
 *
 *   +0    magic[10]     "CfgAnchor" + NUL
 *   +10   hw_version    1 or 2
 *   +11   pad           0; alignment only
 *   +12   lens[8]       362 bytes each:
 *           +0  focal_mm      u16, whole mm; 0 = entry not set
 *           +2  name[64]      ASCII, NUL-padded; at most 63 characters
 *           +66 optical[296]  the entry's message 0x3B file (LF), built by the
 *                             updater from the focal length and the exit
 *                             pupil the user chose (tools/gen_optical.py is
 *                             the reference); all zero when the entry is not
 *                             set.  One that does not start "LF" makes the
 *                             firmware serve optical_file[] instead.
 *
 * At least one entry must be set -- the updater enforces it.  Entry 0 may be
 * unset when several are (f/2.5 then selects nothing); the firmware falls
 * back to the first set entry whenever the saved one is not usable.
 *
 * THE ENTRY IN USE is kept in flash as a cfg record (CFG_TYPE_LENS), whose
 * type tag and doubled value make a never-written or torn row read as "not
 * set".
 *
 * AT BOOT (lens_cfg_boot): the saved entry -- none, > 7, or not set -> the
 * first set entry, searching up from 0, and that is saved.  A single lens is
 * entry 0 alone, and needs no case of its own: it is always the entry chosen,
 * and f/2.5, which selects it, is already in use (lens_cfg_request).
 *
 * The minimum aperture is f/6.3 either way (EM_APERTURE_MIN).
 */
#ifndef LENS_CFG_H
#define LENS_CFG_H
#include <stdint.h>

#define LENS_CFG_MAGIC    "CfgAnchor"
#define LENS_CFG_N        8u
#define LENS_CFG_NAME_LEN 64u     /* 63 characters + NUL, as 0x3F pl[1..64] */
#define LENS_CFG_OPTICAL_LEN 296u /* = OPTICAL_FILE_LEN */

struct lens_cfg_lens {
	uint16_t focal_mm;
	char     name[LENS_CFG_NAME_LEN];
	uint8_t  optical[LENS_CFG_OPTICAL_LEN];
};

struct lens_cfg {
	char     magic[10];
	uint8_t  hw_version;
	uint8_t  pad;
	struct lens_cfg_lens lens[LENS_CFG_N];
};

/* In flash, volatile so no read of it is folded into code: the bytes the
 * updater writes are the only copy. */
extern const volatile struct lens_cfg lens_cfg;

uint8_t lens_cfg_hw_version(void);

/* Chooses the entry in use, as above, and applies it.  Before em_init, which
 * copies it into the messages. */
void lens_cfg_boot(void);

/* Saves `idx` as the entry in use (no flash write if unchanged).  Does not
 * apply it. */
void lens_cfg_select(uint8_t idx);

/* Puts entry `idx`'s focal length, name, lens ID and optical file into what
 * the body is sent: em_focal_mm10, message 0x3F, message 0x07's lens ID as the
 * template's plus `idx`, so a body that keys on the ID reloads after a
 * switch, and em_optical_file. */
void lens_cfg_apply(uint8_t idx);

/* SWITCHING BY APERTURE.  A 0x1B request for f/2.5, f/2.8 ... f/5.6 picks
 * entry 0..7, if that entry is set.  Called
 * from the frame handler; nothing is written yet.  lens_changed: the
 * selection now differs from the entry booted with.  After the 0x0A that ends
 * the capture, the main loop saves it (lens_cfg_commit) and restarts the lens
 * (main.c). */
void lens_cfg_request(uint16_t av);
void lens_cfg_commit(void);
extern volatile uint8_t lens_changed;

/* CLEARING HOME BY APERTURE (user's, 2026-11-26).  A 0x1B request for the
 * minimum aperture, f/6.3, sets lens_home_clear -- single lens or several.
 * The capture's 0x0A then takes the same restart as a lens switch, and that
 * restart clears the stored HOME first (lens_cfg_commit), so the next boot
 * runs the reset and stores a new one (main.c home_from_cfg). */
extern volatile uint8_t lens_home_clear;

#endif
