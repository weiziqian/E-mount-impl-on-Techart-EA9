/* focus_dist.h -- the distance code, and the lens's own model of it.
 *
 * The body can talk about focus in two units it does not have to understand:
 * the lens's position counts, and a DISTANCE CODE.  The code is a u16 on a
 * base-2 logarithmic scale (autofocus.md 2.5):
 *
 *     code = 384 + 64 * log2(D)          D in metres
 *
 * so 1 m is 384, each doubling adds 64, each halving subtracts 64, and
 * 0x0700 means infinity or "further than I can resolve".
 *
 * The lens owns the conversion in both directions, because it depends on the
 * optics.  Three things use it:
 *
 *   - message 0x05 pl[20..21], the subject distance, reported every frame
 *   - message 0x05 pl[23], the same distance as a coarse signed byte
 *   - the two query records 0x22 and 0x2E, whose answers ride in the tail of
 *     message 0x06
 *   - Move mode 3, which commands a position by naming a distance
 *
 * THE MODEL.  This adapter has no electrical contact with the M lens it
 * carries, so it cannot know that lens's focus ring.  What it does know is its
 * own helicoid extension and the focal length it declares, and for a lens
 * whose own ring is at infinity that is enough:
 *
 *     1/f = 1/u + 1/v,  v = f + x   =>   u = f * (f + x) / x
 *
 * with x the extension.  That is exact while the ring is at infinity and a
 * lower bound on extension otherwise, which is the honest position for an
 * adapter: the reported distance is then wrong in absolute terms but still
 * moves the right way by the right proportion, and MF assist and the distance
 * slider need change rather than accuracy (EA9.md 10.1).
 *
 * THE ONE ASSUMED NUMBER is FOCUS_TRAVEL_UM below.  Nothing in this project
 * has measured the helicoid's travel in micrometres -- EA9.md 10.1 step 2
 * lists it as an open item -- so it is stated here as a single constant to
 * correct rather than spread through the arithmetic.
 */
#ifndef FOCUS_DIST_H
#define FOCUS_DIST_H

#include <stdint.h>

/* The distance code for "infinity, or beyond what I can resolve". */
#define FOCUS_DIST_INF   0x0700u

/* Helicoid travel in micrometres, over FOCUS_TRAVEL_COUNTS encoder counts.
 * ASSUMED: the LM-EA9's published extension is 4.5 mm; it has not been
 * measured here.  Every distance this module produces scales with it. */
#define FOCUS_TRAVEL_UM  4500

/* log2(x) * 64, rounded to nearest, for x >= 1.  Exposed because it is the
 * part worth testing on its own. */
int32_t focus_log2_q6(uint32_t x);

/* 2^(q6 / 64), for q6 >= 0, saturating at 0x7FFFFFFF. */
uint32_t focus_exp2_q6(int32_t q6);

/* Encoder counts from the infinity stop -> distance code.  0 counts, and
 * anything whose code would run past the scale, report FOCUS_DIST_INF. */
uint16_t focus_distance_code(int32_t counts, uint16_t focal_mm10);

/* Distance code -> counts from the infinity stop, clamped into the travel.
 * FOCUS_DIST_INF and anything above it give 0, the infinity stop. */
int32_t focus_counts_for_code(uint16_t code, uint16_t focal_mm10);

/* The same distance as message 0x05 pl[23]: a signed byte, about
 * -32 x dioptres, 0xFF at infinity. */
uint8_t focus_distance_coarse(int32_t counts, uint16_t focal_mm10);

#endif
