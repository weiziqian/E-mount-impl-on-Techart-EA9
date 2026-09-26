/* test_focus_dist.c -- the distance code and the lens's model of it.
 *
 * Pure arithmetic with two fixed-point conversions in it, which is exactly the
 * kind of code that is plausible and wrong.  A camera cannot tell us it is
 * wrong either: a lens that reports a smooth, monotonic, entirely fictitious
 * distance looks healthy from the outside.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "focus_dist.h"
#include "focus_map.h"

static int fails;

static void fail(const char *what, long got, long want, long tol)
{
	printf("  FAIL %-46s got %ld, want %ld (+-%ld)\n", what, got, want, tol);
	fails++;
}

static void near(const char *what, long got, long want, long tol)
{
	if (labs(got - want) > tol) {
		fail(what, got, want, tol);
	}
}

/* --- the two fixed-point primitives -------------------------------------- */

static void check_log2(void)
{
	static const struct { uint32_t x; int32_t q6; } K[] = {
		{ 1u,        0 },        /* log2 1 = 0            */
		{ 2u,       64 },
		{ 1024u,   640 },
		{ 1000u,   638 },        /* the metres/mm offset  */
		{ 3u,      101 },        /* 64 * 1.58496 = 101.4  */
		{ 100u,    425 },        /* 64 * 6.64386 = 425.2  */
	};
	unsigned i;

	for (i = 0; i < sizeof(K) / sizeof(K[0]); i++) {
		near("log2_q6", focus_log2_q6(K[i].x), K[i].q6, 1);
	}

	/* Against the real thing, across the range distances actually use. */
	for (uint32_t x = 1; x < 4000000u; x += (x / 16) + 1) {
		int32_t want = (int32_t)(log2((double)x) * 64.0 + 0.5);

		if (labs(focus_log2_q6(x) - want) > 1) {
			fail("log2_q6 across the range", focus_log2_q6(x), want, 1);
			break;
		}
	}
}

static void check_exp2(void)
{
	near("exp2_q6(0)",    focus_exp2_q6(0),    1, 0);
	near("exp2_q6(64)",   focus_exp2_q6(64),   2, 0);
	near("exp2_q6(640)",  focus_exp2_q6(640),  1024, 1);
	near("exp2_q6(638)",  focus_exp2_q6(638),  1000, 2);

	/* Round trip: exp2(log2(x)) == x, to within the scale's own resolution
	 * of 1/64 of an octave, which is about 1.1%. */
	for (uint32_t x = 16; x < 4000000u; x += (x / 8) + 1) {
		uint32_t back = focus_exp2_q6(focus_log2_q6(x));
		uint32_t tol  = x / 64 + 2;

		if (back > x + tol || back + tol < x) {
			fail("exp2(log2(x)) round trip", back, x, tol);
			break;
		}
	}
}

/* --- the distance code --------------------------------------------------- */

#define F 500u                   /* 50.0 mm, what this adapter declares */

static void check_scale(void)
{
	/* The scale itself: 1 m is 384 and a doubling is 64.  Checked by
	 * finding the positions that focus at 1 m and 2 m and asking for their
	 * codes -- which exercises both directions at once. */
	int32_t at1 = focus_counts_for_code(384, F);
	int32_t at2 = focus_counts_for_code(448, F);

	near("1 m round trips to 384", focus_distance_code(at1, F), 384, 1);
	near("2 m round trips to 448", focus_distance_code(at2, F), 448, 1);

	/* 2 m is further, so it needs LESS extension. */
	if (at2 >= at1) {
		fail("2 m needs less extension than 1 m", at2, at1 - 1, 0);
	}

	/* The infinity stop reports infinity, and nothing else does. */
	near("infinity stop", focus_distance_code(0, F), FOCUS_DIST_INF, 0);
	if (focus_distance_code(1, F) == FOCUS_DIST_INF) {
		fail("one count off the stop still reads infinity",
		     focus_distance_code(1, F), 1000, 0);
	}
	near("the infinity code commands the stop",
	     focus_counts_for_code(FOCUS_DIST_INF, F), 0, 0);
	near("past the infinity code commands the stop",
	     focus_counts_for_code(0x7FFF, F), 0, 0);
}

static void check_monotonic_and_bounded(void)
{
	int32_t  c;
	uint16_t prev = 0;
	int      first = 1;

	/* More extension is always a nearer subject: the code must fall, never
	 * rise, as the mechanism runs from infinity to the close stop. */
	for (c = 1; c <= FOCUS_TRAVEL_COUNTS; c++) {
		uint16_t code = focus_distance_code(c, F);

		if (code >= FOCUS_DIST_INF) {
			fail("a finite position reported infinity", c, 0, 0);
			break;
		}
		if (!first && code > prev) {
			fail("distance code rose with extension", code, prev, 0);
			break;
		}
		prev = code;
		first = 0;
	}

	/* And the close stop is somewhere sane for a 50 mm with 4.5 mm of
	 * extension: u = f(f+x)/x = 50 * 54.5 / 4.5 = 605 mm.
	 * code = 384 + 64*log2(0.605) = 338. */
	near("close stop is about 0.6 m",
	     focus_distance_code(FOCUS_TRAVEL_COUNTS, F), 338, 2);
}

static void check_code_round_trip(void)
{
	/* code -> counts -> code, over the range the lens can actually reach.
	 *
	 * The tolerance is ONE COUNT, not a fixed number of code units.  Counts
	 * are the coarser grid far away -- near infinity a single count is
	 * worth several code units, and asserting a flat +-2 there fails on a
	 * conversion pair that is behaving perfectly.  So the round trip has to
	 * land inside the codes of the neighbouring counts. */
	uint16_t code;
	long     worst = 0, worst_at = 0;

	for (code = 338; code < 0x0600; code++) {
		int32_t  c    = focus_counts_for_code(code, F);
		uint16_t back = focus_distance_code(c, F);
		uint16_t lo, hi;

		if (c <= 0 || c >= FOCUS_TRAVEL_COUNTS) {
			continue;               /* clamped: nothing to check */
		}
		/* The code FALLS as counts rise, so c+1 is the low end. */
		lo = focus_distance_code(c + 1, F);
		hi = focus_distance_code(c - 1, F);

		if (back > hi || back < lo) {
			fail("code -> counts -> code left the count's own cell",
			     back, code, (long)(hi - lo));
			break;
		}
		if (labs((long)back - (long)code) > worst) {
			worst = labs((long)back - (long)code);
			worst_at = code;
		}
		/* Inside about 50 m the count grid is the finer of the two, so
		 * there the round trip really should be tight.  Beyond it one
		 * count spans tens of code units and no conversion can be. */
		if (code < 700 && labs((long)back - (long)code) > 2) {
			fail("round trip inside 50 m", back, code, 2);
			break;
		}
	}

	/* Reported, not asserted: it is a property of the two grids, not of
	 * this code.  Far from the lens a single encoder count spans many code
	 * units, so some codes name no reachable position at all -- the cell
	 * check above is what says the conversion picked the right count. */
	printf("  distance round trip: worst %ld code unit(s), at code %ld\n",
	       worst, worst_at);
}

static void check_coarse(void)
{
	/* pl[23]: -32 x dioptres, 0xFF at infinity. */
	near("coarse at the infinity stop", focus_distance_coarse(0, F), 0xFF, 0);

	/* At the close stop, 605 mm = 1.65 dpt, -32 x 1.65 = -53 = 0xCB. */
	near("coarse at the close stop",
	     (int8_t)focus_distance_coarse(FOCUS_TRAVEL_COUNTS, F), -53, 2);

	/* Monotonic the other way: more extension is more dioptres, i.e. a
	 * more negative byte. */
	{
		int8_t prev = 0;
		int    first = 1;

		for (int32_t c = 0; c <= FOCUS_TRAVEL_COUNTS; c += 7) {
			int8_t v = (int8_t)focus_distance_coarse(c, F);

			if (!first && v > prev) {
				fail("coarse distance rose with extension", v, prev, 0);
				break;
			}
			prev = v;
			first = 0;
		}
	}
}

static void check_focal_length_matters(void)
{
	/* The model is u = f(f+x)/x, so a longer lens at the same extension
	 * focuses further away.  If this ever stops being true the focal
	 * length has been dropped from the arithmetic. */
	uint16_t at50 = focus_distance_code(FOCUS_TRAVEL_COUNTS, 500);
	uint16_t at90 = focus_distance_code(FOCUS_TRAVEL_COUNTS, 900);

	if (at90 <= at50) {
		fail("a 90 mm focuses no further than a 50 mm at full extension",
		     at90, at50 + 1, 0);
	}
}

int main(void)
{
	check_log2();
	check_exp2();
	check_scale();
	check_monotonic_and_bounded();
	check_code_round_trip();
	check_coarse();
	check_focal_length_matters();

	printf("focus distance: %d failure(s)\n", fails);
	return fails ? 1 : 0;
}
