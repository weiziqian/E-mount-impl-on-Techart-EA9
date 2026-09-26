/* test_focus.c -- the encoder <-> message 0x06 map.
 *
 * The map is pure arithmetic with two clamps and a divide, which is exactly the
 * kind of code that looks obviously right and is off by one at the boundary.
 * It is also the piece a camera cannot tell us is wrong: a lens that reports a
 * plausible number in the wrong place just focuses badly.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "focus_map.h"

static int fails;

static void eq(const char *what, long got, long want)
{
	if (got != want) {
		printf("  FAIL %-44s got %ld, want %ld\n", what, got, want);
		fails++;
	}
}

/* The constants are NOT free parameters, and asserting them only symbolically
 * is no assertion at all: a mutation that moved the floor passed every other
 * test in this file, because the expectations moved with it.  Pin the literals
 * to the travel the adapter advertises. */
static void check_constants(void)
{
	eq("advertised floor", FOCUS_EM06_LO, 4144);
	eq("advertised ceiling", FOCUS_EM06_HI, 5632);

	/* 5952 counts / 4 = 1488 protocol units, which is exactly the travel
	 * the adapter advertises in message 0x06 (5632 - 4144).  This is what
	 * fixes the slope: a coarser one would otherwise go unnoticed. */
	eq("travel in protocol units matches message 0x06",
	   FOCUS_TRAVEL_COUNTS / FOCUS_COUNTS_PER_UNIT, 1488);
}

static void check_anchor(void)
{
	/* The one landmark the whole scale hangs on. */
	eq("infinity stop reports the floor", focus_counts_to_em06(0), FOCUS_EM06_LO);
	eq("floor commands the infinity stop", focus_em06_to_counts(FOCUS_EM06_LO), 0);

	/* A target below the floor is still infinity, not a negative move. */
	eq("below floor clamps to 0", focus_em06_to_counts(FOCUS_EM06_LO - 1), 0);
	eq("far below floor clamps to 0", focus_em06_to_counts(0), 0);
}

static void check_travel(void)
{
	eq("close stop -> ceiling",
	   focus_counts_to_em06(FOCUS_TRAVEL_COUNTS), FOCUS_EM06_HI);
	eq("ceiling -> close stop",
	   focus_em06_to_counts(FOCUS_EM06_HI), FOCUS_TRAVEL_COUNTS);

	/* Past either end must clamp, never wrap or run the mechanism on. */
	eq("beyond close stop clamps",
	   focus_em06_to_counts(FOCUS_EM06_HI + 500), FOCUS_TRAVEL_COUNTS);
	eq("counts beyond travel clamp",
	   focus_counts_to_em06(FOCUS_TRAVEL_COUNTS + 1000), FOCUS_EM06_HI);
	eq("negative counts clamp", focus_counts_to_em06(-500), FOCUS_EM06_LO);

	/* The no-target sentinel must land on the close stop rather than run
	 * off the end -- it is never driven to, but a clamp that let it through
	 * would be a full-travel slam. */
	eq("no-target sentinel clamps",
	   focus_em06_to_counts(FOCUS_NO_TARGET), FOCUS_TRAVEL_COUNTS);

	/* And the travel must actually span something useful -- a map that
	 * collapsed to a point would pass every clamp test above. */
	if (FOCUS_EM06_HI - FOCUS_EM06_LO < 256) {
		printf("  FAIL travel spans only %d protocol units\n",
		       FOCUS_EM06_HI - FOCUS_EM06_LO);
		fails++;
	}
}

static void check_monotonic_and_roundtrip(void)
{
	int32_t  c;
	uint16_t prev = 0;
	int      first = 1;

	for (c = 0; c <= FOCUS_TRAVEL_COUNTS; c += 7) {
		uint16_t p = focus_counts_to_em06(c);

		if (!first && p < prev) {
			printf("  FAIL not monotonic at %ld counts: %u after %u\n",
			       (long)c, p, prev);
			fails++;
			break;
		}
		prev = p;
		first = 0;
	}

	/* Round-trip: units -> counts -> units must be exact, because the
	 * protocol unit is the coarser of the two spaces.  (counts -> units ->
	 * counts is not, and must not be asserted: four counts share a unit.) */
	for (uint16_t p = FOCUS_EM06_LO; p <= FOCUS_EM06_HI; p++) {
		uint16_t back = focus_counts_to_em06(focus_em06_to_counts(p));

		if (back != p) {
			printf("  FAIL round trip: %u -> %ld -> %u\n",
			       p, (long)focus_em06_to_counts(p), back);
			fails++;
			break;
		}
	}
}

static void check_resolution(void)
{
	/* One protocol unit must be reachable: if the deadband in main.c were
	 * wider than a unit's worth of counts, the finest command the body can
	 * issue would be ignored and focus would quantise coarsely. */
	int32_t one_unit = focus_em06_to_counts(FOCUS_EM06_LO + 1);

	eq("one protocol unit in counts", one_unit, FOCUS_COUNTS_PER_UNIT);
	if (one_unit >= 12) {           /* FOCUS_DEADBAND_COUNTS in main.c */
		printf("  FAIL one unit (%ld counts) is not below the deadband\n",
		       (long)one_unit);
		fails++;
	}
}

int main(void)
{
	check_constants();
	check_anchor();
	check_travel();
	check_monotonic_and_roundtrip();
	check_resolution();

	printf("focus map: %d failure(s)\n", fails);
	return fails ? 1 : 0;
}
