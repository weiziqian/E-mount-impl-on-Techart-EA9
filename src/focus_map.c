#include "focus_map.h"

int32_t focus_em06_to_counts(uint16_t pos)
{
	int32_t c;

	if (pos <= FOCUS_EM06_LO) {
		return 0;
	}
	c = ((int32_t)pos - FOCUS_EM06_LO) * FOCUS_COUNTS_PER_UNIT;
	if (c > FOCUS_TRAVEL_COUNTS) {
		c = FOCUS_TRAVEL_COUNTS;
	}
	return c;
}

uint16_t focus_counts_to_em06(int32_t counts)
{
	if (counts < 0) {
		counts = 0;
	} else if (counts > FOCUS_TRAVEL_COUNTS) {
		counts = FOCUS_TRAVEL_COUNTS;
	}
	return (uint16_t)(FOCUS_EM06_LO + counts / FOCUS_COUNTS_PER_UNIT);
}
