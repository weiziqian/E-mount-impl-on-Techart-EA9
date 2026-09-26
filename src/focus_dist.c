#include "focus_dist.h"
#include "focus_map.h"

/* 64 * log2(1000), rounded: the metres/millimetres offset.  Kept as a name
 * because it appears with the scale's own 384 and the two are easy to confuse.
 */
#define LOG2_Q6_1000  638

int32_t focus_log2_q6(uint32_t x)
{
	uint32_t m, t;
	int      e = 0;
	int32_t  frac = 0;
	int      i;

	if (x == 0) {
		return 0;
	}
	for (t = x; t >>= 1; ) {
		e++;                    /* e = floor(log2 x) */
	}

	/* Normalise to [1, 2) in Q14.  Q14 rather than Q16 so that m * m stays
	 * inside 32 bits: m < 2^15, so m*m < 2^30. */
	if (e <= 14) {
		m = x << (14 - e);
	} else {
		m = x >> (e - 14);
	}

	/* Seven bits, then round to six: repeated squaring, one bit per step.
	 * m^2 lands in [1, 4), and a result at or above 2 means this bit is 1. */
	for (i = 0; i < 7; i++) {
		m = (m * m) >> 14;
		frac <<= 1;
		if (m >= (2u << 14)) {
			m >>= 1;
			frac |= 1;
		}
	}
	return (((int32_t)e << 7) + frac + 1) >> 1;
}

uint32_t focus_exp2_q6(int32_t q6)
{
	/* 2^(1/2), 2^(1/4) ... 2^(1/64), in Q16. */
	static const uint32_t ROOT[6] = {
		92682u, 77936u, 71468u, 68438u, 66972u, 66249u
	};
	/* The accumulator is Q14, NOT Q16.  Q16 overflows: r would reach 2^17
	 * and the largest root is 92682, and 2^17 * 92682 is past 2^32.  In Q14
	 * the product stays under 3.1e9. */
	uint32_t r = 1u << 14;
	int32_t  e;
	int      i;

	if (q6 < 0) {
		return 0;
	}
	e = q6 >> 6;
	/* The low six bits are the fraction; bit 5 is worth 2^(1/2). */
	for (i = 0; i < 6; i++) {
		if (q6 & (1 << (5 - i))) {
			r = (r * ROOT[i]) >> 16;
		}
	}
	/* r is Q14 in [1, 2).  The result is r * 2^e as an integer, shifted in
	 * whichever direction keeps it inside 32 bits. */
	if (e > 30) {
		return 0x7FFFFFFFu;     /* far past anything this lens can mean */
	}
	if (e <= 14) {
		return r >> (14 - e);
	}
	if (e > 31) {
		return 0x7FFFFFFFu;
	}
	return r << (e - 14);
}

/* The extension, in micrometres, for a position measured from the infinity
 * stop.  Rounded rather than truncated: at the infinity end one count is
 * worth about 0.76 um and truncation would report several counts as zero
 * extension, i.e. as infinity. */
static uint32_t extension_um(int32_t counts)
{
	if (counts <= 0) {
		return 0;
	}
	if (counts > FOCUS_TRAVEL_COUNTS) {
		counts = FOCUS_TRAVEL_COUNTS;
	}
	return ((uint32_t)counts * FOCUS_TRAVEL_UM + FOCUS_TRAVEL_COUNTS / 2)
	       / FOCUS_TRAVEL_COUNTS;
}

/* Subject distance in millimetres: u = f * (f + x) / x. */
static uint32_t distance_mm(int32_t counts, uint16_t focal_mm10)
{
	uint32_t x_um = extension_um(counts);
	uint32_t f_um = (uint32_t)focal_mm10 * 100u;   /* mm x 10 -> um */
	uint32_t f_mm = focal_mm10 / 10u;

	if (x_um == 0 || f_mm == 0) {
		return 0;               /* caller reads 0 as infinity */
	}
	return (f_mm * (f_um + x_um)) / x_um;
}

uint16_t focus_distance_code(int32_t counts, uint16_t focal_mm10)
{
	uint32_t d_mm = distance_mm(counts, focal_mm10);
	int32_t  code;

	if (d_mm == 0) {
		return FOCUS_DIST_INF;
	}
	code = 384 + focus_log2_q6(d_mm) - LOG2_Q6_1000;
	if (code < 0) {
		code = 0;
	}
	if (code >= (int32_t)FOCUS_DIST_INF) {
		return FOCUS_DIST_INF;
	}
	return (uint16_t)code;
}

int32_t focus_counts_for_code(uint16_t code, uint16_t focal_mm10)
{
	uint32_t d_mm, x_um, f_um = (uint32_t)focal_mm10 * 100u;
	uint32_t f_mm = focal_mm10 / 10u;
	int32_t  counts;

	if (code >= FOCUS_DIST_INF || f_mm == 0) {
		return 0;               /* the infinity stop */
	}
	/* D in millimetres, from the code. */
	d_mm = focus_exp2_q6((int32_t)code - 384 + LOG2_Q6_1000);
	if (d_mm <= f_mm) {
		return FOCUS_TRAVEL_COUNTS;   /* closer than the lens can reach */
	}

	/* x = f * f / (D - f), the inverse of distance_mm. */
	x_um = (f_mm * f_um) / (d_mm - f_mm);
	if (x_um >= FOCUS_TRAVEL_UM) {
		return FOCUS_TRAVEL_COUNTS;
	}
	counts = (int32_t)((x_um * FOCUS_TRAVEL_COUNTS + FOCUS_TRAVEL_UM / 2)
	                   / FOCUS_TRAVEL_UM);
	if (counts > FOCUS_TRAVEL_COUNTS) {
		counts = FOCUS_TRAVEL_COUNTS;
	}
	return counts;
}

uint8_t focus_distance_coarse(int32_t counts, uint16_t focal_mm10)
{
	uint32_t d_mm = distance_mm(counts, focal_mm10);
	int32_t  v;

	if (d_mm == 0) {
		return 0xFF;            /* infinity: -1 */
	}
	/* -32 x dioptres, dioptres = 1000 / D_mm. */
	v = -(int32_t)((32u * 1000u + d_mm / 2u) / d_mm);
	if (v < -128) {
		v = -128;
	}
	if (v > -1) {
		v = -1;                 /* 0xFF is the infinity end of the scale */
	}
	return (uint8_t)v;
}
