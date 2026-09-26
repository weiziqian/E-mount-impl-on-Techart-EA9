/* focus_pred.c on the host.
 *
 * The module is pure arithmetic with no hardware behind it, so everything it
 * can get wrong is testable here: the direction of the forecast, its bound
 * against the target, what it reports when idle, and the countdown's floor and
 * saturation.
 *
 * The bound is the one that matters.  A forecast that predicts an overshoot
 * asks the body to correct for an error the mechanism is not going to make --
 * worse than sending no forecast at all, which is what the stock does and what
 * this replaces. */
#include <stdio.h>
#include <stdlib.h>
#include "focus_pred.h"

static int failures;

static void fail(const char *what)
{
	printf("FAIL: %s\n", what);
	failures++;
}

#define PERIOD 17u

/* Drive the predictor along a constant-velocity move, sampling every `dt` ms,
 * and hand back the state at the end. */
static void run(struct focus_pred *p, int32_t from, int32_t to,
                int32_t counts_per_ms, uint32_t dt, uint32_t steps)
{
	int32_t  pos = from;
	uint32_t t   = 1000;
	uint32_t i;

	focus_pred_begin(p, from, to, t);
	for (i = 0; i < steps; i++) {
		int32_t d = counts_per_ms * (int32_t)dt;   /* signed */

		t  += dt;
		pos += d;
		if ((d >= 0 && pos > to) || (d < 0 && pos < to)) {
			pos = to;
		}
		focus_pred_sample(p, pos, t);
	}
}

int main(void)
{
	struct focus_pred p;
	int32_t ahead;

	/* --- idle ---------------------------------------------------------
	 * The forecast equals the position.  That is the whole of what a
	 * stationary lens sends, and it is what the motion status and the
	 * in-motion flag are derived from -- an idle predictor that forecast
	 * movement would report the lens as moving forever. */
	p.active = 0;
	p.vel_q8 = 12345;               /* stale velocity must not leak out */
	if (focus_pred_ahead(&p, 3000, PERIOD) != 3000) {
		fail("an idle predictor must forecast no movement");
	}

	/* --- direction, from the very first frame --------------------------
	 * begin() seeds the velocity from the sign of the commanded move, so a
	 * forecast exists before any two samples do.  Without the seed the first
	 * frame of every move would forecast zero. */
	focus_pred_begin(&p, 1000, 4000, 0);
	if (focus_pred_ahead(&p, 1000, PERIOD) <= 1000) {
		fail("a move toward a larger count must forecast forwards");
	}
	focus_pred_begin(&p, 4000, 1000, 0);
	if (focus_pred_ahead(&p, 4000, PERIOD) >= 4000) {
		fail("a move toward a smaller count must forecast backwards");
	}

	/* --- the forecast tracks a measured velocity ----------------------- */
	run(&p, 0, 100000, 10, 5, 40);           /* 10 counts/ms, far from the end */
	ahead = focus_pred_ahead(&p, 2000, PERIOD);
	if (ahead < 2000 + 10 * (int32_t)PERIOD * 8 / 10
	    || ahead > 2000 + 10 * (int32_t)PERIOD * 12 / 10) {
		fail("the forecast did not converge on the measured velocity");
	}

	/* --- the bound: never past the target ------------------------------
	 * Same fast move, but now 3 counts short of the target.  One frame at
	 * 10 counts/ms is 170 counts; the answer must be the target, not
	 * target+167. */
	if (focus_pred_ahead(&p, 100000 - 3, PERIOD) != 100000) {
		fail("the forecast ran past the target");
	}
	/* And in the other direction. */
	run(&p, 100000, 0, -10, 5, 40);
	if (focus_pred_ahead(&p, 3, PERIOD) != 0) {
		fail("a descending forecast ran past the target");
	}

	/* --- the bound survives an overshoot --------------------------------
	 * The mechanism coasts past the target: `left` flips sign while the
	 * velocity still points the old way.  Forecasting further in the
	 * direction of travel here would be a guaranteed-wrong number. */
	run(&p, 0, 4000, 10, 5, 40);
	if (focus_pred_ahead(&p, 4050, PERIOD) != 4050) {
		fail("past the target, the forecast must not add more travel");
	}

	/* --- a slower body -------------------------------------------------
	 * The period is measured, not assumed, so halving the frame rate must
	 * double the distance forecast. */
	run(&p, 0, 100000, 10, 5, 40);
	if (focus_pred_ahead(&p, 0, 34) - 0 <
	    2 * (focus_pred_ahead(&p, 0, 17) - 0) - 4) {
		fail("the forecast does not scale with the frame period");
	}

	/* --- end() clears the state ----------------------------------------
	 * The servo calls end() before the final report of a move.  If the
	 * velocity survived, every idle frame afterwards would keep forecasting
	 * movement that is over. */
	focus_pred_end(&p);
	if (focus_pred_ahead(&p, 1234, PERIOD) != 1234) {
		fail("end() left the predictor live");
	}

	/* --- samples with no time base -------------------------------------
	 * pump() runs far faster than the millisecond clock, so most samples
	 * arrive with dt == 0.  Those carry no velocity information and must be
	 * dropped, not treated as infinite speed. */
	focus_pred_begin(&p, 0, 100000, 500);
	focus_pred_sample(&p, 40, 500);
	focus_pred_sample(&p, 80, 500);
	if (p.vel_q8 != FOCUS_PRED_VEL0_Q8) {
		fail("a zero-duration sample changed the velocity");
	}

	printf("test_focus_pred: %d failure(s)\n", failures);
	return failures ? 1 : 0;
}
