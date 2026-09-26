#include "focus_pred.h"

/* Velocity smoothing.  new = (new + old) / 2 -- one shift, no division, and it
 * halves the effect of a single noisy sample.  The encoder is read over a
 * synchronous-serial transaction inside a loop that also services the protocol,
 * so sample spacing is uneven and single-sample instantaneous velocity is
 * noisy; nothing here should track that noise into a field the body reads. */
static int32_t blend(int32_t old, int32_t fresh)
{
	int32_t d = fresh - old;

	/* Rounded, not truncated.  Plain (fresh - old) / 2 stops moving once
	 * the two are one Q8 tick apart, so the estimate parks permanently just
	 * short of the truth -- and that last tick is enough to turn "one frame
	 * to go" into "two", forever, on a move that is about to arrive. */
	return old + (d + (d >= 0 ? 1 : -1)) / 2;
}

/* Displacement over one frame, in counts, signed.  Split out because the
 * forecast and the countdown are the same quantity read two ways. */
static int32_t step_per_frame(const struct focus_pred *p, uint16_t period_ms)
{
	return (p->vel_q8 * (int32_t)period_ms) / 256;
}

void focus_pred_begin(struct focus_pred *p, int32_t pos, int32_t target,
                      uint32_t now)
{
	p->active   = 1;
	p->target   = target;
	p->last_pos = pos;
	p->last_ms  = now;

	/* Seeded with the SIGN of the commanded move, so the very first
	 * forecast already points the right way.  Measurement replaces the
	 * magnitude within a frame or two. */
	p->vel_q8 = (target >= pos) ? FOCUS_PRED_VEL0_Q8 : -FOCUS_PRED_VEL0_Q8;
}

void focus_pred_retarget(struct focus_pred *p, int32_t target)
{
	if (p->active) {
		p->target = target;
	}
}

void focus_pred_end(struct focus_pred *p)
{
	p->active = 0;
	p->vel_q8 = 0;
}

void focus_pred_sample(struct focus_pred *p, int32_t pos, uint32_t now)
{
	uint32_t dt = now - p->last_ms;

	if (!p->active) {
		return;
	}
	if (dt == 0) {
		return;         /* no time base, no velocity */
	}

	p->vel_q8   = blend(p->vel_q8, ((pos - p->last_pos) * 256) / (int32_t)dt);
	p->last_pos = pos;
	p->last_ms  = now;
}

int32_t focus_pred_ahead(const struct focus_pred *p, int32_t pos,
                         uint16_t period_ms)
{
	int32_t d, left;

	if (!p->active) {
		return pos;     /* idle: the two fields are equal, by design */
	}

	d    = step_per_frame(p, period_ms);
	left = p->target - pos;

	/* Bounded by the remaining distance, in both directions.  A forecast
	 * that predicts an overshoot is worse than no forecast at all: it tells
	 * the body's loop to correct for an error the mechanism is not going to
	 * make.  (Yongnuo gets the same bound structurally, by decrementing a
	 * remaining-distance counter in lockstep with the steps it simulates.) */
	if (left >= 0) {
		if (d < 0) {
			d = 0;
		} else if (d > left) {
			d = left;
		}
	} else {
		if (d > 0) {
			d = 0;
		} else if (d < left) {
			d = left;
		}
	}
	return pos + d;
}
