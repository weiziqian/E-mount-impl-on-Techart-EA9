#ifndef FOCUS_PRED_H
#define FOCUS_PRED_H

#include <stdint.h>

/* The forward-looking field a native lens fills in and the stock LM-EA9 does
 * not: message 0x06 pl[2..3], the focus position ONE BODY FRAME AHEAD, against
 * pl[20..21]'s position now.  A real lens runs a motion simulation over one
 * measured frame period; the result is bounded by the commanded target and
 * equals the position when the mechanism is idle (autofocus.md 4.1, CERTAIN).
 *
 * It also answers "is focus moving", because the forecast differs from the
 * position exactly while something is in flight -- which is what message 0x06
 * pl[0] and message 0x05 pl[22] bit 6 report.
 *
 * A second field used to be derived here, message 0x05 pl[4].  That field is
 * the APERTURE settle countdown and has nothing to do with focus; this adapter
 * has no iris and sends a constant 0.  The countdown is gone.
 *
 * Deliberately a pure module with no encoder, timer or protocol dependency:
 * the servo hands it samples, and it is host-tested on its own.
 *
 * We do NOT reproduce Yongnuo's step-period ramp simulation.  That simulation
 * models one specific stepper's acceleration profile; ours is a DC motor with
 * an encoder, so a measured velocity is both simpler and more honest about what
 * the mechanism is actually doing. */

/* Counts per millisecond, Q8, assumed at the instant a move is commanded --
 * before any two samples exist to measure from.
 *
 * 8 counts/ms is the low end of what the observed moves ran at.  Seeding LOW
 * understates the forecast, which decays toward the truth within a frame or
 * two; seeding high would promise an arrival that does not come. */
#define FOCUS_PRED_VEL0_Q8   (8 << 8)

struct focus_pred {
	uint8_t  active;     /* a move is in progress */
	int32_t  target;     /* encoder counts, the commanded destination */
	int32_t  last_pos;   /* the last sample that advanced the clock */
	uint32_t last_ms;
	int32_t  vel_q8;     /* signed counts per millisecond, Q8 */
};

/* A move has been commanded.  `pos` is where the mechanism is now. */
void focus_pred_begin(struct focus_pred *p, int32_t pos, int32_t target,
                      uint32_t now);

/* The body redirected the move.  Only the destination changes -- the measured
 * velocity is still valid, because the mechanism has not changed speed yet. */
void focus_pred_retarget(struct focus_pred *p, int32_t target);

/* The move is over, however it ended. */
void focus_pred_end(struct focus_pred *p);

/* One position sample from the servo loop.  Samples taken within the same
 * millisecond carry no velocity information and are ignored. */
void focus_pred_sample(struct focus_pred *p, int32_t pos, uint32_t now);

/* Where the mechanism will be one frame from now.  Equals `pos` when idle, and
 * never lies past the target -- which is the property that makes the field safe
 * for a body to act on. */
int32_t focus_pred_ahead(const struct focus_pred *p, int32_t pos,
                         uint16_t period_ms);

#endif /* FOCUS_PRED_H */
