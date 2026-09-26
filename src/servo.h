/* servo.h -- closed-loop relative moves, with the mechanism's safety in mind.
 *
 * WHY THIS EXISTS RATHER THAN A TIMED PULSE.  The first motor test drove a
 * fixed duty for a fixed 200 ms with no feedback in the loop.  At 50% duty the
 * mechanism covers more than its entire advertised travel in that time, so the
 * pulse could only ever end against an end stop, and at 100% it over-travelled
 * hard enough that the stock firmware needed two power cycles before it would
 * drive again (NOTES.md §26).
 *
 * A closed-loop move cannot do that: it has a target, it stops there, and it
 * gives up the moment the mechanism stops responding.  Three independent
 * guards, because any one of them can be the one that matters:
 *
 *   target     stop on arrival, within `tolerance`
 *   stall      no movement for `stall_ms` while driving  ->  we are against a
 *              stop, or the duty is below stiction; release immediately
 *   runaway    travelled well past the target -> something is wrong with the
 *              feedback; release rather than keep pushing
 *   wrong way  moving AWAY from where it was asked to go.  If the encoder's
 *              sign is opposite the motor's, every other guard is satisfied --
 *              it is moving, it is not near the target, it has not overshot --
 *              and the mechanism is driven into a stop at a ramping duty for
 *              the whole timeout.  Found by testing, not by reading.
 *
 * And the duty RAMPS UP from a minimum rather than starting at the cap, so the
 * mechanism never gets more force than it needed to start moving, and the duty
 * at which it first moves is itself the measurement.
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

enum servo_outcome {
	SERVO_OK = 0,        /* reached the target */
	SERVO_STALL,         /* stopped responding -- an end stop, most likely */
	SERVO_TIMEOUT,       /* ran out of time without arriving */
	SERVO_RUNAWAY,       /* overshot far past the target */
	SERVO_WRONG_WAY,     /* moving AWAY from the target -- inverted feedback */
	SERVO_ABORTED,       /* the caller said stop; the target was not reached */
};

struct servo_cfg {
	uint16_t duty_start;   /* where the ramp begins */
	uint16_t duty_max;     /* and where it is capped -- keep this LOW */
	uint16_t duty_step;    /* added each time it fails to move */
	uint16_t ramp_ms;      /* how long to wait before stepping the duty up */
	/* Counts per ramp_ms below which the duty rises.  Must also clear the
	 * noise band: with +-3 counts of jitter, two readings can differ by 6
	 * from noise alone, and a threshold under that lets jitter masquerade
	 * as progress -- so a stiff mechanism never gets more duty and stalls
	 * instead of moving. */
	int16_t  min_progress;
	uint16_t stall_ms;     /* no movement for this long -> stall */
	uint16_t timeout_ms;
	int16_t  tolerance;    /* counts */

	/* Encoder jitter, in counts.  Movement must exceed this to count as
	 * movement at all.
	 *
	 * Without it the stall detector never fires: it reset on any change of
	 * reading, and a stationary encoder still jitters 2-3 counts, so a
	 * mechanism held against a stop looks like it is moving.  Every dry-run
	 * move timed out at 400 ms instead of stalling at ~180 ms (NOTES.md
	 * §29).  The host simulation had no noise, so the suite could not have
	 * caught this -- the hardware did. */
	int16_t  noise;

	/* Called every iteration.  NOT optional in a build that also talks to
	 * a camera: a move takes up to timeout_ms, and the body expects a
	 * status pair on every frame sync (~16 ms).  The first closed-loop run
	 * went silent for up to 440 ms per step, the body re-ran part of its
	 * init handshake mid-move and then stopped sending altogether
	 * (NOTES.md §27). */
	/* Receives the encoder reading the servo has JUST taken, so the caller
	 * can report it without going back to the hardware.  The servo reads
	 * the encoder once per iteration; a pump that read it again doubled the
	 * SPI traffic for a value that had not changed. */
	void (*pump)(int32_t pos);

	/* Dry run: execute the entire loop -- encoder polling, ramp logic,
	 * pump, timing -- but never attach the pins or energise a coil.
	 *
	 * This exists as a CONTROL.  The closed-loop build got power-cycled by
	 * the body and the open-loop one did not, and the two differ in several
	 * ways at once (NOTES.md §28).  A dry run holds every one of those
	 * constant except the coil current, which is the only way to find out
	 * whether the drive itself is what the body objects to. */
	uint8_t  dry;

	/* THE APPROACH SEGMENT.
	 *
	 * Driving flat out all the way to the target and then stopping does not
	 * stop the mechanism there: braking from full speed takes 470-850
	 * counts (NOTES.md §60), so every move overshot and the body spent its
	 * time correcting.
	 *
	 * Within `approach_counts` of the target the duty drops to
	 * `duty_approach` -- ONE rewrite of the compare registers, not a ramp.
	 * That distinction matters: a continuous ramp from 102 to 409 made the
	 * body power-cycle the adapter four times (§31), while "flat 800, then
	 * flat 200" ran clean.  This is the second shape.
	 *
	 * Keep duty_approach at or above the stock servo's own floor of 200
	 * (7.8%); below that the mechanism draws current without moving. */
	int32_t  approach_counts;
	uint16_t duty_approach;

	/* MID-MOVE RETARGETING.
	 *
	 * Without this the servo is deaf for the length of a move.  Measured
	 * moves run 50-200 ms, which at the body's ~16.7 ms frame is three to
	 * eleven frames during which a commanded correction is ignored, after
	 * which the mechanism drives to a target that is by then that stale.
	 *
	 * The stock does not do that.  Its parser raises a new-command flag for
	 * every message 0x04 except the 22-byte clock, and the executor opens
	 * with `if (flag) { mode = 0; motor_stop(); }` -- abort, then re-issue
	 * against the fresh target (EA9.md 3.2).
	 *
	 * Called once per iteration.  Returns 1 and writes an ABSOLUTE encoder
	 * count if the body has commanded somewhere new, 0 otherwise.  NULL
	 * disables retargeting entirely, which is what homing and the park want
	 * -- neither is following the body. */
	uint8_t (*retarget)(int32_t *target);

	/* ABORT.
	 *
	 * Called once per iteration, before the arrival test.  Returns 1 to
	 * stop the move where it stands; the outcome is SERVO_ABORTED and the
	 * brake goes on exactly as it does for any other ending.
	 *
	 * This is not the same thing as a retarget.  A retarget says "go
	 * somewhere else"; an abort says "stop", with no target at all, and a
	 * body that asks for it is entitled to have the mechanism stand still
	 * rather than finish the move it had already asked for.  NULL disables
	 * it, which is what homing and the park want -- neither is following
	 * the body, and neither may be interrupted by it. */
	uint8_t (*abort)(void);

	/* Bound on how many times one call may be redirected.  A body that
	 * retargets forever would otherwise keep the coils energised forever;
	 * past the cap the current leg finishes and the main loop picks up
	 * whatever came next. */
	uint8_t  retarget_max;

	/* Ignore a new target closer than this to the current one.  Matches the
	 * deadband the caller applies to a fresh move: without it, a body
	 * repeating the same target jitters the leg deadlines forever. */
	int32_t  retarget_min;

	/* A reversal goes through the brake for this long before driving the
	 * other way.  Slamming an H-bridge from full forward to full reverse
	 * puts supply plus back-EMF across the winding; the mechanism is small
	 * and the cost of being wrong about that is a dead adapter. */
	uint16_t reverse_brake_ms;

	/* Absolute ceiling on one call, across every leg.  timeout_ms bounds a
	 * single leg and is reset by each retarget, so on its own it bounds
	 * nothing.  0 = no separate ceiling. */
	uint16_t total_ms;
};

#define SERVO_TRACE_N 24

struct servo_result {
	int32_t  start;        /* unwrapped counts */
	int32_t  end;
	uint16_t duty_first;   /* the duty at which it started moving; 0 = never */
	uint16_t duty_peak;
	uint16_t ms;
	uint8_t  outcome;      /* enum servo_outcome */

	/* Raw encoder across the move, so speed and start-up latency can be
	 * read off rather than guessed.  Sampled inside the loop: the previous
	 * version recorded only the endpoints, which is not a trace. */
	uint16_t trace[SERVO_TRACE_N];
	uint8_t  trace_n;
	uint16_t trace_ms;     /* nominal spacing */

	/* How many times the body moved the goalposts, and where it finally
	 * wanted the mechanism.  A move that ends far from the delta it was
	 * asked for is not a fault if these say why. */
	uint8_t  retargets;
	int32_t  target;
};

void servo_move_rel(int32_t delta_counts, const struct servo_cfg *cfg,
                    struct servo_result *out);

#endif
