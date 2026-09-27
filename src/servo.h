/* servo.h -- closed-loop relative moves, with the mechanism's safety in mind.
 *
 * WHY THIS EXISTS RATHER THAN A TIMED PULSE.  The first motor test drove a
 * fixed duty for a fixed 200 ms with no feedback in the loop.  At 50% duty the
 * mechanism covers more than its entire advertised travel in that time, so the
 * pulse could only ever end against an end stop, and at 100% it over-travelled
 * hard enough that the stock firmware needed two power cycles before it would
 * drive again.
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

/* THE CONTROL LAW, and where every number in it came from.
 *
 * Two levels, the same cascade the stock firmware uses (servo.md a measurement run), because
 * measurements on the camera said the stock's structure is right.  What is
 * different is that every constant below is measured rather than guessed, and
 * that the inner loop is given a feedforward term so it does not have to earn
 * the drive from accumulated error.
 *
 *   every SERVO_STEP_MS:
 *       e        = target - pos
 *       v_target = clamp(e * 1000 / stop_ms, +-v_cruise)      counts/s
 *       v_meas   = (pos - pos_last) * 1000 / dt
 *       ev       = v_target - v_meas
 *       vff      = v_target + tau_ms * (v_target - v_prev) / dt
 *       boost   += (keeping up) ? -boost_down : +boost_up
 *       duty     = vff / ff_div + sign(v_target) * (ff_offset + boost)
 *                  + ev / kp_div
 *       if |duty| < duty_floor:  duty = sign * duty_floor, or 0 if braking
 *
 * Every constant below was measured on the adapter itself, mounted on an
 * a9 II.
 */
struct servo_cfg {
	/* THE PROFILE.  v_target = remaining / stop_ms.
	 *
	 * 42 ms is the worst stopping distance measured, per count/ms of entry
	 * speed, over eight cases in two runs.  A profile that never commands
	 * more than this can be stopped BY FRICTION ALONE from anywhere on the
	 * curve -- no brake, no reverse drive -- so it does not depend on the
	 * loop working, or on which friction model is right.  (It is not:
	 * fitting the exponent gives d ~ v^1.52, between viscous and constant
	 * deceleration, and neither pure model fits.)
	 *
	 * The stock commands e/22, twice as fast.  It gets away with that
	 * because its staircase has already dropped the speed long before the
	 * error is small.
	 *
	 * 42 was the first choice and it had NO MARGIN BY CONSTRUCTION: it is
	 * the worst coast, so a profile at e/42 asks the mechanism to arrive
	 * with exactly zero to spare.  60 followed, and it is what the first
	 * autofocus run on a camera found too slow.
	 *
	 * WHY IT WAS THE SLOWNESS.  Below the floor duty the mechanism cannot
	 * be driven at all, and the floor's own speed is about 2.6 counts/ms
	 * -- so wherever the profile asks for less than that, the move stops
	 * being driven and becomes a pulse-and-coast.  At e/60 that regime
	 * starts 156 counts from the target; at e/30, 78.  Every move was
	 * spending a fixed 250-450 ms creeping through it, and a 69-count
	 * move took LONGER than a 1660-count one.
	 *
	 * 30, chosen on the user's judgement after shooting all three.
	 *
	 * The logs cannot separate them.  Over 22-24 moves each, on the
	 * camera:
	 *
	 *     e/30   mean |error| 6.0 counts, sd 6.6, worst 12, median 248 ms
	 *     e/26   mean |error| 7.3,        sd 8.0, worst 12, median 248
	 *     e/22   mean |error| 7.4,        sd 8.3, worst 12, median 220
	 *
	 * e/30 has the smallest spread and the user reported it as the best
	 * of the three to shoot with, but 1.3 counts of mean error is inside
	 * the run-to-run scatter and the difference is not established.  What
	 * IS established is that all three are far better than the e/60 this
	 * line started at, and that the remaining differences are small
	 * enough to settle on feel.
	 *
	 * The sweep below is kept because it is what the simulator says, and
	 * it is worth seeing how confidently it said it.  Swept over the move
	 * sizes a camera actually commands, crossed with breakaway values of
	 * 200, 340 and 536:
	 *
	 *     e/40   222 ms mean   worst error 24   peak 23
	 *     e/30   197 ms        worst error 26   peak 26
	 *     e/22   159 ms        worst error 17   peak 17
	 *     e/18   156 ms        worst error 16   peak 16
	 *
	 * The accuracy improves because a faster profile carries the
	 * mechanism THROUGH the arrival band instead of leaving it to creep
	 * the last stretch, and creeping through the dead zone is where the
	 * error was coming from.  18 buys nothing more.
	 *
	 * TWO HONEST COSTS, because "the stock uses e/22" is only half true.
	 * The stock's stage 1 is a four-rung STAIRCASE and e/2-per-11ms
	 * applies only below 100 counts (servo.md a measurement run).  Between 100 and 600
	 * the stock commands 4.5 and 9.1 counts/ms where e/22 asks for 13.6
	 * and 27 -- so there we are three times the stock, having already
	 * been twice it at e/30.  And 22 ms is shorter than the mechanism's
	 * own driven time constant of 26, so the reference is now faster
	 * than the plant and the tau feedforward is what closes the gap.
	 * Neither is fatal; both are reasons to read the next camera run
	 * carefully rather than to assume this one is free. */
	uint16_t stop_ms;

	/* Speed cap, counts per SECOND.  The mechanism reaches 61 counts/ms at
	 * 65% duty; the stock's own staircase never asks for more than 22.7,
	 * which this model puts at duty 724 -- so the stock does not use the
	 * range it allows itself either. */
	int32_t  v_cruise;

	/* FEEDFORWARD: duty = |v| / ff_div + ff_offset, signed.
	 *
	 * From v = 0.0399 * (duty - 134) counts/ms, fitted over twenty points,
	 * both directions, both runs.  1/0.0399 = 25 duty per count/ms, and
	 * with v in counts/s that is a divisor of 40.
	 *
	 * THE STOCK HAS NO FEEDFORWARD, and this is the single biggest change.
	 * Its PID has to accumulate the drive from error, so every move starts
	 * at the floor and climbs -- which is tolerable for a long move and is
	 * why short ones are slow and unreliable. */
	uint16_t ff_div;
	uint16_t ff_offset;

	/* THE MECHANISM'S OWN TIME CONSTANT, milliseconds.  33, measured.
	 *
	 * Without this the feedforward asks for the drive that would HOLD the
	 * commanded speed, when what the profile actually wants is a speed
	 * that is falling.  Holding and decaying need very different drives,
	 * and at these numbers they differ by a factor of three.
	 *
	 * The plant is tau * dv/dt + v = k * u, so the drive that produces a
	 * given v(t) is u = (v + tau * dv/dt) / k.  Leaving the second term
	 * out costs exactly the lag it describes: the mechanism is still
	 * travelling at the speed commanded tau ago, which is higher, and it
	 * arrives long.  Measured in simulation with it missing: 74 counts
	 * past a 500-count move, 185 past 1000, 278 past 3000 -- an error
	 * proportional to the distance, which is the signature of a tracking
	 * lag rather than of a stopping problem.
	 *
	 * This is also, in effect, what the stock's derivative gain was doing.
	 * Done from the reference instead of from the measurement, it adds no
	 * noise at all. */
	uint16_t tau_ms;

	/* THE SPEED AT WHICH ARRIVAL MAY BE DECLARED, counts/s.
	 *
	 * Position alone is not enough, and this was the one real defect the
	 * first hardware run of this controller found.  Every 200-count move
	 * crossed into tolerance while still travelling 2.7-6.8 counts/ms,
	 * declared OK, braked -- and then coasted 78-87 counts past, because
	 * a braked stop from that speed takes about thirty times it.  The
	 * forward moves happened to cross slower and looked fine; the reverse
	 * ones did not, which is why it read as a direction problem.
	 *
	 * So arrival also requires the mechanism to be slow enough that the
	 * coast cannot carry it out of tolerance: v <= tolerance / coast_T.
	 * With a 16-count tolerance and a 33 ms coast that is about 0.5
	 * counts/ms. */
	int32_t  v_arrive;

	/* HOW MANY CONSECUTIVE CONTROL STEPS the arrival condition must hold
	 * before it is believed.
	 *
	 * Speed alone is not enough either, because a mechanism reversing
	 * direction passes through zero speed on its way.  That was caught on
	 * hardware: a move overshot to 7 counts past the aim, the loop was
	 * already driving it back hard, and at the top of the arc the measured
	 * speed was momentarily nothing -- so the servo declared arrival, let
	 * go, and the mechanism carried on backwards for another 110 counts.
	 *
	 * Standing still is not the same as having stopped.  Requiring the
	 * condition to hold for a few steps tells them apart: a turnaround is
	 * slow for one step, an arrival is slow for as long as you care to
	 * look. */
	uint8_t  arrive_steps;

	/* PROPORTIONAL on velocity error -- a trim, not the main actuator.
	 *
	 * The plant is 39.9 counts/s of steady speed per unit of duty, so the
	 * loop gain is 39.9 / kp_div and the closed-loop time constant is
	 * tau / (1 + that).  At a 5 ms control period there is very little
	 * room: kp_div of 30 already pulls the closed loop down to 1.7
	 * samples, which is where a sampled loop starts to ring.  50 keeps it
	 * near 2.2 samples.
	 *
	 * That is the real reason the feedforward has to be good.  It is not
	 * an optimisation -- the sample rate will not permit a loop gain
	 * large enough to cover for a bad one. */
	uint16_t kp_div;

	/* THE BREAKAWAY BOOST -- a floor that climbs, and NOT an integrator.
	 *
	 * Breakaway is a DISTRIBUTION: twelve ramp trials gave 180..340, the
	 * worst needing 70% more than the median.  No fixed
	 * duty can serve that, so the drive has to climb until the mechanism
	 * goes.
	 *
	 * An ordinary integrator on the velocity error does climb -- it is how
	 * the stock does it -- but its RATE is proportional to the commanded
	 * speed, and the commanded speed is proportional to the distance left.
	 * So a long move is pushed hard and a short one is barely pushed at
	 * all.  servo.md a measurement run records that as a feature of the stock; measured
	 * against this profile it is a defect, and a plain integrator here
	 * left every move of 50 or 100 counts to stall while the duty crept
	 * up for half a second.
	 *
	 * So the climb is at a FIXED rate instead, in duty per control step,
	 * whenever the mechanism is being asked to move and is not keeping up.
	 * boost_up of 8 crosses the whole measured breakaway band in about
	 * 100 ms no matter how far the move is going.  It bleeds off again at
	 * boost_down once the mechanism is keeping pace, so the force that
	 * broke it loose does not stay applied. */
	uint16_t boost_up;
	uint16_t boost_down;
	uint16_t boost_max;

	/* TWO IDEAS TRIED HERE AND REMOVED, recorded so they are not had
	 * again by accident:
	 *
	 *   duty_kick         a floor under the first step, 40% of PER
	 *   boost_drop_above  reset the boost the instant the mechanism goes
	 *
	 * With a climb rate raised to 2.5% per cycle they simulated as a
	 * large win -- 3424 ms to 1514 against a stiff mechanism.  On the
	 * camera they were much WORSE: a 44-count move took 1101 ms and was
	 * cut short by the body, oscillating around its target the whole
	 * time, and the median move went from 208 ms to 402.
	 *
	 * More force breaks the mechanism loose sooner and then overshoots,
	 * and correcting an overshoot costs far more than the breakaway
	 * saved.  The simulator could not see it because its low-speed
	 * coast -- where the whole endgame lives -- is the part of the model
	 * with the least measurement behind it. */

	/* BELOW THIS THE OUTPUT IS ZERO -- not raised to it.
	 *
	 * The stock raises any non-zero output to 200, because below about
	 * that the mechanism draws current without moving.
	 * Mapping to zero instead has the same effect on the dead zone and one
	 * more: a command that is trying to DECELERATE is not pushed back up
	 * into accelerating.  The integrator still climbs through the zone, so
	 * the breakaway ramp survives. */
	uint16_t duty_floor;
	uint16_t duty_max;

	/* THE ARRIVAL BAND, counts.  It must be SMALL, and the reason is that
	 * a move only ever approaches its target from one side: the servo
	 * stops at the near edge of the band, so whatever this is, it becomes
	 * a systematic UNDERSHOOT of about that much.
	 *
	 * It was raised to 32 to stop a move reporting STALL while sitting 17
	 * counts out, and that was the wrong lever.  On hardware every one of
	 * 22 moves then came up short, and a 40-count move could finish after
	 * travelling 8 -- the band was most of the move.  A
	 * systematic bias is worse for autofocus than a larger random error,
	 * because the body has to correct it every single time.
	 *
	 * The stall it was meant to fix is handled by `settle_counts` below,
	 * which is where that problem actually lives. */
	int16_t  tolerance;

	/* A STALL THIS CLOSE TO THE TARGET IS AN ARRIVAL, counts.
	 *
	 * Below the floor duty nothing moves, and at the floor the mechanism
	 * covers 13 counts in one control step -- so the last stretch is a
	 * pulse-and-coast and it can run out of ways to get closer while
	 * still a few counts out.  That is the mechanism reaching its
	 * resolution, not a fault, and reporting STALL for it would have the
	 * caller treat a good position as a failure.
	 *
	 * Distinct from `tolerance` on purpose: the servo keeps TRYING until
	 * it is within tolerance, and only accepts the wider band when it has
	 * demonstrably stopped being able to do better. */
	int16_t  settle_counts;

	/* AIM THIS MUCH BEYOND THE TARGET, in the direction of travel.
	 *
	 * A move approaches from one side and stops at the first sample
	 * inside the arrival band, so it stops at the NEAR EDGE and the
	 * expected error is exactly minus the band width.  That is geometry,
	 * not noise, and two hardware runs agree with it to a count: -12 and
	 * -14 against a band of 12.
	 *
	 * So aim a band's width further on and the stopping point lands on
	 * the target instead of short of it.  Set it to `tolerance` to cancel
	 * the effect exactly; 0 disables it.
	 *
	 * IT SCALES WITH THE PROFILE, and at stop_ms 30 the right value is
	 * ZERO.  The near-edge bias only exists when the mechanism arrives
	 * slowly enough to stop where it first enters the band; a faster
	 * profile still has it moving, so it carries through on its own and
	 * an offset on top becomes an overshoot.  Measured over the camera's
	 * own move sizes: at stop_ms 60 an offset of 12 takes the bias from
	 * -12 to -4, and at stop_ms 30 the same offset takes it from +1 to
	 * +11.  The field stays because the geometry is real and returns
	 * whenever the profile is slowed.
	 *
	 * Only the profile and the arrival test use the aim.  The runaway and
	 * wrong-way guards, and everything reported, use the real target --
	 * a guard that fired against a shifted target would be describing a
	 * place the caller never asked for. */
	int16_t  aim_ahead;

	/* Encoder jitter, counts.  Measured 3-5 peak to peak, sigma 0.8, over
	 * four no-drive cases in two runs.  8 is comfortably clear of it. */
	int16_t  noise;

	/* No movement for this long -> stall.
	 *
	 * 300 ms, where the old bang-bang used 60.  It HAS to exceed the
	 * integrator's climb: sitting still with the duty rising is what
	 * breaking away looks like, and a 60 ms stall detector would abort the
	 * ramp before it did its job. */
	uint16_t stall_ms;
	uint16_t timeout_ms;
	uint16_t total_ms;       /* across every leg; 0 = none */

	/* Called every iteration with the reading just taken.  On a camera it
	 * is not optional -- see the note on cfg->pump in a measurement run. */
	void (*pump)(int32_t pos);

	/* Mid-move retargeting and abort, unchanged.  Both are polled before
	 * the arrival test, so the servo cannot declare success against a
	 * target the body has already replaced or cancelled. */
	uint8_t (*retarget)(int32_t *target);
	uint8_t (*abort)(void);
	uint8_t  retarget_max;
	int32_t  retarget_min;

	/* A reversal goes through the brake for this long before driving the
	 * other way -- but ONLY when it is a genuine slam, i.e. the mechanism
	 * is still moving faster than reverse_brake_above (counts/s) the old
	 * way.  Measured cost of doing it unconditionally: 362 counts of extra
	 * overrun, twice the turnaround distance, in two measured runs.
	 *
	 * The old servo applied it to every retarget that changed sign.  It
	 * could afford to, because its direction was latched per leg and it
	 * could not correct an overshoot at all; this one's profile reverses
	 * by itself whenever it goes past the target, and putting a 20 ms
	 * brake in the middle of every small correction would be absurd. */
	uint16_t reverse_brake_ms;
	int32_t  reverse_brake_above;

	/* Dry run: the whole loop, with no coil ever energised.  Kept as the
	 * control it has always been. */
	uint8_t  dry;
};

#define SERVO_TRACE_N 24

struct servo_result {
	int32_t  start;        /* unwrapped counts */
	int32_t  end;
	/* Furthest past the target it ever got, in the direction of travel.
	 * The end position alone cannot show an overshoot that was corrected,
	 * and correcting overshoot is the thing this controller is for. */
	int32_t  peak_overshoot;
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
