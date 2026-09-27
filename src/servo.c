#include "board.h"
#include "delay.h"
#include "abs_encoder.h"
#include "motor.h"
#include "servo.h"

/* How far past the target counts as a runaway.
 *
 * RAISED FROM 200.  The old controller latched a direction for the whole move
 * and could not correct an overshoot at all, so anything past the target was
 * unrecoverable and 200 was the right place to give up.  This one's profile
 * reverses by itself the moment the error changes sign -- correcting overshoot
 * is ordinary operation now -- so 200 would abort a move that was about to
 * recover.  600 is a tenth of the travel: still far too much to be anything
 * but a broken feedback path. */
#define RUNAWAY_MARGIN  600

/* How far it may travel in the wrong direction before we stop.
 *
 * RAISED FROM 40, which produced a false positive on a camera.  The body
 * issues its next target 20-90 ms after the previous move finishes, and the
 * mechanism is still relaxing backwards from that move when the new one
 * starts.  That spring-back measures 20-28 counts on a scripted run and more
 * than 40 in service, where a move commanded +122 counts was aborted after
 * reading -40 within ten milliseconds.
 *
 * The guard is for INVERTED FEEDBACK, which produces sustained motion away
 * from the target under drive.  Relaxation is a decaying settle.  A larger
 * margin separates them crudely but reliably, and the cost is bounded: 120
 * counts is 2% of the travel, and an inverted encoder is still caught inside
 * a tenth of a second. */
#define WRONG_WAY_MARGIN 120

/* Trace sample spacing.  24 samples covers 192 ms of a 400 ms timeout. */
#define SERVO_TRACE_MS 8

/* THE CONTROL PERIOD.
 *
 * 5 ms, against the stock's 11.  Two measurements set it:
 *
 *   - one iteration of this loop costs 1.06 ms on the camera, consistently
 *     (best 1.034, worst 1.066 across two measured runs), so 5 ms is
 *     four or five iterations and the pump still runs at full rate
 *     underneath.
 *   - velocity comes from differencing position over the period, and the
 *     encoder noise that survives that is 0.16 counts/ms at 5 ms against
 *     0.53 at 1 ms.  The profile commands 1.2 counts/ms at 50 counts of
 *     error, so 5 ms leaves a signal-to-noise of about seven right down to
 *     the target; 1 ms would leave two.
 *
 * Faster than the stock mainly matters at breakaway: the instant the
 * mechanism lets go it is travelling 25 counts/ms, and the loop cannot cut
 * the drive until its next step.  At 11 ms that is 275 counts of lurch; at
 * 5 ms it is 125. */
#define SERVO_STEP_MS  5

/* Clamp helper: keeps the arithmetic below readable. */
static int32_t clamp32(int32_t v, int32_t lo, int32_t hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

/* IS IT THERE, AND SLOW ENOUGH TO STAY THERE?
 *
 * One function because the answer is needed in two places -- the main loop
 * tests it to decide whether to finish, and the control step counts how many
 * steps in a row it has held -- and having the condition written twice is a
 * trap: the two would only have to drift apart by a count for the counter to
 * stop reaching its threshold, and the move would then quietly run to its
 * timeout with nothing to say why. */
static int servo_settled(const struct servo_cfg *cfg, int32_t err,
                         int32_t v_meas)
{
	int32_t spd = v_meas < 0 ? -v_meas : v_meas;

	return err <= cfg->tolerance && err >= -cfg->tolerance
	       && (!cfg->v_arrive || spd <= cfg->v_arrive);
}

void servo_move_rel(int32_t delta, const struct servo_cfg *cfg,
                    struct servo_result *out)
{
	int32_t  start  = abs_encoder_track();
	int32_t  target = start + delta;
	/* Where the controller actually steers, which is a little beyond the
	 * target so that stopping at the near edge of the arrival band leaves
	 * it ON the target.  See cfg->aim_ahead. */
	int32_t  aim;
	uint32_t t0     = millis();
	uint32_t t_last_move = t0;
	int32_t  last   = start;   /* last reading that cleared the noise band */

	/* Per-LEG state.  A retarget starts a new leg, and everything measured
	 * from the beginning of a move restarts with it. */
	int32_t  leg_start = start;
	/* The direction the leg was ASKED for.  Used only by the wrong-way
	 * guard -- the control law itself has no per-leg direction any more,
	 * which is what lets it correct an overshoot instead of giving up on
	 * it. */
	int16_t  leg_sign  = (delta >= 0) ? 1 : -1;
	uint32_t t_leg     = t0;
	uint8_t  n_retarget = 0;

	/* THE CONTROLLER'S OWN STATE. */
	uint32_t t_ctl   = t0;
	int32_t  pos_ctl = start;
	int32_t  v_meas  = 0;
	int32_t  v_prev  = 0;      /* last step's commanded speed */
	int32_t  duty    = 0;
	int32_t  boost   = 0;      /* the climbing floor; see servo.h */
	uint8_t  n_still = 0;      /* consecutive steps meeting the arrival test */

	/* The reversal brake window.  Only a genuine slam goes through it now;
	 * see cfg->reverse_brake_above. */
	uint32_t t_hold = 0;
	int      holding = 0;
	/* After a direction-changing retarget the mechanism still carries
	 * momentum the OLD way and travels further that way before it turns.
	 * From the new leg's point of view that is movement away from the
	 * target -- exactly what the wrong-way guard exists to catch -- so the
	 * guard has to wait, and the baseline it will finally use is the
	 * turnaround point rather than where the command arrived. */
	int      rebase  = 0;
	uint32_t t_rebase = 0;

	uint32_t t_trace = t0;
	uint16_t duty_first = 0;

	out->retargets      = 0;
	out->target         = target;
	out->start          = start;
	out->end            = start;
	out->peak_overshoot = 0;
	out->duty_peak      = 0;
	out->duty_first     = 0;
	out->outcome        = SERVO_TIMEOUT;
	out->trace_n        = 0;
	out->trace_ms       = SERVO_TRACE_MS;

	aim = target + (int32_t)leg_sign * cfg->aim_ahead;

	if (!cfg->dry) {
		motor_attach_pins();
	}

	for (;;) {
		int32_t  pos;
		uint32_t now;

		/* Read FIRST, decide on that reading, pump last. */
		pos = abs_encoder_track();
		now = millis();

		if (out->trace_n < SERVO_TRACE_N
		    && (out->trace_n == 0
		        || (uint32_t)(now - t_trace) >= SERVO_TRACE_MS)) {
			t_trace = now;
			out->trace[out->trace_n++] = abs_encoder_raw();
		}

		if (pos > last + cfg->noise || pos < last - cfg->noise) {
			if (!duty_first && duty) {
				duty_first = (uint16_t)(duty < 0 ? -duty : duty);
			}
			last        = pos;
			t_last_move = now;
		}

		/* How far past the target it has been, in the direction it was
		 * asked to travel.  Recorded every iteration because the whole
		 * point of the new law is that an overshoot gets corrected --
		 * so the final position cannot show one happened. */
		{
			int32_t past = (leg_sign > 0) ? pos - target : target - pos;

			if (past > out->peak_overshoot) {
				out->peak_overshoot = past;
			}
		}

		/* --- the body may move the goalposts ---------------------- */
		if (cfg->retarget && !holding && n_retarget < cfg->retarget_max) {
			int32_t nt = target;

			if (cfg->retarget(&nt)) {
				int32_t change = nt - target;

				if (change < 0) {
					change = -change;
				}
				if (change > cfg->retarget_min) {
					int16_t ns = (nt >= pos) ? 1 : -1;
					/* Captured BEFORE leg_sign is updated.
					 * Testing it afterwards compares the
					 * new sign with itself, so the reverse
					 * brake never fired at all -- and the
					 * bug is invisible in the position
					 * data, because the move still ends in
					 * the right place. */
					int     reversed = (ns != leg_sign);

					target = nt;
					n_retarget++;
					out->retargets = n_retarget;
					out->target    = target;

					if (reversed) {
						rebase   = 1;
						t_rebase = now;
					}
					leg_start   = pos;
					leg_sign    = ns;
					aim         = target
					              + (int32_t)ns * cfg->aim_ahead;
					t_leg       = now;
					t_last_move = now;

					/* The integral is NOT cleared here.
					 * A body re-issuing targets every
					 * 16 ms is exactly the case the
					 * stock's never-reset accumulator
					 * serves (servo.md a measurement run): a mechanism
					 * that will not break loose keeps
					 * accumulating force across the whole
					 * burst instead of starting from the
					 * floor each time. */

					/* Through the brake only if this is a
					 * real slam -- still moving quickly
					 * the other way.  Doing it on every
					 * sign change costs 362 counts of
					 * overrun, measured twice, and most
					 * sign changes here are small
					 * corrections at walking pace. */
					if (reversed
					    && (v_meas > cfg->reverse_brake_above
					        || v_meas < -cfg->reverse_brake_above)
					    && cfg->reverse_brake_ms) {
						holding = 1;
						t_hold  = now;
						if (!cfg->dry) {
							motor_drive(0);
							motor_brake();
						}
					}
				}
			}
		}

		if (holding) {
			/* A deliberate stop, not a stall. */
			t_last_move = now;
			if ((uint32_t)(now - t_hold) >= cfg->reverse_brake_ms) {
				holding = 0;
				t_ctl   = now;      /* no velocity across the gap */
				pos_ctl = pos;
			}
		}

		/* Waiting for the mechanism to finish turning around.  The
		 * baseline follows it while it coasts, so when it does turn
		 * the guard is armed from the furthest point reached, and the
		 * coast itself never reads as travelling the wrong way.
		 *
		 * Bounded by stall_ms: past that it is not coasting, it is
		 * stuck or the feedback is inverted, and the ordinary guards
		 * should get their chance to say so. */
		if (rebase) {
			int32_t moved = (leg_sign > 0) ? pos - leg_start
			                               : leg_start - pos;

			if (moved > cfg->noise
			    || (uint32_t)(now - t_rebase) >= cfg->stall_ms) {
				rebase = 0;
			} else {
				leg_start = pos;
			}
		}

		/* --- the body may call the whole thing off ---------------- */
		if (cfg->abort && cfg->abort()) {
			out->outcome = SERVO_ABORTED;
			break;
		}

		/* arrived -- IN POSITION AND SLOW ENOUGH TO STAY THERE.
		 *
		 * The speed test is not a refinement.  Without it the servo
		 * declares success the instant it crosses into tolerance,
		 * however fast it is going, and the brake then carries it
		 * thirty times its speed further: measured on hardware at
		 * 78-87 counts past a 200-count move.  In position is not
		 * arrived. */
		/* Arrived -- in position AND slow enough that the coast cannot
		 * carry it back out, for `arrive_steps` control steps running.
		 * The dwell is counted in CONTROL STEPS, not iterations:
		 * v_meas only changes when the control step recomputes it, so
		 * counting every pass round the loop would satisfy any dwell
		 * within a millisecond and prove nothing. */
		if (servo_settled(cfg, aim - pos, v_meas)) {
			if (n_still >= cfg->arrive_steps) {
				out->outcome = SERVO_OK;
				break;
			}
		} else {
			n_still = 0;
		}

		/* gone far past it -- the feedback is wrong, not the move */
		if ((leg_sign > 0 && pos > target + RUNAWAY_MARGIN)
		    || (leg_sign < 0 && pos < target - RUNAWAY_MARGIN)) {
			out->outcome = SERVO_RUNAWAY;
			break;
		}

		/* Moving away from where it was asked to go: inverted feedback.
		 * Measured from the leg start, so a corrected overshoot -- which
		 * is past the TARGET, not behind the START -- does not trip it. */
		if (!rebase
		    && ((leg_sign > 0 && pos < leg_start - WRONG_WAY_MARGIN)
		        || (leg_sign < 0 && pos > leg_start + WRONG_WAY_MARGIN))) {
			out->outcome = SERVO_WRONG_WAY;
			break;
		}

		if ((uint32_t)(now - t_leg) >= cfg->timeout_ms) {
			out->outcome = SERVO_TIMEOUT;
			break;
		}
		if (cfg->total_ms && (uint32_t)(now - t0) >= cfg->total_ms) {
			out->outcome = SERVO_TIMEOUT;
			break;
		}

		/* Dead still for stall_ms.  This has to outlast the integral's
		 * climb through the breakaway band -- sitting still while the
		 * duty rises is what starting looks like, not what failing
		 * looks like. */
		if ((uint32_t)(now - t_last_move) >= cfg->stall_ms) {
			int32_t err = target - pos;

			if (err < 0) {
				err = -err;
			}
			/* Stopped, but near enough.  See settle_counts: the
			 * mechanism has a resolution and running out of ways
			 * to get closer than it is not a failure. */
			out->outcome = (cfg->settle_counts
			                && err <= cfg->settle_counts)
			               ? SERVO_OK : SERVO_STALL;
			break;
		}

		/* --- THE CONTROL STEP, at a fixed rate -------------------- */
		if (!holding && (uint32_t)(now - t_ctl) >= SERVO_STEP_MS) {
			int32_t dt = (int32_t)(now - t_ctl);
			int32_t err = aim - pos;
			int32_t v_target, v_ff, ev, mag, want;

			/* counts/s, from this step's own interval rather than
			 * the nominal one -- the loop is polled, not timed, so
			 * dt is 5 or 6 ms and using 5 regardless would bias
			 * every velocity by up to a fifth. */
			v_meas = (pos - pos_ctl) * 1000 / dt;

			v_target = clamp32(err * 1000 / (int32_t)cfg->stop_ms,
			                   -cfg->v_cruise, cfg->v_cruise);
			ev = v_target - v_meas;

			/* The drive that PRODUCES this speed, not the one that
			 * would hold it: u = (v + tau * dv/dt) / k.  See
			 * servo.h -- without the second term the mechanism
			 * lags the profile by tau and arrives long, in
			 * proportion to the distance. */
			v_ff = v_target
			       + (int32_t)cfg->tau_ms * (v_target - v_prev) / dt;
			v_prev = v_target;

			/* The climbing floor.  "Keeping up" is half the
			 * commanded speed: that catches a mechanism that is
			 * stuck AND one that is merely crawling, and it
			 * cannot be satisfied by encoder noise. */
			want = v_target < 0 ? -v_target : v_target;
			if (want > 0
			    && (v_meas < 0 ? -v_meas : v_meas) * 2 < want) {
				boost += cfg->boost_up;
				if (boost > (int32_t)cfg->boost_max) {
					boost = cfg->boost_max;
				}
			} else if (boost > 0) {
				boost -= cfg->boost_down;
				if (boost < 0) {
					boost = 0;
				}
			}

			duty = v_ff / (int32_t)cfg->ff_div
			       + ev / (int32_t)cfg->kp_div;
			if (v_target > 0) {
				duty += (int32_t)cfg->ff_offset + boost;
			} else if (v_target < 0) {
				duty -= (int32_t)cfg->ff_offset + boost;
			}

			duty = clamp32(duty, -(int32_t)cfg->duty_max,
			               (int32_t)cfg->duty_max);
			mag = duty < 0 ? -duty : duty;
			if (mag < (int32_t)cfg->duty_floor) {
				/* The dead zone.  Below the floor the mechanism
				 * draws current without moving,
				 * so the output may not sit there -- but which
				 * way to leave depends on what it was trying to
				 * do.  Still asked to travel: go to the floor,
				 * or a short move would never start at all.
				 * Trying to slow down, or already there: zero,
				 * so the floor cannot push it onward. */
				int32_t speed = v_meas < 0 ? -v_meas : v_meas;

				if (((v_target > 0 && err > cfg->tolerance)
				     || (v_target < 0 && err < -cfg->tolerance))
				    && speed < want) {
					duty = (v_target > 0)
					       ? (int32_t)cfg->duty_floor
					       : -(int32_t)cfg->duty_floor;
					mag  = cfg->duty_floor;
				} else {
					duty = 0;
					mag  = 0;
				}
			}
			if (mag > (int32_t)out->duty_peak) {
				out->duty_peak = (uint16_t)mag;
			}

			if (!cfg->dry) {
				motor_drive((int16_t)duty);
			}
			if (servo_settled(cfg, aim - pos, v_meas)) {
				if (n_still < 255) {
					n_still++;
				}
			} else {
				n_still = 0;
			}

			t_ctl   = now;
			pos_ctl = pos;
		}

		if (cfg->pump) {
			cfg->pump(pos);
		}
	}

	if (!cfg->dry) {
		/* Brake, do not release.  Releasing floats the bridge and the
		 * mechanism coasts; braked, it holds below the
		 * rate at which it falls under its own weight. */
		motor_drive(0);
		motor_brake();
	}

	out->end        = abs_encoder_track();
	{
		int32_t past = (leg_sign > 0) ? out->end - target
		                              : target - out->end;

		if (past > out->peak_overshoot) {
			out->peak_overshoot = past;
		}
	}

	if (cfg->pump) {
		cfg->pump(out->end);
	}
	out->duty_first = duty_first;
	out->ms         = (uint16_t)(millis() - t0);
}
