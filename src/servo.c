#include "board.h"
#include "delay.h"
#include "abs_encoder.h"
#include "motor.h"
#include "servo.h"

/* How far past the target counts as a runaway.  Generous enough that normal
 * overshoot at the lowest useful duty does not trip it, tight enough that a
 * broken feedback path is caught within a fraction of the travel. */
#define RUNAWAY_MARGIN  200

/* How far it may travel in the wrong direction before we stop.  Small: any
 * genuine movement away from the target is a fault, and the only reason not to
 * abort on the first count is encoder noise. */
#define WRONG_WAY_MARGIN 40

/* Trace sample spacing.  24 samples covers 192 ms of a 400 ms timeout. */
#define SERVO_TRACE_MS 8

void servo_move_rel(int32_t delta, const struct servo_cfg *cfg,
                    struct servo_result *out)
{
	int32_t  start  = abs_encoder_track();
	int32_t  target = start + delta;
	int16_t  sign   = (delta >= 0) ? 1 : -1;
	int      approaching = 0;
	uint16_t duty   = cfg->duty_start;
	uint32_t t0     = millis();
	uint32_t t_last_move = t0;
	uint32_t t_ramp = t0;
	int32_t  last   = start;   /* last reading that cleared the noise band */
	int32_t  ramp_ref = start;
	uint16_t duty_first = 0;

	/* Per-LEG state.  A retarget starts a new leg, and everything measured
	 * from the beginning of a move has to start again with it -- otherwise
	 * the new target inherits the old one's deadlines and its wrong-way
	 * baseline, and a reversal trips WRONG_WAY on the first iteration. */
	int32_t  leg_start = start;
	uint32_t t_leg     = t0;
	uint8_t  n_retarget = 0;

	/* The reversal brake window, and the settling that follows it.
	 *
	 * A reversal does not reverse the mechanism: it still carries momentum
	 * the old way and travels further in the OLD direction while the brake
	 * bites.  From the new leg's point of view that is movement away from
	 * the target, which is exactly what the wrong-way guard exists to catch
	 * -- so until the mechanism has actually turned around, the guard has
	 * to be told to wait, and the baseline it will use is the turnaround
	 * point, not the point where the command arrived. */
	uint32_t t_hold = 0;
	int      holding = 0;
	int      rebase  = 0;

	uint32_t t_trace = t0;

	out->retargets  = 0;
	out->target     = target;
	out->start      = start;
	out->duty_peak  = duty;
	out->duty_first = 0;
	out->outcome    = SERVO_TIMEOUT;
	out->trace_n    = 0;
	out->trace_ms   = SERVO_TRACE_MS;

	if (!cfg->dry) {
		motor_attach_pins();
		motor_drive((int16_t)(sign * (int16_t)duty));
	}

	for (;;) {
		int32_t  pos;
		uint32_t now;

		/* Read FIRST, decide on that reading, pump last.
		 *
		 * The stopping decision then acts on the freshest sample
		 * available rather than one taken before the pump -- which on a
		 * mounted camera can transmit a pair of status frames, so the
		 * reading the old order tested was already stale by the time it
		 * was tested. */
		pos = abs_encoder_track();
		now = millis();

		if (out->trace_n < SERVO_TRACE_N
		    && (out->trace_n == 0
		        || (uint32_t)(now - t_trace) >= SERVO_TRACE_MS)) {
			t_trace = now;
			out->trace[out->trace_n++] = abs_encoder_raw();
		}

		if (pos > last + cfg->noise || pos < last - cfg->noise) {
			if (!duty_first) {
				duty_first = duty;
			}
			last        = pos;
			t_last_move = now;
		}

		/* --- the body may move the goalposts ----------------------
		 *
		 * BEFORE the break conditions, deliberately: testing arrival
		 * first would let the servo declare success against a target
		 * the body has already replaced, and return so the main loop
		 * could start the replacement as a fresh move -- which is the
		 * deaf-for-a-whole-move behaviour this exists to remove, just
		 * one iteration smaller. */
		if (cfg->retarget && !holding && n_retarget < cfg->retarget_max) {
			int32_t nt = target;

			if (cfg->retarget(&nt)) {
				int32_t change = nt - target;

				if (change < 0) {
					change = -change;
				}
				if (change > cfg->retarget_min) {
					int16_t ns = (nt >= pos) ? 1 : -1;

					target = nt;
					n_retarget++;
					out->retargets = n_retarget;
					out->target    = target;

					leg_start   = pos;
					t_leg       = now;
					t_ramp      = now;
					approaching = 0;
					duty        = cfg->duty_start;

					/* These two are REDUNDANT and stay
					 * anyway: the loop refreshes t_last_move
					 * on any movement and ramp_ref every
					 * ramp_ms, so neither can be more than
					 * one interval stale.  They survive
					 * mutation testing for that reason --
					 * recorded so the next reader does not
					 * take an untested line for an untested
					 * behaviour.  Restarting a leg means
					 * restarting all of it; leaving holes in
					 * that on the grounds that something
					 * else happens to paper over them is
					 * how the wrong-way baseline bug got in
					 * two hours ago. */
					t_last_move = now;
					ramp_ref    = pos;

					if (ns != sign) {
						/* Through the brake, not
						 * straight into reverse. */
						sign    = ns;
						holding = 1;
						rebase  = 1;
						t_hold  = now;
						if (!cfg->dry) {
							motor_drive(0);
							motor_brake();
						}
					} else if (!cfg->dry) {
						motor_drive((int16_t)(sign * (int16_t)duty));
					}
				}
			}
		}

		if (holding) {
			/* A deliberate stop, not a stall: hold the stall
			 * detector off for the length of the window, or a
			 * reversal reports STALL every time. */
			t_last_move = now;
			if ((uint32_t)(now - t_hold) >= cfg->reverse_brake_ms) {
				holding = 0;
				if (!cfg->dry) {
					motor_drive((int16_t)(sign * (int16_t)duty));
				}
			}
		}

		/* Waiting for the mechanism to finish turning around.  The
		 * baseline follows it while it coasts, so when it does turn the
		 * guard is armed from the furthest point reached -- and the
		 * coast itself is never read as travelling the wrong way.
		 *
		 * Bounded by stall_ms: past that it is not coasting, it is
		 * stuck or the feedback is inverted, and the ordinary guards
		 * should get their chance to say so. */
		if (rebase) {
			int32_t moved = (sign > 0) ? pos - leg_start
			                           : leg_start - pos;

			if (moved > cfg->noise
			    || (uint32_t)(now - t_hold) >= cfg->stall_ms) {
				rebase = 0;
			} else {
				leg_start = pos;
			}
		}

		/* --- the body may call the whole thing off ----------------
		 *
		 * Before the arrival test, for the same reason the retarget is:
		 * declaring success on the iteration a stop arrives would have
		 * the mechanism finish a move the body has just cancelled. */
		if (cfg->abort && cfg->abort()) {
			out->outcome = SERVO_ABORTED;
			break;
		}

		/* arrived */
		if ((sign > 0 && pos >= target - cfg->tolerance)
		    || (sign < 0 && pos <= target + cfg->tolerance)) {
			out->outcome = SERVO_OK;
			break;
		}

		/* gone far past it -- stop pushing, whatever the reason */
		if ((sign > 0 && pos > target + RUNAWAY_MARGIN)
		    || (sign < 0 && pos < target - RUNAWAY_MARGIN)) {
			out->outcome = SERVO_RUNAWAY;
			break;
		}

		/* Moving away from where it was asked to go.  Nothing else
		 * catches this: it is moving, so no stall; it is nowhere near
		 * the target, so no arrival; and it never overshoots, so no
		 * runaway.  Inverted feedback would drive the mechanism into a
		 * stop at a ramping duty for the entire timeout. */
		if (!rebase
		    && ((sign > 0 && pos < leg_start - WRONG_WAY_MARGIN)
		        || (sign < 0 && pos > leg_start + WRONG_WAY_MARGIN))) {
			out->outcome = SERVO_WRONG_WAY;
			break;
		}

		/* Per leg.  Each retarget restarts it -- a redirected move has
		 * not had its time yet. */
		if ((uint32_t)(now - t_leg) >= cfg->timeout_ms) {
			out->outcome = SERVO_TIMEOUT;
			break;
		}

		/* And across every leg, because the per-leg one alone bounds
		 * nothing once something else can reset it. */
		if (cfg->total_ms && (uint32_t)(now - t0) >= cfg->total_ms) {
			out->outcome = SERVO_TIMEOUT;
			break;
		}

		/* Dead still for stall_ms: against a stop, or nothing we are
		 * willing to apply will move it. */
		if ((uint32_t)(now - t_last_move) >= cfg->stall_ms) {
			out->outcome = SERVO_STALL;
			break;
		}

		/* The approach segment: one step down to a lower flat duty
		 * within approach_counts of the target, so the mechanism is
		 * moving slowly when it gets there.  Braking from full speed
		 * carries it 470-850 counts past (NOTES.md §60).
		 *
		 * ONE rewrite of the compare registers, at one crossing -- not a
		 * ramp.  §31's ramp made the body power-cycle the adapter; its
		 * two-flat-segment sibling ran clean. */
		if (cfg->approach_counts && !approaching && !holding) {
			int32_t left = target - pos;

			if (left < 0) {
				left = -left;
			}
			if (left <= cfg->approach_counts) {
				approaching = 1;
				duty = cfg->duty_approach;
				if (!cfg->dry) {
					motor_drive((int16_t)(sign * (int16_t)duty));
				}
				/* A duty CUT is not a stimulus, but the mechanism
				 * is about to slow down and the stall detector
				 * must not read deceleration as a stall. */
				t_last_move = now;
			}
		}

		/* Ramp on PROGRESS, not on movement.  An earlier version raised
		 * the duty only while the position was still exactly at the
		 * start, so a mechanism that twitched one count and then crawled
		 * kept its barely-above-stiction duty and timed out short of the
		 * target -- caught by the host test, which is the whole reason
		 * this is simulated before it drives anything real. */
		if (!holding && (uint32_t)(now - t_ramp) >= cfg->ramp_ms) {
			int32_t progress = pos - ramp_ref;

			if (progress < 0) {
				progress = -progress;
			}
			t_ramp   = now;
			ramp_ref = pos;

			if (!approaching && progress < cfg->min_progress
			    && duty < cfg->duty_max) {
				duty += cfg->duty_step;
				if (duty > cfg->duty_max) {
					duty = cfg->duty_max;
				}
				if (duty > out->duty_peak) {
					out->duty_peak = duty;
				}
				if (!cfg->dry) {
					motor_drive((int16_t)(sign * (int16_t)duty));
				}

				/* A duty increase is a new stimulus and earns a
				 * fresh stall window; otherwise the stall timer
				 * expires while the ramp is still climbing and
				 * the ramp can never reach its cap. */
				t_last_move = now;
			}
		}

		/* Keep whatever else the system owes the world serviced.  On a
		 * mounted camera this is the status loop, and missing it is how
		 * the body decides the lens has stopped answering.  It gets the
		 * position this iteration already read. */
		if (cfg->pump) {
			cfg->pump(pos);
		}
	}

	if (!cfg->dry) {
		/* Brake, do not release.  Releasing floats the bridge inputs and
		 * the mechanism coasts on: measured overshoot of up to 1117
		 * counts after a 3884-count move, always in the direction of
		 * travel, which is why focus could never settle (NOTES.md §57).
		 *
		 * The stock does not release here either -- its release routine
		 * has four callers, all mode/state transitions including the
		 * shutdown sequence, and none of them is the servo. */
		motor_drive(0);
		motor_brake();
	}

	out->end        = abs_encoder_track();

	/* One more report, now that it has stopped.  The brake goes on first --
	 * it is the time-critical step, and a transmit before it would add the
	 * distance travelled during the transmit to the overshoot. */
	if (cfg->pump) {
		cfg->pump(out->end);
	}
	out->duty_first = duty_first;
	out->ms         = (uint16_t)(millis() - t0);
}
