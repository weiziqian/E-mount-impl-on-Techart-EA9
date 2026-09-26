/* test_servo.c -- prove the servo stops, in every way it can go wrong.
 *
 * This is the code that stands between a bug and a damaged helicoid, so the
 * cases that matter are the failures, not the success.
 */
#include <stdarg.h>
#include <stdio.h>
#include "board.h"
#include "servo.h"

extern double sim_pos;
extern int sim_stop_lo, sim_stop_hi, sim_stiction, sim_encoder_stuck, sim_noise;
extern double sim_encoder_gain;
extern uint32_t host_millis;
void sim_reset(void);
int32_t abs_encoder_track(void);

static int failures;
static unsigned pump_calls;
static int32_t last_pump_pos;
static void count_pump(int32_t pos) { last_pump_pos = pos; pump_calls++; }

static void fail(const char *fmt, ...)
{
	va_list ap; va_start(ap, fmt);
	fputs("FAIL: ", stdout); vprintf(fmt, ap); putchar('\n'); va_end(ap);
	failures++;
}

/* The same numbers src/main.c uses. */
static const struct servo_cfg SAFE = {
	.duty_start = 2560 * 4 / 100, .duty_max = 2560 * 16 / 100,
	.duty_step  = 2560 * 2 / 100, .ramp_ms = 20, .min_progress = 8,
	.stall_ms = 40,
	.timeout_ms = 400, .tolerance = 8, .noise = 8, .pump = count_pump,
};
static const char *const NAME[] = { "OK", "STALL", "TIMEOUT", "RUNAWAY",
				    "WRONG_WAY", "ABORTED" };
/* The names are indexed by the outcome, so a new outcome with no name here
 * reads as a null pointer in every failure message.  Pin the count. */
typedef char outcome_names_are_complete[
	(sizeof(NAME) / sizeof(NAME[0]) == SERVO_ABORTED + 1) ? 1 : -1];

/* The servo must leave the mechanism HELD, not floating.  Releasing lets it
 * coast past the target, which is what stopped focus settling on hardware; a
 * simulator without momentum reported this as fine for weeks. */
extern double sim_vel;
extern int    sim_released;
uint32_t millis(void);
#define host_tick millis

static void check_holds_after_move(void)
{
	struct servo_cfg cfg = SAFE;
	struct servo_result r;
	double before;
	int i;

	sim_reset();
	servo_move_rel(2000, &cfg, &r);

	if (sim_released) {
		fail("the servo left the bridge FLOATING -- the mechanism will coast");
	}
	/* Let the brake finish first: it is a decay, not a switch.  Measuring
	 * from the instant the move ends conflates stopping distance with
	 * failure to hold, which are different problems with different fixes. */
	for (i = 0; i < 300; i++) {
		(void)host_tick();
	}
	before = sim_pos;
	for (i = 0; i < 500; i++) {
		(void)host_tick();
	}
	if (sim_pos - before > 4 || before - sim_pos > 4) {
		fail("drifted %.0f counts in 500 ms after the move -- not held",
		     sim_pos - before);
	}
}

/* The whole point of the approach segment: arrive slowly enough that the brake
 * stops the mechanism near the target rather than hundreds of counts past. */
static void check_overshoot(void)
{
	/* Flat drive, no ramp -- the shape the firmware ships. */
	struct servo_cfg fast = {
		.duty_start = 800, .duty_max = 800, .duty_step = 0,
		.ramp_ms = 50, .min_progress = 8, .stall_ms = 60,
		.timeout_ms = 3000, .tolerance = 8, .noise = 8,
		.pump = count_pump,
	};
	struct servo_cfg slow = fast;
	struct servo_cfg full = fast;
	struct servo_result r;
	double over_fast, over_slow, over_full;

	slow.approach_counts = 500;
	slow.duty_approach   = 250;

	/* The focus duty this build now uses: 1280 = 50%, the top of the stock
	 * servo's own range.  Measured beside 800 because the whole reason the
	 * approach segment exists is that overshoot grows with cruise speed --
	 * if raising the duty undid it, this is where it would show. */
	full = slow;
	full.duty_start = 1280;
	full.duty_max   = 1280;

	sim_reset();
	servo_move_rel(4000, &fast, &r);
	for (int i = 0; i < 400; i++) (void)host_tick();
	over_fast = sim_pos - 4000;

	sim_reset();
	servo_move_rel(4000, &slow, &r);
	for (int i = 0; i < 400; i++) (void)host_tick();
	over_slow = sim_pos - 4000;

	sim_reset();
	servo_move_rel(4000, &full, &r);
	for (int i = 0; i < 400; i++) (void)host_tick();
	over_full = sim_pos - 4000;

	printf("  overshoot: 800 flat %.0f counts, 800 + approach %.0f, "
	       "1280 + approach %.0f\n", over_fast, over_slow, over_full);

	/* The approach segment must still do its job at the higher duty.  A
	 * cruise that arrives too fast to decelerate inside approach_counts
	 * would give back everything the segment was added for. */
	if (over_full > over_fast) {
		fail("at duty 1280 the approach segment no longer helps: "
		     "%.0f counts, worse than flat 800 (%.0f)",
		     over_full, over_fast);
	}

	/* The other direction.  Testing only one hid a missing absolute value
	 * on the remaining distance: with a negative move the comparison is
	 * true from the first iteration, so the WHOLE move runs at the approach
	 * duty.  It still arrives, and still arrives gently, so every overshoot
	 * assertion passes -- what gives it away is the time. */
	{
		struct servo_result rn;
		uint16_t flat_ms;

		sim_reset();
		servo_move_rel(-4000, &fast, &rn);
		flat_ms = rn.ms;

		sim_reset();
		servo_move_rel(-4000, &slow, &rn);
		for (int i = 0; i < 400; i++) (void)host_tick();
		if (sim_pos + 4000 < -150 || sim_pos + 4000 > 150) {
			fail("negative move ended %.0f counts off target",
			     sim_pos + 4000);
		}
		if (rn.ms > flat_ms * 2 + 200) {
			fail("negative move took %u ms against %u flat -- the "
			     "approach segment is engaging for the whole move",
			     rn.ms, flat_ms);
		}
	}
	if (over_slow >= over_fast) {
		fail("the approach segment did not reduce overshoot (%.0f vs %.0f)",
		     over_slow, over_fast);
	}
	if (over_slow > 150) {
		fail("still %.0f counts past the target after the approach segment",
		     over_slow);
	}
}

/* The body must be told where the mechanism ENDED UP, not just where it was on
 * the last iteration before the stop condition fired.  Without a report after
 * the loop the final position waits for the main loop to come round, and the
 * frames sent in between carry a position the lens has already left -- the same
 * staleness the in-loop pump was added to fix, just moved to the end. */
static void check_reports_final_position(void)
{
	struct servo_cfg cfg = {
		.duty_start = 800, .duty_max = 800, .duty_step = 0,
		.ramp_ms = 50, .min_progress = 8, .stall_ms = 60,
		.timeout_ms = 3000, .tolerance = 8, .noise = 8,
		.pump = count_pump,
	};
	struct servo_result r;

	sim_reset();
	last_pump_pos = 0x7FFFFFFF;
	servo_move_rel(2000, &cfg, &r);

	if (last_pump_pos != r.end) {
		fail("the last report was %ld but the move ended at %ld",
		     (long)last_pump_pos, (long)r.end);
	}
}

/* --- mid-move retargeting ------------------------------------------------
 *
 * The servo's contract changes here from "drive to this target and return" to
 * "drive toward a target that can move", so every guard measured from the start
 * of a move is now measured from the start of a LEG.  Getting that wrong is
 * silent: a reversal whose wrong-way baseline is still the original start trips
 * WRONG_WAY on its first iteration and the move ends instantly, in the right
 * general area, with a plausible-looking result. */
static int32_t rt_new_target;
static int32_t rt_fire_at;        /* fire once the mechanism passes here */
static int     rt_fired;
static int     rt_count;
static int     rt_repeat;         /* keep firing the same target forever */
static int32_t rt_alt;           /* non-zero: alternate with rt_new_target */
static int     rt_stick_encoder; /* freeze the encoder as the hook fires  */
static int     rt_invert_encoder;
static int     rt_give_up_after; /* stop firing after this many calls */
static int32_t rt_break_below;   /* invert the encoder once pos drops here */
static int     rt_coast_forever; /* motor loses authority; momentum does not */
extern int     sim_stiction;
extern int     sim_encoder_stuck;
extern double  sim_encoder_gain;
extern double  sim_brake_decay;

static uint8_t rt_hook(int32_t *target)
{
	int32_t pos = abs_encoder_track();

	/* A second, independent trigger: break the feedback LATER in the move,
	 * not at the instant the target changes.  The two are different events
	 * and the guards have to survive both orders. */
	if (rt_break_below && rt_fired && pos < rt_break_below) {
		sim_encoder_gain = -1.0;
		rt_break_below = 0;
	}
	if (rt_give_up_after && rt_count >= rt_give_up_after) {
		return 0;
	}
	if (rt_repeat) {
		rt_count++;
		if (rt_alt) {
			int32_t t = rt_new_target;

			rt_new_target = rt_alt;
			rt_alt = t;
		}
		*target = rt_new_target;
		return 1;
	}
	if (rt_fired || pos < rt_fire_at) {
		return 0;
	}
	rt_fired = 1;
	rt_count++;
	*target = rt_new_target;
	/* Break the feedback at the same instant the target moves -- the two
	 * events the guards have to keep telling apart. */
	if (rt_stick_encoder) {
		sim_encoder_stuck = 1;
	}
	if (rt_invert_encoder) {
		sim_encoder_gain = -1.0;
	}
	if (rt_coast_forever) {
		sim_stiction    = 2000;   /* no duty we apply can move it */
		sim_brake_decay = 1.0;    /* and nothing slows it down     */
	}
	return 1;
}

static struct servo_cfg rt_cfg(void)
{
	struct servo_cfg c = {
		.duty_start = 800, .duty_max = 800, .duty_step = 0,
		.ramp_ms = 50, .min_progress = 8, .stall_ms = 60,
		.timeout_ms = 3000, .tolerance = 8, .noise = 8,
		.pump = count_pump,
		.approach_counts = 500, .duty_approach = 250,
		.retarget = rt_hook, .retarget_max = 16, .retarget_min = 12,
		.reverse_brake_ms = 20, .total_ms = 5000,
	};
	return c;
}

static void rt_arm(int32_t fire_at, int32_t new_target)
{
	rt_fire_at = fire_at; rt_new_target = new_target;
	rt_fired = 0; rt_count = 0; rt_repeat = 0; rt_alt = 0;
	rt_stick_encoder = 0; rt_invert_encoder = 0; rt_give_up_after = 0;
	rt_break_below = 0; rt_coast_forever = 0;
}

/* --- the abort hook ------------------------------------------------------
 *
 * The body's 23-byte stop command.  Unlike a retarget it names no new
 * position: the mechanism must stand still where it is, and the caller must be
 * told the target was NOT reached so it acknowledges rather than reporting
 * arrival. */
static int32_t ab_fire_at;
static int     ab_count;

static uint8_t ab_hook(void)
{
	ab_count++;
	return abs_encoder_track() >= ab_fire_at;
}

static void check_abort_stops_the_move(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	sim_reset();
	rt_arm(0, 0);
	cfg.retarget = 0;
	cfg.abort = ab_hook;
	ab_fire_at = 1000;
	ab_count = 0;

	servo_move_rel(4000, &cfg, &r);

	if (!ab_count) {
		fail("abort: the hook never fired");
	}
	if (r.outcome != SERVO_ABORTED) {
		fail("abort: ended %s, expected ABORTED", NAME[r.outcome]);
	}
	/* Stopped near where the hook fired, NOT at the target.  The margin is
	 * the braking distance -- the point is that it is nowhere near 4000. */
	if (r.end > 2500) {
		fail("abort: ran on to %ld; it was told to stop at 1000",
		     (long)r.end);
	}
	if (r.end < 900) {
		fail("abort: stopped at %ld, before the hook could fire",
		     (long)r.end);
	}
}

/* A hook that never fires must change nothing.  Without this, an abort that
 * was simply always-on would pass the test above. */
static void check_abort_silent_is_a_normal_move(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	sim_reset();
	rt_arm(0, 0);
	cfg.retarget = 0;
	cfg.abort = ab_hook;
	ab_fire_at = 0x7FFFFFFF;        /* never */
	ab_count = 0;

	servo_move_rel(2000, &cfg, &r);

	if (!ab_count) {
		fail("silent abort: the hook was never even called");
	}
	if (r.outcome != SERVO_OK) {
		fail("silent abort: ended %s, expected OK", NAME[r.outcome]);
	}
	if (r.end < 1800 || r.end > 2200) {
		fail("silent abort: ended at %ld, expected near 2000",
		     (long)r.end);
	}
}

/* And a NULL hook must be as good as a silent one -- homing and the park pass
 * NULL, and a null-deref there runs the mechanism into an end stop. */
static void check_no_abort_hook(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	sim_reset();
	rt_arm(0, 0);
	cfg.retarget = 0;
	cfg.abort = 0;

	servo_move_rel(2000, &cfg, &r);
	if (r.outcome != SERVO_OK) {
		fail("no abort hook: ended %s, expected OK", NAME[r.outcome]);
	}
}

static void check_retarget_forward(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	sim_reset();
	rt_arm(1000, 3000);
	servo_move_rel(2000, &cfg, &r);

	if (!rt_count) {
		fail("forward retarget: the hook never fired");
	}
	if (r.retargets != 1) {
		fail("forward retarget: reported %u redirections, expected 1",
		     r.retargets);
	}
	if (r.target != 3000) {
		fail("forward retarget: pursued %ld, expected 3000",
		     (long)r.target);
	}
	if (r.outcome != SERVO_OK) {
		fail("forward retarget ended %s, not OK", NAME[r.outcome]);
	}
	if (r.end < 3000 - 200 || r.end > 3000 + 200) {
		fail("forward retarget ended at %ld, not near 3000",
		     (long)r.end);
	}
}

/* The one that fails loudly if the wrong-way baseline is not reset: the new
 * target is BEHIND the mechanism, so from the original start's point of view
 * the move is running backwards. */
static void check_retarget_reversal(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	sim_reset();
	rt_arm(1500, 300);
	servo_move_rel(3000, &cfg, &r);

	if (!rt_count) {
		fail("reversal: the hook never fired");
	}
	if (r.outcome != SERVO_OK) {
		fail("reversal ended %s, not OK -- a reversed leg must not "
		     "read as wrong-way, stalled or timed out", NAME[r.outcome]);
	}
	if (r.end < 300 - 250 || r.end > 300 + 250) {
		fail("reversal ended at %ld, not near 300", (long)r.end);
	}
}

/* A body repeating the same target every frame must not count as redirecting,
 * or the leg deadlines reset forever and nothing ever times out. */
static void check_retarget_deadband(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	sim_reset();
	rt_arm(0, 2005);           /* 5 counts from the commanded 2000 */
	rt_repeat = 1;
	servo_move_rel(2000, &cfg, &r);
	rt_repeat = 0;

	if (r.retargets != 0) {
		fail("a target %d counts away was treated as a redirection",
		     5);
	}
	if (r.outcome != SERVO_OK) {
		fail("deadband case ended %s, not OK", NAME[r.outcome]);
	}
}

/* And a body that really does keep moving the target must still be bounded --
 * this is the coils staying energised, not an abstract loop. */
static void check_retarget_cap(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	sim_reset();
	rt_arm(0, 0);
	/* Alternate far ahead / far behind, every iteration, so it can never
	 * arrive at anything. */
	rt_repeat     = 1;
	rt_new_target = 6000;
	rt_alt        = -6000;
	servo_move_rel(4000, &cfg, &r);
	rt_repeat = 0;
	rt_alt    = 0;

	if (r.retargets > cfg.retarget_max) {
		fail("retargeted %u times against a cap of %u",
		     r.retargets, cfg.retarget_max);
	}
	if (r.ms > cfg.total_ms + 200) {
		fail("ran %u ms against a %u ms ceiling", r.ms, cfg.total_ms);
	}
}

/* The wrong-way guard must be re-baselined at the retarget, not left at the
 * start of the whole move.  Same-direction retarget, then the encoder stops
 * reporting position at all: from the NEW baseline that is a 1000-count jump
 * backwards and the guard fires; from the ORIGINAL one it is a few counts and
 * the move limps on to a stall instead. */
static void check_retarget_rebaselines_wrong_way(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	sim_reset();
	rt_arm(1000, 3000);
	rt_stick_encoder = 1;
	servo_move_rel(4000, &cfg, &r);
	rt_stick_encoder = 0;

	if (r.outcome != SERVO_WRONG_WAY) {
		fail("encoder lost after a retarget ended %s -- the wrong-way "
		     "baseline is still the start of the whole move",
		     NAME[r.outcome]);
	}
}

/* And after a REVERSAL the guard must come back. While the mechanism is still
 * coasting the old way the guard has to wait, but if it waits forever an
 * inverted or dead encoder is never caught again for the rest of the move. */
static void check_reversal_rearms_wrong_way(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	sim_reset();
	/* Reverse at 1500 and aim well past the origin, so no later reading can
	 * be mistaken for arrival or for a runaway -- the only guard left that
	 * can end this move is the one under test. */
	rt_arm(1500, -3000);
	/* Break the feedback only AFTER the turnaround is complete, so the
	 * coast-tolerance window has legitimately opened and closed first. */
	rt_break_below = 1000;
	servo_move_rel(3000, &cfg, &r);
	rt_break_below = 0;

	if (r.outcome != SERVO_WRONG_WAY) {
		fail("encoder inverted after a reversal ended %s -- the "
		     "wrong-way guard never re-armed after the turnaround",
		     NAME[r.outcome]);
	}
}

/* A retarget gives the move a fresh deadline.  Without that, a redirection
 * late in a long move inherits the elapsed time of the leg before it and times
 * out with the new target still ahead of it. */
static void check_retarget_resets_the_deadline(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	/* No approach segment here: it makes the last 500 counts of a leg take
	 * longer than the rest of the move, which would put the two legs on
	 * wildly different clocks and make any single deadline meaningless as a
	 * test of whether the deadline was reset. */
	cfg.approach_counts = 0;
	cfg.timeout_ms = 120;      /* shorter than the whole two-leg journey */
	sim_reset();
	rt_arm(1500, 3000);
	servo_move_rel(2000, &cfg, &r);

	if (!rt_count) {
		fail("deadline test: the hook never fired");
	}
	if (r.outcome != SERVO_OK) {
		fail("a redirected move ended %s -- the new leg inherited the "
		     "old leg's deadline", NAME[r.outcome]);
	}
}

/* The absolute ceiling, on its own.  With the per-move cap on redirections
 * lifted, nothing else bounds a body that keeps moving the target: each
 * redirection resets the per-leg deadline, and a mechanism that keeps changing
 * direction never stalls.  This is the coils staying energised. */
static void check_total_ceiling(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	cfg.retarget_max = 255;
	cfg.total_ms     = 500;

	sim_reset();
	rt_arm(0, 0);
	rt_repeat     = 1;
	rt_new_target = 6000;
	rt_alt        = -6000;
	rt_give_up_after = 4000;   /* so a missing ceiling ends the test rather
	                              than hanging the suite */
	servo_move_rel(4000, &cfg, &r);
	rt_repeat = 0; rt_alt = 0; rt_give_up_after = 0;

	if (r.ms > cfg.total_ms + 200) {
		fail("ran %u ms against a %u ms absolute ceiling",
		     r.ms, cfg.total_ms);
	}
}

/* The reverse brake must not be read as a stall.  reverse_brake_ms and
 * stall_ms are independent numbers, and the guard exists so they do not have to
 * be kept in a particular order by hand -- which is exactly the kind of
 * coupling that survives review and fails in the field. */
static void check_reverse_brake_is_not_a_stall(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	cfg.reverse_brake_ms = 100;    /* deliberately longer than stall_ms */
	cfg.stall_ms         = 60;

	sim_reset();
	/* A mechanism that the brake actually STOPS inside the window.  With
	 * the shipping decay it is still coasting when the window ends, so the
	 * ordinary movement tracking keeps the stall timer fresh by accident
	 * and the guard is never exercised -- the bug would sit here waiting
	 * for a stiffer helicoid. */
	sim_brake_decay = 0.5;
	rt_arm(1500, 300);
	servo_move_rel(3000, &cfg, &r);
	sim_brake_decay = 0.955;

	if (r.outcome == SERVO_STALL) {
		fail("a %u ms reverse brake against a %u ms stall window was "
		     "reported as a stall", cfg.reverse_brake_ms, cfg.stall_ms);
	}
}

/* Under a RAMPING configuration, a redirected move gets a fresh ramp: the duty
 * goes back to duty_start, so the progress reference it is measured against has
 * to go back too.  Left stale, the first comparison sees the whole previous leg
 * as "progress", declines to raise the duty, and the mechanism sits below
 * stiction until the stall detector gives up. */
static void check_retarget_restarts_the_ramp(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	cfg.duty_start      = 100;     /* below the simulated stiction of 150 */
	cfg.duty_max        = 800;
	cfg.duty_step       = 100;
	cfg.ramp_ms         = 20;
	cfg.approach_counts = 0;

	sim_reset();
	rt_arm(1500, 3000);
	servo_move_rel(2000, &cfg, &r);

	if (!rt_count) {
		fail("ramp restart: the hook never fired");
	}
	if (r.outcome != SERVO_OK) {
		fail("a redirected move under a ramp ended %s -- the ramp's "
		     "progress reference was not restarted", NAME[r.outcome]);
	}
}

/* Redirecting out of the approach segment must put the approach segment back
 * in play for the new target.  Otherwise the mechanism runs the whole new leg
 * flat out and arrives at full speed -- the overshoot the segment exists to
 * remove, reappearing only when the body redirects. */
static void check_retarget_reopens_the_approach(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	sim_reset();
	/* Fire inside the approach zone of the original 2000-count move. */
	rt_arm(1700, 4000);
	servo_move_rel(2000, &cfg, &r);
	for (int i = 0; i < 400; i++) {
		(void)host_tick();
	}

	if (!rt_count) {
		fail("approach reopen: the hook never fired");
	}
	if (sim_pos - 4000 > 200) {
		fail("overshot the redirected target by %.0f counts -- the "
		     "approach segment did not re-engage", sim_pos - 4000);
	}
	/* And the duty has to go back up, not stay at the approach duty for the
	 * whole new leg.  That failure arrives at the right place, gently, and
	 * every overshoot assertion passes -- what gives it away is the clock. */
	if (r.ms > 400) {
		fail("the redirected move took %u ms -- it ran the new leg at "
		     "the approach duty", r.ms);
	}
}

/* The coast-tolerance window after a reversal must be bounded in TIME, not
 * only by the mechanism turning around.  Here it never turns around: the motor
 * has no authority and the mechanism carries on the old way. Without the time
 * bound the wrong-way guard is suppressed for the rest of the move and the only
 * thing left to end it is a timeout -- which reports "ran out of time" for what
 * is really a feedback or drive fault. */
static void check_reversal_coast_window_is_bounded(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	sim_reset();
	rt_arm(1500, -3000);
	rt_coast_forever = 1;
	servo_move_rel(3000, &cfg, &r);
	rt_coast_forever = 0;
	sim_stiction = 150; sim_brake_decay = 0.955;

	if (r.outcome != SERVO_WRONG_WAY) {
		fail("a mechanism coasting the wrong way after a reversal "
		     "ended %s, not WRONG_WAY", NAME[r.outcome]);
	}
}

/* A NULL hook must leave the old behaviour exactly as it was. */
static void check_no_retarget_unchanged(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	cfg.retarget = 0;
	sim_reset();
	rt_arm(1000, 3000);
	servo_move_rel(2000, &cfg, &r);

	/* The target is start + delta and `start` carries encoder noise, so it
	 * is 2000 give or take a few counts -- not exactly 2000. */
	if (r.retargets != 0 || r.target < 1980 || r.target > 2020) {
		fail("a NULL retarget hook still redirected the move "
		     "(%u redirections, target %ld)",
		     r.retargets, (long)r.target);
	}
	if (r.outcome != SERVO_OK || r.end < 1800 || r.end > 2200) {
		fail("a NULL retarget hook changed the plain move");
	}
}

int main(void)
{
	check_abort_stops_the_move();
	check_abort_silent_is_a_normal_move();
	check_no_abort_hook();
	check_retarget_forward();
	check_retarget_reversal();
	check_retarget_deadband();
	check_retarget_cap();
	check_retarget_rebaselines_wrong_way();
	check_reversal_rearms_wrong_way();
	check_retarget_resets_the_deadline();
	check_total_ceiling();
	check_reverse_brake_is_not_a_stall();
	check_retarget_restarts_the_ramp();
	check_retarget_reopens_the_approach();
	check_reversal_coast_window_is_bounded();
	check_no_retarget_unchanged();
	check_holds_after_move();
	check_reports_final_position();
	check_overshoot();
	struct servo_result r;

	/* 1. a normal move arrives, and stops near the target */
	sim_reset();
	servo_move_rel(200, &SAFE, &r);
	if (r.outcome != SERVO_OK) {
		fail("free move: outcome %s, expected OK", NAME[r.outcome]);
	}
	/* +-3 counts of jitter on both the arrival test and this readback. */
	if (r.end < 185 || r.end > 265) {
		fail("free move: ended at %d, expected near 200", r.end);
	}
	if (!r.duty_first) {
		fail("free move: never recorded the duty it started moving at");
	}

	/* 2. reverse works and is symmetric */
	sim_reset();
	servo_move_rel(-200, &SAFE, &r);
	if (r.outcome != SERVO_OK || r.end > -185 || r.end < -265) {
		fail("reverse: %s, ended %d", NAME[r.outcome], r.end);
	}

	/* 3. stiction above duty_start: it must RAMP, not give up */
	sim_reset();
	sim_stiction = 2560 * 9 / 100;      /* needs ~9%, ramp starts at 4% */
	servo_move_rel(200, &SAFE, &r);
	if (r.outcome != SERVO_OK) {
		fail("stiction: outcome %s, expected the ramp to break it free",
		     NAME[r.outcome]);
	}
	if (r.duty_first <= SAFE.duty_start) {
		fail("stiction: duty_first %u, expected above the start %u",
		     r.duty_first, SAFE.duty_start);
	}

	/* 4. AN END STOP.  The case that damaged nothing only by luck last time.
	 * It must stall, release, and not have pushed far past the stop. */
	sim_reset();
	sim_stop_hi = 50;
	servo_move_rel(200, &SAFE, &r);
	if (r.outcome != SERVO_STALL) {
		fail("end stop: outcome %s, expected STALL", NAME[r.outcome]);
	}
	if (r.end > 55) {
		fail("end stop: ended at %d, past the stop at 50", r.end);
	}
	if (r.ms > 250) {
		fail("end stop: took %u ms to give up", r.ms);
	}

	/* 4b. THE SAME END STOP, WITH A NOISY ENCODER.  The hardware run showed
	 * every move timing out instead of stalling, because the stall timer
	 * reset on any change of reading and a stationary encoder jitters.  A
	 * noiseless simulation cannot catch that, so the noise is the test. */
	sim_reset();
	sim_stop_hi = 50;
	sim_noise = 3;
	servo_move_rel(200, &SAFE, &r);
	if (r.outcome != SERVO_STALL) {
		fail("noisy end stop: outcome %s, expected STALL -- jitter must "
		     "not look like movement", NAME[r.outcome]);
	}
	if (r.ms > 250) {
		fail("noisy end stop: pushed for %u ms before giving up", r.ms);
	}

	/* 5. stiction so high nothing can move it: still stalls, never hangs */
	sim_reset();
	sim_stiction = 2560;
	servo_move_rel(200, &SAFE, &r);
	if (r.outcome != SERVO_STALL) {
		fail("immovable: outcome %s, expected STALL", NAME[r.outcome]);
	}
	if (r.duty_peak > SAFE.duty_max) {
		fail("immovable: ramped to %u, above the cap %u",
		     r.duty_peak, SAFE.duty_max);
	}

	/* 6. dead encoder -- the servo must not drive blind forever */
	sim_reset();
	sim_encoder_stuck = 1;
	servo_move_rel(200, &SAFE, &r);
	if (r.outcome != SERVO_STALL && r.outcome != SERVO_TIMEOUT) {
		fail("dead encoder: outcome %s, expected it to give up",
		     NAME[r.outcome]);
	}
	if (host_millis > 500) {
		fail("dead encoder: ran %u ms before giving up", host_millis);
	}

	/* 7. feedback wildly wrong -- runaway guard */
	sim_reset();
	sim_encoder_gain = 40.0;
	servo_move_rel(200, &SAFE, &r);
	if (r.outcome != SERVO_OK && r.outcome != SERVO_RUNAWAY) {
		fail("runaway: outcome %s", NAME[r.outcome]);
	}
	if (sim_pos > 400) {
		fail("runaway: mechanism travelled %.0f counts before stopping",
		     sim_pos);
	}

	/* 8. INVERTED FEEDBACK.  Every other guard is satisfied and the
	 * mechanism gets driven into a stop for the whole timeout.  This case
	 * was added because mutating the runaway guard changed nothing -- which
	 * meant the fault it was supposed to cover was not being tested at all. */
	sim_reset();
	sim_encoder_gain = -1.0;
	servo_move_rel(200, &SAFE, &r);
	if (r.outcome != SERVO_WRONG_WAY) {
		fail("inverted feedback: outcome %s, expected WRONG_WAY",
		     NAME[r.outcome]);
	}
	if (sim_pos < -60 || sim_pos > 60) {
		fail("inverted feedback: travelled %.0f counts before stopping",
		     sim_pos);
	}
	if (host_millis > 200) {
		fail("inverted feedback: ran %u ms before stopping", host_millis);
	}

	/* 9. the whole out-and-back sequence is net zero */
	sim_reset();
	{
		static const int32_t SEQ[] = { +200, -200, -200, +200 };
		unsigned i;
		for (i = 0; i < 4; i++) {
			servo_move_rel(SEQ[i], &SAFE, &r);
			if (r.outcome != SERVO_OK) {
				fail("sequence step %u: %s", i, NAME[r.outcome]);
			}
		}
		if (sim_pos < -80 || sim_pos > 80) {
			fail("sequence drifted to %.0f counts; it must be net zero",
			     sim_pos);
		}
		if (sim_pos > 260 || sim_pos < -260) {
			fail("sequence wandered past one step from the start");
		}
	}

	/* The pump is how the protocol keeps running during a move.  A servo
	 * that forgets to call it goes silent for the whole move, which is what
	 * made a real body re-run its init handshake mid-move (NOTES.md §27). */
	sim_reset();
	pump_calls = 0;
	servo_move_rel(200, &SAFE, &r);
	if (pump_calls < 10) {
		fail("the pump was called %u times across a %u ms move -- the "
		     "protocol would go silent", pump_calls, r.ms);
	}
	if (!r.trace_n) {
		fail("no encoder trace recorded");
	}

	printf("servo: %d failure(s)\n", failures);
	return failures ? 1 : 0;
}
