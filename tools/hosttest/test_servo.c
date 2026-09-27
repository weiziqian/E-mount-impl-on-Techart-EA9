/* test_servo.c -- prove the servo stops, in every way it can go wrong.
 *
 * This is the code that stands between a bug and a damaged helicoid, so the
 * cases that matter are the failures, not the success.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "servo.h"

extern double sim_pos;
extern int sim_stop_lo, sim_stop_hi, sim_breakaway, sim_kinetic, sim_kinetic_rev, sim_encoder_stuck, sim_noise;
extern int sim_stick_lo, sim_stick_hi, sim_stick_extra;
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

/* The same numbers src/main.c uses, from the bench measurements. */
static const struct servo_cfg SAFE = {
	.stop_ms   = 30,
	.v_cruise  = 25000,
	.ff_div    = 40,
	.ff_offset = 134,
	.tau_ms    = 26,
	.kp_div    = 50,
	.v_arrive  = 500,
	.arrive_steps = 2,
	.boost_up   = 12,
	.boost_down = 8,
	.boost_max  = 800,
	.duty_floor = 200,
	.duty_max   = 2560 * 50 / 100,
	.tolerance = 12, .settle_counts = 40, .aim_ahead = 0,
	.noise     = 8,
	.stall_ms  = 300,
	/* 3000, matching src/main.c.  It was 1200, which is a value the
	 * firmware does not use -- and at e/30 against the worst breakaway
	 * the ramp measured, a 50-count move takes 1793 ms and was failing
	 * here as a TIMEOUT that the shipped configuration would never
	 * produce.  A test config that differs from the shipped one tests
	 * something nobody runs. */
	.timeout_ms = 3000,
	.pump      = count_pump,
	.reverse_brake_ms    = 20,
	.reverse_brake_above = 15000,
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

/* ARRIVING.  The one thing the old controller could not do.
 *
 * It drove flat out to the target and stopped, and stopping takes 30 ms of
 * coasting whatever you do with the bridge -- so every move overshot by
 * hundreds of counts and the body spent its time correcting (a measurement run
 * measured 334-555 on hardware; this simulator, once calibrated to the
 * mechanism, reproduces 337-514).
 *
 * The new law never asks for a speed the remaining distance cannot absorb, so
 * there is nothing left to coast.  Across sizes and both directions, because
 * testing one size and one direction is how the old suite missed that a whole
 * move was running in the approach segment. */
static void check_arrival(void)
{
	static const int32_t SIZE[] = { 50, 100, 200, 500, 1000, 3000 };
	unsigned i, d;
	int32_t  worst_over = 0, worst_err = 0;
	/* Signed, IN THE DIRECTION OF TRAVEL: negative means it came up
	 * short.  Summed over every case, because the failure this exists to
	 * catch is a bias, not a spread -- see below. */
	int32_t  bias = 0;
	int      n_bias = 0;
	uint32_t total_ms = 0;

	for (d = 0; d < 2; d++) {
		for (i = 0; i < sizeof SIZE / sizeof SIZE[0]; i++) {
			struct servo_cfg cfg = SAFE;
			struct servo_result r;
			int32_t want = d ? -SIZE[i] : SIZE[i];
			int32_t err, over;
			int k;

			sim_reset();
			servo_move_rel(want, &cfg, &r);
			/* Let whatever it is doing finish before judging. */
			for (k = 0; k < 300; k++) {
				(void)host_tick();
			}

			err  = (int32_t)sim_pos - want;
			over = d ? -err : err;
			bias += over;
			n_bias++;
			total_ms += r.ms;
			if (err < 0) {
				err = -err;
			}
			if (err > worst_err) {
				worst_err = err;
			}
			if (over > worst_over) {
				worst_over = over;
			}

			if (r.outcome != SERVO_OK) {
				fail("%+d counts: %s, not OK",
				     (int)want, NAME[r.outcome]);
			}
			/* The tolerance is 16, but the mechanism cannot be
			 * placed finer than one control step at the floor
			 * duty -- about 13 counts -- and then it coasts.  60
			 * is the honest bar for a powered move, and is 15
			 * PROTOCOL units inside the body's own +-70 scatter. */
			if (err > 60) {
				fail("%+d counts ended %d off target",
				     (int)want, (int)err);
			}
		}
	}
	printf("  arrival: worst error %d counts, worst overshoot %d, "
	       "mean bias %+d, %u ms over %d moves\n",
	       (int)worst_err, (int)worst_over, (int)(bias / n_bias),
	       total_ms, n_bias);

	/* HOW LONG IT TAKES, which nothing checked until the first autofocus
	 * run said the lens was visibly slow.
	 *
	 * Every constant in this controller had a guard except the one the
	 * user could actually perceive.  Slackening the profile back to e/60
	 * nearly trebles this and no other assertion here notices.
	 *
	 * 2200 sits between the settings that matter: 1374 ms at the shipped
	 * e/22, 1529 at e/30, 4089 at e/60.  Loose enough to admit e/30 --
	 * that is a defensible choice, not a regression -- and tight enough
	 * to reject the one that made the lens visibly crawl. */
	if (total_ms > 2200) {
		fail("%u ms for %d moves -- the approach profile has been "
		     "slowed; a lens that arrives late is the complaint this "
		     "controller exists to answer", total_ms, n_bias);
	}

	/* And the worst single error, which the per-move check at 60 is too
	 * loose to catch.  A wider arrival band no longer shows up as a bias
	 * -- the faster profile carries the mechanism through it rather than
	 * stopping at its edge -- so it shows up here instead, at 29 counts
	 * against the 11 the shipped configuration gives. */
	if (worst_err > 20) {
		fail("worst arrival error %d counts", (int)worst_err);
	}

	/* NO SYSTEMATIC BIAS.
	 *
	 * A move approaches its target from one side, so anything that lets
	 * it stop early stops it early EVERY TIME and in the same direction.
	 * On hardware, with the arrival band set to 32, all twenty-two moves
	 * came up short and a 40-count move could finish after travelling
	 * eight -- and the suite passed, because it only ever looked at the
	 * worst absolute error.
	 *
	 * A bias is worse than a spread of the same size: the body has to
	 * correct it on every single command, which is indistinguishable
	 * from the lens not going where it was told. */
	/* The threshold is 9, not zero and not the 16 it was.
	 *
	 * A move stops at the near edge of the arrival band, so before
	 * cfg->aim_ahead existed the bias was the band width -- about -12,
	 * and narrowing the band only bought a count or two while trebling
	 * the time.  Aiming a band's width beyond the target cancels the
	 * geometry and leaves -4, which is the pulse-and-coast endgame dying
	 * against Coulomb friction: the mechanism's resolution, not a tuning
	 * choice.
	 *
	 * 9 sits between the two, so removing the aim offset fails here
	 * rather than passing quietly at -12. */
	if (bias / n_bias > 9 || bias / n_bias < -9) {
		fail("mean error %+d counts across %d moves -- that is a BIAS, "
		     "not scatter; the servo is stopping early in the "
		     "direction of travel", (int)(bias / n_bias), n_bias);
	}

	/* The number this replaces: 337-514 counts of overshoot. */
	if (worst_over > 120) {
		fail("worst overshoot %d counts -- the profile is not "
		     "taking the speed away before the target",
		     (int)worst_over);
	}
}

/* A STIFF SPOT PART WAY ALONG, which is where the boost turns into a lurch.
 *
 * One +500 move on the camera accelerated to 387, slid back 25 counts,
 * crawled for 280 ms while the boost wound the duty up to 526, broke free and
 * shot 84 counts past the target -- and then the servo declared arrival AT THE
 * TOP OF THE ARC, where the mechanism was momentarily stationary with a hard reverse
 * drive already applied.  It let go and ran another 110 counts backwards,
 * finishing 82 short.
 *
 * Two faults in one move: a mechanism that cannot be expected to move
 * smoothly, and an arrival test that cannot tell standing still from having
 * stopped.  This case holds both. */
static void check_stiff_spot(void)
{
	struct servo_cfg cfg = SAFE;
	struct servo_result r;
	double peak;
	int i;

	cfg.duty_max = 1280;

	sim_reset();
	/* Positioned so it breaks free CLOSE TO THE TARGET, which is what
	 * makes the mechanism swing past and turn round near it -- and a
	 * turnaround is the only place the arrival test can mistake standing
	 * still for having stopped.  A stiff spot earlier in the travel is
	 * survivable without the dwell and proves nothing about it. */
	sim_stick_lo = 380; sim_stick_hi = 470; sim_stick_extra = 400;
	servo_move_rel(500, &cfg, &r);
	peak = sim_pos;
	for (i = 0; i < 400; i++) {
		(void)host_tick();
		if (sim_pos > peak) {
			peak = sim_pos;
		}
	}

	if (r.outcome != SERVO_OK) {
		fail("stiff spot: %s -- the boost did not get it through",
		     NAME[r.outcome]);
	}
	/* WHERE IT STOPS MUST BE WHERE IT ARRIVED.  The hardware failure was
	 * not the lurch -- it is a mechanism, it lurches -- but the servo
	 * letting go mid-reversal and the mechanism then travelling on. */
	/* 25, which is between the two behaviours rather than outside both:
	 * with the dwell in place the mechanism runs on 10 counts past where
	 * it stops being driven, and without it 37.  A threshold of 40 admits
	 * them both and guards nothing -- which is what it did. */
	if (peak - sim_pos > 25) {
		fail("stiff spot: peaked at %.0f and ended at %.0f -- it let "
		     "go while still moving", peak, sim_pos);
	}
	if (sim_pos - 500 > 60 || sim_pos - 500 < -60) {
		fail("stiff spot: ended %.0f counts off target", sim_pos - 500);
	}
}

/* BREAKAWAY IS A DISTRIBUTION, so the integral has to find it every time.
 *
 * Twelve ramp trials on the camera gave 180..340.  A fixed
 * duty cannot serve that range, which is why the old duty_approach of 250
 * stalled on the tail.  The integral climbs until the mechanism goes; this
 * checks it does so across the whole measured band and beyond it. */
static void check_breakaway_band(void)
{
	/* Up to 420 only.  Camera runs have found spots on the real helicoid
	 * that report duty_first 1280 -- pinned against the ceiling -- and a
	 * case at 1300 was briefly in this list while duty_max was 65%.
	 * That ceiling is back at the stock's 1280, so a 1300-count
	 * breakaway is now a KNOWN LIMITATION rather than something to
	 * assert against: the mechanism cannot be freed from such a spot and
	 * the move will stall until the body re-commands it. */
	static const int band[] = { 180, 210, 240, 280, 340, 420 };
	unsigned i;

	for (i = 0; i < sizeof band / sizeof band[0]; i++) {
		struct servo_cfg cfg = SAFE;
		struct servo_result r;

		sim_reset();
		sim_breakaway = band[i];
		servo_move_rel(400, &cfg, &r);

		if (r.outcome != SERVO_OK) {
			fail("breakaway %d: %s -- the integral did not get it "
			     "moving", band[i], NAME[r.outcome]);
		}
		if (r.duty_first && (int)r.duty_first < band[i] - 40) {
			fail("breakaway %d: reported moving at duty %u, which "
			     "is below what it takes", band[i], r.duty_first);
		}
	}

	/* AND THE CASE THE BOOST ACTUALLY EXISTS FOR.
	 *
	 * A move long enough to command real speed gets a large drive on its
	 * very first control step -- the reference jumps from nothing to the
	 * cruise speed, which asks for an acceleration the clamp turns into
	 * one step at duty_max -- and that alone happens to clear the whole
	 * breakaway band.  So the cases above pass with the boost disabled,
	 * which makes them no test of it: mutating boost_up to zero cost one
	 * failure out of the entire suite.
	 *
	 * A SHORT move gets no such kick.  At 40 counts the profile asks for
	 * 0.95 counts/ms, the first step reaches about duty 270, and a
	 * mechanism that needs 340 simply sits there until something climbs.
	 * That something is the boost, and this is where it is tested. */
	{
		struct servo_cfg cfg = SAFE;
		struct servo_result r;

		sim_reset();
		sim_breakaway = 340;      /* the worst trial measured */
		servo_move_rel(40, &cfg, &r);

		if (r.outcome != SERVO_OK) {
			fail("a 40-count move against a 340 breakaway ended "
			     "%s -- nothing climbed the duty to meet it",
			     NAME[r.outcome]);
		}
	}
}

static void check_reports_final_position(void)
{
	struct servo_cfg cfg = {
		.stop_ms = 30, .v_cruise = 25000,
		.ff_div = 40, .ff_offset = 134, .tau_ms = 26, .kp_div = 50, .v_arrive = 500, .arrive_steps = 2,
		.boost_up = 12, .boost_down = 8, .boost_max = 800,
		.duty_floor = 200, .duty_max = 800,
		.stall_ms = 300,
		.timeout_ms = 3000, .tolerance = 12, .settle_counts = 40, .aim_ahead = 0, .noise = 8,
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
extern int     sim_breakaway, sim_kinetic, sim_kinetic_rev;
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
		/* BOTH friction numbers.  Setting only the breakaway left the
		 * motor its full authority, because the mechanism was already
		 * moving and a moving mechanism is governed by the kinetic
		 * figure -- so the test that was meant to remove the drive
		 * entirely removed nothing, and passed. */
		sim_breakaway   = 2000;
		sim_kinetic = 2000; sim_kinetic_rev = 2000;   /* no duty we apply can move it */
		sim_brake_decay = 1.0;    /* and nothing slows it down    */
	}
	return 1;
}

static struct servo_cfg rt_cfg(void)
{
	struct servo_cfg c = {
		.stop_ms = 30, .v_cruise = 25000,
		.ff_div = 40, .ff_offset = 134, .tau_ms = 26, .kp_div = 50, .v_arrive = 500, .arrive_steps = 2,
		.boost_up = 12, .boost_down = 8, .boost_max = 800,
		.duty_floor = 200, .duty_max = 800, .stall_ms = 300,
		.timeout_ms = 3000, .tolerance = 12, .settle_counts = 40, .aim_ahead = 0, .noise = 8,
		.pump = count_pump,
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

/* THE SAME MOVES, AGAINST A MECHANISM THAT DOES NOT WANT TO START.
 *
 * check_arrival runs at the simulator's default breakaway of 210, which is
 * the easy end of what the ramp measured -- and at that end the breakaway
 * search barely matters, so nothing there protected it.  Mutating the climb
 * rate back to its old value, or removing the first-step kick, failed not one
 * assertion in the whole suite.
 *
 * The real mechanism is not that: on one camera run 13 of 36 moves needed a
 * duty of 1200 or more to start, and those took a median of 370 ms against
 * 186 for the rest.  536 is the worst the ramp ever measured directly.
 *
 * Times at 536, for the six sizes below, both directions:
 *     old search (+12/cycle, bleed, no kick)   3424 ms, worst error 17
 *     as shipped (+64, drop on move, kick)     1514 ms, worst error  9
 */
static void check_arrival_stiff(void)
{
	static const int32_t SIZE[] = { 50, 100, 200, 500, 1000, 3000 };
	unsigned i, d;
	uint32_t total_ms = 0;
	int32_t  worst_err = 0;

	for (d = 0; d < 2; d++) {
		for (i = 0; i < sizeof SIZE / sizeof SIZE[0]; i++) {
			struct servo_cfg cfg = SAFE;
			struct servo_result r;
			int32_t want = d ? -SIZE[i] : SIZE[i];
			int32_t err;
			int k;

			sim_reset();
			sim_breakaway = 536;
			servo_move_rel(want, &cfg, &r);
			for (k = 0; k < 300; k++) {
				(void)host_tick();
			}
			if (r.outcome != SERVO_OK) {
				fail("stiff %+d counts: %s",
				     (int)want, NAME[r.outcome]);
			}
			total_ms += r.ms;
			err = (int32_t)sim_pos - want;
			if (err < 0) {
				err = -err;
			}
			if (err > worst_err) {
				worst_err = err;
			}
		}
	}
	printf("  stiff:   worst error %d counts, %u ms over 12 moves\n",
	       (int)worst_err, total_ms);

	/* NO TIMING ASSERTION HERE, and that is a deliberate retreat.
	 *
	 * A bound was put on this when the fast breakaway search went in, and
	 * the search has since been reverted -- it was much worse on the
	 * camera.  Re-measuring what is left shows why a bound would
	 * be worthless anyway: against a stiff mechanism the total is not a
	 * monotonic function of the profile at all.
	 *
	 *     e/22  2772 ms     e/26  3684     e/30  5283     e/60  4656
	 *
	 * The endgame is a pulse-and-coast and a small change in where the
	 * pulses land changes how many there are.  A threshold over that is
	 * a threshold over noise.  What this case still checks is the part
	 * that IS stable: a stiff mechanism must still be reached, and
	 * reached accurately.
	 *
	 * The timing guard that does work lives in check_arrival, where the
	 * mechanism is the easy one and e/60 shows up as 4089 ms against
	 * 1409. */
	/* 25, not 20.  Against the worst breakaway the ramp measured, the
	 * shipped e/30 lands within 22 counts; e/22 and e/26 manage 14 and
	 * 19.  The slower profile spends longer creeping through the dead
	 * zone at the end and picks up a few more counts of error doing it.
	 * That is a real cost of the setting, chosen on how it shoots, and
	 * the bound records it rather than hiding it. */
	if (worst_err > 25) {
		fail("stiff: worst arrival error %d counts", (int)worst_err);
	}

}

/* IN POSITION IS NOT ARRIVED -- and this case can no longer prove it.
 *
 * The rule is real and was found on hardware: a move
 * overshot to 7 counts past the aim, the loop was already driving it back,
 * and at the top of the arc the measured speed was momentarily nothing.  The
 * servo read position-in-band and speed-below-threshold, declared arrival,
 * let go, and the mechanism carried on backwards for another 110 counts.
 * `arrive_steps` requires the condition to hold for two consecutive control
 * steps, which a turnaround cannot do.
 *
 * WHAT THIS CASE NOW SHOWS IS THAT IT CANNOT BE ISOLATED, which is worth a
 * test of its own kind.  The tau feedforward handles the same situation, and
 * handles it so well that the dwell is never reached: move the target to 60
 * counts ahead of a mechanism at cruise and the derivative term swings the
 * output to full reverse, stopping it 3 counts past -- with the dwell or
 * without it.  Disabling the feedforward does not help either, because then
 * the drive is too weak to overshoot at all.
 *
 * So the dwell is a second line of defence behind a first line that is
 * currently very effective, and the suite has no scenario that reaches past
 * the first.  It is kept on hardware evidence, not on simulation: the stiff
 * spots that produced a measurement run's overshoot are not something this model makes.
 *
 * The case is kept as a regression on the OUTCOME -- a move cut short at
 * speed must still end up in the right place -- which is what it can honestly
 * assert.
 */
static void check_shortened_at_speed(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;
	double peak;
	int i;

	cfg.duty_max = 1280;

	sim_reset();
	rt_arm(1200, 1260);          /* 60 counts ahead, at cruise */
	servo_move_rel(3000, &cfg, &r);
	peak = sim_pos;
	for (i = 0; i < 400; i++) {
		(void)host_tick();
		if (sim_pos > peak) {
			peak = sim_pos;
		}
	}

	if (!rt_count) {
		fail("shortened move: the hook never fired");
	}
	if (r.outcome != SERVO_OK) {
		fail("a move cut short at speed ended %s", NAME[r.outcome]);
	}
	if (sim_pos - 1260 > 60 || sim_pos - 1260 < -60) {
		fail("a move cut short at speed ended %.0f counts out",
		     sim_pos - 1260);
	}
	if (peak - 1260 > 400) {
		fail("and overshot by %.0f counts on the way", peak - 1260);
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
	/* Shorter than the whole two-leg journey, longer than either leg.
	 *
	 * Re-sized twice now.  120 ms was the old bang-bang's, which arrived
	 * fast and overshot.  320 was the profile's, before the endgame
	 * became a pulse-and-coast -- at which point a single 1500-count leg
	 * took 367 ms and the whole two-leg journey only 408, so the test had
	 * almost no window left to discriminate in and failed on the first
	 * leg.  Redirecting FURTHER out gives the second leg real length:
	 * ~350 ms for the first, ~600 for the second, ~950 together. */
	cfg.timeout_ms = 700;
	sim_reset();
	rt_arm(1500, 4000);
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


	if (getenv("SV_DEBUG")) {
		printf("  [rb] outcome %s ms %u end %d target %d retargets %u\n",
		       NAME[r.outcome], r.ms, r.end, r.target, r.retargets);
	}
	if (r.outcome == SERVO_STALL) {
		fail("a %u ms reverse brake against a %u ms stall window was "
		     "reported as a stall", cfg.reverse_brake_ms, cfg.stall_ms);
	}
}

/* A redirect while the mechanism is still stuck must not lose the integral's
 * work.  The body re-issues a target every 16 ms or so, and a mechanism that
 * will not break loose has to keep accumulating force across the whole burst
 * rather than starting from the feedforward each time -- which is exactly why
 * the stock never resets its accumulator (servo.md a measurement run) and why this one does
 * not reset it on a retarget either. */
static void check_retarget_keeps_the_integral(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	cfg.duty_max = 800;

	sim_reset();
	sim_breakaway = 400;           /* well past the feedforward's reach */
	rt_arm(20, 3000);              /* redirect almost immediately */
	servo_move_rel(2000, &cfg, &r);

	if (!rt_count) {
		fail("integral retention: the hook never fired");
	}
	if (r.outcome != SERVO_OK) {
		fail("a redirect while still stuck ended %s -- the integral "
		     "was thrown away and had to start again", NAME[r.outcome]);
	}
}

/* Redirecting must put the profile back in play for the NEW target: the
 * remaining distance is what sets the commanded speed, so a stale target
 * would have the mechanism decelerating toward the wrong place. */
static void check_retarget_reprofiles(void)
{
	struct servo_cfg cfg = rt_cfg();
	struct servo_result r;

	sim_reset();
	/* Fire when the original 2000-count move is nearly done, so the
	 * profile has already wound the speed right down. */
	rt_arm(1700, 4000);
	servo_move_rel(2000, &cfg, &r);
	for (int i = 0; i < 400; i++) {
		(void)host_tick();
	}

	if (!rt_count) {
		fail("reprofile: the hook never fired");
	}
	if (sim_pos - 4000 > 200 || sim_pos - 4000 < -200) {
		fail("ended %.0f counts from the redirected target",
		     sim_pos - 4000);
	}
	/* And it must speed up again for the new distance rather than crawl
	 * the rest at the speed the old target had wound it down to.  That
	 * failure arrives in the right place, gently, and every position
	 * assertion passes -- only the clock shows it. */
	/* 786 ms measured with the throttle reopening properly; 1000 leaves
	 * margin without admitting a leg run at the old wound-down speed,
	 * which would be several times longer. */
	if (r.ms > 1000) {
		fail("the redirected move took %u ms -- it never re-opened "
		     "the throttle for the new distance", r.ms);
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
	check_shortened_at_speed();
	check_retarget_forward();
	check_retarget_reversal();
	check_retarget_deadband();
	check_retarget_cap();
	check_retarget_rebaselines_wrong_way();
	check_reversal_rearms_wrong_way();
	check_retarget_resets_the_deadline();
	check_total_ceiling();
	check_reverse_brake_is_not_a_stall();
	check_retarget_keeps_the_integral();
	check_retarget_reprofiles();
	check_reversal_coast_window_is_bounded();
	check_no_retarget_unchanged();
	check_holds_after_move();
	check_reports_final_position();
	check_arrival();
	check_arrival_stiff();
	check_breakaway_band();
	check_stiff_spot();
	struct servo_result r;

	/* 1. a normal move arrives, and stops near the target */
	sim_reset();
	servo_move_rel(200, &SAFE, &r);
	if (r.outcome != SERVO_OK) {
		fail("free move: outcome %s, expected OK", NAME[r.outcome]);
	}
	/* The band is no longer one-sided.  The old controller could only
	 * overshoot -- it drove to the target and stopped -- so a lower bound
	 * of 185 was safe.  This one finishes by letting go and coasting the
	 * last stretch, which lands it slightly SHORT about as often as
	 * slightly long, and that is the correct behaviour: the alternative
	 * is holding the floor duty all the way in and arriving 86 counts
	 * past.  40 counts is 10 protocol units, against the body's own
	 * +-70-unit target scatter. */
	if (r.end < 160 || r.end > 240) {
		fail("free move: ended at %d, expected near 200", r.end);
	}
	if (!r.duty_first) {
		fail("free move: never recorded the duty it started moving at");
	}

	/* 2. reverse works and is symmetric */
	sim_reset();
	servo_move_rel(-200, &SAFE, &r);
	/* The band is symmetric now.  The tolerance is 32 and the endgame
	 * finishes by coasting, so a move may land a little short as readily
	 * as a little long -- 200 +- 45 is the honest window, and it is 11
	 * protocol units against the body's own +-70 of target scatter. */
	if (r.outcome != SERVO_OK || r.end > -155 || r.end < -245) {
		fail("reverse: %s, ended %d", NAME[r.outcome], r.end);
	}

	/* 3. breakaway well above what the feedforward asks for: the integral
	 * must climb until it goes, rather than giving up. */
	sim_reset();
	sim_breakaway = 2560 * 9 / 100;      /* 230, inside the measured band */
	servo_move_rel(200, &SAFE, &r);
	if (r.outcome != SERVO_OK) {
		fail("breakaway: outcome %s, expected the integral to break "
		     "it free", NAME[r.outcome]);
	}
	if (r.duty_first && r.duty_first < 2560 * 9 / 100 - 40) {
		fail("breakaway: duty_first %u, below what it takes",
		     r.duty_first);
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
	/* The stall window is 300 ms now, against the old bang-bang's 60: it
	 * has to outlast the boost's climb through the breakaway band, which
	 * takes about 130 ms at the worst measured figure.  Pushing into a
	 * stop for that long is what the stock's own homing does at every
	 * power-on (300 ms at duty 800), so it is inside normal operation
	 * rather than a new risk. */
	if (r.ms > 380) {
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
	if (r.ms > 380) {              /* the 300 ms stall window; see above */
		fail("noisy end stop: pushed for %u ms before giving up", r.ms);
	}

	/* 5. stiction so high nothing can move it: still stalls, never hangs */
	sim_reset();
	sim_breakaway = 2560;
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

	/* 7. feedback wildly wrong.
	 *
	 * WRONG_WAY is accepted alongside RUNAWAY, and the reason is worth
	 * stating rather than hiding in a list.  The old controller latched a
	 * direction and could only ever drive past the target, so a
	 * mis-scaled encoder could only ever look like a runaway.  This one
	 * corrects an overshoot, so it turns round and drives back -- and an
	 * encoder reading forty times the truth then makes that correction
	 * look like travelling away from the start.  Both verdicts say the
	 * same thing: the feedback is not to be trusted, stop.  The assertion
	 * that carries the weight is the next one. */
	sim_reset();
	sim_encoder_gain = 40.0;
	servo_move_rel(200, &SAFE, &r);
	if (r.outcome != SERVO_OK && r.outcome != SERVO_RUNAWAY
	    && r.outcome != SERVO_WRONG_WAY) {
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
	/* WRONG_WAY_MARGIN is 120, so it cannot possibly stop in less than
	 * that, and whatever speed it has reached by then it still has to
	 * coast: 161 counts measured.  200 leaves room without admitting a
	 * guard that has stopped firing -- removing the guard entirely lets
	 * the move run to its timeout, thousands of counts away.
	 *
	 * The margin went from 40 to 120 because 40 was inside the
	 * mechanism's own spring-back and a camera tripped it on a perfectly
	 * good move.  This bound moved with it. */
	if (sim_pos < -200 || sim_pos > 200) {
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
	 * made a real body re-run its init handshake mid-move. */
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
