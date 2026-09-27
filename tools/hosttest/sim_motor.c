/* sim_motor.c -- a mechanism for servo.c to drive on the host.
 *
 * Models the three things that matter for safety: stiction (a duty below
 * which nothing moves), speed proportional to duty above it, and HARD END
 * STOPS.  The real mechanism has all three and the first motor test met the
 * third one at speed.
 */
#include <stdint.h>
#include <stdlib.h>
#include "board.h"

struct host_port host_port;

/* --- the simulated mechanism --------------------------------------------- */
double   sim_pos;             /* counts, real-valued */
int      sim_stop_lo = -1000000, sim_stop_hi = 1000000;
/* TWO FRICTION NUMBERS, NOT ONE.  Measured on the camera over two bench runs
 *:
 *
 *   breakaway  the duty needed to start from rest.  NOT a constant -- twelve
 *              ramp trials gave 180 200 200 210 230 240 forward and
 *              190 190 200 200 210 340 reverse.  A distribution with a tail.
 *   kinetic    the duty intercept once it is already moving: 134, from a
 *              twenty-point fit of v = 0.0399 * (duty - 134).
 *
 * The gap between them is the whole problem.  A single sim_stiction could not
 * express it, so the simulator could not produce the failure the hardware
 * produces -- a mechanism that will not start at a duty that would happily
 * keep it going.  Any test of anti-stiction behaviour against the old model
 * was testing nothing. */
int      sim_breakaway = 210;
/* A STIFF SPOT SOMEWHERE ALONG THE TRAVEL.
 *
 * The helicoid is not uniform.  One +500 move on the camera accelerated to
 * 387, slid BACK 25 counts, crawled for 280 ms while the boost wound up to
 * duty 526, then broke free and lurched 84 counts past the target.  Nothing
 * in a model with one breakaway figure for the whole travel can do that, so
 * nothing could test the fix.
 *
 * sim_stick_extra is added to the breakaway threshold while the mechanism is
 * inside [sim_stick_lo, sim_stick_hi].  Zero extra, the default, is the
 * uniform mechanism the earlier runs were modelled with. */
int      sim_stick_lo, sim_stick_hi, sim_stick_extra;
/* THE KINETIC INTERCEPT IS NOT THE SAME BOTH WAYS.  Fitted per direction over
 * two measurement runs: forward 139 and 161, reverse 130 and 138.  A single figure of 134 for both is what the controller's ff_offset
 * assumed, and it is why the reverse 200-count moves on hardware overshot by
 * 51-59 counts while the forward ones did not -- the feedforward over-drives
 * in the direction that needs less.
 *
 * The simulator could not show that with one number, so it reported those
 * moves as fine. */
int      sim_kinetic     = 150;   /* forward */
int      sim_kinetic_rev = 134;   /* reverse: freer once moving */
/* v = 0.0399 * (duty - 134) counts/ms, twenty points across both runs and
 * both directions.  Was 0.0344, from a single 200 ms
 * average that included the rise. */
double   sim_counts_per_ms_per_duty = 0.0399;
int      sim_encoder_stuck;
/* Measured: 3-5 counts peak to peak, sigma 0.7-0.9.  So the
 * half-band is 2, not the 3 guessed here before. */
int      sim_noise = 2;
static unsigned noise_state = 12345;
double   sim_encoder_gain = 1.0;   /* >1 fakes a runaway; <0 inverts the sign */

static int16_t sim_duty;
uint32_t host_millis;

/* MOMENTUM.  The first version of this simulator had none: motor_release()
 * simply zeroed the duty and the mechanism stopped dead.  So the suite could
 * not have caught the real failure -- releasing floats the bridge and the
 * mechanism coasts, overshooting by up to 1117 counts after a 3884-count
 * move.  A model that cannot express the bug cannot test the fix.
 *
 * sim_vel carries counts/ms.  Released, it decays slowly; braked, it is killed
 * at once, which is what shorting the winding does. */
double   sim_vel;
/* Calibrated against the hardware: a move ending at ~20 counts/ms overshot by
 * 474 and 848 counts under the brake, and by over 1100 when
 * released.  These reproduce that order of magnitude -- the model is for
 * exercising the control logic, not for predicting the mechanism. */
/* WHERE THE MECHANISM WENT, and whether it ever hit a stop.
 *
 * A scripted run drives a real helicoid with no camera watching, so the
 * question the host has to answer before anything is flashed is "can this
 * reach an end stop?".  The clamp below is silent
 * -- it just pins sim_pos -- so a test that only looked at the final position
 * could not tell a bounded run from one that spent a second leaning on a stop.
 * These record it. */
double   sim_pos_min, sim_pos_max;
uint32_t sim_clamp_lo_ms, sim_clamp_hi_ms;   /* millis of the LAST clamp, 0 = never */
unsigned sim_clamp_n;

/* All three measured, and all three were wrong before.
 *
 * The decays come from the stopping distance, which is v/(1-decay): braked
 * gave a time constant of 28-42 ms across five cases and released 23-33, so
 * the brake barely shortens a stop -- 0.970 and 0.964 rather than the 0.955
 * and 0.985 guessed here, which differed by 3x when the hardware differs by
 * a fifth.
 *
 * sim_accel_k is 1 - exp(-1/tau) for the DRIVEN time constant, which is not
 * the coasting one.  Under drive the motor's own back-EMF damps the response
 * and it settles faster; coasting with the bridge floating there is no
 * electrical damping at all.  The bench data says so plainly and I read it as
 * one number at first: the free step fits gave 14-26 ms forward, while every
 * coast gave 28-42.  Modelling acceleration with the coast figure made the
 * simulator take 79 ms over a 200-count move that the hardware does in 40,
 * and arrive at a fifth of the speed -- so the overshoot the hardware
 * actually suffers could not appear at all.
 *
 * It was 0.20, i.e. 5 ms, before any of this was measured. */
/* COULOMB FRICTION, counts/ms lost per ms, on top of the exponential decay.
 *
 * Without it a coast only ever asymptotes and never stops, so the model
 * always closed the last few counts to the target on its own -- and could not
 * reproduce the one defect the fourth bench run showed, which was every move
 * finishing short by about the width of the arrival band.  On hardware a
 * mechanism let go at half a count per millisecond travels about two more
 * counts and stops dead; this model had it travel sixteen.
 *
 * It is also why the stopping distance fitted as d ~ v^1.52 rather than as
 * either pure model: viscous drag dominates at 20-37
 * counts/ms, Coulomb friction dominates below about one, and the exponent
 * sits between them.  0.05 reproduces both ends -- 2.5 counts from 0.5
 * counts/ms, and 1200 from 37, where the viscous term rules. */
double   sim_coulomb     = 0.05;

double   sim_coast_decay = 0.964;  /* per ms, bridge floating, T = 28 ms */
double   sim_brake_decay = 0.970;  /* per ms, winding shorted, T = 33 ms */
double   sim_accel_k     = 0.0488; /* tau = 20 ms, DRIVEN */
int      sim_released;             /* 1 = bridge floating */

void motor_init(void) {}
void motor_attach_pins(void) { sim_released = 0; }
void motor_release(void) { sim_duty = 0; sim_released = 1; }
/* NOT sim_vel = 0.  Braking is not instantaneous -- shorting the winding
 * applies a retarding torque and the mechanism still travels while it decays.
 * Zeroing the velocity here was the instant-stop assumption this model exists
 * to remove, just relocated: it made the flat-duty overshoot read as 25 counts
 * where the hardware measured 474-848. */
void motor_brake(void)   { sim_duty = 0; sim_released = 0; }
void motor_drive(int16_t d) { sim_duty = d; sim_released = 0; }

/* Advance the mechanism one millisecond. Called from millis(), which the servo
 * loop polls -- so time only moves when the code under test looks at it. */
static void tick(void)
{
	int mag    = sim_duty < 0 ? -sim_duty : sim_duty;
	int moving = sim_vel > 0.05 || sim_vel < -0.05;
	/* Starting takes more than keeping going.  This is the one line that
	 * makes an anti-stiction integrator testable at all. */
	int kin    = (sim_duty < 0) ? sim_kinetic_rev : sim_kinetic;
	int thresh = moving ? kin : sim_breakaway;

	if (sim_stick_extra && sim_pos >= sim_stick_lo
	    && sim_pos <= sim_stick_hi) {
		thresh += sim_stick_extra;
	}

	if (mag > thresh) {
		/* First-order lag toward the duty's steady speed, not an instant
		 * jump.  The instant version could not represent a deceleration
		 * segment at all: any duty above stiction reached its final
		 * speed within one tick, so approaching slowly looked identical
		 * to arriving at full speed. */
		double v = (mag - kin) * sim_counts_per_ms_per_duty;

		if (sim_duty < 0) {
			v = -v;
		}
		sim_vel += (v - sim_vel) * sim_accel_k;
	} else {
		sim_vel *= sim_released ? sim_coast_decay : sim_brake_decay;
		/* and Coulomb friction on top, which is what actually brings
		 * it to REST rather than merely close to it. */
		if (sim_vel > sim_coulomb) {
			sim_vel -= sim_coulomb;
		} else if (sim_vel < -sim_coulomb) {
			sim_vel += sim_coulomb;
		} else {
			sim_vel = 0.0;
		}
	}
	sim_pos += sim_vel;
	if (sim_pos < sim_stop_lo) {
		sim_pos = sim_stop_lo;
		sim_clamp_lo_ms = host_millis; sim_clamp_n++;
	}
	if (sim_pos > sim_stop_hi) {
		sim_pos = sim_stop_hi;
		sim_clamp_hi_ms = host_millis; sim_clamp_n++;
	}
	if (sim_pos < sim_pos_min) sim_pos_min = sim_pos;
	if (sim_pos > sim_pos_max) sim_pos_max = sim_pos;
}

uint32_t millis(void) { tick(); return ++host_millis; }
void delay_ms(uint32_t ms) { while (ms--) millis(); }

static int noise(void)
{
	if (!sim_noise) return 0;
	noise_state = noise_state * 1103515245u + 12345u;
	return (int)((noise_state >> 16) % (unsigned)(2 * sim_noise + 1)) - sim_noise;
}

/* The real tracker counts from wherever it was last reset; the firmware homes
 * into the end stop and resets there, and every position it works in is
 * measured from that zero.  A sim that ignored the reset would put the
 * mechanism somewhere other than where the firmware thinks it is. */
static int32_t sim_track_off;

void abs_encoder_track_reset(void) { sim_track_off = (int32_t)sim_pos; }

int32_t abs_encoder_track(void)
{
	if (sim_encoder_stuck) return noise();
	return (int32_t)(sim_pos * sim_encoder_gain) + noise() - sim_track_off;
}
uint16_t abs_encoder_raw(void) { return (uint16_t)((int32_t)sim_pos & 0x3FFF); }

void sim_reset(void)
{
	sim_pos = 0; sim_duty = 0; host_millis = 0;
	/* The velocity state MUST be reset too, or a test inherits the previous
	 * one's momentum -- the same class of bug the noise seed comment below
	 * describes, and it hid itself as an unrelated timeout. */
	sim_vel = 0; sim_released = 0;
	sim_stop_lo = -1000000; sim_stop_hi = 1000000;
	sim_breakaway = 210; sim_kinetic = 150; sim_kinetic_rev = 134;
	/* THE DECAYS AND THE LAG TOO.  These were left out, so a test that
	 * changed one -- rt_coast_forever sets sim_brake_decay to 1.0 to take
	 * the drive away -- handed it to every test that ran afterwards, and
	 * each of those had to remember to put it back by hand.  One did, with
	 * a value that had since gone stale.  The result was a suite whose
	 * outcome depended on the order of its cases, which is the exact bug
	 * the noise seed below was made per-test to avoid. */
	sim_coast_decay = 0.964; sim_brake_decay = 0.970; sim_accel_k = 0.0488;
	sim_coulomb = 0.05;
	sim_stick_lo = sim_stick_hi = sim_stick_extra = 0;
	sim_encoder_stuck = 0; sim_encoder_gain = 1.0;
	sim_noise = 2;
	sim_track_off = 0;
	sim_pos_min = sim_pos_max = 0;
	sim_clamp_lo_ms = sim_clamp_hi_ms = 0; sim_clamp_n = 0;
	noise_state = 12345;   /* deterministic, and per-test: otherwise each
	                          case inherits the previous one's noise and the
	                          suite passes or fails on test ORDER */
}
