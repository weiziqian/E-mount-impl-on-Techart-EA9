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
int      sim_stiction = 150;  /* duty below which it does not move */
/* From the measured run: 7772 encoder counts in 200 ms at duty 1280, i.e.
 * 38.9 counts/ms, with stiction taken off the duty first. */
double   sim_counts_per_ms_per_duty = 0.0344;
int      sim_encoder_stuck;
/* The real encoder jitters 2-3 counts when stationary.  The simulation had
 * none, which is why it could not catch a stall detector that treated one
 * count of jitter as movement. */
int      sim_noise = 3;
static unsigned noise_state = 12345;
double   sim_encoder_gain = 1.0;   /* >1 fakes a runaway; <0 inverts the sign */

static int16_t sim_duty;
uint32_t host_millis;

/* MOMENTUM.  The first version of this simulator had none: motor_release()
 * simply zeroed the duty and the mechanism stopped dead.  So the suite could
 * not have caught the real failure -- releasing floats the bridge and the
 * mechanism coasts, overshooting by up to 1117 counts after a 3884-count move
 * (NOTES.md §57).  A model that cannot express the bug cannot test the fix.
 *
 * sim_vel carries counts/ms.  Released, it decays slowly; braked, it is killed
 * at once, which is what shorting the winding does. */
double   sim_vel;
/* Calibrated against the hardware: a move ending at ~20 counts/ms overshot by
 * 474 and 848 counts under the brake (NOTES.md §60), and by over 1100 when
 * released.  These reproduce that order of magnitude -- the model is for
 * exercising the control logic, not for predicting the mechanism. */
double   sim_coast_decay = 0.985;  /* per ms, bridge floating  */
double   sim_brake_decay = 0.955;  /* per ms, winding shorted  */
double   sim_accel_k     = 0.20;   /* approach to the duty's steady speed */
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
	int mag = sim_duty < 0 ? -sim_duty : sim_duty;

	if (mag > sim_stiction) {
		/* First-order lag toward the duty's steady speed, not an instant
		 * jump.  The instant version could not represent a deceleration
		 * segment at all: any duty above stiction reached its final
		 * speed within one tick, so approaching slowly looked identical
		 * to arriving at full speed. */
		double v = (mag - sim_stiction) * sim_counts_per_ms_per_duty;

		if (sim_duty < 0) {
			v = -v;
		}
		sim_vel += (v - sim_vel) * sim_accel_k;
	} else if (sim_released) {
		sim_vel *= sim_coast_decay;       /* floating: carries on */
	} else {
		sim_vel *= sim_brake_decay;       /* shorted winding: decays fast */
	}
	sim_pos += sim_vel;
	if (sim_pos < sim_stop_lo) sim_pos = sim_stop_lo;
	if (sim_pos > sim_stop_hi) sim_pos = sim_stop_hi;
}

uint32_t millis(void) { tick(); return ++host_millis; }
void delay_ms(uint32_t ms) { while (ms--) millis(); }

static int noise(void)
{
	if (!sim_noise) return 0;
	noise_state = noise_state * 1103515245u + 12345u;
	return (int)((noise_state >> 16) % (unsigned)(2 * sim_noise + 1)) - sim_noise;
}

int32_t abs_encoder_track(void)
{
	if (sim_encoder_stuck) return noise();
	return (int32_t)(sim_pos * sim_encoder_gain) + noise();
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
	sim_stiction = 150; sim_encoder_stuck = 0; sim_encoder_gain = 1.0;
	sim_noise = 3;
	noise_state = 12345;   /* deterministic, and per-test: otherwise each
	                          case inherits the previous one's noise and the
	                          suite passes or fails on test ORDER */
}
