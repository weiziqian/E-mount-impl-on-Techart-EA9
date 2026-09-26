#include "board.h"
#include "motor.h"
#include "abs_encoder.h"
#include "delay.h"

static void tcc0_sync_cc(void)
{
	/* bits 8..11 are CC0..CC3; the stock setters wait on exactly this mask */
	while (TCC0->SYNCBUSY.reg & 0x0f00u) {
	}
}

static void tcc0_sync_all(void)
{
	while (TCC0->SYNCBUSY.reg) {
	}
}

/* The stock drive functions re-apply the pin mux on every call, because the
 * release path (0x8e28) takes the pins away from TCC0 entirely.  Same here. */
void motor_attach_pins(void)
{
	PORT->Group[0].PINCFG[MOTOR_PIN_WO0].reg = PORT_PINCFG_PMUXEN;
	PORT->Group[0].PINCFG[MOTOR_PIN_WO1].reg = PORT_PINCFG_PMUXEN;
	PORT->Group[0].PMUX[MOTOR_PIN_WO0 / 2].reg =
	        PORT_PMUX_PMUXE(MOTOR_MUX_WO01) | PORT_PMUX_PMUXO(MOTOR_MUX_WO01);
	PORT->Group[0].PINCFG[MOTOR_PIN_WO2].reg = PORT_PINCFG_PMUXEN;
	PORT->Group[0].PINCFG[MOTOR_PIN_WO3].reg = PORT_PINCFG_PMUXEN;
	PORT->Group[0].PMUX[MOTOR_PIN_WO2 / 2].reg =
	        PORT_PMUX_PMUXE(MOTOR_MUX_WO23) | PORT_PMUX_PMUXO(MOTOR_MUX_WO23);
}

/* coil A = WO[0]/WO[1] on PA08/PA09 -- stock 0x9070 writes CC[0], CC[1] */
static void pair_a(uint16_t fwd, uint16_t rev)
{
	TCC0->CC[0].reg = fwd;
	tcc0_sync_cc();
	TCC0->CC[1].reg = rev;
	tcc0_sync_cc();
}

/* coil B = WO[2]/WO[3] on PA18/PA19 -- stock 0x9098 writes CC[2], CC[3] */
static void pair_b(uint16_t fwd, uint16_t rev)
{
	TCC0->CC[2].reg = fwd;
	tcc0_sync_cc();
	TCC0->CC[3].reg = rev;
	tcc0_sync_cc();
}

void motor_init(void)
{
	PM->APBCMASK.reg |= PM_APBCMASK_TCC0;
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TCC0_TCC1_Val)
	                    | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
	while (GCLK->STATUS.bit.SYNCBUSY) {
	}

	motor_attach_pins();

	TCC0->CTRLA.reg = TCC_CTRLA_SWRST;
	while (TCC0->CTRLA.bit.SWRST || TCC0->SYNCBUSY.bit.SWRST) {
	}

	/* Recovered verbatim, CPTEN bits included -- see board.h. */
	TCC0->CTRLA.reg = MOTOR_TCC_CTRLA_STOCK
	                  | TCC_CTRLA_PRESCALER(MOTOR_PWM_PRESCALER);
	TCC0->WAVE.reg  = TCC_WAVE_WAVEGEN_NPWM;
	tcc0_sync_all();

	TCC0->PER.reg = MOTOR_PWM_PERIOD;
	tcc0_sync_all();

	pair_a(0, 0);
	pair_b(0, 0);

	TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
	tcc0_sync_all();
}

/* The stock release, 0x8e28: hand all four pins back to the PORT, drive PA19
 * high, and mark the servo disabled. */
void motor_release(void)
{
	pair_a(0, 0);
	pair_b(0, 0);

	PORT->Group[0].PINCFG[MOTOR_PIN_WO0].bit.PMUXEN = 0;
	PORT->Group[0].PINCFG[MOTOR_PIN_WO1].bit.PMUXEN = 0;
	PORT->Group[0].PINCFG[MOTOR_PIN_WO2].bit.PMUXEN = 0;
	PORT->Group[0].PINCFG[MOTOR_PIN_WO3].bit.PMUXEN = 0;
	PORT->Group[0].OUTSET.reg = (1u << MOTOR_PIN_WO3);   /* PA19 high */
}

void motor_coast(void)
{
	motor_release();
}

/* Stop, but HOLD.
 *
 * WAVEGEN is NPWM with no inversion, so an output is high while the counter is
 * below its compare and low otherwise: compare 0 means permanently low.  With
 * all four compares at 0 and the pins still muxed to TCC0, both ends of each
 * coil sit at the same rail and the winding is shorted through the bridge --
 * dynamic braking.  Motion induces a current that opposes it.
 *
 * motor_release() is the opposite: it hands the pins back to the PORT, which
 * has no direction set for them, so the bridge inputs float and the mechanism
 * coasts.  That is what let a move overshoot its target by up to 1117 counts
 * (NOTES.md §57) -- the servo stopped at the target and the mechanism did not.
 *
 * Costs nothing at rest: there is no switching, and braking current only flows
 * while something is moving it. */
void motor_brake(void)
{
	motor_attach_pins();
	pair_a(0, 0);
	pair_b(0, 0);
}

void motor_drive(int16_t duty)
{
	uint16_t mag;

	if (duty > MOTOR_DUTY_MAX) {
		duty = MOTOR_DUTY_MAX;
	} else if (duty < -MOTOR_DUTY_MAX) {
		duty = -MOTOR_DUTY_MAX;
	}

	motor_attach_pins();

	if (duty >= 0) {
		mag = (uint16_t)duty;
		pair_a(mag, 0);          /* stock: u >= 1 -> A_plus, B_plus  */
		pair_b(mag, 0);
	} else {
		mag = (uint16_t)(-duty);
		pair_a(0, mag);          /* stock: u <  1 -> A_minus, B_minus */
		pair_b(0, mag);
	}
}

int32_t motor_seek_end(int8_t dir, uint16_t duty,
                       uint32_t min_run_ms, uint32_t stall_ms,
                       uint32_t timeout_ms)
{
	uint32_t start       = millis();
	uint32_t last_change = start;
	int32_t  first       = abs_encoder_position();
	int32_t  last_pos    = first;

	motor_drive(dir >= 0 ? (int16_t)duty : (int16_t)(-(int32_t)duty));

	for (;;) {
		uint32_t now = millis();
		int32_t  p   = abs_encoder_position();

		if (p != last_pos) {
			last_pos    = p;
			last_change = now;
		}
		if ((now - start) >= min_run_ms && (now - last_change) >= stall_ms) {
			break;                       /* position frozen -> at an end stop */
		}
		if ((now - start) >= timeout_ms) {
			break;                       /* safety net */
		}
	}

	motor_release();
	return last_pos - first;
}
