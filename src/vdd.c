/* vdd.c -- measure the MCU's own supply with the ADC.
 *
 * Purpose (NOTES.md §17): when the motor was driven and nothing moved, two
 * explanations survived and were never separated -- the body current-limits the
 * lens, or the drive never reaches the motor.  They predict different supply
 * behaviour under load: sag versus none.
 *
 * WHAT THIS CAN AND CANNOT SEE.  The ADC's SCALEDIOVCC input is the part's own
 * IOVCC -- E-mount pin 5, the 3.15 V logic rail.  The motor runs from pin 2,
 * LENS_POWER, which is a separate contact.  If the body limits only the motor
 * rail, this will read flat and prove nothing about it; if the two share a
 * regulator, or the body limits the lens as a whole, it will sag.  So a sag is
 * strong evidence and a flat reading is weak evidence, not the reverse.
 *
 * The complementary signal costs nothing and is already collected: a brownout
 * shows up as BOD33 in the boot tally's reset causes.
 */
#include "board.h"
#include "clock.h"
#include "delay.h"
#include "vdd.h"

void vdd_init(void)
{
	uint32_t bias, lin;

	clock_enable_peripheral(GCLK_CLKCTRL_ID_ADC_Val, PM_APBCMASK_ADC);

	/* The factory calibration lives in the NVM software-calibration row;
	 * without it the ADC is specified to nothing useful. */
	bias = (*((uint32_t *)ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk)
	       >> ADC_FUSES_BIASCAL_Pos;
	lin  = (*((uint32_t *)ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk)
	       >> ADC_FUSES_LINEARITY_0_Pos;
	lin |= ((*((uint32_t *)ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk)
	        >> ADC_FUSES_LINEARITY_1_Pos) << 5;
	ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(lin);

	ADC->CTRLB.reg    = ADC_CTRLB_PRESCALER_DIV64 | ADC_CTRLB_RESSEL_12BIT;
	ADC->SAMPCTRL.reg = 63;        /* the 1/4 divider needs a long sample */
	ADC->REFCTRL.reg  = ADC_REFCTRL_REFSEL_INT1V;
	ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_1X
	                     | ADC_INPUTCTRL_MUXNEG_GND
	                     | ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC;
	while (ADC->STATUS.bit.SYNCBUSY) {
	}

	ADC->CTRLA.reg = ADC_CTRLA_ENABLE;
	while (ADC->STATUS.bit.SYNCBUSY) {
	}

	(void)vdd_read_mv();           /* the first conversion after enable is junk */
}

static uint16_t convert(void)
{
	ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
	ADC->SWTRIG.reg  = ADC_SWTRIG_START;
	while (!ADC->INTFLAG.bit.RESRDY) {
	}
	return ADC->RESULT.reg;
}

uint16_t vdd_read_mv(void)
{
	/* SCALEDIOVCC presents IOVCC/4 against the 1.0 V internal reference, so
	 * full scale is 4.0 V: mV = result * 4000 / 4095. */
	uint32_t acc = 0;
	unsigned i;

	for (i = 0; i < 8; i++) {
		acc += convert();
	}
	return (uint16_t)((acc / 8u) * 4000u / 4095u);
}

void vdd_read_burst(uint16_t *lo, uint16_t *hi, uint16_t ms)
{
	uint32_t start = millis();
	uint16_t l = 0xFFFF, h = 0;

	do {
		uint16_t v = vdd_read_mv();

		if (v < l) {
			l = v;
		}
		if (v > h) {
			h = v;
		}
	} while ((uint32_t)(millis() - start) < ms);

	*lo = l;
	*hi = h;
}

static uint16_t g_track_lo = 0xFFFF;
static uint16_t g_track_hi;

void vdd_track_reset(void)
{
	g_track_lo = 0xFFFF;
	g_track_hi = 0;
}

void vdd_track_sample(void)
{
	uint16_t v = (uint16_t)((uint32_t)convert() * 4000u / 4095u);

	if (v < g_track_lo) {
		g_track_lo = v;
	}
	if (v > g_track_hi) {
		g_track_hi = v;
	}
}

void vdd_track_result(uint16_t *lo, uint16_t *hi)
{
	*lo = (g_track_lo == 0xFFFF) ? 0 : g_track_lo;
	*hi = g_track_hi;
}
