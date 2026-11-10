/* em_eic.c -- EXTINT[2] and EXTINT[3], the body's two control lines.
 *
 * Config recovered from 0x7564 (NOTES.md §13): CONFIG[0] byte 1 = 0x9b and
 * WAKEUP = 4.  That is EXTINT2 = FILTEN | SENSE_BOTH and EXTINT3 = FILTEN |
 * SENSE_RISE.  The asymmetry is the tell that these are not a quadrature pair.
 */
#include "board.h"
#include "em_eic.h"

#define EIC_CONFIG_BOTH_FILT  0xbu    /* FILTEN | SENSE = BOTH */
#define EIC_CONFIG_RISE_FILT  0x9u    /* FILTEN | SENSE = RISE */
#define EIC_CONFIG_HIGH       0x4u    /* SENSE = HIGH, no filter */

static void (*g_cs_edge)(int level);
static void (*g_vd_edge)(void);
static volatile uint8_t g_sleeping;   /* EIC_Handler only wakes the CPU */

int em_pin_level(uint8_t pin)
{
	/* For an output the meaningful level is what we drive (OUT); for an
	 * input it is what we sense (IN).  DIR selects between them without a
	 * branch: (DIR & (OUT ^ IN)) ^ IN. */
	uint32_t in  = PORT->Group[0].IN.reg;
	uint32_t out = PORT->Group[0].OUT.reg;
	uint32_t dir = PORT->Group[0].DIR.reg;

	return (int)((((dir & (out ^ in)) ^ in) >> pin) & 1u);
}

void em_eic_init(void (*cs_edge)(int level), void (*vd_edge)(void))
{
	g_cs_edge = cs_edge;
	g_vd_edge = vd_edge;

	PM->APBAMASK.reg |= PM_APBAMASK_EIC;
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC_Val)
	                    | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
	while (GCLK->STATUS.bit.SYNCBUSY) {
	}

	PORT->Group[0].PINCFG[EM_PIN_BODY_CS].reg = PORT_PINCFG_PMUXEN | PORT_PINCFG_INEN;
	PORT->Group[0].PINCFG[EM_PIN_BODY_VD].reg = PORT_PINCFG_PMUXEN | PORT_PINCFG_INEN;
	PORT->Group[0].PMUX[EM_PIN_BODY_CS / 2].reg =
	        PORT_PMUX_PMUXE(MUX_PA02A_EIC_EXTINT2) | PORT_PMUX_PMUXO(MUX_PA03A_EIC_EXTINT3);

	EIC->CTRL.reg = EIC_CTRL_SWRST;
	while (EIC->CTRL.bit.SWRST || EIC->STATUS.bit.SYNCBUSY) {
	}

	EIC->CONFIG[0].reg = (EIC_CONFIG_BOTH_FILT << (4 * EM_EXTINT_BODY_CS))
	                     | (EIC_CONFIG_RISE_FILT << (4 * EM_EXTINT_BODY_VD));
	EIC->WAKEUP.reg = (1u << EM_EXTINT_BODY_CS);

	EIC->INTFLAG.reg  = (1u << EM_EXTINT_BODY_CS) | (1u << EM_EXTINT_BODY_VD);
	EIC->INTENSET.reg = (1u << EM_EXTINT_BODY_CS) | (1u << EM_EXTINT_BODY_VD);

	EIC->CTRL.reg = EIC_CTRL_ENABLE;
	while (EIC->STATUS.bit.SYNCBUSY) {
	}

	NVIC_SetPriority(EIC_IRQn, 1);     /* stock: NVIC IPR1 byte for IRQ4 = 0x40 */
	NVIC_EnableIRQ(EIC_IRQn);
}

void em_eic_pause(void)
{
	NVIC_DisableIRQ(EIC_IRQn);
}

void em_eic_resume(void)
{
	EIC->INTFLAG.reg = (1u << EM_EXTINT_BODY_CS) | (1u << EM_EXTINT_BODY_VD);
	NVIC_ClearPendingIRQ(EIC_IRQn);
	NVIC_EnableIRQ(EIC_IRQn);
}

static void eic_set_config(uint32_t config)
{
	EIC->CTRL.reg = 0;
	while (EIC->STATUS.bit.SYNCBUSY) {
	}
	EIC->CONFIG[0].reg = config;
	EIC->CTRL.reg = EIC_CTRL_ENABLE;
	while (EIC->STATUS.bit.SYNCBUSY) {
	}
}

void em_eic_sleep_until_cs_high(void)
{
	const uint32_t cs     = 1u << EM_EXTINT_BODY_CS;
	const uint32_t vd     = 1u << EM_EXTINT_BODY_VD;
	const uint32_t shift  = 4u * EM_EXTINT_BODY_CS;
	uint32_t       config = EIC->CONFIG[0].reg;
	uint32_t       tick   = SysTick->CTRL;

	/* SAMD21 errata: with the default SLEEPPRM the part can fail to wake
	 * from standby.  DISABLED keeps the flash powered in sleep, which costs
	 * a few uA and nothing else. */
	NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;

	/* In STANDBY, GCLK0 -- and with it the EIC's clock -- is stopped, and
	 * an EIC without its clock can only wake the part on a LEVEL, unfiltered.
	 * The caller has already seen the line low, so "high" is the rising edge. */
	EIC->INTENCLR.reg = cs | vd;
	eic_set_config((config & ~(0xFu << shift)) | (EIC_CONFIG_HIGH << shift));
	g_sleeping = 1;
	NVIC_ClearPendingIRQ(EIC_IRQn);
	NVIC_EnableIRQ(EIC_IRQn);

	/* The 1 ms tick would wake the part every millisecond. */
	SysTick->CTRL = tick & ~SysTick_CTRL_TICKINT_Msk;
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

	/* PRIMASK across the check and the WFI: a rise between the two leaves
	 * the interrupt pending, and a pending interrupt ends WFI even masked.
	 * Unmasked, the handler could run first, clear the enable, and leave
	 * the WFI with nothing to wake it. */
	for (;;) {
		__disable_irq();
		if (PORT->Group[0].IN.reg & (1u << EM_PIN_BODY_CS)) {
			__enable_irq();
			break;
		}
		EIC->INTFLAG.reg  = cs;
		EIC->INTENSET.reg = cs;
		__DSB();
		__WFI();
		__enable_irq();          /* the handler runs here */
	}

	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	SysTick->CTRL = tick;
	g_sleeping = 0;

	/* Back to the paused state the caller is in: callbacks off, the
	 * session's edge config restored. */
	NVIC_DisableIRQ(EIC_IRQn);
	EIC->INTENCLR.reg = cs | vd;
	eic_set_config(config);
	EIC->INTFLAG.reg  = cs | vd;
	EIC->INTENSET.reg = cs | vd;
}

void EIC_Handler(void)
{
	uint32_t flags = EIC->INTFLAG.reg;

	/* Asleep: the interrupt's only job was to end the WFI.  A level
	 * interrupt keeps firing while the line is high, so turn it off. */
	if (g_sleeping) {
		EIC->INTENCLR.reg = 1u << EM_EXTINT_BODY_CS;
		EIC->INTFLAG.reg  = flags;
		return;
	}

	if (flags & (1u << EM_EXTINT_BODY_CS)) {
		EIC->INTFLAG.reg = (1u << EM_EXTINT_BODY_CS);
		if (g_cs_edge) {
			/* SENSE_BOTH gives no direction, so read the pin.  The
			 * 0.5 us filter means the level has settled by now. */
			g_cs_edge(em_pin_level(EM_PIN_BODY_CS));
		}
	}
	if (flags & (1u << EM_EXTINT_BODY_VD)) {
		EIC->INTFLAG.reg = (1u << EM_EXTINT_BODY_VD);
		if (g_vd_edge) {
			g_vd_edge();
		}
	}
}
