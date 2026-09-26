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

static void (*g_cs_edge)(int level);
static void (*g_vd_edge)(void);

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

void EIC_Handler(void)
{
	uint32_t flags = EIC->INTFLAG.reg;

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
