/* board.c -- the GPIO state atmel_start_init leaves behind.
 *
 * Read straight off 0x65d8 rather than inferred.  Each pin is configured with
 * its OUT level written BEFORE DIRSET, so it never spends a cycle high-Z at
 * the wrong level -- which matters for PA10, the chip select the body watches.
 *
 *   PA07  output HIGH   encoder chip select, idle high
 *   PA10  output LOW    our chip select to the body; em_init raises it
 *   PA11  output HIGH   purpose not established, replicated
 *   PA16  output LOW    raised by main a moment later, purpose not established
 *   PA23  output LOW    the bootloader's stay-in-bootloader input (EA9.md 11.4)
 *
 * All five also get PMUXEN cleared, which is what the WRCONFIG pairs in the
 * stock routine are doing.
 */
#include "board.h"

void board_init_pins(void)
{
	static const struct {
		uint8_t pin;
		uint8_t high;
	} pins[] = {
		{ 7, 1 }, { 10, 0 }, { 11, 1 }, { 16, 0 }, { 23, 0 },
	};

	for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
		uint32_t mask = 1u << pins[i].pin;

		PORT->Group[0].PINCFG[pins[i].pin].reg &= ~(uint8_t)PORT_PINCFG_PMUXEN;
		if (pins[i].high) {
			PORT->Group[0].OUTSET.reg = mask;
		} else {
			PORT->Group[0].OUTCLR.reg = mask;
		}
		PORT->Group[0].DIRSET.reg = mask;
	}
}
