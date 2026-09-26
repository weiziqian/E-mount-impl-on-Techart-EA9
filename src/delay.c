/* delay.c -- millisecond delay and a free-running millisecond tick on SysTick.
 *
 * The stock firmware uses ASF4's hal_delay on SysTick (0x64ac calls
 * delay_init(SysTick), NOTES.md §13).  This is the same idea written directly,
 * so the building blocks have no ASF4 dependency.
 */
#include "board.h"
#include "delay.h"

static volatile uint32_t g_millis;

void delay_init(void)
{
	SysTick->LOAD = (F_CPU / 1000u) - 1u;
	SysTick->VAL  = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
	                | SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void)
{
	g_millis++;
}

uint32_t millis(void)
{
	return g_millis;
}

void delay_ms(uint32_t ms)
{
	uint32_t start = g_millis;
	while ((uint32_t)(g_millis - start) < ms) {
	}
}

/* Microsecond delay, counted in SysTick cycles rather than in a calibrated
 * loop so it survives -O changes.  The protocol gaps it has to reproduce are
 * 10 us, 100 us and 300 us (emount.c), all far shorter than the 1 ms reload,
 * but the wrap is handled anyway so a long call cannot silently return early.
 */
void delay_us(uint32_t us)
{
	uint32_t target  = us * (F_CPU / 1000000u);
	uint32_t elapsed = 0;
	uint32_t last    = SysTick->VAL;

	while (elapsed < target) {
		uint32_t now = SysTick->VAL;

		/* SysTick counts DOWN, so a normal step is last - now. */
		elapsed += (last >= now) ? (last - now)
		                         : (last + SysTick->LOAD + 1u - now);
		last = now;
	}
}
