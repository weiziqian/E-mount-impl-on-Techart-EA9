/* em_eic.h -- the two body control lines on the EIC. */
#ifndef EM_EIC_H
#define EM_EIC_H

#include <stdint.h>

/* cs_edge is called on both edges of PA02 with the new level; vd_edge on the
 * rising edge of PA03.  Both run in EIC_Handler at NVIC priority 1. */
void em_eic_init(void (*cs_edge)(int level), void (*vd_edge)(void));

/* The level of one port-A pin, driven value if it is an output and sensed
 * value if it is an input -- the idiom the stock firmware uses at 0x53b4. */
int  em_pin_level(uint8_t pin);

#endif /* EM_EIC_H */
