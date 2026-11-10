/* em_eic.h -- the two body control lines on the EIC. */
#ifndef EM_EIC_H
#define EM_EIC_H

#include <stdint.h>

/* cs_edge is called on both edges of PA02 with the new level; vd_edge on the
 * rising edge of PA03.  Both run in EIC_Handler at NVIC priority 1. */
void em_eic_init(void (*cs_edge)(int level), void (*vd_edge)(void));

/* Hold both lines' callbacks off, and let them back on with any edge that
 * arrived meanwhile discarded.  For the standby wait, which polls the chip
 * select itself and must not have the frame path parse its edges. */
void em_eic_pause(void);
void em_eic_resume(void);

/* STANDBY until the body's chip select reads high -- the CPU and the 48 MHz
 * clock stopped, woken by the EIC.  Call only while paused, with the line
 * already seen low; returns still paused.  millis() does not advance while
 * asleep (SysTick stops with the CPU clock). */
void em_eic_sleep_until_cs_high(void);

/* The level of one port-A pin, driven value if it is an output and sensed
 * value if it is an input -- the idiom the stock firmware uses at 0x53b4. */
int  em_pin_level(uint8_t pin);

#endif /* EM_EIC_H */
