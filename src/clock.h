#ifndef CLOCK_H
#define CLOCK_H
#include <stdint.h>
void clock_init(void);
void clock_enable_peripheral(uint8_t gclk_id, uint32_t apbc_mask);
#endif
