/* vdd.h -- the part's own supply rail, in millivolts. */
#ifndef VDD_H
#define VDD_H
#include <stdint.h>

void     vdd_init(void);
uint16_t vdd_read_mv(void);                               /* mean of 8 */
void     vdd_read_burst(uint16_t *lo, uint16_t *hi, uint16_t ms);  /* min/max */

/* Running min/max, for sampling from inside someone else's loop.  The previous
 * motor build measured "under load" AFTER the servo returned and the coils were
 * already released, so every load figure it reported was an idle reading. */
void     vdd_track_reset(void);
void     vdd_track_sample(void);
void     vdd_track_result(uint16_t *lo, uint16_t *hi);

#endif
