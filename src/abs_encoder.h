#ifndef ABS_ENCODER_H
#define ABS_ENCODER_H
#include <stdint.h>

/* The 14-bit absolute magnetic encoder the stock firmware servos on
 * (SERCOM0 SPI, chip select PA07).  See board.h and NOTES.md §16. */

void     abs_encoder_init(void);

/* Raw 14-bit angle, 0..16383.  Wraps. */
uint16_t abs_encoder_raw(void);

/* Unwrapped position, in raw encoder counts, with no half-turn limit.
 *
 * The stock's position (0x8cc0: the wrapped delta from a reference latched at
 * start-up) folds anything past half a magnet revolution to the opposite sign,
 * and reported -2026 for a pulse that visibly drove forward (NOTES.md §25).
 * This keeps a running total instead: call it often enough that the mechanism
 * moves less than half a turn between calls.  At the fastest speed measured
 * (~40 counts/ms) that means anything under ~200 ms.
 *
 * abs_encoder_track_reset() makes the current position 0; the boot reset calls
 * it before its first reading, and then sets the stop it finds as 0. */
void    abs_encoder_track_reset(void);
int32_t abs_encoder_track(void);

/* Start the running total at `pos` instead of 0. */
void    abs_encoder_track_set(int32_t pos);

/* `to - from` between two raw readings, the short way round the 14-bit
 * circle: in [-8192, 8191]. */
int32_t abs_encoder_delta(uint16_t to, uint16_t from);

#endif
