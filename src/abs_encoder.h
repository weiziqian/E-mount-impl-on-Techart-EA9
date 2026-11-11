#ifndef ABS_ENCODER_H
#define ABS_ENCODER_H
#include <stdint.h>

/* The 14-bit absolute magnetic encoder the stock firmware servos on
 * (SERCOM0 SPI, chip select PA07).  See board.h and NOTES.md §16. */

void     abs_encoder_init(void);

/* Raw 14-bit angle, 0..16383.  Wraps. */
uint16_t abs_encoder_raw(void);

/* Latch the current angle as the zero reference.  The stock firmware does this
 * once at start-up (0x8c80) before any move. */
void     abs_encoder_set_reference(void);

/* Position in the units the protocol uses: (wrapped delta >> 2) + 4096.
 * The travel clamps 4144..5632 are in these units. */
int32_t  abs_encoder_position(void);

/* Unwrapped position, in raw encoder counts, with no half-turn limit.
 *
 * abs_encoder_position() folds anything past half a magnet revolution to the
 * opposite sign -- the stock arithmetic, correct for focus travel, but it
 * reported -2026 for a pulse that visibly drove forward (NOTES.md §25).  This
 * keeps a running total instead: call it often enough that the mechanism moves
 * less than half a turn between calls.  At the fastest speed measured
 * (~40 counts/ms) that means anything under ~200 ms.
 *
 * Anything that servos must use this, not abs_encoder_position(). */
void    abs_encoder_track_reset(void);
int32_t abs_encoder_track(void);

/* Start the running total at `pos` instead of 0. */
void    abs_encoder_track_set(int32_t pos);

/* `to - from` between two raw readings, the short way round the 14-bit
 * circle: in [-8192, 8191]. */
int32_t abs_encoder_delta(uint16_t to, uint16_t from);

#endif
