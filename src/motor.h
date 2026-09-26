#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

/* The LM-EA9's actuator, as the stock firmware drives it (NOTES.md §15):
 * TCC0 produces four PWM outputs wired as two H-bridges, and the firmware feeds
 * BOTH bridges the same signed value -- so they are paralleled onto one dc
 * motor.  Sign selects direction, magnitude is the duty. */

#define MOTOR_DUTY_MAX   2560        /* = PER; 100% duty */

void    motor_init(void);
void    motor_drive(int16_t duty);   /* [-MOTOR_DUTY_MAX, +MOTOR_DUTY_MAX] */
void    motor_release(void);         /* stock 0x8e28: un-mux all four pins */
void    motor_attach_pins(void);     /* route PA08/09/18/19 to TCC0        */
void    motor_coast(void);           /* alias for motor_release()          */
void    motor_brake(void);           /* stop and HOLD: coils shorted        */

/* Drive in `dir` until the ABSOLUTE encoder position stops changing for
 * `stall_ms`, i.e. the mechanism has reached an end stop, then release.
 * Returns the signed distance travelled, in the same units as the protocol's
 * travel clamps (4144..5632).
 *
 * `min_run_ms` is a grace period before stall detection arms, so the motor's
 * own start-up delay is not mistaken for a stall.  `timeout_ms` bounds the whole
 * call: if the encoder is miswired and emits noise, this is what stops it. */
int32_t motor_seek_end(int8_t dir, uint16_t duty,
                       uint32_t min_run_ms, uint32_t stall_ms,
                       uint32_t timeout_ms);
#endif
