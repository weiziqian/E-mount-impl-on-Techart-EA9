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

#endif
