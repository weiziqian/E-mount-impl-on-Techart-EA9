/* board.h -- the LM-EA9 board, as recovered from firmware 1.8.0.
 *
 * Every constant here is traceable to config/recovered_config.h and NOTES.md §13.
 * Where a value could NOT be recovered it is marked CHOSEN and explained.
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include "sam.h"

/* ---- clock ---------------------------------------------------------------
 * GCLK0 = DFLL48M, open loop, factory COARSE calibration.  48 MHz feeds
 * SERCOM1, TCC0, TCC1, TC4 and the CPU. */
#define F_CPU                   48000000u

/* ---- motor: TCC0, four outputs, two H-bridges, ONE dc motor --------------
 * PA08 = WO[0] and PA09 = WO[1] are one bridge (call it coil A);
 * PA18 = WO[2] and PA19 = WO[3] are the other (coil B).
 * The stock firmware drives BOTH bridges with the SAME signed value, so the
 * two bridges are paralleled onto one motor: sign = direction, magnitude =
 * duty.  See NOTES.md §15. */
#define MOTOR_PIN_WO0           8      /* PA08, mux E */
#define MOTOR_PIN_WO1           9      /* PA09, mux E */
#define MOTOR_PIN_WO2           18     /* PA18, mux F */
#define MOTOR_PIN_WO3           19     /* PA19, mux F */
#define MOTOR_MUX_WO01          MUX_PA08E_TCC0_WO0
#define MOTOR_MUX_WO23          MUX_PA18F_TCC0_WO2

/* Both recovered.  PER = 2560 from pwm_set_parameters(PWM_0, 0xa00, 0) at
 * 0x904c; PRESCALER from the TCC config table (.data 0x200003ec, initialiser at
 * flash 0x9dc0): TCC0 CTRLA = 0x03000000, so PRESCALER = 0 = DIV1.
 * 48e6 / 2560 = 18.75 kHz. */
#define MOTOR_PWM_PERIOD        2560u
#define MOTOR_PWM_PRESCALER     TCC_CTRLA_PRESCALER_DIV1_Val
#define MOTOR_PWM_HZ            (F_CPU / MOTOR_PWM_PERIOD)

/* The same recovered CTRLA also sets CPTEN0 and CPTEN1, which makes CC0/CC1
 * CAPTURE channels -- yet 0x9070 writes duty into them as if they were compare
 * channels.  That is unexplained (NOTES.md §16).  Replicated verbatim here so
 * first-run behaviour matches the stock firmware; do not "fix" it before the
 * hardware has been observed, because clearing it could energise outputs the
 * stock firmware leaves idle. */
#define MOTOR_TCC_CTRLA_STOCK   0x03000000u

/* ---- the E-mount bus ----------------------------------------------------
 * SERCOM1 on PA00/PA01, plus three control lines.  CORRECTION: an earlier
 * version of this file called PA02/PA03 motor position feedback.  They are
 * not -- protocol_init (0x551c) polls PA02 before anything has moved, and the
 * PA03 callback (0x5120) latches the focus position into message 0x06.  Real
 * position feedback is the SPI encoder below.
 *
 *   PA02  EXTINT[2], BOTH edges  the body's transfer window.  High while the
 *                                body clocks a frame at us: the rising edge
 *                                arms the receiver, the falling edge means a
 *                                frame has arrived (0x53a4).
 *   PA03  EXTINT[3], RISE        the body's frame sync, ~60 Hz.  Latch the
 *                                focus position and arm the status pair.
 *   PA10  output                 our own chip select.  transmit() raises it
 *                                (0x51c8); the end-of-frame routine drops it
 *                                100 us after the last byte (0x5188/0x5280).
 *
 * EIC config recovered from 0x7564: EIC->CONFIG[0] byte 1 = 0x9b -- EXTINT2
 * nibble 0xb = FILTEN | SENSE_BOTH, EXTINT3 nibble 0x9 = FILTEN | SENSE_RISE --
 * and EIC->WAKEUP = 4, so only EXTINT2 can wake the part.
 */
#define EM_PIN_TX               0      /* PA00, mux D = SERCOM1/PAD[0] */
#define EM_PIN_RX               1      /* PA01, mux D = SERCOM1/PAD[1] */
#define EM_MUX_UART             MUX_PA00D_SERCOM1_PAD0
#define EM_PIN_BODY_CS          2      /* PA02, mux A = EIC/EXTINT[2] */
#define EM_PIN_BODY_VD          3      /* PA03, mux A = EIC/EXTINT[3] */
#define EM_EXTINT_BODY_CS       2
#define EM_EXTINT_BODY_VD       3
#define EM_PIN_LENS_CS          10     /* PA10, plain GPIO output */

/* SERCOM1 USART, from _usarts[0] at flash 0x9920 (NOTES.md §13).
 *   CTRLA: MODE=1 internal clock, TXPO=0 (PAD0), RXPO=1 (PAD1),
 *          SAMPR=0 (16x arithmetic), DORD=1 (LSB first)
 *   CTRLB: TXEN|RXEN, CHSIZE=0 (8 bit), SBMODE=0 (1 stop), no parity
 *   BAUD : 16x arithmetic, 48e6/16 * (1 - 0xc000/65536) = 750000 exactly
 * The baud rate is written once and never changed: the adapter has no code
 * path to the 1.5 Mbaud native lenses negotiate (NOTES.md §13). */
#define EM_SERCOM_ADDR          0x42000C00u   /* SERCOM1 */
#define EM_CTRLA                0x40100004u
#define EM_CTRLB                0x00030000u
#define EM_BAUD                 0xC000u

/* ---- absolute position: a 14-bit magnetic encoder on SERCOM0 SPI ---------
 * This, not the EIC, is what the stock firmware servos on.  0x89a4 drops a
 * chip select, clocks two bytes, raises it again and returns the top 14 bits;
 * 0x8cc0 turns that into the position the protocol reports.
 *
 * SPI config recovered from _spis[] (flash 0x995c):
 *   CTRLA 0x0020000c -> master, DOPO=0 (DO=PAD0, SCK=PAD1), DIPO=2 (DI=PAD2),
 *                       CPOL=0 CPHA=0 (mode 0), MSB first
 *   CTRLB 0x00020000 -> RXEN, 8-bit
 *   BAUD  23         -> 48e6 / (2*(23+1)) = 1.0 MHz
 */
#define ENC_SPI_SERCOM_ADDR     0x42000800u   /* SERCOM0 */
#define ENC_SPI_CTRLA           0x0020000cu
#define ENC_SPI_CTRLB           0x00020000u
#define ENC_SPI_BAUD            23u
#define ENC_SPI_PIN_MOSI        4             /* PA04, mux D = SERCOM0/PAD[0] */
#define ENC_SPI_PIN_SCK         5             /* PA05, mux D = SERCOM0/PAD[1] */
#define ENC_SPI_PIN_MISO        6             /* PA06, mux D = SERCOM0/PAD[2] */
#define ENC_SPI_MUX             MUX_PA04D_SERCOM0_PAD0

/* Two chip selects exist.  The stock firmware only ever asserts CS1: both call
 * sites of 0x89a4 pass 1.  CS2 is wired but unused by this firmware. */
#define ENC_CS1_PIN             7             /* PA07, idle high */
#define ENC_CS2_PIN             11            /* PA11, idle high, UNUSED */

/* Position scale, from 0x8cc0:  position = (wrapped_delta >> 2) + 0x1000.
 * The travel clamps the protocol reports (4144 .. 5632, EA9.md §4.10) live in
 * exactly these units, so full travel is (5632-4144)*4 = 5952 encoder counts. */
#define ENC_POS_OFFSET          4096
#define ENC_COUNTS              16384         /* 14-bit wrap */
#define ENC_TRAVEL_LO           4144
#define ENC_TRAVEL_HI           5632

/* Reproduce the GPIO state atmel_start_init (0x65d8) leaves behind. */
void board_init_pins(void);

#endif /* BOARD_H */
