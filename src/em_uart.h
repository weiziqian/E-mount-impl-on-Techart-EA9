/* em_uart.h -- SERCOM1, the E-mount bus. */
#ifndef EM_UART_H
#define EM_UART_H

#include <stdint.h>

#define EM_RX_BUF_LEN 256

/* Written over the head of the buffer at the start of each receive window, so
 * a captured byte that still reads this was never delivered by the body. */
#define EM_RX_POISON      0xA5u
#define EM_RX_POISON_LEN  64u

void     em_uart_init(void);

/* Throw away anything buffered and start a new frame.  Called on the rising
 * edge of the body's chip select. */
void     em_uart_rx_reset(void);

/* How many bytes have landed since the last reset, and the buffer itself.
 * Buffer offset equals frame offset: rx[0] is the 0xF0 (EA9.md §2.1). */
uint16_t em_uart_rx_len(void);
const uint8_t *em_uart_rx_buf(void);

/* Blocking: shifts every byte out and returns once the last one has left the
 * shift register, so the caller can time the chip-select release from it. */
void     em_uart_send(const uint8_t *buf, uint16_t len);

/* Take the bus down, as the stock does in its shutdown tail.
 *
 * After this em_uart_send() returns immediately instead of transmitting.  That
 * is not tidiness: it waits on the DRE flag, which a disabled SERCOM never
 * sets, so a single send after disabling would hang the firmware forever. */
void     em_uart_disable(void);

/* Raw receiver counters, below the frame layer.
 *
 * "rx frames = 0" cannot distinguish a body that stopped pulsing its chip
 * select from one that pulsed but sent nothing, from one whose bytes we could
 * not frame.  These can: bytes counts everything the SERCOM handed us, and
 * status accumulates its error bits (framing, parity, overflow). */
extern volatile uint32_t em_rx_bytes;
extern volatile uint32_t em_rx_errors;
extern volatile uint16_t em_rx_status;   /* accumulated STATUS bits */
extern volatile uint32_t em_t_first_byte; /* ms the very first byte arrived */

#endif /* EM_UART_H */
