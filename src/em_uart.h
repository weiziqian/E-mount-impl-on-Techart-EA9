/* em_uart.h -- SERCOM1, the E-mount bus. */
#ifndef EM_UART_H
#define EM_UART_H

#include <stdint.h>

#define EM_RX_BUF_LEN 256          /* a power of 2: the ring masks with it */

void     em_uart_init(void);

/* THE RECEIVE RING.  Bytes in arrival order, with no idea of windows or
 * frames -- emount.c finds the frames.  One reader: the chip-select interrupt
 * (and em_init before it is armed).
 *
 * count and flush read the DMA's position, which briefly suspends it: call
 * them only while the body's chip select is LOW, when it sends nothing.
 *
 *   em_uart_rx_count    bytes received and not yet consumed
 *   em_uart_rx_at(i)    the i-th of them, 0 = the oldest
 *   em_uart_rx_consume  drop the oldest n
 *   em_uart_rx_flush    drop everything received so far */
uint16_t em_uart_rx_count(void);
uint8_t  em_uart_rx_at(uint16_t i);
void     em_uart_rx_consume(uint16_t n);
void     em_uart_rx_flush(void);

/* Blocking: shifts every byte out and returns once the last one has left the
 * shift register, so the caller can time the chip-select release from it. */
void     em_uart_send(const uint8_t *buf, uint16_t len);

/* Change the bus speed (a BAUD register value, EM_BAUD / EM_BAUD_FAST).
 * Call only after em_uart_send() has returned, so the last frame went out
 * whole at the old speed.  No-op once the bus is disabled. */
void     em_uart_set_baud(uint16_t baud);

/* Take the bus down, as the stock does in its shutdown tail.
 *
 * After this em_uart_send() returns immediately instead of transmitting.  That
 * is not tidiness: it waits on the DRE flag, which a disabled SERCOM never
 * sets, so a single send after disabling would hang the firmware forever. */
void     em_uart_disable(void);

/* Bring the bus back after em_uart_disable(), at the speed it had: BAUD is
 * not touched, so a bus that had switched to 1.5 Mbaud comes back at 1.5.
 * PA00 goes back to the SERCOM (idling high) and the receiver starts empty.
 * Waking from the body's standby needs exactly this -- the body keeps the
 * negotiated speed and does not send 0x0C again (emount.c, em_sleep_until_wake). */
void     em_uart_resume(void);

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
extern volatile uint32_t em_rx_dma_stuck; /* DMA suspends never acknowledged */

#endif /* EM_UART_H */
