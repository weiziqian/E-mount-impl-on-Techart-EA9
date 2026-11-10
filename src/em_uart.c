/* em_uart.c -- SERCOM1, 8N1, LSB first: 750 kbaud from reset, 1.5 Mbaud once
 * message 0x0C has been answered (em_uart_set_baud).
 *
 * RECEIVE: a 256-byte ring, read through em_uart_rx_count / _at / _consume.
 * The ring knows nothing about windows or frames; emount.c finds the frames in
 * it by their 0xF0, length, checksum and 0x55.
 *
 * DMAC channel 0, triggered by the SERCOM's RX, writes every byte into the
 * ring with no CPU at all, endlessly (the descriptor links to itself).  The
 * stock does the same (a DMAC channel into a 255-byte buffer at 0x20000694).
 * A CPU that is stalled -- a flash write, a masked section -- loses nothing.
 * With an RX interrupt doing the moving it did: the SERCOM's own buffer is two
 * bytes deep, and a 2.5 ms stall dropped bytes from the body's first request
 * (dump_fail_start.bin).
 *
 * WHERE THE DMA HAS GOT TO.  The SAMD21 writes a channel's remaining count back
 * to RAM only when the channel stops, so reading the position means suspending
 * the channel, reading, resuming.  Done ONLY WHILE THE BODY'S CHIP SELECT IS
 * LOW (the callers' contract): the body sends nothing then, so nothing can
 * arrive during the suspend however long it takes.  While the line is high
 * the channel is never touched.
 *
 * TRANSMIT is polled and blocking, as before: the longest frame is message
 * 0x08 at 210 bytes = 2.8 ms, sent once, and returning on TXC makes the
 * chip-select release trivially correct.
 */
#include "board.h"
#include "clock.h"
#include "delay.h"
#include "em_uart.h"

#define EMSERCOM ((Sercom *)EM_SERCOM_ADDR)
#define RX_MASK  (EM_RX_BUF_LEN - 1u)       /* EM_RX_BUF_LEN is a power of 2 */

static volatile uint8_t  g_rx[EM_RX_BUF_LEN];
static volatile uint16_t g_wr;      /* ring index of the next byte to arrive */
static uint16_t          g_rd;      /* ring index of the oldest unread byte */
static volatile uint8_t  g_disabled;

volatile uint32_t em_rx_bytes;
volatile uint32_t em_rx_errors;
volatile uint16_t em_rx_status;
volatile uint32_t em_t_first_byte;

static void em_uart_sync(void)
{
	while (EMSERCOM->USART.SYNCBUSY.reg) {
	}
}

/* 128-bit aligned, as the DMAC requires; channel 0 only. */
static DmacDescriptor g_desc __attribute__((aligned(16)));
static DmacDescriptor g_wrb  __attribute__((aligned(16)));
static uint16_t       g_dma_seen;   /* g_wr at the last read, for em_rx_bytes */
volatile uint32_t     em_rx_dma_stuck;

static void dma_init(void)
{
	PM->AHBMASK.reg |= PM_AHBMASK_DMAC;
	PM->APBBMASK.reg |= PM_APBBMASK_DMAC;

	DMAC->CTRL.reg = 0;
	DMAC->CTRL.reg = DMAC_CTRL_SWRST;
	while (DMAC->CTRL.reg & DMAC_CTRL_SWRST) {
	}
	DMAC->BASEADDR.reg = (uint32_t)&g_desc;
	DMAC->WRBADDR.reg  = (uint32_t)&g_wrb;

	/* Incrementing destination: DSTADDR is the address AFTER the last beat. */
	g_desc.BTCTRL.reg   = DMAC_BTCTRL_VALID | DMAC_BTCTRL_BEATSIZE_BYTE
	                      | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_BLOCKACT_NOACT;
	g_desc.BTCNT.reg    = EM_RX_BUF_LEN;
	g_desc.SRCADDR.reg  = (uint32_t)&EMSERCOM->USART.DATA.reg;
	g_desc.DSTADDR.reg  = (uint32_t)g_rx + EM_RX_BUF_LEN;
	g_desc.DESCADDR.reg = (uint32_t)&g_desc;           /* round and round */
	g_wrb.BTCNT.reg     = EM_RX_BUF_LEN;

	DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xF);

	DMAC->CHID.reg = 0;
	DMAC->CHCTRLA.reg = 0;
	DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
	while (DMAC->CHCTRLA.reg & DMAC_CHCTRLA_SWRST) {
	}
	DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0)
	                    | DMAC_CHCTRLB_TRIGSRC(SERCOM1_DMAC_ID_RX)
	                    | DMAC_CHCTRLB_TRIGACT_BEAT;
	DMAC->CHCTRLA.reg = DMAC_CHCTRLA_ENABLE;
}

/* Where the next byte will land, from the channel's remaining count. */
static uint16_t dma_wr(void)
{
	uint32_t t;
	uint16_t left;

	__disable_irq();
	DMAC->CHID.reg = 0;
	DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_SUSP;
	DMAC->CHCTRLB.reg = (DMAC->CHCTRLB.reg & ~DMAC_CHCTRLB_CMD_Msk)
	                    | DMAC_CHCTRLB_CMD_SUSPEND;
	for (t = 0; t < 1000u && !(DMAC->CHINTFLAG.reg & DMAC_CHINTFLAG_SUSP); t++) {
	}
	if (t == 1000u) {
		em_rx_dma_stuck++;
	} else {
		left = g_wrb.BTCNT.reg;
		g_wr = (uint16_t)((EM_RX_BUF_LEN - left) & RX_MASK);
	}
	DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_SUSP;
	DMAC->CHCTRLB.reg = (DMAC->CHCTRLB.reg & ~DMAC_CHCTRLB_CMD_Msk)
	                    | DMAC_CHCTRLB_CMD_RESUME;
	__enable_irq();

	if (g_wr != g_dma_seen) {
		em_rx_bytes += (uint16_t)((g_wr - g_dma_seen) & RX_MASK);
		g_dma_seen = g_wr;
		if (!em_t_first_byte) {
			em_t_first_byte = millis();
		}
	}
	return g_wr;
}

void em_uart_init(void)
{
	clock_enable_peripheral(GCLK_CLKCTRL_ID_SERCOM1_CORE_Val, PM_APBCMASK_SERCOM1);

	/* PA00 = PAD[0] = TX, PA01 = PAD[1] = RX, both mux D.  PA01 needs its
	 * input buffer enabled or the receiver sees a constant high. */
	PORT->Group[0].PINCFG[EM_PIN_TX].reg = PORT_PINCFG_PMUXEN;
	/* PULL-UP on RX.  A UART idle line is high, and the receiver is
	 * enabled before the body starts driving PA01 (emount.c), so without a
	 * pull the line floats and the SERCOM manufactures start bits out of
	 * noise. */
	PORT->Group[0].OUTSET.reg = (1u << EM_PIN_RX);
	PORT->Group[0].PINCFG[EM_PIN_RX].reg = PORT_PINCFG_PMUXEN | PORT_PINCFG_INEN
	                                       | PORT_PINCFG_PULLEN;
	PORT->Group[0].PMUX[EM_PIN_TX / 2].reg =
	        PORT_PMUX_PMUXE(EM_MUX_UART) | PORT_PMUX_PMUXO(EM_MUX_UART);

	EMSERCOM->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
	while (EMSERCOM->USART.CTRLA.bit.SWRST || EMSERCOM->USART.SYNCBUSY.bit.SWRST) {
	}

	EMSERCOM->USART.CTRLA.reg = EM_CTRLA;      /* recovered verbatim */
	EMSERCOM->USART.CTRLB.reg = EM_CTRLB;
	em_uart_sync();
	EMSERCOM->USART.BAUD.reg  = EM_BAUD;

	/* The DMAC takes the data; the interrupt only counts errors. */
	dma_init();
	EMSERCOM->USART.INTENSET.reg = SERCOM_USART_INTENSET_ERROR;

	/* NVIC first, THEN the peripheral.  The other order leaves a gap in
	 * which the SERCOM can receive with nobody to service it, and its
	 * two-byte buffer overflows.  SERCOM1 is IRQ 10 at priority 0, above
	 * the EIC at 1 (NOTES.md §13). */
	NVIC_EnableIRQ(SERCOM1_IRQn);

	EMSERCOM->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
	em_uart_sync();
}

void SERCOM1_Handler(void)
{
	/* An overrun or framing error latches ERROR and stalls nothing, but the
	 * flag must be cleared or STATUS keeps reporting it. */
	if (EMSERCOM->USART.INTFLAG.bit.ERROR) {
		uint16_t st = EMSERCOM->USART.STATUS.reg;

		em_rx_errors++;
		em_rx_status |= st;
		EMSERCOM->USART.INTFLAG.reg = SERCOM_USART_INTFLAG_ERROR;
		EMSERCOM->USART.STATUS.reg  = st;
	}
}

uint16_t em_uart_rx_count(void)
{
	uint16_t wr = dma_wr();
	/* The DMA never waits for a slow reader: a ring that has gone all the
	 * way round reads as nearly empty.  The frame checksum catches what
	 * that corrupts; at 256 bytes it takes a reader 2.7 ms late at 750 kbaud
	 * to get there, against windows of 16 to 48 bytes. */
	return (uint16_t)((wr - g_rd) & RX_MASK);
}

uint8_t em_uart_rx_at(uint16_t i)
{
	return g_rx[(g_rd + i) & RX_MASK];
}

void em_uart_rx_consume(uint16_t n)
{
	g_rd = (uint16_t)((g_rd + n) & RX_MASK);
}

void em_uart_rx_flush(void)
{
	g_rd = dma_wr();
}

void em_uart_disable(void)
{
	g_disabled = 1;
	EMSERCOM->USART.CTRLA.reg &= ~(uint32_t)SERCOM_USART_CTRLA_ENABLE;
	em_uart_sync();
}

void em_uart_resume(void)
{
	/* Mux first, so the pin goes straight from driven-low GPIO to the
	 * idling transmitter without floating in between. */
	PORT->Group[0].PINCFG[EM_PIN_TX].reg |= PORT_PINCFG_PMUXEN;
	EMSERCOM->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
	em_uart_sync();
	g_disabled = 0;
	em_uart_rx_flush();
}

void em_uart_set_baud(uint16_t baud)
{
	if (g_disabled) {
		return;
	}
	/* BAUD is enable-protected: take the SERCOM down, write, bring it back.
	 * Anything half-received is dropped, which is harmless here -- this runs
	 * only between messages 0x0C and 0x0B, a ~20 ms silence on the bus. */
	EMSERCOM->USART.CTRLA.reg &= ~(uint32_t)SERCOM_USART_CTRLA_ENABLE;
	em_uart_sync();
	EMSERCOM->USART.BAUD.reg = baud;
	EMSERCOM->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
	em_uart_sync();
}

void em_uart_send(const uint8_t *buf, uint16_t len)
{
	if (g_disabled) {
		return;
	}
	EMSERCOM->USART.INTFLAG.reg = SERCOM_USART_INTFLAG_TXC;

	for (uint16_t i = 0; i < len; i++) {
		while (!EMSERCOM->USART.INTFLAG.bit.DRE) {
		}
		EMSERCOM->USART.DATA.reg = buf[i];
	}
	while (!EMSERCOM->USART.INTFLAG.bit.TXC) {
	}
}
