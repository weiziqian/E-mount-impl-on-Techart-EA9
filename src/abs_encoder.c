#include "board.h"
#include "abs_encoder.h"

#define ENCSPI  ((Sercom *)ENC_SPI_SERCOM_ADDR)

static uint16_t g_reference;

static int32_t  g_track;        /* unwrapped counts since track_reset */
static uint16_t g_track_last;   /* raw value at the previous call */

static void spi_sync(void)
{
	while (ENCSPI->SPI.SYNCBUSY.reg) {
	}
}

void abs_encoder_init(void)
{
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOM0_CORE_Val)
	                    | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
	while (GCLK->STATUS.bit.SYNCBUSY) {
	}

	/* PA04/PA05/PA06 -> mux D (SERCOM0 PAD0/1/2); PA06 needs its input buffer */
	PORT->Group[0].PINCFG[ENC_SPI_PIN_MOSI].reg = PORT_PINCFG_PMUXEN;
	PORT->Group[0].PINCFG[ENC_SPI_PIN_SCK].reg  = PORT_PINCFG_PMUXEN;
	PORT->Group[0].PINCFG[ENC_SPI_PIN_MISO].reg = PORT_PINCFG_PMUXEN | PORT_PINCFG_INEN;
	PORT->Group[0].PMUX[ENC_SPI_PIN_MOSI / 2].reg =
	        PORT_PMUX_PMUXE(ENC_SPI_MUX) | PORT_PMUX_PMUXO(ENC_SPI_MUX);
	PORT->Group[0].PMUX[ENC_SPI_PIN_MISO / 2].bit.PMUXE = ENC_SPI_MUX;

	/* chip selects are plain GPIO, idle high */
	PORT->Group[0].DIRSET.reg = (1u << ENC_CS1_PIN) | (1u << ENC_CS2_PIN);
	PORT->Group[0].OUTSET.reg = (1u << ENC_CS1_PIN) | (1u << ENC_CS2_PIN);

	ENCSPI->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_SWRST;
	while (ENCSPI->SPI.CTRLA.bit.SWRST || ENCSPI->SPI.SYNCBUSY.bit.SWRST) {
	}

	ENCSPI->SPI.CTRLA.reg = ENC_SPI_CTRLA;     /* recovered verbatim */
	ENCSPI->SPI.CTRLB.reg = ENC_SPI_CTRLB;
	spi_sync();
	ENCSPI->SPI.BAUD.reg  = ENC_SPI_BAUD;

	ENCSPI->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
	spi_sync();
}

static uint8_t spi_xfer(uint8_t out)
{
	while (!ENCSPI->SPI.INTFLAG.bit.DRE) {
	}
	ENCSPI->SPI.DATA.reg = out;
	while (!ENCSPI->SPI.INTFLAG.bit.RXC) {
	}
	return (uint8_t)ENCSPI->SPI.DATA.reg;
}

uint16_t abs_encoder_raw(void)
{
	uint8_t hi, lo;

	PORT->Group[0].OUTCLR.reg = (1u << ENC_CS1_PIN);
	hi = spi_xfer(0);
	lo = spi_xfer(0);
	PORT->Group[0].OUTSET.reg = (1u << ENC_CS1_PIN);

	/* 0x89a4: ((hi << 8) | lo) >> 2 -- the top 14 bits of a big-endian word */
	return (uint16_t)((((uint16_t)hi << 8) | lo) >> 2);
}

void abs_encoder_set_reference(void)
{
	g_reference = abs_encoder_raw();
}

int32_t abs_encoder_position(void)
{
	/* 0x8cc0's wrap handling, written plainly: take the shortest signed
	 * distance from the reference around a 14-bit circle. */
	int32_t d = (int32_t)abs_encoder_raw() - (int32_t)g_reference;

	if (d > ENC_COUNTS / 2) {
		d -= ENC_COUNTS;
	} else if (d < -(ENC_COUNTS / 2)) {
		d += ENC_COUNTS;
	}
	return (d >> 2) + ENC_POS_OFFSET;
}

void abs_encoder_track_reset(void)
{
	g_track      = 0;
	g_track_last = abs_encoder_raw();
}

int32_t abs_encoder_track(void)
{
	uint16_t now = abs_encoder_raw();
	int32_t  d   = (int32_t)((now - g_track_last) & (ENC_COUNTS - 1));

	/* shortest path between two consecutive samples */
	if (d > ENC_COUNTS / 2) {
		d -= ENC_COUNTS;
	}
	g_track     += d;
	g_track_last = now;
	return g_track;
}
