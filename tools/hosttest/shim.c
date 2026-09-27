/* shim.c -- the hardware emount.c talks to, replaced by something observable.
 *
 * Every send is captured in full so the test can check the bytes that would
 * have gone on the wire: length, terminator, checksum, sequence number.
 */
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "em_uart.h"
#include "em_eic.h"

struct host_port host_port;

/* emount.c samples these around the bring-up window; the real ones live in
 * em_uart.c, which the host build replaces. */
volatile uint32_t em_rx_bytes;
volatile uint32_t em_rx_errors;
volatile uint16_t em_rx_status;

/* ---- captured transmissions --------------------------------------------- */
/* The transmit log.  Tests index it as `sent[before + 1]` after a frame sync,
 * so a FULL log does not merely lose frames -- every such index then reads a
 * stale frame and the assertion against it passes or fails at random.  That
 * happened: the suite grew past 64 frames and three event-queue checks read
 * frames from thousands of lines earlier.  Hence the room, and the overflow
 * counter the suite fails on. */
#define MAX_SENT 256
uint8_t  sent[MAX_SENT][256];
uint16_t sent_len[MAX_SENT];
int      sent_cs_asserted[MAX_SENT];   /* was our chip select high? */
int      sent_n;
int      sent_overflow;

/* ---- the frame the "body" is presenting --------------------------------- */
static uint8_t  rx[256];
static uint16_t rx_len;

uint32_t host_millis;

void board_init_pins(void) {}
uint32_t millis(void) { return host_millis; }
void delay_ms(uint32_t ms) { host_millis += ms; }
void delay_us(uint32_t us) { (void)us; }
void clock_enable_peripheral(uint8_t id, uint32_t m) { (void)id; (void)m; }

/* The flash trail.  emount.c writes one tally mark -- "no camera attached" --
 * from inside its chip-select wait, and there is no flash here.  Recorded
 * rather than discarded so a test can assert the mark was made. */
uint8_t  host_tally_code;
uint8_t  host_tally_data;
int      host_tally_n;
void diag_tally_mark(uint8_t code, uint8_t data)
{
	host_tally_code = code;
	host_tally_data = data;
	host_tally_n++;
}

void em_uart_init(void) {}
void em_uart_rx_reset(void) { rx_len = 0; }

/* Put bytes in the receiver without a chip-select edge, which is exactly what
 * happens when the body transmits inside the bring-up window. */
void host_preload_rx(const uint8_t *buf, uint16_t len)
{
	memcpy(rx, buf, len);
	rx_len = len;
}

/* Bytes the body sends INSIDE the bring-up window.  em_init now clears the
 * receiver the moment the chip select goes high, so a plain preload would
 * simply be wiped -- which is correct behaviour and made the old test's
 * premise wrong.  This delivers the payload as the window closes instead. */
static uint8_t win_buf[64];
static uint16_t win_len;

void host_window_payload(const uint8_t *b, uint16_t n)
{
	memcpy(win_buf, b, n);
	win_len = n;
}
uint16_t em_uart_rx_len(void) { return rx_len; }
const uint8_t *em_uart_rx_buf(void) { return rx; }

/* OUTSET and OUTCLR are write-one-to-act registers with no readback, and C
 * gives no way to hook a struct assignment -- so the test clears both before
 * each body frame and reads them afterwards.  Within one frame they then say
 * exactly what em_transmit did with the chip select.  Getting this wrong once
 * made the chip-select assertion untested while appearing to pass. */
void host_reset_cs(void)
{
	host_port.Group[0].OUTSET.reg = 0;
	host_port.Group[0].OUTCLR.reg = 0;
}

void em_uart_send(const uint8_t *buf, uint16_t len)
{
	if (sent_n < MAX_SENT) {
		memcpy(sent[sent_n], buf, len);
		sent_len[sent_n] = len;
		sent_cs_asserted[sent_n] =
		        (host_port.Group[0].OUTSET.reg >> EM_PIN_LENS_CS) & 1;
		sent_n++;
	} else {
		sent_overflow++;
	}
}

int host_cs_released(void)
{
	return (host_port.Group[0].OUTCLR.reg >> EM_PIN_LENS_CS) & 1;
}

/* ---- the EIC, driven by the test --------------------------------------- */
static void (*cs_cb)(int level);
static void (*vd_cb)(void);

void em_eic_init(void (*cs_edge)(int), void (*vd_edge)(void))
{
	cs_cb = cs_edge;
	vd_cb = vd_edge;
}

static int host_cs_pin = 0;

/* em_init polls this twice: once waiting for the body's chip select to rise,
 * once waiting for it to fall.  Script exactly that, so em_init returns
 * instead of spinning, and a third read would show the idle level. */
static const int level_script[] = { 1, 0 };
static unsigned level_idx;

/* BRING-UP SCRIPTING for the two cases the scripted window cannot express: a
 * body that is slow to appear, and no body at all.  em_init's chip-select wait
 * is a bare spin, so nothing advances the clock inside it -- which is why the
 * two-second timeout in there had never once been executed by this suite.
 *
 *   host_cs_rise_ms   when the chip select goes high; 0 = never
 *   host_vd_after_ms  when the frame sync starts clocking; <0 = never
 */
int      host_wait_mode;
uint32_t host_cs_rise_ms;
int      host_vd_after_ms = -1;

/* PER PIN.  em_init polls the chip select twice -- once for the rising edge,
 * once for the falling -- and polls the frame sync freely in between.  One
 * shared script meant the frame-sync polls ate the chip-select entries and
 * em_init spun forever. */
int em_pin_level(uint8_t pin)
{
	if (host_wait_mode) {
		if (pin == EM_PIN_BODY_VD) {
			if (host_vd_after_ms >= 0
			    && (int)host_millis >= host_vd_after_ms) {
				return (host_millis / 2) & 1;
			}
			return 0;
		}
		/* Time advances only when the code under test looks at it, as
		 * it does in sim_motor.c.  One millisecond per turn of
		 * em_init's wait loop. */
		host_millis++;
		if (!host_cs_rise_ms || host_millis < host_cs_rise_ms) {
			return 0;
		}
		return host_millis < host_cs_rise_ms + 2;
	}
	if (pin == EM_PIN_BODY_VD) {
		return 0;              /* idle: no frame sync during bring-up */
	}
	if (level_idx < sizeof(level_script) / sizeof(level_script[0])) {
		int v = level_script[level_idx++];

		if (!v && win_len) {          /* the window just closed */
			/* APPEND, as a real receiver does.  Overwriting made
			 * anything already in the buffer vanish, so the test
			 * could not tell whether em_init clears it when the
			 * window opens -- and the mutant that removed that
			 * clear passed. */
			if (rx_len + win_len <= sizeof(rx)) {
				memcpy(rx + rx_len, win_buf, win_len);
				rx_len += win_len;
			}
			win_len = 0;
		}
		return v;
	}
	return host_cs_pin;
}

/* Present one frame exactly as the wire would: chip select up, bytes, down.
 *
 * WINDOW PADDING.  A real body does not send a window the size of its frame.
 * Every window measured on an a9 II is 16, 32 or 48 bytes, so a 22-byte frame
 * arrives with ten bytes of nothing behind it.  This shim used to deliver the
 * frame and not a byte more, which is why it could not catch a parser that
 * walked to the end of the WINDOW instead of the end of the FRAME -- and that
 * parser drove the helicoid into its close stop on hardware (NOTES.md 82).
 *
 * host_pad_to sets the window size the next frames arrive in.  Zero means "no
 * padding", the old behaviour, for the tests that do not care.
 */
int host_pad_to;

void host_body_frame(const uint8_t *frame, uint16_t len)
{
	uint16_t win = len;

	host_reset_cs();
	host_cs_pin = 1;
	if (cs_cb) cs_cb(1);
	memcpy(rx, frame, len);
	if (host_pad_to > (int)len && host_pad_to <= (int)sizeof(rx)) {
		win = (uint16_t)host_pad_to;
		/* Padding is whatever the line held, NOT zeroes -- zero is not
		 * a record tag, so zero padding would hide the bug this exists
		 * to expose.  0x55 is the terminator byte, which is exactly the
		 * value that turned up as an operand on hardware. */
		memset(&rx[len], 0x55, (size_t)(win - len));
	}
	rx_len = win;
	host_cs_pin = 0;
	if (cs_cb) cs_cb(0);
}

void host_frame_sync(void) { host_reset_cs(); if (vd_cb) vd_cb(); }
void host_set_cs_pin(int level) { host_cs_pin = level; }
