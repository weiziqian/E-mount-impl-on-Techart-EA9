/* test_init.c -- em_init must not discard a frame that arrives while the bus
 * is coming up.
 *
 * A separate program from test_emount because em_init runs once, at the very
 * start, and preloading the receiver would perturb every frame count in the
 * other suite.
 */
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "emount.h"
#include "emount_packets.h"

extern uint8_t  sent[][256];
extern uint16_t sent_len[];
extern int      sent_n;
void host_window_payload(const uint8_t *buf, uint16_t len);
void host_preload_rx(const uint8_t *buf, uint16_t len);
extern int host_tally_n;

static int failures;

int main(void)
{
	uint8_t req[10];

	/* The body puts its session opener in the very window we use to bring
	 * the bus up.  That is what was losing 47 and then 29 bytes on hardware:
	 * the receiver came on five milliseconds late, the buffer began
	 * mid-frame, and the rescue check for 0xF0 at offset zero could never
	 * match. */
	memset(req, 0, sizeof(req));
	req[0] = 0xF0; req[1] = 10; req[3] = 0x02; req[4] = 0x11; req[5] = 0x01;
	req[9] = 0x55;
	/* Junk on the idle line BEFORE the window opens.  The receiver is now
	 * enabled from the start of em_init, so it picks this up; clearing the
	 * buffer as the window opens is what keeps it out of the frame.  Without
	 * that clear the rescue sees junk at offset 0 and finds nothing. */
	{
		static const uint8_t noise[] = { 0x5A, 0xA5, 0x00, 0xFF, 0x13 };

		host_preload_rx(noise, sizeof(noise));
	}
	host_window_payload(req, sizeof(req));

	em_init();

	/* THE "NO CAMERA" MARK MUST NOT FIRE WHEN THERE IS ONE.
	 *
	 * em_init's wait loop marks the tally after two seconds of chip select
	 * low with no frame sync, so a dump can tell a flashing session apart
	 * from a camera that cut power early.  Here the chip select comes up
	 * at once -- a body -- and a mark would be a lie about the one case it
	 * exists to distinguish. */
	if (host_tally_n != 0) {
		printf("FAIL: em_init marked \"no camera\" with a body on the bus\n");
		failures++;
	}

	if (!em_init_frame_rescued) {
		printf("FAIL: the frame received during bring-up was discarded\n");
		failures++;
	}
	if (em_frames_rx != 1) {
		printf("FAIL: rx counted %u, expected 1\n", (unsigned)em_frames_rx);
		failures++;
	}

	em_poll();
	if (sent_n != 1) {
		printf("FAIL: %d replies, expected 1 (the 0x01 answer)\n", sent_n);
		failures++;
	} else if (sent[0][5] != 0x01) {
		printf("FAIL: replied with id %#04x, expected 0x01\n", sent[0][5]);
		failures++;
	}

	printf("init rescue: %d failure(s)\n", failures);
	return failures ? 1 : 0;
}
