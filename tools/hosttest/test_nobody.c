/* test_nobody.c -- em_init's "no camera" mark.
 *
 * em_init waits for the body's chip select forever, as the stock does.  After
 * two seconds of silence it marks the tally, so a dump can tell a flashing
 * session apart from a camera that cut power early -- every dump this project
 * takes carries one, because taking the dump is one.
 *
 * THIS PATH HAD NO COVERAGE AT ALL.  em_init's wait is a bare spin and nothing
 * in the shim advanced the clock inside it, so the two-second timeout had
 * never once been executed off-target; §83's claim that the suite checks it
 * was true only for the case where the chip select comes straight up.  The
 * coverage was written while em_init could give up and let a bench suite
 * run, and it outlives that: the mark is still there, and a false one still
 * makes the decoder discard a real boot.
 *
 * Two scenarios, one per invocation, because em_init runs once per process.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "emount.h"

extern int      host_wait_mode;
extern uint32_t host_cs_rise_ms;
extern int      host_vd_after_ms;
extern uint32_t host_millis;
extern int      host_tally_n;
extern uint8_t  host_tally_code;

static int failures;

static void ok(int cond, const char *what)
{
	if (!cond) {
		printf("FAIL: %s\n", what);
		failures++;
	}
}

int main(int argc, char **argv)
{
	const char *mode = argc > 1 ? argv[1] : "nobody";

	host_wait_mode = 1;

	if (!strcmp(mode, "nobody")) {
		/* Nothing clocks a frame sync, and the chip select stays down
		 * for three seconds -- past the mark.  (It has to rise
		 * eventually or em_init would never return and the test would
		 * hang, which is exactly the real behaviour.) */
		host_cs_rise_ms  = 3000;
		host_vd_after_ms = -1;

		em_init();

		ok(host_tally_n == 1, "exactly one tally mark");
		ok(host_tally_n == 1 && host_tally_code == 0xB6,
		   "and it is the NO BODY mark");
		ok(em_vd_edges == 0, "no frame sync was seen, which is why");
		printf("no-camera: %d mark(s) after %u ms\n",
		       host_tally_n, host_millis);
	} else {
		/* A CAMERA THAT IS SLOW.  It clocks its frame sync from the
		 * start but does not raise the chip select for seven seconds.
		 * em_init must wait, and must NOT mark the tally: one frame
		 * sync edge is proof a body is there, and a false "no camera"
		 * makes the decoder discard a real boot. */
		host_cs_rise_ms  = 7000;
		host_vd_after_ms = 0;

		em_init();

		ok(host_millis >= 7000,
		   "em_init waited for a body that was slow to appear");
		ok(em_vd_edges > 0, "and the frame-sync edges were counted");
		ok(host_tally_n == 0,
		   "no NO BODY mark when frame sync was seen");
		printf("slow-camera: waited %u ms, %u VD edges, %d mark(s)\n",
		       host_millis, (unsigned)em_vd_edges, host_tally_n);
	}

	printf("%s: %d failure(s)\n", mode, failures);
	return failures ? 1 : 0;
}
