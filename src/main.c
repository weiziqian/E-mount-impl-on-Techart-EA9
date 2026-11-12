/* main.c -- the E-mount responder plus the closed-loop focus mechanism.
 *
 * This is the ONLY configuration built.  The shutdown path is
 * the stock's in full: park -> acknowledge -> UART off -> wait for the frame
 * sync -> PA23 -> delay.  Every step of it is measured, not assumed.
 *
 * REPLACES the open-loop version, which damaged nothing permanently but came
 * close.  That one drove a fixed duty for a fixed 200 ms with no feedback: at
 * 50% duty this mechanism covers more than its entire advertised travel in
 * that time, so every pulse ended against an end stop, and the 100% step
 * over-travelled hard enough that the stock firmware needed two power cycles
 * before it would drive the helicoid again.
 *
 * It answered its question -- the motor works, and a measurement run's two candidate
 * explanations were both wrong -- but the parameters were sized against a
 * result that had already been withdrawn, and it had no end-stop protection of
 * any kind.
 *
 * What this one does instead:
 *
 *   - CLOSED LOOP.  Every move has a target and stops there.  A timed pulse
 *     cannot do that; this is the single biggest change.
 *   - SMALL.  +-200 encoder counts = 50 protocol units = 3.4% of the
 *     advertised travel, against the previous version's 130%.
 *   - LOW DUTY, RAMPED.  Starts at 4% and climbs only while the mechanism is
 *     not moving, capped at 16%.  The mechanism never gets more force than it
 *     needed to start, and the duty at which it first moves is the measurement
 *     worth having.
 *   - RETURNS TO WHERE IT STARTED after every excursion, so the sequence is
 *     net-zero and cannot walk toward a stop.
 *   - STOPS THE WHOLE SEQUENCE on the first stall or runaway, instead of
 *     carrying on into the next step.
 *
 * It is still a test that moves a real mechanism. It just cannot run away.
 */
#include "board.h"
#include "clock.h"
#include "delay.h"
#include "diag.h"
#include "em_eic.h"
#include "em_uart.h"
#include "emount.h"
#include "motor.h"
#include "abs_encoder.h"
#include "servo.h"
#include "focus_map.h"
#include "focus_dist.h"
#include "trail.h"
#include "vdd.h"

#define MOTOR_PAGE0     28u
#define TRACE_PAGE0     24u
/* 25 is the gap between the homing trace (24) and the park traces (26, 27). */
#define FHIST_PAGE      25u

/* THE START-UP AND SHUT-DOWN PAGES ARE GONE.
 *
 * Sixteen of the thirty-two pages used to measure how a boot began and how a
 * session ended: two milestones, twelve one-second snapshots that dated the
 * body's two-second power cut, and two ticks measuring how long the adapter
 * stayed powered after the bus went quiet.
 *
 * All three questions are closed.  Start-up and shut-down are stable, the
 * snapshots have reported "ran past the cutoff" every run for many sessions,
 * and the quiet ticks were documented as doing nothing even when they were
 * added.  The boot TALLY stays -- it costs no pages, it is what selects the
 * slot, and it still says whether the body restarted us mid-session.
 *
 * The pages they held now go to the two measurements that are live: which
 * frame arrived in which transfer window, and more than five focus moves.
 */
#define QUIET_MS        100u        /* 12 missed frames: unambiguous */
/* ...but NOT the end of the session.  The body goes quiet for about that long
 * whenever its metering times out and comes straight back, so anything that
 * may only be written once waits for a silence no pause produces. */
#define NORM_QUIET_MS   2000u
#define QUIET_IDLOG_PAGE 3u

/* THE TRANSFER WINDOWS, one page per (length, id) kind the body sent.
 *
 * A frame is delimited by its chip select, but the number of bytes clocked
 * inside that window is not necessarily the frame's length -- and reading the
 * window as the frame drove the helicoid into its stop.  The
 * receiver already captures the first two windows of every kind, with the
 * declared length beside the byte count, and `trail_cap()` has existed to
 * write them out since the responder did.  Nothing ever called it.
 *
 * The session-wide list of distinct window sizes says 48, 16, 32 and
 * occasionally 23 or 27, but not WHICH frame arrived in which.  These pages
 * are that pairing. */
#define CAP_PAGE0       8u
#define CAP_PAGES       (EM_CAP_SLOTS - 1u)   /* the last one is MODE_PAGE */
/* Move records by mode and Scan records by speed unit, written with the
 * normal-class tally.  It takes the eighth capture page: seven raw windows
 * have always been enough to see each command form. */
#define MODE_PAGE       (CAP_PAGE0 + CAP_PAGES)

/* The body's message 0x08 request, raw.  One page, taken back from the focus
 * list, because that frame arrives once per session and carries the bit that
 * decides whether message 0x28 reports the aperture. */
#define M08_PAGE        17u

/* NO TWO WRITERS MAY SHARE A PAGE.  A flash page cannot be rewritten between
 * erases, so a collision leaves the bitwise AND of both records -- readable,
 * plausible and wrong (NOTES.md 35).  The focus list draws on pages the
 * start-up and shut-down records used to own, so it is worth stating what is
 * left where:
 *
 *   0,1,2,4..7,16     focus moves 6..13   (were BOOT, INIT, IDLG, SNAP, QUIET)
 *   17                the body's message 0x08 request
 *   3                 the id log, at the first quiet
 *   8..14             the transfer-window captures
 *   15                Move records by mode, Scan by speed unit
 *   18,19             build consistency and the build marker
 *   20..23, 29        focus moves 1..5
 *   24,26             encoder traces
 *   25                focus history
 *   27                the last focus moves
 *   28,30             the homing and park records
 *   31                the normal-class tally
 *
 * One page per focus move.  They must not overlap the homing record
 * (MOTOR_PAGE0) or the park records (MOTOR_PAGE0 + 2 + n): a flash page cannot
 * be rewritten between erases, so two writers sharing a page leave the bitwise
 * AND of both records, which is silent nonsense. */
static const uint8_t  FOCUS_PAGE[] = { 20, 21, 22, 23, 29,
                                       0, 1, 2, 4, 5, 6, 7, 16 };
#define FOCUS_PAGES (sizeof(FOCUS_PAGE) / sizeof(FOCUS_PAGE[0]))

/* The LAST few moves, written when the bus goes quiet.
 *
 * The five pages above cover the first five moves and then stop, which is the
 * wrong end of the session: focus5 ran for 61 s, made at least five moves, and
 * everything after move #4 -- including whatever happened when autofocus died
 * -- left no record at all.  A rolling record in RAM costs
 * nothing and covers the end.
 *
 * Page 27 was the SECOND park attempt's encoder trace.  A second park attempt
 * has never happened in any run. */
#define FOCUS_LAST_PAGE 27u
#define FOCUS_LAST_N    3u

/* The normal-class frame histogram: every (length, id) the body sent, with a
 * count.  trail_norm() has existed since the responder did and was never
 * called, so the one measurement that says WHICH command forms a body uses has
 * been collected in RAM and thrown away every run.
 *
 * This is page 31, which was the SECOND park attempt's MOTR record -- the
 * counterpart of the page 27 above, and unwritten for the same reason: no run
 * has ever parked twice.  A second park now leaves no MOTR page.  That is a
 * deliberate trade: the question on the table is which of the stock's four
 * inbound command lengths this body actually emits, and nothing else in the
 * trail answers it. */
#define NORM_PAGE       31u

/* The normal-class histogram, at most once.
 *
 * A flash page can be written once between erases, so the two callers -- the
 * park and the quiet detector -- must not both reach it.  A run that ends in a
 * power cut never parks, and a run that parks may never go quiet, so both are
 * needed and the guard is what makes having both safe. */
static uint8_t g_norm_written;

static void norm_page(void)
{
	if (g_norm_written) {
		return;
	}
	g_norm_written = 1;
	trail_norm(NORM_PAGE);
	trail_modes(MODE_PAGE);
	{
		unsigned k;

		for (k = 0; k < CAP_PAGES && k < em_cap_used; k++) {
			trail_cap(CAP_PAGE0 + k, k);
		}
	}
}

/* The quiet id log, at most once, for the same reason.
 *
 * It was written on EVERY quiet transition, and `quiet` clears again the
 * moment the body comes back -- so a session with several pauses wrote the
 * same flash page several times.  Flash bits only go 1 -> 0, so the page ends
 * up as the AND of every write: focus6 wrote 11 ids and later 20, and the
 * count decoded as 11 & 20 = 0, an empty page where two good ones had been. */
static uint8_t g_qidlog_written;

static void quiet_idlog_page(void)
{
	if (g_qidlog_written) {
		return;
	}
	g_qidlog_written = 1;
	trail_idlog(QUIET_IDLOG_PAGE);
}

/* THE EXPERIMENT (user's, 2026-09-20): home to the infinity end the way the
 * stock firmware does, then move one third of the way along the travel using
 * the duty the stock firmware actually uses.
 *
 * Both numbers are recovered from the image, not chosen:
 *
 *   homing   the stock main loop at 0x5e00 drives BOTH "minus" coils at
 *            800/2560 = 31.25% for 300 ms, releases, waits 500 ms, then
 *            latches the position as its reference.  At the speed measured in
 *            a measurement run that covers ~7200 counts against a 5952-count travel, so it
 *            deliberately runs into the end stop and parks there.
 *
 *            CORRECTION (2026-09-27, servo.md 10): that block is DEAD CODE.
 *            Its gate, 0x2000051c[0x14], is in zeroed .bss and the only write
 *            to it in the whole image is the clear at the end of the block
 *            itself.  The stock does NOT home at power-on -- it parks at the
 *            same infinity clamp on the way down and assumes it is still
 *            there.  So "every LM-EA9 does this at every power-on" is
 *            withdrawn; the duty and the durations are still the stock's own
 *            constants, and they are proven by this project's own runs.
 *
 *   moving   the stock servo's PID limiter (0x8bfc) works out to
 *            duty = clamp(accumulator / 200, +-1280), floored to a magnitude
 *            of 200 whenever it drives at all.  So the stock's range is
 *            **200 (7.8%) to 1280 (50%)** -- .data+0x4e8 is the maximum,
 *            +0x4ec the floor, +0x4f0 the divisor.
 *
 *            CORRECTION: an earlier version of this comment said the stock
 *            drove at a fixed 7.8%.  That was wrong twice over -- a divide
 *            routine was read as a clamp, and the struct was mis-indexed so
 *            the real maximum at param[10] was never looked at.  Running the
 *            experiment at 7.8% put it at the FLOOR of the stock's range, and
 *            it stalled after 401 of 1984 counts, which is what the floor does.
 */
static void pump(int32_t pos);
static void focus_report_at(int32_t counts);
static void forecast_model(const struct servo_cfg *cfg);
static uint8_t focus_retarget(int32_t *target);
static uint8_t focus_abort(void);
static void    focus_report_cut(uint8_t running, uint8_t drive_param);
static uint8_t focus_cmd_target(const struct em_focus_cmd *c, int32_t *out);
static void    focus_exec(const struct em_focus_cmd *c);
static void    focus_answer_queries(void);
static void focus_hist_page(uint32_t page);
/* The leg in progress, if any: where it is going and which way it was asked
 * to go.  Reported in message 0x06 pl[0..3] and 0x05's in-motion flag, and the
 * base a same-direction relative move adds to. */
static uint8_t g_moving;
static int32_t g_move_target;
static int8_t  g_move_dir;
static void focus_report(void);
static void build_marker_page(uint32_t page);
static void trace_page(uint32_t page, unsigned k, const struct servo_result *r);

#define HOME_DUTY       800         /* 31.25%, stock 0x5e00 */

/* THE RESET's constants (motor_reset, user's definition 2026-11-12).  The
 * drive runs at constant HOME_DUTY toward infinity until one of:
 *
 *   still     under HOME_STILL_COUNTS of movement for HOME_STILL_MS,
 *             counted from the start of the drive.  The window must outlast
 *             the ~12 ms from drive to first movement (NOTES.md 106.3).
 *   stop      once it has fallen RST_BACK_COUNTS below where it started
 *             (it is moving), two consecutive samples RST_SAMPLE_MS apart
 *             that fall by RST_STOP_FALL or less: it has reached the stop.
 *   reversed  RST_WRONG_COUNTS above where it started: going the wrong way.
 *   timeout   RST_TIMEOUT_MS.
 *
 * Why the still rule cannot mistake stiction for the end stop: at 800 the
 * duty is above every breakaway this mechanism has ever shown (170-536, one
 * spot >=1280), so breakaway at this duty is immediate or never (NOTES.md
 * 96-103 for the breakaway distribution). */
#define HOME_STILL_MS       40
/* Encoder noise is 3-5 counts peak-to-peak with sigma ~0.8 (NOTES.md 97).  6
 * is clear of that and still nothing at all as movement: at this duty the
 * mechanism covers ~27 counts per millisecond. */
#define HOME_STILL_COUNTS   6
/* Trace sample spacing: 24 samples over the first 48 ms of the drive, which
 * is where the still rule makes its decision. */
#define HOME_TRACE_MS       2

#define RST_TIMEOUT_MS      1000
#define RST_SAMPLE_MS       2     /* ~54 counts at full speed */
#define RST_BACK_COUNTS     20
/* Past RST_BACK_COUNTS a moving sample falls ~11-54 counts; at rest two
 * readings differ with sigma ~1.1, so -2 lets noise end it, not motion. */
#define RST_STOP_FALL       2
#define RST_WRONG_COUNTS    50
/* However the drive ended, the settle: coils floating, so the spring
 * compressed against the stop can push back freely.  Over once it has stayed
 * within RST_SETTLE_COUNTS of a reference for RST_SETTLE_STILL_MS (the still
 * rule's reference, so a slow relaxation still counts as movement),
 * RST_SETTLE_MS at most.  Its last reading is the stop. */
#define RST_SETTLE_MS       100
#define RST_SETTLE_STILL_MS 40
#define RST_SETTLE_COUNTS   6

/* How the reset's drive ended -- the trail's MOTR page, w[12] bits 24-31. */
enum { RST_END_TIMEOUT, RST_END_STILL, RST_END_STOP, RST_END_REVERSED };

/* The shutdown park stops this far short of the stop found at boot, so it
 * does not drive into it. */
#define PARK_ABOVE_STOP     20
#define PARK_TIMEOUT_MS     1000

/* 1280 = 50% of PER, the stock's own limiter.
 *
 * RAISED TO 1664 (65%) AND PUT BACK.  The reason for raising it was sound --
 * some places along the travel report `duty_first` 1280, pinned against the
 * cap with the boost having nowhere to go, and those moves take a median of
 * 370 ms against 186 for the rest.
 *
 * But the cap is also what clamps the feedforward's first-step kick, which
 * for any move over 163 counts is asking for far more than the cap anyway
 * (2248 at 300 counts, 4009 at 2000).  So raising the ceiling raised the
 * opening kick on EVERY move, not only the stuck ones, and the camera run
 * that tried it came back slower across the board: median 402 ms against
 * 208, and seven of twenty-six moves cut short by the body.
 *
 * If it is worth trying again it should be as two ceilings -- the profile
 * clamped here and only the breakaway boost allowed past -- and on its own,
 * not alongside three other changes as it was in 5.03. */
#define MOVE_DUTY       1280

/* The speed range a Scan or a Drive is mapped into, in COUNTS PER SECOND.
 *
 * It used to be a duty range, because there was no velocity loop to give a
 * speed to and no measurement to convert one into.  There is now: the body's
 * magnitude maps onto a real speed and the loop delivers it, whatever duty
 * that takes.  Both fields still carry a speed in a unit the protocol does
 * not define (autofocus.md 3.3, 3.4), so the mapping is still a choice -- but
 * it is now a choice of speeds rather than of forces.
 *
 * The floor is about what duty 250 produces, i.e. just above the measured
 * breakaway band; below that no duty reliably moves anything. */
#define SCAN_V_MIN      4000
#define SCAN_V_MAX      40000

/* A Drive's stop_ms: braking only in the last hundred counts or so.  A Scan
 * no longer uses it -- at 40 counts/ms that overran by 100-180 counts into
 * the close-focus stop; it brakes with the ordinary profile and keeps its
 * window at speed with a run-out long enough to brake in (focus_scan). */
#define SCAN_STOP_MS    8

/* THE FOCUS CONFIGURATION.  Every constant here was measured on the
 * adapter; see struct servo_cfg for which measurement, and why. */
static const struct servo_cfg STOCK = {
	/* Profile: v = remaining / 42 ms, the worst measured stopping distance
	 * per count/ms, so friction alone can always stop it. */
	.stop_ms   = 30,
	/* 25 counts/ms.  The stock's staircase tops out at 22.7 and the
	 * mechanism will do 61 at 65% duty, so this is deliberately near the
	 * stock's own working speed rather than near the hardware's limit --
	 * the first thing to raise if focus feels slow, and the easiest to
	 * raise safely because the profile bounds the approach regardless. */
	.v_cruise  = 25000,

	/* v = 0.0399 * (duty - 134) counts/ms, twenty points, both runs. */
	.ff_div    = 40,
	.ff_offset = 134,
	.tau_ms    = 26,          /* the DRIVEN time constant, not the coast */
	.kp_div    = 50,
	/* Arrival also requires the mechanism to be slow enough that the
	 * coast cannot carry it out of tolerance: 16 counts over a 33 ms
	 * coast is 0.5 counts/ms. */
	.v_arrive     = 500,
	.arrive_steps = 2,

	/* THE BREAKAWAY SEARCH: climb 2.5% of PER per 5 ms step while stuck,
	 * and drop the whole lot the instant the mechanism goes.
	 *
	 * The cost of too gentle a climb, measured on a camera: 13 of 36 moves
	 * needed a duty of 1200 or more to start, and those had a median of
	 * 370 ms against 186 for the rest.  At 12 per step the search from the
	 * feedforward's value up to 1200 takes 400 ms on its own.
	 *
	 * The climb rate has a broad optimum and both ends are worse --
	 * 12/cycle gives 189 ms and leaves stalls, 128/cycle gives 170, and
	 * anywhere from 40 to 100 gives 126-139.  64 is the middle of that
	 * plateau rather than its argmax, which is 80: a flat optimum should
	 * be taken at its centre, where being wrong about the mechanism costs
	 * least.
	 *
	 * boost_max has to let the total reach duty_max from a short move's
	 * small feedforward, so it is no longer 800. */
	.boost_up   = 12,
	.boost_down = 8,
	.boost_max  = 800,

	.duty_floor = 200,        /* below it, zero -- see servo.h */
	.duty_max   = MOVE_DUTY,

	.tolerance     = 12,
	.settle_counts = 40,
	.aim_ahead     = 0,       /* zero at this profile speed; see servo.h */
	.noise     = 8,
	.stall_ms  = 300,         /* must outlast the integral's climb */
	.timeout_ms = 3000,
	.total_ms   = 5000,
	.pump       = pump,

	/* Mid-move retargeting, as the stock does (EA9.md a measurement run).  retarget_min
	 * matches FOCUS_DEADBAND_COUNTS so a repeated identical target is not
	 * a redirection. */
	.abort        = focus_abort,
	.retarget     = focus_retarget,
	.retarget_min = 12,

	/* A reversal faster than 5 counts/ms is braked down to 5 before the
	 * drive turns round, to spare the gears -- including the first leg
	 * after one cut short at speed, the case a Scan interrupted by its
	 * Move produces.  100 ms is a bound, not the expected wait. */
	.reverse_brake_ms    = 100,
	.reverse_brake_above = 5000,

	/* A target at either end of the advertised travel is reached once the
	 * mechanism is there or beyond -- never pulled back off the stop. */
	.ends_settle = 1,
	.end_lo      = 0,
	.end_hi      = FOCUS_TRAVEL_COUNTS,
};

static uint16_t g_home_trace[SERVO_TRACE_N];
static uint8_t  g_home_trace_n;

/* Set once the boot reset has found the infinity stop.  Until then
 * abs_encoder_track() is measured from wherever the mechanism happened to be
 * at boot, so zero is not infinity. */
static int      g_homed;

/* Set for the length of motor_reset(); a shutdown that arrives meanwhile sets
 * g_rst_cut, which stops it and leaves it unhomed. */
static uint8_t  g_resetting;
static volatile uint8_t g_rst_cut;
static unsigned g_reset_n;      /* resets this boot; only the first is paged */
static uint8_t  g_parking;      /* a repeated 0x16 from the park's own pump */

/* Which page the next park writes.  A flash page cannot be rewritten between
 * erases -- programming only clears bits -- so two parks writing the same page
 * leave the bitwise AND of both records, which is silent nonsense.  It happened:
 * boot 0 of the motor6 run reported duty_first = 0 alongside 2802 counts moved
 * in 60 ms, a speed never measured.
 *
 * So only the FIRST park records anything.  Every park still moves: each
 * standby begins with a 0x16, and the lens parks for each one. */
static unsigned g_park_n;

static uint32_t g_boot;
static uint32_t g_flash_boots;
static int      g_aborted;

/* The build marker.  There is one configuration now, but the
 * page stays: an ASCII tag in the image is greppable before flashing and
 * self-identifying in a dump, and the whole point of it was that a bare
 * numeric flag is indistinguishable in a binary. */
static void build_marker_page(uint32_t page)
{
	uint32_t w[16];
	unsigned i;

	for (i = 0; i < 16; i++) {
		w[i] = 0;
	}
	w[0] = EM_DIAG_MAGIC;
	w[1] = EM_DIAG_FORMAT;
	w[2] = 0x30595244u;           /* "DRY0" -- coils live */
	w[3] = 0;
	w[4] = 7;
	w[5] = 0x57504C53u;           /* "SLPW" -- bus off, TX low, standby wait; no PA23 */
	diag_page(page, w, 16);
}

static void trace_page(uint32_t page, unsigned k, const struct servo_result *r)
{
	uint32_t w[16];
	unsigned i;

	for (i = 0; i < 16; i++) {
		w[i] = 0;
	}
	w[0] = EM_DIAG_MAGIC;
	w[1] = EM_DIAG_FORMAT;
	w[2] = 0x54524143u;           /* "TRAC" */
	w[3] = ((uint32_t)r->trace_n << 16) | (uint32_t)k;
	for (i = 0; i < r->trace_n && 4 + i / 2 < 16; i++) {
		w[4 + i / 2] |= (uint32_t)r->trace[i] << (16 * (i % 2));
	}
	diag_page(page, w, 16);
}

/* Everything the servo loop owes the rest of the system, once per iteration:
 * answer the body, and take a supply sample while the coils are live. */
static void pump(int32_t pos)
{
	/* Refresh the reported position BEFORE answering, not only between
	 * moves.
	 *
	 * The status pair goes out on every frame sync, about every 16 ms, and
	 * a move takes up to 185 ms -- so without this the body is told the
	 * pre-move position for roughly eleven consecutive frames while the
	 * mechanism is visibly moving.  A lens that reports a stale position
	 * during the only interval where it is changing is giving the body's
	 * focus loop nothing to work with.
	 *
	 * Guarded on g_homed: before the first reset, zero is not infinity,
	 * so the mapping has no anchor and the value would be meaningless. */
	if (g_homed) {
		focus_report_at(pos);
	}
	em_poll();
	/* Queries are answered on the next frame even mid-move, as Yongnuo
	 * does; otherwise one asked during a leg waits for the leg to end.
	 * Not guarded on g_homed: every answer is a conversion of the query's
	 * own operand at the declared focal length (focus_map.c,
	 * focus_dist.c) and never reads the encoder, so it is as right before
	 * homing as after -- and held back, it waited up to 800 ms and a fifth
	 * query was dropped from the four-deep queue. */
	focus_answer_queries();
	vdd_track_sample();
}

/* Has the mechanism stopped?  Tracks the position at the last real movement
 * and how long ago that was; a creep slower than the threshold per call still
 * accumulates, because the reference only moves when the threshold is crossed.
 */
struct home_still {
	int32_t  at;          /* position when it last really moved */
	uint32_t t;           /* millis() then                      */
};

static void home_still_init(struct home_still *st)
{
	st->at = abs_encoder_track();
	st->t  = millis();
}

static uint8_t home_still_for(struct home_still *st, uint32_t ms,
                              int32_t counts)
{
	int32_t now = abs_encoder_track();
	int32_t d   = now - st->at;

	if (d > counts || d < -counts) {
		st->at = now;
		st->t  = millis();
	}
	return (uint32_t)(millis() - st->t) >= ms;
}

/* THE RESET: drive to the infinity stop at constant HOME_DUTY, no boost, until
 * the end test above; settle with the coils floating; and make the reading
 * it rests at track position 0.  Returns that raw reading.  A shutdown that
 * cuts it short (g_rst_cut) leaves it unhomed and braked, and the main loop
 * runs it again after the wake. */
static uint16_t motor_reset(void)
{
	struct home_still   still;
	uint16_t idle_lo, idle_hi, load_lo, load_hi;
	uint16_t raw_start, raw_end;
	int32_t  start, prev;
	int32_t  at_drive_end;
	uint32_t t0, t_start, t_sample, drive_ms, settle_ms;
	uint8_t  reason = RST_END_TIMEOUT, back = 0, flat = 0;
	unsigned i = 0;

	g_home_trace_n = 0;
	g_resetting = 1;
	g_rst_cut   = 0;
	g_moving      = 1;          /* toward infinity */
	g_move_target = 0;
	g_move_dir    = -1;

	/* The idle supply, with the bus served throughout: 20 ms of evidence. */
	t0 = millis();
	idle_lo = 0xFFFF;
	idle_hi = 0;
	while ((uint32_t)(millis() - t0) < 20u) {
		uint16_t v = vdd_read_mv();

		if (v < idle_lo) {
			idle_lo = v;
		}
		if (v > idle_hi) {
			idle_hi = v;
		}
		pump(abs_encoder_track());
	}
	if (idle_lo > idle_hi) {
		idle_lo = 0;            /* not sampled */
	}
	raw_start = abs_encoder_raw();
	start     = prev = abs_encoder_track();
	vdd_track_reset();

	if (!STOCK.dry && !g_rst_cut) {
		motor_attach_pins();
		motor_drive(-HOME_DUTY);
	}
	t_start = t_sample = millis();
	home_still_init(&still);
	while (!g_rst_cut) {
		uint32_t now = millis();

		if ((uint32_t)(now - t_start) >= RST_TIMEOUT_MS) {
			break;
		}
		pump(abs_encoder_track());
		if (i < SERVO_TRACE_N
		    && (uint32_t)(now - t_start) >= i * HOME_TRACE_MS) {
			g_home_trace[i++] = abs_encoder_raw();
		}
		if ((uint32_t)(now - t_sample) >= RST_SAMPLE_MS) {
			int32_t pos = abs_encoder_track();

			t_sample = now;
			if (pos >= start + RST_WRONG_COUNTS) {
				reason = RST_END_REVERSED;
				break;
			}
			if (pos <= start - RST_BACK_COUNTS) {
				back = 1;
			}
			/* Counted only once it is moving; a real fall restarts it. */
			if (back) {
				flat = (pos - prev >= -RST_STOP_FALL) ? flat + 1 : 0;
				if (flat >= 2) {
					reason = RST_END_STOP;
					break;
				}
			}
			prev = pos;
		}
		if (home_still_for(&still, HOME_STILL_MS, HOME_STILL_COUNTS)) {
			reason = RST_END_STILL;
			break;
		}
	}
	g_home_trace_n = (uint8_t)i;

	drive_ms     = millis() - t_start;
	at_drive_end = abs_encoder_track();
	/* Settle, floating.  A shutdown cut has already braked (shutdown_park)
	 * and is left so. */
	if (!g_rst_cut && !STOCK.dry) {
		motor_drive(0);
		motor_release();
	}
	vdd_track_result(&load_lo, &load_hi);

	t0 = millis();
	home_still_init(&still);
	while (!g_rst_cut && (uint32_t)(millis() - t0) < RST_SETTLE_MS) {
		pump(abs_encoder_track());
		if (home_still_for(&still, RST_SETTLE_STILL_MS, RST_SETTLE_COUNTS)) {
			break;
		}
	}
	settle_ms = millis() - t0;
	raw_end  = abs_encoder_raw();

	if (!g_rst_cut) {
		/* The stop is now track position 0. */
		abs_encoder_track_set(0);
		g_homed = 1;
	}

	if (g_reset_n++ == 0) {
		uint32_t w[16];
		uint32_t t[16];
		uint32_t oc = g_rst_cut ? SERVO_ABORTED
		              : reason == RST_END_REVERSED ? SERVO_WRONG_WAY
		              : reason == RST_END_TIMEOUT  ? SERVO_TIMEOUT
		              : SERVO_OK;

		for (i = 0; i < 16; i++) {
			w[i] = 0;
			t[i] = 0;
		}
		w[0]  = EM_DIAG_MAGIC;
		w[1]  = EM_DIAG_FORMAT;
		w[2]  = TAG_MOTR;
		w[3]  = millis();
		/* The reset commands no displacement, so this word carries the
		 * settle length and the handshake time instead. */
		w[4]  = ((uint32_t)settle_ms << 16) | (em_t_handshake & 0xFFFFu);
		w[5]  = ((uint32_t)idle_hi << 16) | idle_lo;
		w[6]  = ((uint32_t)load_hi << 16) | load_lo;
		w[7]  = ((uint32_t)HOME_DUTY << 16) | HOME_DUTY;
		w[8]  = (uint32_t)start;
		w[9]  = (uint32_t)at_drive_end;   /* the settled reading: w[14] */
		w[10] = em_frames_rx;
		w[11] = em_frames_tx;
		w[12] = ((uint32_t)reason << 24) | (oc << 16)
		        | (drive_ms & 0xFFFFu);
		w[13] = (g_flash_boots << 16) | (g_boot & 0xFFFFu);
		/* Bit 31 marks the reset's word (older pages had RCAUSE here). */
		w[14] = (1u << 31) | ((uint32_t)raw_end << 16) | raw_start;
		w[15] = 0;                          /* step 0 */
		diag_page(MOTOR_PAGE0, w, 16);

		t[0] = EM_DIAG_MAGIC;
		t[1] = EM_DIAG_FORMAT;
		t[2] = 0x54524143u;
		t[3] = (uint32_t)g_home_trace_n << 16;
		for (i = 0; i < g_home_trace_n && 4 + i / 2 < 16; i++) {
			t[4 + i / 2] |= (uint32_t)g_home_trace[i]
			                << (16 * (i % 2));
		}
		diag_page(TRACE_PAGE0, t, 16);
	}

	g_moving    = 0;
	g_resetting = 0;
	return raw_end;
}

/* After the 0x16 acknowledgement: what the body is waiting for.
 *
 * The A7 IV, once it has the reply, cuts the motor supply, switches its own
 * UART off and polls RXD every 5 ms until it reads low, for up to ~30 s
 * (msg_0x16.md).  RXD is our TX.  So: switch our UART off, take PA00 away
 * from the SERCOM and drive it low.  Shown on the a9 II to release the body
 * as fast as the stock's PA23 does (NOTES.md §120).
 *
 * Then WAIT, because 0x16 is also how the body enters standby, and nothing in
 * it says which (em_sleep_until_wake, emount.h).  Switching off, the body
 * cuts our power during the wait; in standby it keeps it and wakes us with
 * the pin handshake and a short init at the speed already negotiated.
 * Yongnuo and TTArtisan both do this.
 *
 * PA23 is NO LONGER DRIVEN.  §122 found a quick re-power-on slower without
 * it, but PA23 is believed to be XDETECT, and releasing that tells the body
 * the lens has been removed (A7 IV: `LIF_DETACH_DET`, which cuts the lens's
 * logic power).  That made standby work by accident -- every wake became a
 * cold attach -- and is the likeliest reason it was "needed": without it the
 * body sat in standby waiting for a lens that could not be woken.  Whether
 * the slow re-power-on comes back with a lens that CAN be woken is the first
 * thing to check on the camera.
 *
 * Dropped from the stock sequence:
 *   VD sync      the stock waits for two frame-sync pulses first.  The body
 *                does not look at VD here, so the wait only delayed RXD low,
 *                by up to 500 ms when VD had already stopped (§121).
 *   halt         replaced by the standby wait.
 */
static void powerdown_signal(void)
{
	diag_tally_mark(DIAG_MARK_SHUTREQ, em_m16_pl0);

	/* Let the acknowledgement finish on the wire before the UART goes. */
	delay_ms(50);

	/* MEASURED, and load-bearing: switching the USART off is the only
	 * difference between an image that boots normally and one that stalls
	 * on the next power-on -- a clean single-variable A/B on the a9 II
	 * (§52).  Do not remove it without a measurement.  em_uart_send()
	 * returns immediately once disabled -- it waits on DRE, which a
	 * disabled SERCOM never sets, so one send afterwards would hang the
	 * firmware for good. */
	em_uart_disable();

	PORT->Group[0].OUTCLR.reg = (1u << EM_PIN_TX);
	PORT->Group[0].DIRSET.reg = (1u << EM_PIN_TX);
	PORT->Group[0].PINCFG[EM_PIN_TX].reg &= ~(uint8_t)PORT_PINCFG_PMUXEN;
	diag_tally_mark(DIAG_MARK_TXLOW, 0);

	/* Returns only if the body wakes us from standby. */
	em_sleep_until_wake();
}

/* The shutdown park, called from the message 0x16 handler before the
 * acknowledgement goes out (emount.c): an ordinary focus move (STOCK) to
 * PARK_ABOVE_STOP above the stop the boot reset found, not into it.  Nothing
 * to aim at before that reset has finished.
 *
 * A 0x16 that arrives DURING the boot reset (from the reset's own pump) stops
 * the motor where it is and cuts that reset short. */
static void shutdown_park(void)
{
	struct servo_cfg    cfg = STOCK;
	struct servo_result r;
	uint32_t w[16];
	int32_t  delta;
	unsigned i;

	if (g_resetting) {
		g_rst_cut = 1;
		if (!STOCK.dry) {
			motor_drive(0);
			motor_brake();
		}
		return;
	}
	if (!g_homed || g_parking) {
		return;
	}
	g_parking = 1;
	/* An ordinary move, profile and all, so it slows down on the way in.
	 * The body's hooks are off: nothing it sends now is ours to chase. */
	cfg.abort    = 0;
	cfg.retarget = 0;
	/* Bounded: the body is waiting on the acknowledgement. */
	cfg.timeout_ms = PARK_TIMEOUT_MS;
	/* Left floating, not braked: the body cuts the motor supply next. */
	cfg.end_release = 1;
	delta = PARK_ABOVE_STOP - abs_encoder_track();
	if (delta > cfg.tolerance || delta < -cfg.tolerance) {
		servo_move_rel(delta, &cfg, &r);
	} else {
		delta = 0;              /* already there: the pages are still owed */
		if (!STOCK.dry) {
			motor_release();
		}
	}

	if (g_park_n == 0) {
		if (delta) {
			for (i = 0; i < 16; i++) {
				w[i] = 0;
			}
			w[0]  = EM_DIAG_MAGIC;
			w[1]  = EM_DIAG_FORMAT;
			w[2]  = TAG_MOTR;
			w[3]  = millis();
			w[4]  = (uint32_t)delta;
			w[7]  = ((uint32_t)r.duty_peak << 16) | r.duty_first;
			w[8]  = (uint32_t)r.start;
			w[9]  = (uint32_t)r.end;
			w[10] = em_frames_rx;
			w[11] = em_frames_tx;
			w[12] = ((uint32_t)r.outcome << 16) | r.ms;
			w[13] = (g_flash_boots << 16) | (g_boot & 0xFFFFu);
			w[15] = 2;
			diag_page(MOTOR_PAGE0 + 2, w, 16);
			trace_page(TRACE_PAGE0 + 2, 2, &r);
		}
		trail_fhist(FHIST_PAGE);
		focus_hist_page(FOCUS_LAST_PAGE);
		norm_page();
	}
	g_park_n++;
	g_parking = 0;
}

/* Focus: drive the mechanism to a position the BODY asked for, on message
 * 0x04's 27-byte form, in the units message 0x06 reports.
 *
 * THE MAPPING.  Linear, anchored at the floor of the travel we advertise:
 *
 *     units  = FOCUS_EM06_LO + counts / FOCUS_COUNTS_PER_UNIT
 *     counts = (units - FOCUS_EM06_LO) * FOCUS_COUNTS_PER_UNIT
 *
 * with `counts` measured from the infinity stop, which is where homing puts
 * zero.  So the infinity end reports exactly 4144.
 *
 * Why the divide, when a slope of 1 would be simpler: the encoder runs at four
 * counts per protocol unit.  The advertised travel 4144..5632 spans 1488 =
 * 5952/4 exactly, which is the full mechanical travel in counts.
 *
 * A native lens converts through a non-linear per-lens ladder, which an adapter
 * with no electrical contact with the mounted lens cannot have.  A straight
 * line is the honest approximation: monotonic, continuous, and without the
 * doubling-back a ladder's descending branch shows.
 */
/* Ignore a commanded move smaller than this.  The encoder jitters a few counts
 * at rest, and a servo that re-runs for every one of them would drive the coils
 * continuously while the body holds focus still. */
#define FOCUS_DEADBAND_COUNTS 12

static uint16_t g_focus_moves;
static uint16_t g_focus_aborts;
static uint16_t g_stops_idle;   /* Stops that arrived with nothing running */
static uint16_t g_last_target;
static uint8_t  g_last_op;      /* enum em_focus_op of the running instruction */

/* The servo's abort hook reads this rather than the protocol flag: a Stop, a
 * Scan and a Drive all end the running leg, and only the main loop can tell
 * them apart afterwards.  g_abort_by_stop is which of those it was -- a Stop
 * is answered by the leg it stopped, the others are re-run from rest. */
static uint8_t  g_abort_pending;
static uint8_t  g_abort_by_stop;

/* An instruction the retarget hook could not apply to the running leg --
 * a Stop, a Scan, a Drive.  Handed forward so nothing is lost when the leg
 * unwinds. */
static struct em_focus_cmd g_next;
static uint8_t             g_next_pending;

struct focus_rec {
	int32_t  delta;
	uint16_t ms;
	uint16_t target_u;    /* protocol units, as the body asked */
	uint16_t end_u;       /* protocol units, where we stopped */
	uint8_t  outcome;
	uint8_t  retargets;
	uint8_t  src;
};
static struct focus_rec g_focus_ring[FOCUS_LAST_N];
static uint16_t         g_focus_ring_n;


/* Publish where we are, into every message that carries it. */
/* Publish a position the caller has already read.  Split out so the servo's
 * pump can report without a second encoder transaction -- it reads once per
 * iteration and hands the value over. */
static void focus_report_at(int32_t counts)
{
	/* Message 0x06 is the only message that carries the focus position.
	 *
	 * Anchored at the 4144 we ADVERTISE, not at the raw 4096.  The body
	 * issues its message 0x04 targets inside the travel we advertise, so
	 * reporting from a different floor would put our position and its
	 * targets 48 units apart. */
	/* The running move's target bounds pl[2..3]'s one-frame forecast
	 * (emount.c); at rest the forecast is the position.  Sent as the
	 * target itself, it was the far end of a Scan's window for the whole
	 * sweep (dump13). */
	em_set_focus_position(focus_counts_to_em06(counts),
	                      focus_counts_to_em06(g_moving ? g_move_target
	                                                    : counts));

	/* The motor for message 0x06 pl[0..1]: running for the whole servo
	 * leg, as Yongnuo's motor state is for the whole move, toward the
	 * target. */
	em_set_motor(!g_moving ? 0
	             : g_move_target > counts ? 1
	             : g_move_target < counts ? -1
	             : g_move_dir);

	/* The position trace within the frame, pl[32..38]: sampled here,
	 * where the encoder has just been read, at each eighth of the body's
	 * frame -- as Yongnuo samples it from a timer (emount.c). */
	em_sample_position(focus_counts_to_em06(counts));

	/* And the same state in message 0x05, in units the body can use without
	 * knowing this lens: the subject distance, and the in-motion flag --
	 * the same motor state as 0x06 pl[0..1], taken now, as Yongnuo 50 DF's
	 * 0x05 builder reads it (FUN_08012d08 in FUN_080180e0). */
	em_set_subject_distance(focus_distance_code(counts, em_focal_mm10),
	                        focus_distance_coarse(counts, em_focal_mm10),
	                        (uint8_t)(counts / FOCUS_COUNTS_PER_UNIT / 256));
	em_set_in_motion(g_moving);
}

/* Answer any 0x22 / 0x2E queries the body asked on the last message 0x04.
 * The conversion is the lens's own, which is the whole point of the query:
 * the body can name a distance without knowing these optics. */
static void focus_answer_queries(void)
{
	uint8_t  tag;
	uint16_t operand;

	while (em_take_query(&tag, &operand)) {
		if (tag == EM_REC_Q_POS2D) {
			em_post_query_answer(tag,
			        focus_distance_code(focus_em06_to_counts(operand),
			                            em_focal_mm10));
		} else {
			em_post_query_answer(tag,
			        focus_counts_to_em06(
			                focus_counts_for_code(operand,
			                                      em_focal_mm10)));
		}
	}
}

/* The main loop's form: read, then publish. */
static void focus_report(void)
{
	focus_report_at(abs_encoder_track());
}

/* Where a relative move (modes 4 and 6) counts from.  As Yongnuo does: while
 * a leg is running, a delta in the SAME direction adds to that leg's target;
 * otherwise -- idle, or a reversal -- it counts from where the mechanism is. */
static int32_t focus_rel_base(int16_t delta)
{
	int32_t here = abs_encoder_track();

	if (g_moving
	    && ((delta > 0 && g_move_target > here)
	        || (delta < 0 && g_move_target < here))) {
		return g_move_target;
	}
	return here;
}

/* Resolve an instruction into an ABSOLUTE encoder count, if it names one.
 *
 * Four of the seven instructions are a move to a position, and they differ
 * only in the units the body chose to say it in (autofocus.md 3.1).  All four
 * resolve here, so the servo and the main loop cannot disagree about what a
 * mode byte meant.
 *
 * Returns 0 for the instructions that are not a move to a position -- Stop,
 * Scan, Drive -- which the caller handles as their own shapes of motion.
 *
 * Every target is CLAMPED into the travel this adapter advertises, including
 * the 0x7FFF the body sends when it has no target: it lies far above any
 * lens's range, so a lens that clamps drives to its upper limit either way and
 * the protocol leaves the reading open. */
static uint8_t focus_cmd_target(const struct em_focus_cmd *c, int32_t *out)
{
	int32_t from;

	switch (c->op) {
	case EM_OP_MOVE:
		*out = focus_em06_to_counts(c->target);
		return 1;

	case EM_OP_MOVE_DIST:
		/* Mode 3: the operand is a distance code, and only the lens
		 * can turn it into a position.  Codes at or above 0x0700 are
		 * not accepted as targets; focus_counts_for_code maps them to
		 * the infinity stop, which is the clamped answer. */
		*out = focus_counts_for_code(c->target, em_focal_mm10);
		return 1;

	case EM_OP_MOVE_REL:
		/* Mode 4: a signed delta in position units.  One protocol unit
		 * is FOCUS_COUNTS_PER_UNIT encoder counts. */
		from = focus_rel_base(c->delta);
		*out = from + (int32_t)c->delta * FOCUS_COUNTS_PER_UNIT;
		break;

	case EM_OP_MOVE_DEFOCUS:
		/* Mode 6: a signed delta in defocus units, which the lens
		 * multiplies by the scale it reports every frame in 0x06
		 * pl[13..14].  Same scale, read from the same place the body
		 * reads it, so the two cannot drift apart. */
		from = focus_rel_base(c->delta);
		*out = from + (int32_t)c->delta * (int32_t)em_defocus_scale()
		              * FOCUS_COUNTS_PER_UNIT;
		break;

	default:
		return 0;
	}

	if (*out < 0) {
		*out = 0;
	} else if (*out > FOCUS_TRAVEL_COUNTS) {
		*out = FOCUS_TRAVEL_COUNTS;
	}
	return 1;
}

/* End the running leg and hand `c` forward -- or, for a Stop, report it from
 * the leg it stopped. */
static void focus_cut(const struct em_focus_cmd *c)
{
	g_abort_pending = 1;
	g_abort_by_stop = (c->op == EM_OP_STOP);
	if (!g_abort_by_stop) {
		g_next         = *c;
		g_next_pending = 1;
	}
}

/* The config of the running Scan or Drive leg.  It is live: the servo reads
 * v_cruise on every control step, so a new speed written here takes effect
 * mid-motion. */
static struct servo_cfg g_live_cfg;
static int16_t          g_drive_vel;
static uint8_t          g_drive_param;

/* A new target for the running leg, set when a Drive is taken over in the
 * abort hook, which cannot retarget; the retarget hook hands it to the servo
 * on its next call. */
static int32_t g_absorb_target;
static uint8_t g_absorb_pending;

/* A Drive's speed, by Yongnuo 35 DA's law (FUN_08010c80): |v| x 24000 / 768
 * steps/s capped at 20000, plus 4000 -- and a Yongnuo step is one protocol
 * position unit, so it is units/s here, x FOCUS_COUNTS_PER_UNIT for counts.
 * Its aperture factor is left out (about 1 wide open).
 *
 * The body hands a Scan over to a Drive part-way through the sweep, and on
 * that law the two run at about the same speed (v 192 -> 10 units/ms, scan
 * code 0x8C -> 9.8): the Drive carries the sweep on.  The old mapping, 4000
 * counts/s plus |v| over the whole s16 range, gave v 192 about 1 unit/ms, and
 * the body never found a peak in a sweep that slowed tenfold (dump13). */
static int32_t drive_v_cruise(int16_t vel)
{
	uint32_t mag = (vel < 0) ? (uint32_t)(-(int32_t)vel) : (uint32_t)vel;
	uint32_t ups = mag * 24000u / 768u;     /* units/s */
	int32_t  v;

	if (ups > 20000u) {
		ups = 20000u;
	}
	v = (int32_t)((ups + 4000u) * FOCUS_COUNTS_PER_UNIT);
	return (v > SCAN_V_MAX) ? SCAN_V_MAX : v;
}

/* A Drive in the SAME direction as the running Drive or Scan leg: take its
 * speed and its target -- the limit on that side -- and leave the motor
 * running.  The body sends one every frame, and restarting from rest on each
 * never got the mechanism past breakaway (dump11: +36, +7, -1, -4 counts per
 * 16 ms leg); and it hands a Scan over to a Drive part-way through the sweep.
 *
 * The replaced instruction is answered now, as a replaced one is: 3C p for a
 * Drive, 1F 00 for a Scan.  From here on the leg IS the Drive, which
 * focus_scan sees in g_last_op when the leg returns.  Returns 0 for anything
 * else, which goes on to end the leg as before. */
static uint8_t focus_drive_absorb(const struct em_focus_cmd *c)
{
	int16_t dir = (c->drive_vel > 0) ? 1 : -1;

	if (c->op != EM_OP_DRIVE || !g_moving || dir != g_move_dir
	    || (g_last_op != EM_OP_DRIVE && g_last_op != EM_OP_SCAN)) {
		return 0;
	}
	if (g_last_op == EM_OP_SCAN) {
		em_set_scan_state(EM_SCAN_IDLE);
		em_post_event(EM_REC_SCAN, 0x00u);
	} else {
		em_post_event(EM_REC_DRIVE, (uint8_t)(g_drive_param >> 2));
	}
	g_last_op           = EM_OP_DRIVE;
	g_drive_vel         = c->drive_vel;
	g_drive_param       = c->drive_param;
	g_live_cfg.v_cruise = drive_v_cruise(c->drive_vel);
	g_live_cfg.stop_ms  = SCAN_STOP_MS;
	forecast_model(&g_live_cfg);

	g_absorb_target  = (dir > 0) ? FOCUS_TRAVEL_COUNTS : 0;
	g_absorb_pending = (g_absorb_target != g_move_target);
	g_move_target    = g_absorb_target;
	return 1;
}

/* How a Drive ends: at the limit, or cut short by what came next. */
static void focus_drive_end(uint8_t outcome)
{
	if (outcome == SERVO_ABORTED) {
		focus_report_cut(EM_OP_DRIVE, g_drive_param);
		return;
	}
	/* Drive has no end of its own short of the limit, so its event carries
	 * the top six bits of the record's rec[1] and a 4-byte tail rather than
	 * the usual 2. */
	em_post_event(EM_REC_DRIVE, (uint8_t)(g_drive_param >> 2));
}

/* The servo's other hook: has the body called the move off?  A Stop is not a
 * retarget -- the body wants the mechanism still, so the servo returns
 * SERVO_ABORTED and the caller acknowledges.
 *
 * Called on EVERY servo iteration, unlike the retarget hook, which stops
 * being called while holding.  So a Stop, a Scan or a Drive -- or anything at
 * all during a Scan or a Drive -- is taken here and ends the leg, rather than
 * waiting in the one-slot mailbox where the next Move would overwrite it.  A Move during a Move is left for the
 * retarget hook, or for after the leg. */
static uint8_t focus_abort(void)
{
	uint8_t op = em_peek_focus_op();

	if (!g_abort_pending && op != EM_OP_NONE
	    && (op == EM_OP_STOP || op == EM_OP_SCAN || op == EM_OP_DRIVE
	        || g_last_op == EM_OP_SCAN || g_last_op == EM_OP_DRIVE)) {
		struct em_focus_cmd c;

		if (em_take_focus_cmd(&c) && !focus_drive_absorb(&c)) {
			focus_cut(&c);
		}
	}
	return g_abort_pending;
}

/* The servo's hook: has the body asked for somewhere else while we were on our
 * way?  The servo applies its own deadband, so a target rejected there has
 * still been consumed here -- which is correct, it means "no meaningful
 * change" -- and leaves the predictor at most retarget_min counts out. */
static uint8_t focus_retarget(int32_t *target)
{
	struct em_focus_cmd c;
	int32_t             want;

	if (g_absorb_pending) {
		g_absorb_pending = 0;
		*target = g_absorb_target;
		return 1;
	}
	if (!em_take_focus_cmd(&c)) {
		return 0;
	}
	if (focus_drive_absorb(&c)) {
		if (g_absorb_pending) {
			g_absorb_pending = 0;
			*target = g_absorb_target;
			return 1;
		}
		return 0;
	}

	/* THE INSTRUCTION THAT ARRIVED MID-MOVE.
	 *
	 * A new Move REPLACES the running one, and the leg carries straight on
	 * to the new target (autofocus.md 3.1 rule 4).  Only the latest
	 * instruction is answered, and here the latest one is the one this leg
	 * is now chasing -- so the single event at the end of the leg is its
	 * event, and the replaced target simply never gets one.  Nothing extra
	 * to suppress.
	 *
	 * Anything that is NOT a move to a position ends the leg instead: a
	 * Stop has no target to chase, and a Scan or a Drive is a different
	 * shape of motion that has to start from rest.  A Stop is reported by
	 * the leg it stopped; a Scan or a Drive is handed forward through
	 * g_next so the main loop can run it. */
	if (g_last_op == EM_OP_SCAN || g_last_op == EM_OP_DRIVE
	    || !focus_cmd_target(&c, &want)) {
		/* A Scan or a Drive is never retargeted: whatever arrives --
		 * a Move included -- replaces it (Yongnuo FUN_08016654). */
		focus_cut(&c);
		return 0;
	}

	g_last_target = c.target;
	g_move_target = want;
	*target = want;
	return 1;
}

/* Run one leg of motion and report it.  Everything that moves goes through
 * here, so the diagnostic page and the predictor cannot be bypassed.
 *
 * Returns the outcome.  It does NOT send an event -- who is entitled to an
 * event depends on which instruction this leg belonged to, and only the caller
 * knows that. */
/* Hand message 0x06's forecast the limits of the move about to run, in
 * protocol units: its speed cap, the speed at its full duty (the feedforward
 * model, duty = v / ff_div + ff_offset, solved for v), and its time constant. */
static void forecast_model(const struct servo_cfg *cfg)
{
	uint32_t v_sat = cfg->duty_max > cfg->ff_offset
	                 ? (uint32_t)(cfg->duty_max - cfg->ff_offset) * cfg->ff_div
	                 : 0u;

	em_set_forecast_model((uint32_t)cfg->v_cruise / FOCUS_COUNTS_PER_UNIT,
	                      v_sat / FOCUS_COUNTS_PER_UNIT, cfg->tau_ms);
}

static uint8_t focus_run(int32_t want, uint16_t target_u,
                         const struct servo_cfg *cfg)
{
	struct servo_result r;
	uint32_t w[16];
	int32_t  here, delta;
	unsigned i;

	here  = abs_encoder_track();
	delta = want - here;

	if (delta > -FOCUS_DEADBAND_COUNTS && delta < FOCUS_DEADBAND_COUNTS) {
		/* "A target equal to the current position still counts as
		 * completed, and is reported as such" (autofocus.md 3.1 rule
		 * 5).  A body waiting for an acknowledgement must not be left
		 * waiting by an optimisation of ours. */
		return SERVO_OK;
	}

	/* The reported target is the COMMANDED position, not the servo's
	 * internal start+delta -- they differ by whatever the encoder moved
	 * between the two reads. */
	g_moving      = 1;
	g_move_target = want;
	g_move_dir    = delta >= 0 ? 1 : -1;
	g_absorb_pending = 0;   /* a takeover belongs to the leg it arrived in */
	forecast_model(cfg);
	servo_move_rel(delta, cfg, &r);
	g_moving      = 0;
	focus_report();

	for (i = 0; i < 16; i++) {
		w[i] = 0;
	}
	w[0]  = EM_DIAG_MAGIC;
	w[1]  = EM_DIAG_FORMAT;
	w[2]  = TAG_FOCS;
	w[3]  = millis();
	w[4]  = (uint32_t)delta;
	w[5]  = ((uint32_t)target_u << 16) | focus_counts_to_em06(r.end);
	w[6]  = ((uint32_t)em_t04_target << 16)
	        | ((uint32_t)em_frame_period_ms << 8)
	        | (em_frame_period_n > 255u ? 255u : em_frame_period_n);
	w[7]  = ((uint32_t)r.duty_peak << 16) | r.duty_first;
	w[8]  = (uint32_t)r.start;
	w[9]  = (uint32_t)r.end;
	w[10] = em_frames_rx;
	w[11] = em_frames_tx;
	w[12] = ((uint32_t)r.outcome << 16) | r.ms;
	w[13] = (g_flash_boots << 16) | (g_boot & 0xFFFFu);
	w[14] = ((uint32_t)em_focus_n << 16) | g_focus_moves;
	w[15] = ((uint32_t)g_last_op << 24) | ((uint32_t)r.retargets << 16)
	        | em_t04_n;
	if (g_focus_moves < FOCUS_PAGES) {
		diag_page(FOCUS_PAGE[g_focus_moves], w, 16);
	}

	{
		struct focus_rec *rec =
			&g_focus_ring[g_focus_ring_n % FOCUS_LAST_N];

		rec->delta     = delta;
		rec->ms        = r.ms;
		rec->target_u  = target_u;
		rec->end_u     = focus_counts_to_em06(r.end);
		rec->outcome   = r.outcome;
		rec->retargets = r.retargets;
		rec->src       = g_last_op;
		g_focus_ring_n++;
	}

	g_focus_moves++;
	g_last_target = target_u;
	return r.outcome;
}


/* A Scan or Drive was cut short: report what Yongnuo reports (FUN_08016654,
 * FUN_080164b0).  A Stop always answers 0x1C.  A Drive replaced by a Scan or
 * a Drive answers 0x3C, a Scan replaced by a Drive answers 0x1F; any other
 * replacement leaves the report to the newcomer. */
static void focus_report_cut(uint8_t running, uint8_t drive_param)
{
	if (g_abort_by_stop) {
		em_post_event(EM_REC_STOP, 0x01u);
	} else if (g_next_pending && running == EM_OP_DRIVE
	           && (g_next.op == EM_OP_SCAN || g_next.op == EM_OP_DRIVE)) {
		em_post_event(EM_REC_DRIVE, (uint8_t)(drive_param >> 2));
	} else if (g_next_pending && running == EM_OP_SCAN
	           && g_next.op == EM_OP_DRIVE) {
		em_post_event(EM_REC_SCAN, 0x00u);
	}
}

/* ------------------------------------------------------------------------
 * SCAN -- the two-leg instruction (autofocus.md 3.3).
 *
 * The body asks for focus to cross a window [B .. A] at a steady speed so it
 * can sample the image continuously while focus moves.  That takes two
 * movements:
 *
 *     position ->   run-up        window [B .. A]         run-out
 *                 |<------>|<========================>|<------>|
 *     leg 1:  fast move to the start of the run-up
 *     leg 2:                  constant speed across the whole window ------>
 *
 * The run-up exists so the mechanism is ALREADY at speed when it crosses the
 * first endpoint, and the run-out so it is still at speed when it leaves the
 * last one.  Acceleration and deceleration then happen outside the window,
 * where the body is not measuring.
 *
 * An a9 II sends it once message 0x07 pl[0] declares a native lens; with the
 * stock 02 it sends only Move and Stop (NOTES.md 79).
 */
/* The least run-up and run-out, at any speed. */
#define SCAN_RUNUP_COUNTS   300
#define SCAN_RUNOUT_COUNTS  240

/* Leg 1, the approach (focus_scan): braking over the last ~15 ms of travel,
 * an arrival band of +-100 counts, and arrived once under 3 counts/ms --
 * below the reversal brake's 5, so the sweep turns round without a hold. */
#define SCAN_APPROACH_STOP_MS   15
#define SCAN_APPROACH_TOL       100
#define SCAN_APPROACH_V_ARRIVE  3000

static void focus_scan(const struct em_focus_cmd *c)
{
	int32_t first, second, dir, start, finish, v, runup, runout;
	uint8_t outcome;

	/* The sweep speed.  The speed code is a distance PER BODY FRAME
	 * (Yongnuo 50 DF FUN_0800a3f0): rec[4] & 0x7F units, in defocus units
	 * when bit 7 is set -- times the defocus scale we advertise -- and in
	 * position units when it is clear.  Divided by the measured frame
	 * period, and clamped to what the mechanism can do. */
	{
		uint32_t units = (uint32_t)(c->scan_speed & 0x7Fu)
		                 * ((c->scan_speed & 0x80u)
		                    ? em_defocus_scale() : 1u);
		uint32_t frame = em_frame_period_ms ? em_frame_period_ms : 17u;

		v = (int32_t)(units * FOCUS_COUNTS_PER_UNIT * 1000u / frame);
		if (v < SCAN_V_MIN) {
			v = SCAN_V_MIN;
		} else if (v > SCAN_V_MAX) {
			v = SCAN_V_MAX;
		}
	}

	/* Run-up and run-out sized to the speed, as Yongnuo sizes its run-up
	 * to the distance it takes to reach scan speed: the sweep brakes with
	 * the ordinary profile, which starts v * stop_ms before the target, so
	 * a run-out that long keeps the whole window at constant speed.  The
	 * run-up is the same length: the driven time constant (26 ms) is close
	 * to stop_ms, so reaching speed takes about as far. */
	runout = v * (int32_t)STOCK.stop_ms / 1000;
	runup  = runout;
	if (runout < SCAN_RUNOUT_COUNTS) {
		runout = SCAN_RUNOUT_COUNTS;
	}
	if (runup < SCAN_RUNUP_COUNTS) {
		runup = SCAN_RUNUP_COUNTS;
	}

	/* The sweep runs from `first` to `second` (Yongnuo FUN_0800a820 /
	 * FUN_0800a692).  Endpoint form: A to B, and B to A when
	 * rec[1] & 7 == 6.  Centred form: C+A down to C-B, flags ignored. */
	if (c->scan_flags & 0x01u) {
		int32_t centre = focus_em06_to_counts(c->scan_centre);

		first  = centre + (int32_t)c->scan_a * FOCUS_COUNTS_PER_UNIT;
		second = centre - (int32_t)c->scan_b * FOCUS_COUNTS_PER_UNIT;
	} else if ((c->scan_flags & 0x07u) == 6u) {
		first  = focus_em06_to_counts(c->scan_b);
		second = focus_em06_to_counts(c->scan_a);
	} else {
		first  = focus_em06_to_counts(c->scan_a);
		second = focus_em06_to_counts(c->scan_b);
	}

	/* Run-up beyond `first`, run-out beyond `second`, both outside the
	 * window.  Where the window reaches a travel end there is no room for
	 * them and the clamp below cuts them short; the sweep then slows over
	 * the last part of the window rather than running into the stop. */
	dir    = (second > first) ? 1 : -1;
	start  = first  - dir * runup;
	finish = second + dir * runout;

	/* Bit 3 (endpoint form): start from whichever end is nearer. */
	if (!(c->scan_flags & 0x01u) && (c->scan_flags & 0x08u)) {
		int32_t here = abs_encoder_track();
		int32_t ds   = (here > start)  ? here - start  : start  - here;
		int32_t df   = (here > finish) ? here - finish : finish - here;

		if (df < ds) {
			int32_t t = start;

			start  = finish;
			finish = t;
		}
	}
	if (start < 0) {
		start = 0;
	} else if (start > FOCUS_TRAVEL_COUNTS) {
		start = FOCUS_TRAVEL_COUNTS;
	}
	if (finish < 0) {
		finish = 0;
	} else if (finish > FOCUS_TRAVEL_COUNTS) {
		finish = FOCUS_TRAVEL_COUNTS;
	}

	/* 0x00 through the approach: the sweep has not launched yet. */
	em_set_scan_state(EM_SCAN_IDLE);

	/* Leg 1: get to the run-up point, fast.  focus_run is blocking, so the
	 * sweep cannot start before it ends.
	 *
	 * It does not need the precision of a Move -- anywhere outside the
	 * run-up point will do -- and with the Move profile it took 374-538 ms
	 * for 1000-1900 counts against 144-200 ms for the sweep (dump12): the
	 * slow run the wrong way that the body then waits out on every rescan.
	 * Yongnuo runs this leg at a fixed top speed.  So: full speed, braking
	 * only near the end, and an arrival band SCAN_APPROACH_TOL either side
	 * of a point SCAN_APPROACH_TOL beyond the run-up point, away from the
	 * window -- so the band never reaches into the run-up, whichever way
	 * this leg travels. */
	start -= dir * SCAN_APPROACH_TOL;
	if (start < 0) {
		start = 0;
	} else if (start > FOCUS_TRAVEL_COUNTS) {
		start = FOCUS_TRAVEL_COUNTS;
	}
	g_live_cfg           = STOCK;
	g_live_cfg.v_cruise  = SCAN_V_MAX;
	g_live_cfg.stop_ms   = SCAN_APPROACH_STOP_MS;
	g_live_cfg.tolerance = SCAN_APPROACH_TOL;
	g_live_cfg.v_arrive  = SCAN_APPROACH_V_ARRIVE;
	outcome = focus_run(start, focus_counts_to_em06(start), &g_live_cfg);
	if (g_last_op == EM_OP_DRIVE) {
		focus_drive_end(outcome);       /* a Drive took the leg over */
		return;
	}
	if (outcome == SERVO_ABORTED || g_next_pending) {
		em_set_scan_state(EM_SCAN_IDLE);
		focus_report_cut(EM_OP_SCAN, 0);
		return;
	}

	/* Leg 2: the sweep, at speed v, braking with the ordinary profile --
	 * early, inside the run-out -- so it stops at the target instead of
	 * overrunning it into the close-focus stop. */
	em_scan_launch();
	g_live_cfg          = STOCK;
	g_live_cfg.v_cruise = v;
	outcome = focus_run(finish, focus_counts_to_em06(finish), &g_live_cfg);
	if (g_last_op == EM_OP_DRIVE) {
		focus_drive_end(outcome);       /* a Drive took the sweep over */
		return;
	}

	if (outcome == SERVO_ABORTED) {
		em_set_scan_state(EM_SCAN_IDLE);
		focus_report_cut(EM_OP_SCAN, 0);
		return;
	}
	em_scan_finish();               /* 1F 00 follows five frames later */
}

/* ------------------------------------------------------------------------
 * DRIVE -- run toward one end at a given speed (autofocus.md 3.4).
 *
 * Seen on an a9 II once 0x01 advertised it (dump11): one every frame, and
 * taking over a Scan part-way through.  A same-direction Drive updates the
 * running leg (focus_drive_absorb) rather than replacing it.
 */
static void focus_drive(const struct em_focus_cmd *c)
{
	int32_t want;

	/* Zero or negative runs to the lower limit, positive to the upper. */
	want = (c->drive_vel > 0) ? FOCUS_TRAVEL_COUNTS : 0;

	g_live_cfg          = STOCK;
	g_live_cfg.v_cruise = drive_v_cruise(c->drive_vel);
	g_live_cfg.stop_ms  = SCAN_STOP_MS;
	g_drive_vel         = c->drive_vel;
	g_drive_param       = c->drive_param;

	focus_drive_end(focus_run(want, focus_counts_to_em06(want),
	                          &g_live_cfg));
}

/* MANUAL FOCUS: RETURN TO INFINITY.
 *
 * The stock's v1.5.0 feature ("auto-return to infinity when switching to MF",
 * servo.md 10): on entering manual focus -- message 0x04 pl[3] = 0x81 -- it
 * moves once to the lower clamp, 4144, the infinity end.  Edge-triggered, so
 * it moves once per entry, and leaving manual focus re-arms it.
 *
 * The gate is the stock's eleven consecutive 0x81 frames (EM_MF_FRAMES,
 * counted in emount.c), and on top of that the focus path must be idle --
 * no leg running (focus_exec is blocking, so the main loop only gets here
 * between legs) and no instruction waiting -- so the return can never fight
 * an instruction the body has already issued.  A corrupted frame is excluded
 * by emount.c checking the checksum before it trusts pl[3].
 *
 * The return is the LENS's move, not the body's: it posts no event, and it
 * takes no instruction from the mailbox.  Anything the body sends, or the body
 * leaving manual focus, ends the leg (mf_return_abort) and leaves the
 * instruction where it is for the main loop, which runs it as usual -- so the
 * body is answered exactly as it would be from rest.  A return cut short is
 * not retried (mf_return). */
static struct servo_cfg g_mf_cfg;
static uint8_t          g_mf_prev;       /* em_focus_mf at the last look  */
static uint8_t          g_mf_return_due; /* entered MF, not yet returned  */

static uint8_t mf_return_abort(void)
{
	return em_peek_focus_op() != EM_OP_NONE || !em_focus_mf;
}

static void mf_return(void)
{
	uint8_t outcome;

	g_mf_cfg          = STOCK;
	g_mf_cfg.abort    = mf_return_abort;
	g_mf_cfg.retarget = 0;      /* the body's Moves are not ours to chase */

	/* Not em_set_scan_state(): that would also cancel a 1F 00 still
	 * counting down from the body's last Scan, and that event is owed. */
	g_last_op       = EM_OP_NONE;
	g_abort_pending = 0;
	g_abort_by_stop = 0;
	outcome = focus_run(0, focus_counts_to_em06(0), &g_mf_cfg);
	(void)outcome;

	/* Done either way.  Arrived, or cut short -- by the body leaving
	 * manual focus, or by an instruction, and an instruction issued
	 * after manual focus began is the body positioning focus itself,
	 * which a retry would undo.  The stock, too, returns once per entry. */
	g_mf_return_due = 0;
}

/* Called once, when homing ends.  An entry into manual focus during homing is
 * not an edge: homing is itself the move to infinity, so it has already done
 * what that entry asks for.  Taking it as an edge would also run the return
 * AFTER an instruction that waited out homing -- undoing a position the body
 * chose while already in manual focus.  mf_service never runs during homing
 * (motor_reset only pumps the bus), so this is the only place the two meet. */
static void mf_after_homing(void)
{
	g_mf_prev       = em_focus_mf;
	g_mf_return_due = 0;
}

/* Once per main-loop pass, between legs. */
static void mf_service(void)
{
	uint8_t mf = em_focus_mf;

	if (mf && !g_mf_prev) {
		g_mf_return_due = 1;
	} else if (!mf) {
		g_mf_return_due = 0;    /* back to AF: nothing to do, as before */
	}
	g_mf_prev = mf;

	if (g_mf_return_due && !g_next_pending
	    && em_peek_focus_op() == EM_OP_NONE) {
		mf_return();
	}
}

/* Act on one instruction from the body. */
static void focus_exec(const struct em_focus_cmd *c)
{
	int32_t want;
	uint8_t outcome;

	if (!g_homed) {
		return;                 /* zero is not infinity yet */
	}
	g_last_op       = c->op;
	g_abort_pending = 0;
	g_abort_by_stop = 0;

	/* A new instruction clears the scan status, and a 1F 00 still waiting
	 * out its five frames (Yongnuo FUN_08016654). */
	em_set_scan_state(EM_SCAN_IDLE);

	switch (c->op) {
	case EM_OP_STOP:
		/* "Abandon any pending instruction, bring focus to rest, and
		 * report the stop."  Nothing is running here -- focus_run is
		 * blocking, so a Stop that reaches this point arrived between
		 * moves -- so there is nothing to abandon and the report is
		 * the whole of it.  Parameter 0x01, which is what the Stop
		 * event carries. */
		em_set_scan_state(EM_SCAN_IDLE);
		em_post_event(EM_REC_STOP, 0x01u);
		g_stops_idle++;
		return;

	case EM_OP_SCAN:
		focus_scan(c);
		return;

	case EM_OP_DRIVE:
		focus_drive(c);
		return;

	default:
		break;
	}

	if (!focus_cmd_target(c, &want)) {
		return;
	}
	outcome = focus_run(want, c->target, &STOCK);

	if (outcome == SERVO_ABORTED) {
		/* Cut short.  The Move itself produces no event -- it did not
		 * finish, and only the latest instruction is answered.  If a
		 * Stop is what cut it, the Stop reports its own; if a Scan or
		 * a Drive did, that one reports when IT finishes. */
		g_focus_aborts++;
		if (g_abort_by_stop) {
			em_post_event(EM_REC_STOP, 0x01u);
		}
	} else if (!c->silent) {
		/* A 0x34 move is answered by its own reply, not by 1D 00. */
		em_post_event(EM_REC_MOVE, 0x00u);
	}
}

/* The last few moves, oldest first, written once when the bus goes quiet. */
static void focus_hist_page(uint32_t page)
{
	uint32_t w[16];
	unsigned i;

	for (i = 0; i < 16; i++) {
		w[i] = 0;
	}
	w[0] = EM_DIAG_MAGIC;
	w[1] = EM_DIAG_FORMAT;
	w[2] = TAG_FLST;
	w[3] = millis();
	w[4] = ((uint32_t)g_focus_ring_n << 16) | g_focus_moves;

	for (i = 0; i < FOCUS_LAST_N; i++) {
		unsigned k = (g_focus_ring_n >= FOCUS_LAST_N)
		             ? (g_focus_ring_n + i) % FOCUS_LAST_N : i;
		const struct focus_rec *rec = &g_focus_ring[k];

		w[5 + i * 3] = ((uint32_t)rec->outcome << 24)
		               | ((uint32_t)rec->retargets << 16) | rec->ms;
		w[6 + i * 3] = (uint32_t)rec->delta;
		w[7 + i * 3] = ((uint32_t)rec->target_u << 16) | rec->end_u;
	}
	/* w[5..13] are the three records; w[14] is the only word left, so the
	 * stop counters are BYTES in it, beside the newest record's source:
	 * stops that arrived with nothing running, stops in total, and moves
	 * actually cut short by one.  The 8-bit total saturates -- the NORM
	 * page carries em_stop_n in full, this copy is here so one page tells
	 * the whole story. */
	w[14] = ((uint32_t)(g_stops_idle & 0xFFu) << 24)
	        | ((uint32_t)(em_stop_n & 0xFFu) << 16)
	        | ((uint32_t)(g_focus_aborts & 0xFFu) << 8)
	        | g_focus_ring[(g_focus_ring_n + FOCUS_LAST_N - 1) % FOCUS_LAST_N].src;
	w[15] = ((uint32_t)em_focus_n << 16) | em_t04_n;
	diag_page(page, w, 16);
}

int main(void)
{
	int      hs_marked = 0;
	int      caps_flushed = 0;
	int      caps_written = 0;
	unsigned step = 0;
	int      quiet = 0;

	/* Pins FIRST, before the clocks.
	 *
	 * PORT needs no clock setup -- it is fed from the AHB/APBB clocks that
	 * are already running out of reset -- so this is valid at the very
	 * first instruction and costs nothing.
	 *
	 * It matters because at reset every pin is an INPUT WITH NO PULL.
	 * Five of these drive something: PA10 is the lens chip select and PA23
	 * is read by the body.  A floating CMOS input at the far end can sit
	 * near its threshold and oscillate, so "undriven for seven
	 * milliseconds" is an electrical hazard, not a style question.  The
	 * stock drives them within microseconds of reset.
	 *
	 * Measured to make no difference to boot or shutdown on the a9 II, so
	 * this is chosen on principle, not on evidence -- which is why it is
	 * one ordering and not a switch. */
	board_init_pins();

	clock_init();
	delay_init();

	{
		uint32_t rcause = PM->RCAUSE.reg;

		/* Tally FIRST: it says which slot this boot owns. */
		g_flash_boots = diag_boot_tally(rcause);
		g_boot = diag_begin(g_flash_boots);
		trail_set_boot(g_boot, g_flash_boots);
	}

	/* main's one addition on top of atmel_start_init's pin state: PA16
	 * high.  Deliberately NOT moved up with board_init_pins() -- it stays
	 * ahead of motor_init() exactly as before, so the window in which PA16
	 * is high while the motor PWM is unconfigured does not grow.  Until
	 * here PA16 sits low, which is atmel_start_init's own value for it. */
	PORT->Group[0].OUTSET.reg = (1u << 16);

	abs_encoder_init();
	vdd_init();
	motor_init();
	motor_release();              /* coils stay idle until the test */

	trail_chk(18);
	build_marker_page(19);

	em_set_park_handler(shutdown_park);
	em_set_powerdown_handler(powerdown_signal);

	em_init();
	/* Message 0x34's timed moves: planned at three quarters of a normal
	 * move's cruise speed, as Yongnuo plans at ~3/4 of its top speed. */
	em_set_move34_speed((uint32_t)STOCK.v_cruise * 3u / 4u
	                    / FOCUS_COUNTS_PER_UNIT);
	/* The nearest distance the full extension reaches, at the declared
	 * focal length -- the same model as the subject distance. */
	em_set_min_focus(focus_distance_code(FOCUS_TRAVEL_COUNTS, em_focal_mm10));
	diag_tally_mark(DIAG_MARK_BUSUP, 0);

	abs_encoder_set_reference();
	abs_encoder_track_reset();

	/* The boot reset runs when the body first asks 0x10, and 0x10 waits for
	 * it (em_hold_ready). */
	em_hold_ready();

	for (;;) {
		em_poll();

		/* If the handshake has not completed by two seconds it is not
		 * going to.  The motor pages are then free -- no motor step can
		 * run without a handshake -- so the frames the body did send go
		 * there.  They have never been seen. */
		/* Not "after two seconds": in a failing boot em_init does not
		 * return until 4.6 s and the frames arrive after that, so a
		 * fixed deadline flushes an empty buffer.  Wait until the bus
		 * has been up and then quiet for half a second. */
		if (!caps_flushed && !em_handshake_done && em_t_init_done
		    && (uint32_t)(millis() - (em_t_last_frame ? em_t_last_frame
		                                              : em_t_init_done))
		       >= 500u) {
			unsigned k;

			uint32_t w[16];

			caps_flushed = 1;
			for (k = 0; k < em_pre_used && k < EM_PRE_SLOTS; k++) {
				trail_cap_pre(TRACE_PAGE0 + k, k);
			}

			/* And the timing of the failure, on a page the motor
			 * would have used had it run. */
			for (k = 0; k < 16; k++) {
				w[k] = 0;
			}
			w[0]  = EM_DIAG_MAGIC;
			w[1]  = EM_DIAG_FORMAT;
			w[2]  = TAG_FAIL;
			w[3]  = millis();
			w[4]  = em_t_first_vd;
			w[5]  = em_vd_edges;
			/* two timings per word: times are well under 65 s and
			 * there is no spare word left on this page. */
			w[6]  = (em_t_cs_first_high & 0xFFFFu)
			        | (em_first_window_ms << 16);
			w[7]  = em_t_init_done;
			w[8]  = em_t_first_frame;
			w[9]  = (em_t_last_frame & 0xFFFFu)
			        | (em_t_first_byte << 16);
			w[10] = em_frames_rx;
			w[11] = em_frames_tx;
			/* The three that separate "body silent" from "body
			 * talking, us deaf". */
			w[12] = em_cs_edges;
			/* total, and how many had arrived by the end of the first
			 * window (format 6; before it, inside that window). */
			w[13] = (em_rx_bytes & 0xFFFFu)
			        | (em_rx_bytes_at_close << 16);
			w[14] = ((uint32_t)em_pre_used << 24)
			        | ((uint32_t)em_rx_status << 8)
			        | (em_rx_errors & 0xFFu);
			/* spare high bit of the status word, NOT of the VD
			 * timestamp -- packing it there would have corrupted
			 * a real measurement to carry one flag. */
			w[14] |= (uint32_t)em_init_frame_rescued << 31;
			w[15] = PM->RCAUSE.reg;
			diag_page(MOTOR_PAGE0, w, 16);

			/* The ids in order, on their own page: with no frames
			 * at all the quiet detector never fires, so this run
			 * was the only one with no id log. */
			quiet_idlog_page();
		}

		if (!hs_marked && em_handshake_done) {
			hs_marked = 1;
			diag_tally_mark(DIAG_MARK_HANDSHAKE, 0);
		}

		/* The body's 0x08 and 0x0A requests, once the handshake is over.
		 * The command captures go out with the NORM page instead: they
		 * fill whenever the body first focuses. */
		if (!caps_written && em_handshake_done && millis() >= 3000u) {
			caps_written = 1;
			trail_m08(M08_PAGE);
		}

		/* --- has the body stopped talking? ---
		 *
		 * Observation only.  The 500 ms-silence PARK that used to live
		 * here is GONE: it was invented when it looked as though this
		 * body never sent message 0x16, and once 0x16 turned out to
		 * arrive on every proper power-off it was a second, redundant
		 * trigger for the same action -- and it fired in builds that
		 * were supposed to have no park at all, which quietly ruined
		 * the control in a measurement run.
		 *
		 * The tick pages that measured how long the adapter stayed
		 * powered afterwards are gone with the rest of the shut-down
		 * record; what is left here is the two writes that have to
		 * happen before power goes away. */
		if (em_t_last_frame
		    && (uint32_t)(millis() - em_t_last_frame) >= QUIET_MS) {
			if (!quiet) {
				quiet = 1;
				quiet_idlog_page();
			}
			/* The histogram is final only once the body is really
			 * gone.  Writing it at the FIRST 100 ms gap gave a
			 * histogram of the first 35 seconds of an 85-second
			 * session -- the body pauses for a tenth of a second
			 * whenever metering times out, and the once-only guard
			 * then locked that partial count in.  The park is the
			 * normal writer; this is the fallback for a run that
			 * is cut off, so it waits for a silence no metering
			 * pause produces. */
			if ((uint32_t)(millis() - em_t_last_frame)
			    >= NORM_QUIET_MS) {
				norm_page();
			}
		} else if (quiet && em_t_last_frame
		           && (uint32_t)(millis() - em_t_last_frame) < QUIET_MS) {
			quiet = 0;      /* it came back; that was a hiccup */
		}

		/* Reset once, on the body's first 0x10, then serve whatever
		 * focus targets arrive.  A reset cut short by a shutdown is
		 * run again after the wake: 0x10 is still held. */
		if (!g_aborted && !g_homed && em_reset_wanted) {
			motor_reset();
		}
		if (g_homed && step == 0) {
			mf_after_homing();
			focus_report();
			em_set_ready();
			step = 1;
		}
		if (step == 1) {
			/* Report first: the body is told where we are on every
			 * frame, whether or not it ever commands a move. */
			struct em_focus_cmd cmd;

			focus_report();
			focus_answer_queries();

			/* An instruction the retarget hook handed forward
			 * because it could not be applied to a running leg. */
			if (g_next_pending) {
				g_next_pending = 0;
				if (!g_aborted) {
					focus_exec(&g_next);
				}
			}

			if (em_take_focus_cmd(&cmd) && !g_aborted) {
				focus_exec(&cmd);
			}

			if (!g_aborted) {
				mf_service();
			}
		}
	}
}
