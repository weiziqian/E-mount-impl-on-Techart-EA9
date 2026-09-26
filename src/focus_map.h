/* focus_map.h -- the encoder <-> protocol conversion for the focus position,
 * on its own so it can be tested on the host.
 *
 * ONE space: message 0x06's, which is also the space message 0x04's focus
 * target is expressed in.  It is the device's own travel, and this adapter
 * advertises 4144..5632 for it.
 *
 * Anchoring the report at the 4144 we advertise rather than at the raw 4096
 * matters: the body commands within the travel we advertise, so a report that
 * started 48 units below the advertised floor would put our position and its
 * targets in different frames of reference.
 *
 * The map is LINEAR, which is a deliberate simplification.  A native lens
 * converts through a per-lens ladder of encoder counts.  An adapter with no
 * electrical contact with the lens it carries cannot have that table, so a
 * straight line anchored at infinity is the honest approximation: monotonic,
 * continuous, and free of the doubling-back a ladder's descending branch can
 * show.
 */
#ifndef FOCUS_MAP_H
#define FOCUS_MAP_H

#include <stdint.h>

/* Encoder counts per protocol unit.
 *
 * Four, not one.  The adapter's own message 0x06 space is `counts / 4 + 4096`,
 * and its advertised travel 4144..5632 spans 1488 units = 5952 counts = the
 * full mechanical travel.  So one protocol unit is four encoder counts. */
#define FOCUS_COUNTS_PER_UNIT 4

/* Counts from the infinity stop to the close-focus stop. */
#define FOCUS_TRAVEL_COUNTS   5952

/* The travel this adapter advertises in message 0x06. */
#define FOCUS_EM06_LO         4144
#define FOCUS_EM06_HI         (FOCUS_EM06_LO + FOCUS_TRAVEL_COUNTS / FOCUS_COUNTS_PER_UNIT)

/* The body sends this when it has no target.  It must never be driven to. */
#define FOCUS_NO_TARGET       0x7FFFu

/* protocol units -> counts from the infinity stop.  Clamped at both ends: at
 * or below the advertised floor is infinity, above the travel is the close
 * stop. */
int32_t  focus_em06_to_counts(uint16_t pos);

/* counts from the infinity stop -> protocol units.  Clamped the same way. */
uint16_t focus_counts_to_em06(int32_t counts);

#endif
