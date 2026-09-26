/* sam.h -- host stand-in for CMSIS, so the offline test can compile the real
 * src/board.h and src/emount.c instead of a copy of them.
 *
 * Only the handful of names those two files touch.  Anything else missing is a
 * compile error, which is the point: this must not quietly diverge. */
#ifndef HOST_SAM_H
#define HOST_SAM_H

#include <stdint.h>

#define MUX_PA00D_SERCOM1_PAD0      2
#define MUX_PA04D_SERCOM0_PAD0      3
#define MUX_PA08E_TCC0_WO0          4
#define MUX_PA18F_TCC0_WO2          5
#define TCC_CTRLA_PRESCALER_DIV1_Val 0

#define PORT_PINCFG_PMUXEN          0x01u
#define PORT_PINCFG_INEN            0x02u

struct host_reg8  { uint8_t  reg; };
struct host_reg32 { uint32_t reg; };
struct host_group {
	struct host_reg32 DIR, DIRCLR, DIRSET, DIRTGL;
	struct host_reg32 OUT, OUTCLR, OUTSET, OUTTGL;
	struct host_reg32 IN;
	struct host_reg8  PINCFG[32];
};
struct host_port { struct host_group Group[1]; };

extern struct host_port host_port;
#define PORT (&host_port)

#endif
