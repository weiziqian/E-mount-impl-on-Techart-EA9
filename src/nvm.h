/* nvm.h -- the one place this firmware erases or programs flash.
 *
 * Two kinds of write, each confined to its own region:
 *
 *   LOG  0x16000-0x1C000  the debug trail (diag.c).  Compiled out of the
 *                         release image together with the trail, and the
 *                         Makefile checks no nvm_log_* symbol survived.
 *   CFG  0x1C000-0x1D000  persistent configuration (cfg.c).  In BOTH images:
 *                         it is written only when a value changes.
 *
 * Every call checks its address against its own region and does nothing
 * outside it, so a wrong address can neither reach the bootloader (0x0-0x5000),
 * the app (0x5000-), nor the other kind's region.
 *
 * SAMD21E17A: 64 B page, 256 B erase row.  MANW is set at startup, so a page is
 * programmed only by the explicit write command. */
#ifndef NVM_H
#define NVM_H
#include <stdint.h>

#define NVM_PAGE_SIZE   64u
#define NVM_ROW_SIZE    256u

#define NVM_LOG_BASE    0x00016000u
#define NVM_LOG_END     0x0001C000u
#define NVM_CFG_BASE    0x0001C000u
#define NVM_CFG_END     0x0001D000u

#ifndef EA9_NO_LOG
void nvm_log_erase_row(uint32_t addr);
void nvm_log_write(uint32_t addr, const uint32_t *w, uint32_t words);
#endif

void nvm_cfg_erase_row(uint32_t addr);
void nvm_cfg_write(uint32_t addr, const uint32_t *w, uint32_t words);

#endif
