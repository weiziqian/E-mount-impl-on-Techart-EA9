#include "board.h"
#include "nvm.h"

#define NVM_CMDEX    0xA500u
#define NVM_CMD_ER   0x02u
#define NVM_CMD_WP   0x04u
#define NVM_CMD_PBC  0x44u

static void nvm_wait(void)
{
	while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY)) {
	}
}

static void nvm_cmd(uint32_t byte_addr, uint32_t cmd)
{
	nvm_wait();
	NVMCTRL->ADDR.reg  = byte_addr / 2;
	NVMCTRL->CTRLA.reg = (uint16_t)(NVM_CMDEX | cmd);
	nvm_wait();
}

/* [addr, addr + len) inside [lo, hi). */
static int nvm_inside(uint32_t addr, uint32_t len, uint32_t lo, uint32_t hi)
{
	return addr >= lo && addr < hi && len <= hi - addr;
}

static void nvm_erase_row(uint32_t addr, uint32_t lo, uint32_t hi)
{
	if (addr % NVM_ROW_SIZE || !nvm_inside(addr, NVM_ROW_SIZE, lo, hi)) {
		return;
	}
	nvm_cmd(addr, NVM_CMD_ER);
}

/* Words into ONE page, without disturbing the rest of it: clearing the page
 * buffer sets it to all-ones, and programming only drives bits 1 -> 0, so the
 * words not stored leave the existing flash content alone. */
static void nvm_write(uint32_t addr, const uint32_t *w, uint32_t words,
                      uint32_t lo, uint32_t hi)
{
	uint32_t page = addr & ~(NVM_PAGE_SIZE - 1u);
	uint32_t i;

	if (addr % 4u || words == 0
	    || addr + words * 4u > page + NVM_PAGE_SIZE
	    || !nvm_inside(addr, words * 4u, lo, hi)) {
		return;
	}
	nvm_cmd(page, NVM_CMD_PBC);
	for (i = 0; i < words; i++) {
		((volatile uint32_t *)addr)[i] = w[i];
	}
	nvm_cmd(page, NVM_CMD_WP);
}

#ifndef EA9_NO_LOG
void nvm_log_erase_row(uint32_t addr)
{
	nvm_erase_row(addr, NVM_LOG_BASE, NVM_LOG_END);
}

void nvm_log_write(uint32_t addr, const uint32_t *w, uint32_t words)
{
	nvm_write(addr, w, words, NVM_LOG_BASE, NVM_LOG_END);
}
#endif

void nvm_cfg_erase_row(uint32_t addr)
{
	nvm_erase_row(addr, NVM_CFG_BASE, NVM_CFG_END);
}

void nvm_cfg_write(uint32_t addr, const uint32_t *w, uint32_t words)
{
	nvm_write(addr, w, words, NVM_CFG_BASE, NVM_CFG_END);
}
