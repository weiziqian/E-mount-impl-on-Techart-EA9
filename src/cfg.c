#include "cfg.h"
#include "nvm.h"

/* Each type owns one row; a new type takes the next slot.  Keeping types in
 * separate rows means rewriting one never risks another. */
static uint32_t cfg_addr(uint32_t type)
{
	switch (type) {
	case CFG_TYPE_HOME:
		return NVM_CFG_BASE + 0u * NVM_ROW_SIZE;
	default:
		return 0;
	}
}

int cfg_read(uint32_t type, uint32_t *value)
{
	uint32_t addr = cfg_addr(type);
	const volatile uint32_t *r = (const volatile uint32_t *)addr;

	if (addr == 0 || r[0] != type || r[1] != r[2]) {
		return 0;
	}
	*value = r[1];
	return 1;
}

void cfg_write(uint32_t type, uint32_t value)
{
	uint32_t addr = cfg_addr(type);
	uint32_t old;
	uint32_t w[3];

	if (addr == 0 || (cfg_read(type, &old) && old == value)) {
		return;
	}
	w[0] = type;
	w[1] = value;
	w[2] = value;
	nvm_cfg_erase_row(addr);
	nvm_cfg_write(addr, w, 3);
}
