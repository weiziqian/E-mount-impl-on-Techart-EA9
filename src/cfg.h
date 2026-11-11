/* cfg.h -- persistent configuration in flash, one erase row per type.
 *
 * Row layout (at NVM_CFG_BASE + slot * 256):
 *
 *   +0  type    four ASCII characters, e.g. "HOME"
 *   +4  value
 *   +8  value   the same again
 *
 * A record is valid only when the type matches and both copies agree, so a
 * blank row (all 0xFF), a write cut off by power loss, or a row of another
 * type all read as "not set".  Read it back with
 *
 *   ea9flash.py --dump 0x1c000:0x100
 */
#ifndef CFG_H
#define CFG_H
#include <stdint.h>

#define CFG_FOURCC(a, b, c, d) ((uint32_t)(a) | (uint32_t)(b) << 8 \
                                | (uint32_t)(c) << 16 | (uint32_t)(d) << 24)

/* Raw 14-bit encoder reading at the infinity end stop. */
#define CFG_TYPE_HOME   CFG_FOURCC('H', 'O', 'M', 'E')

/* 1 and *value set if a valid record exists, else 0. */
int  cfg_read(uint32_t type, uint32_t *value);

/* Erases and rewrites the type's row -- unless it already holds this value,
 * so calling it with an unchanged value costs no erase cycle. */
void cfg_write(uint32_t type, uint32_t value);

#endif
