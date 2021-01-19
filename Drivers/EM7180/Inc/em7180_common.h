/*
 * em7180_common.h
 *
 *  Created on: Jan 18, 2021
 *      Author: Daniel Peter Chokola
 *
 *  Library may be used freely and without limit with attribution.
 */

#ifndef EM7180_COMMON_h
#define EM7180_COMMON_h

/* Includes */
#include <stdint.h>

/* Function Prototypes */
void lsm6dsm_write_byte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t lsm6dsm_read_byte(uint8_t address, uint8_t subAddress);
void lsm6dsm_read(uint8_t address, uint8_t subAddress, uint8_t count,
                  uint8_t *dest);

#endif /* EM7180_COMMON_h */
