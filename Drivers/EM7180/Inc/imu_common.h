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

/* Definitions */
#define return_if_fail(cond) \
	if(!(cond))              \
	{                        \
		return;              \
	}
#define return_val_if_fail(cond, val) \
	if(!(cond))                       \
	{                                 \
		return (val);                 \
	}

/* Data Structures */
typedef void (*delay_func_t)(uint32_t);
typedef int32_t (*i2c_write_func_t)(uint16_t, uint16_t, uint16_t, uint8_t*,
                                    uint16_t);
typedef int32_t (*i2c_read_func_t)(uint16_t, uint16_t, uint16_t, uint8_t*,
                                   uint16_t);

/* Function Prototypes */
int32_t i2c_read_byte(i2c_read_func_t read, uint16_t dev_addr, uint16_t addr,
                      uint8_t *byte);
int32_t i2c_write_byte(i2c_write_func_t write, uint16_t dev_addr, uint16_t addr,
                       uint8_t byte);
int32_t i2c_read(i2c_read_func_t read, uint16_t dev_addr, uint16_t addr,
                 uint8_t *data, uint16_t len);
int32_t i2c_write(i2c_write_func_t write, uint16_t dev_addr, uint16_t addr,
                  uint8_t *data, uint16_t len);

#endif /* EM7180_COMMON_h */
