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
#include "i2c.h"

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

/* Function Prototypes */
inline __attribute__((always_inline))  HAL_StatusTypeDef i2c_write_byte(
    I2C_HandleTypeDef *hi2c, uint16_t addr, uint16_t sub_addr, uint8_t data)
{
	return HAL_I2C_Mem_Write(hi2c, addr << 1, sub_addr, 1, &data, 1,
	                         HAL_MAX_DELAY);
}
inline __attribute__((always_inline))  uint8_t i2c_read_byte(
    I2C_HandleTypeDef *hi2c, uint16_t addr, uint16_t sub_addr)
{
	uint8_t temp;

	HAL_I2C_Mem_Read(hi2c, addr << 1, sub_addr, 1, &temp, 1, HAL_MAX_DELAY);

	return temp;
}
inline __attribute__((always_inline))  HAL_StatusTypeDef i2c_read(
    I2C_HandleTypeDef *hi2c, uint16_t addr, uint16_t sub_addr, uint8_t *data,
    uint16_t len)
{
	return HAL_I2C_Mem_Read(hi2c, addr << 1, sub_addr, 1, data, len,
	                        HAL_MAX_DELAY);
}

#endif /* EM7180_COMMON_h */
