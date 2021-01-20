/*
 * lps22hb.c
 * The LPS22HB is a low power barometer, here used as 1 DoF in a 10 DoF
 * absolute orientation solution.
 *
 *  Created on: Jan 18, 2021
 *      Author: Daniel Peter Chokola
 *
 *  Adapted From:
 *      EM7180_LSM6DSM_LIS2MDL_LPS22HB_Butterfly
 *      by: Kris Winer
 *      09/23/2017 Copyright Tlera Corporation
 *
 *  Library may be used freely and without limit with attribution.
 */

/* Includes */
#include <stdint.h>
#include "em7180_common.h"
#include "lps22hb.h"

/* Private Global Variables */

/* Function Prototypes */

/* Function Definitions */
void lps22hb_init(lps22hb_t *lps22hb, uint8_t p_odr)
{
	return_if_fail(lps22hb);

	lps22hb->p_odr = p_odr;
}

void lps22hb_config(lps22hb_t *lps22hb, I2C_HandleTypeDef *hi2c)
{
	// set sample rate by setting bits 6:4
	// enable low-pass filter by setting bit 3 to one
	// bit 2 == 0 means bandwidth is odr/9, bit 2 == 1 means bandwidth is odr/20
	// make sure data not updated during read by setting block data udate (bit 1) to 1
	i2c_write_byte(hi2c, LPS22H_ADDRESS, LPS22H_CTRL_REG1,
	               lps22hb->p_odr << 4 | 0x08 | 0x02);
	i2c_write_byte(hi2c, LPS22H_ADDRESS, LPS22H_CTRL_REG3, 0x04); // enable data ready as interrupt source
}

uint8_t lps22hb_chip_id_get(I2C_HandleTypeDef *hi2c)
{
	// Read the WHO_AM_I register of the altimeter this is a good test of communication
	uint8_t temp = i2c_read_byte(hi2c, LPS22H_ADDRESS, LPS22H_WHOAMI); // Read WHO_AM_I register for LPS22H

	return temp;
}

uint8_t lps22hb_status(I2C_HandleTypeDef *hi2c)
{
	// Read the status register of the altimeter  
	uint8_t temp = i2c_read_byte(hi2c, LPS22H_ADDRESS, LPS22H_STATUS);

	return temp;
}

int32_t lps22hb_pressure_get(I2C_HandleTypeDef *hi2c)
{
	uint8_t data[3];  // 24-bit pressure register data stored here

	i2c_read(hi2c, LPS22H_ADDRESS, (LPS22H_PRESS_OUT_XL | 0x80), data, 3); // bit 7 must be one to read multiple bytes

	return ((int32_t) data[2] << 16 | (int32_t) data[1] << 8 | data[0]);
}

int16_t lps22hb_temp_get(I2C_HandleTypeDef *hi2c)
{
	uint8_t data[2];  // 16-bit pressure register data stored here

	i2c_read(hi2c, LPS22H_ADDRESS, (LPS22H_TEMP_OUT_L | 0x80), data, 2); // bit 7 must be one to read multiple bytes

	return ((int16_t) data[1] << 8 | data[0]);
}
