/*
 * lps22hb.c
 * The LPS22HB is a low power barometer, here used as 1 DoF in a 10 DoF
 * absolute orientation solution.
 *
 *  Created on: Jan 18, 2021
 *      Author: Daniel Peter Chokola
 *
 *  Adapted From:
 *      EM7180_LSM6DSM_XXYYZZ_LPS22HB_Butterfly
 *      by: Kris Winer
 *      09/23/2017 Copyright Tlera Corporation
 *
 *  Library may be used freely and without limit with attribution.
 */

/* Includes */
#include <stdint.h>
#include <stddef.h>
#include "em7180_common.h"
#include "lps22hb.h"

/* Definitions */
/*
 * LPS22H Registers
 * http://www.st.com/content/ccc/resource/technical/document/datasheet/bf/c1/4f/23/61/17/44/8a/DM00140895.pdf/files/DM00140895.pdf/jcr:content/translations/en.DM00140895.pdf
 */
#define LPS22H_INTERRUPT_CFG 0x0B
#define LPS22H_THS_P_L       0x0C
#define LPS22H_THS_P_H       0x0D
#define LPS22H_WHOAMI        0x0F // should return 0xB1
#define LPS22H_CTRL_REG1     0x10
#define LPS22H_CTRL_REG2     0x11
#define LPS22H_CTRL_REG3     0x12
#define LPS22H_FIFO_CTRL     0x14
#define LPS22H_REF_P_XL      0x15
#define LPS22H_REF_P_L       0x16
#define LPS22H_REF_P_H       0x17
#define LPS22H_RPDS_L        0x18
#define LPS22H_RPDS_H        0x19
#define LPS22H_RES_CONF      0x1A
#define LPS22H_INT_SOURCE    0x25
#define LPS22H_FIFO_STATUS   0x26
#define LPS22H_STATUS        0x27
#define LPS22H_PRESS_OUT_XL  0x28
#define LPS22H_PRESS_OUT_L   0x29
#define LPS22H_PRESS_OUT_H   0x2A
#define LPS22H_TEMP_OUT_L    0x2B
#define LPS22H_TEMP_OUT_H    0x2C
#define LPS22H_LPFP_RES      0x33

/* Macros */
#define lps22hb_read_byte(addr, byte) i2c_read_byte((lps22hb->i2c_read_func), (lps22hb->i2c_addr), (addr), (byte))
#define lps22hb_write_byte(addr, byte) i2c_write_byte((lps22hb->i2c_write_func), (lps22hb->i2c_addr), (addr), (byte))
#define lps22hb_read(addr, data, len) i2c_read((lps22hb->i2c_read_func), (lps22hb->i2c_addr), (addr), (data), (len))
#define lps22hb_write(addr, data, len) i2c_write((lps22hb->i2c_write_func), (lps22hb->i2c_addr), (addr), (data), (len))

/* Private Global Variables */

/* Function Prototypes */

/* Function Definitions */
lps22hb_status_t lps22hb_init(lps22hb_t *lps22hb, lps22hb_init_t *init)
{
	int8_t *ptr = (int8_t*) lps22hb;
	size_t i;

	return_val_if_fail(lps22hb, LPS22HB_BAD_ARG);
	return_val_if_fail(init, LPS22HB_BAD_ARG);

	/* zero lps22hb_t struct */
	for(i = 0; i < sizeof(lps22hb_t); i++)
	{
		*ptr++ = 0;
	}

	lps22hb->init = init;

	return LPS22HB_OK;
}

void lps22hb_set_delay_cb(lps22hb_t *lps22hb, delay_func_t delay_func)
{
	return_if_fail(lps22hb);

	lps22hb->delay_func = delay_func;
}

void lps22hb_set_i2c_cbs(lps22hb_t *lps22hb, i2c_read_func_t i2c_read_func,
                         i2c_write_func_t i2c_write_func, uint8_t dev_addr)
{
	return_if_fail(lps22hb);

	lps22hb->i2c_read_func = i2c_read_func;
	lps22hb->i2c_write_func = i2c_write_func;
	lps22hb->i2c_addr = dev_addr;
}

lps22hb_status_t lps22hb_config(lps22hb_t *lps22hb)
{
	int32_t ret = 0;

	/*
	 * set sample rate by setting bits 6:4
	 * enable low-pass filter by setting bit 3 to one
	 * bit 2 == 0 means bandwidth is odr/9, bit 2 == 1 means bandwidth is odr/20
	 * make sure data not updated during read by setting block data udate (bit 1) to 1
	 */
	ret |= lps22hb_write_byte(LPS22H_CTRL_REG1,
	                          lps22hb->init->p_odr << 4 | 0x08 | 0x02);
	ret |= lps22hb_write_byte(LPS22H_CTRL_REG3, 0x04); // enable data ready as interrupt source

	return ret ? LPS22HB_BAD_COMM : LPS22HB_OK;
}

lps22hb_status_t lps22hb_status(lps22hb_t *lps22hb, uint8_t *status)
{
	int32_t ret = 0;

	/* read the status register of the altimeter */
	ret |= lps22hb_read_byte(LPS22H_STATUS, status);

	return ret ? LPS22HB_BAD_COMM : LPS22HB_OK;
}

lps22hb_status_t lps22hb_chip_id_get(lps22hb_t *lps22hb, uint8_t *data)
{
	int32_t ret = 0;

	/* read the WHO_AM_I register of the altimeter; this is a good test of communication */
	ret |= lps22hb_read_byte(LPS22H_WHOAMI, data); // Read WHO_AM_I register for LPS22H

	return ret ? LPS22HB_BAD_COMM : LPS22HB_OK;
}

lps22hb_status_t lps22hb_pressure_get(lps22hb_t *lps22hb, int32_t *data)
{
	int32_t ret = 0;
	uint8_t temp[3];  // 24-bit pressure register data stored here

	ret |= lps22hb_read(LPS22H_PRESS_OUT_XL | 0x80, temp, 3); // bit 7 must be one to read multiple bytes
	*data = ((int32_t) temp[2] << 16 | (int32_t) temp[1] << 8 | temp[0]);

	return ret ? LPS22HB_BAD_COMM : LPS22HB_OK;
}

lps22hb_status_t lps22hb_temp_get(lps22hb_t *lps22hb, int16_t *data)
{
	int32_t ret = 0;
	uint8_t temp[2];  // 16-bit pressure register data stored here

	ret |= lps22hb_read(LPS22H_TEMP_OUT_L | 0x80, temp, 2); // bit 7 must be one to read multiple bytes
	*data = ((int16_t) temp[1] << 8 | temp[0]);

	return ret ? LPS22HB_BAD_COMM : LPS22HB_OK;
}
