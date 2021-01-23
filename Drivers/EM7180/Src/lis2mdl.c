/*
 * lis2mdl.c
 * The LIS2MDL is a low power magnetometer, here used as 3 DoF in a 10 DoF
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
#include <imu_common.h>
#include <lis2mdl.h>
#include <stddef.h>

/* Definitions */
/*
 * Register map for LIS2MDL
 * http://www.st.com/content/ccc/resource/technical/document/datasheet/group3/29/13/d1/e0/9a/4d/4f/30/DM00395193/files/DM00395193.pdf/jcr:content/translations/en.DM00395193.pdf
 */
#define LIS2MDL_OFFSET_X_REG_L        0x45
#define LIS2MDL_OFFSET_X_REG_H        0x46
#define LIS2MDL_OFFSET_Y_REG_L        0x47
#define LIS2MDL_OFFSET_Y_REG_H        0x48
#define LIS2MDL_OFFSET_Z_REG_L        0x49
#define LIS2MDL_OFFSET_Z_REG_H        0x4A
#define LIS2MDL_WHO_AM_I              0x4F
#define LIS2MDL_CFG_REG_A             0x60
#define LIS2MDL_CFG_REG_B             0x61
#define LIS2MDL_CFG_REG_C             0x62
#define LIS2MDL_INT_CTRL_REG          0x63
#define LIS2MDL_INT_SOURCE_REG        0x64
#define LIS2MDL_INT_THS_L_REG         0x65
#define LIS2MDL_INT_THS_H_REG         0x66
#define LIS2MDL_STATUS_REG            0x67
#define LIS2MDL_OUTX_L_REG            0x68
#define LIS2MDL_OUTX_H_REG            0x69
#define LIS2MDL_OUTY_L_REG            0x6A
#define LIS2MDL_OUTY_H_REG            0x6B
#define LIS2MDL_OUTZ_L_REG            0x6C
#define LIS2MDL_OUTZ_H_REG            0x6D
#define LIS2MDL_TEMP_OUT_L_REG        0x6E
#define LIS2MDL_TEMP_OUT_H_REG        0x6F

/* Macros */
#define lis2mdl_read_byte(addr, byte) i2c_read_byte((lis2mdl->i2c_read_func), (lis2mdl->i2c_addr), (addr), (byte))
#define lis2mdl_write_byte(addr, byte) i2c_write_byte((lis2mdl->i2c_write_func), (lis2mdl->i2c_addr), (addr), (byte))
#define lis2mdl_read(addr, data, len) i2c_read((lis2mdl->i2c_read_func), (lis2mdl->i2c_addr), (addr), (data), (len))
#define lis2mdl_write(addr, data, len) i2c_write((lis2mdl->i2c_write_func), (lis2mdl->i2c_addr), (addr), (data), (len))

/* Private Global Variables */

/* Function Prototypes */

/* Function Definitions */
lis2mdl_status_t lis2mdl_init(lis2mdl_t *lis2mdl, lis2mdl_init_t *init)
{
	int8_t *ptr = (int8_t*) lis2mdl;
	size_t i;

	return_val_if_fail(lis2mdl, LIS2MDL_BAD_ARG);
	return_val_if_fail(init, LIS2MDL_BAD_ARG);

	/* zero lis2mdl_t struct */
	for(i = 0; i < sizeof(lis2mdl_t); i++)
	{
		*ptr++ = 0;
	}

	lis2mdl->init = init;

	return LIS2MDL_OK;
}

void lis2mdl_set_delay_cb(lis2mdl_t *lis2mdl, delay_func_t delay_func)
{
	return_if_fail(lis2mdl);

	lis2mdl->delay_func = delay_func;
}

void lis2mdl_set_i2c_cbs(lis2mdl_t *lis2mdl, i2c_read_func_t i2c_read_func,
                         i2c_write_func_t i2c_write_func, uint8_t dev_addr)
{
	return_if_fail(lis2mdl);

	lis2mdl->i2c_read_func = i2c_read_func;
	lis2mdl->i2c_write_func = i2c_write_func;
	lis2mdl->i2c_addr = dev_addr;
}

lis2mdl_status_t lis2mdl_config(lis2mdl_t *lis2mdl)
{
	int32_t ret = 0;

	/* enable temperature compensation (bit 7 == 1), continuous mode (bits 0:1 == 00) */
	ret |= lis2mdl_write_byte(LIS2MDL_CFG_REG_A,
	                          0x80 | lis2mdl->init->m_odr << 2);
	/* enable low pass filter (bit 0 == 1), set to ODR/4 */
	ret |= lis2mdl_write_byte(LIS2MDL_CFG_REG_B, 0x01);
	/* enable data ready on interrupt pin (bit 0 == 1), enable block data read (bit 4 == 1) */
	ret |= lis2mdl_write_byte(LIS2MDL_CFG_REG_C, 0x01 | 0x10);

	return ret ? LIS2MDL_BAD_COMM : LIS2MDL_OK;
}

/* FIXME: haven't explored the usage/usefulness of these yet: */
#if(0)
uint8_t lis2mdl_chip_id_get(lis2mdl_t *lis2mdl)
{
	uint8_t c;
	lis2mdl_read_byte(LIS2MDL_WHO_AM_I, &c);

	return c;
}

void lis2mdl_reset(lis2mdl_t *lis2mdl)
{
	// reset device
	uint8_t temp;
	lis2mdl_read_byte(LIS2MDL_CFG_REG_A, &temp);

	lis2mdl_write_byte(LIS2MDL_CFG_REG_A, temp | 0x20); // Set bit 5 to 1 to reset LIS2MDL
	lis2mdl->delay_func(1);
	lis2mdl_write_byte(LIS2MDL_CFG_REG_A, temp | 0x40); // Set bit 6 to 1 to boot LIS2MDL
	lis2mdl->delay_func(100); // Wait for all registers to reset
}

uint8_t lis2mdl_status(lis2mdl_t *lis2mdl)
{
	// Read the status register of the altimeter  
	uint8_t temp;
	lis2mdl_read_byte(LIS2MDL_STATUS_REG, &temp);
	return temp;
}

void lis2mdl_data_get(lis2mdl_t *lis2mdl, int16_t *destination)
{
	uint8_t data[6];  // x/y/z mag register data stored here
	lis2mdl_read((0x80 | LIS2MDL_OUTX_L_REG), data, 8); // Read the 6 raw data registers into data array

	destination[0] = ((int16_t) data[1] << 8) | data[0]; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t) data[3] << 8) | data[2];
	destination[2] = ((int16_t) data[5] << 8) | data[4];
}

int16_t lis2mdl_temp_get(lis2mdl_t *lis2mdl)
{
	uint8_t data[2];  // x/y/z mag register data stored here
	lis2mdl_read(0x80 | LIS2MDL_TEMP_OUT_L_REG, data, 2); // Read the 8 raw data registers into data array

	int16_t temp = ((int16_t) data[1] << 8) | data[0]; // Turn the MSB and LSB into a signed 16-bit value
	return temp;
}

void lis2mdl_offset_bias(lis2mdl_t *lis2mdl, float *dest1, float *dest2)
{
	int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
	int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] =
	    { 32767, 32767, 32767 }, mag_temp[3] = { 0, 0, 0 };
	float m_res = 0.0015f;

	/* Serial.println("Calculate mag offset bias: move all around to sample the complete response surface!"); */
	lis2mdl->delay_func(4000);

	for(int ii = 0; ii < 4000; ii++)
	{
		lis2mdl_data_get(lis2mdl, mag_temp);
		for(int jj = 0; jj < 3; jj++)
		{
			if(mag_temp[jj] > mag_max[jj])
			{
				mag_max[jj] = mag_temp[jj];
			}
			if(mag_temp[jj] < mag_min[jj])
			{
				mag_min[jj] = mag_temp[jj];
			}
		}
		lis2mdl->delay_func(12);
	}

	m_res = 0.0015f; // fixed sensitivity
	// Get hard iron correction
	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

	dest1[0] = (float) mag_bias[0] * m_res; // save mag biases in G for main program
	dest1[1] = (float) mag_bias[1] * m_res;
	dest1[2] = (float) mag_bias[2] * m_res;

	// Get soft iron correction estimate
	mag_scale[0] = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
	mag_scale[1] = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
	mag_scale[2] = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0f;

	dest2[0] = avg_rad / ((float) mag_scale[0]);
	dest2[1] = avg_rad / ((float) mag_scale[1]);
	dest2[2] = avg_rad / ((float) mag_scale[2]);

	/* Serial.println("Mag Calibration done!"); */
}

void lis2mdl_self_test(lis2mdl_t *lis2mdl)
{
	uint8_t c;
	int16_t temp[3] = { 0, 0, 0 };
	float magTest[3] = { 0., 0., 0. };
	float magNom[3] = { 0., 0., 0. };
	int32_t sum[3] = { 0, 0, 0 };
	float m_res = 0.0015f;

	// first, get average response with self test disabled
	for(int ii = 0; ii < 50; ii++)
	{
		lis2mdl_data_get(lis2mdl, temp);
		sum[0] += temp[0];
		sum[1] += temp[1];
		sum[2] += temp[2];
		lis2mdl->delay_func(50);
	}

	magNom[0] = (float) sum[0] / 50.0f;
	magNom[1] = (float) sum[1] / 50.0f;
	magNom[2] = (float) sum[2] / 50.0f;

	lis2mdl_read_byte(LIS2MDL_CFG_REG_C, &c);
	lis2mdl_write_byte(LIS2MDL_CFG_REG_C, c | 0x02); // enable self test
	lis2mdl->delay_func(100); // let mag respond

	sum[0] = 0;
	sum[1] = 0;
	sum[2] = 0;
	for(int ii = 0; ii < 50; ii++)
	{
		lis2mdl_data_get(lis2mdl, temp);
		sum[0] += temp[0];
		sum[1] += temp[1];
		sum[2] += temp[2];
		lis2mdl->delay_func(50);
	}

	magTest[0] = (float) sum[0] / 50.0f;
	magTest[1] = (float) sum[1] / 50.0f;
	magTest[2] = (float) sum[2] / 50.0f;

	lis2mdl_write_byte(LIS2MDL_CFG_REG_C, c); // return to previous settings/normal mode
	lis2mdl->delay_func(100); // let mag respond

	/* Serial.println("Mag Self Test:"); */
	/* Serial.print("Mx results:"); */
	/* Serial.print((magTest[0] - magNom[0]) * m_res * 1000.0); */
	/* Serial.println(" mG"); */
	/* Serial.print("My results:"); */
	/* Serial.println((magTest[0] - magNom[0]) * m_res * 1000.0); */
	/* Serial.print("Mz results:"); */
	/* Serial.println((magTest[1] - magNom[1]) * m_res * 1000.0); */
	/* Serial.println("Should be between 15 and 500 mG"); */
	/* lis2mdl->delay_func(2000);  // give some time to read the screen */
}
#endif
