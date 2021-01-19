/*
 * lsm6dsm.c
 * The LSM6DSM is a sensor hub with embedded accelerometer and gyroscope, here
 * used as 6 DoF in a 10 DoF absolute orientation solution.
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
#include <math.h>
#include "em7180_common.h"
#include "lsm6dsm.h"

/* Private Global Variables */

/* Function Prototypes */

/* Function Definitions */
void lsm6dsm_init(lsm6dsm_t *lsm6dsm, I2C_HandleTypeDef *hi2c, uint8_t ascale,
                  uint8_t gscale, uint8_t a_odr, uint8_t g_odr)
{
	if(!lsm6dsm)
	{
		return;
	}

	lsm6dsm->hi2c = hi2c;
	lsm6dsm->ascale = ascale;
	lsm6dsm->gscale = gscale;
	lsm6dsm->a_odr = a_odr;
	lsm6dsm->g_odr = g_odr;
}

void lsm6dsm_config(lsm6dsm_t *lsm6dsm)
{
	lsm6dsm_write_byte(LSM6DSM_ADDRESS, LSM6DSM_CTRL1_XL,
	                   lsm6dsm->a_odr << 4 | lsm6dsm->ascale << 2);

	lsm6dsm_write_byte(LSM6DSM_ADDRESS, LSM6DSM_CTRL2_G,
	                   lsm6dsm->g_odr << 4 | lsm6dsm->gscale << 2);

	uint8_t temp = lsm6dsm_read_byte(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C);
	// enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
	lsm6dsm_write_byte(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C, temp | 0x40 | 0x04);
	// by default, interrupts active HIGH, push pull, little endian data
	// (can be changed by writing to bits 5, 4, and 1, resp to above register)

	// enable accel LP2 (bit 7 = 1), set LP2 tp ODR/9 (bit 6 = 1), enable input_composite (bit 3) for low noise
	lsm6dsm_write_byte(LSM6DSM_ADDRESS, LSM6DSM_CTRL8_XL, 0x80 | 0x40 | 0x08);

	// interrupt handling
	lsm6dsm_write_byte(LSM6DSM_ADDRESS, LSM6DSM_DRDY_PULSE_CFG, 0x80); // latch interrupt until data read
	lsm6dsm_write_byte(LSM6DSM_ADDRESS, LSM6DSM_INT1_CTRL, 0x40); // enable significant motion interrupts on INT1
	lsm6dsm_write_byte(LSM6DSM_ADDRESS, LSM6DSM_INT2_CTRL, 0x03); // enable accel/gyro data ready interrupts on INT2
}

uint8_t lsm6dsm_chip_id_get()
{
	uint8_t c = lsm6dsm_read_byte(LSM6DSM_ADDRESS, LSM6DSM_WHO_AM_I);
	return c;
}

float lsm6dsm_ares_get(lsm6dsm_t *lsm6dsm)
{
	float a_res;

	switch(lsm6dsm->ascale)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
		// Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
		case AFS_2G:
			a_res = 2.0f / 32768.0f;
			break;
		case AFS_4G:
			a_res = 4.0f / 32768.0f;
			break;
		case AFS_8G:
			a_res = 8.0f / 32768.0f;
			break;
		case AFS_16G:
			a_res = 16.0f / 32768.0f;
			break;
		default:
			a_res = NAN;
			break;
	}

	return a_res;
}

float lsm6dsm_gres_get(lsm6dsm_t *lsm6dsm)
{
	float g_res;

	switch(lsm6dsm->gscale)
	{
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
		case GFS_250DPS:
			g_res = 250.0f / 32768.0f;
			break;
		case GFS_500DPS:
			g_res = 500.0f / 32768.0f;
			break;
		case GFS_1000DPS:
			g_res = 1000.0f / 32768.0f;
			break;
		case GFS_2000DPS:
			g_res = 2000.0f / 32768.0f;
			break;
		default:
			g_res = NAN;
			break;
	}

	return g_res;
}

void lsm6dsm_reset()
{
	// reset device
	uint8_t temp = lsm6dsm_read_byte(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C);
	lsm6dsm_write_byte(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C, temp | 0x01); // Set bit 0 to 1 to reset LSM6DSM
	HAL_Delay(100); // Wait for all registers to reset
}

void lsm6dsm_selfTest()
{
	int16_t temp[7] = { 0, 0, 0, 0, 0, 0, 0 };
	int16_t accelPTest[3] = { 0, 0, 0 }, accelNTest[3] = { 0, 0, 0 },
	    gyroPTest[3] = { 0, 0, 0 }, gyroNTest[3] = { 0, 0, 0 };
	int16_t accelNom[3] = { 0, 0, 0 }, gyroNom[3] = { 0, 0, 0 };

	lsm6dsm_read_data(temp);
	accelNom[0] = temp[4];
	accelNom[1] = temp[5];
	accelNom[2] = temp[6];
	gyroNom[0] = temp[1];
	gyroNom[1] = temp[2];
	gyroNom[2] = temp[3];

	lsm6dsm_write_byte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x01); // positive accel self test
	HAL_Delay(100); // let accel respond
	lsm6dsm_read_data(temp);
	accelPTest[0] = temp[4];
	accelPTest[1] = temp[5];
	accelPTest[2] = temp[6];

	lsm6dsm_write_byte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x03); // negative accel self test
	HAL_Delay(100); // let accel respond
	lsm6dsm_read_data(temp);
	accelNTest[0] = temp[4];
	accelNTest[1] = temp[5];
	accelNTest[2] = temp[6];

	lsm6dsm_write_byte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x04); // positive gyro self test
	HAL_Delay(100); // let gyro respond
	lsm6dsm_read_data(temp);
	gyroPTest[0] = temp[1];
	gyroPTest[1] = temp[2];
	gyroPTest[2] = temp[3];

	lsm6dsm_write_byte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x0C); // negative gyro self test
	HAL_Delay(100); // let gyro respond
	lsm6dsm_read_data(temp);
	gyroNTest[0] = temp[1];
	gyroNTest[1] = temp[2];
	gyroNTest[2] = temp[3];

	lsm6dsm_write_byte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x00); // normal mode
	HAL_Delay(100); // let accel and gyro respond

	/* Serial.println("Accel Self Test:"); */
	/* Serial.print("+Ax results:"); */
	/* Serial.print((accelPTest[0] - accelNom[0]) * a_res * 1000.0); */
	/* Serial.println(" mg"); */
	/* Serial.print("-Ax results:"); */
	/* Serial.println((accelNTest[0] - accelNom[0]) * a_res * 1000.0); */
	/* Serial.print("+Ay results:"); */
	/* Serial.println((accelPTest[1] - accelNom[1]) * a_res * 1000.0); */
	/* Serial.print("-Ay results:"); */
	/* Serial.println((accelNTest[1] - accelNom[1]) * a_res * 1000.0); */
	/* Serial.print("+Az results:"); */
	/* Serial.println((accelPTest[2] - accelNom[2]) * a_res * 1000.0); */
	/* Serial.print("-Az results:"); */
	/* Serial.println((accelNTest[2] - accelNom[2]) * a_res * 1000.0); */
	/* Serial.println("Should be between 90 and 1700 mg"); */

	/* Serial.println("Gyro Self Test:"); */
	/* Serial.print("+Gx results:"); */
	/* Serial.print((gyroPTest[0] - gyroNom[0]) * g_res); */
	/* Serial.println(" dps"); */
	/* Serial.print("-Gx results:"); */
	/* Serial.println((gyroNTest[0] - gyroNom[0]) * g_res); */
	/* Serial.print("+Gy results:"); */
	/* Serial.println((gyroPTest[1] - gyroNom[1]) * g_res); */
	/* Serial.print("-Gy results:"); */
	/* Serial.println((gyroNTest[1] - gyroNom[1]) * g_res); */
	/* Serial.print("+Gz results:"); */
	/* Serial.println((gyroPTest[2] - gyroNom[2]) * g_res); */
	/* Serial.print("-Gz results:"); */
	/* Serial.println((gyroNTest[2] - gyroNom[2]) * g_res); */
	/* Serial.println("Should be between 20 and 80 dps"); */
	HAL_Delay(2000);

}

void lsm6dsm_offset_bias(lsm6dsm_t *lsm6dsm, float *dest1, float *dest2)
{
	int16_t temp[7] = { 0, 0, 0, 0, 0, 0, 0 };
	int32_t sum[7] = { 0, 0, 0, 0, 0, 0, 0 };
	float a_res = lsm6dsm_ares_get(lsm6dsm);
	float g_res = lsm6dsm_gres_get(lsm6dsm);

	/* Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!"); */
	HAL_Delay(4000);

	for(int ii = 0; ii < 128; ii++)
	{
		lsm6dsm_read_data(temp);
		sum[1] += temp[1];
		sum[2] += temp[2];
		sum[3] += temp[3];
		sum[4] += temp[4];
		sum[5] += temp[5];
		sum[6] += temp[6];
		HAL_Delay(50);
	}

	dest1[0] = sum[1] * g_res / 128.0f;
	dest1[1] = sum[2] * g_res / 128.0f;
	dest1[2] = sum[3] * g_res / 128.0f;
	dest2[0] = sum[4] * a_res / 128.0f;
	dest2[1] = sum[5] * a_res / 128.0f;
	dest2[2] = sum[6] * a_res / 128.0f;

	if(dest2[0] > 0.8f)
	{
		dest2[0] -= 1.0f;
	}  // Remove gravity from the x-axis accelerometer bias calculation
	if(dest2[0] < -0.8f)
	{
		dest2[0] += 1.0f;
	}  // Remove gravity from the x-axis accelerometer bias calculation
	if(dest2[1] > 0.8f)
	{
		dest2[1] -= 1.0f;
	}  // Remove gravity from the y-axis accelerometer bias calculation
	if(dest2[1] < -0.8f)
	{
		dest2[1] += 1.0f;
	}  // Remove gravity from the y-axis accelerometer bias calculation
	if(dest2[2] > 0.8f)
	{
		dest2[2] -= 1.0f;
	}  // Remove gravity from the z-axis accelerometer bias calculation
	if(dest2[2] < -0.8f)
	{
		dest2[2] += 1.0f;
	}  // Remove gravity from the z-axis accelerometer bias calculation

}

void lsm6dsm_read_data(int16_t *destination)
{
	uint8_t rawdata[14];  // x/y/z accel register data stored here
	lsm6dsm_read(LSM6DSM_ADDRESS, LSM6DSM_OUT_TEMP_L, 14, &rawdata[0]); // Read the 14 raw data registers into data array
	destination[0] = ((int16_t) rawdata[1] << 8) | rawdata[0]; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t) rawdata[3] << 8) | rawdata[2];
	destination[2] = ((int16_t) rawdata[5] << 8) | rawdata[4];
	destination[3] = ((int16_t) rawdata[7] << 8) | rawdata[6];
	destination[4] = ((int16_t) rawdata[9] << 8) | rawdata[8];
	destination[5] = ((int16_t) rawdata[11] << 8) | rawdata[10];
	destination[6] = ((int16_t) rawdata[13] << 8) | rawdata[12];
}

// I2C read/write functions for the LSM6DSM
void lsm6dsm_write_byte(uint8_t addr, uint8_t sub_addr, uint8_t data)
{
	uint8_t temp[2];
	temp[0] = sub_addr;
	temp[1] = data;
	/* Wire.transfer(addr, &temp[0], 2, NULL, 0); */
}

uint8_t lsm6dsm_read_byte(uint8_t addr, uint8_t sub_addr)
{
	uint8_t temp[1];
	/* Wire.transfer(addr, &sub_addr, 1, &temp[0], 1); */
	return temp[0];
}

void lsm6dsm_read(uint8_t addr, uint8_t sub_addr, uint8_t count, uint8_t *dest)
{
	/* Wire.transfer(addr, &sub_addr, 1, dest, count); */
}
