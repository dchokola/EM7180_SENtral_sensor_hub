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
#include <imu_common.h>
#include <lsm6dsm.h>
#include <stddef.h>

/* Definitions */
/*
 * LSM6DSM registers
 * http://www.st.com/content/ccc/resource/technical/document/datasheet/76/27/cf/88/c5/03/42/6b/DM00218116.pdf/files/DM00218116.pdf/jcr:content/translations/en.DM00218116.pdf
 */
#define LSM6DSM_FUNC_CFG_ACCESS           0x01
#define LSM6DSM_SENSOR_SYNC_TIME_FRAME    0x04
#define LSM6DSM_SENSOR_SYNC_RES_RATIO     0x05
#define LSM6DSM_FIFO_CTRL1                0x06
#define LSM6DSM_FIFO_CTRL2                0x07
#define LSM6DSM_FIFO_CTRL3                0x08
#define LSM6DSM_FIFO_CTRL4                0x09
#define LSM6DSM_FIFO_CTRL5                0x0A
#define LSM6DSM_DRDY_PULSE_CFG            0x0B
#define LSM6DSM_INT1_CTRL                 0x0D
#define LSM6DSM_INT2_CTRL                 0x0E
#define LSM6DSM_WHO_AM_I                  0x0F  // should be 0x6A
#define LSM6DSM_CTRL1_XL                  0x10
#define LSM6DSM_CTRL2_G                   0x11
#define LSM6DSM_CTRL3_C                   0x12
#define LSM6DSM_CTRL4_C                   0x13
#define LSM6DSM_CTRL5_C                   0x14
#define LSM6DSM_CTRL6_C                   0x15
#define LSM6DSM_CTRL7_G                   0x16
#define LSM6DSM_CTRL8_XL                  0x17
#define LSM6DSM_CTRL9_XL                  0x18
#define LSM6DSM_CTRL10_C                  0x19
#define LSM6DSM_MASTER_CONFIG             0x1A
#define LSM6DSM_WAKE_UP_SRC               0x1B
#define LSM6DSM_TAP_SRC                   0x1C
#define LSM6DSM_D6D_SRC                   0x1D
#define LSM6DSM_STATUS_REG                0x1E
#define LSM6DSM_OUT_TEMP_L                0x20
#define LSM6DSM_OUT_TEMP_H                0x21
#define LSM6DSM_OUTX_L_G                  0x22
#define LSM6DSM_OUTX_H_G                  0x23
#define LSM6DSM_OUTY_L_G                  0x24
#define LSM6DSM_OUTY_H_G                  0x25
#define LSM6DSM_OUTZ_L_G                  0x26
#define LSM6DSM_OUTZ_H_G                  0x27
#define LSM6DSM_OUTX_L_XL                 0x28
#define LSM6DSM_OUTX_H_XL                 0x29
#define LSM6DSM_OUTY_L_XL                 0x2A
#define LSM6DSM_OUTY_H_XL                 0x2B
#define LSM6DSM_OUTZ_L_XL                 0x2C
#define LSM6DSM_OUTZ_H_XL                 0x2D
#define LSM6DSM_SENSORHUB1_REG            0x2E
#define LSM6DSM_SENSORHUB2_REG            0x2F
#define LSM6DSM_SENSORHUB3_REG            0x30
#define LSM6DSM_SENSORHUB4_REG            0x31
#define LSM6DSM_SENSORHUB5_REG            0x32
#define LSM6DSM_SENSORHUB6_REG            0x33
#define LSM6DSM_SENSORHUB7_REG            0x34
#define LSM6DSM_SENSORHUB8_REG            0x35
#define LSM6DSM_SENSORHUB9_REG            0x36
#define LSM6DSM_SENSORHUB10_REG           0x37
#define LSM6DSM_SENSORHUB11_REG           0x38
#define LSM6DSM_SENSORHUB12_REG           0x39
#define LSM6DSM_FIFO_STATUS1              0x3A
#define LSM6DSM_FIFO_STATUS2              0x3B
#define LSM6DSM_FIFO_STATUS3              0x3C
#define LSM6DSM_FIFO_STATUS4              0x3D
#define LSM6DSM_FIFO_DATA_OUT_L           0x3E
#define LSM6DSM_FIFO_DATA_OUT_H           0x3F
#define LSM6DSM_TIMESTAMP0_REG            0x40
#define LSM6DSM_TIMESTAMP1_REG            0x41
#define LSM6DSM_TIMESTAMP2_REG            0x42
#define LSM6DSM_STEP_TIMESTAMP_L          0x49
#define LSM6DSM_STEP_TIMESTAMP_H          0x4A
#define LSM6DSM_STEP_COUNTER_L            0x4B
#define LSM6DSM_STEP_COUNTER_H            0x4C
#define LSM6DSM_SENSORHUB13_REG           0x4D
#define LSM6DSM_SENSORHUB14_REG           0x4E
#define LSM6DSM_SENSORHUB15_REG           0x4F
#define LSM6DSM_SENSORHUB16_REG           0x50
#define LSM6DSM_SENSORHUB17_REG           0x51
#define LSM6DSM_SENSORHUB18_REG           0x52
#define LSM6DSM_FUNC_SRC1                 0x53
#define LSM6DSM_FUNC_SRC2                 0x54
#define LSM6DSM_WRIST_TILT_IA             0x55
#define LSM6DSM_TAP_CFG                   0x58
#define LSM6DSM_TAP_THS_6D                0x59
#define LSM6DSM_INT_DUR2                  0x5A
#define LSM6DSM_WAKE_UP_THS               0x5B
#define LSM6DSM_WAKE_UP_DUR               0x5C
#define LSM6DSM_FREE_FALL                 0x5D
#define LSM6DSM_MD1_CFG                   0x5E
#define LSM6DSM_MD2_CFG                   0x5F
#define LSM6DSM_MASTER_MODE_CODE          0x60
#define LSM6DSM_SENS_SYNC_SPI_ERROR_CODE  0x61
#define LSM6DSM_OUT_MAG_RAW_X_L           0x66
#define LSM6DSM_OUT_MAG_RAW_X_H           0x67
#define LSM6DSM_OUT_MAG_RAW_Y_L           0x68
#define LSM6DSM_OUT_MAG_RAW_Y_H           0x69
#define LSM6DSM_OUT_MAG_RAW_Z_L           0x6A
#define LSM6DSM_OUT_MAG_RAW_Z_H           0x6B
#define LSM6DSM_INT_OIS                   0x6F
#define LSM6DSM_CTRL1_OIS                 0x70
#define LSM6DSM_CTRL2_OIS                 0x71
#define LSM6DSM_CTRL3_OIS                 0x72
#define LSM6DSM_X_OFS_USR                 0x73
#define LSM6DSM_Y_OFS_USR                 0x74
#define LSM6DSM_Z_OFS_USR                 0x75

/* Macros */
#define lsm6dsm_read_byte(addr, byte) i2c_read_byte((lsm6dsm->i2c_read_func), (lsm6dsm->i2c_addr), (addr), (byte))
#define lsm6dsm_write_byte(addr, byte) i2c_write_byte((lsm6dsm->i2c_write_func), (lsm6dsm->i2c_addr), (addr), (byte))
#define lsm6dsm_read(addr, data, len) i2c_read((lsm6dsm->i2c_read_func), (lsm6dsm->i2c_addr), (addr), (data), (len))
#define lsm6dsm_write(addr, data, len) i2c_write((lsm6dsm->i2c_write_func), (lsm6dsm->i2c_addr), (addr), (data), (len))

/* Private Global Variables */

/* Function Prototypes */

/* Function Definitions */
lsm6dsm_status_t lsm6dsm_init(lsm6dsm_t *lsm6dsm, lsm6dsm_init_t *init)
{
	int8_t *ptr = (int8_t*) lsm6dsm;
	size_t i;

	return_val_if_fail(lsm6dsm, LSM6DSM_BAD_ARG);
	return_val_if_fail(init, LSM6DSM_BAD_ARG);

	/* zero lsm6dsm_t struct */
	for(i = 0; i < sizeof(lsm6dsm_t); i++)
	{
		*ptr++ = 0;
	}

	lsm6dsm->init = init;

	return LSM6DSM_OK;
}

void lsm6dsm_set_delay_cb(lsm6dsm_t *lsm6dsm, delay_func_t delay_func)
{
	return_if_fail(lsm6dsm);

	lsm6dsm->delay_func = delay_func;
}

void lsm6dsm_set_i2c_cbs(lsm6dsm_t *lsm6dsm, i2c_read_func_t i2c_read_func,
                         i2c_write_func_t i2c_write_func, uint8_t dev_addr)
{
	return_if_fail(lsm6dsm);

	lsm6dsm->i2c_read_func = i2c_read_func;
	lsm6dsm->i2c_write_func = i2c_write_func;
	lsm6dsm->i2c_addr = dev_addr;
}

lsm6dsm_status_t lsm6dsm_config(lsm6dsm_t *lsm6dsm)
{
	int32_t ret = 0;
	uint8_t temp;

	ret |= lsm6dsm_write_byte(
	    LSM6DSM_CTRL1_XL,
	    lsm6dsm->init->a_odr << 4 | lsm6dsm->init->ascale << 2);
	ret |= lsm6dsm_write_byte(
	    LSM6DSM_CTRL2_G,
	    lsm6dsm->init->g_odr << 4 | lsm6dsm->init->gscale << 2);
	ret |= lsm6dsm_read_byte(LSM6DSM_CTRL3_C, &temp);
	// enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
	ret |= lsm6dsm_write_byte(LSM6DSM_CTRL3_C, temp | 0x40 | 0x04);
	// by default, interrupts active HIGH, push pull, little endian data
	// (can be changed by writing to bits 5, 4, and 1, resp to above register)

	// enable accel LP2 (bit 7 = 1), set LP2 tp ODR/9 (bit 6 = 1), enable input_composite (bit 3) for low noise
	ret |= lsm6dsm_write_byte(LSM6DSM_CTRL8_XL, 0x80 | 0x40 | 0x08);

	// interrupt handling
	ret |= lsm6dsm_write_byte(LSM6DSM_DRDY_PULSE_CFG, 0x80); // latch interrupt until data read
	ret |= lsm6dsm_write_byte(LSM6DSM_INT1_CTRL, 0x40); // enable significant motion interrupts on INT1
	ret |= lsm6dsm_write_byte(LSM6DSM_INT2_CTRL, 0x03); // enable accel/gyro data ready interrupts on INT2

	return ret ? LSM6DSM_BAD_COMM : LSM6DSM_OK;
}

/* FIXME: haven't explored the usage/usefulness of these yet: */
#if(0)
uint8_t lsm6dsm_chip_id_get(lsm6dsm_t *lsm6dsm)
{
	uint8_t c;

	lsm6dsm_read_byte(LSM6DSM_WHO_AM_I, &c);

	return c;
}

float lsm6dsm_ares_get(lsm6dsm_t *lsm6dsm)
{
	float a_res;

	switch(lsm6dsm->init->ascale)
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

	switch(lsm6dsm->init->gscale)
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

void lsm6dsm_reset(lsm6dsm_t *lsm6dsm)
{
	// reset device
	uint8_t temp;

	lsm6dsm_read_byte(LSM6DSM_CTRL3_C, &temp);
	lsm6dsm_write_byte(LSM6DSM_CTRL3_C, temp | 0x01); // Set bit 0 to 1 to reset LSM6DSM
	lsm6dsm->delay_func(100); // Wait for all registers to reset
}

void lsm6dsm_self_test(lsm6dsm_t *lsm6dsm)
{
	int16_t temp[7] = { 0, 0, 0, 0, 0, 0, 0 };
	int16_t accelPTest[3] = { 0, 0, 0 };
	int16_t accelNTest[3] = { 0, 0, 0 };
	int16_t gyroPTest[3] = { 0, 0, 0 };
	int16_t gyroNTest[3] = { 0, 0, 0 };
	int16_t accelNom[3] = { 0, 0, 0 };
	int16_t gyroNom[3] = { 0, 0, 0 };

	lsm6dsm_read_data(lsm6dsm, temp);
	accelNom[0] = temp[4];
	accelNom[1] = temp[5];
	accelNom[2] = temp[6];
	gyroNom[0] = temp[1];
	gyroNom[1] = temp[2];
	gyroNom[2] = temp[3];

	lsm6dsm_write_byte(LSM6DSM_CTRL5_C, 0x01); // positive accel self test
	lsm6dsm->delay_func(100); // let accel respond
	lsm6dsm_read_data(lsm6dsm, temp);
	accelPTest[0] = temp[4];
	accelPTest[1] = temp[5];
	accelPTest[2] = temp[6];

	lsm6dsm_write_byte(LSM6DSM_CTRL5_C, 0x03); // negative accel self test
	lsm6dsm->delay_func(100); // let accel respond
	lsm6dsm_read_data(lsm6dsm, temp);
	accelNTest[0] = temp[4];
	accelNTest[1] = temp[5];
	accelNTest[2] = temp[6];

	lsm6dsm_write_byte(LSM6DSM_CTRL5_C, 0x04); // positive gyro self test
	lsm6dsm->delay_func(100); // let gyro respond
	lsm6dsm_read_data(lsm6dsm, temp);
	gyroPTest[0] = temp[1];
	gyroPTest[1] = temp[2];
	gyroPTest[2] = temp[3];

	lsm6dsm_write_byte(LSM6DSM_CTRL5_C, 0x0C); // negative gyro self test
	lsm6dsm->delay_func(100); // let gyro respond
	lsm6dsm_read_data(lsm6dsm, temp);
	gyroNTest[0] = temp[1];
	gyroNTest[1] = temp[2];
	gyroNTest[2] = temp[3];

	lsm6dsm_write_byte(LSM6DSM_CTRL5_C, 0x00); // normal mode
	lsm6dsm->delay_func(100); // let accel and gyro respond

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
	lsm6dsm->delay_func(2000);
}

void lsm6dsm_offset_bias(lsm6dsm_t *lsm6dsm, float *dest1, float *dest2)
{
	int16_t temp[7] = { 0, 0, 0, 0, 0, 0, 0 };
	int32_t sum[7] = { 0, 0, 0, 0, 0, 0, 0 };
	float a_res = lsm6dsm_ares_get(lsm6dsm);
	float g_res = lsm6dsm_gres_get(lsm6dsm);

	/* Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!"); */
	lsm6dsm->delay_func(4000);

	for(int ii = 0; ii < 128; ii++)
	{
		lsm6dsm_read_data(lsm6dsm, temp);
		sum[1] += temp[1];
		sum[2] += temp[2];
		sum[3] += temp[3];
		sum[4] += temp[4];
		sum[5] += temp[5];
		sum[6] += temp[6];
		lsm6dsm->delay_func(50);
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

static void lsm6dsm_read_data(lsm6dsm_t *lsm6dsm, int16_t *destination)
{
	uint8_t data[14];  // x/y/z accel register data stored here

	lsm6dsm_read(LSM6DSM_OUT_TEMP_L, data, 14); // Read the 14 raw data registers into data array
	destination[0] = ((int16_t) data[1] << 8) | data[0]; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t) data[3] << 8) | data[2];
	destination[2] = ((int16_t) data[5] << 8) | data[4];
	destination[3] = ((int16_t) data[7] << 8) | data[6];
	destination[4] = ((int16_t) data[9] << 8) | data[8];
	destination[5] = ((int16_t) data[11] << 8) | data[10];
	destination[6] = ((int16_t) data[13] << 8) | data[12];
}
#endif
