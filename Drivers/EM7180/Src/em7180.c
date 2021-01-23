/*
 * em7180.c
 *
 *  Created on: Jan 18, 2021
 *      Author: Daniel Peter Chokola
 *
 *  Adapted From:
 *      EM7180_LSM6DSM_LIS2MDL_LPS22HB_Butterfly
 *      by: Kris Winer
 *      06/29/2017 Copyright Tlera Corporation
 *
 *  Library may be used freely and without limit with attribution.
 */

/* Includes */
#include <stdint.h>
#include <stddef.h>
#include "em7180_common.h"
#include "em7180.h"

/* Definitions */
/*
 * EM7180 SENtral register map
 * see http://www.emdeveloper.com/downloads/7180/EMSentral_EM7180_Register_Map_v1_3.pdf
 */
#define EM7180_QX                 0x00 /* this is a 32-bit normalized floating point number read from registers 0x00-03 */
#define EM7180_QY                 0x04 /* this is a 32-bit normalized floating point number read from registers 0x04-07 */
#define EM7180_QZ                 0x08 /* this is a 32-bit normalized floating point number read from registers 0x08-0B */
#define EM7180_QW                 0x0C /* this is a 32-bit normalized floating point number read from registers 0x0C-0F */
#define EM7180_QTIME              0x10 /* uint16_t from registers 0x10-11 */
#define EM7180_MX                 0x12 /*  int16_t from registers 0x12-13 */
#define EM7180_MY                 0x14 /*  int16_t from registers 0x14-15 */
#define EM7180_MZ                 0x16 /*  int16_t from registers 0x16-17 */
#define EM7180_MTIME              0x18 /* uint16_t from registers 0x18-19 */
#define EM7180_AX                 0x1A /*  int16_t from registers 0x1A-1B */
#define EM7180_AY                 0x1C /*  int16_t from registers 0x1C-1D */
#define EM7180_AZ                 0x1E /*  int16_t from registers 0x1E-1F */
#define EM7180_ATIME              0x20 /* uint16_t from registers 0x20-21 */
#define EM7180_GX                 0x22 /*  int16_t from registers 0x22-23 */
#define EM7180_GY                 0x24 /*  int16_t from registers 0x24-25 */
#define EM7180_GZ                 0x26 /*  int16_t from registers 0x26-27 */
#define EM7180_GTIME              0x28 /* uint16_t from registers 0x28-29 */
#define EM7180_Baro               0x2A /* start of two-byte MS5637 pressure data, 16-bit signed interger */
#define EM7180_BaroTIME           0x2C /* start of two-byte MS5637 pressure timestamp, 16-bit unsigned */
#define EM7180_Temp               0x2E /* start of two-byte MS5637 temperature data, 16-bit signed interger */
#define EM7180_TempTIME           0x30 /* start of two-byte MS5637 temperature timestamp, 16-bit unsigned */
#define EM7180_QRateDivisor       0x32 /* uint8_t */
#define EM7180_EnableEvents       0x33
#define EM7180_HostControl        0x34
#define EM7180_EventStatus        0x35
#define EM7180_SensorStatus       0x36
#define EM7180_SentralStatus      0x37
#define EM7180_AlgorithmStatus    0x38
#define EM7180_FeatureFlags       0x39
#define EM7180_ParamAcknowledge   0x3A
#define EM7180_SavedParamByte0    0x3B
#define EM7180_SavedParamByte1    0x3C
#define EM7180_SavedParamByte2    0x3D
#define EM7180_SavedParamByte3    0x3E
#define EM7180_ActualMagRate      0x45
#define EM7180_ActualAccelRate    0x46
#define EM7180_ActualGyroRate     0x47
#define EM7180_ActualBaroRate     0x48
#define EM7180_ActualTempRate     0x49
#define EM7180_ErrorRegister      0x50
#define EM7180_AlgorithmControl   0x54
#define EM7180_MagRate            0x55
#define EM7180_AccelRate          0x56
#define EM7180_GyroRate           0x57
#define EM7180_BaroRate           0x58
#define EM7180_TempRate           0x59
#define EM7180_LoadParamByte0     0x60
#define EM7180_LoadParamByte1     0x61
#define EM7180_LoadParamByte2     0x62
#define EM7180_LoadParamByte3     0x63
#define EM7180_ParamRequest       0x64
#define EM7180_ROMVersion1        0x70
#define EM7180_ROMVersion2        0x71
#define EM7180_RAMVersion1        0x72
#define EM7180_RAMVersion2        0x73
#define EM7180_ProductID          0x90
#define EM7180_RevisionID         0x91
#define EM7180_RunStatus          0x92
#define EM7180_UploadAddress      0x94 /* uint16_t registers 0x94 (MSB)-5(LSB) */
#define EM7180_UploadData         0x96
#define EM7180_CRCHost            0x97 /* uint32_t from registers 0x97-9A */
#define EM7180_ResetRequest       0x9B
#define EM7180_PassThroughStatus  0x9E
#define EM7180_PassThroughControl 0xA0

/* Macros */
#define em7180_read_byte(addr, byte) i2c_read_byte((em7180->i2c_read_func), (em7180->i2c_addr), (addr), (byte))
#define em7180_write_byte(addr, byte) i2c_write_byte((em7180->i2c_write_func), (em7180->i2c_addr), (addr), (byte))
#define em7180_read(addr, data, len) i2c_read((em7180->i2c_read_func), (em7180->i2c_addr), (addr), (data), (len))
#define em7180_write(addr, data, len) i2c_write((em7180->i2c_write_func), (em7180->i2c_addr), (addr), (data), (len))
/* Data Structures */

/* Private Global Variables */

/* Function Prototypes */
static float uint32_reg_to_float(uint8_t *buf);

/* Function Definitions */
em7180_status_t em7180_init(em7180_t *em7180, em7180_init_t *init)
{
	int8_t *ptr = (int8_t*) em7180;
	size_t i;

	return_val_if_fail(em7180, EM7180_BAD_ARG);
	return_val_if_fail(init, EM7180_BAD_ARG);

	/* zero em7180_t struct */
	for(i = 0; i < sizeof(em7180_t); i++)
	{
		*ptr++ = 0;
	}

	em7180->init = init;

	return EM7180_OK;
}

void em7180_set_delay_cb(em7180_t *em7180, delay_func_t delay_func)
{
	return_if_fail(em7180);

	em7180->delay_func = delay_func;
}

void em7180_set_i2c_cbs(em7180_t *em7180, i2c_read_func_t i2c_read_func,
                        i2c_write_func_t i2c_write_func, uint8_t dev_addr)
{
	return_if_fail(em7180);

	em7180->i2c_read_func = i2c_read_func;
	em7180->i2c_write_func = i2c_write_func;
	em7180->i2c_addr = dev_addr;
}

em7180_status_t em7180_config(em7180_t *em7180)
{
	uint8_t resp;
	int32_t ret = 0;

	/* wait for EM7180 to enter unprogrammed or initialized state */
	do
	{
		em7180_read_byte(EM7180_SentralStatus, &resp);
	}
	while(!(resp & 0x08));

	/* configure EM7180 to accept initial register set-up */
	ret |= em7180_write_byte(EM7180_HostControl, 0x00); /* put EM7180 in initialized state to configure registers */
	ret |= em7180_write_byte(EM7180_PassThroughControl, 0x00); /* make sure pass through mode is off */
	ret |= em7180_write_byte(EM7180_HostControl, 0x01); /* force initialize */
	ret |= em7180_write_byte(EM7180_HostControl, 0x00); /* put EM7180 in initialized state to configure registers */

	/* set output data rates */
	ret |= em7180_write_byte(EM7180_MagRate, em7180->init->mag_rate);
	ret |= em7180_write_byte(EM7180_AccelRate, em7180->init->acc_rate);
	ret |= em7180_write_byte(EM7180_GyroRate, em7180->init->gyro_rate);
	ret |= em7180_write_byte(EM7180_BaroRate, 0x80 | em7180->init->baro_rate);
	ret |= em7180_write_byte(EM7180_QRateDivisor, em7180->init->q_rate_div);

	/* configure operating mode */
	ret |= em7180_write_byte(EM7180_AlgorithmControl, 0x00);
	/* enable interrupts for CPUReset, Error, QuaternionResult, MagResult, AccelResult, GyroResult */
	ret |= em7180_write_byte(EM7180_EnableEvents, 0x07);
	/* run EM7180 */
	ret |= em7180_write_byte(EM7180_HostControl, 0x01);

	return ret ? EM7180_BAD_COMM : EM7180_OK;
}

void em7180_reset(em7180_t *em7180)
{
	/* reset EM7180 */
	em7180_write_byte(EM7180_ResetRequest, 0x01);
}

void em7180_passthrough_enable(em7180_t *em7180)
{
	/* put EM7180 in standby mode */
	em7180_write_byte(EM7180_AlgorithmControl, 0x01);
	/* put EM7180 in passthrough mode */
	em7180_write_byte(EM7180_PassThroughControl, 0x01);
}

void em7180_passthrough_disable(em7180_t *em7180)
{
	/* put EM7180 in standby mode */
	em7180_write_byte(EM7180_PassThroughControl, 0x00);
	/* put EM7180 in normal operation */
	em7180_write_byte(EM7180_AlgorithmControl, 0x00);
}

em7180_status_t em7180_quatdata_get(em7180_t *em7180, float *destination)
{
	int32_t ret = 0;
	uint8_t data[16];

	ret |= em7180_read(EM7180_QX, data, 16);
	/* EM7180 quaternion is xi + yj + zk + s */
	destination[0] = uint32_reg_to_float(&data[12]);
	destination[1] = uint32_reg_to_float(&data[0]);
	destination[2] = uint32_reg_to_float(&data[4]);
	destination[3] = uint32_reg_to_float(&data[8]);

	return ret ? EM7180_BAD_COMM : EM7180_OK;
}

em7180_status_t em7180_acceldata_get(em7180_t *em7180, int16_t *destination)
{
	int32_t ret = 0;
	uint8_t data[6];

	ret |= em7180_read(EM7180_AX, data, 6);
	destination[0] = (int16_t) (((int16_t) data[1] << 8) | data[0]);
	destination[1] = (int16_t) (((int16_t) data[3] << 8) | data[2]);
	destination[2] = (int16_t) (((int16_t) data[5] << 8) | data[4]);

	return ret ? EM7180_BAD_COMM : EM7180_OK;
}

em7180_status_t em7180_gyrodata_get(em7180_t *em7180, int16_t *destination)
{
	int32_t ret = 0;
	uint8_t data[6];

	ret |= em7180_read(EM7180_GX, data, 6);
	destination[0] = (int16_t) (((int16_t) data[1] << 8) | data[0]);
	destination[1] = (int16_t) (((int16_t) data[3] << 8) | data[2]);
	destination[2] = (int16_t) (((int16_t) data[5] << 8) | data[4]);

	return ret ? EM7180_BAD_COMM : EM7180_OK;
}

em7180_status_t em7180_magdata_get(em7180_t *em7180, int16_t *destination)
{
	int32_t ret = 0;
	uint8_t data[6];

	ret |= em7180_read(EM7180_MX, data, 6);
	destination[0] = (int16_t) (((int16_t) data[1] << 8) | data[0]);
	destination[1] = (int16_t) (((int16_t) data[3] << 8) | data[2]);
	destination[2] = (int16_t) (((int16_t) data[5] << 8) | data[4]);

	return ret ? EM7180_BAD_COMM : EM7180_OK;
}

em7180_status_t em7180_barodata_get(em7180_t *em7180, int16_t *destination)
{
	int32_t ret = 0;
	uint8_t data[2];

	ret |= em7180_read(EM7180_Baro, data, 2);

	*destination = (((int16_t) data[1] << 8) | data[0]);

	return ret ? EM7180_BAD_COMM : EM7180_OK;
}

em7180_status_t em7180_tempdata_get(em7180_t *em7180, int16_t *destination)
{
	int32_t ret = 0;
	uint8_t data[2];

	ret |= em7180_read(EM7180_Temp, data, 2);

	*destination = (((int16_t) data[1] << 8) | data[0]);

	return ret ? EM7180_BAD_COMM : EM7180_OK;
}

/* FIXME: haven't explored the usage/usefulness of these yet: */
#if(0)
uint8_t em7180_status(em7180_t *em7180)
{
	// Check event status register, way to check data ready by polling rather than interrupt
	uint8_t c;

	em7180_read_byte(EM7180_EventStatus, &c); // reading clears the register and interrupt

	return c;
}

uint8_t em7180_errors(em7180_t *em7180)
{
	uint8_t c;

	em7180_read_byte(EM7180_ErrorRegister, &c); // check error register

	return c;
}

void em7180_gyro_set_fs(em7180_t *em7180, uint16_t gyro_fs)
{
	uint8_t resp;

	em7180_write_byte(EM7180_LoadParamByte0, gyro_fs & (0xFF)); // Gyro LSB
	em7180_write_byte(EM7180_LoadParamByte1, (gyro_fs >> 8) & (0xFF)); // Gyro MSB
	em7180_write_byte(EM7180_LoadParamByte2, 0x00); // Unused
	em7180_write_byte(EM7180_LoadParamByte3, 0x00); // Unused
	em7180_write_byte(EM7180_ParamRequest, 0xcb); // Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a parameter write process
	em7180_write_byte(EM7180_AlgorithmControl, 0x80); // Request parameter transfer procedure
	/* Check the parameter acknowledge register and loop until the result matches parameter request byte */
	do
	{
		em7180_read_byte(EM7180_ParamAcknowledge, &resp);
	}
	while(resp != 0xCB);
	em7180_write_byte(EM7180_ParamRequest, 0x00); // Parameter request = 0 to end parameter transfer process
	em7180_write_byte(EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void em7180_mag_acc_set_fs(em7180_t *em7180, uint16_t mag_fs, uint16_t acc_fs)
{
	uint8_t resp;

	em7180_write_byte(EM7180_LoadParamByte0, mag_fs & (0xFF)); // Mag LSB
	em7180_write_byte(EM7180_LoadParamByte1, (mag_fs >> 8) & (0xFF)); // Mag MSB
	em7180_write_byte(EM7180_LoadParamByte2, acc_fs & (0xFF)); // Acc LSB
	em7180_write_byte(EM7180_LoadParamByte3, (acc_fs >> 8) & (0xFF)); // Acc MSB
	em7180_write_byte(EM7180_ParamRequest, 0xca); // Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a parameter write process
	em7180_write_byte(EM7180_AlgorithmControl, 0x80); // Request parameter transfer procedure
	/* Check the parameter acknowledge register and loop until the result matches parameter request byte */
	do
	{
		em7180_read_byte(EM7180_ParamAcknowledge, &resp);
	}
	while(!(resp == 0xCA));
	em7180_write_byte(EM7180_ParamRequest, 0x00); // Parameter request = 0 to end parameter transfer process
	em7180_write_byte(EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void em7180_set_integer_param(em7180_t *em7180, uint8_t param,
                              uint32_t param_val)
{
	uint8_t resp;

	em7180_write_byte(EM7180_LoadParamByte0, param_val & (0xFF)); // Param LSB
	em7180_write_byte(EM7180_LoadParamByte1, (param_val >> 8) & (0xFF));
	em7180_write_byte(EM7180_LoadParamByte2, (param_val >> 16) & (0xFF));
	em7180_write_byte(EM7180_LoadParamByte3, (param_val >> 24) & (0xFF)); // Param MSB
	em7180_write_byte(EM7180_ParamRequest, param | 0x80); // Parameter is the decimal value with the MSB set high to indicate a parameter write process
	em7180_write_byte(EM7180_AlgorithmControl, 0x80); // Request parameter transfer procedure
	/* Check the parameter acknowledge register and loop until the result matches parameter request byte */
	do
	{
		em7180_read_byte(EM7180_ParamAcknowledge, &resp);
	}
	while(resp != param);
	em7180_write_byte(EM7180_ParamRequest, 0x00); // Parameter request = 0 to end parameter transfer process
	em7180_write_byte(EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void em7180_param_set_float(em7180_t *em7180, uint8_t param, float param_val)
{
	uint8_t resp;
	uint8_t bytes[4];

	float_to_bytes(param_val, bytes);
	em7180_write_byte(EM7180_LoadParamByte0, bytes[0]); // Param LSB
	em7180_write_byte(EM7180_LoadParamByte1, bytes[1]);
	em7180_write_byte(EM7180_LoadParamByte2, bytes[2]);
	em7180_write_byte(EM7180_LoadParamByte3, bytes[3]); // Param MSB
	em7180_write_byte(EM7180_ParamRequest, param | 0x80); // Parameter is the decimal value with the MSB set high to indicate a parameter write process
	em7180_write_byte(EM7180_AlgorithmControl, 0x80); // Request parameter transfer procedure
	/* Check the parameter acknowledge register and loop until the result matches parameter request byte */
	do
	{
		em7180_read_byte(EM7180_ParamAcknowledge, &resp);
	}
	while(resp != param);
	em7180_write_byte(EM7180_ParamRequest, 0x00); // Parameter request = 0 to end parameter transfer process
	em7180_write_byte(EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

float em7180_mres_get(uint8_t Mscale)
{
	float m_res;

	switch(Mscale)
	{
		/*
		 * Possible magnetometer scales (and their register bit settings) are:
		 * 14 bit resolution (0) and 16 bit resolution (1)
		 */
		case MFS_14BITS:
			m_res = 10. * 4912. / 8190.; // Proper scale to return milliGauss
			break;
		case MFS_16BITS:
			m_res = 10. * 4912. / 32760.0; // Proper scale to return milliGauss
			break;
		default:
			m_res = NAN;
			break;
	}

	return m_res;
}

float em7180_gres_get(uint8_t gscale)
{
	float g_res;

	switch(gscale)
	{
		/*
		 * Possible gyro scales (and their register bit settings) are:
		 * 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
		 * Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
		 */
		case GFS_250DPS:
			g_res = 250.0 / 32768.0;
			break;
		case GFS_500DPS:
			g_res = 500.0 / 32768.0;
			break;
		case GFS_1000DPS:
			g_res = 1000.0 / 32768.0;
			break;
		case GFS_2000DPS:
			g_res = 2000.0 / 32768.0;
			break;
		default:
			g_res = NAN;
			break;
	}

	return g_res;
}

float em7180_ares_get(uint8_t ascale)
{
	float a_res;

	switch(ascale)
	{
		/*
		 * Possible accelerometer scales (and their register bit settings) are:
		 * 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs (11).
		 * Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
		 */
		case AFS_2G:
			a_res = 2.0 / 32768.0;
			break;
		case AFS_4G:
			a_res = 4.0 / 32768.0;
			break;
		case AFS_8G:
			a_res = 8.0 / 32768.0;
			break;
		case AFS_16G:
			a_res = 16.0 / 32768.0;
			break;
		default:
			a_res = NAN;
			break;
	}

	return a_res;
}
#endif

static float uint32_reg_to_float(uint8_t *buf)
{
	union
	{
		uint32_t ui32;
		float f;
	} u;

	u.ui32 = (((uint32_t) buf[0]) + (((uint32_t) buf[1]) << 8)
	          + (((uint32_t) buf[2]) << 16) + (((uint32_t) buf[3]) << 24));

	return u.f;
}
