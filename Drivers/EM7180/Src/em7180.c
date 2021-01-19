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
#include <stdbool.h>
#include <math.h>
#include "em7180_common.h"
#include "em7180.h"
#include "i2c.h"

/* Data Structures */

/* Private Global Variables */

/* Function Prototypes */
static void em7180_float_to_bytes(float param_val, uint8_t *buf);
static void m24512dfm_write_byte(uint8_t device_address, uint8_t data_address1,
                                 uint8_t data_address2, uint8_t data);
static void m24512dfm_write(uint8_t device_address, uint8_t data_address1,
                            uint8_t data_address2, uint8_t count, uint8_t *dest);
static uint8_t m24512dfm_read_byte(uint8_t device_address,
                                   uint8_t data_address1, uint8_t data_address2);
static void m24512dfm_read(uint8_t device_address, uint8_t data_address1,
                           uint8_t data_address2, uint8_t count, uint8_t *dest);
static uint8_t em7180_read_byte(uint8_t address, uint8_t subAddress);
static void em7180_read(uint8_t address, uint8_t subAddress, uint8_t count,
                        uint8_t *dest);

/* Function Definitions */
void em7180_init(em7180_t *em7180, lsm6dsm_t *lsm6dsm, I2C_HandleTypeDef *hi2c1,
                 uint16_t acc_fs, uint16_t gyro_fs, uint16_t mag_fs,
                 uint8_t q_rate_div, uint8_t mag_rate, uint8_t acc_rate,
                 uint8_t gyro_rate, uint8_t baro_rate)
{
	if(!em7180)
	{
		return;
	}

	em7180->lsm6dsm = lsm6dsm;
	em7180->acc_fs = acc_fs;
	em7180->gyro_fs = gyro_fs;
	em7180->mag_fs = mag_fs;
	em7180->q_rate_div = q_rate_div;
	em7180->mag_rate = mag_rate;
	em7180->acc_rate = acc_rate;
	em7180->gyro_rate = gyro_rate;
	em7180->baro_rate = baro_rate;

	em7180_config(em7180);
}

void em7180_config(em7180_t *em7180)
{
	uint8_t param[4];
	uint8_t param_xfer;
	uint8_t runStatus;
	uint8_t algoStatus;
	uint8_t passthruStatus;
	uint8_t eventStatus;
	uint8_t sensorStatus;

	// Enter EM7180 initialized state
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_PassThruControl, 0x00); // make sure pass through mode is off
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // Force initialize
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers

	/* Legacy MPU6250 stuff, it seems
	 // Setup LPF bandwidth (BEFORE setting ODR's)
	 lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_ACC_LPF_BW, accBW); // accBW = 3 = 41Hz
	 lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_GYRO_LPF_BW, gyroBW); // gyroBW = 3 = 41Hz */
	// Set accel/gyro/mag desired ODR rates
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_QRateDivisor, em7180->q_rate_div); // quat rate = gyroRt/(1 QRTDiv)
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_MagRate, em7180->mag_rate); // 0x64 = 100 Hz
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_AccelRate, em7180->acc_rate); // 200/10 Hz, 0x14 = 200 Hz
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_GyroRate, em7180->gyro_rate); // 200/10 Hz, 0x14 = 200 Hz
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_BaroRate,
	                   0x80 | em7180->baro_rate); // set enable bit and set Baro rate to 25 Hz, rate = baroRt/2, 0x32 = 25 Hz

	// Configure operating mode
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data
	// Enable interrupt to host upon certain events
	// choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
	// new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_EnableEvents, 0x07);
	// Enable EM7180 run mode
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode
	HAL_Delay(100);

	// EM7180 parameter adjustments
	/* Serial.println("Beginning Parameter Adjustments"); */

	// Disable stillness mode for balancing robot application
	em7180_set_integer_param(0x49, 0x00);

	// Write desired sensor full scale ranges to the EM7180
	em7180_mag_acc_set_fs(em7180->mag_fs, em7180->acc_fs); // 1000 uT == 0x3E8, 8 g == 0x08
	em7180_gyro_set_fs(em7180->gyro_fs); // 2000 dps == 0x7D0

	// Read EM7180 status
	runStatus = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_RunStatus);
	if(runStatus & 0x01)
	{
		/* Serial.println(" EM7180 run status = normal mode"); */
	}
	algoStatus = lsm6dsm_read_byte(EM7180_ADDRESS,
	EM7180_AlgorithmStatus);
	if(algoStatus & 0x01)
	{
		/* Serial.println(" EM7180 standby status"); */
	}
	if(algoStatus & 0x02)
	{
		/* Serial.println(" EM7180 algorithm slow"); */
	}
	if(algoStatus & 0x04)
	{
		/* Serial.println(" EM7180 in stillness mode"); */
	}
	if(algoStatus & 0x08)
	{
		/* Serial.println(" EM7180 mag calibration completed"); */
	}
	if(algoStatus & 0x10)
	{
		/* Serial.println(" EM7180 magnetic anomaly detected"); */
	}
	if(algoStatus & 0x20)
	{
		/* Serial.println(" EM7180 unreliable sensor data"); */
	}
	passthruStatus = lsm6dsm_read_byte(EM7180_ADDRESS,
	EM7180_PassThruStatus);
	if(passthruStatus & 0x01)
	{
		/* Serial.print(" EM7180 in passthru mode!"); */
	}
	eventStatus = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_EventStatus);
	if(eventStatus & 0x01)
	{
		/* Serial.println(" EM7180 CPU reset"); */
	}
	if(eventStatus & 0x02)
	{
		/* Serial.println(" EM7180 Error"); */
	}
	if(eventStatus & 0x04)
	{
		/* Serial.println(" EM7180 new quaternion result"); */
	}
	if(eventStatus & 0x08)
	{
		/* Serial.println(" EM7180 new mag result"); */
	}
	if(eventStatus & 0x10)
	{
		/* Serial.println(" EM7180 new accel result"); */
	}
	if(eventStatus & 0x20)
	{
		/* Serial.println(" EM7180 new gyro result"); */
	}

	HAL_Delay(1000); // give some time to read the screen

	// Check sensor status
	sensorStatus = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_SensorStatus);
	/* Serial.print(" EM7180 sensor status = "); */
	/* Serial.println(sensorStatus); */
	if(sensorStatus & 0x01)
	{
		/* Serial.print("Magnetometer not acknowledging!"); */
	}
	if(sensorStatus & 0x02)
	{
		/* Serial.print("Accelerometer not acknowledging!"); */
	}
	if(sensorStatus & 0x04)
	{
		/* Serial.print("Gyro not acknowledging!"); */
	}
	if(sensorStatus & 0x10)
	{
		/* Serial.print("Magnetometer ID not recognized!"); */
	}
	if(sensorStatus & 0x20)
	{
		/* Serial.print("Accelerometer ID not recognized!"); */
	}
	if(sensorStatus & 0x40)
	{
		/* Serial.print("Gyro ID not recognized!"); */
	}

	/* Serial.print("Actual MagRate = "); */
	/* Serial.print(lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ActualMagRate)); */
	/* Serial.println(" Hz"); */
	/* Serial.print("Actual AccelRate = "); */
	/* Serial.print(10 * lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ActualAccelRate)); */
	/* Serial.println(" Hz"); */
	/* Serial.print("Actual GyroRate = "); */
	/* Serial.print(10 * lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ActualGyroRate)); */
	/* Serial.println(" Hz"); */
	/* Serial.print("Actual BaroRate = "); */
	/* Serial.print(lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ActualBaroRate)); */
	/* Serial.println(" Hz"); */
}

void em7180_chip_id_get()
{
	// Read SENtral device information
	uint16_t ROM1 = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ROMVersion1);
	uint16_t ROM2 = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ROMVersion2);
	/* Serial.print("EM7180 ROM Version: 0x"); */
	/* Serial.print(ROM1, HEX); */
	/* Serial.println(ROM2, HEX); */
	/* Serial.println("Should be: 0xE609"); */
	uint16_t RAM1 = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_RAMVersion1);
	uint16_t RAM2 = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_RAMVersion2);
	/* Serial.print("EM7180 RAM Version: 0x"); */
	/* Serial.print(RAM1); */
	/* Serial.println(RAM2); */
	uint8_t PID = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ProductID);
	/* Serial.print("EM7180 ProductID: 0x"); */
	/* Serial.print(PID, HEX); */
	/* Serial.println(" Should be: 0x80"); */
	uint8_t RID = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_RevisionID);
	/* Serial.print("EM7180 RevisionID: 0x"); */
	/* Serial.print(RID, HEX); */
	/* Serial.println(" Should be: 0x02"); */
}

void em7180_load_fw_from_eeprom()
{
	// Check which sensors can be detected by the EM7180
	uint8_t featureflag = lsm6dsm_read_byte(EM7180_ADDRESS,
	EM7180_FeatureFlags);
	if(featureflag & 0x01)
	{
		/* Serial.println("A barometer is installed"); */
	}
	if(featureflag & 0x02)
	{
		/* Serial.println("A humidity sensor is installed"); */
	}
	if(featureflag & 0x04)
	{
		/* Serial.println("A temperature sensor is installed"); */
	}
	if(featureflag & 0x08)
	{
		/* Serial.println("A custom sensor is installed"); */
	}
	if(featureflag & 0x10)
	{
		/* Serial.println("A second custom sensor is installed"); */
	}
	if(featureflag & 0x20)
	{
		/* Serial.println("A third custom sensor is installed"); */
	}

	HAL_Delay(1000); // give some time to read the screen

	// Check SENtral status, make sure EEPROM upload of firmware was accomplished
	uint8_t STAT = (lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_SentralStatus)
	    & 0x01);
	if(lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)
	{
		/* Serial.println("EEPROM detected on the sensor bus!"); */
	}
	if(lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)
	{
		/* Serial.println("EEPROM uploaded config file!"); */
	}
	if(lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)
	{
		/* Serial.println("EEPROM CRC incorrect!"); */
	}
	if(lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)
	{
		/* Serial.println("EM7180 in initialized state!"); */
	}
	if(lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)
	{
		/* Serial.println("No EEPROM detected!"); */
	}
	int count = 0;
	while(!STAT)
	{
		lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_ResetRequest, 0x01);
		HAL_Delay(500);
		count++;
		STAT = (lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
		if(lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)
		{
			/* Serial.println("EEPROM detected on the sensor bus!"); */
		}
		if(lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)
		{
			/* Serial.println("EEPROM uploaded config file!"); */
		}
		if(lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)
		{
			/* Serial.println("EEPROM CRC incorrect!"); */
		}
		if(lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)
		{
			/* Serial.println("EM7180 in initialized state!"); */
		}
		if(lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)
		{
			/* Serial.println("No EEPROM detected!"); */
		}
		if(count > 10)
		{
			break;
		}
	}

	if(!(lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04))
	{
		/* Serial.println("EEPROM upload successful!"); */
	}
}

uint8_t em7180_status()
{
	// Check event status register, way to check data ready by polling rather than interrupt
	uint8_t c = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register and interrupt
	return c;
}

uint8_t em7180_errors()
{
	uint8_t c = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ErrorRegister); // check error register
	return c;
}

float em7180_uint32_reg_to_float(uint8_t *buf)
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

float em7180_int32_reg_to_float(uint8_t *buf)
{
	union
	{
		int32_t i32;
		float f;
	} u;

	u.i32 = (((int32_t) buf[0]) + (((int32_t) buf[1]) << 8)
	         + (((int32_t) buf[2]) << 16) + (((int32_t) buf[3]) << 24));
	return u.f;
}

static void em7180_float_to_bytes(float param_val, uint8_t *buf)
{
	union
	{
		float f;
		uint8_t u8[sizeof(float)];
	} u;

	u.f = param_val;
	for(uint8_t i = 0; i < sizeof(float); i++)
	{
		buf[i] = u.u8[i];
	}
	// Convert to LITTLE ENDIAN
	/* FIXME: What the hell? */
	for(uint8_t i = 0; i < sizeof(float); i++)
	{
		buf[i] = buf[(sizeof(float) - 1) - i];
	}
}

void em7180_gyro_set_fs(uint16_t gyro_fs)
{
	uint8_t bytes[4], STAT;
	bytes[0] = gyro_fs & (0xFF);
	bytes[1] = (gyro_fs >> 8) & (0xFF);
	bytes[2] = 0x00;
	bytes[3] = 0x00;
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); // Gyro LSB
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); // Gyro MSB
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); // Unused
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); // Unused
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCB); // Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a parameter write process
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer procedure
	STAT = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ParamAcknowledge); // Check the parameter acknowledge register and loop until the result matches parameter request byte
	while(!(STAT == 0xCB))
	{
		STAT = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
	}
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); // Parameter request = 0 to end parameter transfer process
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void em7180_mag_acc_set_fs(uint16_t mag_fs, uint16_t acc_fs)
{
	uint8_t bytes[4], STAT;
	bytes[0] = mag_fs & (0xFF);
	bytes[1] = (mag_fs >> 8) & (0xFF);
	bytes[2] = acc_fs & (0xFF);
	bytes[3] = (acc_fs >> 8) & (0xFF);
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); // Mag LSB
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); // Mag MSB
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); // Acc LSB
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); // Acc MSB
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCA); // Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer procedure
	STAT = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ParamAcknowledge); // Check the parameter acknowledge register and loop until the result matches parameter request byte
	while(!(STAT == 0xCA))
	{
		STAT = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
	}
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); // Parameter request = 0 to end parameter transfer process
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void em7180_set_integer_param(uint8_t param, uint32_t param_val)
{
	uint8_t bytes[4], STAT;
	bytes[0] = param_val & (0xFF);
	bytes[1] = (param_val >> 8) & (0xFF);
	bytes[2] = (param_val >> 16) & (0xFF);
	bytes[3] = (param_val >> 24) & (0xFF);
	param = param | 0x80; // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); // Param LSB
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); // Param MSB
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_ParamRequest, param);
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer procedure
	STAT = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ParamAcknowledge); // Check the parameter acknowledge register and loop until the result matches parameter request byte
	while(!(STAT == param))
	{
		STAT = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
	}
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); // Parameter request = 0 to end parameter transfer process
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void em7180_param_set_float(uint8_t param, float param_val)
{
	uint8_t bytes[4], STAT;
	em7180_float_to_bytes(param_val, &bytes[0]);
	param = param | 0x80; // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); // Param LSB
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); // Param MSB
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_ParamRequest, param);
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer procedure
	STAT = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ParamAcknowledge); // Check the parameter acknowledge register and loop until the result matches parameter request byte
	while(!(STAT == param))
	{
		STAT = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
	}
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); // Parameter request = 0 to end parameter transfer process
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void em7180_quatdata_get(float *destination)
{
	uint8_t rawData[16];  // x/y/z quaternion register data stored here
	em7180_read(EM7180_ADDRESS, EM7180_QX, 16, &rawData[0]); // Read the sixteen raw data registers into data array
	destination[1] = uint32_reg_to_float(&rawData[0]);
	destination[2] = uint32_reg_to_float(&rawData[4]);
	destination[3] = uint32_reg_to_float(&rawData[8]);
	destination[0] = uint32_reg_to_float(&rawData[12]); // SENtral stores quats as qx, qy, qz, q0!

}

void em7180_acceldata_get(int16_t *destination)
{
	uint8_t rawData[6];  // x/y/z accel register data stored here
	em7180_read(EM7180_ADDRESS, EM7180_AX, 6, &rawData[0]); // Read the six raw data registers into data array
	destination[0] = (int16_t) (((int16_t) rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t) (((int16_t) rawData[3] << 8) | rawData[2]);
	destination[2] = (int16_t) (((int16_t) rawData[5] << 8) | rawData[4]);
}

void em7180_gyrodata_get(int16_t *destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	em7180_read(EM7180_ADDRESS, EM7180_GX, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
	destination[0] = (int16_t) (((int16_t) rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t) (((int16_t) rawData[3] << 8) | rawData[2]);
	destination[2] = (int16_t) (((int16_t) rawData[5] << 8) | rawData[4]);
}

void em7180_magdata_get(int16_t *destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	em7180_read(EM7180_ADDRESS, EM7180_MX, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
	destination[0] = (int16_t) (((int16_t) rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t) (((int16_t) rawData[3] << 8) | rawData[2]);
	destination[2] = (int16_t) (((int16_t) rawData[5] << 8) | rawData[4]);
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

int16_t em7180_baro_get()
{
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	em7180_read(EM7180_ADDRESS, EM7180_Baro, 2, &rawData[0]); // Read the two raw data registers sequentially into data array
	return (int16_t) (((int16_t) rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
}

int16_t em7180_temp_get()
{
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	em7180_read(EM7180_ADDRESS, EM7180_Temp, 2, &rawData[0]); // Read the two raw data registers sequentially into data array
	return (int16_t) (((int16_t) rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
}

void em7180_passthrough()
{
	// First put SENtral in standby mode
	uint8_t c = lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_AlgorithmControl);
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_AlgorithmControl, c | 0x01);
	//  c = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
	/*	//  Serial.print("c = "); Serial.println(c); */
	// Verify standby status
	// if(readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus) & 0x01) {
	/* Serial.println("SENtral in standby mode"); */
	// Place SENtral in pass-through mode
	lsm6dsm_write_byte(EM7180_ADDRESS, EM7180_PassThruControl, 0x01);
	if(lsm6dsm_read_byte(EM7180_ADDRESS, EM7180_PassThruStatus) & 0x01)
	{
		/* Serial.println("SENtral in pass-through mode"); */
	}
	else
	{
		/* Serial.println("ERROR! SENtral not in pass-through mode!"); */
	}
}

// I2C communication with the M24512DFM EEPROM is a little different from I2C communication with the usual motion sensor
// since the address is defined by two bytes
static void m24512dfm_write_byte(uint8_t device_address, uint8_t data_address1,
                                 uint8_t data_address2, uint8_t data)
{
	uint8_t temp[2] = { data_address1, data_address2 };
	/* Wire.transfer(device_address, &temp[0], 2, NULL, 0); */
	/* Wire.transfer(device_address, &data, 1, NULL, 0); */
}

static void m24512dfm_write(uint8_t device_address, uint8_t data_address1,
                            uint8_t data_address2, uint8_t count, uint8_t *dest)
{
	if(count > 128)
	{
		count = 128;
		/* Serial.print("Page count cannot be more than 128 bytes!"); */
	}
	uint8_t temp[2] = { data_address1, data_address2 };
	/* Wire.transfer(device_address, &temp[0], 2, NULL, 0); */
	/* Wire.transfer(device_address, &dest[0], count, NULL, 0); */
}

static uint8_t m24512dfm_read_byte(uint8_t device_address,
                                   uint8_t data_address1, uint8_t data_address2)
{
	uint8_t data; // `data` will store the register data
	/* Wire.beginTransmission(device_address);         // Initialize the Tx buffer */
	/* Wire.write(data_address1);        // Put slave register address in Tx buffer */
	/* Wire.write(data_address2);        // Put slave register address in Tx buffer */
	/* Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive */
	/* Wire.requestFrom(device_address, 1); // Read one byte from slave register address */
	/*	data = Wire.read();                      // Fill Rx buffer with result */
	return data;                         // Return data read from slave register
}

static void m24512dfm_read(uint8_t device_address, uint8_t data_address1,
                           uint8_t data_address2, uint8_t count, uint8_t *dest)
{
	uint8_t temp[2] = { data_address1, data_address2 };
	/* Wire.transfer(device_address, &temp[0], 2, dest, count); */
}

// I2C read/write functions for the EM7180
void em7180_write_byte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	uint8_t temp[2];
	temp[0] = subAddress;
	temp[1] = data;
	/* Wire.transfer(address, &temp[0], 2, NULL, 0); */
}

static uint8_t em7180_read_byte(uint8_t address, uint8_t subAddress)
{
	uint8_t temp[1];
	/* Wire.transfer(address, &subAddress, 1, &temp[0], 1); */
	return temp[0];
}

static void em7180_read(uint8_t address, uint8_t subAddress, uint8_t count,
                        uint8_t *dest)
{
	/* Wire.transfer(address, &subAddress, 1, dest, count); */
}
