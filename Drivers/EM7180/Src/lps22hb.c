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
#include "lps22hb.h"

/* Private Global Variables */
static uint8_t _intPin;

/* Function Prototypes */
static void lps22h_write_byte(uint8_t address, uint8_t subAddress, uint8_t data);
static uint8_t lps22h_read_byte(uint8_t address, uint8_t subAddress);
static void lps22h_read(uint8_t address, uint8_t subAddress, uint8_t count,
                        uint8_t *dest);

/* Function Definitions */
lps22h_new(uint8_t pin)
{
	pinMode(pin, INPUT);
	_intPin = pin;
}

uint8_t lps22h_getChipID()
{
	// Read the WHO_AM_I register of the altimeter this is a good test of communication
	uint8_t temp = lps22h_read_byte(LPS22H_ADDRESS, LPS22H_WHOAMI); // Read WHO_AM_I register for LPS22H
	return temp;
}

uint8_t lps22h_status()
{
	// Read the status register of the altimeter  
	uint8_t temp = lps22h_read_byte(LPS22H_ADDRESS, LPS22H_STATUS);
	return temp;
}

int32_t lps22h_pressure_get()
{
	uint8_t rawData[3];  // 24-bit pressure register data stored here
	lps22h_read(LPS22H_ADDRESS, (LPS22H_PRESS_OUT_XL | 0x80), 3, &rawData[0]); // bit 7 must be one to read multiple bytes
	return (int32_t) ((int32_t) rawData[2] << 16 | (int32_t) rawData[1] << 8
	                  | rawData[0]);
}

int16_t lps22h_temp_get()
{
	uint8_t rawData[2];  // 16-bit pressure register data stored here
	lps22h_read(LPS22H_ADDRESS, (LPS22H_TEMP_OUT_L | 0x80), 2, &rawData[0]); // bit 7 must be one to read multiple bytes
	return (int16_t) ((int16_t) rawData[1] << 8 | rawData[0]);
}

void lps22h_init(uint8_t PODR)
{
	// set sample rate by setting bits 6:4 
	// enable low-pass filter by setting bit 3 to one
	// bit 2 == 0 means bandwidth is odr/9, bit 2 == 1 means bandwidth is odr/20
	// make sure data not updated during read by setting block data udate (bit 1) to 1
	lps22h_write_byte(LPS22H_ADDRESS, LPS22H_CTRL_REG1,
	                  PODR << 4 | 0x08 | 0x02);
	lps22h_write_byte(LPS22H_ADDRESS, LPS22H_CTRL_REG3, 0x04); // enable data ready as interrupt source
}

static void lps22h_write_byte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

static uint8_t lps22h_read_byte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data   
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (size_t) 1); // Read one uint8_t from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                  // Return data lps22h_read from slave register
}

static void lps22h_read(uint8_t address, uint8_t subAddress, uint8_t count,
                        uint8_t *dest)
{
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
	Wire.requestFrom(address, (size_t) count); // Read bytes from slave register address 
	while(Wire.available())
	{
		dest[i++] = Wire.lps22h_read();
	}         // Put lps22h_read results in the Rx buffer
}
