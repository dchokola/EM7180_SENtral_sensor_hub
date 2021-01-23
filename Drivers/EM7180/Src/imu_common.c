/*
 * em7180_common.c
 *
 *  Created on: Jan 22, 2021
 *      Author: dan
 */

/* Includes */
#include <imu_common.h>
#include <stdint.h>

/* Function Definitions */

int32_t i2c_read_byte(i2c_read_func_t read, uint16_t dev_addr, uint16_t addr,
                      uint8_t *byte)
{
	return read(dev_addr, addr, 1, byte, 1);
}

int32_t i2c_write_byte(i2c_write_func_t write, uint16_t dev_addr, uint16_t addr,
                       uint8_t byte)
{
	return write(dev_addr, addr, 1, &byte, 1);
}

int32_t i2c_read(i2c_read_func_t read, uint16_t dev_addr, uint16_t addr,
                 uint8_t *data, uint16_t len)
{
	return read(dev_addr, addr, 1, data, len);
}

int32_t i2c_write(i2c_write_func_t write, uint16_t dev_addr, uint16_t addr,
                  uint8_t *data, uint16_t len)
{
	return write(dev_addr, addr, 1, data, len);
}
