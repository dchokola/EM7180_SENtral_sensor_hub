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

#ifndef LIS2MDL_h
#define LIS2MDL_h

/* Includes */
#include <imu_common.h>
#include <stdint.h>

/* Definitions */
#define LIS2MDL_OK       (0)
#define LIS2MDL_BAD_ARG  (1 << 0)
#define LIS2MDL_BAD_COMM (1 << 1)
#define MODR_10Hz   0x00
#define MODR_20Hz   0x01
#define MODR_50Hz   0x02
#define MODR_100Hz  0x03
#define MFS_14BITS  0  // 0.6 mG per LSB
#define MFS_16BITS  1  // 0.15 mG per LSB

/* Data Structures */
typedef int32_t lis2mdl_status_t;
typedef struct lis2mdl_init_s
{
	uint8_t m_odr;
} lis2mdl_init_t;
typedef struct lis2mdl_s
{
	lis2mdl_init_t *init;
	uint8_t i2c_addr;
	delay_func_t delay_func;
	i2c_read_func_t i2c_read_func;
	i2c_write_func_t i2c_write_func;
} lis2mdl_t;

/* Function Prototypes */
lis2mdl_status_t lis2mdl_init(lis2mdl_t *lis2mdl, lis2mdl_init_t *init);
void lis2mdl_set_delay_cb(lis2mdl_t *lis2mdl, delay_func_t delay_func);
void lis2mdl_set_i2c_cbs(lis2mdl_t *lis2mdl, i2c_read_func_t i2c_read_func,
                         i2c_write_func_t i2c_write_func, uint8_t dev_addr);
lis2mdl_status_t lis2mdl_config(lis2mdl_t *lis2mdl);

#endif
