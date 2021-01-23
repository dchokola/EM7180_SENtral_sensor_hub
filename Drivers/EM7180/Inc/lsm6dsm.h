/*
 * lsm6dsm.h
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

#ifndef LSM6DSM_h
#define LSM6DSM_h

/* Includes */
#include <stdint.h>

/* Definitions */
#define LSM6DSM_OK       (0)
#define LSM6DSM_BAD_ARG  (1 << 0)
#define LSM6DSM_BAD_COMM (1 << 1)
#define AFS_2G       0x00
#define AFS_4G       0x02
#define AFS_8G       0x03
#define AFS_16G      0x01
#define GFS_250DPS   0x00
#define GFS_500DPS   0x01
#define GFS_1000DPS  0x02
#define GFS_2000DPS  0x03
#define AODR_12_5Hz  0x01  // same for accel and gyro in normal mode
#define AODR_26Hz    0x02
#define AODR_52Hz    0x03
#define AODR_104Hz   0x04
#define AODR_208Hz   0x05
#define AODR_416Hz   0x06
#define AODR_833Hz   0x07
#define AODR_1660Hz  0x08
#define AODR_3330Hz  0x09
#define AODR_6660Hz  0x0A
#define GODR_12_5Hz  0x01   
#define GODR_26Hz    0x02
#define GODR_52Hz    0x03
#define GODR_104Hz   0x04
#define GODR_208Hz   0x05
#define GODR_416Hz   0x06
#define GODR_833Hz   0x07
#define GODR_1660Hz  0x08
#define GODR_3330Hz  0x09
#define GODR_6660Hz  0x0A

/* Data Structures */
typedef int32_t lsm6dsm_status_t;
typedef struct lsm6dsm_init_s
{
	uint8_t ascale;
	uint8_t gscale;
	uint8_t a_odr;
	uint8_t g_odr;
} lsm6dsm_init_t;
typedef struct lsm6dsm_s
{
	lsm6dsm_init_t *init;
	uint8_t i2c_addr;
	delay_func_t delay_func;
	i2c_read_func_t i2c_read_func;
	i2c_write_func_t i2c_write_func;
} lsm6dsm_t;

/* Function Prototypes */
lsm6dsm_status_t lsm6dsm_init(lsm6dsm_t *lsm6dsm, lsm6dsm_init_t *init);
void lsm6dsm_set_delay_cb(lsm6dsm_t *lsm6dsm, delay_func_t delay_func);
void lsm6dsm_set_i2c_cbs(lsm6dsm_t *lsm6dsm, i2c_read_func_t i2c_read_func,
                         i2c_write_func_t i2c_write_func, uint8_t dev_addr);
lsm6dsm_status_t lsm6dsm_config(lsm6dsm_t *lsm6dsm);

#endif
