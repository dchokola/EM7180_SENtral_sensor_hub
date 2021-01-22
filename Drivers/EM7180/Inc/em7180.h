/*
 * em7180.h
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

#ifndef EM7180_H_
#define EM7180_H_

/* Includes */
#include <stdint.h>
#include "em7180_common.h"

/* Definitions */
#define EM7180_OK       (0)
#define EM7180_BAD_ARG  (1 << 0)
#define EM7180_BAD_COMM (1 << 1)

/* Data Structures */
typedef int32_t em7180_status_t;
typedef struct em7180_init_s
{
	uint16_t acc_fs;
	uint16_t gyro_fs;
	uint16_t mag_fs;
	uint8_t q_rate_div;
	uint8_t mag_rate;
	uint8_t acc_rate;
	uint8_t gyro_rate;
	uint8_t baro_rate;
} em7180_init_t;
typedef struct em7180_s
{
	em7180_init_t *init;
	uint8_t i2c_addr;
	delay_func_t delay_func;
	i2c_read_func_t i2c_read_func;
	i2c_write_func_t i2c_write_func;
} em7180_t;

/* Function Prototypes */
em7180_status_t em7180_init(em7180_t *em7180, em7180_init_t *init);
void em7180_set_delay_cb(em7180_t *em7180, delay_func_t delay_func);
void em7180_set_i2c_cbs(em7180_t *em7180, i2c_read_func_t i2c_read_func,
                        i2c_write_func_t i2c_write_func, uint8_t dev_addr);
em7180_status_t em7180_config(em7180_t *em7180);
void em7180_reset(em7180_t *em7180);
void em7180_passthrough_enable(em7180_t *em7180);
void em7180_passthrough_disable(em7180_t *em7180);
em7180_status_t em7180_quatdata_get(em7180_t *em7180, float *destination);
em7180_status_t em7180_acceldata_get(em7180_t *em7180, int16_t *destination);
em7180_status_t em7180_gyrodata_get(em7180_t *em7180, int16_t *destination);
em7180_status_t em7180_magdata_get(em7180_t *em7180, int16_t *destination);
em7180_status_t em7180_barodata_get(em7180_t *em7180, int16_t *destination);
em7180_status_t em7180_tempdata_get(em7180_t *em7180, int16_t *destination);

#endif /* EM7180_H_ */
