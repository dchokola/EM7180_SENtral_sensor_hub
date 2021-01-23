/*
 * lps22hb.h
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

#ifndef LPS22HB_h
#define LPS22HB_h

/* Includes */
#include <imu_common.h>
#include <stdint.h>

/* Definitions */
#define LPS22HB_OK       (0)
#define LPS22HB_BAD_ARG  (1 << 0)
#define LPS22HB_BAD_COMM (1 << 1)
/* altimeter output data rate */
#define    P_1shot  0x00
#define    P_1Hz    0x01
#define    P_10Hz   0x02
#define    P_25Hz   0x03
#define    P_50Hz   0x04
#define    P_75Hz   0x05

/* Data Structures */
typedef int32_t lps22hb_status_t;
typedef struct lps22hb_init_s
{
	uint8_t p_odr;
} lps22hb_init_t;
typedef struct lps22hb_s
{
	lps22hb_init_t *init;
	uint8_t i2c_addr;
	delay_func_t delay_func;
	i2c_read_func_t i2c_read_func;
	i2c_write_func_t i2c_write_func;
} lps22hb_t;

/* Function Prototypes */
lps22hb_status_t lps22hb_init(lps22hb_t *lps22hb, lps22hb_init_t *init);
void lps22hb_set_delay_cb(lps22hb_t *lps22hb, delay_func_t delay_func);
void lps22hb_set_i2c_cbs(lps22hb_t *lps22hb, i2c_read_func_t i2c_read_func,
                         i2c_write_func_t i2c_write_func, uint8_t dev_addr);
lps22hb_status_t lps22hb_config(lps22hb_t *lps22hb);
lps22hb_status_t lps22hb_status(lps22hb_t *lps22hb, uint8_t *status);
lps22hb_status_t lps22hb_chip_id_get(lps22hb_t *lps22hb, uint8_t *data);
lps22hb_status_t lps22hb_pressure_get(lps22hb_t *lps22hb, int32_t *data);
lps22hb_status_t lps22hb_temp_get(lps22hb_t *lps22hb, int16_t *data);

#endif
