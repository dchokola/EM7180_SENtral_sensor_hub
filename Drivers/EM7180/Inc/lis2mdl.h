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
#include <stdint.h>
#include "i2c.h"

/* Definitions */
//Register map for LIS2MDL'
// http://www.st.com/content/ccc/resource/technical/document/datasheet/group3/29/13/d1/e0/9a/4d/4f/30/DM00395193/files/DM00395193.pdf/jcr:content/translations/en.DM00395193.pdf
#define LIS2MDL_OFFSET_X_REG_L        0x45
#define LIS2MDL_OFFSET_X_REG_H        0x46
#define LIS2MDL_OFFSET_Y_REG_L        0x47
#define LIS2MDL_OFFSET_Y_REG_H        0x48
#define LIS2MDL_OFFSET_Z_REG_L        0x49
#define LIS2MDL_OFFSET_Z_REG_H        0x4A
#define LIS2MDL_WHO_AM_I              0x4F
#define LIS2MDL_CFG_REG_A             0x60
#define LIS2MDL_CFG_REG_B             0x61
#define LIS2MDL_CFG_REG_C             0x62
#define LIS2MDL_INT_CTRL_REG          0x63
#define LIS2MDL_INT_SOURCE_REG        0x64
#define LIS2MDL_INT_THS_L_REG         0x65
#define LIS2MDL_INT_THS_H_REG         0x66
#define LIS2MDL_STATUS_REG            0x67
#define LIS2MDL_OUTX_L_REG            0x68
#define LIS2MDL_OUTX_H_REG            0x69
#define LIS2MDL_OUTY_L_REG            0x6A
#define LIS2MDL_OUTY_H_REG            0x6B
#define LIS2MDL_OUTZ_L_REG            0x6C
#define LIS2MDL_OUTZ_H_REG            0x6D
#define LIS2MDL_TEMP_OUT_L_REG        0x6E
#define LIS2MDL_TEMP_OUT_H_REG        0x6F

#define LIS2MDL_ADDRESS               0x1E

#define MODR_10Hz   0x00
#define MODR_20Hz   0x01
#define MODR_50Hz   0x02
#define MODR_100Hz  0x03

#define MFS_14BITS  0  // 0.6 mG per LSB
#define MFS_16BITS  1  // 0.15 mG per LSB

/* Data Structures */
typedef struct lis2mdl_s
{
	uint8_t m_odr;
} lis2mdl_t;

/* Function Prototypes */
void lis2mdl_init(lis2mdl_t *lis2mdl, uint8_t m_odr);
void lis2mdl_config(lis2mdl_t *lis2mdl, I2C_HandleTypeDef *hi2c);

#endif
