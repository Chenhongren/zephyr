/*
 * Copyright (c) 2022 Esco Medical ApS
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM42670_REG_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM42670_REG_H_

#include <zephyr/sys/util.h>

/* Helper macros for addressing registers in MREG1-3, see datasheet section 13 */
#define REG_MADDR_BASE			0x0028
#define REG_MREG1_SHIFT			8
#define REG_MREG2_SHIFT			9
#define REG_MREG3_SHIFT			10
#define REG_BANK0_OFFSET		0x0000
#define REG_MREG1_OFFSET		(REG_MADDR_BASE << REG_MREG1_SHIFT)
#define REG_MREG2_OFFSET		(REG_MADDR_BASE << REG_MREG2_SHIFT)
#define REG_MREG3_OFFSET		(REG_MADDR_BASE << REG_MREG3_SHIFT)
#define REG_ADDRESS_MASK		GENMASK(7, 0)
#define REG_BANK_MASK			GENMASK(15, 8)
#define REG_SPI_READ_BIT		BIT(7)
#define MREG_R_W_WAIT_US		20 /* 10us, but use 20us to be on the safe side */

/* BANK 0 */
#define REG_MCLK_RDY			(REG_BANK0_OFFSET | 0x00)
#define REG_DEVICE_CONFIG		(REG_BANK0_OFFSET | 0x01)
#define REG_SIGNAL_PATH_RESET		(REG_BANK0_OFFSET | 0x02)
#define REG_DRIVE_CONFIG1		(REG_BANK0_OFFSET | 0x03)
#define REG_DRIVE_CONFIG2		(REG_BANK0_OFFSET | 0x04)
#define REG_DRIVE_CONFIG3		(REG_BANK0_OFFSET | 0x05)
#define REG_INT_CONFIG			(REG_BANK0_OFFSET | 0x06)
#define REG_TEMP_DATA1			(REG_BANK0_OFFSET | 0x09)
#define REG_TEMP_DATA0			(REG_BANK0_OFFSET | 0x0a)
#define REG_ACCEL_DATA_X1		(REG_BANK0_OFFSET | 0x0b)
#define REG_ACCEL_DATA_X0		(REG_BANK0_OFFSET | 0x0c)
#define REG_ACCEL_DATA_Y1		(REG_BANK0_OFFSET | 0x0d)
#define REG_ACCEL_DATA_Y0		(REG_BANK0_OFFSET | 0x0e)
#define REG_ACCEL_DATA_Z1		(REG_BANK0_OFFSET | 0x0f)
#define REG_ACCEL_DATA_Z0		(REG_BANK0_OFFSET | 0x10)
#define REG_GYRO_DATA_X1		(REG_BANK0_OFFSET | 0x11)
#define REG_GYRO_DATA_X0		(REG_BANK0_OFFSET | 0x12)
#define REG_GYRO_DATA_Y1		(REG_BANK0_OFFSET | 0x13)
#define REG_GYRO_DATA_Y0		(REG_BANK0_OFFSET | 0x14)
#define REG_GYRO_DATA_Z1		(REG_BANK0_OFFSET | 0x15)
#define REG_GYRO_DATA_Z0		(REG_BANK0_OFFSET | 0x16)
#define REG_TMST_FSYNCH			(REG_BANK0_OFFSET | 0x17)
#define REG_TMST_FSYNCL			(REG_BANK0_OFFSET | 0x18)
#define REG_APEX_DATA4			(REG_BANK0_OFFSET | 0x1d)
#define REG_APEX_DATA5			(REG_BANK0_OFFSET | 0x1e)
#define REG_PWR_MGMT0			(REG_BANK0_OFFSET | 0x1f)
#define REG_GYRO_CONFIG0		(REG_BANK0_OFFSET | 0x20)
#define REG_ACCEL_CONFIG0		(REG_BANK0_OFFSET | 0x21)
#define REG_TEMP_CONFIG0		(REG_BANK0_OFFSET | 0x22)
#define REG_GYRO_CONFIG1		(REG_BANK0_OFFSET | 0x23)
#define REG_ACCEL_CONFIG1		(REG_BANK0_OFFSET | 0x24)
#define REG_APEX_CONFIG0		(REG_BANK0_OFFSET | 0x25)
#define REG_APEX_CONFIG1		(REG_BANK0_OFFSET | 0x26)
#define REG_WOM_CONFIG			(REG_BANK0_OFFSET | 0x27)
#define REG_FIFO_CONFIG1		(REG_BANK0_OFFSET | 0x28)
#define REG_FIFO_CONFIG2		(REG_BANK0_OFFSET | 0x29)
#define REG_FIFO_CONFIG3		(REG_BANK0_OFFSET | 0x2a)
#define REG_INT_SOURCE0			(REG_BANK0_OFFSET | 0x2b)
#define REG_INT_SOURCE1			(REG_BANK0_OFFSET | 0x2c)
#define REG_INT_SOURCE3			(REG_BANK0_OFFSET | 0x2d)
#define REG_INT_SOURCE4			(REG_BANK0_OFFSET | 0x2e)
#define REG_FIFO_LOST_PKT0		(REG_BANK0_OFFSET | 0x2f)
#define REG_FIFO_LOST_PKT1		(REG_BANK0_OFFSET | 0x30)
#define REG_APEX_DATA0			(REG_BANK0_OFFSET | 0x31)
#define REG_APEX_DATA1			(REG_BANK0_OFFSET | 0x32)
#define REG_APEX_DATA2			(REG_BANK0_OFFSET | 0x33)
#define REG_APEX_DATA3			(REG_BANK0_OFFSET | 0x34)
#define REG_INTF_CONFIG0		(REG_BANK0_OFFSET | 0x35)
#define REG_INTF_CONFIG1		(REG_BANK0_OFFSET | 0x36)
#define REG_INT_STATUS_DRDY		(REG_BANK0_OFFSET | 0x39)
#define REG_INT_STATUS			(REG_BANK0_OFFSET | 0x3a)
#define REG_INT_STATUS2			(REG_BANK0_OFFSET | 0x3b)
#define REG_INT_STATUS3			(REG_BANK0_OFFSET | 0x3c)
#define REG_FIFO_COUNTH			(REG_BANK0_OFFSET | 0x3d)
#define REG_FIFO_COUNTL			(REG_BANK0_OFFSET | 0x3e)
#define REG_FIFO_DATA			(REG_BANK0_OFFSET | 0x3f)
#define REG_WHO_AM_I			(REG_BANK0_OFFSET | 0x75)
#define REG_BLK_SEL_W			(REG_BANK0_OFFSET | 0x79)
#define REG_MADDR_W			(REG_BANK0_OFFSET | 0x7a)
#define REG_M_W				(REG_BANK0_OFFSET | 0x7b)
#define REG_BLK_SEL_R			(REG_BANK0_OFFSET | 0x7c)
#define REG_MADDR_R			(REG_BANK0_OFFSET | 0x7d)
#define REG_M_R				(REG_BANK0_OFFSET | 0x7e)

/* MREG1 */
#define REG_TMST_CONFIG1		(REG_MREG1_OFFSET | 0x00)
#define REG_FIFO_CONFIG5		(REG_MREG1_OFFSET | 0x01)
#define REG_FIFO_CONFIG6		(REG_MREG1_OFFSET | 0x02)
#define REG_FSYNC_CONFIG		(REG_MREG1_OFFSET | 0x03)
#define REG_INT_CONFIG0			(REG_MREG1_OFFSET | 0x04)
#define REG_INT_CONFIG1			(REG_MREG1_OFFSET | 0x05)
#define REG_SENSOR_CONFIG3		(REG_MREG1_OFFSET | 0x06)
#define REG_ST_CONFIG			(REG_MREG1_OFFSET | 0x13)
#define REG_SELFTEST			(REG_MREG1_OFFSET | 0x14)
#define REG_INTF_CONFIG6		(REG_MREG1_OFFSET | 0x23)
#define REG_INTF_CONFIG10		(REG_MREG1_OFFSET | 0x25)
#define REG_INTF_CONFIG7		(REG_MREG1_OFFSET | 0x28)
#define REG_OTP_CONFIG			(REG_MREG1_OFFSET | 0x2b)
#define REG_INT_SOURCE6			(REG_MREG1_OFFSET | 0x2f)
#define REG_INT_SOURCE7			(REG_MREG1_OFFSET | 0x30)
#define REG_INT_SOURCE8			(REG_MREG1_OFFSET | 0x31)
#define REG_INT_SOURCE9			(REG_MREG1_OFFSET | 0x32)
#define REG_INT_SOURCE10		(REG_MREG1_OFFSET | 0x33)
#define REG_APEX_CONFIG2		(REG_MREG1_OFFSET | 0x44)
#define REG_APEX_CONFIG3		(REG_MREG1_OFFSET | 0x45)
#define REG_APEX_CONFIG4		(REG_MREG1_OFFSET | 0x46)
#define REG_APEX_CONFIG5		(REG_MREG1_OFFSET | 0x47)
#define REG_APEX_CONFIG9		(REG_MREG1_OFFSET | 0x48)
#define REG_APEX_CONFIG10		(REG_MREG1_OFFSET | 0x49)
#define REG_APEX_CONFIG11		(REG_MREG1_OFFSET | 0x4a)
#define REG_ACCEL_WOM_X_THR		(REG_MREG1_OFFSET | 0x4b)
#define REG_ACCEL_WOM_Y_THR		(REG_MREG1_OFFSET | 0x4c)
#define REG_ACCEL_WOM_Z_THR		(REG_MREG1_OFFSET | 0x4d)
#define REG_OFFSET_USER0		(REG_MREG1_OFFSET | 0x4e)
#define REG_OFFSET_USER1		(REG_MREG1_OFFSET | 0x4f)
#define REG_OFFSET_USER2		(REG_MREG1_OFFSET | 0x50)
#define REG_OFFSET_USER3		(REG_MREG1_OFFSET | 0x51)
#define REG_OFFSET_USER4		(REG_MREG1_OFFSET | 0x52)
#define REG_OFFSET_USER5		(REG_MREG1_OFFSET | 0x53)
#define REG_OFFSET_USER6		(REG_MREG1_OFFSET | 0x54)
#define REG_OFFSET_USER7		(REG_MREG1_OFFSET | 0x55)
#define REG_OFFSET_USER8		(REG_MREG1_OFFSET | 0x56)
#define REG_ST_STATUS1			(REG_MREG1_OFFSET | 0x63)
#define REG_ST_STATUS2			(REG_MREG1_OFFSET | 0x64)
#define REG_FDR_CONFIG			(REG_MREG1_OFFSET | 0x66)
#define REG_APEX_CONFIG12		(REG_MREG1_OFFSET | 0x67)

/* MREG2 */
#define REG_OTP_CTRL7			(REG_MREG2_OFFSET | 0x06)

/* MREG3 */
#define REG_XA_ST_DATA3			(REG_MREG3_OFFSET | 0x00)
#define REG_YA_ST_DATA3			(REG_MREG3_OFFSET | 0x01)
#define REG_ZA_ST_DATA3			(REG_MREG3_OFFSET | 0x02)
#define REG_XG_ST_DATA3			(REG_MREG3_OFFSET | 0x03)
#define REG_YG_ST_DATA3			(REG_MREG3_OFFSET | 0x04)
#define REG_ZG_ST_DATA3			(REG_MREG3_OFFSET | 0x05)

/* Bank0 REG_MCLK_RDY */
#define BIT_MCLK_RDY			BIT(3)

/* Bank0 REG_DEVICE_CONFIG */
#define BIT_SPI_AP_4WIRE		BIT(2)
#define BIT_SPI_MODE			BIT(0)

/* Bank0 REG_SIGNAL_PATH_RESET */
#define BIT_FIFO_FLUSH			BIT(2)
#define BIT_SOFT_RESET			BIT(4)

/* Bank0 REG_INST_STATUS */
#define BIT_STATUS_RESET_DONE_INT	BIT(4)

/* Bank0 REG_INT_CONFIG */
#define BIT_INT1_POLARITY		BIT(0)
#define BIT_INT1_DRIVE_CIRCUIT		BIT(1)
#define BIT_INT1_MODE			BIT(2)
#define BIT_INT2_POLARITY		BIT(3)
#define BIT_INT2_DRIVE_CIRCUIT		BIT(4)
#define BIT_INT2_MODE			BIT(5)

/* Bank0 REG_PWR_MGMT_0 */
#define MASK_ACCEL_MODE			GENMASK(1, 0)
#define BIT_ACCEL_MODE_OFF		0x00
#define BIT_ACCEL_MODE_LPM		0x02
#define BIT_ACCEL_MODE_LNM		0x03
#define MASK_GYRO_MODE			GENMASK(3, 2)
#define BIT_GYRO_MODE_OFF		0x00
#define BIT_GYRO_MODE_STBY		0x01
#define BIT_GYRO_MODE_LNM		0x03
#define BIT_IDLE			BIT(4)
#define BIT_ACCEL_LP_CLK_SEL		BIT(7)

/* Bank0 REG_INT_SOURCE0 */
#define BIT_INT_AGC_RDY_INT1_EN		BIT(0)
#define BIT_INT_FIFO_FULL_INT1_EN	BIT(1)
#define BIT_INT_FIFO_THS_INT1_EN	BIT(2)
#define BIT_INT_DRDY_INT1_EN		BIT(3)
#define BIT_INT_RESET_DONE_INT1_EN	BIT(4)
#define BIT_INT_PLL_RDY_INT1_EN		BIT(5)
#define BIT_INT_FSYNC_INT1_EN		BIT(6)
#define BIT_INT_ST_INT1_EN		BIT(7)

/* Bank0 REG_INT_STATUS_DRDY */
#define BIT_INT_STATUS_DATA_DRDY	BIT(0)

/* Bank9 REG_INTF_CONFIG1 */
#define BIT_I3C_SDR_EN			BIT(3)
#define BIT_I3C_DDR_EN			BIT(2)
#define MASK_CLKSEL			GENMASK(1, 0)
#define BIT_CLKSEL_INT_RC		0x00
#define BIT_CLKSEL_PLL_OR_RC		0x01
#define BIT_CLKSEL_DISABLE		0x11

/* Bank0 REG_INT_STATUS */
#define BIT_INT_STATUS_AGC_RDY		BIT(0)
#define BIT_INT_STATUS_FIFO_FULL	BIT(1)
#define BIT_INT_STATUS_FIFO_THS		BIT(2)
#define BIT_INT_STATUS_RESET_DONE	BIT(4)
#define BIT_INT_STATUS_PLL_RDY		BIT(5)
#define BIT_INT_STATUS_FSYNC		BIT(6)
#define BIT_INT_STATUS_ST		BIT(7)

/* Bank0 REG_INT_STATUS2 */
#define BIT_INT_STATUS_WOM_Z		BIT(0)
#define BIT_INT_STATUS_WOM_Y		BIT(1)
#define BIT_INT_STATUS_WOM_X		BIT(2)
#define BIT_INT_STATUS_SMD		BIT(3)

/* Bank0 REG_INT_STATUS3 */
#define BIT_INT_STATUS_LOWG_DET		BIT(1)
#define BIT_INT_STATUS_FF_DET		BIT(2)
#define BIT_INT_STATUS_TILT_DET		BIT(3)
#define BIT_INT_STATUS_STEP_CNT_OVFL	BIT(4)
#define BIT_INT_STATUS_STEP_DET		BIT(5)

/* Bank0 REG_ACCEL_CONFIG0 */
#define MASK_ACCEL_UI_FS_SEL		GENMASK(6, 5)
#define BIT_ACCEL_UI_FS_16		0x00
#define BIT_ACCEL_UI_FS_8		0x01
#define BIT_ACCEL_UI_FS_4		0x02
#define BIT_ACCEL_UI_FS_2		0x03
#define MASK_ACCEL_ODR			GENMASK(3, 0)
#define BIT_ACCEL_ODR_1600		0x05
#define BIT_ACCEL_ODR_800		0x06
#define BIT_ACCEL_ODR_400		0x07
#define BIT_ACCEL_ODR_200		0x08
#define BIT_ACCEL_ODR_100		0x09
#define BIT_ACCEL_ODR_50		0x0A
#define BIT_ACCEL_ODR_25		0x0B
#define BIT_ACCEL_ODR_12		0x0C
#define BIT_ACCEL_ODR_6			0x0D
#define BIT_ACCEL_ODR_3			0x0E
#define BIT_ACCEL_ODR_1			0x0F

/* Bank0 REG_GYRO_CONFIG0 */
#define MASK_GYRO_UI_FS_SEL		GENMASK(6, 5)
#define BIT_GYRO_UI_FS_2000		0x00
#define BIT_GYRO_UI_FS_1000		0x01
#define BIT_GYRO_UI_FS_500		0x02
#define BIT_GYRO_UI_FS_250		0x03
#define MASK_GYRO_ODR			GENMASK(3, 0)
#define BIT_GYRO_ODR_1600		0x05
#define BIT_GYRO_ODR_800		0x06
#define BIT_GYRO_ODR_400		0x07
#define BIT_GYRO_ODR_200		0x08
#define BIT_GYRO_ODR_100		0x09
#define BIT_GYRO_ODR_50			0x0A
#define BIT_GYRO_ODR_25			0x0B
#define BIT_GYRO_ODR_12			0x0C

/* misc. defines */
#define WHO_AM_I_ICM42670		0x67
#define MIN_ACCEL_SENS_SHIFT		11
#define ACCEL_DATA_SIZE			6
#define GYRO_DATA_SIZE			6
#define TEMP_DATA_SIZE			2
#define MCLK_POLL_INTERVAL_US		250
#define MCLK_POLL_ATTEMPTS		100
#define SOFT_RESET_TIME_MS		2 /* 1ms + elbow room */

#endif /* ZEPHYR_DRIVERS_SENSOR_ICM42670_REG_H_ */
