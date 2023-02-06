/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM42688_REG_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM42688_REG_H_

#include <zephyr/sys/util.h>

/* Address value has a read bit */
#define REG_SPI_READ_BIT BIT(7)

/* Common bank select register and values */
#define REG_BANK_SEL  0x76
#define MASK_BANK_SEL GENMASK(2, 0)
#define BIT_BANK0     0x000
#define BIT_BANK1     0x001
#define BIT_BANK2     0x010
#define BIT_BANK3     0x011
#define BIT_BANK4     0x100

/* Helper macros for addressing registers in the 4 register banks
 * registers are defined as 16 bit values with the bank in the high
 * byte and the register address in the low byte
 */

#define REG_ADDRESS_MASK      GENMASK(7, 0)
#define REG_BANK_MASK	      GENMASK(15, 8)
#define REG_BANK_OFFSET(bank) (bank << 8)
#define REG_BANK0_OFFSET      REG_BANK_OFFSET(BIT_BANK0)
#define REG_BANK1_OFFSET      REG_BANK_OFFSET(BIT_BANK1)
#define REG_BANK2_OFFSET      REG_BANK_OFFSET(BIT_BANK2)
#define REG_BANK3_OFFSET      REG_BANK_OFFSET(BIT_BANK3)
#define REG_BANK4_OFFSET      REG_BANK_OFFSET(BIT_BANK4)

/* Bank 0 */
#define REG_DEVICE_CONFIG      (REG_BANK0_OFFSET | 0x11)
#define REG_DRIVE_CONFIG       (REG_BANK0_OFFSET | 0x13)
#define REG_INT_CONFIG	       (REG_BANK0_OFFSET | 0x14)
#define REG_FIFO_CONFIG	       (REG_BANK0_OFFSET | 0x16)
#define REG_TEMP_DATA1	       (REG_BANK0_OFFSET | 0x1D)
#define REG_TEMP_DATA0	       (REG_BANK0_OFFSET | 0x1E)
#define REG_ACCEL_DATA_X1      (REG_BANK0_OFFSET | 0x1F)
#define REG_ACCEL_DATA_X0      (REG_BANK0_OFFSET | 0x20)
#define REG_ACCEL_DATA_Y1      (REG_BANK0_OFFSET | 0x21)
#define REG_ACCEL_DATA_Y0      (REG_BANK0_OFFSET | 0x22)
#define REG_ACCEL_DATA_Z1      (REG_BANK0_OFFSET | 0x23)
#define REG_ACCEL_DATA_Z0      (REG_BANK0_OFFSET | 0x24)
#define REG_GYRO_DATA_X1       (REG_BANK0_OFFSET | 0x25)
#define REG_GYRO_DATA_X0       (REG_BANK0_OFFSET | 0x26)
#define REG_GYRO_DATA_Y1       (REG_BANK0_OFFSET | 0x27)
#define REG_GYRO_DATA_Y0       (REG_BANK0_OFFSET | 0x28)
#define REG_GYRO_DATA_Z1       (REG_BANK0_OFFSET | 0x29)
#define REG_GYRO_DATA_Z0       (REG_BANK0_OFFSET | 0x2A)
#define REG_TMST_FSYNCH	       (REG_BANK0_OFFSET | 0x2B)
#define REG_TMST_FSYNCL	       (REG_BANK0_OFFSET | 0x2C)
#define REG_INT_STATUS	       (REG_BANK0_OFFSET | 0x2D)
#define REG_FIFO_COUNTH	       (REG_BANK0_OFFSET | 0x2E)
#define REG_FIFO_COUNTL	       (REG_BANK0_OFFSET | 0x2F)
#define REG_FIFO_DATA	       (REG_BANK0_OFFSET | 0x30)
#define REG_APEX_DATA0	       (REG_BANK0_OFFSET | 0x31)
#define REG_APEX_DATA1	       (REG_BANK0_OFFSET | 0x32)
#define REG_APEX_DATA2	       (REG_BANK0_OFFSET | 0x33)
#define REG_APEX_DATA3	       (REG_BANK0_OFFSET | 0x34)
#define REG_APEX_DATA4	       (REG_BANK0_OFFSET | 0x35)
#define REG_APEX_DATA5	       (REG_BANK0_OFFSET | 0x36)
#define REG_INT_STATUS2	       (REG_BANK0_OFFSET | 0x37)
#define REG_INT_STATUS3	       (REG_BANK0_OFFSET | 0x38)
#define REG_SIGNAL_PATH_RESET  (REG_BANK0_OFFSET | 0x4B)
#define REG_INTF_CONFIG0       (REG_BANK0_OFFSET | 0x4C)
#define REG_INTF_CONFIG1       (REG_BANK0_OFFSET | 0x4D)
#define REG_PWR_MGMT0	       (REG_BANK0_OFFSET | 0x4E)
#define REG_GYRO_CONFIG0       (REG_BANK0_OFFSET | 0x4F)
#define REG_ACCEL_CONFIG0      (REG_BANK0_OFFSET | 0x50)
#define REG_GYRO_CONFIG1       (REG_BANK0_OFFSET | 0x51)
#define REG_GYRO_ACCEL_CONFIG0 (REG_BANK0_OFFSET | 0x52)
#define REG_ACCEL_CONFIG1      (REG_BANK0_OFFSET | 0x53)
#define REG_TMST_CONFIG	       (REG_BANK0_OFFSET | 0x54)
#define REG_APEX_CONFIG0       (REG_BANK0_OFFSET | 0x56)
#define REG_SMD_CONFIG	       (REG_BANK0_OFFSET | 0x57)
#define REG_FIFO_CONFIG1       (REG_BANK0_OFFSET | 0x5F)
#define REG_FIFO_CONFIG2       (REG_BANK0_OFFSET | 0x60)
#define REG_FIFO_CONFIG3       (REG_BANK0_OFFSET | 0x61)
#define REG_FSYNC_CONFIG       (REG_BANK0_OFFSET | 0x62)
#define REG_INT_CONFIG0	       (REG_BANK0_OFFSET | 0x63)
#define REG_INT_CONFIG1	       (REG_BANK0_OFFSET | 0x64)
#define REG_INT_SOURCE0	       (REG_BANK0_OFFSET | 0x65)
#define REG_INT_SOURCE1	       (REG_BANK0_OFFSET | 0x66)
#define REG_INT_SOURCE3	       (REG_BANK0_OFFSET | 0x68)
#define REG_INT_SOURCE4	       (REG_BANK0_OFFSET | 0x69)
#define REG_FIFO_LOST_PKT0     (REG_BANK0_OFFSET | 0x6C)
#define REG_FIFO_LOST_PKT1     (REG_BANK0_OFFSET | 0x6D)
#define REG_SELF_TEST_CONFIG   (REG_BANK0_OFFSET | 0x70)
#define REG_WHO_AM_I	       (REG_BANK0_OFFSET | 0x75)

/* Bank 1 */
#define SENSOR_CONFIG0	     (REG_BANK1_OFFSET | 0x03)
#define GYRO_CONFIG_STATIC2  (REG_BANK1_OFFSET | 0x0B)
#define GYRO_CONFIG_STATIC3  (REG_BANK1_OFFSET | 0x0C)
#define GYRO_CONFIG_STATIC4  (REG_BANK1_OFFSET | 0x0D)
#define GYRO_CONFIG_STATIC5  (REG_BANK1_OFFSET | 0x0E)
#define GYRO_CONFIG_STATIC6  (REG_BANK1_OFFSET | 0x0F)
#define GYRO_CONFIG_STATIC7  (REG_BANK1_OFFSET | 0x10)
#define GYRO_CONFIG_STATIC8  (REG_BANK1_OFFSET | 0x11)
#define GYRO_CONFIG_STATIC9  (REG_BANK1_OFFSET | 0x12)
#define GYRO_CONFIG_STATIC10 (REG_BANK1_OFFSET | 0x13)
#define REG_XG_ST_DATA	     (REG_BANK1_OFFSET | 0x5F)
#define REG_YG_ST_DATA	     (REG_BANK1_OFFSET | 0x60)
#define REG_ZG_ST_DATA	     (REG_BANK1_OFFSET | 0x61)
#define REG_TMSTVAL0	     (REG_BANK1_OFFSET | 0x62)
#define REG_TMSTVAL1	     (REG_BANK1_OFFSET | 0x63)
#define REG_TMSTVAL2	     (REG_BANK1_OFFSET | 0x64)
#define REG_INTF_CONFIG4     (REG_BANK1_OFFSET | 0x7A)
#define REG_INTF_CONFIG5     (REG_BANK1_OFFSET | 0x7B)
#define REG_INTF_CONFIG6     (REG_BANK1_OFFSET | 0x7C)

/* Bank 2 */

/* Bank 4 */
#define REG_INT_SOURCE6 (REG_BANK4_OFFSET | 0x77)
#define REG_INT_SOURCE7 (REG_BANK4_OFFSET | 0x78)
#define REG_INT_SOURCE8 (REG_BANK4_OFFSET | 0x79)
#define REG_INT_SOURCE9 (REG_BANK4_OFFSET | 0x80)

/* Bank0 REG_DEVICE_CONFIG */
#define BIT_SOFT_RESET BIT(0)
#define BIT_SPI_MODE   BIT(4)

/* Bank0 REG_DRIVE_CONFIG */
/* Not used! */

/* Bank0 REG_INT_CONFIG */
#define BIT_INT1_POLARITY      BIT(0)
#define BIT_INT1_DRIVE_CIRCUIT BIT(1)
#define BIT_INT1_MODE	       BIT(2)
#define BIT_INT2_POLARITY      BIT(3)
#define BIT_INT2_DRIVE_CIRCUIT BIT(4)
#define BIT_INT2_MODE	       BIT(5)

/* Bank0 REG_FIFO_CONFIG */
#define MASK_FIFO_MODE		   GENMASK(7, 6)
#define BIT_FIFO_MODE_BYPASS	   0x00
#define BIT_FIFO_MODE_STREAM	   0x01
#define BIT_FIFO_MODE_STOP_ON_FULL 0x10

/* Bank0 REG_INT_STATUS */
#define BIT_INT_STATUS_AGC_RDY	  BIT(0)
#define BIT_INT_STATUS_FIFO_FULL  BIT(1)
#define BIT_INT_STATUS_FIFO_THS	  BIT(2)
#define BIT_INT_STATUS_DATA_RDY	  BIT(3)
#define BIT_INT_STATUS_RESET_DONE BIT(4)
#define BIT_INT_STATUS_PLL_RDY	  BIT(5)
#define BIT_INT_STATUS_FSYNC	  BIT(6)

/* Bank0 REG_INT_STATUS2 */
#define BIT_INT_STATUS_WOM_Z BIT(0)
#define BIT_INT_STATUS_WOM_Y BIT(1)
#define BIT_INT_STATUS_WOM_X BIT(2)
#define BIT_INT_STATUS_SMD   BIT(3)

/* Bank0 REG_INT_STATUS3 */
#define BIT_INT_STATUS_LOWG_DET	     BIT(1)
#define BIT_INT_STATUS_FF_DET	     BIT(2)
#define BIT_INT_STATUS_TILT_DET	     BIT(3)
#define BIT_INT_STATUS_STEP_CNT_OVFL BIT(4)
#define BIT_INT_STATUS_STEP_DET	     BIT(5)

/* Bank0 REG_SIGNAL_PATH_RESET */
#define BIT_FIFO_FLUSH	     BIT(1)
#define BIT_TMST_STROBE	     BIT(2)
#define BIT_ABORT_AND_RESET  BIT(3)
#define BIT_DMP_MEM_RESET_EN BIT(5)
#define BIT_DMP_INIT_EN	     BIT(6)

/* Bank0 REG_INTF_CONFIG0 */
#define MASK_UI_SIFS_CFG	    GENMASK(1, 0)
#define BIT_UI_SIFS_CFG_DISABLE_SPI 0x10
#define BIT_UI_SIFS_CFG_DISABLE_I2C 0x11
#define BIT_SENSOR_DATA_ENDIAN	    BIT(4)
#define BIT_FIFO_COUNT_ENDIAN	    BIT(5)
#define BIT_FIFO_COUNT_REC	    BIT(6)
#define BIT_FIFO_HOLD_LAST_DATA_EN  BIT(7)

/* Bank0 REG_INTF_CONFIG1 */
#define MASK_CLKSEL	     GENMASK(1, 0)
#define BIT_CLKSEL_INT_RC    0x00
#define BIT_CLKSEL_PLL_OR_RC 0x01
#define BIT_CLKSEL_DISABLE   0x11
#define BIT_RTC_MODE	     BIT(2)
#define BIT_ACCEL_LP_CLK_SEL BIT(3)

/* Bank0 REG_PWR_MGMT_0 */
#define MASK_ACCEL_MODE	   GENMASK(1, 0)
#define BIT_ACCEL_MODE_OFF 0x00
#define BIT_ACCEL_MODE_LPM 0x02
#define BIT_ACCEL_MODE_LNM 0x03
#define MASK_GYRO_MODE	   GENMASK(3, 2)
#define BIT_GYRO_MODE_OFF  0x00
#define BIT_GYRO_MODE_STBY 0x01
#define BIT_GYRO_MODE_LNM  0x03
#define BIT_IDLE	   BIT(4)
#define BIT_TEMP_DIS	   BIT(5)

/* Bank0 REG_GYRO_CONFIG0 */
#define MASK_GYRO_UI_FS_SEL   GENMASK(7, 5)
#define BIT_GYRO_UI_FS_2000   0x00
#define BIT_GYRO_UI_FS_1000   0x01
#define BIT_GYRO_UI_FS_500    0x02
#define BIT_GYRO_UI_FS_250    0x03
#define BIT_GYRO_UI_FS_125    0x04
#define BIT_GYRO_UI_FS_62_5   0x05
#define BIT_GYRO_UI_FS_31_25  0x06
#define BIT_GYRO_UI_FS_15_625 0x07
#define MASK_GYRO_ODR	      GENMASK(3, 0)
#define BIT_GYRO_ODR_32000    0x01
#define BIT_GYRO_ODR_16000    0x02
#define BIT_GYRO_ODR_8000     0x03
#define BIT_GYRO_ODR_4000     0x04
#define BIT_GYRO_ODR_2000     0x05
#define BIT_GYRO_ODR_1000     0x06
#define BIT_GYRO_ODR_200      0x07
#define BIT_GYRO_ODR_100      0x08
#define BIT_GYRO_ODR_50	      0x09
#define BIT_GYRO_ODR_25	      0x0A
#define BIT_GYRO_ODR_12_5     0x0B
#define BIT_GYRO_ODR_500      0x0F

/* Bank0 REG_ACCEL_CONFIG0 */
#define MASK_ACCEL_UI_FS_SEL GENMASK(7, 5)
#define BIT_ACCEL_UI_FS_16   0x00
#define BIT_ACCEL_UI_FS_8    0x01
#define BIT_ACCEL_UI_FS_4    0x02
#define BIT_ACCEL_UI_FS_2    0x03
#define MASK_ACCEL_ODR	     GENMASK(3, 0)
#define BIT_ACCEL_ODR_32000  0x01
#define BIT_ACCEL_ODR_16000  0x02
#define BIT_ACCEL_ODR_8000   0x03
#define BIT_ACCEL_ODR_4000   0x04
#define BIT_ACCEL_ODR_2000   0x05
#define BIT_ACCEL_ODR_1000   0x06
#define BIT_ACCEL_ODR_200    0x07
#define BIT_ACCEL_ODR_100    0x08
#define BIT_ACCEL_ODR_50     0x09
#define BIT_ACCEL_ODR_25     0x0A
#define BIT_ACCEL_ODR_12_5   0x0B
#define BIT_ACCEL_ODR_6_25   0x0C
#define BIT_ACCEL_ODR_3_12   0x0D
#define BIT_ACCEL_ODR_1_5625 0x0E
#define BIT_ACCEL_ODR_500    0x0F

/* Bank0 FIFO_CONFIG1 */
#define BIT_FIFO_WM_GT_TH      BIT(5)
#define BIT_FIFO_HIRES_EN      BIT(4)
#define BIT_FIFO_TMST_FSYNC_EN BIT(3)
#define BIT_FIFO_GYRO_EN       BIT(2)
#define BIT_FIFO_ACCEL_EN      BIT(1)
#define BIT_FIFO_TEMP_EN       BIT(0)

/* Bank0 INT_SOURCE0 */
#define BIT_UI_FSYNC_INT1_EN	    BIT(6)
#define BIT_PLL_RDY_INT1_EN	    BIT(5)
#define BIT_RESET_DONE_INT1_EN	    BIT(4)
#define BIT_UI_DRDY_INT1_EN	    BIT(3)
#define BIT_FIFO_THS_INT1_EN	    BIT(2)
#define BIT_FIFO_FULL_INT1_EN	    BIT(1)
#define BIT_FIFO_UI_AGC_RDY_INT1_EN BIT(0)

/* Bank0 REG_FSYNC_CONFIG */
#define MASK_FSYNC_UI_SEL GENMASK(6, 4)
#define BIT_FSYNC_UI

/* Bank0 REG_INT_CONFIG1 */
#define BIT_INT_TPULSE_DURATION	  BIT(6)
#define BIT_INT_TDEASSERT_DISABLE BIT(5)
#define BIT_INT_ASYNC_RESET	  BIT(4)

/* misc. defines */
#define WHO_AM_I_ICM42688     0x47
#define MIN_ACCEL_SENS_SHIFT  11
#define ACCEL_DATA_SIZE	      6
#define GYRO_DATA_SIZE	      6
#define TEMP_DATA_SIZE	      2
#define MCLK_POLL_INTERVAL_US 250
#define MCLK_POLL_ATTEMPTS    100
#define SOFT_RESET_TIME_MS    2 /* 1ms + elbow room */

#endif /* ZEPHYR_DRIVERS_SENSOR_ICM42688_REG_H_ */
