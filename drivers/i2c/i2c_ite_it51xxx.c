/*
 * Copyright (c) 2025 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it51xxx_i2c

#include <errno.h>
#include <soc.h>
#include <soc_dt.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(i2c_ite_it51xxx, CONFIG_I2C_LOG_LEVEL);
#include "i2c_bitbang.h"
#include "i2c-priv.h"

/* it51xxx SMBus registers definition */
/* 0x00. 0x11, 0x29, 0x35, 0xa0, 0xb0: Host Status */
#define SMB_HOSTA       0x00
#define SMB_BDS         BIT(7)
#define SMB_TMOE        BIT(6)
#define SMB_NACK        BIT(5)
#define SMB_FAIL        BIT(4)
#define SMB_BSER        BIT(3)
#define SMB_DVER        BIT(2)
#define SMB_FINTR       BIT(1)
#define SMB_HOBY        BIT(0)
/* 0x01, 0x12, 0x2a, 0x36, 0xa1, 0xb1: Host Control */
#define SMB_HOCTL       0x01
#define SMB_PEC_EN      BIT(7)
#define SMB_SRT         BIT(6)
#define SMB_LABY        BIT(5)
#define SMB_SMCD(n)     FIELD_PREP(GENMASK(4, 2), n)
#define SMB_KILL        BIT(1)
#define SMB_INTREN      BIT(0)
/* 0x03, 0x14, 0x2c, 0x38, 0xa3, 0xb3: Transmit Slave Address */
#define SMB_TRASLA      0x03
#define SMB_DIR         BIT(0)
/* 0x04, 0x15, 0x2d, 0x39, 0xa4, 0xb4: Data 0 */
#define SMB_D0REG       0x04
/* 0x06, 0x17, 0x2f, 0x3b, 0xa7, 0xb7: Host Block Data Byte */
#define SMB_HOBDB(ch)   ((ch) < 4 ? 0x06 : (ch) < 6 ? 0x07 : 0)
/* 0x08, 0x19, 0x5a: Receive Slave Address */
#define SMB_RESLADR(ch) ((ch) < 2 ? 0x08 : (ch) == 2 ? 0x31 : 0)
/* 0x09, 0x1a, 0x5b: Slave Data */
#define SMB_SLDA(ch)    ((ch) < 2 ? 0x09 : (ch) == 2 ? 0x32 : 0)
/* 0x0a, 0x1b, 0x31, 0x3d, 0xa9, 0xb9: SMBus Pin Control */
#define SMB_SMBPCTL(ch) ((ch) < 2 ? 0x0a : (ch) < 4 ? 0x08 : (ch) < 6 ? 0x09 : 0)
#define SMB_HSMBDCS     BIT(1)
#define SMB_HSMBCS      BIT(0)
/* 0x0b, 0x1c, 0x52: Slave Status */
#define SMB_SLSTA(ch)   ((ch) < 2 ? 0x0b : (ch) == 2 ? 0x29 : 0)
#define SMB_SPDS        BIT(5)
#define SMB_RCS         BIT(3)
#define SMB_STS         BIT(2)
#define SMB_SDS         BIT(1)
/* 0x0c, 0x1d, 0x53: Slave Interrupt Control */
#define SMB_SICR(ch)    ((ch) < 2 ? 0x0c : (ch) == 2 ? 0x2a : 0)
#define SMB_SDSEN       BIT(3)
#define SMB_SDLTOEN     BIT(2)
#define SMB_SITEN       BIT(1)
#define SMB_HONOIN      BIT(0)
/* 0x10, 0x21, 0x32, 0x3e, 0xaa, 0xba: Host Control 2 */
#define SMB_HOCTL2(ch)  ((ch) < 2 ? 0x10 : (ch) < 4 ? 0x09 : (ch) < 6 ? 0x0a : 0)
#define SMB_SSCL        BIT(7)
#define SMB_HTIFYEN     BIT(6)
#define SMB_SLVEN       BIT(5)
#define SMB_SMD_TO_EN   BIT(4)
#define I2C_SW_EN       BIT(3)
#define I2C_SW_WAIT     BIT(2)
#define I2C_EN          BIT(1)
#define SMB_SMH_EN      BIT(0)
/* 0x22: 4.7us Low */
#define SMB_4P7USL      0x22
/* 0x23: 4.0us Low */
#define SMB_4P0USL      0x23
/* 0x24: 300ns */
#define SMB_300NS       0x24
/* 0x25: 250ns */
#define SMB_250NS       0x25
/* 0x27: 45.3us Low */
#define SMB_45P3USLREG  0x27
/* 0x28: 45.3us High */
#define SMB_45P3USHREG  0x28
/* 0x33: 4.7us and 4.0us High */
#define SMB_4P7A4P0H    0x33
/* 0x40, 0x41, 0x42, 0x43, 0xab, 0xbb: SCLKTS_A-F */
#define SMB_SCLKTS(ch)  ((ch) < 4 ? (0x40 + (ch)) : (0xab + ((ch) - 4) * 0x10))
/* BIT[1:0]: SMCLK Setting */
#define SMB_CLKS_1M     4
#define SMB_CLKS_400K   3
#define SMB_CLKS_100K   2
#define SMB_CLKS_50K    1
/* 0x45: Master FIFO Control 1 */
#define SMB_MSTFCTRL1   0x45
#define SMB_BLKDS       BIT(4)
#define SMB_FFEN        BIT(3)
/* 0x46: Master FIFO Status 1 */
#define SMB_MSTFSTS1    0x46
#define SMB_FIFO1_EMPTY BIT(7)
#define SMB_FIFO1_FULL  BIT(6)
/* 0x47: Master FIFO Control 2 */
#define SMB_MSTFCTRL2   0x47
#define SMB_FFCHSEL2_F  4
#define SMB_FFCHSEL2_E  3
#define SMB_FFCHSEL2_D  2
#define SMB_FFCHSEL2_C  1
#define SMB_FFCHSEL2_B  0
/* 0x48: Master FIFO Status 2 */
#define SMB_MSTFSTS2    0x48
#define SMB_FIFO2_EMPTY BIT(7)
#define SMB_FIFO2_FULL  BIT(6)
/* 0x5e, 0x62, 0x64: Slave A, B, C Dedicated FIFO Pre-defined Control */
#define SMB_SDFPCTL(ch) ((ch) == 0 ? 0x5e : (ch) == 1 ? 0x62 : (ch) == 2 ? 0x64 : 0)
#define SMB_SADFE       BIT(0)
/* 0x5f, 0x63, 0x65: Slave A, B, C Dedicated FIFO status */
#define SMB_SFFSTA(ch)  ((ch) == 0 ? 0x5f : (ch) == 1 ? 0x63 : (ch) == 2 ? 0x65 : 0)
#define SMB_FIFO_FULL   BIT(6)
/* 0x60: SMBus Design Switch Interface Control */
#define SMB_SDSIC       0x60
#define SMB_DBSTI(n)    FIELD_PREP(GENMASK(7, 4), n)
#define SMB_DASTI(n)    FIELD_PREP(GENMASK(3, 0), n)
/* 0x61: SMBus Design Switch Interface Control 2 */
#define SMB_SDSIC2      0x61
#define SMB_DDSTI(n)    FIELD_PREP(GENMASK(7, 4), n)
#define SMB_DCSTI(n)    FIELD_PREP(GENMASK(3, 0), n)
/* 0x69: I2C Wr to Rd FIFO */
#define SMB_I2CW2RF     0x69
#define SMB_MAIF        BIT(7)
#define SMB_MBCDEFIF    BIT(6)
#define SMB_MDIFI       BIT(3)
#define SMB_MCIFI       BIT(2)
#define SMB_MBIFI       BIT(1)
#define SMB_MAIFI       BIT(0)
/* 0x6a: I2C Wr to Rd FIFO Interrupt Status */
#define SMB_IWRFISTA    0x6a
#define SMB_MFIFI       BIT(7)
#define SMB_MEIFI       BIT(6)
#define SMB_MFIFID      BIT(5)
#define SMB_MEIFID      BIT(4)
#define SMB_MDIFID      BIT(3)
#define SMB_MCIFID      BIT(2)
#define SMB_MBIFID      BIT(1)
#define SMB_MAIFID      BIT(0)
/* 0x6f:  Shared FIFO Function Enable */
#define SMB_SFFE        0x6f
/* 0x77: Slave Shared FIFO Size Select1 */
#define SMB_SSFSS1      0x77
#define SMB_SFSFSB(n)   FIELD_PREP(GENMASK(6, 4), n)
#define SMB_SFSFSA(n)   FIELD_PREP(GENMASK(2, 0), n)
/* 0x80: Shared FIFO Base Address for Slave A */
#define SMB_SFBAS(ch)   (0x80 + (ch) * 2)
/* 0x87: Slave Shared FIFO Size Select2 */
#define SMB_SSFSS2      0x87
#define SMB_SFSFSC(n)   FIELD_PREP(GENMASK(2, 0), n)
/* 0x99: SMBus Design Switch Interface Control 3 */
#define SMB_SDSIC3      0x99
#define SMB_DFSTI(n)    FIELD_PREP(GENMASK(7, 4), n)
#define SMB_DESTI(n)    FIELD_PREP(GENMASK(3, 0), n)
/* 0xc0: Shared FIFO Base Address MSB for Slave A */
#define SMB_SFBAMS(ch)  (0xc0 + (ch))
/* 0xce: SMBus Design Switch Interface Control 4 */
#define SMB_SDSIC4      0xce
#define SMB_DSBSTI(n)   FIELD_PREP(GENMASK(7, 4), n)
#define SMB_DSASTI(n)   FIELD_PREP(GENMASK(3, 0), n)
/* 0xcf: SMBus Design Switch Interface Control 5 */
#define SMB_SDSIC5      0xcf
#define SMB_DSCSTI(n)   FIELD_PREP(GENMASK(7, 4), n)

/* 0x49, 0x4a, 0x4b, 0x4c, 0xac, 0xbc: Host Nack Source */
#define SMB_HONACKSRC(ch)   ((ch) < 4 ? (0x49 + (ch)) : (0xac + ((ch) - 4) * 0x10))
#define SMB_SSMCDTD         BIT(5)
#define SMB_HSMCDTD         BIT(4)
/* 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca: 25ms */
#define SMB_25MSREG(ch)     (0xc8 + (ch))
#define SMB_CLK_LOW_TIMEOUT 0xff /* ~=25 ms */
/* Start smbus session from idle state */
#define SMB_MSG_START       BIT(5)
#define SMB_LINE_SCL_HIGH   BIT(0)
#define SMB_LINE_SDA_HIGH   BIT(1)
#define SMB_LINE_IDLE       (SMB_LINE_SCL_HIGH | SMB_LINE_SDA_HIGH)

#ifdef CONFIG_I2C_IT51XXX_FIFO_MODE
#define SMB_FIFO_MODE_MAX_SIZE  32
#define SMB_FIFO_MODE_TOTAL_LEN 255
#define SMB_MSG_BURST_READ_MASK (I2C_MSG_RESTART | I2C_MSG_STOP | I2C_MSG_READ)

#define FIFO_ENABLE_NODE(idx) DT_PROP(DT_NODELABEL(i2c##idx), fifo_enable)
#define FIFO_ENABLE_COUNT     (FIFO_ENABLE_NODE(1) + FIFO_ENABLE_NODE(2) + FIFO_ENABLE_NODE(3) + \
			       FIFO_ENABLE_NODE(4) + FIFO_ENABLE_NODE(5))
BUILD_ASSERT(FIFO_ENABLE_COUNT <= 1, "More than one node has fifo2-enable property enabled!");
#endif
#ifdef CONFIG_I2C_TARGET
#define SMB_TARGET_IT51XXX_MAX_FIFO_SIZE 16
#endif

struct i2c_it51xxx_config {
	/* I2C alternate configuration */
	const struct pinctrl_dev_config *pcfg;
	/* SCL GPIO cells */
	struct gpio_dt_spec scl_gpios;
	/* SDA GPIO cells */
	struct gpio_dt_spec sda_gpios;
	int transfer_timeout_ms;
	mm_reg_t port_reg;
	mm_reg_t i2cbase;
	uint32_t bitrate;
	uint8_t i2c_irq_base;
	uint8_t i2cs_irq_base;
	uint8_t port;
	uint8_t channel_switch_sel;
	bool fifo_enable;
	bool target_enable;
	bool target_fifo_mode;
	bool target_shared_fifo_mode;
	bool push_pull_recovery;
};

enum i2c_ch_status {
	I2C_CH_NORMAL = 0,
	I2C_CH_REPEAT_START,
	I2C_CH_WAIT_READ,
	I2C_CH_WAIT_NEXT_XFER,
};

struct target_shared_fifo_size_sel {
	uint16_t fifo_size;
	uint8_t value;
};
static const struct target_shared_fifo_size_sel fifo_size_table[] = {
	[0] = {16,  0x1},
	[1] = {32,  0x2},
	[2] = {64,  0x3},
	[3] = {128, 0x4},
	[4] = {256, 0x5},
};

struct i2c_it51xxx_data {
	struct i2c_msg *msg;
	struct k_mutex mutex;
	struct k_sem device_sync_sem;
	struct i2c_bitbang bitbang;
	enum i2c_ch_status i2ccs;
	/* Index into output data */
	size_t widx;
	/* Index into input data */
	size_t ridx;
	/* Error code, if any */
	uint32_t err;
	/* Address of device */
	uint16_t addr_16bit;
	/* Wait for stop bit interrupt */
	uint8_t stop;
#ifdef CONFIG_I2C_IT51XXX_FIFO_MODE
	struct i2c_msg *msgs_list;
	/* Read or write byte counts. */
	uint32_t bytecnt;
	/* Number of messages. */
	uint8_t num_msgs;
	uint8_t msg_index;
#endif
#ifdef CONFIG_I2C_TARGET
	struct i2c_target_config *target_cfg;
	const struct target_shared_fifo_size_sel *fifo_size_list;
	uint32_t w_index;
	uint32_t r_index;
	/* Target mode FIFO buffer. */
	uint8_t in_buffer[CONFIG_I2C_TARGET_IT51XXX_MAX_BUF_SIZE];
	union {
		uint8_t __aligned(4)
			out_buffer[CONFIG_I2C_TARGET_IT51XXX_MAX_BUF_SIZE];
		uint8_t __aligned(16)
			shared_fifo_size[CONFIG_I2C_TARGET_IT51XXX_MAX_SHARE_FIFO_SIZE];
	};
	bool target_attached;
#endif
};

enum i2c_host_status {
	/* Host busy */
	HOSTA_HOBY = 0x01,
	/* Finish Interrupt */
	HOSTA_FINTR = 0x02,
	/* Device error */
	HOSTA_DVER = 0x04,
	/* Bus error */
	HOSTA_BSER = 0x08,
	/* Fail */
	HOSTA_FAIL = 0x10,
	/* Not response ACK */
	HOSTA_NACK = 0x20,
	/* Time-out error */
	HOSTA_TMOE = 0x40,
	/* Byte done status */
	HOSTA_BDS = 0x80,
	/* Error bit is set */
	HOSTA_ANY_ERROR = (HOSTA_DVER | HOSTA_BSER | HOSTA_FAIL | HOSTA_NACK | HOSTA_TMOE),
	/* W/C for next byte */
	HOSTA_NEXT_BYTE = HOSTA_BDS,
	/* W/C host status register */
	HOSTA_ALL_WC_BIT = (HOSTA_FINTR | HOSTA_ANY_ERROR | HOSTA_BDS),
};

enum i2c_reset_cause {
	I2C_RC_NO_IDLE_FOR_START = 1,
	I2C_RC_TIMEOUT,
};

static int i2c_parsing_return_value(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;

	if (!data->err) {
		return 0;
	}

	if (data->err == ETIMEDOUT) {
		/* Connection timed out */
		LOG_ERR("I2C ch%d Address:0x%X Transaction time out.",
			config->port, data->addr_16bit);
	} else {
		LOG_DBG("I2C ch%d Address:0x%X Host error bits message:",
			config->port, data->addr_16bit);
		/* Host error bits message*/
		if (data->err & HOSTA_TMOE) {
			LOG_ERR("Time-out error: hardware time-out error.");
		}
		if (data->err & HOSTA_NACK) {
			LOG_DBG("NACK error: device does not response ACK.");
		}
		if (data->err & HOSTA_FAIL) {
			LOG_ERR("Fail: a processing transmission is killed.");
		}
		if (data->err & HOSTA_BSER) {
			LOG_ERR("BUS error: SMBus has lost arbitration.");
		}
	}

	return -EIO;
}

static int i2c_get_line_levels(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;

	return (sys_read8(config->port_reg + SMB_SMBPCTL(config->port)) &
		(SMB_HSMBDCS | SMB_HSMBCS));
}

static int i2c_is_busy(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;

	return (sys_read8(config->port_reg + SMB_HOSTA) & (HOSTA_HOBY | HOSTA_ALL_WC_BIT));
}

static int i2c_bus_not_available(const struct device *dev)
{
	if (i2c_is_busy(dev) || (i2c_get_line_levels(dev) != SMB_LINE_IDLE)) {
		return -EIO;
	}

	return 0;
}

static void i2c_reset(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;

	/* bit1, kill current transaction. */
	sys_write8(SMB_KILL, config->port_reg + SMB_HOCTL);
	sys_write8(0, config->port_reg + SMB_HOCTL);
	/* W/C host status register */
	sys_write8(HOSTA_ALL_WC_BIT, config->port_reg + SMB_HOSTA);
}

void i2c_r_last_byte(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	uint8_t hoctl_val;

	/*
	 * bit5, The firmware shall write 1 to this bit
	 * when the next byte will be the last byte for i2c read.
	 */
	if ((data->msg->flags & I2C_MSG_STOP) && (data->ridx == data->msg->len - 1)) {
		hoctl_val = sys_read8(config->port_reg + SMB_HOCTL);
		sys_write8(hoctl_val | SMB_LABY, config->port_reg + SMB_HOCTL);
	}
}

void i2c_w2r_change_direction(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	uint8_t hoctl2_val;

	/* I2C switch direction */
	if (sys_read8(config->port_reg + SMB_HOCTL2(config->port)) & I2C_SW_EN) {
		i2c_r_last_byte(dev);
		sys_write8(SMB_BDS, config->port_reg + SMB_HOSTA);
	} else {
		hoctl2_val = sys_read8(config->port_reg + SMB_HOCTL2(config->port));
		sys_write8(hoctl2_val | I2C_SW_EN | I2C_SW_WAIT, config->port_reg +
			   SMB_HOCTL2(config->port));

		sys_write8(SMB_BDS, config->port_reg + SMB_HOSTA);
		i2c_r_last_byte(dev);

		hoctl2_val = sys_read8(config->port_reg + SMB_HOCTL2(config->port));
		sys_write8(hoctl2_val & ~I2C_SW_WAIT, config->port_reg + SMB_HOCTL2(config->port));
	}
}

int i2c_tran_read(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;

	if (data->msg->flags & SMB_MSG_START) {
		/* I2C enable */
		sys_write8(SMB_SMD_TO_EN | I2C_EN | SMB_SMH_EN, config->port_reg +
			   SMB_HOCTL2(config->port));

		sys_write8((uint8_t)(data->addr_16bit << 1) | SMB_DIR,
			   config->port_reg + SMB_TRASLA);
		/* Clear start flag */
		data->msg->flags &= ~SMB_MSG_START;

		if ((data->msg->len == 1) && (data->msg->flags & I2C_MSG_STOP)) {
			sys_write8(SMB_SRT | SMB_LABY | SMB_SMCD(7) | SMB_INTREN,
				   config->port_reg + SMB_HOCTL);
		} else {
			sys_write8(SMB_SRT | SMB_SMCD(7) | SMB_INTREN,
				   config->port_reg + SMB_HOCTL);
		}
	} else {
		if ((data->i2ccs == I2C_CH_REPEAT_START) || (data->i2ccs == I2C_CH_WAIT_READ)) {
			if (data->i2ccs == I2C_CH_REPEAT_START) {
				/* Write to read */
				i2c_w2r_change_direction(dev);
			} else {
				/* For last byte */
				i2c_r_last_byte(dev);
				/* W/C for next byte */
				sys_write8(SMB_BDS, config->port_reg + SMB_HOSTA);
			}
			data->i2ccs = I2C_CH_NORMAL;
		} else if (sys_read8(config->port_reg + SMB_HOSTA) & SMB_BDS) {
			if (data->ridx < data->msg->len) {
				/* To get received data. */
				*(data->msg->buf++) =
					sys_read8(config->port_reg + SMB_HOBDB(config->port));
				data->ridx++;
				/* For last byte */
				i2c_r_last_byte(dev);
				/* Done */
				if (data->ridx == data->msg->len) {
					data->msg->len = 0;
					if (data->msg->flags & I2C_MSG_STOP) {
						/* W/C for finish */
						sys_write8(SMB_BDS, config->port_reg + SMB_HOSTA);

						data->stop = 1;
					} else {
						data->i2ccs = I2C_CH_WAIT_READ;
						return 0;
					}
				} else {
					/* W/C for next byte */
					sys_write8(SMB_BDS, config->port_reg + SMB_HOSTA);
				}
			}
		}
	}

	return 1;
}

int i2c_tran_write(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;

	if (data->msg->flags & SMB_MSG_START) {
		/* I2C enable */
		sys_write8(SMB_SMD_TO_EN | I2C_EN | SMB_SMH_EN, config->port_reg +
			   SMB_HOCTL2(config->port));

		sys_write8((uint8_t)(data->addr_16bit << 1), config->port_reg + SMB_TRASLA);
		/* Send first byte */
		sys_write8(*(data->msg->buf++), config->port_reg + SMB_HOBDB(config->port));

		data->widx++;
		/* Clear start flag */
		data->msg->flags &= ~SMB_MSG_START;

		sys_write8(SMB_SRT | SMB_SMCD(7) | SMB_INTREN, config->port_reg + SMB_HOCTL);
	} else {
		/* Host has completed the transmission of a byte */
		if (sys_read8(config->port_reg + SMB_HOSTA) & SMB_BDS) {
			if (data->widx < data->msg->len) {
				/* Send next byte */
				sys_write8(*(data->msg->buf++),
					   config->port_reg + SMB_HOBDB(config->port));

				data->widx++;
				/* W/C byte done for next byte */
				sys_write8(SMB_BDS, config->port_reg + SMB_HOSTA);

				if (data->i2ccs == I2C_CH_REPEAT_START) {
					data->i2ccs = I2C_CH_NORMAL;
				}
			} else {
				/* Done */
				data->msg->len = 0;
				if (data->msg->flags & I2C_MSG_STOP) {
					/* Set I2C_EN = 0 */
					sys_write8(SMB_SMD_TO_EN | SMB_SMH_EN,
						   config->port_reg + SMB_HOCTL2(config->port));
					/* W/C byte done for finish */
					sys_write8(SMB_BDS, config->port_reg + SMB_HOSTA);

					data->stop = 1;
				} else {
					data->i2ccs = I2C_CH_REPEAT_START;
					return 0;
				}
			}
		}
	}

	return 1;
}

int i2c_pio_transaction(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;

	/* Any error */
	if (sys_read8(config->port_reg + SMB_HOSTA) & HOSTA_ANY_ERROR) {
		data->err = sys_read8(config->port_reg + SMB_HOSTA) & HOSTA_ANY_ERROR;
	} else {
		if (!data->stop) {
			/*
			 * The return value indicates if there is more data to be read or written.
			 * If the return value = 1, it means that the interrupt cannot be disable
			 * and continue to transmit data.
			 */
			if (data->msg->flags & I2C_MSG_READ) {
				return i2c_tran_read(dev);
			} else {
				return i2c_tran_write(dev);
			}
		}
		/* Wait finish */
		if (!(sys_read8(config->port_reg + SMB_HOSTA) & SMB_FINTR)) {
			return 1;
		}
	}
	/* W/C */
	sys_write8(HOSTA_ALL_WC_BIT, config->port_reg + SMB_HOSTA);

	/* Disable the SMBus host interface */
	sys_write8(0, config->port_reg + SMB_HOCTL2(config->port));

	data->stop = 0;
	/* Done doing work */
	return 0;
}

#ifdef CONFIG_I2C_IT51XXX_FIFO_MODE
void i2c_fifo_en_w2r(const struct device *dev, bool enable)
{
	const struct i2c_it51xxx_config *config = dev->config;
	unsigned int key = irq_lock();
	uint8_t i2cw2rf_val;

	i2cw2rf_val = sys_read8(config->i2cbase + SMB_I2CW2RF);

	if (enable) {
		if (config->port == SMB_CHANNEL_A) {
			sys_write8(i2cw2rf_val | SMB_MAIF | SMB_MAIFI,
				   config->i2cbase + SMB_I2CW2RF);
		} else if ((config->port >= SMB_CHANNEL_B) && (config->port <= SMB_CHANNEL_D)) {
			sys_write8(i2cw2rf_val | SMB_MBCDEFIF | BIT(config->port),
				   config->i2cbase + SMB_I2CW2RF);
		} else if (config->port >= SMB_CHANNEL_E) {
			sys_write8(i2cw2rf_val | SMB_MBCDEFIF, config->i2cbase + SMB_I2CW2RF);
			sys_write8(sys_read8(config->i2cbase + SMB_IWRFISTA) |
				   BIT(config->port + 2),
				   config->i2cbase + SMB_IWRFISTA);
		}
	} else {
		if (config->port == SMB_CHANNEL_A) {
			sys_write8(i2cw2rf_val & ~(SMB_MAIF | SMB_MAIFI),
				   config->i2cbase + SMB_I2CW2RF);
		} else if ((config->port >= SMB_CHANNEL_B) && (config->port <= SMB_CHANNEL_D)) {
			sys_write8(i2cw2rf_val & ~(SMB_MBCDEFIF | BIT(config->port)),
				   config->i2cbase + SMB_I2CW2RF);
		} else if (config->port >= SMB_CHANNEL_E) {
			sys_write8(i2cw2rf_val & ~SMB_MBCDEFIF, config->i2cbase + SMB_I2CW2RF);
			sys_write8(sys_read8(config->i2cbase + SMB_IWRFISTA) &
				   ~BIT(config->port + 2),
				   config->i2cbase + SMB_IWRFISTA);
		}
	}

	irq_unlock(key);
}

void i2c_tran_fifo_write_start(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	uint32_t i;
	uint8_t mstfctrl;

	mstfctrl = (config->port == SMB_CHANNEL_A) ? SMB_MSTFCTRL1 : SMB_MSTFCTRL2;

	/* Clear start flag. */
	data->msg->flags &= ~SMB_MSG_START;
	/* Enable SMB channel in FIFO mode. */
	sys_write8(sys_read8(config->i2cbase + mstfctrl) | SMB_FFEN, config->i2cbase + mstfctrl);
	/* I2C enable. */
	sys_write8(SMB_SMD_TO_EN | I2C_EN | SMB_SMH_EN,
		   config->port_reg + SMB_HOCTL2(config->port));
	/* Set write byte counts. */
	sys_write8(data->msg->len, config->port_reg + SMB_D0REG);
	/* Set transmit target address */
	sys_write8((uint8_t)(data->addr_16bit << 1), config->port_reg + SMB_TRASLA);

	/* The maximum fifo size is 32 bytes. */
	data->bytecnt = MIN(data->msg->len, SMB_FIFO_MODE_MAX_SIZE);
	for (i = 0; i < data->bytecnt; i++) {
		/* Set host block data byte. */
		sys_write8(*(data->msg->buf++), config->port_reg + SMB_HOBDB(config->port));
	}
	/* Calculate the remaining byte counts. */
	data->bytecnt = data->msg->len - data->bytecnt;

	/* Set host control */
	sys_write8(SMB_SRT | SMB_SMCD(7) | SMB_INTREN, config->port_reg + SMB_HOCTL);
}

void i2c_tran_fifo_write_next_block(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	uint32_t i, _bytecnt;
	uint8_t mstfctrl;

	mstfctrl = (config->port == SMB_CHANNEL_A) ? SMB_MSTFCTRL1 : SMB_MSTFCTRL2;

	/* The maximum fifo size is 32 bytes. */
	_bytecnt = MIN(data->bytecnt, SMB_FIFO_MODE_MAX_SIZE);
	for (i = 0; i < _bytecnt; i++) {
		/* Set host block data byte. */
		sys_write8(*(data->msg->buf++), config->port_reg + SMB_HOBDB(config->port));
	}

	/* Clear FIFO block done status. */
	sys_write8(sys_read8(config->i2cbase + mstfctrl) | SMB_BLKDS, config->i2cbase + mstfctrl);

	/* Calculate the remaining byte counts. */
	data->bytecnt -= _bytecnt;
}

void i2c_tran_fifo_write_finish(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;

	/* Clear byte count register. */
	sys_write8(0, config->port_reg + SMB_D0REG);
	/* W/C */
	sys_write8(HOSTA_ALL_WC_BIT, config->port_reg + SMB_HOSTA);
	/* Disable the SMBus host interface. */
	sys_write8(0, config->port_reg + SMB_HOCTL2(config->port));
}

int i2c_tran_fifo_w2r_change_direction(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	uint8_t hoctl2_val;

	if (++data->msg_index >= data->num_msgs) {
		LOG_ERR("Current message index is error.");
		data->err = EINVAL;
		/* W/C */
		sys_write8(HOSTA_ALL_WC_BIT, config->port_reg + SMB_HOSTA);
		/* Disable the SMBus host interface. */
		sys_write8(0, config->port_reg + SMB_HOCTL2(config->port));

		return 0;
	}

	/* Set I2C_SW_EN = 1 */
	hoctl2_val = sys_read8(config->port_reg + SMB_HOCTL2(config->port));
	sys_write8(hoctl2_val | I2C_SW_EN | I2C_SW_WAIT,
		   config->port_reg + SMB_HOCTL2(config->port));

	hoctl2_val = sys_read8(config->port_reg + SMB_HOCTL2(config->port));
	sys_write8(hoctl2_val & ~I2C_SW_WAIT, config->port_reg + SMB_HOCTL2(config->port));

	/* Point to the next msg for the read location. */
	data->msg = &data->msgs_list[data->msg_index];
	/* Set read byte counts. */
	sys_write8(data->msg->len, config->port_reg + SMB_D0REG);
	data->bytecnt = data->msg->len;

	/* W/C I2C W2R FIFO interrupt status. */
	sys_write8(BIT(config->port), config->i2cbase + SMB_IWRFISTA);

	return 1;
}

void i2c_tran_fifo_read_start(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	uint8_t mstfctrl;

	mstfctrl = (config->port == SMB_CHANNEL_A) ? SMB_MSTFCTRL1 : SMB_MSTFCTRL2;

	/* Clear start flag. */
	data->msg->flags &= ~SMB_MSG_START;
	/* Enable SMB channel in FIFO mode. */
	sys_write8(sys_read8(config->i2cbase + mstfctrl) | SMB_FFEN, config->i2cbase + mstfctrl);
	/* I2C enable. */
	sys_write8(SMB_SMD_TO_EN | I2C_EN | SMB_SMH_EN,
		   config->port_reg + SMB_HOCTL2(config->port));
	/* Set read byte counts. */
	sys_write8(data->msg->len, config->port_reg + SMB_D0REG);
	/* Set transmit target address */
	sys_write8((uint8_t)(data->addr_16bit << 1) | SMB_DIR, config->port_reg + SMB_TRASLA);

	data->bytecnt = data->msg->len;

	/* Set host control */
	sys_write8(SMB_SRT | SMB_SMCD(7) | SMB_INTREN, config->port_reg + SMB_HOCTL);
}

void i2c_tran_fifo_read_next_block(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	uint32_t i;
	uint8_t mstfctrl;

	mstfctrl = (config->port == SMB_CHANNEL_A) ? SMB_MSTFCTRL1 : SMB_MSTFCTRL2;

	for (i = 0; i < SMB_FIFO_MODE_MAX_SIZE; i++) {
		/* To get received data. */
		*(data->msg->buf++) = sys_read8(config->port_reg + SMB_HOBDB(config->port));
	}
	/* Clear FIFO block done status. */
	sys_write8(sys_read8(config->i2cbase + mstfctrl) | SMB_BLKDS, config->i2cbase + mstfctrl);

	/* Calculate the remaining byte counts. */
	data->bytecnt -= SMB_FIFO_MODE_MAX_SIZE;
}

void i2c_tran_fifo_read_finish(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	uint32_t i;

	for (i = 0; i < data->bytecnt; i++) {
		/* To get received data. */
		*(data->msg->buf++) = sys_read8(config->port_reg + SMB_HOBDB(config->port));
	}

	/* Clear byte count register. */
	sys_write8(0, config->port_reg + SMB_D0REG);
	/* W/C */
	sys_write8(HOSTA_ALL_WC_BIT, config->port_reg + SMB_HOSTA);
	/* Disable the SMBus host interface. */
	sys_write8(0, config->port_reg + SMB_HOCTL2(config->port));
}

int i2c_tran_fifo_write_to_read(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	int ret = 1;
	uint8_t mstfctrl;

	mstfctrl = (config->port == SMB_CHANNEL_A) ? SMB_MSTFCTRL1 : SMB_MSTFCTRL2;

	if (data->msg->flags & SMB_MSG_START) {
		/* Enable I2C write to read FIFO mode. */
		i2c_fifo_en_w2r(dev, true);
		i2c_tran_fifo_write_start(dev);
	} else {
		/* Check block done status. */
		if (sys_read8(config->i2cbase + mstfctrl) & SMB_BLKDS) {
			if (sys_read8(config->port_reg + SMB_HOCTL2(config->port)) & I2C_SW_EN) {
				i2c_tran_fifo_read_next_block(dev);
			} else {
				i2c_tran_fifo_write_next_block(dev);
			}
		} else if (sys_read8(config->i2cbase + SMB_IWRFISTA) & BIT(config->port)) {
			/*
			 * This function returns 0 on a failure to indicate that the current
			 * transaction is completed and returned the data->err.
			 */
			ret = i2c_tran_fifo_w2r_change_direction(dev);
		} else {
			/* Wait finish. */
			if (sys_read8(config->port_reg + SMB_HOSTA) & HOSTA_FINTR) {
				i2c_tran_fifo_read_finish(dev);
				/* Done doing work. */
				ret = 0;
			}
		}
	}

	return ret;
}

int i2c_tran_fifo_read(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	uint8_t mstfctrl;

	mstfctrl = (config->port == SMB_CHANNEL_A) ? SMB_MSTFCTRL1 : SMB_MSTFCTRL2;

	if (data->msg->flags & SMB_MSG_START) {
		i2c_tran_fifo_read_start(dev);
	} else {
		/* Check block done status. */
		if (sys_read8(config->i2cbase + mstfctrl) & SMB_BLKDS) {
			i2c_tran_fifo_read_next_block(dev);
		} else {
			/* Wait finish. */
			if (sys_read8(config->port_reg + SMB_HOSTA) & HOSTA_FINTR) {
				i2c_tran_fifo_read_finish(dev);
				/* Done doing work. */
				return 0;
			}
		}
	}

	return 1;
}

int i2c_tran_fifo_write(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	uint8_t mstfctrl;

	mstfctrl = (config->port == SMB_CHANNEL_A) ? SMB_MSTFCTRL1 : SMB_MSTFCTRL2;

	if (data->msg->flags & SMB_MSG_START) {
		i2c_tran_fifo_write_start(dev);
	} else {
		/* Check block done status. */
		if (sys_read8(config->i2cbase + mstfctrl) & SMB_BLKDS) {
			i2c_tran_fifo_write_next_block(dev);
		} else {
			/* Wait finish. */
			if (sys_read8(config->port_reg + SMB_HOSTA) & HOSTA_FINTR) {
				i2c_tran_fifo_write_finish(dev);
				/* Done doing work. */
				return 0;
			}
		}
	}

	return 1;
}

int i2c_fifo_transaction(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;

	/* Any error. */
	if (sys_read8(config->port_reg + SMB_HOSTA) & HOSTA_ANY_ERROR) {
		data->err = (sys_read8(config->port_reg + SMB_HOSTA) & HOSTA_ANY_ERROR);
	} else {
		if (data->num_msgs == 2) {
			return i2c_tran_fifo_write_to_read(dev);
		} else if (data->msg->flags & I2C_MSG_READ) {
			return i2c_tran_fifo_read(dev);
		} else {
			return i2c_tran_fifo_write(dev);
		}
	}
	/* W/C */
	sys_write8(HOSTA_ALL_WC_BIT, config->port_reg + SMB_HOSTA);
	/* Disable the SMBus host interface. */
	sys_write8(0, config->port_reg + SMB_HOCTL2(config->port));

	return 0;
}

bool fifo_mode_allowed(const struct device *dev, struct i2c_msg *msgs)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;

	/*
	 * If the transaction of write or read is divided into two transfers(not two messages),
	 * the FIFO mode does not support.
	 */
	if (data->i2ccs != I2C_CH_NORMAL) {
		return false;
	}
	/*
	 * FIFO2 only supports one channel of B or C. If the FIFO of channel is not enabled,
	 * it will select PIO mode.
	 */
	if (!config->fifo_enable) {
		return false;
	}
	/*
	 * When there is only one message, use the FIFO mode transfer directly.
	 * Transfer payload too long (>255 bytes), use PIO mode.
	 * Write or read of I2C target address without data, used by cmd_i2c_scan. Use PIO mode.
	 */
	if (data->num_msgs == 1 && (msgs[0].flags & I2C_MSG_STOP) &&
	    (msgs[0].len <= SMB_FIFO_MODE_TOTAL_LEN) && (msgs[0].len != 0)) {
		return true;
	}
	/*
	 * When there are two messages, we need to judge whether or not there is I2C_MSG_RESTART
	 * flag from the second message, and then decide todo the FIFO mode or PIO mode transfer.
	 */
	if (data->num_msgs == 2) {
		/*
		 * The first of two messages must be write.
		 * Transfer payload too long (>255 bytes), use PIO mode.
		 */
		if (((msgs[0].flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) &&
		    (msgs[0].len <= SMB_FIFO_MODE_TOTAL_LEN)) {
			/*
			 * The transfer is i2c_burst_read().
			 *
			 * e.g. msg[0].flags = I2C_MSG_WRITE;
			 *      msg[1].flags = I2C_MSG_RESTART | I2C_MSG_READ |
			 *                     I2C_MSG_STOP;
			 */
			if ((msgs[1].flags == SMB_MSG_BURST_READ_MASK) &&
			    (msgs[1].len <= SMB_FIFO_MODE_TOTAL_LEN)) {
				return true;
			}
		}
	}

	return false;
}
#endif /* CONFIG_I2C_IT51XXX_FIFO_MODE */

#ifdef CONFIG_I2C_TARGET
static void target_i2c_isr_fifo(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	const struct i2c_target_callbacks *target_cb = data->target_cfg->callbacks;
	uint32_t count, len;
	uint8_t sdfpctl;
	uint8_t target_status, fifo_status;

	target_status = sys_read8(config->port_reg + SMB_SLSTA(config->port));
	fifo_status = sys_read8(config->i2cbase + SMB_SFFSTA(config->port));
	/* bit0-4 : FIFO byte count */
	count = fifo_status & GENMASK(4, 0);

	/* Any error */
	if (target_status & SMB_STS) {
		data->w_index = 0;
		data->r_index = 0;
		goto done;
	}
	/* Target data status, the register is waiting for read or write. */
	if (target_status & SMB_SDS) {
		if (target_status & SMB_RCS) {
			uint8_t *rdata = NULL;

			/* Read data callback function */
			target_cb->buf_read_requested(data->target_cfg, &rdata, &len);
			if (len > sizeof(data->out_buffer)) {
				LOG_ERR("The length exceeds target out_buffer size=%d",
					sizeof(data->out_buffer));
				goto done;
			} else {
				memcpy(data->out_buffer, rdata, len);
			}

			for (int i = 0; i < SMB_TARGET_IT51XXX_MAX_FIFO_SIZE; i++) {
				/* Host receiving, target transmitting */
				sys_write8(data->out_buffer[i + data->r_index],
					   config->port_reg + SMB_SLDA(config->port));
			}
			/* Index to next 16 bytes of read buffer */
			data->r_index += SMB_TARGET_IT51XXX_MAX_FIFO_SIZE;
		} else {
			for (int i = 0; i < count; i++) {
				/* Host transmitting, target receiving */
				data->in_buffer[i + data->w_index] =
					sys_read8(config->port_reg + SMB_SLDA(config->port));
			}
			/* Write data done callback function */
			target_cb->buf_write_received(data->target_cfg, data->in_buffer, count);
			/* Index to next 16 bytes of write buffer */
			data->w_index += count;
			if (data->w_index > sizeof(data->in_buffer)) {
				LOG_ERR("The write size exceeds target in_buffer size=%d",
					sizeof(data->out_buffer));
				goto done;
			}
		}
	}
	/* Stop condition, indicate stop condition detected. */
	if (target_status & SMB_SPDS) {
		/* Read data less 16 bytes status */
		if (target_status & SMB_RCS) {
			/* Disable FIFO mode to clear left count */
			sdfpctl = sys_read8(config->i2cbase + SMB_SDFPCTL(config->port));
			sys_write8(sdfpctl & ~SMB_SADFE,
				   config->i2cbase + SMB_SDFPCTL(config->port));
			/* Peripheral A FIFO Enable */
			sdfpctl = sys_read8(config->i2cbase + SMB_SDFPCTL(config->port));
			sys_write8(sdfpctl | SMB_SADFE,
				   config->i2cbase + SMB_SDFPCTL(config->port));
		} else {
			for (int i = 0; i < count; i++) {
				/* Host transmitting, target receiving */
				data->in_buffer[i + data->w_index] =
					sys_read8(config->port_reg + SMB_SLDA(config->port));
			}
			/* Write data done callback function */
			target_cb->buf_write_received(data->target_cfg, data->in_buffer, count);
		}

		/* Transfer done callback function */
		target_cb->stop(data->target_cfg);

		data->w_index = 0;
		data->r_index = 0;
	}

done:
	/* W/C */
	sys_write8(target_status, config->port_reg + SMB_SLSTA(config->port));
}

static void target_i2c_isr_pio(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	const struct i2c_target_callbacks *target_cb = data->target_cfg->callbacks;
	uint8_t target_status;
	uint8_t val;

	target_status = sys_read8(config->port_reg + SMB_SLSTA(config->port));

	/* Any error */
	if (target_status & SMB_STS) {
		data->w_index = 0;
		data->r_index = 0;
		goto done;
	}
	if (target_status & SMB_SDS) {
		if (target_status & SMB_RCS) {
			if (config->target_shared_fifo_mode) {
				uint32_t len;
				uint8_t *rdata = NULL;

				/* Read data callback function */
				target_cb->buf_read_requested(data->target_cfg, &rdata, &len);
				if (len > sizeof(data->shared_fifo_size)) {
					LOG_ERR("The length exceeds fifo size=%d",
						sizeof(data->shared_fifo_size));
					goto done;
				} else {
					memcpy(data->shared_fifo_size, rdata, len);
				}
			} else {
				/* Host receiving, target transmitting */
				if (!data->r_index) {
					target_cb->read_requested(data->target_cfg, &val);
				} else {
					target_cb->read_processed(data->target_cfg, &val);
				}
				/* Write data */
				sys_write8(val, config->port_reg + SMB_SLDA(config->port));
				/* Release clock pin */
				sys_write8(val, config->port_reg + SMB_SLDA(config->port));
				data->r_index++;
			}
		} else {
			/* Host transmitting, target receiving */
			if (!data->w_index) {
				target_cb->write_requested(data->target_cfg);
			}
			/* Read data */
			val = sys_read8(config->port_reg + SMB_SLDA(config->port));
			if(!target_cb->write_received(data->target_cfg, val)) {
				/* Release clock pin */
				val = sys_read8(config->port_reg + SMB_SLDA(config->port));
			}
			data->w_index++;
		}
	}
	/* Stop condition, indicate stop condition detected. */
	if (target_status & SMB_SPDS) {
		/* Transfer done callback function */
		target_cb->stop(data->target_cfg);
		data->w_index = 0;
		data->r_index = 0;

		if (config->target_shared_fifo_mode) {
			uint8_t sffe;

			/* Shared FIFO for target disable */
			sffe = sys_read8(config->i2cbase + SMB_SFFE);
			sys_write8(sffe & ~BIT(config->port + 4), config->i2cbase + SMB_SFFE);
		}
	}

done:
	/* W/C */
	sys_write8(target_status, config->port_reg + SMB_SLSTA(config->port));
}

static void target_i2c_isr(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;

	if (config->target_fifo_mode) {
		target_i2c_isr_fifo(dev);
	} else {
		target_i2c_isr_pio(dev);
	}
}
#endif

static void i2c_it51xxx_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;

#ifdef CONFIG_I2C_TARGET
	if (data->target_attached) {
		target_i2c_isr(dev);
	} else
#endif
	{
#ifdef CONFIG_I2C_IT51XXX_FIFO_MODE
		uint8_t mstfctrl;

		mstfctrl = (config->port == SMB_CHANNEL_A) ? SMB_MSTFCTRL1 : SMB_MSTFCTRL2;
		/* If done doing work, wake up the task waiting for the transfer. */
		if (config->fifo_enable && (sys_read8(config->i2cbase + mstfctrl) & SMB_FFEN)) {
			if (i2c_fifo_transaction(dev)) {
				return;
			}
		} else
#endif
		{
			if (i2c_pio_transaction(dev)) {
				return;
			}
		}
		irq_disable(config->i2c_irq_base);
		k_sem_give(&data->device_sync_sem);
	}
}

static void i2c_standard_port_timing_regs_400khz(const struct device *dev, uint8_t port)
{
	const struct i2c_it51xxx_config *config = dev->config;

	/* Port clock frequency depends on setting of timing registers. */
	sys_write8(0, config->i2cbase + SMB_SCLKTS(port));
	/* Suggested setting of timing registers of 400kHz. */
	sys_write8(0x05, config->i2cbase + SMB_4P7USL);
	sys_write8(0x01, config->i2cbase + SMB_4P0USL);
	sys_write8(0x03, config->i2cbase + SMB_300NS);
	sys_write8(0x03, config->i2cbase + SMB_250NS);
	sys_write8(0xc9, config->i2cbase + SMB_45P3USLREG);
	sys_write8(0x01, config->i2cbase + SMB_45P3USHREG);
	sys_write8(0x00, config->i2cbase + SMB_4P7A4P0H);
}

static void i2c_standard_port_set_frequency(const struct device *dev, int freq_hz, int freq_set)
{
	const struct i2c_it51xxx_config *config = dev->config;
	uint8_t honacksrc;

	/*
	 * If port's clock frequency is 400kHz, we use timing registers for setting. So we can
	 * adjust tlow to meet timing. The others use basic 50/100/1000 KHz setting.
	 */
	if (freq_hz == I2C_BITRATE_FAST) {
		i2c_standard_port_timing_regs_400khz(dev, config->port);
	} else {
		sys_write8(freq_set, config->i2cbase + SMB_SCLKTS(config->port));
	}

	/* Host SMCLK & SMDAT timeout disable */
	honacksrc = sys_read8(config->i2cbase + SMB_HONACKSRC(config->port));
	sys_write8(honacksrc | SMB_HSMCDTD, config->i2cbase + SMB_HONACKSRC(config->port));
}

static int i2c_it51xxx_configure(const struct device *dev, uint32_t dev_config_raw)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *const data = dev->data;
	int ret;
	uint32_t freq_set;

	if (!(I2C_MODE_CONTROLLER & dev_config_raw)) {
		return -EINVAL;
	}

	if (I2C_ADDR_10_BITS & dev_config_raw) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config_raw)) {
	case I2C_SPEED_DT:
		freq_set = SMB_CLKS_50K;
		break;
	case I2C_SPEED_STANDARD:
		freq_set = SMB_CLKS_100K;
		break;
	case I2C_SPEED_FAST:
		freq_set = SMB_CLKS_400K;
		break;
	case I2C_SPEED_FAST_PLUS:
		freq_set = SMB_CLKS_1M;
		break;
	default:
		return -EINVAL;
	}

	i2c_standard_port_set_frequency(dev, config->bitrate, freq_set);

	if (I2C_SPEED_GET(dev_config_raw) == I2C_SPEED_STANDARD ||
	    I2C_SPEED_GET(dev_config_raw) == I2C_SPEED_FAST) {
		ret = i2c_bitbang_configure(&data->bitbang, dev_config_raw);
	} else {
		data->bitbang.dev_config = dev_config_raw;
		ret = 0;
	}

	return ret;
}

static int i2c_it51xxx_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_it51xxx_data *const data = dev->data;
	int ret;

	ret = i2c_bitbang_get_config(&data->bitbang, dev_config);
	if (ret < 0) {
		LOG_ERR("I2C controller not configured: %d", ret);
	}

	return ret;
}

static int i2c_it51xxx_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			        uint16_t addr)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	int ret;

#ifdef CONFIG_I2C_TARGET
	if (data->target_attached) {
		LOG_ERR("Device is registered as target");
		return -EBUSY;
	}
#endif
	/* Lock mutex of i2c controller */
	k_mutex_lock(&data->mutex, K_FOREVER);
	/*
	 * If the transaction of write to read is divided into two transfers, the repeat start
	 * transfer uses this flag to exclude checking bus busy.
	 */
	if (data->i2ccs == I2C_CH_NORMAL) {
		/* Make sure we're in a good state to start */
		if (i2c_bus_not_available(dev)) {
			/* Recovery I2C bus */
			i2c_recover_bus(dev);
			/*
			 * After resetting I2C bus, if I2C bus is not available
			 * (No external pull-up), drop the transaction.
			 */
			if (i2c_bus_not_available(dev)) {
				/* Unlock mutex of i2c controller */
				k_mutex_unlock(&data->mutex);
				return -EIO;
			}
		}

		msgs[0].flags |= SMB_MSG_START;
	}

#ifdef CONFIG_I2C_IT51XXX_FIFO_MODE
	/* Store num_msgs to data struct. */
	data->num_msgs = num_msgs;
	/* Store msgs to data struct. */
	data->msgs_list = msgs;
	data->msg_index = 0;

	bool fifo_mode_enable = fifo_mode_allowed(dev, msgs);
	if (fifo_mode_enable) {
		/* Block to enter power policy. */
		pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	}
#endif
	for (int i = 0; i < num_msgs; i++) {
		data->widx = 0;
		data->ridx = 0;
		data->err = 0;
		data->msg = &msgs[i];
		data->addr_16bit = addr;

#ifdef CONFIG_I2C_IT51XXX_FIFO_MODE
		/*
		 * Start transaction.
		 * The return value indicates if the initial configuration
		 * of I2C transaction for read or write has been completed.
		 */
		if (fifo_mode_enable) {
			if (i2c_fifo_transaction(dev)) {
				/* Enable i2c interrupt */
				irq_enable(config->i2c_irq_base);
			}
		} else
#endif
		{
			if (i2c_pio_transaction(dev)) {
				/* Enable i2c interrupt */
				irq_enable(config->i2c_irq_base);
			}
		}
		/* Wait for the transfer to complete */
		ret = k_sem_take(&data->device_sync_sem, K_MSEC(config->transfer_timeout_ms));
		/*
		 * The irq will be enabled at the condition of start or repeat start of I2C.
		 * If timeout occurs without being wake up during suspend(ex: interrupt is not
		 * fired), the irq should be disabled immediately.
		 */
		irq_disable(config->i2c_irq_base);
		/*
		 * The transaction is dropped on any error(timeout, NACK, fail,bus error,
		 * device error).
		 */
		if (data->err) {
			break;
		}

		if (ret != 0) {
			data->err = ETIMEDOUT;
			/* Reset i2c port */
			i2c_reset(dev);
			LOG_ERR("I2C ch%d:0x%X reset cause %d",
				config->port, data->addr_16bit, I2C_RC_TIMEOUT);
			/* If this message is sent fail, drop the transaction. */
			break;
		}

#ifdef CONFIG_I2C_IT51XXX_FIFO_MODE
		/* In FIFO mode, messages are compressed into a single transaction. */
		if (fifo_mode_enable) {
			break;
		}
#endif
	}
#ifdef CONFIG_I2C_IT51XXX_FIFO_MODE
	if (fifo_mode_enable) {
		uint8_t mstfctrl;

		mstfctrl = (config->port == SMB_CHANNEL_A) ? SMB_MSTFCTRL1 : SMB_MSTFCTRL2;

		/* Disable SMB channels in FIFO mode. */
		sys_write8(sys_read8(config->i2cbase + mstfctrl) & ~SMB_FFEN,
			   config->i2cbase + mstfctrl);

		/* Disable I2C write to read FIFO mode. */
		if (data->num_msgs == 2) {
			i2c_fifo_en_w2r(dev, false);
		}
		/* Permit to enter power policy. */
		pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	}
#endif
	/* Reset i2c channel status */
	if (data->err || (data->msg->flags & I2C_MSG_STOP)) {
		data->i2ccs = I2C_CH_NORMAL;
	}

	/* Save return value. */
	ret = i2c_parsing_return_value(dev);

	/* Unlock mutex of i2c controller */
	k_mutex_unlock(&data->mutex);

	return ret;
}

static void i2c_it51xxx_set_scl(void *io_context, int state)
{
	const struct i2c_it51xxx_config *config = io_context;

	gpio_pin_set_dt(&config->scl_gpios, state);
}

static void i2c_it51xxx_set_sda(void *io_context, int state)
{
	const struct i2c_it51xxx_config *config = io_context;

	gpio_pin_set_dt(&config->sda_gpios, state);
}

static int i2c_it51xxx_get_sda(void *io_context)
{
	const struct i2c_it51xxx_config *config = io_context;
	int ret = gpio_pin_get_dt(&config->sda_gpios);

	/* Default high as that would be a NACK */
	return ret != 0;
}

static const struct i2c_bitbang_io i2c_it51xxx_bitbang_io = {
	.set_scl = i2c_it51xxx_set_scl,
	.set_sda = i2c_it51xxx_set_sda,
	.get_sda = i2c_it51xxx_get_sda,
};

static int i2c_it51xxx_recover_bus(const struct device *dev)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	uint32_t bitrate_cfg;
	int ret;

	/* Output type selection */
	gpio_flags_t flags = GPIO_OUTPUT | (config->push_pull_recovery ? 0 : GPIO_OPEN_DRAIN);
	/* Set SCL of I2C as GPIO pin */
	gpio_pin_configure_dt(&config->scl_gpios, flags);
	/* Set SDA of I2C as GPIO pin */
	gpio_pin_configure_dt(&config->sda_gpios, flags);

	i2c_bitbang_init(&data->bitbang, &i2c_it51xxx_bitbang_io, (void *)config);

	bitrate_cfg = i2c_map_dt_bitrate(config->bitrate) | I2C_MODE_CONTROLLER;
	ret = i2c_bitbang_configure(&data->bitbang, bitrate_cfg);
	if (ret != 0) {
		LOG_ERR("failed to configure I2C bitbang (err %d)", ret);
		goto restore;
	}

	ret = i2c_bitbang_recover_bus(&data->bitbang);
	if (ret != 0) {
		LOG_ERR("failed to recover bus (err %d)", ret);
	}

restore:
	/* Set GPIO back to I2C alternate function of SCL */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to configure I2C pins");
		return ret;
	}

	/* Reset i2c port */
	i2c_reset(dev);
	LOG_ERR("I2C ch%d reset cause %d", config->port, I2C_RC_NO_IDLE_FOR_START);

	return 0;
}

#ifdef CONFIG_I2C_TARGET
static int i2c_it51xxx_target_register(const struct device *dev,
				       struct i2c_target_config *target_cfg)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;
	uint8_t hoctl2;

	if (!target_cfg) {
		return -EINVAL;
	}

	if (target_cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) {
		return -ENOTSUP;
	}

	if (data->target_attached) {
		return -EBUSY;
	}

	data->target_cfg = target_cfg;
	data->target_attached = true;

	/* Slave address[6:0] */
	sys_write8(target_cfg->address, config->port_reg + SMB_RESLADR(config->port));
	/* Reset i2c port */
	i2c_reset(dev);
	/* W/C all slave status */
	sys_write8(GENMASK(7, 0), config->port_reg + SMB_SLSTA(config->port));

	if (config->target_shared_fifo_mode) {
		uint32_t out_data_addr;

		chip_block_idle();

		memset(data->shared_fifo_size, 0, sizeof(data->shared_fifo_size));
		out_data_addr = (uint32_t)data->shared_fifo_size & GENMASK(23, 0);
		/* Define shared FIFO base address bit[11:4] */
		sys_write8((out_data_addr >> 4) & GENMASK(7, 0),
			   config->i2cbase + SMB_SFBAS(config->port));
		/* Define shared FIFO base address bit[17:12] */
		sys_write8((out_data_addr>> 12) & GENMASK(5, 0),
			   config->i2cbase + SMB_SFBAMS(config->port));
	}
	/* Block to enter power policy. */
	pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);

	/* Enable the SMBus slave device */
	hoctl2 = sys_read8(config->port_reg + SMB_HOCTL2(config->port));
	sys_write8(hoctl2 | SMB_SLVEN, config->port_reg + SMB_HOCTL2(config->port));

	ite_intc_isr_clear(config->i2cs_irq_base);
	irq_enable(config->i2cs_irq_base);

	return 0;
}

static int i2c_it51xxx_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
	const struct i2c_it51xxx_config *config = dev->config;
	struct i2c_it51xxx_data *data = dev->data;

	if (!data->target_attached) {
		return -EINVAL;
	}

	irq_disable(config->i2cs_irq_base);

	/* Permit to enter power policy and idle mode. */
	pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	if (config->target_shared_fifo_mode) {
		chip_permit_idle();
	}

	data->target_cfg = NULL;
	data->target_attached = false;

	return 0;
}
#endif /* CONFIG_I2C_TARGET */

static DEVICE_API(i2c, i2c_it51xxx_driver_api) = {
	.configure = i2c_it51xxx_configure,
	.get_config = i2c_it51xxx_get_config,
	.transfer = i2c_it51xxx_transfer,
	.recover_bus = i2c_it51xxx_recover_bus,
#ifdef CONFIG_I2C_TARGET
	.target_register = i2c_it51xxx_target_register,
	.target_unregister = i2c_it51xxx_target_unregister,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

static int i2c_it51xxx_init(const struct device *dev)
{
	struct i2c_it51xxx_data *data = dev->data;
	const struct i2c_it51xxx_config *config = dev->config;
	int error, status;
	uint32_t bitrate_cfg;
	uint8_t sdsic_val;

#ifdef CONFIG_I2C_TARGET
	if (config->target_enable) {
		uint8_t honacksrc;

		LOG_INF("[target]in_buffer=%p, out_buffer=%p\n", data->in_buffer, data->out_buffer);

		if (config->target_fifo_mode) {
			uint8_t sdfpctl;

			/* Target A or B or C FIFO Enable */
			sdfpctl = sys_read8(config->i2cbase + SMB_SDFPCTL(config->port));
			sys_write8(sdfpctl | SMB_SADFE, config->i2cbase +
				   SMB_SDFPCTL(config->port));
		} else if (config->target_shared_fifo_mode) {
			uint8_t sffe, ssfss1, ssfss2, target_fifo_size_val = 0;

			data->fifo_size_list = fifo_size_table;
			for (int i = 0; i <= ARRAY_SIZE(fifo_size_table); i++) {
				if (i == ARRAY_SIZE(fifo_size_table)) {
					LOG_ERR("Unsupported target FIFO size %d",
						sizeof(data->shared_fifo_size));
					return -ENOTSUP;
				}

				if (sizeof(data->shared_fifo_size) ==
				    data->fifo_size_list[i].fifo_size) {
					target_fifo_size_val = data->fifo_size_list[i].value;
					break;
				}
			}
			if (config->port == SMB_CHANNEL_A) {
				ssfss1 = sys_read8(config->i2cbase + SMB_SSFSS1);
				sys_write8(ssfss1 | SMB_SFSFSA(target_fifo_size_val),
					   config->i2cbase + SMB_SSFSS1);
			} else if (config->port == SMB_CHANNEL_B) {
				ssfss1 = sys_read8(config->i2cbase + SMB_SSFSS1);
				sys_write8(ssfss1 | SMB_SFSFSB(target_fifo_size_val),
					   config->i2cbase + SMB_SSFSS1);
			} else if (config->port == SMB_CHANNEL_C) {
				ssfss2 = sys_read8(config->i2cbase + SMB_SSFSS2);
				sys_write8(ssfss2 | SMB_SFSFSC(target_fifo_size_val),
					   config->i2cbase + SMB_SSFSS2);
			}
			/* Shared FIFO for target enable */
			sffe = sys_read8(config->i2cbase + SMB_SFFE);
			sys_write8(sffe | BIT(config->port + 4), config->i2cbase + SMB_SFFE);
		}

		/* Target SMCLK & SMDAT timeout disable */
		honacksrc = sys_read8(config->i2cbase + SMB_HONACKSRC(config->port));
		sys_write8(honacksrc | SMB_SSMCDTD, config->i2cbase + SMB_HONACKSRC(config->port));

		/* Slave interrupt control */
		sys_write8(SMB_SDSEN | SMB_SDLTOEN | SMB_SITEN | SMB_HONOIN,
			   config->port_reg + SMB_SICR(config->port));

		/* Target channelA-C switch selection of interface */
		if (config->port == SMB_CHANNEL_A) {
			sdsic_val = sys_read8(config->i2cbase + SMB_SDSIC4);
			sys_write8((sdsic_val & ~GENMASK(3, 0)) |
				   SMB_DSASTI(config->channel_switch_sel),
				   config->i2cbase + SMB_SDSIC4);
		} else if (config->port == SMB_CHANNEL_B) {
			sdsic_val = sys_read8(config->i2cbase + SMB_SDSIC4);
			sys_write8((sdsic_val & ~GENMASK(7, 4)) |
				   SMB_DSBSTI(config->channel_switch_sel),
				   config->i2cbase + SMB_SDSIC4);
		} else if (config->port == SMB_CHANNEL_C) {
			sdsic_val = sys_read8(config->i2cbase + SMB_SDSIC5);
			sys_write8((sdsic_val & ~GENMASK(7, 4)) |
				   SMB_DSCSTI(config->channel_switch_sel),
				   config->i2cbase + SMB_SDSIC5);
		}

		irq_connect_dynamic(config->i2cs_irq_base, 0, i2c_it51xxx_isr, dev, 0);

		goto pin_config;
	}
#endif
	/* Initialize mutex and semaphore */
	k_mutex_init(&data->mutex);
	k_sem_init(&data->device_sync_sem, 0, K_SEM_MAX_LIMIT);

	/* Enable SMBus function */
	sys_write8(SMB_SMD_TO_EN | SMB_SMH_EN, config->port_reg + SMB_HOCTL2(config->port));
	/* Kill SMBus host transaction. And enable the interrupt for the master interface */
	sys_write8(SMB_KILL | SMB_INTREN, config->port_reg + SMB_HOCTL);
	sys_write8(SMB_INTREN, config->port_reg + SMB_HOCTL);
	/* W/C host status register */
	sys_write8(HOSTA_ALL_WC_BIT, config->port_reg + SMB_HOSTA);
	sys_write8(0, config->port_reg + SMB_HOCTL2(config->port));

	/* Set clock frequency for I2C ports */
	if (config->bitrate == I2C_BITRATE_STANDARD || config->bitrate == I2C_BITRATE_FAST ||
	    config->bitrate == I2C_BITRATE_FAST_PLUS) {
		bitrate_cfg = i2c_map_dt_bitrate(config->bitrate);
	} else {
		/* Device tree specified speed */
		bitrate_cfg = I2C_SPEED_DT << I2C_SPEED_SHIFT;
	}

	error = i2c_it51xxx_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	data->i2ccs = I2C_CH_NORMAL;

	if (error) {
		LOG_ERR("i2c: failure initializing");
		return error;
	}

#ifdef CONFIG_I2C_IT51XXX_FIFO_MODE
	/* Select which port to use FIFO2 except port A */
	if ((config->port != SMB_CHANNEL_A) && config->fifo_enable) {
		sys_write8((config->port - 1), config->i2cbase + SMB_MSTFCTRL2);
	}
#endif
	/* Host channelA-F switch selection of interface */
	if (config->port == SMB_CHANNEL_A) {
		sdsic_val = sys_read8(config->i2cbase + SMB_SDSIC);
		sys_write8((sdsic_val & ~GENMASK(3, 0)) | SMB_DASTI(config->channel_switch_sel),
			   config->i2cbase + SMB_SDSIC);
	} else if (config->port == SMB_CHANNEL_B) {
		sdsic_val = sys_read8(config->i2cbase + SMB_SDSIC);
		sys_write8((sdsic_val & ~GENMASK(7, 4)) | SMB_DBSTI(config->channel_switch_sel),
			   config->i2cbase + SMB_SDSIC);
	} else if (config->port == SMB_CHANNEL_C) {
		sdsic_val = sys_read8(config->i2cbase + SMB_SDSIC2);
		sys_write8((sdsic_val & ~GENMASK(3, 0)) | SMB_DCSTI(config->channel_switch_sel),
			   config->i2cbase + SMB_SDSIC2);
	} else if (config->port == I2C_CHANNEL_D) {
		sdsic_val = sys_read8(config->i2cbase + SMB_SDSIC2);
		sys_write8((sdsic_val & ~GENMASK(7, 4)) | SMB_DDSTI(config->channel_switch_sel),
			   config->i2cbase + SMB_SDSIC2);
	} else if (config->port == I2C_CHANNEL_E) {
		sdsic_val = sys_read8(config->i2cbase + SMB_SDSIC3);
		sys_write8((sdsic_val & ~GENMASK(3, 0)) | SMB_DESTI(config->channel_switch_sel),
			   config->i2cbase + SMB_SDSIC3);
	} else if (config->port == I2C_CHANNEL_F) {
		sdsic_val = sys_read8(config->i2cbase + SMB_SDSIC3);
		sys_write8((sdsic_val & ~GENMASK(7, 4)) | SMB_DFSTI(config->channel_switch_sel),
			   config->i2cbase + SMB_SDSIC3);
	}

	irq_connect_dynamic(config->i2c_irq_base, 0, i2c_it51xxx_isr, dev, 0);

#ifdef CONFIG_I2C_TARGET
pin_config:
#endif
	/* Set the pin to I2C alternate function. */
	status = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (status < 0) {
		LOG_ERR("Failed to configure I2C pins");
		return status;
	}

	return 0;
}

#define I2C_ITE_IT51XXX_INIT(inst)                                                                 \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	BUILD_ASSERT((DT_INST_PROP(inst, clock_frequency) == 50000) ||                             \
		     (DT_INST_PROP(inst, clock_frequency) == I2C_BITRATE_STANDARD) ||              \
		     (DT_INST_PROP(inst, clock_frequency) == I2C_BITRATE_FAST) ||                  \
		     (DT_INST_PROP(inst, clock_frequency) == I2C_BITRATE_FAST_PLUS),               \
		     "Not support I2C bit rate value");                                            \
										                   \
	static const struct i2c_it51xxx_config i2c_it51xxx_cfg_##inst = {                          \
		.i2cbase = DT_REG_ADDR(DT_NODELABEL(i2cbase)),                                     \
		.port_reg = DT_INST_REG_ADDR_BY_IDX(inst, 0),                                      \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.scl_gpios = GPIO_DT_SPEC_INST_GET(inst, scl_gpios),                               \
		.sda_gpios = GPIO_DT_SPEC_INST_GET(inst, sda_gpios),                               \
		.transfer_timeout_ms = DT_INST_PROP(inst, transfer_timeout_ms),                    \
		.bitrate = DT_INST_PROP(inst, clock_frequency),                                    \
		.i2c_irq_base = DT_INST_IRQ_BY_IDX(inst, 0, irq),                                  \
		.i2cs_irq_base = DT_INST_IRQ_BY_IDX(inst, 1, irq),                                 \
		.port = DT_INST_PROP(inst, port_num),                                              \
		.channel_switch_sel = DT_INST_PROP(inst, channel_switch_sel),                      \
		.fifo_enable = DT_INST_PROP(inst, fifo_enable),                                    \
		.target_enable = DT_INST_PROP(inst, target_enable),                                \
		.target_fifo_mode = DT_INST_PROP(inst, target_fifo_mode),                          \
		.target_shared_fifo_mode = DT_INST_PROP(inst, target_shared_fifo_mode),            \
		.push_pull_recovery = DT_INST_PROP(inst, push_pull_recovery),                      \
	};                                                                                         \
										                   \
	static struct i2c_it51xxx_data i2c_it51xxx_data_##inst;                                    \
										                   \
	I2C_DEVICE_DT_INST_DEFINE(inst, i2c_it51xxx_init, NULL, &i2c_it51xxx_data_##inst,          \
				  &i2c_it51xxx_cfg_##inst, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,  \
				  &i2c_it51xxx_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_ITE_IT51XXX_INIT)
