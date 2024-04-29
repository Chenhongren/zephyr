/*
 * Copyright (c) 2024 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it8xxx2_spi

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_it8xxx2, CONFIG_SPI_LOG_LEVEL);

#include <zephyr/irq.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

#include "spi_context.h"

#define PINCTRL_STATE_QUAD_MODE 1

#define SPI_CHIP_SELECT_COUNT 2
#define SPI_MIN_DATA_WIDTH    1
#define SPI_MAX_DATA_WIDTH    8
/* Max data length for command queue mode is 64KB */
#define SPI_CMDQ_WR_DATA_MAX  16//64*1024 //TODO: check

/* ITE8xxx2 SSPI Registers Definition */
#define SPI00_SPIDATA  0x0
#define SPI01_CTRL1    0x01
#define CLOCK_POLARTY  BIT(6)
#define CLOCK_PHASE    BIT(5)
#define SSCK_FREQ_MASK (BIT(2) | BIT(3) | BIT(4))
#define INTERRUPT_EN   BIT(1)

#define SPI02_CTRL2     0x02
#define BYTE_WIDTH_MASK (BIT(3) | BIT(4) | BIT(5))
#define READ_CYCLE BIT(2)
#define BLOCK_SELECT BIT(1)

#define SPI03_STS          0x03
#define BUSY_WAIT_FUNC     BIT(7) //TODO: check
#define SPI_TRANS_END      BIT(5)
#define CH0_START          BIT(4)
#define CH1_START          BIT(3)
#define SPI_TRANS_END_FLAG BIT(1)

#define SPI04_CTRL3 0x04
#define AUTO_MODE   BIT(5)

#define SPI05_CH0_CMD_ADDR_LB     0x05
#define SPI06_CH0_CMD_ADDR_HB     0x06
#define SPI07_DMA_TRANS_CNT_LB    0x07
#define SPI08_DMA_TRANS_CNT_HB    0x08
#define SPI09_WR_CMD_LEN          0x09
#define SPI0A_CH0_DMA_RD_LB       0x0A
#define SPI0B_CH0_DMA_RD_HB       0x0B
#define SPI0C_INT_STS             0x0C
#define SPI_CMDQ_BUS_END_INT_MASK BIT(4)
#define SPI_CMDQ_BUS_END          BIT(0)

#define SPI0D_CTRL5       0x0D
#define CH1_SEL_CMDQ      BIT(5)
#define CH0_SEL_CMDQ      BIT(4)
#define SCK_FREQ_DIV_1_EN BIT(1)
#define CMDQ_MODE_EN      BIT(0)

#define SPI0E_CH0_WR_MEM_ADDR_LB          0x0E
#define SPI0F_CH0_WR_MEM_ADDR_HB          0x0F
#define SPI10_CMDQ_INTERVAL_TIME_PRESCALE 0x10
#define SPI11_CH0_WAIT_TIME_SCALE_REG     0x11
#define SPI12_CH1_CMD_ADDR_LB             0x12
#define SPI13_CH1_CMD_ADDR_HB             0x13
#define SPI14_CH1_WR_MEM_ADDR_LB          0x14
#define SPI15_CH1_WR_MEM_ADDR_HB          0x15
#define SPI16_CH1_WAIT_TIME_SCALE         0x16
#define SPI17_CH1_DMA_RD_LB               0x17
#define SPI18_CH1_DMA_RD_HB               0x18
#define SPI21_CH0_CMD_ADDR_HB2            0x21
#define SPI23_CH0_WR_MEM_ADDR_HB2         0x23
#define SPI25_CH1_CMD_ADDR_HB2            0x25
#define SPI27_CH1_WR_MEM_ADDR_HB2         0x27
#define SPI2D_DSFBCR                      0x2D
#define SPI2E_RECV_DATA                   0x2E
#define SPI33_CTRL6                       0x33

struct spi_it8xxx2_cmdq_data {
	uint8_t spi_write_cmd_length;

	union {
		uint8_t value;
		struct {
			uint8_t cmd_end: 1;
			uint8_t read_write: 1;
			uint8_t auto_check_sts: 1;
			uint8_t cs_active: 1;
			uint8_t reserved: 1;
			uint8_t cmd_mode: 2;
			uint8_t dtr: 1;
		} __packed fields;
	} __packed command;

	uint8_t data_length_lb;
	uint8_t data_length_hb;
	uint8_t data_addr_lb;
	uint8_t data_addr_hb;
	uint8_t check_bit_mask;
	uint8_t check_bit_value;

	uint8_t write_data[SPI_CMDQ_WR_DATA_MAX];
};

struct spi_it8xxx2_config {
	mm_reg_t base;
	const struct pinctrl_dev_config *pcfg;
	bool quad_mode_en;
	uint8_t spi_irq;
};

struct spi_it8xxx2_data {
	struct spi_context ctx;

	struct spi_it8xxx2_cmdq_data cmdq_data;
	uint16_t spi_cmdq_header_addr;

	struct k_sem spi_sem;
};

static inline int spi_it8xxx2_set_freq(const struct device *dev, const uint32_t frequency)
{
	const struct spi_it8xxx2_config *cfg = dev->config;
	uint8_t freq_div[8] = {2, 4, 6, 8, 10, 12, 14, 16};
	uint32_t clk_pll, clk_sspi;
	uint8_t reg_val;

	clk_pll = chip_get_pll_freq();
	clk_sspi = clk_pll / (((IT8XXX2_ECPM_SCDCR2 & 0xF0) >> 4) + 1U);
	if (frequency < (clk_sspi / 16) || frequency > clk_sspi) {
		LOG_ERR("Unsupported frequency %d", frequency);
		return -ENOTSUP;
	}

	if (frequency == clk_sspi) {
		reg_val = sys_read8(cfg->base + SPI0D_CTRL5);
		reg_val |= SCK_FREQ_DIV_1_EN;
		sys_write8(reg_val, cfg->base + SPI0D_CTRL5);
	} else {
		for (int i = 0; i <= ARRAY_SIZE(freq_div); i++) {
			if (i == ARRAY_SIZE(freq_div)) {
				LOG_ERR("Unknown frequency %d", frequency);
				return -ENOTSUP;
			}
			if (frequency == (clk_sspi / freq_div[i])) {
				reg_val = sys_read8(cfg->base + SPI01_CTRL1);
				reg_val = (reg_val & (~SSCK_FREQ_MASK)) | (i << 2);
				sys_write8(reg_val, cfg->base + SPI01_CTRL1);
				break;
			}
		}
	}

	LOG_DBG("freq: pll %dHz, sspi %dHz, ssck %dHz", clk_pll, clk_sspi, frequency);

	return 0;
}

static inline int spi_it8xxx2_set_line_mode(const struct device *dev, uint32_t operation)
{
	const struct spi_it8xxx2_config *cfg = dev->config;
	struct spi_it8xxx2_data *data = dev->data;

	switch (operation & SPI_LINES_MASK) {
	case SPI_LINES_SINGLE:
		LOG_DBG("%s ITE Debug [%d] - single mode", __func__, __LINE__);
		data->cmdq_data.command.fields.cmd_mode = 0;
		break;
	case SPI_LINES_DUAL:
		LOG_DBG("%s ITE Debug [%d] - dual mode", __func__, __LINE__);
		data->cmdq_data.command.fields.cmd_mode = 2;
		break;
	case SPI_LINES_QUAD:
		LOG_DBG("%s ITE Debug [%d] - quad mode", __func__, __LINE__);
		if (cfg->quad_mode_en) {
			data->cmdq_data.command.fields.cmd_mode = 1;
		} else {
			LOG_ERR("Quad mode is not supported");
			return -ENOTSUP;
		}
		break;
	case SPI_LINES_OCTAL:
		__fallthrough;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static int spi_it8xxx2_configure(const struct device *dev, const struct spi_config *spi_cfg)
{
	const struct spi_it8xxx2_config *cfg = dev->config;
	struct spi_it8xxx2_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret;
	uint8_t reg_val;

	if (spi_context_configured(ctx, spi_cfg)) {
		//LOG_DBG("%s ITE Debug [%d] - already configured", __func__, __LINE__);
		return 0;
	}

	/* software flow control */
	if (spi_cs_is_gpio(spi_cfg)) {
		/* TODO: disable hardware CS pins */
		LOG_DBG("%s ITE Debug [%d] - disable hardware CS pins", __func__, __LINE__);
	}

	if (spi_cfg->slave > SPI_CHIP_SELECT_COUNT) {
		LOG_ERR("Slave %d is greater than %d", spi_cfg->slave, SPI_CHIP_SELECT_COUNT);
		return -EINVAL;
	}

	LOG_DBG("chip select: %d, operation: 0x%x", spi_cfg->slave, spi_cfg->operation);
	if (SPI_OP_MODE_GET(spi_cfg->operation) == SPI_OP_MODE_SLAVE) {
		LOG_ERR("Unsupported SPI slave mode");
		return -ENOTSUP;
	}

	reg_val = sys_read8(cfg->base + SPI01_CTRL1);
	if (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPOL) {
		reg_val |= CLOCK_POLARTY;
	} else {
		reg_val &= ~CLOCK_POLARTY;
	}
	if (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPHA) {
		reg_val |= CLOCK_PHASE;
	} else {
		reg_val &= ~CLOCK_PHASE;
	}
	sys_write8(reg_val, cfg->base + SPI01_CTRL1);

	if (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_LOOP) {
		LOG_ERR("Unsupported the loopback mode");
		return -ENOTSUP;
	}

	uint32_t word_size = SPI_WORD_SIZE_GET(spi_cfg->operation);
	if (word_size < SPI_MIN_DATA_WIDTH || word_size > SPI_MAX_DATA_WIDTH) {
		LOG_ERR("The word size %d is invalid", word_size);
		return -ENOTSUP;
	}
	reg_val = sys_read8(cfg->base + SPI02_CTRL2);
	reg_val &= ~BYTE_WIDTH_MASK;
	if (word_size != 8) {
		reg_val |= ((word_size) << 3);
	}
	sys_write8(reg_val, cfg->base + SPI02_CTRL2);

	if (spi_cfg->operation & SPI_HALF_DUPLEX) {
		// TODO: need to check
		LOG_DBG("%s ITE Debug [%d] - half duplex is selected", __func__, __LINE__);
		// printk("Half-duplex not supported");
		// return -ENOTSUP;
	}

	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES)) {
		ret = spi_it8xxx2_set_line_mode(dev, spi_cfg->operation);
		if (ret < 0) {
			return ret;
		}
	}

	ret = spi_it8xxx2_set_freq(dev, spi_cfg->frequency);
	if (ret) {
		return ret;
	}

	/* Set one-shot mode */
	reg_val = sys_read8(cfg->base + SPI04_CTRL3);
	reg_val &= ~AUTO_MODE;
	sys_write8(reg_val, cfg->base + SPI04_CTRL3);

	reg_val = sys_read8(cfg->base + SPI0C_INT_STS);
	reg_val = (reg_val & (~SPI_CMDQ_BUS_END_INT_MASK)) | SPI_CMDQ_BUS_END;
	sys_write8(reg_val, cfg->base + SPI0C_INT_STS);

	ctx->config = spi_cfg;
	return 0;
}

static void spi_it8xxx2_xfer(const struct device *dev)
{
	const struct spi_it8xxx2_config *cfg = dev->config;
	struct spi_it8xxx2_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint8_t reg_val;

	if (!spi_context_tx_buf_on(ctx) && !spi_context_rx_buf_on(ctx)) {
		spi_context_complete(ctx, dev, 0);
		return;
	}

	data->spi_cmdq_header_addr = (uint32_t)(&data->cmdq_data);
	ite_intc_isr_clear(cfg->spi_irq);
	irq_enable(cfg->spi_irq);
	reg_val = sys_read8(cfg->base + SPI01_CTRL1);
	reg_val |= INTERRUPT_EN;
	sys_write8(reg_val, cfg->base + SPI01_CTRL1);

	data->cmdq_data.command.fields.cmd_end = 1;
	data->cmdq_data.check_bit_mask = 0;
	data->cmdq_data.check_bit_value = 0;
	memcpy(data->cmdq_data.write_data, ctx->tx_buf, ctx->tx_len);

	LOG_HEXDUMP_DBG(ctx->tx_buf, ctx->tx_len, "tx:");

	if (spi_context_total_rx_len(ctx) == 0) {
		// TODO: TX only
		printk("ITE Debug TX only\n");
	} else {
		size_t rx_len = spi_context_total_rx_len(ctx);
		size_t tx_len = spi_context_total_tx_len(ctx);
		uint8_t rx_buf_temp[rx_len - tx_len];

		data->cmdq_data.spi_write_cmd_length = ctx->tx_len;
		data->cmdq_data.command.fields.read_write = 1;
		data->cmdq_data.data_length_lb = rx_len - ctx->tx_len;
		data->cmdq_data.data_length_hb = 0;
		data->cmdq_data.data_addr_lb = 0;
		data->cmdq_data.data_addr_hb = 0;

		if (ctx->config->slave == 0) {
				sys_write8((uint8_t)(data->spi_cmdq_header_addr & 0x000000FF),
					   cfg->base + SPI05_CH0_CMD_ADDR_LB);
				sys_write8((uint8_t)((data->spi_cmdq_header_addr & 0x0000FF00) >> 8),
					   cfg->base + SPI06_CH0_CMD_ADDR_HB);

				sys_write8((uint8_t)((uint32_t)rx_buf_temp & 0x000000FF),
					   cfg->base + SPI0E_CH0_WR_MEM_ADDR_LB);
				sys_write8((uint8_t)(((uint32_t)rx_buf_temp & 0x0000FF00) >> 8),
					   cfg->base + SPI0F_CH0_WR_MEM_ADDR_HB);
		} else {
				sys_write8((uint8_t)(data->spi_cmdq_header_addr & 0x000000FF),
					   cfg->base + SPI12_CH1_CMD_ADDR_LB);
				sys_write8((uint8_t)((data->spi_cmdq_header_addr & 0x0000FF00) >> 8),
					   cfg->base + SPI13_CH1_CMD_ADDR_HB);

				sys_write8((uint8_t)((uint32_t)rx_buf_temp & 0x000000FF),
					   cfg->base + SPI14_CH1_WR_MEM_ADDR_LB);
				sys_write8((uint8_t)(((uint32_t)rx_buf_temp & 0x0000FF00) >> 8),
					   cfg->base + SPI15_CH1_WR_MEM_ADDR_HB);
		}

		chip_block_idle();

		reg_val = sys_read8(cfg->base + SPI0D_CTRL5);
		reg_val |= (ctx->config->slave == 0)? CH0_SEL_CMDQ:CH1_SEL_CMDQ;
		sys_write8(reg_val | CMDQ_MODE_EN, cfg->base + SPI0D_CTRL5);

		k_sem_take(&data->spi_sem, K_FOREVER);

		if (ctx->rx_buf == ctx->tx_buf) {
			spi_context_update_rx(ctx, 1, ctx->rx_len);
			memcpy(ctx->rx_buf, rx_buf_temp, ctx->rx_len);
		} else {
			memcpy(ctx->rx_buf, rx_buf_temp, ctx->rx_len);
		}
		LOG_HEXDUMP_DBG(ctx->rx_buf, ctx->rx_len, "rx:");
	}

	spi_context_cs_control(ctx, false);

	reg_val = sys_read8(cfg->base + SPI0C_INT_STS);
	reg_val |= SPI_CMDQ_BUS_END;
	sys_write8(reg_val, cfg->base + SPI0C_INT_STS);

	spi_context_complete(ctx, dev, 0);
}

static int it8xxx2_transceive(const struct device *dev, const struct spi_config *spi_cfg,
			      const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	struct spi_it8xxx2_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}
	if (!rx_bufs) {
		LOG_ERR("RX BUF NULL");
	}

	spi_context_lock(ctx, false, NULL, NULL, spi_cfg);

	/* configure spi */
	ret = spi_it8xxx2_configure(dev, spi_cfg);
	if (ret) {
		spi_context_release(ctx, ret);
		return ret;
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(ctx, true);

	spi_it8xxx2_xfer(dev);

	ret = spi_context_wait_for_completion(ctx);

	spi_context_release(ctx, ret);

	return ret;
}

#ifdef CONFIG_SPI_ASYNC
static int it8xxx2_transceive_async(const struct device *dev, const struct spi_config *spi_cfg,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				    void *userdata)
{
	LOG_DBG("%s ITE Debug [%d]", __func__, __LINE__);

	/* TODO: support async callback function */
	return -ENOTSUP;
}
#endif /* CONFIG_SPI_ASYNC */

static int it8xxx2_release(const struct device *dev, const struct spi_config *config)
{
	LOG_DBG("%s ITE Debug [%d]", __func__, __LINE__);

	struct spi_it8xxx2_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static void it8xxx2_spi_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct spi_it8xxx2_config *cfg = dev->config;
	struct spi_it8xxx2_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint8_t reg_val;

	irq_disable(cfg->spi_irq);

	reg_val = sys_read8(cfg->base + SPI0C_INT_STS);
	reg_val |= SPI_CMDQ_BUS_END;
	sys_write8(reg_val, cfg->base + SPI0C_INT_STS);

	ite_intc_isr_clear(cfg->spi_irq);

	reg_val = sys_read8(cfg->base + SPI0D_CTRL5);
	if (ctx->config->slave == 0) {
		reg_val &= ~CH0_SEL_CMDQ;
	} else {
		reg_val &= ~CH1_SEL_CMDQ;
	}
	sys_write8(reg_val, cfg->base + SPI0D_CTRL5);

	k_sem_give(&data->spi_sem);
	chip_permit_idle();
}

static int spi_it8xxx2_init(const struct device *dev)
{
	const struct spi_it8xxx2_config *cfg = dev->config;
	struct spi_it8xxx2_data *data = dev->data;
	int err;

	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		LOG_ERR("Failed to set default pinctrl");
		return err;
	}

	if (cfg->quad_mode_en) {
		printk("%s ITE Debug [%d] - quad mode enable\n", __func__, __LINE__);
		err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_QUAD_MODE);
		if (err) {
			LOG_ERR("Failed to set quad mode pinctrl");
			return err;
		}
	}

	k_sem_init(&data->spi_sem, 0, 1);

	irq_connect_dynamic(cfg->spi_irq, 0, it8xxx2_spi_isr, dev, 0);

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		LOG_ERR("%s ITE Debug [%d] - Failed to configure CS GPIO pins", __func__,
		       __LINE__);
		return err;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct spi_driver_api spi_it8xxx2_driver_api = {
	.transceive = it8xxx2_transceive,
	.release = it8xxx2_release,

#ifdef CONFIG_SPI_ASYNC
	.transceive_async = it8xxx2_transceive_async,
#endif
};

#define SPI_IT8XXX2_INIT(n)                                                                        \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct spi_it8xxx2_config spi_it8xxx2_cfg_##n = {                             \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.spi_irq = DT_INST_IRQ_BY_IDX(n, 0, irq),                                          \
		.quad_mode_en = DT_INST_PROP_OR(inst, quad_mode, false),                   \
	};                                                                                         \
                                                                                                   \
	static struct spi_it8xxx2_data spi_it8xxx2_data_##n = {                                    \
		SPI_CONTEXT_INIT_LOCK(spi_it8xxx2_data_##n, ctx),                                  \
		SPI_CONTEXT_INIT_SYNC(spi_it8xxx2_data_##n, ctx),                                  \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &spi_it8xxx2_init, NULL, &spi_it8xxx2_data_##n,                   \
			      &spi_it8xxx2_cfg_##n, POST_KERNEL,                                   \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &spi_it8xxx2_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_IT8XXX2_INIT)