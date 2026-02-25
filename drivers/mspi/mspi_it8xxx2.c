/*
 * Copyright (c) 2026 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it8xxx2_mspi

#include <zephyr/logging/log.h>
#if 0 /* TODO */
LOG_MODULE_REGISTER(mspi_it8xxx2, CONFIG_MSPI_LOG_LEVEL);
#else
LOG_MODULE_REGISTER(mspi_it8xxx2, LOG_LEVEL_ERR);
#endif

#include <zephyr/irq.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/policy.h>
#include <soc.h>

#define BYTE_0(x) (uint8_t)(((x) >> 0) & 0xFF)
#define BYTE_1(x) (uint8_t)(((x) >> 8) & 0xFF)
#define BYTE_2(x) (uint8_t)(((x) >> 16) & 0xFF)

#define SRAM_BASE_ADDR DT_REG_ADDR(DT_NODELABEL(sram0))

#define MSPI_MAX_DEVICE 2

#define SPI_CMDQ_WR_CMD_LEN_MAX 16
#define SPI_CMDQ_DATA_LEN_MAX   0xFFFF

#define MSPI01_CTRL1   0x01
#define CLOCK_POLARTY  BIT(6)
#define SSCK_FREQ_MASK (BIT(2) | BIT(3) | BIT(4))
#define INTERRUPT_EN   BIT(1)

#define MSPI04_CTRL3 0x04
#define AUTO_MODE    BIT(5)

#define MSPI05_CH0_CMD_ADDR_LB    0x05
#define MSPI06_CH0_CMD_ADDR_HB    0x06
#define MSPI0C_INT_STS            0x0C
#define SPI_CMDQ_BUS_END_INT_MASK BIT(4)
#define SPI_DMA_RBUF_1_FULL       BIT(2)
#define SPI_DMA_RBUF_0_FULL       BIT(1)
#define SPI_CMDQ_BUS_END          BIT(0)

#define MSPI0D_CTRL5      0x0D
#define CH1_SEL_CMDQ      BIT(5)
#define CH0_SEL_CMDQ      BIT(4)
#define SCK_FREQ_DIV_1_EN BIT(1)
#define CMDQ_MODE_EN      BIT(0)

#define MSPI0E_CH0_WR_MEM_ADDR_LB  0x0E
#define MSPI0F_CH0_WR_MEM_ADDR_HB  0x0F
#define MSPI12_CH1_CMD_ADDR_LB     0x12
#define MSPI13_CH1_CMD_ADDR_HB     0x13
#define MSPI14_CH1_WR_MEM_ADDR_LB  0x14
#define MSPI15_CH1_WR_MEM_ADDR_HB  0x15
#define MSPI21_CH0_CMD_ADDR_HB2    0x21
#define MSPI23_CH0_WR_MEM_ADDR_HB2 0x23
#define MSPI25_CH1_CMD_ADDR_HB2    0x25
#define MSPI27_CH1_WR_MEM_ADDR_HB2 0x27

#define ECPM_BASE_ADDR          DT_REG_ADDR(DT_NODELABEL(ecpm))
#define ECPM05_CLK_GATING_CTRL3 0x05
#define SSPI_CLOCK_GATING       BIT(1)

struct mspi_it8xxx2_cmdq_data {
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

	uint8_t cmd_data[SPI_CMDQ_WR_CMD_LEN_MAX];
};

struct mspi_it8xxx2_config {
	mm_reg_t base;

	const struct pinctrl_dev_config *pcfg;

	struct mspi_cfg mspicfg;

	void (*irq_cfg_func)(void);
	uint8_t spi_irq;
};

struct mspi_it8xxx2_data {
	const struct mspi_dev_id *dev_id;

	struct mspi_dev_cfg dev_cfg;

	struct k_mutex lock;
	struct mspi_xfer xfer;

	struct k_sem sem_sync;

	struct mspi_it8xxx2_cmdq_data cmdq_data;
};

static inline int set_freqency(const struct device *dev, const uint32_t frequency)
{
	const struct mspi_it8xxx2_config *cfg = dev->config;
	const uint8_t freq_div[8] = {2, 4, 6, 8, 10, 12, 14, 16};
	uint32_t clk_pll, clk_sspi;
	uint8_t reg_val;

	clk_pll = chip_get_pll_freq();
	clk_sspi = clk_pll / (((IT8XXX2_ECPM_SCDCR2 & 0xF0) >> 4) + 1U);
	if (frequency < (clk_sspi / 16) || frequency > clk_sspi) {
		LOG_ERR("unsupported frequency %d", frequency);
		return -ENOTSUP;
	}

	if (frequency == clk_sspi) {
		sys_write8(sys_read8(cfg->base + MSPI0D_CTRL5) | SCK_FREQ_DIV_1_EN,
			   cfg->base + MSPI0D_CTRL5);
	} else {
		for (int i = 0; i <= ARRAY_SIZE(freq_div); i++) {
			if (i == ARRAY_SIZE(freq_div)) {
				LOG_ERR("unknown frequency %d", frequency);
				return -ENOTSUP;
			}
			if (frequency == (clk_sspi / freq_div[i])) {
				sys_write8(sys_read8(cfg->base + MSPI0D_CTRL5) & ~SCK_FREQ_DIV_1_EN,
					   cfg->base + MSPI0D_CTRL5);
				reg_val = sys_read8(cfg->base + MSPI01_CTRL1);
				reg_val = (reg_val & (~SSCK_FREQ_MASK)) | (i << 2);
				sys_write8(reg_val, cfg->base + MSPI01_CTRL1);
				break;
			}
		}
	}

	LOG_DBG("freq: pll %dHz, sspi %dHz, ssck %dHz", clk_pll, clk_sspi, frequency);

	return 0;
}

static int mspi_it8xxx2_config(const struct mspi_dt_spec *spec)
{
	const struct mspi_cfg *config = &spec->config;
	const struct mspi_it8xxx2_config *cfg = spec->bus->config;
	struct mspi_it8xxx2_data *data = spec->bus->data;
	int ret;

	LOG_DBG("%s ite deubg %d", __func__, __LINE__);

	if (config->num_periph > MSPI_MAX_DEVICE) {
		LOG_ERR("invalid mspi peripheral number");
		return -ENOTSUP;
	}

	if (config->op_mode != MSPI_OP_MODE_CONTROLLER) {
		LOG_ERR("supported mspi controller mode only");
		return -ENOTSUP;
	}

#if 0
	if (config->max_freq > MSPI_MAX_FREQ) {
		LOG_INST_ERR(cfg->log, "%u, max_freq too large.", __LINE__);
		return -ENOTSUP;
	}
#endif

	if (config->duplex != MSPI_HALF_DUPLEX) {
		LOG_ERR("supported half duplex mode only");
		return -ENOTSUP;
	}

	if (config->dqs_support) {
		LOG_ERR("unsupported dqs mode");
		return -ENOTSUP;
	}

	if (config->re_init) {
		LOG_WRN("todo: ite debug re-init");
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("failed to apply pinctrl, %d", ret);
		return ret;
	}

	/* enable one-shot mode */
	sys_write8(sys_read8(cfg->base + MSPI04_CTRL3) & ~AUTO_MODE, cfg->base + MSPI04_CTRL3);

	cfg->irq_cfg_func();

	return 0;
}

static inline int mspi_dev_cfg_check_save(const struct device *dev,
					  const enum mspi_dev_cfg_mask param_mask,
					  const struct mspi_dev_cfg *dev_cfg)
{
	struct mspi_it8xxx2_data *data = dev->data;
	int ret;

	LOG_DBG("%s ite debug %d", __func__, __LINE__);

	if (param_mask & MSPI_DEVICE_CONFIG_CE_NUM) {
		data->dev_cfg.ce_num = dev_cfg->ce_num;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_FREQUENCY) {
		ret = set_freqency(dev, dev_cfg->freq);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

static int mspi_it8xxx2_dev_config(const struct device *dev, const struct mspi_dev_id *dev_id,
				   const enum mspi_dev_cfg_mask param_mask,
				   const struct mspi_dev_cfg *dev_cfg)
{
	const struct mspi_it8xxx2_config *cfg = dev->config;
	struct mspi_it8xxx2_data *data = dev->data;
	bool locked = false;
	int ret = 0;

	if (data->dev_id != dev_id) {
		if (k_mutex_lock(&data->lock, K_MSEC(CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE))) {
			LOG_ERR("failed to access mspi controller");
			return -EBUSY;
		}

		locked = true;
	}

	if (param_mask == MSPI_DEVICE_CONFIG_NONE && !cfg->mspicfg.sw_multi_periph) {
		/* do nothing but saving the device id */
		data->dev_id = dev_id;
		goto out;
	}

	if (param_mask < MSPI_DEVICE_CONFIG_ALL) {
		if (data->dev_id != dev_id) {
			/* MSPI_DEVICE_CONFIG_ALL should be used */
			LOG_ERR("failed to config device, must be the same device");
			ret = -ENOTSUP;
			goto out;
		}
		ret = mspi_dev_cfg_check_save(dev, param_mask, dev_cfg);
		if (ret) {
			goto out;
		}
	} else if (param_mask == MSPI_DEVICE_CONFIG_ALL) {
		ret = mspi_dev_cfg_check_save(dev, param_mask, dev_cfg);
		if (ret) {
			goto out;
		}
		if (data->dev_id != dev_id) {
			/* Conduct device switching */
		}
	} else {
		LOG_ERR("invalid configuration mask");
		ret = -EINVAL;
		goto out;
	}

	data->dev_id = dev_id;

#if 0
	switch (io_mode) {
	case MSPI_IO_MODE_SINGLE:

		break;

	case MSPI_IO_MODE_DUAL:
	case MSPI_IO_MODE_DUAL_1_1_2:
	case MSPI_IO_MODE_DUAL_1_2_2:

		break;

	case MSPI_IO_MODE_QUAD:
	case MSPI_IO_MODE_QUAD_1_1_4:
	case MSPI_IO_MODE_QUAD_1_4_4:

		break;
	default:
		ret = -ENOTSUP:
	}
#endif

out:
	if (locked) {
		k_mutex_unlock(&data->lock);
	}

	return ret;
}

static int mspi_it8xxx2_get_channel_status(const struct device *dev, uint8_t ch)
{
	LOG_WRN("todo: %s ite debug %d", __func__, __LINE__);

	return 0;
}

static inline void it8xxx2_mspi_xfer(const struct device *dev, const bool rx)
{
	const struct mspi_it8xxx2_config *cfg = dev->config;
	struct mspi_it8xxx2_data *data = dev->data;
	struct mspi_xfer *xfer = &data->xfer;
	uint32_t mem_address;
	size_t tx_len = xfer->cmd_length + xfer->addr_length;
	size_t rx_len = xfer->packets->num_bytes;

	if (tx_len > SPI_CMDQ_WR_CMD_LEN_MAX) {
		return -EINVAL;
	}

	data->cmdq_data.command.fields.cmd_end = 1;
	data->cmdq_data.command.fields.cs_active = 0;
	data->cmdq_data.command.fields.read_write = rx ? 1 : 0;

	data->cmdq_data.spi_write_cmd_length = tx_len;
	memcpy(data->cmdq_data.cmd_data, &xfer->packets->cmd, xfer->cmd_length);
	memcpy(data->cmdq_data.cmd_data + xfer->cmd_length, &xfer->packets->address, xfer->addr_length);
	LOG_HEXDUMP_DBG(data->cmdq_data.cmd_data, tx_len, "tx data:");

	data->cmdq_data.data_length_lb = BYTE_0(rx_len);
	data->cmdq_data.data_length_hb = BYTE_1(rx_len);
	data->cmdq_data.data_addr_lb = 0;
	data->cmdq_data.data_addr_hb = 0;
}

static int start_next_packet(const struct device *dev)
{
	const struct mspi_it8xxx2_config *cfg = dev->config;
	struct mspi_it8xxx2_data *data = dev->data;
	struct mspi_xfer *xfer = &data->xfer;
	bool data_only_packet = !xfer->cmd_length && !xfer->addr_length;
	uint32_t cmd_address, mem_address;
	uint8_t reg_val;

	if (data_only_packet && xfer->packets->num_bytes == 0) {
		/* nothing to do */
		return 0;
	}

	LOG_DBG("chip select: %d", data->dev_cfg.ce_num);
	for (uint32_t i = 0; i < xfer->num_packet; i++) {
		LOG_DBG("%d: dir|cmd|addr|nbyte: %d|%d|%d|%d", i, xfer->packets->dir,
			xfer->cmd_length, xfer->addr_length, xfer->packets->num_bytes);
		if (xfer->cmd_length) {
			LOG_HEXDUMP_INF(&xfer->packets->cmd, xfer->cmd_length, "cmd:");
		}
		if (xfer->addr_length) {
			LOG_HEXDUMP_INF(&xfer->packets->address, xfer->addr_length, "addr:");
		}

		// if (xfer->packets->num_bytes) {
		// 	LOG_HEXDUMP_DBG(xfer->packets->data_buf, xfer->packets->num_bytes, "data:");
		// }
	}

	memset(&data->cmdq_data, 0, sizeof(struct mspi_it8xxx2_cmdq_data));

	switch (xfer->packets->dir) {
	case MSPI_TX:
		LOG_WRN("todo: ite debug %d tx only", __LINE__);
		it8xxx2_mspi_xfer(dev, false);
		/* tx only, nothing to rx */
		break;
	case MSPI_RX:
		if (data_only_packet) {
			LOG_WRN("todo: ite debug %d rx only", __LINE__);
			/* rx only, nothing to tx */
			return -ENOTSUP;
		} else {
			/* tx-rx */
			it8xxx2_mspi_xfer(dev, true);
		}
		break;
	default:
		LOG_ERR("unknown mspi operation");
		return -EINVAL;
	}

	cmd_address = (uint32_t)(&data->cmdq_data) - SRAM_BASE_ADDR;
	mem_address = (uint32_t)xfer->packets->data_buf - SRAM_BASE_ADDR;
	if (data->dev_cfg.ce_num == 0) {
		sys_write8(BYTE_0(cmd_address), cfg->base + MSPI05_CH0_CMD_ADDR_LB);
		sys_write8(BYTE_1(cmd_address), cfg->base + MSPI06_CH0_CMD_ADDR_HB);
		sys_write8(BYTE_2(cmd_address), cfg->base + MSPI21_CH0_CMD_ADDR_HB2);

		if (xfer->packets->num_bytes) {
			sys_write8(BYTE_0(mem_address), cfg->base + MSPI0E_CH0_WR_MEM_ADDR_LB);
			sys_write8(BYTE_1(mem_address), cfg->base + MSPI0F_CH0_WR_MEM_ADDR_HB);
			sys_write8(BYTE_2(mem_address), cfg->base + MSPI23_CH0_WR_MEM_ADDR_HB2);
		}
	} else {
		sys_write8(BYTE_0(cmd_address), cfg->base + MSPI12_CH1_CMD_ADDR_LB);
		sys_write8(BYTE_1(cmd_address), cfg->base + MSPI13_CH1_CMD_ADDR_HB);
		sys_write8(BYTE_2(cmd_address), cfg->base + MSPI25_CH1_CMD_ADDR_HB2);

		if (xfer->packets->num_bytes) {
			sys_write8(BYTE_0(mem_address), cfg->base + MSPI14_CH1_WR_MEM_ADDR_LB);
			sys_write8(BYTE_1(mem_address), cfg->base + MSPI15_CH1_WR_MEM_ADDR_HB);
			sys_write8(BYTE_2(mem_address), cfg->base + MSPI27_CH1_WR_MEM_ADDR_HB2);
		}
	}

	sys_write8(sys_read8(cfg->base + MSPI01_CTRL1) | INTERRUPT_EN, cfg->base + MSPI01_CTRL1);

	reg_val = sys_read8(cfg->base + MSPI0D_CTRL5);
	reg_val |= (data->dev_cfg.ce_num == 0) ? CH0_SEL_CMDQ : CH1_SEL_CMDQ;
	sys_write8(reg_val | CMDQ_MODE_EN, cfg->base + MSPI0D_CTRL5);

	/* TODO:
	 * only the first packet is started for mspi async, next ones, if any,
	 * are started by isr.
	 */
#if 0
	switch (xfer->xfer_mode) {
	case MSPI_PIO:
		LOG_DBG("%s ite debug %d: pio mode is selected", __func__, __LINE__);
		break;
	case MSPI_DMA:
#ifdef CONFIG_MSPI_DMA
		LOG_DBG("%s ite debug %d: dma mode is selected", __func__, __LINE__);
#else
		LOG_ERR("dma mode(it82xx2 cmdq mode) is disabled");
#endif /* CONFIG_MSPI_DMA */
		break;
	default:
		break;
	}
#endif
	if (k_sem_take(&data->sem_sync, K_FOREVER)) {
		LOG_ERR("failed to lock sync sem");
	}

	return 0;
}

static int mspi_it8xxx2_transceive(const struct device *dev, const struct mspi_dev_id *dev_id,
				   const struct mspi_xfer *req)
{
	const struct mspi_it8xxx2_config *cfg = dev->config;
	struct mspi_it8xxx2_data *data = dev->data;

	sys_write8(sys_read8(cfg->base + MSPI0C_INT_STS) & (~SPI_CMDQ_BUS_END_INT_MASK),
		   cfg->base + MSPI0C_INT_STS);

	if (dev_id != data->dev_id) {
		LOG_ERR("controller is not configured for this device");
		return -EINVAL;
	}

	if (req->async && !IS_ENABLED(CONFIG_MULTITHREADING)) {
		LOG_ERR("asynchronous xfer require multithreading");
		return -ENOTSUP;
	}

#if defined(CONFIG_MULTITHREADING)
	if (k_mutex_lock(&data->lock, K_FOREVER)) {
		return -EBUSY;
	}
#endif

	LOG_DBG("ite debug %d mspi async %d", __LINE__, req->async);

	data->xfer = *req;

	start_next_packet(dev);

#if defined(CONFIG_MULTITHREADING)
	k_mutex_unlock(&data->lock);
#endif

	return 0;
}

static void it8xxx2_mspi_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct mspi_it8xxx2_config *cfg = dev->config;
	struct mspi_it8xxx2_data *data = dev->data;
	uint8_t int_sts, reg_val;
	int ret;

	int_sts = sys_read8(cfg->base + MSPI0C_INT_STS);

	if (int_sts & (SPI_DMA_RBUF_0_FULL | SPI_DMA_RBUF_1_FULL)) {
		LOG_INF("isr: triggered dma ring buffer full interrupt, status: 0x%x", reg_val);
	}

	if (int_sts & SPI_CMDQ_BUS_END) {
		LOG_DBG("isr: cs: %d cmdq xfer done", data->dev_cfg.ce_num);
		LOG_HEXDUMP_INF(data->xfer.packets->data_buf, data->xfer.packets->num_bytes,
				"write data:");
		LOG_HEXDUMP_ERR(data->cmdq_data.cmd_data, data->xfer.cmd_length + data->xfer.addr_length, "write data:");
		LOG_HEXDUMP_ERR(data->xfer.packets->data_buf, data->xfer.packets->num_bytes,
				"read data:");

		reg_val = sys_read8(cfg->base + MSPI0D_CTRL5);
		if (data->dev_cfg.ce_num == 0) {
			reg_val &= ~CH0_SEL_CMDQ;
		} else {
			reg_val &= ~CH1_SEL_CMDQ;
		}
		sys_write8(reg_val, cfg->base + MSPI0D_CTRL5);

		k_sem_give(&data->sem_sync);
	}

	sys_write8(int_sts, cfg->base + MSPI0C_INT_STS);
}

static DEVICE_API(mspi, mspi_it8xxx2_api) = {
	.config = mspi_it8xxx2_config,
	.dev_config = mspi_it8xxx2_dev_config,
	.get_channel_status = mspi_it8xxx2_get_channel_status,
	.transceive = mspi_it8xxx2_transceive,
#if 0
	.register_callback = mspi_it8xxx2_register_callback,
	.timing_config = mspi_it8xxx2_timing_config,
	.xip_config = mspi_it8xxx2_xip_config,
#endif
};

static int mspi_it8xxx2_init(const struct device *controller)
{
	LOG_DBG("%s ite debug %d", __func__, __LINE__);

	const struct mspi_it8xxx2_config *cfg = controller->config;
	const struct mspi_dt_spec spec = {
		.bus = controller,
		.config = cfg->mspicfg,
	};

	uint8_t reg_val = sys_read8(ECPM_BASE_ADDR + ECPM05_CLK_GATING_CTRL3);
	sys_write8(reg_val & ~SSPI_CLOCK_GATING, ECPM_BASE_ADDR + ECPM05_CLK_GATING_CTRL3);

	chip_block_idle();
	pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);

	return mspi_it8xxx2_config(&spec);
}

#define MSPI_CONFIG(n)                                                                             \
	{                                                                                          \
		.channel_num = 0,                                                                  \
		.op_mode = DT_INST_ENUM_IDX_OR(n, op_mode, MSPI_OP_MODE_CONTROLLER),               \
		.duplex = DT_INST_ENUM_IDX_OR(n, duplex, MSPI_HALF_DUPLEX),                        \
		.num_periph = DT_INST_CHILD_NUM(n),                                                \
	}

#define MSPI_IT8XXX2_INIT(n)                                                                       \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void mspi_it8xxx2_irq_cfg_func_##n(void)                                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), it8xxx2_mspi_isr,           \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static const struct mspi_it8xxx2_config mspi_it8xxx2_cfg_##n = {                           \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.mspicfg = MSPI_CONFIG(n),                                                         \
		.mspicfg.ce_group = NULL,                                                          \
		.mspicfg.num_ce_gpios = 0,                                                         \
		.mspicfg.re_init = false,                                                          \
		.spi_irq = DT_INST_IRQ(n, irq),                                                    \
		.irq_cfg_func = mspi_it8xxx2_irq_cfg_func_##n,                                     \
	};                                                                                         \
                                                                                                   \
	static struct mspi_it8xxx2_data mspi_it8xxx2_data_##n = {                                  \
		.lock = Z_MUTEX_INITIALIZER(mspi_it8xxx2_data_##n.lock),                           \
		.sem_sync = Z_SEM_INITIALIZER(mspi_it8xxx2_data_##n.sem_sync, 0, 1), \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &mspi_it8xxx2_init, NULL, &mspi_it8xxx2_data_##n,                 \
			      &mspi_it8xxx2_cfg_##n, POST_KERNEL, CONFIG_MSPI_INIT_PRIORITY,       \
			      &mspi_it8xxx2_api);

DT_INST_FOREACH_STATUS_OKAY(MSPI_IT8XXX2_INIT)
