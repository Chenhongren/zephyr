/*
 * Copyright (c) 2026 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <soc_dt.h>

#include <zephyr/drivers/ps2.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/policy.h>

#include <zephyr/drivers/interrupt_controller/wuc_ite_it51xxx.h>

#define DT_DRV_COMPAT ite_it51xxx_ps2

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_instance.h>
#define LOG_LEVEL CONFIG_PS2_LOG_LEVEL
LOG_MODULE_REGISTER(ps2_it51xxx);

#define PS200_CTRL_REG       0x0
#define DEBOUNCE_ENABLE      BIT(4)
#define TX_MODE_SELECTION    BIT(3)
#define HARDWARE_MODE_ENABLE BIT(2)
#define CTRL_CLK_LINE        BIT(1)
#define CTRL_DATA_LINE       BIT(0)

#define PS204_INT_CTRL_REG          0x4
#define TIMEOUT_INT_ENABLE          BIT(3)
#define TRANSACTION_DONE_INT_ENABLE BIT(2)
#define START_INT_ENABLE            BIT(1)
#define SOFTWARE_MODE_INT_ENABLE    BIT(0)

#define PS208_STATUS      0x8
#define BIT_TIMEOUT_ERR   6
#define BIT_FRAME_ERR     5
#define BIT_PARITY_ERR    4
#define XFER_ERROR_MASK   GENMASK(BIT_TIMEOUT_ERR, BIT_PARITY_ERR)
#define TRANSACTIOIN_DONE BIT(3)
#define START_STATUS      BIT(2)
#define CLK_LINE_STATUS   BIT(1)
#define DATA_LINE_STATUS  BIT(0)
#define BUS_IDLE          (CLK_LINE_STATUS | DATA_LINE_STATUS)

#define PS20C_DATA_REG 0xC

/* in 50us units */
#define PS2_RETRY_COUNT 10000

struct it51xxx_wuc {
	const struct device *dev;

	/* wuc pin mask */
	uint8_t mask;
};

struct it51xxx_ps2_data {
	ps2_callback_t callback_isr;
	struct k_sem lock;
	struct k_sem tx_sem;

	int xfer_status;
};

struct it51xxx_ps2_config {
	const struct pinctrl_dev_config *pcfg;
	mm_reg_t base;

	const struct it51xxx_wuc wuc;
	uint8_t wu_irq; /* ps2 clk wu interrupt */
	uint8_t irq_num;

	void (*irq_config_func)(const struct device *dev);
};

#define DEBUG_GPIOA0 1 /* TODO: REMOVEME */
#define DEBUG_GPIOA1 1 /* TODO: REMOVEME */
#define DEBUG_GPIOA2 1 /* TODO: REMOVEME */
#define DEBUG_GPIOA3 1 /* TODO: REMOVEME */

#if DEBUG_GPIOA0
static void it51xxx_toggle_gpioa0(void)
{
	uint8_t gpioa_val;

	sys_write8(0x40, 0xf01610);

	gpioa_val = sys_read8(0xf01601);

	if (gpioa_val & BIT(0)) {
		gpioa_val &= ~BIT(0);
	} else {
		gpioa_val |= BIT(0);
	}

	sys_write8(gpioa_val, 0xf01601);
}
#endif

#if DEBUG_GPIOA1
static void it51xxx_toggle_gpioa1(void)
{
	uint8_t gpioa_val;

	sys_write8(0x40, 0xf01611);

	gpioa_val = sys_read8(0xf01601);

	if (gpioa_val & BIT(1)) {
		gpioa_val &= ~BIT(1);
	} else {
		gpioa_val |= BIT(1);
	}

	sys_write8(gpioa_val, 0xf01601);
}
#endif

#if DEBUG_GPIOA2
static void it51xxx_toggle_gpioa2(void)
{
	uint8_t gpioa_val;

	sys_write8(0x40, 0xf01612);

	gpioa_val = sys_read8(0xf01601);

	if (gpioa_val & BIT(2)) {
		gpioa_val &= ~BIT(2);
	} else {
		gpioa_val |= BIT(2);
	}

	sys_write8(gpioa_val, 0xf01601);
}
#endif

#if DEBUG_GPIOA3
static void it51xxx_toggle_gpioa3(void)
{
	uint8_t gpioa_val;

	sys_write8(0x40, 0xf01613);

	gpioa_val = sys_read8(0xf01601);

	if (gpioa_val & BIT(3)) {
		gpioa_val &= ~BIT(3);
	} else {
		gpioa_val |= BIT(3);
	}

	sys_write8(gpioa_val, 0xf01601);
}
#endif

static inline void enable_standby_state(bool enable)
{
	if (enable) {
		pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	} else {
		pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	}
}

static void it51xxx_ps2_enable_wu_irq(const struct device *dev, bool enable)
{
	const struct it51xxx_ps2_config *cfg = dev->config;

	/* clear pending interrupt */
	it51xxx_wuc_clear_status(cfg->wuc.dev, cfg->wuc.mask);
	ite_intc_isr_clear(cfg->wu_irq);

	if (enable) {
		irq_enable(cfg->wu_irq);
	} else {
		irq_disable(cfg->wu_irq);
	}
}

static void it51xxx_ps2_wu_isr(const void *arg)
{
	const struct device *dev = arg;

	it51xxx_ps2_enable_wu_irq(dev, false);
	enable_standby_state(false);

	LOG_DBG("ps2 clk wu triggered");

#if DEBUG_GPIOA3
	it51xxx_toggle_gpioa3();
#endif
}

static void it51xxx_ps2_wuc_init(const struct device *dev)
{
	const struct it51xxx_ps2_config *cfg = dev->config;

	/* initialize the wui */
	it51xxx_wuc_set_polarity(cfg->wuc.dev, cfg->wuc.mask, WUC_TYPE_EDGE_FALLING);
	it51xxx_wuc_clear_status(cfg->wuc.dev, cfg->wuc.mask);

	/* enable the wui */
	it51xxx_wuc_enable(cfg->wuc.dev, cfg->wuc.mask);

	/* connect wu (ps2 clk) interrupt but make it disabled initially */
	irq_connect_dynamic(cfg->wu_irq, 0, it51xxx_ps2_wu_isr, dev, 0);
}

static int it51xxx_ps2_configure(const struct device *dev, ps2_callback_t callback_isr)
{
	const struct it51xxx_ps2_config *cfg = dev->config;
	struct it51xxx_ps2_data *data = dev->data;

	if (callback_isr == NULL) {
		return -EINVAL;
	}

	data->callback_isr = callback_isr;

	/* configure ps2 bus as idle state */
	sys_write8(DEBOUNCE_ENABLE | CTRL_CLK_LINE | CTRL_DATA_LINE, cfg->base + PS200_CTRL_REG);

	k_sem_give(&data->lock);

	return 0;
}

static int it51xxx_ps2_bus_busy(const struct device *dev)
{
	const struct it51xxx_ps2_config *cfg = dev->config;
	uint8_t status = sys_read8(cfg->base + PS208_STATUS);

	return ((status & BUS_IDLE) != BUS_IDLE || (status & START_STATUS)) ? -EBUSY : 0;
}

static inline void it51xxx_ps2_int_enable(const struct device *dev)
{
	const struct it51xxx_ps2_config *cfg = dev->config;

	sys_write8(TIMEOUT_INT_ENABLE | TRANSACTION_DONE_INT_ENABLE, cfg->base + PS204_INT_CTRL_REG);
}

static int it51xxx_ps2_write(const struct device *dev, uint8_t value)
{
	const struct it51xxx_ps2_config *cfg = dev->config;
	struct it51xxx_ps2_data *data = dev->data;
	int ret, i = 0;
	uint8_t ctrl_val;

	if (k_sem_take(&data->lock, K_NO_WAIT)) {
		return -EACCES;
	}

	/* allow the ps2 controller to complete a rx transaction */
	while (it51xxx_ps2_bus_busy(dev) && (i < PS2_RETRY_COUNT)) {
		k_busy_wait(50);
		i++;
	}

	if (unlikely(i == PS2_RETRY_COUNT)) {
		LOG_ERR("failed to send ps2 data");
		ret = -EBUSY;
		goto out;
	}

	it51xxx_ps2_enable_wu_irq(dev, false);
	enable_standby_state(false);

	/* enable hardware mode, pull down clk line and pull up data line */
	ctrl_val = DEBOUNCE_ENABLE | TX_MODE_SELECTION | HARDWARE_MODE_ENABLE | CTRL_DATA_LINE;
	sys_write8(ctrl_val, cfg->base + PS200_CTRL_REG);

	/* inhibit communication for at least 100 microseconds */
	k_busy_wait(100);

	/* enable interrupt */
	it51xxx_ps2_int_enable(dev);

	/* set data */
	sys_write8(value, cfg->base + PS20C_DATA_REG);

	/* pull down data line */
	ctrl_val &= ~CTRL_DATA_LINE;
	sys_write8(ctrl_val, cfg->base + PS200_CTRL_REG);

	/* pull up clk line */
	ctrl_val |= CTRL_CLK_LINE;
	sys_write8(ctrl_val, cfg->base + PS200_CTRL_REG);

	k_sem_take(&data->tx_sem, K_FOREVER);
	it51xxx_ps2_enable_wu_irq(dev, true);
	enable_standby_state(true);

	irq_disable(cfg->irq_num);
	ret = data->xfer_status;
	irq_enable(cfg->irq_num);

out:
	k_sem_give(&data->lock);

	return ret;
}

static int it51xxx_ps2_inhibit_interface(const struct device *dev)
{
	const struct it51xxx_ps2_config *cfg = dev->config;

	irq_disable(cfg->irq_num);
	it51xxx_ps2_enable_wu_irq(dev, false);

	if (it51xxx_ps2_bus_busy(dev)) {
		enable_standby_state(true);
	}

	/* communication inhibited */
	sys_write8(DEBOUNCE_ENABLE | CTRL_DATA_LINE, cfg->base + PS200_CTRL_REG);

	it51xxx_ps2_enable_wu_irq(dev, true);
	irq_enable(cfg->irq_num);

	LOG_DBG("inhibit interface");

	return 0;
}

static int it51xxx_ps2_enable_interface(const struct device *dev)
{
	const struct it51xxx_ps2_config *cfg = dev->config;

	it51xxx_ps2_enable_wu_irq(dev, false);

	/* set bus as idle state */
	sys_write8(DEBOUNCE_ENABLE | HARDWARE_MODE_ENABLE | CTRL_CLK_LINE | CTRL_DATA_LINE,
		   cfg->base + PS200_CTRL_REG);

	it51xxx_ps2_enable_wu_irq(dev, true);

	LOG_DBG("enable interface");

	return 0;
}

static DEVICE_API(ps2, it51xxx_ps2_api) = {
	.config = it51xxx_ps2_configure,
	.read = NULL,
	.write = it51xxx_ps2_write,
	.disable_callback = it51xxx_ps2_inhibit_interface,
	.enable_callback = it51xxx_ps2_enable_interface,
};

static int it51xxx_ps2_init(const struct device *dev)
{
	const struct it51xxx_ps2_config *cfg = dev->config;
	struct it51xxx_ps2_data *data = dev->data;
	int ret;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("failed to apply pinctrl, ret %d", ret);
		return ret;
	}

	k_sem_init(&data->lock, 0, 1);
	k_sem_init(&data->tx_sem, 0, 1);

	it51xxx_ps2_wuc_init(dev);

	cfg->irq_config_func(dev);

	return 0;
}

static void it51xxx_ps2_isr(const struct device *dev)
{
	const struct it51xxx_ps2_config *cfg = dev->config;
	struct it51xxx_ps2_data *data = dev->data;
	uint8_t int_status;
	bool xfer_is_tx =
		(sys_read8(cfg->base + PS200_CTRL_REG) & TX_MODE_SELECTION) == TX_MODE_SELECTION;

	int_status = sys_read8(cfg->base + PS208_STATUS);
	LOG_DBG("isr: interrupt status 0x%x", int_status);

	if (int_status & XFER_ERROR_MASK) {
#if DEBUG_GPIOA0
		it51xxx_toggle_gpioa0();
#endif
		/* inhibit communication */
		sys_write8(DEBOUNCE_ENABLE | CTRL_DATA_LINE, cfg->base + PS200_CTRL_REG);

		if (IS_BIT_SET(int_status, BIT_TIMEOUT_ERR)) {
			LOG_ERR("isr: %s: timeout event occurs", xfer_is_tx ? "tx" : "rx");

			data->xfer_status = -ETIMEDOUT;
			sys_write8(BIT(BIT_TIMEOUT_ERR), cfg->base + PS208_STATUS);
		}

		if (IS_BIT_SET(int_status, BIT_FRAME_ERR)) {
			LOG_ERR("isr: %s: frame error occurs", xfer_is_tx ? "tx" : "rx");

			data->xfer_status = -EPROTO;
		}

		if (IS_BIT_SET(int_status, BIT_PARITY_ERR)) {
			LOG_ERR("isr: %s: parity error occurs", xfer_is_tx ? "tx" : "rx");

			data->xfer_status = -EIO;
		}

		if (xfer_is_tx) {
			k_sem_give(&data->tx_sem);
		}

		/* start to receive ps2 data */
		sys_write8(DEBOUNCE_ENABLE | HARDWARE_MODE_ENABLE | CTRL_CLK_LINE | CTRL_DATA_LINE,
			   cfg->base + PS200_CTRL_REG);
	}

	if (int_status & TRANSACTIOIN_DONE) {
		LOG_DBG("isr: %s: xfer done", xfer_is_tx ? "tx" : "rx");

		data->xfer_status = 0;
		if (xfer_is_tx) {
#if DEBUG_GPIOA2
			it51xxx_toggle_gpioa2();
#endif
			uint8_t ctrl_val;

			ctrl_val = DEBOUNCE_ENABLE | HARDWARE_MODE_ENABLE | CTRL_CLK_LINE |
				   CTRL_DATA_LINE;
			sys_write8(ctrl_val, cfg->base + PS200_CTRL_REG);

			/* enable transaction interrupt */
			it51xxx_ps2_int_enable(dev);

			k_sem_give(&data->tx_sem);
		} else {
#if DEBUG_GPIOA1
			it51xxx_toggle_gpioa1();
#endif
			it51xxx_ps2_enable_wu_irq(dev, true);
			enable_standby_state(true);
			if (data->callback_isr) {
				data->callback_isr(dev, sys_read8(cfg->base + PS20C_DATA_REG));
			}
		}
	}
}

#define IT51XXX_PS2_INIT(n)                                                                        \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void it51xxx_ps2_config_func_##n(const struct device *dev)                          \
	{                                                                                          \
		ite_intc_irq_polarity_set(DT_INST_IRQN(n), DT_INST_IRQ(n, flags));                 \
		IRQ_CONNECT(DT_INST_IRQN(n), 0, it51xxx_ps2_isr, DEVICE_DT_INST_GET(n), 0);        \
		irq_enable(DT_INST_IRQN(n));                                                       \
	};                                                                                         \
	static struct it51xxx_ps2_config ps2_config_##n = {                                        \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.wuc = {.dev = IT8XXX2_DEV_WUC(0, n), .mask = IT8XXX2_DEV_WUC_MASK(0, n)},         \
		.wu_irq = DT_INST_IRQ_BY_IDX(n, 1, irq),                                           \
		.irq_num = DT_INST_IRQ(n, irq),                                                    \
		.irq_config_func = it51xxx_ps2_config_func_##n,                                    \
	};                                                                                         \
	static struct it51xxx_ps2_data ps2_data_##n;                                               \
	DEVICE_DT_INST_DEFINE(n, it51xxx_ps2_init, NULL, &ps2_data_##n, &ps2_config_##n,           \
			      POST_KERNEL, CONFIG_PS2_INIT_PRIORITY, &it51xxx_ps2_api);

DT_INST_FOREACH_STATUS_OKAY(IT51XXX_PS2_INIT)
