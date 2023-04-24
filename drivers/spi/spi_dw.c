/*
 * Copyright (c) 2015 Intel Corporation.
 * Copyright (c) 2023 Synopsys, Inc. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_designware_spi

/* spi_dw.c - Designware SPI driver implementation */

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_dw);

#if (CONFIG_SPI_LOG_LEVEL == 4)
#define DBG_COUNTER_INIT()	\
	uint32_t __cnt = 0
#define DBG_COUNTER_INC()	\
	(__cnt++)
#define DBG_COUNTER_RESULT()	\
	(__cnt)
#else
#define DBG_COUNTER_INIT() {; }
#define DBG_COUNTER_INC() {; }
#define DBG_COUNTER_RESULT() 0
#endif

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/device.h>

#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#ifdef CONFIG_IOAPIC
#include <zephyr/drivers/interrupt_controller/ioapic.h>
#endif

#include <zephyr/drivers/spi.h>
#include <zephyr/irq.h>

#include "spi_dw.h"
#include "spi_context.h"

#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

static inline bool spi_dw_is_slave(struct spi_dw_data *spi)
{
	return (IS_ENABLED(CONFIG_SPI_SLAVE) &&
		spi_context_is_slave(&spi->ctx));
}

static void completed(const struct device *dev, int error)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;

	if (error) {
		goto out;
	}

	if (spi_context_tx_on(&spi->ctx) ||
	    spi_context_rx_on(&spi->ctx)) {
		return;
	}

out:
	/* need to give time for FIFOs to drain before issuing more commands */
	while (test_bit_sr_busy(info)) {
	}

	/* Disabling interrupts */
	write_imr(info, DW_SPI_IMR_MASK);
	/* Disabling the controller */
	clear_bit_ssienr(info);

	spi_context_cs_control(&spi->ctx, false);

	LOG_DBG("SPI transaction completed %s error",
		    error ? "with" : "without");

	spi_context_complete(&spi->ctx, dev, error);
}

static void push_data(const struct device *dev)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
	uint32_t data = 0U;
	uint32_t f_tx;

	DBG_COUNTER_INIT();

	if (spi_context_rx_on(&spi->ctx)) {
		f_tx = info->fifo_depth - read_txflr(info) -
			read_rxflr(info);
		if ((int)f_tx < 0) {
			f_tx = 0U; /* if rx-fifo is full, hold off tx */
		}
	} else {
		f_tx = info->fifo_depth - read_txflr(info);
	}

	while (f_tx) {
		if (spi_context_tx_buf_on(&spi->ctx)) {
			switch (spi->dfs) {
			case 1:
				data = UNALIGNED_GET((uint8_t *)
						     (spi->ctx.tx_buf));
				break;
			case 2:
				data = UNALIGNED_GET((uint16_t *)
						     (spi->ctx.tx_buf));
				break;
#ifndef CONFIG_ARC
			case 4:
				data = UNALIGNED_GET((uint32_t *)
						     (spi->ctx.tx_buf));
				break;
#endif
			}
		} else if (spi_context_rx_on(&spi->ctx)) {
			/* No need to push more than necessary */
			if ((int)(spi->ctx.rx_len - spi->fifo_diff) <= 0) {
				break;
			}

			data = 0U;
		} else if (spi_context_tx_on(&spi->ctx)) {
			data = 0U;
		} else {
			/* Nothing to push anymore */
			break;
		}

		write_dr(info, data);

		spi_context_update_tx(&spi->ctx, spi->dfs, 1);
		spi->fifo_diff++;

		f_tx--;

		DBG_COUNTER_INC();
	}

	if (!spi_context_tx_on(&spi->ctx)) {
		/* prevents any further interrupts demanding TX fifo fill */
		write_txftlr(info, 0);
	}

	LOG_DBG("Pushed: %d", DBG_COUNTER_RESULT());
}

static void pull_data(const struct device *dev)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;

	DBG_COUNTER_INIT();

	while (read_rxflr(info)) {
		uint32_t data = read_dr(info);

		DBG_COUNTER_INC();

		if (spi_context_rx_buf_on(&spi->ctx)) {
			switch (spi->dfs) {
			case 1:
				UNALIGNED_PUT(data, (uint8_t *)spi->ctx.rx_buf);
				break;
			case 2:
				UNALIGNED_PUT(data, (uint16_t *)spi->ctx.rx_buf);
				break;
#ifndef CONFIG_ARC
			case 4:
				UNALIGNED_PUT(data, (uint32_t *)spi->ctx.rx_buf);
				break;
#endif
			}
		}

		spi_context_update_rx(&spi->ctx, spi->dfs, 1);
		spi->fifo_diff--;
	}

	if (!spi->ctx.rx_len && spi->ctx.tx_len < info->fifo_depth) {
		write_rxftlr(info, spi->ctx.tx_len - 1);
	} else if (read_rxftlr(info) >= spi->ctx.rx_len) {
		write_rxftlr(info, spi->ctx.rx_len - 1);
	}

	LOG_DBG("Pulled: %d", DBG_COUNTER_RESULT());
}

static int spi_dw_configure(const struct spi_dw_config *info,
			    struct spi_dw_data *spi,
			    const struct spi_config *config)
{
	uint32_t ctrlr0 = 0U;

	LOG_DBG("%p (prev %p)", config, spi->ctx.config);

	if (spi_context_configured(&spi->ctx, config)) {
		/* Nothing to do */
		return 0;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	/* Verify if requested op mode is relevant to this controller */
	if (config->operation & SPI_OP_MODE_SLAVE) {
		if (!(info->op_modes & SPI_CTX_RUNTIME_OP_MODE_SLAVE)) {
			LOG_ERR("Slave mode not supported");
			return -ENOTSUP;
		}
	} else {
		if (!(info->op_modes & SPI_CTX_RUNTIME_OP_MODE_MASTER)) {
			LOG_ERR("Master mode not supported");
			return -ENOTSUP;
		}
	}

	if ((config->operation & SPI_TRANSFER_LSB) ||
	    (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	     (config->operation & (SPI_LINES_DUAL |
				   SPI_LINES_QUAD | SPI_LINES_OCTAL)))) {
		LOG_ERR("Unsupported configuration");
		return -EINVAL;
	}

	/* Word size */
	ctrlr0 |= DW_SPI_CTRLR0_DFS(SPI_WORD_SIZE_GET(config->operation));

	/* Determine how many bytes are required per-frame */
	spi->dfs = SPI_WS_TO_DFS(SPI_WORD_SIZE_GET(config->operation));

	/* SPI mode */
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		ctrlr0 |= DW_SPI_CTRLR0_SCPOL;
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		ctrlr0 |= DW_SPI_CTRLR0_SCPH;
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_LOOP) {
		ctrlr0 |= DW_SPI_CTRLR0_SRL;
	}

	/* Installing the configuration */
	write_ctrlr0(info, ctrlr0);

	/* At this point, it's mandatory to set this on the context! */
	spi->ctx.config = config;

	if (!spi_dw_is_slave(spi)) {
		/* Baud rate and Slave select, for master only */
		write_baudr(info, SPI_DW_CLK_DIVIDER(info->clock_frequency,
					       config->frequency));
		write_ser(info, 1 << config->slave);
	}

	if (spi_dw_is_slave(spi)) {
		LOG_DBG("Installed slave config %p:"
			    " ws/dfs %u/%u, mode %u/%u/%u",
			    config,
			    SPI_WORD_SIZE_GET(config->operation), spi->dfs,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPOL) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPHA) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_LOOP) ? 1 : 0);
	} else {
		LOG_DBG("Installed master config %p: freq %uHz (div = %u),"
			    " ws/dfs %u/%u, mode %u/%u/%u, slave %u",
			    config, config->frequency,
			    SPI_DW_CLK_DIVIDER(info->clock_frequency,
					       config->frequency),
			    SPI_WORD_SIZE_GET(config->operation), spi->dfs,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPOL) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPHA) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_LOOP) ? 1 : 0,
			    config->slave);
	}

	return 0;
}

static uint32_t spi_dw_compute_ndf(const struct spi_buf *rx_bufs,
				   size_t rx_count, uint8_t dfs)
{
	uint32_t len = 0U;

	for (; rx_count; rx_bufs++, rx_count--) {
		if (len > (UINT16_MAX - rx_bufs->len)) {
			goto error;
		}

		len += rx_bufs->len;
	}

	if (len) {
		return (len / dfs) - 1;
	}
error:
	return UINT32_MAX;
}

static void spi_dw_update_txftlr(const struct spi_dw_config *info,
				 struct spi_dw_data *spi)
{
	uint32_t dw_spi_txftlr_dflt = (info->fifo_depth * 1) / 2;
	uint32_t reg_data = dw_spi_txftlr_dflt;

	if (spi_dw_is_slave(spi)) {
		if (!spi->ctx.tx_len) {
			reg_data = 0U;
		} else if (spi->ctx.tx_len < dw_spi_txftlr_dflt) {
			reg_data = spi->ctx.tx_len - 1;
		}
	}

	LOG_DBG("TxFTLR: %u", reg_data);

	write_txftlr(info, reg_data);
}

static int transceive(const struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
		      spi_callback_t cb,
		      void *userdata)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
	uint32_t tmod = DW_SPI_CTRLR0_TMOD_TX_RX;
	uint32_t dw_spi_rxftlr_dflt = (info->fifo_depth * 5) / 8;
	uint32_t reg_data;
	int ret;

	spi_context_lock(&spi->ctx, asynchronous, cb, userdata, config);

#ifdef CONFIG_PM_DEVICE
	if (!pm_device_is_busy(dev)) {
		pm_device_busy_set(dev);
	}
#endif /* CONFIG_PM_DEVICE */

	/* Configure */
	ret = spi_dw_configure(info, spi, config);
	if (ret) {
		goto out;
	}

	if (!rx_bufs || !rx_bufs->buffers) {
		tmod = DW_SPI_CTRLR0_TMOD_TX;
	} else if (!tx_bufs || !tx_bufs->buffers) {
		tmod = DW_SPI_CTRLR0_TMOD_RX;
	}

	/* ToDo: add a way to determine EEPROM mode */

	if (tmod >= DW_SPI_CTRLR0_TMOD_RX &&
	    !spi_dw_is_slave(spi)) {
		reg_data = spi_dw_compute_ndf(rx_bufs->buffers,
					      rx_bufs->count,
					      spi->dfs);
		if (reg_data == UINT32_MAX) {
			ret = -EINVAL;
			goto out;
		}

		write_ctrlr1(info, reg_data);
	} else {
		write_ctrlr1(info, 0);
	}

	if (spi_dw_is_slave(spi)) {
		/* Enabling MISO line relevantly */
		if (tmod == DW_SPI_CTRLR0_TMOD_RX) {
			tmod |= DW_SPI_CTRLR0_SLV_OE;
		} else {
			tmod &= ~DW_SPI_CTRLR0_SLV_OE;
		}
	}

	/* Updating TMOD in CTRLR0 register */
	reg_data = read_ctrlr0(info);
	reg_data &= ~DW_SPI_CTRLR0_TMOD_RESET;
	reg_data |= tmod;

	write_ctrlr0(info, reg_data);

	/* Set buffers info */
	spi_context_buffers_setup(&spi->ctx, tx_bufs, rx_bufs, spi->dfs);

	spi->fifo_diff = 0U;

	/* Tx Threshold */
	spi_dw_update_txftlr(info, spi);

	/* Does Rx thresholds needs to be lower? */
	reg_data = dw_spi_rxftlr_dflt;

	if (spi_dw_is_slave(spi)) {
		if (spi->ctx.rx_len &&
		    spi->ctx.rx_len < dw_spi_rxftlr_dflt) {
			reg_data = spi->ctx.rx_len - 1;
		}
	} else {
		if (spi->ctx.rx_len && spi->ctx.rx_len < info->fifo_depth) {
			reg_data = spi->ctx.rx_len - 1;
		}
	}

	/* Rx Threshold */
	write_rxftlr(info, reg_data);

	/* Enable interrupts */
	reg_data = !rx_bufs ?
		DW_SPI_IMR_UNMASK & DW_SPI_IMR_MASK_RX :
		DW_SPI_IMR_UNMASK & DW_SPI_IMR_MASK_TX;
	write_imr(info, reg_data);

	spi_context_cs_control(&spi->ctx, true);

	LOG_DBG("Enabling controller");
	set_bit_ssienr(info);

	ret = spi_context_wait_for_completion(&spi->ctx);
out:
	spi_context_release(&spi->ctx, ret);

	pm_device_busy_clear(dev);

	return ret;
}

static int spi_dw_transceive(const struct device *dev,
			     const struct spi_config *config,
			     const struct spi_buf_set *tx_bufs,
			     const struct spi_buf_set *rx_bufs)
{
	LOG_DBG("%p, %p, %p", dev, tx_bufs, rx_bufs);

	return transceive(dev, config, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_dw_transceive_async(const struct device *dev,
				   const struct spi_config *config,
				   const struct spi_buf_set *tx_bufs,
				   const struct spi_buf_set *rx_bufs,
				   spi_callback_t cb,
				   void *userdata)
{
	LOG_DBG("%p, %p, %p, %p, %p", dev, tx_bufs, rx_bufs, cb, userdata);

	return transceive(dev, config, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_dw_release(const struct device *dev,
			  const struct spi_config *config)
{
	struct spi_dw_data *spi = dev->data;

	if (!spi_context_configured(&spi->ctx, config)) {
		return -EINVAL;
	}

	spi_context_unlock_unconditionally(&spi->ctx);

	return 0;
}

void spi_dw_isr(const struct device *dev)
{
	const struct spi_dw_config *info = dev->config;
	uint32_t int_status;
	int error;

	int_status = read_isr(info);

	LOG_DBG("SPI %p int_status 0x%x - (tx: %d, rx: %d)", dev, int_status,
		read_txflr(info), read_rxflr(info));

	if (int_status & DW_SPI_ISR_ERRORS_MASK) {
		error = -EIO;
		goto out;
	}

	error = 0;

	if (int_status & DW_SPI_ISR_RXFIS) {
		pull_data(dev);
	}

	if (int_status & DW_SPI_ISR_TXEIS) {
		push_data(dev);
	}

out:
	clear_interrupts(info);
	completed(dev, error);
}

static const struct spi_driver_api dw_spi_api = {
	.transceive = spi_dw_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_dw_transceive_async,
#endif /* CONFIG_SPI_ASYNC */
	.release = spi_dw_release,
};

int spi_dw_init(const struct device *dev)
{
	int err;
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;

#ifdef CONFIG_PINCTRL
	pinctrl_apply_state(info->pcfg, PINCTRL_STATE_DEFAULT);
#endif

	info->config_func();

	/* Masking interrupt and making sure controller is disabled */
	write_imr(info, DW_SPI_IMR_MASK);
	clear_bit_ssienr(info);

	LOG_DBG("Designware SPI driver initialized on device: %p", dev);

	err = spi_context_cs_configure_all(&spi->ctx);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&spi->ctx);

	return 0;
}


#if DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay)
void spi_config_0_irq(void);

struct spi_dw_data spi_dw_data_port_0 = {
	SPI_CONTEXT_INIT_LOCK(spi_dw_data_port_0, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_dw_data_port_0, ctx),
	SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(0), ctx)
};

#if DT_NODE_HAS_PROP(DT_INST_PHANDLE(0, clocks), clock_frequency)
#define INST_0_SNPS_DESIGNWARE_SPI_CLOCK_FREQ \
	DT_INST_PROP_BY_PHANDLE(0, clocks, clock_frequency)
#else
#define INST_0_SNPS_DESIGNWARE_SPI_CLOCK_FREQ \
	DT_INST_PROP(0, clock_frequency)
#endif

#ifdef CONFIG_PINCTRL
PINCTRL_DT_INST_DEFINE(0);
#endif
const struct spi_dw_config spi_dw_config_0 = {
	.regs = DT_INST_REG_ADDR(0),
	.clock_frequency = INST_0_SNPS_DESIGNWARE_SPI_CLOCK_FREQ,
	.config_func = spi_config_0_irq,
	.op_modes = SPI_CTX_RUNTIME_OP_MODE_MASTER,
	.fifo_depth = DT_INST_PROP(0, fifo_depth),
#ifdef CONFIG_PINCTRL
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
#endif
#if DT_INST_PROP(0, aux_reg)
	.read_func = aux_reg_read,
	.write_func = aux_reg_write,
	.set_bit_func = aux_reg_set_bit,
	.clear_bit_func = aux_reg_clear_bit,
	.test_bit_func = aux_reg_test_bit
#else
	.read_func = reg_read,
	.write_func = reg_write,
	.set_bit_func = reg_set_bit,
	.clear_bit_func = reg_clear_bit,
	.test_bit_func = reg_test_bit
#endif
};

DEVICE_DT_INST_DEFINE(0, spi_dw_init, NULL,
		    &spi_dw_data_port_0, &spi_dw_config_0,
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
		    &dw_spi_api);

void spi_config_0_irq(void)
{
#if DT_NUM_IRQS(DT_DRV_INST(0)) == 1
#if DT_INST_IRQ_HAS_NAME(0, flags)
#define INST_0_IRQ_FLAGS DT_INST_IRQ_BY_NAME(0, flags, irq)
#else
#define INST_0_IRQ_FLAGS 0
#endif
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(0),
		    INST_0_IRQ_FLAGS);
	irq_enable(DT_INST_IRQN(0));
#else
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, rx_avail, irq),
		    DT_INST_IRQ_BY_NAME(0, rx_avail, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(0),
		    0);
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, tx_req, irq),
		    DT_INST_IRQ_BY_NAME(0, tx_req, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(0),
		    0);
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, err_int, irq),
		    DT_INST_IRQ_BY_NAME(0, err_int, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(0),
		    0);

	irq_enable(DT_INST_IRQ_BY_NAME(0, rx_avail, irq));
	irq_enable(DT_INST_IRQ_BY_NAME(0, tx_req, irq));
	irq_enable(DT_INST_IRQ_BY_NAME(0, err_int, irq));

#endif
}
#endif /* DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay) */

#if DT_NODE_HAS_STATUS(DT_DRV_INST(1), okay)
void spi_config_1_irq(void);

struct spi_dw_data spi_dw_data_port_1 = {
	SPI_CONTEXT_INIT_LOCK(spi_dw_data_port_1, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_dw_data_port_1, ctx),
	SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(1), ctx)
};

#if DT_NODE_HAS_PROP(DT_INST_PHANDLE(1, clocks), clock_frequency)
#define INST_1_SNPS_DESIGNWARE_SPI_CLOCK_FREQ \
	DT_INST_PROP_BY_PHANDLE(1, clocks, clock_frequency)
#else
#define INST_1_SNPS_DESIGNWARE_SPI_CLOCK_FREQ \
	DT_INST_PROP(1, clock_frequency)
#endif

#ifdef CONFIG_PINCTRL
PINCTRL_DT_INST_DEFINE(1);
#endif
static const struct spi_dw_config spi_dw_config_1 = {
	.regs = DT_INST_REG_ADDR(1),
	.clock_frequency = INST_1_SNPS_DESIGNWARE_SPI_CLOCK_FREQ,
	.config_func = spi_config_1_irq,
	.op_modes = SPI_CTX_RUNTIME_OP_MODE_MASTER,
	.fifo_depth = DT_INST_PROP(1, fifo_depth),
#ifdef CONFIG_PINCTRL
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(1),
#endif
#if DT_INST_PROP(1, aux_reg)
	.read_func = aux_reg_read,
	.write_func = aux_reg_write,
	.set_bit_func = aux_reg_set_bit,
	.clear_bit_func = aux_reg_clear_bit,
	.test_bit_func = aux_reg_test_bit
#else
	.read_func = reg_read,
	.write_func = reg_write,
	.set_bit_func = reg_set_bit,
	.clear_bit_func = reg_clear_bit,
	.test_bit_func = reg_test_bit
#endif
};

DEVICE_DT_INST_DEFINE(1, spi_dw_init, NULL,
		    &spi_dw_data_port_1, &spi_dw_config_1,
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
		    &dw_spi_api);

void spi_config_1_irq(void)
{
#if DT_NUM_IRQS(DT_DRV_INST(1)) == 1
#if DT_INST_IRQ_HAS_NAME(1, flags)
#define INST_1_IRQ_FLAGS DT_INST_IRQ_BY_NAME(1, flags, irq)
#else
#define INST_1_IRQ_FLAGS 0
#endif
	IRQ_CONNECT(DT_INST_IRQN(1),
		    DT_INST_IRQ(1, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(1),
		    INST_1_IRQ_FLAGS);
	irq_enable(DT_INST_IRQN(1));
#else
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(1, rx_avail, irq),
		    DT_INST_IRQ_BY_NAME(1, rx_avail, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(1),
		    0);
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(1, tx_req, irq),
		    DT_INST_IRQ_BY_NAME(1, tx_req, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(1),
		    0);
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(1, err_int, irq),
		    DT_INST_IRQ_BY_NAME(1, err_int, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(1),
		    0);

	irq_enable(DT_INST_IRQ_BY_NAME(1, rx_avail, irq));
	irq_enable(DT_INST_IRQ_BY_NAME(1, tx_req, irq));
	irq_enable(DT_INST_IRQ_BY_NAME(1, err_int, irq));

#endif
}
#endif /* DT_NODE_HAS_STATUS(DT_DRV_INST(1), okay) */

#if DT_NODE_HAS_STATUS(DT_DRV_INST(2), okay)
void spi_config_2_irq(void);

struct spi_dw_data spi_dw_data_port_2 = {
	SPI_CONTEXT_INIT_LOCK(spi_dw_data_port_2, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_dw_data_port_2, ctx),
	SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(2), ctx)
};

#if DT_NODE_HAS_PROP(DT_INST_PHANDLE(2, clocks), clock_frequency)
#define INST_2_SNPS_DESIGNWARE_SPI_CLOCK_FREQ \
	DT_INST_PROP_BY_PHANDLE(2, clocks, clock_frequency)
#else
#define INST_2_SNPS_DESIGNWARE_SPI_CLOCK_FREQ \
	DT_INST_PROP(2, clock_frequency)
#endif

#ifdef CONFIG_PINCTRL
PINCTRL_DT_INST_DEFINE(2);
#endif
static const struct spi_dw_config spi_dw_config_2 = {
	.regs = DT_INST_REG_ADDR(2),
	.clock_frequency = INST_2_SNPS_DESIGNWARE_SPI_CLOCK_FREQ,
	.config_func = spi_config_2_irq,
	.op_modes = SPI_CTX_RUNTIME_OP_MODE_MASTER,
	.fifo_depth = DT_INST_PROP(2, fifo_depth),
#ifdef CONFIG_PINCTRL
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(2),
#endif
#if DT_INST_PROP(2, aux_reg)
	.read_func = aux_reg_read,
	.write_func = aux_reg_write,
	.set_bit_func = aux_reg_set_bit,
	.clear_bit_func = aux_reg_clear_bit,
	.test_bit_func = aux_reg_test_bit
#else
	.read_func = reg_read,
	.write_func = reg_write,
	.set_bit_func = reg_set_bit,
	.clear_bit_func = reg_clear_bit,
	.test_bit_func = reg_test_bit
#endif
};

DEVICE_DT_INST_DEFINE(2, spi_dw_init, NULL,
		    &spi_dw_data_port_2, &spi_dw_config_2,
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
		    &dw_spi_api);

void spi_config_2_irq(void)
{
#if DT_NUM_IRQS(DT_DRV_INST(2)) == 1
#if DT_INST_IRQ_HAS_NAME(2, flags)
#define INST_2_IRQ_FLAGS DT_INST_IRQ_BY_NAME(2, flags, irq)
#else
#define INST_2_IRQ_FLAGS 0
#endif
	IRQ_CONNECT(DT_INST_IRQN(2),
		    DT_INST_IRQ(2, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(2),
		    INST_2_IRQ_FLAGS);
	irq_enable(DT_INST_IRQN(2));
#else
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(2, rx_avail, irq),
		    DT_INST_IRQ_BY_NAME(2, rx_avail, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(2),
		    0);
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(2, tx_req, irq),
		    DT_INST_IRQ_BY_NAME(2, tx_req, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(2),
		    0);
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(2, err_int, irq),
		    DT_INST_IRQ_BY_NAME(2, err_int, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(2),
		    0);

	irq_enable(DT_INST_IRQ_BY_NAME(2, rx_avail, irq));
	irq_enable(DT_INST_IRQ_BY_NAME(2, tx_req, irq));
	irq_enable(DT_INST_IRQ_BY_NAME(2, err_int, irq));

#endif
}
#endif /* DT_NODE_HAS_STATUS(DT_DRV_INST(2), okay) */

#if DT_NODE_HAS_STATUS(DT_DRV_INST(3), okay)
void spi_config_3_irq(void);

struct spi_dw_data spi_dw_data_port_3 = {
	SPI_CONTEXT_INIT_LOCK(spi_dw_data_port_3, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_dw_data_port_3, ctx),
	SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(3), ctx)
};

#if DT_NODE_HAS_PROP(DT_INST_PHANDLE(3, clocks), clock_frequency)
#define INST_3_SNPS_DESIGNWARE_SPI_CLOCK_FREQ \
	DT_INST_PROP_BY_PHANDLE(3, clocks, clock_frequency)
#else
#define INST_3_SNPS_DESIGNWARE_SPI_CLOCK_FREQ \
	DT_INST_PROP(3, clock_frequency)
#endif

#ifdef CONFIG_PINCTRL
PINCTRL_DT_INST_DEFINE(3);
#endif
static const struct spi_dw_config spi_dw_config_3 = {
	.regs = DT_INST_REG_ADDR(3),
	.clock_frequency = INST_3_SNPS_DESIGNWARE_SPI_CLOCK_FREQ,
	.config_func = spi_config_3_irq,
	.op_modes = SPI_CTX_RUNTIME_OP_MODE_MASTER,
	.fifo_depth = DT_INST_PROP(3, fifo_depth),
#ifdef CONFIG_PINCTRL
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(3),
#endif
#if DT_INST_PROP(3, aux_reg)
	.read_func = aux_reg_read,
	.write_func = aux_reg_write,
	.set_bit_func = aux_reg_set_bit,
	.clear_bit_func = aux_reg_clear_bit,
	.test_bit_func = aux_reg_test_bit
#else
	.read_func = reg_read,
	.write_func = reg_write,
	.set_bit_func = reg_set_bit,
	.clear_bit_func = reg_clear_bit,
	.test_bit_func = reg_test_bit
#endif
};

DEVICE_DT_INST_DEFINE(3, spi_dw_init, NULL,
		    &spi_dw_data_port_3, &spi_dw_config_3,
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
		    &dw_spi_api);

void spi_config_3_irq(void)
{
#if DT_NUM_IRQS(DT_DRV_INST(3)) == 1
#if DT_INST_IRQ_HAS_NAME(3, flags)
#define INST_3_IRQ_FLAGS DT_INST_IRQ_BY_NAME(3, flags, irq)
#else
#define INST_3_IRQ_FLAGS 0
#endif
	IRQ_CONNECT(DT_INST_IRQN(3),
		    DT_INST_IRQ(3, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(3),
		    INST_3_IRQ_FLAGS);
	irq_enable(DT_INST_IRQN(3));
#else
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(3, rx_avail, irq),
		    DT_INST_IRQ_BY_NAME(3, rx_avail, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(3),
		    0);
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(3, tx_req, irq),
		    DT_INST_IRQ_BY_NAME(3, tx_req, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(3),
		    0);
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(3, err_int, irq),
		    DT_INST_IRQ_BY_NAME(3, err_int, priority),
		    spi_dw_isr, DEVICE_DT_INST_GET(3),
		    0);

	irq_enable(DT_INST_IRQ_BY_NAME(3, rx_avail, irq));
	irq_enable(DT_INST_IRQ_BY_NAME(3, tx_req, irq));
	irq_enable(DT_INST_IRQ_BY_NAME(3, err_int, irq));

#endif
}
#endif /* DT_NODE_HAS_STATUS(DT_DRV_INST(3), okay) */
