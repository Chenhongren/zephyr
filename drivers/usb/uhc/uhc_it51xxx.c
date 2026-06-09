/*
 * Copyright (c) 2026 ITE Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uhc_common.h"

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/usb/uhc.h>

#include <zephyr/logging/log.h>
// LOG_MODULE_REGISTER(uhc_it51xxx, CONFIG_UHC_DRIVER_LOG_LEVEL);
LOG_MODULE_REGISTER(uhc_it51xxx, LOG_LEVEL_DBG);

#define DT_DRV_COMPAT ite_it51xxx_uhc

#define IT51XXX_UHC_EXTEND_CTRL_ENABLED DT_ALL_INST_HAS_PROP_STATUS_OKAY(extend_ctrl)

#define UHC00_TX_CTRL_REGISTER 0x0
#define ISO_EN                 BIT(3)
#define PREAMBLE_EN            BIT(2)
#define SOF_SYNC               BIT(1)
#define START_XFER             BIT(0)

#define UHC01_TX_TRANS_TYPE 0x1
#define TRANSACTION_TYPE(x) FIELD_PREP(GENMASK(1, 0), x)

#define UHC02_TX_LINE_CTRL       0x2
#define FULL_SPEED_LINE_RATE     BIT(4)
#define FULL_SPEED_LINE_POLARITY BIT(3)
#define DIRECT_CTRL              BIT(2)
#define TX_LINE_STATE_DP         BIT(1)
#define TX_LINE_STATE_DM         BIT(0)
#define TX_LINE_STATE_MASK       GENMASK(1, 0)

#define UHC03_TX_SOF_EN 0x3
#define SOF_ENABLE      BIT(0)

#define UHC04_TX_ADDRESS_REG 0x4
#define DEVICE_ADDRESS(x)    FIELD_PREP(GENMASK(6, 0), x)

#define UHC05_TX_ENDPOINT_NUMBER 0x5
#define ENDPOINT_NUM(x)          FIELD_PREP(GENMASK(3, 0), x)

#define UHC06_FRAME_NUMBER_MSB 0x6
#define FRAME_NUM_MSB(x)       FIELD_GET(GENMASK(2, 0), x)

#define UHC07_FRAME_NUMBER_LSB 0x7
#define UHC08_INT_STATUS       0x8
#define UHC09_INT_MASK         0x9
#define PORT2_CONNECTION_EVENT BIT(7)
#define PORT1_CONNECTION_EVENT BIT(6)
#define PORT2_RESUME_EVENT     BIT(5)
#define PORT1_RESUME_EVENT     BIT(4)
#define SOF_SENT               BIT(3)
#define CONNECTION_EVENT       BIT(2)
#define RESUME_EVENT           BIT(1)
#define XFER_DONE              BIT(0)

#define UHC0A_RX_STATUS 0xA
#define DATA_SEQUENCE   BIT(7)
#define ACK_RXED        BIT(6)
#define STALL_RXED      BIT(5)
#define NACK_RXED       BIT(4)
#define RX_TIMEOUT      BIT(3)
#define RX_OVERFLOW     BIT(2)
#define STUFF_ERROR     BIT(1)
#define CRC_ERROR       BIT(0)

#define UHC0B_RX_PID   0xB
#define RECEIVE_PID(x) FIELD_GET(GENMASK(3, 0), x)

#define UHC0C_MISC_CTRL             0xC
#define USB_2PORT_DUPLICATED_SOC    BIT(7)
#define PORT2_FULL_SPEED_EN         BIT(5)
#define PORT1_FULL_SPEED_EN         BIT(4)
#define HOST_PORT_SEL               BIT(3)
#define USB_2PORT_MODE_EN           BIT(2)
#define PORT2_HW_CDP_DET_TO_VSRC_EN BIT(1)
#define PORT1_HW_CDP_DET_TO_VSRC_EN BIT(0)

#define UHC0D_MISC_STATUS        0xD
#define PORT2_LINE_STATE(x)      FIELD_GET(GENMASK(5, 4), x)
#define PORT1_LINE_STATE(x)      FIELD_GET(GENMASK(1, 0), x)

#define UHC0E_RX_CONNECT_STATE 0xE
#define RX_LINE_STATE_MASK     GENMASK(1, 0)
#define LINE_STATE(x)          FIELD_GET(GENMASK(1, 0), x)

#define UHC0F_SOF_TIMER_MSB     0xF
#define UHC20_RX_FIFO_DATA      0x20
#define UHC22_RX_FIFO_COUNT_MSB 0x22
#define UHC23_RX_FIFO_COUNT_LSB 0x23
#define UHC30_TX_FIFO_DATA      0x30

#define UHCE0_HOST_DEVICE_CTRL 0xE0
#define RESET_CORE             BIT(1)
#define HOST_MODE              BIT(0)

#define UHCE1_HOST_DEVICE_VERSION 0xE1
#define VERSION_MAJOR(x)          FIELD_GET(GENMASK(7, 4), x)
#define VERSION_MINOR(x)          FIELD_GET(GENMASK(3, 0), x)

#define UHCE2_PME_WAKEUP 0xE2
#define ASSERT_PME       BIT(0)

#define UHCE4_PORT1_MISC_CTRL 0xE4
#define UHCE8_PORT2_MISC_CTRL 0xE8
#define PULL_DOWN             BIT(4)

#define IT51XXX_STATE_BUS_RESUME 0

enum it51xxx_bus_state {
	J_STATE = 0,
	K_STATE,
	SE0_STATE,
	IDLE_STATE,
};

enum it51xxx_line_state {
	DISCONNECT = 0,
	LOW_SPEED,
	FULL_SPEED,
};

enum it51xxx_transaction_type {
	IT51XXX_XFER_TYPE_SETUP = 0,
	IT51XXX_XFER_TYPE_IN,
	IT51XXX_XFER_TYPE_OUT_DATA0,
	IT51XXX_XFER_TYPE_OUT_DATA1,
};

struct it51xxx_data {
	struct k_thread thread_data;
	struct k_sem xfer_done_sem;

	atomic_t state;
};

struct it51xxx_config {
	mm_reg_t base;

	ite_irq_t irq_no;

#if IT51XXX_UHC_EXTEND_CTRL_ENABLED
	struct {
		mm_reg_t addr;
		uint8_t enable_bit;
		uint8_t disable_bit;
	} extend_ctrl;
#endif /* IT51XXX_UHC_EXTEND_CTRL_ENABLED */

	const struct pinctrl_dev_config *pcfg;

	void (*make_thread)(const struct device *dev);
};

static int it51xxx_drive_bus_state(const struct device *dev, const uint8_t state)
{
	const struct it51xxx_config *config = dev->config;
	uint8_t speed = LINE_STATE(sys_read8(config->base + UHC0E_RX_CONNECT_STATE));
	uint8_t line_ctrl = sys_read8(config->base + UHC02_TX_LINE_CTRL);

	if (speed != LOW_SPEED && speed != FULL_SPEED) {
		LOG_ERR("only supported low-speed and full-speed, speed %d", speed);
		return -ENOTSUP;
	}

	switch (state) {
	case J_STATE:
		if (speed == LOW_SPEED) {
			line_ctrl |= TX_LINE_STATE_DM;
			line_ctrl &= ~TX_LINE_STATE_DP;
		} else {
			line_ctrl |= TX_LINE_STATE_DP;
			line_ctrl &= ~TX_LINE_STATE_DM;
		}
		break;
	case K_STATE:
		if (speed == LOW_SPEED) {
			line_ctrl |= TX_LINE_STATE_DP;
			line_ctrl &= ~TX_LINE_STATE_DM;
		} else {
			line_ctrl |= TX_LINE_STATE_DM;
			line_ctrl &= ~TX_LINE_STATE_DP;
		}
		break;
	case SE0_STATE:
		line_ctrl &= ~(TX_LINE_STATE_DM | TX_LINE_STATE_DP);
		break;
	case IDLE_STATE:
		if (speed == LOW_SPEED) {
			line_ctrl |= TX_LINE_STATE_DM;
			line_ctrl &= ~TX_LINE_STATE_DP;
		} else {
			line_ctrl |= TX_LINE_STATE_DP;
			line_ctrl &= ~TX_LINE_STATE_DM;
		}
		break;
	default:
		LOG_ERR("unknown bus state %d", state);
		return -EINVAL;
	}

	line_ctrl |= DIRECT_CTRL;
	sys_write8(line_ctrl, config->base + UHC02_TX_LINE_CTRL);
	return 0;
}

static int it51xxx_lock(const struct device *dev)
{
	struct uhc_data *data = dev->data;

	return k_mutex_lock(&data->mutex, K_FOREVER);
}

static int it51xxx_unlock(const struct device *dev)
{
	struct uhc_data *data = dev->data;

	return k_mutex_unlock(&data->mutex);
}

static int it51xxx_sof_enable(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	uint8_t reg_val = sys_read8(config->base + UHC03_TX_SOF_EN);

	if (reg_val & SOF_ENABLE) {
		return -EALREADY;
	}

	/* enable frame signaling:
	 * sof packets every 1 ms for full-speed and
	 * keep-alive eop every 1 ms for low-speed
	 */
	sys_write8(reg_val | SOF_ENABLE, config->base + UHC03_TX_SOF_EN);

	LOG_DBG("%s: enable sof generator", dev->name);

	return 0;
}

static int it51xxx_bus_suspend(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	struct it51xxx_data *priv = uhc_get_private(dev);
	int ret;

	/* disable sof generator */
	sys_write8(sys_read8(config->base + UHC03_TX_SOF_EN) & ~SOF_ENABLE,
		   config->base + UHC03_TX_SOF_EN);

	/* set bus as idle state at least 3ms */
	ret = it51xxx_drive_bus_state(dev, IDLE_STATE);
	if (ret) {
		return ret;
	}
	k_msleep(3);

	sys_write8(sys_read8(config->base + UHC02_TX_LINE_CTRL) & ~DIRECT_CTRL,
		   config->base + UHC02_TX_LINE_CTRL);

	uhc_submit_event(dev, UHC_EVT_SUSPENDED, 0);

	LOG_DBG("%s: suspend usb bus", dev->name);

	return 0;
}

static int it51xxx_bus_reset(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	struct it51xxx_data *priv = uhc_get_private(dev);
	int ret;

	/* disable sof generator */
	sys_write8(sys_read8(config->base + UHC03_TX_SOF_EN) & ~SOF_ENABLE,
		   config->base + UHC03_TX_SOF_EN);

	/* set bus as se0 state at least 10ms */
	ret = it51xxx_drive_bus_state(dev, SE0_STATE);
	if (ret) {
		return ret;
	}
	k_msleep(10); /* TODO: CHECKME >50? */
	sys_write8(sys_read8(config->base + UHC02_TX_LINE_CTRL) & ~DIRECT_CTRL,
		   config->base + UHC02_TX_LINE_CTRL);

	uhc_submit_event(dev, UHC_EVT_RESETED, 0);

	LOG_DBG("%s: reset usb bus", dev->name);

	return 0;
}

static int it51xxx_bus_resume(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	struct it51xxx_data *priv = uhc_get_private(dev);
	int ret;

	/* send the resume signaling (k-state) for at least 20ms */
	ret = it51xxx_drive_bus_state(dev, K_STATE);
	if (ret) {
		return ret;
	}
	k_msleep(20);

	/* send low-speed end-of-packet (two bit times of se0 followed by a j-state) */
#if 1 /* TODO: CHECKME: hardware support? */
	ret = it51xxx_drive_bus_state(dev, SE0_STATE);
	if (ret) {
		return ret;
	}
	ret = it51xxx_drive_bus_state(dev, J_STATE);
	if (ret) {
		return ret;
	}

	sys_write8(sys_read8(config->base + UHC02_TX_LINE_CTRL) & ~DIRECT_CTRL,
		   config->base + UHC02_TX_LINE_CTRL);
#endif

	atomic_set_bit(&priv->state, IT51XXX_STATE_BUS_RESUME);

	LOG_DBG("%s: resume usb bus", dev->name);

	return 0;
}

static int it51xxx_enqueue(const struct device *dev, struct uhc_transfer *const xfer)
{
	const struct it51xxx_config *config = dev->config;
	struct it51xxx_data *priv = uhc_get_private(dev);

	LOG_DBG("%s: enqueue addr %#x, ep %#x, stage %d, type %d", dev->name, xfer->udev->addr,
		xfer->ep, xfer->stage, xfer->type);

	(void)uhc_xfer_append(dev, xfer);

	switch (xfer->type) {
	case USB_EP_TYPE_CONTROL:
		LOG_HEXDUMP_DBG(xfer->setup_pkt, sizeof(xfer->setup_pkt), "setup:");
		sys_write8(DEVICE_ADDRESS(xfer->udev->addr), config->base + UHC04_TX_ADDRESS_REG);
		sys_write8(TRANSACTION_TYPE(IT51XXX_XFER_TYPE_SETUP),
			   config->base + UHC01_TX_TRANS_TYPE);
		sys_write8(ENDPOINT_NUM(USB_EP_GET_IDX(xfer->ep)),
			   config->base + UHC05_TX_ENDPOINT_NUMBER);
		for (uint8_t idx = 0; idx < sizeof(xfer->setup_pkt); idx++) {
			sys_write8(xfer->setup_pkt[idx], config->base + UHC30_TX_FIFO_DATA);
		}
		sys_write8(sys_read8(config->base + UHC00_TX_CTRL_REGISTER) | START_XFER,
			   config->base + UHC00_TX_CTRL_REGISTER);

		k_sem_take(&priv->xfer_done_sem, K_FOREVER);
		break;
	case USB_EP_TYPE_ISO:

		break;
	case USB_EP_TYPE_BULK:

		break;
	case USB_EP_TYPE_INTERRUPT:

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int it51xxx_dequeue(const struct device *dev, struct uhc_transfer *const xfer)
{
	struct uhc_data *data = dev->data;
	struct uhc_transfer *tmp;
	unsigned int key;

	key = irq_lock();
	SYS_DLIST_FOR_EACH_CONTAINER(&data->ctrl_xfers, tmp, node) {
		if (xfer == tmp) {
			tmp->err = -ECONNRESET;
		}
	}

	irq_unlock(key);

	return 0;
}

static int uhc_it51xxx_init(const struct device *dev)
{
	struct it51xxx_data *priv = uhc_get_private(dev);

	LOG_DBG("%s: initialize uhc", dev->name);

	return 0;
}

static int uhc_it51xxx_enable(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;

	irq_enable(config->irq_no);

	LOG_DBG("%s: enable uhc", dev->name);

	return 0;
}

static int uhc_it51xxx_disable(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;

	irq_disable(config->irq_no);

	LOG_DBG("%s: disable uhc", dev->name);

	return 0;
}

static int uhc_it51xxx_shutdown(const struct device *dev)
{
	LOG_DBG("%s: power-off uhc", dev->name);

	return 0;
}

static void xfer_work_handler(const struct device *dev)
{
	struct it51xxx_data *priv = uhc_get_private(dev);

	while (true) {
		it51xxx_lock(dev);

		LOG_INF("%s ite debug %d", __func__, __LINE__);

		it51xxx_unlock(dev);

		k_msleep(10000);
	}
}

static void xfer_done_error_check(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	uint8_t rx_status = sys_read8(config->base + UHC0A_RX_STATUS);

	if (rx_status & DATA_SEQUENCE) {
		LOG_DBG("isr: in transaction: data1");
	}

	if (rx_status & ACK_RXED) {
		LOG_DBG("isr: ack is received");
	}

	if (rx_status & STALL_RXED) {
		LOG_WRN("isr: stall is received");
	}

	if (rx_status & NACK_RXED) {
		LOG_ERR("isr: nack is received");
	}

	if (rx_status & RX_TIMEOUT) {
		LOG_ERR("isr: rx timeout");
	}

	if (rx_status & RX_OVERFLOW) {
		LOG_ERR("isr: rx overflow");
	}

	if (rx_status & STUFF_ERROR) {
		LOG_ERR("isr: stuff error");
	}

	if (rx_status & CRC_ERROR) {
		LOG_ERR("isr: crc error");
	}
}

static void it51xxx_uhc_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct it51xxx_config *config = dev->config;
	struct it51xxx_data *priv = uhc_get_private(dev);
	uint8_t int_sts = sys_read8(config->base + UHC08_INT_STATUS);

	LOG_DBG("isr: int status %#x", int_sts);

	if (int_sts & PORT2_CONNECTION_EVENT) {
		uint8_t line_state = PORT2_LINE_STATE(sys_read8(config->base + UHC0D_MISC_STATUS));

		LOG_DBG("isr: port2 connection event %#x", line_state);

		sys_write8(PORT2_CONNECTION_EVENT, config->base + UHC08_INT_STATUS);
	}

	if (int_sts & PORT1_CONNECTION_EVENT) {
		uint8_t line_state = PORT1_LINE_STATE(sys_read8(config->base + UHC0D_MISC_STATUS));

		LOG_DBG("isr: port1 connection event");

		switch (line_state) {
		case LOW_SPEED:
			sys_write8(sys_read8(config->base + UHC0C_MISC_CTRL) & ~PORT1_FULL_SPEED_EN,
				   config->base + UHC0C_MISC_CTRL);
			sys_write8(sys_read8(config->base + UHC02_TX_LINE_CTRL) &
					   ~(FULL_SPEED_LINE_RATE | FULL_SPEED_LINE_POLARITY),
				   config->base + UHC02_TX_LINE_CTRL);

			uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_LS, 0);
			LOG_DBG("low speed device is connected");
			break;
		case FULL_SPEED:
			sys_write8(sys_read8(config->base + UHC0C_MISC_CTRL) | PORT1_FULL_SPEED_EN,
				   config->base + UHC0C_MISC_CTRL);
			sys_write8(sys_read8(config->base + UHC02_TX_LINE_CTRL) |
					   FULL_SPEED_LINE_RATE | FULL_SPEED_LINE_POLARITY,
				   config->base + UHC02_TX_LINE_CTRL);
			uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_FS, 0);

			LOG_DBG("full speed device is connected");
			break;
		case DISCONNECT:
			uhc_submit_event(dev, UHC_EVT_DEV_REMOVED, 0);
			LOG_DBG("device is disconnected");
			break;
		default:
			LOG_DBG("unknown line stats %#x", line_state);
			break;
		}

		sys_write8(PORT1_CONNECTION_EVENT, config->base + UHC08_INT_STATUS);
	}

	if (int_sts & PORT2_RESUME_EVENT) {
		LOG_DBG("isr: port2 resume event");
		sys_write8(PORT2_RESUME_EVENT, config->base + UHC08_INT_STATUS);
	}

	if (int_sts & PORT1_RESUME_EVENT) {
		LOG_DBG("isr: port1 resume event");
		sys_write8(PORT1_RESUME_EVENT, config->base + UHC08_INT_STATUS);
	}

	if (int_sts & SOF_SENT) {
		LOG_DBG("isr: sof sent");
		sys_write8(SOF_SENT, config->base + UHC08_INT_STATUS);
	}

	if (int_sts & CONNECTION_EVENT) {
		uint8_t line_state = LINE_STATE(sys_read8(config->base + UHC0E_RX_CONNECT_STATE));

		LOG_DBG("isr: connection event, line state");

		switch (line_state) {
		case LOW_SPEED:
			sys_write8(sys_read8(config->base + UHC0C_MISC_CTRL) & ~PORT1_FULL_SPEED_EN,
				   config->base + UHC0C_MISC_CTRL);
			sys_write8(sys_read8(config->base + UHC02_TX_LINE_CTRL) &
					   ~(FULL_SPEED_LINE_RATE | FULL_SPEED_LINE_POLARITY),
				   config->base + UHC02_TX_LINE_CTRL);

			uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_LS, 0);
			LOG_DBG("low speed device is connected");
			break;
		case FULL_SPEED:
			sys_write8(sys_read8(config->base + UHC0C_MISC_CTRL) | PORT1_FULL_SPEED_EN,
				   config->base + UHC0C_MISC_CTRL);
			sys_write8(sys_read8(config->base + UHC02_TX_LINE_CTRL) |
					   FULL_SPEED_LINE_RATE | FULL_SPEED_LINE_POLARITY,
				   config->base + UHC02_TX_LINE_CTRL);

			uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_FS, 0);
			LOG_DBG("full speed device is connected");
			break;
		case DISCONNECT:
			uhc_submit_event(dev, UHC_EVT_DEV_REMOVED, 0);
			LOG_DBG("device is disconnected");
			break;
		default:
			LOG_DBG("unknown line stats %#x", line_state);
			break;
		}

		sys_write8(CONNECTION_EVENT, config->base + UHC08_INT_STATUS);
	}

	if (int_sts & RESUME_EVENT) {
		LOG_DBG("isr: resume event");

		if (atomic_test_and_clear_bit(&priv->state, IT51XXX_STATE_BUS_RESUME)) {
			uhc_submit_event(dev, UHC_EVT_RESUMED, 0);
		}

		sys_write8(RESUME_EVENT, config->base + UHC08_INT_STATUS);
	}

	if (int_sts & XFER_DONE) {
		LOG_DBG("isr: xfer done");

		xfer_done_error_check(dev);

		k_sem_give(&priv->xfer_done_sem);

		sys_write8(XFER_DONE, config->base + UHC08_INT_STATUS);
	}
}

static int it51xxx_uhc_preinit(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	struct it51xxx_data *priv = uhc_get_private(dev);
	struct uhc_data *data = dev->data;
	int ret;
	uint8_t reg_val;

	LOG_INF("%s ite debug %d", __func__, __LINE__);

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("failed to apply pinctrl, ret %d", ret);
		return ret;
	}

#if IT51XXX_UHC_EXTEND_CTRL_ENABLED
	reg_val = sys_read8(config->extend_ctrl.addr);

	if (config->extend_ctrl.enable_bit > 7 || config->extend_ctrl.disable_bit > 7) {
		LOG_ERR("invalid bit setting: enable=%d disable=%d", config->extend_ctrl.enable_bit,
			config->extend_ctrl.disable_bit);
		return -EINVAL;
	}

	reg_val |= BIT(config->extend_ctrl.enable_bit);
	reg_val &= ~BIT(config->extend_ctrl.disable_bit);

	sys_write8(reg_val, config->extend_ctrl.addr);
#endif /* IT51XXX_UHC_EXTEND_CTRL_ENABLED */

	k_mutex_init(&data->mutex);
	k_sem_init(&priv->xfer_done_sem, 0, 1);

	config->make_thread(dev);

	/* enable pull down */
	sys_write8(sys_read8(config->base + UHCE4_PORT1_MISC_CTRL) | PULL_DOWN,
		   config->base + UHCE4_PORT1_MISC_CTRL);
	sys_write8(sys_read8(config->base + UHCE8_PORT2_MISC_CTRL) | PULL_DOWN,
		   config->base + UHCE8_PORT2_MISC_CTRL);

	reg_val = PORT2_CONNECTION_EVENT | PORT1_CONNECTION_EVENT | PORT2_RESUME_EVENT |
		  PORT1_RESUME_EVENT | CONNECTION_EVENT | RESUME_EVENT | XFER_DONE;
	sys_write8(reg_val, config->base + UHC09_INT_MASK);

	irq_connect_dynamic(config->irq_no, 0, it51xxx_uhc_isr, dev, 0);

	return 0;
}

static const struct uhc_api it51xxx_uhc_api = {
	.lock = it51xxx_lock,
	.unlock = it51xxx_unlock,
	.init = uhc_it51xxx_init,
	.enable = uhc_it51xxx_enable,
	.disable = uhc_it51xxx_disable,
	.shutdown = uhc_it51xxx_shutdown,

	.bus_reset = it51xxx_bus_reset,
	.sof_enable = it51xxx_sof_enable,
	.bus_suspend = it51xxx_bus_suspend,
	.bus_resume = it51xxx_bus_resume,

	.ep_enqueue = it51xxx_enqueue,
	.ep_dequeue = it51xxx_dequeue,
};

#if IT51XXX_UHC_EXTEND_CTRL_ENABLED
#define IT51XXX_UHC_EXTEND_CTRL(n)                                                                 \
	{                                                                                          \
		.addr = DT_INST_PROP_BY_IDX(n, extend_ctrl, 0),                                    \
		.enable_bit = DT_INST_PROP_BY_IDX(n, extend_ctrl, 1),                              \
		.disable_bit = DT_INST_PROP_BY_IDX(n, extend_ctrl, 2),                             \
	}
#endif /* IT51XXX_UHC_EXTEND_CTRL_ENABLED */

/* clang-format off */
#define IT51XXX_USB_HOST_DEFINE(n)                                                                 \
                                                                                                   \
	K_KERNEL_STACK_DEFINE(uhc_it51xxx_stack_##n, CONFIG_UHC_IT51XXX_STACK_SIZE);               \
                                                                                                   \
	static void uhc_it51xxx_thread_##n(void *dev, void *arg1, void *arg2)                      \
	{                                                                                          \
		ARG_UNUSED(arg1);                                                                  \
		ARG_UNUSED(arg2);                                                                  \
                                                                                                   \
		xfer_work_handler(dev);                                                            \
	}                                                                                          \
                                                                                                   \
	static void uhc_it51xxx_make_thread_##n(const struct device *dev)                          \
	{                                                                                          \
		struct it51xxx_data *priv = uhc_get_private(dev);                                  \
                                                                                                   \
		k_thread_create(&priv->thread_data, uhc_it51xxx_stack_##n,                         \
				K_THREAD_STACK_SIZEOF(uhc_it51xxx_stack_##n),                      \
				uhc_it51xxx_thread_##n, (void *)dev, NULL, NULL, K_PRIO_COOP(8),   \
				0, K_NO_WAIT);                                                     \
		k_thread_name_set(&priv->thread_data, dev->name);                                  \
	}                                                                                          \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static struct it51xxx_config uhc_cfg_##n = {                                               \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.irq_no = DT_INST_IRQ(n, irq),                                                     \
		.make_thread = uhc_it51xxx_make_thread_##n,                                        \
		IF_ENABLED(IT51XXX_UHC_EXTEND_CTRL_ENABLED, (                                      \
			.extend_ctrl = IT51XXX_UHC_EXTEND_CTRL(n),                                 \
		))                                                                                 \
	};                                                                                         \
                                                                                                   \
	static struct it51xxx_data priv_data_##n = {};                                             \
                                                                                                   \
	static struct uhc_data uhc_data_##n = {                                                    \
		.mutex = Z_MUTEX_INITIALIZER(uhc_data_##n.mutex),                                  \
		.priv = &priv_data_##n,                                                            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, it51xxx_uhc_preinit, NULL, &uhc_data_##n, &uhc_cfg_##n,           \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &it51xxx_uhc_api);

DT_INST_FOREACH_STATUS_OKAY(IT51XXX_USB_HOST_DEFINE)
/* clang-format on */
