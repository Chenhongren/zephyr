/*
 * Copyright (c) 2026 ITE Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "udc_common.h"

#include <soc.h>
#include <soc_dt.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/interrupt_controller/wuc_ite_it51xxx.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/interrupt-controller/ite-it51xxx-wuc.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/policy.h>
LOG_MODULE_REGISTER(udc_it51xxx, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define DT_DRV_COMPAT ite_it51xxx_usb

#define IT51XXX_UDC_EXTERN_CTRL_ENABLED DT_ALL_INST_HAS_PROP_STATUS_OKAY(extern_ctrl)
#define IT51XXX_UDC_NUM_ENDPOINTS       8

#define UDC50_TARGET_CTRL        0x50
#define CONNECT_TO_HOST          BIT(6)
#define FULL_SPEED_LINE_RATE     BIT(5)
#define FULL_SPEED_LINE_POLARITY BIT(4)
#define DIRECT_CONTROL           BIT(3)
#define TX_LINE_STATE_DM         BIT(1)
#define GLOBAL_ENABLE            BIT(0)

#define UDC51_TARGET_LINE_STATUS 0x51
#define RX_LINE_STATE_MASK       GENMASK(1, 0)
#define RX_LINE_RESET            0x0

#define UDC52_TARGET_INT_STATUS 0x52
#define UDC53_TARGET_INT_MASK   0x53
#define NAK_SENT                BIT(4)
#define SOF_RECEIVED            BIT(3)
#define RESET_EVENT             BIT(2)
#define RESUME_EVENT            BIT(1)
#define TRANS_DONE              BIT(0)

#define UDC54_TARGET_ADDR    0x54
#define DEVICE_ADDRESS(x)    FIELD_GET(GENMASK(6, 0), x)
#define DEVICE_ADDRESS_RESET 0x00

#define UDC55_SOF_NUM_MSB 0x55
#define SOF_NUM_MSB(x)    FIELD_GET(GENMASK(2, 0), x)

#define UDC56_SOF_NUM_LSB    0x56
#define UDC58_FIFO_OPERATION 0x58

/* endpoint n */
const uint8_t epn_ctrl_base[IT51XXX_UDC_NUM_ENDPOINTS] = {0x40, 0x44, 0x48, 0x4c,
							  0x68, 0x6c, 0xa8, 0xac};
#define UDC40_EPN_CTRL        0x0
#define ENDPOINT_IN_DIRECTION BIT(5)
#define ENDPOINT_ISO_ENABLE   BIT(4)
#define ENDPOINT_SEND_STALL   BIT(3)
#define ENDPOINT_OUTDATA_SEQ  BIT(2)
#define ENDPOINT_READY        BIT(1)
#define ENDPOINT_ENABLE       BIT(0)

#define UDC41_EPN_STATUS  0x1
#define STS_STALL_SENT    BIT(5)
#define BIT_RX_TIMEOUT    3
#define BIT_RX_OVERFLOW   2
#define BIT_STUFF_ERROR   1
#define BIT_CRC_ERROR     0
#define STS_RX_ERROR_MASK GENMASK(BIT_RX_TIMEOUT, BIT_CRC_ERROR)

#define UDC42_EPN_TRANSACTION_TYPE 0x2
#define TRANSACTION_TYPE(x)        FIELD_GET(GENMASK(1, 0), x)

const uint8_t epn_rx_fifo_base[IT51XXX_UDC_NUM_ENDPOINTS] = {0x60, 0x80, 0xa0, 0xc0,
							     0x88, 0x78, 0xc8, 0xf0};
#define UDC60_EPN_RX_FIFO_DATA      0x0
#define UDC62_EPN_RX_FIFO_COUNT_MSB 0x2
#define UDC63_EPN_RX_FIFO_COUNT_LSB 0x3
#define UDC64_EPN_RX_FIFO_CTRL      0x4

const uint8_t epn_tx_fifo_base[IT51XXX_UDC_NUM_ENDPOINTS] = {0x70, 0x90, 0xb0, 0xd0,
							     0x98, 0xb8, 0xd8, 0xf8};
#define UDC70_EPN_TX_FIFO_DATA 0x0
#define UDC74_EPN_TX_FIFO_CTRL 0x4
#define FIFO_FORCE_EMPTY       BIT(0)

enum it51xxx_transaction_types {
	IT51XXX_XFER_TYPE_SETUP = 0,
	IT51XXX_XFER_TYPE_IN,
	IT51XXX_XFER_TYPE_OUT,
};

enum it51xxx_event_type {
	IT51XXX_EVT_XFER = 0,
	IT51XXX_EVT_SETUP_TOKEN,
	IT51XXX_EVT_OUT_TOKEN,
	IT51XXX_EVT_IN_TOKEN,
};

enum it51xxx_ep_ctrl {
	EP_IN_DIRECTION = 0,
	EP_STALL_SEND,
	EP_IOS_ENABLE,
	EP_ENABLE,
	EP_DATA_SEQ_1,
	EP_DATA_SEQ_TOGGLE,
	EP_READY,
};

struct it51xxx_ep_event {
	sys_snode_t node;
	const struct device *dev;
	uint8_t ep;
	enum it51xxx_event_type event;
};

K_MSGQ_DEFINE(evt_msgq, sizeof(struct it51xxx_ep_event), CONFIG_UDC_IT51XXX_EVENT_COUNT,
	      sizeof(uint32_t));

struct it51xxx_data {
	const struct device *dev;

	struct k_work_delayable suspended_work;
	struct k_thread thread_data;
	struct k_sem suspended_sem;

	/* record if the previous transaction of control endpoint is stall */
	bool stall_is_sent;

	uint16_t sof_num;

	atomic_t in_ep_state;
	atomic_t out_ep_state;
};

struct it51xxx_wuc {
	const struct device *dev;
	uint8_t mask;
};

struct it51xxx_config {
	mm_reg_t base;

#if IT51XXX_UDC_EXTERN_CTRL_ENABLED
	struct {
		mm_reg_t addr;
		uint8_t enable_bit;
		uint8_t disable_bit;
	} extern_ctrl;
#endif /* IT51XXX_UDC_EXTERN_CTRL_ENABLED */

	const struct pinctrl_dev_config *pcfg;
	const struct it51xxx_wuc wuc;

	const struct device *clk_dev;
	struct ite_clk_cfg clk_cfg;

	uint8_t usb_irq;
	uint8_t wu_irq;

	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;

	size_t num_of_in_eps;
	size_t num_of_out_eps;
	size_t num_of_eps;

	void (*make_thread)(const struct device *dev);
};

static void it51xxx_event_submit(const struct device *dev, const uint8_t ep,
				 const enum it51xxx_event_type event)
{
	struct it51xxx_ep_event evt;

	evt.dev = dev;
	evt.ep = ep;
	evt.event = event;
	k_msgq_put(&evt_msgq, &evt, K_NO_WAIT);
}

static void it51xxx_enable_resume_int(const struct device *dev, const bool enable)
{
	const struct it51xxx_config *config = dev->config;
	uint8_t int_mask = sys_read8(config->base + UDC53_TARGET_INT_MASK);

	sys_write8(RESUME_EVENT, config->base + UDC52_TARGET_INT_STATUS);
	if (enable) {
		sys_write8(int_mask | RESUME_EVENT, config->base + UDC53_TARGET_INT_MASK);
	} else {
		sys_write8(int_mask & ~RESUME_EVENT, config->base + UDC53_TARGET_INT_MASK);
	}
}

static void it51xxx_enable_sof_int(const struct device *dev, const bool enable)
{
	const struct it51xxx_config *config = dev->config;
	uint8_t int_mask = sys_read8(config->base + UDC53_TARGET_INT_MASK);

	sys_write8(SOF_RECEIVED, config->base + UDC52_TARGET_INT_STATUS);
	if (enable) {
		sys_write8(int_mask | SOF_RECEIVED, config->base + UDC53_TARGET_INT_MASK);
	} else {
		sys_write8(int_mask & ~SOF_RECEIVED, config->base + UDC53_TARGET_INT_MASK);
	}
}

static void it51xxx_set_ep_ctrl(const struct device *dev, uint8_t ep, enum it51xxx_ep_ctrl ctrl,
				bool enable)
{
	const struct it51xxx_config *config = dev->config;
	mem_addr_t ctrl_base = config->base + epn_ctrl_base[USB_EP_GET_IDX(ep)];
	uint8_t epn_ctrl_val, mask;
	unsigned int key = irq_lock();

	epn_ctrl_val = sys_read8(ctrl_base + UDC40_EPN_CTRL) & ~ENDPOINT_READY;

	switch (ctrl) {
	case EP_STALL_SEND:
		mask = ENDPOINT_SEND_STALL;
		break;
	case EP_IN_DIRECTION:
		mask = ENDPOINT_IN_DIRECTION;
		break;
	case EP_IOS_ENABLE:
		mask = ENDPOINT_ISO_ENABLE;
		break;
	case EP_ENABLE:
		mask = ENDPOINT_ENABLE;
		break;
	case EP_READY:
		mask = ENDPOINT_READY;
		break;
	case EP_DATA_SEQ_1:
		mask = ENDPOINT_OUTDATA_SEQ;
		break;
	case EP_DATA_SEQ_TOGGLE:
		if (enable) {
			epn_ctrl_val ^= ENDPOINT_OUTDATA_SEQ;
		}
		goto write_ctrl;
	default:
		irq_unlock(key);
		LOG_ERR("unknown endpoint control type %#x", ctrl);
		return;
	}

	WRITE_BIT(epn_ctrl_val, __builtin_ctz(mask), enable);

write_ctrl:
	sys_write8(epn_ctrl_val, ctrl_base + UDC40_EPN_CTRL);
	irq_unlock(key);
}

static void it51xxx_enable_standby_state(const bool enable)
{
	if (enable) {
		/* deep doze mode */
		pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	} else {
		pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	}
}

static void it51xxx_enable_wu_irq(const struct device *dev, const bool enable)
{
	const struct it51xxx_config *config = dev->config;

	/* clear pending interrupt */
	it51xxx_wuc_clear_status(config->wuc.dev, config->wuc.mask);

	if (enable) {
		irq_enable(config->wu_irq);
	} else {
		irq_disable(config->wu_irq);
	}
}

static void it51xxx_wu_isr(const void *arg)
{
	const struct device *dev = arg;

	it51xxx_enable_wu_irq(dev, false);
	it51xxx_enable_standby_state(false);
	LOG_DBG("usb d+ (wu) triggered");
}

static void it51xxx_wuc_init(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;

	/* initialize the wui */
	it51xxx_wuc_set_polarity(config->wuc.dev, config->wuc.mask, WUC_TYPE_EDGE_FALLING);
	it51xxx_wuc_clear_status(config->wuc.dev, config->wuc.mask);

	/* enable the wui */
	it51xxx_wuc_enable(config->wuc.dev, config->wuc.mask);

	/* connect wu (usb d+) interrupt but make it disabled initially */
	irq_connect_dynamic(config->wu_irq, 0, it51xxx_wu_isr, dev, 0);
}

static int it51xxx_ep_enqueue(const struct device *dev, struct udc_ep_config *const cfg,
			      struct net_buf *const buf)
{
	udc_buf_put(cfg, buf);

	if (cfg->addr == USB_CONTROL_EP_OUT) {
		struct udc_buf_info *bi = udc_get_buf_info(buf);

		if (bi->setup || bi->status) {
			/* SETUP can be received without any action.
			 * The OUT buffer is queued before the data-in stage finishes. To avoid
			 * missing OUT status, firmware enables `EP_READY_ENABLE` in
			 * `work_handler_in()` instead of immediately after queuing the OUT status
			 * buffer. Therefore, no action is needed for OUT status here.
			 *
			 * If the ACK handshake of the last IN data transaction is corrupted,
			 * hardware will not generate the xfer_done interrupt and will not clear
			 * the `EP_READY_ENABLE` bit set for the IN data stage. In this case, the
			 * device still responds with ACK when the host initiates OUT status stage.
			 */
			return 0;
		}
	}

	it51xxx_event_submit(dev, cfg->addr, IT51XXX_EVT_XFER);

	return 0;
}

static int it51xxx_ep_dequeue(const struct device *dev, struct udc_ep_config *const cfg)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);
	const struct it51xxx_config *config = dev->config;

	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		mem_addr_t tx_fifo_base = config->base + epn_tx_fifo_base[ep_idx];

		sys_write8(sys_read8(tx_fifo_base + UDC74_EPN_TX_FIFO_CTRL) | FIFO_FORCE_EMPTY,
			   tx_fifo_base + UDC74_EPN_TX_FIFO_CTRL);
	} else {
		mem_addr_t rx_fifo_base = config->base + epn_rx_fifo_base[ep_idx];

		sys_write8(sys_read8(rx_fifo_base + UDC64_EPN_RX_FIFO_CTRL) | FIFO_FORCE_EMPTY,
			   rx_fifo_base + UDC64_EPN_RX_FIFO_CTRL);
	}

	udc_ep_cancel_queued(dev, cfg);

	udc_ep_set_busy(cfg, false);

	return 0;
}

static inline void ctrl_ep_stall_workaround(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	struct gctrl_it51xxx_regs *const gctrl_regs = GCTRL_IT51XXX_REGS_BASE;
	struct it51xxx_data *priv = udc_get_private(dev);
	mem_addr_t ctrl_base = config->base + epn_ctrl_base[0];
	unsigned int key;
	uint32_t idx = 0;

	key = irq_lock();
	priv->stall_is_sent = true;
	it51xxx_set_ep_ctrl(dev, 0, EP_STALL_SEND, true);
	it51xxx_set_ep_ctrl(dev, 0, EP_READY, true);

	/* it51xxx does not support clearing the STALL bit by hardware; instead, the STALL bit need
	 * to be cleared by firmware. The SETUP token will be STALLed, which isn't compliant to
	 * USB specification, if firmware clears the STALL bit too late. Due to this hardware
	 * limitations, device controller polls to check if the stall bit has been transmitted for
	 * 3ms and then disables it after responding STALLed.
	 */
	while (idx < 198 && !(sys_read8(ctrl_base + UDC41_EPN_STATUS) & STS_STALL_SENT)) {
		/* wait 15.15us */
		gctrl_regs->GCTRL_WNCKR = 0;
		idx++;
	}

	if (idx < 198) {
		it51xxx_set_ep_ctrl(dev, 0, EP_STALL_SEND, false);
	}
	irq_unlock(key);
}

static int it51xxx_ep_set_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
	if (USB_EP_GET_IDX(cfg->addr) == 0) {
		ctrl_ep_stall_workaround(dev);
	} else {
		it51xxx_set_ep_ctrl(dev, cfg->addr, EP_STALL_SEND, true);
		it51xxx_set_ep_ctrl(dev, cfg->addr, EP_READY, true);
	}

	LOG_DBG("endpoint %#x is halted", cfg->addr);

	return 0;
}

static int it51xxx_ep_clear_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
	it51xxx_set_ep_ctrl(dev, cfg->addr, EP_STALL_SEND, false);
	LOG_DBG("endpoint %#x clear halted", cfg->addr);

	return 0;
}

static int it51xxx_ep_enable(const struct device *dev, struct udc_ep_config *const cfg)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);
	const struct it51xxx_config *config = dev->config;

	if (ep_idx == 0) {
		if ((cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) != USB_EP_TYPE_CONTROL) {
			LOG_ERR("only supported control type for endpoint 0");
			return -ENOTSUP;
		}
	} else {
		switch (cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) {
		case USB_EP_TYPE_BULK:
			__fallthrough;
		case USB_EP_TYPE_INTERRUPT:
			it51xxx_set_ep_ctrl(dev, cfg->addr, EP_IOS_ENABLE, false);
			break;
		case USB_EP_TYPE_ISO:
			it51xxx_set_ep_ctrl(dev, cfg->addr, EP_IOS_ENABLE, true);
			break;
		case USB_EP_TYPE_CONTROL:
			__fallthrough;
		default:
			return -ENOTSUP;
		}

		/* enable enqueue/dequeue operation mode */
		if (ep_idx < IT51XXX_UDC_NUM_ENDPOINTS) {
			sys_write8(sys_read8(config->base + UDC58_FIFO_OPERATION) | BIT(ep_idx),
				   config->base + UDC58_FIFO_OPERATION);
		} else {
			LOG_WRN("ep %#x: unknown operation mode setting", cfg->addr);
		}

		if (USB_EP_DIR_IS_IN(cfg->addr)) {
			it51xxx_set_ep_ctrl(dev, cfg->addr, EP_DATA_SEQ_1, false);
			it51xxx_set_ep_ctrl(dev, cfg->addr, EP_IN_DIRECTION, true);

		} else {
			it51xxx_set_ep_ctrl(dev, cfg->addr, EP_IN_DIRECTION, false);
		}
	}

	it51xxx_set_ep_ctrl(dev, cfg->addr, EP_ENABLE, true);
	LOG_DBG("endpoint %#x is enabled", cfg->addr);

	return 0;
}

static int it51xxx_ep_disable(const struct device *dev, struct udc_ep_config *const cfg)
{
	it51xxx_set_ep_ctrl(dev, cfg->addr, EP_ENABLE, false);
	LOG_DBG("endpoint %#x is disabled", cfg->addr);

	return 0;
}

static int it51xxx_host_wakeup(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	struct it51xxx_data *priv = udc_get_private(dev);
	int ret = -EACCES;

	if (udc_is_suspended(dev)) {
		sys_write8(GLOBAL_ENABLE | FULL_SPEED_LINE_POLARITY | FULL_SPEED_LINE_RATE |
				   DIRECT_CONTROL | TX_LINE_STATE_DM | CONNECT_TO_HOST,
			   config->base + UDC50_TARGET_CTRL);

		/* The remote wakeup device must hold the resume signal for
		 * at least 1 ms but for no more than 15 ms
		 */
		k_msleep(2);

		sys_write8(GLOBAL_ENABLE | FULL_SPEED_LINE_POLARITY | FULL_SPEED_LINE_RATE |
				   CONNECT_TO_HOST,
			   config->base + UDC50_TARGET_CTRL);
		ret = k_sem_take(&priv->suspended_sem, K_MSEC(500));
		if (ret < 0) {
			LOG_ERR("failed to wake up host, %d", ret);
		}
	}

	return ret;
}

static int it51xxx_set_address(const struct device *dev, const uint8_t addr)
{
	const struct it51xxx_config *config = dev->config;

	sys_write8(DEVICE_ADDRESS(addr), config->base + UDC54_TARGET_ADDR);
	LOG_DBG("set usb device address %#x", addr);

	return 0;
}

static void suspended_handler(struct k_work *item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(item);
	struct it51xxx_data *priv = CONTAINER_OF(dwork, struct it51xxx_data, suspended_work);
	const struct device *dev = priv->dev;
	const struct it51xxx_config *config = dev->config;
	unsigned int key;

	if (sys_read8(config->base + UDC52_TARGET_INT_STATUS) & SOF_RECEIVED) {
		sys_write8(SOF_RECEIVED, config->base + UDC52_TARGET_INT_STATUS);
		k_work_reschedule(&priv->suspended_work, K_MSEC(5));
		return;
	}

	key = irq_lock();
	if (!udc_is_suspended(dev) && udc_is_enabled(dev)) {
		udc_set_suspended(dev, true);
		udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
		it51xxx_enable_wu_irq(dev, true);
		it51xxx_enable_standby_state(true);

		k_sem_reset(&priv->suspended_sem);
	}

	it51xxx_enable_resume_int(dev, true);

	if (!IS_ENABLED(CONFIG_UDC_ENABLE_SOF)) {
		it51xxx_enable_sof_int(dev, true);
	}

	irq_unlock(key);
}

static int it51xxx_enable(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	struct it51xxx_data *priv = udc_get_private(dev);

	k_sem_init(&priv->suspended_sem, 0, 1);

	atomic_clear(&priv->in_ep_state);
	atomic_clear(&priv->out_ep_state);

	sys_write8(GLOBAL_ENABLE | FULL_SPEED_LINE_POLARITY | FULL_SPEED_LINE_RATE |
			   CONNECT_TO_HOST,
		   config->base + UDC50_TARGET_CTRL);

	/* enable usb d+ and usb interrupt */
	it51xxx_enable_wu_irq(dev, true);
	irq_enable(config->usb_irq);

	return 0;
}

static int it51xxx_disable(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	uint8_t device_ctrl_val = sys_read8(config->base + UDC50_TARGET_CTRL);

	irq_disable(config->usb_irq);

	/* stop pull-up d+ and d- */
	sys_write8(device_ctrl_val & ~CONNECT_TO_HOST, config->base + UDC50_TARGET_CTRL);

	return 0;
}

static int it51xxx_udc_ip_init(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;

	sys_write8(sys_read8(config->base + UDC52_TARGET_INT_STATUS),
		   config->base + UDC52_TARGET_INT_STATUS);
	sys_write8(0x0, config->base + UDC53_TARGET_INT_MASK);
	sys_write8(TRANS_DONE | RESET_EVENT | SOF_RECEIVED | RESUME_EVENT,
		   config->base + UDC53_TARGET_INT_MASK);

	/* reset device address */
	sys_write8(DEVICE_ADDRESS_RESET, config->base + UDC54_TARGET_ADDR);

	return 0;
}

static int it51xxx_init(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	int ret;

	it51xxx_udc_ip_init(dev);

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL,
				     config->ep_cfg_out[0].caps.mps, 0);
	if (ret) {
		LOG_ERR("failed to enable control out endpoint");
		return ret;
	}

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL,
				     config->ep_cfg_in[0].caps.mps, 0);
	if (ret) {
		LOG_ERR("failed to enable control in endpoint");
		return ret;
	}

	return 0;
}

static int it51xxx_shutdown(const struct device *dev)
{
	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("failed to disable control out endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("failed to disable control in endpoint");
		return -EIO;
	}

	return 0;
}

static void it51xxx_lock(const struct device *dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void it51xxx_unlock(const struct device *dev)
{
	udc_unlock_internal(dev);
}

static const struct udc_api it51xxx_api = {
	.ep_enqueue = it51xxx_ep_enqueue,
	.ep_dequeue = it51xxx_ep_dequeue,
	.ep_set_halt = it51xxx_ep_set_halt,
	.ep_clear_halt = it51xxx_ep_clear_halt,
	.ep_enable = it51xxx_ep_enable,
	.ep_disable = it51xxx_ep_disable,
	.host_wakeup = it51xxx_host_wakeup,
	.set_address = it51xxx_set_address,
	.enable = it51xxx_enable,
	.disable = it51xxx_disable,
	.init = it51xxx_init,
	.shutdown = it51xxx_shutdown,
	.lock = it51xxx_lock,
	.unlock = it51xxx_unlock,
};

static int it51xxx_xfer_in_data(const struct device *dev, uint8_t ep, struct net_buf *buf)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(ep);
	const struct it51xxx_config *config = dev->config;
	struct it51xxx_data *priv = udc_get_private(dev);
	mem_addr_t tx_fifo_base = config->base + epn_tx_fifo_base[ep_idx];
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep);
	size_t len;
	unsigned int key;

	sys_write8(sys_read8(tx_fifo_base + UDC74_EPN_TX_FIFO_CTRL) | FIFO_FORCE_EMPTY,
		   tx_fifo_base + UDC74_EPN_TX_FIFO_CTRL);

	if (ep_idx == 0) {
		if (udc_get_buf_info(buf)->status) {
			it51xxx_set_ep_ctrl(dev, ep, EP_DATA_SEQ_1, true);
		}
	} else {
		key = irq_lock();
		atomic_set_bit(&priv->in_ep_state, ep_idx);
	}

	len = MIN(buf->len, udc_mps_ep_size(ep_cfg));
	for (size_t i = 0; i < len; i++) {
		sys_write8(buf->data[i], tx_fifo_base + UDC70_EPN_TX_FIFO_DATA);
	}

	it51xxx_set_ep_ctrl(dev, ep, EP_READY, true);
	if (ep_idx != 0) {
		irq_unlock(key);
	}

	LOG_DBG("written %d packets to endpoint %#x tx fifo", len, ep);

	return 0;
}

static int it51xxx_xfer_out_data(const struct device *dev, uint8_t ep, struct net_buf *buf,
				 size_t len)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(ep);
	const struct it51xxx_config *config = dev->config;
	mem_addr_t rx_fifo_base = config->base + epn_rx_fifo_base[ep_idx];
	uint8_t epn_sts_val = sys_read8(config->base + epn_ctrl_base[ep_idx] + UDC41_EPN_STATUS);

	if (epn_sts_val & STS_RX_ERROR_MASK) {
		if (IS_BIT_SET(epn_sts_val, BIT_RX_TIMEOUT)) {
			LOG_ERR("rx timeout occurs");
		}
		if (IS_BIT_SET(epn_sts_val, BIT_RX_OVERFLOW)) {
			LOG_ERR("rx overflow occurs");
		}
		if (IS_BIT_SET(epn_sts_val, BIT_STUFF_ERROR)) {
			LOG_ERR("rx stuff error");
		}
		if (IS_BIT_SET(epn_sts_val, BIT_CRC_ERROR)) {
			LOG_ERR("rx crc error");
		}
		return -EINVAL;
	}

	len = MIN(net_buf_tailroom(buf), len);
	uint8_t *data_ptr = net_buf_tail(buf);

	for (size_t i = 0; i < len; i++) {
		data_ptr[i] = sys_read8(rx_fifo_base + UDC60_EPN_RX_FIFO_DATA);
	}

	net_buf_add(buf, len);

	return 0;
}

static int work_handler_xfer_continue(const struct device *dev, uint8_t ep, struct net_buf *buf)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (USB_EP_DIR_IS_OUT(ep)) {
		unsigned int key;

		if (ep_idx != 0) {
			struct it51xxx_data *priv = udc_get_private(dev);

			key = irq_lock();
			atomic_set_bit(&priv->out_ep_state, ep_idx);
		}

		it51xxx_set_ep_ctrl(dev, ep_idx, EP_READY, true);
		if (ep_idx != 0) {
			irq_unlock(key);
		}
		return 0;
	}

	return it51xxx_xfer_in_data(dev, ep, buf);
}

static bool it51xxx_fake_token(const struct device *dev, const uint8_t ep, const uint8_t token_type)
{
	struct it51xxx_data *priv = udc_get_private(dev);
	const uint8_t ep_idx = USB_EP_GET_IDX(ep);

	/* When usb transaction done, firmware polls all endpoint xfer_done
	 * flags (enabled but not ready) to identify the completed endpoint.
	 * However, hardware doesn't clear the flags and transfer type, so
	 * stale transfer type information may persist and cause fake token
	 * detection.
	 *
	 * For control endpoint, firmware uses buffer presence to filter fake
	 * tokens. For non-control endpoints, firmware checks the
	 * corresponding endpoint state bit.
	 */
	switch (token_type) {
	case IT51XXX_XFER_TYPE_IN:
		if (ep_idx == 0) {
			struct udc_ep_config *ep_cfg;

			if (priv->stall_is_sent) {
				return true;
			}
			ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);

			return udc_buf_peek(ep_cfg) == NULL;
		}

		return !atomic_test_bit(&priv->in_ep_state, ep_idx);

	case IT51XXX_XFER_TYPE_OUT:
		if (ep_idx == 0) {
			struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
			struct net_buf *buf = udc_buf_peek(ep_cfg);

			if (!buf) {
				return true;
			}

			return udc_get_buf_info(buf)->setup;
		}

		return !atomic_test_bit(&priv->out_ep_state, ep_idx);

	default:
		LOG_ERR("invalid token type(%d)", token_type);
		return true;
	}
}

static inline int work_handler_in(const struct device *dev, uint8_t ep)
{
	struct it51xxx_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg;
	struct net_buf *buf;

	if (it51xxx_fake_token(dev, ep, IT51XXX_XFER_TYPE_IN)) {
		return 0;
	}

	if (ep != USB_CONTROL_EP_IN) {
		unsigned int key = irq_lock();

		atomic_clear_bit(&priv->in_ep_state, USB_EP_GET_IDX(ep));
		irq_unlock(key);
	}

	ep_cfg = udc_get_ep_cfg(dev, ep);

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		LOG_ERR("ep %#x: no buffer to peek", ep);
		return -ENODATA;
	}

	net_buf_pull(buf, MIN(buf->len, udc_mps_ep_size(ep_cfg)));

	it51xxx_set_ep_ctrl(dev, ep, EP_DATA_SEQ_TOGGLE, true);

	if (buf->len) {
		work_handler_xfer_continue(dev, ep, buf);
		return 0;
	}

	if (udc_ep_buf_has_zlp(buf)) {
		work_handler_xfer_continue(dev, ep, buf);
		udc_ep_buf_clear_zlp(buf);
		return 0;
	}

	buf = udc_buf_get(ep_cfg);
	if (buf == NULL) {
		LOG_ERR("ep %#x: no buffer to get", ep);
		return -ENODATA;
	}

	udc_ep_set_busy(ep_cfg, false);

	if (ep == USB_CONTROL_EP_IN) {
		struct udc_buf_info *bi = udc_get_buf_info(buf);

		if (bi->data) {
			/* The `EP_READY_ENABLE` bit enables responses to host-initiated
			 * transactions and is shared by all transaction types (SETUP, IN, and OUT).
			 * The bit is automatically cleared to 0 when the transaction completes.
			 *
			 * The `EP_READY_ENABLE` for OUT status stage must be enabled only after the
			 * IN data token interrupt is received; otherwise the OUT status transaction
			 * may be missed.
			 */
			it51xxx_set_ep_ctrl(dev, ep, EP_READY, true);
		}
	}

	return udc_submit_ep_event(dev, buf, 0);
}

static inline int work_handler_setup(const struct device *dev, uint8_t ep)
{
	const struct it51xxx_config *config = dev->config;
	mem_addr_t ctrl_base = config->base + epn_ctrl_base[USB_EP_GET_IDX(ep)];
	mem_addr_t rx_fifo_base = config->base + epn_rx_fifo_base[USB_EP_GET_IDX(ep)];
	struct it51xxx_data *priv = udc_get_private(dev);
	uint8_t setup[sizeof(struct usb_setup_packet)];
	size_t len;

	if (sys_read8(ctrl_base + UDC41_EPN_STATUS) & STS_RX_ERROR_MASK) {
		LOG_WRN("ctrl ep error status %#x", sys_read8(ctrl_base + UDC41_EPN_STATUS));
		return -EINVAL;
	}

	len = sys_read8(rx_fifo_base + UDC63_EPN_RX_FIFO_COUNT_LSB) +
	      (sys_read8(rx_fifo_base + UDC62_EPN_RX_FIFO_COUNT_MSB) << 8);

	if (len != sizeof(struct usb_setup_packet)) {
		LOG_DBG("setup: %d bytes read from chip", len);
		return 0;
	}

	for (size_t idx = 0; idx < len; idx++) {
		setup[idx] = sys_read8(rx_fifo_base + UDC60_EPN_RX_FIFO_DATA);
	}
	LOG_HEXDUMP_DBG(setup, len, "setup:");

	priv->stall_is_sent = false;

	it51xxx_set_ep_ctrl(dev, USB_CONTROL_EP_OUT, EP_DATA_SEQ_1, true);

	udc_setup_received(dev, setup);

	return 0;
}

static inline int work_handler_out(const struct device *dev, uint8_t ep)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(ep);
	const struct it51xxx_config *config = dev->config;
	struct it51xxx_data *priv = udc_get_private(dev);
	mem_addr_t rx_fifo_base = config->base + epn_rx_fifo_base[ep_idx];
	int ret;
	size_t len;
	struct net_buf *buf;
	struct udc_ep_config *ep_cfg;

	if (it51xxx_fake_token(dev, ep, IT51XXX_XFER_TYPE_OUT)) {
		return 0;
	}

	ep_cfg = udc_get_ep_cfg(dev, ep);
	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		LOG_ERR("ep %#x: no buffer to peek", ep);
		return -ENODATA;
	}

	len = (sys_read8(rx_fifo_base + UDC62_EPN_RX_FIFO_COUNT_MSB) << 8) +
	      sys_read8(rx_fifo_base + UDC63_EPN_RX_FIFO_COUNT_LSB);

	if (len > udc_mps_ep_size(ep_cfg)) {
		LOG_ERR("failed to handle this packet due to the packet size");
		return -ENOBUFS;
	}

	if (ep == USB_CONTROL_EP_OUT) {
		if (udc_get_buf_info(buf)->status && len != 0) {
			LOG_DBG("handled early setup token, %d", len);
			buf = udc_buf_get(ep_cfg);
			return udc_submit_ep_event(dev, buf, 0);
		}
	}

	ret = it51xxx_xfer_out_data(dev, ep, buf, len);
	if (ret) {
		return ret;
	}

	LOG_DBG("ep %#x: handle data out, %zu | %zu", ep, len, net_buf_tailroom(buf));

	if (net_buf_tailroom(buf) && len == udc_mps_ep_size(ep_cfg)) {
		work_handler_xfer_continue(dev, ep, buf);
		if (ep != USB_CONTROL_EP_OUT) {
			ret = udc_submit_ep_event(dev, buf, 0);
		}
		return ret;
	}

	buf = udc_buf_get(ep_cfg);
	if (buf == NULL) {
		LOG_ERR("ep %#x: no buffer to get", ep);
		return -ENODATA;
	}

	udc_ep_set_busy(ep_cfg, false);

	if (ep != USB_CONTROL_EP_OUT) {
		unsigned int key = irq_lock();

		atomic_clear_bit(&priv->out_ep_state, ep_idx);
		irq_unlock(key);
	}

	ret = udc_submit_ep_event(dev, buf, 0);

	return ret;
}

static int work_handler_xfer_next(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	struct net_buf *buf;

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		return -ENODATA;
	}

	return work_handler_xfer_continue(dev, ep_cfg->addr, buf);
}

static void xfer_work_handler(const struct device *dev)
{
	while (true) {
		struct udc_ep_config *ep_cfg;
		struct it51xxx_ep_event evt;
		int ret = 0;

		k_msgq_get(&evt_msgq, &evt, K_FOREVER);

		ep_cfg = udc_get_ep_cfg(evt.dev, evt.ep);

		switch (evt.event) {
		case IT51XXX_EVT_SETUP_TOKEN:
			ret = work_handler_setup(evt.dev, evt.ep);
			break;
		case IT51XXX_EVT_IN_TOKEN:
			ret = work_handler_in(evt.dev, evt.ep);
			break;
		case IT51XXX_EVT_OUT_TOKEN:
			ret = work_handler_out(evt.dev, evt.ep);
			break;
		case IT51XXX_EVT_XFER:
			if (evt.ep == USB_CONTROL_EP_OUT) {
				it51xxx_set_ep_ctrl(dev, 0, EP_READY, true);
			}
			break;
		default:
			LOG_ERR("unknown event type %#x", evt.event);
			ret = -EINVAL;
			break;
		}

		if (ret) {
			udc_submit_event(evt.dev, UDC_EVT_ERROR, ret);
		}

		if (evt.ep != USB_CONTROL_EP_OUT && !udc_ep_is_busy(ep_cfg)) {
			if (work_handler_xfer_next(dev, ep_cfg) == 0) {
				udc_ep_set_busy(ep_cfg, true);
			}
		}
	}
}

static void it51xxx_udc_reset(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	struct it51xxx_data *priv = udc_get_private(dev);
	mem_addr_t rx_fifo_base, tx_fifo_base;

	atomic_clear(&priv->in_ep_state);
	atomic_clear(&priv->out_ep_state);

	for (uint8_t ep_idx = 0; ep_idx < config->num_of_eps; ep_idx++) {
		rx_fifo_base = config->base + epn_rx_fifo_base[ep_idx];
		tx_fifo_base = config->base + epn_tx_fifo_base[ep_idx];

		sys_write8(sys_read8(tx_fifo_base + UDC74_EPN_TX_FIFO_CTRL) | FIFO_FORCE_EMPTY,
			   tx_fifo_base + UDC74_EPN_TX_FIFO_CTRL);

		sys_write8(sys_read8(rx_fifo_base + UDC64_EPN_RX_FIFO_CTRL) | FIFO_FORCE_EMPTY,
			   rx_fifo_base + UDC64_EPN_RX_FIFO_CTRL);
	}

	sys_write8(ENDPOINT_ENABLE, config->base + epn_ctrl_base[0] + UDC40_EPN_CTRL);
	sys_write8(DEVICE_ADDRESS_RESET, config->base + UDC54_TARGET_ADDR);
	sys_write8(NAK_SENT | SOF_RECEIVED, config->base + UDC52_TARGET_INT_STATUS);
}

static inline bool ctrl_ep_is_stalled(const struct device *dev, const uint8_t transtype)
{
	const struct it51xxx_config *config = dev->config;
	mem_addr_t ctrl_base = config->base + epn_ctrl_base[0];
	mem_addr_t rx_fifo_base = config->base + epn_rx_fifo_base[0];

	/* check if the stall bit is set */
	if (sys_read8(ctrl_base + UDC40_EPN_CTRL) & ENDPOINT_SEND_STALL) {
		it51xxx_set_ep_ctrl(dev, 0, EP_STALL_SEND, false);
		if (transtype == IT51XXX_XFER_TYPE_SETUP) {
			sys_write8(sys_read8(rx_fifo_base + UDC64_EPN_RX_FIFO_CTRL) |
					   FIFO_FORCE_EMPTY,
				   rx_fifo_base + UDC64_EPN_RX_FIFO_CTRL);
		}
		LOG_ERR("%#x: cleared stall bit", transtype);
		return true;
	}

	/* check if the in transaction is stalled */
	if (transtype == IT51XXX_XFER_TYPE_IN &&
	    sys_read8(ctrl_base + UDC41_EPN_STATUS) & STS_STALL_SENT) {
		return true;
	}

	return false;
}

static void it51xxx_udc_xfer_done(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;

	for (uint8_t ep_idx = 0; ep_idx < config->num_of_eps; ep_idx++) {
		mem_addr_t ctrl_base = config->base + epn_ctrl_base[ep_idx];
		uint8_t epn_ctrl_val = sys_read8(ctrl_base + UDC40_EPN_CTRL);
		uint8_t xfer_type =
			TRANSACTION_TYPE(sys_read8(ctrl_base + UDC42_EPN_TRANSACTION_TYPE));

		if (ep_idx == 0) {
			if (ctrl_ep_is_stalled(dev, xfer_type)) {
				continue;
			}
		} else {
			if (xfer_type == IT51XXX_XFER_TYPE_SETUP ||
			    it51xxx_fake_token(dev, ep_idx, xfer_type)) {
				continue;
			}
		}

		/* the enable bit is set and the ready bit is cleared if the
		 * transaction is completed.
		 */
		if (!(epn_ctrl_val & ENDPOINT_ENABLE) || epn_ctrl_val & ENDPOINT_READY) {
			continue;
		}

		switch (xfer_type) {
		case IT51XXX_XFER_TYPE_SETUP:
			/* setup xfer done */
			it51xxx_event_submit(dev, ep_idx, IT51XXX_EVT_SETUP_TOKEN);
			break;
		case IT51XXX_XFER_TYPE_IN:
			/* in xfer done */
			it51xxx_event_submit(dev, USB_EP_DIR_IN | ep_idx, IT51XXX_EVT_IN_TOKEN);
			break;
		case IT51XXX_XFER_TYPE_OUT:
			/* out xfer done */
			it51xxx_event_submit(dev, USB_EP_DIR_OUT | ep_idx, IT51XXX_EVT_OUT_TOKEN);
			break;
		default:
			LOG_ERR("isr: unknown xfer type(%d)", xfer_type);
			break;
		}
	}
}

static inline void emit_resume_event(const struct device *dev)
{
	struct it51xxx_data *priv = udc_get_private(dev);

	if (udc_is_suspended(dev) && udc_is_enabled(dev)) {
		udc_set_suspended(dev, false);
		udc_submit_event(dev, UDC_EVT_RESUME, 0);
		k_sem_give(&priv->suspended_sem);
	}
}

static void it51xxx_udc_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct it51xxx_config *config = dev->config;
	struct it51xxx_data *priv = udc_get_private(dev);
	uint8_t int_sts = sys_read8(config->base + UDC52_TARGET_INT_STATUS);

	LOG_DBG("isr: status %#x", int_sts);

	if (int_sts & RESET_EVENT) {
		uint8_t line_sts = sys_read8(config->base + UDC51_TARGET_LINE_STATUS);

		if ((line_sts & RX_LINE_STATE_MASK) == RX_LINE_RESET) {
			it51xxx_udc_reset(dev);
			sys_write8(RESET_EVENT, config->base + UDC52_TARGET_INT_STATUS);

			udc_submit_event(dev, UDC_EVT_RESET, 0);
			return;
		}
		sys_write8(RESET_EVENT, config->base + UDC52_TARGET_INT_STATUS);
	}

	if (int_sts & SOF_RECEIVED) {
		LOG_DBG("isr: sof event");
		if (!IS_ENABLED(CONFIG_UDC_ENABLE_SOF)) {
			it51xxx_enable_sof_int(dev, false);
		} else {
			priv->sof_num =
				(SOF_NUM_MSB(sys_read8(config->base + UDC55_SOF_NUM_MSB)) << 8) |
				sys_read8(config->base + UDC56_SOF_NUM_LSB);
			sys_write8(SOF_RECEIVED, config->base + UDC52_TARGET_INT_STATUS);
			udc_submit_sof_event(dev);
		}
		it51xxx_enable_resume_int(dev, false);
		emit_resume_event(dev);
		k_work_cancel_delayable(&priv->suspended_work);
		k_work_reschedule(&priv->suspended_work, K_MSEC(5));
	}

	if (int_sts & RESUME_EVENT) {
		LOG_DBG("isr: resume event");
		it51xxx_enable_resume_int(dev, false);
		emit_resume_event(dev);
	}

	if (int_sts & TRANS_DONE) {
		LOG_DBG("isr: xfer done event");
		sys_write8(TRANS_DONE, config->base + UDC52_TARGET_INT_STATUS);
		it51xxx_udc_xfer_done(dev);
		return;
	}
}

static int it51xxx_udc_preinit(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	struct udc_data *data = dev->data;
	struct it51xxx_data *priv = udc_get_private(dev);
	int ret;
	uint32_t clk_pll;

	k_mutex_init(&data->mutex);

	ret = clock_control_get_rate(config->clk_dev, (clock_control_subsys_t)&config->clk_cfg,
				     &clk_pll);
	if (ret || clk_pll != KHZ(96000)) {
		LOG_ERR("unsupported pll frequency(%uHz), ret %d", clk_pll, ret);

		return ret ? ret : -EINVAL;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("failed to apply pinctrl, ret %d", ret);
		return ret;
	}

#if IT51XXX_UDC_EXTERN_CTRL_ENABLED
	uint8_t reg_val = sys_read8(config->extern_ctrl.addr);

	if (config->extern_ctrl.enable_bit > 7 || config->extern_ctrl.disable_bit > 7) {
		LOG_ERR("invalid bit setting: enable=%d disable=%d", config->extern_ctrl.enable_bit,
			config->extern_ctrl.disable_bit);
		return -EINVAL;
	}

	reg_val |= BIT(config->extern_ctrl.enable_bit);
	reg_val &= ~BIT(config->extern_ctrl.disable_bit);

	sys_write8(reg_val, config->extern_ctrl.addr);
#endif /* IT51XXX_UDC_EXTERN_CTRL_ENABLED */

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.mps = USB_CONTROL_EP_MPS;
		} else if (i >= config->num_of_in_eps) {
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.iso = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		}

		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		ret = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (ret) {
			LOG_ERR("failed to register out endpoint %#x", config->ep_cfg_out[i].addr);
			return ret;
		}
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			config->ep_cfg_in[i].caps.control = 1;
			config->ep_cfg_in[i].caps.mps = USB_CONTROL_EP_MPS;
		} else if (i < config->num_of_in_eps) {
			config->ep_cfg_in[i].caps.bulk = 1;
			config->ep_cfg_in[i].caps.interrupt = 1;
			config->ep_cfg_in[i].caps.iso = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		}

		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		ret = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (ret != 0) {
			LOG_ERR("failed to register in endpoint %#x", config->ep_cfg_in[i].addr);
			return ret;
		}
	}

	data->caps.rwup = true;
	data->caps.mps0 = UDC_MPS0_64;

	priv->dev = dev;

	k_work_init_delayable(&priv->suspended_work, suspended_handler);

	config->make_thread(dev);

	it51xxx_wuc_init(dev);

	/* connect usb interrupt */
	irq_connect_dynamic(config->usb_irq, 0, it51xxx_udc_isr, dev, 0);

	return 0;
}

BUILD_ASSERT(DT_ALL_INST_HAS_PROP_STATUS_OKAY(extern_ctrl) ||
		     !DT_ANY_INST_HAS_PROP_STATUS_OKAY(extern_ctrl),
	     "extern_ctrl must be defined on either all instances or none");

#if IT51XXX_UDC_EXTERN_CTRL_ENABLED
#define IT51XXX_UDC_EXTERN_CTRL(n)                                                                 \
	{                                                                                          \
		.addr = DT_INST_PROP_BY_IDX(n, extern_ctrl, 0),                                    \
		.enable_bit = DT_INST_PROP_BY_IDX(n, extern_ctrl, 1),                              \
		.disable_bit = DT_INST_PROP_BY_IDX(n, extern_ctrl, 2),                             \
	}
#endif /* IT51XXX_UDC_EXTERN_CTRL_ENABLED */

#define IT51XXX_NUM_OF_EPS(n)                                                                      \
	DT_INST_PROP(n, num_in_endpoints) + DT_INST_PROP(n, num_out_endpoints) -                   \
		DT_INST_PROP(n, num_bidir_endpoints)

/* There are a total of 8 endpoints on IT51XXX SoCs. Except for the control endpoint,
 * each endpoint can only be assigned as either TX or RX. The endpoint direction is
 * configured by the firmware (configuration in DTS) during initialization.
 * Add the build assertion to prevent invalid endpoint number configurations.
 */
#define IT51XXX_USB_EP_NUM_ASSERT(n)                                                               \
	BUILD_ASSERT(IT51XXX_NUM_OF_EPS(n) == IT51XXX_UDC_NUM_ENDPOINTS,                           \
		     "invalid in/out endpoint configuration")

/* clang-format off */
#define IT51XXX_USB_DEVICE_DEFINE(n)                                                               \
	BUILD_ASSERT(DT_INST_PROP(n, num_bidir_endpoints) == 1,                                    \
		     "only supported 1 bi-direction endpoint (ctrl ep)");                          \
	IT51XXX_USB_EP_NUM_ASSERT(n);                                                              \
                                                                                                   \
	K_KERNEL_STACK_DEFINE(udc_it51xxx_stack_##n, CONFIG_UDC_IT51XXX_STACK_SIZE);               \
                                                                                                   \
	static void udc_it51xxx_thread_##n(void *dev, void *arg1, void *arg2)                      \
	{                                                                                          \
		ARG_UNUSED(arg1);                                                                  \
		ARG_UNUSED(arg2);                                                                  \
                                                                                                   \
		xfer_work_handler(dev);                                                            \
	}                                                                                          \
                                                                                                   \
	static void udc_it51xxx_make_thread_##n(const struct device *dev)                          \
	{                                                                                          \
		struct it51xxx_data *priv = udc_get_private(dev);                                  \
                                                                                                   \
		k_thread_create(&priv->thread_data, udc_it51xxx_stack_##n,                         \
				K_THREAD_STACK_SIZEOF(udc_it51xxx_stack_##n),                      \
				udc_it51xxx_thread_##n, (void *)dev, NULL, NULL, K_PRIO_COOP(8),   \
				0, K_NO_WAIT);                                                     \
		k_thread_name_set(&priv->thread_data, dev->name);                                  \
	}                                                                                          \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static struct udc_ep_config ep_cfg_out[IT51XXX_NUM_OF_EPS(n)];                             \
	static struct udc_ep_config ep_cfg_in[IT51XXX_NUM_OF_EPS(n)];                              \
                                                                                                   \
	static struct it51xxx_config udc_cfg_##n = {                                               \
		.base = DT_INST_REG_ADDR(n),                                                       \
		IF_ENABLED(IT51XXX_UDC_EXTERN_CTRL_ENABLED, (                                      \
			.extern_ctrl = IT51XXX_UDC_EXTERN_CTRL(n),                                 \
		))                                                                                 \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clk_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                              \
		.clk_cfg =                                                                         \
			{                                                                          \
				.ctrl = DT_INST_CLOCKS_CELL(n, ctrl),                              \
				.bits = DT_INST_CLOCKS_CELL(n, bits),                              \
			},                                                                         \
		.wuc = {.dev = IT8XXX2_DEV_WUC(0, n), .mask = IT8XXX2_DEV_WUC_MASK(0, n)},         \
		.usb_irq = DT_INST_IRQ_BY_IDX(n, 0, irq),                                          \
		.wu_irq = DT_INST_IRQ_BY_IDX(n, 1, irq),                                           \
		.ep_cfg_in = ep_cfg_out,                                                           \
		.ep_cfg_out = ep_cfg_in,                                                           \
		.num_of_in_eps = DT_INST_PROP(n, num_in_endpoints),                                \
		.num_of_out_eps = DT_INST_PROP(n, num_out_endpoints),                              \
		.num_of_eps = IT51XXX_NUM_OF_EPS(n),                                               \
		.make_thread = udc_it51xxx_make_thread_##n,                                        \
	};                                                                                         \
                                                                                                   \
	static struct it51xxx_data priv_data_##n = {};                                             \
                                                                                                   \
	static struct udc_data udc_data_##n = {                                                    \
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),                                  \
		.priv = &priv_data_##n,                                                            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, it51xxx_udc_preinit, NULL, &udc_data_##n, &udc_cfg_##n,           \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &it51xxx_api);

DT_INST_FOREACH_STATUS_OKAY(IT51XXX_USB_DEVICE_DEFINE)
/* clang-format on */
