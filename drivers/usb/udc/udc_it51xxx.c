/*
 * Copyright (c) 2026 ITE Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "udc_common.h"

#include <soc.h>
#include <soc_dt.h>
#include <zephyr/pm/policy.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/interrupt_controller/wuc_ite_it51xxx.h>
#include <zephyr/dt-bindings/interrupt-controller/ite-it51xxx-wuc.h>
#if 0 /* TODO */
LOG_MODULE_REGISTER(udc_it51xxx, CONFIG_UDC_DRIVER_LOG_LEVEL);
#else
LOG_MODULE_REGISTER(udc_it51xxx, LOG_LEVEL_DBG);
#endif

#define DT_DRV_COMPAT ite_it51xxx_usb

#define UDC50_TARGET_CTRL        0x50
#define EP0_READY_MODE           BIT(7)
#define CONNECT_TO_HOST          BIT(6) /* internal pull-up */
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

#define UDC55_TARGET_FRAME_NUM_MOST_SIGNIFICANT_PART  0x55
#define UDC56_TARGET_FRAME_NUM_LEAST_SIGNIFICANT_PART 0x56
#define UDC58_FIFO_OPERATION                          0x58

/* endpoint n */
uint8_t epn_ctrl_base[8] = {0x40, 0x44, 0x48, 0x4c, 0x68, 0x6c, 0xa8, 0xac};
#define UDC40_EPN_CTRL        0x0
#define ENDPOINT_IN_DIRECTION BIT(5)
#define ENDPOINT_ISO_ENABLE   BIT(4)
#define ENDPOINT_SEND_STALL   BIT(3)
#define ENDPOINT_OUTDATA_SEQ  BIT(2)
#define ENDPOINT_READY        BIT(1)
#define ENDPOINT_ENABLE       BIT(0)

#define UDC41_EPN_STATUS 0x1
#define DC_STALL_SENT    BIT(5)
#define BIT_RX_TIMEOUT   3
#define BIT_RX_OVERFLOW  2
#define BIT_STUFF_ERROR  1
#define BIT_CRC_ERROR    0
#define RX_ERROR_MASK    GENMASK(BIT_RX_TIMEOUT, BIT_CRC_ERROR)

#define UDC42_EPN_TRANSACTION_TYPE     0x2
#define UDC43_EPN_NAK_TRANSACTION_TYPE 0x3
#define TRANSACTION_TYPE(x)            FIELD_GET(GENMASK(1, 0), x)

uint8_t epn_rx_fifo_base[8] = {0x60, 0x80, 0xa0, 0xc0, 0x88, 0x78, 0xc8, 0xf0};
#define UDC60_EPN_RX_FIFO_DATA      0x0
#define UDC62_EPN_RX_FIFO_COUNT_MSB 0x2
#define UDC63_EPN_RX_FIFO_COUNT_LSB 0x3
#define UDC64_EPN_RX_FIFO_CTRL      0x4

uint8_t epn_tx_fifo_base[8] = {0x70, 0x90, 0xb0, 0xd0, 0x98, 0xb8, 0xd8, 0xf8};
#define UDC70_EPN_TX_FIFO_DATA 0x0
#define UDC74_EPN_TX_FIFO_CTRL 0x4
#define FIFO_FORCE_EMPTY       BIT(0)

enum it51xxx_transaction_types {
	IT51XXX_XFER_TYPE_SETUP = 0,
	IT51XXX_XFER_TYPE_IN,
	IT51XXX_XFER_TYPE_OUT,
};

enum it51xxx_event_type {
	IT51XXX_EVT_XFER,
	IT51XXX_EVT_SETUP_TOKEN,
	IT51XXX_EVT_OUT_TOKEN,
	IT51XXX_EVT_IN_TOKEN,
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

	struct k_thread thread_data;

	/* record if the previous transaction of control endpoint is stall */
	bool stall_is_sent;
};

struct it51xxx_config {
	mm_reg_t base;
	const struct pinctrl_dev_config *pcfg;
	uint8_t usb_irq;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
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

static int it51xxx_ep_enqueue(const struct device *dev, struct udc_ep_config *const cfg,
			      struct net_buf *const buf)
{
	udc_buf_put(cfg, buf);
	it51xxx_event_submit(dev, cfg->addr, IT51XXX_EVT_XFER);

	return 0;
}

static int it51xxx_ep_dequeue(const struct device *dev, struct udc_ep_config *const cfg)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);
	const struct it51xxx_config *config = dev->config;
	struct net_buf *buf;
	unsigned int lock_key;

	lock_key = irq_lock();
	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		mem_addr_t tx_fifo_base = config->base + epn_tx_fifo_base[ep_idx];

		sys_write8(sys_read8(tx_fifo_base + UDC74_EPN_TX_FIFO_CTRL) | FIFO_FORCE_EMPTY,
			   tx_fifo_base + UDC74_EPN_TX_FIFO_CTRL);
	} else {
		mem_addr_t rx_fifo_base = config->base + epn_rx_fifo_base[ep_idx];

		sys_write8(sys_read8(rx_fifo_base + UDC64_EPN_RX_FIFO_CTRL) | FIFO_FORCE_EMPTY,
			   rx_fifo_base + UDC64_EPN_RX_FIFO_CTRL);
	}
	irq_unlock(lock_key);

	buf = udc_buf_get_all(cfg);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	udc_ep_set_busy(cfg, false);

	return 0;
}

static inline void ctrl_ep_stall_workaround(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	struct gctrl_it51xxx_regs *const gctrl_regs = GCTRL_IT51XXX_REGS_BASE;
	struct it51xxx_data *priv = udc_get_private(dev);
	mem_addr_t ctrl_base = config->base + epn_ctrl_base[0];
	uint8_t epn_ctrl_val;
	unsigned int lock_key;
	uint32_t idx = 0;

	priv->stall_is_sent = true;
	lock_key = irq_lock();
	epn_ctrl_val =
		sys_read8(ctrl_base + UDC40_EPN_CTRL) | (ENDPOINT_SEND_STALL | ENDPOINT_READY);
	sys_write8(epn_ctrl_val, ctrl_base + UDC40_EPN_CTRL);

	/* it51xxx does not support clearing the STALL bit by hardware; instead, the STALL bit need
	 * to be cleared by firmware. The SETUP token will be STALLed, which isn't compliant to
	 * USB specification, if firmware clears the STALL bit too late. Due to this hardware
	 * limitations, device controller polls to check if the stall bit has been transmitted for
	 * 3ms and then disables it after responding STALLed.
	 */
	while (idx < 198 && !(sys_read8(ctrl_base + UDC41_EPN_STATUS) & DC_STALL_SENT)) {
		/* wait 15.15us */
		gctrl_regs->GCTRL_WNCKR = 0;
		idx++;
	}

	if (idx < 198) {
		epn_ctrl_val = sys_read8(ctrl_base + UDC40_EPN_CTRL) & ~ENDPOINT_SEND_STALL;
		sys_write8(epn_ctrl_val, ctrl_base + UDC40_EPN_CTRL);
	}
	irq_unlock(lock_key);
}

static int it51xxx_ep_set_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);
	const struct it51xxx_config *config = dev->config;

	if (ep_idx == 0) {
#if 1 /* TODO: STILL NEED? */
		ctrl_ep_stall_workaround(dev);
#endif
	} else {
		mem_addr_t ctrl_base = config->base + epn_ctrl_base[ep_idx];
		uint8_t epn_ctrl_val = sys_read8(ctrl_base + UDC40_EPN_CTRL);

		epn_ctrl_val |= (ENDPOINT_SEND_STALL | ENDPOINT_READY);
		sys_write8(epn_ctrl_val, ctrl_base + UDC40_EPN_CTRL);
	}

	LOG_DBG("endpoint %#x is halted", cfg->addr);

	return 0;
}

static int it51xxx_ep_clear_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);
	const struct it51xxx_config *config = dev->config;
	mem_addr_t ctrl_base = config->base + epn_ctrl_base[ep_idx];
	uint8_t epn_ctrl_val = sys_read8(ctrl_base + UDC40_EPN_CTRL);

	epn_ctrl_val |= (ENDPOINT_SEND_STALL | ENDPOINT_READY);
	sys_write8(epn_ctrl_val, ctrl_base + UDC40_EPN_CTRL);

	LOG_DBG("endpoint %#x clear halted", cfg->addr);

	return 0;
}

static int it51xxx_ep_enable(const struct device *dev, struct udc_ep_config *const cfg)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);
	const struct it51xxx_config *config = dev->config;
	uint8_t epn_ctrl_val = sys_read8(config->base + epn_ctrl_base[ep_idx] + UDC40_EPN_CTRL);

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
			epn_ctrl_val &= ~ENDPOINT_ISO_ENABLE;
			break;
		case USB_EP_TYPE_ISO:
			epn_ctrl_val |= ENDPOINT_ISO_ENABLE;
			break;
		case USB_EP_TYPE_CONTROL:
			__fallthrough;
		default:
			return -ENOTSUP;
		}

		/* enable enqueue/dequeue operation fifo mode */
		if (ep_idx < 8) {
			sys_write8(sys_read8(config->base + UDC58_FIFO_OPERATION) | BIT(ep_idx),
				   config->base + UDC58_FIFO_OPERATION);
		} else {
			LOG_WRN("unknown fifo operation setting, ep_idx: %#x", ep_idx);
		}

		if (USB_EP_DIR_IS_IN(cfg->addr)) {
			epn_ctrl_val |= ENDPOINT_IN_DIRECTION;
			/* TODO: NEED TO CHECK */
			epn_ctrl_val &= ~ENDPOINT_OUTDATA_SEQ;
		} else {
			epn_ctrl_val &= ~ENDPOINT_IN_DIRECTION;
		}
	}

	epn_ctrl_val |= ENDPOINT_ENABLE;
	sys_write8(epn_ctrl_val, config->base + epn_ctrl_base[ep_idx] + UDC40_EPN_CTRL);
	LOG_DBG("endpoint %#x is enabled", cfg->addr);

	return 0;
}

static int it51xxx_ep_disable(const struct device *dev, struct udc_ep_config *const cfg)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);
	const struct it51xxx_config *config = dev->config;
	uint8_t epn_ctrl_val = sys_read8(config->base + epn_ctrl_base[ep_idx] + UDC40_EPN_CTRL);

	sys_write8(epn_ctrl_val & ~ENDPOINT_ENABLE,
		   config->base + epn_ctrl_base[ep_idx] + UDC40_EPN_CTRL);
	LOG_DBG("endpoint %#x is disabled", cfg->addr);

	return 0;
}

static int it51xxx_host_wakeup(const struct device *dev)
{
	LOG_ERR("%s ITE Debug %d", __func__, __LINE__);

	return 0;
}

static int it51xxx_set_address(const struct device *dev, const uint8_t addr)
{
	const struct it51xxx_config *config = dev->config;

	sys_write8(DEVICE_ADDRESS(addr), config->base + UDC54_TARGET_ADDR);
	LOG_DBG("set usb device address %#x", addr);

	return 0;
}

static int it51xxx_enable(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	uint8_t device_ctrl_val =
		GLOBAL_ENABLE | FULL_SPEED_LINE_POLARITY | FULL_SPEED_LINE_RATE | CONNECT_TO_HOST;

#if 0 /* TODO */
	struct it51xxx_data *priv = udc_get_private(dev);

	k_sem_init(&priv->suspended_sem, 0, 1);
	k_work_init_delayable(&priv->suspended_work, suspended_handler);
#endif

	sys_write8(device_ctrl_val, config->base + UDC50_TARGET_CTRL);

	/* enable usb d+ and usb interrupt */
#if 0 /* TODO */
	it82xx2_enable_wu_irq(dev, true);
#endif
	irq_enable(config->usb_irq);

	LOG_DBG("%s ITE Debug %d", __func__, __LINE__);

	return 0;
}

static int it51xxx_disable(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;
	uint8_t device_ctrl_val = sys_read8(config->base + UDC50_TARGET_CTRL);

	irq_disable(config->usb_irq);

	/* stop pull-up d+ and d- */
	sys_write8(device_ctrl_val & ~CONNECT_TO_HOST, config->base + UDC50_TARGET_CTRL);

	LOG_DBG("%s ITE Debug %d", __func__, __LINE__);

	return 0;
}

static int it51xxx_udc_ip_init(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;

#if 0 /* TODO: how to reset usb controller? */
	struct usb_it82xx2_regs *const usb_regs = config->base;

	usb_regs->host_device_control = RESET_CORE;
	k_msleep(1);
	usb_regs->port0_misc_control &= ~(PULL_DOWN_EN);
	usb_regs->port1_misc_control &= ~(PULL_DOWN_EN);

	/* clear reset bit */
	usb_regs->host_device_control = 0;
#endif

	sys_write8(sys_read8(config->base + UDC52_TARGET_INT_STATUS),
		   config->base + UDC52_TARGET_INT_STATUS);
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

#if 0 /* TODO: NEED? */
	struct gctrl_it51xxx_regs *const gctrl_regs = GCTRL_IT51XXX_REGS_BASE;
	/*
	 * Disable USB debug path , prevent CPU enter
	 * JTAG mode and then reset by USB command.
	 */
	gctrl_regs->GCTRL_MCCR &= ~(IT8XXX2_GCTRL_MCCR_USB_EN);
	gctrl_regs->gctrl_pmer2 |= IT8XXX2_GCTRL_PMER2_USB_PAD_EN;
#endif

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

	LOG_DBG("%s ITE Debug %d", __func__, __LINE__);

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

	LOG_DBG("%s ITE Debug %d", __func__, __LINE__);

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
	mem_addr_t ctrl_base = config->base + epn_ctrl_base[ep_idx];
	mem_addr_t tx_fifo_base = config->base + epn_tx_fifo_base[ep_idx];
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep);
	size_t len;

	sys_write8(sys_read8(tx_fifo_base + UDC74_EPN_TX_FIFO_CTRL) | FIFO_FORCE_EMPTY,
		   tx_fifo_base + UDC74_EPN_TX_FIFO_CTRL);

	len = MIN(buf->len, udc_mps_ep_size(ep_cfg));
	for (size_t i = 0; i < len; i++) {
		sys_write8(buf->data[i], tx_fifo_base + UDC70_EPN_TX_FIFO_DATA);
	}

	sys_write8(sys_read8(ctrl_base + UDC40_EPN_CTRL) | ENDPOINT_READY,
		   ctrl_base + UDC40_EPN_CTRL);
	LOG_DBG("written %d packets to endpoint %d tx fifo", len, ep);

	return 0;
}

static int it51xxx_xfer_out_data(const struct device *dev, uint8_t ep, struct net_buf *buf)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(ep);
	const struct it51xxx_config *config = dev->config;
	mem_addr_t rx_fifo_base = config->base + epn_rx_fifo_base[ep_idx];
	uint8_t epn_sts_val = sys_read8(config->base + epn_ctrl_base[ep_idx] + UDC41_EPN_STATUS);
	size_t len;

	if (epn_sts_val & RX_ERROR_MASK) {
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

	len = (sys_read8(rx_fifo_base + UDC62_EPN_RX_FIFO_COUNT_MSB) << 8) +
	      sys_read8(rx_fifo_base + UDC63_EPN_RX_FIFO_COUNT_LSB);

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
	const struct it51xxx_config *config = dev->config;
	mem_addr_t ctrl_base = config->base + epn_ctrl_base[USB_EP_GET_IDX(ep)];

	if (USB_EP_DIR_IS_OUT(ep)) {
		sys_write8(sys_read8(ctrl_base + UDC40_EPN_CTRL) | ENDPOINT_READY,
			   ctrl_base + UDC40_EPN_CTRL);
		return 0;
	}

	return it51xxx_xfer_in_data(dev, ep, buf);
}

static int it51xxx_ctrl_feed_dout(const struct device *dev, const size_t length)
{
	const struct it51xxx_config *config = dev->config;
	mem_addr_t ctrl_base = config->base + epn_ctrl_base[0];
	struct udc_ep_config *cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	struct net_buf *buf;

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, length);
	if (buf == NULL) {
		return -ENOMEM;
	}
	udc_buf_put(cfg, buf);

	sys_write8(sys_read8(ctrl_base + UDC40_EPN_CTRL) | ENDPOINT_READY,
		   ctrl_base + UDC40_EPN_CTRL);

	return 0;
}

static inline int work_handler_in(const struct device *dev, uint8_t ep)
{
	const struct it51xxx_config *config = dev->config;
	mem_addr_t ctrl_base = config->base + epn_ctrl_base[USB_EP_GET_IDX(ep)];
	struct udc_ep_config *ep_cfg;
	struct net_buf *buf;
	int ret = 0;

#if 0 /* TODO */
	if (it82xx2_fake_token(dev, ep, DC_IN_TRANS)) {
		return 0;
	}
#endif

	ep_cfg = udc_get_ep_cfg(dev, ep);

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		return -ENODATA;
	}

	net_buf_pull(buf, MIN(buf->len, udc_mps_ep_size(ep_cfg)));

	/* toggle data sequence */
	sys_write8(sys_read8(ctrl_base + UDC40_EPN_CTRL) ^ ENDPOINT_OUTDATA_SEQ,
		   ctrl_base + UDC40_EPN_CTRL);

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
		return -ENODATA;
	}

	udc_ep_set_busy(ep_cfg, false);

	if (ep == USB_CONTROL_EP_IN) {
		if (udc_ctrl_stage_is_status_in(dev) || udc_ctrl_stage_is_no_data(dev)) {
			/* status stage finished, notify upper layer */
			udc_ctrl_submit_status(dev, buf);
		}

		/* update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_status_out(dev)) {
			/*
			 * in transfer finished, release buffer,
			 * feed control out buffer for status stage.
			 */
			net_buf_unref(buf);
			ret = it51xxx_ctrl_feed_dout(dev, 0U);
		}
		return ret;
	}

	return udc_submit_ep_event(dev, buf, 0);
}

static inline int work_handler_setup(const struct device *dev, uint8_t ep)
{
	const struct it51xxx_config *config = dev->config;
	mem_addr_t ctrl_base = config->base + epn_ctrl_base[USB_EP_GET_IDX(ep)];
	struct it51xxx_data *priv = udc_get_private(dev);
	struct net_buf *buf;
	int ret = 0;

	if (udc_ctrl_stage_is_status_out(dev)) {
		struct udc_ep_config *cfg_out;

		/* out -> setup */
		cfg_out = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
		buf = udc_buf_get(cfg_out);
		if (buf) {
			udc_ep_set_busy(cfg_out, false);
			net_buf_unref(buf);
		}
	}

	if (udc_ctrl_stage_is_status_in(dev) || udc_ctrl_stage_is_no_data(dev)) {
		/* in -> setup */
		work_handler_in(dev, USB_CONTROL_EP_IN);
	}

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, sizeof(struct usb_setup_packet));
	if (buf == NULL) {
		LOG_ERR("failed to allocate buffer");
		return -ENOMEM;
	}

	udc_ep_buf_set_setup(buf);
	it51xxx_xfer_out_data(dev, ep, buf);
	if (buf->len != sizeof(struct usb_setup_packet)) {
		LOG_DBG("buffer length %d read from chip", buf->len);
		net_buf_unref(buf);
		return 0;
	}

	priv->stall_is_sent = false;
	LOG_HEXDUMP_DBG(buf->data, buf->len, "setup:");

	udc_ctrl_update_stage(dev, buf);

	/* set data sequence as 1 */
	sys_write8(sys_read8(ctrl_base + UDC40_EPN_CTRL) | ENDPOINT_OUTDATA_SEQ,
		   ctrl_base + UDC40_EPN_CTRL);

	if (udc_ctrl_stage_is_data_out(dev)) {
		/* allocate and feed buffer for data out stage */
		LOG_DBG("s:%p|feed for -out-", buf);
		ret = it51xxx_ctrl_feed_dout(dev, udc_data_stage_length(buf));
		if (ret == -ENOMEM) {
			ret = udc_submit_ep_event(dev, buf, ret);
		}
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		udc_ctrl_submit_s_in_status(dev);
	} else {
		udc_ctrl_submit_s_status(dev);
	}

	return ret;
}

static inline int work_handler_out(const struct device *dev, uint8_t ep)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(ep);
	const struct it51xxx_config *config = dev->config;
	mem_addr_t ctrl_base = config->base + epn_ctrl_base[ep_idx];
	mem_addr_t rx_fifo_base = config->base + epn_rx_fifo_base[ep_idx];
	int ret = 0;
	size_t len;
	struct net_buf *buf;
	struct udc_ep_config *ep_cfg;

#if 0 /* TODO */
	if (it82xx2_fake_token(dev, ep, IT51XXX_XFER_TYPE_OUT)) {
		return 0;
	}
#endif

	ep_cfg = udc_get_ep_cfg(dev, ep);
	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		return -ENODATA;
	}

	len = (sys_read8(rx_fifo_base + UDC62_EPN_RX_FIFO_COUNT_MSB) << 8) +
	      sys_read8(rx_fifo_base + UDC63_EPN_RX_FIFO_COUNT_LSB);

	if (ep == USB_CONTROL_EP_OUT) {
		if (udc_ctrl_stage_is_status_out(dev) && len != 0) {
			LOG_DBG("handle early setup token");
			buf = udc_buf_get(ep_cfg);
			/* notify upper layer */
			udc_ctrl_submit_status(dev, buf);
			/* update to next stage of control transfer */
			udc_ctrl_update_stage(dev, buf);
			return 0;
		}
	}

	if (len > udc_mps_ep_size(ep_cfg)) {
		LOG_ERR("failed to handle this packet due to the packet size");
		return -ENOBUFS;
	}

	it51xxx_xfer_out_data(dev, ep, buf);

	LOG_DBG("handle data out, %zu | %zu", len, net_buf_tailroom(buf));

	if (net_buf_tailroom(buf) && len == udc_mps_ep_size(ep_cfg)) {
		work_handler_xfer_continue(dev, ep, buf);
		if (ep != USB_CONTROL_EP_OUT) {
			ret = udc_submit_ep_event(dev, buf, 0);
		}
		return ret;
	}

	buf = udc_buf_get(ep_cfg);
	if (buf == NULL) {
		return -ENODATA;
	}

	udc_ep_set_busy(ep_cfg, false);

	if (ep == USB_CONTROL_EP_OUT) {
		if (udc_ctrl_stage_is_status_out(dev)) {
			/* status stage finished, notify upper layer */
			udc_ctrl_submit_status(dev, buf);
		}

		/* update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_status_in(dev)) {
			/* set data sequence as 1 */
			sys_write8(sys_read8(ctrl_base + UDC40_EPN_CTRL) | ENDPOINT_OUTDATA_SEQ,
				   ctrl_base + UDC40_EPN_CTRL);
			ret = udc_ctrl_submit_s_out_status(dev, buf);
		}
	} else {
		ret = udc_submit_ep_event(dev, buf, 0);
	}

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

static void it51xxx_udc_xfer_done(const struct device *dev)
{
	const struct it51xxx_config *config = dev->config;

	for (uint8_t ep_idx = 0; ep_idx < config->num_of_eps; ep_idx++) {
		mem_addr_t ctrl_base = config->base + epn_ctrl_base[ep_idx];
		uint8_t epn_ctrl_val = sys_read8(ctrl_base + UDC40_EPN_CTRL);
		uint8_t xfer_type =
			TRANSACTION_TYPE(sys_read8(ctrl_base + UDC42_EPN_TRANSACTION_TYPE));

		/* the enable bit is set and the ready bit is cleared if the
		 * transaction is completed.
		 */
		if (!(epn_ctrl_val & ENDPOINT_ENABLE) || epn_ctrl_val & ENDPOINT_READY) {
			continue;
		}

#if 0 /* TODO: NEED? */
		if (ep_idx != 0) {
			if (it82xx2_fake_token(dev, ep_idx, transtype)) {
				continue;
			}
		}
#endif

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

static void it51xxx_udc_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct it51xxx_config *config = dev->config;
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

#if 0 /* TODO */
	struct it51xxx_data *priv = udc_get_private(dev);

	if (int_sts & SOF_RECEIVED) {
		if (!IS_ENABLED(CONFIG_UDC_ENABLE_SOF)) {
			it82xx2_enable_sof_int(dev, false);
		} else {
			sys_write8(SOF_RECEIVED, config->base + UDC52_TARGET_INT_STATUS);
			udc_submit_sof_event(dev);
		}
		it82xx2_enable_resume_int(dev, false);
		emit_resume_event(dev);
		k_work_cancel_delayable(&priv->suspended_work);
		k_work_reschedule(&priv->suspended_work, K_MSEC(5));
	}

	if (int_sts & RESUME_EVENT) {
		it82xx2_enable_resume_int(dev, false);
		emit_resume_event(dev);
	}
#else

	if (int_sts & SOF_RECEIVED) {
		sys_write8(SOF_RECEIVED, config->base + UDC52_TARGET_INT_STATUS);
	}
	if (int_sts & RESUME_EVENT) {
		sys_write8(RESUME_EVENT, config->base + UDC52_TARGET_INT_STATUS);
	}
#endif

	if (int_sts & TRANS_DONE) {
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

	k_mutex_init(&data->mutex);

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("failed to apply pinctrl, ret %d", ret);
		return ret;
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.mps = USB_CONTROL_EP_MPS;
		} else {
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.iso = 1;
			config->ep_cfg_out[i].caps.mps = 64; /* TODO: CHECKME */
		}

		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		ret = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (ret) {
			LOG_ERR("failed to register out endpoint");
			return ret;
		}
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			config->ep_cfg_in[i].caps.control = 1;
			config->ep_cfg_in[i].caps.mps = USB_CONTROL_EP_MPS;
		} else {
			config->ep_cfg_in[i].caps.bulk = 1;
			config->ep_cfg_in[i].caps.interrupt = 1;
			config->ep_cfg_in[i].caps.iso = 1;
			config->ep_cfg_in[i].caps.mps = 64; /* TODO: CHECKME */
		}

		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		ret = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (ret != 0) {
			LOG_ERR("failed to register in endpoint");
			return ret;
		}
	}

	data->caps.rwup = true;
	data->caps.mps0 = UDC_MPS0_64;

	priv->dev = dev;

	config->make_thread(dev);

	/* Connect USB interrupt */
	irq_connect_dynamic(config->usb_irq, 0, it51xxx_udc_isr, dev, 0);

	/* TODO */
	pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	return 0;
}

#define IT51XXX_USB_DEVICE_DEFINE(n)                                                               \
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
	static struct udc_ep_config ep_cfg_out[DT_INST_PROP(n, num_bidir_endpoints)];              \
	static struct udc_ep_config ep_cfg_in[DT_INST_PROP(n, num_bidir_endpoints)];               \
                                                                                                   \
	static struct it51xxx_config udc_cfg_##n = {                                               \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.usb_irq = DT_INST_IRQ_BY_IDX(n, 0, irq),                                          \
		.ep_cfg_in = ep_cfg_out,                                                           \
		.ep_cfg_out = ep_cfg_in,                                                           \
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),                                \
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
