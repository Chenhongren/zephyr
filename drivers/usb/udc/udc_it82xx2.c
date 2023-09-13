/*
 * Copyright (c) 2023 ITE Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "udc_common.h"

#include <soc_dt.h>
#include <zephyr/pm/policy.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/interrupt_controller/wuc_ite_it8xxx2.h>
#include <zephyr/dt-bindings/interrupt-controller/it8xxx2-wuc.h>
LOG_MODULE_REGISTER(udc_it82xx2, LOG_LEVEL_INF);

#define DT_DRV_COMPAT ite_it82xx2_usb

#define FIFO_NUM			3

/* The related definitions of the register dc_line_status: 0x51 */
#define RX_LINE_STATE_MASK		(RX_LINE_FULL_SPD | RX_LINE_LOW_SPD)
#define RX_LINE_LOW_SPD			0x02
#define RX_LINE_FULL_SPD		0x01
#define RX_LINE_RESET			0x00

#define DC_ADDR_NULL			0x00
#define DC_ADDR_MASK			0x7F

/* The related definitions of the register EP STATUS:
 * 0x41/0x45/0x49/0x4D
 */
#define EP_STATUS_ERROR			0x0F

/* EP Definitions */
// #define EP_VALID_MASK			0x0F
// #define EP_INVALID_MASK			~(USB_EP_DIR_MASK | EP_VALID_MASK)

/* The bit definitions of the register EP RX/TX FIFO Control:
 * EP_RX_FIFO_CONTROL: 0X64/0x84/0xA4/0xC4
 * EP_TX_FIFO_CONTROL: 0X74/0x94/0xB4/0xD4
 */
#define FIFO_FORCE_EMPTY		BIT(0)

/* The bit definitions of the register Host/Device Control: 0XE0 */
#define RESET_CORE			BIT(1)

/* Bit definitions of the register Port0/Port1 MISC Control: 0XE4/0xE8 */
#define PULL_DOWN_EN			BIT(4)

/* Bit definitions of the register EPN0N1_EXTEND_CONTROL_REG: 0X98 ~ 0X9D */
#define EPN1_OUTDATA_SEQ		BIT(4)
#define EPN0_ISO_ENABLE			BIT(2)
#define EPN0_SEND_STALL			BIT(1)
#define EPN0_OUTDATA_SEQ		BIT(0)

/* ENDPOINT[3..0]_STATUS_REG */
#define DC_STALL_SENT			BIT(5)

/* DC_INTERRUPT_STATUS_REG */
#define DC_TRANS_DONE			BIT(0)
#define DC_RESUME_INT			BIT(1)
#define DC_RESET_EVENT			BIT(2)
#define DC_SOF_RECEIVED			BIT(3)
#define DC_NAK_SENT_INT			BIT(4)

/* DC_CONTROL_REG */
#define DC_GLOBAL_ENABLE		BIT(0)
#define DC_TX_LINE_STATE_DM		BIT(1)
#define DC_DIRECT_CONTROL		BIT(3)
#define DC_FULL_SPEED_LINE_POLARITY	BIT(4)
#define DC_FULL_SPEED_LINE_RATE		BIT(5)
#define DC_CONNECT_TO_HOST		BIT(6) /* internal pull-up */

/* ENDPOINT[3..0]_CONTROL_REG */
#define ENDPOINT_EN			BIT(0)
#define ENDPOINT_RDY			BIT(1)
#define EP_OUTDATA_SEQ			BIT(2)
#define EP_SEND_STALL			BIT(3)
#define EP_ISO_ENABLE			BIT(4)
#define EP_DIRECTION			BIT(5)

/* The ep_fifo_res[ep_idx % FIFO_NUM] where the FIFO_NUM is 3 represents the
 * EP mapping because when (ep_idx % FIFO_NUM) is 3, it actually means the EP0.
 */
static const uint8_t ep_fifo_res[3] = {3, 1, 2};

/* Bit [1:0] represents the TRANSACTION_TYPE as follows: */
enum it82xx2_transaction_types {
	DC_SETUP_TRANS,
	DC_IN_TRANS,
	DC_OUTDATA_TRANS,
	DC_ALL_TRANS
};

enum it82xx2_event_type {
	IT82xx2_EVT_XFER,
	IT82xx2_EVT_SETUP_TOKEN,
	IT82xx2_EVT_OUT_TOKEN,
	IT82xx2_EVT_IN_TOKEN,
};

struct it82xx2_ep_event {
	sys_snode_t node;
	const struct device *dev;
	uint8_t ep;
	enum it82xx2_event_type event;
};

K_MEM_SLAB_DEFINE(udc_it82xx2_slab, sizeof(struct it82xx2_ep_event),
		  16, sizeof(void *));


// struct it82xx2_endpoint_data {

// };

struct usb_it8xxx2_wuc {
	/* WUC control device structure */
	const struct device *wucs;
	/* WUC pin mask */
	uint8_t mask;
};

struct it82xx2_priv_data {
	// struct k_work work;
	const struct device *dev;

	struct k_fifo fifo;
	struct k_work_delayable suspended_work;

	struct k_thread thread_data;
	// struct it82xx2_endpoint_data ep_data[16];
	struct k_sem suspended_sem;

	bool suspended;
};

struct usb_it82xx2_config {
	struct usb_it82xx2_regs *const base;
	const struct pinctrl_dev_config *pcfg;
	const struct usb_it8xxx2_wuc *wuc_list;
	uint8_t usb_irq;
	uint8_t wu_irq;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	void (*make_thread)(const struct device *dev);
};

/* Standby(deep doze) mode enable/disable */
static void it82xx2_enable_standby_state(bool enable)
{
	if (enable) {
		pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	} else {
		pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	}
}

/* Wake-up interrupt (USB D+) Enable/Disable */
static void it82xx2_enable_wu_irq(const struct device *dev, bool enable)
{
	const struct usb_it82xx2_config *config = dev->config;

	/* Clear pending interrupt */
	it8xxx2_wuc_clear_status(config->wuc_list[0].wucs, config->wuc_list[0].mask);

	if (enable) {
		irq_enable(config->wu_irq);
	} else {
		irq_disable(config->wu_irq);
	}
}

static void it82xx2_wu_isr(const void *arg)
{
	const struct device *dev = arg;

	it82xx2_enable_wu_irq(dev, false);
	it82xx2_enable_standby_state(false);
	LOG_DBG("USB D+ (WU) Triggered");
}

static void it8xxx2_usb_dc_wuc_init(const struct device *dev)
{
	const struct usb_it82xx2_config *config = dev->config;

	/* Initializing the WUI */
	it8xxx2_wuc_set_polarity(config->wuc_list[0].wucs,
				config->wuc_list[0].mask,
				WUC_TYPE_EDGE_FALLING);
	it8xxx2_wuc_clear_status(config->wuc_list[0].wucs,
				config->wuc_list[0].mask);

	/* Enabling the WUI */
	it8xxx2_wuc_enable(config->wuc_list[0].wucs, config->wuc_list[0].mask);

	/* Connect WU (USB D+) interrupt but make it disabled initially */
	irq_connect_dynamic(config->wu_irq, 0, it82xx2_wu_isr, dev, 0);
}

static void it82xx2_event_submit(const struct device *dev,
				 const uint8_t ep,
				 const enum it82xx2_event_type event)
{
	struct it82xx2_priv_data *priv = udc_get_private(dev);
	struct it82xx2_ep_event *evt;
	int ret = k_mem_slab_alloc(&udc_it82xx2_slab, (void **)&evt, K_NO_WAIT);

	if (ret) {
		// LOG_ERR("[%s] ITE Debug [%d] - ERROR EVT", __func__, __LINE__);
		udc_submit_event(dev, UDC_EVT_ERROR, ret);
		// ????????
		return;
	}

	evt->dev = dev;
	evt->ep = ep;
	evt->event = event;
	k_fifo_put(&priv->fifo, evt);
}

static int it82xx2_ep_enqueue(const struct device *dev,
			       struct udc_ep_config *const cfg,
			       struct net_buf *const buf)
{
	udc_buf_put(cfg, buf);

	it82xx2_event_submit(dev, cfg->addr, IT82xx2_EVT_XFER);
	return 0;
}

static int it82xx2_ep_dequeue(const struct device *dev,
			       struct udc_ep_config *const cfg)
{
	struct net_buf *buf;

	buf = udc_buf_get_all(dev, cfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	udc_ep_set_busy(dev, cfg->addr, false);

	return 0;
}

static int it82xx2_ep_set_halt(const struct device *dev,
				struct udc_ep_config *const cfg)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	struct it82xx2_usb_ep_regs *ep_regs = usb_regs->usb_ep_regs;
	struct gctrl_it8xxx2_regs *const gctrl_regs = GCTRL_IT8XXX2_REGS_BASE;
	const uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);

	if (ep_idx < EP4) {
		ep_regs[ep_idx].ep_ctrl |= EP_SEND_STALL;
	} else {
		LOG_ERR("[%s]ITE Debug[%d]-TODO", __func__, __LINE__);
		return -ENOTSUP;
	}

	if (ep_idx == EP0) {
		ep_regs[EP0].ep_ctrl |= ENDPOINT_RDY;
		uint32_t idx = 0;
		/* polling if stall send for 3ms */
		while (idx < 198 &&
			!(ep_regs[EP0].ep_status & DC_STALL_SENT)) {
			/* wait 15.15us */
			gctrl_regs->GCTRL_WNCKR = 0;
			idx++;
		}

		if (idx < 198) {
			ep_regs[EP0].ep_ctrl &= ~EP_SEND_STALL;
		}
	}

	LOG_DBG("Endpoint%d is halted", ep_idx);

	return 0;
}

static int it82xx2_ep_clear_halt(const struct device *dev,
				  struct udc_ep_config *const cfg)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	struct it82xx2_usb_ep_regs *ep_regs = usb_regs->usb_ep_regs;
	const uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);

	if (ep_idx < EP4) {
		ep_regs[ep_idx].ep_ctrl &= ~EP_SEND_STALL;
	} else {
		LOG_ERR("[%s]ITE Debug[%d]-TODO", __func__, __LINE__);
		return -ENOTSUP;
	}

	LOG_DBG("Endpoint%d clear halted", ep_idx);

	return 0;
}

/* Function it82xx2_get_ep_fifo_ctrl_reg_idx(uint8_t ep_idx):
 *
 * Calculate the register offset index which determines the corresponding
 * EP FIFO Ctrl Registers which is defined as ep_fifo_ctrl[reg_idx] here
 *
 * The ep_fifo_res[ep_idx % FIFO_NUM] represents the EP mapping because when
 * (ep_idx % FIFO_NUM) is 3, it actually means the EP0.
 */
static uint8_t it82xx2_get_ep_fifo_ctrl_reg_idx(uint8_t ep_idx)
{

	uint8_t reg_idx = (ep_idx < EP8) ?
		((ep_fifo_res[ep_idx % FIFO_NUM] - 1) * 2) :
		((ep_fifo_res[ep_idx % FIFO_NUM] - 1) * 2 + 1);

	return reg_idx;
}

/* Function it82xx2_get_ep_fifo_ctrl_reg_val(uint8_t ep_idx):
 *
 * Calculate the register value written to the ep_fifo_ctrl which is defined as
 * ep_fifo_ctrl[reg_idx] here for selecting the corresponding control bit.
 */
static uint8_t it82xx2_get_ep_fifo_ctrl_reg_val(uint8_t ep_idx)
{
	uint8_t reg_val = (ep_idx < EP8) ?
		(1 << ep_idx) : (1 << (ep_idx - EP8));

	return reg_val;
}

enum it82xx2_extend_ep_ctrl {
	/* EPN0N1_EXTEND_CONTROL_REG */
	EXT_EP_ISO_DISABLE,
	EXT_EP_ISO_ENABLE,
	EXT_EP_SEND_STALL,
	EXT_EP_CLEAR_STALL,
	EXT_EP_CHECK_STALL,
	EXT_EP_DATA_SEQ_1,
	EXT_EP_DATA_SEQ_0,
	EXT_EP_DATA_SEQ_INV,
	/* EPN_EXTEND_CONTROL1_REG */
	EXT_EP_DIR_IN,
	EXT_EP_DIR_OUT,
	EXT_EP_ENABLE,
	EXT_EP_DISABLE,
	/* EPN_EXTEND_CONTROL2_REG */
	EXT_EP_READY,
};

/* Mapped to the bit definitions in the EPN_EXTEND_CONTROL1 Register
 * (D6h to DDh) for configuring the FIFO direction and for enabling/disabling
 * the endpoints.
 */
static uint8_t ext_ep_bit_shift[12] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3};

/* Mapping the enum it82xx2_extend_ep_ctrl code to their corresponding bit in
 * the EP45/67/89/1011/1213/1415 Extended Control Registers.
 */
static const uint8_t ext_ctrl_tbl[7] = {
	EPN0_ISO_ENABLE,
	EPN0_ISO_ENABLE,
	EPN0_SEND_STALL,
	EPN0_SEND_STALL,
	EPN0_SEND_STALL,
	EPN0_OUTDATA_SEQ,
	EPN0_OUTDATA_SEQ,
};

/* Indexing of the following control codes:
 * EXT_EP_DIR_IN, EXT_EP_DIR_OUT, EXT_EP_ENABLE, EXT_EP_DISABLE
 */
static const uint8_t epn_ext_ctrl_tbl[4] = {1, 0, 1, 0};

/*
 * Functions it82xx2_epn0n1_ext_ctrl_cfg() and epn0n1_ext_ctrl_cfg_seq_inv()
 * provide the entrance of configuring the EPN0N1 Extended Ctrl Registers.
 *
 * The variable set_clr determines if we set/clear the corresponding bit.
 */
static void it82xx2_epn0n1_ext_ctrl_cfg(const struct device *dev, uint8_t reg_idx, uint8_t bit_mask,
			bool set_clr)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	volatile uint8_t *epn0n1_ext_ctrl =
		usb_regs->fifo_regs[EP_EXT_REGS_9X].ext_4_15.epn0n1_ext_ctrl;

	(set_clr) ? (epn0n1_ext_ctrl[reg_idx] |= bit_mask) :
		(epn0n1_ext_ctrl[reg_idx] &= ~(bit_mask));
}

static void it82xx2_epn0n1_ext_ctrl_cfg_seq_inv(const struct device *dev, uint8_t reg_idx,
			uint8_t bit_mask, bool set_clr)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	volatile uint8_t *epn0n1_ext_ctrl =
		usb_regs->fifo_regs[EP_EXT_REGS_9X].ext_4_15.epn0n1_ext_ctrl;

	bool check = (set_clr) ?
		(epn0n1_ext_ctrl[reg_idx] & EPN0_OUTDATA_SEQ) :
		(epn0n1_ext_ctrl[reg_idx] & EPN1_OUTDATA_SEQ);

	(check) ? (epn0n1_ext_ctrl[reg_idx] &= ~(bit_mask)) :
		(epn0n1_ext_ctrl[reg_idx] |= bit_mask);
}

/* Return the status of STALL bit in the EPN0N1 Extend Control Registers */
static bool it82xx2_epn01n1_check_stall(const struct device *dev, uint8_t reg_idx, uint8_t bit_mask)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	volatile uint8_t *epn0n1_ext_ctrl =
		usb_regs->fifo_regs[EP_EXT_REGS_9X].ext_4_15.epn0n1_ext_ctrl;

	return !!(epn0n1_ext_ctrl[reg_idx] & bit_mask);
}

/* Configuring the EPN Extended Ctrl Registers. */
static void it82xx2_epn_ext_ctrl_cfg1(const struct device *dev, uint8_t reg_idx, uint8_t bit_mask,
			bool set_clr)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	struct epn_ext_ctrl_regs *epn_ext_ctrl =
		usb_regs->fifo_regs[EP_EXT_REGS_DX].ext_0_3.epn_ext_ctrl;

	(set_clr) ? (epn_ext_ctrl[reg_idx].epn_ext_ctrl1 |= bit_mask) :
		(epn_ext_ctrl[reg_idx].epn_ext_ctrl1 &= ~(bit_mask));
}

static void it82xx2_epn_ext_ctrl_cfg2(const struct device *dev, uint8_t reg_idx, uint8_t bit_mask,
			bool set_clr)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	struct epn_ext_ctrl_regs *epn_ext_ctrl =
		usb_regs->fifo_regs[EP_EXT_REGS_DX].ext_0_3.epn_ext_ctrl;

	(set_clr) ? (epn_ext_ctrl[reg_idx].epn_ext_ctrl2 |= bit_mask) :
		(epn_ext_ctrl[reg_idx].epn_ext_ctrl2 &= ~(bit_mask));
}

/* From 98h to 9Dh, the EP45/67/89/1011/1213/1415 Extended Control Registers
 * are defined, and their bits definitions are as follows:
 *
 * Bit    Description
 *  7     Reserved
 *  6     EPPOINT5_ISO_ENABLE
 *  5     EPPOINT5_SEND_STALL
 *  4     EPPOINT5_OUT_DATA_SEQUENCE
 *  3     Reserved
 *  2     EPPOINT4_ISO_ENABLE
 *  1     EPPOINT4_SEND_STALL
 *  0     EPPOINT4_OUT_DATA_SEQUENCE
 *
 * Apparently, we can tell that the EP4 and EP5 share the same register, and
 * the EP6 and EP7 share the same one, and the other EPs are defined in the
 * same way.
 *
 * In the function it82xx2_usb_extend_ep_ctrl() we will obtain the mask/flag
 * according to the bits definitions mentioned above. As for the control code,
 * please refer to the definition of enum it82xx2_extend_ep_ctrl.
 */
static int it82xx2_usb_extend_ep_ctrl(const struct device *dev, uint8_t ep_idx,
			enum it82xx2_extend_ep_ctrl ctrl)
{
	uint8_t reg_idx, mask;
	bool flag;

	if (ep_idx < EP4) {
		return -EINVAL;
	}

	if ((ctrl >= EXT_EP_DIR_IN) && (ctrl < EXT_EP_READY)) {
		/* From EXT_EP_DIR_IN to EXT_EP_DISABLE */
		reg_idx = ep_fifo_res[ep_idx % FIFO_NUM];
		mask = 1 << (ext_ep_bit_shift[ep_idx - 4] * 2 + 1);
		flag = epn_ext_ctrl_tbl[ctrl - EXT_EP_DIR_IN];
		it82xx2_epn_ext_ctrl_cfg1(dev, reg_idx, mask, flag);

	} else if ((ctrl >= EXT_EP_ISO_DISABLE) && (ctrl < EXT_EP_DIR_IN)) {
		/* From EXT_EP_ISO_DISABLE to EXT_EP_DATA_SEQ_0 */
		reg_idx = (ep_idx - 4) >> 1;
		flag = !!(ep_idx & 1);
		mask = flag ? (ext_ctrl_tbl[ctrl] << 4) : (ext_ctrl_tbl[ctrl]);

		if (ctrl == EXT_EP_CHECK_STALL) {
			return it82xx2_epn01n1_check_stall(dev, reg_idx, mask);
		} else if (ctrl == EXT_EP_DATA_SEQ_INV) {
			it82xx2_epn0n1_ext_ctrl_cfg_seq_inv(
				dev, reg_idx, mask, flag);
		} else {
			it82xx2_epn0n1_ext_ctrl_cfg(dev, reg_idx, mask, flag);
		}
	} else if (ctrl == EXT_EP_READY) {
		reg_idx = (ep_idx - 4) >> 1;
		mask = 1 << (ext_ep_bit_shift[ep_idx - 4]);
		it82xx2_epn_ext_ctrl_cfg2(dev, reg_idx, mask, true);
	} else {
		LOG_ERR("Invalid Control Code of Endpoint");
		return -EINVAL;
	}

	return 0;
}

static int it82xx2_ep_enable(const struct device *dev,
			      struct udc_ep_config *const cfg)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	struct it82xx2_usb_ep_regs *ep_regs = usb_regs->usb_ep_regs;
	const uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);

	/* Configure endpoint */
	if (ep_idx != EP0) {
		volatile uint8_t *ep_fifo_ctrl =
			usb_regs->fifo_regs[EP_EXT_REGS_BX].fifo_ctrl.ep_fifo_ctrl;
		uint8_t reg_idx = it82xx2_get_ep_fifo_ctrl_reg_idx(ep_idx);
		uint8_t reg_val = it82xx2_get_ep_fifo_ctrl_reg_val(ep_idx);

		if (USB_EP_DIR_IS_IN(cfg->addr)) {
			if (ep_idx < 4) {
				ep_regs[ep_idx].ep_ctrl |= EP_DIRECTION;
			} else {
				it82xx2_usb_extend_ep_ctrl(dev, ep_idx, EXT_EP_DIR_OUT);
			}
		} else {
			if (ep_idx < 4) {
				ep_regs[ep_idx].ep_ctrl &= ~EP_DIRECTION;
			} else {
				it82xx2_usb_extend_ep_ctrl(dev, ep_idx, EXT_EP_DIR_OUT);
			}
		}

		ep_fifo_ctrl[reg_idx] |= reg_val;

		switch (cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) {
		case USB_EP_TYPE_BULK:
		case USB_EP_TYPE_INTERRUPT:
			if (ep_idx < EP4) {
				ep_regs[ep_idx].ep_ctrl &= ~EP_ISO_ENABLE;
			} else {
				it82xx2_usb_extend_ep_ctrl(dev, ep_idx, EXT_EP_ISO_DISABLE);
			}
			break;
		case USB_EP_TYPE_ISO:
			if (ep_idx < EP4) {
				ep_regs[ep_idx].ep_ctrl |= EP_ISO_ENABLE;
			} else {
				it82xx2_usb_extend_ep_ctrl(dev, ep_idx, EXT_EP_ISO_ENABLE);
			}
			break;
		case USB_EP_TYPE_CONTROL:
		default:
			return -ENOTSUP;
		}
	}

	/* Enable endpoint */
	if (ep_idx < EP4) {
		ep_regs[ep_idx].ep_ctrl |= ENDPOINT_EN;
	} else {
		it82xx2_usb_extend_ep_ctrl(dev, ep_idx, EXT_EP_ENABLE);
	}

	LOG_ERR("Endpoint 0x%02x is enabled", cfg->addr);

	return 0;
}

static int it82xx2_ep_disable(const struct device *dev,
			       struct udc_ep_config *const cfg)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	struct it82xx2_usb_ep_regs *ep_regs = usb_regs->usb_ep_regs;
	const uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);

	if (ep_idx < EP4) {
		ep_regs[ep_idx].ep_ctrl &= ~ENDPOINT_EN;
	} else {
		LOG_ERR("[%s]ITE Debug[%d]-TODO", __func__, __LINE__);
		return -ENOTSUP;
	}

	LOG_DBG("Endpoint 0x%02x is disabled", cfg->addr);

	return 0;
}

static int it82xx2_host_wakeup(const struct device *dev)
{
	struct it82xx2_priv_data *priv = udc_get_private(dev);
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	int ret;

	if (priv->suspended) {
		usb_regs->dc_control =
			DC_GLOBAL_ENABLE | DC_FULL_SPEED_LINE_POLARITY |
			DC_FULL_SPEED_LINE_RATE | DC_DIRECT_CONTROL |
			DC_TX_LINE_STATE_DM | DC_CONNECT_TO_HOST;

		/* The remote wakeup device must hold the resume signal for */
		/* at least 1 ms but for no more than 15 ms                 */
		k_msleep(2);

		usb_regs->dc_control =
			DC_GLOBAL_ENABLE | DC_FULL_SPEED_LINE_POLARITY |
			DC_FULL_SPEED_LINE_RATE | DC_CONNECT_TO_HOST;

		ret = k_sem_take(&priv->suspended_sem, K_MSEC(500));
		if (ret < 0) {
			LOG_ERR("Failed to wake up host");
		}
	}

	return 0;
}

static int it82xx2_set_address(const struct device *dev, const uint8_t addr)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;

	usb_regs->dc_address = addr & DC_ADDR_MASK;

	LOG_DBG("Set usb address 0x%02x", addr);

	return 0;
}

static int it82xx2_usb_dc_ip_init(const struct device *dev)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;

	/* reset usb controller */
	usb_regs->host_device_control = RESET_CORE;
	k_msleep(1);
	usb_regs->port0_misc_control &= ~(PULL_DOWN_EN);
	usb_regs->port1_misc_control &= ~(PULL_DOWN_EN);

	/* clear reset bit */
	usb_regs->host_device_control = 0;

	usb_regs->dc_interrupt_status =
		DC_TRANS_DONE | DC_RESET_EVENT | DC_SOF_RECEIVED;

	usb_regs->dc_interrupt_mask = 0x00;
	usb_regs->dc_interrupt_mask =
		DC_TRANS_DONE | DC_RESET_EVENT | DC_SOF_RECEIVED;

	usb_regs->dc_address = DC_ADDR_NULL;

	return 0;
}

static void it82xx2_enable_sof_int(const struct device *dev, bool enable)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;

	usb_regs->dc_interrupt_status = DC_SOF_RECEIVED;
	if (enable) {
		usb_regs->dc_interrupt_mask |= DC_SOF_RECEIVED;
	} else {
		usb_regs->dc_interrupt_mask &= ~DC_SOF_RECEIVED;
	}
}

void it82xx2_dc_reset(const struct device *dev)
{
	const struct usb_it82xx2_config *config = dev->config;
	const struct it82xx2_priv_data *priv = udc_get_private(dev);
	struct usb_it82xx2_regs *const usb_regs = config->base;
	struct it82xx2_usb_ep_regs *ep_regs = usb_regs->usb_ep_regs;
	struct it82xx2_usb_ep_fifo_regs *ff_regs = usb_regs->fifo_regs;

	for (uint8_t ep_idx = EP0 ; ep_idx < EP4 ; ep_idx++) {
		ff_regs[ep_idx].ep_rx_fifo_ctrl = FIFO_FORCE_EMPTY;
		ff_regs[ep_idx].ep_tx_fifo_ctrl = FIFO_FORCE_EMPTY;
	}

	ep_regs[EP0].ep_ctrl = ENDPOINT_EN;
	usb_regs->dc_address = DC_ADDR_NULL;
	usb_regs->dc_interrupt_status = DC_NAK_SENT_INT | DC_SOF_RECEIVED;
}

static int it82xx2_xfer_in_data(const struct device *dev, uint8_t ep, struct net_buf *buf)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	struct it82xx2_usb_ep_regs *ep_regs = usb_regs->usb_ep_regs;
	struct it82xx2_usb_ep_fifo_regs *ff_regs = usb_regs->fifo_regs;
	// volatile uint8_t *ep_fifo_ctrl =
	// 	usb_regs->fifo_regs[EP_EXT_REGS_BX].fifo_ctrl.ep_fifo_ctrl;
	const uint8_t ep_idx = USB_EP_GET_IDX(ep);
	uint8_t ep_fifo = (ep_idx > EP0) ? (ep_fifo_res[ep_idx % FIFO_NUM]) : 0;

	if (ep_idx >= EP4) {
		LOG_ERR("[%s]ITE Debug[%d]-TODO", __func__, __LINE__);
		return -ENOTSUP;
	}

	/* clear fifo before write*/
	if (ep_idx == EP0) {
		ff_regs[ep_idx].ep_tx_fifo_ctrl = FIFO_FORCE_EMPTY;
	}

	uint16_t len = MIN(buf->len, config->ep_cfg_in[USB_EP_GET_IDX(USB_CONTROL_EP_IN)].caps.mps);

	for (uint16_t i = 0 ; i < len ; i++) {
		ff_regs[ep_fifo].ep_tx_fifo_data = buf->data[i];
	}

	ep_regs[ep_fifo].ep_ctrl |= ENDPOINT_RDY;

	LOG_DBG("Writed %d packets to endpoint%d tx fifo", buf->len, ep_idx);

	return 0;
}

static int it82xx2_xfer_out_data(const struct device *dev, uint8_t ep, struct net_buf *buf)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	struct it82xx2_usb_ep_regs *ep_regs = usb_regs->usb_ep_regs;
	struct it82xx2_usb_ep_fifo_regs *ff_regs = usb_regs->fifo_regs;
	const uint8_t ep_idx = USB_EP_GET_IDX(ep);
	uint8_t ep_fifo = (ep_idx > EP0) ? (ep_fifo_res[ep_idx % FIFO_NUM]) : 0;

	if (ep_idx >= EP4) {
		LOG_ERR("[%s]ITE Debug[%d]-TODO", __func__, __LINE__);
		return -ENOTSUP;
	}

	if (ep_regs[ep].ep_status & EP_STATUS_ERROR) {
		LOG_WRN("EP Error Flag: 0x%02x", ep_regs[ep].ep_status);
		LOG_ERR("[%s]ITE Debug[%d]-ERR", __func__, __LINE__);
		return -1;
	}

	buf->size = (uint16_t)ff_regs[ep_fifo].ep_rx_fifo_dcnt_lsb +
		(((uint16_t)ff_regs[ep_fifo].ep_rx_fifo_dcnt_msb) << 8);

	for (int idx = 0 ; idx < buf->size ; idx++) {
		buf->data[idx] = ff_regs[ep_fifo].ep_rx_fifo_data;
	}

	return 0;
}

static int work_handler_xfer_continue(const struct device *dev, uint8_t ep, struct net_buf *buf) {

	// if (udc_ep_is_busy(dev, ep)) {
	// 	LOG_ERR("endpoint0x%02x is busy", ep);
	// 	return -EBUSY;
	// }

	// udc_ep_set_busy(dev, ep, true);

	if (USB_EP_DIR_IS_OUT(ep)) {
		it82xx2_xfer_out_data(dev, ep, buf);
	} else {
		it82xx2_xfer_in_data(dev, ep, buf);
	}

	return 0;
}

static int work_handler_xfer_next(const struct device *dev, uint8_t ep)
{
	struct net_buf *buf;

	buf = udc_buf_peek(dev, ep);
	if (!buf) {
		return -ENODATA;
	}

	return work_handler_xfer_continue(dev, ep, buf);
}

/*
 * Allocate buffer and initiate a new control OUT transfer,
 * use successive buffer descriptor when next is true.
 */
static int it82xx2_ctrl_feed_dout(const struct device *dev,
				   const size_t length)
{
	struct net_buf *buf;
	int ret = 0;
	struct it82xx2_priv_data *priv = udc_get_private(dev);
	struct udc_ep_config *cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, length);
	if (buf == NULL) {
		return -ENOMEM;
	}

	udc_buf_put(cfg, buf);

	return 0;
}

static int work_handler_setup(const struct device *dev, uint8_t ep)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	struct it82xx2_usb_ep_regs *ep_regs = usb_regs->usb_ep_regs;
	struct it82xx2_usb_ep_fifo_regs *ff_regs = usb_regs->fifo_regs;
	struct net_buf *buf;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	int err;

	if (ep_regs[ep_idx].ep_ctrl & EP_SEND_STALL) {
		ep_regs[ep_idx].ep_ctrl &= ~EP_SEND_STALL;
		ff_regs[ep_idx].ep_rx_fifo_ctrl = FIFO_FORCE_EMPTY;
		LOG_DBG("Cleared stall bit and rx fifo");
		return -EINVAL;
	}

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT,
			     sizeof(struct usb_setup_packet));
	if (!buf) {
		LOG_ERR("Failed to allocate buffer");
		return -ENOMEM;
	}

	work_handler_xfer_continue(dev, ep, buf);

	udc_ep_buf_set_setup(buf);

	net_buf_add(buf, sizeof(struct usb_setup_packet));

	udc_ctrl_update_stage(dev, buf);

	if (udc_ctrl_stage_is_data_out(dev)) {
		/* Allocate and feed buffer for data OUT stage */
		LOG_ERR("s:%p|feed for -out-", buf);
		err = it82xx2_ctrl_feed_dout(dev, udc_data_stage_length(buf));
		if (err == -ENOMEM) {
			err = udc_submit_ep_event(dev, buf, err);
		}
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		udc_ctrl_submit_s_in_status(dev);
	} else {
		udc_ctrl_submit_s_status(dev);
	}

	return err;
}

static int work_handler_in(const struct device *dev, uint8_t ep)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	struct it82xx2_usb_ep_regs *ep_regs = usb_regs->usb_ep_regs;
	struct net_buf *buf;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (ep_regs[ep].ep_ctrl & EP_SEND_STALL) {
		ep_regs[ep].ep_ctrl &= ~EP_SEND_STALL;
		LOG_DBG("Cleared stall bit");
		return -EINVAL;
	}

	buf = udc_buf_get(dev, ep);
	if (!buf) {
		return -ENODATA;
	}
	if (ep == USB_CONTROL_EP_IN) {
		if (udc_ctrl_stage_is_status_in(dev) ||
		    udc_ctrl_stage_is_no_data(dev)) {
			/* Status stage finished, notify upper layer */
			udc_ctrl_submit_status(dev, buf);
		}

		/* Update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_status_out(dev)) {
			/*
			 * IN transfer finished, release buffer,
			 * Feed control OUT buffer for status stage.
			 */
			net_buf_unref(buf);
			it82xx2_ctrl_feed_dout(dev, 0);
		}
	} else {
		LOG_ERR("[%s]ITE Debug[%d]-TODO", __func__, __LINE__);
		return udc_submit_ep_event(dev, buf, 0);
	}

	ep_regs[ep_idx].ep_ctrl |= ENDPOINT_RDY;

	return 0;
}

static int work_handler_out(const struct device *dev, uint8_t ep)
{
	struct net_buf *buf;
	int err = 0;

	buf = udc_buf_get(dev, ep);
	if (buf == NULL) {
		return -ENODATA;
	}

	if (ep == USB_CONTROL_EP_OUT) {
		if (udc_ctrl_stage_is_status_out(dev)) {
			/* s-in-status finished, next bd is already fed */
			LOG_DBG("dout:%p|no feed", buf);
			/* Status stage finished, notify upper layer */
			udc_ctrl_submit_status(dev, buf);
		} else {
			/*
			 * For all other cases we feed with a buffer
			 * large enough for setup packet.
			 */
			LOG_ERR("[%s]ITE Debug[%d]-TODO", __func__, __LINE__);
			// LOG_ERR("dout:%p|feed >setup", buf);
			// err = usbfsotg_ctrl_feed_dout(dev, 8U, false, false);
		}

		/* Update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_status_in(dev)) {
			err = udc_ctrl_submit_s_out_status(dev, buf);
		}
	} else {
		LOG_ERR("[%s]ITE Debug[%d]-TODO", __func__, __LINE__);
		err = udc_submit_ep_event(dev, buf, 0);
	}

	return err;
}

static void xfer_work_handler(const struct device *dev)
{
	struct it82xx2_ep_event *evt;
	struct it82xx2_priv_data *priv = udc_get_private(dev);
	int err = 0;

	while (true) {
		evt = k_fifo_get(&priv->fifo, K_FOREVER);
		if (!evt) {
			goto free;
		}

		switch(evt->event) {
		case IT82xx2_EVT_SETUP_TOKEN:
			err = work_handler_setup(evt->dev, evt->ep);
			break;
		case IT82xx2_EVT_IN_TOKEN:
			err = work_handler_in(evt->dev, evt->ep);
			break;
		case IT82xx2_EVT_OUT_TOKEN:
			err = work_handler_out(evt->dev, evt->ep);
			break;
		case IT82xx2_EVT_XFER:
			err = work_handler_xfer_next(evt->dev, evt->ep);
			break;
		default:
			break;
		}

		if (err) {
			udc_submit_event(evt->dev, UDC_EVT_ERROR, err);
		}

free:
		k_mem_slab_free(&udc_it82xx2_slab, (void **)&evt);
	}
}

static void it82xx2_usb_xfer_done(const struct device *dev)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	struct it82xx2_usb_ep_regs *ep_regs = usb_regs->usb_ep_regs;
	struct net_buf *buf;

	for (uint8_t ep_idx = EP0 ; ep_idx < EP4 ; ep_idx++) {
		uint8_t ep_ctrl = ep_regs[ep_idx].ep_ctrl;

		/* check ready bit ,will be 0 when trans done */
		if ((ep_ctrl & ENDPOINT_EN) && !(ep_ctrl & ENDPOINT_RDY)) {
			uint8_t ep = ep_idx;

			switch (ep_regs[ep_idx].ep_transtype_sts & DC_ALL_TRANS) {
			case DC_SETUP_TRANS:
				/* SETUP transaction done */
				if (config->ep_cfg_out[ep].caps.control == 1) {
					ep_regs[ep_idx].ep_ctrl |= EP_OUTDATA_SEQ;
					it82xx2_event_submit(dev, ep_idx, IT82xx2_EVT_SETUP_TOKEN);
				}
				break;
			case DC_IN_TRANS:
				/* IN transaction done */
				ep |= USB_EP_DIR_IN;

				if (!!(ep_regs[ep_idx].ep_ctrl & EP_OUTDATA_SEQ)) {
					ep_regs[ep_idx].ep_ctrl &= ~EP_OUTDATA_SEQ;
				} else {
					ep_regs[ep_idx].ep_ctrl |= EP_OUTDATA_SEQ;
				}

				buf = udc_buf_peek(dev, ep);
				if (!buf) {
					udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
					break;
				}

				net_buf_pull(buf, MIN(buf->len, config->ep_cfg_in[ep_idx].caps.mps));

				if (buf->len) {
					work_handler_xfer_continue(dev, ep, buf);
				} else {
					if (udc_ep_buf_has_zlp(buf)) {
						LOG_ERR("[%s] ITE Debug [%d] - without verify", __func__, __LINE__);
						work_handler_xfer_continue(dev, ep, buf);
						udc_ep_buf_clear_zlp(buf);
						break;
					}

					it82xx2_event_submit(dev, ep, IT82xx2_EVT_IN_TOKEN);
				}
				break;
			case DC_OUTDATA_TRANS:
				/* OUT transaction done */
				ep |= USB_EP_DIR_OUT;

				if (!!(ep_regs[ep_idx].ep_ctrl & EP_OUTDATA_SEQ)) {
					ep_regs[ep_idx].ep_ctrl &= ~EP_OUTDATA_SEQ;
				} else {
					ep_regs[ep_idx].ep_ctrl |= EP_OUTDATA_SEQ;
				}

				buf = udc_buf_peek(dev, ep);
				if (!buf) {
					udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
					break;
				}
				struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep);
				net_buf_add(buf, buf->len);
				uint16_t mps = config->ep_cfg_out[ep_idx].caps.mps;
				if (net_buf_tailroom(buf) >= mps && buf->len == mps) {
					LOG_ERR("[%s]ITE Debug[%d]-TODO handle > mps",
						__func__, __LINE__);
				} else {
					it82xx2_event_submit(dev, ep_idx, IT82xx2_EVT_OUT_TOKEN);
				}

				break;
			default:
				LOG_ERR("Unknown transaction type");
				continue;
			}
			udc_ep_set_busy(dev, ep, false);
		}
	}
}

static void it82xx2_usb_dc_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	struct it82xx2_priv_data *priv = udc_get_private(dev);

	uint8_t status = usb_regs->dc_interrupt_status &
		usb_regs->dc_interrupt_mask; /* mask non enable int */

	/* reset event */
	if (status & DC_RESET_EVENT) {
		if ((usb_regs->dc_line_status & RX_LINE_STATE_MASK) ==
			RX_LINE_RESET) {
			it82xx2_dc_reset(dev);
			usb_regs->dc_interrupt_status = DC_RESET_EVENT;

			udc_submit_event(dev, UDC_EVT_RESET, 0);
			return;
		} else {
			usb_regs->dc_interrupt_status = DC_RESET_EVENT;
		}
	}

	/* sof received */
	if (status & DC_SOF_RECEIVED) {
		it82xx2_enable_sof_int(dev, false);
		k_work_reschedule(&priv->suspended_work, K_MSEC(5));
	}

	/* transaction done */
	if (status & DC_TRANS_DONE) {
		/* clear interrupt before new transaction */
		usb_regs->dc_interrupt_status = DC_TRANS_DONE;
		it82xx2_usb_xfer_done(dev);
		return;
	}
}

static void suspended_handler(struct k_work *item)
{
	struct it82xx2_priv_data *priv =
		CONTAINER_OF(item, struct it82xx2_priv_data, suspended_work);
	const struct device *dev = priv->dev;
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;

	if (usb_regs->dc_interrupt_status & DC_SOF_RECEIVED) {
		usb_regs->dc_interrupt_status = DC_SOF_RECEIVED;
		if (priv->suspended) {
			udc_set_suspended(dev, false);
			udc_submit_event(dev, UDC_EVT_RESUME, 0);
			priv->suspended = false;
			k_sem_give(&priv->suspended_sem);
		}
		k_work_reschedule(&priv->suspended_work, K_MSEC(5));
		return;
	}

	it82xx2_enable_sof_int(dev, true);

	if (!priv->suspended) {
		udc_set_suspended(dev, true);
		udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
		priv->suspended = true;
		it82xx2_enable_wu_irq(dev, true);
		it82xx2_enable_standby_state(true);

		k_sem_reset(&priv->suspended_sem);
	}
}

static int it82xx2_enable(const struct device *dev)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;
	struct it82xx2_priv_data *priv = udc_get_private(dev);

	k_sem_init(&priv->suspended_sem, 0, 1);
	k_work_init_delayable(&priv->suspended_work, suspended_handler);

	/* Connect USB interrupt */
	irq_connect_dynamic(config->usb_irq, 0, it82xx2_usb_dc_isr, dev, 0);

	usb_regs->dc_control =
		DC_GLOBAL_ENABLE | DC_FULL_SPEED_LINE_POLARITY |
		DC_FULL_SPEED_LINE_RATE | DC_CONNECT_TO_HOST;

	/* Enable USB D+ and USB interrupts */
	it82xx2_enable_wu_irq(dev, true);
	irq_enable(config->usb_irq);

	return 0;
}

static int it82xx2_disable(const struct device *dev)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct usb_it82xx2_regs *const usb_regs = config->base;

	irq_disable(config->usb_irq);

	/* stop pull-up D+ D-*/
	usb_regs->dc_control &= ~DC_CONNECT_TO_HOST;

	return 0;
}

static int it82xx2_init(const struct device *dev)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct it82xx2_priv_data *priv = udc_get_private(dev);
	struct gctrl_it8xxx2_regs *const gctrl_regs = GCTRL_IT8XXX2_REGS_BASE;
	int ret;

	/*
	 * Disable USB debug path , prevent CPU enter
	 * JTAG mode and then reset by USB command.
	 */
	gctrl_regs->GCTRL_MCCR &= ~(IT8XXX2_GCTRL_MCCR_USB_EN);
	gctrl_regs->gctrl_pmer2 |= IT8XXX2_GCTRL_PMER2_USB_PAD_EN;

	it82xx2_usb_dc_ip_init(dev);

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
				     USB_EP_TYPE_CONTROL, config->ep_cfg_out[USB_EP_GET_IDX(EP0)].caps.mps, 0);
	if (ret) {
		LOG_ERR("Failed to enable ep 0x%02x", USB_CONTROL_EP_OUT);
		return ret;
	}

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN,
				      USB_EP_TYPE_CONTROL, config->ep_cfg_in[USB_EP_GET_IDX(EP0)].caps.mps, 0);
	if (ret) {
		LOG_ERR("Failed to enable ep 0x%02x", USB_CONTROL_EP_IN);
		return ret;
	}
	return 0;
}

static int it82xx2_shutdown(const struct device *dev)
{
	LOG_ERR("[%s]ITE Debug[%d]-TODO", __func__, __LINE__);

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	return 0;
}

static int it82xx2_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

static int it82xx2_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

static const struct udc_api it82xx2_api = {
	.ep_enqueue = it82xx2_ep_enqueue,
	.ep_dequeue = it82xx2_ep_dequeue,
	.ep_set_halt = it82xx2_ep_set_halt,
	.ep_clear_halt = it82xx2_ep_clear_halt,
	.ep_try_config = NULL,
	.ep_enable = it82xx2_ep_enable,
	.ep_disable = it82xx2_ep_disable,
	.host_wakeup = it82xx2_host_wakeup,
	.set_address = it82xx2_set_address,
	.enable = it82xx2_enable,
	.disable = it82xx2_disable,
	.init = it82xx2_init,
	.shutdown = it82xx2_shutdown,
	.lock = it82xx2_lock,
	.unlock = it82xx2_unlock,
};

static int it82xx2_usb_driver_preinit(const struct device *dev)
{
	const struct usb_it82xx2_config *config = dev->config;
	struct udc_data *data = dev->data;
	struct it82xx2_priv_data *priv = udc_get_private(dev);
	int err;

	k_mutex_init(&data->mutex);
	k_fifo_init(&priv->fifo);

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("Failed to configure usb pins");
		return err;
	}

	/* Initializing WU (USB D+) */
	it8xxx2_usb_dc_wuc_init(dev);

	for (int i = 0; i < 16; i++) {
		config->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.mps = USB_CONTROL_EP_MPS;
		} else if ((i % 3) == 2){
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.iso = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		}

		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (int i = 0; i < 16; i++) {
		config->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			config->ep_cfg_in[i].caps.control = 1;
			config->ep_cfg_in[i].caps.mps = USB_CONTROL_EP_MPS;
		} else if ((i % 3) != 2){
			config->ep_cfg_in[i].caps.bulk = 1;
			config->ep_cfg_in[i].caps.interrupt = 1;
			config->ep_cfg_in[i].caps.iso = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		}

		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	data->caps.rwup = true;
	data->caps.hs = true;
	data->caps.mps0 = UDC_MPS0_64;

	priv->dev = dev;

	config->make_thread(dev);

	return 0;
}

#define IT82xx2_USB_DEVICE_DEFINE(n)						\
	K_KERNEL_STACK_DEFINE(udc_it82xx2_stack_##n, 1024);	\
										\
	static void udc_it82xx2_thread_##n(void *dev, void *arg1, void *arg2)	\
	{									\
		ARG_UNUSED(arg1);	\
		ARG_UNUSED(arg2);	\
		xfer_work_handler(dev);					\
	}									\
										\
	static void udc_it82xx2_make_thread_##n(const struct device *dev)	\
	{									\
		struct it82xx2_priv_data *priv = udc_get_private(dev);		\
										\
		k_thread_create(&priv->thread_data,				\
				udc_it82xx2_stack_##n,				\
				K_THREAD_STACK_SIZEOF(udc_it82xx2_stack_##n),	\
				udc_it82xx2_thread_##n,			\
				(void *)dev, NULL, NULL,			\
				K_PRIO_COOP(8),\
				0, K_NO_WAIT);					\
		k_thread_name_set(&priv->thread_data, dev->name);		\
	}									\
										\
	PINCTRL_DT_INST_DEFINE(n);	\
	static struct usb_it8xxx2_wuc usb_wuc_##n[IT8XXX2_DT_INST_WUCCTRL_LEN(n)] =	\
		IT8XXX2_DT_WUC_ITEMS_LIST(n);	\
										\
	static struct udc_ep_config ep_cfg_out[16];		\
	static struct udc_ep_config ep_cfg_in[16];		\
										\
	static struct usb_it82xx2_config udc_cfg_##n = {			\
		.base = (struct usb_it82xx2_regs *)DT_INST_REG_ADDR(n),			\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),	\
		.wuc_list = usb_wuc_##n,	\
		.usb_irq = DT_INST_IRQ_BY_IDX(n, 0, irq),	\
		.wu_irq = DT_INST_IRQ_BY_IDX(n, 1, irq),	\
		.ep_cfg_in = ep_cfg_out,					\
		.ep_cfg_out = ep_cfg_in,					\
		.make_thread = udc_it82xx2_make_thread_##n,			\
	};									\
										\
	static struct it82xx2_priv_data priv_data_##n = {				\
	};									\
										\
	static struct udc_data udc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),		\
		.priv = &priv_data_##n,						\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, it82xx2_usb_driver_preinit, NULL,			\
			      &udc_data_##n, &udc_cfg_##n,			\
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &it82xx2_api);

DT_INST_FOREACH_STATUS_OKAY(IT82xx2_USB_DEVICE_DEFINE)
