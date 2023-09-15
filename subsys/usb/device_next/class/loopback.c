/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/usb/udc.h>

#include <zephyr/logging/log.h>
// LOG_MODULE_REGISTER(usb_loopback, CONFIG_USBD_LOOPBACK_LOG_LEVEL);
LOG_MODULE_REGISTER(usb_loopback, LOG_LEVEL_DBG);

/*
 * NOTE: this class is experimental and is in development.
 * Primary purpose currently is testing of the class initialization and
 * interface and endpoint configuration.
 */

/* Internal buffer for intermidiate test data */
static uint8_t lb_buf[1024];

#define LB_VENDOR_REQ_OUT		0x5b
#define LB_VENDOR_REQ_IN		0x5c

/* Make supported vendor request visible for the device stack */
static const struct usbd_cctx_vendor_req lb_vregs =
	USBD_VENDOR_REQ(LB_VENDOR_REQ_OUT, LB_VENDOR_REQ_IN);

struct usb_hid_class_subdescriptor {
	uint8_t bDescriptorType;
	uint16_t wDescriptorLength;
} __packed;

struct usb_hid_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdHID;
	uint8_t bCountryCode;
	uint8_t bNumDescriptors;

	/*
	 * Specification says at least one Class Descriptor needs to
	 * be present (Report Descriptor).
	 */
	struct usb_hid_class_subdescriptor subdesc[1];
} __packed;

struct loopback_desc {
	struct usb_if_descriptor if0;
	struct usb_hid_descriptor if0_hid;
	struct usb_ep_descriptor if0_in_ep;
} __packed;

#define DEFINE_LOOPBACK_DESCRIPTOR(x, _)			\
static struct loopback_desc lb_desc_##x = {			\
	/* Interface descriptor 0 */				\
	.if0 = {						\
		.bLength = sizeof(struct usb_if_descriptor),	\
		.bDescriptorType = USB_DESC_INTERFACE,		\
		.bInterfaceNumber = 0,				\
		.bAlternateSetting = 0,				\
		.bNumEndpoints = 1,				\
		.bInterfaceClass = USB_BCC_HID,		\
		.bInterfaceSubClass = 0,			\
		.bInterfaceProtocol = 0,			\
		.iInterface = 0,				\
	},							\
	.if0_hid = {					\
		.bLength = sizeof(struct usb_hid_descriptor),		\
		.bDescriptorType = 0x21,			\
		.bcdHID = sys_cpu_to_le16(0x0111),		\
		.bCountryCode = 0,					\
		.bNumDescriptors = 1,					\
		.subdesc[0] = {						\
			.bDescriptorType = 0x22,	\
			.wDescriptorLength = 0x34,				\
		},							\
	},					\
	/* Data Endpoint IN */					\
	.if0_in_ep = {						\
		.bLength = sizeof(struct usb_ep_descriptor),		\
		.bDescriptorType = USB_DESC_ENDPOINT,			\
		.bEndpointAddress = 0x81,				\
		.bmAttributes = USB_EP_TYPE_INTERRUPT,					\
		.wMaxPacketSize = sys_cpu_to_le16(16),			\
		.bInterval = 9,		\
	},							\
};								\

static void lb_update(struct usbd_class_node *c_nd,
		      uint8_t iface, uint8_t alternate)
{
	LOG_DBG("Instance %p, interface %u alternate %u changed",
		c_nd, iface, alternate);
}

static int lb_control_to_host(struct usbd_class_node *c_nd,
			      const struct usb_setup_packet *const setup,
			      struct net_buf *const buf)
{
	struct usbd_contex *uds_ctx = c_nd->data->uds_ctx;

	if (setup->RequestType.recipient != USB_REQTYPE_RECIPIENT_DEVICE) {
		errno = -ENOTSUP;
		return 0;
	}

	if (setup->bRequest == LB_VENDOR_REQ_IN) {
		net_buf_add_mem(buf, lb_buf,
				MIN(sizeof(lb_buf), setup->wLength));
		usbd_ep_ctrl_enqueue(uds_ctx, buf);

		LOG_WRN("Device-to-Host, wLength %u | %zu", setup->wLength,
			MIN(sizeof(lb_buf), setup->wLength));

		return 0;
	}

	LOG_ERR("Class request 0x%x not supported", setup->bRequest);
	errno = -ENOTSUP;

	return 0;
}

static int lb_control_to_dev(struct usbd_class_node *c_nd,
			     const struct usb_setup_packet *const setup,
			     const struct net_buf *const buf)
{
	if (setup->RequestType.recipient != USB_REQTYPE_RECIPIENT_DEVICE) {
		errno = -ENOTSUP;
		return 0;
	}

	if (setup->bRequest == LB_VENDOR_REQ_OUT) {
		LOG_WRN("Host-to-Device, wLength %u | %zu", setup->wLength,
			MIN(sizeof(lb_buf), buf->len));
		memcpy(lb_buf, buf->data, MIN(sizeof(lb_buf), buf->len));
		return 0;
	}

	LOG_ERR("Class request 0x%x not supported", setup->bRequest);
	errno = -ENOTSUP;

	return 0;
}

static int lb_request_handler(struct usbd_class_node *c_nd,
			      struct net_buf *buf, int err)
{
	struct udc_buf_info *bi = NULL;

	bi = (struct udc_buf_info *)net_buf_user_data(buf);
	LOG_DBG("%p -> ep 0x%02x, len %u, err %d", c_nd, bi->ep, buf->len, err);
	usbd_ep_buf_free(c_nd->data->uds_ctx, buf);

	return 0;
}

static int lb_init(struct usbd_class_node *c_nd)
{
	LOG_DBG("Init class instance %p", c_nd);

	return 0;
}

struct usbd_class_api lb_api = {
	.update = lb_update,
	.control_to_host = lb_control_to_host,
	.control_to_dev = lb_control_to_dev,
	.request = lb_request_handler,
	.init = lb_init,
};

#define DEFINE_LOOPBACK_CLASS_DATA(x, _)				\
	static struct usbd_class_data lb_class_##x = {			\
		.desc = (struct usb_desc_header *)&lb_desc_##x,		\
		.v_reqs = &lb_vregs,					\
	};								\
									\
	USBD_DEFINE_CLASS(loopback_##x, &lb_api, &lb_class_##x);

LISTIFY(CONFIG_USBD_LOOPBACK_INSTANCES_COUNT, DEFINE_LOOPBACK_DESCRIPTOR, ())
LISTIFY(CONFIG_USBD_LOOPBACK_INSTANCES_COUNT, DEFINE_LOOPBACK_CLASS_DATA, ())
