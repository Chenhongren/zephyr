/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sample_usbd.h>

#include <zephyr/kernel.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_dfu.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

USBD_DEVICE_DEFINE(dfu_usbd,
		   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   0x2fe3, 0xffff);

USBD_DESC_LANG_DEFINE(sample_lang);
USBD_DESC_CONFIG_DEFINE(fs_cfg_desc, "DFU FS Configuration");
USBD_DESC_CONFIG_DEFINE(hs_cfg_desc, "DFU HS Configuration");

static const uint8_t attributes = (IS_ENABLED(CONFIG_SAMPLE_USBD_SELF_POWERED) ?
				   USB_SCD_SELF_POWERED : 0) |
				  (IS_ENABLED(CONFIG_SAMPLE_USBD_REMOTE_WAKEUP) ?
				   USB_SCD_REMOTE_WAKEUP : 0);
/* Full speed configuration */
USBD_CONFIGURATION_DEFINE(sample_fs_config,
			  attributes,
			  CONFIG_SAMPLE_USBD_MAX_POWER, &fs_cfg_desc);

/* High speed configuration */
USBD_CONFIGURATION_DEFINE(sample_hs_config,
			  attributes,
			  CONFIG_SAMPLE_USBD_MAX_POWER, &hs_cfg_desc);

static void switch_to_dfu_mode(struct usbd_context *const ctx);

struct dfu_ramdisk_data {
	const char *name;
	uint32_t last_block;
	uint32_t sector_size;
	uint32_t sector_count;
	union {
		uint32_t uploaded;
		uint32_t downloaded;
	};
};

static struct dfu_ramdisk_data ramdisk0_data = {
	.name = "image0",
};

#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];
struct ring_buf ringbuf;
static bool rx_throttled;
K_SEM_DEFINE(dtr_sem, 0, 1);
static inline void print_baudrate(const struct device *dev);

static int init_dfu_ramdisk_data(struct dfu_ramdisk_data *const data)
{
	int err;

	err = disk_access_init(data->name);
	if (err) {
		return err;
	}

	err = disk_access_status(data->name);
	if (err) {
		return err;
	}

	err = disk_access_ioctl(data->name, DISK_IOCTL_GET_SECTOR_COUNT, &data->sector_count);
	if (err) {
		return err;
	}

	err = disk_access_ioctl(data->name, DISK_IOCTL_GET_SECTOR_SIZE, &data->sector_size);
	if (err) {
		return err;
	}

	LOG_INF("disk %s sector count %u sector size %u",
		data->name, data->sector_count, data->sector_size);

	return err;
}

static int ramdisk_read(void *const priv, const uint32_t block, const uint16_t size,
			uint8_t buf[static CONFIG_USBD_DFU_TRANSFER_SIZE])
{
	struct dfu_ramdisk_data *const data = priv;
	int err;

	if (size == 0) {
		/* There is nothing to upload */
		return 0;
	}

	if (block == 0) {
		if (init_dfu_ramdisk_data(data)) {
			LOG_ERR("Failed to init ramdisk data");
			return -EINVAL;
		}

		data->last_block = 0;
		data->uploaded = 0;
	} else {
		if (data->last_block + 1U != block) {
			return -EINVAL;
		}

	}

	if (block >= data->sector_count) {
		/* Nothing to upload */
		return 0;
	}

	err = disk_access_read(data->name, buf, block, 1);
	if (err) {
		LOG_ERR("Failed to read from RAMdisk");
		return err;
	}

	data->last_block = block;
	data->uploaded += MIN(size, data->sector_size);
	LOG_INF("block %u size %u uploaded %u", block, size, data->uploaded);

	return size;
}

static int ramdisk_write(void *const priv, const uint32_t block, const uint16_t size,
			 const uint8_t buf[static CONFIG_USBD_DFU_TRANSFER_SIZE])
{
	struct dfu_ramdisk_data *const data = priv;
	int err;

	if (block == 0) {
		if (init_dfu_ramdisk_data(data)) {
			LOG_ERR("Failed to init ramdisk data");
			return -EINVAL;
		}

		data->last_block = 0;
		data->downloaded = 0;
	} else {
		if (data->last_block + 1U != block) {
			return -EINVAL;
		}

	}

	if (size == 0) {
		/* Nothing to write */
		return 0;
	}

	err = disk_access_write(data->name, buf, block, 1);
	if (err) {
		LOG_ERR("Failed to write to RAMdisk");
		return err;
	}

	data->last_block = block;
	data->downloaded += size;
	LOG_INF("block %u size %u downloaded %u", block, size, data->downloaded);

	return 0;
}

USBD_DFU_DEFINE_IMG(ramdisk0, "ramdisk0", &ramdisk0_data, ramdisk_read, ramdisk_write, NULL);

static void msg_cb(struct usbd_context *const usbd_ctx,
		   const struct usbd_msg *const msg)
{
	LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));

	if (msg->type == USBD_MSG_CONFIGURATION) {
		LOG_INF("\tConfiguration value %d", msg->status);
	}

	if (usbd_can_detect_vbus(usbd_ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(usbd_ctx)) {
				LOG_ERR("Failed to enable device support");
			}
		}

		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(usbd_ctx)) {
				LOG_ERR("Failed to disable device support");
			}
		}
	}

	if (msg->type == USBD_MSG_DFU_APP_DETACH) {
		switch_to_dfu_mode(usbd_ctx);
	}

	if (msg->type == USBD_MSG_DFU_DOWNLOAD_COMPLETED) {
		if (IS_ENABLED(CONFIG_BOOTLOADER_MCUBOOT) &&
		    IS_ENABLED(CONFIG_APP_USB_DFU_USE_FLASH_BACKEND)) {
			boot_request_upgrade(false);
		}
	}

	if (msg->type == USBD_MSG_CDC_ACM_CONTROL_LINE_STATE) {
		uint32_t dtr = 0U;

		uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			k_sem_give(&dtr_sem);
		}
	}

	if (msg->type == USBD_MSG_CDC_ACM_LINE_CODING) {
		print_baudrate(msg->dev);
	}
}

static void switch_to_dfu_mode(struct usbd_context *const ctx)
{
	int err;

	LOG_INF("Detach USB device");
	usbd_disable(ctx);
	usbd_shutdown(ctx);

	err = usbd_add_descriptor(&dfu_usbd, &sample_lang);
	if (err) {
		LOG_ERR("Failed to initialize language descriptor (%d)", err);
		return;
	}

	if (usbd_caps_speed(&dfu_usbd) == USBD_SPEED_HS) {
		err = usbd_add_configuration(&dfu_usbd, USBD_SPEED_HS, &sample_hs_config);
		if (err) {
			LOG_ERR("Failed to add High-Speed configuration");
			return;
		}

		err = usbd_register_class(&dfu_usbd, "dfu_dfu", USBD_SPEED_HS, 1);
		if (err) {
			LOG_ERR("Failed to add register classes");
			return;
		}

		usbd_device_set_code_triple(&dfu_usbd, USBD_SPEED_HS, 0, 0, 0);
	}

	err = usbd_add_configuration(&dfu_usbd, USBD_SPEED_FS, &sample_fs_config);
	if (err) {
		LOG_ERR("Failed to add Full-Speed configuration");
		return;
	}

	err = usbd_register_class(&dfu_usbd, "dfu_dfu", USBD_SPEED_FS, 1);
	if (err) {
		LOG_ERR("Failed to add register classes");
		return;
	}

	usbd_device_set_code_triple(&dfu_usbd, USBD_SPEED_FS, 0, 0, 0);

	err = usbd_init(&dfu_usbd);
	if (err) {
		LOG_ERR("Failed to initialize USB device support");
		return;
	}

	err = usbd_msg_register_cb(&dfu_usbd, msg_cb);
	if (err) {
		LOG_ERR("Failed to register message callback");
		return;
	}

	err = usbd_enable(&dfu_usbd);
	if (err) {
		LOG_ERR("Failed to enable USB device support");
	}
}

enum kb_report_idx {
	KB_MOD_KEY = 0,
	KB_RESERVED,
	KB_KEY_CODE1,
	KB_KEY_CODE2,
	KB_KEY_CODE3,
	KB_KEY_CODE4,
	KB_KEY_CODE5,
	KB_KEY_CODE6,
	KB_REPORT_COUNT,
};

enum mouse_report_idx {
	MOUSE_BTN_REPORT_IDX = 0,
	MOUSE_X_REPORT_IDX = 1,
	MOUSE_Y_REPORT_IDX = 2,
	MOUSE_WHEEL_REPORT_IDX = 3,
	MOUSE_REPORT_COUNT = 4,
};

static const uint8_t hid_mouse_desc[] = HID_MOUSE_REPORT_DESC(2);
static const uint8_t hid_keyboard_desc[] = HID_KEYBOARD_REPORT_DESC();
static uint32_t kb_duration;
static bool kb_ready;
static bool mouse_ready;

static void kb_iface_ready(const struct device *dev, const bool ready)
{
	LOG_INF("HID device %s interface is %s", dev->name, ready ? "ready" : "not ready");
	kb_ready = ready;
}

static int kb_get_report(const struct device *dev, const uint8_t type, const uint8_t id,
			 const uint16_t len, uint8_t *const buf)
{
	LOG_WRN("Get Report not implemented, Type %u ID %u", type, id);

	return 0;
}

static int kb_set_report(const struct device *dev, const uint8_t type, const uint8_t id,
			 const uint16_t len, const uint8_t *const buf)
{
	if (type != HID_REPORT_TYPE_OUTPUT) {
		LOG_WRN("Unsupported report type");
		return -ENOTSUP;
	}

	return 0;
}

/* Idle duration is stored but not used to calculate idle reports. */
static void kb_set_idle(const struct device *dev, const uint8_t id, const uint32_t duration)
{
	LOG_INF("Set Idle %u to %u", id, duration);
	kb_duration = duration;
}

static uint32_t kb_get_idle(const struct device *dev, const uint8_t id)
{
	LOG_INF("Get Idle %u to %u", id, kb_duration);
	return kb_duration;
}

static void kb_set_protocol(const struct device *dev, const uint8_t proto)
{
	LOG_INF("Protocol changed to %s", proto == 0U ? "Boot Protocol" : "Report Protocol");
}

static void kb_output_report(const struct device *dev, const uint16_t len, const uint8_t *const buf)
{
	LOG_HEXDUMP_DBG(buf, len, "o.r.");
	kb_set_report(dev, HID_REPORT_TYPE_OUTPUT, 0U, len, buf);
}

struct hid_device_ops kb_ops = {
	.iface_ready = kb_iface_ready,
	.get_report = kb_get_report,
	.set_report = kb_set_report,
	.set_idle = kb_set_idle,
	.get_idle = kb_get_idle,
	.set_protocol = kb_set_protocol,
	.output_report = kb_output_report,
};

static void mouse_iface_ready(const struct device *dev, const bool ready)
{
	LOG_INF("HID device %s interface is %s", dev->name, ready ? "ready" : "not ready");
	mouse_ready = ready;
}

static int mouse_get_report(const struct device *dev, const uint8_t type, const uint8_t id,
			    const uint16_t len, uint8_t *const buf)
{
	LOG_WRN("Get Report not implemented, Type %u ID %u", type, id);

	return 0;
}

struct hid_device_ops mouse_ops = {
	.iface_ready = mouse_iface_ready,
	.get_report = mouse_get_report,
};

#define STACK_SIZE      1024
#define THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(keyboard_area, STACK_SIZE);
K_THREAD_STACK_DEFINE(mouse_area, STACK_SIZE);
struct k_thread keyboard_data;
struct k_thread mouse_data;

void keyboard_handler(void *p1, void *p2, void *p3)
{
	const struct device *hid_dev = p1;
	UDC_STATIC_BUF_DEFINE(kb_report, KB_REPORT_COUNT);
	int ret;

	while (true) {
		if (!kb_ready) {
			goto out;
		}

		if (kb_report[KB_KEY_CODE1] != HID_KEY_A) {
			kb_report[KB_KEY_CODE1] = HID_KEY_A;
		} else {
			kb_report[KB_KEY_CODE1] = 0;
		}

		ret = hid_device_submit_report(hid_dev, KB_REPORT_COUNT, kb_report);
		if (ret) {
			LOG_ERR("keyboard submit report error, %d", ret);
		}
out:
		k_sleep(K_MSEC(5));
	}
}

void mouse_handler(void *p1, void *p2, void *p3)
{
	const struct device *mouse_dev = p1;
	UDC_STATIC_BUF_DEFINE(mouse_report, MOUSE_REPORT_COUNT);

	int ret;

	while (true) {
		if (!mouse_ready) {
			goto out;
		}

		mouse_report[MOUSE_X_REPORT_IDX] = 10U;

		ret = hid_device_submit_report(mouse_dev, MOUSE_REPORT_COUNT, mouse_report);
		if (ret) {
			LOG_ERR("mouse submit report error, %d", ret);
		}
out:
		k_sleep(K_MSEC(5));
	}
}

static inline void print_baudrate(const struct device *dev)
{
	uint32_t baudrate;
	int ret;

	ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret) {
		LOG_WRN("Failed to get baudrate, ret code %d", ret);
	} else {
		LOG_INF("Baudrate %u", baudrate);
	}
}

static void interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (true) {
		uart_irq_update(dev);

		if (uart_irq_is_pending(dev) <= 0) {
			break;
		}

		if (!rx_throttled && uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf), sizeof(buffer));

			if (len == 0) {
				/* Throttle because ring buffer is full */
				uart_irq_rx_disable(dev);
				rx_throttled = true;
				continue;
			}

			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) {
				LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			if (rb_len < recv_len) {
				LOG_ERR("Drop %u bytes", recv_len - rb_len);
			}

			LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);
			if (rb_len) {
				uart_irq_tx_enable(dev);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len) {
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			if (rx_throttled) {
				uart_irq_rx_enable(dev);
				rx_throttled = false;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
	}
}

int main(void)
{
	const struct device *uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	const struct device *keyboard_dev = DEVICE_DT_GET(DT_NODELABEL(keyboard_dev));
	const struct device *mouse_dev = DEVICE_DT_GET(DT_NODELABEL(mouse_dev));
	struct usbd_context *sample_usbd;
	int ret;

	if (!device_is_ready(mouse_dev)) {
		LOG_ERR("mouse device is not ready");
		return -EIO;
	}

	if (!device_is_ready(keyboard_dev)) {
		LOG_ERR("keyboard is not ready");
		return -EIO;
	}

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("cdc acm device not ready");
		return -EIO;
	}

	ret = hid_device_register(mouse_dev, hid_mouse_desc, sizeof(hid_mouse_desc), &mouse_ops);
	if (ret != 0) {
		LOG_ERR("Failed to register HID Device, %d", ret);
		return ret;
	}

	ret = hid_device_register(keyboard_dev, hid_keyboard_desc, sizeof(hid_keyboard_desc),
				  &kb_ops);
	if (ret != 0) {
		LOG_ERR("Failed to register HID Device, %d", ret);
		return ret;
	}

	sample_usbd = sample_usbd_init_device(msg_cb);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	if (!usbd_can_detect_vbus(sample_usbd)) {
		ret = usbd_enable(sample_usbd);
		if (ret) {
			LOG_ERR("Failed to enable device support");
			return ret;
		}
	}

	LOG_INF("USB DFU sample is initialized");

	k_thread_create(&keyboard_data, keyboard_area, K_THREAD_STACK_SIZEOF(keyboard_area),
			keyboard_handler, (void *)keyboard_dev, NULL, NULL, THREAD_PRIORITY, 0,
			K_NO_WAIT);

	k_thread_create(&mouse_data, mouse_area, K_THREAD_STACK_SIZEOF(mouse_area), mouse_handler,
			(void *)mouse_dev, NULL, NULL, THREAD_PRIORITY, 0, K_NO_WAIT);

	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

	LOG_INF("Wait for DTR");
	k_sem_take(&dtr_sem, K_FOREVER);
	LOG_INF("DTR set");

	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_DCD, 1);
	if (ret) {
		LOG_WRN("Failed to set DCD, ret code %d", ret);
	}

	ret = uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_DSR, 1);
	if (ret) {
		LOG_WRN("Failed to set DSR, ret code %d", ret);
	}

	/* Wait 100ms for the host to do all settings */
	k_msleep(100);

	uart_irq_callback_set(uart_dev, interrupt_handler);
	/* Enable rx interrupts */
	uart_irq_rx_enable(uart_dev);

	return 0;
}

static int confirm_image_init(void)
{
	if (!boot_is_img_confirmed()) {
		LOG_INF("mark image is confirmed");
		boot_write_img_confirmed();
	}

	return 0;
}

SYS_INIT(confirm_image_init, APPLICATION, 0);
