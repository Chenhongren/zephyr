/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_kbd_matrix.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/spi.h>

#define GOOGLE_BYTE_PER_FRAME 1
#define OUTPUT_PATTERN_SIZE   4

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(kbd), okay)
#define CROS_EC_KEYBOARD_NODE DT_CHOSEN(cros_ec_keyboard)

static const struct device *const kbd_dev = DEVICE_DT_GET(CROS_EC_KEYBOARD_NODE);

static atomic_t disable_scan_mask;

void input_kbd_matrix_drive_column_hook(const struct device *dev, int col)
{
	LOG_DBG("hook: drive column %d ", col);
}

uint8_t keyboard_get_cols(void)
{
	const struct input_kbd_matrix_common_config *cfg = kbd_dev->config;

	return cfg->col_size;
}

uint8_t keyboard_get_rows(void)
{
	const struct input_kbd_matrix_common_config *cfg = kbd_dev->config;

	return cfg->row_size;
}

enum kb_scan_disable_masks {
	/* Reasons why keyboard scanning should be disabled */
	KB_SCAN_DISABLE_LID_CLOSED = (1 << 0),
	KB_SCAN_DISABLE_POWER_BUTTON = (1 << 1),
	KB_SCAN_DISABLE_LID_ANGLE = (1 << 2),
	KB_SCAN_DISABLE_USB_SUSPENDED = (1 << 3),
};

void keyboard_scan_enable(int enable, enum kb_scan_disable_masks mask)
{
	int prev_mask;
	int curr_mask;

	if (enable) {
		prev_mask = atomic_and(&disable_scan_mask, ~mask);
	} else {
		prev_mask = atomic_or(&disable_scan_mask, mask);
	}

	curr_mask = atomic_get(&disable_scan_mask);

	if (prev_mask != curr_mask) {
		LOG_INF("KB disable_scanning_mask changed: 0x%08x", curr_mask);
	}

	if (!pm_device_runtime_is_enabled(kbd_dev)) {
		LOG_WRN("device %s does not support runtime PM", kbd_dev->name);
		return;
	}

	if (prev_mask == 0 && curr_mask != 0) {
		pm_device_runtime_put(kbd_dev);
	} else if (prev_mask != 0 && curr_mask == 0) {
		pm_device_runtime_get(kbd_dev);
	}
}

static int keyboard_input_init(void)
{
	struct input_kbd_matrix_common_data *data = kbd_dev->data;

	/* Initialize as active */
	pm_device_runtime_get(kbd_dev);

	/* fix up the device thread priority */
	k_thread_priority_set(&data->thread, 8);

	return 0;
}

SYS_INIT(keyboard_input_init, APPLICATION, 0);

static void keyboard_input_cb(struct input_event *evt, void *user_data)
{
	static int row;
	static int col;
	static bool pressed;

	switch (evt->code) {
	case INPUT_ABS_X:
		col = evt->value;
		break;
	case INPUT_ABS_Y:
		row = evt->value;
		break;
	case INPUT_BTN_TOUCH:
		pressed = evt->value;
		break;
	}

	if (evt->sync) {
		LOG_WRN("keyboard_state_changed %d %d %d", row, col, pressed);
	}
}
INPUT_CALLBACK_DEFINE(kbd_dev, keyboard_input_cb, NULL);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(kbd), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi0), okay)
static int google_spi_mosi_test(void)
{
	struct device *spi_device = (struct device *)DEVICE_DT_GET(DT_NODELABEL(spi0));
	struct spi_config spi_cfg = {
		.operation = SPI_WORD_SET(8),
		.frequency = MHZ(6),
		.slave = 0,
	};

	/* SCLK = 6MHz,
	 * one frame:  (H) 5 * (1 / 6M) = 833us (L) 3 * (1 / 6M) = 500us
	 * zero frame: (H) 2 * (1 / 6M) = 333us (L) 6 * (1 / 6M) = 1us
	 */
	uint8_t one_frame[GOOGLE_BYTE_PER_FRAME] = {0xF8};
	uint8_t zero_frame[GOOGLE_BYTE_PER_FRAME] = {0xC0};
	uint8_t output_frames[OUTPUT_PATTERN_SIZE * GOOGLE_BYTE_PER_FRAME];
	uint8_t output_seq[OUTPUT_PATTERN_SIZE] = {1, 0, 1, 0};
	int ret;

	if (!spi_device) {
		printk("null spi device\n");
		return -ENODEV;
	}

	for (uint8_t i = 0; i < sizeof(output_seq); i++) {
		memcpy(output_frames + i * GOOGLE_BYTE_PER_FRAME,
		       output_seq[i] ? one_frame : zero_frame, GOOGLE_BYTE_PER_FRAME);
	}

	const struct spi_buf frame_buffers = {.buf = output_frames, .len = sizeof(output_frames)};

	const struct spi_buf_set buf_set = {.buffers = &frame_buffers, .count = 1};

	ret = spi_write(spi_device, &spi_cfg, &buf_set);
	if (ret < 0) {
		printk("failed to send spi data %d\n", ret);
		return ret;
	}

	return 0;
}
SYS_INIT(google_spi_mosi_test, APPLICATION, 0);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(spi0), okay) */

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	return 0;
}
