/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/spi.h>

#define GOOGLE_BYTE_PER_FRAME 1
#define OUTPUT_PATTERN_SIZE   4

static struct {
	uint32_t row;
	uint32_t col;
	bool pressed;
} last_cb_val;
static int callback_calls_count;

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

static void kscan_callback(const struct device *dev, uint32_t row, uint32_t col, bool pressed)
{
	printf("row = %u col = %u %d\n", row, col, pressed);

	callback_calls_count++;
	last_cb_val.row = row;
	last_cb_val.col = col;
	last_cb_val.pressed = pressed;
}

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	static const struct device *input_dev = DEVICE_DT_GET(DT_NODELABEL(kbd));
	static const struct device *kscan_dev = DEVICE_DT_GET(DT_ALIAS(kscan0));

	if (!device_is_ready(kscan_dev)) {
		printf("KBSCAN device is not ready\n");
		return -ENODEV;
	}

	if (kscan_config(kscan_dev, kscan_callback) != 0) {
		printf("Unexpected error code received\n");
		return -EINVAL;
	}

	if (kscan_disable_callback(kscan_dev) != 0) {
		printf("Error while disabling callback\n");
		return EINVAL;
	}

	if (kscan_enable_callback(kscan_dev) != 0) {
		printf("Error while enabling callback\n");
		return EINVAL;
	}

	input_report_abs(input_dev, INPUT_ABS_X, 101, false, K_FOREVER);

	input_report_abs(input_dev, INPUT_ABS_Y, 102, false, K_FOREVER);

	input_report_key(input_dev, INPUT_BTN_TOUCH, 1, true, K_FOREVER);

	input_report_abs(input_dev, INPUT_ABS_X, 103, true, K_FOREVER);

	input_report_key(input_dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);

	google_spi_mosi_test();

	return 0;
}
