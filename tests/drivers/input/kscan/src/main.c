/*
 * Copyright (c) 2026 ITE Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_kbd_matrix.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(keycode_detect, LOG_LEVEL_DBG);

#define KEYBOARD_NODE DT_CHOSEN(keyboard_device)

static const struct device *const kbd_dev = DEVICE_DT_GET(KEYBOARD_NODE);

static int keycode_detect_init(void)
{
	struct input_kbd_matrix_common_data *data = kbd_dev->data;

	/* fix up the device thread priority */
	k_thread_priority_set(&data->thread, 8);

	return 0;
}
SYS_INIT(keycode_detect_init, APPLICATION, 10);

static void keycode_detect_cb(struct input_event *evt, void *user_data)
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
		LOG_DBG("keycode changed - r:%d c:%d press:%d", row, col, pressed);
	}
}
INPUT_CALLBACK_DEFINE(kbd_dev, keycode_detect_cb, NULL);
