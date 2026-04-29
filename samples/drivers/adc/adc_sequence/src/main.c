/*
 * Copyright (c) 2024 Centro de Inovacao EDGE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/device.h>

#define EEPROM_SAMPLE_OFFSET 0

/* ADC node from the devicetree. */
#define ADC_NODE DT_ALIAS(adc0)

/* Auxiliary macro to obtain channel vref, if available. */
#define CHANNEL_VREF(node_id) DT_PROP_OR(node_id, zephyr_vref_mv, 0)

/* Data of ADC device specified in devicetree. */
static const struct device *adc = DEVICE_DT_GET(ADC_NODE);

/* Data array of ADC channels for the specified ADC. */
static const struct adc_channel_cfg channel_cfgs[] = {
	DT_FOREACH_CHILD_SEP(ADC_NODE, ADC_CHANNEL_CFG_DT, (,))};

/* Data array of ADC channel voltage references. */
static uint32_t vrefs_mv[] = {DT_FOREACH_CHILD_SEP(ADC_NODE, CHANNEL_VREF, (,))};

/* Get the number of channels defined on the DTS. */
#define CHANNEL_COUNT ARRAY_SIZE(channel_cfgs)

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>

#define ADC_THREAD_STACK_SIZE 1024
#define ADC_THREAD_PRIORITY   5

K_THREAD_STACK_DEFINE(adc_stack, ADC_THREAD_STACK_SIZE);
static struct k_thread adc_thread_data;

static int adc_read_channel(int ch)
{
	int rv;
	int ret = 0;

	struct adc_sequence seq = {
		.options = NULL,
		.channels = BIT(ch),
		.buffer = &ret,
		.buffer_size = sizeof(ret),
		.resolution = CONFIG_SEQUENCE_RESOLUTION,
		.oversampling = CONFIG_SEQUENCE_OVERSAMPLING,
		.calibrate = false,
	};

	rv = adc_read(adc, &seq);
	if (rv)
		return rv;

	adc_raw_to_millivolts(adc_ref_internal(adc),
			      ADC_GAIN_1, CONFIG_SEQUENCE_RESOLUTION,
			      &ret);
	// printf("%d: %" PRId32 "\n", ch, ret);
	return ret;
}

static void adc_thread(void *arg1, void *arg2, void *arg3)
{
	const int *ch = arg2;

	while (1) {
		adc_read_channel(0);
		adc_read_channel(3);
		adc_read_channel(6);
		k_sleep(K_MSEC(10));
	}
}

void start_adc_thread(const struct device *adc_dev, const int ch)
{
	k_tid_t tid;
	char thread_name[32];

	tid = k_thread_create(&adc_thread_data,
			      adc_stack,
			      K_THREAD_STACK_SIZEOF(adc_stack),
			      adc_thread,
			      (void *)adc_dev, (void *)ch, NULL,
			      ADC_THREAD_PRIORITY,
			      0,
			      K_NO_WAIT);

	snprintf(thread_name, sizeof(thread_name), "adc_thread_%d", ch);
	k_thread_name_set(tid, thread_name);
}

static const struct device *get_eeprom_device(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(eeprom_0));

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found EEPROM device \"%s\"\n", dev->name);
	return dev;
}

#include <zephyr/input/input.h>
#include <zephyr/input/input_kbd_matrix.h>
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
		// printk("keycode changed - r:%d c:%d press:%d\n", row, col, pressed);
	}
}
INPUT_CALLBACK_DEFINE(kbd_dev, keycode_detect_cb, NULL);

int main(void)
{
	int err;
	uint16_t channel_reading[CONFIG_SEQUENCE_SAMPLES][CHANNEL_COUNT];

	/* Configure the sampling sequence to be made. */
	struct adc_sequence sequence = {
		.buffer = channel_reading,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(channel_reading),
		.resolution = CONFIG_SEQUENCE_RESOLUTION,
		.oversampling = CONFIG_SEQUENCE_OVERSAMPLING,
		.options = NULL,
	};

	if (!device_is_ready(adc)) {
		printf("ADC controller device %s not ready\n", adc->name);
		return 0;
	}

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < CHANNEL_COUNT; i++) {
		sequence.channels |= BIT(channel_cfgs[i].channel_id);
		err = adc_channel_setup(adc, &channel_cfgs[i]);
		if (err < 0) {
			printf("Could not setup channel #%d (%d)\n", i, err);
			return 0;
		}
		if ((vrefs_mv[i] == 0) && (channel_cfgs[i].reference == ADC_REF_INTERNAL)) {
			vrefs_mv[i] = adc_ref_internal(adc);
		}
	}

	start_adc_thread(adc, 0);
	start_adc_thread(adc, 3);
	// for (size_t i = 0U; i < 2; i++) {
	// 	start_adc_thread(adc, channel_cfgs[i].channel_id);
	// 	printf("ite debug %d\n", channel_cfgs[i].channel_id);
	// }

	const struct device *eeprom = get_eeprom_device();
	uint8_t temp2[128];

	while(1) {
		int rc = eeprom_read(eeprom, EEPROM_SAMPLE_OFFSET, &temp2, sizeof(temp2));
		if (rc < 0) {
			printk("Error: Couldn't read eeprom: err: %d.\n", rc);
			return 0;
		}

		k_sleep(K_MSEC(10));
	}
	// while(1) {
	// 	// printf("ADC sequence reading [%u]:\n", count++);

	// 	k_sleep(K_MSEC(1));

	// 	for (size_t channel_index = 0U; channel_index < CHANNEL_COUNT; channel_index++) {
	// 		int32_t val_mv = adc_read_channel(channel_cfgs[channel_index].channel_id);

	// 		// printf("- %s, channel %" PRId32 ", %" PRId32 " sequence samples",
	// 		//        adc->name, channel_cfgs[channel_index].channel_id,
	// 		//        CONFIG_SEQUENCE_SAMPLES);

	// 		// printf(" = %" PRId32 "mV\n", val_mv);
	// 	}
	// }

	return 0;
}
