/*
 * Copyright (c) 2025 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_i3c_tgt_mqueue

#include <zephyr/drivers/i3c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tgt_mqueue, LOG_LEVEL_INF);

struct i3c_tgt_mqueue_config {
	void (*make_thread)(const struct device *dev);
};

struct i3c_tgt_mqueue_data {
	const struct device *controller;

	struct i3c_target_config target_config;

	struct k_thread thread_data;

	struct {
		struct {
			uint8_t data[CONFIG_I3CS_IT51XXX_RX_FIFO_SIZE];
			size_t len;
		} rx;

		struct {
			uint8_t data[CONFIG_I3CS_IT51XXX_TX_FIFO_SIZE];
		} tx;
	} buffer;
};

enum event_table {
	priv_read_evt = 0,
	priv_write_evt
};

enum opcode_table {
	/* Qualcomm loopback test: place payload into tx fifo after reception */
	QCOM_LOOPBACK_OPCODE = 0x3C,
	/* ITE Loopback Test: trigger an ibi after the payload has been received */
	ITE_LOOPBACK_OPCODE = 0xBF,
};

K_MSGQ_DEFINE(evt_msgq, sizeof(uint8_t), CONFIG_I3C_TGT_MQUEUE_EVENT_COUNT, sizeof(uint8_t));

static int prepare_tx_fifo(struct i3c_tgt_mqueue_data *data, size_t length)
{
	int ret;

	if (length == 0) {
		length = sizeof(data->buffer.tx.data);
		for (size_t i = 0; i < length; i++) {
			data->buffer.tx.data[i] = i;
		}
	}

	ret = i3c_target_tx_write(data->controller, data->buffer.tx.data,
				  length, 0);
	if (ret < 0) {
		LOG_ERR("%s: failed to write data to tx fifo, ret %d", data->controller->name, ret);
		return ret;
	}
	return 0;
}

static int raise_ibi_hj(const struct device *dev)
{
	struct i3c_ibi request;
	int ret;

	request.ibi_type = I3C_IBI_HOTJOIN;

	ret = i3c_ibi_raise(dev, &request);
	if (ret != 0) {
		LOG_ERR("failed to issue ibi hj");
		return ret;
	}
	return 0;
}

static int raise_ibi_tir(const struct device *dev, uint8_t *buf, const uint16_t data_length)
{
	struct i3c_ibi request;
	int ret;

	request.ibi_type = I3C_IBI_TARGET_INTR;
	request.payload = buf;
	request.payload_len = data_length;

	ret = i3c_ibi_raise(dev, &request);
	if (ret != 0) {
		LOG_ERR("unable to issue ibi tir");
		return ret;
	}
	return 0;
}

static void tgt_mqueue_handler(const struct device *dev)
{
	struct i3c_tgt_mqueue_data *data = dev->data;

	while (true) {
		uint8_t evt;

		k_msgq_get(&evt_msgq, &evt, K_FOREVER);

		if (evt == priv_write_evt) {
			LOG_INF("received %d bytes data", data->buffer.rx.len);
			LOG_HEXDUMP_DBG(data->buffer.rx.data, data->buffer.rx.len, "data:");
			switch (data->buffer.rx.data[0]) {
			case QCOM_LOOPBACK_OPCODE:
				memcpy(data->buffer.tx.data, data->buffer.rx.data,
				       data->buffer.rx.len);
				prepare_tx_fifo(data, data->buffer.rx.len);
				break;
			case ITE_LOOPBACK_OPCODE:
				/* drop the first byte(BFh) */
				raise_ibi_tir(data->controller, data->buffer.rx.data + 1,
					      data->buffer.rx.len - 1);
				break;
			default:
				break;
			}
			data->buffer.rx.len = 0;
		}

		if (evt == priv_read_evt) {
			LOG_INF("finished tx data");
			prepare_tx_fifo(data, 0);
		}
	}
}

static void i3c_tgt_mq_buf_write_received_cb(struct i3c_target_config *config, uint8_t *ptr,
					     uint32_t len)
{
	struct i3c_tgt_mqueue_data *data =
		CONTAINER_OF(config, struct i3c_tgt_mqueue_data, target_config);

	data->buffer.rx.len = len;

	if (sizeof(data->buffer.rx.data) < len) {
		data->buffer.rx.len = sizeof(data->buffer.rx.data);
		LOG_WRN("received data(%d) is too much, only copy %d bytes", len,
			data->buffer.rx.len);
	}
	memcpy(data->buffer.rx.data, ptr, data->buffer.rx.len);
}

static int i3c_tgt_mq_buf_read_requested_cb(struct i3c_target_config *config, uint8_t **ptr,
					    uint32_t *len, uint8_t *hdr_mode)
{
	ARG_UNUSED(config);
	ARG_UNUSED(ptr);
	ARG_UNUSED(len);
	ARG_UNUSED(hdr_mode);

	/* private read transfer is finished */
	return 0;
}

static int i3c_tgt_mq_stop_cb(struct i3c_target_config *config)
{
	struct i3c_tgt_mqueue_data *data =
		CONTAINER_OF(config, struct i3c_tgt_mqueue_data, target_config);
	uint8_t evt;

	if (data->buffer.rx.len != 0) {
		evt = priv_write_evt;
		k_msgq_put(&evt_msgq, &evt, K_NO_WAIT);
	} else {
		evt = priv_read_evt;
		k_msgq_put(&evt_msgq, &evt, K_NO_WAIT);
	}
	return 0;
}

static const struct i3c_target_callbacks i3c_target_callbacks = {
	.buf_write_received_cb = i3c_tgt_mq_buf_write_received_cb,
	.buf_read_requested_cb = i3c_tgt_mq_buf_read_requested_cb,
	.stop_cb = i3c_tgt_mq_stop_cb,
};

static int i3c_tgt_mqueue_init(const struct device *dev)
{
	const struct i3c_tgt_mqueue_config *config = dev->config;
	struct i3c_tgt_mqueue_data *data = dev->data;
	int ret;

	data->target_config.callbacks = &i3c_target_callbacks;

	ret = i3c_target_register(data->controller, &data->target_config);
	if (ret) {
		LOG_ERR("failed to register %s, ret %d", dev->name, ret);
		return ret;
	}

	config->make_thread(dev);
	LOG_INF("registered %s on %s controller", dev->name, data->controller->name);

	raise_ibi_hj(data->controller);

	prepare_tx_fifo(data, 0);

	return 0;
}

#define I3C_TARGET_MQUEUE_INIT(n)                                                                  \
                                                                                                   \
	K_KERNEL_STACK_DEFINE(i3c_tgt_mqueue_stack_##n, CONFIG_I3C_TGT_MQUEUE_STACK_SIZE);         \
	static void i3c_tgt_mqueue_thread_##n(void *dev, void *arg1, void *arg2)                   \
	{                                                                                          \
		ARG_UNUSED(arg1);                                                                  \
		ARG_UNUSED(arg2);                                                                  \
		tgt_mqueue_handler(dev);                                                           \
	}                                                                                          \
                                                                                                   \
	static void i3c_tgt_mqueue_make_thread_##n(const struct device *dev)                       \
	{                                                                                          \
		struct i3c_tgt_mqueue_data *data = dev->data;                                      \
                                                                                                   \
		k_thread_create(&data->thread_data, i3c_tgt_mqueue_stack_##n,                      \
				K_THREAD_STACK_SIZEOF(i3c_tgt_mqueue_stack_##n),                   \
				i3c_tgt_mqueue_thread_##n, (void *)dev, NULL, NULL,                \
				K_PRIO_PREEMPT(CONFIG_I3C_TGT_MQUEUE_THREAD_PRIORITY), 0,          \
				K_NO_WAIT);                                                        \
		k_thread_name_set(&data->thread_data, dev->name);                                  \
	}                                                                                          \
                                                                                                   \
	static const struct i3c_tgt_mqueue_config i3c_tgt_mqueue_config_##n = {                    \
		.make_thread = i3c_tgt_mqueue_make_thread_##n,                                     \
	};                                                                                         \
                                                                                                   \
	static struct i3c_tgt_mqueue_data i3c_tgt_mqueue_data_##n = {                              \
		.controller = DEVICE_DT_GET(DT_INST_BUS(n)),                                       \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, i3c_tgt_mqueue_init, NULL, &i3c_tgt_mqueue_data_##n,              \
			      &i3c_tgt_mqueue_config_##n, POST_KERNEL, 51, NULL);

DT_INST_FOREACH_STATUS_OKAY(I3C_TARGET_MQUEUE_INIT)
