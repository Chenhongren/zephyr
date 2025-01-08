/*
 * Copyright (c) 2025 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_i3c_tgt_mqueue

#include <zephyr/drivers/i3c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tgt_mqueue, LOG_LEVEL_DBG);

struct i3c_tgt_mqueue_config {
};

struct i3c_tgt_mqueue_data {
	const struct device *controller;

	struct i3c_target_config target_config;
	struct k_work tgt_stop_work;

	struct {
		struct {
			uint8_t data[CONFIG_I3CS_IT51XXX_RX_FIFO_SIZE];
			size_t len;
			bool full;
		} rx;

		struct {
			uint8_t data[CONFIG_I3CS_IT51XXX_TX_FIFO_SIZE];
		} tx;

	} buffer;
};

enum opcode_table {
	LOOPBACK_OPCODE = 0xBF,
};

static int prepare_tx_fifo(struct i3c_tgt_mqueue_data *data)
{
	int ret;

	for (size_t i = 0; i < sizeof(data->buffer.tx.data); i++) {
		data->buffer.tx.data[i] = i;
	}

	ret = i3c_target_tx_write(data->controller, data->buffer.tx.data,
				  sizeof(data->buffer.tx.data), 0);
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

static void i3c_target_work(struct k_work *work)
{
	struct i3c_tgt_mqueue_data *data =
		CONTAINER_OF(work, struct i3c_tgt_mqueue_data, tgt_stop_work);

	if (data->buffer.rx.len != 0 || data->buffer.rx.full) {
		uint8_t opcode;

		LOG_INF("received %d bytes data",
			data->buffer.rx.full ? sizeof(data->buffer.rx.data) : data->buffer.rx.len);
		LOG_HEXDUMP_INF(data->buffer.rx.data,
				data->buffer.rx.full ? sizeof(data->buffer.rx.data)
						     : data->buffer.rx.len,
				"data:");

		opcode = data->buffer.rx.data[0];
		switch (opcode) {
		case LOOPBACK_OPCODE:
			raise_ibi_tir(data->controller, data->buffer.rx.data,
				      data->buffer.rx.full ? sizeof(data->buffer.rx.data)
							   : data->buffer.rx.len);
			break;
		default:
			break;
		};
		data->buffer.rx.full = false;
		data->buffer.rx.len = 0;
	} else {
		LOG_INF("finished tx data");
		prepare_tx_fifo(data);
	}
}

static int i3c_tgt_mq_write_requested_cb(struct i3c_target_config *config)
{
	LOG_INF("%s ITE Debug %d", __func__, __LINE__);
	return 0;
}

static int i3c_tgt_mq_write_received_cb(struct i3c_target_config *config, uint8_t val)
{
	LOG_INF("%s ITE Debug %d", __func__, __LINE__);
	return 0;
}

static int i3c_tgt_mq_read_requested_cb(struct i3c_target_config *config, uint8_t *val)
{
	LOG_INF("%s ITE Debug %d", __func__, __LINE__);
	return 0;
}

static int i3c_tgt_mq_read_processed_cb(struct i3c_target_config *config, uint8_t *val)
{
	LOG_INF("%s ITE Debug %d", __func__, __LINE__);
	return 0;
}

static void i3c_tgt_mq_buf_write_received_cb(struct i3c_target_config *config, uint8_t *ptr,
					     uint32_t len)
{
	struct i3c_tgt_mqueue_data *data =
		CONTAINER_OF(config, struct i3c_tgt_mqueue_data, target_config);

	data->buffer.rx.len = len;

	if (sizeof(data->buffer.rx.data) < len) {
		data->buffer.rx.len = sizeof(data->buffer.rx.data);
		data->buffer.rx.full = true;
		LOG_WRN("received data(%d) is too much, only copy %d bytes", len,
			data->buffer.rx.len);
	}
	memcpy(data->buffer.rx.data, ptr, data->buffer.rx.len);
}

static int i3c_tgt_mq_buf_read_requested_cb(struct i3c_target_config *config, uint8_t **ptr,
					    uint32_t *len, uint8_t *hdr_mode)
{
	LOG_INF("%s ITE Debug %d", __func__, __LINE__);
	return 0;
}

static int i3c_tgt_mq_stop_cb(struct i3c_target_config *config)
{
	struct i3c_tgt_mqueue_data *data =
		CONTAINER_OF(config, struct i3c_tgt_mqueue_data, target_config);

	k_work_submit(&data->tgt_stop_work);

	return 0;
}

static const struct i3c_target_callbacks i3c_target_loopback_callbacks = {
	.write_requested_cb = i3c_tgt_mq_write_requested_cb,
	.write_received_cb = i3c_tgt_mq_write_received_cb,
	.read_requested_cb = i3c_tgt_mq_read_requested_cb,
	.read_processed_cb = i3c_tgt_mq_read_processed_cb,
	.buf_write_received_cb = i3c_tgt_mq_buf_write_received_cb,
	.buf_read_requested_cb = i3c_tgt_mq_buf_read_requested_cb,
	.stop_cb = i3c_tgt_mq_stop_cb,
};

static int i3c_tgt_mqueue_init(const struct device *dev)
{
	struct i3c_tgt_mqueue_data *data = dev->data;
	int ret;

	data->target_config.callbacks = &i3c_target_loopback_callbacks;

	ret = i3c_target_register(data->controller, &data->target_config);
	if (ret) {
		LOG_ERR("failed to register %s, ret %d", dev->name, ret);
		return ret;
	}

	k_work_init(&data->tgt_stop_work, i3c_target_work);

	LOG_INF("registered %s on %s controller", dev->name, data->controller->name);

	raise_ibi_hj(data->controller);

	prepare_tx_fifo(data);
	return 0;
}

#define I3C_TARGET_MQUEUE_INIT(n)                                                                  \
	static const struct i3c_tgt_mqueue_config i3c_tgt_mqueue_config_##n = {};                  \
                                                                                                   \
	static struct i3c_tgt_mqueue_data i3c_tgt_mqueue_data_##n = {                              \
		.controller = DEVICE_DT_GET(DT_INST_BUS(n)),                                       \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, i3c_tgt_mqueue_init, NULL, &i3c_tgt_mqueue_data_##n,              \
			      &i3c_tgt_mqueue_config_##n, POST_KERNEL, 51, NULL);

DT_INST_FOREACH_STATUS_OKAY(I3C_TARGET_MQUEUE_INIT)
