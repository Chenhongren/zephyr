/*
 * Copyright (c) 2025 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_i2c_target

#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ite_i2c_target, LOG_LEVEL_WRN);

#define ITE_I2C_TGT_TX_LEN 8

struct ite_i2c_tgt_config {
	mm_reg_t reg;
	struct i2c_dt_spec bus;
};

struct ite_i2c_tgt_data {
	const struct device *controller;
	const struct device *dev;

	struct i2c_target_config target_cfg;

	struct {
		uint8_t tx;
		uint8_t rx;
	} pio_sequence_number;

	uint8_t dma_tx_prt[ITE_I2C_TGT_TX_LEN];
};

static int ite_tgt_write_requested(struct i2c_target_config *tgt_config)
{
	struct ite_i2c_tgt_data *data =
		CONTAINER_OF(tgt_config, struct ite_i2c_tgt_data, target_cfg);

	LOG_DBG("%s: %s()", data->dev->name, __func__);

	return 0;
}

static int ite_tgt_write_received(struct i2c_target_config *tgt_config, uint8_t in)
{
	struct ite_i2c_tgt_data *data =
		CONTAINER_OF(tgt_config, struct ite_i2c_tgt_data, target_cfg);

	LOG_DBG("%s: %s()", data->dev->name, __func__);

	if (data->pio_sequence_number.rx == 0xFF) {
		data->pio_sequence_number.rx = in;
	} else if (data->pio_sequence_number.rx != in) {
		LOG_ERR("DATA ERROR!! %d != %d", data->pio_sequence_number.rx, in);
	}
	data->pio_sequence_number.rx++;

	return 0;
}

static int ite_tgt_read_requested(struct i2c_target_config *tgt_config, uint8_t *out)
{
	struct ite_i2c_tgt_data *data =
		CONTAINER_OF(tgt_config, struct ite_i2c_tgt_data, target_cfg);

	LOG_DBG("%s: %s()", data->dev->name, __func__);

	*out = data->pio_sequence_number.tx;
	data->pio_sequence_number.tx++;

	return 0;
}

static int ite_tgt_read_processed(struct i2c_target_config *tgt_config, uint8_t *out)
{
	struct ite_i2c_tgt_data *data =
		CONTAINER_OF(tgt_config, struct ite_i2c_tgt_data, target_cfg);

	LOG_DBG("%s: %s()", data->dev->name, __func__);

	*out = 0x55;

	return 0;
}

static int ite_tgt_stop(struct i2c_target_config *tgt_config)
{
	struct ite_i2c_tgt_data *data =
		CONTAINER_OF(tgt_config, struct ite_i2c_tgt_data, target_cfg);

	LOG_DBG("%s: %s()", data->dev->name, __func__);

	return 0;
}

static void ite_tgt_error(struct i2c_target_config *tgt_config, enum i2c_error_reason err_code)
{
	struct ite_i2c_tgt_data *data =
		CONTAINER_OF(tgt_config, struct ite_i2c_tgt_data, target_cfg);

	LOG_ERR("%s: detected error code %d", data->dev->name, err_code);
}

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
static void ite_tgt_buf_write_received(struct i2c_target_config *tgt_config, uint8_t *ptr,
				       uint32_t len)
{
	struct ite_i2c_tgt_data *data =
		CONTAINER_OF(tgt_config, struct ite_i2c_tgt_data, target_cfg);

	LOG_DBG("%s: %s()", data->dev->name, __func__);
}

static int ite_tgt_buf_read_requested(struct i2c_target_config *tgt_config, uint8_t **ptr,
				      uint32_t *len)
{
	struct ite_i2c_tgt_data *data =
		CONTAINER_OF(tgt_config, struct ite_i2c_tgt_data, target_cfg);

	LOG_DBG("%s: %s()", data->dev->name, __func__);

	for (int i = 0; i < ITE_I2C_TGT_TX_LEN; i++) {
		data->dma_tx_prt[i] = i;
	}

	*ptr = data->dma_tx_prt;
	*len = sizeof(data->dma_tx_prt);

	return 0;
}
#endif /* CONFIG_I2C_TARGET_BUFFER_MODE */

static const struct i2c_target_callbacks ite_i2c_tgt_callbacks = {
	.write_requested = ite_tgt_write_requested,
	.read_requested = ite_tgt_read_requested,
	.write_received = ite_tgt_write_received,
	.read_processed = ite_tgt_read_processed,
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	.buf_write_received = ite_tgt_buf_write_received,
	.buf_read_requested = ite_tgt_buf_read_requested,
#endif /* CONFIG_I2C_TARGET_BUFFER_MODE */
	.stop = ite_tgt_stop,
	.error = ite_tgt_error,
};

static int ite_i2c_tgt_init(const struct device *dev)
{
	const struct ite_i2c_tgt_config *config = dev->config;
	struct ite_i2c_tgt_data *data = dev->data;
	int ret;

	data->dev = dev;

	data->target_cfg.address = config->reg;
	data->target_cfg.callbacks = &ite_i2c_tgt_callbacks;

	ret = i2c_target_register(config->bus.bus, &data->target_cfg);
	if (ret) {
		LOG_ERR("failed to register %s target device", dev->name);
	}

	data->pio_sequence_number.tx = 0;
	data->pio_sequence_number.rx = 0xFF;

	LOG_INF("registered %s on controller", dev->name);

	return ret;
}

#define ITE_I2C_TARGET_INIT(n)                                                                     \
	static const struct ite_i2c_tgt_config ite_i2c_tgt_config_##n = {                          \
		.bus = I2C_DT_SPEC_INST_GET(n),                                                    \
		.reg = DT_INST_REG_ADDR(n),                                                        \
	};                                                                                         \
	static struct ite_i2c_tgt_data ite_i2c_tgt_data_##n = {                                    \
		.controller = DEVICE_DT_GET(DT_INST_BUS(n)),                                       \
	};                                                                                         \
                                                                                                   \
	static struct ite_i2c_tgt_data ite_i2c_tgt_data_##n;                                       \
	DEVICE_DT_INST_DEFINE(n, ite_i2c_tgt_init, NULL, &ite_i2c_tgt_data_##n,                    \
			      &ite_i2c_tgt_config_##n, POST_KERNEL, 51, NULL);

DT_INST_FOREACH_STATUS_OKAY(ITE_I2C_TARGET_INIT)
