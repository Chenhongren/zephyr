/*
 * Copyright (c) 2017 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_at24

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <string.h>
#include <zephyr/drivers/i2c/slave/eeprom.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_slave);

struct i2c_eeprom_slave_data {
	struct i2c_slave_config config;
	uint32_t buffer_size;
	uint8_t *buffer;
	uint32_t buffer_idx;
	bool first_write;
};

struct i2c_eeprom_slave_config {
	struct i2c_dt_spec bus;
	uint32_t buffer_size;
	uint8_t *buffer;
};

int eeprom_slave_program(const struct device *dev, const uint8_t *eeprom_data,
			 unsigned int length)
{
	struct i2c_eeprom_slave_data *data = dev->data;

	if (length > data->buffer_size) {
		return -EINVAL;
	}

	memcpy(data->buffer, eeprom_data, length);

	return 0;
}

int eeprom_slave_read(const struct device *dev, uint8_t *eeprom_data,
		      unsigned int offset)
{
	struct i2c_eeprom_slave_data *data = dev->data;

	if (!data || offset >= data->buffer_size) {
		return -EINVAL;
	}

	*eeprom_data = data->buffer[offset];

	return 0;
}

static int eeprom_slave_write_requested(struct i2c_slave_config *config)
{
	struct i2c_eeprom_slave_data *data = CONTAINER_OF(config,
						struct i2c_eeprom_slave_data,
						config);

	LOG_DBG("eeprom: write req");

	data->first_write = true;

	return 0;
}

static int eeprom_slave_read_requested(struct i2c_slave_config *config,
				       uint8_t *val)
{
	struct i2c_eeprom_slave_data *data = CONTAINER_OF(config,
						struct i2c_eeprom_slave_data,
						config);

	*val = data->buffer[data->buffer_idx];

	LOG_DBG("eeprom: read req, val=0x%x", *val);

	/* Increment will be done in the read_processed callback */

	return 0;
}

static int eeprom_slave_write_received(struct i2c_slave_config *config,
				       uint8_t val)
{
	struct i2c_eeprom_slave_data *data = CONTAINER_OF(config,
						struct i2c_eeprom_slave_data,
						config);

	LOG_DBG("eeprom: write done, val=0x%x", val);

	/* In case EEPROM wants to be R/O, return !0 here could trigger
	 * a NACK to the I2C controller, support depends on the
	 * I2C controller support
	 */

	if (data->first_write) {
		data->buffer_idx = val;
		data->first_write = false;
	} else {
		data->buffer[data->buffer_idx++] = val;
	}

	data->buffer_idx = data->buffer_idx % data->buffer_size;

	return 0;
}

static int eeprom_slave_read_processed(struct i2c_slave_config *config,
				       uint8_t *val)
{
	struct i2c_eeprom_slave_data *data = CONTAINER_OF(config,
						struct i2c_eeprom_slave_data,
						config);

	/* Increment here */
	data->buffer_idx = (data->buffer_idx + 1) % data->buffer_size;

	*val = data->buffer[data->buffer_idx];

	LOG_DBG("eeprom: read done, val=0x%x", *val);

	/* Increment will be done in the next read_processed callback
	 * In case of STOP, the byte won't be taken in account
	 */

	return 0;
}

static int eeprom_slave_stop(struct i2c_slave_config *config)
{
	struct i2c_eeprom_slave_data *data = CONTAINER_OF(config,
						struct i2c_eeprom_slave_data,
						config);

	LOG_DBG("eeprom: stop");

	data->first_write = true;

	return 0;
}

static int eeprom_slave_register(const struct device *dev)
{
	const struct i2c_eeprom_slave_config *cfg = dev->config;
	struct i2c_eeprom_slave_data *data = dev->data;

	return i2c_slave_register(cfg->bus.bus, &data->config);
}

static int eeprom_slave_unregister(const struct device *dev)
{
	const struct i2c_eeprom_slave_config *cfg = dev->config;
	struct i2c_eeprom_slave_data *data = dev->data;

	return i2c_slave_unregister(cfg->bus.bus, &data->config);
}

static const struct i2c_slave_driver_api api_funcs = {
	.driver_register = eeprom_slave_register,
	.driver_unregister = eeprom_slave_unregister,
};

static const struct i2c_slave_callbacks eeprom_callbacks = {
	.write_requested = eeprom_slave_write_requested,
	.read_requested = eeprom_slave_read_requested,
	.write_received = eeprom_slave_write_received,
	.read_processed = eeprom_slave_read_processed,
	.stop = eeprom_slave_stop,
};

static int i2c_eeprom_slave_init(const struct device *dev)
{
	struct i2c_eeprom_slave_data *data = dev->data;
	const struct i2c_eeprom_slave_config *cfg = dev->config;

	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C controller device not ready");
		return -ENODEV;
	}

	data->buffer_size = cfg->buffer_size;
	data->buffer = cfg->buffer;
	data->config.address = cfg->bus.addr;
	data->config.callbacks = &eeprom_callbacks;

	return 0;
}

#define I2C_EEPROM_INIT(inst)						\
	static struct i2c_eeprom_slave_data				\
		i2c_eeprom_slave_##inst##_dev_data;			\
									\
	static uint8_t							\
	i2c_eeprom_slave_##inst##_buffer[(DT_INST_PROP(inst, size))];	\
									\
	static const struct i2c_eeprom_slave_config			\
		i2c_eeprom_slave_##inst##_cfg = {			\
		.bus = I2C_DT_SPEC_INST_GET(inst),			\
		.buffer_size = DT_INST_PROP(inst, size),		\
		.buffer = i2c_eeprom_slave_##inst##_buffer		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			    &i2c_eeprom_slave_init,			\
			    NULL,			\
			    &i2c_eeprom_slave_##inst##_dev_data,	\
			    &i2c_eeprom_slave_##inst##_cfg,		\
			    POST_KERNEL,				\
			    CONFIG_I2C_SLAVE_INIT_PRIORITY,		\
			    &api_funcs);

DT_INST_FOREACH_STATUS_OKAY(I2C_EEPROM_INIT)
