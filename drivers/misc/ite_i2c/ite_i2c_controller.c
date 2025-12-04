/*
 * Copyright (c) 2025 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_i2c_controller

#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ite_i2c_controller, LOG_LEVEL_INF);

#define ITE_I2C_CTRL_RX_LEN 8

struct ite_i2c_ctrl_config {
	mm_reg_t reg;
	const struct device *bus;
};

struct ite_i2c_ctrl_data {
	const struct device *controller;
	const struct device *dev;

	struct k_work_delayable work;

	struct {
		uint8_t tx;
		uint8_t rx[ITE_I2C_CTRL_RX_LEN];
	} buffer;
};

static void i2c_ctrl_work(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct ite_i2c_ctrl_data *data = CONTAINER_OF(dwork, struct ite_i2c_ctrl_data, work);
	const struct device *dev = data->dev;
	const struct ite_i2c_ctrl_config *config = dev->config;

	int ret = i2c_write(data->controller, &data->buffer.tx, 1, config->reg);
	if (ret) {
		// LOG_ERR("failed to write i2c data, ret %d", ret);
		goto out;
	}
	data->buffer.tx++;

	k_sleep(K_USEC(0));

	ret = i2c_read(data->controller, data->buffer.rx, sizeof(data->buffer.rx), config->reg);
	if (ret) {
		// LOG_ERR("failed to read i2c data, ret %d", ret);
	} else {
		LOG_DBG("read %d bytes: 0x%02x 0x%02x...", (int)sizeof(data->buffer.rx),
			data->buffer.rx[0], data->buffer.rx[1]);
	}

out:
	k_work_reschedule(&data->work, K_USEC(0));
}

static int ite_i2c_ctrl_init(const struct device *dev)
{
	const struct ite_i2c_ctrl_config *config = dev->config;
	struct ite_i2c_ctrl_data *data = dev->data;

	LOG_INF("registered %s on %s controller", dev->name, config->bus->name);

	data->dev = dev;
	data->buffer.tx = 0;

	k_work_init_delayable(&data->work, i2c_ctrl_work);
	k_work_reschedule(&data->work, K_MSEC(5));

	return 0;
}

#define ITE_I2C_CONTROLLER_INIT(n)                                                                 \
	static const struct ite_i2c_ctrl_config ite_i2c_ctrl_config_##n = {                        \
		.bus = DEVICE_DT_GET(DT_INST_BUS(n)),                                              \
		.reg = DT_INST_REG_ADDR(n),                                                        \
	};                                                                                         \
	static struct ite_i2c_ctrl_data ite_i2c_ctrl_data_##n = {                                  \
		.controller = DEVICE_DT_GET(DT_INST_BUS(n)),                                       \
	};                                                                                         \
                                                                                                   \
	static struct ite_i2c_ctrl_data ite_i2c_ctrl_data_##n;                                     \
	DEVICE_DT_INST_DEFINE(n, ite_i2c_ctrl_init, NULL, &ite_i2c_ctrl_data_##n,                  \
			      &ite_i2c_ctrl_config_##n, POST_KERNEL, 51, NULL);

DT_INST_FOREACH_STATUS_OKAY(ITE_I2C_CONTROLLER_INIT)
