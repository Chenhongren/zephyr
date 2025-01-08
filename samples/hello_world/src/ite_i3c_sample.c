/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include "libi3c_controller.h"
#include "libi3c_target.h"

#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/target_device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i3c_sample_2evb, LOG_LEVEL_DBG);

#ifdef CONFIG_ITE_I2C_TEST
#define TEST_ZEPHYR_API 1
#endif /* CONFIG_ITE_I2C_TEST */

#ifdef CONFIG_ITE_LPS22DF_TEST
#define TEST_ZEPHYR_API 1
#endif /* CONFIG_ITE_LPS22DF_TEST */

#ifdef CONFIG_ITE_2EVB_TEST
/* only apply at the controller side */
#define TEST_BROADCAST_CCC 0
#define TEST_DIRECT_CCC    0
#define TEST_ZEPHYR_API    0
#endif /* CONFIG_ITE_2EVB_TEST */

int main(void)
{

#ifdef CONFIG_ITE_I2C_TEST
#ifdef TEST_ZEPHYR_API
	const struct device *dev;
	int dev_addr = 0x5D;
	uint8_t reg_addr = 0x0F;
	uint8_t reg_value_origin;
	uint8_t buf[16];
	int ret;

	dev = device_get_by_dt_nodelabel("i3c0");
	if (!dev) {
		LOG_ERR("failed to get i3c0 device");
		return -ENODEV;
	}

	ret = i2c_write_read(dev, dev_addr, &reg_addr, 1, buf, 1);
	if (ret < 0) {
		LOG_ERR("i2c: failed to read from device: 0x%x", dev_addr);
		return -EIO;
	}
	LOG_INF("i2c: finished write-then-read test. offset[%02x]: 0x%x", reg_addr, buf[0]);

	reg_addr = 0x10;
	ret = i2c_reg_read_byte(dev, dev_addr, reg_addr, buf);
	if (ret < 0) {
		LOG_ERR("i2c: failed to read from device: 0x%x", dev_addr);
		return -EIO;
	}
	LOG_INF("i2c: finished to read byte test. offset[%02x]: 0x%x", reg_addr, buf[0]);
	reg_value_origin = buf[0];

	ret = i2c_reg_write_byte(dev, dev_addr, reg_addr, 0x0F);
	if (ret < 0) {
		LOG_ERR("i2c: failed to write from device: 0x%x", dev_addr);
		return -EIO;
	}

	ret = i2c_reg_read_byte(dev, dev_addr, reg_addr, buf);
	if (ret < 0) {
		LOG_ERR("i2c: failed to read from device: 0x%x", dev_addr);
		return -EIO;
	}
	if (buf[0] != 0x0F) {
		LOG_INF("i2c: failed to write byte test");
		return -EIO;
	}
	LOG_INF("i2c: finished to write byte test. offset[%02x]: 0x%x", reg_addr, buf[0]);

	ret = i2c_reg_update_byte(dev, dev_addr, reg_addr, 0xFF, reg_value_origin);
	if (ret < 0) {
		LOG_ERR("Failed to read from device: 0x%x", dev_addr);
		return -EIO;
	}

	ret = i2c_reg_read_byte(dev, dev_addr, reg_addr, buf);
	if (ret < 0) {
		LOG_ERR("i2c: failed to read from device: 0x%x", dev_addr);
		return -EIO;
	}
	if (reg_value_origin != buf[0]) {
		LOG_INF("i2c: failed to update byte test");
		return -EIO;
	}
	LOG_INF("i2c: finished to update byte test. offset[%02xh]: 0x%x", reg_addr, buf[0]);

	reg_addr = 0x0;
	ret = i2c_burst_read(dev, dev_addr, reg_addr, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("i2c: failed to burst read test");
		return -EIO;
	}
	LOG_INF("i2c: finished to burst read test.");
	LOG_HEXDUMP_INF(buf, sizeof(buf), "burst read:");
#endif /* TEST_ZEPHYR_API */
#endif /* CONFIG_ITE_I2C_TEST */

#ifdef CONFIG_ITE_LPS22DF_TEST
	const struct device *controll_dev_1, *ctrl_dummy_sensor, *st_lps22df;

	controll_dev_1 = device_get_binding("i3c@f03c00");
	if (!controll_dev_1) {
		LOG_ERR("failed to get i3c device");
		return -ENODEV;
	}

	ctrl_dummy_sensor = device_get_binding("ctrl_dummy_sensor");
	if (!ctrl_dummy_sensor) {
		LOG_ERR("failed to get ctrl_dummy_sensor dev");
	}

	st_lps22df = device_get_binding("st_lps22df");
	if (!st_lps22df) {
		LOG_ERR("failed to get st_lps22df dev");
	}

#if TEST_ZEPHYR_API
	const uint8_t reg_addr = 0x10;
	uint8_t reg_value, reg_value_origin;

	libi3c_reg_read_byte(controll_dev_1, ctrl_dummy_sensor, NULL, reg_addr, &reg_value_origin);
	libi3c_reg_write_byte(controll_dev_1, ctrl_dummy_sensor, reg_addr, 0xEF);
	libi3c_reg_read_byte(controll_dev_1, ctrl_dummy_sensor, NULL, reg_addr, &reg_value);
	if (reg_value != 0xEF) {
		LOG_INF("failed to write/read register");
		return -EINVAL;
	}
	libi3c_reg_update_byte(controll_dev_1, ctrl_dummy_sensor, NULL, reg_addr, 0xFF,
			       reg_value_origin);
	libi3c_reg_read_byte(controll_dev_1, ctrl_dummy_sensor, NULL, reg_addr, &reg_value);
	if (reg_value != reg_value_origin) {
		LOG_INF("failed to update/read register");
		return -EINVAL;
	}
	LOG_INF("finished to test write/read/update register");
#endif /* TEST_ZEPHYR_API */
#endif /* CONFIG_ITE_LPS22DF_TEST */

#ifdef CONFIG_ITE_2EVB_TEST
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c2), okay) || DT_NODE_HAS_STATUS(DT_NODELABEL(i3c3), okay)
	LOG_INF("2 EVBs: target side...");
	return 0;
#endif

	LOG_INF("2 EVBs: controller side...");
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c0), okay)
	const struct device *controll_dev_1, *dummy_dev1, *dummy_dev2;
	controll_dev_1 = device_get_binding("i3c@f03c00");
	if (!controll_dev_1) {
		LOG_ERR("failed to get i3c device");
		return -ENODEV;
	}

	dummy_dev1 = device_get_binding("i3c_dummy_1");
	if (!dummy_dev1) {
		LOG_ERR("failed to get i3c_dummy_1 dev");
		return -ENODEV;
	}

	dummy_dev2 = device_get_binding("i3c_dummy_2");
	if (!dummy_dev2) {
		LOG_ERR("failed to get i3c_dummy_2 dev");
		return -ENODEV;
	}

#if TEST_BROADCAST_CCC
	cmd_i3c_ccc_rstdaa(controll_dev_1);
	i3c_do_daa(controll_dev_1);
#endif /* TEST_BROADCAST_CCC */

#if TEST_DIRECT_CCC
	cmd_i3c_ccc_getpid(controll_dev_1, dummy_dev1);
	cmd_i3c_ccc_getpid(controll_dev_1, dummy_dev2);

	i3c_ccc_do_getpid_multi_tdev(controll_dev_1, dummy_dev1, dummy_dev2);
	i3c_ccc_do_setmwl_multi_tdev(controll_dev_1, dummy_dev1, dummy_dev2);
	cmd_i3c_ccc_getmwl(controll_dev_1, dummy_dev1);
	cmd_i3c_ccc_getmwl(controll_dev_1, dummy_dev2);
#endif /* TEST_DIRECT_CCC */

#if TEST_ZEPHYR_API
	libi3c_write(controll_dev_1, dummy_dev1);
	k_sleep(K_MSEC(1));
	libi3c_read(controll_dev_1, dummy_dev1, NULL);
	k_sleep(K_MSEC(1));
	libi3c_reg_write_byte(controll_dev_1, dummy_dev1, 0xDD, 0xD0);
	k_sleep(K_MSEC(1));

	/* I3CS direct mode is unsupported */
	// libi3c_burst_write(controll_dev_1, dummy_dev1);

	/* I3CS doesn't support private write-then-read */
	// libi3c_write_read(controll_dev_1, dummy_dev1, NULL);
	// libi3c_burst_read(controll_dev_1, dummy_dev1, NULL);
	// libi3c_reg_read_byte(controll_dev_1, dummy_dev1, NULL);
	// libi3c_reg_update_byte(controll_dev_1, dummy_dev1, NULL, 0xEE, 0x0F, 0x0A);

	LOG_INF("finished zephyr i3c_transfer_api test");
#endif /* TEST_ZEPHYR_API */

#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(i3c0), okay)*/
#endif /* CONFIG_ITE_2EVB_TEST */
	return 0;
}
