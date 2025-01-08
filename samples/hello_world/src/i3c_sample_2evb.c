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

/* only apply at the controller side */
#define TEST_BROADCAST_CCC 0
#define TEST_DIRECT_CCC    0
#define TEST_ZEPHYR_API    0

int main(void)
{

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
	libi3c_reg_write_byte(controll_dev_1, dummy_dev1);
	k_sleep(K_MSEC(1));

	/* I3CS direct mode is unsupported */
	// libi3c_burst_write(controll_dev_1, dummy_dev1);

	/* I3CS doesn't support private write-then-read */
	// libi3c_write_read(controll_dev_1, dummy_dev1, NULL);
	// libi3c_burst_read(controll_dev_1, dummy_dev1, NULL);
	// libi3c_reg_read_byte(controll_dev_1, dummy_dev1, NULL);
	// libi3c_reg_update_byte(controll_dev_1, dummy_dev1, NULL);

	LOG_INF("finished zephyr i3c_transfer_api test");
#endif /* TEST_ZEPHYR_API */

#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(i3c0), okay)*/

	return 0;
}
