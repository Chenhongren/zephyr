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
LOG_MODULE_REGISTER(i3c_sample, LOG_LEVEL_DBG);

/* Test with it513xx_i3cm_i3cs.overlay */
#define TEST_BROADCAST_CCC   1
#define TEST_DIRECT_CCC      1
#define TEST_PRIVATE_WRITE   1
#define TEST_PRIVATE_READ    1
#define TEST_TARGET_HOT_JOIN 1
#define TEST_TARGET_IBI      1
#define TEST_TARGET_TX_WRITE 1
#define TEST_ZEPHYR_API      1

int main(void)
{
	__maybe_unused uint8_t buf[16];
	__maybe_unused size_t write_ready_per_time = 7;
	__maybe_unused bool write = true;
	__maybe_unused bool without_7eh = true;

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
	if (!dummy_dev1) {
		LOG_ERR("failed to get i3c_dummy_2 dev");
		return -ENODEV;
	}
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c2), okay)
	const struct device *target_dev_1;
	target_dev_1 = device_get_binding("i3c@f03d00");
	if (!target_dev_1) {
		LOG_ERR("failed to bind target device i3c@f03d00");
		return -ENODEV;
	}
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c3), okay)
	const struct device *target_dev_2;
	target_dev_2 = device_get_binding("i3c@f03d80");
	if (!target_dev_2) {
		LOG_ERR("failed to bind target device i3c@f03d80");
		return -ENODEV;
	}
#endif

#if TEST_BROADCAST_CCC
	cmd_i3c_ccc_rstdaa(controll_dev_1);
	i3c_do_daa(controll_dev_1);
#endif /* TEST_BROADCAST_CCC */

#if TEST_DIRECT_CCC
	cmd_i3c_ccc_getpid(controll_dev_1, dummy_dev1);
	cmd_i3c_ccc_getpid(controll_dev_1, dummy_dev2);

	i3c_ccc_do_getpid_multi_tdev(controll_dev_1, dummy_dev1, dummy_dev2);
	i3c_ccc_do_setmwl_multi_tdev(controll_dev_1, dummy_dev1, dummy_dev2);
#endif /* TEST_DIRECT_CCC */

#if TEST_PRIVATE_WRITE
	for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
		i3c_write_read_nbytes(controll_dev_1, dummy_dev1, write, write_ready_per_time, bios,
				      without_7eh);
	}
	without_7eh = false;
	for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
		i3c_write_read_nbytes(controll_dev_1, dummy_dev1, write, write_ready_per_time, bios,
				      without_7eh);
	}
	without_7eh = true;
	for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
		i3c_write_read_nbytes(controll_dev_1, dummy_dev2, write, write_ready_per_time, bios,
				      without_7eh);
	}
	without_7eh = false;
	for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
		i3c_write_read_nbytes(controll_dev_1, dummy_dev2, write, write_ready_per_time, bios,
				      without_7eh);
	}
	without_7eh = true;
	for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
		i3c_write_read_nbytes_repeated_start(controll_dev_1, dummy_dev1, write,
						     write_ready_per_time, bios, without_7eh);
	}
	without_7eh = false;
	for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
		i3c_write_read_nbytes_repeated_start(controll_dev_1, dummy_dev1, write,
						     write_ready_per_time, bios, without_7eh);
	}
	without_7eh = true;
	for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
		i3c_write_read_nbytes_repeated_start(controll_dev_1, dummy_dev2, write,
						     write_ready_per_time, bios, without_7eh);
	}
	without_7eh = false;
	for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
		i3c_write_read_nbytes_repeated_start(controll_dev_1, dummy_dev2, write,
						     write_ready_per_time, bios, without_7eh);
	}

	LOG_INF("finished private write test");
	k_sleep(K_MSEC(1000));
#endif /* TEST_PRIVATE_WRITE */

#if TEST_PRIVATE_READ
	uint8_t count = 0xFF;

	write = false;
	without_7eh = true;
	for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
		/* perpare tx data at target side for reading */
		for (uint8_t j = 0; j < write_ready_per_time; j++) {
			buf[j] = 0xFF - count;
			count--;
		}
		i3c_target_tx_write(target_dev_1, buf, write_ready_per_time, 0);
		/* start reading */
		i3c_write_read_nbytes(controll_dev_1, dummy_dev1, write, write_ready_per_time, bios,
				      without_7eh);
	}
	without_7eh = false;
	for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
		/* perpare tx data at target side for reading */
		for (uint8_t j = 0; j < write_ready_per_time; j++) {
			buf[j] = 0xFF - count;
			count--;
		}
		i3c_target_tx_write(target_dev_1, buf, write_ready_per_time, 0);
		/* start reading */
		i3c_write_read_nbytes(controll_dev_1, dummy_dev1, write, write_ready_per_time, bios,
				      without_7eh);
	}
	without_7eh = true;
	for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
		/* perpare tx data at target side for reading */
		for (uint8_t j = 0; j < write_ready_per_time; j++) {
			buf[j] = 0xFF - count;
			count--;
		}
		i3c_target_tx_write(target_dev_2, buf, write_ready_per_time, 0);
		/* start reading */
		i3c_write_read_nbytes(controll_dev_1, dummy_dev2, write, write_ready_per_time, bios,
				      without_7eh);
	}
	without_7eh = false;
	for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
		/* perpare tx data at target side for reading */
		for (uint8_t j = 0; j < write_ready_per_time; j++) {
			buf[j] = 0xFF - count;
			count--;
		}
		i3c_target_tx_write(target_dev_2, buf, write_ready_per_time, 0);
		/* start reading */
		i3c_write_read_nbytes(controll_dev_1, dummy_dev2, write, write_ready_per_time, bios,
				      without_7eh);
	}

	// count = 0;
	// without_7eh = true;
	// for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
	// 	/* perpare tx data at target side for reading */
	// 	for (uint8_t j = 0; j < write_ready_per_time * 2; j++) {
	// 		buf[j] = count;
	// 		count++;
	// 	}
	// 	i3c_target_tx_write(target_dev_1, buf, write_ready_per_time * 2, 0);
	// 	/* start reading */
	// 	i3c_write_read_nbytes_repeated_start(controll_dev_1, dummy_dev1, write,
	// write_ready_per_time, bios, without_7eh);
	// }
	// without_7eh = false;
	// for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
	// 	/* perpare tx data at target side for reading */
	// 	for (uint8_t j = 0; j < write_ready_per_time * 2; j++) {
	// 		buf[j] = count;
	// 		count++;
	// 	}
	// 	i3c_target_tx_write(target_dev_1, buf, write_ready_per_time * 2, 0);
	// 	/* start reading */
	// 	i3c_write_read_nbytes_repeated_start(controll_dev_1, dummy_dev1, write,
	// write_ready_per_time, bios, 					     without_7eh);
	// }
	// without_7eh = true;
	// for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
	// 	/* perpare tx data at target side for reading */
	// 	for (uint8_t j = 0; j < write_ready_per_time * 2; j++) {
	// 		buf[j] = count;
	// 		count++;
	// 	}
	// 	i3c_target_tx_write(target_dev_2, buf, write_ready_per_time * 2, 0);
	// 	/* start reading */
	// 	i3c_write_read_nbytes_repeated_start(controll_dev_1, dummy_dev2, write,
	// write_ready_per_time, bios, 					     without_7eh);
	// }
	// without_7eh = false;
	// for (uint8_t i = 0, bios = 0; i < 3; i++, bios++) {
	// 	/* perpare tx data at target side for reading */
	// 	for (uint8_t j = 0; j < write_ready_per_time * 2; j++) {
	// 		buf[j] = count;
	// 		count++;
	// 	}
	// 	i3c_target_tx_write(target_dev_2, buf, write_ready_per_time * 2, 0);
	// 	/* start reading */
	// 	i3c_write_read_nbytes_repeated_start(controll_dev_1, dummy_dev2, write,
	// write_ready_per_time, bios, 					     without_7eh);
	// }
	LOG_INF("finished private read test");
	k_sleep(K_MSEC(1000));
#endif /* TEST_PRIVATE_READ */

#if TEST_TARGET_HOT_JOIN
	/* reset daa to issue hot-join */
	cmd_i3c_ccc_rstdaa(controll_dev_1);

	libi3c_target_ibi_hj(target_dev_1);
	LOG_INF("finished hot-join test");
	k_sleep(K_MSEC(1000));
#endif /* TEST_TARGET_HOT_JOIN */

#if TEST_TARGET_IBI
	uint16_t data_length = 2;

	buf[0] = 0xAA;
	buf[1] = 0xBB;

	libi3c_target_ibi_tir(target_dev_1, buf, data_length);
	LOG_INF("finished ibi test");
	k_sleep(K_MSEC(1000));
#endif /* TEST_TARGET_IBI */

/* TODO: need to check IBI TIR? */
#if TEST_TARGET_TX_WRITE
	for (int i = 0; i < 16; i++) {
		buf[i] = 0xFF - i;
	}
	i3c_target_tx_write(target_dev_1, buf, sizeof(buf), 0);
	LOG_INF("finished target tx write test");
	k_sleep(K_MSEC(1000));
#endif /* TEST_TARGET_TX_WRITE */

#if TEST_ZEPHYR_API
	libi3c_write(controll_dev_1, dummy_dev1);
	libi3c_read(controll_dev_1, dummy_dev1, target_dev_1);
	libi3c_write_read(controll_dev_1, dummy_dev1, target_dev_1);
	libi3c_burst_read(controll_dev_1, dummy_dev1, target_dev_1);
	libi3c_burst_write(controll_dev_1, dummy_dev1);
	// libi3c_reg_read_byte(controll_dev_1, dummy_dev1, target_dev_1);
	// libi3c_reg_write_byte(controll_dev_1, dummy_dev1);
	// libi3c_reg_update_byte(controll_dev_1, dummy_dev1, target_dev_1);

	LOG_INF("finished zephyr i3c_transfer_api test");
	k_sleep(K_MSEC(1000));
#endif /* TEST_ZEPHYR_API */

	return 0;
}
