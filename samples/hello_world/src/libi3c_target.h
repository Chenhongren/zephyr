/*
 * Copyright (c) 2025 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LIBI3C_TARGET_H_
#define _LIBI3C_TARGET_H_
#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/i3c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

int libi3c_target_ibi_tir(const struct device *dev, uint8_t *buf, const uint16_t data_length);
int libi3c_target_ibi_hj(const struct device *dev);

#endif /* _LIBI3C_TARGET_H_ */
