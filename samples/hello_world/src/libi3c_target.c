/*
 * Copyright (c) 2025 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "libi3c_target.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(libi3c_target, LOG_LEVEL_DBG);

int libi3c_target_ibi_tir(const struct device *dev, uint8_t *buf, const uint16_t data_length)
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

	// LOG_INF("finished to issue ibi tir");
	return 0;
}

int libi3c_target_ibi_hj(const struct device *dev)
{
	struct i3c_ibi request;
	int ret;

	request.ibi_type = I3C_IBI_HOTJOIN;
	ret = i3c_ibi_raise(dev, &request);
	if (ret != 0) {
		LOG_ERR("unable to issue ibi hj");
		return ret;
	}

	LOG_INF("finished to issue ibi hj");
	return 0;
}
