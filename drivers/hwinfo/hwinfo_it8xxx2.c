/*
 * Copyright (c) 2023 ITE Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/hwinfo.h>
#include <string.h>

struct it8xxx2_device_id {
	uint8_t chip_version;
	uint8_t chip_id[3];
};

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	struct it8xxx2_device_id dev_id;

	if (buffer == NULL) {
		return -EINVAL;
	}

	if (length > sizeof(dev_id)) {
		length = sizeof(dev_id);
	}

	dev_id.chip_version = IT8XXX2_GCTRL_ECHIPVER;
	dev_id.chip_id[0] = IT8XXX2_GCTRL_ECHIPID1;
	dev_id.chip_id[1] = IT8XXX2_GCTRL_ECHIPID2;
	dev_id.chip_id[2] = IT8XXX2_GCTRL_ECHIPID3;

	memcpy(buffer, &dev_id, sizeof(dev_id));

	return length;
}
