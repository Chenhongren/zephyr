/*
 * Copyright (c) 2025 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LIBI3C_CONTROLLER_H_
#define _LIBI3C_CONTROLLER_H_
#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/i3c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

struct i3c_device_desc *get_i3c_attached_desc_from_dev_name(const struct device *dev,
							    const char *tdev_name);
int i3c_get_tdev_desc(const struct device *dev, const struct device *tdev,
		      struct i3c_device_desc **desc);
int cmd_i3c_ccc_rstdaa(const struct device *dev);
int cmd_i3c_ccc_getpid(const struct device *dev, const struct device *tdev);
int cmd_i3c_ccc_getbcr(const struct device *dev, const struct device *tdev);
int i3c_ccc_do_getpid_multi_tdev(const struct device *dev, const struct device *tdev1,
				 const struct device *tdev2);
int cmd_i3c_ccc_getmwl(const struct device *dev, const struct device *tdev);
int i3c_ccc_do_setmwl_multi_tdev(const struct device *dev, const struct device *tdev1,
				 const struct device *tdev2);
int i3c_write_read_without_7eh(struct i3c_device_desc *target, const bool write, const uint8_t *buf,
			       uint32_t num_bytes);
int i3c_write_read_nbytes(const struct device *dev, const struct device *tdev, const bool write,
			  const size_t nbytes, const uint8_t bios, const bool without_7eh);
int i3c_write_repeated_start(struct i3c_device_desc *target, const uint8_t *buf1,
			     const uint32_t num_bytes1, const uint8_t *buf2,
			     const uint32_t num_bytes2, const bool without_7eh);
int i3c_read_repeated_start(struct i3c_device_desc *target, const uint8_t *buf1,
			    const uint32_t num_bytes1, const uint8_t *buf2,
			    const uint32_t num_bytes2, const bool without_7eh);
int i3c_write_read_nbytes_repeated_start(const struct device *dev, const struct device *tdev,
					 const bool write, const size_t nbytes, const uint8_t bios,
					 const bool without_7eh);

int libi3c_write(const struct device *controller_dev, const struct device *dummy_dev);
int libi3c_read(const struct device *controller_dev, const struct device *dummy_dev,
		const struct device *target_dev);
int libi3c_write_read(const struct device *controller_dev, const struct device *dummy_dev,
		      const struct device *target_dev);
int libi3c_burst_read(const struct device *controller_dev, const struct device *dummy_dev,
		      const struct device *target_dev);
int libi3c_burst_write(const struct device *controller_dev, const struct device *dummy_dev);
int libi3c_reg_read_byte(const struct device *controller_dev, const struct device *dummy_dev,
			 const struct device *target_dev);
int libi3c_reg_write_byte(const struct device *controller_dev, const struct device *dummy_dev);
int libi3c_reg_update_byte(const struct device *controller_dev, const struct device *dummy_dev,
			   const struct device *target_dev);

#endif /* _LIBI3C_CONTROLLER_H_ */
