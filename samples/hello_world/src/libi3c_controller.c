/*
 * Copyright (c) 2025 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "libi3c_controller.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(libi3c_controller, LOG_LEVEL_DBG);

struct i3c_device_desc *get_i3c_attached_desc_from_dev_name(const struct device *dev,
							    const char *tdev_name)
{
	struct i3c_device_desc *i3c_desc;

	I3C_BUS_FOR_EACH_I3CDEV(dev, i3c_desc) {
		/* only look for a device with the same name */
		if (strcmp(i3c_desc->dev->name, tdev_name) == 0) {
			return i3c_desc;
		}
	}

	return NULL;
}

int i3c_get_tdev_desc(const struct device *dev, const struct device *tdev,
		      struct i3c_device_desc **desc)
{
	*desc = get_i3c_attached_desc_from_dev_name(dev, tdev->name);
	if (!*desc) {
		LOG_ERR("I3C: Device %s not attached to bus.", tdev->name);
		return -ENODEV;
	}

	return 0;
}

int cmd_i3c_ccc_rstdaa(const struct device *dev)
{
	struct i3c_device_desc *desc;
	int ret;

	ret = i3c_ccc_do_rstdaa_all(dev);
	if (ret < 0) {
		LOG_ERR("failed to send broadcast ccc rstdaa");
		return ret;
	}

	/* reset all devices DA */
	I3C_BUS_FOR_EACH_I3CDEV(dev, desc) {
		desc->dynamic_addr = 0;
		LOG_INF("reseted dynamic address for device %s", desc->dev->name);
	}

	return ret;
}

int cmd_i3c_ccc_getpid(const struct device *dev, const struct device *tdev)
{
	struct i3c_device_desc *desc;
	struct i3c_ccc_getpid pid;
	int ret;

	ret = i3c_get_tdev_desc(dev, tdev, &desc);
	if (ret != 0) {
		return ret;
	}

	ret = i3c_ccc_do_getpid(desc, &pid);
	if (ret < 0) {
		LOG_ERR("I3C: unable to send CCC GETPID.");
		return ret;
	}

	LOG_INF("%s pid: 0x%012llx", tdev->name, sys_get_be48(pid.pid));

	return ret;
}

int cmd_i3c_ccc_getbcr(const struct device *dev, const struct device *tdev)
{
	struct i3c_device_desc *desc;
	struct i3c_ccc_getbcr bcr;
	int ret;

	ret = i3c_get_tdev_desc(dev, tdev, &desc);
	if (ret != 0) {
		return ret;
	}

	ret = i3c_ccc_do_getbcr(desc, &bcr);
	if (ret < 0) {
		LOG_ERR("I3C: unable to send CCC GETBCR.");
		return ret;
	}

	LOG_INF("%s bcr: 0x%02x", tdev->name, bcr.bcr);

	return ret;
}

int i3c_ccc_do_getpid_multi_tdev(const struct device *dev, const struct device *tdev1,
				 const struct device *tdev2)
{
	struct i3c_ccc_payload ccc_payload;
	struct i3c_ccc_target_payload ccc_tgt_payload[4];
	struct i3c_ccc_getpid pid[4];
	struct i3c_device_desc *target1, *target2;
	int ret;

	ret = i3c_get_tdev_desc(dev, tdev1, &target1);
	if (ret != 0) {
		return ret;
	}

	ret = i3c_get_tdev_desc(dev, tdev2, &target2);
	if (ret != 0) {
		return ret;
	}

	ccc_tgt_payload[0].addr = target1->dynamic_addr;
	ccc_tgt_payload[0].rnw = 1;
	ccc_tgt_payload[0].data = &pid[0].pid[0];
	ccc_tgt_payload[0].data_len = sizeof(pid[0].pid);

	ccc_tgt_payload[1].addr = target2->dynamic_addr;
	ccc_tgt_payload[1].rnw = 1;
	ccc_tgt_payload[1].data = &pid[1].pid[0];
	ccc_tgt_payload[1].data_len = sizeof(pid[1].pid);

	ccc_tgt_payload[2].addr = target1->dynamic_addr;
	ccc_tgt_payload[2].rnw = 1;
	ccc_tgt_payload[2].data = &pid[2].pid[0];
	ccc_tgt_payload[2].data_len = sizeof(pid[2].pid);

	ccc_tgt_payload[3].addr = target2->dynamic_addr;
	ccc_tgt_payload[3].rnw = 1;
	ccc_tgt_payload[3].data = &pid[3].pid[0];
	ccc_tgt_payload[3].data_len = sizeof(pid[3].pid);

	memset(&ccc_payload, 0, sizeof(ccc_payload));
	ccc_payload.ccc.id = I3C_CCC_GETPID;
	ccc_payload.targets.payloads = ccc_tgt_payload;
	ccc_payload.targets.num_targets = 4;

	ret = i3c_do_ccc(target1->bus, &ccc_payload);
	if (!ret) {
		LOG_INF("%s pid: 0x%012llx, %s pid: 0x%012llx", tdev1->name,
			sys_get_be48(pid[0].pid), tdev2->name, sys_get_be48(pid[1].pid));
		LOG_INF("%s pid: 0x%012llx, %s pid: 0x%012llx", tdev1->name,
			sys_get_be48(pid[2].pid), tdev2->name, sys_get_be48(pid[3].pid));
	}
	return ret;
}

int cmd_i3c_ccc_getmwl(const struct device *dev, const struct device *tdev)
{
	struct i3c_device_desc *desc;
	struct i3c_ccc_mwl mwl;
	int ret;

	ret = i3c_get_tdev_desc(dev, tdev, &desc);
	if (ret != 0) {
		return ret;
	}

	ret = i3c_ccc_do_getmwl(desc, &mwl);
	if (ret < 0) {
		LOG_INF("I3C: unable to send CCC GETMWL");
		return ret;
	}

	LOG_INF("%s mwl: 0x%04x", tdev->name, mwl.len);
	desc->data_length.mwl = mwl.len;

	return ret;
}

int i3c_ccc_do_setmwl_multi_tdev(const struct device *dev, const struct device *tdev1,
				 const struct device *tdev2)
{
	struct i3c_ccc_payload ccc_payload;
	struct i3c_ccc_target_payload ccc_tgt_payload[2];
	uint8_t data1[2], data2[2];
	struct i3c_device_desc *target1, *target2;
	struct i3c_ccc_mwl mwl[2];
	int ret;

	ret = i3c_get_tdev_desc(dev, tdev1, &target1);
	if (ret != 0) {
		return ret;
	}

	ret = i3c_get_tdev_desc(dev, tdev2, &target2);
	if (ret != 0) {
		return ret;
	}

	memset(&ccc_payload, 0, sizeof(ccc_payload));

	ccc_tgt_payload[0].addr = target1->dynamic_addr;
	ccc_tgt_payload[0].rnw = 0;
	ccc_tgt_payload[0].data = &data1[0];
	ccc_tgt_payload[0].data_len = sizeof(data1);

	ccc_tgt_payload[1].addr = target2->dynamic_addr;
	ccc_tgt_payload[1].rnw = 0;
	ccc_tgt_payload[1].data = &data2[0];
	ccc_tgt_payload[1].data_len = sizeof(data2);

	ccc_payload.ccc.id = I3C_CCC_SETMWL(false);
	ccc_payload.targets.payloads = &ccc_tgt_payload[0];
	ccc_payload.targets.num_targets = 2;

	mwl[0].len = 0x1AA;
	mwl[1].len = 0x2BB;
	/* The actual length is MSB first. So order the data. */
	sys_put_be16(mwl[0].len, data1);
	sys_put_be16(mwl[1].len, data2);

	ret = i3c_do_ccc(target1->bus, &ccc_payload);
	if (!ret) {
		LOG_INF("finish set multi mwl");
	}
	return ret;
}

int i3c_write_read_without_7eh(struct i3c_device_desc *target, const bool write, const uint8_t *buf,
			       uint32_t num_bytes)
{
	struct i3c_msg msg;

	msg.buf = (uint8_t *)buf;
	msg.len = num_bytes;
	msg.flags = write ? (I3C_MSG_WRITE | I3C_MSG_NBCH | I3C_MSG_STOP)
			  : (I3C_MSG_READ | I3C_MSG_NBCH | I3C_MSG_STOP);
	msg.hdr_mode = 0;
	msg.hdr_cmd_code = 0;

	return i3c_transfer(target, &msg, 1);
}

int i3c_write_read_nbytes(const struct device *dev, const struct device *tdev, const bool write,
			  const size_t nbytes, const uint8_t bios, const bool without_7eh)
{
	struct i3c_device_desc *target;
	uint8_t buf[64] = {0};
	int ret;

	ret = i3c_get_tdev_desc(dev, tdev, &target);
	if (ret != 0) {
		return ret;
	}

	if (write) {
		for (size_t i = 0; i < nbytes; i++) {
			buf[i] = i + bios * 7;
		}

		if (without_7eh) {
			ret = i3c_write_read_without_7eh(target, write, buf, nbytes);
		} else {
			ret = i3c_write(target, buf, nbytes);
		}
	} else {
		if (without_7eh) {
			ret = i3c_write_read_without_7eh(target, write, buf, nbytes);
		} else {
			ret = i3c_read(target, buf, nbytes);
		}
		LOG_HEXDUMP_INF(buf, nbytes, "rx:");
	}

	if (ret < 0) {
		LOG_ERR("failed to %s to device: %s", write ? "write" : "read", tdev->name);
		return -EIO;
	}
	return 0;
}

int i3c_write_repeated_start(struct i3c_device_desc *target, const uint8_t *buf1,
			     const uint32_t num_bytes1, const uint8_t *buf2,
			     const uint32_t num_bytes2, const bool without_7eh)
{
	struct i3c_msg msg[2];

	msg[0].buf = (uint8_t *)buf1;
	msg[0].len = num_bytes1;
	msg[0].flags = without_7eh ? (I3C_MSG_WRITE | I3C_MSG_NBCH) : (I3C_MSG_WRITE);
	msg[0].hdr_mode = 0;
	msg[0].hdr_cmd_code = 0;

	msg[1].buf = (uint8_t *)buf2;
	msg[1].len = num_bytes2;
	msg[1].flags = without_7eh ? (I3C_MSG_WRITE | I3C_MSG_NBCH | I3C_MSG_STOP)
				   : (I3C_MSG_WRITE | I3C_MSG_STOP);
	msg[1].hdr_mode = 0;
	msg[1].hdr_cmd_code = 0;

	return i3c_transfer(target, msg, 2);
}

int i3c_read_repeated_start(struct i3c_device_desc *target, const uint8_t *buf1,
			    const uint32_t num_bytes1, const uint8_t *buf2,
			    const uint32_t num_bytes2, const bool without_7eh)
{
	struct i3c_msg msg[2];

	msg[0].buf = (uint8_t *)buf1;
	msg[0].len = num_bytes1;
	msg[0].flags = without_7eh ? (I3C_MSG_READ | I3C_MSG_NBCH | I3C_MSG_RESTART)
				   : (I3C_MSG_READ | I3C_MSG_RESTART);
	msg[0].hdr_mode = 0;
	msg[0].hdr_cmd_code = 0;

	msg[1].buf = (uint8_t *)buf2;
	msg[1].len = num_bytes2;
	msg[1].flags = without_7eh ? (I3C_MSG_READ | I3C_MSG_NBCH | I3C_MSG_STOP)
				   : (I3C_MSG_READ | I3C_MSG_STOP);
	msg[1].hdr_mode = 0;
	msg[1].hdr_cmd_code = 0;

	return i3c_transfer(target, msg, 2);
}

int i3c_write_read_nbytes_repeated_start(const struct device *dev, const struct device *tdev,
					 const bool write, const size_t nbytes, const uint8_t bios,
					 const bool without_7eh)
{
	struct i3c_device_desc *target;
	uint8_t buf1[64] = {0}, buf2[64] = {0};
	int ret;

	ret = i3c_get_tdev_desc(dev, tdev, &target);
	if (ret != 0) {
		return ret;
	}

	if (write) {
		for (size_t i = 0; i < nbytes; i++) {
			buf1[i] = i + bios * 7;
		}

		for (size_t i = 0; i < nbytes + 1; i++) {
			buf2[i] = 0xff - (i + bios * 7);
		}

		ret = i3c_write_repeated_start(target, buf1, nbytes, buf2, nbytes + 1, without_7eh);
	} else {
		ret = i3c_read_repeated_start(target, buf1, nbytes, buf2, nbytes, without_7eh);
		LOG_HEXDUMP_INF(buf1, nbytes, "rx-1 :");
		LOG_HEXDUMP_INF(buf2, nbytes, "rx-2 :");
	}

	if (ret < 0) {
		LOG_ERR("failed to write to device: %s", tdev->name);
		return -EIO;
	}
	return 0;
}

int libi3c_write(const struct device *controller_dev, const struct device *dummy_dev)
{
	int ret;
	struct i3c_device_desc *target;
	uint8_t tx_buf[10];

	ret = i3c_get_tdev_desc(controller_dev, dummy_dev, &target);
	if (ret != 0) {
		LOG_ERR("failed to get i3c target describing");
		return ret;
	}

	for (uint32_t i = 0; i < sizeof(tx_buf); i++) {
		tx_buf[i] = i;
	}

	ret = i3c_write(target, tx_buf, sizeof(tx_buf));
	if (ret) {
		LOG_ERR("failed to perform i3c write");
		return ret;
	}

	return 0;
}

int libi3c_read(const struct device *controller_dev, const struct device *dummy_dev,
		const struct device *target_dev)
{
	int ret;
	struct i3c_device_desc *target;
	uint8_t tgt_buf[10];
	uint8_t rx_buf[10];
	uint8_t hdr_mode = 0;

	if (sizeof(tgt_buf) != sizeof(rx_buf)) {
		LOG_ERR("tgt and rx buffer size should be same");
		return -EINVAL;
	}

	ret = i3c_get_tdev_desc(controller_dev, dummy_dev, &target);
	if (ret != 0) {
		LOG_ERR("failed to get i3c target describing");
		return ret;
	}

	if (target_dev) {
		/* prepare data for controller read */
		for (uint32_t i = 0; i < sizeof(tgt_buf); i++) {
			tgt_buf[i] = i;
		}
		i3c_target_tx_write(target_dev, tgt_buf, sizeof(tgt_buf), hdr_mode);
	}

	ret = i3c_read(target, rx_buf, sizeof(rx_buf));
	if (ret) {
		LOG_ERR("failed to perform i3c read");
		return ret;
	}

	if (target_dev) {
		ret = memcmp(tgt_buf, rx_buf, sizeof(rx_buf));
		if (ret) {
			LOG_ERR("%s: received data is error", __func__);
			return -EINVAL;
		}
	}

	return 0;
}

int libi3c_write_read(const struct device *controller_dev, const struct device *dummy_dev,
		      const struct device *target_dev)
{
	int ret;
	struct i3c_device_desc *target;
	uint8_t tgt_buf[10];
	uint8_t tx_buf[10];
	uint8_t rx_buf[10];
	uint8_t hdr_mode = 0;

	if (sizeof(tgt_buf) != sizeof(rx_buf)) {
		LOG_ERR("tgt and rx buffer size should be same");
		return -EINVAL;
	}

	ret = i3c_get_tdev_desc(controller_dev, dummy_dev, &target);
	if (ret != 0) {
		LOG_ERR("failed to get i3c target describing");
		return ret;
	}

	if (target_dev) {
		/* prepare data for controller read */
		for (uint32_t i = 0; i < sizeof(tgt_buf); i++) {
			tgt_buf[i] = i;
		}
		i3c_target_tx_write(target_dev, tgt_buf, sizeof(tgt_buf), hdr_mode);
	}

	for (uint32_t i = 0; i < sizeof(tx_buf); i++) {
		tx_buf[i] = 0xff - i;
	}

	ret = i3c_write_read(target, tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf));
	if (ret) {
		LOG_ERR("failed to perform i3c write read");
		return ret;
	}

	if (target_dev) {
		ret = memcmp(tgt_buf, rx_buf, sizeof(rx_buf));
		if (ret) {
			LOG_ERR("%s: received data is error", __func__);
			return -EINVAL;
		}
	}

	return 0;
}

int libi3c_burst_read(const struct device *controller_dev, const struct device *dummy_dev,
		      const struct device *target_dev)
{
	int ret;
	struct i3c_device_desc *target;
	uint8_t start_addr = 0xAA;
	uint8_t tgt_buf[10];
	uint8_t rx_buf[10];
	uint8_t hdr_mode = 0;

	if (sizeof(tgt_buf) != sizeof(rx_buf)) {
		LOG_ERR("tgt and rx buffer size should be same");
		return -EINVAL;
	}

	ret = i3c_get_tdev_desc(controller_dev, dummy_dev, &target);
	if (ret != 0) {
		LOG_ERR("failed to get i3c target describing");
		return ret;
	}

	if (target_dev) {
		/* prepare data for controller read */
		for (uint32_t i = 0; i < sizeof(tgt_buf); i++) {
			tgt_buf[i] = i;
		}
		i3c_target_tx_write(target_dev, tgt_buf, sizeof(tgt_buf), hdr_mode);
	}

	ret = i3c_burst_read(target, start_addr, rx_buf, sizeof(rx_buf));
	if (ret) {
		LOG_ERR("failed to perform i3c burst read");
		return ret;
	}

	if (target_dev) {
		ret = memcmp(tgt_buf, rx_buf, sizeof(rx_buf));
		if (ret) {
			LOG_ERR("%s: received data is error", __func__);
			return -EINVAL;
		}
	}

	return 0;
}

int libi3c_burst_write(const struct device *controller_dev, const struct device *dummy_dev)
{
	int ret;
	struct i3c_device_desc *target;
	uint8_t start_addr = 0xBB;
	uint8_t tx_buf[10];

	ret = i3c_get_tdev_desc(controller_dev, dummy_dev, &target);
	if (ret != 0) {
		LOG_ERR("failed to get i3c target describing");
		return ret;
	}

	for (uint32_t i = 0; i < sizeof(tx_buf); i++) {
		tx_buf[i] = i;
	}

	ret = i3c_burst_write(target, start_addr, tx_buf, sizeof(tx_buf));
	if (ret) {
		LOG_ERR("failed to perform i3c burst write");
		return ret;
	}

	return 0;
}

int libi3c_reg_read_byte(const struct device *controller_dev, const struct device *dummy_dev,
			 const struct device *target_dev)
{
	int ret;
	struct i3c_device_desc *target;
	uint8_t reg_addr = 0xCC;
	uint8_t reg_value;
	uint8_t tgt_buf = 0xC0;
	uint8_t hdr_mode = 0;

	ret = i3c_get_tdev_desc(controller_dev, dummy_dev, &target);
	if (ret != 0) {
		LOG_ERR("failed to get i3c target describing");
		return ret;
	}

	if (target_dev) {
		i3c_target_tx_write(target_dev, &tgt_buf, 1, hdr_mode);
	}

	ret = i3c_reg_read_byte(target, reg_addr, &reg_value);
	if (ret) {
		LOG_ERR("failed to perform i3c reg read byte");
		return ret;
	}

	if (target_dev) {
		if (reg_value != tgt_buf) {
			LOG_ERR("%s: failed to read byte", __func__);
			return -EIO;
		}
	}

	return 0;
}

int libi3c_reg_write_byte(const struct device *controller_dev, const struct device *dummy_dev)
{
	int ret;
	struct i3c_device_desc *target;
	uint8_t reg_addr = 0xDD;
	uint8_t reg_value = 0xD0;

	ret = i3c_get_tdev_desc(controller_dev, dummy_dev, &target);
	if (ret != 0) {
		LOG_ERR("failed to get i3c target describing");
		return ret;
	}

	ret = i3c_reg_write_byte(target, reg_addr, reg_value);
	if (ret) {
		LOG_ERR("failed to perform i3c reg write byte");
		return ret;
	}

	return 0;
}

int libi3c_reg_update_byte(const struct device *controller_dev, const struct device *dummy_dev,
			   const struct device *target_dev)
{
	int ret;
	struct i3c_device_desc *target;
	uint8_t reg_addr = 0xEE;
	uint8_t tgt_buf = 0xE0;
	uint8_t hdr_mode = 0;

	ret = i3c_get_tdev_desc(controller_dev, dummy_dev, &target);
	if (ret != 0) {
		LOG_ERR("failed to get i3c target describing");
		return ret;
	}

	if (target_dev) {
		i3c_target_tx_write(target_dev, &tgt_buf, 1, hdr_mode);
	}

	/* mask = 0x0F, value 0x0A
	 * this means set bit[3:0] = Ah
	 */
	ret = i3c_reg_update_byte(target, reg_addr, 0x0F, 0x0A);
	if (ret) {
		LOG_ERR("failed to perform i3c reg update byte");
		return ret;
	}

	return 0;
}
