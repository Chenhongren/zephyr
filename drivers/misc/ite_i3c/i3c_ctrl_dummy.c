/*
 * Copyright (c) 2025 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_i3c_ctrl_dummy

#include <zephyr/drivers/i3c.h>

#include <zephyr/logging/log.h>
#define LOG_MODULE_NAME i3c_ctrl_dummy
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

#define IBI_MDB_GROUP_MASK              GENMASK(7, 5)
#define IBI_MDB_GROUP_PENDING_READ_NOTI 5

#define MAX_READ_DATA_LEN (CONFIG_I3CM_IT51XXX_DLM_SIZE / 2)

enum opcode_table {
	ITE_PRIVATE_READ_OPCODE = 0x0A,
};

struct i3c_ctrl_dummy_config {
	const struct device *bus;
	const uint64_t pid;
};

struct i3c_ctrl_dummy_data {
	const struct device *dev;

	struct i3c_device_desc *desc;
	struct k_work read_work; /* private read work */
	uint8_t buf[MAX_READ_DATA_LEN];

	struct k_work_delayable status_check_work; /* delayable work to check the target's status */
	bool is_present;
};

static void i3c_status_check_work(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct i3c_ctrl_dummy_data *data =
		CONTAINER_OF(dwork, struct i3c_ctrl_dummy_data, status_check_work);
	const struct device *dev = data->dev;
	union i3c_ccc_getstatus status;
	int ret;

	if (!data->is_present) {
		if (data->desc->dynamic_addr) {
			LOG_INF("device %s is joined", dev->name);
#ifdef CONFIG_I3C_USE_IBI
			if (i3c_device_is_ibi_capable(data->desc)) {
				if (!i3c_ibi_enable(data->desc)) {
					LOG_INF("enabled ibi for device %s", dev->name);
				}
			}
#endif /* CONFIG_I3C_USE_IBI */
			data->is_present = true;
		} else {
			goto out;
		}
	}

	/* in accordance with the i3c spec,
	 * If the target detaches or loses power from the I3C Bus, the controller may not be aware
	 * of it. To detect this condition, the controller sends a GETSTATUS CCC.
	 */
	ret = i3c_ccc_do_getstatus(data->desc, &status, GETSTATUS_FORMAT_1,
				   GETSTATUS_FORMAT_2_INVALID);
	if (ret < 0) {
		if (data->is_present) {
			/* NOTE: do NOT disable ibi, or the hot-join event requested by the target
			 * will fail */
			LOG_INF("device %s is absent, recycle the dynamic address(0x%x)", dev->name,
				data->desc->dynamic_addr);
			data->desc->dynamic_addr = 0;
			data->is_present = false;
		}
		goto out;
	}

	/* According to the DMTF MCTP over I3C spec(DSP0233 v1.0.1):
	 * In polling mode (when IBIs are disabled), MCTP I3C Targets must return a non-zero value
	 * in the pending interrupt field of the GETSTATUS CCC response to indicate that at least
	 * one MCTP packet is available to be read.
	 */
	if (I3C_CCC_GETSTATUS_NUM_INT(status.fmt1.status)) {
		LOG_DBG("%ld pending interrupt(s)", I3C_CCC_GETSTATUS_NUM_INT(status.fmt1.status));
		k_work_submit(&data->read_work);
	}

out:
	k_work_reschedule(&data->status_check_work,
			  K_MSEC(CONFIG_I3C_CTRL_DUMMY_GETSTATUS_POLLING_MS));
}

static void i3c_ctrl_read_work(struct k_work *work)
{
	struct i3c_ctrl_dummy_data *data =
		CONTAINER_OF(work, struct i3c_ctrl_dummy_data, read_work);
	struct i3c_msg msg;
	int ret;

	msg.buf = data->buf;
	msg.len = MAX_READ_DATA_LEN;
	msg.flags = I3C_MSG_READ | I3C_MSG_STOP;
	ret = i3c_transfer(data->desc, &msg, 1);
	if (ret == 0) {
		LOG_HEXDUMP_WRN(msg.buf, msg.num_xfer, "Pending/Private read data:");
	}
}

#ifdef CONFIG_I3C_USE_IBI
static int i3c_ctrl_dummy_ibi_cb(struct i3c_device_desc *target, struct i3c_ibi_payload *payload)
{
	const struct device *dev = target->dev;
	struct i3c_ctrl_dummy_data *data = dev->data;

	if (payload == NULL || payload->payload_len == 0) {
		LOG_INF("received ibi without payload");
	} else {
		if (payload->payload_len) {
			LOG_HEXDUMP_INF(payload->payload, payload->payload_len, "IBI payload:");

			if (FIELD_GET(IBI_MDB_GROUP_MASK, payload->payload[0]) ==
			    IBI_MDB_GROUP_PENDING_READ_NOTI) {
				k_work_submit(&data->read_work);
			} else if (payload->payload[0] == ITE_PRIVATE_READ_OPCODE) {
				k_work_submit(&data->read_work);
			}
		}
	}

	return 0;
}
#endif /* CONFIG_I3C_USE_IBI */

static int i3c_ctrl_dummy_init(const struct device *dev)
{
	const struct i3c_ctrl_dummy_config *config = dev->config;
	struct i3c_ctrl_dummy_data *data = dev->data;
	uint64_t pid = config->pid;
	const struct i3c_device_id i3c_id = I3C_DEVICE_ID(pid);

	LOG_INF("registered %s(pid %012llx) on %s controller", dev->name, config->pid,
		config->bus->name);

	data->dev = dev;
	data->desc = i3c_device_find(config->bus, &i3c_id);
#ifdef CONFIG_I3C_USE_IBI
	data->desc->ibi_cb = i3c_ctrl_dummy_ibi_cb;
#endif /* CONFIG_I3C_USE_IBI */
	if (i3c_device_is_ibi_capable(data->desc)) {
#ifdef CONFIG_I3C_USE_IBI
		if (!i3c_ibi_enable(data->desc)) {
			LOG_INF("enabled ibi for device %s", dev->name);
			data->is_present = true;
		}
#else
		LOG_INF("KCONFIG IBI is disabled");
#endif /* CONFIG_I3C_USE_IBI */
	}

	k_work_init(&data->read_work, i3c_ctrl_read_work);

	k_work_init_delayable(&data->status_check_work, i3c_status_check_work);
	k_work_reschedule(&data->status_check_work, K_MSEC(5));

	return 0;
}

#define I3C_CONTROLLER_DUMMY_INIT(n)                                                               \
	static const struct i3c_ctrl_dummy_config i3c_ctrl_dummy_config_##n = {                    \
		.bus = DEVICE_DT_GET(DT_INST_BUS(n)),                                              \
		.pid = ((uint64_t)DT_PROP_BY_IDX(DT_DRV_INST(n), reg, 1) << 32) |                  \
		       DT_PROP_BY_IDX(DT_DRV_INST(n), reg, 2),                                     \
	};                                                                                         \
                                                                                                   \
	static struct i3c_ctrl_dummy_data i3c_ctrl_dummy_data_##n;                                 \
	DEVICE_DT_INST_DEFINE(n, i3c_ctrl_dummy_init, NULL, &i3c_ctrl_dummy_data_##n,              \
			      &i3c_ctrl_dummy_config_##n, POST_KERNEL, 51, NULL);

DT_INST_FOREACH_STATUS_OKAY(I3C_CONTROLLER_DUMMY_INIT)
