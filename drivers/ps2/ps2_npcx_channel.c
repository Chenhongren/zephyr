/*
 * Copyright (c) 2021 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcx_ps2_channel

/**
 * @file
 * @brief Nuvoton NPCX PS/2 driver
 *
 * This file contains the driver of PS/2 buses (channels) which provides the
 * connection between Zephyr PS/2 API fucntions and NPCX PS/2 controller driver
 * to support PS/2 transactions.
 *
 */

#include <drivers/clock_control.h>
#include <drivers/ps2.h>
#include <soc.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(ps2_npcx_channel, CONFIG_PS2_LOG_LEVEL);

#include "ps2_npcx_controller.h"

/* Device config */
struct ps2_npcx_ch_config {
	/* pinmux configuration */
	const uint8_t alts_size;
	const struct npcx_alt *alts_list;
	/* Indicate the channel's number of the PS/2 channel device */
	uint8_t channel_id;
	const struct device *ps2_ctrl;
};

/* PS/2 api functions */
static int ps2_npcx_ch_configure(const struct device *dev,
				 ps2_callback_t callback_isr)
{
	const struct ps2_npcx_ch_config *const config = dev->config;
	int ret;

	ret = ps2_npcx_ctrl_configure(config->ps2_ctrl, config->channel_id,
				      callback_isr);
	if (ret != 0)
		return ret;

	return ps2_npcx_ctrl_enable_interface(config->ps2_ctrl,
					      config->channel_id, 1);
}

static int ps2_npcx_ch_write(const struct device *dev, uint8_t value)
{
	const struct ps2_npcx_ch_config *const config = dev->config;

	return ps2_npcx_ctrl_write(config->ps2_ctrl, config->channel_id, value);
}

static int ps2_npcx_ch_enable_interface(const struct device *dev)
{
	const struct ps2_npcx_ch_config *const config = dev->config;

	return ps2_npcx_ctrl_enable_interface(config->ps2_ctrl,
					      config->channel_id, 1);
}

static int ps2_npcx_ch_inhibit_interface(const struct device *dev)
{
	const struct ps2_npcx_ch_config *const config = dev->config;

	return ps2_npcx_ctrl_enable_interface(config->ps2_ctrl,
					      config->channel_id, 0);
}

/* PS/2 driver registration */
static int ps2_npcx_channel_init(const struct device *dev)
{
	const struct ps2_npcx_ch_config *const config = dev->config;

	if (!device_is_ready(config->ps2_ctrl)) {
		LOG_ERR("%s device not ready", config->ps2_ctrl->name);
		return -ENODEV;
	}

	/* Configure pin-mux for PS/2 device */
	npcx_pinctrl_mux_configure(config->alts_list, config->alts_size, 1);

	return 0;
}

static const struct ps2_driver_api ps2_channel_npcx_driver_api = {
	.config = ps2_npcx_ch_configure,
	.read = NULL,
	.write = ps2_npcx_ch_write,
	.disable_callback = ps2_npcx_ch_inhibit_interface,
	.enable_callback = ps2_npcx_ch_enable_interface,
};

/* PS/2 channel initialization macro functions */
#define NPCX_PS2_CHANNEL_INIT(inst)                                            \
									       \
	static const struct npcx_alt ps2_channel_alts##inst[] =                \
		NPCX_DT_ALT_ITEMS_LIST(inst);                                  \
									       \
	static const struct ps2_npcx_ch_config ps2_npcx_ch_cfg_##inst = {      \
		.channel_id = DT_INST_PROP(inst, channel),                     \
		.alts_size = ARRAY_SIZE(ps2_channel_alts##inst),               \
		.alts_list = ps2_channel_alts##inst,                           \
		.ps2_ctrl = DEVICE_DT_GET(DT_INST_PARENT(inst)),               \
	};                                                                     \
									       \
	DEVICE_DT_INST_DEFINE(inst, ps2_npcx_channel_init, NULL, NULL,         \
			      &ps2_npcx_ch_cfg_##inst, POST_KERNEL,            \
			      CONFIG_PS2_CHANNEL_INIT_PRIORITY,                \
			      &ps2_channel_npcx_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NPCX_PS2_CHANNEL_INIT)

/* PS/2 channel driver must be initialized after PS/2 controller driver */
BUILD_ASSERT(CONFIG_PS2_CHANNEL_INIT_PRIORITY > CONFIG_PS2_INIT_PRIORITY);
