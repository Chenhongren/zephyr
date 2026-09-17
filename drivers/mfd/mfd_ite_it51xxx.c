/*
 * Copyright (c) 2026 ITE Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it51xxx_mfd

#include <zephyr/device.h>
#include <zephyr/drivers/mfd/ite_it51xxx.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mfd_ite_it51xxx, CONFIG_MFD_LOG_LEVEL);

#define HWCRYPTO01_STATUS    0x01
#define SHA_CALCULATION_DONE BIT(2)

#define HWCRYPTO42_STATUS    0x42
#define RSA_CALCULATION_DONE BIT(1)

Z_GENERIC_SECTION(.__hwcrypto_dlm_block) union hwcrypto_dlm_block hwcrypto_dlm;

struct mfd_ite_it51xxx_config {
	mm_reg_t base;
	void (*irq_config_func)(const struct device *dev);
};

struct mfd_ite_it51xxx_data {
	struct k_mutex lock;

	complete_cb_t sha_complete_cb;
	void *sha_complete_user_data;
	complete_cb_t rsa_complete_cb;
	void *rsa_complete_user_data;
};

void it51xxx_hwcrypto_register_sha_cb(const struct device *dev, complete_cb_t callback,
				      void *user_data)
{
	struct mfd_ite_it51xxx_data *data = dev->data;

	data->sha_complete_cb = callback;
	data->sha_complete_user_data = user_data;
}

void it51xxx_hwcrypto_register_rsa_cb(const struct device *dev, complete_cb_t callback,
				      void *user_data)
{
	struct mfd_ite_it51xxx_data *data = dev->data;

	data->rsa_complete_cb = callback;
	data->rsa_complete_user_data = user_data;
}

static void it51xxx_hwcrypto_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct mfd_ite_it51xxx_config *config = dev->config;
	struct mfd_ite_it51xxx_data *data = dev->data;
	uint8_t rsa_sts = sys_read8(config->base + HWCRYPTO42_STATUS);
	uint8_t sha_sts = sys_read8(config->base + HWCRYPTO01_STATUS);

	if (rsa_sts & RSA_CALCULATION_DONE) {
		LOG_DBG("isr: rsa completed");

		if (data->rsa_complete_cb != NULL) {
			data->rsa_complete_cb(data->rsa_complete_user_data, rsa_sts);
		}
		sys_write8(rsa_sts | RSA_CALCULATION_DONE, config->base + HWCRYPTO42_STATUS);
	}

	if (sha_sts & SHA_CALCULATION_DONE) {
		LOG_DBG("isr: sha completed");

		if (data->sha_complete_cb != NULL) {
			data->sha_complete_cb(data->sha_complete_user_data, sha_sts);
		}
		sys_write8(sha_sts | SHA_CALCULATION_DONE, config->base + HWCRYPTO01_STATUS);
	}
}

mm_reg_t mfd_ite_it51xxx_get_base(const struct device *dev)
{
	const struct mfd_ite_it51xxx_config *config = dev->config;

	return config->base;
}

int mfd_ite_it51xxx_lock(const struct device *dev, k_timeout_t timeout)
{
	struct mfd_ite_it51xxx_data *data = dev->data;

	return k_mutex_lock(&data->lock, timeout);
}

void mfd_ite_it51xxx_unlock(const struct device *dev)
{
	struct mfd_ite_it51xxx_data *data = dev->data;

	k_mutex_unlock(&data->lock);
}

static int mfd_ite_it51xxx_init(const struct device *dev)
{
	const struct mfd_ite_it51xxx_config *config = dev->config;
	struct mfd_ite_it51xxx_data *data = dev->data;

	k_mutex_init(&data->lock);

	config->irq_config_func(dev);

	return 0;
}

#define MFD_ITE_IT51XXX_DEFINE(n)                                                                  \
	static void mfd_ite_it51xxx_irq_config_func_##n(const struct device *dev)                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), 0, it51xxx_hwcrypto_isr, DEVICE_DT_INST_GET(n), 0);   \
		irq_enable(DT_INST_IRQN(n));                                                       \
	};                                                                                         \
	static struct mfd_ite_it51xxx_data mfd_ite_it51xxx_data_##n;                               \
	static const struct mfd_ite_it51xxx_config mfd_ite_it51xxx_config_##n = {                  \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.irq_config_func = mfd_ite_it51xxx_irq_config_func_##n,                            \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, mfd_ite_it51xxx_init, NULL, &mfd_ite_it51xxx_data_##n,            \
			      &mfd_ite_it51xxx_config_##n, PRE_KERNEL_1,                           \
			      CONFIG_MFD_ITE_IT51XXX_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MFD_ITE_IT51XXX_DEFINE)
