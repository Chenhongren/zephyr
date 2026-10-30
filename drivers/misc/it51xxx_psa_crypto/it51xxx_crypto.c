/*
 * Copyright (c) 2026 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "it51xxx_crypto.h"
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT ite_it51xxx_crypto

Z_GENERIC_SECTION(.__hwcrypto_dlm_block) union hwcrypto_dlm_block hwcrypto_dlm;

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "it51xxx: supported one crypto controller only");

struct it51xxx_crypto_config {
	mm_reg_t base;
};

struct it51xxx_crypto_data {
	struct k_mutex lock;
};

mm_reg_t it51xxx_crypto_get_base(const struct device *dev)
{
	const struct it51xxx_crypto_config *config = dev->config;

	return config->base;
}

int it51xxx_crypto_lock(const struct device *dev, k_timeout_t timeout)
{
	struct it51xxx_crypto_data *data = dev->data;

	return k_mutex_lock(&data->lock, timeout);
}

void it51xxx_crypto_unlock(const struct device *dev)
{
	struct it51xxx_crypto_data *data = dev->data;

	k_mutex_unlock(&data->lock);
}

static int it51xxx_crypto_init(const struct device *dev)
{
	struct it51xxx_crypto_data *data = dev->data;

	k_mutex_init(&data->lock);

	return 0;
}

#define IT51XXX_CRYPTO_INIT(n)                                                                     \
	static struct it51xxx_crypto_data it51xxx_crypto_data_##n;                                 \
	static const struct it51xxx_crypto_config it51xxx_crypto_config_##n = {                    \
		.base = DT_INST_REG_ADDR(n),                                                       \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, it51xxx_crypto_init, NULL, &it51xxx_crypto_data_##n,              \
			      &it51xxx_crypto_config_##n, PRE_KERNEL_1,                            \
			      CONFIG_IT51XXX_PSA_CRYPTO_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IT51XXX_CRYPTO_INIT)
