/*
 * Copyright (c) 2026 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it51xxx_hw_rsa

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(it51xxx_hw_rsa, LOG_LEVEL_INF);

#include <soc_common.h>
#include <zephyr/drivers/misc/it51xxx_hw_rsa/it51xxx_hw_rsa.h>
#include <zephyr/pm/policy.h>

#define RSA40_CTRL             0x40
#define RSA_ENABLE             BIT(7)
#define RSA_RESET              BIT(6)
#define RSA_EXPONENT_MASK      GENMASK(5, 4)
#define RSA_EXPONENT_SELECT(n) FIELD_PREP(GENMASK(5, 4), n)
#define RSA_INTERRUPT_ENABLE   BIT(1)
#define RSA_CLOCK_ENABLE       BIT(0)

#define RSA41_CTRL_2             0x41
#define PUBLIC_MODULUS_LENGTH(n) (n / 32)

#define RSA42_STATUS          0x42
#define RSA_CALCULATION_START BIT(7)
#define RSA_CALCULATION_DONE  BIT(1)
#define RSA_BUSY              BIT(0)

#define RSA44_BASE_ADDR_BYTE_1 0x44
#define RSA_BASE_ADDR_LB(n)    FIELD_PREP(GENMASK(7, 4), n)

#define RSA45_BASE_ADDR_BYTE_2 0x45
#define RSA_BASE_ADDR_HB(n)    FIELD_PREP(GENMASK(1, 0), n)

enum it51xxx_rsa_key_select {
	key_public_exp_65537 = 0,
	key_public_exp_3,
	key_private,
	key_reserved,
};

struct it51xxx_dlm_base {
	mm_reg_t addr;
	size_t size;
};

struct it51xxx_hw_rsa_config {
	mm_reg_t base;

	struct {
		struct it51xxx_dlm_base key_private;
		struct it51xxx_dlm_base key_public;
		struct it51xxx_dlm_base messages;
	} dlm;

	void (*irq_config_func)(const struct device *dev);
};

struct it51xxx_hw_rsa_data {
	struct k_sem rsa_cal_done;
	struct k_mutex lock;
};

static void it51xxx_enable_standby_state(const struct device *dev, const bool enable)
{
	ARG_UNUSED(dev);

	if (enable) {
		chip_permit_idle();
		pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	} else {
		chip_block_idle();
		pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	}
}

static int it51xxx_hw_rsa_start(const struct device *dev)
{
	const struct it51xxx_hw_rsa_config *config = dev->config;
	struct it51xxx_hw_rsa_data *data = dev->data;
	int ret;

	sys_write8(RSA_CALCULATION_START, config->base + RSA42_STATUS);

	/* wait for completion */
	ret = k_sem_take(&data->rsa_cal_done, K_MSEC(CONFIG_IT51XXX_HW_RSA_CALCULATION_TIMEOUT_MS));

	if (ret) {
		LOG_ERR("hw rsa calculation timeout");
		return ret;
	}

	sys_write8(sys_read8(config->base + RSA42_STATUS) | RSA_CALCULATION_DONE,
		   config->base + RSA42_STATUS);
	sys_write8(sys_read8(config->base + RSA40_CTRL) & ~(RSA_ENABLE | RSA_CLOCK_ENABLE),
		   config->base + RSA40_CTRL);

	return 0;
}

static int it51xxx_hw_rsa_internal_encode(const struct device *dev,
					  struct it51xxx_hw_rsa_messages const *msgs)
{
	const struct it51xxx_hw_rsa_config *config = dev->config;
	struct it51xxx_hw_rsa_data *data = dev->data;
	int ret;
	uint8_t rsa_ctrl_val;

	if (!msgs->input || msgs->input_sz == 0) {
		LOG_ERR("null plaintext %zu", msgs->input_sz);
		return -EINVAL;
	}

	if (msgs->input_sz > config->dlm.messages.size ||
	    msgs->output_sz > config->dlm.messages.size) {
		LOG_ERR("plaintext/ciphertext size is overflow (%zu/%zu > %zu)", msgs->input_sz,
			msgs->output_sz, config->dlm.messages.size);
		return -ENOBUFS;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	it51xxx_enable_standby_state(dev, false);

	memset((void *)config->dlm.messages.addr, 0, config->dlm.messages.size);
	for (size_t i = 0; i < msgs->input_sz; i++) {
		sys_write8(msgs->input[msgs->input_sz - i - 1], config->dlm.messages.addr + i);
	}

	rsa_ctrl_val = sys_read8(config->base + RSA40_CTRL) & ~RSA_EXPONENT_MASK;
	rsa_ctrl_val |= RSA_ENABLE | RSA_CLOCK_ENABLE | RSA_EXPONENT_SELECT(key_private);
	sys_write8(rsa_ctrl_val, config->base + RSA40_CTRL);

	ret = it51xxx_hw_rsa_start(dev);
	if (ret) {
		goto out;
	}

	memcpy(msgs->output, (void *)config->dlm.messages.addr, msgs->output_sz);

	LOG_HEXDUMP_INF(msgs->output, msgs->output_sz, "encrypt:");

out:
	it51xxx_enable_standby_state(dev, true);
	k_mutex_unlock(&data->lock);

	return ret;
}

static int it51xxx_hw_rsa_internal_decode(const struct device *dev, const bool exponent_3,
					  struct it51xxx_hw_rsa_messages const *msgs)
{
	const struct it51xxx_hw_rsa_config *config = dev->config;
	struct it51xxx_hw_rsa_data *data = dev->data;
	int ret;
	uint8_t rsa_ctrl_val;

	if (!msgs->input || msgs->input_sz == 0) {
		LOG_ERR("null ciphertext %zu", msgs->input_sz);
		return -EINVAL;
	}

	if (msgs->input_sz > config->dlm.messages.size ||
	    msgs->output_sz > config->dlm.messages.size) {
		LOG_ERR("ciphertext/plaintext size is overflow (%zu/%zu > %zu)", msgs->input_sz,
			msgs->output_sz, config->dlm.messages.size);
		return -ENOBUFS;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	it51xxx_enable_standby_state(dev, false);

	memset((void *)config->dlm.messages.addr, 0, config->dlm.messages.size);
	memcpy((void *)config->dlm.messages.addr, msgs->input, msgs->input_sz);

	rsa_ctrl_val = sys_read8(config->base + RSA40_CTRL) & ~RSA_EXPONENT_MASK;
	rsa_ctrl_val |= RSA_ENABLE | RSA_CLOCK_ENABLE;
	rsa_ctrl_val |= exponent_3 ? RSA_EXPONENT_SELECT(key_public_exp_3)
				   : RSA_EXPONENT_SELECT(key_public_exp_65537);
	sys_write8(rsa_ctrl_val, config->base + RSA40_CTRL);

	ret = it51xxx_hw_rsa_start(dev);
	if (ret) {
		goto out;
	}

	for (size_t i = 0; i < msgs->output_sz; i++) {
		msgs->output[msgs->output_sz - i - 1] = sys_read8(config->dlm.messages.addr + i);
	}

	LOG_HEXDUMP_INF(msgs->output, msgs->output_sz, "decrypt:");

out:
	it51xxx_enable_standby_state(dev, true);
	k_mutex_unlock(&data->lock);

	return ret;
}

static int it51xxx_hw_rsa_internal_config(const struct device *dev,
					  const struct it51xxx_hw_rsa_keys keys)
{
	const struct it51xxx_hw_rsa_config *config = dev->config;
	struct it51xxx_hw_rsa_data *data = dev->data;
	size_t rsa_bits;

	if (!keys.private_key || !keys.public_key) {
		LOG_ERR("private/public key empty");
		return -EINVAL;
	}

	if (keys.private_key_sz != keys.public_key_sz) {
		LOG_ERR("private and public key size are different, %zu/%zu", keys.private_key_sz,
			keys.public_key_sz);
		return -EINVAL;
	}

	if (config->dlm.key_private.size < keys.private_key_sz) {
		LOG_ERR("private key is overflow, %d/%d", keys.private_key_sz,
			config->dlm.key_private.size);
		return -ENOBUFS;
	}

	if (config->dlm.key_public.size < keys.public_key_sz) {
		LOG_ERR("public key is overflow, %d/%d", keys.public_key_sz,
			config->dlm.key_public.size);
		return -ENOBUFS;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	rsa_bits = keys.private_key_sz * BITS_PER_BYTE;
	if (rsa_bits > 4096 || rsa_bits < 512) {
		LOG_ERR("unsupported ras key size %zu", rsa_bits);
		k_mutex_unlock(&data->lock);
		return -ENOTSUP;
	}

	sys_write8(PUBLIC_MODULUS_LENGTH(rsa_bits), config->base + RSA41_CTRL_2);

	memcpy((void *)config->dlm.key_private.addr, keys.private_key, keys.private_key_sz);
	memcpy((void *)config->dlm.key_public.addr, keys.public_key, keys.public_key_sz);

	k_mutex_unlock(&data->lock);
	LOG_INF("set private/public keys");

	return 0;
}

static void it51xxx_hw_rsa_isr(const struct device *dev)
{
	const struct it51xxx_hw_rsa_config *config = dev->config;
	struct it51xxx_hw_rsa_data *data = dev->data;
	uint8_t rsa_sts = sys_read8(config->base + RSA42_STATUS);

	if (rsa_sts & RSA_CALCULATION_DONE) {
		LOG_INF("isr: hw rsa calculation completed");
		k_sem_give(&data->rsa_cal_done);
		sys_write8(rsa_sts | RSA_CALCULATION_DONE, config->base + RSA42_STATUS);
	}
}

static int it51xxx_hw_rsa_init(const struct device *dev)
{
	const struct it51xxx_hw_rsa_config *config = dev->config;
	struct it51xxx_hw_rsa_data *data = dev->data;
	mm_reg_t dlm_base = MIN(config->dlm.key_public.addr,
				MIN(config->dlm.key_private.addr, config->dlm.messages.addr));

	k_sem_init(&data->rsa_cal_done, 0, 1);
	k_mutex_init(&data->lock);

	LOG_INF("dlm addr %#lx, public %#lx(%zu), messages %#lx(%zu), private %#lx(%zu)", dlm_base,
		config->dlm.key_public.addr, config->dlm.key_public.size, config->dlm.messages.addr,
		config->dlm.messages.size, config->dlm.key_private.addr,
		config->dlm.key_private.size);
	sys_write8(FIELD_GET(GENMASK(15, 8), dlm_base), config->base + RSA44_BASE_ADDR_BYTE_1);
	sys_write8(FIELD_GET(GENMASK(17, 16), dlm_base), config->base + RSA45_BASE_ADDR_BYTE_2);

	config->irq_config_func(dev);

	sys_write8(RSA_INTERRUPT_ENABLE, config->base + RSA40_CTRL);

	return 0;
}

static DEVICE_API(it51xxx_hw_rsa, it51xxx_hw_rsa_driver_api) = {
	.configure = it51xxx_hw_rsa_internal_config,
	.encrypt = it51xxx_hw_rsa_internal_encode,
	.decrypt = it51xxx_hw_rsa_internal_decode,
};

#define IT51XXX_HW_RSA_INIT(n)                                                                     \
	static void it51xxx_hw_rsa_config_func_##n(const struct device *dev)                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), 0, it51xxx_hw_rsa_isr, DEVICE_DT_INST_GET(n), 0);     \
		irq_enable(DT_INST_IRQN(n));                                                       \
	};                                                                                         \
                                                                                                   \
	static const struct it51xxx_hw_rsa_config hw_rsa_config_##n = {                            \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.dlm.key_public.addr = DT_INST_REG_ADDR_BY_IDX(n, 1),                              \
		.dlm.messages.addr = DT_INST_REG_ADDR_BY_IDX(n, 2),                                \
		.dlm.key_private.addr = DT_INST_REG_ADDR_BY_IDX(n, 3),                             \
		.dlm.key_public.size = DT_INST_REG_SIZE_BY_IDX(n, 1),                              \
		.dlm.messages.size = DT_INST_REG_SIZE_BY_IDX(n, 2),                                \
		.dlm.key_private.size = DT_INST_REG_SIZE_BY_IDX(n, 3),                             \
		.irq_config_func = it51xxx_hw_rsa_config_func_##n,                                 \
	};                                                                                         \
                                                                                                   \
	static struct it51xxx_hw_rsa_data hw_rsa_data_##n = {};                                    \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &it51xxx_hw_rsa_init, NULL, &hw_rsa_data_##n, &hw_rsa_config_##n, \
			      POST_KERNEL, 51, &it51xxx_hw_rsa_driver_api);

DT_INST_FOREACH_STATUS_OKAY(IT51XXX_HW_RSA_INIT)
