/*
 * Copyright (c) 2026 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it51xxx_rsa

#include <soc_common.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/drivers/mfd/ite_it51xxx.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(crypto_it51xxx_rsa, CONFIG_CRYPTO_LOG_LEVEL);

#define RSA40_CTRL             0x40
#define RSA_ENABLE             BIT(7)
#define RSA_EXPONENT_MASK      GENMASK(5, 4)
#define RSA_EXPONENT_SELECT(n) FIELD_PREP(GENMASK(5, 4), n)
#define RSA_INTERRUPT_ENABLE   BIT(1)
#define RSA_CLOCK_ENABLE       BIT(0)

#define RSA41_CTRL_2                      0x41
#define RSA_PUBLIC_MODULUS_LENGTH_DIVISOR 32U
#define PUBLIC_MODULUS_LENGTH(n)          (n / RSA_PUBLIC_MODULUS_LENGTH_DIVISOR)

#define RSA42_STATUS          0x42
#define RSA_CALCULATION_START BIT(7)
#define RSA_CALCULATION_DONE  BIT(1)

#define RSA44_BASE_ADDR_BYTE_1 0x44
#define RSA_BASE_ADDR_LB(n)    FIELD_PREP(GENMASK(7, 4), n)

#define RSA45_BASE_ADDR_BYTE_2 0x45
#define RSA_BASE_ADDR_HB(n)    FIELD_PREP(GENMASK(1, 0), n)

#define rsa_dlm (hwcrypto_dlm.rsa)

enum it51xxx_rsa_key_select {
	KEY_PUBLIC_EXP_65537 = 0,
	KEY_PUBLIC_EXP_3,
	KEY_PRIVATE,
	KEY_RESERVED,
};

struct it51xxx_rsa_config {
	const struct device *mfd;
	void (*irq_config_func)(const struct device *dev);
};

struct it51xxx_rsa_data {
	struct k_sem rsa_cal_done;

	struct {
		uint8_t public[IT51XXX_HWCRYPTO_RSA_MAX_BYTE_LEN];
		uint8_t private[IT51XXX_HWCRYPTO_RSA_MAX_BYTE_LEN];
		size_t size;
	} keys;
};

static int it51xxx_start_rsa_calculation(const struct device *dev)
{
	const struct it51xxx_rsa_config *config = dev->config;
	const mm_reg_t base = mfd_ite_it51xxx_get_base(config->mfd);
	struct it51xxx_rsa_data *data = dev->data;

	sys_write8(RSA_CALCULATION_START, base + RSA42_STATUS);

	/* wait for completion */
	k_sem_take(&data->rsa_cal_done, K_FOREVER);

	sys_write8(sys_read8(base + RSA40_CTRL) & ~(RSA_ENABLE | RSA_CLOCK_ENABLE),
		   base + RSA40_CTRL);

	return 0;
}

static void it51xxx_rsa_set_keys(const struct device *dev)
{
	const struct it51xxx_rsa_config *config = dev->config;
	const mm_reg_t base = mfd_ite_it51xxx_get_base(config->mfd);
	struct it51xxx_rsa_data *data = dev->data;

	memset(rsa_dlm.key_private, 0, sizeof(rsa_dlm.key_private));
	memset(rsa_dlm.key_public, 0, sizeof(rsa_dlm.key_public));

	sys_write8(PUBLIC_MODULUS_LENGTH(data->keys.size * BITS_PER_BYTE), base + RSA41_CTRL_2);
	for (size_t i = 0; i < data->keys.size; i++) {
		rsa_dlm.key_private[i] = data->keys.private[data->keys.size - i - 1];
	}
	for (size_t i = 0; i < data->keys.size; i++) {
		rsa_dlm.key_public[i] = data->keys.public[data->keys.size - i - 1];
	}
}

static inline int validate_hash_and_sign(struct rsa_pkt *pkt)
{
	if (!pkt->hash || pkt->hash_length == 0) {
		LOG_ERR("null hash %zu", pkt->sign_length);
		return -EINVAL;
	}

	if (!pkt->sign || pkt->sign_length == 0) {
		LOG_ERR("null signature %zu", pkt->sign_length);
		return -EINVAL;
	}

	if (pkt->hash_length > sizeof(rsa_dlm.messages) ||
	    pkt->sign_length > sizeof(rsa_dlm.messages)) {
		LOG_ERR("hash/signature size is overflow (%zu/%zu > %zu)", pkt->hash_length,
			pkt->sign_length, sizeof(rsa_dlm.messages));
		return -ENOBUFS;
	}

	return 0;
}

static int it51xxx_rsa_encode(struct rsa_ctx *ctx, struct rsa_pkt *pkt)
{
	const struct device *dev = ctx->device;
	const struct it51xxx_rsa_config *config = dev->config;
	const mm_reg_t base = mfd_ite_it51xxx_get_base(config->mfd);
	int ret;
	uint8_t rsa_ctrl_val;

	ret = validate_hash_and_sign(pkt);
	if (ret) {
		return ret;
	}

	(void)mfd_ite_it51xxx_lock(config->mfd, K_FOREVER);

	chip_block_idle();

	it51xxx_rsa_set_keys(dev);

	memset(rsa_dlm.messages, 0, sizeof(rsa_dlm.messages));
	for (size_t i = 0; i < pkt->hash_length; i++) {
		rsa_dlm.messages[i] = pkt->hash[pkt->hash_length - i - 1];
	}

#if ITE_DEBUG
	LOG_HEXDUMP_DBG(rsa_dlm.key_public, sizeof(rsa_dlm.key_public), "B: dlm: key_public:");
	LOG_HEXDUMP_DBG(rsa_dlm.messages, sizeof(rsa_dlm.messages), "B: dlm: messages:");
	LOG_HEXDUMP_DBG(rsa_dlm.key_private, sizeof(rsa_dlm.key_private), "B: dlm: key_private:");
#endif

	rsa_ctrl_val = sys_read8(base + RSA40_CTRL) & ~RSA_EXPONENT_MASK;
	rsa_ctrl_val |= RSA_ENABLE | RSA_CLOCK_ENABLE | RSA_EXPONENT_SELECT(KEY_PRIVATE);
	sys_write8(rsa_ctrl_val, base + RSA40_CTRL);

	ret = it51xxx_start_rsa_calculation(dev);
	if (ret) {
		goto out;
	}

	for (size_t i = 0; i < pkt->sign_length; i++) {
		pkt->sign[pkt->sign_length - i - 1] = rsa_dlm.messages[i];
	}

	LOG_HEXDUMP_DBG(pkt->sign, pkt->sign_length, "encrypt:");

#if ITE_DEBUG
	LOG_HEXDUMP_DBG(rsa_dlm.key_public, sizeof(rsa_dlm.key_public), "A: dlm: key_public:");
	LOG_HEXDUMP_DBG(rsa_dlm.messages, sizeof(rsa_dlm.messages), "A: dlm: messages:");
	LOG_HEXDUMP_DBG(rsa_dlm.key_private, sizeof(rsa_dlm.key_private), "A: dlm: key_private:");
#endif

out:
	chip_permit_idle();
	mfd_ite_it51xxx_unlock(config->mfd);

	return ret;
}

static int it51xxx_rsa_decode(struct rsa_ctx *ctx, struct rsa_pkt *pkt, const bool exponent_3)
{
	const struct device *dev = ctx->device;
	const struct it51xxx_rsa_config *config = dev->config;
	const mm_reg_t base = mfd_ite_it51xxx_get_base(config->mfd);
	int ret;
	uint8_t rsa_ctrl_val;

	ret = validate_hash_and_sign(pkt);
	if (ret) {
		return ret;
	}

	(void)mfd_ite_it51xxx_lock(config->mfd, K_FOREVER);

	chip_block_idle();

	it51xxx_rsa_set_keys(dev);

	memset(rsa_dlm.messages, 0, sizeof(rsa_dlm.messages));
	for (size_t i = 0; i < pkt->sign_length; i++) {
		rsa_dlm.messages[i] = pkt->sign[pkt->sign_length - i - 1];
	}

	rsa_ctrl_val = sys_read8(base + RSA40_CTRL) & ~RSA_EXPONENT_MASK;
	rsa_ctrl_val |= RSA_ENABLE | RSA_CLOCK_ENABLE;
	rsa_ctrl_val |= exponent_3 ? RSA_EXPONENT_SELECT(KEY_PUBLIC_EXP_3)
				   : RSA_EXPONENT_SELECT(KEY_PUBLIC_EXP_65537);
	sys_write8(rsa_ctrl_val, base + RSA40_CTRL);

	ret = it51xxx_start_rsa_calculation(dev);
	if (ret) {
		goto out;
	}

	for (size_t i = 0; i < pkt->hash_length; i++) {
		pkt->hash[pkt->hash_length - i - 1] = rsa_dlm.messages[i];
	}

	LOG_HEXDUMP_DBG(pkt->hash, pkt->hash_length, "decrypt:");

out:
	chip_permit_idle();
	mfd_ite_it51xxx_unlock(config->mfd);

	return ret;
}

static int validate_rsa_key(const struct device *dev, const struct rsa_key keys)
{
	const struct it51xxx_rsa_config *config = dev->config;
	struct it51xxx_rsa_data *data = dev->data;
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

	if (sizeof(rsa_dlm.key_private) < keys.private_key_sz) {
		LOG_ERR("private key is overflow, %zu/%zu", keys.private_key_sz,
			sizeof(rsa_dlm.key_private));
		return -ENOBUFS;
	}

	if (sizeof(rsa_dlm.key_public) < keys.public_key_sz) {
		LOG_ERR("public key is overflow, %zu/%zu", keys.public_key_sz,
			sizeof(rsa_dlm.key_public));
		return -ENOBUFS;
	}

	if (keys.private_key_sz > IT51XXX_HWCRYPTO_RSA_MAX_BYTE_LEN ||
	    keys.private_key_sz < IT51XXX_HWCRYPTO_RSA_MIN_BYTE_LEN) {
		LOG_ERR("unsupported rsa key size %zu (bytes)", keys.private_key_sz);
		return -ENOTSUP;
	}

	rsa_bits = keys.private_key_sz * BITS_PER_BYTE;

	if (rsa_bits == 0U || (rsa_bits % RSA_PUBLIC_MODULUS_LENGTH_DIVISOR) != 0U) {
		return -EINVAL;
	}

	(void)mfd_ite_it51xxx_lock(config->mfd, K_FOREVER);

	memcpy(data->keys.private, keys.private_key, keys.private_key_sz);
	memcpy(data->keys.public, keys.public_key, keys.public_key_sz);
	data->keys.size = keys.private_key_sz;

	mfd_ite_it51xxx_unlock(config->mfd);
	LOG_DBG("set private/public keys");

	return 0;
}

static void it51xxx_rsa_isr(const struct device *dev)
{
	const struct it51xxx_rsa_config *config = dev->config;
	const mm_reg_t base = mfd_ite_it51xxx_get_base(config->mfd);
	struct it51xxx_rsa_data *data = dev->data;
	uint8_t rsa_sts = sys_read8(base + RSA42_STATUS);

	if (rsa_sts & RSA_CALCULATION_DONE) {
		k_sem_give(&data->rsa_cal_done);
		sys_write8(rsa_sts | RSA_CALCULATION_DONE, base + RSA42_STATUS);
	}
}

static int it51xxx_rsa_begin_session(const struct device *dev, struct rsa_ctx *ctx,
				     enum rsa_algo algo, struct rsa_key key)
{
	int ret;

	if (algo != CRYPTO_RSA_ALGO) {
		return -ENOTSUP;
	}

	ret = validate_rsa_key(dev, key);
	if (ret) {
		return ret;
	}

	ctx->sign_hndlr = it51xxx_rsa_encode;
	ctx->verify_hndlr = it51xxx_rsa_decode;
	ctx->device = dev;

	return 0;
}

static int it51xxx_rsa_session_free(const struct device *dev, struct rsa_ctx *ctx)
{
	return 0;
}

static int it51xxx_rsa_init(const struct device *dev)
{
	const struct it51xxx_rsa_config *config = dev->config;
	const mm_reg_t base = mfd_ite_it51xxx_get_base(config->mfd);
	struct it51xxx_rsa_data *data = dev->data;

	k_sem_init(&data->rsa_cal_done, 0, 1);

	LOG_INF("rsa dlm addr %#x(%zu)", (uint32_t)&rsa_dlm, sizeof(rsa_dlm));
	sys_write8(FIELD_GET(GENMASK(15, 8), (uint32_t)&rsa_dlm), base + RSA44_BASE_ADDR_BYTE_1);
	sys_write8(FIELD_GET(GENMASK(17, 16), (uint32_t)&rsa_dlm), base + RSA45_BASE_ADDR_BYTE_2);
	sys_write8(RSA_INTERRUPT_ENABLE, base + RSA40_CTRL);

	config->irq_config_func(dev);

	return 0;
}

static DEVICE_API(crypto, it51xxx_crypto_api) = {
	.rsa_begin_session = it51xxx_rsa_begin_session,
	.rsa_free_session = it51xxx_rsa_session_free,
};

#define IT51XXX_RSA_INIT(n)                                                                        \
	static void it51xxx_rsa_config_func_##n(const struct device *dev)                          \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(DT_PARENT(DT_DRV_INST(n))), 0, it51xxx_rsa_isr,                \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_IRQN(DT_PARENT(DT_DRV_INST(n))));                                    \
	};                                                                                         \
                                                                                                   \
	static const struct it51xxx_rsa_config rsa_config_##n = {                                  \
		.mfd = DEVICE_DT_GET(DT_INST_PARENT(n)),                                           \
		.irq_config_func = it51xxx_rsa_config_func_##n,                                    \
	};                                                                                         \
                                                                                                   \
	static struct it51xxx_rsa_data rsa_data_##n;                                               \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &it51xxx_rsa_init, NULL, &rsa_data_##n, &rsa_config_##n,          \
			      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY, &it51xxx_crypto_api);

DT_INST_FOREACH_STATUS_OKAY(IT51XXX_RSA_INIT)
