/*
 * Copyright (c) 2026 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define MBEDTLS_DECLARE_PRIVATE_IDENTIFIERS

#include "it51xxx_crypto.h"
#include "it51xxx_crypto_rsa.h"
#include "it51xxx_psa_mbedtls_origin.h"

#include "psa_crypto_rsa.h"
#include "psa_crypto_core.h"
#include "mbedtls/platform.h"
#include "mbedtls/private/bignum.h"
#include "mbedtls/oid.h"

#include <soc_common.h>
#include <zephyr/logging/log.h>
#define LOG_LEVEL CONFIG_IT51XXX_PSA_CRYPTO_LOG_LEVEL
LOG_MODULE_REGISTER(it51xxx_psa_crypto_rsa);

#define DT_DRV_COMPAT    ite_it51xxx_psa_rsa
#define IT51XXX_RSA_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(DT_DRV_COMPAT)
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "it51xxx: supported one crypto-rsa node only");

#define IT51XXX_RSA_IRQ DT_IRQN(DT_PARENT(IT51XXX_RSA_NODE))

#define rsa_dlm (hwcrypto_dlm.rsa)

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

enum it51xxx_rsa_key_select {
	KEY_PUBLIC_EXP_65537 = 0,
	KEY_PUBLIC_EXP_3,
	KEY_PRIVATE,
	KEY_RESERVED,
};

struct it51xxx_crypto_rsa_config {
	const struct device *crypto_dev;
	void (*irq_config_func)(const struct device *dev);
};

struct it51xxx_crypto_rsa_data {
	struct k_sem rsa_cal_done;
};

static const uint32_t it51xxx_rsa_supported_exponents[] = {3U, 65537U};

static const struct device *rsa_dev;

static void it51xxx_rsa_isr(const struct device *dev)
{
	const struct it51xxx_crypto_rsa_config *config = dev->config;
	const mm_reg_t base = it51xxx_crypto_get_base(config->crypto_dev);
	struct it51xxx_crypto_rsa_data *data = dev->data;
	uint8_t rsa_sts = sys_read8(base + RSA42_STATUS);

	if (rsa_sts & RSA_CALCULATION_DONE) {
		k_sem_give(&data->rsa_cal_done);
		sys_write8(rsa_sts | RSA_CALCULATION_DONE, base + RSA42_STATUS);
	}
}

static inline int it51xxx_crypto_rsa_exp_valid(const mbedtls_rsa_context *rsa)
{
	uint8_t exponent[4];
	int ret;

	for (size_t i = 0; i < ARRAY_SIZE(it51xxx_rsa_supported_exponents); i++) {
		if (mbedtls_mpi_cmp_int(&rsa->MBEDTLS_PRIVATE(E),
					it51xxx_rsa_supported_exponents[i]) == 0) {
			return 0;
		}
	}

	ret = mbedtls_mpi_write_binary(&rsa->MBEDTLS_PRIVATE(E), exponent, sizeof(exponent));
	if (ret) {
		LOG_ERR("failed to export exponent (%zu bits), %d",
			mbedtls_mpi_bitlen(&rsa->MBEDTLS_PRIVATE(E)), ret);
	} else {
		LOG_HEXDUMP_ERR(exponent, sizeof(exponent), "unsupported exponent:");
	}

	return PSA_ERROR_NOT_SUPPORTED;
}

static inline int it51xxx_crypto_rsa_add_pkcs1_v15_padding(const mbedtls_rsa_context *rsa,
							   unsigned int hashlen,
							   const uint8_t *hash, uint8_t *encode)
{
	size_t sig_len = mbedtls_rsa_get_len(rsa);
	int ret = rsa_rsassa_pkcs1_v15_encode(mbedtls_rsa_get_md_alg(rsa), hashlen, hash, sig_len,
					      encode);

	if (ret) {
		return ret;
	}

	LOG_HEXDUMP_DBG(encode, sig_len, "padding data:");

	return 0;
}

static void it51xxx_rsa_dlm_init(void)
{
	memset(rsa_dlm.messages, 0, sizeof(rsa_dlm.messages));
	memset(rsa_dlm.key_private, 0, sizeof(rsa_dlm.key_private));
	memset(rsa_dlm.key_public, 0, sizeof(rsa_dlm.key_public));
}

static int it51xxx_set_rsa_keys(mbedtls_rsa_context *rsa)
{
	const struct it51xxx_crypto_rsa_config *config = rsa_dev->config;
	const mm_reg_t base = it51xxx_crypto_get_base(config->crypto_dev);
	int ret;
	struct {
		uint8_t public[IT51XXX_HWCRYPTO_RSA_MAX_BYTE_LEN];
		uint8_t private[IT51XXX_HWCRYPTO_RSA_MAX_BYTE_LEN];
		size_t size;
	} keys;

	ret = it51xxx_crypto_rsa_exp_valid(rsa);
	if (ret) {
		return ret;
	}

	keys.size = mbedtls_rsa_get_len(rsa);
	ret = mbedtls_mpi_write_binary(&rsa->MBEDTLS_PRIVATE(N), (unsigned char *)keys.public,
				       keys.size);
	if (ret) {
		return ret;
	}

	ret = mbedtls_mpi_write_binary(&rsa->MBEDTLS_PRIVATE(D), (unsigned char *)keys.private,
				       keys.size);
	if (ret) {
		return ret;
	}

	for (size_t i = 0; i < keys.size; i++) {
		rsa_dlm.key_private[i] = keys.private[keys.size - i - 1];
	}

	for (size_t i = 0; i < keys.size; i++) {
		rsa_dlm.key_public[i] = keys.public[keys.size - i - 1];
	}

	sys_write8(PUBLIC_MODULUS_LENGTH(keys.size * BITS_PER_BYTE), base + RSA41_CTRL_2);

	return 0;
}

static int it51xxx_start_rsa_calculation(void)
{
	const struct it51xxx_crypto_rsa_config *config = rsa_dev->config;
	const mm_reg_t base = it51xxx_crypto_get_base(config->crypto_dev);
	struct it51xxx_crypto_rsa_data *data = rsa_dev->data;

	sys_write8(RSA_CALCULATION_START, base + RSA42_STATUS);

	/* wait for completion */
	k_sem_take(&data->rsa_cal_done, K_FOREVER);

	sys_write8(sys_read8(base + RSA40_CTRL) & ~(RSA_ENABLE | RSA_CLOCK_ENABLE),
		   base + RSA40_CTRL);

	return 0;
}

static int it51xxx_crypto_rsa_sign(mbedtls_md_type_t md_alg, mbedtls_rsa_context *rsa,
				   unsigned int hashlen, const uint8_t *hash, unsigned char *sig)
{
	const struct it51xxx_crypto_rsa_config *config = rsa_dev->config;
	const mm_reg_t base = it51xxx_crypto_get_base(config->crypto_dev);
	int ret;
	size_t sig_len = mbedtls_rsa_get_len(rsa);
	uint8_t rsa_ctrl_val;
	uint8_t hash_with_padding[sig_len];

	chip_block_idle();
	it51xxx_rsa_dlm_init();

	ret = it51xxx_set_rsa_keys(rsa);
	if (ret) {
		goto out;
	}

	ret = it51xxx_crypto_rsa_add_pkcs1_v15_padding(rsa, hashlen, hash, hash_with_padding);
	if (ret) {
		goto out;
	}

	for (size_t i = 0; i < sig_len; i++) {
		rsa_dlm.messages[i] = hash_with_padding[sig_len - i - 1];
	}

	rsa_ctrl_val = sys_read8(base + RSA40_CTRL) & ~RSA_EXPONENT_MASK;
	rsa_ctrl_val |= RSA_ENABLE | RSA_CLOCK_ENABLE | RSA_EXPONENT_SELECT(KEY_PRIVATE);
	sys_write8(rsa_ctrl_val, base + RSA40_CTRL);

	ret = it51xxx_start_rsa_calculation();
	if (ret) {
		goto out;
	}

	for (size_t i = 0; i < sig_len; i++) {
		sig[sig_len - i - 1] = rsa_dlm.messages[i];
	}

	LOG_HEXDUMP_DBG(sig, sig_len, "signature:");

out:
	chip_permit_idle();

	return ret;
}

static int it51xxx_crypto_rsa_verify(mbedtls_rsa_context *rsa, unsigned int hashlen,
				     const uint8_t *hash, const unsigned char *sig)
{
	bool exponent_3 = !mbedtls_mpi_cmp_int(&rsa->MBEDTLS_PRIVATE(E), 3);
	const struct it51xxx_crypto_rsa_config *config = rsa_dev->config;
	const mm_reg_t base = it51xxx_crypto_get_base(config->crypto_dev);
	int ret;
	size_t sig_len = mbedtls_rsa_get_len(rsa);
	uint8_t rsa_ctrl_val;
	uint8_t encoded[sig_len];
	uint8_t encoded_expected[sig_len];

	chip_block_idle();
	it51xxx_rsa_dlm_init();

	ret = it51xxx_set_rsa_keys(rsa);
	if (ret) {
		goto out;
	}

	ret = it51xxx_crypto_rsa_add_pkcs1_v15_padding(rsa, hashlen, hash, encoded_expected);
	if (ret) {
		goto out;
	}

	for (size_t i = 0; i < sig_len; i++) {
		rsa_dlm.messages[i] = sig[sig_len - i - 1];
	}

	rsa_ctrl_val = sys_read8(base + RSA40_CTRL) & ~RSA_EXPONENT_MASK;
	rsa_ctrl_val |= RSA_ENABLE | RSA_CLOCK_ENABLE;
	rsa_ctrl_val |= exponent_3 ? RSA_EXPONENT_SELECT(KEY_PUBLIC_EXP_3)
				   : RSA_EXPONENT_SELECT(KEY_PUBLIC_EXP_65537);
	sys_write8(rsa_ctrl_val, base + RSA40_CTRL);

	ret = it51xxx_start_rsa_calculation();
	if (ret) {
		goto out;
	}

	for (size_t i = 0; i < sig_len; i++) {
		encoded[sig_len - i - 1] = rsa_dlm.messages[i];
	}

	LOG_HEXDUMP_DBG(encoded, sig_len, "verify:");

	if (memcmp(encoded, encoded_expected, sig_len)) {
		ret = PSA_ERROR_INVALID_SIGNATURE;
	}

out:
	chip_permit_idle();

	return ret;
}

static psa_status_t it51xxx_psa_rsa_sign_hash(const psa_key_attributes_t *attributes,
					      const uint8_t *key_buffer, size_t key_buffer_size,
					      psa_algorithm_t alg, const uint8_t *hash,
					      size_t hash_length, uint8_t *signature,
					      size_t signature_size, size_t *signature_length)
{
	const struct it51xxx_crypto_rsa_config *config = rsa_dev->config;
	psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
	int ret = PSA_ERROR_CORRUPTION_DETECTED;
	mbedtls_rsa_context *rsa = NULL;
	mbedtls_md_type_t md_alg;

	status = mbedtls_psa_rsa_load_representation(psa_get_key_type(attributes), key_buffer,
						     key_buffer_size, &rsa);
	if (status != PSA_SUCCESS) {
		goto exit;
	}

	status = psa_rsa_decode_md_type(alg, hash_length, &md_alg);
	if (status != PSA_SUCCESS) {
		goto exit;
	}

	if (signature_size < mbedtls_rsa_get_len(rsa)) {
		status = PSA_ERROR_BUFFER_TOO_SMALL;
		goto exit;
	}

	if (mbedtls_rsa_get_len(rsa) > IT51XXX_HWCRYPTO_RSA_MAX_BYTE_LEN ||
	    mbedtls_rsa_get_len(rsa) < IT51XXX_HWCRYPTO_RSA_MIN_BYTE_LEN) {
		LOG_ERR("unsupported rsa key size %zu (bytes)", mbedtls_rsa_get_len(rsa));
		status = PSA_ERROR_NOT_SUPPORTED;
		goto exit;
	}

#if defined(MBEDTLS_PSA_BUILTIN_ALG_RSA_PKCS1V15_SIGN)
	if (PSA_ALG_IS_RSA_PKCS1V15_SIGN(alg)) {
		ret = mbedtls_rsa_set_padding(rsa, MBEDTLS_RSA_PKCS_V15, md_alg);
		if (ret == 0) {
			(void)it51xxx_crypto_lock(config->crypto_dev, K_FOREVER);
			ret = it51xxx_crypto_rsa_sign(md_alg, rsa, (unsigned int)hash_length, hash,
						      signature);
			it51xxx_crypto_unlock(config->crypto_dev);
		}
	} else
#endif /* MBEDTLS_PSA_BUILTIN_ALG_RSA_PKCS1V15_SIGN */
	{
		status = PSA_ERROR_INVALID_ARGUMENT;
		goto exit;
	}

	if (ret == 0) {
		*signature_length = mbedtls_rsa_get_len(rsa);
	}
	status = mbedtls_to_psa_error(ret);

exit:
	mbedtls_rsa_free(rsa);
	mbedtls_free(rsa);

	return status;
}

static psa_status_t it51xxx_psa_rsa_verify_hash(const psa_key_attributes_t *attributes,
						const uint8_t *key_buffer, size_t key_buffer_size,
						psa_algorithm_t alg, const uint8_t *hash,
						size_t hash_length, const uint8_t *signature,
						size_t signature_length)
{
	const struct it51xxx_crypto_rsa_config *config = rsa_dev->config;
	psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
	int ret;
	mbedtls_rsa_context *rsa = NULL;
	mbedtls_md_type_t md_alg;

	status = mbedtls_psa_rsa_load_representation(psa_get_key_type(attributes), key_buffer,
						     key_buffer_size, &rsa);
	if (status != PSA_SUCCESS) {
		goto exit;
	}

	status = psa_rsa_decode_md_type(alg, hash_length, &md_alg);
	if (status != PSA_SUCCESS) {
		goto exit;
	}

	if (signature_length != mbedtls_rsa_get_len(rsa)) {
		status = PSA_ERROR_INVALID_SIGNATURE;
		goto exit;
	}

	if (mbedtls_rsa_get_len(rsa) > IT51XXX_HWCRYPTO_RSA_MAX_BYTE_LEN ||
	    mbedtls_rsa_get_len(rsa) < IT51XXX_HWCRYPTO_RSA_MIN_BYTE_LEN) {
		LOG_ERR("unsupported rsa key size %zu (bytes)", mbedtls_rsa_get_len(rsa));
		status = PSA_ERROR_NOT_SUPPORTED;
		goto exit;
	}

#if defined(MBEDTLS_PSA_BUILTIN_ALG_RSA_PKCS1V15_SIGN)
	if (PSA_ALG_IS_RSA_PKCS1V15_SIGN(alg)) {
		ret = mbedtls_rsa_set_padding(rsa, MBEDTLS_RSA_PKCS_V15, md_alg);
		if (ret == 0) {
			(void)it51xxx_crypto_lock(config->crypto_dev, K_FOREVER);
			ret = it51xxx_crypto_rsa_verify(rsa, (unsigned int)hash_length, hash,
							signature);
			it51xxx_crypto_unlock(config->crypto_dev);
		}
	} else
#endif /* MBEDTLS_PSA_BUILTIN_ALG_RSA_PKCS1V15_SIGN */
	{
		status = PSA_ERROR_INVALID_ARGUMENT;
		goto exit;
	}

	/* Mbed TLS distinguishes "invalid padding" from "valid padding but
	 * the rest of the signature is invalid". This has little use in
	 * practice and PSA doesn't report this distinction. */
	status = (ret == MBEDTLS_ERR_RSA_INVALID_PADDING) ? PSA_ERROR_INVALID_SIGNATURE
							  : mbedtls_to_psa_error(ret);

exit:
	mbedtls_rsa_free(rsa);
	mbedtls_free(rsa);

	return status;
}

psa_status_t ite_it51xxx_transparent_sign_hash(const psa_key_attributes_t *attributes,
					       const uint8_t *key_buffer, size_t key_buffer_size,
					       psa_algorithm_t alg, const uint8_t *hash,
					       size_t hash_length, uint8_t *signature,
					       size_t signature_size, size_t *signature_length)
{
	if (psa_get_key_type(attributes) == PSA_KEY_TYPE_RSA_KEY_PAIR) {
		if (PSA_ALG_IS_RSA_PKCS1V15_SIGN(alg)) {
#if defined(MBEDTLS_PSA_BUILTIN_ALG_RSA_PKCS1V15_SIGN)
			return it51xxx_psa_rsa_sign_hash(attributes, key_buffer, key_buffer_size,
							 alg, hash, hash_length, signature,
							 signature_size, signature_length);
#endif /* defined(MBEDTLS_PSA_BUILTIN_ALG_RSA_PKCS1V15_SIGN) */
		} else {
			return PSA_ERROR_INVALID_ARGUMENT;
		}
	}

	(void)key_buffer;
	(void)key_buffer_size;
	(void)hash;
	(void)hash_length;
	(void)signature;
	(void)signature_size;
	(void)signature_length;

	return PSA_ERROR_NOT_SUPPORTED;
}

psa_status_t ite_it51xxx_transparent_verify_hash(const psa_key_attributes_t *attributes,
						 const uint8_t *key_buffer, size_t key_buffer_size,
						 psa_algorithm_t alg, const uint8_t *hash,
						 size_t hash_length, const uint8_t *signature,
						 size_t signature_length)
{

	if (PSA_KEY_TYPE_IS_RSA(psa_get_key_type(attributes))) {
		if (PSA_ALG_IS_RSA_PKCS1V15_SIGN(alg)) {
#if defined(MBEDTLS_PSA_BUILTIN_ALG_RSA_PKCS1V15_SIGN)
			return it51xxx_psa_rsa_verify_hash(attributes, key_buffer, key_buffer_size,
							   alg, hash, hash_length, signature,
							   signature_length);
#endif /* defined(MBEDTLS_PSA_BUILTIN_ALG_RSA_PKCS1V15_SIGN) */
		} else {
			return PSA_ERROR_INVALID_ARGUMENT;
		}
	}

	(void)key_buffer;
	(void)key_buffer_size;
	(void)hash;
	(void)hash_length;
	(void)signature;
	(void)signature_length;

	return PSA_ERROR_NOT_SUPPORTED;
}

static int it51xxx_crypto_rsa_init(const struct device *dev)
{
	const struct it51xxx_crypto_rsa_config *config = dev->config;
	const mm_reg_t base = it51xxx_crypto_get_base(config->crypto_dev);
	struct it51xxx_crypto_rsa_data *data = dev->data;

	rsa_dev = dev;

	k_sem_init(&data->rsa_cal_done, 0, 1);

	LOG_INF("rsa dlm addr %#x(%zu)", (uint32_t)&rsa_dlm, sizeof(rsa_dlm));
	sys_write8(FIELD_GET(GENMASK(15, 8), (uint32_t)&rsa_dlm), base + RSA44_BASE_ADDR_BYTE_1);
	sys_write8(FIELD_GET(GENMASK(17, 16), (uint32_t)&rsa_dlm), base + RSA45_BASE_ADDR_BYTE_2);
	sys_write8(RSA_INTERRUPT_ENABLE, base + RSA40_CTRL);

	config->irq_config_func(dev);

	return 0;
}

#define IT51XXX_CRYPTO_RSA_INIT(n)                                                                 \
	static void it51xxx_crypto_rsa_config_func_##n(const struct device *dev)                   \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(DT_PARENT(DT_DRV_INST(n))), 0, it51xxx_rsa_isr,                \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_IRQN(DT_PARENT(DT_DRV_INST(n))));                                    \
	};                                                                                         \
                                                                                                   \
	static const struct it51xxx_crypto_rsa_config crypto_rsa_config_##n = {                    \
		.crypto_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                    \
		.irq_config_func = it51xxx_crypto_rsa_config_func_##n,                             \
	};                                                                                         \
                                                                                                   \
	static struct it51xxx_crypto_rsa_data crypto_rsa_data_##n;                                 \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &it51xxx_crypto_rsa_init, NULL, &crypto_rsa_data_##n,             \
			      &crypto_rsa_config_##n, POST_KERNEL,                                 \
			      CONFIG_IT51XXX_PSA_CRYPTO_RSA_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IT51XXX_CRYPTO_RSA_INIT)
