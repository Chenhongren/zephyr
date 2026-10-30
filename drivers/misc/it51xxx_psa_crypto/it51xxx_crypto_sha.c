/*
 * Copyright (c) 2026 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define MBEDTLS_DECLARE_PRIVATE_IDENTIFIERS

#include "it51xxx_psa_mbedtls_origin.h"
#include "it51xxx_crypto.h"
#include "it51xxx_crypto_sha.h"

#include "psa_crypto_core.h"

#include <soc_common.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#define LOG_LEVEL CONFIG_IT51XXX_PSA_CRYPTO_LOG_LEVEL
LOG_MODULE_REGISTER(it51xxx_psa_crypto_sha);

#define DT_DRV_COMPAT    ite_it51xxx_psa_sha
#define IT51XXX_SHA_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(DT_DRV_COMPAT)
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "it51xxx: supported one crypto-sha node only");

#define LEGACY_SHA_DT_DRV_COMPAT ite_it51xxx_sha
BUILD_ASSERT(!DT_HAS_COMPAT_STATUS_OKAY(LEGACY_SHA_DT_DRV_COMPAT),
	     "legacy sha and psa-crypto-sha devices cannot be enabled simultaneously");

#define IT51XXX_SHA_IRQ DT_IRQN(DT_PARENT(IT51XXX_SHA_NODE))

#define SHA00_CTRL       0x00
#define SHA_WRITE_BACK   BIT(2)
#define SHA_INIT         BIT(1)
#define SHA_LOAD_AND_EXE BIT(0)

#define SHA01_STATUS 0x01
#define INT_ENABLE   BIT(3)
#define INT_STATUS   BIT(2)
#define SHA_BUSY     BIT(0)

#define SHA02_EXECUTION_COUNTER      0x02
#define SHA224_SHA256_EXE_ONCE_BYTES 64
#define SHA384_SHA512_EXE_ONCE_BYTES 128

#define SHA03_BASE_ADDR_0 0x03
#define BASE_ADDR_0_MASK  GENMASK(7, 6)
#define BASE_ADDR_0(x)    ((uint8_t)((x) & BASE_ADDR_0_MASK))

#define SHA04_BASE_ADDR_1 0x04
#define BASE_ADDR_1(x)    ((uint8_t)FIELD_GET(GENMASK(15, 8), (x)))

#define SHA05_SETTING    0x05
#define SELECT_SHA2_MASK GENMASK(2, 1)
#define SELECT_SHA2(x)   FIELD_PREP(SELECT_SHA2_MASK, x)
#define SHA1_FAMILY      BIT(0)

#define SHA06_BASE_ADDR_2 0x06
#define BASE_ADDR_2(x)    ((uint8_t)FIELD_GET(GENMASK(17, 16), (x)))

#define SHA_SHA256_BLOCK_LEN 64 /* TODO */

#define IT51XXX_SHA_INIT 1U

#define sha256_dlm hwcrypto_dlm.sha256

enum it51xxx_sha2_algorithm {
	IT51XXX_SHA2_SHA256 = 0,
	IT51XXX_SHA2_SHA512,
	IT51XXX_SHA2_SHA224,
	IT51XXX_SHA2_SHA384,
};

enum it51xxx_sha256_execute_bytes {
	IT51XXX_SHA256_EXE_64_BYTE = 64,
	IT51XXX_SHA256_EXE_512_BYTE = 512,
	IT51XXX_SHA256_EXE_1024_BYTE = 1024,
};

static const uint8_t sha224_sha256_supported_execute_times[] = {1U, 8U, 16U};

struct it51xxx_crypto_sha_config {
	const struct device *crypto_dev;
	void (*irq_config_func)(const struct device *dev);
};

struct it51xxx_crypto_sha_data {
	struct k_sem sha_cal_done;
};

static const struct device *sha_dev;

static void it51xxx_sha_isr(const struct device *dev)
{
	const struct it51xxx_crypto_sha_config *config = dev->config;
	const mm_reg_t base = it51xxx_crypto_get_base(config->crypto_dev);
	struct it51xxx_crypto_sha_data *data = dev->data;
	uint8_t sha_sts = sys_read8(base + SHA01_STATUS);

	if (sha_sts & INT_STATUS) {
		k_sem_give(&data->sha_cal_done);
		sys_write8(sha_sts | INT_STATUS, base + SHA01_STATUS);
	}
}

static int it51xxx_sha_set_algorithm(psa_algorithm_t alg)
{
	const struct it51xxx_crypto_sha_config *config = sha_dev->config;
	const mm_reg_t base = it51xxx_crypto_get_base(config->crypto_dev);

	switch (alg) {
	case PSA_ALG_SHA_256:
		sys_write8(SELECT_SHA2(IT51XXX_SHA2_SHA256), base + SHA05_SETTING);
		break;
	case PSA_ALG_SHA_224:
		__fallthrough;
	case PSA_ALG_SHA_384:
		__fallthrough;
	case PSA_ALG_SHA_512:
		__fallthrough;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static void it51xxx_sha256_init(void)
{
	const struct it51xxx_crypto_sha_config *config = sha_dev->config;
	const mm_reg_t base = it51xxx_crypto_get_base(config->crypto_dev);

	sha256_dlm.sha_init = IT51XXX_SHA_INIT;
	sha256_dlm.total_len = 0;
	sha256_dlm.input_bytes_index = 0;

	sys_write8(BASE_ADDR_0((uintptr_t)&sha256_dlm), base + SHA03_BASE_ADDR_0);
	sys_write8(BASE_ADDR_1((uintptr_t)&sha256_dlm), base + SHA04_BASE_ADDR_1);
	sys_write8(BASE_ADDR_2((uintptr_t)&sha256_dlm), base + SHA06_BASE_ADDR_2);
}

/* TODO: check return code */
static int it51xxx_sha256_module_calculation(const uint16_t calculation_bytes)
{
	const struct it51xxx_crypto_sha_config *config = sha_dev->config;
	const mm_reg_t base = it51xxx_crypto_get_base(config->crypto_dev);
	struct it51xxx_crypto_sha_data *data = sha_dev->data;
	uint8_t reg_val;

	for (size_t i = 0; i <= ARRAY_SIZE(sha224_sha256_supported_execute_times); i++) {
		if (i == ARRAY_SIZE(sha224_sha256_supported_execute_times)) {
			return -ENOTSUP;
		}

		if (calculation_bytes ==
		    SHA224_SHA256_EXE_ONCE_BYTES * sha224_sha256_supported_execute_times[i]) {
			reg_val = sha224_sha256_supported_execute_times[i] - 1;
			break;
		}
	}

	/* load data from dlm */
	sys_write8(reg_val, base + SHA02_EXECUTION_COUNTER);
	while (sys_read8(base + SHA01_STATUS) & SHA_BUSY)
		;

	reg_val = 0;

	if (sha256_dlm.sha_init) {
		sha256_dlm.sha_init = 0;
		reg_val |= SHA_INIT;
	}
	sys_write8(reg_val | SHA_LOAD_AND_EXE, base + SHA00_CTRL);
	k_sem_take(&data->sha_cal_done, K_FOREVER);

	sha256_dlm.input_bytes_index = 0;

	return 0;
}

static void it51xxx_write_back_hash_result(uint8_t *hash, size_t *hash_length)
{
	const struct it51xxx_crypto_sha_config *config = sha_dev->config;
	const mm_reg_t base = it51xxx_crypto_get_base(config->crypto_dev);
	struct it51xxx_crypto_sha_data *data = sha_dev->data;

	sys_write8(BASE_ADDR_0((uintptr_t)&sha256_dlm.hash), base + SHA03_BASE_ADDR_0);
	sys_write8(BASE_ADDR_1((uintptr_t)&sha256_dlm.hash), base + SHA04_BASE_ADDR_1);
	sys_write8(BASE_ADDR_2((uintptr_t)&sha256_dlm.hash), base + SHA06_BASE_ADDR_2);

	/* write back the hash result to dlm */
	sys_write8(SHA_WRITE_BACK, base + SHA00_CTRL);
	k_sem_take(&data->sha_cal_done, K_FOREVER);

	memcpy(hash, sha256_dlm.hash, sizeof(sha256_dlm.hash));
	*hash_length = sizeof(sha256_dlm.hash);
}

static void it51xxx_sha_calculate_hash(const uint8_t *input, size_t input_length, uint8_t *hash,
				       size_t *hash_length)
{
	uint32_t in_buf_idx = 0;
	uint32_t rem_len = input_length;

	it51xxx_sha256_init();

	/* data length >= 1k bytes */
	while (rem_len >= IT51XXX_SHA256_EXE_1024_BYTE) {
		rem_len -= IT51XXX_SHA256_EXE_1024_BYTE;

		memcpy(sha256_dlm.input_bytes + sha256_dlm.input_bytes_index, input + in_buf_idx,
		       IT51XXX_SHA256_EXE_1024_BYTE);

		sha256_dlm.input_bytes_index += IT51XXX_SHA256_EXE_1024_BYTE;
		in_buf_idx += IT51XXX_SHA256_EXE_1024_BYTE;

		it51xxx_sha256_module_calculation(IT51XXX_SHA256_EXE_1024_BYTE);
	}

	/* data length >= 512 bytes */
	while (rem_len >= IT51XXX_SHA256_EXE_512_BYTE) {
		rem_len -= IT51XXX_SHA256_EXE_512_BYTE;

		memcpy(sha256_dlm.input_bytes + sha256_dlm.input_bytes_index, input + in_buf_idx,
		       IT51XXX_SHA256_EXE_512_BYTE);

		sha256_dlm.input_bytes_index += IT51XXX_SHA256_EXE_512_BYTE;
		in_buf_idx += IT51XXX_SHA256_EXE_512_BYTE;

		it51xxx_sha256_module_calculation(IT51XXX_SHA256_EXE_512_BYTE);
	}

	/* 0 <= data length < 512 bytes */
	while (rem_len) {
		rem_len--;
		sha256_dlm.input_bytes[sha256_dlm.input_bytes_index++] = input[in_buf_idx++];

		/*
		 * If fill full 64 bytes then execute HW calculation.
		 * If not, will execute in later finish block.
		 */
		if (sha256_dlm.input_bytes_index >= SHA_SHA256_BLOCK_LEN) {
			it51xxx_sha256_module_calculation(IT51XXX_SHA256_EXE_64_BYTE);
		}
	}

	sha256_dlm.total_len = input_length;

	/* append the padding bit (0x80), then zero-fill the rest of the block. */
	sha256_dlm.input_bytes[sha256_dlm.input_bytes_index++] = 0x80;
	memset(&sha256_dlm.input_bytes[sha256_dlm.input_bytes_index], 0,
	       SHA_SHA256_BLOCK_LEN - sha256_dlm.input_bytes_index);

	/* Reserve the last eight bytes for the message length in bits.
	 * If fewer than eight bytes remain after appending the padding
	 * bit, process this block and zero-fill new block for the length
	 * field.
	 */
	if (sha256_dlm.input_bytes_index > 56) {
		it51xxx_sha256_module_calculation(IT51XXX_SHA256_EXE_64_BYTE);

		memset(&sha256_dlm.input_bytes[sha256_dlm.input_bytes_index], 0,
		       SHA_SHA256_BLOCK_LEN - sha256_dlm.input_bytes_index);
	}

	/* Store the low 32 bits of the message length in bits in big-endian
	 * byte order, converting from the CPU's little-endian representation.
	 */
	sha256_dlm.input_words[15] = sys_cpu_to_be32(sha256_dlm.total_len * 8);

	it51xxx_sha256_module_calculation(IT51XXX_SHA256_EXE_64_BYTE);

	it51xxx_write_back_hash_result(hash, hash_length);
}

psa_status_t ite_it51xxx_transparent_hash_compute(psa_algorithm_t alg, const uint8_t *input,
						  size_t input_length, uint8_t *hash,
						  size_t hash_size, size_t *hash_length)
{
	const struct it51xxx_crypto_sha_config *config = sha_dev->config;
	size_t actual_hash_length = PSA_HASH_LENGTH(alg);

	if (!hash || !hash_size) {
		return PSA_ERROR_BUFFER_TOO_SMALL;
	}

	/* Fill the output buffer with something that isn't a valid hash
	 * (barring an attack on the hash and deliberately-crafted input),
	 * in case the caller doesn't check the return status properly. */

	/* If hash_size is 0 then hash may be NULL and then the
	 * call to memset would have undefined behavior. */
	if (hash_size != 0) {
		memset(hash, '!', hash_size);
	}

	if (hash_size < actual_hash_length) {
		return PSA_ERROR_BUFFER_TOO_SMALL;
	}

	if (input_length > UINT32_MAX / 8U) {
		LOG_ERR("input length is too large: %zu/%zu bytes", input_length,
			(size_t)(UINT32_MAX / 8U));
		return PSA_ERROR_INVALID_ARGUMENT;
	}

	(void)it51xxx_crypto_lock(config->crypto_dev, K_FOREVER);

	*hash_length = 0;
	if (it51xxx_sha_set_algorithm(alg)) {
		it51xxx_crypto_unlock(config->crypto_dev);
		return PSA_ERROR_NOT_SUPPORTED;
	}

	chip_block_idle();
	it51xxx_sha_calculate_hash(input, input_length, hash, hash_length);
	chip_permit_idle();

	LOG_HEXDUMP_DBG(hash, *hash_length, "hash:");

	it51xxx_crypto_unlock(config->crypto_dev);

	return PSA_SUCCESS;
}

static int it51xxx_crypto_sha_init(const struct device *dev)
{
	const struct it51xxx_crypto_sha_config *config = dev->config;
	const mm_reg_t base = it51xxx_crypto_get_base(config->crypto_dev);
	struct it51xxx_crypto_sha_data *data = dev->data;

	k_sem_init(&data->sha_cal_done, 0, 1);

	sha_dev = dev;

	/* enable sha interrupt */
	sys_write8(sys_read8(base + SHA01_STATUS) | INT_ENABLE, base + SHA01_STATUS);

	config->irq_config_func(dev);

	return 0;
}

#define IT51XXX_CRYPTO_SHA_INIT(n)                                                                 \
	static void it51xxx_crypto_sha_config_func_##n(const struct device *dev)                   \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(DT_PARENT(DT_DRV_INST(n))), 0, it51xxx_sha_isr,                \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_IRQN(DT_PARENT(DT_DRV_INST(n))));                                    \
	};                                                                                         \
                                                                                                   \
	static const struct it51xxx_crypto_sha_config crypto_sha_config_##n = {                    \
		.crypto_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                    \
		.irq_config_func = it51xxx_crypto_sha_config_func_##n,                             \
	};                                                                                         \
                                                                                                   \
	static struct it51xxx_crypto_sha_data crypto_sha_data_##n;                                 \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &it51xxx_crypto_sha_init, NULL, &crypto_sha_data_##n,             \
			      &crypto_sha_config_##n, POST_KERNEL,                                 \
			      CONFIG_IT51XXX_PSA_CRYPTO_SHA_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IT51XXX_CRYPTO_SHA_INIT)
