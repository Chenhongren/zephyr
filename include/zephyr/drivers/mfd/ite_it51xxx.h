/*
 * Copyright (c) 2026 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for IT51XXX MFD driver
 * @ingroup mfd_interface_ite_it51xxx
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MFD_ITE_IT51XXX_H_
#define ZEPHYR_INCLUDE_DRIVERS_MFD_ITE_IT51XXX_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>

/**
 * @defgroup mfd_interface_ite_it51xxx MFD ITE IT51XXX interface
 * @ingroup mfd_interfaces
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#define IT51XXX_HWCRYPTO_DLM_SIZE KB(4)

#define SHA_SHA256_HASH_LEN        32
#define SHA_SHA256_BLOCK_LEN       64
#define SHA_SHA256_HASH_LEN_WORDS  (SHA_SHA256_HASH_LEN / sizeof(uint32_t))
#define SHA_SHA256_BLOCK_LEN_WORDS (SHA_SHA256_BLOCK_LEN / sizeof(uint32_t))

/*
 * If the input message is more than 1K bytes, taking 10K bytes for example,
 * it should run 10 times SHA hardwired loading and execution, and process 1K bytes each time.
 */
#define SHA_HW_MAX_INPUT_LEN       1024
#define SHA_HW_MAX_INPUT_LEN_WORDS (SHA_HW_MAX_INPUT_LEN / sizeof(uint32_t))

#define IT51XXX_HWCRYPTO_RSA_MIN_BYTE_LEN 64  /* 64 bytes (512 bits) */
#define IT51XXX_HWCRYPTO_RSA_MAX_BYTE_LEN 512 /* 512 bytes (4096 bits) */

/*
 * This struct is used by the hardware and must be stored in RAM first 4k-byte
 * and aligned on a 256-byte boundary.
 */
struct it51xxx_sha_dlm {
	union {
		/* SHA data buffer */
		uint32_t w_sha[SHA_HW_MAX_INPUT_LEN_WORDS];
		uint8_t w_input[SHA_HW_MAX_INPUT_LEN];
	};
	/* H[0] ~ H[7] */
	uint32_t h[SHA_SHA256_HASH_LEN_WORDS];
	uint32_t sha_init;
	uint32_t w_input_index;
	uint32_t total_len;
} __aligned(256);

struct it51xxx_rsa_dlm {
	uint8_t key_public[IT51XXX_HWCRYPTO_RSA_MAX_BYTE_LEN];  /* 000h – 1FFh */
	uint8_t reserved_1[512];                                /* 200h - 3FFh */
	uint8_t messages[IT51XXX_HWCRYPTO_RSA_MAX_BYTE_LEN];    /* 400h – 5FFh */
	uint8_t reserved_2[1024];                               /* 600h - 9FFh */
	uint8_t key_private[IT51XXX_HWCRYPTO_RSA_MAX_BYTE_LEN]; /* A00h – BFFh */
	uint8_t reserved_3[1024];                               /* C00h - FFFh */
};

/* The it51xxx hardware crypto engines, including SHA, RSA...etc,
 * share the same 4 KB DLM memory region. The union ensures that
 * each engine uses the same underlying memory.
 */
union hwcrypto_dlm_block {
	struct it51xxx_sha_dlm sha;
	struct it51xxx_rsa_dlm rsa;
	uint8_t raw_data[IT51XXX_HWCRYPTO_DLM_SIZE];
};

extern union hwcrypto_dlm_block hwcrypto_dlm;

/**
 * @brief Get the IT51XXX MFD register base address.
 *
 * The base address is shared by the hardware crypto (AES/SHA/RSA..etc) child
 * functions.
 *
 * @param[in] dev Pointer to the parent MFD device.
 *
 * @return Base address of the MFD register block.
 */
mm_reg_t mfd_ite_it51xxx_get_base(const struct device *dev);

/**
 * @brief Lock access to the shared IT51XXX MFD hardware.
 *
 * RSA, SHA, and other child drivers must acquire this lock before accessing
 * shared MFD resources.
 *
 * @param[in] dev     Pointer to the parent MFD device.
 * @param[in] timeout Lock acquisition timeout.
 *
 * @retval 0 Lock acquired successfully.
 * @retval -EAGAIN The timeout expired before the lock was acquired.
 */
int mfd_ite_it51xxx_lock(const struct device *dev, k_timeout_t timeout);

/**
 * @brief Unlock access to the shared IT51XXX MFD hardware.
 *
 * @param[in] dev Pointer to the parent MFD device.
 */
void mfd_ite_it51xxx_unlock(const struct device *dev);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* ZEPHYR_INCLUDE_DRIVERS_MFD_ITE_IT51XXX_H_ */
