/*
 * Copyright (c) 2026 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public APIs for the it51xxx hw rsa driver
 * @ingroup it51xxx_hw_rsa_interface
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_IT51XXX_HW_RSA_IT51XXX_HW_RSA_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_IT51XXX_HW_RSA_IT51XXX_HW_RSA_H_

#include <stdint.h>
#include <zephyr/sys/slist.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/internal/syscall_handler.h>

#ifdef __cplusplus
extern "C" {
#endif

/* for encryption, the input is plaintext and the output is ciphertext,
 * and vice versa for decryption
 */
struct it51xxx_hw_rsa_messages {
	const uint8_t *input;
	size_t input_sz;
	uint8_t *output;
	size_t output_sz;
};

struct it51xxx_hw_rsa_keys {
	const uint8_t *private_key;
	size_t private_key_sz;
	const uint8_t *public_key;
	size_t public_key_sz;
};

__subsystem struct it51xxx_hw_rsa_driver_api {
	int (*configure)(const struct device *dev, const struct it51xxx_hw_rsa_keys keys);
	int (*encrypt)(const struct device *dev, struct it51xxx_hw_rsa_messages const *msgs);
	int (*decrypt)(const struct device *dev, const bool exponent_3,
		       struct it51xxx_hw_rsa_messages const *msgs);
};

static inline int it51xxx_hw_rsa_configure(const struct device *dev,
					   const struct it51xxx_hw_rsa_keys keys)
{
	const struct it51xxx_hw_rsa_driver_api *api =
		(const struct it51xxx_hw_rsa_driver_api *)dev->api;

	if (api->configure == NULL) {
		return -ENOSYS;
	}

	return api->configure(dev, keys);
}

static inline int it51xxx_hw_rsa_encrypt(const struct device *dev,
					 struct it51xxx_hw_rsa_messages const *msgs)
{
	const struct it51xxx_hw_rsa_driver_api *api =
		(const struct it51xxx_hw_rsa_driver_api *)dev->api;

	if (api->encrypt == NULL) {
		return -ENOSYS;
	}

	return api->encrypt(dev, msgs);
}

static inline int it51xxx_hw_rsa_decrypt(const struct device *dev, const bool exponent_3,
					 struct it51xxx_hw_rsa_messages const *msgs)
{
	const struct it51xxx_hw_rsa_driver_api *api =
		(const struct it51xxx_hw_rsa_driver_api *)dev->api;

	if (api->encrypt == NULL) {
		return -ENOSYS;
	}

	return api->decrypt(dev, exponent_3, msgs);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_IT51XXX_HW_RSA_IT51XXX_HW_RSA_H_ */
