/*
 * Copyright (c) 2026 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_CRYPTO_RSA_H_
#define ZEPHYR_INCLUDE_CRYPTO_RSA_H_

/* Forward declarations */
struct rsa_ctx;
struct rsa_pkt;

/** RSA Algorithm */
enum rsa_algo {
	CRYPTO_RSA_ALGO = 1,
};

typedef int (*sign_op_t)(struct rsa_ctx *ctx, struct rsa_pkt *pkt);
typedef int (*verify_op_t)(struct rsa_ctx *ctx, struct rsa_pkt *pkt, bool exp_3);

struct rsa_key {
	/** RSA private key. */
	const uint8_t *private_key;
	/** Size of the private key, in bytes. */
	size_t private_key_sz;
	/** RSA public key. */
	const uint8_t *public_key;
	/** Size of the public key, in bytes. */
	size_t public_key_sz;
};

struct rsa_ctx {
	/**
	 * Device driver instance this crypto context relates to.
	 *
	 * This is populated by rsa_begin_session().
	 */
	const struct device *device;

	sign_op_t	sign_hndlr;

	verify_op_t	verify_hndlr;
};

struct rsa_pkt {
	/** Hash buffer. */
	uint8_t *hash;
	/** Size of the hash buffer, in bytes. */
	size_t hash_length;
	/** Sign buffer. */
	uint8_t *sign;
	/** Size of the sign buffer, in bytes. */
	size_t sign_length;

	struct rsa_ctx *ctx;
};

#endif /* ZEPHYR_INCLUDE_CRYPTO_RSA_H_ */
