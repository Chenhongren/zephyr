/*
 * Copyright (c) 2026 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/crypto/crypto.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "rsa_2048_keys.h"
#include "rsa_4096_keys.h"

ZTEST(crypto_rsa, rsa_2048_key_bits)
{
	const struct device *dev = (struct device *)DEVICE_DT_GET(DT_NODELABEL(rsa));
	const bool exponent_3 = false;
	enum rsa_algo algo = CRYPTO_RSA_ALGO;
	int ret;
	static uint8_t output[sizeof(ciphertext_2048)];
	struct rsa_ctx ctx;
	struct rsa_pkt pkt;
	struct rsa_key keys = {
		.private_key = private_key_2048,
		.private_key_sz = sizeof(private_key_2048),
		.public_key = public_key_2048,
		.public_key_sz = sizeof(public_key_2048),
	};

	ret = rsa_begin_session(dev, &ctx, algo, keys);
	zassert_equal(ret, 0, "begin_session failed");

	pkt.hash = plaintext_2048;
	pkt.hash_length = sizeof(plaintext_2048);
	pkt.sign = output;
	pkt.sign_length = sizeof(output);
	memset(output, 0, sizeof(output));
	ret = rsa_sign(&ctx, &pkt);
	zassert_equal(ret, 0, "failed to perform rsa sign");
	ret = memcmp(pkt.sign, ciphertext_2048, sizeof(ciphertext_2048));
	zassert_equal(ret, 0, "signature error");

	pkt.sign = ciphertext_2048;
	pkt.sign_length = sizeof(ciphertext_2048);
	pkt.hash = output;
	pkt.hash_length = sizeof(output);
	memset(output, 0, sizeof(output));
	ret = rsa_verify(&ctx, &pkt, exponent_3);
	zassert_equal(ret, 0, "failed to perform verify");
	ret = memcmp(pkt.hash, plaintext_2048, sizeof(plaintext_2048));
	zassert_equal(ret, 0, "verify error");

	ret = rsa_free_session(dev, &ctx);
	zassert_equal(ret, 0, "free_session failed");
}

ZTEST(crypto_rsa, rsa_4096_key_bits)
{
	const struct device *dev = (struct device *)DEVICE_DT_GET(DT_NODELABEL(rsa));
	const bool exponent_3 = false;
	enum rsa_algo algo = CRYPTO_RSA_ALGO;
	int ret;
	static uint8_t output[sizeof(ciphertext_4096)];
	struct rsa_ctx ctx;
	struct rsa_pkt pkt;
	struct rsa_key keys = {
		.private_key = private_key_4096,
		.private_key_sz = sizeof(private_key_4096),
		.public_key = public_key_4096,
		.public_key_sz = sizeof(public_key_4096),
	};

	ret = rsa_begin_session(dev, &ctx, algo, keys);
	zassert_equal(ret, 0, "begin_session failed");

	pkt.hash = plaintext_4096;
	pkt.hash_length = sizeof(plaintext_4096);
	pkt.sign = output;
	pkt.sign_length = sizeof(output);
	memset(output, 0, sizeof(output));
	ret = rsa_sign(&ctx, &pkt);
	zassert_equal(ret, 0, "failed to perform rsa sign");
	ret = memcmp(pkt.sign, ciphertext_4096, sizeof(ciphertext_4096));
	zassert_equal(ret, 0, "signature error");

	pkt.sign = ciphertext_4096;
	pkt.sign_length = sizeof(ciphertext_4096);
	pkt.hash = output;
	pkt.hash_length = sizeof(output);
	memset(output, 0, sizeof(output));
	ret = rsa_verify(&ctx, &pkt, exponent_3);
	zassert_equal(ret, 0, "failed to perform verify");
	ret = memcmp(pkt.hash, plaintext_4096, sizeof(plaintext_4096));
	zassert_equal(ret, 0, "verify error");

	ret = rsa_free_session(dev, &ctx);
	zassert_equal(ret, 0, "free_session failed");
}

ZTEST_SUITE(crypto_rsa, NULL, NULL, NULL, NULL, NULL);
