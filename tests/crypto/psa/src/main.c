/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Test psa_crypto_init() and psa_generate_random() on the PSA implementation
 * provided by Mbed TLS (platforms using TFM are filtered out in the yaml file).
 */

#include <zephyr/ztest.h>

#include <psa/crypto.h>

ZTEST_USER(psa_crypto_test_suite, test_generate_random)
{
	uint8_t tmp[64];
	psa_status_t status;

	status = psa_generate_random(tmp, sizeof(tmp));
	zassert_equal(status, PSA_SUCCESS);
}

ZTEST_USER(psa_crypto_test_suite, test_sha1)
{
	uint8_t in_buf[] = { 'a' };
	uint8_t out_buf[PSA_HASH_LENGTH(PSA_ALG_SHA_1)] = { 0 };
	uint8_t out_buf_ref[PSA_HASH_LENGTH(PSA_ALG_SHA_1)] = {
		0x86, 0xf7, 0xe4, 0x37, 0xfa, 0xa5, 0xa7, 0xfc, 0xe1, 0x5d,
		0x1d, 0xdc, 0xb9, 0xea, 0xea, 0xea, 0x37, 0x76, 0x67, 0xb8
	};
	size_t out_len;
	psa_status_t status;

	status = psa_hash_compute(PSA_ALG_SHA_1, in_buf, sizeof(in_buf),
				  out_buf, sizeof(out_buf), &out_len);
	zassert_equal(status, PSA_SUCCESS);
	zassert_mem_equal(out_buf, out_buf_ref, sizeof(out_buf_ref));
}

ZTEST_USER(psa_crypto_test_suite, test_sha224)
{
	uint8_t in_buf[] = { 'a' };
	uint8_t out_buf[PSA_HASH_LENGTH(PSA_ALG_SHA_224)] = { 0 };
	uint8_t out_buf_ref[PSA_HASH_LENGTH(PSA_ALG_SHA_224)] = {
		0xab, 0xd3, 0x75, 0x34, 0xc7, 0xd9, 0xa2, 0xef, 0xb9, 0x46,
		0x5d, 0xe9, 0x31, 0xcd, 0x70, 0x55, 0xff, 0xdb, 0x88, 0x79,
		0x56, 0x3a, 0xe9, 0x80, 0x78, 0xd6, 0xd6, 0xd5
	};
	size_t out_len;
	psa_status_t status;

	status = psa_hash_compute(PSA_ALG_SHA_224, in_buf, sizeof(in_buf),
				  out_buf, sizeof(out_buf), &out_len);
	zassert_equal(status, PSA_SUCCESS);
	zassert_mem_equal(out_buf, out_buf_ref, sizeof(out_buf_ref));
}

ZTEST_USER(psa_crypto_test_suite, test_sha256)
{
	uint8_t in_buf[] = { 'a' };
	uint8_t out_buf[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = { 0 };
	uint8_t out_buf_ref[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = {
		0xca, 0x97, 0x81, 0x12, 0xca, 0x1b, 0xbd, 0xca, 0xfa, 0xc2,
		0x31, 0xb3, 0x9a, 0x23, 0xdc, 0x4d, 0xa7, 0x86, 0xef, 0xf8,
		0x14, 0x7c, 0x4e, 0x72, 0xb9, 0x80, 0x77, 0x85, 0xaf, 0xee,
		0x48, 0xbb
	};
	size_t out_len;
	psa_status_t status;

	status = psa_hash_compute(PSA_ALG_SHA_256, in_buf, sizeof(in_buf),
				  out_buf, sizeof(out_buf), &out_len);
	zassert_equal(status, PSA_SUCCESS);
	zassert_mem_equal(out_buf, out_buf_ref, sizeof(out_buf_ref));
}

ZTEST_USER(psa_crypto_test_suite, test_sha384)
{
	uint8_t in_buf[] = { 'a' };
	uint8_t out_buf[PSA_HASH_LENGTH(PSA_ALG_SHA_384)] = { 0 };
	uint8_t out_buf_ref[PSA_HASH_LENGTH(PSA_ALG_SHA_384)] = {
		0x54, 0xa5, 0x9b, 0x9f, 0x22, 0xb0, 0xb8, 0x08, 0x80, 0xd8,
		0x42, 0x7e, 0x54, 0x8b, 0x7c, 0x23, 0xab, 0xd8, 0x73, 0x48,
		0x6e, 0x1f, 0x03, 0x5d, 0xce, 0x9c, 0xd6, 0x97, 0xe8, 0x51,
		0x75, 0x03, 0x3c, 0xaa, 0x88, 0xe6, 0xd5, 0x7b, 0xc3, 0x5e,
		0xfa, 0xe0, 0xb5, 0xaf, 0xd3, 0x14, 0x5f, 0x31
	};
	size_t out_len;
	psa_status_t status;

	status = psa_hash_compute(PSA_ALG_SHA_384, in_buf, sizeof(in_buf),
				  out_buf, sizeof(out_buf), &out_len);
	zassert_equal(status, PSA_SUCCESS);
	zassert_mem_equal(out_buf, out_buf_ref, sizeof(out_buf_ref));
}

ZTEST_USER(psa_crypto_test_suite, test_sha512)
{
	uint8_t in_buf[] = { 'a' };
	uint8_t out_buf[PSA_HASH_LENGTH(PSA_ALG_SHA_512)] = { 0 };
	uint8_t out_buf_ref[PSA_HASH_LENGTH(PSA_ALG_SHA_512)] = {
		0x1f, 0x40, 0xfc, 0x92, 0xda, 0x24, 0x16, 0x94, 0x75, 0x09,
		0x79, 0xee, 0x6c, 0xf5, 0x82, 0xf2, 0xd5, 0xd7, 0xd2, 0x8e,
		0x18, 0x33, 0x5d, 0xe0, 0x5a, 0xbc, 0x54, 0xd0, 0x56, 0x0e,
		0x0f, 0x53, 0x02, 0x86, 0x0c, 0x65, 0x2b, 0xf0, 0x8d, 0x56,
		0x02, 0x52, 0xaa, 0x5e, 0x74, 0x21, 0x05, 0x46, 0xf3, 0x69,
		0xfb, 0xbb, 0xce, 0x8c, 0x12, 0xcf, 0xc7, 0x95, 0x7b, 0x26,
		0x52, 0xfe, 0x9a, 0x75
	};
	size_t out_len;
	psa_status_t status;

	status = psa_hash_compute(PSA_ALG_SHA_512, in_buf, sizeof(in_buf),
				  out_buf, sizeof(out_buf), &out_len);
	zassert_equal(status, PSA_SUCCESS);
	zassert_mem_equal(out_buf, out_buf_ref, sizeof(out_buf_ref));
}

ZTEST_USER(psa_crypto_test_suite, test_hmac_sha256)
{
	uint8_t key[] = { 'a' };
	psa_key_attributes_t key_attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t key_id = PSA_KEY_ID_NULL;
	uint8_t in_buf[] = { 'a' };
	uint8_t out_buf[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = { 0 };
	uint8_t out_buf_ref[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = {
		0x3e, 0xcf, 0x53, 0x88, 0xe2, 0x20, 0xda, 0x9e,
		0x0f, 0x91, 0x94, 0x85, 0xde, 0xb6, 0x76, 0xd8,
		0xbe, 0xe3, 0xae, 0xc0, 0x46, 0xa7, 0x79, 0x35,
		0x3b, 0x46, 0x34, 0x18, 0x51, 0x1e, 0xe6, 0x22
	};
	size_t out_len;
	psa_status_t status;

	psa_set_key_type(&key_attr, PSA_KEY_TYPE_HMAC);
	psa_set_key_algorithm(&key_attr, PSA_ALG_HMAC(PSA_ALG_SHA_256));
	psa_set_key_usage_flags(&key_attr, PSA_KEY_USAGE_SIGN_MESSAGE);

	status = psa_import_key(&key_attr, key, sizeof(key), &key_id);
	zassert_equal(status, PSA_SUCCESS);

	status = psa_mac_compute(key_id, PSA_ALG_HMAC(PSA_ALG_SHA_256),
				 in_buf, sizeof(in_buf),
				 out_buf, sizeof(out_buf), &out_len);
	zassert_equal(status, PSA_SUCCESS);
	zassert_mem_equal(out_buf, out_buf_ref, sizeof(out_buf_ref));

	status = psa_destroy_key(key_id);
	zassert_equal(status, PSA_SUCCESS);
}

ZTEST_USER(psa_crypto_test_suite, test_aes_ecb)
{
	uint8_t key[] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
			 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf};
	psa_key_attributes_t key_attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t key_id = PSA_KEY_ID_NULL;
	uint8_t in_buf[PSA_BLOCK_CIPHER_BLOCK_LENGTH(PSA_KEY_TYPE_AES)];
#define AES_ENCRYPTED_OUTPUT_SIZE \
	PSA_CIPHER_ENCRYPT_OUTPUT_SIZE(PSA_KEY_TYPE_AES, PSA_ALG_ECB_NO_PADDING, sizeof(in_buf))
	uint8_t out_buf[AES_ENCRYPTED_OUTPUT_SIZE] = { 0 };
	uint8_t out_buf_ref[AES_ENCRYPTED_OUTPUT_SIZE] = {
		0xea, 0x5e, 0x61, 0xae, 0x81, 0x67, 0xca, 0xa0,
		0x58, 0x63, 0x88, 0xeb, 0x9a, 0x7c, 0xb7, 0x55
	};
	size_t out_len;
	psa_status_t status;

	memset(in_buf, 0x5, sizeof(in_buf));

	psa_set_key_type(&key_attr, PSA_KEY_TYPE_AES);
	psa_set_key_algorithm(&key_attr, PSA_ALG_ECB_NO_PADDING);
	psa_set_key_usage_flags(&key_attr, PSA_KEY_USAGE_ENCRYPT);
	status = psa_import_key(&key_attr, key, sizeof(key), &key_id);
	zassert_equal(status, PSA_SUCCESS);

	status = psa_cipher_encrypt(key_id, PSA_ALG_ECB_NO_PADDING, in_buf, sizeof(in_buf), out_buf,
				    sizeof(out_buf), &out_len);
	zassert_equal(status, PSA_SUCCESS);

	zassert_mem_equal(out_buf, out_buf_ref, sizeof(out_buf_ref));

	status = psa_destroy_key(key_id);
	zassert_equal(status, PSA_SUCCESS);
}

#include "rsa_2048.h"
#include "rsa_4096.h"

#include <stdio.h>
#include <psa/crypto.h>
#include <zephyr/ztest.h>

#define SIGN_ALG PSA_ALG_RSA_PKCS1V15_SIGN(PSA_ALG_SHA_256)

__maybe_unused static void print_hex(const char *label, const uint8_t *data, size_t length)
{
	printf("%s (%zu bytes):\n", label, length);

	for (size_t i = 0; i < length; i++) {
		printf("%02X ", (unsigned int)data[i]);

		if ((i + 1) % 16 == 0) {
			printf("\n");
		}
	}

	if (length % 16 != 0) {
		printf("\n");
	}
}

ZTEST_USER(psa_crypto_test_suite, test_rsa_2048)
{
	psa_status_t status;
	psa_key_id_t private_key_id = 0;
	psa_key_id_t public_key_id = 0;
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
	const uint8_t message[] = "Hello RSA";
	uint8_t received_data[sizeof(message) - 1];
	uint8_t hash[PSA_HASH_LENGTH(PSA_ALG_SHA_256)];
	uint8_t signature[PSA_SIGN_OUTPUT_SIZE(PSA_KEY_TYPE_RSA_KEY_PAIR, RSA_BITS_2048, SIGN_ALG)];
	size_t hash_length = 0;
	size_t signature_length = 0;

	status = psa_crypto_init();
	if (status != PSA_SUCCESS) {
		printf("<error> failed to initialize PSA Crypto: %ld\n", (long)status);
		goto cleanup;
	}

	/* Import the private key instead of generating a key pair */
	psa_set_key_type(&attributes, PSA_KEY_TYPE_RSA_KEY_PAIR);
	psa_set_key_bits(&attributes, RSA_BITS_2048);
	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_HASH);
	psa_set_key_algorithm(&attributes, SIGN_ALG);
	status = psa_import_key(&attributes, private_key_der, sizeof(private_key_der),
				&private_key_id);
	psa_reset_key_attributes(&attributes);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to import private key: %ld\n", (long)status);
		goto cleanup;
	}

	/* Import a separate public key for verification */
	psa_set_key_type(&attributes, PSA_KEY_TYPE_RSA_PUBLIC_KEY);
	psa_set_key_bits(&attributes, RSA_BITS_2048);
	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_VERIFY_HASH);
	psa_set_key_algorithm(&attributes, SIGN_ALG);
	status =
		psa_import_key(&attributes, public_key_der, sizeof(public_key_der), &public_key_id);
	psa_reset_key_attributes(&attributes);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to import public key: %ld\n", (long)status);
		goto cleanup;
	}

#if 0
	print_hex("Message", message, sizeof(message));
#endif

	/* Hash the original message, excluding the terminating '\0' */
	status = psa_hash_compute(PSA_ALG_SHA_256, message, sizeof(message) - 1, hash, sizeof(hash),
				  &hash_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to hash original message: %ld\n", (long)status);
		goto cleanup;
	}

#if 0
	print_hex("Hash(SHA256)", hash, hash_length);
#endif

	/* Sign using the imported private key */
	status = psa_sign_hash(private_key_id, SIGN_ALG, hash, hash_length, signature,
			       sizeof(signature), &signature_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to sign hash: %ld\n", (long)status);
		goto cleanup;
	}

#if 0
	print_hex("Signature", signature, signature_length);
#endif

	/* Simulate receiving the message and independently hash it */
	memcpy(received_data, message, sizeof(received_data));
	status = psa_hash_compute(PSA_ALG_SHA_256, received_data, sizeof(received_data), hash,
				  sizeof(hash), &hash_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to hash received data: %ld\n", (long)status);
		goto cleanup;
	}

	/* Verify using the independently imported public key */
	status = psa_verify_hash(public_key_id, SIGN_ALG, hash, hash_length, signature,
				 signature_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to verify received data: %ld\n", (long)status);
		goto cleanup;
	}
	printf("<info> RSA sign/verify OK, signature: %zu bytes\n", signature_length);

	/* Modified data must fail verification */
	received_data[0] ^= 0x01;
	status = psa_hash_compute(PSA_ALG_SHA_256, received_data, sizeof(received_data), hash,
				  sizeof(hash), &hash_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to hash modified data: %ld\n", (long)status);
		goto cleanup;
	}
	status = psa_verify_hash(public_key_id, SIGN_ALG, hash, hash_length, signature,
				 signature_length);
	if (status != PSA_ERROR_INVALID_SIGNATURE) {
		printf("<error> expected invalid signature, got: %ld\n", (long)status);
		if (status == PSA_SUCCESS) {
			status = PSA_ERROR_GENERIC_ERROR;
		}
		goto cleanup;
	}
	printf("<info> modified data rejected as expected\n");
	status = PSA_SUCCESS;

cleanup:
	psa_reset_key_attributes(&attributes);
	if (public_key_id != 0) {
		psa_status_t destroy_status = psa_destroy_key(public_key_id);
		if (status == PSA_SUCCESS) {
			status = destroy_status;
		}
	}
	if (private_key_id != 0) {
		psa_status_t destroy_status = psa_destroy_key(private_key_id);
		if (status == PSA_SUCCESS) {
			status = destroy_status;
		}
	}
	zassert_equal(status, PSA_SUCCESS, "RSA test failed: %ld", (long)status);
}

#if 0
ZTEST_USER(psa_crypto_test_suite, test_rsa_2048_gen_key)
{
	psa_status_t status;
	psa_key_id_t key = 0;
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;

	const uint8_t message[] = "Hello RSA";
	uint8_t hash[PSA_HASH_LENGTH(PSA_ALG_SHA_256)];
	uint8_t signature[PSA_SIGN_OUTPUT_SIZE(
		PSA_KEY_TYPE_RSA_KEY_PAIR, RSA_BITS_2048, SIGN_ALG)];

	/* Static buffers reduce test thread stack usage */
	static uint8_t public_key[PSA_EXPORT_KEY_OUTPUT_SIZE(
		PSA_KEY_TYPE_RSA_PUBLIC_KEY, RSA_BITS_2048)];
	static uint8_t private_key[PSA_EXPORT_KEY_OUTPUT_SIZE(
		PSA_KEY_TYPE_RSA_KEY_PAIR, RSA_BITS_2048)];

	size_t hash_length = 0;
	size_t signature_length = 0;
	size_t public_key_length = 0;
	size_t private_key_length = 0;

	status = psa_crypto_init();
	if (status != PSA_SUCCESS) {
		printf("<error> failed to init PSA Crypto: %ld\n",
		       (long)status);
		goto cleanup;
	}

	printf("<info> generate RSA key pair...\n");
	psa_set_key_type(&attributes, PSA_KEY_TYPE_RSA_KEY_PAIR);
	psa_set_key_bits(&attributes, RSA_BITS_2048);
	psa_set_key_usage_flags(&attributes,
			       PSA_KEY_USAGE_SIGN_HASH |
			       PSA_KEY_USAGE_VERIFY_HASH |
			       PSA_KEY_USAGE_EXPORT);
	psa_set_key_algorithm(&attributes, SIGN_ALG);

	status = psa_generate_key(&attributes, &key);
	psa_reset_key_attributes(&attributes);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to generate key: %ld\n",
		       (long)status);
		goto cleanup;
	}

	/* Export and print the public key */
	status = psa_export_public_key(key,
				       public_key, sizeof(public_key),
				       &public_key_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to export public key: %ld\n",
		       (long)status);
		goto cleanup;
	}

	print_hex("RSA public key (DER)", public_key, public_key_length);

	/* Export and print the private key */
	status = psa_export_key(key,
				private_key, sizeof(private_key),
				&private_key_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to export private key: %ld\n",
		       (long)status);
		goto cleanup;
	}

	print_hex("RSA private key (DER)", private_key, private_key_length);

	/* Calculate the message hash, excluding the terminating '\0' */
	printf("<info> compute SHA-256 hash...\n");
	status = psa_hash_compute(PSA_ALG_SHA_256,
				  message, sizeof(message) - 1,
				  hash, sizeof(hash), &hash_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to compute hash: %ld\n",
		       (long)status);
		goto cleanup;
	}

	printf("<info> sign using the private key...\n");
	status = psa_sign_hash(key, SIGN_ALG,
			       hash, hash_length,
			       signature, sizeof(signature),
			       &signature_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to sign hash: %ld\n",
		       (long)status);
		goto cleanup;
	}

	printf("<info> verify using the public key...\n");
	status = psa_verify_hash(key, SIGN_ALG,
				 hash, hash_length,
				 signature, signature_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to verify hash: %ld\n",
		       (long)status);
		goto cleanup;
	}

	printf("RSA sign/verify OK, signature: %zu bytes\n",
	       signature_length);

cleanup:
	/* Clear the exported private key from memory */
	{
		volatile uint8_t *p = private_key;

		for (size_t i = 0; i < sizeof(private_key); i++) {
			p[i] = 0;
		}
	}

	psa_reset_key_attributes(&attributes);

	if (key != 0) {
		psa_status_t destroy_status = psa_destroy_key(key);

		if (status == PSA_SUCCESS) {
			status = destroy_status;
		}
	}

	if (status != PSA_SUCCESS) {
		printf("PSA error: %ld\n", (long)status);
	}

	zassert_equal(status, PSA_SUCCESS);
}
#endif

ZTEST_USER(psa_crypto_test_suite, test_rsa_4096)
{
	psa_status_t status;
	psa_key_id_t private_key_id = 0;
	psa_key_id_t public_key_id = 0;
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
	const uint8_t message[] = "Hello RSA";
	uint8_t received_data[sizeof(message) - 1];
	uint8_t hash[PSA_HASH_LENGTH(PSA_ALG_SHA_256)];
	uint8_t signature[PSA_SIGN_OUTPUT_SIZE(PSA_KEY_TYPE_RSA_KEY_PAIR, RSA_BITS_4096, SIGN_ALG)];
	size_t hash_length = 0;
	size_t signature_length = 0;

	status = psa_crypto_init();
	if (status != PSA_SUCCESS) {
		printf("<error> failed to initialize PSA Crypto: %ld\n", (long)status);
		goto cleanup;
	}

	/* Import the private key instead of generating a key pair */
	psa_set_key_type(&attributes, PSA_KEY_TYPE_RSA_KEY_PAIR);
	psa_set_key_bits(&attributes, RSA_BITS_4096);
	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_HASH);
	psa_set_key_algorithm(&attributes, SIGN_ALG);
	status = psa_import_key(&attributes, rsa4096_private_key_der,
				sizeof(rsa4096_private_key_der), &private_key_id);
	psa_reset_key_attributes(&attributes);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to import private key: %ld\n", (long)status);
		goto cleanup;
	}

	/* Import a separate public key for verification */
	psa_set_key_type(&attributes, PSA_KEY_TYPE_RSA_PUBLIC_KEY);
	psa_set_key_bits(&attributes, RSA_BITS_4096);
	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_VERIFY_HASH);
	psa_set_key_algorithm(&attributes, SIGN_ALG);
	status = psa_import_key(&attributes, rsa4096_public_key_der, sizeof(rsa4096_public_key_der),
				&public_key_id);
	psa_reset_key_attributes(&attributes);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to import public key: %ld\n", (long)status);
		goto cleanup;
	}

#if 0
	print_hex("Message", message, sizeof(message));
#endif

	/* Hash the original message, excluding the terminating '\0' */
	status = psa_hash_compute(PSA_ALG_SHA_256, message, sizeof(message) - 1, hash, sizeof(hash),
				  &hash_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to hash original message: %ld\n", (long)status);
		goto cleanup;
	}

#if 0
	print_hex("Hash(SHA256)", hash, hash_length);
#endif

	/* Sign using the imported private key */
	status = psa_sign_hash(private_key_id, SIGN_ALG, hash, hash_length, signature,
			       sizeof(signature), &signature_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to sign hash: %ld\n", (long)status);
		goto cleanup;
	}

#if 0
	print_hex("Signature", signature, signature_length);
#endif

	/* Simulate receiving the message and independently hash it */
	memcpy(received_data, message, sizeof(received_data));
	status = psa_hash_compute(PSA_ALG_SHA_256, received_data, sizeof(received_data), hash,
				  sizeof(hash), &hash_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to hash received data: %ld\n", (long)status);
		goto cleanup;
	}

	/* Verify using the independently imported public key */
	status = psa_verify_hash(public_key_id, SIGN_ALG, hash, hash_length, signature,
				 signature_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to verify received data: %ld\n", (long)status);
		goto cleanup;
	}
	printf("<info> RSA sign/verify OK, signature: %zu bytes\n", signature_length);

	/* Modified data must fail verification */
	received_data[0] ^= 0x01;
	status = psa_hash_compute(PSA_ALG_SHA_256, received_data, sizeof(received_data), hash,
				  sizeof(hash), &hash_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to hash modified data: %ld\n", (long)status);
		goto cleanup;
	}
	status = psa_verify_hash(public_key_id, SIGN_ALG, hash, hash_length, signature,
				 signature_length);
	if (status != PSA_ERROR_INVALID_SIGNATURE) {
		printf("<error> expected invalid signature, got: %ld\n", (long)status);
		if (status == PSA_SUCCESS) {
			status = PSA_ERROR_GENERIC_ERROR;
		}
		goto cleanup;
	}
	printf("<info> modified data rejected as expected\n");
	status = PSA_SUCCESS;

cleanup:
	psa_reset_key_attributes(&attributes);
	if (public_key_id != 0) {
		psa_status_t destroy_status = psa_destroy_key(public_key_id);
		if (status == PSA_SUCCESS) {
			status = destroy_status;
		}
	}
	if (private_key_id != 0) {
		psa_status_t destroy_status = psa_destroy_key(private_key_id);
		if (status == PSA_SUCCESS) {
			status = destroy_status;
		}
	}
	zassert_equal(status, PSA_SUCCESS, "RSA test failed: %ld", (long)status);
}
#if 0
ZTEST_USER(psa_crypto_test_suite, test_rsa_4096_gen_key)
{
	psa_status_t status;
	psa_key_id_t key = 0;
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;

	const uint8_t message[] = "Hello RSA";
	uint8_t hash[PSA_HASH_LENGTH(PSA_ALG_SHA_256)];
	uint8_t signature[PSA_SIGN_OUTPUT_SIZE(
		PSA_KEY_TYPE_RSA_KEY_PAIR, RSA_BITS_4096, SIGN_ALG)];

	/* Static buffers reduce test thread stack usage */
	static uint8_t public_key[PSA_EXPORT_KEY_OUTPUT_SIZE(
		PSA_KEY_TYPE_RSA_PUBLIC_KEY, RSA_BITS_4096)];
	static uint8_t private_key[PSA_EXPORT_KEY_OUTPUT_SIZE(
		PSA_KEY_TYPE_RSA_KEY_PAIR, RSA_BITS_4096)];

	size_t hash_length = 0;
	size_t signature_length = 0;
	size_t public_key_length = 0;
	size_t private_key_length = 0;

	status = psa_crypto_init();
	if (status != PSA_SUCCESS) {
		printf("<error> failed to init PSA Crypto: %ld\n",
		       (long)status);
		goto cleanup;
	}

	printf("<info> generate RSA key pair...\n");
	psa_set_key_type(&attributes, PSA_KEY_TYPE_RSA_KEY_PAIR);
	psa_set_key_bits(&attributes, RSA_BITS_4096);
	psa_set_key_usage_flags(&attributes,
			       PSA_KEY_USAGE_SIGN_HASH |
			       PSA_KEY_USAGE_VERIFY_HASH |
			       PSA_KEY_USAGE_EXPORT);
	psa_set_key_algorithm(&attributes, SIGN_ALG);

	status = psa_generate_key(&attributes, &key);
	psa_reset_key_attributes(&attributes);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to generate key: %ld\n",
		       (long)status);
		goto cleanup;
	}

	/* Export and print the public key */
	status = psa_export_public_key(key,
				       public_key, sizeof(public_key),
				       &public_key_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to export public key: %ld\n",
		       (long)status);
		goto cleanup;
	}

	print_hex("RSA public key (DER)", public_key, public_key_length);

	/* Export and print the private key */
	status = psa_export_key(key,
				private_key, sizeof(private_key),
				&private_key_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to export private key: %ld\n",
		       (long)status);
		goto cleanup;
	}

	print_hex("RSA private key (DER)", private_key, private_key_length);

	/* Calculate the message hash, excluding the terminating '\0' */
	printf("<info> compute SHA-256 hash...\n");
	status = psa_hash_compute(PSA_ALG_SHA_256,
				  message, sizeof(message) - 1,
				  hash, sizeof(hash), &hash_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to compute hash: %ld\n",
		       (long)status);
		goto cleanup;
	}

	printf("<info> sign using the private key...\n");
	status = psa_sign_hash(key, SIGN_ALG,
			       hash, hash_length,
			       signature, sizeof(signature),
			       &signature_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to sign hash: %ld\n",
		       (long)status);
		goto cleanup;
	}

	printf("<info> verify using the public key...\n");
	status = psa_verify_hash(key, SIGN_ALG,
				 hash, hash_length,
				 signature, signature_length);
	if (status != PSA_SUCCESS) {
		printf("<error> failed to verify hash: %ld\n",
		       (long)status);
		goto cleanup;
	}

	printf("RSA sign/verify OK, signature: %zu bytes\n",
	       signature_length);

cleanup:
	/* Clear the exported private key from memory */
	{
		volatile uint8_t *p = private_key;

		for (size_t i = 0; i < sizeof(private_key); i++) {
			p[i] = 0;
		}
	}

	psa_reset_key_attributes(&attributes);

	if (key != 0) {
		psa_status_t destroy_status = psa_destroy_key(key);

		if (status == PSA_SUCCESS) {
			status = destroy_status;
		}
	}

	if (status != PSA_SUCCESS) {
		printf("PSA error: %ld\n", (long)status);
	}

	zassert_equal(status, PSA_SUCCESS);
}
#endif

ZTEST_SUITE(psa_crypto_test_suite, NULL, NULL, NULL, NULL, NULL);
