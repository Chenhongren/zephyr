/*
 * Copyright The Mbed TLS Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IT51XXX_PSA_MBEDTLS_ORIGIN_H
#define IT51XXX_PSA_MBEDTLS_ORIGIN_H

#include "mbedtls/md.h"
#include "psa/crypto.h"

#include "mbedtls/private/bignum.h"
#include "psa_crypto_rsa.h"

/* Construct a PKCS v1.5 encoding of a hashed message
 *
 * This is used both for signature generation and verification.
 *
 * Parameters:
 * - md_alg:  Identifies the hash algorithm used to generate the given hash;
 *            MBEDTLS_MD_NONE if raw data is signed.
 * - hashlen: Length of hash in case hashlen is MBEDTLS_MD_NONE.
 * - hash:    Buffer containing the hashed message or the raw data.
 * - dst_len: Length of the encoded message.
 * - dst:     Buffer to hold the encoded message.
 *
 * Assumptions:
 * - hash has size hashlen if md_alg == MBEDTLS_MD_NONE.
 * - hash has size corresponding to md_alg if md_alg != MBEDTLS_MD_NONE.
 * - dst points to a buffer of size at least dst_len.
 *
 */
int rsa_rsassa_pkcs1_v15_encode(mbedtls_md_type_t md_alg, unsigned int hashlen,
				const unsigned char *hash, size_t dst_len, unsigned char *dst);

int rsa_pss_expected_salt_len(psa_algorithm_t alg, const mbedtls_rsa_context *rsa,
			      size_t hash_length);

int hash_mprime(const unsigned char *hash, size_t hlen, const unsigned char *salt, size_t slen,
		unsigned char *out, mbedtls_md_type_t md_alg);

int mgf_mask(unsigned char *dst, size_t dlen, unsigned char *src, size_t slen,
	     mbedtls_md_type_t md_alg);

int rsa_rsassa_pss_verify_check(psa_algorithm_t alg, mbedtls_rsa_context *rsa, unsigned int hashlen,
				const uint8_t *hash, int expected_salt_len, unsigned char *buf);

int rsa_rsassa_pss_sign_no_mode_check(mbedtls_rsa_context *ctx,
				      int (*f_rng)(void *, unsigned char *, size_t), void *p_rng,
				      mbedtls_md_type_t md_alg, unsigned int hashlen,
				      const unsigned char *hash, int saltlen, unsigned char *sig);

int mbedtls_ct_rsaes_pkcs1_v15_unpadding(unsigned char *input, size_t ilen, unsigned char *output,
					 size_t output_max_len, size_t *olen);

#endif /* IT51XXX_PSA_MBEDTLS_ORIGIN_H */
