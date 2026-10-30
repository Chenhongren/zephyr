/*
 *  The RSA public-key cryptosystem
 *
 *  Copyright The Mbed TLS Contributors
 *  SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later
 */

#include "it51xxx_psa_mbedtls_origin.h"

#include "psa_crypto_rsa.h"

#include "mbedtls/error.h"
#include "mbedtls/oid.h"
#include "mbedtls/psa_util.h"

#include <limits.h>
#include <string.h>

extern int mbedtls_oid_get_oid_by_md(mbedtls_md_type_t md_alg, const char **oid, size_t *olen);

/* Decode the hash algorithm from alg and store the mbedtls encoding in
 * md_alg. Verify that the hash length is acceptable. */
psa_status_t psa_rsa_decode_md_type(psa_algorithm_t alg, size_t hash_length,
				    mbedtls_md_type_t *md_alg)
{
	psa_algorithm_t hash_alg = PSA_ALG_SIGN_GET_HASH(alg);
	*md_alg = mbedtls_md_type_from_psa_alg(hash_alg);

	/* The Mbed TLS RSA module uses an unsigned int for hash length
	 * parameters. Validate that it fits so that we don't risk an
	 * overflow later. */
#if SIZE_MAX > UINT_MAX
	if (hash_length > UINT_MAX) {
		return PSA_ERROR_INVALID_ARGUMENT;
	}
#endif

	/* For signatures using a hash, the hash length must be correct. */
	if (alg != PSA_ALG_RSA_PKCS1V15_SIGN_RAW) {
		if (*md_alg == MBEDTLS_MD_NONE) {
			return PSA_ERROR_NOT_SUPPORTED;
		}
		if (mbedtls_md_get_size_from_type(*md_alg) != hash_length) {
			return PSA_ERROR_INVALID_ARGUMENT;
		}
	}

	return PSA_SUCCESS;
}

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
				const unsigned char *hash, size_t dst_len, unsigned char *dst)
{
	size_t oid_size = 0;
	size_t nb_pad = dst_len;
	unsigned char *p = dst;
	const char *oid = NULL;

	/* Are we signing hashed or raw data? */
	if (md_alg != MBEDTLS_MD_NONE) {
		unsigned char md_size = mbedtls_md_get_size_from_type(md_alg);
		if (md_size == 0) {
			return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
		}

		if (mbedtls_oid_get_oid_by_md(md_alg, &oid, &oid_size) != 0) {
			return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
		}

		if (hashlen != md_size) {
			return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
		}

		/* Double-check that 8 + hashlen + oid_size can be used as a
		 * 1-byte ASN.1 length encoding and that there's no overflow. */
		if (8 + hashlen + oid_size >= 0x80 || 10 + hashlen < hashlen ||
		    10 + hashlen + oid_size < 10 + hashlen) {
			return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
		}

		/*
		 * Static bounds check:
		 * - Need 10 bytes for five tag-length pairs.
		 *   (Insist on 1-byte length encodings to protect against variants of
		 *    Bleichenbacher's forgery attack against lax PKCS#1v1.5 verification)
		 * - Need hashlen bytes for hash
		 * - Need oid_size bytes for hash alg OID.
		 */
		if (nb_pad < 10 + hashlen + oid_size) {
			return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
		}
		nb_pad -= 10 + hashlen + oid_size;
	} else {
		if (nb_pad < hashlen) {
			return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
		}

		nb_pad -= hashlen;
	}

	/* Need space for signature header and padding delimiter (3 bytes),
	 * and 8 bytes for the minimal padding */
	if (nb_pad < 3 + 8) {
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	}
	nb_pad -= 3;

	/* Now nb_pad is the amount of memory to be filled
	 * with padding, and at least 8 bytes long. */

	/* Write signature header and padding */
	*p++ = 0;
	*p++ = MBEDTLS_RSA_SIGN;
	memset(p, 0xFF, nb_pad);
	p += nb_pad;
	*p++ = 0;

	/* Are we signing raw data? */
	if (md_alg == MBEDTLS_MD_NONE) {
		memcpy(p, hash, hashlen);
		return 0;
	}

	/* Signing hashed data, add corresponding ASN.1 structure
	 *
	 * DigestInfo ::= SEQUENCE {
	 *   digestAlgorithm DigestAlgorithmIdentifier,
	 *   digest Digest }
	 * DigestAlgorithmIdentifier ::= AlgorithmIdentifier
	 * Digest ::= OCTET STRING
	 *
	 * Schematic:
	 * TAG-SEQ + LEN [ TAG-SEQ + LEN [ TAG-OID  + LEN [ OID  ]
	 *                                 TAG-NULL + LEN [ NULL ] ]
	 *                 TAG-OCTET + LEN [ HASH ] ]
	 */
	*p++ = MBEDTLS_ASN1_SEQUENCE | MBEDTLS_ASN1_CONSTRUCTED;
	*p++ = (unsigned char)(0x08 + oid_size + hashlen);
	*p++ = MBEDTLS_ASN1_SEQUENCE | MBEDTLS_ASN1_CONSTRUCTED;
	*p++ = (unsigned char)(0x04 + oid_size);
	*p++ = MBEDTLS_ASN1_OID;
	*p++ = (unsigned char)oid_size;
	memcpy(p, oid, oid_size);
	p += oid_size;
	*p++ = MBEDTLS_ASN1_NULL;
	*p++ = 0x00;
	*p++ = MBEDTLS_ASN1_OCTET_STRING;
	*p++ = (unsigned char)hashlen;
	memcpy(p, hash, hashlen);
	p += hashlen;

	/* Just a sanity-check, should be automatic
	 * after the initial bounds check. */
	if (p != dst + dst_len) {
		mbedtls_platform_zeroize(dst, dst_len);
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	}

	return 0;
}
