/*
 * Copyright The Mbed TLS Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define MBEDTLS_DECLARE_PRIVATE_IDENTIFIERS
#include "it51xxx_psa_mbedtls_origin.h"

#include "mbedtls/error.h"
#include "mbedtls/md.h"
#include "mbedtls/oid.h"
#include "mbedtls/psa_util.h"
#include "psa_crypto_rsa.h"
#include "constant_time_internal.h"
#include "mbedtls/constant_time.h"
#include <limits.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

extern int mbedtls_oid_get_oid_by_md(mbedtls_md_type_t md_alg, const char **oid, size_t *olen);

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
		 * 1-byte ASN.1 length encoding and that there's no overflow.
		 */
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
	 * and 8 bytes for the minimal padding
	 */
	if (nb_pad < 3 + 8) {
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	}
	nb_pad -= 3;

	/* Now nb_pad is the amount of memory to be filled
	 * with padding, and at least 8 bytes long.
	 */

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
	 * after the initial bounds check.
	 */
	if (p != dst + dst_len) {
		mbedtls_platform_zeroize(dst, dst_len);
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	}

	return 0;
}

int rsa_pss_expected_salt_len(psa_algorithm_t alg, const mbedtls_rsa_context *rsa,
			      size_t hash_length)
{
	if (PSA_ALG_IS_RSA_PSS_ANY_SALT(alg)) {
		return MBEDTLS_RSA_SALT_LEN_ANY;
	}
	/* Otherwise: standard salt length, i.e. largest possible salt length
	 * up to the hash length. */
	int klen = (int)mbedtls_rsa_get_len(rsa); // known to fit
	int hlen = (int)hash_length;              // known to fit
	int room = klen - 2 - hlen;
	if (room < 0) {
		return 0; // there is no valid signature in this case anyway
	} else if (room > hlen) {
		return hlen;
	} else {
		return room;
	}
}

/**
 * Generate and apply the MGF1 operation (from PKCS#1 v2.1) to a buffer.
 *
 * \param dst       buffer to mask
 * \param dlen      length of destination buffer
 * \param src       source of the mask generation
 * \param slen      length of the source buffer
 * \param md_alg    message digest to use
 */
int mgf_mask(unsigned char *dst, size_t dlen, unsigned char *src, size_t slen,
	     mbedtls_md_type_t md_alg)
{
	unsigned char counter[4];
	unsigned char *p;
	unsigned int hlen;
	size_t i, use_len;
	unsigned char mask[MBEDTLS_MD_MAX_SIZE];
	int ret = 0;
	const mbedtls_md_info_t *md_info;
	mbedtls_md_context_t md_ctx;

	mbedtls_md_init(&md_ctx);
	md_info = mbedtls_md_info_from_type(md_alg);
	if (md_info == NULL) {
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	}

	mbedtls_md_init(&md_ctx);
	if ((ret = mbedtls_md_setup(&md_ctx, md_info, 0)) != 0) {
		goto exit;
	}

	hlen = mbedtls_md_get_size(md_info);

	memset(mask, 0, sizeof(mask));
	memset(counter, 0, 4);

	/* Generate and apply dbMask */
	p = dst;

	while (dlen > 0) {
		use_len = hlen;
		if (dlen < hlen) {
			use_len = dlen;
		}

		if ((ret = mbedtls_md_starts(&md_ctx)) != 0) {
			goto exit;
		}
		if ((ret = mbedtls_md_update(&md_ctx, src, slen)) != 0) {
			goto exit;
		}
		if ((ret = mbedtls_md_update(&md_ctx, counter, 4)) != 0) {
			goto exit;
		}
		if ((ret = mbedtls_md_finish(&md_ctx, mask)) != 0) {
			goto exit;
		}

		for (i = 0; i < use_len; ++i) {
			*p++ ^= mask[i];
		}

		counter[3]++;

		dlen -= use_len;
	}

exit:
	mbedtls_platform_zeroize(mask, sizeof(mask));
	mbedtls_md_free(&md_ctx);

	return ret;
}

/**
 * Generate Hash(M') as in RFC 8017 page 43 points 5 and 6.
 *
 * \param hash      the input hash
 * \param hlen      length of the input hash
 * \param salt      the input salt
 * \param slen      length of the input salt
 * \param out       the output buffer - must be large enough for \p md_alg
 * \param md_alg    message digest to use
 */
int hash_mprime(const unsigned char *hash, size_t hlen, const unsigned char *salt, size_t slen,
		unsigned char *out, mbedtls_md_type_t md_alg)
{
	const unsigned char zeros[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	mbedtls_md_context_t md_ctx;
	int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;

	const mbedtls_md_info_t *md_info = mbedtls_md_info_from_type(md_alg);
	if (md_info == NULL) {
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	}

	mbedtls_md_init(&md_ctx);
	if ((ret = mbedtls_md_setup(&md_ctx, md_info, 0)) != 0) {
		goto exit;
	}
	if ((ret = mbedtls_md_starts(&md_ctx)) != 0) {
		goto exit;
	}
	if ((ret = mbedtls_md_update(&md_ctx, zeros, sizeof(zeros))) != 0) {
		goto exit;
	}
	if ((ret = mbedtls_md_update(&md_ctx, hash, hlen)) != 0) {
		goto exit;
	}
	if ((ret = mbedtls_md_update(&md_ctx, salt, slen)) != 0) {
		goto exit;
	}
	if ((ret = mbedtls_md_finish(&md_ctx, out)) != 0) {
		goto exit;
	}

exit:
	mbedtls_md_free(&md_ctx);

	return ret;
}

int rsa_rsassa_pss_sign_no_mode_check(mbedtls_rsa_context *ctx,
				      int (*f_rng)(void *, unsigned char *, size_t), void *p_rng,
				      mbedtls_md_type_t md_alg, unsigned int hashlen,
				      const unsigned char *hash, int saltlen, unsigned char *sig)
{
	size_t olen;
	unsigned char *p = sig;
	unsigned char *salt = NULL;
	size_t slen, min_slen, hlen, offset = 0;
	int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
	size_t msb;
	mbedtls_md_type_t hash_id;

	if ((md_alg != MBEDTLS_MD_NONE || hashlen != 0) && hash == NULL) {
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	}

	if (f_rng == NULL) {
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	}

	olen = ctx->MBEDTLS_PRIVATE(len);

	if (md_alg != MBEDTLS_MD_NONE) {
		/* Gather length of hash to sign */
		size_t exp_hashlen = mbedtls_md_get_size_from_type(md_alg);
		if (exp_hashlen == 0) {
			return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
		}

		if (hashlen != exp_hashlen) {
			return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
		}
	}

	hash_id = (mbedtls_md_type_t)ctx->MBEDTLS_PRIVATE(hash_id);
	if (hash_id == MBEDTLS_MD_NONE) {
		hash_id = md_alg;
	}
	hlen = mbedtls_md_get_size_from_type(hash_id);
	if (hlen == 0) {
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	}

	if (saltlen == MBEDTLS_RSA_SALT_LEN_ANY) {
		/* Calculate the largest possible salt length, up to the hash size.
		 * Normally this is the hash length, which is the maximum salt length
		 * according to FIPS 185-4 §5.5 (e) and common practice. If there is not
		 * enough room, use the maximum salt length that fits. The constraint is
		 * that the hash length plus the salt length plus 2 bytes must be at most
		 * the key length. This complies with FIPS 186-4 §5.5 (e) and RFC 8017
		 * (PKCS#1 v2.2) §9.1.1 step 3. */
		min_slen = hlen - 2;
		if (olen < hlen + min_slen + 2) {
			return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
		} else if (olen >= hlen + hlen + 2) {
			slen = hlen;
		} else {
			slen = olen - hlen - 2;
		}
	} else if ((saltlen < 0) || (saltlen + hlen + 2 > olen)) {
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	} else {
		slen = (size_t)saltlen;
	}

	memset(sig, 0, olen);

	/* Note: EMSA-PSS encoding is over the length of N - 1 bits */
	msb = mbedtls_mpi_bitlen(&ctx->MBEDTLS_PRIVATE(N)) - 1;
	p += olen - hlen - slen - 2;
	*p++ = 0x01;

	/* Generate salt of length slen in place in the encoded message */
	salt = p;
	if ((ret = f_rng(p_rng, salt, slen)) != 0) {
		return MBEDTLS_ERROR_ADD(MBEDTLS_ERR_RSA_RNG_FAILED, ret);
	}

	p += slen;

	/* Generate H = Hash( M' ) */
	ret = hash_mprime(hash, hashlen, salt, slen, p, hash_id);
	if (ret != 0) {
		return ret;
	}

	/* Compensate for boundary condition when applying mask */
	if (msb % 8 == 0) {
		offset = 1;
	}

	/* maskedDB: Apply dbMask to DB */
	ret = mgf_mask(sig + offset, olen - hlen - 1 - offset, p, hlen, hash_id);
	if (ret != 0) {
		return ret;
	}

	msb = mbedtls_mpi_bitlen(&ctx->MBEDTLS_PRIVATE(N)) - 1;
	sig[0] &= 0xFF >> (olen * 8 - msb);

	p += hlen;
	*p++ = 0xBC;

	return 0;
}

int rsa_rsassa_pss_verify_check(psa_algorithm_t alg, mbedtls_rsa_context *rsa, unsigned int hashlen,
				const uint8_t *hash, int expected_salt_len, unsigned char *buf)
{
	int ret;
	size_t siglen = rsa->MBEDTLS_PRIVATE(len);
	mbedtls_md_type_t md_alg = mbedtls_md_type_from_psa_alg(alg);
	unsigned char *p;
	unsigned char *hash_start;
	unsigned int hlen;
	unsigned char result[MBEDTLS_MD_MAX_SIZE];
	size_t observed_salt_len, msb;

	if ((md_alg != MBEDTLS_MD_NONE || hashlen != 0) && hash == NULL) {
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	}

	if (siglen < 16) {
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	}

	p = buf;

	if (buf[siglen - 1] != 0xBC) {
		return PSA_ERROR_INVALID_PADDING;
	}

	if (md_alg != MBEDTLS_MD_NONE) {
		/* Gather length of hash to sign */
		size_t exp_hashlen = mbedtls_md_get_size_from_type(md_alg);
		if (exp_hashlen == 0) {
			return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
		}

		if (hashlen != exp_hashlen) {
			return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
		}
	}

	hlen = mbedtls_md_get_size_from_type(md_alg);
	if (hlen == 0) {
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	}

	/*
	 * Note: EMSA-PSS verification is over the length of N - 1 bits
	 */
	msb = mbedtls_mpi_bitlen(&rsa->MBEDTLS_PRIVATE(N)) - 1;

	if (buf[0] >> (8 - siglen * 8 + msb)) {
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	}

	/* Compensate for boundary condition when applying mask */
	if (msb % 8 == 0) {
		p++;
		siglen -= 1;
	}

	if (siglen < hlen + 2) {
		return MBEDTLS_ERR_RSA_BAD_INPUT_DATA;
	}
	hash_start = p + siglen - hlen - 1;

	ret = mgf_mask(p, siglen - hlen - 1, hash_start, hlen, md_alg);
	if (ret) {
		return ret;
	}

	buf[0] &= 0xFF >> (siglen * 8 - msb);

	while (p < hash_start - 1 && *p == 0) {
		p++;
	}

	if (*p++ != 0x01) {
		return PSA_ERROR_INVALID_PADDING;
	}

	observed_salt_len = (size_t)(hash_start - p);

	if (expected_salt_len != MBEDTLS_RSA_SALT_LEN_ANY &&
	    observed_salt_len != (size_t)expected_salt_len) {
		return PSA_ERROR_INVALID_PADDING;
	}

	/*
	 * Generate H = Hash( M' )
	 */
	ret = hash_mprime(hash, hashlen, p, observed_salt_len, result, md_alg);
	if (ret) {
		return ret;
	}

	if (memcmp(hash_start, result, hlen) != 0) {
		return MBEDTLS_ERR_RSA_VERIFY_FAILED;
	}

	return ret;
}

int mbedtls_ct_rsaes_pkcs1_v15_unpadding(unsigned char *input, size_t ilen, unsigned char *output,
					 size_t output_max_len, size_t *olen)
{
	int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
	size_t i, plaintext_max_size;

	/* The following variables take sensitive values: their value must
	 * not leak into the observable behavior of the function other than
	 * the designated outputs (output, olen, return value). Otherwise
	 * this would open the execution of the function to
	 * side-channel-based variants of the Bleichenbacher padding oracle
	 * attack. Potential side channels include overall timing, memory
	 * access patterns (especially visible to an adversary who has access
	 * to a shared memory cache), and branches (especially visible to
	 * an adversary who has access to a shared code cache or to a shared
	 * branch predictor). */
	size_t pad_count = 0;
	mbedtls_ct_condition_t bad;
	mbedtls_ct_condition_t pad_done;
	size_t plaintext_size = 0;
	mbedtls_ct_condition_t output_too_large;

	plaintext_max_size = (output_max_len > ilen - 11) ? ilen - 11 : output_max_len;

	/* Check and get padding length in constant time and constant
	 * memory trace. The first byte must be 0. */
	bad = mbedtls_ct_bool(input[0]);

	/* Decode EME-PKCS1-v1_5 padding: 0x00 || 0x02 || PS || 0x00
	 * where PS must be at least 8 nonzero bytes. */
	bad = mbedtls_ct_bool_or(bad, mbedtls_ct_uint_ne(input[1], MBEDTLS_RSA_CRYPT));

	/* Read the whole buffer. Set pad_done to nonzero if we find
	 * the 0x00 byte and remember the padding length in pad_count. */
	pad_done = MBEDTLS_CT_FALSE;
	for (i = 2; i < ilen; i++) {
		mbedtls_ct_condition_t found = mbedtls_ct_uint_eq(input[i], 0);
		pad_done = mbedtls_ct_bool_or(pad_done, found);
		pad_count += mbedtls_ct_uint_if_else_0(mbedtls_ct_bool_not(pad_done), 1);
	}

	/* If pad_done is still zero, there's no data, only unfinished padding. */
	bad = mbedtls_ct_bool_or(bad, mbedtls_ct_bool_not(pad_done));

	/* There must be at least 8 bytes of padding. */
	bad = mbedtls_ct_bool_or(bad, mbedtls_ct_uint_gt(8, pad_count));

	/* If the padding is valid, set plaintext_size to the number of
	 * remaining bytes after stripping the padding. If the padding
	 * is invalid, avoid leaking this fact through the size of the
	 * output: use the maximum message size that fits in the output
	 * buffer. Do it without branches to avoid leaking the padding
	 * validity through timing. RSA keys are small enough that all the
	 * size_t values involved fit in unsigned int. */
	plaintext_size = mbedtls_ct_uint_if(bad, (unsigned)plaintext_max_size,
					    (unsigned)(ilen - pad_count - 3));

	/* Set output_too_large to 0 if the plaintext fits in the output
	 * buffer and to 1 otherwise. */
	output_too_large = mbedtls_ct_uint_gt(plaintext_size, plaintext_max_size);

	/* Set ret without branches to avoid timing attacks. Return:
	 * - INVALID_PADDING if the padding is bad (bad != 0).
	 * - OUTPUT_TOO_LARGE if the padding is good but the decrypted
	 *   plaintext does not fit in the output buffer.
	 * - 0 if the padding is correct. */
	ret = mbedtls_ct_error_if(
		bad, PSA_ERROR_INVALID_PADDING,
		mbedtls_ct_error_if_else_0(output_too_large, PSA_ERROR_BUFFER_TOO_SMALL));

	/* If the padding is bad or the plaintext is too large, zero the
	 * data that we're about to copy to the output buffer.
	 * We need to copy the same amount of data
	 * from the same buffer whether the padding is good or not to
	 * avoid leaking the padding validity through overall timing or
	 * through memory or cache access patterns. */
	mbedtls_ct_zeroize_if(mbedtls_ct_bool_or(bad, output_too_large), input + 11, ilen - 11);

	/* If the plaintext is too large, truncate it to the buffer size.
	 * Copy anyway to avoid revealing the length through timing, because
	 * revealing the length is as bad as revealing the padding validity
	 * for a Bleichenbacher attack. */
	plaintext_size = mbedtls_ct_uint_if(output_too_large, (unsigned)plaintext_max_size,
					    (unsigned)plaintext_size);

	/* Move the plaintext to the leftmost position where it can start in
	 * the working buffer, i.e. make it start plaintext_max_size from
	 * the end of the buffer. Do this with a memory access trace that
	 * does not depend on the plaintext size. After this move, the
	 * starting location of the plaintext is no longer sensitive
	 * information. */
	mbedtls_ct_memmove_left(input + ilen - plaintext_max_size, plaintext_max_size,
				plaintext_max_size - plaintext_size);

	/* Finally copy the decrypted plaintext plus trailing zeros into the output
	 * buffer. If output_max_len is 0, then output may be an invalid pointer
	 * and the result of memcpy() would be undefined; prevent undefined
	 * behavior making sure to depend only on output_max_len (the size of the
	 * user-provided output buffer), which is independent from plaintext
	 * length, validity of padding, success of the decryption, and other
	 * secrets. */
	if (output_max_len != 0) {
		memcpy(output, input + ilen - plaintext_max_size, plaintext_max_size);
	}

	/* Report the amount of data we copied to the output buffer. In case
	 * of errors (bad padding or output too large), the value of *olen
	 * when this function returns is not specified. Making it equivalent
	 * to the good case limits the risks of leaking the padding validity. */
	*olen = plaintext_size;

	return ret;
}
