/*
 * Copyright (c) 2026 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IT51XXX_CRYPTO_RSA_H
#define IT51XXX_CRYPTO_RSA_H

/**
 * @file it51xxx_crypto_rsa.h
 *
 * This file contains the declaration of the entry points associated to the
 * asymmetric signature capability as described by the PSA Cryptoprocessor
 * Driver interface specification
 *
 */

#include "psa/crypto.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sign a precomputed hash of a message
 *
 * @param[in]  attributes       Attributes of the key to use
 * @param[in]  key_buffer       Key material buffer
 * @param[in]  key_buffer_size  Size in bytes of the key
 * @param[in]  alg              Algorithm to use
 * @param[in]  hash             Hash to sign buffer
 * @param[in]  hash_length      Size in bytes of the data to sign
 * @param[out] signature        Buffer to hold the signature data
 * @param[in]  signature_size   Size in bytes of the signature buffer
 * @param[out] signature_length Size in bytes of the signature
 *
 * @return  PSA_SUCCESS on success. Error code from \ref psa_status_t on
 *          failure
 */
psa_status_t ite_it51xxx_transparent_sign_hash(const psa_key_attributes_t *attributes,
					       const uint8_t *key_buffer, size_t key_buffer_size,
					       psa_algorithm_t alg, const uint8_t *hash,
					       size_t hash_length, uint8_t *signature,
					       size_t signature_size, size_t *signature_length);

/**
 * @brief Verify a message signature on a hash
 *
 * @param[in] attributes       Attributes of the key to use
 * @param[in] key_buffer       Key material buffer
 * @param[in] key_buffer_size  Size in bytes of the key
 * @param[in] alg              Algorithm to use
 * @param[in] hash             Hash to sign buffer
 * @param[in] hash_length      Size in bytes of the data to sign
 * @param[in] signature        Signature to verify
 * @param[in] signature_length Size in bytes of the signature
 *
 * @return  PSA_SUCCESS on success. Error code from \ref psa_status_t on
 *          failure
 */
psa_status_t ite_it51xxx_transparent_verify_hash(const psa_key_attributes_t *attributes,
						 const uint8_t *key_buffer, size_t key_buffer_size,
						 psa_algorithm_t alg, const uint8_t *hash,
						 size_t hash_length, const uint8_t *signature,
						 size_t signature_length);

#ifdef __cplusplus
}
#endif
#endif /* IT51XXX_CRYPTO_RSA_H */
