/*
 * Copyright (c) 2026 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IT51XXX_CRYPTO_SHA_H
#define IT51XXX_CRYPTO_SHA_H

/** @file it51xxx_crypto_sha.h
 *
 * TODO
 *
 */

psa_status_t ite_it51xxx_transparent_hash_compute(psa_algorithm_t alg, const uint8_t *input,
						  size_t input_length, uint8_t *hash,
						  size_t hash_size, size_t *hash_length);

#ifdef __cplusplus
}
#endif
#endif /* IT51XXX_CRYPTO_RSA_H */
