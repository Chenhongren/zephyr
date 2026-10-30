# Copyright (c) 2026 ITE Corporation.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

message(STATUS "The it51xxx psa crypto driver is included")

set(tfpsacrypto_target "${TF_PSA_CRYPTO_TARGET_PREFIX}tfpsacrypto")

zephyr_compile_definitions(
  PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT
  PSA_CRYPTO_DRIVER_PRESENT
  PSA_CRYPTO_DRIVER_ITE_IT51XXX
)

zephyr_compile_definitions_ifdef(
  CONFIG_IT51XXX_PSA_CRYPTO_RSA
  PSA_CRYPTO_DRIVER_ITE_IT51XXX_RSA
)

zephyr_compile_definitions_ifdef(
  CONFIG_IT51XXX_PSA_CRYPTO_SHA
  PSA_CRYPTO_DRIVER_ITE_IT51XXX_SHA
)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

zephyr_library_include_directories(
  "$<TARGET_PROPERTY:${tfpsacrypto_target},INCLUDE_DIRECTORIES>"
)

zephyr_library_sources(
  it51xxx_crypto.c
  it51xxx_psa_mbedtls_origin.c
)

zephyr_library_sources_ifdef(CONFIG_IT51XXX_PSA_CRYPTO_RSA it51xxx_crypto_rsa.c)
zephyr_library_sources_ifdef(CONFIG_IT51XXX_PSA_CRYPTO_SHA it51xxx_crypto_sha.c)
