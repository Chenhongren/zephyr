# Copyright (c) 2024 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0


cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mbedtls)

zephyr_include_directories(include)

target_sources(app PRIVATE src/main.c)

zephyr_compile_definitions(PSA_CRYPTO_DRIVER_ITE_IT51XXX)

include(FetchContent)
include(remote_library.cmake)
set(FETCHCONTENT_QUIET FALSE)

# Default configuration of tf-psa-crypto-drivers repository
set(PSA_CRYPTO_DRIVER_PATH    "DOWNLOAD"             CACHE PATH   "Path to ite crypto driver")
set(PSA_CRYPTO_DRIVER_NAME    "vendor/riscv/it51xxx" CACHE STRING "PSA Crypto driver name")
set(PSA_CRYPTO_DRIVER_VERSION "it51xxx/psa-drivers"  CACHE STRING "Revision(Branch) to fetch")

fetch_remote_library(
	LIB_NAME                tf-psa-crypto-drivers
	LIB_SOURCE_PATH_VAR     PSA_CRYPTO_DRIVER_PATH
	LIB_PATCH_DIR           ${CMAKE_CURRENT_LIST_DIR}
	LIB_BASE_DIR            "${CMAKE_BINARY_DIR}/driver"
	FETCH_CONTENT_ARGS
		GIT_REPOSITORY      https://github.com/Chenhongren/tf-psa-crypto-drivers.git
		GIT_TAG             ${PSA_CRYPTO_DRIVER_VERSION}
		GIT_SHALLOW         FALSE
		GIT_PROGRESS        TRUE
)

set(ITE_PSA_CRYTPO_DRIVER_PATH ${PSA_CRYPTO_DRIVER_PATH}/${PSA_CRYPTO_DRIVER_NAME} CACHE PATH "ITE Crypto driver path")

add_subdirectory(${ITE_PSA_CRYTPO_DRIVER_PATH} ite-psa-crypto-drivers)

if(TARGET ite_it51xxx_psa_driver_api)
    message(STATUS "ite_it51xxx_psa_driver_api target exists")
else()
    message(STATUS "ite_it51xxx_psa_driver_api target does not exist yet")
endif()

target_link_libraries(mbedtls
    PUBLIC
        ite_it51xxx_psa_driver_api
        ${TF_PSA_CRYPTO_TARGET_PREFIX}tfpsacrypto
)
