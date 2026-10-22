# Copyright (c) 2026 ITE Corporation.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(crypto_rsa)

target_sources(app PRIVATE src/main.c)
