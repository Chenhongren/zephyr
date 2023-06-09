# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sched_userspace_bench)

set_property(
   SOURCE src/user.c
   PROPERTY COMPILE_DEFINITIONS
   __ZEPHYR_USER__
)

set_property(
   SOURCE src/main.c
   PROPERTY COMPILE_DEFINITIONS
   __ZEPHYR_SUPERVISOR__
)

target_sources(app PRIVATE src/main.c src/user.c)

target_include_directories(app PRIVATE
  ${ZEPHYR_BASE}/kernel/include
  ${ZEPHYR_BASE}/arch/${ARCH}/include
  )
