# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)

target_sources_ifdef(CONFIG_ITE_2EVB_TEST app PRIVATE src/i3c_sample_2evb.c src/libi3c_controller.c src/libi3c_target.c)
target_sources_ifndef(CONFIG_ITE_2EVB_TEST app PRIVATE src/main.c)
