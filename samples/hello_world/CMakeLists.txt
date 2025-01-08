# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)

if (CONFIG_ITE_2EVB_TEST OR CONFIG_ITE_LPS22DF_TEST OR CONFIG_ITE_I2C_TEST)
target_sources(app PRIVATE src/ite_i3c_sample.c src/libi3c_controller.c src/libi3c_target.c)
else()
target_sources(app PRIVATE src/main.c)
endif() #CONFIG_ITE_2EVB_TEST
