# Copyright (c) 2025 ITE Corporation.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ITE_I2C_CONTROLLER ite_i2c_controller.c)
zephyr_library_sources_ifdef(CONFIG_ITE_I2C_TARGET ite_i2c_target.c)
