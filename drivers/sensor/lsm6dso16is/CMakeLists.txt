# ST Microelectronics LSM6DSO16IS 6-axis IMU sensor driver
#
# Copyright (c) 2023 STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources(lsm6dso16is.c)
zephyr_library_sources_ifdef(CONFIG_LSM6DSO16IS_SENSORHUB  lsm6dso16is_shub.c)
zephyr_library_sources_ifdef(CONFIG_LSM6DSO16IS_TRIGGER    lsm6dso16is_trigger.c)

zephyr_library_include_directories(../stmemsc)
