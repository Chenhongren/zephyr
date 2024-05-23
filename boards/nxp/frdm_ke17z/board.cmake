# Copyright 2024 NXP
# SPDX-License-Identifier: Apache-2.0

board_runner_args(linkserver "--device=MKE17Z256xxx7:FRDM-KE17Z")
board_runner_args(jlink "--device=MKE17Z256xxx7" "--reset-after-load")

include(${ZEPHYR_BASE}/boards/common/linkserver.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
