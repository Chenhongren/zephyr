# Copyright (c) 2025 ITE Corporation.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_I3C_CTRL_DUMMY i3c_ctrl_dummy.c)
zephyr_library_sources_ifdef(CONFIG_I3C_TGT_MQUEUE i3c_tgt_mqueue.c)
