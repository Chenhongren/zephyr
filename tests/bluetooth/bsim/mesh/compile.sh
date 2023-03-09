#!/usr/bin/env bash
# Copyright 2023 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

# Compile all the applications needed by the bsim tests

#set -x #uncomment this line for debugging
set -ue

: "${BSIM_OUT_PATH:?BSIM_OUT_PATH must be defined}"
: "${BSIM_COMPONENTS_PATH:?BSIM_COMPONENTS_PATH must be defined}"
: "${ZEPHYR_BASE:?ZEPHYR_BASE must be set to point to the zephyr root\
 directory}"

WORK_DIR="${WORK_DIR:-${ZEPHYR_BASE}/bsim_out}"
BOARD="${BOARD:-nrf52_bsim}"
BOARD_ROOT="${BOARD_ROOT:-${ZEPHYR_BASE}}"

mkdir -p ${WORK_DIR}

source ${ZEPHYR_BASE}/tests/bluetooth/bsim/compile.source

app=tests/bluetooth/bsim/mesh compile
app=tests/bluetooth/bsim/mesh conf_overlay=overlay_low_lat.conf compile
app=tests/bluetooth/bsim/mesh conf_overlay=overlay_pst.conf compile
app=tests/bluetooth/bsim/mesh conf_overlay=overlay_gatt.conf compile
app=tests/bluetooth/bsim/mesh conf_file=prj_mesh1d1.conf compile
app=tests/bluetooth/bsim/mesh \
  conf_file=prj_mesh1d1.conf conf_overlay=overlay_pst.conf compile
app=tests/bluetooth/bsim/mesh \
  conf_file=prj_mesh1d1.conf conf_overlay=overlay_gatt.conf compile
app=tests/bluetooth/bsim/mesh \
  conf_file=prj_mesh1d1.conf conf_overlay=overlay_low_lat.conf compile

wait_for_background_jobs
