#!/usr/bin/env bash
# Copyright 2023 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

# Compile all the applications needed by the bsim tests in these subfolders

#set -x #uncomment this line for debugging
set -ue

: "${BSIM_COMPONENTS_PATH:?BSIM_COMPONENTS_PATH must be defined}"
: "${ZEPHYR_BASE:?ZEPHYR_BASE must be set to point to the zephyr root\
 directory}"

WORK_DIR="${WORK_DIR:-${ZEPHYR_BASE}/bsim_out}"

BOARD_ROOT="${BOARD_ROOT:-${ZEPHYR_BASE}}"

mkdir -p ${WORK_DIR}

source ${ZEPHYR_BASE}/tests/bsim/compile.source

app=tests/bsim/bluetooth/host/gatt/authorization compile
app=tests/bsim/bluetooth/host/gatt/caching compile
app=tests/bsim/bluetooth/host/gatt/general compile
app=tests/bsim/bluetooth/host/gatt/notify compile
app=tests/bsim/bluetooth/host/gatt/notify_multiple compile
app=tests/bsim/bluetooth/host/gatt/settings compile
app=tests/bsim/bluetooth/host/gatt/settings conf_file=prj_2.conf compile
app=tests/bsim/bluetooth/host/gatt/ccc_store compile
app=tests/bsim/bluetooth/host/gatt/ccc_store conf_file=prj_2.conf compile
app=tests/bsim/bluetooth/host/gatt/sc_indicate compile

wait_for_background_jobs
