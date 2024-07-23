/*
 * Copyright (c) 2017-2021 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

void bt_scan_reset(void);

bool bt_id_scan_random_addr_check(void);

int bt_le_scan_set_enable(uint8_t enable);

void bt_periodic_sync_disable(void);
