/* bttester.h - Bluetooth tester headers */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 * Copyright (c) 2022 Codecoup
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/addr.h>
#include <sys/types.h>

static inline void tester_set_bit(uint8_t *addr, unsigned int bit)
{
	uint8_t *p = addr + (bit / 8U);

	*p |= BIT(bit % 8);
}

static inline uint8_t tester_test_bit(const uint8_t *addr, unsigned int bit)
{
	const uint8_t *p = addr + (bit / 8U);

	return *p & BIT(bit % 8);
}

void tester_init(void);
void tester_rsp(uint8_t service, uint8_t opcode, uint8_t index, uint8_t status);
void tester_send(uint8_t service, uint8_t opcode, uint8_t index, uint8_t *data,
		 size_t len);

struct btp_handler {
	uint8_t opcode;
	ssize_t expect_len;
	uint8_t (*func)(uint8_t index, const void *cmd, uint16_t cmd_len,
			void *rsp, uint16_t *rsp_len);
};

void tester_register_command_handlers(uint8_t service,
				      const struct btp_handler *handlers,
				      size_t num);

uint8_t tester_init_gatt(void);
uint8_t tester_unregister_gatt(void);

uint8_t tester_init_l2cap(void);
uint8_t tester_unregister_l2cap(void);

uint8_t tester_init_mesh(void);
uint8_t tester_unregister_mesh(void);
void tester_handle_mesh(uint8_t opcode, uint8_t index, uint8_t *data, uint16_t len);

uint8_t tester_init_vcp(void);
uint8_t tester_unregister_vcp(void);

uint8_t tester_init_vcs(void);
uint8_t tester_unregister_vcs(void);

uint8_t tester_init_aics(void);
uint8_t tester_unregister_aics(void);

uint8_t tester_init_vocs(void);
uint8_t tester_unregister_vocs(void);

uint8_t tester_init_ias(void);
uint8_t tester_unregister_ias(void);

uint8_t tester_init_gap(void);
uint8_t tester_unregister_gap(void);

void tester_init_core(void);

uint8_t tester_init_bap(void);
uint8_t tester_unregister_bap(void);
void tester_handle_pacs(uint8_t opcode, uint8_t index, uint8_t *data,
			uint16_t len);
