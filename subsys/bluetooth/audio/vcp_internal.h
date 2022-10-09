/**
 * @file
 * @brief Internal Header for Bluetooth Volume Control Service (VCS).
 *
 * Copyright (c) 2020 Bose Corporation
 * Copyright (c) 2020-2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_AUDIO_VCP_INTERNAL_
#define ZEPHYR_INCLUDE_BLUETOOTH_AUDIO_VCP_INTERNAL_

/* VCS opcodes */
#define BT_VCP_OPCODE_REL_VOL_DOWN                      0x00
#define BT_VCP_OPCODE_REL_VOL_UP                        0x01
#define BT_VCP_OPCODE_UNMUTE_REL_VOL_DOWN               0x02
#define BT_VCP_OPCODE_UNMUTE_REL_VOL_UP                 0x03
#define BT_VCP_OPCODE_SET_ABS_VOL                       0x04
#define BT_VCP_OPCODE_UNMUTE                            0x05
#define BT_VCP_OPCODE_MUTE                              0x06

struct vcs_state {
	uint8_t volume;
	uint8_t mute;
	uint8_t change_counter;
} __packed;

struct vcs_control {
	uint8_t opcode;
	uint8_t counter;
} __packed;

struct vcs_control_vol {
	struct vcs_control cp;
	uint8_t volume;
} __packed;

#if defined(CONFIG_BT_VCP_CLIENT)
struct bt_vcp_client {
	struct vcs_state state;
	uint8_t flags;

	uint16_t start_handle;
	uint16_t end_handle;
	uint16_t state_handle;
	uint16_t control_handle;
	uint16_t flag_handle;
	struct bt_gatt_subscribe_params state_sub_params;
	struct bt_gatt_discover_params state_sub_disc_params;
	struct bt_gatt_subscribe_params flag_sub_params;
	struct bt_gatt_discover_params flag_sub_disc_params;
	bool cp_retried;

	bool busy;
	struct vcs_control_vol cp_val;
	struct bt_gatt_write_params write_params;
	struct bt_gatt_read_params read_params;
	struct bt_gatt_discover_params discover_params;
	struct bt_uuid_16 uuid;
	struct bt_conn *conn;

	uint8_t vocs_inst_cnt;
	struct bt_vocs *vocs[CONFIG_BT_VCP_CLIENT_MAX_VOCS_INST];
	uint8_t aics_inst_cnt;
	struct bt_aics *aics[CONFIG_BT_VCP_CLIENT_MAX_AICS_INST];
};
#endif /* CONFIG_BT_VCP_CLIENT */

#if defined(CONFIG_BT_VCP)
struct bt_vcp_server {
	struct vcs_state state;
	uint8_t flags;
	struct bt_vcp_cb *cb;
	uint8_t volume_step;

	struct bt_gatt_service *service_p;
	struct bt_vocs *vocs_insts[CONFIG_BT_VCP_VOCS_INSTANCE_COUNT];
	struct bt_aics *aics_insts[CONFIG_BT_VCP_AICS_INSTANCE_COUNT];
};
#endif /* CONFIG_BT_VCP */

/* Struct used as a common type for the api */
struct bt_vcp {
	bool client_instance;
	union {
#if defined(CONFIG_BT_VCP)
		struct bt_vcp_server srv;
#endif /* CONFIG_BT_VCP */
#if defined(CONFIG_BT_VCP_CLIENT)
		struct bt_vcp_client cli;
#endif /* CONFIG_BT_VCP_CLIENT */
	};
};

int bt_vcp_client_included_get(struct bt_vcp *vcp,
			       struct bt_vcp_included *included);
int bt_vcp_client_read_vol_state(struct bt_vcp *vcp);
int bt_vcp_client_read_flags(struct bt_vcp *vcp);
int bt_vcp_client_vol_down(struct bt_vcp *vcp);
int bt_vcp_client_vol_up(struct bt_vcp *vcp);
int bt_vcp_client_unmute_vol_down(struct bt_vcp *vcp);
int bt_vcp_client_unmute_vol_up(struct bt_vcp *vcp);
int bt_vcp_client_set_volume(struct bt_vcp *vcp, uint8_t volume);
int bt_vcp_client_unmute(struct bt_vcp *vcp);
int bt_vcp_client_mute(struct bt_vcp *vcp);

bool bt_vcp_client_valid_vocs_inst(struct bt_vcp *vcp, struct bt_vocs *vocs);
bool bt_vcp_client_valid_aics_inst(struct bt_vcp *vcp, struct bt_aics *aics);
#endif /* ZEPHYR_INCLUDE_BLUETOOTH_AUDIO_VCP_INTERNAL_*/
