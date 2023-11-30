/** @file
 *  @brief Internal APIs for Bluetooth MCP.
 */

/*
 * Copyright (c) 2019 - 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_AUDIO_MCP_INTERNAL_
#define ZEPHYR_INCLUDE_BLUETOOTH_AUDIO_MCP_INTERNAL_

#include <zephyr/types.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/ots.h>
#include "../services/ots/ots_client_internal.h"

struct mcs_instance_t *lookup_inst_by_conn(struct bt_conn *conn);

struct mcs_instance_t {
	uint16_t start_handle;
	uint16_t end_handle;
	uint16_t player_name_handle;
#ifdef CONFIG_BT_MCC_OTS
	uint16_t icon_obj_id_handle;
#endif /* CONFIG_BT_MCC_OTS */
	uint16_t icon_url_handle;
	uint16_t track_changed_handle;
	uint16_t track_title_handle;
	uint16_t track_duration_handle;
	uint16_t track_position_handle;
	uint16_t playback_speed_handle;
	uint16_t seeking_speed_handle;
#ifdef CONFIG_BT_MCC_OTS
	uint16_t segments_obj_id_handle;
	uint16_t current_track_obj_id_handle;
	uint16_t next_track_obj_id_handle;
	uint16_t current_group_obj_id_handle;
	uint16_t parent_group_obj_id_handle;
#endif /* CONFIG_BT_MCC_OTS */
	uint16_t playing_order_handle;
	uint16_t playing_orders_supported_handle;
	uint16_t media_state_handle;
	uint16_t cp_handle;
	uint16_t opcodes_supported_handle;
#ifdef CONFIG_BT_MCC_OTS
	uint16_t scp_handle;
	uint16_t search_results_obj_id_handle;
#endif /* CONFIG_BT_MCC_OTS */
	uint16_t content_control_id_handle;


	/* The write buffer is used for
	 * - track position    (4 octets)
	 * - playback speed    (1 octet)
	 * - playing order     (1 octet)
	 * - the control point (5 octets)
	 *                     (1 octet opcode + optionally 4 octet param)
	 *                     (mpl_cmd.opcode + mpl_cmd.param)
	 * If the object transfer client is included, it is also used for
	 * - object IDs (6 octets - BT_OTS_OBJ_ID_SIZE) and
	 * - the search control point (64 octets - SEARCH_LEN_MAX)
	 *
	 * If there is no OTC, the largest is control point
	 * If OTC is included, the largest is the search control point
	 */
#ifdef CONFIG_BT_MCC_OTS
	char write_buf[SEARCH_LEN_MAX];
#else
	/* Trick to be able to use sizeof on members of a struct type */
	/* TODO: Rewrite the mpl_cmd to have the "use_param" parameter */
	/* separately, and the opcode and param alone as a struct */
	char write_buf[sizeof(((struct mpl_cmd *)0)->opcode) +
		       sizeof(((struct mpl_cmd *)0)->param)];
#endif /* CONFIG_BT_MCC_OTS */

	struct bt_gatt_discover_params  discover_params;
	struct bt_gatt_read_params      read_params;
	struct bt_gatt_write_params     write_params;

/** Any fields below here cannot be memset as part of a reset */
	bool busy;

	struct bt_gatt_subscribe_params player_name_sub_params;
	struct bt_gatt_subscribe_params track_changed_sub_params;
	struct bt_gatt_subscribe_params track_title_sub_params;
	struct bt_gatt_subscribe_params track_duration_sub_params;
	struct bt_gatt_subscribe_params track_position_sub_params;
	struct bt_gatt_subscribe_params playback_speed_sub_params;
	struct bt_gatt_subscribe_params seeking_speed_sub_params;
#ifdef CONFIG_BT_MCC_OTS
	struct bt_gatt_subscribe_params current_track_obj_sub_params;
	struct bt_gatt_subscribe_params next_track_obj_sub_params;
	struct bt_gatt_subscribe_params parent_group_obj_sub_params;
	struct bt_gatt_subscribe_params current_group_obj_sub_params;
#endif /* CONFIG_BT_MCC_OTS */
	struct bt_gatt_subscribe_params playing_order_sub_params;
	struct bt_gatt_subscribe_params media_state_sub_params;
	struct bt_gatt_subscribe_params cp_sub_params;
	struct bt_gatt_subscribe_params opcodes_supported_sub_params;
#ifdef CONFIG_BT_MCC_OTS
	struct bt_gatt_subscribe_params scp_sub_params;
	struct bt_gatt_subscribe_params search_results_obj_sub_params;
#endif /* CONFIG_BT_MCC_OTS */

#ifdef CONFIG_BT_MCC_OTS
	struct bt_ots_client otc;
#endif /* CONFIG_BT_MCC_OTS */
};

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_AUDIO_MCP_INTERNAL_ */
