/* gap.h - Bluetooth tester headers */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 * Copyright (c) 2022 Codecoup
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/addr.h>

/* GAP Service */
/* commands */
#define BTP_GAP_READ_SUPPORTED_COMMANDS		0x01
struct btp_gap_read_supported_commands_rp {
	uint8_t data[0];
} __packed;

#define BTP_GAP_READ_CONTROLLER_INDEX_LIST	0x02
struct btp_gap_read_controller_index_list_rp {
	uint8_t num;
	uint8_t index[];
} __packed;

#define BTP_GAP_SETTINGS_POWERED		0
#define BTP_GAP_SETTINGS_CONNECTABLE		1
#define BTP_GAP_SETTINGS_FAST_CONNECTABLE	2
#define BTP_GAP_SETTINGS_DISCOVERABLE		3
#define BTP_GAP_SETTINGS_BONDABLE		4
#define BTP_GAP_SETTINGS_LINK_SEC_3		5
#define BTP_GAP_SETTINGS_SSP			6
#define BTP_GAP_SETTINGS_BREDR			7
#define BTP_GAP_SETTINGS_HS			8
#define BTP_GAP_SETTINGS_LE			9
#define BTP_GAP_SETTINGS_ADVERTISING		10
#define BTP_GAP_SETTINGS_SC			11
#define BTP_GAP_SETTINGS_DEBUG_KEYS		12
#define BTP_GAP_SETTINGS_PRIVACY		13
#define BTP_GAP_SETTINGS_CONTROLLER_CONFIG	14
#define BTP_GAP_SETTINGS_STATIC_ADDRESS		15

#define BTP_GAP_READ_CONTROLLER_INFO		0x03
struct btp_gap_read_controller_info_rp {
	uint8_t  address[6];
	uint32_t supported_settings;
	uint32_t current_settings;
	uint8_t  cod[3];
	uint8_t  name[249];
	uint8_t  short_name[11];
} __packed;

#define BTP_GAP_RESET				0x04
struct btp_gap_reset_rp {
	uint32_t current_settings;
} __packed;

#define BTP_GAP_SET_POWERED			0x05
struct btp_gap_set_powered_cmd {
	uint8_t powered;
} __packed;
struct btp_gap_set_powered_rp {
	uint32_t current_settings;
} __packed;

#define BTP_GAP_SET_CONNECTABLE			0x06
struct btp_gap_set_connectable_cmd {
	uint8_t connectable;
} __packed;
struct btp_gap_set_connectable_rp {
	uint32_t current_settings;
} __packed;

#define BTP_GAP_SET_FAST_CONNECTABLE		0x07
struct btp_gap_set_fast_connectable_cmd {
	uint8_t fast_connectable;
} __packed;
struct btp_gap_set_fast_connectable_rp {
	uint32_t current_settings;
} __packed;

#define BTP_GAP_NON_DISCOVERABLE		0x00
#define BTP_GAP_GENERAL_DISCOVERABLE		0x01
#define BTP_GAP_LIMITED_DISCOVERABLE		0x02

#define BTP_GAP_SET_DISCOVERABLE		0x08
struct btp_gap_set_discoverable_cmd {
	uint8_t discoverable;
} __packed;
struct btp_gap_set_discoverable_rp {
	uint32_t current_settings;
} __packed;

#define BTP_GAP_SET_BONDABLE			0x09
struct btp_gap_set_bondable_cmd {
	uint8_t bondable;
} __packed;
struct btp_gap_set_bondable_rp {
	uint32_t current_settings;
} __packed;

#define BTP_GAP_START_ADVERTISING		0x0a
struct btp_gap_start_advertising_cmd {
	uint8_t adv_data_len;
	uint8_t scan_rsp_len;
	uint8_t adv_sr_data[];
} __packed;
struct btp_gap_start_advertising_rp {
	uint32_t current_settings;
} __packed;

#define BTP_GAP_STOP_ADVERTISING		0x0b
struct btp_gap_stop_advertising_rp {
	uint32_t current_settings;
} __packed;

#define BTP_GAP_DISCOVERY_FLAG_LE		0x01
#define BTP_GAP_DISCOVERY_FLAG_BREDR		0x02
#define BTP_GAP_DISCOVERY_FLAG_LIMITED		0x04
#define BTP_GAP_DISCOVERY_FLAG_LE_ACTIVE_SCAN	0x08
#define BTP_GAP_DISCOVERY_FLAG_LE_OBSERVE	0x10
#define BTP_GAP_DISCOVERY_FLAG_OWN_ID_ADDR	0x20

#define BTP_GAP_START_DISCOVERY			0x0c
struct btp_gap_start_discovery_cmd {
	uint8_t flags;
} __packed;

#define BTP_GAP_STOP_DISCOVERY			0x0d

#define BTP_GAP_CONNECT				0x0e
struct btp_gap_connect_cmd {
	uint8_t address_type;
	uint8_t address[6];
} __packed;

#define BTP_GAP_DISCONNECT			0x0f
struct btp_gap_disconnect_cmd {
	uint8_t  address_type;
	uint8_t  address[6];
} __packed;

#define BTP_GAP_IO_CAP_DISPLAY_ONLY		0
#define BTP_GAP_IO_CAP_DISPLAY_YESNO		1
#define BTP_GAP_IO_CAP_KEYBOARD_ONLY		2
#define BTP_GAP_IO_CAP_NO_INPUT_OUTPUT		3
#define BTP_GAP_IO_CAP_KEYBOARD_DISPLAY		4

#define BTP_GAP_SET_IO_CAP			0x10
struct btp_gap_set_io_cap_cmd {
	uint8_t io_cap;
} __packed;

#define BTP_GAP_PAIR				0x11
struct btp_gap_pair_cmd {
	uint8_t address_type;
	uint8_t address[6];
} __packed;

#define BTP_GAP_UNPAIR				0x12
struct btp_gap_unpair_cmd {
	uint8_t address_type;
	uint8_t address[6];
} __packed;

#define BTP_GAP_PASSKEY_ENTRY			0x13
struct btp_gap_passkey_entry_cmd {
	uint8_t  address_type;
	uint8_t  address[6];
	uint32_t passkey;
} __packed;

#define BTP_GAP_PASSKEY_CONFIRM			0x14
struct btp_gap_passkey_confirm_cmd {
	uint8_t address_type;
	uint8_t address[6];
	uint8_t match;
} __packed;

#define BTP_GAP_START_DIRECTED_ADV_HD		BIT(0)
#define BTP_GAP_START_DIRECTED_ADV_OWN_ID	BIT(1)
#define BTP_GAP_START_DIRECTED_ADV_PEER_RPA	BIT(2)

#define BTP_GAP_START_DIRECTED_ADV		0x15
struct btp_gap_start_directed_adv_cmd {
	uint8_t address_type;
	uint8_t address[6];
	uint16_t options;
} __packed;
struct btp_gap_start_directed_adv_rp {
	uint32_t current_settings;
} __packed;

#define BTP_GAP_CONN_PARAM_UPDATE		0x16
struct btp_gap_conn_param_update_cmd {
	uint8_t address_type;
	uint8_t address[6];
	uint16_t interval_min;
	uint16_t interval_max;
	uint16_t latency;
	uint16_t timeout;
} __packed;

#define BTP_GAP_PAIRING_CONSENT			0x17
struct btp_gap_pairing_consent_cmd {
	uint8_t address_type;
	uint8_t address[6];
	uint8_t consent;
} __packed;

#define BTP_GAP_OOB_LEGACY_SET_DATA		0x18
struct btp_gap_oob_legacy_set_data_cmd {
	uint8_t oob_data[16];
} __packed;

#define BTP_GAP_OOB_SC_GET_LOCAL_DATA		0x19
struct btp_gap_oob_sc_get_local_data_rp {
	uint8_t rand[16];
	uint8_t conf[16];
} __packed;

#define BTP_GAP_OOB_SC_SET_REMOTE_DATA		0x1a
struct btp_gap_oob_sc_set_remote_data_cmd {
	uint8_t rand[16];
	uint8_t conf[16];
} __packed;

#define BTP_GAP_SET_MITM			0x1b
struct btp_gap_set_mitm {
	uint8_t mitm;
} __packed;

#define BTP_GAP_SET_FILTER_LIST			0x1c
struct btp_gap_set_filter_list {
	uint8_t cnt;
	bt_addr_le_t addr[0];
} __packed;

/* events */
#define BTP_GAP_EV_NEW_SETTINGS			0x80
struct btp_gap_new_settings_ev {
	uint32_t current_settings;
} __packed;

#define BTP_GAP_DEVICE_FOUND_FLAG_RSSI		0x01
#define BTP_GAP_DEVICE_FOUND_FLAG_AD		0x02
#define BTP_GAP_DEVICE_FOUND_FLAG_SD		0x04

#define BTP_GAP_EV_DEVICE_FOUND			0x81
struct btp_gap_device_found_ev {
	uint8_t  address_type;
	uint8_t  address[6];
	int8_t   rssi;
	uint8_t  flags;
	uint16_t eir_data_len;
	uint8_t  eir_data[];
} __packed;

#define BTP_GAP_EV_DEVICE_CONNECTED		0x82
struct btp_gap_device_connected_ev {
	uint8_t address_type;
	uint8_t address[6];
	uint16_t interval;
	uint16_t latency;
	uint16_t timeout;
} __packed;

#define BTP_GAP_EV_DEVICE_DISCONNECTED		0x83
struct btp_gap_device_disconnected_ev {
	uint8_t address_type;
	uint8_t address[6];
} __packed;

#define BTP_GAP_EV_PASSKEY_DISPLAY		0x84
struct btp_gap_passkey_display_ev {
	uint8_t  address_type;
	uint8_t  address[6];
	uint32_t passkey;
} __packed;

#define BTP_GAP_EV_PASSKEY_ENTRY_REQ		0x85
struct btp_gap_passkey_entry_req_ev {
	uint8_t address_type;
	uint8_t address[6];
} __packed;

#define BTP_GAP_EV_PASSKEY_CONFIRM_REQ		0x86
struct btp_gap_passkey_confirm_req_ev {
	uint8_t  address_type;
	uint8_t  address[6];
	uint32_t passkey;
} __packed;

#define BTP_GAP_EV_IDENTITY_RESOLVED		0x87
struct btp_gap_identity_resolved_ev {
	uint8_t address_type;
	uint8_t address[6];
	uint8_t identity_address_type;
	uint8_t identity_address[6];
} __packed;

#define BTP_GAP_EV_CONN_PARAM_UPDATE		0x88
struct btp_gap_conn_param_update_ev {
	uint8_t address_type;
	uint8_t address[6];
	uint16_t interval;
	uint16_t latency;
	uint16_t timeout;
} __packed;

#define BTP_GAP_SEC_LEVEL_UNAUTH_ENC		0x01
#define BTP_GAP_SEC_LEVEL_AUTH_ENC		0x02
#define BTP_GAP_SEC_LEVEL_AUTH_SC		0x03

#define BTP_GAP_EV_SEC_LEVEL_CHANGED		0x89
struct btp_gap_sec_level_changed_ev {
	uint8_t address_type;
	uint8_t address[6];
	uint8_t sec_level;
} __packed;

#define BTP_GAP_EV_PAIRING_CONSENT_REQ		0x8a
struct btp_gap_pairing_consent_req_ev {
	uint8_t address_type;
	uint8_t address[6];
} __packed;

#define BTP_GAP_EV_BOND_LOST			0x8b
struct btp_gap_bond_lost_ev {
	uint8_t address_type;
	uint8_t address[6];
} __packed;

#define BTP_GAP_EV_PAIRING_FAILED		0x8c
struct btp_gap_bond_pairing_failed_ev {
	uint8_t address_type;
	uint8_t address[6];
	uint8_t reason;
} __packed;
