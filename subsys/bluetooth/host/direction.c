/*
 * Copyright (c) 2020-2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include <assert.h>
#include <sys/check.h>
#include <sys/byteorder.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include <bluetooth/direction.h>

#include "hci_core.h"
#include "scan.h"
#include "conn_internal.h"
#include "direction_internal.h"

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_DF)
#define LOG_MODULE_NAME bt_df
#include "common/log.h"

/* @brief Antenna information for LE Direction Finding */
struct bt_le_df_ant_info {
	/* Bitfield holding optional switching and sampling rates */
	uint8_t switch_sample_rates;
	/* Available antennae number */
	uint8_t num_ant;
	/* Maximum supported antennae switching pattern length */
	uint8_t max_switch_pattern_len;
	/* Maximum length of CTE in 8[us] units */
	uint8_t max_cte_len;
};

static struct bt_le_df_ant_info df_ant_info;
#if defined(CONFIG_BT_DF_CONNECTIONLESS_CTE_RX) || defined(CONFIG_BT_DF_CONNECTION_CTE_RX)
const static uint8_t df_dummy_switch_pattern[BT_HCI_LE_SWITCH_PATTERN_LEN_MIN] = { 0, 0 };
#endif /* CONFIG_BT_DF_CONNECTIONLESS_CTE_RX || CONFIG_BT_DF_CONNECTION_CTE_RX */

#define DF_AOD_TX_1US_SUPPORT(supp)             (supp & BT_HCI_LE_1US_AOD_TX)
#define DF_AOD_RX_1US_SUPPORT(supp)             (supp & BT_HCI_LE_1US_AOD_RX)
#define DF_AOA_RX_1US_SUPPORT(supp)             (supp & BT_HCI_LE_1US_AOA_RX)

#define DF_SAMPLING_ANTENNA_NUMBER_MIN 0x2

#if defined(CONFIG_BT_DF_CONNECTIONLESS_CTE_RX) || defined(CONFIG_BT_DF_CONNECTION_CTE_RX)
static bool validate_cte_rx_common_params(uint8_t cte_type, uint8_t slot_durations,
					  uint8_t num_ant_ids, const uint8_t *ant_ids);
#endif /* CONFIG_BT_DF_CONNECTIONLESS_CTE_RX || CONFIG_BT_DF_CONNECTION_CTE_RX */

#if defined(CONFIG_BT_DF_CONNECTIONLESS_CTE_RX)
static bool validate_cl_cte_rx_params(const struct bt_df_per_adv_sync_cte_rx_param *params);
static void
prepare_cl_cte_rx_enable_cmd_params(struct net_buf *buf, struct bt_le_per_adv_sync *sync,
				    const struct bt_df_per_adv_sync_cte_rx_param *params,
				    bool enable);
static int hci_df_set_cl_cte_rx_enable(struct bt_le_per_adv_sync *sync, bool enable,
				       const struct bt_df_per_adv_sync_cte_rx_param *params);
#endif /* CONFIG_BT_DF_CONNECTIONLESS_CTE_RX */

#if defined(CONFIG_BT_DF_CONNECTION_CTE_RX)
static void prepare_conn_cte_rx_enable_cmd_params(struct net_buf *buf, struct bt_conn *conn,
						  const struct bt_df_conn_cte_rx_param *params,
						  bool enable);
static int hci_df_set_conn_cte_rx_enable(struct bt_conn *conn, bool enable,
					 const struct bt_df_conn_cte_rx_param *params);
#endif /* CONFIG_BT_DF_CONNECTION_CTE_RX */

static int hci_df_set_cl_cte_tx_params(const struct bt_le_ext_adv *adv,
				    const struct bt_df_adv_cte_tx_param *params)
{
	struct bt_hci_cp_le_set_cl_cte_tx_params *cp;
	struct net_buf *buf;

	/* If AoD is not enabled, ant_ids are ignored by controller:
	 * BT Core spec 5.2 Vol 4, Part E sec. 7.8.80.
	 */
	if (params->cte_type == BT_HCI_LE_AOD_CTE_1US ||
	    params->cte_type == BT_HCI_LE_AOD_CTE_2US) {

		if (!BT_FEAT_LE_ANT_SWITCH_TX_AOD(bt_dev.le.features)) {
			return -EINVAL;
		}

		if (params->cte_type == BT_HCI_LE_AOD_CTE_1US &&
		    !DF_AOD_TX_1US_SUPPORT(df_ant_info.switch_sample_rates)) {
			return -EINVAL;
		}

		if (params->num_ant_ids < BT_HCI_LE_SWITCH_PATTERN_LEN_MIN ||
		    params->num_ant_ids > BT_HCI_LE_SWITCH_PATTERN_LEN_MAX ||
		    !params->ant_ids) {
			return -EINVAL;
		}
	} else if (params->cte_type != BT_HCI_LE_AOA_CTE) {
		return -EINVAL;
	}

	if (params->cte_len < BT_HCI_LE_CTE_LEN_MIN ||
	    params->cte_len > BT_HCI_LE_CTE_LEN_MAX) {
		return -EINVAL;
	}

	if (params->cte_count < BT_HCI_LE_CTE_COUNT_MIN ||
	    params->cte_count > BT_HCI_LE_CTE_COUNT_MAX) {
		return -EINVAL;
	}

	buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_CL_CTE_TX_PARAMS,
				sizeof(*cp) + params->num_ant_ids);
	if (!buf) {
		return -ENOBUFS;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = adv->handle;
	cp->cte_len = params->cte_len;
	cp->cte_type = params->cte_type;
	cp->cte_count = params->cte_count;

	if (params->num_ant_ids) {
		uint8_t *dest_ant_ids = net_buf_add(buf, params->num_ant_ids);

		memcpy(dest_ant_ids, params->ant_ids, params->num_ant_ids);
		cp->switch_pattern_len = params->num_ant_ids;
	} else {
		cp->switch_pattern_len = 0;
	}

	return bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_CL_CTE_TX_PARAMS,
				    buf, NULL);
}

/* @brief Function provides information about DF antennae numer and
 *	  controller capabilities related with Constant Tone Extension.
 *
 * @param[out] switch_sample_rates      Optional switching and sampling rates.
 * @param[out] num_ant                  Antennae number.
 * @param[out] max_switch_pattern_len   Maximum supported antennae switching
 *                                      paterns length.
 * @param[out] max_cte_len              Maximum length of CTE in 8[us] units.
 *
 * @return Zero in case of success, other value in case of failure.
 */
static int hci_df_read_ant_info(uint8_t *switch_sample_rates,
				uint8_t *num_ant,
				uint8_t *max_switch_pattern_len,
				uint8_t *max_cte_len)
{
	__ASSERT_NO_MSG(switch_sample_rates);
	__ASSERT_NO_MSG(num_ant);
	__ASSERT_NO_MSG(max_switch_pattern_len);
	__ASSERT_NO_MSG(max_cte_len);

	struct bt_hci_rp_le_read_ant_info *rp;
	struct net_buf *rsp;
	int err;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_READ_ANT_INFO, NULL, &rsp);
	if (err) {
		BT_ERR("Failed to read antenna information");
		return err;
	}

	rp = (void *)rsp->data;

	BT_DBG("DF: sw. sampl rates: %x ant num: %u , max sw. pattern len: %u,"
	       "max CTE len %d", rp->switch_sample_rates, rp->num_ant,
	       rp->max_switch_pattern_len, rp->max_cte_len);

	*switch_sample_rates = rp->switch_sample_rates;
	*num_ant = rp->num_ant;
	*max_switch_pattern_len = rp->max_switch_pattern_len;
	*max_cte_len = rp->max_cte_len;

	net_buf_unref(rsp);

	return 0;
}

/* @brief Function handles send of HCI commnad to enable or disables CTE
 *        transmission for given advertising set.
 *
 * @param[in] adv               Pointer to advertising set
 * @param[in] enable            Enable or disable CTE TX
 *
 * @return Zero in case of success, other value in case of failure.
 */
static int hci_df_set_adv_cte_tx_enable(struct bt_le_ext_adv *adv,
					bool enable)
{
	struct bt_hci_cp_le_set_cl_cte_tx_enable *cp;
	struct bt_hci_cmd_state_set state;
	struct net_buf *buf;

	buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_CL_CTE_TX_ENABLE, sizeof(*cp));
	if (!buf) {
		return -ENOBUFS;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	(void)memset(cp, 0, sizeof(*cp));

	cp->handle = adv->handle;
	cp->cte_enable = enable ? 1 : 0;

	bt_hci_cmd_state_set_init(buf, &state, adv->flags, BT_PER_ADV_CTE_ENABLED,
				  enable);

	return bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_CL_CTE_TX_ENABLE,
				   buf, NULL);
}

#if defined(CONFIG_BT_DF_CONNECTIONLESS_CTE_RX) || defined(CONFIG_BT_DF_CONNECTION_CTE_RX)
static bool validate_cte_rx_common_params(uint8_t cte_type, uint8_t slot_durations,
					  uint8_t num_ant_ids, const uint8_t *ant_ids)
{
	if (!(cte_type & (BT_DF_CTE_TYPE_AOA | BT_DF_CTE_TYPE_AOD_1US | BT_DF_CTE_TYPE_AOD_2US))) {
		return false;
	}

	if (cte_type & BT_DF_CTE_TYPE_AOA) {
		if (df_ant_info.num_ant < DF_SAMPLING_ANTENNA_NUMBER_MIN ||
		    !BT_FEAT_LE_ANT_SWITCH_RX_AOA(bt_dev.le.features)) {
			return false;
		}

		if (!(slot_durations == BT_HCI_LE_ANTENNA_SWITCHING_SLOT_2US ||
		      (slot_durations == BT_HCI_LE_ANTENNA_SWITCHING_SLOT_1US &&
		       DF_AOA_RX_1US_SUPPORT(df_ant_info.switch_sample_rates)))) {
			return false;
		}

		if (num_ant_ids < BT_HCI_LE_SWITCH_PATTERN_LEN_MIN ||
		    num_ant_ids > df_ant_info.max_switch_pattern_len || !ant_ids) {
			return false;
		}
	}

	return true;
}
#endif /* CONFIG_BT_DF_CONNECTIONLESS_CTE_RX || CONFIG_BT_DF_CONNECTION_CTE_RX */

#if defined(CONFIG_BT_DF_CONNECTIONLESS_CTE_RX)
static bool validate_cl_cte_rx_params(const struct bt_df_per_adv_sync_cte_rx_param *params)
{
	if (params->max_cte_count > BT_HCI_LE_SAMPLE_CTE_COUNT_MAX) {
		return false;
	}

	if (params->cte_type & BT_DF_CTE_TYPE_AOA) {
		return validate_cte_rx_common_params(params->cte_type, params->slot_durations,
						     params->num_ant_ids, params->ant_ids);
	}

	return true;
}

static void
prepare_cl_cte_rx_enable_cmd_params(struct net_buf *buf, struct bt_le_per_adv_sync *sync,
				    const struct bt_df_per_adv_sync_cte_rx_param *params,
				    bool enable)
{
	struct bt_hci_cp_le_set_cl_cte_sampling_enable *cp;
	const uint8_t *ant_ids;

	cp = net_buf_add(buf, sizeof(*cp));
	(void)memset(cp, 0, sizeof(*cp));

	cp->sync_handle = sys_cpu_to_le16(sync->handle);
	cp->sampling_enable = enable ? 1 : 0;

	if (enable) {
		uint8_t *dest_ant_ids;

		cp->max_sampled_cte = params->max_cte_count;

		if (params->cte_type & BT_DF_CTE_TYPE_AOA) {
			cp->slot_durations = params->slot_durations;
			cp->switch_pattern_len = params->num_ant_ids;
			ant_ids = params->ant_ids;
		} else {
			/* Those values are put here due to constraints from HCI command
			 * specification: Bluetooth Core Spec. 5.3 Vol 4,Part E, sec 7.8.82.
			 * There is no right way to successfully send the command to enable CTE
			 * receive for AoD mode (e.g. device equipped with single antenna).
			 */
			cp->slot_durations = BT_HCI_LE_ANTENNA_SWITCHING_SLOT_2US;
			cp->switch_pattern_len = ARRAY_SIZE(df_dummy_switch_pattern);
			ant_ids = &df_dummy_switch_pattern[0];
		}

		dest_ant_ids =	net_buf_add(buf, params->num_ant_ids);
		memcpy(dest_ant_ids, ant_ids, cp->switch_pattern_len);
	}
}

static int hci_df_set_cl_cte_rx_enable(struct bt_le_per_adv_sync *sync, bool enable,
				       const struct bt_df_per_adv_sync_cte_rx_param *params)
{
	struct bt_hci_rp_le_set_cl_cte_sampling_enable *rp;
	struct bt_hci_cmd_state_set state;
	struct net_buf *buf, *rsp;
	int err;

	if (enable) {
		if (!validate_cl_cte_rx_params(params)) {
			return -EINVAL;
		}
	}

	/* If CTE Rx is enabled, command parameters total length must include
	 * antenna ids, so command size if extended by num_and_ids.
	 */
	buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_CL_CTE_SAMPLING_ENABLE,
				(sizeof(struct bt_hci_cp_le_set_cl_cte_sampling_enable) +
				 (enable ? params->num_ant_ids : 0)));
	if (!buf) {
		return -ENOBUFS;
	}

	prepare_cl_cte_rx_enable_cmd_params(buf, sync, params, enable);

	bt_hci_cmd_state_set_init(buf, &state, sync->flags, BT_PER_ADV_SYNC_CTE_ENABLED, enable);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_CL_CTE_SAMPLING_ENABLE, buf, &rsp);
	if (err) {
		return err;
	}

	rp = (void *)rsp->data;
	if (sync->handle != sys_le16_to_cpu(rp->sync_handle)) {
		err = -EIO;
	} else {
		sync->cte_type = (enable ? params->cte_type : 0);
	}

	net_buf_unref(rsp);

	return err;
}

void hci_df_prepare_connectionless_iq_report(struct net_buf *buf,
					     struct bt_df_per_adv_sync_iq_samples_report *report,
					     struct bt_le_per_adv_sync **per_adv_sync_to_report)
{
	struct bt_hci_evt_le_connectionless_iq_report *evt;
	struct bt_le_per_adv_sync *per_adv_sync;

	if (buf->len < sizeof(*evt)) {
		BT_ERR("Unexpected end of buffer");
		return;
	}

	evt = net_buf_pull_mem(buf, sizeof(*evt));

	per_adv_sync = bt_hci_get_per_adv_sync(sys_le16_to_cpu(evt->sync_handle));

	if (!per_adv_sync) {
		BT_ERR("Unknown handle 0x%04X for iq samples report",
		       sys_le16_to_cpu(evt->sync_handle));
		return;
	}

	if (!atomic_test_bit(per_adv_sync->flags, BT_PER_ADV_SYNC_CTE_ENABLED)) {
		BT_ERR("Received PA CTE report when CTE receive disabled");
		return;
	}

	if (!(per_adv_sync->cte_type & BIT(evt->cte_type))) {
		BT_DBG("CTE filtered out by cte_type: %u", evt->cte_type);
		return;
	}

	report->chan_idx = evt->chan_idx;
	report->rssi = sys_le16_to_cpu(evt->rssi);
	report->rssi_ant_id = evt->rssi_ant_id;
	report->cte_type = BIT(evt->cte_type);
	report->packet_status = evt->packet_status;
	report->slot_durations = evt->slot_durations;
	report->per_evt_counter = sys_le16_to_cpu(evt->per_evt_counter);
	report->sample_count = evt->sample_count;
	report->sample = &evt->sample[0];

	*per_adv_sync_to_report = per_adv_sync;
}
#endif /* CONFIG_BT_DF_CONNECTIONLESS_CTE_RX */

#if defined(CONFIG_BT_CTLR_DF_CONN_CTE_RSP)
/* @brief Function sets CTE parameters for connection object
 *
 * @param[in] cte_types         Allowed response CTE types
 * @param[in] num_ant_id        Number of available antenna identification
 *                              patterns in @p ant_id array.
 * @param[in] ant_id            Array with antenna identification patterns.
 *
 * @return Zero in case of success, other value in case of failure.
 */
static int hci_df_set_conn_cte_tx_param(struct bt_conn *conn, uint8_t cte_types,
					uint8_t num_ant_id, uint8_t *ant_id)
{
	__ASSERT_NO_MSG(conn);
	__ASSERT_NO_MSG(cte_types != 0);

	struct bt_hci_cp_le_set_conn_cte_tx_params *cp;
	struct bt_hci_rp_le_set_conn_cte_tx_params *rp;
	struct net_buf *buf, *rsp;
	int err;

	/* If AoD is not enabled, ant_ids are ignored by controller:
	 * BT Core spec 5.2 Vol 4, Part E sec. 7.8.84.
	 */
	if (cte_types & BT_HCI_LE_AOD_CTE_RSP_1US ||
	    cte_types & BT_HCI_LE_AOD_CTE_RSP_2US) {

		if (num_ant_id < BT_HCI_LE_SWITCH_PATTERN_LEN_MIN ||
		    num_ant_id > BT_HCI_LE_SWITCH_PATTERN_LEN_MAX ||
		    !ant_id) {
			return -EINVAL;
		}
		__ASSERT_NO_MSG((sizeof(*cp) + num_ant_id) <  UINT8_MAX);
	}

	buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_CONN_CTE_TX_PARAMS,
				sizeof(*cp) + num_ant_id);
	if (!buf) {
		return -ENOBUFS;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(conn->handle);
	cp->cte_types = cte_types;

	if (num_ant_id) {
		uint8_t *dest_ant_id = net_buf_add(buf, num_ant_id);

		memcpy(dest_ant_id, ant_id, num_ant_id);
		cp->switch_pattern_len = num_ant_id;
	} else {
		cp->switch_pattern_len = 0;
	}

	err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_CONN_CTE_TX_PARAMS,
				   buf, &rsp);
	if (err) {
		return err;
	}

	rp = (void *)rsp->data;
	if (conn->handle != sys_le16_to_cpu(rp->handle)) {
		err = -EIO;
	}

	net_buf_unref(rsp);

	return err;
}
#endif /* CONFIG_BT_CTLR_DF_CONN_CTE_RSP */

#if defined(CONFIG_BT_DF_CONNECTION_CTE_RX)
static void prepare_conn_cte_rx_enable_cmd_params(struct net_buf *buf, struct bt_conn *conn,
						  const struct bt_df_conn_cte_rx_param *params,
						  bool enable)
{
	struct bt_hci_cp_le_set_conn_cte_rx_params *cp;
	const uint8_t *ant_ids;

	cp = net_buf_add(buf, sizeof(*cp));
	(void)memset(cp, 0, sizeof(*cp));

	cp->handle = sys_cpu_to_le16(conn->handle);
	cp->sampling_enable = enable ? 1 : 0;

	if (enable) {
		uint8_t *dest_ant_ids;

		if (params->cte_type & BT_DF_CTE_TYPE_AOA) {
			cp->slot_durations = params->slot_durations;
			cp->switch_pattern_len = params->num_ant_ids;
			ant_ids = params->ant_ids;
		} else {
			/* Those values are put here due to constraints from HCI command
			 * specification: Bluetooth Core Spec. 5.3 Vol 4,Part E, sec 7.8.85.
			 * There is no right way to successfully send the command to enable CTE
			 * receive for AoD mode (e.g. device equipped with single antenna).
			 */
			cp->slot_durations = BT_HCI_LE_ANTENNA_SWITCHING_SLOT_2US;
			cp->switch_pattern_len = ARRAY_SIZE(df_dummy_switch_pattern);
			ant_ids = &df_dummy_switch_pattern[0];
		}

		dest_ant_ids = net_buf_add(buf, params->num_ant_ids);
		(void)memcpy(dest_ant_ids, ant_ids, cp->switch_pattern_len);
	}
}

static int hci_df_set_conn_cte_rx_enable(struct bt_conn *conn, bool enable,
					 const struct bt_df_conn_cte_rx_param *params)
{
	struct bt_hci_rp_le_set_conn_cte_rx_params *rp;
	struct bt_hci_cmd_state_set state;
	struct net_buf *buf, *rsp;
	int err;

	if (enable) {
		if (!validate_cte_rx_common_params(params->cte_type, params->slot_durations,
						   params->num_ant_ids, params->ant_ids)) {
			return -EINVAL;
		}
	}

	/* If CTE Rx is enabled, command parameters total length must include
	 * antenna ids, so command size if extended by num_and_ids.
	 */
	buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_CONN_CTE_RX_PARAMS,
				(sizeof(struct bt_hci_rp_le_set_conn_cte_rx_params) +
				 (enable ? params->num_ant_ids : 0)));
	if (!buf) {
		return -ENOBUFS;
	}

	prepare_conn_cte_rx_enable_cmd_params(buf, conn, params, enable);

	bt_hci_cmd_state_set_init(buf, &state, conn->flags, BT_CONN_CTE_RX_ENABLED, enable);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_CONN_CTE_RX_PARAMS, buf, &rsp);
	if (err) {
		return err;
	}

	rp = (void *)rsp->data;
	if (conn->handle != sys_le16_to_cpu(rp->handle)) {
		err = -EIO;
	} else {
		conn->cte_type = (enable ? params->cte_type : 0);
	}

	net_buf_unref(rsp);

	return err;
}

int hci_df_prepare_connection_iq_report(struct net_buf *buf,
					 struct bt_df_conn_iq_samples_report *report,
					 struct bt_conn **conn_to_report)
{
	struct bt_hci_evt_le_connection_iq_report *evt;
	struct bt_conn *conn;

	if (buf->len < sizeof(*evt)) {
		BT_ERR("Unexpected end of buffer");
		return -EINVAL;
	}

	evt = net_buf_pull_mem(buf, sizeof(*evt));

	conn = bt_conn_lookup_handle(sys_le16_to_cpu(evt->conn_handle));

	if (!conn) {
		BT_ERR("Unknown conn handle 0x%04X for iq samples report",
		       sys_le16_to_cpu(evt->conn_handle));
		return -EINVAL;
	}

	if (!atomic_test_bit(conn->flags, BT_CONN_CTE_RX_ENABLED)) {
		BT_ERR("Received conn CTE report when CTE receive disabled");
		return -EINVAL;
	}

	if (!(conn->cte_type & BIT(evt->cte_type))) {
		BT_DBG("CTE filtered out by cte_type: %u", evt->cte_type);
		return -EINVAL;
	}

	report->chan_idx = evt->data_chan_idx;
	report->rx_phy = evt->rx_phy;
	report->chan_idx = evt->data_chan_idx;
	report->rssi = evt->rssi;
	report->rssi_ant_id = evt->rssi_ant_id;
	report->cte_type = BIT(evt->cte_type);
	report->packet_status = evt->packet_status;
	report->slot_durations = evt->slot_durations;
	report->conn_evt_counter = sys_le16_to_cpu(evt->conn_evt_counter);
	report->sample_count = evt->sample_count;
	report->sample = evt->sample;

	*conn_to_report = conn;

	return 0;
}
#endif /* CONFIG_BT_DF_CONNECTION_CTE_RX */

#if defined(CONFIG_BT_DF_CONNECTION_CTE_RSP)
static void prepare_conn_cte_rsp_enable_cmd_params(struct net_buf *buf, const struct bt_conn *conn,
						   bool enable)
{
	struct bt_hci_cp_le_conn_cte_rsp_enable *cp;

	cp = net_buf_add(buf, sizeof(*cp));
	(void)memset(cp, 0, sizeof(*cp));

	cp->handle = sys_cpu_to_le16(conn->handle);
	cp->enable = enable ? 1U : 0U;
}

static int hci_df_set_conn_cte_rsp_enable(struct bt_conn *conn, bool enable)
{
	struct bt_hci_rp_le_conn_cte_rsp_enable *rp;
	struct bt_hci_cmd_state_set state;
	struct net_buf *buf, *rsp;
	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_LE_CONN_CTE_RSP_ENABLE,
				sizeof(struct bt_hci_cp_le_conn_cte_rsp_enable));
	if (!buf) {
		return -ENOBUFS;
	}

	prepare_conn_cte_rsp_enable_cmd_params(buf, conn, enable);

	bt_hci_cmd_state_set_init(buf, &state, conn->flags, BT_CONN_CTE_RSP_ENABLED, enable);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_CONN_CTE_RSP_ENABLE, buf, &rsp);
	if (err) {
		return err;
	}

	rp = (void *)rsp->data;
	if (conn->handle != sys_le16_to_cpu(rp->handle)) {
		err = -EIO;
	}

	net_buf_unref(rsp);

	return err;
}
#endif /* CONFIG_BT_DF_CONNECTION_CTE_RSP */

/* @brief Function initializes Direction Finding in Host
 *
 * @return Zero in case of success, other value in case of failure.
 */
int le_df_init(void)
{
	uint8_t max_switch_pattern_len;
	uint8_t switch_sample_rates;
	uint8_t max_cte_len;
	uint8_t num_ant;
	int err;

	err = hci_df_read_ant_info(&switch_sample_rates, &num_ant,
			     &max_switch_pattern_len, &max_cte_len);
	if (err) {
		return err;
	}

	df_ant_info.max_switch_pattern_len = max_switch_pattern_len;
	df_ant_info.switch_sample_rates = switch_sample_rates;
	df_ant_info.max_cte_len = max_cte_len;
	df_ant_info.num_ant = num_ant;

	BT_DBG("DF initialized.");
	return 0;
}

int bt_df_set_adv_cte_tx_param(struct bt_le_ext_adv *adv,
				const struct bt_df_adv_cte_tx_param *params)
{
	__ASSERT_NO_MSG(adv);
	__ASSERT_NO_MSG(params);

	int err;

	if (!BT_FEAT_LE_CONNECTIONLESS_CTE_TX(bt_dev.le.features)) {
		return -ENOTSUP;
	}

	/* Check if BT_ADV_PARAMS_SET is set, because it implies the set
	 * has already been created.
	 */
	if (!atomic_test_bit(adv->flags, BT_ADV_PARAMS_SET)) {
		return -EINVAL;
	}

	if (atomic_test_bit(adv->flags, BT_PER_ADV_CTE_ENABLED)) {
		return -EINVAL;
	}

	err = hci_df_set_cl_cte_tx_params(adv, params);
	if (err) {
		return err;
	}

	atomic_set_bit(adv->flags, BT_PER_ADV_CTE_PARAMS_SET);

	return 0;
}

static int bt_df_set_adv_cte_tx_enabled(struct bt_le_ext_adv *adv, bool enable)
{
	if (!atomic_test_bit(adv->flags, BT_PER_ADV_PARAMS_SET)) {
		return -EINVAL;
	}

	if (!atomic_test_bit(adv->flags, BT_PER_ADV_CTE_PARAMS_SET)) {
		return -EINVAL;
	}

	if (enable == atomic_test_bit(adv->flags, BT_PER_ADV_CTE_ENABLED)) {
		return -EALREADY;
	}

	return hci_df_set_adv_cte_tx_enable(adv, enable);
}

int bt_df_adv_cte_tx_enable(struct bt_le_ext_adv *adv)
{
	__ASSERT_NO_MSG(adv);
	return bt_df_set_adv_cte_tx_enabled(adv, true);
}

int bt_df_adv_cte_tx_disable(struct bt_le_ext_adv *adv)
{
	__ASSERT_NO_MSG(adv);
	return bt_df_set_adv_cte_tx_enabled(adv, false);
}

#if defined(CONFIG_BT_DF_CONNECTIONLESS_CTE_RX)
static int
bt_df_set_per_adv_sync_cte_rx_enable(struct bt_le_per_adv_sync *sync, bool enable,
				     const struct bt_df_per_adv_sync_cte_rx_param *params)
{
	if (!BT_FEAT_LE_CONNECTIONLESS_CTE_RX(bt_dev.le.features)) {
		return -ENOTSUP;
	}

	if (!atomic_test_bit(sync->flags, BT_PER_ADV_SYNC_SYNCED)) {
		return -EINVAL;
	}

	if (!enable &&
	    !atomic_test_bit(sync->flags, BT_PER_ADV_SYNC_CTE_ENABLED)) {
		return -EALREADY;
	}

	return hci_df_set_cl_cte_rx_enable(sync, enable, params);
}

int bt_df_per_adv_sync_cte_rx_enable(struct bt_le_per_adv_sync *sync,
				     const struct bt_df_per_adv_sync_cte_rx_param *params)
{
	CHECKIF(!sync) {
		return -EINVAL;
	}
	CHECKIF(!params) {
		return -EINVAL;
	}

	return bt_df_set_per_adv_sync_cte_rx_enable(sync, true, params);
}

int bt_df_per_adv_sync_cte_rx_disable(struct bt_le_per_adv_sync *sync)
{
	CHECKIF(!sync) {
		return -EINVAL;
	}

	return bt_df_set_per_adv_sync_cte_rx_enable(sync, false, NULL);
}
#endif /* CONFIG_BT_DF_CONNECTIONLESS_CTE_RX */

#if defined(CONFIG_BT_DF_CONNECTION_CTE_RX)
static int bt_df_set_conn_cte_rx_enable(struct bt_conn *conn, bool enable,
					const struct bt_df_conn_cte_rx_param *params)
{
	if (!BT_FEAT_LE_RX_CTE(bt_dev.le.features)) {
		BT_WARN("Receiving Constant Tone Extensions is not supported");
		return -ENOTSUP;
	}

	if (conn->state != BT_CONN_CONNECTED) {
		BT_ERR("not connected!");
		return -ENOTCONN;
	}

	return hci_df_set_conn_cte_rx_enable(conn, enable, params);
}

int bt_df_conn_cte_rx_enable(struct bt_conn *conn, const struct bt_df_conn_cte_rx_param *params)
{
	CHECKIF(!conn)
	{
		return -EINVAL;
	}
	CHECKIF(!params)
	{
		return -EINVAL;
	}

	return bt_df_set_conn_cte_rx_enable(conn, true, params);
}

int bt_df_conn_cte_rx_disable(struct bt_conn *conn)
{
	CHECKIF(!conn)
	{
		return -EINVAL;
	}

	return bt_df_set_conn_cte_rx_enable(conn, false, NULL);
}
#endif /* CONFIG_BT_DF_CONNECTION_CTE_RX */

#if defined(CONFIG_BT_DF_CONNECTION_CTE_RSP)
static int bt_df_set_conn_cte_rsp_enable(struct bt_conn *conn, bool enable)
{
	CHECKIF(!conn) {
		return -EINVAL;
	}

	if (!BT_FEAT_LE_CONNECTION_CTE_RESP(bt_dev.le.features)) {
		BT_WARN("CTE response procedure is not supported");
		return -ENOTSUP;
	}

	if (conn->state != BT_CONN_CONNECTED) {
		BT_ERR("not connected");
		return -ENOTCONN;
	}

	if (!atomic_test_bit(conn->flags, BT_CONN_CTE_TX_PARAMS_SET)) {
		BT_ERR("Can't start CTE response procedure before CTE TX params setup");
		return -EINVAL;
	}

	return hci_df_set_conn_cte_rsp_enable(conn, enable);
}

int bt_df_conn_cte_rsp_enable(struct bt_conn *conn)
{
	return bt_df_set_conn_cte_rsp_enable(conn, true);
}

int bt_df_conn_cte_rsp_disable(struct bt_conn *conn)
{
	return bt_df_set_conn_cte_rsp_enable(conn, false);
}
#endif /* CONFIG_BT_DF_CONNECTION_CTE_RSP */
