/*
 * Copyright (c) 2016-2021 Nordic Semiconductor ASA
 * Copyright (c) 2016 Vinayak Kariappa Chettimada
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LL_VERSION_NUMBER BT_HCI_VERSION_5_3

#if defined(CONFIG_BT_CTLR_LE_ENC)
#define LL_FEAT_BIT_ENC BIT64(BT_LE_FEAT_BIT_ENC)
#else /* !CONFIG_BT_CTLR_LE_ENC */
#define LL_FEAT_BIT_ENC 0
#endif /* !CONFIG_BT_CTLR_LE_ENC */

#if defined(CONFIG_BT_CTLR_CONN_PARAM_REQ)
#define LL_FEAT_BIT_CONN_PARAM_REQ BIT64(BT_LE_FEAT_BIT_CONN_PARAM_REQ)
#else /* !CONFIG_BT_CTLR_CONN_PARAM_REQ */
#define LL_FEAT_BIT_CONN_PARAM_REQ 0
#endif /* !CONFIG_BT_CTLR_CONN_PARAM_REQ */

#if defined(CONFIG_BT_CTLR_EXT_REJ_IND)
#define LL_FEAT_BIT_EXT_REJ_IND BIT64(BT_LE_FEAT_BIT_EXT_REJ_IND)
#else /* !CONFIG_BT_CTLR_EXT_REJ_IND */
#define LL_FEAT_BIT_EXT_REJ_IND 0
#endif /* !CONFIG_BT_CTLR_EXT_REJ_IND */

#if defined(CONFIG_BT_CTLR_PER_INIT_FEAT_XCHG)
#define LL_FEAT_BIT_PER_INIT_FEAT_XCHG BIT64(BT_LE_FEAT_BIT_PER_INIT_FEAT_XCHG)
#else /* !CONFIG_BT_CTLR_PER_INIT_FEAT_XCHG */
#define LL_FEAT_BIT_PER_INIT_FEAT_XCHG 0
#endif /* !CONFIG_BT_CTLR_PER_INIT_FEAT_XCHG */

#if defined(CONFIG_BT_CTLR_LE_PING)
#define LL_FEAT_BIT_PING BIT64(BT_LE_FEAT_BIT_PING)
#else /* !CONFIG_BT_CTLR_LE_PING */
#define LL_FEAT_BIT_PING 0
#endif /* !CONFIG_BT_CTLR_LE_PING */

/* Maximum supported ACL Tx fragment size is limited by uint8_t len field in
 * the PDU structure of the Controller implementation. 4 octets reserved for
 * MIC in encrypted ACL PDUs, hence ACL Tx fragment maximum size of 251 octets.
 */
#define LL_LENGTH_OCTETS_TX_MAX MIN(CONFIG_BT_BUF_ACL_TX_SIZE, 251U)

#if defined(CONFIG_BT_CTLR_DATA_LENGTH_MAX)
#define LL_FEAT_BIT_DLE BIT64(BT_LE_FEAT_BIT_DLE)
#define LL_LENGTH_OCTETS_RX_MAX CONFIG_BT_CTLR_DATA_LENGTH_MAX
#else
#define LL_FEAT_BIT_DLE 0
#define LL_LENGTH_OCTETS_RX_MAX 27
#endif /* CONFIG_BT_CTLR_DATA_LENGTH_MAX */

#if defined(CONFIG_BT_CTLR_PRIVACY)
#define LL_FEAT_BIT_PRIVACY BIT64(BT_LE_FEAT_BIT_PRIVACY)
#else /* !CONFIG_BT_CTLR_PRIVACY */
#define LL_FEAT_BIT_PRIVACY 0
#endif /* !CONFIG_BT_CTLR_PRIVACY */

#if defined(CONFIG_BT_CTLR_EXT_SCAN_FP)
#define LL_FEAT_BIT_EXT_SCAN BIT64(BT_LE_FEAT_BIT_EXT_SCAN)
#else /* !CONFIG_BT_CTLR_EXT_SCAN_FP */
#define LL_FEAT_BIT_EXT_SCAN 0
#endif /* !CONFIG_BT_CTLR_EXT_SCAN_FP */

#if defined(CONFIG_BT_CTLR_PHY_2M)
#define LL_FEAT_BIT_PHY_2M BIT64(BT_LE_FEAT_BIT_PHY_2M)
#else /* !CONFIG_BT_CTLR_PHY_2M */
#define LL_FEAT_BIT_PHY_2M 0
#endif /* !CONFIG_BT_CTLR_PHY_2M */

#if defined(CONFIG_BT_CTLR_SMI_TX)
#if defined(CONFIG_BT_CTLR_SMI_TX_SETTING)
#define LL_FEAT_BIT_SMI_TX (ll_settings_smi_tx() ? \
			    BIT64(BT_LE_FEAT_BIT_SMI_TX) : 0)
#else /* !CONFIG_BT_CTLR_SMI_TX_SETTING */
#define LL_FEAT_BIT_SMI_TX BIT64(BT_LE_FEAT_BIT_SMI_TX)
#endif /* !CONFIG_BT_CTLR_SMI_TX_SETTING */
#else /* !CONFIG_BT_CTLR_SMI_TX */
#define LL_FEAT_BIT_SMI_TX 0
#endif /* !CONFIG_BT_CTLR_SMI_TX */

#if defined(CONFIG_BT_CTLR_SMI_RX)
#define LL_FEAT_BIT_SMI_RX BIT64(BT_LE_FEAT_BIT_SMI_RX)
#else /* !CONFIG_BT_CTLR_SMI_RX */
#define LL_FEAT_BIT_SMI_RX 0
#endif /* !CONFIG_BT_CTLR_SMI_RX */

#if defined(CONFIG_BT_CTLR_PHY_CODED)
#define LL_FEAT_BIT_PHY_CODED BIT64(BT_LE_FEAT_BIT_PHY_CODED)
#else /* !CONFIG_BT_CTLR_PHY_CODED */
#define LL_FEAT_BIT_PHY_CODED 0
#endif /* !CONFIG_BT_CTLR_PHY_CODED */

#if defined(CONFIG_BT_CTLR_ADV_EXT)
#define LL_FEAT_BIT_EXT_ADV BIT64(BT_LE_FEAT_BIT_EXT_ADV)
#if defined(CONFIG_BT_OBSERVER)
#define LL_EXT_OCTETS_RX_MAX CONFIG_BT_CTLR_ADV_EXT_RX_PDU_LEN_MAX
#else /* !CONFIG_BT_OBSERVER */
#define LL_EXT_OCTETS_RX_MAX 0
#endif /* !CONFIG_BT_OBSERVER */
#else /* !CONFIG_BT_CTLR_ADV_EXT */
#define LL_FEAT_BIT_EXT_ADV 0
#define LL_EXT_OCTETS_RX_MAX 0
#endif /* !CONFIG_BT_CTLR_ADV_EXT */

#if defined(CONFIG_BT_CTLR_ADV_PERIODIC) || \
	defined(CONFIG_BT_CTLR_SYNC_PERIODIC)
#define LL_FEAT_BIT_PER_ADV BIT64(BT_LE_FEAT_BIT_PER_ADV)
#else /* !CONFIG_BT_CTLR_ADV_PERIODIC && !CONFIG_BT_CTLR_SYNC_PERIODIC */
#define LL_FEAT_BIT_PER_ADV 0
#endif /* !CONFIG_BT_CTLR_ADV_PERIODIC && !CONFIG_BT_CTLR_SYNC_PERIODIC */

#if defined(CONFIG_BT_CTLR_CHAN_SEL_2)
#define LL_FEAT_BIT_CHAN_SEL_2 BIT64(BT_LE_FEAT_BIT_CHAN_SEL_ALGO_2)
#else /* !CONFIG_BT_CTLR_CHAN_SEL_2 */
#define LL_FEAT_BIT_CHAN_SEL_2 0
#endif /* !CONFIG_BT_CTLR_CHAN_SEL_2 */

#if defined(CONFIG_BT_CTLR_MIN_USED_CHAN)
#define LL_FEAT_BIT_MIN_USED_CHAN \
		BIT64(BT_LE_FEAT_BIT_MIN_USED_CHAN_PROC)
#else /* !CONFIG_BT_CTLR_MIN_USED_CHAN */
#define LL_FEAT_BIT_MIN_USED_CHAN 0
#endif /* !CONFIG_BT_CTLR_MIN_USED_CHAN */

#if defined(CONFIG_BT_CTLR_DF) && defined(CONFIG_BT_CTLR_DF_CONN_CTE_REQ)
#define LL_FEAT_BIT_CONNECTION_CTE_REQ BIT64(BT_LE_FEAT_BIT_CONN_CTE_REQ)
#else /* !CONFIG_BT_CTLR_DF && !CONFIG_BT_CTLR_DF_CONN_CTE_REQ */
#define LL_FEAT_BIT_CONNECTION_CTE_REQ 0
#endif /* !CONFIG_BT_CTLR_DF && !CONFIG_BT_CTLR_DF_CONN_CTE_REQ */

#if defined(CONFIG_BT_CTLR_DF) && defined(CONFIG_BT_CTLR_DF_CONN_CTE_RSP)
#define LL_FEAT_BIT_CONNECTION_CTE_RESP BIT64(BT_LE_FEAT_BIT_CONN_CTE_RESP)
#else /* !CONFIG_BT_CTLR_DF && !CONFIG_BT_CTLR_DF_CONN_CTE_RSP */
#define LL_FEAT_BIT_CONNECTION_CTE_RESP 0
#endif /* !CONFIG_BT_CTLR_DF && !CONFIG_BT_CTLR_DF_CONN_CTE_RSP */


#if defined(CONFIG_BT_CTLR_DF) && defined(CONFIG_BT_CTLR_DF_ADV_CTE_TX)
#define LL_FEAT_BIT_CONNECTIONLESS_CTE_TX \
	BIT64(BT_LE_FEAT_BIT_CONNECTIONLESS_CTE_TX)
#else /* !CONFIG_BT_CTLR_DF && !CONFIG_BT_CTLR_DF_ADV_CTE_TX */
#define LL_FEAT_BIT_CONNECTIONLESS_CTE_TX 0
#endif /* !CONFIG_BT_CTLR_DF && !CONFIG_BT_CTLR_DF_ADV_CTE_TX */

#if defined(CONFIG_BT_CTLR_DF) && defined(CONFIG_BT_CTLR_DF_SCAN_CTE_RX)
#define LL_FEAT_BIT_CONNECTIONLESS_CTE_RX \
	BIT64(BT_LE_FEAT_BIT_CONNECTIONLESS_CTE_RX)
#else /* !CONFIG_BT_CTLR_DF && !CONFIG_BT_CTLR_DF_SCAN_CTE_RX */
#define LL_FEAT_BIT_CONNECTIONLESS_CTE_RX 0
#endif /* !CONFIG_BT_CTLR_DF && !CONFIG_BT_CTLR_DF_SCAN_CTE_RX */

#if defined(CONFIG_BT_CTLR_DF) && defined(CONFIG_BT_CTLR_DF_ANT_SWITCH_TX)
#define LL_FEAT_BIT_ANT_SWITCH_TX_AOD \
	BIT64(BT_LE_FEAT_BIT_ANT_SWITCH_TX_AOD)
#else /* !CONFIG_BT_CTLR_DF && !CONFIG_BT_CTLR_DF_ANT_SWITCH_TX */
#define LL_FEAT_BIT_ANT_SWITCH_TX_AOD 0
#endif /* !CONFIG_BT_CTLR_DF && !CONFIG_BT_CTLR_DF_ANT_SWITCH_TX */

#if defined(CONFIG_BT_CTLR_DF) && defined(CONFIG_BT_CTLR_DF_ANT_SWITCH_RX)
#define LL_FEAT_BIT_ANT_SWITCH_RX_AOA \
	BIT64(BT_LE_FEAT_BIT_ANT_SWITCH_RX_AOA)
#else /* !CONFIG_BT_CTLR_DF && !CONFIG_BT_CTLR_DF_ANT_SWITCH_RX */
#define LL_FEAT_BIT_ANT_SWITCH_RX_AOA 0
#endif /* !CONFIG_BT_CTLR_DF && !CONFIG_BT_CTLR_DF_ANT_SWITCH_RX */

#if defined(CONFIG_BT_CTLR_DF) && defined(CONFIG_BT_CTLR_DF_CTE_RX)
#define LL_FEAT_BIT_RX_CTE BIT64(BT_LE_FEAT_BIT_RX_CTE)
#else /* !CONFIG_BT_CTLR_DF && !CONFIG_BT_CTLR_DF_CTE_RX */
#define LL_FEAT_BIT_RX_CTE 0
#endif /* !CONFIG_BT_CTLR_DF && !CONFIG_BT_CTLR_DF_CTE_RX */

#if defined(CONFIG_BT_CTLR_SCA_UPDATE)
#define LL_FEAT_BIT_SCA_UPDATE BIT64(BT_LE_FEAT_BIT_SCA_UPDATE)
#else /* !CONFIG_BT_CTLR_SCA_UPDATE */
#define LL_FEAT_BIT_SCA_UPDATE 0
#endif /* !CONFIG_BT_CTLR_SCA_UPDATE */

#if defined(CONFIG_BT_CTLR_CENTRAL_ISO)
#define LL_FEAT_BIT_CIS_CENTRAL BIT64(BT_LE_FEAT_BIT_CIS_CENTRAL)
#else /* !CONFIG_BT_CTLR_CENTRAL_ISO */
#define LL_FEAT_BIT_CIS_CENTRAL 0
#endif /* !CONFIG_BT_CTLR_CENTRAL_ISO */

#if defined(CONFIG_BT_CTLR_PERIPHERAL_ISO)
#define LL_FEAT_BIT_CIS_PERIPHERAL BIT64(BT_LE_FEAT_BIT_CIS_PERIPHERAL)
#else /* !CONFIG_BT_CTLR_PERIPHERAL_ISO */
#define LL_FEAT_BIT_CIS_PERIPHERAL 0
#endif /* !CONFIG_BT_CTLR_PERIPHERAL_ISO */

#if defined(CONFIG_BT_CTLR_CENTRAL_ISO) || \
	defined(CONFIG_BT_CTLR_PERIPHERAL_ISO)
#define LL_CIS_OCTETS_TX_MAX MIN(CONFIG_BT_CTLR_CONN_ISO_PDU_LEN_MAX, \
				 CONFIG_BT_CTLR_ISO_TX_BUFFER_SIZE)
#define LL_CIS_OCTETS_RX_MAX CONFIG_BT_CTLR_CONN_ISO_PDU_LEN_MAX
#else /* !CONFIG_BT_CTLR_CENTRAL_ISO && !CONFIG_BT_CTLR_PERIPHERAL_ISO */
#define LL_CIS_OCTETS_TX_MAX 0
#define LL_CIS_OCTETS_RX_MAX 0
#endif /* !CONFIG_BT_CTLR_CENTRAL_ISO && !CONFIG_BT_CTLR_PERIPHERAL_ISO */

#if defined(CONFIG_BT_CTLR_ADV_ISO)
#define LL_FEAT_BIT_ISO_BROADCASTER BIT64(BT_LE_FEAT_BIT_ISO_BROADCASTER)
#if defined(CONFIG_BT_ISO_TX_MTU)
#define LL_BIS_OCTETS_TX_MAX MIN(CONFIG_BT_CTLR_ADV_ISO_PDU_LEN_MAX, \
				 CONFIG_BT_ISO_TX_MTU)
#else /* !CONFIG_BT_ISO_TX_MTU */
#define LL_BIS_OCTETS_TX_MAX CONFIG_BT_CTLR_ADV_ISO_PDU_LEN_MAX
#endif /* !CONFIG_BT_ISO_TX_MTU */
#else /* !CONFIG_BT_CTLR_ADV_ISO */
#define LL_FEAT_BIT_ISO_BROADCASTER 0
#define LL_BIS_OCTETS_TX_MAX 0
#endif /* !CONFIG_BT_CTLR_ADV_ISO */

#if defined(CONFIG_BT_CTLR_SYNC_ISO)
#define LL_FEAT_BIT_SYNC_RECEIVER BIT64(BT_LE_FEAT_BIT_SYNC_RECEIVER)
#define LL_BIS_OCTETS_RX_MAX CONFIG_BT_CTLR_SYNC_ISO_PDU_LEN_MAX
#else /* !CONFIG_BT_CTLR_SYNC_ISO */
#define LL_FEAT_BIT_SYNC_RECEIVER 0
#define LL_BIS_OCTETS_RX_MAX 0
#endif /* !CONFIG_BT_CTLR_SYNC_ISO */

#if defined(CONFIG_BT_CTLR_ADV_PERIODIC_ADI_SUPPORT) || \
	defined(CONFIG_BT_CTLR_SYNC_PERIODIC_ADI_SUPPORT)
#define LL_FEAT_BIT_PERIODIC_ADI_SUPPORT BIT64(BT_LE_FEAT_BIT_PER_ADV_ADI_SUPP)
#else  /* !CONFIG_BT_CTLR_ADV_PERIODIC_ADI_SUPPORT &&
	* !CONFIG_BT_CTLR_SYNC_PERIODIC_ADI_SUPPORT
	*/
#define LL_FEAT_BIT_PERIODIC_ADI_SUPPORT 0U
#endif /* !CONFIG_BT_CTLR_ADV_PERIODIC_ADI_SUPPORT &&
	* !CONFIG_BT_CTLR_SYNC_PERIODIC_ADI_SUPPORT
	*/

/* All defined feature bits */
#define LL_FEAT_BIT_MASK         0xFFFFFFFFFFULL

/*
 * LL_FEAT_BIT_MASK_VALID is defined as per
 * Core Spec V5.3 Volume 6, Part B, chapter 4.6
 */
#define LL_FEAT_BIT_MASK_VALID   0xEFF787CF2FULL

/* Mask to filter away octet 0 for feature exchange */
#define LL_FEAT_FILTER_OCTET0    (LL_FEAT_BIT_MASK & ~0xFFULL)

/* Mask for host controlled features */
#define LL_FEAT_HOST_BIT_MASK    0x4100000000ULL

/* Feature bits of this controller */
#define LL_FEAT                  (LL_FEAT_BIT_ENC | \
				  LL_FEAT_BIT_CONN_PARAM_REQ | \
				  LL_FEAT_BIT_EXT_REJ_IND | \
				  LL_FEAT_BIT_PER_INIT_FEAT_XCHG | \
				  LL_FEAT_BIT_PING | \
				  LL_FEAT_BIT_DLE | \
				  LL_FEAT_BIT_PRIVACY | \
				  LL_FEAT_BIT_EXT_SCAN | \
				  LL_FEAT_BIT_PHY_2M | \
				  LL_FEAT_BIT_SMI_TX | \
				  LL_FEAT_BIT_SMI_RX | \
				  LL_FEAT_BIT_PHY_CODED | \
				  LL_FEAT_BIT_EXT_ADV | \
				  LL_FEAT_BIT_PER_ADV | \
				  LL_FEAT_BIT_CONNECTION_CTE_REQ | \
				  LL_FEAT_BIT_CONNECTION_CTE_RESP | \
				  LL_FEAT_BIT_CONNECTIONLESS_CTE_TX | \
				  LL_FEAT_BIT_CONNECTIONLESS_CTE_RX | \
				  LL_FEAT_BIT_ANT_SWITCH_TX_AOD | \
				  LL_FEAT_BIT_ANT_SWITCH_RX_AOA | \
				  LL_FEAT_BIT_RX_CTE | \
				  LL_FEAT_BIT_SCA_UPDATE | \
				  LL_FEAT_BIT_CHAN_SEL_2 | \
				  LL_FEAT_BIT_MIN_USED_CHAN | \
				  LL_FEAT_BIT_CIS_CENTRAL | \
				  LL_FEAT_BIT_CIS_PERIPHERAL | \
				  LL_FEAT_BIT_ISO_BROADCASTER | \
				  LL_FEAT_BIT_SYNC_RECEIVER | \
				  LL_FEAT_BIT_PERIODIC_ADI_SUPPORT)
