/*
 * Copyright (c) 2023 Vestas Wind Systems A/S
 * Copyright (c) 2020 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_DRIVERS_CAN_MCAN_H_
#define ZEPHYR_DRIVERS_CAN_MCAN_H_

#include <zephyr/cache.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

/*
 * The Bosch M_CAN register definitions correspond to those found in the Bosch M_CAN Controller Area
 * Network User's Manual, Revision 3.3.0.
 */

/* Core Release register */
#define CAN_MCAN_CREL         0x000
#define CAN_MCAN_CREL_REL     GENMASK(31, 28)
#define CAN_MCAN_CREL_STEP    GENMASK(27, 24)
#define CAN_MCAN_CREL_SUBSTEP GENMASK(23, 20)
#define CAN_MCAN_CREL_YEAR    GENMASK(19, 16)
#define CAN_MCAN_CREL_MON     GENMASK(15, 8)
#define CAN_MCAN_CREL_DAY     GENMASK(7, 0)

/* Endian register */
#define CAN_MCAN_ENDN     0x004
#define CAN_MCAN_ENDN_ETV GENMASK(31, 0)

/* Customer register */
#define CAN_MCAN_CUST      0x008
#define CAN_MCAN_CUST_CUST GENMASK(31, 0)

/* Data Bit Timing & Prescaler register */
#define CAN_MCAN_DBTP        0x00C
#define CAN_MCAN_DBTP_TDC    BIT(23)
#define CAN_MCAN_DBTP_DBRP   GENMASK(20, 16)
#define CAN_MCAN_DBTP_DTSEG1 GENMASK(12, 8)
#define CAN_MCAN_DBTP_DTSEG2 GENMASK(7, 4)
#define CAN_MCAN_DBTP_DSJW   GENMASK(3, 0)

/* Test register */
#define CAN_MCAN_TEST       0x010
#define CAN_MCAN_TEST_SVAL  BIT(21)
#define CAN_MCAN_TEST_TXBNS GENMASK(20, 16)
#define CAN_MCAN_TEST_PVAL  BIT(13)
#define CAN_MCAN_TEST_TXBNP GENMASK(12, 8)
#define CAN_MCAN_TEST_RX    BIT(7)
#define CAN_MCAN_TEST_TX    GENMASK(6, 5)
#define CAN_MCAN_TEST_LBCK  BIT(4)

/* RAM Watchdog register */
#define CAN_MCAN_RWD     0x014
#define CAN_MCAN_RWD_WDV GENMASK(15, 8)
#define CAN_MCAN_RWD_WDC GENMASK(7, 0)

/* CC Control register */
#define CAN_MCAN_CCCR      0x018
#define CAN_MCAN_CCCR_NISO BIT(15)
#define CAN_MCAN_CCCR_TXP  BIT(14)
#define CAN_MCAN_CCCR_EFBI BIT(13)
#define CAN_MCAN_CCCR_PXHD BIT(12)
#define CAN_MCAN_CCCR_WMM  BIT(11)
#define CAN_MCAN_CCCR_UTSU BIT(10)
#define CAN_MCAN_CCCR_BRSE BIT(9)
#define CAN_MCAN_CCCR_FDOE BIT(8)
#define CAN_MCAN_CCCR_TEST BIT(7)
#define CAN_MCAN_CCCR_DAR  BIT(6)
#define CAN_MCAN_CCCR_MON  BIT(5)
#define CAN_MCAN_CCCR_CSR  BIT(4)
#define CAN_MCAN_CCCR_CSA  BIT(3)
#define CAN_MCAN_CCCR_ASM  BIT(2)
#define CAN_MCAN_CCCR_CCE  BIT(1)
#define CAN_MCAN_CCCR_INIT BIT(0)

/* Nominal Bit Timing & Prescaler register */
#define CAN_MCAN_NBTP        0x01C
#define CAN_MCAN_NBTP_NSJW   GENMASK(31, 25)
#define CAN_MCAN_NBTP_NBRP   GENMASK(24, 16)
#define CAN_MCAN_NBTP_NTSEG1 GENMASK(15, 8)
#define CAN_MCAN_NBTP_NTSEG2 GENMASK(6, 0)

/* Timestamp Counter Configuration register */
#define CAN_MCAN_TSCC     0x020
#define CAN_MCAN_TSCC_TCP GENMASK(19, 16)
#define CAN_MCAN_TSCC_TSS GENMASK(1, 0)

/* Timestamp Counter Value register */
#define CAN_MCAN_TSCV     0x024
#define CAN_MCAN_TSCV_TSC GENMASK(15, 0)

/* Timeout Counter Configuration register */
#define CAN_MCAN_TOCC      0x028
#define CAN_MCAN_TOCC_TOP  GENMASK(31, 16)
#define CAN_MCAN_TOCC_TOS  GENMASK(2, 1)
#define CAN_MCAN_TOCC_ETOC BIT(1)

/* Timeout Counter Value register */
#define CAN_MCAN_TOCV     0x02C
#define CAN_MCAN_TOCV_TOC GENMASK(15, 0)

/* Error Counter register */
#define CAN_MCAN_ECR     0x040
#define CAN_MCAN_ECR_CEL GENMASK(23, 16)
#define CAN_MCAN_ECR_RP  BIT(15)
#define CAN_MCAN_ECR_REC GENMASK(14, 8)
#define CAN_MCAN_ECR_TEC GENMASK(7, 0)

/* Protocol Status register */
#define CAN_MCAN_PSR      0x044
#define CAN_MCAN_PSR_TDCV GENMASK(22, 16)
#define CAN_MCAN_PSR_PXE  BIT(14)
#define CAN_MCAN_PSR_RFDF BIT(13)
#define CAN_MCAN_PSR_RBRS BIT(12)
#define CAN_MCAN_PSR_RESI BIT(11)
#define CAN_MCAN_PSR_DLEC GENMASK(10, 8)
#define CAN_MCAN_PSR_BO   BIT(7)
#define CAN_MCAN_PSR_EW   BIT(6)
#define CAN_MCAN_PSR_EP   BIT(5)
#define CAN_MCAN_PSR_ACT  GENMASK(4, 3)
#define CAN_MCAN_PSR_LEC  GENMASK(2, 0)

/* Transmitter Delay Compensation register */
#define CAN_MCAN_TDCR      0x048
#define CAN_MCAN_TDCR_TDCO GENMASK(14, 8)
#define CAN_MCAN_TDCR_TDCF GENMASK(6, 0)

/* Interrupt register */
#define CAN_MCAN_IR      0x050
#define CAN_MCAN_IR_ARA  BIT(29)
#define CAN_MCAN_IR_PED  BIT(28)
#define CAN_MCAN_IR_PEA  BIT(27)
#define CAN_MCAN_IR_WDI  BIT(26)
#define CAN_MCAN_IR_BO   BIT(25)
#define CAN_MCAN_IR_EW   BIT(24)
#define CAN_MCAN_IR_EP   BIT(23)
#define CAN_MCAN_IR_ELO  BIT(22)
#define CAN_MCAN_IR_BEU  BIT(21)
#define CAN_MCAN_IR_BEC  BIT(20)
#define CAN_MCAN_IR_DRX  BIT(19)
#define CAN_MCAN_IR_TOO  BIT(18)
#define CAN_MCAN_IR_MRAF BIT(17)
#define CAN_MCAN_IR_TSW  BIT(16)
#define CAN_MCAN_IR_TEFL BIT(15)
#define CAN_MCAN_IR_TEFF BIT(14)
#define CAN_MCAN_IR_TEFW BIT(13)
#define CAN_MCAN_IR_TEFN BIT(12)
#define CAN_MCAN_IR_TFE  BIT(11)
#define CAN_MCAN_IR_TCF  BIT(10)
#define CAN_MCAN_IR_TC   BIT(9)
#define CAN_MCAN_IR_HPM  BIT(8)
#define CAN_MCAN_IR_RF1L BIT(7)
#define CAN_MCAN_IR_RF1F BIT(6)
#define CAN_MCAN_IR_RF1W BIT(5)
#define CAN_MCAN_IR_RF1N BIT(4)
#define CAN_MCAN_IR_RF0L BIT(3)
#define CAN_MCAN_IR_RF0F BIT(2)
#define CAN_MCAN_IR_RF0W BIT(1)
#define CAN_MCAN_IR_RF0N BIT(0)

/* Interrupt Enable register */
#define CAN_MCAN_IE       0x054
#define CAN_MCAN_IE_ARAE  BIT(29)
#define CAN_MCAN_IE_PEDE  BIT(28)
#define CAN_MCAN_IE_PEAE  BIT(27)
#define CAN_MCAN_IE_WDIE  BIT(26)
#define CAN_MCAN_IE_BOE   BIT(25)
#define CAN_MCAN_IE_EWE   BIT(24)
#define CAN_MCAN_IE_EPE   BIT(23)
#define CAN_MCAN_IE_ELOE  BIT(22)
#define CAN_MCAN_IE_BEUE  BIT(21)
#define CAN_MCAN_IE_BECE  BIT(20)
#define CAN_MCAN_IE_DRXE  BIT(19)
#define CAN_MCAN_IE_TOOE  BIT(18)
#define CAN_MCAN_IE_MRAFE BIT(17)
#define CAN_MCAN_IE_TSWE  BIT(16)
#define CAN_MCAN_IE_TEFLE BIT(15)
#define CAN_MCAN_IE_TEFFE BIT(14)
#define CAN_MCAN_IE_TEFWE BIT(13)
#define CAN_MCAN_IE_TEFNE BIT(12)
#define CAN_MCAN_IE_TFEE  BIT(11)
#define CAN_MCAN_IE_TCFE  BIT(10)
#define CAN_MCAN_IE_TCE   BIT(9)
#define CAN_MCAN_IE_HPME  BIT(8)
#define CAN_MCAN_IE_RF1LE BIT(7)
#define CAN_MCAN_IE_RF1FE BIT(6)
#define CAN_MCAN_IE_RF1WE BIT(5)
#define CAN_MCAN_IE_RF1NE BIT(4)
#define CAN_MCAN_IE_RF0LE BIT(3)
#define CAN_MCAN_IE_RF0FE BIT(2)
#define CAN_MCAN_IE_RF0WE BIT(1)
#define CAN_MCAN_IE_RF0NE BIT(0)

/* Interrupt Line Select register */
#define CAN_MCAN_ILS       0x058
#define CAN_MCAN_ILS_ARAL  BIT(29)
#define CAN_MCAN_ILS_PEDL  BIT(28)
#define CAN_MCAN_ILS_PEAL  BIT(27)
#define CAN_MCAN_ILS_WDIL  BIT(26)
#define CAN_MCAN_ILS_BOL   BIT(25)
#define CAN_MCAN_ILS_EWL   BIT(24)
#define CAN_MCAN_ILS_EPL   BIT(23)
#define CAN_MCAN_ILS_ELOL  BIT(22)
#define CAN_MCAN_ILS_BEUL  BIT(21)
#define CAN_MCAN_ILS_BECL  BIT(20)
#define CAN_MCAN_ILS_DRXL  BIT(19)
#define CAN_MCAN_ILS_TOOL  BIT(18)
#define CAN_MCAN_ILS_MRAFL BIT(17)
#define CAN_MCAN_ILS_TSWL  BIT(16)
#define CAN_MCAN_ILS_TEFLL BIT(15)
#define CAN_MCAN_ILS_TEFFL BIT(14)
#define CAN_MCAN_ILS_TEFWL BIT(13)
#define CAN_MCAN_ILS_TEFNL BIT(12)
#define CAN_MCAN_ILS_TFEL  BIT(11)
#define CAN_MCAN_ILS_TCFL  BIT(10)
#define CAN_MCAN_ILS_TCL   BIT(9)
#define CAN_MCAN_ILS_HPML  BIT(8)
#define CAN_MCAN_ILS_RF1LL BIT(7)
#define CAN_MCAN_ILS_RF1FL BIT(6)
#define CAN_MCAN_ILS_RF1WL BIT(5)
#define CAN_MCAN_ILS_RF1NL BIT(4)
#define CAN_MCAN_ILS_RF0LL BIT(3)
#define CAN_MCAN_ILS_RF0FL BIT(2)
#define CAN_MCAN_ILS_RF0WL BIT(1)
#define CAN_MCAN_ILS_RF0NL BIT(0)

/* Interrupt Line Enable register */
#define CAN_MCAN_ILE       0x05C
#define CAN_MCAN_ILE_EINT1 BIT(1)
#define CAN_MCAN_ILE_EINT0 BIT(0)

/* Global filter configuration register */
#define CAN_MCAN_GFC      0x080
#define CAN_MCAN_GFC_ANFS GENMASK(5, 4)
#define CAN_MCAN_GFC_ANFE GENMASK(3, 2)
#define CAN_MCAN_GFC_RRFS BIT(1)
#define CAN_MCAN_GFC_RRFE BIT(0)

/* Standard ID Filter Configuration register */
#define CAN_MCAN_SIDFC       0x084
#define CAN_MCAN_SIDFC_LSS   GENMASK(23, 16)
#define CAN_MCAN_SIDFC_FLSSA GENMASK(15, 2)

/* Extended ID Filter Configuration register */
#define CAN_MCAN_XIDFC       0x088
#define CAN_MCAN_XIDFC_LSS   GENMASK(22, 16)
#define CAN_MCAN_XIDFC_FLESA GENMASK(15, 2)

/* Extended ID AND Mask register */
#define CAN_MCAN_XIDAM      0x090
#define CAN_MCAN_XIDAM_EIDM GENMASK(28, 0)

/* High Priority Message Status register */
#define CAN_MCAN_HPMS      0x094
#define CAN_MCAN_HPMS_FLST BIT(15)
#define CAN_MCAN_HPMS_FIDX GENMASK(14, 8)
#define CAN_MCAN_HPMS_MSI  GENMASK(7, 6)
#define CAN_MCAN_HPMS_BIDX GENMASK(5, 0)

/* New Data 1 register */
#define CAN_MCAN_NDAT1    0x098
#define CAN_MCAN_NDAT1_ND GENMASK(31, 0)

/* New Data 2 register */
#define CAN_MCAN_NDAT2    0x09C
#define CAN_MCAN_NDAT2_ND GENMASK(31, 0)

/* Rx FIFO 0 Configuration register */
#define CAN_MCAN_RXF0C      0x0A0
#define CAN_MCAN_RXF0C_F0OM BIT(31)
#define CAN_MCAN_RXF0C_F0WM GENMASK(30, 24)
#define CAN_MCAN_RXF0C_F0S  GENMASK(22, 16)
#define CAN_MCAN_RXF0C_F0SA GENMASK(15, 2)

/* Rx FIFO 0 Status register */
#define CAN_MCAN_RXF0S      0x0A4
#define CAN_MCAN_RXF0S_RF0L BIT(25)
#define CAN_MCAN_RXF0S_F0F  BIT(24)
#define CAN_MCAN_RXF0S_F0PI GENMASK(21, 16)
#define CAN_MCAN_RXF0S_F0GI GENMASK(13, 8)
#define CAN_MCAN_RXF0S_F0FL GENMASK(6, 0)

/* Rx FIFO 0 Acknowledge register */
#define CAN_MCAN_RXF0A      0x0A8
#define CAN_MCAN_RXF0A_F0AI GENMASK(5, 0)

/* Rx Buffer Configuration register */
#define CAN_MCAN_RXBC      0x0AC
#define CAN_MCAN_RXBC_RBSA GENMASK(15, 2)

/* Rx FIFO 1 Configuration register */
#define CAN_MCAN_RXF1C      0x0B0
#define CAN_MCAN_RXF1C_F1OM BIT(31)
#define CAN_MCAN_RXF1C_F1WM GENMASK(30, 24)
#define CAN_MCAN_RXF1C_F1S  GENMASK(22, 16)
#define CAN_MCAN_RXF1C_F1SA GENMASK(15, 2)

/* Rx FIFO 1 Status register */
#define CAN_MCAN_RXF1S      0x0B4
#define CAN_MCAN_RXF1S_RF1L BIT(25)
#define CAN_MCAN_RXF1S_F1F  BIT(24)
#define CAN_MCAN_RXF1S_F1PI GENMASK(21, 16)
#define CAN_MCAN_RXF1S_F1GI GENMASK(13, 8)
#define CAN_MCAN_RXF1S_F1FL GENMASK(6, 0)

/* Rx FIFO 1 Acknowledge register */
#define CAN_MCAN_RXF1A      0x0B8
#define CAN_MCAN_RXF1A_F1AI GENMASK(5, 0)

/* Rx Buffer/FIFO Element Size Configuration register */
#define CAN_MCAN_RXESC      0x0BC
#define CAN_MCAN_RXESC_RBDS GENMASK(10, 8)
#define CAN_MCAN_RXESC_F1DS GENMASK(6, 4)
#define CAN_MCAN_RXESC_F0DS GENMASK(2, 0)

/* Tx Buffer Configuration register */
#define CAN_MCAN_TXBC      0x0C0
#define CAN_MCAN_TXBC_TFQM BIT(30)
#define CAN_MCAN_TXBC_TFQS GENMASK(29, 24)
#define CAN_MCAN_TXBC_NDTB GENMASK(21, 16)
#define CAN_MCAN_TXBC_TBSA GENMASK(15, 2)

/* Tx FIFO/Queue Status register */
#define CAN_MCAN_TXFQS       0x0C4
#define CAN_MCAN_TXFQS_TFQF  BIT(21)
#define CAN_MCAN_TXFQS_TFQPI GENMASK(20, 16)
#define CAN_MCAN_TXFQS_TFGI  GENMASK(12, 8)
#define CAN_MCAN_TXFQS_TFFL  GENMASK(5, 0)

/* Tx Buffer Element Size Configuration register */
#define CAN_MCAN_TXESC      0x0C8
#define CAN_MCAN_TXESC_TBDS GENMASK(2, 0)

/* Tx Buffer Request Pending register */
#define CAN_MCAN_TXBRP     0x0CC
#define CAN_MCAN_TXBRP_TRP GENMASK(31, 0)

/* Tx Buffer Add Request register */
#define CAN_MCAN_TXBAR    0x0D0
#define CAN_MCAN_TXBAR_AR GENMASK(31, 0)

/* Tx Buffer Cancellation Request register */
#define CAN_MCAN_TXBCR    0x0D4
#define CAN_MCAN_TXBCR_CR GENMASK(31, 0)

/* Tx Buffer Transmission Occurred register */
#define CAN_MCAN_TXBTO    0x0D8
#define CAN_MCAN_TXBTO_TO GENMASK(31, 0)

/* Tx Buffer Cancellation Finished register */
#define CAN_MCAN_TXBCF    0x0DC
#define CAN_MCAN_TXBCF_CF GENMASK(31, 0)

/* Tx Buffer Transmission Interrupt Enable register */
#define CAN_MCAN_TXBTIE     0x0E0
#define CAN_MCAN_TXBTIE_TIE GENMASK(31, 0)

/* Tx Buffer Cancellation Finished Interrupt Enable register */
#define CAN_MCAN_TXBCIE      0x0E4
#define CAN_MCAN_TXBCIE_CFIE GENMASK(31, 0)

/* Tx Event FIFO Configuration register */
#define CAN_MCAN_TXEFC      0x0F0
#define CAN_MCAN_TXEFC_EFWM GENMASK(29, 24)
#define CAN_MCAN_TXEFC_EFS  GENMASK(21, 16)
#define CAN_MCAN_TXEFC_EFSA GENMASK(15, 2)

/* Tx Event FIFO Status register */
#define CAN_MCAN_TXEFS      0x0F4
#define CAN_MCAN_TXEFS_TEFL BIT(25)
#define CAN_MCAN_TXEFS_EFF  BIT(24)
#define CAN_MCAN_TXEFS_EFPI GENMASK(20, 16)
#define CAN_MCAN_TXEFS_EFGI GENMASK(12, 8)
#define CAN_MCAN_TXEFS_EFFL GENMASK(5, 0)

/* Tx Event FIFO Acknowledge register */
#define CAN_MCAN_TXEFA      0x0F8
#define CAN_MCAN_TXEFA_EFAI GENMASK(4, 0)

#ifdef CONFIG_CAN_MCUX_MCAN
#define MCAN_DT_PATH DT_NODELABEL(can0)
#else
#define MCAN_DT_PATH DT_PATH(soc, can)
#endif

/**
 * @brief Indexes for the cells in the devicetree bosch,mram-cfg property. These match the
 * description of the cells in the bosch,m_can-base devicetree binding.
 */
enum can_mcan_mram_cfg {
	CAN_MCAN_MRAM_CFG_OFFSET = 0,
	CAN_MCAN_MRAM_CFG_STD_FILTER,
	CAN_MCAN_MRAM_CFG_EXT_FILTER,
	CAN_MCAN_MRAM_CFG_RX_FIFO0,
	CAN_MCAN_MRAM_CFG_RX_FIFO1,
	CAN_MCAN_MRAM_CFG_RX_BUFFER,
	CAN_MCAN_MRAM_CFG_TX_EVENT,
	CAN_MCAN_MRAM_CFG_TX_BUFFER,
	CAN_MCAN_MRAM_CFG_NUM_CELLS
};

/**
 * @brief Get the Bosch M_CAN Message RAM offset
 *
 * @param node_id node identifier
 * @return the Message RAM offset in bytes
 */
#define CAN_MCAN_DT_MRAM_OFFSET(node_id) \
	DT_PROP_BY_IDX(node_id, bosch_mram_cfg, 0)

/**
 * @brief Get the number of standard (11-bit) filter elements in Bosch M_CAN Message RAM
 *
 * @param node_id node identifier
 * @return the number of standard (11-bit) filter elements
 */
#define CAN_MCAN_DT_MRAM_STD_FILTER_ELEM(node_id) \
	DT_PROP_BY_IDX(node_id, bosch_mram_cfg, 1)

/**
 * @brief Get the number of extended (29-bit) filter elements in Bosch M_CAN Message RAM
 *
 * @param node_id node identifier
 * @return the number of extended (29-bit) filter elements
 */
#define CAN_MCAN_DT_MRAM_EXT_FILTER_ELEM(node_id) \
	DT_PROP_BY_IDX(node_id, bosch_mram_cfg, 2)

/**
 * @brief Get the number of Rx FIFO 0 elements in Bosch M_CAN Message RAM
 *
 * @param node_id node identifier
 * @return the number of Rx FIFO 0 elements
 */
#define CAN_MCAN_DT_MRAM_RX_FIFO0_ELEM(node_id) \
	DT_PROP_BY_IDX(node_id, bosch_mram_cfg, 3)

/**
 * @brief Get the number of Rx FIFO 1 elements in Bosch M_CAN Message RAM
 *
 * @param node_id node identifier
 * @return the number of Rx FIFO 1 elements
 */
#define CAN_MCAN_DT_MRAM_RX_FIFO1_ELEM(node_id) \
	DT_PROP_BY_IDX(node_id, bosch_mram_cfg, 4)

/**
 * @brief Get the number of Rx Buffer elements in Bosch M_CAN Message RAM
 *
 * @param node_id node identifier
 * @return the number of Rx Buffer elements
 */
#define CAN_MCAN_DT_MRAM_RX_BUFFER_ELEM(node_id) \
	DT_PROP_BY_IDX(node_id, bosch_mram_cfg, 5)

/**
 * @brief Get the number of Tx Event FIFO elements in Bosch M_CAN Message RAM
 *
 * @param node_id node identifier
 * @return the number of Tx Event FIFO elements
 */
#define CAN_MCAN_DT_MRAM_TX_EVENT_FIFO_ELEM(node_id) \
	DT_PROP_BY_IDX(node_id, bosch_mram_cfg, 6)

/**
 * @brief Get the number of Tx Buffer elements in Bosch M_CAN Message RAM
 *
 * @param node_id node identifier
 * @return the number of Tx Buffer elements
 */
#define CAN_MCAN_DT_MRAM_TX_BUFFER_ELEM(node_id) \
	DT_PROP_BY_IDX(node_id, bosch_mram_cfg, 7)

/**
 * @brief Equivalent to CAN_MCAN_DT_MRAM_OFFSET(DT_DRV_INST(inst))
 * @param inst DT_DRV_COMPAT instance number
 * @return the Message RAM offset in bytes
 * @see CAN_MCAN_DT_MRAM_OFFSET()
 */
#define CAN_MCAN_DT_INST_MRAM_OFFSET(inst)			\
	CAN_MCAN_DT_MRAM_OFFSET(DT_DRV_INST(inst))

/**
 * @brief Equivalent to CAN_MCAN_DT_MRAM_STD_FILTER_ELEM(DT_DRV_INST(inst))
 * @param inst DT_DRV_COMPAT instance number
 * @return the number of standard (11-bit) elements
 * @see CAN_MCAN_DT_MRAM_STD_FILTER_ELEM()
 */
#define CAN_MCAN_DT_INST_MRAM_STD_FILTER_ELEM(inst)		\
	CAN_MCAN_DT_MRAM_STD_FILTER_ELEMDT_DRV_INST(inst))

/**
 * @brief Equivalent to CAN_MCAN_DT_MRAM_EXT_FILTER_ELEM(DT_DRV_INST(inst))
 * @param inst DT_DRV_COMPAT instance number
 * @return the number of extended (29-bit) elements
 * @see CAN_MCAN_DT_MRAM_EXT_FILTER_ELEM()
 */
#define CAN_MCAN_DT_INST_MRAM_EXT_FILTER_ELEM(inst)		\
	CAN_MCAN_DT_MRAM_EXT_FILTER_ELEM(DT_DRV_INST(inst))

/**
 * @brief Equivalent to CAN_MCAN_DT_MRAM_RX_FIFO0_ELEM(DT_DRV_INST(inst))
 * @param inst DT_DRV_COMPAT instance number
 * @return the number of Rx FIFO 0 elements
 * @see CAN_MCAN_DT_MRAM_RX_FIFO0_ELEM()
 */
#define CAN_MCAN_DT_INST_MRAM_RX_FIFO0_ELEM(inst)		\
	CAN_MCAN_DT_MRAM_RX_FIFO0_ELEM(DT_DRV_INST(inst))

/**
 * @brief Equivalent to CAN_MCAN_DT_MRAM_RX_FIFO1_ELEM(DT_DRV_INST(inst))
 * @param inst DT_DRV_COMPAT instance number
 * @return the number of Rx FIFO 1 elements
 * @see CAN_MCAN_DT_MRAM_RX_FIFO1_ELEM()
 */
#define CAN_MCAN_DT_INST_MRAM_RX_FIFO1_ELEM(inst)		\
	CAN_MCAN_DT_MRAM_RX_FIFO1_ELEM(DT_DRV_INST(inst))

/**
 * @brief Equivalent to CAN_MCAN_DT_MRAM_RX_BUFFER_ELEM(DT_DRV_INST(inst))
 * @param inst DT_DRV_COMPAT instance number
 * @return the number of Rx Buffer elements
 * @see CAN_MCAN_DT_MRAM_RX_BUFFER_ELEM()
 */
#define CAN_MCAN_DT_INST_MRAM_RX_BUFFER_ELEM(inst)		\
	CAN_MCAN_DT_MRAM_RX_BUFFER_ELEM(DT_DRV_INST(inst))

/**
 * @brief Equivalent to CAN_MCAN_DT_MRAM_TX_EVENT_ELEM(DT_DRV_INST(inst))
 * @param inst DT_DRV_COMPAT instance number
 * @return the number of Tx Event FIFO elements
 * @see CAN_MCAN_DT_MRAM_TX_EVENT_FIFO_ELEM()
 */
#define CAN_MCAN_DT_INST_MRAM_TX_EVENT_FIFO_ELEM(inst)		\
	CAN_MCAN_DT_MRAM_TX_EVENT_FIFO_ELEM(DT_DRV_INST(inst))

/**
 * @brief Equivalent to CAN_MCAN_DT_MRAM_TX_BUFFER_ELEM(DT_DRV_INST(inst))
 * @param inst DT_DRV_COMPAT instance number
 * @return the number of Tx Buffer elements
 * @see CAN_MCAN_DT_MRAM_TX_BUFFER_ELEM()
 */
#define CAN_MCAN_DT_INST_MRAM_TX_BUFFER_ELEM(inst)		\
	CAN_MCAN_DT_MRAM_TX_BUFFER_ELEM(DT_DRV_INST(inst))

#define NUM_STD_FILTER_ELEMENTS    CAN_MCAN_DT_MRAM_STD_FILTER_ELEM(MCAN_DT_PATH)
#define NUM_EXT_FILTER_ELEMENTS    CAN_MCAN_DT_MRAM_EXT_FILTER_ELEM(MCAN_DT_PATH)
#define NUM_RX_FIFO0_ELEMENTS      CAN_MCAN_DT_MRAM_RX_FIFO0_ELEM(MCAN_DT_PATH)
#define NUM_RX_FIFO1_ELEMENTS      CAN_MCAN_DT_MRAM_RX_FIFO1_ELEM(MCAN_DT_PATH)
#define NUM_RX_BUF_ELEMENTS        CAN_MCAN_DT_MRAM_RX_BUFFER_ELEM(MCAN_DT_PATH)
#define NUM_TX_EVENT_FIFO_ELEMENTS CAN_MCAN_DT_MRAM_TX_EVENT_FIFO_ELEM(MCAN_DT_PATH)
#define NUM_TX_BUF_ELEMENTS        CAN_MCAN_DT_MRAM_TX_BUFFER_ELEM(MCAN_DT_PATH)

/* Assert that the Message RAM configuration meets the M_CAN IP core restrictions */
BUILD_ASSERT(NUM_STD_FILTER_ELEMENTS <= 128, "Maximum Standard filter elements exceeded");
BUILD_ASSERT(NUM_EXT_FILTER_ELEMENTS <= 64, "Maximum Extended filter elements exceeded");
BUILD_ASSERT(NUM_RX_FIFO0_ELEMENTS <= 64, "Maximum Rx FIFO 0 elements exceeded");
BUILD_ASSERT(NUM_RX_FIFO1_ELEMENTS <= 64, "Maximum Rx FIFO 1 elements exceeded");
BUILD_ASSERT(NUM_RX_BUF_ELEMENTS <= 64, "Maximum Rx Buffer elements exceeded");
BUILD_ASSERT(NUM_TX_EVENT_FIFO_ELEMENTS <= 32, "Maximum Tx Buffer elements exceeded");
BUILD_ASSERT(NUM_TX_BUF_ELEMENTS <= 32, "Maximum Tx Buffer elements exceeded");

#ifdef CONFIG_CAN_STM32FD
#define NUM_STD_FILTER_DATA CONFIG_CAN_MAX_STD_ID_FILTER
#define NUM_EXT_FILTER_DATA CONFIG_CAN_MAX_EXT_ID_FILTER
#else
#define NUM_STD_FILTER_DATA NUM_STD_FILTER_ELEMENTS
#define NUM_EXT_FILTER_DATA NUM_EXT_FILTER_ELEMENTS
#endif

struct can_mcan_rx_fifo_hdr {
	union {
		struct {
			uint32_t ext_id: 29; /* Extended Identifier */
			uint32_t rtr: 1;     /* Remote Transmission Request*/
			uint32_t xtd: 1;     /* Extended identifier */
			uint32_t esi: 1;     /* Error state indicator */
		};
		struct {
			uint32_t pad1: 18;
			uint32_t std_id: 11; /* Standard Identifier */
			uint32_t pad2: 3;
		};
	};

	uint32_t rxts: 16; /* Rx timestamp */
	uint32_t dlc: 4;   /* Data Length Code */
	uint32_t brs: 1;   /* Bit Rate Switch */
	uint32_t fdf: 1;   /* FD Format */
	uint32_t res: 2;   /* Reserved */
	uint32_t fidx: 7;  /* Filter Index */
	uint32_t anmf: 1;  /* Accepted non-matching frame */
} __packed __aligned(4);

struct can_mcan_rx_fifo {
	struct can_mcan_rx_fifo_hdr hdr;
	union {
		uint8_t data[64];
		uint32_t data_32[16];
	};
} __packed __aligned(4);

struct can_mcan_tx_buffer_hdr {
	union {
		struct {
			uint32_t ext_id: 29; /* Identifier */
			uint32_t rtr: 1;     /* Remote Transmission Request*/
			uint32_t xtd: 1;     /* Extended identifier */
			uint32_t esi: 1;     /* Error state indicator */
		};
		struct {
			uint32_t pad1: 18;
			uint32_t std_id: 11; /* Identifier */
			uint32_t pad2: 3;
		};
	};
	uint16_t res1;   /* Reserved */
	uint8_t dlc: 4;  /* Data Length Code */
	uint8_t brs: 1;  /* Bit Rate Switch */
	uint8_t fdf: 1;  /* FD Format */
	uint8_t res2: 1; /* Reserved */
	uint8_t efc: 1;  /* Event FIFO control (Store Tx events) */
	uint8_t mm;      /* Message marker */
} __packed __aligned(4);

struct can_mcan_tx_buffer {
	struct can_mcan_tx_buffer_hdr hdr;
	union {
		uint8_t data[64];
		uint32_t data_32[16];
	};
} __packed __aligned(4);

#define CAN_MCAN_TE_TX	0x1 /* TX event */
#define CAN_MCAN_TE_TXC 0x2 /* TX event in spite of cancellation */

struct can_mcan_tx_event_fifo {
	uint32_t id: 29; /* Identifier */
	uint32_t rtr: 1; /* Remote Transmission Request*/
	uint32_t xtd: 1; /* Extended identifier */
	uint32_t esi: 1; /* Error state indicator */

	uint16_t txts;  /* TX Timestamp */
	uint8_t dlc: 4; /* Data Length Code */
	uint8_t brs: 1; /* Bit Rate Switch */
	uint8_t fdf: 1; /* FD Format */
	uint8_t et: 2;	/* Event type */
	uint8_t mm;	/* Message marker */
} __packed __aligned(4);

#define CAN_MCAN_FCE_DISABLE	0x0
#define CAN_MCAN_FCE_FIFO0	0x1
#define CAN_MCAN_FCE_FIFO1	0x2
#define CAN_MCAN_FCE_REJECT	0x3
#define CAN_MCAN_FCE_PRIO	0x4
#define CAN_MCAN_FCE_PRIO_FIFO0 0x5
#define CAN_MCAN_FCE_PRIO_FIFO1 0x7

#define CAN_MCAN_SFT_RANGE    0x0
#define CAN_MCAN_SFT_DUAL     0x1
#define CAN_MCAN_SFT_MASKED   0x2
#define CAN_MCAN_SFT_DISABLED 0x3

struct can_mcan_std_filter {
	uint32_t id2: 11; /* ID2 for dual or range, mask otherwise */
	uint32_t res: 5;
	uint32_t id1: 11;
	uint32_t sfce: 3; /* Filter config */
	uint32_t sft: 2;  /* Filter type */
} __packed __aligned(4);

#define CAN_MCAN_EFT_RANGE_XIDAM 0x0
#define CAN_MCAN_EFT_DUAL	 0x1
#define CAN_MCAN_EFT_MASKED	 0x2
#define CAN_MCAN_EFT_RANGE	 0x3

struct can_mcan_ext_filter {
	uint32_t id1: 29;
	uint32_t efce: 3; /* Filter config */
	uint32_t id2: 29; /* ID2 for dual or range, mask otherwise */
	uint32_t res: 1;
	uint32_t eft: 2; /* Filter type */
} __packed __aligned(4);

struct can_mcan_msg_sram {
	struct can_mcan_std_filter std_filt[NUM_STD_FILTER_ELEMENTS];
	struct can_mcan_ext_filter ext_filt[NUM_EXT_FILTER_ELEMENTS];
	struct can_mcan_rx_fifo rx_fifo0[NUM_RX_FIFO0_ELEMENTS];
	struct can_mcan_rx_fifo rx_fifo1[NUM_RX_FIFO1_ELEMENTS];
	struct can_mcan_rx_fifo rx_buffer[NUM_RX_BUF_ELEMENTS];
	struct can_mcan_tx_event_fifo tx_event_fifo[NUM_TX_EVENT_FIFO_ELEMENTS];
	struct can_mcan_tx_buffer tx_buffer[NUM_TX_BUF_ELEMENTS];
} __packed __aligned(4);

#define CAN_MCAN_MRAM_OFFSET_STD_FILTER offsetof(struct can_mcan_msg_sram, std_filt)
#define CAN_MCAN_MRAM_OFFSET_EXT_FILTER offsetof(struct can_mcan_msg_sram, ext_filt)
#define CAN_MCAN_MRAM_OFFSET_RX_FIFO0 offsetof(struct can_mcan_msg_sram, rx_fifo0)
#define CAN_MCAN_MRAM_OFFSET_RX_FIFO1 offsetof(struct can_mcan_msg_sram, rx_fifo1)
#define CAN_MCAN_MRAM_OFFSET_RX_BUFFER offsetof(struct can_mcan_msg_sram, rx_buffer)
#define CAN_MCAN_MRAM_OFFSET_TX_EVENT_FIFO offsetof(struct can_mcan_msg_sram, tx_event_fifo)
#define CAN_MCAN_MRAM_OFFSET_TX_BUFFER offsetof(struct can_mcan_msg_sram, tx_buffer)

struct can_mcan_data {
	struct k_mutex lock;
	struct k_sem tx_sem;
	struct k_mutex tx_mtx;
	can_tx_callback_t tx_fin_cb[NUM_TX_BUF_ELEMENTS];
	void *tx_fin_cb_arg[NUM_TX_BUF_ELEMENTS];
	can_rx_callback_t rx_cb_std[NUM_STD_FILTER_DATA];
	can_rx_callback_t rx_cb_ext[NUM_EXT_FILTER_DATA];
	void *cb_arg_std[NUM_STD_FILTER_DATA];
	void *cb_arg_ext[NUM_EXT_FILTER_DATA];
	can_state_change_callback_t state_change_cb;
	void *state_change_cb_data;
	uint32_t std_filt_fd_frame;
	uint32_t std_filt_rtr;
	uint32_t std_filt_rtr_mask;
	uint16_t ext_filt_fd_frame;
	uint16_t ext_filt_rtr;
	uint16_t ext_filt_rtr_mask;
	bool started;
#ifdef CONFIG_CAN_FD_MODE
	bool fd;
#endif /* CONFIG_CAN_FD_MODE */
	void *custom;
} __aligned(4);

/**
 * @brief Bosch M_CAN driver front-end callback for reading a register value
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param reg Register offset
 * @param[out] val Register value
 *
 * @retval 0 If successful.
 * @retval -ENOTSUP Register not supported.
 * @retval -EIO General input/output error.
 */
typedef int (*can_mcan_read_reg_t)(const struct device *dev, uint16_t reg, uint32_t *val);

/**
 * @brief Bosch M_CAN driver front-end callback for writing a register value
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param reg Register offset
 * @param val Register value
 *
 * @retval 0 If successful.
 * @retval -ENOTSUP Register not supported.
 * @retval -EIO General input/output error.
 */
typedef int (*can_mcan_write_reg_t)(const struct device *dev, uint16_t reg, uint32_t val);

/**
 * @brief Bosch M_CAN driver front-end callback for reading from Message RAM
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param offset Offset from the start of the Message RAM for the given Bosch M_CAN instance. The
 *        offset must be 32-bit aligned.
 * @param[out] dst Destination for the data read. The destination address must be 32-bit aligned.
 * @param len Number of bytes to read. Must be a multiple of 4.
 *
 * @retval 0 If successful.
 * @retval -EIO General input/output error.
 */
typedef int (*can_mcan_read_mram_t)(const struct device *dev, uint16_t offset, void *dst,
				    size_t len);

/**
 * @brief Bosch M_CAN driver front-end callback for writing to Message RAM
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param offset Offset from the start of the Message RAM for the given Bosch M_CAN instance. The
 *        offset must be 32-bit aligned.
 * @param src Source for the data to be written. The source address must be 32-bit aligned.
 * @param len Number of bytes to write. Must be a multiple of 4.
 *
 * @retval 0 If successful.
 * @retval -EIO General input/output error.
 */
typedef int (*can_mcan_write_mram_t)(const struct device *dev, uint16_t offset, const void *src,
				     size_t len);

/**
 * @brief Bosch M_CAN driver front-end callback for clearing Message RAM
 *
 * Clear Message RAM by writing 0 to all words.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param offset Offset from the start of the Message RAM for the given Bosch M_CAN instance. The
 *        offset must be 32-bit aligned.
 * @param len Number of bytes to clear. Must be a multiple of 4.
 *
 * @retval 0 If successful.
 * @retval -EIO General input/output error.
 */
typedef int (*can_mcan_clear_mram_t)(const struct device *dev, uint16_t offset, size_t len);

/**
 * @brief Bosch M_CAN driver front-end operations.
 */
struct can_mcan_ops {
	can_mcan_read_reg_t read_reg;
	can_mcan_write_reg_t write_reg;
	can_mcan_read_mram_t read_mram;
	can_mcan_write_mram_t write_mram;
	can_mcan_clear_mram_t clear_mram;
};

struct can_mcan_config {
	struct can_mcan_ops *ops;
	uint32_t bus_speed;
	uint16_t sjw;
	uint16_t sample_point;
	uint16_t prop_ts1;
	uint16_t ts2;
#ifdef CONFIG_CAN_FD_MODE
	uint32_t bus_speed_data;
	uint16_t sample_point_data;
	uint8_t sjw_data;
	uint8_t prop_ts1_data;
	uint8_t ts2_data;
	uint8_t tx_delay_comp_offset;
#endif
	const struct device *phy;
	uint32_t max_bitrate;
	const void *custom;
};

/**
 * @brief Static initializer for @p can_mcan_config struct
 *
 * @param node_id Devicetree node identifier
 * @param _custom Pointer to custom driver frontend configuration structure
 * @param _ops Pointer to front-end @a can_mcan_ops
 */
#ifdef CONFIG_CAN_FD_MODE
#define CAN_MCAN_DT_CONFIG_GET(node_id, _custom, _ops)                                             \
	{                                                                                          \
		.ops = _ops,                                                                       \
		.bus_speed = DT_PROP(node_id, bus_speed),                                          \
		.sjw = DT_PROP(node_id, sjw),                                                      \
		.sample_point = DT_PROP_OR(node_id, sample_point, 0),                              \
		.prop_ts1 = DT_PROP_OR(node_id, prop_seg, 0) + DT_PROP_OR(node_id, phase_seg1, 0), \
		.ts2 = DT_PROP_OR(node_id, phase_seg2, 0),                                         \
		.bus_speed_data = DT_PROP(node_id, bus_speed_data),                                \
		.sjw_data = DT_PROP(node_id, sjw_data),                                            \
		.sample_point_data = DT_PROP_OR(node_id, sample_point_data, 0),                    \
		.prop_ts1_data = DT_PROP_OR(node_id, prop_seg_data, 0) +                           \
				 DT_PROP_OR(node_id, phase_seg1_data, 0),                          \
		.ts2_data = DT_PROP_OR(node_id, phase_seg2_data, 0),                               \
		.tx_delay_comp_offset = DT_PROP(node_id, tx_delay_comp_offset),                    \
		.phy = DEVICE_DT_GET_OR_NULL(DT_PHANDLE(node_id, phys)),                           \
		.max_bitrate = DT_CAN_TRANSCEIVER_MAX_BITRATE(node_id, 8000000),                   \
		.custom = _custom,                                                                 \
	}
#else /* CONFIG_CAN_FD_MODE */
#define CAN_MCAN_DT_CONFIG_GET(node_id, _custom, _ops)                                             \
	{                                                                                          \
		.ops = _ops,                                                                       \
		.bus_speed = DT_PROP(node_id, bus_speed),                                          \
		.sjw = DT_PROP(node_id, sjw),                                                      \
		.sample_point = DT_PROP_OR(node_id, sample_point, 0),                              \
		.prop_ts1 = DT_PROP_OR(node_id, prop_seg, 0) + DT_PROP_OR(node_id, phase_seg1, 0), \
		.ts2 = DT_PROP_OR(node_id, phase_seg2, 0),                                         \
		.phy = DEVICE_DT_GET_OR_NULL(DT_PHANDLE(node_id, phys)),                           \
		.max_bitrate = DT_CAN_TRANSCEIVER_MAX_BITRATE(node_id, 1000000),                   \
		.custom = _custom,                                                                 \
	}
#endif /* !CONFIG_CAN_FD_MODE */

/**
 * @brief Static initializer for @p can_mcan_config struct from DT_DRV_COMPAT instance
 *
 * @param inst DT_DRV_COMPAT instance number
 * @param _custom Pointer to custom driver frontend configuration structure
 * @param _ops Pointer to front-end @a can_mcan_ops
 * @see CAN_MCAN_DT_CONFIG_GET()
 */
#define CAN_MCAN_DT_CONFIG_INST_GET(inst, _custom, _ops)                                           \
	CAN_MCAN_DT_CONFIG_GET(DT_DRV_INST(inst), _custom, _ops)

/**
 * @brief Initializer for a @a can_mcan_data struct
 * @param _msg_ram Pointer to message RAM structure
 * @param _custom Pointer to custom driver frontend data structure
 */
#define CAN_MCAN_DATA_INITIALIZER(_custom)                                                         \
	{                                                                                          \
		.custom = _custom,                                                                 \
	}

/**
 * @brief Bosch M_CAN driver front-end callback helper for reading a memory mapped register
 *
 * @param base Register base address
 * @param reg Register offset
 * @param[out] val Register value
 *
 * @retval 0 Memory mapped register read always succeeds.
 */
static inline int can_mcan_sys_read_reg(mm_reg_t base, uint16_t reg, uint32_t *val)
{
	*val = sys_read32(base + reg);

	return 0;
}

/**
 * @brief Bosch M_CAN driver front-end callback helper for writing a memory mapped register
 *
 * @param base Register base address
 * @param reg Register offset
 * @param val Register value
 *
 * @retval 0 Memory mapped register write always succeeds.
 */
static inline int can_mcan_sys_write_reg(mm_reg_t base, uint16_t reg, uint32_t val)
{
	sys_write32(val, base + reg);

	return 0;
}

/**
 * @brief Bosch M_CAN driver front-end callback helper for reading from memory mapped Message RAM
 *
 * @param base Base address of the Message RAM for the given Bosch M_CAN instance. The base address
 *        must be 32-bit aligned.
 * @param offset Offset from the start of the Message RAM for the given Bosch M_CAN instance. The
 *        offset must be 32-bit aligned.
 * @param[out] dst Destination for the data read. The destination address must be 32-bit aligned.
 * @param len Number of bytes to read. Must be a multiple of 4.
 *
 * @retval 0 If successful.
 * @retval -EIO General input/output error.
 */
static inline int can_mcan_sys_read_mram(mem_addr_t base, uint16_t offset, void *dst, size_t len)
{
	volatile uint32_t *src32 = (volatile uint32_t *)(base + offset);
	uint32_t *dst32 = (uint32_t *)dst;
	size_t len32 = len / sizeof(uint32_t);

	__ASSERT(base % 4U == 0U, "base must be a multiple of 4");
	__ASSERT(offset % 4U == 0U, "offset must be a multiple of 4");
	__ASSERT(POINTER_TO_UINT(dst) % 4U == 0U, "dst must be 32-bit aligned");
	__ASSERT(len % 4U == 0U, "len must be a multiple of 4");

#if defined(CONFIG_CACHE_MANAGEMENT) && defined(CONFIG_DCACHE)
	int err;

	err = sys_cache_data_invd_range((void *)(base + offset), len);
	if (err != 0) {
		return err;
	}
#endif /* !defined(CONFIG_CACHE_MANAGEMENT) && defined(CONFIG_DCACHE) */

	while (len32-- > 0) {
		*dst32++ = *src32++;
	}

	return 0;
}

/**
 * @brief Bosch M_CAN driver front-end callback helper for writing to memory mapped Message RAM
 *
 * @param base Base address of the Message RAM for the given Bosch M_CAN instance. The base address
 *        must be 32-bit aligned.
 * @param offset Offset from the start of the Message RAM for the given Bosch M_CAN instance. The
 *        offset must be 32-bit aligned.
 * @param src Source for the data to be written. The source address must be 32-bit aligned.
 * @param len Number of bytes to write. Must be a multiple of 4.
 *
 * @retval 0 If successful.
 * @retval -EIO General input/output error.
 */
static inline int can_mcan_sys_write_mram(mem_addr_t base, uint16_t offset, const void *src,
					  size_t len)
{
	volatile uint32_t *dst32 = (volatile uint32_t *)(base + offset);
	const uint32_t *src32 = (const uint32_t *)src;
	size_t len32 = len / sizeof(uint32_t);

	__ASSERT(base % 4U == 0U, "base must be a multiple of 4");
	__ASSERT(offset % 4U == 0U, "offset must be a multiple of 4");
	__ASSERT(POINTER_TO_UINT(src) % 4U == 0U, "src must be 32-bit aligned");
	__ASSERT(len % 4U == 0U, "len must be a multiple of 4");

	while (len32-- > 0) {
		*dst32++ = *src32++;
	}

#if defined(CONFIG_CACHE_MANAGEMENT) && defined(CONFIG_DCACHE)
	return sys_cache_data_flush_range((void *)(base + offset), len);
#else /* defined(CONFIG_CACHE_MANAGEMENT) && defined(CONFIG_DCACHE) */
	return 0;
#endif /* !defined(CONFIG_CACHE_MANAGEMENT) && defined(CONFIG_DCACHE) */
}

/**
 * @brief Bosch M_CAN driver front-end callback helper for clearing memory mapped Message RAM
 *
 * Clear Message RAM by writing 0 to all words.
 *
 * @param base Base address of the Message RAM for the given Bosch M_CAN instance. The base address
 *        must be 32-bit aligned.
 * @param offset Offset from the start of the Message RAM for the given Bosch M_CAN instance. The
 *        offset must be 32-bit aligned.
 * @param len Number of bytes to clear. Must be a multiple of 4.
 *
 * @retval 0 If successful.
 * @retval -EIO General input/output error.
 */
static inline int can_mcan_sys_clear_mram(mem_addr_t base, uint16_t offset, size_t len)
{
	volatile uint32_t *dst32 = (volatile uint32_t *)(base + offset);
	size_t len32 = len / sizeof(uint32_t);

	__ASSERT(base % 4U == 0U, "base must be a multiple of 4");
	__ASSERT(offset % 4U == 0U, "offset must be a multiple of 4");
	__ASSERT(len % 4U == 0U, "len must be a multiple of 4");

	while (len32-- > 0) {
		*dst32++ = 0U;
	}

#if defined(CONFIG_CACHE_MANAGEMENT) && defined(CONFIG_DCACHE)
	return sys_cache_data_flush_range((void *)(base + offset), len);
#else /* defined(CONFIG_CACHE_MANAGEMENT) && defined(CONFIG_DCACHE) */
	return 0;
#endif /* !defined(CONFIG_CACHE_MANAGEMENT) && defined(CONFIG_DCACHE) */
}

/**
 * @brief Read a Bosch M_CAN register
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param reg Register offset
 * @param[out] val Register value
 *
 * @retval 0 If successful.
 * @retval -ENOTSUP Register not supported.
 * @retval -EIO General input/output error.
 */
int can_mcan_read_reg(const struct device *dev, uint16_t reg, uint32_t *val);

/**
 * @brief Write a Bosch M_CAN register
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param reg Register offset
 * @param val Register value
 *
 * @retval 0 If successful.
 * @retval -ENOTSUP Register not supported.
 * @retval -EIO General input/output error.
 */
int can_mcan_write_reg(const struct device *dev, uint16_t reg, uint32_t val);

/**
 * @brief Read from Bosch M_CAN Message RAM
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param offset Offset from the start of the Message RAM for the given Bosch M_CAN instance. The
 *        offset must be 32-bit aligned.
 * @param[out] dst Destination for the data read.
 * @param len Number of bytes to read. Must be a multiple of 4.
 *
 * @retval 0 If successful.
 * @retval -EIO General input/output error.
 */
static inline int can_mcan_read_mram(const struct device *dev, uint16_t offset, void *dst,
				     size_t len)
{
	const struct can_mcan_config *config = dev->config;

	return config->ops->read_mram(dev, offset, dst, len);
}

/**
 * @brief Write to Bosch M_CAN Message RAM
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param offset Offset from the start of the Message RAM for the given Bosch M_CAN instance. The
 *        offset must be 32-bit aligned.
 * @param src Source for the data to be written
 * @param len Number of bytes to write. Must be a multiple of 4.
 *
 * @retval 0 If successful.
 * @retval -EIO General input/output error.
 */
static inline int can_mcan_write_mram(const struct device *dev, uint16_t offset, const void *src,
				      size_t len)
{
	const struct can_mcan_config *config = dev->config;

	return config->ops->write_mram(dev, offset, src, len);
}

/**
 * @brief Clear Bosch M_CAN Message RAM
 *
 * Clear Message RAM by writing 0 to all words.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param offset Offset from the start of the Message RAM for the given Bosch M_CAN instance. The
 *        offset must be 32-bit aligned.
 * @param len Number of bytes to clear. Must be a multiple of 4.
 *
 * @retval 0 If successful.
 * @retval -EIO General input/output error.
 */
static inline int can_mcan_clear_mram(const struct device *dev, uint16_t offset, size_t len)
{
	const struct can_mcan_config *config = dev->config;

	return config->ops->clear_mram(dev, offset, len);
}

/**
 * @brief Configure Bosch M_MCAN Message RAM start addresses.
 *
 * Bosch M_CAN driver front-end callback helper function for configuring the start addresses of the
 * Bosch M_CAN Rx FIFO0 (RXFOC), Rx FIFO1 (RXF1C), Rx Buffer (RXBCC), Tx Buffer (TXBC), and Tx Event
 * FIFO (TXEFC) in Message RAM.
 *
 * The start addresses (containing bits 15:2 since Bosch M_CAN message RAM is accessed as 32 bit
 * words) are calculated relative to the provided Message RAM Base Address (mrba).
 *
 * Some Bosch M_CAN implementations use a fixed Message RAM configuration, other use a fixed memory
 * area and relative addressing, others again have custom registers for configuring the Message
 * RAM. It is the responsibility of the front-end driver to call this function during driver
 * initialization as needed.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param mrba Message RAM Base Address.
 * @param mram Message RAM Address.
 *
 * @retval 0 If successful.
 * @retval -EIO General input/output error.
 */
int can_mcan_configure_mram(const struct device *dev, uintptr_t mrba, uintptr_t mram);

int can_mcan_get_capabilities(const struct device *dev, can_mode_t *cap);

int can_mcan_start(const struct device *dev);

int can_mcan_stop(const struct device *dev);

int can_mcan_set_mode(const struct device *dev, can_mode_t mode);

int can_mcan_set_timing(const struct device *dev, const struct can_timing *timing);

int can_mcan_set_timing_data(const struct device *dev, const struct can_timing *timing_data);

int can_mcan_init(const struct device *dev);

void can_mcan_line_0_isr(const struct device *dev);

void can_mcan_line_1_isr(const struct device *dev);

int can_mcan_recover(const struct device *dev, k_timeout_t timeout);

int can_mcan_send(const struct device *dev, const struct can_frame *frame, k_timeout_t timeout,
		  can_tx_callback_t callback, void *user_data);

int can_mcan_get_max_filters(const struct device *dev, bool ide);

int can_mcan_add_rx_filter(const struct device *dev, can_rx_callback_t callback, void *user_data,
			   const struct can_filter *filter);

void can_mcan_remove_rx_filter(const struct device *dev, int filter_id);

int can_mcan_get_state(const struct device *dev, enum can_state *state,
		       struct can_bus_err_cnt *err_cnt);

void can_mcan_set_state_change_callback(const struct device *dev,
					can_state_change_callback_t callback, void *user_data);

int can_mcan_get_max_bitrate(const struct device *dev, uint32_t *max_bitrate);

void can_mcan_enable_configuration_change(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_CAN_MCAN_H_ */
