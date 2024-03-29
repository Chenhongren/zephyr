/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_IMX8QM_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_IMX8QM_PINCTRL_H_

/* values for pad field */
#define SC_P_UART0_RTS_B 23
#define SC_P_UART0_CTS_B 24
#define SC_P_SAI1_RXD 128
#define SC_P_SAI1_TXC 130
#define SC_P_SAI1_TXD 131
#define SC_P_SAI1_TXFS 132

/* mux values */
#define IMX8QM_DMA_LPUART2_RX_UART0_RTS_B 2 /* UART0_RTS_B ---> DMA_LPUART2_RX */
#define IMX8QM_DMA_LPUART2_TX_UART0_CTS_B 2 /* DMA_LPUART2_TX ---> UART0_CTS_B */
#define IMX8QM_AUD_SAI1_RXD_SAI1_RXD 0 /* AUD_SAI1_RXD <--- SAI1_RXD */
#define IMX8QM_AUD_SAI1_TXC_SAI1_TXC 0 /* AUD_SAI1_TXC <---> SAI1_TXC */
#define IMX8QM_AUD_SAI1_TXD_SAI1_TXD 0 /* AUD_SAI1_TXD ---> SAI1_TXD */
#define IMX8QM_AUD_SAI1_TXFS_SAI1_TXFS 0 /* AUD_SAI1_TXFS <---> SAI1_TXFS */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_IMX8QM_PINCTRL_H_ */
