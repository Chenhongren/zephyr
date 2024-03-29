/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_IMX8QXP_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_IMX8QXP_PINCTRL_H_

/* values for pad field */
#define SC_P_SAI1_RXD 86
#define SC_P_SAI1_RXC 87
#define SC_P_SAI1_RXFS 88
#define SC_P_SPI0_CS1 96
#define SC_P_UART2_TX 113
#define SC_P_UART2_RX 114

/* mux values */
#define IMX8QXP_DMA_LPUART2_RX_UART2_RX 0 /* UART2_RX ---> DMA_LPUART2_RX */
#define IMX8QXP_DMA_LPUART2_TX_UART2_TX 0 /* DMA_LPUART2_TX ---> UART2_TX */
#define IMX8QXP_ADMA_SAI1_TXFS_SAI1_RXFS 1 /* ADMA_SAI1_TXFS <---> SAI1_RXFS */
#define IMX8QXP_ADMA_SAI1_RXD_SAI1_RXD 0 /* ADMA_SAI1_RXD <--- SAI1_RXD */
#define IMX8QXP_ADMA_SAI1_TXC_SAI1_RXC 1 /* ADMA_SAI1_TXC <---> SAI1_RXC */
#define IMX8QXP_ADMA_SAI1_TXD_SPI0_CS1 2 /* ADMA_SAI1_TXD ---> SPI0_CS1 */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_IMX8QXP_PINCTRL_H_ */
