/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Use the NRF_RTC instance for coarse radio event scheduling */
#define NRF_RTC NRF_RTC0

/* HAL abstraction of event timer prescaler value */
#define HAL_EVENT_TIMER_PRESCALER_VALUE 4U

/* NRF Radio HW timing constants
 * - provided in US and NS (for higher granularity)
 * - based on empirical measurements and sniffer logs
 */

/* TXEN->TXIDLE + TXIDLE->TX (with fast Radio ramp-up mode)
 * in microseconds for LE 1M PHY.
 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_FAST_NS 40900 /*40.1 + 0.8*/
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_FAST_US \
	HAL_RADIO_NS2US_ROUND(HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_FAST_NS)

/* TXEN->TXIDLE + TXIDLE->TX (with default Radio ramp-up mode)
 * in microseconds for LE 1M PHY.
 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_DEFAULT_NS 140900 /*140.1 + 0.8*/
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_DEFAULT_US \
	HAL_RADIO_NS2US_ROUND(HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_DEFAULT_NS)

/* TXEN->TXIDLE + TXIDLE->TX (with default Radio ramp-up mode
 * and no HW TIFS auto-switch) in microseconds for LE 1M PHY.
 */
 /* 129.5 + 0.8 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_DEFAULT_NO_HW_TIFS_NS 130300
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_DEFAULT_NO_HW_TIFS_US \
	HAL_RADIO_NS2US_ROUND( \
		HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_DEFAULT_NO_HW_TIFS_NS)

/* TXEN->TXIDLE + TXIDLE->TX (with fast Radio ramp-up mode)
 * in microseconds for LE 2M PHY.
 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_FAST_NS 40000 /* 40.1 - 0.1 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_FAST_US \
	HAL_RADIO_NS2US_ROUND(HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_FAST_NS)

/* TXEN->TXIDLE + TXIDLE->TX (with default Radio ramp-up mode)
 * in microseconds for LE 2M PHY.
 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_DEFAULT_NS 144900 /* 145 - 0.1 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_DEFAULT_US \
	HAL_RADIO_NS2US_ROUND(HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_DEFAULT_NS)

/* TXEN->TXIDLE + TXIDLE->TX (with default Radio ramp-up mode and
 * no HW TIFS auto-switch) in microseconds for LE 2M PHY.
 */
/* 129.5 - 0.1 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_DEFAULT_NO_HW_TIFS_NS 129400
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_DEFAULT_NO_HW_TIFS_US \
	HAL_RADIO_NS2US_ROUND( \
		HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_DEFAULT_NO_HW_TIFS_NS)

/* TXEN->TXIDLE + TXIDLE->TX (with fast Radio ramp-up mode)
 * in microseconds for LE CODED PHY [S2].
 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_FAST_NS 42300 /* 40.1 + 2.2 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_FAST_US \
	HAL_RADIO_NS2US_ROUND(HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_FAST_NS)

/* TXEN->TXIDLE + TXIDLE->TX (with default Radio ramp-up mode)
 * in microseconds for LE 2M PHY [S2].
 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_DEFAULT_NS 132200 /* 130 + 2.2 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_DEFAULT_US \
	HAL_RADIO_NS2US_ROUND(HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_DEFAULT_NS)

/* TXEN->TXIDLE + TXIDLE->TX (with default Radio ramp-up mode and
 * no HW TIFS auto-switch) in microseconds for LE 2M PHY [S2].
 */
/* 129.5 + 2.2 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_DEFAULT_NO_HW_TIFS_NS 131700
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_DEFAULT_NO_HW_TIFS_US \
	HAL_RADIO_NS2US_ROUND( \
		HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_DEFAULT_NO_HW_TIFS_NS)

/* TXEN->TXIDLE + TXIDLE->TX (with fast Radio ramp-up mode)
 * in microseconds for LE CODED PHY [S8].
 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_FAST_NS 42300 /* 40.1 + 2.2 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_FAST_US \
	HAL_RADIO_NS2US_ROUND(HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_FAST_NS)
/* TXEN->TXIDLE + TXIDLE->TX (with default Radio ramp-up mode)
 * in microseconds for LE 2M PHY [S8].
 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_DEFAULT_NS 121800 /*119.6 + 2.2*/
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_DEFAULT_US \
	HAL_RADIO_NS2US_ROUND(HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_DEFAULT_NS)

/* TXEN->TXIDLE + TXIDLE->TX (with default Radio ramp-up mode and
 * no HW TIFS auto-switch) in microseconds for LE 2M PHY [S8].
 */
 /* 129.5 + 2.2 */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_DEFAULT_NO_HW_TIFS_NS 131700
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_DEFAULT_NO_HW_TIFS_US \
	HAL_RADIO_NS2US_ROUND( \
		HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_DEFAULT_NO_HW_TIFS_NS)

/* RXEN->RXIDLE + RXIDLE->RX (with fast Radio ramp-up mode)
 * in microseconds for LE 1M PHY.
 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_FAST_NS 40300 /* 40.1 + 0.2 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_FAST_US \
	HAL_RADIO_NS2US_CEIL(HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_FAST_NS)

/* RXEN->RXIDLE + RXIDLE->RX (with default Radio ramp-up mode)
 * in microseconds for LE 1M PHY.
 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_DEFAULT_NS 140300 /*140.1 + 0.2*/
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_DEFAULT_US \
	HAL_RADIO_NS2US_CEIL(HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_DEFAULT_NS)

/* RXEN->RXIDLE + RXIDLE->RX (with default Radio ramp-up mode and
 * no HW TIFS auto-switch) in microseconds for LE 1M PHY.
 */
/* 129.5 + 0.2 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_DEFAULT_NO_HW_TIFS_NS 129700
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_DEFAULT_NO_HW_TIFS_US \
	HAL_RADIO_NS2US_CEIL( \
		HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_DEFAULT_NO_HW_TIFS_NS)

/* RXEN->RXIDLE + RXIDLE->RX (with fast Radio ramp-up mode)
 * in microseconds for LE 2M PHY.
 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_FAST_NS 40300 /* 40.1 + 0.2 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_FAST_US \
	HAL_RADIO_NS2US_CEIL(HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_FAST_NS)

/* RXEN->RXIDLE + RXIDLE->RX (with default Radio ramp-up mode)
 * in microseconds for LE 2M PHY.
 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_DEFAULT_NS 144800 /*144.6 + 0.2*/
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_DEFAULT_US \
	HAL_RADIO_NS2US_CEIL(HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_DEFAULT_NS)

/* RXEN->RXIDLE + RXIDLE->RX (with default Radio ramp-up mode and
 * no HW TIFS auto-switch) in microseconds for LE 2M PHY.
 */
/* 129.5 + 0.2 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_DEFAULT_NO_HW_TIFS_NS 129700
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_DEFAULT_NO_HW_TIFS_US \
	HAL_RADIO_NS2US_CEIL( \
		HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_DEFAULT_NO_HW_TIFS_NS)

/* RXEN->RXIDLE + RXIDLE->RX (with fast Radio ramp-up mode)
 * in microseconds for LE Coded PHY [S2].
 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_FAST_NS 40300 /* 40.1 + 0.2 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_FAST_US \
	HAL_RADIO_NS2US_CEIL(HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_FAST_NS)

/* RXEN->RXIDLE + RXIDLE->RX (with default Radio ramp-up mode)
 * in microseconds for LE Coded PHY [S2].
 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_DEFAULT_NS 130200 /* 130 + 0.2 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_DEFAULT_US \
	HAL_RADIO_NS2US_CEIL(HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_DEFAULT_NS)

/* RXEN->RXIDLE + RXIDLE->RX (with default Radio ramp-up mode
 * and no HW TIFS auto-switch) in microseconds for LE Coded PHY [S2].
 */
/* 129.5 + 0.2 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_DEFAULT_NO_HW_TIFS_NS 129700
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_DEFAULT_NO_HW_TIFS_US \
	HAL_RADIO_NS2US_CEIL( \
		HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_DEFAULT_NO_HW_TIFS_NS)

/* RXEN->RXIDLE + RXIDLE->RX (with fast Radio ramp-up mode)
 * in microseconds for LE Coded PHY [S8].
 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_FAST_NS 40300 /* 40.1 + 0.2 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_FAST_US \
	HAL_RADIO_NS2US_CEIL(HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_FAST_NS)

/* RXEN->RXIDLE + RXIDLE->RX (with default Radio ramp-up mode)
 * in microseconds for LE Coded PHY [S8].
 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_DEFAULT_NS 120200 /* 120.0 + 0.2 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_DEFAULT_US \
	HAL_RADIO_NS2US_CEIL(HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_DEFAULT_NS)

/* RXEN->RXIDLE + RXIDLE->RX (with default Radio ramp-up mode and
 * no HW TIFS auto-switch) in microseconds for LE Coded PHY [S8].
 */
/* 129.5 + 0.2 */
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_DEFAULT_NO_HW_TIFS_NS 129700
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_DEFAULT_NO_HW_TIFS_US \
	HAL_RADIO_NS2US_CEIL( \
		HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_DEFAULT_NO_HW_TIFS_NS)

#define HAL_RADIO_NRF52820_TX_CHAIN_DELAY_1M_US  1 /* ceil(0.6) */
#define HAL_RADIO_NRF52820_TX_CHAIN_DELAY_1M_NS  600 /* 0.6 */
#define HAL_RADIO_NRF52820_TX_CHAIN_DELAY_2M_US  1 /* ceil(0.6) */
#define HAL_RADIO_NRF52820_TX_CHAIN_DELAY_2M_NS  600 /* 0.6 */
#define HAL_RADIO_NRF52820_TX_CHAIN_DELAY_S2_US  1 /* ceil(0.6) */
#define HAL_RADIO_NRF52820_TX_CHAIN_DELAY_S2_NS  600 /* 0.6 */
#define HAL_RADIO_NRF52820_TX_CHAIN_DELAY_S8_US  1 /* ceil(0.6) */
#define HAL_RADIO_NRF52820_TX_CHAIN_DELAY_S8_NS  600 /* 0.6 */

#define HAL_RADIO_NRF52820_RX_CHAIN_DELAY_1M_US  10 /* ceil(9.4) */
#define HAL_RADIO_NRF52820_RX_CHAIN_DELAY_1M_NS  9400 /* 9.4 */
#define HAL_RADIO_NRF52820_RX_CHAIN_DELAY_2M_US  5 /* ceil(5.0) */
#define HAL_RADIO_NRF52820_RX_CHAIN_DELAY_2M_NS  5000 /* 5.0 */
#define HAL_RADIO_NRF52820_RX_CHAIN_DELAY_S2_US  25 /* ceil(19.6) */
#define HAL_RADIO_NRF52820_RX_CHAIN_DELAY_S2_NS  24600 /* 19.6 */
#define HAL_RADIO_NRF52820_RX_CHAIN_DELAY_S8_US  30 /* ceil(29.6) */
#define HAL_RADIO_NRF52820_RX_CHAIN_DELAY_S8_NS  29600 /* 29.6 */

#if defined(CONFIG_BT_CTLR_RADIO_ENABLE_FAST)
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_US \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_FAST_US
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_NS \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_FAST_NS

#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_US \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_FAST_US
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_NS \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_FAST_NS

#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_US \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_FAST_US
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_NS \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_FAST_NS

#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_US \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_FAST_US
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_NS \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_FAST_NS

#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_US \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_FAST_US
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_NS \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_FAST_NS

#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_US \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_FAST_US
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_NS \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_FAST_NS

#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_US \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_FAST_US
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_NS \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_FAST_NS

#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_US \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_FAST_US
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_NS \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_FAST_NS

#else /* !CONFIG_BT_CTLR_RADIO_ENABLE_FAST */
#if defined(CONFIG_BT_CTLR_TIFS_HW)
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_US \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_DEFAULT_US
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_NS \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_DEFAULT_NS

#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_US \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_DEFAULT_US
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_NS \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_DEFAULT_NS

#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_US \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_DEFAULT_US
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_NS \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_DEFAULT_NS

#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_US \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_DEFAULT_US
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_NS \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_DEFAULT_NS

#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_US \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_DEFAULT_US
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_NS \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_DEFAULT_NS

#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_US \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_DEFAULT_US
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_NS \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_DEFAULT_NS

#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_US \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_DEFAULT_US
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_NS \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_DEFAULT_NS

#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_US \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_DEFAULT_US
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_NS \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_DEFAULT_NS

#else /* !CONFIG_BT_CTLR_TIFS_HW */
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_US \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_DEFAULT_NO_HW_TIFS_US
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_NS \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_DEFAULT_NO_HW_TIFS_NS

#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_US \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_DEFAULT_NO_HW_TIFS_US
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_NS \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_DEFAULT_NO_HW_TIFS_NS

#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_US \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_DEFAULT_NO_HW_TIFS_US
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_NS \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_DEFAULT_NO_HW_TIFS_NS

#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_US \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_DEFAULT_NO_HW_TIFS_US
#define HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_NS \
	HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_DEFAULT_NO_HW_TIFS_NS

#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_US \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_DEFAULT_NO_HW_TIFS_US
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_NS \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_DEFAULT_NO_HW_TIFS_NS

#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_US \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_DEFAULT_NO_HW_TIFS_US
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_NS \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_DEFAULT_NO_HW_TIFS_NS

#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_US \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_DEFAULT_NO_HW_TIFS_US
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_NS \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_DEFAULT_NO_HW_TIFS_NS

#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_US \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_DEFAULT_NO_HW_TIFS_US
#define HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_NS \
	HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_DEFAULT_NO_HW_TIFS_NS
#endif /* !CONFIG_BT_CTLR_TIFS_HW */
#endif /* !CONFIG_BT_CTLR_RADIO_ENABLE_FAST */

/* HAL abstraction of Radio bitfields */
#define HAL_RADIO_INTENSET_DISABLED_Msk           RADIO_INTENSET_DISABLED_Msk
#define HAL_RADIO_SHORTS_TRX_END_DISABLE_Msk      RADIO_SHORTS_END_DISABLE_Msk
#define HAL_RADIO_SHORTS_TRX_PHYEND_DISABLE_Msk   RADIO_SHORTS_PHYEND_DISABLE_Msk
#define HAL_RADIO_CLEARPATTERN_CLEARPATTERN_Clear RADIO_CLEARPATTERN_CLEARPATTERN_Clear

/* HAL abstraction of Radio IRQ number */
#define HAL_RADIO_IRQn                          RADIO_IRQn

/* SoC specific NRF_RADIO power-on reset value. Refer to Product Specification,
 * RADIO Registers section for the documented reset values.
 *
 * NOTE: Only implementation used values defined here.
 *       In the future if MDK or nRFx header include these, use them instead.
 */
#define HAL_RADIO_RESET_VALUE_DFEMODE       0x00000000UL
#define HAL_RADIO_RESET_VALUE_CTEINLINECONF 0x00002800UL

static inline void hal_radio_reset(void)
{
	/* TODO: Add any required setup for each radio event
	 */
}

static inline void hal_radio_stop(void)
{
	/* TODO: Add any required cleanup of actions taken in hal_radio_reset()
	 */
}

static inline void hal_radio_ram_prio_setup(void)
{
	struct {
		uint32_t volatile reserved_0[0x5a0 >> 2];
		uint32_t volatile bridge_type;
		uint32_t volatile reserved_1[((0xe00 - 0x5a0) >> 2) - 1];
		struct {
			uint32_t volatile CPU0;
			uint32_t volatile SPIS1;
			uint32_t volatile RADIO;
			uint32_t volatile ECB;
			uint32_t volatile CCM;
			uint32_t volatile AAR;
			uint32_t volatile reserved;
			uint32_t volatile UARTE;
			uint32_t volatile SERIAL0;
		} RAMPRI;
	} volatile *NRF_AMLI = (void volatile *)0x40000000UL;

	NRF_AMLI->RAMPRI.CPU0    = 0xFFFFFFFFUL;
	NRF_AMLI->RAMPRI.SPIS1   = 0xFFFFFFFFUL;
	NRF_AMLI->RAMPRI.RADIO   = 0x00000000UL;
	NRF_AMLI->RAMPRI.ECB     = 0xFFFFFFFFUL;
	NRF_AMLI->RAMPRI.CCM     = 0x00000000UL;
	NRF_AMLI->RAMPRI.AAR     = 0xFFFFFFFFUL;
	NRF_AMLI->RAMPRI.UARTE   = 0xFFFFFFFFUL;
	NRF_AMLI->RAMPRI.SERIAL0 = 0xFFFFFFFFUL;
}

static inline uint32_t hal_radio_phy_mode_get(uint8_t phy, uint8_t flags)
{
	uint32_t mode;

	switch (phy) {
	case BIT(0):
	default:
		mode = RADIO_MODE_MODE_Ble_1Mbit;
		break;

	case BIT(1):
		mode = RADIO_MODE_MODE_Ble_2Mbit;
		break;

#if defined(CONFIG_BT_CTLR_PHY_CODED)
	case BIT(2):
		if (flags & 0x01) {
			mode = RADIO_MODE_MODE_Ble_LR125Kbit;
		} else {
			mode = RADIO_MODE_MODE_Ble_LR500Kbit;
		}
		break;
#endif /* CONFIG_BT_CTLR_PHY_CODED */
	}

	return mode;
}

static inline uint32_t hal_radio_tx_power_min_get(void)
{
	return RADIO_TXPOWER_TXPOWER_Neg40dBm;
}

static inline uint32_t hal_radio_tx_power_max_get(void)
{
	return RADIO_TXPOWER_TXPOWER_Pos8dBm;
}

static inline uint32_t hal_radio_tx_power_floor(int8_t tx_power_lvl)
{
	if (tx_power_lvl >= (int8_t)RADIO_TXPOWER_TXPOWER_Pos8dBm) {
		return RADIO_TXPOWER_TXPOWER_Pos8dBm;
	}

	if (tx_power_lvl >= (int8_t)RADIO_TXPOWER_TXPOWER_Pos7dBm) {
		return RADIO_TXPOWER_TXPOWER_Pos7dBm;
	}

	if (tx_power_lvl >= (int8_t)RADIO_TXPOWER_TXPOWER_Pos6dBm) {
		return RADIO_TXPOWER_TXPOWER_Pos6dBm;
	}

	if (tx_power_lvl >= (int8_t)RADIO_TXPOWER_TXPOWER_Pos5dBm) {
		return RADIO_TXPOWER_TXPOWER_Pos5dBm;
	}

	if (tx_power_lvl >= (int8_t)RADIO_TXPOWER_TXPOWER_Pos4dBm) {
		return RADIO_TXPOWER_TXPOWER_Pos4dBm;
	}

	if (tx_power_lvl >= (int8_t)RADIO_TXPOWER_TXPOWER_Pos3dBm) {
		return RADIO_TXPOWER_TXPOWER_Pos3dBm;
	}

	if (tx_power_lvl >= (int8_t)RADIO_TXPOWER_TXPOWER_Pos2dBm) {
		return RADIO_TXPOWER_TXPOWER_Pos2dBm;
	}

	if (tx_power_lvl >= (int8_t)RADIO_TXPOWER_TXPOWER_0dBm) {
		return RADIO_TXPOWER_TXPOWER_0dBm;
	}

	if (tx_power_lvl >= (int8_t)RADIO_TXPOWER_TXPOWER_Neg4dBm) {
		return RADIO_TXPOWER_TXPOWER_Neg4dBm;
	}

	if (tx_power_lvl >= (int8_t)RADIO_TXPOWER_TXPOWER_Neg8dBm) {
		return RADIO_TXPOWER_TXPOWER_Neg8dBm;
	}

	if (tx_power_lvl >= (int8_t)RADIO_TXPOWER_TXPOWER_Neg12dBm) {
		return RADIO_TXPOWER_TXPOWER_Neg12dBm;
	}

	if (tx_power_lvl >= (int8_t)RADIO_TXPOWER_TXPOWER_Neg16dBm) {
		return RADIO_TXPOWER_TXPOWER_Neg16dBm;
	}

	if (tx_power_lvl >= (int8_t)RADIO_TXPOWER_TXPOWER_Neg20dBm) {
		return RADIO_TXPOWER_TXPOWER_Neg20dBm;
	}

	/* Note: The -30 dBm power level is deprecated so ignore it! */
	return RADIO_TXPOWER_TXPOWER_Neg40dBm;
}

static inline uint32_t hal_radio_tx_ready_delay_us_get(uint8_t phy, uint8_t flags)
{
	switch (phy) {
	default:
	case BIT(0):
		return HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_US;
	case BIT(1):
		return HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_US;

#if defined(CONFIG_BT_CTLR_PHY_CODED)
	case BIT(2):
		if (flags & 0x01) {
			return HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_US;
		} else {
			return HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_US;
		}
#endif /* CONFIG_BT_CTLR_PHY_CODED */
	}
}

static inline uint32_t hal_radio_rx_ready_delay_us_get(uint8_t phy, uint8_t flags)
{
	switch (phy) {
	default:
	case BIT(0):
		return HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_US;
	case BIT(1):
		return HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_US;

#if defined(CONFIG_BT_CTLR_PHY_CODED)
	case BIT(2):
		if (flags & 0x01) {
			return HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_US;
		} else {
			return HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_US;
		}
#endif /* CONFIG_BT_CTLR_PHY_CODED */
	}
}

static inline uint32_t hal_radio_tx_chain_delay_us_get(uint8_t phy, uint8_t flags)
{
	switch (phy) {
	default:
	case BIT(0):
		return HAL_RADIO_NRF52820_TX_CHAIN_DELAY_1M_US;
	case BIT(1):
		return HAL_RADIO_NRF52820_TX_CHAIN_DELAY_2M_US;

#if defined(CONFIG_BT_CTLR_PHY_CODED)
	case BIT(2):
		if (flags & 0x01) {
			return HAL_RADIO_NRF52820_TX_CHAIN_DELAY_S8_US;
		} else {
			return HAL_RADIO_NRF52820_TX_CHAIN_DELAY_S2_US;
		}
#endif /* CONFIG_BT_CTLR_PHY_CODED */
	}
}

static inline uint32_t hal_radio_rx_chain_delay_us_get(uint8_t phy, uint8_t flags)
{
	switch (phy) {
	default:
	case BIT(0):
		return HAL_RADIO_NRF52820_RX_CHAIN_DELAY_1M_US;
	case BIT(1):
		return HAL_RADIO_NRF52820_RX_CHAIN_DELAY_2M_US;

#if defined(CONFIG_BT_CTLR_PHY_CODED)
	case BIT(2):
		if (flags & 0x01) {
			return HAL_RADIO_NRF52820_RX_CHAIN_DELAY_S8_US;
		} else {
			return HAL_RADIO_NRF52820_RX_CHAIN_DELAY_S2_US;
		}
#endif /* CONFIG_BT_CTLR_PHY_CODED */
	}
}

static inline uint32_t hal_radio_tx_ready_delay_ns_get(uint8_t phy, uint8_t flags)
{
	switch (phy) {
	default:
	case BIT(0):
		return HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_1M_NS;
	case BIT(1):
		return HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_2M_NS;

#if defined(CONFIG_BT_CTLR_PHY_CODED)
	case BIT(2):
		if (flags & 0x01) {
			return HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S8_NS;
		} else {
			return HAL_RADIO_NRF52820_TXEN_TXIDLE_TX_S2_NS;
		}
#endif /* CONFIG_BT_CTLR_PHY_CODED */
	}
}

static inline uint32_t hal_radio_rx_ready_delay_ns_get(uint8_t phy, uint8_t flags)
{
	switch (phy) {
	default:
	case BIT(0):
		return HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_1M_NS;
	case BIT(1):
		return HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_2M_NS;

#if defined(CONFIG_BT_CTLR_PHY_CODED)
	case BIT(2):
		if (flags & 0x01) {
			return HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S8_NS;
		} else {
			return HAL_RADIO_NRF52820_RXEN_RXIDLE_RX_S2_NS;
		}
#endif /* CONFIG_BT_CTLR_PHY_CODED */
	}
}

static inline uint32_t hal_radio_tx_chain_delay_ns_get(uint8_t phy, uint8_t flags)
{
	switch (phy) {
	default:
	case BIT(0):
		return HAL_RADIO_NRF52820_TX_CHAIN_DELAY_1M_NS;
	case BIT(1):
		return HAL_RADIO_NRF52820_TX_CHAIN_DELAY_2M_NS;

#if defined(CONFIG_BT_CTLR_PHY_CODED)
	case BIT(2):
		if (flags & 0x01) {
			return HAL_RADIO_NRF52820_TX_CHAIN_DELAY_S8_NS;
		} else {
			return HAL_RADIO_NRF52820_TX_CHAIN_DELAY_S2_NS;
		}
#endif /* CONFIG_BT_CTLR_PHY_CODED */
	}
}

static inline uint32_t hal_radio_rx_chain_delay_ns_get(uint8_t phy, uint8_t flags)
{
	switch (phy) {
	default:
	case BIT(0):
		return HAL_RADIO_NRF52820_RX_CHAIN_DELAY_1M_NS;
	case BIT(1):
		return HAL_RADIO_NRF52820_RX_CHAIN_DELAY_2M_NS;

#if defined(CONFIG_BT_CTLR_PHY_CODED)
	case BIT(2):
		if (flags & 0x01) {
			return HAL_RADIO_NRF52820_RX_CHAIN_DELAY_S8_NS;
		} else {
			return HAL_RADIO_NRF52820_RX_CHAIN_DELAY_S2_NS;
		}
#endif /* CONFIG_BT_CTLR_PHY_CODED */
	}
}
