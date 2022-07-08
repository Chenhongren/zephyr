/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ESP32S2_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ESP32S2_H_

/* System Clock Source */
#define ESP32_CLK_SRC_XTAL             0U
#define ESP32_CLK_SRC_PLL              1U
#define ESP32_CLK_SRC_RTC8M            2U
#define ESP32_CLK_SRC_APLL             3U

/* Supported CPU Frequencies */
#define ESP32_CLK_CPU_26M              26U
#define ESP32_CLK_CPU_40M              40U
#define ESP32_CLK_CPU_80M              80U
#define ESP32_CLK_CPU_160M             160U
#define ESP32_CLK_CPU_240M             240U

/* Supported XTAL Frequencies */
#define ESP32_CLK_XTAL_40M             0U

/* Supported RTC fast clock frequencies */
#define ESP32_RTC_FAST_CLK_FREQ_8M     8500000U
#define ESP32_RTC_FAST_CLK_FREQ_APPROX ESP32_RTC_FAST_CLK_FREQ_8M

/* Supported RTC slow clock frequencies */
#define ESP32_RTC_SLOW_CLK_FREQ_90K    90000U
#define ESP32_RTC_SLOW_CLK_FREQ_8MD256 (ESP32_RTC_FAST_CLK_FREQ_APPROX / 256)
#define ESP32_RTC_SLOW_CLK_FREQ_32K    32768U

/* Modules IDs
 * These IDs are actually offsets in CLK and RST Control registers.
 * These IDs shouldn't be changed unless there is a Hardware change
 * from Espressif.
 *
 * Basic Modules
 * Registers: DPORT_PERIP_CLK_EN_REG, DPORT_PERIP_RST_EN_REG
 */
#define ESP32_LEDC_MODULE              0
#define ESP32_UART0_MODULE             1
#define ESP32_UART1_MODULE             2
#define ESP32_USB_MODULE               3
#define ESP32_I2C0_MODULE              4
#define ESP32_I2C1_MODULE              5
#define ESP32_I2S0_MODULE              6
#define ESP32_I2S1_MODULE              7
#define ESP32_TIMG0_MODULE             8
#define ESP32_TIMG1_MODULE             9
#define ESP32_PWM0_MODULE              10
#define ESP32_PWM1_MODULE              11
#define ESP32_PWM2_MODULE              12
#define ESP32_PWM3_MODULE              13
#define ESP32_UHCI0_MODULE             14
#define ESP32_UHCI1_MODULE             15
#define ESP32_RMT_MODULE               16
#define ESP32_PCNT_MODULE              17
#define ESP32_SPI_MODULE               18
#define ESP32_FSPI_MODULE              19
#define ESP32_HSPI_MODULE              20
#define ESP32_SPI2_DMA_MODULE          21
#define ESP32_SPI3_DMA_MODULE          22
#define ESP32_TWAI_MODULE              23
#define ESP32_RNG_MODULE               24
#define ESP32_WIFI_MODULE              25
#define ESP32_WIFI_BT_COMMON_MODULE    26
#define ESP32_SYSTIMER_MODULE          27
#define ESP32_AES_MODULE               28
#define ESP32_SHA_MODULE               29
#define ESP32_RSA_MODULE               30
#define ESP32_CRYPTO_DMA_MODULE        31
#define ESP32_AES_DMA_MODULE           32
#define ESP32_SHA_DMA_MODULE           33
#define ESP32_DEDIC_GPIO_MODULE        34
#define ESP32_MODULE_MAX               35

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ESP32S2_H_ */
