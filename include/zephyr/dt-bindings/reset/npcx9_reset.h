/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_RESET_NPCX9_RESET_H
#define ZEPHYR_INCLUDE_DT_BINDINGS_RESET_NPCX9_RESET_H

#define NPCX_RESET_SWRST_CTL1_OFFSET 0
#define NPCX_RESET_SWRST_CTL2_OFFSET 32
#define NPCX_RESET_SWRST_CTL3_OFFSET 64
#define NPCX_RESET_SWRST_CTL4_OFFSET 96

#define NPCX_RESET_GPIO0    (NPCX_RESET_SWRST_CTL1_OFFSET + 0)
#define NPCX_RESET_GPIO1    (NPCX_RESET_SWRST_CTL1_OFFSET + 1)
#define NPCX_RESET_GPIO2    (NPCX_RESET_SWRST_CTL1_OFFSET + 2)
#define NPCX_RESET_GPIO3    (NPCX_RESET_SWRST_CTL1_OFFSET + 3)
#define NPCX_RESET_GPIO4    (NPCX_RESET_SWRST_CTL1_OFFSET + 4)
#define NPCX_RESET_GPIO5    (NPCX_RESET_SWRST_CTL1_OFFSET + 5)
#define NPCX_RESET_GPIO6    (NPCX_RESET_SWRST_CTL1_OFFSET + 6)
#define NPCX_RESET_GPIO7    (NPCX_RESET_SWRST_CTL1_OFFSET + 7)
#define NPCX_RESET_GPIO8    (NPCX_RESET_SWRST_CTL1_OFFSET + 8)
#define NPCX_RESET_GPIO9    (NPCX_RESET_SWRST_CTL1_OFFSET + 9)
#define NPCX_RESET_GPIOA    (NPCX_RESET_SWRST_CTL1_OFFSET + 10)
#define NPCX_RESET_GPIOB    (NPCX_RESET_SWRST_CTL1_OFFSET + 11)
#define NPCX_RESET_GPIOC    (NPCX_RESET_SWRST_CTL1_OFFSET + 12)
#define NPCX_RESET_GPIOD    (NPCX_RESET_SWRST_CTL1_OFFSET + 13)
#define NPCX_RESET_GPIOE    (NPCX_RESET_SWRST_CTL1_OFFSET + 14)
#define NPCX_RESET_GPIOF    (NPCX_RESET_SWRST_CTL1_OFFSET + 15)
#define NPCX_RESET_ITIM64   (NPCX_RESET_SWRST_CTL1_OFFSET + 16)
#define NPCX_RESET_ITIM32_1 (NPCX_RESET_SWRST_CTL1_OFFSET + 18)
#define NPCX_RESET_ITIM32_2 (NPCX_RESET_SWRST_CTL1_OFFSET + 19)
#define NPCX_RESET_ITIM32_3 (NPCX_RESET_SWRST_CTL1_OFFSET + 20)
#define NPCX_RESET_ITIM32_4 (NPCX_RESET_SWRST_CTL1_OFFSET + 21)
#define NPCX_RESET_ITIM32_5 (NPCX_RESET_SWRST_CTL1_OFFSET + 22)
#define NPCX_RESET_ITIM32_6 (NPCX_RESET_SWRST_CTL1_OFFSET + 23)
#define NPCX_RESET_MTC      (NPCX_RESET_SWRST_CTL1_OFFSET + 25)
#define NPCX_RESET_MIWU0    (NPCX_RESET_SWRST_CTL1_OFFSET + 26)
#define NPCX_RESET_MIWU1    (NPCX_RESET_SWRST_CTL1_OFFSET + 27)
#define NPCX_RESET_MIWU2    (NPCX_RESET_SWRST_CTL1_OFFSET + 28)
#define NPCX_RESET_GDMA     (NPCX_RESET_SWRST_CTL1_OFFSET + 29)
#define NPCX_RESET_FIU      (NPCX_RESET_SWRST_CTL1_OFFSET + 30)

#define NPCX_RESET_PMC     (NPCX_RESET_SWRST_CTL2_OFFSET + 0)
#define NPCX_RESET_SHI     (NPCX_RESET_SWRST_CTL2_OFFSET + 2)
#define NPCX_RESET_SPIP    (NPCX_RESET_SWRST_CTL2_OFFSET + 3)
#define NPCX_RESET_PECI    (NPCX_RESET_SWRST_CTL2_OFFSET + 5)
#define NPCX_RESET_CRUART2 (NPCX_RESET_SWRST_CTL2_OFFSET + 6)
#define NPCX_RESET_ADC     (NPCX_RESET_SWRST_CTL2_OFFSET + 7)
#define NPCX_RESET_SMB0    (NPCX_RESET_SWRST_CTL2_OFFSET + 8)
#define NPCX_RESET_SMB1    (NPCX_RESET_SWRST_CTL2_OFFSET + 9)
#define NPCX_RESET_SMB2    (NPCX_RESET_SWRST_CTL2_OFFSET + 10)
#define NPCX_RESET_SMB3    (NPCX_RESET_SWRST_CTL2_OFFSET + 11)
#define NPCX_RESET_SMB4    (NPCX_RESET_SWRST_CTL2_OFFSET + 12)
#define NPCX_RESET_SMB5    (NPCX_RESET_SWRST_CTL2_OFFSET + 13)
#define NPCX_RESET_SMB6    (NPCX_RESET_SWRST_CTL2_OFFSET + 14)
#define NPCX_RESET_TWD     (NPCX_RESET_SWRST_CTL2_OFFSET + 15)
#define NPCX_RESET_PWM0    (NPCX_RESET_SWRST_CTL2_OFFSET + 16)
#define NPCX_RESET_PWM1    (NPCX_RESET_SWRST_CTL2_OFFSET + 17)
#define NPCX_RESET_PWM2    (NPCX_RESET_SWRST_CTL2_OFFSET + 18)
#define NPCX_RESET_PWM3    (NPCX_RESET_SWRST_CTL2_OFFSET + 19)
#define NPCX_RESET_PWM4    (NPCX_RESET_SWRST_CTL2_OFFSET + 20)
#define NPCX_RESET_PWM5    (NPCX_RESET_SWRST_CTL2_OFFSET + 21)
#define NPCX_RESET_PWM6    (NPCX_RESET_SWRST_CTL2_OFFSET + 22)
#define NPCX_RESET_PWM7    (NPCX_RESET_SWRST_CTL2_OFFSET + 23)
#define NPCX_RESET_MFT16_1 (NPCX_RESET_SWRST_CTL2_OFFSET + 24)
#define NPCX_RESET_MFT16_2 (NPCX_RESET_SWRST_CTL2_OFFSET + 25)
#define NPCX_RESET_MFT16_3 (NPCX_RESET_SWRST_CTL2_OFFSET + 26)
#define NPCX_RESET_SMB7    (NPCX_RESET_SWRST_CTL2_OFFSET + 27)
#define NPCX_RESET_CRUART1 (NPCX_RESET_SWRST_CTL2_OFFSET + 28)
#define NPCX_RESET_PS2     (NPCX_RESET_SWRST_CTL2_OFFSET + 29)
#define NPCX_RESET_SDP     (NPCX_RESET_SWRST_CTL2_OFFSET + 30)
#define NPCX_RESET_KBS     (NPCX_RESET_SWRST_CTL2_OFFSET + 31)

#define NPCX_RESET_SIOCFG  (NPCX_RESET_SWRST_CTL3_OFFSET + 0)
#define NPCX_RESET_SERPORT (NPCX_RESET_SWRST_CTL3_OFFSET + 1)
#define NPCX_RESET_I3C     (NPCX_RESET_SWRST_CTL3_OFFSET + 5)
#define NPCX_RESET_MSWC    (NPCX_RESET_SWRST_CTL3_OFFSET + 8)
#define NPCX_RESET_SHM     (NPCX_RESET_SWRST_CTL3_OFFSET + 9)
#define NPCX_RESET_PMCH1   (NPCX_RESET_SWRST_CTL3_OFFSET + 10)
#define NPCX_RESET_PMCH2   (NPCX_RESET_SWRST_CTL3_OFFSET + 11)
#define NPCX_RESET_PMCH3   (NPCX_RESET_SWRST_CTL3_OFFSET + 12)
#define NPCX_RESET_PMCH4   (NPCX_RESET_SWRST_CTL3_OFFSET + 13)
#define NPCX_RESET_KBC     (NPCX_RESET_SWRST_CTL3_OFFSET + 15)
#define NPCX_RESET_C2HOST  (NPCX_RESET_SWRST_CTL3_OFFSET + 16)
#define NPCX_RESET_CRUART3 (NPCX_RESET_SWRST_CTL3_OFFSET + 18)
#define NPCX_RESET_CRUART4 (NPCX_RESET_SWRST_CTL3_OFFSET + 19)
#define NPCX_RESET_LFCG    (NPCX_RESET_SWRST_CTL3_OFFSET + 20)
#define NPCX_RESET_DEV     (NPCX_RESET_SWRST_CTL3_OFFSET + 22)
#define NPCX_RESET_SYSCFG  (NPCX_RESET_SWRST_CTL3_OFFSET + 23)
#define NPCX_RESET_SBY     (NPCX_RESET_SWRST_CTL3_OFFSET + 24)
#define NPCX_RESET_BBRAM   (NPCX_RESET_SWRST_CTL3_OFFSET + 25)
#define NPCX_RESET_SHA     (NPCX_RESET_SWRST_CTL3_OFFSET + 29)

#define NPCX_RESET_MDMA1 (NPCX_RESET_SWRST_CTL4_OFFSET + 24)
#define NPCX_RESET_MDMA2 (NPCX_RESET_SWRST_CTL4_OFFSET + 25)
#define NPCX_RESET_MDMA3 (NPCX_RESET_SWRST_CTL4_OFFSET + 26)
#define NPCX_RESET_MDMA4 (NPCX_RESET_SWRST_CTL4_OFFSET + 27)
#define NPCX_RESET_MDMA5 (NPCX_RESET_SWRST_CTL4_OFFSET + 28)

#define NPCX_RESET_ID_START NPCX_RESET_GPIO0
#define NPCX_RESET_ID_END   NPCX_RESET_MDMA5
#endif
