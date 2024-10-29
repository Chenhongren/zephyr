/*
 * Copyright (c) 2024 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ITE_IT8XXX2_SOC_CLOCK_H_
#define _ITE_IT8XXX2_SOC_CLOCK_H_

#include <stdint.h>

struct it8xxx2_clk_cfg {
	uint8_t bus;
	uint8_t ctrl;
	uint8_t bit;
};

#endif /* _ITE_IT8XXX2_SOC_CLOCK_H_ */