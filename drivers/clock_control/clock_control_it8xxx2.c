/*
 * Copyright (c) 2024 ITE Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it8xxx2_ecpm

#include "soc_clock.h"

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_it8xxx2);

#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/it8xxx2_clock.h>

#define ECPM02_CLOCK_GATING_CTRL_2 0x02
#define SWUC_CLOCK_GATING          BIT(4)

#define ECPM03_PLL_CTRL     0x03
#define DEEP_DOZE_MODE_CTRL BIT(1)
#define PLL_POWER_DOWN_CTRL BIT(0)

#define ECPM04_AUTO_CLOCK_GATING 0x04
#define AUTO_UART1_CLOCK_GATING  BIT(6)
#define AUTO_UART2_CLOCK_GATING  BIT(5)
#define AUTO_SSPI_CLOCK_GATING   BIT(4)

#define ECPM05_CLOCK_GATING_CTRL_3 0x05
#define PECI_CLOCK_GATING          BIT(4)
#define UART_CLOCK_GATING          BIT(3)
#define SSPI_CLOCK_GATING          BIT(2)
#define DBGR_CLOCK_GATING          BIT(1)

#define ECPM06_PLL_FREQ 0x06
#define PLL_FREQ_MASK   GENMASK(3, 0)

#define ECPM08_PLL_CLOCK_SOURCE_STATUS                  0x08
#define PLL_CLOCK_SOURCE                                BIT(7)
#define INTERNAL_TO_EXTERNAL_CLOCK_SW_REQ_IS_POSTPONING BIT(7)

#define ECPM09_CLOCK_GATING_CTRL_4 0x09
#define SMB_CH_C_CLOCK_GATING      BIT(4)
#define SMB_CH_B_CLOCK_GATING      BIT(3)
#define SMB_CH_A_CLOCK_GATING      BIT(2)
#define SMB_CLOCK_GATING           BIT(1)
#define SMB_CEC_CLOCK_GATING       BIT(0)

#define ECPM0C_SYSTEM_CLOCK_DIVIDE_CTRL_0 0x0C
#define FND_CLOCK_FREQ_SELECT             GENMASK(6, 4)
#define MCU_CLOCK_FREQ_SELECT             GENMASK(2, 0)

#define ECPM0D_SYSTEM_CLOCK_DIVIDE_CTRL_1 0x0D
#define UART_CLOCK_FREQ_SELECT            GENMASK(3, 0)

#define ECPM0E_SYSTEM_CLOCK_DIVIDE_CTRL_2 0x0E
#define SSPI_CLOCK_FREQ_SELECT_MASK       GENMASK(7, 4)
#define SMB_CLOCK_FREQ_SELECT_MASK        GENMASK(3, 0)

#define ECPM0F_SYSTEM_CLOCK_DIVIDE_CTRL_3 0x0F
#define JTAG_CLOCK_FREQ_SELECT            GENMASK(7, 4)
#define EC_CLOCK_FREQ_SELECT              GENMASK(3, 0)

#define ECPM10_SYSTEM_CLOCK_DIVIDE_CTRL_4 0x10
#define USBPD_CLOCK_FREQ_SELECT           GENMASK(7, 4)
#define PWM_CLOCK_FREQ_SELECT             GENMASK(3, 0)

#define ECPM13_CLOCK_GATING_CTRL_5 0x13
#define PD2_CLOCK_GATING           BIT(6)
#define SPI_SLAVE_CLOCK_GATING     BIT(3)
#define PD1_CLOCK_GATING           BIT(1)
#define PD0_CLOCK_GATING           BIT(0)

#define ECPM15_CLOCK_GATING_CTRL_6 0x15
#define RISCV_JTAG_CLOCK_GATING    BIT(3)
#define RISCV_FPU_CLOCK_GATING     BIT(2)

#define ECPM20_PLL_FREQ_AUTO_CAL_CTRL_0 0x20
#define PLL_FREQ_AUTO_CAL_EN            BIT(7)
#define LOCK_TUNING_FACTORS_OF_LCO      BIT(3)

#define ECPM21_PLL_FREQ_AUTO_CAL_CTRL_1 0x21
#define ECPM40_PLL_FREQ_AUTO_CAL_CTRL_2 0x40
#define AUTO_CAL_ENABLE                 BIT(1)
#define PLL_FREQ_AUTO_CAL_START         BIT(0)
#define AUTO_CAL_ENABLE_AND_START       (AUTO_CAL_ENABLE | PLL_FREQ_AUTO_CAL_START)

#define ECPM54_LC_OSCILLATOR_TUNING_FACTOR_2 0x54
#define LCO_SC_FACTOR_MASK                   GENMASK(6, 4)
#define LCO_SC_FACTOR(n)                     FIELD_PREP(LCO_SC_FACTOR_MASK, n)

#define ECPM55_LC_OSCILLATOR_CTRL 0x55
#define LCO_Power_CTRL            BIT(1)

#define ECPM57_LC_OSCILLATOR_CTRL_1 0x57
#define LDO_Power_CTRL              BIT(1)

struct clock_it8xxx2_config {
	mm_reg_t base;
};

static inline uint32_t it8xxx2_get_pll_freq(const struct device *dev)
{
	const struct clock_it8xxx2_config *cfg = dev->config;
	const uint32_t pll_frequency_table[8] = {MHZ(8),  MHZ(16), MHZ(24), MHZ(32),
						 MHZ(48), MHZ(64), MHZ(72), MHZ(96)};
	uint8_t id;

	id = sys_read8(cfg->base + ECPM06_PLL_FREQ) & PLL_FREQ_MASK;
	if (id > ARRAY_SIZE(pll_frequency_table)) {
		return 0;
	}

	return pll_frequency_table[id];
}

static int it8xxx2_clock_control_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	const mm_reg_t ecpm_base = ((const struct clock_it8xxx2_config *)dev->config)->base;
	struct it8xxx2_clk_cfg *clk_cfg = (struct it8xxx2_clk_cfg *)(sub_system);
	uint8_t reg_val;

	reg_val = sys_read8(ecpm_base + clk_cfg->ctrl);
	sys_write8(reg_val & ~BIT(clk_cfg->bit), ecpm_base + clk_cfg->ctrl);
	return 0;
}

static int it8xxx2_clock_control_off(const struct device *dev, clock_control_subsys_t sub_system)
{
	const mm_reg_t ecpm_base = ((const struct clock_it8xxx2_config *)dev->config)->base;
	struct it8xxx2_clk_cfg *clk_cfg = (struct it8xxx2_clk_cfg *)(sub_system);
	uint8_t reg_val;

	reg_val = sys_read8(ecpm_base + clk_cfg->ctrl);
	sys_write8(reg_val | BIT(clk_cfg->bit), ecpm_base + clk_cfg->ctrl);
	return 0;
}

static int it8xxx2_clock_control_get_rate(const struct device *dev,
					  clock_control_subsys_t sub_system, uint32_t *rate)
{
	const mm_reg_t ecpm_base = ((const struct clock_it8xxx2_config *)dev->config)->base;
	struct it8xxx2_clk_cfg *clk_cfg = (struct it8xxx2_clk_cfg *)(sub_system);
	uint32_t pll_freq, clk_div;
	int ret = 0;

	if (!rate) {
		return -EINVAL;
	}

	pll_freq = it8xxx2_get_pll_freq(dev);
	if (pll_freq == 0) {
		LOG_ERR("Failed to get pll frequency");
		return -ENOTSUP;
	}

	switch (clk_cfg->bus) {
	case IT8XXX2_SSPI_MODULE:
		clk_div = sys_read8(ecpm_base + ECPM0E_SYSTEM_CLOCK_DIVIDE_CTRL_2) &
			  SSPI_CLOCK_FREQ_SELECT_MASK;
		clk_div = (clk_div >> 4) + 1;
		*rate = pll_freq / clk_div;
		break;
	case IT8XXX2_SMB_MODULE:
		clk_div = sys_read8(ecpm_base + ECPM0E_SYSTEM_CLOCK_DIVIDE_CTRL_2) &
			  SMB_CLOCK_FREQ_SELECT_MASK;
		clk_div += 1;
		*rate = pll_freq / clk_div;
		break;
	default:
		LOG_ERR("Unknown clock bus %d", clk_cfg->bus);
		ret = -ENOTSUP;
		break;
	}
	return ret;
}

static int it8xxx2_clock_control_set_rate(const struct device *dev,
					  clock_control_subsys_t sub_system,
					  clock_control_subsys_rate_t rate)
{
	const mm_reg_t ecpm_base = ((const struct clock_it8xxx2_config *)dev->config)->base;
	struct it8xxx2_clk_cfg *clk_cfg = (struct it8xxx2_clk_cfg *)(sub_system);
	uint32_t pll_freq;
	uint8_t reg_val;

	pll_freq = it8xxx2_get_pll_freq(dev);
	if (pll_freq == 0) {
		LOG_ERR("Failed to get pll frequency");
		return -ENOTSUP;
	}

	switch (clk_cfg->bus) {
	case IT8XXX2_SSPI_MODULE:
		for (int i = 1; i <= 16; ++i) {
			if (pll_freq / i == (uintptr_t)rate) {
				reg_val = sys_read8(ecpm_base + ECPM0E_SYSTEM_CLOCK_DIVIDE_CTRL_2) &
					  ~SSPI_CLOCK_FREQ_SELECT_MASK;
				reg_val |= FIELD_PREP(SSPI_CLOCK_FREQ_SELECT_MASK, i - 1);
				sys_write8(reg_val, ecpm_base + ECPM0E_SYSTEM_CLOCK_DIVIDE_CTRL_2);
				return 0;
			}
		}
		return -ENOTSUP;
	default:
		LOG_ERR("Unknown clock bus %d", clk_cfg->bus);
		return -ENOTSUP;
	}
}

static struct clock_control_driver_api it8xxx2_clk_api = {
	.on = it8xxx2_clock_control_on,
	.off = it8xxx2_clock_control_off,
	.get_rate = it8xxx2_clock_control_get_rate,
	.set_rate = it8xxx2_clock_control_set_rate,
};

#define IT8XXX2_CLOCK_INIT(n)                                                                      \
	static const struct clock_it8xxx2_config it8xxx2_clk_cfg_##n = {                           \
		.base = DT_INST_REG_ADDR(n),                                                       \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, &it8xxx2_clk_cfg_##n, PRE_KERNEL_1,             \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &it8xxx2_clk_api);

// TODO: check the CONFIG_CLOCK_CONTROL_INIT_PRIORITY and CONFIG_IT8XXX2_PLL_SEQUENCE_PRIORITY

DT_INST_FOREACH_STATUS_OKAY(IT8XXX2_CLOCK_INIT)
