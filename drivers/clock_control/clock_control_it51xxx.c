/*
 * Copyright (c) 2025 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it51xxx_ecpm

#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/ite-it51xxx-clock.h>
#include <zephyr/dt-bindings/interrupt-controller/ite-it51xxx-intc.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(clock_control_it51xxx, LOG_LEVEL_ERR);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "only one ite,it51xxx-ecpm compatible node can be supported");

#define PLLFREQ_MASK GENMASK(3, 0)

static const uint8_t pll_cfg[] = {
	[PLL_18400_KHZ] = 0x01,
	[PLL_32300_KHZ] = 0x03,
	[PLL_64500_KHZ] = 0x07,
	[PLL_48000_KHZ] = 0x09,
};

struct clock_control_it51xxx_data {
	const uint8_t *pll_configuration;
};

/* Driver config */
struct clock_control_it51xxx_config {
	struct ecpm_it51xxx_regs *reg_base;
	int pll_freq;
};

/* Clock controller local functions */
static inline int clock_control_it51xxx_on(const struct device *dev,
					   clock_control_subsys_t sub_system)
{
	const struct clock_control_it51xxx_config *const config = dev->config;
	struct ite_clk_cfg *clk_cfg = (struct ite_clk_cfg *)(sub_system);
	volatile uint8_t *crtl_reg = (uint8_t *)config->reg_base + clk_cfg->ctrl;
	uint8_t reg_mask = clk_cfg->bits;

	/* Enable the clock of this module */
	*crtl_reg &= ~reg_mask;

	return 0;
}

static inline int clock_control_it51xxx_off(const struct device *dev,
					    clock_control_subsys_t sub_system)
{
	const struct clock_control_it51xxx_config *const config = dev->config;
	struct ite_clk_cfg *clk_cfg = (struct ite_clk_cfg *)(sub_system);
	volatile uint8_t *crtl_reg = (uint8_t *)config->reg_base + clk_cfg->ctrl;
	uint8_t reg_mask = clk_cfg->bits;
	uint8_t tmp_mask = 0;

	/* CGCTRL3R, bit 6, must always write a 1. */
	tmp_mask |= (clk_cfg->ctrl == IT51XXX_ECPM_CGCTRL3R_OFF) ? 0x40 : 0x00;
	*crtl_reg |= reg_mask | tmp_mask;

	return 0;
}

static int clock_control_it51xxx_get_rate(const struct device *dev,
					  clock_control_subsys_t sub_system, uint32_t *rate)
{
	const struct clock_control_it51xxx_config *const config = dev->config;
	struct ecpm_it51xxx_regs *const ecpm_regs = config->reg_base;
	int reg_val = ecpm_regs->PLLFREQR & PLLFREQ_MASK;

	switch (reg_val) {
	case 0x01:
		*rate = KHZ(18400);
		break;
	case 0x03:
		*rate = KHZ(32300);
		break;
	case 0x07:
		*rate = KHZ(64500);
		break;
	case 0x09:
		*rate = KHZ(48000);
		break;
	default:
		return -ERANGE;
	}

	return 0;
}

static void pll_change_isr(const void *unused)
{
	ARG_UNUSED(unused);

	/*
	 * We are here because we have completed changing PLL sequence,
	 * so disabled PLL frequency change event interrupt.
	 */
	irq_disable(IT51XXX_IRQ_PLL_CHANGE);
}

static void chip_configure_pll(const struct device *dev, uint8_t pll)
{
	const struct clock_control_it51xxx_config *config = dev->config;
	struct ecpm_it51xxx_regs *const ecpm_regs = config->reg_base;

	/* Set pll frequency change event */
	IRQ_CONNECT(IT51XXX_IRQ_PLL_CHANGE, 0, pll_change_isr, NULL, IRQ_TYPE_EDGE_RISING);
	/* Clear interrupt status of pll frequency change event */
	ite_intc_isr_clear(IT51XXX_IRQ_PLL_CHANGE);

	irq_enable(IT51XXX_IRQ_PLL_CHANGE);
	/*
	 * Configure PLL clock dividers.
	 * Writing data to these registers doesn't change the PLL frequency immediately until the
	 * status is changed into wakeup from the sleep mode.
	 * The following code is intended to make the system enter sleep mode, and wait PLL
	 * frequency change event to wakeup chip to complete PLL update.
	 */
	ecpm_regs->PLLFREQR = pll;
	/* Chip sleep after wait for interrupt (wfi) instruction */
	chip_pll_ctrl(CHIP_PLL_SLEEP);
	/* Chip sleep and wait timer wake it up */
	__asm__ volatile("wfi");
	/* Chip sleep and wait timer wake it up */
	chip_pll_ctrl(CHIP_PLL_DOZE);
}

static int clock_control_it51xxx_init(const struct device *dev)
{
	const struct clock_control_it51xxx_config *config = dev->config;
	struct ecpm_it51xxx_regs *const ecpm_regs = config->reg_base;
	struct clock_control_it51xxx_data *data = dev->data;
	int reg_val = ecpm_regs->PLLFREQR & PLLFREQ_MASK;

	/* Disable auto gating and enable it by the respective module. */
	ecpm_regs->AUTOCG &= ~(IT51XXX_ECPM_AUART1CG | IT51XXX_ECPM_AUART2CG |
			       IT51XXX_ECPM_ASSPICG | IT51XXX_ECPM_ACIRCG);
	/* The following modules are gated in the initial state */
	ecpm_regs->CGCTRL2R = IT51XXX_ECPM_CIRCG | IT51XXX_ECPM_SWUCCG;
	ecpm_regs->CGCTRL3R |= IT51XXX_ECPM_PECICG | IT51XXX_ECPM_SSPICG;

	if (IS_ENABLED(CONFIG_ITE_IT51XXX_INTC)) {
		ite_intc_save_and_disable_interrupts();
	}

	if (reg_val != data->pll_configuration[config->pll_freq]) {
		/* configure PLL clock */
		chip_configure_pll(dev, data->pll_configuration[config->pll_freq]);
	}

	if (IS_ENABLED(CONFIG_ITE_IT51XXX_INTC)) {
		ite_intc_restore_interrupts();
	}

	return 0;
}

/* Clock controller driver registration */
static DEVICE_API(clock_control, clock_control_it51xxx_api) = {
	.on = clock_control_it51xxx_on,
	.off = clock_control_it51xxx_off,
	.get_rate = clock_control_it51xxx_get_rate,
};

static struct clock_control_it51xxx_data clock_control_it51xxx_data = {
	.pll_configuration = pll_cfg,
};

static const struct clock_control_it51xxx_config clock_control_it51xxx_cfg = {
	.reg_base = (struct ecpm_it51xxx_regs *)DT_INST_REG_ADDR(0),
	.pll_freq = DT_INST_PROP(0, pll_frequency),
};

DEVICE_DT_INST_DEFINE(0, clock_control_it51xxx_init, NULL, &clock_control_it51xxx_data,
		      &clock_control_it51xxx_cfg, PRE_KERNEL_1,
		      CONFIG_IT51XXX_PLL_SEQUENCE_PRIORITY, &clock_control_it51xxx_api);
