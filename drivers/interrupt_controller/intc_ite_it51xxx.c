/*
 * Copyright (c) 2025 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/dt-bindings/interrupt-controller/ite-it51xxx-intc.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(intc_ite_it51xxx, LOG_LEVEL_DBG);

#define IT51XXX_INTC_NODE        DT_NODELABEL(intc)
#define II51XXX_INTC_GROUP_COUNT 26
#define MAX_REGISR_IRQ_NUM       8
#define IVECT_OFFSET_WITH_IRQ    0x10

const struct it51xxx_intc_config *intc_config;

struct it51xxx_intc_config {
	/* Pointer to intc registers. */
	struct intc_it51xxx_regs *base;
	uintptr_t isr_reg[II51XXX_INTC_GROUP_COUNT];
	uintptr_t ier_reg[II51XXX_INTC_GROUP_COUNT];
	uintptr_t ielmr_reg[II51XXX_INTC_GROUP_COUNT];
	uintptr_t ipolr_reg[II51XXX_INTC_GROUP_COUNT];
};

/* Interrupt number of INTC module */
static uint8_t intc_irq;
static uint8_t ier_setting[II51XXX_INTC_GROUP_COUNT];

void ite_intc_save_and_disable_interrupts(void)
{
	volatile uint8_t _ier __unused;
	/* Disable global interrupt for critical section */
	unsigned int key = irq_lock();

	/* Save and disable interrupts */
	for (int i = 0; i < II51XXX_INTC_GROUP_COUNT; i++) {
		ier_setting[i] = *((volatile uint8_t *)(intc_config->ier_reg[i]));
		*(volatile uint8_t *)(intc_config->ier_reg[i]) = 0;
	}
	/*
	 * This load operation will guarantee the above modification of
	 * SOC's register can be seen by any following instructions.
	 * Note: Barrier instruction can not synchronize chip register,
	 * so we introduce workaround here.
	 */
	_ier = *((volatile uint8_t *)(intc_config->ier_reg[II51XXX_INTC_GROUP_COUNT - 1]));

	irq_unlock(key);
}

void ite_intc_restore_interrupts(void)
{
	/*
	 * Ensure the highest priority interrupt will be the first fired
	 * interrupt when soc is ready to go.
	 */
	unsigned int key = irq_lock();

	/* Restore interrupt state */
	for (int i = 0; i < II51XXX_INTC_GROUP_COUNT; i++) {
		*((volatile uint8_t *)(intc_config->ier_reg[i])) = ier_setting[i];
	}

	irq_unlock(key);
}

void ite_intc_isr_clear(unsigned int irq)
{
	uint32_t g, i;

	if (irq > CONFIG_NUM_IRQS) {
		return;
	}
	g = irq / MAX_REGISR_IRQ_NUM;
	i = irq % MAX_REGISR_IRQ_NUM;

	sys_write8(BIT(i), intc_config->isr_reg[g]);
}

void ite_intc_irq_enable(unsigned int irq)
{
	uint32_t g, i;
	uint8_t en;

	if (irq > CONFIG_NUM_IRQS) {
		return;
	}
	g = irq / MAX_REGISR_IRQ_NUM;
	i = irq % MAX_REGISR_IRQ_NUM;

	/* critical section due to run a bit-wise OR operation */
	unsigned int key = irq_lock();

	en = sys_read8(intc_config->ier_reg[g]);
	sys_write8(en | BIT(i), intc_config->ier_reg[g]);

	irq_unlock(key);
}

void ite_intc_irq_disable(unsigned int irq)
{
	uint32_t g, i;
	uint8_t en;

	volatile uint8_t _ier __unused;

	if (irq > CONFIG_NUM_IRQS) {
		return;
	}
	g = irq / MAX_REGISR_IRQ_NUM;
	i = irq % MAX_REGISR_IRQ_NUM;

	/* critical section due to run a bit-wise OR operation */
	unsigned int key = irq_lock();

	en = sys_read8(intc_config->ier_reg[g]);
	sys_write8(en & ~BIT(i), intc_config->ier_reg[g]);
	/*
	 * This load operation will guarantee the above modification of
	 * SOC's register can be seen by any following instructions.
	 */
	_ier = sys_read8(intc_config->ier_reg[g]);

	irq_unlock(key);
}

void ite_intc_irq_polarity_set(unsigned int irq, unsigned int flags)
{
	uint32_t g, i;
	uint8_t tri;

	if ((irq > CONFIG_NUM_IRQS) || ((flags & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH)) {
		return;
	}
	g = irq / MAX_REGISR_IRQ_NUM;
	i = irq % MAX_REGISR_IRQ_NUM;

	tri = sys_read8(intc_config->ipolr_reg[g]);
	if ((flags & IRQ_TYPE_LEVEL_HIGH) || (flags & IRQ_TYPE_EDGE_RISING)) {
		sys_write8(tri & ~BIT(i), intc_config->ipolr_reg[g]);
	} else {
		sys_write8(tri | BIT(i), intc_config->ipolr_reg[g]);
	}

	tri = sys_read8(intc_config->ielmr_reg[g]);
	if ((flags & IRQ_TYPE_LEVEL_LOW) || (flags & IRQ_TYPE_LEVEL_HIGH)) {
		sys_write8(tri & ~BIT(i), intc_config->ielmr_reg[g]);
	} else {
		sys_write8(tri | BIT(i), intc_config->ielmr_reg[g]);
	}

	/* W/C interrupt status of the pin */
	sys_write8(BIT(i), intc_config->isr_reg[g]);
}

int ite_intc_irq_is_enable(unsigned int irq)
{
	uint32_t g, i;
	uint8_t en;

	if (irq > CONFIG_NUM_IRQS) {
		return 0;
	}
	g = irq / MAX_REGISR_IRQ_NUM;
	i = irq % MAX_REGISR_IRQ_NUM;

	en = sys_read8(intc_config->ier_reg[g]);

	return (en & BIT(i));
}

uint8_t ite_intc_get_irq_num(void)
{
	return intc_irq;
}

uint8_t get_irq(void *arg)
{
	ARG_UNUSED(arg);

	struct intc_it51xxx_regs *intc_regs = intc_config->base;

	/* wait until two equal interrupt values are read */
	do {
		/* Read interrupt number from interrupt vector register */
		intc_irq = intc_regs->INTC_VECT;
		/*
		 * WORKAROUND: when the interrupt vector register (INTC_VECT)
		 * isn't latched in a load operation, we read it again to make
		 * sure the value we got is the correct value.
		 */
	} while (intc_irq != intc_regs->INTC_VECT);
	/* determine interrupt number */
	intc_irq -= IVECT_OFFSET_WITH_IRQ;
	/* clear interrupt status */
	ite_intc_isr_clear(intc_irq);
	/* return interrupt number */
	return intc_irq;
}

static int it51xxx_intc_init(const struct device *dev)
{
	intc_config = dev->config;

	return 0;
}

static const struct it51xxx_intc_config it51xxx_intc_cfg = {
	.base = (struct intc_it51xxx_regs *)DT_REG_ADDR(IT51XXX_INTC_NODE),
	.isr_reg = DT_PROP(IT51XXX_INTC_NODE, isr_reg),
	.ier_reg = DT_PROP(IT51XXX_INTC_NODE, ier_reg),
	.ielmr_reg = DT_PROP(IT51XXX_INTC_NODE, ielmr_reg),
	.ipolr_reg = DT_PROP(IT51XXX_INTC_NODE, ipolr_reg),
};
DEVICE_DT_DEFINE(IT51XXX_INTC_NODE, &it51xxx_intc_init, NULL, NULL, &it51xxx_intc_cfg, PRE_KERNEL_1,
		 0, NULL);

void soc_interrupt_init(void)
{
	uintptr_t ier_reg[II51XXX_INTC_GROUP_COUNT] = DT_PROP(IT51XXX_INTC_NODE, ier_reg);

	/* Ensure interrupts of soc are disabled at default */
	for (int i = 0; i < II51XXX_INTC_GROUP_COUNT; i++) {
		*(volatile uint8_t *)ier_reg[i] = 0;
	}

	/* Enable M-mode external interrupt */
	csr_set(mie, MIP_MEIP);
}
