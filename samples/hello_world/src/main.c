/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/drivers/interrupt_controller/wuc_ite_it8xxx2.h>
#include <zephyr/dt-bindings/interrupt-controller/it8xxx2-wuc.h>
#include <zephyr/irq.h>
#include <zephyr/dt-bindings/interrupt-controller/ite-it51xxx-intc.h>

static void it51xxx_wu176_isr(const void *arg)
{
	printf("wu176 is triggered\n");
	sys_write8(BIT(0), 0xf01b4d);
}

static void it51xxx_wu177_isr(const void *arg)
{
	printf("wu177 is triggered\n");
	sys_write8(BIT(1), 0xf01b4d);
}

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	/* Connect WU90 (USB D+) interrupt but make it disabled initially */
	irq_connect_dynamic(IT51XXX_IRQ_WU176, 0, it51xxx_wu176_isr, 0, 0);
	irq_connect_dynamic(IT51XXX_IRQ_WU177, 0, it51xxx_wu177_isr, 0, 0);

	/* write-1-clear wake-up interrupt status */
	sys_write8(BIT(1) | BIT(0), 0xf01b4d);

	/* enable usb device in gpiof4/gpiof5 */
	sys_write8(BIT(1), 0xf016cf);

	/* enable usb pull down */
	sys_write8(BIT(5), 0xf016c2);

	irq_enable(IT51XXX_IRQ_WU176);
	irq_enable(IT51XXX_IRQ_WU177);

	return 0;
}
