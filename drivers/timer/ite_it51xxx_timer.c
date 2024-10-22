/*
 * Copyright (c) 2025 ITE Corporation. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it51xxx_timer

#include <soc.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>

LOG_MODULE_REGISTER(timer, LOG_LEVEL_ERR);

BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == 32768,
	     "Hardware timer frequency is fixed at 32768Hz");

BUILD_ASSERT(!IS_ENABLED(CONFIG_TICKLESS_KERNEL),
	     "Tickless mode is not supported on it513xx series");

/* Event timer configurations */
#define EVENT_TIMER      EXT_TIMER_2
#define EVENT_TIMER_IRQ  DT_INST_IRQ_BY_IDX(0, 0, irq)
#define EVENT_TIMER_FLAG DT_INST_IRQ_BY_IDX(0, 0, flags)

#define MS_TO_COUNT(hz, ms) ((hz) * (ms) / 1000)
#define ETPSR_9200K         KHZ(9200)
#define ETPSR_32768         32768
#define ETPSR_1024          1024
#define ETPSR_32            32
#define CLOCK_65K           (65536 / 1000)
/*
 * One system (kernel) tick is as how much HW timer counts
 */
#define HW_CNT_PER_SYS_TICK (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

static uint32_t system_ticks;

enum ext_timer_raw_cnt {
	EXT_NOT_RAW_CNT = 0,
	EXT_RAW_CNT,
};

enum ext_timer_int {
	EXT_WITHOUT_TIMER_INT = 0,
	EXT_WITH_TIMER_INT,
};

static void evt_timer_isr(const void *unused)
{
	struct timer_it51xxx_regs *const timer_regs = IT51XXX_EXT_TIMER_BASE;

	ARG_UNUSED(unused);

	/* W/C event timer interrupt status */
	ite_intc_isr_clear(EVENT_TIMER_IRQ);
	/* Re-start event timer */
	timer_regs->ETWCTRL |= IT51XXX_ETWD_ET2RST;

	system_ticks += MS_TO_COUNT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC, 1);
	/* Informs kernel that one system tick has elapsed */
	sys_clock_announce(MS_TO_COUNT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC, 1));
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(ticks);
	ARG_UNUSED(idle);
}

uint32_t sys_clock_elapsed(void)
{
	return 0;
}

uint32_t sys_clock_cycle_get_32(void)
{
	return system_ticks;
}

static int timer_init(enum ext_timer_idx ext_timer, enum ext_clk_src_sel clock_source_sel,
		      enum ext_timer_raw_cnt raw, uint32_t ms, uint32_t irq_num, uint32_t irq_flag,
		      enum ext_timer_int with_int)
{
	struct timer_it51xxx_regs *const timer_regs = IT51XXX_EXT_TIMER_BASE;
	uint32_t hw_cnt;

	if (raw == EXT_RAW_CNT) {
		hw_cnt = ms;
	} else {
		if (clock_source_sel == EXT_PSR_32P768K) {
			hw_cnt = MS_TO_COUNT(ETPSR_32768, ms);
		} else if (clock_source_sel == EXT_PSR_1P024K) {
			hw_cnt = MS_TO_COUNT(ETPSR_1024, ms);
		} else if (clock_source_sel == EXT_PSR_32) {
			hw_cnt = MS_TO_COUNT(ETPSR_32, ms);
		} else if (clock_source_sel == EXT_PSR_EC_CLK) {
			hw_cnt = MS_TO_COUNT(ETPSR_9200K, ms);
		} else {
			LOG_ERR("Timer %d clock source error !", ext_timer);
			return -EINVAL;
		}
	}

	if (hw_cnt == 0) {
		LOG_ERR("Timer %d count shouldn't be 0 !", ext_timer);
		return -EINVAL;
	}

	/* Set rising edge triggered of external timer x */
	ite_intc_irq_polarity_set(irq_num, irq_flag);

	/* Clear interrupt status of external timer x */
	ite_intc_isr_clear(irq_num);

	if (ext_timer == EVENT_TIMER) {
		/* Set clock source of external event timer */
		timer_regs->ET2PSR = clock_source_sel;
		/* Set count of external event timer */
		timer_regs->ET2CNTLH2R = (hw_cnt >> 16) & 0xff;
		timer_regs->ET2CNTLHR = (hw_cnt >> 8) & 0xff;
		timer_regs->ET2CNTLLR = hw_cnt & 0xff;
	}

	if (with_int == EXT_WITH_TIMER_INT) {
		irq_enable(irq_num);
	} else {
		irq_disable(irq_num);
	}

	return 0;
}

#ifdef CONFIG_ARCH_HAS_CUSTOM_BUSY_WAIT
void arch_busy_wait(uint32_t usec_to_wait)
{
	struct gctrl_it51xxx_regs *const gctrl_regs = GCTRL_IT51XXX_REGS_BASE;
	uint32_t hw_cnt;

	if (!usec_to_wait) {
		return;
	}

	/* Convert msec to counter value based on a frequency of 65.536KHz */
	hw_cnt = (usec_to_wait * CLOCK_65K) / MSEC_PER_SEC + 1;

	for (int i = 0; i < hw_cnt; i++) {
		/*
		 * Writing 00h to this register and the CPU program counter
		 * will be paused until the next low to high transition of
		 * 65.536KHz clock.
		 */
		gctrl_regs->GCTRL_WNCKR = 0x0;
	}
}
#endif

static int sys_clock_driver_init(void)
{
	int ret;

	/* Set 24-bit timer2 for timeout event */
	IRQ_CONNECT(EVENT_TIMER_IRQ, 0, evt_timer_isr, NULL, EVENT_TIMER_FLAG);
	/* Start a event timer in one system tick */
	ret = timer_init(EVENT_TIMER, EXT_PSR_32P768K, EXT_NOT_RAW_CNT,
			 MAX((1 * HW_CNT_PER_SYS_TICK), 1), EVENT_TIMER_IRQ, EVENT_TIMER_FLAG,
			 EXT_WITH_TIMER_INT);
	if (ret < 0) {
		LOG_ERR("Init event timer failed");
		return ret;
	}

	return 0;
}
SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
