/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <drivers/timer/system_timer.h>
#include <sys_clock.h>
#include <spinlock.h>
#include <soc.h>


BUILD_ASSERT_MSG(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == 48000000,
		 "XEC CC timer HW frequency is not 48000000");

#define CYC_PER_TICK (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC	\
		      / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_TICKS ((0xffffffffu - CYC_PER_TICK) / CYC_PER_TICK)
#define MAX_CYCLES (MAX_TICKS * CYC_PER_TICK)
#define MIN_DELAY 400

/*
 * Clock select b[6:4] = 000b = divide by 1
 */
#define CCT_CTRL_COMP0_DIS_FREE_RUN_DIS		0x0001UL
#define CCT_CTRL_COMP0_DIS_FREE_RUN_EN		0x0003UL
#define CCT_CTRL_COMP0_EN_FREE_RUN_EN		0x0103UL
#define CCT_CTRL_CLR_COMP0_STATUS		(1UL << 25)
#define CCT_CTRL_CLR_COMP1_STATUS		(1UL << 24)

static struct k_spinlock lock;
static unsigned int last_count;
static unsigned int last_cycles;


static void set_ccompare(u32_t val)
{
	CCT_REGS->CTRL = CCT_CTRL_COMP0_DIS_FREE_RUN_EN;
	CCT_REGS->COMP0 = val;
	CCT_REGS->CTRL = CCT_CTRL_COMP0_EN_FREE_RUN_EN;
}

static u32_t ccount(void)
{
	return CCT_REGS->FREE_RUN;
}

static void cct_comp0_isr(void *arg)
{
	ARG_UNUSED(arg);

	GPIO_CTRL_REGS->CTRL_0015 = 0x00240UL;

	k_spinlock_key_t key = k_spin_lock(&lock);
	u32_t curr = ccount();
	u32_t dticks = (curr - last_count) / CYC_PER_TICK;

	last_cycles += (curr - last_cycles); /* MCHP */

	last_count += dticks * CYC_PER_TICK;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		u32_t next = last_count + CYC_PER_TICK;

		if ((s32_t)(next - curr) < MIN_DELAY) {
			next += CYC_PER_TICK;
		}
		set_ccompare(next);
	}

	k_spin_unlock(&lock, key);
	z_clock_announce(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ? dticks : 1);
	GPIO_CTRL_REGS->CTRL_0015 = 0x10240UL;
}

int z_clock_driver_init(struct device *device)
{
	/* reset free run and clear comparators */
	CCT_REGS->CTRL = MCHP_CCT_CTRL_ACTIVATE | MCHP_CCT_CTRL_FRUN_RESET
			 | CCT_CTRL_CLR_COMP0_STATUS
			 | CCT_CTRL_CLR_COMP0_STATUS;
	GIRQ18_REGS->SRC = MCHP_CCT_CMP0_GIRQ_VAL;
	GIRQ18_REGS->EN_SET = MCHP_CCT_CMP0_GIRQ_VAL;

	IRQ_CONNECT(CCT_CMP0_IRQn, 0, cct_comp0_isr, 0, 0);
	set_ccompare(CYC_PER_TICK);
	irq_enable(CCT_CMP0_IRQn);

	return 0;
}

void z_clock_set_timeout(s32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

#if defined(CONFIG_TICKLESS_KERNEL)

	ticks = ticks == K_FOREVER ? MAX_TICKS : ticks;
	ticks = MAX(MIN(ticks - 1, (s32_t)MAX_TICKS), 0);

	k_spinlock_key_t key = k_spin_lock(&lock);
	u32_t curr = ccount(), cyc;

	/* Round up to next tick boundary */
	cyc = ticks * CYC_PER_TICK + (curr - last_count) + (CYC_PER_TICK - 1);
	cyc = (cyc / CYC_PER_TICK) * CYC_PER_TICK;
	cyc += last_count;
	if ((cyc - curr) < MIN_DELAY) {
		cyc += CYC_PER_TICK;
	}

	set_ccompare(cyc);
	k_spin_unlock(&lock, key);
#endif
}

u32_t z_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	u32_t ret = (ccount() - last_count) / CYC_PER_TICK;

	k_spin_unlock(&lock, key);
	return ret;
}

u32_t z_timer_cycle_get_32(void)
{
	return ccount();
}

#ifdef CONFIG_ARCH_HAS_CUSTOM_BUSY_WAIT

#define DELAY_CALL_OVERHEAD_US 0

/*
 * Default busy wait algorithm causing failure in timer tests.
 */
void z_arch_busy_wait(u32_t usec_to_wait)
{
	if (usec_to_wait < DELAY_CALL_OVERHEAD_US) {
		return;
	}

	usec_to_wait -= DELAY_CALL_OVERHEAD_US;
	/* use 64-bit math to prevent overflow when multiplying */
	u32_t cycles_to_wait = (u32_t)(
		(u64_t)usec_to_wait * 48U
	);
	u32_t start_cycles = CCT_REGS->FREE_RUN & 0x7FFFFFFFUL;

	for (;;) {
		u32_t current_cycles = CCT_REGS->FREE_RUN & 0x7FFFFFFFUL;
		/* handle the rollover on an unsigned 32-bit value */
		if (((current_cycles - start_cycles) & 0x7FFFFFFFUL)
				>= cycles_to_wait) {
			break;
		}
	}
}
#endif
