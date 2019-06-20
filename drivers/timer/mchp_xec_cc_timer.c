/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <drivers/system_timer.h>
#include <sys_clock.h>
#include <spinlock.h>
#include <arch/arm/cortex_m/cmsis.h>

#define DEBUG_CC_TIMER

#define TIMER_IRQ UTIL_CAT(XCHAL_TIMER,		\
			   UTIL_CAT(CONFIG_XTENSA_TIMER_ID, _INTERRUPT))

#define CYC_PER_TICK (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC	\
		      / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_TICKS ((0xffffffffu - CYC_PER_TICK) / CYC_PER_TICK)
#define MIN_DELAY 1000

#define CCT_CTRL_STS_MASK (MCHP_CCT_CTRL_COMP1_SET + MCHP_CCT_CTRL_COMP0_SET +\
			   MCHP_CCT_CTRL_COMP1_CLR + MCHP_CCT_CTRL_COMP0_CLR)

#define CCT_CTRL_NOT_STS_MASK ~(CCT_CTRL_STS_MASK)

static struct k_spinlock lock;
static unsigned int last_count;

/*
 * Update CCT control register except for comparator set status and
 * clear status bits.
 */
#if 0
static void cct_ctrl_set(u32_t val)
{
	CCT_REGS->CTRL = (CCT_REGS->CTRL & CCT_CTRL_NOT_STS_MASK)
			 | (val & CCT_CTRL_NOT_STS_MASK);
}
#endif

static INLINE void cct_ctrl_fr_start(void)
{
	CCT_REGS->CTRL = (CCT_REGS->CTRL & CCT_CTRL_NOT_STS_MASK)
			  | MCHP_CCT_CTRL_FRUN_EN;
}

static INLINE void cct_ctrl_fr_stop(void)
{
	CCT_REGS->CTRL = (CCT_REGS->CTRL & CCT_CTRL_NOT_STS_MASK)
			  & ~(MCHP_CCT_CTRL_FRUN_EN);
}

/*
 * data sheet recommends disabling the CCT before updateing.
 */
static void set_ccompare(u32_t val)
{
	cct_ctrl_fr_stop();
	CCT_REGS->COMP0 = val;
	cct_ctrl_fr_start();
}

static u32_t ccount(void)
{
	return CCT_REGS->FREE_RUN;
}

#if 0
static void cct_comp0_enable(u8_t enable)
{
	if (enable) {
		CCT_REGS->CTRL = (CCT_REGS->CTRL & CCT_CTRL_NOT_STS_MASK)
				  | MCHP_CCT_CTRL_COMP0_EN;
	} else {
		CCT_REGS->CTRL = (CCT_REGS->CTRL & CCT_CTRL_NOT_STS_MASK)
				  & ~(MCHP_CCT_CTRL_COMP0_EN);
	}
}
#endif

static void cct_comp0_clear(void)
{
	CCT_REGS->CTRL = (CCT_REGS->CTRL & CCT_CTRL_NOT_STS_MASK)
			  | MCHP_CCT_CTRL_COMP0_CLR;
	GIRQ18_REGS->SRC = MCHP_CCT_CMP0_GIRQ_VAL;
}

#if 0
static void cct_comp0_ien(uint8_t enable)
{
	if (enable) {
		GIRQ18_REGS->EN_SET = MCHP_CCT_CMP0_GIRQ_VAL;
	} else {
		GIRQ18_REGS->EN_CLR = MCHP_CCT_CMP0_GIRQ_VAL;
	}
}
#endif

/*
 * XEC capture compare timer comparator 0 ISR
 */
static void xec_cct_comp0_isr(void *arg)
{
	ARG_UNUSED(arg);

#ifdef DEBUG_CC_TIMER
	GPIO_CTRL_REGS->CTRL_0015 ^= (1ul << 16);
#endif
	cct_comp0_clear();

	k_spinlock_key_t key = k_spin_lock(&lock);
	u32_t curr = ccount();
	u32_t dticks = (curr - last_count) / CYC_PER_TICK;

	last_count += dticks * CYC_PER_TICK;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL) ||
	    IS_ENABLED(CONFIG_QEMU_TICKLESS_WORKAROUND)) {
		u32_t next = last_count + CYC_PER_TICK;

		if ((s32_t)(next - curr) < MIN_DELAY) {
			next += CYC_PER_TICK;
		}
		set_ccompare(next);
	}

	k_spin_unlock(&lock, key);
	z_clock_announce(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ? dticks : 1);
}

int z_clock_driver_init(struct device *device)
{
	mchp_pcr_periph_slp_ctrl(PCR_CCT, 0);
	mchp_pcr_periph_reset(PCR_CCT);

	GIRQ18_REGS->SRC = MCHP_CCT_CMP0_GIRQ_VAL + MCHP_CCT_CMP1_GIRQ_VAL;
	CCT_REGS->COMP0 = CYC_PER_TICK;
	CCT_REGS->CTRL = MCHP_CCT_CTRL_ACTIVATE + MCHP_CCT_CTRL_TCLK_DIV_2
			 + MCHP_CCT_CTRL_COMP0_EN;

	IRQ_CONNECT(CCT_CMP0_IRQn, 1, xec_cct_comp0_isr, 0, 0);
	irq_enable(CCT_CMP0_IRQn);
	GIRQ18_REGS->EN_SET = MCHP_CCT_CMP0_GIRQ_VAL;

	CCT_REGS->CTRL = MCHP_CCT_CTRL_ACTIVATE + MCHP_CCT_CTRL_TCLK_DIV_2
			 + MCHP_CCT_CTRL_COMP0_EN + MCHP_CCT_CTRL_FRUN_EN;

	return 0;
}

void z_clock_set_timeout(s32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

#if defined(CONFIG_TICKLESS_KERNEL) && !defined(CONFIG_QEMU_TICKLESS_WORKAROUND)
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

