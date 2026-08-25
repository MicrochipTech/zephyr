/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_cct_ktimer

#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/irq.h>
#include <cmsis_core.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/drivers/timer/system_timer_lpm.h>
#include <zephyr/sys/clock.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

/* Capture Compare Timer (CCT) register offsets */
#define XEC_CCT_CR_OFS      0
#define XEC_CCT_CAP_CR0_OFS 0x04U
#define XEC_CCT_CAP_CR1_OFS 0x08U
#define XEC_CCT_FRT_CNT_OFS 0x0CU /* 32-bit r/w */
#define XEC_CCT_CAP0_OFS    0x10U /* all capture regs are 32-bit read-only */
#define XEC_CCT_CAP1_OFS    0x14U
#define XEC_CCT_CAP2_OFS    0x18U
#define XEC_CCT_CAP3_OFS    0x1CU
#define XEC_CCT_CAP4_OFS    0x20U
#define XEC_CCT_CAP5_OFS    0x24U
#define XEC_CCT_COMP0_OFS   0x28U /* 32-bit r/w */
#define XEC_CCT_COMP1_OFS   0x2CU /* 32-bit r/w */
#define XEC_CCT_INMUX_OFS   0x30U

/* Control TCLK and FCLK field values */
#define XEC_CCT_TFCLK_VAL_DIV1   0
#define XEC_CCT_TFCLK_VAL_DIV2   1U
#define XEC_CCT_TFCLK_VAL_DIV4   2U
#define XEC_CCT_TFCLK_VAL_DIV8   3U
#define XEC_CCT_TFCLK_VAL_DIV16  4U
#define XEC_CCT_TFCLK_VAL_DIV32  5U
#define XEC_CCT_TFCLK_VAL_DIV64  6U
#define XEC_CCT_TFCLK_VAL_DIV128 7U

/* Capture Control Edge field values */
#define XEC_CCT_CAP_CR_FE  0
#define XEC_CCT_CAP_CR_RE  1U
#define XEC_CCT_CAP_CR_BE  2U
#define XEC_CCT_CAP_CR_DIS 3U

/* Control fields: all fields r/w unless otherwise noted */
#define XEC_CCT_CR_ACTV_POS      0
#define XEC_CCT_CR_FRT_EN_POS    1
#define XEC_CCT_CR_FRT_RST_POS   2 /* self-clearing */
#define XEC_CCT_CR_TCLK_POS      4
#define XEC_CCT_CR_TCLK_MSK      GENMASK(6, 4)
#define XEC_CCT_CR_TCLK_SET(v)   FIELD_PREP(XEC_CCT_CR_TCLK_MSK, (v))
#define XEC_CCT_CR_TCLK_GET(r)   FIELD_GET(XEC_CCT_CR_TCLK_MSK, (r))
#define XEC_CCT_CR_COMP0_EN_POS  8
#define XEC_CCT_CR_COMP1_EN_POS  9
#define XEC_CCT_CR_COMP1_SET_POS 16 /* r/w-1-to-set */
#define XEC_CCT_CR_COMP0_SET_POS 17 /* r/w-1-to-set */
#define XEC_CCT_CR_COMP1_CLR_POS 24 /* r/w-1-to-clear */
#define XEC_CCT_CR_COMP0_CLR_POS 25 /* r/w-1-to-clear */

/* girqs pairs are parallel to interrupts/interrupt-names */
#define XEC_CCT_BASE (mm_reg_t) DT_INST_REG_ADDR(0)

#define XEC_CCT_PCR_SCR_VAL DT_INST_PROP(0, pcr_scr)

#define XEC_CCT_COMP0_GIRQ_IDX 7
#define XEC_CCT_COMP1_GIRQ_IDX 8

#define XEC_CCT_COMP0_GIRQ_POS_IDX (XEC_CCT_COMP0_GIRQ_IDX + 1)
#define XEC_CCT_COMP1_GIRQ_POS_IDX (XEC_CCT_COMP1_GIRQ_IDX + 1)

#define XEC_CCT_COMP0_GIRQ MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(0, girqs, XEC_CCT_COMP0_GIRQ_IDX))
#define XEC_CCT_COMP1_GIRQ MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(0, girqs, XEC_CCT_COMP1_GIRQ_IDX))

#define XEC_CCT_COMP0_GIRQ_POS                                                                     \
	MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(0, girqs, XEC_CCT_COMP0_GIRQ_IDX))
#define XEC_CCT_COMP1_GIRQ_POS                                                                     \
	MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(0, girqs, XEC_CCT_COMP1_GIRQ_IDX))

#define XEC_CCT_DIVIDER XEC_CCT_TFCLK_VAL_DIV4

/*
 * The TCLK divider field value is also the power-of-two shift, so the CCT
 * free-running counter frequency is input-clock >> divider (48 MHz / 4 =
 * 12 MHz for DIV4). The kernel's HW cycle rate must match this.
 *
 * Because it does match, the generic timer core is left on its default
 * TIMER_CORE_CYCLES_PER_SEC (the kernel rate) and emits sys_clock_cycle_get_32/64()
 * for us: the counter is already in the unit the kernel expects to read.
 */
#define XEC_CCT_INPUT_CLOCK DT_INST_PROP(0, input_clock)
#define XEC_CCT_FREQ_HZ     (XEC_CCT_INPUT_CLOCK >> XEC_CCT_DIVIDER)

BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == XEC_CCT_FREQ_HZ,
	     "CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC must equal the CCT TCLK frequency "
	     "(input-clock >> divider)");
/* Cycles per tick must be exact: a non-integer ratio makes each announced tick
 * correspond to a truncated cycle count, so kernel time drifts against real
 * time and the error accumulates without bound. The core checks that the ratio
 * is non-zero and that a tick fits the counter; that it divides evenly is this
 * driver's own requirement.
 */
BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC % CONFIG_SYS_CLOCK_TICKS_PER_SEC == 0,
	     "CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC must be an integer multiple of "
	     "CONFIG_SYS_CLOCK_TICKS_PER_SEC (else the tick rate drifts)");
BUILD_ASSERT(!IS_ENABLED(CONFIG_SMP), "XEC CCT kernel timer does not support SMP");

/* Compare0 interrupt (named in devicetree) drives the system tick. */
#define XEC_CCT_COMP0_IRQ_NUM  DT_INST_IRQ_BY_NAME(0, comp0, irq)
#define XEC_CCT_COMP0_IRQ_PRIO DT_INST_IRQ_BY_NAME(0, comp0, priority)

#if !defined(CONFIG_SYSTEM_TIMER_LPM_COMPANION_NONE)
/* Set while the system is idle with an armed LPM companion. */
static bool timeout_idle;
/* Cycle count captured at idle entry, to measure how far the CCT itself
 * advanced (nonzero only if its clock survived the low-power state).
 */
static uint32_t cycle_pre_idle;
#endif

/*
 * Software extension of the free-running counter, in cycles.
 *
 * The CCT counter is the cycle time base, but it is clocked from the PLL and so
 * stops in the deep-sleep states this driver targets: it freezes and resumes,
 * having lost the sleep. Time recovered from the always-on companion is folded
 * in here (see sys_clock_idle_exit()), which keeps the cycle domain the core
 * accounts in continuous with real time even though the hardware register is
 * not. Outside a low-power window this is a constant and the count is the raw
 * register.
 *
 * It is written only from sys_clock_idle_exit(), on the CPU's way out of idle
 * with the clock lock held. Nothing else on this (UP-only, see the SMP assert
 * above) CPU can observe the read below mid-update: no ISR and no reprogram
 * path touches it, which is what TIMER_CORE_COUNTER_NONATOMIC would be for. So
 * the cycle getter stays a lock-free pair of loads, which matters because it is
 * on the thread-usage timestamp hot path.
 */
static uint32_t cct_cycle_offset;

/*
 * A free-running 32-bit counter matched by a 32-bit comparator that fires on
 * equality only, so the core wraps the compare write in its verify loop and
 * this driver needs no minimum-delay floor of its own (the MIN_DELAY that
 * predated the core, and the tuning that went with it, are gone).
 */
#define TIMER_CORE_BACKEND_COMPARE_EXACT
#define TIMER_CORE_COUNTER_WIDTH 32

/*
 * Cycles a compare must lead the counter by to be caught. The comparator lives
 * in the TCLK domain (input clock >> divider) while the write arrives on the
 * AHB clock, so a match programmed at the default lead of one cycle can be
 * crossed before it lands. Four TCLK cycles (333 ns at 12 MHz) covers a
 * multi-flop synchroniser. Getting this too low costs only extra iterations of
 * the core's verify loop, never a lost interrupt; it is the one number here
 * worth re-checking against silicon.
 */
#define TIMER_CORE_ALARM_LEAD_CYCLES 4

static inline uint32_t timer_driver_cycle_get(void)
{
	return sys_read32(XEC_CCT_BASE + XEC_CCT_FRT_CNT_OFS) + cct_cycle_offset;
}

static inline void timer_driver_set_compare(uint32_t cycles)
{
	/* Back out of the extended cycle domain into the counter's own. */
	sys_write32(cycles - cct_cycle_offset, XEC_CCT_BASE + XEC_CCT_COMP0_OFS);
}

#include "system_timer_generic.h"

static void xec_cct_isr(const void *arg)
{
	ARG_UNUSED(arg);

	k_spinlock_key_t key = sys_clock_lock();

	/* Clearing the GIRQ18 source bit alone acks the compare interrupt
	 * (per README: CCT control bits 16/17/24/25 can be ignored).
	 */
	soc_ecia_girq_status_clear(XEC_CCT_COMP0_GIRQ, XEC_CCT_COMP0_GIRQ_POS);

	/*
	 * Announce whatever the counter says elapsed, even on the compare that
	 * ends a low-power window. That span is the time the CCT measured
	 * itself; the span it slept through is recovered separately by
	 * sys_clock_idle_exit(), so the two are disjoint and neither is counted
	 * twice. Which of the two runs first does not matter.
	 */
	timer_core_announce_from(key);
}

/*
 * ---------------------------------------------------------------------------
 * Low-power companion (CONFIG_SYSTEM_TIMER_LPM_COMPANION_HOOKS)
 *
 * The CCT is clocked from the SoC PLL, which is powered down in the deep-sleep
 * states this driver targets, so the CCT cannot keep time there. Across those
 * states timekeeping is delegated to the always-on 32 KHz RTOS timer -- a
 * 32-bit down-counter that loads a preload value, counts to zero, and raises
 * an interrupt. The SoC PM/idle path calls z_sys_clock_lpm_enter() just before
 * WFI to arm a wake no later than the requested time, and z_sys_clock_lpm_exit()
 * after wake to recover how long the system actually slept.
 * ---------------------------------------------------------------------------
 */
#ifdef CONFIG_SYSTEM_TIMER_LPM_COMPANION_HOOKS

/* RTOS timer (companion) register offsets */
#define XEC_RTMR_CNT_OFS  0u    /* 32-bit R/W down-counter */
#define XEC_RTMR_PRLD_OFS 0x04u /* 32-bit R/W preload */
#define XEC_RTMR_CR_OFS   0x08u /* 32-bit R/W control */

/* RTOS timer control bits */
#define XEC_RTMR_CR_ACTV_POS   0 /* activate block */
#define XEC_RTMR_CR_ARL_EN_POS 1 /* auto-reload enable */
#define XEC_RTMR_CR_START_POS  2 /* start countdown */

/* Single-shot start value: activate + start, no auto-reload (README: CR=0x05) */
#define XEC_RTMR_CR_SINGLE_SHOT (BIT(XEC_RTMR_CR_ACTV_POS) | BIT(XEC_RTMR_CR_START_POS))

/* Companion DT node: the always-on 32 KHz RTOS timer (accessed by macro only;
 * it must not be bound by any other driver in HOOKS mode, e.g. MCHP_XEC_RTOS_TIMER
 * or the rtimer Counter driver, or the IRQ_CONNECT below would conflict).
 */
#define XEC_RTMR_NODE     DT_NODELABEL(rtimer)
#define XEC_RTMR_BASE     (uintptr_t)DT_REG_ADDR(XEC_RTMR_NODE)
#define XEC_RTMR_IRQ_NUM  DT_IRQN(XEC_RTMR_NODE)
#define XEC_RTMR_IRQ_PRIO DT_IRQ(XEC_RTMR_NODE, priority)
#define XEC_RTMR_FREQ_HZ  DT_PROP(XEC_RTMR_NODE, clock_frequency)
#define XEC_RTMR_GIRQ     DT_PROP_BY_IDX(XEC_RTMR_NODE, girqs, 0)
#define XEC_RTMR_GIRQ_POS DT_PROP_BY_IDX(XEC_RTMR_NODE, girqs, 1)

BUILD_ASSERT(XEC_RTMR_FREQ_HZ == 32768, "LPM companion RTOS timer must run at 32768 Hz");

/*
 * 32768 Hz means exactly 512 / 15625 ticks per microsecond. Both ratios are
 * exact, so convert with 64-bit rational arithmetic and no floating point:
 *   ticks = us    * 512   / 15625
 *   us    = ticks * 15625 / 512
 */
#define XEC_RTMR_US_NUM 512u
#define XEC_RTMR_US_DEN 15625u

/*
 * Largest preload we arm. The count register is 32-bit, but the validated XEC
 * RTOS system-timer driver treats only bits[27:0] as a reliable count (it uses
 * the top nibble as a STOPPED sentinel), so stay within that range. 0x0fffffff
 * ticks is ~2.27 hours -- far beyond any single idle window. A longer request
 * simply wakes early and the kernel re-arms, which the "wake no later than"
 * contract explicitly permits.
 */
#define XEC_RTMR_MAX_TICKS 0x0fffffffu

/* Preload programmed at the last z_sys_clock_lpm_enter(), read back at exit. */
static uint32_t xec_lpm_scheduled_ticks;

static inline uint32_t xec_rtmr_us_to_ticks(uint64_t us)
{
	uint64_t t = (us * XEC_RTMR_US_NUM) / XEC_RTMR_US_DEN;

	if (t > XEC_RTMR_MAX_TICKS) {
		t = XEC_RTMR_MAX_TICKS;
	}
	if (t == 0u) {
		t = 1u; /* guarantee a real countdown */
	}

	return (uint32_t)t;
}

static inline uint64_t xec_rtmr_ticks_to_us(uint32_t ticks)
{
	return ((uint64_t)ticks * XEC_RTMR_US_DEN) / XEC_RTMR_US_NUM;
}

/*
 * Companion wake ISR. Its only job is to acknowledge the RTOS timer so the
 * interrupt does not re-assert after waking the core; elapsed-time recovery is
 * done in z_sys_clock_lpm_exit(). It deliberately does not call
 * sys_clock_announce() -- the CCT remains the system timer.
 *
 * After exiting WFI the arch idle code re-enables interrupts before
 * sys_clock_idle_exit() runs, so this handler may fire before lpm_exit(). It
 * only stops the timer and clears status; it never touches the count register,
 * which stays at its terminal value (0) for lpm_exit() to read.
 */
static void xec_lpm_companion_isr(const void *arg)
{
	ARG_UNUSED(arg);

	sys_write32(0, XEC_RTMR_BASE + XEC_RTMR_CR_OFS);
	soc_ecia_girq_status_clear(XEC_RTMR_GIRQ, XEC_RTMR_GIRQ_POS);
}

/* One-time companion setup: quiesce, connect the wake ISR, and enable the GIRQ
 * source so a terminal count can wake the core. Called from driver init.
 */
static void xec_lpm_companion_init(void)
{
	sys_write32(0, XEC_RTMR_BASE + XEC_RTMR_CR_OFS);
	soc_ecia_girq_ctrl(XEC_RTMR_GIRQ, XEC_RTMR_GIRQ_POS, 0);
	soc_ecia_girq_status_clear(XEC_RTMR_GIRQ, XEC_RTMR_GIRQ_POS);
	NVIC_ClearPendingIRQ(XEC_RTMR_IRQ_NUM);

	IRQ_CONNECT(XEC_RTMR_IRQ_NUM, XEC_RTMR_IRQ_PRIO, xec_lpm_companion_isr, NULL, 0);
	irq_enable(XEC_RTMR_IRQ_NUM);

	soc_ecia_girq_ctrl(XEC_RTMR_GIRQ, XEC_RTMR_GIRQ_POS, 1);
}

void z_sys_clock_lpm_enter(uint64_t max_lpm_time_us)
{
	mm_reg_t base = XEC_RTMR_BASE;
	uint32_t preload = xec_rtmr_us_to_ticks(max_lpm_time_us);

	xec_lpm_scheduled_ticks = preload;

	/* Drop any stale wake status from a previous LPM cycle. */
	soc_ecia_girq_status_clear(XEC_RTMR_GIRQ, XEC_RTMR_GIRQ_POS);
	NVIC_ClearPendingIRQ(XEC_RTMR_IRQ_NUM);

	/* Required start sequence (README): CR=0, write preload, CR=single-shot. */
	sys_write32(0, base + XEC_RTMR_CR_OFS);
	sys_write32(preload, base + XEC_RTMR_PRLD_OFS);
	sys_write32(XEC_RTMR_CR_SINGLE_SHOT, base + XEC_RTMR_CR_OFS);
}

uint64_t z_sys_clock_lpm_exit(void)
{
	mm_reg_t base = XEC_RTMR_BASE;
	uint32_t remaining = sys_read32(base + XEC_RTMR_CNT_OFS);
	uint32_t elapsed_ticks;

	/* Stop the companion and clear its wake status/pending. */
	sys_write32(0, base + XEC_RTMR_CR_OFS);
	soc_ecia_girq_status_clear(XEC_RTMR_GIRQ, XEC_RTMR_GIRQ_POS);
	NVIC_ClearPendingIRQ(XEC_RTMR_IRQ_NUM);

	if (remaining == 0u) {
		/* Reached terminal count: the full programmed window elapsed. */
		elapsed_ticks = xec_lpm_scheduled_ticks;
	} else if (remaining < xec_lpm_scheduled_ticks) {
		/* Woken early by another source: elapsed = programmed - remaining. */
		elapsed_ticks = xec_lpm_scheduled_ticks - remaining;
	} else {
		/* No measurable time elapsed. */
		elapsed_ticks = 0u;
	}

	return xec_rtmr_ticks_to_us(elapsed_ticks);
}

#endif /* CONFIG_SYSTEM_TIMER_LPM_COMPANION_HOOKS */

#if !defined(CONFIG_SYSTEM_TIMER_LPM_COMPANION_NONE)

void sys_clock_idle_enter(uint32_t ticks)
{
	__ASSERT(sys_clock_is_locked(), "system clock lock not held");

	/* SYS_CLOCK_IDLE_FOREVER converts to a wakeup days out, which is the
	 * intent: the companion may wake earlier, never later. Its own range cap
	 * turns that into an early wake the kernel simply re-arms from.
	 */
	uint64_t timeout_us = k_ticks_to_us_ceil64(ticks);

	timeout_idle = true;
	cycle_pre_idle = timer_driver_cycle_get();

	/* Arm the always-on companion to guarantee wake even if the CCT clock
	 * (PLL) stops in the upcoming low-power state. Compare0 stays armed at
	 * the deadline the core last programmed, so if the CCT keeps running it
	 * wakes us itself.
	 */
	z_sys_clock_lpm_enter(timeout_us);
}

void sys_clock_idle_exit(void)
{
	if (!timeout_idle) {
		return;
	}

	k_spinlock_key_t key = sys_clock_lock();

	/*
	 * How long the CCT itself measured, against how long the companion says
	 * the system was away. The difference is the span the CCT lost with its
	 * clock, which is nothing at all if the clock survived.
	 *
	 * This assumes the counter *freezes* and resumes across PLL-off. If it
	 * is instead reset, the delta below is garbage and this needs
	 * CONFIG_SYSTEM_TIMER_RESET_BY_LPM handling, which is not implemented.
	 */
	uint32_t cct_cycles = timer_driver_cycle_get() - cycle_pre_idle;
	uint64_t cct_us = k_cyc_to_us_floor64(cct_cycles);
	uint64_t companion_us = z_sys_clock_lpm_exit();
	uint64_t missed_cycles = 0;

	if (companion_us > cct_us) {
		missed_cycles = k_us_to_cyc_floor64(companion_us - cct_us);
	}

	/*
	 * Fold the lost span into the cycle domain so the counter and the
	 * core's announce baseline stay in step, then hand the same span to the
	 * core directly rather than routing it through the counter: a sleep can
	 * outrun what 32 bits express, and only this path pays for the wider
	 * arithmetic.
	 *
	 * The sleep alone: what was already un-announced when the CPU went idle
	 * stays in the counter, as does the sub-tick remainder of the span
	 * above, and the next announce picks both up.
	 */
	cct_cycle_offset += (uint32_t)missed_cycles;
	timeout_idle = false;

	timer_core_announce_cycles64_from(key, missed_cycles);
}

#endif /* !CONFIG_SYSTEM_TIMER_LPM_COMPANION_NONE */

void sys_clock_disable(void)
{
	/* Stop the free-running timer. */
	sys_clear_bit(XEC_CCT_BASE + XEC_CCT_CR_OFS, XEC_CCT_CR_FRT_EN_POS);
	soc_ecia_girq_ctrl(XEC_CCT_COMP0_GIRQ, XEC_CCT_COMP0_GIRQ_POS, 0);
}

static int sys_clock_driver_init(void)
{
	mm_reg_t regbase = XEC_CCT_BASE;
	uint32_t v = 0;

	/* The system tick uses Compare0 only; keep its GIRQ source disabled
	 * until the handler is connected and the counter is running.
	 */
	soc_ecia_girq_ctrl(XEC_CCT_COMP0_GIRQ, XEC_CCT_COMP0_GIRQ_POS, 0);
	soc_xec_pcr_sleep_en_clear((uint8_t)(XEC_CCT_PCR_SCR_VAL));

	/* Activate the block, reset the free-running counter, enable the
	 * Compare0 match, and select the TCLK divider. COMP0_SET/CLR bits are
	 * left untouched (per README they can be ignored).
	 */
	v = (BIT(XEC_CCT_CR_ACTV_POS) | BIT(XEC_CCT_CR_FRT_RST_POS) | BIT(XEC_CCT_CR_COMP0_EN_POS) |
	     XEC_CCT_CR_TCLK_SET(XEC_CCT_DIVIDER));

	sys_write32(v, regbase + XEC_CCT_CR_OFS);

	soc_ecia_girq_status_clear(XEC_CCT_COMP0_GIRQ, XEC_CCT_COMP0_GIRQ_POS);

	IRQ_CONNECT(XEC_CCT_COMP0_IRQ_NUM, XEC_CCT_COMP0_IRQ_PRIO, xec_cct_isr, NULL, 0);
	irq_enable(XEC_CCT_COMP0_IRQ_NUM);

	/* Start the free-running timer before seeding the baseline from it. */
	sys_set_bit(regbase + XEC_CCT_CR_OFS, XEC_CCT_CR_FRT_EN_POS);

	/* Seed the announce baseline from the counter and arm the first tick. */
	timer_core_init();

	soc_ecia_girq_ctrl(XEC_CCT_COMP0_GIRQ, XEC_CCT_COMP0_GIRQ_POS, 1U);

#ifdef CONFIG_SYSTEM_TIMER_LPM_COMPANION_HOOKS
	xec_lpm_companion_init();
#endif

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
