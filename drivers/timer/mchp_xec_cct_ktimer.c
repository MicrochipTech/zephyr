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
 */
#define XEC_CCT_INPUT_CLOCK DT_INST_PROP(0, input_clock)
#define XEC_CCT_FREQ_HZ     (XEC_CCT_INPUT_CLOCK >> XEC_CCT_DIVIDER)

BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == XEC_CCT_FREQ_HZ,
	     "CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC must equal the CCT TCLK frequency "
	     "(input-clock >> divider)");
/* CYC_PER_TICK must be exact: a non-integer ratio makes each announced tick
 * correspond to a truncated cycle count, so kernel time drifts against real
 * time and the error accumulates without bound.
 */
BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC % CONFIG_SYS_CLOCK_TICKS_PER_SEC == 0,
	     "CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC must be an integer multiple of "
	     "CONFIG_SYS_CLOCK_TICKS_PER_SEC (else the tick rate drifts)");
BUILD_ASSERT(!IS_ENABLED(CONFIG_SMP), "XEC CCT kernel timer does not support SMP");

/* Compare0 interrupt (named in devicetree) drives the system tick. */
#define XEC_CCT_COMP0_IRQ_NUM  DT_INST_IRQ_BY_NAME(0, comp0, irq)
#define XEC_CCT_COMP0_IRQ_PRIO DT_INST_IRQ_BY_NAME(0, comp0, priority)

#if !defined(CONFIG_SYSTEM_TIMER_LPM_COMPANION_NONE)
/* Accumulated cycles missed by CCT while its clock was stopped in LPM. */
static uint32_t cct_compensation;
/* Set while the system is idle with an armed LPM companion. */
static bool timeout_idle;
/* Compensated CCT count captured at idle entry. */
static uint32_t cycle_pre_idle;
#endif

static inline uint32_t cct_comp(void)
{
#if !defined(CONFIG_SYSTEM_TIMER_LPM_COMPANION_NONE)
	return cct_compensation;
#else
	return 0U;
#endif
}

static inline uint32_t xec_cct_count_raw(void)
{
	return sys_read32(XEC_CCT_BASE + XEC_CCT_FRT_CNT_OFS);
}

static inline uint32_t xec_cct_count(void)
{
	return xec_cct_count_raw() + cct_comp();
}

static inline void xec_cct_set_comp0(uint32_t cycle)
{
	sys_write32(cycle, XEC_CCT_BASE + XEC_CCT_COMP0_OFS);
}

/*
 * Free-running 32-bit up-counter which wraps, paired with comparator 0 which
 * sets status and fires an interrupt on an exact match (count == comp0).
 * The cycle count carries the low-power compensation (cct_comp()), so the raw
 * comparator value written is the target minus that offset.
 */
#define TIMER_CORE_BACKEND_COMPARE_EXACT
#define TIMER_CORE_COUNTER_WIDTH 32

static inline uint32_t timer_driver_cycle_get(void)
{
	return xec_cct_count();
}

static inline void timer_driver_set_compare(uint64_t cycles)
{
	xec_cct_set_comp0((uint32_t)cycles - cct_comp());
}

#include "system_timer_generic.h"

static void xec_cct_isr(const void *arg)
{
	ARG_UNUSED(arg);

	/* Clear CCT comparator 0 GIRQ status bit. No need to touch CCT control */
	soc_ecia_girq_status_clear(XEC_CCT_COMP0_GIRQ, XEC_CCT_COMP0_GIRQ_POS);

#if !defined(CONFIG_SYSTEM_TIMER_LPM_COMPANION_NONE)
	if (timeout_idle) {
		/* Woke from an LPM window: defer all timekeeping to
		 * sys_clock_idle_exit(), which reconciles CCT vs companion.
		 */
		return;
	}
#endif

	timer_core_announce();
}

void sys_clock_idle_enter(uint32_t ticks)
{
	__ASSERT(sys_clock_is_locked(), "system clock lock not held");

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

#if defined(CONFIG_SYSTEM_TIMER_LPM_COMPANION_NONE)
	if (ticks != SYS_CLOCK_IDLE_FOREVER) {
		sys_clock_set_timeout(ticks, false);
		return;
	}

	/* Nothing to wake up for and the uptime may drift: park Compare0 and
	 * deactivate CCT so it cannot wake the CPU on wrap.
	 * sys_clock_idle_exit() re-activates it.
	 */
	xec_cct_set_comp0(xec_cct_count_raw() - 1U);
	soc_ecia_girq_status_clear(XEC_CCT_COMP0_GIRQ, XEC_CCT_COMP0_GIRQ_POS);
	NVIC_ClearPendingIRQ(XEC_CCT_COMP0_IRQ_NUM);
	sys_clear_bit(XEC_CCT_BASE + XEC_CCT_CR_OFS, XEC_CCT_CR_ACTV_POS);
#else
	/* Capture compensated count before deactivating CCT. Park Compare0 and
	 * deactivate CCT (ACTV=0) before WFI so its CLK_REQ is deasserted in
	 * deep sleep and it stays gated across WFI and the post-wake PLL lock
	 * window (where peripherals otherwise receive an inaccurate ring
	 * oscillator clock).
	 */
	cycle_pre_idle = xec_cct_count();
	xec_cct_set_comp0(xec_cct_count_raw() - 1U);
	soc_ecia_girq_status_clear(XEC_CCT_COMP0_GIRQ, XEC_CCT_COMP0_GIRQ_POS);
	NVIC_ClearPendingIRQ(XEC_CCT_COMP0_IRQ_NUM);
	sys_clear_bit(XEC_CCT_BASE + XEC_CCT_CR_OFS, XEC_CCT_CR_ACTV_POS);

	if (ticks == SYS_CLOCK_IDLE_FOREVER) {
		return;
	}

	/* Arm the always-on 32 KHz companion to wake the system and measure
	 * elapsed time across sleep and PLL lock.
	 */
	uint64_t timeout_us = k_ticks_to_us_ceil64(ticks);

	timeout_idle = true;
	z_sys_clock_lpm_enter(timeout_us);
#endif
}

void sys_clock_idle_exit(void)
{
	k_spinlock_key_t key = sys_clock_lock();

	/* Clear any stale Compare0 match status and re-activate CCT now that
	 * the PLL is locked and accurate.
	 */
	soc_ecia_girq_status_clear(XEC_CCT_COMP0_GIRQ, XEC_CCT_COMP0_GIRQ_POS);
	NVIC_ClearPendingIRQ(XEC_CCT_COMP0_IRQ_NUM);
	sys_set_bit(XEC_CCT_BASE + XEC_CCT_CR_OFS, XEC_CCT_CR_ACTV_POS);

#if !defined(CONFIG_SYSTEM_TIMER_LPM_COMPANION_NONE)
	if (!timeout_idle) {
		sys_clock_unlock(key);
		return;
	}

	uint64_t companion_us = z_sys_clock_lpm_exit();
	uint32_t cct_diff = xec_cct_count() - cycle_pre_idle;
	uint64_t cct_us = k_cyc_to_us_floor64(cct_diff);
	uint64_t missed_us = (companion_us > cct_us) ? (companion_us - cct_us) : 0U;
	uint64_t missed_cycles = k_us_to_cyc_floor64(missed_us);

	/* Capture unannounced cycles already in the CCT counter (pre-idle
	 * sub-tick + cct_diff) before folding missed_cycles into cct_compensation,
	 * then announce the combined span in 64-bit cycle math.
	 */
	uint32_t cct_unannounced = timer_core_cycles_since(timer_core_last_cycle);

	cct_compensation += (uint32_t)missed_cycles;
	xec_cct_set_comp0(xec_cct_count_raw() - 1U);
	timeout_idle = false;

	timer_core_announce_cycles64_from(key, (uint64_t)cct_unannounced + missed_cycles);
#else
	sys_clock_unlock(key);
#endif
}

void sys_clock_disable(void)
{
	/* Stop the free-running timer. */
	sys_clear_bit(XEC_CCT_BASE + XEC_CCT_CR_OFS, XEC_CCT_CR_FRT_EN_POS);
	soc_ecia_girq_ctrl(XEC_CCT_COMP0_GIRQ, XEC_CCT_COMP0_GIRQ_POS, 0);
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
/* Auto-reload start value: activate + auto-reload + start (CR=0x07) */
#define XEC_RTMR_CR_AUTO_RELOAD                                                                    \
	(BIT(XEC_RTMR_CR_ACTV_POS) | BIT(XEC_RTMR_CR_ARL_EN_POS) | BIT(XEC_RTMR_CR_START_POS))

/* Minimum preload (in 32.768 kHz ticks) to enable auto-reload so post-wake PLL
 * lock latency (up to 3 ms ~ 99 ticks) is also measured if the companion hits
 * terminal count to wake the system.
 */
#define XEC_RTMR_POST_WAKE_MIN_TICKS 256u

/* Companion DT node: the always-on 32 KHz RTOS timer (accessed by macro only;
 * it must not be bound by any other driver in HOOKS mode, e.g. MCHP_XEC_RTOS_TIMER
 * or the rtimer Counter driver, or the IRQ_CONNECT below would conflict).
 */
#define XEC_RTMR_NODE     DT_NODELABEL(rtimer)
#define XEC_RTMR_BASE     (mm_reg_t) DT_REG_ADDR(XEC_RTMR_NODE)
#define XEC_RTMR_IRQ_NUM  DT_IRQN(XEC_RTMR_NODE)
#define XEC_RTMR_IRQ_PRIO DT_IRQ(XEC_RTMR_NODE, priority)
#define XEC_RTMR_FREQ_HZ  DT_PROP(XEC_RTMR_NODE, clock_frequency)
#define XEC_RTMR_GIRQ     DT_PROP_BY_IDX(XEC_RTMR_NODE, girqs, 0)
#define XEC_RTMR_GIRQ_POS DT_PROP_BY_IDX(XEC_RTMR_NODE, girqs, 1)

BUILD_ASSERT(XEC_RTMR_FREQ_HZ == 32768, "LPM companion RTOS timer must run at 32768 Hz");
BUILD_ASSERT(!DT_NODE_HAS_STATUS_OKAY(XEC_RTMR_NODE),
	     "rtimer DT node must be disabled when used as LPM companion");

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
static bool xec_lpm_auto_reload;
static volatile bool xec_lpm_expired;

static inline uint32_t xec_rtmr_us_to_ticks(uint64_t us)
{
	if (us >= (UINT64_MAX / XEC_RTMR_US_NUM)) {
		return XEC_RTMR_MAX_TICKS;
	}

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
 */
static void xec_lpm_companion_isr(const void *arg)
{
	ARG_UNUSED(arg);

	xec_lpm_expired = true;
	soc_ecia_girq_status_clear(XEC_RTMR_GIRQ, XEC_RTMR_GIRQ_POS);
}

/* One-time companion setup: quiesce, connect the wake ISR, and enable the GIRQ
 * source so a terminal count can wake the core. Called from timer_core_init().
 */
void z_sys_clock_lpm_init(void)
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
	bool auto_reload = (preload > XEC_RTMR_POST_WAKE_MIN_TICKS);
	uint32_t cr_start = auto_reload ? XEC_RTMR_CR_AUTO_RELOAD : XEC_RTMR_CR_SINGLE_SHOT;

	xec_lpm_scheduled_ticks = preload;
	xec_lpm_auto_reload = auto_reload;
	xec_lpm_expired = false;

	/* Drop any stale wake status from a previous LPM cycle. */
	soc_ecia_girq_status_clear(XEC_RTMR_GIRQ, XEC_RTMR_GIRQ_POS);
	NVIC_ClearPendingIRQ(XEC_RTMR_IRQ_NUM);

	/* Required start sequence: CR=0, CR=ACTV, write preload, CR=start. */
	sys_write32(0, base + XEC_RTMR_CR_OFS);
	sys_write32(BIT(XEC_RTMR_CR_ACTV_POS), base + XEC_RTMR_CR_OFS);
	sys_write32(preload, base + XEC_RTMR_PRLD_OFS);
	sys_write32(cr_start, base + XEC_RTMR_CR_OFS);
}

uint64_t z_sys_clock_lpm_exit(void)
{
	mm_reg_t base = XEC_RTMR_BASE;
	uint32_t girq_status1 = 0u;
	uint32_t girq_status2 = 0u;
	uint32_t remaining;
	uint32_t elapsed_ticks;
	bool expired;

	/* Sample GIRQ status around the CNT read to detect a terminal-count
	 * transition that occurs while reading CNT.
	 */
	(void)soc_ecia_girq_status(XEC_RTMR_GIRQ, &girq_status1);
	remaining = sys_read32(base + XEC_RTMR_CNT_OFS);
	(void)soc_ecia_girq_status(XEC_RTMR_GIRQ, &girq_status2);
	if (((girq_status1 ^ girq_status2) & BIT(XEC_RTMR_GIRQ_POS)) != 0u) {
		remaining = sys_read32(base + XEC_RTMR_CNT_OFS);
	}

	expired = xec_lpm_expired || ((girq_status2 & BIT(XEC_RTMR_GIRQ_POS)) != 0u);

	/* Stop the companion and clear its wake status/pending. */
	sys_write32(0, base + XEC_RTMR_CR_OFS);
	xec_lpm_expired = false;
	soc_ecia_girq_status_clear(XEC_RTMR_GIRQ, XEC_RTMR_GIRQ_POS);
	NVIC_ClearPendingIRQ(XEC_RTMR_IRQ_NUM);

	if (expired) {
		/* Reached terminal count: the full programmed window elapsed,
		 * plus any post-wake ticks counted after auto-reload (such as
		 * the deep-sleep PLL lock wait).
		 */
		elapsed_ticks = xec_lpm_scheduled_ticks;
		if (xec_lpm_auto_reload && (remaining > 0u) &&
		    (remaining < xec_lpm_scheduled_ticks)) {
			elapsed_ticks += (xec_lpm_scheduled_ticks - remaining);
		}
	} else if ((remaining > 0u) && (remaining < xec_lpm_scheduled_ticks)) {
		/* Woken early by another source: elapsed = programmed - remaining. */
		elapsed_ticks = xec_lpm_scheduled_ticks - remaining;
	} else {
		/* Either remaining == xec_lpm_scheduled_ticks, or remaining == 0
		 * while !expired (meaning the timer is still in the initial
		 * < 30.5 us AHB-to-32KHz preload synchronization window).
		 */
		elapsed_ticks = 0u;
	}

	return xec_rtmr_ticks_to_us(elapsed_ticks);
}

#endif /* CONFIG_SYSTEM_TIMER_LPM_COMPANION_HOOKS */

static int sys_clock_driver_init(void)
{
	mm_reg_t regbase = XEC_CCT_BASE;
	uint32_t v = 0;

	/* The system tick uses Compare0 only; keep its GIRQ source disabled
	 * until the handler is connected and the counter is running.
	 */
	soc_ecia_girq_ctrl(XEC_CCT_COMP0_GIRQ, XEC_CCT_COMP0_GIRQ_POS, 0);
	soc_xec_pcr_sleep_en_clear((uint8_t)(XEC_CCT_PCR_SCR_VAL));

	/* Activate the block, reset the free-running counter, park Compare0
	 * before enabling COMP0_EN so it does not match 0 on reset, and select
	 * the TCLK divider. COMP0_SET/CLR bits are left untouched.
	 */
	v = (BIT(XEC_CCT_CR_ACTV_POS) | BIT(XEC_CCT_CR_FRT_RST_POS) |
	     XEC_CCT_CR_TCLK_SET(XEC_CCT_DIVIDER));
	sys_write32(v, regbase + XEC_CCT_CR_OFS);

	xec_cct_set_comp0(UINT32_MAX);
	sys_set_bit(regbase + XEC_CCT_CR_OFS, XEC_CCT_CR_COMP0_EN_POS);

	soc_ecia_girq_status_clear(XEC_CCT_COMP0_GIRQ, XEC_CCT_COMP0_GIRQ_POS);
	NVIC_ClearPendingIRQ(XEC_CCT_COMP0_IRQ_NUM);

	IRQ_CONNECT(XEC_CCT_COMP0_IRQ_NUM, XEC_CCT_COMP0_IRQ_PRIO, xec_cct_isr, NULL, 0);
	irq_enable(XEC_CCT_COMP0_IRQ_NUM);

	/* Start the free-running timer, then seed the announce baseline and
	 * arm the first tick on Compare0.
	 */
	sys_set_bit(regbase + XEC_CCT_CR_OFS, XEC_CCT_CR_FRT_EN_POS);

	timer_core_init();

	soc_ecia_girq_ctrl(XEC_CCT_COMP0_GIRQ, XEC_CCT_COMP0_GIRQ_POS, 1U);

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
