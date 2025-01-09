/*
 * Copyright (c) 2020 Intel Corporation.
 * Copyright (c) 2021 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm/arch.h>
#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/util.h>
#include <zephyr/timing/timing.h>
#include <soc.h>

/*
 * This code is conditionally built please refer to the SoC cmake file and
 * is not built normally. If this is not built then timer5 is available
 * for other uses. Timer5 chip level DTS has a compatible for the XEC
 * counter driver and status = disabled. If it is enabled then the
 * counter driver will take over timer5 and it can't be used here.
 */
#define XEC_BTMR5_NODE DT_NODELABEL(timer5)
#define XEC_BTMR5_EN DT_NODE_HAS_STATUS_OKAY(XEC_BTMR5_NODE)
#define XEC_BTRM5_HAS_COMPAT DT_NODE_HAS_PROP(XEC_BTMR5_NODE, compatible)
#define XEC_BTMR5_IN_USE (XEC_BTMR5_EN && XEC_BTRM5_HAS_COMPAT)

/* check has status okay and has compatible property */
BUILD_ASSERT(!XEC_BTMR5_IN_USE,
	"BUILD ERROR: When SOC_HAS_TIMING_FUNCTIONS=y timer5 node must not be used by a driver");

#define BTMR_XEC_REG_BASE MCHP_BTMR_BASE_ADDR(5)

void soc_timing_init(void)
{
	struct btmr_regs *regs = (struct btmr_regs *)BTMR_XEC_REG_BASE;

	/* Setup counter */
	regs->CTRL = MCHP_BTMR_CTRL_ENABLE | MCHP_BTMR_CTRL_AUTO_RESTART |
		     MCHP_BTMR_CTRL_COUNT_UP; /* prescale=0 runs at full speed */

	regs->PRLD = 0; /* Preload */
	regs->CNT = 0; /* Counter value */

	regs->IEN = 0; /* Disable interrupt */
	regs->STS = 1; /* Clear interrupt */
}

void soc_timing_start(void)
{
	struct btmr_regs *regs = (struct btmr_regs *)BTMR_XEC_REG_BASE;

	regs->CTRL |= MCHP_BTMR_CTRL_START;
}

void soc_timing_stop(void)
{
	struct btmr_regs *regs = (struct btmr_regs *)BTMR_XEC_REG_BASE;

	regs->CTRL &= ~MCHP_BTMR_CTRL_START;
}

timing_t soc_timing_counter_get(void)
{
	struct btmr_regs *regs = (struct btmr_regs *)BTMR_XEC_REG_BASE;

	return (timing_t)(regs->CNT);
}

uint64_t soc_timing_cycles_get(volatile timing_t *const start,
			       volatile timing_t *const end)
{
	return *end - *start;
}

/* The basic timers input frequency is MCHP_BTMR_BASE_FREQ.
 * Timer frequency is MCHP_BTMR_BASE_FREQ / (pre-scaler + 1)
 * The above configuration sets pre-scaler = 0.
 */
uint64_t soc_timing_freq_get(void)
{
	return (MCHP_BTMR_BASE_FREQ);
}

uint64_t soc_timing_cycles_to_ns(uint64_t cycles)
{
	return cycles * NSEC_PER_SEC / MCHP_BTMR_BASE_FREQ;
}

uint64_t soc_timing_cycles_to_ns_avg(uint64_t cycles, uint32_t count)
{
	return (uint32_t)soc_timing_cycles_to_ns(cycles) / count;
}

uint32_t soc_timing_freq_get_mhz(void)
{
	return (uint32_t)(soc_timing_freq_get() / (uint32_t)MHZ(1));
}
