/*
 * Copyright (c) 2020 Intel Corporation.
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

/* MEC152x basic timers input source is 48MHz */
#define MEC152X_BTMR_BASE_FREQ MHZ(48)

void soc_timing_init(void)
{
	/* Setup counter */
	B32TMR1_REGS->CTRL = MCHP_BTMR_CTRL_ENABLE |
			     MCHP_BTMR_CTRL_AUTO_RESTART |
			     MCHP_BTMR_CTRL_COUNT_UP;

	B32TMR1_REGS->PRLD = 0; /* Preload */
	B32TMR1_REGS->CNT = 0; /* Counter value */

	B32TMR1_REGS->IEN = 0; /* Disable interrupt */
	B32TMR1_REGS->STS = 1; /* Clear interrupt */
}

void soc_timing_start(void)
{
	B32TMR1_REGS->CTRL |= MCHP_BTMR_CTRL_START;
}

void soc_timing_stop(void)
{
	B32TMR1_REGS->CTRL &= ~MCHP_BTMR_CTRL_START;
}

timing_t soc_timing_counter_get(void)
{
	return B32TMR1_REGS->CNT;
}

uint64_t soc_timing_cycles_get(volatile timing_t *const start,
			       volatile timing_t *const end)
{
	return (*end - *start);
}

/* The basic timers input frequency is MEC152X_BTMR_BASE_FREQ.
 * Timer frequency is MCHP_BTMR_BASE_FREQ / (pre-scaler + 1)
 * The above configuration sets pre-scaler = 0.
 */
uint64_t soc_timing_freq_get(void)
{
	return (uint64_t)MEC152X_BTMR_BASE_FREQ;
}

uint64_t soc_timing_cycles_to_ns(uint64_t cycles)
{
	return (cycles) * (NSEC_PER_SEC) / (uint32_t)(MEC152X_BTMR_BASE_FREQ);
}

uint64_t soc_timing_cycles_to_ns_avg(uint64_t cycles, uint32_t count)
{
	return (uint32_t)soc_timing_cycles_to_ns(cycles) / count;
}

uint32_t soc_timing_freq_get_mhz(void)
{
	return (uint32_t)(soc_timing_freq_get() / (uint32_t)MHZ(1));
}
