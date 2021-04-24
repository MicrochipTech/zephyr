/*
 * Copyright (c) 2019 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DEVICE_POWER_H
#define __DEVICE_POWER_H

#ifndef _ASMLANGUAGE

/* Enable SoC control over peripherals only when drivers do not support
 * power management
 */
#ifndef CONFIG_DEVICE_POWER_MANAGEMENT

/* Comment out to use JTAG without interruptions.
 * Beware this blocks PLL going off, hence should be enabled
 * for power consumption measurements
 * Note: To attach JTAG for any debug need to be performed with breakpoint
 * prior to deep sleep entry.
 */
#define DEEP_SLEEP_JTAG

/*
 * Enabling this would take a snapshot from clock requested values,
 * these can be dump on exit to identify which HW block is blocking.
 */
#define DEEP_SLEEP_CLK_REQ_DUMP

/*
 * Some peripherals if enabled always assert their CLK_REQ bits.
 * For example, any peripheral with a clock generator such as
 * timers, counters, UART, etc. We save the enables for these
 * peripherals, disable them, and restore the enabled state upon
 * wake.
 */
#define DEEP_SLEEP_PERIPH_SAVE_RESTORE


#define NUM_DS_TIMER_ENTRIES 4

struct ds_timer_info {
	uintptr_t addr;
	uint32_t restore_mask;
};
struct ds_dev_info {
	uint32_t ecs[2];
	uint32_t timers[NUM_DS_TIMER_ENTRIES];
	uint8_t uart_info[3];
};

#endif /* CONFIG_DEVICE_POWER_MANAGEMENT */

void soc_lite_sleep_enable(void);

void soc_deep_sleep_enable(void);
void soc_deep_sleep_disable(void);
void soc_deep_sleep_periph_save(void);
void soc_deep_sleep_periph_restore(void);
void soc_deep_sleep_wait_clk_idle(void);
void soc_deep_sleep_non_wake_en(void);
void soc_deep_sleep_non_wake_dis(void);
void soc_deep_sleep_wake_en(void);
void soc_deep_sleep_wake_dis(void);

#endif

#endif
