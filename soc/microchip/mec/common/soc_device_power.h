/*
 * Copyright (c) 2019 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DEVICE_POWER_H
#define __DEVICE_POWER_H

#ifndef _ASMLANGUAGE

#include <stdbool.h>
#include <stdint.h>

void soc_deep_sleep_periph_save(void);
void soc_deep_sleep_periph_restore(void);
void soc_deep_sleep_non_wake_en(void);
void soc_deep_sleep_non_wake_dis(void);
void soc_deep_sleep_wake_en(void);
void soc_deep_sleep_wake_dis(void);

#ifdef CONFIG_SOC_MEC_PM_DEBUG_DEEP_SLEEP_CLK_REQ
void soc_debug_sleep_clk_req(void);
#endif

/* MEC ktimer timer driver */
void soc_ktimer_pm_entry(bool is_deep_sleep);
void soc_ktimer_pm_exit(bool is_deep_sleep);

#endif /* _ASMLANGUAGE */
#endif /* __DEVICE_POWER_H */
