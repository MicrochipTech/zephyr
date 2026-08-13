/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_MICROCHIP_MEC_COMMON_SOC_PM_H_
#define _SOC_MICROCHIP_MEC_COMMON_SOC_PM_H_

#include <stdint.h>

#ifdef CONFIG_PM
void soc_i2c_wake_prepare(uintptr_t i2c_reg_base, uint8_t girq, uint8_t girq_pos);
void soc_i2c_wake_clear(uintptr_t i2c_reg_base, uint8_t girq, uint8_t girq_pos);
uint8_t soc_i2c_get_enable(uintptr_t i2c_reg_base);
void soc_i2c_set_enable(uintptr_t i2c_reg_base, uint8_t enable);
#endif

#ifdef CONFIG_ARCH_HAS_CUSTOM_BUSY_WAIT
uint8_t mchp_xec_rtimer_busy_wait_off();
uint8_t mchp_xec_rtimer_busy_wait_on();
#endif

#endif /* _SOC_MICROCHIP_MEC_COMMON_SOC_PM_H_ */
