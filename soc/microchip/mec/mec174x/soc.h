/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SOC_MICROCHIP_MEC_MEC174X_SOC_H
#define __SOC_MICROCHIP_MEC_MEC174X_SOC_H

#define SYSCLK_DEFAULT_IOSC_HZ MHZ(96)

#ifndef _ASMLANGUAGE

#define __FPU_PRESENT           CONFIG_CPU_HAS_FPU
#define __MPU_PRESENT           CONFIG_CPU_HAS_ARM_MPU

#define __CM4_REV               0x0201

#define __VTOR_PRESENT          1
#define __NVIC_PRIO_BITS        3
#define __Vendor_SysTickConfig  0
#define __FPU_DP                0
#define __ICACHE_PRESENT        0
#define __DCACHE_PRESENT        0
#define __DTCM_PRESENT          0

typedef enum {
	/* ARM Cortex-M4 Specific Interrupt Numbers */
	Reset_IRQn              = -15,
	NonMaskableInt_IRQn     = -14,
	HardFault_IRQn          = -13,
	MemoryManagement_IRQn   = -12,
	BusFault_IRQn           = -11,
	UsageFault_IRQn         = -10,
	SVCall_IRQn             = -5,
	DebugMonitor_IRQn       = -4,
	PendSV_IRQn             = -2,
	SysTick_IRQn            = -1,
	FIRST_PERIPH_IRQn       = 0,
	MAX_IRQn = CONFIG_NUM_IRQS
} IRQn_Type;

#include <core_cm4.h>
#include <soc_common.h>

#endif
#endif
