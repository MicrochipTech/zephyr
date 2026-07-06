/*
 * (c) 2026 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_MICROCHIP_MR_MEC176X_SOC_H_
#define SOC_MICROCHIP_MR_MEC176X_SOC_H_

#ifndef _ASMLANGUAGE

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

/*
 * ARM Cortex-M55 CMSIS feature configuration for Mount Rainier (DOS section 4.10).
 * core_cm55.h supplies safe #ifndef defaults (FPU/MPU/DSP/cache = 0, prio bits = 3,
 * VTOR = 1); we only override what differs from those defaults.
 *
 *   M55 + Helium (MVE int+float) + single-precision FPU, 16-region MPU, no cache.
 *   TrustZone/SAU exists in HW but is left disabled for this plain-secure bring-up,
 *   so __SAUREGION_PRESENT stays at the default 0.
 */
#define __FPU_PRESENT  CONFIG_CPU_HAS_FPU      /* single precision (FPU_DP stays 0) */
#define __MPU_PRESENT  CONFIG_CPU_HAS_ARM_MPU
#define __DSP_PRESENT  1U                      /* Helium prerequisite (ARMV8_M_DSP) */
#define __MVE_PRESENT  1U                      /* Helium integer MVE                */
#define __MVE_FP       1U                      /* Helium floating-point MVE         */
#define __NVIC_PRIO_BITS 3U                    /* 8 priority levels                 */

/**
 * @brief Cortex-M55 system exception numbers.
 *
 * CMSIS NVIC inline helpers in core_cm55.h take an IRQn_Type, so the enum must be
 * visible before that header is included. External (peripheral) interrupts use raw
 * NVIC numbers via Zephyr's IRQ_CONNECT and are not enumerated here. The Mount
 * Rainier GIRQ -> NVIC map (Mount_Rainier_GIRQ_mapping.xlsx) tops out at NVIC 197.
 */
typedef enum {
	Reset_IRQn            = -15,
	NonMaskableInt_IRQn   = -14,
	HardFault_IRQn        = -13,
	MemoryManagement_IRQn = -12,
	BusFault_IRQn         = -11,
	UsageFault_IRQn       = -10,
	SecureFault_IRQn      = -9,   /* M55 with TrustZone */
	SVCall_IRQn           = -5,
	DebugMonitor_IRQn     = -4,
	PendSV_IRQn           = -2,
	SysTick_IRQn          = -1,
} IRQn_Type;

#include <core_cm55.h>

/*
 * XEC SoC helpers + peripheral register headers reused from the upstream MEC common
 * tree (compiled by reference, see this series' CMakeLists.txt). soc_common.h pulls
 * in everything the microchip,xec-uart and XEC pinctrl drivers expect from <soc.h>:
 * the MMCR bit ops, PCR clock-gate ungate, ECIA/GIRQ control, pinctrl_soc.h, and the
 * UART/GPIO register offset macros. All of these live in common/ (not series-specific).
 */
#include <soc_common.h>

#endif /* _ASMLANGUAGE */

#endif /* SOC_MICROCHIP_MR_MEC176X_SOC_H_ */
