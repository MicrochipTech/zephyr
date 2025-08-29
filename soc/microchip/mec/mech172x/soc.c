/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <soc_cmn_init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/mchp_xec_clock_control.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#ifdef CONFIG_SOC_PREP_HOOK
/* Invoked before prep code has initialized global data */
void soc_prep_hook(void)
{
	mec5_soc_common_init();
}
#endif

#ifdef CONFIG_SOC_EARLY_INIT_HOOK
/* Invoked before kernal and driver initialized. Global data has been initialized */
void soc_early_init_hook(void)
{
	z_mchp_xec_pcr_vb_pll_init();
}
#endif