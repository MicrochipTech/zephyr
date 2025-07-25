/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/__assert.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <soc_cmn_init.h>

#define MEC_ECS_BASE_ADDR		DT_REG_ADDR(DT_NODELABEL(ecs))
#define MEC_ECS_GPB_PWR_OFS		0x64u
#define MEC_ECS_GPB_PWR_ADDR		(MEC_ECS_BASE_ADDR + MEC_ECS_GPB_PWR_OFS)
#define MEC_ECS_GPB_PWR_VTR3_18_POS	2

void soc_early_init_hook(void)
{
	/* Configure GPIO bank before usage
	 * VTR1 is not configurable
	 * VTR2 doesn't need configuration if setting VTR2_STRAP
	 */
#ifdef CONFIG_SOC_MEC1501_VTR3_1_8V
	sys_set_bit(MEC_ECS_GPB_PWR_ADDR, MEC_ECS_GPB_PWR_VTR3_18_POS);
#endif

	mec_soc_common_init();
}
