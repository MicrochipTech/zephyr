/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <soc.h>

#define XEC_ECS_REG_BASE DT_REG_ADDR(DT_NODELABEL(ecs))

static void soc_init_debug_interface(void)
{
	uint32_t raddr = XEC_ECS_REG_BASE;
	uint32_t msk = MCHP_XEC_ECS_DBG_CR_CFG_MSK, r = 0, v = 0;

#if defined(CONFIG_SOC_XEC_DEBUG_ETM_DISABLED)
	sys_clear_bit(raddr + MCHP_XEC_ECS_ETM_CR_OFS, MCHP_XEC_ECS_ETM_CR_EN_POS);
#else
	sys_set_bit(raddr + MCHP_XEC_ECS_ETM_CR_OFS, MCHP_XEC_ECS_ETM_CR_EN_POS);
#endif

#if defined(CONFIG_SOC_XEC_DEBUG_DISABLED)
	sys_clear_bit(raddr + MCHP_XEC_ECS_DBG_CR_OFS, MCHP_XEC_ECS_DBG_CR_EN_POS);
#elif defined(CONFIG_SOC_XEC_DEBUG_MODE_JTAG)
	v = MCHP_XEC_ECS_DBG_CR_SET(MCHP_XEC_ECS_DBG_CR_JTAG);
#elif defined(CONFIG_SOC_XEC_DEBUG_MODE_SWD)
	v = MCHP_XEC_ECS_DBG_CR_SET(MCHP_XEC_ECS_DBG_CR_SWD);
#elif defined(CONFIG_SOC_XEC_DEBUG_MODE_SWD_SWV)
	v = MCHP_XEC_ECS_DBG_CR_SET(MCHP_XEC_ECS_DBG_CR_SWD_SWV);
#else
	msk = 0;
#endif

	if (msk != 0) {
		r = sys_read32(raddr + MCHP_XEC_ECS_DBG_CR_OFS);
		r &= ~msk;
		r |= v;
		r |= BIT(MCHP_XEC_ECS_DBG_CR_EN_POS);
		sys_write32(r, raddr + MCHP_XEC_ECS_DBG_CR_OFS);
	}
}

int mec5_soc_common_init(void)
{
	soc_init_debug_interface();
	soc_ecia_init(MCHP_MEC_ECIA_GIRQ_AGGR_ONLY_BM, MCHP_MEC_ECIA_GIRQ_DIRECT_CAP_BM, 0);

	return 0;
}
