/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm/arch.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>

#define MEC_ECS_BASE_ADDR		DT_REG_ADDR(DT_NODELABEL(ecs))
#define MEC_ECS_ETM_CR_OFS		0x14u
#define MEC_ECS_ETM_CR_ADDR		(MEC_ECS_BASE_ADDR + MEC_ECS_ETM_CR_OFS)
#define MEC_ECS_ETM_CR_EN_POS		0

#define MEC_ECS_DCR_OFS			0x20u
#define MEC_ECS_DCR_ADDR		(MEC_ECS_BASE_ADDR + MEC_ECS_DCR_OFS)
#define MEC_ECS_DCR_EN_POS		0
#define MEC_ECS_DCR_MODE_POS		1
#define MEC_ECS_DCR_MODE_MSK		GENMASK(2, 1)
#define MEC_ECS_DCR_MODE_JTAG		0
#define MEC_ECS_DCR_MODE_SWD_SWV	1u
#define MEC_ECS_DCR_MODE_SWD		2u
#define MEC_ECS_DCR_MODE_SET(m)		FIELD_PREP(MEC_ECS_DCR_MODE_MSK, (m))
#define MEC_ECS_DCR_MODE_GET(m)		FIELD_GET(MEC_ECS_DCR_MODE_MSK, (m))
#define MEC_ECS_DCR_PU_EN_POS		3
#define MEC_ECS_DCR_LOCKED_POS		5 /* disable and lock */

/* NOTE hardware supports SWD, SWV, and ETM enabled at the same time. */

#define MEC_DBG_FLAG_EN			BIT(0)
#define MEC_DBG_FLAG_SWD		BIT(1)
#define MEC_DBG_FLAG_SWV		BIT(2)
#define MEC_DBG_FLAG_PU			BIT(3)
#define MEC_DBG_FLAG_ETM		BIT(4)
#define MEC_DBG_FLAG_LOCK		BIT(7)

#if defined(CONFIG_SOC_MEC_DEBUG_DISABLED)
#define MEC_DBG_FLAGS_VAL 0
#endif

#if defined(CONFIG_SOC_MEC_DEBUG_WITHOUT_TRACING)
#define MEC_DBG_FLAGS_VAL (MEC_DBG_FLAG_EN | MEC_DBG_FLAG_SWD)
#endif

#if defined(CONFIG_SOC_MEC_DEBUG_AND_TRACING)
#if defined(CONFIG_SOC_MEC_DEBUG_AND_ETM_TRACING)
#define MEC_DBG_FLAGS_VAL (MEC_DBG_FLAG_EN | MEC_DBG_FLAG_SWD | MEC_DBG_FLAG_ETM)
#endif
#if defined(CONFIG_SOC_MEC_DEBUG_AND_SWV_TRACING)
#define MEC_DBG_FLAGS_VAL (MEC_DBG_FLAG_EN | MEC_DBG_FLAG_SWD | MEC_DBG_FLAG_SWV)
#endif
#endif

#if defined(MEC_DBG_FLAGS_VAL)
static void mec_soc_debug_config(uint32_t flags)
{
	uint32_t mode = 0, r = 0;

	if ((flags & MEC_DBG_FLAG_EN) == 0) {
		sys_clear_bit(MEC_ECS_ETM_CR_ADDR, MEC_ECS_ETM_CR_EN_POS);
		sys_clear_bit(MEC_ECS_DCR_ADDR, MEC_ECS_DCR_EN_POS);
		if ((flags & MEC_DBG_FLAG_LOCK) != 0) {
			sys_set_bit(MEC_ECS_DCR_ADDR, MEC_ECS_DCR_LOCKED_POS);
		}
		return;
	}

	if ((flags & MEC_DBG_FLAG_ETM) != 0) {
		sys_set_bit(MEC_ECS_ETM_CR_ADDR, MEC_ECS_ETM_CR_EN_POS);
	}

	if ((flags & MEC_DBG_FLAG_SWD) != 0) {
		mode = MEC_ECS_DCR_MODE_SET(MEC_ECS_DCR_MODE_SWD);
		if ((flags & MEC_DBG_FLAG_SWV) != 0) {
			mode = MEC_ECS_DCR_MODE_SET(MEC_ECS_DCR_MODE_SWD_SWV);
		}
	} else { /* JTAG mode */
		mode = MEC_ECS_DCR_MODE_SET(MEC_ECS_DCR_MODE_JTAG);
	}

	r = sys_read32(MEC_ECS_DCR_ADDR);
	r &= (uint32_t)~MEC_ECS_DCR_MODE_MSK;

	if ((flags & MEC_DBG_FLAG_PU) != 0) {
		r |= BIT(MEC_ECS_DCR_PU_EN_POS);
	}

	r |= (mode | BIT(MEC_ECS_DCR_EN_POS));

	sys_write32(r, MEC_ECS_DCR_ADDR);
}
#endif /* defined(MEC_DBG_FLAGS_VAL) */

int mec_soc_common_init(void)
{
#if defined(MEC_DBG_FLAGS_VAL)
	mec_soc_debug_config(MEC_DBG_FLAGS_VAL);
#endif
	soc_ecia_init(MCHP_MEC_ECIA_GIRQ_AGGR_ONLY_BM, MCHP_MEC_ECIA_GIRQ_DIRECT_CAP_BM, 0);

	return 0;
}
