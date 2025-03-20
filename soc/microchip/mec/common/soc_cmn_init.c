/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <mec_ecia_api.h>
#include <mec_ecs_api.h>

static void mec5_soc_init_debug_interface(void)
{
	uint8_t etm_en = 0;
	enum mec_debug_mode dbg_mode = MEC_DEBUG_MODE_DISABLE;

#if defined(CONFIG_SOC_MEC_DEBUG_WITHOUT_TRACING)
	dbg_mode = MEC_DEBUG_MODE_SWD;
#endif

#if defined(CONFIG_SOC_MEC_DEBUG_AND_TRACING)
	dbg_mode = MEC_DEBUG_MODE_SWD_SWV;
#if defined(CONFIG_SOC_MEC_DEBUG_AND_ETM_TRACING)
	dbg_mode = MEC_DEBUG_MODE_SWD;
	etm_en = 1u;
#endif
#endif

	mec_hal_ecs_etm_pins(etm_en);
	mec_hal_ecs_debug_port(dbg_mode);
}

int mec5_soc_common_init(void)
{
	mec5_soc_init_debug_interface();
	mec_hal_ecia_init(MEC5_ECIA_DIRECT_BITMAP, 1, 0);

	return 0;
}
