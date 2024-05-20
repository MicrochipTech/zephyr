/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/pm/pm.h>

#include "soc_device_power.h"

/* HAL */
#include <mec_adc_api.h>
#include <mec_btimer_api.h>
#include <mec_ecia_api.h>
#include <mec_ecs_api.h>
#include <mec_kscan_api.h>
#include <mec_pcr_api.h>
#include <mec_ps2_api.h>
#include <mec_uart_api.h>

#if defined(CONFIG_SOC_MEC_PM_DEBUG_DEEP_SLEEP_CLK_REQ)
#define SOC_VBAT_MEM_PM_OFS CONFIG_SOC_MEC_PM_DEBUG_DS_CLK_REQ_VBAT_MEM_OFS

void soc_debug_sleep_clk_req(void)
{
	mec_hal_pcr_save_clk_req_to_vbatm(SOC_VBAT_MEM_PM_OFS);
}
#endif

/*
 * Allow peripherals connected to external masters to wake the PLL but not
 * the EC. Once the peripheral has serviced the external master the PLL
 * will be turned back off. For example, if the eSPI master requests eSPI
 * configuration information or state of virtual wires the EC doesn't need
 * to be involved. The hardware can power on the PLL long enough to service
 * the request and then turn the PLL back off.  The SMBus and I2C peripherals
 * in slave mode can also make use of this feature.
 */
void soc_deep_sleep_non_wake_en(void)
{
#if !defined(CONFIG_PM_DEVICE)
	mec_hal_girq_bm_en(MEC_GIRQ22_ID, UINT32_MAX, 1);
#endif
}

void soc_deep_sleep_non_wake_dis(void)
{
#if !defined(CONFIG_PM_DEVICE)
	mec_hal_girq_bm_en(MEC_GIRQ22_ID, UINT32_MAX, 0);
	mec_hal_girq_bm_clr_src(MEC_GIRQ22_ID, UINT32_MAX);
#endif
}

/* These device have extra wake features
 * HAL only enables wake features if the peripheral is enabled.
 */
void soc_deep_sleep_wake_en(void)
{
#if !defined(CONFIG_PM_DEVICE)
	mec_hal_kscan_wake_enable(1);
	mec_hal_ps2_wake_enables(1);
#endif
}

void soc_deep_sleep_wake_dis(void)
{
#if !defined(CONFIG_PM_DEVICE)
	mec_hal_kscan_wake_enable(1);
	mec_hal_ps2_wake_enables(1);
#endif
}

void soc_deep_sleep_periph_save(void)
{
#if defined(CONFIG_RTOS_TIMER)
	soc_ktimer_pm_entry(true);
#endif
#if defined(CONFIG_SOC_MEC_PM_DS_PERIPH_SAVE_RESTORE)
	mec_hal_adc_pm_save_disable();
	mec_hal_btimer_pm_save_disable();
	mec_hal_uart_pm_save_disable();
#endif
}

void soc_deep_sleep_periph_restore(void)
{
#if defined(CONFIG_SOC_MEC_PM_DS_PERIPH_SAVE_RESTORE)
	mec_hal_uart_pm_restore();
	mec_hal_btimer_pm_restore();
	mec_hal_adc_pm_restore();
#endif
#if defined(CONFIG_RTOS_TIMER)
	soc_ktimer_pm_exit(true);
#endif
}
