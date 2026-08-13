/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_xec_i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>

#include "xec_vci.h"

LOG_MODULE_DECLARE(app, CONFIG_LOG_DEFAULT_LEVEL);

/* ---- PM Notifier ---- */

struct app_i2c_info {
	uintptr_t reg_base;
	bool is_wake_device;
	uint8_t girq_wake;
	uint8_t girq_wake_pos;
};

struct app_uart_info {
	uintptr_t reg_base;
};

struct app_btmr_info {
	uintptr_t reg_base;
};

/* XEC Basic timers do not obey their PCR SLP_EN signal and automatically clear their CLK_REQ
 * signal. Software must save/clear/restore each basic timer's enable bit in the control register.
 * We could iterate over DT compatible but that assumes the application is using the known
 * drivers. If the application is using the HW directly and leaves it enabled then the PCR HW
 * will not disable the PLL in deep sleep.
 * There are two drivers we must check:
 * Zephyr kernel timer driver: microchip_xec_rtos_timer
 *   This driver uses one of the 32-bit basic timers for k_busy_wait. The basic timer is specified
 *   by phandle property busy_wait_timer.
 *
 * Zephyr counter driver: microchip_xec_timer
 */
#define XEC_BASIC_TMR_CR_OFS    0x10U
#define XEC_BASIC_TMR_CR_EN_POS 0

#define XEC_BTMR_COUNTER_CNT DT_NUM_INST_STATUS_OKAY(microchip_xec_timer)
#define XEC_BTMR_KT_CNT      DT_NUM_INST_STATUS_OKAY(microchip_xec_rtos_timer)

#define XEC_BTMR_TOTAL_CNT (XEC_BTMR_COUNTER_CNT + XEC_BTMR_KT_CNT)

#define XEC_BT_APP_INFO(node_id) { .reg_base = (uintptr_t)DT_REG_ADDR(node_id), },

#define XEC_RT_BWT_INFO(rt_node_id) \
	{ .reg_base = (uintptr_t)DT_REG_ADDR(DT_PHANDLE(rt_node_id, busy_wait_timer)), },

#if 0
const struct app_btmr_info app_btmr_info_table[] = {
	DT_FOREACH_STATUS_OKAY(microchip_xec_timer, XEC_BT_APP_INFO)
	DT_FOREACH_STATUS_OKAY(microchip_xec_rtos_timer, XEC_RT_BWT_INFO)
};
#else
const struct app_btmr_info app_btmr_info_table[] = {
	DT_FOREACH_STATUS_OKAY(microchip_xec_timer, XEC_BT_APP_INFO)
};
#endif

static uint8_t app_btmr_pm_save[XEC_BTMR_TOTAL_CNT];

static void xec_btmrs_pm_deep_sleep(bool enter_ds)
{
	for (uint32_t i = 0; i < ARRAY_SIZE(app_btmr_info_table); i++) {
		const struct app_btmr_info *p = &app_btmr_info_table[i];
		uint32_t cr = sys_read32(p->reg_base + XEC_BASIC_TMR_CR_OFS);

		if (enter_ds) {
			app_btmr_pm_save[i] = (uint8_t)(cr & BIT(XEC_BASIC_TMR_CR_EN_POS));
			sys_write32(cr & (uint32_t)~BIT(XEC_BASIC_TMR_CR_EN_POS),
				    p->reg_base + XEC_BASIC_TMR_CR_OFS);
		} else {
			cr |= (app_btmr_pm_save[i] & BIT(XEC_BASIC_TMR_CR_EN_POS));
			sys_write32(cr, p->reg_base + XEC_BASIC_TMR_CR_OFS);
		}
	}
}

#define XEC_I2C_V3_BM_CNT DT_NUM_INST_STATUS_OKAY(microchip_xec_i2c_v3_bm)
#define XEC_I2C_V3_NL_CNT DT_NUM_INST_STATUS_OKAY(microchip_xec_i2c_v3_nl)

#define XEC_I2C_V3_CNT (XEC_I2C_V3_BM_CNT + XEC_I2C_V3_NL_CNT)

#define XEC_I2C_GIRQ(nid) MCHP_XEC_ECIA_GIRQ(DT_PROP_BY_IDX(nid, girqs, 1))

#define XEC_I2C_GIRQ_POS(nid) MCHP_XEC_ECIA_GIRQ_POS(DT_PROP_BY_IDX(nid, girqs, 1))

#define XEC_I2C_V3_BM_APP_INFO(nid) \
	{ .reg_base = DT_REG_ADDR(nid), \
	  .is_wake_device = DT_PROP(nid, wakeup_source), \
	  .girq_wake = XEC_I2C_GIRQ(nid), \
	  .girq_wake_pos = XEC_I2C_GIRQ_POS(nid), \
	},

#define XEC_I2C_V3_NL_APP_INFO(nid) \
	{ .reg_base = DT_REG_ADDR(nid), \
	  .is_wake_device = DT_PROP(nid, wakeup_source), \
	  .girq_wake = XEC_I2C_GIRQ(nid), \
	  .girq_wake_pos = XEC_I2C_GIRQ_POS(nid), \
	},

const struct app_i2c_info app_i2c_info_table[] = {
	DT_FOREACH_STATUS_OKAY(microchip_xec_i2c_v3_bm, XEC_I2C_V3_BM_APP_INFO)
	DT_FOREACH_STATUS_OKAY(microchip_xec_i2c_v3_nl, XEC_I2C_V3_NL_APP_INFO)
};

static uint8_t app_i2c_save[XEC_I2C_V3_CNT];

static void xec_i2c_pm_deep_sleep(bool enter_ds)
{
	for (uint32_t n = 0; n < ARRAY_SIZE(app_i2c_info_table); n++) {
		const struct app_i2c_info *p = &app_i2c_info_table[n];

		if (p->is_wake_device) {
			if (enter_ds) {
				soc_i2c_wake_prepare(p->reg_base, p->girq_wake, p->girq_wake_pos);
			} else {
				soc_i2c_wake_clear(p->reg_base, p->girq_wake, p->girq_wake_pos);
			}
		} else {
			if (enter_ds) {
				app_i2c_save[n] = soc_i2c_get_enable(p->reg_base);
				soc_i2c_set_enable(p->reg_base, 0);
			} else {
				soc_i2c_set_enable(p->reg_base, app_i2c_save[n]);
			}
		}
	}
}

/* --- garbage eSPI v2 driver ----
 * If eSPI enabled we configure its PLL wake GIRQ22 on deep sleep entry.
 */
#define XEC_ESPI_NODE DT_NODELABEL(espi0)

#define XEC_ESPI_WK_GIRQ(node_id)     MCHP_XEC_ECIA_GIRQ_POS(DT_PROP_BY_IDX(node_id, girqs, 11))
#define XEC_ESPI_WK_GIRQ_POS(node_id) MCHP_XEC_ECIA_GIRQ_POS(DT_PROP_BY_IDX(node_id, girqs, 11))

static void xec_espi_pm_deep_sleep(bool enter_ds)
{
#if DT_HAS_COMPAT_STATUS_OKAY(microchip_xec_espi_v2)
	uint8_t girq_wake = XEC_ESPI_WK_GIRQ(XEC_ESPI_NODE);
	uint8_t girq_wake_pos = XEC_ESPI_WK_GIRQ_POS(XEC_ESPI_NODE);
	uint8_t enable = (enter_ds) ? 1U : 0;

	soc_ecia_girq_status_clear(girq_wake, girq_wake_pos);
	soc_ecia_girq_ctrl(girq_wake, girq_wake_pos, enable);
#endif
}

/* --- UART ---
 * XEC UART does not obey PCR SLP_EN assertion. If the UART is enabled it asserts its CLK_REQ
 * signal. We must clear the activate bit in each UART the application is using.
 */
#define XEC_UART_ACTIVATE_REG_OFS 0x330U

#define XEC_UART_APP_CNT DT_NUM_INST_STATUS_OKAY(microchip_xec_uart)

#define XEC_UART_APP_INFO(node_id) { .reg_base = (uintptr_t)DT_REG_ADDR(node_id), },

const struct app_uart_info app_uart_info_table[] = {
	DT_FOREACH_STATUS_OKAY(microchip_xec_uart, XEC_UART_APP_INFO)
};

static uint8_t app_uart_pm_save[XEC_UART_APP_CNT];

static void xec_uart_pm_deep_sleep(bool enter_ds)
{
	for (uint32_t i = 0; i < ARRAY_SIZE(app_uart_info_table); i++) {
		const struct app_uart_info *p = &app_uart_info_table[i];

		if (enter_ds) {
			app_uart_pm_save[i] = sys_read8(p->reg_base + XEC_UART_ACTIVATE_REG_OFS);
			sys_write8(0, p->reg_base + XEC_UART_ACTIVATE_REG_OFS);
		} else {
			sys_write8(app_uart_pm_save[i], p->reg_base + XEC_UART_ACTIVATE_REG_OFS);
		}
	}
}

static void xec_i3c_pm_deep_sleep(bool enter_ds)
{
	/* TODO when I3C driver is available */
}

static void xec_spi_target_pm_deep_sleep(bool enter_ds)
{
	/* TODO when SPI target driver is available */
}

/* Interrupts are locked when PM subsystem calls this */
static void app_pm_entry(enum pm_state state_to_enter)
{
	if (state_to_enter == PM_STATE_SUSPEND_TO_RAM) { /* enter deep sleep */
		xec_btmrs_pm_deep_sleep(true);
		xec_i2c_pm_deep_sleep(true);
		xec_espi_pm_deep_sleep(true);
		xec_i3c_pm_deep_sleep(true);
		xec_spi_target_pm_deep_sleep(true);
		xec_uart_pm_deep_sleep(true);
	}
}

static void app_pm_exit(enum pm_state state_exit_from)
{
	if (state_exit_from == PM_STATE_SUSPEND_TO_RAM) { /* exit deep sleep */
		xec_btmrs_pm_deep_sleep(false);
		xec_i2c_pm_deep_sleep(false);
		xec_espi_pm_deep_sleep(false);
		xec_i3c_pm_deep_sleep(false);
		xec_spi_target_pm_deep_sleep(false);
		xec_uart_pm_deep_sleep(false);
	}
}

static struct pm_notifier app_pm_notifier = {
	.state_entry = app_pm_entry,
	.state_exit = app_pm_exit,
	.report_substate = false,
};

void app_register_pm_notifier(void)
{
	LOG_INF("Register App PM notifier");
	pm_notifier_register(&app_pm_notifier);
}
