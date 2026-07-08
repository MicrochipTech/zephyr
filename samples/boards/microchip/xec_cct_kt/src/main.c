/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/pinctrl.h>

#include "xec_vci.h"

#define SLEEP_MS   3000U
#define ITERATIONS 0

#define XEC_VBAT_MEM_BASE  DT_REG_ADDR(DT_NODELABEL(bbram))
#define XEC_ECS_BASE       DT_REG_ADDR(DT_NODELABEL(ecs))
#define XEC_ECS_DBG_CR_OFS 0x20U

/* Test-clock-out pad (GPIO060, AF2). Not owned by a device driver, so define
 * its pin-control config here and apply it manually from main().
 */
#define APP_PINS_NODE DT_NODELABEL(app_pin_list)
PINCTRL_DT_DEFINE(APP_PINS_NODE);
static const struct pinctrl_dev_config *app_pins_pcfg = PINCTRL_DT_DEV_CONFIG_GET(APP_PINS_NODE);

/*
 * Exercise the CCT kernel timer across tickless sleeps. Each iteration sleeps
 * long enough to let the PM policy enter a low-power state (the board's
 * suspend-to-ram residency is 2 s), which hands timekeeping to the 32 KHz RTOS
 * timer companion. If both the system timer and the LPM hooks are correct, the
 * uptime delta and the free-running cycle-counter delta should both track the
 * requested sleep time.
 */
int main(void)
{
	uint32_t i = 0, r = 0;
	int ret = 0;

	printk("XEC CCT kernel timer sample\n");
	printk("system timer HW cycles/sec: %u\n", sys_clock_hw_cycles_per_sec());
	printk("ticks/sec: %d\n", CONFIG_SYS_CLOCK_TICKS_PER_SEC);

	/* Route the test clock to GPIO060 for scope measurement. */
	ret = pinctrl_apply_state(app_pins_pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		printk("app-pin-list pinctrl apply failed: %d\n", ret);
	} else {
		printk("test clock routed to GPIO060 (AF2)\n");
		printk("VCI_IN1 pin function enabled");
	}

	mchp_vci_in_input_enable(BIT(MEC_VCI_IN1_POS));
	mchp_vci_in_latch_enable(BIT(MEC_VCI_IN1_POS));
	mchp_vci_edge_detect_clr_all();
	mchp_vci_in_latch_reset(BIT(MEC_VCI_IN1_POS));

	printk("Disconnect Debugger and press switch S4 when ready");
	r = mchp_vci_pedge_detect() & mchp_vci_nedge_detect();
	while ((r & BIT(MEC_VCI_IN1_POS)) == 0) {
		r = mchp_vci_pedge_detect() & mchp_vci_nedge_detect();
	}

	printk("Button S4 pressed. Disabling Debug interface");
	sys_write32(0, XEC_ECS_BASE + XEC_ECS_DBG_CR_OFS);
	sys_write32(0, 0xE000EDFCU);
	sys_write32(0xA05F0000U, 0xE0000EDFU);
	sys_write32(0xC5ACCE55U, 0xE0000FB0U);
	sys_write32(0, 0xE0000FB0U);
	sys_write32(0, 0xE0000E80U);
	sys_write32(0, 0xE0040304U);
	sys_write32(0, 0xE000EDFCU);

	r = sys_read32(XEC_ECS_BASE + XEC_ECS_DBG_CR_OFS);
	printk("ECS reg 0x%08x = 0x%08x", (XEC_ECS_BASE + XEC_ECS_DBG_CR_OFS), r);

#if ITERATIONS == 0
	while (true) {
		i++;
#else
	for (i = 0; i < ITERATIONS; i++) {
#endif
		uint32_t cyc0 = k_cycle_get_32();
		int64_t up0 = k_uptime_get();

		k_msleep(SLEEP_MS);

		uint32_t cyc1 = k_cycle_get_32();
		int64_t up1 = k_uptime_get();

		/* Unsigned subtraction is wrap-correct for the 32-bit counter. */
		uint32_t dcyc = cyc1 - cyc0;

		printk("iter %u: requested %u ms, uptime +%lld ms, "
		       "cycles +%u (%u ms by cycle count)\n",
		       i, SLEEP_MS, (long long)(up1 - up0), dcyc, k_cyc_to_ms_near32(dcyc));

		r = sys_read32(XEC_VBAT_MEM_BASE);
		printk("PCR CLK_REQ0 = 0x%08x", r);
		r = sys_read32(XEC_VBAT_MEM_BASE + 4U);
		printk("PCR CLK_REQ1 = 0x%08x", r);
		r = sys_read32(XEC_VBAT_MEM_BASE + 8U);
		printk("PCR CLK_REQ2 = 0x%08x", r);
		r = sys_read32(XEC_VBAT_MEM_BASE + 0xCU);
		printk("PCR CLK_REQ3 = 0x%08x", r);
		r = sys_read32(XEC_VBAT_MEM_BASE + 0x10U);
		printk("PCR CLK_REQ4 = 0x%08x", r);
	}

	printk("Sample complete\n");

	return 0;
}
