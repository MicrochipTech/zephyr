/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <pm/pm.h>
#include <sys/sys_io.h>
#include <soc.h>

static struct gpio_regs * const gpio = (struct gpio_regs * const)(
	DT_REG_ADDR(DT_NODELABEL(gpio_000_036)));
static struct ecs_regs * const ecs = (struct ecs_regs * const)(
	DT_REG_ADDR(DT_NODELABEL(ecs)));
static struct vci_regs * const vci = (struct vci_regs * const)(
	DT_REG_ADDR(DT_NODELABEL(vci0)));

void print_deep_sleep_debug(void);

void main(void)
{
	uint32_t temp = 0;
	bool jtag_enabled = false;

	const struct pm_state_info s_to_idle = {
		.state = PM_STATE_SUSPEND_TO_IDLE,
		.substate_id = 0,
		.min_residency_us = 1000000 / 10,
		.exit_latency_us = 1,
	};

	const struct pm_state_info s_to_ram = {
		.state = PM_STATE_SUSPEND_TO_RAM,
		.substate_id = 1,
		.min_residency_us = 2000000 / 10,
		.exit_latency_us = 3,
	};

	printk("Hello World! %s\n", CONFIG_BOARD);

	gpio->CTRL[MCHP_GPIO_0241_ID] = 0x0240U; /* LED4 OFF */

	if (ecs->DEBUG_CTRL != 0) {
		jtag_enabled = true;
	}

	/* MEC172x EVB GPIO162/VCI_IN1# is connected through a Schmitt trigger
	 * to button S4. Requires a jumper on J56 3-4 and no jumper on J55 3-4.
	 * We configure VCI block to filter and latch VCI_IN1#
	 */
	vci->CONFIG = 0;
	vci->PEDGE_DET = BIT(1);
	vci->NEDGE_DET = BIT(1);
	vci->INPUT_EN = ~BIT(1);
	vci->LATCH_EN = BIT(1);
	vci->LATCH_RST = BIT(1);
	vci->BUFFER_EN = BIT(1);
	temp = vci->CONFIG;
	printk("VCI inputs = 0x%02x\n", vci->CONFIG & 0x7Fu);

	printk("VCI_IN1 enabled. Press button S4 to disable JTAG and continue\n");

	do {
		temp = vci->PEDGE_DET & vci->NEDGE_DET & BIT(1);
	} while (!temp);

	vci->PEDGE_DET = BIT(1);
	vci->NEDGE_DET = BIT(1);
	vci->LATCH_RST = BIT(1);

	/* Kill JTAG */
	ecs->ETM_CTRL = 0U;
	ecs->DEBUG_CTRL = 0U;
	jtag_enabled = false;

	printk("JTAG disabled\n");

	for (int i = 0; i < 10; i++) {
		printk("Force Idle: %d\n", (i + 1));
		k_msleep(2000);

		bool bret = pm_state_force(0, &s_to_idle);

		k_msleep(1000);

		printk("Wake from Idle. bret = %d\n", bret);
	}

	printk("Try Suspend to RAM(deep sleep)\n");

	uint64_t count = 0;

	while (1) {
		count++;

		printk("Force Suspend to RAM(Deep Sleep): %llu\n", count);
		k_msleep(3000);

		bool bret = pm_state_force(0, &s_to_ram);

		k_msleep(3000);

		printk("Wake from Deep Sleep. bret = %d\n", bret);
		print_deep_sleep_debug();

		temp = vci->PEDGE_DET & vci->NEDGE_DET & BIT(1);

		if (temp) {
			vci->PEDGE_DET = BIT(1);
			vci->NEDGE_DET = BIT(1);
			vci->LATCH_RST = BIT(1);
			printk("VCI_IN1 detected (button S4): ");
			if (jtag_enabled) {
				ecs->DEBUG_CTRL = 0U;
				printk("JTAG Disabled\n");
			} else {
				ecs->DEBUG_CTRL = 0x3U;
				printk("JTAG Enabled\n");
			}
		}
	}
}

void print_deep_sleep_debug(void)
{
	volatile uint32_t *pvbm = (volatile uint32_t *)0x4000a800u;
	int i = 0;

	while (i < 5) {
		printk("PCR CLK_REQ[%d] = 0x%08x\n", i, pvbm[i]);
		i++;
	}

	printk("PCR SYS_SLP_CTRL = 0x%08x\n", pvbm[i++]);
	printk("ECS Sleep Status Mirror = 0x%08x\n", pvbm[i]);
}
