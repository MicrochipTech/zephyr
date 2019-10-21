/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <soc.h>

#include <logging/log.h>
#define CONFIG_LOG_LEVEL 3
LOG_MODULE_REGISTER(helloworld, CONFIG_LOG_LEVEL);


volatile u32_t vg1;
void main(void)
{
	GPIO_CTRL_REGS->CTRL_0013 = 0x10240UL;
	GPIO_CTRL_REGS->CTRL_0156 = 0x10240UL;

	printk("Hello World! %s\n", CONFIG_BOARD);

	vg1 = 0U;

	while (1) {
		GPIO_CTRL_REGS->CTRL_0156 = 0x00240UL;

		/* Trigger Deep Sleep 1 state. 48MHz PLL off */
		k_sleep(3000);

		GPIO_CTRL_REGS->CTRL_0156 = 0x10240UL;

		k_busy_wait(3000);

		LOG_INF("Wake from Deep Sleep\n");

		/* Trigger Light Sleep 1 state. 48MHz PLL stays on */
		k_sleep(1500);

		LOG_INF("Wake from Light Sleep\n");
		k_busy_wait(100);

	}
}
