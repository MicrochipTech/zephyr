/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <soc.h>

volatile u32_t vg1;
volatile s32_t sg1, sg2, sg3, sg4;

void main(void)
{
	GPIO_CTRL_REGS->CTRL_0013 = 0x10240UL;

	printk("Hello World! %s\n", CONFIG_BOARD);

	vg1 = 0U;

	sg1 = 100;
	sg2 = 500;
	sg3 = sg1 - sg2;
	sg4 = -1 * sg3;

	while (1) {
#if 0
		GPIO_CTRL_REGS->CTRL_0013 = 0x00240UL;
		k_sleep(1);
		GPIO_CTRL_REGS->CTRL_0013 = 0x10240UL;

		vg1 = 0U;
		while (vg1 < 1000U) {
			vg1 += GPIO_CTRL_REGS->CTRL_0013;
		}

		GPIO_CTRL_REGS->CTRL_0157 = 0x00240UL;
		k_sleep(1);
		GPIO_CTRL_REGS->CTRL_0157 = 0x10240UL;

		vg1 = 0U;
		while (vg1 < 1000U) {
			vg1 += GPIO_CTRL_REGS->CTRL_0157;
		}
#else
		GPIO_CTRL_REGS->CTRL_0013 ^= (1ul << 16);
		k_sleep(1);
#endif
	}
}
