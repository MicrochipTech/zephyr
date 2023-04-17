/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>


uint32_t regs[] = {
	0x400811e0,
	0x4008102c,
	0x40081028,
	0x40081010,
	0x4008100c,
	0x400810f0,
	0x400810f4
};

#define REG32(a)    *((volatile uint32_t *)(uintptr_t)(a))

int main(void)
{
	printk("Hello World! %s no fix\n", CONFIG_BOARD);

	printk("\nDelta observed\n");
	for (int i = 0; i < ARRAY_SIZE(regs); i++) {
		printk("%08x=%08x\n", regs[i], REG32(regs[i]));
	}


	printk("\nAll registers\n");
	for (uint32_t reg = 0x40081000; reg <= 0x400812B4; reg += 4) {
		printk("%08x=%08x\n", reg, REG32(reg));
	}

	return 0;
}
