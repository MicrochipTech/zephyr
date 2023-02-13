/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);
	printk("First Commit\n");
	printk("Second Commit\n");
}
