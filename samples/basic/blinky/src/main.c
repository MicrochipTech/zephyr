/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/* The devicetree node identifier for the "led1" alias. */
#define LED1_NODE DT_ALIAS(led1)
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

void main(void)
{
	int ret;
	int led1ConfigStat;

	if ((!gpio_is_ready_dt(&led)) || (!gpio_is_ready_dt(&led1))) {
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	led1ConfigStat = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	if ((ret < 0)||(led1ConfigStat < 0)) {
		return;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		led1ConfigStat = gpio_pin_toggle_dt(&led1);
		if ((ret < 0) || (led1ConfigStat < 0)){
			return;
		}
		k_msleep(SLEEP_TIME_MS);
	}
}
