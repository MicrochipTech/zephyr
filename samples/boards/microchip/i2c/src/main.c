/*
 * Copyright (c) 2025 Microchip Technology, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <stdio.h>
#include <string.h>
#include <soc.h>

#define LOG_MODULE_NAME i2c_app
LOG_MODULE_REGISTER(LOG_MODULE_NAME)

#define I2C0_NODE DT_NODELABEL(i2c0)
#define I2C1_NODE DT_NODELABEL(i2c1)
#define I2C2_NODE DT_NODELABLE(i2c2)

static const struct device *const i2c0_dev = DEVICE_DT_GET(I2C0_NODE);
static const struct device *const i2c1_dev = DEVICE_DT_GET(I2C1_NODE);
static const struct device *const i2c2_dev = DEVICE_DT_GET(I2C2_NODE);

int main(void)
{
	int ret;
	size_t chunk_size = 0;

	LOG_INF("It lives! %s\n", CONFIG_BOARD);

	if (!device_is_ready(i2c0_dev)) {
		printf("ERROR: i2c0 device is not ready!\n");
		return -EIO;
	}

	if (!device_is_ready(i2c1_dev)) {
		printf("ERROR: i2c1 device is not ready!\n");
		return -EIO;
	}
	
	if (!device_is_ready(i2c2_dev)) {
		printf("ERROR: i2c2 device is not ready!\n");
		return -EIO;
	}
	

	LOG_INF("Application done\n");

	return 0;
}

