/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/tracing/tracing.h>

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *i2c_bb_dev = DEVICE_DT_GET(DT_ALIAS(i2c_bb));
static const struct device *i2c_nl_dev = DEVICE_DT_GET(DT_ALIAS(i2c_nl));

int main(void)
{
	LOG_INF("Microchip I2C on EVB sample");

	if (!device_is_ready(i2c_bb_dev)) {
		LOG_ERR("I2C Byte mode controller is not ready!\n");
		return -1;
	}

	if (!device_is_ready(i2c_nl_dev)) {
		LOG_ERR("I2C-NL controller is not ready!\n");
		return -2;
	}

	LOG_ERR("\nProgram End\n");

	return 0;
}
