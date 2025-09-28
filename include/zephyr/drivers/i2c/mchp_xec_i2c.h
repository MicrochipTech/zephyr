/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_MEC5_I2C_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_MEC5_I2C_H_

#include <zephyr/device.h>

#define MCHP_I2C_NUM_PORTS 16

int i2c_mchp_xec_port_get(const struct device *dev, uint8_t *port);
int i2c_mchp_xec_port_set(const struct device *dev, uint8_t port,
			  uint8_t new_cfg, uint32_t i2c_devconfig);

/* DEBUG */
int i2c_mchp_xec_v3_debug_init(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_MEC5_I2C_H_ */
