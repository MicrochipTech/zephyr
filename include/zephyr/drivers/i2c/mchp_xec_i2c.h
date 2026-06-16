/*
 * Copyright (c) 2026 Microchip Technologies Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_XEC_I2C_NL_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_XEC_I2C_NL_H_

#include <zephyr/device.h>

int mchp_xec_i2c_nl_port_get(const struct device *i2c_dev, uint8_t *port);

int mchp_xec_i2c_nl_port_set(const struct device *i2c_dev, uint8_t port);

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_XEC_I2C_NL_H_ */
