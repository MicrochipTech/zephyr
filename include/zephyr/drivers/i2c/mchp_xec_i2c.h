/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I2C_MCHP_XEC_I2C_H_
#define ZEPHYR_DRIVERS_I2C_MCHP_XEC_I2C_H_

#include <zephyr/types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_I2C_XEC_NL_PORT_SWITCHING

int mchp_xec_i2c_nl_port_config(const struct device *dev, uint32_t dev_config, uint8_t port);

int mchp_xec_i2c_nl_port_config_get(const struct device *dev, uint32_t *dev_config, uint8_t *port);

int mchp_xec_i2c_nl_port_set(const struct device *dev, uint8_t port);

int mchp_xec_i2c_nl_port_get(const struct device *dev, uint8_t *port);

#endif /* CONFIG_I2C_XEC_NL_PORT_SWITCHING */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_I2C_MCHP_XEC_I2C_H_ */
