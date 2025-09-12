/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_MEC5_I2C_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_MEC5_I2C_H_

#define MCHP_I2C_NUM_PORTS 16

/** @brief MCHP I2C controller has HW MUX able to map any port to any controller */
int i2c_mchp_configure(const struct device *dev, uint32_t dev_config, uint8_t port_num);

/** @brief MCHP I2C controller get current port configuration */
int i2c_mchp_get_port(const struct device *dev, uint8_t *port_num);

/* DEBUG */
int i2c_mchp_xec_v3_debug_init(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_MEC5_I2C_H_ */
