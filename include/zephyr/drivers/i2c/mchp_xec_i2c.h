/*
 * Copyright (c) 2026 Microchip Technologies Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_XEC_I2C_NL_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_XEC_I2C_NL_H_

#include <zephyr/device.h>

#ifdef CONFIG_I2C_MCHP_XEC_NL
/**
 * @brief Get the physical port the I2C-NL controller hardware is configured for.
 *
 * @param i2c_port_dev I2C-NL port device on the controller to query.
 * @param port Pointer to store the physical port number.
 *
 * @retval 0 Success
 * @retval -EINVAL Invalid argument
 */
int mchp_xec_i2c_nl_port_get(const struct device *i2c_port_dev, uint8_t *port);

/**
 * @brief Route the I2C-NL controller to a physical port.
 *
 * The port must belong to an enabled I2C-NL port device on the same controller.
 * Its pin configuration and bus frequency are applied. Nothing is changed when the
 * controller is already routed to the port.
 *
 * @param i2c_port_dev I2C-NL port device on the controller to route.
 * @param port Physical port number.
 *
 * @retval 0 Success
 * @retval -EINVAL Invalid argument or no enabled port device for the port
 * @retval -EBUSY Controller or bus is busy
 * @retval -errno Negative errno from applying the port pin configuration
 */
int mchp_xec_i2c_nl_port_set(const struct device *i2c_port_dev, uint8_t port);
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_XEC_I2C_NL_H_ */
