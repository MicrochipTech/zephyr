/*
 * Copyright (c) 2026 Microchip Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_XEC_H
#define ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_XEC_H

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#ifdef CONFIG_I2C_XEC_MUX

/** @brief Get current Port the I2C controller is using.
 *
 * @param dev      Pointer to the device structure for an I2C controller
 *                 driver configured in controller mode.
 * @param port_sel Pointer to an unsigned 8-bit type to hold the port number.
 *
 * @retval 0       If successful.
 * @retval -EINVAL if either parameter is NULL
 */
int i2c_xec_v2_get_port(const struct device *dev, uint8_t *port_sel);

/** @brief Switch port and set new frequency
 *
 * @param dev        Pointer to the device structure for an I2C controller
 * @param port_sel   New I2C port
 * @param clock_freq Frequency for new port or 0 if same frequency as current port.
 *
 * @retval 0             Success
 * @retval -EINVAL       Invalid port number or frequency
 * @retval -EIO          I2C bus error detected on new port
 * @retval -ECONNABORTED Detected an external controller owning the bus on the new port
 * @retval -ETIMEDOUT    Timeout waiting on bus to settle on new port
 */
int i2c_xec_v2_set_port(const struct device *dev, uint8_t port_sel, uint32_t clock_freq);

#endif /* CONFIG_I2C_XEC_MUX */

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_XEC_H */
