/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I2C_I2C_MCHP_MEC5_NL_H_
#define ZEPHYR_DRIVERS_I2C_I2C_MCHP_MEC5_NL_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Helpers used by the companion i2c_mchp_mec5_mux driver so that it never
 * touches controller registers directly.
 */
int mec5_i2c_nl_stop_and_reset(const struct device *dev);
int mec5_i2c_nl_apply_port_freq(const struct device *dev, uint8_t port, uint32_t freq_hz);
int mec5_i2c_nl_apply_pinctrl(const struct device *dev,
			      const struct pinctrl_dev_config *pcfg);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_I2C_I2C_MCHP_MEC5_NL_H_ */
