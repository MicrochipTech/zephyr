/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I2C_I2C_MCHP_XEC_V3_H_
#define ZEPHYR_DRIVERS_I2C_I2C_MCHP_XEC_V3_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

struct pinctrl_dev_config;

/*
 * Internal API exposed by the Microchip XEC I2C v3 controller driver to the
 * companion port driver. Applications bind to port devices; port callbacks
 * serialize access and delegate to these helpers.
 *
 * Every call below takes the owning controller device handle. The port
 * identifier is the low nibble of the DT "port" property built with
 * MCHP_XEC_I2C_CTRL_PORT(ctrl, port).
 */

void mchp_xec_i2c_ctrl_mutex_lock(const struct device *ctrl);
void mchp_xec_i2c_ctrl_mutex_unlock(const struct device *ctrl);

int mchp_xec_i2c_ctrl_configure(const struct device *ctrl, uint8_t port,
				uint32_t freq_hz);

int mchp_xec_i2c_ctrl_get_speed(const struct device *ctrl, uint8_t port,
				uint32_t *speed);

int mchp_xec_i2c_ctrl_transfer(const struct device *ctrl, uint8_t port,
			       uint32_t freq_hz, struct i2c_msg *msgs,
			       uint8_t num_msgs, uint16_t addr);

#ifdef CONFIG_I2C_CALLBACK
int mchp_xec_i2c_ctrl_transfer_cb(const struct device *ctrl, uint8_t port,
				  uint32_t freq_hz, struct i2c_msg *msgs,
				  uint8_t num_msgs, uint16_t addr,
				  i2c_callback_t cb, void *userdata);
#endif

int mchp_xec_i2c_ctrl_recover_bus(const struct device *ctrl, uint8_t port,
				  uint32_t freq_hz,
				  const struct pinctrl_dev_config *pcfg);

#ifdef CONFIG_I2C_TARGET
int mchp_xec_i2c_ctrl_target_register(const struct device *ctrl, uint8_t port,
				      struct i2c_target_config *cfg);

int mchp_xec_i2c_ctrl_target_unregister(const struct device *ctrl, uint8_t port,
					struct i2c_target_config *cfg);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_I2C_I2C_MCHP_XEC_V3_H_ */
