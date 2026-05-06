/*
 * Copyright 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I2C_MICROCHIP_XEC_I2C_V3_H_
#define ZEPHYR_DRIVERS_I2C_MICROCHIP_XEC_I2C_V3_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>

#ifdef __cplusplus
extern "C" {
#endif

void mchp_xec_i2c_v3_ctrl_lock(const struct device *ctrl);
void mchp_xec_i2c_v3_ctrl_unlock(const struct device *ctrl);

/* The following APIs must be called after acquiring the controller driver lock.
 * On return, the caller must release the controller driver lock.
 */
int mchp_xec_i2c_v3_ctrl_port_switch(const struct device *ctrl, uint32_t freq, uint8_t port);

int mchp_i2c_xec_v3_config(const struct device *dev, uint32_t dev_config, uint8_t port);
int mchp_i2c_xec_v3_get_config(const struct device *dev, uint32_t *dev_config, uint8_t *port);

int mchp_i2c_xec_v3_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			     uint16_t addr);

int mchp_xec_i2c_v3_ctrl_recover_bus(const struct device *dev,
				     const struct pinctrl_dev_config *pcfg);

#ifdef CONFIG_I2C_CALLBACK
int mchp_i2c_v3_transfer_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			    uint16_t addr, i2c_callback_t cb, void *userdata);
#endif

#ifdef CONFIG_I2C_TARGET
int mchp_xec_i2c_v3_ctrl_target_register(const struct device *ctrl,
					 struct i2c_target_config *tcfg);

int mchp_xec_i2c_v3_ctrl_target_unregister(const struct device *ctrl,
					   struct i2c_target_config *tcfg);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_I2C_MICROCHIP_XEC_I2C_V3_H_ */
