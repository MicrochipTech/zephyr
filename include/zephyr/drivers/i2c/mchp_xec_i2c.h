/*
 * Copyright (c) 2026 Microchip Technologies Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_XEC_I2C_NL_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_XEC_I2C_NL_H_

#include <zephyr/device.h>

#ifdef CONFIG_I2C_MCHP_XEC_V3_NL
int mchp_xec_i2c_nl_port_get(const struct device *i2c_port_dev, uint8_t *port);
int mchp_xec_i2c_nl_port_set(const struct device *i2c_port_dev, uint8_t port);

#ifdef CONFIG_I2C_MCHP_XEC_V3_NL_STATE_CAPTURE
int mchp_xec_i2c_nl_clear_capture(const struct device *port);
int mchp_xec_i2c_nl_copy_capture(const struct device *port, uint8_t *capdest, size_t capdest_size);
#endif
#endif

#ifdef CONFIG_I2C_MCHP_XEC_V3_BM
int mchp_xec_i2c_bm_port_get(const struct device *i2c_port_dev, uint8_t *port);
int mchp_xec_i2c_bm_port_set(const struct device *i2c_port_dev, uint8_t port);

#ifdef CONFIG_I2C_MCHP_XEC_V3_BM_STATE_CAPTURE
int mchp_xec_i2c_bm_clear_capture(const struct device *port);
int mchp_xec_i2c_bm_copy_capture(const struct device *port, uint8_t *capdest, size_t capdest_size);
#endif
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_XEC_I2C_NL_H_ */
