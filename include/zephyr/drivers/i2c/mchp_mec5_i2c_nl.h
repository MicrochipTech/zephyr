/*
 * Copyright (c) 2023 SILA Embedded Solutions GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_MEC5_I2C_NL_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_MEC5_I2C_NL_H_

int i2c_mchp_nl_clr_buffers(const struct device *dev, uint8_t val);

int i2c_mchp_nl_clr_debug_data(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2C_MCHP_MEC5_I2C_NL_H_ */
