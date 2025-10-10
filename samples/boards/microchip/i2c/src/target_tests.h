/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _ZEPHYR_SAMPLES_BOARDS_MICROCHIP_I2C_TARGET_TESTS_H
#define _ZEPHYR_SAMPLES_BOARDS_MICROCHIP_I2C_TARGET_TESTS_H

#include <zephyr/device.h>

#define I2C1_TARG_ADDR1 0x40u
#define I2C1_TARG_ADDR2 0x41u
#define I2C2_TARG_ADDR1 0x20u
#define I2C2_TARG_ADDR2 0x21u

int target_config(const struct device *i2c_dev, uint16_t targ_addr);
int target_i2c_nl_prepare_tests(const struct device *i2c_host_dev);
int target_i2c_nl_run_tests(const struct device *i2c_host_dev);

#endif /* _ZEPHYR_SAMPLES_BOARDS_MICROCHIP_I2C_TARGET_TESTS_H */
