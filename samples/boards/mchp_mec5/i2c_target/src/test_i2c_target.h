/*
 * Copyright (c) 2024 Microchip Technology, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TEST_I2C_TARGET
#define TEST_I2C_TARGET

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#define I2C_TARGET_MODE_ADDR0 0x40
#define I2C_TARGET_MODE_ADDR1 0x41
/* #define I2C_TARGET_USE_BOTH_TARGETS */

int test_i2c_target_register(const struct device *i2c_dev, bool register_target,
			     uint8_t target_id);

int test_i2c_target_wr_init(void);

int test_i2c_target_write(const struct device *i2c_cm_dev, const struct device *i2c_tm_dev,
			  uint16_t tm_addr, uint8_t *data, size_t datasz);

int test_i2c_target_read_data_init(uint8_t *data, size_t datasz);
int test_i2c_target_read(const struct device *i2c_cm_dev, const struct device *i2c_tm_dev,
			 uint16_t tm_addr, uint8_t *data, size_t datasz);
bool test_i2c_target_read_done(const struct device *i2c_tm_dev, uint16_t tm_addr);

int test_i2c_target_combined(const struct device *i2c_cm_dev, const struct device *i2c_tm_dev,
			     uint16_t tm_addr, uint8_t *wdata, size_t wrsz,
			     uint8_t *rdata, size_t rdsz);

#endif /* TEST_I2C_TARGET */
