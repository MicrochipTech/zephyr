/*
 * Copyright (c) 2026 Microchip Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ZEPHYR_DRIVERS_I2C_I2C_MCHP_XEC_COMMON_H_
#define _ZEPHYR_DRIVERS_I2C_I2C_MCHP_XEC_COMMON_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/sys_io.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XEC_I2C_SCL_LINE_POS 0
#define XEC_I2C_SDA_LINE_POS 1
#define XEC_I2C_LINE_MASK (BIT(XEC_I2C_SCL_LINE_POS) | BIT(XEC_I2C_SDA_LINE_POS))

void i2c_xec_ctrl_reset(mm_reg_t regbase, uint8_t port, uint16_t enc_pcr_scr);

int i2c_xec_prog_timing(mm_reg_t regbase, uint32_t freq_hz);

uint32_t i2c_xec_freq_from_dev_config(uint32_t dev_config);

uint8_t i2c_xec_get_lines(mm_reg_t regbase, const struct gpio_dt_spec *scl_gpio,
                          const struct gpio_dt_spec *sda_gpio);

int i2c_xec_bus_recovery(mm_reg_t rb);

#ifdef __cplusplus
}
#endif

#endif /* _ZEPHYR_DRIVERS_I2C_I2C_MCHP_XEC_COMMON_H_ */
