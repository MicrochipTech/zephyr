/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/sys/sys_io.h>

#define XEC_I2C_CONFIG_OFS   0x28U
#define XEC_I2C_WAKE_STS_OFS 0x60U
#define XEC_I2C_WAKE_EN_OFS  0x64U

#define XEC_I2C_CONFIG_EN_POS   10
#define XEC_I2C_WAKE_EN_STS_POS 0

/* Assumes controller has been enabled and configured for target mode */
void soc_i2c_wake_prepare(uintptr_t i2c_reg_base, uint8_t girq, uint8_t girq_pos)
{
	sys_clear_bit(i2c_reg_base + XEC_I2C_WAKE_EN_OFS, XEC_I2C_WAKE_EN_STS_POS);
	sys_set_bit(i2c_reg_base + XEC_I2C_WAKE_STS_OFS, XEC_I2C_WAKE_EN_STS_POS);
	soc_ecia_girq_status_clear(girq, girq_pos);
	sys_set_bit(i2c_reg_base + XEC_I2C_WAKE_EN_OFS, XEC_I2C_WAKE_EN_STS_POS);
	soc_ecia_girq_ctrl(girq, girq_pos, MCHP_MEC_ECIA_GIRQ_EN);
}

void soc_i2c_wake_clear(uintptr_t i2c_reg_base, uint8_t girq, uint8_t girq_pos)
{
	soc_ecia_girq_ctrl(girq, girq_pos, MCHP_MEC_ECIA_GIRQ_DIS);
	sys_clear_bit(i2c_reg_base + XEC_I2C_WAKE_EN_OFS, XEC_I2C_WAKE_EN_STS_POS);
	sys_set_bit(i2c_reg_base + XEC_I2C_WAKE_STS_OFS, XEC_I2C_WAKE_EN_STS_POS);
	soc_ecia_girq_status_clear(girq, girq_pos);
}

uint8_t soc_i2c_get_enable(uintptr_t i2c_reg_base)
{
	if (sys_test_bit(i2c_reg_base + XEC_I2C_CONFIG_OFS, XEC_I2C_CONFIG_EN_POS) != 0) {
		return 1U;
	}

	return 0;
}

void soc_i2c_set_enable(uintptr_t i2c_reg_base, uint8_t enable)
{
	if (enable != 0) {
		sys_set_bit(i2c_reg_base + XEC_I2C_CONFIG_OFS, XEC_I2C_CONFIG_EN_POS);
	} else {
		sys_clear_bit(i2c_reg_base + XEC_I2C_CONFIG_OFS, XEC_I2C_CONFIG_EN_POS);
	}
}
