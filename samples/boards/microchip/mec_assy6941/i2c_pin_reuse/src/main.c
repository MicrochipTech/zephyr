/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define I2C_CTRL0_NODE   DT_NODELABEL(i2c_smb_0)
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define I2C_CTRL0_REG_BASE DT_REG_ADDR(I2C_CTRL0_NODE)

#define MCHP_XEC_I2C_CFG_REG_OFS 0x28U
#define MCHP_XEC_I2C_CFG_EN_POS  10

#define XEC_GPIO_BANK0_BASE_ADDR DT_REG_ADDR(DT_NODELABEL(gpio_000_036))
#define XEC_GPIO_PIN_CTRL_REG_ADDR(bank_base, pin_pos) \
	((uint32_t)(bank_base) + ((uint32_t)(pin_pos) * 4U))

#define XEC_GPIO_003_CR_ADDR XEC_GPIO_PIN_CTRL_REG_ADDR(XEC_GPIO_BANK0_BASE_ADDR, 3U)
#define XEC_GPIO_004_CR_ADDR XEC_GPIO_PIN_CTRL_REG_ADDR(XEC_GPIO_BANK0_BASE_ADDR, 4U)

#define PCA9555_CMD_PORT0_IN  0
#define PCA9555_CMD_PORT1_IN  1u
#define PCA9555_CMD_PORT0_OUT 2u
#define PCA9555_CMD_PORT1_OUT 3u
#define PCA9555_CMD_PORT0_POL 4u
#define PCA9555_CMD_PORT1_POL 5u
#define PCA9555_CMD_PORT0_CFG 6u
#define PCA9555_CMD_PORT1_CFG 7u

#define PCA9555_I2C_NODE DT_NODELABEL(pca9555_evb)
#define PCA9555_I2C_PORT DT_PROP(DT_PARENT(PCA9555_I2C_NODE), port_sel)
#define PCA9555_I2C_ADDR DT_REG_ADDR(PCA9555_I2C_NODE)

#define LTC2489_I2C_NODE DT_NODELABEL(ltc2489_evb))
#define LTC2489_I2C_PORT DT_PROP(DT_PARENT(LTC2489_I2C_NODE), port_sel)
#define LTC2489_I2C_ADDR DT_REG_ADDR(DT_NODELABEL(ltc2489_evb))

#define APP_I2C_TX_BUF_SIZE 32
#define APP_I2C_RX_BUF_SIZE 32

/* !!!! WARNING NOT A GOOD SOLUTION !!!!
 * Getting a pointer to a driver's PINCTRL structure requires
 * CONFIG_PINCTRL_NON_STATIC=y
 * This Kconfig forces ALL PINCTRL structures be located in data RAM
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 * Instead we will duplicate the specific I2C controller pinctrl-0 in zephyr,user
 */
#if 0
static const struct pinctrl_dev_config *i2c_port0_pcfg = PINCTRL_DT_DEV_CONFIG_GET(I2C_CTRL0_NODE);
#else
// PINCTRL_DT_DEFINE(ZEPHYR_USER_NODE);
//static const struct pinctrl_dev_config *i2c_port0_pcfg = PINCTRL_DT_DEV_CONFIG_GET(ZEPHYR_USER_NODE);
#endif

const struct gpio_dt_spec i2c_port0_sda_pin_dt =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, i2c_port0_gpios, 0);

const struct gpio_dt_spec i2c_port0_scl_pin_dt =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, i2c_port0_gpios, 1);

const struct device *i2c0_dev = DEVICE_DT_GET(I2C_CTRL0_NODE);

static uint8_t i2c_tx_buf[APP_I2C_TX_BUF_SIZE];
static uint8_t i2c_rx_buf[APP_I2C_RX_BUF_SIZE];

int main(void)
{
	uint32_t gpio_ctrl[2] = {0};
	uint32_t i2c0_config = 0, ntx = 0, nrx = 0;
	int rc = 0;
	uint16_t i2c_addr = 0;

	LOG_INF("I2C pin reuse test app: main entry");

	if (!device_is_ready(i2c0_dev)) {
		LOG_ERR("I2C Controller 0 is not ready!");
		return -1;
	}

	i2c0_config = I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD);

	rc = i2c_configure(i2c0_dev, i2c0_config);
	if (rc != 0) {
		LOG_ERR("I2C Controller 0 config value: 0x%08x got error (%d)", i2c0_config, rc);
		return -2;
	}

	LOG_INF("Read PCA9555 I2C device port 0 and port 1 registers");
	i2c_addr = (uint16_t)PCA9555_I2C_ADDR;
	i2c_tx_buf[0] = PCA9555_CMD_PORT0_IN;

	ntx = 1U;
	nrx = 2U;

	rc = i2c_write_read(i2c0_dev, i2c_addr, (const void *)i2c_tx_buf, ntx,
			    (void *)i2c_rx_buf, nrx);
	if (rc != 0) {
		LOG_ERR("i2c_write_read error (%d)", rc);
		return -3;
	}

	LOG_INF("PCA9555 Input Port0 = 0x%02X  Port1 = 0x%02X", i2c_rx_buf[0], i2c_rx_buf[1]);

	LOG_INF("Re-use I2C Port 0 pins");

	LOG_INF("1. Disable I2C Controller 0");
	sys_clear_bit(I2C_CTRL0_REG_BASE + MCHP_XEC_I2C_CFG_REG_OFS, MCHP_XEC_I2C_CFG_EN_POS);

	LOG_INF("2. Save I2C Controller 0 pin GPIO control register values");
	gpio_ctrl[0] = sys_read32(XEC_GPIO_003_CR_ADDR);
	gpio_ctrl[1] = sys_read32(XEC_GPIO_004_CR_ADDR);
	LOG_INF("Saved GPIO 003 Control reg = 0x%0x", gpio_ctrl[0]);
	LOG_INF("Saved GPIO 004 Control reg = 0x%0x", gpio_ctrl[1]);

	LOG_INF("3. Reconfigure I2C Port 0 pins as GPIO Output drive high and enable input");
	rc = gpio_pin_configure_dt(&i2c_port0_sda_pin_dt, GPIO_OUTPUT_HIGH | GPIO_INPUT);
	if (rc != 0) {
		LOG_ERR("Error configuring SDA as GPIO_OUTPUT_HIGH (%d)", rc);
		return -4;
	}

	rc = gpio_pin_configure_dt(&i2c_port0_scl_pin_dt, GPIO_OUTPUT_HIGH | GPIO_INPUT);
	if (rc != 0) {
		LOG_ERR("Error configuring SCL as GPIO_OUTPUT_HIGH (%d)", rc);
		return -5;
	}

	LOG_INF("Reconfig of GPIO003 as GPIO = 0x%0x", sys_read32(XEC_GPIO_003_CR_ADDR));
	LOG_INF("Reconfig of GPIO004 as GPIO = 0x%0x", sys_read32(XEC_GPIO_004_CR_ADDR));

	k_busy_wait(10U); /* 10 microseconds */

	LOG_INF("4. Use the pins as GPIOs");

	/* App would use the pins */
	LOG_INF("5. Drive a square wave on SDA");
	for (int i = 0; i < 10; i++) {
		gpio_pin_set_dt(&i2c_port0_sda_pin_dt, 0);
		k_busy_wait(10U);
		gpio_pin_set_dt(&i2c_port0_sda_pin_dt, 1);
		k_busy_wait(10U);
	}

	/* Process to re-enable I2C */

	/* Fist generate an I2C STOP in case any I2C devices attached to these pin were
	 * put into a bad state by our use of the pins as GPIOs
	 * I2C STOP is SCL is stable High and 1/2 to 1 I2C clock later SDA has low to high edge
	 */

	LOG_INF("6. Before restoring pins for I2C, generate an I2C STOP to ensure any I2C devices are OK");

	/* Drive SCL high and SDA low */
	gpio_pin_set_dt(&i2c_port0_scl_pin_dt, 1);
	gpio_pin_set_dt(&i2c_port0_sda_pin_dt, 0);

	k_busy_wait(10U); /* One 100 KHz clock */

	/* Drive SDA high */
	gpio_pin_set_dt(&i2c_port0_sda_pin_dt, 1);

	k_busy_wait(10U); /* One 100 KHz clock */

	rc = gpio_pin_get_dt(&i2c_port0_scl_pin_dt);
	if (rc != 1) {
		LOG_ERR("After using pins to generate I2C STOP, we see SCL is LOW (%d)", rc);
		return -6;
	}

	rc = gpio_pin_get_dt(&i2c_port0_sda_pin_dt);
	if (rc != 1) {
		LOG_ERR("After using pins to generate I2C STOP, we see SDA is LOW (%d)", rc);
		return -6;
	}

	LOG_INF("7. Restore pins GPIO Control registers back to I2C driver PINCTRL settings");
	sys_write32(gpio_ctrl[0], XEC_GPIO_003_CR_ADDR);
	sys_write32(gpio_ctrl[1], XEC_GPIO_004_CR_ADDR);

	/* LOG_INF("Re-enable I2C Controller 0"); */
	sys_set_bit(I2C_CTRL0_REG_BASE + MCHP_XEC_I2C_CFG_REG_OFS, MCHP_XEC_I2C_CFG_EN_POS);

	/* LOG_INF("Re-configure I2C Controller 0"); */
	rc = i2c_configure(i2c0_dev, i2c0_config);
	if (rc != 0) {
		LOG_ERR("I2C Controller 0 config value: 0x%08x got error (%d)", i2c0_config, rc);
		return -7;
	}

	LOG_INF("8. Read PCA9555 I2C device port 0 and port 1 registers");
	i2c_addr = (uint16_t)PCA9555_I2C_ADDR;
	i2c_tx_buf[0] = PCA9555_CMD_PORT0_IN;

	ntx = 1U;
	nrx = 2U;

	rc = i2c_write_read(i2c0_dev, i2c_addr, (const void *)i2c_tx_buf, ntx,
			    (void *)i2c_rx_buf, nrx);
	if (rc != 0) {
		LOG_ERR("i2c_write_read error (%d)", rc);
		return -9;
	}

	LOG_INF("PCA9555 Input Port0 = 0x%02X  Port1 = 0x%02X", i2c_rx_buf[0], i2c_rx_buf[1]);

	LOG_INF("Application End");

	return 0;
}
