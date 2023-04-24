/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2022 Microchip Technology, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <soc.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define MCHP_XEC_GPIO_BASE DT_REG_ADDR(DT_NODELABEL(gpio_000_036))
#define MCHP_XEC_GPIO_CTRL_REG_ADDR(pin) ((uint32_t)MCHP_XEC_GPIO_BASE + ((uint32_t)(pin) * 4u))

#define MB85RC256V_FRAM_I2C_ADDR 0x50u

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c_smb_0));
static const struct device *i2c_dev_1 = DEVICE_DT_GET(DT_NODELABEL(i2c_smb_1));

static const struct gpio_dt_spec test_i2c_sda = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), i2c_sda_gpios);
static const struct gpio_dt_spec test_i2c_scl = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), i2c_scl_gpios);
static const struct gpio_dt_spec rsmrst = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), rsmrst_gpios);

PINCTRL_DT_DEFINE(DT_PATH(zephyr_user));

const struct pinctrl_dev_config *app_pinctrl_cfg = PINCTRL_DT_DEV_CONFIG_GET(DT_PATH(zephyr_user));

static volatile uint32_t spin_val;
static volatile int ret_val;
static uint32_t gpio_ctrl_values[4];

#define APP_DBG_MAX_CAP_GPIO_CTRL 32
struct cap_gpio_ctrl {
	uint8_t id;
	uint32_t gpctrl;
};

static uint32_t cap_gpio_ctrl_idx;
static struct cap_gpio_ctrl app_dbg_gpio_ctr[APP_DBG_MAX_CAP_GPIO_CTRL];

static void spin_on(uint32_t id, int rval);

static int intel_pin_test1(uint32_t pin_num, gpio_flags_t gpio_flags, uint32_t *ctrl_values);

void capture_gpio_control(uint8_t id, uint32_t gpio_ctrl_val);

void main(void)
{
	int ret = 0;
	uint32_t temp = 0;
	uint32_t ctrl1 = 0, ctrl2 = 0;

	cap_gpio_ctrl_idx = 0;
	memset(app_dbg_gpio_ctr, 0, sizeof(app_dbg_gpio_ctr));

	temp = sys_read32(MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0161_ID));
	LOG_INF("Reset VCI_IN2 GPIO pin control register value = 0x%08x", temp);

	temp = sys_read32(MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0000_ID));
	LOG_INF("Reset VCI_IN3 GPIO pin control register value = 0x%08x", temp);

	ret = pinctrl_apply_state(app_pinctrl_cfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("App pin control state apply: %d", ret);
		spin_on(1, ret);
	}

	temp = sys_read32(MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0161_ID));
	LOG_INF("After PINCTRL VCI_IN2 GPIO pin control register value = 0x%08x", temp);

	temp = sys_read32(MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0000_ID));
	LOG_INF("After PINCTRL VCI_IN3 GPIO pin control register value = 0x%08x", temp);

	temp = 0x10240;
	sys_write32(temp, MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0141_ID));

	temp = 0x10240;
	sys_write32(temp, MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0142_ID));

	/* Configure GPIO055 as ROM would when it has driven the pin inactive (high) */
	temp = 0x10200u;
	sys_write32(temp, MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0055_ID));

	memset(gpio_ctrl_values, 0, sizeof(gpio_ctrl_values));

	LOG_INF("Trigger scope");

	/* drive GPIO 0141 low for trigger to analyzer */
	temp = 0x00240;
	sys_write32(temp, MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0141_ID));

	k_busy_wait(20u);

	/* intel_pin_test1(MCHP_GPIO_0055_ID, GPIO_INPUT | GPIO_PULL_UP); */
	intel_pin_test1(MCHP_GPIO_0055_ID, GPIO_OUTPUT_HIGH, gpio_ctrl_values);

	k_busy_wait(20u);

	/* drive GPIO 0141 back high */
	temp = 0x10240;
	sys_write32(temp, MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0141_ID));

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device 0 is not ready!");
		spin_on(6, ret);
	}

	if (!device_is_ready(i2c_dev_1)) {
		LOG_ERR("I2C device 1 is not ready!");
		spin_on(7, ret);
	}

	/* Try enabling GPIO interrupt on a pin set to alternate function by PINCTRL */
	const struct device *itest_gpio_port_dev = DEVICE_DT_GET(DT_NODELABEL(gpio_000_036));

	ctrl1 = sys_read32(MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0012_ID));
	LOG_INF("i2c07_sda_gpio012 GPIO pin control register value = 0x%08x", ctrl1);

	LOG_INF("Configure i2c07_sda_gpio012 for falling edge interrupt");
	/* drive GPIO 0141 low */
	temp = 0x00240;
	sys_write32(temp, MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0141_ID));

	ret = gpio_pin_interrupt_configure(itest_gpio_port_dev, MCHP_GPIO_012, GPIO_INT_EDGE_FALLING);

	/* drive GPIO 0141 back high */
	temp = 0x10240;
	sys_write32(temp, MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0141_ID));

	if (ret) {
		LOG_ERR("GPIO interrupt configure failed: %d", ret);
	}

	ctrl2 = sys_read32(MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0012_ID));
	LOG_INF("i2c07_sda_gpio012 GPIO pin control register value = 0x%08x", ctrl2);
	if ((ctrl2 & MCHP_GPIO_CTRL_IDET_MASK) == MCHP_GPIO_CTRL_IDET_FEDGE) {
		LOG_INF("Success: Pin configured for falling edge interrupt detection");
	} else {
		LOG_ERR("Failure: Pin interrupt detection field not configured properly");
	}

	if ((ctrl2 & 0x1ff0fu) != (ctrl1 & 0x1ff0fu)) {
		LOG_ERR("Failure: Other GPIO control fields modified!");
	}

	if ((ctrl2 & BIT(MCHP_GPIO_CTRL_INPAD_VAL_POS))
	    ^ (ctrl1 & BIT(MCHP_GPIO_CTRL_INPAD_VAL_POS))) {
		LOG_INF("WARNING: Pin's input state changed. Was this expected?");
	}


	/* Try turning on internal pull-up only with PINCTRL
	 * no other pin configuration should be touched
	 */
	LOG_INF("Enable internal pull-up on i2c07_sda_gpio012 and leave other fields untouched");
	ctrl1 = sys_read32(MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0012_ID));
	LOG_INF("i2c07_sda_gpio012 GPIO pin control register value = 0x%08x", ctrl1);

	/* drive GPIO 0141 low */
	temp = 0x00240;
	sys_write32(temp, MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0141_ID));

	uint32_t pin_num = MCHP_GPIO_0012_ID;
	const pinctrl_soc_pin_t pin = (MCHP_XEC_PINMUX(pin_num, MCHP_AF1)
				       | (1u << MCHP_XEC_PU_POS));

	ret = pinctrl_configure_pins(&pin, 1, PINCTRL_REG_NONE);
	if (ret) {
		LOG_ERR("Call to PINCTRL to configure i2c07_sda_gpio012 pull-up failed: %d", ret);
	}

	/* drive GPIO 0141 back high */
	temp = 0x10240;
	sys_write32(temp, MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0141_ID));

	ctrl2 = sys_read32(MCHP_XEC_GPIO_CTRL_REG_ADDR(MCHP_GPIO_0012_ID));
	LOG_INF("i2c07_sda_gpio012 GPIO pin control register value = 0x%08x", ctrl2);
	if ((ctrl2 & 0x1fffcu) != (ctrl1 & 0x1fffcu)) {
		LOG_ERR("Failure: Other GPIO control fields modified!");
	}
	if ((ctrl2 & MCHP_GPIO_CTRL_PUD_MASK) == MCHP_GPIO_CTRL_PUD_PU) {
		LOG_INF("Pin's internal pull-up enabled");
	} else {
		LOG_ERR("Pin's internal pull field not set for pull-up");
	}
	if ((ctrl2 & BIT(MCHP_GPIO_CTRL_INPAD_VAL_POS))
	    ^ (ctrl1 & BIT(MCHP_GPIO_CTRL_INPAD_VAL_POS))) {
		LOG_INF("WARNING: Pin's input state changed. Expected only if pin's initial "
			"state was low when internal pull-up enabled");
	}

	LOG_INF("Application Done");
	spin_on(256, 0);
}

/* Intel test which fails
 * gpio_force_configure_pin(port_pin = RSMRST_PWRGD_MAF_P, flags = GPIO_INPUT | GPIO_PULL_UP);
 * gpio_force_configure_pin(port_pin = PM_RSMRST_MAF_P, flags = GPIO_OUTPUT_HIGH);
 *
 * struct gpio_port_pin pp;
 * validate_device(port_pin, &pp); fills in pp if pin is valid
 * port_idx = gpio_get_port(port_pin);
 *
 * gpio_force_configure_pin calls pinctrl then gpio
 * pinctrl_soc_pin_t pin = { .pinmux = MCHP_XEC_PINMUX(get_absolute_gpio_num(port_pin), MCHP_GPIO) };
 * pin instruction PINCTRL to set the pin to GPIO Mode (switch off alternate function)
 *
 * pinctrl_configure_pins(&pin, 1, PINCTRL_REG_NONE)
 *
 *
 * gpio_pin_configure(pp.gpio_dev, pp.pin, flags)
 *
 * intel_pin_test1(MCHP_GPIO_0055_ID, GPIO_INPUT | GPIO_PULL_UP)
 */
static int intel_pin_test1(uint32_t pin_num, gpio_flags_t gpio_flags, uint32_t *ctrl_values)
{
	int ret;
	mem_addr_t ctrl_addr;
	const struct device *gpio_port_dev;

	if (!ctrl_values) {
		LOG_ERR("Intel pin test1 invalid ctrl values array pointer");
		return -EINVAL;
	}

	if (pin_num < 040u) {
		gpio_port_dev = DEVICE_DT_GET(DT_NODELABEL(gpio_000_036));
	} else if (pin_num < 0100u) {
		gpio_port_dev = DEVICE_DT_GET(DT_NODELABEL(gpio_040_076));
	} else if (pin_num < 0140u) {
		gpio_port_dev = DEVICE_DT_GET(DT_NODELABEL(gpio_100_136));
	} else if (pin_num < 0200u) {
		gpio_port_dev = DEVICE_DT_GET(DT_NODELABEL(gpio_140_176));
	} else if (pin_num < 0240u) {
		gpio_port_dev = DEVICE_DT_GET(DT_NODELABEL(gpio_200_236));
	} else if (pin_num < 0300u) {
		gpio_port_dev = DEVICE_DT_GET(DT_NODELABEL(gpio_240_276));
	} else {
		LOG_ERR("Intel pin test1 invalid pin number");
		return -EINVAL;
	}

	gpio_pin_t pin_pos = pin_num % 32u;

	const pinctrl_soc_pin_t pin = MCHP_XEC_PINMUX(pin_num, MCHP_GPIO);

	ctrl_addr = 0x40081000u + (pin_num * 4u);
	ctrl_values[0] = sys_read32(ctrl_addr);

	ret = pinctrl_configure_pins(&pin, 1, PINCTRL_REG_NONE);
	if (ret) {
		LOG_ERR("PINCTRL error %d", ret);
		return ret;
	}

	ctrl_values[1] = sys_read32(ctrl_addr);

	ret = gpio_pin_configure(gpio_port_dev, pin_pos, gpio_flags);
	if (ret) {
		LOG_ERR("gpio config error %d", ret);
		return ret;
	}

	ctrl_values[2] = sys_read32(ctrl_addr);

	return 0;
}

void capture_gpio_control(uint8_t id, uint32_t gpio_ctrl_val)
{
	if (cap_gpio_ctrl_idx < APP_DBG_MAX_CAP_GPIO_CTRL) {
		app_dbg_gpio_ctr[cap_gpio_ctrl_idx].id = id;
		app_dbg_gpio_ctr[cap_gpio_ctrl_idx].gpctrl = gpio_ctrl_val;
		cap_gpio_ctrl_idx++;
	}
}

static void spin_on(uint32_t id, int rval)
{
	spin_val = id;
	ret_val = rval;

	log_panic(); /* flush log buffers */

	while (spin_val) {
		;
	}
}
