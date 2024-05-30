/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2022 Microchip Technology, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <soc.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

/* MEC5 HAL */
#include <mec_ecs_api.h>
#include <mec_gpio_api.h>
#include <mec_pcr_api.h>
#include <mec_vci_api.h>

/* local */
#include "power_mgmt.h"

/* output drive high using alternate output mode */
#define SCOPE_GPIO_CTRL_INIT 0x10240u

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

PINCTRL_DT_DEFINE(ZEPHYR_USER_NODE);

const struct pinctrl_dev_config *app_pinctrl_cfg = PINCTRL_DT_DEV_CONFIG_GET(ZEPHYR_USER_NODE);

static const struct device *i2c_dev = DEVICE_DT_GET(DT_ALIAS(i2c0));

/* Access devices on an I2C bus using Device Tree child nodes of the I2C controller */
static const struct i2c_dt_spec pca9555_dts = I2C_DT_SPEC_GET(DT_NODELABEL(pca9555_evb));

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

static volatile uint32_t spin_val;
static volatile int ret_val;
static struct i2c_msg msgs[4];
static uint8_t buf1[16];
static uint8_t buf2[16];

static void spin_on(uint32_t id, int rval);
static int config_adc_channels(void);

int read_pca9555(void);
int read_adc_channels(void);

/* The following GPIO's are manipulated directly to provide scope triggers.
 * GPIO012 (I2C07_SDA/I3C01_SDA_TARGET) JP14 16-17 ON
 * GPIO013 (I2C07_SCL/I3C01_SCL_TARGET) JP14 7-8 ON, J57 5-6 OFF(not installed),
 * GPIO014 (PWM6) JP14 10-11 ON JP79 13-14 OFF(not installed)
 *
 * Board pull-ups
 * GPIO012 JP76 21-22 (7.5K pull-up to VTR2)
 * GPIO013 JP76 23-24 (7.5K pull-up to VTR2)
 * GPIO014 JP76 13-14 (100K pull -up to VTR2)
 *
 * J22 (1x6)
 * 1 = GPIO013/I2C07_SCL/I3C01_SCL
 * 2 = GND
 * 3 = GPIO012/I2C07_SDA/I3C01_SDA
 * 4 = NC
 * 5 = VTR2
 * 6 = GND
 *
 * JP79 2x12
 * 13 = GPIO14/PWM6
 */
int main(void)
{
	uint32_t vci_pin_bitmap = BIT(MEC_VCI_IN1_POS) | BIT(MEC_VCI_IN2_POS);
	uint32_t i2c_dev_config = 0;
	bool use_logging = true;
	int ret = 0;
	uint8_t cycles = 0;
	uint8_t nmsgs = 0;

	LOG_INF("MEC5 SoC PM sample: board: %s", DT_N_P_compatible_IDX_0);

	mec_hal_gpio_set_config(MEC_PIN_0012, SCOPE_GPIO_CTRL_INIT);
	mec_hal_gpio_set_config(MEC_PIN_0013, SCOPE_GPIO_CTRL_INIT);
	mec_hal_gpio_set_config(MEC_PIN_0014, SCOPE_GPIO_CTRL_INIT);

	ret = pinctrl_apply_state(app_pinctrl_cfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("App pin control state apply error: %d", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	ret = config_adc_channels();
	if (ret) {
		LOG_ERR("ADC config failed (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device is not ready! (%d)", -3);
		spin_on((uint32_t)__LINE__, -3);
	}

	i2c_dev_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;

	ret = i2c_configure(i2c_dev, i2c_dev_config);
	if (ret) {
		LOG_ERR("I2C configuration error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	nmsgs = 1u;
	buf1[0] = 2u; /* PCA9555 cmd=2 is output port 0 */
	buf1[1] = 0x55u;
	buf1[2] = 0xaau;
	msgs[0].buf = buf1;
	msgs[0].len = 3u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&pca9555_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C write to PCA9555 error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	ret = mec_hal_vci_in_input_enable(MEC_VCI, vci_pin_bitmap);
	if (ret != MEC_RET_OK) {
		LOG_ERR("App VCI input enable error: %d", ret);
		spin_on((uint32_t)__LINE__, 1);
	}
	ret = mec_hal_vci_in_latch_enable(MEC_VCI, vci_pin_bitmap);
	if (ret != MEC_RET_OK) {
		LOG_ERR("App VCI input latch enable error: %d", ret);
		spin_on((uint32_t)__LINE__, 1);
	}
	ret = mec_hal_vci_in_latch_reset(MEC_VCI, vci_pin_bitmap);
	if (ret != MEC_RET_OK) {
		LOG_ERR("App VCI input latch reset error: %d", ret);
		spin_on((uint32_t)__LINE__, 1);
	}

	mec_hal_vci_edge_detect_clr_all(MEC_VCI);

	cycles = 255u;
	ret = test_pwr_mgmt_singlethread(use_logging, cycles);
	if (ret) {
		LOG_ERR("Power single thread test returned error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Test multi-thread PM");
	ret = test_pwr_mgmt_multithread(use_logging, cycles);
	if (ret) {
		LOG_ERR("Power multi-thread test returned error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Application Done (%d)", ret);
	spin_on(256, 0);

	return 0;
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

static int config_adc_channels(void)
{
	int ret = 0;

	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			LOG_ERR("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return ret;
		}

		ret = adc_channel_setup_dt(&adc_channels[i]);
		if (ret < 0) {
			LOG_ERR("Could not setup channel #%d (%d)\n", i, ret);
			return ret;
		}
	}

	return 0;
}

int read_adc_channels(void)
{
	int ret;
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		int32_t val_mv;

		LOG_INF("ADC - %s, channel %d: ", adc_channels[i].dev->name,
		       adc_channels[i].channel_id);

		(void)adc_sequence_init_dt(&adc_channels[i], &sequence);

		ret = adc_read_dt(&adc_channels[i], &sequence);
		if (ret < 0) {
			LOG_ERR("Could not read (%d)", ret);
			continue;
		}

		/*
		 * If using differential mode, the 16 bit value
		 * in the ADC sample buffer should be a signed 2's
		 * complement value.
		 */
		if (adc_channels[i].channel_cfg.differential) {
			val_mv = (int32_t)((int16_t)buf);
		} else {
			val_mv = (int32_t)buf;
		}
		LOG_INF("%"PRId32, val_mv);
		ret = adc_raw_to_millivolts_dt(&adc_channels[i], &val_mv);
		/* conversion to mV may not be supported, skip if not */
		if (ret < 0) {
			LOG_ERR("Value in mV not available");
		} else {
			LOG_INF(" = %"PRId32" mV", val_mv);
		}
	}

	return 0;
}

int read_pca9555(void)
{
	int ret = 0;
	uint8_t nmsgs = 2u;

	buf1[0] = 0u; /* PCA9555 cmd=0 is input port 0 */
	buf2[0] = 0x55u; /* receive buffer */
	buf2[1] = 0x55u;

	msgs[0].buf = buf1;
	msgs[0].len = 1u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf2;
	msgs[1].len = 2u;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&pca9555_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C transfer error: write cmd byte, read 2 data bytes: (%d)", ret);
		return ret;
	}

	LOG_INF("PCA9555 Port 0 input = 0x%02x  Port 1 input = 0x%02x", buf2[0], buf2[1]);
	return 0;
}
