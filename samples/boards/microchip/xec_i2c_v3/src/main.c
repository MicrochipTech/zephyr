/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/dt-bindings/i2c/mchp-xec-i2c.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define PCA9555_CMD_PORT0_IN  0
#define PCA9555_CMD_PORT1_IN  1u
#define PCA9555_CMD_PORT0_OUT 2u
#define PCA9555_CMD_PORT1_OUT 3u
#define PCA9555_CMD_PORT0_POL 4u
#define PCA9555_CMD_PORT1_POL 5u
#define PCA9555_CMD_PORT0_CFG 6u
#define PCA9555_CMD_PORT1_CFG 7u

#define LTC2489_ADC_CONV_TIME_MS 150
#define LTC2489_ADC_READ_RETRIES 10

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define I2C_SMB_GET_DEV(nid) DEVICE_DT_GET(nid),

#define NODE_PCA9555 DT_NODELABEL(pca9555_evb)
#define NODE_LTC2489 DT_NODELABEL(ltc2489_evb)
#define NODE_FRAM    DT_NODELABEL(mb85rc256v_fram)

const struct i2c_dt_spec pca9555_spec = I2C_DT_SPEC_GET(NODE_PCA9555);
const struct i2c_dt_spec ltc2489_spec = I2C_DT_SPEC_GET(NODE_LTC2489);
const struct i2c_dt_spec mb_fram_spec = I2C_DT_SPEC_GET(NODE_FRAM);

/* Microchip byte-mode I2C driver supporting port switching */
static const struct device *i2c_smb_bm_ctrls[] = {
	DT_FOREACH_STATUS_OKAY(microchip_xec_i2c_v3, I2C_SMB_GET_DEV)};

#define I2C_MAX_MSGS    8
#define I2C_TX_BUF_SIZE 256
#define I2C_RX_BUF_SIZE 256

static struct i2c_msg msgs[I2C_MAX_MSGS];
static uint8_t i2c_tx_buf[I2C_TX_BUF_SIZE];
static uint8_t i2c_rx_buf[I2C_RX_BUF_SIZE];

static void init_test_buffers(void)
{
	memset((void *)msgs, 0, sizeof(msgs));
	memset(i2c_tx_buf, 0x55, I2C_TX_BUF_SIZE);
	memset(i2c_rx_buf, 0xAA, I2C_RX_BUF_SIZE);
}

static int check_i2c_hw_ready(const struct i2c_dt_spec *dts)
{
	if (dts == NULL) {
		return -EINVAL;
	}

	if (device_is_ready(dts->bus) == false) {
		LOG_ERR("Device %s is not ready", dts->bus->name);
		return -ENOTSUP;
	}

	return 0;
}

int main(void)
{
	int rc = 0;

	init_test_buffers();

	for (size_t n = 0; n < ARRAY_SIZE(i2c_smb_bm_ctrls); n++) {
		const struct device *i2c_cr_dev = i2c_smb_bm_ctrls[n];

		if (device_is_ready(i2c_cr_dev)) {
			LOG_ERR("I2C controller %s is not ready", i2c_cr_dev->name);
			rc = -ENOSYS;
		}
	}

	if (rc != 0) {
		goto app_end;
	}

	rc = check_i2c_hw_ready(&pca9555_spec);
	if (rc != 0) {
		goto app_end;
	}

	rc = check_i2c_hw_ready(&ltc2489_spec);
	if (rc != 0) {
		goto app_end;
	}

	rc = check_i2c_hw_ready(&mb_fram_spec);
	if (rc != 0) {
		goto app_end;
	}


app_end:
	LOG_INF("App End");

	return 0;
}
