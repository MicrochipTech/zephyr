/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
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

#define I2C_SMB_GET_DEV(nid) DEVICE_DT_GET(nid),

static const struct device *i2c_smb_ctrls[] = {
	DT_FOREACH_STATUS_OKAY(microchip_xec_i2c_v3, I2C_SMB_GET_DEV)
};

/* Ports on the controllers */
static const struct device *i2c_smb_ports[] = {
	DT_FOREACH_STATUS_OKAY(microchip_xec_i2c_v3_port, I2C_SMB_GET_DEV)
};

#define PCA9555_EVB_NODE     DT_NODELABEL(pca9555_evb)
#define LTC2489_EVB_NODE     DT_NODELABEL(ltc2489_evb)
#define MB85RC256V_FRAM_NODE DT_NODELABEL(mb85rc256v_fram)

const struct i2c_dt_spec pca9555_spec = I2C_DT_SPEC_GET(PCA9555_EVB_NODE);
const struct i2c_dt_spec ltc2489_spec = I2C_DT_SPEC_GET(LTC2489_EVB_NODE);
const struct i2c_dt_spec mb_fram_spec = I2C_DT_SPEC_GET(MB85RC256V_FRAM_NODE);

#define I2C_MAX_MSGS 8
#define I2C_TX_BUF_SIZE 256
#define I2C_RX_BUF_SIZE 256

static struct i2c_msg msgs[I2C_MAX_MSGS];
static uint8_t i2c_tx_buf[I2C_TX_BUF_SIZE];
static uint8_t i2c_rx_buf[I2C_RX_BUF_SIZE];

static int pca9555_test1(const struct i2c_dt_spec *dts);
static int ltc2489_test1(const struct i2c_dt_spec *dts);
static int fram_test1(const struct i2c_dt_spec *dts);

int main(void)
{
	int ctrl_ready_count = 0;
	int port_ready_count = 0;
	int rc = 0;

	memset((void *)msgs, 0, sizeof(msgs));
	memset(i2c_tx_buf, 0x55, I2C_TX_BUF_SIZE);
	memset(i2c_rx_buf, 0xAA, I2C_RX_BUF_SIZE);

	for (size_t i = 0; i < ARRAY_SIZE(i2c_smb_ctrls); i++) {
		const struct device *ctrl_dev = i2c_smb_ctrls[i];

		if (ctrl_dev == NULL) {
			continue;
		}

		if (device_is_ready(ctrl_dev)) {
			ctrl_ready_count++;
			LOG_INF("I2C Controller device %s is ready", ctrl_dev->name);
		} else {
			LOG_ERR("I2C Controller device %s is NOT ready", ctrl_dev->name);
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(i2c_smb_ports); i++) {
		const struct device *port_dev = i2c_smb_ports[i];

		if (port_dev == NULL) {
			continue;
		}

		if (device_is_ready(port_dev)) {
			port_ready_count++;
			LOG_INF("I2C Port device %s is ready", port_dev->name);
		} else {
			LOG_ERR("I2C Port device %s is NOT ready", port_dev->name);
		}
	}

	log_flush();

	if ((ctrl_ready_count != ARRAY_SIZE(i2c_smb_ctrls)) ||
	    (port_ready_count != ARRAY_SIZE(i2c_smb_ports))) {
		LOG_ERR("Skipping tests since some or all drivers are not ready");
		goto app_exit;
	}

	LOG_INF("Test communication with PCA9555");
	rc = pca9555_test1(&pca9555_spec);
	if (rc == 0) {
		LOG_INF("PCA9555 test 1 PASS");
	} else if (rc == -ENOTSUP) {
		LOG_INF("PCA9555 test1 not implemented");
	} else {
		LOG_ERR("PCA9555 test 1 FAIL (%d)", rc);
	}

	LOG_INF("Test communication with LTC2489");
	rc = ltc2489_test1(&ltc2489_spec);
	if (rc == 0) {
		LOG_INF("LTC2489 test 1 PASS");
	} else if (rc == -ENOTSUP) {
		LOG_INF("LTC2489 test 1 not implemented");
	} else {
		LOG_ERR("LTC2489 test 1 FAIL (%d)", rc);
	}

	LOG_INF("Test communication with FRAM");
	rc = fram_test1(&mb_fram_spec);
	if (rc == 0) {
		LOG_INF("FRAM test 1 PASS");
	} else if (rc == -ENOTSUP) {
		LOG_INF("FRAM test 1 not implemented");
	} else {
		LOG_ERR("FRAM test 1 FAIL (%d)", rc);
	}

app_exit:
	LOG_INF("Program End");
	log_flush();

	return 0;
}

static int pca9555_test1(const struct i2c_dt_spec *dts)
{
	int rc = -ENOTSUP;

	if (dts == NULL) {
		return -EINVAL;
	}

	memset(i2c_tx_buf, 0x55, sizeof(i2c_tx_buf));
	memset(i2c_rx_buf, 0xAA, sizeof(i2c_rx_buf));

	i2c_tx_buf[0] = PCA9555_CMD_PORT0_IN;

	rc = i2c_write_read_dt(dts, i2c_tx_buf, 1U, i2c_rx_buf, 2U);
	if (rc != 0) {
		return rc;
	}

	LOG_INF("PCA9555 read ports 0x%0x 0x%0x", i2c_rx_buf[0], i2c_rx_buf[1]);

	return rc;
}

static int ltc2489_test1(const struct i2c_dt_spec *dts)
{
	int rc = -ENOTSUP;

	if (dts == NULL) {
		return -EINVAL;
	}

	return rc;
}

static int fram_test1(const struct i2c_dt_spec *dts)
{
	int rc = -ENOTSUP;

	if (dts == NULL) {
		return -EINVAL;
	}

	memset(i2c_tx_buf, 0x55, sizeof(i2c_tx_buf));
	memset(i2c_rx_buf, 0xAA, sizeof(i2c_rx_buf));

	i2c_tx_buf[0] = 0x43U;
	i2c_tx_buf[1] = 0x21U;
	i2c_tx_buf[2] = 0x01U;
	i2c_tx_buf[3] = 0x02U;
	i2c_tx_buf[4] = 0x03U;
	i2c_tx_buf[5] = 0x04U;

	rc = i2c_write_dt(dts, i2c_tx_buf, 6U);
	if (rc != 0) {
		return rc;
	}

	i2c_tx_buf[0] = 0x43U;
	i2c_tx_buf[1] = 0x21U;

	rc = i2c_write_read_dt(dts, i2c_tx_buf, 2U, i2c_rx_buf, 4U);
	if (rc != 0) {
		return rc;
	}

	rc = memcmp(&i2c_tx_buf[2], i2c_rx_buf, 4U);
	if (rc != 0) {
		rc = -EPERM;
	}

	return rc;
}
