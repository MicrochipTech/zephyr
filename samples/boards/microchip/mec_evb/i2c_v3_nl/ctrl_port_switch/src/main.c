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
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/target/eeprom.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/dt-bindings/i2c/mchp-xec-i2c.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

/* #define APP_TEST_LTC2489 */

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
#ifdef APP_TEST_LTC2489
const struct i2c_dt_spec ltc2489_spec = I2C_DT_SPEC_GET(NODE_LTC2489);
#endif
const struct i2c_dt_spec mb_fram_spec = I2C_DT_SPEC_GET(NODE_FRAM);

static const struct device *i2c_smb_ctrls[] = {
	DT_FOREACH_STATUS_OKAY(microchip_xec_i2c_v3, I2C_SMB_GET_DEV)};

/* Ports on the controllers */
static const struct device *i2c_smb_ports[] = {
	DT_FOREACH_STATUS_OKAY(microchip_xec_i2c_v3_port, I2C_SMB_GET_DEV)};

#define I2C_MAX_MSGS    8
#define I2C_TX_BUF_SIZE 256
#define I2C_RX_BUF_SIZE 256

static struct i2c_msg msgs[I2C_MAX_MSGS];
static uint8_t i2c_tx_buf[I2C_TX_BUF_SIZE];
static uint8_t i2c_rx_buf[I2C_RX_BUF_SIZE];

static int pca9555_test1(const struct i2c_dt_spec *dts, uint8_t port, uint16_t *port_value);
#ifdef APP_TEST_LTC2489
static int ltc2489_test1(const struct i2c_dt_spec *dts);
#endif
static int fram_test1(const struct i2c_dt_spec *dts);
static int fram_test2(const struct i2c_dt_spec *dts);

static volatile bool run = false;

#ifdef APP_TEST_LTC2489
static volatile bool app_dbg_halt = true;
#endif

int main(void)
{
	uint64_t test_loops = 0;
	uint64_t pca9555_errors = 0;
	uint64_t fram_errors = 0;
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

	if ((ctrl_ready_count > 0) && (port_ready_count > 0)) {
		LOG_INF("Enter test loop");
		run = true;
	} else {
		LOG_ERR("ctrl_ready_count = %u port_ready_count = %u",
		        ctrl_ready_count, port_ready_count);
	}

	log_flush();

	while (run) {
		test_loops++;

		rc = pca9555_test1(&pca9555_spec, PCA9555_CMD_PORT0_IN, NULL);
		if (rc != 0) {
			pca9555_errors++;
#if 0
			LOG_ERR("PCA9555 test error (%d)", rc);
			break;
#endif
		}

		rc = fram_test1(&mb_fram_spec);
		if (rc != 0) {
			fram_errors++;
#if 0
			LOG_ERR("FRAM test1 error (%d)", rc);
			break;
#endif
		}

		rc = pca9555_test1(&pca9555_spec, PCA9555_CMD_PORT1_IN, NULL);
		if (rc != 0) {
			pca9555_errors++;
#if 0
			LOG_ERR("PCA9555 test error (%d)", rc);
			break;
#endif
		}

		rc = fram_test2(&mb_fram_spec);
		if (rc != 0) {
			fram_errors++;
#if 0
			LOG_ERR("FRAM test2 error (%d)", rc);
			break;
#endif
		}

#ifdef APP_TEST_LTC2489
		rc = ltc2489_test1(&ltc2489_spec);
		if (rc != 0) {
			LOG_ERR("LTC2489 test error (%d)", rc);
			break;
		}
#endif
	};

	LOG_INF("Test loop exit: loops = %llu", test_loops);
	LOG_INF("Program End");
	log_flush();

	return 0;
}

static int pca9555_test1(const struct i2c_dt_spec *dts, uint8_t port, uint16_t *port_value)
{
	int rc = -ENOTSUP;

	if (dts == NULL) {
		return -EINVAL;
	}

	memset(i2c_tx_buf, 0x55, sizeof(i2c_tx_buf));
	memset(i2c_rx_buf, 0xAA, sizeof(i2c_rx_buf));

	i2c_tx_buf[0] = port;

	rc = i2c_write_read_dt(dts, i2c_tx_buf, 1U, i2c_rx_buf, 2U);
	if (rc != 0) {
		return rc;
	}

	if (port_value != NULL) {
		*port_value = ((uint16_t)i2c_rx_buf[1] << 8) | i2c_rx_buf[0];
	}

	return rc;
}

#ifdef APP_TEST_LTC2489
/* LTC2489 is a troublesome device
 * It will NAK its address if it is busy. Power on reset causes it to be busy.
 */
static int ltc2489_test1(const struct i2c_dt_spec *dts)
{
	int rc = -ENOTSUP;
	uint32_t adc_retry_count = 0, reading = 0;

	if (dts == NULL) {
		return -EINVAL;
	}

	/* Read ADC channel 0. Selecting the channel initiates a reading */
	i2c_tx_buf[0] = 0xb0U; /* 1011_0000 selects channel 0 as single ended */

	rc = i2c_write_dt(dts, i2c_tx_buf, 1U);
	if (rc != 0) {
		while (app_dbg_halt) {
			;
		}
		LOG_ERR("I2C write to LTC2489 channel select error (%d)", rc);
		return rc;
	}

	/* LTC2489 will NAK while it is converting */
	k_sleep(K_MSEC(LTC2489_ADC_CONV_TIME_MS));

	do {
		i2c_rx_buf[0] = 0x55U;
		i2c_rx_buf[1] = 0x55U;
		i2c_rx_buf[2] = 0x55U;

		rc = i2c_read_dt(dts, i2c_rx_buf, 3U);
		if (rc != 0) {
			adc_retry_count++;
		}

	} while ((rc != 0) && (adc_retry_count < LTC2489_ADC_READ_RETRIES));

	if (rc == 0) {
		reading = ((uint32_t)i2c_rx_buf[0] + ((uint32_t)i2c_rx_buf[1] << 8) +
			   ((uint32_t)i2c_rx_buf[2] << 16));
		LOG_INF("LTC2489 conversion done: raw reading = 0x%0x", reading);
	} else {
		LOG_ERR("LTC2489 conversion timeout: FAIL");
	}

	return rc;
}
#endif

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

static int fram_test2(const struct i2c_dt_spec *dts)
{
	int rc = -ENOTSUP;

	if (dts == NULL) {
		return -EINVAL;
	}

	memset(i2c_tx_buf, 0x55, sizeof(i2c_tx_buf));
	memset(i2c_rx_buf, 0xAA, sizeof(i2c_rx_buf));

	i2c_tx_buf[0] = 0x12U;
	i2c_tx_buf[1] = 0x30U;
	for (uint32_t i = 0; i < 32U; i++) {
		i2c_tx_buf[i + 2U] = (uint8_t)(i % 256U);
	}

	rc = i2c_write_dt(dts, i2c_tx_buf, 34U);
	if (rc != 0) {
		return rc;
	}

	i2c_tx_buf[0] = 0x12U;
	i2c_tx_buf[1] = 0x30U;

	rc = i2c_write_read_dt(dts, i2c_tx_buf, 2U, i2c_rx_buf, 32U);
	if (rc != 0) {
		return rc;
	}

	rc = memcmp(&i2c_tx_buf[2], i2c_rx_buf, 32U);
	if (rc != 0) {
		rc = -EPERM;
	}

	return rc;
}
