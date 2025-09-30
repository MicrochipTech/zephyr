/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sys/errno.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <soc.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_xec_i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define PCA9555_CMD_PORT0_IN  0
#define PCA9555_CMD_PORT1_IN  1u
#define PCA9555_CMD_PORT0_OUT 2u
#define PCA9555_CMD_PORT1_OUT 3u
#define PCA9555_CMD_PORT0_POL 4u
#define PCA9555_CMD_PORT1_POL 5u
#define PCA9555_CMD_PORT0_CFG 6u
#define PCA9555_CMD_PORT1_CFG 7u

#define PCA9555_I2C_ADDR DT_REG_ADDR(DT_NODELABEL(pca9555_evb))
#define LTC2489_I2C_ADDR DT_REG_ADDR(DT_NODELABEL(ltc2489_evb))
#define FRAM_I2C_ADDR    0x50u

static const struct device *i2c0_dev = DEVICE_DT_GET(DT_ALIAS(i2c0));
static const struct device *i2c1_dev = DEVICE_DT_GET(DT_ALIAS(i2c1));

#if DT_HAS_COMPAT_STATUS_OKAY(microchip_xec_i2c_nl)
#define APP_HAS_I2C_NL
static const struct device *i2c_nl_dev = DEVICE_DT_GET(DT_ALIAS(i2c_nl));
#endif

volatile uint32_t spin_val;

uint8_t i2c_wr_buf[64];
uint8_t i2c_rd_buf[64];

#define I2C1_TARG_ADDR1 0x40u
#define I2C1_TARG_ADDR2 0x41u

#define TARGET_I2C_ADDR2_RX_BUF_SIZE 10
#define TARGET_I2C_ADDR2_TX_BUF_SIZE 10

uint8_t i2c0_tx_buf[256];
uint8_t i2c0_rx_buf[256];

#ifdef APP_HAS_I2C_NL
static int i2c_nl_test1(const struct device *i2c_dev);
#endif

static int fill_buf(uint8_t *buf, uint32_t buflen, uint8_t val, uint8_t flags);

int main(void)
{
	uint32_t i2c0_config = 0;
	uint16_t i2c_addr = 0;
	size_t nread = 0, nwrite = 0;
#ifdef APP_HAS_I2C_NL
	bool i2c_nl_ready = false;
#endif
	int rc = 0;

	if (!device_is_ready(i2c0_dev)) {
		LOG_ERR("I2C 0 device [%s] not ready!", i2c0_dev->name);
		return -1;
	}

	if (!device_is_ready(i2c1_dev)) {
		LOG_ERR("I2C 1 device [%s] not ready!", i2c1_dev->name);
		return -2;
	}

#ifdef APP_HAS_I2C_NL
	if (!device_is_ready(i2c_nl_dev)) {
		LOG_ERR("I2C 2 device [%s] not ready!", i2c_nl_dev->name);
		return -2;
	} else {
		i2c_nl_ready = true;
	}
#endif

	LOG_INF("Configure I2C 0 as controller at 100KHz");

	i2c0_config = I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD);

	rc = i2c_configure(i2c0_dev, i2c0_config);
	if (rc != 0) {
		LOG_ERR("I2C 0 device config failed");
		return -3;
	}

	LOG_INF("PCA9555 on I2C 0: Read the two data registers");

	memset(i2c_rd_buf, 0x55, sizeof(i2c_rd_buf));

	i2c_addr = (uint16_t)PCA9555_I2C_ADDR;
	i2c_wr_buf[0] = PCA9555_CMD_PORT0_IN;

	nread = 2u;
	nwrite = 1u;

	rc = i2c_write_read(i2c0_dev, i2c_addr, (const void *)i2c_wr_buf, nwrite,
			    (void *)i2c_rd_buf, nread);
	if (rc != 0) {
		LOG_ERR("i2c_write_read error (%d)", rc);
		return -4;
	}

#ifdef APP_HAS_I2C_NL
	rc = i2c_nl_test1(i2c_nl_dev);
	LOG_INF("I2C-NL test 1 returned (%d)", rc);
#endif

	LOG_INF("Program End");

	return 0;
}

static int fill_buf(uint8_t *buf, uint32_t buflen, uint8_t val, uint8_t flags)
{
	if ((buf == NULL) || (buflen == 0)) {
		return -EINVAL;
	}

	if (flags == 0) {
		memset((void *)buf, val, buflen);
	} else {
		for (uint32_t n = 0; n < buflen; n++) {
			buf[n] = val;
			val++;
		}
	}

	return 0;
}

#ifdef APP_HAS_I2C_NL
#define TEST_ORDER_WR_RD
 /* HW issue exists independent of transaction order. */

static int i2c_nl_test1(const struct device *i2c_dev)
{
	struct i2c_msg msgs[4] = {0};
	int rc = 0;
	uint16_t i2c_addr = 0;
	uint8_t nmsgs = 0;

	fill_buf(i2c0_tx_buf, 16u, 0x11u, 1u);

#ifdef TEST_ORDER_WR_RD
	LOG_INF("Test I2C-NL one write message");
	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 1u;

	msgs[0].buf = i2c0_tx_buf;
	msgs[0].len = 4u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);

	LOG_INF("Test I2C-NL one read message");
	memset((void *)i2c0_rx_buf, 0xAA, 8);

	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 1u;

	msgs[0].buf = i2c0_rx_buf;
	msgs[0].len = 4u;
	msgs[0].flags = I2C_MSG_READ | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);
#else
	LOG_INF("Test I2C-NL one read message");
	memset((void *)i2c0_rx_buf, 0xAA, 8);

	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 1u;

	msgs[0].buf = i2c0_rx_buf;
	msgs[0].len = 4u;
	msgs[0].flags = I2C_MSG_READ | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);

	LOG_INF("Test I2C-NL one write message");
	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 1u;

	msgs[0].buf = i2c0_tx_buf;
	msgs[0].len = 4u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);
#endif
	LOG_INF("Test I2C-NL two write messages");
	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 2u;

	msgs[0].buf = i2c0_tx_buf;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = &i2c0_tx_buf[2];
	msgs[1].len = 4u;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);

	LOG_INF("Test I2C-NL Write-Read");
	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 2u;

	msgs[0].buf = i2c0_tx_buf;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = i2c0_rx_buf;
	msgs[1].len = 8u;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);

	LOG_INF("Test I2C-NL Write message > driver xfrbuf size");
	fill_buf(i2c0_tx_buf, 66u, 0, 1);

	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 1u;

	msgs[0].buf = i2c0_tx_buf;
	msgs[0].len = 66u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);

	return 0;
}
#endif /* APP_HAS_I2C_NL */
