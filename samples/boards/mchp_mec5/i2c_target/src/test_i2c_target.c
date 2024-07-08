/*
 * Copyright (c) 2024 Microchip Technology, Inc.
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
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(app);

#include "test_i2c_target.h"

static int i2c_target0_wr_req_cb(struct i2c_target_config *config);
static int i2c_target0_wr_rcvd_cb(struct i2c_target_config *config, uint8_t val);

static int i2c_target0_rd_req_cb(struct i2c_target_config *config, uint8_t *val);
static int i2c_target0_rd_proc_cb(struct i2c_target_config *config, uint8_t *val);

static int i2c_target0_stop_cb(struct i2c_target_config *config);

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
static void i2c_target0_buf_wr_rcvd_cb(struct i2c_target_config *config, uint8_t *ptr,
				       uint32_t len);
static int i2c_target0_buf_rd_req_cb(struct i2c_target_config *config, uint8_t **ptr,
				     uint32_t *len);
#endif

#ifdef I2C_TARGET_USE_BOTH_TARGETS
static int i2c_target1_wr_req_cb(struct i2c_target_config *config);
static int i2c_target1_wr_rcvd_cb(struct i2c_target_config *config, uint8_t val);

static int i2c_target1_rd_req_cb(struct i2c_target_config *config, uint8_t *val);
static int i2c_target1_rd_proc_cb(struct i2c_target_config *config, uint8_t *val);

static int i2c_target1_stop_cb(struct i2c_target_config *config);

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
static void i2c_target1_buf_wr_rcvd_cb(struct i2c_target_config *config, uint8_t *ptr,
				       uint32_t len);
static int i2c_target1_buf_rd_req_cb(struct i2c_target_config *config, uint8_t **ptr,
				     uint32_t *len);
#endif
#endif

const struct i2c_target_callbacks i2c_target0_cb = {
	.write_requested = i2c_target0_wr_req_cb,
	.read_requested = i2c_target0_rd_req_cb,
	.write_received = i2c_target0_wr_rcvd_cb,
	.read_processed = i2c_target0_rd_proc_cb,
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	.buf_write_received = i2c_target0_buf_wr_rcvd_cb,
	.buf_read_requested = i2c_target0_buf_rd_req_cb,
#endif
	.stop = i2c_target0_stop_cb,
};

#ifdef I2C_TARGET_USE_BOTH_TARGETS
const struct i2c_target_callbacks i2c_target1_cb = {
	.write_requested = i2c_target1_wr_req_cb,
	.read_requested = i2c_target1_rd_req_cb,
	.write_received = i2c_target1_wr_rcvd_cb,
	.read_processed = i2c_target1_rd_proc_cb,
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	.buf_write_received = i2c_target1_buf_wr_rcvd_cb,
	.buf_read_requested = i2c_target1_buf_rd_req_cb,
#endif
	.stop = i2c_target1_stop_cb,
};
#endif

struct i2c_target_config i2c_target0 = {
	.flags = 0,
	.address = I2C_TARGET_MODE_ADDR0,
	.callbacks = &i2c_target0_cb,
};

#ifdef I2C_TARGET_USE_BOTH_TARGETS
struct i2c_target_config i2c_target1 = {
	.flags = 0,
	.address = I2C_TARGET_MODE_ADDR1,
	.callbacks = &i2c_target1_cb,
}
#endif

size_t target_test_wr_len;
size_t target_test_wr_rx_len;

size_t target_test_rd_len;
size_t target_test_rd_tx_len;

#define TM_TX_BUF_SIZE 256
uint8_t tm_tx_buf[TM_TX_BUF_SIZE];

#define TM_RX_BUF_SIZE 256
uint8_t tm_rx_buf[TM_RX_BUF_SIZE];

static int i2c_target0_wr_req_cb(struct i2c_target_config *config)
{
	return 0;
}

static int i2c_target0_wr_rcvd_cb(struct i2c_target_config *config, uint8_t val)
{
	return 0;
}

static int i2c_target0_rd_req_cb(struct i2c_target_config *config, uint8_t *val)
{
	return 0;
}

static int i2c_target0_rd_proc_cb(struct i2c_target_config *config, uint8_t *val)
{
	return 0;
}

static int i2c_target0_stop_cb(struct i2c_target_config *config)
{
	return 0;
}

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
static void i2c_target0_buf_wr_rcvd_cb(struct i2c_target_config *config, uint8_t *ptr,
				       uint32_t len)
{
	if (!config || !ptr || !len) {
		return;
	}

	target_test_wr_rx_len = len;

	LOG_INF("TM CB Write received: cfg=%p buf=%p len=%u", config, ptr, len);

	if (ptr && len) {
		memcpy(tm_rx_buf, ptr, len);
	}
}

static int i2c_target0_buf_rd_req_cb(struct i2c_target_config *config, uint8_t **ptr,
				     uint32_t *len)
{
	if (!config || !ptr || !*ptr || !len) {
		return -EINVAL;
	}

	LOG_INF("TM CB Read request");

	tm_tx_buf[0] = 0x11;
	tm_tx_buf[1] = 0x22;
	tm_tx_buf[2] = 0x33;
	tm_tx_buf[3] = 0x34;

	*ptr = tm_tx_buf;
	*len = 4;

	return 0;
}
#endif


static struct i2c_target_config *get_target_config(uint8_t target_id)
{
	switch (target_id) {
	case 0:
		return &i2c_target0;
#ifdef I2C_TARGET_USE_BOTH_TARGETS
	case 1:
		return &i2c_target1;
#endif
	default:
		return NULL;
	}
}

int test_i2c_target_register(const struct device *i2c_dev, bool register_target,
			     uint8_t target_id)
{
	struct i2c_target_config *tcfg = get_target_config(target_id);
	int ret = 0;

	if (!i2c_dev || !tcfg) {
		return -EINVAL;
	}

	if (register_target) {
		ret = i2c_target_register(i2c_dev, tcfg);
		if (ret) {
			return ret;
		}
	} else {
		ret = i2c_target_unregister(i2c_dev, tcfg);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

int test_i2c_target_write(const struct device *i2c_cm_dev, const struct device *i2c_tm_dev,
			  uint16_t tm_addr, uint8_t *data, size_t datasz)
{
	struct i2c_msg msg = {0};
	int ret = 0;

	LOG_INF("Target Write: %u bytes to I2C address 0x%0x", datasz, tm_addr);

	msg.buf = data;
	msg.len = datasz;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	target_test_wr_len = datasz;

	/* synchronous transfer */
	ret = i2c_transfer(i2c_cm_dev, &msg, 1, tm_addr);
	if (ret) {
		LOG_ERR("Target write error (%d)", ret);
		return ret;
	}

	if (target_test_wr_rx_len != datasz) {
		LOG_INF("Target captured only %u bytes of %u from External Controller",
			target_test_wr_rx_len, datasz);
	}

	ret = memcmp(tm_rx_buf, data, target_test_wr_rx_len);
	if (ret) {
		ret = -EILSEQ;
		LOG_ERR("Target write data != send data");
	} else {
		LOG_INF("Target captured bytes match");
	}

	return ret;
}

int test_i2c_target_read(const struct device *i2c_cm_dev, const struct device *i2c_tm_dev,
			 uint16_t tm_addr, uint8_t *data, size_t datasz)
{
	struct i2c_msg msgs[2] = {0};
	int ret = 0;

	msgs[0].buf = data;
	msgs[0].len = datasz;
	msgs[0].flags = I2C_MSG_READ | I2C_MSG_STOP;

	target_test_rd_len = datasz;

	/* synchronous transfer */
	ret = i2c_transfer(i2c_cm_dev, msgs, 1, tm_addr);

	return ret;
}

