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
#include <zephyr/sys/sys_io.h>

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
}

static int i2c_target0_buf_rd_req_cb(struct i2c_target_config *config, uint8_t **ptr,
				     uint32_t *len)
{
	if (!config || !ptr || !*ptr || !len) {
		return -EINVAL;
	}

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
	struct i2c_target_config *tcfg = NULL;
	int ret = 0;

	if (!i2c_dev || get_target_config(target_id)) {
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