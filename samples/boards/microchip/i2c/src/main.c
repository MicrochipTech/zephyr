/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <soc.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
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

static const struct device *i2c0_dev = DEVICE_DT_GET(DT_ALIAS(i2c0));
static const struct device *i2c1_dev = DEVICE_DT_GET(DT_ALIAS(i2c1));

uint8_t i2c_wr_buf[64];
uint8_t i2c_rd_buf[64];

#ifdef CONFIG_I2C_TARGET

static int i2c1_targ_read_req_cb(struct i2c_target_config *config, uint8_t *val);
static int i2c1_targ_read_proc_cb(struct i2c_target_config *config, uint8_t *val);

static int i2c1_targ_write_req_cb(struct i2c_target_config *config);
static int i2c1_targ_write_recv_cb(struct i2c_target_config *config, uint8_t val);

static int i2c1_targ_stop_cb(struct i2c_target_config *config);

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
static void i2c1_targ_buf_write_recv_cb(struct i2c_target_config *config, uint8_t *ptr,
					uint32_t len);
static int i2c1_targ_buf_read_req_cb(struct i2c_target_config *config, uint8_t **ptr,
				     uint32_t *len);
#endif

const struct i2c_target_callbacks i2c1_targ_cbs = {
	.write_requested = i2c1_targ_write_req_cb,
	.write_received = i2c1_targ_write_recv_cb,
	.read_requested = i2c1_targ_read_req_cb,
	.read_processed = i2c1_targ_read_proc_cb,
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	.buf_write_received = i2c1_targ_buf_write_recv_cb,
	.buf_read_requested = i2c1_targ_buf_read_req_cb,
#endif
	.stop = i2c1_targ_stop_cb,
};

struct i2c_target_config app_i2c_targets[] = {
	{
		.flags = 0,
		.address = 0x40u,
		.callbacks = &i2c1_targ_cbs,
	},
	{
		.flags = 0,
		.address = 0x41u,
		.callbacks = &i2c1_targ_cbs,
	},
};

static int config_i2c_as_target(const struct device *i2c_dev, struct i2c_target_config *targets,
				uint8_t num_targets);
#endif /* CONFIG_I2C_TARGET */

int main(void)
{
	uint32_t i2c0_config = 0;
	uint16_t i2c_addr = 0;
	size_t nread = 0, nwrite = 0;
	int rc = 0;

	if (!device_is_ready(i2c0_dev)) {
		LOG_ERR("I2C 0 device [%s] not ready!", i2c0_dev->name);
		return -1;
	}

	if (!device_is_ready(i2c1_dev)) {
		LOG_ERR("I2C 1 device [%s] not ready!", i2c1_dev->name);
		return -2;
	}

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

#ifdef CONFIG_I2C_TARGET
	rc = config_i2c_as_target(i2c1_dev, app_i2c_targets, 2u);
	if (rc != 0) {
		LOG_ERR("Configuring %s as I2C target failed: (%d)", i2c1_dev->name, rc);
		return -5;
	}
#endif

	LOG_INF("Program End");

	return 0;
}

#ifdef CONFIG_I2C_TARGET

static int config_i2c_as_target(const struct device *i2c_dev, struct i2c_target_config *targets,
				uint8_t num_targets)
{
	int rc = 0, ret_val = 0;

	if ((i2c_dev == NULL) || (targets == NULL)) {
		return -EINVAL;
	}

	for (uint8_t n = 0; n < num_targets; n++) {
		rc = i2c_target_register(i2c_dev, &targets[n]);
		if (rc != 0) {
			ret_val = -EIO;
			LOG_ERR("Register target %u failed (%d)", n, rc);
		}
	}

	return ret_val;
}

static int i2c1_targ_read_req_cb(struct i2c_target_config *config, uint8_t *val)
{
	return -ENOTSUP;
}

static int i2c1_targ_read_proc_cb(struct i2c_target_config *config, uint8_t *val)
{
	return -ENOTSUP;
}

static int i2c1_targ_write_req_cb(struct i2c_target_config *config)
{
	return -ENOTSUP;
}

static int i2c1_targ_write_recv_cb(struct i2c_target_config *config, uint8_t val)
{
	return -ENOTSUP;
}

static int i2c1_targ_stop_cb(struct i2c_target_config *config)
{
	return -ENOTSUP;
}

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
static void i2c1_targ_buf_write_recv_cb(struct i2c_target_config *config, uint8_t *ptr,
					uint32_t len)
{
	return;
}

static int i2c1_targ_buf_read_req_cb(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len)
{
	return -ENOTSUP;
}
#endif /* CONFIG_I2C_TARGET_BUFFER_MODE */
#endif /* CONFIG_I2C_TARGET */
