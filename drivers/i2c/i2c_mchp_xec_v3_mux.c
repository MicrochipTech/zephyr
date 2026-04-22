/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_i2c_v3_mux

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/logging/log.h>

#include "i2c-priv.h"
#include "i2c_mchp_xec_v3.h"

LOG_MODULE_REGISTER(i2c_mchp_xec_v3_mux, CONFIG_I2C_LOG_LEVEL);

struct xec_v3_mux_config {
	const struct device *ctrl;             /* backing controller */
	const struct pinctrl_dev_config *pcfg; /* SCL/SDA pins for this port */
	uint32_t default_bitrate;              /* clock-frequency DT prop */
	uint8_t port;                          /* port-sel DT prop */
};

struct xec_v3_mux_data {
	uint32_t bitrate; /* current requested bitrate, may be retargeted via configure() */
};

static uint32_t xec_v3_mux_freq_from_cfg(uint32_t dev_config)
{
	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		return KHZ(100);
	case I2C_SPEED_FAST:
		return KHZ(400);
	case I2C_SPEED_FAST_PLUS:
		return MHZ(1);
	default:
		return 0;
	}
}

static uint32_t xec_v3_mux_cfg_from_freq(uint32_t freq_hz)
{
	if (freq_hz == KHZ(100)) {
		return I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
	}
	if (freq_hz == KHZ(400)) {
		return I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_CONTROLLER;
	}
	if (freq_hz == MHZ(1)) {
		return I2C_SPEED_SET(I2C_SPEED_FAST_PLUS) | I2C_MODE_CONTROLLER;
	}
	return 0;
}

static int xec_v3_mux_configure(const struct device *dev, uint32_t dev_config)
{
	const struct xec_v3_mux_config *cfg = dev->config;
	struct xec_v3_mux_data *data = dev->data;
	uint32_t freq_hz;
	int ret;

	if (!(dev_config & I2C_MODE_CONTROLLER)) {
		return -ENOTSUP;
	}
	if (dev_config & I2C_ADDR_10_BITS) {
		return -ENOTSUP;
	}

	freq_hz = xec_v3_mux_freq_from_cfg(dev_config);
	if (freq_hz == 0) {
		return -EINVAL;
	}

	xec_v3_ctrl_lock(cfg->ctrl);
	ret = xec_v3_ctrl_set_port_freq_locked(cfg->ctrl, cfg->port, freq_hz);
	if (ret == 0) {
		data->bitrate = freq_hz;
	}
	xec_v3_ctrl_unlock(cfg->ctrl);
	return ret;
}

static int xec_v3_mux_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct xec_v3_mux_data *data = dev->data;
	uint32_t cfg;

	if (dev_config == NULL) {
		return -EINVAL;
	}
	cfg = xec_v3_mux_cfg_from_freq(data->bitrate);
	if (cfg == 0) {
		return -EIO;
	}
	*dev_config = cfg;
	return 0;
}

static int xec_v3_mux_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			       uint16_t addr)
{
	const struct xec_v3_mux_config *cfg = dev->config;
	struct xec_v3_mux_data *data = dev->data;
	int ret;

	xec_v3_ctrl_lock(cfg->ctrl);
	ret = xec_v3_ctrl_set_port_freq_locked(cfg->ctrl, cfg->port, data->bitrate);
	if (ret == 0) {
		ret = xec_v3_ctrl_transfer_locked(cfg->ctrl, msgs, num_msgs, addr);
	}
	xec_v3_ctrl_unlock(cfg->ctrl);
	return ret;
}

#ifdef CONFIG_I2C_CALLBACK
static int xec_v3_mux_transfer_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				  uint16_t addr, i2c_callback_t cb, void *userdata)
{
	const struct xec_v3_mux_config *cfg = dev->config;
	struct xec_v3_mux_data *data = dev->data;
	int ret;

	xec_v3_ctrl_lock(cfg->ctrl);
	ret = xec_v3_ctrl_set_port_freq_locked(cfg->ctrl, cfg->port, data->bitrate);
	if (ret != 0) {
		xec_v3_ctrl_unlock(cfg->ctrl);
		return ret;
	}
	ret = xec_v3_ctrl_transfer_cb_locked(cfg->ctrl, msgs, num_msgs, addr, cb, userdata);
	if (ret != 0) {
		/* On error, lock ownership stays with us; release it. On success,
		 * the async completion path owns the unlock.
		 */
		xec_v3_ctrl_unlock(cfg->ctrl);
	}
	return ret;
}
#endif

#ifdef CONFIG_I2C_TARGET
static int xec_v3_mux_target_register(const struct device *dev,
				      struct i2c_target_config *target_cfg)
{
	const struct xec_v3_mux_config *cfg = dev->config;
	struct xec_v3_mux_data *data = dev->data;
	int ret;

	xec_v3_ctrl_lock(cfg->ctrl);
	ret = xec_v3_ctrl_set_port_freq_locked(cfg->ctrl, cfg->port, data->bitrate);
	if (ret == 0) {
		ret = xec_v3_ctrl_target_register_locked(cfg->ctrl, target_cfg);
	}
	xec_v3_ctrl_unlock(cfg->ctrl);
	return ret;
}

static int xec_v3_mux_target_unregister(const struct device *dev,
					struct i2c_target_config *target_cfg)
{
	const struct xec_v3_mux_config *cfg = dev->config;
	int ret;

	xec_v3_ctrl_lock(cfg->ctrl);
	ret = xec_v3_ctrl_target_unregister_locked(cfg->ctrl, target_cfg);
	xec_v3_ctrl_unlock(cfg->ctrl);
	return ret;
}
#endif

static int xec_v3_mux_recover_bus(const struct device *dev)
{
	const struct xec_v3_mux_config *cfg = dev->config;
	struct xec_v3_mux_data *data = dev->data;
	int ret;

	xec_v3_ctrl_lock(cfg->ctrl);
	ret = xec_v3_ctrl_set_port_freq_locked(cfg->ctrl, cfg->port, data->bitrate);
	if (ret == 0) {
		ret = xec_v3_ctrl_recover_bus_locked(cfg->ctrl);
	}
	xec_v3_ctrl_unlock(cfg->ctrl);
	return ret;
}

static int xec_v3_mux_init(const struct device *dev)
{
	const struct xec_v3_mux_config *cfg = dev->config;
	struct xec_v3_mux_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->ctrl)) {
		LOG_ERR("controller %s not ready", cfg->ctrl->name);
		return -ENODEV;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl apply failed (%d)", ret);
		return ret;
	}

	data->bitrate = cfg->default_bitrate;
	return 0;
}

static DEVICE_API(i2c, xec_v3_mux_driver_api) = {
	.configure = xec_v3_mux_configure,
	.get_config = xec_v3_mux_get_config,
	.transfer = xec_v3_mux_transfer,
	.recover_bus = xec_v3_mux_recover_bus,
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = xec_v3_mux_transfer_cb,
#endif
#ifdef CONFIG_I2C_TARGET
	.target_register = xec_v3_mux_target_register,
	.target_unregister = xec_v3_mux_target_unregister,
#endif
};

#define XEC_V3_MUX_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct xec_v3_mux_data xec_v3_mux_data_##n;                                         \
	static const struct xec_v3_mux_config xec_v3_mux_cfg_##n = {                               \
		.ctrl = DEVICE_DT_GET(DT_INST_PHANDLE(n, controller)),                             \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.default_bitrate = DT_INST_PROP(n, clock_frequency),                               \
		.port = DT_INST_PROP(n, port_sel),                                                 \
	};                                                                                         \
	I2C_DEVICE_DT_INST_DEFINE(n, xec_v3_mux_init, NULL, &xec_v3_mux_data_##n,                  \
				  &xec_v3_mux_cfg_##n, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,      \
				  &xec_v3_mux_driver_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_V3_MUX_INIT)
