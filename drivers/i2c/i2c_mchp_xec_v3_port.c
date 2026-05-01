/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_i2c_v3_port

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_mchp_xec_v3_port, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"
#include "i2c_mchp_xec_v3.h"

struct i2c_xec_v3_port_cfg {
	const struct device *ctrl;
	uint32_t bitrate;
	uint8_t port_idx;
	const struct pinctrl_dev_config *pcfg;
};

static int i2c_xec_v3_port_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	uint32_t freq_hz;
	int ret;

	if ((dev_config & I2C_MODE_CONTROLLER) == 0) {
		return -ENOTSUP;
	}
	if ((dev_config & I2C_ADDR_10_BITS) != 0) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		freq_hz = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		freq_hz = KHZ(400);
		break;
	case I2C_SPEED_FAST_PLUS:
		freq_hz = MHZ(1);
		break;
	default:
		return -EINVAL;
	}

	mchp_xec_i2c_ctrl_mutex_lock(cfg->ctrl);
	ret = mchp_xec_i2c_ctrl_configure(cfg->ctrl, cfg->port_idx, freq_hz);
	mchp_xec_i2c_ctrl_mutex_unlock(cfg->ctrl);
	return ret;
}

static int i2c_xec_v3_port_get_config(const struct device *dev, uint32_t *dev_config)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	uint32_t speed;
	int ret;

	if (dev_config == NULL) {
		return -EINVAL;
	}

	mchp_xec_i2c_ctrl_mutex_lock(cfg->ctrl);
	ret = mchp_xec_i2c_ctrl_get_speed(cfg->ctrl, cfg->port_idx, &speed);
	mchp_xec_i2c_ctrl_mutex_unlock(cfg->ctrl);

	if (ret == 0) {
		*dev_config = I2C_MODE_CONTROLLER | speed;
	}
	return ret;
}

static int i2c_xec_v3_port_transfer(const struct device *dev, struct i2c_msg *msgs,
				    uint8_t num_msgs, uint16_t addr)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	int ret;

	mchp_xec_i2c_ctrl_mutex_lock(cfg->ctrl);
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		mchp_xec_i2c_ctrl_mutex_unlock(cfg->ctrl);
		return ret;
	}
	ret = mchp_xec_i2c_ctrl_transfer(cfg->ctrl, cfg->port_idx, cfg->bitrate, msgs, num_msgs,
					 addr);
	mchp_xec_i2c_ctrl_mutex_unlock(cfg->ctrl);
	return ret;
}

#ifdef CONFIG_I2C_CALLBACK
static int i2c_xec_v3_port_transfer_cb(const struct device *dev, struct i2c_msg *msgs,
				       uint8_t num_msgs, uint16_t addr, i2c_callback_t cb,
				       void *userdata)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	int ret;

	/* The controller holds bus_lock across the async transfer and releases
	 * it from its completion ISR before invoking cb. If begin_xfer fails
	 * synchronously we release it here.
	 */
	mchp_xec_i2c_ctrl_mutex_lock(cfg->ctrl);
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		mchp_xec_i2c_ctrl_mutex_unlock(cfg->ctrl);
		return ret;
	}
	ret = mchp_xec_i2c_ctrl_transfer_cb(cfg->ctrl, cfg->port_idx, cfg->bitrate, msgs, num_msgs,
					    addr, cb, userdata);
	if (ret != 0) {
		mchp_xec_i2c_ctrl_mutex_unlock(cfg->ctrl);
	}
	return ret;
}
#endif

static int i2c_xec_v3_port_recover_bus(const struct device *dev)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	int ret;

	mchp_xec_i2c_ctrl_mutex_lock(cfg->ctrl);
	ret = mchp_xec_i2c_ctrl_recover_bus(cfg->ctrl, cfg->port_idx, cfg->bitrate, cfg->pcfg);
	mchp_xec_i2c_ctrl_mutex_unlock(cfg->ctrl);
	return ret;
}

#ifdef CONFIG_I2C_TARGET
static int i2c_xec_v3_port_target_register(const struct device *dev, struct i2c_target_config *tcfg)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	int ret;

	if (tcfg == NULL) {
		return -EINVAL;
	}

	mchp_xec_i2c_ctrl_mutex_lock(cfg->ctrl);
	/* Apply pinctrl so the target's port is actually muxed to the
	 * controller. A target on a port that is not currently active won't
	 * receive traffic, but pinctrl must still be applied once so the
	 * muxing is latched before any later port switch reprograms OA.
	 */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		goto out;
	}
	ret = mchp_xec_i2c_ctrl_target_register(cfg->ctrl, cfg->port_idx, tcfg);
out:
	mchp_xec_i2c_ctrl_mutex_unlock(cfg->ctrl);
	return ret;
}

static int i2c_xec_v3_port_target_unregister(const struct device *dev,
					     struct i2c_target_config *tcfg)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	int ret;

	if (tcfg == NULL) {
		return -EINVAL;
	}

	mchp_xec_i2c_ctrl_mutex_lock(cfg->ctrl);
	ret = mchp_xec_i2c_ctrl_target_unregister(cfg->ctrl, cfg->port_idx, tcfg);
	mchp_xec_i2c_ctrl_mutex_unlock(cfg->ctrl);
	return ret;
}
#endif

static DEVICE_API(i2c, i2c_xec_v3_port_api) = {
	.configure = i2c_xec_v3_port_configure,
	.get_config = i2c_xec_v3_port_get_config,
	.transfer = i2c_xec_v3_port_transfer,
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = i2c_xec_v3_port_transfer_cb,
#endif
	.recover_bus = i2c_xec_v3_port_recover_bus,
#ifdef CONFIG_I2C_TARGET
	.target_register = i2c_xec_v3_port_target_register,
	.target_unregister = i2c_xec_v3_port_target_unregister,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

static int i2c_xec_v3_port_init(const struct device *dev)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	uint32_t i2c_cfg;
	int ret;

	if (!device_is_ready(cfg->ctrl)) {
		LOG_ERR("i2c v3 port: controller %s not ready", cfg->ctrl->name);
		return -ENODEV;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("i2c v3 port: pinctrl apply failed (%d)", ret);
		return ret;
	}

	i2c_cfg = I2C_MODE_CONTROLLER | i2c_map_dt_bitrate(cfg->bitrate);
	if (i2c_map_dt_bitrate(cfg->bitrate) == 0) {
		return -EINVAL;
	}
	return i2c_xec_v3_port_configure(dev, i2c_cfg);
}

#define I2C_XEC_V3_PORT_INIT(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct i2c_xec_v3_port_cfg i2c_xec_v3_port_cfg_##n = {                        \
		.ctrl = DEVICE_DT_GET(DT_INST_PHANDLE(n, controller)),                             \
		.bitrate = DT_INST_PROP(n, clock_frequency),                                       \
		.port_idx = (uint8_t)(DT_INST_PROP(n, port) & 0x0FU),                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
	};                                                                                         \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_xec_v3_port_init, NULL, NULL, &i2c_xec_v3_port_cfg_##n,   \
				  POST_KERNEL, CONFIG_I2C_MCHP_XEC_V3_PORT_INIT_PRIORITY,          \
				  &i2c_xec_v3_port_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_XEC_V3_PORT_INIT)
