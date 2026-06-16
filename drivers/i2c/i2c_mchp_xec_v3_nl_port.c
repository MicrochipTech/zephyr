/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_i2c_v3_nl_port

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_xec_i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_mchp_xec_v3_nl_port, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"
#include "i2c_mchp_xec_v3.h"

#define I2C_XEC_V3_PORT_FLAG_IS_DFLT BIT(0)

struct i2c_xec_v3_nl_port_cfg {
	const struct device *ctrl;
	uint32_t bitrate;
	uint8_t port_idx;
	uint8_t flags;
	const struct pinctrl_dev_config *pcfg;
};

struct i2c_x3c_v3_nl_port_data {
	struct k_mutex_lock;
};

static int i2c_xec_v3_nl_port_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_xec_v3_nl_port_cfg *cfg = dev->config;
	int ret = 0;

	k_mutex_lock(&xdat->lock, K_FOREVER);
	/* TODO switch the port */

	ret = i2c_configure(cfg->ctrl, dev_config);

	k_mutex_unlock(&xdat->lock);

	return ret;
}

static int i2c_xec_v3_nl_port_get_config(const struct device *dev, uint32_t *dev_config)
{
	const struct i2c_xec_v3_nl_port_cfg *cfg = dev->config;

	return mchp_i2c_xec_v3_nl_get_config(cfg->ctrl, dev_config, NULL);
}

static int i2c_xec_v3_nl_port_transfer(const struct device *dev, struct i2c_msg *msgs,
				       uint8_t num_msgs, uint16_t addr)
{
	const struct i2c_xec_v3_nl_port_cfg *cfg = dev->config;
	int ret = 0;

	mchp_xec_i2c_v3_nl_ctrl_lock(cfg->ctrl);

	ret = mchp_xec_i2c_v3_nl_ctrl_port_switch(cfg->ctrl, cfg->bitrate, cfg->port_idx);

	if (ret == 0) {
		ret = mchp_i2c_xec_v3_nl_transfer(cfg->ctrl, msgs, num_msgs, addr);
	}

	mchp_xec_i2c_v3_nl_ctrl_unlock(cfg->ctrl);

	return ret;
}

static int i2c_xec_v3_nl_port_recover_bus(const struct device *dev)
{
	const struct i2c_xec_v3_nl_port_cfg *cfg = dev->config;
	int ret = 0;

	mchp_xec_i2c_v3_nl_ctrl_lock(cfg->ctrl);

	ret = mchp_xec_i2c_v3_nl_ctrl_port_switch(cfg->ctrl, cfg->bitrate, cfg->port_idx);
	if (ret == 0) {
		ret = mchp_xec_i2c_v3_nl_ctrl_recover_bus(cfg->ctrl, cfg->pcfg);
	}

	mchp_xec_i2c_v3_nl_ctrl_unlock(cfg->ctrl);

	return ret;
}

#ifdef CONFIG_I2C_TARGET
static int i2c_xec_v3_nl_port_target_register(const struct device *dev,
					      struct i2c_target_config *tcfg)
{
	const struct i2c_xec_v3_nl_port_cfg *cfg = dev->config;
	int ret = 0;

	if (tcfg == NULL) {
		return -EINVAL;
	}

	mchp_xec_i2c_v3_nl_ctrl_lock(cfg->ctrl);

	ret = mchp_xec_i2c_v3_nl_ctrl_port_switch(cfg->ctrl, cfg->bitrate, cfg->port_idx);
	if (ret == 0) {
		ret = mchp_xec_i2c_v3_nl_ctrl_target_register(cfg->ctrl, tcfg);
	}

	mchp_xec_i2c_v3_nl_ctrl_unlock(cfg->ctrl);

	return ret;
}

static int i2c_xec_v3_nl_port_target_unregister(const struct device *dev,
						struct i2c_target_config *tcfg)
{
	const struct i2c_xec_v3_nl_port_cfg *cfg = dev->config;
	int ret = 0;

	if (tcfg == NULL) {
		return -EINVAL;
	}

	mchp_xec_i2c_v3_nl_ctrl_lock(cfg->ctrl);

	ret = mchp_xec_i2c_v3_nl_ctrl_port_switch(cfg->ctrl, cfg->bitrate, cfg->port_idx);
	if (ret == 0) {
		ret = mchp_xec_i2c_v3_nl_ctrl_target_unregister(cfg->ctrl, tcfg);
	}

	mchp_xec_i2c_v3_nl_ctrl_unlock(cfg->ctrl);

	return ret;
}
#endif

int mchp_xec_i2c_nl_port_get(const struct device *i2c_dev, uint8_t *port)
{
	if ((i2c_dev == NULL) || (port == NULL)) {
		return -EINVAL;
	}

	if (xec_i2c_nl_is_busy(i2c_dev)) {
		return -EBUSY;
	}

	const struct i2c_xec_v3_nl_port_cfg *xcfg = i2c_dev->config;
	uint32_t i2c_cfg = 0;

	k_mutex_lock(&xdat->lock, K_FOREVER);

	i2c_cfg = sys_read32(xcfg->base + XEC_I2C_CFG_OFS);
	*port = (uint8_t)XEC_I2C_CFG_PORT_GET(i2c_cfg);

	k_mutex_unlock(&xdat->lock);

	return 0;
}

int mchp_xec_i2c_nl_port_set(const struct device *i2c_dev, uint8_t port)
{
	if ((i2c_dev == NULL) || (port >= MCHP_XEC_I2C_PORT_MAX)) {
		return -EINVAL;
	}

	if (xec_i2c_nl_is_busy(i2c_dev)) {
		return -EBUSY;
	}

	const struct i2c_xec_v3_nl_port_cfg *xcfg = i2c_dev->config;
	struct xec_i2c_nl_data *const xdat = i2c_dev->data;
	uint32_t v = XEC_I2C_CFG_PORT_SET((uint32_t)port);

	k_mutex_lock(&xdat->lock, K_FOREVER);

	soc_mmcr_mask_set(xcfg->base + XEC_I2C_CFG_OFS, v, XEC_I2C_CFG_PORT_MSK);

	k_mutex_unlock(&xdat->lock);

	return 0;
}

static DEVICE_API(i2c, i2c_xec_v3_nl_port_api) = {
	.configure = i2c_xec_v3_nl_port_configure,
	.get_config = i2c_xec_v3_nl_port_get_config,
	.transfer = i2c_xec_v3_nl_port_transfer,
	.recover_bus = i2c_xec_v3_nl_port_recover_bus,
#ifdef CONFIG_I2C_TARGET
	.target_register = i2c_xec_v3_nl_port_target_register,
	.target_unregister = i2c_xec_v3_nl_port_target_unregister,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

static int i2c_xec_v3_nl_port_init(const struct device *dev)
{
	const struct i2c_xec_v3_nl_port_cfg *cfg = dev->config;
	struct i2c_x3c_v3_nl_port_data *xdat = dev->data;
	uint32_t i2c_cfg = 0;
	int ret = 0;

	k_mutex_init(&xdat->lock);

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

	if ((cfg->flags & I2C_XEC_V3_PORT_FLAG_IS_DFLT) != 0) {
		ret = i2c_xec_v3_nl_port_configure(dev, i2c_cfg);
	}

	return ret;
}

#define I2C_XEC_V3_PORT_FLAGS(inst) COND_CODE_1(DT_INST_PROP(inst, default_port), (1), (0))

#define I2C_XEC_V3_NL_PORT_INIT(n)                                                                 \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct i2c_x3c_v3_nl_port_data i2c_xec_v3_nl_port_data_##n;                         \
	static const struct i2c_xec_v3_nl_port_cfg i2c_xec_v3_nl_port_cfg_##n = {                  \
		.ctrl = DEVICE_DT_GET(DT_INST_PHANDLE(n, controller)),                             \
		.bitrate = DT_INST_PROP(n, clock_frequency),                                       \
		.port_idx = (uint8_t)(DT_INST_PROP(n, port) & 0x0FU),                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.flags = I2C_XEC_V3_PORT_FLAGS(n),                                                 \
	};                                                                                         \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_xec_v3_nl_port_init, NULL,                                \
				  &i2c_xec_v3_nl_port_data_##n,                                    \
				  &i2c_xec_v3_nl_port_cfg_##n,                                     \
				  POST_KERNEL, CONFIG_I2C_MCHP_XEC_V3_PORT_INIT_PRIORITY,          \
				  &i2c_xec_v3_nl_port_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_XEC_V3_NL_PORT_INIT)
