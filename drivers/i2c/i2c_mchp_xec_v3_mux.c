/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip MEC5 per-port I2C MUX driver.
 *
 * Layered on top of a microchip,mec5-i2c-nl controller. Each child node in
 * the devicetree represents one SCL/SDA pin pair (port, CONFIG[3:0]) with its
 * own pinctrl state and clock-frequency. The MUX serializes access behind a
 * mutex; on every transfer it checks whether the underlying controller needs
 * to be reprogrammed (port or frequency change) and, if so, stops+resets the
 * controller, reapplies pinctrl, and reprograms port/timing before delegating
 * the transfer.
 */

#define DT_DRV_COMPAT microchip_xec_v3_i2c_mux

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <errno.h>

#include "i2c_mchp_mec5_nl.h"

LOG_MODULE_REGISTER(i2c_mchp_xec_v3_mux, CONFIG_I2C_LOG_LEVEL);

/* Root: one MUX node, wraps a single controller. */
struct mec5_mux_root_config {
	const struct device *controller;
};

struct mec5_mux_root_data {
	struct k_mutex bus_lock;
	uint8_t cur_port;
	uint32_t cur_freq_hz;
	bool programmed;
};

/* Child: one pin-pair port. */
struct mec5_mux_port_config {
	const struct device *root;
	const struct pinctrl_dev_config *pcfg;
	uint32_t freq_hz;
	uint8_t port;
};

static inline const struct mec5_mux_root_config *
port_root_cfg(const struct device *port_dev)
{
	const struct mec5_mux_port_config *p = port_dev->config;

	return p->root->config;
}

static inline struct mec5_mux_root_data *
port_root_data(const struct device *port_dev)
{
	const struct mec5_mux_port_config *p = port_dev->config;

	return p->root->data;
}

/* Reprogram controller when port or frequency differ from the cached state.
 * Caller must hold root->bus_lock.
 */
static int mec5_mux_select(const struct device *port_dev)
{
	const struct mec5_mux_port_config *p = port_dev->config;
	const struct mec5_mux_root_config *root_cfg = port_root_cfg(port_dev);
	struct mec5_mux_root_data *root = port_root_data(port_dev);
	int ret;

	if (root->programmed &&
	    root->cur_port == p->port &&
	    root->cur_freq_hz == p->freq_hz) {
		return 0;
	}

	ret = mec5_i2c_nl_stop_and_reset(root_cfg->controller);
	if (ret) {
		return ret;
	}

	ret = mec5_i2c_nl_apply_pinctrl(root_cfg->controller, p->pcfg);
	if (ret) {
		LOG_ERR("%s: pinctrl apply failed (%d)", port_dev->name, ret);
		return ret;
	}

	ret = mec5_i2c_nl_apply_port_freq(root_cfg->controller, p->port, p->freq_hz);
	if (ret) {
		LOG_ERR("%s: port/freq apply failed (%d)", port_dev->name, ret);
		return ret;
	}

	root->cur_port = p->port;
	root->cur_freq_hz = p->freq_hz;
	root->programmed = true;
	return 0;
}

static int mec5_mux_configure(const struct device *port_dev, uint32_t dev_config)
{
	const struct mec5_mux_port_config *p = port_dev->config;
	const struct mec5_mux_root_config *root_cfg = port_root_cfg(port_dev);
	struct mec5_mux_root_data *root = port_root_data(port_dev);
	int ret;

	k_mutex_lock(&root->bus_lock, K_FOREVER);

	ret = mec5_mux_select(port_dev);
	if (ret == 0) {
		ret = i2c_configure(root_cfg->controller, dev_config);
		if (ret == 0) {
			/* configure() may have changed speed; resync cache. */
			root->cur_freq_hz = p->freq_hz;
		}
	}

	k_mutex_unlock(&root->bus_lock);
	return ret;
}

static int mec5_mux_get_config(const struct device *port_dev, uint32_t *dev_config)
{
	const struct mec5_mux_root_config *root_cfg = port_root_cfg(port_dev);
	struct mec5_mux_root_data *root = port_root_data(port_dev);
	int ret;

	k_mutex_lock(&root->bus_lock, K_FOREVER);
	ret = mec5_mux_select(port_dev);
	if (ret == 0) {
		ret = i2c_get_config(root_cfg->controller, dev_config);
	}
	k_mutex_unlock(&root->bus_lock);
	return ret;
}

static int mec5_mux_transfer(const struct device *port_dev,
			     struct i2c_msg *msgs,
			     uint8_t num_msgs,
			     uint16_t addr)
{
	const struct mec5_mux_root_config *root_cfg = port_root_cfg(port_dev);
	struct mec5_mux_root_data *root = port_root_data(port_dev);
	int ret;

	k_mutex_lock(&root->bus_lock, K_FOREVER);

	ret = mec5_mux_select(port_dev);
	if (ret == 0) {
		ret = i2c_transfer(root_cfg->controller, msgs, num_msgs, addr);
	}

	k_mutex_unlock(&root->bus_lock);
	return ret;
}

#ifdef CONFIG_I2C_CALLBACK
static int mec5_mux_transfer_cb(const struct device *port_dev,
				struct i2c_msg *msgs,
				uint8_t num_msgs,
				uint16_t addr,
				i2c_callback_t cb,
				void *userdata)
{
	const struct mec5_mux_root_config *root_cfg = port_root_cfg(port_dev);
	struct mec5_mux_root_data *root = port_root_data(port_dev);
	int ret;

	/* The async path cannot hold the mutex across the callback. Program
	 * the port synchronously, then hand off; concurrent transfers on
	 * other ports must wait for the i2c_transfer_cb completion before
	 * reprogramming.
	 */
	k_mutex_lock(&root->bus_lock, K_FOREVER);
	ret = mec5_mux_select(port_dev);
	k_mutex_unlock(&root->bus_lock);

	if (ret) {
		return ret;
	}

	return i2c_transfer_cb(root_cfg->controller, msgs, num_msgs, addr, cb, userdata);
}
#endif /* CONFIG_I2C_CALLBACK */

static int mec5_mux_recover_bus(const struct device *port_dev)
{
	const struct mec5_mux_root_config *root_cfg = port_root_cfg(port_dev);
	struct mec5_mux_root_data *root = port_root_data(port_dev);
	int ret;

	k_mutex_lock(&root->bus_lock, K_FOREVER);
	ret = mec5_mux_select(port_dev);
	if (ret == 0) {
		ret = i2c_recover_bus(root_cfg->controller);
	}
	k_mutex_unlock(&root->bus_lock);
	return ret;
}

#ifdef CONFIG_I2C_TARGET
static int mec5_mux_target_register(const struct device *port_dev,
				    struct i2c_target_config *tcfg)
{
	const struct mec5_mux_root_config *root_cfg = port_root_cfg(port_dev);
	struct mec5_mux_root_data *root = port_root_data(port_dev);
	int ret;

	k_mutex_lock(&root->bus_lock, K_FOREVER);
	ret = mec5_mux_select(port_dev);
	if (ret == 0) {
		ret = i2c_target_register(root_cfg->controller, tcfg);
	}
	k_mutex_unlock(&root->bus_lock);
	return ret;
}

static int mec5_mux_target_unregister(const struct device *port_dev,
				      struct i2c_target_config *tcfg)
{
	const struct mec5_mux_root_config *root_cfg = port_root_cfg(port_dev);
	struct mec5_mux_root_data *root = port_root_data(port_dev);
	int ret;

	k_mutex_lock(&root->bus_lock, K_FOREVER);
	ret = i2c_target_unregister(root_cfg->controller, tcfg);
	k_mutex_unlock(&root->bus_lock);
	return ret;
}
#endif /* CONFIG_I2C_TARGET */

static DEVICE_API(i2c, mec5_mux_api) = {
	.configure   = mec5_mux_configure,
	.get_config  = mec5_mux_get_config,
	.transfer    = mec5_mux_transfer,
	.recover_bus = mec5_mux_recover_bus,
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = mec5_mux_transfer_cb,
#endif
#ifdef CONFIG_I2C_TARGET
	.target_register   = mec5_mux_target_register,
	.target_unregister = mec5_mux_target_unregister,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

static int mec5_mux_root_init(const struct device *dev)
{
	const struct mec5_mux_root_config *cfg = dev->config;
	struct mec5_mux_root_data *data = dev->data;

	if (!device_is_ready(cfg->controller)) {
		LOG_ERR("%s: controller %s not ready", dev->name,
			cfg->controller->name);
		return -ENODEV;
	}

	data->programmed = false;
	data->cur_port = UINT8_MAX;
	data->cur_freq_hz = 0;
	return 0;
}

static int mec5_mux_port_init(const struct device *port_dev)
{
	const struct mec5_mux_port_config *p = port_dev->config;

	if (!device_is_ready(p->root)) {
		LOG_ERR("%s: MUX root not ready", port_dev->name);
		return -ENODEV;
	}
	return 0;
}

/* ---- Devicetree instantiation -------------------------------------- */

#define MEC5_MUX_PORT_DEFINE(child)                                           \
	PINCTRL_DT_DEFINE(child);                                             \
	static const struct mec5_mux_port_config                              \
		mec5_mux_port_cfg_##child = {                                 \
			.root    = DEVICE_DT_GET(DT_PARENT(child)),           \
			.pcfg    = PINCTRL_DT_DEV_CONFIG_GET(child),          \
			.freq_hz = DT_PROP(child, clock_frequency),           \
			.port    = (uint8_t)DT_REG_ADDR(child),               \
		};                                                            \
	DEVICE_DT_DEFINE(child, mec5_mux_port_init, NULL,                     \
			 NULL, &mec5_mux_port_cfg_##child,                    \
			 POST_KERNEL,                                         \
			 CONFIG_I2C_MCHP_MEC5_MUX_PORT_INIT_PRIORITY,         \
			 &mec5_mux_api);

#define MEC5_MUX_ROOT_DEFINE(n)                                               \
	static const struct mec5_mux_root_config mec5_mux_root_cfg_##n = {    \
		.controller = DEVICE_DT_GET(DT_INST_PHANDLE(n, controller)),  \
	};                                                                    \
	static struct mec5_mux_root_data mec5_mux_root_data_##n = {           \
		.bus_lock = Z_MUTEX_INITIALIZER(mec5_mux_root_data_##n.bus_lock), \
	};                                                                    \
	DEVICE_DT_INST_DEFINE(n, mec5_mux_root_init, NULL,                    \
			      &mec5_mux_root_data_##n,                        \
			      &mec5_mux_root_cfg_##n,                         \
			      POST_KERNEL,                                    \
			      CONFIG_I2C_MCHP_MEC5_MUX_ROOT_INIT_PRIORITY,    \
			      NULL);                                          \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(n, MEC5_MUX_PORT_DEFINE)

BUILD_ASSERT(CONFIG_I2C_MCHP_MEC5_MUX_PORT_INIT_PRIORITY >                    \
		     CONFIG_I2C_MCHP_MEC5_MUX_ROOT_INIT_PRIORITY,             \
	     "MUX port children must init after the MUX root");

DT_INST_FOREACH_STATUS_OKAY(MEC5_MUX_ROOT_DEFINE)
