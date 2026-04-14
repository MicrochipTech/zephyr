/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr I2C mux driver for Microchip XEC internal PORT_SEL multiplexer.
 *
 * The XEC SMB controller can route SCL/SDA to one of 16 physical bus ports
 * via the PORT_SEL field in the Configuration Register. This driver exposes
 * each port as a standard Zephyr I2C bus device, following the same pattern
 * as the TCA954x external I2C mux driver.
 *
 * Each child node in the device tree represents one port. Transfers on a
 * child device transparently select the corresponding port on the parent
 * XEC I2C controller before forwarding the transaction.
 *
 * Port switching requires a full controller reset (which clears port and
 * timing registers). Both are reprogrammed before re-enabling. An
 * optimization skips the reset if the requested port and speed match the
 * currently active configuration.
 */

#define DT_DRV_COMPAT microchip_xec_i2c_mux

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "i2c_mchp_xec_regs.h"

LOG_MODULE_REGISTER(i2c_xec_mux, CONFIG_I2C_LOG_LEVEL);

/* Provided by i2c_mchp_xec_v2.c */
extern int i2c_xec_v2_switch_port(const struct device *dev, uint8_t port_sel, uint32_t clock_freq);
extern int i2c_xec_v2_get_port(const struct device *dev, uint8_t *port_sel);
#if defined(CONFIG_SOC_SERIES_MEC172X) || defined(CONFIG_SOC_SERIES_MEC15XX)
extern void i2c_xec_v2_set_active_gpios(const struct device *dev, const struct gpio_dt_spec *scl,
					const struct gpio_dt_spec *sda);
#endif

/* ----------------------------------------------------------------
 * Data structures
 * ---------------------------------------------------------------- */

/* Root mux device: one per XEC I2C controller instance that uses muxing */
struct xec_mux_root_config {
	const struct device *parent_dev; /* parent XEC I2C controller */
};

struct xec_mux_root_data {
	struct k_mutex lock;
	uint8_t current_port;   /* currently selected port (0xFF = none) */
	uint32_t current_clock; /* currently configured clock frequency */
};

/* Channel device: one per port_sel value */
struct xec_mux_channel_config {
	const struct device *root;
	uint8_t port_sel;         /* PORT_SEL value from DT reg property */
	uint32_t clock_frequency; /* per-channel clock freq, 0 = use parent default */
#if defined(CONFIG_SOC_SERIES_MEC172X) || defined(CONFIG_SOC_SERIES_MEC15XX)
	struct gpio_dt_spec sda_gpio;
	struct gpio_dt_spec scl_gpio;
#endif
};

/* ----------------------------------------------------------------
 * Helper access functions
 * ---------------------------------------------------------------- */

static inline struct xec_mux_root_data *get_root_data_from_channel(const struct device *dev)
{
	const struct xec_mux_channel_config *ch_cfg = dev->config;

	return ch_cfg->root->data;
}

static inline const struct xec_mux_root_config *
get_root_config_from_channel(const struct device *dev)
{
	const struct xec_mux_channel_config *ch_cfg = dev->config;

	return ch_cfg->root->config;
}

/* ----------------------------------------------------------------
 * Port selection
 * ---------------------------------------------------------------- */

/*
 * Select the given port on the parent controller.
 * Skips the expensive reset+reconfigure if port and speed haven't changed.
 */
static int xec_mux_set_channel(const struct device *root_dev, uint8_t port_sel, uint32_t clock_freq)
{
	struct xec_mux_root_data *data = root_dev->data;
	const struct xec_mux_root_config *cfg = root_dev->config;

	/* Optimization: skip reset if port and clock haven't changed */
	if (data->current_port == port_sel &&
	    (clock_freq == 0 || data->current_clock == clock_freq)) {
		return 0;
	}

	int ret = i2c_xec_v2_switch_port(cfg->parent_dev, port_sel, clock_freq);

	if (ret == 0) {
		data->current_port = port_sel;
		if (clock_freq != 0) {
			data->current_clock = clock_freq;
		}
	} else {
		LOG_ERR("Failed to switch to port %u: %d", port_sel, ret);
	}

	return ret;
}

/* ----------------------------------------------------------------
 * Zephyr I2C driver API callbacks
 * ---------------------------------------------------------------- */

static int xec_mux_configure(const struct device *dev, uint32_t dev_config)
{
	const struct xec_mux_root_config *root_cfg = get_root_config_from_channel(dev);

	return i2c_configure(root_cfg->parent_dev, dev_config);
}

static int xec_mux_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			    uint16_t addr)
{
	struct xec_mux_root_data *root_data = get_root_data_from_channel(dev);
	const struct xec_mux_root_config *root_cfg = get_root_config_from_channel(dev);
	const struct xec_mux_channel_config *ch_cfg = dev->config;
	int ret;

	ret = k_mutex_lock(&root_data->lock, K_MSEC(5000));
	if (ret != 0) {
		return ret;
	}

	/* Select this channel's port (and optionally reconfigure speed) */
	ret = xec_mux_set_channel(ch_cfg->root, ch_cfg->port_sel, ch_cfg->clock_frequency);
	if (ret != 0) {
		goto end_trans;
	}

#if defined(CONFIG_SOC_SERIES_MEC172X) || defined(CONFIG_SOC_SERIES_MEC15XX)
	/* Update parent's active GPIO references for this port's SCL/SDA pins */
	i2c_xec_v2_set_active_gpios(root_cfg->parent_dev, &ch_cfg->scl_gpio, &ch_cfg->sda_gpio);
#endif

	/* Forward the transfer to the parent XEC I2C controller */
	ret = i2c_transfer(root_cfg->parent_dev, msgs, num_msgs, addr);

end_trans:
	k_mutex_unlock(&root_data->lock);
	return ret;
}

/* ----------------------------------------------------------------
 * Initialization
 * ---------------------------------------------------------------- */

static int xec_mux_root_init(const struct device *dev)
{
	struct xec_mux_root_data *data = dev->data;
	const struct xec_mux_root_config *cfg = dev->config;

	if (!device_is_ready(cfg->parent_dev)) {
		LOG_ERR("Parent I2C bus %s not ready", cfg->parent_dev->name);
		return -ENODEV;
	}

	k_mutex_init(&data->lock);
	data->current_port = 0xFF; /* no port selected yet */
	data->current_clock = 0;

	return 0;
}

static int xec_mux_channel_init(const struct device *dev)
{
	const struct xec_mux_channel_config *ch_cfg = dev->config;

	if (!device_is_ready(ch_cfg->root)) {
		LOG_ERR("Mux root %s not ready", ch_cfg->root->name);
		return -ENODEV;
	}

	if (ch_cfg->port_sel >= XEC_I2C_CFG_MAX_PORT) {
		LOG_ERR("Invalid port_sel %u for %s (max %u)", ch_cfg->port_sel, dev->name,
			XEC_I2C_CFG_MAX_PORT - 1);
		return -EINVAL;
	}

#if defined(CONFIG_SOC_SERIES_MEC172X) || defined(CONFIG_SOC_SERIES_MEC15XX)
	if (ch_cfg->sda_gpio.port == NULL || ch_cfg->scl_gpio.port == NULL) {
		LOG_ERR("MEC172x requires sda-gpios and scl-gpios for mux port %u",
			ch_cfg->port_sel);
		return -EINVAL;
	}

	/* If this channel's port matches the parent controller's current default
	 * port, set the active GPIOs now so pin state reading works from init
	 * time. Without this the controller would have null GPIO references
	 * between init and the first mux transfer.
	 */
	{
		const struct xec_mux_root_config *root_cfg = ch_cfg->root->config;
		uint8_t parent_port;

		if (i2c_xec_v2_get_port(root_cfg->parent_dev, &parent_port) == 0 &&
		    parent_port == ch_cfg->port_sel) {
			i2c_xec_v2_set_active_gpios(root_cfg->parent_dev, &ch_cfg->scl_gpio,
						    &ch_cfg->sda_gpio);
		}
	}
#endif

	return 0;
}

/* ----------------------------------------------------------------
 * Driver API struct
 * ---------------------------------------------------------------- */

static DEVICE_API(i2c, xec_mux_api) = {
	.configure = xec_mux_configure,
	.transfer = xec_mux_transfer,
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

/* ----------------------------------------------------------------
 * Device tree instantiation macros
 * ---------------------------------------------------------------- */

#if defined(CONFIG_SOC_SERIES_MEC172X) || defined(CONFIG_SOC_SERIES_MEC15XX)
#define XEC_MUX_CH_GPIO_INIT(node_id)                                                              \
	.sda_gpio = GPIO_DT_SPEC_GET_OR(node_id, sda_gpios, {0}),                                  \
	.scl_gpio = GPIO_DT_SPEC_GET_OR(node_id, scl_gpios, {0}),
#else
#define XEC_MUX_CH_GPIO_INIT(node_id)
#endif

#define XEC_MUX_CHILD_DEFINE(node_id)                                                              \
	static const struct xec_mux_channel_config xec_mux_ch_config_##node_id = {                 \
		.root = DEVICE_DT_GET(DT_PARENT(node_id)),                                         \
		.port_sel = DT_REG_ADDR(node_id),                                                  \
		.clock_frequency = DT_PROP_OR(node_id, clock_frequency, 0),                        \
		XEC_MUX_CH_GPIO_INIT(node_id)};                                                    \
	DEVICE_DT_DEFINE(node_id, xec_mux_channel_init, NULL, NULL, &xec_mux_ch_config_##node_id,  \
			 POST_KERNEL, CONFIG_I2C_XEC_MUX_CHANNEL_INIT_PRIO, &xec_mux_api);

#define XEC_MUX_ROOT_DEFINE(inst)                                                                  \
	static const struct xec_mux_root_config xec_mux_root_config_##inst = {                     \
		.parent_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, i2c_parent)),                    \
	};                                                                                         \
	static struct xec_mux_root_data xec_mux_root_data_##inst;                                  \
	DEVICE_DT_INST_DEFINE(inst, xec_mux_root_init, NULL, &xec_mux_root_data_##inst,            \
			      &xec_mux_root_config_##inst, POST_KERNEL,                            \
			      CONFIG_I2C_XEC_MUX_ROOT_INIT_PRIO, NULL);                            \
	DT_INST_FOREACH_CHILD(inst, XEC_MUX_CHILD_DEFINE)

DT_INST_FOREACH_STATUS_OKAY(XEC_MUX_ROOT_DEFINE)
