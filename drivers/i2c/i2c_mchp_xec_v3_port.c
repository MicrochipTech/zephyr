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

#define I2C_XEC_V3_PORT_FLAG_IS_DFLT BIT(0)

struct i2c_xec_v3_port_cfg {
	const struct device *ctrl;
	uint32_t bitrate;
	uint8_t port_idx;
	uint8_t flags;
	const struct pinctrl_dev_config *pcfg;
};

/*
 * The controller is shared between ports and is reprogrammed on every port
 * switch, so each port must carry the bus rate it wants re-applied. Keeping
 * that rate in runtime data rather than reading cfg->bitrate directly is what
 * makes i2c_configure() stick: previously every entry point below re-asserted
 * the static devicetree clock-frequency, silently undoing a caller's speed
 * change on the very next transfer.
 */
struct i2c_xec_v3_port_data {
	uint32_t bitrate;
};

/**
 * @brief Map a Zephyr dev_config speed code to its bus frequency in Hz.
 *
 * @param dev_config Zephyr I2C dev_config word; only the speed field is read.
 * @param bitrate    Out: bus frequency in Hz. Untouched unless 0 is returned.
 *
 * @retval 0 Speed recognised and @p bitrate written.
 * @retval -ENOTSUP The speed code is not one this controller supports.
 */
static int i2c_xec_v3_port_speed_to_bitrate(uint32_t dev_config, uint32_t *bitrate)
{
	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		*bitrate = I2C_BITRATE_STANDARD;
		return 0;
	case I2C_SPEED_FAST:
		*bitrate = I2C_BITRATE_FAST;
		return 0;
	case I2C_SPEED_FAST_PLUS:
		*bitrate = I2C_BITRATE_FAST_PLUS;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int i2c_xec_v3_port_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	struct i2c_xec_v3_port_data *data = dev->data;
	uint32_t bitrate = 0;
	int ret = 0;

	/* Resolve the rate first: an unsupported speed must leave the port's
	 * active rate untouched rather than half-applied.
	 */
	ret = i2c_xec_v3_port_speed_to_bitrate(dev_config, &bitrate);
	if (ret != 0) {
		return ret;
	}

	mchp_xec_i2c_v3_ctrl_lock(cfg->ctrl);
	ret = mchp_i2c_xec_v3_config(cfg->ctrl, dev_config, cfg->port_idx);
	if (ret == 0) {
		/* Latch it so the next port switch re-applies this rate. */
		data->bitrate = bitrate;
	}
	mchp_xec_i2c_v3_ctrl_unlock(cfg->ctrl);

	return ret;
}

static int i2c_xec_v3_port_get_config(const struct device *dev, uint32_t *dev_config)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	struct i2c_xec_v3_port_data *data = dev->data;
	int ret = 0;

	/* Report this port's own rate, not the controller's live BUS_CLK, and do
	 * it under the controller lock: the controller-side accessors are all
	 * documented as lock-held, and an unlocked read races a concurrent
	 * config_hw() mid-reconfiguration.
	 */
	mchp_xec_i2c_v3_ctrl_lock(cfg->ctrl);
	ret = mchp_i2c_xec_v3_get_port_config(cfg->ctrl, data->bitrate, dev_config);
	mchp_xec_i2c_v3_ctrl_unlock(cfg->ctrl);

	return ret;
}

static int i2c_xec_v3_port_transfer(const struct device *dev, struct i2c_msg *msgs,
				    uint8_t num_msgs, uint16_t addr)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	struct i2c_xec_v3_port_data *data = dev->data;
	int ret = 0;

	mchp_xec_i2c_v3_ctrl_lock(cfg->ctrl);

	ret = mchp_xec_i2c_v3_ctrl_port_switch(cfg->ctrl, data->bitrate, cfg->port_idx);

	if (ret == 0) {
		ret = mchp_i2c_xec_v3_transfer(cfg->ctrl, msgs, num_msgs, addr);
	}

	mchp_xec_i2c_v3_ctrl_unlock(cfg->ctrl);

	return ret;
}

#ifdef CONFIG_I2C_CALLBACK
/* The controller holds the bus lock across the async transfer and releases
 * it from its completion ISR before invoking cb. If the transfer API fails
 * synchronously we release the lock here.
 */
static int i2c_xec_v3_port_transfer_cb(const struct device *dev, struct i2c_msg *msgs,
				       uint8_t num_msgs, uint16_t addr, i2c_callback_t cb,
				       void *userdata)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	struct i2c_xec_v3_port_data *data = dev->data;
	int ret = 0;

	if (num_msgs == 0U) {
		if (cb != NULL) {
			cb(dev, 0, userdata);
		}
		return 0;
	}

	if (msgs == NULL) {
		if (cb != NULL) {
			cb(dev, -EINVAL, userdata);
		}
		return -EINVAL;
	}

	mchp_xec_i2c_v3_ctrl_lock(cfg->ctrl);

	ret = mchp_xec_i2c_v3_ctrl_port_switch(cfg->ctrl, data->bitrate, cfg->port_idx);
	if (ret == 0) {
		ret = mchp_i2c_v3_transfer_cb(cfg->ctrl, msgs, num_msgs, addr, cb, userdata);
	}

	if (ret != 0) {
		mchp_xec_i2c_v3_ctrl_unlock(cfg->ctrl);
	}

	return ret;
}
#endif

static int i2c_xec_v3_port_recover_bus(const struct device *dev)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	struct i2c_xec_v3_port_data *data = dev->data;
	int ret = 0;

	mchp_xec_i2c_v3_ctrl_lock(cfg->ctrl);

	ret = mchp_xec_i2c_v3_ctrl_port_switch(cfg->ctrl, data->bitrate, cfg->port_idx);
	if (ret == 0) {
		ret = mchp_xec_i2c_v3_ctrl_recover_bus(cfg->ctrl, cfg->pcfg);
	}

	mchp_xec_i2c_v3_ctrl_unlock(cfg->ctrl);

	return ret;
}

#ifdef CONFIG_I2C_TARGET
static int i2c_xec_v3_port_target_register(const struct device *dev, struct i2c_target_config *tcfg)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	struct i2c_xec_v3_port_data *data = dev->data;
	int ret = 0;

	if (tcfg == NULL) {
		return -EINVAL;
	}

	mchp_xec_i2c_v3_ctrl_lock(cfg->ctrl);

	ret = mchp_xec_i2c_v3_ctrl_port_switch(cfg->ctrl, data->bitrate, cfg->port_idx);
	if (ret == 0) {
		ret = mchp_xec_i2c_v3_ctrl_target_register(cfg->ctrl, tcfg);
	}

	mchp_xec_i2c_v3_ctrl_unlock(cfg->ctrl);

	return ret;
}

static int i2c_xec_v3_port_target_unregister(const struct device *dev,
					     struct i2c_target_config *tcfg)
{
	const struct i2c_xec_v3_port_cfg *cfg = dev->config;
	struct i2c_xec_v3_port_data *data = dev->data;
	int ret = 0;

	if (tcfg == NULL) {
		return -EINVAL;
	}

	mchp_xec_i2c_v3_ctrl_lock(cfg->ctrl);

	ret = mchp_xec_i2c_v3_ctrl_port_switch(cfg->ctrl, data->bitrate, cfg->port_idx);
	if (ret == 0) {
		ret = mchp_xec_i2c_v3_ctrl_target_unregister(cfg->ctrl, tcfg);
	}

	mchp_xec_i2c_v3_ctrl_unlock(cfg->ctrl);

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
	struct i2c_xec_v3_port_data *data = dev->data;
	uint32_t i2c_cfg = 0;
	int ret = 0;

	/* Seed the runtime rate from devicetree. Non-default ports skip the
	 * configure call below, so this must not depend on that running.
	 */
	data->bitrate = cfg->bitrate;

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
		ret = i2c_xec_v3_port_configure(dev, i2c_cfg);
	}

	return ret;
}

#define I2C_XEC_V3_PORT_FLAGS(inst) COND_CODE_1(DT_INST_PROP(inst, default_port), (1), (0))

#define I2C_XEC_V3_PORT_INIT(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct i2c_xec_v3_port_cfg i2c_xec_v3_port_cfg_##n = {                        \
		.ctrl = DEVICE_DT_GET(DT_INST_PHANDLE(n, controller)),                             \
		.bitrate = DT_INST_PROP(n, clock_frequency),                                       \
		.port_idx = (uint8_t)(DT_INST_PROP(n, port) & 0x0FU),                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.flags = I2C_XEC_V3_PORT_FLAGS(n),                                                 \
	};                                                                                         \
	static struct i2c_xec_v3_port_data i2c_xec_v3_port_data_##n;                               \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_xec_v3_port_init, NULL, &i2c_xec_v3_port_data_##n,        \
				  &i2c_xec_v3_port_cfg_##n, POST_KERNEL,                           \
				  CONFIG_I2C_MCHP_XEC_V3_PORT_INIT_PRIORITY,                       \
				  &i2c_xec_v3_port_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_XEC_V3_PORT_INIT)
