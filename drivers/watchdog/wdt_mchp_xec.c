/* wdt_xec.c - Microchip XEC watchdog driver */

#define DT_DRV_COMPAT microchip_xec_watchdog

/*
 * Copyright (c) 2019 Intel Corporation.
 * Copyright (c) 2021 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_WDT_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(wdt_mchp_xec);

#include <drivers/watchdog.h>
#include <soc.h>
#include <errno.h>

struct wdt_xec_config {
	uintptr_t base;
	uint8_t girq;
	uint8_t girq_pos;
};

struct wdt_xec_data {
	wdt_callback_t cb;
	bool timeout_installed;
};

#define WDT_XEC_CONFIG(_dev)				\
	(((const struct wdt_xec_config * const)		\
	  _dev->config))

#define WDT_XEC_REG_BASE(_dev)				\
	((struct wdt_regs *)				\
	 ((const struct wdt_xec_config * const)		\
	  _dev->config)->base)

#define WDT_XEC_DATA(_dev)				\
	((struct wdt_xec_data *const)(_dev)->data)

static int wdt_xec_setup(const struct device *dev, uint8_t options)
{
	struct wdt_xec_data *data = WDT_XEC_DATA(dev);
	struct wdt_regs *regs = WDT_XEC_REG_BASE(dev);

	if (regs->CTRL & MCHP_WDT_CTRL_EN) {
		return -EBUSY;
	}

	if (!data->timeout_installed) {
		LOG_ERR("No valid WDT timeout installed");
		return -EINVAL;
	}

	if (options & WDT_OPT_PAUSE_IN_SLEEP) {
		LOG_WRN("WDT_OPT_PAUSE_IN_SLEEP is not supported");
		return -ENOTSUP;
	}

	if (options & WDT_OPT_PAUSE_HALTED_BY_DBG) {
		regs->CTRL |= MCHP_WDT_CTRL_JTAG_STALL_EN;
	} else {
		regs->CTRL &= ~MCHP_WDT_CTRL_JTAG_STALL_EN;
	}

	regs->CTRL |= MCHP_WDT_CTRL_EN;

	LOG_DBG("WDT Setup and enabled");

	return 0;
}

static int wdt_xec_disable(const struct device *dev)
{
	struct wdt_xec_data *data = WDT_XEC_DATA(dev);
	struct wdt_regs *regs = WDT_XEC_REG_BASE(dev);

	if (!(regs->CTRL & MCHP_WDT_CTRL_EN)) {
		return -EALREADY;
	}

	regs->CTRL &= ~MCHP_WDT_CTRL_EN;
	data->timeout_installed = false;

	LOG_DBG("WDT Disabled");

	return 0;
}

static int wdt_xec_install_timeout(const struct device *dev,
				   const struct wdt_timeout_cfg *config)
{
	struct wdt_xec_data *data = WDT_XEC_DATA(dev);
	struct wdt_regs *regs = WDT_XEC_REG_BASE(dev);

	if (regs->CTRL & MCHP_WDT_CTRL_EN) {
		return -EBUSY;
	}

	if (config->window.min > 0U) {
		data->timeout_installed = false;
		return -EINVAL;
	}

	regs->LOAD = 0;

	data->cb = config->callback;
	if (data->cb) {
		regs->CTRL |= MCHP_WDT_CTRL_MODE_IRQ;
		regs->IEN |= MCHP_WDT_IEN_EVENT_IRQ_EN;

		LOG_DBG("WDT callback enabled");
	} else {
		/* Setting WDT_FLAG_RESET_SOC or not will have no effect:
		 * even after the cb, if anything is done, SoC will reset
		 */
		regs->CTRL &= ~MCHP_WDT_CTRL_MODE_IRQ;
		regs->IEN &= ~MCHP_WDT_IEN_EVENT_IRQ_EN;

		LOG_DBG("WDT Reset enabled");
	}

	/* Since it almost takes 1ms to decrement the load register
	 * (See datasheet 18.6.1.4: 33/32.768 KHz = 1.007ms)
	 * Let's use the given window directly.
	 */
	regs->LOAD = config->window.max;

	data->timeout_installed = true;

	return 0;
}

static int wdt_xec_feed(const struct device *dev, int channel_id)
{
	struct wdt_regs *regs = WDT_XEC_REG_BASE(dev);

	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);

	if (!(regs->CTRL & MCHP_WDT_CTRL_EN)) {
		return -EINVAL;
	}

	LOG_DBG("WDT Kicking");

	regs->KICK = 1;

	return 0;
}

static void wdt_xec_isr(const struct device *dev)
{
	const struct wdt_xec_config *cfg = WDT_XEC_CONFIG(dev);
	struct wdt_xec_data *data = WDT_XEC_DATA(dev);
	struct wdt_regs *regs = WDT_XEC_REG_BASE(dev);

	LOG_DBG("WDT ISR");

	if (data->cb) {
		data->cb(dev, 0);
	}

	mchp_xec_ecia_girq_src_clr(cfg->girq, cfg->girq_pos);

	regs->IEN &= ~MCHP_WDT_IEN_EVENT_IRQ_EN;
}

static const struct wdt_driver_api wdt_xec_api = {
	.setup = wdt_xec_setup,
	.disable = wdt_xec_disable,
	.install_timeout = wdt_xec_install_timeout,
	.feed = wdt_xec_feed,
};

static int wdt_xec_init(const struct device *dev)
{
	const struct wdt_xec_config *cfg = WDT_XEC_CONFIG(dev);

	if (IS_ENABLED(CONFIG_WDT_DISABLE_AT_BOOT)) {
		wdt_xec_disable(dev);
	}

	mchp_xec_ecia_girq_src_en(cfg->girq, cfg->girq_pos);

	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    wdt_xec_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	return 0;
}

static const struct wdt_xec_config wdt_xec_0_config = {
	.base = DT_INST_REG_ADDR(0),
	.girq = DT_INST_PROP_BY_IDX(0, girqs, 0),
	.girq_pos = DT_INST_PROP_BY_IDX(0, girqs, 1),
};

static struct wdt_xec_data wdt_xec_dev_data;

DEVICE_DT_INST_DEFINE(0, wdt_xec_init, NULL,
		    &wdt_xec_dev_data, &wdt_xec_0_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &wdt_xec_api);
