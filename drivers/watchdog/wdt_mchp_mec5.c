/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_watchdog

#include <errno.h>
#include <soc.h>
#include <mec_wdt_api.h>

#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/pm/device.h>
#include <zephyr/irq.h>

#define LOG_LEVEL CONFIG_WDT_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_mchp_mec5);

/* Microchip MEC5 HAL based watchdog timer (WDT)
 * WDT clock source is 32 KHz domain allowing it to operate when the
 * PLL is off (deep sleep).
 * WDT rate is 33 32KHz cycles = 1.007 ms per count.
 * WDT Counter 16-bits which is loaded from the 16-bit Load register
 * when the WDT enable bit is set.
 * WDT count can be selectively stalled(paused) by any of the following three events:
 *   Debug event: JTAG_nRESET pin input is inactive (high).
 *   Hibernation timer 0 is enabled and counting.
 *   Week timer is enabled and counting.
 * WDT has tow modes of operation:
 *  Non-interrupt mode:
 *    When enabled and count reaches 0 the WDT reset status bit is set in the VBAT
 *    Powered register bank and the SoC is reset.
 *  Interrupt mode:
 *    When enabled in interrupt mode and count reaches 0 the WDT reloads its count
 *    register from the Load register, clears the WDT_RESET control register bit, and
 *    generates the WDT interrupt ot the EC. Once the WDT counts down a second time
 *    with WDT_RESET bit clear it will reset per non-interrupt mode. The ISR may
 *    choose to disable the WDT to prevent reset or reload it with a different
 *    count down value.
 */

struct wdt_mec5_dev_cfg {
	struct mec_wdt_regs *regs;
	void (*irq_config)(const struct device *dev);
};

#define WDT_MEC5_DEV_DATA_FLAG_EN BIT(0)

struct wdt_mec5_dev_data {
	const struct wdt_timeout_cfg *tmout_head;
	uint8_t options;
	uint8_t flags;
};

/* Configure watchdog timer settings for all timeouts.
 * Call setup after calling install_timeouts API.
 * Supported options:
 * WDT_OPT_PAUSE_IN_SLEEP - Pause if entering sleep. Implement in PM_DEVICE support.
 * WDT_OPT_PAUSE_HALTED_BY_DBG - If debugger active halt WDT count down.
 *   MEC5 WDT supports this feature.
 */
static int wdt_mec5_setup(const struct device *dev, uint8_t options)
{
	const struct wdt_mec5_dev_cfg *devcfg = dev->config;
	struct mec_wdt_regs *const regs = devcfg->regs;
	struct wdt_mec5_dev_data *data = dev->data;

	if (mec_hal_wdt_is_enabled(regs)) {
		return -EBUSY;
	}

	if (!data->tmout_head) {
		LOG_ERR("WDT timeout not installed");
		return -EINVAL;
	}

	if ((options & WDT_OPT_PAUSE_IN_SLEEP) && !IS_ENABLED(CONFIG_PM_DEVICE)) {
		LOG_WRN("WDT_OPT_PAUSE_IN_SLEEP is not supported");
		return -ENOTSUP;
	} else {
		data->options |= WDT_OPT_PAUSE_IN_SLEEP;
	}

	if (options & WDT_OPT_PAUSE_HALTED_BY_DBG) {
		mec_hal_wdt_debug_stall(regs, 1);
		data->options |= WDT_OPT_PAUSE_HALTED_BY_DBG;
	} else {
		mec_hal_wdt_debug_stall(regs, 0);
		data->options &= (uint8_t)~(WDT_OPT_PAUSE_HALTED_BY_DBG);
	}

	mec_hal_wdt_enable(regs);
	data->flags |= WDT_MEC5_DEV_DATA_FLAG_EN;

	LOG_DBG("WDT Setup and enabled");

	return 0;
}

static int wdt_mec5_disable(const struct device *dev)
{
	const struct wdt_mec5_dev_cfg *devcfg = dev->config;
	struct mec_wdt_regs *const regs = devcfg->regs;
	struct wdt_mec5_dev_data *data = dev->data;

	if (!mec_hal_wdt_is_enabled(regs)) {
		return -EALREADY;
	}

	mec_hal_wdt_disable(regs);
	data->flags &= (uint8_t)~(WDT_MEC5_DEV_DATA_FLAG_EN);
	data->tmout_head = NULL;

	LOG_DBG("WDT Disabled");

	return 0;
}

#ifdef CONFIG_WDT_MULTISTAGE
/* WDT_FLAG_RESET_NONE
 * WDT_FLAG_RESET_CPU_CORE - Not supported by HW
 * WDT_FLAG_RESET_SOC
 *
 * None may have WDT_FLAG_RESET_CPU_CORE set
 * Last may have WDT_FLAG_RESET_SOC
 */
static int check_wdt_timeouts(const struct wdt_timeout_cfg *tmout_cfg)
{
	const struct wdt_timeout_cfg *p = tmout_cfg;

	if (!p) {
		return -EINVAL;
	}

	while (!p) {
		if (p->window.min || !p->window.max) {
			return -EINVAL;
		}
		if (p->flags & WDT_FLAG_RESET_CPU_CORE) {
			return -EINVAL;
		}
		p = p->next;
	}

	return 0;
}

static bool wdt_timeouts_have_callback(const struct wdt_timeout_cfg *tmout_cfg)
{
	const struct wdt_timeout_cfg *p = tmout_cfg;

	if (!p) {
		return false;
	}

	while (!p) {
		if (p->callback) {
			return true;
		}
		p = p->next;
	}

	return false;
}

static bool require_wdt_interrupt(const struct wdt_timeout_cfg *tmout_cfg)
{
	if (tmout_cfg->callback || || tmout_cfg->next
	    || !(tmout_cfg->flags & WDT_FLAG_RESET_SOC)) {
		return true;
	}

	return false;
}

#else
static int check_wdt_timeouts(const struct wdt_timeout_cfg *tmout_cfg)
{
	const struct wdt_timeout_cfg *p = tmout_cfg;

	if (!p || p->window.min || !p->window.max || (p->flags & WDT_FLAG_RESET_CPU_CORE)) {
		return -EINVAL;
	}

	return 0;
}

static bool require_wdt_interrupt(const struct wdt_timeout_cfg *tmout_cfg)
{
	if (tmout_cfg->callback || !(tmout_cfg->flags & WDT_FLAG_RESET_SOC)) {
		return true;
	}

	return false;
}
#endif

/* MEC5 WDT hardware does not support windowed timeouts.
 * The timeout config window min must be 0.
 * wdt_timeout_cfg.flags
 * WDT_FLAG_RESET_NONE - We can support this by using WDT interrupt mode
 * and disabling the WDT in the ISR.
 * WDT_FLAG_RESET_CPU_CORE - Hardware does not support this mode.
 * WDT_FLAG_RESET_SOC - If ISR mode disabled the WDT will reset the SoC.
 * If ISR is enabled, WDT automatically reload and clear interrupt mode
 * bit in control register. When WDT counts down the second time it will
 * reset SoC (interrupt mode is disabled).
 * struct wdt_timeout_cfg member struct wdt_window has two members:
 *   uint32_t min, max in milliseconds
 *   MEC5 WDT HW does not support a timeout window so we expect min == 0.
 *   MEC5 WDT HW count units are 1.007 ms in a 16-bit counter register.
 *   If max > UINT16_MAX we set WDT counter to UINT16_MAX.
 * CONFIG_WDT_MULTISTAGE support
 *   We can support multiple stages via software.
 *   Stages are a linked list of wdt_timeout_cfg structures
 *   Each structure has WDT flags.
 *   The linked list must be passed in one call to install timeout.
 *
 *   Scenarios:
 *	timeout1: callback, no reset: Need WDT interrupt
 *	timeout2: no callback, no reset: Need WDT interrupt to load next.
 *	timeout3: callback, reset: Need WDT interrupt
 */
static int wdt_mec5_install_timeout(const struct device *dev,
				    const struct wdt_timeout_cfg *tmout_cfg)
{
	const struct wdt_mec5_dev_cfg *devcfg = dev->config;
	struct mec_wdt_regs *const regs = devcfg->regs;
	struct wdt_mec5_dev_data *data = dev->data;
	uint16_t load_val = UINT16_MAX;
	int ret = check_wdt_timeouts(tmout_cfg);

	if (ret) {
		data->tmout_head = NULL;
		return ret;
	}

	if (mec_hal_wdt_is_enabled(regs)) {
		return -EBUSY;
	}

	data->tmout_head = tmout_cfg;

	/* set LOAD to reset default */
	mec_hal_wdt_reload(regs, UINT16_MAX);

	if (require_wdt_interrupt(tmout_cfg)) {
		mec_hal_wdt_intr_ctrl(regs, 1);
		LOG_DBG("WDT interrupt enabled");
	} else {
		mec_hal_wdt_intr_ctrl(regs, 0);
		LOG_DBG("WDT Reset enabled");
	}

	if (tmout_cfg->window.max < UINT16_MAX) {
		load_val = (uint16_t)tmout_cfg->window.max;
		LOG_DBG("WDT timeout exceeds 16-bit HW counter. Clipping to 0xffff");
	}

	mec_hal_wdt_reload(regs, load_val);

	return 0;
}

static int wdt_mec5_feed(const struct device *dev, int channel_id)
{
	const struct wdt_mec5_dev_cfg *devcfg = dev->config;
	struct mec_wdt_regs *const regs = devcfg->regs;

	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);

	if (!mec_hal_wdt_is_enabled(regs)) {
		return -EINVAL;
	}

	LOG_DBG("WDT Feed");

	mec_hal_wdt_restart(regs);

	return 0;
}

#ifdef CONFIG_WDT_MULTISTAGE
static void wdt_multistate_next(const struct device *dev)
{

}
#endif

/* WDT fires an interrupt if control register WDT_RESET bit is set and its interrupt
 * enable register enable bit is set. Once the WDT count expires WDT asserts its
 * interrupt signal and clears the WDT_RESET bit in the control register. WDT also
 * reloads COUNT from the LOAD register.
 *
 * Not CONFIG_WDT_MULTISTAGE
 *  if current timeout has callback exists invoke it
 *  if current timeout flags has WDT_FLAG_RESET_SOC then reconfigure WDT for 1 count (1.007ms)
 *  and reload it. With WDT.Control.WDT_IEN cleared by HW on ISR entry then WDT will reset
 *  SoC on next count down.
 *  else disable WDT.
 */
static void wdt_mec5_isr(const struct device *dev)
{
	const struct wdt_mec5_dev_cfg *devcfg = dev->config;
	struct mec_wdt_regs *const regs = devcfg->regs;
	struct wdt_mec5_dev_data *data = dev->data;
	const struct wdt_timeout_cfg *tmcfg = data->tmout_head;

	LOG_DBG("WDT ISR");

	if (tmcfg && tmcfg->callback) {
		tmcfg->callback(dev, 0);
	}

	if (tmcfg->flags & WDT_FLAG_RESET_SOC) {
		mec_hal_wdt_reload(regs, 1);
		/* one tick (1.007 ms) to reset */
		while (mec_hal_wdt_count(regs)) {
			;
		}
	}

#ifdef CONFIG_WDT_MULTISTAGE
	if (tmcfg->next) {
		tmcfg = tmcfg->next;
		data->tmout_head = tmcfg;
		mec_hal_wdt_reload(regs, tmcfg->max);
		if (tmcfg->flags & WDT_FLAG_RESET_SOC) {
			return;
		}
	}
#endif
	mec_hal_wdt_intr_ctrl(regs, 1);
}

static const struct wdt_driver_api wdt_mec5_api = {
	.setup = wdt_mec5_setup,
	.disable = wdt_mec5_disable,
	.install_timeout = wdt_mec5_install_timeout,
	.feed = wdt_mec5_feed,
};

static int wdt_mec5_init(const struct device *dev)
{
	const struct wdt_mec5_dev_cfg *devcfg = dev->config;
	struct mec_wdt_regs *const regs = devcfg->regs;

	if (IS_ENABLED(CONFIG_WDT_DISABLE_AT_BOOT)) {
		mec_hal_wdt_init(regs, 0, 0);
	}

	if (devcfg->irq_config) {
		devcfg->irq_config(dev);
	}

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int wdt_mec5_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct wdt_mec5_dev_cfg *devcfg = dev->config;
	struct mec_wdt_regs *const regs = devcfg->regs;
	struct wdt_mec5_dev_data *data = dev->data;
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		if ((data->options & WDT_OPT_PAUSE_IN_SLEEP) &&
		    (data->flags & WDT_MEC5_DEV_DATA_FLAG_EN)) {
			mec_hal_wdt_enable(regs);
		}
	break;
	case PM_DEVICE_ACTION_SUSPEND:
		if (data->options & WDT_OPT_PAUSE_IN_SLEEP) {
			mec_hal_wdt_disable(regs);
		}
	break;
	default:
		ret = -ENOTSUP;
	}
	return ret;
}
#endif /* CONFIG_PM_DEVICE */

#define WDT_MEC5_DEV_INIT(i)						\
	static struct wdt_mec5_dev_data wdt_mec5_dev_data_##i;		\
									\
	static void wdt_mec5_irq_config_##i (const struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(i),				\
			    DT_INST_IRQ(i, priority),			\
			    wdt_mec5_isr,				\
			    DEVICE_DT_INST_GET(i), 0);			\
		irq_enable(DT_INST_IRQN(i));				\
	}								\
									\
	PM_DEVICE_DT_INST_DEFINE(i, wdt_mec5_pm_action);		\
									\
	static const struct wdt_mec5_dev_cfg wdt_mec5_devcfg_##i = {	\
		.regs = (struct mec_wdt_regs *)(DT_INST_REG_ADDR(0)),	\
		.irq_config = wdt_mec5_irq_config_##i,			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(i, wdt_mec5_init,				\
			      PM_DEVICE_DT_INST_GET(i),			\
			      &wdt_mec5_dev_data_##i,			\
			      &wdt_mec5_devcfg_##i,			\
			      PRE_KERNEL_1,				\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &wdt_mec5_api);

DT_INST_FOREACH_STATUS_OKAY(WDT_MEC5_DEV_INIT)
