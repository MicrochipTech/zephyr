/*
 * Copyright (c) 2024 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_input_kscan

#include <cmsis_core.h>
#include <errno.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_kbd_matrix.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_ecia_api.h>
#include <mec_kscan_api.h>

LOG_MODULE_REGISTER(input_kscan_mec5, CONFIG_INPUT_LOG_LEVEL);

struct mec5_ikscan_devcfg {
	struct input_kbd_matrix_common_config common;
	struct mec_kscan_regs *regs;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
	int irq_num;
	bool wakeup_source;
};

struct mec5_ikscan_data {
	struct input_kbd_matrix_common_data common;
	bool pm_lock_taken;
};

static void mec5_ikscan_device_busy_set(const struct device *dev)
{
#ifdef CONFIG_PM_POLICY_DEVICE_CONSTRAINTS
	pm_policy_device_power_lock_get(dev);
#else
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
#endif
}

static void mec5_ikscan_device_busy_clear(const struct device *dev)
{
#ifdef CONFIG_PM_POLICY_DEVICE_CONSTRAINTS
	pm_policy_device_power_lock_put(dev);
#else
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
#endif
}

static void mec5_ikscan_drive_column(const struct device *dev, int col)
{
	struct mec5_ikscan_devcfg const *devcfg = dev->config;
	struct mec_kscan_regs *regs = devcfg->regs;

	if (col == INPUT_KBD_MATRIX_COLUMN_DRIVE_ALL) {
		/* enable key scan and drive all KSO outputs */
		mec_hal_kscan_kso_drive_all(regs);
	} else if (col == INPUT_KBD_MATRIX_COLUMN_DRIVE_NONE) {
		/* disable key scan and drive all */
		mec_hal_kscan_kso_disable_keyscan(regs);
	} else {
		/* Drive KSO specified by parameter col low */
		mec_hal_kscan_kso_select(regs, (uint8_t)(col & 0xfu), 0);
	}
}

static kbd_row_t mec5_ikscan_read_row(const struct device *dev)
{
	const struct mec5_ikscan_devcfg *const devcfg = dev->config;
	struct mec_kscan_regs *const regs = devcfg->regs;

	/* In this implementation a 1 means key pressed */
	return (uint8_t)~mec_hal_kscan_ksi_state(regs);
}

static void mec5_ikscan_isr(const struct device *dev)
{
	const struct mec5_ikscan_devcfg *const devcfg = dev->config;
	struct mec_kscan_regs *const regs = devcfg->regs;

	mec_hal_kscan_girq_dis(regs);
	mec_hal_kscan_girq_clr(regs);

	input_kbd_matrix_poll_start(dev);
}

static void mec5_ikscan_set_detect_mode(const struct device *dev, bool enabled)
{
	struct mec5_ikscan_data *const data = dev->data;
	const struct mec5_ikscan_devcfg *const devcfg = dev->config;
	struct mec_kscan_regs *const regs = devcfg->regs;

	if (enabled) {
		if (data->pm_lock_taken) {
			mec5_ikscan_device_busy_clear(dev);
			data->pm_lock_taken = false;
		}

		mec_hal_kscan_ksi_status_clr(regs, MEC_KSCAN_KSI_INTR_ALL);
		mec_hal_kscan_girq_clr(regs);
		NVIC_ClearPendingIRQ(devcfg->irq_num);
		irq_enable(devcfg->irq_num);
		mec_hal_kscan_girq_en(regs);
	} else {
		mec5_ikscan_device_busy_set(dev);
		data->pm_lock_taken = true;
	}
}

#ifdef CONFIG_PM_DEVICE
static int mec5_ikscan_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct mec5_ikscan_devcfg *const devcfg = dev->config;
	struct mec_kscan_regs *const regs = devcfg->regs;
	int ret = input_kbd_matrix_pm_action(dev, action);

	if (ret < 0) {
		return ret;
	}

	if (devcfg->wakeup_source) {
		return 0;
	}

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret != 0) {
			LOG_ERR("XEC KSCAN pinctrl init failed (%d)", ret);
			return ret;
		}

		/* Need KSO scan enable API that clears bit[6] */
		/* Clear status register */
		mec_hal_kscan_ksi_status_clr(regs, MEC_KSCAN_KSI_INTR_ALL);
		mec_hal_kscan_ksi_intr_en_set(regs, MEC_KSCAN_KSI_INTR_ALL);
		break;

	case PM_DEVICE_ACTION_SUSPEND:
		mec_hal_kscan_kso_disable_keyscan(regs);
		mec_hal_kscan_ksi_intr_en_set(regs, 0);
		ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_SLEEP);
		if (ret != -ENOENT) {
			/* pinctrl-1 does not exist */
			return ret;
		}
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static int mec5_ikscan_init(const struct device *dev)
{
	const struct mec5_ikscan_devcfg *const devcfg = dev->config;
	struct mec_kscan_regs *const regs = devcfg->regs;
	uint32_t kscan_init_flags = (MEC_KSCAN_CFG_ENABLE | MEC_KSCAN_CFG_RESET
				     | MEC_KSCAN_KSO_PREDRIVE_EN | MEC_KSCAN_INTR_EN);
	int ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);

	if (ret != 0) {
		LOG_ERR("XEC KSCAN pinctrl init failed (%d)", ret);
		return ret;
	}

	ret = mec_hal_kscan_init(regs, kscan_init_flags, MEC_KSCAN_KSI_INTR_ALL);
	if (ret != MEC_RET_OK) {
		LOG_ERR("MEC5 KSCAN init failed (%d)", ret);
		return -EIO;
	}

	if (devcfg->irq_config_func) {
		devcfg->irq_config_func();
	}

	return input_kbd_matrix_common_init(dev);
}

static const struct input_kbd_matrix_api mec5_ikscan_api = {
	.drive_column = mec5_ikscan_drive_column,
	.read_row = mec5_ikscan_read_row,
	.set_detect_mode = mec5_ikscan_set_detect_mode,
};

#define MEC5_INPUT_KSCAN_INIT(i)								\
	BUILD_ASSERT(IN_RANGE(DT_INST_PROP(i, row_size), 1, 8), "invalid row-size");		\
	BUILD_ASSERT(IN_RANGE(DT_INST_PROP(i, col_size), 1, 18), "invalid col-size");		\
	static void mec5_ikscan_irq_config_func_##i(void)					\
	{											\
		IRQ_CONNECT(DT_INST_IRQN(i),							\
			    DT_INST_IRQ(i, priority),						\
			    mec5_ikscan_isr,							\
			    DEVICE_DT_INST_GET(i), 0);						\
		irq_enable(DT_INST_IRQN(i));							\
	}											\
	PINCTRL_DT_INST_DEFINE(i);								\
	PM_DEVICE_DT_INST_DEFINE(i, mec5_ikscan_pm_action);					\
	INPUT_KBD_MATRIX_DT_INST_DEFINE(i);							\
	static const struct mec5_ikscan_devcfg mec5_ikscan_dcfg_##i = {				\
		.common = INPUT_KBD_MATRIX_DT_INST_COMMON_CONFIG_INIT(i, &mec5_ikscan_api),	\
		.regs = (struct mec_kscan_regs *)DT_INST_REG_ADDR(i),				\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),					\
		.irq_num = DT_INST_IRQN(i),							\
		.wakeup_source = DT_INST_PROP(i, wakeup_source),				\
		.irq_config_func = mec5_ikscan_irq_config_func_##i,				\
	};											\
	static struct mec5_ikscan_data mec5_ikscan_data_##i;					\
	DEVICE_DT_INST_DEFINE(i, mec5_ikscan_init, PM_DEVICE_DT_INST_GET(i),			\
			      &mec5_ikscan_data_##i, &mec5_ikscan_dcfg_##i,			\
			      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MEC5_INPUT_KSCAN_INIT)
