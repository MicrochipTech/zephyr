/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_ps2

#include <cmsis_core.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/drivers/ps2.h>
#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/gpio.h>

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_ecia_api.h>
#include <mec_ps2_api.h>

#define LOG_LEVEL CONFIG_PS2_LOG_LEVEL
LOG_MODULE_REGISTER(ps2_mchp_mec5);

/* in 50us units */
#define MEC5_PS2_TIMEOUT	10000u
#define MEC5_PS2_BUSY_WAIT_US	50u

#define MEC5_PS2_ANY_RX_ERR	(MEC_PS2_STS_RX_TMOUT | MEC_PS2_STS_PARITY_ERR |\
				 MEC_PS2_STS_FRAME_ERR)

struct ps2_mec5_devcfg {
	struct mec_ps2_regs *const regs;
	int isr_nvic;
	void (*irq_config_func)(void);
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_PM_DEVICE
	struct gpio_dt_spec wakerx_gpio;
	bool wakeup_source;
#endif
	uint8_t port;
};

struct ps2_mec5_data {
	ps2_callback_t callback_isr;
	struct k_sem tx_lock;
};

#ifdef CONFIG_PM_DEVICE
static void ps2_mec5_device_busy_set(const struct device *dev)
{
	pm_device_busy_set(dev);
}

static void ps2_mec5_device_busy_clear(const struct device *dev)
{
	pm_device_busy_clear(dev);
}
#else
static void ps2_mec5_device_busy_set(const struct device *dev)
{
#ifdef CONFIG_PM_POLICY_DEVICE_CONSTRAINTS
	pm_policy_device_power_lock_get(dev);
#else
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
#endif
}

static void ps2_mec5_device_busy_clear(const struct device *dev)
{
#ifdef CONFIG_PM_POLICY_DEVICE_CONSTRAINTS
	pm_policy_device_power_lock_put(dev);
#else
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
#endif
}
#endif

/* Configure and enble PS/2 controller and its interrupt.
 * Handle the case of self test for a PS2 device already finished.
 * Read PS/2 data to clear HW data ready status and then clear
 * all other PS/2 status. Instances must be allocated before the BAT
 * (Basic Assurance Test) or the host may time out.
 */
static int ps2_mec5_configure(const struct device *dev, ps2_callback_t callback_isr)
{
	const struct ps2_mec5_devcfg *const devcfg = dev->config;
	struct ps2_mec5_data *const data = dev->data;
	struct mec_ps2_regs *const regs = devcfg->regs;
	uint8_t opmsk = MEC_PS2_CTRL_OP_MSK_DIR | MEC_PS2_CTRL_OP_MSK_EN;
	uint8_t op = MEC_PS2_CTRL_OP_DIR_RX | MEC_PS2_CTRL_OP_ENABLE;
	uint8_t  __attribute__((unused)) temp;

	if (!callback_isr) {
		return -EINVAL;
	}

	ps2_mec5_device_busy_clear(dev);

	data->callback_isr = callback_isr;

	temp = mec_hal_ps2_read_data(regs);
	mec_hal_ps2_clr_status(regs, MEC_PS2_STATUS_ALL);
	mec_hal_ps2_girq_clr(regs);

	/* Enable FSM and interrupt generation to NVIC */
	mec_hal_ps2_control(regs, op, opmsk);
	mec_hal_ps2_girq_ctrl(regs, 1);

	k_sem_give(&data->tx_lock);

	return 0;
}

/* Write value to device connected to this PS/2 Controller.
 * This is a blocking operation. The controller may be currently
 * reading or writing data to the device. We must wait the current
 * operation to finish.
 */
static int ps2_mec5_write(const struct device *dev, uint8_t value)
{
	const struct ps2_mec5_devcfg *const devcfg = dev->config;
	struct ps2_mec5_data *const data = dev->data;
	struct mec_ps2_regs *const regs = devcfg->regs;
	uint32_t status = 0u;
	int i = 0;
	uint8_t opmsk = MEC_PS2_CTRL_OP_MSK_DIR | MEC_PS2_CTRL_OP_MSK_EN;
	uint8_t op = MEC_PS2_CTRL_OP_DIR_RX | MEC_PS2_CTRL_OP_DISABLE;

	uint8_t  __attribute__((unused)) temp;

	if (k_sem_take(&data->tx_lock, K_NO_WAIT)) {
		return -EACCES;
	}

	status = mec_hal_ps2_get_status(regs) & (MEC_PS2_STS_RX_BUSY | MEC_PS2_STS_TX_IDLE);
	while ((status != MEC_PS2_STS_TX_IDLE) && (i < MEC5_PS2_TIMEOUT)) {
		k_busy_wait(MEC5_PS2_BUSY_WAIT_US);
		i++;
		status = (mec_hal_ps2_get_status(regs)
			  & (MEC_PS2_STS_RX_BUSY | MEC_PS2_STS_TX_IDLE));
	}

	if (unlikely(i == MEC5_PS2_TIMEOUT)) {
		LOG_DBG("PS2 write timed out");
		return -ETIMEDOUT;
	}

	ps2_mec5_device_busy_set(dev);

	/* disable and set direction to read */
	mec_hal_ps2_control(regs, op, opmsk);

	/* clear the RX data ready R/O status */
	temp = mec_hal_ps2_read_data(regs);
	/* HW requirement to not change PS/2 control register too quickly */
	k_sleep(K_MSEC(1));
	/* clear all other status */
	mec_hal_ps2_clr_status(regs, MEC_PS2_STATUS_ALL);
	mec_hal_ps2_girq_clr(regs);

	/* Enable for TX */
	op = MEC_PS2_CTRL_OP_DIR_TX | MEC_PS2_CTRL_OP_ENABLE;
	mec_hal_ps2_control(regs, op, opmsk);
	mec_hal_ps2_send_data(regs, value);

	k_sem_give(&data->tx_lock);

	return 0;
}

/* Inhibit PS/2 controller. The external PS/2 device sees the clock low and does not
 * generate clocks.
 * Set direction to read and disable (drivers PS/2 clock low).
 * Clear PS/2 controller status
 * Clear PS/2 GIRQ in MEC5 interrupt aggregator
 * Clear PS/2 NVIC status from GIRQ.
 */
static int ps2_mec5_inhibit_interface(const struct device *dev)
{
	const struct ps2_mec5_devcfg *const devcfg = dev->config;
	struct ps2_mec5_data *const data = dev->data;
	struct mec_ps2_regs *const regs = devcfg->regs;
	uint8_t opmsk = MEC_PS2_CTRL_OP_MSK_DIR | MEC_PS2_CTRL_OP_MSK_EN;
	uint8_t op = MEC_PS2_CTRL_OP_DIR_RX | MEC_PS2_CTRL_OP_DISABLE;

	if (k_sem_take(&data->tx_lock, K_MSEC(10)) != 0) {
		return -EACCES;
	}

	mec_hal_ps2_control(regs, op, opmsk);
	mec_hal_ps2_clr_status(regs, MEC_PS2_STATUS_ALL);
	mec_hal_ps2_girq_clr(regs);
	NVIC_ClearPendingIRQ(devcfg->isr_nvic);

	ps2_mec5_device_busy_clear(dev);
	k_sem_give(&data->tx_lock);

	return 0;
}

/* Enable PS/2 interface. PS/2 controller stops driving the clock low allowing the
 * external device to generate clocks based when it has a message to send/receive.
 * We enable the PS/2 controller for receive.
 */
static int ps2_mec5_enable_interface(const struct device *dev)
{
	const struct ps2_mec5_devcfg *const devcfg = dev->config;
	struct ps2_mec5_data *const data = dev->data;
	struct mec_ps2_regs *const regs = devcfg->regs;
	uint8_t opmsk = MEC_PS2_CTRL_OP_MSK_DIR | MEC_PS2_CTRL_OP_MSK_EN;
	uint8_t op = MEC_PS2_CTRL_OP_DIR_RX | MEC_PS2_CTRL_OP_ENABLE;

	if (k_sem_take(&data->tx_lock, K_MSEC(10)) != 0) {
		return -EACCES;
	}

	mec_hal_ps2_clr_status(regs, MEC_PS2_STATUS_ALL);
	mec_hal_ps2_girq_clr(regs);
	mec_hal_ps2_control(regs, op, opmsk);

	k_sem_give(&data->tx_lock);

	return 0;
}

#ifdef CONFIG_PM_DEVICE
/* Suspend:
 *  If this PS/2 interface DT wakeup source is true
 *    Clear and enable GIRQ PS/2 wake interrupt and if wakerx_gpio pin was specified the
 *    corresponding GPIO interrupt.
 *  Else
 *    Apply default PINCTRL state.
 *    Disable PS/2 interface logic preserving direction.
 * Resume:
 *  If this PS/2 interface DT wakeup source is true
 *    Disable and clear GIRQ PS/2 wake interrupt and if wakerx_gpio pin was specified the
 *    corresponding GPIO interrupt.
 *  Else
 *   Apply default PINCTRL state and enable PS/2 interface in same direction it was when
 *   suspend was entered.
 */
static int ps2_mec5_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct ps2_mec5_devcfg *const devcfg = dev->config;
	struct mec_ps2_regs *const regs = devcfg->regs;
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		if (devcfg->wakeup_source) {
			if (devcfg->wakerx_gpio.port != NULL) {
				ret = gpio_pin_interrupt_configure_dt(
						&devcfg->wakerx_gpio,
						GPIO_INT_DISABLE);
				if (ret < 0) {
					LOG_ERR("Fail to disable PS2 wake interrupt (ret %d)", ret);
					return ret;
				}
			}

			mec_hal_ps2_girq_wake_enable(regs, devcfg->port, 0);
			mec_hal_ps2_girq_wake_clr(regs, devcfg->port);
		} else {
			ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);
			mec_hal_ps2_control(regs, MEC_PS2_CTRL_OP_ENABLE, MEC_PS2_CTRL_OP_MSK_EN);
		}
	break;
	case PM_DEVICE_ACTION_SUSPEND:
		if (devcfg->wakeup_source) {
			/* Enable PS2 wake interrupt
			 * Configure Falling Edge Trigger interrupt on PS2DAT pin
			 */
			mec_hal_ps2_girq_wake_clr(regs, devcfg->port);
			mec_hal_ps2_girq_wake_enable(regs, devcfg->port, 1u);

			if (devcfg->wakerx_gpio.port != NULL) {
				ret = gpio_pin_interrupt_configure_dt(
					&devcfg->wakerx_gpio,
					GPIO_INT_MODE_EDGE | GPIO_INT_TRIG_LOW);
				if (ret < 0) {
					LOG_ERR("Fail to enable PS2 wake interrupt(ret %d)", ret);
					return ret;
				}
			}
		} else {
			mec_hal_ps2_control(regs, MEC_PS2_CTRL_OP_DISABLE, MEC_PS2_CTRL_OP_MSK_EN);
			ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_SLEEP);
			if (ret == -ENOENT) { /* pinctrl-1 does not exist.  */
				ret = 0;
			}
		}
	break;
	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

/* PS/2 ISR
 * NOTE: PS/2 target interface has some read-only status that are only
 * cleared by performing other actions such as reading the receive buffer
 * for receive data ready.
 */
static void ps2_mec5_isr(const struct device *dev)
{
	const struct ps2_mec5_devcfg *const devcfg = dev->config;
	struct ps2_mec5_data *const data = dev->data;
	struct mec_ps2_regs *const regs = devcfg->regs;
	bool release_pm_lock = true;
	uint32_t status = 0;
	uint8_t ps2_data = 0;

	/* read status and clear all status except data ready */
	status = mec_hal_ps2_get_status(regs);
	mec_hal_ps2_clr_status(regs, status);
	mec_hal_ps2_girq_clr(regs);

	if (status & MEC_PS2_STS_DATA_RDY) {
		ps2_mec5_device_busy_set(dev);

		/* disable to inhibit external device from generating more clocks */
		mec_hal_ps2_control(regs, MEC_PS2_CTRL_OP_DISABLE | MEC_PS2_CTRL_OP_DIR_RX,
				    MEC_PS2_CTRL_OP_MSK_DIR | MEC_PS2_CTRL_OP_MSK_EN);

		ps2_data = mec_hal_ps2_read_data(regs);
		/* reading data clears R/O data available status. We can clear GIRQ */
		mec_hal_ps2_girq_clr(regs);

		if (data->callback_isr) {
			data->callback_isr(dev, ps2_data);
		}
	} else if (status & (MEC_PS2_STS_TX_TMOUT | MEC_PS2_STS_TX_START_TMOUT)) {
		LOG_ERR("TX timeout: 0x%x", status);
	} else if (status & MEC5_PS2_ANY_RX_ERR) {
		release_pm_lock = false;
		LOG_ERR("RX error: 0x%x", status);
	} else if (status & MEC_PS2_STS_TX_IDLE) { /* TX done */
		LOG_DBG("TX idle: 0x%x", status);
	}

	if (release_pm_lock) {
		ps2_mec5_device_busy_clear(dev);
	}

	mec_hal_ps2_control(regs, MEC_PS2_CTRL_OP_ENABLE | MEC_PS2_CTRL_OP_DIR_RX,
			    MEC_PS2_CTRL_OP_MSK_EN | MEC_PS2_CTRL_OP_MSK_DIR);
}

static const struct ps2_driver_api ps2_mec5_driver_api = {
	.config = ps2_mec5_configure,
	.read = NULL,
	.write = ps2_mec5_write,
	.disable_callback = ps2_mec5_inhibit_interface,
	.enable_callback = ps2_mec5_enable_interface,
};

static int ps2_mec5_init(const struct device *dev)
{
	const struct ps2_mec5_devcfg *const devcfg = dev->config;
	struct ps2_mec5_data *const data = dev->data;
	struct mec_ps2_regs *const regs = devcfg->regs;
	uint32_t init_flags = MEC_PS2_FLAGS_RESET;
	int ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("MEC5 PS2 pinctrl init failed (%d)", ret);
		return ret;
	}

	ret = mec_hal_ps2_init(regs, init_flags);
	if (ret) {
		LOG_ERR("MEC5 PS2 HAL init failed (%d)", ret);
		return -EIO;
	}

	k_sem_init(&data->tx_lock, 0, 1);

	devcfg->irq_config_func();

	return 0;
}

/* To enable wakeup on the PS2, the DTS needs to have two entries defined
 * in the corresponding PS2 node in the DTS specifying it as a wake source
 * and specifying the PS2DAT GPIO; example as below
 *
 *	wakerx-gpios = <MCHP_GPIO_DECODE_115 GPIO_ACTIVE_HIGH>
 *	wakeup-source;
 */
#ifdef CONFIG_PM_DEVICE
#define MEC5_PS2_PM_WAKEUP(n)						\
		.wakeup_source = (uint8_t)DT_INST_PROP_OR(n, wakeup_source, 0),	\
		.wakerx_gpio = GPIO_DT_SPEC_INST_GET_OR(n, wakerx_gpios, {0}),
#else
#define MEC5_PS2_PM_WAKEUP(index) /* Not used */
#endif

#define MEC5_PS2_PINCTRL_CFG(inst) PINCTRL_DT_INST_DEFINE(inst)
#define MEC5_PS2_CONFIG(inst)							\
	static const struct ps2_mec5_devcfg ps2_mec5_dcfg_##inst = {		\
		.regs = (struct mec_ps2_regs * const)(DT_INST_REG_ADDR(inst)),	\
		.isr_nvic = DT_INST_IRQN(inst),					\
		.irq_config_func = ps2_mec5_irq_config_func_##inst,		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),			\
		.port = DT_INST_PROP_OR(inst, port, 0),				\
		MEC5_PS2_PM_WAKEUP(inst)					\
	}

#define PS2_MEC5_DEVICE(i)							\
										\
	static void ps2_mec5_irq_config_func_##i(void)				\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(i),					\
			    DT_INST_IRQ(i, priority),				\
			    ps2_mec5_isr,					\
			    DEVICE_DT_INST_GET(i), 0);				\
		irq_enable(DT_INST_IRQN(i));					\
	}									\
										\
	static struct ps2_mec5_data ps2_mec5_port_data_##i;			\
										\
	MEC5_PS2_PINCTRL_CFG(i);						\
										\
	MEC5_PS2_CONFIG(i);							\
										\
	PM_DEVICE_DT_INST_DEFINE(i, ps2_mec5_pm_action);			\
										\
	DEVICE_DT_INST_DEFINE(i, &ps2_mec5_init,				\
		PM_DEVICE_DT_INST_GET(i),					\
		&ps2_mec5_port_data_##i, &ps2_mec5_dcfg_##i,			\
		POST_KERNEL, CONFIG_PS2_INIT_PRIORITY,				\
		&ps2_mec5_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PS2_MEC5_DEVICE)
