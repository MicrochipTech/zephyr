/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_eeprom

#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/kernel.h>
#include <soc.h>

#include <zephyr/drivers/pinctrl.h>

#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

/* HAL */
#include <mec_eeprom_api.h>

LOG_MODULE_REGISTER(eeprom_mec5, CONFIG_EEPROM_LOG_LEVEL);

#define EEPROM_MEC5_PAGE_RD_TIME_MS_DFLT	2
/* write time is constant for 1 byte to page size */
#define EEPROM_MEC5_WR_TIME_MS_DFLT		5

#define EEPROM_MEC5_SPIN_WAIT_US		1000u
#define EEPROM_MEC5_SPIN_LOOPS			6u

#define EEPROM_MEC5_CFG_FLAG_JTAG_LOCK		BIT(0)

struct eeprom_mec5_devcfg {
	struct mec_eeprom_ctrl_regs *const regs;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config)(const struct device *dev);
	size_t size;
	uint32_t pswd;
	uint8_t cfg_flags;
	uint8_t page_size;
	uint8_t page_rd_time_ms;
	uint8_t wr_time_ms;
};

struct eeprom_mec5_data {
	struct k_sem lock;
	struct k_sem sync;
	uint32_t isr_status;
#if defined(CONFIG_PM) && !defined(CONFIG_PM_DEVICE)
	const struct pm_state_info *states;
	uint8_t num_pm_states;
#endif
};

#if defined(CONFIG_PM) || defined(CONFIG_PM_DEVICE)
static void eeprom_mec5_pm_busy_set(const struct device *dev)
{
#if defined(CONFIG_PM_DEVICE)
	pm_device_busy_set(dev);
#elif defined(CONFIG_PM)
	struct eeprom_mec5_data *const data = dev->data;

	for (uint8_t n = 0; n < data->num_pm_state; n++) {
		if (states[n].state >= PM_STATE_SUSPEND_TO_IDLE) {
			pm_policy_state_lock_get(states[n].state, PM_ALL_SUBSTATES);
		}
	}
#endif
}

static void eeprom_mec5_pm_busy_clear(const struct device *dev)
{
#if defined(CONFIG_PM_DEVICE)
	pm_device_busy_clear(dev);
#elif defined(CONFIG_PM)
	struct eeprom_mec5_data *const data = dev->data;

	for (uint8_t n = 0; n < data->num_pm_state; n++) {
		if (states[n].state >= PM_STATE_SUSPEND_TO_IDLE) {
			pm_policy_state_lock_put(states[n].state, PM_ALL_SUBSTATES);
		}
	}
#endif
}
#else
static void eeprom_mec5_pm_busy_set(const struct device *dev)
{
	ARG_UNUSED(dev);
}
static void eeprom_mec5_pm_busy_clear(const struct device *dev)
{
	ARG_UNUSED(dev);
}
#endif /* defined(CONFIG_PM) || defined(CONFIG_PM_DEVICE) */

/* Send command to EEPROM and wait for completion.
 * MEC_HAL_EEPROM_OP_READ_DATA
 * MEC_HAL_EEPROM_OP_WRITE_DATA
 * MEC_HAL_EEPROM_OP_READ_STATUS ignores offset and len
 * MEC_HAL_EEPROM_OP_WRITE_STATUS ignores offset and len
 */
static int eeprom_mec5_send_cmd(const struct device *dev, uint8_t cmd,
				uint32_t offset, uint32_t len)
{
	const struct eeprom_mec5_devcfg *devcfg = dev->config;
	struct mec_eeprom_ctrl_regs *const regs = devcfg->regs;
	struct eeprom_mec5_data *const data = dev->data;
	uint32_t tmout_ms = 0;
	int ret = 0;

	if (len > MEC_HAL_EEPROM_MAX_XFR_LEN) {
		return -EINVAL;
	}

	if (cmd == MEC_HAL_EEPROM_OP_READ_DATA) {
		tmout_ms = devcfg->page_rd_time_ms;
	} else if (cmd == MEC_HAL_EEPROM_OP_WRITE_DATA) {
		tmout_ms = devcfg->wr_time_ms;
	} else {
		tmout_ms = 1u;
	}

	k_sem_reset(&data->sync);

	/* Use interrupts */
	mec_hal_eeprom_status_clr(regs, BIT(MEC_HAL_EEPROM_STS_DONE_POS)
					| BIT(MEC_HAL_EEPROM_STS_ERR_POS));
	mec_hal_eeprom_intr_en(regs, 1, BIT(MEC_HAL_EEPROM_INTR_DONE_POS)
					| BIT(MEC_HAL_EEPROM_INTR_ERR_POS));

	ret = mec_hal_eeprom_xfr_start(regs, cmd, offset, len);
	if (ret) {
		LOG_ERR("HAL Start transfer error(%d)", ret);
		return -EIO;
	}

	ret = k_sem_take(&data->sync, K_MSEC(tmout_ms));
	if (ret == -EAGAIN) {
		LOG_ERR("Timeout");
		return -ETIMEDOUT;
	}

	/* Transfer error? */
	if (data->isr_status & BIT(MEC_HAL_EEPROM_STS_ERR_POS)) {
		LOG_ERR("Illegal cmd or cmd sent while busy");
		return -EIO;
	}

	return ret;
}

/* Send Read Status command to EEPROM and if successful retrieve 8-bit status
 * from byte 0 of the controller's 32-byte data buffer.
 */
static int eeprom_mec5_read_status(const struct device *dev, uint8_t *eeprom_status)
{
	const struct eeprom_mec5_devcfg *devcfg = dev->config;
	struct mec_eeprom_ctrl_regs *const regs = devcfg->regs;
	int ret = eeprom_mec5_send_cmd(dev, MEC_HAL_EEPROM_OP_READ_STATUS, 0, 0);

	if (ret) {
		return ret;
	}

	if (eeprom_status) {
		mec_hal_eeprom_buffer_rd(regs, eeprom_status, 1);
	}

	return 0;
}

/* Read EEPROM 8-bit status until busy clears or timeout */
static int eeprom_mec5_wait_not_busy(const struct device *dev, uint32_t sleep_us,
				     uint32_t loops)
{
	uint32_t n = 0;

	do {
		uint8_t eeprom_status = 0xffu;

		if (!eeprom_mec5_read_status(dev, &eeprom_status)) {
			if (!(eeprom_status & BIT(MEC_HAL_EEPROM_FABRIC_WR_BUSY_POS))) {
				return 0;
			}
		}

		k_sleep(K_USEC(sleep_us));
	} while (n++ < loops);

	LOG_ERR("Busy wait timeout");

	return -ETIMEDOUT;
}

/* MEC5 EEPROM controller allows writes within a page up to the page size.
 * Writes cannot cross a page boundary. Requests crossing a page boundary
 * must be broken up into separate transactions.
 */
static int eeprom_mec5_write(const struct device *dev, off_t offset, const void *buf, size_t len)
{
	const struct eeprom_mec5_devcfg *devcfg = dev->config;
	struct mec_eeprom_ctrl_regs *const regs = devcfg->regs;
	struct eeprom_mec5_data *const data = dev->data;
	const uint8_t *src = buf;
	uint32_t maxlen = 0, xfrlen = 0, ofs = 0;
	int ret = 0;

	if (len == 0) {
		return 0;
	}

	if ((offset + len) > devcfg->size) {
		LOG_WRN("attempt to write past device boundary");
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER);

	eeprom_mec5_pm_busy_set(dev);

	if (devcfg->pswd) {
		mec_hal_eeprom_unlock(regs, devcfg->pswd);
	}

	ofs = offset;
	while (len) {
		maxlen = (ofs + devcfg->page_size) & ~((uint32_t)devcfg->page_size - 1u);
		maxlen -= ofs;

		xfrlen = len;
		if (xfrlen > maxlen) {
			xfrlen = maxlen;
		}

		ret = mec_hal_eeprom_buffer_wr(regs, src, xfrlen);
		if (ret) {
			ret = -EIO;
			break;
		}

		ret = eeprom_mec5_send_cmd(dev, MEC_HAL_EEPROM_OP_WRITE_DATA,
					   ofs, xfrlen);
		if (ret) {
			break;
		}

		ret = eeprom_mec5_wait_not_busy(dev, EEPROM_MEC5_SPIN_WAIT_US,
						EEPROM_MEC5_SPIN_LOOPS);
		if (ret) {
			break;
		}

		src += xfrlen;
		ofs += xfrlen;
		len -= xfrlen;
	}

	if (devcfg->pswd) {
		mec_hal_eeprom_lock(regs);
	}

	eeprom_mec5_pm_busy_clear(dev);

	k_sem_give(&data->lock);

	return ret;
}

/* MEC5 EEPROM controller allows reads the page size. Reads may cross page boundaries.
 * We must break up reads into page sized chunks.
 */
static int eeprom_mec5_read(const struct device *dev, off_t offset, void *buf, size_t len)
{
	const struct eeprom_mec5_devcfg *devcfg = dev->config;
	struct mec_eeprom_ctrl_regs *const regs = devcfg->regs;
	struct eeprom_mec5_data *const data = dev->data;
	uint8_t *dest = buf;
	uint32_t xfrlen = 0, ofs = 0;
	int ret = 0;

	if (len == 0) {
		return 0;
	}

	if ((offset + len) > devcfg->size) {
		LOG_WRN("attempt to read past device boundary");
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER);

	eeprom_mec5_pm_busy_set(dev);

	if (devcfg->pswd) {
		mec_hal_eeprom_unlock(regs, devcfg->pswd);
	}

	ofs = offset;
	while (len) {
		xfrlen = len;
		if (xfrlen > MEC_HAL_EEPROM_MAX_XFR_LEN) {
			xfrlen = MEC_HAL_EEPROM_MAX_XFR_LEN;
		}

		ret = eeprom_mec5_send_cmd(dev, MEC_HAL_EEPROM_OP_READ_DATA,
					   ofs, xfrlen);
		if (ret) {
			break;
		}

		ret = mec_hal_eeprom_buffer_rd(regs, dest, xfrlen);
		if (ret) {
			ret = -EIO;
		}

		dest += xfrlen;
		ofs += xfrlen;
		len -= xfrlen;
	}

	if (devcfg->pswd) {
		mec_hal_eeprom_lock(regs);
	}

	eeprom_mec5_pm_busy_clear(dev);

	k_sem_give(&data->lock);

	return ret;
}

static size_t eeprom_mec5_size(const struct device *dev)
{
	const struct eeprom_mec5_devcfg *devcfg = dev->config;

	return devcfg->size;
}

static void eeprom_mec5_isr(const struct device *dev)
{
	const struct eeprom_mec5_devcfg *devcfg = dev->config;
	struct mec_eeprom_ctrl_regs *const regs = devcfg->regs;
	struct eeprom_mec5_data *const data = dev->data;
	uint32_t hwsts = 0;

	hwsts = mec_hal_eeprom_status(regs);
	mec_hal_eeprom_status_clr(regs, hwsts);

	data->isr_status = hwsts;
	LOG_DBG("ISR HW status = 0x%0x", hwsts);

	k_sem_give(&data->sync);
}

#ifdef CONFIG_PM_DEVICE
static int eeprom_mec5_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct eeprom_mec5_devcfg *devcfg = dev->config;
	struct mec_eeprom_ctrl_regs *const regs = devcfg->regs;
	int ret;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret != 0) {
			LOG_ERR("PM resume: pinctrl setup failed (%d)", ret);
			return ret;
		}
		mec_hal_eeprom_activate(regs, 1);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		mec_hal_eeprom_activate(regs, 0); /* gates off clocks */
		ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_SLEEP);
		if (ret == -ENOENT) { /* pinctrl-1 does not exist? */
			ret = 0;
		}
		break;
	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */



#if defined(CONFIG_PM) && !defined(CONFIG_PM_DEVICE)
static void eeprom_mec5_pm_cfg(const struct device *dev);
{
	struct eeprom_mec5_data *const data = dev->data;

	data->num_pm_states = pm_state_cpu_get_all(0, &data->states);
}
#endif

static int eeprom_mec5_init(const struct device *dev)
{
	const struct eeprom_mec5_devcfg *devcfg = dev->config;
	struct mec_eeprom_ctrl_regs *const regs = devcfg->regs;
	uint32_t flags = MEC_HAL_EEPROM_CFG_SRST;
	int ret = 0;

	if (devcfg->pswd) {
		flags |= (MEC_HAL_EEPROM_CFG_LOAD_PSWD | MEC_HAL_EEPROM_CFG_LOCK_ON_PSWD);
	}
	if (devcfg->cfg_flags & EEPROM_MEC5_CFG_FLAG_JTAG_LOCK) {
		flags |= MEC_HAL_EEPROM_CFG_LOCK_ON_JTAG;
	}

	ret = mec_hal_eeprom_init(regs, flags, devcfg->pswd);
	if (ret != MEC_RET_OK) {
		LOG_ERR("init failed (%d)", ret);
		return -EIO;
	}

	ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("pinctrl init failed (%d)", ret);
		return ret;
	}

	if (devcfg->irq_config) {
		devcfg->irq_config(dev);
		mec_hal_eeprom_girq_ctrl(regs, 1);
	}

	mec_hal_eeprom_activate(regs, 1);

#if defined(CONFIG_PM) && !defined(CONFIG_PM_DEVICE)
	eeprom_mec5_pm_cfg(dev);
#endif

	return 0;
}

static const struct eeprom_driver_api eeprom_mec5_api = {
	.read = eeprom_mec5_read,
	.write = eeprom_mec5_write,
	.size = eeprom_mec5_size,
};

#define EEPROM_MEC5_INIT_LOCK(_data) \
	.lock = Z_SEM_INITIALIZER(_data.lock, 1, 1)

#define EEPROM_MEC5_INIT_SYNC(_data) \
	.sync = Z_SEM_INITIALIZER(_data.sync, 0, 1)

#define EEPROM_MEC5_CFG_FLAGS(inst) \
	(DT_INST_PROP_OR(inst, lock_jtag_active, 0) * EEPROM_MEC5_CFG_FLAG_JTAG_LOCK)

#define EEPROM_MEC5_PAGE_RD_TIME_VAL(inst) \
	(uint8_t)(DT_INST_PROP(inst, page_read_time_ms) & 0xffu)

#define EEPROM_MEC5_WR_TIME_VAL(inst) \
	(uint8_t)(DT_INST_PROP(inst, write_time_ms) & 0xffu)

#define DEFINE_MEC5_EEPROM(inst)						\
	PINCTRL_DT_INST_DEFINE(inst)						\
	PM_DEVICE_DT_INST_DEFINE(inst, eeprom_mec5_pm_action);			\
										\
	static struct eeprom_mec5_data eeprom_mec5_drv_data_##inst = {		\
		EEPROM_MEC5_INIT_LOCK(eeprom_mec5_drv_data_##inst),		\
		EEPROM_MEC5_INIT_SYNC(eeprom_mec5_drv_data_##inst),		\
	};									\
										\
	static void eeprom_mec5_irq_cfg_##inst(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(inst),					\
			    DT_INST_IRQ(inst, priority),			\
			    eeprom_mec5_isr,					\
			    DEVICE_DT_INST_GET(inst), 0);			\
		irq_enable(DT_INST_IRQN(inst));					\
	}									\
										\
	static const struct eeprom_mec5_devcfg eeprom_mec5_devcfg_##inst = {	\
		.regs = (struct mec_eeprom_ctrl_regs *)DT_INST_REG_ADDR(inst),	\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),			\
		.irq_config = eeprom_mec5_irq_cfg_##inst,			\
		.size = DT_INST_PROP(inst, size),				\
		.pswd = DT_INST_PROP_OR(inst, lock_password, 0),		\
		.cfg_flags = EEPROM_MEC5_CFG_FLAGS(inst),			\
		.page_size = (uint8_t)DT_INST_PROP(inst, page_size),		\
		.page_rd_time_ms = EEPROM_MEC5_PAGE_RD_TIME_VAL(inst),		\
		.wr_time_ms = EEPROM_MEC5_WR_TIME_VAL(inst),			\
	};									\
	DEVICE_DT_INST_DEFINE(inst, &eeprom_mec5_init,				\
			      PM_DEVICE_DT_INST_GET(inst),			\
			      &eeprom_mec5_drv_data_##inst,			\
			      &eeprom_mec5_devcfg_##inst,			\
			      POST_KERNEL, CONFIG_EEPROM_INIT_PRIORITY,		\
			      &eeprom_mec5_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_MEC5_EEPROM);
