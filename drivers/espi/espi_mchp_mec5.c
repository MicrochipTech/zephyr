/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_espi

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(espi, CONFIG_ESPI_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/dt-bindings/espi/mchp_mec5_espi.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <soc.h>
#include <zephyr/irq.h>

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_ecia_api.h>
#include <mec_espi_api.h>

/* local */
#include "espi_utils.h"
#include "espi_mchp_mec5.h"
#include "mchp/espi_mchp_mec5_regs.h"

/* If SoC was configured for Boot-ROM to load Zephyr application from
 * eSPI CAF then the eSPI link is up and Host has enabled VW channel.
 */
static bool is_espi_bootrom_config(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iob = (mm_reg_t)drvcfg->ioc_base;

	if ((sys_test_bit8(iob + ESPI_ACTV, ESPI_ACTV_EN_POS) != 0) &&
	    (sys_test_bit8(iob + ESPI_VW_READY, ESPI_CHAN_RDY_POS) != 0)) {
		return true;
	}

	return false;
}

/* eSPI Reset interrupt handler */
static void espi_mec5_ereset_isr(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t iob = (mm_reg_t)drvcfg->ioc_base;
	struct espi_event evt = { ESPI_BUS_RESET, 0, 0 };
	uint8_t erst_sts = 0, n_erst_state = 0;

	erst_sts = sys_read8(iob + ESPI_RESET_SR);
	sys_write8(iob + ESPI_RESET_SR, erst_sts);
	n_erst_state = (erst_sts >> ESPI_RESET_SR_STATE_POS) & BIT(0);
	evt.evt_data = n_erst_state;

#ifdef CONFIG_ESPI_PERIPHERAL_CHANNEL
	espi_mec5_pc_erst_config(dev, n_erst_state);
#endif
#ifdef CONFIG_ESPI_VWIRE_CHANNEL
	espi_mec5_vw_erst_config(dev, n_erst_state);
#endif
#ifdef CONFIG_ESPI_OOB_CHANNEL
	espi_mec5_oob_erst_config(dev, n_erst_state);
#endif
#ifdef CONFIG_ESPI_FLASH_CHANNEL
	espi_mec5_fc_erst_config(dev, n_erst_state);
#endif
	espi_send_callbacks(&data->callbacks, dev, evt);
}

/* -------- Public API -------- */
static int espi_mec5_config_api(const struct device *dev, struct espi_cfg *cfg)
{
	return -ENOTSUP;
}

static bool espi_mec5_get_chan_status_api(const struct device *dev, enum espi_channel ch)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iob = (mm_reg_t)drvcfg->ioc_base;
	uint32_t ofs = 0;

	if (ch == ESPI_CHANNEL_PERIPHERAL) {
		ofs = ESPI_PC_READY;
	} else if (ch == ESPI_CHANNEL_VWIRE) {
		ofs = ESPI_VW_READY;
	} else if (ch == ESPI_CHANNEL_OOB) {
		ofs = ESPI_OOB_READY;
	} else if (ch == ESPI_CHANNEL_FLASH) {
		ofs = ESPI_FC_READY;
	} else {
		return false;
	}

	if (sys_test_bit8(iob + ofs, ESPI_CHAN_RDY_POS)) {
		return true;
	}

	return false;
}

static int espi_mec5_manage_cb_api(const struct device *dev, struct espi_callback *callback,
				   bool set)
{
	struct espi_mec5_drv_data *data = dev->data;

	if ((callback == NULL) || (callback->handler == NULL)) {
		return -EINVAL;
	}

	return espi_manage_callback(&data->callbacks, callback, set);
}

static int espi_mec5_driver_init(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	struct espi_mec5_drv_data *data = dev->data;
	int ret = 0;

	data->dev = dev;
	data->status = 0;

	ret = pinctrl_apply_state(drvcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		return ret;
	}

	if (is_espi_bootrom_config(dev)) {
		return 0;
	}

#ifdef CONFIG_ESPI_VWIRE_CHANNEL
	ret = espi_mec5_init_vwires(dev);
	if (ret != 0) {
		return ret;
	}
#endif

	if (drvcfg->irq_config) {
		drvcfg->irq_config(dev);
	}

	return 0;
}

#if CONFIG_PM_DEVICE
static int espi_mec5_pm_action(const struct device *dev, enum pm_device_action action)
{
	/* TODO PM_DEVICE_ACTION_xxx */
	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static DEVICE_API(espi, espi_mec5_driver_api) = {
	.config = espi_mec5_config_api,
	.get_channel_status = espi_mec5_get_chan_status_api,
#ifdef CONFIG_ESPI_PERIPHERAL_CHANNEL
	.read_request = espi_mec5_read_req_api,
	.write_request = espi_mec5_write_req_api,
	.read_lpc_request = espi_mec5_read_lpc_req_api,
	.write_lpc_request = espi_mec5_write_lpc_req_api,
#endif
#ifdef CONFIG_ESPI_VWIRE_CHANNEL
	.send_vwire = espi_mec5_send_vwire_api,
	.receive_vwire = espi_mec5_recv_vwire_api,
#endif
#ifdef CONFIG_ESPI_OOB_CHANNEL
	.send_oob = espi_mec5_send_oob_api,
	.receive_oob = espi_mec5_recv_oob_api,
#endif
#ifdef CONFIG_ESPI_FLASH_CHANNEL
	.flash_read = espi_mec5_flash_read_api,
	.flash_write = espi_mec5_flash_write_api,
	.flash_erase = espi_mec5_flash_erase_api,
#endif
	.manage_callback = espi_mec5_manage_cb_api,
};

#define MEC5_ESPI_IRQ_CONNECT(inst, dev)                                                           \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, erst, irq),                                          \
		    DT_INST_IRQ_BY_NAME(inst, erst, priority), espi_mec5_ereset_isr,               \
		    DEVICE_DT_INST_GET(inst), 0);                                                  \
	irq_enable(DT_INST_IRQ_BY_NAME(inst, erst, irq));                                          \
	sys_write32(BIT(ESPI_GIRQ_ERST_POS), ESPI_GIRQ_ENSET_ADDR);

#ifdef CONFIG_ESPI_VWIRE_CHANNEL
#define MEC5_ESPI_VW_IRQ_CONNECT(dev) espi_mec5_vw_irq_connect(dev);
#else
#define MEC5_ESPI_VW_IRQ_CONNECT(dev)
#endif /* CONFIG_ESPI_VWIRE_CHANNEL */

#ifdef CONFIG_ESPI_PERIPHERAL_CHANNEL
#define MEC5_ESPI_PC_IRQ_CONNECT(dev) espi_mec5_pc_irq_connect(dev);
#else
#define MEC5_ESPI_PC_IRQ_CONNECT(dev)
#endif /* CONFIG_ESPI_PERIPHERAL_CHANNEL */

#ifdef CONFIG_ESPI_OOB_CHANNEL
#define MEC5_ESPI_OOB_IRQ_CONNECT(dev) espi_mec5_oob_irq_connect(dev)
#define MEC5_ESPI_OOB_DATA(inst) \
	static uint8_t espi_mec5_oob_rx_buf_##inst[CONFIG_ESPI_OOB_BUFFER_SIZE] __aligned(4)
#define MEC5_ESPI_OOB_DATA_INIT(inst) .oob_rxb = espi_mec5_oob_rx_buf_##inst,
#define MEC5_ESPI_OOB_DCFG(inst) .oob_rxb_size = sizeof(espi_mec5_oob_rx_buf_##inst),
#else
#define MEC5_ESPI_OOB_IRQ_CONNECT(dev)
#define MEC5_ESPI_OOB_DATA(inst)
#define MEC5_ESPI_OOB_DATA_INIT(inst)
#define MEC5_ESPI_OOB_DCFG(inst)
#endif /* CONFIG_ESPI_PERIPHERAL_CHANNEL */

#ifdef CONFIG_ESPI_FLASH_CHANNEL
#define MEC5_ESPI_FLASH_IRQ_CONNECT(dev) espi_mec5_fc_irq_connect(dev)
#else
#define MEC5_ESPI_FLASH_IRQ_CONNECT(dev)
#endif /* CONFIG_ESPI_PERIPHERAL_CHANNEL */

#define MEC5_ESPI_DEVICE(inst)                                                                     \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static void espi_mec5_irq_config_##inst(const struct device *dev)                          \
	{                                                                                          \
		MEC5_ESPI_IRQ_CONNECT(inst, dev);                                                  \
		MEC5_ESPI_VW_IRQ_CONNECT(dev);                                                     \
		MEC5_ESPI_PC_IRQ_CONNECT(dev);                                                     \
		MEC5_ESPI_OOB_IRQ_CONNECT(dev);                                                    \
		MEC5_ESPI_FLASH_IRQ_CONNECT(dev);                                                  \
	}                                                                                          \
	MEC5_ESPI_OOB_DATA(inst);                                                                  \
	struct espi_mec5_drv_data espi_mec5_drv_data_##inst = {                                    \
		MEC5_ESPI_OOB_DATA_INIT(inst)                                                      \
	};                                                                                         \
	struct espi_mec5_drv_cfg espi_mec5_dcfg_##inst = {                                         \
		.ioc_base = DT_INST_REG_ADDR_BY_NAME(inst, io),                                    \
		.memc_base = DT_INST_REG_ADDR_BY_NAME(inst, mem),                                  \
		.vwc_base = DT_INST_REG_ADDR_BY_NAME(inst, vw),                                    \
		.irq_config = espi_mec5_irq_config_##inst,                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		MEC5_ESPI_OOB_DCFG(inst)                                                           \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(inst, espi_mec5_pm_action);                                       \
	DEVICE_DT_INST_DEFINE(inst, &espi_mec5_driver_init, PM_DEVICE_DT_INST_GET(inst),           \
			      &espi_mec5_drv_data_##inst, &espi_mec5_dcfg_##inst, POST_KERNEL,     \
			      CONFIG_ESPI_INIT_PRIORITY, &espi_mec5_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MEC5_ESPI_DEVICE)
