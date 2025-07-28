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
#include "mchp/espi_mchp_regs.h"

// espi_alert_od_capable

/* If SoC was configured for Boot-ROM to load Zephyr application from
 * eSPI CAF then the eSPI link is up and Host has enabled VW channel.
 */
static bool is_espi_bootrom_config(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iob = (mm_reg_t)drvcfg->ioc_base;

	if ((sys_test_bit8(iob + MEC_ESPI_ACTV_OFS, MEC_ESPI_ACTV_EN_POS) != 0) &&
	    (sys_test_bit8(iob + MEC_ESPI_VW_RDY_OFS, MEC_ESPI_CHAN_RDY_POS) != 0)) {
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
	struct espi_event evt = {ESPI_BUS_RESET, 0, 0};
	uint8_t erst_sts = 0, n_erst_state = 0;

	erst_sts = sys_read8(iob + MEC_ESPI_RESET_SR_OFS);
	sys_write8(iob + MEC_ESPI_RESET_SR_OFS, erst_sts);
	n_erst_state = (erst_sts >> MEC_ESPI_RESET_SR_STATE_POS) & BIT(0);
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
/* eSPI supports advertising these groups of I/O modes to the Host.
 * single I/O only,
 * single or dual,
 * single or quad,
 * single, dual, or quad.
 */
struct espi_iom_enc {
	uint8_t iom;
	uint8_t iom_enc;
};

const struct espi_iom_enc espi_iom_enc_tbl[] = {
	{
		.iom = ESPI_IO_MODE_SINGLE_LINE,
		.iom_enc = MEC_ESPI_CAP1_IOM_S_VAL
	},
	{
		.iom = ESPI_IO_MODE_SINGLE_LINE | ESPI_IO_MODE_DUAL_LINES,
		.iom_enc = MEC_ESPI_CAP1_IOM_SD_VAL,
	},
	{
		.iom = ESPI_IO_MODE_SINGLE_LINE | ESPI_IO_MODE_QUAD_LINES,
		.iom_enc = MEC_ESPI_CAP1_IOM_SQ_VAL,
	},
	{
		.iom = ESPI_IO_MODE_SINGLE_LINE | ESPI_IO_MODE_DUAL_LINES | ESPI_IO_MODE_QUAD_LINES,
		.iom_enc = MEC_ESPI_CAP1_IOM_SDQ_VAL,
	},
};

static uint8_t espi_mec5_encode_io_mode(enum espi_io_mode mode)
{
	uint8_t iom_enc = UINT8_MAX;

	for (size_t n = 0; n < ARRAY_SIZE(espi_iom_enc_tbl); n++) {
		if (mode == espi_iom_enc_tbl[n].iom) {
			iom_enc = espi_iom_enc_tbl[n].iom_enc;
			iom_enc <<= MEC_ESPI_CAP1_IOM_POS;
			break;
		}
	}

	return iom_enc;
}

static uint8_t espi_mec5_encode_channel_support(enum espi_channel channels)
{
	uint32_t chan_enc = 0;

	if ((channels & ESPI_CHANNEL_PERIPHERAL) != 0) {
		chan_enc |= BIT(MEC_ESPI_CAP0_PC_SUPP_POS);
	}

	if ((channels & ESPI_CHANNEL_VWIRE) != 0) {
		chan_enc |= BIT(MEC_ESPI_CAP0_VW_SUPP_POS);
	}

	if ((channels & ESPI_CHANNEL_OOB) != 0) {
		chan_enc |= BIT(MEC_ESPI_CAP0_OOB_SUPP_POS);
	}

	if ((channels & ESPI_CHANNEL_FLASH) != 0) {
		chan_enc |= BIT(MEC_ESPI_CAP0_FC_SUPP_POS);
	}

	return chan_enc;
}

static uint8_t espi_mec5_encode_max_freq(uint8_t max_freq_mhz)
{
	uint32_t encf = 0;

	switch (max_freq_mhz) {
	case 25u:
		encf = MEC_ESPI_CAP1_MAX_FREQ_25M;
		break;
	case 33u:
		encf = MEC_ESPI_CAP1_MAX_FREQ_33M;
		break;
	case 50u:
		encf = MEC_ESPI_CAP1_MAX_FREQ_50M;
		break;
	case 66u:
		encf = MEC_ESPI_CAP1_MAX_FREQ_66M;
		break;
	default:
		encf = MEC_ESPI_CAP1_MAX_FREQ_20M;
		break;
	}

	return encf;
}

static int espi_mec5_config_api(const struct device *dev, struct espi_cfg *cfg,
				const void *vend_ext)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iob = (mm_reg_t)drvcfg->ioc_base;
	uint8_t cap0 = 0, cap1 = 0;

	if (cfg == NULL) {
		return -EINVAL;
	}

	cap0 = espi_mec5_encode_channel_support(cfg->channel_caps);
	sys_write8(cap0, iob + MEC_ESPI_CAP0_OFS);

	cap1 = espi_mec5_encode_io_mode(cfg->io_caps);
	if (cap1 == UINT8_MAX) {
		return -EINVAL;
	}

	cap1 |= espi_mec5_encode_max_freq(cfg->max_freq);

	if ((drvcfg->flags & MEC5_ESPI_DCFG_FLAG_OD_CAP) != 0) {
		cap1 |= MEC_BIT(MEC_ESPI_CAP1_ALERT_OD_CAP_POS);
	}

	sys_write8(cap1, iob + MEC_ESPI_CAP1_OFS);

#if defined(CONFIG_ESPI_PERIPHERAL_CHANNEL)
	if ((cfg->channel_caps & ESPI_CHANNEL_PERIPHERAL) != 0) {
		int rc = espi_mec5_pc_config(dev, vend_ext);

		if (rc != 0) {
			return rc;
		}
	}
#endif
	/* activate eSPI device */
	sys_set_bit8(iob + MEC_ESPI_ACTV_OFS, MEC_ESPI_ACTV_EN_POS);

	return 0;
}

static bool espi_mec5_get_chan_status_api(const struct device *dev, enum espi_channel ch)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iob = (mm_reg_t)drvcfg->ioc_base;
	uint32_t ofs = 0;

	if (ch == ESPI_CHANNEL_PERIPHERAL) {
		ofs = MEC_ESPI_PC_RDY_OFS;
	} else if (ch == ESPI_CHANNEL_VWIRE) {
		ofs = MEC_ESPI_VW_RDY_OFS;
	} else if (ch == ESPI_CHANNEL_OOB) {
		ofs = MEC_ESPI_OOB_RDY_OFS;
	} else if (ch == ESPI_CHANNEL_FLASH) {
		ofs = MEC_ESPI_FC_RDY_OFS;
	} else {
		return false;
	}

	if (sys_test_bit8(iob + ofs, MEC_ESPI_CHAN_RDY_POS)) {
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
	sys_write32(BIT(MEC_ESPI_GIRQ_ERST_POS), MEC_ESPI_GIRQ_ENSET_ADDR);

#ifdef CONFIG_ESPI_VWIRE_CHANNEL
#define MEC5_ESPI_VW_IRQ_CONNECT(dev) espi_mec5_vw_irq_connect(dev);
#else
#define MEC5_ESPI_VW_IRQ_CONNECT(dev)
#endif /* CONFIG_ESPI_VWIRE_CHANNEL */

#ifdef CONFIG_ESPI_PERIPHERAL_CHANNEL
#define MEC5_ESPI_PC_IRQ_CONNECT(dev) espi_mec5_pc_irq_connect(dev);
#define MEC5_ESPI_PC_DCFG(inst)                                                                    \
	.mem_bar_msw = (uint16_t)DT_INST_PROP_OR(inst, host_memmap_addr_high, 0),                  \
	.sram_bar_msw = (uint16_t)DT_INST_PROP_OR(inst, sram_bar_addr_high, 0),
#else
#define MEC5_ESPI_PC_IRQ_CONNECT(dev)
#define MEC5_ESPI_PC_DCFG(inst)
#endif /* CONFIG_ESPI_PERIPHERAL_CHANNEL */

#ifdef CONFIG_ESPI_OOB_CHANNEL
#define MEC5_ESPI_OOB_IRQ_CONNECT(dev) espi_mec5_oob_irq_connect(dev)
#define MEC5_ESPI_OOB_DATA(inst)                                                                   \
	static uint8_t espi_mec5_oob_rx_buf_##inst[CONFIG_ESPI_OOB_BUFFER_SIZE] __aligned(4)
#define MEC5_ESPI_OOB_DATA_INIT(inst) \
	.oob_tx_sync = Z_SEM_INITIALIZER(espi_mec5_drv_data_##inst.oob_tx_sync, 0, 1), \
	.oob_rx_sync = Z_SEM_INITIALIZER(espi_mec5_drv_data_##inst.oob_tx_sync, 0, 1), \
	.oob_rxb = espi_mec5_oob_rx_buf_##inst,

#define MEC5_ESPI_OOB_DCFG(inst)      .oob_rxb_size = sizeof(espi_mec5_oob_rx_buf_##inst),
#else
#define MEC5_ESPI_OOB_IRQ_CONNECT(dev)
#define MEC5_ESPI_OOB_DATA(inst)
#define MEC5_ESPI_OOB_DATA_INIT(inst)
#define MEC5_ESPI_OOB_DCFG(inst)
#endif /* CONFIG_ESPI_PERIPHERAL_CHANNEL */

#ifdef CONFIG_ESPI_FLASH_CHANNEL
#define MEC5_ESPI_FLASH_IRQ_CONNECT(dev) espi_mec5_fc_irq_connect(dev)
#define MEC5_ESPI_FLASH_DATA_INIT(inst) \
	.fc_sync = Z_SEM_INITIALIZER(espi_mec5_drv_data_##inst.fc_sync, 0, 1),
#else
#define MEC5_ESPI_FLASH_IRQ_CONNECT(dev)
#define MEC5_ESPI_FLASH_DATA_INIT(int)
#endif /* CONFIG_ESPI_PERIPHERAL_CHANNEL */

#define MEC5_ESPI_FLAGS_DCFG(inst) \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, espi_alert_od_capable), (0x1u), (0))

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
		MEC5_ESPI_FLASH_DATA_INIT(inst)                                                    \
	};                                                                                         \
	struct espi_mec5_drv_cfg espi_mec5_dcfg_##inst = {                                         \
		.ioc_base = DT_INST_REG_ADDR_BY_NAME(inst, io),                                    \
		.memc_base = DT_INST_REG_ADDR_BY_NAME(inst, mem),                                  \
		.vwc_base = DT_INST_REG_ADDR_BY_NAME(inst, vw),                                    \
		.irq_config = espi_mec5_irq_config_##inst,                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.flags = MEC5_ESPI_FLAGS_DCFG(inst),                                               \
		MEC5_ESPI_PC_DCFG(inst)                                                            \
		MEC5_ESPI_OOB_DCFG(inst)};                                                         \
	PM_DEVICE_DT_INST_DEFINE(inst, espi_mec5_pm_action);                                       \
	DEVICE_DT_INST_DEFINE(inst, &espi_mec5_driver_init, PM_DEVICE_DT_INST_GET(inst),           \
			      &espi_mec5_drv_data_##inst, &espi_mec5_dcfg_##inst, POST_KERNEL,     \
			      CONFIG_ESPI_INIT_PRIORITY, &espi_mec5_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MEC5_ESPI_DEVICE)
