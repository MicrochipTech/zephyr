/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_espi_ng

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/espi/mchp_xec_espi_ng.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

/* local */
#include "espi_utils.h"
#include "espi_mchp_xec_ng.h"
#include "mchp/mchp_xec_espi_regs.h"

LOG_MODULE_REGISTER(espi, CONFIG_ESPI_LOG_LEVEL);

static bool is_espi_bootrom_config(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mem_addr_t iob = devcfg->base;

	if ((soc_test_bit8(iob + XEC_ESPI_ACTV_OFS, XEC_ESPI_ACTV_EN_POS) != 0) &&
	    (soc_test_bit8(iob + XEC_ESPI_VW_RDY_OFS, XEC_ESPI_CHAN_RDY_POS) != 0)) {
		return true;
	}

	return false;
}

static uint32_t get_max_freq_encoded(uint8_t max_freq_mhz)
{
	uint32_t enc_freq = 0;

	if (max_freq_mhz > 50u) {
		enc_freq = XEC_ESPI_CAP1_MAX_FREQ_66M;
	} else if (max_freq_mhz > 33u) {
		enc_freq = XEC_ESPI_CAP1_MAX_FREQ_50M;
	} else if (max_freq_mhz > 20u) {
		enc_freq = XEC_ESPI_CAP1_MAX_FREQ_33M;
	} else {
		enc_freq = XEC_ESPI_CAP1_MAX_FREQ_20M;
	}

	return enc_freq;
}

static uint32_t get_channels_supported(enum espi_channel chan_caps)
{
	uint32_t cap0 = 0;

#ifdef CONFIG_ESPI_PERIPHERAL_CHANNEL
	cap0 |= BIT(XEC_ESPI_CAP0_PC_SUPP_POS);
#endif
#ifdef CONFIG_ESPI_VWIRE_CHANNEL
	cap0 |= BIT(XEC_ESPI_CAP0_VW_SUPP_POS);
#endif
#ifdef CONFIG_ESPI_OOB_BUFFER_SIZE
	cap0 |= BIT(XEC_ESPI_CAP0_OOB_SUPP_POS);
#endif
#ifdef CONFIG_ESPI_FLASH_CHANNEL
	cap0 |= BIT(XEC_ESPI_CAP0_FC_SUPP_POS);
#endif
	return cap0;
}

static uint32_t io_mode_encoded(enum espi_io_mode iom)
{
	uint32_t iom_enc = 0;

	switch ((uint32_t)iom) {
	case ESPI_IO_MODE_SINGLE_LINE:
		iom_enc = XEC_ESPI_CAP1_IOM_S;
		break;
	case (ESPI_IO_MODE_SINGLE_LINE | ESPI_IO_MODE_DUAL_LINES):
		iom_enc = XEC_ESPI_CAP1_IOM_SD;
		break;
	case (ESPI_IO_MODE_SINGLE_LINE | ESPI_IO_MODE_QUAD_LINES):
		iom_enc = XEC_ESPI_CAP1_IOM_SQ;
		break;
	case (ESPI_IO_MODE_SINGLE_LINE | ESPI_IO_MODE_DUAL_LINES | ESPI_IO_MODE_QUAD_LINES):
		iom_enc = XEC_ESPI_CAP1_IOM_SDQ;
		break;
	default:
		iom_enc = XEC_ESPI_CAP1_IOM_S;
	}

	return iom_enc;
}

/* ---------------- Public API ---------------- */
static int xec_espi_ng_config_api(const struct device *dev, struct espi_cfg *cfg)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mem_addr_t iob = devcfg->base;
	uint32_t cap = 0;
	int rc = 0;

	if (cfg == NULL) {
		return -EINVAL;
	}

	soc_clear_bit8(iob + XEC_ESPI_ACTV_OFS, XEC_ESPI_ACTV_EN_POS);

	cap = get_channels_supported(cfg->channel_caps);
	sys_write8(cap, iob + XEC_ESPI_CAP0_OFS);

	cap = get_max_freq_encoded(cfg->max_freq);
	cap |= io_mode_encoded(cfg->io_caps);
	/* TODO set alert pin is opendrain capable from DT */

	sys_write8(cap, iob + XEC_ESPI_CAP1_OFS);

	rc = xec_espi_ng_vw_init1(dev);
	if (rc != 0) {
		LOG_ERR("H2T VWire IRQ detect failed(%d)", rc);
		return rc;
	}

	soc_set_bit8(iob + XEC_ESPI_ACTV_OFS, XEC_ESPI_ACTV_EN_POS);

	return 0;
}

static bool xec_espi_ng_get_chan_status_api(const struct device *dev, enum espi_channel ch)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mem_addr_t iob = devcfg->base;
	uint32_t ofs = 0;

	switch (ch) {
	case ESPI_CHANNEL_PERIPHERAL:
		ofs = XEC_ESPI_PC_RDY_OFS;
		break;
	case ESPI_CHANNEL_VWIRE:
		ofs = XEC_ESPI_VW_RDY_OFS;
		break;
	case ESPI_CHANNEL_OOB:
		ofs = XEC_ESPI_OOB_RDY_OFS;
		break;
	case ESPI_CHANNEL_FLASH:
		ofs = XEC_ESPI_FC_RDY_OFS;
		break;
	default:
		return false;
	}

	if (soc_test_bit8(iob + ofs, XEC_ESPI_CHAN_RDY_POS) != 0) {
		return true;
	}

	return false;
}

static int xec_espi_ng_mcb_api(const struct device *dev, struct espi_callback *cb, bool set)
{
	struct xec_espi_ng_data *data = dev->data;

	if (cb == NULL) {
		return -EINVAL;
	}

	return espi_manage_callback(&data->cbs, cb, set);
}

#ifdef CONFIG_PM_DEVICE
static int xec_espi_ng_pm_action(const struct device *dev, enum pm_device_action action)
{
	return 0;
}
#endif

/* -------- interrupt handlers -------- */

/* eSPI Reset from Host */
static void xec_espi_ng_erst_irq(const struct device *dev)
{
	/* TODO */
}

#ifdef CONFIG_ESPI_PERIPHERAL_CHANNEL
static void xec_espi_ng_pc_irq(const struct device *dev)
{
	xec_espi_ng_pc_handler(dev);
}

static void xec_espi_ng_bm1_irq(const struct device *dev)
{
	xec_espi_ng_bm1_handler(dev);
}

static void xec_espi_ng_bm2_irq(const struct device *dev)
{
	xec_espi_ng_bm2_handler(dev);
}

static void xec_espi_ng_ltr_irq(const struct device *dev)
{
	xec_espi_ng_ltr_handler(dev);
}
#endif

#ifdef CONFIG_ESPI_VWIRE_CHANNEL
static void xec_espi_ng_vw_chen_irq(const struct device *dev)
{
	xec_espi_ng_vw_chen_handler(dev);
}

/* Controller-to-Target VWire group interrupts are always aggregated */
static void xec_espi_ng_vwct_0_6_irq(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	struct xec_espi_ng_data *data = dev->data;

	/* must disable since we will processing outside of the ISR */
	irq_disable(devcfg->irqn_vwb0);

	k_work_submit(&data->kwq_vwb0);
}

static void xec_espi_ng_vwct_7_10_irq(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	struct xec_espi_ng_data *data = dev->data;

	irq_disable(devcfg->irqn_vwb1);

	k_work_submit(&data->kwq_vwb1);
}
#endif

#ifdef CONFIG_ESPI_OOB_CHANNEL
static void xec_espi_ng_oob_dn_isr(const struct device *dev)
{
	xec_espi_ng_oob_dn_handler(dev);
}

static void xec_espi_ng_oob_up_isr(const struct device *dev)
{
	xec_espi_ng_oob_up_handler(dev);
}
#endif

#ifdef CONFIG_ESPI_FLASH_CHANNEL
static void xec_espi_ng_fc_irq(const struct device *dev)
{
	xec_espi_ng_fc_handler(dev);
}
#endif

static int xec_espi_ng_driver_init(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	struct xec_espi_ng_data *data = dev->data;
	int rc = 0;

	sys_slist_init(&data->cbs);

#ifdef CONFIG_ESPI_VWIRE_CHANNEL
	k_work_init(&data->kwq_vwb0, &xec_espi_ng_vw_bank0_kworker);
	k_work_init(&data->kwq_vwb1, &xec_espi_ng_vw_bank1_kworker);
#endif
#ifdef CONFIG_ESPI_OOB_CHANNEL
	k_sem_init(&data->oob_lock, 1u, 1u);
	k_sem_init(&data->oob_rx_sync, 1u, 1u);
	k_sem_init(&data->oob_tx_sync, 1u, 1u);
#endif
#ifdef CONFIG_ESPI_FC_CHANNEL
	k_sem_init(&data->fc_lock, 1u, 1u);
	k_sem_init(&data->fc_sync, 1u, 1u);
#endif
	if (is_espi_bootrom_config(dev) == true) {
		LOG_INF("Boot-ROM has enabled eSPI");
	}

	rc = pinctrl_apply_state(devcfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("PINCTRL failed (%d)", rc);
		return rc;
	}

	if (devcfg->irq_connect != NULL) {
		devcfg->irq_connect();
	}

	return 0;
}

static DEVICE_API(espi, xec_espi_ng_driver_api) = {
	.config = xec_espi_ng_config_api,
	.get_channel_status = xec_espi_ng_get_chan_status_api,
#ifdef CONFIG_ESPI_PERIPHERAL_CHANNEL
	.read_request = xec_espi_ng_pc_rd_api,
	.write_request = xec_espi_ng_pc_wr_api,
	.read_lpc_request = xec_espi_ng_pc_lpc_rd_api,
	.write_lpc_request = xec_espi_ng_pc_lpc_wr_api,
#endif
#ifdef CONFIG_ESPI_VWIRE_CHANNEL
	.send_vwire = xec_espi_ng_vw_send_api,
	.receive_vwire = xec_espi_ng_vw_recv_api,
#endif
#ifdef CONFIG_ESPI_OOB_CHANNEL
	.send_oob = xec_espi_ng_oob_send_api,
	.receive_oob = xec_espi_ng_oob_recv_api,
#endif
#ifdef CONFIG_ESPI_FLASH_CHANNEL
	.flash_read = xec_espi_ng_fc_rd_api,
	.flash_write = xec_espi_ng_fc_wr_api,
	.flash_erase = xec_espi_ng_fc_er_api,
#endif
	.manage_callback = xec_espi_ng_mcb_api,
};

#ifdef CONFIG_ESPI_PERIPHERAL_CHANNEL
#define XEC_ESPI_NG_PC_IRQ_CONNECT(i) \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, pc, irq), \
		    DT_INST_IRQ_BY_NAME(i, pc, priority), \
		    xec_espi_ng_pc_irq, \
		    DEVICE_DT_INST_GET(i), 0); \
	irq_enable(DT_INST_IRQ_BY_NAME(i, pc, irq)); \
	soc_ecia_girq_ctrl(XEC_ESPI_GIRQ, XEC_ESPI_GIRQ_PC_POS, 1u); \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, bm1, irq), \
		    DT_INST_IRQ_BY_NAME(i, bm1, priority), \
		    xec_espi_ng_bm1_irq, \
		    DEVICE_DT_INST_GET(i), 0); \
	irq_enable(DT_INST_IRQ_BY_NAME(i, bm1, irq)); \
	soc_ecia_girq_ctrl(XEC_ESPI_GIRQ, XEC_ESPI_GIRQ_BM1_POS, 1u); \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, bm2, irq), \
		    DT_INST_IRQ_BY_NAME(i, bm2, priority), \
		    xec_espi_ng_bm2_irq, \
		    DEVICE_DT_INST_GET(i), 0); \
	irq_enable(DT_INST_IRQ_BY_NAME(i, bm2, irq)); \
	soc_ecia_girq_ctrl(XEC_ESPI_GIRQ, XEC_ESPI_GIRQ_BM2_POS, 1u); \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, ltr, irq), \
		    DT_INST_IRQ_BY_NAME(i, ltr, priority), \
		    xec_espi_ng_ltr_irq, \
		    DEVICE_DT_INST_GET(i), 0); \
	irq_enable(DT_INST_IRQ_BY_NAME(i, ltr, irq)); \
	soc_ecia_girq_ctrl(XEC_ESPI_GIRQ, XEC_ESPI_GIRQ_LTR_POS, 1u);
#else
#define XEC_ESPI_NG_PC_IRQ_CONNECT(i)
#endif

#ifdef CONFIG_ESPI_VWIRE_CHANNEL
#define XEC_ESPI_NG_VW_IRQ_CONNECT(i) \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, vw_chan_en, irq), \
		    DT_INST_IRQ_BY_NAME(i, vw_chan_en, priority), \
		    xec_espi_ng_vw_chen_irq, \
		    DEVICE_DT_INST_GET(i), 0); \
	irq_enable(DT_INST_IRQ_BY_NAME(i, vw_chan_en, irq)); \
	soc_ecia_girq_ctrl(XEC_ESPI_GIRQ, XEC_ESPI_GIRQ_VW_CHEN_POS, 1u); \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, vwct_0_6, irq), \
		    DT_INST_IRQ_BY_NAME(i, vwct_0_6, priority), \
		    xec_espi_ng_vwct_0_6_irq, \
		    DEVICE_DT_INST_GET(i), 0); \
	irq_enable(DT_INST_IRQ_BY_NAME(i, vwct_0_6, irq)); \
	soc_ecia_girq_ctrl_bm(XEC_ESPI_GIRQ_VW_BANK0, XEC_ESPI_GIRQ_VW_BANK0_MSK, 1u); \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, vwct_7_10, irq), \
		    DT_INST_IRQ_BY_NAME(i, vwct_7_10, priority), \
		    xec_espi_ng_vwct_7_10_irq, \
		    DEVICE_DT_INST_GET(i), 0); \
	irq_enable(DT_INST_IRQ_BY_NAME(i, vwct_7_10, irq)); \
	soc_ecia_girq_ctrl_bm(XEC_ESPI_GIRQ_VW_BANK1, XEC_ESPI_GIRQ_VW_BANK1_MSK, 1u)
#else
#define XEC_ESPI_NG_VW_IRQ_CONNECT(i)
#endif

#ifdef CONFIG_ESPI_OOB_CHANNEL
#define XEC_ESPI_NG_OOB_IRQ_CONNECT(i) \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, oob_dn, irq), \
		    DT_INST_IRQ_BY_NAME(i, oob_dn, priority), \
		    xec_espi_ng_oob_dn_isr, \
		    DEVICE_DT_INST_GET(i), 0); \
	irq_enable(DT_INST_IRQ_BY_NAME(i, oob_dn, irq)); \
	soc_ecia_girq_ctrl(XEC_ESPI_GIRQ, XEC_ESPI_GIRQ_OOB_DN_POS, 1u); \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, oob_up, irq), \
		    DT_INST_IRQ_BY_NAME(i, oob_up, priority), \
		    xec_espi_ng_oob_up_isr, \
		    DEVICE_DT_INST_GET(i), 0); \
	irq_enable(DT_INST_IRQ_BY_NAME(i, oob_up, irq)); \
	soc_ecia_girq_ctrl(XEC_ESPI_GIRQ, XEC_ESPI_GIRQ_OOB_UP_POS, 1u)
#else
#define XEC_ESPI_NG_OOB_IRQ_CONNECT(i)
#endif

#ifdef CONFIG_ESPI_FLASH_CHANNEL
#define XEC_ESPI_NG_FC_IRQ_CONNECT(i) \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, fc, irq), \
		    DT_INST_IRQ_BY_NAME(i, fc, priority), \
		    xec_espi_ng_fc_irq, \
		    DEVICE_DT_INST_GET(i), 0); \
	irq_enable(DT_INST_IRQ_BY_NAME(i, fc, irq)); \
	soc_ecia_girq_ctrl(XEC_ESPI_GIRQ, XEC_ESPI_GIRQ_FC_POS, 1u)
#else
#define XEC_ESPI_NG_FC_IRQ_CONNECT(i)
#endif

#define XEC_ESPI_NG_IRQ_CONNECT(i) \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, erst, irq), \
		    DT_INST_IRQ_BY_NAME(i, erst, priority), \
		    xec_espi_ng_erst_irq, \
		    DEVICE_DT_INST_GET(i), 0); \
	irq_enable(DT_INST_IRQ_BY_NAME(i, erst, irq)); \
	soc_ecia_girq_ctrl(XEC_ESPI_GIRQ, XEC_ESPI_GIRQ_ERST_POS, 1u)

#define XEC_ESPI_NG_DEVICE(i) \
	PINCTRL_DT_INST_DEFINE(i); \
	PM_DEVICE_DT_INST_DEFINE(i, xec_espi_ng_pm_action); \
	static struct xec_espi_ng_data xec_espi_ng_data##i; \
	void xec_espi_ng_irq_connect##i(void) \
	{ \
		XEC_ESPI_NG_IRQ_CONNECT(i); \
		XEC_ESPI_NG_PC_IRQ_CONNECT(i); \
		XEC_ESPI_NG_VW_IRQ_CONNECT(i); \
		XEC_ESPI_NG_OOB_IRQ_CONNECT(i); \
		XEC_ESPI_NG_FC_IRQ_CONNECT(i); \
	} \
	static const struct xec_espi_ng_drvcfg xec_espi_ng_dcfg##i = { \
		.base = (mem_addr_t)DT_INST_REG_ADDR(i), \
		.mbase = (mem_addr_t)DT_INST_REG_ADDR_BY_IDX(i, 1), \
		.vwbase = (mem_addr_t)DT_INST_REG_ADDR_BY_IDX(i, 2), \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i), \
		.irqn_vwb0 = DT_INST_IRQ_BY_NAME(i, vwct_0_6, irq), \
		.irqn_vwb1 = DT_INST_IRQ_BY_NAME(i, vwct_7_10, irq), \
	}; \
	DEVICE_DT_INST_DEFINE(i, &xec_espi_ng_driver_init, PM_DEVICE_DT_INST_GET(i), \
			&xec_espi_ng_data##i, &xec_espi_ng_dcfg##i, \
			POST_KERNEL, CONFIG_ESPI_INIT_PRIORITY, \
			&xec_espi_ng_driver_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_ESPI_NG_DEVICE)
