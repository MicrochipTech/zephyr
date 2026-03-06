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
#include "../espi_utils.h"
#include "../espi_mchp_xec_ng.h"
#include "mchp_xec_espi_regs.h"

LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

/* Peripheral channel peripheral device table */
#if 0
microchip,xec-espi-ng-host-dev
espi_emi0: espi_emi0 {
	ldn = <0x10>;
	host-io-config-index = <0x68>;
	host-mem-config-index = <0x6c>;
	host-address = <0>;
	sirq-config-indexes = <0xb7 0xb8>;
	sirqs = <0xff 0xff>;
	sirq-names = "host_event", "ec_to_host";
	status = "disabled";
};
#endif

#define HD_NODE DT_PATH(mchp_xec_espi_ng_pc_host_dev)

struct xec_espi_ng_hd {
	uint32_t host_addr;
	uint8_t io_cfg_idx;
	uint8_t mem_cfg_idx;
	uint8_t ldn;
	uint8_t nsirqs;
	uint8_t sirq_cfg_idxs[2];
	uint8_t sirq_slots[2];
};

#define XEC_ESPI_NG_HD_M1(nid) { \
	.host_addr = DT_PROP_OR(nid, host_address, 0), \
	.io_cfg_idx = DT_PROP(nid, host_io_config_index), \
	.mem_cfg_idx = DT_PROP(nid, host_mem_config_index), \
	.ldn = (uint8_t)DT_PROP(nid, ldn), \
	.nsirqs = DT_PROP_LEN_OR(nid, sirqs, 0), \
	.sirq_cfg_idxs = {0, 0}, \
	.sirq_slots = {0, 0}, \
},

const struct xec_espi_ng_hd xec_espi_ng_hd_tbl[] = {
	DT_FOREACH_CHILD_STATUS_OKAY(HD_NODE, XEC_ESPI_NG_HD_M1)
};

const struct xec_espi_ng_hd *get_hd_by_ldn(uint8_t ldn)
{
	for (size_t i = 0; i < ARRAY_SIZE(xec_espi_ng_hd_tbl); i++) {
		const struct xec_espi_ng_hd *p = &xec_espi_ng_hd_tbl[i];

		if (p->ldn == ldn) {
			return p;
		}
	}

	return NULL;
}

/* Configure Host devices for operation after eSPI harware has released them from reset.
 * This routine is called on de-assertion of the eSPI PLTRST virtual wire.
 */
int xec_espi_ng_host_dev_config(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mm_reg_t iob = devcfg->base;
	mm_reg_t mb = devcfg->mbase;
	mm_reg_t ofs = 0;
	uint32_t val = 0;

	for (size_t i = 0; i < ARRAY_SIZE(xec_espi_ng_hd_tbl); i++) {
		const struct xec_espi_ng_hd *p = &xec_espi_ng_hd_tbl[i];

		if (p->host_addr == 0) {
			continue;
		}

		if (p->host_addr < UINT16_MAX) { /* configure I/O BAR? */
			ofs = XEC_ESPI_IOB_HAV_OFS(p->io_cfg_idx);
			val = XEC_ESPI_IOB_HAV_HADDR_SET(p->host_addr);
			val |= BIT(XEC_ESPI_IOB_HAV_VALID_POS);
			sys_write32(val, iob + ofs);
		} else { /* Address is Host memory */
			/* These registers are 16-bit aligned */
			ofs = XEC_ESPI_MEMB_HOST_OFS(p->mem_cfg_idx);
			/* bit[15:0] has valid bit
			 * bits[31:16] = Host address b[15:0]
			 * bits[47:32] = Host address b[31:16]
			 */
			sys_write16((uint16_t)p->host_addr, mb + ofs + 2u);
			sys_write16((uint16_t)(p->host_addr >> 16), mb + ofs + 4u);
			sys_set_bit(mb + ofs, XEC_ESPI_MC_BAR_CFG_VAL_POS);
		}

		/* Serial-IRQ */
		if (p->nsirqs == 0) {
			continue;
		}

		/* TODO */
	}

	return 0;
}

/* ---------------------------------------------- */
static bool pc_is_ready(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mem_addr_t iob = devcfg->base;

	if (soc_test_bit8(iob + XEC_ESPI_PC_RDY_OFS, XEC_ESPI_CHAN_RDY_POS) != 0) {
		return true;
	}

	return false;
}

int xec_espi_ng_pc_rd_api(const struct device *dev, struct espi_request_packet *req)
{
	if (pc_is_ready(dev) == false) {
		return -EIO;
	}

	return 0;
}

int xec_espi_ng_pc_wr_api(const struct device *dev, struct espi_request_packet *req)
{
	if (pc_is_ready(dev) == false) {
		return -EIO;
	}

	return 0;
}

/* TODO - data needs to change to another type or we must cast it (ugly and dangerous) */
int xec_espi_ng_pc_lpc_rd_api(const struct device *dev, enum lpc_peripheral_opcode op,
                              uint32_t *data)
{
	if (pc_is_ready(dev) == false) {
		return -EIO;
	}

	return 0;
}

int xec_espi_ng_pc_lpc_wr_api(const struct device *dev, enum lpc_peripheral_opcode op,
                              uint32_t *data)
{
	if (pc_is_ready(dev) == false) {
		return -EIO;
	}

	return 0;
}

/* Peripheral channel handler called from core driver ISR
 * 1. Channel enable change
 * 2. BM enable change
 * 3. PC bus error
 */
void xec_espi_ng_pc_handler(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	struct xec_espi_ng_data *data = dev->data;
	mem_addr_t iob = devcfg->base;
	uint32_t pc_status = sys_read32(iob + XEC_ESPI_PC_SR_OFS);
	struct espi_event ev = {
		.evt_type = ESPI_BUS_EVENT_CHANNEL_READY,
		.evt_details = ESPI_CHANNEL_PERIPHERAL,
		.evt_data = 0,
	};

	sys_write32(pc_status, iob + XEC_ESPI_PC_SR_OFS);
	soc_ecia_girq_status_clear(XEC_ESPI_GIRQ, XEC_ESPI_GIRQ_PC_POS);

	if ((pc_status & BIT(XEC_ESPI_PC_SR_CHEN_CHG_POS)) != 0) {
		if ((pc_status & BIT(XEC_ESPI_PC_SR_CHEN_STATE_POS)) != 0) { /* 0 -> 1 enable */
			ev.evt_data |= ESPI_PC_EVT_BUS_CHANNEL_READY;
			soc_set_bit8(iob + XEC_ESPI_PC_RDY_OFS, XEC_ESPI_CHAN_RDY_POS);
		}

		espi_send_callbacks(&data->cbs, dev, ev);
	}

	if ((pc_status & BIT(XEC_ESPI_PC_SR_BMEN_CHG_POS)) != 0) {
		if ((pc_status & BIT(XEC_ESPI_PC_SR_BMEN_STATE_POS)) != 0) {
			ev.evt_data |= ESPI_PC_EVT_BUS_MASTER_ENABLE;
		}
	}

	/* Our HW can signal an error on our internal AHB due to a misprogrammed BAR
	 * or other configuration error.
	 * TODO extend enum espi_pc_event adding bit[7] in a MCHP XEC specific header file.
	 */
	if ((pc_status & BIT(XEC_ESPI_PC_SR_ABERR_POS)) != 0) {
		ev.evt_data |= BIT(7);
	}

	espi_send_callbacks(&data->cbs, dev, ev);
}

#if defined(CONFIG_ESPI_PERIPHERAL_BC1_SUPPORT) || defined(CONFIG_ESPI_PERIPHERAL_BC2_SUPPORT)
void xec_espi_ng_bm_handler(const struct device *dev, uint8_t chan)
{
	struct xec_espi_ng_data *data = dev->data;
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mem_addr_t mb = devcfg->mbase;
	struct espi_event ev = {
		.evt_type = ESPI_BUS_PERIPHERAL_NOTIFICATION,
		.evt_details = ESPI_PERIPHERAL_BUS_CTRL1,
		.evt_data = ESPI_PERIPHERAL_NODATA,
	};
	uint32_t bmsts = sys_read32(mb + XEC_ESPI_MBC_SR_OFS);

	sys_write32(bmsts, mb + XEC_ESPI_MBC_SR_OFS);

	if (chan != 0) {
		ev.evt_details = ESPI_PERIPHERAL_BUS_CTRL2;
	}

	/* TODO */

	espi_send_callbacks(&data->cbs, dev, ev);
}
#endif

#ifdef CONFIG_ESPI_PERIPHERAL_BC1_SUPPORT
/* PC Bus Master 1 handler called from core driver ISR
 * 1. BM1 transfer done
 */
void xec_espi_ng_bm1_handler(const struct device *dev)
{
	xec_espi_ng_bm_handler(dev, 0);
}
#endif

#ifdef CONFIG_ESPI_PERIPHERAL_BC2_SUPPORT
/* PC Bus Master 2 handler called from core driver ISR
 * 1. BM2 transfer done
 */
void xec_espi_ng_bm2_handler(const struct device *dev)
{
	xec_espi_ng_bm_handler(dev, 1);
}
#endif

#ifdef CONFIG_ESPI_PERIPHERAL_LTR_SUPPORT
/* PC LTR handler called from core driver ISR
 * 1. LTR transfer done
 * 2. LTR overrun error (start transfer while busy)
 * 3. LTR disabled by Host
 */
void xec_espi_ng_ltr_handler(const struct device *dev)
{
	struct xec_espi_ng_data *data = dev->data;
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mem_addr_t iob = devcfg->base;
	struct espi_event ev = {
		.evt_type = ESPI_BUS_PERIPHERAL_NOTIFICATION,
		.evt_details = ESPI_PERIPHERAL_LTR,
		.evt_data = ESPI_PERIPHERAL_NODATA,
	};
	uint32_t ltr_sts = sys_read32(iob + XEC_ESPI_PC_LTR_SR_OFS);

	sys_write32(ltr_sts, iob + XEC_ESPI_PC_LTR_SR_OFS);

	/* TODO */

	espi_send_callbacks(&data->cbs, dev, ev);
}
#endif
