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

/* PC Bus Master 1 handler called from core driver ISR
 * 1. BM1 transfer done
 */
void xec_espi_ng_bm1_handler(const struct device *dev)
{
	/* TODO */
}

/* PC Bus Master 2 handler called from core driver ISR
 * 1. BM2 transfer done
 */
void xec_espi_ng_bm2_handler(const struct device *dev)
{
	/* TODO */
}

/* PC LTR handler called from core driver ISR
 * 1. LTR transfer done
 * 2. LTR overrun error (start transfer while busy)
 * 3. LTR disabled by Host
 */
void xec_espi_ng_ltr_handler(const struct device *dev)
{
	/* TODO */
}
