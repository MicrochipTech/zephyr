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

int xec_espi_ng_oob_send_api(const struct device *dev, struct espi_oob_packet *pkt)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	struct xec_espi_ng_data *data = dev->data;

	if (pkt == NULL) {
		return -EINVAL;
	}

	k_sem_take(&data->oob_lock, K_FOREVER);

	/* clear tx buffer before returning */
	memset(devcfg->oob_txb, 0x55, sizeof(devcfg->oob_txb_sz));

	k_sem_give(&data->oob_lock);

	return 0;
}

int xec_espi_ng_oob_recv_api(const struct device *dev, struct espi_oob_packet *pkt)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	struct xec_espi_ng_data *data = dev->data;

	if (pkt == NULL) {
		return -EINVAL;
	}

	k_sem_take(&data->oob_lock, K_FOREVER);

	memset(devcfg->oob_rxb, 0x55, sizeof(devcfg->oob_rxb_sz));

	k_sem_give(&data->oob_lock);

	return 0;
}

/* OOB received packet from eSPI Host */
void xec_espi_ng_oob_rx_handler(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	struct xec_espi_ng_data *data = dev->data;
	mem_addr_t iob = devcfg->base;
	uint32_t oob_status = sys_read32(iob + XEC_ESPI_OOB_RX_SR_OFS);
	uint32_t rxlen = sys_read32(iob + XEC_ESPI_OOB_RXL_OFS);

	sys_write32(oob_status, iob + XEC_ESPI_OOB_RX_SR_OFS);
	soc_ecia_girq_status_clear(XEC_ESPI_GIRQ, XEC_ESPI_GIRQ_OOB_RX_POS);

	data->oob_rx_status = oob_status;

	if ((oob_status & BIT(XEC_ESPI_OOB_RX_SR_DONE_POS)) != 0) {
		data->oob_rx_len = XEC_ESPI_OOB_RXL_MLEN_GET(rxlen);
		k_sem_give(&data->oob_rx_sync);
	}
}

/* OOB transmitted packet to eSPI Host. TX also handles OOB channel
 * enable change. If channel is enabled by Host we must configure
 * OOB TX and RX buffers and interrupts and then set OOB Ready.
 */
void xec_espi_ng_oob_tx_handler(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	struct xec_espi_ng_data *data = dev->data;
	mem_addr_t iob = devcfg->base;
	uint32_t oob_status = sys_read32(iob + XEC_ESPI_OOB_TX_SR_OFS);
	uint32_t val = 0;
	struct espi_event ev = {
		.evt_type = ESPI_BUS_EVENT_CHANNEL_READY,
		.evt_details = ESPI_CHANNEL_OOB,
		.evt_data = 0,
	};

	sys_write32(oob_status, iob + XEC_ESPI_OOB_TX_SR_OFS);
	soc_ecia_girq_status_clear(XEC_ESPI_GIRQ, XEC_ESPI_GIRQ_OOB_TX_POS);

	data->oob_tx_status = oob_status;

	if ((oob_status & BIT(XEC_ESPI_OOB_TX_SR_CENC_POS)) != 0) {
		if ((oob_status & BIT(XEC_ESPI_OOB_TX_SR_CHEN_POS)) != 0) { /* 0->1 enable */
			ev.evt_data |= ESPI_PC_EVT_BUS_CHANNEL_READY;

			/* program OOB TX and RX buffer address and length registers */
			sys_write32((uint32_t)devcfg->oob_rxb, iob + XEC_ESPI_OOB_RX_BA_LSW_OFS);
			sys_write32((uint32_t)devcfg->oob_txb, iob + XEC_ESPI_OOB_TX_BA_LSW_OFS);

			val = XEC_ESPI_OOB_RXL_BLEN_SET((uint32_t)devcfg->oob_rxb_sz);
			soc_mmcr_mask_set(iob + XEC_ESPI_OOB_RXL_OFS, val,
			                  XEC_ESPI_OOB_RXL_BLEN_MSK);

			val = BIT(XEC_ESPI_OOB_TX_IER_CENC_POS) | BIT(XEC_ESPI_OOB_TX_IER_DONE_POS);
			sys_write32(val, iob + XEC_ESPI_OOB_TX_IER_OFS);
			sys_write32(BIT(XEC_ESPI_OOB_RX_IER_DONE_POS),
			            iob + XEC_ESPI_OOB_RX_IER_OFS);

			/* Set OOB RX Available. Our HW can accept OOB packets from host  */
			sys_set_bit(iob + XEC_ESPI_OOB_RX_CR_OFS, XEC_ESPI_OOB_RX_CR_SRA_POS);

			/* Set OOB Ready letting Host know OOB channel is operational */
			soc_set_bit8(iob + XEC_ESPI_OOB_RDY_OFS, BIT(XEC_ESPI_CHAN_RDY_POS));

			espi_send_callbacks(&data->cbs, dev, ev);
		}
	}

	if ((oob_status & BIT(XEC_ESPI_OOB_TX_SR_DONE_POS)) != 0) {
		k_sem_give(&data->oob_tx_sync);
	}
}
