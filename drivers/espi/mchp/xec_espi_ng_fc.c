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
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
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

#if 0
struct espi_flash_packet {
	/** Pointer to the data buffer. */
	uint8_t *buf;
	/** Flash address to access. */
	uint32_t flash_addr;
	/**
	 * Length of the data in bytes for read/write, or the size of the
	 * sector/block for an erase operation.
	 */
	uint16_t len;
};
#endif

static uint8_t fc_buf[CONFIG_ESPI_FLASH_BUFFER_SIZE] __aligned(8);

static bool fc_is_busy(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;

	if (sys_test_bit(devcfg->base + XEC_ESPI_FC_CFG_OFS, XEC_ESPI_FC_CFG_BUSY_POS) != 0) {
		return true;
	}

	return 0;
}

/* blocking flash read */
int xec_espi_ng_fc_rd_api(const struct device *dev, struct espi_flash_packet *pkt)
{
	struct xec_espi_ng_data *data = dev->data;

	if (pkt == NULL) {
		return -EINVAL;
	}

	k_sem_take(&data->fc_lock, K_FOREVER);

	if (fc_is_busy(dev) == true) {
		k_sem_give(&data->fc_lock);
		return -EBUSY;
	}

	/* TODO */

	/* clear buffer before triggering read */
	memset(fc_buf, 0x55, sizeof(fc_buf));

	k_sem_give(&data->fc_lock);

	return 0;
}

int xec_espi_ng_fc_wr_api(const struct device *dev, struct espi_flash_packet *pkt)
{
	struct xec_espi_ng_data *data = dev->data;

	if (pkt == NULL) {
		return -EINVAL;
	}

	k_sem_take(&data->fc_lock, K_FOREVER);

	if (fc_is_busy(dev) == true) {
		k_sem_give(&data->fc_lock);
		return -EBUSY;
	}

	/* TODO */

	/* after HW finishes clear driver buffer */
	memset(fc_buf, 0x55, sizeof(fc_buf));

	k_sem_give(&data->fc_lock);

	return 0;
}

int xec_espi_ng_fc_er_api(const struct device *dev, struct espi_flash_packet *pkt)
{
	struct xec_espi_ng_data *data = dev->data;

	if (pkt == NULL) {
		return -EINVAL;
	}

	if (fc_is_busy(dev) == true) {
		k_sem_give(&data->fc_lock);
		return -EBUSY;
	}

	k_sem_take(&data->fc_lock, K_FOREVER);

	/* TODO */

	k_sem_give(&data->fc_lock);

	return 0;
}

/* TODO what if channel enable change (enable to disable) occurs during
 * on on-going flash transfer?
 */
void xec_espi_ng_fc_handler(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	struct xec_espi_ng_data *data = dev->data;
	mem_addr_t iob = devcfg->base;
	struct espi_event ev = {
		.evt_type = ESPI_BUS_EVENT_CHANNEL_READY,
		.evt_details = ESPI_CHANNEL_FLASH,
		.evt_data = 0,
	};
	uint32_t sr = sys_read32(iob + XEC_ESPI_FC_SR_OFS);

	data->fc_status = sr;

	sys_write32(sr, iob + XEC_ESPI_FC_SR_OFS);

	/* Did flash channel enable change? */
	if ((sr & BIT(XEC_ESPI_FC_SR_CHEN_CHG_POS)) != 0) {
		if ((sr & BIT(XEC_ESPI_FC_SR_CHEN_STATE_POS)) != 0) { /* 0->1 enable */
			/* set flash channel ready */
			soc_set_bit8(iob + XEC_ESPI_FC_RDY_OFS, XEC_ESPI_CHAN_RDY_POS);
			ev.evt_data |= ESPI_PC_EVT_BUS_CHANNEL_READY;
		}

		espi_send_callbacks(&data->cbs, dev, ev);
	}

	if ((sr & BIT(XEC_ESPI_FC_SR_DONE_POS)) != 0) {
		/* TODO transfer done */
		sys_clear_bit(iob + XEC_ESPI_FC_IER_OFS, XEC_ESPI_FC_IER_DONE_POS);
		k_sem_give(&data->fc_sync);
	}
}
