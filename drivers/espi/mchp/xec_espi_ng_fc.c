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

#define XEC_ESPI_FC_ERASE_CMD_LEN 1u
#define XEC_ESPI_FC_OP_EC_TAG 0
#define XEC_ESPI_FC_OP_TIMEOUT_MS 1000u

/* NOTE: nESPI_RESET or Host clearing flash channel enable will cause
 * our hardware to clear flash channel ready bit.
 */
static bool fc_is_ready(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mm_reg_t iom_base = devcfg->base;

	if (soc_test_bit8(iom_base + XEC_ESPI_FC_RDY_OFS, XEC_ESPI_CHAN_RDY_POS) != 0) {
		return true;
	}

	return false;
}

static bool fc_is_busy(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;

	if (sys_test_bit(devcfg->base + XEC_ESPI_FC_CFG_OFS, XEC_ESPI_FC_CFG_BUSY_POS) != 0) {
		return true;
	}

	return 0;
}

int static xec_espi_ng_fc_op(const struct device *dev, struct espi_flash_packet *pkt, uint8_t op)
{
	struct xec_espi_ng_data *const data = dev->data;
	const struct xec_espi_ng_drvcfg *drvcfg = dev->config;
	mm_reg_t iob = drvcfg->base;
	uint32_t fc_cr = 0, xfr_len = 0;
	int rc = 0;

	if (fc_is_ready(dev) == false) {
		return -ECONNRESET;
	}

	if (fc_is_busy(dev) == true) {
		return -EBUSY;
	}

	if (IS_PTR_ALIGNED(pkt->buf, uint32_t) == false) {
		LOG_ERR("eSPI FC op: packet buffer not 4-byte aligned");
		return -EINVAL;
	}

	k_sem_take(&data->fc_lock, K_FOREVER);
	k_sem_reset(&data->fc_sync);

	data->fc_status = 0;

	xfr_len = pkt->len;
	if ((op == XEC_ESPI_FC_CR_FUNC_ERASE_SM) || (op == XEC_ESPI_FC_CR_FUNC_ERASE_LG)) {
		xfr_len = XEC_ESPI_FC_ERASE_CMD_LEN;
	}

	fc_cr = XEC_ESPI_FC_CR_FUNC_SET((uint32_t)op);
	fc_cr |= XEC_ESPI_FC_CR_TAG_SET(XEC_ESPI_FC_OP_EC_TAG);

	sys_write32(XEC_ESPI_FC_SR_ERR_ALL_MSK, iob + XEC_ESPI_FC_SR_OFS);

	sys_write32((uint32_t)pkt->flash_addr, iob + XEC_ESPI_FC_FA_OFS);
	sys_write32((uint32_t)pkt->buf, iob + XEC_ESPI_FC_BA_OFS);
	sys_write32(xfr_len, iob + XEC_ESPI_FC_LEN_OFS);

	sys_write32(fc_cr, iob + XEC_ESPI_FC_CR_OFS);
	sys_set_bit(iob + XEC_ESPI_FC_IER_OFS, XEC_ESPI_FC_IER_DONE_POS);
	sys_set_bit(iob + XEC_ESPI_FC_CR_OFS, XEC_ESPI_FC_CR_START_POS);

	rc = k_sem_take(&data->fc_sync, K_MSEC(XEC_ESPI_FC_OP_TIMEOUT_MS));
	if (rc != -EAGAIN) {
		sys_clear_bit(iob + XEC_ESPI_FC_IER_OFS, XEC_ESPI_FC_IER_DONE_POS);
		sys_set_bit(iob + XEC_ESPI_FC_CR_OFS, XEC_ESPI_FC_CR_ABORT_POS);
		LOG_ERR("eSPI FC op=%u timed out", op);
		rc = -ETIMEDOUT;
		goto fc_op_exit;
	}

	if ((data->fc_status & XEC_ESPI_FC_SR_ERR_ALL_MSK) != 0) {
		LOG_ERR("eSPI FC op: err sts = 0x%0x", data->fc_status);
		rc = -EIO;
	}

fc_op_exit:
	k_sem_give(&data->fc_lock);

	return rc;
}

/* blocking flash read */
int xec_espi_ng_fc_rd_api(const struct device *dev, struct espi_flash_packet *pkt)
{
	if (pkt == NULL) {
		return -EINVAL;
	}

	return xec_espi_ng_fc_op(dev, pkt, XEC_ESPI_FC_CR_FUNC_READ);
}

int xec_espi_ng_fc_wr_api(const struct device *dev, struct espi_flash_packet *pkt)
{
	if (pkt == NULL) {
		return -EINVAL;
	}

	return xec_espi_ng_fc_op(dev, pkt, XEC_ESPI_FC_CR_FUNC_WRITE);
}

int xec_espi_ng_fc_er_api(const struct device *dev, struct espi_flash_packet *pkt)
{
	uint8_t op = 0;

	if (pkt == NULL) {
		return -EINVAL;
	}

	op = XEC_ESPI_FC_CR_FUNC_ERASE_SM;
	if (pkt->len != 0) {
		op = XEC_ESPI_FC_CR_FUNC_ERASE_LG;
	}

	return xec_espi_ng_fc_op(dev, pkt, op);
}

/* eSPI flash channel interrupt handler
 * Events:
 *   Host setting or clearing flash channel enable.
 *   Flash channel operation DONE.
 * NOTE: if the Host disables the channel any on-going
 * flash operation will cause our FC hardware to set
 * the disabled by controller status.
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

	sys_write32(sr, iob + XEC_ESPI_FC_SR_OFS);

	data->fc_status = sr;

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
		sys_clear_bit(iob + XEC_ESPI_FC_IER_OFS, XEC_ESPI_FC_IER_DONE_POS);
		k_sem_give(&data->fc_sync);
	}
}
