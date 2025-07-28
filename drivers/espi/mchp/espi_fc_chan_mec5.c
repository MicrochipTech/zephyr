/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_espi

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

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
#include "../espi_utils.h"
#include "../espi_mchp_mec5.h"
#include "espi_mchp_regs.h"

/* -------- eSPI Flash Channel -------- */

#define FC_EC_TAG        0
#define FC_EC_TIMEOUT_MS CONFIG_ESPI_FLASH_TIMEOUT_MS
#define FC_ERR_MSK       (BIT(ESPI_FC_SR_DIS_WB_POS) | BIT(ESPI_FC_SR_ABERR_POS) |\
			  BIT(ESPI_FC_SR_ABORT_POS) | BIT(ESPI_FC_SR_OVR_POS) |\
			  BIT(ESPI_FC_SR_INC_POS) | BIT(ESPI_FC_SR_FAIL_POS) |\
			  BIT(ESPI_FC_SR_START_OVFL_POS) | BIT(ESPI_FC_SR_BAD_REQ_POS))

static void espi_mec5_fc_isr(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t iob = drvcfg->ioc_base;
	struct espi_event evt = {
		.evt_type = ESPI_BUS_EVENT_CHANNEL_READY,
		.evt_details = ESPI_CHANNEL_FLASH,
		.evt_data = 0};
	uint32_t status = sys_read32(iob + MEC_ESPI_FC_SR_OFS);

	sys_write32(status, iob + MEC_ESPI_FC_SR_OFS);
	data->fc_status = status;

	if (status & BIT(MEC_ESPI_FC_SR_CHEN_CHG_POS)) {
		if ((status & BIT(MEC_ESPI_FC_SR_CHEN_STATE_POS)) != 0) { /* enabled by Host? */
			sys_set_bit8(iob + MEC_ESPI_FC_RDY_OFS, MEC_ESPI_CHAN_RDY_POS);
			evt.evt_data = 1u;
		}

		espi_send_callbacks(&data->callbacks, dev, evt);
	}

	if ((status & BIT(MEC_ESPI_FC_SR_DONE_POS)) != 0) {
		k_sem_give(&data->fc_sync);
	}
}

void espi_mec5_fc_irq_connect(const struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, fc, irq), DT_INST_IRQ_BY_NAME(0, fc, priority),
		    espi_mec5_fc_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, fc, irq));

	sys_write32(MEC_ESPI_GIRQ_ENSET_ADDR, BIT(MEC_ESPI_GIRQ_FC_POS));
}

void espi_mec5_fc_erst_config(const struct device *dev, uint8_t n_erst_state)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iob = drvcfg->ioc_base;

	if (n_erst_state != 0) {
		sys_write32(BIT(MEC_ESPI_FC_IER_CHG_EN_POS), iob + MEC_ESPI_FC_IER_OFS);
	}
}

#define FC_READ 0
#define FC_WRITE 1
#define FC_ERASE 2

/* TODO For erase should be check pkt->len again max_size?
 * One catch is the one erase mode where 4K and 64K are allowed?
 * In that case we would use 64K
*/
static int espi_mec5_flash_check_params(struct espi_flash_packet *pkt, uint32_t max_size,
					uint8_t op)
{
	if (pkt == NULL) {
		return -EINVAL;
	}

	if ((op == FC_READ) || (op == FC_WRITE)) {
		if (!IS_ALIGNED(pkt->buf, 4)) {
			return -EINVAL;
		}

		if ((pkt->buf == NULL) && (pkt->len != 0)) {
			return -EINVAL;
		}

		if (pkt->len > max_size) {
			return -E2BIG;
		}

		return 0;
	} else if (op == FC_WRITE) {
		return 0;
	} else {
		return -EINVAL;
	}
}

static bool fc_is_enabled(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iob = drvcfg->ioc_base;

	if (sys_test_bit8(iob + MEC_ESPI_FC_RDY_OFS, MEC_ESPI_CHAN_RDY_POS) != 0) {
		return true;
	}

	return false;
}

static bool fc_is_busy(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iob = drvcfg->ioc_base;

	if (sys_test_bit(iob + MEC_ESPI_FC_CFG_OFS, MEC_ESPI_FC_CFG_BUSY_POS) != 0) {
		return true;
	}

	return false;
}

static uint32_t fc_max_rw_size(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iob = drvcfg->ioc_base;
	uint32_t fcfg = sys_read32(iob + MEC_ESPI_FC_CFG_OFS);

	fcfg = MEC_ESPI_FC_CFG_MPLD_GET(fcfg);

	return (1u << (fcfg + 5u));
}

/* eSPI flash channel supports one erase capability with two erase sizes: 4KB and 64KB
 * If the Host configuration supports both sizes examine the caller's erase request
 * size and select the 4KB or 64KB command.
 */
static uint32_t fc_erase_mode(const struct device *dev, uint32_t erase_len)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iob = drvcfg->ioc_base;
	uint32_t erase_ctrl = MEC_ESPI_FC_CR_FUNC_ERASE_SM;
	uint32_t fcfg = sys_read32(iob + MEC_ESPI_FC_CFG_OFS);

	fcfg = MEC_ESPI_FC_CFG_EBSZ_GET(fcfg);
	if (fcfg == MEC_ESPI_FC_CFG_EBSZ_4K_64K_VAL) {
		if (erase_len > 4096u) {
			erase_ctrl = MEC_ESPI_FC_CR_FUNC_ERASE_LG;
		}
	}

	return erase_ctrl;
}

static void fc_abort(const struct device *dev, uint8_t op)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iob = drvcfg->ioc_base;

	sys_set_bit(iob + MEC_ESPI_FC_CR_OFS, MEC_ESPI_FC_CR_ABORT_POS);
}

static int espi_mec5_fc_op(const struct device *dev, struct espi_flash_packet *pckt, uint8_t op)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t iob = drvcfg->ioc_base;
	uint32_t max_size = fc_max_rw_size(dev);
	uint32_t ctrl = 0, status = 0, len = 0, buf_addr = 0;
	int rc = 0;

	if (fc_is_enabled(dev) == false) {
		return -EIO;
	}

	if (fc_is_busy(dev) == true) {
		return -EBUSY;
	}

	rc = espi_mec5_flash_check_params(pckt, max_size, op);
	if (rc != 0) {
		return rc;
	}

	data->fc_len = pckt->len;
	data->fc_buf = pckt->buf;
	k_sem_reset(&data->fc_sync);

	buf_addr = (uint32_t)pckt->buf;
	len = pckt->len;
	ctrl = MEC_ESPI_FC_CR_TAG_SET(FC_EC_TAG) | BIT(MEC_ESPI_FC_CR_START_POS);
	if (op == FC_WRITE) {
		ctrl |= MEC_ESPI_FC_CR_FUNC_WRITE;
	} else if (op == FC_ERASE) {
		ctrl |= fc_erase_mode(dev, len);
		/* Even for erase HW wants non-zero buffer address. No tranfer */
		buf_addr = (uint32_t)&data->fc_data;
		len = 1u; /* erase requires non-zero length (1 recommended) */
	} else {
		ctrl |= MEC_ESPI_FC_CR_FUNC_READ;
	}

	sys_write32(UINT32_MAX, iob + MEC_ESPI_FC_SR_OFS);
	sys_write32(pckt->flash_addr, iob + MEC_ESPI_FC_FA_OFS);
	sys_write32(buf_addr, iob + MEC_ESPI_FC_BA_OFS);
	sys_write32(pckt->len, iob + MEC_ESPI_FC_LEN_OFS);

	sys_set_bit(iob + MEC_ESPI_FC_IER_OFS, MEC_ESPI_FC_IER_DONE_POS);
	sys_write32(ctrl, iob + MEC_ESPI_FC_CR_OFS);

	rc = k_sem_take(&data->fc_sync, K_MSEC(FC_EC_TIMEOUT_MS));
	if (rc == -EAGAIN) {
		fc_abort(dev, FC_READ);
		rc = k_sem_take(&data->fc_sync, K_MSEC(FC_EC_TIMEOUT_MS));
	}

	if (rc == -EAGAIN) {
		data->fc_status = sys_read32(iob + MEC_ESPI_FC_SR_OFS);
		return -ETIMEDOUT;
	}

	/* interrupt fired */
	status = data->fc_status;
	if ((status & MEC_ESPI_FC_SR_ERR_ALL_MSK) != 0) {
		if (status & BIT(MEC_ESPI_FC_SR_ABORT_POS)) {
			return -ETIMEDOUT;
		}
		return -EIO;
	}

	return 0;
}

int espi_mec5_flash_read_api(const struct device *dev, struct espi_flash_packet *pckt)
{
	return espi_mec5_fc_op(dev, pckt, FC_READ);
}

int espi_mec5_flash_write_api(const struct device *dev, struct espi_flash_packet *pckt)
{
	return espi_mec5_fc_op(dev, pckt, FC_WRITE);
}

int espi_mec5_flash_erase_api(const struct device *dev, struct espi_flash_packet *pckt)
{
	return espi_mec5_fc_op(dev, pckt, FC_ERASE);
}
