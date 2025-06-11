/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_espi

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

#include <errno.h>
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
#include "espi_mchp_mec5_regs.h"

#if 0 /* from ../espi_mchp_mec5.h */

struct espi_mec5_drv_data {
	const struct device *dev;
	volatile uint32_t status;
	struct espi_callback vwcb;
	sys_slist_t callbacks;
#if defined(CONFIG_ESPI_OOB_CHANNEL)
	struct k_sem oob_tx_sync;
	struct k_sem oob_rx_sync;
	uint8_t *oob_rxb;
	uint16_t oob_rx_len;
#endif
};

#endif

/* -------- OOB Channel -------- */
#define OOB_TARGET_TAG_NUM	0
#define OOB_TX_MAX_TIMEOUT_MS	200u
#define OOB_RX_MAX_TIMEOUT_MS	200u

#define OOB_TX_ANY_ERROR						\
	(BIT(ESPI_OOB_TX_SR_ABERR_POS) | BIT(ESPI_OOB_TX_SR_OVR_POS) | BIT(ESPI_OOB_TX_SR_BAD_REQ_POS))

#define OOB_RX_ANY_ERROR	(BIT(ESPI_OOB_RX_SR_ABERR_POS) | BIT(ESPI_OOB_RX_SR_OVR_POS))

/* MEC OOB hardware has 9 bytes of added buffer size */
static uint32_t mec5_oob_get_max_payload_size(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iob = drvcfg->ioc_base;
	uint32_t msize = sys_read8(iob + ESPI_CAP_OOB);

	msize = ESPI_CAP_OOB_MPLD(msize);
	switch (msize) {
	case 2u:
		return 128u + ESPI_OOB_ADDED_SIZE;
	case 3u:
		return 256u + ESPI_OOB_ADDED_SIZE;
	default:
		return 64u + ESPI_OOB_ADDED_SIZE;
	}
}

/* OOB registers and OOB channel ready are reset by nESPI_RESET from the Host.
 * This routine is called when the Host sends OOB Enable message. We configure
 * OOB TX/RX hardware and set OOB channel ready informing the Host OOB is usable.
 */
static void espi_mec5_oob_config(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t iob = drvcfg->ioc_base;
	uint32_t r = 0;

	sys_write32((uint32_t)data->oob_rxb, iob + ESPI_OOB_RX_BA);

	/* set OOB RX buffer maximum size */
	r = sys_read32(iob + ESPI_OOB_RXL);
	r &= (uint32_t)~(ESPI_OOB_RXL_BLEN_MSK);
	r |= (((uint32_t)drvcfg->oob_rxb_size << ESPI_OOB_RXL_BLEN_POS) & ESPI_OOB_RXL_BLEN_MSK);
	sys_write32(r, iob + ESPI_OOB_RXL);

	/* set RX buffer available */
	sys_set_bit(iob + ESPI_OOB_RX_CR, ESPI_OOB_RX_CR_SRA_POS);

	/* enable OOB channel RX interrupt */
	sys_set_bit(iob + ESPI_OOB_RX_IER, ESPI_OOB_RX_IER_DONE_POS);

	/* Inform Host OOB channel is ready to use */
	sys_set_bit8(iob + ESPI_OOB_READY, ESPI_CHAN_RDY_POS);
 }

/* OOB channel upstream data transfer (Target to Host) and OOB channel enable change */
static void espi_mec5_oob_up_isr(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t iob = drvcfg->ioc_base;
	struct espi_event evt = {
		.evt_type = ESPI_BUS_EVENT_CHANNEL_READY,
		.evt_details = ESPI_CHANNEL_OOB,
		.evt_data = 0,
	};
	uint32_t oob_tx_sts = sys_read32(iob + ESPI_OOB_TX_SR);

	sys_write32(oob_tx_sts, iob + ESPI_OOB_TX_SR);
	sys_write32(BIT(ESPI_GIRQ_OOB_UP_POS), ESPI_GIRQ_STS_ADDR);

	data->oob_tx_status = oob_tx_sts;

	if (oob_tx_sts & BIT(ESPI_OOB_TX_SR_DONE_POS)) {
		k_sem_give(&data->oob_tx_sync);
	}

	if (oob_tx_sts & BIT(ESPI_OOB_TX_SR_CENC_POS)) {
		if (oob_tx_sts & BIT(ESPI_OOB_TX_SR_CHEN_POS)) { /* channel was enabled? */
			espi_mec5_oob_config(dev);
			evt.evt_data = 1;
		} else {
			evt.evt_data = 0;
		}
		evt.evt_type = ESPI_BUS_EVENT_CHANNEL_READY;
		evt.evt_details = ESPI_CHANNEL_OOB;
		espi_send_callbacks(&data->callbacks, dev, evt);
	}
}

/* OOB channel downstream data transfer (Host to Target) */
static void espi_mec5_oob_dn_isr(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t iob = drvcfg->ioc_base;
#ifdef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
	struct espi_event evt = { .evt_type = ESPI_BUS_EVENT_OOB_RECEIVED,
				  .evt_details = 0,
				  .evt_data = 0 };
#endif
	uint32_t oob_rx_sts = sys_read32(iob + ESPI_OOB_RX_SR);

	sys_write32(oob_rx_sts, iob + ESPI_OOB_RX_SR);
	sys_write32(BIT(ESPI_GIRQ_OOB_DN_POS), ESPI_GIRQ_STS_ADDR);

	data->oob_rx_status = oob_rx_sts;

	if (oob_rx_sts & BIT(ESPI_OOB_RX_SR_DONE_POS)) {
#ifdef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
		evt.evt_details = mec_hal_espi_oob_received_len(iob);
		espi_send_callbacks(&data->callbacks, dev, evt);
#else
		k_sem_give(&data->oob_rx_sync);
#endif
	}
}

void espi_mec5_oob_irq_connect(const struct device *dev)
{
/*	const struct espi_mec5_drv_cfg *drvcfg = dev->config; */
/*	mm_reg_t iob = drvcfg->ioc_base; */
	uint32_t oob_ien_msk = BIT(ESPI_GIRQ_OOB_UP_POS) | BIT(ESPI_GIRQ_OOB_DN_POS);

	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, oob_up, irq),
		DT_INST_IRQ_BY_NAME(0, oob_up, priority),
		espi_mec5_oob_up_isr,
		DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, oob_up, irq));

	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, oob_dn, irq),
		DT_INST_IRQ_BY_NAME(0, oob_dn, priority),
		espi_mec5_oob_dn_isr,
		DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, oob_dn, irq));

	sys_write32(oob_ien_msk, ESPI_GIRQ_ENSET_ADDR);
}

/* Called by core driver nESPI_RESET handler.
 * If nESPI_RESET is 1 (de-asserted) we enable OOB channel enable change interrupt.
 */
void espi_mec5_oob_erst_config(const struct device *dev, uint8_t n_erst_state)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iob = drvcfg->ioc_base;

	if (n_erst_state != 0) {
		sys_write32(iob + ESPI_OOB_TX_IER, BIT(ESPI_OOB_TX_IER_CENC_POS));
	}
}

int espi_mec5_send_oob_api(const struct device *dev, struct espi_oob_packet *pckt)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t iob = drvcfg->ioc_base;
	uint32_t max_size = mec5_oob_get_max_payload_size(dev);
	uint32_t status = 0;
	int rc = 0;

	if ((pckt == NULL) || !IS_ALIGNED(pckt->buf, 4) || (pckt->len > max_size)) {
		return -EINVAL;
	}

	if ((pckt->buf == NULL) || (pckt->len == 0)) { /* nothing to do */
		return 0;
	}

	if (sys_test_bit8(iob + ESPI_OOB_READY, ESPI_CHAN_RDY_POS) == 0) {
		return -EIO; /* OOB channel not ready */
	}

	status = sys_read32(iob + ESPI_OOB_TX_SR);
	if ((status & BIT(ESPI_OOB_TX_SR_BUSY_POS)) != 0) {
		return -EBUSY;
	}

	sys_write32(iob + ESPI_OOB_TX_SR, status);
	sys_write32(iob + ESPI_OOB_TX_BA, (uint32_t)pckt->buf);
	sys_write32(iob + ESPI_OOB_TX_LEN, pckt->len);
	sys_set_bit(iob + ESPI_OOB_TX_IER, ESPI_OOB_TX_IER_DONE_POS);
	sys_write32(iob + ESPI_OOB_TX_CR, (ESPI_OOB_TX_TAG(OOB_TARGET_TAG_NUM) |
					   BIT(ESPI_OOB_TX_CR_START_POS)));

	rc = k_sem_take(&data->oob_tx_sync, K_MSEC(OOB_TX_MAX_TIMEOUT_MS));
	if (rc == -EAGAIN) {
		status = sys_read32(iob + ESPI_OOB_TX_SR);
		sys_write32(iob + ESPI_OOB_TX_SR, UINT32_MAX);
		LOG_ERR("TX failed: HW status 0x%0x", status);
		return -EIO;
	}

	if ((data->oob_tx_status & OOB_TX_ANY_ERROR) != 0) {
		return -EIO;
	}

	return 0;
}

static void espi_mec5_oob_rx_prep(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t iob = drvcfg->ioc_base;
	uint32_t r = 0;

	sys_clear_bit(iob + ESPI_OOB_RX_IER, ESPI_OOB_RX_IER_DONE_POS);
	sys_clear_bit(iob + ESPI_OOB_RX_CR, ESPI_OOB_RX_CR_SRA_POS);
	sys_write32(UINT32_MAX, iob + ESPI_OOB_RX_SR);

	memset(data->oob_rxb, 0, drvcfg->oob_rxb_size);

	sys_write32((uint32_t)data->oob_rxb, iob + ESPI_OOB_RX_BA);
	r = sys_read32(iob + ESPI_OOB_RXL);
	r &= (uint32_t)~(ESPI_OOB_RXL_BLEN_MSK);
	r |= ESPI_OOB_RXL_BLEN((uint32_t)drvcfg->oob_rxb_size);
	sys_write32(r, iob + ESPI_OOB_RXL);

	/* enable OOB channel RX interrupt */
	sys_set_bit(iob + ESPI_OOB_RX_IER, ESPI_OOB_RX_IER_DONE_POS);

	/* set RX buffer available */
	sys_set_bit(iob + ESPI_OOB_RX_CR, ESPI_OOB_RX_CR_SRA_POS);
}

int espi_mec5_recv_oob_api(const struct device *dev, struct espi_oob_packet *pckt)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t iob = drvcfg->ioc_base;
	uint32_t status = 0;

	if ((pckt == NULL) || (pckt->buf == NULL)) {
		return -EINVAL;
	}

	status = sys_read32(iob + ESPI_OOB_RX_SR);
	if (status & OOB_RX_ANY_ERROR) {
		LOG_ERR("RX error 0x%0x", status);
		return -EIO;
	}

#ifndef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
	int rc = k_sem_take(&data->oob_rx_sync, K_MSEC(OOB_RX_MAX_TIMEOUT_MS));

	if (rc == -EAGAIN) {
		return -ETIMEDOUT;
	}
#endif
	if (data->oob_rx_status & OOB_RX_ANY_ERROR) {
		LOG_ERR("RX error 0x%0x", data->oob_rx_status);
		data->oob_rx_status = 0;
		espi_mec5_oob_rx_prep(dev);
		return -EIO;
	}

	if (data->oob_rx_len > pckt->len) {
		LOG_ERR("space rx %u vs %u", data->oob_rx_len, pckt->len);
		espi_mec5_oob_rx_prep(dev);
		return -EIO;
	}

	memcpy(pckt->buf, data->oob_rxb, data->oob_rx_len);

	espi_mec5_oob_rx_prep(dev);

	return 0;
}
