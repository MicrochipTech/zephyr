/*
 * Copyright (c) 2021 Microchip Technology Inc.
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Microchip XEC eSPI peripheral channel Embedded Memory Interface (EMI).
 *
 * An EMI block gives the Host windowed access to two regions of SoC data
 * memory through a small set of Host I/O registers, plus a pair of one byte
 * mailboxes and a set of EC to Host software interrupt sources.
 *
 * MEC hardware has three identical instances. The V2 eSPI driver supports only
 * the first of them, as static code inside espi_mchp_xec_host_v2.c compiled in
 * by CONFIG_ESPI_PERIPHERAL_EC_HOST_CMD, and points its region 0 at a buffer
 * the eSPI driver itself declares and sizes from two Kconfig symbols. It never
 * enables the EMI interrupt, so a Host write to the mailbox is invisible, and a
 * board that wants a second EMI window has nowhere to describe it.
 *
 * Here one multi instance driver serves all three. Each region comes from the
 * emi-mems device tree property on the node, or from a caller that owns its own
 * buffer and installs it with mchp_xec_espi_pc_emi_region_set(). The driver
 * keeps the region values and re-applies them from its peripheral channel event
 * handler, because MEC hardware holds the Host facing registers in reset while
 * eSPI Reset or PLTRST is asserted.
 *
 * The Host I/O BAR, the Memory BAR and the two Serial IRQs (Host event and EC
 * to Host) are described on the node as io-bars, mem-bars and sirqs, and are
 * programmed by the eSPI controller. Nothing here touches them.
 */

#define DT_DRV_COMPAT microchip_xec_espi_pc_emi

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi/mchp_xec_espi.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "../espi_mchp_xec_v3.h"
#include "espi_pc_xec.h"

LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

/* The SoC register tree has no EMI header, so the block layout is spelled out
 * here. Offsets below 0x100 are the registers the Host reaches through the I/O
 * BAR; the ones at 0x100 and above are EC only.
 */
struct xec_emi_regs {
	volatile uint8_t RT_H2EMB;
	volatile uint8_t RT_E2HMB;
	volatile uint16_t RT_RGO;
	volatile uint32_t RT_DATA;
	volatile uint16_t RT_ISRC;
	volatile uint16_t RT_IMASK;
	volatile uint8_t RT_AID;
	uint8_t RSVD1[3];
	volatile uint8_t RT_ASAID;
	uint8_t RSVD2[3];
	uint8_t RSVD3[(0x100U - 0x014U)];
	volatile uint8_t H2EMB;
	volatile uint8_t E2HMB;
	uint8_t RSVD4[2];
	volatile uint32_t MRB[2 * MCHP_XEC_ESPI_PC_EMI_REGION_MAX];
	volatile uint16_t ISEN;
	volatile uint16_t IHCEN;
	uint8_t RSVD5[8];
	volatile uint32_t AIDS[8];
};

BUILD_ASSERT(sizeof(struct xec_emi_regs) == 0x140U, "XEC EMI register layout mismatch");

/* MRB holds base and limit for each region as consecutive registers. */
#define XEC_EMI_MRB_BASE(region) ((region) * 2U)
#define XEC_EMI_MRB_LIM(region)  (((region) * 2U) + 1U)

#define XEC_EMI_BASE_MSK GENMASK(31, 2)
#define XEC_EMI_LIM_RD_MSK GENMASK(14, 2)
#define XEC_EMI_LIM_WR_MSK GENMASK(30, 18)

#define XEC_EMI_LIM(rd, wr)                                                                        \
	((((uint32_t)(rd)) & XEC_EMI_LIM_RD_MSK) |                                                 \
	 ((((uint32_t)(wr)) << 16) & XEC_EMI_LIM_WR_MSK))

/* Bit 0 of the interrupt source set register is driven by hardware when the EC
 * writes the EC to Host mailbox. Only bits 1 and above are software sources.
 */
#define XEC_EMI_SWI_MSK GENMASK(15, 1)

/* Cells per emi-mems entry, matching emi-mem-cells in the binding. */
#define XEC_EMI_MEM_CELLS 3

struct xec_emi_region {
	uint32_t base;
	uint16_t rd_limit;
	uint16_t wr_limit;
};

struct espi_pc_emi_xec_config {
	struct xec_emi_regs *regs;
	const struct device *espi_dev;
	void (*irq_connect)(void);
	const uint32_t *mems;
	uint32_t ecia_info;
	uint8_t mems_len;
	uint8_t ldn;
};

struct espi_pc_emi_xec_data {
	struct mchp_xec_espi_pc_cb pc_cb;
	struct xec_emi_region regions[MCHP_XEC_ESPI_PC_EMI_REGION_MAX];
};

static void emi_region_apply(const struct device *dev, enum mchp_xec_espi_pc_emi_region region)
{
	const struct espi_pc_emi_xec_config *cfg = dev->config;
	struct espi_pc_emi_xec_data *data = dev->data;
	const struct xec_emi_region *r = &data->regions[region];

	cfg->regs->MRB[XEC_EMI_MRB_BASE(region)] = r->base & XEC_EMI_BASE_MSK;
	cfg->regs->MRB[XEC_EMI_MRB_LIM(region)] = XEC_EMI_LIM(r->rd_limit, r->wr_limit);
}

/* (Re)apply everything the block loses while it is held in reset. */
static void emi_block_config(const struct device *dev)
{
	const struct espi_pc_emi_xec_config *cfg = dev->config;

	/* Discard a stale Host command byte before the Host can be told the
	 * window is live again.
	 */
	cfg->regs->H2EMB = 0U;

	for (int i = 0; i < MCHP_XEC_ESPI_PC_EMI_REGION_MAX; i++) {
		emi_region_apply(dev, (enum mchp_xec_espi_pc_emi_region)i);
	}
}

int mchp_xec_espi_pc_emi_region_set(const struct device *dev,
				    enum mchp_xec_espi_pc_emi_region region, void *base,
				    uint16_t rd_limit, uint16_t wr_limit)
{
	struct espi_pc_emi_xec_data *data = NULL;
	unsigned int key;

	if ((dev == NULL) || (region >= MCHP_XEC_ESPI_PC_EMI_REGION_MAX)) {
		return -EINVAL;
	}

	data = dev->data;

	key = irq_lock();
	data->regions[region].base = (uint32_t)(uintptr_t)base;
	data->regions[region].rd_limit = rd_limit;
	data->regions[region].wr_limit = wr_limit;
	emi_region_apply(dev, region);
	irq_unlock(key);

	return 0;
}

int mchp_xec_espi_pc_emi_mbox_get(const struct device *dev, uint8_t *h2ec, uint8_t *ec2h)
{
	const struct espi_pc_emi_xec_config *cfg = NULL;

	if (dev == NULL) {
		return -EINVAL;
	}

	cfg = dev->config;

	if (h2ec != NULL) {
		*h2ec = cfg->regs->H2EMB;
	}

	if (ec2h != NULL) {
		*ec2h = cfg->regs->E2HMB;
	}

	return 0;
}

int mchp_xec_espi_pc_emi_mbox_set(const struct device *dev, uint8_t ec2h)
{
	const struct espi_pc_emi_xec_config *cfg = NULL;

	if (dev == NULL) {
		return -EINVAL;
	}

	cfg = dev->config;
	cfg->regs->E2HMB = ec2h;

	return 0;
}

int mchp_xec_espi_pc_emi_host_swi_set(const struct device *dev, uint16_t swi_bits)
{
	const struct espi_pc_emi_xec_config *cfg = NULL;

	if (dev == NULL) {
		return -EINVAL;
	}

	cfg = dev->config;
	cfg->regs->ISEN = swi_bits & XEC_EMI_SWI_MSK;

	return 0;
}

/* Host wrote the Host-to-EC mailbox register. */
static void emi_isr(const struct device *dev)
{
	const struct espi_pc_emi_xec_config *cfg = dev->config;
	struct espi_event evt = {
		.evt_type = ESPI_BUS_PERIPHERAL_NOTIFICATION,
		.evt_details = MCHP_XEC_ESPI_PERIPHERAL_EMI,
		.evt_data = 0U,
	};
	struct mchp_xec_espi_pc_emi_evt *emi_evt =
		(struct mchp_xec_espi_pc_emi_evt *)&evt.evt_data;

	emi_evt->ldn = cfg->ldn;
	emi_evt->data = cfg->regs->H2EMB;

	/* Clear the latched status before invoking the application so a write
	 * arriving from the Host while the callback runs is not lost.
	 */
	xec_pc_girq_clr(cfg->ecia_info);

	mchp_xec_espi_v3_send_callbacks(cfg->espi_dev, evt);
}

static void emi_espi_event(const struct device *espi_dev, const struct device *dev,
			   enum mchp_xec_espi_pc_event evt)
{
	const struct espi_pc_emi_xec_config *cfg = dev->config;

	ARG_UNUSED(espi_dev);

	if (xec_pc_evt_is_hw_usable(evt)) {
		/* The controller has re-programmed our Host I/O BAR, Memory BAR
		 * and Serial IRQs.
		 */
		emi_block_config(dev);
		xec_pc_girq_clr(cfg->ecia_info);
		xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_EN);
	} else {
		xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	}
}

static int espi_pc_emi_xec_init(const struct device *dev)
{
	const struct espi_pc_emi_xec_config *cfg = dev->config;
	struct espi_pc_emi_xec_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->espi_dev)) {
		LOG_ERR("EMI ldn %u: eSPI controller not ready", cfg->ldn);
		return -ENODEV;
	}

	/* Seed the region shadow from the device tree. A node without emi-mems
	 * leaves both regions zeroed, which disables Host access to them until
	 * a caller installs one with mchp_xec_espi_pc_emi_region_set().
	 */
	for (uint8_t i = 0; (i + XEC_EMI_MEM_CELLS) <= cfg->mems_len;
	     i += XEC_EMI_MEM_CELLS) {
		struct xec_emi_region *r = &data->regions[i / XEC_EMI_MEM_CELLS];

		r->base = cfg->mems[i];
		r->rd_limit = (uint16_t)cfg->mems[i + 1];
		r->wr_limit = (uint16_t)cfg->mems[i + 2];
	}

	xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	xec_pc_girq_clr(cfg->ecia_info);

	emi_block_config(dev);

	data->pc_cb.pc_dev = dev;
	data->pc_cb.handler = emi_espi_event;
	data->pc_cb.evt_mask = XEC_PC_EVT_MASK_ALL;

	ret = mchp_xec_espi_pc_register(cfg->espi_dev, &data->pc_cb);
	if (ret != 0) {
		LOG_ERR("EMI ldn %u: eSPI registration failed (%d)", cfg->ldn, ret);
		return ret;
	}

	cfg->irq_connect();

	/* The source stays masked until the controller reports the Host facing
	 * decoders are valid.
	 */

	return 0;
}

#define XEC_EMI_MEMS_DEFINE(n)                                                                     \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(n, emi_mems),                                             \
		   (BUILD_ASSERT((DT_INST_PROP_LEN(n, emi_mems) % XEC_EMI_MEM_CELLS) == 0,         \
				 "emi-mems needs three cells per region");                         \
		    BUILD_ASSERT(DT_INST_PROP_LEN(n, emi_mems) <=                                  \
					 (XEC_EMI_MEM_CELLS * MCHP_XEC_ESPI_PC_EMI_REGION_MAX),    \
				 "emi-mems describes more regions than the block has");            \
		    static const uint32_t emi_mems_##n[] = DT_INST_PROP(n, emi_mems);))

#define XEC_EMI_MEMS_INIT(n)                                                                       \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(n, emi_mems),                                             \
		   (.mems = emi_mems_##n, .mems_len = DT_INST_PROP_LEN(n, emi_mems),))

#define XEC_EMI_DEVICE(n)                                                                          \
	XEC_EMI_MEMS_DEFINE(n)                                                                     \
	static void emi_irq_connect_##n(void)                                                      \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), emi_isr,                    \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static const struct espi_pc_emi_xec_config espi_pc_emi_xec_cfg_##n = {                     \
		.regs = (struct xec_emi_regs *)DT_INST_REG_ADDR(n),                                \
		.espi_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                      \
		.irq_connect = emi_irq_connect_##n,                                                \
		XEC_EMI_MEMS_INIT(n).ecia_info = DT_INST_PROP_BY_IDX(n, girqs, 0),                 \
		.ldn = DT_INST_PROP(n, ldn),                                                       \
	};                                                                                         \
	static struct espi_pc_emi_xec_data espi_pc_emi_xec_data_##n;                               \
	DEVICE_DT_INST_DEFINE(n, espi_pc_emi_xec_init, NULL, &espi_pc_emi_xec_data_##n,            \
			      &espi_pc_emi_xec_cfg_##n, POST_KERNEL,                               \
			      CONFIG_ESPI_XEC_V3_PC_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(XEC_EMI_DEVICE)
