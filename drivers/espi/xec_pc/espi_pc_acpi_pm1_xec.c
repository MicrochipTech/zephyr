/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Microchip XEC eSPI peripheral channel ACPI PM1 block.
 *
 * The ACPI fixed hardware PM1 status, enable and control registers the Host
 * sees in its I/O space. Hardware raises one interrupt per register group when
 * the Host writes it: control, enable and status.
 *
 * The V2 eSPI driver has no support for this block at all. Its dispatch tables
 * in espi_mchp_xec_host_v2.c carry NULL for both the ACPI PM1 init and IRQ
 * connect slots, so a board using the V2 driver cannot see a Host write to PM1
 * even though the node exists in the device tree. This driver adds it.
 *
 * The generic enum espi_virtual_peripheral has no ACPI fixed hardware entry and
 * enum lpc_peripheral_opcode has no PM1 opcodes, so the driver reports events
 * under the vendor peripheral identifier MCHP_XEC_ESPI_PERIPHERAL_ACPI_PM1 and
 * exposes register access through mchp_xec_espi_pc_acpi_pm1_get() and
 * mchp_xec_espi_pc_acpi_pm1_set(). It registers no LPC opcode range.
 *
 * The Host I/O BAR is described on this node as io-bars and programmed by the
 * eSPI controller. The block generates no Serial IRQ of its own: SCI and SMI
 * reach the Host as virtual wires driven through the glue logic device.
 */

#define DT_DRV_COMPAT microchip_xec_espi_pc_acpi_pm1

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

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "Only one XEC eSPI peripheral channel ACPI PM1 block is supported");

/* The three Host write interrupts, in the order the node lists its interrupts,
 * its interrupt-names and its girqs.
 */
enum acpi_pm1_irq {
	ACPI_PM1_IRQ_CTL = 0,
	ACPI_PM1_IRQ_EN,
	ACPI_PM1_IRQ_STS,
	ACPI_PM1_IRQ_COUNT,
};

struct espi_pc_acpi_pm1_xec_config {
	struct acpi_pm1_regs *regs;
	const struct device *espi_dev;
	uint32_t ecia_info[ACPI_PM1_IRQ_COUNT];
};

struct espi_pc_acpi_pm1_xec_data {
	struct mchp_xec_espi_pc_cb pc_cb;
};

/* EC-side view of the register the Host wrote, indexed by
 * enum mchp_xec_espi_pc_acpi_pm1_reg.
 */
static volatile uint8_t *acpi_pm1_reg(const struct espi_pc_acpi_pm1_xec_config *cfg,
				      enum mchp_xec_espi_pc_acpi_pm1_reg reg)
{
	switch (reg) {
	case MCHP_XEC_ESPI_PC_ACPI_PM1_REG_STS1:
		return &cfg->regs->EC_STS1;
	case MCHP_XEC_ESPI_PC_ACPI_PM1_REG_STS2:
		return &cfg->regs->EC_STS2;
	case MCHP_XEC_ESPI_PC_ACPI_PM1_REG_EN1:
		return &cfg->regs->EC_EN1;
	case MCHP_XEC_ESPI_PC_ACPI_PM1_REG_EN2:
		return &cfg->regs->EC_EN2;
	case MCHP_XEC_ESPI_PC_ACPI_PM1_REG_CTRL1:
		return &cfg->regs->EC_CTRL1;
	case MCHP_XEC_ESPI_PC_ACPI_PM1_REG_CTRL2:
		return &cfg->regs->EC_CTRL2;
	case MCHP_XEC_ESPI_PC_ACPI_PM1_REG_CTRL21:
		return &cfg->regs->EC_CTRL21;
	case MCHP_XEC_ESPI_PC_ACPI_PM1_REG_CTRL22:
		return &cfg->regs->EC_CTRL22;
	case MCHP_XEC_ESPI_PC_ACPI_PM1_REG_PM_STS:
		return &cfg->regs->EC_PM_STS;
	default:
		return NULL;
	}
}

int mchp_xec_espi_pc_acpi_pm1_get(const struct device *dev,
				  enum mchp_xec_espi_pc_acpi_pm1_reg reg, uint8_t *val)
{
	volatile uint8_t *r = NULL;

	if ((dev == NULL) || (val == NULL)) {
		return -EINVAL;
	}

	r = acpi_pm1_reg(dev->config, reg);
	if (r == NULL) {
		return -EINVAL;
	}

	*val = *r;

	return 0;
}

int mchp_xec_espi_pc_acpi_pm1_set(const struct device *dev,
				  enum mchp_xec_espi_pc_acpi_pm1_reg reg, uint8_t val)
{
	volatile uint8_t *r = NULL;

	if (dev == NULL) {
		return -EINVAL;
	}

	r = acpi_pm1_reg(dev->config, reg);
	if (r == NULL) {
		return -EINVAL;
	}

	*r = val;

	return 0;
}

/* Host wrote one of the three PM1 register groups. Report which one and the
 * value it now holds.
 */
static void acpi_pm1_isr(const struct device *dev, enum acpi_pm1_irq irq,
			 enum mchp_xec_espi_pc_acpi_pm1_reg reg)
{
	const struct espi_pc_acpi_pm1_xec_config *cfg = dev->config;
	struct espi_event evt = {
		.evt_type = ESPI_BUS_PERIPHERAL_NOTIFICATION,
		.evt_details = MCHP_XEC_ESPI_PERIPHERAL_ACPI_PM1,
		.evt_data = 0U,
	};
	struct mchp_xec_espi_pc_acpi_pm1_evt *pm1_evt =
		(struct mchp_xec_espi_pc_acpi_pm1_evt *)&evt.evt_data;

	pm1_evt->reg = (uint32_t)reg;
	pm1_evt->data = *acpi_pm1_reg(cfg, reg);

	xec_pc_girq_clr(cfg->ecia_info[irq]);

	mchp_xec_espi_v3_send_callbacks(cfg->espi_dev, evt);
}

static void acpi_pm1_ctl_isr(const struct device *dev)
{
	acpi_pm1_isr(dev, ACPI_PM1_IRQ_CTL, MCHP_XEC_ESPI_PC_ACPI_PM1_REG_CTRL2);
}

static void acpi_pm1_en_isr(const struct device *dev)
{
	acpi_pm1_isr(dev, ACPI_PM1_IRQ_EN, MCHP_XEC_ESPI_PC_ACPI_PM1_REG_EN2);
}

static void acpi_pm1_sts_isr(const struct device *dev)
{
	acpi_pm1_isr(dev, ACPI_PM1_IRQ_STS, MCHP_XEC_ESPI_PC_ACPI_PM1_REG_STS2);
}

static void acpi_pm1_girq_all(const struct espi_pc_acpi_pm1_xec_config *cfg, uint8_t enable)
{
	for (int i = 0; i < ACPI_PM1_IRQ_COUNT; i++) {
		xec_pc_girq_ctrl(cfg->ecia_info[i], enable);
	}
}

static void acpi_pm1_espi_event(const struct device *espi_dev, const struct device *dev,
				enum mchp_xec_espi_pc_event evt)
{
	const struct espi_pc_acpi_pm1_xec_config *cfg = dev->config;

	ARG_UNUSED(espi_dev);

	if (xec_pc_evt_is_hw_usable(evt)) {
		/* The controller has programmed our Host I/O BAR. Discard any
		 * status latched while the block was held in reset before
		 * arming, so the application does not see a stale Host write.
		 */
		for (int i = 0; i < ACPI_PM1_IRQ_COUNT; i++) {
			xec_pc_girq_clr(cfg->ecia_info[i]);
		}

		acpi_pm1_girq_all(cfg, MCHP_MEC_ECIA_GIRQ_EN);
	} else {
		acpi_pm1_girq_all(cfg, MCHP_MEC_ECIA_GIRQ_DIS);
	}
}

static int espi_pc_acpi_pm1_xec_init(const struct device *dev)
{
	const struct espi_pc_acpi_pm1_xec_config *cfg = dev->config;
	struct espi_pc_acpi_pm1_xec_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->espi_dev)) {
		LOG_ERR("ACPI PM1: eSPI controller not ready");
		return -ENODEV;
	}

	acpi_pm1_girq_all(cfg, MCHP_MEC_ECIA_GIRQ_DIS);

	for (int i = 0; i < ACPI_PM1_IRQ_COUNT; i++) {
		xec_pc_girq_clr(cfg->ecia_info[i]);
	}

	data->pc_cb.pc_dev = dev;
	data->pc_cb.handler = acpi_pm1_espi_event;
	data->pc_cb.evt_mask = XEC_PC_EVT_MASK_ALL;

	ret = mchp_xec_espi_pc_register(cfg->espi_dev, &data->pc_cb);
	if (ret != 0) {
		LOG_ERR("ACPI PM1: eSPI registration failed (%d)", ret);
		return ret;
	}

	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, ctl, irq), DT_INST_IRQ_BY_NAME(0, ctl, priority),
		    acpi_pm1_ctl_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, ctl, irq));

	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, en, irq), DT_INST_IRQ_BY_NAME(0, en, priority),
		    acpi_pm1_en_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, en, irq));

	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, sts, irq), DT_INST_IRQ_BY_NAME(0, sts, priority),
		    acpi_pm1_sts_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, sts, irq));

	/* The GIRQ sources stay masked until the controller reports that the
	 * Host facing decoder is out of reset.
	 */

	return 0;
}

static const struct espi_pc_acpi_pm1_xec_config espi_pc_acpi_pm1_xec_cfg0 = {
	.regs = (struct acpi_pm1_regs *)DT_INST_REG_ADDR(0),
	.espi_dev = DEVICE_DT_GET(DT_INST_PARENT(0)),
	.ecia_info = {
		[ACPI_PM1_IRQ_CTL] = DT_INST_PROP_BY_IDX(0, girqs, 0),
		[ACPI_PM1_IRQ_EN] = DT_INST_PROP_BY_IDX(0, girqs, 1),
		[ACPI_PM1_IRQ_STS] = DT_INST_PROP_BY_IDX(0, girqs, 2),
	},
};

static struct espi_pc_acpi_pm1_xec_data espi_pc_acpi_pm1_xec_data0;

DEVICE_DT_INST_DEFINE(0, espi_pc_acpi_pm1_xec_init, NULL, &espi_pc_acpi_pm1_xec_data0,
		      &espi_pc_acpi_pm1_xec_cfg0, POST_KERNEL,
		      CONFIG_ESPI_XEC_V3_PC_INIT_PRIORITY, NULL);
