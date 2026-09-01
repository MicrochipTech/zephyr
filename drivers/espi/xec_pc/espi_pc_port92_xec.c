/*
 * Copyright (c) 2021 Microchip Technology Inc.
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Microchip XEC eSPI peripheral channel fast 8042 Port92 logical device.
 *
 * The V2 eSPI driver folds this block into its 8042 KBC support: init_kbc0()
 * in espi_mchp_xec_host_v2.c writes the Port92 activate register and its Host
 * I/O BAR under CONFIG_ESPI_PERIPHERAL_8042_FAST_KBC_PORT92. Here it is its
 * own driver bound to its own device tree node, so a board can decode Port92
 * without pulling in the KBC driver and vice versa.
 *
 * The block has no interrupt to the EC. It provides the legacy fast CPU reset
 * and gate A20 handshake: the Host writes port 0x92, hardware latches the two
 * bits, and firmware drives the A20 gate through the registers below. The Host
 * I/O BAR is described on this node and programmed by the eSPI controller.
 */

#define DT_DRV_COMPAT microchip_xec_espi_pc_port92

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi/mchp_xec_espi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "../espi_mchp_xec_v3.h"
#include "espi_pc_xec.h"

LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "Only one XEC eSPI peripheral channel Port92 is supported");

struct espi_pc_port92_xec_config {
	struct port92_regs *regs;
	const struct device *espi_dev;
};

struct espi_pc_port92_xec_data {
	struct mchp_xec_espi_pc_cb pc_cb;
};

static void port92_block_config(const struct device *dev)
{
	const struct espi_pc_port92_xec_config *cfg = dev->config;

	cfg->regs->ACTV = MCHP_PORT92_ACTV_ENABLE;
}

static void port92_espi_event(const struct device *espi_dev, const struct device *dev,
			      enum mchp_xec_espi_pc_event evt)
{
	ARG_UNUSED(espi_dev);

	if (xec_pc_evt_is_hw_usable(evt)) {
		/* The controller has re-programmed our Host I/O BAR. */
		port92_block_config(dev);
	}
}

static int espi_pc_port92_xec_init(const struct device *dev)
{
	const struct espi_pc_port92_xec_config *cfg = dev->config;
	struct espi_pc_port92_xec_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->espi_dev)) {
		LOG_ERR("Port92: eSPI controller not ready");
		return -ENODEV;
	}

	port92_block_config(dev);

	data->pc_cb.pc_dev = dev;
	data->pc_cb.handler = port92_espi_event;
	data->pc_cb.evt_mask = XEC_PC_EVT_MASK_HW_USABLE;

	ret = mchp_xec_espi_pc_register(cfg->espi_dev, &data->pc_cb);
	if (ret != 0) {
		LOG_ERR("Port92: eSPI registration failed (%d)", ret);
		return ret;
	}

	return 0;
}

static const struct espi_pc_port92_xec_config espi_pc_port92_xec_cfg0 = {
	.regs = (struct port92_regs *)DT_INST_REG_ADDR(0),
	.espi_dev = DEVICE_DT_GET(DT_INST_PARENT(0)),
};

static struct espi_pc_port92_xec_data espi_pc_port92_xec_data0;

DEVICE_DT_INST_DEFINE(0, espi_pc_port92_xec_init, NULL, &espi_pc_port92_xec_data0,
		      &espi_pc_port92_xec_cfg0, POST_KERNEL,
		      CONFIG_ESPI_XEC_V3_PC_INIT_PRIORITY, NULL);
