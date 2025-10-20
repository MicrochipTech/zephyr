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
	/* TODO */
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
