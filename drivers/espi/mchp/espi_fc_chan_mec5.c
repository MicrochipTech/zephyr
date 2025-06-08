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
#include "espi_mchp_mec5_regs.h"

/* -------- eSPI Flash Channel -------- */

static void espi_mec5_fc_isr(const struct device *dev)
{
	/*	const struct espi_mec5_drv_cfg *drvcfg = dev->config; */
	/*	mm_reg_t iob = drvcfg->ioc_base; */

	/* TODO record and clear FC status */

	sys_write32(BIT(ESPI_GIRQ_FC_POS), ESPI_GIRQ_STS_ADDR);

	/* TODO */
}

void espi_mec5_fc_irq_connect(const struct device *dev)
{
	/*	const struct espi_mec5_drv_cfg *drvcfg = dev->config; */
	/*	mm_reg_t iob = drvcfg->ioc_base; */

	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, fc, irq), DT_INST_IRQ_BY_NAME(0, fc, priority),
		    espi_mec5_fc_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, fc, irq));

	sys_write32(BIT(ESPI_GIRQ_FC_POS), ESPI_GIRQ_ENSET_ADDR);
}

int espi_mec5_flash_read_api(const struct device *dev, struct espi_flash_packet *pckt)
{
	LOG_INF("MEC5 flash read: dev = %p pckt = %p", dev, pckt);
	return 0;
}

int espi_mec5_flash_write_api(const struct device *dev, struct espi_flash_packet *pckt)
{
	LOG_INF("MEC5 flash write: dev = %p pckt = %p", dev, pckt);
	return 0;
}

int espi_mec5_flash_erase_api(const struct device *dev, struct espi_flash_packet *pckt)
{
	LOG_INF("MEC5 flash erase: dev = %p pckt = %p", dev, pckt);
	return 0;
}
