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

/* -------- Peripheral Channel -------- */
#define PC_MBOX0_NODE DT_NODELABEL(mbox0)
#define PC_KBC0_NODE DT_NODELABEL(kbc0)
#define PC_AEC0_NODE DT_NODELABEL(acpi_ec0)
#define PC_AEC1_NODE DT_NODELABEL(acpi_ec1)
#define PC_AEC2_NODE DT_NODELABEL(acpi_ec2)
#define PC_AEC3_NODE DT_NODELABEL(acpi_ec3)
#define PC_AEC4_NODE DT_NODELABEL(acpi_ec4)
#define PC_GLUE_NODE DT_NODELABEL(glue)
#define PC_EMI0_NODE DT_NODELABEL(emi0)
#define PC_EMI1_NODE DT_NODELABEL(emi1)
#define PC_EMI2_NODE DT_NODELABEL(emi2)
#define PC_BDP0_NODE DT_NODELABEL(p80bd0)

#define ESPI_MEC5_PC_DEV_PATH DT_PATH(mchp_mec5_espi_pc_host_dev)

/* Generate PC device table */
#define HAS_HOST_ADDR_PROP(node_id) DT_NODE_HAS_PROP(node_id, host_address)

struct espi_mec5_pc_device {
	uint32_t haddr_lsw;
	uint8_t sirq0_idx;
	uint8_t sirq0_slot;
	uint8_t sirq1_idx;
	uint8_t sirq1_slot;
	uint8_t ldn;
	uint8_t flags;
};

#define ESPI_MEC5_PC_DEV_BAR_SIRQ(node_id) \
	{ \
		.haddr_lsw = DT_PROP(node_id, host_address), \
		.ldn = DT_PROP(node_id, ldn), \
	},

/* PC device BAR and Serial-IRQ table */
const struct espi_mec5_pc_device espi_mec5_pc_devtbl[] = {
	DT_FOREACH_CHILD_STATUS_OKAY(ESPI_MEC5_PC_DEV_PATH, ESPI_MEC5_PC_DEV_BAR_SIRQ)
};

static int espi_mec5_pc_config_bars(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *devcfg = dev->config;
	struct mec_espi_io_regs *ioregs = (struct mec_espi_io_regs *)devcfg->ioc_base;
	struct mec_espi_mem_regs *mregs = (struct mec_espi_mem_regs *)devcfg->memc_base;
	int ret = 0;

	for (size_t n = 0; n < ARRAY_SIZE(espi_mec5_pc_devtbl); n++) {
		const struct espi_mec5_pc_device *p = &espi_mec5_pc_devtbl[n];

		if (p->haddr_lsw > UINT16_MAX) {
			ret = mec_hal_espi_mbar_cfg(mregs, p->ldn, p->haddr_lsw, 1u);
		} else {
			ret =  mec_hal_espi_iobar_cfg(ioregs, p->ldn,
						      (uint16_t)p->haddr_lsw & UINT16_MAX, 1);
		}
	}

	return 0;
}

/* ISR for Peripheral Channel events:
 * Channel enable change by Host
 * Bus Master enable change by Host
 * PC cycle errors
 */
static void espi_mec5_pc_isr(const struct device *dev)
{
	/* TODO */
}

void espi_mec5_pc_irq_connect(const struct device *dev)
{
/*	const struct espi_mec5_drv_cfg *drvcfg = dev->config; */
/*	mm_reg_t iob = drvcfg->ioc_base; */
	uint32_t pc_ien_msk = BIT(ESPI_GIRQ_PC_POS);

	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, pc, irq),
		DT_INST_IRQ_BY_NAME(0, pc, priority),
		espi_mec5_pc_isr,
		DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, pc, irq));
	/* TODO mec_hal_espi_pc_girq_ctrl(1); */

	sys_write32(pc_ien_msk, ESPI_GIRQ_ENSET_ADDR);
}

int espi_mec5_pc_pltrst_handler(const struct device *dev, uint8_t pltrst_state)
{
	int ret1 = espi_mec5_pc_config_bars(dev);

	/* TODO Serial-IRQs */

	return ret1;
}

int espi_mec5_read_req_api(const struct device *dev, struct espi_request_packet *req)
{
	return -ENOTSUP;
}

int espi_mec5_write_req_api(const struct device *dev, struct espi_request_packet *req)
{
	return -ENOTSUP;
}

int espi_mec5_read_lpc_req_api(const struct device *dev, enum lpc_peripheral_opcode op,
				uint32_t *data)
{
	return -ENOTSUP;
}

int espi_mec5_write_lpc_req_api(const struct device *dev, enum lpc_peripheral_opcode op,
				uint32_t *data)
{
	return -ENOTSUP;
}
