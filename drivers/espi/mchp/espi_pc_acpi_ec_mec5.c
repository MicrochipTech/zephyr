/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_espi_pc_acpi_ec

#include <zephyr/kernel.h>
#include <soc.h>
#include <errno.h>
#include <zephyr/arch/common/ffs.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi/espi_mchp_mec.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/espi/mchp_mec5_espi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include "../espi_utils.h"
#include "../espi_mchp_mec5.h"
#include "espi_mchp_pcd_regs.h"

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_retval.h>
#include <mec_pcr_api.h>
#include <mec_ecia_api.h>
#include <mec_espi_api.h>
#include <mec_acpi_ec_api.h>

LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

#define ESPI_MEC5_PC_AEC_4B_DATA_MODE 0x01u

struct espi_mec5_pc_aec_devcfg {
	uintptr_t regbase;
	const struct device *espi_dev;
	void (*irq_config_func)(void);
	uint8_t ibf_girq;
	uint8_t ibf_girq_pos;
	uint8_t obe_girq;
	uint8_t obe_girq_pos;
	uint8_t app_id;
	uint8_t flags;
};

/* Implemented a common ISR for IBF and OBE
 * IBF is set when the Host writes a byte/word to the ACPI device Host-to-EC
 * register mapped to Host address space. We clear IBF by reading the data
 * from the EC alias of the data register.
 * ACPI EC sets OBF status when it write data to the EC-to-Host data register.
 * OBF does not cause an interrupt. When the Host reads the EC-to-Host data
 * register, OBF is clear causing an OBE interrupt. We detect OBE by looking
 * at the latched OBE GIRQ status.
 */
/* TODO - We can't set evt_details[7:0] = ESPI_PERIPHERAL_HOST_IO
 * Why?
 * ESPI_PERIPHERAL_HOST_IO, ESPI_PERIPHERAL_HOST_IO_PVT, ESPI_PERIPHERAL_EC_HOST_CMD
 * require knowledge of the command bytes.
 * Instead we will use an ID value derived from ACPI_EC register base.
 * We have an APP_ID we put in MISC0 of evt_details.
 * What should be use for ESPI_PERIPHERAL_???
 */
static void espi_mec5_pc_aec_isr(const struct device *dev)
{
	const struct espi_mec5_pc_aec_devcfg *drvcfg = dev->config;
	const struct device *espi_dev = drvcfg->espi_dev;
	struct espi_mec5_drv_data *espi_data = espi_dev->data;
	struct espi_event aec_evt = {
		.evt_type = ESPI_BUS_PERIPHERAL_NOTIFICATION,
		.evt_details = ESPI_PERIPHERAL_HOST_IO,
		.evt_data = ESPI_PERIPHERAL_NODATA,
	};
	uintptr_t rb = drvcfg->regbase;
	uint32_t cd = 0, girq_result = 0;
	uint8_t status = 0;
	uint8_t size = 1u;

	if ((sys_read8(rb + MEC_AEC_BC_OFS) & BIT(MEC_AEC_BC_4BEN_POS)) != 0) {
		size = 4u;
	}

	status = sys_read8(rb + MEC_AEC_SR_OFS);
	if ((status & BIT(MEC_AEC_SR_IBF_POS)) != 0) {
		cd = sys_read32(rb + MEC_AEC_H2E_DATA0_OFS);
		aec_evt.evt_data = cd;
		if ((status & BIT(MEC_AEC_SR_CMD_POS)) != 0) {
			aec_evt.evt_details |= BIT(ESPI_EVENT_DETAILS_PC_CMD_POS);
		}
		aec_evt.evt_details |= ESPI_EVENT_DETAILS_PC_SZ(size);
		mec_hal_girq_bm_clr_src(drvcfg->ibf_girq, BIT(drvcfg->ibf_girq_pos));
	} else {
		girq_result = mec_hal_girq_result_get(drvcfg->obe_girq);
		if (girq_result & BIT(drvcfg->obe_girq_pos)) {
			mec_hal_girq_bm_clr_src(drvcfg->obe_girq, BIT(drvcfg->obe_girq_pos));
		}
	}

	aec_evt.evt_details |= ESPI_EVENT_DETAILS_PC_MISC0(drvcfg->app_id);
	espi_send_callbacks(&espi_data->callbacks, espi_dev, aec_evt);
}

static int espi_mec5_pc_aec_init(const struct device *dev)
{
	const struct espi_mec5_pc_aec_devcfg *drvcfg = dev->config;

	/* TODO init */

	if ((drvcfg->flags & ESPI_MEC5_PC_AEC_4B_DATA_MODE) != 0) {
		soc_mmcr_set_bit8(drvcfg->regbase + MEC_AEC_BC_OFS, MEC_AEC_BC_4BEN_POS);
	} else {
		soc_mmcr_clear_bit8(drvcfg->regbase + MEC_AEC_BC_OFS, MEC_AEC_BC_4BEN_POS);
	}

/*	int rc = mec_hal_acpi_ec_init(drvcfg->regbase, flags); */

	if (drvcfg->irq_config_func) {
		drvcfg->irq_config_func();
		soc_ecia_girq_ctrl(drvcfg->ibf_girq, drvcfg->ibf_girq_pos, 1u);
	}

	return 0;
}

#define MEC5_ESPI_PC_AEC_DEVICE(i)                                                                 \
	static void espi_mec5_pc_aec_irq_cfg##i(void)                                              \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, ibf, irq),                                      \
			    DT_INST_IRQ_BY_NAME(i, ibf, priority), espi_mec5_pc_aec_isr,           \
			    DEVICE_DT_INST_GET(i), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(i, ibf, irq));                                      \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, obe, irq),                                      \
			    DT_INST_IRQ_BY_NAME(i, obe, priority), espi_mec5_pc_aec_isr,           \
			    DEVICE_DT_INST_GET(i), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(i, obe, irq));                                      \
	};                                                                                         \
	const struct espi_mec5_pc_aec_devcfg espi_mec5_pc_aec_dcfg##i = {                          \
		.regbase = DT_INST_REG_ADDR(i),                                                    \
		.espi_dev = DEVICE_DT_GET(DT_INST_PARENT(i)),                                      \
		.irq_config_func = espi_mec5_pc_aec_irq_cfg##i,                                    \
		.ibf_girq = DT_INST_PROP_BY_IDX(i, girqs, 0),                                      \
		.ibf_girq_pos = DT_INST_PROP_BY_IDX(i, girqs, 1),                                  \
		.obe_girq = DT_INST_PROP_BY_IDX(i, girqs, 2),                                      \
		.obe_girq_pos = DT_INST_PROP_BY_IDX(i, girqs, 3),                                  \
		.app_id = (uint8_t)DT_INST_PROP(i, app_id),                                        \
		.flags = DT_INST_PROP_OR(i, four_byte_data_mode, 0),                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(i, espi_mec5_pc_aec_init, NULL, NULL, &espi_mec5_pc_aec_dcfg##i,     \
			      POST_KERNEL, CONFIG_ESPI_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MEC5_ESPI_PC_AEC_DEVICE)
