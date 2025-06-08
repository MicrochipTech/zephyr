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

/* -------- eSPI Virtual Wire Channel -------- */

/* eSPI virtual wire table entry
 * signal is a enum espi_vwire_signal from espi.h, we hope this enum remains
 * zero based and actual numeric values do not exceed 255.
 * host_idx is the Host Index containing this vwire as defined in the eSPI specification.
 * source-bit is the bit postion [0:3] in the host index and MEC5 HW.
 * reg_idx is the index of the MEC5 vwire register group for this Host Index
 * flags indicate the group is Controller-to-Target or Target-to-Controller, reset source,
 * interrupt detection, etc.
 * MEC5 eSPI hardware support 11 controller-to-target and 11 target-to-controller VWire groups.
 * Each VWire group contains 4 wires. Maximum supported number of VWires is 88.
 */
BUILD_ASSERT(((ESPI_VWIRE_SIGNAL_SLP_S3) == 0), "VWires in espi header have been renumbered!!!");
BUILD_ASSERT(((ESPI_VWIRE_SIGNAL_COUNT) <= 88), "Max HW VWire count exceeded!!!");

struct espi_mec5_vwire {
	uint8_t host_idx;
	uint8_t source_bit;
	uint8_t reg_idx;
	uint8_t flags;
};

/* DT macros used to generate the tables of Controller-to-Target and
 * Target-to-Controller virutal wires enabled on the platform.
 * Table entries depend upon device tree configuration of individual
 * virtual wires and groups.
 */

#define MEC5_DT_ESPI_CT_VWIRES_NODE DT_PATH(mchp_mec5_espi_ct_vwires)
#define MEC5_DT_ESPI_TC_VWIRES_NODE DT_PATH(mchp_mec5_espi_tc_vwires)

#define MCHP_DT_ESPI_CTVW_BY_NAME(name) DT_CHILD(MEC5_DT_ESPI_CT_VWIRES_NODE, name)
#define MCHP_DT_ESPI_TCVW_BY_NAME(name) DT_CHILD(MEC5_DT_ESPI_TC_VWIRES_NODE, name)

#define MEC5_VW_SIGNAL(node_id)     DT_STRING_UPPER_TOKEN(node_id, vw_name)
#define MEC5_VW_SOURCE_BIT(node_id) DT_PROP(node_id, source_bit)

#define MEC5_VW_HOST_IDX(node_id)   DT_PROP_BY_PHANDLE(node_id, vw_group, host_index)
#define MEC5_VW_HW_REG_IDX(node_id) DT_PROP_BY_PHANDLE(node_id, vw_group, vw_reg)

#define MEC5_VW_RST_SRC(node_id) DT_ENUM_IDX(DT_PHANDLE(node_id, vw_group), reset_source)

#define MEC5_VW_CT_FLAGS(node_id)                                                                  \
	((DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node_id, vw_group, 0), direction) & 0x1) |                 \
	 ((DT_PROP(node_id, reset_state) & 0x1) << 1) | ((MEC5_VW_RST_SRC(node_id) & 0x3) << 2) |  \
	 ((DT_ENUM_IDX_OR(node_id, irq_sel, 0) & 0x7) << 4))

#define MEC5_VW_TC_FLAGS(node_id)                                                                  \
	((DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node_id, vw_group, 0), direction) & 0x1) |                 \
	 ((DT_PROP(node_id, reset_state) & 0x1) << 1) | ((MEC5_VW_RST_SRC(node_id) & 0x3) << 2))

#define MEC5_ESPI_CTVW_ENTRY(node_id)                                                              \
	[(MEC5_VW_SIGNAL(node_id))] = {                                                            \
		.host_idx = MEC5_VW_HOST_IDX(node_id),                                             \
		.source_bit = MEC5_VW_SOURCE_BIT(node_id),                                         \
		.reg_idx = MEC5_VW_HW_REG_IDX(node_id),                                            \
		.flags = MEC5_VW_CT_FLAGS(node_id),                                                \
	},

#define MEC5_ESPI_TCVW_ENTRY(node_id)                                                              \
	[(MEC5_VW_SIGNAL(node_id))] = {                                                            \
		.host_idx = MEC5_VW_HOST_IDX(node_id),                                             \
		.source_bit = MEC5_VW_SOURCE_BIT(node_id),                                         \
		.reg_idx = MEC5_VW_HW_REG_IDX(node_id),                                            \
		.flags = MEC5_VW_TC_FLAGS(node_id),                                                \
	},

#define MEC5_ESPI_CTVW_FLAGS_DIR(x)       ((uint8_t)(x) & 0x01u)
#define MEC5_ESPI_CTVW_FLAGS_RST_STATE(x) (((uint8_t)(x) & 0x02u) >> 1)
#define MEC5_ESPI_CTVW_FLAGS_RST_SRC(x)   (((uint8_t)(x) & 0x0cu) >> 2)
#define MEC5_ESPI_CTVW_FLAGS_IRQSEL(x)    (((uint8_t)(x) & 0x70u) >> 4)

#define MEC5_ESPI_VW_FLAGS_DIR_IS_TC(x) ((uint8_t)(x) & 0x01u)

static const struct espi_mec5_vwire espi_mec5_vw_tbl[] = {
	DT_FOREACH_CHILD_STATUS_OKAY(MEC5_DT_ESPI_CT_VWIRES_NODE, MEC5_ESPI_CTVW_ENTRY)
		DT_FOREACH_CHILD_STATUS_OKAY(MEC5_DT_ESPI_TC_VWIRES_NODE, MEC5_ESPI_TCVW_ENTRY)};

/* Create a look up table for the Controller-to-Target VWire interrupt handler.
 * Hardware has 11 C2T VWire registers. Each register controls a group of 4 vwires.
 * The array is indexed by the (vw_reg_index * 4) + vwire_source_position
 * where vw_reg_index in [0, 10] and vwire_source_position in [0, 3]
 * Each array value is the enum espi_vwire_signal stored an an uint8_t.
 * The eSPI specification allows a maximum of 256 virtual wires. Some of these are
 * dedicated for serial IRQ.
 */
#define MEC5_ESPI_CTVW_ISR_IDX(node_id)                                                            \
	(((uint32_t)MEC5_VW_HW_REG_IDX(node_id) * 4u) + MEC5_VW_SOURCE_BIT(node_id))

#define MEC5_ESPI_CTVW_ISR_ENTRY(node_id)                                                          \
	[MEC5_ESPI_CTVW_ISR_IDX(node_id)] = (uint8_t)(MEC5_VW_SIGNAL(node_id) & 0xffu),

static const uint8_t __aligned(4) espi_mec5_vw_ct_isr_tbl[] = {
	DT_FOREACH_CHILD_STATUS_OKAY(MEC5_DT_ESPI_CT_VWIRES_NODE, MEC5_ESPI_CTVW_ISR_ENTRY)};

/* Configure VW registers
 * H2T VWire register is 96-bits
 * T2H VWire register is 64-bits
 * word 0 of both are the same
 * word 1 of H2T contains IRQ_SEL fields
 * word 1 of T2H and word 2 of H2T contain the state of each of the 4 vwires in the group
 */
static int config_vw(mm_reg_t vwbase, const struct espi_mec5_vwire *vw)
{
	mm_reg_t raddr = 0;
	uint32_t r = 0, temp = 0;

	if ((vw == NULL) || (vw->reg_idx >= ESPI_VW_NGRPS) || (vw->source_bit > 3u)) {
		return -EINVAL;
	}

	raddr = vwbase + ESPI_HTVW_GRPW(vw->reg_idx, 0);

	r = sys_read32(raddr) & (uint32_t)~(ESPI_VW_W0_HI_MSK);
	r |= (uint32_t)vw->host_idx << ESPI_VW_W0_HI_POS;
	temp = ((vw->flags >> 2) & 0x3u); /* reset source */
	r |= ((temp << ESPI_VW_W0_RSRC_POS) & ESPI_VW_W0_RSRC_MSK);
	temp = (uint32_t)((vw->flags >> 1) & 0x1u) << vw->source_bit; /* reset state */
	r |= temp;
	sys_write32(r, raddr);

	if (vw->flags & BIT(0)) { /* H2T? */
		/* configure IRQ_SEL */
		raddr += 4u;
		r = sys_read32(raddr);
		r &= (uint32_t)~(ESPI_VW_H2T_W1_ISEL_MSK(vw->source_bit));
		temp = MEC5_ESPI_CTVW_FLAGS_IRQSEL(vw->flags);
		r |= (temp << ESPI_VW_H2T_W1_ISEL_POS(vw->source_bit));
		sys_write32(r, raddr);
	}

	return 0;
}

/* Initialize MEC5 eSPI target virtual wire registers static configuration
 * set by DT. Configuration which is not changed by ESPI_nRESET or nPLTRST.
 */
int espi_mec5_init_vwires(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t vwbase = (mm_reg_t)drvcfg->vwc_base;
	int rc1 = 0, rc2 = 0;

	for (size_t n = 0; n < ARRAY_SIZE(espi_mec5_vw_tbl); n++) {
		const struct espi_mec5_vwire *vw = &espi_mec5_vw_tbl[n];

		rc1 = config_vw(vwbase, vw);
		if (rc1) {
			rc2 = rc1;
			LOG_ERR("VWire HI=0x%02x src=%u failed", vw->host_idx, vw->source_bit);
		}
	}

	return rc2;
}

/* -------- eSPI driver VWire API -------- */
int espi_mec5_send_vwire_api(const struct device *dev, enum espi_vwire_signal vw, uint8_t level)
{
	return -ENOTSUP;
}

int espi_mec5_recv_vwire_api(const struct device *dev, enum espi_vwire_signal vw, uint8_t *level)
{
	return -ENOTSUP;
}

static void espi_mec5_htvw_common_isr(const struct device *dev, uint32_t vgbase, uint8_t vwbank)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t vwb = drvcfg->vwc_base;
	uint32_t idx = 0, result = 0, vwidx = 0, vwpos = 0;
	int signal = 0;
	unsigned int pos = 0;
	struct espi_event evt = {ESPI_BUS_EVENT_VWIRE_RECEIVED, 0, 0};

	result = sys_read32(vgbase + ESPI_GIRQ_RESULT_OFS);

	pos = find_lsb_set(result);
	if (pos == 0) {
		LOG_ERR("HTVW no ISR result bit!");
		return;
	}

	/* clear latched GIRQ status */
	--pos;
	sys_write32(BIT(pos), vgbase + ESPI_GIRQ_STS_OFS);

	/* compute VW register group index and bit [0, 3] in group */
	vwidx = pos / 4u;
	vwpos = pos % 4u;
	if (vwbank != 0) {
		vwidx += 7u;
	}

	if ((sys_read32(vwb + ESPI_HTVW_GRPW(vwidx, 2u)) & BIT(vwpos * 8u)) != 0) {
		evt.evt_data = 1u;
	}

	idx = (vwidx * 4u) + vwpos;
	signal = (int)espi_mec5_vw_ct_isr_tbl[idx];
	evt.evt_details = (uint32_t)signal;

	/* TODO special handling */
	if (signal == ESPI_VWIRE_SIGNAL_PLTRST) {
		if (evt.evt_data != 0) {
			espi_mec5_pc_pltrst_handler(dev, 1u);
		}
	}

	espi_send_callbacks(&data->callbacks, dev, evt);
}

static void espi_mec5_htvw_0_6_isr(const struct device *dev)
{
	espi_mec5_htvw_common_isr(dev, ESPI_GIRQ_VWB0_BASE, 0);
}

static void espi_mec5_htvw_7_10_isr(const struct device *dev)
{
	espi_mec5_htvw_common_isr(dev, ESPI_GIRQ_VWB1_BASE, 1);
}

/* Virtual wire channel enable change interrupt handler
 * VWire channel enable bit set/cleared by the Host is connected directly to GIRQ19 status
 * causing and is active high. The interrupt must be disabled when the channel is enabled.
 */
static void espi_mec5_vw_chen_isr(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t iob = drvcfg->ioc_base;
	struct espi_event evt = {.evt_type = ESPI_BUS_EVENT_CHANNEL_READY,
				 .evt_details = ESPI_CHANNEL_VWIRE,
				 .evt_data = 0};

	sys_write32(BIT(ESPI_GIRQ_VW_CHEN_POS), ESPI_GIRQ_ENCLR_ADDR);
	sys_write32(BIT(ESPI_GIRQ_VW_CHEN_POS), ESPI_GIRQ_STS_ADDR);

	if ((sys_read32(iob + ESPI_OFR_RST) & BIT(ESPI_VW_SR_CHEN_POS)) != 0) {
		sys_set_bit(iob + ESPI_VWC_TAF, ESPI_VW_RDY_POS);
		evt.evt_data = 1u;
	} else {
		sys_write32(BIT(ESPI_GIRQ_VW_CHEN_POS), ESPI_GIRQ_ENSET_ADDR);
	}

	espi_send_callbacks(&data->callbacks, dev, evt);
}

void espi_mec5_vw_irq_connect(const struct device *dev)
{
	uint32_t gvwb0 = ESPI_GIRQ_VWB0_BASE + ESPI_GIRQ_ENSET_OFS;
	uint32_t gvwb1 = ESPI_GIRQ_VWB1_BASE + ESPI_GIRQ_ENSET_OFS;

	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, vw_chan_en, irq),
		    DT_INST_IRQ_BY_NAME(0, vw_chan_en, priority), espi_mec5_vw_chen_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, vw_chan_en, irq));
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, vwct_0_6, irq),
		    DT_INST_IRQ_BY_NAME(0, vwct_0_6, priority), espi_mec5_htvw_0_6_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, vwct_0_6, irq));
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, vwct_7_10, irq),
		    DT_INST_IRQ_BY_NAME(0, vwct_7_10, priority), espi_mec5_htvw_7_10_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, vwct_7_10, irq));
	sys_write32(BIT(ESPI_GIRQ_VW_CHEN_POS), ESPI_GIRQ_ENSET_ADDR);
	sys_write32(UINT32_MAX, gvwb0);
	sys_write32(UINT32_MAX, gvwb1);
}
