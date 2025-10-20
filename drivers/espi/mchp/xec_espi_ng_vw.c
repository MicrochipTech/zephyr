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
#include "../espi_utils.h"
#include "../espi_mchp_xec_ng.h"
#include "mchp_xec_espi_regs.h"

LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

#define XEC_ESPI_NG_H2T_VW_NODE DT_PATH(mchp_xec_espi_ng_h2t_vwires)
#define XEC_ESPI_NG_T2H_VW_NODE DT_PATH(mchp_xec_espi_ng_t2h_vwires)
#define XEC_ESPI_NG_H2T_VW_MAP_NODE DT_PATH(mchp_xec_espi_ng_h2t_vw_map)
#define XEC_ESPI_NG_T2H_VW_MAP_NODE DT_PATH(mchp_xec_espi_ng_t2h_vw_map)

/* from xec-espi-ng-vw-groups.dtsi */
#define XEC_ESPI_NG_H2T_VWG_NODE DT_PATH(mchp_xec_espi_ng_h2t_vw_groups)
#define XEC_ESPI_NG_T2H_VWG_NODE DT_PATH(mchp_xec_espi_ng_t2h_vw_groups)

struct xec_espi_ng_vw {
	uint8_t host_idx;
	uint8_t vw_reg_num;
	uint8_t src_pos;
	uint8_t reset_state;
	uint8_t reset_src;
	uint8_t irq_sel;
	uint8_t dir;
	uint8_t rsvd7;
};

#define XEC_ESPI_NG_VW_SIGNAL(nid) DT_STRING_UPPER_TOKEN(nid, vw_name)

#define XEC_ESPI_NG_VW_HOST_IDX(nid) DT_PROP_BY_PHANDLE(nid, vw_group, host_index)
#define XEC_ESPI_NG_VW_REG_IDX(nid) DT_PROP_BY_PHANDLE(nid, vw_group, vw_reg)
#define XEC_ESPI_NG_VW_RST_SRC(nid) DT_ENUM_IDX(DT_PHANDLE(nid, vw_group), reset_source)
#define XEC_ESPI_NG_VW_DIR(nid) DT_ENUM_IDX(DT_PHANDLE_BY_IDX(nid, vw_group, 0), direction)

#define XEC_ESPI_NG_VW(nid) \
	[XEC_ESPI_NG_VW_SIGNAL(nid)] = { \
		.host_idx = (uint8_t)XEC_ESPI_NG_VW_HOST_IDX(nid), \
		.vw_reg_num = (uint8_t)XEC_ESPI_NG_VW_REG_IDX(nid), \
		.src_pos = (uint8_t)DT_PROP(nid, source_bit), \
		.reset_state = (uint8_t)DT_PROP(nid, reset_state), \
		.reset_src = XEC_ESPI_NG_VW_RST_SRC(nid), \
		.irq_sel = (uint8_t)DT_ENUM_IDX(nid, irq_sel), \
		.dir = XEC_ESPI_NG_VW_DIR(nid), \
	},

static const struct xec_espi_ng_vw xec_espi_ng_vw_tbl[] = {
	DT_FOREACH_CHILD(XEC_ESPI_NG_H2T_VW_NODE, XEC_ESPI_NG_VW)
	DT_FOREACH_CHILD(XEC_ESPI_NG_T2H_VW_NODE, XEC_ESPI_NG_VW)
};

#if 1
struct xec_espi_ng_vw_grp {
	uint8_t host_idx;
	uint8_t dir;
	uint8_t vw_reg_idx;
	uint8_t vw_mask;
	uint8_t reset_src;
	uint8_t reset_state;
	uint8_t rsvd[2];
	uint8_t irq_selects[4];
	uint8_t src_enum_vals[4];
};

#define XEC_ESPI_NG_VWG_SN_FULL(nid, idx) \
	DT_CAT(ESPI_VWIRE_SIGNAL_, DT_PROP_BY_IDX(nid, source_names, idx))

/* #define XEC_ESPI_NG_VWG_SIGNAL(nid, idx) DT_STRING_UPPER_TOKEN(nid, source_names) */

#define XEC_ESPI_NG_H2T_VWGRP(nid) \
	{ \
		.host_idx = DT_PROP(nid, host_index), \
		.dir = DT_ENUM_IDX(nid, direction), \
		.vw_reg_idx = DT_PROP(nid, vw_reg_idx), \
		.vw_mask = DT_PROP(nid, vw_mask), \
		.reset_src = DT_ENUM_IDX(nid, reset_source), \
		.reset_state = DT_PROP(nid, reset_state), \
		.irq_selects = { \
			DT_PROP_BY_IDX(nid, irq_selects, 0), \
			DT_PROP_BY_IDX(nid, irq_selects, 1), \
			DT_PROP_BY_IDX(nid, irq_selects, 2), \
			DT_PROP_BY_IDX(nid, irq_selects, 3), \
		}, \
		.src_enum_vals = { \
			DT_ENUM_IDX_BY_IDX(nid, source_names, 0), \
			DT_ENUM_IDX_BY_IDX(nid, source_names, 1), \
			DT_ENUM_IDX_BY_IDX(nid, source_names, 2), \
			DT_ENUM_IDX_BY_IDX(nid, source_names, 3), \
		}, \
	},

#define XEC_ESPI_NG_T2H_VWGRP(nid) \
	{ \
		.host_idx = DT_PROP(nid, host_index), \
		.dir = DT_ENUM_IDX(nid, direction), \
		.vw_reg_idx = DT_PROP(nid, vw_reg_idx), \
		.vw_mask = DT_PROP(nid, vw_mask), \
		.reset_src = DT_ENUM_IDX(nid, reset_source), \
		.reset_state = DT_PROP(nid, reset_state), \
		.src_enum_vals = { \
			DT_ENUM_IDX_BY_IDX(nid, source_names, 0), \
			DT_ENUM_IDX_BY_IDX(nid, source_names, 1), \
			DT_ENUM_IDX_BY_IDX(nid, source_names, 2), \
			DT_ENUM_IDX_BY_IDX(nid, source_names, 3), \
		}, \
	},

static const struct xec_espi_ng_vw_grp xec_espi_ng_vwgrp_tbl[] __aligned(8) = {
	DT_FOREACH_CHILD_STATUS_OKAY(XEC_ESPI_NG_H2T_VWG_NODE, XEC_ESPI_NG_H2T_VWGRP)
	DT_FOREACH_CHILD_STATUS_OKAY(XEC_ESPI_NG_H2T_VWG_NODE, XEC_ESPI_NG_T2H_VWGRP)
};

const struct xec_espi_ng_vw_grp *xec_espi_ng_find_vw_group(uint8_t host_idx)
{
	for (size_t i = 0; i < ARRAY_SIZE(xec_espi_ng_vwgrp_tbl); i++) {
		const struct xec_espi_ng_vw_grp *p = &xec_espi_ng_vwgrp_tbl[i];

		if (p->host_idx == host_idx) {
			return p;
		}
	}

	return NULL;
}
#endif

static const struct xec_espi_ng_vw *find_vw(uint32_t vw_enum_val)
{
	if (vw_enum_val >= ARRAY_SIZE(xec_espi_ng_vw_tbl)) {
		return NULL;
	}

	return &xec_espi_ng_vw_tbl[vw_enum_val];
}

/* Configure IRQ detection of Host-to-Target VWires
 * H2T VWire IRQ_SELECT field is reset on chip reset not eSPI platform reset.
 * Host-to-Target VWires:
 *  host index
 *  reset sources
 *  reset states - manually load into sources?
 *  TODO
 */
int xec_espi_ng_vw_init1(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mem_addr_t vb = devcfg->vwbase;
	uint32_t ofs = 0, v = 0;

	for (size_t i = 0; i < ARRAY_SIZE(xec_espi_ng_vw_tbl); i++) {
		const struct xec_espi_ng_vw *p = &xec_espi_ng_vw_tbl[i];

		if (p->dir != 0) {
			continue;
		}

		ofs = XEC_ESPI_VW_HT_GRPW(p->vw_reg_num, 1);

		v = sys_read32(vb + ofs);
		v &= (uint32_t)~XEC_ESPI_VW_H2T_W1_ISEL_MSK(p->src_pos);
		v |= XEC_ESPI_VW_H2T_W1_ISEL_SET(p->src_pos, (uint32_t)p->irq_sel);
		sys_write32(v, vb + ofs);
	}

	return 0;
}

/* Configure reset source and reset state for VWires.
 * VWire registers are always reset on chip reset. An additional reset event
 * can be configured for each Virtal Wire.
 */
int xec_espi_ng_vw_init2(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mem_addr_t vb = devcfg->vwbase;
	uint32_t ofs = 0, v = 0;

	for (size_t i = 0; i < ARRAY_SIZE(xec_espi_ng_vw_tbl); i++) {
		const struct xec_espi_ng_vw *p = &xec_espi_ng_vw_tbl[i];

		if (p->dir == 0) {
			ofs = XEC_ESPI_VW_HT_GRPW(p->vw_reg_num, 0);

			v = sys_read32(vb + ofs);
			v &= (uint32_t)~XEC_ESPI_VW_H2T_W1_ISEL_MSK(p->src_pos);
			v |= XEC_ESPI_VW_H2T_W1_ISEL_SET(p->src_pos, (uint32_t)p->irq_sel);
			sys_write32(v, vb + ofs);
		} else {
			ofs = XEC_ESPI_VW_TH_GRPW(p->vw_reg_num, 0);
		}
	}

	return 0;
}

int xec_espi_ng_vw_send_api(const struct device *dev, enum espi_vwire_signal vw, uint8_t level)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mem_addr_t vb = devcfg->vwbase;
	const struct xec_espi_ng_vw *p = find_vw(vw);
	uint32_t ofs = 0;
	uint8_t pos = 0;
#if 1
	const struct xec_espi_ng_vw_grp *pg = xec_espi_ng_find_vw_group(0x41);

	if (pg == NULL) {
		return -ENODEV;
	}
#endif
	/* unknown VWire OR VWire is Host-to-Target */
	if ((p == NULL) || (p->dir == 0)) {
		return -EINVAL;
	}

	ofs = XEC_ESPI_VW_TH_GRPW(p->vw_reg_num, 1);
	pos = XEC_ESPI_VW_STATE_POS(p->src_pos);

	sys_set_bit(vb + ofs, pos);

#ifdef CONFIG_ESPI_VWIRE_T2H_POLL_HOST_READ_US
	uint32_t poll_time_us = CONFIG_ESPI_VWIRE_T2H_POLL_HOST_READ_US;

	ofs = XEC_ESPI_VW_TH_GRPW(p->vw_reg_num, 1);
	pos = XEC_ESPI_VW_T2H_W0_CHG_POS(p->src_pos);

	while (sys_test_bit(vb + ofs, pos) != 0) {
		if (poll_time_us == 0) {
			break;
		}
		k_busy_wait(1);
		poll_time_us--;
	}
#endif

	return 0;
}

int xec_espi_ng_vw_recv_api(const struct device *dev, enum espi_vwire_signal vw, uint8_t *level)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mem_addr_t vb = devcfg->vwbase;
	const struct xec_espi_ng_vw *p = find_vw(vw);
	uint32_t ofs = 0;
	uint8_t pos = 0;

	/* level is NULL OR unknown VWire OR VWire is Target-to-Host */
	if ((level == NULL) || (p == NULL) || (p->dir == 1)) {
		return -EINVAL;
	}

	ofs = XEC_ESPI_VW_HT_GRPW(p->vw_reg_num, 2);
	pos = XEC_ESPI_VW_STATE_POS(p->src_pos);

	if (sys_test_bit(vb + ofs, pos) != 0) {
		*level = 1u;
	} else {
		*level = 0u;
	}

	return 0;
}

#ifdef CONFIG_ESPI_AUTOMATIC_BOOT_DONE_ACKNOWLEDGE
static void set_boot_done_vwires(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mem_addr_t vb = devcfg->vwbase;
	const struct xec_espi_ng_vw *vwbs = find_vw(ESPI_VWIRE_SIGNAL_TARGET_BOOT_STS);
	const struct xec_espi_ng_vw *vwbd = find_vw(ESPI_VWIRE_SIGNAL_TARGET_BOOT_DONE);
	uint32_t ofs = 0;
	uint8_t pos = 0;

	if ((vwbs == NULL) || (vwbd == NULL)) {
		return;
	}

	ofs = XEC_ESPI_VW_TH_GRPW(vwbs->vw_reg_num, 1);
	pos = XEC_ESPI_VW_STATE_POS(vwbs->src_pos);

	sys_set_bit(vb + ofs, pos);

	ofs = XEC_ESPI_VW_TH_GRPW(vwbd->vw_reg_num, 1);
	pos = XEC_ESPI_VW_STATE_POS(vwbd->src_pos);

	sys_set_bit(vb + ofs, pos);
}
#endif

/* VWire channel interrupt handlers called from core driver */
void xec_espi_ng_vw_chen_handler(const struct device *dev)
{
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	struct xec_espi_ng_data *data = dev->data;
	mem_addr_t iob = devcfg->base;
	struct espi_event ev = {
		.evt_type = ESPI_BUS_EVENT_CHANNEL_READY,
		.evt_details = ESPI_CHANNEL_VWIRE,
		.evt_data = 0,
	};

	if (soc_test_bit8(iob + XEC_ESPI_VW_SR_OFS, XEC_ESPI_VW_SR_CHEN_POS) != 0) {
		/* VW channel enable is a level we must disable the interrupt */
		soc_ecia_girq_ctrl(XEC_ESPI_GIRQ, XEC_ESPI_GIRQ_VW_CHEN_POS, 0);
		/* set VW channel ready */
		ev.evt_data = 1u;
		soc_set_bit8(iob + XEC_ESPI_VW_RDY_OFS, XEC_ESPI_CHAN_RDY_POS);
#ifdef CONFIG_ESPI_AUTOMATIC_BOOT_DONE_ACKNOWLEDGE
		set_boot_done_vwires(dev);
#endif
	}

	soc_ecia_girq_status_clear(XEC_ESPI_GIRQ, XEC_ESPI_GIRQ_VW_CHEN_POS);

	espi_send_callbacks(&data->cbs, dev, ev);
}

const struct xec_espi_ng_vw *find_h2t_vw_by_girq(uint8_t girq_pos, enum espi_vwire_signal *signal)
{
	uint8_t grp_num = girq_pos / 4u;
	uint8_t vw_pos = girq_pos % 4u;

	for (size_t n = 0; n < ARRAY_SIZE(xec_espi_ng_vw_tbl); n++) {
		const struct xec_espi_ng_vw *p = &xec_espi_ng_vw_tbl[n];

		if (p->dir != 0) {
			continue;
		}

		if ((p->vw_reg_num == grp_num) && (p->src_pos == vw_pos)) {
			if (signal != NULL) {
				*signal = (enum espi_vwire_signal)n;
			}

			return p;
		}
	}

	return NULL;
}

/* Host-to-Target VWire groups 0 - 7
 * Each group contains 4 VWires per eSPI specification.
 * XEC eSPI VWire HW does not expose eSPI VWire protocol valid bits.
 * XEC HW maintains internal current state and when PUT_VW packet arrives
 * it updates the state of each VW group in the PUT_VW packet.
 * For each group, HW only makes changes to those VWires with valid = 1.
 * HW maps each VWire in a group to an individual GIRQ source bit. We must find
 * the struct xec_espi_ng_vw corresponding to each GIRQ result bit that is set.
 */
void xec_espi_ng_vw_bank0_kworker(struct k_work *work)
{
	struct xec_espi_ng_data *data = CONTAINER_OF(work, struct xec_espi_ng_data, kwq_vwb0);
	const struct device *dev = data->dev;
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mem_addr_t vb = devcfg->vwbase;
	struct espi_event ev = {0};
	uint32_t result = 0, ofs = 0, pos = 0, vw_pos = 0;
	enum espi_vwire_signal signal = 0;

	soc_ecia_girq_result(XEC_ESPI_GIRQ_VW_BANK0, &result);

	while ((result != 0) && (pos < 24u)) {
		if ((result & BIT(pos)) != 0) {
			result &= (uint32_t)~BIT(pos);
			soc_ecia_girq_status_clear(XEC_ESPI_GIRQ_VW_BANK0, pos);

			const struct xec_espi_ng_vw *p = find_h2t_vw_by_girq(pos, &signal);

			if (p != NULL) {
				ev.evt_type = ESPI_BUS_EVENT_VWIRE_RECEIVED;
				ev.evt_details = signal;
				ev.evt_data = 0;

				ofs = XEC_ESPI_VW_HT_GRPW(p->vw_reg_num, 2);
				vw_pos = XEC_ESPI_VW_STATE_POS(p->src_pos);

				if (sys_test_bit(vb + ofs, vw_pos) != 0) {
					ev.evt_data = 1u;
				}

				espi_send_callbacks(&data->cbs, dev, ev);
			}
		}

		pos++;
	}

	irq_enable(devcfg->irqn_vwb0);
}

void xec_espi_ng_vw_bank1_kworker(struct k_work *work)
{
	struct xec_espi_ng_data *data = CONTAINER_OF(work, struct xec_espi_ng_data, kwq_vwb1);
	const struct device *dev = data->dev;
	const struct xec_espi_ng_drvcfg *devcfg = dev->config;
	mem_addr_t vb = devcfg->vwbase;
	struct espi_event ev = {0};
	uint32_t result = 0, ofs = 0, pos = 0, vw_pos = 0;
	enum espi_vwire_signal signal = 0;

	soc_ecia_girq_result(XEC_ESPI_GIRQ_VW_BANK1, &result);

	while ((result != 0) && (pos < 16u)) {
		if ((result & BIT(pos)) != 0) {
			result &= (uint32_t)~BIT(pos);
			soc_ecia_girq_status_clear(XEC_ESPI_GIRQ_VW_BANK1, pos);

			const struct xec_espi_ng_vw *p = find_h2t_vw_by_girq(pos, &signal);

			if (p != NULL) {
				ev.evt_type = ESPI_BUS_EVENT_VWIRE_RECEIVED;
				ev.evt_details = signal;
				ev.evt_data = 0;

				ofs = XEC_ESPI_VW_HT_GRPW(p->vw_reg_num, 2);
				vw_pos = XEC_ESPI_VW_STATE_POS(p->src_pos);

				if (sys_test_bit(vb + ofs, vw_pos) != 0) {
					ev.evt_data = 1u;
				}

				espi_send_callbacks(&data->cbs, dev, ev);
			}

		}

		pos++;
	}

	irq_enable(devcfg->irqn_vwb1);
}
