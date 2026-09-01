/*
 * Copyright (c) 2021 Microchip Technology Inc.
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ESPI_MCHP_XEC_ESPI_V3_H_
#define ZEPHYR_DRIVERS_ESPI_MCHP_XEC_ESPI_V3_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/slist.h>

/* #define ESPI_XEC_V3_DEBUG	1 */

struct espi_isr {
	uint8_t girq_id;
	uint8_t girq_pos;
	void (*the_isr)(const struct device *dev);
};

struct espi_vw_isr {
	uint8_t signal;
	uint8_t girq_id;
	uint8_t girq_pos;
	void (*the_isr)(int girq, int bpos, void *dev);
};

struct espi_xec_irq_info {
	uint8_t gid;  /* GIRQ id [8, 26] */
	uint8_t gpos; /* bit position in GIRQ [0, 31] */
	uint8_t anid; /* Aggregated GIRQ NVIC number */
	uint8_t dnid; /* Direct GIRQ NVIC number */
};

struct espi_xec_config {
	uint32_t ioc_base_addr;
	uint32_t mc_base_addr;
	uint32_t vw_base_addr;
	uint16_t pcr_scr;
	uint8_t irq_info_size;
	uint8_t rsvd[1];
	const struct espi_xec_irq_info *irq_info_list;
	const struct pinctrl_dev_config *pcfg;
};

struct espi_xec_data {
	sys_slist_t callbacks;
	/* Peripheral channel logical device drivers registered with this
	 * controller. Walked on ESPI_RESET, peripheral channel enable change,
	 * and PLTRST virtual wire change, and by the LPC request shims.
	 */
	sys_slist_t pc_cbs;
	struct k_sem tx_lock;
	struct k_sem rx_lock;
	struct k_sem flash_lock;
#ifdef ESPI_XEC_V3_DEBUG
	uint32_t espi_rst_count;
#endif
};

struct xec_signal {
	uint8_t host_idx;
	uint8_t bit;
	uint8_t xec_reg_idx;
	uint8_t flags;
};

enum mchp_msvw_regs {
	MCHP_MSVW00,
	MCHP_MSVW01,
	MCHP_MSVW02,
	MCHP_MSVW03,
	MCHP_MSVW04,
	MCHP_MSVW05,
	MCHP_MSVW06,
	MCHP_MSVW07,
	MCHP_MSVW08,
};

enum mchp_smvw_regs {
	MCHP_SMVW00,
	MCHP_SMVW01,
	MCHP_SMVW02,
	MCHP_SMVW03,
	MCHP_SMVW04,
	MCHP_SMVW05,
	MCHP_SMVW06,
	MCHP_SMVW07,
	MCHP_SMVW08,
};

enum xec_espi_girq_idx {
	pc_girq_idx = 0,
	bm1_girq_idx,
	bm2_girq_idx,
	ltr_girq_idx,
	oob_up_girq_idx,
	oob_dn_girq_idx,
	fc_girq_idx,
	rst_girq_idx,
	vw_ch_en_girq_idx,
	ht_vw_bank0_idx,
	ht_vw_bank1_idx,
	max_girq_idx,
};

/* Deliver an eSPI event to the applications that registered a callback on the
 * eSPI controller device. Peripheral channel logical device drivers are
 * separate devices but applications keep registering their callbacks on the
 * controller, so the peripheral drivers route their notifications through here.
 */
void mchp_xec_espi_v3_send_callbacks(const struct device *espi_dev, struct espi_event evt);

#endif /* ZEPHYR_DRIVERS_ESPI_MCHP_XEC_ESPI_V3_H_ */
