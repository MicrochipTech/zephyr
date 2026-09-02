/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Microchip XEC eSPI peripheral channel glue (VCC power good generation) logic.
 *
 * The block derives VCC_PWRGD2, the power good signal the Host chipset sees,
 * and derives the S0ix connected standby state from the SLP_S0# and CPU_C10
 * inputs so that power good can be taken from a second, lower current,
 * regulator chain while the system is idle. C10 may come from a pin or from the
 * HOST_C10 eSPI virtual wire. Both derived signals are reported to the EC as
 * edge interrupts.
 *
 * The V2 eSPI driver has no support for this block. Its dispatch tables in
 * espi_mchp_xec_host_v2.c carry NULL for both the glue logic init and IRQ
 * connect slots, so the node exists in the device tree with no code behind it.
 *
 * How the board wires the inputs is device tree data, applied here rather than
 * left to the application, because hardware requires the polarity and source
 * selections to be in place before the detection source field is set: setting
 * the detection source while an input is inverted or selected wrongly glitches
 * VCC_PWRGD2, and a glitch there corrupts the chipset's idea of the platform
 * power state regardless of what the platform is actually doing. That ordering
 * is easy to get wrong once and impossible to observe from firmware, so the
 * driver owns it.
 *
 * Which of the two derived signals raises an event, and whether S0ix detection
 * runs at all, stay with the application through mchp_xec_espi_pc_glue_ictrl()
 * and mchp_xec_espi_pc_glue_s0ix_detect_enable(). The detection enable is a
 * runtime register hardware clears on every PLTRST, so this driver re-applies
 * the last value asked for once the controller reports the Host facing hardware
 * usable again. That is the whole reason a peripheral channel driver registers
 * for these events: the application asked for detection once and should not
 * have to watch the bus to keep it.
 */

#define DT_DRV_COMPAT microchip_xec_espi_pc_glue

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
	     "Only one XEC eSPI peripheral channel glue logic block is supported");

/* Signals carried by the three signal monitor registers. The state, interrupt
 * pending and interrupt enable registers all use this layout, and the values
 * match the MCHP_XEC_ESPI_PC_GLUE_SIG_* bits the public API reports.
 */
#define XEC_GLUE_SIG_PWRGD 0x01U
#define XEC_GLUE_SIG_S0IX  0x02U
#define XEC_GLUE_SIG_MSK   (XEC_GLUE_SIG_PWRGD | XEC_GLUE_SIG_S0IX)

BUILD_ASSERT(XEC_GLUE_SIG_PWRGD == MCHP_XEC_ESPI_PC_GLUE_SIG_PWRGD,
	     "Glue logic power good signal bit mismatch");
BUILD_ASSERT(XEC_GLUE_SIG_S0IX == MCHP_XEC_ESPI_PC_GLUE_SIG_S0IX,
	     "Glue logic S0ix signal bit mismatch");

/* Which power good input VCC_PWRGD2 follows in S0ix. Hardware reserves the
 * value 2.
 */
#define XEC_GLUE_PWRGD_SRC_MSK      GENMASK(1, 0)
#define XEC_GLUE_PWRGD_SRC_VCC      0U
#define XEC_GLUE_PWRGD_SRC_S0IX_PIN 1U
#define XEC_GLUE_PWRGD_SRC_BOTH     3U

/* How the S0ix state is derived */
#define XEC_GLUE_DET_SRC_MSK    GENMASK(1, 0)
#define XEC_GLUE_DET_SLP_S0_INV BIT(2)
#define XEC_GLUE_DET_C10_INV    BIT(3)
#define XEC_GLUE_DET_C10_VWIRE  BIT(4)
#define XEC_GLUE_DET_FW_PGSEL   BIT(5)

#define XEC_GLUE_S0IX_DET_EN BIT(0)

/* The S0ix detection enable at offset 4 is the block's one runtime register and
 * is reachable by the Host through a BAR. Everything from 0x10C on is EC only.
 */
struct xec_glue_regs {
	uint8_t RSVD1[4];
	volatile uint8_t S0IX_DET_EN;
	uint8_t RSVD2[3];
	uint8_t RSVD3[(0x10cU - 0x008U)];
	volatile uint32_t PWRGD_SRC_CFG;
	volatile uint32_t S0IX_DET_CFG;
	uint8_t RSVD4[(0x128U - 0x114U)];
	volatile uint32_t SMON;
	volatile uint32_t SMON_IPEND;
	volatile uint32_t SMON_IEN;
};

BUILD_ASSERT(sizeof(struct xec_glue_regs) == 0x134U, "XEC glue logic register layout mismatch");

struct espi_pc_glue_xec_config {
	struct xec_glue_regs *regs;
	const struct device *espi_dev;
	uint32_t ecia_info;
	uint8_t pwrgd_src_cfg;
	uint8_t s0ix_det_cfg;
};

BUILD_ASSERT(MCHP_XEC_ESPI_PC_GLUE_SRC_PWRGD == MCHP_XEC_ESPI_PC_GLUE_SIG_PWRGD &&
		     MCHP_XEC_ESPI_PC_GLUE_SRC_S0IX == MCHP_XEC_ESPI_PC_GLUE_SIG_S0IX,
	     "Glue logic interrupt source bits must match the signal bits they report");

struct espi_pc_glue_xec_data {
	struct mchp_xec_espi_pc_cb pc_cb;
	/* S0ix detection enable the application asked for. Not an interrupt
	 * source: it decides whether the block derives the S0ix signal at all.
	 */
	uint8_t s0ix_det_en;
};

static void glue_isr(const struct device *dev)
{
	const struct espi_pc_glue_xec_config *cfg = dev->config;
	struct espi_event evt = {
		.evt_type = ESPI_BUS_PERIPHERAL_NOTIFICATION,
		.evt_details = MCHP_XEC_ESPI_PERIPHERAL_GLUE,
		.evt_data = 0U,
	};
	struct mchp_xec_espi_pc_glue_evt *glue_evt =
		(struct mchp_xec_espi_pc_glue_evt *)&evt.evt_data;
	uint32_t edges = cfg->regs->SMON_IPEND & cfg->regs->SMON_IEN & XEC_GLUE_SIG_MSK;

	/* Clear only the edges being reported. An edge latched for a signal
	 * whose interrupt is disabled is not ours to drop, and one arriving
	 * between the read above and the write below is left pending rather
	 * than lost.
	 */
	cfg->regs->SMON_IPEND = edges;

	xec_pc_girq_clr(cfg->ecia_info);

	if (edges == 0U) {
		return;
	}

	glue_evt->state = (uint32_t)(cfg->regs->SMON & XEC_GLUE_SIG_MSK);
	glue_evt->edges = edges;

	mchp_xec_espi_v3_send_callbacks(cfg->espi_dev, evt);
}

/* Apply the requested interrupt sources to the signal monitor. Both are edge
 * sources on a signal change, so each is armed as soon as it is asked for. Edges
 * latched while a signal was masked say nothing the application asked to hear,
 * so they are dropped as it is armed and it is told about the next change.
 */
static void glue_intr_apply(const struct device *dev)
{
	const struct espi_pc_glue_xec_config *cfg = dev->config;
	struct espi_pc_glue_xec_data *data = dev->data;
	uint8_t want = (uint8_t)(data->pc_cb.intr_src_en & XEC_GLUE_SIG_MSK);
	uint8_t newly = want & (uint8_t)~cfg->regs->SMON_IEN;

	if (newly != 0U) {
		cfg->regs->SMON_IPEND = newly;
	}

	cfg->regs->SMON_IEN = want;
}

static void glue_block_config(const struct device *dev)
{
	const struct espi_pc_glue_xec_config *cfg = dev->config;
	struct espi_pc_glue_xec_data *data = dev->data;

	cfg->regs->PWRGD_SRC_CFG = cfg->pwrgd_src_cfg;

	/* Input polarity and source selections first, detection source last, so
	 * VCC_PWRGD2 cannot glitch while the block is being configured.
	 */
	cfg->regs->S0IX_DET_CFG = cfg->s0ix_det_cfg & (uint8_t)~XEC_GLUE_DET_SRC_MSK;
	cfg->regs->S0IX_DET_CFG = cfg->s0ix_det_cfg;

	cfg->regs->S0IX_DET_EN = data->s0ix_det_en;

	/* Configuring the detection above can move the derived S0ix state, and
	 * such an edge says nothing about the platform, so start from a clean
	 * pending register and let the application read the state it wants
	 * through mchp_xec_espi_pc_glue_state_get().
	 */
	cfg->regs->SMON_IPEND = XEC_GLUE_SIG_MSK;

	glue_intr_apply(dev);
}

static void glue_espi_event(const struct device *espi_dev, const struct device *dev,
			    enum mchp_xec_espi_pc_event evt)
{
	const struct espi_pc_glue_xec_config *cfg = dev->config;

	ARG_UNUSED(espi_dev);

	if (!xec_pc_evt_is_hw_usable(evt)) {
		xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
		return;
	}

	/* PLTRST clears the S0ix detection enable, so re-apply the whole block
	 * configuration rather than track which register this event cleared.
	 */
	glue_block_config(dev);

	xec_pc_girq_clr(cfg->ecia_info);
	xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_EN);
}

int mchp_xec_espi_pc_glue_state_get(const struct device *dev, uint8_t *state)
{
	const struct espi_pc_glue_xec_config *cfg = NULL;

	if ((dev == NULL) || (state == NULL)) {
		return -EINVAL;
	}

	cfg = dev->config;

	*state = (uint8_t)(cfg->regs->SMON & XEC_GLUE_SIG_MSK);

	return 0;
}

int mchp_xec_espi_pc_glue_ictrl(const struct device *dev, uint8_t sig_mask, uint8_t enable)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	/* Kept for callers written against the signal bits. Signals outside the
	 * two this block carries are dropped rather than refused, which is what
	 * this entry point has always done; mchp_xec_espi_pc_intr_set() is the
	 * strict one.
	 */
	sig_mask &= XEC_GLUE_SIG_MSK;
	if (sig_mask == 0U) {
		return -EINVAL;
	}

	return mchp_xec_espi_pc_intr_set(dev, sig_mask, (enable != 0U) ? sig_mask : 0U);
}

int mchp_xec_espi_pc_glue_s0ix_detect_enable(const struct device *dev, uint8_t enable)
{
	const struct espi_pc_glue_xec_config *cfg = NULL;
	struct espi_pc_glue_xec_data *data = NULL;

	if (dev == NULL) {
		return -EINVAL;
	}

	cfg = dev->config;
	data = dev->data;

	/* Remember the request. Hardware clears this register on every PLTRST
	 * and glue_block_config() puts it back from here.
	 */
	data->s0ix_det_en = (enable != 0U) ? XEC_GLUE_S0IX_DET_EN : 0U;
	cfg->regs->S0IX_DET_EN = data->s0ix_det_en;

	return 0;
}

static int espi_pc_glue_xec_init(const struct device *dev)
{
	const struct espi_pc_glue_xec_config *cfg = dev->config;
	struct espi_pc_glue_xec_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->espi_dev)) {
		LOG_ERR("Glue: eSPI controller not ready");
		return -ENODEV;
	}

	xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	xec_pc_girq_clr(cfg->ecia_info);

	data->pc_cb.pc_dev = dev;
	data->pc_cb.handler = glue_espi_event;
	data->pc_cb.evt_mask = XEC_PC_EVT_MASK_ALL;
	data->pc_cb.intr_apply = glue_intr_apply;
	/* Both derived signals start masked. Nothing but the application knows
	 * which of them its power sequencing cares about. No generic
	 * espi_interrupt_flags bit describes them either, so intr_flags stays
	 * clear and espi_interrupt_config() leaves this block alone.
	 */
	data->pc_cb.intr_src_supported =
		MCHP_XEC_ESPI_PC_GLUE_SRC_PWRGD | MCHP_XEC_ESPI_PC_GLUE_SRC_S0IX;
	data->pc_cb.intr_src_en = 0U;
	data->s0ix_det_en = 0U;

	glue_block_config(dev);

	ret = mchp_xec_espi_pc_register(cfg->espi_dev, &data->pc_cb);
	if (ret != 0) {
		LOG_ERR("Glue: eSPI registration failed (%d)", ret);
		return ret;
	}

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), glue_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_EN);

	return 0;
}

/* The device tree names the two input combination as one enum value, but
 * hardware reserves the field value 2, so that index maps to 3.
 */
#define XEC_GLUE_PWRGD_SRC_CFG(inst)                                                               \
	((DT_INST_ENUM_IDX(inst, pwrgd_source) == 2) ? XEC_GLUE_PWRGD_SRC_BOTH                     \
						     : DT_INST_ENUM_IDX(inst, pwrgd_source))

#define XEC_GLUE_S0IX_DET_CFG(inst)                                                                \
	((uint8_t)(DT_INST_ENUM_IDX(inst, s0ix_detect_source) |                                    \
		   (DT_INST_PROP(inst, slp_s0_inverted) ? XEC_GLUE_DET_SLP_S0_INV : 0U) |          \
		   (DT_INST_PROP(inst, c10_inverted) ? XEC_GLUE_DET_C10_INV : 0U) |                \
		   ((DT_INST_ENUM_IDX(inst, c10_source) != 0) ? XEC_GLUE_DET_C10_VWIRE : 0U) |     \
		   (DT_INST_PROP(inst, fw_pwrgd_select) ? XEC_GLUE_DET_FW_PGSEL : 0U)))

static const struct espi_pc_glue_xec_config espi_pc_glue_xec_cfg0 = {
	.regs = (struct xec_glue_regs *)DT_INST_REG_ADDR(0),
	.espi_dev = DEVICE_DT_GET(DT_INST_PARENT(0)),
	.ecia_info = DT_INST_PROP_BY_IDX(0, girqs, 0),
	.pwrgd_src_cfg = XEC_GLUE_PWRGD_SRC_CFG(0),
	.s0ix_det_cfg = XEC_GLUE_S0IX_DET_CFG(0),
};

static struct espi_pc_glue_xec_data espi_pc_glue_xec_data0;

DEVICE_DT_INST_DEFINE(0, espi_pc_glue_xec_init, NULL, &espi_pc_glue_xec_data0,
		      &espi_pc_glue_xec_cfg0, POST_KERNEL, CONFIG_ESPI_XEC_V3_PC_INIT_PRIORITY,
		      NULL);
