/*
 * Copyright (c) 2019 Intel Corporation
 * Copyright (c) 2021 Microchip Technology Inc.
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Microchip XEC eSPI peripheral channel 8042 keyboard controller.
 *
 * Derived from the 8042 KBC support the V2 eSPI driver implements inside
 * espi_mchp_xec_host_v2.c. Here the logical device is a standalone Zephyr
 * driver bound to its own device tree node: it owns the KBC block registers and
 * the IBF and OBE interrupts, and registers with the eSPI controller for the
 * events that tell it when its Host facing decoders are valid. The Host I/O BAR
 * and the two Serial-IRQ slots are described on this node but programmed by the
 * controller, because MEC hardware holds those registers in reset while eSPI
 * Reset or PLTRST is asserted.
 */

#define DT_DRV_COMPAT microchip_xec_espi_pc_kbc

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
	     "Only one XEC eSPI peripheral channel 8042 KBC is supported");

struct espi_pc_kbc_xec_config {
	struct kbc_regs *regs;
	const struct device *espi_dev;
	uint32_t ibf_ecia_info;
	uint32_t obe_ecia_info;
};

struct espi_pc_kbc_xec_data {
	struct mchp_xec_espi_pc_cb pc_cb;
	/* Whether the application wants Host facing interrupts from this block.
	 * Enabled until espi_interrupt_config() says otherwise, so an
	 * application that never calls it behaves as it did under V2.
	 */
	bool intr_en;
};

/* Apply the KBC block configuration. Called at init and again every time the
 * controller reports the Host facing hardware has become usable.
 */
static void kbc_block_config(const struct device *dev)
{
	const struct espi_pc_kbc_xec_config *cfg = dev->config;
	struct kbc_regs *regs = cfg->regs;

	regs->KBC_CTRL |= MCHP_KBC_CTRL_AUXH;
	regs->KBC_CTRL |= MCHP_KBC_CTRL_OBFEN;
	/* This is the activate register, but the HAL has a funny name */
	regs->ACTV = MCHP_KBC_ACTV_EN;
}

/* Apply the tracked interrupt enable to the hardware. The Host writes a command
 * whenever it likes, so IBF is armed whenever interrupts are on. OBE is a level
 * source that stays asserted for as long as the output buffer is empty, so it is
 * left masked here and armed by the application once it has a byte for the Host.
 */
static void kbc_intr_apply(const struct device *dev)
{
	const struct espi_pc_kbc_xec_config *cfg = dev->config;
	struct espi_pc_kbc_xec_data *data = dev->data;

	if (data->intr_en) {
		xec_pc_girq_clr(cfg->ibf_ecia_info);
		xec_pc_girq_ctrl(cfg->ibf_ecia_info, MCHP_MEC_ECIA_GIRQ_EN);
	} else {
		xec_pc_girq_ctrl(cfg->ibf_ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
		xec_pc_girq_ctrl(cfg->obe_ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	}
}

/* Called from the controller's espi_interrupt_config() implementation. */
static void kbc_intr_cfg(const struct device *dev, bool enable)
{
	struct espi_pc_kbc_xec_data *data = dev->data;

	data->intr_en = enable;
	kbc_intr_apply(dev);
}

static void kbc_ibf_isr(const struct device *dev)
{
	const struct espi_pc_kbc_xec_config *cfg = dev->config;
	struct kbc_regs *regs = cfg->regs;

#ifdef CONFIG_ESPI_XEC_V3_PC_KBC_IBF_EVT_DATA
	/* Chrome solution */
	struct espi_event evt = {
		ESPI_BUS_PERIPHERAL_NOTIFICATION,
		ESPI_PERIPHERAL_8042_KBC,
		ESPI_PERIPHERAL_NODATA,
	};
	struct espi_evt_data_kbc *kbc_evt = (struct espi_evt_data_kbc *)&evt.evt_data;

	/*
	 * Indicates if the host sent a command or data.
	 * 0 = data
	 * 1 = Command.
	 */
	kbc_evt->type = (regs->EC_KBC_STS & MCHP_KBC_STS_CD) ? 1 : 0;
	/* The data in KBC Input Buffer */
	kbc_evt->data = regs->EC_DATA;
	/* KBC Input Buffer Full event */
	kbc_evt->evt = HOST_KBC_EVT_IBF;
#else
	/* Windows solution */
	/* The high byte contains information from the host, and the lower byte
	 * specifies if the host sent a command or data. 1 = Command.
	 */
	uint32_t isr_data = ((regs->EC_KBC_STS & MCHP_KBC_STS_CD) << E8042_ISR_CMD_DATA_POS);

	isr_data |= ((regs->EC_DATA & 0xffU) << E8042_ISR_DATA_POS);

	struct espi_event evt = {.evt_type = ESPI_BUS_PERIPHERAL_NOTIFICATION,
				 .evt_details = ESPI_PERIPHERAL_8042_KBC,
				 .evt_data = isr_data};
#endif
	xec_pc_girq_clr(cfg->ibf_ecia_info);

	mchp_xec_espi_v3_send_callbacks(cfg->espi_dev, evt);
}

static void kbc_obe_isr(const struct device *dev)
{
	const struct espi_pc_kbc_xec_config *cfg = dev->config;

#ifdef CONFIG_ESPI_XEC_V3_PC_KBC_OBE_CBK
	/* Chrome solution */
	struct espi_event evt = {
		ESPI_BUS_PERIPHERAL_NOTIFICATION,
		ESPI_PERIPHERAL_8042_KBC,
		ESPI_PERIPHERAL_NODATA,
	};
	struct espi_evt_data_kbc *kbc_evt = (struct espi_evt_data_kbc *)&evt.evt_data;

	/* Disable KBC OBE interrupt first */
	xec_pc_girq_ctrl(cfg->obe_ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);

	/*
	 * Notify application that host already read out data. The application
	 * might need to clear status register via espi_write_lpc_request() with
	 * E8042_CLEAR_FLAG opcode in callback.
	 */
	kbc_evt->evt = HOST_KBC_EVT_OBE;
	kbc_evt->data = 0;
	kbc_evt->type = 0;

	xec_pc_girq_clr(cfg->obe_ecia_info);

	mchp_xec_espi_v3_send_callbacks(cfg->espi_dev, evt);
#else
	/* Windows solution: disable and clear the interrupt source */
	xec_pc_girq_ctrl(cfg->obe_ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	xec_pc_girq_clr(cfg->obe_ecia_info);
#endif
}

/* Serve the E8042_* opcode range on behalf of the espi_read_lpc_request() and
 * espi_write_lpc_request() shims in the controller driver.
 */
static int kbc_lpc_request(const struct device *dev, enum lpc_peripheral_opcode op, uint32_t *data,
			   bool write)
{
	const struct espi_pc_kbc_xec_config *cfg = dev->config;
	struct kbc_regs *regs = cfg->regs;

	if (data == NULL) {
		return -EINVAL;
	}

	/* Make sure kbc 8042 is on */
	if (!(regs->KBC_CTRL & MCHP_KBC_CTRL_OBFEN)) {
		return -ENOTSUP;
	}

	if (!write) {
		switch (op) {
		case E8042_OBF_HAS_CHAR:
			/* EC has written data back to host. OBF is automatically
			 * cleared after host reads the data.
			 */
			*data = (regs->EC_KBC_STS & MCHP_KBC_STS_OBF) ? 1 : 0;
			break;
		case E8042_IBF_HAS_CHAR:
			*data = (regs->EC_KBC_STS & MCHP_KBC_STS_IBF) ? 1 : 0;
			break;
		case E8042_READ_KB_STS:
			*data = regs->EC_KBC_STS;
			break;
		default:
			return -EINVAL;
		}

		return 0;
	}

	switch (op) {
	case E8042_WRITE_KB_CHAR:
		regs->EC_DATA = *data & 0xffU;
		break;
	case E8042_WRITE_MB_CHAR:
		regs->EC_AUX_DATA = *data & 0xffU;
		break;
	case E8042_RESUME_IRQ:
		xec_pc_girq_clr(cfg->ibf_ecia_info);
		xec_pc_girq_ctrl(cfg->ibf_ecia_info, MCHP_MEC_ECIA_GIRQ_EN);
		break;
	case E8042_PAUSE_IRQ:
		xec_pc_girq_ctrl(cfg->ibf_ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
		break;
	case E8042_CLEAR_OBF:
		/* Reading the Host register is what clears OBF. */
		(void)regs->HOST_AUX_DATA;
		break;
	case E8042_SET_FLAG:
		/* FW shouldn't modify these flags directly */
		*data &= ~(MCHP_KBC_STS_OBF | MCHP_KBC_STS_IBF | MCHP_KBC_STS_AUXOBF);
		regs->EC_KBC_STS |= *data;
		break;
	case E8042_CLEAR_FLAG:
		/* FW shouldn't modify these flags directly */
		*data |= (MCHP_KBC_STS_OBF | MCHP_KBC_STS_IBF | MCHP_KBC_STS_AUXOBF);
		regs->EC_KBC_STS &= ~(*data);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void kbc_espi_event(const struct device *espi_dev, const struct device *dev,
			   enum mchp_xec_espi_pc_event evt)
{
	const struct espi_pc_kbc_xec_config *cfg = dev->config;

	ARG_UNUSED(espi_dev);

	if (xec_pc_evt_is_hw_usable(evt)) {
		/* The controller has re-programmed our Host I/O BAR and the two
		 * Serial-IRQ slots. Re-arm the block, and the interrupt sources
		 * if that is what the application last asked for.
		 */
		kbc_block_config(dev);
		kbc_intr_apply(dev);
	} else {
		/* Host facing registers are about to be held in reset. Stop
		 * generating events the application cannot act on, without
		 * disturbing what it asked for.
		 */
		xec_pc_girq_ctrl(cfg->ibf_ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
		xec_pc_girq_ctrl(cfg->obe_ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	}
}

static int espi_pc_kbc_xec_init(const struct device *dev)
{
	const struct espi_pc_kbc_xec_config *cfg = dev->config;
	struct espi_pc_kbc_xec_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->espi_dev)) {
		LOG_ERR("KBC: eSPI controller not ready");
		return -ENODEV;
	}

	kbc_block_config(dev);

	xec_pc_girq_clr(cfg->ibf_ecia_info);
	xec_pc_girq_clr(cfg->obe_ecia_info);

	data->pc_cb.pc_dev = dev;
	data->pc_cb.handler = kbc_espi_event;
	data->pc_cb.evt_mask = XEC_PC_EVT_MASK_ALL;
	data->pc_cb.lpc_request = kbc_lpc_request;
	data->pc_cb.lpc_op_start = E8042_START_OPCODE;
	data->pc_cb.lpc_op_end = E8042_MAX_OPCODE;
	data->pc_cb.intr_cfg = kbc_intr_cfg;
	data->pc_cb.intr_flags = ESPI_PERIPHERAL_8042_KBC_EVENTS;
	data->intr_en = true;

	ret = mchp_xec_espi_pc_register(cfg->espi_dev, &data->pc_cb);
	if (ret != 0) {
		LOG_ERR("KBC: eSPI registration failed (%d)", ret);
		return ret;
	}

	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, ibf, irq), DT_INST_IRQ_BY_NAME(0, ibf, priority),
		    kbc_ibf_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, ibf, irq));

	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, obe, irq), DT_INST_IRQ_BY_NAME(0, obe, priority),
		    kbc_obe_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, obe, irq));

	kbc_intr_apply(dev);

	return 0;
}

static const struct espi_pc_kbc_xec_config espi_pc_kbc_xec_cfg0 = {
	.regs = (struct kbc_regs *)DT_INST_REG_ADDR(0),
	.espi_dev = DEVICE_DT_GET(DT_INST_PARENT(0)),
	.ibf_ecia_info = DT_INST_PROP_BY_IDX(0, girqs, 0),
	.obe_ecia_info = DT_INST_PROP_BY_IDX(0, girqs, 1),
};

static struct espi_pc_kbc_xec_data espi_pc_kbc_xec_data0;

DEVICE_DT_INST_DEFINE(0, espi_pc_kbc_xec_init, NULL, &espi_pc_kbc_xec_data0,
		      &espi_pc_kbc_xec_cfg0, POST_KERNEL, CONFIG_ESPI_XEC_V3_PC_INIT_PRIORITY,
		      NULL);
