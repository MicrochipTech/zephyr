/*
 * Copyright (c) 2021 Microchip Technology Inc.
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Microchip XEC eSPI peripheral channel Host/EC mailbox logical device.
 *
 * The block holds a one byte Host-to-EC register, a one byte EC-to-Host
 * register, an SMI trigger and mask pair, and 32 data bytes, all reached by the
 * Host through one Host I/O decoder and one Host memory BAR. Both decoders and
 * both Serial IRQs are described on this device tree node and programmed by the
 * eSPI controller, so this driver owns only the block registers and the single
 * EC interrupt the Host-to-EC register raises.
 *
 * The public mchp_xec_espi_pc_mailbox_* API keeps the signatures the V2 driver
 * published; the device handle callers pass is now this device instead of the
 * eSPI controller. Two of those functions behave differently here:
 *
 *  - mailbox_get_all() and mailbox_set_all() move all 32 bytes. The V2
 *    implementations loop n over 0 and 4 only and index src[n] and dest[n] with
 *    the same n they use as a byte offset, so they move 8 bytes taken from the
 *    wrong two words of the buffer.
 *  - num_mboxes is a count rather than an exclusive end index, so a caller can
 *    ask for a run of bytes anywhere in the 32 byte block. Under V2,
 *    mbox_idx = 4 with num_mboxes = 4 copies nothing.
 *
 * Serial IRQ enable and disable goes through the controller. V2 writes the
 * controller SIRQ registers directly from mailbox_ictrl(), which a peripheral
 * channel driver must not do: hardware holds that register bank in reset while
 * eSPI Reset or PLTRST is asserted and only the controller tracks that state.
 */

#define DT_DRV_COMPAT microchip_xec_espi_pc_mbox

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi/mchp_xec_espi.h>
#include <zephyr/dt-bindings/espi/mchp-xec-espi-v3.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "../espi_mchp_xec_v3.h"
#include "espi_pc_xec.h"

LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "Only one XEC eSPI peripheral channel mailbox is supported");

/* Number of 32-bit words behind the 32 mailbox data bytes */
#define XEC_MBOX_NUM_32BIT_DATA (MCHP_XEC_MAX_MAILBOX_INDEX / 4U)

/* The Host index and data pair at offset 0 is the Host side of the block. The
 * EC side starts at offset 0x100. Each of the four EC side command registers is
 * one byte in a 32-bit slot.
 */
struct xec_mbox_regs {
	volatile uint8_t HINDEX;
	volatile uint8_t HDATA;
	uint8_t RSVD1[2];
	uint8_t RSVD2[(0x100U - 0x004U)];
	volatile uint8_t H2EC;
	uint8_t RSVD3[3];
	volatile uint8_t EC2H;
	uint8_t RSVD4[3];
	volatile uint8_t SMI_SRC;
	uint8_t RSVD5[3];
	volatile uint8_t SMI_MASK;
	uint8_t RSVD6[3];
	volatile uint32_t DATA[XEC_MBOX_NUM_32BIT_DATA];
};

BUILD_ASSERT(sizeof(struct xec_mbox_regs) == 0x130U, "XEC mailbox register layout mismatch");

struct espi_pc_mbox_xec_config {
	struct xec_mbox_regs *regs;
	const struct device *espi_dev;
	uint32_t ecia_info;
};

BUILD_ASSERT(MCHP_XEC_ESPI_PC_MBOX_SRC_EC == BIT(MCHP_XEC_ESPI_PC_MBOX_ISRC_EC),
	     "Mailbox EC interrupt source bit must match enum mchp_xec_espi_pc_mbox_isrc");

struct espi_pc_mbox_xec_data {
	struct mchp_xec_espi_pc_cb pc_cb;
};

/* Byte view of the 32 mailbox data registers. Hardware allows both widths and
 * the SoC is little endian, so byte index n is the byte the Host sees at data
 * offset n either way.
 */
static inline volatile uint8_t *mbox_data_bytes(const struct espi_pc_mbox_xec_config *cfg)
{
	return (volatile uint8_t *)cfg->regs->DATA;
}

/*
 * The Host raises this interrupt by writing the Host-to-EC register. Writing
 * 0xFF back is what clears the interrupt source in hardware, but it also clears
 * the register to zero, which a Host polling for command completion may read as
 * the EC having finished. Mask the interrupt instead and leave the register for
 * the application to consume; the application re-arms the source through
 * mchp_xec_espi_pc_mailbox_ictrl() once it has written its response.
 */
static void mbox_isr(const struct device *dev)
{
	const struct espi_pc_mbox_xec_config *cfg = dev->config;
	struct espi_event evt = {
		ESPI_BUS_PERIPHERAL_NOTIFICATION,
		MCHP_XEC_ESPI_PERIPHERAL_MAILBOX,
		ESPI_PERIPHERAL_NODATA,
	};

	evt.evt_data = cfg->regs->H2EC;

	xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	xec_pc_girq_clr(cfg->ecia_info);

	mchp_xec_espi_v3_send_callbacks(cfg->espi_dev, evt);
}

/* Apply the requested interrupt source to the hardware. The Host-to-EC register
 * write is an edge source, so it is armed as soon as it is asked for, and status
 * latched while it was masked is discarded on the way in.
 *
 * mbox_isr() masks the source in hardware without changing the request, because
 * the Host has one command outstanding until the application answers it. Reading
 * the request back therefore reports the source enabled while the hardware is
 * masked, and re-arming it is what mchp_xec_espi_pc_mailbox_ictrl() is for.
 */
static void mbox_intr_apply(const struct device *dev)
{
	const struct espi_pc_mbox_xec_config *cfg = dev->config;
	struct espi_pc_mbox_xec_data *data = dev->data;

	if ((data->pc_cb.intr_src_en & MCHP_XEC_ESPI_PC_MBOX_SRC_EC) != 0U) {
		xec_pc_girq_clr(cfg->ecia_info);
		xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_EN);
	} else {
		xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	}
}

static void mbox_espi_event(const struct device *espi_dev, const struct device *dev,
			    enum mchp_xec_espi_pc_event evt)
{
	const struct espi_pc_mbox_xec_config *cfg = dev->config;

	ARG_UNUSED(espi_dev);

	if (!xec_pc_evt_is_hw_usable(evt)) {
		xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
		return;
	}

	/* The controller has re-programmed our decoders. Drop whatever the Host
	 * left latched across the reset and take interrupts again, so a command
	 * issued right after de-assertion is seen even if the application had
	 * left the source masked from an earlier command.
	 */
	mbox_intr_apply(dev);
}

int mchp_xec_espi_pc_mailbox_get(const struct device *dev, uint8_t mbox_idx, uint8_t num_mboxes,
				 uint8_t *dest)
{
	const struct espi_pc_mbox_xec_config *cfg = NULL;
	volatile uint8_t *data = NULL;

	if ((dev == NULL) || (dest == NULL) || (mbox_idx >= MCHP_XEC_MAX_MAILBOX_INDEX) ||
	    ((uint32_t)mbox_idx + (uint32_t)num_mboxes > MCHP_XEC_MAX_MAILBOX_INDEX)) {
		return -EINVAL;
	}

	cfg = dev->config;
	data = mbox_data_bytes(cfg);

	for (uint8_t n = 0U; n < num_mboxes; n++) {
		dest[n] = data[mbox_idx + n];
	}

	return 0;
}

int mchp_xec_espi_pc_mailbox_set(const struct device *dev, uint8_t mbox_idx, uint8_t num_mboxes,
				 const uint8_t *src)
{
	const struct espi_pc_mbox_xec_config *cfg = NULL;
	volatile uint8_t *data = NULL;

	if ((dev == NULL) || (src == NULL) || (mbox_idx >= MCHP_XEC_MAX_MAILBOX_INDEX) ||
	    ((uint32_t)mbox_idx + (uint32_t)num_mboxes > MCHP_XEC_MAX_MAILBOX_INDEX)) {
		return -EINVAL;
	}

	cfg = dev->config;
	data = mbox_data_bytes(cfg);

	for (uint8_t n = 0U; n < num_mboxes; n++) {
		data[mbox_idx + n] = src[n];
	}

	return 0;
}

int mchp_xec_espi_pc_mailbox_get_all(const struct device *dev, uint32_t *dest)
{
	const struct espi_pc_mbox_xec_config *cfg = NULL;

	if ((dev == NULL) || (dest == NULL)) {
		return -EINVAL;
	}

	cfg = dev->config;

	for (size_t n = 0U; n < XEC_MBOX_NUM_32BIT_DATA; n++) {
		dest[n] = cfg->regs->DATA[n];
	}

	return 0;
}

int mchp_xec_espi_pc_mailbox_set_all(const struct device *dev, const uint32_t *src)
{
	const struct espi_pc_mbox_xec_config *cfg = NULL;

	if ((dev == NULL) || (src == NULL)) {
		return -EINVAL;
	}

	cfg = dev->config;

	for (size_t n = 0U; n < XEC_MBOX_NUM_32BIT_DATA; n++) {
		cfg->regs->DATA[n] = src[n];
	}

	return 0;
}

int mchp_xec_espi_pc_mailbox_cmd(const struct device *dev, enum mchp_xec_espi_pc_mbox_cmd_id cmd_id,
				 uint8_t cmd)
{
	const struct espi_pc_mbox_xec_config *cfg = NULL;

	if (dev == NULL) {
		return -EINVAL;
	}

	cfg = dev->config;

	switch (cmd_id) {
	case MCHP_XEC_ESPI_PC_MBOX_CMD_ID_HOST_TO_EC:
		cfg->regs->H2EC = cmd;
		break;
	case MCHP_XEC_ESPI_PC_MBOX_CMD_ID_EC_TO_HOST:
		cfg->regs->EC2H = cmd;
		break;
	case MCHP_XEC_ESPI_PC_MBOX_CMD_ID_SMI_SRC:
		cfg->regs->SMI_SRC = cmd;
		break;
	case MCHP_XEC_ESPI_PC_MBOX_CMD_ID_SMI_MASK:
		cfg->regs->SMI_MASK = cmd;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int mchp_xec_espi_pc_mailbox_ictrl(const struct device *dev, enum mchp_xec_espi_pc_mbox_isrc id,
				   uint8_t enable)
{
	const struct espi_pc_mbox_xec_config *cfg = NULL;

	if (dev == NULL) {
		return -EINVAL;
	}

	cfg = dev->config;

	switch (id) {
	case MCHP_XEC_ESPI_PC_MBOX_ISRC_EC:
		/* Goes through the per source API so the two cannot disagree
		 * about what the application asked for, and so the request
		 * survives the next bus reset.
		 */
		return mchp_xec_espi_pc_intr_set(dev, MCHP_XEC_ESPI_PC_MBOX_SRC_EC,
						 (enable != 0U) ? MCHP_XEC_ESPI_PC_MBOX_SRC_EC
								: 0U);
	case MCHP_XEC_ESPI_PC_MBOX_ISRC_SIRQ:
		return mchp_xec_espi_v3_sirq_enable(cfg->espi_dev, MCHP_XEC_ESPI_SIRQ_MBOX, enable);
	case MCHP_XEC_ESPI_PC_MBOX_ISRC_SIRQ_SMI:
		return mchp_xec_espi_v3_sirq_enable(cfg->espi_dev, MCHP_XEC_ESPI_SIRQ_MBOX_SMI,
						    enable);
	default:
		return -EINVAL;
	}

	return 0;
}

static int espi_pc_mbox_xec_init(const struct device *dev)
{
	const struct espi_pc_mbox_xec_config *cfg = dev->config;
	struct espi_pc_mbox_xec_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->espi_dev)) {
		LOG_ERR("Mailbox: eSPI controller not ready");
		return -ENODEV;
	}

	xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	xec_pc_girq_clr(cfg->ecia_info);

	data->pc_cb.pc_dev = dev;
	data->pc_cb.handler = mbox_espi_event;
	data->pc_cb.evt_mask = XEC_PC_EVT_MASK_ALL;
	data->pc_cb.intr_apply = mbox_intr_apply;
	/* Armed until the application says otherwise, so one that never asks
	 * behaves as it did under V2. The block's two Serial IRQ outputs are not
	 * EC interrupt sources and stay with the controller, so they are not
	 * listed here. No generic espi_interrupt_flags bit describes the mailbox,
	 * so intr_flags stays clear.
	 */
	data->pc_cb.intr_src_supported = MCHP_XEC_ESPI_PC_MBOX_SRC_EC;
	data->pc_cb.intr_src_en = MCHP_XEC_ESPI_PC_MBOX_SRC_EC;

	ret = mchp_xec_espi_pc_register(cfg->espi_dev, &data->pc_cb);
	if (ret != 0) {
		LOG_ERR("Mailbox: eSPI registration failed (%d)", ret);
		return ret;
	}

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), mbox_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_EN);

	return 0;
}

static const struct espi_pc_mbox_xec_config espi_pc_mbox_xec_cfg0 = {
	.regs = (struct xec_mbox_regs *)DT_INST_REG_ADDR(0),
	.espi_dev = DEVICE_DT_GET(DT_INST_PARENT(0)),
	.ecia_info = DT_INST_PROP_BY_IDX(0, girqs, 0),
};

static struct espi_pc_mbox_xec_data espi_pc_mbox_xec_data0;

DEVICE_DT_INST_DEFINE(0, espi_pc_mbox_xec_init, NULL, &espi_pc_mbox_xec_data0,
		      &espi_pc_mbox_xec_cfg0, POST_KERNEL, CONFIG_ESPI_XEC_V3_PC_INIT_PRIORITY,
		      NULL);
