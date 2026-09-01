/*
 * Copyright (c) 2021 Microchip Technology Inc.
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Microchip XEC eSPI peripheral channel Port80 BIOS debug logical device.
 *
 * The block captures Host writes to a four byte I/O range, normally 80h to
 * 83h, into a 32 entry FIFO. Each FIFO entry carries the captured byte plus the
 * byte lane it was written to and whether it began a one, two or four byte Host
 * write, so multi-byte POST codes can be reassembled. Hardware implements a
 * second, alias, I/O decoder feeding the same FIFO, described by its own device
 * tree node and pointed at from this one by the alias-decoder phandle.
 *
 * The eSPI controller programs the Host I/O BARs of this node and of the alias
 * node. What is left to this driver is the block itself: the activate bits, the
 * FIFO threshold interrupt, and the alias byte lane selector. All of those are
 * (re)applied whenever the controller reports the Host facing hardware is
 * usable again, because the Host I/O decoders come back from reset armed while
 * the block is not.
 *
 * Three things differ from the V2 implementation in espi_mchp_xec_host_v2.c:
 *
 *  - The interrupt is connected with this node's own interrupt priority.
 *    connect_irq_p80bd0() passes DT_IRQ(DT_NODELABEL(acpi_ec1), priority) with
 *    the Port80 interrupt number.
 *  - Both ISRs read the FIFO once per iteration, so no entry is popped and then
 *    dropped. The V2 multi-byte ISR pops an entry at the bottom of its loop and
 *    discards it when the loop then ends on its read budget, losing one
 *    captured byte.
 *  - The multi-byte event carries the byte lane bit map in evt_details bits
 *    [19:16] and the Host write width in bits [23:20], as its own comment
 *    documents. V2 ORs the width into both fields, so bits [19:16] read back as
 *    the width and lane map mixed together.
 *
 * The alias decoder is left inactive when the device tree gives no
 * alias-decoder phandle, so a board that decodes only the primary range pays
 * nothing for it.
 */

#define DT_DRV_COMPAT microchip_xec_espi_pc_p80bd

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi/mchp_xec_espi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "../espi_mchp_xec_v3.h"
#include "espi_pc_xec.h"

LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "Only one XEC eSPI peripheral channel Port80 BIOS debug port is supported");

/* Bytes drained from the capture FIFO per interrupt */
#define XEC_P80BD_FIFO_READ_COUNT 8U

/* Data with attributes register. Reading it pops one FIFO entry. */
#define XEC_P80BD_DATRB_DATA_MSK     GENMASK(7, 0)
#define XEC_P80BD_DATRB_DATA_GET(r)  FIELD_GET(XEC_P80BD_DATRB_DATA_MSK, (r))
#define XEC_P80BD_DATRB_LANE_MSK     GENMASK(9, 8)
#define XEC_P80BD_DATRB_LANE_GET(r)  FIELD_GET(XEC_P80BD_DATRB_LANE_MSK, (r))
#define XEC_P80BD_DATRB_LEN_MSK      GENMASK(11, 10)
#define XEC_P80BD_DATRB_LEN_GET(r)   FIELD_GET(XEC_P80BD_DATRB_LEN_MSK, (r))
#define XEC_P80BD_DATRB_NOT_EMPTY    BIT(12)

/* Length field values: a one byte Host write and every byte after the first of
 * a wider one share LEN_CONT. LEN_ORPHAN marks a byte hardware could not place,
 * left over from an earlier FIFO overrun.
 */
#define XEC_P80BD_DATRB_LEN_CONT   0U
#define XEC_P80BD_DATRB_LEN_1_OF_2 1U
#define XEC_P80BD_DATRB_LEN_1_OF_4 2U
#define XEC_P80BD_DATRB_LEN_ORPHAN 3U

#define XEC_P80BD_CONFIG_FLUSH_FIFO   BIT(0)
#define XEC_P80BD_CONFIG_CLR_SNAPSHOT BIT(1)

#define XEC_P80BD_IEN_THRESHOLD BIT(0)

#define XEC_P80BD_ACTV_ENABLE BIT(0)

/* Byte lane the alias Host I/O address maps to in the capture range */
#define XEC_P80BD_BYTE_LANE_MSK GENMASK(1, 0)

/* Host write width, in bytes, reported in evt_details bits [23:20] */
#define XEC_P80BD_EVT_WIDTH_POS 20
#define XEC_P80BD_EVT_LANES_POS 16

/* Offset 0 is the Host side capture address. The EC side starts at 0x100 and
 * the logical device activate register sits at 0x330.
 */
struct xec_p80bd_regs {
	volatile uint8_t HDATA;
	uint8_t RSVD1[(0x100U - 0x001U)];
	volatile uint32_t DATRB;
	volatile uint32_t CONFIG;
	volatile uint8_t STATUS;
	volatile uint8_t IEN;
	uint8_t RSVD2[2];
	volatile uint32_t SNAPSHOT;
	volatile uint32_t CAPTURE;
	uint8_t RSVD3[(0x330U - 0x114U)];
	volatile uint8_t ACTV;
	uint8_t RSVD4[3];
};

BUILD_ASSERT(sizeof(struct xec_p80bd_regs) == 0x334U, "XEC Port80 register layout mismatch");

/* The alias decoder window repeats the Host side capture address and holds its
 * own activate register plus the byte lane selector.
 */
struct xec_p80bd_alias_regs {
	volatile uint8_t HDATA;
	uint8_t RSVD1[(0x330U - 0x001U)];
	volatile uint8_t ACTV;
	uint8_t RSVD2[(0x3f0U - 0x331U)];
	volatile uint8_t BYTE_LANE;
	uint8_t RSVD3[3];
};

BUILD_ASSERT(sizeof(struct xec_p80bd_alias_regs) == 0x3f4U,
	     "XEC Port80 alias register layout mismatch");

struct espi_pc_p80bd_xec_config {
	struct xec_p80bd_regs *regs;
	struct xec_p80bd_alias_regs *alias_regs;
	const struct device *espi_dev;
	uint32_t ecia_info;
	uint8_t alias_byte_lane;
};

struct espi_pc_p80bd_xec_data {
	struct mchp_xec_espi_pc_cb pc_cb;
};

#ifdef CONFIG_ESPI_XEC_V3_PC_P80BD_MULTIBYTE
/*
 * Reassemble one Host write per interrupt and report it as a single event:
 *
 *   evt_data          the one, two or four byte value the Host wrote
 *   evt_details[15:0] ESPI_PERIPHERAL_DEBUG_PORT80
 *   evt_details[19:16] bit map of the byte lanes the value occupies
 *   evt_details[23:20] width of the Host write in bytes
 *
 * A byte lane bit map rather than a shift count is what lets a listener place
 * the value when the Host writes, say, only 82h and 83h of the range.
 */
static void p80bd_isr(const struct device *dev)
{
	const struct espi_pc_p80bd_xec_config *cfg = dev->config;
	struct espi_event evt = {ESPI_BUS_PERIPHERAL_NOTIFICATION, 0, ESPI_PERIPHERAL_NODATA};
	uint8_t iodata[4] = {0};
	uint8_t lanes = 0U;
	uint8_t nbytes = 0U;
	uint8_t width = 0U;
	uint32_t data = 0U;

	for (uint8_t count = XEC_P80BD_FIFO_READ_COUNT; count > 0U; count--) {
		uint32_t dattr = cfg->regs->DATRB;
		uint8_t lane;
		uint8_t len;

		if ((dattr & XEC_P80BD_DATRB_NOT_EMPTY) == 0U) {
			break;
		}

		len = (uint8_t)XEC_P80BD_DATRB_LEN_GET(dattr);
		lane = (uint8_t)XEC_P80BD_DATRB_LANE_GET(dattr);

		if (len == XEC_P80BD_DATRB_LEN_ORPHAN) {
			/* Left over from an overrun. Hardware cannot say which
			 * Host write it belonged to, so drop it along with
			 * anything assembled so far.
			 */
			lanes = 0U;
			nbytes = 0U;
			width = 0U;
			continue;
		}

		if (len == XEC_P80BD_DATRB_LEN_1_OF_2) {
			width = 2U;
		} else if (len == XEC_P80BD_DATRB_LEN_1_OF_4) {
			width = 4U;
		}

		if ((len != XEC_P80BD_DATRB_LEN_CONT) && (nbytes != 0U)) {
			/* A new Host write starts here, so the one being
			 * assembled was truncated. Start over from this byte
			 * rather than discard it.
			 */
			lanes = 0U;
			nbytes = 0U;
		} else if (width == 0U) {
			/* A continuation byte with no first byte seen is a one
			 * byte Host write.
			 */
			width = 1U;
		}

		iodata[lane] = (uint8_t)XEC_P80BD_DATRB_DATA_GET(dattr);
		lanes |= BIT(lane);
		nbytes++;

		if (nbytes >= width) {
			break;
		}
	}

	xec_pc_girq_clr(cfg->ecia_info);

	if (lanes == 0U) {
		return;
	}

	for (uint8_t n = 4U; n > 0U; n--) {
		data <<= 8;
		data |= iodata[n - 1U];
	}

	evt.evt_data = data;
	evt.evt_details = ESPI_PERIPHERAL_DEBUG_PORT80 |
			  ((uint32_t)lanes << XEC_P80BD_EVT_LANES_POS) |
			  ((uint32_t)width << XEC_P80BD_EVT_WIDTH_POS);

	mchp_xec_espi_v3_send_callbacks(cfg->espi_dev, evt);
}
#else
/*
 * Report every captured byte as its own event, the way the V2 driver does when
 * CONFIG_ESPI_XEC_P80_MULTIBYTE is off:
 *
 *   evt_data           captured byte, with bit 16 set so a POST code of zero is
 *                      not the ESPI_PERIPHERAL_NODATA value
 *   evt_details[15:0]  ESPI_PERIPHERAL_DEBUG_PORT80
 *   evt_details[23:16] ESPI_PERIPHERAL_INDEX_0 or _1 for byte lane 0 or 1
 *
 * There is no peripheral index for byte lanes 2 and 3, so bytes written to the
 * upper half of the capture range are dropped. Enable
 * CONFIG_ESPI_XEC_V3_PC_P80BD_MULTIBYTE to see them.
 */
static void p80bd_isr(const struct device *dev)
{
	const struct espi_pc_p80bd_xec_config *cfg = dev->config;
	struct espi_event evt = {ESPI_BUS_PERIPHERAL_NOTIFICATION, 0, ESPI_PERIPHERAL_NODATA};

	for (uint8_t count = XEC_P80BD_FIFO_READ_COUNT; count > 0U; count--) {
		uint32_t dattr = cfg->regs->DATRB;
		uint8_t lane;

		if ((dattr & XEC_P80BD_DATRB_NOT_EMPTY) == 0U) {
			break;
		}

		lane = (uint8_t)XEC_P80BD_DATRB_LANE_GET(dattr);
		if (lane > 1U) {
			continue;
		}

		evt.evt_data = (uint32_t)XEC_P80BD_DATRB_DATA_GET(dattr) | BIT(16);
		evt.evt_details = ESPI_PERIPHERAL_DEBUG_PORT80 |
				  ((uint32_t)((lane == 0U) ? ESPI_PERIPHERAL_INDEX_0
							   : ESPI_PERIPHERAL_INDEX_1)
				   << XEC_P80BD_EVT_LANES_POS);

		mchp_xec_espi_v3_send_callbacks(cfg->espi_dev, evt);
	}

	xec_pc_girq_clr(cfg->ecia_info);
}
#endif /* CONFIG_ESPI_XEC_V3_PC_P80BD_MULTIBYTE */

/* Arm the capture block. The Host I/O decoders are the controller's business;
 * everything here belongs to the block and is cleared by the same reset that
 * clears the BARs, so this runs again on every event saying the Host facing
 * hardware is usable.
 */
static void p80bd_block_config(const struct device *dev)
{
	const struct espi_pc_p80bd_xec_config *cfg = dev->config;

	/* Whatever the Host left in the FIFO belongs to the platform state
	 * before the reset, so drop it rather than report stale POST codes.
	 * This also leaves the threshold at one entry, so a single byte Host
	 * write interrupts.
	 */
	cfg->regs->CONFIG = XEC_P80BD_CONFIG_FLUSH_FIFO | XEC_P80BD_CONFIG_CLR_SNAPSHOT;

	cfg->regs->ACTV = XEC_P80BD_ACTV_ENABLE;
	cfg->regs->IEN = XEC_P80BD_IEN_THRESHOLD;

	if (cfg->alias_regs != NULL) {
		cfg->alias_regs->BYTE_LANE = cfg->alias_byte_lane & XEC_P80BD_BYTE_LANE_MSK;
		cfg->alias_regs->ACTV = XEC_P80BD_ACTV_ENABLE;
	}
}

static void p80bd_espi_event(const struct device *espi_dev, const struct device *dev,
			     enum mchp_xec_espi_pc_event evt)
{
	const struct espi_pc_p80bd_xec_config *cfg = dev->config;

	ARG_UNUSED(espi_dev);

	if (!xec_pc_evt_is_hw_usable(evt)) {
		xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
		return;
	}

	/* The controller has re-programmed our Host I/O BARs. */
	p80bd_block_config(dev);

	xec_pc_girq_clr(cfg->ecia_info);
	xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_EN);
}

static int espi_pc_p80bd_xec_init(const struct device *dev)
{
	const struct espi_pc_p80bd_xec_config *cfg = dev->config;
	struct espi_pc_p80bd_xec_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->espi_dev)) {
		LOG_ERR("Port80: eSPI controller not ready");
		return -ENODEV;
	}

	xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	xec_pc_girq_clr(cfg->ecia_info);

	p80bd_block_config(dev);

	data->pc_cb.pc_dev = dev;
	data->pc_cb.handler = p80bd_espi_event;
	data->pc_cb.evt_mask = XEC_PC_EVT_MASK_ALL;

	ret = mchp_xec_espi_pc_register(cfg->espi_dev, &data->pc_cb);
	if (ret != 0) {
		LOG_ERR("Port80: eSPI registration failed (%d)", ret);
		return ret;
	}

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), p80bd_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	xec_pc_girq_ctrl(cfg->ecia_info, MCHP_MEC_ECIA_GIRQ_EN);

	return 0;
}

#define XEC_P80BD_ALIAS_REGS(inst)                                                                 \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, alias_decoder),                                    \
		    ((struct xec_p80bd_alias_regs *)DT_REG_ADDR(                                   \
			     DT_INST_PHANDLE(inst, alias_decoder))),                               \
		    (NULL))

#define XEC_P80BD_ALIAS_BYTE_LANE(inst)                                                            \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, alias_decoder),                                    \
		    (DT_PROP_OR(DT_INST_PHANDLE(inst, alias_decoder), host_io_addr_mask, 0)), (0))

static const struct espi_pc_p80bd_xec_config espi_pc_p80bd_xec_cfg0 = {
	.regs = (struct xec_p80bd_regs *)DT_INST_REG_ADDR(0),
	.alias_regs = XEC_P80BD_ALIAS_REGS(0),
	.espi_dev = DEVICE_DT_GET(DT_INST_PARENT(0)),
	.ecia_info = DT_INST_PROP_BY_IDX(0, girqs, 0),
	.alias_byte_lane = XEC_P80BD_ALIAS_BYTE_LANE(0),
};

static struct espi_pc_p80bd_xec_data espi_pc_p80bd_xec_data0;

DEVICE_DT_INST_DEFINE(0, espi_pc_p80bd_xec_init, NULL, &espi_pc_p80bd_xec_data0,
		      &espi_pc_p80bd_xec_cfg0, POST_KERNEL, CONFIG_ESPI_XEC_V3_PC_INIT_PRIORITY,
		      NULL);
