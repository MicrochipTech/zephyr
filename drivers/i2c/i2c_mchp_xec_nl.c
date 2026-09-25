/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip XEC version 3.8 I2C HW Network-Layer (NL) I2C driver.
 * The XEC I2C supports 7-bit I2C addressing only.
 * The HW supports can act as controller and target. In this driver
 * we support either controller or target at runtime.
 * The NL hardware FSM drives one full I2C transaction (START to STOP)
 * by pulling bytes from a Microchip DMAC channel and pushing read bytes
 * back to it. The transmit byte stream includes the target START and
 * optional RPT-START address bytes.
 */
#include <errno.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_xec_i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/util.h>

/* register defines */
#include "i2c_mchp_xec_regs.h"

LOG_MODULE_REGISTER(i2c_mchp_xec_nl, CONFIG_I2C_LOG_LEVEL);

#define XEC_I2C_NL_SAVE_ELEN_HCMD

/* Sentinel freqhz value: use timing[XEC_I2C_NL_TM_DT] verbatim instead
 * of a bucketed 100k/400k/1M row (I2C_SPEED_DT / a "timing-dt" port).
 */
#define XEC_I2C_NL_FREQ_DT UINT32_MAX

/* Message parser defines */
#define I2C_HW_CTRL_START      BIT(XEC_I2C_HCMD_START0_POS)
#define I2C_HW_CTRL_RPT_START  BIT(XEC_I2C_HCMD_STARTN_POS)
#define I2C_HW_CTRL_STOP       BIT(XEC_I2C_HCMD_STOP_POS)

/* Parser will allow I2C probe (zero-length write) */
#define I2C_HW_ALLOW_ZERO_LEN_WRITE 0

/* XEC I2C-NL HW implements 16-bit write and read count register fields
 * We will use XEC DMA driver block chaining to handle the maximum two
 * messages. Maximum number of blocks should be 4 for two message I2C Write-Read
 * DMA block[0] START address, len=1
 * DMA block[1] msg[0].buf, msg[0].len for data write phase
 * DMA block[2] RPT-START address, len=1
 * DMA block[3] msg[1].buf, msg[1].len for data read phase
 * MAX_RX_SEGS of 2 is only used for a two msg read sequence.
 */
#define I2C_HW_MAX_COUNT       UINT16_MAX
#define I2C_XFER_MAX_TX_SEGS   3
#define I2C_XFER_MAX_RX_SEGS   2

enum i2c_xfer_kind {
	I2C_XFER_WRITE,
	I2C_XFER_READ,
	I2C_XFER_WRITE_WRITE,
	I2C_XFER_READ_READ,
	I2C_XFER_WRITE_READ,
};

struct i2c_xfer_seg {
	uint8_t *buf;
	uint32_t len;
};

struct i2c_xfer_desc {
	enum i2c_xfer_kind kind;
	uint16_t addr;          /* 7-bit target address */
	uint8_t addr_wr;        /* (addr << 1) | 0 -- DMA source, keep in place */
	uint8_t addr_rd;        /* (addr << 1) | 1 -- DMA source, keep in place */

	struct i2c_xfer_seg tx[I2C_XFER_MAX_TX_SEGS];
	uint8_t tx_nseg;
	struct i2c_xfer_seg rx[I2C_XFER_MAX_RX_SEGS];
	uint8_t rx_nseg;

	uint16_t tx_count;      /* value for TX data count register */
	uint16_t rx_count;      /* value for RX data count register */
	uint32_t ctrl;          /* I2C_HW_CTRL_* bits to set before GO */
	uint32_t elen;
};

/* XEC I2C controller timing configuration per frequency */
enum xec_i2c_nl_timing_row {
	XEC_I2C_NL_TIMING_100K,
	XEC_I2C_NL_TIMING_400K,
	XEC_I2C_NL_TIMING_1M,
	XEC_I2C_NL_TIMING_DT,
	XEC_I2C_NL_TIMING_COUNT,
};

struct xec_i2c_timing {
	uint32_t data_timing;
	uint32_t idle_scaling;
	uint32_t timeout_scaling;
	uint16_t bus_clock;
	uint8_t rpt_start_hold_tm;
	uint8_t mr1;
};

/* Controller device configuration */
struct xec_i2c_nl_config {
	uintptr_t regbase;
	const struct device *dma_dev;
	void (*irq_connect)(void);
	uint8_t cm_dma_chan;
	uint8_t cm_dma_slot;
	uint16_t enc_pcr;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t girq_wk;
	uint8_t girq_wk_pos;
	bool has_dt_timing;
	struct xec_i2c_timing timing[XEC_I2C_NL_TIMING_COUNT];
};

/* Controller device driver data */
struct xec_i2c_nl_data {
	const struct device *controller;
	struct k_mutex lock; /* API lock */
	struct k_sem xfr_done; /* init count 0, limit 1 */

	struct i2c_xfer_desc desc;

	uint8_t active_port;
	uint32_t active_freq;
};

/* Port device configuration */
struct xec_i2c_nl_port_config {
	const struct device *controller;
	const struct pinctrl_dev_config *pincfg;
	uint32_t bitrate;
	uint8_t port_id;
	bool is_default;
};

/* Port device driver data */
struct xec_i2c_nl_port_data {
	uint32_t runtime_freq;
};

/* Message parser: Due to I2C-NL HW FSM limitation we limit the driver to a maximum of two
 * struct i2c_msg. Allowed message sequences are:
 * One I2C write or I2C read with mandatory I2C_MSG_STOP flag.
 * I2C probe using a one message with msg0.buf == NULL and msg0.len == 0
 *   I2C-NL write_count = 1, read_count = 0. if ACK on address return success, else error
 * Single transaction START to STOP same direction. Data coming to/from two buffers.
 * Two write msgs:
 *   msg0.flags = I2C_MSG_WRITE and msg1.flags = I2C_MSG_WRITE | I2C_MSG_STOP.
 * Two read msgs:
 *   msg0.flags = I2C_MSG_READ and msg1.flags = I2C_MSG_READ | I2C_MSG_STOP.
 * I2C combined Write-Read where msg0 is write and msg1 is read:
 *   msg0.flags = I2C_MSG_WRITE, msg1.flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP.
 */


/* XEC I2C controller supports 7-bit I2C addressing only */
static inline bool xec_i2c_is_valid_address(uint16_t i2c_address)
{
	if ((i2c_address & 0x7fU) != 0) {
		return false;
	}

	return true;
}

/* Write-1-to-clear the named status bits in the Completion register while
 * preserving its read/write control bits[5:2]. The completion register mixes
 * RW1C status (IDLE, BER, ...) with RW enables (DTEN/HCEN/TCEN/BIDEN) in one
 * word, so a bare sys_write32 of a status constant would also write 0 into
 * those enables.
 */
static inline void xec_i2c_v3_cmpl_clear(uintptr_t base, uint32_t bits)
{
	uint32_t rw = sys_read32(base + XEC_I2C_CMPL_OFS) & XEC_I2C_CMPL_RW_MSK;

	sys_write32(rw | (bits & XEC_I2C_CMPL_RW1C_MSK), base + XEC_I2C_CMPL_OFS);
}

/*
 * Map a Zephyr I2C_SPEED_* value (I2C_SPEED_GET() of an i2c_configure()
 * request) to the Hz value xec_i2c_nl_timing_for() keys its lookup on.
 * Returns 0 for a speed this HW's timing table has no row for
 * (I2C_SPEED_HIGH/ULTRA -- this NL engine tops out at timing_1000k) or
 * an unrecognized value; 0 is never a valid return otherwise.
 */
static uint32_t xec_i2c_nl_speed_to_freq(uint32_t speed)
{
	switch (speed) {
	case I2C_SPEED_STANDARD:
		return KHZ(100);
	case I2C_SPEED_FAST:
		return KHZ(400);
	case I2C_SPEED_FAST_PLUS:
		return MHZ(1);
	case I2C_SPEED_DT:
		return XEC_I2C_NL_FREQ_DT;
	default:
		return 0U;
	}
}

/* Inverse of xec_i2c_nl_speed_to_freq(), for i2c_get_config(); same
 * bucketing xec_i2c_nl_timing_for() uses so the two never disagree.
 */
static uint32_t xec_i2c_nl_freq_to_speed(uint32_t freqhz)
{
	if (freqhz == XEC_I2C_NL_FREQ_DT) {
		return I2C_SPEED_DT;
	}
	if (freqhz <= KHZ(100)) {
		return I2C_SPEED_STANDARD;
	}
	if (freqhz <= KHZ(400)) {
		return I2C_SPEED_FAST;
	}
	return I2C_SPEED_FAST_PLUS;
}

/*
 * Frequency a port should (re)program the controller with, in
 * precedence order: this port's i2c_configure()-set runtime override,
 * else its own DT clock-frequency, else the controller's DT default.
 */
static uint32_t xec_i2c_nl_port_freq(const struct xec_i2c_nl_port_config *port_cfg,
				      const struct xec_i2c_nl_port_data *port_data)
{
	const struct xec_i2c_nl_config *ctrl_cfg = port_cfg->controller->config;

	if (port_data->runtime_freq != 0U) {
		return port_data->runtime_freq;
	}
	if (port_cfg->bitrate != 0U) {
		return port_cfg->bitrate;
	}
	return ctrl_cfg->dflt_freq;
}

static const struct xec_i2c_nl_timing *
xec_i2c_nl_timing_for(const struct xec_i2c_nl_config *cfg, uint32_t freqhz)
{
	if (freqhz == XEC_I2C_NL_FREQ_DT) {
		return &cfg->timing[XEC_I2C_NL_TIMING_DT];
	}
	if (freqhz <= KHZ(100)) {
		return &cfg->timing[XEC_I2C_NL_TIMING_100K];
	}
	if (freqhz <= KHZ(400)) {
		return &cfg->timing[XEC_I2C_NL_TIMING_400K];
	}
	return &cfg->timing[XEC_I2C_NL_TIMING_1M];
}

/* Configure controller timings, frequency, and port.
 * Reset the controller using the XEC PCR peripheral reset.
 */
static int xec_i2c_nl_program_ctrl(const struct xec_i2c_nl_config *ctrl_cfg,
				   struct xec_i2c_nl_data *ctrl_data, uint32_t freq,
				   uint8_t port_id)
{
	uintptr_t regbase = ctrl_cfg->regbase;
	int rc = 0;

	/* TODO */

	return rc;
}
/* Select controller port and/or frequency
 * If port and frequency are what we want do nothing else
 * reconfigure the controller for new port and/or frequeny
 * Note controller requires reset on port/freq change
 */
static int xec_i2c_nl_apply_port(const struct xec_i2c_nl_port_config *port_cfg,
				 struct xec_i2c_nl_port_data *port_data,
				 const struct xec_i2c_nl_config *ctrl_cfg,
				 struct xec_i2c_nl_data *ctrl_data);

static int xec_i2c_nl_apply_port(const struct device *port_dev)
{
	uint32_t freq = xec_i2c_nl_port_freq(port_cfg, port_data);
	uint8_t port = port_cfg->port_id;
	int rc = 0;

	if ((ctrl_data->active_port == port) && (ctrl_data->active_freq == freq)) {
		return 0;
	}

	rc = pinctrl_apply_state(port_cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("I2C-NL apply port pincfg error (%d)", rc);
		return rc;
	}

	return xec_i2c_nl_program_ctrl(ctrl_cfg, ctrl_data, freq, port);
}

/* API: configuration */
static int xec_i2c_nl_vport_config(const struct device *port_dev, uint32_t i2c_config)
{
	/* TODO */
	return 0;
}

/* API: get configuration */
static int xec_i2c_nl_vport_get_config(const struct device *port_dev, uint32_t *i2c_config)
{
	/* TODO */
	if (i2c_config == NULL) {
		return -EINVAL;
	}

	return 0;
}
/* API: bus recovery */
static int xec_i2c_nl_vport_recover_bus(const struct device *port_dev)
{
	/* TODO */
	return 0;
}

/* Message parsing */
static inline bool msg_is_read(const struct i2c_msg *m)
{
	return (m->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ;
}

static int check_msg(const struct i2c_msg *m, bool allow_empty)
{
	if (m->flags & I2C_MSG_ADDR_10_BITS) {
		return -ENOTSUP;
	}
	if (m->len == 0U) {
		return allow_empty ? 0 : -EINVAL;
	}
	if (m->buf == NULL) {
		return -EINVAL;
	}
	return 0;
}

/* Append a segment; zero-length segments are skipped (no empty DMA blocks). */
static void push_tx(struct i2c_xfer_desc *d, uint32_t *total, uint8_t *buf, uint32_t len)
{
	if (len == 0U) {
		return;
	}
	d->tx[d->tx_nseg].buf = buf;
	d->tx[d->tx_nseg].len = len;
	d->tx_nseg++;
	*total += len;
}

static void push_rx(struct i2c_xfer_desc *d, uint32_t *total, uint8_t *buf, uint32_t len)
{
	if (len == 0U) {
		return;
	}
	d->rx[d->rx_nseg].buf = buf;
	d->rx[d->rx_nseg].len = len;
	d->rx_nseg++;
	*total += len;
}

static int parse_one(const struct i2c_msg *m0, struct i2c_xfer_desc *d,
		     uint32_t *tx_tot, uint32_t *rx_tot)
{
	int rc;

	/* Missing STOP only happens with CONFIG_I2C_ALLOW_NO_STOP_TRANSACTIONS */
	if (!(m0->flags & I2C_MSG_STOP)) {
		return -ENOTSUP;
	}

	d->ctrl = I2C_HW_CTRL_START | I2C_HW_CTRL_STOP;

	if (msg_is_read(m0)) {
		rc = check_msg(m0, false);
		if (rc) {
			return rc;
		}
		d->kind = I2C_XFER_READ;
		push_tx(d, tx_tot, &d->addr_rd, 1U);
		push_rx(d, rx_tot, m0->buf, m0->len);
	} else {
		rc = check_msg(m0, I2C_HW_ALLOW_ZERO_LEN_WRITE);
		if (rc) {
			return rc;
		}
		d->kind = I2C_XFER_WRITE;
		push_tx(d, tx_tot, &d->addr_wr, 1U);
		push_tx(d, tx_tot, m0->buf, m0->len);
	}
	return 0;
}

static int parse_two(const struct i2c_msg *m0, const struct i2c_msg *m1,
		     struct i2c_xfer_desc *d,
		     uint32_t *tx_tot, uint32_t *rx_tot)
{
	bool r0 = msg_is_read(m0);
	bool r1 = msg_is_read(m1);
	int rc;

	/* STOP only on the second message */
	if ((m0->flags & I2C_MSG_STOP) || !(m1->flags & I2C_MSG_STOP)) {
		return -ENOTSUP;
	}

	rc = check_msg(m0, false);
	if (rc) {
		return rc;
	}
	rc = check_msg(m1, false);
	if (rc) {
		return rc;
	}

	if (!r0 && r1) {
		/* Write then read: repeated start required on the read */
		if (!(m1->flags & I2C_MSG_RESTART)) {
			return -ENOTSUP;
		}
		d->kind = I2C_XFER_WRITE_READ;
		d->ctrl = I2C_HW_CTRL_START | I2C_HW_CTRL_RPT_START |
			  I2C_HW_CTRL_STOP;
		push_tx(d, tx_tot, &d->addr_wr, 1U);
		push_tx(d, tx_tot, m0->buf, m0->len);
		push_tx(d, tx_tot, &d->addr_rd, 1U);
		push_rx(d, rx_tot, m1->buf, m1->len);
		return 0;
	}

	if (r0 == r1) {
		/* Same direction: one continuous transfer, no repeated start */
		if (m1->flags & I2C_MSG_RESTART) {
			return -ENOTSUP;
		}
		d->ctrl = I2C_HW_CTRL_START | I2C_HW_CTRL_STOP;
		if (r0) {
			d->kind = I2C_XFER_READ_READ;
			push_tx(d, tx_tot, &d->addr_rd, 1U);
			push_rx(d, rx_tot, m0->buf, m0->len);
			push_rx(d, rx_tot, m1->buf, m1->len);
		} else {
			d->kind = I2C_XFER_WRITE_WRITE;
			push_tx(d, tx_tot, &d->addr_wr, 1U);
			push_tx(d, tx_tot, m0->buf, m0->len);
			push_tx(d, tx_tot, m1->buf, m1->len);
		}
		return 0;
	}

	/* Read then write: second direction change, not supported */
	return -ENOTSUP;
}

int xec_i2c_xfer_parse(const struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr,
		       struct i2c_xfer_desc *desc)
{
	uint32_t tx_tot = 0U;
	uint32_t rx_tot = 0U;
	int rc;

	if ((msgs == NULL) || (desc == NULL) || (num_msgs == 0U)) {
		return -EINVAL;
	}
	/* 10-bit addressing is rejected per message; addr must be 7-bit */
	if (addr > 0x7FU) {
		return -EINVAL;
	}

	memset(desc, 0, sizeof(*desc));
	desc->addr = addr;
	desc->addr_wr = (uint8_t)(addr << 1);
	desc->addr_rd = (uint8_t)((addr << 1) | 1U);

	switch (num_msgs) {
	case 1:
		rc = parse_one(&msgs[0], desc, &tx_tot, &rx_tot);
		break;
	case 2:
		rc = parse_two(&msgs[0], &msgs[1], desc, &tx_tot, &rx_tot);
		break;
	default:
		rc = -ENOTSUP;
		break;
	}
	if (rc) {
		return rc;
	}

	/* Totals include address bytes on TX; each must fit a 16-bit count */
	if ((tx_tot > I2C_HW_MAX_COUNT) || (rx_tot > I2C_HW_MAX_COUNT)) {
		return -EINVAL;
	}
	desc->tx_count = (uint16_t)tx_tot;
	desc->rx_count = (uint16_t)rx_tot;
	return 0;
}

/* Build DMA blocks required for a transaction */
void xec_i2c_nl_xfer_build_dma_blocks(const struct i2c_xfer_desc *desc,
				      uintptr_t tx_data_reg, uintptr_t rx_data_reg,
				      struct dma_block_config tx_blk[I2C_XFER_MAX_TX_SEGS],
				      struct dma_block_config rx_blk[I2C_XFER_MAX_RX_SEGS])
{
	for (uint8_t i = 0; i < desc->tx_nseg; i++) {
		struct dma_block_config *b = &tx_blk[i];

		memset(b, 0, sizeof(*b));
		b->source_address = (uintptr_t)desc->tx[i].buf;
		b->dest_address = tx_data_reg;
		b->block_size = desc->tx[i].len;
		b->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		b->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		b->next_block = (i + 1U < desc->tx_nseg) ? &tx_blk[i + 1U] : NULL;
	}

	for (uint8_t i = 0; i < desc->rx_nseg; i++) {
		struct dma_block_config *b = &rx_blk[i];

		memset(b, 0, sizeof(*b));
		b->source_address = rx_data_reg;
		b->dest_address = (uintptr_t)desc->rx[i].buf;
		b->block_size = desc->rx[i].len;
		b->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		b->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		b->next_block = (i + 1U < desc->rx_nseg) ? &rx_blk[i + 1U] : NULL;
	}
}

/* my_i2c_hw_set_counts */
static void xec_i2c_nl_hw_set_counts(const struct xec_i2c_nl_config *ctrl_cfg, uint16_t tx,
				     uint16_t rx)
{
	uintptr_t rb = ctrl_cfg->regbase;
	uint32_t lc = XEC_I2C_HCMD_WCL_SET(tx);
	uint32_t hc = XEC_I2C_ELEN_HWR_SET(tx >> 8);

	lc |= XEC_I2C_HCMD_RCL_SET(rx);
	hc |= XEC_I2C_ELEN_HRD_SET(rx >> 8);

	soc_mmcr_mask_set(rb + XEC_I2C_ELEN_OFS, hc, XEC_I2C_ELEN_HWR_MSK | XEC_I2C_ELEN_HRD_MSK);
	soc_mmcr_mask_set(rb + XEC_I2C_HCMD_OFS, lc, XEC_I2C_HCMD_WCL_MSK | XEC_I2C_HCMD_RCL_MSK);
}

#define XEC_HCMD_NO_CNT_MSK 0xffffU
/* my_i2c_hw_set_ctrl_and_go */
static void xec_i2c_nl_hw_set_ctrl_and_go(const struct xec_i2c_nl_config *ctrl_cfg, uint32_t ctrl)
{
	uintptr_t rb = ctrl_cfg->regbase;

	soc_mmcr_mask_set(rb + XEC_I2C_HCMD_OFS, ctrl, XEC_HCMD_NO_CNT_MSK); 
}

/* Is this enough? my_i2c_abort_hw */
static void xec_i2c_nl_abort_hw(const struct xec_i2c_nl_config *ctrl_cfg,
				struct xec_i2c_nl_data *ctrl_data)
{
	dma_stop(ctrl_cfg->cm_dma_chan);
	sys_write32(0, ctrl_cfg->regbase + XEC_I2C_HCMD_OFS);
}

/* my_i2c_start_hw */
static int xec_i2c_nl_start_hw(const struct xec_i2c_nl_config *ctrl_cfg,
			       struct xec_i2c_nl_data *ctrl_data)
{
	const struct i2c_xfer_desc *d = &data->desc;
	/* Stack is fine: the DMA driver consumes these inside dma_config() */
	struct dma_block_config tx_blk[I2C_XFER_MAX_TX_SEGS];
	struct dma_block_config rx_blk[I2C_XFER_MAX_RX_SEGS];
	int rc;

	xec_i2c_nl_xfer_build_dma_blocks(d, cfg->tx_data_reg, cfg->rx_data_reg, tx_blk, rx_blk);

	/* RX first so it's armed before the controller starts clocking */
	if (d->rx_nseg != 0) {
		struct dma_config rx_cfg = {
			.dma_slot = cfg->rx_dma_slot,
			.channel_direction = PERIPHERAL_TO_MEMORY,
			.source_data_size = 1,
			.dest_data_size = 1,
			.source_burst_length = 1,
			.dest_burst_length = 1,
			.block_count = d->rx_nseg,
			.head_block = &rx_blk[0],
		};

		rc = dma_config(cfg->dma_dev, cfg->rx_dma_chan, &rx_cfg);
		if (rc == 0) {
			rc = dma_start(cfg->dma_dev, cfg->rx_dma_chan);
		}
		if (rc) {
			return rc;
		}
	}

	struct dma_config tx_cfg = {
		.dma_slot = cfg->tx_dma_slot,
		.channel_direction = MEMORY_TO_PERIPHERAL,
		.source_data_size = 1,
		.dest_data_size = 1,
		.source_burst_length = 1,
		.dest_burst_length = 1,
		.block_count = d->tx_nseg,
		.head_block = &tx_blk[0],
	};

	rc = dma_config(cfg->dma_dev, cfg->tx_dma_chan, &tx_cfg);
	if (rc == 0) {
		rc = dma_start(cfg->dma_dev, cfg->tx_dma_chan);
	}
	if (rc != 0) {
		if (d->rx_nseg != 0) {
			dma_stop(cfg->dma_dev, cfg->rx_dma_chan);
		}
		return rc;
	}

	xec_i2c_nl_hw_set_counts(ctrl_cfg, d->tx_count, d->rx_count);
	xec_i2c_nl_hw_set_ctrl_and_go(ctrl_cfg, d->ctrl);

	return 0;
}

/* API: synchronous transfer
 * NOTE: msgs == NULL and num_msgs == 0 is bus ping
 * Transmit target write address and no data
 * If NAK return -ENXIO
 * if ACK return 0 (success)
 * HCMD: RUN=1, PROCEED=1, START0=1, STARTN=0, STOP=0, wrCnt=1, rdCnt=0
 */
static int xec_i2c_nl_vport_xfr(const struct device *port_dev, struct i2c_msg *msgs,
				uint8_t num_msgs, uint16_t i2c_address)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	const struct xec_i2c_nl_config *ctrl_cfg = port_cfg->controller->config;
	struct xec_i2c_nl_data *ctrl_data = port_cfg->controller->data;

	if (!xec_i2c_is_valid_address(i2c_address)) {
		return -EINVAL;
	}

	if (msgs == NULL) {
		if (num_msgs != 0) {
			return -EINVAL;
		}
	} else if (num_msgs == 0) {
		return 0; /* nothing to do */
	}

	/* TODO */

	return 0;
}

#ifdef CONFIG_I2C_CALLBACK
/* API: asynchronous transfer */
#endif

#ifdef CONFIG_I2C_TARGET
/* API: target register */

/* API: target unregister */
#endif

/* Port driver initialization */
static int xec_i2c_nl_port_init(const struct device *port_dev)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	struct xec_i2c_nl_port_data *port_data = port_dev->data;
	const struct device *ctrl_dev = port_cfg->controller;
	const struct xec_i2c_nl_config *ctrl_cfg = ctrl_dev->config;
	struct xec_i2c_nl_data *ctrl_data = ctrl_dev->data;

	return xec_i2c_nl_apply_port(port_cfg, port_data, ctrl_cfg, ctrl_data);
}

/* Controller driver initialization */
static int xec_i2c_nl_ctrl_init(const struct device *ctrl_dev)
{
	struct xec_i2c_nl_data *const ctrl_data = ctrl_dev->data;

	k_mutex_init(&ctrl_data->lock);
	k_sem_init(&ctrl_data->xfr_done, 0, 1);

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int xec_i2c_nl_vport_pm_action_cb(const struct device *port_dev,
					 enum pm_device_action action)
{
	int rc = 0;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		/* TODO */
		break;
	case PM_DEVICE_ACTION_RESUME:
		/* TODO */
		break;
	case PM_DEVICE_ACTION_TURN_OFF:
		/* TODO */
		break;
	case PM_DEVICE_ACTION_TURN_ON:
		/* TODO */
		break;
	default:
		return -EINVAL;
	}

	return rc;
}
#endif

static DEVICE_API(i2c, xec_i2c_nl_port_api) = {
	.configure = xec_i2c_nl_vport_config,
	.get_config = xec_i2c_nl_vport_get_config,
	.transfer = xec_i2c_nl_vport_xfr,
	.recover_bus = xec_i2c_nl_vport_recover_bus,
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = xec_i2c_nl_vport_xfr_cb,
#endif
#ifdef CONFIG_I2C_TARGET
	.target_register = xec_i2c_nl_vport_target_register,
	.target_unregister = xec_i2c_nl_vport_target_unregister,
#endif
};

/* Port device instances */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT microchip_xec_i2c_nl_port

#if defined(CONFIG_PM_DEVICE)
#define XEC_I2C_V3_NL_VPORT_PM_DEVICE_DT_INST_DEFINE(inst) \
	PM_DEVICE_DT_INST_DEFINE(inst, xec_i2c_v3_nl_vport_pm_action_cb);
#else
#define XEC_I2C_V3_NL_VPORT_PM_DEVICE_DT_INST_DEFINE(inst)
#endif
/* A port is the boot/default routing if and only if its controller's default-port
 * phandle names this port node. Controllers without a default-port yield 0
 * (no boot pre-routing; the first transfer applies its own port lazily).
 */
#define XEC_I2C_NL_PORT_IS_DEFAULT(inst)                                                           \
	COND_CODE_1(DT_NODE_HAS_PROP(DT_INST_PHANDLE(inst, controller), default_port),             \
		    (DT_SAME_NODE(DT_DRV_INST(inst),                                               \
				  DT_PHANDLE(DT_INST_PHANDLE(inst, controller), default_port))),   \
		    (0))

#define XEC_I2C_NL_PORT_INIT(inst)                                                                 \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static const struct xec_i2c_nl_port_config xec_i2c_nl_port_cfg_##inst = {                  \
		.controller = DEVICE_DT_GET(DT_INST_PHANDLE(inst, controller)),                    \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		.bitrate = DT_INST_PROP_OR(inst, clock_frequency, I2C_BITRATE_STANDARD),           \
		.port_id = (uint8_t)(DT_INST_PROP(inst, port) & 0x0FU),                            \
		.is_default = XEC_I2C_NL_PORT_IS_DEFAULT(inst),                                    \
	};                                                                                         \
	static struct xec_i2c_nl_port_data xec_i2c_nl_port_xdat_##inst;                            \
	PM_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_vport_pm_action_cb);                             \
	I2C_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_port_init, PM_DEVICE_DT_INST_GET(inst),         \
				  NULL, &xec_i2c_nl_port_cfg_##inst,                               \
				  POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,                           \
				  &xec_i2c_nl_port_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_PORT_INIT)
