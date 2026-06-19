/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip XEC I2Cv3 Network-Layer (NL) I2C driver.
 *
 * The NL hardware FSM drives one full I2C transaction (START to STOP)
 * by pulling bytes from a Microchip DMAC channel and pushing read bytes
 * back to it. Software builds a contiguous TX bounce buffer of the form
 *
 *     [ wr-addr | wr-data... | rd-addr ]
 *
 * (the trailing rd-addr byte is omitted on a write-only transfer),
 * configures the DMA channel for MEMORY_TO_PERIPHERAL targeting the
 * controller's HTX register, and writes the host-command (HCMD) register.
 * The HW then:
 *
 *   write-only:  pulls (1 + wr_len) bytes via DMA, drives
 *                START -> wr-addr -> wr-data... -> STOP, fires HDONE.
 *   write-read:  pulls (1 + wr_len + 1) bytes via DMA, drives
 *                START -> wr-addr -> wr-data... -> Sr -> rd-addr,
 *                then PAUSEs (the HW clears HCMD.PROCEED) so software
 *                can reprogram the DMA channel for PERIPHERAL_TO_MEMORY
 *                targeting the user's RX buffer. Software sets
 *                HCMD.PROCEED, the HW clocks rd_len bytes into memory
 *                via DMA and drives STOP. HDONE fires both for the
 *                PAUSE event and the final STOP.
 *   read-only:   same as write-read with wr_len == 0; the bounce buffer
 *                holds just the rd-addr byte.
 *
 * Per the v3.8 errata captured in the repository README, on a read the HW
 * fires HDONE the moment the last byte is clocked off the bus while the
 * DMA channel may still be moving that byte to memory. To avoid that
 * race the driver enables the controller's NL-mode STOP-detect interrupt
 * (CFG.STD_NL_IEN, source bit SR.STO) and uses STOP-detect as the
 * "transfer complete" signal. STOP-detect fires after the controller has
 * driven STOP on the bus AND the bus has returned to idle, which
 * post-dates both HDONE and the DMA-done callback in either direction.
 * HDONE is then only used to detect the PAUSE state on write-then-read
 * (and pure-read) transfers.
 *
 * Per the same notes, the HCMD/TCMD/ELEN registers are live-updated by
 * the HW once started, so software touches them only when the FSM is
 * idle (before kickoff) or in PAUSE state (PROCEED cleared).
 *
 * Multi-message i2c_transfer() calls map onto the NL FSM by collapsing
 * the message array into a single START-to-STOP transaction:
 *
 *   - Up to N consecutive write descriptors are concatenated into the
 *     bounce buffer (after the wr-addr byte) and clocked out in one
 *     write phase.
 *   - Up to M consecutive read descriptors follow. If M == 1 the read
 *     DMA targets the user buffer directly; if M > 1 the read DMA
 *     targets the bounce buffer (the TX phase has finished by the
 *     time the read phase starts) and the driver scatters the
 *     received bytes into the M user buffers via memcpy after STOP-
 *     detect. The XEC DMA controller does not support multi-block
 *     scatter, and reload-while-busy isn't permitted, so a single
 *     contiguous DMA + post-scatter is the only correct shape.
 *
 * Anything outside the (writes-then-reads, single START, single STOP)
 * shape returns -ENOTSUP.
 *
 * Limitations of this driver:
 *   - controller mode only; no I2C target.
 *   - synchronous transfer only (no CONFIG_I2C_CALLBACK).
 *   - 7-bit addresses only.
 *   - the bounce buffer must accommodate both 1 + sum(wr_len) + 1 (TX
 *     side) and, for M > 1 reads, sum(rd_len) (RX scatter staging).
 *     Sized per-instance via the bounce-buffer-size devicetree
 *     property; oversize transfers return -ENOSPC.
 */

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
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(i2c_mchp_xec_v3_nl, CONFIG_I2C_LOG_LEVEL);

#include "i2c_mchp_xec_regs.h"

/* Status / command bit shorthands ------------------------------------------*/

#define CMPL_HDONE BIT(XEC_I2C_CMPL_HDONE_POS)
#define CMPL_HNAK  BIT(XEC_I2C_CMPL_HNAKX_POS)
#define CMPL_LAB   BIT(XEC_I2C_CMPL_LAB_STS_POS)
#define CMPL_BER   BIT(XEC_I2C_CMPL_BER_STS_POS)
#define CMPL_ERR   (CMPL_HNAK | CMPL_LAB | CMPL_BER)

#define HCMD_RUN     BIT(XEC_I2C_HCMD_RUN_POS)
#define HCMD_PROCEED BIT(XEC_I2C_HCMD_PROC_POS)
#define HCMD_START0  BIT(XEC_I2C_HCMD_START0_POS)
#define HCMD_STARTN  BIT(XEC_I2C_HCMD_STARTN_POS)
#define HCMD_STOP    BIT(XEC_I2C_HCMD_STOP_POS)

/* Read-only legacy status register at offset 0; bit 5 latches a STOP
 * condition detected on the bus when CFG.STD_NL_IEN is enabled.
 */
#define SR_STO BIT(XEC_I2C_SR_STO_POS)

/* WCL/RCL are 8 bits in HCMD; ELEN.HWR/HRD extend each by another 8 bits. */
#define XEC_I2C_NL_LEN_MAX 0xFFFFU

/* I2C-bus R/W bit, the LSB of the address byte. 0 = write, 1 = read. */
#define XEC_I2C_NL_RWBIT_WRITE 0x00U
#define XEC_I2C_NL_RWBIT_READ  0x01U

#define XEC_I2C_NL_INVALID_PORT 0xFFU

#define XEC_I2C_NL_TIMEOUT K_MSEC(1000)

/* Default I2C control-register value: ESO+ACK+PIN. PIN is also raised at
 * reset to clear any latent PIN-asserted state in the legacy I2C engine.
 */
#define XEC_I2C_NL_CR_DFLT \
	(BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS) | BIT(XEC_I2C_CR_PIN_POS))

enum xec_i2c_nl_state {
	XEC_I2C_NL_IDLE,
	XEC_I2C_NL_TX,
	XEC_I2C_NL_RX,
};

#define I2C_DIR_NONE 0
#define I2C_DIR_WR   1U
#define I2C_DIR_RD   2U

struct xec_i2c_nl_mgrp {
	struct i2c_msg *mwr;
	struct i2c_msg *mrd;
	uint16_t nwr;
	uint16_t nrd;
	uint8_t *bufptr;
	uint32_t remlen;
};

/* Controller bus-clock and timing rows for a 16 MHz BAUD clock. Source:
 * Microchip I2C-SMBus controller v3.8 datasheet.
 */
struct xec_i2c_nl_timing {
	uint32_t data_timing;
	uint32_t idle_scaling;
	uint32_t timeout_scaling;
	uint16_t bus_clock;
	uint8_t mr1;
};

static const struct xec_i2c_nl_timing xec_i2c_nl_timing_tbl[] = {
	{	/* 100 kHz, 50/50 duty */
		.data_timing     = 0x0C4D5006U,
		.idle_scaling    = 0x01FC01EDU,
		.timeout_scaling = 0x4B9CC2C7U,
		.bus_clock       = 0x4F4FU,
		.mr1             = 0x05U,
	},
	{	/* 400 kHz, lo:hi ~ 1.53 */
		.data_timing     = 0x040A0A06U,
		.idle_scaling    = 0x01000050U,
		.timeout_scaling = 0x159CC2C7U,
		.bus_clock       = 0x0F17U,
		.mr1             = 0x05U,
	},
	{	/* 1 MHz, lo:hi ~ 1.8 */
		.data_timing     = 0x04060601U,
		.idle_scaling    = 0x01000050U,
		.timeout_scaling = 0x089CC2C7U,
		.bus_clock       = 0x0509U,
		.mr1             = 0x05U,
	},
};

struct xec_i2c_nl_config {
	mm_reg_t base;
	const struct device *dma_dev;
	void (*irq_connect)(void);
	uint8_t *bounce_buf;
	size_t bounce_buf_size;
	uint32_t dflt_freq;
	uint8_t girq;
	uint8_t girq_pos;
	uint16_t enc_pcr;
	uint8_t dma_chan;
	uint8_t dma_slot;
};

struct xec_i2c_nl_data {
	struct k_sem lock;
	struct k_sem pause_sem;
	struct k_sem done_sem;

	enum xec_i2c_nl_state state;
	int xfer_err;

	uint8_t active_port;	/* XEC_I2C_NL_INVALID_PORT until programmed */
	uint32_t active_freq;

	struct xec_i2c_nl_mgrp mgrp;
};

struct xec_i2c_nl_port_config {
	const struct device *parent;
	const struct pinctrl_dev_config *pcfg;
	uint32_t bitrate;
	uint8_t port_id;
	bool is_default;
};

/* Parsed summary of an i2c_transfer() request after flag/shape validation.
 * Filled by xec_i2c_nl_parse and consumed by xec_i2c_nl_run.
 */
struct xec_i2c_nl_xfer {
	struct i2c_msg *msgs;
	uint8_t num_msgs;
	uint8_t first_read;     /* index of first read msg, or num_msgs if none */
	uint16_t total_wr_len;  /* sum of write-msg lens (capped at LEN_MAX)    */
	uint16_t total_rd_len;  /* sum of read-msg lens                         */
	bool has_read;
	bool rx_via_bounce;     /* true when M > 1 — DMA can't scatter, so the
				 * read phase lands in the bounce buffer and is
				 * memcpy'd into the user buffers afterward.
				 */
};

/* -------------------------------------------------------------------------
 * Controller helpers
 * -------------------------------------------------------------------------*/

static const struct xec_i2c_nl_timing *xec_i2c_nl_timing_for(uint32_t freqhz)
{
	if (freqhz <= KHZ(100)) {
		return &xec_i2c_nl_timing_tbl[0];
	}
	if (freqhz <= KHZ(400)) {
		return &xec_i2c_nl_timing_tbl[1];
	}
	return &xec_i2c_nl_timing_tbl[2];
}

/* Full controller programming: PCR reset, GIRQ enable, port select, timing,
 * and HDONE interrupt enable. Called from ctrl_init and whenever vport
 * configure changes the bus frequency.
 *
 * Must only be called when no transfer is in flight (lock held by caller,
 * or before any transfers are issued).
 */
static int xec_i2c_nl_program_ctrl(const struct device *ctrl, uint32_t freqhz, uint8_t port)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	const struct xec_i2c_nl_timing *tm = xec_i2c_nl_timing_for(freqhz);
	mm_reg_t base = cfg->base;

	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, MCHP_MEC_ECIA_GIRQ_DIS);
	soc_xec_pcr_reset_en(cfg->enc_pcr);
	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);

	/* PIN=1 to clear any latent assertion left by the legacy engine. */
	sys_write8(BIT(XEC_I2C_CR_PIN_POS), base + XEC_I2C_CR_OFS);

	/* Port select, filters on, general-call disabled, HDONE interrupt
	 * enabled, NL-mode STOP-detect interrupt enabled. ENAB is set last
	 * after timing has been written.
	 *
	 * STD_NL_IEN gates the v3.8 NL-mode STOP-detect interrupt. When set,
	 * the controller fires its line every time SR[5]=STO transitions
	 * high (i.e., the bus enters its idle state after a STOP). The
	 * driver uses this — not HDONE — as the transfer-complete signal:
	 * STOP-detect post-dates both HDONE and the DMA-done callback, so
	 * waiting on it avoids the read-direction race captured in the
	 * repository README.
	 */
	sys_write32(XEC_I2C_CFG_PORT_SET(port) |
			BIT(XEC_I2C_CFG_FEN_POS) |
			BIT(XEC_I2C_CFG_GC_DIS_POS) |
			BIT(XEC_I2C_CFG_HD_IEN_POS) |
			BIT(XEC_I2C_CFG_STD_NL_IEN_POS),
		    base + XEC_I2C_CFG_OFS);

	sys_write32(tm->data_timing, base + XEC_I2C_DT_OFS);
	sys_write32(tm->idle_scaling, base + XEC_I2C_ISC_OFS);
	sys_write32(tm->timeout_scaling, base + XEC_I2C_TMOUT_SC_OFS);
	sys_write32((uint32_t)tm->bus_clock, base + XEC_I2C_BCLK_OFS);
	sys_write32((uint32_t)tm->mr1, base + XEC_I2C_MR1_OFS);

	sys_write8(XEC_I2C_NL_CR_DFLT, base + XEC_I2C_CR_OFS);
	sys_set_bit(base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);

	/* Bus may already be idle from PCR reset, so SR.STO may be high.
	 * Clear the GIRQ status one more time before unmasking, so that
	 * latch can't ride into NVIC the moment we enable.
	 */
	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);
	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, MCHP_MEC_ECIA_GIRQ_EN);

	data->active_freq = freqhz;
	data->active_port = port;

	return 0;
}

static int xec_i2c_nl_apply_port(const struct device *port_dev)
{
	const struct xec_i2c_nl_port_config *pc = port_dev->config;
	const struct xec_i2c_nl_config *cfg = pc->parent->config;
	struct xec_i2c_nl_data *data = pc->parent->data;
	int rc;

	if (data->active_port == pc->port_id) {
		return 0;
	}

	rc = pinctrl_apply_state(pc->pcfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("pinctrl_apply_state(%s)=%d", port_dev->name, rc);
		return rc;
	}

	soc_mmcr_mask_set(cfg->base + XEC_I2C_CFG_OFS,
			  XEC_I2C_CFG_PORT_SET(pc->port_id),
			  XEC_I2C_CFG_PORT_MSK);
	data->active_port = pc->port_id;
	return 0;
}

/* Reset the controller hard via PCR and re-arm it.
 *
 * On real silicon, a bus error (HNAK / LAB / BER) leaves the I2C engine in a
 * state where the GIRQ status latch and/or the CMPL R/W1C bits cannot be
 * fully cleared by writing 1s alone — the next vport_transfer then sees a
 * spurious HDONE-looking ISR fire as soon as the GIRQ is unmasked. The only
 * reliable recovery is a peripheral-level reset via the PCR block, which
 * clears all internal latches; xec_i2c_nl_program_ctrl runs that reset
 * (soc_xec_pcr_reset_en) and re-applies our configuration on the way out,
 * including a fresh soc_ecia_girq_status_clear and GIRQ enable.
 *
 * Restore the previously-active port if known, otherwise fall back to port 0
 * — and invalidate active_port so the next transfer re-applies pinctrl for
 * its own port.
 */
static void xec_i2c_nl_abort(const struct device *ctrl)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	uint32_t freq = (data->active_freq != 0U) ? data->active_freq : cfg->dflt_freq;
	uint8_t port = (data->active_port == XEC_I2C_NL_INVALID_PORT) ? 0U : data->active_port;

	dma_stop(cfg->dma_dev, cfg->dma_chan);
	(void)xec_i2c_nl_program_ctrl(ctrl, freq, port);
	data->active_port = XEC_I2C_NL_INVALID_PORT;
}

/* -------------------------------------------------------------------------
 * DMA callbacks
 * -------------------------------------------------------------------------*/

static void xec_i2c_nl_tx_dma_cb(const struct device *dma_dev, void *user_data,
				 uint32_t channel, int status)
{
	struct xec_i2c_nl_data *data = user_data;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status >= 0) {
		/* DMA-done on TX merely means the bytes have been pushed into
		 * the controller's transmit register. The I2C HW is still
		 * clocking them out and will fire HDONE (or, on write+read,
		 * HDONE for PAUSE) when its work is done.
		 */
		return;
	}

	data->xfer_err = status;
	/* Wake whichever waiter is parked on this transfer. */
	k_sem_give(&data->pause_sem);
	k_sem_give(&data->done_sem);
}

static void xec_i2c_nl_rx_dma_cb(const struct device *dma_dev, void *user_data,
				 uint32_t channel, int status)
{
	struct xec_i2c_nl_data *data = user_data;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		data->xfer_err = status;
		k_sem_give(&data->done_sem);
		return;
	}

	/* DMA-done on success is not the completion signal — STOP-detect
	 * via SR.STO will fire after the bus has returned to idle, which
	 * the v3.8 HW only reaches some clocks AFTER the last byte has
	 * landed in memory. Releasing the calling thread here would beat
	 * STOP-detect to the punch and leave the bus mid-STOP from the
	 * caller's perspective.
	 */
}

/* -------------------------------------------------------------------------
 * I2C controller ISR
 * -------------------------------------------------------------------------*/

static void xec_i2c_nl_isr(const struct device *ctrl)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	mm_reg_t base = cfg->base;
	uint32_t cmpl = sys_read32(base + XEC_I2C_CMPL_OFS);
	uint8_t sr = sys_read8(base + XEC_I2C_SR_OFS);

	if ((cmpl & CMPL_ERR) != 0U) {
		if ((cmpl & CMPL_HNAK) != 0U) {
			data->xfer_err = -ENXIO;
		} else if ((cmpl & CMPL_LAB) != 0U) {
			data->xfer_err = -EAGAIN;
		} else {
			data->xfer_err = -EIO;
		}

		/* Clear the latched CMPL bits and stop DMA. The full PCR
		 * reset that recovers the engine happens on the thread side
		 * (xec_i2c_nl_abort) — doing it here would race with the
		 * thread coming out of its sem wait.
		 */
		sys_write32(CMPL_ERR | CMPL_HDONE, base + XEC_I2C_CMPL_OFS);
		dma_stop(cfg->dma_dev, cfg->dma_chan);
		k_sem_give(&data->pause_sem);
		k_sem_give(&data->done_sem);
		goto out;
	}

	/* HDONE — only used to detect the PAUSE state on a write-then-read
	 * (or pure-read) transfer. Final completion is signalled by
	 * SR.STO below.
	 */
	if ((cmpl & CMPL_HDONE) != 0U) {
		sys_write32(CMPL_HDONE, base + XEC_I2C_CMPL_OFS);

		if (data->state == XEC_I2C_NL_TX) {
			uint32_t hcmd = sys_read32(base + XEC_I2C_HCMD_OFS);

			if ((hcmd & HCMD_RUN) != 0U &&
			    (hcmd & HCMD_PROCEED) == 0U) {
				/* PAUSE: HW finished TX of all (wr-addr |
				 * wr-data | rd-addr) bytes, cleared PROCEED,
				 * and is waiting for SW to switch DMA into
				 * PERIPHERAL_TO_MEMORY for the read phase.
				 */
				k_sem_give(&data->pause_sem);
			}
		}
	}

	/* STOP-detect — the controller's NL-mode STOP-detect interrupt
	 * fires after the bus has truly returned to idle following the
	 * STOP condition. This post-dates HDONE and the RX-DMA callback
	 * (the latter on read direction, where DMA may still be moving
	 * the last byte to memory when HDONE asserts — see README). Use
	 * STOP-detect as the transfer-complete signal in preference to
	 * either of the earlier events.
	 *
	 * Gate on data->state so a stale SR.STO bit observed outside an
	 * active transfer (bus is idle long-term) cannot spuriously
	 * release a future caller — between transfers the thread sets
	 * state = IDLE before the next k_sem_reset(&done_sem).
	 */
	if ((sr & SR_STO) != 0U && data->state != XEC_I2C_NL_IDLE) {
		k_sem_give(&data->done_sem);
	}

out:
	/* The XEC GIRQ status bit is an edge latch — NVIC stays asserted
	 * until SW writes 1 to clear it, regardless of whether the CMPL
	 * RW1C bits behind it are still set. Clearing here avoids ISR
	 * re-entry on transient or spurious edges (notably after a bus
	 * error, where the latch can outlive the CMPL clear).
	 */
	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);
}

/* -------------------------------------------------------------------------
 * Transfer pipeline
 * -------------------------------------------------------------------------*/

static int xec_i2c_nl_setup_tx_dma(const struct device *ctrl, size_t total_write)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	struct dma_block_config block = {
		.source_address = (uint32_t)cfg->bounce_buf,
		.dest_address = cfg->base + XEC_I2C_HTX_OFS,
		.source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
		.block_size = total_write,
	};
	struct dma_config dcfg = {
		.dma_slot = cfg->dma_slot,
		.channel_direction = MEMORY_TO_PERIPHERAL,
		.source_data_size = 1,
		.dest_data_size = 1,
		.source_burst_length = 1,
		.dest_burst_length = 1,
		.block_count = 1,
		.head_block = &block,
		.dma_callback = xec_i2c_nl_tx_dma_cb,
		.user_data = data,
		.complete_callback_en = 1,
		.error_callback_dis = 0,
	};

	return dma_config(cfg->dma_dev, cfg->dma_chan, &dcfg);
}

static int xec_i2c_nl_setup_rx_dma(const struct device *ctrl, uint8_t *buf, size_t len)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	struct dma_block_config block = {
		.source_address = cfg->base + XEC_I2C_HRX_OFS,
		.dest_address = (uint32_t)buf,
		.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
		.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.block_size = len,
	};
	struct dma_config dcfg = {
		.dma_slot = cfg->dma_slot,
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.source_data_size = 1,
		.dest_data_size = 1,
		.source_burst_length = 1,
		.dest_burst_length = 1,
		.block_count = 1,
		.head_block = &block,
		.dma_callback = xec_i2c_nl_rx_dma_cb,
		.user_data = data,
		.complete_callback_en = 1,
		.error_callback_dis = 0,
	};

	return dma_config(cfg->dma_dev, cfg->dma_chan, &dcfg);
}

/* Walk the i2c_msg array, validate the shape, and produce the summary
 * xec_i2c_nl_run consumes. Any flag/shape that can't be issued as a
 * single START-to-STOP NL transaction is rejected here.
 *
 * Accepted shape: [N writes] [M reads], 0 <= N, 0 <= M, N+M >= 1, and
 * total write or total read length is non-zero. The first read (when
 * preceded by a write) must carry I2C_MSG_RESTART. I2C_MSG_STOP is
 * accepted only on the last message; STOP is asserted in HCMD
 * unconditionally regardless of whether it was set on the last msg.
 */
static int xec_i2c_nl_parse(const struct xec_i2c_nl_config *cfg,
			    struct i2c_msg *msgs, uint8_t num_msgs,
			    struct xec_i2c_nl_xfer *xfer)
{
	bool seen_read = false;

	xfer->msgs = msgs;
	xfer->num_msgs = num_msgs;
	xfer->first_read = num_msgs;
	xfer->total_wr_len = 0;
	xfer->total_rd_len = 0;
	xfer->has_read = false;
	xfer->rx_via_bounce = false;

	for (uint8_t i = 0; i < num_msgs; i++) {
		const uint16_t flags = msgs[i].flags;
		const bool is_read = (flags & I2C_MSG_READ) != 0U;
		const bool is_last = (i == (uint8_t)(num_msgs - 1U));

		if ((flags & I2C_MSG_ADDR_10_BITS) != 0U) {
			return -ENOTSUP;
		}

		/* TODO can we support I2C_MSG_STOP in mid-array?
		 *
		 */
		if ((flags & I2C_MSG_STOP) != 0U && !is_last) {
			/* STOP mid-array would split into two transactions. */
			return -ENOTSUP;
		}

		if (is_read) {
			if (!seen_read) {
				xfer->first_read = i;
				seen_read = true;
				if (i > 0U && (flags & I2C_MSG_RESTART) == 0U) {
					return -ENOTSUP;
				}
			}
			uint32_t total = (uint32_t)xfer->total_rd_len + msgs[i].len;

			if (total > XEC_I2C_NL_LEN_MAX) {
				return -EMSGSIZE;
			}
			xfer->total_rd_len = (uint16_t)total;
		} else {
			if (seen_read) {
				/* NL FSM cannot reverse direction. */
				return -ENOTSUP;
			}
			uint32_t total = (uint32_t)xfer->total_wr_len + msgs[i].len;

			if (total > XEC_I2C_NL_LEN_MAX) {
				return -EMSGSIZE;
			}
			xfer->total_wr_len = (uint16_t)total;
		}
	}

	/* Pure-write with total length 0 is allowed — that's an
	 * address-probe / ping transfer. Pure-read with total length 0,
	 * however, has no useful semantics: WCL would be 1 (rd-addr) and
	 * RCL would be 0, so the HW would drive START + rd-addr + STOP and
	 * no bytes would be transferred. Reject it.
	 */
	if (seen_read && xfer->total_rd_len == 0U) {
		return -EINVAL;
	}

	xfer->has_read = seen_read;
	xfer->rx_via_bounce = seen_read &&
			      ((uint8_t)(num_msgs - xfer->first_read) > 1U);

	uint32_t tx_total = 1U + xfer->total_wr_len + (xfer->has_read ? 1U : 0U);

	if (tx_total > cfg->bounce_buf_size) {
		return -ENOSPC;
	}
	if (xfer->rx_via_bounce && xfer->total_rd_len > cfg->bounce_buf_size) {
		return -ENOSPC;
	}
	if (tx_total > XEC_I2C_NL_LEN_MAX) {
		return -EMSGSIZE;
	}

	return 0;
}

/* Build the contiguous TX bounce buffer:
 *
 *     [ wr-addr | wr1 | wr2 | ... | wrN | rd-addr-if-reads ]
 *
 * The wr-addr and rd-addr bytes encode the I2C R/W bit; wr-data is
 * omitted on a pure read; rd-addr is omitted when no read follows.
 */
static void xec_i2c_nl_fill_tx_bounce(const struct xec_i2c_nl_config *cfg,
				      uint16_t addr,
				      const struct xec_i2c_nl_xfer *xfer)
{
	uint8_t *buf = cfg->bounce_buf;
	size_t off = 1U;
	uint8_t end = xfer->has_read ? xfer->first_read : xfer->num_msgs;

	buf[0] = (uint8_t)(((addr & 0x7FU) << 1) | XEC_I2C_NL_RWBIT_WRITE);

	for (uint8_t i = 0; i < end; i++) {
		if (xfer->msgs[i].len > 0U) {
			memcpy(&buf[off], xfer->msgs[i].buf, xfer->msgs[i].len);
			off += xfer->msgs[i].len;
		}
	}

	if (xfer->has_read) {
		buf[1U + xfer->total_wr_len] =
			(uint8_t)(((addr & 0x7FU) << 1) | XEC_I2C_NL_RWBIT_READ);
	}
}

/* Copy bytes from the bounce buffer (used as a single RX DMA target on
 * M > 1 reads) into each user-supplied read buffer in order.
 */
static void xec_i2c_nl_scatter_rx(const struct xec_i2c_nl_config *cfg,
				  const struct xec_i2c_nl_xfer *xfer)
{
	const uint8_t *src = cfg->bounce_buf;
	size_t off = 0U;

	for (uint8_t i = xfer->first_read; i < xfer->num_msgs; i++) {
		if (xfer->msgs[i].len > 0U) {
			memcpy(xfer->msgs[i].buf, &src[off], xfer->msgs[i].len);
			off += xfer->msgs[i].len;
		}
	}
}

static int xec_i2c_nl_run(const struct device *ctrl, uint16_t addr,
			  const struct xec_i2c_nl_xfer *xfer)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	mm_reg_t base = cfg->base;
	uint16_t total_write = 1U + xfer->total_wr_len + (xfer->has_read ? 1U : 0U);
	int rc;

	xec_i2c_nl_fill_tx_bounce(cfg, addr, xfer);

	data->xfer_err = 0;
	k_sem_reset(&data->pause_sem);
	k_sem_reset(&data->done_sem);
	data->state = XEC_I2C_NL_TX;

	rc = xec_i2c_nl_setup_tx_dma(ctrl, total_write);
	if (rc != 0) {
		LOG_ERR("tx dma_config: %d", rc);
		data->state = XEC_I2C_NL_IDLE;
		return rc;
	}

	rc = dma_start(cfg->dma_dev, cfg->dma_chan);
	if (rc != 0) {
		LOG_ERR("tx dma_start: %d", rc);
		data->state = XEC_I2C_NL_IDLE;
		return rc;
	}

	/* Prime ELEN with the high bytes of WCL/RCL, then write HCMD to start
	 * the FSM. STARTN is only meaningful when a read follows the write.
	 */
	sys_write32(XEC_I2C_ELEN_HWR_SET((uint32_t)total_write >> 8) |
			XEC_I2C_ELEN_HRD_SET((uint32_t)xfer->total_rd_len >> 8),
		    base + XEC_I2C_ELEN_OFS);

	uint32_t hcmd = XEC_I2C_HCMD_WCL_SET((uint32_t)total_write & 0xFFU) |
			XEC_I2C_HCMD_RCL_SET((uint32_t)xfer->total_rd_len & 0xFFU) |
			HCMD_RUN | HCMD_PROCEED | HCMD_START0 | HCMD_STOP;

	if (xfer->has_read) {
		hcmd |= HCMD_STARTN;
	}
	sys_write32(hcmd, base + XEC_I2C_HCMD_OFS);

	if (xfer->has_read) {
		rc = k_sem_take(&data->pause_sem, XEC_I2C_NL_TIMEOUT);
		if (rc != 0) {
			LOG_ERR("pause wait: %d", rc);
			xec_i2c_nl_abort(ctrl);
			data->state = XEC_I2C_NL_IDLE;
			return -ETIMEDOUT;
		}
		if (data->xfer_err != 0) {
			xec_i2c_nl_abort(ctrl);
			data->state = XEC_I2C_NL_IDLE;
			return data->xfer_err;
		}

		/* Reprogram the channel for PERIPH->MEM and resume the FSM.
		 * For M == 1 the user buffer is the DMA target directly; for
		 * M > 1 the bounce buffer (no longer needed by TX, which has
		 * completed) is reused as the staging area, and the bytes
		 * are scattered into the user buffers after STOP-detect.
		 */
		dma_stop(cfg->dma_dev, cfg->dma_chan);

		uint8_t *rx_dst = xfer->rx_via_bounce ?
				  cfg->bounce_buf :
				  xfer->msgs[xfer->first_read].buf;

		rc = xec_i2c_nl_setup_rx_dma(ctrl, rx_dst, xfer->total_rd_len);
		if (rc != 0) {
			LOG_ERR("rx dma_config: %d", rc);
			xec_i2c_nl_abort(ctrl);
			data->state = XEC_I2C_NL_IDLE;
			return rc;
		}

		data->state = XEC_I2C_NL_RX;

		rc = dma_start(cfg->dma_dev, cfg->dma_chan);
		if (rc != 0) {
			LOG_ERR("rx dma_start: %d", rc);
			xec_i2c_nl_abort(ctrl);
			data->state = XEC_I2C_NL_IDLE;
			return rc;
		}
		sys_set_bit(base + XEC_I2C_HCMD_OFS, XEC_I2C_HCMD_PROC_POS);
	}

	rc = k_sem_take(&data->done_sem, XEC_I2C_NL_TIMEOUT);
	if (rc != 0) {
		LOG_ERR("done wait: %d", rc);
		xec_i2c_nl_abort(ctrl);
		data->state = XEC_I2C_NL_IDLE;
		return -ETIMEDOUT;
	}

	if (data->xfer_err != 0) {
		/* I2C ISR errors clear CMPL and stop DMA themselves; a TX-DMA
		 * error path bypasses that, so make sure the controller and
		 * DMA channel are both quiesced before returning.
		 */
		xec_i2c_nl_abort(ctrl);
		data->state = XEC_I2C_NL_IDLE;
		return data->xfer_err;
	}

	if (xfer->rx_via_bounce) {
		xec_i2c_nl_scatter_rx(cfg, xfer);
	}

	data->state = XEC_I2C_NL_IDLE;
	return 0;
}

/* -------------------------------------------------------------------------
 * Zephyr i2c_driver_api
 * -------------------------------------------------------------------------*/

static int xec_i2c_nl_vport_transfer(const struct device *port_dev,
				     struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr)
{
	const struct xec_i2c_nl_port_config *pc = port_dev->config;
	const struct device *ctrl = pc->parent;
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	struct xec_i2c_nl_xfer xfer;
	int rc;

	if (num_msgs == 0U || msgs == NULL) {
		return -EINVAL;
	}
	if ((addr & ~0x7FU) != 0U) {
		return -EINVAL; /* 7-bit only */
	}

	rc = xec_i2c_nl_parse(cfg, msgs, num_msgs, &xfer);
	if (rc != 0) {
		return rc;
	}

	k_sem_take(&data->lock, K_FOREVER);

	rc = xec_i2c_nl_apply_port(port_dev);
	if (rc != 0) {
		k_sem_give(&data->lock);
		return rc;
	}

	rc = xec_i2c_nl_run(ctrl, addr, &xfer);

	k_sem_give(&data->lock);
	return rc;
}

static int xec_i2c_nl_xfr(const struct device *port_dev, struct i2c_msg *msgs, uint8_t num_msgs,
			  uint16_t addr)
{
	const struct xec_i2c_nl_port_config *pc = port_dev->config;
	const struct device *ctrl = pc->parent;
	const struct xec_i2c_nl_config *xcfg = ctrl->config;
	struct xec_i2c_nl_data *const xdat = ctrl->data;
	struct xec_i2c_nl_mgrp *g = &xdat->mgrp;
	struct xec_i2c_nl_xfer xfer;
	int rc = 0;
	uint8_t dir = 0;

	if ((num_msgs == 0U) || (msgs == NULL) || ((addr & ~0x7FU) != 0U)) {
		return -EINVAL;
	}

	memset((void *)g, 0, sizeof(struct xec_i2c_nl_mgrp));

	for (uint8_t i = 0; i < num_msgs; i++) {
		struct i2c_msg *m = &msgs[i];
		struct i2c_msg *mnext = NULL;

		if ((i + 1U) < num_msgs) {
			mnext = &msgs[i+1U];
		}

		if ((m->flags & I2C_MSG_READ) != 0) {
			/* read message */
			if (g->mrd == NULL) {
				g->mrd = m;
			} else {

			}
		} else { /* write message */

		}
	}

	return 0;
}

static int xec_i2c_nl_vport_configure(const struct device *port_dev, uint32_t dev_config)
{
	const struct xec_i2c_nl_port_config *pc = port_dev->config;
	const struct device *ctrl = pc->parent;
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	uint32_t freq;
	int rc = 0;

	if ((dev_config & I2C_MODE_CONTROLLER) == 0U) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		freq = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		freq = KHZ(400);
		break;
	case I2C_SPEED_FAST_PLUS:
		freq = MHZ(1);
		break;
	case I2C_SPEED_DT:
		freq = cfg->dflt_freq;
		break;
	default:
		return -ENOTSUP;
	}

	k_sem_take(&data->lock, K_FOREVER);

	if (freq != data->active_freq) {
		/* Reprogramming the controller resets the port-mux too; force
		 * the next transfer to re-apply pinctrl + MUX for its port.
		 */
		rc = xec_i2c_nl_program_ctrl(ctrl, freq, pc->port_id);
		if (rc == 0) {
			data->active_port = XEC_I2C_NL_INVALID_PORT;
		}
	}

	k_sem_give(&data->lock);
	return rc;
}

static int xec_i2c_nl_vport_get_config(const struct device *port_dev, uint32_t *dev_config)
{
	const struct xec_i2c_nl_port_config *pc = port_dev->config;
	struct xec_i2c_nl_data *data = pc->parent->data;
	uint32_t speed;

	if (dev_config == NULL) {
		return -EINVAL;
	}

	if (data->active_freq <= KHZ(100)) {
		speed = I2C_SPEED_STANDARD;
	} else if (data->active_freq <= KHZ(400)) {
		speed = I2C_SPEED_FAST;
	} else {
		speed = I2C_SPEED_FAST_PLUS;
	}

	*dev_config = I2C_MODE_CONTROLLER | I2C_SPEED_SET(speed);
	return 0;
}

static DEVICE_API(i2c, xec_i2c_nl_port_api) = {
	.configure = xec_i2c_nl_vport_configure,
	.get_config = xec_i2c_nl_vport_get_config,
	.transfer = xec_i2c_nl_vport_transfer,
};

/* -------------------------------------------------------------------------
 * Custom mchp_xec_i2c.h API for runtime port query/select
 * -------------------------------------------------------------------------*/

int mchp_xec_i2c_nl_port_get(const struct device *i2c_dev, uint8_t *port)
{
	const struct xec_i2c_nl_port_config *pc;

	if (i2c_dev == NULL || port == NULL) {
		return -EINVAL;
	}

	pc = i2c_dev->config;
	*port = pc->port_id;
	return 0;
}

int mchp_xec_i2c_nl_port_set(const struct device *i2c_dev, uint8_t port)
{
	const struct xec_i2c_nl_port_config *pc;
	const struct xec_i2c_nl_config *cfg;
	struct xec_i2c_nl_data *data;

	if (i2c_dev == NULL || port >= XEC_I2C_CFG_MAX_PORT) {
		return -EINVAL;
	}

	pc = i2c_dev->config;
	cfg = pc->parent->config;
	data = pc->parent->data;

	k_sem_take(&data->lock, K_FOREVER);
	soc_mmcr_mask_set(cfg->base + XEC_I2C_CFG_OFS,
			  XEC_I2C_CFG_PORT_SET(port),
			  XEC_I2C_CFG_PORT_MSK);
	data->active_port = port;
	k_sem_give(&data->lock);
	return 0;
}

/* -------------------------------------------------------------------------
 * Init
 * -------------------------------------------------------------------------*/

static int xec_i2c_nl_ctrl_init(const struct device *ctrl)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	int rc;

	if (!device_is_ready(cfg->dma_dev)) {
		LOG_ERR("dma %s not ready", cfg->dma_dev->name);
		return -ENODEV;
	}

	k_sem_init(&data->lock, 1, 1);
	k_sem_init(&data->pause_sem, 0, 1);
	k_sem_init(&data->done_sem, 0, 1);

	data->state = XEC_I2C_NL_IDLE;
	data->active_port = XEC_I2C_NL_INVALID_PORT;
	data->active_freq = 0;

	rc = xec_i2c_nl_program_ctrl(ctrl, cfg->dflt_freq, 0);
	if (rc != 0) {
		return rc;
	}
	/* No port has had pinctrl applied yet — force the next transfer to
	 * re-apply pinctrl and re-program the MUX for its own port.
	 */
	data->active_port = XEC_I2C_NL_INVALID_PORT;

	if (cfg->irq_connect != NULL) {
		cfg->irq_connect();
	}

	return 0;
}

static int xec_i2c_nl_port_init(const struct device *port_dev)
{
	const struct xec_i2c_nl_port_config *pc = port_dev->config;

	if (!device_is_ready(pc->parent)) {
		return -ENODEV;
	}

	if (pc->is_default) {
		int rc = xec_i2c_nl_apply_port(port_dev);

		if (rc != 0) {
			return rc;
		}
	}

	return 0;
}

/* -------------------------------------------------------------------------
 * Devicetree instantiation
 * -------------------------------------------------------------------------*/

#define DT_DRV_COMPAT microchip_xec_i2c_v3_nl

#define XEC_I2C_NL_GIRQ(inst)     MCHP_XEC_ECIA_GIRQ(DT_INST_PROP(inst, girqs))
#define XEC_I2C_NL_GIRQ_POS(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP(inst, girqs))

/* The controller binding does not carry clock-frequency — it lives on the
 * port nodes. The controller's default frequency is only the value the
 * controller boots up at; vport_configure() reprograms the bus rate per
 * port-device.
 */
#define XEC_I2C_NL_DFLT_FREQ(inst) I2C_BITRATE_STANDARD

#define XEC_I2C_NL_CTRL_INIT(inst)							\
	static uint8_t xec_i2c_nl_bounce_##inst[DT_INST_PROP(inst, bounce_buffer_size)];\
	static void xec_i2c_nl_irq_connect_##inst(void)					\
	{										\
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),		\
			    xec_i2c_nl_isr, DEVICE_DT_INST_GET(inst), 0);		\
		irq_enable(DT_INST_IRQN(inst));						\
	}										\
	static const struct xec_i2c_nl_config xec_i2c_nl_cfg_##inst = {			\
		.base = DT_INST_REG_ADDR(inst),						\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR(inst)),			\
		.irq_connect = xec_i2c_nl_irq_connect_##inst,				\
		.bounce_buf = xec_i2c_nl_bounce_##inst,					\
		.bounce_buf_size = DT_INST_PROP(inst, bounce_buffer_size),		\
		.dflt_freq = XEC_I2C_NL_DFLT_FREQ(inst),				\
		.girq = XEC_I2C_NL_GIRQ(inst),						\
		.girq_pos = XEC_I2C_NL_GIRQ_POS(inst),					\
		.enc_pcr = DT_INST_PROP(inst, pcr_scr),					\
		.dma_chan = DT_INST_DMAS_CELL_BY_IDX(inst, 0, channel),			\
		.dma_slot = DT_INST_DMAS_CELL_BY_IDX(inst, 0, trigsrc),			\
	};										\
	static struct xec_i2c_nl_data xec_i2c_nl_data_##inst;				\
	DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_ctrl_init, NULL,				\
			      &xec_i2c_nl_data_##inst, &xec_i2c_nl_cfg_##inst,		\
			      POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,			\
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_CTRL_INIT)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT microchip_xec_i2c_v3_nl_port

#define XEC_I2C_NL_PORT_INIT(inst)							\
	PINCTRL_DT_INST_DEFINE(inst);							\
	static const struct xec_i2c_nl_port_config xec_i2c_nl_port_cfg_##inst = {	\
		.parent = DEVICE_DT_GET(DT_INST_PHANDLE(inst, controller)),		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),				\
		.bitrate = DT_INST_PROP_OR(inst, clock_frequency, I2C_BITRATE_STANDARD),\
		.port_id = (uint8_t)(DT_INST_PROP(inst, port) & 0x0FU),			\
		.is_default = DT_INST_PROP(inst, default_port),				\
	};										\
	I2C_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_port_init, NULL, NULL,		\
				  &xec_i2c_nl_port_cfg_##inst,				\
				  POST_KERNEL,						\
				  CONFIG_I2C_MCHP_XEC_V3_NL_PORT_INIT_PRIORITY,		\
				  &xec_i2c_nl_port_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_PORT_INIT)
