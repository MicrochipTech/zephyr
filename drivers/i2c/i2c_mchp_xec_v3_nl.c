/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip MEC5 I2C/SMB driver using the controller's Network-Layer
 * transfer engine end-to-end under DMA. The I2C address byte(s) are part
 * of the DMA payload; the Command-register interrupts are not used.
 *
 * Supported:
 *   - I2C Write, Read, and Write-Read (with repeated START) on the host side.
 *   - Target mode (CONFIG_I2C_TARGET + CONFIG_I2C_TARGET_BUFFER_MODE).
 *   - CONFIG_I2C_CALLBACK async completion.
 *   - Controller-TX messages larger than the driver-owned DMA buffer; the
 *     DMA callback (DMA ISR context — must not touch I2C regs) reloads the
 *     buffer with the next chunk while the Network layer keeps pacing I2C.
 *
 * 7-bit addressing only (HW limitation). Port switching + per-port frequency
 * is delegated to the companion microchip,mec5-i2c-mux driver.
 */

#define DT_DRV_COMPAT microchip_xec_v3_i2c_nl

#include <soc.h>
#include <string.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(i2c_mchp_xec_v3_nl, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"
#include "i2c_mchp_mec5_nl.h"
#include "i2c_mchp_xec_regs.h"

/* ---- Tunables --------------------------------------------------------- */

#define MEC5_I2C_XFER_TIMEOUT_MS  500
#define MEC5_I2C_RESET_DELAY_US   10

/* Completion bits that constitute a hard error (R/W1C). */
#define MEC5_I2C_CMPL_ERR_MSK                                                                      \
	(BIT(XEC_I2C_CMPL_BER_STS_POS) | BIT(XEC_I2C_CMPL_LAB_STS_POS) |                           \
	 BIT(XEC_I2C_CMPL_DTS_STS_POS) | BIT(XEC_I2C_CMPL_HCTO_STS_POS) |                          \
	 BIT(XEC_I2C_CMPL_TCTO_STS_POS) | BIT(XEC_I2C_CMPL_TNAKR_STS_POS) |                        \
	 BIT(XEC_I2C_CMPL_HNAKX_POS))

/* Completion bits we clear at the start of each transfer (all R/W1C). */
#define MEC5_I2C_CMPL_CLR_MSK    XEC_I2C_CMPL_RW1C_MSK

/* ---- DMA index constants matching dma-names order --------------------- */

enum {
	MEC5_DMA_IDX_HOST_TX = 0,
	MEC5_DMA_IDX_HOST_RX = 1,
	MEC5_DMA_IDX_TGT_TX  = 2,
	MEC5_DMA_IDX_TGT_RX  = 3,
	MEC5_DMA_IDX_COUNT
};

/* ---- Transfer phase --------------------------------------------------- */

enum mec5_xfer_phase {
	PHASE_IDLE = 0,
	PHASE_H_WRITE,       /* host TX only */
	PHASE_H_READ,        /* host TX (1 addr byte) + host RX */
	PHASE_H_WR_RD_WRITE, /* write phase of WR-RD, PAUSE expected at WCL=0 */
	PHASE_H_WR_RD_READ,  /* read phase of WR-RD, armed from PAUSE ISR */
	PHASE_T_RX,          /* target receiving */
	PHASE_T_TX,          /* target transmitting */
};

/* ---- Timings ---------------------------------------------------------- */

struct mec5_i2c_timing {
	uint32_t freq_hz;
	uint32_t data_tm;
	uint32_t idle_sc;
	uint32_t timeout_sc;
	uint32_t bus_clock;
	uint8_t  rpt_sta_htm;
};

static const struct mec5_i2c_timing mec5_i2c_timings[] = {
	{KHZ(100), XEC_I2C_SMB_DATA_TM_100K, XEC_I2C_SMB_IDLE_SC_100K,
	 XEC_I2C_SMB_TMO_SC_100K, XEC_I2C_SMB_BUS_CLK_100K, XEC_I2C_SMB_RSHT_100K},
	{KHZ(400), XEC_I2C_SMB_DATA_TM_400K, XEC_I2C_SMB_IDLE_SC_400K,
	 XEC_I2C_SMB_TMO_SC_400K, XEC_I2C_SMB_BUS_CLK_400K, XEC_I2C_SMB_RSHT_400K},
	{MHZ(1),   XEC_I2C_SMB_DATA_TM_1M,   XEC_I2C_SMB_IDLE_SC_1M,
	 XEC_I2C_SMB_TMO_SC_1M,   XEC_I2C_SMB_BUS_CLK_1M,   XEC_I2C_SMB_RSHT_1M},
};

/* ---- Device config + data -------------------------------------------- */

struct mec5_dma_ep {
	const struct device *ctlr;
	uint32_t channel;
	uint32_t slot;
};

struct mec5_i2c_cfg {
	mm_reg_t base;
	uint32_t clock_freq;
	uint16_t buffer_size;
	uint8_t  port_sel;
	uint8_t  girq;
	uint8_t  girq_pos;
	uint16_t enc_pcr;
	const struct pinctrl_dev_config *pcfg;
	struct mec5_dma_ep dma[MEC5_DMA_IDX_COUNT];
	uint8_t *dma_buf;
	void (*irq_config_func)(void);
};

struct mec5_i2c_data {
	struct k_mutex bus_lock;
	struct k_sem xfer_sem;
	uint32_t dev_config;
	uint32_t cur_freq_hz;
	uint8_t  cur_port;

	/* Active transfer bookkeeping */
	enum mec5_xfer_phase phase;
	uint32_t cmpl_snap;
	int      xfer_err;

	/* Controller-TX chunking state (thread sets up, DMA cb advances) */
	const uint8_t *tx_src;
	uint32_t tx_remaining;
	uint16_t tx_chunk;

#ifdef CONFIG_I2C_CALLBACK
	i2c_callback_t async_cb;
	void *async_userdata;
	struct i2c_msg *async_msgs;
	uint8_t async_num_msgs;
	uint16_t async_addr;
#endif

#ifdef CONFIG_I2C_TARGET
	struct i2c_target_config *target_cfg;
	bool target_attached;
#endif
};

/* ---- Small helpers --------------------------------------------------- */

static inline uint32_t mec5_rd32(const struct mec5_i2c_cfg *c, uint32_t off)
{
	return sys_read32(c->base + off);
}

static inline void mec5_wr32(const struct mec5_i2c_cfg *c, uint32_t off, uint32_t v)
{
	sys_write32(v, c->base + off);
}

static inline void mec5_wr8(const struct mec5_i2c_cfg *c, uint32_t off, uint8_t v)
{
	sys_write8(v, c->base + off);
}

static const struct mec5_i2c_timing *mec5_lookup_timing(uint32_t freq_hz)
{
	for (size_t i = 0; i < ARRAY_SIZE(mec5_i2c_timings); i++) {
		if (mec5_i2c_timings[i].freq_hz == freq_hz) {
			return &mec5_i2c_timings[i];
		}
	}
	return NULL;
}

static int mec5_apply_timing(const struct mec5_i2c_cfg *c, uint32_t freq_hz)
{
	const struct mec5_i2c_timing *t = mec5_lookup_timing(freq_hz);

	if (t == NULL) {
		return -EINVAL;
	}

	mec5_wr32(c, XEC_I2C_DT_OFS,        t->data_tm);
	mec5_wr32(c, XEC_I2C_ISC_OFS,       t->idle_sc);
	mec5_wr32(c, XEC_I2C_TMOUT_SC_OFS,  t->timeout_sc);
	mec5_wr32(c, XEC_I2C_BCLK_OFS,      t->bus_clock);
	mec5_wr8(c,  XEC_I2C_RSHT_OFS,      t->rpt_sta_htm);

	return 0;
}

static uint32_t mec5_freq_from_cfg(uint32_t dev_config)
{
	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:  return KHZ(100);
	case I2C_SPEED_FAST:      return KHZ(400);
	case I2C_SPEED_FAST_PLUS: return MHZ(1);
	default:                  return 0;
	}
}

/* PCR reset + port select + filter enable + completion-interrupt enables.
 * After this the controller is READY but not yet enabled.
 */
static void mec5_core_reset(const struct device *dev)
{
	const struct mec5_i2c_cfg *c = dev->config;
	uint32_t cfg;

	soc_xec_pcr_reset_en(c->enc_pcr);
	k_busy_wait(MEC5_I2C_RESET_DELAY_US);

	cfg = XEC_I2C_CFG_PORT_SET(c->port_sel) | BIT(XEC_I2C_CFG_FEN_POS) |
	      BIT(XEC_I2C_CFG_GC_DIS_POS);
	mec5_wr32(c, XEC_I2C_CFG_OFS, cfg);

	/* Clear COMPLETION R/W1C status. */
	mec5_wr32(c, XEC_I2C_CMPL_OFS, MEC5_I2C_CMPL_CLR_MSK);

	k_busy_wait(MEC5_I2C_RESET_DELAY_US);
}

static void mec5_enable_completion_ien(const struct mec5_i2c_cfg *c, bool target_mode)
{
	uint32_t cfg = mec5_rd32(c, XEC_I2C_CFG_OFS);

	cfg |= BIT(XEC_I2C_CFG_IDLE_IEN_POS) | BIT(XEC_I2C_CFG_HD_IEN_POS);
	if (target_mode) {
		cfg |= BIT(XEC_I2C_CFG_TD_IEN_POS) | BIT(XEC_I2C_CFG_AAT_IEN_POS);
	}
	/* Command-register interrupts (ENI) stay disabled: NL uses COMPLETION only. */
	mec5_wr32(c, XEC_I2C_CFG_OFS, cfg);
}

static void mec5_enable_controller(const struct mec5_i2c_cfg *c)
{
	uint32_t cfg = mec5_rd32(c, XEC_I2C_CFG_OFS);

	cfg |= BIT(XEC_I2C_CFG_ENAB_POS);
	mec5_wr32(c, XEC_I2C_CFG_OFS, cfg);
}

static void mec5_disable_controller(const struct mec5_i2c_cfg *c)
{
	uint32_t cfg = mec5_rd32(c, XEC_I2C_CFG_OFS);

	cfg &= ~BIT(XEC_I2C_CFG_ENAB_POS);
	mec5_wr32(c, XEC_I2C_CFG_OFS, cfg);
}

/* ---- DMA plumbing ---------------------------------------------------- */

/* Device endpoint addresses for the four directions. */
static uint32_t mec5_dma_endpoint(const struct mec5_i2c_cfg *c, int idx)
{
	switch (idx) {
	case MEC5_DMA_IDX_HOST_TX: return c->base + XEC_I2C_HTX_OFS;
	case MEC5_DMA_IDX_HOST_RX: return c->base + XEC_I2C_HRX_OFS;
	case MEC5_DMA_IDX_TGT_TX:  return c->base + XEC_I2C_TTX_OFS;
	case MEC5_DMA_IDX_TGT_RX:  return c->base + XEC_I2C_TRX_OFS;
	default:                   return 0;
	}
}

static bool mec5_dma_is_tx(int idx)
{
	return idx == MEC5_DMA_IDX_HOST_TX || idx == MEC5_DMA_IDX_TGT_TX;
}

static void mec5_dma_cb(const struct device *dma_dev, void *user_data,
			uint32_t channel, int status);

/* Configure + start a single DMA burst of `len` bytes against endpoint `idx`.
 * `mem_addr` is always the memory-side pointer (source when TX, dest when RX).
 * On error the DMA channel is left stopped and a negative errno is returned.
 */
static int mec5_dma_arm(const struct device *dev, int idx, uint8_t *mem_addr, uint32_t len)
{
	const struct mec5_i2c_cfg *c = dev->config;
	const struct mec5_dma_ep *ep = &c->dma[idx];
	struct dma_config dcfg = {0};
	struct dma_block_config blk = {0};
	uint32_t ep_addr = mec5_dma_endpoint(c, idx);
	bool tx = mec5_dma_is_tx(idx);
	int ret;

	if (!device_is_ready(ep->ctlr)) {
		return -ENODEV;
	}

	dcfg.dma_slot           = ep->slot;
	dcfg.channel_direction  = tx ? MEMORY_TO_PERIPHERAL : PERIPHERAL_TO_MEMORY;
	dcfg.source_data_size   = 1;
	dcfg.dest_data_size     = 1;
	dcfg.source_burst_length = 1;
	dcfg.dest_burst_length  = 1;
	dcfg.block_count        = 1;
	dcfg.head_block         = &blk;
	dcfg.user_data          = (void *)dev;
	dcfg.dma_callback       = mec5_dma_cb;

	blk.block_size = len;
	if (tx) {
		blk.source_address = (uintptr_t)mem_addr;
		blk.dest_address   = ep_addr;
		blk.source_addr_adj = 0; /* increment */
		blk.dest_addr_adj   = 2; /* no change */
	} else {
		blk.source_address = ep_addr;
		blk.dest_address   = (uintptr_t)mem_addr;
		blk.source_addr_adj = 2; /* no change */
		blk.dest_addr_adj   = 0; /* increment */
	}

	ret = dma_config(ep->ctlr, ep->channel, &dcfg);
	if (ret != 0) {
		return ret;
	}

	return dma_start(ep->ctlr, ep->channel);
}

static void mec5_dma_stop_all(const struct device *dev)
{
	const struct mec5_i2c_cfg *c = dev->config;

	for (int i = 0; i < MEC5_DMA_IDX_COUNT; i++) {
		if (c->dma[i].ctlr != NULL && device_is_ready(c->dma[i].ctlr)) {
			dma_stop(c->dma[i].ctlr, c->dma[i].channel);
		}
	}
}

/* ---- Controller-TX chunking ----------------------------------------- */

/* Called from the DMA ISR for the host-TX channel. When `tx_remaining > 0`
 * we copy the next chunk of the caller buffer into dma_buf and reload+start
 * the DMA channel. The Network layer's decrementing HCMD.WCL is what stops
 * the I2C side; we just keep feeding bytes until it hits zero.
 */
static void mec5_tx_cb_reload(const struct device *dev)
{
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;
	const struct mec5_dma_ep *ep = &c->dma[MEC5_DMA_IDX_HOST_TX];
	uint32_t ep_addr = mec5_dma_endpoint(c, MEC5_DMA_IDX_HOST_TX);
	uint32_t chunk = MIN(c->buffer_size, d->tx_remaining);

	memcpy(c->dma_buf, d->tx_src, chunk);
	d->tx_src       += chunk;
	d->tx_remaining -= chunk;

	(void)dma_reload(ep->ctlr, ep->channel, (uintptr_t)c->dma_buf, ep_addr, chunk);
	(void)dma_start(ep->ctlr, ep->channel);
}

/* ---- Completion ISR -------------------------------------------------- */

#ifdef CONFIG_I2C_TARGET
static void mec5_target_handle_rx_done(const struct device *dev, uint32_t cmpl);
static void mec5_target_rearm_rx(const struct device *dev);
#endif

/* Arm host-RX DMA for the read phase of a WR-RD, then set PROCEED. */
static void mec5_isr_handle_pause(const struct device *dev)
{
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;
	uint32_t hcmd;
	uint32_t rcl;

	hcmd = mec5_rd32(c, XEC_I2C_HCMD_OFS);
	rcl  = XEC_I2C_HCMD_RCL_GET(hcmd) |
	       (XEC_I2C_ELEN_HRD_GET(mec5_rd32(c, XEC_I2C_ELEN_OFS)) << 8);

	/* Land the read phase into dma_buf starting at offset 0. */
	if (mec5_dma_arm(dev, MEC5_DMA_IDX_HOST_RX, c->dma_buf, rcl) != 0) {
		d->xfer_err = -EIO;
		k_sem_give(&d->xfer_sem);
		return;
	}

	d->phase = PHASE_H_WR_RD_READ;
	/* Order: DMA armed, then PROCEED (per README). */
	hcmd = mec5_rd32(c, XEC_I2C_HCMD_OFS);
	hcmd |= BIT(XEC_I2C_HCMD_PROC_POS);
	mec5_wr32(c, XEC_I2C_HCMD_OFS, hcmd);
}

static void mec5_nl_isr(const struct device *dev)
{
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;
	uint32_t cmpl = mec5_rd32(c, XEC_I2C_CMPL_OFS);
	bool post = false;

	d->cmpl_snap = cmpl;
	/* Write-1-clear the sticky bits we observed. */
	mec5_wr32(c, XEC_I2C_CMPL_OFS, cmpl & XEC_I2C_CMPL_RW1C_MSK);

	if ((cmpl & MEC5_I2C_CMPL_ERR_MSK) != 0) {
		d->xfer_err = -EIO;
		post = true;
		goto out_post;
	}

	/* PAUSE check: HDONE asserted with HCMD.RUN=1 but PROC=0 means the HW
	 * decremented WCL to 0 and is waiting for software to program the read
	 * phase.
	 */
	if ((cmpl & BIT(XEC_I2C_CMPL_HDONE_POS)) != 0 &&
	    d->phase == PHASE_H_WR_RD_WRITE) {
		uint32_t hcmd = mec5_rd32(c, XEC_I2C_HCMD_OFS);

		if ((hcmd & BIT(XEC_I2C_HCMD_RUN_POS)) != 0 &&
		    (hcmd & BIT(XEC_I2C_HCMD_PROC_POS)) == 0) {
			mec5_isr_handle_pause(dev);
			/* Not done yet; wait for next completion event. */
			goto out;
		}
	}

#ifdef CONFIG_I2C_TARGET
	if ((cmpl & BIT(XEC_I2C_CMPL_TDONE_POS)) != 0 && d->target_attached) {
		mec5_target_handle_rx_done(dev, cmpl);
		/* Target events do not post xfer_sem; controller thread is idle. */
		goto out;
	}
#endif

	/* Full host-side completion: IDLE + HDONE with RUN=0. */
	if ((cmpl & (BIT(XEC_I2C_CMPL_IDLE_POS) | BIT(XEC_I2C_CMPL_HDONE_POS))) != 0) {
		uint32_t hcmd = mec5_rd32(c, XEC_I2C_HCMD_OFS);

		if ((hcmd & BIT(XEC_I2C_HCMD_RUN_POS)) == 0 &&
		    d->phase != PHASE_IDLE &&
		    d->phase != PHASE_T_RX && d->phase != PHASE_T_TX) {
			post = true;
		}
	}

out_post:
	if (post) {
		k_sem_give(&d->xfer_sem);
	}
out:
	return;
}

/* ---- DMA callback (DMA ISR context, NO I2C register access!) -------- */

static void mec5_dma_cb(const struct device *dma_dev, void *user_data,
			uint32_t channel, int status)
{
	const struct device *dev = user_data;
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;

	ARG_UNUSED(dma_dev);

	if (status < 0) {
		d->xfer_err = status;
		k_sem_give(&d->xfer_sem);
		return;
	}

	/* Only the host-TX channel in a chunking transfer does work here:
	 * the I2C completion ISR is responsible for finishing everything else.
	 */
	if (channel == c->dma[MEC5_DMA_IDX_HOST_TX].channel && d->tx_remaining > 0) {
		mec5_tx_cb_reload(dev);
	}
}

/* ---- Controller-side message execution ------------------------------ */

/* Common prep: clear snapshot, stop prior DMA, clear completion. */
static void mec5_begin_transfer(struct mec5_i2c_data *d, const struct mec5_i2c_cfg *c)
{
	d->xfer_err = 0;
	d->cmpl_snap = 0;
	d->tx_src = NULL;
	d->tx_remaining = 0;
	mec5_wr32(c, XEC_I2C_CMPL_OFS, MEC5_I2C_CMPL_CLR_MSK);
	k_sem_reset(&d->xfer_sem);
}

static int mec5_wait_done(struct mec5_i2c_data *d)
{
	if (k_sem_take(&d->xfer_sem, K_MSEC(MEC5_I2C_XFER_TIMEOUT_MS)) != 0) {
		return -ETIMEDOUT;
	}
	return d->xfer_err;
}

/* Program HCMD + ELEN in one pass. WCL/RCL are the FULL lengths (low 8 bits in
 * HCMD, upper bits in ELEN), which the HW decrements byte-by-byte.
 */
static void mec5_program_hcmd(const struct mec5_i2c_cfg *c, uint32_t wcl, uint32_t rcl,
			      bool start, bool startn, bool stop)
{
	uint32_t elen = mec5_rd32(c, XEC_I2C_ELEN_OFS) &
			~(XEC_I2C_ELEN_HWR_MSK | XEC_I2C_ELEN_HRD_MSK);
	uint32_t hcmd = 0;

	elen |= XEC_I2C_ELEN_HWR_SET((wcl >> 8) & 0xFFU);
	elen |= XEC_I2C_ELEN_HRD_SET((rcl >> 8) & 0xFFU);
	mec5_wr32(c, XEC_I2C_ELEN_OFS, elen);

	hcmd = XEC_I2C_HCMD_WCL_SET(wcl & 0xFFU) |
	       XEC_I2C_HCMD_RCL_SET(rcl & 0xFFU);
	if (start) {
		hcmd |= BIT(XEC_I2C_HCMD_START0_POS);
	}
	if (startn) {
		hcmd |= BIT(XEC_I2C_HCMD_STARTN_POS);
	}
	if (stop) {
		hcmd |= BIT(XEC_I2C_HCMD_STOP_POS);
	}
	hcmd |= BIT(XEC_I2C_HCMD_RUN_POS);
	mec5_wr32(c, XEC_I2C_HCMD_OFS, hcmd);
}

/* addr7 is the 7-bit I2C address; rw=0 for write, rw=1 for read. */
static inline uint8_t mec5_addr_byte(uint16_t addr7, uint8_t rw)
{
	return (uint8_t)(((addr7 & 0x7FU) << 1) | (rw & 0x1U));
}

/* Execute a single Write message. `len` may exceed buffer-size; if it does,
 * we prime the buffer with the address + as much data as fits, arm host-TX DMA
 * for that chunk, program HCMD.WCL = full (addr + len), and the DMA callback
 * will stream the rest from dev->data->tx_src.
 */
static int mec5_do_write(const struct device *dev, uint16_t addr7,
			 const uint8_t *buf, uint32_t len, bool stop)
{
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;
	uint32_t total = len + 1U; /* address + payload */
	uint32_t first_chunk = MIN((uint32_t)c->buffer_size, total);
	uint32_t data_in_first = first_chunk - 1U; /* address byte takes slot 0 */

	if (total > XEC_I2C_NL_MAX_LEN) {
		return -EMSGSIZE;
	}

	mec5_begin_transfer(d, c);

	c->dma_buf[0] = mec5_addr_byte(addr7, 0);
	if (data_in_first > 0) {
		memcpy(&c->dma_buf[1], buf, data_in_first);
	}
	d->tx_src       = buf + data_in_first;
	d->tx_remaining = len - data_in_first;
	d->tx_chunk     = first_chunk;
	d->phase        = PHASE_H_WRITE;

	int ret = mec5_dma_arm(dev, MEC5_DMA_IDX_HOST_TX, c->dma_buf, first_chunk);

	if (ret != 0) {
		return ret;
	}

	mec5_program_hcmd(c, total, 0, true, false, stop);

	return mec5_wait_done(d);
}

/* Execute a single Read message. Two DMA channels run concurrently:
 * host-TX pushes exactly 1 address byte into HTX, and host-RX lands `len` bytes
 * from HRX into dma_buf[0..len-1] (then we copy out to the caller buffer).
 */
static int mec5_do_read(const struct device *dev, uint16_t addr7,
			uint8_t *buf, uint32_t len, bool stop)
{
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;

	if (len == 0 || len > (c->buffer_size) || len > XEC_I2C_NL_MAX_LEN) {
		return len == 0 ? 0 : -EMSGSIZE;
	}

	mec5_begin_transfer(d, c);

	/* Address byte goes out via host-TX DMA, reusing slot 0 of dma_buf. */
	c->dma_buf[0] = mec5_addr_byte(addr7, 1);
	d->phase = PHASE_H_READ;

	int ret = mec5_dma_arm(dev, MEC5_DMA_IDX_HOST_TX, c->dma_buf, 1U);

	if (ret != 0) {
		return ret;
	}
	/* Land received data starting at dma_buf[0]; we copy out after completion. */
	ret = mec5_dma_arm(dev, MEC5_DMA_IDX_HOST_RX, c->dma_buf, len);
	if (ret != 0) {
		return ret;
	}

	mec5_program_hcmd(c, 1U, len, true, false, stop);

	ret = mec5_wait_done(d);
	if (ret == 0) {
		memcpy(buf, c->dma_buf, len);
	}
	return ret;
}

/* Write-Read with repeated START. The write phase runs first; at WCL=0 the HW
 * PAUSEs; mec5_isr_handle_pause() arms the read DMA and sets PROCEED.
 */
static int mec5_do_write_read(const struct device *dev, uint16_t addr7,
			      const uint8_t *wr, uint32_t wlen,
			      uint8_t *rd, uint32_t rlen)
{
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;
	uint32_t total_wr = wlen + 2U; /* addr, data..., repeated-START addr */

	if (wlen + 2U > c->buffer_size) {
		/* WR-RD with chunked write phase is not supported: the repeated-
		 * START addr byte lives at [wlen+1] of the write buffer and must
		 * be DMA'd with the payload. Callers should issue separate write
		 * then read instead.
		 */
		return -EMSGSIZE;
	}
	if (rlen > c->buffer_size || rlen > XEC_I2C_NL_MAX_LEN) {
		return -EMSGSIZE;
	}

	mec5_begin_transfer(d, c);

	c->dma_buf[0] = mec5_addr_byte(addr7, 0);
	if (wlen > 0) {
		memcpy(&c->dma_buf[1], wr, wlen);
	}
	c->dma_buf[wlen + 1U] = mec5_addr_byte(addr7, 1);

	d->phase = PHASE_H_WR_RD_WRITE;

	int ret = mec5_dma_arm(dev, MEC5_DMA_IDX_HOST_TX, c->dma_buf, total_wr);

	if (ret != 0) {
		return ret;
	}

	mec5_program_hcmd(c, total_wr, rlen, true, true, true);

	ret = mec5_wait_done(d);
	if (ret == 0) {
		memcpy(rd, c->dma_buf, rlen);
	}
	return ret;
}

/* ---- i2c_driver_api entry points ------------------------------------ */

static int mec5_transfer_one(const struct device *dev, struct i2c_msg *msgs,
			     uint8_t num_msgs, uint16_t addr)
{
	/* Fold adjacent write-then-read into a single Network-layer WR-RD when
	 * it fits the addressing model (same target, read after write, STOP on
	 * the read). Otherwise issue each msg independently.
	 */
	if (num_msgs == 2 &&
	    (msgs[0].flags & I2C_MSG_READ) == 0 &&
	    (msgs[1].flags & I2C_MSG_READ) != 0 &&
	    (msgs[1].flags & I2C_MSG_RESTART) != 0 &&
	    (msgs[1].flags & I2C_MSG_STOP) != 0) {
		return mec5_do_write_read(dev, addr, msgs[0].buf, msgs[0].len,
					  msgs[1].buf, msgs[1].len);
	}

	for (uint8_t i = 0; i < num_msgs; i++) {
		bool stop = (msgs[i].flags & I2C_MSG_STOP) != 0;
		int rc;

		if ((msgs[i].flags & I2C_MSG_READ) != 0) {
			rc = mec5_do_read(dev, addr, msgs[i].buf, msgs[i].len, stop);
		} else {
			rc = mec5_do_write(dev, addr, msgs[i].buf, msgs[i].len, stop);
		}
		if (rc != 0) {
			return rc;
		}
	}
	return 0;
}

static int mec5_transfer(const struct device *dev, struct i2c_msg *msgs,
			 uint8_t num_msgs, uint16_t addr)
{
	struct mec5_i2c_data *d = dev->data;
	int ret;

	if (num_msgs == 0) {
		return 0;
	}

	k_mutex_lock(&d->bus_lock, K_FOREVER);
	ret = mec5_transfer_one(dev, msgs, num_msgs, addr);
	k_mutex_unlock(&d->bus_lock);
	return ret;
}

#ifdef CONFIG_I2C_CALLBACK
static int mec5_transfer_cb(const struct device *dev, struct i2c_msg *msgs,
			    uint8_t num_msgs, uint16_t addr,
			    i2c_callback_t cb, void *userdata)
{
	/* Simple blocking implementation: run the transfer and dispatch the
	 * completion callback synchronously. This keeps the driver portable
	 * while still exercising the CONFIG_I2C_CALLBACK path from the user's
	 * point of view.
	 */
	int ret = mec5_transfer(dev, msgs, num_msgs, addr);

	if (cb != NULL) {
		cb(dev, ret, userdata);
	}
	return ret;
}
#endif

static int mec5_configure(const struct device *dev, uint32_t dev_config)
{
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;
	uint32_t freq;
	int ret;

	if ((dev_config & I2C_ADDR_10_BITS) != 0) {
		return -ENOTSUP;
	}
	if ((dev_config & I2C_MODE_CONTROLLER) == 0) {
		return -ENOTSUP;
	}

	freq = mec5_freq_from_cfg(dev_config);
	if (freq == 0) {
		return -EINVAL;
	}

	k_mutex_lock(&d->bus_lock, K_FOREVER);

	mec5_disable_controller(c);
	mec5_core_reset(dev);
	ret = mec5_apply_timing(c, freq);
	if (ret == 0) {
		mec5_enable_completion_ien(c, IS_ENABLED(CONFIG_I2C_TARGET));
		mec5_enable_controller(c);
		d->dev_config  = dev_config;
		d->cur_freq_hz = freq;
		d->cur_port    = c->port_sel;
	}

	k_mutex_unlock(&d->bus_lock);
	return ret;
}

static int mec5_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct mec5_i2c_data *d = dev->data;

	if (dev_config == NULL) {
		return -EINVAL;
	}
	*dev_config = d->dev_config;
	return 0;
}

/* ---- Target mode ---------------------------------------------------- */

#ifdef CONFIG_I2C_TARGET

static void mec5_target_rearm_rx(const struct device *dev)
{
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;
	uint32_t tcmd;

	/* DMA sized for the full buffer: it absorbs own-address byte + data +
	 * the HW-generated trailing zero after STOP. Read count is set big so
	 * the HW never stops early; the actual length is recovered from TCMD
	 * after the transaction.
	 */
	if (mec5_dma_arm(dev, MEC5_DMA_IDX_TGT_RX, c->dma_buf, c->buffer_size) != 0) {
		LOG_ERR("target RX DMA arm failed");
		return;
	}

	tcmd = XEC_I2C_TCMD_WCL_SET(c->buffer_size & 0xFFU) |
	       XEC_I2C_TCMD_RCL_SET(c->buffer_size & 0xFFU) |
	       BIT(XEC_I2C_TCMD_RUN_POS);
	mec5_wr32(c, XEC_I2C_TCMD_OFS, tcmd);
	d->phase = PHASE_T_RX;
}

static void mec5_target_handle_rx_done(const struct device *dev, uint32_t cmpl)
{
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;
	const struct i2c_target_callbacks *cbs;
	uint32_t tcmd;
	uint32_t remaining;
	uint32_t received;

	if (d->target_cfg == NULL) {
		return;
	}
	cbs = d->target_cfg->callbacks;

	ARG_UNUSED(cmpl);

	tcmd = mec5_rd32(c, XEC_I2C_TCMD_OFS);
	remaining = XEC_I2C_TCMD_WCL_GET(tcmd) |
		    (XEC_I2C_ELEN_TWR_GET(mec5_rd32(c, XEC_I2C_ELEN_OFS)) << 8);
	received = (uint32_t)c->buffer_size - remaining;

	/* Strip byte[0] (own-address) and the HW trailing zero at the end. */
	if (received >= 2U) {
		uint32_t payload_len = received - 2U;

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
		if (cbs != NULL && cbs->buf_write_received != NULL && payload_len > 0) {
			cbs->buf_write_received(d->target_cfg, &c->dma_buf[1], payload_len);
		}
#else
		if (cbs != NULL && cbs->write_received != NULL) {
			for (uint32_t i = 0; i < payload_len; i++) {
				cbs->write_received(d->target_cfg, c->dma_buf[1U + i]);
			}
		}
#endif
	}

	if (cbs != NULL && cbs->stop != NULL) {
		cbs->stop(d->target_cfg);
	}

	mec5_target_rearm_rx(dev);
}

static int mec5_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;

	if (cfg == NULL || (cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&d->bus_lock, K_FOREVER);

	if (d->target_attached) {
		k_mutex_unlock(&d->bus_lock);
		return -EBUSY;
	}

	mec5_wr32(c, XEC_I2C_OA_OFS, XEC_I2C_OA_1_SET(cfg->address));
	d->target_cfg = cfg;
	d->target_attached = true;

	mec5_enable_completion_ien(c, true);
	mec5_target_rearm_rx(dev);

	k_mutex_unlock(&d->bus_lock);
	return 0;
}

static int mec5_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;

	k_mutex_lock(&d->bus_lock, K_FOREVER);

	if (!d->target_attached || d->target_cfg != cfg) {
		k_mutex_unlock(&d->bus_lock);
		return -EINVAL;
	}

	mec5_wr32(c, XEC_I2C_TCMD_OFS, 0);
	mec5_dma_stop_all(dev);

	mec5_wr32(c, XEC_I2C_OA_OFS, 0);
	d->target_cfg = NULL;
	d->target_attached = false;
	d->phase = PHASE_IDLE;

	k_mutex_unlock(&d->bus_lock);
	return 0;
}
#endif /* CONFIG_I2C_TARGET */

/* ---- Helpers exported to the MUX driver ---------------------------- */

int mec5_i2c_nl_stop_and_reset(const struct device *dev)
{
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;

	k_mutex_lock(&d->bus_lock, K_FOREVER);
	mec5_disable_controller(c);
	mec5_dma_stop_all(dev);
	mec5_core_reset(dev);
	d->phase = PHASE_IDLE;
	k_mutex_unlock(&d->bus_lock);
	return 0;
}

int mec5_i2c_nl_apply_port_freq(const struct device *dev, uint8_t port, uint32_t freq_hz)
{
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;
	uint32_t cfg;
	int ret;

	if (port >= XEC_I2C_CFG_MAX_PORT) {
		return -EINVAL;
	}

	k_mutex_lock(&d->bus_lock, K_FOREVER);

	mec5_disable_controller(c);

	cfg = mec5_rd32(c, XEC_I2C_CFG_OFS);
	cfg &= ~XEC_I2C_CFG_PORT_MSK;
	cfg |= XEC_I2C_CFG_PORT_SET(port);
	mec5_wr32(c, XEC_I2C_CFG_OFS, cfg);

	ret = mec5_apply_timing(c, freq_hz);
	if (ret == 0) {
		mec5_enable_completion_ien(c, IS_ENABLED(CONFIG_I2C_TARGET));
		mec5_enable_controller(c);
		d->cur_port = port;
		d->cur_freq_hz = freq_hz;
	}

	k_mutex_unlock(&d->bus_lock);
	return ret;
}

int mec5_i2c_nl_apply_pinctrl(const struct device *dev,
			      const struct pinctrl_dev_config *pcfg)
{
	if (pcfg == NULL) {
		return -EINVAL;
	}
	return pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT);
}

/* ---- Driver API + init --------------------------------------------- */

static DEVICE_API(i2c, mec5_i2c_api) = {
	.configure   = mec5_configure,
	.get_config  = mec5_get_config,
	.transfer    = mec5_transfer,
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = mec5_transfer_cb,
#endif
#ifdef CONFIG_I2C_TARGET
	.target_register   = mec5_target_register,
	.target_unregister = mec5_target_unregister,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

static int mec5_i2c_init(const struct device *dev)
{
	const struct mec5_i2c_cfg *c = dev->config;
	struct mec5_i2c_data *d = dev->data;
	uint32_t bitrate_cfg;
	int ret;

	k_mutex_init(&d->bus_lock);
	k_sem_init(&d->xfer_sem, 0, 1);
	d->phase = PHASE_IDLE;

	/* Pins BEFORE controller enable — else Bus Error. */
	ret = pinctrl_apply_state(c->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("pinctrl_apply_state: %d", ret);
		return ret;
	}

	soc_xec_pcr_sleep_en_clear(c->enc_pcr);

	bitrate_cfg = i2c_map_dt_bitrate(c->clock_freq);
	if (bitrate_cfg == 0) {
		return -EINVAL;
	}

	ret = mec5_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	if (ret != 0) {
		return ret;
	}

	if (c->irq_config_func != NULL) {
		c->irq_config_func();
	}

	return 0;
}

/* ---- DT instantiation ---------------------------------------------- */

#define MEC5_DMA_EP(n, name, idx)                                                                  \
	[idx] = {                                                                                  \
		.ctlr    = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, name)),                      \
		.channel = DT_INST_DMAS_CELL_BY_NAME(n, name, channel),                            \
		.slot    = DT_INST_DMAS_CELL_BY_NAME(n, name, trigsrc),                            \
	}

#define MEC5_I2C_GIRQ(inst, idx)                                                                   \
	(uint8_t)MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, idx))
#define MEC5_I2C_GIRQ_POS(inst, idx)                                                               \
	(uint8_t)MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, idx))

#define MEC5_I2C_DEVICE(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void mec5_i2c_irq_cfg_##n(void);                                                    \
	static uint8_t mec5_i2c_dma_buf_##n[DT_INST_PROP_OR(n, buffer_size, 64)]                   \
		__aligned(4);                                                                      \
	static struct mec5_i2c_data mec5_i2c_data_##n;                                             \
	static const struct mec5_i2c_cfg mec5_i2c_cfg_##n = {                                      \
		.base        = (mm_reg_t)DT_INST_REG_ADDR(n),                                      \
		.clock_freq  = DT_INST_PROP(n, clock_frequency),                                   \
		.buffer_size = DT_INST_PROP_OR(n, buffer_size, 64),                                \
		.port_sel    = DT_INST_PROP(n, port_sel),                                          \
		.girq        = MEC5_I2C_GIRQ(n, 0),                                                \
		.girq_pos    = MEC5_I2C_GIRQ_POS(n, 0),                                            \
		.enc_pcr     = DT_INST_PROP(n, pcr_scr),                                           \
		.pcfg        = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                  \
		.dma = {                                                                           \
			MEC5_DMA_EP(n, host_tx,   MEC5_DMA_IDX_HOST_TX),                           \
			MEC5_DMA_EP(n, host_rx,   MEC5_DMA_IDX_HOST_RX),                           \
			MEC5_DMA_EP(n, target_tx, MEC5_DMA_IDX_TGT_TX),                            \
			MEC5_DMA_EP(n, target_rx, MEC5_DMA_IDX_TGT_RX),                            \
		},                                                                                 \
		.dma_buf = mec5_i2c_dma_buf_##n,                                                   \
		.irq_config_func = mec5_i2c_irq_cfg_##n,                                           \
	};                                                                                         \
	I2C_DEVICE_DT_INST_DEFINE(n, mec5_i2c_init, NULL, &mec5_i2c_data_##n,                      \
				  &mec5_i2c_cfg_##n, POST_KERNEL,                                  \
				  CONFIG_I2C_INIT_PRIORITY, &mec5_i2c_api);                        \
	static void mec5_i2c_irq_cfg_##n(void)                                                     \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), mec5_nl_isr,                \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(MEC5_I2C_DEVICE)
