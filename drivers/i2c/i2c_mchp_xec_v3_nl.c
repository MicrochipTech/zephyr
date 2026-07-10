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
 *                HCMD has START0 | STOP, no STARTN.
 *   write-read:  pulls (1 + wr_len + 1) bytes via DMA, drives
 *                START -> wr-addr -> wr-data... -> Sr -> rd-addr,
 *                then PAUSEs (the HW clears HCMD.PROCEED) so software
 *                can reprogram the DMA channel for PERIPHERAL_TO_MEMORY
 *                targeting the user's RX buffer. Software sets
 *                HCMD.PROCEED, the HW clocks rd_len bytes into memory
 *                via DMA and drives STOP. HDONE fires both for the
 *                PAUSE event and the final STOP. HCMD has
 *                START0 | STARTN | STOP.
 *   read-only:   pulls 1 byte (the rd-addr) via DMA, drives
 *                START -> rd-addr, then PAUSEs for the direction
 *                switch (same way as write-read, but with no
 *                preceding write phase). HCMD has START0 | STOP --
 *                STARTN is NOT set; emitting an extra Sr after a
 *                zero-length write phase is a degenerate I2C shape
 *                and is rejected as a protocol error by the v3.8
 *                target FSM (CMPL.TPROT).
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

#define TCMD_RUN     BIT(XEC_I2C_TCMD_RUN_POS)
#define TCMD_PROCEED BIT(XEC_I2C_TCMD_PROC_POS)

/* Target-mode CMPL bits. CMPL.TDONE fires both for the host-read pause
 * (TCMD.RUN==1, TCMD.PROCEED==0) and for the end of an inbound buffer
 * fill (TCMD.RCL exhausted). The end-of-transaction signal is
 * CMPL.IDLE -- the same NBB 0->1 edge latch the controller-mode path
 * uses. On v3.8 silicon CFG.STD_NL_IEN (bit 27) does NOT cause SR.STO
 * or CMPL.DTS_STS to fire on an externally-generated STOP, so the
 * driver does not enable it; CMPL.IDLE alone carries the signal.
 */
#define CMPL_TDONE     BIT(XEC_I2C_CMPL_TDONE_POS)
#define CMPL_TPROT     BIT(XEC_I2C_CMPL_TPROT_POS)
#define CMPL_RPT_RD    BIT(XEC_I2C_CMPL_RPT_RD_POS)
#define CMPL_RPT_WR    BIT(XEC_I2C_CMPL_RPT_WR_POS)
#define CMPL_TNAKR_STS BIT(XEC_I2C_CMPL_TNAKR_STS_POS)
#define CMPL_DTS_STS   BIT(XEC_I2C_CMPL_DTS_STS_POS)

#define CFG_TD_IEN BIT(XEC_I2C_CFG_TD_IEN_POS)
#define CFG_HD_IEN BIT(XEC_I2C_CFG_HD_IEN_POS)

#define CFG_AAT_IEN BIT(XEC_I2C_CFG_AAT_IEN_POS)

/* Bit 0 of the matched target address byte is the bus R/W bit:
 * 0 -> host wrote to us (we deliver data via buf_write_received),
 * 1 -> host read from us (no write delivery).
 */
#define XEC_I2C_NL_TGT_RBIT 0x01U

/* Bus-idle status / interrupt-enable. CMPL.IDLE latches when SR.NBB
 * transitions 0->1 (the controller has driven STOP and released the
 * lines, so the bus is no longer busy). CFG.IDLE_IEN gates that latch
 * onto the controller's interrupt output. v3.8 silicon has a bug where
 * setting IDLE_IEN while NBB==1 (already idle) immediately fires the
 * interrupt — the driver works around this by enabling IDLE_IEN inside
 * the HDONE ISR (controller mode) where NBB is guaranteed to be 0
 * because the controller is still mid-STOP-generation when HDONE fires.
 *
 * Target mode is different: a host-write that fills the target buffer
 * never fires TDONE (the FSM stalls after the post-RCL=0 NAK and never
 * asserts TDONE on the host's STOP), so IDLE_IEN MUST be on before the
 * host's STOP for every write shape -- otherwise the target wedges with
 * AAT=1, RUN=1, PROC=1 and holds the bus. But arming IDLE_IEN while the
 * bus is idle (as target_arm would have to, between transactions) trips
 * the NBB==1 bug and fires a spurious IRQ on every (re-)arm.
 *
 * The fix mirrors controller mode's HDONE trick: gate IDLE_IEN on the
 * address-match interrupt. target_arm enables CFG.AAT_IEN (bit 28) and
 * leaves IDLE_IEN OFF. AAT_IEN only fires when a host actually matches
 * one of our OWN addresses (~7 clocks in), so it never fires at idle.
 * The address-match ISR runs with NBB==0 (transaction open, before any
 * STOP) and there enables IDLE_IEN -- the exact NBB==0 window the bug
 * requires -- then disables AAT_IEN so a repeated START within the same
 * transaction does not re-fire it. IDLE_IEN then stays on until the
 * closing STOP's IDLE, where handle_stop re-arms via target_arm
 * (AAT_IEN back on, IDLE_IEN off). AAT_IEN carries no address identity
 * (DATA/IAS are not updated until the 8th clock) -- it is used ONLY as
 * the "a transaction is opening" edge to arm IDLE_IEN. The DMA-progress
 * guard in target_handle_stop is retained as a backstop.
 */
#define CMPL_IDLE    BIT(XEC_I2C_CMPL_IDLE_POS)
#define CFG_IDLE_IEN BIT(XEC_I2C_CFG_IDLE_IEN_POS)

/* RW1C bits the target-mode driver acknowledges and clears across every
 * transaction. TPROT, RPT_RD, RPT_WR are informational status latches
 * the HW asserts during certain transaction shapes (e.g. v3.8 silicon
 * sets RPT_RD on host-write-then-Sr-read sequences) and that do NOT
 * fire an interrupt on their own. TNAKR_STS latches when the FSM NAKs
 * a byte (the buffer-fill case asserts it); empirically, leaving it
 * set into the next transaction biases the FSM to NAK after the
 * address byte on a subsequent host-write, so it must be cleared at
 * every re-arm just like TPROT/RPT_*. DTS_STS is cleared
 * defensively even though the v3.8 STD_NL_IEN path does not assert it
 * for an externally-generated STOP. Left unacked these survive into
 * the next transaction and confuse anyone reading CMPL after the
 * fact -- the driver clears them at every re-arm.
 */
#define CMPL_TGT_CLEAR                                                                             \
	(CMPL_TDONE | CMPL_IDLE | CMPL_TPROT | CMPL_RPT_RD | CMPL_RPT_WR | CMPL_TNAKR_STS |        \
	 CMPL_DTS_STS)

/* BBCR (bit-bang control register) has two operating modes on v3.8:
 *
 *   Live-readback (BBM_EN=0, CM=1, i.e. BBCR=0x80): pins stay on the
 *   I2C engine; BBCR.SCL_IN / BBCR.SDA_IN reflect the live line state.
 *   The driver leaves BBCR in this mode whenever the bus-recovery
 *   path is not actively driving the lines, so any read picks up the
 *   true line state without disturbing I2C operation.
 *
 *   Bit-bang drive (BBM_EN=1, CM=0): pins are routed to BB control.
 *   Bits 1 and 2 are the SCL/SDA "direction" bits — 0 = input (line
 *   released to the external pull-up, floats high), 1 = output (line
 *   driven low by HW). Bits 3 and 4 (the legacy output-value bits)
 *   are not used on v3.8 silicon — direction alone selects drive-low
 *   versus release. The four BBCR_BB_* values below cover every
 *   combination the recovery sequence needs.
 */
#define BBCR_SCL_IN BIT(XEC_I2C_BBCR_SCL_IN_POS)
#define BBCR_SDA_IN BIT(XEC_I2C_BBCR_SDA_IN_POS)

#define BBCR_LIVE_RD     0x80U /* CM=1, BBM_EN=0: I2C-driven, readback live  */
#define BBCR_BB_RELEASED 0x01U /* BBM_EN=1, both dirs=input, both released   */
#define BBCR_BB_SCL_LOW  0x03U /* BBM_EN=1, SCL drive-low, SDA released      */
#define BBCR_BB_SDA_LOW  0x05U /* BBM_EN=1, SDA drive-low, SCL released      */

/* "Bus is idle and controller is healthy" pattern in the legacy SR
 * register: PIN=1 (no service required), NBB=1 (bus not busy), no
 * error bits set. Anything else triggers bus recovery.
 */
#define SR_IDLE 0x81U

/* Recovery timing: nine clocks at ~100 kHz with one STOP, repeated
 * up to 10 times against a stuck slave. SCL stuck-low timeout is 10
 * polls at 1 ms each (10 ms total) — long enough to ride out a
 * slow-clocking slave but not long enough to wedge the calling
 * thread for "real" timeouts.
 */
#define XEC_I2C_NL_BB_HALF_PERIOD_US   5U
#define XEC_I2C_NL_BB_POLL_INTERVAL_US 1000U
#define XEC_I2C_NL_BB_SCL_POLL_LOOPS   10U
#define XEC_I2C_NL_BB_SDA_RECOV_LOOPS  10U
#define XEC_I2C_NL_BB_RECOV_CLOCKS     9U

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
#define XEC_I2C_NL_CR_DFLT                                                                         \
	(BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS) | BIT(XEC_I2C_CR_PIN_POS))

enum xec_i2c_nl_state {
	XEC_I2C_NL_IDLE,
	XEC_I2C_NL_TX,
	XEC_I2C_NL_RX,
};

enum xec_i2c_nl_mode {
	XEC_I2C_NL_MODE_CONTROLLER,
	XEC_I2C_NL_MODE_TARGET,
};

/* Per-target-address slot. The v3.8 OA register exposes two 7-bit
 * address slots; up to two i2c_target_register() calls populate them.
 */
struct xec_i2c_nl_target_slot {
	struct i2c_target_config *cfg;
};

/* Target-mode RX/TX phase tracking, used by the target ISR to know
 * whether the next event is a fresh address byte, an inbound data
 * stream, or an outbound stream the driver is mid-clocking.
 */
enum xec_i2c_nl_target_phase {
	XEC_I2C_NL_TGT_IDLE, /* Armed, waiting for an address match. */
	XEC_I2C_NL_TGT_RX,   /* Inbound write in progress.           */
	XEC_I2C_NL_TGT_TX,   /* Outbound read in progress.           */
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
	{
		/* 100 kHz, 50/50 duty */
		.data_timing = 0x0C4D5006U,
		.idle_scaling = 0x01FC01EDU,
		.timeout_scaling = 0x4B9CC2C7U,
		.bus_clock = 0x4F4FU,
		.mr1 = 0x05U,
	},
	{
		/* 400 kHz, lo:hi ~ 1.53 */
		.data_timing = 0x040A0A06U,
		.idle_scaling = 0x01000050U,
		.timeout_scaling = 0x159CC2C7U,
		.bus_clock = 0x0F17U,
		.mr1 = 0x05U,
	},
	{
		/* 1 MHz, lo:hi ~ 1.8 */
		.data_timing = 0x04060601U,
		.idle_scaling = 0x01000050U,
		.timeout_scaling = 0x089CC2C7U,
		.bus_clock = 0x0509U,
		.mr1 = 0x05U,
	},
};

struct xec_i2c_nl_config {
	uintptr_t base;
	const struct device *dma_dev;
	void (*irq_connect)(void);
	uint8_t *bounce_buf;
	size_t bounce_buf_size;
	uint8_t *tgt_rx_buf;    /* NULL when target mode is unsupported */
	size_t tgt_rx_buf_size; /* 0 when target mode is unsupported    */
	/* Target-mode streaming-RX configuration. tgt_rx_chunk_count == 1
	 * preserves the historical "single buf_write_received per
	 * transaction" shape: the bounce buffer is a single block, RCL is
	 * pre-armed to its size, and the host's STOP drives the only
	 * delivery via handle_stop. tgt_rx_chunk_count > 1 splits the
	 * bounce buffer into N equal-sized chunks and configures the
	 * target DMA channel as a cyclic chain; per-chunk DONE callbacks
	 * dispatch each chunk via buf_write_received and the bounce
	 * buffer is reused across cycles, so transactions of arbitrary
	 * size fit through a small per-instance bounce.
	 */
	uint16_t tgt_rx_chunk_count;
	uint16_t tgt_rx_chunk_size; /* tgt_rx_buf_size / tgt_rx_chunk_count */
	uint32_t dflt_freq;
	uint8_t girq;
	uint8_t girq_pos;
	uint16_t enc_pcr;
	uint8_t dma_chan;     /* host-mode channel  */
	uint8_t dma_slot;     /* host-mode trigsrc  */
	uint8_t tgt_dma_chan; /* target-mode channel; valid if tgt_rx_buf != NULL */
	uint8_t tgt_dma_slot; /* target-mode trigsrc */
};

/* Parsed summary of an i2c_transfer() request after flag/shape validation.
 * Filled by xec_i2c_nl_parse and consumed by xec_i2c_nl_run. Defined ahead
 * of xec_i2c_nl_data so the async path can cache the in-flight group by value.
 */
struct xec_i2c_nl_xfer {
	struct i2c_msg *msgs;
	uint8_t num_msgs;
	uint8_t first_read;    /* index of first read msg, or num_msgs if none */
	uint16_t total_wr_len; /* sum of write-msg lens (capped at LEN_MAX)    */
	uint16_t total_rd_len; /* sum of read-msg lens                         */
	bool has_read;
	bool rx_via_bounce; /* true when M > 1 — DMA can't scatter, so the
			     * read phase lands in the bounce buffer and is
			     * memcpy'd into the user buffers afterward.
			     */
};

struct xec_i2c_nl_data {
	const struct device *ctrl;
	struct k_sem lock;
	struct k_sem pause_sem;
	struct k_sem done_sem;

	enum xec_i2c_nl_state state;
	int xfer_err;

	uint8_t active_port; /* XEC_I2C_NL_INVALID_PORT until programmed */
	uint32_t active_freq;

#ifdef CONFIG_I2C_MCHP_XEC_V3_NL_STATE_CAPTURE
	volatile uint32_t capidx;
	volatile uint8_t capture[CONFIG_I2C_MCHP_XEC_V3_NL_STATE_CAPTURE_SIZE] __aligned(4);
#endif
};

struct xec_i2c_nl_port_config {
	const struct device *parent;
	const struct pinctrl_dev_config *pcfg;
	uint32_t bitrate;
	uint8_t port_id;
	bool is_default;
};

#ifdef CONFIG_I2C_MCHP_XEC_V3_NL_STATE_CAPTURE
static void xec_i2c_nl_cap_init(struct xec_i2c_nl_data *xdat)
{
	xdat->capidx = 0;
	memset((void *)xdat->capture, 0, CONFIG_I2C_MCHP_XEC_V3_NL_STATE_CAPTURE_SIZE);
}

static void xec_i2c_nl_cap_update(struct xec_i2c_nl_data *xdat, uint8_t capval)
{
	if (xdat->capidx >= CONFIG_I2C_MCHP_XEC_V3_NL_STATE_CAPTURE_SIZE) {
		return;
	}

	xdat->capture[xdat->capidx++] = capval;
}

int mchp_xec_i2c_nl_clear_capture(const struct device *port_dev)
{
	if (port_dev == NULL) {
		return -EINVAL;
	}

	const struct xec_i2c_nl_port_config *pc = port_dev->config;
	struct xec_i2c_nl_data *const xdat = pc->parent->data;

	if (xdat->state != XEC_I2C_NL_IDLE) {
		return -EBUSY;
	}

	k_sem_take(&xdat->lock, K_FOREVER);
	xec_i2c_nl_cap_init(xdat);
	k_sem_give(&xdat->lock);

	return 0;
}

int mchp_xec_i2c_nl_copy_capture(const struct device *port_dev, uint8_t *capdest,
				 size_t capdest_size)
{
	if ((port_dev == NULL) || (capdest == NULL)) {
		return -EINVAL;
	}

	if (capdest_size == 0) {
		return 0;
	}

	const struct xec_i2c_nl_port_config *pc = port_dev->config;
	struct xec_i2c_nl_data *const xdat = pc->parent->data;

	if (xdat->state != XEC_I2C_NL_IDLE) {
		return -EBUSY;
	}

	k_sem_take(&xdat->lock, K_FOREVER);

	size_t n = (capdest_size < CONFIG_I2C_MCHP_XEC_V3_NL_STATE_CAPTURE_SIZE)
			   ? capdest_size
			   : CONFIG_I2C_MCHP_XEC_V3_NL_STATE_CAPTURE_SIZE;

	memcpy(capdest, (const void *)xdat->capture, n);

	k_sem_give(&xdat->lock);

	return 0;
}
#else
static void xec_i2c_nl_cap_init(struct xec_i2c_nl_data *xdat)
{
}

static void xec_i2c_nl_cap_update(struct xec_i2c_nl_data *xdat, uint8_t capval)
{
}

int mchp_xec_i2c_nl_clear_capture(const struct device *port_dev)
{
	return -ENOSYS;
}

int mchp_xec_i2c_nl_copy_capture(const struct device *i2c_nl_dev, uint8_t *capdest,
				 size_t capdest_size)
{
	return -ENOSYS;
}
#endif

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

/* Full controller programming: PCR reset, GIRQ enable, port select, timing,
 * and HDONE interrupt enable. Called from ctrl_init and whenever vport
 * configure changes the bus frequency.
 *
 * Must only be called when no transfer is in flight (lock held by caller,
 * or before any transfers are issued).
 *
 * Sequence:
 * Disable controller before PCR reset. Reset affects both port mux and frequency.
 * Short delay after reset to allow clearing of status to propagate.
 * Program controller registers
 *
 */
static int xec_i2c_nl_program_ctrl(const struct device *ctrl, uint32_t freqhz, uint8_t port)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	const struct xec_i2c_nl_timing *tm = xec_i2c_nl_timing_for(freqhz);
	uintptr_t base = cfg->base;

	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, MCHP_MEC_ECIA_GIRQ_DIS);

	sys_write32(0U, base + XEC_I2C_CFG_OFS);

	soc_xec_pcr_reset_en(cfg->enc_pcr);
	k_busy_wait(10U);

	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);

	/* PIN=1 to clear any latent assertion left by the legacy engine. */
	sys_write8(BIT(XEC_I2C_CR_PIN_POS), base + XEC_I2C_CR_OFS);

	/* Port select, filters on, general-call disabled, HDONE interrupt
	 * enabled. IDLE_IEN is intentionally LEFT OFF here — see the
	 * comment on CFG_IDLE_IEN above and the ISR for why it has to be
	 * enabled later (inside the HDONE handler at NL-finished time).
	 * ENAB is set last after timing has been written.
	 */
	sys_write32(XEC_I2C_CFG_PORT_SET(port) | BIT(XEC_I2C_CFG_FEN_POS) |
			    BIT(XEC_I2C_CFG_GC_DIS_POS) | BIT(XEC_I2C_CFG_HD_IEN_POS),
		    base + XEC_I2C_CFG_OFS);

	/* Clear any latched CMPL bits we care about so that a stale state
	 * (left over from a prior run before the PCR reset, or from the
	 * power-on default) cannot fire the moment GIRQ is enabled.
	 */
	xec_i2c_v3_cmpl_clear(base, CMPL_HDONE | CMPL_IDLE | CMPL_ERR);

	sys_write32(tm->data_timing, base + XEC_I2C_DT_OFS);
	sys_write32(tm->idle_scaling, base + XEC_I2C_ISC_OFS);
	sys_write32(tm->timeout_scaling, base + XEC_I2C_TMOUT_SC_OFS);
	sys_write32((uint32_t)tm->bus_clock, base + XEC_I2C_BCLK_OFS);
	sys_write32((uint32_t)tm->mr1, base + XEC_I2C_MR1_OFS);

	sys_write8(XEC_I2C_NL_CR_DFLT, base + XEC_I2C_CR_OFS);
	sys_set_bit(base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);
	/* Enable-to-first-transfer settling window; matches v2. */
	k_busy_wait(20U);

	/* Leave BBCR in live-readback mode so any later read of
	 * BBCR.SCL_IN / BBCR.SDA_IN (e.g. from the recovery path)
	 * reflects the true line state without engaging bit-bang
	 * drive. Pins remain under I2C control.
	 */
	sys_write8(BBCR_LIVE_RD, base + XEC_I2C_BBCR_OFS);

	/* Clear the GIRQ status one more time before unmasking so any
	 * latch from PCR reset or earlier configuration cannot ride into
	 * NVIC the moment we enable.
	 */
	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);
	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, MCHP_MEC_ECIA_GIRQ_EN);

	data->active_freq = freqhz;
	data->active_port = port;

	return 0;
}

/* The v3.8 controller requires a full PCR reset when the port MUX
 * (or frequency) changes. A soft RMW of CFG.PORT alone -- which
 * this function did in prior revisions -- leaves internal FSM
 * state stale and the very next transfer on the new port returns
 * -ENXIO or -EIO on the address byte with no NAK on the wire.
 * Reset here through program_ctrl, which:
 *   - disables the GIRQ,
 *   - runs soc_xec_pcr_reset_en to clear all internal latches,
 *   - re-writes CFG (with the new port), timing, CR, and BBCR,
 *   - clears CMPL RW1C latches,
 *   - re-enables the GIRQ,
 *   - updates data->active_port / active_freq on the way out.
 * program_ctrl is safe to call here because every caller of
 * apply_port either holds data->lock (vport_transfer,
 * vport_recover_bus) or runs before any transfers are issued
 * (port_init).
 */
static int xec_i2c_nl_apply_port(const struct device *port_dev)
{
	const struct xec_i2c_nl_port_config *pc = port_dev->config;
	const struct device *ctrl = pc->parent;
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	uint32_t freq = 0;
	int rc = 0;

	if (data->active_port == pc->port_id) {
		return 0;
	}

	rc = pinctrl_apply_state(pc->pcfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("pinctrl_apply_state(%s)=%d", port_dev->name, rc);
		return rc;
	}

	freq = (pc->bitrate != 0U) ? pc->bitrate : cfg->dflt_freq;
	return xec_i2c_nl_program_ctrl(ctrl, freq, pc->port_id);
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

/* Drive XEC_I2C_NL_BB_RECOV_CLOCKS SCL pulses at ~100 kHz while leaving
 * SDA released. Caller must already have engaged bit-bang mode (BBCR
 * set to BBCR_BB_RELEASED).
 */
static void xec_i2c_nl_bb_clock_burst(uintptr_t base)
{
	for (uint32_t i = 0; i < XEC_I2C_NL_BB_RECOV_CLOCKS; i++) {
		sys_write8(BBCR_BB_SCL_LOW, base + XEC_I2C_BBCR_OFS);
		k_busy_wait(XEC_I2C_NL_BB_HALF_PERIOD_US);
		sys_write8(BBCR_BB_RELEASED, base + XEC_I2C_BBCR_OFS);
		k_busy_wait(XEC_I2C_NL_BB_HALF_PERIOD_US);
	}
}

/* Generate an I2C STOP condition: SDA low -> high while SCL stays high.
 * Caller must already be in bit-bang mode with SCL released.
 */
static void xec_i2c_nl_bb_stop(uintptr_t base)
{
	sys_write8(BBCR_BB_SDA_LOW, base + XEC_I2C_BBCR_OFS);
	k_busy_wait(XEC_I2C_NL_BB_HALF_PERIOD_US);
	sys_write8(BBCR_BB_RELEASED, base + XEC_I2C_BBCR_OFS);
	k_busy_wait(XEC_I2C_NL_BB_HALF_PERIOD_US);
}

/* Recover the bus when SR != SR_IDLE on entry to a transfer. Called
 * with the controller lock held.
 *
 *   1. PCR-reset and reprogram the controller for (freq, port). The
 *      new program_ctrl leaves BBCR in live-readback mode, so the
 *      next BBCR read returns the live SCL/SDA state with pins still
 *      on the I2C engine. If SR now reads SR_IDLE the controller
 *      alone was the problem.
 *   2. Poll BBCR.SCL_IN up to XEC_I2C_NL_BB_SCL_POLL_LOOPS times at
 *      XEC_I2C_NL_BB_POLL_INTERVAL_US apart. SCL stuck low past that
 *      means a slave is holding the clock — this side cannot unstick
 *      it; return -EIO.
 *   3. If SDA is low, switch the pins to bit-bang control
 *      (BBCR.BBM_EN=1, CM=0, both directions=input -> released) and
 *      drive 9 SCL clocks + a STOP, up to XEC_I2C_NL_BB_SDA_RECOV_LOOPS
 *      times or until SDA releases. While in bit-bang mode the
 *      readback bits remain valid (per v3.8 BBCR semantics: in BB
 *      mode the readback shows what external HW is driving).
 *   4. Return BBCR to live-readback mode (BBCR_LIVE_RD = 0x80). The
 *      lines are now back under I2C control and BBCR.SCL_IN /
 *      BBCR.SDA_IN remain valid for the final check.
 *   5. PCR-reset and reprogram once more so the I2C engine starts
 *      fresh on the now-recovered bus. (program_ctrl re-writes BBCR
 *      to BBCR_LIVE_RD as part of its tail.)
 *   6. Final live read: both SCL and SDA must be high; otherwise
 *      return -EIO.
 *
 * On success the controller is left configured for (freq, port) with
 * BBCR in live-readback mode and ready to issue a transfer.
 */
static int xec_i2c_nl_bus_recover(const struct device *ctrl, uint32_t freq, uint8_t port)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	uintptr_t base = cfg->base;
	uint8_t bbcr = 0;
	int rc;

	rc = xec_i2c_nl_program_ctrl(ctrl, freq, port);
	if (rc != 0) {
		return rc;
	}
	if (sys_read8(base + XEC_I2C_SR_OFS) == SR_IDLE) {
		return 0;
	}

	for (uint32_t i = 0; i < XEC_I2C_NL_BB_SCL_POLL_LOOPS; i++) {
		bbcr = sys_read8(base + XEC_I2C_BBCR_OFS);
		if ((bbcr & BBCR_SCL_IN) != 0U) {
			break;
		}
		k_busy_wait(XEC_I2C_NL_BB_POLL_INTERVAL_US);
	}
	if ((bbcr & BBCR_SCL_IN) == 0U) {
		LOG_ERR("i2c-recover: SCL stuck low");
		return -EIO;
	}

	if ((bbcr & BBCR_SDA_IN) == 0U) {
		sys_write8(BBCR_BB_RELEASED, base + XEC_I2C_BBCR_OFS);
		k_busy_wait(XEC_I2C_NL_BB_HALF_PERIOD_US);

		for (uint32_t i = 0; i < XEC_I2C_NL_BB_SDA_RECOV_LOOPS; i++) {
			xec_i2c_nl_bb_clock_burst(base);
			xec_i2c_nl_bb_stop(base);
			bbcr = sys_read8(base + XEC_I2C_BBCR_OFS);
			if ((bbcr & BBCR_SDA_IN) != 0U) {
				break;
			}
		}
	}

	/* Return pins to I2C control with live readback still on. */
	sys_write8(BBCR_LIVE_RD, base + XEC_I2C_BBCR_OFS);

	(void)xec_i2c_nl_program_ctrl(ctrl, freq, port);

	bbcr = sys_read8(base + XEC_I2C_BBCR_OFS);
	if ((bbcr & (BBCR_SCL_IN | BBCR_SDA_IN)) != (BBCR_SCL_IN | BBCR_SDA_IN)) {
		LOG_ERR("i2c-recover: SCL=%u SDA=%u still not both high",
			(bbcr & BBCR_SCL_IN) ? 1U : 0U, (bbcr & BBCR_SDA_IN) ? 1U : 0U);
		return -EIO;
	}

	return 0;
}

/* Completion signaling. In synchronous mode the ISR/DMA callbacks release
 * the calling thread via pause_sem/done_sem; in async mode (async_active)
 * they instead advance data->astep and submit the controller's work item so
 * the next step runs on the shared work-queue thread. These helpers hide
 * that fork so the ISR body reads the same in both builds. On error the
 * caller sets data->xfer_err first; the work handler checks it before astep.
 */
static inline void xec_i2c_nl_signal_pause(struct xec_i2c_nl_data *data)
{
	k_sem_give(&data->pause_sem);
}

static inline void xec_i2c_nl_signal_done(struct xec_i2c_nl_data *data)
{
	k_sem_give(&data->done_sem);
}

static inline void xec_i2c_nl_signal_error(struct xec_i2c_nl_data *data)
{
	k_sem_give(&data->pause_sem);
	k_sem_give(&data->done_sem);
}

/* DMA callbacks */

static void xec_i2c_nl_tx_dma_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
				 int status)
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
	/* Wake whichever waiter is parked on this transfer (or, in async
	 * mode, hand the error to the work queue).
	 */
	xec_i2c_nl_signal_error(data);
}

/* DMA-done on success is not the completion signal — the IDLE
 * interrupt (enabled inside the HDONE handler at NL-finished
 * time) fires after the bus has returned to idle, which the
 * v3.8 HW only reaches some clocks AFTER the last byte has
 * landed in memory. Releasing the calling thread here would beat
 * IDLE to the punch and leave the bus mid-STOP from the
 * caller's perspective.
 */
static void xec_i2c_nl_rx_dma_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
				 int status)
{
	struct xec_i2c_nl_data *data = user_data;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		data->xfer_err = status;
		xec_i2c_nl_signal_done(data);
		return;
	}
}

/* I2C controller ISR */
static void xec_i2c_nl_isr(const struct device *ctrl)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	uintptr_t base = cfg->base;
	uint32_t cmpl = sys_read32(base + XEC_I2C_CMPL_OFS);
	uint32_t cfgr = 0, hcmd = 0;

	xec_i2c_nl_cap_update(data, 0x80U);

	if ((cmpl & CMPL_ERR) != 0U) {
		xec_i2c_nl_cap_update(data, 0x81U);
		if ((cmpl & CMPL_HNAK) != 0U) {
			xec_i2c_nl_cap_update(data, 0x82U);
			data->xfer_err = -ENXIO;
		} else if ((cmpl & CMPL_LAB) != 0U) {
			xec_i2c_nl_cap_update(data, 0x83U);
			data->xfer_err = -EAGAIN;
		} else {
			xec_i2c_nl_cap_update(data, 0x84U);
			data->xfer_err = -EIO;
		}

		/* Clear the latched CMPL bits and stop DMA. The full PCR
		 * reset that recovers the engine happens on the thread side
		 * (xec_i2c_nl_abort) — doing it here would race with the
		 * thread coming out of its sem wait.
		 */
		xec_i2c_v3_cmpl_clear(base, CMPL_ERR | CMPL_HDONE | CMPL_IDLE);
		dma_stop(cfg->dma_dev, cfg->dma_chan);
		xec_i2c_nl_signal_error(data);
		goto out;
	}

	/* HDONE — distinguishes PAUSE (mid-transfer direction switch) from
	 * NL-finished. PROCEED is cleared by the HW on every HDONE; RUN
	 * is cleared only at NL-finished:
	 *
	 *   HCMD.RUN==1 && HCMD.PROCEED==0  -> PAUSE
	 *   HCMD.RUN==0 && HCMD.PROCEED==0  -> NL processing complete
	 *
	 * NL-finished is NOT yet the end of the transfer on the bus: per
	 * the v3.8 errata note, the controller has not yet driven STOP or
	 * released the lines at this point. We enable CFG.IDLE_IEN here
	 * (NBB is guaranteed 0 — bus is busy mid-STOP) to wake on the
	 * NBB 0->1 edge that signals the bus is actually idle. The IDLE
	 * branch below then signals done_sem.
	 */
	if ((cmpl & CMPL_HDONE) != 0U) {
		xec_i2c_nl_cap_update(data, 0x85U);
		xec_i2c_v3_cmpl_clear(base, CMPL_HDONE);

		if (data->state == XEC_I2C_NL_TX || data->state == XEC_I2C_NL_RX) {
			xec_i2c_nl_cap_update(data, 0x86U);

			hcmd = sys_read32(base + XEC_I2C_HCMD_OFS);

			if ((hcmd & HCMD_RUN) != 0U && (hcmd & HCMD_PROCEED) == 0U) {
				/* PAUSE — driver thread will reprogram DMA
				 * for the read phase and resume by setting
				 * HCMD.PROCEED.
				 */
				xec_i2c_nl_cap_update(data, 0x87U);
				xec_i2c_nl_signal_pause(data);
			} else if ((hcmd & HCMD_RUN) == 0U && (hcmd & HCMD_PROCEED) == 0U) {
				/* NL-finished. Enable the IDLE interrupt to
				 * detect the post-STOP bus-idle edge.
				 *
				 * If the bus has already gone idle by the
				 * time we get here (a slow or preempted
				 * ISR), the AND-with-IEN check below will
				 * pick it up in this same ISR invocation.
				 */
				xec_i2c_nl_cap_update(data, 0x88U);
				sys_set_bit(base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
			}
		}
	}

	/* IDLE — the bus has truly returned to idle (NBB transitioned 0
	 * to 1 after STOP). Per the v3.8 IDLE-IEN HW bug we only trust
	 * this signal when we ourselves enabled CFG.IDLE_IEN above, so
	 * AND the CMPL.IDLE status bit with the live IEN bit. CMPL.IDLE
	 * may have been latched outside an active transfer (bus idle at
	 * boot, between calls, etc.); without the IEN check we would
	 * spuriously release the next caller.
	 */
	if ((cmpl & CMPL_IDLE) != 0U) {
		xec_i2c_nl_cap_update(data, 0x89U);

		cfgr = sys_read32(base + XEC_I2C_CFG_OFS);

		if ((cfgr & CFG_IDLE_IEN) != 0U) {
			/* Disable IDLE_IEN now so the next transfer starts
			 * with IEN=0 — re-enabling at NL-finished is the
			 * documented workaround for the v3.8 bug that fires
			 * IDLE immediately if IEN is asserted while NBB==1.
			 */
			xec_i2c_nl_cap_update(data, 0x8AU);
			sys_clear_bit(base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
			xec_i2c_v3_cmpl_clear(base, CMPL_IDLE);

			if (data->state != XEC_I2C_NL_IDLE) {
				xec_i2c_nl_cap_update(data, 0x8BU);
				xec_i2c_nl_signal_done(data);
			}
		}
	}

out:
	/* The XEC GIRQ status bit is an edge latch — NVIC stays asserted
	 * until SW writes 1 to clear it, regardless of whether the CMPL
	 * RW1C bits behind it are still set. Clearing here avoids ISR
	 * re-entry on transient or spurious edges.
	 */
	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);

	xec_i2c_nl_cap_update(data, 0x8FU);
}

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

/* Walk the (sub-)array of i2c_msg's that make up one NL transaction,
 * validate the shape, and produce the summary xec_i2c_nl_run consumes.
 * Any flag/shape that can't be issued as a single START-to-STOP NL
 * transaction is rejected here.
 *
 * Callers in this driver always split a multi-transaction msg array at
 * I2C_MSG_STOP before calling parse, so the (msgs, num_msgs) range
 * passed in represents exactly one transaction. STOP appearing on a
 * non-last element of that range therefore signals an internal split
 * bug and is rejected as -ENOTSUP rather than silently tolerated.
 *
 * Accepted within a single transaction: [N writes] [M reads], 0 <= N,
 * 0 <= M, with total write or total read length non-zero. The first
 * read (when preceded by a write) must carry I2C_MSG_RESTART; STOP is
 * asserted in HCMD unconditionally regardless of whether it was set
 * on the last msg.
 */
static int xec_i2c_nl_parse(const struct xec_i2c_nl_config *cfg, struct i2c_msg *msgs,
			    uint8_t num_msgs, struct xec_i2c_nl_xfer *xfer)
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
	xfer->rx_via_bounce = seen_read && ((uint8_t)(num_msgs - xfer->first_read) > 1U);

	/* tx_total mirrors what xec_i2c_nl_run actually pushes through DMA:
	 *   pure-read (writes==0, has_read==1): just the rd-addr byte (1).
	 *   write-only / ping:                   1 + sum(wr_len).
	 *   write + read:                        1 + sum(wr_len) + 1.
	 */
	uint32_t tx_total;

	if (xfer->has_read && xfer->total_wr_len == 0U) {
		tx_total = 1U;
	} else {
		tx_total = 1U + xfer->total_wr_len + (xfer->has_read ? 1U : 0U);
	}

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

/* Build the contiguous TX bounce buffer. Three layouts depending on
 * the transfer shape:
 *
 *   Pure write / address-probe ping (has_read=false):
 *     [ wr-addr | wr1 | wr2 | ... | wrN ]
 *     bus: START + wr-addr + wr-data... + STOP
 *
 *   Write + read (has_read=true, total_wr_len > 0):
 *     [ wr-addr | wr1 | ... | wrN | rd-addr ]
 *     bus: START + wr-addr + wr-data... + Sr + rd-addr + rd-data... + STOP
 *
 *   Pure read (has_read=true, total_wr_len == 0):
 *     [ rd-addr ]
 *     bus: START + rd-addr + rd-data... + STOP
 *
 * The pure-read case MUST NOT emit a wr-addr-then-Sr-then-rd-addr
 * sequence: the v3.8 target HW flags that as a protocol error
 * (CMPL.TPROT in target mode), and it's a degenerate I2C shape
 * regardless (zero-length write phase followed immediately by a
 * repeated start).
 */
static void xec_i2c_nl_fill_tx_bounce(const struct xec_i2c_nl_config *cfg, uint16_t addr,
				      const struct xec_i2c_nl_xfer *xfer)
{
	uint8_t *buf = cfg->bounce_buf;
	uint8_t addr7 = (uint8_t)((addr & 0x7FU) << 1);

	if (xfer->has_read && xfer->total_wr_len == 0U) {
		/* Pure-read: only the rd-addr byte goes on the bus. */
		buf[0] = (uint8_t)(addr7 | XEC_I2C_NL_RWBIT_READ);
		return;
	}

	buf[0] = (uint8_t)(addr7 | XEC_I2C_NL_RWBIT_WRITE);

	size_t off = 1U;
	uint8_t end = xfer->has_read ? xfer->first_read : xfer->num_msgs;

	for (uint8_t i = 0; i < end; i++) {
		if (xfer->msgs[i].len > 0U) {
			memcpy(&buf[off], xfer->msgs[i].buf, xfer->msgs[i].len);
			off += xfer->msgs[i].len;
		}
	}

	if (xfer->has_read) {
		buf[1U + xfer->total_wr_len] = (uint8_t)(addr7 | XEC_I2C_NL_RWBIT_READ);
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

/* A single NL transaction is driven in three non-blocking pieces so the
 * same hardware sequence can be composed either by the synchronous
 * wrapper below (which blocks on semaphores between pieces) or, later, by
 * an event-driven async work handler (which returns between pieces and is
 * re-entered on each ISR event). See i2c_mchp_xec_v3_nl_callback_design.md.
 *
 *   xec_i2c_nl_start_group()      front half: fill bounce, TX DMA, HCMD
 *   xec_i2c_nl_begin_read_phase() middle (on PAUSE): RX DMA, PROCEED
 *   xec_i2c_nl_finish_group()     tail (on DONE): scatter_rx, IDLE
 *
 * None of the three wait. Each returns after launching hardware; the
 * caller is responsible for waiting for the next event and for aborting
 * on error (the pieces never abort themselves, so they are reusable from
 * both the thread and the future work-queue context).
 */

/* Front half of a single NL transaction: fill the TX bounce buffer,
 * program and start TX DMA, and write HCMD to launch the FSM. Does not
 * wait for completion — the caller picks up on the PAUSE (write->read) or
 * DONE event. On a setup failure nothing has been kicked off on the wire,
 * so no abort is needed; the state is returned to IDLE and the error is
 * propagated (matching the original inline behavior).
 */
static int xec_i2c_nl_start_group(const struct device *ctrl, uint16_t addr,
				  const struct xec_i2c_nl_xfer *xfer)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	uintptr_t base = cfg->base;
	bool pure_read = xfer->has_read && (xfer->total_wr_len == 0U);
	uint16_t total_write =
		pure_read ? 1U : (uint16_t)(1U + xfer->total_wr_len + (xfer->has_read ? 1U : 0U));
	int rc;

	xec_i2c_nl_cap_update(data, 0x20U);

	xec_i2c_nl_fill_tx_bounce(cfg, addr, xfer);

	data->xfer_err = 0;
	k_sem_reset(&data->pause_sem);
	k_sem_reset(&data->done_sem);
	data->state = XEC_I2C_NL_TX;

	rc = xec_i2c_nl_setup_tx_dma(ctrl, total_write);
	if (rc != 0) {
		xec_i2c_nl_cap_update(data, 0x21U);
		LOG_ERR("tx dma_config: %d", rc);
		data->state = XEC_I2C_NL_IDLE;
		return rc;
	}

	rc = dma_start(cfg->dma_dev, cfg->dma_chan);
	if (rc != 0) {
		xec_i2c_nl_cap_update(data, 0x22U);
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
			XEC_I2C_HCMD_RCL_SET((uint32_t)xfer->total_rd_len & 0xFFU) | HCMD_RUN |
			HCMD_PROCEED | HCMD_START0 | HCMD_STOP;

	if (xfer->has_read && !pure_read) {
		xec_i2c_nl_cap_update(data, 0x23U);
		hcmd |= HCMD_STARTN;
	}

	xec_i2c_nl_cap_update(data, 0x24U);
	sys_write32(hcmd, base + XEC_I2C_HCMD_OFS);

	return 0;
}

/* Middle of a write->read transaction, entered on the PAUSE event: the
 * FSM has clocked out the write portion and stalled with PROCEED clear.
 * Reprogram the channel for PERIPH->MEM and resume the FSM.
 *
 * For M == 1 the user buffer is the DMA target directly; for M > 1 the
 * bounce buffer (no longer needed by TX, which has completed) is reused
 * as the staging area, and the bytes are scattered into the user buffers
 * after STOP-detect. On failure the caller aborts and returns the error.
 */
static int xec_i2c_nl_begin_read_phase(const struct device *ctrl,
				       const struct xec_i2c_nl_xfer *xfer)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	uintptr_t base = cfg->base;
	int rc;

	dma_stop(cfg->dma_dev, cfg->dma_chan);

	uint8_t *rx_dst = xfer->rx_via_bounce ? cfg->bounce_buf : xfer->msgs[xfer->first_read].buf;

	rc = xec_i2c_nl_setup_rx_dma(ctrl, rx_dst, xfer->total_rd_len);
	if (rc != 0) {
		xec_i2c_nl_cap_update(data, 0x28U);
		LOG_ERR("rx dma_config: %d", rc);
		return rc;
	}

	data->state = XEC_I2C_NL_RX;

	rc = dma_start(cfg->dma_dev, cfg->dma_chan);
	if (rc != 0) {
		xec_i2c_nl_cap_update(data, 0x29U);
		LOG_ERR("rx dma_start: %d", rc);
		return rc;
	}

	xec_i2c_nl_cap_update(data, 0x2AU);
	sys_set_bit(base + XEC_I2C_HCMD_OFS, XEC_I2C_HCMD_PROC_POS);

	return 0;
}

/* Tail of a transaction, entered on the DONE (post-STOP IDLE) event:
 * scatter the staged bytes into the user buffers for M > 1 reads and
 * return the controller to IDLE.
 */
static void xec_i2c_nl_finish_group(const struct device *ctrl, const struct xec_i2c_nl_xfer *xfer)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;

	if (xfer->rx_via_bounce) {
		xec_i2c_nl_cap_update(data, 0x2EU);
		xec_i2c_nl_scatter_rx(cfg, xfer);
	}

	data->state = XEC_I2C_NL_IDLE;

	xec_i2c_nl_cap_update(data, 0x2FU);
}

/* Synchronous driver of one NL transaction: compose the three pieces
 * above, blocking on pause_sem (write->read direction switch) and
 * done_sem (post-STOP IDLE) between them. Behavior is identical to the
 * previous monolithic implementation.
 */
static int xec_i2c_nl_run(const struct device *ctrl, uint16_t addr,
			  const struct xec_i2c_nl_xfer *xfer)
{
	struct xec_i2c_nl_data *data = ctrl->data;
	int rc;

	rc = xec_i2c_nl_start_group(ctrl, addr, xfer);
	if (rc != 0) {
		/* start_group already returned state to IDLE; nothing is on
		 * the wire yet, so no abort is required.
		 */
		return rc;
	}

	if (xfer->has_read) {
		xec_i2c_nl_cap_update(data, 0x25U);
		rc = k_sem_take(&data->pause_sem, XEC_I2C_NL_TIMEOUT);
		if (rc != 0) {
			xec_i2c_nl_cap_update(data, 0x26U);
			LOG_ERR("pause wait: %d", rc);
			xec_i2c_nl_abort(ctrl);
			data->state = XEC_I2C_NL_IDLE;
			return -ETIMEDOUT;
		}
		if (data->xfer_err != 0) {
			xec_i2c_nl_cap_update(data, 0x27U);
			xec_i2c_nl_abort(ctrl);
			data->state = XEC_I2C_NL_IDLE;
			return data->xfer_err;
		}

		rc = xec_i2c_nl_begin_read_phase(ctrl, xfer);
		if (rc != 0) {
			xec_i2c_nl_abort(ctrl);
			data->state = XEC_I2C_NL_IDLE;
			return rc;
		}
	}

	xec_i2c_nl_cap_update(data, 0x2BU);
	rc = k_sem_take(&data->done_sem, XEC_I2C_NL_TIMEOUT);
	if (rc != 0) {
		xec_i2c_nl_cap_update(data, 0x2CU);
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
		xec_i2c_nl_cap_update(data, 0x2DU);
		xec_i2c_nl_abort(ctrl);
		data->state = XEC_I2C_NL_IDLE;
		return data->xfer_err;
	}

	xec_i2c_nl_finish_group(ctrl, xfer);

	return 0;
}

/* Zephyr i2c_driver_api */

static int xec_i2c_nl_vport_transfer(const struct device *port_dev, struct i2c_msg *msgs,
				     uint8_t num_msgs, uint16_t addr)
{
	const struct xec_i2c_nl_port_config *pc = port_dev->config;
	const struct device *ctrl = pc->parent;
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	int rc = 0;
	uint32_t freq = 0;
	uint8_t group_start = 0, group_end = 0, group_len = 0;

	if (num_msgs == 0U || msgs == NULL) {
		return -EINVAL;
	}
	if ((addr & ~0x7FU) != 0U) {
		return -EINVAL; /* 7-bit only */
	}

	k_sem_take(&data->lock, K_FOREVER);

	xec_i2c_nl_cap_init(data);

	xec_i2c_nl_cap_update(data, 1U);

	rc = xec_i2c_nl_apply_port(port_dev);
	if (rc != 0) {
		xec_i2c_nl_cap_update(data, 2U);
		k_sem_give(&data->lock);
		return rc;
	}

	/* Sanity-check the bus before kicking anything off. SR_IDLE
	 * (PIN=1, NBB=1, no error bits) means the controller is healthy
	 * and the bus is free; anything else means a previous transfer
	 * left state behind, the slave is still holding lines, or the
	 * physical port we just switched to is in a bad state. Run the
	 * bit-bang recovery sequence and bail if it can't restore SR_IDLE.
	 */
	if (sys_read8(cfg->base + XEC_I2C_SR_OFS) != SR_IDLE) {
		freq = (data->active_freq != 0U) ? data->active_freq : cfg->dflt_freq;

		xec_i2c_nl_cap_update(data, 3U);
		rc = xec_i2c_nl_bus_recover(ctrl, freq, pc->port_id);
		if (rc != 0) {
			xec_i2c_nl_cap_update(data, 4U);
			k_sem_give(&data->lock);
			return rc;
		}
		/* The recovery just ran program_ctrl, which writes the
		 * CFG.PORT field for `pc->port_id` and updates active_port.
		 * apply_port already applied pinctrl earlier, so no
		 * further work is needed before the transfer.
		 */
	}

	/* Zephyr's I2C API allows the msg array to contain multiple
	 * complete transactions: each I2C_MSG_STOP closes one transaction
	 * and the following msg (if any) opens the next with a fresh
	 * START. The NL HW only ever runs one transaction per HCMD write,
	 * so split the array into groups (each group = one transaction =
	 * one xec_i2c_nl_run call) and walk them sequentially under the
	 * controller lock. The caller sees one atomic i2c_transfer() —
	 * no other caller can interleave between sub-transactions.
	 *
	 * A group runs from group_start to the first msg carrying
	 * I2C_MSG_STOP within the remainder of the array, or to the last
	 * msg of the array (whichever comes first). The last msg of the
	 * last group does not need to carry I2C_MSG_STOP — the driver
	 * always asserts STOP in HCMD anyway.
	 */
	xec_i2c_nl_cap_update(data, 5U);

	group_start = 0;
	while (group_start < num_msgs) {
		group_end = group_start;

		while (group_end < (uint8_t)(num_msgs - 1U) &&
		       (msgs[group_end].flags & I2C_MSG_STOP) == 0U) {
			group_end++;
		}

		struct xec_i2c_nl_xfer xfer;

		group_len = (uint8_t)((group_end - group_start) + 1U);

		rc = xec_i2c_nl_parse(cfg, &msgs[group_start], group_len, &xfer);
		if (rc != 0) {
			xec_i2c_nl_cap_update(data, 6U);
			break;
		}

		rc = xec_i2c_nl_run(ctrl, addr, &xfer);
		if (rc != 0) {
			xec_i2c_nl_cap_update(data, 7U);
			break;
		}

		group_start = (uint8_t)(group_end + 1U);
	}

	xec_i2c_nl_cap_update(data, 8U);

	k_sem_give(&data->lock);

	return rc;
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

	if (freq != data->active_freq || data->active_port != pc->port_id) {
		rc = pinctrl_apply_state(pc->pcfg, PINCTRL_STATE_DEFAULT);
		if (rc == 0) {
			rc = xec_i2c_nl_program_ctrl(ctrl, freq, pc->port_id);
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

static int xec_i2c_nl_vport_recover_bus(const struct device *port_dev)
{
	const struct xec_i2c_nl_port_config *pc = port_dev->config;
	const struct device *ctrl = pc->parent;
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	uint32_t freq = (data->active_freq != 0U) ? data->active_freq : cfg->dflt_freq;
	int rc;

	k_sem_take(&data->lock, K_FOREVER);

	rc = xec_i2c_nl_apply_port(port_dev);
	if (rc != 0) {
		k_sem_give(&data->lock);
		return rc;
	}

	rc = xec_i2c_nl_bus_recover(ctrl, freq, pc->port_id);

	k_sem_give(&data->lock);
	return rc;
}

static DEVICE_API(i2c, xec_i2c_nl_port_api) = {
	.configure = xec_i2c_nl_vport_configure,
	.get_config = xec_i2c_nl_vport_get_config,
	.transfer = xec_i2c_nl_vport_transfer,
	.recover_bus = xec_i2c_nl_vport_recover_bus,
#ifdef CONFIG_I2C_RTIO
	/* RTIO submissions use the default work queue, which dispatches each SQE
	 * as a synchronous i2c_transfer. (Native async is provided via
	 * .transfer_cb above when CONFIG_I2C_CALLBACK is enabled.)
	 */
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

/* Custom mchp_xec_i2c.h API for runtime port query/select */

int mchp_xec_i2c_nl_port_get(const struct device *i2c_port_dev, uint8_t *port)
{
	const struct xec_i2c_nl_port_config *pc = NULL;

	if (i2c_port_dev == NULL || port == NULL) {
		return -EINVAL;
	}

	pc = i2c_port_dev->config;
	*port = pc->port_id;
	return 0;
}

int mchp_xec_i2c_nl_port_set(const struct device *i2c_port_dev, uint8_t port)
{
	if (i2c_port_dev == NULL || port >= XEC_I2C_CFG_MAX_PORT) {
		return -EINVAL;
	}

	const struct xec_i2c_nl_port_config *pc = i2c_port_dev->config;
	const struct xec_i2c_nl_config *cfg = pc->parent->config;
	struct xec_i2c_nl_data *const data = pc->parent->data;
	uint32_t freq = 0;
	int rc = 0;

	k_sem_take(&data->lock, K_FOREVER);
	if (data->active_port == port) {
		k_sem_give(&data->lock);
		return 0;
	}

	freq = (data->active_freq != 0U) ? data->active_freq : cfg->dflt_freq;
	rc = xec_i2c_nl_program_ctrl(pc->parent, freq, port);
	k_sem_give(&data->lock);

	return rc;
}

/* Driver initialization */

static int xec_i2c_nl_ctrl_init(const struct device *ctrl)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	int rc;

	data->ctrl = ctrl;
	data->state = XEC_I2C_NL_IDLE;
	data->active_port = XEC_I2C_NL_INVALID_PORT;
	data->active_freq = 0;

	k_sem_init(&data->lock, 1, 1);
	k_sem_init(&data->pause_sem, 0, 1);
	k_sem_init(&data->done_sem, 0, 1);

	if (!device_is_ready(cfg->dma_dev)) {
		LOG_ERR("dma %s not ready", cfg->dma_dev->name);
		return -ENODEV;
	}

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

/* Devicetree instantiation */

#define DT_DRV_COMPAT microchip_xec_i2c_v3_nl

#define XEC_I2C_NL_GIRQ(inst, idx)     MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, idx))
#define XEC_I2C_NL_GIRQ_POS(inst, idx) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, idx))

/* The controller binding does not carry clock-frequency — it lives on the
 * port nodes. The controller's default frequency is only the value the
 * controller boots up at; vport_configure() reprograms the bus rate per
 * port-device.
 */
#define XEC_I2C_NL_DFLT_FREQ(inst) I2C_BITRATE_STANDARD

/* Build-time guard: if the controller names a default-port, that port's
 * "controller" phandle must point back to this controller node. Expands to
 * nothing for controllers that omit default-port.
 */
#define XEC_I2C_NL_DEFPORT_ASSERT(inst)                                                            \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, default_port),                                      \
		   (BUILD_ASSERT(DT_SAME_NODE(DT_DRV_INST(inst),                                   \
			DT_PHANDLE(DT_INST_PHANDLE(inst, default_port), controller)),              \
			"default-port must reference a port on this controller");))

#define XEC_I2C_NL_CTRL_INIT(inst)                                                                 \
	XEC_I2C_NL_DEFPORT_ASSERT(inst)                                                            \
	static uint8_t                                                                             \
		__aligned(4) xec_i2c_nl_bounce_##inst[DT_INST_PROP(inst, bounce_buffer_size)];     \
	static void xec_i2c_nl_irq_connect_##inst(void)                                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), xec_i2c_nl_isr,       \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
	static const struct xec_i2c_nl_config xec_i2c_nl_cfg_##inst = {                            \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR(inst)),                                 \
		.irq_connect = xec_i2c_nl_irq_connect_##inst,                                      \
		.bounce_buf = xec_i2c_nl_bounce_##inst,                                            \
		.bounce_buf_size = DT_INST_PROP(inst, bounce_buffer_size),                         \
		.dflt_freq = XEC_I2C_NL_DFLT_FREQ(inst),                                           \
		.girq = XEC_I2C_NL_GIRQ(inst, 0),                                                  \
		.girq_pos = XEC_I2C_NL_GIRQ_POS(inst, 0),                                          \
		.enc_pcr = DT_INST_PROP(inst, pcr_scr),                                            \
		.dma_chan = DT_INST_DMAS_CELL_BY_NAME(inst, host, channel),                        \
		.dma_slot = DT_INST_DMAS_CELL_BY_NAME(inst, host, trigsrc),                        \
	};                                                                                         \
	static struct xec_i2c_nl_data xec_i2c_nl_data_##inst;                                      \
	DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_ctrl_init, NULL,                                    \
			      &xec_i2c_nl_data_##inst, &xec_i2c_nl_cfg_##inst, POST_KERNEL,        \
			      CONFIG_I2C_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_CTRL_INIT)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT microchip_xec_i2c_v3_nl_port

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
		.parent = DEVICE_DT_GET(DT_INST_PHANDLE(inst, controller)),                        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.bitrate = DT_INST_PROP_OR(inst, clock_frequency, I2C_BITRATE_STANDARD),           \
		.port_id = (uint8_t)(DT_INST_PROP(inst, port) & 0x0FU),                            \
		.is_default = XEC_I2C_NL_PORT_IS_DEFAULT(inst),                                    \
	};                                                                                         \
	I2C_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_port_init, NULL, NULL,                          \
				  &xec_i2c_nl_port_cfg_##inst, POST_KERNEL,                        \
				  CONFIG_I2C_MCHP_XEC_V3_NL_PORT_INIT_PRIORITY,                    \
				  &xec_i2c_nl_port_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_PORT_INIT)
