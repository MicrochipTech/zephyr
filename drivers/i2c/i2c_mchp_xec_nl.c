/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip XEC version 3.8 I2C HW Network-Layer (NL) I2C driver.
 * The XEC I2C supports 7-bit I2C addressing only.
 * The HW host (controller) and target state machines run in parallel,
 * each with its own DMA channel. With CONFIG_I2C_TARGET_BUFFER_MODE up to
 * two targets can be registered on one port; the controller then stays on
 * that port and controller transfers on it run alongside target mode.
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
#include <zephyr/sys/util.h>

/* register defines */
#include "i2c_mchp_xec_regs.h"
#include "i2c_mchp_xec_nl_parser.h"

LOG_MODULE_REGISTER(i2c_mchp_xec_nl, CONFIG_I2C_LOG_LEVEL);

#define XEC_I2C_NL_SAVE_ELEN_HCMD

/* Sentinel freqhz value: use timing[XEC_I2C_NL_TIMING_DT] verbatim instead
 * of a bucketed 100k/400k/1M row (I2C_SPEED_DT / a "timing-dt" port).
 */
#define XEC_I2C_NL_FREQ_DT UINT32_MAX

/* active_port value until the controller is routed to a port */
#define XEC_I2C_NL_PORT_NONE 0xffU

/* Default value for I2C.Control register: Enable output, enable auto-ACK, clear service status */
#define XEC_I2C_NL_CR_DFLT \
	(BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS) | BIT(XEC_I2C_CR_PIN_POS))

/* Control register with the byte mode interrupt enabled, for arming a target that is
 * started by the byte mode service request rather than by the address match.
 */
#define XEC_I2C_NL_CR_ENI (XEC_I2C_NL_CR_DFLT | BIT(XEC_I2C_CR_ENI_POS))

/* Control register with the byte mode interrupt disabled and PIN left alone. PIN reads
 * as a service request, and writing it back as 1 clears the status and answers the
 * request, which would let byte mode carry on with a byte the network layer is being
 * given. Writing 0 keeps output enable and auto-ACK and touches nothing else.
 */
#define XEC_I2C_NL_CR_HOLD (BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS))

/* The same, with the byte mode interrupt enabled: arms it without clearing the status
 * or answering a service request already outstanding.
 */
#define XEC_I2C_NL_CR_ENI_HOLD (XEC_I2C_NL_CR_HOLD | BIT(XEC_I2C_CR_ENI_POS))

/* Enable bit-bang live SCL/SDA monitoring and bit-bang mode is disable */
#define XEC_I2C_BBCR_LIVE_RD BIT(XEC_I2C_BBCR_CM_POS)

/* Completion register status bits of a host transaction */
#define XEC_I2C_NL_CMPL_HOST_STS                                                                   \
	(BIT(XEC_I2C_CMPL_HDONE_POS) | BIT(XEC_I2C_CMPL_HNAKX_POS) |                               \
	 BIT(XEC_I2C_CMPL_LAB_STS_POS) | BIT(XEC_I2C_CMPL_BER_STS_POS) |                           \
	 BIT(XEC_I2C_CMPL_CHDH_STS_POS) | BIT(XEC_I2C_CMPL_CHDL_STS_POS) |                         \
	 BIT(XEC_I2C_CMPL_TCTO_STS_POS) | BIT(XEC_I2C_CMPL_HCTO_STS_POS) |                         \
	 BIT(XEC_I2C_CMPL_DTS_STS_POS))

/* Completion register status bits of a host transaction that no other state machine
 * reports into. The rest of XEC_I2C_NL_CMPL_HOST_STS describes the bus and is shared
 * with the target state machine.
 */
#define XEC_I2C_NL_CMPL_HOST_OWNED                                                                 \
	(BIT(XEC_I2C_CMPL_HDONE_POS) | BIT(XEC_I2C_CMPL_HNAKX_POS) |                               \
	 BIT(XEC_I2C_CMPL_IDLE_POS))

/* Completion register errors that end a host transaction and require a controller reset */
#define XEC_I2C_NL_CMPL_HOST_FATAL                                                                 \
	(BIT(XEC_I2C_CMPL_LAB_STS_POS) | BIT(XEC_I2C_CMPL_BER_STS_POS) |                           \
	 BIT(XEC_I2C_CMPL_TMO_STS_POS))

/* Either entry reports the start of a target transaction */
#if defined(CONFIG_I2C_MCHP_XEC_NL_TGT_AAT) || defined(CONFIG_I2C_MCHP_XEC_NL_TGT_HANDOFF_ON_PIN)
#define XEC_I2C_NL_TGT_START 1
#endif

/* Completion register status bits that say why a target transaction ended. Without one
 * of these an ending has no explanation, which is the stall the watchdog and the idle
 * interrupt exist to recover.
 */
#define XEC_I2C_NL_CMPL_TGT_ENDED                                                                  \
	(XEC_I2C_NL_CMPL_HOST_FATAL | BIT(XEC_I2C_CMPL_TNAKR_STS_POS) |                            \
	 BIT(XEC_I2C_CMPL_TPROT_POS))

/* Completion register status bits of a target transaction */
#define XEC_I2C_NL_CMPL_TGT_STS                                                                    \
	(BIT(XEC_I2C_CMPL_TDONE_POS) | BIT(XEC_I2C_CMPL_TNAKR_STS_POS) |                           \
	 BIT(XEC_I2C_CMPL_TPROT_POS) | BIT(XEC_I2C_CMPL_RPT_RD_POS) | BIT(XEC_I2C_CMPL_RPT_WR_POS))

/* Extended length register halves, written with 16-bit accesses so host and target
 * updates do not interfere. Host half: write count bits[15:8] in bits[7:0], read count
 * bits[15:8] in bits[15:8]. Target half: the same for the target counts.
 */
#define XEC_I2C_NL_ELEN_HOST_OFS XEC_I2C_ELEN_OFS
#define XEC_I2C_NL_ELEN_TGT_OFS  (XEC_I2C_ELEN_OFS + 2U)
#define XEC_I2C_NL_ELEN_TGT_WR_MSK GENMASK(7, 0)
#define XEC_I2C_NL_ELEN_TGT_RD_MSK GENMASK(15, 8)

/* Configuration register target transmit and receive buffer flush bits */
#define XEC_I2C_NL_CFG_FLUSH_TGT (BIT(XEC_I2C_CFG_FTTX_POS) | BIT(XEC_I2C_CFG_FTRX_POS))

/* Configuration register host transmit and receive buffer flush bits */
#define XEC_I2C_NL_CFG_FLUSH_HOST (BIT(XEC_I2C_CFG_FHTX_POS) | BIT(XEC_I2C_CFG_FHRX_POS))

/* Status register bits a new host transaction can not start with */
#define XEC_I2C_NL_SR_ERR (BIT(XEC_I2C_SR_BER_POS) | BIT(XEC_I2C_SR_LAB_POS))

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

/* BBM_EN=1, both dirs=input, both released */
#define BBCR_BB_RELEASED BIT(XEC_I2C_BBCR_EN_POS)
/* BBM_EN=1, SCL drive-low, SDA released */
#define BBCR_BB_SCL_LOW (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CD_POS))
/* BBM_EN=1, SDA drive-low, SCL released */
#define BBCR_BB_SDA_LOW (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_DD_POS))

#define SR_IDLE (BIT(XEC_I2C_SR_PIN_POS) | BIT(XEC_I2C_SR_NBB_POS))

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

/* The TX DMA block chain of a request is one DMA configuration */
#ifdef CONFIG_DMA_MCHP_XEC_MAX_BLOCKS_PER_CHAN
BUILD_ASSERT(CONFIG_DMA_MCHP_XEC_MAX_BLOCKS_PER_CHAN >= I2C_XFER_MAX_TX_SEGS,
	     "I2C-NL requires CONFIG_DMA_MCHP_XEC_MAX_BLOCKS_PER_CHAN >= 3");
#endif

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
	uint8_t mr0;
};

/* Controller device configuration */
struct xec_i2c_nl_config {
	uintptr_t regbase;
	const struct device *dma_dev;
	uint8_t dma_chan1;
	uint8_t dma_slot_cm;
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	bool has_tgt_dma;
	uint8_t dma_chan2;
	uint8_t dma_slot_tm;
	uint16_t tgt_buf_size;
	uint8_t *tgt_buf;
#endif
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t girq_wk;
	uint8_t girq_wk_pos;
	uint16_t enc_pcr;
	bool has_dt_timing;
	uint32_t dflt_freq;
	void (*irq_connect)(void);
	struct xec_i2c_timing timing[XEC_I2C_NL_TIMING_COUNT];
};

#ifdef CONFIG_I2C_MCHP_XEC_NL_ISR_CAPTURE
#define XEC_I2C_NL_NCAP_ENTRIES CONFIG_I2C_MCHP_XEC_NL_ISR_CAPTURE_NUM_ENTRIES

struct xec_i2c_nl_isr_capture {
	volatile uint32_t hcmd;
	volatile uint32_t tcmd;
	volatile uint32_t extlen;
	volatile uint32_t cmpl;
	volatile uint32_t cfg;
	volatile uint8_t sr;
	volatile uint8_t wksts;
	volatile uint8_t shad_addr;
	volatile uint8_t shad_data;
};
#endif

/* Controller device driver data */
struct xec_i2c_nl_data {
	const struct device *controller;
	struct k_sem lock; /* API lock: binary semaphore, can be released from ISR context */
	struct k_sem xfr_done; /* init count 0, limit 1 */

	/* Current request. Holds DMA source/destination bytes: must stay in place */
	struct i2c_xfer_desc desc;

	/* DMA configurations. Fixed fields are set at init; a request only updates
	 * addresses, sizes, and block counts so the ISR needs no memset.
	 */
	struct dma_config dma_tx;
	struct dma_config dma_rx;
	struct dma_block_config tx_blks[I2C_XFER_MAX_TX_SEGS];
	struct dma_block_config rx_blks[I2C_XFER_MAX_RX_SEGS];

	/* Request result set by the ISR */
	int xfr_err;
	uint32_t xfr_cmpl;
	bool xfr_reset;

#ifdef CONFIG_I2C_CALLBACK
	/* Asynchronous transfer: messages, current request, and completion callback */
	bool xfr_async;
	bool async_cb_pending;
	int async_result;
	uint16_t async_addr;
	uint8_t async_num_msgs;
	uint8_t async_idx; /* first message of the current request */
	uint8_t async_n;   /* messages in the current request */
	struct i2c_msg *async_msgs;
	const struct device *async_port_dev;
	i2c_callback_t async_cb;
	void *async_userdata;
	struct k_timer async_timer; /* per request time-out */
#endif

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	/* Target mode: targets in OWN_ADDRESS_1 and OWN_ADDRESS_2, listening on one port */
	struct i2c_target_config *tgt_cfg[XEC_I2C_OA_NUM_TARGETS];
	const struct device *tgt_port_dev;
	struct i2c_target_config *tgt_active; /* target addressed by the current transaction */
#ifdef XEC_I2C_NL_TGT_START
	/* Counts the starts of target transactions, so it names the one in progress */
	uint32_t tgt_seq;
#endif
#ifdef CONFIG_I2C_MCHP_XEC_NL_TGT_WDOG
	/* Target of a transaction the watchdog ended on a lost arbitration, whose error
	 * has not been reported yet. The hardware reports such a transaction done later,
	 * when a STOP from a later transaction sets it, and that report is what carries
	 * the reason to the application. NULL when nothing is waiting.
	 *
	 * tgt_stalled_seq is the transaction the held target is waiting for a report on,
	 * and a report is that transaction's only while it still equals tgt_seq.
	 */
	struct i2c_target_config *tgt_stalled;
	uint32_t tgt_stalled_seq;
#endif
	uint32_t tgt_rx_armed; /* receive count the target receive DMA was armed with */
	uint32_t tgt_rx_off;   /* first data byte: 1 after START address, 0 after RPT-START */
	struct dma_config dma_trx;
	struct dma_config dma_ttx;
	struct dma_block_config trx_blk;
	struct dma_block_config ttx_blk;
#endif

#ifdef CONFIG_I2C_MCHP_XEC_NL_TGT_WDOG
	/* Target watchdog: started by the address match when a transaction begins and
	 * stopped when the transaction is reported done, so expiry means neither
	 * happened.
	 */
	struct k_timer tgt_wdog;
#endif

	uint8_t active_port;
	uint32_t active_freq;

#ifdef CONFIG_I2C_MCHP_XEC_NL_STATE_CAPTURE
	volatile uint32_t cap_idx;
	volatile uint8_t capbuf[CONFIG_I2C_MCHP_XEC_NL_STATE_CAPTURE_SIZE];

	/* Event counters. The capture buffer is reset at the start of every host
	 * transfer, so it only ever holds the most recent one and a rare event is
	 * erased by the traffic that follows it. These are never reset, so they
	 * still read correctly at the end of a long run. All are written from the
	 * ISR only.
	 *
	 * cnt_tgt_done     target transactions completed, the denominator for the
	 *                  rest
	 * cnt_tgt_err      of those, the ones that reported an error reason
	 * cnt_tgt_stuck    target done seen with the state machine still running
	 *                  and proceeding, counted whether or not the recovery for
	 *                  it is enabled
	 * cnt_isr_unclaimed  interrupts the target half did not claim while the target
	 *                  was armed and the hardware had not reported it done, that is
	 *                  a target GIRQ with no target source behind it. Host
	 *                  interrupts are not counted: they reach the same place.
	 * cnt_tgt_done_rx, cnt_tgt_done_tx  completed transactions split by the phase
	 *                  they ended in, the denominator for a rate that is expected to
	 *                  follow one direction and not the other.
	 * cnt_tgt_aat      addressed-as-target interrupts, one per target transaction
	 *                  the hardware matched an address for.
	 * cnt_tgt_wdog     target transactions the watchdog ended, and of those
	 * cnt_tgt_wdog_reset  the ones whose status said the controller needed a reset
	 *                  rather than only arming the target again.
	 * cnt_tgt_done_late  of cnt_tgt_done, the reports that arrived for a transaction
	 *                  the watchdog had already ended
	 * cnt_tgt_hold_stale  held reports dropped because a later transaction was
	 *                  addressed before the one they were waiting for reported,
	 *                  which is the error the application was not told about
	 * cnt_tgt_aat_miss  transactions reported done with the address match still
	 *                  armed, so the match was never seen for them and they ran
	 *                  without a watchdog
	 * cnt_tgt_idle     transactions ended by the bus going idle with the hardware
	 *                  never reporting them done, which only the address match
	 *                  hand-off can see
	 *
	 * cnt_tgt_aat_miss is what makes the two sides below agree. An address match is
	 * reported at the end of the seventh clock of the address byte and the status
	 * clears when the network layer moves that byte out after the ninth, so the ISR
	 * has about two clock periods to see it and a transaction whose window it missed
	 * is never counted as started. Measured rather than inferred: the match enable
	 * is cleared when a match is claimed and set again only when the target is
	 * armed, so finding it still set at target done says this transaction's match
	 * went unseen.
	 *
	 * cnt_tgt_aat is the count to divide by, not cnt_tgt_done: every transaction
	 * the hardware matched an address for is one address match, but a stall the
	 * watchdog ends still reports done later, when a STOP from a later transaction
	 * sets it, and by then the target has been armed again and that done is counted
	 * as a transaction of its own. The three agree as
	 *
	 *   cnt_tgt_aat + cnt_tgt_aat_miss = (cnt_tgt_done - cnt_tgt_done_late)
	 *                                    + cnt_tgt_wdog
	 *
	 * which is worth checking at the end of a run: anything left over is a
	 * transaction ended by a path none of these counts.
	 */
	volatile uint32_t cnt_tgt_done;
	volatile uint32_t cnt_tgt_err;
	volatile uint32_t cnt_tgt_stuck;
	volatile uint32_t cnt_isr_unclaimed;
	volatile uint32_t cnt_tgt_wdog;
	volatile uint32_t cnt_tgt_wdog_reset;
	volatile uint32_t cnt_tgt_done_rx;
	volatile uint32_t cnt_tgt_done_tx;
	volatile uint32_t cnt_tgt_aat;
	volatile uint32_t cnt_tgt_done_late;
	volatile uint32_t cnt_tgt_hold_stale;
	volatile uint32_t cnt_tgt_aat_miss;
	volatile uint32_t cnt_tgt_idle;
#endif
#ifdef CONFIG_I2C_MCHP_XEC_NL_ISR_CAPTURE
	volatile uint32_t isr_cap_idx;
	struct xec_i2c_nl_isr_capture isrcap[XEC_I2C_NL_NCAP_ENTRIES];
#endif
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

#ifdef CONFIG_I2C_MCHP_XEC_NL_STATE_CAPTURE
static void xec_i2c_nl_state_cap_init(struct xec_i2c_nl_data *data)
{
	data->cap_idx = 0;
	memset((void *)data->capbuf, 0, CONFIG_I2C_MCHP_XEC_NL_STATE_CAPTURE_SIZE);
}

static void xec_i2c_nl_state_cap_update(struct xec_i2c_nl_data *data, uint8_t val)
{
	if (data->cap_idx >= CONFIG_I2C_MCHP_XEC_NL_STATE_CAPTURE_SIZE) {
		return;
	}

	data->capbuf[data->cap_idx++] = val;
}

#define XEC_I2C_NL_STATE_CAP_INIT(data) xec_i2c_nl_state_cap_init(data)
#define XEC_I2C_NL_STATE_CAP_UPDATE(data, val) xec_i2c_nl_state_cap_update(data, val)
/* Counter increments are not reset by XEC_I2C_NL_STATE_CAP_INIT() on purpose */
#define XEC_I2C_NL_CNT_INC(data, member) ((data)->member++)


#else
#define XEC_I2C_NL_STATE_CAP_INIT(data)
#define XEC_I2C_NL_STATE_CAP_UPDATE(data, val)
#define XEC_I2C_NL_CNT_INC(data, member)
#endif

#ifdef CONFIG_I2C_MCHP_XEC_NL_ISR_CAPTURE
static void xec_i2c_nl_isr_cap_init(struct xec_i2c_nl_data *data)
{
	data->isr_cap_idx = 0;
	memset(data->isrcap, 0, sizeof(data->isrcap));
}

#if 0
struct xec_i2c_nl_isr_capture {
	uint32_t hcmd;
	uint32_t tcmd;
	uint32_t extlen;
	uint32_t cmpl;
	uint32_t cfg;
	uint8_t sr;
	uint8_t wksts;
	uint8_t shad_addr;
	uint8_t shad_data;
};
#endif
static void xec_i2c_nl_isr_cap_update(struct xec_i2c_nl_data *data)
{
	const struct xec_i2c_nl_config *ctrl_cfg = data->controller->config;
	uintptr_t rb = ctrl_cfg->regbase;
	

	if (data->isr_cap_idx >= XEC_I2C_NL_NCAP_ENTRIES) {
		return;
	}

	struct xec_i2c_nl_isr_capture *cp = &data->isrcap[data->isr_cap_idx++];

	cp->hcmd = sys_read32(rb + XEC_I2C_HCMD_OFS);
	cp->tcmd = sys_read32(rb + XEC_I2C_TCMD_OFS);
	cp->extlen = sys_read32(rb + XEC_I2C_ELEN_OFS);
	cp->cmpl = sys_read32(rb + XEC_I2C_CMPL_OFS);
	cp->cfg = sys_read32(rb + XEC_I2C_CFG_OFS);
	cp->sr = sys_read8(rb + XEC_I2C_SR_OFS);
	cp->wksts = sys_read8(rb + XEC_I2C_WKSR_OFS);
	cp->shad_addr = sys_read8(rb + XEC_I2C_IAS_OFS);
	cp->shad_data = sys_read8(rb + XEC_I2C_IDS_OFS);
}

#define XEC_I2C_NL_ISR_CAP_INIT(data) xec_i2c_nl_isr_cap_init(data)
#define XEC_I2C_NL_ISR_CAP_UPDATE(data) xec_i2c_nl_isr_cap_update(data)
#else
#define XEC_I2C_NL_ISR_CAP_INIT(data)
#define XEC_I2C_NL_ISR_CAP_UPDATE(data)
#endif

/* XEC I2C controller supports 7-bit I2C addressing only */
static inline bool xec_i2c_is_valid_address(uint16_t i2c_address)
{
	if ((i2c_address & ~0x7fU) != 0U) {
		return false;
	}

	return true;
}

/* A target is registered: the controller must stay on the targets' port */
static inline bool xec_i2c_nl_tgt_registered(const struct xec_i2c_nl_data *ctrl_data)
{
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	return ctrl_data->tgt_port_dev != NULL;
#else
	ARG_UNUSED(ctrl_data);
	return false;
#endif
}

/* The internal host must not address one of the controller's own target addresses */
static inline bool xec_i2c_nl_is_tgt_addr(const struct xec_i2c_nl_data *ctrl_data, uint16_t addr)
{
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	for (size_t i = 0; i < ARRAY_SIZE(ctrl_data->tgt_cfg); i++) {
		if ((ctrl_data->tgt_cfg[i] != NULL) && (ctrl_data->tgt_cfg[i]->address == addr)) {
			return true;
		}
	}
#else
	ARG_UNUSED(ctrl_data);
	ARG_UNUSED(addr);
#endif
	return false;
}

/* Write-1-to-clear the named status bits in the Completion register while
 * preserving its read/write control bits[5:2]. The completion register mixes
 * RW1C status (IDLE, BER, ...) with RW enables (DTEN/HCEN/TCEN/BIDEN) in one
 * word, so a bare sys_write32 of a status constant would also write 0 into
 * those enables.
 *
 * Those four enables are left at their reset value of 0 and this driver never sets
 * them. Do not enable them: the time-out hardware behind them was built for the
 * original network layer, whose write and read counts were 8 bits, and it was not
 * reworked when the Extended Length register widened those counts to 16. Using it
 * means keeping Extended Length at 0, which caps a transfer at 254 bytes of data
 * once the address bytes are taken out of the write count, and caps a target
 * receive buffer at 255. This driver and its bindings are built for the 16-bit
 * counts, and the i2c_targ_mode sample already asks for a 259 byte target buffer.
 *
 * The consequence is that TMO_STS never asserts, so the TMO_STS branch of
 * xec_i2c_nl_tgt_done() and the TMO_STS bit in XEC_I2C_NL_CMPL_HOST_FATAL are
 * unreachable. They are kept because the status is defined and a future part may
 * fix the time-out hardware, not because they have been seen to happen. The
 * I2C_ERROR_TIMEOUT reason itself is reachable, from the target watchdog in
 * xec_i2c_nl_tgt_wdog_expiry(), which is software.
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

static const struct xec_i2c_timing *
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

/* Clear the run-time and wake GIRQ status. A GIRQ status bit latches again while its
 * I2C source is active, so the I2C status feeding it must be cleared first: the caller
 * clears the completion register status (HDONE/TDONE and their sources) and this
 * routine clears the wake status (START bit detected) before the GIRQs.
 */
static void xec_i2c_nl_clear_girqs(const struct xec_i2c_nl_config *ctrl_cfg)
{
	sys_write32(BIT(XEC_I2C_WKSR_SB_POS), ctrl_cfg->regbase + XEC_I2C_WKSR_OFS);
	soc_ecia_girq_status_clear(ctrl_cfg->girq, ctrl_cfg->girq_pos);
	soc_ecia_girq_status_clear(ctrl_cfg->girq_wk, ctrl_cfg->girq_wk_pos);
}

static void xec_i2c_prog_freq(const struct xec_i2c_nl_config *ctrl_cfg,
			      const struct xec_i2c_timing *timing)
{
	uintptr_t rb = ctrl_cfg->regbase;

	sys_write32(timing->bus_clock, rb + XEC_I2C_BCLK_OFS);
	sys_write32(timing->data_timing, rb + XEC_I2C_DT_OFS);
	sys_write32(timing->idle_scaling, rb + XEC_I2C_ISC_OFS);
	sys_write32(timing->timeout_scaling, rb + XEC_I2C_TMOUT_SC_OFS);
	soc_mmcr_mask_set8(rb + XEC_I2C_RSHT_OFS, timing->rpt_start_hold_tm, XEC_I2C_RSHT_MSK);
	soc_mmcr_mask_set8(rb + XEC_I2C_MR0_OFS, timing->mr0, XEC_I2C_MR0_TM_MSK);
}

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
static void xec_i2c_nl_tgt_arm(const struct xec_i2c_nl_config *ctrl_cfg,
			       struct xec_i2c_nl_data *ctrl_data);
#endif

#ifdef CONFIG_I2C_MCHP_XEC_NL_TGT_WDOG
static void xec_i2c_nl_tgt_end(const struct xec_i2c_nl_config *ctrl_cfg,
			       struct xec_i2c_nl_data *ctrl_data, int reason, bool reset);
#endif

/* Configure controller timings, frequency, and port.
 * Reset the controller using the XEC PCR peripheral reset.
 * This routine does not use any wait spin loops (k_busy_wait or others)
 * It should be safe to call from any asynchronous code path.
 * NOTE: This I2C controller when enabled always samples the SCL/SDA pins.
 * If the pins are not in the expected state the controller will set its
 * bus error and/or lost arbitration status bits. Expected pin states are:
 * This I2C controller in idle state: both SCL and SDA should be high
 * This I2C is controller driving pins: both SCL and SDA should be state it is driving.
 *   Note: If this I2C is not driving SCL the target can drive SCL to clock stretch.
 * This I2C is target: If this controller is clock stretching SCL should be low.
 * Sample window: Unknown.
 */
static int xec_i2c_nl_program_ctrl(const struct xec_i2c_nl_config *ctrl_cfg,
				   struct xec_i2c_nl_data *ctrl_data, uint32_t freq,
				   uint8_t port_id)
{
	uintptr_t rb = ctrl_cfg->regbase;
	uint32_t r = 0;
	int rc = 0;
	const struct xec_i2c_timing *timing = xec_i2c_nl_timing_for(ctrl_cfg, freq);

	/* Ensure a complete reset using PCR peripheral reset. Self-clearing reset enable */
	soc_xec_pcr_reset_en(ctrl_cfg->enc_pcr);
	sys_write8(BIT(XEC_I2C_CR_PIN_POS), rb + XEC_I2C_CR_OFS);
	/* clear latched GIRQ status */
	xec_i2c_nl_clear_girqs(ctrl_cfg);

	/* while disabled program port selection, filter, and bus timings */
	r = (XEC_I2C_CFG_PORT_SET(port_id) | BIT(XEC_I2C_CFG_FEN_POS) |
	     BIT(XEC_I2C_CFG_GC_DIS_POS));
	sys_write32(r, rb + XEC_I2C_CFG_OFS);

	xec_i2c_prog_freq(ctrl_cfg, timing);

	sys_write8(XEC_I2C_NL_CR_DFLT, rb + XEC_I2C_CR_OFS);
	/* Enable. Controller begins to sample pins. If pins are not both high the controller
	 * will detect this and set status (BER and/or LAB) after N clocks.
	 * We do not want to add delay in this function because it may be called in an
	 * asynchronous code path. It is the caller's responsibility to add delay after
	 * this routine returns if it can.
	 */
	sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);

	/* Enable live monitoring of SCL/SDA pins in bit-bang control register. This feature
	 * is in I2C-SMB controller version 3.8+ hardware. The bit is read-only in early HW.
	 * We must make sure bit-bang mode is disabled and the live monitor bit is set.
	 */
	sys_write8(XEC_I2C_BBCR_LIVE_RD, rb + XEC_I2C_BBCR_OFS);

	/* paranoid, clear latched status again */
	xec_i2c_nl_clear_girqs(ctrl_cfg);

	ctrl_data->active_freq = freq;
	ctrl_data->active_port = port_id;

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	/* The reset cleared the own addresses and the target state machine. Nothing to
	 * put back when no target is registered, and testing that here rather than
	 * leaving it to the arming keeps a controller that is only ever a host from
	 * recording target state it has none of: this runs at initialisation, so every
	 * such controller began its capture with an arming that did nothing.
	 */
	if (xec_i2c_nl_tgt_registered(ctrl_data)) {
		xec_i2c_nl_tgt_arm(ctrl_cfg, ctrl_data);
	}
#endif

	return rc;
}
/* Controller busy: host state machine running or bus not free (NBB is 1 when free) */
static bool xec_i2c_nl_is_busy(const struct xec_i2c_nl_config *ctrl_cfg)
{
	uintptr_t rb = ctrl_cfg->regbase;

	if ((sys_test_bit(rb + XEC_I2C_HCMD_OFS, XEC_I2C_HCMD_RUN_POS) != 0) ||
	    (sys_test_bit(rb + XEC_I2C_SR_OFS, XEC_I2C_SR_NBB_POS) == 0)) {
		return true;
	}

	return false;
}

/* Let the controller sample the pins after xec_i2c_nl_program_ctrl() enables it, so a
 * bad pin state shows up as BER/LAB before a transaction starts. Thread context only:
 * interrupt context paths skip the delay.
 */
static void xec_i2c_nl_port_settle(void)
{
	if (!k_is_in_isr()) {
		k_busy_wait(CONFIG_I2C_MCHP_XEC_NL_PORT_SETTLE_US);
	}
}

/* Select controller port and/or frequency
 * If port and frequency are what we want do nothing else
 * reconfigure the controller for new port and/or frequeny
 * Note controller requires reset on port/freq change
 */
static int xec_i2c_nl_apply_port(const struct xec_i2c_nl_port_config *port_cfg,
				 struct xec_i2c_nl_port_data *port_data,
				 const struct xec_i2c_nl_config *ctrl_cfg,
				 struct xec_i2c_nl_data *ctrl_data)
{
	uint32_t freq = xec_i2c_nl_port_freq(port_cfg, port_data);
	uint8_t port = port_cfg->port_id;
	int rc = 0;

	if ((ctrl_data->active_port == port) && (ctrl_data->active_freq == freq)) {
		return 0;
	}

	if (xec_i2c_nl_tgt_registered(ctrl_data)) {
		/* Registered targets listen on the active port: it can not change */
		if (port != ctrl_data->active_port) {
			return -EBUSY;
		}
		/* A frequency change resets the controller: only with the bus idle */
		if (xec_i2c_nl_is_busy(ctrl_cfg)) {
			return -EBUSY;
		}
	}

	rc = pinctrl_apply_state(port_cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("I2C-NL apply port pincfg error (%d)", rc);
		return rc;
	}

	rc = xec_i2c_nl_program_ctrl(ctrl_cfg, ctrl_data, freq, port);
	if (rc != 0) {
		return rc;
	}

	xec_i2c_nl_port_settle();

	return 0;
}

/* API: configuration
 * Controller mode with 7-bit addressing only. The bus frequency is stored per port and
 * applied now: the controller is switched to this port and/or reprogrammed for the new
 * frequency if either differs. Transfers on the port also apply its port and frequency.
 * I2C_SPEED_DT selects the controller's timing-dt row when it exists, else it clears
 * the runtime frequency so the port's DT clock-frequency is used again.
 */
static int xec_i2c_nl_vport_config(const struct device *port_dev, uint32_t i2c_config)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	struct xec_i2c_nl_port_data *port_data = port_dev->data;
	const struct xec_i2c_nl_config *ctrl_cfg = port_cfg->controller->config;
	struct xec_i2c_nl_data *ctrl_data = port_cfg->controller->data;
	uint32_t speed = I2C_SPEED_GET(i2c_config);
	uint32_t freq = 0;
	uint32_t prev_freq = 0;
	int rc = 0;

	if (((i2c_config & I2C_MODE_CONTROLLER) == 0U) ||
	    ((i2c_config & I2C_ADDR_10_BITS) != 0U)) {
		return -ENOTSUP;
	}

	if ((speed == I2C_SPEED_DT) && !ctrl_cfg->has_dt_timing) {
		freq = 0U;
	} else {
		freq = xec_i2c_nl_speed_to_freq(speed);
		if (freq == 0U) {
			return -ENOTSUP;
		}
	}

	k_sem_take(&ctrl_data->lock, K_FOREVER);

	if (xec_i2c_nl_is_busy(ctrl_cfg)) {
		rc = -EBUSY;
		goto unlock;
	}

	prev_freq = port_data->runtime_freq;
	port_data->runtime_freq = freq;

	rc = xec_i2c_nl_apply_port(port_cfg, port_data, ctrl_cfg, ctrl_data);
	if (rc != 0) {
		port_data->runtime_freq = prev_freq;
	}

unlock:
	k_sem_give(&ctrl_data->lock);

	return rc;
}

/* API: get configuration
 * Reports the port's frequency in effect: runtime, else DT clock-frequency, else the
 * controller default. I2C_SPEED_DT is reported while the timing-dt row is selected.
 */
static int xec_i2c_nl_vport_get_config(const struct device *port_dev, uint32_t *i2c_config)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	struct xec_i2c_nl_port_data *port_data = port_dev->data;
	struct xec_i2c_nl_data *ctrl_data = port_cfg->controller->data;
	uint32_t freq = 0;

	if (i2c_config == NULL) {
		return -EINVAL;
	}

	k_sem_take(&ctrl_data->lock, K_FOREVER);
	freq = xec_i2c_nl_port_freq(port_cfg, port_data);
	k_sem_give(&ctrl_data->lock);

	*i2c_config = I2C_MODE_CONTROLLER | I2C_SPEED_SET(xec_i2c_nl_freq_to_speed(freq));

	return 0;
}

/* API: bus recovery and helpers */

/* Drive XEC_I2C_NL_BB_RECOV_CLOCKS SCL pulses at ~100 kHz while leaving
 * SDA released. Caller must already have engaged bit-bang mode (BBCR
 * set to BBCR_BB_RELEASED).
 * Tuned for 100KHz I2C bus clock.
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
 * Tuned for 100KHz I2C bus clock.
 */
static void xec_i2c_nl_bb_stop(uintptr_t base)
{
	sys_write8(BBCR_BB_SDA_LOW, base + XEC_I2C_BBCR_OFS);
	k_busy_wait(XEC_I2C_NL_BB_HALF_PERIOD_US);
	sys_write8(BBCR_BB_RELEASED, base + XEC_I2C_BBCR_OFS);
	k_busy_wait(XEC_I2C_NL_BB_HALF_PERIOD_US);
}

static int xec_i2c_nl_bus_recover(const struct xec_i2c_nl_config *ctrl_cfg,
				  struct xec_i2c_nl_data *ctrl_data, uint32_t freq, uint8_t port)
{
	uintptr_t base = ctrl_cfg->regbase;
	uint8_t bbcr = 0;
	int rc = xec_i2c_nl_program_ctrl(ctrl_cfg, ctrl_data, freq, port);

	if (rc != 0) {
		return rc;
	}

	/* Let the controller sample the pins before checking for an idle bus */
	xec_i2c_nl_port_settle();

	if (sys_read8(base + XEC_I2C_SR_OFS) == SR_IDLE) {
		return 0;
	}

	/* Bit-bang mode muxes SCL/SDA away from the I2C logic: the bus clock
	 * programming does not affect recovery timing.
	 */
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
	sys_write8(XEC_I2C_BBCR_LIVE_RD, base + XEC_I2C_BBCR_OFS);

	/* Reset the controller, clearing BER/LAB latched during recovery, and let it
	 * sample the pins now connected to the I2C logic again.
	 */
	rc = xec_i2c_nl_program_ctrl(ctrl_cfg, ctrl_data, freq, port);
	if (rc != 0) {
		return rc;
	}
	xec_i2c_nl_port_settle();

	bbcr = sys_read8(base + XEC_I2C_BBCR_OFS);
	if ((bbcr & (BBCR_SCL_IN | BBCR_SDA_IN)) != (BBCR_SCL_IN | BBCR_SDA_IN)) {
		LOG_ERR("i2c-recover: SCL=%u SDA=%u still not both high",
			(bbcr & BBCR_SCL_IN) ? 1U : 0U, (bbcr & BBCR_SDA_IN) ? 1U : 0U);
		return -EIO;
	}

	return 0;
}

/* API: bus recovery of the bus on this port.
 * The controller is routed to this port first: its pins are applied when the
 * controller is on another port. xec_i2c_nl_bus_recover() resets and programs the
 * controller for this port and frequency, which it keeps afterwards.
 */
static int xec_i2c_nl_vport_recover_bus(const struct device *port_dev)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	struct xec_i2c_nl_port_data *port_data = port_dev->data;
	const struct xec_i2c_nl_config *ctrl_cfg = port_cfg->controller->config;
	struct xec_i2c_nl_data *ctrl_data = port_cfg->controller->data;
	uint32_t freq = 0;
	int rc = 0;

	k_sem_take(&ctrl_data->lock, K_FOREVER);

	freq = xec_i2c_nl_port_freq(port_cfg, port_data);

	if (ctrl_data->active_port != port_cfg->port_id) {
		/* Registered targets listen on the active port */
		if (xec_i2c_nl_tgt_registered(ctrl_data)) {
			rc = -EBUSY;
			goto unlock;
		}
		rc = pinctrl_apply_state(port_cfg->pincfg, PINCTRL_STATE_DEFAULT);
		if (rc != 0) {
			LOG_ERR("I2C-NL recover port pincfg error (%d)", rc);
			goto unlock;
		}
	}

	rc = xec_i2c_nl_bus_recover(ctrl_cfg, ctrl_data, freq, port_cfg->port_id);

unlock:
	k_sem_give(&ctrl_data->lock);

	return rc;
}

/* Load the TX or RX segments of the current request into the DMA block chain,
 * configure, and start the controller mode DMA channel. Callable from ISR context.
 */
static int xec_i2c_nl_dma_start(const struct xec_i2c_nl_config *ctrl_cfg,
				struct xec_i2c_nl_data *ctrl_data, bool rx)
{
	const struct i2c_xfer_desc *d = &ctrl_data->desc;
	const struct i2c_xfer_seg *segs = rx ? d->rx : d->tx;
	struct dma_block_config *blks = rx ? ctrl_data->rx_blks : ctrl_data->tx_blks;
	struct dma_config *dcfg = rx ? &ctrl_data->dma_rx : &ctrl_data->dma_tx;
	uint8_t nseg = rx ? d->rx_nseg : d->tx_nseg;
	int rc = 0;

	if (nseg == 0U) {
		return -EIO;
	}

	for (uint8_t i = 0; i < nseg; i++) {
		if (rx) {
			blks[i].dest_address = (uintptr_t)segs[i].buf;
		} else {
			blks[i].source_address = (uintptr_t)segs[i].buf;
		}
		blks[i].block_size = segs[i].len;
		blks[i].next_block = ((i + 1U) < nseg) ? &blks[i + 1U] : NULL;
	}
	dcfg->block_count = nseg;

	rc = dma_config(ctrl_cfg->dma_dev, ctrl_cfg->dma_chan1, dcfg);
	if (rc != 0) {
		return rc;
	}

	return dma_start(ctrl_cfg->dma_dev, ctrl_cfg->dma_chan1);
}

/* Set the fixed fields of the controller mode TX and RX DMA configurations */
static void xec_i2c_nl_dma_init(const struct xec_i2c_nl_config *ctrl_cfg,
				struct xec_i2c_nl_data *ctrl_data)
{
	struct dma_config *dcfg = &ctrl_data->dma_tx;

	dcfg->dma_slot = ctrl_cfg->dma_slot_cm;
	dcfg->channel_direction = MEMORY_TO_PERIPHERAL;
	dcfg->source_data_size = 1U;
	dcfg->dest_data_size = 1U;
	dcfg->head_block = ctrl_data->tx_blks;
	for (size_t i = 0; i < ARRAY_SIZE(ctrl_data->tx_blks); i++) {
		ctrl_data->tx_blks[i].dest_address = ctrl_cfg->regbase + XEC_I2C_HTX_OFS;
		ctrl_data->tx_blks[i].source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		ctrl_data->tx_blks[i].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	dcfg = &ctrl_data->dma_rx;
	dcfg->dma_slot = ctrl_cfg->dma_slot_cm;
	dcfg->channel_direction = PERIPHERAL_TO_MEMORY;
	dcfg->source_data_size = 1U;
	dcfg->dest_data_size = 1U;
	dcfg->head_block = ctrl_data->rx_blks;
	for (size_t i = 0; i < ARRAY_SIZE(ctrl_data->rx_blks); i++) {
		ctrl_data->rx_blks[i].source_address = ctrl_cfg->regbase + XEC_I2C_HRX_OFS;
		ctrl_data->rx_blks[i].source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		ctrl_data->rx_blks[i].dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	/* Target mode DMA channel: receive into the driver buffer, transmit from the
	 * application's buffer.
	 */
	dcfg = &ctrl_data->dma_trx;
	dcfg->dma_slot = ctrl_cfg->dma_slot_tm;
	dcfg->channel_direction = PERIPHERAL_TO_MEMORY;
	dcfg->source_data_size = 1U;
	dcfg->dest_data_size = 1U;
	dcfg->block_count = 1U;
	dcfg->head_block = &ctrl_data->trx_blk;
	ctrl_data->trx_blk.source_address = ctrl_cfg->regbase + XEC_I2C_TRX_OFS;
	ctrl_data->trx_blk.dest_address = (uintptr_t)ctrl_cfg->tgt_buf;
	ctrl_data->trx_blk.block_size = ctrl_cfg->tgt_buf_size;
	ctrl_data->trx_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	ctrl_data->trx_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;

	dcfg = &ctrl_data->dma_ttx;
	dcfg->dma_slot = ctrl_cfg->dma_slot_tm;
	dcfg->channel_direction = MEMORY_TO_PERIPHERAL;
	dcfg->source_data_size = 1U;
	dcfg->dest_data_size = 1U;
	dcfg->block_count = 1U;
	dcfg->head_block = &ctrl_data->ttx_blk;
	ctrl_data->ttx_blk.dest_address = ctrl_cfg->regbase + XEC_I2C_TTX_OFS;
	ctrl_data->ttx_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	ctrl_data->ttx_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
#endif
}

/* Stop the controller and DMA, then reset and reprogram the controller for its
 * active port and frequency. Used after a timeout or a transaction error.
 */
static void xec_i2c_nl_reset(const struct xec_i2c_nl_config *ctrl_cfg,
			     struct xec_i2c_nl_data *ctrl_data)
{
	uintptr_t rb = ctrl_cfg->regbase;
	unsigned int key = irq_lock();

	sys_clear_bits(rb + XEC_I2C_CFG_OFS,
		       BIT(XEC_I2C_CFG_HD_IEN_POS) | BIT(XEC_I2C_CFG_IDLE_IEN_POS));
	irq_unlock(key);

	(void)dma_stop(ctrl_cfg->dma_dev, ctrl_cfg->dma_chan1);
	(void)xec_i2c_nl_program_ctrl(ctrl_cfg, ctrl_data, ctrl_data->active_freq,
				      ctrl_data->active_port);
	xec_i2c_nl_port_settle();
	k_sem_reset(&ctrl_data->xfr_done);
}

/* Clear status and empty the host buffers. Must be done before the TX DMA channel
 * is started: the controller requests TX data as soon as the host transmit buffer
 * is empty. Interrupts are locked: the target ISR also updates the configuration
 * register.
 */
static void xec_i2c_nl_prep_hw(const struct xec_i2c_nl_config *ctrl_cfg,
			       struct xec_i2c_nl_data *ctrl_data)
{
	uintptr_t rb = ctrl_cfg->regbase;
	uint32_t bits = XEC_I2C_NL_CMPL_HOST_OWNED;
	unsigned int key = irq_lock();

	/* LAB, BER and the time-out sources behind TMO_STS report the bus, not one state
	 * machine, and xec_i2c_nl_tgt_done() reads them. This runs in thread context, so
	 * clearing them here would discard the status of a target transaction already in
	 * flight on the port the targets share with the host. Clear them only when no
	 * target is registered; otherwise rely on the host ISR clearing the status it
	 * observed in its own snapshot.
	 */
	if (!IS_ENABLED(CONFIG_I2C_MCHP_XEC_NL_FIX_HOST_CMPL_MASK) ||
	    !xec_i2c_nl_tgt_registered(ctrl_data)) {
		bits |= XEC_I2C_NL_CMPL_HOST_STS;
	}

	xec_i2c_v3_cmpl_clear(rb, bits);
	sys_set_bits(rb + XEC_I2C_CFG_OFS, XEC_I2C_NL_CFG_FLUSH_HOST);
	xec_i2c_nl_clear_girqs(ctrl_cfg);
	irq_unlock(key);
}

/* Program the transfer counts and start the host state machine. Interrupts are
 * locked for the configuration register update: the target ISR also updates it.
 */
static void xec_i2c_nl_start_hw(const struct xec_i2c_nl_config *ctrl_cfg,
				struct xec_i2c_nl_data *ctrl_data)
{
	uintptr_t rb = ctrl_cfg->regbase;
	struct i2c_xfer_desc *d = &ctrl_data->desc;
	unsigned int key = 0;
	uint32_t hcmd = 0;

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x12U);

	/* Counts are 16-bit: bits[7:0] in HCMD, bits[15:8] in the extended length register
	 * host half.
	 */
	d->elen = XEC_I2C_ELEN_HWR_SET(d->tx_count >> 8) | XEC_I2C_ELEN_HRD_SET(d->rx_count >> 8);
	sys_write16((uint16_t)d->elen, rb + XEC_I2C_NL_ELEN_HOST_OFS);

	hcmd = XEC_I2C_HCMD_WCL_SET(d->tx_count & 0xffU);
	hcmd |= XEC_I2C_HCMD_RCL_SET(d->rx_count & 0xffU);
	hcmd |= d->ctrl | BIT(XEC_I2C_HCMD_PROC_POS) | BIT(XEC_I2C_HCMD_RUN_POS);

	key = irq_lock();
	sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_HD_IEN_POS);
	irq_unlock(key);

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x13U);

#ifdef CONFIG_I2C_MCHP_XEC_NL_TGT_HANDOFF_ON_PIN
	/* An armed target has the byte mode interrupt enabled, and it must not be while
	 * the host command register runs: it interrupts per byte, which is the whole
	 * performance of a network layer transfer. Disabling it does not make the
	 * controller deaf, the hardware still matching an address after a START or
	 * RPT-START and holding SCL to claim the transfer, so an external controller
	 * addressing a target now waits for this transfer rather than losing its
	 * transaction. The target is armed again when this one finishes.
	 */
	sys_write8(XEC_I2C_NL_CR_HOLD, rb + XEC_I2C_CR_OFS);
#endif
	sys_write32(hcmd, rb + XEC_I2C_HCMD_OFS);
}

/* Start the request in ctrl_data->desc. Callable from ISR context. */
static int xec_i2c_nl_req_start(const struct xec_i2c_nl_config *ctrl_cfg,
				struct xec_i2c_nl_data *ctrl_data)
{
	int rc = 0;

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x10U);

	k_sem_reset(&ctrl_data->xfr_done);
	ctrl_data->xfr_err = 0;
	ctrl_data->xfr_cmpl = 0U;
	ctrl_data->xfr_reset = false;

	xec_i2c_nl_prep_hw(ctrl_cfg, ctrl_data);

	rc = xec_i2c_nl_dma_start(ctrl_cfg, ctrl_data, false);
	if (rc != 0) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x11U);
		LOG_ERR("I2C-NL TX DMA start error (%d)", rc);
		return rc;
	}

	xec_i2c_nl_start_hw(ctrl_cfg, ctrl_data);

	return 0;
}

/* Execute the request in ctrl_data->desc and wait for it to finish */
static int xec_i2c_nl_xfr_one(const struct xec_i2c_nl_config *ctrl_cfg,
			      struct xec_i2c_nl_data *ctrl_data)
{
	int rc = xec_i2c_nl_req_start(ctrl_cfg, ctrl_data);

	if (rc != 0) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x14U);
		return rc;
	}

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x20U);

	rc = k_sem_take(&ctrl_data->xfr_done, I2C_TRANSFER_TIMEOUT);
	if (rc != 0) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x21U);
		LOG_ERR("I2C-NL transfer timeout (%d): addr 0x%02x", ctrl_data->xfr_err,
			ctrl_data->desc.addr);
		xec_i2c_nl_reset(ctrl_cfg, ctrl_data);
		/* An error seen before the time-out, e.g. NAK then no bus idle, is the cause */
		return (ctrl_data->xfr_err != 0) ? ctrl_data->xfr_err : -ETIMEDOUT;
	}

	if (ctrl_data->xfr_reset) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x22U);
		LOG_ERR("I2C-NL transfer error (%d): addr 0x%02x cmpl 0x%08x",
			ctrl_data->xfr_err, ctrl_data->desc.addr, ctrl_data->xfr_cmpl);
		xec_i2c_nl_reset(ctrl_cfg, ctrl_data);
	}

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x23U);
	return ctrl_data->xfr_err;
}

/* Number of messages in the request starting at msgs[0]: up to and including the
 * first message with I2C_MSG_STOP, else all remaining messages.
 */
static uint8_t xec_i2c_nl_req_len(const struct i2c_msg *msgs, uint8_t num_msgs)
{
	for (uint8_t i = 0; i < num_msgs; i++) {
		if ((msgs[i].flags & I2C_MSG_STOP) != 0U) {
			return i + 1U;
		}
	}

	return num_msgs;
}

/* Check every request of a transfer before the bus is used */
static int xec_i2c_nl_validate(const struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr)
{
	struct i2c_xfer_desc desc;
	uint8_t n = 0;
	int rc = 0;

	if (!xec_i2c_is_valid_address(addr) || (msgs == NULL)) {
		return -EINVAL;
	}

	for (uint8_t idx = 0; idx < num_msgs; idx += n) {
		n = xec_i2c_nl_req_len(&msgs[idx], num_msgs - idx);
		rc = xec_i2c_nl_xfer_parse(&msgs[idx], n, addr, &desc);
		if (rc != 0) {
			return rc;
		}
	}

	return 0;
}

/* API: synchronous transfer
 * The messages are split into requests at each I2C_MSG_STOP. Each request is one
 * START to STOP transaction executed by the host state machine. All requests are
 * validated before the bus is used. Execution stops at the first failed request.
 */
static int xec_i2c_nl_vport_xfr(const struct device *port_dev, struct i2c_msg *msgs,
				uint8_t num_msgs, uint16_t i2c_address)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	struct xec_i2c_nl_port_data *port_data = port_dev->data;
	const struct xec_i2c_nl_config *ctrl_cfg = port_cfg->controller->config;
	struct xec_i2c_nl_data *ctrl_data = port_cfg->controller->data;
	uint8_t idx = 0;
	uint8_t n = 0;
	int rc = 0;

	if (num_msgs == 0U) {
		return 0;
	}

	rc = xec_i2c_nl_validate(msgs, num_msgs, i2c_address);
	if (rc != 0) {
		return rc;
	}

	if (xec_i2c_nl_is_tgt_addr(ctrl_data, i2c_address)) {
		return -EINVAL;
	}

	k_sem_take(&ctrl_data->lock, K_FOREVER);

	if (IS_ENABLED(CONFIG_I2C_MCHP_XEC_NL_STATE_CAPTURE_INIT_ON_XFR)) {
		XEC_I2C_NL_STATE_CAP_INIT(ctrl_data);
	}
	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 1U);

	XEC_I2C_NL_ISR_CAP_INIT(ctrl_data);

	rc = xec_i2c_nl_apply_port(port_cfg, port_data, ctrl_cfg, ctrl_data);
	if (rc != 0) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 2U);
		goto unlock;
	}

	/* Bus error or lost arbitration latched in the controller core */
	if ((sys_read8(ctrl_cfg->regbase + XEC_I2C_SR_OFS) & XEC_I2C_NL_SR_ERR) != 0U) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 3U);
		xec_i2c_nl_reset(ctrl_cfg, ctrl_data);
	}

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 4U);
	for (idx = 0; idx < num_msgs; idx += n) {
		n = xec_i2c_nl_req_len(&msgs[idx], num_msgs - idx);
		rc = xec_i2c_nl_xfer_parse(&msgs[idx], n, i2c_address, &ctrl_data->desc);
		if (rc != 0) {
			XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 5U);
			break;
		}

		rc = xec_i2c_nl_xfr_one(ctrl_cfg, ctrl_data);
		if (rc != 0) {
			XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 6U);
			break;
		}
	}

unlock:
	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 7U);
	k_sem_give(&ctrl_data->lock);

	return rc;
}

#ifdef CONFIG_I2C_CALLBACK
/* End an asynchronous transfer. The callback is invoked by
 * xec_i2c_nl_async_notify() once interrupts are unlocked.
 */
static void xec_i2c_nl_async_end(struct xec_i2c_nl_data *ctrl_data, int result)
{
	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xA0U);
	(void)k_timer_stop(&ctrl_data->async_timer);
	ctrl_data->xfr_async = false;
	ctrl_data->async_result = result;
	ctrl_data->async_cb_pending = true;
}

/* Start the asynchronous request at async_idx and its time-out */
static int xec_i2c_nl_async_start_req(const struct xec_i2c_nl_config *ctrl_cfg,
				      struct xec_i2c_nl_data *ctrl_data)
{
	struct i2c_msg *msgs = &ctrl_data->async_msgs[ctrl_data->async_idx];
	uint8_t left = ctrl_data->async_num_msgs - ctrl_data->async_idx;
	int rc = 0;

	ctrl_data->async_n = xec_i2c_nl_req_len(msgs, left);
	rc = xec_i2c_nl_xfer_parse(msgs, ctrl_data->async_n, ctrl_data->async_addr,
				   &ctrl_data->desc);
	if (rc != 0) {
		return rc;
	}

	if (!K_TIMEOUT_EQ(I2C_TRANSFER_TIMEOUT, K_FOREVER)) {
		k_timer_start(&ctrl_data->async_timer, I2C_TRANSFER_TIMEOUT, K_NO_WAIT);
	}

	return xec_i2c_nl_req_start(ctrl_cfg, ctrl_data);
}

/* Invoke the completion callback of an ended asynchronous transfer. The API lock is
 * released first so the callback can start another transfer.
 */
static void xec_i2c_nl_async_notify(struct xec_i2c_nl_data *ctrl_data)
{
	unsigned int key = irq_lock();
	bool pending = ctrl_data->async_cb_pending;
	i2c_callback_t cb = ctrl_data->async_cb;
	const struct device *port_dev = ctrl_data->async_port_dev;
	void *userdata = ctrl_data->async_userdata;
	int result = ctrl_data->async_result;

	ctrl_data->async_cb_pending = false;
	irq_unlock(key);

	if (!pending) {
		return;
	}

	k_sem_give(&ctrl_data->lock);

	if (cb != NULL) {
		cb(port_dev, result, userdata);
	}
}

/* Asynchronous request time-out. Runs with interrupts locked so it can not interleave
 * with the controller ISR. A timer restarted for the next request while this handler
 * waited has remaining time and is ignored.
 */
static void xec_i2c_nl_async_timeout(struct k_timer *timer)
{
	struct xec_i2c_nl_data *ctrl_data = CONTAINER_OF(timer, struct xec_i2c_nl_data,
							 async_timer);
	const struct xec_i2c_nl_config *ctrl_cfg = ctrl_data->controller->config;
	unsigned int key = irq_lock();

	if (ctrl_data->xfr_async && (k_timer_remaining_ticks(timer) == 0)) {
		LOG_ERR("I2C-NL async transfer timeout (%d): addr 0x%02x", ctrl_data->xfr_err,
			ctrl_data->desc.addr);
		xec_i2c_nl_reset(ctrl_cfg, ctrl_data);
		xec_i2c_nl_async_end(ctrl_data, (ctrl_data->xfr_err != 0) ? ctrl_data->xfr_err
									  : -ETIMEDOUT);
	}

	irq_unlock(key);

	xec_i2c_nl_async_notify(ctrl_data);
}

/* An asynchronous request ended (ISR context): reset the controller if needed, then
 * start the next request or end the transfer.
 */
static void xec_i2c_nl_async_req_done(const struct xec_i2c_nl_config *ctrl_cfg,
				      struct xec_i2c_nl_data *ctrl_data)
{
	int rc = ctrl_data->xfr_err;

	if (ctrl_data->xfr_reset) {
		LOG_ERR("I2C-NL transfer error (%d): addr 0x%02x cmpl 0x%08x", rc,
			ctrl_data->desc.addr, ctrl_data->xfr_cmpl);
		xec_i2c_nl_reset(ctrl_cfg, ctrl_data);
	}

	if (rc == 0) {
		ctrl_data->async_idx += ctrl_data->async_n;
		if (ctrl_data->async_idx < ctrl_data->async_num_msgs) {
			rc = xec_i2c_nl_async_start_req(ctrl_cfg, ctrl_data);
			if (rc == 0) {
				return;
			}
		}
	}

	xec_i2c_nl_async_end(ctrl_data, rc);
}

/* API: asynchronous transfer. Callable from ISR context.
 * Returns -EWOULDBLOCK when a transfer is in progress on the controller. Requests are
 * started from the controller ISR; cb is invoked from ISR context with the result of
 * the first failed request, or 0. msgs must stay valid until cb is invoked.
 */
static int xec_i2c_nl_vport_xfr_cb(const struct device *port_dev, struct i2c_msg *msgs,
				   uint8_t num_msgs, uint16_t i2c_address, i2c_callback_t cb,
				   void *userdata)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	struct xec_i2c_nl_port_data *port_data = port_dev->data;
	const struct xec_i2c_nl_config *ctrl_cfg = port_cfg->controller->config;
	struct xec_i2c_nl_data *ctrl_data = port_cfg->controller->data;
	int rc = 0;

	if (num_msgs == 0U) {
		if (cb != NULL) {
			cb(port_dev, 0, userdata);
		}
		return 0;
	}

	rc = xec_i2c_nl_validate(msgs, num_msgs, i2c_address);
	if (rc != 0) {
		return rc;
	}

	if (xec_i2c_nl_is_tgt_addr(ctrl_data, i2c_address)) {
		return -EINVAL;
	}

	if (k_sem_take(&ctrl_data->lock, K_NO_WAIT) != 0) {
		return -EWOULDBLOCK;
	}

	if (IS_ENABLED(CONFIG_I2C_MCHP_XEC_NL_STATE_CAPTURE_INIT_ON_XFR)) {
		XEC_I2C_NL_STATE_CAP_INIT(ctrl_data);
	}
	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 8U);

	rc = xec_i2c_nl_apply_port(port_cfg, port_data, ctrl_cfg, ctrl_data);
	if (rc != 0) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 9U);
		goto unlock;
	}

	/* Bus error or lost arbitration latched in the controller core */
	if ((sys_read8(ctrl_cfg->regbase + XEC_I2C_SR_OFS) & XEC_I2C_NL_SR_ERR) != 0U) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x0AU);
		xec_i2c_nl_reset(ctrl_cfg, ctrl_data);
	}

	ctrl_data->async_port_dev = port_dev;
	ctrl_data->async_msgs = msgs;
	ctrl_data->async_num_msgs = num_msgs;
	ctrl_data->async_addr = i2c_address;
	ctrl_data->async_idx = 0U;
	ctrl_data->async_cb = cb;
	ctrl_data->async_userdata = userdata;
	ctrl_data->async_cb_pending = false;
	ctrl_data->xfr_async = true;

	rc = xec_i2c_nl_async_start_req(ctrl_cfg, ctrl_data);
	if (rc == 0) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x0BU);
		return 0;
	}

	ctrl_data->xfr_async = false;
	(void)k_timer_stop(&ctrl_data->async_timer);

unlock:
	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x0CU);
	k_sem_give(&ctrl_data->lock);

	return rc;
}
#endif /* CONFIG_I2C_CALLBACK */

/* A request ended (ISR context). Synchronous: wake the waiting thread, which resets
 * the controller if needed.
 */
static void xec_i2c_nl_req_done(const struct xec_i2c_nl_config *ctrl_cfg,
				struct xec_i2c_nl_data *ctrl_data)
{
#ifdef CONFIG_I2C_CALLBACK
	if (ctrl_data->xfr_async) {
		xec_i2c_nl_async_req_done(ctrl_cfg, ctrl_data);
		return;
	}
#endif
	k_sem_give(&ctrl_data->xfr_done);
}

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
/* Target mode
 * The target state machine receives into the driver buffer by DMA. The START address
 * is stored at offset 0 and a RPT-START address after the data. The state machine
 * pauses (TDONE with TCMD.RUN=1, TCMD.PROC=0) on a read request and on a RPT-START write;
 * it stops (TDONE with TCMD.RUN=0) when a transaction ends. When the external controller
 * reads fewer bytes than supplied, only the STOP detect interrupt ends the transaction.
 * Receive overflow is NACKed by hardware: the data is dropped and reported as
 * I2C_ERROR_SIZE.
 *
 * That overflow does not set target done. Four runs of a 15 byte external write to a
 * target with an 11 byte buffer all ended with the completion register reading
 * 0x20010000, the overflow NACK and bus idle with no done and the target command
 * register still running and proceeding, after the hardware had ACKed the address and
 * 11 data bytes and NACKed the 12th. So nothing on the done path can report it: the
 * idle interrupt does, where it is enabled, and the watchdog does 50 ms later where it
 * is not.
 */

/* Bytes received by the target since its receive DMA was started */
static uint32_t xec_i2c_nl_tgt_rx_count(const struct xec_i2c_nl_config *ctrl_cfg,
					struct xec_i2c_nl_data *ctrl_data)
{
	uintptr_t rb = ctrl_cfg->regbase;
	uint32_t remaining = XEC_I2C_TCMD_RCL_GET(sys_read32(rb + XEC_I2C_TCMD_OFS));

	remaining |= XEC_I2C_ELEN_TRD_GET(sys_read32(rb + XEC_I2C_ELEN_OFS)) << 8;
	if (remaining > ctrl_data->tgt_rx_armed) {
		return 0U;
	}

	return ctrl_data->tgt_rx_armed - remaining;
}

/* Start the target receive DMA into the whole buffer and set the receive count
 * bits[15:8]. The caller writes bits[7:0] to TCMD.
 */
static int xec_i2c_nl_tgt_rx_start(const struct xec_i2c_nl_config *ctrl_cfg,
				   struct xec_i2c_nl_data *ctrl_data)
{
	uintptr_t rb = ctrl_cfg->regbase;
	uint16_t elen = 0;
	int rc = dma_config(ctrl_cfg->dma_dev, ctrl_cfg->dma_chan2, &ctrl_data->dma_trx);

	if (rc != 0) {
		return rc;
	}

	rc = dma_start(ctrl_cfg->dma_dev, ctrl_cfg->dma_chan2);
	if (rc != 0) {
		return rc;
	}

	elen = sys_read16(rb + XEC_I2C_NL_ELEN_TGT_OFS) & ~XEC_I2C_NL_ELEN_TGT_RD_MSK;
	elen |= FIELD_PREP(XEC_I2C_NL_ELEN_TGT_RD_MSK, ctrl_cfg->tgt_buf_size >> 8);
	sys_write16(elen, rb + XEC_I2C_NL_ELEN_TGT_OFS);
	ctrl_data->tgt_rx_armed = ctrl_cfg->tgt_buf_size;

	return 0;
}

/* Hand a target transaction to the network layer: arm the receive DMA for the whole
 * buffer, enable the done interrupt, and start the target command register.
 *
 * Called from arming, which runs this before the transaction starts, or from the
 * address match in the ISR once it has, depending on
 * CONFIG_I2C_MCHP_XEC_NL_TGT_AAT_HANDOFF.
 */
static int xec_i2c_nl_tgt_nl_start(const struct xec_i2c_nl_config *ctrl_cfg,
				   struct xec_i2c_nl_data *ctrl_data)
{
	uintptr_t rb = ctrl_cfg->regbase;
	int rc = xec_i2c_nl_tgt_rx_start(ctrl_cfg, ctrl_data);

	if (rc != 0) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x34U);
		LOG_ERR("I2C-NL target RX DMA start error (%d)", rc);
		return rc;
	}

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x35U);
	sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_TD_IEN_POS);
	sys_write32(XEC_I2C_TCMD_RCL_SET(ctrl_cfg->tgt_buf_size & 0xffU) |
		    BIT(XEC_I2C_TCMD_PROC_POS) | BIT(XEC_I2C_TCMD_RUN_POS),
		    rb + XEC_I2C_TCMD_OFS);

	return 0;
}

/* Program the own addresses and arm the target state machine for the next
 * transaction. Does nothing when no target is registered. Callable from ISR context;
 * runs with interrupts locked as thread context callers race the target ISR.
 *
 * Ends by clearing the GIRQs. Every caller in the ISR has already cleared them before
 * getting here, but these writes latch them again: clearing the status, writing the
 * Control register and rewriting the command register all do. A source that is
 * genuinely active re-asserts, so nothing real is dropped.
 *
 * It does not stop the interrupt being taken once more after a transaction, with the
 * completion register reading 0 and nothing claiming it. Three runs with this in place
 * all ended that way, and so did the same transaction with no hand-off at all, so
 * whatever latches it is downstream of the GIRQ. It costs one ISR entry and the ISR
 * handles it, so it is left alone.
 *
 * cnt_isr_unclaimed counts it only when the target is armed with its done interrupt
 * enabled, which is the arrangement without the hand-off. With the hand-off the done
 * interrupt is disabled between transactions, so the same entry is not counted: 0
 * there and 1 without it is the counter's reach, not a difference in the hardware.
 */
static void xec_i2c_nl_tgt_arm_locked(const struct xec_i2c_nl_config *ctrl_cfg,
				      struct xec_i2c_nl_data *ctrl_data)
{
	uintptr_t rb = ctrl_cfg->regbase;
	uint32_t oa = 0;

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x32U);

	for (size_t i = 0; i < ARRAY_SIZE(ctrl_data->tgt_cfg); i++) {
		if (ctrl_data->tgt_cfg[i] != NULL) {
			oa |= XEC_I2C_OA_SET(i, ctrl_data->tgt_cfg[i]->address);
		}
	}
	sys_write32(oa, rb + XEC_I2C_OA_OFS);

	(void)dma_stop(ctrl_cfg->dma_dev, ctrl_cfg->dma_chan2);
	sys_set_bits(rb + XEC_I2C_CFG_OFS, XEC_I2C_NL_CFG_FLUSH_TGT);
	xec_i2c_v3_cmpl_clear(rb, XEC_I2C_NL_CMPL_TGT_STS);

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x33U);

	ctrl_data->tgt_active = NULL;

	/* Where the data starts in the receive buffer. The network layer puts the
	 * address byte it matched at offset 0 and takes a receive count for it, so the
	 * data follows it. With the hand-off, byte mode matched the address and the
	 * network layer never saw it: an 11 byte buffer ACKed 11 data bytes and NACKed
	 * the 12th, where counting the address would have stopped at 10, and the buffer
	 * held the data from offset 0.
	 */
	ctrl_data->tgt_rx_off = IS_ENABLED(CONFIG_I2C_MCHP_XEC_NL_TGT_AAT_HANDOFF) ? 0U : 1U;

#ifdef CONFIG_I2C_MCHP_XEC_NL_TGT_WDOG
	k_timer_stop(&ctrl_data->tgt_wdog);
	ctrl_data->tgt_stalled = NULL;
#endif
#ifdef XEC_I2C_NL_TGT_START
	/* Arm whichever interrupt reports the start of the next transaction. Arming runs
	 * from the target register API and again after every target transaction, so it is
	 * armed once per transaction.
	 *
	 * The address match status needs no clearing. The network layer moves the address
	 * byte out of the Data register as part of running the transaction, and that
	 * clears it, so it is not still set from the transaction just ending.
	 *
	 * Clear the Status register so a later read reports the state then and not the
	 * history of earlier transactions: its error bits latch, so without this the first
	 * bus error or arbitration loss would make every later check look unhealthy.
	 * Nothing is lost, the target error reasons come from the Completion register.
	 *
	 * Write the value programming uses, output enable and auto-ACK together with PIN,
	 * not PIN alone: the Control register is write only, so a write of PIN alone would
	 * clear output enable and auto-ACK with it and leave the target deaf. The byte
	 * mode entry writes the same thing plus its enable, so the one write both clears
	 * the status and arms.
	 */
	if (IS_ENABLED(CONFIG_I2C_MCHP_XEC_NL_TGT_HANDOFF_ON_PIN)) {
		/* Unless a transaction is already waiting on us. The hardware matches an
		 * address and holds SCL whether or not the interrupt is enabled, so one
		 * can be outstanding here, left over the length of a host transfer that
		 * had to disable it. Clearing the status would answer that service
		 * request and let byte mode take a byte that is the network layer's, and
		 * would drop the match that says who it is for, so arm without touching
		 * either and let it interrupt.
		 */
		uint8_t sr = sys_read8(rb + XEC_I2C_SR_OFS);

		if (((sr & BIT(XEC_I2C_SR_AAT_POS)) != 0U) &&
		    ((sr & BIT(XEC_I2C_SR_PIN_POS)) == 0U)) {
			sys_write8(XEC_I2C_NL_CR_ENI_HOLD, rb + XEC_I2C_CR_OFS);
		} else {
			sys_write8(XEC_I2C_NL_CR_ENI, rb + XEC_I2C_CR_OFS);
		}
	} else {
		sys_write8(XEC_I2C_NL_CR_DFLT, rb + XEC_I2C_CR_OFS);
		sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_AAT_IEN_POS);
	}
#endif

	if (IS_ENABLED(CONFIG_I2C_MCHP_XEC_NL_TGT_AAT_HANDOFF)) {
		/* Wait for the match with nothing else running. The target command
		 * register reads 0 and the receive DMA stays stopped, so the byte mode
		 * state machine is what takes the address byte, and the ISR hands the
		 * transaction on from there.
		 *
		 * The done enable is cleared rather than left over from the transaction
		 * that just ended, so that finding it set means a transaction is in
		 * flight. The idle enable and status go with it: idle is enabled for the
		 * length of a transaction only, and the status latches.
		 *
		 * Clearing that enable cannot take an idle the host half was waiting on.
		 * It waits on one only with no target registered, and registering a
		 * target takes the API lock that a host transfer holds for its whole
		 * length, so there is no host request in flight when arming runs.
		 */
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x36U);
		sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_TD_IEN_POS);
		sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
		xec_i2c_v3_cmpl_clear(rb, BIT(XEC_I2C_CMPL_IDLE_POS));
		sys_write32(0U, rb + XEC_I2C_TCMD_OFS);
		xec_i2c_nl_clear_girqs(ctrl_cfg);
		return;
	}

	(void)xec_i2c_nl_tgt_nl_start(ctrl_cfg, ctrl_data);
	xec_i2c_nl_clear_girqs(ctrl_cfg);
}

static void xec_i2c_nl_tgt_arm(const struct xec_i2c_nl_config *ctrl_cfg,
			       struct xec_i2c_nl_data *ctrl_data)
{
	unsigned int key = 0;

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x30U);

	if (!xec_i2c_nl_tgt_registered(ctrl_data)) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x31U);
		return;
	}

	key = irq_lock();
	xec_i2c_nl_tgt_arm_locked(ctrl_cfg, ctrl_data);
	irq_unlock(key);
}

#ifdef CONFIG_I2C_MCHP_XEC_NL_TGT_WDOG
/* Recover a target transaction the hardware never reports done.
 *
 * The target state machine can be left running with no completion interrupt
 * pending, which was observed after it lost arbitration while transmitting: the
 * target command register still running and proceeding, no target done, and the
 * target NACKing its own address. Neither STOP detect enable reports anything this
 * driver can act on, the idle interrupt is ruled out while a target is armed, and
 * the hardware time-outs are unusable with 16-bit counts, so this is the only
 * escape.
 *
 * Armed per transaction by the address match and stopped when the transaction is
 * reported done, so expiry means the transaction neither finished nor reported.
 * Runs from the timer interrupt.
 *
 * It races one other way out, which is why a stall clears on its own eventually:
 * target done is set either when a target count reaches 0 or when an external STOP
 * is detected with the counts still non-zero, so a STOP from any later transaction
 * on the bus ends a stalled one too. The watchdog is what bounds how long that
 * takes, rather than the only thing that can end it.
 */
static void xec_i2c_nl_tgt_wdog_expiry(struct k_timer *timer)
{
	struct xec_i2c_nl_data *ctrl_data =
		CONTAINER_OF(timer, struct xec_i2c_nl_data, tgt_wdog);
	const struct xec_i2c_nl_config *ctrl_cfg = ctrl_data->controller->config;
	struct i2c_target_config *stalled = NULL;
	uint32_t tcmd = 0;
	int reason = I2C_ERROR_TIMEOUT;
	bool reset = false;
	uint8_t sr = 0;

	if (!xec_i2c_nl_tgt_registered(ctrl_data)) {
		return;
	}

	tcmd = sys_read32(ctrl_cfg->regbase + XEC_I2C_TCMD_OFS);
	sr = sys_read8(ctrl_cfg->regbase + XEC_I2C_SR_OFS);

	/* Arming the target again is the cheap recovery: it stops the target DMA, flushes
	 * the target buffers, clears the target status and rewrites the target command
	 * register, which is the state a stalled transaction needs put back. Resetting the
	 * controller is the expensive one: it disables the controller, which drops the
	 * arming of every target registered on it until this function puts it back.
	 *
	 * Arming alone is enough for a stall. Two runs of 100 loops on
	 * mec_assy6941/mec1753_qlj, one resetting at all 61 of its recoveries and one
	 * resetting at none of its 56, ended the same number of stalls, so a reset does
	 * not recover anything that arming does not. Nor did the count run away in the
	 * run that never reset, which it would have if the state machine had stayed
	 * stalled and expired the watchdog again.
	 *
	 * Only a bus error calls for the reset. The other two status bits that show up
	 * here do not:
	 *
	 * Lost arbitration stays asserted only through the byte it was detected in. It is
	 * cleared on the rising edge of the status PIN bit, when the network layer services
	 * the byte the controller asked for. Finding it set at a stall therefore says the
	 * loss happened in the byte where service stopped, not that the controller is
	 * wedged, so arming is enough.
	 *
	 * NBB stays 0 until the external controller issues a STOP and releases the lines,
	 * at least half a bus clock after the last byte, so at a stall it is always 0 and
	 * says nothing about the controller. Testing the whole byte against the idle value
	 * is what the first version of this did, and it sent every recovery to the reset.
	 */
	reset = ((sr & BIT(XEC_I2C_SR_BER_POS)) != 0U);

	/* A lost arbitration in the status says what stopped this transaction. Whether
	 * anything will say so again depends on the bus, and NBB is what says which:
	 *
	 * Bus still busy. The STOP has not happened yet, and when it does the hardware
	 * reports the transaction done with the loss still latched in the completion
	 * register. Leave the reason to that report and hold the target until it arrives,
	 * so the application is told once, with what actually happened, rather than told
	 * a time-out now and an arbitration loss later for the same transfer. Clearing
	 * the active target is what holds the report back: xec_i2c_nl_tgt_end() reports
	 * to it and reports nothing without it, so neither the error nor the stop
	 * callback runs here. Set the held target after that call, which arms the target
	 * again and clears the field.
	 *
	 * Bus idle. The STOP has already been and gone, so nothing further is coming and
	 * a hold would wait for a report that cannot arrive, until the next arming threw
	 * it away unreported. Report the loss here instead. A run of 100 loops on
	 * mec_assy6941/mec1753_qlj expired 40 times with a loss, 20 of them with the bus
	 * already idle, and holding those cost the application 20 errors it was never
	 * told about.
	 *
	 * Without the loss there is nothing to wait for and nothing better to say than
	 * that the transaction ran out of time. That reason is this watchdog's alone: the
	 * hardware time-out status cannot produce one, for the reason the comment on
	 * xec_i2c_v3_cmpl_clear() gives.
	 */
	if ((sr & BIT(XEC_I2C_SR_LAB_POS)) != 0U) {
		reason = I2C_ERROR_ARBITRATION;
		if ((sr & BIT(XEC_I2C_SR_NBB_POS)) == 0U) {
			stalled = ctrl_data->tgt_active;
			ctrl_data->tgt_active = NULL;
			reason = -1;
		}
	}

	XEC_I2C_NL_CNT_INC(ctrl_data, cnt_tgt_wdog);
	if (reset) {
		XEC_I2C_NL_CNT_INC(ctrl_data, cnt_tgt_wdog_reset);
	}

	LOG_ERR("I2C-NL target transaction timed out (tcmd 0x%08x sr 0x%02x), %s%s", tcmd,
		sr, reset ? "resetting" : "re-arming",
		(stalled != NULL) ? ", arbitration lost, held" : "");

	xec_i2c_nl_tgt_end(ctrl_cfg, ctrl_data, reason, reset);

	ctrl_data->tgt_stalled = stalled;
	ctrl_data->tgt_stalled_seq = ctrl_data->tgt_seq;
}
#endif

/* Target of the transaction being reported, or NULL.
 *
 * The network layer puts the address it matched at the start of the receive buffer,
 * so the buffer names the target. With the hand-off it never saw that byte and the
 * address shadow register is what names it.
 *
 * Read here and not at the match: the match is reported at the end of the seventh
 * clock of the address and the hardware does not copy the address into the shadow
 * register until after the eighth, so an ISR that is not held up reads it before it
 * means anything. By the time a transaction is reported it has been valid for the
 * whole of it.
 */
static struct i2c_target_config *xec_i2c_nl_tgt_match(struct xec_i2c_nl_data *ctrl_data,
						      uint8_t addr);

static struct i2c_target_config *xec_i2c_nl_tgt_of_xfr(const struct xec_i2c_nl_config *ctrl_cfg,
						       struct xec_i2c_nl_data *ctrl_data,
						       const uint8_t *buf, uint32_t off)
{
	if (IS_ENABLED(CONFIG_I2C_MCHP_XEC_NL_TGT_AAT_HANDOFF)) {
		uint8_t ias = (uint8_t)(sys_read32(ctrl_cfg->regbase + XEC_I2C_IAS_OFS) & 0xffU);

		return xec_i2c_nl_tgt_match(ctrl_data, ias >> 1);
	}

	if (off == 0U) {
		return NULL;
	}

	return xec_i2c_nl_tgt_match(ctrl_data, buf[0] >> 1);
}

/* Registered target with 7-bit address addr, else NULL */
static struct i2c_target_config *xec_i2c_nl_tgt_match(struct xec_i2c_nl_data *ctrl_data,
						      uint8_t addr)
{
	for (size_t i = 0; i < ARRAY_SIZE(ctrl_data->tgt_cfg); i++) {
		if ((ctrl_data->tgt_cfg[i] != NULL) && (ctrl_data->tgt_cfg[i]->address == addr)) {
			return ctrl_data->tgt_cfg[i];
		}
	}

	return NULL;
}

static void xec_i2c_nl_tgt_deliver(struct i2c_target_config *tgt, uint8_t *buf, uint32_t len)
{
	if ((tgt != NULL) && (len != 0U)) {
		tgt->callbacks->buf_write_received(tgt, buf, len);
	}
}

/* End the target transaction: report an error (reason >= 0), invoke the stop callback,
 * and re-arm. A bus error or lost arbitration needs a controller reset, which re-arms
 * the target; it is left to the host path when a host transaction is running.
 */
static void xec_i2c_nl_tgt_end(const struct xec_i2c_nl_config *ctrl_cfg,
			       struct xec_i2c_nl_data *ctrl_data, int reason, bool reset)
{
	struct i2c_target_config *tgt = ctrl_data->tgt_active;
	uintptr_t rb = ctrl_cfg->regbase;

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xB0U);

	if (reason >= 0) {
		XEC_I2C_NL_CNT_INC(ctrl_data, cnt_tgt_err);
	}

	(void)dma_stop(ctrl_cfg->dma_dev, ctrl_cfg->dma_chan2);

#ifdef CONFIG_I2C_MCHP_XEC_NL_TGT_WDOG
	/* The transaction is being reported, so it did not stall */
	k_timer_stop(&ctrl_data->tgt_wdog);
#endif

	/* A transaction is reported to the target one way or the other, never both. It
	 * failed, and the single error callback is all of it, or it succeeded, and the
	 * stop callback closes the data the receive and read-request callbacks carried.
	 * A failed transaction delivers no data either: every delivery is behind a test
	 * that no reason has been set.
	 *
	 * An external write-read is reported a phase at a time, which is the one case
	 * where a target hears about a transaction before it ends. The external host
	 * sends START, the write address, its data bytes, then RPT-START and the read
	 * address. The RPT-START pauses the state machine and xec_i2c_nl_tgt_pause()
	 * delivers the data bytes to the receive callback and asks the read-request
	 * callback for the buffer to answer the read with, because the target cannot
	 * supply that buffer without first being told what was written. A failure in the
	 * read phase after that is still one error callback and no stop, and a clean read
	 * phase ends at target done with the stop callback.
	 *
	 * Those delivered bytes are a phase that completed. A failure in the write phase
	 * does not reach them: the dispatch in xec_i2c_nl_tgt_isr() routes a transaction
	 * whose completion register holds any of XEC_I2C_NL_CMPL_HOST_FATAL to the done
	 * path, so the pause path, and the delivery in it, is reached only with none of
	 * them set.
	 */
	if (tgt != NULL) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xB1U);
		if (reason >= 0) {
			if (tgt->callbacks->error != NULL) {
				XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xB2U);
				tgt->callbacks->error(tgt, (enum i2c_error_reason)reason);
			}
		} else if (tgt->callbacks->stop != NULL) {
			XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xB3U);
			(void)tgt->callbacks->stop(tgt);
		} else {
			/* No callback for this outcome */
		}
	}

	if (reset && (sys_test_bit(rb + XEC_I2C_HCMD_OFS, XEC_I2C_HCMD_RUN_POS) == 0)) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xB4U);
		(void)xec_i2c_nl_program_ctrl(ctrl_cfg, ctrl_data, ctrl_data->active_freq,
					      ctrl_data->active_port);
		return;
	}

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xB5U);
	xec_i2c_nl_tgt_arm(ctrl_cfg, ctrl_data);
}

/* Target state machine paused on a RPT-START write or a read request. Deliver the
 * write data received before the pause, then continue receiving or start transmitting.
 */
static void xec_i2c_nl_tgt_pause(const struct xec_i2c_nl_config *ctrl_cfg,
				 struct xec_i2c_nl_data *ctrl_data)
{
	uintptr_t rb = ctrl_cfg->regbase;
	uint8_t *buf = ctrl_cfg->tgt_buf;
	uint32_t rcvd = xec_i2c_nl_tgt_rx_count(ctrl_cfg, ctrl_data);
	uint32_t off = ctrl_data->tgt_rx_off;
	struct i2c_target_config *tgt = NULL;
	uint32_t tcmd = 0;
	uint16_t elen = 0;
	uint8_t *ptr = NULL;
	uint32_t len = 0;
	uint8_t addr_byte = 0;
	int rc = 0;

	if (rcvd == 0U) {
		xec_i2c_nl_tgt_end(ctrl_cfg, ctrl_data, I2C_ERROR_GENERIC, false);
		return;
	}

	ctrl_data->tgt_active = xec_i2c_nl_tgt_of_xfr(ctrl_cfg, ctrl_data, buf, off);

	/* The address byte that caused the pause is the last byte received */
	addr_byte = buf[rcvd - 1U];
	if ((rcvd - 1U) > off) {
		xec_i2c_nl_tgt_deliver(ctrl_data->tgt_active, &buf[off], rcvd - 1U - off);
	}

	tgt = xec_i2c_nl_tgt_match(ctrl_data, addr_byte >> 1);
	if (tgt == NULL) {
		xec_i2c_nl_tgt_end(ctrl_cfg, ctrl_data, I2C_ERROR_GENERIC, false);
		return;
	}
	ctrl_data->tgt_active = tgt;

	tcmd = sys_read32(rb + XEC_I2C_TCMD_OFS);

	if ((addr_byte & 1U) == 0U) {
		/* RPT-START write: receive its data from the buffer start */
		ctrl_data->tgt_rx_off = 0U;
		rc = xec_i2c_nl_tgt_rx_start(ctrl_cfg, ctrl_data);
		if (rc != 0) {
			xec_i2c_nl_tgt_end(ctrl_cfg, ctrl_data, I2C_ERROR_DMA, false);
			return;
		}
		tcmd &= ~XEC_I2C_TCMD_RCL_MSK;
		tcmd |= XEC_I2C_TCMD_RCL_SET(ctrl_cfg->tgt_buf_size & 0xffU);
		sys_write32(tcmd | BIT(XEC_I2C_TCMD_PROC_POS), rb + XEC_I2C_TCMD_OFS);
		return;
	}

	/* Read request. With no data the hardware resends the transmit buffer and sets
	 * TPROT, reported at the end of the transaction.
	 */
	if ((tgt->callbacks->buf_read_requested(tgt, &ptr, &len) != 0) || (ptr == NULL)) {
		len = 0U;
	}
	len = MIN(len, I2C_HW_MAX_COUNT);

	sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_FTTX_POS);
	if (len != 0U) {
		ctrl_data->ttx_blk.source_address = (uintptr_t)ptr;
		ctrl_data->ttx_blk.block_size = len;
		rc = dma_config(ctrl_cfg->dma_dev, ctrl_cfg->dma_chan2, &ctrl_data->dma_ttx);
		if (rc == 0) {
			rc = dma_start(ctrl_cfg->dma_dev, ctrl_cfg->dma_chan2);
		}
		if (rc != 0) {
			LOG_ERR("I2C-NL target TX DMA start error (%d)", rc);
			len = 0U;
		}
	}

	elen = sys_read16(rb + XEC_I2C_NL_ELEN_TGT_OFS) & ~XEC_I2C_NL_ELEN_TGT_WR_MSK;
	elen |= FIELD_PREP(XEC_I2C_NL_ELEN_TGT_WR_MSK, len >> 8);
	sys_write16(elen, rb + XEC_I2C_NL_ELEN_TGT_OFS);

	tcmd &= ~XEC_I2C_TCMD_WCL_MSK;
	tcmd |= XEC_I2C_TCMD_WCL_SET(len & 0xffU);
	sys_write32(tcmd | BIT(XEC_I2C_TCMD_PROC_POS), rb + XEC_I2C_TCMD_OFS);
}

/* Target state machine stopped: the transaction ended */
static void xec_i2c_nl_tgt_done(const struct xec_i2c_nl_config *ctrl_cfg,
				struct xec_i2c_nl_data *ctrl_data, uint32_t cmpl)
{
	uintptr_t rb = ctrl_cfg->regbase;
	uint8_t *buf = ctrl_cfg->tgt_buf;
	uint32_t off = ctrl_data->tgt_rx_off;
	uint32_t rcvd = 0;
	uint32_t tx_left = 0;
	int reason = -1;
	bool reset = false;
	bool late = false;

	XEC_I2C_NL_CNT_INC(ctrl_data, cnt_tgt_done);

#ifdef CONFIG_I2C_MCHP_XEC_NL_TGT_WDOG
	/* A transaction the watchdog ended on a lost arbitration may be being reported
	 * now. Take its target back as the active one so the error reaches it from here,
	 * which is the one report the application gets for that transfer: the watchdog
	 * left it to this, having no reason to give that the completion register does not
	 * hold.
	 *
	 * Only while this report is that transaction's. Nothing should get between the
	 * two, the report needing a STOP and the bus not reaching a START without one, so
	 * no later transaction can be addressed before the stalled one reports. But a
	 * report that never comes would otherwise leave the held target for whichever
	 * transaction reported next, and hand it an arbitration loss that was not its
	 * own. The address match that starts a transaction numbers it, so a number that
	 * has moved on says the held report missed its turn: drop it, and let this
	 * transaction report as itself.
	 *
	 * That leaves one way to mistake them, a report that never comes and an address
	 * match the ISR missed the window on, which takes both to go wrong at once.
	 */
	if (ctrl_data->tgt_stalled != NULL) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xA2U);
		if (ctrl_data->tgt_stalled_seq == ctrl_data->tgt_seq) {
			XEC_I2C_NL_CNT_INC(ctrl_data, cnt_tgt_done_late);
			ctrl_data->tgt_active = ctrl_data->tgt_stalled;
			late = true;
		} else {
			XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xA3U);
			XEC_I2C_NL_CNT_INC(ctrl_data, cnt_tgt_hold_stale);
		}
		ctrl_data->tgt_stalled = NULL;
	}
#endif

	/* A lost arbitration resets the controller, and it has to. Reading the status bit
	 * argues otherwise, the loss being asserted only through the byte it was detected
	 * in and cleared when the network layer services that byte, so by the time the
	 * transaction is reported done the controller is no longer in the loss. Something
	 * else in the block is, and only programming the controller again clears it: with
	 * this left arming the target, a run of 100 loops on mec_assy6941/mec1753_qlj
	 * failed every one of the 100 host transfers the application makes to another
	 * device on the same controller, 83 of them timing out, against 24 failures in
	 * the same run with the reset in place. Transfers on a second controller moved by
	 * a handful, which is the noise this test has.
	 *
	 * The target watchdog does take a lost arbitration without resetting, and that
	 * much measured clean. The difference is that it recovers a transaction the
	 * hardware never reported, where here the hardware has reported one it was in the
	 * middle of.
	 */
	if ((cmpl & BIT(XEC_I2C_CMPL_LAB_STS_POS)) != 0U) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x96U);
		reason = I2C_ERROR_ARBITRATION;
		reset = true;
	} else if ((cmpl & BIT(XEC_I2C_CMPL_BER_STS_POS)) != 0U) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x97U);
		reason = I2C_ERROR_GENERIC;
		reset = true;
	} else if ((cmpl & BIT(XEC_I2C_CMPL_TMO_STS_POS)) != 0U) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x98U);
		reason = I2C_ERROR_TIMEOUT;
		reset = true;
	} else if ((cmpl & BIT(XEC_I2C_CMPL_TNAKR_STS_POS)) != 0U) {
		/* Receive overflow NACKed by hardware: drop the data */
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x99U);
		reason = I2C_ERROR_SIZE;
	}

	/* The watchdog deferred to this report because it saw a lost arbitration in the
	 * status, so say so even if the completion register no longer does. Reporting
	 * nothing is the one outcome that would leave the transfer unaccounted for.
	 */
	if (late && (reason < 0)) {
		reason = I2C_ERROR_ARBITRATION;
	}

	/* TTR reads 0 when the target finished the receive phase and 1 when it finished
	 * the transmit phase. The I2C-SMB v3.8 data sheet documents TTR the other way
	 * round, but that contradicts both the HTR description of the host state machine
	 * in the same table and the vendor HAL, and state capture on MEC1753 shows TTR
	 * clear at TDONE for an external write. Do not invert this test to match the
	 * data sheet: doing so drops every write transaction.
	 */
	/* Split the completed transactions by the phase they ended in, so a rate measured
	 * per transaction can be compared against the receive count rather than against a
	 * guess at how many of them were receives. The transmit arm below is reached only
	 * with TPROT set, so it cannot carry this count.
	 */
	if ((cmpl & BIT(XEC_I2C_CMPL_TTR_POS)) == 0U) {
		XEC_I2C_NL_CNT_INC(ctrl_data, cnt_tgt_done_rx);
	} else {
		XEC_I2C_NL_CNT_INC(ctrl_data, cnt_tgt_done_tx);
	}

	if ((cmpl & BIT(XEC_I2C_CMPL_TTR_POS)) == 0U) {
		/* Receive phase ended: a write transaction */
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x9AU);
		rcvd = xec_i2c_nl_tgt_rx_count(ctrl_cfg, ctrl_data);
		if ((rcvd != 0U) && (ctrl_data->tgt_active == NULL)) {
			XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x9BU);
			ctrl_data->tgt_active =
				xec_i2c_nl_tgt_of_xfr(ctrl_cfg, ctrl_data, buf, off);
		}
		if ((reason < 0) && (rcvd > off)) {
			XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x9CU);
			xec_i2c_nl_tgt_deliver(ctrl_data->tgt_active, &buf[off], rcvd - off);
		}
	} else if ((reason < 0) && ((cmpl & BIT(XEC_I2C_CMPL_TPROT_POS)) != 0U)) {
		/* Transmit phase ended: a read transaction. TPROT with the write
		 * count at 0 means the external host read beyond the data supplied.
		 */
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x9DU);
		tx_left = XEC_I2C_TCMD_WCL_GET(sys_read32(rb + XEC_I2C_TCMD_OFS));
		tx_left |= XEC_I2C_ELEN_TWR_GET(sys_read32(rb + XEC_I2C_ELEN_OFS)) << 8;
		if (tx_left == 0U) {
			XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x9EU);
			reason = I2C_ERROR_SIZE;
		}
	}

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x9FU);
	xec_i2c_nl_tgt_end(ctrl_cfg, ctrl_data, reason, reset);
}

#ifdef XEC_I2C_NL_TGT_START
/* The interrupt that says a target transaction has started.
 *
 * The address match is reported at the end of the seventh clock of the address, before
 * the hardware has finished with the byte. The byte mode service request is asserted
 * after it has ACKed an address it matched, and holds SCL low until it is answered, so
 * entering on it means the address byte is complete and everything the hardware
 * captured about it is valid. There is no enable to read back for it, the Control
 * register being write only: what says the target is armed and waiting is the done
 * interrupt being disabled, which arming leaves that way and only
 * xec_i2c_nl_tgt_nl_start() sets.
 */
static inline bool xec_i2c_nl_tgt_started(const struct xec_i2c_nl_config *ctrl_cfg, uint32_t cfg)
{
	uint8_t sr = sys_read8(ctrl_cfg->regbase + XEC_I2C_SR_OFS);

	if (IS_ENABLED(CONFIG_I2C_MCHP_XEC_NL_TGT_HANDOFF_ON_PIN)) {
		return (((cfg & BIT(XEC_I2C_CFG_TD_IEN_POS)) == 0U) &&
			((sr & BIT(XEC_I2C_SR_AAT_POS)) != 0U) &&
			((sr & BIT(XEC_I2C_SR_PIN_POS)) == 0U));
	}

	return (((cfg & BIT(XEC_I2C_CFG_AAT_IEN_POS)) != 0U) &&
		((sr & BIT(XEC_I2C_SR_AAT_POS)) != 0U));
}
#endif

/* Target part of the controller ISR. Clears the target status, then the GIRQs, before
 * acting. Returns true if a target event was handled.
 */
static bool xec_i2c_nl_tgt_isr(const struct xec_i2c_nl_config *ctrl_cfg,
			       struct xec_i2c_nl_data *ctrl_data, uint32_t cfg, uint32_t cmpl)
{
	uintptr_t rb = ctrl_cfg->regbase;
	uint32_t tcmd = 0;

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x90U);

#ifdef XEC_I2C_NL_TGT_START
	/* A target transaction has started. Which interrupt says so is all the two
	 * entries differ in, so everything done about it afterwards is here.
	 *
	 * Address match entry clears the enable, not the status: the Status register is
	 * read only, and the status clears itself when the network layer moves the
	 * address byte out of the Data register. Leaving the enable set would re-enter
	 * here until it did.
	 *
	 * Byte mode entry has nothing to clear an enable in, the Control register being
	 * write only, so it writes the register back without that bit. It must, before
	 * either command register runs: byte mode interrupts per byte, which would fight
	 * the network layer here and take away the whole reason for using it on a host
	 * transfer. PIN is written as 0 so the service request is disabled and not
	 * answered, the byte being the network layer's to take.
	 */
	if (xec_i2c_nl_tgt_started(ctrl_cfg, cfg)) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xA8U);
		XEC_I2C_NL_CNT_INC(ctrl_data, cnt_tgt_aat);
		ctrl_data->tgt_seq++;
		if (IS_ENABLED(CONFIG_I2C_MCHP_XEC_NL_TGT_HANDOFF_ON_PIN)) {
			sys_write8(XEC_I2C_NL_CR_HOLD, rb + XEC_I2C_CR_OFS);
		} else {
			sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_AAT_IEN_POS);
		}

		if (IS_ENABLED(CONFIG_I2C_MCHP_XEC_NL_TGT_AAT_HANDOFF)) {
			/* The byte mode state machine took the address byte and the
			 * network layer has not been started. Hand the transaction to
			 * it: enable the end of transaction interrupts, program the
			 * receive DMA, and start the target command register last. The
			 * address byte is left in the Data register for the network
			 * layer to move out, which is also what clears the status read
			 * above.
			 *
			 * The idle status is cleared before its enable, not after: it
			 * latches, so whatever the bus did before this transaction
			 * would otherwise report as this one going idle the moment the
			 * enable is set.
			 */
			XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xA5U);
			xec_i2c_v3_cmpl_clear(rb, BIT(XEC_I2C_CMPL_IDLE_POS));
			sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
			if (xec_i2c_nl_tgt_nl_start(ctrl_cfg, ctrl_data) != 0) {
				XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xA6U);
				xec_i2c_nl_tgt_end(ctrl_cfg, ctrl_data, I2C_ERROR_DMA,
						   false);
				xec_i2c_nl_clear_girqs(ctrl_cfg);
				return true;
			}
		}

#ifdef CONFIG_I2C_MCHP_XEC_NL_TGT_WDOG
		/* Start the watchdog for this transaction, so one the hardware leaves
		 * running without ever reporting it done is still ended. Reporting it
		 * done stops the timer.
		 */
		k_timer_start(&ctrl_data->tgt_wdog,
			      K_MSEC(CONFIG_I2C_MCHP_XEC_NL_TGT_WDOG_MS), K_NO_WAIT);
#endif
		xec_i2c_nl_clear_girqs(ctrl_cfg);
		return true;
	}
#endif

	if (((cfg & BIT(XEC_I2C_CFG_TD_IEN_POS)) != 0U) &&
	    ((cmpl & BIT(XEC_I2C_CMPL_TDONE_POS)) != 0U)) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x91U);
		xec_i2c_v3_cmpl_clear(rb, cmpl & XEC_I2C_NL_CMPL_TGT_STS);
		xec_i2c_nl_clear_girqs(ctrl_cfg);

#ifdef CONFIG_I2C_MCHP_XEC_NL_TGT_AAT
		/* The match enable is cleared when a match is claimed and set again only
		 * when the target is armed, so still finding it set here says the match
		 * for this transaction was never seen. With the hand-off that cannot
		 * happen: the done interrupt this branch runs on is only enabled by the
		 * match, so a done without one means the hardware reported a transaction
		 * the driver never started.
		 */
		if ((cfg & BIT(XEC_I2C_CFG_AAT_IEN_POS)) != 0U) {
			XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xA4U);
			XEC_I2C_NL_CNT_INC(ctrl_data, cnt_tgt_aat_miss);
		}
#endif

		tcmd = sys_read32(rb + XEC_I2C_TCMD_OFS);
		if (((tcmd & BIT(XEC_I2C_TCMD_RUN_POS)) == 0U) ||
		    ((cmpl & XEC_I2C_NL_CMPL_HOST_FATAL) != 0U) ||
		    ((cmpl & BIT(XEC_I2C_CMPL_TNAKR_STS_POS)) != 0U)) {
			XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x92U);
			xec_i2c_nl_tgt_done(ctrl_cfg, ctrl_data, cmpl);
		} else if ((tcmd & BIT(XEC_I2C_TCMD_PROC_POS)) == 0U) {
			XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x93U);
			xec_i2c_nl_tgt_pause(ctrl_cfg, ctrl_data);
		} else {
			/* TDONE with the state machine left running and proceeding.
			 * Hardware clears the RUN bit when a target transaction
			 * completes and PROCEED when it pauses, so this combination
			 * should not occur.
			 * The status and the GIRQs are already cleared, so returning
			 * without acting would strand the target with no further
			 * interrupt to recover on. Put it back to a known armed state.
			 *
			 * The state is recorded whether or not the recovery is enabled,
			 * so turning the recovery off to measure what it changes does
			 * not also hide the state it recovers from.
			 */
			XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xA1U);
			XEC_I2C_NL_CNT_INC(ctrl_data, cnt_tgt_stuck);
			LOG_ERR("I2C-NL target done with TCMD still running (0x%08x)", tcmd);
			if (IS_ENABLED(CONFIG_I2C_MCHP_XEC_NL_FIX_TGT_TDONE_STUCK)) {
				xec_i2c_nl_tgt_end(ctrl_cfg, ctrl_data, I2C_ERROR_GENERIC,
						   false);
			}
		}
		return true;
	}

#ifdef CONFIG_I2C_MCHP_XEC_NL_TGT_AAT_HANDOFF
	/* The bus went idle and the transaction was not reported done, which is reached
	 * only after the done branch above has declined it. Idle fires once both lines
	 * have been high for a window, so the transaction is over however it ended, and
	 * nothing further is coming for it: end it here.
	 *
	 * This is the recovery the watchdog does, from the hardware saying the bus is
	 * free rather than from a timer saying long enough has passed, so it does not
	 * need the watchdog's hold: the bus being idle is what the hold waits for.
	 * Report what the status says and reset only on a bus error, for the reasons
	 * xec_i2c_nl_tgt_wdog_expiry() gives.
	 *
	 * Only the hand-off can get here. Armed the other way the target command
	 * register runs for as long as a target is registered, which is what rules the
	 * idle interrupt out, and the host half owns the enable instead.
	 */
	if (xec_i2c_nl_tgt_registered(ctrl_data) &&
	    ((cfg & BIT(XEC_I2C_CFG_IDLE_IEN_POS)) != 0U) &&
	    ((cmpl & BIT(XEC_I2C_CMPL_IDLE_POS)) != 0U)) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0xA7U);
		XEC_I2C_NL_CNT_INC(ctrl_data, cnt_tgt_idle);
		sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
		xec_i2c_v3_cmpl_clear(rb, BIT(XEC_I2C_CMPL_IDLE_POS));

		/* Report it as the transaction it was, from the completion register,
		 * which is what holds the reason and what the done path already reads.
		 * Target done is also where the target is matched and the received data
		 * delivered, so go through it rather than around it.
		 *
		 * Log only when the completion register does not say why the transaction
		 * ended. When it does, this path is not a fault but how that ending is
		 * found: a receive overflow NACK ends a transaction with no target done
		 * at all, so the idle interrupt is the only thing that reports it, and
		 * the reason reaches the application through the error callback either
		 * way. Logging it regardless put an error line against every overflow,
		 * next to the one the application prints for the same transfer.
		 */
		if ((cmpl & XEC_I2C_NL_CMPL_TGT_ENDED) == 0U) {
			LOG_ERR("I2C-NL target stalled, ended at bus idle (cmpl 0x%08x)",
				cmpl);
		}
		xec_i2c_nl_tgt_done(ctrl_cfg, ctrl_data, cmpl);
		xec_i2c_nl_clear_girqs(ctrl_cfg);
		return true;
	}
#endif

	/* There is no STOP detect path here. The Configuration register has two STOP
	 * detect interrupt enables and neither is usable by this driver. Both were
	 * measured on MEC1753 over runs of a few hundred target transactions each, with a
	 * scope confirming the transfers do end with a STOP.
	 *
	 * Bit 27, the enable meant for a network layer driver, reports nothing that can
	 * be acted on. The Status register STOP status is never set, armed in either
	 * direction; the detect never delivers an interrupt that can be confirmed; and
	 * arming it for the receive direction adds about 0.8 interrupts per target
	 * transaction carrying no status at all, the bits common to all of them being
	 * none and the bits set on any of them being host status only.
	 *
	 * Bit 24, the enable meant for a driver doing byte mode interrupts, is worse.
	 * Setting it in this network layer plus DMA configuration stops the controller
	 * generating interrupts at all: its ISR did not run once over 100 loops, its
	 * targets NACKed their own addresses on all 500 transfers addressed to them, and
	 * its own host transfers timed out. A second controller in the same run, with no
	 * target registered and so with the bit never set, was unaffected.
	 *
	 * The Status register STOP status was never seen set in any configuration,
	 * including with no STOP detect enabled at all. The hardware detects an
	 * externally generated STOP in target mode alone, and earlier versions did so
	 * only while the target was receiving, which is not the direction a transaction
	 * that needs rescuing stalls in.
	 *
	 * A target transaction the hardware never reports done is recovered by
	 * I2C_MCHP_XEC_NL_TGT_WDOG instead. Do not re-add either enable from the data
	 * sheet.
	 */
	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x95U);

	/* An interrupt the target half did not claim is only interesting when the target is
	 * armed and the hardware did not report it done: that is a target GIRQ asserted with
	 * no target source behind it. Every host interrupt also reaches here, and on a
	 * controller with no target registered so does every interrupt it ever takes, so
	 * counting all of them measures host traffic rather than anything spurious.
	 */
	if (((cfg & BIT(XEC_I2C_CFG_TD_IEN_POS)) != 0U) &&
	    ((cmpl & BIT(XEC_I2C_CMPL_TDONE_POS)) == 0U)) {
		XEC_I2C_NL_CNT_INC(ctrl_data, cnt_isr_unclaimed);
	}

	return false;
}

/* API: target register
 * Up to two targets per controller (OWN_ADDRESS_1 and OWN_ADDRESS_2), with different
 * addresses, on one port. The controller is routed to that port and stays there while
 * a target is registered: operations needing another port return -EBUSY.
 */
static int xec_i2c_nl_vport_target_register(const struct device *port_dev,
					    struct i2c_target_config *cfg)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	struct xec_i2c_nl_port_data *port_data = port_dev->data;
	const struct xec_i2c_nl_config *ctrl_cfg = port_cfg->controller->config;
	struct xec_i2c_nl_data *ctrl_data = port_cfg->controller->data;
	size_t slot = ARRAY_SIZE(ctrl_data->tgt_cfg);
	int rc = 0;

	if ((cfg == NULL) || (cfg->callbacks == NULL) ||
	    (cfg->callbacks->buf_write_received == NULL) ||
	    (cfg->callbacks->buf_read_requested == NULL)) {
		return -EINVAL;
	}

	if ((cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0U) {
		return -ENOTSUP;
	}

	if ((cfg->address == 0U) || (cfg->address > 0x7FU)) {
		return -EINVAL;
	}

	if (!ctrl_cfg->has_tgt_dma) {
		return -ENODEV;
	}

	if ((ctrl_cfg->tgt_buf == NULL) || (ctrl_cfg->tgt_buf_size == 0U)) {
		return -ENOSYS;
	}

	k_sem_take(&ctrl_data->lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(ctrl_data->tgt_cfg); i++) {
		if ((ctrl_data->tgt_cfg[i] == cfg) ||
		    ((ctrl_data->tgt_cfg[i] != NULL) &&
		     (ctrl_data->tgt_cfg[i]->address == cfg->address))) {
			rc = -EINVAL;
			goto unlock;
		}
		if ((ctrl_data->tgt_cfg[i] == NULL) && (slot == ARRAY_SIZE(ctrl_data->tgt_cfg))) {
			slot = i;
		}
	}

	if (slot == ARRAY_SIZE(ctrl_data->tgt_cfg)) {
		rc = -EBUSY;
		goto unlock;
	}

	if (xec_i2c_nl_tgt_registered(ctrl_data)) {
		/* All targets listen on one port. Re-arming needs an idle bus. */
		if ((ctrl_data->tgt_port_dev != port_dev) || xec_i2c_nl_is_busy(ctrl_cfg)) {
			rc = -EBUSY;
			goto unlock;
		}
	} else {
		rc = xec_i2c_nl_apply_port(port_cfg, port_data, ctrl_cfg, ctrl_data);
		if (rc != 0) {
			goto unlock;
		}
		ctrl_data->tgt_port_dev = port_dev;
	}

	ctrl_data->tgt_cfg[slot] = cfg;
	xec_i2c_nl_tgt_arm(ctrl_cfg, ctrl_data);

unlock:
	k_sem_give(&ctrl_data->lock);

	return rc;
}

/* API: target unregister. Removing the last target resets the controller, which is the
 * only way to stop the target state machine.
 */
static int xec_i2c_nl_vport_target_unregister(const struct device *port_dev,
					      struct i2c_target_config *cfg)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	const struct xec_i2c_nl_config *ctrl_cfg = port_cfg->controller->config;
	struct xec_i2c_nl_data *ctrl_data = port_cfg->controller->data;
	int rc = -EINVAL;

	if (cfg == NULL) {
		return -EINVAL;
	}

	k_sem_take(&ctrl_data->lock, K_FOREVER);

	if (ctrl_data->tgt_port_dev != port_dev) {
		goto unlock;
	}

	for (size_t i = 0; i < ARRAY_SIZE(ctrl_data->tgt_cfg); i++) {
		if (ctrl_data->tgt_cfg[i] == cfg) {
			ctrl_data->tgt_cfg[i] = NULL;
			rc = 0;
		}
	}

	if (rc != 0) {
		goto unlock;
	}

	if ((ctrl_data->tgt_cfg[0] == NULL) && (ctrl_data->tgt_cfg[1] == NULL)) {
		unsigned int key = irq_lock();

		ctrl_data->tgt_port_dev = NULL;
#ifdef CONFIG_I2C_MCHP_XEC_NL_TGT_WDOG
		/* Last target leaving: no arming follows to drop a held report */
		ctrl_data->tgt_stalled = NULL;
#endif
		sys_clear_bit(ctrl_cfg->regbase + XEC_I2C_CFG_OFS, XEC_I2C_CFG_TD_IEN_POS);
		irq_unlock(key);
		(void)dma_stop(ctrl_cfg->dma_dev, ctrl_cfg->dma_chan2);
		(void)xec_i2c_nl_program_ctrl(ctrl_cfg, ctrl_data, ctrl_data->active_freq,
					      ctrl_data->active_port);
		xec_i2c_nl_port_settle();
	} else {
		xec_i2c_nl_tgt_arm(ctrl_cfg, ctrl_data);
	}

unlock:
	k_sem_give(&ctrl_data->lock);

	return rc;
}
#elif defined(CONFIG_I2C_TARGET)
/* Target mode requires CONFIG_I2C_TARGET_BUFFER_MODE: DMA gives no per byte events */
static int xec_i2c_nl_vport_target_register(const struct device *port_dev,
					    struct i2c_target_config *cfg)
{
	ARG_UNUSED(port_dev);
	ARG_UNUSED(cfg);

	return -ENOTSUP;
}

static int xec_i2c_nl_vport_target_unregister(const struct device *port_dev,
					      struct i2c_target_config *cfg)
{
	ARG_UNUSED(port_dev);
	ARG_UNUSED(cfg);

	return -ENOTSUP;
}
#endif /* CONFIG_I2C_TARGET_BUFFER_MODE */

/* End a host transaction: stop DMA and the HDONE interrupt. When the host state
 * machine has stopped and no controller reset is needed, wait for the IDLE interrupt
 * (bus released after STOP) before waking the thread. Otherwise wake the thread now
 * to reset the controller. The IDLE interrupt must not be enabled while HRUN is set.
 */
static void xec_i2c_nl_isr_finish(const struct xec_i2c_nl_config *ctrl_cfg,
				  struct xec_i2c_nl_data *ctrl_data, uint32_t hcmd)
{
	uintptr_t rb = ctrl_cfg->regbase;

	(void)dma_stop(ctrl_cfg->dma_dev, ctrl_cfg->dma_chan1);
	sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_HD_IEN_POS);

#ifdef CONFIG_I2C_MCHP_XEC_NL_TGT_HANDOFF_ON_PIN
	/* Put the byte mode interrupt back, the host command register having stopped.
	 * Not while a target transaction is in flight: the network layer owns the
	 * controller then and arming would throw that transaction away. The done
	 * interrupt being enabled is what says one is, and when it ends its own arming
	 * puts this back.
	 */
	if (xec_i2c_nl_tgt_registered(ctrl_data) &&
	    (sys_test_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_TD_IEN_POS) == 0)) {
		xec_i2c_nl_tgt_arm(ctrl_cfg, ctrl_data);
	}
#endif

	if (ctrl_data->xfr_err != 0) {
		sys_set_bits(rb + XEC_I2C_CFG_OFS, XEC_I2C_NL_CFG_FLUSH_HOST);
	}

	if (ctrl_data->xfr_reset || ((hcmd & BIT(XEC_I2C_HCMD_RUN_POS)) != 0U)) {
		ctrl_data->xfr_reset = true;
		xec_i2c_nl_req_done(ctrl_cfg, ctrl_data);
		return;
	}

	/* An armed target keeps TCMD.RUN set, which rules out the IDLE interrupt */
	if (xec_i2c_nl_tgt_registered(ctrl_data)) {
		xec_i2c_nl_req_done(ctrl_cfg, ctrl_data);
		return;
	}

	xec_i2c_v3_cmpl_clear(rb, BIT(XEC_I2C_CMPL_IDLE_POS));
	sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
}

/* Controller ISR
 * HDONE fires when the host state machine stops:
 *   BER, LAB, time-out, or NAK: transaction ended with an error.
 *   HPROCEED=0, HRUN=1: write count reached 0 and a read phase follows. Configure
 *     RX DMA and set HPROCEED to continue. The bus is stretched until then.
 *   HPROCEED=0, HRUN=0: transaction complete.
 * IDLE fires, when enabled at the end of a transaction, once the bus is released.
 */
static void xec_i2c_nl_isr_handler(const struct device *ctrl_dev)
{
	const struct xec_i2c_nl_config *ctrl_cfg = ctrl_dev->config;
	struct xec_i2c_nl_data *ctrl_data = ctrl_dev->data;
	uintptr_t rb = ctrl_cfg->regbase;
	uint32_t cfg = 0; /* sys_read32(rb + XEC_I2C_CFG_OFS); */
	uint32_t cmpl = 0; /* sys_read32(rb + XEC_I2C_CMPL_OFS); */
	uint32_t hcmd = 0;
	bool handled = false;
	int rc = 0;

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x80U);
	XEC_I2C_NL_ISR_CAP_UPDATE(ctrl_data);

	cfg = sys_read32(rb + XEC_I2C_CFG_OFS);
	cmpl = sys_read32(rb + XEC_I2C_CMPL_OFS);

	/* Each path clears the I2C status, then the GIRQs, before enabling any new
	 * interrupt source (IDLE, HPROCEED, the next request, or the target re-arm).
	 */
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	handled = xec_i2c_nl_tgt_isr(ctrl_cfg, ctrl_data, cfg, cmpl);
#endif

	/* The host half waits on idle only with no target registered, which is how
	 * xec_i2c_nl_isr_finish() decides to enable it, so a registered target means the
	 * enable is not this half's and the target half above has dealt with it. Without
	 * this test that half's idle would wake a host request that is not waiting.
	 */
	if (!xec_i2c_nl_tgt_registered(ctrl_data) &&
	    ((cfg & BIT(XEC_I2C_CFG_IDLE_IEN_POS)) != 0U) &&
	    ((cmpl & BIT(XEC_I2C_CMPL_IDLE_POS)) != 0U)) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x81U);
		sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
		xec_i2c_v3_cmpl_clear(rb, BIT(XEC_I2C_CMPL_IDLE_POS));
		xec_i2c_nl_clear_girqs(ctrl_cfg);


		xec_i2c_nl_req_done(ctrl_cfg, ctrl_data);
		return;
	}

	if (((cfg & BIT(XEC_I2C_CFG_HD_IEN_POS)) == 0U) ||
	    ((cmpl & BIT(XEC_I2C_CMPL_HDONE_POS)) == 0U)) {
		/* No enabled host source is active */
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x82U);
		if (!handled) {
			XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x83U);
			xec_i2c_nl_clear_girqs(ctrl_cfg);
		}
		return;
	}

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x84U);
	xec_i2c_v3_cmpl_clear(rb, cmpl & XEC_I2C_NL_CMPL_HOST_STS);
	xec_i2c_nl_clear_girqs(ctrl_cfg);
	hcmd = sys_read32(rb + XEC_I2C_HCMD_OFS);

	if ((cmpl & XEC_I2C_NL_CMPL_HOST_FATAL) != 0U) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x85U);
		ctrl_data->xfr_err = -EIO;
		ctrl_data->xfr_cmpl = cmpl;
		ctrl_data->xfr_reset = true;
	} else if ((cmpl & BIT(XEC_I2C_CMPL_HNAKX_POS)) != 0U) {
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x86U);
		ctrl_data->xfr_err = -ENXIO;
		ctrl_data->xfr_cmpl = cmpl;
	} else if ((hcmd & BIT(XEC_I2C_HCMD_PROC_POS)) != 0U) {
		/* HPROCEED=1 with HDONE is not a valid host state */
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x87U);
		ctrl_data->xfr_err = -EIO;
		ctrl_data->xfr_cmpl = cmpl;
		ctrl_data->xfr_reset = true;
	} else if ((hcmd & BIT(XEC_I2C_HCMD_RUN_POS)) != 0U) {
		/* Write to read turn around */
		XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x88U);
		rc = xec_i2c_nl_dma_start(ctrl_cfg, ctrl_data, true);
		if (rc == 0) {
			XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x89U);
			sys_set_bit(rb + XEC_I2C_HCMD_OFS, XEC_I2C_HCMD_PROC_POS);
			return;
		}
		ctrl_data->xfr_err = rc;
		ctrl_data->xfr_cmpl = cmpl;
		ctrl_data->xfr_reset = true;
	}

	xec_i2c_nl_isr_finish(ctrl_cfg, ctrl_data, hcmd);

	XEC_I2C_NL_STATE_CAP_UPDATE(ctrl_data, 0x8FU);
}

/* With asynchronous transfers the handler runs with interrupts locked so it can not
 * interleave with the request time-out handler. The completion callback runs after.
 */
static void xec_i2c_nl_isr(const struct device *ctrl_dev)
{
#ifdef CONFIG_I2C_CALLBACK
	unsigned int key = irq_lock();

	xec_i2c_nl_isr_handler(ctrl_dev);
	irq_unlock(key);

	xec_i2c_nl_async_notify(ctrl_dev->data);
#else
	xec_i2c_nl_isr_handler(ctrl_dev);
#endif
}

/* Controller driver initialization */
static int xec_i2c_nl_ctrl_init(const struct device *ctrl_dev)
{
	const struct xec_i2c_nl_config *ctrl_cfg = ctrl_dev->config;
	struct xec_i2c_nl_data *const ctrl_data = ctrl_dev->data;

	ctrl_data->controller = ctrl_dev;
	ctrl_data->active_port = XEC_I2C_NL_PORT_NONE;
	k_sem_init(&ctrl_data->lock, 1, 1);
	k_sem_init(&ctrl_data->xfr_done, 0, 1);

	if (!device_is_ready(ctrl_cfg->dma_dev)) {
		return -ENODEV;
	}

	xec_i2c_nl_dma_init(ctrl_cfg, ctrl_data);

#ifdef CONFIG_I2C_CALLBACK
	k_timer_init(&ctrl_data->async_timer, xec_i2c_nl_async_timeout, NULL);
#endif

#ifdef CONFIG_I2C_MCHP_XEC_NL_TGT_WDOG
	/* Started per transaction by the address match, stopped when the transaction is
	 * reported done, so nothing runs while the bus is quiet.
	 */
	k_timer_init(&ctrl_data->tgt_wdog, xec_i2c_nl_tgt_wdog_expiry, NULL);
#endif

	if (ctrl_cfg->irq_connect != NULL) {
		ctrl_cfg->irq_connect();
	}
	(void)soc_ecia_girq_ctrl(ctrl_cfg->girq, ctrl_cfg->girq_pos, 1U);

	return 0;
}

/* Port driver initialization */
static int xec_i2c_nl_port_init(const struct device *port_dev)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	struct xec_i2c_nl_port_data *port_data = port_dev->data;
	const struct device *ctrl_dev = port_cfg->controller;
	const struct xec_i2c_nl_config *ctrl_cfg = ctrl_dev->config;
	struct xec_i2c_nl_data *ctrl_data = ctrl_dev->data;

	/* Only the controller's default port is routed at init */
	if (!port_cfg->is_default) {
		return 0;
	}

	return xec_i2c_nl_apply_port(port_cfg, port_data, ctrl_cfg, ctrl_data);
}

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

/* Controller device instances */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT microchip_xec_i2c_nl

#define XEC_I2C_NL_DFLT_FREQ(inst) I2C_BITRATE_STANDARD

#define XEC_I2C_NL_GIRQ(inst, idx)     MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, idx))
#define XEC_I2C_NL_GIRQ_POS(inst, idx) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, idx))

#define XEC_I2C_NL_TIMING_FROM_DT(inst, prop)                                                      \
	{                                                                                          \
		.data_timing = DT_INST_PROP_BY_IDX(inst, prop, 0),                                 \
		.idle_scaling = DT_INST_PROP_BY_IDX(inst, prop, 1),                                \
		.timeout_scaling = DT_INST_PROP_BY_IDX(inst, prop, 2),                             \
		.bus_clock = DT_INST_PROP_BY_IDX(inst, prop, 3),                                   \
		.rpt_start_hold_tm = DT_INST_PROP_BY_IDX(inst, prop, 4),                           \
		.mr0 = DT_INST_PROP_BY_IDX(inst, prop, 5),                                         \
	}

#define XEC_I2C_NL_TIMING_DFLT(dtm, isc, tmo, bclk, rsht)                                          \
	{                                                                                          \
		.data_timing = (dtm), .idle_scaling = (isc), .timeout_scaling = (tmo),             \
		.bus_clock = (bclk), .rpt_start_hold_tm = (rsht),                                  \
		.mr0 = XEC_I2C_MR0_TM_BAUD16M,                                                     \
	}

#define XEC_I2C_NL_TIMING_ROW(inst, prop, dtm, isc, tmo, bclk, rsht)                               \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop),                                             \
		    (XEC_I2C_NL_TIMING_FROM_DT(inst, prop)),                                       \
		    (XEC_I2C_NL_TIMING_DFLT(dtm, isc, tmo, bclk, rsht)))

#define XEC_I2C_NL_TIMING_ROWS(inst)                                                               \
	{XEC_I2C_NL_TIMING_ROW(inst, timing_100k, XEC_I2C_SMB_DATA_TM_100K,                        \
			   XEC_I2C_SMB_IDLE_SC_100K, XEC_I2C_SMB_TMO_SC_100K,                      \
			   XEC_I2C_SMB_BUS_CLK_100K, XEC_I2C_SMB_RSHT_100K),                       \
	 XEC_I2C_NL_TIMING_ROW(inst, timing_400k, XEC_I2C_SMB_DATA_TM_400K,                        \
			   XEC_I2C_SMB_IDLE_SC_400K, XEC_I2C_SMB_TMO_SC_400K,                      \
			   XEC_I2C_SMB_BUS_CLK_400K, XEC_I2C_SMB_RSHT_400K),                       \
	 XEC_I2C_NL_TIMING_ROW(inst, timing_1000k, XEC_I2C_SMB_DATA_TM_1M,                         \
			   XEC_I2C_SMB_IDLE_SC_1M, XEC_I2C_SMB_TMO_SC_1M,                          \
			   XEC_I2C_SMB_BUS_CLK_1M, XEC_I2C_SMB_RSHT_1M),                           \
	 XEC_I2C_NL_TIMING_ROW(inst, timing_dt, XEC_I2C_SMB_DATA_TM_100K,                          \
			   XEC_I2C_SMB_IDLE_SC_100K, XEC_I2C_SMB_TMO_SC_100K,                      \
			   XEC_I2C_SMB_BUS_CLK_100K, XEC_I2C_SMB_RSHT_100K)}

#define XEC_I2C_NL_TIMING_ASSERT(inst, prop)                                                       \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, prop),                                              \
		(BUILD_ASSERT(DT_INST_PROP_LEN(inst, prop) == 6,                                   \
			      #prop " needs exactly 6 cells");                                     \
		 BUILD_ASSERT(DT_INST_PROP_BY_IDX(inst, prop, 3) <= 0xFFFF,                        \
			      #prop " bus-clock exceeds 16 bits");                                 \
		 BUILD_ASSERT(DT_INST_PROP_BY_IDX(inst, prop, 4) <= 0xFF,                          \
			      #prop " rpt-start-hold exceeds 8 bits");                             \
		 BUILD_ASSERT(DT_INST_PROP_BY_IDX(inst, prop, 5) <= 0xFF,                          \
			      #prop " mr0-timing exceeds 8 bits");))

#define XEC_I2C_NL_DEFPORT_ASSERT(inst)                                                            \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, default_port),                                      \
		   (BUILD_ASSERT(DT_SAME_NODE(DT_DRV_INST(inst),                                   \
			DT_PHANDLE(DT_INST_PHANDLE(inst, default_port), controller)),              \
			"default-port must reference a port on this controller");))

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
/* Target receive buffer: the largest external write plus the START address byte */
#define XEC_I2C_NL_TGT_BUF_DEFINE(inst)                                                            \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, target_buffer_size),                               \
		    (BUILD_ASSERT((DT_INST_PROP(inst, target_buffer_size) >= 2) &&                 \
				  (DT_INST_PROP(inst, target_buffer_size) <= 0xFFFF),              \
				  "target-buffer-size must be 2 to 65535");                        \
		     static uint8_t xec_i2c_nl_tgt_buf_##inst[DT_INST_PROP(inst,                   \
							       target_buffer_size)];),     \
		    ())

#define XEC_I2C_NL_TGT_CFG(inst)                                                                   \
	.has_tgt_dma = DT_INST_DMAS_HAS_NAME(inst, target),                                        \
	.dma_chan2 = COND_CODE_1(DT_INST_DMAS_HAS_NAME(inst, target),                              \
				 (DT_INST_DMAS_CELL_BY_NAME(inst, target, channel)), (0)),         \
	.dma_slot_tm = COND_CODE_1(DT_INST_DMAS_HAS_NAME(inst, target),                            \
				   (DT_INST_DMAS_CELL_BY_NAME(inst, target, trigsrc)), (0)),       \
	.tgt_buf_size = DT_INST_PROP_OR(inst, target_buffer_size, 0),                              \
	.tgt_buf = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, target_buffer_size),                    \
			       (xec_i2c_nl_tgt_buf_##inst), (NULL)),
#else
#define XEC_I2C_NL_TGT_BUF_DEFINE(inst)
#define XEC_I2C_NL_TGT_CFG(inst)
#endif

#define XEC_I2C_NL_INIT(inst) \
	XEC_I2C_NL_DEFPORT_ASSERT(inst)                                                            \
	XEC_I2C_NL_TGT_BUF_DEFINE(inst)                                                            \
	XEC_I2C_NL_TIMING_ASSERT(inst, timing_100k)                                                \
	XEC_I2C_NL_TIMING_ASSERT(inst, timing_400k)                                                \
	XEC_I2C_NL_TIMING_ASSERT(inst, timing_1000k)                                               \
	XEC_I2C_NL_TIMING_ASSERT(inst, timing_dt)                                                  \
	static void xec_i2c_nl_irq_connect_##inst(void) \
	{ \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), xec_i2c_nl_isr, \
			    DEVICE_DT_INST_GET(inst), 0); \
		irq_enable(DT_INST_IRQN(inst)); \
	}; \
	static const struct xec_i2c_nl_config xec_i2c_nl_cfg_##inst = { \
		.regbase = (uintptr_t)DT_INST_REG_ADDR(inst), \
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR(inst)), \
		.dma_chan1 = DT_INST_DMAS_CELL_BY_NAME(inst, host, channel), \
		.dma_slot_cm = DT_INST_DMAS_CELL_BY_NAME(inst, host, trigsrc), \
		XEC_I2C_NL_TGT_CFG(inst) \
		.enc_pcr = DT_INST_PROP(inst, pcr_scr), \
		.girq = XEC_I2C_NL_GIRQ(inst, 0), \
		.girq_pos = XEC_I2C_NL_GIRQ_POS(inst, 0), \
		.girq_wk = XEC_I2C_NL_GIRQ(inst, 1), \
		.girq_wk_pos = XEC_I2C_NL_GIRQ_POS(inst, 1), \
		.has_dt_timing = DT_INST_NODE_HAS_PROP(inst, timing_dt), \
		.dflt_freq = XEC_I2C_NL_DFLT_FREQ(inst), \
		.irq_connect = xec_i2c_nl_irq_connect_##inst, \
		.timing = XEC_I2C_NL_TIMING_ROWS(inst), }; \
	static struct xec_i2c_nl_data xec_i2c_nl_data_##inst; \
	DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_ctrl_init, NULL, \
			      &xec_i2c_nl_data_##inst, &xec_i2c_nl_cfg_##inst, \
			      POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_INIT)

/* Port device instances */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT microchip_xec_i2c_nl_port

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
	I2C_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_port_init, NULL,                                \
				  &xec_i2c_nl_port_xdat_##inst, &xec_i2c_nl_port_cfg_##inst,       \
				  POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,                           \
				  &xec_i2c_nl_port_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_PORT_INIT)

/* All enabled port devices. Used to find the port device, and its pin
 * configuration, for a physical port number on a given controller.
 */
#define XEC_I2C_NL_PORT_DEV(inst) DEVICE_DT_INST_GET(inst),

static const struct device *const xec_i2c_nl_port_devs[] = {
	DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_PORT_DEV)
};

static const struct device *xec_i2c_nl_find_port_dev(const struct device *ctrl_dev, uint8_t port)
{
	for (size_t i = 0; i < ARRAY_SIZE(xec_i2c_nl_port_devs); i++) {
		const struct xec_i2c_nl_port_config *port_cfg = xec_i2c_nl_port_devs[i]->config;

		if ((port_cfg->controller == ctrl_dev) && (port_cfg->port_id == port)) {
			return xec_i2c_nl_port_devs[i];
		}
	}

	return NULL;
}

/* Return the physical port the controller hardware is currently configured for */
int mchp_xec_i2c_nl_port_get(const struct device *i2c_port_dev, uint8_t *port)
{
	const struct xec_i2c_nl_port_config *port_cfg = NULL;
	const struct xec_i2c_nl_config *ctrl_cfg = NULL;
	struct xec_i2c_nl_data *ctrl_data = NULL;
	uint32_t cfg = 0;

	if ((i2c_port_dev == NULL) || (port == NULL)) {
		return -EINVAL;
	}

	port_cfg = i2c_port_dev->config;
	ctrl_cfg = port_cfg->controller->config;
	ctrl_data = port_cfg->controller->data;

	k_sem_take(&ctrl_data->lock, K_FOREVER);
	cfg = sys_read32(ctrl_cfg->regbase + XEC_I2C_CFG_OFS);
	k_sem_give(&ctrl_data->lock);

	*port = (uint8_t)XEC_I2C_CFG_PORT_GET(cfg);

	return 0;
}

/* Route the controller of i2c_port_dev to physical port. The port must belong to
 * an enabled port device on the same controller, which supplies the pin configuration
 * and bus frequency.
 */
int mchp_xec_i2c_nl_port_set(const struct device *i2c_port_dev, uint8_t port)
{
	const struct xec_i2c_nl_port_config *port_cfg = NULL;
	const struct device *ctrl_dev = NULL;
	const struct xec_i2c_nl_config *ctrl_cfg = NULL;
	struct xec_i2c_nl_data *ctrl_data = NULL;
	const struct device *new_port_dev = NULL;
	int rc = 0;

	if ((i2c_port_dev == NULL) || (port >= XEC_I2C_MAX_PORTS)) {
		return -EINVAL;
	}

	port_cfg = i2c_port_dev->config;
	ctrl_dev = port_cfg->controller;
	ctrl_cfg = ctrl_dev->config;
	ctrl_data = ctrl_dev->data;

	new_port_dev = xec_i2c_nl_find_port_dev(ctrl_dev, port);
	if (new_port_dev == NULL) {
		return -EINVAL;
	}

	k_sem_take(&ctrl_data->lock, K_FOREVER);

	if (xec_i2c_nl_is_busy(ctrl_cfg)) {
		rc = -EBUSY;
	} else if (ctrl_data->active_port != port) {
		rc = xec_i2c_nl_apply_port(new_port_dev->config, new_port_dev->data, ctrl_cfg,
					   ctrl_data);
	}

	k_sem_give(&ctrl_data->lock);

	return rc;
}
