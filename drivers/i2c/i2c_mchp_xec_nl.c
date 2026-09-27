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

/* Enable bit-bang live SCL/SDA monitoring and bit-bang mode is disable */
#define XEC_I2C_BBCR_LIVE_RD BIT(XEC_I2C_BBCR_CM_POS)

/* Completion register status bits of a host transaction */
#define XEC_I2C_NL_CMPL_HOST_STS                                                                   \
	(BIT(XEC_I2C_CMPL_HDONE_POS) | BIT(XEC_I2C_CMPL_HNAKX_POS) |                               \
	 BIT(XEC_I2C_CMPL_LAB_STS_POS) | BIT(XEC_I2C_CMPL_BER_STS_POS) |                           \
	 BIT(XEC_I2C_CMPL_CHDH_STS_POS) | BIT(XEC_I2C_CMPL_CHDL_STS_POS) |                         \
	 BIT(XEC_I2C_CMPL_TCTO_STS_POS) | BIT(XEC_I2C_CMPL_HCTO_STS_POS) |                         \
	 BIT(XEC_I2C_CMPL_DTS_STS_POS))

/* Completion register errors that end a host transaction and require a controller reset */
#define XEC_I2C_NL_CMPL_HOST_FATAL                                                                 \
	(BIT(XEC_I2C_CMPL_LAB_STS_POS) | BIT(XEC_I2C_CMPL_BER_STS_POS) |                           \
	 BIT(XEC_I2C_CMPL_TMO_STS_POS))

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
	uint8_t dma_chan2;
	uint8_t dma_slot_tm;
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

/* XEC I2C controller supports 7-bit I2C addressing only */
static inline bool xec_i2c_is_valid_address(uint16_t i2c_address)
{
	if ((i2c_address & ~0x7fU) != 0U) {
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
 * is empty.
 */
static void xec_i2c_nl_prep_hw(const struct xec_i2c_nl_config *ctrl_cfg)
{
	uintptr_t rb = ctrl_cfg->regbase;

	xec_i2c_v3_cmpl_clear(rb, XEC_I2C_NL_CMPL_HOST_STS | BIT(XEC_I2C_CMPL_IDLE_POS));
	sys_set_bits(rb + XEC_I2C_CFG_OFS, XEC_I2C_NL_CFG_FLUSH_HOST);
	xec_i2c_nl_clear_girqs(ctrl_cfg);
}

/* Program the transfer counts and start the host state machine */
static void xec_i2c_nl_start_hw(const struct xec_i2c_nl_config *ctrl_cfg,
				struct xec_i2c_nl_data *ctrl_data)
{
	uintptr_t rb = ctrl_cfg->regbase;
	struct i2c_xfer_desc *d = &ctrl_data->desc;
	uint32_t elen = sys_read32(rb + XEC_I2C_ELEN_OFS);
	uint32_t hcmd = 0;

	/* Counts are 16-bit: bits[7:0] in HCMD, bits[15:8] in the extended length register */
	d->elen = XEC_I2C_ELEN_HWR_SET(d->tx_count >> 8) | XEC_I2C_ELEN_HRD_SET(d->rx_count >> 8);
	elen &= ~(XEC_I2C_ELEN_HWR_MSK | XEC_I2C_ELEN_HRD_MSK);
	sys_write32(elen | d->elen, rb + XEC_I2C_ELEN_OFS);

	hcmd = XEC_I2C_HCMD_WCL_SET(d->tx_count & 0xffU);
	hcmd |= XEC_I2C_HCMD_RCL_SET(d->rx_count & 0xffU);
	hcmd |= d->ctrl | BIT(XEC_I2C_HCMD_PROC_POS) | BIT(XEC_I2C_HCMD_RUN_POS);

	sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_HD_IEN_POS);
	sys_write32(hcmd, rb + XEC_I2C_HCMD_OFS);
}

/* Start the request in ctrl_data->desc. Callable from ISR context. */
static int xec_i2c_nl_req_start(const struct xec_i2c_nl_config *ctrl_cfg,
				struct xec_i2c_nl_data *ctrl_data)
{
	int rc = 0;

	k_sem_reset(&ctrl_data->xfr_done);
	ctrl_data->xfr_err = 0;
	ctrl_data->xfr_cmpl = 0U;
	ctrl_data->xfr_reset = false;

	xec_i2c_nl_prep_hw(ctrl_cfg);

	rc = xec_i2c_nl_dma_start(ctrl_cfg, ctrl_data, false);
	if (rc != 0) {
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
		return rc;
	}

	rc = k_sem_take(&ctrl_data->xfr_done, I2C_TRANSFER_TIMEOUT);
	if (rc != 0) {
		LOG_ERR("I2C-NL transfer timeout (%d): addr 0x%02x", ctrl_data->xfr_err,
			ctrl_data->desc.addr);
		xec_i2c_nl_reset(ctrl_cfg, ctrl_data);
		/* An error seen before the time-out, e.g. NAK then no bus idle, is the cause */
		return (ctrl_data->xfr_err != 0) ? ctrl_data->xfr_err : -ETIMEDOUT;
	}

	if (ctrl_data->xfr_reset) {
		LOG_ERR("I2C-NL transfer error (%d): addr 0x%02x cmpl 0x%08x",
			ctrl_data->xfr_err, ctrl_data->desc.addr, ctrl_data->xfr_cmpl);
		xec_i2c_nl_reset(ctrl_cfg, ctrl_data);
	}

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

	k_sem_take(&ctrl_data->lock, K_FOREVER);

	rc = xec_i2c_nl_apply_port(port_cfg, port_data, ctrl_cfg, ctrl_data);
	if (rc != 0) {
		goto unlock;
	}

	/* Bus error or lost arbitration latched in the controller core */
	if ((sys_read8(ctrl_cfg->regbase + XEC_I2C_SR_OFS) & XEC_I2C_NL_SR_ERR) != 0U) {
		xec_i2c_nl_reset(ctrl_cfg, ctrl_data);
	}

	for (idx = 0; idx < num_msgs; idx += n) {
		n = xec_i2c_nl_req_len(&msgs[idx], num_msgs - idx);
		rc = xec_i2c_nl_xfer_parse(&msgs[idx], n, i2c_address, &ctrl_data->desc);
		if (rc != 0) {
			break;
		}

		rc = xec_i2c_nl_xfr_one(ctrl_cfg, ctrl_data);
		if (rc != 0) {
			break;
		}
	}

unlock:
	k_sem_give(&ctrl_data->lock);

	return rc;
}

#ifdef CONFIG_I2C_CALLBACK
/* End an asynchronous transfer. The callback is invoked by
 * xec_i2c_nl_async_notify() once interrupts are unlocked.
 */
static void xec_i2c_nl_async_end(struct xec_i2c_nl_data *ctrl_data, int result)
{
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

	if (k_sem_take(&ctrl_data->lock, K_NO_WAIT) != 0) {
		return -EWOULDBLOCK;
	}

	rc = xec_i2c_nl_apply_port(port_cfg, port_data, ctrl_cfg, ctrl_data);
	if (rc != 0) {
		goto unlock;
	}

	/* Bus error or lost arbitration latched in the controller core */
	if ((sys_read8(ctrl_cfg->regbase + XEC_I2C_SR_OFS) & XEC_I2C_NL_SR_ERR) != 0U) {
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
		return 0;
	}

	ctrl_data->xfr_async = false;
	(void)k_timer_stop(&ctrl_data->async_timer);

unlock:
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

	if (ctrl_data->xfr_err != 0) {
		sys_set_bits(rb + XEC_I2C_CFG_OFS, XEC_I2C_NL_CFG_FLUSH_HOST);
	}

	if (ctrl_data->xfr_reset || ((hcmd & BIT(XEC_I2C_HCMD_RUN_POS)) != 0U)) {
		ctrl_data->xfr_reset = true;
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
	uint32_t cfg = sys_read32(rb + XEC_I2C_CFG_OFS);
	uint32_t cmpl = sys_read32(rb + XEC_I2C_CMPL_OFS);
	uint32_t hcmd = 0;
	int rc = 0;

	/* Each path clears the I2C status, then the GIRQs, before enabling any new
	 * interrupt source (IDLE, HPROCEED, or the next request).
	 */
	if (((cfg & BIT(XEC_I2C_CFG_IDLE_IEN_POS)) != 0U) &&
	    ((cmpl & BIT(XEC_I2C_CMPL_IDLE_POS)) != 0U)) {
		sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
		xec_i2c_v3_cmpl_clear(rb, BIT(XEC_I2C_CMPL_IDLE_POS));
		xec_i2c_nl_clear_girqs(ctrl_cfg);
		xec_i2c_nl_req_done(ctrl_cfg, ctrl_data);
		return;
	}

	if (((cfg & BIT(XEC_I2C_CFG_HD_IEN_POS)) == 0U) ||
	    ((cmpl & BIT(XEC_I2C_CMPL_HDONE_POS)) == 0U)) {
		/* No enabled source is active */
		xec_i2c_nl_clear_girqs(ctrl_cfg);
		return;
	}

	xec_i2c_v3_cmpl_clear(rb, cmpl & XEC_I2C_NL_CMPL_HOST_STS);
	xec_i2c_nl_clear_girqs(ctrl_cfg);
	hcmd = sys_read32(rb + XEC_I2C_HCMD_OFS);

	if ((cmpl & XEC_I2C_NL_CMPL_HOST_FATAL) != 0U) {
		ctrl_data->xfr_err = -EIO;
		ctrl_data->xfr_cmpl = cmpl;
		ctrl_data->xfr_reset = true;
	} else if ((cmpl & BIT(XEC_I2C_CMPL_HNAKX_POS)) != 0U) {
		ctrl_data->xfr_err = -ENXIO;
		ctrl_data->xfr_cmpl = cmpl;
	} else if ((hcmd & BIT(XEC_I2C_HCMD_PROC_POS)) != 0U) {
		/* HPROCEED=1 with HDONE is not a valid host state */
		ctrl_data->xfr_err = -EIO;
		ctrl_data->xfr_cmpl = cmpl;
		ctrl_data->xfr_reset = true;
	} else if ((hcmd & BIT(XEC_I2C_HCMD_RUN_POS)) != 0U) {
		/* Write to read turn around */
		rc = xec_i2c_nl_dma_start(ctrl_cfg, ctrl_data, true);
		if (rc == 0) {
			sys_set_bit(rb + XEC_I2C_HCMD_OFS, XEC_I2C_HCMD_PROC_POS);
			return;
		}
		ctrl_data->xfr_err = rc;
		ctrl_data->xfr_cmpl = cmpl;
		ctrl_data->xfr_reset = true;
	}

	xec_i2c_nl_isr_finish(ctrl_cfg, ctrl_data, hcmd);
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

#define XEC_I2C_NL_INIT(inst) \
	XEC_I2C_NL_DEFPORT_ASSERT(inst)                                                            \
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
		.dma_chan2 = DT_INST_DMAS_CELL_BY_NAME(inst, target, channel), \
		.dma_slot_tm = DT_INST_DMAS_CELL_BY_NAME(inst, target, trigsrc), \
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
