/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip XEC I2Cv3 byte mode I2C driver.
 */

#define DT_DRV_COMPAT microchip_xec_i2c_v3

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

LOG_MODULE_REGISTER(i2c_mchp_xec_v3, CONFIG_I2C_LOG_LEVEL);

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
 * asserts TDONE on the host's STOP), so IDLE_IEN MUST be on across the
 * full lifetime of the armed target -- not just inside the read-pause
 * branch -- otherwise the target wedges with AAT=1, RUN=1, PROC=1 and
 * holds the bus. target_arm therefore enables IDLE_IEN itself, and
 * target_handle_stop carries a DMA-progress guard so the one-shot
 * spurious IRQ that the IEN-enable-while-NBB==1 bug may produce is a
 * no-op (consumed bytes == 0 -> early return).
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
#define XEC_I2C_V3_BB_HALF_PERIOD_US   5U
#define XEC_I2C_V3_BB_POLL_INTERVAL_US 1000U
#define XEC_I2C_V3_BB_SCL_POLL_LOOPS   10U
#define XEC_I2C_V3_BB_SDA_RECOV_LOOPS  10U
#define XEC_I2C_V3_BB_RECOV_CLOCKS     9U

/* WCL/RCL are 8 bits in HCMD; ELEN.HWR/HRD extend each by another 8 bits. */
#define XEC_I2C_V3_LEN_MAX 0xFFFFU

/* I2C-bus R/W bit, the LSB of the address byte. 0 = write, 1 = read. */
#define XEC_I2C_V3_RWBIT_WRITE 0x00U
#define XEC_I2C_V3_RWBIT_READ  0x01U

#define XEC_I2C_V3_INVALID_PORT 0xFFU

#define XEC_I2C_V3_TIMEOUT K_MSEC(1000)

/* Default I2C control-register value: ESO+ACK+PIN. PIN is also raised at
 * reset to clear any latent PIN-asserted state in the legacy I2C engine.
 */
#define XEC_I2C_V3_CR_NO_EIN_DFLT                                                                  \
	(BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS) | BIT(XEC_I2C_CR_PIN_POS))

#define XEC_I2C_V3_CR_DFLT (XEC_I2C_CR_NO_EIN_DFLT | BIT(XEC_I2C_CR_EIN_POS))

enum xec_i2c_nl_state {
	XEC_I2C_V3_IDLE,
	XEC_I2C_V3_TX,
	XEC_I2C_V3_RX,
	XEC_I2C_V3_WAIT_IDLE,
	XEC_I2C_V3_ERROR,
};

enum xec_i2c_nl_mode {
	XEC_I2C_V3_MODE_CONTROLLER,
	XEC_I2C_V3_MODE_TARGET,
};

/* Controller bus-clock and timing rows for a 16 MHz BAUD clock. Source:
 * Microchip I2C-SMBus controller v3.8 datasheet.
 */
struct xec_i2c_v3_timing {
	uint32_t data_timing;
	uint32_t idle_scaling;
	uint32_t timeout_scaling;
	uint16_t bus_clock;
	uint8_t mr1;
};

static const struct xec_i2c_v3_timing xec_i2c_v3_timing_tbl[] = {
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

struct xec_i2c_v3_config {
	uintptr_t base;
	void (*irq_connect)(void);
	uint32_t dflt_freq;
	uint8_t girq;
	uint8_t girq_pos;
	uint16_t enc_pcr;
};

struct xec_i2c_v3_ir_data {
	volatile uint32_t hcmd;
	volatile uint32_t tcmd;
	volatile uint32_t cmpl;
	volatile uint32_t cfg;
};

struct xec_i2c_v3_data {
	const struct device *ctrl;
	struct k_sem lock;
	struct k_sem pause_sem;
	struct k_sem done_sem;

	enum xec_i2c_v3_state state;
	int xfer_err;

	uint8_t active_port; /* XEC_I2C_NL_INVALID_PORT until programmed */
	uint32_t active_freq;

#ifdef CONFIG_I2C_MCHP_XEC_V3_STATE_CAPTURE
	volatile uint32_t capidx;
	volatile uint8_t capture[CONFIG_I2C_MCHP_XEC_V3_STATE_CAPTURE_SIZE];
	volatile uint32_t iridx;
	struct xec_i2c_nl_ir_data ir_data[256];
#endif
};

struct xec_i2c_v3_port_config {
	const struct device *parent;
	const struct pinctrl_dev_config *pcfg;
	uint32_t bitrate;
	uint8_t port_id;
	bool is_default;
};

/* Parsed summary of an i2c_transfer() request after flag/shape validation.
 * Filled by xec_i2c_nl_parse and consumed by xec_i2c_nl_run.
 */
struct xec_i2c_v3_xfer {
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

#ifdef CONFIG_I2C_MCHP_XEC_V3_STATE_CAPTURE
static void xec_i2c_v3_cap_init(struct xec_i2c_v3_data *xdat)
{
	xdat->capidx = 0;
	memset((void *)xdat->capture, 0, CONFIG_I2C_MCHP_XEC_V3_STATE_CAPTURE_SIZE);

	xdat->iridx = 0;
	memset((void *)xdat->ir_data, 0, 256U * sizeof(struct xec_i2c_nl_ir_data));
}

static void xec_i2c_v3_cap_update(struct xec_i2c_v3_data *xdat, uint8_t capval)
{
	if (xdat->capidx >= CONFIG_I2C_MCHP_XEC_V3_STATE_CAPTURE_SIZE) {
		return;
	}

	xdat->capture[xdat->capidx++] = capval;
}

static void xec_i2c_v3_ir_update(struct xec_i2c_v3_data *xdat)
{
	if (xdat->iridx >= 256U) {
		return;
	}

	const struct device *ctrl = xdat->ctrl;
	const struct xec_i2c_v3_config *xcfg = ctrl->config;
	mm_reg_t rb = xcfg->base;
	uint32_t idx = xdat->iridx;

	xdat->iridx++;

	xdat->ir_data[idx].hcmd = sys_read32(rb + XEC_I2C_HCMD_OFS);
	xdat->ir_data[idx].tcmd = sys_read32(rb + XEC_I2C_TCMD_OFS);
	xdat->ir_data[idx].cmpl = ((sys_read32(rb + XEC_I2C_CMPL_OFS) & 0xffffff00U) |
				   sys_read8(rb + XEC_I2C_SR_OFS));
	xdat->ir_data[idx].cfg = sys_read32(rb + XEC_I2C_CFG_OFS);
}

int mchp_xec_i2c_v3_clear_capture(const struct device *i2c_v3_dev)
{
	if (i2c_v3_dev == NULL) {
		return -EINVAL;
	}

	struct xec_i2c_v3_data *const xdat = i2c_v3_dev->data;

	if (xdat->state != XEC_I2C_NL_IDLE) {
		return -EBUSY;
	}

	k_sem_take(&xdat->lock, K_FOREVER);
	xec_i2c_v3_cap_init(xdat);
	k_sem_give(&xdat->lock);

	return 0;
}

int mchp_xec_i2c_v3_copy_capture(const struct device *i2c_v3_dev, uint8_t *capdest,
				 size_t capdest_size)
{
	if ((i2c_v3_dev == NULL) || (capdest == NULL)) {
		return -EINVAL;
	}

	if (capdest_size == 0) {
		return 0;
	}

	struct xec_i2c_v3_data *const xdat = i2c_v3_dev->data;

	if (xdat->state != XEC_I2C_NL_IDLE) {
		return -EBUSY;
	}

	k_sem_take(&xdat->lock, K_FOREVER);

	size_t n = (capdest_size < CONFIG_I2C_MCHP_XEC_V3_STATE_CAPTURE_SIZE)
			   ? capdest_size
			   : CONFIG_I2C_MCHP_XEC_V3_STATE_CAPTURE_SIZE;

	memcpy(capdest, (const void *)xdat->capture, n);

	k_sem_give(&xdat->lock);

	return 0;
}
#else
static void xec_i2c_v3_cap_init(struct xec_i2c_v3_data *xdat)
{
}

static void xec_i2c_v3_cap_update(struct xec_i2c_v3_data *xdat, uint8_t capval)
{
}

static void xec_i2c_v3_ir_update(struct xec_i2c_v3_data *xdat)
{
}

int mchp_xec_i2c_v3_clear_capture(const struct device *i2c_v3_dev)
{
	return -ENOSYS;
}

int mchp_xec_i2c_v3_copy_capture(const struct device *i2c_v3_dev, uint8_t *capdest,
				 size_t capdest_size)
{
	return -ENOSYS;
}
#endif /* CONFIG_I2C_MCHP_XEC_V3_STATE_CAPTURE */

static const struct xec_i2c_v3_timing *xec_i2c_v3_timing_for(uint32_t freqhz)
{
	if (freqhz <= KHZ(100)) {
		return &xec_i2c_v3_timing_tbl[0];
	}
	if (freqhz <= KHZ(400)) {
		return &xec_i2c_v3_timing_tbl[1];
	}
	return &xec_i2c_v3_timing_tbl[2];
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
static int xec_i2c_v3_program_ctrl(const struct device *ctrl, uint32_t freqhz, uint8_t port)
{
	const struct xec_i2c_v3_config *cfg = ctrl->config;
	struct xec_i2c_v3_data *data = ctrl->data;
	const struct xec_i2c_v3_timing *tm = xec_i2c_v3_timing_for(freqhz);
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
	sys_write32(CMPL_HDONE | CMPL_IDLE | CMPL_ERR, base + XEC_I2C_CMPL_OFS);

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
static int xec_i2c_v3_apply_port(const struct device *port_dev)
{
	const struct xec_i2c_v3_port_config *pc = port_dev->config;
	const struct device *ctrl = pc->parent;
	const struct xec_i2c_v3_config *cfg = ctrl->config;
	struct xec_i2c_v3_data *data = ctrl->data;
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

	return xec_i2c_v3_program_ctrl(ctrl, freq, pc->port_id);
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
static void xec_i2c_v3_abort(const struct device *ctrl)
{
	const struct xec_i2c_v3_config *cfg = ctrl->config;
	struct xec_i2c_v3_data *data = ctrl->data;
	uint32_t freq = (data->active_freq != 0U) ? data->active_freq : cfg->dflt_freq;
	uint8_t port = (data->active_port == XEC_I2C_NL_INVALID_PORT) ? 0U : data->active_port;

	(void)xec_i2c_v3_program_ctrl(ctrl, freq, port);

	data->active_port = XEC_I2C_NL_INVALID_PORT;
}

/* Drive XEC_I2C_NL_BB_RECOV_CLOCKS SCL pulses at ~100 kHz while leaving
 * SDA released. Caller must already have engaged bit-bang mode (BBCR
 * set to BBCR_BB_RELEASED).
 */
static void xec_i2c_v3_bb_clock_burst(mm_reg_t base)
{
	for (uint32_t i = 0; i < XEC_I2C_V3_BB_RECOV_CLOCKS; i++) {
		sys_write8(BBCR_BB_SCL_LOW, base + XEC_I2C_BBCR_OFS);
		k_busy_wait(XEC_I2C_V3_BB_HALF_PERIOD_US);
		sys_write8(BBCR_BB_RELEASED, base + XEC_I2C_BBCR_OFS);
		k_busy_wait(XEC_I2C_V3_BB_HALF_PERIOD_US);
	}
}

/* Generate an I2C STOP condition: SDA low -> high while SCL stays high.
 * Caller must already be in bit-bang mode with SCL released.
 */
static void xec_i2c_v3_bb_stop(mm_reg_t base)
{
	sys_write8(BBCR_BB_SDA_LOW, base + XEC_I2C_BBCR_OFS);
	k_busy_wait(XEC_I2C_V3_BB_HALF_PERIOD_US);
	sys_write8(BBCR_BB_RELEASED, base + XEC_I2C_BBCR_OFS);
	k_busy_wait(XEC_I2C_V3_BB_HALF_PERIOD_US);
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
static int xec_i2c_v3_bus_recover(const struct device *ctrl, uint32_t freq, uint8_t port)
{
	const struct xec_i2c_v3_config *cfg = ctrl->config;
	uintptr_t base = cfg->base;
	uint8_t bbcr = 0;
	int rc = 0;

	rc = xec_i2c_v3_program_ctrl(ctrl, freq, port);
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
		k_busy_wait(XEC_I2C_V3_BB_POLL_INTERVAL_US);
	}
	if ((bbcr & BBCR_SCL_IN) == 0U) {
		LOG_ERR("i2c-recover: SCL stuck low");
		return -EIO;
	}

	if ((bbcr & BBCR_SDA_IN) == 0U) {
		sys_write8(BBCR_BB_RELEASED, base + XEC_I2C_BBCR_OFS);
		k_busy_wait(XEC_I2C_V3_BB_HALF_PERIOD_US);

		for (uint32_t i = 0; i < XEC_I2C_V3_BB_SDA_RECOV_LOOPS; i++) {
			xec_i2c_v3_bb_clock_burst(base);
			xec_i2c_v3_bb_stop(base);
			bbcr = sys_read8(base + XEC_I2C_BBCR_OFS);
			if ((bbcr & BBCR_SDA_IN) != 0U) {
				break;
			}
		}
	}

	/* Return pins to I2C control with live readback still on. */
	sys_write8(BBCR_LIVE_RD, base + XEC_I2C_BBCR_OFS);

	(void)xec_i2c_v3_program_ctrl(ctrl, freq, port);

	bbcr = sys_read8(base + XEC_I2C_BBCR_OFS);
	if ((bbcr & (BBCR_SCL_IN | BBCR_SDA_IN)) != (BBCR_SCL_IN | BBCR_SDA_IN)) {
		LOG_ERR("i2c-recover: SCL=%u SDA=%u still not both high",
			(bbcr & BBCR_SCL_IN) ? 1U : 0U, (bbcr & BBCR_SDA_IN) ? 1U : 0U);
		return -EIO;
	}

	return 0;
}

/* I2C controller ISR */
static void xec_i2c_v3_isr(const struct device *ctrl)
{
	const struct xec_i2c_v3_config *cfg = ctrl->config;
	struct xec_i2c_v3_data *data = ctrl->data;
	uintptr_t base = cfg->base;
	uint32_t cmpl = sys_read32(base + XEC_I2C_CMPL_OFS);
	uint32_t cfgr = 0, hcmd = 0;

	xec_i2c_v3_cap_update(data, 0x80U);
	xec_i2c_v3_ir_update(data);

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
		sys_write32(CMPL_ERR | CMPL_HDONE | CMPL_IDLE, base + XEC_I2C_CMPL_OFS);
		dma_stop(cfg->dma_dev, cfg->dma_chan);
		k_sem_give(&data->pause_sem);
		k_sem_give(&data->done_sem);
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
		sys_write32(CMPL_HDONE, base + XEC_I2C_CMPL_OFS);

		if (data->state == XEC_I2C_NL_TX || data->state == XEC_I2C_NL_RX) {
			xec_i2c_nl_cap_update(data, 0x86U);

			hcmd = sys_read32(base + XEC_I2C_HCMD_OFS);

			if ((hcmd & HCMD_RUN) != 0U && (hcmd & HCMD_PROCEED) == 0U) {
				/* PAUSE — driver thread will reprogram DMA
				 * for the read phase and resume by setting
				 * HCMD.PROCEED.
				 */
				xec_i2c_nl_cap_update(data, 0x87U);
				k_sem_give(&data->pause_sem);
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
			sys_write32(CMPL_IDLE, base + XEC_I2C_CMPL_OFS);

			if (data->state != XEC_I2C_NL_IDLE) {
				xec_i2c_nl_cap_update(data, 0x8BU);
				k_sem_give(&data->done_sem);
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
static int xec_i2c_v3_parse(const struct xec_i2c_nl_config *cfg, struct i2c_msg *msgs,
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

#if 0
static int xec_i2c_v3_run(const struct device *ctrl, uint16_t addr,
			  const struct xec_i2c_v3_xfer *xfer)
{
	rc = k_sem_take(&data->done_sem, XEC_I2C_V3_TIMEOUT);
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

	data->state = XEC_I2C_V3_IDLE;

	xec_i2c_nl_cap_update(data, 0x2FU);

	return 0;
}
#endif

/* Zephyr i2c_driver_api */

/* Support synchronous only
 * Loop over msgs
 *   if msg->buf == NULL or msg->len == 0 then return -EINVAL
 *   if msg->flags has I2C_MSG_ADDR_10_BITS flag set then return -ENOTSUP
 * end loop
 */
static int xec_i2c_v3_vport_transfer(const struct device *port_dev, struct i2c_msg *msgs,
				     uint8_t num_msgs, uint16_t addr)
{
	const struct xec_i2c_v3_port_config *pc = port_dev->config;
	const struct device *ctrl = pc->parent;
	const struct xec_i2c_v3_config *cfg = ctrl->config;
	struct xec_i2c_v3_data *data = ctrl->data;
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

	xec_i2c_v3_cap_init(data);

	xec_i2c_v3_cap_update(data, 1U);

	rc = xec_i2c_v3_apply_port(port_dev);
	if (rc != 0) {
		xec_i2c_v3_cap_update(data, 2U);
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

		xec_i2c_v3_cap_update(data, 3U);
		rc = xec_i2c_v3_bus_recover(ctrl, freq, pc->port_id);
		if (rc != 0) {
			xec_i2c_v3_cap_update(data, 4U);
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
	xec_i2c_v3_cap_update(data, 5U);

	group_start = 0;
	while (group_start < num_msgs) {
		group_end = group_start;

		while (group_end < (uint8_t)(num_msgs - 1U) &&
		       (msgs[group_end].flags & I2C_MSG_STOP) == 0U) {
			group_end++;
		}

		struct xec_i2c_nl_xfer xfer;

		group_len = (uint8_t)((group_end - group_start) + 1U);

		rc = xec_i2c_v3_parse(cfg, &msgs[group_start], group_len, &xfer);
		if (rc != 0) {
			xec_i2c_v3_cap_update(data, 6U);
			break;
		}

		rc = xec_i2c_v3_run(ctrl, addr, &xfer);
		if (rc != 0) {
			xec_i2c_v3_cap_update(data, 7U);
			break;
		}

		group_start = (uint8_t)(group_end + 1U);
	}

	xec_i2c_v3_cap_update(data, 8U);

	k_sem_give(&data->lock);

	return rc;
}

static int xec_i2c_v3_vport_configure(const struct device *port_dev, uint32_t dev_config)
{
	const struct xec_i2c_v3_port_config *pc = port_dev->config;
	const struct device *ctrl = pc->parent;
	const struct xec_i2c_v3_config *cfg = ctrl->config;
	struct xec_i2c_v3_data *data = ctrl->data;
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
			rc = xec_i2c_v3_program_ctrl(ctrl, freq, pc->port_id);
		}
	}

	k_sem_give(&data->lock);

	return rc;
}

static int xec_i2c_v3_vport_get_config(const struct device *port_dev, uint32_t *dev_config)
{
	const struct xec_i2c_v3_port_config *pc = port_dev->config;
	struct xec_i2c_v3_data *data = pc->parent->data;
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

static int xec_i2c_v3_vport_recover_bus(const struct device *port_dev)
{
	const struct xec_i2c_v3_port_config *pc = port_dev->config;
	const struct device *ctrl = pc->parent;
	const struct xec_i2c_v3_config *cfg = ctrl->config;
	struct xec_i2c_v3_data *data = ctrl->data;
	uint32_t freq = (data->active_freq != 0U) ? data->active_freq : cfg->dflt_freq;
	int rc = 0;

	k_sem_take(&data->lock, K_FOREVER);

	rc = xec_i2c_v3_apply_port(port_dev);
	if (rc != 0) {
		k_sem_give(&data->lock);
		return rc;
	}

	rc = xec_i2c_v3_bus_recover(ctrl, freq, pc->port_id);

	k_sem_give(&data->lock);

	return rc;
}

static DEVICE_API(i2c, xec_i2c_v3_port_api) = {
	.configure = xec_i2c_v3_vport_configure,
	.get_config = xec_i2c_v3_vport_get_config,
	.transfer = xec_i2c_v3_vport_transfer,
	.recover_bus = xec_i2c_v3_vport_recover_bus,
#ifdef CONFIG_I2C_RTIO
	/* This driver does not support callback/async I2C due to the overhead of parsing
	 * messages to fit the I2C-NL hardware. We use the default RTIO work queue which dispatches
	 * each SQE as a synchronous i2c_transfer.
	 */
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

/* Custom mchp_xec_i2c.h API for runtime port query/select */

int mchp_xec_i2c_v3_port_get(const struct device *i2c_dev, uint8_t *port)
{
	const struct xec_i2c_v3_port_config *pc;

	if (i2c_dev == NULL || port == NULL) {
		return -EINVAL;
	}

	pc = i2c_dev->config;
	*port = pc->port_id;
	return 0;
}

int mchp_xec_i2c_v3_port_set(const struct device *i2c_dev, uint8_t port)
{
	const struct xec_i2c_v3_port_config *pc;
	const struct xec_i2c_v3_config *cfg;
	struct xec_i2c_v3_data *data;
	uint32_t freq = 0;
	int rc = 0;

	if (i2c_dev == NULL || port >= XEC_I2C_CFG_MAX_PORT) {
		return -EINVAL;
	}

	pc = i2c_dev->config;
	cfg = pc->parent->config;
	data = pc->parent->data;

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

static int xec_i2c_v3_ctrl_init(const struct device *ctrl)
{
	const struct xec_i2c_v3_config *cfg = ctrl->config;
	struct xec_i2c_v3_data *data = ctrl->data;
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

static int xec_i2c_v3_port_init(const struct device *port_dev)
{
	const struct xec_i2c_v3_port_config *pc = port_dev->config;

	if (!device_is_ready(pc->parent)) {
		return -ENODEV;
	}

	if (pc->is_default) {
		int rc = xec_i2c_v3_apply_port(port_dev);

		if (rc != 0) {
			return rc;
		}
	}

	return 0;
}

/* Devicetree instantiation */

#define XEC_I2C_GIRQ(inst)     MCHP_XEC_ECIA_GIRQ(DT_INST_PROP(inst, girqs))
#define XEC_I2C_GIRQ_POS(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP(inst, girqs))

/* The controller binding does not carry clock-frequency — it lives on the
 * port nodes. The controller's default frequency is only the value the
 * controller boots up at; vport_configure() reprograms the bus rate per
 * port-device.
 */
#define XEC_I2C_DFLT_FREQ(inst) I2C_BITRATE_STANDARD

#define XEC_I2C_V3_CTRL_INIT(inst)                                                                 \
	static void xec_i2c_v3_irq_connect_##inst(void)                                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), xec_i2c_v3_isr,       \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
	static const struct xec_i2c_v3_config xec_i2c_v3_cfg_##inst = {                            \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.irq_connect = xec_i2c_v3_irq_connect_##inst,                                      \
		.dflt_freq = XEC_I2C_NL_DFLT_FREQ(inst),                                           \
		.girq = XEC_I2C_NL_GIRQ(inst),                                                     \
		.girq_pos = XEC_I2C_NL_GIRQ_POS(inst),                                             \
		.enc_pcr = DT_INST_PROP(inst, pcr_scr),                                            \
	};                                                                                         \
	static struct xec_i2c_v3_data xec_i2c_v3_data_##inst;                                      \
	DEVICE_DT_INST_DEFINE(inst, xec_i2c_v3_ctrl_init, NULL, &xec_i2c_v3_data_##inst,           \
			      &xec_i2c_v3_cfg_##inst, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,       \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_V3_CTRL_INIT)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT microchip_xec_i2c_v3_port

#define XEC_I2C_V3_PORT_INIT(inst)                                                                 \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static const struct xec_i2c_v3_port_config xec_i2c_v3_port_cfg_##inst = {                  \
		.parent = DEVICE_DT_GET(DT_INST_PHANDLE(inst, controller)),                        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.bitrate = DT_INST_PROP_OR(inst, clock_frequency, I2C_BITRATE_STANDARD),           \
		.port_id = (uint8_t)(DT_INST_PROP(inst, port) & 0x0FU),                            \
		.is_default = DT_INST_PROP(inst, default_port),                                    \
	};                                                                                         \
	I2C_DEVICE_DT_INST_DEFINE(                                                                 \
		inst, xec_i2c_v3_port_init, NULL, NULL, &xec_i2c_v3_port_cfg_##inst, POST_KERNEL,  \
		CONFIG_I2C_MCHP_XEC_V3_PORT_INIT_PRIORITY, &xec_i2c_v3_port_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_V3_PORT_INIT)
