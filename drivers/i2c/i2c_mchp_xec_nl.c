/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip XEC version 3.8 I2C HW Network-Layer (NL) I2C driver.
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

/* Default I2C control-register value: ESO+ACK+PIN. PIN is also raised at
 * reset to clear any latent PIN-asserted state in the legacy I2C engine.
 */
#define XEC_I2C_NL_CR_DFLT                                                                         \
	(BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS) | BIT(XEC_I2C_CR_PIN_POS))

#define CMPL_HDONE BIT(XEC_I2C_CMPL_HDONE_POS)
#define CMPL_IDLE  BIT(XEC_I2C_CMPL_IDLE_POS)
#define CMPL_HNAK  BIT(XEC_I2C_CMPL_HNAKX_POS)
#define CMPL_LAB   BIT(XEC_I2C_CMPL_LAB_STS_POS)
#define CMPL_BER   BIT(XEC_I2C_CMPL_BER_STS_POS)
#define CMPL_ERR   (CMPL_HNAK | CMPL_LAB | CMPL_BER)

/* Master-mode command register live bits (see README/CLAUDE.md HW model). */
#define HCMD_RUN     BIT(XEC_I2C_HCMD_RUN_POS)
#define HCMD_PROCEED BIT(XEC_I2C_HCMD_PROC_POS)
#define HCMD_START0  BIT(XEC_I2C_HCMD_START0_POS)
#define HCMD_STARTN  BIT(XEC_I2C_HCMD_STARTN_POS)
#define HCMD_STOP    BIT(XEC_I2C_HCMD_STOP_POS)

/* v3.8 erratum bit: must only be armed while the bus is NOT already idle
 * (see the ISR and xec_i2c_nl_program_ctrl()'s CFG write for why it is
 * otherwise left off).
 */
#define CFG_IDLE_IEN BIT(XEC_I2C_CFG_IDLE_IEN_POS)

#ifdef CONFIG_I2C_TARGET
/* Target-mode command register (same shape as HCMD, different bit
 * layout -- see i2c_mchp_xec_regs.h, do not reuse HCMD's WCL/RCL macros).
 */
#define TCMD_RUN     BIT(XEC_I2C_TCMD_RUN_POS)
#define TCMD_PROCEED BIT(XEC_I2C_TCMD_PROC_POS)

/* CFG bits controlling which mode's completion interrupt is enabled, and
 * the address-match interrupt that starts a target-mode transaction.
 */
#define CFG_HD_IEN  BIT(XEC_I2C_CFG_HD_IEN_POS)
#define CFG_TD_IEN  BIT(XEC_I2C_CFG_TD_IEN_POS)
#define CFG_AAT_IEN BIT(XEC_I2C_CFG_AAT_IEN_POS)

/* Target-mode completion bits (same CMPL register controller mode uses). */
#define CMPL_TDONE      BIT(XEC_I2C_CMPL_TDONE_POS)
#define CMPL_TPROT      BIT(XEC_I2C_CMPL_TPROT_POS)
#define CMPL_RPT_RD     BIT(XEC_I2C_CMPL_RPT_RD_POS)
#define CMPL_RPT_WR     BIT(XEC_I2C_CMPL_RPT_WR_POS)
#define CMPL_TNAKR_STS  BIT(XEC_I2C_CMPL_TNAKR_STS_POS)
#define CMPL_TGT_CLEAR  (CMPL_TDONE | CMPL_IDLE | CMPL_TPROT | CMPL_RPT_RD | CMPL_RPT_WR | \
			 CMPL_TNAKR_STS)
#endif

#define BBCR_SCL_IN BIT(XEC_I2C_BBCR_SCL_IN_POS)
#define BBCR_SDA_IN BIT(XEC_I2C_BBCR_SDA_IN_POS)

#define BBCR_LIVE_RD     BIT(XEC_I2C_BBCR_CM_POS)
#define BBCR_BB_RELEASED BIT(XEC_I2C_BBCR_EN_POS) /* BBM_EN=1, both dirs=input, both released */
/* BBM_EN=1, SCL drive-low, SDA released */
#define BBCR_BB_SCL_LOW  (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CD_POS))
/* BBM_EN=1, SDA drive-low, SCL released */
#define BBCR_BB_SDA_LOW  (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_DD_POS)) 

/* struct i2c_msg[] -> HW request parsing; kept SoC-independent, see the
 * file. Included here, ahead of struct xec_i2c_nl_data, because that
 * struct embeds a struct xec_i2c_nl_request by value.
 */
#include "i2c_mchp_xec_nl_msg.c"

enum xec_i2c_nl_timing_row {
	XEC_I2C_NL_TM_100K,
	XEC_I2C_NL_TM_400K,
	XEC_I2C_NL_TM_1M,
	XEC_I2C_NL_TM_DT,
	XEC_I2C_NL_TM_COUNT,
};

struct xec_i2c_nl_timing {
	uint32_t data_timing;
	uint32_t idle_scaling;
	uint32_t timeout_scaling;
	uint16_t bus_clock;
	uint8_t rpt_start_hold_tm;
	uint8_t mr1;
};

/* Controller device structure */
struct xec_i2c_nl_config {
	uintptr_t base;
	const struct device *dma_dev;
	void (*irq_connect)(void);
	uint32_t dflt_freq;
	uint8_t cm_dma_chan;
	uint8_t cm_dma_slot;
	uint16_t enc_pcr;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t girq_wk;
	uint8_t girq_wk_pos;
	bool has_dt_timing;
#ifdef CONFIG_I2C_CALLBACK
	/* TODO */
#endif
#ifdef CONFIG_I2C_TARGET
	uint8_t *tm_rx_buf;      /* NULL when DT has no target-buffer-size (-ENOSYS at register) */
	uint16_t tm_rx_buf_size; /* 0 in that case; else <= 65535 (build-asserted) */
	uint8_t tm_dma_chan;     /* meaningful only when tm_dma_valid */
	uint8_t tm_dma_slot;
	bool tm_dma_valid;       /* true iff DT dmas[] has a "target"-named entry (else -ENODEV) */
	bool wakeup_source;
#endif
	struct xec_i2c_nl_timing timing[XEC_I2C_NL_TM_COUNT];
};

/* One DMA source/dest segment of the currently-armed HW request's write or
 * read phase -- see the segment model in the Task 3 design notes. A single
 * request's write phase is NOT one contiguous DMA buffer (this HW has no
 * scatter-gather): it is the address byte, then each write message's own
 * (separately allocated) buffer, then optionally a second address byte.
 * DMA must be re-armed at every one of these boundaries.
 */
enum xec_i2c_nl_seg_kind {
	XEC_I2C_NL_SEG_ADDR0, /* data->addr_byte[0]: write-phase START0 addr byte */
	XEC_I2C_NL_SEG_WMSG,  /* msgs[cur_seg_msg_idx].buf: a write-phase message */
	XEC_I2C_NL_SEG_ADDRN, /* data->addr_byte[1]: write-phase STARTN addr byte */
	XEC_I2C_NL_SEG_RMSG,  /* msgs[cur_seg_msg_idx].buf: a read-phase message  */
};

#ifdef CONFIG_I2C_TARGET
enum xec_i2c_nl_tgt_phase {
	XEC_I2C_NL_TGT_IDLE, /* armed; DMA in RX config, waiting for AAT/TDONE */
	XEC_I2C_NL_TGT_TX,   /* handle_read_pause reconfigured the channel for host-read TX */
};

/* Target-mode state: tracked separately from the (already large)
 * controller-mode transfer state above so target-mode helpers can take a
 * single struct xec_i2c_nl_target_data * rather than more flat fields.
 * count != 0 IS "in target mode" -- deliberately no separate mode enum,
 * so there's no second value that could drift out of sync with slots[].
 */
struct xec_i2c_nl_target_data {
	struct i2c_target_config *slots[XEC_I2C_OA_NUM_TARGETS]; /* one per OA slot */
	uint8_t count; /* # populated slots; 0 == controller mode */
	enum xec_i2c_nl_tgt_phase phase;
};
#endif

/* Controller data structure */
struct xec_i2c_nl_data {
	const struct device *ctrl;
	uint8_t addr_byte[2] __aligned(4);
	uint32_t active_freq;
	uint8_t active_port;

	/* Busy guard (not a mutex: completion, and thus the final unlock,
	 * happens from ISR context for the async path, and Zephyr mutexes
	 * cannot be unlocked from ISR). Sync xfr takes it K_FOREVER; async
	 * xfr_cb takes it K_NO_WAIT and returns -EWOULDBLOCK on failure.
	 */
	struct k_sem lock;
	struct k_sem done_sem; /* sync callers block on this for the whole xfr */

	/* In-flight transfer: the caller's original submission, valid for as
	 * long as `lock` is held by a transfer.
	 */
	struct i2c_msg *xfr_msgs;
	uint8_t xfr_num_msgs;
	uint16_t xfr_i2c_addr;
	const struct device *xfr_port_dev; /* diagnostics only */
	bool xfr_active;                   /* guards finish_xfr() re-entry */

	/* Currently-armed HW request, and where to resume parsing for the
	 * next one (== xfr_num_msgs means "nothing left").
	 */
	struct xec_i2c_nl_request cur_req;
	uint8_t next_start_idx;

	/* DMA segment cursor within cur_req. */
	enum xec_i2c_nl_seg_kind cur_seg_kind;
	uint8_t cur_seg_msg_idx; /* meaningful for WMSG/RMSG segments only */

	int xfr_result; /* accumulated result for xfr_msgs[]; sticky, first error wins */

#ifdef CONFIG_I2C_CALLBACK
	i2c_callback_t cb; /* NULL => sync caller owns the in-flight xfr */
	void *cb_userdata;
#endif
#ifdef CONFIG_I2C_TARGET
	struct xec_i2c_nl_target_data tgt;
#endif
};

/* Port device struture */
struct xec_i2c_nl_port_config {
	const struct device *controller;
	const struct pinctrl_dev_config *pincfg;
	uint32_t bitrate;
	uint8_t port_id;
	bool is_default;
};

/*
 * Port device data: per-port state that must survive the driver
 * switching the shared controller away to another port and back.
 * runtime_freq is 0 until the app calls i2c_configure() on this port
 * at least once (0 Hz is never a valid bus frequency, so it doubles
 * as "unset"); once set it takes precedence over the port's DT
 * clock-frequency every time this port is (re)selected -- see
 * xec_i2c_nl_port_freq().
 */
struct xec_i2c_nl_port_data {
	uint32_t runtime_freq;
};

/* Sentinel freqhz value: use timing[XEC_I2C_NL_TM_DT] verbatim instead
 * of a bucketed 100k/400k/1M row (I2C_SPEED_DT / a "timing-dt" port).
 */
#define XEC_I2C_NL_FREQ_DT UINT32_MAX

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
		return &cfg->timing[XEC_I2C_NL_TM_DT];
	}
	if (freqhz <= KHZ(100)) {
		return &cfg->timing[XEC_I2C_NL_TM_100K];
	}
	if (freqhz <= KHZ(400)) {
		return &cfg->timing[XEC_I2C_NL_TM_400K];
	}
	return &cfg->timing[XEC_I2C_NL_TM_1M];
}

static int xec_i2c_nl_program_ctrl(const struct device *ctrl, uint32_t freqhz, uint8_t port)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	const struct xec_i2c_nl_timing *tm = xec_i2c_nl_timing_for(cfg, freqhz);
	uintptr_t base = cfg->base;

	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, MCHP_MEC_ECIA_GIRQ_DIS);

	sys_write32(0U, base + XEC_I2C_CFG_OFS);

	soc_xec_pcr_reset_en(cfg->enc_pcr);
	k_busy_wait(10U);

	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);
	soc_ecia_girq_status_clear(cfg->girq_wk, cfg->girq_wk_pos);

	/* PIN=1 to clear any latent assertion left by the legacy engine. */
	sys_write8(BIT(XEC_I2C_CR_PIN_POS), base + XEC_I2C_CR_OFS);

	/* Port select, filters on, general-call disabled, HDONE interrupt
	 * enabled. IDLE_IEN is intentionally LEFT OFF here — see the
	 * comment on CFG_IDLE_IEN above and the ISR for why it has to be
	 * enabled later (inside the HDONE handler at NL-finished time).
	 * ENAB is set last after timing has been written.
	 */
	sys_write32((XEC_I2C_CFG_PORT_SET(port) | BIT(XEC_I2C_CFG_FEN_POS) |
		     BIT(XEC_I2C_CFG_GC_DIS_POS) | BIT(XEC_I2C_CFG_HD_IEN_POS)),
		     base + XEC_I2C_CFG_OFS);

	/* Clear any latched CMPL bits we care about so that a stale state
	 * (left over from a prior run before the PCR reset, or from the
	 * power-on default) cannot fire the moment GIRQ is enabled.
	 */
	xec_i2c_v3_cmpl_clear(base, XEC_I2C_CMPL_RW1C_MSK);

	sys_write32(tm->data_timing, base + XEC_I2C_DT_OFS);
	sys_write32(tm->idle_scaling, base + XEC_I2C_ISC_OFS);
	sys_write32(tm->timeout_scaling, base + XEC_I2C_TMOUT_SC_OFS);
	sys_write32((uint32_t)tm->bus_clock, base + XEC_I2C_BCLK_OFS);
	soc_mmcr_mask_set8(base + XEC_I2C_RSHT_OFS, tm->rpt_start_hold_tm, XEC_I2C_RSHT_MSK);
	soc_mmcr_mask_set8(base + XEC_I2C_MR0_OFS, tm->mr1, XEC_I2C_MR0_TM_MSK);

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
	soc_ecia_girq_status_clear(cfg->girq_wk, cfg->girq_wk_pos);
	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, MCHP_MEC_ECIA_GIRQ_EN);

	data->active_freq = freqhz;
	data->active_port = port;

	return 0;
}

static int xec_i2c_nl_apply_port(const struct device *port_dev)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	struct xec_i2c_nl_port_data *port_data = port_dev->data;
	const struct device *ctrl = port_cfg->controller;
	struct xec_i2c_nl_data *ctrl_data = ctrl->data;
	uint32_t freq = xec_i2c_nl_port_freq(port_cfg, port_data);
	int rc;

	/* Same port AND same frequency already programmed: nothing to do.
	 * A port match alone is not enough -- i2c_configure() can change
	 * this port's frequency while it is the one already selected, and
	 * that still has to reprogram the timing registers.
	 */
	if (ctrl_data->active_port == port_cfg->port_id && ctrl_data->active_freq == freq) {
		return 0;
	}

	rc = pinctrl_apply_state(port_cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("pinctrl_apply_state(%s)=%d", port_dev->name, rc);
		return rc;
	}

	return xec_i2c_nl_program_ctrl(ctrl, freq, port_cfg->port_id);
}

/* I2C configure API */
static int xec_i2c_nl_vport_config(const struct device *port_dev, uint32_t i2c_config)
{
	struct xec_i2c_nl_port_data *port_data = port_dev->data;
	uint32_t freq;

	if (!(i2c_config & I2C_MODE_CONTROLLER)) {
		return -ENOTSUP; /* target-only mode has nothing to configure here */
	}

#ifdef CONFIG_I2C_TARGET
	{
		const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
		struct xec_i2c_nl_data *ctrl_data = port_cfg->controller->data;

		if (ctrl_data->tgt.count != 0U) {
			return -EBUSY; /* already a target; can't reconfigure as controller */
		}
	}
#endif

	freq = xec_i2c_nl_speed_to_freq(I2C_SPEED_GET(i2c_config));
	if (freq == 0U) {
		return -ENOTSUP;
	}

	/* Sticky per port: xec_i2c_nl_apply_port() compares this against
	 * the controller's active port AND active frequency, so it
	 * reprograms immediately if this port is already selected, and
	 * again on every future switch back to this port.
	 */
	port_data->runtime_freq = freq;

	return xec_i2c_nl_apply_port(port_dev);
}

/* I2C get config API */
static int xec_i2c_nl_vport_get_config(const struct device *port_dev, uint32_t *i2c_config)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	const struct xec_i2c_nl_port_data *port_data = port_dev->data;

	*i2c_config = I2C_MODE_CONTROLLER |
		      I2C_SPEED_SET(xec_i2c_nl_freq_to_speed(
			      xec_i2c_nl_port_freq(port_cfg, port_data)));

	return 0;
}

/* 7-bit target address; matches the address-byte layout xec_i2c_nl_parse_msgs()
 * and the Master TX register expect.
 */
#define XEC_I2C_NL_ADDR_MASK 0x7FU

/* Forward declarations: xec_i2c_nl_dma_cb() is registered as the DMA
 * channel's completion callback before it's defined, and it (along with
 * xec_i2c_nl_isr(), further below) needs xec_i2c_nl_finish_xfr() before
 * that is defined.
 */
static void xec_i2c_nl_dma_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
			       int status);
static void xec_i2c_nl_finish_xfr(const struct device *ctrl, int result);

/* Resolve a DMA segment to its memory-side address/length -- see the
 * segment model in the Task 3 design notes (struct xec_i2c_nl_data's
 * cur_seg_kind/cur_seg_msg_idx comment).
 */
static void xec_i2c_nl_seg_addr(const struct xec_i2c_nl_data *data,
				 enum xec_i2c_nl_seg_kind kind, uint8_t msg_idx,
				 uintptr_t *mem_addr, uint32_t *len)
{
	switch (kind) {
	case XEC_I2C_NL_SEG_ADDR0:
		*mem_addr = (uintptr_t)&data->addr_byte[0];
		*len = 1U;
		break;
	case XEC_I2C_NL_SEG_ADDRN:
		*mem_addr = (uintptr_t)&data->addr_byte[1];
		*len = 1U;
		break;
	case XEC_I2C_NL_SEG_WMSG:
	case XEC_I2C_NL_SEG_RMSG:
	default:
		*mem_addr = (uintptr_t)data->xfr_msgs[msg_idx].buf;
		*len = data->xfr_msgs[msg_idx].len;
		break;
	}
}

static bool xec_i2c_nl_seg_advance_once(const struct xec_i2c_nl_request *req,
					 enum xec_i2c_nl_seg_kind *kind, uint8_t *msg_idx)
{
	switch (*kind) {
	case XEC_I2C_NL_SEG_ADDR0:
		if (req->num_write_msgs > 0U) {
			*kind = XEC_I2C_NL_SEG_WMSG;
			*msg_idx = req->first_write_msg_idx;
			return true;
		}
		return false;
	case XEC_I2C_NL_SEG_WMSG:
		if ((uint16_t)(*msg_idx + 1U) <
		    (uint16_t)req->first_write_msg_idx + req->num_write_msgs) {
			(*msg_idx)++;
			return true;
		}
		if (req->flags & XEC_I2C_NL_REQ_STARTN) {
			*kind = XEC_I2C_NL_SEG_ADDRN;
			return true;
		}
		return false;
	case XEC_I2C_NL_SEG_ADDRN:
		return false;
	case XEC_I2C_NL_SEG_RMSG:
	default:
		if ((uint16_t)(*msg_idx + 1U) <
		    (uint16_t)req->first_read_msg_idx + req->num_read_msgs) {
			(*msg_idx)++;
			return true;
		}
		return false;
	}
}

/* Advance (kind, msg_idx) to the next segment of data->cur_req, skipping
 * any zero-length message rather than arming a zero-byte DMA block.
 * Returns false once the current phase (write or read) is exhausted.
 */
static bool xec_i2c_nl_seg_next(const struct xec_i2c_nl_data *data,
				 enum xec_i2c_nl_seg_kind *kind, uint8_t *msg_idx)
{
	while (xec_i2c_nl_seg_advance_once(&data->cur_req, kind, msg_idx)) {
		if ((*kind != XEC_I2C_NL_SEG_WMSG && *kind != XEC_I2C_NL_SEG_RMSG) ||
		    data->xfr_msgs[*msg_idx].len > 0U) {
			return true;
		}
	}
	return false;
}

/* Arm one write-phase DMA segment. first_segment_of_phase selects a fresh
 * dma_config() (direction/slot may have changed since the last phase) vs.
 * the cheaper dma_reload() (same direction, only src/dst/size change).
 * Both dma_config()/dma_reload() only program the channel; dma_start()
 * is always required afterward.
 */
static int xec_i2c_nl_dma_arm_write(const struct device *ctrl, uintptr_t mem_addr, uint32_t len,
				     bool first_segment_of_phase)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	uintptr_t dev_addr = cfg->base + XEC_I2C_HTX_OFS;
	int rc;

	if (!first_segment_of_phase) {
		rc = dma_reload(cfg->dma_dev, cfg->cm_dma_chan, mem_addr, dev_addr, len);
	} else {
		struct dma_block_config blk = {
			.source_address = mem_addr,
			.dest_address = dev_addr,
			.block_size = len,
			.source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
			.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
		};
		struct dma_config dcfg = {
			.dma_slot = cfg->cm_dma_slot,
			.channel_direction = MEMORY_TO_PERIPHERAL,
			.source_data_size = 1U,
			.dest_data_size = 1U,
			.source_burst_length = 1U,
			.dest_burst_length = 1U,
			.complete_callback_en = 1U,
			.block_count = 1U,
			.head_block = &blk,
			.dma_callback = xec_i2c_nl_dma_cb,
			.user_data = data,
		};

		dma_stop(cfg->dma_dev, cfg->cm_dma_chan);
		rc = dma_config(cfg->dma_dev, cfg->cm_dma_chan, &dcfg);
	}
	if (rc != 0) {
		return rc;
	}

	return dma_start(cfg->dma_dev, cfg->cm_dma_chan);
}

/* Arm one read-phase DMA segment; mirrors xec_i2c_nl_dma_arm_write() with
 * source/dest swapped (peripheral -> memory).
 */
static int xec_i2c_nl_dma_arm_read(const struct device *ctrl, uintptr_t mem_addr, uint32_t len,
				    bool first_segment_of_phase)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	uintptr_t dev_addr = cfg->base + XEC_I2C_HRX_OFS;
	int rc;

	if (!first_segment_of_phase) {
		rc = dma_reload(cfg->dma_dev, cfg->cm_dma_chan, dev_addr, mem_addr, len);
	} else {
		struct dma_block_config blk = {
			.source_address = dev_addr,
			.dest_address = mem_addr,
			.block_size = len,
			.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
			.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		};
		struct dma_config dcfg = {
			.dma_slot = cfg->cm_dma_slot,
			.channel_direction = PERIPHERAL_TO_MEMORY,
			.source_data_size = 1U,
			.dest_data_size = 1U,
			.source_burst_length = 1U,
			.dest_burst_length = 1U,
			.complete_callback_en = 1U,
			.block_count = 1U,
			.head_block = &blk,
			.dma_callback = xec_i2c_nl_dma_cb,
			.user_data = data,
		};

		dma_stop(cfg->dma_dev, cfg->cm_dma_chan);
		rc = dma_config(cfg->dma_dev, cfg->cm_dma_chan, &dcfg);
	}
	if (rc != 0) {
		return rc;
	}

	return dma_start(cfg->dma_dev, cfg->cm_dma_chan);
}

/* DMA channel completion callback: fires once a single segment's bytes
 * have actually moved (this DMA is hardware flow-controlled by the I2C
 * FSM's own per-byte request line). Never signals phase or request
 * completion -- xec_i2c_nl_isr() owns that via HDONE/IDLE. See the
 * segment model notes for why these are two independent interrupt
 * sources that must not race.
 */
static void xec_i2c_nl_dma_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
			       int status)
{
	struct xec_i2c_nl_data *data = user_data;
	const struct device *ctrl = data->ctrl;
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	enum xec_i2c_nl_seg_kind kind = data->cur_seg_kind;
	uint8_t msg_idx = data->cur_seg_msg_idx;
	uintptr_t mem_addr;
	uint32_t len;
	bool is_read;
	int rc;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		dma_stop(cfg->dma_dev, cfg->cm_dma_chan);
		xec_i2c_nl_finish_xfr(ctrl, status);
		return;
	}

	if (!xec_i2c_nl_seg_next(data, &kind, &msg_idx)) {
		return; /* phase exhausted; wait for xec_i2c_nl_isr() */
	}

	xec_i2c_nl_seg_addr(data, kind, msg_idx, &mem_addr, &len);

	is_read = (kind == XEC_I2C_NL_SEG_RMSG);
	rc = is_read ? xec_i2c_nl_dma_arm_read(ctrl, mem_addr, len, false)
		     : xec_i2c_nl_dma_arm_write(ctrl, mem_addr, len, false);
	if (rc != 0) {
		dma_stop(cfg->dma_dev, cfg->cm_dma_chan);
		xec_i2c_nl_finish_xfr(ctrl, rc);
		return;
	}

	data->cur_seg_kind = kind;
	data->cur_seg_msg_idx = msg_idx;
}

/* 7-bit address + R/W bit, matching the Master TX register's expected
 * layout: bits[7:1]=address, bit[0]=0(write)/1(read).
 */
#define XEC_I2C_NL_ADDR_BYTE(addr7, is_read) ((uint8_t)(((addr7) << 1) | ((is_read) ? 1U : 0U)))

/* Build the address byte(s) DMA sources for this request's START0 and,
 * if present, STARTN. A request with no write messages is a plain read,
 * but its address byte is still sent through the write-phase data path
 * (see xec_i2c_nl_parse_msgs()'s doc comment) with the read bit set.
 */
static void xec_i2c_nl_build_addr_bytes(struct xec_i2c_nl_data *data, uint16_t addr7,
					 const struct xec_i2c_nl_request *req)
{
	data->addr_byte[0] = XEC_I2C_NL_ADDR_BYTE(addr7, req->num_write_msgs == 0U);
	if (req->flags & XEC_I2C_NL_REQ_STARTN) {
		data->addr_byte[1] = XEC_I2C_NL_ADDR_BYTE(addr7, true);
	}
}

/* Parse and arm the next HW request for the in-flight transfer
 * (data->xfr_msgs/xfr_num_msgs/next_start_idx). Shared by fresh
 * submission (xec_i2c_nl_vport_xfr[_cb]()) and ISR-driven continuation
 * (xec_i2c_nl_isr()'s CMPL.IDLE handler). Returns 0, or a negative errno
 * if parsing or arming DMA/HCMD failed -- the caller decides how to
 * finish the transfer in that case.
 */
static int xec_i2c_nl_arm_request(const struct device *ctrl)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	uintptr_t base = cfg->base;
	struct xec_i2c_nl_request *req = &data->cur_req;
	bool force_stop;
	uint32_t hcmd;
	int rc;

	rc = xec_i2c_nl_parse_msgs(data->xfr_msgs, data->xfr_num_msgs, data->next_start_idx, req);
	if (rc < 0) {
		return rc;
	}
	data->next_start_idx = req->last_msg_idx + 1U;

	xec_i2c_nl_build_addr_bytes(data, data->xfr_i2c_addr, req);

	data->cur_seg_kind = XEC_I2C_NL_SEG_ADDR0;
	data->cur_seg_msg_idx = XEC_I2C_NL_NO_MSG_IDX;

	rc = xec_i2c_nl_dma_arm_write(ctrl, (uintptr_t)&data->addr_byte[0], 1U, true);
	if (rc != 0) {
		return rc;
	}

	sys_write32(XEC_I2C_ELEN_HWR_SET(req->write_count >> 8) |
			    XEC_I2C_ELEN_HRD_SET(req->read_count >> 8),
		    base + XEC_I2C_ELEN_OFS);

	/* This HW has no validated way to hold the bus open across two
	 * separate HCMD.RUN pulses, so every request boundary closes with
	 * STOP -- both the ordinary case (req already flagged STOP) and the
	 * rare case where xec_i2c_nl_parse_msgs() ended a request without
	 * one (>64KB combined length, or an unsupported mid-request
	 * I2C_MSG_RESTART shape) but more messages remain.
	 */
	force_stop = (req->flags & XEC_I2C_NL_REQ_STOP) ||
		     (data->next_start_idx < data->xfr_num_msgs);

	hcmd = HCMD_RUN | HCMD_PROCEED | XEC_I2C_HCMD_WCL_SET(req->write_count & 0xFFU) |
	       XEC_I2C_HCMD_RCL_SET(req->read_count & 0xFFU);
	if (req->flags & XEC_I2C_NL_REQ_START0) {
		hcmd |= HCMD_START0;
	}
	if (req->flags & XEC_I2C_NL_REQ_STARTN) {
		hcmd |= HCMD_STARTN;
	}
	if (force_stop) {
		hcmd |= HCMD_STOP;
	}
	sys_write32(hcmd, base + XEC_I2C_HCMD_OFS);

	return 0;
}

/* Finish the whole caller-visible transfer (xfr_msgs[0..xfr_num_msgs) --
 * possibly several HW requests), called from exactly one place per
 * transfer: xec_i2c_nl_isr() (success or HW error) or, on a submission-
 * time arm failure, the transfer entry points themselves (which give the
 * lock back directly instead, to avoid a double k_sem_give -- see
 * xec_i2c_nl_vport_xfr[_cb]()). xfr_active guards against a pathological
 * same-tick double call (e.g. an error arriving alongside IDLE).
 */
static void xec_i2c_nl_finish_xfr(const struct device *ctrl, int result)
{
	struct xec_i2c_nl_data *data = ctrl->data;

	if (!data->xfr_active) {
		return;
	}
	data->xfr_active = false;
	data->xfr_result = (data->xfr_result == 0) ? result : data->xfr_result;

#ifdef CONFIG_I2C_CALLBACK
	if (data->cb != NULL) {
		i2c_callback_t cb = data->cb;
		void *userdata = data->cb_userdata;

		data->cb = NULL;
		/* Give the lock back before invoking cb() so a callback that
		 * immediately resubmits doesn't deadlock on its own lock.
		 */
		k_sem_give(&data->lock);
		cb(data->xfr_port_dev, data->xfr_result, userdata);
		return;
	}
#endif
	k_sem_give(&data->done_sem);
}

/* I2C synchronous transfer API */
static int xec_i2c_nl_vport_xfr(const struct device *port_dev, struct i2c_msg *msgs,
				uint8_t num_msgs, uint16_t i2c_address)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	const struct device *ctrl = port_cfg->controller;
	struct xec_i2c_nl_data *data = ctrl->data;
	int rc;

	if (msgs == NULL || num_msgs == 0U || (i2c_address & ~XEC_I2C_NL_ADDR_MASK) != 0U) {
		return -EINVAL;
	}

#ifdef CONFIG_I2C_TARGET
	if (data->tgt.count != 0U) {
		return -EBUSY; /* already a target; would block K_FOREVER otherwise */
	}
#endif

	k_sem_take(&data->lock, K_FOREVER);

#ifdef CONFIG_I2C_TARGET
	if (data->tgt.count != 0U) {
		/* Closes the race against a target_register() that ran between
		 * the check above and this lock acquisition -- both take the
		 * same data->lock, so this pair of checks brackets every place
		 * tgt.count can change.
		 */
		k_sem_give(&data->lock);
		return -EBUSY;
	}
#endif

	rc = xec_i2c_nl_apply_port(port_dev);
	if (rc != 0) {
		k_sem_give(&data->lock);
		return rc;
	}

	data->xfr_msgs = msgs;
	data->xfr_num_msgs = num_msgs;
	data->xfr_i2c_addr = i2c_address;
	data->xfr_port_dev = port_dev;
	data->xfr_active = true;
	data->xfr_result = 0;
	data->next_start_idx = 0U;
#ifdef CONFIG_I2C_CALLBACK
	data->cb = NULL;
#endif

	rc = xec_i2c_nl_arm_request(ctrl);
	if (rc != 0) {
		data->xfr_active = false;
		k_sem_give(&data->lock);
		return rc;
	}

	k_sem_take(&data->done_sem, K_FOREVER);

	rc = data->xfr_result;
	k_sem_give(&data->lock);

	return rc;
}

#ifdef CONFIG_I2C_CALLBACK
/* I2C asynchronous transfer API */
static int xec_i2c_nl_vport_xfr_cb(const struct device *port_dev, struct i2c_msg *msgs,
				   uint8_t num_msgs, uint16_t i2c_address, i2c_callback_t cb,
				   void *userdata)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	const struct device *ctrl = port_cfg->controller;
	struct xec_i2c_nl_data *data = ctrl->data;
	int rc;

	if (msgs == NULL || num_msgs == 0U || cb == NULL ||
	    (i2c_address & ~XEC_I2C_NL_ADDR_MASK) != 0U) {
		return -EINVAL;
	}

#ifdef CONFIG_I2C_TARGET
	if (data->tgt.count != 0U) {
		return -EBUSY;
	}
#endif

	/* @isr_ok: i2c_transfer_cb() submission must not block. */
	if (k_sem_take(&data->lock, K_NO_WAIT) != 0) {
		return -EWOULDBLOCK;
	}

#ifdef CONFIG_I2C_TARGET
	if (data->tgt.count != 0U) {
		k_sem_give(&data->lock);
		return -EBUSY;
	}
#endif

	rc = xec_i2c_nl_apply_port(port_dev);
	if (rc != 0) {
		k_sem_give(&data->lock);
		return rc;
	}

	data->xfr_msgs = msgs;
	data->xfr_num_msgs = num_msgs;
	data->xfr_i2c_addr = i2c_address;
	data->xfr_port_dev = port_dev;
	data->xfr_active = true;
	data->xfr_result = 0;
	data->next_start_idx = 0U;
	data->cb = cb;
	data->cb_userdata = userdata;

	rc = xec_i2c_nl_arm_request(ctrl);
	if (rc != 0) {
		data->xfr_active = false;
		data->cb = NULL;
		k_sem_give(&data->lock);
		return rc;
	}

	return 0;
}
#endif

#ifdef CONFIG_I2C_TARGET
/* Forward declarations: xec_i2c_nl_target_arm()/_handle_error() are
 * needed before they're defined, matching the controller-mode forward
 * declarations above.
 */
static int xec_i2c_nl_target_arm(const struct device *ctrl);
static void xec_i2c_nl_target_handle_error(const struct device *ctrl,
					    enum i2c_error_reason reason);
static void xec_i2c_nl_tm_rx_dma_cb(const struct device *dma_dev, void *user_data,
				     uint32_t channel, int status);
static void xec_i2c_nl_tm_tx_dma_cb(const struct device *dma_dev, void *user_data,
				     uint32_t channel, int status);

/* Incoming-address shadow register: bits[7:1]=matched 7-bit address,
 * bit[0]=R/W. Read live wherever the current transaction's address/
 * direction is needed rather than caching it, so DMA progress and
 * interrupt delivery can never disagree about which target is live.
 */
static uint8_t xec_i2c_nl_target_addr_byte(const struct xec_i2c_nl_config *cfg)
{
	return (uint8_t)sys_read32(cfg->base + XEC_I2C_IAS_OFS);
}

static struct i2c_target_config *xec_i2c_nl_target_lookup(struct xec_i2c_nl_data *data,
							    uint8_t addr7)
{
	uint32_t i;

	for (i = 0; i < XEC_I2C_OA_NUM_TARGETS; i++) {
		if (data->tgt.slots[i] != NULL && data->tgt.slots[i]->address == addr7) {
			return data->tgt.slots[i];
		}
	}
	return NULL;
}

static struct i2c_target_config *xec_i2c_nl_target_active(const struct xec_i2c_nl_config *cfg,
							    struct xec_i2c_nl_data *data)
{
	return xec_i2c_nl_target_lookup(data, xec_i2c_nl_target_addr_byte(cfg) >> 1);
}

/* Persistent PERIPHERAL_TO_MEMORY block, TRX -> tm_rx_buf, (re-)armed once
 * per transaction by xec_i2c_nl_target_arm(). complete_callback_en is left
 * off: delivery is driven by reading TCMD.RCL/ELEN.TRD byte counts from
 * xec_i2c_nl_target_handle_read_pause()/_handle_stop(), not by a DMA
 * completion event -- the real completion signal in either direction is
 * the I2C-side CMPL.TDONE/IDLE, not DMA block-complete.
 */
static int xec_i2c_nl_tm_dma_arm_rx(const struct device *ctrl)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	struct dma_block_config blk = {
		.source_address = cfg->base + XEC_I2C_TRX_OFS,
		.dest_address = (uintptr_t)cfg->tm_rx_buf,
		.block_size = cfg->tm_rx_buf_size,
		.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
		.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	};
	struct dma_config dcfg = {
		.dma_slot = cfg->tm_dma_slot,
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.source_data_size = 1U,
		.dest_data_size = 1U,
		.source_burst_length = 1U,
		.dest_burst_length = 1U,
		.block_count = 1U,
		.head_block = &blk,
		.dma_callback = xec_i2c_nl_tm_rx_dma_cb,
		.user_data = data,
	};
	int rc;

	dma_stop(cfg->dma_dev, cfg->tm_dma_chan);
	rc = dma_config(cfg->dma_dev, cfg->tm_dma_chan, &dcfg);
	if (rc != 0) {
		return rc;
	}

	return dma_start(cfg->dma_dev, cfg->tm_dma_chan);
}

/* Reactive single-block MEMORY_TO_PERIPHERAL config, rebuilt on every
 * buf_read_requested() call: a target can't know the host's read length
 * in advance, and this sources zero-copy directly from the application's
 * own buffer (unlike RX, which must land in tm_rx_buf first since the app
 * has no buffer to hand over ahead of a write).
 */
static int xec_i2c_nl_tm_dma_arm_tx(const struct device *ctrl, const uint8_t *buf, uint32_t len)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	struct dma_block_config blk = {
		.source_address = (uintptr_t)buf,
		.dest_address = cfg->base + XEC_I2C_TTX_OFS,
		.block_size = len,
		.source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
	};
	struct dma_config dcfg = {
		.dma_slot = cfg->tm_dma_slot,
		.channel_direction = MEMORY_TO_PERIPHERAL,
		.source_data_size = 1U,
		.dest_data_size = 1U,
		.source_burst_length = 1U,
		.dest_burst_length = 1U,
		.block_count = 1U,
		.head_block = &blk,
		.dma_callback = xec_i2c_nl_tm_tx_dma_cb,
		.user_data = data,
	};
	int rc;

	dma_stop(cfg->dma_dev, cfg->tm_dma_chan);
	rc = dma_config(cfg->dma_dev, cfg->tm_dma_chan, &dcfg);
	if (rc != 0) {
		return rc;
	}

	return dma_start(cfg->dma_dev, cfg->tm_dma_chan);
}

static void xec_i2c_nl_tm_rx_dma_cb(const struct device *dma_dev, void *user_data,
				     uint32_t channel, int status)
{
	struct xec_i2c_nl_data *data = user_data;
	const struct xec_i2c_nl_config *cfg = data->ctrl->config;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		dma_stop(cfg->dma_dev, cfg->tm_dma_chan);
		xec_i2c_nl_target_handle_error(data->ctrl, I2C_ERROR_DMA);
	}
}

static void xec_i2c_nl_tm_tx_dma_cb(const struct device *dma_dev, void *user_data,
				     uint32_t channel, int status)
{
	struct xec_i2c_nl_data *data = user_data;
	const struct xec_i2c_nl_config *cfg = data->ctrl->config;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		dma_stop(cfg->dma_dev, cfg->tm_dma_chan);
		xec_i2c_nl_target_handle_error(data->ctrl, I2C_ERROR_DMA);
	}
}

/* (Re-)arm for the next target-mode transaction: RX DMA staged into
 * tm_rx_buf, TCMD/ELEN preset for an incoming write, AAT_IEN armed
 * (IDLE_IEN left off -- v3.8 erratum, see CFG_IDLE_IEN above). Called at
 * first target_register() and after every completed/errored transaction.
 */
static int xec_i2c_nl_target_arm(const struct device *ctrl)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	uintptr_t base = cfg->base;
	uint32_t rval;
	int rc;

	data->tgt.phase = XEC_I2C_NL_TGT_IDLE;

	rc = xec_i2c_nl_tm_dma_arm_rx(ctrl);
	if (rc != 0) {
		return rc;
	}

	xec_i2c_v3_cmpl_clear(base, CMPL_TGT_CLEAR);

	/* WCL=1 here is a placeholder that xec_i2c_nl_target_handle_read_pause()
	 * overwrites with the real length before releasing PROCEED -- it
	 * never reaches the bus. RCL/ELEN.TRD are set from the staged RX
	 * buffer size so the FSM NAKs once that buffer fills.
	 */
	rval = sys_read32(base + XEC_I2C_ELEN_OFS);
	rval &= ~(XEC_I2C_ELEN_TRD_MSK | XEC_I2C_ELEN_TWR_MSK);
	rval |= XEC_I2C_ELEN_TRD_SET((uint32_t)cfg->tm_rx_buf_size >> 8);
	sys_write32(rval, base + XEC_I2C_ELEN_OFS);

	sys_write32(XEC_I2C_TCMD_RCL_SET((uint32_t)cfg->tm_rx_buf_size & 0xFFU) |
			    XEC_I2C_TCMD_WCL_SET(1U) | TCMD_RUN | TCMD_PROCEED,
		    base + XEC_I2C_TCMD_OFS);

	xec_i2c_v3_cmpl_clear(base, CMPL_IDLE);
	soc_mmcr_mask_set(base + XEC_I2C_CFG_OFS, CFG_AAT_IEN, CFG_AAT_IEN | CFG_IDLE_IEN);

	return 0;
}

/* Bytes actually landed in tm_rx_buf for the write phase just finished,
 * derived from how far TCMD.RCL/ELEN.TRD counted down from the armed
 * buffer size -- mirrors how controller mode tracks bytes via HCMD's
 * counts rather than trusting DMA progress directly.
 */
static uint32_t xec_i2c_nl_target_rx_consumed(const struct xec_i2c_nl_config *cfg)
{
	uint32_t rcl = XEC_I2C_TCMD_RCL_GET(sys_read32(cfg->base + XEC_I2C_TCMD_OFS));
	uint32_t trd = XEC_I2C_ELEN_TRD_GET(sys_read32(cfg->base + XEC_I2C_ELEN_OFS));

	return (uint32_t)cfg->tm_rx_buf_size - ((trd << 8) | rcl);
}

/* TCMD.RUN=1,PROC=0 with the incoming address byte's R/W bit set: the host
 * wants to read. Deliver any write payload from a preceding write-then-
 * repeated-start-read combined transaction first, then ask the app for
 * data to send and arm TX DMA before releasing PROCEED.
 */
static void xec_i2c_nl_target_handle_read_pause(const struct device *ctrl)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	uintptr_t base = cfg->base;
	struct i2c_target_config *tcfg = xec_i2c_nl_target_active(cfg, data);
	uint8_t *buf = NULL;
	uint32_t len = 0;
	uint32_t consumed;
	uint32_t rval;
	int rc;

	if (tcfg == NULL) {
		xec_i2c_nl_target_handle_error(ctrl, I2C_ERROR_GENERIC);
		return;
	}

	consumed = xec_i2c_nl_target_rx_consumed(cfg);
	if (consumed > 1U) {
		tcfg->callbacks->buf_write_received(tcfg, &cfg->tm_rx_buf[1], consumed - 1U);
	}

	rc = tcfg->callbacks->buf_read_requested(tcfg, &buf, &len);
	if (rc != 0 || buf == NULL || len == 0U) {
		xec_i2c_nl_target_handle_error(ctrl, I2C_ERROR_GENERIC);
		return;
	}
	if (len > 0xFFFFU) {
		len = 0xFFFFU; /* 16-bit HW count cap */
	}

	rc = xec_i2c_nl_tm_dma_arm_tx(ctrl, buf, len);
	if (rc != 0) {
		xec_i2c_nl_target_handle_error(ctrl, I2C_ERROR_DMA);
		return;
	}
	data->tgt.phase = XEC_I2C_NL_TGT_TX;

	rval = sys_read32(base + XEC_I2C_ELEN_OFS);
	rval = (rval & ~XEC_I2C_ELEN_TWR_MSK) | XEC_I2C_ELEN_TWR_SET(len >> 8);
	sys_write32(rval, base + XEC_I2C_ELEN_OFS);

	sys_write32(XEC_I2C_TCMD_WCL_SET(len & 0xFFU) | TCMD_RUN | TCMD_PROCEED,
		    base + XEC_I2C_TCMD_OFS);
}

/* CMPL.IDLE gated on our own IDLE_IEN: the closing STOP. tgt.phase says
 * whether the channel was still RX-configured (plain host-write closed
 * out -- deliver the staged payload) or TX-configured (host-read closed
 * out -- no RX payload to strip). Always fires stop() then re-arms.
 */
static void xec_i2c_nl_target_handle_stop(const struct device *ctrl)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	struct i2c_target_config *tcfg = xec_i2c_nl_target_active(cfg, data);

	if (tcfg == NULL) {
		xec_i2c_nl_target_arm(ctrl);
		return;
	}

	if (data->tgt.phase == XEC_I2C_NL_TGT_IDLE) {
		uint32_t consumed = xec_i2c_nl_target_rx_consumed(cfg);

		if (consumed > 1U) {
			tcfg->callbacks->buf_write_received(tcfg, &cfg->tm_rx_buf[1],
							     consumed - 1U);
		}
	}

	tcfg->callbacks->stop(tcfg);

	xec_i2c_nl_target_arm(ctrl);
}

static void xec_i2c_nl_target_handle_error(const struct device *ctrl,
					    enum i2c_error_reason reason)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	struct i2c_target_config *tcfg = xec_i2c_nl_target_active(cfg, data);

	if (tcfg != NULL && tcfg->callbacks->error != NULL) {
		tcfg->callbacks->error(tcfg, reason);
	}

	xec_i2c_nl_target_arm(ctrl);
}

/* Target-mode ISR, dispatched to from xec_i2c_nl_isr() while any target
 * address is registered; the controller-mode HDONE/IDLE logic below this
 * function is untouched.
 */
static void xec_i2c_nl_isr_target(const struct device *ctrl, uint32_t cmpl)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	uintptr_t base = cfg->base;
	uint32_t cfgr;

	if (cmpl & (CMPL_LAB | CMPL_BER)) {
		enum i2c_error_reason reason =
			(cmpl & CMPL_LAB) ? I2C_ERROR_ARBITRATION : I2C_ERROR_GENERIC;

		xec_i2c_v3_cmpl_clear(base, CMPL_LAB | CMPL_BER | CMPL_TGT_CLEAR);
		dma_stop(cfg->dma_dev, cfg->tm_dma_chan);
		xec_i2c_nl_target_handle_error(ctrl, reason);
		return;
	}

	/* Address-match -> IDLE_IEN handoff. AAT_IEN is safe to leave armed
	 * at idle (only fires on a real OWN-address match); IDLE_IEN is not
	 * (same v3.8 erratum CFG_IDLE_IEN documents for controller mode).
	 * The address-match interrupt is the one guaranteed NBB==0 point in
	 * a transaction, so it's where the erratum-sensitive IDLE_IEN gets
	 * armed -- one-shot per transaction. No early return: a same-tick
	 * TDONE/IDLE coincidence still falls through to the checks below,
	 * mirroring controller mode's HDONE->IDLE fall-through.
	 */
	cfgr = sys_read32(base + XEC_I2C_CFG_OFS);
	if (cfgr & CFG_AAT_IEN) {
		cfgr = (cfgr & ~CFG_AAT_IEN) | CFG_IDLE_IEN;
		sys_write32(cfgr, base + XEC_I2C_CFG_OFS);
	}

	if (cmpl & CMPL_TDONE) {
		uint32_t tcmd = sys_read32(base + XEC_I2C_TCMD_OFS);
		uint8_t addr_byte = xec_i2c_nl_target_addr_byte(cfg);
		bool host_read = (addr_byte & 0x01U) != 0U;
		bool fsm_paused = (tcmd & TCMD_RUN) && !(tcmd & TCMD_PROCEED);

		xec_i2c_v3_cmpl_clear(base, CMPL_TDONE);

		if (host_read && fsm_paused) {
			xec_i2c_nl_target_handle_read_pause(ctrl);
		}
		/* else: same TCMD pattern but write direction -- a host-write
		 * buffer-fill NAK (staged buffer full). Nothing to do; the
		 * closing STOP's CMPL.IDLE drives handle_stop() below (in
		 * this ISR entry or a later one).
		 */
	}

	cfgr = sys_read32(base + XEC_I2C_CFG_OFS);
	if ((cmpl & CMPL_IDLE) && (cfgr & CFG_IDLE_IEN)) {
		xec_i2c_v3_cmpl_clear(base, CMPL_IDLE);
		xec_i2c_nl_target_handle_stop(ctrl);
	}
}

static int xec_i2c_nl_vport_target_register(const struct device *port_dev,
					    struct i2c_target_config *target_cfg)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	const struct device *ctrl = port_cfg->controller;
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	int free_slot = -1;
	int rc = 0;
	uint32_t i;

	if (target_cfg == NULL || target_cfg->callbacks == NULL) {
		return -EINVAL;
	}
	if ((target_cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0U ||
	    (target_cfg->address & ~XEC_I2C_NL_ADDR_MASK) != 0U) {
		return -ENOTSUP; /* this HW is 7-bit-address only */
	}
	if (target_cfg->callbacks->buf_write_received == NULL ||
	    target_cfg->callbacks->buf_read_requested == NULL) {
		return -ENOSYS; /* buffer-mode callbacks only */
	}
	if (cfg->tm_rx_buf == NULL) {
		return -ENOSYS; /* DT: target-buffer-size absent */
	}
	if (!cfg->tm_dma_valid) {
		return -ENODEV; /* DT: no "target" dmas entry */
	}

	k_sem_take(&data->lock, K_FOREVER);

	for (i = 0; i < XEC_I2C_OA_NUM_TARGETS; i++) {
		if (data->tgt.slots[i] == target_cfg) {
			rc = -EALREADY;
			goto out;
		}
		if (data->tgt.slots[i] != NULL &&
		    data->tgt.slots[i]->address == target_cfg->address) {
			rc = -EADDRINUSE;
			goto out;
		}
		if (data->tgt.slots[i] == NULL && free_slot < 0) {
			free_slot = (int)i;
		}
	}
	if (free_slot < 0) {
		rc = -ENOSPC;
		goto out;
	}

	if (data->tgt.count == 0U) {
		/* First registration: fix pin routing and bus timing via the
		 * existing controller-mode path first -- program_ctrl()
		 * resets CFG to controller defaults (HD_IEN on) as a side
		 * effect, so it must run before the TD_IEN/AAT_IEN swap
		 * below, not after.
		 */
		rc = xec_i2c_nl_apply_port(port_dev);
		if (rc != 0) {
			goto out;
		}

		dma_stop(cfg->dma_dev, cfg->cm_dma_chan);

		soc_mmcr_mask_set(cfg->base + XEC_I2C_OA_OFS,
				   XEC_I2C_OA_SET((uint32_t)free_slot, target_cfg->address),
				   XEC_I2C_OA_MSK((uint32_t)free_slot));
		data->tgt.slots[free_slot] = target_cfg;
		data->tgt.count = 1U;

		soc_mmcr_mask_set(cfg->base + XEC_I2C_CFG_OFS, CFG_TD_IEN,
				   CFG_TD_IEN | CFG_HD_IEN);

		rc = xec_i2c_nl_target_arm(ctrl);
		if (rc != 0) {
			data->tgt.slots[free_slot] = NULL;
			data->tgt.count = 0U;
			soc_mmcr_mask_set(cfg->base + XEC_I2C_OA_OFS, 0,
					   XEC_I2C_OA_MSK((uint32_t)free_slot));
			soc_mmcr_mask_set(cfg->base + XEC_I2C_CFG_OFS, CFG_HD_IEN,
					   CFG_TD_IEN | CFG_HD_IEN);
			goto out;
		}
	} else {
		/* Second slot: controller already in target mode. Update OA
		 * only, under irq_lock() so an address-match interrupt
		 * landing the instant OA goes live never sees a populated OA
		 * bit whose slots[] entry isn't written yet.
		 */
		unsigned int key = irq_lock();

		data->tgt.slots[free_slot] = target_cfg;
		soc_mmcr_mask_set(cfg->base + XEC_I2C_OA_OFS,
				   XEC_I2C_OA_SET((uint32_t)free_slot, target_cfg->address),
				   XEC_I2C_OA_MSK((uint32_t)free_slot));
		irq_unlock(key);
		data->tgt.count++;
	}

out:
	k_sem_give(&data->lock);
	return rc;
}

static int xec_i2c_nl_vport_target_unregister(const struct device *port_dev,
					      struct i2c_target_config *target_cfg)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	const struct xec_i2c_nl_config *cfg = port_cfg->controller->config;
	struct xec_i2c_nl_data *data = port_cfg->controller->data;
	int slot = -1;
	uint32_t i;

	if (target_cfg == NULL) {
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER);

	for (i = 0; i < XEC_I2C_OA_NUM_TARGETS; i++) {
		if (data->tgt.slots[i] == target_cfg) {
			slot = (int)i;
			break;
		}
	}
	if (slot < 0) {
		k_sem_give(&data->lock);
		return -EINVAL;
	}

	{
		unsigned int key = irq_lock();

		soc_mmcr_mask_set(cfg->base + XEC_I2C_OA_OFS, 0, XEC_I2C_OA_MSK((uint32_t)slot));
		data->tgt.slots[slot] = NULL;
		irq_unlock(key);
	}
	data->tgt.count--;

	if (data->tgt.count == 0U) {
		/* AAT_IEN transiently flips IDLE_IEN on mid-transaction (see
		 * the ISR), so a stale IDLE_IEN surviving into a later
		 * program_ctrl() call would immediately retrigger the v3.8
		 * spurious-IDLE erratum -- clear both explicitly.
		 */
		dma_stop(cfg->dma_dev, cfg->tm_dma_chan);
		sys_write32(0, cfg->base + XEC_I2C_TCMD_OFS);
		soc_mmcr_mask_set(cfg->base + XEC_I2C_CFG_OFS, CFG_HD_IEN,
				   CFG_TD_IEN | CFG_HD_IEN | CFG_IDLE_IEN | CFG_AAT_IEN);
		xec_i2c_v3_cmpl_clear(cfg->base, CMPL_TGT_CLEAR);
		data->tgt.phase = XEC_I2C_NL_TGT_IDLE;
	}

	k_sem_give(&data->lock);
	return 0;
}
#endif

/* ---- Controller interrupt handler --- */
static void xec_i2c_nl_isr(const struct device *ctrl_dev)
{
	const struct xec_i2c_nl_config *cfg = ctrl_dev->config;
	struct xec_i2c_nl_data *data = ctrl_dev->data;
	uintptr_t base = cfg->base;
	uint32_t cmpl = sys_read32(base + XEC_I2C_CMPL_OFS);

#ifdef CONFIG_I2C_TARGET
	if (data->tgt.count != 0U) {
		xec_i2c_nl_isr_target(ctrl_dev, cmpl);
		goto out;
	}
#endif

	if (cmpl & CMPL_ERR) {
		int err = (cmpl & CMPL_HNAK) ? -ENXIO : (cmpl & CMPL_LAB) ? -EAGAIN : -EIO;

		xec_i2c_v3_cmpl_clear(base, CMPL_ERR | CMPL_HDONE | CMPL_IDLE);
		dma_stop(cfg->dma_dev, cfg->cm_dma_chan);
		xec_i2c_nl_finish_xfr(ctrl_dev, err);
		goto out;
	}

	if (cmpl & CMPL_HDONE) {
		uint32_t hcmd = sys_read32(base + XEC_I2C_HCMD_OFS);

		xec_i2c_v3_cmpl_clear(base, CMPL_HDONE);

		if ((hcmd & HCMD_RUN) && !(hcmd & HCMD_PROCEED)) {
			/* PAUSE: write phase done, direction switching to
			 * read. Arm the first read segment before letting
			 * the FSM proceed.
			 *
			 * Known gap: if this first read message is itself
			 * zero-length, this arms a zero-byte DMA block rather
			 * than skipping it the way xec_i2c_nl_seg_next() does
			 * for later segments -- not handled, see Task 3 notes.
			 */
			uint8_t msg_idx = data->cur_req.first_read_msg_idx;
			uintptr_t mem_addr;
			uint32_t len;
			int rc;

			xec_i2c_nl_seg_addr(data, XEC_I2C_NL_SEG_RMSG, msg_idx, &mem_addr, &len);
			rc = xec_i2c_nl_dma_arm_read(ctrl_dev, mem_addr, len, true);
			if (rc != 0) {
				dma_stop(cfg->dma_dev, cfg->cm_dma_chan);
				xec_i2c_nl_finish_xfr(ctrl_dev, rc);
				goto out;
			}
			data->cur_seg_kind = XEC_I2C_NL_SEG_RMSG;
			data->cur_seg_msg_idx = msg_idx;
			sys_set_bit(base + XEC_I2C_HCMD_OFS, XEC_I2C_HCMD_PROC_POS);
		} else if (!(hcmd & HCMD_RUN) && !(hcmd & HCMD_PROCEED)) {
			/* NL-finished for this HW request; bus not yet
			 * physically idle. v3.8 erratum: CFG.IDLE_IEN must
			 * not be armed while the bus is already idle (fires
			 * spuriously), so it is only turned on here, right
			 * before the closing STOP's idle transition.
			 */
			sys_set_bit(base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
		}
	}

	if (cmpl & CMPL_IDLE) {
		uint32_t cfgr = sys_read32(base + XEC_I2C_CFG_OFS);

		/* Only IDLE_IEN we ourselves armed above identifies this as
		 * the true end of the HW request we just finished; ignore
		 * any other IDLE source.
		 */
		if (cfgr & CFG_IDLE_IEN) {
			sys_clear_bit(base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
			xec_i2c_v3_cmpl_clear(base, CMPL_IDLE);

			if (data->next_start_idx >= data->xfr_num_msgs) {
				xec_i2c_nl_finish_xfr(ctrl_dev, 0);
			} else {
				int rc = xec_i2c_nl_arm_request(ctrl_dev);

				if (rc != 0) {
					dma_stop(cfg->dma_dev, cfg->cm_dma_chan);
					xec_i2c_nl_finish_xfr(ctrl_dev, rc);
				}
			}
		}
	}

out:
	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);
}

#ifdef CONFIG_PM_DEVICE
static int xec_i2c_nl_ctrl_pm_action_cb(const struct device *i2c_port, enum pm_device_action action)
{
	/* TODO */
	return -ENOTSUP;
}

static int xec_i2c_nl_vport_pm_action_cb(const struct device *i2c_port,
					 enum pm_device_action action)
{
	/* TODO */
	return -ENOTSUP;
}
#endif /* CONFIG_PM_DEVICE */

/* Driver initialization */
static int xec_i2c_nl_ctrl_init(const struct device *ctrl_dev)
{
	const struct xec_i2c_nl_config *ctrl_cfg = ctrl_dev->config;
	struct xec_i2c_nl_data *ctrl_data = ctrl_dev->data;

	ctrl_data->ctrl = ctrl_dev;
	k_sem_init(&ctrl_data->lock, 1, 1);
	k_sem_init(&ctrl_data->done_sem, 0, 1);

	/* TODO */

	if (ctrl_cfg->irq_connect != NULL) {
		ctrl_cfg->irq_connect();
	}

	return 0;
}

static int xec_i2c_nl_port_init(const struct device *port_dev)
{
	/* port_data->runtime_freq starts at 0 ("never configured by the
	 * app") from static zero-init; xec_i2c_nl_port_freq() already
	 * falls back to the port's DT clock-frequency in that case.
	 */
	return xec_i2c_nl_apply_port(port_dev);
}

static DEVICE_API(i2c, xec_i2c_nl_port_api) = {
	.configure = xec_i2c_nl_vport_config,
	.get_config = xec_i2c_nl_vport_get_config,
	.transfer = xec_i2c_nl_vport_xfr,
	/* TODO .recover_bus */
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = xec_i2c_nl_vport_xfr_cb,
#endif
#ifdef CONFIG_I2C_TARGET
	.target_register = xec_i2c_nl_vport_target_register,
	.target_unregister = xec_i2c_nl_vport_target_unregister,
#endif
};

/* Controller DT */
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
		.mr1 = DT_INST_PROP_BY_IDX(inst, prop, 5),                                         \
	}

#define XEC_I2C_NL_TIMING_DFLT(dtm, isc, tmo, bclk, rsht)                                          \
	{                                                                                          \
		.data_timing = (dtm), .idle_scaling = (isc), .timeout_scaling = (tmo),             \
		.bus_clock = (bclk), .rpt_start_hold_tm = (rsht),                                  \
		.mr1 = XEC_I2C_MR0_TM_BAUD16M,                                                     \
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

#define XEC_I2C_NL_TIMING_ASSERT(inst, prop)                                                           \
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

#ifdef CONFIG_I2C_TARGET
#define XEC_I2C_NL_HAS_TM_DMA(inst) DT_INST_DMAS_HAS_NAME(inst, target)
#define XEC_I2C_NL_HAS_TM_BUF(inst) DT_INST_NODE_HAS_PROP(inst, target_buffer_size)

#define XEC_I2C_NL_TM_BUF_DEF(inst)                                                                \
	COND_CODE_1(XEC_I2C_NL_HAS_TM_BUF(inst),                                                    \
		(BUILD_ASSERT(DT_INST_PROP(inst, target_buffer_size) <= 0xFFFF,                     \
			      "target-buffer-size exceeds 16-bit HW count");                        \
		 static uint8_t __aligned(4)                                                        \
			 xec_i2c_nl_tm_buf_##inst[DT_INST_PROP(inst, target_buffer_size)];),        \
		())

#define XEC_I2C_NL_TM_FIELDS(inst)                                                                  \
	.tm_rx_buf = COND_CODE_1(XEC_I2C_NL_HAS_TM_BUF(inst), (xec_i2c_nl_tm_buf_##inst), (NULL)),   \
	.tm_rx_buf_size = COND_CODE_1(XEC_I2C_NL_HAS_TM_BUF(inst),                                   \
				      (DT_INST_PROP(inst, target_buffer_size)), (0)),                \
	.tm_dma_valid = XEC_I2C_NL_HAS_TM_DMA(inst),                                                 \
	.tm_dma_chan = COND_CODE_1(XEC_I2C_NL_HAS_TM_DMA(inst),                                      \
				   (DT_INST_DMAS_CELL_BY_NAME(inst, target, channel)), (0)),         \
	.tm_dma_slot = COND_CODE_1(XEC_I2C_NL_HAS_TM_DMA(inst),                                      \
				   (DT_INST_DMAS_CELL_BY_NAME(inst, target, trigsrc)), (0)),         \
	.wakeup_source = DT_INST_PROP(inst, wakeup_source),
#else
#define XEC_I2C_NL_TM_BUF_DEF(inst)
#define XEC_I2C_NL_TM_FIELDS(inst)
#endif

#define XEC_I2C_NL_CTRL_INIT(inst)                                                                 \
	XEC_I2C_NL_DEFPORT_ASSERT(inst)                                                            \
	XEC_I2C_NL_TIMING_ASSERT(inst, timing_100k)                                                \
	XEC_I2C_NL_TIMING_ASSERT(inst, timing_400k)                                                \
	XEC_I2C_NL_TIMING_ASSERT(inst, timing_1000k)                                               \
	XEC_I2C_NL_TIMING_ASSERT(inst, timing_dt)                                                  \
	XEC_I2C_NL_TM_BUF_DEF(inst)                                                                 \
	static void xec_i2c_nl_irq_connect_##inst(void)                                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), xec_i2c_nl_isr,       \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
	static const struct xec_i2c_nl_config xec_i2c_nl_dcfg_##inst = {                           \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR(inst)),                                 \
		.irq_connect = xec_i2c_nl_irq_connect_##inst,                                      \
		.dflt_freq = XEC_I2C_NL_DFLT_FREQ(inst),                                           \
		.has_dt_timing = DT_INST_NODE_HAS_PROP(inst, timing_dt),                           \
		.girq = XEC_I2C_NL_GIRQ(inst, 0),                                                  \
		.girq_pos = XEC_I2C_NL_GIRQ_POS(inst, 0),                                          \
		.girq_wk = XEC_I2C_NL_GIRQ(inst, 1),                                               \
		.girq_wk_pos = XEC_I2C_NL_GIRQ_POS(inst, 1),                                       \
		.enc_pcr = DT_INST_PROP(inst, pcr_scr),                                            \
		.cm_dma_chan = DT_INST_DMAS_CELL_BY_NAME(inst, host, channel),                     \
		.cm_dma_slot = DT_INST_DMAS_CELL_BY_NAME(inst, host, trigsrc),                     \
		XEC_I2C_NL_TM_FIELDS(inst)                                                         \
		.timing = XEC_I2C_NL_TIMING_ROWS(inst), };                                         \
	static struct xec_i2c_nl_data xec_i2c_nl_data_##inst;                                      \
	PM_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_ctrl_pm_action_cb);                              \
	DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_ctrl_init, PM_DEVICE_DT_INST_GET(inst),             \
			      &xec_i2c_nl_data_##inst, &xec_i2c_nl_dcfg_##inst, POST_KERNEL,       \
			      CONFIG_I2C_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_CTRL_INIT)

/* Port DT */
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
	static const struct xec_i2c_nl_port_config xec_i2c_nl_port_dcfg_##inst = {                 \
		.controller = DEVICE_DT_GET(DT_INST_PHANDLE(inst, controller)),                    \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		.bitrate = DT_INST_PROP_OR(inst, clock_frequency, I2C_BITRATE_STANDARD),           \
		.port_id = (uint8_t)(DT_INST_PROP(inst, port) & 0x0FU),                            \
		.is_default = XEC_I2C_NL_PORT_IS_DEFAULT(inst),                                    \
	};                                                                                         \
	static struct xec_i2c_nl_port_data xec_i2c_nl_port_data_##inst;                            \
	PM_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_vport_pm_action_cb);                             \
	I2C_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_port_init, PM_DEVICE_DT_INST_GET(inst),         \
				  &xec_i2c_nl_port_data_##inst, &xec_i2c_nl_port_dcfg_##inst,      \
				  POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,                           \
				  &xec_i2c_nl_port_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_PORT_INIT)
