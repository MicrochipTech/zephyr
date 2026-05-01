/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/arch/common/sys_bitops.h"
#define DT_DRV_COMPAT microchip_xec_i2c_v3

#include <soc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_mchp_xec_v3, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"
#include "i2c_mchp_xec_v3.h"
#include "i2c_mchp_xec_regs.h"

/* Hardware needs ~15 ms for pin-state resync after reset + port/freq change.
 * README.txt calls out a safe range of 10-35 ms. This is a physical wait
 * for the controller's 16 MHz BAUD clock to observe stable pin states
 * through the port mux; shortening it risks the first transfer firing
 * into an unready controller.
 */
#define XEC_V3_PIN_RESYNC_US 15000U

/* Bus-free / transfer timeouts */
#define XEC_V3_BUS_FREE_TIMEOUT_US 10000U
#define XEC_V3_XFER_TIMEOUT_MS     100

/* Bit-bang recovery constants (same as v2) */
#define XEC_V3_BB_DELAY_US        5
#define XEC_V3_BB_SCL_SAMPLE_US   50
#define XEC_V3_BB_SCL_LOW_RETRIES 10
#define XEC_V3_BB_SDA_LOW_RETRIES 3

#define XEC_V3_PORT_NONE 0xFFU

/*
 * Most bits in the COMPLETION register are functional only in SMB
 * network-layer mode (when the Host Command / Target Command registers
 * drive block transfers). On-hardware testing confirmed that bits 2-6,
 * 8-12, 16-17, 19-21, 24-25, and 30-31 never activate in the byte-mode
 * transfers this driver performs. Only CMPL bits 13 (BER), 14 (LAB),
 * and 29 (IDLE) are useful here — and even then we prefer the STATUS
 * register for BER/LAB. The ISR therefore consumes CMPL.IDLE for
 * STOP-wait completion, and ignores the rest.
 */

/* I2C addr|R/W byte builders. Hardware is 7-bit only. */
#define XEC_V3_ADDR_W(a) (((uint8_t)((a) & 0x7FU)) << 1)
#define XEC_V3_ADDR_R(a) ((((uint8_t)((a) & 0x7FU)) << 1) | BIT(0))

struct xec_v3_timing {
	uint32_t freq_hz;
	uint32_t data_tm;
	uint32_t idle_sc;
	uint32_t tmout_sc;
	uint32_t bus_clock;
	uint8_t rpt_sta_htm;
};

/* Reused verbatim from v2: values are hardware-specific, not driver-specific. */
static const struct xec_v3_timing xec_v3_timing_tbl[] = {
	{KHZ(100), XEC_I2C_SMB_DATA_TM_100K, XEC_I2C_SMB_IDLE_SC_100K, XEC_I2C_SMB_TMO_SC_100K,
	 XEC_I2C_SMB_BUS_CLK_100K, XEC_I2C_SMB_RSHT_100K},
	{KHZ(400), XEC_I2C_SMB_DATA_TM_400K, XEC_I2C_SMB_IDLE_SC_400K, XEC_I2C_SMB_TMO_SC_400K,
	 XEC_I2C_SMB_BUS_CLK_400K, XEC_I2C_SMB_RSHT_400K},
	{MHZ(1), XEC_I2C_SMB_DATA_TM_1M, XEC_I2C_SMB_IDLE_SC_1M, XEC_I2C_SMB_TMO_SC_1M,
	 XEC_I2C_SMB_BUS_CLK_1M, XEC_I2C_SMB_RSHT_1M},
};

enum xec_v3_state {
	XEC_V3_IDLE,
	XEC_V3_ADDR_W,
	XEC_V3_ADDR_R,
	XEC_V3_WRITE,
	XEC_V3_READ,
	XEC_V3_STOP_WAIT,
	XEC_V3_DONE,
};

struct xec_v3_target_slot {
	struct i2c_target_config *cfg;
	bool in_use;
	bool buffer_mode;
};

struct i2c_mchp_xec_v3_cfg {
	mm_reg_t base_addr;
	uint8_t girq;
	uint8_t girq_pos;
	uint16_t enc_pcr;
	uint32_t irqn;
	void (*irq_config_func)(void);
#if defined(CONFIG_I2C_TARGET) && defined(CONFIG_I2C_TARGET_BUFFER_MODE)
	/* Per-instance target-mode RX buffer. Size comes from the
	 * "target-buffer-size" DT property so each controller can be tuned
	 * for its expected target-write payload. The backing storage is a
	 * static array declared alongside this config in the init macro.
	 */
	uint8_t *tgt_rx_buf;
	uint16_t tgt_rx_buf_size;
#endif
};

#ifdef CONFIG_I2C_MCHP_XEC_V3_INSTRUMENT_ISR
struct i2c_mchp_xec_v3_im {
	uint32_t cmpl;
	uint32_t cfg;
	uint8_t sr;
	uint8_t cr;
};
#endif

struct i2c_mchp_xec_v3_data {
	/* Binary semaphore used as a bus lock. Unlike k_mutex, k_sem_give()
	 * is ISR-safe, which matters for CONFIG_I2C_CALLBACK — the completion
	 * ISR needs to release the bus before invoking the user callback.
	 */
	struct k_sem bus_lock;
	struct k_sem xfer_done;

	/* Sticky port/freq state — 0xFF / 0 means "uninitialized". */
	uint8_t active_port;
	uint32_t active_freq;
	bool hw_enabled;

	/* Shadow copy of the I2C.CONTROL register. The hardware CR is
	 * write-only, so every CR write in the driver goes through
	 * xec_cr_wr() which mirrors the value here. Read this instead of
	 * the register when the driver needs to know the current CR state.
	 */
	uint8_t cr_shadow;

	/* Per-transfer state, valid only between transfer-start and completion. */
	struct i2c_msg *msgs;
	uint8_t num_msgs;
	uint8_t cur_msg;
	uint16_t buf_idx;
	uint16_t dev_addr;
	int xfer_err;
	enum xec_v3_state state;

#ifdef CONFIG_I2C_CALLBACK
	i2c_callback_t async_cb;
	void *async_userdata;
	bool async_active;
#endif

#ifdef CONFIG_I2C_TARGET
	/* Target mode is locked to a single port. First target_register()
	 * call pins target_port; subsequent registrations (up to HW's two
	 * own-address slots) must use the same port. Controller-mode
	 * transfers on a different port are refused while target_port is
	 * set — the app must target_unregister() first to free the port.
	 */
	uint8_t target_port; /* XEC_V3_PORT_NONE when no target is registered */
	struct xec_v3_target_slot targets[XEC_I2C_OA_NUM_TARGETS];
	bool target_busy;     /* mid-transaction in target mode */
	bool target_read_dir; /* true if external controller is reading */
	struct i2c_target_config *target_active;
	bool target_active_buf; /* true if active target uses buffer mode */
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	/* tgt_rx_buf / tgt_rx_buf_size live in the controller cfg because
	 * their size comes from DT. Only tgt_rx_len is runtime state.
	 */
	size_t tgt_rx_len;
	/* tgt_tx_buf is populated by the target's buf_read_requested callback,
	 * which hands us a pointer into its own memory. We only read from it;
	 * keeping it as plain uint8_t * avoids a const-cast at the callsite.
	 */
	uint8_t *tgt_tx_buf;
	uint32_t tgt_tx_len;
	uint32_t tgt_tx_idx;
#endif
#endif
#ifdef CONFIG_I2C_MCHP_XEC_V3_INSTRUMENT_ISR
	volatile uint32_t im_idx;
	struct i2c_mchp_xec_v3_im im[CONFIG_I2C_MCHP_XEC_V3_INSTRUMENT_ISR_ENTRIES];
#endif
#ifdef CONFIG_I2C_MCHP_XEC_V3_INSTRUMENT_STATE
	volatile uint32_t sm_idx;
	uint8_t sm[CONFIG_I2C_MCHP_XEC_V3_INSTRUMENT_STATE_ENTRIES];
#endif
};

#ifdef CONFIG_I2C_MCHP_XEC_V3_INSTRUMENT_ISR
static void im_clear(struct i2c_mchp_xec_v3_data *data)
{
	data->im_idx = 0;
	memset(data->im, 0,
	       (sizeof(struct i2c_mchp_xec_v3_im) * CONFIG_I2C_MCHP_XEC_V3_INSTRUMENT_ISR_ENTRIES));
}

static void im_update(struct i2c_mchp_xec_v3_data *data, uint32_t cmpl, uint32_t cfg, uint8_t sr,
		      uint8_t cr)
{
	if (data == NULL) {
		return;
	}

	uint32_t i = data->im_idx;

	if (i < CONFIG_I2C_MCHP_XEC_V3_INSTRUMENT_ISR_ENTRIES) {
		data->im[i].cmpl = cmpl;
		data->im[i].cfg = cfg;
		data->im[i].sr = sr;
		data->im[i].cr = cr;
		data->im_idx++;
	}
}
#else
static void im_clear(struct i2c_mchp_xec_v3_data *data)
{
}

static void im_update(struct i2c_mchp_xec_v3_data *data, uint32_t cmpl, uint32_t cfg, uint8_t sr,
		      uint8_t cr)
{
}
#endif

#ifdef CONFIG_I2C_MCHP_XEC_V3_INSTRUMENT_STATE
static void sm_clear(struct i2c_mchp_xec_v3_data *data)
{
	data->sm_idx = 0;
	memset(data->sm, 0, CONFIG_I2C_MCHP_XEC_V3_INSTRUMENT_STATE_ENTRIES);
}

static void sm_update(struct i2c_mchp_xec_v3_data *data, uint8_t tp)
{
	if (data == NULL) {
		return;
	}

	uint32_t i = data->sm_idx;

	if (i < CONFIG_I2C_MCHP_XEC_V3_INSTRUMENT_STATE_ENTRIES) {
		data->sm[i] = tp;
		data->sm_idx++;
	}
}
#else
static void sm_clear(struct i2c_mchp_xec_v3_data *data)
{
}

static void sm_update(struct i2c_mchp_xec_v3_data *data, uint8_t tp)
{
}
#endif

/* ------------------------------------------------------------------ */
/* Low-level register access                                          */
/* ------------------------------------------------------------------ */

static inline uint8_t xec_sr(mm_reg_t rb)
{
	return sys_read8(rb + XEC_I2C_SR_OFS);
}

/*
 * Write I2C.CONTROL and update the software shadow. CR is write-only in
 * hardware, so the shadow is the only way to know the current CR state.
 * All CR writes in this driver must go through this helper.
 */
static inline void xec_cr_wr(const struct device *ctrl, uint8_t cr)
{
	const struct i2c_mchp_xec_v3_cfg *cfg = ctrl->config;
	struct i2c_mchp_xec_v3_data *data = ctrl->data;

	data->cr_shadow = cr;
	sys_write8(cr, cfg->base_addr + XEC_I2C_CR_OFS);
}

static inline void xec_data_wr(mm_reg_t rb, uint8_t val)
{
	sys_write8(val, rb + XEC_I2C_DATA_OFS);
}

static inline uint8_t xec_data_rd(mm_reg_t rb)
{
	return sys_read8(rb + XEC_I2C_DATA_OFS);
}

/* ------------------------------------------------------------------ */
/* Port / frequency / timing                                          */
/* ------------------------------------------------------------------ */

static int xec_v3_prog_timing(mm_reg_t rb, uint32_t freq_hz)
{
	const struct xec_v3_timing *t = NULL;

	for (size_t i = 0; i < ARRAY_SIZE(xec_v3_timing_tbl); i++) {
		if (xec_v3_timing_tbl[i].freq_hz == freq_hz) {
			t = &xec_v3_timing_tbl[i];
			break;
		}
	}
	if (t == NULL) {
		return -EINVAL;
	}

	sys_write32(t->data_tm, rb + XEC_I2C_DT_OFS);
	sys_write32(t->idle_sc, rb + XEC_I2C_ISC_OFS);
	sys_write32(t->tmout_sc, rb + XEC_I2C_TMOUT_SC_OFS);
	sys_write32(t->bus_clock, rb + XEC_I2C_BCLK_OFS);
	sys_write8(t->rpt_sta_htm, rb + XEC_I2C_RSHT_OFS);

	return 0;
}

#ifdef CONFIG_I2C_TARGET
/*
 * Program the Own Address register for the currently active port.
 *
 * Hardware has two 7-bit address slots. Writing a non-zero value to a
 * slot enables target-mode match on that address; writing zero disables
 * the slot. (Zero would overlap the general-call address 0x00, but the
 * driver sets CFG.GC_DIS=1 in xec_v3_reconfigure so general-call match
 * stays off regardless.)
 *
 * Since target mode is pinned to target_port, OA is programmed only
 * when active_port == target_port. On any other active port OA stays
 * zero so the controller never responds to target addresses while the
 * pins aren't the target's port anyway.
 */
static void xec_v3_apply_target_oa(const struct device *ctrl)
{
	const struct i2c_mchp_xec_v3_cfg *cfg = ctrl->config;
	struct i2c_mchp_xec_v3_data *data = ctrl->data;
	uint32_t oa = 0;

	if (data->target_port != XEC_V3_PORT_NONE && data->target_port == data->active_port) {
		for (int i = 0; i < XEC_I2C_OA_NUM_TARGETS; i++) {
			uint8_t a = 0;

			if (data->targets[i].in_use) {
				a = (uint8_t)data->targets[i].cfg->address & 0x7FU;
			}
			oa |= XEC_I2C_OA_SET(i, a);
		}
	}
	sys_write32(oa, cfg->base_addr + XEC_I2C_OA_OFS);
}
#endif /* CONFIG_I2C_TARGET */

/*
 * Full reconfigure: PCR reset → CFG(port,FEN) → timing → enable → 15 ms resync.
 * Called only when the active port or frequency changes, or on error recovery.
 * Must be called with bus_lock held.
 */
static int xec_v3_reconfigure(const struct device *ctrl, uint8_t port, uint32_t freq_hz)
{
	const struct i2c_mchp_xec_v3_cfg *cfg = ctrl->config;
	struct i2c_mchp_xec_v3_data *data = ctrl->data;
	mm_reg_t rb = cfg->base_addr;
	int ret;

	irq_disable(cfg->irqn);

	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, 0);
	soc_xec_pcr_reset_en(cfg->enc_pcr);

	/* CFG: port + FEN + GC_DIS + STD_IEN; controller still disabled.
	 *
	 * STD_IEN (bit 24 of CONFIGURATION — distinct from CMPL bit 24
	 * HNAKX, which is SMB-mode only) enables byte-mode STOP detection:
	 * with it set, hardware sets STATUS.STO when a STOP condition is
	 * detected in both Controller and Target modes. Without it the
	 * STS bit never asserts, and target_stop / STOP_WAIT completion
	 * have to fall back to NBB alone.
	 */
	sys_write32((BIT(XEC_I2C_CFG_GC_DIS_POS) | BIT(XEC_I2C_CFG_FEN_POS) |
		     XEC_I2C_CFG_PORT_SET((uint32_t)port)),
		    rb + XEC_I2C_CFG_OFS);

	ret = xec_v3_prog_timing(rb, freq_hz);
	if (ret != 0) {
		LOG_ERR("i2c v3: unsupported freq %u Hz", freq_hz);
		return ret;
	}

	/* PIN=1 clears status. ESO+ACK+ENI is the idle CR state: output
	 * drivers on, auto-ACK on, PIN service interrupt enabled. ENI is
	 * always on — it only fires on real events (address match, data
	 * byte ready, start/stop completion). The target-mode
	 * address-match logic is gated by the OWN_ADDRESS register alone:
	 * writing a non-zero 7-bit address there enables match detection,
	 * and a hit sets STATUS.AAT=1 + STATUS.PIN=0, which routes through
	 * ENI to the NVIC. The AAT_IEN bit in CFG is not needed.
	 */
	xec_cr_wr(ctrl, (BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) |
			 BIT(XEC_I2C_CR_ACK_POS) | BIT(XEC_I2C_CR_ENI_POS)));

	/* Enable controller. */
	sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);
	data->hw_enabled = true;

	/* Physical resync: 16 MHz BAUD clock must observe stable pin state
	 * through the newly muxed port. Hardware requirement (README.txt
	 * lists 10–35 ms as safe). This driver initializes at POST_KERNEL
	 * so k_busy_wait is valid here — its underlying timer infrastructure
	 * is not available at PRE_KERNEL_1 on XEC.
	 */
	k_busy_wait(XEC_V3_PIN_RESYNC_US);

	data->active_port = port;
	data->active_freq = freq_hz;

#ifdef CONFIG_I2C_TARGET
	/* Programming OWN_ADDRESS is sufficient to arm target-mode match. */
	xec_v3_apply_target_oa(ctrl);
#endif

	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);
	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, 1U);
	irq_enable(cfg->irqn);

	return 0;
}

static int xec_v3_select(const struct device *ctrl, uint8_t port, uint32_t freq_hz)
{
	struct i2c_mchp_xec_v3_data *data = ctrl->data;

#ifdef CONFIG_I2C_TARGET
	/* While target mode is registered on a port, the controller is
	 * pinned to that port — switching would reprogram pinctrl + OA
	 * and drop target-mode visibility on the original port's bus.
	 * Caller must unregister to free the port.
	 */
	if (data->target_port != XEC_V3_PORT_NONE && data->target_port != port) {
		return -EBUSY;
	}
#endif

	if (data->hw_enabled && data->active_port == port && data->active_freq == freq_hz) {
		return 0;
	}
	return xec_v3_reconfigure(ctrl, port, freq_hz);
}

/* ------------------------------------------------------------------ */
/* Controller-mode helpers                                            */
/* ------------------------------------------------------------------ */

static void xec_v3_gen_start(const struct device *ctrl, uint8_t addr_byte)
{
	const struct i2c_mchp_xec_v3_cfg *cfg = ctrl->config;

	xec_data_wr(cfg->base_addr, addr_byte);
	xec_cr_wr(ctrl, BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) |
				BIT(XEC_I2C_CR_STA_POS) | BIT(XEC_I2C_CR_ACK_POS) |
				BIT(XEC_I2C_CR_ENI_POS));
}

static void xec_v3_gen_rstart(const struct device *ctrl, uint8_t addr_byte)
{
	const struct i2c_mchp_xec_v3_cfg *cfg = ctrl->config;

	/* Repeated-start: do NOT set PIN — clock runs straight into the
	 * next address phase without releasing the bus.
	 */
	xec_cr_wr(ctrl, BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_STA_POS) |
				BIT(XEC_I2C_CR_ACK_POS) | BIT(XEC_I2C_CR_ENI_POS));
	xec_data_wr(cfg->base_addr, addr_byte);
}

static void xec_v3_gen_stop(const struct device *ctrl)
{
	xec_cr_wr(ctrl, BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) |
				BIT(XEC_I2C_CR_STO_POS) | BIT(XEC_I2C_CR_ACK_POS) |
				BIT(XEC_I2C_CR_ENI_POS));
}

static int xec_v3_wait_bus_free(mm_reg_t rb, uint32_t timeout_us)
{
	if (!WAIT_FOR((xec_sr(rb) & BIT(XEC_I2C_SR_NBB_POS)) != 0, timeout_us, NULL)) {
		return -ETIMEDOUT;
	}
	return 0;
}

/*
 * Kick off a message (first or after advance). Programs START/RESTART as
 * appropriate, transitions state, and returns. ISR drives subsequent bytes.
 */
static void xec_v3_kick_msg(const struct device *ctrl)
{
	const struct i2c_mchp_xec_v3_cfg *cfg = ctrl->config;
	struct i2c_mchp_xec_v3_data *data = ctrl->data;
	struct i2c_msg *m = &data->msgs[data->cur_msg];
	mm_reg_t rb = cfg->base_addr;
	bool is_read = (m->flags & I2C_MSG_READ) != 0;
	uint8_t ab = is_read ? XEC_V3_ADDR_R(data->dev_addr) : XEC_V3_ADDR_W(data->dev_addr);

	sm_update(data, 0x20U);

	data->buf_idx = 0;

	if (data->cur_msg == 0 || (m->flags & I2C_MSG_RESTART) != 0) {
		if (data->cur_msg == 0) {
			sm_update(data, 0x21U);
			/* clear latched IDLE status before START generation */
			sys_set_bit(rb + XEC_I2C_CMPL_OFS, XEC_I2C_CMPL_IDLE_POS);
			xec_v3_gen_start(ctrl, ab);
		} else {
			sm_update(data, 0x22U);
			xec_v3_gen_rstart(ctrl, ab);
		}
		data->state = is_read ? XEC_V3_ADDR_R : XEC_V3_ADDR_W;
		sm_update(data, 0x23U);
		return;
	}

	sm_update(data, 0x24U);

	/* Same direction, no restart — continue the current phase. Rare,
	 * only valid when prior msg did not have STOP and direction matches.
	 */
	data->state = is_read ? XEC_V3_READ : XEC_V3_WRITE;
	if (!is_read && m->len > 0) {
		sm_update(data, 0x25U);
		xec_data_wr(rb, m->buf[0]);
		data->buf_idx = 1;
	} else if (is_read) {
		sm_update(data, 0x26U);
		/* No address-cycle discard needed here — we never left READ. */
		(void)xec_data_rd(rb); /* kick clocks for next byte */
	}

	sm_update(data, 0x27U);
}

static void xec_v3_finish(const struct device *ctrl, int err)
{
	struct i2c_mchp_xec_v3_data *data = ctrl->data;

	data->xfer_err = err;
	data->state = XEC_V3_DONE;

	sm_update(data, 0x30U);

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	data->tgt_tx_buf = NULL;
	data->tgt_tx_len = 0;
	data->tgt_tx_idx = 0;
	data->tgt_rx_len = 0;
#endif

#ifdef CONFIG_I2C_CALLBACK
	if (data->async_active) {
		sm_update(data, 0x31U);
		i2c_callback_t cb = data->async_cb;
		void *ud = data->async_userdata;

		data->async_active = false;
		data->async_cb = NULL;
		data->async_userdata = NULL;
		k_sem_give(&data->bus_lock);
		if (cb != NULL) {
			sm_update(data, 0x32U);
			cb(ctrl, err, ud);
		}
		return;
	}
#endif
	k_sem_give(&data->xfer_done);

	sm_update(data, 0x33U);
}

/* Advance to next message, or STOP + finish if done. */
static void xec_v3_advance(const struct device *ctrl)
{
	const struct i2c_mchp_xec_v3_cfg *cfg = ctrl->config;
	struct i2c_mchp_xec_v3_data *data = ctrl->data;
	struct i2c_msg *m = &data->msgs[data->cur_msg];
	mm_reg_t rb = cfg->base_addr;

	sm_update(data, 0x40U);

	if (data->cur_msg + 1U < data->num_msgs) {
		sm_update(data, 0x41U);
		data->cur_msg++;
		xec_v3_kick_msg(ctrl);
		return;
	}

	if ((m->flags & I2C_MSG_STOP) != 0) {
		sm_update(data, 0x42U);
		xec_v3_gen_stop(ctrl);
		data->state = XEC_V3_STOP_WAIT;
		/* IDLE interrupt will fire when bus goes NBB=1. */
		sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
		return;
	}

	sm_update(data, 0x43U);

	/* No STOP flag: leave bus open, complete transfer. */
	xec_v3_finish(ctrl, 0);
}

/* ------------------------------------------------------------------ */
/* Target-mode ISR helpers                                            */
/* ------------------------------------------------------------------ */

#ifdef CONFIG_I2C_TARGET
static struct i2c_target_config *xec_v3_match_target(struct i2c_mchp_xec_v3_data *data,
						     uint8_t addr7, bool *is_buffer)
{
	if (data->target_port == XEC_V3_PORT_NONE || data->target_port != data->active_port) {
		return NULL;
	}
	for (int i = 0; i < XEC_I2C_OA_NUM_TARGETS; i++) {
		struct xec_v3_target_slot *s = &data->targets[i];

		if (s->in_use && (s->cfg->address & 0x7FU) == addr7) {
			*is_buffer = s->buffer_mode;
			return s->cfg;
		}
	}
	return NULL;
}

static void xec_v3_target_set_nack(const struct device *ctrl)
{
	/* Clear ACK bit so next received byte is NACKed. */
	xec_cr_wr(ctrl,
		  BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ENI_POS));
}

static void xec_v3_target_restart(const struct device *ctrl)
{
	struct i2c_mchp_xec_v3_data *const data = ctrl->data;

	sm_update(data, 0xD0U);

	/* Re-arm: PIN=1 clears status, ACK on for next addr match. */
	xec_cr_wr(ctrl, BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) |
				BIT(XEC_I2C_CR_ACK_POS) | BIT(XEC_I2C_CR_ENI_POS));
}

static void xec_v3_target_addr_phase(const struct device *ctrl)
{
	const struct i2c_mchp_xec_v3_cfg *cfg = ctrl->config;
	struct i2c_mchp_xec_v3_data *data = ctrl->data;
	mm_reg_t rb = cfg->base_addr;
	/* Reading DATA in target mode returns the matched address byte
	 * (addr<<1 | R/W) and clears AAT/SAD/LRB status. The separate IAS
	 * shadow register holds the same byte without side-effects, but
	 * since we want to clear AAT here anyway, DATA is enough.
	 */
	uint8_t rx_data = xec_data_rd(rb);
	uint8_t addr7 = (rx_data >> 1) & 0x7FU;
	bool is_read = (rx_data & BIT(0)) != 0;
	bool is_buf = false;
	struct i2c_target_config *tcfg = xec_v3_match_target(data, addr7, &is_buf);
	const struct i2c_target_callbacks *cbs = (tcfg != NULL) ? tcfg->callbacks : NULL;

	sm_update(data, 0x90U);

	sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_STD_IEN_POS);

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	/* Flush on Repeated-START boundary: if we arrive here while the
	 * previous phase was a buffer-mode write with bytes accumulated,
	 * deliver them to the app before switching to the new direction.
	 * The external controller completed its write phase and is now
	 * starting a new one (commonly W→R for i2c_write_read protocols);
	 * without this flush buf_write_received would never fire for the
	 * data written before the Rep-START.
	 */
	if (data->target_busy && data->target_active_buf && !data->target_read_dir &&
	    data->tgt_rx_len > 0 && data->target_active != NULL &&
	    data->target_active->callbacks != NULL &&
	    data->target_active->callbacks->buf_write_received != NULL) {
		sm_update(data, 0x91U);
		data->target_active->callbacks->buf_write_received(
			data->target_active, cfg->tgt_rx_buf, (uint32_t)data->tgt_rx_len);
		data->tgt_rx_len = 0;
	}
#endif

	data->target_active = tcfg;
	data->target_active_buf = is_buf;
	data->target_read_dir = is_read;
	data->target_busy = true;

	if (tcfg == NULL || cbs == NULL) {
		sm_update(data, 0x92U);
		xec_v3_target_set_nack(ctrl);
		return;
	}

	if (is_read) {
		uint8_t val = 0;

		sm_update(data, 0x93U);

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
		if (is_buf && cbs->buf_read_requested != NULL) {
			int rc =
				cbs->buf_read_requested(tcfg, &data->tgt_tx_buf, &data->tgt_tx_len);

			sm_update(data, 0x94U);

			data->tgt_tx_idx = 0;
			if (rc != 0 || data->tgt_tx_buf == NULL || data->tgt_tx_len == 0) {
				sm_update(data, 0x95U);
				xec_v3_target_set_nack(ctrl);
				return;
			}
			val = data->tgt_tx_buf[data->tgt_tx_idx++];
			xec_data_wr(rb, val);
			return;
		}
#endif
		if (cbs->read_requested != NULL) {
			cbs->read_requested(tcfg, &val);
		}
		xec_data_wr(rb, val);
	} else {
		sm_update(data, 0x96U);
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
		if (is_buf) {
			sm_update(data, 0x97U);
			data->tgt_rx_len = 0;
			/* Leave ACK enabled; bytes accumulate in tgt_rx_buf. */
			return;
		}
#endif
		if (cbs->write_requested != NULL && cbs->write_requested(tcfg) != 0) {
			sm_update(data, 0x98U);
			xec_v3_target_set_nack(ctrl);
		}
	}
}

static void xec_v3_target_data_phase(const struct device *ctrl)
{
	const struct i2c_mchp_xec_v3_cfg *cfg = ctrl->config;
	struct i2c_mchp_xec_v3_data *data = ctrl->data;
	mm_reg_t rb = cfg->base_addr;
	struct i2c_target_config *tcfg = data->target_active;
	const struct i2c_target_callbacks *cbs = (tcfg != NULL) ? tcfg->callbacks : NULL;

	sm_update(data, 0xA0U);

	if (tcfg == NULL || cbs == NULL) {
		sm_update(data, 0xA1U);
		xec_v3_target_set_nack(ctrl);
		return;
	}

	if (data->target_read_dir) {
		/* External controller is reading — supply next byte, or default
		 * after it NACKs (LRB_AD0=1 in SR means last byte was NACKed).
		 */
		uint8_t val = 0;

		sm_update(data, 0xA2U);

		if ((xec_sr(rb) & BIT(XEC_I2C_SR_LRB_AD0_POS)) != 0) {
			/* External master NACKed — push a dummy byte to release
			 * SCL; STOP will come next.
			 */
			sm_update(data, 0xA3U);
			xec_data_wr(rb, 0);
			return;
		}

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
		if (data->target_active_buf && data->tgt_tx_buf != NULL) {
			sm_update(data, 0xA4U);
			if (data->tgt_tx_idx < data->tgt_tx_len) {
				sm_update(data, 0xA5U);
				val = data->tgt_tx_buf[data->tgt_tx_idx++];
			}
			xec_data_wr(rb, val);
			return;
		}
#endif
		if (cbs->read_processed != NULL) {
			sm_update(data, 0xA6U);
			cbs->read_processed(tcfg, &val);
		}
		xec_data_wr(rb, val);
	} else {
		uint8_t v = xec_data_rd(rb);

		sm_update(data, 0xA7U);

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
		if (data->target_active_buf) {
			if (data->tgt_rx_len < cfg->tgt_rx_buf_size) {
				sm_update(data, 0xA8U);
				cfg->tgt_rx_buf[data->tgt_rx_len++] = v;
			} else {
				sm_update(data, 0xA9U);
				xec_v3_target_set_nack(ctrl);
			}
			return;
		}
#endif
		if (cbs->write_received == NULL || cbs->write_received(tcfg, v) != 0) {
			sm_update(data, 0xAAU);
			xec_v3_target_set_nack(ctrl);
		}
	}
}

static void xec_v3_target_stop(const struct device *ctrl)
{
	const struct i2c_mchp_xec_v3_cfg *cfg = ctrl->config;
	struct i2c_mchp_xec_v3_data *data = ctrl->data;
	struct i2c_target_config *tcfg = data->target_active;
	const struct i2c_target_callbacks *cbs = (tcfg != NULL) ? tcfg->callbacks : NULL;

	/* STATUS.STO was already cleared at the top of the ISR via the
	 * unconditional DATA discard for any STO-bearing interrupt.
	 */

	sm_update(data, 0xC0U);

	sys_clear_bit(cfg->base_addr + XEC_I2C_CFG_OFS, XEC_I2C_CFG_STD_IEN_POS);

	if (tcfg != NULL && cbs != NULL) {
		sm_update(data, 0xC1U);
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
		if (data->target_active_buf && !data->target_read_dir &&
		    cbs->buf_write_received != NULL && data->tgt_rx_len > 0) {
			sm_update(data, 0xC2U);
			cbs->buf_write_received(tcfg, cfg->tgt_rx_buf, (uint32_t)data->tgt_rx_len);
			data->tgt_rx_len = 0;
		}
#endif
		if (cbs->stop != NULL) {
			sm_update(data, 0xC3U);
			cbs->stop(tcfg);
		}
	}

	im_clear(data);
	sm_clear(data);

	data->target_busy = false;
	data->target_active = NULL;

	sm_update(data, 0xC4U);
	xec_v3_target_restart(ctrl);
}
#endif /* CONFIG_I2C_TARGET */

/* ------------------------------------------------------------------ */
/* ISR                                                                */
/* ------------------------------------------------------------------ */

static void xec_v3_isr(const struct device *ctrl)
{
	const struct i2c_mchp_xec_v3_cfg *cfg = ctrl->config;
	struct i2c_mchp_xec_v3_data *data = ctrl->data;
	mm_reg_t rb = cfg->base_addr;
	uint8_t sr = xec_sr(rb);
	uint32_t compl = sys_read32(rb + XEC_I2C_CMPL_OFS);
	uint32_t i2c_cfg = sys_read32(rb + XEC_I2C_CFG_OFS);

	im_update(data, compl, i2c_cfg, sr, data->cr_shadow);
	sm_update(data, 0x80U);

	/* W1C the R/W1C status bits we just snapshotted. Most of RW1C_MSK
	 * covers network-layer status bits that never activate in byte
	 * mode; writing 1 to them is a harmless no-op.
	 */
	sys_write32(compl & XEC_I2C_CMPL_RW1C_MSK, rb + XEC_I2C_CMPL_OFS);
	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);

	/* STOP detected on the bus — including STOPs from transactions we
	 * are not part of (hardware quirk: with CFG.STD_IEN set the
	 * controller detects every bus STOP, not just its own). STATUS.STO
	 * is read-only; the only way to clear it (and release PIN 0→1 so
	 * the GIRQ source stops asserting) is to read and discard DATA.
	 * Do it unconditionally here so every code path — including the
	 * "spurious STOP while idle" path that otherwise falls through
	 * every dispatcher and infinite-loops — gets its STO cleared.
	 * Snapshot sr is still valid for downstream dispatch decisions.
	 */
	if ((sr & BIT(XEC_I2C_SR_STO_POS)) != 0) {
		/* disable STOP detection */
		sm_update(data, 0x81U);
		sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_STD_IEN_POS);
		(void)xec_data_rd(rb);
	}

	/* Bus error / arbitration loss trumps everything. */
	if ((sr & (BIT(XEC_I2C_SR_BER_POS) | BIT(XEC_I2C_SR_LAB_POS))) != 0) {
		sm_update(data, 0x82U);
#ifdef CONFIG_I2C_TARGET
		if (data->target_busy) {
			struct i2c_target_config *tcfg = data->target_active;

			if (tcfg != NULL && tcfg->callbacks != NULL &&
			    tcfg->callbacks->error != NULL) {
				tcfg->callbacks->error(tcfg, (sr & BIT(XEC_I2C_SR_LAB_POS))
								     ? I2C_ERROR_ARBITRATION
								     : I2C_ERROR_GENERIC);
			}
			xec_v3_target_stop(ctrl);
			return;
		}
#endif
		if (data->state != XEC_V3_IDLE && data->state != XEC_V3_DONE) {
			/* Force full reset on next transfer. */
			data->active_port = XEC_V3_PORT_NONE;
			data->hw_enabled = false;
			xec_v3_finish(ctrl, (sr & BIT(XEC_I2C_SR_LAB_POS)) ? -EAGAIN : -EIO);
		}
		return;
	}

	/* STOP_WAIT completion. Any of IDLE / NBB / STO signals transfer
	 * end. STATUS.STO was already cleared at the top of the ISR.
	 */
	if (data->state == XEC_V3_STOP_WAIT) {
		sm_update(data, 0x83U);
		if ((compl & BIT(XEC_I2C_CMPL_IDLE_POS)) != 0 ||
		    (sr & BIT(XEC_I2C_SR_NBB_POS)) != 0 || (sr & BIT(XEC_I2C_SR_STO_POS)) != 0) {
			sys_clear_bit(rb + XEC_I2C_CMPL_OFS, XEC_I2C_CMPL_IDLE_POS);
			sys_clear_bits(rb + XEC_I2C_CFG_OFS, (BIT(XEC_I2C_CFG_IDLE_IEN_POS) |
							      BIT(XEC_I2C_CFG_STD_IEN_POS)));
			xec_v3_finish(ctrl, 0);
		}
		return;
	}

	sm_update(data, 0x84U);

#ifdef CONFIG_I2C_TARGET
	/* Re-sample STATUS before dispatching target-mode work. The initial
	 * sr read at the top of the ISR can race ahead of AAT settling when
	 * a Repeated-START arrives: the PIN service asserts for the new
	 * transaction before the matched-address bit has propagated into
	 * STATUS.AAT. Dispatching on the stale sr would land us in
	 * data_phase, which would read DATA and treat the newly-latched
	 * address byte as if it were data (observed as tgt_rx_len
	 * over-counting by one on W→R transitions). The CMPL register's
	 * RPT_WR/RPT_RD bits would normally signal this boundary, but they
	 * are SMB network-layer flags that do not activate in the byte-mode
	 * transfers this driver uses.
	 */
	sr = xec_sr(rb);

	sm_update(data, 0x85U);

	/* Target-mode paths. AAT valid only when PIN==0. */
	if ((sr & (BIT(XEC_I2C_SR_AAT_POS) | BIT(XEC_I2C_SR_PIN_POS))) == BIT(XEC_I2C_SR_AAT_POS)) {
		xec_v3_target_addr_phase(ctrl);
		return;
	}
	if (data->target_busy) {
		sm_update(data, 0x86U);
		if ((sr & BIT(XEC_I2C_SR_STO_POS)) != 0 || (sr & BIT(XEC_I2C_SR_NBB_POS)) != 0) {
			xec_v3_target_stop(ctrl);
		} else {
			xec_v3_target_data_phase(ctrl);
		}
		return;
	}
#endif

	/* Controller-mode state machine. ISR fires on PIN 1→0. */
	if (data->state == XEC_V3_IDLE || data->state == XEC_V3_DONE) {
		/* TODO disable all interrupts? */
		sm_update(data, 0x87U);
		return; /* spurious — no transfer in progress */
	}

	struct i2c_msg *m = &data->msgs[data->cur_msg];

	switch (data->state) {
	case XEC_V3_ADDR_W:
		sm_update(data, 0x88U);
		if ((sr & BIT(XEC_I2C_SR_LRB_AD0_POS)) != 0) {
			xec_v3_gen_stop(ctrl);
			data->state = XEC_V3_STOP_WAIT;
			sys_set_bits(rb + XEC_I2C_CFG_OFS, (BIT(XEC_I2C_CFG_IDLE_IEN_POS) |
							    BIT(XEC_I2C_CFG_STD_IEN_POS)));
			data->xfer_err = -EIO; /* address NACK */
			break;
		}
		data->state = XEC_V3_WRITE;
		if (m->len == 0) {
			xec_v3_advance(ctrl);
			break;
		}
		xec_data_wr(rb, m->buf[0]);
		data->buf_idx = 1;
		break;

	case XEC_V3_WRITE:
		sm_update(data, 0x89U);
		if ((sr & BIT(XEC_I2C_SR_LRB_AD0_POS)) != 0) {
			xec_v3_gen_stop(ctrl);
			data->state = XEC_V3_STOP_WAIT;
			sys_set_bits(rb + XEC_I2C_CFG_OFS, (BIT(XEC_I2C_CFG_IDLE_IEN_POS) |
							    BIT(XEC_I2C_CFG_STD_IEN_POS)));
			data->xfer_err = -EIO;
			break;
		}
		if (data->buf_idx < m->len) {
			xec_data_wr(rb, m->buf[data->buf_idx++]);
			break;
		}
		xec_v3_advance(ctrl);
		break;

	case XEC_V3_ADDR_R:
		sm_update(data, 0x8AU);
		if ((sr & BIT(XEC_I2C_SR_LRB_AD0_POS)) != 0) {
			xec_v3_gen_stop(ctrl);
			data->state = XEC_V3_STOP_WAIT;
			sys_set_bits(rb + XEC_I2C_CFG_OFS, (BIT(XEC_I2C_CFG_IDLE_IEN_POS) |
							    BIT(XEC_I2C_CFG_STD_IEN_POS)));
			data->xfer_err = -EIO;
			break;
		}
		/* Address is in DATA; first read discards it and clocks out the
		 * first data byte (read-ahead). If this is a zero-length read,
		 * advance without reading.
		 */
		if (m->len == 0) {
			xec_v3_advance(ctrl);
			break;
		}
		if (m->len == 1 && (m->flags & I2C_MSG_STOP) != 0) {
			/* Single-byte read + STOP: disable ACK before the
			 * discard read so byte[0] — which the discard clocks
			 * out — gets NACKed on receipt.
			 */
			xec_cr_wr(ctrl, BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ENI_POS));
		}
		/* For len>=2, leave ACK enabled here; CTRL_READ handles the
		 * disable-before-last-byte sequence as bytes stream in.
		 */
		(void)xec_data_rd(rb); /* discard address; kicks clocks for byte[0] */
		data->state = XEC_V3_READ;
		break;

	case XEC_V3_READ: {
		uint16_t remaining = (uint16_t)(m->len - data->buf_idx);

		sm_update(data, 0x8BU);

		if (remaining == 1 && (m->flags & I2C_MSG_STOP) != 0) {
			/* Last byte: issue STOP, then read DATA (no clocks). */
			xec_v3_gen_stop(ctrl);
			m->buf[data->buf_idx++] = xec_data_rd(rb);
			data->state = XEC_V3_STOP_WAIT;
			sys_set_bits(rb + XEC_I2C_CFG_OFS, (BIT(XEC_I2C_CFG_IDLE_IEN_POS) |
							    BIT(XEC_I2C_CFG_STD_IEN_POS)));
			break;
		}
		if (remaining == 2 && (m->flags & I2C_MSG_STOP) != 0) {
			/* Before second-to-last: disable ACK so last byte is NACKed. */
			xec_cr_wr(ctrl, BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ENI_POS));
		}
		m->buf[data->buf_idx++] = xec_data_rd(rb);
		if (data->buf_idx >= m->len) {
			xec_v3_advance(ctrl);
		}
		break;
	}

	default:
		sm_update(data, 0x8CU);
		/* Spurious interrupt — clear and move on. */
		break;
	}

	sm_update(data, 0x8FU);
}

/* ------------------------------------------------------------------ */
/* Bus recovery (BBCR bit-bang; no GPIO driver)                       */
/* ------------------------------------------------------------------ */

static int xec_v3_bb_recover(mm_reg_t rb)
{
	uint8_t bb;
	int rc = -EBUSY;

	bb = BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_SCL_POS) | BIT(XEC_I2C_BBCR_SDA_POS);
	sys_write8(bb, rb + XEC_I2C_BBCR_OFS);

	/* If SCL is stuck low, bail. */
	if ((sys_read8(rb + XEC_I2C_BBCR_OFS) & BIT(XEC_I2C_BBCR_SCL_IN_POS)) == 0) {
		for (int i = 0; i < XEC_V3_BB_SCL_LOW_RETRIES; i++) {
			k_busy_wait(XEC_V3_BB_SCL_SAMPLE_US);
			if ((sys_read8(rb + XEC_I2C_BBCR_OFS) & BIT(XEC_I2C_BBCR_SCL_IN_POS)) !=
			    0) {
				goto scl_ok;
			}
		}
		rc = -EBUSY;
		goto out;
	}

scl_ok:
	if ((sys_read8(rb + XEC_I2C_BBCR_OFS) & BIT(XEC_I2C_BBCR_SDA_IN_POS)) != 0) {
		rc = 0;
		goto out;
	}

	/* Toggle SCL up to 9 times per attempt to clock out a stuck slave. */
	for (int attempt = 0; attempt < XEC_V3_BB_SDA_LOW_RETRIES; attempt++) {
		bb = BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CD_POS) |
		     BIT(XEC_I2C_BBCR_SCL_POS) | BIT(XEC_I2C_BBCR_SDA_POS);
		sys_write8(bb, rb + XEC_I2C_BBCR_OFS);
		k_busy_wait(XEC_V3_BB_DELAY_US);

		for (int j = 0; j < 9; j++) {
			if ((sys_read8(rb + XEC_I2C_BBCR_OFS) & BIT(XEC_I2C_BBCR_SDA_IN_POS)) !=
			    0) {
				break;
			}
			bb = BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CD_POS) |
			     BIT(XEC_I2C_BBCR_SDA_POS);
			sys_write8(bb, rb + XEC_I2C_BBCR_OFS);
			k_busy_wait(XEC_V3_BB_DELAY_US);
			bb = BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CD_POS) |
			     BIT(XEC_I2C_BBCR_SCL_POS) | BIT(XEC_I2C_BBCR_SDA_POS);
			sys_write8(bb, rb + XEC_I2C_BBCR_OFS);
			k_busy_wait(XEC_V3_BB_DELAY_US);
		}

		/* Manufacture a STOP on the bit-bang lines. */
		bb = BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_SCL_POS) |
		     BIT(XEC_I2C_BBCR_DD_POS);
		sys_write8(bb, rb + XEC_I2C_BBCR_OFS);
		k_busy_wait(XEC_V3_BB_DELAY_US);
		bb = BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_SCL_POS) |
		     BIT(XEC_I2C_BBCR_SDA_POS);
		sys_write8(bb, rb + XEC_I2C_BBCR_OFS);
		k_busy_wait(XEC_V3_BB_DELAY_US);

		bb = sys_read8(rb + XEC_I2C_BBCR_OFS) &
		     (BIT(XEC_I2C_BBCR_SCL_IN_POS) | BIT(XEC_I2C_BBCR_SDA_IN_POS));
		if (bb == (BIT(XEC_I2C_BBCR_SCL_IN_POS) | BIT(XEC_I2C_BBCR_SDA_IN_POS))) {
			rc = 0;
			goto out;
		}
	}

out:
	/* Setting CM reconnects SCL/SDA to the I2C logic and enables monitor
	 * mode. Required before the controller can drive the bus again.
	 */
	sys_write8(BIT(XEC_I2C_BBCR_CM_POS), rb + XEC_I2C_BBCR_OFS);
	return rc;
}

/* ------------------------------------------------------------------ */
/* Public API (port driver → controller driver)                       */
/* ------------------------------------------------------------------ */

void mchp_xec_i2c_ctrl_mutex_lock(const struct device *ctrl)
{
	struct i2c_mchp_xec_v3_data *data = ctrl->data;

	k_sem_take(&data->bus_lock, K_FOREVER);
}

void mchp_xec_i2c_ctrl_mutex_unlock(const struct device *ctrl)
{
	struct i2c_mchp_xec_v3_data *data = ctrl->data;

	k_sem_give(&data->bus_lock);
}

int mchp_xec_i2c_ctrl_configure(const struct device *ctrl, uint8_t port, uint32_t freq_hz)
{
	if (port >= XEC_I2C_MAX_PORTS) {
		return -EINVAL;
	}
	return xec_v3_select(ctrl, port, freq_hz);
}

int mchp_xec_i2c_ctrl_get_speed(const struct device *ctrl, uint8_t port, uint32_t *speed)
{
	struct i2c_mchp_xec_v3_data *data = ctrl->data;

	ARG_UNUSED(port);
	if (speed == NULL) {
		return -EINVAL;
	}
	if (!data->hw_enabled) {
		return -EIO;
	}

	switch (data->active_freq) {
	case KHZ(100):
		*speed = I2C_SPEED_STANDARD << I2C_SPEED_SHIFT;
		break;
	case KHZ(400):
		*speed = I2C_SPEED_FAST << I2C_SPEED_SHIFT;
		break;
	case MHZ(1):
		*speed = I2C_SPEED_FAST_PLUS << I2C_SPEED_SHIFT;
		break;
	default:
		return -ERANGE;
	}
	return 0;
}

static bool is_targ_addr(const struct device *ctrl, uint16_t addr)
{
	const struct i2c_mchp_xec_v3_cfg *xcfg = ctrl->config;
	mm_reg_t rb = xcfg->base_addr;
	uint32_t oa = sys_read32(rb + XEC_I2C_OA_OFS);

	if ((XEC_I2C_OA_1_GET(oa) == addr) || (XEC_I2C_OA_2_GET(oa) == addr)) {
		return true;
	}

	return false;
}

static int xec_v3_begin_xfer(const struct device *ctrl, uint8_t port, uint32_t freq_hz,
			     struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr)
{
	struct i2c_mchp_xec_v3_data *data = ctrl->data;
	int ret = 0;

	sm_update(data, 0x10U);

	if (num_msgs == 0 || msgs == NULL) {
		sm_update(data, 0x11U);
		return -EINVAL;
	}
#ifdef CONFIG_I2C_TARGET
	/* Do not start a transaction to an address registered as a target on the
	 * same controller
	 */
	if (is_targ_addr(ctrl, addr)) {
		sm_update(data, 0x12U);
		return -EBADR;
	}

	if (data->target_busy) {
		sm_update(data, 0x13U);
		return -EBUSY;
	}
#endif

	ret = xec_v3_select(ctrl, port, freq_hz);
	if (ret != 0) {
		sm_update(data, 0x14U);
		return ret;
	}

	sm_update(data, 0x15U);

	ret = xec_v3_wait_bus_free(((const struct i2c_mchp_xec_v3_cfg *)ctrl->config)->base_addr,
				   XEC_V3_BUS_FREE_TIMEOUT_US);
	if (ret != 0) {
		sm_update(data, 0x16U);
		return ret;
	}

	data->msgs = msgs;
	data->num_msgs = num_msgs;
	data->cur_msg = 0;
	data->dev_addr = addr;
	data->xfer_err = 0;
	data->state = XEC_V3_ADDR_W; /* overridden by kick_msg */

	sm_update(data, 0x17U);

	xec_v3_kick_msg(ctrl);

	sm_update(data, 0x18U);

	return 0;
}

int mchp_xec_i2c_ctrl_transfer(const struct device *ctrl, uint8_t port, uint32_t freq_hz,
			       struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr)
{
	struct i2c_mchp_xec_v3_data *data = ctrl->data;
	int ret = 0;

	im_clear(data);
	sm_clear(data);

	sm_update(data, 1U);

	ret = xec_v3_begin_xfer(ctrl, port, freq_hz, msgs, num_msgs, addr);
	if (ret != 0) {
		sm_update(data, 2U);
		return ret;
	}

	if (k_sem_take(&data->xfer_done, K_MSEC(XEC_V3_XFER_TIMEOUT_MS)) != 0) {
		/* Hang: reset and surface a timeout. */
		sm_update(data, 3U);
		data->active_port = XEC_V3_PORT_NONE;
		data->hw_enabled = false;
		data->state = XEC_V3_IDLE;
		return -ETIMEDOUT;
	}

	data->state = XEC_V3_IDLE;

	sm_update(data, 4U);
	return data->xfer_err;
}

#ifdef CONFIG_I2C_CALLBACK
int mchp_xec_i2c_ctrl_transfer_cb(const struct device *ctrl, uint8_t port, uint32_t freq_hz,
				  struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr,
				  i2c_callback_t cb, void *userdata)
{
	struct i2c_mchp_xec_v3_data *data = ctrl->data;
	int ret = 0;

	im_clear(data);
	sm_clear(data);

	sm_update(data, 8U);

	/* Caller must hold bus_lock; we keep holding it across the async
	 * transfer and release it from xec_v3_finish() before invoking cb.
	 */
	data->async_cb = cb;
	data->async_userdata = userdata;
	data->async_active = true;

	ret = xec_v3_begin_xfer(ctrl, port, freq_hz, msgs, num_msgs, addr);
	if (ret != 0) {
		sm_update(data, 9U);
		data->async_active = false;
		data->async_cb = NULL;
		data->async_userdata = NULL;
	}

	sm_update(data, 10U);
	return ret;
}
#endif

int mchp_xec_i2c_ctrl_recover_bus(const struct device *ctrl, uint8_t port, uint32_t freq_hz,
				  const struct pinctrl_dev_config *pcfg)
{
	const struct i2c_mchp_xec_v3_cfg *cfg = ctrl->config;
	struct i2c_mchp_xec_v3_data *data = ctrl->data;
	int ret;

	/* Ensure the target port's pins are muxed to this controller before
	 * we drive the BB logic. Also force a fresh reconfigure so CFG.ENAB
	 * and port select are programmed.
	 */
	if (pcfg != NULL) {
		ret = pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT);
		if (ret != 0) {
			return ret;
		}
	}
	ret = xec_v3_reconfigure(ctrl, port, freq_hz);
	if (ret != 0) {
		return ret;
	}

	ret = xec_v3_bb_recover(cfg->base_addr);

	/* Recovery always trashes controller state; force reconfigure next call. */
	data->active_port = XEC_V3_PORT_NONE;
	data->hw_enabled = false;
	return ret;
}

#ifdef CONFIG_I2C_TARGET
/*
 * Target mode is pinned to a single port. The first successful register
 * call sets target_port; subsequent register calls must use the same
 * port (HW has two own-address slots, so up to two targets can share
 * that one port). To move target mode to a different port the caller
 * must unregister all currently registered targets first.
 */
int mchp_xec_i2c_ctrl_target_register(const struct device *ctrl, uint8_t port,
				      struct i2c_target_config *tcfg)
{
	struct i2c_mchp_xec_v3_data *data = ctrl->data;
	int free_idx = -1;

	if (tcfg == NULL || port >= XEC_I2C_MAX_PORTS) {
		return -EINVAL;
	}
	if ((tcfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0) {
		return -ENOTSUP;
	}
	if (data->target_port != XEC_V3_PORT_NONE && data->target_port != port) {
		/* Target mode is already pinned to a different port.
		 * Caller must unregister everything on that port first.
		 */
		return -EBUSY;
	}
	if (data->target_busy) {
		return -EBUSY;
	}

	for (int i = 0; i < XEC_I2C_OA_NUM_TARGETS; i++) {
		if (data->targets[i].in_use && data->targets[i].cfg->address == tcfg->address) {
			return -EALREADY;
		}
		if (!data->targets[i].in_use && free_idx < 0) {
			free_idx = i;
		}
	}
	if (free_idx < 0) {
		return -ENOMEM;
	}

	im_clear(data);
	sm_clear(data);

	data->targets[free_idx].cfg = tcfg;
	data->targets[free_idx].in_use = true;
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	data->targets[free_idx].buffer_mode =
		(tcfg->callbacks != NULL) && ((tcfg->callbacks->buf_write_received != NULL) ||
					      (tcfg->callbacks->buf_read_requested != NULL));
#else
	data->targets[free_idx].buffer_mode = false;
#endif

	data->target_port = port;

	if (data->active_port == port && data->hw_enabled) {
		/* Writing OWN_ADDRESS arms the HW address-match logic; no CFG
		 * bit toggle is needed. The idle CR state (with ENI set from
		 * reconfigure) routes the resulting PIN interrupt to the NVIC.
		 */
		xec_v3_apply_target_oa(ctrl);
	}
	return 0;
}

int mchp_xec_i2c_ctrl_target_unregister(const struct device *ctrl, uint8_t port,
					struct i2c_target_config *tcfg)
{
	const struct i2c_mchp_xec_v3_cfg *cfg = ctrl->config;
	struct i2c_mchp_xec_v3_data *data = ctrl->data;

	if (tcfg == NULL || port >= XEC_I2C_MAX_PORTS) {
		return -EINVAL;
	}
	if (data->target_port != port) {
		return -ENOENT;
	}

	for (int i = 0; i < XEC_I2C_OA_NUM_TARGETS; i++) {
		if (data->targets[i].in_use && data->targets[i].cfg == tcfg) {
			bool others = false;

			data->targets[i].in_use = false;
			data->targets[i].cfg = NULL;

			for (int j = 0; j < XEC_I2C_OA_NUM_TARGETS; j++) {
				if (data->targets[j].in_use) {
					others = true;
					break;
				}
			}
			if (!others) {
				/* No targets left — release the port lock so
				 * future transfers can freely switch ports.
				 */
				data->target_port = XEC_V3_PORT_NONE;
				/* Clear OA explicitly so a stray address match
				 * can't happen before the next reconfigure.
				 */
				if (data->hw_enabled) {
					sys_write32(0, cfg->base_addr + XEC_I2C_OA_OFS);
				}
			} else if (data->active_port == port && data->hw_enabled) {
				xec_v3_apply_target_oa(ctrl);
			}
			return 0;
		}
	}
	return -ENOENT;
}
#endif /* CONFIG_I2C_TARGET */

/* ------------------------------------------------------------------ */
/* Init                                                               */
/* ------------------------------------------------------------------ */

static int i2c_mchp_xec_v3_init(const struct device *ctrl)
{
	const struct i2c_mchp_xec_v3_cfg *cfg = ctrl->config;
	struct i2c_mchp_xec_v3_data *data = ctrl->data;

	k_sem_init(&data->bus_lock, 1, 1);
	k_sem_init(&data->xfer_done, 0, 1);

	data->active_port = XEC_V3_PORT_NONE;
	data->active_freq = 0;
	data->hw_enabled = false;
	data->state = XEC_V3_IDLE;
	data->cr_shadow = 0; /* matches HW CR state after PCR reset */
#ifdef CONFIG_I2C_TARGET
	data->target_port = XEC_V3_PORT_NONE;
#endif

	/* Disable PCR sleep for this block, but DO NOT enable hardware yet.
	 * The port driver runs at a higher priority value (51 vs. 50) in the
	 * same POST_KERNEL phase, so it executes after this controller init.
	 * Its init applies pinctrl and then calls mchp_xec_i2c_ctrl_configure()
	 * which actually enables CFG.ENAB. Microchip hardware requires pins
	 * muxed before CFG.ENAB = 1.
	 */
	soc_xec_pcr_sleep_en_clear((uint8_t)cfg->enc_pcr);

	if (cfg->irq_config_func != NULL) {
		cfg->irq_config_func();
	}
	return 0;
}

#define XEC_V3_GIRQ(inst, idx) (uint8_t)MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, idx))
#define XEC_V3_GIRQ_POS(inst, idx)                                                                 \
	(uint8_t)MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, idx))

/* Target-mode RX buffer: only allocated when CONFIG_I2C_TARGET_BUFFER_MODE
 * is selected; sized by the per-instance "target-buffer-size" DT property
 * (YAML default 256). When buffer mode is not built in, the storage, the
 * cfg fields, and the DT read all compile out.
 */
#if defined(CONFIG_I2C_TARGET) && defined(CONFIG_I2C_TARGET_BUFFER_MODE)
#define XEC_V3_TGT_BUF_DECL(n)                                                                     \
	static uint8_t xec_v3_tgt_rx_buf_##n[DT_INST_PROP(n, target_buffer_size)];
#define XEC_V3_TGT_BUF_INIT(n)                                                                     \
	.tgt_rx_buf = xec_v3_tgt_rx_buf_##n, .tgt_rx_buf_size = DT_INST_PROP(n, target_buffer_size),
#else
#define XEC_V3_TGT_BUF_DECL(n)
#define XEC_V3_TGT_BUF_INIT(n)
#endif

#define I2C_MCHP_XEC_V3_INIT(n)                                                                    \
	static void xec_v3_irq_cfg_##n(void);                                                      \
	XEC_V3_TGT_BUF_DECL(n)                                                                     \
	static struct i2c_mchp_xec_v3_data xec_v3_data_##n;                                        \
	static const struct i2c_mchp_xec_v3_cfg xec_v3_cfg_##n = {                                 \
		.base_addr = (mm_reg_t)DT_INST_REG_ADDR(n),                                        \
		.girq = XEC_V3_GIRQ(n, 0),                                                         \
		.girq_pos = XEC_V3_GIRQ_POS(n, 0),                                                 \
		.enc_pcr = (uint16_t)DT_INST_PROP(n, pcr_scr),                                     \
		.irqn = DT_INST_IRQN(n),                                                           \
		.irq_config_func = xec_v3_irq_cfg_##n,                                             \
		XEC_V3_TGT_BUF_INIT(n)};                                                           \
	DEVICE_DT_INST_DEFINE(n, i2c_mchp_xec_v3_init, NULL, &xec_v3_data_##n, &xec_v3_cfg_##n,    \
			      POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, NULL);                        \
	static void xec_v3_irq_cfg_##n(void)                                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), xec_v3_isr,                 \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(I2C_MCHP_XEC_V3_INIT)
