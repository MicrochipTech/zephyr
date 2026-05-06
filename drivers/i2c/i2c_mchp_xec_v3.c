/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip MEC5 I2C controller — Zephyr API glue.
 *
 * Drives a byte-by-byte, 7-bit-address I2C peripheral defined by README.txt
 * at the project root. Supports controller + target modes, callback-based
 * async transfers, buffer-mode target callbacks, and RTIO via fallback.
 */

#define DT_DRV_COMPAT microchip_xec_i2c_v3

#include <errno.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/clock/mchp_xec_pcr.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_mchp_xec_v3, CONFIG_I2C_LOG_LEVEL);

#include "i2c_mchp_xec_v3.h"

/* MCHP DEBUG */
#define I2C_MEC5_STATE_DEBUG
#define I2C_MEC5_STATE_DEBUG_MAX 256U

/* --- Register offsets -------------------------------------------------- */

#define I2C_MEC5_CTRL_OFS        0x00U /* write-only */
#define I2C_MEC5_STATUS_OFS      0x00U /* read-only  */
#define I2C_MEC5_OWN_ADDR_OFS    0x04U
#define I2C_MEC5_DATA_OFS        0x08U
#define I2C_MEC5_COMPLETION_OFS  0x20U
#define I2C_MEC5_CONFIG_OFS      0x28U
#define I2C_MEC5_BUS_CLK_OFS     0x2CU
#define I2C_MEC5_BBCR_OFS        0x38U

/* --- Control register (offset 0x00, W/O) ------------------------------- */

#define I2C_MEC5_CTRL_ACK        BIT(0)
#define I2C_MEC5_CTRL_STO        BIT(1)
#define I2C_MEC5_CTRL_STA        BIT(2)
#define I2C_MEC5_CTRL_ENI        BIT(3)
#define I2C_MEC5_CTRL_ESO        BIT(6)
#define I2C_MEC5_CTRL_PCLR       BIT(7)

/* --- Status register (offset 0x00, R/O) -------------------------------- */

#define I2C_MEC5_STATUS_NBB      BIT(0) /* bus-busy-not: 1 = idle */
#define I2C_MEC5_STATUS_LAB      BIT(1) /* lost arbitration */
#define I2C_MEC5_STATUS_AAT      BIT(2) /* addressed as target */
#define I2C_MEC5_STATUS_LRB      BIT(3) /* last RX bit: 0=ACK, 1=NAK */
#define I2C_MEC5_STATUS_BER      BIT(4) /* bus error */
#define I2C_MEC5_STATUS_STS      BIT(5) /* external STOP detected */
#define I2C_MEC5_STATUS_NOSVC    BIT(7) /* 0 = service required */

/* Expected idle Status value before generating START (README §START step 1) */
#define I2C_MEC5_STATUS_IDLE_EXPECTED \
	(I2C_MEC5_STATUS_NBB | I2C_MEC5_STATUS_NOSVC)

/* --- OwnAddr register (offset 0x04, R/W) ------------------------------- */

#define I2C_MEC5_OWN_ADDR0_POS   0U
#define I2C_MEC5_OWN_ADDR0_MSK   (0x7FU << I2C_MEC5_OWN_ADDR0_POS)
#define I2C_MEC5_OWN_ADDR1_POS   8U
#define I2C_MEC5_OWN_ADDR1_MSK   (0x7FU << I2C_MEC5_OWN_ADDR1_POS)

/* --- Completion register (offset 0x20) --------------------------------- */

#define I2C_MEC5_COMP_BER        BIT(13) /* R/W1C latched BER          */
#define I2C_MEC5_COMP_LAB        BIT(14) /* R/W1C latched LAB          */
#define I2C_MEC5_COMP_IDLE       BIT(29) /* R/W1C latched NBB (idle)   */
#define I2C_MEC5_COMP_RW1C_MASK  (I2C_MEC5_COMP_BER | I2C_MEC5_COMP_LAB | \
				  I2C_MEC5_COMP_IDLE)

/* --- Configuration register (offset 0x28, R/W) ------------------------- */

#define I2C_MEC5_CFG_PORT_POS    0U
#define I2C_MEC5_CFG_PORT_MSK    (0xFU << I2C_MEC5_CFG_PORT_POS)
#define I2C_MEC5_CFG_MAX_PORTS   16U
#define I2C_MEC5_CFG_FEN         BIT(8)  /* filter enable (required)  */
#define I2C_MEC5_CFG_SRST        BIT(9)  /* soft reset: hold >=62.5ns */
#define I2C_MEC5_CFG_ENAB        BIT(10) /* controller enable         */
#define I2C_MEC5_CFG_GC_DIS      BIT(14) /* disable general-call resp */
#define I2C_MEC5_CFG_STOP_DET_EN BIT(24) /* enable STOP detect IRQ    */
#define I2C_MEC5_CFG_ENIDI       BIT(29) /* enable IDLE detect IRQ    */

/* --- Bus Clock register (offset 0x2C, R/W) ----------------------------- */

/*
 * SCL_Hz = 16_000_000 / ((LOW_PERIOD + 1) + (HIGH_PERIOD + 1))
 * Baud clock = 16 MHz, period 62.5 ns.
 */
#define I2C_MEC5_BAUD_CLOCK_HZ   16000000U
#define I2C_MEC5_BUS_CLK_LOW_POS  0U
#define I2C_MEC5_BUS_CLK_LOW_MSK  (0xFFU << I2C_MEC5_BUS_CLK_LOW_POS)
#define I2C_MEC5_BUS_CLK_HIGH_POS 8U
#define I2C_MEC5_BUS_CLK_HIGH_MSK (0xFFU << I2C_MEC5_BUS_CLK_HIGH_POS)

#define I2C_MEC5_BUS_CLK_PACK(low, high) \
	((((uint16_t)(low) & 0xFFU) << I2C_MEC5_BUS_CLK_LOW_POS) | \
	 (((uint16_t)(high) & 0xFFU) << I2C_MEC5_BUS_CLK_HIGH_POS))

/* --- Bit-Bang Control register (offset 0x38, R/W) ---------------------- */
#define I2C_MEC5_BBCR_BBM_EN    BIT(0) /* Enable bit-bang mode. SCL/SDA controlled by BB logic */
#define I2C_MEC5_BBCR_CLDIR_OUT BIT(1) /* BB mode SCL is an output */
#define I2C_MEC5_BBCR_DADIR_OUT BIT(2) /* BB mode SDA is an output */
#define I2C_MEC5_BBCR_SCL_TS    BIT(3) /* BB mode, SCL is output 1=tri-state, 0=drive low */
#define I2C_MEC5_BBCR_SDA_TS    BIT(4) /* BB mode, SDA is output 1=tri-state, 0=drive low */
#define I2C_MEC5_BBCR_CLKI_HI   BIT(5) /* Read only SCL live state is high */
#define I2C_MEC5_BBCR_DATI_HI   BIT(6) /* Read only SDA live state is high */
#define I2C_MEC5_BBCR_LIVECM_EN BIT(7) /* enable live SCL/SDA in bits [6:5] */

/* --- Target address format helper -------------------------------------- */

#define I2C_MEC5_ADDR_BYTE(addr7, read) \
	((uint8_t)((((addr7) & 0x7FU) << 1) | ((read) ? 0x01U : 0x00U)))

/* ---- Post reset pin sample wait --------------------------------------- */
#define I2C_MEC5_POST_RESET_PIN_WAIT_US 50U

/* ---- Transfer timeout ------------------------------------------------- */
#define I2C_MEC5_XFR_TIMEOUT_MS (CONFIG_I2C_MCHP_XEC_V3_TRANSFER_TIMEOUT_MS)

/* --- Controller + target state enums ---------------------------------- */
enum mec5_ctrl_state {
	CTRL_IDLE = 0,
	CTRL_START_SENT,  /* addr byte clocking out, waiting for 9th-clock ACK */
	CTRL_TX_DATA,     /* transmitting message data, byte-by-byte IRQ      */
	CTRL_RX_DATA,     /* mid-stream reads; reading Data clocks next byte   */
	CTRL_RX_LAST,     /* reading final byte of current read msg            */
	CTRL_IDLE_WAIT,   /* STS handled; awaiting Completion.IDLE latch       */
	CTRL_DONE,
	CTRL_ERROR,
};

enum mec5_target_state {
	TGT_IDLE = 0,
	TGT_ADDRESSED_RX,     /* external master is writing to us */
	TGT_ADDRESSED_TX,     /* external master is reading from us */
	TGT_STOP_PENDING,     /* external STOP detected, awaiting cleanup */
};

enum mec5_sm_action {
	ACT_NONE = 0,
	ACT_SERVICE,          /* NOSVC=0, normal byte transfer in progress */
	ACT_BUS_ERROR,        /* Status.BER */
	ACT_LOST_ARB,         /* Status.LAB */
	ACT_TGT_ADDRESSED,    /* Status.AAT */
	ACT_STOP_DET,         /* Status.STS */
	ACT_IDLE,             /* Completion.IDLE latch */
};

/* --- Per-instance configuration (const, from DT) ---------------------- */

struct i2c_mec5_config {
	mm_reg_t base;
	uint8_t  girq;
	uint8_t  girq_pos;
	uint8_t  enc_pcr;
	bool     general_call;
	void (*irq_connect)(void);
#ifdef CONFIG_I2C_TARGET
	uint8_t *targ_buf;
	uint32_t targ_buf_size;
#endif
};

/* --- Per-instance runtime data ---------------------------------------- */

struct i2c_mec5_data {
	const struct device *dev;

	/* bus_lock is a binary semaphore (not k_mutex) so the async
	 * completion path can release it from the workqueue thread.
	 */
	struct k_mutex bus_lock;
	struct k_sem xfer_done;

	/* Current controller transaction */
	struct i2c_msg *msgs;
	uint32_t buf_idx;
	uint8_t  num_msgs;
	uint8_t  cur_msg;
	uint16_t addr;
	int      result;

	enum mec5_ctrl_state state;
	const struct i2c_mec5_xfer_ops *ops;

	uint32_t active_freq;
	uint8_t active_port;

#ifdef CONFIG_I2C_CALLBACK
	i2c_callback_t cb;
	void          *userdata;
	struct k_work  cb_work;
#endif

#ifdef CONFIG_I2C_TARGET
	/* Two OwnAddr slots; matched slot is decoded from the address byte
	 * clocked in on AAT.
	 */
	struct i2c_target_config *targets[2];
	enum mec5_target_state    tgt_state;
	uint8_t                   tgt_slot;
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	/* Internal buffer for buffer-mode RX accumulation and TX serving. */
	uint8_t  *tgt_buf;
	uint32_t tgt_buf_idx;
	uint32_t tgt_buf_len; /* meaningful for TX; caps at bufsize */
	uint8_t *tgt_tx_ptr;  /* TX source when buf_read_requested returns
			       * its own pointer (may differ from tgt_buf).
			       */
#endif
#endif
#ifdef CONFIG_I2C_MCHP_XEC_V3_ENABLE_RECOV_COUNTERS
	uint64_t recov_attempts;
	uint64_t recov_fails;
#endif
#ifdef I2C_MEC5_STATE_DEBUG
	uint32_t dbg_state_idx;
	uint8_t dbg_states[I2C_MEC5_STATE_DEBUG_MAX];
#endif
};

#ifdef I2C_MEC5_STATE_DEBUG
static void i2c_mec5_dbg_state_init(struct i2c_mec5_data *data)
{
	data->dbg_state_idx = 0;
	memset(data->dbg_states, 0, I2C_MEC5_STATE_DEBUG_MAX);
}

static void i2c_mec5_dbg_state_update(struct i2c_mec5_data *data, uint8_t state_val)
{
	if (data->dbg_state_idx >= I2C_MEC5_STATE_DEBUG_MAX) {
		return;
	}

	data->dbg_states[data->dbg_state_idx++] = state_val;
}
#else
static void i2c_mec5_dbg_state_init(struct i2c_mec5_data *data)
{
}
static void i2c_mec5_dbg_state_update(struct i2c_mec5_data *data, uint8_t state_val)
{
}
#endif
/* --- Data-movement abstraction (DMA swap-in seam) --------------------- */

/*
 * The state machine never touches msg->buf directly. A future DMA backend
 * replaces these three pointers and calls on_complete() from its own IRQ.
 */
struct i2c_mec5_xfer_ops {
	/* Write a byte to the Data register (side effect: clocks out byte,
	 * clears Status.NOSVC on TX path).
	 */
	void (*push_byte)(struct i2c_mec5_data *ctx, uint8_t b);

	/* Read a byte from the Data register (side effect: clears PIN/NOSVC
	 * and, when owning the bus with STO=0, clocks the next byte).
	 */
	uint8_t (*pull_byte)(struct i2c_mec5_data *ctx);

	/* Called by the SM when the transaction is done or has errored.
	 * For PIO this signals the waiter; a DMA backend may do teardown
	 * before signaling.
	 */
	void (*on_complete)(struct i2c_mec5_data *ctx, int result);
};

/* --- Register access helpers ------------------------------------------ */

static inline uint8_t i2c_mec5_status_r(const struct i2c_mec5_config *cfg)
{
	return sys_read8(cfg->base + I2C_MEC5_STATUS_OFS);
}

static inline void i2c_mec5_ctrl_w(const struct i2c_mec5_config *cfg, uint8_t v)
{
	sys_write8(v, cfg->base + I2C_MEC5_CTRL_OFS);
}

static inline uint8_t i2c_mec5_data_r(const struct i2c_mec5_config *cfg)
{
	return sys_read8(cfg->base + I2C_MEC5_DATA_OFS);
}

static inline void i2c_mec5_data_w(const struct i2c_mec5_config *cfg, uint8_t v)
{
	sys_write8(v, cfg->base + I2C_MEC5_DATA_OFS);
}

static inline uint32_t i2c_mec5_completion_r(const struct i2c_mec5_config *cfg)
{
	return sys_read32(cfg->base + I2C_MEC5_COMPLETION_OFS);
}

static inline void i2c_mec5_completion_w(const struct i2c_mec5_config *cfg, uint32_t v)
{
	sys_write32(v, cfg->base + I2C_MEC5_COMPLETION_OFS);
}

static inline uint32_t i2c_mec5_config_r(const struct i2c_mec5_config *cfg)
{
	return sys_read32(cfg->base + I2C_MEC5_CONFIG_OFS);
}

static inline void i2c_mec5_config_w(const struct i2c_mec5_config *cfg, uint32_t v)
{
	sys_write32(v, cfg->base + I2C_MEC5_CONFIG_OFS);
}

static inline uint16_t i2c_mec5_bus_clk_r(const struct i2c_mec5_config *cfg)
{
	return sys_read16(cfg->base + I2C_MEC5_BUS_CLK_OFS);
}

static inline void i2c_mec5_bus_clk_w(const struct i2c_mec5_config *cfg, uint16_t v)
{
	sys_write16(v, cfg->base + I2C_MEC5_BUS_CLK_OFS);
}

static inline uint32_t i2c_mec5_own_addr_r(const struct i2c_mec5_config *cfg)
{
	return sys_read32(cfg->base + I2C_MEC5_OWN_ADDR_OFS);
}

static inline void i2c_mec5_own_addr_w(const struct i2c_mec5_config *cfg, uint32_t v)
{
	sys_write32(v, cfg->base + I2C_MEC5_OWN_ADDR_OFS);
}

static inline uint8_t i2c_mec5_bbcr_r(const struct i2c_mec5_config *cfg)
{
	return sys_read8(cfg->base + I2C_MEC5_BBCR_OFS);
}

static inline void i2c_mec5_bbcr_w(const struct i2c_mec5_config *cfg, uint8_t v)
{
	sys_write8(v, cfg->base + I2C_MEC5_BBCR_OFS);
}

/* --- Control-register word builders ----------------------------------- */

#define CTRL_BASE  (I2C_MEC5_CTRL_PCLR | I2C_MEC5_CTRL_ESO | I2C_MEC5_CTRL_ENI)

static inline void ctrl_write(const struct i2c_mec5_config *cfg, uint8_t extra)
{
	i2c_mec5_ctrl_w(cfg, CTRL_BASE | extra);
}

/* --- Forward decls for static helpers --------------------------------- */

static void begin_msg(struct i2c_mec5_data *ctx);
static void advance_or_stop(struct i2c_mec5_data *ctx, int result);
static void schedule_stop(struct i2c_mec5_data *ctx, int result);


/* --- Entry points ----------------------------------------------------- */

static int i2c_mec5_sm_kickoff(struct i2c_mec5_data *ctx, struct i2c_msg *msgs, uint8_t num_msgs,
			       uint16_t addr)
{
	const struct i2c_mec5_config *cfg = ctx->dev->config;
	uint8_t status = i2c_mec5_status_r(cfg);
	int ret = 0;

	i2c_mec5_dbg_state_update(ctx, 0x10U);

	/* README §START step 1: bus must be idle (NBB=1, NOSVC=1, others 0). */
	if (status != I2C_MEC5_STATUS_IDLE_EXPECTED) {
		i2c_mec5_dbg_state_update(ctx, 0x12U);
		LOG_WRN("kickoff: bus not idle, status=0x%02x", status);
		ret = -EBUSY;
	} else {
		i2c_mec5_dbg_state_update(ctx, 0x11U);
		ctx->msgs = msgs;
		ctx->num_msgs = num_msgs;
		ctx->cur_msg = 0;
		ctx->addr = addr;
		ctx->result = 0;

		/* Clear any stale Completion latches before starting. */
		i2c_mec5_completion_w(cfg, I2C_MEC5_COMP_RW1C_MASK);

		begin_msg(ctx);
	}

	return ret;
}

/* --- START / repeated-START ------------------------------------------- */

static void begin_msg(struct i2c_mec5_data *ctx)
{
	const struct i2c_mec5_config *cfg = ctx->dev->config;
	struct i2c_msg *m = &ctx->msgs[ctx->cur_msg];
	bool is_read = (m->flags & I2C_MSG_READ) == I2C_MSG_READ;
	uint8_t addr_byte = I2C_MEC5_ADDR_BYTE(ctx->addr, is_read);

	i2c_mec5_dbg_state_update(ctx, 0x20);

	ctx->buf_idx = 0;
	ctx->state = CTRL_START_SENT;

	/* README §START steps 2-5. */
	i2c_mec5_ctrl_w(cfg, I2C_MEC5_CTRL_PCLR | I2C_MEC5_CTRL_ESO | I2C_MEC5_CTRL_ACK);

	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);
	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, MCHP_MEC_ECIA_GIRQ_EN);

	i2c_mec5_data_w(cfg, addr_byte);
	i2c_mec5_ctrl_w(cfg, I2C_MEC5_CTRL_PCLR | I2C_MEC5_CTRL_ESO |
				    I2C_MEC5_CTRL_ENI | I2C_MEC5_CTRL_STA |
				    I2C_MEC5_CTRL_ACK);

	i2c_mec5_dbg_state_update(ctx, 0x21U);
}

/* --- Message termination --------------------------------------------- */

static void advance_or_stop(struct i2c_mec5_data *ctx, int result)
{
	struct i2c_msg *m = &ctx->msgs[ctx->cur_msg];
	bool last_msg = (ctx->cur_msg + 1U) == ctx->num_msgs;
	bool force_stop = (m->flags & I2C_MSG_STOP) == I2C_MSG_STOP;

	i2c_mec5_dbg_state_update(ctx, 0x30);

	if (result != 0 || last_msg || force_stop) {
		i2c_mec5_dbg_state_update(ctx, 0x31);
		schedule_stop(ctx, result);
	} else {
		i2c_mec5_dbg_state_update(ctx, 0x32);
		ctx->cur_msg++;
		begin_msg(ctx);
	}

	i2c_mec5_dbg_state_update(ctx, 0x33);
}

static void schedule_stop(struct i2c_mec5_data *ctx, int result)
{
	const struct i2c_mec5_config *cfg = ctx->dev->config;
	uint32_t cfgv;

	i2c_mec5_dbg_state_update(ctx, 0x34);

	ctx->result = result;
	ctx->state = CTRL_IDLE_WAIT;

	/*
	 * Controller-mode STOP waits only for Completion.IDLE. On this
	 * hardware, Status.STS (enabled via I2C_STOP_DET_EN) fires only in
	 * target mode - it is never asserted when WE generate the STOP.
	 * Enabling STOP_DET_EN in controller mode would leave the driver
	 * spinning on an IRQ source that will never fire.
	 */
	i2c_mec5_completion_w(cfg, I2C_MEC5_COMP_IDLE);
	cfgv = i2c_mec5_config_r(cfg);
	cfgv |= I2C_MEC5_CFG_ENIDI;
	i2c_mec5_config_w(cfg, cfgv);
	ctrl_write(cfg, I2C_MEC5_CTRL_STO | I2C_MEC5_CTRL_ACK);

	i2c_mec5_dbg_state_update(ctx, 0x35);
}

/* --- Per-state handlers ---------------------------------------------- */

static void handle_start_sent(struct i2c_mec5_data *ctx, uint8_t status)
{
	const struct i2c_mec5_config *cfg = ctx->dev->config;
	struct i2c_msg *m = &ctx->msgs[ctx->cur_msg];
	bool is_read = (m->flags & I2C_MSG_READ) == I2C_MSG_READ;

	i2c_mec5_dbg_state_update(ctx, 0x40U);

	if ((status & I2C_MEC5_STATUS_LRB) != 0U) {
		i2c_mec5_dbg_state_update(ctx, 0x41U);
		/* Address NAK'd. Generate STOP and signal -EIO. */
		schedule_stop(ctx, -EIO);
	} else if (m->len == 0U) {
		i2c_mec5_dbg_state_update(ctx, 0x42U);
		/* Zero-length probe ping: address was ACK'd, nothing to
		 * transfer. Advance to next msg or STOP.
		 */
		advance_or_stop(ctx, 0);
	} else if (!is_read) {
		i2c_mec5_dbg_state_update(ctx, 0x43U);
		ctx->ops->push_byte(ctx, m->buf[0]);
		ctx->buf_idx = 1U;
		ctx->state = CTRL_TX_DATA;
	} else if (m->len == 1U) {
		i2c_mec5_dbg_state_update(ctx, 0x44U);
		/* Single-byte read: NAK the one byte we're about to clock
		 * in. The next Control write before reading the byte will
		 * set STO to suppress further clocks.
		 */
		ctrl_write(cfg, 0U);                   /* ACK=0 */
		(void)ctx->ops->pull_byte(ctx);        /* dummy read kicks
							* byte 0 clocking
							*/
		ctx->state = CTRL_RX_LAST;
	} else {
		i2c_mec5_dbg_state_update(ctx, 0x45U);
		ctrl_write(cfg, I2C_MEC5_CTRL_ACK);    /* byte 0 ACKs */
		(void)ctx->ops->pull_byte(ctx);        /* dummy read */
		ctx->state = CTRL_RX_DATA;
	}

	i2c_mec5_dbg_state_update(ctx, 0x46U);
}

static void handle_tx_data(struct i2c_mec5_data *ctx, uint8_t status)
{
	struct i2c_msg *m = &ctx->msgs[ctx->cur_msg];

	i2c_mec5_dbg_state_update(ctx, 0x47U);

	if ((status & I2C_MEC5_STATUS_LRB) != 0U) {
		i2c_mec5_dbg_state_update(ctx, 0x48U);
		/* Data byte NAK'd — target refused. Per spec this is a
		 * legitimate end-of-transaction marker for some devices,
		 * but Zephyr convention treats it as -EIO.
		 */
		schedule_stop(ctx, -EIO);
	} else if (ctx->buf_idx < m->len) {
		i2c_mec5_dbg_state_update(ctx, 0x49U);
		ctx->ops->push_byte(ctx, m->buf[ctx->buf_idx]);
		ctx->buf_idx++;
	} else {
		i2c_mec5_dbg_state_update(ctx, 0x4AU);
		/* All bytes in this msg ACK'd. Move on. */
		advance_or_stop(ctx, 0);
	}

	i2c_mec5_dbg_state_update(ctx, 0x4BU);
}

static void handle_rx_data(struct i2c_mec5_data *ctx)
{
	const struct i2c_mec5_config *cfg = ctx->dev->config;
	struct i2c_msg *m = &ctx->msgs[ctx->cur_msg];

	i2c_mec5_dbg_state_update(ctx, 0x50U);

	/*
	 * Reading Data (pull_byte) clocks in the NEXT byte. So the ACK/STOP
	 * policy that applies to the byte clocked in by THIS read must be
	 * written to Control BEFORE pull_byte().
	 *
	 *   buf_idx+1 == len-1  →  the next byte clocked in will be the
	 *                          final one. Clear ACK so it NAKs.
	 *   buf_idx+1 == len    →  we've already clocked in the final byte
	 *                          (it's in Data now). This path is RX_LAST,
	 *                          not here.
	 */
	if ((ctx->buf_idx + 1U) == (m->len - 1U)) {
		i2c_mec5_dbg_state_update(ctx, 0x51U);
		ctrl_write(cfg, 0U); /* next byte NAKs */
	}

	m->buf[ctx->buf_idx] = ctx->ops->pull_byte(ctx);
	ctx->buf_idx++;

	if (ctx->buf_idx == (m->len - 1U)) {
		i2c_mec5_dbg_state_update(ctx, 0x52U);
		ctx->state = CTRL_RX_LAST;
	}

	i2c_mec5_dbg_state_update(ctx, 0x53U);
}

static void handle_rx_last(struct i2c_mec5_data *ctx)
{
	const struct i2c_mec5_config *cfg = ctx->dev->config;
	struct i2c_msg *m = &ctx->msgs[ctx->cur_msg];
	uint32_t cfgv;

	i2c_mec5_dbg_state_update(ctx, 0x54U);

	/*
	 * The final byte is already in Data. Reading it with STO=0 would
	 * cause the hardware to clock another byte we don't want. Set STO
	 * first to suppress further clocks and schedule the STOP sequence;
	 * the read itself clears Status.PIN so the IRQ deasserts.
	 *
	 * Note: we always STOP after an RX message, even when more messages
	 * follow. A true repeated-START-from-RX requires orchestrating the
	 * STA write against the in-progress byte clocking, which this
	 * hardware doesn't cleanly support. After STOP completes,
	 * handle_idle_wait() restarts the next message with a fresh START.
	 *
	 * Controller-mode STOP uses only ENIDI - Status.STS is target-only
	 * on this silicon (see schedule_stop()).
	 */
	i2c_mec5_completion_w(cfg, I2C_MEC5_COMP_IDLE);
	cfgv = i2c_mec5_config_r(cfg);
	cfgv |= I2C_MEC5_CFG_ENIDI;
	i2c_mec5_config_w(cfg, cfgv);
	ctrl_write(cfg, I2C_MEC5_CTRL_STO | I2C_MEC5_CTRL_ACK);

	m->buf[ctx->buf_idx] = ctx->ops->pull_byte(ctx);
	ctx->buf_idx++;
	ctx->state = CTRL_IDLE_WAIT;

	i2c_mec5_dbg_state_update(ctx, 0x55U);
}

static void handle_idle_wait(struct i2c_mec5_data *ctx, enum mec5_sm_action act)
{
	bool more_msgs = false;

	i2c_mec5_dbg_state_update(ctx, 0x56U);

	if (act == ACT_IDLE) {
		i2c_mec5_dbg_state_update(ctx, 0x57U);

		more_msgs = (ctx->result == 0) &&
			    ((ctx->cur_msg + 1U) < ctx->num_msgs);

		if (more_msgs) {
			i2c_mec5_dbg_state_update(ctx, 0x58U);
			/* STOP was forced by the RX path; resume the
			 * transaction with a fresh START for the next msg.
			 */
			ctx->cur_msg++;
			begin_msg(ctx);
		} else {
			i2c_mec5_dbg_state_update(ctx, 0x59U);
			ctx->state = CTRL_DONE;
			ctx->ops->on_complete(ctx, ctx->result);
		}
	}

	i2c_mec5_dbg_state_update(ctx, 0x5AU);
}

/* --- Public step function (called from ISR) --------------------------- */

static void i2c_mec5_sm_step(struct i2c_mec5_data *ctx, enum mec5_sm_action act, uint8_t status)
{
	i2c_mec5_dbg_state_update(ctx, 0x60U);

	/* Error actions preempt state-based dispatch.
	 * BER: hardware requires soft-reset (thread path handles).
	 * LAB: bus taken by another master; signal retry.
	 */
	if (act == ACT_BUS_ERROR) {
		i2c_mec5_dbg_state_update(ctx, 0x61U);
		ctx->state = CTRL_ERROR;
		ctx->ops->on_complete(ctx, -EIO);
	} else if (act == ACT_LOST_ARB) {
		i2c_mec5_dbg_state_update(ctx, 0x62U);
		ctx->state = CTRL_ERROR;
		ctx->ops->on_complete(ctx, -EAGAIN);
	} else {
		switch (ctx->state) {
		case CTRL_START_SENT:
			i2c_mec5_dbg_state_update(ctx, 0x63U);
			handle_start_sent(ctx, status);
			break;
		case CTRL_TX_DATA:
			i2c_mec5_dbg_state_update(ctx, 0x64U);
			handle_tx_data(ctx, status);
			break;
		case CTRL_RX_DATA:
			i2c_mec5_dbg_state_update(ctx, 0x65U);
			handle_rx_data(ctx);
			break;
		case CTRL_RX_LAST:
			i2c_mec5_dbg_state_update(ctx, 0x66U);
			handle_rx_last(ctx);
			break;
		case CTRL_IDLE_WAIT:
			i2c_mec5_dbg_state_update(ctx, 0x67U);
			handle_idle_wait(ctx, act);
			break;
		case CTRL_IDLE:
		case CTRL_DONE:
		case CTRL_ERROR:
		default:
			i2c_mec5_dbg_state_update(ctx, 0x68U);
			/* Spurious IRQ; nothing to do. */
			break;
		}
	}
}

/* -------- pio transfer ops -------- */
static void pio_push_byte(struct i2c_mec5_data *ctx, uint8_t b)
{
	const struct i2c_mec5_config *cfg = ctx->dev->config;

	/* Writing Data clocks the byte out and clears Status.NOSVC on TX. */
	i2c_mec5_data_w(cfg, b);
}

static uint8_t pio_pull_byte(struct i2c_mec5_data *ctx)
{
	const struct i2c_mec5_config *cfg = ctx->dev->config;

	/*
	 * Reading Data clears Status.PIN/NOSVC. If this controller owns the
	 * bus (NBB=0) and Control.STO is 0, the read also causes hardware
	 * to clock the next byte. The SM therefore arranges Control state
	 * (ACK for mid-stream, !ACK for last byte, STO for terminal read)
	 * BEFORE invoking pull_byte on the byte that should trigger the
	 * next clock burst.
	 */
	return i2c_mec5_data_r(cfg);
}

static void pio_on_complete(struct i2c_mec5_data *ctx, int result)
{
	const struct i2c_mec5_config *cfg = ctx->dev->config;
	uint8_t i2c_sr = 0;

	if (ctx->state == CTRL_ERROR) {
		i2c_sr = i2c_mec5_status_r(cfg);
		if ((i2c_sr & I2C_MEC5_STATUS_NOSVC) == 0) {
			i2c_mec5_data_r(cfg);
			i2c_mec5_ctrl_w(cfg, 0xC3U); /* gen STOP */
		}
	}

	ctx->result = result;
	k_sem_give(&ctx->xfer_done);

#ifdef CONFIG_I2C_CALLBACK
	if (ctx->cb != NULL) {
		k_work_submit(&ctx->cb_work);
	}
#endif
}

static const struct i2c_mec5_xfer_ops i2c_mec5_pio_ops = {
	.push_byte   = pio_push_byte,
	.pull_byte   = pio_pull_byte,
	.on_complete = pio_on_complete,
};

/* --- Bitrate helpers -------------------------------------------------- */

static int mec5_speed_to_freq(uint32_t speed, uint32_t *freq)
{
	if (speed == I2C_SPEED_STANDARD) {
		*freq = KHZ(100);
	} else if (speed == I2C_SPEED_FAST) {
		*freq = KHZ(400);
	} else if (speed == I2C_SPEED_FAST_PLUS) {
		*freq = KHZ(1000);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int mec5_bitrate_apply(const struct device *dev, uint32_t speed)
{
	const struct i2c_mec5_config *cfg = dev->config;
	uint32_t hz;
	uint32_t divisor;
	uint8_t half;
	int ret = 0;

	switch (speed) {
	case I2C_SPEED_STANDARD:
		hz = 100000U;
		break;
	case I2C_SPEED_FAST:
		hz = 400000U;
		break;
	case I2C_SPEED_FAST_PLUS:
		hz = 1000000U;
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	if (ret == 0) {
		/*
		 * SCL_Hz = 16_000_000 / ((LOW+1)+(HIGH+1))
		 * divisor = (LOW+1)+(HIGH+1)
		 * Symmetric split: LOW = HIGH = divisor/2 - 1.
		 */
		divisor = I2C_MEC5_BAUD_CLOCK_HZ / hz;
		if (divisor < 2U || divisor > 512U || (divisor & 1U) != 0U) {
			ret = -ERANGE;
		} else {
			half = (uint8_t)((divisor / 2U) - 1U);
			i2c_mec5_bus_clk_w(cfg, I2C_MEC5_BUS_CLK_PACK(half, half));
		}
	}

	return ret;
}

static uint32_t mec5_bitrate_to_speed(uint32_t hz)
{
	uint32_t speed;

	if (hz >= 1000000U) {
		speed = I2C_SPEED_FAST_PLUS;
	} else if (hz >= 400000U) {
		speed = I2C_SPEED_FAST;
	} else {
		speed = I2C_SPEED_STANDARD;
	}

	return speed;
}

static int mec5_port_apply(const struct device *dev, uint8_t port)
{
	const struct i2c_mec5_config *xcfg = dev->config;
	uint32_t cfgv = i2c_mec5_config_r(xcfg);

	if (port >= I2C_MEC5_CFG_MAX_PORTS) {
		return -ENOTSUP;
	}

	cfgv &= (uint32_t)~(I2C_MEC5_CFG_PORT_MSK);
	cfgv |= (((uint32_t)port << I2C_MEC5_CFG_PORT_POS) & I2C_MEC5_CFG_PORT_MSK);
	i2c_mec5_config_w(xcfg, cfgv);

	return 0;
}

#ifdef CONFIG_I2C_TARGET
static bool i2c_mec5_target_enabled(const struct device *dev)
{
	const struct i2c_mec5_config *xcfg = dev->config;

	if (i2c_mec5_own_addr_r(xcfg) != 0) {
		return true;
	}

	return false;
}

/* After I2C controller reset we re-configure target Own Addresses.
 * Refer to Section 6, table 6-4.
 */
static void config_hw_targets(const struct device *dev)
{
	const struct i2c_mec5_config *xcfg = dev->config;
	struct i2c_mec5_data *const xdat = dev->data;
	uint32_t oa = 0;

	if (xdat->targets[0] != NULL) {
		oa |= (((uint32_t)xdat->targets[0]->address << I2C_MEC5_OWN_ADDR0_POS) &
		       I2C_MEC5_OWN_ADDR0_MSK);
	}

	if (xdat->targets[1] != NULL) {
		oa |= (((uint32_t)xdat->targets[1]->address << I2C_MEC5_OWN_ADDR1_POS) &
		       I2C_MEC5_OWN_ADDR1_MSK);
	}

	i2c_mec5_own_addr_w(xcfg, oa);
}
#endif

static int config_hw(const struct device *dev, uint32_t freq_hz, uint8_t port, bool ien)
{
	const struct i2c_mec5_config *xcfg = dev->config;
	struct i2c_mec5_data *const xdat = dev->data;
	uint32_t speed = mec5_bitrate_to_speed(freq_hz);
	uint32_t regv = 0;
	int i = 0, ret = 0;

	soc_ecia_girq_ctrl(xcfg->girq, xcfg->girq_pos, MCHP_MEC_ECIA_GIRQ_DIS);

	soc_xec_pcr_reset_en(xcfg->enc_pcr);
	i2c_mec5_config_w(xcfg, I2C_MEC5_CFG_SRST);
	for (i = 0; i < 16U; i++) {
		i2c_mec5_config_r(xcfg);
	}
	i2c_mec5_config_w(xcfg, 0);
	/* reset, disabled, and defaults to 100 KHz bus clock */

	/* Section 6. Table 6-4 */
	i2c_mec5_ctrl_w(xcfg, I2C_MEC5_CTRL_PCLR);

	/* disable response to I2C General call and enable digital filter */
	regv = i2c_mec5_config_r(xcfg);
	regv |= I2C_MEC5_CFG_GC_DIS;
	regv |= I2C_MEC5_CFG_FEN;
	i2c_mec5_config_w(xcfg, regv);

#ifdef CONFIG_I2C_TARGET
	config_hw_targets(dev);
#endif

	ret = mec5_bitrate_apply(dev, speed);
	if (ret) {
		return ret;
	}

	ret = mec5_port_apply(dev, port);
	if (ret) {
		return ret;
	}

	xdat->active_freq = freq_hz;
	xdat->active_port = port;

	/* Enable auto-ACK of received data and matching addresses */
	ctrl_write(xcfg, I2C_MEC5_CTRL_ACK);

	regv = i2c_mec5_config_r(xcfg);
	regv |= I2C_MEC5_CFG_ENAB;
	i2c_mec5_config_w(xcfg, regv);

	/* Enable live SCL/SDA pin states in BB-Control. HW V3.8 and above */
	i2c_mec5_bbcr_w(xcfg, I2C_MEC5_BBCR_LIVECM_EN);
#if 0
	for (i = 0; i < 16U; i++) {
		regv = i2c_mec5_completion_r(xcfg);
	}
#else
	k_busy_wait(I2C_MEC5_POST_RESET_PIN_WAIT_US);
#endif

	/* clear I2C.COMPLETION R/W1C status */
	regv = i2c_mec5_completion_r(xcfg) | I2C_MEC5_COMP_RW1C_MASK;
	i2c_mec5_completion_w(xcfg, regv);

	soc_ecia_girq_status_clear(xcfg->girq, xcfg->girq_pos);
	if (ien == true) {
		soc_ecia_girq_ctrl(xcfg->girq, xcfg->girq_pos, MCHP_MEC_ECIA_GIRQ_EN);
	}

#if 0
	regv = i2c_mec5_status_r(xcfg);
	if (regv != I2C_MEC5_STATUS_IDLE_EXPECTED) {
		return -EIO;
	}
#endif

	return 0;
}

/* --- Port to Controller driver API ------------------------------------ */
void mchp_xec_i2c_v3_ctrl_lock(const struct device *ctrl)
{
	struct i2c_mec5_data *const data = ctrl->data;

	k_mutex_lock(&data->bus_lock, K_FOREVER);
}

void mchp_xec_i2c_v3_ctrl_unlock(const struct device *ctrl)
{
	struct i2c_mec5_data *const data = ctrl->data;

	k_mutex_unlock(&data->bus_lock);
}

/* Caller must acquire Controller lock before calling */
int mchp_xec_i2c_v3_ctrl_port_switch(const struct device *ctrl, uint32_t freq, uint8_t port)
{
	struct i2c_mec5_data *const data = ctrl->data;

	if (ctrl == NULL) {
		return -EINVAL;
	}

	if ((freq == data->active_freq) && (port == data->active_port)) {
		return 0; /* nothing to do */
	}

	return config_hw(ctrl, freq, port, true);
}

/* --- Zephyr API: configure / get_config ------------------------------- */

/* internal */
static int i2c_mec5_config(const struct device *dev, uint32_t freq_hz, uint8_t port)
{
	struct i2c_mec5_data *ctx = dev->data;
	int ret = 0;

	if (port >= I2C_MEC5_CFG_MAX_PORTS) {
		LOG_ERR("Invalid port %u", port);
		return -ENOTSUP;
	}

	if ((freq_hz != KHZ(100)) && (freq_hz != KHZ(400)) && (freq_hz != KHZ(1000))) {
		return -ENOTSUP;
	}

	if ((port != ctx->active_port) || (freq_hz != ctx->active_freq)) {
		ret = config_hw(dev, freq_hz, port, true);
	}

	return ret;
}

/* Public API Uses device tree port value */
static int i2c_mec5_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_mec5_data *data = dev->data;
	uint32_t speed = I2C_SPEED_GET(dev_config);
	uint32_t freq = 0;
	int ret = 0;

	if ((((dev_config & I2C_MODE_CONTROLLER)) == 0) ||
	    ((dev_config & I2C_ADDR_10_BITS) != 0U)) {
		return -ENOTSUP;
	}

	ret = mec5_speed_to_freq(speed, &freq);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(&data->bus_lock, K_FOREVER);
	ret = i2c_mec5_config(dev, freq, data->active_port);
	k_mutex_unlock(&data->bus_lock);

	return ret;
}

static uint32_t get_bus_clock_from_hw(const struct device *dev)
{
	const struct i2c_mec5_config *cfg = dev->config;
	uint16_t bclk = i2c_mec5_bus_clk_r(cfg);
	uint32_t low = (bclk & I2C_MEC5_BUS_CLK_LOW_MSK) >> I2C_MEC5_BUS_CLK_LOW_POS;
	uint32_t high = (bclk & I2C_MEC5_BUS_CLK_HIGH_MSK) >> I2C_MEC5_BUS_CLK_HIGH_POS;
	uint32_t divisor = (low + 1U) + (high + 1U);
	uint32_t hz = I2C_MEC5_BAUD_CLOCK_HZ / divisor;

	return hz;
}

static int i2c_mec5_get_i2c_cfg(const struct device *dev, uint32_t *dev_config)
{
	uint32_t freq = 0, i2c_config = 0;
	int ret = 0;

	if (dev_config != NULL) {
		freq = get_bus_clock_from_hw(dev);
		i2c_config = I2C_SPEED_SET(mec5_bitrate_to_speed(freq));
#ifdef CONFIG_I2C_TARGET
		if (!i2c_mec5_target_enabled(dev)) {
			i2c_config |= I2C_MODE_CONTROLLER;
		}
#else
		i2c_config |= I2C_MODE_CONTROLLER;
#endif
		*dev_config = i2c_config;
	} else {
		ret = -EINVAL;
	}

	return ret;
}

static int i2c_mec5_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_mec5_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->bus_lock, K_FOREVER);

	ret = i2c_mec5_get_i2c_cfg(dev, dev_config);

	k_mutex_unlock(&data->bus_lock);

	return ret;
}

/* Port to Controller configure API.
 * Port sequence:
 * Call Port to Controller API to acquire lock
 * Call this API
 * Call Port to Controller API to release lock
 */
int mchp_i2c_xec_v3_config(const struct device *dev, uint32_t dev_config, uint8_t port)
{
	uint32_t speed = I2C_SPEED_GET(dev_config);
	uint32_t freq = 0;
	int ret = 0;

	if (dev == NULL) {
		return -EINVAL;
	}

	if ((((dev_config & I2C_MODE_CONTROLLER)) == 0) ||
	    ((dev_config & I2C_ADDR_10_BITS) != 0U)) {
		return -ENOTSUP;
	}

	ret = mec5_speed_to_freq(speed, &freq);
	if (ret != 0) {
		return ret;
	}

	if (port >= I2C_MEC5_CFG_MAX_PORTS) {
		LOG_ERR("Invalid port %u", port);
		return -ENOTSUP;
	}

	return i2c_mec5_config(dev, freq, port);
}

/* Port to Controller get config API.
 * Port sequence:
 * Call Port to Controller API to acquire lock
 * Call this API
 * Call Port to Controller API to release lock
 */
int mchp_i2c_xec_v3_get_config(const struct device *dev, uint32_t *dev_config, uint8_t *port)
{
	const struct i2c_mec5_config *xcfg = NULL;
	int ret = 0;

	if (dev == NULL) {
		return -EINVAL;
	}

	xcfg = dev->config;

	ret = i2c_mec5_get_i2c_cfg(dev, dev_config);

	if (port != NULL) {
		*port = (uint8_t)((i2c_mec5_config_r(xcfg) & I2C_MEC5_CFG_PORT_MSK) >>
				  I2C_MEC5_CFG_PORT_POS);
	}

	return ret;
}

/* --- Zephyr API: recover_bus ----------------------------------------- */

static int i2c_mec5_bb_check_scl(const struct device *ctrl, uint16_t nloops)
{
	const struct i2c_mec5_config *cfg = ctrl->config;
	uint8_t bbcr = i2c_mec5_bbcr_r(cfg);

	while (((bbcr & I2C_MEC5_BBCR_CLKI_HI) == 0) && (nloops != 0)) {
		k_busy_wait(10U);
		bbcr = i2c_mec5_bbcr_r(cfg);
		nloops--;
	}

	if ((bbcr & I2C_MEC5_BBCR_CLKI_HI) == 0) {
		i2c_mec5_bbcr_w(cfg, I2C_MEC5_BBCR_LIVECM_EN);
		return -ETIMEDOUT;
	}

	return 0;
}

/* Generate requested number of I2C clocks
 * Each clock pulse is a transition from low to high and back to low.
 * Caller is required to configure BB control of SCL and SDA as outputs and tri-stated(not driven).
 */
static void i2c_mec5_bb_gen_clocks(const struct device *ctrl, uint16_t num_clocks)
{
	const struct i2c_mec5_config *cfg = ctrl->config;
	uint8_t bbcr = i2c_mec5_bbcr_r(cfg);
	uint8_t bbcr_scl_hi = bbcr;
	uint8_t bbcr_scl_lo = bbcr & (uint8_t)~(I2C_MEC5_BBCR_SCL_TS);

	if ((num_clocks != 0) && ((bbcr & I2C_MEC5_BBCR_CLKI_HI) != 0)) {
		/* begin with SCL low */
		i2c_mec5_bbcr_w(cfg, bbcr_scl_lo);
		k_busy_wait(5U);
	}

	for (uint16_t n = 0; n < num_clocks; n++) {
		i2c_mec5_bbcr_w(cfg, bbcr_scl_hi); /* low-to-high */
		k_busy_wait(5U);
		i2c_mec5_bbcr_w(cfg, bbcr_scl_lo); /* high-to-low */
		k_busy_wait(5U);
	}

	/* Release SCL so we don't appear to be clock stretching */
	bbcr = i2c_mec5_bbcr_r(cfg);
	if ((bbcr & I2C_MEC5_BBCR_CLKI_HI) == 0) {
		i2c_mec5_bbcr_w(cfg, bbcr_scl_hi);
	}
}

/* Use I2C_SMB controller's bit-bang feature to generate an I2C STOP
 * which is defined as Low to High (rising edge) on SDA while SCL is high.
 * Caller must enable Bit-Bang mode with SCL and SDA as outputs.
 * 1. If SCL is low then release it and delay.
 * 2. Drive SDA low for one 1/2 100 KHz clock
 * 3. Release SDA
 * On exit, both BB Control should have both SCL and SDA tri-stated.
 */
static int i2c_mec5_bb_gen_stop(const struct device *ctrl)
{
	const struct i2c_mec5_config *cfg = ctrl->config;
	uint8_t bbcr = i2c_mec5_bbcr_r(cfg);

	/* Is SCL low? */
	if ((bbcr & I2C_MEC5_BBCR_CLKI_HI) == 0) {
		/* Yes, release SCL and delay to let pull-up pull it high */
		bbcr |= I2C_MEC5_BBCR_SCL_TS;
		i2c_mec5_bbcr_w(cfg, bbcr);
		k_busy_wait(5U);
	}

	/* Drive SDA low while SCL is high for 1/2 100KHz clock */
	bbcr &= (uint8_t)~(I2C_MEC5_BBCR_SDA_TS);
	i2c_mec5_bbcr_w(cfg, bbcr);
	k_busy_wait(5U);

	/* release SDA */
	bbcr |= I2C_MEC5_BBCR_SDA_TS;
	i2c_mec5_bbcr_w(cfg, bbcr);
	k_busy_wait(5U);

	return 0;
}

/* I2C v3.8 hardware lets us read pin live without disconnecting pins from
 * internal I2C logic and reconnecting to bit-bang logic.
 * Recovery sequence:
 * 1. Attempt controller reset. If successful return
 * 2. Check pin states using BBCR live feature.
 * 3. If SCL is low spin sampling SCL. If it remains low after spin period return error
 * 4. Enable bit-bang control mode where SCL/SDA pins states are set by BB hardware.
 * 5. Loop N times
 *      Generate a 9 clocks at ~100 KHz on SCL
 *      Samples SCL and SDA as inputs. If both are high exit loop with success.
 *    End Loop
 * 6. Disable bit-bang control. Keep live pin read enabled.
 * 7. Reset controller.
 * 8. If SCL and SDA are both high return success
 * 9. Else return error
 */
static int i2c_mec5_bb_recover(const struct device *ctrl, uint32_t freq, uint8_t port)
{
	const struct i2c_mec5_config *cfg = ctrl->config;
	int ret = 0;
	uint32_t n = 0;
	uint8_t bbcr = 0, i2c_sr = 0;
	uint8_t both_hi_msk = (I2C_MEC5_BBCR_CLKI_HI | I2C_MEC5_BBCR_DATI_HI);

	ret = config_hw(ctrl, freq, port, false);
	bbcr = i2c_mec5_bbcr_r(cfg);
	if (ret == 0) {
		i2c_sr = i2c_mec5_status_r(cfg);
		if (((bbcr & both_hi_msk) == both_hi_msk) && (i2c_sr == 0x81U)) {
			soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, MCHP_MEC_ECIA_GIRQ_EN);
			return 0;
		}
	}

	ret = i2c_mec5_bb_check_scl(ctrl, 100U);
	if (ret != 0) {
		LOG_ERR("I2C recov: SCL stuck low");
		return ret;
	}

	/* Enable Bit-Bang control of SCL and SDA with as outputs and tri-stated */
	bbcr = (I2C_MEC5_BBCR_BBM_EN | I2C_MEC5_BBCR_LIVECM_EN | I2C_MEC5_BBCR_CLDIR_OUT |
		I2C_MEC5_BBCR_DADIR_OUT | I2C_MEC5_BBCR_SCL_TS | I2C_MEC5_BBCR_SDA_TS);
	i2c_mec5_bbcr_w(cfg, bbcr);

	n = 100U;
	while (n != 0) {
		i2c_mec5_bb_gen_clocks(ctrl, 9U);
		k_busy_wait(10U);
		/* Generate an I2C STOP */
		ret = i2c_mec5_bb_gen_stop(ctrl);
		if (ret) {
			LOG_ERR("I2C recov: cannot gen STOP");
			return ret;
		}

		k_busy_wait(10U);
		bbcr = i2c_mec5_bbcr_r(cfg);

		if ((bbcr & both_hi_msk) == both_hi_msk) {
			break;
		}

		n--;
	}

	bbcr = I2C_MEC5_BBCR_LIVECM_EN;
	i2c_mec5_bbcr_w(cfg, bbcr);
	k_busy_wait(10U);

	ret = config_hw(ctrl, freq, port, false);

	bbcr = i2c_mec5_bbcr_r(cfg);
	if ((bbcr & both_hi_msk) != both_hi_msk) {
		LOG_INF("I2C recov failed");
		return -EIO;
	}

	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, MCHP_MEC_ECIA_GIRQ_EN);

	return 0;
}

static int i2c_mec5_recover_bus(const struct device *ctrl)
{
	struct i2c_mec5_data *ctx = ctrl->data;

	return i2c_mec5_bb_recover(ctrl, ctx->active_freq, ctx->active_port);
}

/* Port-to-Controller API passed PINCTRL info if needed */
int mchp_xec_i2c_v3_ctrl_recover_bus(const struct device *ctrl,
				     const struct pinctrl_dev_config *pcfg)
{
	struct i2c_mec5_data *ctx = ctrl->data;

	return i2c_mec5_bb_recover(ctrl, ctx->active_freq, ctx->active_port);
}

/* --- Zephyr API: transfer (sync) -------------------------------------- */

static int check_ctrl(const struct device *dev)
{
	const struct i2c_mec5_config *drvcfg = dev->config;
	struct i2c_mec5_data *ctx = dev->data;
	int ret = 0;
	uint8_t i2c_sr = i2c_mec5_status_r(drvcfg);

	if (i2c_sr != I2C_MEC5_STATUS_IDLE_EXPECTED) {
#ifdef CONFIG_I2C_MCHP_XEC_V3_ENABLE_RECOV_COUNTERS
		ctx->recov_attempts++;
#endif
		ret = i2c_mec5_bb_recover(dev, ctx->active_freq, ctx->active_port);
		if (ret != 0) {
#ifdef CONFIG_I2C_MCHP_XEC_V3_ENABLE_RECOV_COUNTERS
			ctx->recov_fails++;
#endif
		}
	}

	return ret;
}

/* Prerequitie: caller must have acquired the lock */
static int i2c_mec5_xfr(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			uint16_t addr)
{
	struct i2c_mec5_data *ctx = dev->data;
	int ret = 0;

	i2c_mec5_dbg_state_init(ctx);

#ifdef CONFIG_I2C_CALLBACK
	ctx->cb = NULL;
	ctx->userdata = NULL;
#endif
	i2c_mec5_dbg_state_update(ctx, 1U);

#ifdef CONFIG_I2C_TARGET
	if (i2c_mec5_target_enabled(dev)) {
		i2c_mec5_dbg_state_update(ctx, 2U);
		return -EBUSY;
	}
#endif

	ret = check_ctrl(dev);
	if (ret != 0) {
		i2c_mec5_dbg_state_update(ctx, 3U);
		return ret;
	}

	ret = i2c_mec5_sm_kickoff(ctx, msgs, num_msgs, addr);
	if (ret == 0) {
		if (k_sem_take(&ctx->xfer_done, K_MSEC(I2C_MEC5_XFR_TIMEOUT_MS)) != 0) {
			i2c_mec5_dbg_state_update(ctx, 4U); /* !!! TODO interrupt keeps firing !!! */
			config_hw(dev, ctx->active_freq, ctx->active_port, true);
			LOG_ERR("transfer timed out");
			ret = -ETIMEDOUT;
			ctx->state = CTRL_IDLE;
		} else {
			i2c_mec5_dbg_state_update(ctx, 5U);
			ret = ctx->result;
		}
	}

	ctx->state = CTRL_IDLE;

	i2c_mec5_dbg_state_update(ctx, 6U);

	return ret;
}

static int i2c_mec5_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			     uint16_t addr)
{
	struct i2c_mec5_data *ctx = dev->data;
	int ret = 0;

	if (num_msgs == 0U) {
		return 0;
	} else if (msgs == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&ctx->bus_lock, K_FOREVER);
	k_sem_reset(&ctx->xfer_done);

	i2c_mec5_dbg_state_update(ctx, 1U);

	ret = i2c_mec5_xfr(dev, msgs, num_msgs, addr);

	k_mutex_unlock(&ctx->bus_lock);

	return ret;
}

/* Port to Controller API. Port sequence:
 * Call Port to Controller to acquire lock
 * Call Port to Controller API to switch port
 * if port switch successful call this transfer API
 * Call Port to Controller API to release lock
 */
int mchp_i2c_xec_v3_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			     uint16_t addr)
{
	struct i2c_mec5_data *const ctx = dev->data;

	if (num_msgs == 0U) {
		return 0;
	} else if (msgs == NULL) {
		return -EINVAL;
	}

	k_sem_reset(&ctx->xfer_done);

	return i2c_mec5_xfr(dev, msgs, num_msgs, addr);
}

/* --- Zephyr API: transfer_cb (async) --------------------------------- */

#ifdef CONFIG_I2C_CALLBACK

static void i2c_mec5_cb_work_handler(struct k_work *work)
{
	struct i2c_mec5_data *ctx = CONTAINER_OF(work, struct i2c_mec5_data, cb_work);
	i2c_callback_t cb = ctx->cb;
	void *userdata = ctx->userdata;
	int result = ctx->result;

	ctx->cb = NULL;
	ctx->userdata = NULL;
	ctx->state = CTRL_IDLE;
	k_mutex_unlock(&ctx->bus_lock);

	if (cb != NULL) {
		cb(ctx->dev, result, userdata);
	}
}

/* Caller must acquire lock */
static int i2c_mec5_xfr_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			   uint16_t addr, i2c_callback_t cb, void *userdata)
{
	struct i2c_mec5_data *ctx = dev->data;
	int ret = 0;

#ifdef CONFIG_I2C_TARGET
	if (i2c_mec5_target_enabled(dev)) {
		return -EBUSY;
	}
#endif

	ret = check_ctrl(dev);
	if (ret != 0) {
		return ret;
	}

	ctx->cb = cb;
	ctx->userdata = userdata;

	ret = i2c_mec5_sm_kickoff(ctx, msgs, num_msgs, addr);
	if (ret != 0) {
		ctx->cb = NULL;
		ctx->userdata = NULL;
	}

	return ret;
}

static int i2c_mec5_transfer_cb(const struct device *dev,
				struct i2c_msg *msgs,
				uint8_t num_msgs,
				uint16_t addr,
				i2c_callback_t cb,
				void *userdata)
{
	struct i2c_mec5_data *ctx = dev->data;
	int ret = 0;
	uint8_t i2c_src = 0;

	if (num_msgs == 0U) {
		if (cb != NULL) {
			cb(dev, 0, userdata);
		}
		return 0;
	}

	if (msgs == NULL) {
		if (cb != NULL) {
			cb(dev, -EINVAL, userdata);
		}
		return -EINVAL;
	}

	/* Async path: lock the mutex until the callback runs.
	 * The cb_work handler releases it
	 */
	k_mutex_lock(&ctx->bus_lock, K_FOREVER);

	ret = i2c_mec5_xfr_cb(dev, msgs, num_msgs, addr, cb, userdata);
	if (ret != 0) {
		k_mutex_unlock(&ctx->bus_lock);
	}

	return ret;
}

/* Port to Controller asynchronous transfer API.
 * Requires driver to:
 * Call Controller lock acquire API
 * Call this API
 * If error call Controller lock release API
 */
int mchp_i2c_v3_transfer_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			    uint16_t addr, i2c_callback_t cb, void *userdata)
{
	return i2c_mec5_xfr_cb(dev, msgs, num_msgs, addr, cb, userdata);
}
#endif /* CONFIG_I2C_CALLBACK */

#ifdef CONFIG_I2C_TARGET

/* XEC I2C V3 target-mode support.
 * Control-register word for writes made DURING an active target transaction.
 *
 * CRITICAL: does NOT include PCLR. PCLR de-asserts all Status bits (including
 * NOSVC -> 1), which releases the target-mode clock stretch. In target-TX,
 * stretch must only be released by writing Data with the response byte; any
 * earlier Control-with-PCLR write releases stretch while the Data register
 * still holds the received address byte, causing the hardware to transmit
 * the address byte as the first data byte of the read phase.
 *
 * Use CTRL_TGT_REST (with PCLR) only at STOP / cleanup, when the stretch
 * mechanic is no longer in play.
 */
#define CTRL_TGT_RUN  (I2C_MEC5_CTRL_ESO | I2C_MEC5_CTRL_ENI)
#define CTRL_TGT_REST (I2C_MEC5_CTRL_PCLR | CTRL_TGT_RUN)

/* --- OwnAddr slot helpers -------------------------------------------- */

static int find_free_slot(struct i2c_mec5_data *ctx)
{
	int slot = -1;
	int i;

	for (i = 0; i < 2; i++) {
		if (ctx->targets[i] == NULL) {
			slot = i;
			break;
		}
	}
	return slot;
}

static int find_slot_by_addr(struct i2c_mec5_data *ctx, uint8_t addr7)
{
	int slot = -1;
	int i;

	for (i = 0; i < 2; i++) {
		if (ctx->targets[i] != NULL &&
		    ctx->targets[i]->address == addr7) {
			slot = i;
			break;
		}
	}
	return slot;
}

static void own_addr_write_slot(const struct i2c_mec5_config *cfg,
				int slot, uint8_t addr7)
{
	uint32_t v = i2c_mec5_own_addr_r(cfg);

	if (slot == 0) {
		v &= ~I2C_MEC5_OWN_ADDR0_MSK;
		v |= ((uint32_t)addr7 & 0x7FU) << I2C_MEC5_OWN_ADDR0_POS;
	} else {
		v &= ~I2C_MEC5_OWN_ADDR1_MSK;
		v |= ((uint32_t)addr7 & 0x7FU) << I2C_MEC5_OWN_ADDR1_POS;
	}
	i2c_mec5_own_addr_w(cfg, v);
}

/* --- Register API ---------------------------------------------------- */

static int i2c_mec5_target_register_impl(const struct device *dev,
					 struct i2c_target_config *tcfg)
{
	struct i2c_mec5_data *ctx = dev->data;
	const struct i2c_mec5_config *cfg = dev->config;
	unsigned int key = 0;
	int slot = 0;
	int ret = 0;

	if (tcfg == NULL || tcfg->callbacks == NULL) {
		ret = -EINVAL;
	} else if ((tcfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0U) {
		ret = -ENOTSUP;
	} else {
		key = irq_lock();
		slot = find_free_slot(ctx);
		if (slot < 0) {
			ret = -ENOMEM;
		} else {
			ctx->targets[slot] = tcfg;
			own_addr_write_slot(cfg, slot, (uint8_t)tcfg->address);
			ret = 0;
		}
		irq_unlock(key);
	}

	return ret;
}

static int i2c_mec5_target_unregister_impl(const struct device *dev,
					   struct i2c_target_config *tcfg)
{
	struct i2c_mec5_data *ctx = dev->data;
	const struct i2c_mec5_config *cfg = dev->config;
	unsigned int key;
	int i;
	int ret = -ENOENT;

	key = irq_lock();
	for (i = 0; i < 2; i++) {
		if (ctx->targets[i] == tcfg) {
			ctx->targets[i] = NULL;
			own_addr_write_slot(cfg, i, 0U);
			ret = 0;
			break;
		}
	}
	irq_unlock(key);

	return ret;
}

int mchp_xec_i2c_v3_ctrl_target_register(const struct device *ctrl,
					 struct i2c_target_config *tcfg)
{
	if ((ctrl == NULL) || (tcfg == NULL)) {
		return -EINVAL;
	}

	return i2c_mec5_target_register_impl(ctrl, tcfg);
}

int mchp_xec_i2c_v3_ctrl_target_unregister(const struct device *ctrl,
					   struct i2c_target_config *tcfg)
{
	if ((ctrl == NULL) || (tcfg == NULL)) {
		return -EINVAL;
	}

	return i2c_mec5_target_unregister_impl(ctrl, tcfg);
}
/* --- ISR dispatch helpers -------------------------------------------- */

static bool has_buf_mode(const struct i2c_target_config *tcfg)
{
	bool yes = false;

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	yes = (tcfg->callbacks->buf_write_received != NULL) ||
	      (tcfg->callbacks->buf_read_requested != NULL);
#else
	ARG_UNUSED(tcfg);
#endif
	return yes;
}

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
/*
 * Flush any accumulated buffer-mode RX bytes to the application and reset
 * the buffer. Called on phase boundaries (repeated START while in RX) and
 * at STOP, so repeated-START-without-STOP sequences do not lose data.
 */
static void flush_rx_buffer(struct i2c_mec5_data *ctx)
{
	struct i2c_target_config *tcfg = ctx->targets[ctx->tgt_slot];

	if (tcfg != NULL && has_buf_mode(tcfg) &&
	    tcfg->callbacks->buf_write_received != NULL &&
	    ctx->tgt_buf_idx > 0U) {
		tcfg->callbacks->buf_write_received(tcfg,
						    ctx->tgt_buf,
						    ctx->tgt_buf_idx);
	}
	ctx->tgt_buf_idx = 0U;
}
#endif

static void handle_addressed(struct i2c_mec5_data *ctx, uint8_t status)
{
	const struct i2c_mec5_config *cfg = ctx->dev->config;
	struct i2c_target_config *tcfg = NULL;
	uint32_t regv = 0;
	int slot = -1;
	bool is_read_from_target = false;
	uint8_t addr_byte = 0;
	uint8_t addr7 = 0;
	uint8_t val = 0;

	ARG_UNUSED(status);

	i2c_mec5_dbg_state_update(ctx, 0x90U);

	/* Enable STOP and IDLE detection */
	regv = i2c_mec5_completion_r(cfg);
	regv |= 0xffffff00u;
	i2c_mec5_completion_w(cfg, regv);
	regv = i2c_mec5_config_r(cfg);
	regv |= I2C_MEC5_CFG_ENIDI | I2C_MEC5_CFG_STOP_DET_EN;
	i2c_mec5_config_w(cfg, regv);

	/*
	 * Repeated-START boundary: if we were mid-RX, the current buffer-mode
	 * contents belong to the phase that just ended. Flush them before we
	 * overwrite tgt_buf_idx for the new phase. (Safe no-op if tgt_state
	 * was TGT_IDLE or TX — buffer is already empty.)
	 */
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	if (ctx->tgt_state == TGT_ADDRESSED_RX) {
		i2c_mec5_dbg_state_update(ctx, 0x91U);
		flush_rx_buffer(ctx);
	}
#endif

	/*
	 * Reading Data yields the matched address + R/W bit and clears AAT.
	 * In RX direction this also sets NOSVC=1 (releases stretch so the
	 * external master can clock the first data byte). In TX direction
	 * NOSVC stays 0 — stretch remains active until we write Data with
	 * the first response byte.
	 */
	addr_byte = i2c_mec5_data_r(cfg);
	addr7 = (addr_byte >> 1) & 0x7FU;
	is_read_from_target = (addr_byte & 0x01U) != 0U;

	slot = find_slot_by_addr(ctx, addr7);
	if (slot < 0) {
		i2c_mec5_dbg_state_update(ctx, 0x92U);
		/* Addressed but no matching registered config — ignore.
		 * Write a benign 0xFF to Data in case hardware is in TX,
		 * to avoid transmitting the stale address byte.
		 */
		if (is_read_from_target) {
			i2c_mec5_data_w(cfg, 0xFFU);
		}
		ctx->tgt_state = TGT_IDLE;
	} else if (is_read_from_target) {
		i2c_mec5_dbg_state_update(ctx, 0x93U);
		ctx->tgt_slot = (uint8_t)slot;
		tcfg = ctx->targets[slot];
		ctx->tgt_state = TGT_ADDRESSED_TX;
		/*
		 * TX path — write Data with first response byte FIRST, with
		 * no intervening Control write. The Data write is what sets
		 * NOSVC=1 and releases the stretch; a Control|PCLR in between
		 * would release stretch prematurely with the address byte
		 * still in Data.
		 */
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
		if (has_buf_mode(tcfg) &&
		    tcfg->callbacks->buf_read_requested != NULL) {
			uint8_t *p = NULL;
			uint32_t len = 0;
			int rc = tcfg->callbacks->buf_read_requested(tcfg, &p, &len);

			i2c_mec5_dbg_state_update(ctx, 0x94U);

			if (rc == 0 && p != NULL && len > 0U) {
				i2c_mec5_dbg_state_update(ctx, 0x95U);
				ctx->tgt_tx_ptr = p;
				ctx->tgt_buf_len = len;
				i2c_mec5_data_w(cfg, p[0]);
				ctx->tgt_buf_idx = 1U;
			} else {
				i2c_mec5_data_w(cfg, 0xFFU);
			}
		} else
#endif
		{
			i2c_mec5_dbg_state_update(ctx, 0x96U);
			if (tcfg->callbacks->read_requested != NULL &&
			    tcfg->callbacks->read_requested(tcfg, &val) == 0) {
				i2c_mec5_dbg_state_update(ctx, 0x97U);
				i2c_mec5_data_w(cfg, val);
			} else {
				i2c_mec5_data_w(cfg, 0xFFU);
			}
		}
	} else {
		i2c_mec5_dbg_state_update(ctx, 0x98U);
		ctx->tgt_slot = (uint8_t)slot;
		tcfg = ctx->targets[slot];
		ctx->tgt_state = TGT_ADDRESSED_RX;
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
		ctx->tgt_buf_idx = 0U;
#endif
		/*
		 * RX path — re-prime Control.ACK (without PCLR) so a previous
		 * phase that cleared ACK (buffer near-full or negative
		 * write_received) doesn't cause the first byte of this phase
		 * to be NAK'd. Reading Data above already released stretch
		 * for the master's first data byte; Control.ACK only needs
		 * to be set before that byte's 9th clock.
		 */
		i2c_mec5_ctrl_w(cfg, CTRL_TGT_RUN | I2C_MEC5_CTRL_ACK);
		if (!has_buf_mode(tcfg) &&
		    tcfg->callbacks->write_requested != NULL) {
			i2c_mec5_dbg_state_update(ctx, 0x99U);
			(void)tcfg->callbacks->write_requested(tcfg);
		}
	}

	i2c_mec5_dbg_state_update(ctx, 0x9AU);
}

static void handle_target_service(struct i2c_mec5_data *ctx, uint8_t status)
{
	const struct i2c_mec5_config *cfg = ctx->dev->config;
	struct i2c_target_config *tcfg = ctx->targets[ctx->tgt_slot];
	uint8_t rx;
	uint8_t next;

	i2c_mec5_dbg_state_update(ctx, 0xA0U);

	if (tcfg == NULL) {
		i2c_mec5_dbg_state_update(ctx, 0xA1U);
		/* Registration vanished mid-transaction; just drain. */
		if (ctx->tgt_state == TGT_ADDRESSED_RX) {
			i2c_mec5_dbg_state_update(ctx, 0xA2U);
			(void)i2c_mec5_data_r(cfg);
		} else {
			i2c_mec5_data_w(cfg, 0xFFU);
		}
	} else if (ctx->tgt_state == TGT_ADDRESSED_RX) {
		i2c_mec5_dbg_state_update(ctx, 0xA3U);
		rx = i2c_mec5_data_r(cfg);
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
		if (has_buf_mode(tcfg)) {
			i2c_mec5_dbg_state_update(ctx, 0xA4U);
			if (ctx->tgt_buf_idx < cfg->targ_buf_size) {
				i2c_mec5_dbg_state_update(ctx, 0xA5U);
				ctx->tgt_buf[ctx->tgt_buf_idx] = rx;
				ctx->tgt_buf_idx++;
			}
			/* If the buffer is about to be full, clear Control.ACK
			 * so the hardware NAKs the next byte (README §target
			 * receiver next-to-last-byte handling).
			 */
			if ((ctx->tgt_buf_idx + 1U) >= cfg->targ_buf_size) {
				i2c_mec5_dbg_state_update(ctx, 0xA6U);
				i2c_mec5_ctrl_w(cfg, CTRL_TGT_RUN);
			}
		} else
#endif
		if (tcfg->callbacks->write_received != NULL) {
			i2c_mec5_dbg_state_update(ctx, 0xA7U);
			/*
			 * Per Zephyr API: a negative return means "don't
			 * accept the next byte". Clear Control.ACK so the
			 * hardware NAKs the next 9th clock.
			 */
			if (tcfg->callbacks->write_received(tcfg, rx) != 0) {
				i2c_mec5_dbg_state_update(ctx, 0xA8U);
				i2c_mec5_ctrl_w(cfg, CTRL_TGT_RUN);
			}
		}
	} else if (ctx->tgt_state == TGT_ADDRESSED_TX) {
		i2c_mec5_dbg_state_update(ctx, 0xA9U);
		if ((status & I2C_MEC5_STATUS_LRB) != 0U) {
			i2c_mec5_dbg_state_update(ctx, 0xAAU);
			/* Master NAK'd: transaction ending. README target-TX:
			 * dummy write to clear Status; not transmitted.
			 */
			i2c_mec5_data_w(cfg, 0xFFU);
			ctx->tgt_state = TGT_STOP_PENDING;
		} else {
			i2c_mec5_dbg_state_update(ctx, 0xABU);
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
			if (has_buf_mode(tcfg) && ctx->tgt_tx_ptr != NULL) {
				i2c_mec5_dbg_state_update(ctx, 0xACU);
				if (ctx->tgt_buf_idx < ctx->tgt_buf_len) {
					i2c_mec5_dbg_state_update(ctx, 0xADU);
					i2c_mec5_data_w(cfg,
						ctx->tgt_tx_ptr[ctx->tgt_buf_idx]);
					ctx->tgt_buf_idx++;
				} else {
					i2c_mec5_data_w(cfg, 0xFFU);
				}
			} else
#endif
			if (tcfg->callbacks->read_processed != NULL &&
			    tcfg->callbacks->read_processed(tcfg, &next) == 0) {
				i2c_mec5_dbg_state_update(ctx, 0xAEU);
				i2c_mec5_data_w(cfg, next);
			} else {
				i2c_mec5_data_w(cfg, 0xFFU);
			}
		}
	}

	i2c_mec5_dbg_state_update(ctx, 0xAFU);
}

static void handle_target_stop(struct i2c_mec5_data *ctx)
{
	const struct i2c_mec5_config *cfg = ctx->dev->config;
	struct i2c_target_config *tcfg = ctx->targets[ctx->tgt_slot];
	uint32_t regv = 0;

	i2c_mec5_dbg_state_update(ctx, 0xB0U);

	/* Required: disable STOP detection before clear sequence */
	regv = i2c_mec5_config_r(cfg);
	regv &= (uint32_t)~(I2C_MEC5_CFG_STOP_DET_EN);
	i2c_mec5_config_w(cfg, regv);

	/* README: read-discard Data to clear Status.STS. */
	(void)i2c_mec5_data_r(cfg);

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	if (ctx->tgt_state == TGT_ADDRESSED_RX) {
		i2c_mec5_dbg_state_update(ctx, 0xB1U);
		flush_rx_buffer(ctx);
	}
#endif

	if (tcfg != NULL && tcfg->callbacks->stop != NULL) {
		i2c_mec5_dbg_state_update(ctx, 0xB2U);
		(void)tcfg->callbacks->stop(tcfg);
	}

	/* Return to rest state: ACK re-primed so the next AAT starts clean
	 * even if we never re-enter handle_addressed before then. PCLR is
	 * safe here — the transaction is over, no stretch is in play.
	 */
	i2c_mec5_ctrl_w(cfg, CTRL_TGT_REST | I2C_MEC5_CTRL_ACK);

	ctx->tgt_state = TGT_IDLE;
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	ctx->tgt_buf_len = 0U;
	ctx->tgt_tx_ptr = NULL;
#endif

	i2c_mec5_dbg_state_update(ctx, 0xB3U);
}

/* --- Dispatcher (called from main ISR when controller is idle) ------- */

static void i2c_mec5_target_error(struct i2c_mec5_data *ctx, enum mec5_sm_action act,
				  uint8_t status)
{
	struct i2c_target_config *tcfg = NULL;
	enum i2c_error_reason err_reason = I2C_ERROR_GENERIC;

	i2c_mec5_dbg_state_update(ctx, 0xB4U);

	if (act == ACT_LOST_ARB) {
		i2c_mec5_dbg_state_update(ctx, 0xB5U);
		err_reason = I2C_ERROR_ARBITRATION;
	}

	if (ctx->tgt_state != TGT_IDLE) {
		i2c_mec5_dbg_state_update(ctx, 0xB6U);
		tcfg = ctx->targets[ctx->tgt_slot];
		if ((tcfg != NULL) && (tcfg->callbacks != NULL) &&
		    (tcfg->callbacks->error != NULL)) {
			i2c_mec5_dbg_state_update(ctx, 0xB7U);
			tcfg->callbacks->error(tcfg, err_reason);
		}
	} else { /* send to all targets */
		i2c_mec5_dbg_state_update(ctx, 0xB8U);
		for (size_t n = 0; n < 2U; n++) {
			tcfg = ctx->targets[n];
			if ((tcfg != NULL) && (tcfg->callbacks != NULL) &&
			    (tcfg->callbacks->error != NULL)) {
				i2c_mec5_dbg_state_update(ctx, 0xB9U);
				tcfg->callbacks->error(tcfg, err_reason);
			}
		}
	}

	ctx->tgt_state = TGT_IDLE;

	i2c_mec5_dbg_state_update(ctx, 0xBAU);
}

static void i2c_mec5_target_isr_dispatch(struct i2c_mec5_data *ctx, enum mec5_sm_action act,
					 uint8_t status)
{
	if (act == ACT_TGT_ADDRESSED) {
		handle_addressed(ctx, status);
	} else if (act == ACT_STOP_DET) {
		handle_target_stop(ctx);
	} else if (act == ACT_SERVICE) {
		if (ctx->tgt_state != TGT_IDLE) {
			handle_target_service(ctx, status);
		}
	} else if ((act == ACT_BUS_ERROR) || (act == ACT_LOST_ARB)) {
		i2c_mec5_target_error(ctx, act, status);
	} else {
		/* Other actions (IDLE) not expected in target-only path. */
		i2c_mec5_dbg_state_update(ctx, 0x8AU);
		return;
	}
}

#endif /* CONFIG_I2C_TARGET */

/* --- ISR (single exit) ----------------------------------------------- */

static void i2c_mec5_isr(const struct device *dev)
{
	struct i2c_mec5_data *ctx = dev->data;
	const struct i2c_mec5_config *cfg = dev->config;
	uint8_t  status = i2c_mec5_status_r(cfg);
	uint32_t comp   = i2c_mec5_completion_r(cfg);
	uint32_t icfg = i2c_mec5_config_r(cfg);
	enum mec5_sm_action act;
	uint32_t comp_clear = 0U, regv = 0;

	i2c_mec5_dbg_state_update(ctx, 0x80U);

	/* Decode highest-priority action. Order matters: BER/LAB are terminal,
	 * target addressing and bus events get handled before normal service.
	 */
	if ((status & I2C_MEC5_STATUS_BER) != 0U) {
		i2c_mec5_dbg_state_update(ctx, 0x81U);
		act = ACT_BUS_ERROR;
		comp_clear |= I2C_MEC5_COMP_BER;
	} else if ((status & I2C_MEC5_STATUS_LAB) != 0U) {
		i2c_mec5_dbg_state_update(ctx, 0x82U);
		act = ACT_LOST_ARB;
		comp_clear |= I2C_MEC5_COMP_LAB;
	} else if ((status & I2C_MEC5_STATUS_AAT) != 0U) {
		i2c_mec5_dbg_state_update(ctx, 0x83U);
		act = ACT_TGT_ADDRESSED;
	} else if (((comp & I2C_MEC5_COMP_IDLE) != 0U) && ((icfg & I2C_MEC5_CFG_ENIDI) != 0)) {
		i2c_mec5_dbg_state_update(ctx, 0x84U);
		act = ACT_IDLE;
		comp_clear |= I2C_MEC5_COMP_IDLE;
		/*
		 * Completion.IDLE is level-latched against NBB. Clearing the
		 * latch alone does not stop re-trigger — the peripheral keeps
		 * re-asserting the IRQ while NBB=1 and ENIDI=1. Disable ENIDI
		 * HERE (not in the SM handler) so no further IDLE IRQs fire
		 * even if sm_step takes time.
		 */
		regv  = i2c_mec5_config_r(cfg);
		regv &= (uint32_t)~(I2C_MEC5_CFG_ENIDI | I2C_MEC5_CFG_STOP_DET_EN);
		i2c_mec5_config_w(cfg, regv);

	} else if ((status & I2C_MEC5_STATUS_STS) != 0U) {
		i2c_mec5_dbg_state_update(ctx, 0x85U);
		act = ACT_STOP_DET;
	} else if ((status & I2C_MEC5_STATUS_NOSVC) == 0U) {
		i2c_mec5_dbg_state_update(ctx, 0x86U);
		act = ACT_SERVICE;
	} else {
		i2c_mec5_dbg_state_update(ctx, 0x87U);
		act = ACT_NONE;
	}

	/* Controller-mode transaction in progress takes priority.
	 * Otherwise route to target-mode dispatcher (compile-time gated).
	 */
	if (ctx->state != CTRL_IDLE) {
		i2c_mec5_sm_step(ctx, act, status);
	}
#ifdef CONFIG_I2C_TARGET
	else {
		i2c_mec5_target_isr_dispatch(ctx, act, status);
	}
#endif

	i2c_mec5_dbg_state_update(ctx, 0x88U);

	/* Tail: clear latched Completion bits we acted on. Hardware will
	 * re-assert the peripheral interrupt until its Status bits are
	 * cleared through the side-effect reads/writes above. No explicit
	 * NVIC ack needed — irq_connect'd level IRQ auto-acks on peripheral
	 * deassert.
	 */
	if (comp_clear != 0U) {
		i2c_mec5_dbg_state_update(ctx, 0x89U);
		i2c_mec5_completion_w(cfg, comp_clear);
	}

	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);
	i2c_mec5_dbg_state_update(ctx, 0x8FU);
}

/* --- Init ------------------------------------------------------------ */

static int i2c_mec5_init(const struct device *dev)
{
	const struct i2c_mec5_config *cfg = dev->config;
	struct i2c_mec5_data *ctx = dev->data;

	ctx->dev = dev;
	ctx->ops = &i2c_mec5_pio_ops;
	ctx->state = CTRL_IDLE;
	ctx->active_freq = 0;
	ctx->active_port = I2C_MEC5_CFG_MAX_PORTS;

	k_mutex_init(&ctx->bus_lock);
	k_sem_init(&ctx->xfer_done, 0, 1);

#ifdef CONFIG_I2C_CALLBACK
	k_work_init(&ctx->cb_work, i2c_mec5_cb_work_handler);
#endif

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	ctx->tgt_buf = cfg->targ_buf;
#endif

	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, MCHP_MEC_ECIA_GIRQ_DIS);
	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);

	if (cfg->irq_connect != NULL) {
		cfg->irq_connect();
	}

	return 0;
}

/* --- Driver API struct ----------------------------------------------- */

static DEVICE_API(i2c, i2c_mec5_driver_api) = {
	.configure     = i2c_mec5_configure,
	.get_config    = i2c_mec5_get_config,
	.transfer      = i2c_mec5_transfer,
	.recover_bus   = i2c_mec5_recover_bus,
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb   = i2c_mec5_transfer_cb,
#endif
#ifdef CONFIG_I2C_TARGET
	.target_register   = i2c_mec5_target_register_impl,
	.target_unregister = i2c_mec5_target_unregister_impl,
#endif
#ifdef CONFIG_I2C_RTIO
	/* TODO(rtio-native): native SQE-into-SM path once performance warrants. */
	.iodev_submit  = i2c_iodev_submit_fallback,
#endif
};

/* --- Per-instance instantiation -------------------------------------- */
#define I2C_MEC5_GIRQ_DT(inst) MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, 0))
#define I2C_MEC5_GIRQ_POS_DT(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, 0))

#ifdef CONFIG_I2C_TARGET
#define I2C_MEC5_TARG_BUF_DEF_DT(i)                                            \
	static __aligned(4) uint8_t i2c_mec5_targ_buf_##i[DT_INST_PROP(i, target_buf_size)];

#define I2C_MEC5_TARG_BUF_DT(i)                                                \
	.targ_buf = i2c_mec5_targ_buf_##i,                                     \
	.targ_buf_size = DT_INST_PROP(i, target_buf_size),
#else
#define I2C_MEC5_TARG_BUF_DEF_DT(i)
#define I2C_MEC5_TARG_BUF_DT(i)
#endif

#define I2C_MEC5_INIT(inst)                                                    \
	static void i2c_mec5_irq_connect_##inst(void)                          \
	{                                                                      \
		IRQ_CONNECT(DT_INST_IRQN(inst),                                \
			    DT_INST_IRQ(inst, priority),                       \
			    i2c_mec5_isr,                                      \
			    DEVICE_DT_INST_GET(inst), 0);                      \
		irq_enable(DT_INST_IRQN(inst));                                \
	}                                                                      \
	I2C_MEC5_TARG_BUF_DEF_DT(inst)                                         \
	static struct i2c_mec5_data i2c_mec5_data_##inst;                      \
									       \
	static const struct i2c_mec5_config i2c_mec5_cfg_##inst = {            \
		.base         = DT_INST_REG_ADDR(inst),                        \
		.girq = I2C_MEC5_GIRQ_DT(inst),                                \
		.girq_pos = I2C_MEC5_GIRQ_POS_DT(inst),                        \
		.enc_pcr = DT_INST_PROP(inst, pcr_scr),                        \
		.general_call = DT_INST_PROP(inst, general_call),              \
		.irq_connect  = i2c_mec5_irq_connect_##inst,                   \
		I2C_MEC5_TARG_BUF_DT(inst)                                     \
	};                                                                     \
									       \
	I2C_DEVICE_DT_INST_DEFINE(inst,                                        \
				  i2c_mec5_init, NULL,                         \
				  &i2c_mec5_data_##inst,                       \
				  &i2c_mec5_cfg_##inst,                        \
				  POST_KERNEL,                                 \
				  CONFIG_I2C_INIT_PRIORITY,                    \
				  &i2c_mec5_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_MEC5_INIT)
