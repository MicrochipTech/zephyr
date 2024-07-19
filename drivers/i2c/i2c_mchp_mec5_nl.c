/*
 * Copyright (c) 2024 Microchip Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_i2c_nl

#include <zephyr/kernel.h>
#include <soc.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/devicetree/dma.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/sys_io.h>
LOG_MODULE_REGISTER(i2c_nl_mchp, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_dmac_api.h>
#include <mec_ecia_api.h>
#include <mec_i2c_api.h>

#define I2C_NL_DEBUG_ISR
#define I2C_NL_DEBUG_STATE
#define I2C_NL_DEBUG_STATE_ENTRIES	64

#define I2C_NL_USE_DMA_DRIVER

#define I2C_NL_RESET_WAIT_US		20

#define I2C_NL_SMBUS_TMOUT_MAX_MS	35

/* I2C timeout is  10 ms (WAIT_INTERVAL * WAIT_COUNT) */
#define WAIT_INTERVAL_US		50
#define WAIT_COUNT			200
#define STOP_WAIT_COUNT			500
#define PIN_CFG_WAIT			50

/* I2C recover SCL low retries */
#define I2C_NL_RECOVER_SCL_LOW_RETRIES	10
/* I2C recover SDA low retries */
#define I2C_NL_RECOVER_SDA_LOW_RETRIES	3
/* I2C recovery bit bang delay */
#define I2C_NL_RECOVER_BB_DELAY_US	5
/* I2C recovery SCL sample delay */
#define I2C_NL_RECOVER_SCL_DELAY_US	50

#define I2C_NL_CTRL_WR_DLY		8

#define I2C_NL_MAX_TARGET_ADDRS		2

enum i2c_mec5_nl_state {
	I2C_NL_STATE_CLOSED = 0,
	I2C_NL_STATE_OPEN,
};

enum i2c_mec5_nl_error {
	I2C_NL_ERR_NONE = 0,
	I2C_NL_ERR_BUS,
	I2C_NL_ERR_LOST_ARB,
	I2C_NL_ERR_NACK_FROM_TARGET,
	I2C_NL_ERR_TIMEOUT,
};

enum i2c_mec5_nl_direction {
	I2C_NL_DIR_NONE = 0,
	I2C_NL_DIR_WR,
	I2C_NL_DIR_RD,
};

enum i2c_mec5_nl_fmt_addr {
	I2C_NL_FMT_ADDR_WR = 0,
	I2C_NL_FMT_ADDR_RD,
};

enum i2c_mec5_nl_start {
	I2C_NL_START_NONE = 0,
	I2C_NL_START_NORM,
	I2C_NL_START_RPT,
};

enum i2c_mec5_nl_isr_state {
	I2C_NL_ISR_STATE_GEN_START = 0,
	I2C_NL_ISR_STATE_CHK_ACK,
	I2C_NL_ISR_STATE_WR_DATA,
	I2C_NL_ISR_STATE_RD_DATA,
	I2C_NL_ISR_STATE_GEN_STOP,
	I2C_NL_ISR_STATE_EV_IDLE,
	I2C_NL_ISR_STATE_NEXT_MSG,
	I2C_NL_ISR_STATE_EXIT_1,
	I2C_NL_ISR_STATE_MAX
};

enum i2c_mec5_nl_kevents {
	I2C_NL_KEV_IDLE_POS = 0,
	I2C_NL_KEV_BERR_POS,
	I2C_NL_KEV_LAB_ERR_POS,
	I2C_NL_KEV_CM_NAK_POS,
	I2C_NL_KEV_W2R_POS,
	I2C_NL_KEV_CM_DONE_POS,
	I2C_NL_KEV_TM_START_POS,
	I2C_NL_KEV_TM_STOP_POS,
	I2C_NL_KEV_TM_DONE_POS,
	I2C_NL_KEV_DMA_CM_DONE_POS,
	I2C_NL_KEV_DMA_CM_ERR_POS,
	I2C_NL_KEV_DMA_TM_DONE_POS,
	I2C_NL_KEV_DMA_TM_ERR_POS,
};

#define I2C_NL_WAIT_EVENTS_MSK (BIT(I2C_NL_KEV_IDLE_POS) | BIT(I2C_NL_KEV_CM_NAK_POS) \
				| BIT(I2C_NL_KEV_DMA_CM_ERR_POS) \
				| BIT(I2C_NL_KEV_DMA_TM_ERR_POS))

struct i2c_mec5_nl_config {
	struct mec_i2c_smb_regs *regs;
	uint32_t clock_freq;
	uint32_t init_pin_wait_us;
	uint32_t cfg_pin_wait_us;
	uint8_t port_sel;
	struct gpio_dt_spec sda_gpio;
	struct gpio_dt_spec scl_gpio;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
	const struct device *dma_dev_cm;
	const struct device *dma_dev_tm;
	struct dma_regs *dma_cm_regs;
	struct dma_regs *dma_tm_regs;
	uint8_t dma_cm_chan;
	uint8_t dma_tm_chan;
	uint8_t cm_dma_trigsrc;
	uint8_t tm_dma_trigsrc;
};

#define I2C_NL_MAX_XFR_LEN		0xfffcu

#define I2C_NL_DRV_XFRBUF_PAD_SIZE	4u

#define I2C_MEC5_NL_DMA_BLK_CFGS	2

#define I2C_NL_DMA_BLK_WR		0
#define I2C_NL_DMA_BLK_RD		1

#define I2C_NL_XFR_FLAG_ASYNC		BIT(0)
#define I2C_NL_XFR_FLAG_TM_IDLE_REQ	BIT(4)

#ifdef I2C_NL_DEBUG_STATE
struct i2c_mec5_nl_dbg {
	uint8_t state;
	uint8_t i2c_status;
	uint32_t i2c_compl;
	uint32_t i2c_config;
	uint32_t i2c_cm_cmd;
	uint32_t i2c_tm_cmd;
};
#endif

/* I2C message group for Network layer hardware
 * Hardware can support protocols:
 * 1. I2C read of a single message. Length <= I2C_NL_MAX_XFR_LEN
 * 2. I2C write of multiple messages. Combined lenth <= I2C_NL_MAX_XFR_LEN
 * 3. I2C write multiple followed by one I2C read. Same length limits.
 * The driver only supports on read message to avoid a HW race conditions.
 */
struct i2c_msg_group {
	uint16_t wr_len;
	uint8_t wr_idx;		/* start index in message array of firt write message */
	uint8_t wr_msg_cnt;	/* number of consecutive write messages */
	uint16_t rd_len;
	uint8_t rd_idx;		/* index in message array of single read message */
	uint8_t rd_msg_cnt;	/* number of consecutive read messages (0 or 1) */
};

struct i2c_mec5_nl_data {
	const struct i2c_mec5_nl_config *devcfg;
	struct mec_i2c_smb_ctx ctx;
	struct k_sem lock;
	struct k_event events;
	uint32_t uconfig;
	uint32_t i2c_status;
	uint32_t xfr_tmout;
	struct i2c_msg_group mgrp;
	struct i2c_msg *msgs;
	uint8_t nmsgs;
	uint8_t msgidx;
	uint8_t xfrflags;
	uint8_t rsvd1[1];
	uint8_t *rembuf;
	uint16_t remlen;
	uint8_t rsvd2[2];
	uint8_t cm_target_wr_i2c_addr;
	uint8_t speed_id;
	uint8_t state;
	uint8_t mdone;
	struct dma_config dma_cfg_cm_tx;
	struct dma_config dma_cfg_cm_rx;
	struct dma_block_config dma_blk_cm_tx;
	struct dma_block_config dma_blk_cm_rx;
	uint8_t *xfrbuf;
	size_t xfrbuf_max;

#ifdef CONFIG_I2C_TARGET
	struct i2c_target_config *target_cfgs[I2C_NL_MAX_TARGET_ADDRS];
	struct i2c_target_config *active_target_cfg;
	struct dma_config dma_cfg_tm_tx;
	struct dma_config dma_cfg_tm_rx;
	struct dma_block_config dma_blk_tm_tx;
	struct dma_block_config dma_blk_tm_rx;
	struct dma_status dsts;
	uint8_t *tm_tx_buf;
	size_t tm_tx_buf_max;
	uint8_t *tm_rx_buf;
	size_t tm_rx_buf_max;
	bool targets_registered;
	uint8_t active_addr_rw;
	uint8_t rsvd3[3];
#endif

#ifdef I2C_NL_DEBUG_ISR
	volatile uint32_t dbg_isr_cnt;
#ifdef CONFIG_I2C_TARGET
	volatile uint32_t dbg_isr_tm_cnt;
	volatile uint32_t dbg_isr_tm_aat_cnt;
#endif
#endif
#ifdef I2C_NL_DEBUG_STATE
	volatile uint32_t dbg_state_idx;
	struct i2c_mec5_nl_dbg dbg_states[I2C_NL_DEBUG_STATE_ENTRIES];
#endif
};

#ifdef I2C_NL_DEBUG_ISR

#define I2C_NL_DEBUG_ISR_COUNT_UPDATE(data) data->dbg_isr_cnt++;

#ifdef CONFIG_I2C_TARGET
#define I2C_NL_DEBUG_ISR_TM_COUNT_UPDATE(data) data->dbg_isr_tm_cnt++
#define I2C_NL_DEBUG_ISR_TM_AAT_COUNT_UPDATE(data) data->dbg_isr_tm_aat_cnt++
#else
#define I2C_NL_DEBUG_ISR_TM_COUNT_UPDATE(data)
#define I2C_NL_DEBUG_ISR_TM_AAT_COUNT_UPDATE(data)
#endif

static inline void i2c_mec5_nl_dbg_isr_init(struct i2c_mec5_nl_data *data)
{
	data->dbg_isr_cnt = 0;
#ifdef CONFIG_I2C_TARGET
	data->dbg_isr_tm_cnt = 0;
	data->dbg_isr_tm_aat_cnt = 0;
#endif
}
#define I2C_NL_DEBUG_ISR_INIT(d) i2c_mec5_nl_dbg_isr_init(d)
#else
#define I2C_NL_DEBUG_ISR_COUNT_UPDATE(data)
#define I2C_NL_DEBUG_ISR_TM_COUNT_UPDATE(data)
#define I2C_NL_DEBUG_ISR_TM_AAT_COUNT_UPDATE(data)
#define I2C_NL_DEBUG_ISR_INIT(d)
#endif

#ifdef I2C_NL_DEBUG_STATE
static void i2c_mec5_nl_dbg_state_init(struct i2c_mec5_nl_data *data)
{
	data->dbg_state_idx = 0u;
	memset((void *)data->dbg_states, 0, sizeof(data->dbg_states));
}

static void i2c_mec5_nl_dbg_state_update(struct i2c_mec5_nl_data *data, uint8_t state)
{
	const struct i2c_mec5_nl_config *devcfg = data->devcfg;
	struct mec_i2c_smb_regs *regs = devcfg->regs;

	uint32_t idx = data->dbg_state_idx;

	if (data->dbg_state_idx < I2C_NL_DEBUG_STATE_ENTRIES) {
		struct i2c_mec5_nl_dbg *d = &data->dbg_states[idx];

		d->state = state;
		d->i2c_status = regs->STATUS;
		d->i2c_compl = regs->COMPL;
		d->i2c_config = regs->CONFIG;
		d->i2c_cm_cmd = regs->CM_CMD;
		d->i2c_tm_cmd = regs->TM_CMD;

		data->dbg_state_idx = ++idx;
	}
}

#define I2C_NL_DEBUG_STATE_INIT(pd) i2c_mec5_nl_dbg_state_init(pd)
#define I2C_NL_DEBUG_STATE_UPDATE(pd, state) i2c_mec5_nl_dbg_state_update(pd, state)
#else
#define I2C_NL_DEBUG_STATE_INIT(pd)
#define I2C_NL_DEBUG_STATE_UPDATE(pd, state)

#endif /* I2C_NL_DEBUG_STATE */

/* NOTE: I2C controller detects Lost Arbitration during START, Rpt-START,
 * data, and ACK phases not during STOP phase.
 */

static int wait_bus_free(const struct device *dev, uint32_t nwait)
{
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t count = nwait;
	uint32_t sts = 0;

	while (count--) {
		sts = mec_hal_i2c_smb_status(hwctx, 0);
		data->i2c_status = sts;
		if (sts & BIT(MEC_I2C_STS_LL_NBB_POS)) {
			break; /* bus is free */
		}
		k_busy_wait(WAIT_INTERVAL_US);
	}

	/* check for bus error, lost arbitration or external stop */
	if ((sts & 0xffu) == (BIT(MEC_I2C_STS_LL_NBB_POS) | BIT(MEC_I2C_STS_LL_NIPEND_POS))) {
		return 0;
	}

	if (sts & BIT(MEC_I2C_STS_LL_BER_POS)) {
		return I2C_NL_ERR_BUS;
	}

	if (sts & BIT(MEC_I2C_STS_LL_LRB_AD0_POS)) {
		return I2C_NL_ERR_LOST_ARB;
	}

	return I2C_NL_ERR_TIMEOUT;
}

/*
 * returns state of I2C SCL and SDA lines.
 * b[0] = SCL, b[1] = SDA
 * Call soc specific routine to read GPIO pad input.
 * Why? We can get the pins from our PINCTRL info but
 * we do not know which pin is I2C clock and which pin
 * is I2C data. There's no ordering in PINCTRL DT unless
 * we impose an order.
 */
static int check_lines(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	gpio_port_value_t sda = 0, scl = 0;

	gpio_port_get_raw(devcfg->sda_gpio.port, &sda);
	scl = sda;
	if (devcfg->sda_gpio.port != devcfg->scl_gpio.port) {
		gpio_port_get_raw(devcfg->scl_gpio.port, &scl);
	}

	if ((sda & BIT(devcfg->sda_gpio.pin)) && (scl & BIT(devcfg->scl_gpio.pin))) {
		return 0;
	}

	return -EIO;
}

static int bb_recover(const struct device *dev)
{
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	int ret = 0;
	uint32_t cnt = I2C_NL_RECOVER_SCL_LOW_RETRIES;
	uint8_t pinmsk = BIT(MEC_I2C_BB_SCL_POS) | BIT(MEC_I2C_BB_SDA_POS);
	uint8_t pins = 0u;

	/* Switch I2C pint to controller's bit-bang drivers and tri-state */
	mec_hal_i2c_smb_bbctrl(hwctx, 1u, pinmsk);
	pins = mec_hal_i2c_smb_bbctrl_pin_states(hwctx);

	/* If SCL is low continue sampling hoping it will go high on its own */
	while (!(pins & BIT(MEC_I2C_BB_SCL_POS))) {
		if (cnt) {
			cnt--;
		} else {
			break;
		}
		k_busy_wait(I2C_NL_RECOVER_SCL_DELAY_US);
		pins = mec_hal_i2c_smb_bbctrl_pin_states(hwctx);
	}

	pins = mec_hal_i2c_smb_bbctrl_pin_states(hwctx);
	if (!(pins & BIT(MEC_I2C_BB_SCL_POS))) {
		ret = -EBUSY;
		goto disable_bb_exit;
	}

	/* SCL is high, check SDA */
	if (pins & BIT(MEC_I2C_BB_SDA_POS)) {
		ret = 0; /* both high */
		goto disable_bb_exit;
	}

	/* SCL is high and SDA is low. Loop generating 9 clocks until
	 * we observe SDA high or loop terminates
	 */
	ret = -EBUSY;
	for (int i = 0; i < I2C_NL_RECOVER_SDA_LOW_RETRIES; i++) {
		mec_hal_i2c_smb_bbctrl(hwctx, 1u, pinmsk);

		/* 9 clocks */
		for (int j = 0; j < 9; j++) {
			pinmsk = BIT(MEC_I2C_BB_SDA_POS);
			mec_hal_i2c_smb_bbctrl(hwctx, 1u, pinmsk);
			k_busy_wait(I2C_NL_RECOVER_BB_DELAY_US);
			pinmsk |= BIT(MEC_I2C_BB_SCL_POS);
			mec_hal_i2c_smb_bbctrl(hwctx, 1u, pinmsk);
			k_busy_wait(I2C_NL_RECOVER_BB_DELAY_US);
		}

		pins = mec_hal_i2c_smb_bbctrl_pin_states(hwctx);
		if (pins == 0x3u) { /* Both high? */
			ret = 0;
			goto disable_bb_exit;
		}

		/* generate I2C STOP */
		pinmsk = BIT(MEC_I2C_BB_SDA_POS);
		mec_hal_i2c_smb_bbctrl(hwctx, 1u, pinmsk);
		k_busy_wait(I2C_NL_RECOVER_BB_DELAY_US);
		pinmsk |= BIT(MEC_I2C_BB_SCL_POS);
		mec_hal_i2c_smb_bbctrl(hwctx, 1u, pinmsk);
		k_busy_wait(I2C_NL_RECOVER_BB_DELAY_US);

		pins = mec_hal_i2c_smb_bbctrl_pin_states(hwctx);
		if (pins == 0x3u) { /* Both high? */
			ret = 0;
			goto disable_bb_exit;
		}
	}

disable_bb_exit:
	mec_hal_i2c_smb_bbctrl(hwctx, 0u, 0u);

	return ret;
}

static int i2c_mec5_nl_reset_config(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct mec_i2c_smb_cfg mcfg;
	int ret;

	hwctx->base = devcfg->regs;
	hwctx->i2c_ctrl_cached = 0;

	data->state = I2C_NL_STATE_CLOSED;
	data->i2c_status = 0;

	mcfg.std_freq = data->speed_id;
	mcfg.cfg_flags = 0u;
	mcfg.port = devcfg->port_sel;
	mcfg.target_addr1 = 0;
	mcfg.target_addr2 = 0;

	ret = mec_hal_i2c_smb_init(hwctx, &mcfg, NULL);
	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	/* wait for NBB=1, BER, LAB, or timeout */
	ret = wait_bus_free(dev, WAIT_COUNT);

	return ret;
}

static int recover_bus(const struct device *dev)
{
	int ret;

	LOG_ERR("I2C attempt bus recovery\n");

	/* Try controller reset first */
	ret = i2c_mec5_nl_reset_config(dev);
	if (ret == 0) {
		ret = check_lines(dev);
	}

	if (!ret) {
		return 0;
	}

	ret = bb_recover(dev);
	if (ret == 0) {
		ret = wait_bus_free(dev, WAIT_COUNT);
	}

	return ret;
}

/* i2c_configure API */
static int i2c_mec5_nl_configure(const struct device *dev, uint32_t dev_config_raw)
{
	struct i2c_mec5_nl_data *data = dev->data;

	if (!(dev_config_raw & I2C_MODE_CONTROLLER)) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config_raw)) {
	case I2C_SPEED_STANDARD:
		data->speed_id = MEC_I2C_STD_FREQ_100K;
		break;
	case I2C_SPEED_FAST:
		data->speed_id = MEC_I2C_STD_FREQ_400K;
		break;
	case I2C_SPEED_FAST_PLUS:
		data->speed_id = MEC_I2C_STD_FREQ_1M;
		break;
	default:
		return -EINVAL;
	}

	int ret = i2c_mec5_nl_reset_config(dev);

	return ret;
}

static void pr_data_struct_info(const struct device *dev)
{
	const struct i2c_mec5_nl_config *devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;

	LOG_INF("I2C-NL driver devcfg @ %p size = %u", devcfg, sizeof(struct i2c_mec5_nl_config));
	LOG_INF("I2C-NL driver data @ %p size = %u", data, sizeof(struct i2c_mec5_nl_data));

	LOG_INF("I2C-NL data offset .xfrbuf_max = 0x%0x",
		offsetof(struct i2c_mec5_nl_data, xfrbuf_max));
	LOG_INF("I2C-NL data offset .active_addr_rw = 0x%0x",
		offsetof(struct i2c_mec5_nl_data, active_addr_rw));

	LOG_INF("I2C-NL data xfrbuf = 0x%0x", (uint32_t)data->xfrbuf);
	LOG_INF("I2C-NL data tm_tx_buf = 0x%0x", (uint32_t)data->tm_tx_buf);
	LOG_INF("I2C-NL data tm_rx_buf = 0x%0x", (uint32_t)data->tm_rx_buf);
}

/* i2c_get_config API */
static int i2c_mec5_nl_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_mec5_nl_data *data = dev->data;
	uint32_t dcfg = 0u;

	pr_data_struct_info(dev);

	if (!dev_config) {
		return -EINVAL;
	}

	switch (data->speed_id) {
	case MEC_I2C_STD_FREQ_1M:
		dcfg = I2C_SPEED_SET(I2C_SPEED_FAST_PLUS);
		break;
	case MEC_I2C_STD_FREQ_400K:
		dcfg = I2C_SPEED_SET(I2C_SPEED_FAST);
		break;
	default:
		dcfg = I2C_SPEED_SET(I2C_SPEED_STANDARD);
		break;
	}

	dcfg |= I2C_MODE_CONTROLLER;
	*dev_config = dcfg;

	return 0;
}

/* MEC5 I2C controller support 7-bit addressing only.
 * Format 7-bit address for as it appears on the bus as an 8-bit
 * value with R/W bit at bit[0], 0(write), 1(read).
 */
static inline uint8_t fmt_addr(uint16_t addr, uint8_t read)
{
	uint8_t fmt_addr = (uint8_t)((addr & 0x7fu) << 1);

	if (read) {
		fmt_addr |= BIT(0);
	}

	return fmt_addr;
}

/* Issue I2C STOP only if controller owns the bus otherwise
 * clear driver state and re-arm controller for next
 * controller-mode or target-mode transaction.
 * Reason for ugly code sequence:
 * Brain-dead I2C controller has write-only control register
 * containing enable interrupt bit. This is the enable for ACK/NACK,
 * bus error and lost arbitration.
 */
static int do_stop(const struct device *dev)
{
	const struct i2c_mec5_nl_config *devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t ev, evw;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x20);

	/* Can we trust GIRQ status has been cleared before we re-enable? */
	if (mec_hal_i2c_smb_is_bus_owned(hwctx)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x21);
		data->mdone = 0;
		mec_hal_i2c_smb_stop_gen(hwctx);
		mec_hal_i2c_smb_girq_status_clr(hwctx);
		mec_hal_i2c_smb_idle_intr_enable(hwctx, 1);
		mec_hal_i2c_smb_girq_ctrl(hwctx, MEC_I2C_SMB_GIRQ_EN);

		evw = (BIT(I2C_NL_KEV_IDLE_POS) | BIT(I2C_NL_KEV_BERR_POS)
			| BIT(I2C_NL_KEV_LAB_ERR_POS));
		ev = k_event_wait(&data->events, evw, false, K_MSEC(100));
		if (!ev) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x22);
			LOG_ERR("Gen STOP timeout");
		}

		dma_stop(devcfg->dma_dev_cm, devcfg->dma_cm_chan);
		mec_hal_i2c_nl_cmd_clear(hwctx, MEC_I2C_NL_CM_SEL);
#ifdef CONFIG_I2C_TARGET
		dma_stop(devcfg->dma_dev_tm, devcfg->dma_tm_chan);
		mec_hal_i2c_nl_cmd_clear(hwctx, MEC_I2C_NL_TM_SEL);
#endif
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x23);
	}

	data->state = I2C_NL_STATE_CLOSED;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x24);

	return 0;
}

/* DMA channel callbacks
 * dev = pointer central DMA device structure
 * user_data = pointer to I2C-NL driver data structure
 * chan = DMA channel
 * status:
 *	0 = completion of all blocks in the linked list
 *	1 = completion of each individual block
 *	< 0 = error
 *
 * dma_config API allows one to choose when the DMA callback is invoked
 * by the DMA ISR:
 * DMA Error
 * One each block completion or all blocks completed.
 * We configured for Error or completion of all blocks.
 *
 * Timing of DMA callback is based on transfer direction.
 * Transmit to target I2C (DMA memory to device). DMA will finish before I2C.
 * Receive from target I2C (DMA device to memory). I2C will finish before DMA.
 *
 * If a DMA error occurs we set a kevent DMA error flag. Driver waits for
 * I2C controller IDLE, I2C error, and DMA error event flags.
 *
 */
static void i2c_mec5_nl_cm_dma_cb(const struct device *dev, void *user_data,
				  uint32_t chan, int status)
{
	struct i2c_mec5_nl_data *data = (struct i2c_mec5_nl_data *)user_data;
	const struct i2c_mec5_nl_config *devcfg = data->devcfg;
	struct mec_i2c_smb_regs *regs = devcfg->regs;
	struct i2c_msg *m = NULL;
	struct dma_status dma_sts = { 0 };
	uint32_t src = 0, dest = 0;
	size_t dmalen = 0;
	int ret = dma_get_status(dev, chan, &dma_sts);

	if (status < 0) {
		LOG_ERR("I2C-NL CM DMA CB error (%d): chan %u drv data at (%p) ",status, chan, data);
		dma_stop(devcfg->dma_dev_cm, devcfg->dma_cm_chan);
		k_event_post(&data->events, BIT(I2C_NL_KEV_DMA_CM_ERR_POS));
		return;
	}

	if (ret) {
		LOG_ERR("I2C-NL CM DMA CB: invalid device or channel");
		return;
	}

	if (dma_sts.dir == MEMORY_TO_PERIPHERAL) {
		dest = (uint32_t)&regs->CM_TXB;
		if (data->rembuf && data->remlen) {
			src = (uint32_t)data->rembuf;
			dmalen = data->remlen;
			data->mgrp.wr_idx++;
		} else if (data->mgrp.wr_idx < data->mgrp.wr_msg_cnt) {
			m = &data->msgs[data->mgrp.wr_idx++];
			src = (uint32_t)m->buf;
			dmalen = (uint32_t)m->len;
		}

		if (dmalen) {
			dma_reload(devcfg->dma_dev_cm, devcfg->dma_cm_chan, src, dest, dmalen);
			dma_start(devcfg->dma_dev_cm, devcfg->dma_cm_chan);
			return;
		}
	}

	k_event_post(&data->events, BIT(I2C_NL_KEV_DMA_CM_DONE_POS));
}

#ifdef CONFIG_I2C_TARGET
/* TODO */
static void i2c_mec5_nl_tm_dma_cb(const struct device *dev, void *user_data,
				  uint32_t chan, int status)
{
	struct i2c_mec5_nl_data *data = (struct i2c_mec5_nl_data *)user_data;
	const struct i2c_mec5_nl_config *devcfg = data->devcfg;
	uint32_t kev = BIT(I2C_NL_KEV_DMA_TM_DONE_POS);

	if (status < 0) {
		LOG_ERR("I2C-NL DMA CB2 error (%d): drv data at (%p)", status, data);
		dma_stop(devcfg->dma_dev_tm, devcfg->dma_tm_chan);
		kev = BIT(I2C_NL_KEV_DMA_TM_ERR_POS);
	}

	k_event_post(&data->events, kev);
}
#endif

static void mec5_nl_hw_prep(const struct device *dev)
{
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t mask = (BIT(MEC_I2C_IEN_BYTE_MODE_POS) | BIT(MEC_I2C_IEN_IDLE_POS) |
			 BIT(MEC_I2C_NL_IEN_CM_DONE_POS) | BIT(MEC_I2C_NL_IEN_TM_DONE_POS) |
			 BIT(MEC_I2C_NL_IEN_AAT_POS));

	mec_hal_i2c_smb_girq_ctrl(hwctx, MEC_I2C_SMB_GIRQ_DIS);
	mec_hal_i2c_smb_intr_ctrl(hwctx, mask, 0);
	mec_hal_i2c_smb_status(hwctx, 1);
	mec_hal_i2c_smb_wake_status_clr(hwctx);
	mec_hal_i2c_smb_girq_ctrl(hwctx, MEC_I2C_SMB_GIRQ_EN | MEC_I2C_SMB_GIRQ_CLR_STS);
}

/* I2C Read or Write or Combined Write-Read
 * S wrAddr [A] wrData1 [A] wrData2 [A] ... wrDataN [A]
 * Sr rdAddr [A] [rdData1] A [rdData2] A ... [rdDataM] N S
 * Target may NACK wrAddr, any wrData, or rdAddr
 * Controller issues STOP on any NACK or after ACK of rdData
 * I2C-NL transmits wrAddr plus message data.
 * xfrbuf[0] = wrAddr
 * tx_len = 1
 * tx_len_rem = 0;
 * n_tx_dma_blocks = 1
 * if wrmsg->len <= XFRBUF_DATA_LEN_MAX
 *    memcpy(&xfrbuf[1], wrmsg->buf, wrmsg->len)
 *    tx_len += wrmsg->len
 * else
 *    memcpy(&xfrbuf[1], wrmsg->buf, XFRBUF_DATA_LEN_MAX)
 *    tx_len += XFRBUF_DATA_LEN_MAX;
 *    tx_len_rem = wrmsg->len - XFRBUF_DATA_LEN_MAX
 *    n_tx_dma_blocks++
 * endif
 * DMA block 1: (xfrbuf, tx_len)
 * if tx_len_rem
 *     DMA block 2: (wrmsg->buf[XFRBUF_DATA_LEN_MAX], tx_len_rem)
 * TX DMA config for n_tx_dma_blocks
 * RX DMA config for 1 block
 * DMA block 3: (rdmsg->buf, rdmsg->len)
 *
 * Caller has checked the messages for NULL pointer and len <= 0xfffc
 */
#define MEC5_I2C_NL_XFLAG_ASYNC BIT(0)
#define MEC5_I2C_NL_XFLAG_START BIT(1)

#define I2C_NL_ERRORS \
	(BIT(I2C_NL_KEV_DMA_CM_ERR_POS) | BIT(I2C_NL_KEV_DMA_TM_ERR_POS) \
	 | BIT(I2C_NL_KEV_BERR_POS) | BIT(I2C_NL_KEV_LAB_ERR_POS) \
	 | BIT(I2C_NL_KEV_CM_NAK_POS))

static int msg_check(struct i2c_msg_group *g, struct i2c_msg *m, uint8_t msg_idx)
{
	LOG_INF("I2C-NL Msg[%u] check", msg_idx);

	if (!m->buf || !m->len) {
		LOG_ERR("NULL buf or len");
		return -EINVAL;
	}

	if (m->flags & I2C_MSG_ADDR_10_BITS) {
		LOG_ERR("I2C 10-bit addr not supported by HW");
		return -EINVAL;
	}

	if (m->len > I2C_NL_MAX_XFR_LEN) {
		LOG_ERR("Length exceeds HW capabilities");
		return -EMSGSIZE;
	}

	if (!(m->flags & I2C_MSG_READ)) {
		if (m->len > (I2C_NL_MAX_XFR_LEN - g->wr_len)) {
			LOG_ERR("Accumulated write length exceeds HW capabilities");
			return -EMSGSIZE;
		}
	}

	return 0;
}

/* Update the message group with new message.
 * Expects msg_check called on m to insure length is in HW limits.
 */
static int msg_grp_update(struct i2c_msg_group *g, struct i2c_msg *m, uint8_t msg_idx)
{
	LOG_INF("I2C-NL Msg Group Update: msg[%u]", msg_idx);

	if (!g || !m) { /* can be removed */
		LOG_ERR("Bad group or msg pointer!");
		return -EINVAL;
	}

	if (m->flags & I2C_MSG_READ) {
		if (!g->rd_msg_cnt) {
			g->rd_len = m->len;
			g->rd_msg_cnt++;
			g->rd_idx = msg_idx;
		} else {
			LOG_ERR("One read msg per group!");
			return -EIO;
		}
	} else { /* write message */
		g->wr_len += (uint16_t)(m->len & 0xffffu);
		if (!g->wr_msg_cnt) {
			g->wr_idx = msg_idx;
		}
		g->wr_msg_cnt++;
	}

	return 0;
}

static int i2c_mec5_nl_cm_dma_start(const struct device *dev, uint8_t *buf, size_t blen,
				    uint8_t dir, bool start)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;
	struct dma_config *dcfg = NULL;
	struct dma_block_config *dbcfg = NULL;
	int ret = 0;

	if (dir == I2C_NL_DIR_WR) {
		dcfg = &data->dma_cfg_cm_tx;
		dbcfg = &data->dma_blk_cm_tx;
		dbcfg->source_address = (uint32_t)buf;
	} else {
		dcfg = &data->dma_cfg_cm_rx;
		dbcfg = &data->dma_blk_cm_rx;
		dbcfg->dest_address = (uint32_t)buf;
	}

	dbcfg->block_size = blen;

	ret = dma_config(devcfg->dma_dev_cm, devcfg->dma_cm_chan, dcfg);
	if (ret) {
		return ret;
	}

	if (start) {
		ret = dma_start(devcfg->dma_dev_cm, devcfg->dma_cm_chan);
	}

	return ret;
}

/* Configure I2C-NL and one DMA channel for message group.
 * Start I2C-NL transaction with I2C-NL and DMA interrupts enabled.
 * I2C Write protocol: TX DMA only
 * I2C Read protocol: TX DMA address, RX DMA data.
 * I2C Write-Read protocol:
 *   TX DMA target write address, optional data, Rpt-START target read address, RX DMA data
 * This routine configures TX DMA, I2C-NL and starts both.
 * TX DMA callback handles more TX data.
 * I2C-NL ISR handles reconfiguring DMA channel for RX.
 *  This requires calls to dma_config & dma_start.
 *  DMA driver reload API does not allow direction change.
 */
static int msg_grp_start_xfr(const struct device *dev)
{
/*	const struct i2c_mec5_nl_config *const devcfg = dev->config; */
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct i2c_msg_group *g = &data->mgrp;
	uint8_t *xbuf = data->xfrbuf;
	uint8_t *xbuf_end = data->xfrbuf;
	uint32_t i2c_nl_wr_len = g->wr_len + 1;
	uint32_t dma_mem2dev_len = 1;
	uint32_t hal_flags = (MEC_I2C_NL_FLAG_START | MEC_I2C_NL_FLAG_STOP
			      | MEC_I2C_NL_FLAG_CM_DONE_IEN);
	uint32_t i2c_nl_rd_len = 0;
	struct i2c_msg *m = NULL;
	int ret = 0;
	uint16_t max_idx = 0;

	mec5_nl_hw_prep(dev);
	k_event_clear(&data->events, UINT32_MAX);

	data->i2c_status = 0;
	data->rembuf = NULL;
	data->remlen = 0;
	data->mdone = 0;

	xbuf[0] = data->cm_target_wr_i2c_addr;
	xbuf_end = &xbuf[1];

	max_idx = g->wr_idx + g->wr_msg_cnt;
	if (max_idx > 255u) {
		max_idx = 255u;
	}
	while (g->wr_idx < max_idx) {
		m = &data->msgs[g->wr_idx];
		if (m->len <= data->xfrbuf_max) {
			memcpy(xbuf_end, m->buf, m->len);
			xbuf_end += m->len;
			dma_mem2dev_len += m->len;
		} else {
			memcpy(xbuf_end, m->buf, data->xfrbuf_max);
			xbuf_end += m->len;
			dma_mem2dev_len += data->xfrbuf_max;
			data->rembuf = &m->buf[data->xfrbuf_max];
			data->remlen = m->len - data->xfrbuf_max;
			break;
		}
		g->wr_idx++;
	}

	if (g->rd_msg_cnt) {
		m = &data->msgs[g->rd_idx];
		i2c_nl_rd_len = m->len;
		if (g->wr_msg_cnt) { /* store read-address at end of transmit data */
			*xbuf_end = xbuf[0] | BIT(0);
			i2c_nl_wr_len++;
			dma_mem2dev_len++;
			hal_flags |= MEC_I2C_NL_FLAG_RPT_START;
		} else { /* I2C read protocol: START address is target read */
			xbuf[0] |= BIT(0);
		}
	}

	ret = i2c_mec5_nl_cm_dma_start(dev, data->xfrbuf, dma_mem2dev_len, I2C_NL_DIR_WR, true);
	if (ret) {
		LOG_ERR("CM TX DMA start");
		return ret;
	}

	ret = mec_hal_i2c_nl_cm_cfg_start(hwctx, i2c_nl_wr_len, i2c_nl_rd_len, hal_flags);
	if (ret) {
		LOG_ERR("I2C-NL start err (%d)", ret);
		ret = -EIO;
	}

	return ret;
}

/* Compute timeout based on message group write and read lengths
 * I2C-NL driver can handle messages of up to 0xFFFC bytes.
 * @ 100 KHz 6000 ms, @ 400 KHz 1500 ms, 1 MHz 600 ms
 * Over estimate 10 clocks / byte and add 10 ms to total.
 * if computed timeout < SMBus timeout of 35 ms then use 35 ms.
 */
static void msg_grp_compute_timeout(const struct device *dev)
{
	struct i2c_mec5_nl_data *data = dev->data;
	struct i2c_msg_group *g = &data->mgrp;
	uint32_t i2c_clks = g->wr_len + g->rd_len; /* lengths are 16-bit */
	uint32_t i2c_freq = KHZ(100);
	uint32_t tmout = 0;

	if (data->speed_id == MEC_I2C_STD_FREQ_1M) {
		i2c_freq = MHZ(1);
	} else if (data->speed_id == MEC_I2C_STD_FREQ_400K) {
		i2c_freq = KHZ(400);
	}

	i2c_clks *= 10000u;
	tmout = i2c_clks / i2c_freq;
	tmout += 10u; /* paranoid: add 10 ms */
	if (tmout < I2C_NL_SMBUS_TMOUT_MAX_MS) {
		tmout = I2C_NL_SMBUS_TMOUT_MAX_MS;
	}

	data->xfr_tmout = tmout;
}

static int i2c_mec5_nl_wait_events(const struct device *dev, uint32_t evmask)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;
	int ret = 0;
	uint32_t ev = 0;

	ev = k_event_wait(&data->events, evmask, false, K_MSEC(data->xfr_tmout));

	if (!ev) {
		LOG_ERR("I2C-NL CM event timeout");
		do_stop(dev);
		dma_stop(devcfg->dma_dev_cm, devcfg->dma_cm_chan);
		dma_stop(devcfg->dma_dev_tm, devcfg->dma_tm_chan);
		ret = -ETIMEDOUT;
	} else if (ev & I2C_NL_ERRORS) {
		LOG_ERR("CM event errors = 0x%08x", ev);
		ret = -EIO;
	}

	return ret;
}

/* Process I2C messages into a I2C-NL HW message group and start transfer.
 * Concantenate as much of the write messages into driver transfer buffer
 * before starting.
 */
static int process_i2c_msgs(const struct device *dev)
{
	struct i2c_mec5_nl_data *data = dev->data;
	struct i2c_msg_group *g = &data->mgrp;
	int ret = 0;
	uint8_t n = data->msgidx;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x10);

	while (n < data->nmsgs) {
		struct i2c_msg *m = &data->msgs[n];

		ret = msg_check(g, m, n);
		if (ret) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x11);
			LOG_ERR("check msg[%u] error (%d)\n", n, ret);
			return ret;
		}

		ret = msg_grp_update(g, m, n);
		if (ret) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x12);
			LOG_ERR("update error: msg[%u] = (%p, %u, 0x%0x)",
				n , m->buf, m->len, m->flags);
			return ret;
		}

		if (m->flags & (I2C_MSG_READ | I2C_MSG_STOP)) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x13);
			msg_grp_compute_timeout(dev);
			ret = msg_grp_start_xfr(dev);
			if (ret) {
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x14);
				LOG_ERR("start xfr error (%d)", ret);
				return ret;
			}
			if (data->xfrflags & I2C_NL_XFR_FLAG_ASYNC) {
				return 0;
			}

			ret = i2c_mec5_nl_wait_events(dev, I2C_NL_WAIT_EVENTS_MSK);
			if (ret) {
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x15);
				LOG_ERR("wait events returned (%d)", ret);
				break;
			}

			memset(g, 0, sizeof(struct i2c_msg_group));
		}
		n++;
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x1f);

	return ret;
}

/* Parse I2C messages into transfer group
 * Start network layer + DMA transfer on the group.
 * Repeat until all messages processed.
 */
static int i2c_mec5_nl_transfer_cmn(const struct device *dev, struct i2c_msg *msgs,
				    uint8_t num_msgs, uint16_t addr, bool async,
				    i2c_callback_t cb, void *userdata)
{
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	int ret = 0;

	if (!msgs || !num_msgs) {
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER); /* decrements count */

#ifdef CONFIG_I2C_TARGET
	/* Driver does not support multi-master mode */
	if (data->targets_registered) {
		k_sem_give(&data->lock);
		return -EBUSY;
	}
#endif

	k_event_clear(&data->events, UINT32_MAX);

	I2C_NL_DEBUG_STATE_INIT(data);
	I2C_NL_DEBUG_ISR_INIT(data);

	ret = check_lines(dev);
	data->i2c_status = mec_hal_i2c_smb_status(hwctx, 1);
	if (ret || (data->i2c_status & BIT(MEC_I2C_STS_LL_BER_POS))) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x50);
		ret = recover_bus(dev);
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x1);

	if (ret) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x2);
		data->state = I2C_NL_STATE_CLOSED;
		goto mec5_unlock;
	}

	memset(&data->mgrp, 0, sizeof(struct i2c_msg_group));
	data->xfr_tmout = 0;
	data->msgs = msgs;
	data->nmsgs = num_msgs;
	data->msgidx = 0;
	data->xfrflags = (async) ? I2C_NL_XFR_FLAG_ASYNC : 0;

	data->state = I2C_NL_STATE_OPEN;
	data->cm_target_wr_i2c_addr = fmt_addr(addr, I2C_NL_FMT_ADDR_WR);

	ret = process_i2c_msgs(dev);

	if (ret) { /* if error issue STOP if bus is still owned by controller */
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x7);
		do_stop(dev);
		goto mec5_unlock;
	}

	if (async) {
		return 0;
	}

mec5_unlock:
	I2C_NL_DEBUG_STATE_UPDATE(data, 0x8);
	if (!mec_hal_i2c_smb_is_bus_owned(hwctx)) {
		data->state = I2C_NL_STATE_CLOSED;
	}
	k_sem_give(&data->lock); /* increment count up to limit */

	return ret;
}

static int i2c_mec5_nl_transfer(const struct device *dev, struct i2c_msg *msgs,
				uint8_t num_msgs, uint16_t addr)
{
	return i2c_mec5_nl_transfer_cmn(dev, msgs, num_msgs, addr, false, NULL, NULL);
}

#ifdef CONFIG_I2C_TARGET

static int i2c_mec5_nl_tm_dma_start(const struct device *dev, uint8_t *buf, size_t blen,
				    uint8_t dir, bool start)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;
	struct dma_config *dcfg = NULL;
	struct dma_block_config *dbcfg = NULL;
	int ret = 0;

	if (dir == I2C_NL_DIR_WR) {
		dcfg = &data->dma_cfg_tm_tx;
		dbcfg = &data->dma_blk_tm_tx;
		dbcfg->source_address = (uint32_t)buf;
	} else {
		dcfg = &data->dma_cfg_tm_rx;
		dbcfg = &data->dma_blk_tm_rx;
		dbcfg->dest_address = (uint32_t)buf;
	}

	dbcfg->block_size = blen;

	ret = dma_config(devcfg->dma_dev_tm, devcfg->dma_tm_chan, dcfg);
	if (ret) {
		return ret;
	}

	if (start) {
		ret = dma_start(devcfg->dma_dev_tm, devcfg->dma_tm_chan);
	}

	return ret;
}

/* !!!!
 * TODO look at STM32 I2C V2 target implementation. Has two target addresses also.
 * !!!!
 */
/* Controller supports two 7-bit I2C target addresses.
 * I2C-NL HW FSM requires target mode read and write counts in the target mode
 * register to be non-zero and the DMA channel used for target mode to be armed for
 * device to memory transfer.
 * We arm target mode DMA channel for peripheral to memory transfer since the external Controller
 * always writes the target address after START.
 * NOTE: We must configure I2C-NL write and read counts with non-zero values to ensure the HW FSM
 * will handle both I2C_WRITE and I2C combined write-read.
 */
static int i2c_mec5_nl_target_arm(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	/* uint32_t tm_flags = MEC_I2C_NL_TM_FLAG_DONE_IEN | MEC_I2C_NL_TM_FLAG_STOP_IEN; */
	uint32_t tm_flags = MEC_I2C_NL_TM_FLAG_DONE_IEN | MEC_I2C_NL_TM_FLAG_AAT_IEN;
	/* uint32_t tm_flags = MEC_I2C_NL_TM_FLAG_DONE_IEN; */
	uint32_t clrmsk = BIT(MEC_I2C_IEN_IDLE_POS) | BIT(MEC_I2C_NL_IEN_CM_DONE_POS)
			  | BIT(MEC_I2C_NL_IEN_TM_DONE_POS) | BIT(MEC_I2C_NL_IEN_AAT_POS);
	uint16_t nrx = data->tm_rx_buf_max + I2C_NL_DRV_XFRBUF_PAD_SIZE;
	uint16_t ntx = I2C_NL_MAX_XFR_LEN;
	int ret = 0;

	dma_stop(devcfg->dma_dev_tm, devcfg->dma_tm_chan);
	mec_hal_i2c_smb_intr_ctrl(hwctx, clrmsk, 0);
	mec_hal_i2c_nl_cmd_clear(hwctx, MEC_I2C_NL_TM_SEL);
	mec_hal_i2c_nl_flush_buffers(hwctx->base);

	ret = mec_hal_i2c_nl_tm_config(hwctx, ntx, nrx, tm_flags);
	if (ret != MEC_RET_OK) {
		ret = -EIO;
	}

	ret = i2c_mec5_nl_tm_dma_start(dev, data->tm_rx_buf, nrx, I2C_NL_DIR_RD, true);

	return ret;
}

static int i2c_mec5_nl_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	bool config_tm = false;
	int ret;

	if (!cfg || (cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS)) {
		return -EINVAL;
	}

	ret = k_sem_take(&data->lock, K_NO_WAIT);
	if (ret) {
		return ret;
	}

	data->active_target_cfg = NULL;

	for (uint8_t n = 0; n < I2C_NL_MAX_TARGET_ADDRS; n++) {
		if (data->target_cfgs[n] == NULL) {
			data->target_cfgs[n] = cfg;
			ret = mec_hal_i2c_smb_set_target_addr(hwctx, n, cfg->address);
			if (ret == MEC_RET_OK) {
				config_tm = true;
				ret = 0;
			} else {
				ret = -EIO;
			}
			break;
		}
	}

	if (config_tm) {
		I2C_NL_DEBUG_STATE_INIT(data);
		I2C_NL_DEBUG_ISR_INIT(data);
		data->i2c_status = 0;
		data->rembuf = NULL;
		data->remlen = 0;
		data->mdone = 0;
		/* TODO:
		 * problem setting these two data items here.
		 * If app registers a second target while first target
		 * is active.
		 */
		data->active_target_cfg = 0;
		data->active_addr_rw = 0;

		ret = i2c_mec5_nl_target_arm(dev);
		if (!ret) {
			data->targets_registered = true;
		}
	}

	k_sem_give(&data->lock);

	return ret;
}

static int i2c_mec5_nl_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	int dis_count = 0;
	int ret;

	if (!cfg || (cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS)) {
		return -EINVAL;
	}

	ret = k_sem_take(&data->lock, K_NO_WAIT);
	if (ret) {
		return ret;
	}

	for (uint8_t n = 0; n < I2C_NL_MAX_TARGET_ADDRS; n++) {
		if (data->target_cfgs[n] != NULL) {
			if (data->target_cfgs[n]->address == cfg->address) {
				dis_count++;
				data->target_cfgs[n] = NULL;
				ret = mec_hal_i2c_smb_set_target_addr(hwctx, n, 0);
				if (ret) {
					ret = -EIO;
				}
				break;
			}
		} else {
			dis_count++;
		}
	}

	if (dis_count >= I2C_NL_MAX_TARGET_ADDRS) {
		ret = mec_hal_i2c_smb_intr_ctrl(hwctx, (BIT(MEC_I2C_NL_IEN_TM_DONE_POS)
							| BIT(MEC_I2C_NL_IEN_AAT_POS)), 0);
		dma_stop(devcfg->dma_dev_tm, devcfg->dma_tm_chan);
		data->targets_registered = false;
		data->active_target_cfg = 0;
		data->active_addr_rw = 0;
	}

	k_sem_give(&data->lock);

	return ret;
}
#else
static int i2c_mec5_nl_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
	return -ENOTSUP;
}

static int i2c_mec5_nl_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
	return -ENOTSUP;
}
#endif

#ifdef CONFIG_I2C_TARGET
/* TODO Target Mode ISR modifications
 * STOP or STOP->IDLE event callback to
 * typedef int (*i2c_target_stop_cb_t)(struct i2c_target_config *config);
 *
 * Only support target mode if CONFIG_I2C_TARGET_BUFFER_MODE is enabled
 * When external Controller issues read to us we make callback:
 * typedef int (*i2c_target_buf_read_requested_cb_t)(
 *		struct i2c_target_config *config, uint8_t **ptr, uint32_t *len)
 * to get a buffer and length of data to transmit to host.
 *
 * When external Controller issues a write to us we make callback:
 * typedef void (*i2c_target_buf_write_received_cb_t)(
 *		struct i2c_target_config *config, uint8_t *ptr, uint32_t len);
 * We pass address of the driver buff containing data written to us by the external
 * controller and length of the data.
 */
#endif /* CONFIG_I2C_TARGET */

/* Controller Mode ISR  */

#if 1
static uint32_t	check_errors(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct mec_i2c_smb_regs *regs = devcfg->regs;
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t i2c_hw_sts = data->i2c_status;
	uint32_t err_ev = 0;

	/* Bus Error? */
	if (i2c_hw_sts & BIT(MEC_I2C_STS_LL_BER_POS)) {
		data->mdone = 1;
		err_ev |= BIT(I2C_NL_KEV_BERR_POS);
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x82);
	}

	/* Lost Arbitration */
	if (i2c_hw_sts & BIT(MEC_I2C_STS_LL_LAB_POS)) {
		data->mdone = 1;
		err_ev |= BIT(I2C_NL_KEV_LAB_ERR_POS);
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x83);
	}

	/* Does a NAK to I2C-NL CM cause HW to auto-generate STOP? */
	if (i2c_hw_sts & BIT(MEC_I2C_STS_CM_TX_NACK_POS)) {
		data->mdone = 1;
		err_ev |= BIT(I2C_NL_KEV_CM_NAK_POS);
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x84);
	}

	if (err_ev) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x85);
		dma_stop(devcfg->dma_dev_cm, devcfg->dma_cm_chan);
		mec_hal_i2c_nl_cmd_clear(hwctx, MEC_I2C_NL_CM_SEL);
#ifdef CONFIG_I2C_TARGET
		dma_stop(devcfg->dma_dev_tm, devcfg->dma_tm_chan);
		mec_hal_i2c_nl_cmd_clear(hwctx, MEC_I2C_NL_TM_SEL);
#endif
		/* flush NL internal buffers */
		mec_hal_i2c_nl_flush_buffers(regs);
	}

	return err_ev;
}
#endif

static void start_cm_read_phase(const struct device *dev)
{
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct i2c_msg_group *g = &data->mgrp;
	struct i2c_msg *m = &data->msgs[g->rd_idx];

	i2c_mec5_nl_cm_dma_start(dev, m->buf, m->len, I2C_NL_DIR_RD, true);
	mec_hal_i2c_nl_cm_proceed(hwctx);
	k_event_post(&data->events, BIT(I2C_NL_KEV_W2R_POS));
}

#ifdef CONFIG_I2C_TARGET
#if 0
static void start_tm_read_phase(const struct device *dev)
{
	struct i2c_mec5_nl_data *data = dev->data;
/* TODO	struct mec_i2c_smb_ctx *hwctx = &data->ctx; */

	/* TODO */

	k_event_post(&data->events, BIT(I2C_NL_KEV_W2R_POS));
}
#endif

#if 0
static int tm_write_recv(const struct device *dev)
{
	struct i2c_mec5_nl_data *data = dev->data;

	if (tcfg && tcfg->callbacks && tcfg->callbacks->buf_write_received) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x92);
			tcfg->callbacks->buf_write_received(tcfg, data->tm_rx_buf, data_len);
		}
		/* we need to restart the RX DMA channel to handle next
		 * transaction from external Controller. If previous
		 * transaction ended with read by external Controller(mem2dev)
		 * we must switch DMA channel direction back to mem2dev.
		 */
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x93);
		i2c_mec5_nl_tm_dma_start(dev, data->tm_rx_buf, data->tm_rx_buf_max,
					 I2C_NL_DIR_RD, true);
		/* SPROCEED and SRUN were cleared in I2C.TM_CMD, set them */
		mec_hal_i2c_nl_tm_proceed(regs);
}
#endif
#endif

#ifdef CONFIG_I2C_TARGET

/* find pointer to struct i2c_target_config with matching address.
 * dev = pointer to driver device structure
 * cap_i2c_addr = I2C address captured by HW. b[7:1]=7-bit I2C address, b[0]=nW/R
 */
static struct i2c_target_config *find_i2c_target(const struct device *dev, uint16_t cap_i2c_addr)
{
	struct i2c_mec5_nl_data *data = dev->data;
	uint16_t i2c_addr = cap_i2c_addr >> 1;

	for (uint8_t n = 0; n < I2C_NL_MAX_TARGET_ADDRS; n++) {
		struct i2c_target_config *p = data->target_cfgs[n];

		if ((p != NULL) && (p->address == i2c_addr)) {
			return p;
		}
	}

	return NULL;
}

volatile int vret1;

/* ISSUES:
 * 1. I2C-NL target mode hardware receive (external Controller write) triggers
 * RX DMA channel to read one extra byte. This byte is not part of the I2C
 * transaction and its value appears to be always 0. But I2C-NL TM read count
 * is not decremented for this extra byte. It does cause the DMA channel to
 * increment its count resulting in a DMA done interrupt.
 * 2. Timing of I2C-NL interrupt and DMA channel done interrupt.
 *    Target mode external controller write to target data flow is into I2C
 *    then I2C triggers DMA to transfer the byte from I2C TM RX buffer to memory.
 *    I2C-NL interrupt usually fires first. If the DMA channel has reached termination
 *    state (mem start == mem end or other termination state), the DMA channel interrupt
 *    will fire. This creates a race condition where I2C-NL TM ISR must reprogram the
 *    DMA channel and itself in preparation for the next message from the external
 *    controller. We have observed reprogramming of the DMA channel being overwritten
 *    by DMA ISR.
 */
static void tm_handle_idle(const struct device *dev, uint8_t target_addr)
{
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct i2c_target_config *tcfg = NULL;
	uint8_t *pdata = NULL;
	uint32_t data_len = 0;

	mec_hal_i2c_smb_idle_intr_enable(hwctx, 0);
	mec_hal_i2c_smb_idle_status_clr(hwctx);

	data->mdone = 1;
	data_len = mec_hal_i2c_nl_tm_transfered(hwctx, MEC_I2C_NL_TM_DIR_RX);

	tcfg = find_i2c_target(dev, (uint16_t)target_addr);
	if (tcfg && tcfg->callbacks && tcfg->callbacks->buf_write_received) {
		/* HW writes target address and data to buffer
		 * We only pass data to app.
		 * TODO: check if Rpt-START address is present
		 */
		if (data_len) {
			data_len--;
		}
		pdata = data->tm_rx_buf;
		pdata++;
		tcfg->callbacks->buf_write_received(tcfg, pdata, data_len);
	}

	k_event_clear(&data->events, UINT32_MAX);
	i2c_mec5_nl_target_arm(dev);
}

/* ISSUES:
 * 1. HW changes value of I2C.Status from 0x84 to 0x80 after ? clocks.
 *    ISR entry read of I2C.Status = 0x84 whereas when breakpoint on tm_process
 *    halts debugger and human reads registers I2C.Status = 0x80.
 *
 * 2. I2C-NL does not stop stretching the clock until the ISR exits. How I2C-NL
 *    detects the ISR is exiting? Stepped through the ISR and scope showed clock
 *    stretching continued until Zephyr ISR exit path executed.
 *
 * 3. For a write of 64 bytes from the external Controller we get one interrupt.
 *    data->dbg_isr_tm_cnt = 1 and data->dbg_isr_tm_aat_cnt = 1
 *    We never get TM_DONE interrupt so the TM is never reconfigured!
 */
static void tm_process(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct mec_i2c_smb_regs *regs = devcfg->regs;
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t kev = 0;
	uint32_t i2c_hw_sts = data->i2c_status;
	uint8_t target_addr = regs->SHAD_ADDR;

	I2C_NL_DEBUG_ISR_TM_COUNT_UPDATE(data);
	I2C_NL_DEBUG_STATE_UPDATE(data, 0x90);

	/* TODO invoke write received callback at IDLE after STOP */

	if (mec_hal_i2c_smb_is_aat_ien(hwctx)) {
		I2C_NL_DEBUG_ISR_TM_AAT_COUNT_UPDATE(data);
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x91);

		mec_hal_i2c_smb_idle_status_clr(hwctx);
		mec_hal_i2c_smb_intr_ctrl(hwctx, BIT(MEC_I2C_NL_IEN_AAT_POS), 0);
		mec_hal_i2c_smb_intr_ctrl(hwctx, BIT(MEC_I2C_IEN_IDLE_POS), 1);
		data->xfrflags |= I2C_NL_XFR_FLAG_TM_IDLE_REQ;
		data->active_addr_rw = target_addr;
		if (target_addr & BIT(0)) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x92);
			/* TODO: DMA dev2mem for target address. Reconfigure for mem2dev to send
			 * data requested by external Controller.
			 * We invoke read requested callback to application which supplies us with
			 * a buffer pointer and length.
			 * How to handle application returning NULL pointer and/or 0 length?
			 */
		} else {
			/* TODO: various scenarios
			 * ExtCtrl wrote < (tm_rd_cnt - 3) bytes
			 *	We get AAT then later TM_DONE then later IDLE (after STOP)
			 *		we invoke callback at TM_DONE
			 * ExtCtrl wrote (tm_rd_cnt - 3) bytes
			 *	We get AAT and no TM_DONE then later IDLE (after STOP)
			 *		we invoke callback at IDLE
			 * ExtCtrl wrote (tm_rd_cnt - 2) bytes: experiment not done
			 * ExtCtrl wrote (tm_rd_cnt - 1) bytes: experiment not done
			 * ExtCtrl wrote >= tm_rd_cnt bytes: HW NACK's byte when tm_rd_cnt 1->0. Need to confirm.
			 */
		}
	} else if (i2c_hw_sts & BIT(MEC_I2C_STS_TM_DONE_POS)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x98);
		if (target_addr & BIT(0)) { /* External Controller sent target read address */
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x99);
			/* TODO */
		} else { /* External Controller sent target write address */
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x9C);
			kev |= BIT(I2C_NL_KEV_TM_DONE_POS);
			if (i2c_hw_sts & BIT(MEC_I2C_STS_IDLE_POS)) {
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x9D);
				kev |= BIT(I2C_NL_KEV_IDLE_POS);
				tm_handle_idle(dev, target_addr);
			}
		}
	} else if (i2c_hw_sts & BIT(MEC_I2C_STS_IDLE_POS)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x9E);
		kev |= BIT(I2C_NL_KEV_IDLE_POS);
		tm_handle_idle(dev, target_addr);
	}

	if (kev) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x9F);
		k_event_post(&data->events, kev);
	}
}
#endif /* CONFIG_I2C_TARGET */

static void cm_process(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t kev = 0, i2c_hw_sts = 0, i2c_nl_ev = 0;

	i2c_hw_sts = data->i2c_status;

	/* if (mec_hal_i2c_smb_is_idle_intr(hwctx)) { */
	if ((i2c_hw_sts & BIT(MEC_I2C_STS_IDLE_POS))
	    && mec_hal_i2c_smb_is_idle_ien(hwctx)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x81);
		mec_hal_i2c_smb_idle_intr_enable(hwctx, 0);
		kev |= BIT(I2C_NL_KEV_IDLE_POS);
		data->mdone = 1;
	}

	kev |= check_errors(dev);

	if (!data->mdone && (i2c_hw_sts & BIT(MEC_I2C_STS_CM_DONE_POS))) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x86);
		i2c_nl_ev = mec_hal_i2c_nl_cm_event(hwctx);
		if (i2c_nl_ev == MEC_I2C_NL_CM_EVENT_W2R) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x89);
			start_cm_read_phase(dev);
		} else {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x8A);
			dma_stop(devcfg->dma_dev_cm, devcfg->dma_cm_chan);
			kev |= BIT(I2C_NL_KEV_CM_DONE_POS);
			data->mdone = 1;
		}
	}

	if (data->mdone) {
		/* check for late idle */
		i2c_hw_sts = mec_hal_i2c_smb_status(hwctx, 0);
		if (i2c_hw_sts & BIT(MEC_I2C_STS_LL_NBB_POS)) {
			mec_hal_i2c_smb_idle_intr_enable(hwctx, 0);
			mec_hal_i2c_smb_idle_status_clr(hwctx);
			kev |= BIT(I2C_NL_KEV_IDLE_POS);
		} else {
			mec_hal_i2c_smb_idle_intr_enable(hwctx, 1);
		}
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x8e);
		k_event_post(&data->events, kev);
	}
}

/* I2C ISR - must handle synchronos and asynchronous Controller Mode transfers
 * and Target Mode.
 * Interrupt events:
 *	CM_DONE when either CM write or read count reaches 0.
 *	TM_DONE when either TM write or read count reaches 0.
 *	IDLE if ISR has enabled IDLE interrupt. IDLE indicates bus has been freed.
 *	Bus Error, Lost Arbitration, and NAK will also cause CM_DONE or TM_DONE
 *
 * NOTE: IDLE interrupt should only be enabled in the ISR. IDLE detection HW logic looks
 * at state of SDA/SCL and will fire before START is issued/detected. IDLE interrupt provides
 * a safer method of knowing when STOP has completed and this controller sees SDA/SCL not being
 * driven.
 *
 *
 */
static void i2c_mec5_nl_isr(const struct device *dev)
{
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;

	I2C_NL_DEBUG_ISR_COUNT_UPDATE(data);
	I2C_NL_DEBUG_STATE_UPDATE(data, 0x80);

	/* get status and clear it */
	data->i2c_status = mec_hal_i2c_smb_status(hwctx, 1);

#ifdef CONFIG_I2C_TARGET
	if (data->targets_registered) {
		tm_process(dev);
	} else {
		cm_process(dev);
	}
#else
	cm_process(dev);
#endif

	mec_hal_i2c_smb_girq_status_clr(hwctx);

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x8f);
}

static const struct i2c_driver_api i2c_mec5_nl_driver_api = {
	.configure = i2c_mec5_nl_configure,
	.get_config = i2c_mec5_nl_get_config,
	.transfer = i2c_mec5_nl_transfer,
	.target_register = i2c_mec5_nl_target_register,
	.target_unregister = i2c_mec5_nl_target_unregister,
};

/* channel config:
 * completion callback at block list completion
 * error callback enabled
 * source and destination handshakes are hardware
 * set pointers to block config, callback function, and
 * callback is driver data.
 * all other members are 0
 * block config:
 * If TX
 *    channel direction is mem2dev
 *    source address is future memory location and destination
 *    address is I2C-NL CM TX buffer register address.
 *    increment source address (memory buffer)
 *    do not increment destination address (register)
 * else (RX)
 *    channel direction is dev2mem
 *    source address is I2C_NL CM RX buffer register address and destination
 *    address is memory buffer.
 *    do not increment source address (register)
 *    increment destination address (memory)
 */
static void init_dma_config(const struct device *dev, struct dma_config *dcfg,
			    struct dma_block_config *dbcfg, uint8_t cm, uint8_t dir)
{
	const struct i2c_mec5_nl_config *devcfg = dev->config;
	struct mec_i2c_smb_regs *regs = devcfg->regs;
	struct i2c_mec5_nl_data *data = dev->data;

	memset(dcfg, 0, sizeof(struct dma_config));
	memset(dbcfg, 0 ,sizeof(struct dma_block_config));

#ifdef CONFIG_I2C_TARGET
	if (cm) {
		dcfg->dma_slot = devcfg->cm_dma_trigsrc;
		dcfg->dma_callback = i2c_mec5_nl_cm_dma_cb;
	} else {
		dcfg->dma_slot = devcfg->tm_dma_trigsrc;
		dcfg->dma_callback = i2c_mec5_nl_tm_dma_cb;
	}
#else
	dcfg->dma_slot = devcfg->cm_dma_trigsrc;
	dcfg->dma_callback = i2c_mec5_nl_cm_dma_cb;
#endif

	dcfg->source_data_size = 1u;
	dcfg->dest_data_size = 1u;
	dcfg->block_count = 1u;
	dcfg->head_block = dbcfg;
	dcfg->user_data = (void *)data;

	if (dir == I2C_NL_DIR_WR) {
		dcfg->channel_direction = MEMORY_TO_PERIPHERAL;
		if (cm) {
			dbcfg->dest_address = (uint32_t)&regs->CM_TXB;
		} else {
			dbcfg->dest_address = (uint32_t)&regs->TM_TXB;
		}
		dbcfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dbcfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		dcfg->channel_direction = PERIPHERAL_TO_MEMORY;
		if (cm) {
			dbcfg->source_address = (uint32_t)&regs->CM_RXB;
		} else {
			dbcfg->source_address = (uint32_t)&regs->TM_RXB;
		}
		dbcfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dbcfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}
}

static int i2c_mec5_nl_init_dma(const struct device *dev)
{
	const struct i2c_mec5_nl_config *devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;

	if (!device_is_ready(devcfg->dma_dev_cm) || !device_is_ready(devcfg->dma_dev_tm)) {
		LOG_ERR("One or both DMA devices not ready");
		return -EIO;
	}

	init_dma_config(dev, &data->dma_cfg_cm_tx, &data->dma_blk_cm_tx, 1, I2C_NL_DIR_WR);
	init_dma_config(dev, &data->dma_cfg_cm_rx, &data->dma_blk_cm_rx, 1, I2C_NL_DIR_RD);
#ifdef CONFIG_I2C_TARGET
	init_dma_config(dev, &data->dma_cfg_tm_tx, &data->dma_blk_tm_tx, 0, I2C_NL_DIR_WR);
	init_dma_config(dev, &data->dma_cfg_tm_rx, &data->dma_blk_tm_rx, 0, I2C_NL_DIR_RD);
#endif
	return 0;
}

#ifdef I2C_NL_USE_DMA_DRIVER
/* DMA request returns the channel number (>=0) else a negative error */
static int i2c_mec5_nl_request_dma_channels(const struct device *i2c_dev)
{
	const struct i2c_mec5_nl_config *devcfg = i2c_dev->config;
	enum dma_channel_filter i2c_nl_dma_filter = DMA_CHANNEL_NORMAL;
	int ret;
	uint8_t chan;

	chan = devcfg->dma_cm_chan;
	ret = dma_request_channel(devcfg->dma_dev_cm, (void *)&i2c_nl_dma_filter);

	if (ret >= 0) {
		chan = devcfg->dma_tm_chan;
		ret = dma_request_channel(devcfg->dma_dev_tm, (void *)&i2c_nl_dma_filter);
	}

	if (ret < 0) {
		LOG_ERR("DMA request chan %u error(%d)", chan, ret);
	}

	return ret;
}
#endif

static int i2c_mec5_nl_init(const struct device *dev)
{
	const struct i2c_mec5_nl_config *devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	int ret;
	uint32_t bitrate_cfg;

	k_sem_init(&data->lock, 1, 1);
	k_event_init(&data->events);

	hwctx->base = devcfg->regs;
	hwctx->i2c_ctrl_cached = 0;
	data->devcfg = devcfg;
	data->state = I2C_NL_STATE_CLOSED;
	data->i2c_status = 0;
	data->mdone = 0;

#ifdef CONFIG_I2C_TARGET
	memset(data->target_cfgs, 0, sizeof(data->target_cfgs));
#endif
	ret = i2c_mec5_nl_init_dma(dev);
	if (ret) {
		return ret;
	}

	ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("pinctrl setup failed (%d)", ret);
		return ret;
	}

#ifdef I2C_NL_USE_DMA_DRIVER
	ret = i2c_mec5_nl_request_dma_channels(dev);
	if (ret < 0) {
		return ret;
	}
#endif

	bitrate_cfg = i2c_map_dt_bitrate(devcfg->clock_freq);
	if (!bitrate_cfg) {
		return -EINVAL;
	}

	/* Default configuration */
	ret = i2c_mec5_nl_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	if (ret) {
		return ret;
	}

	if (devcfg->irq_config_func) {
		devcfg->irq_config_func();
		mec_hal_i2c_smb_girq_ctrl(hwctx, MEC_I2C_SMB_GIRQ_EN | MEC_I2C_SMB_GIRQ_CLR_STS);
	}

	return 0;
}

/* dmas property is a phandle-array type containing:
 * phandle to DMA channel device
 * DMA channel number (0-based)
 * I2C controller trigger id for TX or RX
 * dma-cells:
 *   - channel
 *   - trigsrc
 */
#define I2C_NL_MEC5_DMA_CHAN(i, idx) \
	DT_INST_PHA_BY_IDX(i, dmas, idx, channel)

#define I2C_NL_MEC5_DMA_TRIGSRC(i, idx) \
	DT_INST_PHA_BY_IDX(i, dmas, idx, trigsrc)

#define I2C_NL_NODE(i) DT_INST(i, DT_DRV_COMPAT)

#define I2C_NL_DT_INST_DMA_CTLR(i, name) \
	DT_INST_DMAS_CTLR_BY_NAME(i, name)

#define I2C_NL_DT_INST_DMA_DEV(i, name) \
	DEVICE_DT_GET(I2C_NL_DT_INST_DMA_CTLR(i, name))

#define I2C_NL_DT_INST_DMA_REG_ADDR(i, name) \
	(struct dma_regs *)DT_REG_ADDR(I2C_NL_DT_INST_DMA_CTLR(i, name))

/* TODO we must add data item for size of each array ;<( */
#define I2C_NL_DT_CM_TX_BUF_MAX(i) DT_INST_PROP(i, cm_tx_buffer_size)
#define I2C_NL_DT_CM_TX_BUF_SIZE(i)						\
	(DT_INST_PROP(i, cm_tx_buffer_size) + I2C_NL_DRV_XFRBUF_PAD_SIZE)
#define I2C_NL_DT_DEFINE_CM_TX_BUF(i)						\
	static uint8_t i2c_mec5_nl_cm_tx_buf_##i[I2C_NL_DT_CM_TX_BUF_SIZE(i)] __aligned(4);

#ifdef CONFIG_I2C_TARGET
#if 0
#define I2C_NL_DT_TM_TX_BUF_MAX(i) DT_INST_PROP(i, tm_tx_buffer_size)
#define I2C_NL_DT_TM_TX_BUF_SIZE(i)						\
	(DT_INST_PROP(i, tm_tx_buffer_size) + I2C_NL_DRV_XFRBUF_PAD_SIZE)
#define I2C_NL_DT_DEFINE_TM_TX_BUF(i)						\
	static uint8_t i2c_mec5_nl_tm_tx_buf_##i[I2C_NL_DT_TM_TX_BUF_SIZE(i)] __aligned(4);
#endif
#define I2C_NL_DT_DEFINE_TM_TX_BUF(i)

#define I2C_NL_DT_TM_RX_BUF_MAX(i) DT_INST_PROP(i, tm_rx_buffer_size)
#define I2C_NL_DT_TM_RX_BUF_SIZE(i)						\
	(DT_INST_PROP(i, tm_rx_buffer_size) + I2C_NL_DRV_XFRBUF_PAD_SIZE)
#define I2C_NL_DT_DEFINE_TM_RX_BUF(i)						\
	static uint8_t i2c_mec5_nl_tm_rx_buf_##i[I2C_NL_DT_TM_RX_BUF_SIZE(i)] __aligned(4);

#define I2C_NL_DT_DATA_TM_TX_BUF(i)

#if 0
#define I2C_NL_DT_DATA_TM_TX_BUF(i)						\
	.tm_tx_buf = i2c_mec5_nl_tm_tx_buf_##i,					\
	.tm_tx_buf_max = I2C_NL_DT_TM_TX_BUF_MAX(i),
#endif

#define I2C_NL_DT_DATA_TM_RX_BUF(i)						\
	.tm_rx_buf = i2c_mec5_nl_tm_rx_buf_##i,					\
	.tm_rx_buf_max = I2C_NL_DT_TM_RX_BUF_MAX(i),

#else
#define I2C_NL_DT_DEFINE_TM_TX_BUF(i)
#define I2C_NL_DT_DEFINE_TM_RX_BUF(i)
#define I2C_NL_DT_DATA_TM_TX_BUF(i)
#define I2C_NL_DT_DATA_TM_RX_BUF(i)
#endif

#ifdef CONFIG_I2C_TARGET
BUILD_ASSERT(IS_ENABLED(CONFIG_I2C_TARGET_BUFFER_MODE),
	     "I2C target is enabled. This driver requires Target Buffer Mode.");
#endif

#define I2C_NL_INST_IRQ_PRI(i) DT_INST_IRQ(i, priority)

#define I2C_NL_DMA_TM_IRQ_PRI(i) \
	DT_IRQ_BY_IDX(DT_NODELABEL(dmac), I2C_NL_MEC5_DMA_CHAN(i, 1), priority)

/* Make sure DMA channel for target mode interrupt priority is higher than
 * I2C controller's interrupt priority. Cortex-Mx NVIC priority numeric values
 * are reversed: 0 is highest priority.
 */
#define I2C_NL_CHECK_IRQ_PRI(i)							\
	BUILD_ASSERT(I2C_NL_DMA_TM_IRQ_PRI(i) < I2C_NL_INST_IRQ_PRI(i),		\
		"TM DMA channel IRQ priority must be higher than I2C-NL")

#define I2C_NL_DEVICE(i)							\
	I2C_NL_CHECK_IRQ_PRI(i);						\
										\
	PINCTRL_DT_INST_DEFINE(i);						\
										\
	static void i2c_mec5_nl_irq_config_func_##i(void);			\
										\
	I2C_NL_DT_DEFINE_CM_TX_BUF(i)						\
	I2C_NL_DT_DEFINE_TM_TX_BUF(i)						\
	I2C_NL_DT_DEFINE_TM_RX_BUF(i)						\
										\
	static struct i2c_mec5_nl_data i2c_mec5_nl_data_##i = {			\
		.xfrbuf = i2c_mec5_nl_cm_tx_buf_##i,				\
		.xfrbuf_max = I2C_NL_DT_CM_TX_BUF_MAX(i),			\
		I2C_NL_DT_DATA_TM_TX_BUF(i)					\
		I2C_NL_DT_DATA_TM_RX_BUF(i)					\
	};									\
										\
	static const struct i2c_mec5_nl_config i2c_mec5_nl_dcfg_##i = {		\
		.regs = (struct mec_i2c_smb_regs *)DT_INST_REG_ADDR(i),		\
		.port_sel = DT_INST_PROP(i, port_sel),				\
		.clock_freq = DT_INST_PROP(i, clock_frequency),			\
		.init_pin_wait_us = DT_INST_PROP_OR(i, init_pin_wait, 100),	\
		.cfg_pin_wait_us = DT_INST_PROP_OR(i, config_pin_wait, 35000),	\
		.sda_gpio = GPIO_DT_SPEC_INST_GET(i, sda_gpios),		\
		.scl_gpio = GPIO_DT_SPEC_INST_GET(i, scl_gpios),		\
		.irq_config_func = i2c_mec5_nl_irq_config_func_##i,		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),			\
		.dma_dev_cm = I2C_NL_DT_INST_DMA_DEV(i, cm),			\
		.dma_dev_tm = I2C_NL_DT_INST_DMA_DEV(i, tm),			\
		.dma_cm_regs = I2C_NL_DT_INST_DMA_REG_ADDR(i, cm),		\
		.dma_tm_regs = I2C_NL_DT_INST_DMA_REG_ADDR(i, tm),		\
		.dma_cm_chan = I2C_NL_MEC5_DMA_CHAN(i, 0),			\
		.dma_tm_chan = I2C_NL_MEC5_DMA_CHAN(i, 1),			\
		.cm_dma_trigsrc = I2C_NL_MEC5_DMA_TRIGSRC(i, 0),		\
		.tm_dma_trigsrc = I2C_NL_MEC5_DMA_TRIGSRC(i, 1),		\
	};									\
	I2C_DEVICE_DT_INST_DEFINE(i, i2c_mec5_nl_init, NULL,			\
		&i2c_mec5_nl_data_##i, &i2c_mec5_nl_dcfg_##i,			\
		POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,				\
		&i2c_mec5_nl_driver_api);					\
										\
	static void i2c_mec5_nl_irq_config_func_##i(void)			\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(i),					\
			    DT_INST_IRQ(i, priority),				\
			    i2c_mec5_nl_isr,					\
			    DEVICE_DT_INST_GET(i), 0);				\
		irq_enable(DT_INST_IRQN(i));					\
	}

DT_INST_FOREACH_STATUS_OKAY(I2C_NL_DEVICE)
