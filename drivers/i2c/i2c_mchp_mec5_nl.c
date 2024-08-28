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
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/sys_io.h>
LOG_MODULE_REGISTER(i2c_nl_mchp, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_dmac_api.h>
#include <mec_ecia_api.h>
#include <mec_i2c_api.h>

#define I2C_NL_DEBUG_ISR
#define I2C_NL_DEBUG_ISR_ENTRIES		64
#define I2C_NL_DEBUG_STATE
#define I2C_NL_DEBUG_STATE_ENTRIES		256
#define I2C_NL_DEBUG_NO_KEV_WAIT_TIMEOUT

#define I2C_MEC5_NL_RESET_WAIT_US		20

/* SMBus specification default timeout in milliseconds */
#define I2C_MEC5_NL_SMBUS_TMOUT_MAX_MS		35

/* I2C timeout is  10 ms (WAIT_INTERVAL * WAIT_COUNT) */
#define I2C_MEC5_NL_WAIT_INTERVAL_US		50
#define I2C_MEC5_NL_WAIT_COUNT			200
#define I2C_MEC5_NL_STOP_WAIT_COUNT		500
#define I2C_MEC5_NL_PIN_CFG_WAIT		50

#define I2C_MEC5_NL_MAX_XFR_LEN			0xfff8u
/* padding lengths: I2C-NL hardware requires target
 * address in the memory buffer.
 * start_target_address | data | rpt_start_target_address | data
 */
#define I2C_MEC5_NL_CM_TX_BUF_PAD_LEN		8
#define I2C_MEC5_NL_TM_RX_BUF_PAD_LEN		8

#define I2C_MEC5_NL_XFRBUF_DATA_LEN_MAX		(CONFIG_I2C_MCHP_MEC5_NL_BUFFER_SIZE)
#define I2C_MEC5_NL_XFRBUF_SIZE			((I2C_MEC5_NL_XFRBUF_DATA_LEN_MAX) + 8u)

#define I2C_MEC5_NL_MISC_CFG_PORT_POS		0
#define I2C_MEC5_NL_MISC_CFG_PORT_MSK		0xfu

enum i2c_mec5_nl_state {
	I2C_NL_STATE_CLOSED = 0,
	I2C_NL_STATE_OPEN,
};

enum i2c_mec5_nl_direction {
	I2C_NL_DIR_WR = 0,
	I2C_NL_DIR_RD,
};

enum i2c_mec5_nl_error {
	I2C_NL_ERR_NONE = 0,
	I2C_NL_ERR_BUS,
	I2C_NL_ERR_LOST_ARB,
	I2C_NL_ERR_NACK_FROM_TARGET,
	I2C_NL_ERR_TIMEOUT,
};

enum i2c_mec5_nl_kevents {
	I2C_NL_KEV_IDLE_POS = 0,
	I2C_NL_KEV_BERR_POS,
	I2C_NL_KEV_LAB_ERR_POS,
	I2C_NL_KEV_CM_NAK_POS,
	I2C_NL_KEV_W2R_POS,
	I2C_NL_KEV_CM_DONE_POS,
	I2C_NL_KEV_DMA_CM_DONE_POS,
	I2C_NL_KEV_DMA_CM_ERR_POS,
	I2C_NL_KEV_TM_AAT_POS,
	I2C_NL_KEV_TM_DONE_POS,
	I2C_NL_KEV_TM_TURN_AROUND_POS,
	I2C_NL_KEV_DMA_TM_DONE_POS,
	I2C_NL_KEV_DMA_TM_ERR_POS,
};

#define I2C_NL_WAIT_EVENTS_MSK (BIT(I2C_NL_KEV_IDLE_POS) | BIT(I2C_NL_KEV_CM_NAK_POS) \
				| BIT(I2C_NL_KEV_DMA_CM_ERR_POS) \
				| BIT(I2C_NL_KEV_DMA_TM_ERR_POS))

#define I2C_NL_ERRORS \
	(BIT(I2C_NL_KEV_DMA_CM_ERR_POS) | BIT(I2C_NL_KEV_DMA_TM_ERR_POS) \
	 | BIT(I2C_NL_KEV_BERR_POS) | BIT(I2C_NL_KEV_LAB_ERR_POS) \
	 | BIT(I2C_NL_KEV_CM_NAK_POS))


struct i2c_mec5_nl_config {
	struct mec_i2c_smb_regs *regs;
	uint32_t init_pin_wait_us;
	uint32_t cfg_pin_wait_us;
	struct gpio_dt_spec sda_gpio;
	struct gpio_dt_spec scl_gpio;
	void (*irq_config_func)(void);
	const struct pinctrl_dev_config *pcfg;
	const struct device *dma_dev;
	uint8_t cm_dma_chan;
	uint8_t cm_dma_trigsrc;
#ifdef CONFIG_I2C_TARGET
	uint8_t tm_dma_chan;
	uint8_t tm_dma_trigsrc;
#endif
};

/* I2C message group for Network layer hardware
 * Hardware can support protocols:
 * 1. I2C read of a single message. Length <= I2C_MEC5_NL_MAX_XFR_LEN
 * 2. I2C write of multiple messages. Combined lenth <= I2C_MEC5_NL_MAX_XFR_LEN
 * 3. I2C write multiple followed by one I2C read. Same length limits.
 * The driver only supports one read message to avoid a HW race conditions.
 */
struct i2c_msg_group {
	uint16_t wr_len;
	uint8_t wr_idx;		/* start index in message array of firt write message */
	uint8_t wr_msg_cnt;	/* number of consecutive write messages */
	uint16_t rd_len;
	uint8_t rd_idx;		/* index in message array of single read message */
	uint8_t rd_msg_cnt;	/* number of consecutive read messages (0 or 1) */
};

#ifdef I2C_NL_DEBUG_ISR
struct i2c_mec5_dbg_isr_data {
	uint32_t isr_cnt;
	uint32_t status;
	uint32_t config;
	uint32_t cm_cmd;
	uint32_t tm_cmd;
};
#endif

struct i2c_mec5_nl_data {
	const struct i2c_mec5_nl_config *devcfg;
	struct mec_i2c_smb_ctx ctx;
	struct k_sem lock;
	struct k_event events;
	uint32_t clock_freq_hz;
	uint32_t i2c_status;
	uint32_t xfr_tmout;
	uint8_t cm_target_i2c_wr_addr;
	uint8_t xdone;
	uint8_t state;
	uint8_t misc_cfg; /* b[3:0]=port, b[6:4]=rsvd, b[7]=0(CM), 1(TM) */
	uint8_t *rembuf;
	uint16_t remlen;
	uint8_t num_msgs;
	uint8_t msgidx;
	struct i2c_msg *msgs;
	struct i2c_msg_group mgrp;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
	uint8_t *xfrbuf;
#ifdef CONFIG_I2C_TARGET
	struct i2c_target_config *target1_cfg;
	struct i2c_target_config *target2_cfg;
	uint8_t *tm_rx_buf;
	size_t tm_rx_buf_sz;
	uint32_t tm_cmd_addr;
	uint8_t *tm_tx_buf;
	uint32_t tm_tx_buf_sz;
	uint32_t tm_tx_buf_xfr_sz;
#endif
#ifdef I2C_NL_DEBUG_ISR
	volatile uint32_t dbg_isr_cnt;
	volatile uint32_t dbg_isr_idx;
	struct i2c_mec5_dbg_isr_data dbg_isr_data[I2C_NL_DEBUG_ISR_ENTRIES];
#endif
#ifdef I2C_NL_DEBUG_STATE
	atomic_t dbg_state_idx;
	uint8_t dbg_states[I2C_NL_DEBUG_STATE_ENTRIES];
#endif
};

#ifdef CONFIG_I2C_TARGET
static void i2c_mec5_nl_tm_dma_cb(const struct device *dev, void *user_data,
				  uint32_t channel, int status);
#endif

#ifdef I2C_NL_DEBUG_ISR
static inline void i2c_mec5_nl_dbg_isr_init(struct i2c_mec5_nl_data *data)
{
	data->dbg_isr_cnt = 0;
	data->dbg_isr_idx = 0;
	memset(data->dbg_isr_data, 0, sizeof(data->dbg_isr_data));
}

static inline void i2c_mec5_nl_dbg_isr_cnt_update(struct i2c_mec5_nl_data *data)
{
	data->dbg_isr_cnt++;
}

static inline void i2c_mec5_nl_dbg_isr_data_update(struct i2c_mec5_nl_data *data)
{
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct mec_i2c_smb_regs *regs = hwctx->base;
	uint32_t idx = data->dbg_isr_idx;

	data->dbg_isr_idx++;

	if (idx < I2C_NL_DEBUG_ISR_ENTRIES) {
		data->dbg_isr_data[idx].isr_cnt = data->dbg_isr_cnt;
		data->dbg_isr_data[idx].status = data->i2c_status;
		data->dbg_isr_data[idx].config = regs->CONFIG;
		data->dbg_isr_data[idx].cm_cmd = regs->CM_CMD;
		data->dbg_isr_data[idx].tm_cmd = regs->TM_CMD | (regs->SHAD_ADDR << 24);
	}
}

#define I2C_NL_DEBUG_ISR_INIT(d) i2c_mec5_nl_dbg_isr_init(d)
#define I2C_NL_DEBUG_ISR_COUNT_UPDATE(d) i2c_mec5_nl_dbg_isr_cnt_update(d)
#define I2C_NL_DEBUG_ISR_DATA_UPDATE(d) i2c_mec5_nl_dbg_isr_data_update(d)
#else
#define I2C_NL_DEBUG_ISR_INIT(d)
#define I2C_NL_DEBUG_ISR_COUNT_UPDATE(d)
#define I2C_NL_DEBUG_ISR_DATA_UPDATE(d)
#endif

#ifdef I2C_NL_DEBUG_STATE
static void i2c_mec5_nl_dbg_state_init(struct i2c_mec5_nl_data *data)
{
	data->dbg_state_idx = ATOMIC_INIT(0);
	memset(data->dbg_states, 0, sizeof(data->dbg_states));
}

static void i2c_mec5_nl_dbg_state_update(struct i2c_mec5_nl_data *data, uint8_t state)
{
	atomic_val_t idx = atomic_inc(&data->dbg_state_idx); /* returns previous value */

	if (idx < I2C_NL_DEBUG_STATE_ENTRIES) {
		data->dbg_states[idx] = state;
	}
}

#define I2C_NL_DEBUG_STATE_INIT(d) i2c_mec5_nl_dbg_state_init(d)
#define I2C_NL_DEBUG_STATE_UPDATE(d, state) i2c_mec5_nl_dbg_state_update(d, state)
#else
#define I2C_NL_DEBUG_STATE_INIT(d)
#define I2C_NL_DEBUG_STATE_UPDATE(d, state)
#endif


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
		k_busy_wait(I2C_MEC5_NL_WAIT_INTERVAL_US);
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

static int i2c_mec5_nl_reset_config(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct mec_i2c_smb_cfg mcfg = {0};
	int ret = 0;

	hwctx->base = devcfg->regs;
	hwctx->i2c_ctrl_cached = 0;
	data->i2c_status = 0;

	if (data->clock_freq_hz >= MHZ(1)) {
		mcfg.std_freq = MEC_I2C_STD_FREQ_1M;
	} else if (data->clock_freq_hz >= KHZ(400)) {
		mcfg.std_freq = MEC_I2C_STD_FREQ_400K;
	} else {
		mcfg.std_freq = MEC_I2C_STD_FREQ_100K;
	}

	mcfg.port = ((data->misc_cfg >> I2C_MEC5_NL_MISC_CFG_PORT_POS)
		     & I2C_MEC5_NL_MISC_CFG_PORT_MSK);

#ifdef CONFIG_I2C_TARGET
	if (data->target1_cfg || data->target2_cfg) {
		mcfg.cfg_flags |= MEC_I2C_SMB_CFG_PRESERVE_TARGET_ADDRS;
	}
#endif

	ret = mec_hal_i2c_smb_init(hwctx, &mcfg, NULL);
	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	return 0;
}

/* MEC5 I2C controller support 7-bit addressing only.
 * Format 7-bit address for as it appears on the bus as an 8-bit
 * value with R/W bit at bit[0], 0(write), 1(read).
 */
static inline uint8_t fmt_addr(uint16_t addr, enum i2c_mec5_nl_direction dir)
{
	uint8_t fmt_addr = (uint8_t)((addr & 0x7fu) << 1);

	if (dir == I2C_NL_DIR_RD) {
		fmt_addr |= BIT(0);
	}

	return fmt_addr;
}

/* message group helpers */
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

	if (m->len > I2C_MEC5_NL_MAX_XFR_LEN) {
		LOG_ERR("Length exceeds HW capabilities");
		return -EMSGSIZE;
	}

	if (!(m->flags & I2C_MSG_READ)) {
		if (m->len > (I2C_MEC5_NL_MAX_XFR_LEN - g->wr_len)) {
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

/* I2C Controller mode DMA channel callback */
static void i2c_mec5_nl_cm_dma_cb(const struct device *dev, void *user_data,
				  uint32_t channel, int status)
{
	struct i2c_mec5_nl_data *data = (struct i2c_mec5_nl_data *)user_data;
	const struct i2c_mec5_nl_config *devcfg = data->devcfg;
	struct mec_i2c_smb_regs *regs = devcfg->regs;
	struct dma_status dma_sts = {0};
	struct i2c_msg *m = NULL;
	uint32_t dest = 0, src = 0;
	size_t dmalen = 0;
	int ret = 0;

	if (status < 0) {
		LOG_ERR("I2C-NL CM DMA CB chan %u error (%d)", channel, status);
		k_event_post(&data->events, BIT(I2C_NL_KEV_DMA_CM_ERR_POS));
	}

	ret = dma_get_status(dev, channel, &dma_sts);
	if (ret) {
		LOG_ERR("I2C-NL CM DMA CB chan %u get status error (%d)", channel, status);
		return;
	}

	if (channel == devcfg->cm_dma_chan) {
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
				dma_reload(dev, channel, src, dest, dmalen);
				dma_start(dev, channel);
				return;
			}
		}
		k_event_post(&data->events, BIT(I2C_NL_KEV_DMA_CM_DONE_POS));
	}
}

#define I2C_MEC5_NL_DMA_START_FLAG_CM	0
#define I2C_MEC5_NL_DMA_START_FLAG_TM	BIT(0)
#define I2C_MEC5_NL_DMA_START_FLAG_RUN	BIT(4)

static int i2c_mec5_nl_dma_start(const struct device *dev, uint8_t *buf, size_t blen,
				 uint8_t dir, uint32_t flags)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct mec_i2c_smb_regs *const regs = devcfg->regs;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct dma_config *dcfg = &data->dma_cfg;
	struct dma_block_config *dblk = &data->dma_blk_cfg;
	uint32_t chan = (uint32_t)devcfg->cm_dma_chan;
	uint32_t slot = (uint32_t)devcfg->cm_dma_trigsrc;
	int ret = 0;

	memset(dcfg, 0, sizeof(struct dma_config));
	memset(dblk, 0, sizeof(struct dma_block_config));

#ifdef CONFIG_I2C_TARGET
	if (flags & I2C_MEC5_NL_DMA_START_FLAG_TM) { /* is target mode? */
		chan = (uint32_t)devcfg->tm_dma_chan;
		slot = (uint32_t)devcfg->tm_dma_trigsrc;
	}
#endif

	if (dir == I2C_NL_DIR_WR) {
		dcfg->channel_direction = MEMORY_TO_PERIPHERAL;
		dblk->source_address = (uint32_t)buf;
		dblk->dest_address = (uint32_t)&regs->CM_TXB;
#ifdef CONFIG_I2C_TARGET
		if (flags & I2C_MEC5_NL_DMA_START_FLAG_TM) {
			dblk->dest_address = (uint32_t)&regs->TM_TXB;
		}
#endif
		dblk->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dblk->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		dcfg->channel_direction = PERIPHERAL_TO_MEMORY;
		dblk->source_address = (uint32_t)&regs->CM_RXB;
#ifdef CONFIG_I2C_TARGET
		if (flags & I2C_MEC5_NL_DMA_START_FLAG_TM) {
			dblk->source_address = (uint32_t)&regs->TM_RXB;
		}
#endif
		dblk->dest_address = (uint32_t)buf;
		dblk->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dblk->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}

	dblk->block_size = blen;

	dcfg->dma_slot = slot;
	dcfg->source_data_size = 1;
	dcfg->dest_data_size = 1;
	dcfg->block_count = 1;
	dcfg->head_block = dblk;
	dcfg->user_data = (void *)data;
	dcfg->dma_callback = i2c_mec5_nl_cm_dma_cb;
#ifdef CONFIG_I2C_TARGET
	if (flags & I2C_MEC5_NL_DMA_START_FLAG_TM) {
		dcfg->dma_callback = i2c_mec5_nl_tm_dma_cb;
	}
#endif
	ret = dma_config(devcfg->dma_dev, chan, dcfg);
	if ((ret == 0) && (flags & I2C_MEC5_NL_DMA_START_FLAG_RUN)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x40u);
		ret = dma_start(devcfg->dma_dev, chan);
	}

	return ret;
}

static void mec5_nl_hw_prep(const struct device *dev)
{
	struct i2c_mec5_nl_data *const data = dev->data;
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
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct i2c_msg_group *g = &data->mgrp;
	uint8_t *xbuf = data->xfrbuf;
	uint8_t *xbuf_end = data->xfrbuf;
	uint32_t i2c_nl_wr_len = g->wr_len + 1;
	uint32_t dma_mem2dev_len = 1;
	uint32_t hal_flags = (MEC_I2C_NL_FLAG_START | MEC_I2C_NL_FLAG_STOP
			      | MEC_I2C_NL_FLAG_CM_DONE_IEN | MEC_I2C_NL_FLAG_IDLE_IEN);
	uint32_t i2c_nl_rd_len = 0;
	struct i2c_msg *m = NULL;
	int ret = 0;
	uint16_t max_idx = 0;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x20u);

	mec5_nl_hw_prep(dev);
	k_event_clear(&data->events, UINT32_MAX);

	data->i2c_status = 0;
	data->rembuf = NULL;
	data->remlen = 0;
	data->xdone = 0;

	xbuf[0] = data->cm_target_i2c_wr_addr;
	xbuf_end = &xbuf[1];

	max_idx = g->wr_idx + g->wr_msg_cnt;
	if (max_idx > 255u) {
		max_idx = 255u;
	}

	while (g->wr_idx < max_idx) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x21u);
		m = &data->msgs[g->wr_idx];
		if (m->len <= I2C_MEC5_NL_XFRBUF_DATA_LEN_MAX) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x22u);
			memcpy(xbuf_end, m->buf, m->len);
			xbuf_end += m->len;
			dma_mem2dev_len += m->len;
		} else {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x23u);
			memcpy(xbuf_end, m->buf, I2C_MEC5_NL_XFRBUF_DATA_LEN_MAX);
			xbuf_end += m->len;
			dma_mem2dev_len += I2C_MEC5_NL_XFRBUF_DATA_LEN_MAX;
			data->rembuf = &m->buf[I2C_MEC5_NL_XFRBUF_DATA_LEN_MAX];
			data->remlen = m->len - I2C_MEC5_NL_XFRBUF_DATA_LEN_MAX;
			break;
		}
		g->wr_idx++;
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x24u);
	if (g->rd_msg_cnt) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x25u);
		m = &data->msgs[g->rd_idx];
		i2c_nl_rd_len = m->len;
		if (g->wr_msg_cnt) { /* store read-address at end of transmit data */
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x26u);
			*xbuf_end = xbuf[0] | BIT(0);
			i2c_nl_wr_len++;
			dma_mem2dev_len++;
			hal_flags |= MEC_I2C_NL_FLAG_RPT_START;
		} else { /* I2C read protocol: START address is target read */
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x27u);
			xbuf[0] |= BIT(0);
		}
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x28u);
	ret = i2c_mec5_nl_dma_start(dev, data->xfrbuf, dma_mem2dev_len, I2C_NL_DIR_WR,
				    (I2C_MEC5_NL_DMA_START_FLAG_CM
				     | I2C_MEC5_NL_DMA_START_FLAG_RUN));
	if (ret) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x29u);
		LOG_ERR("CM TX DMA start error (%d)", ret);
		return ret;
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x2Au);
	ret = mec_hal_i2c_nl_cm_cfg_start(hwctx, i2c_nl_wr_len, i2c_nl_rd_len, hal_flags);
	if (ret) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x2Bu);
		LOG_ERR("I2C-NL start err (%d)", ret);
		ret = -EIO;
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x2Fu);

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
	struct i2c_mec5_nl_data *const data = dev->data;
	struct i2c_msg_group *g = &data->mgrp;
	uint32_t i2c_clks = g->wr_len + g->rd_len; /* lengths are 16-bit */
	uint32_t i2c_freq = data->clock_freq_hz;
	uint32_t tmout = 0;

	i2c_clks *= 10000u;
	tmout = i2c_clks / i2c_freq;
	tmout += 10u; /* paranoid: add 10 ms */
	if (tmout < I2C_MEC5_NL_SMBUS_TMOUT_MAX_MS) {
		tmout = I2C_MEC5_NL_SMBUS_TMOUT_MAX_MS;
	}

	data->xfr_tmout = tmout;
}

static int i2c_mec5_nl_do_stop(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t ev, evw;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x60u);

	/* Can we trust GIRQ status has been cleared before we re-enable? */
	if (mec_hal_i2c_smb_is_bus_owned(hwctx)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x61u);
		data->xdone = 0;
		mec_hal_i2c_smb_stop_gen(hwctx);
		mec_hal_i2c_smb_girq_status_clr(hwctx);
		mec_hal_i2c_smb_idle_intr_enable(hwctx, 1);
		mec_hal_i2c_smb_girq_ctrl(hwctx, MEC_I2C_SMB_GIRQ_EN);

		evw = (BIT(I2C_NL_KEV_IDLE_POS) | BIT(I2C_NL_KEV_BERR_POS)
			| BIT(I2C_NL_KEV_LAB_ERR_POS));
		ev = k_event_wait(&data->events, evw, false, K_MSEC(100));
		if (!ev) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x62u);
			LOG_ERR("Gen STOP timeout");
		}

		dma_stop(devcfg->dma_dev, devcfg->cm_dma_chan);
		mec_hal_i2c_nl_cmd_clear(hwctx, MEC_I2C_NL_CM_SEL);
	}

	data->state = I2C_NL_STATE_CLOSED;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x6Fu);

	return 0;
}

static int i2c_mec5_nl_wait_events(const struct device *dev, uint32_t evmask)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;
	int ret = 0;
	uint32_t ev = 0;
#ifdef I2C_NL_DEBUG_NO_KEV_WAIT_TIMEOUT
	k_timeout_t event_wait_timeout = K_FOREVER;
#else
	k_timeout_t event_wait_timeout = K_MSEC(data->xfr_tmout)
#endif

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x30u);

	ev = k_event_wait(&data->events, evmask, false, event_wait_timeout);

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x31u);

	if (!ev) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x32u);
		LOG_ERR("I2C-NL CM event timeout");
		i2c_mec5_nl_do_stop(dev);
		mec_hal_dma_chan_stop(devcfg->cm_dma_chan);
		ret = -ETIMEDOUT;
	} else if (ev & I2C_NL_ERRORS) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x33u);
		LOG_ERR("CM event errors = 0x%08x", ev);
		ret = -EIO;
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x34u);

	return ret;
}

/* Process I2C messages into a I2C-NL HW message group and start transfer.
 * Concantenate as much of the write messages into driver transfer buffer
 * before starting.
 */
static int process_i2c_msgs(const struct device *dev)
{
	struct i2c_mec5_nl_data *const data = dev->data;
	struct i2c_msg_group *g = &data->mgrp;
	int ret = 0;
	uint8_t n = data->msgidx;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x10u);

	while (n < data->num_msgs) {
		struct i2c_msg *m = &data->msgs[n];

		ret = msg_check(g, m, n);
		if (ret) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x11u);
			LOG_ERR("check msg[%u] error (%d)", n, ret);
			return ret;
		}

		ret = msg_grp_update(g, m, n);
		if (ret) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x12u);
			LOG_ERR("update error: msg[%u] = (%p, %u, 0x%0x)",
				n , m->buf, m->len, m->flags);
			return ret;
		}

		if (m->flags & (I2C_MSG_READ | I2C_MSG_STOP)) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x13u);
			msg_grp_compute_timeout(dev);
			ret = msg_grp_start_xfr(dev);
			if (ret) {
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x14u);
				LOG_ERR("start xfr error (%d)", ret);
				return ret;
			}

			I2C_NL_DEBUG_STATE_UPDATE(data, 0x15u);
			ret = i2c_mec5_nl_wait_events(dev, I2C_NL_WAIT_EVENTS_MSK);
			if (ret) {
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x16u);
				LOG_ERR("wait events returned (%d)", ret);
				break;
			}

			I2C_NL_DEBUG_STATE_UPDATE(data, 0x17u);
			memset(g, 0, sizeof(struct i2c_msg_group));
		}
		n++;
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x1Fu);

	return ret;
}

#ifdef CONFIG_I2C_TARGET

static bool i2c_targets_are_registered(const struct device *dev)
{
	struct i2c_mec5_nl_data *const data = dev->data;

	if (data->target1_cfg || data->target2_cfg) {
		return true;
	}

	return false;
}

/* Handle TM DMA channel interrupts: error or done
 * We limit maximum transfers to I2C_MEC5_NL_MAX_XFR_LEN (0xfff8) bytes.
 * to avoid much more complex code logic. If arbitrary sizes must be handled
 * (full 32-bit byte count) we must prevent I2C-NL TM wrCnt reaching 1 or 0
 * until the last chunk of data. We must also track the application buffer
 * in order to reload the DMA channel. Finally, we must experiment with I2C-NL
 * HW to insure it clock stretches when TM DMA is done with each chunk.
 * This is why we are limiting transfer to I2C_MEC5_NL_MAX_XFR_LEN bytes.
 */
static void i2c_mec5_nl_tm_dma_cb(const struct device *dev, void *user_data,
				  uint32_t channel, int status)
{
	struct i2c_mec5_nl_data *data = (struct i2c_mec5_nl_data *)user_data;
	struct dma_status dma_sts = {0};
	int ret = 0;

#ifdef I2C_NL_DEBUG_ISR
	LOG_INF("I2C-NL TM DMA CB: chan %u status %d", channel, status);
#endif

	if (status < 0) {
		k_event_post(&data->events, BIT(I2C_NL_KEV_DMA_TM_ERR_POS));
	}

	ret = dma_get_status(dev, channel, &dma_sts);
	if (ret) {
		LOG_ERR("DMA chan %u get status error (%d)", channel, status);
		return;
	}

	if (!dma_sts.busy) {
		k_event_post(&data->events, BIT(I2C_NL_KEV_DMA_TM_DONE_POS));
	}
}

/* Configure and ARM I2C-NL and TM DMA channel for receiving data
 * from an external I2C controller. External I2C read and write
 * always transmits START followed by the target address.
 * If the target address matches one of the two addresses in
 * this I2C controller's OWN_ADDR register then I2C-NL HW FSM
 * will trigger DMA to read the address. TM DMA must be configured
 * for peripheral-to-memory from TM_RXB register to driver tm_rx_buf.
 * I2C-NL TM read and write counts are set to our chosen maximum,
 * 0xfff8 bytes. DMA is used to limit the data based on the
 * buffer size: driver tm_rx_buf size or buffer size supplied by
 * the application read requested callback.
 * NOTE 1: External Controller write N bytes of data
 * driver tm_rx_buf = target_addr, data0, data1, ... dataN-1
 * If N > buffer size, DMA will stop and I2C-NL HW FSM will NAK
 * NOTE 2: External Controller read N bytes of data
 * I2C-NL HW moves target_addr to tm_rx_buf
 * I2C-NL clock stretches and signals FW to change DMA direction
 * FW(ISR) invokes read requested callback to obtain buffer and length
 * FW(ISR) reconfigures TM DMA channel for memory-to-peripheral
 * using app buffer and length.
 * FW(ISR) signals I2C-NL to proceed.
 * I2C-NL HW FSM stops clock stretching
 * External Controller resumes issuing clocks on I2C SCL line.
 * I2C-NL HW FSM triggers TM DMA to supply data and puts it on SDA line.
 */
static int i2c_mec5_nl_target_arm(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t tm_flags = MEC_I2C_NL_TM_FLAG_DONE_IEN | MEC_I2C_NL_TM_FLAG_RUN;
	uint32_t clrmsk = BIT(MEC_I2C_IEN_IDLE_POS) | BIT(MEC_I2C_NL_IEN_CM_DONE_POS)
			  | BIT(MEC_I2C_NL_IEN_TM_DONE_POS) | BIT(MEC_I2C_NL_IEN_AAT_POS);
	uint16_t nrx = I2C_MEC5_NL_MAX_XFR_LEN;
	uint16_t ntx = I2C_MEC5_NL_MAX_XFR_LEN;

	int ret = 0;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x50u);

	dma_stop(devcfg->dma_dev, devcfg->tm_dma_chan);
	mec_hal_i2c_smb_intr_ctrl(hwctx, clrmsk, 0);
	mec_hal_i2c_nl_cmd_clear(hwctx, MEC_I2C_NL_TM_SEL);
	mec_hal_i2c_nl_flush_buffers(hwctx->base);

	/* clear all events before re-arming TM */
	k_event_clear(&data->events, UINT32_MAX);

	ret = mec_hal_i2c_nl_tm_config(hwctx, ntx, nrx, tm_flags);
	if (ret != MEC_RET_OK) {
		ret = -EIO;
	}

	ret = i2c_mec5_nl_dma_start(dev, data->tm_rx_buf, nrx, I2C_NL_DIR_RD,
				    (I2C_MEC5_NL_DMA_START_FLAG_TM
				     | I2C_MEC5_NL_DMA_START_FLAG_RUN));

	return ret;
}

/* Return a pointer to the struct i2c_target_config with address matching target
 * address matched by I2C-NL OWN_ADDR register addresses.
 * parameter addr is the full 8-bit address plus nW/R bit.
 * taddr8[0] = nW/R
 * taddr8[7:1] = target address
 */
static struct i2c_target_config *i2c_target_find(const struct device *dev, uint16_t taddr8)
{
	struct i2c_mec5_nl_data *const data = dev->data;
	uint16_t taddr7 = taddr8 >> 1;

	if (data->target1_cfg && (data->target1_cfg->address == taddr7)) {
		return data->target1_cfg;
	}

	if (data->target2_cfg && (data->target2_cfg->address == taddr7)) {
		return data->target2_cfg;
	}

	return NULL;
}

/*
 * shad_addr: b[7:1]=matched target address, b[0]=nW/R
 * Lots of corner cases to handle:
 * I2C-NL expected target address as first byte of the buffer.
 * Read requested:
 *	Zephyr I2C target API does not allow registering a read
 *	request buffer. When we receive a read request we must
 *	stretch the clock and invoke the read request callback.
 *	If the application gives us a buffer we configure I2C-NL
 *	TM_CMD and DMA channel, set TM_PROCEED and let the HW
 *	move data from memory to I2C-NL to I2C lines.
 * Write received:
 *	Write fits driver buffer
 *	Write fits but is START wrAddr wrData ... RPT-START wrAddr more_wrData
 *		Do we make two callbacks?
 */
static void tm_write_cb(const struct device *dev, uint8_t shad_addr)
{
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t rdcnt = 0, dlen = 0;
	struct i2c_target_config *tcfg = i2c_target_find(dev, shad_addr);

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xA0u);

	if (!tcfg || !tcfg->callbacks) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xA1u);
		return;
	}

	const struct i2c_target_callbacks *cb = tcfg->callbacks;

	if (cb->buf_write_received) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xA2u);
		rdcnt = mec_hal_i2c_nl_tm_xfr_count_get(hwctx, 1);
		if (hwctx->rdcnt > rdcnt) {
			dlen = hwctx->rdcnt - rdcnt;
			if (dlen) { /* I2C-NL DMA's address and data into buffer */
				dlen--;
			}
		}

		cb->buf_write_received(tcfg, &data->tm_rx_buf[1], dlen);
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xAFu);
}

/* Target transmitted START and matching target read address.
 * I2C-NL will clock stretch on a matching target read address.
 * Invoke application read requested API to obtain a buffer and buffer length.
 * If application supplied a buffer:
 *   Configure TM DMA for memory-to-peripheral from the buffer.
 *   Set TM_CMD PROCEED bit to restart I2C-NL.
 * Else application callback return an error (no buffer)
 *   We need to cancel the transaction.
 *   !!! How to force I2C-NL to stop the transaction since the external
 *   !!! controller is generating clocks and START/STOP? !!!
 *   Corner case bug in this driver, we don't find a struct i2c_target_config?
 *   We could config TM DMA for memory-to-peripheral from the tm_rx_buf of length 1
 *   This points to the received target address.
 *   Also, set TM_CMD wrCnt = 1.
 */
static void target_read_config(const struct device *dev, uint8_t shad_addr)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct mec_i2c_smb_regs *regs = devcfg->regs;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct i2c_target_config *tcfg = i2c_target_find(dev, shad_addr);
	int ret = 0;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xB0u);

	data->tm_tx_buf = NULL;
	data->tm_tx_buf_sz = 0;
	data->tm_tx_buf_xfr_sz = 0;

	if (!tcfg) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xB1u);
		return;
	}

	const struct i2c_target_callbacks *cb = tcfg->callbacks;

	if (!cb) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xB2u);
		return;
	}

	if (cb->buf_read_requested) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xB3u);
		ret = cb->buf_read_requested(tcfg, &data->tm_tx_buf, &data->tm_tx_buf_sz);
		if (ret == 0) {
			data->tm_tx_buf_xfr_sz = data->tm_tx_buf_sz;
			if (data->tm_tx_buf_xfr_sz > I2C_MEC5_NL_MAX_XFR_LEN) {
				data->tm_tx_buf_xfr_sz = I2C_MEC5_NL_MAX_XFR_LEN;
			}

			I2C_NL_DEBUG_STATE_UPDATE(data, 0xB4u);
			i2c_mec5_nl_dma_start(dev, data->tm_tx_buf, data->tm_tx_buf_xfr_sz,
					      I2C_NL_DIR_WR,
					      (I2C_MEC5_NL_DMA_START_FLAG_TM
					       | I2C_MEC5_NL_DMA_START_FLAG_RUN));
			mec_hal_i2c_nl_tm_proceed(hwctx);
		} else { /* app did not supply a buffer, terminate transaction */
			/* Config DMA to read from one byte from driver tm_rx_buf.
			 * If external Controller continues generating clocks.
			 * I2C-NL HW (when TM wrCnt == 0) will driver SDA = 1
			 * It's up to external Controller to issue STOP then we
			 * will get IDLE interrupt when lines are detected idle.
			 */
			I2C_NL_DEBUG_STATE_UPDATE(data, 0xB5u);
			i2c_mec5_nl_dma_start(dev, data->tm_rx_buf, 1u, I2C_NL_DIR_WR,
					      (I2C_MEC5_NL_DMA_START_FLAG_TM
					       | I2C_MEC5_NL_DMA_START_FLAG_RUN));
			mec_hal_i2c_nl_tm_xfr_count_set(regs, 0, 1);
			mec_hal_i2c_nl_tm_proceed(hwctx);
		}
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xBFu);
}

static void target_stop_cb(const struct device *dev, uint8_t shad_addr)
{
#ifdef I2C_NL_DEBUG_STATE
	struct i2c_mec5_nl_data *const data = dev->data;
#endif
	struct i2c_target_config *tcfg = i2c_target_find(dev, shad_addr);

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xC0u);

	if (!tcfg) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xC1u);
		return;
	}

	const struct i2c_target_callbacks *cb = tcfg->callbacks;

	if (cb->stop) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xC2u);
		cb->stop(tcfg);
		/* DEBUG */
		memset(data->tm_rx_buf, 0x55, data->tm_rx_buf_sz);
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xCFu);
}

/* I2C-NL configured with maximum TM read and write counts.
 * TM DMA channel initially configured for peripheral-to-memory
 * because I2C write and read protocols both begin with a write
 * of the target address. If the target address matches one of the
 * two addresses in this I2C controller's OWN_ADDR register then
 * I2C-NL HW FSM will trigger DMA to read the address. TM DMA must
 * be configured for peripheral-to-memory from TM_RXB register to
 * the driver's tm_rx_buf. DMA is used to limit the data based on the
 * buffer size: driver tm_rx_buf size or buffer size supplied by
 * the application read requested callback.
 * NOTE 1: External Controller write N bytes of data
 * driver tm_rx_buf = target_addr, data0, data1, ... dataN-1
 * If N > buffer size, DMA will stop and I2C-NL HW FSM will NAK
 * NOTE 2: External Controller read N bytes of data
 * I2C-NL HW moves target_addr to tm_rx_buf
 * I2C-NL clock stretches and signals FW to change DMA direction
 * FW(ISR) invokes read requested callback to obtain buffer and length
 * FW(ISR) reconfigures TM DMA channel for memory-to-peripheral
 * using app buffer and length.
 * FW(ISR) signals I2C-NL to proceed.
 * I2C-NL HW FSM stops clock stretching
 * External Controller resumes issuing clocks on I2C SCL line.
 * I2C-NL HW FSM triggers TM DMA to supply data and puts it on SDA line.
 * NOTE 3: I2C-NL also stores the target address in its shadow address register.
 *
 * HW response to I2C protocols:
 * I2C-NL and DMA are armed for external Controller transmitting to Target.
 * NOTE: I2C-NL does not trigger unless address transmitted by external Controller
 * matches one of the target addresses in the OWN_ADDR register.
 *
 * I2C Write: [START] [wrAddr] ACK [optional Nw wrData bytes] ACK [STOP]
 *   I2C-NL triggers TM DMA channel which moves target address and
 *   optional data written by external Controller to driver tm_rx_buf.
 *   if DMA channel stops due to buffer full, or I2C-NL TM_CMD.rdCnt reaches 0,
 *   or external Controller generates STOP then I2C-NL clears TM_CMD.PROCEED and TM_CMD.RUN
 *   and generates TM_DONE interrupt.
 *   Handler invokes application target_buf_received callback.
 *   After callback re-arm I2C-NL and TM DMA channel.
 *
 * I2C Read: [START] [rdAddr] ACK rdData1 [ACK] ... rdDataN [NACK] [STOP]
 *  I2C-NL triggers TM DMA channel which moves target address to driver tm_rx_buf.
 *  I2C-NL then clock stretches, clears TM_CMD.PROCEED bit and fires TM_DONE interrupt
 *  TM handler calls registered buf_read_requested callback to get (buf, len).
 *    Re-configure TM DMA channel for mem2dev using (buf, len)
 *    Set TM_CMD.PROCEED bit to 1
 *  I2C-NL stops clock stretching and triggers TM DMA to supply data.
 *  External Controller generates clocks and I2C-NL puts data onto SDA line.
 *  This continues until External Controller NAK's last byte it wants and
 *  issues STOP. NOTE: if External Controller reads more bytes than were in
 *  buffer then TM DMA reaches buffer end and I2C-NL will supply 0xFF on SDA
 *  until external controller NAK's and issues STOP.
 *  On NAK and STOP I2C-NL will fire another TM_DONE interrupt with TM_CMD
 *  PROCEED:RUN=00b. ISR does not need to do anything except clear status
 *  and exit. NOTE: ISR has previously enabled IDLE interrupt.
 *  After bus goes idle (SCL/SDA are both high for 1/2 I2C clock), I2C-NL
 *  will fire the IDLE interrupt.
 *  TM Handler invokes registered STOP callback.
 *  TM Handler re-arms TM hardware and TM DMA for dev2mem.
 *
 * I2C Combined Write-Read:
 *   [START] [wrAddr] ACK [optional Nw wrData bytes] ACK [RPT-START] [rdAddr] [ACK] rdData1 ACK
 *   ... rdDataN [NACK] [STOP]
 *
 */
static uint32_t i2c_mec5_nl_tm_handler(const struct device *dev, bool bus_is_idle)
{
	/* const struct i2c_mec5_nl_config *const devcfg = dev->config; */
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t tev = 0, tm_cmd = 0, pr = 0;
	uint8_t shad_addr = 0, shad_data = 0;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x90u);

	tm_cmd = mec_hal_i2c_nl_cmd_get(hwctx, 1);
	shad_addr = mec_hal_i2c_nl_shad_addr_get(hwctx->base);
	shad_data = mec_hal_i2c_nl_shad_data_get(hwctx->base);

	/* tm_cmd[31:24] are rsvd 0. Store the captured address there */
	data->tm_cmd_addr = ((uint32_t)shad_addr << 24) | tm_cmd;

	if (bus_is_idle) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x97u);
		target_stop_cb(dev, shad_addr);
		i2c_mec5_nl_target_arm(dev);
	} else if (data->i2c_status & BIT(MEC_I2C_STS_TM_DONE_POS)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x91u);
		tev |= BIT(I2C_NL_KEV_TM_DONE_POS);
		/* safe to enable IDLE interrupt */
		mec_hal_i2c_smb_intr_ctrl(hwctx, BIT(MEC_I2C_IEN_IDLE_POS), 1);
		pr = tm_cmd & 0x03u; /* PROCEED:RUN bits */
		if (shad_addr & BIT(0)) { /* read from target (us) */
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x92u);
			if (pr == 0x01u) { /* turn-around */
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x93u);
				target_read_config(dev, shad_addr);
			} else if (pr == 0) {
				/* finished and exit. Wait for external Controller
				 * to generate STOP and bus got idle.
				 */
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x94u);
			}
		} else { /* write to target (us) */
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x98u);
			tm_write_cb(dev, shad_addr);
			if (pr == 0x01u) {
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x99u);
				mec_hal_i2c_nl_tm_proceed(hwctx);
			}
		}
	} else {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x9Eu);
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x9fu);
	return tev;
}
#endif

/* I2C-NL interrupt handler for both controller and target modes. */
static void i2c_mec5_nl_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t cm_cmd_msk = (BIT(MEC_I2C_SMB_CM_CMD_RUN_Pos)
				| BIT(MEC_I2C_SMB_CM_CMD_PROCEED_Pos));
	uint32_t cmd, events = 0;
	bool idle = false;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x80u);
	I2C_NL_DEBUG_ISR_COUNT_UPDATE(data);

	data->i2c_status = mec_hal_i2c_smb_status(hwctx, 1u);

	I2C_NL_DEBUG_ISR_DATA_UPDATE(data);

	if (mec_hal_i2c_smb_is_idle_ien(hwctx) && (data->i2c_status & BIT(MEC_I2C_STS_IDLE_POS))) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x81u);
		mec_hal_i2c_smb_intr_ctrl(hwctx, BIT(MEC_I2C_IEN_IDLE_POS), 0);
		idle = true;
		events |= BIT(I2C_NL_KEV_IDLE_POS);
	}

	if (data->i2c_status & BIT(MEC_I2C_STS_BERR_POS)) {
		events |= BIT(I2C_NL_KEV_BERR_POS);
	}
	if (data->i2c_status & BIT(MEC_I2C_STS_LAB_POS)) {
		events |= BIT(I2C_NL_KEV_LAB_ERR_POS);
	}
	if (data->i2c_status & BIT(MEC_I2C_STS_CM_TX_NACK_POS)) {
		events |= BIT(I2C_NL_KEV_CM_NAK_POS);
	}
	if (data->i2c_status & BIT(MEC_I2C_STS_CM_DONE_POS)) {
		events |= BIT(I2C_NL_KEV_CM_DONE_POS);
	}

	cmd = mec_hal_i2c_nl_cmd_get(hwctx, 0);
	/* Is HW FSM direction change from TX to RX? FW must reconfigure DMA channel for RX */
	if ((cmd & cm_cmd_msk) == BIT(MEC_I2C_SMB_CM_CMD_RUN_Pos)) {
		events |= BIT(I2C_NL_KEV_W2R_POS);
		i2c_mec5_nl_dma_start(dev, data->msgs[data->mgrp.rd_idx].buf, data->mgrp.rd_len,
				      I2C_NL_DIR_RD, (I2C_MEC5_NL_DMA_START_FLAG_CM
						      | I2C_MEC5_NL_DMA_START_FLAG_RUN));
		/* trigger I2C-NL to continue with read phase */
		mec_hal_i2c_nl_cm_proceed(hwctx);
	}

#ifdef CONFIG_I2C_TARGET
	if (i2c_targets_are_registered(dev)) {
		events |= i2c_mec5_nl_tm_handler(dev, idle);
	}
#endif

	if (events) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x8Eu);
		k_event_post(&data->events, events);
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x8Fu);
}

/* ---- Public API ---- */

static int i2c_mec5_nl_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_mec5_nl_data *data = dev->data;
	uint32_t speed = I2C_SPEED_GET(dev_config);
	int ret = 0;

	switch (speed) {
	case I2C_SPEED_STANDARD:
		data->clock_freq_hz = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		data->clock_freq_hz = KHZ(400);
		break;
	case I2C_SPEED_FAST_PLUS:
		data->clock_freq_hz = MHZ(1);
		break;
	default:
		return -EINVAL;
	}

	ret = i2c_mec5_nl_reset_config(dev);
	if (ret) {
		return ret;
	}

	/* wait for NBB=1, BER, LAB, or timeout */
	ret = wait_bus_free(dev, I2C_MEC5_NL_WAIT_COUNT);

	return ret;
}

static int i2c_mec5_nl_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_mec5_nl_data *data = dev->data;
	uint32_t bus_freq_hz = 0, cfg = 0;
	int ret = 0;

	if (!dev_config) {
		return -EINVAL;
	}

	ret = mec_hal_i2c_smb_bus_freq_get(&data->ctx, &bus_freq_hz);
	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	ret = 0;

	if (!(data->misc_cfg & BIT(7))) {
		cfg |= I2C_MODE_CONTROLLER;
	}

	if ((bus_freq_hz >= KHZ(90)) && (bus_freq_hz <= KHZ(110))) {
		cfg |= I2C_SPEED_SET(I2C_SPEED_STANDARD);
	} else if ((bus_freq_hz >= KHZ(340)) && (bus_freq_hz <= KHZ(440))) {
		cfg |= I2C_SPEED_SET(I2C_SPEED_FAST);
	} else if ((bus_freq_hz >= KHZ(900)) && (bus_freq_hz <= KHZ(1100))) {
		cfg |= I2C_SPEED_SET(I2C_SPEED_FAST_PLUS);
	} else {
		ret = -ERANGE;
	}

	*dev_config = cfg;

	return ret;
}

static void i2c_mec5_nl_xfr_data_init(const struct device *dev, struct i2c_msg *msgs,
				      uint8_t num_msgs, uint16_t addr)
{
	struct i2c_mec5_nl_data *const data = dev->data;

	memset(&data->mgrp, 0, sizeof(struct i2c_msg_group));

	data->cm_target_i2c_wr_addr = fmt_addr(addr, I2C_NL_DIR_WR);
	data->state = I2C_NL_STATE_OPEN;
	data->num_msgs = num_msgs;
	data->msgidx = 0;
	data->msgs = msgs;
}

static int i2c_mec5_nl_transfer(const struct device *dev, struct i2c_msg *msgs,
				uint8_t num_msgs, uint16_t addr)
{
	struct i2c_mec5_nl_data *const data = dev->data;
	int ret = 0;

	if (!msgs && !num_msgs) {
		return 0; /* Nothing to do */
	}

	if (!msgs || !num_msgs) {
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER); /* decrements count */
	k_event_clear(&data->events, UINT32_MAX);

	I2C_NL_DEBUG_ISR_INIT(data);
	I2C_NL_DEBUG_STATE_INIT(data);
	I2C_NL_DEBUG_STATE_UPDATE(data, 1u);

	i2c_mec5_nl_xfr_data_init(dev, msgs, num_msgs, addr);

	ret = process_i2c_msgs(dev);

	I2C_NL_DEBUG_STATE_UPDATE(data, 2u);

	data->state = I2C_NL_STATE_CLOSED;
	k_sem_give(&data->lock);

	return ret;
}

#ifdef CONFIG_I2C_TARGET

/* I2C-NL supports up to two 7-bit target addresses */
static int i2c_mec5_nl_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
	/* const struct i2c_mec5_nl_config *const devcfg = dev->config; */
	struct i2c_mec5_nl_data *const data = dev->data;
	int ret = 0;
	uint8_t targ_addr = 0;

	if (!cfg || (cfg->address > 0x7fu)) {
		return -EINVAL;
	}

	if (k_sem_take(&data->lock, K_NO_WAIT) != 0) {
		return -EBUSY;
	}

	targ_addr = (uint8_t)(cfg->address & 0x7fu);

	if (!data->target1_cfg && !data->target2_cfg) {
		I2C_NL_DEBUG_ISR_INIT(data);
		I2C_NL_DEBUG_STATE_INIT(data);
		data->target1_cfg = cfg;
		mec_hal_i2c_smb_set_target_addr(&data->ctx, 0, targ_addr);
		ret = i2c_mec5_nl_target_arm(dev);
	} else if (!data->target1_cfg) {
		data->target1_cfg = cfg;
		mec_hal_i2c_smb_set_target_addr(&data->ctx, 0, targ_addr);
	} else if (!data->target2_cfg) {
		data->target2_cfg = cfg;
		mec_hal_i2c_smb_set_target_addr(&data->ctx, 1, targ_addr);
	} else {
		ret = -EAGAIN;
	}

	k_sem_give(&data->lock);

	return ret;
}

static int i2c_mec5_nl_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;

	if (k_sem_take(&data->lock, K_NO_WAIT) != 0) {
		return -EBUSY;
	}

	if (data->target1_cfg == cfg) {
		data->target1_cfg = NULL;
		mec_hal_i2c_smb_set_target_addr(&data->ctx, 0, 0);

	} else if (data->target2_cfg == cfg) {
		data->target2_cfg = NULL;
		mec_hal_i2c_smb_set_target_addr(&data->ctx, 1, 0);
	}

	if (!data->target1_cfg && !data->target2_cfg) {
		mec_hal_i2c_smb_intr_ctrl(&data->ctx, BIT(MEC_I2C_NL_IEN_TM_DONE_POS), 0);
		mec_hal_i2c_nl_cmd_clear(&data->ctx, 1);
		mec_hal_i2c_smb_status(&data->ctx, 1);
		dma_stop(devcfg->dma_dev, devcfg->tm_dma_chan);
	}

	k_sem_give(&data->lock);

	return 0;
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
#endif /* CONFIG_I2C_TARGET */

static const struct i2c_driver_api i2c_mec5_nl_driver_api = {
	.configure = i2c_mec5_nl_configure,
	.get_config = i2c_mec5_nl_get_config,
	.transfer = i2c_mec5_nl_transfer,
	.target_register = i2c_mec5_nl_target_register,
	.target_unregister = i2c_mec5_nl_target_unregister,
};
/* end public API */

static int i2c_mec5_nl_init_dma(const struct device *dev)
{
	const struct i2c_mec5_nl_config *devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct dma_config *dcfg = &data->dma_cfg;
	struct dma_block_config *dblk = &data->dma_blk_cfg;

	memset(dcfg, 0, sizeof(struct dma_config));
	memset(dblk, 0, sizeof(struct dma_block_config));

	if (!device_is_ready(devcfg->dma_dev)) {
		LOG_ERR("MEC5 Central DMA device not ready");
		return -EIO;
	}

	dcfg->dma_slot = devcfg->cm_dma_trigsrc;
	dcfg->channel_direction = MEMORY_TO_PERIPHERAL;
	dcfg->source_data_size = 1;
	dcfg->dest_data_size = 1;
	dcfg->block_count = 1;
	dcfg->head_block = dblk;
	dcfg->user_data = (void *)data;
	dcfg->dma_callback = i2c_mec5_nl_cm_dma_cb;

	return 0;
}

static int i2c_mec5_nl_init(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t bitrate_cfg = 0;
	int ret = 0;

	k_sem_init(&data->lock, 1, 1);
	k_event_init(&data->events);

	data->devcfg = devcfg;
	data->i2c_status = 0;
	data->xdone = 0;
	hwctx->base = devcfg->regs;
	hwctx->i2c_ctrl_cached = 0;

	ret = i2c_mec5_nl_init_dma(dev);
	if (ret) {
		return ret;
	}

	ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("pinctrl setup failed (%d)", ret);
		return ret;
	}

	bitrate_cfg = i2c_map_dt_bitrate(data->clock_freq_hz);
	if (!bitrate_cfg) {
		return -EINVAL;
	}

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

#define I2C_MEC5_NL_DT_INST_DMA_CTLR(i, name)					\
	DT_INST_DMAS_CTLR_BY_NAME(i, name)

#define I2C_MEC5_NL_DT_INST_DMA_DEV(i, name)					\
	DEVICE_DT_GET(I2C_MEC5_NL_DT_INST_DMA_CTLR(i, name))

#define I2C_MEC5_NL_DT_MISC_CFG(i)						\
	(uint8_t)(DT_INST_PROP_OR(i, port_sel, 0) & 0x0f)

#define I2C_NL_MEC5_DMA_CHAN(i, idx) \
	DT_INST_PHA_BY_IDX(i, dmas, idx, channel)

#define I2C_NL_MEC5_DMA_TRIGSRC(i, idx) \
	DT_INST_PHA_BY_IDX(i, dmas, idx, trigsrc)

#define I2C_NL_MEC5_CM_TX_BUF_SIZE(i) \
	(DT_INST_PROP(i, cm_tx_buf_size) + I2C_MEC5_NL_CM_TX_BUF_PAD_LEN)

#define I2C_NL_MEC5_CM_TX_BUF(i) \
	static uint8_t i2c_mec5_nl_cm_tx_buf_##i[I2C_NL_MEC5_CM_TX_BUF_SIZE(i)] __aligned(4);

#ifdef CONFIG_I2C_TARGET

BUILD_ASSERT(IS_ENABLED(CONFIG_I2C_TARGET_BUFFER_MODE),
	     "Target buffer mode must be used when Target mode enabled in the build!");

#define I2C_MEC5_NL_TM_DMA(i)							\
	.tm_dma_chan = I2C_NL_MEC5_DMA_CHAN(i, 1),				\
	.tm_dma_trigsrc = I2C_NL_MEC5_DMA_TRIGSRC(i, 1),

#define I2C_NL_MEC5_TM_RX_BUF_SIZE(i) \
	(DT_INST_PROP(i, tm_rx_buf_size) + I2C_MEC5_NL_TM_RX_BUF_PAD_LEN)

#define I2C_NL_MEC5_TM_RX_BUF(i) \
	static uint8_t i2c_mec5_nl_tm_rx_buf_##i[I2C_NL_MEC5_TM_RX_BUF_SIZE(i)] __aligned(4);

#define I2C_NL_MEC5_TM_RX_BUF_PTR(i)						\
	.tm_rx_buf = &i2c_mec5_nl_tm_rx_buf_##i[0],				\
	.tm_rx_buf_sz = I2C_NL_MEC5_TM_RX_BUF_SIZE(i),

#else
#define I2C_NL_MEC5_TM_RX_BUF(i)
#define I2C_NL_MEC5_TM_RX_BUF_PTR(i)
#define I2C_MEC5_NL_TM_DMA(i)
#endif

#define I2C_MEC5_NL_DEVICE(i)							\
	I2C_NL_MEC5_CM_TX_BUF(i)						\
	I2C_NL_MEC5_TM_RX_BUF(i)						\
	struct i2c_mec5_nl_data i2c_mec5_nl_data_##i = {			\
		.clock_freq_hz = DT_INST_PROP(i, clock_frequency),		\
		.misc_cfg = I2C_MEC5_NL_DT_MISC_CFG(i),				\
		.xfrbuf = &i2c_mec5_nl_cm_tx_buf_##i[0],			\
		I2C_NL_MEC5_TM_RX_BUF_PTR(i)					\
	};									\
	PINCTRL_DT_INST_DEFINE(i);						\
	static void i2c_mec5_nl_irq_config_func_##i(void)			\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(i),					\
			    DT_INST_IRQ(i, priority),				\
			    i2c_mec5_nl_isr,					\
			    DEVICE_DT_INST_GET(i), 0);				\
		irq_enable(DT_INST_IRQN(i));					\
	}									\
	struct i2c_mec5_nl_config i2c_mec5_nl_devcfg_##i = {			\
		.regs = (struct mec_i2c_smb_regs *)DT_INST_REG_ADDR(i),		\
		.init_pin_wait_us = DT_INST_PROP_OR(i, init_pin_wait, 100),	\
		.cfg_pin_wait_us = DT_INST_PROP_OR(i, config_pin_wait, 35000),	\
		.sda_gpio = GPIO_DT_SPEC_INST_GET(i, sda_gpios),		\
		.scl_gpio = GPIO_DT_SPEC_INST_GET(i, scl_gpios),		\
		.irq_config_func = i2c_mec5_nl_irq_config_func_##i,		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),			\
		.dma_dev = I2C_MEC5_NL_DT_INST_DMA_DEV(i, cm),			\
		.cm_dma_chan = I2C_NL_MEC5_DMA_CHAN(i, 0),			\
		.cm_dma_trigsrc = I2C_NL_MEC5_DMA_TRIGSRC(i, 0),		\
		I2C_MEC5_NL_TM_DMA(i)						\
	};									\
	I2C_DEVICE_DT_INST_DEFINE(i,						\
				  i2c_mec5_nl_init,				\
				  NULL,						\
				  &i2c_mec5_nl_data_##i,			\
				  &i2c_mec5_nl_devcfg_##i,			\
				  POST_KERNEL,					\
				  CONFIG_I2C_INIT_PRIORITY,			\
				  &i2c_mec5_nl_driver_api)

DT_INST_FOREACH_STATUS_OKAY(I2C_MEC5_NL_DEVICE)
