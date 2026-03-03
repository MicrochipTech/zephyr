/*
 * Copyright (c) 2026 Microchip Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_i2c_nl

#include <soc.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_xec_i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(i2c_xec_nl, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h" /* dependency on logging */
#include "i2c_mchp_xec_common.h"
#include "i2c_mchp_xec_regs.h"

BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(dmac)),
             "Central DMA controller, dmac, not enabled!");

#ifdef CONFIG_I2C_TARGET
BUILD_ASSERT(CONFIG_I2C_TARGET_BUFFER_MODE, "XEC I2C-NL requires I2C target buffer mode!");
#endif

#define XEC_I2C_NL_XFR_BUF_SZ_DFLT 32U
#define XEC_I2C_NL_XFR_BUF_PAD_SZ  4U

#define XEC_I2C_NL_XFR_CNT_MAX 0xFFFCU

#define XEC_I2C_NL_MODE_CM 0
#define XEC_I2C_NL_MODE_TM 1

#define XEC_I2C_NL_DIR_NONE 0
#define XEC_I2C_NL_DIR_WR   1U
#define XEC_I2C_NL_DIR_RD   2U

#define XEC_I2C_NL_MAX_TARGETS 2

enum i2c_xec_nl_errors {
	XEC_I2C_NL_ERR_NONE = 0,
	XEC_I2C_NL_ERR_DMA_CFG,
	XEC_I2C_NL_ERR_CM_NAK,
	XEC_I2C_NL_ERR_LOST_ARB,
	XEC_I2C_NL_ERR_BUS,
	XEC_I2C_NL_ERR_TM_RX_NAK,
	XEC_I2C_NL_ERR_TM_TPROT,
	XEC_I2C_NL_ERR_MAX,
};

struct i2c_xec_nl_drv_cfg {
	mm_reg_t regbase;
	uint32_t xfr_bufsz;
	uint8_t *xfrbuf;
	const struct device *cm_dma_dev;
	const struct device *tm_dma_dev;
	uint8_t cm_dma_chan;
	uint8_t cm_dma_trigsrc;
	uint8_t tm_dma_chan;
	uint8_t tm_dma_trigsrc;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t enc_scr;
	uint8_t port;
	uint32_t clk_freq;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
	struct gpio_dt_spec sda_gpio;
	struct gpio_dt_spec scl_gpio;
};

#define I2C_XEC_NL_XFR_STATE_START     0
#define I2C_XEC_NL_XFR_STATE_WR_MSGS   1U
#define I2C_XEC_NL_XFR_STATE_RPT_START 2U
#define I2C_XEC_NL_XFR_STATE_PAUSE     3U
#define I2C_XEC_NL_XFR_STATE_RD_MSGS   4U
#define I2C_XEC_NL_XFR_STATE_DONE      5U

struct i2c_xec_nl_xfr {
	uint16_t wrlen;
	uint16_t rdlen;
	uint8_t wr_addr;
	uint8_t wridx;
	uint8_t nwr;
	uint8_t xerr;
	uint8_t rd_addr;
	uint8_t rdidx;
	uint8_t nrd;
	uint8_t state;
};

struct i2c_xec_nl_drv_data {
	volatile uint32_t i2c_compl;
	volatile uint8_t i2c_sr;
	volatile uint8_t i2c_cr;
	uint8_t port;
	uint8_t addr;
	uint32_t dev_config;
	const struct device *dev;
	struct k_spinlock spin_lock;
	struct k_mutex lock_mut;
	struct k_sem sync_sem;
	struct i2c_xec_nl_xfr xfr;
	struct i2c_msg *msgs;
	uint8_t num_msgs;
	uint8_t msg_idx;
	uint32_t cmd;
	uint32_t extlen;
	int dma_cb_sts;
	struct dma_status dma_chan_sts;
	struct dma_config dma_chan_cfg;
	struct dma_block_config tx_dma_blk_cfg;
	struct dma_block_config rx_dma_blk_cfg;
#ifdef CONFIG_I2C_CALLBACK
	i2c_callback_t async_cb;
	void *async_cb_ud;
#endif
#ifdef CONFIG_I2C_TARGET
	struct i2c_target_config *target_cfgs[XEC_I2C_NL_MAX_TARGETS];
	uint8_t targ_bitmap;
#endif
	uint32_t temp_buf;
#ifdef CONFIG_I2C_XEC_NL_TRACE_DEBUG
	volatile uint32_t dbg_idx;
	volatile uint8_t dbg_tp[CONFIG_I2C_XEC_NL_TRACE_DEBUG_BUF_SIZE];
#endif
};

#ifdef CONFIG_I2C_XEC_NL_TRACE_DEBUG

static void dbg_tp_clear(struct i2c_xec_nl_drv_data *data)
{
	data->dbg_idx = 0;
	memset((void *)data->dbg_tp, 0, CONFIG_I2C_XEC_NL_TRACE_DEBUG_BUF_SIZE);
}

static void dbg_tp_update(struct i2c_xec_nl_drv_data *data, uint8_t val)
{
	uint32_t idx = data->dbg_idx;

	if (idx < CONFIG_I2C_XEC_NL_TRACE_DEBUG_BUF_SIZE) {
		data->dbg_tp[idx] = val;
		data->dbg_idx++;
	}
}
#else
static void dbg_tp_clear(struct i2c_xec_nl_drv_data *data) {}
static void dbg_tp_update(struct i2c_xec_nl_drv_data *data, uint8_t val) {}
#endif

/* I2C control register is write-only. Save what we write */
static void i2c_xec_nl_cr_wr(const struct device *dev, uint8_t cr_val)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	const struct i2c_xec_nl_drv_cfg *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->regbase;

	data->i2c_cr = cr_val;
	sys_write8(cr_val, rb + XEC_I2C_CR_OFS);
}

#ifdef CONFIG_I2C_TARGET
static void prog_own_addrs(const struct device *dev)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	const struct i2c_xec_nl_drv_cfg *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->regbase;
	uint32_t oarv = 0;

	for (uint8_t i = 0; i < XEC_I2C_NL_MAX_TARGETS; i++) {
		struct i2c_target_config *tcfg = data->target_cfgs[i];

		if (tcfg != NULL) {
			oarv |= XEC_I2C_OA_SET(i, (uint32_t)tcfg->address);
			data->targ_bitmap |= BIT(i);
		} else {
			data->targ_bitmap &= ~BIT(i);
		}
	}

	sys_write32(oarv, rb + XEC_I2C_OA_OFS);
}

static bool targets_registered(const struct device *dev)
{
	const struct i2c_xec_nl_drv_cfg *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->regbase;

	if (sys_read32(rb + XEC_I2C_OA_OFS) != 0) {
		return true;
	}

	return false;
}
#else
static bool targets_registered(const struct device *dev)
{
	return false;
}
#endif /* CONFIG_I2C_TARGET */

static int i2c_xec_nl_cfg(const struct device *dev, uint32_t dev_config)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	const struct i2c_xec_nl_drv_cfg *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->regbase;
	uint32_t cfgval = 0, cfgmsk = 0, freq_hz = 0;
	int rc = 0;
	uint8_t crval = 0;

	soc_xec_pcr_sleep_en_clear(drvcfg->enc_scr);
	soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 0);
	i2c_xec_ctrl_reset(rb, data->port, drvcfg->enc_scr);
	soc_ecia_girq_status_clear(drvcfg->girq, drvcfg->girq_pos);

	/* PIN=1 clears internal service status, ESO=0 disabled serial interface */
	crval = BIT(XEC_I2C_CR_PIN_POS);
	i2c_xec_nl_cr_wr(dev, crval);

#ifdef CONFIG_I2C_TARGET
	prog_own_addrs(dev);
#endif

	freq_hz = i2c_xec_freq_from_dev_config(dev_config);
	if (freq_hz == 0) {
		LOG_ERR("XEC I2C-NL unsupported I2C freq");
		return -EINVAL;
	}

	rc = i2c_xec_prog_timing(rb, freq_hz);
	if (rc != 0) {
		LOG_ERR("No HW timing for frequency %u", freq_hz);
		return rc;
	}

	data->dev_config = dev_config;

	/* Set port mux, enable filter, and disable I2C general call target response.
	 *  Must be done before enabling the controller.
	 */
	cfgmsk = (BIT(XEC_I2C_CFG_PORT_MSK) | BIT(XEC_I2C_CFG_FEN_POS) |
	          BIT(XEC_I2C_CFG_GC_DIS_POS));
	cfgval = (XEC_I2C_CFG_PORT_SET((uint32_t)data->port) | BIT(XEC_I2C_CFG_FEN_POS) |
	          BIT(XEC_I2C_CFG_GC_DIS_POS));

	soc_mmcr_mask_set(rb + XEC_I2C_CFG_OFS, cfgval, cfgmsk);

	/* Allow PIN assertions, enable serial interface, enable auto-acknowlegment */
	crval = BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS);
	i2c_xec_nl_cr_wr(dev, crval);

	/* enable controller */
	sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);

	return 0;
}

static int i2c_xec_nl_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	int rc = 0;

	rc = k_mutex_lock(&data->lock_mut, K_NO_WAIT);
	if (rc != 0) {
		return -EBUSY;
	}

	if (targets_registered(dev) == false) {
		rc = i2c_xec_nl_cfg(dev, dev_config);
	} else {
		rc = -EADDRINUSE;
	}

	k_mutex_unlock(&data->lock_mut);

	return rc;
}

static int i2c_xec_nl_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	int rc = 0;

	if (dev_config == NULL) {
		return -EINVAL;
	}

	rc = k_mutex_lock(&data->lock_mut, K_NO_WAIT);
	if (rc != 0) {
		return -EBUSY;
	}

	*dev_config = data->dev_config;

	k_mutex_unlock(&data->lock_mut);

	return rc;
}

static int xec_i2c_nl_recover_bus(const struct device *dev)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	const struct i2c_xec_nl_drv_cfg *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->regbase;
	uint32_t dev_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
	int rc = 0;

	i2c_xec_nl_cfg(dev, dev_config);
	k_sleep(K_MSEC(35));

	if (sys_read8(rb + XEC_I2C_SR_OFS) !=
	    (BIT(XEC_I2C_SR_PIN_POS) | BIT(XEC_I2C_SR_NBB_POS))) {
		rc = i2c_xec_bus_recovery(rb);
		if (rc != 0) {
			return rc;
		}
	}

	/* reconfig for previous speed settings */
	rc = i2c_xec_nl_cfg(dev, data->dev_config);
	k_sleep(K_MSEC(35));

	return rc;
}

static int check_msgs(struct i2c_msg *msgs, uint8_t num_msgs)
{
	uint32_t rxlen = 0, txlen = 0;

	for (uint8_t n = 0u; n < num_msgs; n++) {
		struct i2c_msg *m = &msgs[n];
		struct i2c_msg *mn = NULL;

		if ((n + 1u) < num_msgs) {
			mn = m + 1u;
		}

		if ((m->buf == NULL) || (m->len == 0)) {
			LOG_ERR("I2C msg[%u] has no buffer and/or length", n);
			return -EINVAL;
		}

		if ((m->flags & I2C_MSG_ADDR_10_BITS) != 0) {
			LOG_ERR("I2C msg[%u] 10-bit addr not supported by HW", n);
			return -EINVAL;
		}

		if (m->len > XEC_I2C_NL_MAX_LEN) {
			LOG_ERR("I2C msg[%u] too large", n);
			return -E2BIG;
		}

		if ((m->flags & I2C_MSG_READ) != 0) {
			rxlen += m->len;
			if (rxlen > XEC_I2C_NL_MAX_LEN) {
				LOG_ERR("I2C msg[%u] exceeded rx max len", n);
				return -E2BIG;
			}
		} else {
			txlen += m->len;
			if (txlen > XEC_I2C_NL_MAX_LEN) {
				LOG_ERR("I2C msg[%u] exceeded tx max len", n);
				return -E2BIG;
			}
		}

		if (mn == NULL) {
			m->flags |= I2C_MSG_STOP;
		} else {
			if ((m->flags & I2C_MSG_READ) == 0) {
				if ((mn->flags & I2C_MSG_READ) != 0) {
					/* write-to-read */
					mn->flags |= I2C_MSG_RESTART;
				}
			} else { /* m is read message */
				if ((mn->flags & I2C_MSG_READ) == 0) { /* next is write msg */
					m->flags |= I2C_MSG_STOP;
				}
			}
		}

		if ((m->flags & I2C_MSG_STOP) != 0) {
			rxlen = 0;
			txlen = 0;
		}
	}

	return 0;
}

/* broken for two messages:
 * msg[0] = I2C_MSG_WRITE len=2
 * msg[1] = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP len=4
 * produces group with on write message len=2
 * BUG: we are not adding +1 to wrlen for START and +1 for RPT-START!
 */
static int build_msg_group(const struct device *dev)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	struct i2c_xec_nl_xfr *xfr = &data->xfr;
	uint32_t wrlen = 0, rdlen = 0;
	uint8_t cdir = XEC_I2C_NL_DIR_NONE, pdir = XEC_I2C_NL_DIR_NONE;

	xfr->wrlen = 0;
	xfr->rdlen = 0;
	xfr->nwr = 0;
	xfr->xerr = 0;
	xfr->nrd = 0;
	xfr->state = I2C_XEC_NL_XFR_STATE_DONE;
	xfr->wr_addr = (data->addr & 0x7FU) << 1;
	xfr->rd_addr = xfr->wr_addr | BIT(0);

	for (uint8_t i = data->msg_idx; i < data->num_msgs; i++) {
		const struct i2c_msg *m = &data->msgs[i];

		if ((m->flags & I2C_MSG_READ) != 0) {
			cdir = XEC_I2C_NL_DIR_RD;
		} else {
			cdir = XEC_I2C_NL_DIR_WR;
		}

		/* Driver does not support RPT-START with same direction */
		if ((pdir == cdir) && ((m->flags & I2C_MSG_RESTART) != 0)) {
			break;
		}

		if (cdir == XEC_I2C_NL_DIR_RD) {
			if (xfr->nrd == 0) {
				xfr->rdidx = i;
			}
			xfr->nrd++;
			rdlen += m->len;
		} else { /* write message */
			if (pdir == XEC_I2C_NL_DIR_RD) {
				/* direction change read to write not allowed */
				break;
			}
			if (xfr->nwr == 0) {
				xfr->wridx = i;
			}
			xfr->nwr++;
			wrlen += m->len;
		}

		if ((m->flags & I2C_MSG_STOP) != 0) {
			break;
		}

		pdir = cdir;
	}

	if (xfr->nwr != 0) {
		xfr->state = I2C_XEC_NL_XFR_STATE_WR_MSGS;
	} else {
		xfr->state = I2C_XEC_NL_XFR_STATE_RD_MSGS;
	}

	wrlen++; /* Add one for START target address */
	if ((wrlen != 0) && (rdlen != 0)) {
		wrlen++; /* Add one for RPT-START target address */
	}

	if ((wrlen > XEC_I2C_NL_XFR_CNT_MAX) || (rdlen > XEC_I2C_NL_XFR_CNT_MAX)) {
		return -E2BIG;
	}

	xfr->wrlen = (uint16_t)wrlen;
	xfr->rdlen = (uint16_t)rdlen;

	return 0;
}

static void xec_nl_dma_cb(const struct device *dma_dev, void *user_data, uint32_t chan, int status)
{
	struct i2c_xec_nl_drv_data *const data = (struct i2c_xec_nl_drv_data *)user_data;
	const struct device *i2c_nl_dev = data->dev;
	const struct i2c_xec_nl_drv_cfg *i2c_drvcfg = i2c_nl_dev->config;
	struct i2c_xec_nl_xfr *xfr = &data->xfr;
	struct dma_status *dsts = &data->dma_chan_sts;
	uint32_t src = 0, dst = 0, size = 0;
	bool start_dma = false;

	dbg_tp_update(data, 0xD0U);

	data->dma_cb_sts = status;
	dma_get_status(dma_dev, chan, dsts);

/* 	k_spinlock_key_t key = k_spin_lock(&data->spin_lock); */

	dbg_tp_update(data, 0xD1U);

	if (xfr->state == I2C_XEC_NL_XFR_STATE_WR_MSGS) {
		dbg_tp_update(data, 0xD2U);
		if (xfr->nwr > 0) {
			dbg_tp_update(data, 0xD3U);
			dst = (uint32_t)(i2c_drvcfg->regbase + XEC_I2C_HTX_OFS);
			src = (uint32_t)data->msgs[xfr->wridx].buf;
			size = (uint32_t)data->msgs[xfr->wridx].len;
			xfr->wridx++;
			xfr->nwr--;
			start_dma = true;
			if (xfr->nwr == 0) {
				dbg_tp_update(data, 0xD4U);
				xfr->state = I2C_XEC_NL_XFR_STATE_DONE;
				if (xfr->nrd != 0) {
					dbg_tp_update(data, 0xD5U);
					xfr->state = I2C_XEC_NL_XFR_STATE_RPT_START;
				}
			}
		}
	} else if (xfr->state == I2C_XEC_NL_XFR_STATE_RPT_START) {
		dbg_tp_update(data, 0xD8U);
		dst = (uint32_t)(i2c_drvcfg->regbase + XEC_I2C_HTX_OFS);
		src = (uint32_t)&xfr->rd_addr;
		size = 1U;
		start_dma = true;
		xfr->state = I2C_XEC_NL_XFR_STATE_PAUSE;
	} else if (xfr->state == I2C_XEC_NL_XFR_STATE_RD_MSGS) {
		dbg_tp_update(data, 0xD9U);
		if (xfr->nrd > 0) {
			dbg_tp_update(data, 0xDAU);
			src = (uint32_t)(i2c_drvcfg->regbase + XEC_I2C_HRX_OFS);
			dst = (uint32_t)data->msgs[xfr->rdidx].buf;
			size = (uint32_t)data->msgs[xfr->rdidx].len;
			xfr->rdidx++;
			xfr->nrd--;
			start_dma = true;
		} else {
			dbg_tp_update(data, 0xDBU);
			xfr->state = I2C_XEC_NL_XFR_STATE_DONE;
		}
	} else {
		dbg_tp_update(data, 0xDCU);
		xfr->state = I2C_XEC_NL_XFR_STATE_DONE;
	}

/*	k_spin_unlock(&data->spin_lock, key); */

	if (start_dma == true) {
		dbg_tp_update(data, 0xDDU);
		dma_reload(dma_dev, chan, src, dst, size);
		dbg_tp_update(data, 0xDEU);
		dma_start(dma_dev, chan);
	}

	dbg_tp_update(data, 0xDFU);
}

#if 1 /* XEC_I2C_NL_DUAL_DMA */
static int xec_nl_dma_config(const struct device *dev, uint8_t chan, uint8_t mode,
                             enum dma_channel_direction dir)
{
	const struct i2c_xec_nl_drv_cfg *drvcfg = dev->config;
	struct i2c_xec_nl_drv_data *const data = dev->data;
	struct dma_config *dcfg = &data->dma_chan_cfg;
	const struct device *dma_dev = drvcfg->cm_dma_dev;
	struct dma_block_config *dblk = NULL;
	uint32_t regofs = 0, slot = 0;

	slot = drvcfg->cm_dma_trigsrc;
	if (mode != XEC_I2C_NL_MODE_CM) {
		dma_dev = drvcfg->tm_dma_dev;
		slot = drvcfg->tm_dma_trigsrc;
	}

	if (dir == MEMORY_TO_PERIPHERAL) {
		regofs = XEC_I2C_HTX_OFS;
		if (mode != XEC_I2C_NL_MODE_CM) {
			regofs = XEC_I2C_TTX_OFS;
		}

		dblk = &data->tx_dma_blk_cfg;
		dblk->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dblk->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dblk->dest_address = drvcfg->regbase;
		dblk->dest_address = drvcfg->regbase + regofs;
		dblk->source_address = (uint32_t)&data->temp_buf;
	} else {
		regofs = XEC_I2C_HRX_OFS;
		if (mode != XEC_I2C_NL_MODE_CM) {
			regofs = XEC_I2C_TRX_OFS;
		}

		dblk = &data->rx_dma_blk_cfg;
		dblk->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dblk->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dblk->source_address = drvcfg->regbase;
		dblk->source_address = drvcfg->regbase + regofs;
		dblk->dest_address = (uint32_t)&data->temp_buf;
	}

	dblk->block_size = 1U;
	dblk->next_block = NULL;

	dcfg->dma_slot = slot;
	dcfg->channel_direction = dir;
	dcfg->source_data_size = 1U;
	dcfg->dest_data_size = 1U;
	dcfg->block_count = 1U;
	dcfg->head_block = dblk;
	dcfg->user_data = (void *)data;
	dcfg->dma_callback = xec_nl_dma_cb;

	return dma_config(dma_dev, chan, dcfg);
}
#else
static int xec_nl_dma_chan_config(const struct device *dev, enum dma_channel_direction dir,
                                  uint8_t trigsrc)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	struct dma_config *dcfg = &data->dma_chan_cfg;

	dbg_tp_update(data, 0x20U);

	if ((dir != MEMORY_TO_PERIPHERAL) && (dir != PERIPHERAL_TO_MEMORY)) {
		dbg_tp_update(0x21U);
		LOG_ERR("Bad DMA direction!");
		return -EINVAL;
	}

	/* How well does the compiler optimize these bitfield accesses since most are 0 */
	dcfg->dma_slot = trigsrc;
	dcfg->channel_direction = dir;
	dcfg->half_complete_callback_en = 0; /* disabled */
	dcfg->complete_callback_en = 0; /* callback at list completion */
	dcfg->error_callback_dis = 0; /* allow callback on error */
	dcfg->source_handshake = 0; /* HW */
	dcfg->dest_handshake = 0; /* HW */
	dcfg->channel_priority = 0; /* NA */
	dcfg->source_chaining_en = 0; /* NA */
	dcfg->dest_chaining_en = 0; /* NA */
	dcfg->linked_channel = 0; /* NA */
	dcfg->cyclic = 0; /* NA */
	dcfg->source_data_size = 1U;
	dcfg->dest_data_size = 1U;
	dcfg->block_count = 1U;
	dcfg->head_block = &data->dma_blk_cfg;
	dcfg->user_data = (void *)data;
	dcfg->dma_callback = xec_nl_dma_cb;

	dbg_tp_update(data, 0x23U);

	return 0;
}

static int xec_nl_dma_block_config(const struct device *dev, enum dma_channel_direction dir,
                                   uint8_t trigsrc, uint8_t *buf, uint32_t bufsz)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	const struct i2c_xec_nl_drv_cfg *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->regbase;
	struct dma_block_config *blkcfg = &data->dma_blk_cfg;
	uint32_t src_addr = 0, dest_addr = 0;
	uint16_t src_addr_adj = 0, dest_addr_adj = 0;

	dbg_tp_update(data, 0x30U);

	if (dir == MEMORY_TO_PERIPHERAL) {
		dbg_tp_update(data, 0x31U);
		src_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		src_addr = (uint32_t)buf;
		dest_addr = (uint32_t)rb;
		if (trigsrc == drvcfg->cm_dma_trigsrc) {
			dbg_tp_update(data, 0x32U);
			dest_addr += XEC_I2C_HTX_OFS;
		} else {
			dbg_tp_update(data, 0x33U);
			dest_addr += XEC_I2C_TTX_OFS;
		}
	} else if (dir == PERIPHERAL_TO_MEMORY) {
		dbg_tp_update(data, 0x34U);
		src_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dest_addr = (uint32_t)buf;
		src_addr = (uint32_t)rb;
		if (trigsrc == drvcfg->cm_dma_trigsrc) {
			dbg_tp_update(data, 0x35U);
			src_addr += XEC_I2C_HRX_OFS;
		} else {
			dbg_tp_update(data, 0x36U);
			src_addr += XEC_I2C_TRX_OFS;
		}
	} else {
		dbg_tp_update(data, 0x37U);
		return -EINVAL;
	}

	blkcfg->source_address = src_addr;
	blkcfg->dest_address = dest_addr;
	blkcfg->source_gather_interval = 0; /* NA */
	blkcfg->dest_scatter_interval = 0; /* NA */
	blkcfg->dest_scatter_count = 0; /* NA */
	blkcfg->source_gather_count = 0; /* NA */

	blkcfg->block_size = bufsz;
	blkcfg->next_block = NULL; /* no other blocks */
	blkcfg->source_gather_en = 0; /* NA */
	blkcfg->dest_scatter_en = 0; /* NA */

	blkcfg->source_addr_adj = src_addr_adj;
	blkcfg->dest_addr_adj = dest_addr_adj;

	blkcfg->source_reload_en = 0; /* disabled */
	blkcfg->dest_reload_en = 0; /* disabled */
	blkcfg->fifo_mode_control = 0; /* NA */
	blkcfg->flow_control_mode = 0; /* NA */

	dbg_tp_update(data, 0x3FU);

	return 0;
}
#endif

static int bus_ready(const struct device *dev)
{
	const struct i2c_xec_nl_drv_cfg *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->regbase;
	uint8_t sr = sys_read8(rb + XEC_I2C_SR_OFS);

	if (sr != (BIT(XEC_I2C_SR_PIN_POS) | BIT(XEC_I2C_SR_NBB_POS))) {
		return -EIO;
	}

	return 0;
}

void pr_xfr_begin(const struct device *dev)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	struct i2c_xec_nl_xfr *xfr = &data->xfr;
	struct i2c_msg *m = NULL;
	uint8_t i = 0, midx = 0;

	LOG_INF("XEC I2C-NL Xfr begin");
	LOG_INF(" msg group wrlen = 0x%04x",xfr->wrlen);
	LOG_INF(" msg group rdlen = 0x%04x",xfr->rdlen);
	LOG_INF(" msg group wridx = %u", xfr->wridx);
	LOG_INF(" msg group rdidx = %u", xfr->rdidx);
	LOG_INF(" msg group nwr = %u", xfr->nwr);
	LOG_INF(" msg group nrd = %u", xfr->nrd);

	midx = xfr->wridx;
	for (i = 0; i < xfr->nwr; i++) {
		m = &data->msgs[midx];
		LOG_INF("msg[%u] is write: buf=0x%0x len=%u flags=0x%02x",
		        midx, (uint32_t)m->buf, m->len, m->flags);
		midx++;
	}

	midx = xfr->rdidx;
	for (i = 0; i < xfr->nrd; i++) {
		m = &data->msgs[midx];
		LOG_INF("msg[%u] is read: buf=0x%0x len=%u flags=0x%02x",
		        midx, (uint32_t)m->buf, m->len, m->flags);
		midx++;
	}
}

/* TODO pass buffer pointer and length! */
#if 0
static int i2c_xec_nl_dma_cfg(const struct device *i2c_dev, uint8_t *buf, uint32_t buflen,
                              enum dma_channel_direction dir)
{
	struct i2c_xec_nl_drv_data *const data = i2c_dev->data;
	const struct i2c_xec_nl_drv_cfg *i2c_drvcfg = i2c_dev->config;
	int rc = 0;

	k_spinlock_key_t key = k_spin_lock(&data->spin_lock);

	dbg_tp_update(data, 0x40U);

	xec_nl_dma_block_config(i2c_dev, dir, i2c_drvcfg->cm_dma_trigsrc, buf, buflen);
	xec_nl_dma_chan_config(i2c_dev, dir, i2c_drvcfg->cm_dma_trigsrc);

	rc = dma_config(i2c_drvcfg->cm_dma_dev, i2c_drvcfg->cm_dma_chan, &data->dma_chan_cfg);
	if (rc != 0) {
		dbg_tp_update(data, 0x41U);
		k_spin_unlock(&data->spin_lock, key);
		LOG_ERR("DMA chan %u config error (%d)", i2c_drvcfg->cm_dma_chan, rc);
		return rc;
	}

	rc = dma_start(i2c_drvcfg->cm_dma_dev, i2c_drvcfg->cm_dma_chan);
	if (rc != 0) {
		dbg_tp_update(data, 0x42U);
		k_spin_unlock(&data->spin_lock, key);
		LOG_ERR("DMA chan %u start error (%d)", i2c_drvcfg->cm_dma_chan, rc);
		return rc;
	}

	k_spin_unlock(&data->spin_lock, key);

	dbg_tp_update(data, 0x4FU);

	return rc;
}

static int i2c_xec_nl_dma_blk_cfg(const struct device *i2c_dev, uint8_t *buf, uint32_t buflen,
                                  enum dma_channel_direction dir)
{
	struct i2c_xec_nl_drv_data *data = i2c_dev->data;
	struct dma_block_config *dblk = &data->tx_dma_blk_cfg;

	if (dir == MEMORY_TO_PERIPHERAL) {
		dblk->source_address = (uint32_t)buf;
	} else if (dir == PERIPHERAL_TO_MEMORY) {
		dblk = &data->rx_dma_blk_cfg;
		dblk->dest_address = (uint32_t)buf;
	} else {
		return -EINVAL;
	}

	dblk->block_size = buflen;

	return 0;
}
#endif

/* I2C Write: PROC:RUN=11, START0=1, STARTN=0, STOP=1, wrCnt=1+Nw, rdCnt=0
 *   START
 *   DMA 1-byte target write address
 *   DMA N byte write data
 *   CM-DONE with PROC:RUN=00
 *   STOP
 *   IDLE
 *
 * I2C Read: PROC:RUN=11, START0=1, STARTN=0, STOP=1, wrCnt=1, rdCnt=Nr
 *   START
 *   DMA 1-byte target read address
 *   CM-DONE with PROC:RUN=01
 *   DMA N byte read data
 *   CM-DONE with PROC:RUN=00
 *   STOP
 *   IDLE
 *
 * I2C Write-Read: PROC:RUN=11, START0=1, STARTN=1, STOP=1, wrCnt=1+Nw, rdCnt=Nr,
 *   START
 *   DMA 1-byte target write address
 *   DMA N byte write data
 *   RPT-START generated by I2C-NL when STARTN==1 and wrCnt=1
 *   DMA 1-byte target read address
 *   CM-DONE with PROC:RUN=01
 *   DMA M bytes read data
 *   CM-DONE with PROC:RUN=00
 *   STOP
 *   IDLE
 *
 */
static int i2c_xec_nl_xfr_begin(const struct device *dev)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	const struct i2c_xec_nl_drv_cfg *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->regbase;
	struct i2c_xec_nl_xfr *xfr = &data->xfr;
	int rc = 0;

	dbg_tp_update(data, 0x10U);

	pr_xfr_begin(dev);

	soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 0);
	sys_write32(0, rb + XEC_I2C_HCMD_OFS);
	sys_write32(0, rb + XEC_I2C_ELEN_OFS);
	sys_write32(XEC_I2C_CMPL_RW1C_MSK, rb + XEC_I2C_CMPL_OFS);
	sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_HD_IEN_POS);
	soc_ecia_girq_status_clear(drvcfg->girq, drvcfg->girq_pos);

	data->extlen = XEC_I2C_ELEN_HWR_SET((uint32_t)xfr->wrlen >> 8);
	data->extlen |= XEC_I2C_ELEN_HRD_SET((uint32_t)xfr->rdlen >> 8);

	data->cmd = XEC_I2C_HCMD_WCL_SET((uint32_t)xfr->wrlen);
	data->cmd |= XEC_I2C_HCMD_RCL_SET((uint32_t)xfr->rdlen);
	data->cmd |= (BIT(XEC_I2C_HCMD_START0_POS) | BIT(XEC_I2C_HCMD_STOP_POS) |
	              BIT(XEC_I2C_HCMD_PROC_POS) | BIT(XEC_I2C_HCMD_RUN_POS));

	dbg_tp_update(data, 0x11U);

	if ((xfr->nwr != 0) && (xfr->nrd != 0)) {
		dbg_tp_update(data, 0x12U);
		data->cmd |= BIT(XEC_I2C_HCMD_STARTN_POS);
	}

	rc = dma_reload(drvcfg->cm_dma_dev, drvcfg->cm_dma_chan, (uint32_t)&xfr->wr_addr,
	                rb + XEC_I2C_HTX_OFS, 1U);
	if (rc != 0) {
		dbg_tp_update(data, 0x13U);
		LOG_ERR("XEC-NL DMA reload error (%d)", rc);
		return rc;
	}

	rc = dma_start(drvcfg->cm_dma_dev, drvcfg->cm_dma_chan);
	if (rc != 0) {
		dbg_tp_update(data, 0x14U);
		LOG_ERR("XEC-NL DMA start error (%d)", rc);
		return rc;
	}

	sys_write32(data->extlen, rb + XEC_I2C_ELEN_OFS);
	sys_write32(data->cmd, rb + XEC_I2C_HCMD_OFS); /* start I2C-NL */
	soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 1U);

	dbg_tp_update(data, 0x15U);

	rc = k_sem_take(&data->sync_sem, K_FOREVER);
	if (rc != 0) {
		dbg_tp_update(data, 0x16U);
		LOG_ERR("Sync semaphore wait error (%d)", rc);
	}

	dbg_tp_update(data, 0x1FU);

	return rc;
}

/* Zephyr is deprecating leaving I2C open on transfer exit
 * For now, we assume transfers are closed and bus is idle!
 */
static int i2c_xec_nl_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				uint16_t addr)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	int rc = 0;

	if ((addr & ~0x7Fu) != 0) {
		LOG_ERR("HW supports 7-bit addresses only");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock_mut, K_FOREVER);
#ifdef CONFIG_I2C_TARGET
	if (data->targ_bitmap != 0) {
		k_mutex_unlock(&data->lock_mut);
		return -EBUSY;
	}
#endif

	rc = check_msgs(msgs, num_msgs);
	if (rc != 0) {
		return rc;
	}

	rc = bus_ready(dev);
	if (rc != 0) {
		LOG_ERR("Bus not ready!");
		goto xfr_exit;
	}

	dbg_tp_clear(data);

	data->addr = addr & 0x7FU; /* address is b[6:0] */
	data->msgs = msgs;
	data->num_msgs = num_msgs;
	data->msg_idx = 0;

	rc = build_msg_group(dev);
	if (rc != 0) {
		LOG_ERR("Build message group failed!");
		goto xfr_exit;
	}

	rc = i2c_xec_nl_xfr_begin(dev);
	if (rc != 0) {
		LOG_ERR("Transfer begin failed!");
	}

xfr_exit:
	dbg_tp_update(data, 0xFU);
	k_mutex_unlock(&data->lock_mut);

	return rc;
}

#ifdef CONFIG_I2C_CALLBACK
static int i2c_xec_nl_transfer_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				  uint16_t addr, i2c_callback_t cb, void *user_data)
{
	return -ENOTSUP;
}
#endif

#ifdef CONFIG_I2C_TARGET
static int xec_i2c_nl_target_register(const struct device *dev, struct i2c_target_config *tcfg)
{
	return -ENOTSUP;
}

static int xec_i2c_nl_target_unregister(const struct device *dev, struct i2c_target_config *tcfg)
{
	return -ENOTSUP;
}
#endif

/* -------- I2C controller interrupt service routine and helpers -------- */

static bool i2c_xec_nl_errors(const struct device *dev)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	struct i2c_xec_nl_xfr *xfr = &data->xfr;

	if ((data->i2c_sr & BIT(XEC_I2C_SR_BER_POS)) != 0) {
		xfr->xerr = XEC_I2C_NL_ERR_BUS;
		return true;
	}

	if ((data->i2c_sr & BIT(XEC_I2C_SR_LAB_POS)) != 0) {
		xfr->xerr = XEC_I2C_NL_ERR_LOST_ARB;
		return true;
	}

	return false;
}

#ifdef CONFIG_I2C_TARGET
static bool i2c_xec_nl_tm_handler(const struct device *dev)
{
#if 0 /* TODO */
	struct i2c_xec_nl_drv_data *const data = dev->data;
	struct i2c_xec_nl_xfr *xfr = &data->xfr;
	const struct i2c_xec_nl_drv_cfg *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->regbase;
	bool tm_xfr_done = 0;

	if (i2c_xec_nl_errors(dev) == true) {
		return true;
	}

	return tm_xfr_done;
#else
	return true; /* done */
#endif
}
#endif

#define I2C_XEC_NL_CMD_ALL_DONE 0
#define I2C_XEC_NL_CMD_PAUSE    BIT(XEC_I2C_HCMD_RUN_POS)

static bool i2c_xec_nl_cm_handler(const struct device *dev)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	struct i2c_xec_nl_xfr *xfr = &data->xfr;
	const struct i2c_xec_nl_drv_cfg *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->regbase;
	uint32_t cmd_msk = BIT(XEC_I2C_HCMD_RUN_POS) | BIT(XEC_I2C_HCMD_PROC_POS);
	uint32_t hcmd = 0, src = 0, dst = 0, size = 0;
	int rc = 0;
	bool xfr_done = false;

	dbg_tp_update(data, 0x90U);

	sys_set_bit(rb + XEC_I2C_CMPL_OFS,  XEC_I2C_CMPL_HDONE_POS);

	if (i2c_xec_nl_errors(dev) == true) {
		dbg_tp_update(data, 0x91U);
		return true;
	}

	/* TODO revisit for I2C Read.*/
	if ((xfr->state != I2C_XEC_NL_XFR_STATE_RD_MSGS) &&
	    ((data->i2c_compl & BIT(XEC_I2C_CMPL_HNAKX_POS)) != 0)) {
		dbg_tp_update(data, 0x92U);
		/* Target NAK'd address or data written to it */
		xfr->xerr = XEC_I2C_NL_ERR_CM_NAK;
		return true;
	}

	hcmd = sys_read32(rb + XEC_I2C_HCMD_OFS);

	if ((hcmd & cmd_msk) == I2C_XEC_NL_CMD_ALL_DONE) {
		dbg_tp_update(data, 0x93U);
		/* Both I2C-NL write and read counts are 0 */
		xfr_done = true;
	} else if ((hcmd & cmd_msk) == I2C_XEC_NL_CMD_PAUSE) {
		dbg_tp_update(data, 0x94U);
		/* RPT-START target_read_address has been transmitted.
		 * We must re-configure DMA channel for PERIPHERAL_TO_MEMORY, configure
		 * DMA block for first read message, and unpause I2C-NL by setting proceed bit.
		 * DMA channel usage when driver. We can do this by not supporting multi-master.
		 * The driver is Controller-Mode at driver init.
		 * If the application registers a target then the driver switches to target
		 * mode and no Controller-Mode transfers are allowed.
		 * This save us the overhead of dma_config API.
		 *  CM for mem2Dev direction
		 *  TM for dev2mem direction
		 */
		src = (uint32_t)(rb + XEC_I2C_HRX_OFS);
		dst = (uint32_t)data->msgs[xfr->rdidx].buf;
		size = (uint32_t)data->msgs[xfr->rdidx].len;

		rc = dma_reload(drvcfg->tm_dma_dev, drvcfg->tm_dma_chan, src, dst, size);
		if (rc != 0) {
			dbg_tp_update(data, 0x95U);
			/* TODO abort the transfer! */
			xfr->xerr = XEC_I2C_NL_ERR_DMA_CFG;
			return true;

		}

		rc = dma_start(drvcfg->tm_dma_dev, drvcfg->tm_dma_chan);
		if (rc != 0) {
			dbg_tp_update(data, 0x96U);
			/* TODO abort the transfer! */
			xfr->xerr = XEC_I2C_NL_ERR_DMA_CFG;
			return true;
		}

/*		k_spinlock_key_t key = k_spin_lock(&data->spin_lock); */

		dbg_tp_update(data, 0x97U);

		xfr->rdidx++;
		xfr->nrd--;
		xfr->state = I2C_XEC_NL_XFR_STATE_RD_MSGS;

/*		k_spin_unlock(&data->spin_lock, key); */

		/* trigger I2C-NL to release SCL and begin read phase */
		sys_set_bits(rb + XEC_I2C_HCMD_OFS, (BIT(XEC_I2C_HCMD_RUN_POS) |
		                                     BIT(XEC_I2C_HCMD_PROC_POS)));

		soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 1U);

		dbg_tp_update(data, 0x98U);
	}

	dbg_tp_update(data, 0x9FU);

	return xfr_done;
}

void i2c_xec_nl_isr(const struct device *dev)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	const struct i2c_xec_nl_drv_cfg *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->regbase;
	uint32_t cfg = 0;
	bool xfr_done = false;

	dbg_tp_update(data, 0x80U);

	data->i2c_sr = sys_read8(rb + XEC_I2C_SR_OFS);
	data->i2c_compl = sys_read32(rb + XEC_I2C_CMPL_OFS);
	cfg = sys_read32(rb + XEC_I2C_CFG_OFS);

	soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 0);

	if ((cfg & BIT(XEC_I2C_CFG_IDLE_IEN_POS)) != 0) {
		dbg_tp_update(data, 0x81U);
		if ((data->i2c_compl & BIT(XEC_I2C_CMPL_IDLE_POS)) != 0) {
			dbg_tp_update(data, 0x82U);
			sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
			sys_clear_bit(rb + XEC_I2C_CMPL_OFS, XEC_I2C_CMPL_IDLE_POS);
			/* TODO record reason! */
			k_sem_give(&data->sync_sem);
		}
	} else { /* enable IDLE. Must be done in ISR */
		dbg_tp_update(data, 0x83U);
		sys_clear_bit(rb + XEC_I2C_CMPL_OFS, XEC_I2C_CMPL_IDLE_POS);
		sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
	}

#ifdef CONFIG_I2C_TARGET
	if ((data->i2c_compl & BIT(XEC_I2C_CMPL_TDONE_POS)) != 0) {
		dbg_tp_update(data, 0x84U);
		xfr_done = i2c_xec_nl_tm_handler(dev);
		/* TODO re-arm target */
		return;
	}
#endif
	if ((data->i2c_compl & BIT(XEC_I2C_CMPL_HDONE_POS)) != 0) {
		dbg_tp_update(data, 0x85U);
		xfr_done = i2c_xec_nl_cm_handler(dev);
	}

	dbg_tp_update(data, 0x86U);

	sys_set_bits(rb + XEC_I2C_CMPL_OFS, data->i2c_compl);
	soc_ecia_girq_status_clear(drvcfg->girq, drvcfg->girq_pos);

	if (xfr_done == true) {
		dbg_tp_update(data, 0x87U);
		k_sem_give(&data->sync_sem);
	}

	dbg_tp_update(data, 0x8FU);
}

#ifdef CONFIG_PM_DEVICE
/* When target mode support is not present or target mode not enabled we can turn off
 * the controller and pins in suspend. If target mode is enabled the I2C as target
 * can wake the SoC. There are two parts to the wake:
 * 1. I2C wake enable and status registers.
 * 2. GIRQ22 PLL wake which turns on the PLL when edge on SDA is detected. Once the PLL
 *    is up and I2C determines there is a START with an address it should respond to,
 *    the controller will generate an interrupt to the EC to fully wake the system.
 *    If the edge event is an address for this target, HW will turn the PLL off and
 *    reenter suspend sleep state.
 */
static int i2c_xec_nl_pm_action(const struct device *dev, enum pm_device_action action)
{
	/* TODO */
	return -ENOTSUP;
}
#endif

static int i2c_xec_nl_init(const struct device *dev)
{
	struct i2c_xec_nl_drv_data *const data = dev->data;
	const struct i2c_xec_nl_drv_cfg *drvcfg = dev->config;
	uint32_t bitrate_cfg = i2c_map_dt_bitrate(drvcfg->clk_freq);
	int rc = 0;

	data->dev = dev;

	if (bitrate_cfg == 0) {
		return -EINVAL;
	}

	rc = pinctrl_apply_state(drvcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("XEC-NL I2C pinctrl setup failed (%d)", rc);
		return rc;
	}

	k_mutex_init(&data->lock_mut);
	k_sem_init(&data->sync_sem, 0, 1);

	data->port = drvcfg->port;

	rc = i2c_xec_nl_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	if (rc != 0) {
		LOG_ERR("XEC-NL I2C config error (%d)", rc);
		return rc;
	}

	if (drvcfg->irq_config_func != NULL) {
		drvcfg->irq_config_func();
		soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 1U);
	}

	rc = xec_nl_dma_config(dev, drvcfg->cm_dma_chan, XEC_I2C_NL_MODE_CM, MEMORY_TO_PERIPHERAL);
	if (rc != 0) {
		LOG_ERR("XEC-NL mem2dev DMA chan config error (%d)", rc);
		return rc;
	}

	rc = xec_nl_dma_config(dev, drvcfg->tm_dma_chan, XEC_I2C_NL_MODE_CM, PERIPHERAL_TO_MEMORY);
	if (rc != 0) {
		LOG_ERR("XEC-NL dev2mem DMA chan config error (%d)", rc);
	}

	return rc;
}

static DEVICE_API(i2c, i2c_xec_nl_driver_api) = {
	.configure = i2c_xec_nl_configure,
	.get_config = i2c_xec_nl_get_config,
	.transfer = i2c_xec_nl_transfer,
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = i2c_xec_nl_transfer_cb,
#endif
#ifdef CONFIG_I2C_TARGET
	.target_register = xec_i2c_nl_target_register,
	.target_unregister = xec_i2c_nl_target_unregister,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit  i2c_iodev_submit_fallback,
#endif
	.recover_bus = xec_i2c_nl_recover_bus,
};

#define XEC_I2C_NL_GIRQ_DT(inst, idx) \
	(uint8_t)MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, idx))
#define XEC_I2C_NL_GIRQ_POS_DT(inst, idx) \
	(uint8_t)MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, idx))

#define I2C_XEC_NL_DMA_NODE(i, name)    DT_INST_DMAS_CTLR_BY_NAME(i, name)
#define I2C_XEC_NL_DMA_DEVICE(i, name)  DEVICE_DT_GET(I2C_XEC_NL_DMA_NODE(i, name))
#define I2C_XEC_NL_DMA_CHAN(i, name)    DT_INST_DMAS_CELL_BY_NAME(i, name, channel)
#define I2C_XEC_NL_DMA_TRIGSRC(i, name) DT_INST_DMAS_CELL_BY_NAME(i, name, trigsrc)

#define I2C_XEC_NL_XFRBUF_SIZE(i) DT_PROP_OR(i, xfr_buffer_size, XEC_I2C_NL_XFR_BUF_SZ_DFLT)
#define I2C_XEC_NL_XFRBUF_DEFINE(i) \
	static uint8_t i2c_xec_nl_xfrbuf##i[I2C_XEC_NL_XFRBUF_SIZE(i)] __attribute__((aligned(4)))
#define I2C_XEC_NL_XFRBUF_GET(i) i2c_xec_nl_xfrbuf##i

#ifdef CONFIG_I2C_TARGET
#define I2C_XEC_NL_TM_CFG(inst)
#else
#define I2C_XEC_NL_TM_CFG(inst) \
	.tm_dma_dev = I2C_XEC_NL_DMA_DEVICE(inst, target_mode), \
	.tm_dma_chan = I2C_XEC_NL_DMA_CHAN(i, target_mode), \
	.tm_dma_trigsrc = I2C_XEC_NL_DMA_TRIGSRC(i, target_mode),
#endif

#define I2C_XEC_NL_DEVICE(i) \
	struct i2c_xec_nl_drv_data i2c_xec_nl_drv_data##i = { \
		.dev = DEVICE_DT_INST_GET(i), \
	}; \
	I2C_XEC_NL_XFRBUF_DEFINE(i); \
	PINCTRL_DT_INST_DEFINE(i); \
	void i2c_xec_i2c_nl_irq_connect##i(void) { \
		IRQ_CONNECT(DT_INST_IRQN(i), DT_INST_IRQ(i, priority), i2c_xec_nl_isr, \
			    DEVICE_DT_INST_GET(i), 0); \
		irq_enable(DT_INST_IRQN(i)); \
	} \
	const struct i2c_xec_nl_drv_cfg i2c_xec_nl_drv_cfg##i = { \
		.regbase = (mm_reg_t)DT_INST_REG_ADDR(i), \
		.xfr_bufsz = I2C_XEC_NL_XFRBUF_SIZE(i), \
		.xfrbuf = I2C_XEC_NL_XFRBUF_GET(i), \
		.cm_dma_dev = I2C_XEC_NL_DMA_DEVICE(i, host_mode), \
		.tm_dma_dev = I2C_XEC_NL_DMA_DEVICE(i, target_mode), \
		.cm_dma_chan = I2C_XEC_NL_DMA_CHAN(i, host_mode), \
		.cm_dma_trigsrc = I2C_XEC_NL_DMA_TRIGSRC(i, host_mode), \
		.tm_dma_chan = I2C_XEC_NL_DMA_CHAN(i, target_mode), \
		.tm_dma_trigsrc = I2C_XEC_NL_DMA_TRIGSRC(i, target_mode), \
		.girq = XEC_I2C_NL_GIRQ_DT(i, 0), \
		.girq_pos = XEC_I2C_NL_GIRQ_POS_DT(i, 0), \
		.enc_scr = (uint8_t)DT_INST_PROP(i, pcr_scr), \
		.port = (uint8_t)DT_INST_PROP(i, port_sel), \
		.clk_freq = DT_INST_PROP(i, clock_frequency), \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i), \
		.irq_config_func = i2c_xec_i2c_nl_irq_connect##i, \
		.sda_gpio = GPIO_DT_SPEC_INST_GET(i, sda_gpios), \
		.scl_gpio = GPIO_DT_SPEC_INST_GET(i, scl_gpios), \
	}; \
	PM_DEVICE_DT_INST_DEFINE(i, i2c_xec_nl_pm_action); \
	I2C_DEVICE_DT_INST_DEFINE(i, i2c_xec_nl_init, PM_DEVICE_DT_INST_GET(i), \
				  &i2c_xec_nl_drv_data##i, &i2c_xec_nl_drv_cfg##i, \
				  POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, \
				  &i2c_xec_nl_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_XEC_NL_DEVICE)
