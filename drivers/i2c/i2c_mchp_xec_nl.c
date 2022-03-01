/*
 * Copyright (c) 2022 Microchip Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "reg/mec172x_i2c_smb.h"
#define DT_DRV_COMPAT microchip_xec_i2c_nl

#include <zephyr.h>
#include <kernel.h>
#include <soc.h>
#include <errno.h>
#include <devicetree/dma.h>
#include <drivers/clock_control.h>
#include <drivers/dma.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/interrupt_controller/intc_mchp_xec_ecia.h>
#include <drivers/pinctrl.h>
#include <irq.h>
#include <sys/printk.h>
#include <sys/sys_io.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_mchp, CONFIG_I2C_LOG_LEVEL);

/* #define I2C_XEC_NL_DEBUG */

#define SPEED_100KHZ_BUS	0
#define SPEED_400KHZ_BUS	1
#define SPEED_1MHZ_BUS		2

#define I2C_CTRL_OWN_ADDR_1	0x7EU
#define I2C_CTRL_OWN_ADDR_2	0x7FU
#define I2C_CTRL_OWN_ADDR	((uint32_t)(I2C_CTRL_OWN_ADDR_1) |	\
				 ((uint32_t)(I2C_CTRL_OWN_ADDR_2) << 8))

/* I2C Read/Write bit pos */
#define I2C_READ_WRITE_POS	0

/* I2C SCL and SDA lines(signals) */
#define I2C_LINES_SCL_POS	0
#define I2C_LINES_SDA_POS	1
#define I2C_LINES_SCL_HI	BIT(I2C_LINES_SCL_POS)
#define I2C_LINES_SDA_HI	BIT(I2C_LINES_SDA_POS)
#define I2C_LINES_BOTH_HI	(I2C_LINES_SCL_HI | I2C_LINES_SDA_HI)

/* Network layer TX buffer size */
#define I2C_NWL_TX_MSG_MAX	64U
#define I2C_NWL_TX_BUF_SIZE	(I2C_NWL_TX_MSG_MAX + 4U)

#define I2C_XEC_CFG_WAIT_MS	35U
#define RESET_WAIT_US		20U
#define I2C_ENABLE_WAIT_US	160U
#define WAIT_MPROCEED_US	2U
#define WAIT_MPROCEED_LOOPS	100U

/* I2C recover SCL low retries */
#define I2C_RECOVER_SCL_LOW_RETRIES	10
/* I2C recover SDA low retries */
#define I2C_RECOVER_SDA_LOW_RETRIES	3
/* I2C recovery bit bang delay */
#define I2C_RECOVER_BB_DELAY_US		5
/* I2C recovery SCL sample delay */
#define I2C_RECOVER_SCL_DELAY_US	50

/* driver states */
#define I2C_XEC_STATE_CLOSED		0
#define I2C_XEC_STATE_OPEN_TX		1
#define I2C_XEC_STATE_OPEN_RX		2
#define I2C_XEC_STATE_STOP		3

/* driver flags */
#define I2C_XEC_EVENT_NONE		0
#define I2C_XEC_EVENT_START		1
#define I2C_XEC_EVENT_RPT_START		2
#define I2C_XEC_EVENT_STOP		3
#define I2C_XEC_EVENT_DATA_XFR_DONE	4

struct xec_nl_speed_cfg {
	uint32_t bus_clk;
	uint32_t data_timing;
	uint32_t start_hold_time;
	uint32_t idle_scale;
	uint32_t timeout_scale;
};

struct i2c_nl_xec_config {
	struct i2c_smb_regs * const regs;
	uint8_t port_sel;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t pcr_idx;
	uint8_t pcr_bitpos;
	uint8_t ctr_dma_chan;
	uint8_t ctr_dma_id;
	uint8_t dev_dma_chan;
	uint8_t dev_dma_id;
	const struct device *dma_dev;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
	const struct gpio_dt_spec scl_gpio;
	const struct gpio_dt_spec sda_gpio;
};

struct i2c_nl_xec_data {
	struct k_mutex ctr_mutex;
	struct k_event ctr_events;
	uint32_t i2c_compl;
	uint32_t dma_err;
	uint8_t i2c_status;
	uint8_t i2c_ctrl;
	uint8_t i2c_addr;
	uint8_t speed_id;
	uint8_t state;
	uint8_t event;
	uint8_t dma_done;
	uint8_t nl_done;
#ifdef I2C_XEC_NL_DEBUG
	uint32_t dbg_idx;
	uint32_t dbg_i2c_mcmd[4];
	uint32_t dbg_i2c_compl[4];
	uint8_t dbg_i2c_status[4];
#endif
	struct i2c_msg rem;
	uint32_t dbi;
	uint8_t db[I2C_NWL_TX_BUF_SIZE];
};

/* Recommended programming values based on 16MHz
 * i2c_baud_clk_period/bus_clk_period - 2 = (low_period + hi_period)
 * bus_clk_reg (16MHz/100KHz -2) = 0x4F + 0x4F
 *             (16MHz/400KHz -2) = 0x0F + 0x17
 *             (16MHz/1MHz -2) = 0x05 + 0x09
 */
static const struct xec_nl_speed_cfg xec_cfg_params[] = {
	[SPEED_100KHZ_BUS] = {
		.bus_clk            = 0x00004F4F,
		.data_timing        = 0x0C4D5006,
		.start_hold_time    = 0x0000004D,
		.idle_scale         = 0x01FC01ED,
		.timeout_scale      = 0x4B9CC2C7,
	},
	[SPEED_400KHZ_BUS] = {
		.bus_clk            = 0x00000F17,
		.data_timing        = 0x040A0A06,
		.start_hold_time    = 0x0000000A,
		.idle_scale         = 0x01000050,
		.timeout_scale      = 0x159CC2C7,
	},
	[SPEED_1MHZ_BUS] = {
		.bus_clk            = 0x00000509,
		.data_timing        = 0x04060601,
		.start_hold_time    = 0x00000006,
		.idle_scale         = 0x10000050,
		.timeout_scale      = 0x089CC2C7,
	},
};

/* return state of I2C SCL and SDA lines by reading GPIO input. */
static int get_lines(const struct device *dev, uint8_t *lines)
{
	const struct i2c_nl_xec_config * const cfg = dev->config;
	uint8_t  temp = 0;

	if (!lines) {
		return -EINVAL;
	}

	int ret = gpio_pin_get_raw(cfg->scl_gpio.port, cfg->scl_gpio.pin);

	if (ret < 0) {
		return ret;
	}

	if (ret > 0) {
		temp |= I2C_LINES_SCL_HI;
	}

	ret = gpio_pin_get_raw(cfg->sda_gpio.port, cfg->sda_gpio.pin);
	if (ret < 0) {
		return ret;
	}

	if (ret > 0) {
		temp |= I2C_LINES_SDA_HI;
	}

	*lines = temp;

	return 0;
}

static void ctrl_reset(const struct device *dev)
{
	const struct i2c_nl_xec_config * const cfg = dev->config;
	struct i2c_smb_regs * const regs = cfg->regs;

	/* assert reset for >= 1 BAUD clock (16 MHz) */
	regs->CFG |= MCHP_I2C_SMB_CFG_RESET;
	regs->BLKID = 0U; /* AHB clock is 48HMz. */
	regs->BLKID = 0U; /* one AHB access is minimum 3 clocks */
	regs->BLKID = 0U;
	regs->BLKID = 0U;
	regs->CFG &= ~(MCHP_I2C_SMB_CFG_RESET);

	regs->CFG |= MCHP_I2C_SMB_CFG_FLUSH_SXBUF_WO |
		     MCHP_I2C_SMB_CFG_FLUSH_SRBUF_WO |
		     MCHP_I2C_SMB_CFG_FLUSH_MXBUF_WO |
		     MCHP_I2C_SMB_CFG_FLUSH_MRBUF_WO;
}

static void ctrl_set_port(struct i2c_smb_regs *regs, uint8_t port)
{
	uint32_t temp = ((uint32_t)port  << MCHP_I2C_SMB_CFG_PORT_SEL_POS) &
			 MCHP_I2C_SMB_CFG_PORT_SEL_MASK;

	regs->CFG = (regs->CFG & ~MCHP_I2C_SMB_CFG_PORT_SEL_MASK) | temp;
}

static void ctrl_config1(const struct device *dev)
{
	const struct i2c_nl_xec_config * const cfg = dev->config;
	struct i2c_nl_xec_data * const data = dev->data;
	struct i2c_smb_regs * const regs = cfg->regs;

	ctrl_set_port(regs, cfg->port_sel);

	if (data->speed_id > ARRAY_SIZE(xec_cfg_params)) {
		data->speed_id = 0U;
	}

	const struct xec_nl_speed_cfg *spdcfg = &xec_cfg_params[data->speed_id];

	regs->CTRLSTS = MCHP_I2C_SMB_CTRL_PIN;
	regs->OWN_ADDR = I2C_CTRL_OWN_ADDR;

	regs->BUSCLK = spdcfg->bus_clk;
	regs->CTRLSTS = (MCHP_I2C_SMB_CTRL_PIN | MCHP_I2C_SMB_CTRL_ESO |
			 MCHP_I2C_SMB_CTRL_ACK);

	regs->DATATM = spdcfg->data_timing;
	regs->RSHTM = spdcfg->start_hold_time;
	regs->IDLSC = spdcfg->idle_scale;
	regs->TMOUTSC = spdcfg->timeout_scale;

	/* filter enable */
	regs->CFG |= MCHP_I2C_SMB_CFG_FEN;

	/* clear sticky status */
	regs->COMPL = MCHP_I2C_SMB_CMPL_RW1C_MASK;

	/* enable controller */
	regs->CFG |= MCHP_I2C_SMB_CFG_ENAB;
}

static int ctrl_config(const struct device *dev)
{
	const struct i2c_nl_xec_config * const cfg = dev->config;
	struct i2c_nl_xec_data * const data = dev->data;
	struct i2c_smb_regs * const regs = cfg->regs;

	ctrl_reset(dev);
	ctrl_config1(dev);

	/* wait SMBus timeout max */
	k_msleep(I2C_XEC_CFG_WAIT_MS);

	data->i2c_status = regs->CTRLSTS;

	if (data->i2c_status & MCHP_I2C_SMB_STS_BER) {
		return -EIO;
	}

	return 0;
}

static int i2c_nl_xec_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_nl_xec_data * const data = dev->data;
	uint32_t temp = I2C_SPEED_GET(dev_config);

	if (!(dev_config & I2C_MODE_MASTER)) {
		return -EINVAL;
	}

	switch (temp) {
	case I2C_SPEED_STANDARD:
		data->speed_id = 0U;
		break;
	case I2C_SPEED_FAST:
		data->speed_id = 1U;
		break;
	case I2C_SPEED_FAST_PLUS:
		data->speed_id = 2U;
		break;
	default:
		return -EINVAL;
	}

	return ctrl_config(dev);
}

static int i2c_nl_xec_get_config(const struct device *dev, uint32_t *dev_config)
{
	const struct i2c_nl_xec_config * const cfg = dev->config;
	struct i2c_smb_regs * const regs = cfg->regs;
	uint32_t temp = 0;

	if (!dev || !dev_config) {
		return -EINVAL;
	}

	temp = regs->BUSCLK;
	for (size_t n = 0; n < ARRAY_SIZE(xec_cfg_params); n++) {
		if (temp == xec_cfg_params[n].bus_clk) {
			*dev_config = n + I2C_SPEED_STANDARD;
			break;
		}
	}

	*dev_config |= I2C_MODE_MASTER;

	return 0;
}

static int i2c_nl_xec_recover_bus(const struct device *dev)
{
	const struct i2c_nl_xec_config * const cfg = dev->config;
	struct i2c_smb_regs * const regs = cfg->regs;
	int i, j, ret;

	LOG_ERR("I2C attempt bus recovery\n");

	/* reset controller to a known state */
	regs->CFG = MCHP_I2C_SMB_CFG_RESET;
	k_busy_wait(RESET_WAIT_US);

	regs->CFG = BIT(14) | MCHP_I2C_SMB_CFG_FEN |
		    (cfg->port_sel & MCHP_I2C_SMB_CFG_PORT_SEL_MASK);
	regs->CFG |= MCHP_I2C_SMB_CFG_FLUSH_SXBUF_WO |
		     MCHP_I2C_SMB_CFG_FLUSH_SRBUF_WO |
		     MCHP_I2C_SMB_CFG_FLUSH_MXBUF_WO |
		     MCHP_I2C_SMB_CFG_FLUSH_MRBUF_WO;
	regs->CTRLSTS = MCHP_I2C_SMB_CTRL_PIN;
	/* Enable bit-bang mode: are SCL and SDA tri-stated inputs */
	regs->BBCTRL = MCHP_I2C_SMB_BB_EN | MCHP_I2C_SMB_BB_CL |
		       MCHP_I2C_SMB_BB_DAT;

	/* SCL is low: read N times and hope external device releases it */
	if (!(regs->BBCTRL & MCHP_I2C_SMB_BB_CLKI_RO)) {
		for (i = 0;; i++) {
			if (i >= I2C_RECOVER_SCL_LOW_RETRIES) {
				ret = -EBUSY;
				goto recov_exit;
			}
			k_busy_wait(I2C_RECOVER_SCL_DELAY_US);
			if (regs->BBCTRL & MCHP_I2C_SMB_BB_CLKI_RO) {
				break; /* SCL went High */
			}
		}
	}

	if (regs->BBCTRL & MCHP_I2C_SMB_BB_DATI_RO) {
		ret = 0;
		goto recov_exit;
	}

	ret = -EBUSY;
	/* SDA recovery */
	for (i = 0; i < I2C_RECOVER_SDA_LOW_RETRIES; i++) {
		/* SCL output mode and tri-stated */
		regs->BBCTRL = MCHP_I2C_SMB_BB_EN |
			       MCHP_I2C_SMB_BB_SCL_DIR_OUT |
			       MCHP_I2C_SMB_BB_CL |
			       MCHP_I2C_SMB_BB_DAT;
		k_busy_wait(I2C_RECOVER_BB_DELAY_US);

		for (j = 0; j < 9; j++) {
			if (regs->BBCTRL & MCHP_I2C_SMB_BB_DATI_RO) {
				break;
			}
			/* drive SCL low */
			regs->BBCTRL = MCHP_I2C_SMB_BB_EN |
				       MCHP_I2C_SMB_BB_SCL_DIR_OUT |
				       MCHP_I2C_SMB_BB_DAT;
			k_busy_wait(I2C_RECOVER_BB_DELAY_US);
			/* release SCL: pulled high by external pull-up */
			regs->BBCTRL = MCHP_I2C_SMB_BB_EN |
				       MCHP_I2C_SMB_BB_SCL_DIR_OUT |
				       MCHP_I2C_SMB_BB_CL |
				       MCHP_I2C_SMB_BB_DAT;
			k_busy_wait(I2C_RECOVER_BB_DELAY_US);
		}

		/* SCL is High. Produce rising edge on SCL for STOP */
		regs->BBCTRL = MCHP_I2C_SMB_BB_EN | MCHP_I2C_SMB_BB_CL |
				MCHP_I2C_SMB_BB_SDA_DIR_OUT; /* drive low */
		k_busy_wait(I2C_RECOVER_BB_DELAY_US);
		regs->BBCTRL = MCHP_I2C_SMB_BB_EN | MCHP_I2C_SMB_BB_CL |
				MCHP_I2C_SMB_BB_DAT; /* release SCL */
		k_busy_wait(I2C_RECOVER_BB_DELAY_US);

		/* check if SCL and SDA are both high */
		uint8_t bb = regs->BBCTRL &
			(MCHP_I2C_SMB_BB_CLKI_RO | MCHP_I2C_SMB_BB_DATI_RO);

		if (bb == (MCHP_I2C_SMB_BB_CLKI_RO | MCHP_I2C_SMB_BB_DATI_RO)) {
			ret = 0; /* successful recovery */
			goto recov_exit;
		}
	}

recov_exit:
	/* BB mode disable reconnects SCL and SDA to I2C logic. */
	regs->BBCTRL = 0;
	ctrl_config(dev); /* reset and reconfigure controller */

	return ret;
}

static void i2c_nl_xec_dma_done_cb(const struct device *dev, void *arg,
				   uint32_t id, int error_code)
{
	struct i2c_nl_xec_data * const data = ((const struct device *)arg)->data;

	ARG_UNUSED(dev);

	data->dma_err = error_code;
	data->dma_done = 1u;
	k_event_post(&data->ctr_events, BIT(1));
}

/* Network Layer has two important limitations:
 * 1. Network Layer HW state machine was designed for ordered transaction:
 *    START, transmit, receive, and STOP. When HW state machine write
 *    count reaches 0 it will transition to read phase. Once HW state machine
 *    is in the read phase there is no going back to transmit phase without
 *    a STOP and starting a new transaction.
 * 2. The I2C with Network Layer can be controller or target device. Each
 *    I2C-SMB block has two sets of DMA req/done signal pairs. One pair is
 *    for controller mode and one pair for target device mode. Each pair is
 *    identified by a DMA flow control device ID number programmed into the
 *    DMA channel. Controller and target modes have one DMA flow control ID
 *    for both transmit and receive operation. Firmware must switch the DMA
 *    channel direction between transmit and receive phases of the I2C transaction.
 *    While FW reconfigures the DMA channel, the I2C HW stretches the I2C clock.
 *    If the I2C-SMB HW timeouts are enabled FW has a finite amount of time to
 *    reconfigure the DMA channel and continue the transaction.
 *
 * Flow:
 * Check state if open or closed. If closed check state of I2C lines and attempt
 * recovery if both lines are not high (idle).
 * If state is closed set driver flag indicating we need a START and set driver direction as NONE
 * Iterate over num_msgs
 *   if msg flag direction is write AND driver direction is READ then break with ERROR
 *      (issue STOP if not closed?)
 *   if msg flag direction is write
 *     set driver diection = WRITE
 *     if driver START flag or message Rpt-START flag set then
 *       NOTE: if both flags set then use START logic since its beginning of transaction
 *       put address in driver buffer: shift left by 1 and set b[0]=0(write)
 *       copy buffer_len - 1 byte from message to driver buffer
 *       configure NL for write N bytes where N is exact number or 0xFFu
 *       depending upon msg STOP flag, size of message, and buffer size.
 *       Multiple scenarios: driver buffer is limited in size: also applies to non-START cases
 *       1. message data fits in driver buffer. One write transaction.
 *       2. message data does not fit in buffer and may be larger than HW can handle (255 bytes).
 *          Sending the message requires multiple HW transctions.
 *          Logic can be implemeted in ISR to flag when message is done.
 *          We need additional variables in driver data to track.
 *
 *   if msg flag direction is read
 *     set driver direction = READ
 *     if driver START flag or message Rpt-START flag set then
 *       NOTE: if both flags set use START logic.
 *       put address in driver buffer: shift left by 1 and set b[0]=1(read)
 */

static int i2c_nl_dma_cfg(const struct device *dev, uint32_t src, uint32_t dest,
			  uint32_t nbytes, enum dma_channel_direction dir, dma_callback_t cb)
{
	const struct i2c_nl_xec_config * const cfg = dev->config;
	struct dma_config dma_cfg = { 0 };
	struct dma_block_config dma_block_cfg = { 0 };
	int ret = 0;

	dma_cfg.channel_direction = dir;

	if (dir == MEMORY_TO_PERIPHERAL) {
		dma_block_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_block_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		dma_block_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_block_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	dma_block_cfg.source_address = src;
	dma_block_cfg.dest_address = dest;

	dma_cfg.dma_slot = cfg->ctr_dma_id;
	dma_cfg.source_data_size = 1U;
	dma_cfg.dest_data_size = 1U;
	dma_cfg.dma_callback = cb;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.complete_callback_en = 1U;
	dma_cfg.error_callback_en = 1U;
	dma_cfg.block_count = 1U;
	dma_cfg.head_block = &dma_block_cfg;

	dma_block_cfg.block_size = nbytes;

	ret = dma_config(cfg->dma_dev, cfg->ctr_dma_chan, &dma_cfg);
	if (ret) {
		LOG_ERR("XEC I2C DMA config error: (%d)", ret);
		return ret;
	}

	return 0;
}

static int i2c_nl_xec_xfr(const struct device *dev, struct i2c_msg *msg, uint16_t addr)
{
	struct i2c_nl_xec_data * const data = dev->data;
	const struct i2c_nl_xec_config * const cfg = dev->config;
	struct i2c_smb_regs * const regs = cfg->regs;
	void *mptr = NULL;
	uint32_t dmalen = 0U;
	uint32_t rdcnt = 0U;
	uint32_t wrcnt = 0U;
	uint32_t mcmd = 0U;
	uint32_t wait_count = 0U;
	uint32_t events = 0U;
	int ret = 0;
	uint8_t fmt_addr  = (addr & 0x7fu) << 1;

	/* clear event flags before we start any transaction */
	k_event_set(&data->ctr_events, 0u);

	if (!msg) {
		return -EINVAL;
	}

	if (msg->flags & I2C_MSG_READ) {
		fmt_addr |= BIT(0);
	}

	data->dma_err = 0u;
	data->dma_done = 0u;
	data->nl_done = 0u;
	data->dbi = 0u;

	mptr = msg->buf;
	dmalen = msg->len;

	regs->EXTLEN = 0u;

	if (data->event == I2C_XEC_EVENT_START) {
		regs->CFG |= (MCHP_I2C_SMB_CFG_FLUSH_SXBUF_WO |
			      MCHP_I2C_SMB_CFG_FLUSH_SRBUF_WO |
			      MCHP_I2C_SMB_CFG_FLUSH_MXBUF_WO |
			      MCHP_I2C_SMB_CFG_FLUSH_MRBUF_WO);

		regs->CTRLSTS = (MCHP_I2C_SMB_CTRL_PIN | MCHP_I2C_SMB_CTRL_ESO |
				 MCHP_I2C_SMB_CTRL_ACK);
		data->i2c_ctrl = (MCHP_I2C_SMB_CTRL_PIN | MCHP_I2C_SMB_CTRL_ESO |
				  MCHP_I2C_SMB_CTRL_ACK);

		/* After writing control do we need to wait N I2C BAUD clocks before
		 * reading I2C status?
		 * data->i2c_status = regs->CTRLSTS;
		 */
		if (msg->len > I2C_NWL_TX_MSG_MAX) {
			return -EINVAL;
		}
	}

	if (msg->flags & I2C_MSG_READ) {
		data->state = I2C_XEC_STATE_OPEN_RX;
		rdcnt = dmalen;
		if ((data->event == I2C_XEC_EVENT_START) || (msg->flags & I2C_MSG_RESTART)) {
			wrcnt = 1u;
		}
		ret = i2c_nl_dma_cfg(dev, (uint32_t)&regs->MTR_RXB, (uint32_t)mptr,
				     dmalen, PERIPHERAL_TO_MEMORY, i2c_nl_xec_dma_done_cb);
	} else {
		data->state = I2C_XEC_STATE_OPEN_TX;
		wrcnt = dmalen;
		if ((data->event == I2C_XEC_EVENT_START) || (msg->flags & I2C_MSG_RESTART)) {
			wrcnt++;
		}
		ret = i2c_nl_dma_cfg(dev, (uint32_t)mptr, (uint32_t)&regs->MTR_TXB,
				     dmalen, MEMORY_TO_PERIPHERAL, i2c_nl_xec_dma_done_cb);
	}

	if (ret) {
		return ret;
	}

	regs->COMPL = MCHP_I2C_SMB_CMPL_RW1C_MASK;
	data->i2c_status = regs->CTRLSTS;
	data->i2c_compl = regs->COMPL;

	/* enable NL MDONE interrupt */
	regs->CFG |= MCHP_I2C_SMB_CFG_ENMI;

	mcmd = ((wrcnt & 0xffu) << MCHP_I2C_SMB_MSTR_CMD_WR_CNT_POS) |
		((rdcnt & 0xffu) << MCHP_I2C_SMB_MSTR_CMD_RD_CNT_POS);

	mcmd |= (MCHP_I2C_SMB_MSTR_CMD_MRUN | MCHP_I2C_SMB_MSTR_CMD_MPROCEED);

	if (data->event == I2C_XEC_EVENT_START) {
		mcmd |= MCHP_I2C_SMB_MSTR_CMD_START0;
	} else if (msg->flags & I2C_MSG_RESTART) {
		mcmd |= MCHP_I2C_SMB_MSTR_CMD_STARTN;
	}
	if (msg->flags & I2C_MSG_STOP) {
		mcmd |= MCHP_I2C_SMB_MSTR_CMD_STOP;
	}

#ifdef I2C_XEC_NL_DEBUG
	data->dbg_i2c_mcmd[0] = mcmd;
	data->dbg_i2c_compl[0] = regs->COMPL;
	data->dbg_i2c_status[0] = regs->CTRLSTS;
#endif

	regs->MCMD = mcmd;

	if ((data->event == I2C_XEC_EVENT_START) || (msg->flags & I2C_MSG_RESTART)) {
		regs->MTR_TXB = fmt_addr;
		if (rdcnt) {
			wait_count = WAIT_MPROCEED_LOOPS;
			while (regs->MCMD & MCHP_I2C_SMB_MSTR_CMD_MPROCEED) {
				k_busy_wait(WAIT_MPROCEED_US);
				if (!wait_count) {
					return -ETIMEDOUT;
				}
				wait_count--;
			}
#ifdef I2C_XEC_NL_DEBUG
			data->dbg_i2c_mcmd[1] = regs->MCMD;
			data->dbg_i2c_compl[1] = regs->COMPL;
			data->dbg_i2c_status[1] = regs->CTRLSTS;
#endif
			regs->MCMD = regs->MCMD | MCHP_I2C_SMB_MSTR_CMD_MPROCEED;
		}
	}

	ret = dma_start(cfg->dma_dev, cfg->ctr_dma_chan);
	if (ret) {
		LOG_ERR("XEC I2C NL start error: (%d)", ret);
		return ret;
	}

	/* Use Zephyr k_event to wait for event flags set by DMA ISR callback
	 * and I2C ISR. The order ISR's fire changes based transfer direction.
	 * NOTE: kernel k_event is not enabled by default. For this driver
	 * we added a select config directive to enable kernel k_event support.
	 */
	events = k_event_wait_all(&data->ctr_events, 0x03U, false, K_MSEC(1000));
	if (!events) {
		LOG_ERR("XEC I2C NL events timed out");
		return -ETIMEDOUT;
	}

#ifdef I2C_XEC_NL_DEBUG
	data->dbg_i2c_mcmd[2] = regs->MCMD;
	data->dbg_i2c_compl[2] = regs->COMPL;
	data->dbg_i2c_status[2] = regs->CTRLSTS;
#endif

	if (data->dma_err) {
		LOG_ERR("XEC I2C NL DMA error 0x%08x", data->dma_err);
		ret = -EIO;
	}

	if (msg->flags & I2C_MSG_STOP) {
		data->state = I2C_XEC_STATE_CLOSED;
	}

	return ret;
}

static int i2c_nl_xec_transfer(const struct device *dev, struct i2c_msg *msgs,
			       uint8_t num_msgs, uint16_t addr)
{
	struct i2c_nl_xec_data * const data = dev->data;
	const struct i2c_nl_xec_config * const cfg = dev->config;
	struct i2c_smb_regs * const regs = cfg->regs;
	struct i2c_msg *m = NULL;
	int ret = 0;
	uint8_t lines = 0;

	if (!msgs || !num_msgs) {
		return -EINVAL;
	}

	k_mutex_lock(&data->ctr_mutex, K_FOREVER);

#ifdef I2C_XEC_NL_DEBUG
	for (ret = 0; ret < 4; ret++) {
		data->dbg_i2c_mcmd[ret] = 0;
		data->dbg_i2c_compl[ret] = 0;
		data->dbg_i2c_status[ret] = 0;
	}
	ret = 0;
#endif

	data->event = I2C_XEC_EVENT_NONE;

	for (uint8_t i = 0; i < num_msgs; i++) {
		m = &msgs[i];

		if (data->state == I2C_XEC_STATE_CLOSED) {
			k_busy_wait(20);
			ret = get_lines(dev, &lines);
			if (ret) {
				break;
			}

			if (lines != I2C_LINES_BOTH_HI) {
				ret = i2c_nl_xec_recover_bus(dev);
				if (ret) {
					ret = -EIO;
					break;
				}
			}

			data->i2c_status = regs->CTRLSTS;
			if (data->i2c_status & MCHP_I2C_SMB_STS_BER) {
				ret = ctrl_config(dev);
				if (ret) {
					break;
				}
			}

			data->event = I2C_XEC_EVENT_START;
		}

		ret = i2c_nl_xec_xfr(dev, m, addr);
		if (ret) {
			break;
		}

		data->event = I2C_XEC_EVENT_NONE;
	}

	if (ret) {
		/* issue STOP */
		regs->CTRLSTS = (MCHP_I2C_SMB_CTRL_PIN | MCHP_I2C_SMB_CTRL_ESO |
				 MCHP_I2C_SMB_CTRL_STO | MCHP_I2C_SMB_CTRL_ACK);
		data->state = I2C_XEC_STATE_CLOSED;
	}

	k_mutex_unlock(&data->ctr_mutex);

	return ret;
}

static void i2c_nl_xec_isr(void *arg)
{
	const struct device *dev = arg;
	const struct i2c_nl_xec_config * const cfg = dev->config;
	struct i2c_nl_xec_data * const data = dev->data;
	struct i2c_smb_regs * const regs = cfg->regs;

	data->i2c_compl = regs->COMPL;
	data->i2c_status = regs->CTRLSTS;

	regs->COMPL = data->i2c_compl;

	data->nl_done = 1u;
	k_event_post(&data->ctr_events, BIT(0));
}

static const struct i2c_driver_api i2c_nl_xec_dma_api = {
	.configure = i2c_nl_xec_configure,
	.transfer = i2c_nl_xec_transfer,
	.get_config = i2c_nl_xec_get_config,
	.recover_bus = i2c_nl_xec_recover_bus,
};

static int i2c_nl_xec_init(const struct device *dev)
{
	const struct i2c_nl_xec_config * const cfg = dev->config;
	struct i2c_nl_xec_data * const data = dev->data;
	int ret;

	data->state = I2C_XEC_STATE_CLOSED;

	k_mutex_init(&data->ctr_mutex);
	k_event_init(&data->ctr_events);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("XEC I2C pinctrl setup failed (%d)", ret);
		return ret;
	}

	/* Default configuration */
	ret = i2c_nl_xec_configure(dev,
				   I2C_MODE_MASTER |
				   I2C_SPEED_SET(I2C_SPEED_STANDARD));
	if (ret) {
		return ret;
	}

	cfg->irq_config_func();
	mchp_xec_ecia_girq_src_en(cfg->girq, cfg->girq_pos);

	if (!device_is_ready(cfg->dma_dev)) {
		return -ENODEV;
	}

	return 0;
}

#define I2C_NL_XEC_DEVICE(n)						\
									\
	static struct i2c_nl_xec_data i2c_nl_xec_data_##n;		\
									\
	PINCTRL_DT_INST_DEFINE(n);					\
									\
	static void i2c_nl_xec_irq_config_func_##n(void)		\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    i2c_nl_xec_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQN(n));				\
	}								\
									\
	static const struct i2c_nl_xec_config i2c_nl_xec_config_##n = {	\
		.regs =	(struct i2c_smb_regs * const)DT_INST_REG_ADDR(n), \
		.port_sel = DT_INST_PROP(n, port_sel),			\
		.girq = DT_INST_PROP_BY_IDX(n, girqs, 0),		\
		.girq_pos = DT_INST_PROP_BY_IDX(n, girqs, 1),		\
		.pcr_idx = DT_INST_PROP_BY_IDX(n, pcrs, 0),		\
		.pcr_bitpos = DT_INST_PROP_BY_IDX(n, pcrs, 1),		\
		.ctr_dma_chan = MCHP_XEC_DT_INST_DMA_CHANNEL(n, ctr),	\
		.ctr_dma_id = MCHP_XEC_DT_INST_DMA_TRIGSRC(n, ctr),	\
		.irq_config_func = i2c_nl_xec_irq_config_func_##n,	\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		.dev_dma_chan = MCHP_XEC_DT_INST_DMA_CHANNEL(n, dev),	\
		.dev_dma_id = MCHP_XEC_DT_INST_DMA_TRIGSRC(n, dev),	\
		.dma_dev = DEVICE_DT_GET(MCHP_XEC_DT_INST_DMA_CTLR(n, ctr)), \
		.scl_gpio = GPIO_DT_SPEC_INST_GET(n, scl_gpios),	\
		.sda_gpio = GPIO_DT_SPEC_INST_GET(n, sda_gpios),	\
	};								\
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_nl_xec_init, NULL,		\
		&i2c_nl_xec_data_##n, &i2c_nl_xec_config_##n,		\
		POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,			\
		&i2c_nl_xec_dma_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_NL_XEC_DEVICE)
