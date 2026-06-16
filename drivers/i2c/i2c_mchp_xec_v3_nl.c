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

#define DT_DRV_COMPAT microchip,xec-i2c-v3-nl

#include <errno.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_xec_i2c_nl.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/clock/mchp_xec_pcr.h>
#include <zephyr/dt-bindings/i2c/mchp-xec-i2c.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_mchp_xec_v3_nl, CONFIG_I2C_LOG_LEVEL);

#include "i2c_mchp_xec_regs.h"

#define XEC_I2C_NL_BAUD_CLOCK_HZ MHZ(16)

/* I2C-NL controller is idle */
#define XEC_I2C_SR_IDLE BIT(XEC_I2C_SR_PIN_POS) | BIT(XEC_I2C_SR_NBB_POS)

/* --- Per-instance configuration (const, from DT) ---------------------- */

struct xec_i2c_nl_config {
	mm_reg_t base;
	uint8_t  girq;
	uint8_t  girq_pos;
	uint8_t  enc_pcr;
	void (*irq_connect)(void);
	const struct pinctrl_dev_config *pincfg;
};

/* --- Per-instance runtime data ---------------------------------------- */

enum i2c_tx_phase {
	PHASE_1_ADDRESS,
	PHASE_2_BULK,
	PHASE_3_FINAL_RPT_START
};

struct xec_i2c_nl_data {
#ifndef CONFIG_I2C_MCHP_XEC_V3_NL_PORT
	struct k_mutex lock;
#endif
	struct k_sem transfer_sem;
	struct k_sem pause_sem;
	int status_error;
    
	/* Asynchronous tracking variables */
	enum i2c_tx_phase tx_phase;
	uint32_t i2c_tx_reg_addr;
    
	/* Phase 2 context (Direct application RAM) */
	uint8_t *user_bulk_buf;
	size_t user_bulk_size;
    
	/* Phase 3 context (Driver bounce buffer) */
	uint8_t *tx_bounce_buf;
	uint8_t *rx_dest_buf;
	
	/* I2C control register is write-only, cache value written here */
	uint8_t i2c_cr_cache;
};

/* --- Register access helpers ------------------------------------------ */

#ifdef CONFIG_I2C_MCHP_XEC_V3_NL_PORT
/* App calls port driver. Port driver locks, switches port, and calls this driver */
static void xec_i2c_v3_nl_lock(const struct device *dev)
{
	return;
}

static void xec_i2c_v3_nl_unlock(const struct device *dev)
{
	return;
}
#else
static void xec_i2c_v3_nl_lock(const struct device *dev)
{
	struct xec_i2c_nl_data *const xdat = dev->data;

	k_mutex_lock(&xdat->lock, K_FOREVER);
}

static void xec_i2c_v3_nl_unlock(const struct device *dev)
{
	struct xec_i2c_nl_data *const xdat = dev->data;

	k_mutex_unlock(&xdat->lock);
}
#endif

static inline void i2c_ctrl_set(const struct device *dev, uint8_t crval)
{
	const struct xec_i2c_nl_config *cfg = dev->config;
	struct xec_i2c_nl_data const *xdat = dev->data;

	xdat->i2c_cr_cache = crval;
	sys_write8(cfg->base + XEC_I2C_CR_OFS, crval);
}

static bool xec_i2c_nl_is_busy(const struct device *dev)
{
	const struct xec_i2c_nl_config *cfg = dev->config;
	uint8_t i2c_sr = sys_read8(cfg->base + XEC_I2C_SR_OFS);

	if (i2c_sr == XEC_I2C_SR_IDLE) {
		return false;
	}

	return true;
}

/* --- Bitrate helpers -------------------------------------------------- */

static int xec_i2c_speed_to_freq(uint32_t speed, uint32_t *freq)
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

static int xec_i2c_bitrate_apply(const struct device *dev, uint32_t speed)
{
	const struct xec_i2c_nl_config *cfg = dev->config;
	uint32_t hz = 0, divisor = 0, half = 0;
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
		divisor = XEC_I2C_NL_BAUD_CLOCK_HZ / hz;
		if (divisor < 2U || divisor > 512U || (divisor & 1U) != 0U) {
			ret = -ERANGE;
		} else {
			half = ((divisor / 2U) - 1U);
			sys_write32(cfg->base + XEC_I2C_BCLK_OFS,
				    (XEC_I2C_BCLK_LOP_SET(half) | XEC_I2C_BCLK_HIP_SET(half)));
		}
	}

	return ret;
}

static uint32_t xec_i2c_bitrate_to_speed(uint32_t hz)
{
	uint32_t speed = 0;

	if (hz >= 1000000U) {
		speed = I2C_SPEED_FAST_PLUS;
	} else if (hz >= 400000U) {
		speed = I2C_SPEED_FAST;
	} else {
		speed = I2C_SPEED_STANDARD;
	}

	return speed;
}

static int xec_i2c_port_apply(const struct device *dev, uint8_t port)
{
	const struct xec_i2c_nl_config *xcfg = dev->config;
	uint32_t v = 0;

	if (port >= XEC_I2C_CFG_MAX_PORT) {
		return -ENOTSUP;
	}

	v = XEC_I2C_CFG_PORT_SET((uint32_t)port);
	soc_mmcr_mask_set(xcfg->base + XEC_I2C_CFG_OFS, v, XEC_I2C_CFG_PORT_MSK);

	return 0;
}

static int config_hw(const struct device *dev, uint32_t freq_hz, uint8_t port, bool ien)
{
	const struct xec_i2c_nl_config *xcfg = dev->config;
	struct i2c_mec5_data *const xdat = dev->data;
	uint32_t speed = xec_i2c_bitrate_to_speed(freq_hz);
	uint32_t regv = 0;
	int i = 0, ret = 0;

	soc_ecia_girq_ctrl(xcfg->girq, xcfg->girq_pos, MCHP_MEC_ECIA_GIRQ_DIS);

	soc_xec_pcr_reset_en(xcfg->enc_pcr);
	sys_set_bit(xcfg->base + I2C_XEC_CFG_OFS, XEC_I2C_CFG_RST_POS);
	for (i = 0; i < 16U; i++) {
		i2c_nl_config_r(xcfg);
	}
	sys_clear_bit(xcfg->base + I2C_XEC_CFG_OFS, XEC_I2C_CFG_RST_POS);

	/* reset disables and defaults to 100 KHz bus clock */

	/* Section 6. Table 6-4 */
	i2c_ctrl_set(dev, BIT(XEC_I2C_CR_PIN_POS));

	/* disable response to I2C General call and enable digital filter */
	sys_set_bits(xcfg->base + XEC_I2C_CFG_OFS, (BIT(XEC_I2C_CFG_GC_DIS_POS) |
						    BIT(XEC_I2C_CFG_FEN_POS));

	ret = xec_i2c_bitrate_apply(dev, speed);
	if (ret) {
		return ret;
	}

	ret = xec_i2c_port_apply(dev, port);
	if (ret) {
		return ret;
	}

	xdat->active_freq = freq_hz;
	xdat->active_port = port;

	/* Enable auto-ACK of received data and matching addresses */
	i2c_ctrl_set(dev, (BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ACK_POS));

	/* Enable live SCL/SDA pin states in BB-Control. HW V3.8 and above */
	sys_set_bit(xcfg->base + XEC_I2C_BBCR_OFS, XEC_I2C_BBCR_CM_POS);

	/* Enable controller */
	sys_set_bit(xcfg->base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);

	k_busy_wait(I2C_MEC5_POST_RESET_PIN_WAIT_US);

	/* clear I2C.COMPLETION R/W1C status */
	sys_set_bits(xcfg->base + XEC_I2C_CMPL_OFS, XEC_I2C_CMPL_RW1C_MSK);

	soc_ecia_girq_status_clear(xcfg->girq, xcfg->girq_pos);
	if (ien == true) {
		soc_ecia_girq_ctrl(xcfg->girq, xcfg->girq_pos, MCHP_MEC_ECIA_GIRQ_EN);
	}

	return 0;
}

static uint32_t get_bus_clock_from_hw(const struct device *dev)
{
	const struct xec_i2c_nl_config *cfg = dev->config;
	uint16_t bclk = sys_read32(cfg->base + XEC_I2C_BCLK_OFS);
	uint32_t low = XEC_I2C_BCLK_LOP_GET(bclk);
	uint32_t high = XEC_I2C_BCLK_HIP_GET(bclk)p
	uint32_t divisor = (low + 1U) + (high + 1U);
	uint32_t hz = XEC_I2C_NL_BAUD_CLOCK_HZ / divisor;

	return hz;
}

/* --- Zephyr API: recover_bus ----------------------------------------- */

static int xec_i2c_bb_check_scl(const struct device *ctrl, uint16_t nloops)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	uint8_t bbcr = sys_read8(cfg->base + XEC_I2C_BBCR_OFS);

	while (((bbcr & BIT(XEC_I2C_BBCR_SCL_IN_POS)) == 0) && (nloops != 0)) {
		k_busy_wait(10U);
		bbcr = sys_read8(cfg->base + XEC_I2C_BBCR_OFS);
		nloops--;
	}

	if ((bbcr & BIT(XEC_I2C_BBCR_SCL_IN_POS)) == 0) {
		sys_write8(BIT(XEC_I2C_BBCR_CM_POS), cfg->base + XEC_I2C_BBCR_OFS);
		return -ETIMEDOUT;
	}

	return 0;
}

/* Generate requested number of I2C clocks
 * Each clock pulse is a transition from low to high and back to low.
 * Caller is required to configure BB control of SCL and SDA as outputs and tri-stated(not driven).
 */
static void xec_i2c_bb_gen_clocks(const struct device *ctrl, uint16_t num_clocks)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	uint8_t bbcr = sys_read8(cfg->base + XEC_I2C_BBCR_OFS);
	uint8_t bbcr_scl_hi = bbcr;
	uint8_t bbcr_scl_lo = bbcr & (uint8_t)~(XEC_I2C_BBCR_SCL_POS);

	if ((num_clocks != 0) && ((bbcr & BIT(XEC_I2C_BBCR_SCL_IN_POS)) != 0)) {
		/* begin with SCL low */
		sys_write8(bbcr_scl_lo, cfg->base + XEC_I2C_BBCR_OFS);
		k_busy_wait(5U);
	}

	for (uint16_t n = 0; n < num_clocks; n++) {
		sys_write8(bbcr_scl_hi, cfg->base + XEC_I2C_BBCR_OFS); /* low-to-high */
		k_busy_wait(5U);
		sys_write8(bbcr_scl_lo, cfg->base + XEC_I2C_BBCR_OFS); /* high-to-low */
		k_busy_wait(5U);
	}

	/* Release SCL so we don't appear to be clock stretching */
	bbcr = sys_read8(cfg->base + XEC_I2C_BBCR_OFS);
	if ((bbcr & BIT(XEC_I2C_BBCR_SCL_IN_POS)) == 0) {
		sys_write8(bbcr_scl_hi, cfg->base + XEC_I2C_BBCR_OFS); /* tri-state SCL */
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
static int xec_i2c_bb_gen_stop(const struct device *ctrl)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	uint8_t bbcr = sys_read8(cfg->base + XEC_I2C_BBCR_OFS);

	/* Is SCL low? */
	if ((bbcr & BIT(XEC_I2C_BBCR_SCL_IN_POS)) == 0) {
		/* Yes, release SCL and delay to let pull-up pull it high */
		bbcr |= BIT(XEC_I2C_BBCR_SCL_POS);
		sys_write8(bbcr, cfg->base + XEC_I2C_BBCR_OFS);
		k_busy_wait(5U);
	}

	/* Drive SDA low while SCL is high for 1/2 100KHz clock */
	bbcr &= (uint8_t)~BIT(XEC_I2C_BBCR_SDA_POS);
	sys_write8(bbcr, cfg->base + XEC_I2C_BBCR_OFS);
	k_busy_wait(5U);

	/* release SDA */
	bbcr |= BIT(XEC_I2C_BBCR_SDA_POS);
	sys_write8(bbcr, cfg->base + XEC_I2C_BBCR_OFS);
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
static int xec_i2c_bb_recover(const struct device *ctrl, uint32_t freq, uint8_t port)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	int ret = 0;
	uint32_t n = 0;
	uint8_t bbcr = 0, i2c_sr = 0;
	uint8_t both_hi_msk = (BIT(XEC_I2C_BBCR_SCL_IN_POS) | BIT(XEC_I2C_BBCR_SDA_IN_POS));

	ret = config_hw(ctrl, freq, port, false);
	bbcr = sys_read8(cfg->base + XEC_I2C_BBCR_OFS);
	if (ret == 0) {
		i2c_sr = sys_read8(cfg->base + XEC_I2C_SR_OFS);
		if (((bbcr & both_hi_msk) == both_hi_msk) && (i2c_sr == XEC_I2C_SR_IDLE)) {
			soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, MCHP_MEC_ECIA_GIRQ_EN);
			return 0;
		}
	}

	ret = xec_i2c_bb_check_scl(ctrl, 100U);
	if (ret != 0) {
		LOG_ERR("I2C recov: SCL stuck low");
		return ret;
	}

	/* Enable Bit-Bang control of SCL and SDA with as outputs and tri-stated */
	bbcr = (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CM_POS) | BIT(XEC_I2C_BBCR_CD_POS) |
		BIT(XEC_I2C_BBCR_DD_POS) | BIT(XEC_I2C_BBCR_SCL_POS) | BIT(XEC_I2C_BBCR_SDA_POS));
	sys_write8(bbcr, cfg->base + XEC_I2C_BBCR_OFS);

	n = 100U;
	while (n != 0) {
		xec_i2c_bb_gen_clocks(ctrl, 9U);
		k_busy_wait(10U);
		/* Generate an I2C STOP */
		ret = xec_i2c_bb_gen_stop(ctrl);
		if (ret) {
			LOG_ERR("I2C recov: cannot gen STOP");
			return ret;
		}

		k_busy_wait(10U);
		bbcr = sys_read8(cfg->base + XEC_I2C_BBCR_OFS);

		if ((bbcr & both_hi_msk) == both_hi_msk) {
			break;
		}

		n--;
	}

	bbcr = BIT(XEC_I2C_BBCR_CM_POS);
	sys_write8(bbcr, cfg->base + XEC_I2C_BBCR_OFS);
	k_busy_wait(10U);

	ret = config_hw(ctrl, freq, port, false);

	bbcr = sys_read8(cfg->base + XEC_I2C_BBCR_OFS);
	if ((bbcr & both_hi_msk) != both_hi_msk) {
		LOG_INF("I2C recov failed");
		return -EIO;
	}

	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, MCHP_MEC_ECIA_GIRQ_EN);

	return 0;
}

/* ---- API: transfer ---- */

static int i2c_dma_execute_read_phase(const struct device *dev, struct i2c_msg *rx_msg)
{
	const struct i2c_dma_config *config = dev->config;
	struct i2c_dma_data *data = dev->data;

	/*
	 * STEP 1: AWAIT HARDWARE PAUSE STATE
	 * The thread blocks here while the TX DMA callback chain pushes out the bulk 
	 * data and the controller finishes the Repeated Start address byte.
	 */
	k_sem_take(&data->pause_sem, K_FOREVER);

	/* Abort immediately if the transmit phase threw an error (like an address NAK) */
	if (data->status_error != 0) {
		return data->status_error;
	}

	/*
	 * STEP 2: RECONFIGURE DMA CHANNEL FOR RECEPTION
	 * Explicitly stop the channel to cleanly swap directions from TX to RX.
	 */
	dma_stop(config->dma_dev, config->dma_channel);

	struct dma_block_config rx_dma_block = {
		.source_address = config->base_addr + 0x58,  // RX Data Register Offset
		.dest_address = (uint32_t)rx_msg->buf,       // Points directly to user's RX memory
		.block_size = rx_msg->len,                   // Read count requested by Zephyr application
	};

	struct dma_config rx_dma_cfg = {
		.channel_direction = PERIPHERAL_TO_MEMORY,  // Direction flipped!
		.source_data_size = 1,
		.dest_data_size = 1,
		.block_count = 1,
		.head_block = &rx_dma_block,
		.dma_callback = i2c_rx_dma_callback,         // Attach the reception callback
		.user_data = data,
	};

	int ret = dma_config(config->dma_dev, config->dma_channel, &rx_dma_cfg);
	if (ret < 0) {
		return ret;
	}

	/* Arm the DMA engine. It is now waiting for the I2C hardware to assert REQUEST. */
	dma_start(config->dma_dev, config->dma_channel);

	/* =========================================================================
	 * STEP 3: UNLEASH THE I2C HARDWARE FSM FROM PAUSE
	 * Read the live command register, safely append the PROCEED bit, and write it back.
	 */
	uint32_t live_cmd = sys_read32(config->base_addr + 0xC);
	live_cmd |= (1 << 1);  // PROCEED = 1
    
	sys_write32(live_cmd, config->base_addr + 0xC);

	/*
	 * STEP 4: AWAIT ABSOLUTE TRANSACTION COMPLETION
	 * The thread blocks safely here. The I2C controller clocks in data bytes,
	 * asserts REQUEST, the DMA copies them to rx_msg->buf, and once the 
	 * read_count hits 0, the I2C hardware generates a STOP and fires HDONE.
	 */
	k_sem_take(&data->transfer_sem, K_FOREVER);

	return data->status_error;
}

static int i2c_dma_kickoff_tx_pipeline(const struct device *dev, uint16_t addr, uint32_t write_cnt,
				       uint32_t read_cnt, bool has_read)
{
	const struct i2c_dma_config *config = dev->config;
	struct i2c_dma_data *data = dev->data;

	data->tx_bounce_buf = (addr << 1) | 0; // Phase 1 address byte
	data->i2c_tx_reg_addr = config->base_addr + 0x54;

	struct dma_block_config dma_block = {
		.source_address = (uint32_t)&data->tx_bounce_buf,
		.dest_address = data->i2c_tx_reg_addr,
		.block_size = 1,
	};

	struct dma_config dma_cfg = {
		.channel_direction = MEMORY_TO_PERIPHERAL,
		.source_data_size = 1,
		.dest_data_size = 1,
		.block_count = 1,
		.head_block = &dma_block,
		.dma_callback = i2c_tx_dma_callback,
		.user_data = data,
	};

	int ret = dma_config(config->dma_dev, config->dma_channel, &dma_cfg);
	if (ret < 0) {
		return ret;
	}

	dma_start(config->dma_dev, config->dma_channel);

	/* Construct Live Command Register Image */
	uint32_t cmd = (1 << 0)  | // RUN = 1
			(1 << 8)  | // START0 = 1
			(1 << 10) | // STOP = 1
			(write_cnt << 15) |
			(read_cnt << 16);

	if (has_read) {
		cmd |= (1 << 9); // STARTN = 1
	}

	sys_write32(cmd, config->base_addr + 0xC);

	return 0;
}

static int xec_i2c_nl_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			       uint16_t addr)
{
	struct xec_i2c_nl_data *data = dev->data;
	struct i2c_msg *write_msg = NULL;
	struct i2c_msg *read_msg = NULL;

	/* =========================================================================
	 * STEP 1: VALIDATE ZEPHYR MESSAGE CHAINS AGAINST HW CONSTRAINTS
	 * ========================================================================= */
	if (num_msgs == 0 || msgs == NULL) {
		return -EINVAL;
	}

	/* Analyze the message array structure */
	for (uint8_t i = 0; i < num_msgs; i++) {
		if ((msgs[i].flags & I2C_MSG_READ) == 0) {
			/* It's a write message */
			if (write_msg != NULL) {
				/* Hardware limitation: We can only accept ONE consolidated write
				 * buffer because we direct-map Phase 2 to a single contiguous
				 * user pointer.
				 */
				return -ENOTSUP; 
			}
			write_msg = &msgs[i];
		} else {
			/* It's a read message */
			if (read_msg != NULL) {
				/* Hardware can only do a single continuous read phase */
				return -ENOTSUP;
			}
			/* Check mandatory layout rule: Reads MUST follow a Repeated START */
			if (!(msgs[i].flags & I2C_MSG_RESTART)) {
				return -ENOTSUP; 
			}
			read_msg = &msgs[i];
		}
	}

	/* Edge Case Validation */
	if (write_msg == NULL && read_msg != NULL) {
		/* Hardware cannot do a pure read! The FSM requires START0/STARTN logic 
		 * which forces an initial write transaction to transmit target 
		 * addresses.
		 */
		return -ENOTSUP; 
	}

	/* =========================================================================
	 * STEP 2: PRE-CALCULATE HARDWARE LIVE REGISTER TRANSACTIONS
	 * ========================================================================= */
	uint32_t total_write_count = 0;
	uint32_t total_read_count = 0;

	if (write_msg != NULL) {
		/* Phase 1 (1B Address) + Phase 2/3 (Payload length) */
		total_write_count += 1 + write_msg->len; 
	}

	if (read_msg != NULL) {
		/* Phase 3 requires packing the Repeated Start device address (1B) */
		total_write_count += 1; 
		total_read_count = read_msg->len;
	}

	/* Enforce baseline hardware floor: To execute our 3-Phase asymmetric DMA reload,
	 * the write payload buffer must be at least 3 bytes long so Phase 3 math works.
	 */
	if (write_msg->len < 3) {
		return -EINVAL; 
	}

	/* Reset driver execution tracking flags safely inside a thread context */
	data->status_error = 0;

	/* =========================================================================
	 * STEP 3: EXECUTE TRANSACTION FLOW
	 * ========================================================================= */
	if (read_msg == NULL) {
		/* ---------------------------------------------------------------------
		 * FLOW A: PURE WRITE TRANSACTION
		 * --------------------------------------------------------------------- */
		// Configure asymmetric async stage context
		data->tx_phase = PHASE_1_ADDRESS;
		data->user_bulk_size = write_msg->len - 3;
		data->user_bulk_buf = write_msg->buf;

		// Stage final buffer slice (No read address appended for pure writes)
		uint32_t final_offset = write_msg->len - 3;
		memcpy(&data->tx_bounce_buf, &write_msg->buf[final_offset], 3);

		// Fire the pipeline (Re-uses async DMA configuration function from earlier)
		int ret = i2c_dma_kickoff_tx_pipeline(dev, addr, total_write_count, 0, false);
		if (ret < 0) return ret;
		
		// Block on transfer_sem. Pure write ISR falls straight through to completion.
		k_sem_take(&data->transfer_sem, K_FOREVER);
	} else {
		/* ---------------------------------------------------------------------
		 * FLOW B: WRITE-THEN-READ (REPEATED START) TRANSACTION
		 * --------------------------------------------------------------------- */
		// Configure asymmetric async stage context
		data->tx_phase = PHASE_1_ADDRESS;
		data->user_bulk_size = write_msg->len - 3;
		data->user_bulk_buf = write_msg->buf;

		// Stage final buffer slice and pack repeated-start target read address
		uint32_t final_offset = write_msg->len - 3;
		memcpy(&data->tx_bounce_buf, &write_msg->buf[final_offset], 3);
		data->tx_bounce_buf = (addr << 1) | 1; // 4th byte: Read target address!

		// Fire transmission pipeline
		int ret = i2c_dma_kickoff_tx_pipeline(dev, addr, total_write_count, total_read_count, true);
		if (ret < 0) return ret;

		// Execute subsequent Read Phase (handles pause_sem blocking & DMA flipping)
		ret = i2c_dma_execute_read_phase(dev, read_msg);
		if (ret < 0) return ret;
	}

	/* Return standard Zephyr API code (0 for success, negative for failure) */
	return data->status_error;
}

/* ---- ISR ---- */

static void xec_i2c_nl_isr(const struct device *dev)
{
	/* TODO */
}

/* --- Init ------------------------------------------------------------ */

static int xec_i2c_nl_init(const struct device *dev)
{
	/* TODO */
	return 0;
}

/* --- Driver API struct ----------------------------------------------- */

static DEVICE_API(i2c, xec_i2c_nl_driver_api) = {
	.configure     = xec_i2c_nl_configure,
	.get_config    = xec_i2c_nl_get_config,
	.transfer      = xec_i2c_nl_transfer,
	.recover_bus   = xec_i2c_nl_recover_bus,
#ifdef CONFIG_I2C_RTIO
	.iodev_submit  = i2c_iodev_submit_fallback,
#endif
};

/* --- Per-instance instantiation -------------------------------------- */
#define XEC_I2C_NL_GIRQ_DT(inst) MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, 0))
#define XEC_I2C_NL_GIRQ_POS_DT(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, 0))

#define XEC_I2C_NL_INIT(inst)                                                  \
	PINCTRL_DT_INST_DEFINE(inst);                                          \
	static void xec_i2c_nl_irq_connect_##inst(void)                        \
	{                                                                      \
		IRQ_CONNECT(DT_INST_IRQN(inst),                                \
			    DT_INST_IRQ(inst, priority),                       \
			    i2c_mec5_isr,                                      \
			    DEVICE_DT_INST_GET(inst), 0);                      \
		irq_enable(DT_INST_IRQN(inst));                                \
	}                                                                           \
	static uint8_t tx_bounce_buf##inst[DT_INST_PROP(inst, bounce_buffer_size)]; \
	static struct xec_i2c_nl_data xec_i2c_nl_devdat##inst = {                   \
		.tx_bounce_buf = tx_bounce_buf##inst,                               \
	};                                                                          \
	static const struct xec_i2c_nl_config xec_i2c_nl_devcfg##inst = {      \
		.base = (mm_reg_t)DT_INST_REG_ADDR(inst),                      \
		.girq = (uint8_t)XEC_I2C_NL_GIRQ_DT(inst),                     \
		.girq_pos (uint8_t)XEC_I2C_NL_GIRQ_POS_DT(inst),               \
		.enc_pcr = DT_INST_PROP(inst, pcr_scr),                        \
		.irq_connect = xec_i2c_nl_irq_connect_##inst,                  \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                \
	};                                                                     \
	I2C_DEVICE_DT_INST_DEFINE(inst,                                        \
				  xec_i2c_nl_init, NULL,                       \
				  &xec_i2c_nl_devdat##inst,                    \
				  &xec_i2c_nl_devcfg##inst,                    \
				  POST_KERNEL,                                 \
				  CONFIG_I2C_INIT_PRIORITY,                    \
				  &xec_i2c_nl_driver_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_INIT)
