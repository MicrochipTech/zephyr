/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip XEC I2Cv3 common functions
 */

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

#include "i2c_mchp_xec_regs.h"

/* --- Bitrate helpers -------------------------------------------------- */

int xec_i2c_v3_speed_to_freq(uint32_t speed, uint32_t *freq)
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

const uint16_t bus_clock_tbl[3] = {
	0x4F4FU, 0x0F17U, 0x0509U, /* 100k, 400k (1.53), 1000k (1.8) */
};

int xec_i2c_v3_bitrate_apply(mm_reg_t i2c_base, uint32_t speed, uint32_t dt_clock_freq)
{
	uint32_t v = 0, divisor = 0, half = 0;

	switch (speed) {
	case I2C_SPEED_STANDARD:
		v = bus_clock_tbl[0];
		break;
	case I2C_SPEED_FAST:
		v = bus_clock_tbl[1];
		break;
	case I2C_SPEED_FAST_PLUS:
		v = bus_clock_tbl[2];
		break;
	case I2C_SPEED_DT:
		/* TODO: we can't do this because we don't have DT speed from driver config struct! */
		if (dt_clock_freq == 0) {
			return -EINVAL;
		}

		divisor = XEC_I2C_SMB_BAUD_CLK / dt_clock_freq;
		if ((divisor < 2U) || (divisor > 512U) || (divisor & 1U)) {
			return -ERANGE;
		}

		half = (divisor / 2U) - 1U;
		v = XEC_I2C_BCLK_LOP_SET(half) | XEC_I2C_BCLK_HIP_SET(half);
		break;
	default:
		return -ERANGE;
	}

	soc_mmcr_mask_set(i2c_base + XEC_I2C_BCLK_OFS, v,
			  (XEC_I2C_BCLK_LOP_MSK | XEC_I2C_BCLK_HIP_MSK));

	return 0;
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
