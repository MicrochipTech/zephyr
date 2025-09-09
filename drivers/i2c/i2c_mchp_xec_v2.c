/*
 * Copyright (c) 2019 Intel Corporation
 * Copyright (c) 2021 Microchip Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_i2c_v2

#include <soc.h>
#include <errno.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_xec_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/interrupt_controller/intc_mchp_xec_ecia.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(i2c_mchp, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

#define XEC_I2C_BM_USE_ZEPHYR_REG_ACCESS

#define XEC_I2C_CFG_CLR_BUFS (BIT(XEC_I2C_CFG_FTTX_POS) | BIT(XEC_I2C_CFG_FTRX_POS) | \
			      BIT(XEC_I2C_CFG_FHTX_POS) | BIT(XEC_I2C_CFG_FHRX_POS))
#define XEC_I2C_BBCR_EN_SCL_SDA_OUT \
	(BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_SCL_POS) | BIT(XEC_I2C_BBCR_SDA_POS))
#define XEC_I2C_BBCR_EN_SCL_OUT_SDA_TS \
	(BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CD_POS) | BIT(XEC_I2C_BBCR_SCL_POS) | \
	 BIT(XEC_I2C_BBCR_SDA_POS))
#define XEC_I2C_BBCR_EN_SCL_OUT_LO_SDA_TS \
	(BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CD_POS) | BIT(XEC_I2C_BBCR_SDA_POS))
#define XEC_I2C_BBCR_SCL_SDA_IN_MSK (BIT(XEC_I2C_BBCR_SCL_IN_POS) | BIT(XEC_I2C_BBCR_SDA_IN_POS))

#define XEC_I2C_CR_CLR_PIN_ESO_ACK \
	(BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS))

#define XEC_I2C_CR_CLR_PIN_ESO_ENI_ACK \
	(XEC_I2C_CR_CLR_PIN_ESO_ACK | BIT(XEC_I2C_CR_ENI_POS))


#define SPEED_100KHZ_BUS	0
#define SPEED_400KHZ_BUS	1
#define SPEED_1MHZ_BUS		2

#define EC_OWN_I2C_ADDR		0x7F
#define RESET_WAIT_US		20

/* I2C timeout is  10 ms (WAIT_INTERVAL * WAIT_COUNT) */
#define WAIT_INTERVAL		50
#define WAIT_COUNT		200
#define STOP_WAIT_COUNT		500
#define PIN_CFG_WAIT		50

/* I2C Read/Write bit pos */
#define I2C_READ_WRITE_POS	0

/* I2C recover SCL low retries */
#define I2C_RECOVER_SCL_LOW_RETRIES	10
/* I2C recover SDA low retries */
#define I2C_RECOVER_SDA_LOW_RETRIES	3
/* I2C recovery bit bang delay */
#define I2C_RECOVER_BB_DELAY_US		5
/* I2C recovery SCL sample delay */
#define I2C_RECOVER_SCL_DELAY_US	50

/* I2C SCL and SDA lines(signals) */
#define I2C_LINES_SCL_HI	BIT(0)
#define I2C_LINES_SDA_HI	BIT(1)
#define I2C_LINES_BOTH_HI	(I2C_LINES_SCL_HI | I2C_LINES_SDA_HI)

#define I2C_START		0U
#define I2C_RPT_START		1U

#define I2C_ENI_DIS		0U
#define I2C_ENI_EN		1U

#define I2C_WAIT_PIN_DEASSERT	0U
#define I2C_WAIT_PIN_ASSERT	1U

#define I2C_XEC_CTRL_WR_DLY	8

#define I2C_XEC_STATE_STOPPED	1U
#define I2C_XEC_STATE_OPEN	2U

#define I2C_XEC_OK		0
#define I2C_XEC_ERR_LAB		1
#define I2C_XEC_ERR_BUS		2
#define I2C_XEC_ERR_TMOUT	3

#define XEC_GPIO_CTRL_BASE	DT_REG_ADDR(DT_NODELABEL(gpio_000_036))

struct xec_speed_cfg {
	uint32_t bus_clk;
	uint32_t data_timing;
	uint32_t start_hold_time;
	uint32_t idle_scale;
	uint32_t timeout_scale;
};

struct i2c_xec_config {
	uint32_t port_sel;
	uint32_t base_addr;
	uint32_t clock_freq;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t pcr;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
};

struct i2c_xec_data {
	uint8_t state;
	uint8_t read_discard;
	uint8_t speed_id;
	struct i2c_target_config *target_cfg;
	bool target_attached;
	bool target_read;
	uint32_t i2c_compl;
	uint8_t i2c_ctrl;
	uint8_t i2c_addr;
	uint8_t i2c_status;
};

/* Recommended programming values based on 16MHz
 * i2c_baud_clk_period/bus_clk_period - 2 = (low_period + hi_period)
 * bus_clk_reg (16MHz/100KHz -2) = 0x4F + 0x4F
 *             (16MHz/400KHz -2) = 0x0F + 0x17
 *             (16MHz/1MHz -2) = 0x05 + 0x09
 */
static const struct xec_speed_cfg xec_cfg_params[] = {
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

static void i2c_ctl_wr(const struct device *dev, uint8_t ctrl)
{
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data = dev->data;
	mem_addr_t i2c_base = (mem_addr_t)cfg->base_addr;
	data->i2c_ctrl = ctrl;
	sys_write8(ctrl, i2c_base + XEC_I2C_CR_OFS);

	for (int i = 0; i < I2C_XEC_CTRL_WR_DLY; i++) {
		sys_write8(0, i2c_base + XEC_I2C_BLKID_OFS);
	}
}

static int i2c_xec_reset_config(const struct device *dev);

static int wait_bus_free(const struct device *dev, uint32_t nwait)
{
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data = dev->data;
	mem_addr_t i2c_base = (mem_addr_t)cfg->base_addr;
	uint32_t count = nwait;
	uint8_t sts = 0;

	while (count--) {
		sts = sys_read8(i2c_base + XEC_I2C_SR_OFS);
		data->i2c_status = sts;
		if ((sts & BIT(XEC_I2C_SR_NBB_POS)) != 0) {
			break; /* bus is free */
		}
		k_busy_wait(WAIT_INTERVAL);
	}

	/* NBB -> 1 not busy can occur for STOP, BER, or LAB */
	if (sts == (BIT(XEC_I2C_SR_PIN_POS) | BIT(XEC_I2C_SR_NBB_POS))) {
		/* No service requested(PIN=1), NotBusy(NBB=1), and no errors */
		return 0;
	}

	if ((sts & BIT(XEC_I2C_SR_BER_POS)) != 0) {
		return I2C_XEC_ERR_BUS;
	}

	if ((sts & BIT(XEC_I2C_SR_LAB_POS)) != 0) {
		return I2C_XEC_ERR_LAB;
	}

	return I2C_XEC_ERR_TMOUT;
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
static uint32_t get_lines(const struct device *dev)
{
	const struct i2c_xec_config *cfg = dev->config;
	mem_addr_t i2c_base = (mem_addr_t)cfg->base_addr;
	uint8_t bbcr = sys_read8(i2c_base + XEC_I2C_BBCR_OFS);
	uint32_t lines = 0u;

	if ((bbcr & BIT(XEC_I2C_BBCR_SCL_IN_POS)) != 0) {
		lines |= BIT(0);
	}
	if ((bbcr & BIT(XEC_I2C_BBCR_SDA_IN_POS)) != 0) {
		lines |= BIT(1);
	}

	return lines;
}

static int i2c_xec_reset_config(const struct device *dev)
{
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data = dev->data;
	mem_addr_t i2c_base = (mem_addr_t)cfg->base_addr;
	uint32_t temp = 0;

	data->state = I2C_XEC_STATE_STOPPED;
	data->read_discard = 0;

	/* Assert RESET */
	xec_pcr_sleep_en_clear(cfg->pcr);
	xec_pcr_reset_en(cfg->pcr);

	sys_write32(XEC_I2C_CFG_CLR_BUFS, i2c_base + XEC_I2C_CFG_OFS);
	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);

	/* PIN=1 to clear all status except NBB and synchronize */
	i2c_ctl_wr(dev, BIT(XEC_I2C_CR_PIN_POS));

	/*
	 * Controller implements two peripheral addresses for itself.
	 * It always monitors whether an external controller issues START
	 * plus target address. We should write valid peripheral addresses
	 * that do not match any peripheral on the bus.
	 * An alternative is to use the default 0 value which is the
	 * general call address and disable the general call match
	 * enable in the configuration register.
	 */
	temp = XEC_I2C_OA_1_SET(EC_OWN_I2C_ADDR) | XEC_I2C_OA_2_SET(EC_OWN_I2C_ADDR);
	sys_write32(temp, i2c_base + XEC_I2C_OA_OFS);

#ifdef CONFIG_I2C_TARGET
	if (data->target_cfg) {
		temp &= ~XEC_I2C_OA_1_MSK;
		temp |= XEC_I2C_OA_1_SET(data->target_cfg->address);
		sys_write32(temp, i2c_base + XEC_I2C_OA_OFS);
	}
#endif

	/* Port number and filter enable MUST be written before enabling */
	temp = sys_read32(i2c_base + XEC_I2C_CFG_OFS) & ~XEC_I2C_CFG_PORT_MSK;
	temp |= XEC_I2C_CFG_PORT_SET(cfg->port_sel);
	temp |= BIT(XEC_I2C_CFG_FEN_POS) | BIT(XEC_I2C_CFG_GC_POS);

	/*
	 * Before enabling the controller program the desired bus clock,
	 * repeated start hold time, data timing, and timeout scaling
	 * registers.
	 */
	sys_write32(xec_cfg_params[data->speed_id].bus_clk, i2c_base + XEC_I2C_BCLK_OFS);
	sys_write32(xec_cfg_params[data->speed_id].start_hold_time, i2c_base + XEC_I2C_RSHT_OFS);
	sys_write32(xec_cfg_params[data->speed_id].data_timing, i2c_base + XEC_I2C_DT_OFS);
	sys_write32(xec_cfg_params[data->speed_id].timeout_scale, i2c_base + XEC_I2C_TMOUT_SC_OFS);
	sys_write32(xec_cfg_params[data->speed_id].idle_scale, i2c_base + XEC_I2C_ISC_OFS);

	/*
	 * PIN=1 clears all status except NBB
	 * ESO=1 enables output drivers
	 * ACK=1 enable ACK generation when data/address is clocked in.
	 */
	i2c_ctl_wr(dev, (BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) |
			 BIT(XEC_I2C_CR_ACK_POS)));

	/* Enable controller */
	sys_set_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);
	sys_write8(BIT(XEC_I2C_BBCR_CM_POS), i2c_base + XEC_I2C_BBCR_OFS);

	k_busy_wait(RESET_WAIT_US);

	/* wait for NBB=1, BER, LAB, or timeout */
	return wait_bus_free(dev, WAIT_COUNT);
}

/*
 * If SCL is low sample I2C_RECOVER_SCL_LOW_RETRIES times with a 5 us delay
 * between samples. If SCL remains low then return -EBUSY
 * If SCL is High and SDA is low then loop up to I2C_RECOVER_SDA_LOW_RETRIES
 * times driving the pins:
 * Drive SCL high
 * delay I2C_RECOVER_BB_DELAY_US
 * Generate 9 clock pulses on SCL checking SDA before each falling edge of SCL
 *   If SDA goes high exit clock loop else to all 9 clocks
 * Drive SDA low, delay 5 us, release SDA, delay 5 us
 * Both lines are high then exit SDA recovery loop
 * Both lines should not be driven
 * Check both lines: if any bad return error else return success
 * NOTE 1: Bit-bang mode uses a HW MUX to switch the lines away from the I2C
 * controller logic to BB logic.
 * NOTE 2: Bit-bang mode requires HW timeouts to be disabled.
 * NOTE 3: Bit-bang mode requires the controller's configuration enable bit
 * to be set.
 * NOTE 4: The controller must be reset after using bit-bang mode.
 */
static int i2c_xec_recover_bus(const struct device *dev)
{
	const struct i2c_xec_config *cfg = dev->config;
	mem_addr_t i2c_base = (mem_addr_t)cfg->base_addr;
	uint32_t r = 0;
	int i = 0, j = 0, ret = 0;
	uint8_t bb = 0;

	LOG_ERR("I2C attempt bus recovery\n");

	/* reset controller to a known state */
	xec_pcr_reset_en(cfg->pcr);

	r = (BIT(XEC_I2C_CFG_GC_POS) | BIT(XEC_I2C_CFG_FEN_POS) |
	     XEC_I2C_CFG_PORT_SET(cfg->port_sel));
	sys_write32(r, i2c_base + XEC_I2C_CFG_OFS);
	sys_set_bits(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_CLR_BUFS);
	sys_write8(BIT(XEC_I2C_CR_PIN_POS), i2c_base + XEC_I2C_CR_OFS);
	sys_write8(XEC_I2C_BBCR_EN_SCL_SDA_OUT, i2c_base + XEC_I2C_BBCR_OFS);

	if (sys_test_bit(i2c_base + XEC_I2C_BBCR_OFS, XEC_I2C_BBCR_SCL_IN_POS) == 0) {
		for (i = 0;; i++) {
			if (i >= I2C_RECOVER_SCL_LOW_RETRIES) {
				ret = -EBUSY;
				goto recov_exit;
			}
			k_busy_wait(I2C_RECOVER_SCL_DELAY_US);
			if (sys_test_bit(i2c_base + XEC_I2C_BBCR_OFS, XEC_I2C_BBCR_SCL_IN_POS)) {
				break; /* SCL went High */
			}
		}
	}

	if (sys_test_bit(i2c_base + XEC_I2C_BBCR_OFS, XEC_I2C_BBCR_SDA_IN_POS) != 0) {
		ret = 0;
		goto recov_exit;
	}

	ret = -EBUSY;
	/* SDA recovery */
	for (i = 0; i < I2C_RECOVER_SDA_LOW_RETRIES; i++) {
		/* SCL output mode and tri-stated */
		sys_write8(XEC_I2C_BBCR_EN_SCL_OUT_SDA_TS, i2c_base + XEC_I2C_BBCR_OFS);
		k_busy_wait(I2C_RECOVER_BB_DELAY_US);

		for (j = 0; j < 9; j++) {
			if (sys_test_bit(i2c_base + XEC_I2C_BBCR_OFS,
					 XEC_I2C_BBCR_SDA_IN_POS) != 0) {
				break;
			}

			/* drive SCL low */
			sys_write8(XEC_I2C_BBCR_EN_SCL_OUT_LO_SDA_TS, i2c_base + XEC_I2C_BBCR_OFS);
			k_busy_wait(I2C_RECOVER_BB_DELAY_US);
			/* release SCL: pulled high by external pull-up */
			sys_write8(XEC_I2C_BBCR_EN_SCL_OUT_SDA_TS, i2c_base + XEC_I2C_BBCR_OFS);
			k_busy_wait(I2C_RECOVER_BB_DELAY_US);
		}

		/* SCL is High. Produce rising edge on SCL for STOP by driving SCL low */
		sys_write8(XEC_I2C_BBCR_EN_SCL_OUT_LO_SDA_TS, i2c_base + XEC_I2C_BBCR_OFS);
		k_busy_wait(I2C_RECOVER_BB_DELAY_US);
		sys_write8(XEC_I2C_BBCR_EN_SCL_OUT_SDA_TS, i2c_base + XEC_I2C_BBCR_OFS);
		k_busy_wait(I2C_RECOVER_BB_DELAY_US);

		/* check if SCL and SDA are both high */
		bb = sys_read8(i2c_base + XEC_I2C_BBCR_OFS);

		if ((bb & XEC_I2C_BBCR_SCL_SDA_IN_MSK) == XEC_I2C_BBCR_SCL_SDA_IN_MSK) {
			ret = 0; /* successful recovery */
			goto recov_exit;
		}
	}

recov_exit:
	/* BB mode disable reconnects SCL and SDA to I2C logic. */
	sys_write8(BIT(XEC_I2C_BBCR_CM_POS), i2c_base + XEC_I2C_BBCR_OFS);
	sys_write8(BIT(XEC_I2C_CR_PIN_POS), i2c_base + XEC_I2C_CR_OFS); /* clear PIN status */
	i2c_xec_reset_config(dev); /* reset controller */

	return ret;
}

#ifdef CONFIG_I2C_TARGET
/*
 * Restart I2C controller as target for ACK of address match.
 * Setting PIN clears all status in I2C.Status register except NBB.
 */
static void restart_target(const struct device *dev)
{
	i2c_ctl_wr(dev, (BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) |
			 BIT(XEC_I2C_CR_ENI_POS) | BIT(XEC_I2C_CR_ACK_POS)));
}

/*
 * Configure I2C controller acting as target to NACK the next received byte.
 * NOTE: Firmware must re-enable ACK generation before the start of the next
 * transaction otherwise the controller will NACK its target addresses.
 */
static void target_config_for_nack(const struct device *dev)
{
	i2c_ctl_wr(dev, (BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) |
			 BIT(XEC_I2C_CR_ENI_POS)));
}
#endif

static int wait_pin(const struct device *dev, bool pin_assert, uint32_t nwait)
{
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data = dev->data;
	mem_addr_t i2c_base = (mem_addr_t)cfg->base_addr;

	for (;;) {
		k_busy_wait(WAIT_INTERVAL);

		data->i2c_compl = sys_read32(i2c_base + XEC_I2C_COMP_OFS);
		data->i2c_status = sys_read8(i2c_base + XEC_I2C_SR_OFS);

		if (data->i2c_status & BIT(XEC_I2C_SR_BER_POS)) {
			return I2C_XEC_ERR_BUS;
		}

		if (data->i2c_status & BIT(XEC_I2C_SR_LAB_POS)) {
			return I2C_XEC_ERR_LAB;
		}

		if ((data->i2c_status & BIT(XEC_I2C_SR_PIN_POS)) == 0) {
			if (pin_assert) {
				return 0;
			}
		} else if (!pin_assert) {
			return 0;
		}

		if (nwait) {
			--nwait;
		} else {
			break;
		}
	}

	return I2C_XEC_ERR_TMOUT;
}

static int gen_start(const struct device *dev, uint8_t addr8,
		     bool is_repeated)
{
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data = dev->data;
	mem_addr_t i2c_base = (mem_addr_t)cfg->base_addr;
	uint8_t ctrl = BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_STA_POS) | BIT(XEC_I2C_CR_ACK_POS);

	data->i2c_addr = addr8;

	if (is_repeated) {
		i2c_ctl_wr(dev, ctrl);
		sys_write8(addr8, i2c_base + XEC_I2C_DATA_OFS);
	} else {
		ctrl |= BIT(XEC_I2C_CR_PIN_POS);
		sys_write8(addr8, i2c_base + XEC_I2C_DATA_OFS);
		i2c_ctl_wr(dev, ctrl);
	}

	return 0;
}

static int gen_stop(const struct device *dev)
{
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data = dev->data;
	mem_addr_t i2c_base = (mem_addr_t)cfg->base_addr;
	uint8_t ctrl = (BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) | \
			BIT(XEC_I2C_CR_STO_POS) | BIT(XEC_I2C_CR_ACK_POS));

	data->i2c_ctrl = ctrl;
	sys_write8(ctrl, i2c_base + XEC_I2C_CR_OFS);

	return 0;
}

static int do_stop(const struct device *dev, uint32_t nwait)
{
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data = dev->data;
	mem_addr_t i2c_base = (mem_addr_t)cfg->base_addr;
	uint32_t lines = 0;
	int ret = 0;

	data->state = I2C_XEC_STATE_STOPPED;
	data->read_discard = 0;

	gen_stop(dev);
	ret = wait_bus_free(dev, nwait);
	if (ret) {
		lines = get_lines(dev);
		if (lines != I2C_LINES_BOTH_HI) {
			i2c_xec_recover_bus(dev);
		} else {
			ret = i2c_xec_reset_config(dev);
		}
	}

	if (ret == 0) {
		/* stop success: prepare for next transaction */
		sys_write8(XEC_I2C_CR_CLR_PIN_ESO_ACK, i2c_base + XEC_I2C_CR_OFS);
	}

	return ret;
}

static int do_start(const struct device *dev, uint8_t addr8, bool is_repeated)
{
	struct i2c_xec_data *data = dev->data;
	int ret = 0;

	gen_start(dev, addr8, is_repeated);
	ret = wait_pin(dev, I2C_WAIT_PIN_ASSERT, WAIT_COUNT);
	if (ret) {
		i2c_xec_reset_config(dev);
		return ret;
	}

	/* PIN 1->0: check for NACK */
	if ((data->i2c_status & BIT(XEC_I2C_SR_LRB_AD0_POS)) != 0) {
		gen_stop(dev);
		ret = wait_bus_free(dev, WAIT_COUNT);
		if (ret) {
			i2c_xec_reset_config(dev);
		}
		return -EIO;
	}

	return 0;
}

static int i2c_xec_configure(const struct device *dev, uint32_t dev_config_raw)
{
	struct i2c_xec_data *data = dev->data;

	if (!(dev_config_raw & I2C_MODE_CONTROLLER)) {
		return -ENOTSUP;
	}

	if (dev_config_raw & I2C_ADDR_10_BITS) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config_raw)) {
	case I2C_SPEED_STANDARD:
		data->speed_id = SPEED_100KHZ_BUS;
		break;
	case I2C_SPEED_FAST:
		data->speed_id = SPEED_400KHZ_BUS;
		break;
	case I2C_SPEED_FAST_PLUS:
		data->speed_id = SPEED_1MHZ_BUS;
		break;
	default:
		return -EINVAL;
	}

	int ret = i2c_xec_reset_config(dev);

	return ret;
}

/* I2C Controller transmit: polling implementation */
static int ctrl_tx(const struct device *dev, struct i2c_msg *msg, uint16_t addr)
{
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data = dev->data;
	mem_addr_t i2c_base = (mem_addr_t)cfg->base_addr;
	int ret = 0;
	uint8_t mflags = msg->flags;
	uint8_t addr8 = (uint8_t)((addr & 0x7FU) << 1);

	if (data->state == I2C_XEC_STATE_STOPPED) {
		data->i2c_addr = addr8;
		/* Is bus free and controller ready? */
		ret = wait_bus_free(dev, WAIT_COUNT);
		if (ret) {
			ret = i2c_xec_recover_bus(dev);
			if (ret) {
				return ret;
			}
		}

		ret = do_start(dev, addr8, I2C_START);
		if (ret) {
			return ret;
		}

		data->state = I2C_XEC_STATE_OPEN;

	} else if (mflags & I2C_MSG_RESTART) {
		data->i2c_addr = addr8;
		ret = do_start(dev, addr8, I2C_RPT_START);
		if (ret) {
			return ret;
		}
	}

	for (size_t n = 0; n < msg->len; n++) {
		sys_write8(msg->buf[n], i2c_base + XEC_I2C_DATA_OFS);

		ret = wait_pin(dev, I2C_WAIT_PIN_ASSERT, WAIT_COUNT);
		if (ret) {
			i2c_xec_reset_config(dev);
			return ret;
		}

		if ((data->i2c_status & BIT(XEC_I2C_SR_LRB_AD0_POS)) != 0) { /* NACK? */
			do_stop(dev, STOP_WAIT_COUNT);
			return -EIO;
		}
	}

	if (mflags & I2C_MSG_STOP) {
		ret = do_stop(dev, STOP_WAIT_COUNT);
	}

	return ret;
}

/*
 * I2C Controller receive: polling implementation
 * Transmitting a target address with BIT[0] == 1 causes the controller
 * to enter controller-read mode where every read of I2CDATA generates
 * clocks for the next byte. When we generate START or Repeated-START
 * and transmit an address the address is also clocked in during
 * address transmission. The address must read and discarded.
 * Read of I2CDATA returns data currently in I2C read buffer, sets
 * I2CSTATUS.PIN = 1, and !!generates clocks for the next
 * byte!!
 * For this controller to NACK the last byte we must clear the
 * I2C CTRL register ACK bit BEFORE reading the next to last
 * byte. Before reading the last byte we configure I2C CTRL to generate a STOP
 * and then read the last byte from I2 DATA.
 * When controller is in STOP mode it will not generate clocks when I2CDATA is
 * read. UGLY HW DESIGN.
 * We will NOT attempt to follow this HW design for Controller read except
 * when all information is available: STOP message flag set AND number of
 * bytes to read including dummy is >= 2. General usage can result in the
 * controller not NACK'ing the last byte.
 */
static int ctrl_rx(const struct device *dev, struct i2c_msg *msg, uint16_t addr)
{
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data = dev->data;
	mem_addr_t i2c_base = (mem_addr_t)cfg->base_addr;
	int ret = 0;
	size_t data_len = msg->len;
	uint8_t mflags = msg->flags;
	uint8_t addr8 = (uint8_t)(((addr & 0x7FU) << 1) | BIT(0));
	uint8_t temp = 0;

	if (data->state == I2C_XEC_STATE_STOPPED) {
		data->i2c_addr = addr8;
		/* Is bus free and controller ready? */
		ret = wait_bus_free(dev, WAIT_COUNT);
		if (ret) {
			i2c_xec_reset_config(dev);
			return ret;
		}

		ret = do_start(dev, addr8, I2C_START);
		if (ret) {
			return ret;
		}

		data->state = I2C_XEC_STATE_OPEN;

		/* controller clocked address into I2CDATA */
		data->read_discard = 1U;

	} else if (mflags & I2C_MSG_RESTART) {
		data->i2c_addr = addr8;
		ret = do_start(dev, addr8, I2C_RPT_START);
		if (ret) {
			return ret;
		}

		/* controller clocked address into I2CDATA */
		data->read_discard = 1U;
	}

	if (!data_len) { /* requested message length is 0 */
		ret = 0;
		if (mflags & I2C_MSG_STOP) {
			data->state = I2C_XEC_STATE_STOPPED;
			data->read_discard = 0;
			ret = do_stop(dev, STOP_WAIT_COUNT);
		}
		return ret;
	}

	if (data->read_discard) {
		data_len++;
	}

	uint8_t *p8 = &msg->buf[0];

	while (data_len) {
		if (mflags & I2C_MSG_STOP) {
			if (data_len == 2) {
				i2c_ctl_wr(dev, BIT(XEC_I2C_CR_ESO_POS));
			} else if (data_len == 1) {
				break;
			}
		}

		temp = sys_read8(i2c_base + XEC_I2C_DATA_OFS);

		if (data->read_discard) {
			data->read_discard = 0;
		} else {
			*p8++ = temp;
		}
		ret = wait_pin(dev, I2C_WAIT_PIN_ASSERT, WAIT_COUNT);
		if (ret) {
			i2c_xec_reset_config(dev);
			return ret;
		}
		data_len--;
	}

	if (mflags & I2C_MSG_STOP) {
		data->state = I2C_XEC_STATE_STOPPED;
		data->read_discard = 0;
		ret = do_stop(dev, STOP_WAIT_COUNT);
		if (ret == 0) {
			*p8 = sys_read8(i2c_base + XEC_I2C_DATA_OFS);
		}
	}

	return ret;
}

static int i2c_xec_transfer(const struct device *dev, struct i2c_msg *msgs,
			    uint8_t num_msgs, uint16_t addr)
{
	struct i2c_xec_data *data = dev->data;
	int ret = 0;

#ifdef CONFIG_I2C_TARGET
	if (data->target_attached) {
		LOG_ERR("Device is registered as target");
		return -EBUSY;
	}
#endif

	for (uint8_t i = 0; i < num_msgs; i++) {
		struct i2c_msg *m = &msgs[i];

		if ((m->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			ret = ctrl_tx(dev, m, addr);
		} else {
			ret = ctrl_rx(dev, m, addr);
		}
		if (ret) {
			data->state = I2C_XEC_STATE_STOPPED;
			data->read_discard = 0;
			LOG_ERR("i2x_xfr: flags: %x error: %d", m->flags, ret);
			break;
		}
	}

	return ret;
}

static void i2c_xec_bus_isr(const struct device *dev)
{
#ifdef CONFIG_I2C_TARGET
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data = dev->data;
	const struct i2c_target_callbacks *target_cb = data->target_cfg->callbacks;
	mem_addr_t i2c_base = (mem_addr_t)cfg->base_addr;
	int ret = 0;
	uint32_t status = 0, compl_status = 0, i2c_cfg = 0;
	uint8_t val = 0, dummy = 0, rx_data = 0;

	/* Get current status */
	status = sys_read8(i2c_base + XEC_I2C_SR_OFS);
	compl_status = sys_read32(i2c_base + XEC_I2C_COMP_OFS);
	i2c_cfg = sys_read32(i2c_base + XEC_I2C_CFG_OFS);

	/* Idle interrupt enabled and active? */
	if (((i2c_cfg & BIT(XEC_I2C_CFG_IDLE_IEN_POS)) != 0) &&
	    ((compl_status & BIT(XEC_I2C_COMP_IDLE_POS)) != 0)) {
		sys_clear_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
		if ((status & BIT(XEC_I2C_SR_NBB_POS)) != 0) {
			restart_target(dev);
			goto clear_iag;
		}
	}

	if (!data->target_attached) {
		goto clear_iag;
	}

	/* Bus Error */
	if ((status & BIT(XEC_I2C_SR_BER_POS)) != 0) {
		if (target_cb->stop) {
			target_cb->stop(data->target_cfg);
		}
		restart_target(dev);
		goto clear_iag;
	}

	/* External stop */
	if ((status & BIT(XEC_I2C_SR_STO_POS)) != 0) {
		if (target_cb->stop) {
			target_cb->stop(data->target_cfg);
		}
		restart_target(dev);
		goto clear_iag;
	}

	/* Address byte handling */
	if ((status & BIT(XEC_I2C_SR_AAT_POS)) != 0) {
		if ((status & BIT(XEC_I2C_SR_PIN_POS)) != 0) {
			goto clear_iag;
		}

		rx_data = sys_read8(i2c_base + XEC_I2C_DATA_OFS);

		if (rx_data & BIT(I2C_READ_WRITE_POS)) {
			/* target transmitter mode */
			data->target_read = true;
			val = dummy;
			if (target_cb->read_requested) {
				target_cb->read_requested(
					data->target_cfg, &val);

				/* Application target transmit handler
				 * does not have data to send. In
				 * target transmit mode the external
				 * Controller is ACK's data we send.
				 * All we can do is keep sending dummy
				 * data. We assume read_requested does
				 * not modify the value pointed to by val
				 * if it has not data(returns error).
				 */
			}
			/*
			 * Writing I2CData causes this HW to release SCL
			 * ending clock stretching. The external Controller
			 * senses SCL released and begins generating clocks
			 * and capturing data driven by this controller
			 * on SDA. External Controller ACK's data until it
			 * wants no more then it will NACK.
			 */
			sys_write8(val, i2c_base + XEC_I2C_DATA_OFS);
			goto clear_iag; /* Exit ISR */
		} else {
			/* target receiver mode */
			data->target_read = false;
			if (target_cb->write_requested) {
				ret = target_cb->write_requested(data->target_cfg);
				if (ret) {
					/*
					 * Application handler can't accept
					 * data. Configure HW to NACK next
					 * data transmitted by external
					 * Controller.
					 * !!! TODO We must re-program our HW
					 * for address ACK before next
					 * transaction is begun !!!
					 */
					target_config_for_nack(dev);
				}
			}
			goto clear_iag; /* Exit ISR */
		}
	}

	if (data->target_read) { /* Target transmitter mode */
		/* Master has Nacked, then just write a dummy byte */
		status = sys_read8(i2c_base + XEC_I2C_SR_OFS);

		if ((status & BIT(XEC_I2C_SR_LRB_AD0_POS)) != 0) {
			/*
			 * ISSUE: HW will not detect external STOP in
			 * target transmit mode. Enable IDLE interrupt
			 * to catch PIN 0 -> 1 and NBB 0 -> 1.
			 */
			sys_set_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);
			/*
			 * dummy write causes this controller's PIN status
			 * to de-assert 0 -> 1. Data is not transmitted.
			 * SCL is not driven low by this controller.
			 */
			sys_write8(dummy, i2c_base + XEC_I2C_DATA_OFS);
			status = sys_read8(i2c_base + XEC_I2C_SR_OFS);

		} else {
			val = dummy;
			if (target_cb->read_processed) {
				target_cb->read_processed(
					data->target_cfg, &val);
			}

			sys_write8(val, i2c_base + XEC_I2C_DATA_OFS);
		}
	} else { /* target receiver mode */
		/*
		 * Reading the I2CData register causes this target to release
		 * SCL. The external Controller senses SCL released generates
		 * clocks for transmitting the next data byte.
		 * Reading I2C Data register causes PIN status 0 -> 1.
		 */
		val = sys_read8(i2c_base + XEC_I2C_DATA_OFS);

		if (target_cb->write_received) {
			/*
			 * Call back returns error if we should NACK
			 * next byte.
			 */
			ret = target_cb->write_received(data->target_cfg, val);
			if (ret) {
				/*
				 * Configure HW to NACK next byte. It will not
				 * generate clocks for another byte of data
				 */
				target_config_for_nack(dev);
			}
		}
	}

clear_iag:
	sys_write32(compl_status, i2c_base + XEC_I2C_COMP_OFS);
	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);
#endif /* CONFIG_I2C_TARGET */
}

#ifdef CONFIG_I2C_TARGET
static int i2c_xec_target_register(const struct device *dev, struct i2c_target_config *config)
{
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data = dev->data;
	int ret;

	if (!config) {
		return -EINVAL;
	}

	if (data->target_attached) {
		return -EBUSY;
	}

	/* Wait for any outstanding transactions to complete so that
	 * the bus is free
	 */
	ret = wait_bus_free(dev, WAIT_COUNT);
	if (ret) {
		return ret;
	}

	data->target_cfg = config;

	ret = i2c_xec_reset_config(dev);
	if (ret) {
		return ret;
	}

	restart_target(dev);

	data->target_attached = true;

	/* Clear before enabling girq bit */
	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);
	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, 1u);

	return 0;
}

static int i2c_xec_target_unregister(const struct device *dev, struct i2c_target_config *config)
{
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data = dev->data;

	if (!data->target_attached) {
		return -EINVAL;
	}

	data->target_cfg = NULL;
	data->target_attached = false;

	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, 0);

	return 0;
}
#endif

static DEVICE_API(i2c, i2c_xec_driver_api) = {
	.configure = i2c_xec_configure,
	.transfer = i2c_xec_transfer,
#ifdef CONFIG_I2C_TARGET
	.target_register = i2c_xec_target_register,
	.target_unregister = i2c_xec_target_unregister,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

static int i2c_xec_init(const struct device *dev)
{
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data =
		(struct i2c_xec_data *const) (dev->data);
	int ret;
	uint32_t bitrate_cfg;

	data->state = I2C_XEC_STATE_STOPPED;
	data->target_cfg = NULL;
	data->target_attached = false;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("XEC I2C pinctrl setup failed (%d)", ret);
		return ret;
	}

	bitrate_cfg = i2c_map_dt_bitrate(cfg->clock_freq);
	if (!bitrate_cfg) {
		return -EINVAL;
	}

	/* Default configuration */
	ret = i2c_xec_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	if (ret) {
		return ret;
	}

#ifdef CONFIG_I2C_TARGET
	const struct i2c_xec_config *config =
	(const struct i2c_xec_config *const) (dev->config);

	config->irq_config_func();
#endif
	return 0;
}

#define XEC_I2C_GIRQ_DT(inst) MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, 0))
#define XEC_I2C_GIRQ_POS_DT(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, 0))

#define I2C_XEC_DEVICE(n)						\
									\
	PINCTRL_DT_INST_DEFINE(n);					\
									\
	static void i2c_xec_irq_config_func_##n(void);			\
									\
	static struct i2c_xec_data i2c_xec_data_##n;			\
	static const struct i2c_xec_config i2c_xec_config_##n = {	\
		.base_addr = DT_INST_REG_ADDR(n),			\
		.port_sel = DT_INST_PROP(n, port_sel),			\
		.clock_freq = DT_INST_PROP(n, clock_frequency),		\
		.girq = XEC_I2C_GIRQ_DT(n),				\
		.girq_pos = XEC_I2C_GIRQ_POS_DT(n),			\
		.pcr = DT_INST_PROP(n, pcr),				\
		.irq_config_func = i2c_xec_irq_config_func_##n,		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
	};								\
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_xec_init, NULL,		\
		&i2c_xec_data_##n, &i2c_xec_config_##n,			\
		POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,			\
		&i2c_xec_driver_api);					\
									\
	static void i2c_xec_irq_config_func_##n(void)			\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    i2c_xec_bus_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQN(n));				\
	}

DT_INST_FOREACH_STATUS_OKAY(I2C_XEC_DEVICE)
