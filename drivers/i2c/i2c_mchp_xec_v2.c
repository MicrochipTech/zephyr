/*
 * Copyright (c) 2019 Intel Corporation
 * Copyright (c) 2021 Microchip Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_i2c_v2

#include <soc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_xec_v2, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"
#include "i2c_mchp_xec_regs.h"
#include "i2c_mchp_xec_common.h"

#define SPEED_100KHZ_BUS 0
#define SPEED_400KHZ_BUS 1
#define SPEED_1MHZ_BUS   2

#define EC_OWN_I2C_ADDR 0x7F
#define RESET_WAIT_US   20

/* I2C timeout is  10 ms (WAIT_INTERVAL * WAIT_COUNT) */
#define WAIT_INTERVAL   50
#define WAIT_COUNT      200
#define STOP_WAIT_COUNT 500
#define PIN_CFG_WAIT    50

/* I2C Read/Write bit pos */
#define I2C_READ_WRITE_POS 0

/* I2C SCL and SDA lines(signals) */
#define I2C_LINES_SCL_HI  BIT(SOC_I2C_SCL_POS)
#define I2C_LINES_SDA_HI  BIT(SOC_I2C_SDA_POS)
#define I2C_LINES_BOTH_HI (I2C_LINES_SCL_HI | I2C_LINES_SDA_HI)

#define I2C_START     0U
#define I2C_RPT_START 1U

#define I2C_ENI_DIS 0U
#define I2C_ENI_EN  1U

#define I2C_WAIT_PIN_DEASSERT 0U
#define I2C_WAIT_PIN_ASSERT   1U

#define I2C_XEC_CTRL_WR_DLY 8

#define I2C_XEC_STATE_STOPPED 1U
#define I2C_XEC_STATE_OPEN    2U

#define I2C_XEC_OK        0
#define I2C_XEC_ERR_LAB   1
#define I2C_XEC_ERR_BUS   2
#define I2C_XEC_ERR_TMOUT 3

struct i2c_xec_config {
	uint32_t base_addr;
	uint32_t clock_freq;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t enc_pcr;
	uint8_t port_sel;
	struct gpio_dt_spec sda_gpio;
	struct gpio_dt_spec scl_gpio;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
};

struct i2c_xec_data {
	uint32_t dev_config;
	uint32_t i2c_compl;
	uint8_t i2c_ctrl;
	uint8_t i2c_addr;
	uint8_t i2c_status;
	uint8_t state;
	uint8_t read_discard;
	uint8_t speed_id;
	struct k_mutex mux;
	struct i2c_target_config *target_cfg;
	bool target_attached;
	bool target_read;
};

static void i2c_ctl_wr(const struct device *dev, uint8_t ctrl)
{
	struct i2c_xec_data *const data = dev->data;
	const struct i2c_xec_config *cfg = dev->config;
	mm_reg_t rb = cfg->base_addr;

	data->i2c_ctrl = ctrl;
	sys_write8(ctrl, rb + XEC_I2C_CR_OFS);

	for (int i = 0; i < I2C_XEC_CTRL_WR_DLY; i++) {
		sys_write8(ctrl, rb + XEC_I2C_BLKID_OFS);
	}
}

static int i2c_xec_reset_config(const struct device *dev);

static int wait_bus_free(const struct device *dev, uint32_t nwait)
{
	struct i2c_xec_data *const data = dev->data;
	const struct i2c_xec_config *cfg = dev->config;
	mm_reg_t rb = cfg->base_addr;
	uint32_t count = nwait;
	uint8_t sts = 0;

	while (count--) {
		sts = sys_read8(rb + XEC_I2C_SR_OFS);
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
		return -EIO;
	}

	if ((sts & BIT(XEC_I2C_SR_LAB_POS)) != 0) {
		return -ECONNABORTED;
	}

	return -ETIMEDOUT;
}

static void i2c_xec_initial_cfg(const struct device *dev)
{
	const struct i2c_xec_config *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->base_addr;
	uint32_t val = (BIT(XEC_I2C_CFG_GC_DIS_POS) | BIT(XEC_I2C_CFG_FEN_POS) |
			XEC_I2C_CFG_PORT_SET((uint32_t)drvcfg->port_sel));

	sys_write32(val, rb + XEC_I2C_CFG_OFS);
}

static int i2c_xec_reset_config(const struct device *dev)
{
	struct i2c_xec_data *const data = dev->data;
	const struct i2c_xec_config *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->base_addr;
	uint32_t freq_hz = 0;
	int rc = 0;

	data->state = I2C_XEC_STATE_STOPPED;
	data->read_discard = 0;

	soc_xec_pcr_sleep_en_clear(drvcfg->enc_pcr);
	soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 0);
	i2c_xec_ctrl_reset(rb, drvcfg->port_sel, drvcfg->enc_pcr);
	soc_ecia_girq_status_clear(drvcfg->girq, drvcfg->girq_pos);

	/* PIN=1 to clear all status except NBB and synchronize */
	i2c_ctl_wr(dev, BIT(XEC_I2C_CR_PIN_POS));

#ifdef CONFIG_I2C_TARGET
	if (data->target_cfg != NULL) {
		sys_write32(XEC_I2C_OA_1_SET(data->target_cfg->address), rb + XEC_I2C_OA_OFS);
	}
#endif
	/* Port number and filter enable MUST be written before enabling controller */
	i2c_xec_initial_cfg(dev);

	/*
	 * Before enabling the controller program the desired bus clock,
	 * repeated start hold time, data timing, and timeout scaling
	 * registers.
	 */
	freq_hz = i2c_xec_freq_from_dev_config(data->dev_config);

	rc = i2c_xec_prog_timing(rb, freq_hz);

	/*
	 * PIN=1 clears all status except NBB
	 * ESO=1 enables output drivers
	 * ACK=1 enable ACK generation when data/address is clocked in.
	 */
	i2c_ctl_wr(dev,
		   (BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS)));

	/* Enable controller */
	sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);
	k_busy_wait(RESET_WAIT_US);

	/* wait for NBB=1, BER, LAB, or timeout */
	return wait_bus_free(dev, WAIT_COUNT);
}

static uint32_t get_lines(const struct device *dev)
{
	const struct i2c_xec_config *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->base_addr;

	return (uint32_t)i2c_xec_get_lines(rb, &drvcfg->scl_gpio, &drvcfg->sda_gpio);
}

#ifdef CONFIG_I2C_TARGET
/*
 * Restart I2C controller as target for ACK of address match.
 * Setting PIN clears all status in I2C.Status register except NBB.
 */
static void restart_target(const struct device *dev)
{
	i2c_ctl_wr(dev, BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) |
				BIT(XEC_I2C_CR_ACK_POS) | BIT(XEC_I2C_CR_ENI_POS));
}

/*
 * Configure I2C controller acting as target to NACK the next received byte.
 * NOTE: Firmware must re-enable ACK generation before the start of the next
 * transaction otherwise the controller will NACK its target addresses.
 */
static void target_config_for_nack(const struct device *dev)
{
	i2c_ctl_wr(dev,
		   BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ENI_POS));
}
#endif

static int wait_pin(const struct device *dev, bool pin_assert, uint32_t nwait)
{
	struct i2c_xec_data *const data = dev->data;
	const struct i2c_xec_config *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->base_addr;

	for (;;) {
		k_busy_wait(WAIT_INTERVAL);

		data->i2c_compl = sys_read32(rb + XEC_I2C_CMPL_OFS);
		data->i2c_status = sys_read8(rb + XEC_I2C_SR_OFS);

		if ((data->i2c_status & BIT(XEC_I2C_SR_BER_POS)) != 0) {
			return -EIO;
		}

		if ((data->i2c_status & BIT(XEC_I2C_SR_LAB_POS)) != 0) {
			return -ECONNABORTED;
		}

		if ((data->i2c_status & BIT(XEC_I2C_SR_PIN_POS)) == 0) {
			if (pin_assert) {
				return 0;
			}
		} else if (pin_assert == false) {
			return 0;
		}

		if (nwait != 0) {
			--nwait;
		} else {
			break;
		}
	}

	return -ETIMEDOUT;
}

static int gen_start(const struct device *dev, uint8_t addr8, bool is_repeated)
{
	struct i2c_xec_data *const data = dev->data;
	const struct i2c_xec_config *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->base_addr;
	uint8_t ctrl = BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_STA_POS) | BIT(XEC_I2C_CR_ACK_POS);

	data->i2c_addr = addr8;

	if (is_repeated == true) {
		i2c_ctl_wr(dev, ctrl);
		sys_write8(addr8, rb + XEC_I2C_DATA_OFS);
	} else {
		ctrl |= BIT(XEC_I2C_CR_PIN_POS);
		sys_write8(addr8, rb + XEC_I2C_DATA_OFS);
		i2c_ctl_wr(dev, ctrl);
	}

	return 0;
}

static int gen_stop(const struct device *dev)
{
	uint8_t ctrl = BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_STO_POS) |
		       BIT(XEC_I2C_CR_ACK_POS);

	i2c_ctl_wr(dev, ctrl);

	return 0;
}

static int do_start(const struct device *dev, uint8_t addr8, bool is_repeated)
{
	struct i2c_xec_data *const data = dev->data;
	int ret = 0;

	gen_start(dev, addr8, is_repeated);

	ret = wait_pin(dev, I2C_WAIT_PIN_ASSERT, WAIT_COUNT);
	if (ret != 0) {
		i2c_xec_reset_config(dev);
		return ret;
	}

	/* PIN 1->0: check for NACK */
	if ((data->i2c_status & BIT(XEC_I2C_SR_LRB_AD0_POS)) != 0) {
		gen_stop(dev);

		ret = wait_bus_free(dev, WAIT_COUNT);
		if (ret != 0) {
			i2c_xec_reset_config(dev);
		}

		return -EIO;
	}

	return 0;
}

static int i2c_xec_v2_configure(const struct device *dev, uint32_t dev_config_raw)
{
	struct i2c_xec_data *const data = dev->data;

	if ((dev_config_raw & I2C_MODE_CONTROLLER) == 0) {
		return -ENOTSUP;
	}

	if ((dev_config_raw & I2C_ADDR_10_BITS) != 0) {
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

	data->dev_config = dev_config_raw;

	return i2c_xec_reset_config(dev);
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
static int i2c_xec_v2_recover_bus(const struct device *dev)
{
	const struct i2c_xec_config *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->base_addr;
	uint32_t dev_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
	int rc = 0;

	/* reset controller and configure for 100 KHz before recovery attempt */
	rc = i2c_xec_v2_configure(dev, dev_config);
	if (rc != 0) {
		rc = i2c_xec_bus_recovery(rb);
		if (rc != 0) {
			return rc;
		}
	}

	/* reconfigure controller at previous speed setting */
	rc = i2c_xec_reset_config(dev);

	return rc;
}

static int do_stop(const struct device *dev, uint32_t nwait)
{
	struct i2c_xec_data *const data = dev->data;
	uint32_t lines = 0;
	int ret = 0;

	data->state = I2C_XEC_STATE_STOPPED;
	data->read_discard = 0;

	gen_stop(dev);

	ret = wait_bus_free(dev, nwait);
	if (ret != 0) {
		lines = get_lines(dev);

		if (lines != I2C_LINES_BOTH_HI) {
			i2c_xec_v2_recover_bus(dev);
		} else {
			ret = i2c_xec_reset_config(dev);
		}
	}

	if (ret == 0) {
		/* stop success: prepare for next transaction */
		i2c_ctl_wr(dev, (BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) |
				 BIT(XEC_I2C_CR_ACK_POS)));
	}

	return ret;
}

/* I2C Controller transmit: polling implementation */
static int ctrl_tx(const struct device *dev, struct i2c_msg *msg, uint16_t addr)
{
	struct i2c_xec_data *const data = dev->data;
	const struct i2c_xec_config *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->base_addr;
	int ret = 0;
	uint8_t mflags = msg->flags;
	uint8_t addr8 = (uint8_t)((addr & 0x7FU) << 1);

	if (data->state == I2C_XEC_STATE_STOPPED) {
		data->i2c_addr = addr8;

		/* Is bus free and controller ready? */
		ret = wait_bus_free(dev, WAIT_COUNT);
		if (ret != 0) {
			ret = i2c_xec_v2_recover_bus(dev);
			if (ret != 0) {
				return ret;
			}
		}

		ret = do_start(dev, addr8, I2C_START);
		if (ret != 0) {
			return ret;
		}

		data->state = I2C_XEC_STATE_OPEN;

	} else if ((mflags & I2C_MSG_RESTART) != 0) {
		data->i2c_addr = addr8;

		ret = do_start(dev, addr8, I2C_RPT_START);
		if (ret != 0) {
			return ret;
		}
	}

	for (size_t n = 0; n < msg->len; n++) {
		/* writing I2C.DATA register causes PIN status to de-assert */
		sys_write8(msg->buf[n], rb + XEC_I2C_DATA_OFS);

		ret = wait_pin(dev, I2C_WAIT_PIN_ASSERT, WAIT_COUNT);
		if (ret != 0) {
			i2c_xec_reset_config(dev);
			return ret;
		}

		if ((data->i2c_status & BIT(XEC_I2C_SR_LRB_AD0_POS)) != 0) { /* NACK? */
			do_stop(dev, STOP_WAIT_COUNT);
			return -EIO;
		}
	}

	if ((mflags & I2C_MSG_STOP) != 0) {
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
	struct i2c_xec_data *const data = dev->data;
	const struct i2c_xec_config *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->base_addr;
	uint8_t *p8 = NULL;
	size_t data_len = msg->len;
	uint8_t mflags = msg->flags;
	uint8_t addr8 = (uint8_t)(((addr & 0x7FU) << 1) | BIT(0));
	uint8_t temp = 0;
	int ret = 0;

	if (data->state == I2C_XEC_STATE_STOPPED) {
		data->i2c_addr = addr8;

		/* Is bus free and controller ready? */
		ret = wait_bus_free(dev, WAIT_COUNT);
		if (ret != 0) {
			i2c_xec_reset_config(dev);
			return ret;
		}

		ret = do_start(dev, addr8, I2C_START);
		if (ret != 0) {
			return ret;
		}

		data->state = I2C_XEC_STATE_OPEN;

		/* controller clocked address into I2CDATA */
		data->read_discard = 1U;

	} else if ((mflags & I2C_MSG_RESTART) != 0) {
		data->i2c_addr = addr8;

		ret = do_start(dev, addr8, I2C_RPT_START);
		if (ret != 0) {
			return ret;
		}

		/* controller clocked address into I2CDATA */
		data->read_discard = 1U;
	}

	if (data_len != 0) { /* requested message length is 0 */
		ret = 0;

		if ((mflags & I2C_MSG_STOP) != 0) {
			data->state = I2C_XEC_STATE_STOPPED;
			data->read_discard = 0;
			ret = do_stop(dev, STOP_WAIT_COUNT);
		}

		return ret;
	}

	if (data->read_discard != 0) {
		data_len++;
	}

	p8 = &msg->buf[0];

	while (data_len) {
		if ((mflags & I2C_MSG_STOP) != 0) {
			if (data_len == 2U) {
				i2c_ctl_wr(dev, BIT(XEC_I2C_CR_ESO_POS));
			} else if (data_len == 1U) {
				break;
			}
		}

		/* read I2C.DATA register returns current captured data and generates clocks
		 * for the next data byte (read-ahead)
		 */
		temp = sys_read8(rb + XEC_I2C_DATA_OFS);

		if (data->read_discard != 0) {
			data->read_discard = 0;
		} else {
			*p8++ = temp;
		}

		ret = wait_pin(dev, I2C_WAIT_PIN_ASSERT, WAIT_COUNT);
		if (ret != 0) {
			i2c_xec_reset_config(dev);
			return ret;
		}

		data_len--;
	}

	if ((mflags & I2C_MSG_STOP) != 0) {
		data->state = I2C_XEC_STATE_STOPPED;
		data->read_discard = 0;

		ret = do_stop(dev, STOP_WAIT_COUNT);
		if (ret == 0) {
			/* After instructing I2C controller to genrerate a STOP we must read
			 * the last byte from the target from I2C.DATA. HW does not generate
			 * clocks since we generated a STOP.
			 */
			*p8 = sys_read8(rb + XEC_I2C_DATA_OFS);
		}
	}

	return ret;
}

static int i2c_xec_v2_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			       uint16_t addr)
{
	struct i2c_xec_data *data = dev->data;
	int ret = 0;

#ifdef CONFIG_I2C_TARGET
	if (data->target_attached == true) {
		LOG_ERR("Device is registered as target");
		return -EBUSY;
	}
#endif

	k_mutex_lock(&data->mux, K_FOREVER);

	for (uint8_t i = 0; i < num_msgs; i++) {
		struct i2c_msg *m = &msgs[i];

		if ((m->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			ret = ctrl_tx(dev, m, addr);
		} else {
			ret = ctrl_rx(dev, m, addr);
		}

		if (ret != 0) {
			data->state = I2C_XEC_STATE_STOPPED;
			data->read_discard = 0;
			LOG_ERR("i2x_xfr: flags: %x error: %d", m->flags, ret);
			break;
		}
	}

	k_mutex_unlock(&data->mux);

	return ret;
}

static void i2c_xec_v2_isr(const struct device *dev)
{
#ifdef CONFIG_I2C_TARGET
	struct i2c_xec_data *const data = dev->data;
	const struct i2c_xec_config *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->base_addr;
	struct i2c_target_config *tcfg = data->target_cfg;
	const struct i2c_target_callbacks *tcbs = NULL;
	int ret = 0;
	uint32_t status = 0;
	uint32_t compl_status = 0, config = 0;
	uint8_t val = 0;
	uint8_t dummy = 0U;
	uint8_t rx_data = 0U;

	/* Get current status */
	status = sys_read8(rb + XEC_I2C_SR_OFS);
	compl_status = sys_read32(rb + XEC_I2C_CMPL_OFS) & XEC_I2C_CMPL_RW1C_MSK;
	config = sys_read32(rb + XEC_I2C_CFG_OFS);

	/* Idle interrupt enabled and active? */
	if (((config & BIT(XEC_I2C_CFG_IDLE_IEN_POS)) != 0) &&
	    ((compl_status & BIT(XEC_I2C_CMPL_IDLE_POS)) != 0)) {
		sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);

		if ((status & BIT(XEC_I2C_SR_NBB_POS)) != 0) {
			restart_target(dev);
			goto clear_iag;
		}
	}

	if (data->target_attached == false) {
		goto clear_iag;
	}

	if (tcfg != NULL) {
		tcbs = tcfg->callbacks;
	}

	/* External STOP or Bus Error: restart target handling */
	if ((status & (BIT(XEC_I2C_SR_BER_POS) | BIT(XEC_I2C_SR_STO_POS))) != 0) {
		if ((tcbs != NULL) && (tcbs->stop != NULL)) {
			tcbs->stop(data->target_cfg);
		}

		restart_target(dev);
		goto clear_iag;
	}

	/* Address byte handling */
	if ((status & BIT(XEC_I2C_SR_AAT_POS)) != 0) {
		if ((status & BIT(XEC_I2C_SR_PIN_POS)) != 0) {
			goto clear_iag;
		}

		rx_data = sys_read8(rb + XEC_I2C_DATA_OFS);

		if ((rx_data & BIT(I2C_READ_WRITE_POS)) != 0) {
			/* target transmitter mode */
			data->target_read = true;
			val = dummy;

			if ((tcbs != NULL) && (tcbs->read_requested != NULL)) {
				tcbs->read_requested(data->target_cfg, &val);

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
			sys_write8(val, rb + XEC_I2C_DATA_OFS);
			goto clear_iag; /* Exit ISR */
		} else {
			/* target receiver mode */
			data->target_read = false;

			if ((tcbs != NULL) && (tcbs->write_requested != NULL)) {
				ret = tcbs->write_requested(data->target_cfg);
				if (ret != 0) {
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

	if (data->target_read == true) { /* Target transmitter mode */

		/* Master has Nacked, then just write a dummy byte */
		status = sys_read8(rb + XEC_I2C_SR_OFS);

		if ((status & BIT(XEC_I2C_SR_LRB_AD0_POS)) != 0) {
			/*
			 * ISSUE: HW will not detect external STOP in
			 * target transmit mode. Enable IDLE interrupt
			 * to catch PIN 0 -> 1 and NBB 0 -> 1.
			 */
			sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_IDLE_IEN_POS);

			/*
			 * dummy write causes this controller's PIN status
			 * to de-assert 0 -> 1. Data is not transmitted.
			 * SCL is not driven low by this controller.
			 */
			sys_write8(dummy, rb + XEC_I2C_DATA_OFS);

			status = sys_read8(rb + XEC_I2C_SR_OFS);

		} else {
			val = dummy;
			if ((tcbs != NULL) && (tcbs->read_processed != NULL)) {
				tcbs->read_processed(data->target_cfg, &val);
			}

			sys_write8(val, rb + XEC_I2C_DATA_OFS);
		}
	} else { /* target receiver mode */
		/*
		 * Reading the I2CData register causes this target to release
		 * SCL. The external Controller senses SCL released generates
		 * clocks for transmitting the next data byte.
		 * Reading I2C Data register causes PIN status 0 -> 1.
		 */
		val = sys_read8(rb + XEC_I2C_DATA_OFS);

		if ((tcbs != NULL) && (tcbs->write_received != NULL)) {
			/*
			 * Call back returns error if we should NACK
			 * next byte.
			 */
			ret = tcbs->write_received(data->target_cfg, val);
			if (ret != 0) {
				/*
				 * Configure HW to NACK next byte. It will not
				 * generate clocks for another byte of data
				 */
				target_config_for_nack(dev);
			}
		}
	}

clear_iag:
	sys_write32(compl_status, rb + XEC_I2C_CMPL_OFS);
	soc_ecia_girq_status_clear(drvcfg->girq, drvcfg->girq_pos);
#endif
}

#ifdef CONFIG_I2C_TARGET
static int i2c_xec_v2_target_register(const struct device *dev, struct i2c_target_config *config)
{
	const struct i2c_xec_config *drvcfg = dev->config;
	struct i2c_xec_data *const data = dev->data;
	int ret = 0;

	if (config == NULL) {
		return -EINVAL;
	}

	if (data->target_attached == true) {
		return -EBUSY;
	}

	/* Wait for any outstanding transactions to complete so that
	 * the bus is free
	 */
	ret = wait_bus_free(dev, WAIT_COUNT);
	if (ret != 0) {
		return ret;
	}

	data->target_cfg = config;

	ret = i2c_xec_reset_config(dev);
	if (ret != 0) {
		return ret;
	}

	restart_target(dev);

	data->target_attached = true;

	/* Clear before enabling girq bit */
	soc_ecia_girq_status_clear(drvcfg->girq, drvcfg->girq_pos);
	soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 1U);

	return 0;
}

static int i2c_xec_v2_target_unregister(const struct device *dev, struct i2c_target_config *config)
{
	const struct i2c_xec_config *drvcfg = dev->config;
	struct i2c_xec_data *const data = dev->data;

	if (data->target_attached == false) {
		return -EINVAL;
	}

	data->target_cfg = NULL;
	data->target_attached = false;

	soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 0);
	soc_ecia_girq_status_clear(drvcfg->girq, drvcfg->girq_pos);

	return 0;
}
#endif

static DEVICE_API(i2c, i2c_xec_v2_driver_api) = {
	.configure = i2c_xec_v2_configure,
	.transfer = i2c_xec_v2_transfer,
#ifdef CONFIG_I2C_TARGET
	.target_register = i2c_xec_v2_target_register,
	.target_unregister = i2c_xec_v2_target_unregister,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

static int i2c_xec_v2_init(const struct device *dev)
{
	const struct i2c_xec_config *drvcfg = dev->config;
	struct i2c_xec_data *const data = dev->data;
	int ret = 0;
	uint32_t bitrate_cfg = 0;

	data->state = I2C_XEC_STATE_STOPPED;
	data->target_cfg = NULL;
	data->target_attached = false;

	ret = pinctrl_apply_state(drvcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("XEC I2C pinctrl setup failed (%d)", ret);
		return ret;
	}

	bitrate_cfg = i2c_map_dt_bitrate(drvcfg->clock_freq);
	if (bitrate_cfg == 0) {
		return -EINVAL;
	}

	soc_xec_pcr_sleep_en_clear(drvcfg->enc_pcr);

	/* Default configuration */
	ret = i2c_xec_v2_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	if (ret != 0) {
		return ret;
	}

	k_mutex_init(&data->mux);

#ifdef CONFIG_I2C_TARGET
	if (drvcfg->irq_config_func != NULL) {
		drvcfg->irq_config_func();
	}
#endif
	return 0;
}

#define XEC_I2C_GIRQ_DT(inst, idx)                                                                 \
	(uint8_t)MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, idx))
#define XEC_I2C_GIRQ_POS_DT(inst, idx)                                                             \
	(uint8_t)MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, idx))

#define I2C_XEC_DEVICE(n)                                                                          \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void i2c_xec_irq_config_func_##n(void);                                             \
	static struct i2c_xec_data i2c_xec_data_##n;                                               \
	static const struct i2c_xec_config i2c_xec_config_##n = {                                  \
		.base_addr = (mm_reg_t)DT_INST_REG_ADDR(n),                                        \
		.port_sel = DT_INST_PROP(n, port_sel),                                             \
		.clock_freq = DT_INST_PROP(n, clock_frequency),                                    \
		.girq = XEC_I2C_GIRQ_DT(n, 0),                                                     \
		.girq_pos = XEC_I2C_GIRQ_POS_DT(n, 0),                                             \
		.enc_pcr = DT_INST_PROP(n, pcr_scr),                                               \
		.irq_config_func = i2c_xec_irq_config_func_##n,                                    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.sda_gpio = GPIO_DT_SPEC_INST_GET(n, sda_gpios),                                   \
		.scl_gpio = GPIO_DT_SPEC_INST_GET(n, scl_gpios),                                   \
	};                                                                                         \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_xec_v2_init, NULL, &i2c_xec_data_##n,                     \
				  &i2c_xec_config_##n, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,      \
				  &i2c_xec_v2_driver_api);                                         \
                                                                                                   \
	static void i2c_xec_irq_config_func_##n(void)                                              \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i2c_xec_v2_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(I2C_XEC_DEVICE)
