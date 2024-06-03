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

/* #define I2C_NL_DEBUG_USE_SPIN_LOOP */
/* #define I2C_NL_DEBUG_ISR */
/* #define I2C_NL_DEBUG_STATE */
#define I2C_NL_DEBUG_STATE_ENTRIES	64

#define I2C_NL_RESET_WAIT_US		20

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
	I2C_NL_KEV_TM_DONE_POS,
	I2C_NL_KEV_DMA_A_ERR_POS,
	I2C_NL_KEV_DMA_B_ERR_POS,
};

#define I2C_MEC5_NL_DMA_BLK_CFGS	4

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
	const struct device *dma_dev_a;
	const struct device *dma_dev_b;
	struct dma_regs *dma_a_regs;
	struct dma_regs *dma_b_regs;
	uint8_t dma_a_chan;
	uint8_t dma_b_chan;
	uint8_t cm_dma_trigsrc;
	uint8_t tm_dma_trigsrc;
	uint8_t target_addr[2];
};

#define I2C_NL_XFR_FLAG_START_REQ	0x01
#define I2C_NL_XFR_FLAG_STOP_REQ	0x02
#define I2C_NL_XFR_FLAG_RPT_START_REQ	0x04

#define I2C_NL_XFR_STS_NACK		0x01
#define I2C_NL_XFR_STS_BER		0x02
#define I2C_NL_XFR_STS_LAB		0x04

#define I2C_NL_MAX_XFR_LEN	0xfffcu
#define XFRBUF_DATA_LEN_MAX	(CONFIG_I2C_MCHP_MEC5_NL_BUFFER_SIZE)

#define I2C_NL_DMA_BLK_WR	0
#define I2C_NL_DMA_BLK_RD	1

struct i2c_mec5_nl_data {
	struct mec_i2c_smb_ctx ctx;
	struct k_sem lock;
	struct k_event events;
	uint32_t uconfig;
	uint32_t i2c_status;
#ifdef CONFIG_I2C_TARGET
	uint8_t target_addr[2];
#endif
	uint8_t cm_target_wr_i2c_addr;
	uint8_t speed_id;
	uint8_t state;
	uint8_t mdone;
	uint8_t rsvd1[2];
	uint16_t nl_ntxb;
	uint16_t nl_nrxb;
#ifdef CONFIG_I2C_CALLBACK
	i2c_callback_t i2c_cb;
	void *i2c_cb_udata;
#endif
	struct dma_config dma_tx_cfg;
	struct dma_config dma_rx_cfg;
	struct dma_block_config dblk[I2C_MEC5_NL_DMA_BLK_CFGS];
	uint8_t xfrbuf[CONFIG_I2C_MCHP_MEC5_NL_BUFFER_SIZE + 4];
#ifdef I2C_NL_DEBUG_ISR
	volatile uint32_t dbg_isr_cnt;
	volatile uint32_t dbg_isr_sts;
	volatile uint32_t dbg_isr_compl;
	volatile uint32_t dbg_isr_cfg;
#endif
#ifdef MEC5_I2C_DEBUG_STATE
	volatile uint32_t dbg_state_idx;
	uint8_t dbg_states[MEC5_I2C_DEBUG_STATE_ENTRIES];
#endif
};

#ifdef I2C_NL_DEBUG_ISR

static inline void i2c_mec5_nl_dbg_isr_init(struct i2c_mec5_data *data)
{
	data->dbg_isr_cnt = 0;
	data->dbg_isr_sts = 0;
	data->dbg_isr_compl = 0;
	data->dbg_isr_cfg = 0;
}
#define I2C_NL_DEBUG_ISR_INIT() i2c_mec5_nl_dbg_isr_init()
#else
#define I2C_NL_DEBUG_ISR_INIT()
#endif

#ifdef I2C_NL_DEBUG_STATE
static void i2c_mec5_nl_dbg_state_init(struct i2c_mec5_data *data)
{
	data->dbg_state_idx = 0u;
	memset((void *)data->dbg_states, 0, sizeof(data->dbg_states));
}

static void i2c_mec5_nl_dbg_state_update(struct i2c_mec5_data *data, uint8_t state)
{
	uint32_t idx = data->dbg_state_idx;

	if (data->dbg_state_idx < I2C_NL_DEBUG_STATE_ENTRIES) {
		data->dbg_states[idx] = state;
		data->dbg_state_idx = ++idx;
	}
}
#define I2C_NL_DEBUG_STATE_INIT(pd) i2c_mec5_nl_dbg_state_init(pd)
#define I2C_NL_DEBUG_STATE_UPDATE(pd, state) i2c_mec5_nl_dbg_state_update(pd, state)
#else
#define I2C_NL_DEBUG_STATE_INIT(pd)
#define I2C_NL_DEBUG_STATE_UPDATE(pd, state)
#endif

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
	mcfg.target_addr1 = devcfg->target_addr[0];
	mcfg.target_addr2 = devcfg->target_addr[1];

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

/* i2c_get_config API */
static int i2c_mec5_nl_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_mec5_nl_data *data = dev->data;
	uint32_t dcfg = 0u;

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
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t evw = (BIT(I2C_NL_KEV_IDLE_POS) | BIT(I2C_NL_KEV_BERR_POS)
			| BIT(I2C_NL_KEV_LAB_ERR_POS));
	uint32_t ev = 0;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x20);

	/* Can we trust GIRQ status has been cleared before we re-enable? */
	if (mec_hal_i2c_smb_is_bus_owned(hwctx)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x21);
		data->mdone = 0;
		mec_hal_i2c_smb_stop_gen(hwctx);
		mec_hal_i2c_smb_girq_status_clr(hwctx);
		mec_hal_i2c_smb_idle_intr_enable(hwctx, 1);
		mec_hal_i2c_smb_girq_ctrl(hwctx, MEC_I2C_SMB_GIRQ_EN);
#ifdef MEC5_I2C_DEBUG_USE_SPIN_LOOP
		while (!data->mdone) {
			;
		}
#else
		ev = k_event_wait(&data->events, evw, false, K_MSEC(100));
		if (!ev) {
			LOG_ERR("Gen STOP timeout");
		}
#endif
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x22);
	}

	data->state = I2C_NL_STATE_CLOSED;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x23);

	return 0;
}

/* Due to the nature of the I2C network layer HW design we limit acceptable
 * I2C messages to: I2C Read, I2C Write, and I2C combined write-read where
 * The last struct i2c_msg must have the I2C_MSG_STOP flag set.
 * Supported messages:
 * I2C_WRITE with I2C_MSG_STOP flag in last message
 * I2C_READ with I2C_MSG_STOP flag in last message
 * I2C combined Write-Read: 1st message is write with no stop, 2nd message is
 * read with I2C_MSG_STOP flag.
 */
static int i2c_mec5_nl_check_msgs(struct i2c_msg *msgs, uint8_t num_msgs)
{
	uint32_t txlen = 0, rxlen = 0;
	bool ovfl = false;

	if (num_msgs > 2u) {
		LOG_ERR("More than 2 messages");
		return -EINVAL;
	}

	for (uint8_t n = 0u; n < num_msgs; n++) {
		struct i2c_msg *m = &msgs[n];

		if (m->buf == NULL) {
			LOG_ERR("Message buffer is NULL");
			return -EINVAL;
		}

		if (m->len == 0) {
			LOG_ERR("Message length is 0");
			return -EINVAL;
		}

		if (m->flags & I2C_MSG_ADDR_10_BITS) {
			LOG_ERR("HW does not support 10-bit addresses");
			return -EINVAL;
		}

		if (m->flags & I2C_MSG_READ) {
			ovfl = u32_add_overflow(m->len, rxlen, &rxlen);
		} else {
			ovfl = u32_add_overflow(m->len, txlen, &txlen);
		}
		if (ovfl) {
			LOG_ERR("Message sizes overflowed 32-bits");
			return -EINVAL;
		}

		if (n == (num_msgs - 1u)) {
			if (!(m->flags & I2C_MSG_STOP)) {
				LOG_ERR("Last message does not have STOP flag set");
				return -EINVAL;
			}
		}
	}

	if ((txlen > I2C_NL_MAX_XFR_LEN) || (rxlen > I2C_NL_MAX_XFR_LEN)) {
		LOG_ERR("TX and/or RX total msg len exceed HW capabilites");
		return -EINVAL;
	}

	if (num_msgs == 2u) {
		if (((msgs[0].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) ||
		    ((msgs[1].flags & I2C_MSG_RW_MASK) != I2C_MSG_READ)) {
			LOG_ERR("Read followed by write in one transaction not supported");
			return -EINVAL;
		}
	}

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
 * TODO if a DMA error occurs we need to signal the I2C driver.
 * Currently the driver waits on a semaphore set by the I2C ISR.
 * Implementation will be changed to use Zephyr k_event flags and I2C transfer
 * routine will wait on multiple event flags: I2C ISR event(s) and DMA error event.
 */
static void i2c_mec5_nl_dma_cb(const struct device *dev, void *user_data,
			       uint32_t chan, int status)
{
	struct i2c_mec5_nl_data *data = (struct i2c_mec5_nl_data *)user_data;

	if (status < 0) {
		LOG_ERR("I2C-NL DMA CB error (%d): drv data at (%p) ",status, data);
		k_event_post(&data->events, BIT(I2C_NL_KEV_DMA_A_ERR_POS));
	}
}

static void i2c_mec5_nl_dma_cb2(const struct device *dev, void *user_data,
				uint32_t chan, int status)
{
	struct i2c_mec5_nl_data *data = (struct i2c_mec5_nl_data *)user_data;

	if (status < 0) {
		LOG_ERR("I2C-NL DMA CB2 error (%d): drv data at (%p)", status, data);
		k_event_post(&data->events, BIT(I2C_NL_KEV_DMA_B_ERR_POS));
	}
}

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

static void dblk_init(struct dma_block_config *blkcfg, void *src, void *dest, uint32_t nbytes,
		      bool mem2dev)
{
	blkcfg->source_address = (uint32_t)src;
	blkcfg->dest_address = (uint32_t)dest;
	blkcfg->block_size = nbytes;
	blkcfg->next_block = NULL;
	if (mem2dev) {
		blkcfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		blkcfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		blkcfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blkcfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}
}

static void dcfg_init(struct dma_config *dcfg, uint8_t slot, uint8_t dir, void *udata)
{

	dcfg->dma_slot = slot;
	if (dir == I2C_NL_DIR_WR) {
		dcfg->channel_direction = MEMORY_TO_PERIPHERAL;
	} else {
		dcfg->channel_direction = PERIPHERAL_TO_MEMORY;
	}

	dcfg->source_data_size = 1u;
	dcfg->dest_data_size = 1u;
	dcfg->block_count = 1u;
	dcfg->user_data = udata;
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

static int mec5_nl_i2c_xfr(const struct device *dev, struct i2c_msg *wrmsg,
			   struct i2c_msg *rdmsg, uint32_t xflags)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct mec_i2c_smb_regs *regs = devcfg->regs;
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct dma_config *dma_tx_cfg = &data->dma_tx_cfg;
	struct dma_config *dma_rx_cfg = &data->dma_rx_cfg;
	struct dma_block_config *tx_dma_blk1 = &data->dblk[0];
	struct dma_block_config *tx_dma_blk2 = &data->dblk[1];
	struct dma_block_config *tx_dma_blk3 = &data->dblk[3];
	struct dma_block_config *rx_dma_blk1 = &data->dblk[2];
	uint8_t *xbuf = data->xfrbuf;
	int err = 0;
	uint32_t ntx = 0, ntx_rem = 0;
	uint32_t hal_flags = (MEC_I2C_NL_FLAG_START | MEC_I2C_NL_FLAG_STOP
			      | MEC_I2C_NL_FLAG_CM_DONE_IEN);

	mec5_nl_hw_prep(dev);

	if (wrmsg && rdmsg) {
		hal_flags |= MEC_I2C_NL_FLAG_RPT_START;
	}

	data->mdone = 0;
	data->nl_ntxb = 0;
	data->nl_nrxb = 0;

	memset(dma_tx_cfg, 0, sizeof(struct dma_config));
	memset(dma_rx_cfg, 0, sizeof(struct dma_config));
	memset(data->dblk, 0, sizeof(struct dma_block_config) * I2C_MEC5_NL_DMA_BLK_CFGS);

	dcfg_init(dma_tx_cfg, devcfg->cm_dma_trigsrc, I2C_NL_DIR_WR, data);
	dcfg_init(dma_rx_cfg, devcfg->cm_dma_trigsrc, I2C_NL_DIR_RD, data);

	dma_tx_cfg->head_block = tx_dma_blk1;
	dma_tx_cfg->dma_callback = i2c_mec5_nl_dma_cb;

	dma_rx_cfg->head_block = rx_dma_blk1;
	dma_rx_cfg->dma_callback = i2c_mec5_nl_dma_cb2;

	if (wrmsg) {
		xbuf[0] = data->cm_target_wr_i2c_addr;
		ntx++; /* transmit wrAddr after start */
		if (wrmsg->len <= XFRBUF_DATA_LEN_MAX) {
			memcpy(&xbuf[1], wrmsg->buf, wrmsg->len);
			ntx += wrmsg->len;
			if (rdmsg) { /* store Sr rdAddr and adjust tx len */
				ntx++;
				xbuf[wrmsg->len + 1] = data->cm_target_wr_i2c_addr | BIT(0);
			}
		} else {
			memcpy(&xbuf[1], wrmsg->buf, XFRBUF_DATA_LEN_MAX);
			ntx += XFRBUF_DATA_LEN_MAX;
			ntx_rem = wrmsg->len - XFRBUF_DATA_LEN_MAX;
			if (rdmsg) {
				ntx++;
				xbuf[XFRBUF_DATA_LEN_MAX + 2] =
					data->cm_target_wr_i2c_addr | BIT(0);
			}
		}
	} else if (rdmsg) {
		xbuf[0] = data->cm_target_wr_i2c_addr | BIT(0);
		ntx = 1;
	}

	data->nl_ntxb = ntx + ntx_rem;

	dblk_init(tx_dma_blk1, data->xfrbuf, (void *)&regs->CM_TXB, ntx, true);

	if (ntx_rem) {
		dma_tx_cfg->block_count++;
		tx_dma_blk1->next_block = tx_dma_blk2;
		dblk_init(tx_dma_blk2, &wrmsg->buf[XFRBUF_DATA_LEN_MAX],
			  (void *)&regs->CM_TXB, ntx_rem, true);
		if (rdmsg) {
			dma_tx_cfg->block_count++;
			tx_dma_blk2->next_block = tx_dma_blk3;
			dblk_init(tx_dma_blk3, &xbuf[XFRBUF_DATA_LEN_MAX + 2],
				  (void *)&regs->CM_TXB, 1u, true);
		}
	}

	/* configure DMA for transmit phase */
	err = dma_config(devcfg->dma_dev_a, devcfg->dma_a_chan, dma_tx_cfg);
	if (err) {
		LOG_ERR("TX DMA cfg err (%d)", err);
		return err;
	}

	if (rdmsg) {
		dblk_init(rx_dma_blk1, (void *)&regs->CM_RXB, rdmsg->buf, rdmsg->len, false);
		/* configure DMA for receive phase */
		err = dma_config(devcfg->dma_dev_b, devcfg->dma_b_chan, dma_rx_cfg);
		if (err) {
			LOG_ERR("RX DMA cfg err (%d)", err);
			return err;
		}

		data->nl_nrxb = rdmsg->len;
	}

	/* Start TX DMA */
	err = dma_start(devcfg->dma_dev_a, devcfg->dma_a_chan);
	if (err) {
		LOG_ERR("TX DMA start err (%d)", err);
		return err;
	}

	if (xflags & MEC5_I2C_NL_XFLAG_START) {
		err = mec_hal_i2c_nl_cm_cfg_start(hwctx, data->nl_ntxb,
						  data->nl_nrxb, hal_flags);
		if (err) {
			LOG_ERR("I2C-NL start err (%d)", err);
			return -EIO;
		}
	}

	return 0;
}

#define I2C_NL_ERRORS \
	(BIT(I2C_NL_KEV_DMA_A_ERR_POS) | BIT(I2C_NL_KEV_DMA_B_ERR_POS) \
	 | BIT(I2C_NL_KEV_BERR_POS) | BIT(I2C_NL_KEV_LAB_ERR_POS) \
	 | BIT(I2C_NL_KEV_CM_NAK_POS))

/* i2c_transfer API - Synchronous using interrupts */
static int i2c_mec5_nl_transfer_cmn(const struct device *dev, struct i2c_msg *msgs,
				    uint8_t num_msgs, uint16_t addr, bool async,
				    i2c_callback_t cb, void *userdata)
{
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct i2c_msg *wrmsg = NULL;
	struct i2c_msg *rdmsg = NULL;
	uint32_t xflags = 0;
	uint32_t ev = 0, ev_wait = 0;
	int ret = 0;

	if (!msgs || !num_msgs) {
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER); /* decrements count */
	k_event_clear(&data->events, UINT32_MAX);

	I2C_NL_DEBUG_ISR_INIT();

#ifdef CONFIG_I2C_CALLBACK
	data->i2c_cb = cb;
	data->i2c_cb_udata = userdata;
#endif

	ret = i2c_mec5_nl_check_msgs(msgs, num_msgs);
	if (ret) {
		goto mec5_unlock;
	}

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

	data->state = I2C_NL_STATE_OPEN;
	data->cm_target_wr_i2c_addr = fmt_addr(addr, I2C_NL_FMT_ADDR_WR);

	if (num_msgs == 1) {
		wrmsg = &msgs[0];
		if (msgs[0].flags & I2C_MSG_READ) {
			rdmsg = &msgs[0];
		}
	} else {
		wrmsg = &msgs[0];
		rdmsg = &msgs[1];
	}

	xflags = MEC5_I2C_NL_XFLAG_START;
	ret = mec5_nl_i2c_xfr(dev, wrmsg, rdmsg, xflags);
	if (ret) { /* if error issue STOP if bus is still owned by controller */
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x7);
		do_stop(dev);
		goto mec5_unlock;
	}

#ifdef CONFIG_I2C_CALLBACK
	if (async) {
		return 0;
	}
#endif

	ev_wait = (BIT(I2C_NL_KEV_IDLE_POS) | BIT(I2C_NL_KEV_CM_NAK_POS)
		   | BIT(I2C_NL_KEV_DMA_A_ERR_POS) | BIT(I2C_NL_KEV_DMA_B_ERR_POS));
	ev = k_event_wait(&data->events, ev_wait, false, K_MSEC(100));
	if (!ev) {
		ret = -ETIMEDOUT;
		LOG_ERR("I2C-NL CM timeout");
		do_stop(dev);
		goto mec5_unlock;
	}

	if (ev & I2C_NL_ERRORS) {
		LOG_ERR("CM event errors = 0x%08x", ev);
		ret = -EIO;
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

#ifdef CONFIG_I2C_CALLBACK
static int i2c_mec5_nl_transfer_cb(const struct device *dev, struct i2c_msg *msgs,
				   uint8_t num_msgs, uint16_t addr,
				   i2c_callback_t cb, void *userdata)
{
	return i2c_mec5_nl_transfer_cmn(dev, msgs, num_msgs, addr, true, cb, userdata);
}
#endif

/* Future: Controller can handle two targets */
static int i2c_mec5_nl_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
	return -ENOTSUP;
}

static int i2c_mec5_nl_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
	return -ENOTSUP;
}

/* Controller Mode ISR  */

/* TODO
 * CONFIG_I2C_TARGET_BUFFER_MODE if enabled use these two API's
 * instead of the per byte target API's
 *
 * Copy data from application buffer to driver TM buffer for TX
 * typedef void (*i2c_target_buf_write_received_cb_t)(
 *		struct i2c_target_config *config, uint8_t *ptr, uint32_t len);
 *
 * typedef int (*i2c_target_buf_read_requested_cb_t)(
 *		struct i2c_target_config *config, uint8_t **ptr, uint32_t *len);
 *Sets application ptr to driver data buffer and len to driver buffer length
 */
static void i2c_mec5_nl_isr(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct mec_i2c_smb_regs *regs = devcfg->regs;
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t i2c_hw_sts = 0, cm_event = 0, kev = 0;
	int idle_active = 0;
#ifdef CONFIG_I2C_CALLBACK
	int result = 0;
#endif
	I2C_NL_DEBUG_STATE_UPDATE(data, 0x80);

#ifdef I2C_NL_DEBUG_ISR
	data->dbg_isr_cnt++;
	data->dbg_isr_sts = sys_read8((mem_addr_t)hwctx->base);
	data->dbg_isr_compl = sys_read32((mem_addr_t)hwctx->base + 0x20u);
	data->dbg_isr_cfg = sys_read32((mem_addr_t)hwctx->base + 0x28u);
	while (data->mdone) { /* should not hang here */
		;
	}
#endif
	idle_active = mec_hal_i2c_smb_is_idle_intr(hwctx);
	if (idle_active) { /* turn off as soon as possible */
		mec_hal_i2c_smb_idle_intr_enable(hwctx, 0);
		data->mdone = 1;
		kev |= BIT(I2C_NL_KEV_IDLE_POS);
	}

	i2c_hw_sts = mec_hal_i2c_smb_status(hwctx, 1);
	data->i2c_status = i2c_hw_sts;
	mec_hal_i2c_smb_wake_status_clr(hwctx);
	mec_hal_i2c_smb_girq_status_clr(hwctx);

	/* Bus Error? */
	if (i2c_hw_sts & BIT(MEC_I2C_STS_LL_BER_POS)) {
		data->mdone = 1;
		kev |= BIT(I2C_NL_KEV_BERR_POS);
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x81);
	}

	/* Lost Arbitration */
	if (i2c_hw_sts & BIT(MEC_I2C_STS_LL_LAB_POS)) {
		data->mdone = 1;
		kev |= BIT(I2C_NL_KEV_LAB_ERR_POS);
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x82);
	}

	/* Does a NAK to I2C-NL CM cause HW to auto-generate STOP? */
	if (data->i2c_status & BIT(MEC_I2C_STS_CM_TX_NACK_POS)) {
		data->mdone = 1;
		kev |= BIT(I2C_NL_KEV_CM_NAK_POS);
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x84);
	}

	if (data->mdone) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x8f);
		k_event_post(&data->events, kev);
		if (kev & BIT(I2C_NL_KEV_IDLE_POS)) {
#ifdef CONFIG_I2C_CALLBACK
			k_sem_give(&data->lock);
			if (data->i2c_cb) {
				if (kev & I2C_NL_ERRORS) {
					result = -EIO;
				}
				data->i2c_cb(dev, result, data->i2c_cb_udata);
			}
#endif
		} else {
			mec_hal_i2c_smb_idle_intr_enable(hwctx, 1);
		}
		return;
	}

	cm_event = mec_hal_i2c_nl_cm_event(regs);

	if (cm_event == MEC_I2C_NL_CM_EVENT_W2R) {
		dma_start(devcfg->dma_dev_b, devcfg->dma_b_chan);
		mec_hal_i2c_nl_cm_proceed(regs);
		k_event_post(&data->events, BIT(I2C_NL_KEV_W2R_POS));
	} else if (cm_event == MEC_I2C_NL_CM_EVENT_ALL_DONE) {
		mec_hal_i2c_smb_idle_intr_enable(hwctx, 1);
		k_event_post(&data->events, BIT(I2C_NL_KEV_CM_DONE_POS));
	}
} /* i2c_mec5_isr */

static const struct i2c_driver_api i2c_mec5_nl_driver_api = {
	.configure = i2c_mec5_nl_configure,
	.get_config = i2c_mec5_nl_get_config,
	.transfer = i2c_mec5_nl_transfer,
	.target_register = i2c_mec5_nl_target_register,
	.target_unregister = i2c_mec5_nl_target_unregister,
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = i2c_mec5_nl_transfer_cb,
#endif
};

static int i2c_mec5_nl_init_dma(const struct device *dev)
{
	const struct i2c_mec5_nl_config *devcfg = dev->config;

	if (!device_is_ready(devcfg->dma_dev_a) || !device_is_ready(devcfg->dma_dev_b)) {
		LOG_ERR("One or both DMA devices not ready");
		return -EIO;
	}

	return 0;
}

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
	data->state = I2C_NL_STATE_CLOSED;
	data->i2c_status = 0;
	data->mdone = 0;

	ret = i2c_mec5_nl_init_dma(dev);
	if (ret) {
		return ret;
	}

	ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("pinctrl setup failed (%d)", ret);
		return ret;
	}

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

#define I2C_NL_DT_INST_TARG_ADDR(i, n) \
	COND_CODE_1(DT_INST_PROP_HAS_IDX(i, target_addrs, n), \
		   (DT_INST_PROP_BY_IDX(i, target_addrs, n)), (0))

#define I2C_NL_DEVICE(i)							\
										\
	PINCTRL_DT_INST_DEFINE(i);						\
										\
	static void i2c_mec5_nl_irq_config_func_##i(void);			\
										\
	static struct i2c_mec5_nl_data i2c_mec5_nl_data_##i;			\
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
		.dma_dev_a = I2C_NL_DT_INST_DMA_DEV(i, cm),			\
		.dma_dev_b = I2C_NL_DT_INST_DMA_DEV(i, tm),			\
		.dma_a_regs = I2C_NL_DT_INST_DMA_REG_ADDR(i, cm),		\
		.dma_b_regs = I2C_NL_DT_INST_DMA_REG_ADDR(i, tm),		\
		.dma_a_chan = I2C_NL_MEC5_DMA_CHAN(i, 0),			\
		.dma_b_chan = I2C_NL_MEC5_DMA_CHAN(i, 1),			\
		.cm_dma_trigsrc = I2C_NL_MEC5_DMA_TRIGSRC(i, 0),		\
		.tm_dma_trigsrc = I2C_NL_MEC5_DMA_TRIGSRC(i, 1),		\
		.target_addr[0] = I2C_NL_DT_INST_TARG_ADDR(i, 0),		\
		.target_addr[1] = I2C_NL_DT_INST_TARG_ADDR(i, 1),		\
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
