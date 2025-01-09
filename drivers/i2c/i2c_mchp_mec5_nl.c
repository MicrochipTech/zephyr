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
#include <zephyr/drivers/i2c/mchp_mec5_i2c.h>
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
#define I2C_NL_DEBUG_DMA
#define I2C_NL_DEBUG_DMA_ENTRIES		8

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

#define I2C_MEC5_NL_MISC_CFG_PORT_POS		0
#define I2C_MEC5_NL_MISC_CFG_PORT_MSK		0xfu

#define I2C_MEC5_NL_MAX_DMA_CFGS 4

enum i2c_mec5_nl_state {
	I2C_NL_STATE_CLOSED = 0,
	I2C_NL_STATE_CM,
	I2C_NL_STATE_TM,
	I2C_NL_STATE_MAX,
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
	I2C_NL_KEV_TM_RPT_WR_POS,
	I2C_NL_KEV_TM_RPT_RD_POS,
	I2C_NL_KEV_TM_TPROT_POS,
	I2C_NL_KEV_TNAKR_POS,
	I2C_NL_KEV_DMA_TM_DONE_POS,
	I2C_NL_KEV_DMA_TM_ERR_POS,
};

#define I2C_NL_EVENTS_ERRORS \
	(BIT(I2C_NL_KEV_BERR_POS) | BIT(I2C_NL_KEV_LAB_ERR_POS) | BIT(I2C_NL_KEV_CM_NAK_POS))

#define I2C_NL_WAIT_EVENTS_MSK (BIT(I2C_NL_KEV_IDLE_POS) | BIT(I2C_NL_KEV_CM_NAK_POS) \
				| BIT(I2C_NL_KEV_DMA_CM_ERR_POS) \
				| BIT(I2C_NL_KEV_DMA_TM_ERR_POS))

#define I2C_NL_ERRORS \
	(BIT(I2C_NL_KEV_DMA_CM_ERR_POS) | BIT(I2C_NL_KEV_DMA_TM_ERR_POS) \
	 | BIT(I2C_NL_KEV_BERR_POS) | BIT(I2C_NL_KEV_LAB_ERR_POS) \
	 | BIT(I2C_NL_KEV_CM_NAK_POS))

#define I2C_NL_ALL_TM_EVENTS \
	(BIT(I2C_NL_KEV_TM_AAT_POS) | BIT(I2C_NL_KEV_TM_DONE_POS) |\
	 BIT(I2C_NL_KEV_TM_TURN_AROUND_POS) | BIT(I2C_NL_KEV_DMA_TM_DONE_POS) |\
	 BIT(I2C_NL_KEV_DMA_TM_ERR_POS))

struct i2c_mec5_nl_config {
	struct mec_i2c_smb_regs *i2c_regs;
	uint32_t init_pin_wait_us;
	uint32_t cfg_pin_wait_us;
	struct gpio_dt_spec sda_gpio;
	struct gpio_dt_spec scl_gpio;
	void (*irq_config_func)(void);
	const struct pinctrl_dev_config *pcfg;
	uint8_t cm_dma_chan;
	uint8_t cm_dma_trigsrc;
	uint16_t cm_tx_buf_max_sz;
#ifdef CONFIG_I2C_TARGET
	uint16_t tm_rx_buf_sz;
	uint8_t *tm_rx_buf;
	uint8_t tm_dma_chan;
	uint8_t tm_dma_trigsrc;
#endif
};

#ifdef I2C_NL_DEBUG_ISR
struct i2c_mec5_dbg_isr_data {
	uint32_t isr_cnt;
	uint32_t sts;
	uint32_t cfg;
	uint32_t cm_cmd;
	uint32_t tm_cmd;
	uint32_t ev;
};
#endif

#ifdef I2C_NL_DEBUG_DMA
struct i2c_mec5_dbg_dma {
	uint32_t maddr;
	uint32_t maddr_end;
	uint32_t daddr;
	uint32_t ctrl;
	uint32_t intr_act; /* byte[0]=chan, byte[1]=actv, byte[2]=status, byte[3]=ien */
};
#endif

struct i2c_mec5_nl_data {
	const struct i2c_mec5_nl_config *devcfg;
	struct mec_i2c_smb_ctx ctx;
	struct k_sem lock;
	struct k_event events;
	uint32_t ev;
	uint32_t clock_freq_hz;
	uint32_t i2c_status;
	uint32_t xfr_tmout;
	uint8_t *xfrbuf;
	uint32_t xfrlen;
	struct i2c_msg *msgs;
	uint16_t total_rx_len;
	uint16_t total_tx_len;
	uint8_t num_msgs;
	uint8_t msgidx;
	uint8_t i2c_addr;
	uint8_t state;
	uint8_t xdone;
	uint8_t misc_cfg; /* b[3:0]=port, b[6:4]=rsvd, b[7]=0(CM), 1(TM) */
	uint8_t xflags;
	uint8_t didx;
	struct mec_dma_cfg3 *tx_dma_head;
	struct mec_dma_cfg3 *rx_dma_head;
	struct mec_dma_cfg3 *dma_curr;
	struct mec_dma_cfg3 dblks[I2C_MEC5_NL_MAX_DMA_CFGS];
#ifdef CONFIG_I2C_TARGET
	struct i2c_target_config *target1_cfg;
	struct i2c_target_config *target2_cfg;
	struct i2c_target_config *curr_target;
	struct mec_dma_cfg3 tm_dma_cfg;
	struct mec_dma_cfg3 tm_tx_dma_cfg;
	uint8_t *tm_rx_ptr;
	uint32_t tm_rd_cnt;
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
#ifdef I2C_NL_DEBUG_DMA
	volatile uint32_t dbg_cm_isr_cnt; // TODO rename
	volatile uint32_t dbg_cm_dma_idx;
	struct i2c_mec5_dbg_dma dbg_cm_dma[I2C_NL_DEBUG_DMA_ENTRIES];
#ifdef CONFIG_I2C_TARGET
	volatile uint32_t dbg_tm_isr_cnt; // TODO rename
	volatile uint32_t dbg_tm_dma_idx;
	struct i2c_mec5_dbg_dma dbg_tm_dma[I2C_NL_DEBUG_DMA_ENTRIES];
#endif
#endif
};

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
		data->dbg_isr_data[idx].sts = (regs->COMPL & 0xffffff00u) | (regs->STATUS & 0xffu);
		data->dbg_isr_data[idx].cfg = regs->CONFIG;
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
#endif /* #ifdef I2C_NL_DEBUG_ISR */

#ifdef I2C_NL_DEBUG_STATE
static void i2c_mec5_nl_dbg_state_init(struct i2c_mec5_nl_data *data)
{
	data->dbg_state_idx = ATOMIC_INIT(0);
	memset(data->dbg_states, 0, sizeof(data->dbg_states));
}

/* note: atomic_inc returns previous value */
static void i2c_mec5_nl_dbg_state_update(struct i2c_mec5_nl_data *data, uint8_t state)
{
	atomic_val_t idx = atomic_inc(&data->dbg_state_idx);

	if (idx < I2C_NL_DEBUG_STATE_ENTRIES) {
		data->dbg_states[idx] = state;
	}
}

#define I2C_NL_DEBUG_STATE_INIT(d) i2c_mec5_nl_dbg_state_init((d))
#define I2C_NL_DEBUG_STATE_UPDATE(d, state) i2c_mec5_nl_dbg_state_update((d), (state))
#else
#define I2C_NL_DEBUG_STATE_INIT(d)
#define I2C_NL_DEBUG_STATE_UPDATE(d, state)
#endif

#ifdef I2C_NL_DEBUG_DMA
static inline void i2c_mec5_nl_dbg_cm_dma_init(struct i2c_mec5_nl_data *data)
{
	data->dbg_cm_isr_cnt = 0;
	data->dbg_cm_dma_idx = 0;
	memset(data->dbg_cm_dma, 0, sizeof(data->dbg_cm_dma));
}

static inline void i2c_mec5_nl_dbg_cm_isr_cnt_update(struct i2c_mec5_nl_data *data)
{
	data->dbg_cm_isr_cnt++;
}

static inline void i2c_mec5_nl_dbg_cm_dma_update(struct i2c_mec5_nl_data *data)
{
	const struct i2c_mec5_nl_config *const devcfg = data->devcfg;
	uint32_t idx = data->dbg_cm_dma_idx;
	struct mec_dma_chan_regs *dma_regs = NULL;
	uint32_t temp = 0;

	dma_regs = (struct mec_dma_chan_regs *)mec_hal_dma_chan_reg_addr(devcfg->cm_dma_chan);

	data->dbg_cm_dma_idx++;

	if (idx < I2C_NL_DEBUG_DMA_ENTRIES) {
		temp = dma_regs->IEN;
		temp <<= 8;
		temp |= (dma_regs->ISTATUS & 0xffu);
		temp <<= 8;
		temp |= (dma_regs->ACTV & 0xffu);
		temp <<= 8;
		temp |= devcfg->cm_dma_chan;
		data->dbg_cm_dma[idx].maddr = dma_regs->MSTART;
		data->dbg_cm_dma[idx].maddr_end = dma_regs->MEND;
		data->dbg_cm_dma[idx].daddr = dma_regs->DSTART;
		data->dbg_cm_dma[idx].ctrl = dma_regs->CTRL;
		data->dbg_cm_dma[idx].intr_act = temp;
	}
}

#ifdef CONFIG_I2C_TARGET
static inline void i2c_mec5_nl_dbg_tm_dma_init(struct i2c_mec5_nl_data *data)
{
	data->dbg_tm_isr_cnt = 0;
	data->dbg_tm_dma_idx = 0;
	memset(data->dbg_tm_dma, 0, sizeof(data->dbg_tm_dma));
}

static inline void i2c_mec5_nl_dbg_tm_isr_cnt_update(struct i2c_mec5_nl_data *data)
{
	data->dbg_tm_isr_cnt++;
}

static inline void i2c_mec5_nl_dbg_tm_dma_update(struct i2c_mec5_nl_data *data)
{
	const struct i2c_mec5_nl_config *const devcfg = data->devcfg;
	uint32_t idx = data->dbg_tm_dma_idx;
	struct mec_dma_chan_regs *dma_regs = NULL;
	uint32_t temp = 0;

	dma_regs = (struct mec_dma_chan_regs *)mec_hal_dma_chan_reg_addr(devcfg->tm_dma_chan);

	data->dbg_tm_dma_idx++;

	if (idx < I2C_NL_DEBUG_DMA_ENTRIES) {
		temp = dma_regs->IEN;
		temp <<= 8;
		temp |= (dma_regs->ISTATUS & 0xffu);
		temp <<= 8;
		temp |= (dma_regs->ACTV & 0xffu);
		temp <<= 8;
		temp |= devcfg->tm_dma_chan;
		data->dbg_tm_dma[idx].maddr = dma_regs->MSTART;
		data->dbg_tm_dma[idx].maddr_end = dma_regs->MEND;
		data->dbg_tm_dma[idx].daddr = dma_regs->DSTART;
		data->dbg_tm_dma[idx].ctrl = dma_regs->CTRL;
		data->dbg_tm_dma[idx].intr_act = temp;
	}
}
#endif /* CONFIG_I2C_TARGET */

#define I2C_NL_DEBUG_CM_DMA_INIT(d) i2c_mec5_nl_dbg_cm_dma_init((d))
#define I2C_NL_DEBUG_CM_DMA_ISR_COUNT_UPDATE(d) i2c_mec5_nl_dbg_cm_isr_cnt_update((d))
#define I2C_NL_DEBUG_CM_DMA_DATA_UPDATE(d) i2c_mec5_nl_dbg_cm_dma_update((d))
#ifdef CONFIG_I2C_TARGET
#define I2C_NL_DEBUG_TM_DMA_INIT(d) i2c_mec5_nl_dbg_tm_dma_init((d))
#define I2C_NL_DEBUG_TM_DMA_ISR_COUNT_UPDATE(d) i2c_mec5_nl_dbg_tm_isr_cnt_update((d))
#define I2C_NL_DEBUG_TM_DMA_DATA_UPDATE(d) i2c_mec5_nl_dbg_tm_dma_update((d))
#endif
#else
#define I2C_NL_DEBUG_CM_DMA_INIT(d)
#define I2C_NL_DEBUG_CM_DMA_ISR_COUNT_UPDATE(d)
#define I2C_NL_DEBUG_CM_DMA_DATA_UPDATE(d)
#define I2C_NL_DEBUG_TM_DMA_INIT(d)
#define I2C_NL_DEBUG_TM_DMA_ISR_COUNT_UPDATE(d)
#define I2C_NL_DEBUG_TM_DMA_DATA_UPDATE(d)
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

	hwctx->base = devcfg->i2c_regs;
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

#if 0 /* Do we need this? */
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
#endif

/* Compute timeout based on message group write and read lengths
 * I2C-NL driver can handle messages of up to 0xFFFC bytes.
 * @ 100 KHz 6000 ms, @ 400 KHz 1500 ms, 1 MHz 600 ms
 * Over estimate 10 clocks / byte and add 10 ms to total.
 * if computed timeout < SMBus timeout of 35 ms then use 35 ms.
 */
static void msgs_compute_timeout(const struct device *dev)
{
	struct i2c_mec5_nl_data *const data = dev->data;
	uint32_t i2c_freq = data->clock_freq_hz;
	uint32_t i2c_clks = 0;
	uint32_t tmout = 0;

	i2c_clks = data->total_rx_len + data->total_tx_len;
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

		mec_hal_dma_chan_stop(devcfg->cm_dma_chan);
		/* TODO target mode? */
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
	k_timeout_t event_wait_timeout = K_MSEC(data->xfr_tmout);
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
		mec_hal_dma_chan_stop(devcfg->cm_dma_chan);
		ret = -EIO;
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x34u);

	return ret;
}

/* HW and driver limit nrx and ntx to 0xfff8u */
static void i2c_mec5_nl_xfr_data_init(const struct device *dev, struct i2c_msg *msgs,
				      uint8_t num_msgs, uint16_t addr)
{
	struct i2c_mec5_nl_data *const data = dev->data;

	data->total_rx_len = 0;
	data->total_tx_len = 0;
	data->xfrlen = 0;
	data->tx_dma_head = NULL;
	data->rx_dma_head = NULL;
	data->dma_curr = NULL;
	data->msgs = msgs;
	data->num_msgs = num_msgs;
	data->i2c_addr = (uint8_t)(addr & 0x7fu);
	data->msgidx = 0;
	data->didx = 0;
	data->xflags = 0;
	data->state = I2C_NL_STATE_CM;

	memset(data->dblks, 0, sizeof(data->dblks));
}

/* Check if HW and driver supports messages.
 * Support messages 7-bit I2C addressing only.
 * Maximum number of messages is 2.
 * One I2C write or one I2C read or I2C combined write-read.
 * Last message must have I2C_MSG_STOP flag.
 * Write message maximum length is HW limit I2C_MEC5_NL_MAX_XFR_LEN.
 * Read message maximum length is HW limit (I2C_MEC5_NL_MAX_XFR_LEN).
 */
static int req_is_supported(struct i2c_msg *msgs, uint8_t num_msgs)
{
	uint32_t nrx = 0, ntx = 0;
	uint8_t mseq = 0;

	if (!msgs || (num_msgs == 0) || (num_msgs > 2u)) {
		LOG_ERR("Bad message array and/or number of messages(> 2)");
		return false;
	}

	for (uint8_t n = 0; n < num_msgs; n++) {
		struct i2c_msg *m = &msgs[n];

		if ((m->buf == NULL) || (m->len == 0)) {
			LOG_ERR("I2C msg[%u] NULL buffer and/or zero length", n);
			return false;
		}

		if (m->flags & I2C_MSG_ADDR_10_BITS) {
			LOG_ERR("I2C msg[%u] has 10-bit address flag not supported by HW", n);
			return false;
		}

		mseq <<= 4;
		if (m->flags & I2C_MSG_READ) {
			if (m->len > I2C_MEC5_NL_MAX_XFR_LEN) {
				LOG_ERR("I2C read msg[%u] len %u exceeds HW limit %u", n, m->len,
					I2C_MEC5_NL_MAX_XFR_LEN);
				return false;
			}
			nrx += m->len;
			mseq |= 2u;
		} else {
			if (m->len > I2C_MEC5_NL_MAX_XFR_LEN) {
				LOG_ERR("I2C write msg[%u] len %u exceeds HW limit %u", n, m->len,
					I2C_MEC5_NL_MAX_XFR_LEN);
				return false;
			}
			ntx += m->len;
			mseq |= 1u;
		}
	}

	/* supported mseq are 0x01(wr), 0x02(rd), 0x12(wr-rd). Bad sequences are
	 * 0x11(wr-wr), 0x21(rd-wr), 0x22(rd-rd)
	 */
	if ((mseq == 0x11u) || ((mseq & 0xf0u) == 0x20u)) {
		LOG_ERR("HW can only do: write, read, or combined write-RptStart-read");
		return false;
	}

	return true;
}

static void append_to_dma_list(struct mec_dma_cfg3 **ptr_to_head, struct mec_dma_cfg3 *item)
{
	if (*ptr_to_head == NULL) {
		*ptr_to_head = item;
	} else {
		struct mec_dma_cfg3 *p = *ptr_to_head;

		while (p->next) {
			p = p->next;
		}
		p->next = item;
	}
}

/* flags: b[0]=0(mem2dev), 1(dev2mem). b[1]=0(no interrupt), 1(enable DMA channel done) */
#define ADD_DMA_MEM2DEV 0
#define ADD_DMA_DEV2MEM 0x1
#define ADD_DMA_IEN 0x2

struct mec_dma_cfg3 *add_dma(const struct device *dev, uint8_t *src, uint32_t len, uint32_t flags)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_regs *i2c_regs = devcfg->i2c_regs;
	struct mec_dma_cfg3 *d = NULL;

	/* data->didx = index to unused struct mec_dma_cfg3 */
	if (data->didx >= I2C_MEC5_NL_MAX_DMA_CFGS) {
		return NULL;
	}

	d = &data->dblks[data->didx++];

	d->next = NULL;
	d->mem_addr = (uint32_t)src;
	d->dev_addr = (uint32_t)&i2c_regs->CM_TXB;
	d->nbytes = len;
	d->unitsz = MEC_DMAC_UNIT_SIZE_1;
	d->dir = MEC_DMAC_DIR_MEM_TO_DEV;
	d->hwfc_dev = devcfg->cm_dma_trigsrc;
	d->flags = MEC_DMA_CFG3_FLAG_INCR_MEM_ADDR;
	if (flags & ADD_DMA_IEN) {
		d->flags |= MEC_DMA_CFG3_FLAG_DONE_IEN;
	}

	if (flags & ADD_DMA_DEV2MEM) {
		d->dev_addr = (uint32_t)&i2c_regs->CM_RXB;
		d->dir = MEC_DMAC_DIR_DEV_TO_MEM;
		append_to_dma_list(&data->rx_dma_head, d);
	} else {
		append_to_dma_list(&data->tx_dma_head, d);
	}

	return d;
}

static int start_i2c_cm_xfr(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct i2c_msg *m = data->msgs;
	uint8_t *xbuf = data->xfrbuf;
	uint8_t *data_ptr = NULL;
	uint32_t data_len = 0, cm_flags = 0, ntx = 0, nrx = 0;
	int ret = 0;
	uint8_t target_i2c_addr = (data->i2c_addr & 0x7fu) << 1; /* write address */

	xbuf[0] = target_i2c_addr;
	data->xfrlen = 1;
	ntx = 1;

	data_ptr = m->buf;
	data_len = m->len;

	if (!(m->flags & I2C_MSG_READ)) { /* write direction? */
		ntx += data_len;
		if (data_len <= devcfg->cm_tx_buf_max_sz) {
			memcpy(&xbuf[1], data_ptr, data_len);
			data->xfrlen += data_len;
			if (data->num_msgs == 2) { /* add Rpt-START read address */
				xbuf[data_len + 1] = target_i2c_addr | BIT(0);
				ntx++;
				data->xfrlen++;
				cm_flags |= MEC_I2C_NL_FLAG_RPT_START;
			}
			if (!add_dma(dev, xbuf, ntx, ADD_DMA_MEM2DEV)) {
				return -ENOMEM;
			}
		} else { /* multi-DMA for TX */
			if (!add_dma(dev, xbuf, 1, ADD_DMA_MEM2DEV | ADD_DMA_IEN)) {
				return -ENOMEM;
			}
			if (!add_dma(dev, data_ptr, data_len, ADD_DMA_MEM2DEV | ADD_DMA_IEN)) {
				return -ENOMEM;
			}
			if (data->num_msgs == 2) {
				xbuf[1] = target_i2c_addr | BIT(0);
				if (!add_dma(dev, &xbuf[1], 1, ADD_DMA_MEM2DEV)) {
					return -ENOMEM;
				}
				ntx++;
				cm_flags |= MEC_I2C_NL_FLAG_RPT_START;
			}
		}
		if (data->num_msgs == 2u) {
			m++; /* second message must be read */
			add_dma(dev, m->buf, m->len, ADD_DMA_DEV2MEM);
			nrx = m->len;
		}
	} else { /* first and only message is I2C read */
		xbuf[0] |= BIT(0); /* target read address */
		if (!add_dma(dev, xbuf, ntx, ADD_DMA_MEM2DEV)) {
			return -ENOMEM;
		}
		if (!add_dma(dev, data_ptr, data_len, ADD_DMA_DEV2MEM)) {
			return -ENOMEM;
		}
		nrx = data_len;
	}

	ret = mec_hal_dma_chan_cfg3(devcfg->cm_dma_chan, data->tx_dma_head);
	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	data->dma_curr = data->tx_dma_head;
	mec_hal_dma_chan_start(devcfg->cm_dma_chan);

	cm_flags |= (MEC_I2C_NL_FLAG_START | MEC_I2C_NL_FLAG_STOP | MEC_I2C_NL_FLAG_CM_DONE_IEN);
	ret = mec_hal_i2c_nl_cm_cfg_start(hwctx, ntx, nrx, cm_flags);
	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	return 0;
}

#if 0
static bool is_i2c_nl_cm_error(uint32_t i2c_status)
{
	if (!(i2c_status & (BIT(MEC_I2C_STS_BERR_POS) | BIT(MEC_I2C_STS_LAB_POS)
			  | BIT(MEC_I2C_STS_CM_TX_NACK_POS)))) {
		return false;
	}

	if (i2c_status & BIT(MEC_I2C_STS_BERR_POS)) {
		LOG_ERR("CM BERR");
	}
	if (i2c_status & BIT(MEC_I2C_STS_LAB_POS)) {
		LOG_ERR("CM LAB");
	}
	if (i2c_status & BIT(MEC_I2C_STS_CM_TX_NACK_POS)) {
		LOG_ERR("CM NACK");
	}

	return true;
}
#endif

#define I2C_NL_CM_FSM_MSK (BIT(MEC_I2C_SMB_CM_CMD_RUN_Pos) | BIT(MEC_I2C_SMB_CM_CMD_PROCEED_Pos))
#define I2C_NL_TM_FSM_MSK (BIT(MEC_I2C_SMB_TM_CMD_RUN_Pos) | BIT(MEC_I2C_SMB_TM_CMD_PROCEED_Pos))

#define I2C_NL_CM_FSM_TA BIT(MEC_I2C_SMB_CM_CMD_RUN_Pos)
#define I2C_NL_TM_FSM_TA BIT(MEC_I2C_SMB_TM_CMD_RUN_Pos)

static uint32_t i2c_mec5_nl_get_events(const struct device *dev, uint32_t i2c_status)
{
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t cmd = 0u, ev = 0u;

	if (mec_hal_i2c_smb_is_idle_ien(hwctx) && (i2c_status & BIT(MEC_I2C_STS_IDLE_POS))) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x81u);
		mec_hal_i2c_smb_intr_ctrl(hwctx, BIT(MEC_I2C_IEN_IDLE_POS), 0);
		ev |= BIT(I2C_NL_KEV_IDLE_POS);
	}

	if (i2c_status & BIT(MEC_I2C_STS_BERR_POS)) {
		ev |= BIT(I2C_NL_KEV_BERR_POS);
	}
	if (i2c_status & BIT(MEC_I2C_STS_LAB_POS)) {
		ev |= BIT(I2C_NL_KEV_LAB_ERR_POS);
	}
	if (i2c_status & BIT(MEC_I2C_STS_CM_TX_NACK_POS)) {
		ev |= BIT(I2C_NL_KEV_CM_NAK_POS);
	}
	if (i2c_status & BIT(MEC_I2C_STS_CM_DONE_POS)) {
		ev |= BIT(I2C_NL_KEV_CM_DONE_POS);

		cmd = mec_hal_i2c_nl_cmd_get(hwctx, 0);
		if ((cmd & I2C_NL_CM_FSM_MSK) == I2C_NL_CM_FSM_TA) {
			ev |= BIT(I2C_NL_KEV_W2R_POS);
		}
	}
#ifdef CONFIG_I2C_TARGET
	if (i2c_status & BIT(MEC_I2C_STS_TM_DONE_POS)) {
		ev |= BIT(I2C_NL_KEV_TM_DONE_POS);

		cmd = mec_hal_i2c_nl_cmd_get(hwctx, 1);
		if ((cmd & I2C_NL_TM_FSM_MSK) == I2C_NL_TM_FSM_TA) {
			ev |= BIT(I2C_NL_KEV_TM_TURN_AROUND_POS);
		}
	}

	if (i2c_status & BIT(MEC_I2C_STS_TM_RPTS_WR_POS)) {
		ev |= BIT(I2C_NL_KEV_TM_RPT_WR_POS);
	}

	if (i2c_status & BIT(MEC_I2C_STS_TM_RPTS_RD_POS)) {
		ev |= BIT(I2C_NL_KEV_TM_RPT_RD_POS);
	}

	if (i2c_status & BIT(MEC_I2C_STS_TM_PERR_POS)) {
		ev |= BIT(I2C_NL_KEV_TM_TPROT_POS);
	}

	if (i2c_status & BIT(MEC_I2C_STS_TM_NACKR_POS)) {
		ev |= BIT(I2C_NL_KEV_TNAKR_POS);
	}
#endif
	mec_hal_i2c_smb_girq_status_clr(hwctx);

	return ev;
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

#if 0
struct i2c_target_callbacks {
	i2c_target_write_requested_cb_t write_requested;
	i2c_target_read_requested_cb_t read_requested;
	i2c_target_write_received_cb_t write_received;
	i2c_target_read_processed_cb_t read_processed;
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	i2c_target_buf_write_received_cb_t buf_write_received;
	i2c_target_buf_read_requested_cb_t buf_read_requested;
#endif
	i2c_target_stop_cb_t stop;
};

/** @brief Function called when a write to the device is completed.
 *
 * This function is invoked by the controller when it completes
 * reception of data from the source buffer to the destination
 * buffer in an ongoing write operation to the device.
 *
 * @param config the configuration structure associated with the
 * device to which the operation is addressed.
 *
 * @param ptr pointer to the buffer that contains the data to be transferred.
 *
 * @param len the length of the data to be transferred.
 */
typedef void (*i2c_target_buf_write_received_cb_t)(
		struct i2c_target_config *config, uint8_t *ptr, uint32_t len);

/** @brief Function called when a read from the device is initiated.
 *
 * This function is invoked by the controller when the bus is ready to
 * provide additional data by buffer for a read operation from the address
 * associated with the device.
 *
 * The value returned in @p **ptr and @p *len will be transmitted. A success
 * return shall cause the controller to react to additional read operations.
 * An error return shall cause the controller to ignore bus operations until
 * a new start condition is received.
 *
 * @param config the configuration structure associated with the
 * device to which the operation is addressed.
 *
 * @param ptr pointer to storage for the address of data buffer to return
 * for the read request.
 *
 * @param len pointer to storage for the length of the data to be transferred
 * for the read request.
 *
 * @return 0 if data has been provided, or a negative error code.
 */
typedef int (*i2c_target_buf_read_requested_cb_t)(
		struct i2c_target_config *config, uint8_t **ptr, uint32_t *len);
#endif

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

/* Handle target mode done interrupt status: CONFIG_I2C_TARGET_BUFFER_MODE enabled
 * TM done has multiple interpretations.
 * Experiment 1:
 *	TM DMA configured for dev2mem of Nd bytes
 *	I2C-NL TM_CMD rdCnt maximum 0xfff8u
 *	External Controller writes > Nd bytes.
 *	What happens?
 *
 * External Controller write to target(this device)
 *   I2C-NL TM_DONE interrupt: if captured target addres is write address
 *   invoke buf_write_received callback passing driver (dataptr, datalen)
 *   On return, re-enable TM DMA and I2C-NL to accept more data from external Controller.
 *
 * External Controller read from target(this device)
 *   I2C-NL TM_DONE  interrupt: if captured target address is read address
 *   NOTE: HW FSM probably clears TM_CMD PROCEED bit
 *   invoke buf_read_requested callback passing (&rx_buf_ptr, &rx_buf_len)
 *   callback fills in pointers with its buffer address and length.
 *   We configure TX DMA for mem2dev from the app buffer to I2C-NL.TM_TXB 8-bit register.
 *   Set TM_CMD PROCEED=1
 *
 * NOTE: In both scenarios above we must make sure TM_CMD wrCnt and rdCnt are always > 3.
 *       to prevent HW FSM from halting.
 *
 * void buf_write_received(struct i2c_target_config *config, uint8_t *ptr, uint32_t len)
 * void buf_read_requested(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len)
 * int stop(struct i2c_target_config *config)
 */

#if 0 /* unused */
static int i2c_mec5_nl_tm_rx_reconfig(const struct device *dev, uint32_t flags)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t tm_flags = MEC_I2C_NL_TM_FLAG_DONE_IEN | MEC_I2C_NL_TM_FLAG_RUN;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xA0u);

	/* reconfigure for next write data */
	if (flags & BIT(4)) {
		data->tm_dma_cfg.flags |= MEC_DMA_CFG3_FLAG_DONE_IEN;
	} else {
		data->tm_dma_cfg.flags &= (uint8_t)~MEC_DMA_CFG3_FLAG_DONE_IEN;
	}

	mec_hal_i2c_nl_tm_config(hwctx, I2C_MEC5_NL_MAX_XFR_LEN,
				 I2C_MEC5_NL_MAX_XFR_LEN, tm_flags);
	mec_hal_dma_chan_cfg3(devcfg->tm_dma_chan, &data->tm_dma_cfg);
	mec_hal_dma_chan_start(devcfg->tm_dma_chan);

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xA1u);

	return 0;
}
#endif

/* Configure I2C-NL target mode and corresponding DMA channel.
 * flags:
 *   b[0] = 1 enable I2C-NL target mode and DMA channel
 * We ONLY support receiving up to driver TM RX buffer limit of data as
 * specified by device tree.
 * I2C-NL uses the DMA to store the 8-bit target address with R/nW bit plus
 * data. Driver TM RX buffer is padded to allow room for alignment and
 * the target address. The TM RX buffer size from driver config is based
 * on buffer data size from device tree.
 * We configure TM DMA to store starting at offset 0x3 of TM RX buffer forcing
 * data to be aligned on a >= 4-byte boundary.
 */
static int i2c_mec5_nl_target_mode(const struct device *dev, uint8_t flags)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint8_t *tmrxb = devcfg->tm_rx_buf;
	uint32_t tm_flags = MEC_I2C_NL_TM_FLAG_DONE_IEN | MEC_I2C_NL_TM_FLAG_RUN;
	uint32_t clrmsk = BIT(MEC_I2C_IEN_IDLE_POS) | BIT(MEC_I2C_NL_IEN_CM_DONE_POS)
			  | BIT(MEC_I2C_NL_IEN_TM_DONE_POS) | BIT(MEC_I2C_NL_IEN_AAT_POS);
	uint16_t nrx = devcfg->tm_rx_buf_sz + 1u;
	uint16_t ntx = I2C_MEC5_NL_MAX_XFR_LEN;
	enum mec_dmac_channel tm_dma_chan = (enum mec_dmac_channel)devcfg->tm_dma_chan;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x50u);

	mec_hal_i2c_smb_intr_ctrl(hwctx, clrmsk, 0);
	mec_hal_i2c_nl_cmd_clear(hwctx, MEC_I2C_NL_TM_SEL);
	mec_hal_i2c_nl_flush_buffers(hwctx->base);

	/* clear all events before re-arming TM */
	k_event_clear(&data->events, I2C_NL_ALL_TM_EVENTS);

	data->tm_rx_ptr = &tmrxb[3];
	data->tm_rd_cnt = 0;

	if (flags & BIT(0)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x51u);
		mec_hal_i2c_nl_tm_config(hwctx, ntx, nrx, tm_flags);
		mec_hal_dma_chan_cfg3(tm_dma_chan, &data->tm_dma_cfg);
		I2C_NL_DEBUG_TM_DMA_DATA_UPDATE(data);
		mec_hal_dma_chan_start(tm_dma_chan);
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x52u);

	return 0;
}

static void i2c_mec5_nl_tm_stop_cb(const struct device *dev)
{
	struct i2c_mec5_nl_data *const data = dev->data;

	if (data->curr_target) {
		const struct i2c_target_callbacks *cbs = data->curr_target->callbacks;

		if (cbs && cbs->stop) {
			cbs->stop(data->curr_target);
		}
	}
}

/* TM events. More than one may be set.
 * For example I2C_NL_KEV_TM_TURN_AROUND_POS will be set with I2C_NL_KEV_TM_DONE_POS
 * Observation:
 *   I2C Read: TM_DONE set, STATUS.AAT=1, TM_CMD PROC:RUN 11b -> 01b
 *
 * BIT(I2C_NL_KEV_TM_DONE_POS)
 *  High level done. Check other bits. Could be pause or if TM PROCEED=RUN=0 then a count reached 0.
 *  ??? Is TM_DONE set for all the below events ???
 * BIT(I2C_NL_KEV_TM_TURN_AROUND_POS)
 *  HW FSM paused. Check other bits to determine why
 * BIT(I2C_NL_KEV_TM_RPT_WR_POS)
    HW FSM paused due to reception of I2C Rpt-START + wrAddr and wrAddr matched OWN_ADDR
 * BIT(I2C_NL_KEV_TM_RPT_RD_POS)
 *  HW FSM paused due to reception of I2C Rpt-START + rdAddr and rdAddr matched OWN_ADDR
 * BIT(I2C_NL_KEV_TM_TPROT_POS) one of
 *  1. TM wrCnt decremented to 0 before external NAK
 *  2. exernal NAK recevied before wrCnt reached 0
 * BIT(I2C_NL_KEV_TNAKR_POS);
 */
/* external Controller sent START or Rpt-START with rdAddr.
 * I2C-NL takes the 7-bit target address plus R/nW bit (1 for read).
 * It then compares the 7-bit address to the two addresses in its OWN_ADDR reg.
 * If the address matches I2C-NL will generate an ACK on the 9th clock, drive
 * SCL low to clock stretch, clear TM_CMD.PROCEED bit, and generate TM_DONE
 * interrupt. After the ACK I2C-NL also triggers TM DMA to read the target
 * address from the TM_RXB register and store it in memory.
 * We need to re-configure TM DMA for mem2dev, start TM DMA, and
 * set TM_CMD.PROCEED to 1. This will cause I2C-NL to stop clock stretching and
 * the external Controller will begin generating clocks. I2C-NL will drive the
 * data onto SDA and check for (n)ACK on the 9th clock of each data byte.
 * Zephyr I2C target mode expects the driver to invoke an application callback
 * to get (data buffer pointer, len).
 * ISSUES:
 * I2C-NL wrCnt has been programmed to our allowed maximum (0xfff8u).
 * Do limit app to one buffer of data per read? If yes, then we must reprogram
 * wrCnt to the exact buffer size. If no, then we reprogram wrCnt to max (0xfff8u)
 * and depend on external Controller NAK of last data byte it wants to receive.
 * We assume NAK will cause I2C-NL to trigger another interrupt.
 */
static void i2c_mec5_nl_tm_handler(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct mec_i2c_smb_regs *i2c_regs = devcfg->i2c_regs;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct mec_dma_cfg3 *tmd = &data->tm_tx_dma_cfg;
	uint8_t *tmrxb = &devcfg->tm_rx_buf[4];
	uint32_t rxmax = devcfg->cm_tx_buf_max_sz + 1u;
	uint32_t ev = data->ev;
	uint32_t data_len = 0;
	uint8_t i2c_addr = 0;
	enum mec_dmac_channel chan = devcfg->tm_dma_chan;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x90u);

	data->state = I2C_NL_STATE_TM;

	i2c_addr = mec_hal_i2c_nl_shad_addr_get(i2c_regs);
	data->tm_cmd_addr = (uint32_t)i2c_addr << 24;
	data->tm_cmd_addr |= (mec_hal_i2c_nl_cmd_get(hwctx, 1) & 0x00ffffffu);

	data->curr_target = i2c_target_find(dev, i2c_addr);
	if (!data->curr_target) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x9Eu);
		LOG_ERR("I2C-NL TM: I2C addr 0x%02x not registered", i2c_addr);
		return;
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x91u);
	const struct i2c_target_callbacks *cbs = data->curr_target->callbacks;


	if (ev & BIT(I2C_NL_KEV_TM_TURN_AROUND_POS)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x92u);

/* struct i2c_target_config *config, uint8_t **ptr, uint32_t *len */

		int ret = cbs->buf_read_requested(data->curr_target, &data->tm_tx_buf,
						  &data->tm_tx_buf_sz);
		/* TODO check pointer is NULL and len == 0
		 * ret == 0 means we can respond to more clocks to read more data
		 * ret != 0 means we don't repond (provide data)
		 * How do we abort the transfer?
		 */

		tmd->mem_addr = (uint32_t)data->tm_tx_buf;
		tmd->nbytes = data->tm_tx_buf_sz;

		mec_hal_dma_chan_cfg3(chan, &data->tm_tx_dma_cfg);
		I2C_NL_DEBUG_TM_DMA_DATA_UPDATE(data);
		mec_hal_dma_chan_start(chan);

		mec_hal_i2c_nl_tm_xfr_count_set(i2c_regs, MEC_I2C_NL_TM_DIR_TX, data->tm_tx_buf_sz);
		mec_hal_i2c_nl_tm_proceed(hwctx);
	} else {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x98u);
		if (i2c_addr & BIT(0)) { /* read request? */
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x99u);
		} else { /* write data received and moved into memory by TM DMA channel */
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x9Cu);
			if (cbs && cbs->buf_write_received) {
				I2C_NL_DEBUG_STATE_UPDATE(data, 0xA3u);
				data_len = (rxmax - mec_hal_i2c_nl_tm_xfr_count_get(hwctx, 1));
				/* remove target address from total received length */
				if (data_len) {
					data_len--;
				}

				cbs->buf_write_received(data->curr_target, tmrxb, data_len);
			}

			i2c_mec5_nl_target_mode(dev, 1);
			/* i2c_mec5_nl_tm_rx_reconfig(dev, 0); */
		}
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x9Fu);
}
#endif /* CONFIG_I2C_TARGET */

static void i2c_mec5_nl_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t cmd = 0;
	uint32_t i2c_status = 0, events = 0;
	enum i2c_mec5_nl_state curr_state = data->state;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x80u);
	I2C_NL_DEBUG_ISR_COUNT_UPDATE(data);
	I2C_NL_DEBUG_ISR_DATA_UPDATE(data);

	i2c_status = mec_hal_i2c_smb_status(hwctx, 1u);
	data->i2c_status = i2c_status;

	events = i2c_mec5_nl_get_events(dev, i2c_status);
	data->ev = events;
#if 1 /* HACK */
	uint32_t idx = data->dbg_isr_idx;

	if (idx) {
		idx--;
	}
	data->dbg_isr_data[idx].ev = events;
#endif
	if (events & I2C_NL_EVENTS_ERRORS) {
		goto i2c_mec5_nl_isr_exit;
	}

	if (events & BIT(I2C_NL_KEV_IDLE_POS)) {
		data->state = I2C_NL_STATE_CLOSED;
#ifdef CONFIG_I2C_TARGET
		if (curr_state == I2C_NL_STATE_TM) {
			i2c_mec5_nl_tm_stop_cb(dev);
		}
#endif
	} else {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x82u);
		mec_hal_i2c_smb_intr_ctrl(hwctx, BIT(MEC_I2C_IEN_IDLE_POS), 1);
	}

#ifdef CONFIG_I2C_TARGET
	if (i2c_targets_are_registered(dev)) {
		if (events & BIT(I2C_NL_KEV_TM_DONE_POS)) {
			i2c_mec5_nl_tm_handler(dev);
		}
	}
#endif

	if (events & BIT(I2C_NL_KEV_CM_DONE_POS)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x83u);
		if (events & BIT(I2C_NL_KEV_W2R_POS)) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x84u);
			/* Configure RX DMA. Support only one RX DMA message */
			if (!data->rx_dma_head) {
				/* !!!!Driver Error!!!! */
				LOG_ERR("I2C-NL ISR: CM_CMD=0x%0x driver RX DMA is NULL", cmd);
				mec_hal_i2c_smb_intr_ctrl(hwctx, 0x1eu, 0);
				goto i2c_mec5_nl_isr_exit;
			}
			mec_hal_dma_chan_cfg3(devcfg->cm_dma_chan, data->rx_dma_head);
			mec_hal_dma_chan_start(devcfg->cm_dma_chan);
			mec_hal_i2c_nl_cm_proceed(hwctx);
		}
	}

i2c_mec5_nl_isr_exit:
	if (events) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x8Eu);
		k_event_post(&data->events, events);
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x8Fu);
}

static void i2c_mec5_nl_cm_dma_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	enum mec_dmac_channel chan = devcfg->cm_dma_chan;
	struct mec_dma_cfg3 *dma_cfg = NULL;
	uint32_t dma_status = 0;

	I2C_NL_DEBUG_CM_DMA_ISR_COUNT_UPDATE(data);
	I2C_NL_DEBUG_STATE_UPDATE(data, 0xD0u);

	mec_hal_dma_chan_intr_en(chan, 0);
	mec_hal_dma_chan_intr_status(chan, &dma_status);
	mec_hal_dma_chan_intr_status_clr(chan);

	if (dma_status & BIT(MEC_DMA_CHAN_ISTATUS_BERR_Pos)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xDEu);
		LOG_ERR("I2C-NL DMA chan %u bus error", chan);
		/* TODO force I2C-NL stop and release lines */
		return;
	}

	/* we finished the DMA entry at data->dma_head, reconfigure for next entry in list */
	if (data->dma_curr) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xD1u);
		dma_cfg = data->dma_curr->next;
		data->dma_curr = dma_cfg;
		if (dma_cfg) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0xD2u);
			mec_hal_dma_chan_cfg3(devcfg->cm_dma_chan, dma_cfg);
			mec_hal_dma_chan_start(devcfg->cm_dma_chan);
		}
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xDFu);
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

/* Side-band API */
int i2c_mchp_nl_configure(const struct device *dev, uint32_t dev_config, uint8_t port_num)
{
	if (!dev || (port_num >= MCHP_I2C_NUM_PORTS)) {
		return -EINVAL;
	}

	struct i2c_mec5_nl_data *data = dev->data;

	data->misc_cfg &= (uint32_t)~I2C_MEC5_NL_MISC_CFG_PORT_MSK;
	data->misc_cfg |= (((uint32_t)port_num << I2C_MEC5_NL_MISC_CFG_PORT_POS)
			   & I2C_MEC5_NL_MISC_CFG_PORT_MSK);

	return i2c_mec5_nl_configure(dev, dev_config);
}

int i2c_mchp_nl_get_port(const struct device *dev, uint8_t *port_num)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct mec_i2c_smb_regs *regs = devcfg->i2c_regs;
	uint8_t port = mec_hal_i2c_smb_port_get(regs);

	if (!port_num) {
		return -EINVAL;
	}

	*port_num = port;
	return 0;
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

static int i2c_mec5_nl_transfer(const struct device *dev, struct i2c_msg *msgs,
				uint8_t num_msgs, uint16_t addr)
{
	struct i2c_mec5_nl_data *const data = dev->data;
	int ret = 0;

	if (addr & 0xff80u) {
		LOG_ERR("Bad I2C address: HW only supports 7-bit addresses");
		return -EINVAL;
	}

	if (!req_is_supported(msgs, num_msgs)) {
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER); /* decrements count */
	k_event_clear(&data->events, UINT32_MAX);

	I2C_NL_DEBUG_ISR_INIT(data);
	I2C_NL_DEBUG_STATE_INIT(data);
	I2C_NL_DEBUG_STATE_UPDATE(data, 1u);
	I2C_NL_DEBUG_CM_DMA_INIT(data);

	i2c_mec5_nl_xfr_data_init(dev, msgs, num_msgs, addr);
	msgs_compute_timeout(dev);

	ret = start_i2c_cm_xfr(dev);
	if (ret == 0) {
		ret = i2c_mec5_nl_wait_events(dev, I2C_NL_WAIT_EVENTS_MSK);
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 2u);

	data->state = I2C_NL_STATE_CLOSED;
	k_sem_give(&data->lock);

	return ret;
}

#ifdef CONFIG_I2C_TARGET
/* I2C-NL supports up to two 7-bit target addresses */
static int i2c_mec5_nl_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
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
		I2C_NL_DEBUG_TM_DMA_INIT(data);
		data->target1_cfg = cfg;
		mec_hal_i2c_smb_set_target_addr(&data->ctx, 0, targ_addr);
		ret = i2c_mec5_nl_target_mode(dev, 1);
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
		mec_hal_dma_chan_stop(devcfg->tm_dma_chan);
	}

	k_sem_give(&data->lock);

	return 0;
}
#else
/* I2C-NL supports up to two 7-bit target addresses */
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

/* Configure DMA channels used by I2C-NL driver.
 * I2C-NL uses Central DMA controller channels. We must not reset Central DMA unless
 * it has not yet been enabled.
 * NOTE: Target receive where external Controller writes to this device.
 * I2C-NL triggers TM DMA channel to store the target adddress byte followed by data.
 * We configure TM DMA memory buffer to be offset 3 in the drivers sized padded buffer.
 * This allows us to pass an aligned pointer to the application callback since all it
 * wants is the data.
 */
static int i2c_mec5_nl_dma_init(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	int ret = 0;

	/* HAL DMA init */
	if (!mec_hal_dmac_is_enabled()) {
		mec_hal_dmac_init(0);
	}

	ret = mec_hal_dma_chan_init(devcfg->cm_dma_chan);
	if (ret != MEC_RET_OK) {
		LOG_ERR("CM DMA chan init failed (%d)", ret);
		return -EIO;
	}

#ifdef CONFIG_I2C_TARGET
	struct mec_i2c_smb_regs *i2c_regs = devcfg->i2c_regs;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_dma_cfg3 *tmd = &data->tm_dma_cfg;

	tmd->next = NULL;
	tmd->mem_addr = (uint32_t)&devcfg->tm_rx_buf[3];
	tmd->dev_addr = (uint32_t)&i2c_regs->TM_RXB;
#if 0
	tmd->nbytes = devcfg->tm_rx_buf_sz + 1u; /* buffer created by driver is size padded */
#else
	tmd->nbytes = devcfg->tm_rx_buf_sz + 2u; /* EXPERIMENT. I2C-NL trigger TM DMA for bogus 0 after real data */
#endif
	tmd->unitsz = 1u;
	tmd->dir = MEC_DMAC_DIR_DEV_TO_MEM;
	tmd->hwfc_dev = devcfg->tm_dma_trigsrc;
	tmd->flags = MEC_DMA_CFG3_FLAG_INCR_MEM_ADDR;

	tmd = &data->tm_tx_dma_cfg;
	tmd->next = NULL;
	tmd->mem_addr = 0;
	tmd->nbytes = 0;
	tmd->dev_addr = (uint32_t)&i2c_regs->TM_TXB;
	tmd->unitsz = 1u;
	tmd->dir = MEC_DMAC_DIR_MEM_TO_DEV;
	tmd->hwfc_dev = devcfg->tm_dma_trigsrc;
	tmd->flags = MEC_DMA_CFG3_FLAG_INCR_MEM_ADDR;

	ret = mec_hal_dma_chan_init((enum mec_dmac_channel)devcfg->tm_dma_chan);
	if (ret != MEC_RET_OK) {
		LOG_ERR("TM DMA chan init failed (%d)", ret);
		return -EIO;
	}
#endif
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
	data->xfr_tmout = I2C_MEC5_NL_SMBUS_TMOUT_MAX_MS;

	hwctx->base = devcfg->i2c_regs;
	hwctx->i2c_ctrl_cached = 0;

	ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("pinctrl setup failed (%d)", ret);
		return ret;
	}

	/* HAL DMA init */
	ret = i2c_mec5_nl_dma_init(dev);
	if (ret) {
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
		mec_hal_dma_chan_ia_enable(devcfg->cm_dma_chan);
#ifdef CONFIG_I2C_TARGET
		mec_hal_dma_chan_ia_disable(devcfg->tm_dma_chan);
#endif
	}

	return 0;
}

/* Node id of DMA channel */
#define I2C_MEC5_NL_DT_INST_DMA_CTLR(i, name) DT_INST_DMAS_CTLR_BY_NAME(i, name)

#define I2C_MEC5_NL_DT_INST_DMA_DEV(i, name) \
	DEVICE_DT_GET(I2C_MEC5_NL_DT_INST_DMA_CTLR(i, name))

#define I2C_MEC5_NL_DT_MISC_CFG(i) \
	(uint8_t)(DT_INST_PROP_OR(i, port_sel, 0) & 0x0f)

#define I2C_NL_MEC5_DMA_CHAN(i, idx) \
	DT_INST_PHA_BY_IDX(i, dmas, idx, channel)

#define I2C_NL_MEC5_DMA_TRIGSRC(i, idx) \
	DT_INST_PHA_BY_IDX(i, dmas, idx, trigsrc)

#define I2C_NL_MEC5_DMA_CM_IRQN(i) \
	DT_IRQ_BY_IDX(I2C_MEC5_NL_DT_INST_DMA_CTLR(i, cm), I2C_NL_MEC5_DMA_CHAN(i, 0), irq)

#define I2C_NL_MEC5_DMA_CM_IRQ_PRI(i) \
	DT_IRQ_BY_IDX(I2C_MEC5_NL_DT_INST_DMA_CTLR(i, cm), I2C_NL_MEC5_DMA_CHAN(i, 0), priority)

#define I2C_NL_MEC5_CM_TX_BUF_SIZE(i) DT_INST_PROP(i, cm_tx_buf_size)

#define I2C_NL_MEC5_CM_TX_MEM_SIZE(i) \
	(I2C_NL_MEC5_CM_TX_BUF_SIZE(i) + I2C_MEC5_NL_CM_TX_BUF_PAD_LEN)

#define I2C_NL_MEC5_CM_TX_BUF(i) \
	static uint8_t i2c_mec5_nl_cm_tx_buf_##i[I2C_NL_MEC5_CM_TX_MEM_SIZE(i)] __aligned(4);

#ifdef CONFIG_I2C_TARGET

BUILD_ASSERT(IS_ENABLED(CONFIG_I2C_TARGET_BUFFER_MODE),
	     "Target buffer mode must be used when Target mode enabled in the build!");

#define I2C_NL_MEC5_TM_RX_BUF_SIZE(i) DT_INST_PROP(i, tm_rx_buf_size)

#define I2C_NL_MEC5_TM_RX_MEM_SIZE(i) \
	(I2C_NL_MEC5_TM_RX_BUF_SIZE(i) + I2C_MEC5_NL_TM_RX_BUF_PAD_LEN)

#define I2C_NL_MEC5_TM_RX_BUF(i) \
	static uint8_t i2c_mec5_nl_tm_rx_buf_##i[I2C_NL_MEC5_TM_RX_MEM_SIZE(i)] __aligned(4);

#define I2C_MEC5_NL_TM_DMA(i) \
	.tm_rx_buf_sz = (uint16_t)I2C_NL_MEC5_TM_RX_BUF_SIZE(i), \
	.tm_rx_buf = i2c_mec5_nl_tm_rx_buf_##i, \
	.tm_dma_chan = I2C_NL_MEC5_DMA_CHAN(i, 1), \
	.tm_dma_trigsrc = I2C_NL_MEC5_DMA_TRIGSRC(i, 1),

#define I2C_NL_MEC5_DMA_TM_IRQN(i) \
	DT_IRQ_BY_IDX(I2C_MEC5_NL_DT_INST_DMA_CTLR(i, tm), I2C_NL_MEC5_DMA_CHAN(i, 1), irq)

#define I2C_NL_MEC5_DMA_TM_IRQ_PRI(i) \
	DT_IRQ_BY_IDX(I2C_MEC5_NL_DT_INST_DMA_CTLR(i, tm), I2C_NL_MEC5_DMA_CHAN(i, 1), priority)

#else
#define I2C_NL_MEC5_TM_RX_BUF(i)
#define I2C_NL_MEC5_TM_RX_BUF_PTR(i)
#define I2C_MEC5_NL_TM_DMA(i)
#endif

#define I2C_MEC5_NL_DEVICE(i)                                                   \
	BUILD_ASSERT((I2C_NL_MEC5_DMA_CM_IRQ_PRI(i) < DT_INST_IRQ(i, priority)),\
		"CM DMA channel ISR priority must be higher than I2C");         \
	I2C_NL_MEC5_CM_TX_BUF(i)                                                \
	I2C_NL_MEC5_TM_RX_BUF(i)                                                \
	struct i2c_mec5_nl_data i2c_mec5_nl_data_##i = {                        \
		.clock_freq_hz = DT_INST_PROP(i, clock_frequency),		\
		.misc_cfg = I2C_MEC5_NL_DT_MISC_CFG(i),				\
		.xfrbuf = &i2c_mec5_nl_cm_tx_buf_##i[0],			\
	};									\
	PINCTRL_DT_INST_DEFINE(i);						\
	static void i2c_mec5_nl_irq_config_func_##i(void)			\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(i),					\
			    DT_INST_IRQ(i, priority),				\
			    i2c_mec5_nl_isr,					\
			    DEVICE_DT_INST_GET(i), 0);				\
		irq_enable(DT_INST_IRQN(i));					\
		IRQ_CONNECT(I2C_NL_MEC5_DMA_CM_IRQN(i), \
			    I2C_NL_MEC5_DMA_CM_IRQ_PRI(i), \
			    i2c_mec5_nl_cm_dma_isr, \
			    DEVICE_DT_INST_GET(i), 0); \
		irq_enable(I2C_NL_MEC5_DMA_CM_IRQN(i)); \
	}									\
	struct i2c_mec5_nl_config i2c_mec5_nl_devcfg_##i = {			\
		.i2c_regs = (struct mec_i2c_smb_regs *)DT_INST_REG_ADDR(i),	\
		.init_pin_wait_us = DT_INST_PROP_OR(i, init_pin_wait, 100),	\
		.cfg_pin_wait_us = DT_INST_PROP_OR(i, config_pin_wait, 35000),	\
		.sda_gpio = GPIO_DT_SPEC_INST_GET(i, sda_gpios),		\
		.scl_gpio = GPIO_DT_SPEC_INST_GET(i, scl_gpios),		\
		.irq_config_func = i2c_mec5_nl_irq_config_func_##i,		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),			\
		.cm_dma_chan = I2C_NL_MEC5_DMA_CHAN(i, 0),			\
		.cm_dma_trigsrc = I2C_NL_MEC5_DMA_TRIGSRC(i, 0),		\
		.cm_tx_buf_max_sz = (uint16_t)I2C_NL_MEC5_CM_TX_BUF_SIZE(i),	\
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
