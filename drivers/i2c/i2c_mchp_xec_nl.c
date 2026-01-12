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
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(i2c_xec_nl, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h" /* dependency on logging */

/* #define XEC_I2C_DEBUG_USE_SPIN_LOOP */
#define XEC_I2C_DEBUG_ISR
#define XEC_I2C_DEBUG_STATE
#define XEC_I2C_DEBUG_STATE_ENTRIES 256

#define RESET_WAIT_US 20

/* I2C timeout is  10 ms (WAIT_INTERVAL * WAIT_COUNT) */
#define WAIT_INTERVAL   50
#define WAIT_COUNT      200
#define STOP_WAIT_COUNT 500
#define PIN_CFG_WAIT    50

/* I2C recover SCL low retries */
#define I2C_XEC_RECOVER_SCL_LOW_RETRIES 10
/* I2C recover SDA low retries */
#define I2C_XEC_RECOVER_SDA_LOW_RETRIES 3
/* I2C recovery bit bang delay */
#define I2C_XEC_RECOVER_BB_DELAY_US     5
/* I2C recovery SCL sample delay */
#define I2C_XEC_RECOVER_SCL_DELAY_US    50

#define I2C_XEC_CTRL_WR_DLY 8

/* get_lines bit positions */
#define XEC_I2C_SCL_LINE_POS 0
#define XEC_I2C_SDA_LINE_POS 1
#define XEC_I2C_LINES_MSK    (BIT(XEC_I2C_SCL_LINE_POS) | BIT(XEC_I2C_SDA_LINE_POS))

#define XEC_I2C_CR_PIN_ESO_ACK                                                                     \
	(BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS))

#define XEC_I2C_CR_PIN_ESO_ENI_ACK (XEC_I2C_CR_PIN_ESO_ACK | BIT(XEC_I2C_CR_ENI_POS))

#define XEC_I2C_CR_START                                                                           \
	(BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_STA_POS) |             \
	 BIT(XEC_I2C_CR_ACK_POS))

#define XEC_I2C_CR_START_ENI (XEC_I2C_CR_START | BIT(XEC_I2C_CR_ENI_POS))

#define XEC_I2C_CR_RPT_START                                                                       \
	(BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_STA_POS) | BIT(XEC_I2C_CR_ACK_POS))

#define XEC_I2C_CR_RPT_START_ENI (XEC_I2C_CR_RPT_START | BIT(XEC_I2C_CR_ENI_POS))

#define XEC_I2C_CR_STOP                                                                            \
	(BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_STO_POS) |             \
	 BIT(XEC_I2C_CR_ACK_POS))

#define XEC_I2C_TM_HOST_READ_IGNORE_VAL 0xffu

#define XEC_I2C_NL_XFR_BUF_PAD_SZ 4

/* Controller hardware supports five I2C targets.
 * Two are configurable 7-bit addresses
 * I2C general call address 0
 * SMBus host address = 0x08
 * SMBus device address = 0x61
 */
#define XEC_I2C_NL_MAX_TARGETS 5

/* Driver atomic flags */
#define XEC_I2C_NL_AF_DMA_DONE_POS 0
#define XEC_I2C_NL_AF_DMA_ERR_POS  1
#define XEC_I2C_NL_AF_HDONE_POS    4
#define XEC_I2C_NL_AF_HNAK_POS     5
#define XEC_I2C_NL_AF_HP_POS       6

enum xec_i2c_state {
	XEC_I2C_STATE_CLOSED = 0,
	XEC_I2C_STATE_OPEN,
};

enum xec_i2c_error {
	XEC_I2C_ERR_NONE = 0,
	XEC_I2C_ERR_BUS,
	XEC_I2C_ERR_LOST_ARB,
	XEC_I2C_ERR_TIMEOUT,
};

enum xec_i2c_direction {
	XEC_I2C_DIR_NONE = 0,
	XEC_I2C_DIR_WR,
	XEC_I2C_DIR_RD,
};

enum xec_i2c_start {
	XEC_I2C_START_NONE = 0,
	XEC_I2C_START_NORM,
	XEC_I2C_START_RPT,
};

enum i2c_xec_isr_state {
	I2C_XEC_ISR_STATE_GEN_START = 0,
	I2C_XEC_ISR_STATE_CHK_ACK,
	I2C_XEC_ISR_STATE_WR_DATA,
	I2C_XEC_ISR_STATE_RD_DATA,
	I2C_XEC_ISR_STATE_GEN_STOP,
	I2C_XEC_ISR_STATE_EV_IDLE,
	I2C_XEC_ISR_STATE_NEXT_MSG,
	I2C_XEC_ISR_STATE_EXIT_1,
#ifdef CONFIG_I2C_TARGET
	I2C_XEC_ISR_STATE_TM_HOST_RD,
	I2C_XEC_ISR_STATE_TM_HOST_WR,
	I2C_XEC_ISR_STATE_TM_EV_STOP,
#endif
	I2C_XEC_ISR_STATE_MAX
};

enum i2c_xec_std_freq {
	XEC_I2C_STD_FREQ_100K = 0,
	XEC_I2C_STD_FREQ_400K,
	XEC_I2C_STD_FREQ_1M,
	XEC_I2C_STD_FREQ_MAX,
};

struct xec_i2c_timing {
	uint32_t freq_hz;
	uint32_t data_tm;    /* data timing  */
	uint32_t idle_sc;    /* idle scaling */
	uint32_t timeout_sc; /* timeout scaling */
	uint32_t bus_clock;  /* bus clock hi/lo pulse widths */
	uint8_t rpt_sta_htm; /* repeated start hold time */
};

struct xec_i2c_nl_config {
	mem_addr_t base;
	uint32_t clock_freq;
	struct gpio_dt_spec sda_gpio;
	struct gpio_dt_spec scl_gpio;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
	const struct device *hc_dma_dev;
	volatile uint8_t *xfrbuf;
	uint32_t xfrbuf_size;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t girq_wk;
	uint8_t girq_wk_pos;
	uint8_t pcr;
	uint8_t port;
	uint8_t hc_dma_chan;
	uint8_t hc_dma_trigsrc;
#ifdef CONFIG_I2C_TARGET
	const struct device *tc_dma_dev;
	uint8_t tc_dma_chan;
	uint8_t tc_dma_trigsrc;
#endif
};

#define I2C_XEC_XFR_FLAG_START_REQ 0x01
#define I2C_XEC_XFR_FLAG_STOP_REQ  0x02

#define I2C_XEC_XFR_STS_NACK 0x01
#define I2c_MEC5_XFR_STS_BER 0x02
#define I2c_MEC5_XFR_STS_LAB 0x04

struct i2c_xec_cm_xfr {
	volatile uint8_t *mbuf;
	volatile size_t mlen;
	volatile uint8_t xfr_sts;
	uint8_t mdir;
	uint8_t target_addr;
	uint8_t mflags;
};

/* We must support Zephyr i2c wrapper routines:
 * i2c_read - one I2C_MSG_READ
 * i2c_write - one I2C_MSG_WRITE
 * i2c_burst_write - two I2C_MSG_WRITE
 * i2c_write_read - one I2C_MSG_WRITE, one I2C_MSG_READ
 * i2c_write_read_dt calls i2_write_read
 * i2c_read_dt calls i2c_read
 * i2c_write_dt calls i2c_write
 * i2c_burst_write_dt calls i2c_burst_write
 * i2c_burst_read calls i2c_write_read
 * i2c_reg_read_byte calls i2c_write_read
 * i2c_reg_read_byte_dt calls i2c_reg_read_byte
 * i2c_reg_write_byte calls i2c_write
 * i2c_reg_write_byte_dt calls i2c_reg_write_byte
 * i2c_reg_update_byte calls i2c_reg_read_byte, i2c_reg_write_byte
 * i2c_reg_update_byte_dt calls i2c_reg_upadate_byte
 * Our message group must support
 * Minimum two I2C_MSG_WRITE and one I2C_MSG_READ
 *
 * DMA structure requirements if all DMA structures are configured before
 * the transaction is started.
 * 2 I2C_MSG_WRITE & 2 I2C_MSG_READ
 * Option 1: No internal write buffering
 * Two DMA config structures one for write and one for read
 * One DMA block config for address byte
 * One DMA block config for first write message
 * One DMA block config for second and last write message
 * One DMA block config for first read message
 * One DMA block config for second and last read message
 * Total:
 *  2 DMA config structures: 2 * 32       =  64 bytes
 *  5 DMA block config structures: 5 * 28 = 140 bytes
 *                                        = 204 bytes
 *
 * Option 2: Internal write buffer with both I2C_MSG_WRITE exceeding buffer size
 * two DMA config structures
 * One DMA block config for address plus partial first I2C_MSG_WRITE
 * One DMA block config for remainder of first I2C_MSG_WRITE
 * One DMA block config for second and last I2C_MSG_WRITE
 * One DMA block config for first read message
 * One DMA block config for second and last read message
 * Size size as Option 1
 *
 * The only advantage to Option 2 is performance. For messages that all fit in
 * the driver xfr buffer there is only one DMA block config required and no
 * timing gaps in waveform as DMA driver reloads with next DMA block config.
 */
struct i2c_xec_nl_mgroup {
	struct i2c_msg *wm;
	struct i2c_msg *rm;
	uint8_t nwm;
	uint8_t nrm;
};

#define XEC_I2C_NL_DMA_CFGS_MAX     2
#define XEC_I2C_NL_DMA_BLK_CFGS_MAX 5

struct xec_i2c_nl_data {
	struct k_work kwq;
	const struct device *dev;
	volatile uint32_t i2c_compl;
	volatile uint32_t dma_status;
	ATOMIC_DEFINE(aflags, 32);
	struct k_mutex lock_mut;
	struct k_sem sync_sem;
	struct i2c_msg *msgs;
	struct i2c_msg *msgs_end;
	uint32_t i2c_config;
	uint32_t clock_freq;
	uint32_t xfrbuf_len;
	volatile uint8_t mdone;
	uint8_t i2c_cr_shadow;
	uint8_t i2c_sr;
	uint8_t port_sel;
	uint8_t wraddr;
	uint8_t state;
	uint8_t xfr_state;
	uint8_t cm_dir;
	uint8_t tm_dir;
	struct dma_config dma_cfg[XEC_I2C_NL_DMA_CFGS_MAX];
	struct dma_block_config dma_blk_cfg[XEC_I2C_NL_DMA_BLK_CFGS_MAX];
#ifdef CONFIG_I2C_TARGET
	uint16_t targ_addr;
	uint8_t targ_data;
	uint8_t targ_ignore;
	uint8_t targ_active;
	uint8_t ntargets;
	uint8_t *targ_buf_ptr;
	uint32_t targ_buf_len;
	struct i2c_target_config *curr_target;
	struct i2c_target_config *target_cfgs[XEC_I2C_NL_MAX_TARGETS];
#endif
#ifdef XEC_I2C_DEBUG_STATE
	volatile uint32_t dbg_state_idx;
	uint8_t dbg_states[XEC_I2C_DEBUG_STATE_ENTRIES];
#endif
};

static const struct xec_i2c_timing xec_i2c_timing_tbl[] = {
	{KHZ(100), XEC_I2C_SMB_DATA_TM_100K, XEC_I2C_SMB_IDLE_SC_100K, XEC_I2C_SMB_TMO_SC_100K,
	 XEC_I2C_SMB_BUS_CLK_100K, XEC_I2C_SMB_RSHT_100K},
	{KHZ(400), XEC_I2C_SMB_DATA_TM_400K, XEC_I2C_SMB_IDLE_SC_400K, XEC_I2C_SMB_TMO_SC_400K,
	 XEC_I2C_SMB_BUS_CLK_400K, XEC_I2C_SMB_RSHT_400K},
	{MHZ(1), XEC_I2C_SMB_DATA_TM_1M, XEC_I2C_SMB_IDLE_SC_1M, XEC_I2C_SMB_TMO_SC_1M,
	 XEC_I2C_SMB_BUS_CLK_1M, XEC_I2C_SMB_RSHT_1M},
};

#ifdef XEC_I2C_DEBUG_ISR
volatile uint32_t i2c_xec_nl_isr_cnt;
volatile uint32_t i2c_xec_nl_isr_sts;
volatile uint32_t i2c_xec_nl_isr_compl;
volatile uint32_t i2c_xec_nl_isr_cfg;

static inline void xec_i2c_nl_dbg_isr_init(void)
{
	i2c_xec_nl_isr_cnt = 0;
}
#define XEC_I2C_DEBUG_ISR_INIT() xec_i2c_nl_dbg_isr_init()
#else
#define XEC_I2C_DEBUG_ISR_INIT()
#endif

#ifdef XEC_I2C_DEBUG_STATE
static void xec_i2c_nl_dbg_state_init(struct xec_i2c_nl_data *data)
{
	data->dbg_state_idx = 0u;
	memset((void *)data->dbg_states, 0, sizeof(data->dbg_states));
}

static void xec_i2c_nl_dbg_state_update(struct xec_i2c_nl_data *data, uint8_t state)
{
	uint32_t idx = data->dbg_state_idx;

	if (data->dbg_state_idx < XEC_I2C_DEBUG_STATE_ENTRIES) {
		data->dbg_states[idx] = state;
		data->dbg_state_idx = ++idx;
	}
}
#define XEC_I2C_DEBUG_STATE_INIT(pd)          xec_i2c_nl_dbg_state_init(pd)
#define XEC_I2C_DEBUG_STATE_UPDATE(pd, state) xec_i2c_nl_dbg_state_update(pd, state)
#else
#define XEC_I2C_DEBUG_STATE_INIT(pd)
#define XEC_I2C_DEBUG_STATE_UPDATE(pd, state)
#endif

/* NOTE: I2C controller detects Lost Arbitration during START, Rpt-START,
 * data, and ACK phases not during STOP phase.
 */

static int xec_i2c_nl_prog_standard_timing(const struct device *dev, uint32_t freq_hz)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t rb = devcfg->base;

	for (size_t n = 0; n < ARRAY_SIZE(xec_i2c_timing_tbl); n++) {
		const struct xec_i2c_timing *p = &xec_i2c_timing_tbl[n];

		if (freq_hz == p->freq_hz) {
			sys_write32(p->data_tm, rb + XEC_I2C_DT_OFS);
			sys_write32(p->idle_sc, rb + XEC_I2C_ISC_OFS);
			sys_write32(p->timeout_sc, rb + XEC_I2C_TMOUT_SC_OFS);
			sys_write16(p->bus_clock, rb + XEC_I2C_BCLK_OFS);
			sys_write8(p->rpt_sta_htm, rb + XEC_I2C_RSHT_OFS);

			return 0;
		}
	}

	return -EINVAL;
}

static void xec_i2c_nl_cr_write(const struct device *dev, uint8_t ctrl_val)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;

	data->i2c_cr_shadow = ctrl_val;
	sys_write8(ctrl_val, devcfg->base + XEC_I2C_CR_OFS);
}

#ifdef CONFIG_I2C_TARGET
static void xec_i2c_nl_cr_write_mask(const struct device *dev, uint8_t clr_msk, uint8_t set_msk)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;

	data->i2c_cr_shadow = (data->i2c_cr_shadow & (uint8_t)~clr_msk) | set_msk;
	sys_write8(data->i2c_cr_shadow, devcfg->base + XEC_I2C_CR_OFS);
}
#endif

static int wait_bus_free(const struct device *dev, uint32_t nwait)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	mem_addr_t rb = devcfg->base;
	uint32_t count = nwait;
	uint8_t sts = 0;

	while (count--) {
		sts = sys_read8(rb + XEC_I2C_SR_OFS);
		data->i2c_sr = sts;

		if ((sts & BIT(XEC_I2C_SR_NBB_POS)) != 0) {
			break; /* bus is free */
		}

		k_busy_wait(WAIT_INTERVAL);
	}

	/* check for bus error, lost arbitration or external stop */
	if (sts == (BIT(XEC_I2C_SR_NBB_POS) | BIT(XEC_I2C_SR_PIN_POS))) {
		return 0;
	}

	if ((sts & BIT(XEC_I2C_SR_BER_POS)) != 0) {
		return XEC_I2C_ERR_BUS;
	}

	if ((sts & BIT(XEC_I2C_SR_LAB_POS)) != 0) {
		return XEC_I2C_ERR_LOST_ARB;
	}

	return XEC_I2C_ERR_TIMEOUT;
}

/* return 0 if SCL and SDA are both high else return -EIO */
#if defined(CONFIG_SOC_SERIES_MEC172X)
static int check_lines(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
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

/* returns uint8_t with bit[0] = SCL and bit[1] = SDA */
static uint8_t get_lines(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	gpio_port_value_t sda = 0, scl = 0;
	uint8_t lines = 0;

	gpio_port_get_raw(devcfg->scl_gpio.port, &scl);
	gpio_port_get_raw(devcfg->sda_gpio.port, &sda);

	if ((sda & BIT(devcfg->scl_gpio.pin)) != 0) {
		lines |= BIT(XEC_I2C_SCL_LINE_POS);
	}

	if ((sda & BIT(devcfg->sda_gpio.pin)) != 0) {
		lines |= BIT(XEC_I2C_SDA_LINE_POS);
	}

	return lines;
}
#else
static int check_lines(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t rb = devcfg->base;
	uint8_t himsk = BIT(XEC_I2C_BBCR_SCL_IN_POS) | BIT(XEC_I2C_BBCR_SDA_IN_POS);
	uint8_t bbcr = 0;

	sys_write8(BIT(XEC_I2C_BBCR_CM_POS), rb + XEC_I2C_BBCR_OFS);
	bbcr = sys_read8(rb + XEC_I2C_BBCR_OFS);

	if ((bbcr & himsk) == himsk) {
		return 0;
	}

	return -EIO;
}

/* returns uint8_t with bit[0] = SCL and bit[1] = SDA */
static uint8_t get_lines(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t rb = devcfg->base;
	uint8_t bbcr = 0;
	uint8_t lines = 0;

	sys_write8(BIT(XEC_I2C_BBCR_CM_POS), rb + XEC_I2C_BBCR_OFS);
	bbcr = sys_read8(rb + XEC_I2C_BBCR_OFS);

	if ((bbcr & BIT(XEC_I2C_BBCR_SCL_IN_POS)) != 0) {
		lines |= BIT(XEC_I2C_SCL_LINE_POS);
	}

	if ((bbcr & BIT(XEC_I2C_BBCR_SDA_IN_POS)) != 0) {
		lines |= BIT(XEC_I2C_SDA_LINE_POS);
	}

	return lines;
}
#endif

#ifdef CONFIG_I2C_TARGET
#if 0
static int prog_dt_target_addresses(const struct device *dev, uint8_t enable)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	mem_addr_t rb = devcfg->base;
	uint32_t rv1 = 0, rv2 = 0, n = 0;
	int ret = 0;

	for (n = 0; n < data->ntargets; n++) {
		struct i2c_target_config *tcfg = target_cfgs[n];

		if (tcfg == NULL) {
			continue;
		}

		switch (tcfg->address) {
		case XEC_I2C_GEN_CALL_ADDR:
			if (enable != 0) {
				sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_GC_DIS_POS);
			} else {
				sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_GC_DIS_POS);
			}
			break;
		case XEC_I2C_SMB_HOST_ADDR:
			__fallthrough;
		case XEC_I2C_SMB_DEVICE_ADDR:
			if (enable != 0) {
				sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_DSA_POS);
			} else {
				sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_DSA_POS);
			}
			break;
		default:
			rv1 = sys_read32(rb + XEC_I2C_OA_OFS);
			rv2 = rv1;
			if ((rv2 & 0xffu) == 0) {
				rv2 |= (tcfg->address & 0x7fu);
			} else if ((regval & 0xff00) == 0) {
				rv2 |= ((tcfg->address & 0x7fu) << 8);
			} else {
				ret = -ENFILE;
			}

			if (rv2 != rv1) {
				sys_write32(rv2, rb + XEC_I2C_OA_OFS);
			}

			break;
		}

		if (enable == 0) {
			target_cfgs[n] = NULL;
		}
	}

	return ret;
}
#endif
#endif /* CONFIG_I2C_TARGET */

static int xec_i2c_nl_reset_config(const struct device *dev, uint8_t port)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	mem_addr_t rb = devcfg->base;
	uint32_t val = 0, i2c_cfg = 0, i2c_oa = 0;
	int rc = 0;
	uint8_t crval = 0;

	data->i2c_cr_shadow = 0;

	data->state = XEC_I2C_STATE_CLOSED;
	data->i2c_cr_shadow = 0;
	data->i2c_sr = 0;
	data->i2c_compl = 0;
	data->read_discard = 0;
	data->mdone = 0;

	/* save target addresses and enables */
	i2c_oa = sys_read32(rb + XEC_I2C_OA_OFS);
	i2c_cfg = sys_read32(rb + XEC_I2C_CFG_OFS) & (BIT(XEC_I2C_CFG_DSA_POS) |
	                                              BIT(XEC_I2C_CFG_GC_DIS_POS));

	if (data->i2c_config & I2C_MODE_CONTROLLER) {
		i2c_cfg |= BIT(XEC_I2C_CFG_GC_DIS_POS);
	}

	soc_xec_pcr_sleep_en_clear(devcfg->pcr);
	/* reset I2C controller using PCR reset feature */
	soc_xec_pcr_reset_en(devcfg->pcr);

	sys_write32(i2c_oa, rb + XEC_I2C_OA_OFS);
	sys_write32(i2c_cfg, rb + XEC_I2C_CFG_OFS);

	crval = BIT(XEC_I2C_CR_PIN_POS);
	xec_i2c_nl_cr_write(dev, crval);

	/* timing registers */
	xec_i2c_nl_prog_standard_timing(dev, data->clock_freq);

	/* enable output driver and ACK logic */
	crval = XEC_I2C_CR_PIN_ESO_ENI_ACK;
	xec_i2c_nl_cr_write(dev, crval);

	/* port and filter enable */
	val = XEC_I2C_CFG_PORT_SET((uint32_t)port);
	val |= BIT(XEC_I2C_CFG_FEN_POS);
	sys_set_bits(rb + XEC_I2C_CFG_OFS, val);

	/* Enable live monitoring of SDA and SCL. No effect on MEC15xx and MEC172x */
	sys_write8(BIT(XEC_I2C_BBCR_CM_POS), rb + XEC_I2C_BBCR_OFS);

	/* enable */
	sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);

	/* wait for NBB=1, BER, LAB, or timeout */
	rc = wait_bus_free(dev, WAIT_COUNT);

	return rc;
}

static int xec_i2c_nl_bb_recover(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	mem_addr_t rb = devcfg->base;
	int ret = 0;
	uint32_t cnt = I2C_XEC_RECOVER_SCL_LOW_RETRIES;
	uint8_t bbcr = 0;
	uint8_t lines = 0u;

	xec_i2c_nl_reset_config(dev, data->port_sel);

	lines = get_lines(dev);
	if ((lines & XEC_I2C_LINES_MSK) == XEC_I2C_LINES_MSK) {
		return 0;
	}

	/* Disconnect SDL and SDA from I2C logic and connect to bit-bang logic */
	bbcr = BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CM_POS);
	sys_write8(bbcr, rb + XEC_I2C_BBCR_OFS);

	lines = get_lines(dev);

	/* If SCL is low continue sampling hoping it will go high on its own */
	while (!(lines & BIT(XEC_I2C_SCL_LINE_POS))) {
		if (cnt) {
			cnt--;
		} else {
			break;
		}
		k_busy_wait(I2C_XEC_RECOVER_SCL_DELAY_US);
		lines = get_lines(dev);
	}

	lines = get_lines(dev);
	if ((lines & BIT(XEC_I2C_SCL_LINE_POS)) == 0) {
		ret = -EBUSY;
		goto disable_bb_exit;
	}

	/* SCL is high, check SDA */
	if ((lines & BIT(XEC_I2C_SDA_LINE_POS)) != 0) {
		ret = 0; /* both high */
		goto disable_bb_exit;
	}

	/* SCL is high and SDA is low. Loop generating 9 clocks until
	 * we observe SDA high or loop terminates
	 */
	ret = -EBUSY;
	for (int i = 0; i < I2C_XEC_RECOVER_SDA_LOW_RETRIES; i++) {
		bbcr = 0x81u; /* SCL & SDA tri-state (inputs) */
		sys_write8(bbcr, rb + XEC_I2C_BBCR_OFS);

		/* 9 clocks */
		for (int j = 0; j < 9; j++) {
			/* drive SCL low */
			bbcr = 0x83u; /* SCL output drive low, SDA tri-state input */
			sys_write8(bbcr, rb + XEC_I2C_BBCR_OFS);
			k_busy_wait(I2C_XEC_RECOVER_BB_DELAY_US);
			/* drive SCL high */
			bbcr = 0x81u; /* SCL & SDA tri-state inputs */
			sys_write8(bbcr, rb + XEC_I2C_BBCR_OFS);
			k_busy_wait(I2C_XEC_RECOVER_BB_DELAY_US);
		}

		lines = get_lines(dev);
		if ((lines & XEC_I2C_LINES_MSK) == XEC_I2C_LINES_MSK) { /* Both high? */
			ret = 0;
			goto disable_bb_exit;
		}

		/* generate I2C STOP.  While SCL is high SDA transitions low to high */
		bbcr = 0x85u; /* SCL tri-state input (high), drive SDA low */
		sys_write8(bbcr, rb + XEC_I2C_BBCR_OFS);
		k_busy_wait(I2C_XEC_RECOVER_BB_DELAY_US);
		bbcr = 0x81u; /* SCL and SDA tri-state inputs. */
		sys_write8(bbcr, rb + XEC_I2C_BBCR_OFS);
		k_busy_wait(I2C_XEC_RECOVER_BB_DELAY_US);

		lines = get_lines(dev);
		if ((lines & XEC_I2C_LINES_MSK) == XEC_I2C_LINES_MSK) { /* Both high? */
			ret = 0;
			goto disable_bb_exit;
		}
	}

disable_bb_exit:
	bbcr = 0x80u;
	sys_write8(0x80u, rb + XEC_I2C_BBCR_OFS);

	return ret;
}

static int xec_i2c_nl_recover_bus(const struct device *dev)
{
	struct xec_i2c_nl_data *const data = dev->data;
	int ret = 0;

	LOG_ERR("I2C attempt bus recovery\n");

	/* Try controller reset first */
	ret = xec_i2c_nl_reset_config(dev, data->port_sel);
	if (ret == 0) {
		ret = check_lines(dev);
	}

	if (ret != 0) {
		return 0;
	}

	ret = xec_i2c_nl_bb_recover(dev);
	if (ret == 0) {
		ret = wait_bus_free(dev, WAIT_COUNT);
	}

	return ret;
}

static int xec_i2c_nl_cfg(const struct device *dev, uint32_t dev_config_raw)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	uint8_t port = devcfg->port;

	switch (I2C_SPEED_GET(dev_config_raw)) {
	case I2C_SPEED_STANDARD:
		data->clock_freq = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		data->clock_freq = KHZ(400);
		break;
	case I2C_SPEED_FAST_PLUS:
		data->clock_freq = MHZ(1);
		break;
	default:
		return -EINVAL;
	}

	data->i2c_config = dev_config_raw;
#ifdef CONFIG_I2C_XEC_PORT_MUX
	port = I2C_XEC_PORT_GET(dev_config_raw);
#endif

	return xec_i2c_nl_reset_config(dev, port);
}

/* i2c_configure API */
static int xec_i2c_nl_configure(const struct device *dev, uint32_t dev_config_raw)
{
	struct xec_i2c_nl_data *const data = dev->data;
	int rc = 0;

	if (!(dev_config_raw & I2C_MODE_CONTROLLER)) {
		return -ENOTSUP;
	}

	rc = k_mutex_lock(&data->lock_mut, K_NO_WAIT);
	if (rc != 0) {
		return rc;
	}

	rc = xec_i2c_nl_cfg(dev, dev_config_raw);

	k_mutex_unlock(&data->lock_mut);

	return rc;
}

/* i2c_get_config API */
static int xec_i2c_nl_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct xec_i2c_nl_data *const data = dev->data;
	uint32_t dcfg = 0u;

	if (dev_config == NULL) {
		return -EINVAL;
	}

	dcfg = data->i2c_config;

#ifdef CONFIG_I2C_TARGET
	if (data->ntargets == 0) {
		dcfg |= I2C_MODE_CONTROLLER;
	} else {
		dcfg &= ~I2C_MODE_CONTROLLER;
	}
#else
	dcfg |= I2C_MODE_CONTROLLER;
#endif
	*dev_config = dcfg;

	return 0;
}

static void xec_i2c_nl_dma_cb(const struct device *dma_dev, void *user_data,
			      uint32_t chan, int status)
{
	struct xec_i2c_nl_data *data = (struct xec_i2c_nl_data *)user_data;
/*	const struct device *i2c_dev = data->dev; */
/*	const struct xec_i2c_nl_config *i2c_drvcfg = i2c_dev->config; */
	struct dma_status dstatus = {0};
	int rc = dma_get_status(dma_dev, chan, &dstatus);

	if (rc != 0) {
		atomic_set_bit(data->aflags, XEC_I2C_NL_AF_DMA_ERR_POS);
		LOG_ERR("HC DMA CB status error (%d)", rc);
		return;
	}

#if 0
	if (dstatus.dir == MEMORY_TO_PERIPHERAL) {

	} else if (dstatus.dir == PERIPHERAL_TO_MEMORY) {

	}
#endif
	atomic_set_bit(data->aflags, XEC_I2C_NL_AF_DMA_DONE_POS);
}

/* Configure struct dma_config based on flags:
 * bit[0] = 0 (Host mode), 1 (Target mode)
 * bit[1] = 0(memory to peripheral), 1(peripheral to memory)
 * bit[2] = 0(callback at list done), 1(callback on each block)
 * bit[3] = 0(enable error callback), 1(disable error callback)
 */
static int xec_i2c_nl_dma_cfg(const struct device *dev, uint32_t blk_cnt, uint32_t flags)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	struct dma_config *dcfg = &data->dma_cfg;

	memset(dcfg, 0, sizeof(struct dma_config));

	dcfg->dma_slot = devcfg->hc_dma_trigsrc;

	if ((flags & BIT(1)) == 0) {
		dcfg->channel_direction = MEMORY_TO_PERIPHERAL; /* transmit */
	} else {
		dcfg->channel_direction = PERIPHERAL_TO_MEMORY;
	}

	if ((flags & BIT(2)) != 0) {
		dcfg->complete_callback_en = 1u; /* cb on each block */
	}

	if ((flags & BIT(3)) != 0) {
		dcfg->error_callback_dis = 1u;
	}

	/* handshakes = 0 (HW) */

	dcfg->source_data_size = 1u;
	dcfg->dest_data_size = 1u;

	dcfg->block_count = blk_cnt;
	dcfg->head_block = &data->dma_blk_cfg[0];
	dcfg->user_data = (void *)data;

	dcfg->dma_callback = xec_i2c_nl_dma_cb;

	return 0;
}

static int xec_i2c_nl_dma_blk_cfg(const struct device *dev,
                                  struct dma_block_config *dblk,
                                  struct dma_block_config *next,
                                  uint32_t mem_addr, uint32_t nbytes, uint32_t flags)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t rb = devcfg->base;

	memset((void *)dblk, 0, sizeof(struct dma_block_config));

	dblk->block_size = nbytes;

	if ((flags & BIT(1)) == 0) { /* memory to peripheral */
		dblk->source_address = mem_addr;
		dblk->dest_address = rb;
		if ((flags & BIT(0)) == 0) { /* host mode? */
			dblk->dest_address += XEC_I2C_HTX_OFS;
		} else {
			dblk->dest_address += XEC_I2C_TTX_OFS;
		}
		dblk->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dblk->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		dblk->source_address = rb;
		if ((flags & BIT(0)) == 0) { /* host mode? */
			dblk->source_address += XEC_I2C_HRX_OFS;
		} else {
			dblk->source_address += XEC_I2C_TRX_OFS;
		}
		dblk->dest_address = mem_addr;
		dblk->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dblk->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}

	return 0;
}

/* XEC I2C controller support 7-bit addressing only.
 * Format 7-bit address for as it appears on the bus as an 8-bit
 * value with R/W bit at bit[0], 0(write), 1(read).
 */
static inline uint8_t i2c_xec_fmt_addr(uint16_t addr, uint8_t read)
{
	uint8_t fmt_addr = (uint8_t)((addr & 0x7fu) << 1);

	if (read != 0) {
		fmt_addr |= BIT(0);
	}

	return fmt_addr;
}

/* Microchip MEC I2C network layer hardware design limitations:
 * HW must know total number of bytes to be transmitted and received
 * before transaction is started. Transmit and receive lengths are
 * independent 16-bit register values. I2C-NL hardware uses a DMA
 * channel to move each byte transmitted to the I2C-NL TX data register.
 * This includes addresses for START and RPT-START. If HW PEC is enabled
 * the PEC CRC-8 transmitted when the HW write count reaches 0.
 * I2C-NL triggers the receive DMA channel to read each byte it has
 * shifted in and ACK'd. When the HW read count reaches 0 and if FW has
 * set the generate STOP bit in the Host command register HW will generate
 * I2C STOP and after minimum 1/2 I2C clock release the I2C lines.
 * When the I2C-NL HW FSM changes direction from transmit to read it cannot
 * go back to transmit until it issues a STOP. Therefore the messages we
 * can handler are limited to:
 * I2C Write
 * I2C Read
 * I2C Combined Write-Read
 * where Nw = number of consecutive write messages
 *       Nr = number of consecutive read messages
 * and total length of Nw messages and total length of Nr messages are both <= 0xFFF8 bytes
 * The driver will insert RPT-START in the last write if it detects a write to read direction
 * change.
 * The driver will insert STOP in the last read if it detects a read to write direction change.
 *
 * To implement Nw and Nr consecutive message we implement the driver as follows:
 * struct xec_i2c_nl_data contains
 *    one struct dma_config
 *    one struct dma_block_config
 *
 */
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
					m->flags |= I2C_MSG_RESTART;
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

#if 0
static int i2c_xec_xfr_begin(const struct device *dev, uint16_t addr)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	struct i2c_xec_cm_xfr *xfr = &data->cm_xfr;
	struct i2c_msg *m = data->msgs;
	mem_addr_t rb = devcfg->base;
	int rc = 0;
	uint8_t target_addr = 0;
	uint8_t ctrl = XEC_I2C_CR_START_ENI;
	uint8_t start = XEC_I2C_START_NORM;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x10);

	target_addr = i2c_xec_fmt_addr(addr, 0);
	data->wraddr = target_addr;

	if ((data->msgs[0].flags & I2C_MSG_READ) != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x11);
		target_addr |= BIT(0);
		xfr->mdir = XEC_I2C_DIR_RD;
	} else {
		xfr->mdir = XEC_I2C_DIR_WR;
	}

	data->mdone = 0;
	xfr->mbuf = m->buf;
	xfr->mlen = m->len;
	xfr->xfr_sts = 0;
	xfr->target_addr = target_addr;
	xfr->mflags = I2C_XEC_XFR_FLAG_START_REQ;

	if ((sys_read8(rb + XEC_I2C_SR_OFS) & BIT(XEC_I2C_SR_NBB_POS)) == 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x12);
		if ((data->cm_dir != xfr->mdir) || (m->flags & I2C_MSG_RESTART)) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x13);
			start = XEC_I2C_START_RPT;
			ctrl = XEC_I2C_CR_RPT_START_ENI;
		}
	}

	data->cm_dir = xfr->mdir;
	if (m->flags & I2C_MSG_STOP) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x14);
		xfr->mflags |= I2C_XEC_XFR_FLAG_STOP_REQ;
	}

	soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 0);
	soc_ecia_girq_status_clear(devcfg->girq, devcfg->girq_pos);

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x15);

	/* Generate (RPT)-START and transmit address for write or read */
	if (ctrl == XEC_I2C_CR_START_ENI) { /* START? */
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x16);
		sys_write8(target_addr, rb + XEC_I2C_DATA_OFS);
		xec_i2c_nl_cr_write(dev, ctrl);
	} else { /* RPT-START */
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x17);
		xec_i2c_nl_cr_write(dev, ctrl);
		sys_write8(target_addr, rb + XEC_I2C_DATA_OFS);
	}

	soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 1u);
	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x18);

#ifdef XEC_I2C_DEBUG_USE_SPIN_LOOP
	while (!data->mdone) {
		;
	}
#else
	rc = k_sem_take(&data->sync_sem, K_MSEC(100));
	if (rc != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x19);
		return -ETIMEDOUT;
	}
#endif
	if (xfr->xfr_sts) { /* error */
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x1A);
		return -EIO;
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x1B);

	return 0;
}
#endif

#if 0
/* Configure struct dma_config based on flags:
 * bit[0] = 0 (Host mode), 1 (Target mode)
 * bit[1] = 0(memory to peripheral), 1(peripheral to memory)
 * bit[2] = 0(callback at list done), 1(callback on each block)
 * bit[3] = 0(enable error callback), 1(disable error callback)
 */
static int xec_i2c_nl_dma_cfg(const struct device *dev, uint32_t blk_cnt, uint32_t flags)
#endif

static int xec_i2c_nl_xfr_begin(const struct device *dev, uint16_t addr)
{
	const struct xec_i2c_nl_config *drvcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	mem_addr_t rb = drvcfg->base;
	uint32_t max_buf_len = drvcfg->xfrbuf_size - 4u;
	uint8_t *xbuf = (uint8_t *)drvcfg->xfrbuf;
	uint32_t hcmd = 0, extlen = 0, d = 0, dblkcnt = 0, dcfg_flags = 0, rdcnt = 0, wrcnt = 0;
	int rc = 0, frc = 0;
	uint8_t target_addr = 0, idx = 0;
	bool started = false;
	bool dma_done = false, dma_err = false;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x10);

	target_addr = i2c_xec_fmt_addr(addr, 0);
	data->wraddr = target_addr;

	data->remptr = NULL;
	data->remlen = 0;

	data->cm_dir = XEC_I2C_DIR_NONE;

	for (uint8_t n = 0; n < data->num_msgs; n++) {
		struct i2c_msg *m = &data->msgs[n];

		sys_write32(0, rb + XEC_I2C_HCMD_OFS);
		sys_write32(0, rb + XEC_I2C_ELEN_OFS);
		sys_write32(XEC_I2C_CMPL_RW1C_MSK, rb + XEC_I2C_CMPL_OFS);

		idx = 0;
		rdcnt = 0;
		wrcnt = 0;
		if (started == false) {
			xbuf[0] = (uint8_t)target_addr;
			wrcnt = 1u;
			idx = 1u;
		}

		if ((m->flags & I2C_MSG_READ) == 0) { /* write msg? */
			data->cm_dir = XEC_I2C_DIR_WR;

			if (m->len <= max_buf_len) {
				dblkcnt = 1u;
				wrcnt += m->len;

				memcpy(&xbuf[idx], m->buf, m->len);

				xec_i2c_nl_dma_blk_cfg(dev, &data->dma_blk_cfg[0], NULL,
						       (uint32_t)drvcfg->xfrbuf, wrcnt, 0);
			} else {
				dblkcnt = 2u;
				wrcnt += m->len;
				d = m->len - max_buf_len;

				memcpy(&xbuf[idx], m->buf, d);
				idx += d;

				xec_i2c_nl_dma_blk_cfg(dev, &data->dma_blk_cfg[1], NULL,
						       (uint32_t)(m->buf + d), d, 0);
				xec_i2c_nl_dma_blk_cfg(dev, &data->dma_blk_cfg[0],
						       &data->dma_blk_cfg[1],
						       (uint32_t)drvcfg->xfrbuf, idx, 0);
			}
		} else { /* read msg */
			data->cm_dir = XEC_I2C_DIR_RD;
			dblkcnt = 1u;
			dcfg_flags = BIT(1);
			/* TODO this requires transmit of target address (1 byte)
			 * then HW will pause generating an HDONE interrupt and clear
			 * PROC bit only. FW must reconfigure/start DMA and set PROC = 1
			 */
			xec_i2c_nl_dma_blk_cfg(dev, &data->dma_blk_cfg[0], NULL,
			                       (uint32_t)m->buf, m->len, BIT(1));
		}

		dma_done = false;
		dma_err = false;

		rc = xec_i2c_nl_dma_cfg(dev, dblkcnt, dcfg_flags);
		if (rc != 0) {
			LOG_ERR("DMA config error (%d)", rc);
			return rc;
		}

		dma_config(drvcfg->hc_dma_dev, drvcfg->hc_dma_chan, &data->dma_cfg);
		dma_start(drvcfg->hc_dma_dev, drvcfg->hc_dma_chan);

		extlen = (wrcnt & 0xff00u) >> 8;
		extlen |= (rdcnt & 0xff00u);

		hcmd = (wrcnt & 0xff) << 16;
		hcmd |= (rdcnt & 0xff) << 24;

		if (started == false) {
			started = true;
			hcmd |= BIT(XEC_I2C_HCMD_START0_POS);
		}

		if ((m->flags & I2C_MSG_RESTART) != 0) {
			hcmd |= BIT(XEC_I2C_HCMD_STARTN_POS);
		}

		if ((m->flags & I2C_MSG_STOP) != 0) {
			hcmd |= BIT(XEC_I2C_HCMD_STOP_POS);
		}

		hcmd |= (BIT(XEC_I2C_HCMD_PROC_POS) | BIT(XEC_I2C_HCMD_RUN_POS));

		sys_write32(extlen, rb + XEC_I2C_ELEN_OFS);
		sys_write32(hcmd, rb + XEC_I2C_HCMD_OFS);

		while (1) {
			d = sys_read32(rb + XEC_I2C_CMPL_OFS);
			dma_done = atomic_test_and_clear_bit(data->aflags,
			                                     XEC_I2C_NL_AF_DMA_DONE_POS);
			dma_err = atomic_test_and_clear_bit(data->aflags,
			                                    XEC_I2C_NL_AF_DMA_ERR_POS);
			if (dma_err == true) {
				return -EIO;
			}

			if ((d & BIT(XEC_I2C_CMPL_HDONE_POS)) != 0) {
				break;
			}
		}

		LOG_INF("I2C msg[%u] done", n);
		LOG_INF("  I2C.CMPL = 0x%0x", d);
		d = sys_read32(rb + XEC_I2C_HCMD_OFS);
		LOG_INF("  I2C.HCMD = 0x%0x", d);
		LOG_INF("  DMA done = %d  DMA err = %d", dma_done, dma_err);
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x1B);

	return frc;
}

/* called from transfer routine */
static void xec_i2c_nl_kworker(struct k_work *work)
{
	struct xec_i2c_nl_data *data = CONTAINER_OF(work, struct xec_i2c_nl_data, kwq);
	const struct device *dev = data->dev;
	const struct xec_i2c_nl_config *drvcfg = dev->config;

	/* TODO */
}

/* i2c_transfer API - Synchronous using interrupts
 * The call wrapper in i2c.h returns if num_msgs is 0.
 * It does not check for msgs being a NULL pointer and accesses msgs.
 * NOTE 1: Zephyr I2C documentation states an I2C driver can be switched
 * between Host and Target modes by registering and unregistering targets.
 * NOTE 2:
 * XEC I2C controller supports up to 5 target addresses:
 * Two address match registers.
 * I2C general call (address 0). UNTESTED!
 * Two SMBus fixed addresses defined in the SMBus spec. UNTESTED!
 * We many need to remove general call and the two SMBus fixed address code logic!
 */
static int xec_i2c_nl_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			       uint16_t addr)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	mem_addr_t rb = devcfg->base;
	int rc = 0;

	k_mutex_lock(&data->lock_mut, K_FOREVER);
#ifdef CONFIG_I2C_TARGET
	if ((data->i2c_config & I2C_MODE_CONTROLLER) == 0)
		k_mutex_unlock(&data->lock_mut);
		return -EBUSY;
	}
#endif
	pm_device_busy_set(dev);
	k_sem_reset(&data->sync_sem);

	XEC_I2C_DEBUG_ISR_INIT();

	rc = check_msgs(msgs, num_msgs);
	if (rc != 0) {
		goto xec_unlock;
	}

	if (data->state != XEC_I2C_STATE_OPEN) {
		XEC_I2C_DEBUG_STATE_INIT(data);

		rc = check_lines(dev);
		data->i2c_sr = sys_read8(rb + XEC_I2C_SR_OFS);
		data->i2c_compl = sys_read32(rb + XEC_I2C_CMPL_OFS);

		if (rc || (data->i2c_sr & BIT(XEC_I2C_SR_BER_POS))) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x50);
			rc = xec_i2c_nl_recover_bus(dev);
		}
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x1);

	if (rc) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x2);
		data->state = XEC_I2C_STATE_CLOSED;
		goto xec_unlock;
	}

	data->state = XEC_I2C_STATE_OPEN;

	data->msg_idx = 0;
	data->num_msgs = num_msgs;
	data->msgs = msgs;

	rc = xec_i2c_nl_xfr_begin(dev, addr);
	if (rc) { /* if error issue STOP if bus is still owned by controller */
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x7);
		/* TODO stop HW */
	}

xec_unlock:
	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x8);

	if ((sys_read8(rb + XEC_I2C_SR_OFS) & BIT(XEC_I2C_SR_NBB_POS)) == 0) {
		data->cm_dir = XEC_I2C_DIR_NONE;
		data->state = XEC_I2C_STATE_CLOSED;
	}

	pm_device_busy_clear(dev);
	k_mutex_unlock(&data->lock_mut);

	return rc;
}

/* ISR helpers and state handlers */
static void xec_i2c_nl_isr(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;

	/* clears I2C controller's GIRQ enable causing GIRQ result
	 * signal to clear. GIRQ result is the input to the NVIC.
	 */
	soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 0);
}

#ifdef CONFIG_PM_DEVICE
/* TODO Add logic to enable I2C wake if target mode is active.
 * For deep sleep this requires enabling GIRQ22 wake clocks feature.
 */
#if 0
#ifdef CONFIG_I2C_TARGET
	if ((data->i2c_config & I2C_MODE_CONTROLLER) == 0)
		k_mutex_unlock(&data->lock_mut);
		return -EBUSY;
	}
#endif
#endif
static int xec_i2c_nl_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
#ifdef CONFIG_I2C_TARGET
	struct xec_i2c_nl_data *const data = dev->data;
#endif
	mem_addr_t rb = devcfg->base;

	LOG_DBG("PM action: %d", (int)action);

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
#ifdef CONFIG_I2C_TARGET
		if ((data->i2c_config & I2C_MODE_CONTROLLER) == 0) {
			sys_clear_bit(rb + XEC_I2C_WKSR_OFS, XEC_I2C_WKSR_SB_POS);
			sys_set_bit(rb + XEC_I2C_WKCR_OFS, XEC_I2C_WKCR_SBEN_POS);
			soc_ecia_girq_status_clear(devcfg->girq_wk, devcfg->girq_wk_pos);
			soc_ecia_girq_ctrl(devcfg->girq_wk, devcfg->girq_wk_pos, 1);
		}
#endif
		/* TODO will this cause I2C Wake to not work?
		 * If we keep the enable set does I2C keep its CLK_REQ at 1?
		 */
		sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);
		break;
	case PM_DEVICE_ACTION_RESUME:
		sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif

int xec_i2c_nl_debug_init(const struct device *dev)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	struct xec_i2c_nl_data *data = dev->data;

	XEC_I2C_DEBUG_STATE_INIT(data);
	XEC_I2C_DEBUG_ISR_INIT();

	return 0;
}

#ifdef CONFIG_I2C_RTIO
static int xec_i2c_nl_iodev_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	return -ENOTSUP; /* TODO */
}
#endif

#ifdef CONFIG_I2C_TARGET

static int xec_i2c_nl_target_decode(const struct device *dev, uint16_t i2c_addr, uint8_t en)
{
	const struct xec_i2c_nl_config *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->base;
	uint32_t msk = 0x7fu, own_addr = 0;
	int rc = -ENXIO;

	if (i2c_addr == XEC_I2C_GEN_CALL_ADDR) {
		if (en != 0) {
			sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_GC_DIS_POS);
		} else {
			sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_GC_DIS_POS);
		}
		rc = 0;
	} else if ((i2c_addr == XEC_I2C_SMB_HOST_ADDR) ||
	           (i2c_addr == XEC_I2C_SMB_DEVICE_ADDR)) {
		if (en != 0) {
			sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_DSA_POS);
		} else {
			sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_DSA_POS);
		}
		rc = 0;
	} else {
		own_addr = sys_read32(rb + XEC_I2C_OA_OFS);

		for (i = 0; i < 2; i++) {
			if (en != 0) {
				if ((own_addr & msk) == 0) {
					own_addr |= (i2c_addr & msk);
					rc = 0;
					break;
				}
			} else {
				if ((own_addr & msk) == (i2c_addr & msk)) {
					own_addr &= ~msk;
					rc = 0;
					break;
				}
			}

			msk <<= 8u;
			i2c_addr <<= 8u;
		}

		sys_write32(own_addr, rb + XEC_I2C_OA_OFS);
	}

	return rc;
}

static int xec_i2c_nl_target_register(const struct device *dev, struct i2c_target_config *tcfg)
{
	const struct xec_i2c_nl_config *drvcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	int rc = -ENXIO;

	if ((tcfg == NULL) || (tcfg->addr > 0x7fu)) {
		return -EINVAL;
	}

	if (k_mutex_lock(&data->lock_mut, K_MSEC(50)) != 0) {
		return -EBUSY;
	}

	for (uint8_t n = 0; n < XEC_I2C_NL_MAX_TARGETS; n++) {
		if (data->target_cfgs[n] == NULL) {
			rc = xec_i2c_nl_target_decode(dev, tcfg->addr, 1);
			if (rc == 0) {
				data->target_cfgs[n] = tcfg;
				data->ntargets++;
				data->i2c_config &= ~(I2C_MODE_CONTROLLER);
			}
			break;
		}
	}

	k_mutex_unlock(&data->lock_mut);

	return rc;
}

static int xec_i2c_nl_target_unregister(const struct device *dev, struct i2c_target_config *tcfg)
{
	const struct xec_i2c_nl_config *drvcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	int rc = -ENXIO;

	if ((tcfg == NULL) || (tcfg->addr > 0x7fu)) {
		return -EINVAL;
	}

	if (k_mutex_lock(&data->lock_mut, K_MSEC(50)) != 0) {
		return -EBUSY;
	}

	for (uint8_t n = 0; n < XEC_I2C_NL_MAX_TARGETS; n++) {
		if (data->target_cfgs[n] == tcfg) {
			data->target_cfgs[n] = NULL;
			if (data->ntargets != 0) {
				data->ntargets--;
			}
			rc = xec_i2c_nl_target_decode(dev, tcfg->addr, 0);
			break;
		}
	}

	if (data->ntargets == 0) {
		data->i2c_config |= I2C_MODE_CONTROLLER;
	}

	k_mutex_unlock(&data->lock_mut);

	return rc;
}
#endif

static int xec_i2c_nl_init(const struct device *dev)
{
	const struct xec_i2c_nl_config *cfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	int rc = 0;
	uint32_t i2c_config = 0;

	xec_i2c_nl_debug_init(dev);

	data->dev = dev;
	data->state = XEC_I2C_STATE_CLOSED;
	data->i2c_compl = 0;
	data->i2c_cr_shadow = 0;
	data->i2c_sr = 0;
	data->mdone = 0;

	rc = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("pinctrl setup failed (%d)", rc);
		return rc;
	}

	i2c_config = i2c_map_dt_bitrate(cfg->clock_freq);
	if (i2c_config == 0) {
		return -EINVAL;
	}

	i2c_config |= I2C_MODE_CONTROLLER;
#ifdef CONFIG_I2C_XEC_PORT_MUX
	i2c_config |= I2C_XEC_PORT_SET((uint32_t)cfg->port);
#endif
	/* Default configuration */
	rc = xec_i2c_nl_configure(dev, i2c_config);
	if (rc != 0) {
		return rc;
	}

	k_work_init(&data->kwq, &xec_i2c_nl_kworker);
	k_mutex_init(&data->lock_mut);
	k_sem_init(&data->sync_sem, 0, 1);

	if (cfg->irq_config_func) {
		cfg->irq_config_func();
	}

	return 0;
}

static DEVICE_API(i2c, xec_i2c_nl_driver_api) = {
	.configure = xec_i2c_nl_configure,
	.get_config = xec_i2c_nl_get_config,
	.transfer = xec_i2c_nl_transfer,
#ifdef CONFIG_I2C_TARGET
	.target_register = xec_i2c_nl_target_register,
	.target_unregister = xec_i2c_nl_target_unregister,
#else
	.target_register = NULL,
	.target_unregister = NULL,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit  xec_i2c_nl_iodev_submit,
#endif
	.recover_bus = xec_i2c_nl_recover_bus,
};

#define XEC_I2C_NL_GIRQ_DT(inst, idx) \
	MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, idx))

#define XEC_I2C_NL_GIRQ_POS_DT(inst, idx) \
	MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, idx))

#define XEC_I2C_NL_XFR_BUF_SIZE(inst) \
	(DT_INST_PROP_OR(inst, xfr_buffer_size, 32) + XEC_I2C_NL_XFR_BUF_PAD_SZ)

#define XEC_I2C_NL_XFR_BUF(inst) \
	static uint8_t xec_i2c_nl_xfrbuf##inst[XEC_I2C_NL_XFR_BUF_SIZE(inst)] __aligned(4)

#define XEC_I2C_NL_DMA_NODE(i, name)    DT_INST_DMAS_CTLR_BY_NAME(i, name)
#define XEC_I2C_NL_DMA_DEVICE(i, name)  DEVICE_DT_GET(XEC_I2C_NL_DMA_NODE(i, name))
#define XEC_I2C_NL_DMA_CHAN(i, name)    DT_INST_DMAS_CELL_BY_NAME(i, name, channel)
#define XEC_I2C_NL_DMA_TRIGSRC(i, name) DT_INST_DMAS_CELL_BY_NAME(i, name, trigsrc)

#ifdef CONFIG_I2C_TARGET
#define XEC_I2C_NL_TM_DMA_INFO(inst)                                                               \
	.tc_dma_dev = XEC_I2C_NL_DMA_DEVICE(inst, target_mode),                                    \
	.tc_dma_chan = XEC_I2C_NL_DMA_CHAN(inst, target_mode),                                     \
	.tc_dma_trigsrc = XEC_I2C_NL_DMA_TRIGSRC(inst, target_mode),
#else
#define XEC_I2C_NL_TM_DMA_INFO(inst)
#endif

#define XEC_I2C_NL_DEVICE(i)                                                                       \
	PINCTRL_DT_INST_DEFINE(i);                                                                 \
	static void xec_i2c_nl_irq_config_func_##i(void)                                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(i), DT_INST_IRQ(i, priority), xec_i2c_nl_isr,             \
			    DEVICE_DT_INST_GET(i), 0);                                             \
		irq_enable(DT_INST_IRQN(i));                                                       \
	}                                                                                          \
	XEC_I2C_NL_XFR_BUF(i);                                                                     \
	static struct xec_i2c_nl_data xec_i2c_nl_data##i = {.port_sel = DT_INST_PROP(i, port_sel)};\
	static const struct xec_i2c_nl_config xec_i2c_nl_config##i = {                             \
		.base = (mem_addr_t)DT_INST_REG_ADDR(i),                                           \
		.clock_freq = DT_INST_PROP(i, clock_frequency),                                    \
		.sda_gpio = GPIO_DT_SPEC_INST_GET(i, sda_gpios),                                   \
		.scl_gpio = GPIO_DT_SPEC_INST_GET(i, scl_gpios),                                   \
		.irq_config_func = xec_i2c_nl_irq_config_func_##i,                                 \
		.hc_dma_dev = XEC_I2C_NL_DMA_DEVICE(i, host_mode),                                 \
		.xfrbuf = xec_i2c_nl_xfrbuf##i,                                                    \
		.xfrbuf_size = XEC_I2C_NL_XFR_BUF_SIZE(i),                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),                                         \
		.girq = XEC_I2C_NL_GIRQ_DT(i, 0),                                                  \
		.girq_pos = XEC_I2C_NL_GIRQ_POS_DT(i, 0),                                          \
		.girq_wk = XEC_I2C_NL_GIRQ_DT(i, 1),                                               \
		.girq_wk_pos = XEC_I2C_NL_GIRQ_POS_DT(i, 1),                                       \
		.pcr = (uint8_t)DT_INST_PROP(i, pcr),                                              \
		.port = (uint8_t)DT_INST_PROP(i, port_sel),                                        \
		.hc_dma_chan = XEC_I2C_NL_DMA_CHAN(i, host_mode),                                  \
		.hc_dma_trigsrc = XEC_I2C_NL_DMA_TRIGSRC(i, host_mode),                            \
		XEC_I2C_NL_TM_DMA_INFO(inst)                                                       \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(i, xec_i2c_nl_pm_action);                                         \
	I2C_DEVICE_DT_INST_DEFINE(i, xec_i2c_nl_init, PM_DEVICE_DT_INST_GET(i),                    \
				  &xec_i2c_nl_data##i, &xec_i2c_nl_config##i,                      \
				  POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,                           \
				  &xec_i2c_nl_driver_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_DEVICE)
