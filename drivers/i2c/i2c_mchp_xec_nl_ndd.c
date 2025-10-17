/*
 * Copyright 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_i2c_nl_ndd

#include <soc.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_xec_i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/atomic.h>
#include "zephyr/sys/sys_io.h"
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_mchp_xec_nl_ndd);

/* unused #include "i2c-priv.h" */

#define XEC_I2C_NL_DEBUG_ISR
#define XEC_I2C_DEBUG_STATE
#define XEC_I2C_DEBUG_STATE_ENTRIES 256
#define XEC_I2C_DEBUG_CLR_XFRBUF_ON_TRANSFER
#define XEC_I2C_DEBUG_INTR
#define XEC_I2C_DEBUG_KWORKQ

#define XEC_DMAC_BASE (mem_addr_t)(DT_REG_ADDR(DT_NODELABEL(dmac)))

#define XEC_I2C_NL_TARGET_ADDR_NONE 0xf800u

#define XEC_I2C_NL_TX_BUF_SZ       16u
#define XEC_I2C_NL_TX_BUF_PAD_SZ   4u
#define XEC_I2C_NL_TX_BUF_TOTAL_SZ (XEC_I2C_NL_TX_BUF_SZ + XEC_I2C_NL_TX_BUF_PAD_SZ)

#define XEC_I2C_NL_XFRBUF_PAD_SZ 8

#define XEC_I2C_NL_CFG_TM BIT(0)

#define I2C_XEC_CR_PIN_ESO_ACK                                                                     \
	(BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS))

#define I2C_XEC_CR_ESO_NOACK BIT(XEC_I2C_CR_ESO_POS)

#define I2C_XEC_BB_EN_SCL_DRIVE_LO                                                                 \
	(BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CD_POS) | BIT(XEC_I2C_BBCR_CM_POS))
#define I2C_XEC_BB_EN_SCL_TRI_STATE (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CM_POS))

#define I2C_XEC_BB_EN_SDA_DRIVE_LO                                                                 \
	(BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_DD_POS) | BIT(XEC_I2C_BBCR_CM_POS))
#define I2C_XEC_BB_EN_SDA_TRI_STATE (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CM_POS))

#define I2C_XEC_BB_SCL_SDA_HIGH_MSK (BIT(XEC_I2C_BBCR_SCL_IN_POS) | BIT(XEC_I2C_BBCR_SDA_IN_POS))

#define XEC_I2C_NL_DMA_BLOCK_MAX 3

#define XEC_I2C_NL_DMA_CFG_HC       BIT(0)
#define XEC_I2C_NL_DMA_CFG_TC       0
#define XEC_I2C_NL_DMA_CFG_M2D      BIT(1)
#define XEC_I2C_NL_DMA_CFG_D2M      0
#define XEC_I2C_NL_DMA_CFG_NO_INCRM BIT(2)

#ifdef CONFIG_I2C_TARGET
BUILD_ASSERT(CONFIG_I2C_TARGET_BUFFER_MODE,
	     "MCHP I2C-NL-NDD target support requires I2C_TARGET_BUFFER_MODE");
#endif

enum xec_i2c_nl_dir {
	XEC_I2C_NL_DIR_DEV_TO_MEM = 0,
	XEC_I2C_NL_DIR_MEM_TO_DEV,
};

enum xec_i2c_nl_cm_state {
	XEC_CM_STATE_CLOSED = 0,
	XEC_CM_STATE_DATA_TX,
	XEC_CM_STATE_DATA_RX,
};

enum xec_i2c_nl_protocol {
	XEC_I2C_NL_PROTO_WR = 0,
	XEC_I2C_NL_PROTO_RD,
	XEC_I2C_NL_PROTO_WR_WR,
	XEC_I2C_NL_PROTO_WR_RD,
	XEC_I2C_NL_PROTO_MAX,
};

struct xec_i2c_nl_msg_group {
	struct i2c_msg *m1; /* starting message in group */
	struct i2c_msg *m2;
	uint32_t flags;
};

struct xec_i2c_nl_xfr {
	uint32_t txb_addr;
	uint32_t rxb_addr;
	uint16_t ntx;
	uint16_t nrx;
	uint16_t i2c_addr;
	uint16_t flags;
};

struct xec_i2c_nl_config {
	mem_addr_t i2c_base;
	uint32_t bitrate;
	const struct pinctrl_dev_config *pin_cfg;
	void (*irq_config)(void);
	uint8_t hm_dma_chan;
	uint8_t hm_dma_trigsrc;
	uint8_t tm_dma_chan;
	uint8_t tm_dma_trigsrc;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t girq_wake;
	uint8_t girq_wake_pos;
	uint8_t pcr;
	uint8_t port;
	volatile uint8_t *xfrbuf;
	uint32_t xfrbuf_sz;
};

#define XEC_AF_HM_ASYNC_POS    0
#define XEC_AF_IDLE_POS        1
#define XEC_AF_HM_DMA_DONE_POS 4
#define XEC_AF_HM_PAUSE_POS    5
#define XEC_AF_HM_ALL_DONE_POS 6
#define XEC_AF_HM_UNKNOWN_POS  7
#define XEC_AF_TM_DMA_DONE_POS 8
#define XEC_AF_TM_PAUSE_POS    9
#define XEC_AF_TM_ALL_DONE_POS 10
#define XEC_AF_TM_UNKNOWN_POS  11

#define XEC_I2C_NL_XFR_BER_POS  0
#define XEC_I2C_NL_XFR_LAB_POS  1
#define XEC_I2C_NL_XFR_HNAK_POS 2

struct xec_i2c_nl_data {
	const struct device *dev;
	struct k_mutex lock_mut;
	struct k_sem sync_sem;
#ifdef CONFIG_I2C_XEC_NL_USE_KWORKQUEUE
	struct k_work kwq;
#endif
	uint32_t discard;
	uint32_t i2c_config;
	uint32_t xfr_status;
	volatile uint32_t i2c_status;
	volatile uint32_t i2c_cfg_reg;
	volatile uint32_t hcmd;
	volatile uint32_t tcmd;
	struct soc_xec_dma_chan_cfg dcfgs[4]; /* 4 * 16 = 64 bytes */
	struct i2c_msg *msgs;
	struct xec_i2c_nl_msg_group mg;
	ATOMIC_DEFINE(aflags, 32);
	uint32_t htcmd;
	uint32_t xfrbuf_len;
	uint16_t i2c_addr;
	uint8_t i2c_cr;
	uint8_t dcfgs_cnt;
	uint8_t dcfgs_idx;
	uint8_t num_msgs;
	uint8_t midx;
	uint8_t hflags;
	uint16_t wrcnt;
	uint16_t rdcnt;
#ifdef CONFIG_I2C_CALLBACK
	i2c_callback_t cb;
	void *userdata;
#endif
#ifdef CONFIG_I2C_TARGET
	struct soc_xec_dma_chan_cfg tdcfg;
	struct i2c_target_config *target_cfg_oa1;
	struct i2c_target_config *target_cfg_oa2;
#ifdef CONFIG_I2C_XEC_NL_TARGET_GENERAL_CALL
	struct i2c_target_config *target_cfg_gen_call;
#endif
#ifdef CONFIG_I2C_XEC_NL_TARGET_SMBUS_HOST_DEV_ADDRESSES
	struct i2c_target_config *target_cfg_smb_host;
	struct i2c_target_config *target_cfg_smb_dev;
#endif
	uint8_t *buf_rd_req_ptr;
	uint32_t buf_rd_req_len;
	uint8_t target_bitmap;
#endif /* CONFIG_I2C_TARGET */
#ifdef XEC_I2C_NL_DEBUG_ISR
	volatile uint32_t isr_count;
#endif
#ifdef XEC_I2C_DEBUG_STATE
	volatile uint32_t nl_fsm_done;
	volatile uint32_t nl_fsm_pause;
	volatile uint32_t dbg_state_idx;
	uint8_t dbg_states[XEC_I2C_DEBUG_STATE_ENTRIES];
#endif
};

struct xec_i2c_nl_timing {
	uint32_t freq_hz;
	uint32_t data_timing;
	uint32_t idle_scaling;
	uint32_t timeout_scaling;
	uint16_t bus_clock;
	uint8_t rpt_sta_hold_tm;
};

static const struct xec_i2c_nl_timing xec_i2c_nl_timing_tbl[] = {
	{KHZ(100), XEC_I2C_SMB_DATA_TM_100K, XEC_I2C_SMB_IDLE_SC_100K, XEC_I2C_SMB_TMO_SC_100K,
	 XEC_I2C_SMB_BUS_CLK_100K, XEC_I2C_SMB_RSHT_100K},
	{KHZ(400), XEC_I2C_SMB_DATA_TM_400K, XEC_I2C_SMB_IDLE_SC_400K, XEC_I2C_SMB_TMO_SC_400K,
	 XEC_I2C_SMB_BUS_CLK_400K, XEC_I2C_SMB_RSHT_400K},
	{MHZ(1), XEC_I2C_SMB_DATA_TM_1M, XEC_I2C_SMB_IDLE_SC_1M, XEC_I2C_SMB_TMO_SC_1M,
	 XEC_I2C_SMB_BUS_CLK_1M, XEC_I2C_SMB_RSHT_1M},
};

#ifdef XEC_I2C_NL_DEBUG_ISR
static void xec_i2c_nl_dbg_isr_init(struct xec_i2c_nl_data *data)
{
	data->isr_count = 0;
	data->hcmd = 0;
	data->tcmd = 0;
}
#define XEC_I2C_DEBUG_ISR_INIT(pd) xec_i2c_nl_dbg_isr_init(pd)
#else
#define XEC_I2C_DEBUG_ISR_INIT(pd)
#endif

#ifdef XEC_I2C_DEBUG_STATE
static void xec_i2c_nl_dbg_state_init(struct xec_i2c_nl_data *data)
{
	data->nl_fsm_done = 0;
	data->nl_fsm_pause = 0;
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
#endif /* XEC_I2C_DEBUG_STATE */

static void xec_i2c_cr_write(const struct device *dev, uint8_t ctrl_val)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;

	data->i2c_cr = ctrl_val;
	sys_write8(ctrl_val, devcfg->i2c_base + XEC_I2C_CR_OFS);
}

static int xec_i2c_nl_prog_standard_timing(const struct device *dev, uint32_t freq_hz)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t i2c_base = devcfg->i2c_base;

	for (size_t n = 0; n < ARRAY_SIZE(xec_i2c_nl_timing_tbl); n++) {
		const struct xec_i2c_nl_timing *p = &xec_i2c_nl_timing_tbl[n];

		if (freq_hz == p->freq_hz) {
			sys_write32(p->data_timing, i2c_base + XEC_I2C_DT_OFS);
			sys_write32(p->idle_scaling, i2c_base + XEC_I2C_ISC_OFS);
			sys_write32(p->timeout_scaling, i2c_base + XEC_I2C_TMOUT_SC_OFS);
			sys_write16(p->bus_clock, i2c_base + XEC_I2C_BCLK_OFS);
			sys_write8(p->rpt_sta_hold_tm, i2c_base + XEC_I2C_RSHT_OFS);

			return 0;
		}
	}

	return -EINVAL;
}

static uint32_t xec_i2c_status_get(mem_addr_t i2c_base)
{
	uint32_t v = sys_read32(i2c_base + XEC_I2C_CMPL_OFS);

	v &= ~GENMASK(7, 0);
	v |= sys_read8(i2c_base + XEC_I2C_SR_OFS);

	return v;
}

static bool xec_i2c_nl_bus_is_idle(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t rb = devcfg->i2c_base;

	if (soc_test_bit8(rb + XEC_I2C_SR_OFS, XEC_I2C_SR_NBB_POS) != 0) {
		return true;
	}

	return false;
}

/* After I2C-NL controller reset and configuration we need to
 * configure DMA channel used by controller host mode.
 */
static void xec_i2c_nl_dma_clear(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;

	soc_xec_dmac_chan_clear(devcfg->hm_dma_chan);
}

static int xec_i2c_nl_hw_cfg(const struct device *dev, uint32_t bitrate, uint8_t port,
			     uint32_t flags)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint32_t v = 0;
	int rc = 0;
	uint8_t temp8 = 0;

	soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 0);

	xec_i2c_nl_dma_clear(dev);
	soc_xec_pcr_sleep_en_clear(devcfg->pcr);
	soc_xec_pcr_reset_en(devcfg->pcr);

	soc_ecia_girq_status_clear(devcfg->girq, devcfg->girq_pos);

	/* Clear service (PIN) */
	xec_i2c_cr_write(dev, BIT(XEC_I2C_CR_PIN_POS));

	rc = xec_i2c_nl_prog_standard_timing(dev, bitrate);
	if (rc != 0) {
		return rc;
	}

	/* Clear service, enable output, and enable ACK generation */
	temp8 = BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS);
	xec_i2c_cr_write(dev, temp8);

	/* port mux and filter enable */
	v = XEC_I2C_CFG_PORT_SET((uint32_t)port);
	v |= BIT(XEC_I2C_CFG_FEN_POS);
	v |= BIT(XEC_I2C_CFG_GC_DIS_POS);
	sys_write32(v, i2c_base + XEC_I2C_CFG_OFS);

	/* enable bit-bang continuous pin monitor. v3.8 HW only */
	sys_set_bit(i2c_base + XEC_I2C_BBCR_OFS, XEC_I2C_BBCR_CM_POS);

	/* enable: controller begins sampling SCL/SDA pins over time inverval ?
	 * Should we delay after enable?
	 */
	sys_set_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);

	return 0;
}

static uint32_t i2c_bitrate_from_config(uint32_t i2c_config)
{
	uint32_t bitrate = 0;

	switch (I2C_SPEED_GET(i2c_config)) {
	case I2C_SPEED_STANDARD:
		bitrate = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		bitrate = KHZ(400);
		break;
	case I2C_SPEED_FAST_PLUS:
		bitrate = MHZ(1);
		break;
	default:
		break;
	}

	return bitrate;
}

/* Public API */
static int xec_i2c_nl_configure(const struct device *dev, uint32_t i2c_config)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	int rc = 0;
	uint32_t bitrate = 0, cfg_flags = 0;
	uint8_t port = 0;

	bitrate = i2c_bitrate_from_config(i2c_config);
	if (bitrate == 0) { /* unsupported I2C speed? */
		return -EINVAL;
	}

	rc = k_mutex_lock(&data->lock_mut, K_NO_WAIT);
	if (rc != 0) {
		return rc;
	}

	port = devcfg->port;
#ifdef CONFIG_I2C_XEC_PORT_MUX
	port = I2C_XEC_PORT_GET(i2c_config);
#endif

	rc = xec_i2c_nl_hw_cfg(dev, bitrate, port, cfg_flags);
	if (rc == 0) {
		data->i2c_config = i2c_config;
	}

	k_mutex_unlock(&data->lock_mut);

	return rc;
}

static int xec_i2c_nl_get_config(const struct device *dev, uint32_t *i2c_config)
{
	struct xec_i2c_nl_data *const data = dev->data;
	uint32_t cfg = 0;

	if (i2c_config == NULL) {
		return -EINVAL;
	}

	cfg = data->i2c_config;
	*i2c_config = cfg;

	return 0;
}

/* How can we detect if this controller is driving the bus or an external
 * device is driving SCL and/or SDA low?
 * 1. Reset this controller
 * 2. Enable bit-bang in tri-state mode.
 * 3. If line go high then this controller is no longer driving the line(s)
 *    else its an external device
 * 4. External device driving line: do recovery sequence
 *    a. SDA stuck low
 *       Drive SDA high using bit-bang HW
 *       n = 0;
 *       while (n < 10) {
 *              Read SDA
 *              if SDA == 0 {
 *                  generate clock pulse on SCL (1-0-1 transition)
 *              } else {
 *                  sda_state = 1
 *                  break
 *              }
 *       }
 *       if sda_state == 1
 *          Generate STOP using bit-bang HW
 *          report success
 *
 *    OR toggle SCL (1-0-1) 20 times and then generate a STOP
 *
 *    OR issue 10 I2C clocks
 *       >5us delay, SDA low, >5us delay, SCL low (START)
 *       >5us delay, SCL high, >5us delay, SDA high (STOP)
 *
 */
static int xec_i2c_nl_recover_bus(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint8_t port = XEC_I2C_CFG_PORT_GET(sys_read32(i2c_base + XEC_I2C_CFG_OFS));
	uint8_t bbcr = 0;

	xec_i2c_nl_hw_cfg(dev, KHZ(100), port, 0); /* reset and config controller */

	k_sleep(K_MSEC(35));

	/* enable bit-bang SCL and SDA tri-state mode. SCL and SDA are disconnected from I2C
	 * controller logic and connected to bit-bang logic. We can read the pins states via
	 * bit-bang control registers. If the either pin is low then an external device is
	 * driving the pin(s).
	 */
	bbcr = (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_SCL_POS) | BIT(XEC_I2C_BBCR_SDA_POS) |
		BIT(XEC_I2C_BBCR_CM_POS));
	sys_write8(bbcr, i2c_base + XEC_I2C_BBCR_OFS);

	bbcr = sys_read8(i2c_base + XEC_I2C_BBCR_OFS);

	if ((bbcr & BIT(XEC_I2C_BBCR_SCL_IN_POS)) == 0) {
		/* SCL is being held low by somthing. Nothing we can do */
		sys_write8(BIT(XEC_I2C_BBCR_CM_POS), i2c_base + XEC_I2C_BBCR_OFS);
		return -EIO;
	}

	if ((bbcr & BIT(XEC_I2C_BBCR_SDA_IN_POS)) != 0) {
		/* SDA and SCL are both high. We are ok. */
		sys_write8(BIT(XEC_I2C_BBCR_CM_POS), i2c_base + XEC_I2C_BBCR_OFS);
		return 0;
	}

	/* generate 10 I2C clocks at 100 KHz */
	for (int i = 0; i < 10; i++) {
		bbcr = I2C_XEC_BB_EN_SCL_DRIVE_LO | I2C_XEC_BB_EN_SDA_TRI_STATE;
		sys_write8(bbcr, i2c_base + XEC_I2C_BBCR_OFS); /* drive SCL low */
		k_busy_wait(5);
		bbcr = I2C_XEC_BB_EN_SCL_TRI_STATE | I2C_XEC_BB_EN_SDA_TRI_STATE;
		sys_write8(bbcr, i2c_base + XEC_I2C_BBCR_OFS); /* release SCL */
		k_busy_wait(5);
	}

	/* generate an I2C START */
	k_busy_wait(5);
	/* drive SDA low */
	bbcr = I2C_XEC_BB_EN_SCL_TRI_STATE | I2C_XEC_BB_EN_SDA_DRIVE_LO;
	sys_write8(bbcr, i2c_base + XEC_I2C_BBCR_OFS);
	k_busy_wait(5);
	/* drive SCL low */
	bbcr = I2C_XEC_BB_EN_SCL_DRIVE_LO | I2C_XEC_BB_EN_SDA_DRIVE_LO;
	sys_write8(bbcr, i2c_base + XEC_I2C_BBCR_OFS);
	k_busy_wait(5);

	/* generate an I2C STOP */
	k_busy_wait(5);
	/* release SCL */
	bbcr = I2C_XEC_BB_EN_SCL_TRI_STATE | I2C_XEC_BB_EN_SDA_DRIVE_LO;
	sys_write8(bbcr, i2c_base + XEC_I2C_BBCR_OFS);
	k_busy_wait(5);
	/* release SCL */
	bbcr = I2C_XEC_BB_EN_SCL_TRI_STATE | I2C_XEC_BB_EN_SDA_TRI_STATE;
	sys_write8(bbcr, i2c_base + XEC_I2C_BBCR_OFS);
	k_busy_wait(5);

	bbcr = sys_read8(i2c_base + XEC_I2C_BBCR_OFS);

	/* turn off bit-bang mode. I2C SCL and SDA are re-connect to I2C controller logic */
	sys_write8(BIT(XEC_I2C_BBCR_CM_POS), i2c_base + XEC_I2C_BBCR_OFS);

	if ((bbcr & I2C_XEC_BB_SCL_SDA_HIGH_MSK) != I2C_XEC_BB_SCL_SDA_HIGH_MSK) {
		return -EIO;
	}

	k_sleep(K_MSEC(35));

	return 0;
}

/* Check messages passed by app are valid for our I2C controller.
 * MEC I2C supports 7-bit I2C addressing only.
 * I2C-NL network layer read/write counts are 16-bit and we further limit this to 0xFFF8 bytes.
 * I2C-NL uses DMA to move the data pattern to/from I2C-NL data buffer registers.
 * I2C-NL expects the memory buffer to contain the I2C START and RPT-START addresses along
 * with optional PEC bytes. We limit size to max HW count 0xffff - 8 bytes to be safe.
 */
static int xec_i2c_nl_msgs_valid(struct i2c_msg *msgs, uint8_t num_msgs)
{
	if ((msgs == NULL) || (num_msgs == 0)) {
		return -EINVAL;
	}

	for (uint8_t n = 0; n < num_msgs; n++) {
		struct i2c_msg *m = &msgs[n];

		LOG_DBG("msg[%u] buf=%p len=%u flags=0x%02x", n, (void *)m->buf, m->len, m->flags);

		if ((m->buf == NULL) || (m->len == 0)) {
			return -EINVAL;
		}

		if ((m->flags & I2C_MSG_ADDR_10_BITS) != 0) {
			return -ENOTSUP;
		}

		if (m->len > XEC_I2C_NL_MAX_LEN) {
			return -E2BIG;
		}
	}

	return 0;
}

#define XEC_I2C_NL_HM_CFG_FLUSH BIT(XEC_I2C_CFG_FHTX_POS) | BIT(XEC_I2C_CFG_FHRX_POS)
#define XEC_I2C_NL_HM_CFG_IEN   BIT(XEC_I2C_CFG_HD_IEN_POS)
#define XEC_I2C_NL_TM_CFG_FLUSH BIT(XEC_I2C_CFG_FTTX_POS) | BIT(XEC_I2C_CFG_FTRX_POS)
#define XEC_I2C_NL_TM_CFG_IEN   BIT(XEC_I2C_CFG_TD_IEN_POS) | BIT(XEC_I2C_CFG_STD_NL_IEN_POS)

#define XEC_I2C_NL_HM 0
#define XEC_I2C_NL_TM 1u

static void xec_i2c_nl_clean(const struct device *dev, uint8_t tm)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t rb = devcfg->i2c_base;
	uint32_t cfg_ien = XEC_I2C_NL_HM_CFG_IEN;
	uint32_t cfg_flush = XEC_I2C_NL_HM_CFG_FLUSH;
	uint32_t cmd_ofs = XEC_I2C_HCMD_OFS;

	if (tm != 0) {
		cfg_ien = XEC_I2C_NL_TM_CFG_IEN;
		cfg_flush = XEC_I2C_NL_TM_CFG_FLUSH;
		cmd_ofs = XEC_I2C_TCMD_OFS;
	}

	soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 0);
	sys_write32(0, rb + cmd_ofs);
	sys_write32(0, rb + XEC_I2C_ELEN_OFS);
	sys_clear_bits(rb + XEC_I2C_CFG_OFS, cfg_ien);
	sys_set_bits(rb + XEC_I2C_CFG_OFS, cfg_flush);
	sys_write32(1u, rb + XEC_I2C_WKSR_OFS);
	sys_write32(XEC_I2C_CMPL_RW1C_MSK, rb + XEC_I2C_CMPL_OFS);
	soc_ecia_girq_status_clear(devcfg->girq, devcfg->girq_pos);
}

/* Return a pointer to the next DMA configuration but do not increment the
 * index. If the caller uses the DMA config it is required to call incr_dma_cfg_idx
 * to increment the index.
 */
static struct soc_xec_dma_chan_cfg *next_dma_cfg(const struct device *i2c_dev)
{
	struct xec_i2c_nl_data *data = i2c_dev->data;
	uint16_t didx = data->dcfgs_idx + 1u;

	if (didx < (uint16_t)data->dcfgs_cnt) {
		return &data->dcfgs[didx];
	}

	return NULL;
}

static void incr_dma_cfg_idx(const struct device *i2c_dev)
{
	struct xec_i2c_nl_data *data = i2c_dev->data;
	uint16_t didx = data->dcfgs_idx + 1u;

	if (didx < (uint16_t)data->dcfgs_cnt) {
		data->dcfgs_idx = (uint8_t)didx;
	}
}

/* Trigger I2C-NL hardware to begin the configured transfer.
 * I2C-NL HW FSM begins the transfer when its Host command register run and proceed bits are
 * both written to 1. Transfer configuration requires:
 * I2C.HCMD wrCountLSB != 0 since every transaction requires transmitting a START address.
 * rdCountLSB != 0 if there is a read phase.
 * START0 bit = 1 indicating HW should generate START (required!)
 * STARTN bit = 1 if the protocol is Write, RPT-START, Read
 * STOP = 1 This driver does not support leaving a transaction open.
 * Bits[15:8] of the 16-bit write and read counts are located in fields of the EXTLEN register
 * which should be programmed before writing HCMD.
 * Don't forget to configure and start the DMA channel before starting I2C-NL.
 */
static int xec_i2c_nl_xfr_trigger(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	mem_addr_t rb = devcfg->i2c_base;
	uint32_t dma_start_flags = XEC_DMAC_START_IEN;
	uint32_t cmd = 0, extlen = 0, cfg = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x40);

	xec_i2c_nl_clean(dev, XEC_I2C_NL_HM);

	cmd = data->hflags;
	cmd <<= XEC_I2C_HCMD_START0_POS;
	cmd |= BIT(XEC_I2C_HCMD_RUN_POS) | BIT(XEC_I2C_HCMD_PROC_POS);
	cmd |= XEC_I2C_HCMD_WCL_SET((uint32_t)data->wrcnt);
	cmd |= XEC_I2C_HCMD_RCL_SET((uint32_t)data->rdcnt);
	data->htcmd = cmd;
	/* read count MSB in bits[15:8], write count MSB in bits[7:0] */
	extlen = (data->rdcnt & 0xff00u) + ((data->wrcnt >> 8) & 0xffu);
	sys_write32(extlen, rb + XEC_I2C_ELEN_OFS);

	soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 1u);

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x41);

	soc_xec_dmac_chan_cfg2(devcfg->hm_dma_chan, &data->dcfgs[data->dcfgs_idx], NULL);
	soc_xec_dmac_chan_start(devcfg->hm_dma_chan, dma_start_flags);

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x42);
#ifdef XEC_I2C_DEBUG_INTR
	*(volatile uint32_t *)0x40081080 = 0x10240u;
	*(volatile uint32_t *)0x40081098 = 0x10240u;

#endif
	cfg = sys_read32(rb + XEC_I2C_CFG_OFS);
	cfg |= BIT(XEC_I2C_CFG_HD_IEN_POS);
	sys_write32(cfg, rb + XEC_I2C_CFG_OFS);
	sys_write32(cmd, rb + XEC_I2C_HCMD_OFS); /* triggers HW */

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x4F);

	return 0;
}

static bool is_msg(struct i2c_msg *m, uint8_t flag_msk)
{
	if ((m->flags & flag_msk) == flag_msk) {
		return true;
	}

	return false;
}

static bool i2c_nl_cnt_add(uint32_t a, uint32_t b, uint32_t *c)
{
	if ((a >= 0xfffcu) || (b >= 0xfffcu)) {
		return false;
	}

	if ((a + b) > 0xfffcu) {
		return false;
	}

	if (c != NULL) {
		*c = a + b;
	}

	return true;
}

#define XEC_DMA_CHAN_CFG_DEV2MEM 0
#define XEC_DMA_CHAN_CFG_MEM2DEV BIT(0)
#define XEC_DMA_CHAN_CFG_HM      0
#define XEC_DMA_CHAN_CFG_TM      BIT(1)

static void xec_dma_chan_cfg(const struct device *dev, struct soc_xec_dma_chan_cfg *dcfg,
			     uint32_t maddr, uint32_t nbytes, uint8_t flags)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t i2c_base = devcfg->i2c_base;
#ifdef XEC_I2C_DEBUG_STATE
	struct xec_i2c_nl_data *data = dev->data;
#endif
	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x30);

	dcfg->maddr = maddr;
	dcfg->nbytes = nbytes;
	dcfg->flags = (XEC_DMAC_CHAN_CFG_HFC | XEC_DMAC_CHAN_CFG_INCRM |
		       XEC_DMAC_CHAN_CFG_UNITS_SET(XEC_DMAC_CHAN_CFG_UNITS_1));

	if ((flags & XEC_DMA_CHAN_CFG_MEM2DEV) != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x31);
		dcfg->flags |= XEC_DMAC_CHAN_CFG_MEM2DEV;
	} else {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x32);
		dcfg->flags |= XEC_DMAC_CHAN_CFG_DEV2MEM;
	}

	if ((flags & XEC_DMA_CHAN_CFG_TM) == 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x33);
		dcfg->flags |= XEC_DMAC_CHAN_CFG_HDEVID_SET((uint32_t)devcfg->hm_dma_trigsrc);
		if ((flags & XEC_DMA_CHAN_CFG_MEM2DEV) != 0) {
			dcfg->daddr = (uint32_t)(i2c_base + XEC_I2C_HTX_OFS);
		} else {
			dcfg->daddr = (uint32_t)(i2c_base + XEC_I2C_HRX_OFS);
		}
	} else {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x34);
		dcfg->flags |= XEC_DMAC_CHAN_CFG_HDEVID_SET((uint32_t)devcfg->tm_dma_trigsrc);
		if ((flags & XEC_DMA_CHAN_CFG_MEM2DEV) != 0) {
			dcfg->daddr = (uint32_t)(i2c_base + XEC_I2C_TTX_OFS);
		} else {
			dcfg->daddr = (uint32_t)(i2c_base + XEC_I2C_TRX_OFS);
		}
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x35);
}

static int xec_i2c_nl_cfg_mgrp_xfr(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	struct xec_i2c_nl_msg_group *g = &data->mg;
	struct soc_xec_dma_chan_cfg *dcfgs = data->dcfgs;
	uint8_t *xfrbuf = (uint8_t *)devcfg->xfrbuf;
	struct i2c_msg *m = NULL;
	uint32_t wrcnt = 0, rdcnt = 0;
	uint8_t didx = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x20);

#ifdef XEC_I2C_DEBUG_CLR_XFRBUF_ON_TRANSFER
	memset((void *)xfrbuf, 0x55u, devcfg->xfrbuf_sz);
#endif

	data->hflags = BIT(0) | BIT(2);
	data->dcfgs_cnt = 0;
	data->dcfgs_idx = 0;

	wrcnt = 1u;
	data->xfrbuf_len = 1u;
	xfrbuf[0] = (uint8_t)((data->i2c_addr & 0x7Fu) << 1);

	/* START and transmit 7-bit target address plus nW/R bit */
	xec_dma_chan_cfg(dev, &dcfgs[didx], (uint32_t)xfrbuf, 1u,
			 (XEC_DMA_CHAN_CFG_MEM2DEV | XEC_DMA_CHAN_CFG_HM));

	m = g->m1;
	if ((g->flags & 0xfu) == 1u) { /* first msg is write? */
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x21);
		wrcnt += m->len;
		if (m->len < (devcfg->xfrbuf_sz - XEC_I2C_NL_XFRBUF_PAD_SZ)) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x22);
			memcpy((void *)&xfrbuf[1], m->buf, m->len);
			data->xfrbuf_len += m->len;
			dcfgs[didx].nbytes += m->len;
		} else {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x23);
			didx++;
			xec_dma_chan_cfg(dev, &dcfgs[didx], (uint32_t)m->buf, m->len,
					 (XEC_DMA_CHAN_CFG_MEM2DEV | XEC_DMA_CHAN_CFG_HM));
		}
	} else { /* first is read */
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x24);
		xfrbuf[0] |= BIT(0);
		rdcnt += m->len;
		didx++;
		xec_dma_chan_cfg(dev, &dcfgs[didx], (uint32_t)m->buf, m->len,
				 (XEC_DMA_CHAN_CFG_DEV2MEM | XEC_DMA_CHAN_CFG_HM));
	}

	m = g->m2; /* does group have a second msg? */
	if (m != NULL) {
		if ((g->flags & 0xf0u) == 0x10u) { /* write? */
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x25);
			wrcnt += m->len;
			if ((g->flags & 0xf) == 1u) { /* first msg is write? */
				/* does xfrbuf have space to merge msgs? */
				XEC_I2C_DEBUG_STATE_UPDATE(data, 0x26);
				if (m->len <= (devcfg->xfrbuf_sz - XEC_I2C_NL_XFRBUF_PAD_SZ) -
						      data->xfrbuf_len) {
					XEC_I2C_DEBUG_STATE_UPDATE(data, 0x27);
					memcpy((void *)&xfrbuf[data->xfrbuf_len], m->buf, m->len);
					data->xfrbuf_len += m->len;
					dcfgs[didx].nbytes += m->len;
				} else {
					XEC_I2C_DEBUG_STATE_UPDATE(data, 0x28);
					didx++;
					xec_dma_chan_cfg(
						dev, &dcfgs[didx], (uint32_t)m->buf, m->len,
						(XEC_DMA_CHAN_CFG_MEM2DEV | XEC_DMA_CHAN_CFG_HM));
				}
			}
		} else { /* second msg is read */
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x29);
			rdcnt += m->len;
			if ((g->flags & 0xf) == 1u) { /* first msg is write? */
				XEC_I2C_DEBUG_STATE_UPDATE(data, 0x2A);
				/* we need RPT-START rdAddr */
				data->hflags |= BIT(1);
				xfrbuf[data->xfrbuf_len] = xfrbuf[0] + BIT(0);
				if (data->xfrbuf_len > 1u) {
					XEC_I2C_DEBUG_STATE_UPDATE(data, 0x2B);
					wrcnt++;
					dcfgs[0].nbytes++;
				} else {
					XEC_I2C_DEBUG_STATE_UPDATE(data, 0x2C);
					++didx;
					xec_dma_chan_cfg(
						dev, &dcfgs[didx],
						(uint32_t)&xfrbuf[data->xfrbuf_len], 1u,
						(XEC_DMA_CHAN_CFG_MEM2DEV | XEC_DMA_CHAN_CFG_HM));
				}
			}

			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x2D);

			++didx;
			xec_dma_chan_cfg(dev, &dcfgs[didx], (uint32_t)m->buf, m->len,
					 (XEC_DMA_CHAN_CFG_DEV2MEM | XEC_DMA_CHAN_CFG_HM));
		}
	}

	data->dcfgs_cnt = ++didx;

	if ((wrcnt > 0xffffu) || (rdcnt > 0xffffu)) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x2E);
		return -E2BIG;
	}

	data->wrcnt = wrcnt & 0xffffu;
	data->rdcnt = rdcnt & 0xffffu;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x2F);

	return 0;
}

/* A message group can up to two messages.
 * One message with I2C_MSG_STOP
 * Two messages where the second has I2C_MSG_STOP
 * If the app passes more than two messages without I2C_MSG_STOP then we force a STOP on
 * the second message and begin a new group on the third.
 * Allowed groups of two messages due to HW limitations.
 * Write-Write
 * Read-Read
 * Write-Read: HW supports START write RPT-START read STOP
 * Read-Write illegal. HW doesn't support START read RPT-START write STOP
 * NOTE: these may not be consecutive messages if a message has NULL pointer or zero length.
 * These types of messages should have been caught by the message check performed in the
 * transfer API.
 */
static int build_msg_group(const struct device *dev)
{
	struct xec_i2c_nl_data *const data = dev->data;
	struct xec_i2c_nl_msg_group *g = &data->mg;
	int rc = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x10);

	g->m1 = NULL;
	g->m2 = NULL;
	g->flags = 0;

	while (data->midx < data->num_msgs) {
		struct i2c_msg *m = &data->msgs[data->midx];

		if ((m->buf == NULL) || (m->len == 0) || (m->len > 0xfffcu)) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x18);
			data->midx++;
			continue;
		}

		if (g->m1 == NULL) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x11);
			g->m1 = m;
			if (is_msg(m, I2C_MSG_READ) == true) {
				XEC_I2C_DEBUG_STATE_UPDATE(data, 0x12);
				g->flags |= 0x02u;
			} else {
				XEC_I2C_DEBUG_STATE_UPDATE(data, 0x13);
				g->flags |= 0x01u;
			}
			if (is_msg(m, I2C_MSG_STOP) == true) {
				XEC_I2C_DEBUG_STATE_UPDATE(data, 0x14);
				data->midx++;
				break;
			}
		} else if (g->m2 == NULL) {
			if (is_msg(m, I2C_MSG_READ) == true) {
				XEC_I2C_DEBUG_STATE_UPDATE(data, 0x15);
				g->m2 = m;
				g->flags |= 0x20u;
			} else {
				XEC_I2C_DEBUG_STATE_UPDATE(data, 0x16);
				if (is_msg(g->m1, I2C_MSG_READ) == true) {
					/* HW can't do Read-Write */
					XEC_I2C_DEBUG_STATE_UPDATE(data, 0x17);
					break;
				}
				g->m2 = m;
				g->flags |= 0x10u;
				data->midx++;
				break;
			}
		}
		data->midx++;
	}

	if ((g->m1 == NULL) && (g->m2 == NULL)) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x19);
		return -ENOMSG;
	}

	/* if both are same direction check if total length overflows HW */
	if ((g->flags & 0x0fu) == ((g->flags >> 4) & 0x0fu)) {
		if (i2c_nl_cnt_add(g->m1->len, g->m2->len, NULL) == false) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x1A);
			rc = -E2BIG;
		}
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x1F);

	return rc;
}

#ifdef CONFIG_I2C_LOG_LEVEL_DBG
static void pr_xfr_cfg(const struct device *dev)
{
	struct xec_i2c_nl_data *const data = dev->data;

	LOG_DBG("data.hflags = 0x%02x wrcnt = 0x%0x rdcnt = 0x%0x", data->hflags, data->wrcnt,
		data->rdcnt);
	LOG_DBG("data.dcfgs_cnt = %u  data.dcfgs_idx = %u", data->dcfgs_cnt, data->dcfgs_idx);

	for (uint8_t n = 0; n < data->dcfgs_cnt; n++) {
		struct soc_xec_dma_chan_cfg *d = &data->dcfgs[n];

		LOG_DBG("dcfg[%u] @ 0x%0x", n, (uint32_t)d);
		LOG_DBG("0x%0x 0x%0x 0x%0x 0x%0x", d->daddr, d->maddr, d->nbytes, d->flags);
	}
}

static void pr_mgrp(struct xec_i2c_nl_msg_group *g)
{
	if (g == NULL) {
		LOG_ERR("Group is NULL");
	}

	LOG_DBG("grp.m1 = %p grp.m2 = %p grp.flags=0x%0x", (void *)g->m1, (void *)g->m2, g->flags);
	if (g->m1 != NULL) {
		LOG_DBG("m1.buf = %p m1.len = %u m1.flags = 0x%02x", (void *)g->m1->buf, g->m1->len,
			g->m1->flags);
	}
	if (g->m2 != NULL) {
		LOG_DBG("m2.buf = %p m2.len = %u m2.flags = 0x%02x", (void *)g->m2->buf, g->m2->len,
			g->m1->flags);
	}
}

#define XEC_I2C_NL_DEBUG_PR_XFR_CFG(dev) pr_xfr_cfg(dev)
#define XEC_I2C_NL_DEBUG_PR_MGRP(g) pr_mgrp(g)
#else
#define XEC_I2C_NL_DEBUG_PR_XFR_CFG(dev)
#define XEC_I2C_NL_DEBUG_PR_MGRP(g)
#endif /* CONFIG_I2C_LOG_LEVEL_DBG */

static int xec_i2c_nl_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			       uint16_t addr)
{
	struct xec_i2c_nl_data *const data = dev->data;
	int rc = 0;

	/* check array of struct i2c_msg for NULL ptrs and lengths */
	rc = xec_i2c_nl_msgs_valid(msgs, num_msgs);
	if (rc != 0) {
		LOG_ERR("Found invalid I2C message");
		return rc;
	}

	k_mutex_lock(&data->lock_mut, K_FOREVER);

#ifdef CONFIG_I2C_TARGET
	if (data->target_bitmap != 0) {
		LOG_ERR("Target mode enabled!");
		return -EBUSY;
	}
#endif
	XEC_I2C_DEBUG_ISR_INIT(data);
	XEC_I2C_DEBUG_STATE_INIT(data);
	XEC_I2C_DEBUG_STATE_UPDATE(data, 1);

	if (addr > XEC_I2C_TARGET_ADDR_MSK) {
		k_mutex_unlock(&data->lock_mut);
		return -EINVAL; /* hardware only supports 7-bit I2C addresses */
	}

	pm_device_busy_set(dev);

	XEC_I2C_DEBUG_STATE_UPDATE(data, 2);

	if (xec_i2c_nl_bus_is_idle(dev) == false) {
		LOG_DBG("I2C-NL xfr bus is not idle");
		pm_device_busy_clear(dev);
		k_mutex_unlock(&data->lock_mut);
		return -EIO;
	}

	data->xfr_status = 0;
	data->i2c_addr = addr;
	data->msgs = msgs;
	data->num_msgs = num_msgs;
	data->midx = 0;
	data->wrcnt = 0;
	data->rdcnt = 0;
	data->htcmd = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 3);

	while (data->midx < data->num_msgs) {
		rc = build_msg_group(dev);
		if (rc != 0) {
			continue;
		}

		XEC_I2C_NL_DEBUG_PR_MGRP(&data->mg);

		k_sem_reset(&data->sync_sem);

		XEC_I2C_DEBUG_STATE_UPDATE(data, 8);

		xec_i2c_nl_cfg_mgrp_xfr(dev);

		XEC_I2C_NL_DEBUG_PR_XFR_CFG(dev);

		xec_i2c_nl_xfr_trigger(dev);

		k_sem_take(&data->sync_sem, K_FOREVER);

		XEC_I2C_DEBUG_STATE_UPDATE(data, 9);

		if (data->xfr_status != 0) {
			rc = -EIO;
			break;
		}
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xA);
	pm_device_busy_clear(dev);
	k_mutex_unlock(&data->lock_mut);

	return rc;
}

#ifdef CONFIG_I2C_CALLBACK

static int xec_i2c_nl_async_iter(const struct device *dev)
{
	struct xec_i2c_nl_data *const data = dev->data;
	int rc = 0;

	/* TODO iterate over messages, will be called from ISR or kworkq handler
	 * If no more messages or error in build_msg_group we must clear XEC_AF_ASYNC_POS
	 */
	if (data->midx < data->num_msgs) {
		rc = build_msg_group(dev);
		if (rc != 0) {
			atomic_clear_bit(data->aflags, XEC_AF_HM_ASYNC_POS);
			return rc;
		}

		xec_i2c_nl_cfg_mgrp_xfr(dev);

		xec_i2c_nl_xfr_trigger(dev);
	}

	return rc;
}

/* Wrapper in i2c.h checks
 * 1. cb is NULL and returns -ENOSYS
 * 2. num_msgs == 0. If 0 invokes callback and returns 0
 * 3. if CONFIG_I2C_ALLOW_NO_STOP_TRANSACTIONS is not enabled force I2C_MSG_STOP
 *    on last message.
 */
static int xec_i2c_nl_transfer_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				  uint16_t addr, i2c_callback_t cb, void *userdata)
{
	struct xec_i2c_nl_data *const data = dev->data;
	int rc = 0;

#ifdef CONFIG_I2C_TARGET
	if (data->target_bitmap != 0) {
		LOG_ERR("Target mode enabled!");
		return -EBUSY;
	}
#endif

	if (addr > XEC_I2C_TARGET_ADDR_MSK) {
		LOG_ERR("HW only supports 7-bit I2C addresses");
		return -EINVAL; /* hardware only supports 7-bit I2C addresses */
	}

	rc = xec_i2c_nl_msgs_valid(msgs, num_msgs);
	if (rc != 0) {
		LOG_ERR("Found invalid I2C message");
		return rc;
	}

	rc = k_mutex_lock(&data->lock_mut, K_NO_WAIT);
	if (rc != 0) {
		return -EWOULDBLOCK;
	}

	pm_device_busy_set(dev);

	k_sem_reset(&data->sync_sem);

	data->i2c_addr = addr;
	data->msgs = msgs;
	data->num_msgs = num_msgs;
	data->midx = 0;
	data->wrcnt = 0;
	data->rdcnt = 0;
	data->htcmd = 0;

	data->cb = cb;
	data->userdata = userdata;

	atomic_set_bit(data->aflags, XEC_AF_HM_ASYNC_POS);

	return xec_i2c_nl_async_iter(dev);
}

#define XEC_I2C_NL_ASYNC_ITER(dev) xec_i2c_nl_async_iter(dev)
#else
#define XEC_I2C_NL_ASYNC_ITER(dev)
#endif /* CONFIG_I2C_CALLBACK */

#ifdef CONFIG_I2C_TARGET

static struct i2c_target_config *find_target_config(const struct device *dev)
{
	struct xec_i2c_nl_data *data = dev->data;

	if ((data->target_cfg_oa1 != NULL) && (data->target_cfg_oa1->address == data->i2c_addr)) {
		return data->target_cfg_oa1;
	}

	if ((data->target_cfg_oa2 != NULL) && (data->target_cfg_oa2->address == data->i2c_addr)) {
		return data->target_cfg_oa2;
	}

#ifdef CONFIG_I2C_XEC_NL_TARGET_GENERAL_CALL
	if ((data->target_cfg_gen_call != NULL) &&
	    (data->target_cfg_gen_call->address == data->i2c_addr)) {
		return data->target_cfg_gen_call;
	}
#endif
#ifdef CONFIG_I2C_XEC_NL_TARGET_SMBUS_HOST_DEV_ADDRESSES
	if ((data->target_cfg_smb_host != NULL) &&
	    (data->target_cfg_smb_host->address == data->i2c_addr)) {
		return data->target_cfg_smb_host;
	}

	if ((data->target_cfg_smb_dev != NULL) &&
	    (data->target_cfg_smb_dev->address == data->i2c_addr)) {
		return data->target_cfg_smb_dev;
	}
#endif
	return NULL;
}
#endif /* CONFIG_I2C_TARGET */

#ifdef CONFIG_I2C_XEC_NL_USE_KWORKQUEUE

static void hm_dma_done(const struct device *i2c_dev)
{
	const struct xec_i2c_nl_config *devcfg = i2c_dev->config;
	struct xec_i2c_nl_data *data = i2c_dev->data;
	struct soc_xec_dma_chan_cfg *dcfg = NULL;
	mem_addr_t dma_base = 0;
	uint32_t dma_ctrl = 0;
	bool curr_m2d = false;
	bool next_m2d = false;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x98);

	dcfg = next_dma_cfg(i2c_dev);
	if (dcfg != NULL) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x99);

		dma_base = soc_xec_dmac_chan_base(devcfg->hm_dma_chan);
		dma_ctrl = sys_read32(dma_base + XEC_DMA_CHAN_CR_OFS);

		if ((dma_ctrl & BIT(XEC_DMA_CHAN_CR_M2D_POS)) != 0) {
			curr_m2d = true;
		}

		if ((dcfg->flags & XEC_DMAC_CHAN_CFG_MEM2DEV) != 0) {
			next_m2d = true;
		}

		if (curr_m2d == next_m2d) { /* same direction? */
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x9A);
			incr_dma_cfg_idx(i2c_dev); /* indicate we processed this DMA config */
			soc_xec_dmac_chan_cfg2(devcfg->hm_dma_chan, dcfg, NULL);
			soc_xec_dmac_chan_start(devcfg->hm_dma_chan, XEC_DMAC_START_IEN);
		}
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x9B);
}

/* I2C-NL HW FSM Pause state:
 * When I2C-NL HW decrements write count ot 0 and if read count is non-zero
 * it will clock stretch, clear I2C.HCMD.PROC bit, and set HM_DONE status.
 * DMA must be reconfigured for read phase (dev2mem) and we must set I2C.HCMD.PROC bit to 1.
 * These two action have been split across hm_dma_done (DMA reconfig) and hm_done.
 * The driver requires DMA channel interrupt to be higher priority so its done event
 * is serviced before HM pause.
 */
static void hm_pause(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	mem_addr_t i2c_base = devcfg->i2c_base;
	struct soc_xec_dma_chan_cfg *dcfg = NULL;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x94);

#ifdef XEC_I2C_DEBUG_STATE
	data->nl_fsm_pause = sys_read32(i2c_base + XEC_I2C_NFSM_OFS);
#endif

	dcfg = next_dma_cfg(dev);
	if ((dcfg != NULL) && ((dcfg->flags & XEC_DMAC_CHAN_CFG_MEM2DEV) == 0)) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x95);

		incr_dma_cfg_idx(dev); /* indicate we processed this DMA config */

		soc_xec_dmac_chan_cfg2(devcfg->hm_dma_chan, dcfg, NULL);
		soc_xec_dmac_chan_start(devcfg->hm_dma_chan, XEC_DMAC_START_IEN);

		sys_set_bit(i2c_base + XEC_I2C_HCMD_OFS, XEC_I2C_HCMD_PROC_POS);
		sys_set_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_HD_IEN_POS);

		soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 1u);
	} else {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x96);
		/* I2C-NL HW-FSM cannot be forced to generate STOP except by
		 * letting it decrement its write and read counts to 0.
		 * Since pause state is write to read direction change we reconfigure
		 * a two byte read from target to discard location. We must set read
		 * count to two because I2C hardware always performs read ahead.
		 */

		/* Configure DMA to driver discard buffer */
		dcfg = &data->dcfgs[0];
		dcfg->daddr = (uint32_t)(i2c_base + XEC_I2C_HRX_OFS);
		dcfg->maddr = (uint32_t)&data->discard;
		dcfg->nbytes = 2u;
		dcfg->flags = (XEC_DMAC_CHAN_CFG_DEV2MEM | XEC_DMAC_CHAN_CFG_HFC |
			       XEC_DMAC_CHAN_CFG_INCRM | XEC_DMAC_CHAN_UNIT_1B |
			       XEC_DMAC_CHAN_CFG_HDEVID_SET(devcfg->hm_dma_trigsrc));

		soc_xec_dmac_chan_cfg2(devcfg->hm_dma_chan, dcfg, NULL);
		soc_xec_dmac_chan_start(devcfg->hm_dma_chan, 0);

		/* rdCntMSB=0, wrCntMSB=0 */
		sys_write32(0, i2c_base + XEC_I2C_ELEN_OFS);
		/* flush host mode buffers */
		sys_set_bits(i2c_base + XEC_I2C_CFG_OFS,
			     (BIT(XEC_I2C_CFG_FHRX_POS) | BIT(XEC_I2C_CFG_FHTX_POS)));

		sys_set_bit(i2c_base + XEC_I2C_CMPL_OFS, XEC_I2C_CMPL_HDONE_POS);
		sys_set_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_HD_IEN_POS);
		sys_write32(0x2000403, i2c_base + XEC_I2C_HCMD_OFS);
		soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 1u);
	}

	XEC_I2C_DEBUG_STATE_UPDATE((struct xec_i2c_nl_data *)dev->data, 0x97);
}

static void hm_all_done(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	mem_addr_t i2c_base = devcfg->i2c_base;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x90);

#ifdef XEC_I2C_DEBUG_STATE
	data->nl_fsm_done = sys_read32(i2c_base + XEC_I2C_NFSM_OFS);
	data->nl_fsm_pause = 0;
#endif

	sys_set_bits(i2c_base + XEC_I2C_CFG_OFS,
		     (BIT(XEC_I2C_CFG_FHRX_POS) | BIT(XEC_I2C_CFG_FHTX_POS)));

	if (data->i2c_status & BIT(XEC_I2C_CMPL_BER_STS_POS)) {
		data->xfr_status |= BIT(XEC_I2C_NL_XFR_BER_POS);
	}

	if (data->i2c_status & BIT(XEC_I2C_CMPL_LAB_STS_POS)) {
		data->xfr_status |= BIT(XEC_I2C_NL_XFR_LAB_POS);
	}

	if (data->i2c_status & BIT(XEC_I2C_CMPL_HNAKX_POS)) {
		data->xfr_status |= BIT(XEC_I2C_NL_XFR_HNAK_POS);
	}

	if (data->xfr_status == 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x91);
		/* if asynchronouse we build next message group and start transaction */
		if (IS_ENABLED(CONFIG_I2C_CALLBACK) &&
		    (atomic_test_bit(data->aflags, XEC_AF_HM_ASYNC_POS) == true)) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x92);
			XEC_I2C_NL_ASYNC_ITER(dev); /* It can fail */
		} else {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x93);
			soc_xec_dmac_chan_deactivate(devcfg->hm_dma_chan);
			k_sem_give(&data->sync_sem);
		}
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x95);
}

static void hm_unknown(const struct device *dev)
{
	struct xec_i2c_nl_data *data = dev->data;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x9E);

	LOG_ERR("Unkown Host Mode Event: Status=0x%0x HCmd=0x%0x", data->i2c_status, data->hcmd);
}

#ifdef CONFIG_I2C_TARGET
static void tm_dma_done(const struct device *dev)
{
	XEC_I2C_DEBUG_STATE_UPDATE((struct xec_i2c_nl_data *)dev->data, 0xC0);
}

static void tm_pause(const struct device *dev)
{
	XEC_I2C_DEBUG_STATE_UPDATE((struct xec_i2c_nl_data *)dev->data, 0xB8);
}

static void tm_all_done(const struct device *dev)
{
	XEC_I2C_DEBUG_STATE_UPDATE((struct xec_i2c_nl_data *)dev->data, 0xB0);
}

static void tm_unknown(const struct device *dev)
{
	struct xec_i2c_nl_data *data = dev->data;

	XEC_I2C_DEBUG_STATE_UPDATE((struct xec_i2c_nl_data *)dev->data, 0xBD);

	LOG_ERR("Unkown Target Mode Event: Status=0x%0x HCmd=0x%0x", data->i2c_status, data->tcmd);
}
#endif

static void xec_i2c_nl_kworker(struct k_work *work)
{
	struct xec_i2c_nl_data *data = CONTAINER_OF(work, struct xec_i2c_nl_data, kwq);
	const struct device *dev = data->dev;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x80);

#ifdef CONFIG_I2C_TARGET
	if (atomic_test_and_clear_bit(data->aflags, XEC_AF_TM_DMA_DONE_POS) == true) {
		tm_dma_done(dev);
	}

	if (atomic_test_and_clear_bit(data->aflags, XEC_AF_TM_PAUSE_POS) == true) {
		tm_pause(dev);
	}

	if (atomic_test_and_clear_bit(data->aflags, XEC_AF_TM_ALL_DONE_POS) == true) {
		tm_all_done(dev);
	}

	if (atomic_test_and_clear_bit(data->aflags, XEC_AF_TM_UNKNOWN_POS) == true) {
		tm_unknown(dev);
	}
#endif
	if (atomic_test_and_clear_bit(data->aflags, XEC_AF_HM_DMA_DONE_POS) == true) {
		hm_dma_done(dev);
	}

	if (atomic_test_and_clear_bit(data->aflags, XEC_AF_HM_PAUSE_POS) == true) {
		hm_pause(dev);
	}

	if (atomic_test_and_clear_bit(data->aflags, XEC_AF_HM_ALL_DONE_POS) == true) {
		hm_all_done(dev);
	}

	if (atomic_test_and_clear_bit(data->aflags, XEC_AF_HM_UNKNOWN_POS) == true) {
		hm_unknown(dev);
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x8F);
}
#endif /* CONFIG_I2C_XEC_NL_USE_KWORKQUEUE */

/* I2C-NL interrupt handler */
static void xec_i2c_nl_isr(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t i2c_base = devcfg->i2c_base;
	struct xec_i2c_nl_data *data = dev->data;
	uint32_t i2c_status = 0, hcmd = 0, tcmd = 0;
	uint8_t dbg_state = 0;

#ifdef XEC_I2C_DEBUG_INTR
	*(volatile uint32_t *)0x40081080 = 0x0240u;
#endif
	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x70);

#ifdef XEC_I2C_NL_DEBUG_ISR
	data->isr_count++;
#endif
	data->i2c_status = xec_i2c_status_get(i2c_base);
	data->i2c_cfg_reg = sys_read32(i2c_base + XEC_I2C_CFG_OFS);
	data->hcmd = sys_read32(i2c_base + XEC_I2C_HCMD_OFS);
	data->tcmd = sys_read32(i2c_base + XEC_I2C_TCMD_OFS);

	sys_write32(data->i2c_cfg_reg & 0x00ffffffu, i2c_base + XEC_I2C_CFG_OFS);
	sys_write32(XEC_I2C_CMPL_RW1C_MSK, i2c_base + XEC_I2C_CMPL_OFS);
	soc_ecia_girq_status_clear(devcfg->girq, devcfg->girq_pos);

	hcmd = data->hcmd;
	tcmd = data->tcmd;
	i2c_status = data->i2c_status;

#ifdef CONFIG_I2C_TARGET
	if ((i2c_status & BIT(XEC_I2C_CMPL_TDONE_POS)) != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x72);
		if ((tcmd & 0x03u) == 0x01u) {
			atomic_set_bit(data->aflags, XEC_AF_TM_PAUSE_POS);
		} else if ((tcmd & 0x03u) == 0) {
			atomic_set_bit(data->aflags, XEC_AF_TM_ALL_DONE_POS);
		} else {
			atomic_set_bit(data->aflags, XEC_AF_TM_UNKNOWN_POS);
		}
	}
#endif
	if ((i2c_status & BIT(XEC_I2C_CMPL_HDONE_POS)) != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x71);
		if ((hcmd & 0x03u) == 0x01u) {
			atomic_set_bit(data->aflags, XEC_AF_HM_PAUSE_POS);
		} else if ((hcmd & 0x03u) == 0) {
			atomic_set_bit(data->aflags, XEC_AF_HM_ALL_DONE_POS);
		} else {
			atomic_set_bit(data->aflags, XEC_AF_HM_UNKNOWN_POS);
		}
	}

	/* start work queue thread for this driver. Return values:
	 * 0 work was already submitted to a queue
	 * 1 work was not submitted and has been queue'd
	 * 2 work was running and has been queued to the queue that was running it
	 * -EBUSY rejected because work item is cancelling or queue draining or queue is plugged
	 * -EINVAL kernel work queue is null and item never run
	 * -ENODEV kernel work queue has not been started
	 */
#ifdef XEC_I2C_DEBUG_KWORKQ
	int rc = k_work_submit(&data->kwq);

	switch (rc) {
	case -ENODEV:
		dbg_state = 0xf5;
		break;
	case -EINVAL:
		dbg_state = 0xf4;
		break;
	case -EBUSY:
		dbg_state = 0xf3;
		break;
	case 2:
		dbg_state = 0xf2;
		break;
	case 1:
		dbg_state = 0xf1;
		break;
	case 0:
		dbg_state = 0xf0;
		break;
	default:
		dbg_state = 0xffu;
		break;
	}
	XEC_I2C_DEBUG_STATE_UPDATE(data, dbg_state);
#else
	k_work_submit(&data->kwq);
#endif

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x7F);
#ifdef XEC_I2C_DEBUG_INTR
	*(volatile uint32_t *)0x40081080 = 0x10240u;
#endif
}

static void xec_i2c_nl_hm_dma_isr(const struct device *i2c_dev)
{
	const struct xec_i2c_nl_config *devcfg = i2c_dev->config;
	struct xec_i2c_nl_data *data = i2c_dev->data;
	uint32_t dma_status = 0;
	uint8_t dbg_state = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xD0);

	soc_xec_dmac_status_get(devcfg->hm_dma_chan, &dma_status);
	soc_xec_dmac_status_clear_all(devcfg->hm_dma_chan);

	if ((dma_status & BIT(XEC_DMA_CHAN_IESR_BERR_POS)) != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xD1);
	}

	atomic_set_bit(data->aflags, XEC_AF_HM_DMA_DONE_POS);
	/* k_sem_give(&data->event_sem); */

	/* start work queue thread for this driver. Return values:
	 * 0 work was already submitted to a queue
	 * 1 work was not submitted and has been queue'd
	 * 2 work was running and has been queued to the queue that was running it
	 * -EBUSY rejected because work item is cancelling or queue draining or queue is plugged
	 * -EINVAL kernel work queue is null and item never run
	 * -ENODEV kernel work queue has not been started
	 */
#ifdef XEC_I2C_DEBUG_KWORKQ
	int rc = k_work_submit(&data->kwq);

	switch (rc) {
	case -ENODEV:
		dbg_state = 0xe5;
		break;
	case -EINVAL:
		dbg_state = 0xe4;
		break;
	case -EBUSY:
		dbg_state = 0xe3;
		break;
	case 2:
		dbg_state = 0xe2;
		break;
	case 1:
		dbg_state = 0xe1;
		break;
	case 0:
		dbg_state = 0xe0;
		break;
	default:
		dbg_state = 0xe6u;
		break;
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, dbg_state);
#else
	k_work_submit(&data->kwq);
#endif
	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xD3);
}

#ifdef CONFIG_I2C_TARGET
static void xec_i2c_nl_tm_dma_isr(const struct device *i2c_dev)
{
	const struct xec_i2c_nl_config *devcfg = i2c_dev->config;
	struct xec_i2c_nl_data *data = i2c_dev->data;
	uint32_t dma_status = 0;
	uint8_t dbg_state = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xD8);

	soc_xec_dmac_status_get(devcfg->hm_dma_chan, &dma_status);
	soc_xec_dmac_status_clear_all(devcfg->hm_dma_chan);

	if ((dma_status & BIT(XEC_DMA_CHAN_IESR_BERR_POS)) != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xD9);
	}

	atomic_set_bit(data->aflags, XEC_AF_TM_DMA_DONE_POS);
	/* k_sem_give(&data->event_sem); */

	/* start work queue thread for this driver. Return values:
	 * 0 work was already submitted to a queue
	 * 1 work was not submitted and has been queue'd
	 * 2 work was running and has been queued to the queue that was running it
	 * -EBUSY rejected because work item is cancelling or queue draining or queue is plugged
	 * -EINVAL kernel work queue is null and item never run
	 * -ENODEV kernel work queue has not been started
	 */
#ifdef XEC_I2C_DEBUG_KWORKQ
	int rc = k_work_submit(&data->kwq);

	switch (rc) {
	case -ENODEV:
		dbg_state = 0xed;
		break;
	case -EINVAL:
		dbg_state = 0xec;
		break;
	case -EBUSY:
		dbg_state = 0xeb;
		break;
	case 2:
		dbg_state = 0xea;
		break;
	case 1:
		dbg_state = 0xe9;
		break;
	case 0:
		dbg_state = 0xe8;
		break;
	default:
		dbg_state = 0xeeu;
		break;
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, dbg_state);
#else
	k_work_submit(&data->kwq);
#endif
	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xDB);
}
#endif /* CONFIG_I2C_TARGET */

#ifdef CONFIG_PM_DEVICE
/* TODO Add logic to enable I2C wake if target mode is active.
 * For deep sleep this requires enabling GIRQ22 wake clocks feature.
 */
static int i2c_xec_nl_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t i2c_base = devcfg->i2c_base;

	LOG_DBG("PM action: %d", (int)action);

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
#ifdef CONFIG_I2C_TARGET
		struct xec_i2c_nl_data *data = dev->data;

		if (data->target_bitmap; != 0) {
			sys_clear_bit(i2c_base + XEC_I2C_WKSR_OFS, XEC_I2C_WKSR_SB_POS);
			sys_set_bit(i2c_base + XEC_I2C_WKCR_OFS, XEC_I2C_WKCR_SBEN_POS);
			/* Enable GIRQ22 non-interrupt HW wake */
			soc_ecia_girq_ctrl(devcfg->girq_wake, devcfg->girq_wake_pos, 1u);
		}
#else
		sys_clear_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);
#endif
		break;
	case PM_DEVICE_ACTION_RESUME:
		soc_ecia_girq_status_clear(devcfg->girq_wake, devcfg->girq_wake_pos);
		sys_clear_bit(i2c_base + XEC_I2C_WKCR_OFS, XEC_I2C_WKCR_SBEN_POS);
		sys_clear_bit(i2c_base + XEC_I2C_WKSR_OFS, XEC_I2C_WKSR_SB_POS);
		sys_set_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif

#ifdef CONFIG_I2C_TARGET
static int check_target_config(struct i2c_target_config *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}

	if (cfg->address > 0x7Fu) {
		LOG_ERR("HW only supports 7-bit I2C addresses");
		return -EINVAL;
	}

	if (cfg->callbacks == NULL) {
		LOG_ERR("No callbacks");
		return -EINVAL;
	}

	if (cfg->callbacks->buf_write_received == NULL) {
		LOG_ERR("No call callback for received writes!");
		return -EINVAL;
	}

	if (cfg->callbacks->buf_read_requested == NULL) {
		LOG_ERR("No callback for read requests!");
		return -EINVAL;
	}

	if (cfg->callbacks->stop == NULL) {
		LOG_ERR("No callback for stop!");
		return -EINVAL;
	}

	return 0;
}

static int cfg_own_addr(const struct device *dev, struct i2c_target_config *cfg)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint32_t oaval = sys_read32(i2c_base + XEC_I2C_OA_OFS);
	int rc = 0;

	if (data->target_cfg_oa1 == NULL) {
		data->target_bitmap |= BIT(0);
		data->target_cfg_oa1 = cfg;
		oaval &= (uint32_t)~XEC_I2C_OA_1_MSK;
		oaval |= XEC_I2C_OA_1_SET((uint32_t)cfg->address);
		sys_write32(oaval, i2c_base + XEC_I2C_OA_OFS);
	} else if (data->target_cfg_oa2 == NULL) {
		data->target_bitmap |= BIT(1);
		data->target_cfg_oa2 = cfg;
		oaval &= (uint32_t)~XEC_I2C_OA_2_MSK;
		oaval |= XEC_I2C_OA_2_SET((uint32_t)cfg->address);
		sys_write32(oaval, i2c_base + XEC_I2C_OA_OFS);
	} else {
		rc = -EADDRINUSE;
	}

	return 0;
}

/* API for register one target at a time since it passes struct i2c_target_config not a list
 * containing struct i2c_target_config.
 */
static int xec_i2c_nl_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	struct soc_xec_dma_chan_cfg *dcfg = &data->tdcfg;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint32_t orig_target_bitmap = data->target_bitmap;
	uint32_t extlen = 0, tcmd = 0;
	int rc = 0;

	rc = check_target_config(cfg);
	if (rc != 0) {
		return rc;
	}

	rc = k_mutex_lock(&data->lock_mut, K_NO_WAIT);
	if (rc != 0) {
		return -EAGAIN;
	}

	rc = -EADDRINUSE;
	switch (cfg->address) {
#ifdef CONFIG_I2C_XEC_NL_TARGET_GENERAL_CALL
	case XEC_I2C_GEN_CALL_ADDR:
		if (data->target_cfg_gen_call == NULL) {
			data->target_bitmap |= BIT(2);
			data->target_cfg_gen_call = cfg;
			sys_clear_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_GC_DIS_POS);
			rc = 0;
		}
		break;
#endif
#ifdef CONFIG_I2C_XEC_NL_TARGET_SMBUS_HOST_DEV_ADDRESSES
	case XEC_I2C_SMB_HOST_ADDR:
		if (data->target_cfg_smb_host == NULL) {
			data->target_bitmap |= BIT(3);
			data->target_cfg_smb_host = cfg;
			sys_set_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_DSA_POS);
			rc = 0;
		}
		break;
	case XEC_I2C_SMB_DEVICE_ADDR:
		if (data->target_cfg_smb_dev == NULL) {
			data->target_bitmap |= BIT(4);
			data->target_cfg_smb_dev = cfg;
			sys_set_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_DSA_POS);
			rc = 0;
		}
		break;
#endif
	default:
		rc = cfg_own_addr(dev, cfg);
		break;
	} /* end switch */

	if ((rc == 0) && (orig_target_bitmap == 0)) {
		soc_xec_dmac_chan_clear(devcfg->hm_dma_chan);

		dcfg->daddr = (uint32_t)i2c_base + XEC_I2C_TRX_OFS;
		dcfg->maddr = (uint32_t)devcfg->xfrbuf;
		dcfg->nbytes = (uint32_t)devcfg->xfrbuf_sz;
		dcfg->flags = (XEC_DMAC_CHAN_CFG_DEV2MEM | XEC_DMAC_CHAN_CFG_HFC |
			       XEC_DMAC_CHAN_CFG_UNITS_SET(XEC_DMAC_CHAN_CFG_UNITS_1) |
			       XEC_DMAC_CHAN_CFG_INCRM |
			       XEC_DMAC_CHAN_CFG_HDEVID_SET(devcfg->tm_dma_trigsrc));

		soc_xec_dmac_chan_cfg2(devcfg->tm_dma_chan, dcfg, NULL);
		soc_xec_dmac_chan_start(devcfg->tm_dma_chan, 0);

		extlen = XEC_I2C_ELEN_TRD_SET(dcfg->nbytes >> 8);
		extlen |= XEC_I2C_ELEN_TWR_SET(XEC_I2C_NL_MAX_LEN >> 8);

		tcmd = XEC_I2C_TCMD_RCL_SET(dcfg->nbytes);
		tcmd |= XEC_I2C_TCMD_WCL_SET(XEC_I2C_NL_MAX_LEN);
		tcmd |= (BIT(XEC_I2C_TCMD_RUN_POS) | BIT(XEC_I2C_TCMD_PROC_POS));

		sys_write32(extlen, i2c_base + XEC_I2C_ELEN_OFS);
		sys_write32(tcmd, i2c_base + XEC_I2C_TCMD_OFS);
		sys_set_bits(i2c_base + XEC_I2C_CFG_OFS,
			     (BIT(XEC_I2C_CFG_TD_IEN_POS) | BIT(XEC_I2C_CFG_STD_NL_IEN_POS)));
		soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 1u);
	}

	k_mutex_unlock(&data->lock_mut);

	return rc;
}

static int xec_i2c_nl_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint32_t msk = 0;
	int rc = 0;

	if (cfg == NULL) {
		return -EINVAL;
	}

	rc = k_mutex_lock(&data->lock_mut, K_NO_WAIT);
	if (rc != 0) {
		return -EAGAIN;
	}

	switch (cfg->address) {
#ifdef CONFIG_I2C_XEC_NL_TARGET_GENERAL_CALL
	case XEC_I2C_GEN_CALL_ADDR:
		data->target_cfg_gen_call = NULL;
		data->target_bitmap &= ~BIT(2);
		sys_set_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_GC_DIS_POS);
		break;
#endif
#ifdef CONFIG_I2C_XEC_NL_TARGET_SMBUS_HOST_DEV_ADDRESSES
	case XEC_I2C_SMB_HOST_ADDR:
		data->target_cfg_smb_host = NULL;
		data->target_bitmap &= ~BIT(3);
		if (data->target_cfg_smb_dev == NULL) {
			sys_clear_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_DSA_POS);
		}
		break;
	case XEC_I2C_SMB_DEVICE_ADDR:
		data->target_cfg_smb_dev = cfg;
		data->target_bitmap &= ~BIT(4);
		if (data->target_cfg_smb_host == NULL) {
			sys_clear_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_DSA_POS);
		}
		break;
#endif
	default:
		if (cfg == data->target_cfg_oa1) {
			data->target_cfg_oa1 = NULL;
			data->target_bitmap &= ~BIT(0);
			msk = XEC_I2C_OA_1_MSK;
		} else if (cfg == data->target_cfg_oa2) {
			data->target_cfg_oa2 = NULL;
			data->target_bitmap &= ~BIT(1);
			msk = XEC_I2C_OA_1_MSK;
		} else {
			rc = -ENOENT;
		}

		if (msk != 0) {
			sys_clear_bits(i2c_base + XEC_I2C_OA_OFS, msk);
		}

		break;
	} /* end switch */

	if (data->target_bitmap == 0) {
		sys_clear_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_TD_IEN_POS);
		soc_xec_dmac_chan_clear(devcfg->tm_dma_chan);
		sys_write32(0, i2c_base + XEC_I2C_TCMD_OFS);
		sys_write32(0, i2c_base + XEC_I2C_ELEN_OFS);
		sys_set_bit(i2c_base + XEC_I2C_CMPL_OFS, XEC_I2C_CMPL_TDONE_POS);
		soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 0);
		soc_ecia_girq_status_clear(devcfg->girq, devcfg->girq_pos);
	}

	k_mutex_unlock(&data->lock_mut);

	return rc;
}
#endif

/* Driver initialization */
static int xec_i2c_nl_dma_init(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	uint32_t chan_girq_en_mask = BIT(devcfg->hm_dma_chan) | BIT(devcfg->tm_dma_chan);
	int rc = 0;

	if (IS_ENABLED(CONFIG_DMA_MCHP_XEC) != 0) {
		const struct device *xec_dmac_dev = DEVICE_DT_GET(DT_NODELABEL(dmac));
		bool result1 = dma_chan_filter(xec_dmac_dev, devcfg->hm_dma_chan, NULL);
		bool result2 = dma_chan_filter(xec_dmac_dev, devcfg->tm_dma_chan, NULL);

		if ((result1 == true) || (result2 == true)) {
			rc = -EBUSY;
			LOG_ERR("One or both DMA chans owned by XEC DMA driver!");
		}
	} else {
		rc = soc_xec_dmac_init(chan_girq_en_mask);
	}

	return rc;
}

static int xec_i2c_nl_init(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	int rc = 0;

	data->dev = dev;
	data->i2c_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
#ifdef CONFIG_I2C_XEC_PORT_MUX
	data->i2c_config |= I2C_XEC_PORT_SET(devcfg->port);
#endif

	k_mutex_init(&data->lock_mut);
	k_sem_init(&data->sync_sem, 0, K_SEM_MAX_LIMIT);
#ifdef CONFIG_I2C_XEC_NL_USE_KWORKQUEUE
	k_work_init(&data->kwq, &xec_i2c_nl_kworker);
#endif
#ifdef CONFIG_I2C_TARGET
	data->target_bitmap = 0;
#endif

	rc = xec_i2c_nl_dma_init(dev);
	if (rc != 0) {
		return rc;
	}

	rc = pinctrl_apply_state(devcfg->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("Pinctrl error (%d)", rc);
		return rc;
	}

	/* configure I2C controller */
	rc = xec_i2c_nl_hw_cfg(dev, devcfg->bitrate, devcfg->port, 0);
	if (rc != 0) {
		LOG_ERR("HW config error (%d)", rc);
		return rc;
	}

	if (devcfg->irq_config != NULL) {
		devcfg->irq_config();
		soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 1u);
	}

	return 0;
}

int i2c_mchp_xec_nl_debug_init(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;

	memset((void *)devcfg->xfrbuf, 0xaa, devcfg->xfrbuf_sz);

	data->htcmd = 0;
	data->hflags = 0;

	XEC_I2C_DEBUG_ISR_INIT(data);
	XEC_I2C_DEBUG_STATE_INIT(data);

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
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = xec_i2c_nl_transfer_cb,
#endif
#ifdef CONFIG_I2C_RTIO
	/* default submit i2c_iodev_submit_fallback */
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
	.recover_bus = xec_i2c_nl_recover_bus,
};

#define XEC_I2C_GIRQ_DT(inst)     MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, 0))
#define XEC_I2C_GIRQ_POS_DT(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, 0))

#define XEC_I2C_WK_GIRQ_DT(inst) MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, 1))
#define XEC_I2C_WK_GIRQ_POS_DT(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, 1))

#define XEC_I2C_NL_DMA_NODE(i, name)    DT_INST_DMAS_CTLR_BY_NAME(i, name)
#define XEC_I2C_NL_DMA_CHAN(i, name)    DT_INST_DMAS_CELL_BY_NAME(i, name, channel)
#define XEC_I2C_NL_DMA_TRIGSRC(i, name) DT_INST_DMAS_CELL_BY_NAME(i, name, trigsrc)

#define XEC_I2C_NL_DMA_IRQN(i, name)                                                               \
	DT_IRQ_BY_IDX(XEC_I2C_NL_DMA_NODE(i, name), XEC_I2C_NL_DMA_CHAN(i, name), irq)

#define XEC_I2C_NL_DMA_IRQP(i, name)                                                               \
	DT_IRQ_BY_IDX(XEC_I2C_NL_DMA_NODE(i, name), XEC_I2C_NL_DMA_CHAN(i, name), priority)

#define XEC_I2C_NL_XFRBUF_SIZE(i)                                                                  \
	(DT_INST_PROP_OR(i, xfr_buffer_size, 32) + XEC_I2C_NL_XFRBUF_PAD_SZ)

#define XEC_I2C_NL_XFRBUF_DEFINE(i)                                                                \
	static volatile uint8_t xec_i2c_nl##i##_xfrbuf[XEC_I2C_NL_XFRBUF_SIZE(i)] __aligned(4)

#define XEC_I2C_DATA_DEFINE(i) struct xec_i2c_nl_data xec_i2c_nl##i##_data

#ifdef CONFIG_I2C_TARGET
#define XEC_I2C_NL_TM_IRQ_CONN(inst)                                                               \
	IRQ_CONNECT(XEC_I2C_NL_DMA_IRQN(inst, tm), XEC_I2C_NL_DMA_IRQP(inst, tm),                  \
		    xec_i2c_nl_tm_dma_isr, DEVICE_DT_INST_GET(inst), 0);                           \
	irq_enable(XEC_I2C_NL_DMA_IRQN(inst, tm));
#else
#define XEC_I2C_NL_TM_IRQ_CONN(inst)
#endif

#define XEC_I2C_NL_DEVICE(inst)                                                                    \
	XEC_I2C_NL_XFRBUF_DEFINE(inst);                                                            \
	XEC_I2C_DATA_DEFINE(inst);                                                                 \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static void xec_i2c_nl##inst##_irq_config(void)                                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), xec_i2c_nl_isr,       \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
		IRQ_CONNECT(XEC_I2C_NL_DMA_IRQN(inst, hm), XEC_I2C_NL_DMA_IRQP(inst, hm),          \
			    xec_i2c_nl_hm_dma_isr, DEVICE_DT_INST_GET(inst), 0);                   \
		irq_enable(XEC_I2C_NL_DMA_IRQN(inst, hm));                                         \
		XEC_I2C_NL_TM_IRQ_CONN(inst)                                                       \
	}                                                                                          \
	static const struct xec_i2c_nl_config xec_i2c_nl##inst##_cfg = {                           \
		.i2c_base = (mem_addr_t)DT_INST_REG_ADDR(inst),                                    \
		.bitrate = DT_INST_PROP_OR(inst, clock_frequency, I2C_BITRATE_STANDARD),           \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
		.irq_config = xec_i2c_nl##inst##_irq_config,                                       \
		.hm_dma_chan = XEC_I2C_NL_DMA_CHAN(inst, hm),                                      \
		.hm_dma_trigsrc = XEC_I2C_NL_DMA_TRIGSRC(inst, hm),                                \
		.tm_dma_chan = XEC_I2C_NL_DMA_CHAN(inst, tm),                                      \
		.tm_dma_trigsrc = XEC_I2C_NL_DMA_TRIGSRC(inst, tm),                                \
		.girq = XEC_I2C_GIRQ_DT(inst),                                                     \
		.girq_pos = XEC_I2C_GIRQ_POS_DT(inst),                                             \
		.girq_wake = XEC_I2C_WK_GIRQ_DT(inst),                                             \
		.girq_wake_pos = XEC_I2C_WK_GIRQ_POS_DT(inst),                                     \
		.pcr = DT_INST_PROP(inst, pcr),                                                    \
		.port = DT_INST_PROP(inst, port_sel),                                              \
		.xfrbuf_sz = XEC_I2C_NL_XFRBUF_SIZE(inst),                                         \
		.xfrbuf = xec_i2c_nl##inst##_xfrbuf,                                               \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(inst, i2c_xec_nl_pm_action);                                      \
	I2C_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_init, PM_DEVICE_DT_INST_GET(inst),              \
				  &xec_i2c_nl##inst##_data, &xec_i2c_nl##inst##_cfg, POST_KERNEL,  \
				  CONFIG_I2C_INIT_PRIORITY, &xec_i2c_nl_driver_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_DEVICE)
