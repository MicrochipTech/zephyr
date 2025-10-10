/*
 * Copyright 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_i2c_nl

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
#include "zephyr/sys/slist.h"
#include "zephyr/sys/sys_io.h"
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_mchp_xec_nl);

#include "i2c-priv.h"

BUILD_ASSERT(CONFIG_I2C_TARGET_BUFFER_MODE,
	     "MCHP I2C-NL target support requires I2C_TARGET_BUFFER_MODE");

/* #define XEC_I2C_NL_DEBUG_ISR */
/* #define XEC_I2C_DEBUG_STATE */
#define XEC_I2C_DEBUG_STATE_ENTRIES 256

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
	struct i2c_msg *m; /* starting message in group */
	uint8_t nmsgs;     /* number of messages in group */
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
	const struct device *hc_dma_dev;
	uint8_t hc_dma_chan;
	uint8_t hc_dma_trigsrc;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t pcr;
	uint8_t port;
	volatile uint8_t *xfrbuf;
	uint32_t xfrbuf_sz;
#ifdef CONFIG_I2C_TARGET
	sys_slist_t target_list;
	struct i2c_target_config *curr_target;
	const struct device *tc_dma_dev;
	uint8_t tc_dma_chan;
	uint8_t tc_dma_trigsrc;
	uint8_t ntargets;
#endif
};

/* alternate method 1: use one struct dma_clock_config
 *  requires DMA callback to reload m2d channel if tx data doesn't fit driver tx buffer
 *      struct dma_config m2d_cfg;         32 bytes
 *	struct dma_config d2m_cfg;         32 bytes
 *	struct dma_block_config dcfg;      28 bytes
 * 	struct i2c_msg m2d_msgs[3];        3 * 12 bytes = 36
 *	struct i2c_msg d2m_msg;            12 bytes
 *                                  total  140 bytes
 *
 * alternate method 1: use one struct dma_clock_config
 *  requires DMA callback to reload m2d channel if tx data doesn't fit driver tx buffer
 *      struct dma_config m2d_cfg;         32 bytes
 *	struct dma_config d2m_cfg;         32 bytes
 *	struct dma_block_config dcfg;      28 bytes
 *      flags (16 or 32 bits)			1 byte
 * 		bit[0] = driver tx buffer
 * 		bit[1] = struct i2c_msg.
 * 		bit[2] = driver tx buffer end last byte of driver tx buffer contains RPT-START addr
 *      driver tx buffer len (32 bits or 16 bits)?  2 or 4 bytes
 * 	uint8_t index for bit[1]			1 byte
 *				total = 92 + 1 + 4 + 1 = 98 bytes
 *
 * Will the above alternate methods work for all message combinations implemented in zephyr i2c.h
 * One write msg 	OK
 * 	DMA driver tx buffer, len=msg.len + 1
 * 		OR
 * 	DMA write address, len=1
 * 	DMA msg, len=msg.len
 * One read msg		OK
 * 	DMA driver tx buffer = read address, len=1
 * 	DMA to msg.buf, len=msg.len uses d2m
 * Two write msgs	?
 * 	worst case both are larger than driver buffer
 * 	DMA write address, len=1	setup/start in transfer API
 * 	DMA msg[0], len=msg[0].len	m2d callback
 * 	DMA msg[1], len=msg[1].len	m2d callback
 *
 * One write, one read	?
 * 	DMA tx buffer, len=1+msg[0].len		m2d callback on done, no reload
 * 	DMA msg[1], len=msg[1].len		d2m callback on done, no reload
 * 		OR
 *	DMA tx buffer = write address, len=1	m2d callback on done, reload msg[0]
 * 	DMA msg[0], len=msg[0].len	m2d callback on done, no reload
 * 	DMA msg[1], len=msg[1].len	d2m callback on done, no reload
 */

#define XEC_AFLAG_WR_PH_BUF_POS    0
#define XEC_AFLAG_WR_PH_BUF2_POS   1
#define XEC_AFLAG_RD_PH_BUF_POS    2
#define XEC_AFLAG_EN_GIRQ_POS      7
#define XEC_AFLAG_TM_START_DET_POS 8
#define XEC_AFLAG_TM_STOP_DET_POS  9

struct xec_i2c_nl_data {
	const struct device *dev;
	struct k_mutex lock_mut;
	struct k_sem sync_sem;
	uint32_t i2c_config;
#ifdef CONFIG_I2C_XEC_NL_USE_KWORKQUEUE
	struct k_work kwq;
#endif
	volatile uint32_t i2c_status;
	volatile uint32_t i2c_cfg_reg;
	uint16_t i2c_addr;
	uint8_t i2c_cr;
	uint8_t target_mode;
	struct dma_config dma_cfg;           /* 32 bytes */
	struct dma_block_config dma_blk_cfg; /* 28 bytes */
	struct i2c_msg *msgs;
	uint8_t *write_phase_buf;
	uint32_t write_phase_len;
	uint8_t *write_phase_buf2;
	uint32_t write_phase_len2;
	uint8_t *read_phase_buf;
	uint32_t read_phase_len;
	ATOMIC_DEFINE(aflags, 32);
	uint32_t htcmd;
	uint8_t num_msgs;
	uint8_t midx;
	uint8_t num_msgs_xfr;
	uint8_t htm_state;
	uint8_t protocol;
	uint8_t hflags;
	uint16_t wrcnt;
	uint16_t rdcnt;
#ifdef CONFIG_I2C_TARGET
	sys_slist_t target_list;
	struct i2c_target_config *curr_target;
	uint8_t ntargets;
	volatile uint8_t tm_addr;
	volatile uint16_t tm_rd_cnt;
	uint32_t tm_total_rd_cnt;
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	uint32_t buf_req_size;
	uint8_t *buf_req_ptr;
#endif
#endif
#ifdef XEC_I2C_NL_DEBUG_ISR
	volatile uint32_t isr_count;
	volatile uint32_t hcmd;
	volatile uint32_t tcmd;
#endif
#ifdef XEC_I2C_DEBUG_STATE
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

/* local prototypes */
static void xec_i2c_nl_hc_cb(const struct device *dev, void *user_data, uint32_t chan, int status);
#ifdef CONFIG_I2C_TARGET
static void xec_i2c_nl_tc_cb(const struct device *dev, void *user_data, uint32_t chan, int status);
#endif

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

#ifdef CONFIG_I2C_TARGET
static bool xec_i2c_nl_is_tm(const struct device *dev)
{
	struct xec_i2c_nl_data *const data = dev->data;

	if (data->ntargets != 0) {
		return true;
	}

	return false;
}

static uint32_t prog_target_addresses(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	mem_addr_t rb = devcfg->i2c_base;
	sys_snode_t *sn = NULL;
	uint32_t own_addrs = 0, n = 0;

	SYS_SLIST_FOR_EACH_NODE(&data->target_list, sn) {
		struct i2c_target_config *ptc = CONTAINER_OF(sn, struct i2c_target_config, node);

		if (ptc != NULL) {
			n++;
			if (ptc->address == XEC_I2C_GEN_CALL_ADDR) {
				sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_GC_DIS_POS);
			} else if ((ptc->address == XEC_I2C_SMB_HOST_ADDR) ||
				   (ptc->address == XEC_I2C_SMB_DEVICE_ADDR)) {
				sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_DSA_POS);
			} else {
				own_addrs = sys_read32(rb + XEC_I2C_OA_OFS);
				if ((own_addrs & XEC_I2C_OA_1_MSK) == 0) {
					own_addrs |= XEC_I2C_OA_1_SET(ptc->address);
					sys_write32(own_addrs, rb + XEC_I2C_OA_OFS);
				} else if ((own_addrs & XEC_I2C_OA_2_MSK) == 0) {
					own_addrs |= XEC_I2C_OA_2_SET(ptc->address);
					sys_write32(own_addrs, rb + XEC_I2C_OA_OFS);
				}
			}
		}
	}

	return n;
}
#endif

static void xec_i2c_nl_dma_cfg_init(const struct device *dev, struct dma_config *dcfg,
				    uint8_t flags)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x30);

	dcfg->complete_callback_en = 0u; /* callback on completion only */
	dcfg->error_callback_dis = 0u;   /* allow callback on errro */
	dcfg->source_handshake = 0u;     /* HW handshake */
	dcfg->dest_handshake = 0u;       /* HW handshake */
	dcfg->channel_priority = 0u;     /* N/A */
	dcfg->source_chaining_en = 0u;   /* N/A */
	dcfg->dest_chaining_en = 0u;     /* N/A */
	dcfg->linked_channel = 0u;       /* N/A */
	dcfg->cyclic = 0u;               /* N/A */
	dcfg->source_data_size = 1u;     /* I2C data register is byte sized */
	dcfg->dest_data_size = 1u;       /* I2C data register is byte sized */
	dcfg->source_burst_length = 0u;  /* N/A */
	dcfg->dest_burst_length = 0u;    /* N/A */
	dcfg->block_count = 1u;          /* this driver implements one struct dma_block_config */
	dcfg->user_data = (void *)data;  /* point to I2C driver data */

	dcfg->head_block = &data->dma_blk_cfg;

#ifdef CONFIG_I2C_TARGET
	if ((flags & XEC_I2C_NL_DMA_CFG_HC) != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x31);
		dcfg->dma_slot = devcfg->hc_dma_trigsrc;
		dcfg->dma_callback = xec_i2c_nl_hc_cb;
	} else {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x32);
		dcfg->dma_slot = devcfg->tc_dma_trigsrc;
		dcfg->dma_callback = xec_i2c_nl_tc_cb;
	}
#else
	dcfg->dma_slot = devcfg->hc_dma_trigsrc;
	dcfg->dma_callback = xec_i2c_nl_hc_cb;
#endif
	if ((flags & XEC_I2C_NL_DMA_CFG_M2D) != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x33);
		dcfg->channel_direction = MEMORY_TO_PERIPHERAL;
	} else {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x34);
		dcfg->channel_direction = PERIPHERAL_TO_MEMORY;
	}
}

static void xec_i2c_nl_dma_cfg_block(const struct device *dev, struct dma_block_config *blkcfg,
				     void *buf, uint32_t len, uint8_t flags)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t rb = devcfg->i2c_base;
#ifdef XEC_I2C_DEBUG_STATE
	struct xec_i2c_nl_data *data = dev->data;
#endif
	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x38);

	if ((flags & XEC_I2C_NL_DMA_CFG_M2D) != 0) {
		blkcfg->source_address = (uint32_t)buf;
		if ((flags & XEC_I2C_NL_DMA_CFG_HC) != 0) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x39);
			blkcfg->dest_address = (uint32_t)(rb + XEC_I2C_HTX_OFS);
		} else {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x3A);
			blkcfg->dest_address = (uint32_t)(rb + XEC_I2C_TTX_OFS);
		}
		blkcfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		blkcfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		if ((flags & XEC_I2C_NL_DMA_CFG_HC) != 0) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x3B);
			blkcfg->source_address = (uint32_t)(rb + XEC_I2C_HRX_OFS);
		} else {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x3C);
			blkcfg->source_address = (uint32_t)(rb + XEC_I2C_TRX_OFS);
		}
		blkcfg->dest_address = (uint32_t)buf;
		blkcfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blkcfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}

	blkcfg->source_gather_interval = 0;
	blkcfg->dest_scatter_interval = 0;
	blkcfg->dest_scatter_count = 0;
	blkcfg->source_gather_count = 0;
	blkcfg->block_size = len;
	blkcfg->next_block = NULL;
	blkcfg->source_gather_en = 0;
	blkcfg->dest_scatter_en = 0;
	blkcfg->dest_reload_en = 0;
	blkcfg->fifo_mode_control = 0;
	blkcfg->flow_control_mode = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x3D);
}

/* After I2C-NL controller reset and configuration we need to
 * configure DMA channel used by controller host mode.
 */
static void xec_i2c_nl_dma_clear(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;

	dma_stop(devcfg->hc_dma_dev, devcfg->hc_dma_chan);
#ifdef CONFIG_I2C_TARGET
	dma_stop(devcfg->tc_dma_dev, devcfg->tc_dma_chan);
#endif
}

#ifdef CONFIG_I2C_TARGET
static void xec_i2c_nl_tm_cfg1(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint32_t tcmd = (BIT(XEC_I2C_TCMD_RUN_POS) | BIT(XEC_I2C_TCMD_PROC_POS) |
			 XEC_I2C_TCMD_WCL_SET(0xffu) | XEC_I2C_TCMD_RCL_SET(0xffu));
	uint8_t dma_flags = XEC_I2C_NL_DMA_CFG_TC | XEC_I2C_NL_DMA_CFG_D2M;

	atomic_clear(data->aflags);

	data->htm_state = 0;
	data->curr_target = NULL;
	data->buf_req_ptr = NULL;
	data->buf_req_size = 0;
	data->tm_rd_cnt = 0xffffu;
	data->tm_total_rd_cnt = 0;
	data->htcmd = tcmd;

	/* clear target mode done IEN and enable it later (toggle) */
	sys_clear_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_TD_IEN_POS);

	sys_write32(0, i2c_base + XEC_I2C_TCMD_OFS);
	xec_i2c_cr_write(dev, I2C_XEC_CR_PIN_ESO_ACK);
	data->i2c_status = xec_i2c_status_get(i2c_base);

	/* flush I2C-NL TM buffers */
	sys_set_bits(i2c_base + XEC_I2C_CFG_OFS,
		     (BIT(XEC_I2C_CFG_FTTX_POS) | BIT(XEC_I2C_CFG_FTRX_POS)));
	sys_set_bits(i2c_base + XEC_I2C_CMPL_OFS, XEC_I2C_CMPL_RW1C_MSK);

	xec_i2c_nl_dma_cfg_init(dev, &data->dma_cfg, dma_flags);
	xec_i2c_nl_dma_cfg_block(dev, &data->dma_blk_cfg, (void *)devcfg->xfrbuf, devcfg->xfrbuf_sz,
				 dma_flags);

	dma_config(devcfg->tc_dma_dev, devcfg->tc_dma_chan, &data->dma_cfg);
	dma_start(devcfg->tc_dma_dev, devcfg->tc_dma_chan);

	/* configure I2C controller extended length and target command registers
	 * Development purposes: set write and read count to max.
	 * Required to set TPROCEED b[1], and TRUN bit[0] to 1.
	 */
	sys_write32((XEC_I2C_ELEN_TRD_SET(0xffu) + XEC_I2C_ELEN_TWR_SET(0xffu)),
		    i2c_base + XEC_I2C_ELEN_OFS);

	sys_write32(tcmd, i2c_base + XEC_I2C_TCMD_OFS);

	k_busy_wait(100u); /* HW needs time to synchronize with SCL/SDA pins */

	data->i2c_status = xec_i2c_status_get(i2c_base);

	sys_set_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_TD_IEN_POS);
}
#endif

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

#ifdef CONFIG_I2C_TARGET
	prog_target_addresses(dev);
#endif

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

#ifdef CONFIG_I2C_TARGET
	if (xec_i2c_nl_is_tm(dev) == true) {
		xec_i2c_nl_tm_cfg1(dev);
		soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 1u);
	}
#endif

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
#ifdef CONFIG_I2C_TARGET
	if (data->ntargets != 0) {
		cfg &= ~I2C_MODE_CONTROLLER;
	} else {
		cfg |= I2C_MODE_CONTROLLER;
	}
#endif
	*i2c_config = cfg;

	return 0;
}

/* I2C Host controller mode DMA channel callback */
static void xec_i2c_nl_hc_cb(const struct device *dev, void *user_data, uint32_t chan, int status)
{
	struct xec_i2c_nl_data *data = (struct xec_i2c_nl_data *)user_data;
	const struct device *i2c_dev = data->dev;
	const struct xec_i2c_nl_config *devcfg = i2c_dev->config;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint32_t dst = (uint32_t)(i2c_base + XEC_I2C_HTX_OFS);
	struct dma_status dma_status = {0};
	uint32_t src = 0, len = 0;
	int rc = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xF0);

	if (status == 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xF1);

		dma_get_status(dev, chan, &dma_status);

		if (dma_status.dir == MEMORY_TO_PERIPHERAL) {
			if (atomic_test_and_clear_bit(data->aflags, XEC_AFLAG_WR_PH_BUF_POS) ==
			    true) {
				XEC_I2C_DEBUG_STATE_UPDATE(data, 0xF2);
				src = (uint32_t)data->write_phase_buf;
				len = data->write_phase_len;
			} else if (atomic_test_and_clear_bit(data->aflags,
							     XEC_AFLAG_WR_PH_BUF2_POS) == true) {
				XEC_I2C_DEBUG_STATE_UPDATE(data, 0xF3);
				src = (uint32_t)data->write_phase_buf2;
				len = data->write_phase_len2;
			}

			if ((src != 0) && (len != 0)) {
				rc = dma_reload(dev, chan, src, dst, len);
				if (rc != 0) {
					XEC_I2C_DEBUG_STATE_UPDATE(data, 0xF4);
				}
			}
		}
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xF5);
}

/* TODO mutex/semaphore while changing? */
#ifdef CONFIG_I2C_TARGET
static struct i2c_target_config *find_target(struct xec_i2c_nl_data *data, uint16_t i2c_addr)
{
	sys_snode_t *sn = NULL;

	SYS_SLIST_FOR_EACH_NODE(&data->target_list, sn) {
		struct i2c_target_config *ptc = CONTAINER_OF(sn, struct i2c_target_config, node);

		if ((ptc != NULL) && (ptc->address == i2c_addr)) {
			return ptc;
		}
	}

	return NULL;
}

static int xec_i2c_nl_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	mem_addr_t rb = devcfg->i2c_base;
	uint32_t oaval = 0;
	int rc = 0;

	if (cfg == NULL) {
		return -EINVAL;
	}

	if (((cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0) || (cfg->address > 0x7FU)) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock_mut, K_FOREVER);

	rc = -ENFILE;
	if (data->ntargets < 6) {
		struct i2c_target_config *ptc = find_target(data, cfg->address);

		if (ptc == NULL) {
			sys_slist_append(&data->target_list, &cfg->node);
			if (cfg->address == XEC_I2C_GEN_CALL_ADDR) {
				/* enable general call */
				sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_GC_DIS_POS);
			} else if ((cfg->address == XEC_I2C_SMB_HOST_ADDR) ||
				   (cfg->address == XEC_I2C_SMB_DEVICE_ADDR)) {
				/* enable DSA */
				sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_DSA_POS);
			} else { /* use one of the two own addresses */
				oaval = sys_read32(rb + XEC_I2C_OA_OFS);
				if (XEC_I2C_OA_1_GET(oaval) == 0) {
					oaval |= XEC_I2C_OA_1_SET(cfg->address);
					sys_write32(oaval, rb + XEC_I2C_OA_OFS);
				} else if (XEC_I2C_OA_2_GET(oaval) == 0) {
					oaval |= XEC_I2C_OA_2_SET(cfg->address);
					sys_write32(oaval, rb + XEC_I2C_OA_OFS);
				}
			}
			rc = 0;
		}
	}

	if (rc == 0) {
		if (data->ntargets == 0) {
			XEC_I2C_DEBUG_ISR_INIT(data);
			XEC_I2C_DEBUG_STATE_INIT(data);
			xec_i2c_nl_tm_cfg1(dev);
		}
		data->ntargets++;
		soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 1u);
	}

	k_mutex_unlock(&data->lock_mut);

	return 0;
}

/* TODO refactor */
static int xec_i2c_nl_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	mem_addr_t rb = devcfg->i2c_base;
	uint32_t oaval = 0;
	uint16_t taddr1 = 0, taddr2 = 0;
	int rc = 0;
	bool removed = false;

	if (cfg == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock_mut, K_FOREVER);

	if (data->ntargets == 0) {
		goto targ_unreg_release_lock;
	}

	removed = sys_slist_find_and_remove(&data->target_list, &cfg->node);

	if (removed == false) {
		rc = -ENOSYS;
		goto targ_unreg_release_lock;
	}

	data->ntargets--;

	if (cfg->address == XEC_I2C_GEN_CALL_ADDR) { /* disable general call */
		sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_GC_DIS_POS);
	} else if ((cfg->address == XEC_I2C_SMB_HOST_ADDR) ||
		   (cfg->address == XEC_I2C_SMB_DEVICE_ADDR)) {
		sys_clear_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_DSA_POS);
	} else { /* one of the own addresses */
		oaval = sys_read32(rb + XEC_I2C_OA_OFS);
		taddr1 = XEC_I2C_OA_1_GET(oaval);
		taddr2 = XEC_I2C_OA_2_GET(oaval);
		if (taddr1 == (uint32_t)cfg->address) {
			oaval &= ~XEC_I2C_OA_1_MSK;
			sys_write32(oaval, rb + XEC_I2C_OA_OFS);
		} else if (taddr2 == (uint32_t)cfg->address) {
			oaval &= ~XEC_I2C_OA_2_MSK;
			sys_write32(oaval, rb + XEC_I2C_OA_OFS);
		}
	}

targ_unreg_release_lock:
	/* TODO if all targets removed then call k_work_cancel_sync() to
	 * remove target mode worker thread?
	 */
	k_mutex_unlock(&data->lock_mut);

	return rc;
}
#endif /* CONFIG_I2C_TARGET */

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

/* Closed, I2C Write + STOP
 * 	ntx = 1 + msg->len
 * 	nrx = 0
 * 	i2c_nl_flags = START0 | STOP
 *
 * Closed, I2C Write
 * 	ntx = max
 * 	nrx = max
 * 	i2c_nl_flags = START0
 *
 * Closed, I2C Read + STOP
 * 	ntx = 1
 * 	nrx = msg->len
 * 	i2c_nl_flags = START0 | STARTN | STOP
 *
 * Open, I2C Write
 *
 * !!! How to handle one message with STOP and data length > HW max? !!!
 * Write message:
 * 	START wrAddr [A] data[0] [A] ... data[N-1] [A] STOP
 * 	                              | HW length stops here !
 *	we can't let I2C-NL wrCnt reach <= 2.
 * 	Idea 1:
 * 		Set I2C-NL wrCnt or rdCnt to 128
 * 		Limit DMA to 64 bytes
 * 		DMA done
 * 			chunk_len = 64
 * 			if not last chunk
 * 				reprogram I2C-NL wrCnt or rdCnt to 128
 * 			else
 * 				if STOP required
 * 					chunk_len = remaining length
 * 				reprogram I2C-NL wrCnt or rdCnt to remaining length
 * 				!!! DOES NOT WORK DUE TO:
 * 				!!! DMA mem2dev stops but I2C-NL is still clocking out data
 * 				!!! we can't touch I2C-NL registers in DMA ISR while DMA is
 * 				!!! still working.
 * 				!!! Same issue for I2C read. DMA dev2mem reads from I2C-NL register
 * 				!!! which triggers I2C-NL to read-ahead. DMA can stop before I2C.
 * 				and set I2C-NL STOP flag
 * 			endif
 * 			reload DMA for chunk_len
 * 			restart DMA
 *
 * Idea 2 based on 1
 * struct xec_dma_buf {
 * 	uint8_t *buf;
 * 	uint32_t len;
 * 	uint32_t hw_data_addr;
 * 	uint8_t flags;
 * 	struct xec_dma_buf *next;
 * };
 * Set HCMD.wrCnt = 128, HCMD.rdCnt = 128, and HCMD.STOP=1
 * if CLOSED
 * 	HCMD.START0 = 1
 * 	DMA.len = 1
 * 	DMA.buf = i2c_write_addr buf
 * 	DMA.dir = Mem2Dev
 * 	dma_next.buf = m->buf
 * 	dma_next.len = m->len
 * 	dma_next.
 * else
 *
 * endif
 *
 * Parse messages into struct xec_i2c_nl_msg_group
 * One I2C Write with STOP
 * OR
 * One I2C Read with STOP
 * OR
 * Two message sequence:
 * 	Write Nw bytes
 * 	Read Nr bytes
 *
 */

/* API wrapper in i2c.h returns on num_msgs == 0. It also accesses msgs[] without checking for
 * NULL which will result in a fault if msgs is NULL.
 * This routine will perform a complete I2C transaction from START to STOP.
 * If no START flag is present on the first message, the driver adds START.
 * If no STOP flag is present in the last message, the driver adds STOP.
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * We must handle i2c_write, i2c_read, i2c_write_read, i2c_burst_write, etc wrappers
 * in i2c.h
 * One read or write msg with stop. -> START wrAddr wrData STOP or START rdAddr [rdData] STOP
 * Two write msgs, second with stop. -> START wrAddr msg0_wrData msg1_wrData STOP
 * Two msgs: write then read with stop. -> START wrAddr wrData RPT-START rdAddr [rdData] STOP
 * Option A: no double buffering
 * 	4 structures containing info for each phase
 * 	I2C Write 1
 * 		struct[0] START plus target write address, src=data->fmt_target_addr, len=1
 *              struct[1] target data, src=msg[m0]->buf, len=msg[0]->len
 * 		hw_flags = START0 | STOP
 * 	I2C Write 2
 * 		struct[0] START plus target write address, src=data->fmt_target_addr, len=1
 *              struct[1] target data msg[0], src=msg[0]->buf, len=msg[0]->len
 *		struct[2] target data msg[1], src=msg[1]->buf, len=msg[1]->len
 * 		hw_flags = START0 | STOP
 * 	I2C Read 1
 * 		struct[0] START plus target read address, src=data->fmt_target_addr, len=1
 * 		struct[1] dest=msg[0].buf, len=msg[0]->len
 * 		hw_flags = START0 | STOP
 * 	I2C Write-Read
 * 		struct[0] START plus target write address, src=data->fmt_target_addr, len=1
 * 		struct[1] target data, src=msg[m0]->buf, len=msg[0]->len
 * 		struct[2] target read address, src=data->fmt_target_addr2, len=1
 * 		struct[3] dest=msg[1].buf, len=msg[1]->len
 *
 * TODO
 * #define XEC_I2C_NL_TX_BUF_SZ     16u
 * #define XEC_I2C_NL_TX_BUF_PAD_SZ 4u
 * if tx message < (16 - 4u) then put it all in data->xfrbuf[] and use one m2d DMA
 *
 */
static int xec_i2c_nl_msgs_valid(struct i2c_msg *msgs, uint8_t num_msgs)
{
	if ((msgs == NULL) || (num_msgs == 0)) {
		return -EINVAL;
	}

	for (uint8_t n = 0; n < num_msgs; n++) {
		struct i2c_msg *m = &msgs[n];

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

/* Sets driver data:
 * I2C-NL control flagsh: hflags: START0 and STOP
 * I2C-NL write count: wrnct
 * I2C-NL read count: rdcnt
 * Number of struct soc_xec_dma_chan_cfg's used
 * filled in the struct soc_xec_dma_chan_cfg's
 */
static bool is_xfr_one_msg(struct i2c_msg *m, struct i2c_msg *mnext)
{
	if ((mnext == NULL) || ((m->flags & I2C_MSG_STOP) != 0) ||
	    ((m->flags & I2C_MSG_READ) != 0)) {
		return true;
	}

	return false;
}

static bool is_xfr_wr_rd(struct i2c_msg *m, struct i2c_msg *mnext)
{
	if ((mnext != NULL) && ((m->flags & I2C_MSG_READ) == 0) &&
	    ((mnext->flags & I2C_MSG_READ) != 0)) {
		return true;
	}

	return false;
}

#if 0
static void pr_msg(struct i2c_msg *m)
{
	LOG_DBG("I2C msg @ %p", m);
	if (m == NULL) {
		return;
	}

	LOG_DBG("  buf   = %p", (void *)m->buf);
	LOG_DBG("  len   = %u", m->len);
	LOG_DBG("  flags = 0x%02x", m->flags);
}

static void pr_xfr_data(struct xec_i2c_nl_data *data)
{
	if (data == NULL) {
		return;
	}

	LOG_DBG("I2C-NL data @ %p", (void *)data);
	LOG_DBG("  i2c_addr = 0x%0x", data->i2c_addr);
	LOG_DBG("  msgs = %p", (void *)data->msgs);
	LOG_DBG("  num_msgs = %u", data->num_msgs);
	LOG_DBG("  midx = %u", data->midx);
	LOG_DBG("  num_msgs_xfr = %u", data->num_msgs_xfr);
	LOG_DBG("  hflags = 0x%0x", data->hflags);
	LOG_DBG("  wrcnt = 0x%0x", data->wrcnt);
	LOG_DBG("  rdcnt = 0x%0x", data->rdcnt);
}
#endif

static void xfr_cfg_set_atomic_flags(const struct device *dev)
{
	struct xec_i2c_nl_data *data = dev->data;

	if ((data->write_phase_buf != NULL) && (data->write_phase_len != 0)) {
		atomic_set_bit(data->aflags, XEC_AFLAG_WR_PH_BUF_POS);
	}

	if ((data->write_phase_buf2 != NULL) && (data->write_phase_len2 != 0)) {
		atomic_set_bit(data->aflags, XEC_AFLAG_WR_PH_BUF2_POS);
	}

	if ((data->read_phase_buf != NULL) && (data->read_phase_len != 0)) {
		atomic_set_bit(data->aflags, XEC_AFLAG_RD_PH_BUF_POS);
	}
}

static int xfr_cfg_one_msg(const struct device *dev, struct i2c_msg *m)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	volatile uint8_t *xfrbuf = devcfg->xfrbuf;
	uint32_t dma_flags = XEC_I2C_NL_DMA_CFG_HC | XEC_I2C_NL_DMA_CFG_M2D;
	uint32_t ntx = 0, nrx = 0, ntx_buf = 0, dma_tx_len = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x10);

	data->write_phase_buf = NULL;
	data->write_phase_len = 0;
	data->write_phase_buf2 = NULL;
	data->write_phase_len2 = 0;
	data->read_phase_buf = NULL;
	data->read_phase_len = 0;
	data->num_msgs_xfr = 1u;
	data->hflags = BIT(0) | BIT(2); /* START0 | STOP */

	ntx = 1u;
	xfrbuf[0] = (data->i2c_addr & 0x7Fu) << 1;
	if ((m->flags & I2C_MSG_READ) != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x11);
		dma_tx_len = 1u;
		xfrbuf[0] |= BIT(0);
		nrx = m->len;
		data->protocol = XEC_I2C_NL_PROTO_RD;
		data->read_phase_buf = m->buf;
		data->read_phase_len = m->len;
	} else {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x12);
		ntx += m->len;
		data->protocol = XEC_I2C_NL_PROTO_WR;
		if (m->len <= (devcfg->xfrbuf_sz - XEC_I2C_NL_XFRBUF_PAD_SZ)) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x13);
			dma_tx_len = ntx;
			memcpy((void *)&xfrbuf[1], m->buf, m->len);
		} else {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x14);
			ntx_buf = (devcfg->xfrbuf_sz - XEC_I2C_NL_XFRBUF_PAD_SZ);
			dma_tx_len = ntx_buf + 1u;
			data->write_phase_buf = m->buf + ntx_buf;
			data->write_phase_len = m->len - ntx_buf;
			memcpy((void *)&xfrbuf[1], m->buf, ntx_buf);
		}
	}

	xfr_cfg_set_atomic_flags(dev);

	data->wrcnt = (uint16_t)ntx;
	data->rdcnt = (uint16_t)nrx;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x15);

	xec_i2c_nl_dma_cfg_init(dev, &data->dma_cfg, dma_flags);
	xec_i2c_nl_dma_cfg_block(dev, &data->dma_blk_cfg, (void *)xfrbuf, dma_tx_len, dma_flags);

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x16);

	return 0;
}

static int xfr_cfg_wr_rd(const struct device *dev, struct i2c_msg *mwr, struct i2c_msg *mrd)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	volatile uint8_t *xfrbuf = devcfg->xfrbuf;
	uint32_t dma_flags = XEC_I2C_NL_DMA_CFG_HC | XEC_I2C_NL_DMA_CFG_M2D;
	uint32_t ntx = 0, nrx = 0, tx_buf_max_len = 0, cplen = 0, rem = 0, dma_blk_len = 0;
	uint8_t i2c_wr_addr = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x17);

	data->num_msgs_xfr = 2u;
	data->protocol = XEC_I2C_NL_PROTO_WR_RD;
	data->write_phase_buf = NULL;
	data->write_phase_len = 0;
	data->write_phase_buf2 = NULL;
	data->write_phase_len2 = 0;
	data->read_phase_buf = mrd->buf;
	data->read_phase_len = mrd->len;

	dma_blk_len = 1u;    /* 1-byte I2C address */
	ntx = mwr->len + 2u; /* add START and RPT-START addresses */
	nrx = mrd->len;
	data->wrcnt = (uint16_t)ntx;
	data->rdcnt = (uint16_t)nrx;

	data->hflags = BIT(0) | BIT(1) | BIT(2); /* START0 | STARTN | STOP */

	i2c_wr_addr = (data->i2c_addr & 0x7Fu) << 1;
	*xfrbuf++ = i2c_wr_addr;

	tx_buf_max_len = devcfg->xfrbuf_sz - XEC_I2C_NL_XFRBUF_PAD_SZ;
	cplen = mwr->len;

	if (cplen > tx_buf_max_len) {
		cplen = tx_buf_max_len;
		rem = mwr->len - tx_buf_max_len;
		data->write_phase_buf = mwr->buf + cplen;
		data->write_phase_len = rem;
	}

	memcpy((void *)xfrbuf, mwr->buf, cplen);
	xfrbuf += cplen;
	dma_blk_len += cplen;

	if (rem != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x18);
		devcfg->xfrbuf[tx_buf_max_len] = i2c_wr_addr | BIT(0);
		data->write_phase_buf2 = (uint8_t *)&devcfg->xfrbuf[tx_buf_max_len];
		data->write_phase_len2 = 1u;
	} else {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x19);
		*xfrbuf++ = i2c_wr_addr | BIT(0);
		dma_blk_len += 1u; /* RPT-START address at end of msg */
	}

	xfr_cfg_set_atomic_flags(dev);

	xec_i2c_nl_dma_cfg_init(dev, &data->dma_cfg, dma_flags);
	xec_i2c_nl_dma_cfg_block(dev, &data->dma_blk_cfg, (void *)devcfg->xfrbuf, dma_blk_len,
				 dma_flags);

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x1A);

	return 0;
}

static int xfr_cfg_wr_wr(const struct device *dev, struct i2c_msg *mwr1, struct i2c_msg *mwr2)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	volatile uint8_t *xfrbuf = devcfg->xfrbuf;
	uint32_t dma_flags = XEC_I2C_NL_DMA_CFG_HC | XEC_I2C_NL_DMA_CFG_M2D;
	uint32_t ntx = 0, buf_max_len = 0, cplen = 0, rem = 0, dma_blk_len = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x1B);

	data->protocol = XEC_I2C_NL_PROTO_WR_WR;
	data->write_phase_buf = NULL;
	data->write_phase_len = 0;
	data->read_phase_buf = NULL;
	data->read_phase_len = 0;

	dma_blk_len = 1u; /* 1-byte address */
	*xfrbuf++ = (data->i2c_addr & 0x7Fu) << 1;
	buf_max_len = devcfg->xfrbuf_sz - XEC_I2C_NL_XFRBUF_PAD_SZ;

	cplen = mwr1->len;
	if (cplen > buf_max_len) {
		cplen = buf_max_len;
		rem = mwr1->len - buf_max_len;
	}

	memcpy((void *)xfrbuf, mwr1->buf, cplen);
	xfrbuf += cplen;
	dma_blk_len += cplen;

	if (rem != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x1C);
		data->write_phase_buf = mwr1->buf + cplen;
		data->write_phase_len = rem;
		data->write_phase_buf2 = mwr2->buf;
		data->write_phase_len2 = mwr2->len;
	} else {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x1D);
		rem = buf_max_len - cplen;
		memcpy((void *)xfrbuf, mwr2->buf, rem);
		xfrbuf += rem;
		dma_blk_len += rem;
		data->write_phase_buf = mwr2->buf + rem;
		data->write_phase_len = mwr2->len - rem;
	}

	xfr_cfg_set_atomic_flags(dev);

	data->num_msgs_xfr = 2u;
	ntx = mwr1->len + mwr2->len + 1u; /* add START address */
	data->wrcnt = (uint16_t)ntx;
	data->rdcnt = 0;

	data->hflags = BIT(0) | BIT(2); /* START0 | STOP */

	xec_i2c_nl_dma_cfg_init(dev, &data->dma_cfg, dma_flags);
	xec_i2c_nl_dma_cfg_block(dev, &data->dma_blk_cfg, (void *)devcfg->xfrbuf, dma_blk_len,
				 dma_flags);

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x1E);

	return 0;
}

#define XEC_I2C_NL_HM_CFG_FLUSH BIT(XEC_I2C_CFG_FHTX_POS) | BIT(XEC_I2C_CFG_FHRX_POS)
/* #define XEC_I2C_NL_HM_CFG_IEN   BIT(XEC_I2C_CFG_HD_IEN_POS) | BIT(XEC_I2C_CFG_STD_NL_IEN_POS) */
#define XEC_I2C_NL_HM_CFG_IEN   BIT(XEC_I2C_CFG_HD_IEN_POS)
#define XEC_I2C_NL_TM_CFG_FLUSH BIT(XEC_I2C_CFG_FTTX_POS) | BIT(XEC_I2C_CFG_FTRX_POS)
#define XEC_I2C_NL_TM_CFG_IEN   BIT(XEC_I2C_CFG_TD_IEN_POS) | BIT(XEC_I2C_CFG_STD_NL_IEN_POS)

#define TRIG_ORIG 0
#define TRIG_OPT1 1
#define TRIG_OPT2 2
#define TRIG_OPT3 3
#define TRIG_OPT4 4

#define TRIG_OPTION TRIG_ORIG

static int xec_i2c_nl_xfr_trigger(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	mem_addr_t rb = devcfg->i2c_base;
	uint32_t cfg_ien = XEC_I2C_NL_HM_CFG_IEN;
	uint32_t cmd = 0, extlen = 0;
#if TRIG_OPTION == TRIG_OPT1
	unsigned int lock = 0;
#endif
	int rc = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x20);

	soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 0);
	sys_write32(0, rb + XEC_I2C_HCMD_OFS);
	sys_write32(0, rb + XEC_I2C_ELEN_OFS);
	sys_clear_bits(rb + XEC_I2C_CFG_OFS, cfg_ien);
	sys_write32(1u, rb + XEC_I2C_WKSR_OFS);
	sys_set_bits(rb + XEC_I2C_CFG_OFS, XEC_I2C_NL_HM_CFG_FLUSH);
	sys_write32(XEC_I2C_CMPL_RW1C_MSK, rb + XEC_I2C_CMPL_OFS);
	soc_ecia_girq_status_clear(devcfg->girq, devcfg->girq_pos);

	/* configure DMA channel */
	rc = dma_config(devcfg->hc_dma_dev, devcfg->hc_dma_chan, &data->dma_cfg);
	if (rc != 0) {
		LOG_ERR("I2C-NL xfr trigger DMA config error (%d)", rc);
		return rc;
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x21);

	soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 1u);
	dma_start(devcfg->hc_dma_dev, devcfg->hc_dma_chan);

	cmd = data->hflags;
	cmd <<= XEC_I2C_HCMD_START0_POS;
	cmd |= BIT(XEC_I2C_HCMD_RUN_POS) | BIT(XEC_I2C_HCMD_PROC_POS);
	cmd |= XEC_I2C_HCMD_WCL_SET((uint32_t)data->wrcnt);
	cmd |= XEC_I2C_HCMD_RCL_SET((uint32_t)data->rdcnt);
	extlen = (data->rdcnt & 0xff00u) + ((data->wrcnt >> 8) & 0xffu);
	sys_write32(extlen, rb + XEC_I2C_ELEN_OFS);

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x22);
	data->htcmd = cmd;
#if TRIG_OPTION == TRIG_ORIG /* DOES NOT WORK. WORKS adding clear of HDONE_IEN bit in ISR handler  \
			      */
	sys_set_bits(rb + XEC_I2C_CFG_OFS, cfg_ien);
	/*	__ISB(); */
	/*	__DMB(); */
	sys_write32(cmd, rb + XEC_I2C_HCMD_OFS); /* triggers HW */
/*	__ISB(); */
/*	__DMB();
	data->htcmd = sys_read32(rb + XEC_I2C_HCMD_OFS);
	__ISB(); */
#elif TRIG_OPTION == TRIG_OPT1 /* WORKED */
	lock = irq_lock();
	sys_write32(cmd, rb + XEC_I2C_HCMD_OFS); /* triggers HW */
	sys_set_bits(rb + XEC_I2C_CFG_OFS, cfg_ien);
	irq_unlock(lock);
#elif TRIG_OPTION == TRIG_OPT2 /* DOES NOT WORK */
	atomic_set_bit((atomic_t *)(rb + XEC_I2C_CFG_OFS), XEC_I2C_CFG_HD_IEN_POS);
	atomic_set((atomic_t *)(rb + XEC_I2C_HCMD_OFS), (atomic_val_t)cmd);
#elif TRIG_OPTION == TRIG_OPT3 /* DOES NOT WORK */
	atomic_set((atomic_t *)(rb + XEC_I2C_HCMD_OFS), (atomic_val_t)cmd);
	atomic_set_bit((atomic_t *)(rb + XEC_I2C_CFG_OFS), XEC_I2C_CFG_HD_IEN_POS);
#elif TRIG_OPTION == TRIG_OPT4 /* WORKED */
	sys_set_bits(rb + XEC_I2C_CFG_OFS, cfg_ien);
	sys_write32(cmd, rb + XEC_I2C_HCMD_OFS);
	sys_write8(0x80u, rb + XEC_I2C_BBCR_OFS);
#else
#error "I2C-NL NO TRIGGER OPTION DEFINED!"
#endif
	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x23);

	return 0;
}

static int xec_i2c_nl_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			       uint16_t addr)
{
	struct xec_i2c_nl_data *const data = dev->data;
#if 0
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct soc_xec_dma_chan_cfg *dma_cfgs = &data->dcfg[0];
	mem_addr_t i2c_base = devcfg->i2c_base
#endif
	struct i2c_msg *m = NULL;
	struct i2c_msg *mnext = NULL;
	int rc = 0;

	/* TODO check array of struct i2c_msg for NULL ptrs and lengths */
	rc = xec_i2c_nl_msgs_valid(msgs, num_msgs);
	if (rc != 0) {
		LOG_DBG("Found invalid I2C message");
		return rc;
	}

	k_mutex_lock(&data->lock_mut, K_FOREVER);

	XEC_I2C_DEBUG_ISR_INIT(data);
	XEC_I2C_DEBUG_STATE_INIT(data);
	XEC_I2C_DEBUG_STATE_UPDATE(data, 1);

#ifdef CONFIG_I2C_TARGET
	if (xec_i2c_nl_is_tm(dev) == true) {
		k_mutex_unlock(&data->lock_mut);
		return -EBUSY;
	}
#endif
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

	data->i2c_addr = addr;
	data->msgs = msgs;
	data->num_msgs = num_msgs;
	data->midx = 0;
	data->htm_state = 0;
	data->wrcnt = 0;
	data->rdcnt = 0;
	data->htcmd = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 3);

	while (data->midx < data->num_msgs) {
		m = &msgs[data->midx];
		mnext = NULL;

		if ((data->num_msgs - data->midx) > 1u) {
			mnext = m + 1u;
		}

		data->num_msgs_xfr = 0;

		if (is_xfr_one_msg(m, mnext) == true) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 4);
			rc = xfr_cfg_one_msg(dev, m);
		} else if (is_xfr_wr_rd(m, mnext) == true) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 5);
			rc = xfr_cfg_wr_rd(dev, m, mnext);
		} else {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 6);
			rc = xfr_cfg_wr_wr(dev, m, mnext);
		}

		/* do transfer */
		if (rc != 0) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 7);
			LOG_ERR("Message processing error (%d)", rc);
			break;
		}
#if 0
		LOG_DBG("xfr config rc = (%d)", rc);
		LOG_DBG("data msg index = %u", data->midx);
		LOG_DBG("m = %p", m);
		pr_msg(m);
		LOG_DBG("mnext = %p", mnext);
		pr_msg(mnext);
		pr_xfr_data(data);

		LOG_DBG("I2C-NL trigger xfr");
#endif
		k_sem_reset(&data->sync_sem);

		XEC_I2C_DEBUG_STATE_UPDATE(data, 8);
		xec_i2c_nl_xfr_trigger(dev);

		k_sem_take(&data->sync_sem, K_FOREVER);
#if 0
		LOG_DBG("I2C-NL sync_sem taken");
		LOG_DBG("I2C-NL I2C status = 0x%0x", data->i2c_status);
#endif
		XEC_I2C_DEBUG_STATE_UPDATE(data, 9);
		data->midx += data->num_msgs_xfr;
	} /* end for num_msgs */

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xA);
	pm_device_busy_clear(dev);
	k_mutex_unlock(&data->lock_mut);

	return rc;
}

#ifdef CONFIG_I2C_CALLBACK
static int xec_i2c_nl_transfer_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				  uint16_t addr, i2c_callback_t cb, void *userdata)
{
	/* TODO */
	return -ENOTSUP;
}
#endif

#ifdef CONFIG_I2C_RTIO
static int xec_i2c_nl_iodev_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	/* TODO */
	return -ENOTSUP;
}
#endif

/* I2C-NL interrupt helper Host mode */
#ifdef CONFIG_I2C_TARGET

#define I2C_NL_BUF_WR_RECV_CB_POS 0
#define I2C_NL_BUF_RD_REQ_CB_POS  1
#define I2C_NL_BUF_STOP_POS       2

static uint16_t tm_read_count_get(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint16_t rdcnt = 0, temp = 0;

	rdcnt = XEC_I2C_ELEN_TRD_GET(sys_read32(i2c_base + XEC_I2C_ELEN_OFS));
	temp = XEC_I2C_TCMD_RCL_GET(sys_read32(i2c_base + XEC_I2C_TCMD_OFS));
	rdcnt = ((rdcnt << 8) + temp) & 0xffffu;

	return rdcnt;
}

/* TODO
 * Scenario where external Host writes/reads number of bytes >= DMA buffer size.
 * DMA channel asserts DONE triggering an interrupt. DMA driver ISR invokes this
 * callback.
 * If external Host is writing data
 * 	we invoke I2C buf_write_recv callback passing pointer to the buffer this
 * 	DMA channel is using. The length is based on where in the Host I2C write
 * 	protocol we are: if START then byte 0 of buffer contains target address.
 * 	We don't pass that to app. If STOP is present we should not copy data and let
 * 	I2C-NL ISR handle data copy. How to detect this? If START or in the middle and
 * 	no STOP we make callback to app and reload channel. TODO do we reprogram I2C-NL
 * 	rdCnt to original value? This is dangerous due to I2C HW read-ahead ;<(
 * If external Host is reading data.
 * 	We call buf_read_req to get another chunk of data. If buf_read_req returns error
 * 	code we do no reload the channel. Is there anything we can do to force I2C-NL
 * 	to supply 0xFF as data until external Host issues STOP?
 *      If TCMD.wrCnt reaches 0 and Host requests more data then I2C-NL will transmit
 *      the last contents of TM TXB register to the Host and set TPROT status bit in
 * 	the I2C-NL.CMPL register. It may be safe to change I2C-NL TCMD.wrCnt since there
 *      is no read-ahead and I2C-NL is clock-stretching.
 *
 * ISSUE: Peripheral-to-Memory
 * External Host is writing to us and the write fills the buffer causing DMA to stop and
 * make this callback.
 * Host has not issued I2C STOP after byte that caused DMA to stop.
 *   We make buf_write_recv callback to app.
 *   We reload DMA channel causing I2C-NL to stop clock stretching and let Host send more data.
 * Host generated I2C STOP after byte that caused DMA to stop.
 *   We don't make buf_write_recv callback
 *   We don't reload DMA channel
 *   I2C-NL will generate TM_DONE interrupt on STOP. Also, STOP detection will cause
 *   I2C-NL to trigger DMA to read dummy/PEC byte.
 * QUESTION: How can we detect if STOP has occurred from this DMA callback?
 *   1 .external Host generates clocks and data for its last byte tranmitted to us.
 *   2. On the 9th clock I2C-NL ACK's the byte and triggers DMA to read the byte from
 *      I2C-NL TM RXB register.
 *      !! If DMA (memEnd - memStart) <= 1 generate DMA interrupt !!
 *   3. Some time after the 9th clock external Host generates STOP
 *   4. I2C-NL detects STOP, moves PEC bytes to I2C-NL TM RXB and triggers DMA to read it.
 *   5. DMA issues bus request to read dummy/PEC byte from I2C-NL.
 *       !! If DMA (memEnd - memStart) <= 1 generate DMA interrupt !!
 *   I2C-NL also clears TCMD.PROC and TCMD.RUN and sets TM_DONE causing an I2C-NL interrupt
 *   How do we solve the race condition between DMA interrupt/callback and I2C-NL interrupt ISR or
 *   kworkq thread?
 *
 * In Target mode the I2C-NL will be clock stretching or "done" when the DMA makes this callback.
 * It is safe to read I2C-NL registers.
 */
i2c_target_buf_read_requested_cb_t get_buf_rd_req(struct i2c_target_config *tcfg)
{
	if ((tcfg == NULL) || (tcfg->callbacks == NULL)) {
		return NULL;
	}

	return tcfg->callbacks->buf_read_requested;
}

i2c_target_buf_write_received_cb_t get_buf_wr_recv(struct i2c_target_config *tcfg)
{
	if ((tcfg == NULL) || (tcfg->callbacks == NULL)) {
		return NULL;
	}

	return tcfg->callbacks->buf_write_received;
}

i2c_target_stop_cb_t get_stop_cb(struct i2c_target_config *tcfg)
{
	if ((tcfg == NULL) || (tcfg->callbacks == NULL)) {
		return NULL;
	}

	return tcfg->callbacks->stop;
}

static void xec_i2c_nl_tc_cb(const struct device *dev, void *user_data, uint32_t chan, int status)
{
	struct xec_i2c_nl_data *data = (struct xec_i2c_nl_data *)user_data;
	const struct device *i2c_dev = data->dev;
	const struct xec_i2c_nl_config *devcfg = i2c_dev->config;
	mem_addr_t i2c_base = devcfg->i2c_base;
	struct i2c_target_config *tcfg = NULL;
	struct dma_status dma_status = {0};
	uint8_t *data_ptr = NULL;
	int rc = 0;
	uint32_t dma_xfr_len = 0;
	uint8_t tm_addr = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xE0);

	dma_get_status(dev, chan, &dma_status);

	tm_addr = sys_read8(i2c_base + XEC_I2C_IAS_OFS);
	tcfg = find_target(data, (tm_addr >> 1));

	if (dma_status.dir == MEMORY_TO_PERIPHERAL) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xE1);
		/* external Host I2C Read and TCMD write-count != 0 the I2C-NL is clock stretching.
		 * invoke buf_read_req callback to get another buffer full of data.
		 */
		i2c_target_buf_read_requested_cb_t fp = get_buf_rd_req(tcfg);

		if (fp != NULL) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0xE2);
			rc = fp(tcfg, &data->buf_req_ptr, &data->buf_req_size);
			if (rc == 0) {
				XEC_I2C_DEBUG_STATE_UPDATE(data, 0xE3);
				dma_reload(dev, chan, (uint32_t)data->buf_req_ptr,
					   i2c_base + XEC_I2C_TTX_OFS, data->buf_req_size);
			}
		}
	} else { /* PERIPHERAL_TO_MEMORY */
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xE4);

		dma_xfr_len = devcfg->xfrbuf_sz - dma_status.pending_length;

		if (dma_xfr_len != 0) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0xE5);
			data_ptr = (uint8_t *)devcfg->xfrbuf;

			if (data->tm_total_rd_cnt == 0) {
				XEC_I2C_DEBUG_STATE_UPDATE(data, 0xE6);
				/* HW has pre-pended target address, decrement length by 1 */
				dma_xfr_len--;
				data_ptr++;
			}

			data->tm_total_rd_cnt += dma_xfr_len;

			i2c_target_buf_write_received_cb_t fp = get_buf_wr_recv(tcfg);

			if (fp != NULL) {
				XEC_I2C_DEBUG_STATE_UPDATE(data, 0xE7);
				fp(tcfg, data_ptr, dma_xfr_len);
			}
		}

		/* reload DMA channel */
		dma_reload(dev, chan, i2c_base + XEC_I2C_TRX_OFS, (uint32_t)devcfg->xfrbuf,
			   devcfg->xfrbuf_sz);
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xEF);
}

/* Handle I2C-NL turn-around
 * external Host I2C Write-Read
 * START wrAddr wrData1, ..., wrDataN, RPT-START rdAddr -> TM_DONE interrupt with PROC:RUN=01b
 * Software should invoke buf_write_received callback passing point and length of wrData.
 * Next, software must reconfigure the DMA channel for dev2mem direction, start DMA, and set
 * I2C-NL TCMD PROC:RUN=11b. I2C-NL will release SCL allowing the external Host to issue
 * clocks. I2C-NL will trigger DMA to supply data from memory to be consumed by external Host.
 * At turn-around I2C-NL is clock stretching and is not asserting signal to trigger
 * TM DMA channel. We can use DMA get status API to determine the amount of data
 * the external Host transmitted to us during the write phase.
 * Had to hack DMA driver to get good data for case where DMA ISR does not fire.
 * Do not know if this broke DMA driver when multiple DMA blocks are transferred!
 * !!!! The I2C-NL 0 PEC-disabled byte appeared again. This time I2C-NL triggers DMA
 * to read PEC or 0(PEC disabled) byte after external Host transmits RPT-START address.
 * For I2C write-read nwrite = 2 bytes and nread = 4 bytes our HW triggers DMA for 5 bytes.
 * xfrbuf = wrAddr, data1, data2, rdAddr, 0x00
 * Hacked DMA driver returns: dma_stat.pending_length = 0x7, dma_stat.total_copied = 0x5
 * !!!!!!!! THIS MAY BE SIMILAR ISSUE TO HOST MODE DONE IEN TOGGLE !!!!!!!!!!!!!!
 * In Host mode we observed first transaction worked and driver was leaving HDONE_IEN=1
 * in the I2C-NL.CONFIG register. All status was clear.
 * The next Host mode transaction would not work. No I2C SCL/SDA line change. If one then used
 * the debugger to halt and look at I2C-NL registers everything was correct for starting the
 * transfer. Using the debugger to write the same value to HCMD or BBCR or some other registers
 * would then trigger the HW to begin the transaction (generate START and clock out the address).
 * Could a similar issue exist with TDONE_IEN in I2C.CONFIG registers? We leave it enabled
 * through the target transfer and use the GIRQ enable bit to disable interrupts.
 *
 */
static void tm_fsm_turn_around(const struct device *dev)
{
	struct xec_i2c_nl_data *data = dev->data;
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t i2c_base = devcfg->i2c_base;
	struct dma_status dma_stat = {0};
	uint8_t *dataptr = NULL;
	uint32_t ext_host_wr_cnt = 0;
	int rc = 0;
	uint8_t dma_flags = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xC0);

	data->buf_req_ptr = NULL;
	data->buf_req_size = 0;

	dma_get_status(devcfg->tc_dma_dev, devcfg->tc_dma_chan, &dma_stat);

	ext_host_wr_cnt = (uint32_t)dma_stat.total_copied;
	if (ext_host_wr_cnt != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xC1);
		dataptr = (uint8_t *)devcfg->xfrbuf;
		ext_host_wr_cnt--; /* remove post-pended PEC byte */
		if (data->tm_total_rd_cnt == 0) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0xC2);
			ext_host_wr_cnt--; /* remove pre-pended write address */
			dataptr++;
		}

		if (ext_host_wr_cnt != 0) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0xC3);
			i2c_target_buf_write_received_cb_t fp_wr =
				get_buf_wr_recv(data->curr_target);

			if (fp_wr != NULL) {
				XEC_I2C_DEBUG_STATE_UPDATE(data, 0xC4);
				fp_wr(data->curr_target, dataptr, ext_host_wr_cnt);
			}
		}
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xC5);

	i2c_target_buf_read_requested_cb_t fp_rr = get_buf_rd_req(data->curr_target);

	rc = -EINVAL;
	if (fp_rr != NULL) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xC6);
		rc = fp_rr(data->curr_target, &data->buf_req_ptr, &data->buf_req_size);
	}

	if (rc == 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xC7);
		dma_flags = XEC_I2C_NL_DMA_CFG_TC | XEC_I2C_NL_DMA_CFG_M2D;
		xec_i2c_nl_dma_cfg_init(dev, &data->dma_cfg, dma_flags);
		xec_i2c_nl_dma_cfg_block(dev, &data->dma_blk_cfg, (void *)data->buf_req_ptr,
					 data->buf_req_size, dma_flags);

		dma_config(devcfg->tc_dma_dev, devcfg->tc_dma_chan, &data->dma_cfg);
		dma_start(devcfg->tc_dma_dev, devcfg->tc_dma_chan);
	} else {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xC8);
		/* no callback or app callback says ignore remaing transfer until STOP.
		 * point DMA to driver data object with DMA no src increment flag
		 * OR we could try setting TCMD.wrCntLSB = 0 and EXTLEN.wrCntMSB = 0
		 * If the Host keeps reading, I2C-NL will return the last byte read.
		 */
		dma_stop(devcfg->tc_dma_dev, devcfg->tc_dma_chan);
		sys_clear_bits(i2c_base + XEC_I2C_ELEN_OFS, XEC_I2C_ELEN_TWR_MSK);
		sys_clear_bits(i2c_base + XEC_I2C_TCMD_OFS, XEC_I2C_TCMD_WCL_MSK);
	}

	sys_set_bit(i2c_base + XEC_I2C_TCMD_OFS, XEC_I2C_TCMD_PROC_POS);
	soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 1u);

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xCF);
}

static void tm_fsm_done(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	uint8_t *dataptr = (uint8_t *)devcfg->xfrbuf;
	uint32_t nbytes = 0;
	uint16_t rdcnt = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xD0);

	if ((data->tm_addr & BIT(0)) == 0) { /* I2C write */
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xD1);

		i2c_target_buf_write_received_cb_t fp_wr = get_buf_wr_recv(data->curr_target);

		if (fp_wr != NULL) {
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0xD2);

			rdcnt = tm_read_count_get(dev);

			if (rdcnt < data->tm_rd_cnt) {
				XEC_I2C_DEBUG_STATE_UPDATE(data, 0xD3);
				nbytes = data->tm_rd_cnt - rdcnt;
				nbytes--;                        /* remove 0 byte appended by HW */
				nbytes -= data->tm_total_rd_cnt; /* passed by DMA CB */
				if (data->tm_total_rd_cnt == 0) {
					XEC_I2C_DEBUG_STATE_UPDATE(data, 0xD4);
					nbytes--; /* remove address prepended by HW */
					dataptr++;
				}

				if (nbytes != 0) {
					XEC_I2C_DEBUG_STATE_UPDATE(data, 0xD5);
					fp_wr(data->curr_target, dataptr, nbytes);
				}
			}
		}
	} else { /* I2C Read */
		 /* TODO do we anything when at TM_DONE for read phase?
		  * DMA has finished and last byte has been clocked out.
		  * External Host has issued STOP.
		  * Question: do we get TM_DONE due to STOP or is
		  * there another trigger.  DMA has signalled done
		  * but I2C-NL write count may be > 0.
		  */
	}

	i2c_target_stop_cb_t fp_stop = get_stop_cb(data->curr_target);

	if (fp_stop != NULL) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xDE);
		fp_stop(data->curr_target);
	}

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xDF);
}

/* Entered if I2C-NL HW has set TM_DONE status bit
 * Observations:
 * Host I2C writes to this target data len < our buffer size.
 *  I2C-NL detects START clocks in address if not matching it ignores it. No status.
 *  I2C_NL detects START clocks in matching address, generates ACK on 9th clock and
 *  triggers target mode DMA channel.
 *  TM DMA reads byte from TM RX buffer register and stores to memory.
 *  I2C-NL decrements the readCount. TCMD register contains readCountLSB and
 *  EXTLEN register contains readCountMSB.
 *  TM DMA read causes I2C-NL to release SCL allowing Host to send next byte.
 *  Repeat
 *  Host generates STOP
 *  I2C-NL triggers DMA to read PEC byte from I2C-NL TM RX buffer and store to memory.
 *  I2C-NL does this even if PEC is disabled.
 *  Memory buffer contains: target-addr, data0, data1, ..., dataN, 0x00
 *  I2C-NL sets TM_DONE status in Completion register. If corresponding TM_DONE_IEN is
 *  set in the Configuration register an interrupt is generated.
 *  ISR should see TCMD PROC:RUN bits[1:0]=00b
 *
 * Scenarios to test:
 * external Host writes < buffer size
 *   this could also be Host write of read address
 * external Host write == buffer size
 * external Host writes > buffer size
 *
 * TODO is there a way we can detect first time in for target?
 * meaning START + targ_addr?
 * The AAT bit should tell us.
 */
static void xec_i2c_nl_tm_handler(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint32_t tcmd = sys_read32(i2c_base + XEC_I2C_TCMD_OFS);

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xA1);

	/* This will not work because I2C-NL has triggered DMA to read
	 * on reception and match of target address in I2C.DATA register.
	 * I2C-NL HW FSM moves matched address from I2C.DATA to I2C-NL.RXB register.
	 * This has two side effects on HW:
	 * 1. I2C.SR.AAT bit is cleared
	 * 2. I2C releases SCL allowing external Host to write first data byte.
	 * I2C-NL will only signal TM_DONE status during a Host I2C write for:
	 *   1. Host generates STOP.
	 *   2. I2C-NL TCMD read count is decremented to 0.
	 *   3. Lost arbitration due to a different external Host acquiring the bus.
	 *   4. Bus Error.
	 * TM DMA channel will signal DONE via its callback if the amount of data
	 * fills its buffer. This could occur on the same byte if the Host transmits
	 * a number of bytes equal to the driver xfrbuf size. Or if the Host transmits
	 * more bytes and DMA reaches DONE. For this case I2C-NL will clock stretch.
	 * DMA callback must invoke write received callback and when cb returns reload
	 * the DMA channel. DMA callback should not reprogram I2C-NL because that could
	 * cause I2C-NL HW FSM to get out of sync. This gets complicated.
	 */
	if (data->curr_target == NULL) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xA2);
		data->tm_addr = sys_read8(i2c_base + XEC_I2C_IAS_OFS);
		data->curr_target = find_target(data, (data->tm_addr >> 1));
	}

	if ((tcmd & 0x03u) == 0x01u) { /* HW pause state? */
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xA3);
		/* external Host generated RPT-START */
		data->htm_state = 0xA3u;
		tm_fsm_turn_around(dev);
	} else if ((tcmd & 0x03u) == 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xA5);
		data->htm_state = 0xA5u;
		tm_fsm_done(dev);
		xec_i2c_nl_tm_cfg1(dev);
	} else {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0xAD);
		/* can we get here if PROC:RUN==11b */
		data->htm_state = 0xADu;
	}

	sys_set_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_TD_IEN_POS);

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0xAF);
}
#endif

/* Called from either ISR or kworkqueue thread. I2C controller interrupt
 * has been disabled via its GIRQ bit.
 * If called from a kernel thread and I2C interrupt is re-enabled and if
 * the interrupt fires it will reschedule the thread to run again after
 * the thread completes.
 */
static void xec_i2c_nl_handler(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint32_t dma_flags = XEC_I2C_NL_DMA_CFG_HC | XEC_I2C_NL_DMA_CFG_D2M;
	uint32_t hcmd = 0;

	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x80);

#ifdef CONFIG_I2C_TARGET
	if ((data->i2c_status & BIT(XEC_I2C_CMPL_TDONE_POS)) != 0) {
		xec_i2c_nl_tm_handler(dev);
		goto xec_i2c_nl_handler_exit;
	}
#endif

	if ((data->i2c_status & (BIT(XEC_I2C_SR_BER_POS) | BIT(XEC_I2C_SR_LAB_POS) |
				 BIT(XEC_I2C_CMPL_HNAKX_POS))) != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x81);
		k_sem_give(&data->sync_sem);
		return;
	}

	if (((data->i2c_cfg_reg & BIT(XEC_I2C_CFG_IDLE_IEN_POS)) != 0) &&
	    ((data->i2c_status & BIT(XEC_I2C_CMPL_IDLE_POS)) != 0)) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x83);
		k_sem_give(&data->sync_sem);
		goto xec_i2c_nl_handler_exit;
	}

	if ((data->i2c_status & BIT(XEC_I2C_SR_STO_POS)) != 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x84);
		k_sem_give(&data->sync_sem);
		goto xec_i2c_nl_handler_exit;
	}

	hcmd = sys_read32(i2c_base + XEC_I2C_HCMD_OFS);
	if ((hcmd & 0x03u) == 0) {
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x85);
		sys_set_bits(i2c_base + XEC_I2C_CFG_OFS,
			     BIT(XEC_I2C_CFG_FHRX_POS) | BIT(XEC_I2C_CFG_FHTX_POS));
		k_sem_give(&data->sync_sem);
	} else if ((hcmd & 0x03u) == 0x01u) { /* turn-around for read phase? */
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x86);
		if (atomic_test_and_clear_bit(data->aflags, XEC_AFLAG_RD_PH_BUF_POS) == true) {
			xec_i2c_nl_dma_cfg_init(dev, &data->dma_cfg, dma_flags);
			xec_i2c_nl_dma_cfg_block(dev, &data->dma_blk_cfg,
						 (void *)data->read_phase_buf, data->read_phase_len,
						 dma_flags);
			dma_config(devcfg->hc_dma_dev, devcfg->hc_dma_chan, &data->dma_cfg);
			dma_start(devcfg->hc_dma_dev, devcfg->hc_dma_chan);
			sys_write32(hcmd | BIT(XEC_I2C_HCMD_PROC_POS), i2c_base + XEC_I2C_HCMD_OFS);
			sys_set_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_HD_IEN_POS);
		} else {
			/* TODO should not happen! */
			XEC_I2C_DEBUG_STATE_UPDATE(data, 0x87);
		}
	} else {
		/* TODO can we get an interrupt with bits[1:0]=11b ? */
		XEC_I2C_DEBUG_STATE_UPDATE(data, 0x88);
	}

xec_i2c_nl_handler_exit:
	XEC_I2C_DEBUG_STATE_UPDATE(data, 0x8F);
}

#ifdef CONFIG_I2C_XEC_NL_USE_KWORKQUEUE
static void xec_i2c_nl_kworker(struct k_work *work)
{
	struct xec_i2c_nl_data *data = CONTAINER_OF(work, struct xec_i2c_nl_data, kwq);
	const struct device *dev = data->dev;

	xec_i2c_nl_handler(dev);
}
#endif /* CONFIG_I2C_XEC_NL_USE_KWORKQUEUE */

/* I2C-NL interrupt handler */
static void xec_i2c_nl_isr(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t i2c_base = devcfg->i2c_base;
	struct xec_i2c_nl_data *data = dev->data;

	XEC_I2C_DEBUG_STATE_UPDATE((struct xec_i2c_nl_data *)dev->data, 0x70);

#ifdef XEC_I2C_NL_DEBUG_ISR
	data->isr_count++;
	data->hcmd = sys_read32(i2c_base + XEC_I2C_HCMD_OFS);
	data->tcmd = sys_read32(i2c_base + XEC_I2C_TCMD_OFS);
#endif
	data->i2c_status = xec_i2c_status_get(i2c_base);
	data->i2c_cfg_reg = sys_read32(i2c_base + XEC_I2C_CFG_OFS);
	sys_write32(data->i2c_cfg_reg & 0x00ffffffu, i2c_base + XEC_I2C_CFG_OFS);
	sys_write32(XEC_I2C_CMPL_RW1C_MSK, i2c_base + XEC_I2C_CMPL_OFS);
	soc_ecia_girq_status_clear(devcfg->girq, devcfg->girq_pos);

#ifdef CONFIG_I2C_XEC_NL_USE_KWORKQUEUE
	XEC_I2C_DEBUG_STATE_UPDATE((struct xec_i2c_nl_data *)dev->data, 0x71);
	k_work_submit(&data->kwq);
#else
	xec_i2c_nl_handler(dev);
#endif
	XEC_I2C_DEBUG_STATE_UPDATE((struct xec_i2c_nl_data *)dev->data, 0x7F);
}

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
		sys_clear_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);
		break;
	case PM_DEVICE_ACTION_RESUME:
		sys_set_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif

/* Driver initialization */
static int xec_i2c_nl_dma_init(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	int rc = 0, requested_chan = 0;

	if (device_is_ready(devcfg->hc_dma_dev) == false) {
		LOG_ERR("Host mode DMA driver not ready!");
		return -ENODEV;
	}

	requested_chan = (int)devcfg->hc_dma_chan;
	rc = dma_request_channel(devcfg->hc_dma_dev, (void *)&requested_chan);
	if ((rc < 0) || (rc != requested_chan)) {
		LOG_ERR("Requested chan (%u) got (%d)", devcfg->hc_dma_chan, rc);
		return -EAGAIN;
	}

#ifdef CONFIG_I2C_TARGET
	if (device_is_ready(devcfg->tc_dma_dev) == false) {
		LOG_ERR("Target mode DMA driver not ready");
		return -ENODEV;
	}

	requested_chan = (int)devcfg->tc_dma_chan;
	rc = dma_request_channel(devcfg->tc_dma_dev, (void *)&requested_chan);
	if ((rc < 0) || (rc != requested_chan)) {
		LOG_ERR("Requested chan (%u) got (%d)", devcfg->tc_dma_chan, rc);
		return -EAGAIN;
	}
#endif
	xec_i2c_nl_dma_cfg_init(dev, &data->dma_cfg,
				(XEC_I2C_NL_DMA_CFG_HC | XEC_I2C_NL_DMA_CFG_M2D));

	return 0;
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
	sys_slist_init(&data->target_list);
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
	.iodev_submit xec_i2c_nl_iodev_submit,
#endif
	.recover_bus = xec_i2c_nl_recover_bus,
};

#define XEC_I2C_GIRQ_DT(inst)     MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, 0))
#define XEC_I2C_GIRQ_POS_DT(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, 0))

#define XEC_I2C_NL_DMA_NODE(i, name)    DT_INST_DMAS_CTLR_BY_NAME(i, name)
#define XEC_I2C_NL_DMA_DEVICE(i, name)  DEVICE_DT_GET(XEC_I2C_NL_DMA_NODE(i, name))
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
#define XEC_I2C_NL_TM_DMA_INFO(inst)                                                               \
	.tc_dma_dev = XEC_I2C_NL_DMA_DEVICE(inst, target_mode),                                    \
	.tc_dma_chan = XEC_I2C_NL_DMA_CHAN(inst, target_mode),                                     \
	.tc_dma_trigsrc = XEC_I2C_NL_DMA_TRIGSRC(inst, target_mode),
#else
#define XEC_I2C_NL_TM_DMA_INFO(inst)
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
	}                                                                                          \
	static const struct xec_i2c_nl_config xec_i2c_nl##inst##_cfg = {                           \
		.i2c_base = (mem_addr_t)DT_INST_REG_ADDR(inst),                                    \
		.bitrate = DT_INST_PROP_OR(inst, clock_frequency, I2C_BITRATE_STANDARD),           \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
		.irq_config = xec_i2c_nl##inst##_irq_config,                                       \
		.hc_dma_dev = XEC_I2C_NL_DMA_DEVICE(inst, host_mode),                              \
		.hc_dma_chan = XEC_I2C_NL_DMA_CHAN(inst, host_mode),                               \
		.hc_dma_trigsrc = XEC_I2C_NL_DMA_TRIGSRC(inst, host_mode),                         \
		.girq = XEC_I2C_GIRQ_DT(inst),                                                     \
		.girq_pos = XEC_I2C_GIRQ_POS_DT(inst),                                             \
		.pcr = DT_INST_PROP(inst, pcr),                                                    \
		.port = DT_INST_PROP(inst, port_sel),                                              \
		.xfrbuf_sz = XEC_I2C_NL_XFRBUF_SIZE(inst),                                         \
		.xfrbuf = xec_i2c_nl##inst##_xfrbuf,                                               \
		XEC_I2C_NL_TM_DMA_INFO(inst)};                                                     \
	PM_DEVICE_DT_INST_DEFINE(inst, i2c_xec_nl_pm_action);                                      \
	I2C_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_init, PM_DEVICE_DT_INST_GET(inst),              \
				  &xec_i2c_nl##inst##_data, &xec_i2c_nl##inst##_cfg, POST_KERNEL,  \
				  CONFIG_I2C_INIT_PRIORITY, &xec_i2c_nl_driver_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_DEVICE)
