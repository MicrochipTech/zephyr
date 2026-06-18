/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip XEC I2Cv3 using network layer hardware plus DMA.
 *
 * Controller supports 7-bit I2C addresses only.
 * Network layer can only use DMA to move both I2C address and
 * data between memory and the I2C hardware.
 * Network layer transmit memory buffer layout:
 * offset        description
 * 0             target write address
 * 1             data byte 0
 * ...
 * N             data byte N-1
 * N+1           target read address if protocol is write-read
 * I2C-NL HW fires DONE interrupt and clear CMD register PROCEED bit.
 * Driver reconfigures DMA for peripheral to memory targeting
 * application I2C message buffers.
 *
 */

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_xec_i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/clock/mchp_xec_pcr.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/dt-bindings/i2c/mchp-xec-i2c.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>

LOG_MODULE_REGISTER(i2c_mchp_xec_v3_nl, CONFIG_I2C_LOG_LEVEL);

#include "i2c_mchp_xec_regs.h"

#if 0
/* Register Offsets */
#define I2C_CMD_REG(base)      ((base) + 0x0C)
#define I2C_STATUS_REG(base)   ((base) + 0x20)
#define I2C_CONFIG_REG(base)   ((base) + 0x28)
#define I2C_TX_DATA_REG(base)  ((base) + 0x54)
#define I2C_RX_DATA_REG(base)  ((base) + 0x58)

/* Status Register Bitmasks */
#define STATUS_HDONE           BIT(30)
#define STATUS_HNAK            BIT(24)
#define STATUS_LAB             BIT(14)
#define STATUS_BER             BIT(13)

/* Command Register Bitmasks */
#define CMD_RUN                BIT(0)
#define CMD_PROCEED            BIT(1)
#define CMD_START0             BIT(8)
#define CMD_STARTN             BIT(9)
#define CMD_STOP               BIT(10)
#endif

#define STATUS_HDONE BIT(XEC_I2C_CMPL_HDONE_POS)
#define STATUS_HNAK  BIT(XEC_I2C_CMPL_HNAKX_POS)
#define STATUS_LAB   BIT(XEC_I2C_CMPL_LAB_STS_POS)
#define STATUS_BER   BIT(XEC_I2C_CMPL_BER_STS_POS)

#define CMPL_REG_ERR_MSK (STATUS_HNAK | STATUS_LAB | STATUS_BER)

/* Host Command Register Bitmasks */
#define HCMD_RUN                BIT(XEC_I2C_HCMD_RUN_POS)
#define HCMD_PROCEED            BIT(XEC_I2C_HCMD_PROC_POS)
#define HCMD_START0             BIT(XEC_I2C_HCMD_START0_POS)
#define HCMD_STARTN             BIT(XEC_I2C_HCMD_STARTN_POS)
#define HCMD_STOP               BIT(XEC_I2C_HCMD_STOP_POS)

#define XEC_I2C_TIMING_100K_IDX  0
#define XEC_I2C_TIMING_400K_IDX  1U
#define XEC_I2C_TIMING_1000K_IDX 2U
#define XEC_I2C_TIMING_IDX_MAX   3U

#define XEC_I2C_TIMING_DATA_POS 0
#define XEC_I2C_TIMING_IDLE_POS 1
#define XEC_I2C_TIMING_TMO_POS  2
#define XEC_I2C_TIMING_BCLK_POS 3
#define XEC_I2C_TIMING_MRV_POS  4

#define XEC_I2C_TIMING_CFG_BITMAP \
	(BIT(XEC_I2C_TIMING_DATA_POS) | BIT(XEC_I2C_TIMING_IDLE_POS) |\
	 BIT(XEC_I2C_TIMING_TMO_POS) | BIT(XEC_I2C_TIMING_MRV_POS))

/* Controller timing register for default frequencies: 100k, 400k, and 1000k */
struct i2c_xec_timing {
	uint32_t data_timing;
	uint32_t idle_scaling;
	uint32_t timeout_scaling;
	uint16_t bus_clock;
	uint8_t mchp_reg68_val;
};

enum i2c_tx_phase {
	PHASE_1_ADDRESS,
	PHASE_2_BULK,
	PHASE_3_FINAL_RPT_START
};

/* Shared Physical Controller Context (Instantiated at PRE_KERNEL_1) */
struct xec_i2c_nl_config {
	mm_reg_t base;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t enc_pcr;
	uint8_t dflt_port;
	uint32_t dflt_freq;
	size_t bounce_buf_size;
	void (*irq_connect)(void);
	const struct pinctrl_dev_config *pincfg;
	const struct device *dma_dev;
	uint8_t cm_dma_chan;
	uint8_t cm_dma_slot;
};

struct xec_i2c_nl_data {
	const struct device *ctrl;
	struct k_sem lock_sem;
	struct k_sem transfer_sem;
	struct k_sem pause_sem;
	struct k_sem dma_done_sem;
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
	uint8_t active_port;
	uint16_t clkdiv;
	uint32_t active_freq;
};

/* Virtual I2C Port Context (Instantiated at POST_KERNEL) */
struct xec_i2c_nl_virtual_port_config {
	const struct device *parent_ctrl;
	uint32_t port_id;
	uint32_t bitrate;
};

/* Controller standard timing values: 100, 400, and 1000 KHz
 * From Microchip I2C-SMBus controller v3.8 data sheet for 16MHz I2C BAUD clock.
 */
static const struct i2c_xec_timing i2c_xec_timing_tbl[] = {
	{
		.data_timing = 0x0C4D5006U,
		.idle_scaling = 0x01FC01EDU,
		.timeout_scaling = 0x4B9CC2C7U,
		.bus_clock = 0x4F4FU, /* Lo-to-Hi ration = 1 */
		.mchp_reg68_val = 0x5U,
	},
	{
		.data_timing = 0x040A0A06U,
		.idle_scaling = 0x01000050U,
		.timeout_scaling = 0x159CC2C7U,
		.bus_clock = 0x0F17U, /* Lo-to-Hi ratio = 1.53 */
		.mchp_reg68_val = 0x5U,
	},
	{
		.data_timing = 0x04060601U,
		.idle_scaling = 0x01000050U,
		.timeout_scaling = 0x089CC2C7U,
		.bus_clock = 0x0509U, /* Lo-to-Hi ratio = 1.8 */
		.mchp_reg68_val = 0x5U,
	},
};

static inline void i2c_ctrl_set(const struct device *dev, uint8_t crval)
{
	const struct xec_i2c_nl_config *cfg = dev->config;
	struct xec_i2c_nl_data *const xdat = dev->data;

	xdat->i2c_cr_cache = crval;
	sys_write8(cfg->base + XEC_I2C_CR_OFS, crval);
}

static uint32_t compute_bus_clock(uint32_t freqhz)
{
	uint32_t bclk = 0, total_cycles = 0, low_cycles = 0, high_cycles = 0;

	total_cycles = (XEC_I2C_SMB_BAUD_CLK + (freqhz / 2U)) / freqhz;

	if (total_cycles < 2U) {
		total_cycles = 2U;
	}

	if (total_cycles > (UINT8_MAX * 2U)) {
		total_cycles = (UINT8_MAX * 2U);
	}

	if (freqhz < KHZ(100)) {
		/* Use 50% duty cycle. For odd freqhz values make low larger */
		low_cycles = (total_cycles + 1U) / 2U;
		high_cycles = total_cycles - low_cycles;
	} else {
		/* low 60% and high 40% */
		low_cycles = ((total_cycles * 3U) + 2U) / 5U;
		high_cycles = total_cycles - low_cycles;

		if (high_cycles < 1U) {
			high_cycles = 1U;
			low_cycles = total_cycles - high_cycles;
		}

		if (low_cycles < 1U) {
			low_cycles = 1U;
			high_cycles = total_cycles - low_cycles;
		}
	}

	/* clamp */
	if (low_cycles > (UINT8_MAX + 1U)) {
		low_cycles = UINT8_MAX;
	}

	if (high_cycles > (UINT8_MAX + 1U)) {
		high_cycles = UINT8_MAX;
	}

	bclk = (low_cycles - 1U) + ((high_cycles - 1U) << 8U);

	return bclk;
}

static int xec_i2c_compute_clock_dividers(uint32_t freqhz, uint32_t *clkdiv)
{
	uint32_t bclk = 0;
	int rc = 0;

	if ((freqhz == 0) || (freqhz > MHZ(1))) {
		bclk = i2c_xec_timing_tbl[XEC_I2C_TIMING_100K_IDX].bus_clock;
		rc = -ERANGE;
	} else if (freqhz > MHZ(1)) {
		bclk = i2c_xec_timing_tbl[XEC_I2C_TIMING_1000K_IDX].bus_clock;
		rc = -ERANGE;
	} else if (freqhz == I2C_BITRATE_STANDARD) { /* KHZ(100) */
		bclk = i2c_xec_timing_tbl[XEC_I2C_TIMING_100K_IDX].bus_clock;
	} else if (freqhz == I2C_BITRATE_FAST) { /* KHZ(400) */
		bclk = i2c_xec_timing_tbl[XEC_I2C_TIMING_400K_IDX].bus_clock;
	} else if (freqhz == I2C_BITRATE_FAST_PLUS) { /* KHZ(1000) */
		bclk = i2c_xec_timing_tbl[XEC_I2C_TIMING_1000K_IDX].bus_clock;
	} else { /* compute */
		bclk = compute_bus_clock(freqhz);
	}

	*clkdiv = bclk;

	return rc;
}

static void xec_i2c_prog_timing(const struct device *ctrl, uint32_t freqhz, uint16_t mask)
{
	const struct xec_i2c_nl_config *xcfg = ctrl->config;
	mm_reg_t rb = xcfg->base;
	const struct i2c_xec_timing *pt = NULL;

	if (freqhz <= KHZ(100)) {
		pt = &i2c_xec_timing_tbl[0];
	} else if (freqhz <= KHZ(400)) {
		pt = &i2c_xec_timing_tbl[1];
	} else {
		pt = &i2c_xec_timing_tbl[2];
	}

	if ((mask & BIT(XEC_I2C_TIMING_DATA_POS)) != 0) {
		sys_write32(pt->data_timing, rb + XEC_I2C_DT_OFS);
	}

	if ((mask & BIT(XEC_I2C_TIMING_IDLE_POS)) != 0) {
		sys_write32(pt->idle_scaling, rb + XEC_I2C_ISC_OFS);
	}

	if ((mask & BIT(XEC_I2C_TIMING_TMO_POS)) != 0) {
		sys_write32(pt->timeout_scaling, rb + XEC_I2C_TMOUT_SC_OFS);
	}

	if ((mask & BIT(XEC_I2C_TIMING_BCLK_POS)) != 0) {
		sys_write32((uint32_t)pt->bus_clock, rb + XEC_I2C_BCLK_OFS);
	}

	if ((mask & BIT(XEC_I2C_TIMING_MRV_POS)) != 0) {
		sys_write32((uint32_t)pt->mchp_reg68_val, rb + XEC_I2C_MR1_OFS);
	}
}

static int xec_i2c_nl_cfg(const struct device *ctrl, uint32_t freqhz, uint8_t port)
{
	const struct xec_i2c_nl_config *xcfg = ctrl->config;
	struct xec_i2c_nl_data *const xdat = ctrl->data;
	mm_reg_t rb = xcfg->base;
	uint32_t v = 0;
	int rc = 0;

	soc_ecia_girq_ctrl(xcfg->girq, xcfg->girq_pos, MCHP_MEC_ECIA_GIRQ_DIS);
	soc_xec_pcr_reset_en(xcfg->enc_pcr);
	soc_ecia_girq_status_clear(xcfg->girq, xcfg->girq_pos);

	i2c_ctrl_set(ctrl, BIT(XEC_I2C_CR_PIN_POS));

	sys_set_bits(rb + XEC_I2C_CFG_OFS, (BIT(XEC_I2C_CFG_GC_DIS_POS) |
					    BIT(XEC_I2C_CFG_FEN_POS)));

	v = XEC_I2C_CFG_PORT_SET((uint32_t)xdat->active_port);
	soc_mmcr_mask_set(rb + XEC_I2C_CFG_OFS, v, XEC_I2C_CFG_PORT_MSK);

	v = XEC_I2C_BCLK_BAUD16M_100K; /* default 100KHz */
	rc = xec_i2c_compute_clock_dividers(freqhz, &v);
	if (rc != 0) {
		LOG_WRN("Clamped frequency to 100K or 1MHz");
	}

	xdat->clkdiv = (uint16_t)v;
	sys_write32(v, rb + XEC_I2C_BCLK_OFS);

	xec_i2c_prog_timing(ctrl, freqhz, XEC_I2C_TIMING_CFG_BITMAP);

	i2c_ctrl_set(ctrl, (BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) |
			    BIT(XEC_I2C_CR_ACK_POS)));

	sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);

	return 0;
}

/* =========================================================================
 * 1. ASYMMETRIC DMA TRANSMIT CALLBACK CHAIN
 * ========================================================================= */
static void xec_i2c_nl_tx_dma_callback(const struct device *dma_dev, void *user_data,
				       uint32_t channel, int status)
{
	struct xec_i2c_nl_data *const xdat = user_data;

	if (status < 0) {
		xdat->status_error = status;
		k_sem_give(&xdat->transfer_sem);
		return;
	}

	switch (xdat->tx_phase) {
	case PHASE_1_ADDRESS:
		if (xdat->user_bulk_size > 0) {
			xdat->tx_phase = PHASE_2_BULK;
			dma_reload(dma_dev, channel, (uint32_t)xdat->user_bulk_buf,
				   xdat->i2c_tx_reg_addr, xdat->user_bulk_size);
			dma_start(dma_dev, channel);
			break;
		}
		/* Fallthrough seamlessly if payload length was 3 bytes or less */

	case PHASE_2_BULK:
		xdat->tx_phase = PHASE_3_FINAL_RPT_START;
		dma_reload(dma_dev, channel, (uint32_t)xdat->tx_bounce_buf,
			   xdat->i2c_tx_reg_addr, 4); /* Always 4 bytes */
		dma_start(dma_dev, channel);
		break;

	case PHASE_3_FINAL_RPT_START:
		/* Handed off completely to the physical I2C Controller ISR */
		break;
	default:
		break; /* Let I2C ISR handle it */
	}
}

/* Standard Zephyr DMA Reception Callback */
static void xec_i2c_nl_rx_dma_callback(const struct device *dma_dev, void *user_data,
				       uint32_t channel, int status)
{
	struct xec_i2c_nl_data *const xdat = user_data;

	if (status < 0) {
		xdat->status_error = status;
		k_sem_give(&xdat->transfer_sem);
	}
}

/* =========================================================================
 * 2. GLOBAL PHYSICAL I2C CONTROLLER ISR
 * ========================================================================= */


void xec_i2c_nl_ctrl_isr(const struct device *dev)
{
	const struct xec_i2c_nl_config *xcfg = dev->config;
	struct xec_i2c_nl_data *const xdat = dev->data;
	uint32_t rb = xcfg->base;
	uint32_t status = sys_read32(rb + XEC_I2C_CMPL_OFS);
	uint32_t error_mask = CMPL_REG_ERR_MSK;
	uint32_t v = 0;

	/* Catch and parse physical bus errors first */
	if (status & error_mask) {
		if ((status & STATUS_HNAK) != 0) {
			xdat->status_error = -ENXIO;
		} else if ((status & STATUS_LAB) != 0) {
			xdat->status_error = -EAGAIN;
		} else if ((status & STATUS_BER) != 0) {
			xdat->status_error = -EIO;
		}

		v = status & (error_mask | STATUS_HDONE | XEC_I2C_CMPL_RW_MSK);
		sys_write32(v, rb + XEC_I2C_CMPL_OFS);
		dma_stop(xcfg->dma_dev, xcfg->cm_dma_chan);

		k_sem_give(&xdat->pause_sem);
		k_sem_give(&xdat->transfer_sem);
		return;
	}

	if (status & STATUS_HDONE) {
		uint32_t cmd = sys_read32(rb + XEC_I2C_HCMD_OFS);
		sys_write32(STATUS_HDONE, rb + XEC_I2C_CMPL_OFS);

		/* Identify physical hardware PAUSE state */
		if ((cmd & HCMD_RUN) && !(cmd & HCMD_PROCEED) && (xdat->user_bulk_buf != NULL)) {
			xdat->user_bulk_buf = NULL; /* Soft latch clear */
			k_sem_give(&xdat->pause_sem);
		} else {
			k_sem_give(&xdat->transfer_sem);
		}
	}
}

/* =========================================================================
 * 3. CORE SUB-PIPELINE METHODS
 * ========================================================================= */
static int xec_i2c_nl_dma_kickoff_tx(const struct device *parent_ctrl, uint16_t addr,
				     uint32_t write_cnt, uint32_t read_cnt, bool has_read)
{
	const struct xec_i2c_nl_config *xcfg = parent_ctrl->config;
	struct xec_i2c_nl_data *const xdat = parent_ctrl->data;
	mm_reg_t rb = xcfg->base;
	uint32_t cmd = 0, extlen = 0;

	xdat->tx_bounce_buf[0] = (uint8_t)(((addr & 0x7FU) << 1) | 0); /* Target Write Address Flag */
	xdat->i2c_tx_reg_addr = rb + XEC_I2C_HTX_OFS;

	/* Note: C90 onwards zero initializes unreferenced structure members when
	 * the structure is "brace" initialized.
	 */
	struct dma_block_config dma_block = {
		.source_address = (uint32_t)xdat->tx_bounce_buf,
		.dest_address = xdat->i2c_tx_reg_addr,
		.block_size = 1,
	};

	struct dma_config dma_cfg = {
		.channel_direction = MEMORY_TO_PERIPHERAL,
		.source_data_size = 1,
		.dest_data_size = 1,
		.block_count = 1,
		.head_block = &dma_block,
		.dma_callback = xec_i2c_nl_tx_dma_callback,
		.user_data = xdat,
	};

	int ret = dma_config(xcfg->dma_dev, xcfg->cm_dma_chan, &dma_cfg);

	if (ret < 0) {
		return ret;
	}

	dma_start(xcfg->dma_dev, xcfg->cm_dma_chan);

	/* Prime and execute the Live FSM */
	extlen = XEC_I2C_ELEN_HWR_SET(write_cnt >> 8) | XEC_I2C_ELEN_HRD_SET(read_cnt >> 8);
	cmd = XEC_I2C_HCMD_WCL_SET(write_cnt) | XEC_I2C_HCMD_RCL_SET(read_cnt);
	cmd |= HCMD_RUN | HCMD_PROCEED | HCMD_START0 | HCMD_STOP;

	if (has_read) {
		cmd |= HCMD_STARTN;
	}

	sys_write32(cmd, rb + XEC_I2C_HCMD_OFS);

	return 0;
}

static int xec_i2c_nl_execute_read_phase(const struct device *parent_ctrl, struct i2c_msg *rx_msg)
{
	const struct xec_i2c_nl_config *xcfg = parent_ctrl->config;
	struct xec_i2c_nl_data *const xdat = parent_ctrl->data;
	mm_reg_t rb = xcfg->base;

	k_sem_take(&xdat->pause_sem, K_FOREVER);

	if (xdat->status_error) {
		return xdat->status_error;
	}

	dma_stop(xcfg->dma_dev, xcfg->cm_dma_chan);

	struct dma_block_config rx_dma_block = {
		.source_address = rb + XEC_I2C_HRX_OFS,
		.dest_address = (uint32_t)rx_msg->buf,
		.block_size = rx_msg->len,
	};

	struct dma_config rx_dma_cfg = {
		.channel_direction = PERIPHERAL_TO_MEMORY, /* Swap direction */
		.source_data_size = 1,
		.dest_data_size = 1,
		.block_count = 1,
		.head_block = &rx_dma_block,
		.dma_callback = xec_i2c_nl_rx_dma_callback,
		.user_data = xdat,
	};

	int ret = dma_config(xcfg->dma_dev, xcfg->cm_dma_chan, &rx_dma_cfg);

	if (ret < 0) {
		return ret;
	}

	dma_start(xcfg->dma_dev, xcfg->cm_dma_chan);

	/* Release the Hardware FSM out of PAUSE state */
	sys_set_bit(rb + XEC_I2C_HCMD_OFS, XEC_I2C_HCMD_PROC_POS);

	k_sem_take(&xdat->transfer_sem, K_FOREVER);

	return xdat->status_error;
}

/* =========================================================================
 * 4. ZEPHYR STANDARD I2C MASTER API METHOD
 * ========================================================================= */
static int xec_i2c_nl_vport_transfer(const struct device *port_dev, struct i2c_msg *msgs,
				     uint8_t num_msgs, uint16_t addr)
{
	const struct xec_i2c_nl_virtual_port_config *port_cfg = port_dev->config;
	const struct device *parent_ctrl = port_cfg->parent_ctrl;
	struct xec_i2c_nl_data *const xdat = parent_ctrl->data;
	struct i2c_msg *write_msg = NULL;
	struct i2c_msg *read_msg = NULL;

	if (num_msgs == 0 || msgs == NULL) {
		return -EINVAL;
	}

	/* Parse Zephyr message frame targets against hardware limitations */
	for (uint8_t i = 0; i < num_msgs; i++) {
		if ((msgs[i].flags & I2C_MSG_READ) == 0) {
			if (write_msg != NULL) {
				return -ENOTSUP; /* Max 1 contiguous write descriptor block */
			}

			write_msg = &msgs[i];
		} else {
			if (read_msg != NULL || !(msgs[i].flags & I2C_MSG_RESTART)) {
				return -ENOTSUP;
			}

			read_msg = &msgs[i];
		}
	}

	if (write_msg == NULL || write_msg->len < 3) {
		return -EINVAL; /* Core floors required by math */
	}

	/* Calculate Grand Hardware Counter Values */
	uint32_t total_write_count = 1 + write_msg->len; /* Address (1B) + payload */
	uint32_t total_read_count = 0;

	if (read_msg != NULL) {
		total_write_count += 1; /* Pack read address byte into tail */
		total_read_count = read_msg->len;
	}

	/* Acquire multi-port hardware resource lock */
	k_sem_take(&xdat->lock_sem, K_FOREVER);

	/* Apply port switching routing context to hardware */
	// Example: sys_write32(port_cfg->port_id, hw_cfg->base_addr + CUSTOM_MUX_REG);

	xdat->status_error = 0;

	/* Execute Driver Streams */
	xdat->tx_phase = PHASE_1_ADDRESS;
	xdat->user_bulk_size = write_msg->len - 3;
        xdat->user_bulk_buf = write_msg->buf;

	uint32_t final_offset = write_msg->len - 3;

	memcpy(xdat->tx_bounce_buf, &write_msg->buf[final_offset], 3);

	if (read_msg == NULL) {
		int ret = xec_i2c_nl_dma_kickoff_tx(parent_ctrl, addr, total_write_count, 0, false);

		if (ret == 0) {
			k_sem_take(&xdat->transfer_sem, K_FOREVER);
		}
	} else {
		xdat->tx_bounce_buf[3] = (addr << 1) | 1;
		/* Append Read Target Address */

		int ret = xec_i2c_nl_dma_kickoff_tx(parent_ctrl, addr, total_write_count, total_read_count, true);

		if (ret == 0) {
			xec_i2c_nl_execute_read_phase(parent_ctrl, read_msg);
		}
    }

    int final_status = xdat->status_error;

    k_sem_give(&xdat->lock_sem);

    return final_status;
}

static int xec_i2c_nl_vport_configure(const struct device *port_dev, uint32_t dev_config)
{
	const struct xec_i2c_nl_virtual_port_config *vcfg = port_dev->config;
	const struct device *ctrl_dev = vcfg->parent_ctrl;
	const struct xec_i2c_nl_config *xcfg = ctrl_dev->config;
	struct xec_i2c_nl_data *const xdat = ctrl_dev->data;
	int rc = 0;
	uint32_t req_freq = 0;
	uint8_t req_port = XEC_I2C_CFG_MAX_PORT;

	if ((dev_config & I2C_MODE_CONTROLLER) == 0) {
		LOG_ERR("Config CTRL as target not allowed");
		return -EINVAL;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD: /* 100 KHz */
		req_freq = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		req_freq = KHZ(400);
		break;
	case I2C_SPEED_FAST_PLUS:
		req_freq = KHZ(1000);
		break;
	case I2C_SPEED_DT:
		req_freq = xcfg->dflt_freq;
		break;
	default:
		LOG_ERR("Requested config unsupported frequency");
		return -ERANGE;
	}

	req_port = vcfg->port_id;
	if (req_port >= XEC_I2C_CFG_MAX_PORT) {
		LOG_ERR("Requested config unsupported port ID");
		return -ERANGE;
	}

	if ((req_freq == xdat->active_freq) && (req_port == xdat->active_port)) {
		return 0;
	}

	rc = xec_i2c_nl_cfg(ctrl_dev, req_freq, req_port);
	if (rc == 0) {
		xdat->active_freq = req_freq;
		xdat->active_port = req_port;
	}

	return rc;
}

static int xec_i2c_nl_vport_get_config(const struct device *port_dev, uint32_t *dev_config)
{
	const struct xec_i2c_nl_virtual_port_config *vcfg = port_dev->config;
	const struct device *ctrl_dev = vcfg->parent_ctrl;
	const struct xec_i2c_nl_config *xcfg = ctrl_dev->config;
	struct xec_i2c_nl_data *const xdat = ctrl_dev->data;
	uint32_t dcfg = I2C_MODE_CONTROLLER;

	if (dev_config == NULL) {
		return -EINVAL;
	}

	/* TODO should get_config always trigger a port switch?
	 * Or if our virtual port_id matches current HW port we return valid dev_config
	 * otherwise return an error code?
	 */

	*dev_config = dcfg;

	return 0;
}

static const struct i2c_driver_api xec_i2c_nl_vport_api_funcs = {
	.configure = xec_i2c_nl_vport_configure,
	.get_config = xec_i2c_nl_vport_get_config,
	.transfer = xec_i2c_nl_vport_transfer,
	.target_register = NULL,
	.target_unregister = NULL,
	.recover_bus = NULL,
};

/* ==== ZEPHYR KERNEL DRIVER INSTANTIATION CORE ================================================= */
static int xec_i2c_nl_ctrl_init(const struct device *ctrl)
{
	const struct xec_i2c_nl_config *xcfg = ctrl->config;
	struct xec_i2c_nl_data *const xdat = ctrl->data;

	if (!device_is_ready(xcfg->dma_dev)) {
		return -ENODEV;
	}

	k_sem_init(&xdat->lock_sem, 1, 1);
	k_sem_init(&xdat->transfer_sem, 0, 1);
	k_sem_init(&xdat->pause_sem, 0, 1);

	if (xcfg->irq_connect != NULL) {
		xcfg->irq_connect();
	}

	/* Initialize base macro configuration block */
#if 0 /* TODO evaluate if we really need to enable HDONE and the block here */
	sys_write32((1 << 30) | (1 << 10), I2C_CONFIG_REG(config->base_addr));
#endif

	return 0;
}

static int xec_i2c_nl_virtual_port_init(const struct device *port_dev)
{
	const struct xec_i2c_nl_virtual_port_config *port_cfg = port_dev->config;

	if (!device_is_ready(port_cfg->parent_ctrl)) {
		return -ENODEV;
	}

	return 0;
}

/* MACRO GEN LAYER 1: Instantiate Physical Parent Macro Engines (PRE_KERNEL_1) */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT microchip_xec_i2c_v3nl

#define I2C_PARENT_CTRL_INIT(inst) \
	static uint8_t parent_bounce_buf_##inst[DT_INST_PROP(inst, bounce_buffer_size)]; \
	static void i2c_parent_irq_config_##inst(void);			\
	static const struct xec_i2c_nl_config xec_i2c_nl_ctrl_cfg_##inst = { \
		.base_addr = DT_INST_REG_ADDR(inst),			\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR(inst)),	\
		.dma_channel = DT_INST_DMAS_CELL(inst, channel),	\
		.bounce_buf_size = DT_INST_PROP(inst, bounce_buffer_size), \
		.irq_config_func = i2c_parent_irq_config_##inst,	\
	};								\
	static struct xec_i2c_nl_data xec_i2c_nl_ctrl_data_##inst = {	\
		.tx_bounce_buf = parent_bounce_buf_##inst,		\
	};								\
	DEVICE_DT_INST_DEFINE(inst,					\
			      xec_i2c_nl_ctrl_init,			\
			      NULL,					\
			      &xec_i2c_nl_ctrl_data_##inst,		\
			      &xec_i2c_nl_ctrl_cfg_##inst,		\
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, \
			      NULL);					\
	static void i2c_parent_irq_config_##inst(void)			\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), \
			    xec_i2c_nl_ctrl_isr, DEVICE_DT_INST_GET(inst), 0); \
		irq_enable(DT_INST_IRQN(inst));				\
	}

DT_INST_FOREACH_STATUS_OKAY(I2C_PARENT_CTRL_INIT)

/* MACRO GEN LAYER 2: Instantiate Virtual Child Ports (POST_KERNEL) */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT microchip_xec_i2c_v3nl_port

#define I2C_VIRTUAL_PORT_INIT(inst) \
	static const struct xec_i2c_nl_virtual_port_config xec_i2c_nl_port_cfg_##inst = { \
		.parent_ctrl = DEVICE_DT_GET(DT_INST_PHANDLE(inst, controller)), \
		.port_id = DT_INST_PROP(inst, port),			\
		.bitrate = DT_INST_PROP(inst, clock_frequency),		\
	};								\
	I2C_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_virtual_port_init, NULL, NULL, \
				  &xec_i2c_nl_port_cfg_##inst,		\
				  POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, \
				  &xec_i2c_nl_vport_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(I2C_VIRTUAL_PORT_INIT)
