/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip XEC I2Cv3 Network-Layer (NL) I2C driver.
 *
 * The NL hardware FSM drives one full I2C transaction (START to STOP)
 * by pulling bytes from a Microchip DMAC channel and pushing read bytes
 * back to it. Software builds a contiguous TX bounce buffer of the form
 *
 *     [ wr-addr | wr-data... | rd-addr ]
 *
 * (the trailing rd-addr byte is omitted on a write-only transfer),
 * configures the DMA channel for MEMORY_TO_PERIPHERAL targeting the
 * controller's HTX register, and writes the host-command (HCMD) register.
 */
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_xec_i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(i2c_mchp_xec_v3_nl, CONFIG_I2C_LOG_LEVEL);

#include "i2c_mchp_xec_regs.h"

#define CMPL_ERR (BIT(XEC_I2C_CMPL_HNAKX_POS) | BIT(XEC_I2C_CMPL_LAB_STS_POS) |\
		  BIT(XEC_I2C_CMPL_BER_STS_POS))

/* Default I2C control-register value: ESO+ACK+PIN. PIN is also raised at
 * reset to clear any latent PIN-asserted state in the legacy I2C engine.
 */
#define XEC_I2C_NL_CR_DFLT                                                                         \
	(BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS) | BIT(XEC_I2C_CR_PIN_POS))

#define BBCR_SCL_IN BIT(XEC_I2C_BBCR_SCL_IN_POS)
#define BBCR_SDA_IN BIT(XEC_I2C_BBCR_SDA_IN_POS)

#define BBCR_LIVE_RD     BIT(XEC_I2C_BBCR_CM_POS) /* CM=1, BBM_EN=0: I2C-driven, readback live */
#define BBCR_BB_RELEASED BIT(XEC_I2C_BBCR_EN_POS) /* BBM_EN=1, both dirs=input, both released */
/* BBM_EN=1, SCL drive-low, SDA released */
#define BBCR_BB_SCL_LOW (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CD_POS))
/* BBM_EN=1, SDA drive-low, SCL released */
#define BBCR_BB_SDA_LOW (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_DD_POS))

struct xec_i2c_nl_dev_cfg {
	uintptr_t regbase;
	void (*xec_irq_connect)(void);
	const struct device *dma_dev;
	uint8_t dma_chan;
	uint8_t dma_slot;
	uint16_t enc_pcr;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t girq_wk;
	uint8_t girq_wk_pos;
	bool wakeup_source;
};

struct xec_i2c_nl_dev_data {
	struct device *ctrl;
	struct k_sem lock;
	struct k_sem pause_sem;
	struct k_sem done_sem;
	uint32_t active_freq;
	uint8_t active_port;
};

struct xec_i2c_nl_port_dev_cfg {
	const struct device *ctrl;
	const struct pinctrl_dev_config *pincfg;
	uint32_t bitrate;
	uint8_t port;
	bool is_default;
};

struct xec_i2c_nl_timing {
	uint32_t data_timing;
	uint32_t idle_scaling;
	uint32_t timeout_scaling;
	uint16_t bus_clock;
	uint8_t rpt_start_hold_tm;
	uint8_t mr1;
};

static const struct xec_i2c_nl_timing xec_i2c_nl_timing_tbl[] = {
	{ /* 100 kHz, 50/50 duty */
		.data_timing = XEC_I2C_SMB_DATA_TM_100K,
		.idle_scaling = XEC_I2C_SMB_IDLE_SC_100K,
		.timeout_scaling = XEC_I2C_SMB_TMO_SC_100K,
		.bus_clock = XEC_I2C_SMB_BUS_CLK_100K,
		.rpt_start_hold_tm = XEC_I2C_SMB_RSHT_100K,
		.mr1 = XEC_I2C_MR0_TM_BAUD16M,
	},
	{ /* 400 kHz, lo:hi ~ 1.53 */
		.data_timing = XEC_I2C_SMB_DATA_TM_400K,
		.idle_scaling = XEC_I2C_SMB_IDLE_SC_400K,
		.timeout_scaling = XEC_I2C_SMB_TMO_SC_400K,
		.bus_clock = XEC_I2C_SMB_BUS_CLK_400K,
		.rpt_start_hold_tm = XEC_I2C_SMB_RSHT_400K,
		.mr1 = XEC_I2C_MR0_TM_BAUD16M,
	},
	{ /* 1 MHz, lo:hi ~ 1.8 */
		.data_timing = XEC_I2C_SMB_DATA_TM_1M,
		.idle_scaling = XEC_I2C_SMB_IDLE_SC_1M,
		.timeout_scaling = XEC_I2C_SMB_TMO_SC_1M,
		.bus_clock = XEC_I2C_SMB_BUS_CLK_1M,
		.rpt_start_hold_tm = XEC_I2C_SMB_RSHT_1M,
		.mr1 = XEC_I2C_MR0_TM_BAUD16M,
	},
};

static const struct xec_i2c_nl_timing *xec_i2c_nl_timing_for(uint32_t freqhz)
{
	if (freqhz <= KHZ(100)) {
		return &xec_i2c_nl_timing_tbl[0];
	}
	if (freqhz <= KHZ(400)) {
		return &xec_i2c_nl_timing_tbl[1];
	}
	return &xec_i2c_nl_timing_tbl[2];
}

/* Write-1-to-clear the named status bits in the Completion register while
 * preserving its read/write control bits[5:2]. The completion register mixes
 * RW1C status (IDLE, BER, ...) with RW enables (DTEN/HCEN/TCEN/BIDEN) in one
 * word, so a bare sys_write32 of a status constant would also write 0 into
 * those enables.
 */
static inline void xec_i2c_v3_cmpl_clear(uintptr_t base, uint32_t bits)
{
	uint32_t rw = sys_read32(base + XEC_I2C_CMPL_OFS) & XEC_I2C_CMPL_RW_MSK;

	sys_write32(rw | (bits & XEC_I2C_CMPL_RW1C_MSK), base + XEC_I2C_CMPL_OFS);
}

/* Full controller programming: PCR reset, GIRQ enable, port select, timing,
 * and HDONE interrupt enable. Called from ctrl_init and whenever vport
 * configure changes the bus frequency.
 *
 * Must only be called when no transfer is in flight (lock held by caller,
 * or before any transfers are issued).
 *
 * Sequence:
 * Disable controller before PCR reset. Reset affects both port mux and frequency.
 * Short delay after reset to allow clearing of status to propagate.
 * Program controller registers
 *
 */
static int xec_i2c_nl_program_ctrl(const struct device *ctrl, uint32_t freqhz, uint8_t port)
{
	const struct xec_i2c_nl_dev_cfg *xcfg = ctrl->config;
	struct xec_i2c_nl_dev_data *const xdat = ctrl->data;
	const struct xec_i2c_nl_timing *tm = xec_i2c_nl_timing_for(freqhz);
	uintptr_t rb = xcfg->regbase;

	soc_ecia_girq_ctrl(xcfg->girq, xcfg->girq_pos, MCHP_MEC_ECIA_GIRQ_DIS);

	sys_write32(0U, rb + XEC_I2C_CFG_OFS);

	soc_xec_pcr_reset_en(xcfg->enc_pcr);
	k_busy_wait(10U);

	soc_ecia_girq_status_clear(xcfg->girq, xcfg->girq_pos);

	/* PIN=1 to clear any latent assertion left by the legacy engine. */
	sys_write8(BIT(XEC_I2C_CR_PIN_POS), rb + XEC_I2C_CR_OFS);

	/* Port select, filters on, general-call disabled, HDONE interrupt
	 * enabled. IDLE_IEN is intentionally LEFT OFF here — see the
	 * comment on CFG_IDLE_IEN above and the ISR for why it has to be
	 * enabled later (inside the HDONE handler at NL-finished time).
	 * ENAB is set last after timing has been written.
	 */
	sys_write32((XEC_I2C_CFG_PORT_SET(port) | BIT(XEC_I2C_CFG_FEN_POS) |
		     BIT(XEC_I2C_CFG_GC_DIS_POS) | BIT(XEC_I2C_CFG_HD_IEN_POS)),
		    rb + XEC_I2C_CFG_OFS);

	/* Clear any latched CMPL bits we care about so that a stale state
	 * (left over from a prior run before the PCR reset, or from the
	 * power-on default) cannot fire the moment GIRQ is enabled.
	 */
	xec_i2c_v3_cmpl_clear(rb, (BIT(XEC_I2C_CMPL_HDONE_POS) | BIT(XEC_I2C_CMPL_IDLE_POS) |
					CMPL_ERR));

	sys_write32(tm->data_timing, rb + XEC_I2C_DT_OFS);
	sys_write32(tm->idle_scaling, rb + XEC_I2C_ISC_OFS);
	sys_write32(tm->timeout_scaling, rb + XEC_I2C_TMOUT_SC_OFS);
	sys_write32((uint32_t)tm->bus_clock, rb + XEC_I2C_BCLK_OFS);
	soc_mmcr_mask_set8(rb + XEC_I2C_RSHT_OFS, tm->rpt_start_hold_tm, XEC_I2C_RSHT_MSK);
	soc_mmcr_mask_set8(rb + XEC_I2C_MR0_OFS, tm->mr1, XEC_I2C_MR0_TM_MSK);

	sys_write8(XEC_I2C_NL_CR_DFLT, rb + XEC_I2C_CR_OFS);
	sys_set_bit(rb + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);
	/* Enable-to-first-transfer settling window; matches v2. */
	k_busy_wait(20U);

	/* Leave BBCR in live-readback mode so any later read of
	 * BBCR.SCL_IN / BBCR.SDA_IN (e.g. from the recovery path)
	 * reflects the true line state without engaging bit-bang
	 * drive. Pins remain under I2C control.
	 */
	sys_write8(BBCR_LIVE_RD, rb + XEC_I2C_BBCR_OFS);

	/* Clear the GIRQ status one more time before unmasking so any
	 * latch from PCR reset or earlier configuration cannot ride into
	 * NVIC the moment we enable.
	 */
	soc_ecia_girq_status_clear(xcfg->girq, xcfg->girq_pos);
	soc_ecia_girq_ctrl(xcfg->girq, xcfg->girq_pos, MCHP_MEC_ECIA_GIRQ_EN);

	xdat->active_freq = freqhz;
	xdat->active_port = port;

	return 0;
}

int mchp_xec_i2c_nl_port_get(const struct device *i2c_port_dev, uint8_t *port)
{
	return -ENOTSUP;
}

int mchp_xec_i2c_nl_port_set(const struct device *i2c_port_dev, uint8_t port)
{
	return -ENOTSUP;
}

#ifdef CONFIG_I2C_MCHP_XEC_V3_NL_STATE_CAPTURE
int mchp_xec_i2c_nl_clear_capture(const struct device *port)
{
	return -ENOTSUP;
}
int mchp_xec_i2c_nl_copy_capture(const struct device *port, uint8_t *capdest, size_t capdest_size)
{
	return -ENOTSUP;
}
#endif

static int xec_i2c_nl_vport_config(const struct device *port_dev, uint32_t i2c_config)
{
	const struct xec_i2c_nl_port_dev_cfg *port_cfg = port_dev->config;
	const struct device *ctrl_dev = port_cfg->ctrl;
	uint32_t freqhz = 0; /* TODO */
	uint8_t port = 0; /* TODO */
	int rc = 0;

	rc = xec_i2c_nl_program_ctrl(ctrl_dev, freqhz, port);

	return rc;
}

static int xec_i2c_nl_vport_get_config(const struct device *port_dev, uint32_t *i2c_config)
{
	if (i2c_config == NULL) {
		return -EINVAL;
	}

	return -ENOTSUP;
}

static int xec_i2c_nl_vport_transfer(const struct device *port_dev, struct i2c_msg *msgs,
                                     uint8_t num_msgs, uint16_t addr)
{
	return -ENOTSUP;
}

static int xec_i2c_nl_vport_recover_bus(const struct device *port_dev)
{
	return -ENOTSUP;
}

/* I2C controller ISR */
static void xec_i2c_nl_isr(void)
{
	/* TODO */
}

/* Controller and Port initialization */
static int xec_i2c_ctrl_init(const struct device *ctrl_dev)
{
	/* TODO */
	return 0;
}

static int xec_i2c_nl_vport_init(const struct device *port_dev)
{
	/* TODO */
	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int xec_i2c_nl_ctrl_pm_action_cb(const struct device *i2c_ctrl_dev,
                                        enum pm_device_action action)
{
	/* TODO */
	return 0;
}

static int xec_i2c_nl_vport_pm_action_cb(const struct device *port_dev,
                                         enum pm_device_action action)
{
	/* TODO */
	return 0;
}
#endif

/* I2C driver API table */
static DEVICE_API(i2c, xec_i2c_nl_port_api) = {
	.configure = xec_i2c_nl_vport_config,
	.get_config = xec_i2c_nl_vport_get_config,
	.transfer = xec_i2c_nl_vport_transfer,
	.recover_bus = xec_i2c_nl_vport_recover_bus,
};

/* Controller devicetree instantiation */
#define DT_DRV_COMPAT microchip_xec_i2c_v3_nl

#define XEC_I2C_NL_GIRQ(inst, idx)     MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, idx))
#define XEC_I2C_NL_GIRQ_POS(inst, idx) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, idx))

#define XEC_I2C_NL_CTRL_INST(inst) \
	static void xec_i2c_nl_irq_conn_##inst(void) \
	{ \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), xec_i2c_nl_isr, \
			    DEVICE_DT_INST_GET(inst), 0); \
		irq_enable(DT_INST_IRQN(inst)); \
	} \
	static const struct xec_i2c_nl_dev_cfg xec_i2c_nl_xcfg_##inst = { \
		.regbase = (uintptr_t)DT_INST_REG_ADDR(inst), \
		.xec_irq_connect = xec_i2c_nl_irq_conn_##inst, \
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR(inst)), \
		.dma_chan = DT_INST_DMAS_CELL_BY_NAME(inst, host, channel), \
		.dma_slot = DT_INST_DMAS_CELL_BY_NAME(inst, host, trigsrc), \
		.enc_pcr = DT_INST_PROP(inst, pcr_scr), \
		.girq = XEC_I2C_NL_GIRQ(inst, 0), \
		.girq_pos = XEC_I2C_NL_GIRQ_POS(inst, 0), \
		.girq_wk = XEC_I2C_NL_GIRQ(inst, 1), \
		.girq_wk_pos = XEC_I2C_NL_GIRQ_POS(inst, 1), \
		.wakeup_source = DT_INST_PROP(inst, wakeup_source), \
	}; \
	static struct xec_i2c_nl_dev_data xec_i2c_nl_xdat_##inst; \
	PM_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_ctrl_pm_action_cb); \
	DEVICE_DT_INST_DEFINE(inst, xec_i2c_ctrl_init, PM_DEVICE_DT_INST_GET(inst), \
			      &xec_i2c_nl_xdat_##inst, &xec_i2c_nl_xcfg_##inst, \
			      POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, NULL);


DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_CTRL_INST)

/* Port devicetree instantiation */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT microchip_xec_i2c_v3_nl_port

#define XEC_I2C_NL_PORT_IS_DEFAULT(inst)                                                           \
	COND_CODE_1(DT_NODE_HAS_PROP(DT_INST_PHANDLE(inst, controller), default_port),             \
		    (DT_SAME_NODE(DT_DRV_INST(inst),                                               \
				  DT_PHANDLE(DT_INST_PHANDLE(inst, controller), default_port))),   \
		    (0))

#define XEC_I2C_NL_PORT_INST(inst) \
	PINCTRL_DT_INST_DEFINE(inst); \
	static const struct xec_i2c_nl_port_dev_cfg xec_i2c_port_xcfg_##inst = { \
		.ctrl = DEVICE_DT_GET(DT_INST_PHANDLE(inst, controller)), \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), \
		.bitrate = DT_INST_PROP_OR(inst, clock_frequency, I2C_BITRATE_STANDARD), \
		.port = (uint8_t)(DT_INST_PROP(inst, port) & 0x0fU), \
		.is_default = XEC_I2C_NL_PORT_IS_DEFAULT(inst), \
	}; \
	PM_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_vport_pm_action_cb); \
	I2C_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_vport_init, PM_DEVICE_DT_INST_GET(inst), \
				  NULL, &xec_i2c_port_xcfg_##inst, POST_KERNEL, \
				  CONFIG_I2C_INIT_PRIORITY, &xec_i2c_nl_port_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_PORT_INST)
