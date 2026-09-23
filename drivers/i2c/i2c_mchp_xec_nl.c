/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip XEC version 3.8 I2C HW Network-Layer (NL) I2C driver.
 */
#include <errno.h>
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

/* register defines */
#include "i2c_mchp_xec_regs.h"

LOG_MODULE_REGISTER(i2c_mchp_xec_nl, CONFIG_I2C_LOG_LEVEL);

/* Default I2C control-register value: ESO+ACK+PIN. PIN is also raised at
 * reset to clear any latent PIN-asserted state in the legacy I2C engine.
 */
#define XEC_I2C_NL_CR_DFLT                                                                         \
	(BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS) | BIT(XEC_I2C_CR_PIN_POS))

#define CMPL_HDONE BIT(XEC_I2C_CMPL_HDONE_POS)
#define CMPL_IDLE  BIT(XEC_I2C_CMPL_IDLE_POS)
#define CMPL_HNAK  BIT(XEC_I2C_CMPL_HNAKX_POS)
#define CMPL_LAB   BIT(XEC_I2C_CMPL_LAB_STS_POS)
#define CMPL_BER   BIT(XEC_I2C_CMPL_BER_STS_POS)
#define CMPL_ERR   (CMPL_HNAK | CMPL_LAB | CMPL_BER)

#define BBCR_SCL_IN BIT(XEC_I2C_BBCR_SCL_IN_POS)
#define BBCR_SDA_IN BIT(XEC_I2C_BBCR_SDA_IN_POS)

#define BBCR_LIVE_RD     BIT(XEC_I2C_BBCR_CM_POS)
#define BBCR_BB_RELEASED BIT(XEC_I2C_BBCR_EN_POS) /* BBM_EN=1, both dirs=input, both released */
/* BBM_EN=1, SCL drive-low, SDA released */
#define BBCR_BB_SCL_LOW  (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CD_POS))
/* BBM_EN=1, SDA drive-low, SCL released */
#define BBCR_BB_SDA_LOW  (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_DD_POS)) 

enum xec_i2c_nl_timing_row {
	XEC_I2C_NL_TM_100K,
	XEC_I2C_NL_TM_400K,
	XEC_I2C_NL_TM_1M,
	XEC_I2C_NL_TM_DT,
	XEC_I2C_NL_TM_COUNT,
};

struct xec_i2c_nl_timing {
	uint32_t data_timing;
	uint32_t idle_scaling;
	uint32_t timeout_scaling;
	uint16_t bus_clock;
	uint8_t rpt_start_hold_tm;
	uint8_t mr1;
};

/* Controller device structure */
struct xec_i2c_nl_config {
	uintptr_t base;
	const struct device *dma_dev;
	void (*irq_connect)(void);
	uint32_t dflt_freq;
	uint8_t cm_dma_chan;
	uint8_t cm_dma_slot;
	uint16_t enc_pcr;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t girq_wk;
	uint8_t girq_wk_pos;
	bool has_dt_timing;
#ifdef CONFIG_I2C_CALLBACK
	/* TODO */
#endif
#ifdef CONFIG_I2C_TARGET
	uint8_t tm_dma_chan;
	uint8_t tm_dma_slot;
	bool wakeup_source;
	/* TODO more */
#endif
	struct xec_i2c_nl_timing timing[XEC_I2C_NL_TM_COUNT];
};

/* Controller data structure */
struct xec_i2c_nl_data {
	const struct device *ctrl;
	uint8_t addr_byte[2] __aligned(4);
	uint32_t active_freq;
	uint8_t active_port;
	/* TODO more */
#ifdef CONFIG_I2C_CALLBACK
	/* TODO more */
#endif
#ifdef CONFIG_I2C_TARGET
	/* TODO more */
#endif
};

/* struct i2c_msg[] -> HW request parsing; kept SoC-independent, see the file. */
#include "i2c_mchp_xec_nl_msg.c"

/* Port device struture */
struct xec_i2c_nl_port_config {
	const struct device *controller;
	const struct pinctrl_dev_config *pincfg;
	uint32_t bitrate;
	uint8_t port_id;
	bool is_default;
};

/*
 * Port device data: per-port state that must survive the driver
 * switching the shared controller away to another port and back.
 * runtime_freq is 0 until the app calls i2c_configure() on this port
 * at least once (0 Hz is never a valid bus frequency, so it doubles
 * as "unset"); once set it takes precedence over the port's DT
 * clock-frequency every time this port is (re)selected -- see
 * xec_i2c_nl_port_freq().
 */
struct xec_i2c_nl_port_data {
	uint32_t runtime_freq;
};

/* Sentinel freqhz value: use timing[XEC_I2C_NL_TM_DT] verbatim instead
 * of a bucketed 100k/400k/1M row (I2C_SPEED_DT / a "timing-dt" port).
 */
#define XEC_I2C_NL_FREQ_DT UINT32_MAX

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

/*
 * Map a Zephyr I2C_SPEED_* value (I2C_SPEED_GET() of an i2c_configure()
 * request) to the Hz value xec_i2c_nl_timing_for() keys its lookup on.
 * Returns 0 for a speed this HW's timing table has no row for
 * (I2C_SPEED_HIGH/ULTRA -- this NL engine tops out at timing_1000k) or
 * an unrecognized value; 0 is never a valid return otherwise.
 */
static uint32_t xec_i2c_nl_speed_to_freq(uint32_t speed)
{
	switch (speed) {
	case I2C_SPEED_STANDARD:
		return KHZ(100);
	case I2C_SPEED_FAST:
		return KHZ(400);
	case I2C_SPEED_FAST_PLUS:
		return MHZ(1);
	case I2C_SPEED_DT:
		return XEC_I2C_NL_FREQ_DT;
	default:
		return 0U;
	}
}

/* Inverse of xec_i2c_nl_speed_to_freq(), for i2c_get_config(); same
 * bucketing xec_i2c_nl_timing_for() uses so the two never disagree.
 */
static uint32_t xec_i2c_nl_freq_to_speed(uint32_t freqhz)
{
	if (freqhz == XEC_I2C_NL_FREQ_DT) {
		return I2C_SPEED_DT;
	}
	if (freqhz <= KHZ(100)) {
		return I2C_SPEED_STANDARD;
	}
	if (freqhz <= KHZ(400)) {
		return I2C_SPEED_FAST;
	}
	return I2C_SPEED_FAST_PLUS;
}

/*
 * Frequency a port should (re)program the controller with, in
 * precedence order: this port's i2c_configure()-set runtime override,
 * else its own DT clock-frequency, else the controller's DT default.
 */
static uint32_t xec_i2c_nl_port_freq(const struct xec_i2c_nl_port_config *port_cfg,
				      const struct xec_i2c_nl_port_data *port_data)
{
	const struct xec_i2c_nl_config *ctrl_cfg = port_cfg->controller->config;

	if (port_data->runtime_freq != 0U) {
		return port_data->runtime_freq;
	}
	if (port_cfg->bitrate != 0U) {
		return port_cfg->bitrate;
	}
	return ctrl_cfg->dflt_freq;
}

static const struct xec_i2c_nl_timing *
xec_i2c_nl_timing_for(const struct xec_i2c_nl_config *cfg, uint32_t freqhz)
{
	if (freqhz == XEC_I2C_NL_FREQ_DT) {
		return &cfg->timing[XEC_I2C_NL_TM_DT];
	}
	if (freqhz <= KHZ(100)) {
		return &cfg->timing[XEC_I2C_NL_TM_100K];
	}
	if (freqhz <= KHZ(400)) {
		return &cfg->timing[XEC_I2C_NL_TM_400K];
	}
	return &cfg->timing[XEC_I2C_NL_TM_1M];
}

static int xec_i2c_nl_program_ctrl(const struct device *ctrl, uint32_t freqhz, uint8_t port)
{
	const struct xec_i2c_nl_config *cfg = ctrl->config;
	struct xec_i2c_nl_data *data = ctrl->data;
	const struct xec_i2c_nl_timing *tm = xec_i2c_nl_timing_for(cfg, freqhz);
	uintptr_t base = cfg->base;

	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, MCHP_MEC_ECIA_GIRQ_DIS);

	sys_write32(0U, base + XEC_I2C_CFG_OFS);

	soc_xec_pcr_reset_en(cfg->enc_pcr);
	k_busy_wait(10U);

	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);
	soc_ecia_girq_status_clear(cfg->girq_wk, cfg->girq_wk_pos);

	/* PIN=1 to clear any latent assertion left by the legacy engine. */
	sys_write8(BIT(XEC_I2C_CR_PIN_POS), base + XEC_I2C_CR_OFS);

	/* Port select, filters on, general-call disabled, HDONE interrupt
	 * enabled. IDLE_IEN is intentionally LEFT OFF here — see the
	 * comment on CFG_IDLE_IEN above and the ISR for why it has to be
	 * enabled later (inside the HDONE handler at NL-finished time).
	 * ENAB is set last after timing has been written.
	 */
	sys_write32((XEC_I2C_CFG_PORT_SET(port) | BIT(XEC_I2C_CFG_FEN_POS) |
		     BIT(XEC_I2C_CFG_GC_DIS_POS) | BIT(XEC_I2C_CFG_HD_IEN_POS)),
		     base + XEC_I2C_CFG_OFS);

	/* Clear any latched CMPL bits we care about so that a stale state
	 * (left over from a prior run before the PCR reset, or from the
	 * power-on default) cannot fire the moment GIRQ is enabled.
	 */
	xec_i2c_v3_cmpl_clear(base, XEC_I2C_CMPL_RW1C_MSK);

	sys_write32(tm->data_timing, base + XEC_I2C_DT_OFS);
	sys_write32(tm->idle_scaling, base + XEC_I2C_ISC_OFS);
	sys_write32(tm->timeout_scaling, base + XEC_I2C_TMOUT_SC_OFS);
	sys_write32((uint32_t)tm->bus_clock, base + XEC_I2C_BCLK_OFS);
	soc_mmcr_mask_set8(base + XEC_I2C_RSHT_OFS, tm->rpt_start_hold_tm, XEC_I2C_RSHT_MSK);
	soc_mmcr_mask_set8(base + XEC_I2C_MR0_OFS, tm->mr1, XEC_I2C_MR0_TM_MSK);

	sys_write8(XEC_I2C_NL_CR_DFLT, base + XEC_I2C_CR_OFS);
	sys_set_bit(base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);
	/* Enable-to-first-transfer settling window; matches v2. */
	k_busy_wait(20U);

	/* Leave BBCR in live-readback mode so any later read of
	 * BBCR.SCL_IN / BBCR.SDA_IN (e.g. from the recovery path)
	 * reflects the true line state without engaging bit-bang
	 * drive. Pins remain under I2C control.
	 */
	sys_write8(BBCR_LIVE_RD, base + XEC_I2C_BBCR_OFS);

	/* Clear the GIRQ status one more time before unmasking so any
	 * latch from PCR reset or earlier configuration cannot ride into
	 * NVIC the moment we enable.
	 */
	soc_ecia_girq_status_clear(cfg->girq, cfg->girq_pos);
	soc_ecia_girq_status_clear(cfg->girq_wk, cfg->girq_wk_pos);
	soc_ecia_girq_ctrl(cfg->girq, cfg->girq_pos, MCHP_MEC_ECIA_GIRQ_EN);

	data->active_freq = freqhz;
	data->active_port = port;

	return 0;
}

static int xec_i2c_nl_apply_port(const struct device *port_dev)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	struct xec_i2c_nl_port_data *port_data = port_dev->data;
	const struct device *ctrl = port_cfg->controller;
	struct xec_i2c_nl_data *ctrl_data = ctrl->data;
	int rc;

	if (ctrl_data->active_port == port_cfg->port_id) {
		return 0;
	}

	rc = pinctrl_apply_state(port_cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("pinctrl_apply_state(%s)=%d", port_dev->name, rc);
		return rc;
	}

	return xec_i2c_nl_program_ctrl(ctrl, xec_i2c_nl_port_freq(port_cfg, port_data),
				       port_cfg->port_id);
}

/* I2C configure API */
static int xec_i2c_nl_vport_config(const struct device *port_dev, uint32_t i2c_config)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	struct xec_i2c_nl_port_data *port_data = port_dev->data;
	const struct device *ctrl = port_cfg->controller;
	struct xec_i2c_nl_data *ctrl_data = ctrl->data;
	uint32_t freq;

	if (!(i2c_config & I2C_MODE_CONTROLLER)) {
		return -ENOTSUP; /* target-only mode has nothing to configure here */
	}

	freq = xec_i2c_nl_speed_to_freq(I2C_SPEED_GET(i2c_config));
	if (freq == 0U) {
		return -ENOTSUP;
	}

	/* Sticky per port: applies now if this port is already selected on
	 * the shared controller, and again on every future switch back to
	 * this port -- see xec_i2c_nl_apply_port()/xec_i2c_nl_port_freq().
	 */
	port_data->runtime_freq = freq;

	if (ctrl_data->active_port == port_cfg->port_id) {
		return xec_i2c_nl_program_ctrl(ctrl, freq, port_cfg->port_id);
	}

	return 0;
}

/* I2C get config API */
static int xec_i2c_nl_vport_get_config(const struct device *port_dev, uint32_t *i2c_config)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	const struct xec_i2c_nl_port_data *port_data = port_dev->data;

	*i2c_config = I2C_MODE_CONTROLLER |
		      I2C_SPEED_SET(xec_i2c_nl_freq_to_speed(
			      xec_i2c_nl_port_freq(port_cfg, port_data)));

	return 0;
}

/* I2C synchronous transfer API */
static int xec_i2c_nl_vport_xfr(const struct device *port_dev, struct i2c_msg *msgs,
				uint8_t num_msgs, uint16_t i2c_address)
{
	/* TODO */
	return 0;
}

#ifdef CONFIG_I2C_CALLBACK
/* I2C asynchronous transfer API */
static int xec_i2c_nl_vport_xfr_cb(const struct device *port_dev, struct i2c_msg *msgs,
				   uint8_t num_msgs, uint16_t i2c_address, i2c_callback_t cb,
				   void *userdata)
{
	/* TODO */
	return 0;
}
#endif

#ifdef CONFIG_I2C_TARGET
static int xec_i2c_nl_vport_target_register(const struct device *port_dev,
					    struct i2c_target_config *target_cfg)
{
	/* TODO */
	return 0;
}

static int xec_i2c_nl_vport_target_unregister(const struct device *port_dev,
					      struct i2c_target_config *target_cfg)
{
	/* TODO */
	return 0;
}
#endif

/* ---- Controller interrupt handler --- */
static void xec_i2c_nl_isr(const struct device *ctrl_dev)
{
	/* TODO */
}

#ifdef CONFIG_PM_DEVICE
static int xec_i2c_nl_ctrl_pm_action_cb(const struct device *i2c_port, enum pm_device_action action)
{
	/* TODO */
	return -ENOTSUP;
}

static int xec_i2c_nl_vport_pm_action_cb(const struct device *i2c_port,
					 enum pm_device_action action)
{
	/* TODO */
	return -ENOTSUP;
}
#endif /* CONFIG_PM_DEVICE */

/* Driver initialization */
static int xec_i2c_nl_ctrl_init(const struct device *ctrl_dev)
{
	const struct xec_i2c_nl_config *ctrl_cfg = ctrl_dev->config;

	/* TODO */

	if (ctrl_cfg->irq_connect != NULL) {
		ctrl_cfg->irq_connect();
	}

	return 0;
}

static int xec_i2c_nl_port_init(const struct device *port_dev)
{
	const struct xec_i2c_nl_port_config *port_cfg = port_dev->config;
	struct xec_i2c_nl_port_data *const port_data = port_dev->data;

	port_data->runtime_freq = port_cfg->bitrate;

	return xec_i2c_nl_apply_port(port_dev);
}

static DEVICE_API(i2c, xec_i2c_nl_port_api) = {
	.configure = xec_i2c_nl_vport_config,
	.get_config = xec_i2c_nl_vport_get_config,
	.transfer = xec_i2c_nl_vport_xfr,
	/* TODO .recover_bus */
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = xec_i2c_nl_vport_xfr_cb,
#endif
#ifdef CONFIG_I2C_TARGET
	.target_register = xec_i2c_nl_vport_target_register,
	.target_unregister = xec_i2c_nl_vport_target_unregister,
#endif
};

/* Controller DT */
#define DT_DRV_COMPAT microchip_xec_i2c_nl

#define XEC_I2C_NL_DFLT_FREQ(inst) I2C_BITRATE_STANDARD

#define XEC_I2C_NL_GIRQ(inst, idx)     MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, idx))
#define XEC_I2C_NL_GIRQ_POS(inst, idx) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, idx))

#define XEC_I2C_NL_TIMING_FROM_DT(inst, prop)                                                      \
	{                                                                                          \
		.data_timing = DT_INST_PROP_BY_IDX(inst, prop, 0),                                 \
		.idle_scaling = DT_INST_PROP_BY_IDX(inst, prop, 1),                                \
		.timeout_scaling = DT_INST_PROP_BY_IDX(inst, prop, 2),                             \
		.bus_clock = DT_INST_PROP_BY_IDX(inst, prop, 3),                                   \
		.rpt_start_hold_tm = DT_INST_PROP_BY_IDX(inst, prop, 4),                           \
		.mr1 = DT_INST_PROP_BY_IDX(inst, prop, 5),                                         \
	}

#define XEC_I2C_NL_TIMING_DFLT(dtm, isc, tmo, bclk, rsht)                                          \
	{                                                                                          \
		.data_timing = (dtm), .idle_scaling = (isc), .timeout_scaling = (tmo),             \
		.bus_clock = (bclk), .rpt_start_hold_tm = (rsht),                                  \
		.mr1 = XEC_I2C_MR0_TM_BAUD16M,                                                     \
	}

#define XEC_I2C_NL_TIMING_ROW(inst, prop, dtm, isc, tmo, bclk, rsht)                               \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop),                                             \
		    (XEC_I2C_NL_TIMING_FROM_DT(inst, prop)),                                       \
		    (XEC_I2C_NL_TIMING_DFLT(dtm, isc, tmo, bclk, rsht)))

#define XEC_I2C_NL_TIMING_ROWS(inst)                                                               \
	{XEC_I2C_NL_TIMING_ROW(inst, timing_100k, XEC_I2C_SMB_DATA_TM_100K,                        \
			   XEC_I2C_SMB_IDLE_SC_100K, XEC_I2C_SMB_TMO_SC_100K,                      \
			   XEC_I2C_SMB_BUS_CLK_100K, XEC_I2C_SMB_RSHT_100K),                       \
	 XEC_I2C_NL_TIMING_ROW(inst, timing_400k, XEC_I2C_SMB_DATA_TM_400K,                        \
			   XEC_I2C_SMB_IDLE_SC_400K, XEC_I2C_SMB_TMO_SC_400K,                      \
			   XEC_I2C_SMB_BUS_CLK_400K, XEC_I2C_SMB_RSHT_400K),                       \
	 XEC_I2C_NL_TIMING_ROW(inst, timing_1000k, XEC_I2C_SMB_DATA_TM_1M,                         \
			   XEC_I2C_SMB_IDLE_SC_1M, XEC_I2C_SMB_TMO_SC_1M,                          \
			   XEC_I2C_SMB_BUS_CLK_1M, XEC_I2C_SMB_RSHT_1M),                           \
	 XEC_I2C_NL_TIMING_ROW(inst, timing_dt, XEC_I2C_SMB_DATA_TM_100K,                          \
			   XEC_I2C_SMB_IDLE_SC_100K, XEC_I2C_SMB_TMO_SC_100K,                      \
			   XEC_I2C_SMB_BUS_CLK_100K, XEC_I2C_SMB_RSHT_100K)}

#define XEC_I2C_NL_TIMING_ASSERT(inst, prop)                                                           \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, prop),                                              \
		(BUILD_ASSERT(DT_INST_PROP_LEN(inst, prop) == 6,                                   \
			      #prop " needs exactly 6 cells");                                     \
		 BUILD_ASSERT(DT_INST_PROP_BY_IDX(inst, prop, 3) <= 0xFFFF,                        \
			      #prop " bus-clock exceeds 16 bits");                                 \
		 BUILD_ASSERT(DT_INST_PROP_BY_IDX(inst, prop, 4) <= 0xFF,                          \
			      #prop " rpt-start-hold exceeds 8 bits");                             \
		 BUILD_ASSERT(DT_INST_PROP_BY_IDX(inst, prop, 5) <= 0xFF,                          \
			      #prop " mr0-timing exceeds 8 bits");))

#define XEC_I2C_NL_DEFPORT_ASSERT(inst)                                                            \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, default_port),                                      \
		   (BUILD_ASSERT(DT_SAME_NODE(DT_DRV_INST(inst),                                   \
			DT_PHANDLE(DT_INST_PHANDLE(inst, default_port), controller)),              \
			"default-port must reference a port on this controller");))

#define XEC_I2C_NL_CTRL_INIT(inst)                                                                 \
	XEC_I2C_NL_DEFPORT_ASSERT(inst)                                                            \
	XEC_I2C_NL_TIMING_ASSERT(inst, timing_100k)                                                \
	XEC_I2C_NL_TIMING_ASSERT(inst, timing_400k)                                                \
	XEC_I2C_NL_TIMING_ASSERT(inst, timing_1000k)                                               \
	XEC_I2C_NL_TIMING_ASSERT(inst, timing_dt)                                                  \
	static void xec_i2c_nl_irq_connect_##inst(void)                                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), xec_i2c_nl_isr,       \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
	static const struct xec_i2c_nl_config xec_i2c_nl_dcfg_##inst = {                           \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR(inst)),                                 \
		.irq_connect = xec_i2c_nl_irq_connect_##inst,                                      \
		.dflt_freq = XEC_I2C_NL_DFLT_FREQ(inst),                                           \
		.has_dt_timing = DT_INST_NODE_HAS_PROP(inst, timing_dt),                           \
		.girq = XEC_I2C_NL_GIRQ(inst, 0),                                                  \
		.girq_pos = XEC_I2C_NL_GIRQ_POS(inst, 0),                                          \
		.girq_wk = XEC_I2C_NL_GIRQ(inst, 1),                                               \
		.girq_wk_pos = XEC_I2C_NL_GIRQ_POS(inst, 1),                                       \
		.enc_pcr = DT_INST_PROP(inst, pcr_scr),                                            \
		.cm_dma_chan = DT_INST_DMAS_CELL_BY_NAME(inst, host, channel),                     \
		.cm_dma_slot = DT_INST_DMAS_CELL_BY_NAME(inst, host, trigsrc),                     \
		.timing = XEC_I2C_NL_TIMING_ROWS(inst), };                                         \
	static struct xec_i2c_nl_data xec_i2c_nl_data_##inst;                                      \
	PM_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_ctrl_pm_action_cb);                              \
	DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_ctrl_init, PM_DEVICE_DT_INST_GET(inst),             \
			      &xec_i2c_nl_data_##inst, &xec_i2c_nl_dcfg_##inst, POST_KERNEL,       \
			      CONFIG_I2C_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_CTRL_INIT)

/* Port DT */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT microchip_xec_i2c_nl_port

/* A port is the boot/default routing if and only if its controller's default-port
 * phandle names this port node. Controllers without a default-port yield 0
 * (no boot pre-routing; the first transfer applies its own port lazily).
 */
#define XEC_I2C_NL_PORT_IS_DEFAULT(inst)                                                           \
	COND_CODE_1(DT_NODE_HAS_PROP(DT_INST_PHANDLE(inst, controller), default_port),             \
		    (DT_SAME_NODE(DT_DRV_INST(inst),                                               \
				  DT_PHANDLE(DT_INST_PHANDLE(inst, controller), default_port))),   \
		    (0))

#define XEC_I2C_NL_PORT_INIT(inst)                                                                 \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static const struct xec_i2c_nl_port_config xec_i2c_nl_port_dcfg_##inst = {                 \
		.controller = DEVICE_DT_GET(DT_INST_PHANDLE(inst, controller)),                    \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		.bitrate = DT_INST_PROP_OR(inst, clock_frequency, I2C_BITRATE_STANDARD),           \
		.port_id = (uint8_t)(DT_INST_PROP(inst, port) & 0x0FU),                            \
		.is_default = XEC_I2C_NL_PORT_IS_DEFAULT(inst),                                    \
	};                                                                                         \
	static struct xec_i2c_nl_port_data xec_i2c_nl_port_data_##inst;                            \
	PM_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_vport_pm_action_cb);                             \
	I2C_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_port_init, PM_DEVICE_DT_INST_GET(inst),         \
				  &xec_i2c_nl_port_data_##inst, &xec_i2c_nl_port_dcfg_##inst,      \
				  POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,                           \
				  &xec_i2c_nl_port_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_PORT_INIT)
