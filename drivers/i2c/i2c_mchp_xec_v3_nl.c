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
	const struct pinctrl_dev_cfg *pincfg;
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
	return -ENOTSUP;
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
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTRL(inst)), \
		.dma_chan = DT_INST_DMAS_CELL_BY_NAME(inst, host, channel), \
		.dma_slot = DT_INST_DMAS_CELL_BY_NAME(inst, host, trigscr), \
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
	static const struct xec_i2c_port_dev_cfg *xec_i2c_port_xcfg_##inst = { \
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
