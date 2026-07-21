/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip XEC I2Cv3 byte mode I2C driver.
 */

#include "soc_ecia.h"
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
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(i2c_mchp_xec_v3_bm, CONFIG_I2C_LOG_LEVEL);

#include "i2c_mchp_xec_regs.h"

struct xec_i2c_v3_bm_xcfg {
	uintptr_t base;
	uint32_t dflt_freq;
	uint16_t enc_pcr;
	uint8_t girq;
	uint8_t girq_pos;
	void (*irq_connect)(void);

};

struct xec_i2c_v3_bm_xdat {
	uint32_t i2c_cmpl;
	uint32_t i2c_cfg;
	uint8_t i2c_sr;
	uint8_t i2c_cr_cache;
	struct i2c_msg *msgs;
	uint16_t nmsgs;
	struct k_mutex lock;
	struct k_sem sync;
};

struct xec_i2c_v3_bm_port_xcfg {
	const struct device *parent;
	const struct pinctrl_dev_config *pcfg;
	uint32_t bitrate;
	uint8_t port_id;
	bool is_default;
};

static void xec_i2c_v3_bm_isr(const struct device *ctrl_dev)
{
	const struct xec_i2c_v3_bm_xcfg *xcfg = ctrl_dev->config;
	struct xec_i2c_v3_bm_xdat *const xdat = ctrl_dev->data;
	uintptr_t base = xcfg->base;

	xdat->i2c_cmpl = sys_read32(base + XEC_I2C_CMPL_OFS);
	xdat->i2c_sr = sys_read8(base + XEC_I2C_SR_OFS);

	soc_ecia_girq_status_clear(xcfg->girq, xcfg->girq_pos);
}

/* ---- API ---- */
static int xec_i2c_v3_bm_vport_configure(const struct device *port_dev, uint32_t i2c_config)
{
	return 0;
}

static int xec_i2c_v3_bm_vport_get_config(const struct device *port_dev, uint32_t *i2c_config)
{
	return 0;
}

static int xec_i2c_v3_bm_vport_transfer(const struct device *port_dev, struct i2c_msg *msgs,
                                        uint8_t num_msgs, uint16_t address)
{
	return 0;
}

static int xec_i2c_v3_bm_vport_recover_bus(const struct device *port_dev)
{
	return 0;
}

static int xec_i2c_v3_bm_cr_init(const struct device *cr_dev, uint32_t freq, uint8_t port)
{
	return 0;
}

static int xec_i2c_v3_bm_port_init(const struct device *port_dev)
{
	const struct xec_i2c_v3_bm_port_xcfg *pc = port_dev->config;
	int rc = 0;

	rc = pinctrl_apply_state(pc->pcfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("pinctrl_apply_state(%s)=%d", port_dev->name, rc);
		return rc;
	}

	if (pc->is_default) {
		rc = xec_i2c_v3_bm_cr_init(pc->parent, pc->bitrate, pc->port_id);
	}

	return 0;
}

static DEVICE_API(i2c, xec_i2c_v3_bm_port_api) = {
	.configure = xec_i2c_v3_bm_vport_configure,
	.get_config = xec_i2c_v3_bm_vport_get_config,
	.transfer = xec_i2c_v3_bm_vport_transfer,
	.recover_bus = xec_i2c_v3_bm_vport_recover_bus,
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

/* core driver device structure */
#define DT_DRV_COMPAT microchip_xec_i2c_v3_bm

#define XEC_I2C_V3_DFLT_FREQ(inst) I2C_BITRATE_STANDARD

#define XEC_I2C_V3_GIRQ(inst)     MCHP_XEC_ECIA_GIRQ(DT_INST_PROP(inst, girqs))
#define XEC_I2C_V3_GIRQ_POS(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP(inst, girqs))

#define XEC_I2C_V3_BM_CTRL_INIT(inst)                                                              \
	static void xec_i2c_v3_bm_irq_connect_##inst(void)                                         \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), xec_i2c_v3_bm_isr,    \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
	static const struct xec_i2c_v3_bm_xcfg xec_i2c_v3_bm_xcfg_##inst = {                       \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.irq_connect = xec_i2c_v3_bm_irq_connect_##inst,                                   \
		.dflt_freq = XEC_I2C_V3_DFLT_FREQ(inst),                                           \
		.girq = XEC_I2C_V3_GIRQ(inst),                                                     \
		.girq_pos = XEC_I2C_V3_GIRQ_POS(inst),                                             \
		.enc_pcr = DT_INST_PROP(inst, pcr_scr),                                            \
	};                                                                                         \
	static struct xec_i2c_v3_bm_xdat xec_i2c_v3_bm_xdat_##inst;                                \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &xec_i2c_v3_bm_xdat_##inst,                        \
			      &xec_i2c_v3_bm_xcfg_##inst, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,   \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_V3_BM_CTRL_INIT)

/* port driver device structure */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT microchip_xec_i2c_v3_bm_port

#define XEC_I2C_V3_BM_PORT_INIT(inst)                                                              \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static const struct xec_i2c_v3_bm_port_xcfg xec_i2c_v3_bm_port_xcfg_##inst = {             \
		.parent = DEVICE_DT_GET(DT_INST_PHANDLE(inst, controller)),                        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.bitrate = DT_INST_PROP_OR(inst, clock_frequency, I2C_BITRATE_STANDARD),           \
		.port_id = (uint8_t)(DT_INST_PROP(inst, port) & 0x0FU),                            \
		.is_default = DT_INST_PROP(inst, default_port),                                    \
	};                                                                                         \
	I2C_DEVICE_DT_INST_DEFINE(                                                                 \
		inst, xec_i2c_v3_bm_port_init, NULL, NULL, &xec_i2c_v3_bm_port_xcfg_##inst,        \
		POST_KERNEL, CONFIG_I2C_MCHP_XEC_V3_BM_PORT_INIT_PRIORITY,                         \
		&xec_i2c_v3_bm_port_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_V3_BM_PORT_INIT)
