/*
 * Copyright 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_i2c_xec_nl

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_mchp_xec_nl);

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>

#include "i2c-priv.h"
#include "i2c_mchp_xec_regs.h"

struct xec_i2c_nl_config {
	mem_addr_t i2c_base;
	const struct pinctrl_dev_config *pin_cfg;
	void (*irq_config)(void);
	uint8_t girq;
	uint8_t girq_pos;
};

struct xec_i2c_nl_data {
	uint32_t i2c_config;
	volatile uint32_t i2c_status;
};

static uint32_t xec_i2c_status_get(mem_addr_t i2c_base)
{
	uint32_t v = sys_read32(i2c_base + XEC_I2C_COMP_OFS);

	v &= ~GENMASK(7, 0);
	v |= sys_read8(i2c_base + XEC_I2C_SR_OFS);

	return v;
}

/* Public API */
static int xec_i2c_nl_configure(const struct device *dev, uint32_t i2c_config)
{
	/* TODO */
	return 0;
}

static int xec_i2c_nl_get_config(const struct device *dev, uint32_t *i2c_config)
{
	/* TODO */
	return 0;
}

static int xec_i2c_nl_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			       uint16_t addr)
{
	/* TODO */
	return 0;
}

static int xec_i2c_nl_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
#ifdef CONFIG_I2C_TARGET
	/* TODO */
	return 0;
#else
	return -ENOTSUP;
#endif
}

static int xec_i2c_nl_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
#ifdef CONFIG_I2C_TARGET
	/* TODO */
	return 0;
#else
	return -ENOTSUP;
#endif
}

static int xec_i2c_nl_recover_bus(const struct device *dev)
{
	/* TODO */
	return 0;
}

#ifdef CONFIG_I2C_CALLBACK
static int xec_i2c_nl_transfer_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				  uint16_t addr, i2c_callback_t cb, void *userdata)
{
	/* TODO */
	return 0;
}
#endif

#ifdef CONFIG_I2C_RTIO
static int xec_i2c_nl_iodev_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	/* TODO */
	return 0;
}
#endif

/* Driver initialization */
static int xec_i2c_nl_init(const struct device *dev)
{
	/* TODO */
	return 0;
}

/* I2C-NL interrupt handler */
static void xec_i2c_nl_isr(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	mem_addr_t i2c_base = devcfg->i2c_base;

	data->i2c_status = xec_i2c_status_get(i2c_base);

	sys_write32(XEC_I2C_COMP_RW1C_MSK, i2c_base + XEC_I2C_COMP_OFS);
	soc_ecia_girq_status_clear(devcfg->girq, devcfg->girq_pos);
}

static DEVICE_API(i2c, xec_i2c_nl_driver_api) = {
	.configure = xec_i2c_nl_configure,
	.get_config = xec_i2c_nl_get_config,
	.transfer = xec_i2c_nl_transfer,
	.target_register = xec_i2c_nl_target_register,
	.target_unregister = xec_i2c_nl_target_unregister,
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = xec_i2c_nl_transfer_cb,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit xec_i2c_nl_iodev_submit,
#endif
	.recover_bus = xec_i2c_nl_recover_bus,
};

#define XEC_I2C_GIRQ_DT(inst) MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, 0))
#define XEC_I2C_GIRQ_POS_DT(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, 0))

#define XEC_I2C_NL_DEVICE(inst) \
	struct xec_i2c_nl_data xec_i2c_nl##inst##_data; \
	PINCTRL_DT_INST_DEFINE(inst); \
	static void xec_i2c_nl##inst##_irq_config(void) { \
		IRQ_CONNECT(DT_INST_IRQN(inst),				\
			    DT_INST_IRQ(inst, priority),		\
			    xec_i2c_nl_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQN(n)); \
	} \
	static const struct xec_i2c_nl_config xec_i2c_nl##inst##_cfg = { \
		.i2c_base = (mem_addr_t)DT_INST_REG_ADDR(inst), \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), \
		.irq_config = xec_i2c_nl##inst##_irq_config, \
		.girq = XEC_I2C_GIRQ_DT(inst), \
		.girq_pos = XEC_I2C_GIRQ_POS_DT(inst), \
	}; \
	I2C_DEVICE_DT_INST_DEFINE(n, xec_i2c_nl_init, NULL,		\
		&xec_i2c_nl##inst##_data, &xec_i2c_nl##inst##_cfg,	\
		POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,			\
		&xec_i2c_nl_driver_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_DEVICE)
