/*
 * Copyright (c) 2026 Microchip Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#define DT_DRV_COMPAT microchip_mspi_xec_qmspi

#include <soc.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/clock/mchp_xec_pcr.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/policy.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mspi_xec_qmspi, CONFIG_MSPI_LOG_LEVEL);

#define XEC_MSPI_CFG_FLAG_SWMP_POS   0
#define XEC_MSPI_CFG_FLAG_REINIT_POS 1
#define XEC_MSPI_CFG_FLAG_DQS_POS    2

struct xec_mspi_cfg {
	uint32_t max_freq;
	struct gpio_dt_spec *ce_group;
	uint8_t num_ce_gpios;
	uint8_t num_periph;
	uint8_t flags;
};

/* driver configuration structure */
struct mspi_xec_qmspi_drv_cfg {
	mm_reg_t regbase;
	void (*config_irq)(void);
	const struct pinctrl_dev_config *pcfg;
	struct xec_mspi_cfg xmspicfg;
	uint8_t enc_pcr;
	uint8_t girq;
	uint8_t girq_pos;
};

struct mspi_xec_qmspi_drv_dat {
	volatile uint32_t qstatus;
	struct k_mutex lock;
	struct k_sem sync;
	struct xec_mspi_cfg active_xmspicfg;
};

static void inline qwr32(mm_reg_t qb, uint32_t ofs, uint32_t val)
{
	sys_write32(val, qb + ofs);
}

static uint32_t inline qrd32(mm_reg_t qb, uint32_t ofs)
{
	return sys_read32(qb + ofs);
}

static void qmspi_reset(mm_reg_t qb)
{
	mm_reg_t mode_reg = qb + XEC_QSPI_MODE_OFS;

	sys_set_bit(mode_reg, XEC_QSPI_MODE_SRST_POS);
	while (sys_test_bit(mode_reg, XEC_QSPI_MODE_SRST_POS) != 0) {
	}
}

#define XEC_QMSPI_FREQ_MAX MHZ(96)
#define XEC_QMSPI_FREQ_MIN ((XEC_QMSPI_FREQ_MAX) / (UINT16_MAX + 1U))

static uint32_t qmspi_calc_freq_div(uint32_t freq_hz)
{
	uint32_t fdiv = 1U;

	if (freq_hz >= XEC_QMSPI_FREQ_MAX) {
		return 1U; /* clamp to max (divide by 1) */
	}

	if (freq_hz <= XEC_QMSPI_FREQ_MIN) {
		return 0U; /* special HW value (divide by 2^16) */
	}

	return ((XEC_QMSPI_FREQ_MAX) / freq_hz);
}

static void qmspi_prog_freq(mm_reg_t qb, uint32_t freq_hz)
{
	uint32_t fdiv = qmspi_calc_freq_div(freq_hz);

	fdiv = XEC_QSPI_MODE_CK_DIV_SET(fdiv);
	soc_mmcr_mask_set(qb + XEC_QSPI_MODE_OFS, fdiv, XEC_QSPI_MODE_CK_DIV_MSK);
}

static int mspi_xec_qmspi_cfg(const struct device *controller)
{
	const struct mspi_xec_qmspi_drv_cfg *xcfg = controller->config;
	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;
	struct xec_mspi_cfg *acfg = &xdat->active_xmspicfg;
	mm_reg_t qb = xcfg->regbase;

	if ((acfg->flags & BIT(XEC_MSPI_CFG_FLAG_REINIT_POS)) != 0) {
		qmspi_reset(qb);
	}

	sys_clear_bit(qb + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_ACTV_POS);

	qmspi_prog_freq(qb, acfg->max_freq);

	/* TODO
	 * CPOL, CPHA_SDI, CPHA_SDO
	 * this is from struct mspi_dev_cfg
	 */

	return 0;
}

static int validate_mspi_cfg(const struct mspi_cfg *mcfg)
{
	if (mcfg->channel_num != 0) {
		LOG_ERR("MSPI CR cfg: chan 0 only");
		return -ENOTSUP;
	}

	if (mcfg->op_mode != MSPI_OP_MODE_CONTROLLER) {
		LOG_ERR("MSPI CR cfg: controller mode only");
		return -ENOTSUP;
	}

	if (mcfg->duplex != MSPI_HALF_DUPLEX) {
		LOG_ERR("MSPI CR cfg: half duplex mode only");
		return -ENOTSUP;
	}

	if (mcfg->dqs_support) {
		LOG_ERR("MSPI CR cfg: non-DQS mode only");
		return -ENOTSUP;
	}

	if (mcfg->sw_multi_periph == false) {
		LOG_ERR("MSPI CR cfg: sw_multi_periph must be true");
		return -ENOTSUP;
	}

	if ((mcfg->num_periph - mcfg->num_ce_gpios) > XEC_QSPI_MAX_CS) {
		LOG_ERR("MSPI CR cfg: two HW chip enables only");
		return -ENOTSUP;
	}

	if (mcfg->max_freq > MHZ(96)) {
		LOG_ERR("MSPI CR cfg: max_freq %u too large", mcfg->max_freq);
		return -ENOTSUP;
	}

	return 0;
}

static int api_mspi_xec_qmspi_config(const struct mspi_dt_spec *spec)
{
	const struct device *controller = spec->bus;
	const struct mspi_cfg *mcfg = &spec->config;
	int rc = 0;

	if (controller == NULL) {
		return -EINVAL;
	}

	rc = validate_mspi_cfg(mcfg);
	if (rc != 0) {
		return rc;
	}

	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;
	struct xec_mspi_cfg *acfg = &xdat->active_xmspicfg;

	acfg->max_freq = mcfg->max_freq;
	acfg->ce_group = mcfg->ce_group;
	acfg->num_ce_gpios = mcfg->num_ce_gpios;
	acfg->num_periph = mcfg->num_periph;
	acfg->flags = BIT(XEC_MSPI_CFG_FLAG_SWMP_POS);

	if (mcfg->re_init) {
		acfg->flags |= BIT(XEC_MSPI_CFG_FLAG_REINIT_POS);
	}

	return mspi_xec_qmspi_cfg(controller);
}

static int api_mspi_xec_qmspi_dev_config(const struct device *controller,
					 const struct mspi_dev_id *dev_id,
					 const enum mspi_dev_cfg_mask param_mask,
					 const struct mspi_dev_cfg *cfg)
{
	/* TODO implement */
	return 0;
}

static int api_mspi_xec_qmspi_get_chan_status(const struct device *controller, uint8_t ch)
{
	/* TODO implement */
	return 0;
}

static int api_mspi_xec_qmspi_transceive(const struct device *controller,
					 const struct mspi_dev_id *dev_id,
					 const struct mspi_xfer *req)
{
	/* TODO implement */
	return 0;
}

static int api_mspi_xec_qmspi_register_cb(const struct device *controller,
					  const struct mspi_dev_id *dev_id,
					  const enum mspi_bus_event evt_type,
					  mspi_callback_handler_t cb,
					  struct mspi_callback_context *ctx)
{
	/* TODO implement */
	return 0;
}

static int api_mspi_xec_timing_config(const struct device *controller,
				      const struct mspi_dev_id *dev_id, const uint32_t param_mask,
				      void *timing_cfg)
{
	/* TODO implement */
	return 0;
}

static void mspi_xec_qmspi_isr(const struct device *controller)
{
	/* TODO */
}

#ifdef CONFIG_PM_DEVICE
static int mspi_xec_qmspi_pm_action(const struct device *controller, enum pm_device_action action)
{
	const struct mspi_xec_qmspi_drv_cfg *xcfg = controller->config;
	mm_reg_t qb = xcfg->regbase;
	int rc = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		rc = pinctrl_apply_state(xcfg->pcfg, PINCTRL_STATE_DEFAULT);
		sys_set_bit(qb + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_ACTV_POS);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		sys_clear_bit(qb + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_ACTV_POS);
		rc = pinctrl_apply_state(xcfg->pcfg, PINCTRL_STATE_SLEEP);
		if (rc == -ENOENT) { /* pin sleep state not present */
			rc = 0;
		}
		break;
	default:
		rc = -ENOTSUP;
	}

	return rc;
}
#endif

#ifdef CONFIG_DEVICE_DEINIT_SUPPORT
static int mspi_xec_qmspi_deinit(const struct device *controller)
{
	/* TODO */
	return 0;
}
#endif

static int mspi_xec_qmspi_init(const struct device *controller)
{
	const struct mspi_xec_qmspi_drv_cfg *xcfg = controller->config;
	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;
	int rc = 0;

	k_mutex_init(&xdat->lock);
	k_sem_init(&xdat->sync, 0, 1);

	soc_xec_pcr_sleep_en_clear(xcfg->enc_pcr);

	rc = pinctrl_apply_state(xcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("MSPI XEC-QM PinCtrl default state error (%d)", rc);
		return rc;
	}

	if (xcfg->config_irq != NULL) {
		xcfg->config_irq();
		soc_ecia_girq_ctrl(xcfg->girq, xcfg->girq_pos, 1);
	}

	const struct mspi_dt_spec spec = {
		.bus = controller,
		.config = {
			.channel_num = 0,
			.op_mode = MSPI_OP_MODE_CONTROLLER,
			.duplex = MSPI_HALF_DUPLEX,
			.dqs_support = false,
			.sw_multi_periph = true,
			.ce_group = xcfg->xmspicfg.ce_group,
			.num_ce_gpios = xcfg->xmspicfg.num_ce_gpios,
			.num_periph = xcfg->xmspicfg.num_periph,
			.max_freq = xcfg->xmspicfg.max_freq,
			.re_init = true,
		},
	};

	return api_mspi_xec_qmspi_config(&spec);
}

static DEVICE_API(mspi, mspi_xec_qmspi_driver_api) = {
	.config = api_mspi_xec_qmspi_config,
	.dev_config = api_mspi_xec_qmspi_dev_config,
	.get_channel_status = api_mspi_xec_qmspi_get_chan_status,
	.transceive = api_mspi_xec_qmspi_transceive,
	.register_callback = api_mspi_xec_qmspi_register_cb,
	.timing_config = api_mspi_xec_timing_config,
};

#define MSPI_XEC_QMSPI_GIRQ(inst) MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, 0))
#define MSPI_XEC_QMSPI_GIRQ_POS(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, 0))

#define MSPI_XEC_QMSPI_IRQ_HANDLER(inst)                                                       \
	static void mspi_xec_qmspi_irq_cfg_func_##inst(void)                                   \
	{                                                                                      \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),                   \
			    mspi_xec_qmspi_isr, DEVICE_DT_INST_GET(inst), 0);                  \
		irq_enable(DT_INST_IRQN(inst));                                                \
	}

#define MSPI_MCHP_XEC_QMSPI_INIT(inst)                                                         \
                                                                                               \
	static struct gpio_dt_spec mspi_xec_qmspi_ce_gpios##inst[] = \
		MSPI_CE_GPIOS_DT_SPEC_INST_GET(inst);    \
                                                                                               \
	PINCTRL_DT_INST_DEFINE(inst);                                                          \
                                                                                               \
	MSPI_XEC_QMSPI_IRQ_HANDLER(inst)                                                       \
                                                                                               \
	PM_DEVICE_DT_INST_DEFINE(inst, mspi_xec_qmspi_pm_action);                              \
                                                                                               \
	static const struct mspi_xec_qmspi_drv_cfg mspi_xec_qmspi_xcfg_##inst = {              \
		.regbase = (mm_reg_t)DT_INST_REG_ADDR(inst),                                   \
		.config_irq = mspi_xec_qmspi_irq_cfg_func_##inst,                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                  \
		.xmspicfg = {                                                                  \
			.max_freq = DT_INST_PROP_OR(inst, clock_frequency, MHZ(96)),           \
			.ce_group = mspi_xec_qmspi_ce_gpios##inst,                             \
			.num_ce_gpios = ARRAY_SIZE(mspi_xec_qmspi_ce_gpios##inst),             \
			.num_periph = DT_INST_CHILD_NUM(inst),                                 \
			.flags = (uint8_t)BIT(XEC_MSPI_CFG_FLAG_SWMP_POS),                     \
		},                                                                             \
		.enc_pcr = DT_INST_PROP(inst, pcr),                                            \
		.girq = MSPI_XEC_QMSPI_GIRQ(inst),                                             \
		.girq_pos = MSPI_XEC_QMSPI_GIRQ_POS(inst),                                     \
	};                                                                                     \
	static struct mspi_xec_qmspi_drv_dat mspi_xec_qmspi_xdat_##inst;                       \
                                                                                               \
	DEVICE_DT_INST_DEINIT_DEFINE(inst, &mspi_xec_qmspi_init, &mspi_xec_qmspi_deinit,       \
				     PM_DEVICE_DT_INST_GET(inst),                              \
				     &mspi_xec_qmspi_xdat_##inst,                              \
				     &mspi_xec_qmspi_xcfg_##inst,                              \
				     POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,          \
				     &mspi_xec_qmspi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MSPI_MCHP_XEC_QMSPI_INIT)
