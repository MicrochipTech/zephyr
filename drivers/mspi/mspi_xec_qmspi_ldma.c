/*
 * Copyright (c) 2025, Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_mspi_ldma

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mspi_xec_ldma, CONFIG_MSPI_LOG_LEVEL);

#define XEC_QSPI_MAX_CHANNELS 1

struct mspi_mchp_xec_drvcfg {
	mm_reg_t regbase;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
	uint32_t dflt_clk_freq;
	struct mspi_cfg mspicfg;
	uint16_t enc_pcr;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t clk_taps[2];
	uint8_t ctrl_taps[2];
	uint8_t dcsda;
};

struct mspi_mchp_xec_data {
	volatile uint32_t hwstatus;
	const struct mspi_dev_id *dev_id;
	struct k_mutex lock;
	struct k_sem sync;
	struct mspi_dev_cfg dev_cfg;
	mspi_callback_handler_t cb;
	struct mspi_callback_context *cb_ctx;
};

/* Save register Boot-ROM may have touched based on OTP.
 * Do soft reset
 * Restore saved registers
 */
static void qspi_soft_reset(const struct device *controller)
{
	const struct mspi_mchp_xec_drvcfg *drvcfg = controller->config;
	mm_reg_t qb = drvcfg->regbase;
	uint32_t reg_save_restore[4] = {0};

	reg_save_restore[0] = sys_read32(qb + XEC_QSPI_CSTM_OFS);
	reg_save_restore[1] = sys_read32(qb + XEC_QSPI_TAPS_OFS);
	reg_save_restore[2] = sys_read32(qb + XEC_QSPI_TAPS_ADJ_OFS);
	reg_save_restore[3] = sys_read32(qb + XEC_QSPI_TAPS_CR2_OFS);

	sys_set_bit(qb + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_SRST_POS);
	while (sys_test_bit(qb + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_SRST_POS) != 0) {
		;
	}

	sys_write32(reg_save_restore[0], qb + XEC_QSPI_CSTM_OFS);
	sys_write32(reg_save_restore[1], qb + XEC_QSPI_TAPS_OFS);
	sys_write32(reg_save_restore[2], qb + XEC_QSPI_TAPS_ADJ_OFS);
	sys_write32(reg_save_restore[3], qb + XEC_QSPI_TAPS_CR2_OFS);
}

/* Calculate the frequency divider field from requested frequency in hertz.
 * QSPI frequency divider uses values [0, 0xffff] where the non-zero values
 * are treated as the actual divider and 0 is a divider of 0x10000.
 */
static uint32_t qspi_calc_fdiv(uint32_t freq_hz)
{
	uint32_t fdiv = 1u; /* divide by 1 */

	if (freq_hz < CONFIG_MSPI_XEC_QMSPI_LDMA_MAX_FREQ) {
		fdiv = CONFIG_MSPI_XEC_QMSPI_LDMA_MAX_FREQ / freq_hz;
		if (fdiv > XEC_QSPI_FDIV_MAX) {
			fdiv = 0;
		}
	}

	return fdiv;
}

/* Configure controller based on struct mspi_cfg parameters.
 * QSPI is a controller only SPI peripheral supporting full-duplex, dual, and quad
 * protocols. Half-duplex operation requires connecting IO0 and IO1 together on the board.
 * This driver is not supporting half-duplex at this time. Do not call this internal
 * routine unless mspi_mchp_xec_ctrl_cfg_valid has been called to validate the configuration.
 * The controller configuration structure does not include IO mode (full-duplex, dual, quad)
 * information. We configure the controller for full-duplex.
 */
static int mspi_mchp_xec_qspi_cfg(const struct device *controller, const struct mspi_cfg *mspicfg)
{
	const struct mspi_mchp_xec_drvcfg *drvcfg = controller->config;
	mm_reg_t rb = drvcfg->regbase;
	uint32_t d = XEC_QSPI_CR_IFM_SET(XEC_QSPI_CR_IFM_FD) | BIT(XEC_QSPI_CR_DESCR_EN_POS);
	uint32_t fdiv = 1u;

	sys_clear_bit(rb + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_ACTV_POS);
	soc_xec_pcr_sleep_en_clear(drvcfg->enc_pcr);
	soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 0);

	if (mspicfg->re_init == true) {
		qspi_soft_reset(controller);
	}

	/* set to full-duplex and descriptor mode enabled */
	sys_write32(d, rb + XEC_QSPI_CR_OFS);

	/* calculate and program frequency */
	fdiv = qspi_calc_fdiv(mspicfg->max_freq);
	d = sys_read32(rb + XEC_QSPI_MODE_OFS);
	d &= ~(XEC_QSPI_MODE_CK_DIV_MSK);
	d |= XEC_QSPI_MODE_CK_DIV_SET(d);
	sys_write32(d, rb + XEC_QSPI_MODE_OFS);

	sys_write32(rb + XEC_QSPI_SR_OFS, UINT32_MAX);
	soc_ecia_girq_status_clear(drvcfg->girq, drvcfg->girq_pos);
	soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 1);

	sys_set_bit(rb + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_ACTV_POS);

	return 0;
}

/* Check MPSI controller configuration */
static int mspi_mchp_xec_ctrl_cfg_valid(const struct mspi_cfg *mspicfg)
{
	if (mspicfg->op_mode != MSPI_OP_MODE_CONTROLLER) {
		LOG_ERR("XEC MSPI supports controller mode only");
		return -ENOTSUP;
	}

	if (mspicfg->max_freq > CONFIG_MSPI_XEC_QMSPI_LDMA_MAX_FREQ) {
		LOG_ERR("Max freq exceeds %u", mspicfg->max_freq);
		return -ENOTSUP;
	}

	if (mspicfg->duplex != MSPI_FULL_DUPLEX) {
		LOG_ERR("XEC MSPI supports full duplex only");
		return -ENOTSUP;
	}

	if (mspicfg->channel_num >= XEC_QSPI_MAX_CHANNELS) {
		LOG_ERR("XEC MPSI supports one controller only");
		return -ENOTSUP;
	}

	if (mspicfg->dqs_support == true) {
		LOG_ERR("XEC MSPI does not support DQS");
		return -ENOTSUP;
	}

	if (mspicfg->sw_multi_periph == false) {
		LOG_ERR("XEC MSPI requires SW multi-periph");
		return -ENOTSUP;
	}

	return 0;
}

static int mspi_mchp_xec_config(const struct mspi_dt_spec *spec)
{
	if ((spec == NULL) || (spec->bus == NULL)) {
		return -EINVAL;
	}

	if (soc_taf_enabled() != 0) {
		LOG_ERR("init: XEC MSPI owned by TAF");
		return -EACCES;
	}

	const struct device *controller = spec->bus;
	const struct mspi_cfg *mspicfg = &spec->config;
	const struct mspi_mchp_xec_drvcfg *drvcfg = controller->config;
	mm_reg_t rb = drvcfg->regbase;
	int ret = 0;

	ret = mspi_mchp_xec_ctrl_cfg_valid(mspicfg);
	if (ret != 0) {
		return ret;
	}

	soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 0);

	if (mspicfg->re_init == true) {
		qspi_soft_reset(controller);
	}

	sys_clear_bit(rb + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_ACTV_POS);

	ret = mspi_mchp_xec_qspi_cfg(controller, mspicfg);

	sys_write32(rb + XEC_QSPI_SR_OFS, UINT32_MAX);
	soc_ecia_girq_status_clear(drvcfg->girq, drvcfg->girq_pos);
	soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 1);

	sys_set_bit(rb + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_ACTV_POS);

	return ret;
}

/* Configure controller with device parameters specified in param_mask.
 * These include chip select, frequency, IO mode, data rate, clock polarity,
 * data transmit and receive phases, data endianess, tri-state clocks, and
 * device specific commands.
 */
#if 0
struct mspi_dev_id {
	struct gpio_dt_spec ce; /* device gpio ce */
	uint16_t dev_idx; /* device index on DT */
}
#endif
static int mspi_mchp_xec_dev_config(const struct device *controller,
                                    const struct mspi_dev_id *dev_id,
                                    const enum mspi_dev_cfg_mask param_mask,
                                    const struct mspi_dev_cfg *cfg)
{
	const struct mspi_mchp_xec_drvcfg *drvcfg = controller->config;
	struct mspi_mchp_xec_data *const data = controller->data;
	mm_reg_t rb = drvcfg->regbase;
	bool locked = false;

	if (soc_taf_enabled() != 0) {
		LOG_ERR("init: XEC MSPI owned by TAF");
		return -EACCES;
	}

	if ((dev_id == NULL) || (cfg == NULL)) {
		return -EINVAL;
	}

	if (data->dev_id != dev_id) {
		if (k_mutex_lock(&data->lock, K_MSEC(CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE))) {
			LOG_ERR("Lock timeout");
			return -EBUSY;
		}
		locked = true;
	}

	/* TODO - it is conditioned with && (drvcfg->mspicfg.sw_multi_periph == false) */
	if ((param_mask == MSPI_DEVICE_CONFIG_NONE) &&
	    (drvcfg->mspicfg.sw_multi_periph == false)) {
		data->dev_id = dev_id;
		goto dev_config_exit;
	}

	/* TODO */

dev_config_exit:
	if (locked == true) {
		k_mutex_unlock(&data->lock);
	}

	return 0;
}

static int mspi_mchp_xec_timing_config(const struct device *controller,
                                       const struct mspi_dev_id *dev_id,
                                       const uint32_t param_mask, void *timing_cfg)
{
	return 0;
}

static int mspi_mchp_xec_get_channel_status(const struct device *controller, uint8_t ch)
{
	return 0;
}

static int mspi_mchp_xec_register_callback(const struct device *controller,
                                           const struct mspi_dev_id *dev_id,
                                           const enum mspi_bus_event evt_type,
                                           mspi_callback_handler_t cb,
                                           struct mspi_callback_context *ctx)
{
	return 0;
}

static int mspi_mchp_xec_transceive(const struct device *controller,
                                    const struct mspi_dev_id *dev_id,
                                    const struct mspi_xfer *req)
{
	return 0;
}

static void mspi_mchp_xec_isr(const struct device *controller)
{
	/* ISR TODO */
}

static int mspi_mchp_xec_init(const struct device *controller)
{
	const struct mspi_mchp_xec_drvcfg *drvcfg = controller->config;
	int ret = 0;

	if (soc_taf_enabled() != 0) {
		LOG_ERR("init: XEC MSPI owned by TAF");
		return -EACCES;
	}

	soc_xec_pcr_sleep_en_clear(drvcfg->enc_pcr);

	qspi_soft_reset(controller);

	ret = pinctrl_apply_state(drvcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("XEC MSPI pinctrl apply state failed (%d)", ret);
		return ret;
	}

	if (drvcfg->irq_config_func != NULL) {
		drvcfg->irq_config_func();
		soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 1u);
	}

	const struct mspi_dt_spec mspi_spec = {
		.bus = controller,
		.config = drvcfg->mspicfg,
	};

	ret = mspi_mchp_xec_config(&mspi_spec);
	if (ret != 0) {
		LOG_ERR("XEC MSPI init config error (%d)", ret);
		return -EIO;
	}

	return 0;
}

static DEVICE_API(mspi, mspi_mchp_xec_driver_api) = {
	.config             = mspi_mchp_xec_config,
	.dev_config         = mspi_mchp_xec_dev_config,
	.timing_config      = mspi_mchp_xec_timing_config,
	.get_channel_status = mspi_mchp_xec_get_channel_status,
	.register_callback  = mspi_mchp_xec_register_callback,
	.transceive         = mspi_mchp_xec_transceive,
};

#define XEC_QSPI_DT_GIRQ(inst) MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, 0))
#define XEC_QSPI_DT_GIRQ_POS(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, 0))

#define XEC_QSPI_DT_TAPS(inst, prop, idx) \
	COND_CODE_1(DT_PROP_HAS_IDX(inst, prop, idx), (DT_INST_PROP_BY_IDX(inst, prop, idx)), (0))

/* MSPI control config */
#define MCHP_XEC_MSPI_CONFIG(index)                                                            \
	{                                                                                      \
		.channel_num = 0,                                                              \
		.op_mode = DT_INST_ENUM_IDX_OR(index, op_mode, MSPI_OP_MODE_CONTROLLER),       \
		.duplex = DT_INST_ENUM_IDX_OR(index, duplex, MSPI_FULL_DUPLEX),                \
		.max_freq = DT_INST_PROP_OR(index, clock_frequency, MHZ(12)),                  \
		.dqs_support = false, /* MCHP XEC QSPI doesn't support DQS */                  \
		.num_periph = DT_INST_CHILD_NUM(index),                                        \
		.sw_multi_periph = DT_INST_PROP(index, software_multiperipheral),              \
		.num_ce_gpios = ARRAY_SIZE(ce_gpios##index),                                   \
		.ce_group = ce_gpios##index,                                                   \
	}

#define MCHP_XEC_MSPI_DEFINE(inst)                                                          \
	PINCTRL_DT_INST_DEFINE(inst);                                                       \
	static void mspi_mec_irq_config##inst(void)                                         \
	{                                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),                \
			    mspi_mchp_xec_isr, DEVICE_DT_INST_GET(inst), 0);                \
		irq_enable(DT_INST_IRQN(inst));                                             \
	}                                                                                   \
	static struct gpio_dt_spec ce_gpios##inst[] = MSPI_CE_GPIOS_DT_SPEC_INST_GET(inst); \
	static struct mspi_mchp_xec_data mspi_mec_data##inst = {                            \
		.hwstatus = 0,                                                              \
		.dev_id = NULL,                                                             \
		.lock = Z_MUTEX_INITIALIZER(mspi_mec_data##inst.lock),                      \
		.sync = Z_SEM_INITIALIZER(mspi_mec_data##inst.sync, 0, 1),                  \
		.dev_cfg = {0},                                                             \
	};                                                                                  \
	static const struct mspi_mchp_xec_drvcfg mspi_mec_drvcfg##inst = {                  \
		.regbase = DT_INST_REG_ADDR(inst),                                          \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                               \
		.irq_config_func = mspi_mec_irq_config##inst,                               \
		.dflt_clk_freq = DT_INST_PROP_OR(inst, clock_frequency, MHZ(12)),           \
		.mspicfg = MCHP_XEC_MSPI_CONFIG(inst),                                      \
		.enc_pcr = DT_INST_PROP(inst, pcr),                                         \
		.girq = XEC_QSPI_DT_GIRQ(inst),                                             \
		.girq_pos = XEC_QSPI_DT_GIRQ(inst),                                         \
		.clk_taps[0] = (uint8_t)XEC_QSPI_DT_TAPS(inst, clock_taps, 0),              \
		.clk_taps[1] = (uint8_t)XEC_QSPI_DT_TAPS(inst, clock_taps, 1),              \
		.ctrl_taps[0] = (uint8_t)XEC_QSPI_DT_TAPS(inst, ctrl_taps, 0),              \
		.ctrl_taps[1] = (uint8_t)XEC_QSPI_DT_TAPS(inst, ctrl_taps, 1),              \
		.dcsda = DT_INST_PROP_OR(inst, dcsda, 6),                                   \
	};                                                                                  \
	DEVICE_DT_INST_DEFINE(inst,                                                         \
			      mspi_mchp_xec_init,                                           \
			      NULL,                                                         \
			      &mspi_mec_data##inst,                                         \
			      &mspi_mec_drvcfg##inst,                                       \
			      POST_KERNEL,                                                  \
			      CONFIG_MSPI_INIT_PRIORITY,                                    \
			      &mspi_mchp_xec_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MCHP_XEC_MSPI_DEFINE)
