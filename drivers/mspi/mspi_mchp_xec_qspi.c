/*
 * Copyright (c) 2026, Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_qspi_mspi

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(mspi_xec_qspi, CONFIG_MSPI_LOG_LEVEL);

#define XEC_QSPI_MAX_CHANNELS 1

enum mchp_xec_ce_type {
	MCHP_XEC_CE_HW = 0,
	MCHP_XEC_CE_GPIO,
};

struct mchp_xec_target_cfg {
	uint8_t target_id;
	enum mchp_xec_ce_type ce_type;
	uint8_t hw_ce_num;
	struct gpio_dt_spec gpio_ce;
	uint32_t max_freq;
};

struct mchp_xec_mspi_cfg {
	mm_reg_t regbase;
	void (*config_irq)(void);
	const struct pinctrl_dev_config *pcfg;
	const struct mchp_xec_target_cfg *targets;
	struct mspi_cfg mspicfg;
	size_t num_targets;
	size_t num_ce_gpios;
	uint16_t enc_pcr;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t num_hw_ce;
};

struct mchp_xec_mspi_data {
	struct k_mutex lock;
	struct k_sem sync;
	volatile uint32_t qstatus;
	const struct mspi_dev_id *dev_id;
	struct mspi_cfg drv_cfg;
	struct mspi_dev_cfg dev_cfg;
	mspi_callback_handler_t cb[MSPI_BUS_EVENT_MAX];
	struct mspi_callback_context *cb_ctx[MSPI_BUS_EVENT_MAX];
};

void debug_pr_mspi_dev_id(const struct mspi_dev_id *did)
{
	LOG_INF("struct mspi_dev_id at 0x%08x", (uint32_t)did);
	LOG_INF("  ce.port = 0x%08x", (uint32_t)did->ce.port);
	LOG_INF("  ce.pin = %u", did->ce.pin);
	LOG_INF("  ce.dt_flags = 0x%0x", did->ce.dt_flags);
	LOG_INF("  dev_idx = %u", did->dev_idx);
}

void debug_pr_mspi_cfg(const struct mspi_cfg *mcfg)
{
	LOG_INF("struct mspi_cfg at 0x%08x", (uint32_t)mcfg);
	LOG_INF("  channel_num = 0x%02x",mcfg->channel_num);
	LOG_INF("  op_mode = %u", mcfg->op_mode);
	LOG_INF("  duplex = %u", mcfg->duplex);
	LOG_INF("  dqs_support = %u", mcfg->dqs_support);
	LOG_INF("  sw_multi_periph = %u", mcfg->sw_multi_periph);
	LOG_INF("  ce_group at 0x%08x", (uint32_t)mcfg->ce_group);
	LOG_INF("  num_ce_gpios = %u", mcfg->num_ce_gpios);
	LOG_INF("  num_periph = %u", mcfg->num_periph);
	LOG_INF("  max_freq = %u", mcfg->max_freq);
	LOG_INF("  re_init = %u", mcfg->re_init);
	for (uint32_t i = 0; i < mcfg->num_ce_gpios; i++) {
		struct gpio_dt_spec *g = &mcfg->ce_group[i];

		LOG_INF("  ce_gpios[%u].port at 0x%0x", i, (uint32_t)g->port);
		LOG_INF("  ce_gpios[%u].pin = %u", i, g->pin);
		LOG_INF("  ce_gpios[%u].dt_flags = 0x%0x", i, g->dt_flags);

	}
}

void debug_pr_mspi_dev_cfg(const struct mspi_dev_cfg *dcfg)
{
	LOG_INF("struct mspi_dev_cfg at 0x%08x", (uint32_t)dcfg);
	LOG_INF("  ce_num = %u", dcfg->ce_num);
	LOG_INF("  freq = %u", dcfg->freq);
	LOG_INF("  io_mode = %u", dcfg->io_mode);
	LOG_INF("  data_rate = %u", dcfg->data_rate);
	LOG_INF("  cpp = %u", dcfg->cpp);
	LOG_INF("  endian = %u", dcfg->endian);
	LOG_INF("  ce_polarity = %u", dcfg->ce_polarity);
	LOG_INF("  dqs_enable = %u", dcfg->dqs_enable);
	LOG_INF("  rx_dummy = %u", dcfg->rx_dummy);
	LOG_INF("  tx_dummy = %u", dcfg->tx_dummy);
	LOG_INF("  read_cmd = %u", dcfg->read_cmd);
	LOG_INF("  write_cmd = %u", dcfg->write_cmd);
	LOG_INF("  cmd_length = %u", dcfg->cmd_length);
	LOG_INF("  addr_length = %u", dcfg->addr_length);
	LOG_INF("  mem_boundary = %u", dcfg->mem_boundary);
	LOG_INF("  time_to_break = %u", dcfg->time_to_break);
}

void debug_pr_mspi_xfr_packet(const struct mspi_xfer_packet *pkt)
{
	if (pkt == NULL) {
		LOG_INF("NULL xfer packet");
		return;
	}

	LOG_INF("struct mspi_xfer_packet at 0x%08x", (uint32_t)pkt);
	LOG_INF("  dir = %u", pkt->dir);
	LOG_INF("  cb_mask = 0x%0x", pkt->cb_mask);
	LOG_INF("  cmd = 0x%0x", pkt->cmd);
	LOG_INF("  address = 0x%0x", pkt->address);
	LOG_INF("  num_bytes = %u", pkt->num_bytes);
	LOG_INF("  data_buf = 0x%08x", (uint32_t)pkt->data_buf);
}

void debug_pr_mspi_xfr(const struct mspi_xfer *xfr)
{
	if (xfr == NULL) {
		LOG_INF("NULL mspi_xfr");
		return;
	}

	LOG_INF("struct mspi_xfer at 0x%08x", (uint32_t)xfr);
	LOG_INF("  async = %u", (uint32_t)xfr->async);
	LOG_INF("  xfer_mode = %u", xfr->xfer_mode);
	LOG_INF("  tx_dummy = %u", xfr->tx_dummy);
	LOG_INF("  rx_dummy = %u", xfr->rx_dummy);
	LOG_INF("  cmd_length = %u", xfr->cmd_length);
	LOG_INF("  addr_length = %u", xfr->addr_length);
	LOG_INF("  hold_ce = %u", xfr->hold_ce);
	LOG_INF("  ce_sw_ctrl.gpio.port = 0x%0x", (uint32_t)xfr->ce_sw_ctrl.gpio.port);
	LOG_INF("  ce_sw_ctrl.gpio.pin = %u", (uint32_t)xfr->ce_sw_ctrl.gpio.pin);
	LOG_INF("  ce_sw_ctrl.gpio.dt_flags = 0x%0x", (uint32_t)xfr->ce_sw_ctrl.gpio.dt_flags);
	LOG_INF("  ce_sw_ctrl.delay = %u", xfr->ce_sw_ctrl.delay);
	LOG_INF("  priority = %u", xfr->priority);
	LOG_INF("  packets = 0x%08x", (uint32_t)xfr->packets);
	LOG_INF("  num_packet = %u", xfr->num_packet);
	LOG_INF("  timeout = %u", xfr->timeout);
	for (uint32_t i = 0; i < xfr->num_packet; i++) {
		const struct mspi_xfer_packet *p = &xfr->packets[i];

		debug_pr_mspi_xfr_packet(p);
	}
}

static const struct mchp_xec_target_cfg *get_target_cfg_by_id(const struct device *controller,
                                                              uint8_t target_id)
{
	const struct mchp_xec_mspi_cfg *xcfg = controller->config;

	for (size_t n = 0; n < xcfg->num_targets; n++) {
		const struct mchp_xec_target_cfg *tcfg = &xcfg->targets[n];

		if (tcfg->target_id == target_id) {
			return tcfg;
		}
	}

	return NULL;
}

static const struct mchp_xec_target_cfg *get_target_cfg_by_id_type(const struct device *controller,
                                                                    uint8_t target_id,
                                                                    enum mchp_xec_ce_type ce_type)
{
	const struct mchp_xec_mspi_cfg *xcfg = controller->config;

	for (size_t n = 0; n < xcfg->num_targets; n++) {
		const struct mchp_xec_target_cfg *tcfg = &xcfg->targets[n];

		if ((tcfg->target_id == target_id) && (tcfg->ce_type == ce_type)) {
			return tcfg;
		}
	}

	return NULL;
}

static int xec_mspi_cfg_valid(const struct mspi_cfg *ctrl_cfg)
{
	uint32_t max_qspi_freq = soc_core_clock_get();

	if (ctrl_cfg->op_mode != MSPI_OP_MODE_CONTROLLER) {
		LOG_ERR("XEC MSPI supports controller mode only");
		return -ENOTSUP;
	}

	if (ctrl_cfg->max_freq > max_qspi_freq) {
		LOG_ERR("Max freq exceeds %u", ctrl_cfg->max_freq);
		return -ENOTSUP;
	}

	if (ctrl_cfg->duplex != MSPI_HALF_DUPLEX) {
		LOG_ERR("XEC MSPI driver supports half-duplex only");
		return -ENOTSUP;
	}

	if (ctrl_cfg->channel_num >= XEC_QSPI_MAX_CHANNELS) {
		LOG_ERR("XEC MPSI supports one controller only");
		return -ENOTSUP;
	}

	if (ctrl_cfg->dqs_support == true) {
		LOG_ERR("XEC MSPI does not support DQS");
		return -ENOTSUP;
	}

	if (ctrl_cfg->sw_multi_periph == false) {
		LOG_ERR("XEC MSPI requires SW multi-periph");
		return -ENOTSUP;
	}

	if ((ctrl_cfg->num_periph == 0) || (ctrl_cfg->num_periph < ctrl_cfg->num_ce_gpios)) {
		return -EINVAL;
	}

	return 0;
}

/* XEC QSPI controller supports single, dual, or quad only I/O lines */
static bool iom_is_supported(enum mspi_io_mode io_mode)
{
	if (io_mode >= MSPI_IO_MODE_OCTAL) {
		return false;
	}

	return true;
}

static int xec_hw_mspi_cfg(const struct device *controller, const struct mspi_cfg *ctrl_cfg)
{
	const struct mchp_xec_mspi_cfg *xcfg = controller->config;
	mm_reg_t qb = xcfg->regbase;
	uint32_t d = XEC_QSPI_CR_IFM_SET(XEC_QSPI_CR_IFM_FD) | BIT(XEC_QSPI_CR_DESCR_EN_POS);
	uint32_t fdiv = 0;

	LOG_INF("XEC HW MSPI config");

	soc_ecia_girq_ctrl(xcfg->girq, xcfg->girq_pos, 0);

	sys_clear_bit(qb + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_ACTV_POS);

	if (ctrl_cfg->re_init == true) {
		LOG_INF("XEC HW MSPI config do soft reset");
		soc_xec_qspi_soft_reset(qb);
	}

	/* set to full-duplex and descriptor mode enabled */
	sys_write32(d, qb + XEC_QSPI_CR_OFS);

	/* calculate and program frequency */
	if (soc_xec_qspi_calc_fdiv(ctrl_cfg->max_freq, &fdiv) < 0) {
		LOG_ERR("Bad default DT freq %u defaulting to safe value", ctrl_cfg->max_freq);
		fdiv = 8U; /* TODO */
	}

	fdiv = XEC_QSPI_MODE_CK_DIV_SET(fdiv);
	soc_mmcr_mask_set(qb + XEC_QSPI_MODE_OFS, fdiv, XEC_QSPI_MODE_CK_DIV_MSK);

	sys_write32(UINT32_MAX, qb + XEC_QSPI_SR_OFS);
	soc_ecia_girq_status_clear(xcfg->girq, xcfg->girq_pos);
	soc_ecia_girq_ctrl(xcfg->girq, xcfg->girq_pos, 1);

	sys_set_bit(qb + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_ACTV_POS);

	return 0;
}

static int xec_mspi_ctrl_cfg(const struct device *controller, const struct mspi_cfg *ctrl_cfg)
{
	const struct mchp_xec_mspi_cfg *xcfg = controller->config;
	mm_reg_t qb = xcfg->regbase;
	int rc = 0;

	debug_pr_mspi_cfg(ctrl_cfg);

	rc = xec_mspi_cfg_valid(ctrl_cfg);
	if (rc != 0) {
		LOG_ERR("MSPI Ctrl config not valid (%d)", rc);
		return rc;
	}

	rc = xec_hw_mspi_cfg(controller, ctrl_cfg);
	if (rc != 0) {
		LOG_ERR("MSPI QSPI config error (%d)", rc);
		return rc;
	}

	sys_write32(UINT32_MAX, qb + XEC_QSPI_SR_OFS);
	soc_ecia_girq_status_clear(xcfg->girq, xcfg->girq_pos);
	soc_ecia_girq_ctrl(xcfg->girq, xcfg->girq_pos, 1);

	sys_set_bit(qb + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_ACTV_POS);

	return 0;
}

/* API - Configure QSPI controller */
static int mchp_xec_mspi_config(const struct mspi_dt_spec *spec)
{
	LOG_INF("XEC MSPI config");

	if ((spec == NULL) || (spec->bus == NULL)) {
		LOG_ERR("Null spec or spec->bus");
		return -EINVAL;
	}

	if (soc_taf_enabled() != 0) {
		LOG_ERR("init: XEC MSPI owned by TAF");
		return -EACCES;
	}

	return xec_mspi_ctrl_cfg(spec->bus, &spec->config);
}

/* -------- MSPI Device Configuration -------- */
#define MCHP_XEC_MSPI_DEV_CFG_MSK \
	(MSPI_DEVICE_CONFIG_CE_NUM | MSPI_DEVICE_CONFIG_FREQUENCY | MSPI_DEVICE_CONFIG_IO_MODE |\
	 MSPI_DEVICE_CONFIG_CPP | MSPI_DEVICE_CONFIG_RX_DUMMY | MSPI_DEVICE_CONFIG_TX_DUMMY |\
	 MSPI_DEVICE_CONFIG_READ_CMD | MSPI_DEVICE_CONFIG_WRITE_CMD | MSPI_DEVICE_CONFIG_CMD_LEN |\
	 MSPI_DEVICE_CONFIG_ADDR_LEN)

static int xec_mspi_dev_config(const struct device *controller, uint32_t pmsk,
                               const struct mspi_dev_cfg *cfg)
{
	const struct mchp_xec_mspi_cfg *xcfg = controller->config;
	struct mchp_xec_mspi_data *const xdat = controller->data;

	if ((pmsk & MSPI_DEVICE_CONFIG_CE_NUM) != 0) {
		if (cfg->ce_num >= (xcfg->num_targets)) {
			LOG_ERR("XEC MSPI DevCfg ce_num (%u) >= num targets (%u)", cfg->ce_num,
			        xcfg->num_targets);
			return -ENOTSUP;
		}
		xdat->dev_cfg.ce_num = cfg->ce_num;
	}

	if ((pmsk & MSPI_DEVICE_CONFIG_FREQUENCY) != 0) {
		if (cfg->freq > xcfg->mspicfg.max_freq) {
			LOG_ERR("XEC MSPI DevCfg freq (%u) >= max (%u)", cfg->freq,
			        xcfg->mspicfg.max_freq);
			return -ENOTSUP;
		}
	}

	if ((pmsk & MSPI_DEVICE_CONFIG_IO_MODE) != 0) {
		if (iom_is_supported(cfg->io_mode) == false) {
			LOG_ERR("XEC MSPI DevCfg io_mode (%u) not supported", cfg->io_mode);
			return -ENOTSUP;
		}
		xdat->dev_cfg.io_mode = cfg->io_mode;
	}

	if ((pmsk & MSPI_DEVICE_CONFIG_CPP) != 0) {
		if (cfg->cpp > MSPI_CPP_MODE_3) {
			LOG_ERR("XEC MSPI DevCfg unknown CPP  (%u)", cfg->cpp);
			return -EINVAL;
		}
		xdat->dev_cfg.cpp = cfg->cpp;
	}

	if ((pmsk & MSPI_DEVICE_CONFIG_RX_DUMMY) != 0) {
		if (cfg->rx_dummy > 0x7FFFU) {
			LOG_ERR("XEC MSPI DevCfg rx dummy (0x%0x) > max HW 0x7FFF cycles",
			        cfg->rx_dummy);
			return -ENOTSUP;
		}
		xdat->dev_cfg.rx_dummy = cfg->rx_dummy;
	}

	if ((pmsk & MSPI_DEVICE_CONFIG_TX_DUMMY) != 0) {
		if (cfg->tx_dummy > 0x7FFFU) {
			LOG_ERR("XEC MSPI DevCfg tx dummy (0x%0x) > max HW 0x7FFF cycles",
			        cfg->tx_dummy);
			return -ENOTSUP;
		}
		xdat->dev_cfg.tx_dummy = cfg->tx_dummy;
	}

	if ((pmsk & MSPI_DEVICE_CONFIG_READ_CMD) != 0) {
		xdat->dev_cfg.read_cmd = cfg->read_cmd;
	}

	if ((pmsk & MSPI_DEVICE_CONFIG_WRITE_CMD) != 0) {
		xdat->dev_cfg.write_cmd = cfg->write_cmd;
	}

	if ((pmsk & MSPI_DEVICE_CONFIG_CMD_LEN) != 0) {
		if (cfg->cmd_length > 4U) {
			LOG_ERR("XEC MSPI DevCfg cmd len (%u) > 4", cfg->cmd_length);
			return -ENOTSUP;
		}
		xdat->dev_cfg.cmd_length = cfg->cmd_length;
	}

	if ((pmsk & MSPI_DEVICE_CONFIG_ADDR_LEN) != 0) {
		if (cfg->addr_length > 4U) {
			LOG_ERR("XEC MSPI DevCfg cmd len (%u) > 4", cfg->addr_length);
			return -ENOTSUP;
		}
		xdat->dev_cfg.addr_length = cfg->addr_length;
	}

	return 0;
}

/* Attached device configuration API implementation
 * Driver supports modifying these parameters:
 * MSPI_DEVICE_CONFIG_NONE
 * MSPI_DEVICE_CONFIG_CE_NUM
 * MSPI_DEVICE_CONFIG_FREQUENCY
 * MSPI_DEVICE_CONFIG_IO_MODE
 * MSPI_DEVICE_CONFIG_CPP
 * MSPI_DEVICE_CONFIG_RX_DUMMY
 * MSPI_DEVICE_CONFIG_TX_DUMMY
 * MSPI_DEVICE_CONFIG_READ_CMD
 * MSPI_DEVICE_CONFIG_WRITE_CMD
 * MSPI_DEVICE_CONFIG_CMD_LEN
 * MSPI_DEVICE_CONFIG_ADDR_LEN
 * MSPI_DEVICE_CONFIG_ALL - we all the above supported parameters
 *
 * Do not support changing:
 * MSPI_DEVICE_CONFIG_DATA_RATE must be MSPI_DATA_RATE_SINGLE for all phases
 * MSPI_DEVICE_CONFIG_ENDIAN must be MSPI_XFER_LITTLE_ENDIAN
 * MSPI_DEVICE_CE_POL must be MSPI_CE_ACTIVE_LOW, we do support MSPI_DEVICE_CONFIG_CPP modes 0-3
 * MSPI_DEVICE_CONFIG_DQS (data strobe signal). Our QSPI does not support
 * MSPI_DEVICE_CONFIG_MEM_BOUND - target device has restricted address boundary or refresh
 *                                requirements. Driver does not support at this time.
 * MPSI_DEVICE_CONFIG_BREAK_TIME - Used in with MEM_BOUND for controller to pause a transfer
 *                                 Our HW does not support.
 *
 *
 */
static int mchp_xec_mspi_dev_config(const struct device *controller,
                                    const struct mspi_dev_id *dev_id,
                                    const enum mspi_dev_cfg_mask param_mask,
                                    const struct mspi_dev_cfg *cfg)
{
	struct mchp_xec_mspi_data *const xdat = controller->data;
	uint32_t pmsk = 0;
	int rc = 0;

	LOG_INF("XEC MSPI dev_config");

	LOG_INF("param_mask = 0x%08x", (uint32_t)param_mask);
	debug_pr_mspi_dev_id(dev_id);
	debug_pr_mspi_dev_cfg(cfg);

	if ((dev_id == NULL) || (cfg == NULL)) {
		return -EINVAL;
	}

	if (param_mask == MSPI_DEVICE_CONFIG_NONE) {
		/* change target device only */
		/* TODO do we need verification?
		 * struct mspi_dev_id has members ce (struct gpio_dt_spec) and dev_idx
		 * an index into DT. If the caller has a different DT based on runtime
		 * board jumpers, etc. we can only check if the caller also updated
		 * struct mspi_cfg via the mspi_config() API. This requires an instance of
		 * struct mspi_cfg to be located in driver data!
		 */
		xdat->dev_id = dev_id;
		return 0;
	}

	if ((param_mask & ~(MCHP_XEC_MSPI_DEV_CFG_MSK)) != 0) {
		return -ENOTSUP;
	} else if ((param_mask & MSPI_DEVICE_CONFIG_ALL) != 0) {
		pmsk = MCHP_XEC_MSPI_DEV_CFG_MSK;
	} else {
		pmsk = (uint32_t)param_mask;
	}

	rc = xec_mspi_dev_config(controller, pmsk, cfg);
	if (rc != 0) {
		return rc;
	}

	xdat->dev_id = dev_id;

	return 0;
}

/* Attached device timing API implementation (optional) */
static int mchp_xec_mspi_timing_config(const struct device *controller,
                                       const struct mspi_dev_id *dev_id,
                                       const uint32_t param_mask, void *timing_cfg)
{
	LOG_INF("XEC MSPI timing config");

	/* TODO */
	return 0;
}

/* Get channel status API implementation. QSPI support one channel */
static int mchp_xec_mspi_get_channel_status(const struct device *controller, uint8_t ch)
{
	LOG_INF("XEC MSPI get chan status");

	/* TODO */
	return 0;
}

/* Register callback API implementation */
static int mchp_xec_mspi_register_callback(const struct device *controller,
                                           const struct mspi_dev_id *dev_id,
                                           const enum mspi_bus_event evt_type,
                                           mspi_callback_handler_t cb,
                                           struct mspi_callback_context *ctx)
{
	struct mchp_xec_mspi_data *const xdat = controller->data;

	LOG_INF("XEC MSPI register callback");

	if ((dev_id == NULL) || (dev_id != xdat->dev_id) || (evt_type >= MSPI_BUS_EVENT_MAX)) {
		return -EINVAL;
	}

	/* TODO do we need locking? */
	xdat->cb[evt_type] = cb;
	xdat->cb_ctx[evt_type] = ctx;

	return 0;
}

/* Transceive API implementation
 * Information we need to program HW and start the transfer(s)
 * MSPI driver init at build time parses chip devices under MSPI controller node
 * and fills in struct mspi_cfg .ce_groups pointer, num_ce_gpios, and num_periph
 * This struct mspi_cfg is in driver config structure.
 * We must also put a struct mspi_cfg in driver data in case the application calls
 * MSPI controller config API with a new struct mspi_cfg at runtime!
 *
 * Caller passes pointer to struct mspi_dev_id which contains:
 *     struct gpio_dt_spec ce and uint16_t dev_idx
 *       ce = fills in struct gpio_dt_spec from parent MSPI controller node ce_gpios[dev_idx]
 *       dev_idx = reg property (unit address) from child spi device node
 *
 */
#if 0
static int xec_gpio_dt_spec_match(const struct gpio_dt_spec *g1, const struct gpio_dt_spec *g2)
{
	if ((g1->port != g2->port) || (g1->pin != g2->pin) || (g1->dt_flags != g2->dt_flags)) {
		return -ENOENT;
	}

	return 0;
}
#endif

#if 0
/* More complex...
 * If we allow run-time discovery of the peripheral's connected to the MPSI
 * due to board jumper changes, how to we handle this?
 * 1. struct mspi_cfg is usually treated as  constant and stored in the driver's code space
 *    config struture but it contains the array of GPIO pins and number of elements in the array.
 * 2. If we allow run time determination of flash devices then we handle many combinations:
 *    Only GPIO pins used at CE's. No HW CE's used.
 *    Only HW CE's used. No GPIO pins.
 *    Mix where the number of each may change.
 * 3. If we move struct mspi_cfg to driver data space we add more issues.
 *    Application may use one instance of the MSPI structures and modify then between API calls.
 *    If we do not use deep comparison then we will use old configuration data.
 * 4. struct mspi_cfg CE GPIOs is an array. A deep comparison must iterate the array checking each
 *    element!
 */
static int xec_validate_mspi_dev_id(const struct device *controller,
                                    const struct mspi_dev_id *dev_id)
{
	const struct mchp_xec_mspi_cfg *xcfg = controller->config;
	struct mchp_xec_mspi_data *const xdat = controller->data;

	if (dev_id == NULL) {
		return -EINVAL;
	}

	if (xdat->dev_id != NULL) {
		if (xdat->dev_id->dev_idx != dev_id->dev_idx) {
			return -ENOENT;
		}

		return xec_gpio_dt_spec_match(NULL, NULL);
	}

	return 0;
}
#endif

static int xec_mspi_validate_xfr_params(const struct device *controller,
                                        const struct mspi_dev_id *dev_id,
                                        const struct mspi_xfer *req)
{
	struct mchp_xec_mspi_data *const xdat = controller->data;

	if ((dev_id == NULL) || (req == NULL)) {
		return -EINVAL;
	}

#ifndef CONFIG_MSPI_DMA
	if (req->xfer_mode == MSPI_DMA) {
		LOG_ERR("XEC MPSI xfr: driver does not support DMA");
		return -ENOTSUP;
	}
#endif

#ifndef CONFIG_MSPI_ASYNC
	if (req->async == true) {
		LOG_ERR("XEC MSPI xfr: driver does not support Async");
		return -ENOTSUP;
	}
#endif

	if (dev_id != xdat->dev_id) {
		LOG_ERR("XEC MSPI xfr: Device ID mismatch");
		return -EINVAL;
	}

	/* TODO struct mspi_xfr contains:
	 * tx_dummy, rx_dummy, also set in device config
	 * cmd_length, also set in device config
	 * addr_length, also set in device config
	 * hold_ce, keep CE asserted after all packets in this transfer request.
	 * ce_sw_ctrl What is this for?
	 *   contains struct gpio_dt_spec gpio, and uint32_t delay
	 *   delay is microseconds before first clock after asserting CE and delay
	 *   after last clock before de-asserting CE. How do we know to use ce_sw_ctrl?
	 *   Gemini says check ce_sw_ctrl.gpio.port. If NULL use HW CE, if not NULL use
	 *   the GPIO specified and delay parameter. Also look at hold_ce.
	 * priority What is this for?
	 * *packets - need to check for null pointer with num_packets != 0
	 * num_packets
	 * timeout - Units of milliseconds
	 */

	return 0;
}

/* MSPI transfer
 * dev_id contains dev_idx a value that is a target index. The value should match
 *                 the reg property value for the child node.
 *                 ce the struct gpio_dt_spec only valid if this target does not have
 *                 mspi-hardware-cd-num property.
 * NOTE: mspi_dev_config API allows the caller to specify struct mspi_dev_id and
 *       struct mspi_dev_cfg. We copy those structures into our driver data.
 *
 * Check the passed dev_id matches our driver data dev_id.
 *   Easy check to to compare addresses.
 *   Robust check is a deep compare of contents.
 */
static int mchp_xec_mspi_transceive(const struct device *controller,
                                    const struct mspi_dev_id *dev_id,
                                    const struct mspi_xfer *req)
{
	const struct mchp_xec_mspi_cfg *xcfg = controller->config;
	struct mchp_xec_mspi_data *const xdat = controller->data;
	int rc = 0;

	LOG_INF("XEC MSPI transceive");

	debug_pr_mspi_dev_id(dev_id);
	debug_pr_mspi_xfr(req);

	rc = xec_mspi_validate_xfr_params(controller, dev_id, req);
	if (rc != 0) {
		return rc;
	}

#ifdef CONFIG_MSPI_DMA
	/* TODO */
#else
	if (req->xfer_mode == MSPI_DMA) {
		LOG_ERR("XEC MPSI xfr: driver does not support DMA");
		return -ENOTSUP;
	}
#endif

#ifdef CONFIG_MSPI_ASYNC
	/* TODO */
#else
	if (req->async == true) {
		LOG_ERR("XEC MSPI xfr: driver does not support Async");
		return -ENOTSUP;
	}
#endif

	k_mutex_lock(&xdat->lock, K_FOREVER);


	/* common exit path */
	k_mutex_unlock(&xdat->lock);

	return rc;
}

/* QSPI interrupt handler */
static void mchp_xec_mspi_irq_handler(const struct device *controller)
{
	const struct mchp_xec_mspi_cfg *xcfg = controller->config;
	struct mchp_xec_mspi_data *const xdat = controller->data;
	mm_reg_t qb = xcfg->regbase;
	uint32_t qsr = 0;

	qsr = sys_read32(qb + XEC_QSPI_SR_OFS);
	xdat->qstatus = qsr;
	sys_write32(qsr, qb + XEC_QSPI_SR_OFS);

	/* TODO */
}

#ifdef CONFIG_PM_DEVICE
static int mchp_xec_mspi_pm_action(const struct device *controller, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_TURN_ON:
		/* TODO same as resume? */
		break;
	case PM_DEVICE_ACTION_RESUME:
		/* TODO re-enable pins and device */
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif

#ifdef CONFIG_DEVICE_DEINIT_SUPPORT
static int mchp_xec_mspi_deinit(const struct device *controller)
{
	/* TODO */
#ifdef CONFIG_PM_DEVICE
	/* PM must have suspended the device before we can de-initialize the driver */
#else
	/* TODO Suspend device */
#endif

	/* TODO deinit the device */

	return 0;
}
#endif

static int mchp_xec_mspi_init(const struct device *controller)
{
	const struct mchp_xec_mspi_cfg *xcfg = controller->config;
	struct mchp_xec_mspi_data *xdat = controller->data;

	LOG_INF("XEC MSPI init");
	LOG_INF("xdat at 0x%08x", (uint32_t)xdat);
	LOG_INF("xcfg at 0x%08x", (uint32_t)xcfg);
	LOG_INF("Reg base = 0x%08x", (uint32_t)xcfg->regbase);
	LOG_INF("Num Targets = %u", xcfg->num_targets);
	LOG_INF("Num CE-GPIOS = %u", xcfg->num_ce_gpios);

	soc_xec_pcr_sleep_en_clear(xcfg->enc_pcr);

	if (xcfg->config_irq != NULL) {
		xcfg->config_irq();
		soc_ecia_girq_ctrl(xcfg->girq, xcfg->girq_pos, 1);
	}

	const struct mspi_dt_spec spec = {
		.bus = controller,
		.config = xcfg->mspicfg, /* Copy. Does compiler generate a deep copy? */
	};

	return mchp_xec_mspi_config(&spec);
}

static DEVICE_API(mspi, mchp_xec_mspi_api) = {
	.config             = mchp_xec_mspi_config,
	.dev_config         = mchp_xec_mspi_dev_config,
	.timing_config      = mchp_xec_mspi_timing_config,
	.get_channel_status = mchp_xec_mspi_get_channel_status,
	.register_callback  = mchp_xec_mspi_register_callback,
	.transceive         = mchp_xec_mspi_transceive,
};

#define MCHP_XEC_MSPI_EMPTY_GPIO_SPEC { .port = NULL, .pin = 0, .dt_flags = 0 }

#define MCHP_XEC_CHILD_TARGET_ID(nid) DT_REG_ADDR(nid)
#define MCHP_XEC_CHILD_TARGET_ID_INST(inst) MCHP_XEC_CHILD_TARGET_ID(DT_DRV_INST(inst))

#define MCHP_XEC_CHILD_HAS_HW_CE(nid) DT_NODE_HAS_PROP(nid, mspi_hardware_ce_num)
#define MCHP_XEC_CHILD_HAS_HW_CE_INST(inst) MCHP_XEC_CHILD_HAS_HW_CE(DT_DRV_INST(inst))

#define MCHP_XEC_CHILD_HW_CE_NUM_OR_ZERO(node_id)                             \
    COND_CODE_1(DT_NODE_HAS_PROP(node_id, mspi_hardware_ce_num),              \
        (DT_PROP(node_id, mspi_hardware_ce_num)),                             \
        (0))

#define MCHP_XEC_CHILD_HW_CE_NUM_OR_ZERO_INST(inst) \
	MCHP_XEC_CHILD_HW_CE_NUM_OR_ZERO(DT_DRV_INST(inst))

#define MCHP_XEC_CHILD_CE_TYPE(node_id)                                       \
    COND_CODE_1(DT_NODE_HAS_PROP(node_id, mspi_hardware_ce_num),              \
        (MCHP_XEC_CE_HW),                                                     \
        (MCHP_XEC_CE_GPIO))

#define MCHP_XEC_CHILD_CE_TYPE_INST(inst) MCHP_XEC_CHILD_CE_TYPE(DT_DRV_INST(inst))

#define MCHP_XEC_CHILD_GPIO_CE_OR_EMPTY(ctrl_node, child_node)                      \
    COND_CODE_1(DT_NODE_HAS_PROP(child_node, mspi_hardware_ce_num),                 \
        (MCHP_XEC_MSPI_EMPTY_GPIO_SPEC),                                            \
        (GPIO_DT_SPEC_GET_BY_IDX(ctrl_node, ce_gpios, DT_REG_ADDR_RAW(child_node))))

#define MCHP_XEC_MSPI_TARGET_CFG_INIT(child_node, ctrl_node)         \
	{                                                            \
		.target_id = DT_REG_ADDR(child_node),                \
		.ce_type = MCHP_XEC_CHILD_CE_TYPE(child_node),       \
		.hw_ce_num = MCHP_XEC_CHILD_HW_CE_NUM_OR_ZERO(child_node), \
		.gpio_ce = MCHP_XEC_CHILD_GPIO_CE_OR_EMPTY(ctrl_node, child_node), \
		.max_freq = DT_PROP(child_node, mspi_max_frequency), \
	},

#define MCHP_XEC_MSPI_DT_GIRQ(inst) MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, 0))
#define MCHP_XEC_MSPI_DT_GIRQ_POS(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, 0))

#define MCHP_XEC_MSPI_DT_TAPS(inst, prop, idx) \
	COND_CODE_1(DT_PROP_HAS_IDX(inst, prop, idx), (DT_INST_PROP_BY_IDX(inst, prop, idx)), (0))

#define MCHP_XEC_CHILD_HAS_HW_CE_NUM(node_id) \
	+ COND_CODE_1(DT_NODE_HAS_PROP(node_id, mspi_hardware_ce_num), (1), (0))

#define MCHP_XEC_NUM_CHILD_WITH_HW_CE_NUM(parent) \
	(0 DT_FOREACH_CHILD(parent, MCHP_XEC_CHILD_HAS_HW_CE_NUM))

/* MSPI control config
 * QSPI controller does not support DQS (dual-data-rate)
 * QSPI controller requires software_multiperipheral to be true. Driver must select
 * one of the chip enables QSPI controls directly or GPIO chip enable.
 * ce_group = pointer to array of struct gpio_dt_spec
 * num_ce_gpios = Number of elements in ce_group, must match number of child devices using
 *   gpio (no mspi-hardware-ce-num property in the child)
 * ce_group = array of struct gpio_dt_spec
 * num_periph = total number of child devices =
 *   num(nodes with mspi-hardware-ce-num) + num_ce_gpios
 * NOTE:
 * We performed and experiment renumbering the child nodes from 0 through 3 to 4 through 7.
 * Build failed. This implies child nodes reg(unit) address must start at 0.
 */
#define MCHP_XEC_MSPI_CONFIG(inst)                                                            \
	{                                                                                     \
		.channel_num = 0,                                                             \
		.op_mode = DT_INST_ENUM_IDX_OR(inst, op_mode, MSPI_OP_MODE_CONTROLLER),       \
		.duplex = DT_INST_ENUM_IDX_OR(inst, duplex, MSPI_FULL_DUPLEX),                \
		.max_freq = DT_INST_PROP_OR(inst, clock_frequency, MHZ(96)),                  \
		.dqs_support = DT_INST_PROP_OR(inst, dqs_support, 0),                         \
		.num_periph = DT_INST_CHILD_NUM(inst),                                        \
		.sw_multi_periph = DT_INST_PROP_OR(inst, software_multiperipheral, 1),        \
		.num_ce_gpios = ARRAY_SIZE(ce_gpios##inst),                                   \
		.ce_group = (struct gpio_dt_spec *)ce_gpios##inst,                            \
	}

#define MCHP_XEC_MSPI_DEFINE(inst) \
	PINCTRL_DT_INST_DEFINE(inst); \
	PM_DEVICE_DT_INST_DEFINE(inst, mchp_xec_mspi_pm_action); \
	static void mchp_xec_mspi_cfg_irq(void) { \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), \
			    mchp_xec_mspi_irq_handler, DEVICE_DT_INST_GET(inst), 0 ); \
		irq_enable(DT_INST_IRQN(inst)); \
	} \
	static const struct gpio_dt_spec ce_gpios##inst[] = MSPI_CE_GPIOS_DT_SPEC_INST_GET(inst); \
	static const struct mchp_xec_target_cfg mchp_xec_mspi_targ_cfg_##inst[] = { \
		DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(inst, MCHP_XEC_MSPI_TARGET_CFG_INIT, \
							DT_DRV_INST(inst)) \
	}; \
	static const struct mchp_xec_mspi_cfg mchp_xec_mspi_xcfg_##inst = { \
		.regbase = DT_INST_REG_ADDR(inst), \
		.config_irq = mchp_xec_mspi_cfg_irq, \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), \
		.targets = mchp_xec_mspi_targ_cfg_##inst, \
		.mspicfg = MCHP_XEC_MSPI_CONFIG(inst), \
		.num_targets = ARRAY_SIZE(mchp_xec_mspi_targ_cfg_##inst), \
		.num_ce_gpios = DT_INST_PROP_LEN(inst, ce_gpios), \
		.enc_pcr = DT_INST_PROP(inst, pcr_scr), \
		.girq = MCHP_XEC_MSPI_DT_GIRQ(inst), \
		.girq_pos = MCHP_XEC_MSPI_DT_GIRQ_POS(inst), \
		.num_hw_ce = DT_INST_PROP_OR(inst, num_hw_ce, XEC_QSPI_MAX_CS), \
	}; \
	static struct mchp_xec_mspi_data mchp_xec_mspi_xdat_##inst = { \
		.lock = Z_MUTEX_INITIALIZER(mchp_xec_mspi_xdat_##inst.lock), \
		.sync = Z_SEM_INITIALIZER(mchp_xec_mspi_xdat_##inst.sync, 0, 1), \
		.qstatus = 0, \
		.dev_id = NULL, \
		.drv_cfg = {0}, \
		.dev_cfg = {0}, \
		.cb = {NULL}, \
		.cb_ctx = {0}, \
	}; \
	DEVICE_DT_INST_DEINIT_DEFINE(inst, mchp_xec_mspi_init, mchp_xec_mspi_deinit, \
				     PM_DEVICE_DT_INST_GET(inst), \
				     &mchp_xec_mspi_xdat_##inst, \
				     &mchp_xec_mspi_xcfg_##inst, \
				     POST_KERNEL, CONFIG_MSPI_INIT_PRIORITY, \
				     &mchp_xec_mspi_api);

DT_INST_FOREACH_STATUS_OKAY(MCHP_XEC_MSPI_DEFINE)
