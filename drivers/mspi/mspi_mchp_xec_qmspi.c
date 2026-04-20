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

#define XEC_MSPI_QMSPI_DEBUG 1
#define XEC_MSPI_QMSPI_DEBUG_REG_CAP_BUF_MAX 8

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

#define QMSPI_IOM_CMD_IDX 0
#define QMSPI_IOM_ADR_IDX 1U
#define QMSPI_IOM_TSC_IDX 2U
#define QMSPI_IOM_DAT_IDX 3U

struct mspi_xec_dev_cfg {
	uint32_t cmd_rd; /* Use cmd_len to determine number of bytes (0-4) */
	uint32_t cmd_wr; /* Use cmd_len to determine number of bytes (0-4) */
	uint8_t iom[4]; /* [0]:cmd phase, [1]:adr phase, [2]:tsc phase, [3]:data phase */
	uint16_t fdiv; /* HW specific: 0=2^16 divider, 1 to 0xffffU actual divider */
	uint16_t rx_tsc;
	uint16_t tx_tsc;
	uint8_t ce_num;
	uint8_t cpp;
	uint8_t cmd_len; /* 0 - 4 */
	uint8_t addr_len; /* 0 - 4 */
};

struct xec_mspi_context {
	struct mspi_xfer xfer;
	uint32_t packet_idx_max;
	uint32_t packet_idx;
	uint8_t *data_ptr;
	uint32_t data_len;
	struct k_sem lock;
};

struct mspi_xec_qmspi_drv_dat {
#ifdef XEC_MSPI_QMSPI_DEBUG
	volatile uint32_t isr_count;
#endif
	volatile uint32_t qstatus;
	volatile uint32_t qien;
	struct k_mutex lock;
	struct k_sem sync;
	struct xec_mspi_cfg active_xmspicfg;
	const struct mspi_dev_id *dev_id;
	struct mspi_xec_dev_cfg xdev_cfg;
	mspi_callback_handler_t cbs[MSPI_BUS_EVENT_MAX];
	struct mspi_callback_context *cb_ctxs[MSPI_BUS_EVENT_MAX];
	struct xec_mspi_context ctx;
#ifdef XEC_MSPI_QMSPI_DEBUG
	volatile uint32_t qcap_idx;
	volatile uint32_t qsr[XEC_MSPI_QMSPI_DEBUG_REG_CAP_BUF_MAX];
	volatile uint32_t qier[XEC_MSPI_QMSPI_DEBUG_REG_CAP_BUF_MAX];
#endif
};

#ifdef XEC_MSPI_QMSPI_DEBUG
static void dbg_init_qmspi(const struct device *controller)
{
	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;

	xdat->isr_count = 0;
	xdat->qcap_idx = 0;

	for (uint32_t i = 0; i < XEC_MSPI_QMSPI_DEBUG_REG_CAP_BUF_MAX; i++) {
		xdat->qsr[i] = 0;
		xdat->qier[i] = 0;
	}
}

static void dbg_qmspi_update(const struct device *controller, uint32_t qsr, uint32_t qien)
{
	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;
	uint32_t idx = xdat->qcap_idx;

	if (idx < XEC_MSPI_QMSPI_DEBUG_REG_CAP_BUF_MAX) {
		xdat->qsr[idx] = qsr;
		xdat->qier[idx] = qien;
		xdat->qcap_idx++;
	}
}

static void dbg_pr_qmspi_regs(mm_reg_t qb)
{
	uint32_t n = 0;

	LOG_INF("QMSPI @ 0x%0x", (uint32_t)qb);
	LOG_INF("  Mode = 0x%0x", sys_read32(qb + XEC_QSPI_MODE_OFS));
	LOG_INF("  CR = 0x%0x", sys_read32(qb + XEC_QSPI_CR_OFS));
	LOG_INF("  IFCR = 0x%0x", sys_read32(qb + XEC_QSPI_IFCR_OFS));
	LOG_INF("  SR = 0x%0x", sys_read32(qb + XEC_QSPI_SR_OFS));
	LOG_INF("  BCNT_CR = 0x%0x", sys_read32(qb + XEC_QSPI_BCNT_SR_OFS));
	LOG_INF("  IER = 0x%0x", sys_read32(qb + XEC_QSPI_IER_OFS));
	LOG_INF("  BCNT_TR = 0x%0x", sys_read32(qb + XEC_QSPI_BCNT_TR_OFS));
	LOG_INF("  CSTM = 0x%0x", sys_read32(qb + XEC_QSPI_CSTM_OFS));
	for (n = 0; n < XEC_QSPI_MAX_DESCR_IDX; n++) {
		LOG_INF("  Descr[%u] = 0x%0x", n, sys_read32(qb + XEC_QSPI_DESCR_OFS(n)));
	}
	LOG_INF("  Descr LDMA RX BM = 0x%0x", sys_read32(qb + XEC_QSPI_LDMA_RX_EN_OFS));
	LOG_INF("  Descr LDMA TX BM = 0x%0x", sys_read32(qb + XEC_QSPI_LDMA_TX_EN_OFS));
	for (n = 0; n < XEC_QSPI_LDMA_CHX_MAX; n++) {
		uint32_t ofs = XEC_QSPI_LDMA_CHX_CR_OFS(n);

		if (n < XEC_QSPI_LDMA_TX_CH0) {
			LOG_INF("  RX LDMA Chan %u", n);
		} else {
			LOG_INF("  TX LDMA Chan %u", (n - XEC_QSPI_LDMA_TX_CH0));
		}
		LOG_INF("    CR = 0x%0x", sys_read32(qb + ofs));
		LOG_INF("    SA = 0x%0x", sys_read32(qb + ofs + XEC_QSPI_LDMA_CH_SA_OFS));
		LOG_INF("    LR = 0x%0x", sys_read32(qb + ofs + XEC_QSPI_LDMA_CH_LR_OFS));
	}
}
#endif

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

static void qmspi_prog_fdiv(mm_reg_t qb, uint32_t fdiv)
{
	uint32_t fd = XEC_QSPI_MODE_CK_DIV_SET(fdiv);

	soc_mmcr_mask_set(qb + XEC_QSPI_MODE_OFS, fd, XEC_QSPI_MODE_CK_DIV_MSK);
}

static void qmspi_prog_cpp(mm_reg_t qb, uint8_t cpp)
{
	uint32_t val = XEC_QSPI_MODE_CP_SET((uint32_t)cpp);

	soc_mmcr_mask_set(qb + XEC_QSPI_MODE_OFS, val, XEC_QSPI_MODE_CP_MSK);
}

static uint16_t qmspi_calc_freq_div(uint32_t freq_hz)
{
	if (freq_hz >= XEC_QSPI_MAX_FREQ) {
		return 1U; /* clamp to max (divide by 1) */
	}

	if (freq_hz <= XEC_QSPI_MIN_FREQ) {
		return 0U; /* special HW value (divide by 2^16) */
	}

	/* integer divide truncates */
	return (uint16_t)((XEC_QSPI_MAX_FREQ) / freq_hz);
}

static void qmspi_prog_freq(mm_reg_t qb, uint32_t freq_hz)
{
	uint32_t fdiv = qmspi_calc_freq_div(freq_hz);

	fdiv = XEC_QSPI_MODE_CK_DIV_SET(fdiv);
	soc_mmcr_mask_set(qb + XEC_QSPI_MODE_OFS, fdiv, XEC_QSPI_MODE_CK_DIV_MSK);
}

/* Return a byte containing bit 0 = QMSPI CPOL, bit 1 = QMSPI CPHA_SDI, bit 2 = QMSPI CPHA_SDO */
static uint8_t qmspi_cpp(enum mspi_cpp_mode cpp_mode, uint32_t freq_hz)
{
	uint8_t cpp_bm = 0; /* SPI Mode 0 */

	if (cpp_mode == MSPI_CPP_MODE_3) {
		cpp_bm = 0x7U; /* CPOL=1, CPHA_SDI=1, CPHA_SDO=1 */
	} else if (cpp_mode == MSPI_CPP_MODE_2) {
		cpp_bm = 0x1U; /* CPOL=1, CPHA_SDI=0, CPHA_SDO=0 */
	} else if (cpp_mode == MSPI_CPP_MODE_1) {
		cpp_bm = 0x3U; /* CPOL=1, CPHA_SDI=1, CPHA_SDO=0 */
	} else {
		cpp_bm = 0; /* SPI Mode 0: CPOL=0, CPHA_SDI=0, CPHA_SDO=0 */
	}

	if (freq_hz >= MHZ(48)) {
		cpp_bm ^= BIT(2); /* invert CPHA_SDO */
	}

	return cpp_bm;
}

#define QMSPI_IOM_CMD_GET(iom_enc) ((iom_enc) & 0x3U)
#define QMSPI_IOM_ADR_GET(iom_enc) (((iom_enc) & 0x30U) >> 4)
#define QMSPI_IOM_TSC_GET(iom_enc) (((iom_enc) & 0x300U) >> 8)
#define QMSPI_IOM_DAT_GET(iom_enc) (((iom_enc) & 0x3000U) >> 12)

/* IO mode encoded to QMSPI control/descriptor IOM field
 * 0 = single, 1=dual, 2=quad, 3=rsvd
 */
static const uint16_t qmspi_iom_bms[] = {
	0x0000U, /* MSPI_IO_MODE_SINGLE */
	0x1011U, /* MSPI_IO_MODE_DUAL */
	0x1000U, /* MSPI_IO_MODE_DUAL_1_1_2 */
	0x1010U, /* MSPI_IO_MODE_DUAL_1_2_2 */
	0x2022U, /* MSPI_IO_MODE_QUAD */
	0x2000U, /* MSPI_IO_MODE_QUAD_1_1_4 */
	0x2020U, /* MSPI_IO_MODE_QUAD_1_4_4 */
};

/* MEC172x requires cleanup sequence if controller is not reset */
static void qmspi_cleanup(mm_reg_t qb, uint32_t mode_val)
{
	sys_write32(0, qb + XEC_QSPI_MODE_OFS);
	sys_write32(0, qb + XEC_QSPI_LDMA_RX_EN_OFS);
	sys_write32(0, qb + XEC_QSPI_LDMA_RX_EN_OFS);
	for (uint32_t n = 0; n < XEC_QSPI_LDMA_CHX_MAX; n++) {
		uint32_t ofs = XEC_QSPI_LDMA_CHX_CR_OFS(n); /* LDMA channel control reg offset */

		sys_write32(0, ofs);
		sys_write32(0, ofs + XEC_QSPI_LDMA_CH_SA_OFS);
		sys_write32(0, ofs + XEC_QSPI_LDMA_CH_LR_OFS);
	}

	sys_write32(mode_val, qb + XEC_QSPI_MODE_OFS);
}

/* Transfer in progress? */
static bool mspi_xec_qmspi_inp(const struct device *controller)
{
	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;

	if (k_sem_count_get(&xdat->ctx.lock) == 0) {
		return true;
	}

	return false;
}

/* Appy settings translated from struct mspi_cfg
 * If re-init requested we soft-reset the controller forcing the frequency, CPP, and
 * HW chip enable fields to their default values:
 *  frequency divider = 0 (divide by 2^16)
 *  CPOL=CPHA_SDI=CPHA_SDO=0 -> SPI Mode 0
 *  HW chip enable = 00b -> chip enable 0
 *
 * MSPI struct mspi_cfg has max_freq parameter, no CPP and chip enable params
 * MSPI struct mspi_dev_cfg has ce_num, freq and cpp params
 * We can use current struct mspi_dev_cfg from driver data to configure freq, CPP, and CE.
 * During driver init we get struct mspi_dev_cfg values from DT or if not present use
 * defaults: CPP = SPI Mode 0, CE = HW CS0, and freq = max_freq from struct mspi_cfg or
 * should we use a "safe" default freq, 24MHz?
 *
 * mspi_stm32_qspi.c driver data struct mspi_dev_cfg dev_cfg, set to 0 in driver macros
 *
 */
static int mspi_xec_qmspi_cfg(const struct device *controller)
{
	const struct mspi_xec_qmspi_drv_cfg *xcfg = controller->config;
	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;
	struct xec_mspi_cfg *acfg = &xdat->active_xmspicfg;
	mm_reg_t qb = xcfg->regbase;

	if ((acfg->flags & BIT(XEC_MSPI_CFG_FLAG_REINIT_POS)) != 0) {
		/* reset results in CPOL=CPHA_SDI=CPHA_SDO=0 -> Mode 0
		 * MSPI CPP param for phase is in struct mspi_dev_cfg.
		 */
		qmspi_reset(qb);
	} else {
		/* Mode register default value: CPP=Mode 0, CE=0, fdiv=0
		 * RX/TX local-DMA disabled
		 */
		qmspi_cleanup(qb, 0);
	}

	qmspi_prog_freq(qb, acfg->max_freq);

	sys_set_bit(qb + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_ACTV_POS);

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

	if ((mcfg->max_freq > XEC_QSPI_MAX_FREQ) || (mcfg->max_freq <= XEC_QSPI_MIN_FREQ)) {
		LOG_ERR("MSPI CR cfg: max_freq %u out of range", mcfg->max_freq);
		return -ENOTSUP;
	}

	return 0;
}

/* API - Configure MSPI controller based on passed spec containing the controller device
 *       pointer and a struct mspi_cfg.
 * MSPI struct mspi_cfg contains:
 *  maximum frequency in Hz the controller config will support
 *  optional pointer to a array of struct gpio_dt_spec and num_ce_gpios for GPIOs for use
 *  as chip enables.
 *  Number of peripherals is the total number of child nodes under this controller.
 *  booleans for DQS, software-multiperipheral, and re-init.
 * We translate mspi_cfg into a smaller structure in the driver.
 */
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

	rc = mspi_xec_qmspi_cfg(controller);
	if (rc != 0) {
		return rc;
	}

	/* Why not use k_sem_reset? */
	if (k_sem_count_get(&xdat->ctx.lock) == 0) {
		k_sem_give(&xdat->ctx.lock);
	}

	return 0;
}

/* Configure controller and transfer based on param_mask and associated values in devcfg.
 * We support params:
 *   MSPI_DEVICE_CONFIG_CE_NUM
 *   MSPI_DEVICE_CONFIG_FREQUENCY
 *   MSPI_DEVICE_CONFIG_IO_MODE
 *   MSPI_DEVICE_CONFIG_CPP
 *   MSPI_DEVICE_CONFIG_RX_DUMMY
 *   MSPI_DEVICE_CONFIG_TX_DUMMY
 *   MSPI_DEVICE_CONFIG_READ_CMD
 *   MSPI_DEVICE_CONFIG_WRITE_CMD
 *   MSPI_DEVICE_CONFIG_CMD_LEN (0-4)
 *   MSPI_DEVICE_CONFIG_ADDR_LEN (0-4)
 *   MSPI_DEVICE_CONFIG_ALL has all bits set
 */
static int mspi_xec_qm_dev_cfg(const struct device *controller,
			       const enum mspi_dev_cfg_mask param_mask,
			       const struct mspi_dev_cfg *devcfg)
{
	const struct mspi_xec_qmspi_drv_cfg *xcfg = controller->config;
	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;
	struct mspi_xec_dev_cfg *xdcfg = &xdat->xdev_cfg;
	mm_reg_t qb = xcfg->regbase;
	uint16_t temp = 0;

	if (devcfg == NULL) {
		return -EINVAL;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_FREQUENCY) != 0) {
		xdcfg->fdiv = qmspi_calc_freq_div(devcfg->freq);
		qmspi_prog_fdiv(qb, (uint32_t)xdcfg->fdiv);
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_IO_MODE) != 0) {
		if (devcfg->io_mode >= MSPI_IO_MODE_OCTAL) {
			return -EINVAL;
		}

		temp = qmspi_iom_bms[devcfg->io_mode];
		xdcfg->iom[QMSPI_IOM_CMD_IDX] = QMSPI_IOM_CMD_GET(temp);
		xdcfg->iom[QMSPI_IOM_ADR_IDX] = QMSPI_IOM_ADR_GET(temp);
		xdcfg->iom[QMSPI_IOM_TSC_IDX] = QMSPI_IOM_TSC_GET(temp);
		xdcfg->iom[QMSPI_IOM_DAT_IDX] = QMSPI_IOM_DAT_GET(temp);
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_CPP) != 0) {
		xdcfg->cpp = qmspi_cpp(devcfg->cpp, devcfg->freq);
		qmspi_prog_cpp(qb, xdcfg->cpp);
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_RX_DUMMY) != 0) {
		if (devcfg->rx_dummy > 0x7fffU) {
			return -EINVAL;
		}
		xdcfg->rx_tsc = devcfg->rx_dummy;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_TX_DUMMY) != 0) {
		if (devcfg->tx_dummy > 0x7fffU) {
			return -EINVAL;
		}
		xdcfg->tx_tsc = devcfg->tx_dummy;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_READ_CMD) != 0) {
		xdcfg->cmd_rd = devcfg->read_cmd;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_WRITE_CMD) != 0) {
		xdcfg->cmd_wr = devcfg->write_cmd;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_CMD_LEN) != 0) {
		if (devcfg->cmd_length > 4U) {
			return -EINVAL;
		}
		xdcfg->cmd_len = devcfg->cmd_length;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_ADDR_LEN) != 0) {
		if (devcfg->addr_length > 4U) {
			return -EINVAL;
		}
		xdcfg->addr_len = devcfg->addr_length;
	}

	return 0;
}

/* API - Controller and transfer parameters specific to a SPI child device
 * NOTE: controller HW state is not changed. Parameters are translated to
 * HW specific values and stored in driver data.
 * TODO special handling for dev_id and boolean software_multiperipheral?
 */
static int api_mspi_xec_qmspi_dev_config(const struct device *controller,
					 const struct mspi_dev_id *dev_id,
					 const enum mspi_dev_cfg_mask param_mask,
					 const struct mspi_dev_cfg *cfg)
{
	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;
	int rc = 0;

	if (dev_id == NULL) {
		LOG_ERR("MSPI Dev Cfg: dev_id is NULL");
		return -EINVAL;
	}

	k_mutex_lock(&xdat->lock, K_FOREVER);

	if (param_mask == MSPI_DEVICE_CONFIG_NONE) {
		xdat->dev_id = dev_id;
	} else {
		rc = mspi_xec_qm_dev_cfg(controller, param_mask, cfg);
		if (rc == 0) {
			xdat->dev_id = dev_id;
		}
	}

	k_mutex_unlock(&xdat->lock);

	return rc;
}

static int api_mspi_xec_qmspi_get_chan_status(const struct device *controller, uint8_t ch)
{
	const struct mspi_xec_qmspi_drv_cfg *xcfg = controller->config;
	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;
	mm_reg_t qb = xcfg->regbase;

	if (ch != 0) {
		return -EINVAL;
	}

	if (mspi_xec_qmspi_inp(controller) ||
	    (sys_test_bit(qb + XEC_QSPI_SR_OFS, XEC_QSPI_SR_CS_ASSERTED_POS) != 0)) {
		return -EBUSY;
	}

	/* Not busy
	 * clear pointer to struct mspi_dev_id in driver data
	 * Inform PM subystem device is not busy
	 * Make sure mutex is unlocked
	 */
	xdat->dev_id = NULL;
	k_sem_give(&xdat->ctx.lock);
	k_mutex_unlock(&xdat->lock);

	return 0;
}

/* TODO - we are supporting MSPI_BUS_ERROR and MSPI_BUS_XFER_COMPLETE at this time.
 * We may need to support MSPI_BUS_TIMEOUT.
 * HW does not automatically do bus resets so we don't support MSPI_BUS_RESET.
 */
static int api_mspi_xec_qmspi_register_cb(const struct device *controller,
					  const struct mspi_dev_id *dev_id,
					  const enum mspi_bus_event evt_type,
					  mspi_callback_handler_t cb,
					  struct mspi_callback_context *ctx)
{
	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;
	int rc = 0;

	if ((evt_type != MSPI_BUS_ERROR) && (evt_type != MSPI_BUS_XFER_COMPLETE)) {
		return -EINVAL;
	}

	k_mutex_lock(&xdat->lock, K_FOREVER);

	if (mspi_xec_qmspi_inp(controller)) {
		rc = -EBUSY;
		goto reg_cb_exit;
	}

	if (dev_id != xdat->dev_id) {
		rc = -ESTALE;
		goto reg_cb_exit;
	}

	xdat->cbs[evt_type] = cb;
	xdat->cb_ctxs[evt_type] = ctx;

reg_cb_exit:
	k_mutex_unlock(&xdat->lock);

	return 0;
}

/* chip select timing
 * TAPS selection for control and clock signals
 * TODO need custom header file in zephyr/drivers/mspi/ folder
 * define param masks for chip select timing and TAPS parameters
 * define parameter structure for casting timing_cfg.
 */
static int api_mspi_xec_timing_config(const struct device *controller,
				      const struct mspi_dev_id *dev_id, const uint32_t param_mask,
				      void *timing_cfg)
{
	/* TODO implement */
	return 0;
}

/* QMSPI descriptor fields:
 * ifm - 2 bits
 * txm - 2 bits
 * txdma - 2 bits 0=dis, 1-3=TX LDMA chan 0-2
 * rxen - 1 bit
 * rxdma - 2 bits 0=dis, 1-3=RX LDMA chan 0-2
 * close - 1 bit
 * last - 1 bit
 * qunits - 2 bits 0=bits, 1=bytes, 2=4 bytes, 3=16 bytes
 * num_qunits - 14 bits
 */
#define QMSPI_DESCR_FLAG_TX_POS    0
#define QMSPI_DESCR_FLAG_TX_DATA   0x1U
#define QMSPI_DESCR_FLAG_TX_ZEROS  0x2U
#define QMSPI_DESCR_FLAG_TX_ONES   0x3U
#define QMSPI_DESCR_FLAG_TX_MSK    0x3U
#define QMSPI_DESCR_FLAG_RX_EN     0x8U
#define QMSPI_DESCR_FLAG_CLOSE     0x10U
#define QMSPI_DESCR_FLAG_LAST      0x20U

static inline uint32_t build_descr(uint8_t ifm, uint8_t ldma_chan, uint8_t qunits,
				   uint16_t nqunits, uint8_t next_descr, uint8_t flags)
{
	uint32_t descr = XEC_QSPI_CR_IFM_SET((uint32_t)ifm);
	uint32_t txm = 0;

	descr |= XEC_QSPI_CR_QUNIT_SET((uint32_t)qunits);
	descr |= XEC_QSPI_CR_NQUNITS_SET((uint32_t)nqunits);
	descr |= XEC_QSPI_DR_ND_SET((uint32_t)next_descr);

	if ((flags & QMSPI_DESCR_FLAG_TX_MSK) != 0) {
		txm = (uint32_t)((flags & QMSPI_DESCR_FLAG_TX_MSK) >> QMSPI_DESCR_FLAG_TX_POS);
		descr |= XEC_QSPI_CR_TXM_SET(txm);
		descr |= XEC_QSPI_CR_TXDMA_SET((uint32_t)ldma_chan);
	}

	if ((flags & QMSPI_DESCR_FLAG_RX_EN) != 0) {
		descr |= BIT(XEC_QSPI_CR_RX_EN_POS);
		descr |= XEC_QSPI_CR_RXDMA_SET((uint32_t)ldma_chan);
	}

	if ((flags & QMSPI_DESCR_FLAG_CLOSE) != 0) {
		descr |= BIT(XEC_QSPI_CR_CLOSE_EN_POS);
	}

	if ((flags & QMSPI_DESCR_FLAG_LAST) != 0) {
		descr |= BIT(XEC_QSPI_DR_LD_POS);
	}

	return descr;
}

#ifdef CONFIG_MSPI_MCHP_XEC_QMSPI_LDMA
/* if req->cmd_length != 0
 *    create descriptor for TX cmd byte(s)
 *      iom from struct xec_mspi_cfg iom member bits[1:0]
 *    config TX LDMA channel 0 for tranmsit of command
 *    set TX LDMA Descr enable bit for descriptor
 * endif
 * if req->addr_length != 0
 *    format address MSBF
 *    create descriptor for TX address bytes(s)
 *      iom from struct xec_mspi_cfg iom member bits[5:4]
 *    config TX LDMA channel 1 for transmit of address
 *    set TX LDMA enable descript bit map for bit[descr_id]
 * endif
 * ntsc = 0
 * if pkt->dir == MSPI_TX
 *   ntsc = xfr->tx_dummy
 * else
 *   ntsc = xfr->rx_dummy
 * endif
 * if ntsc != 0
 *   create descriptor for tri-state clock
 *     iom = 0 (single)
 *     TX & RX disabled
 *     qunits = bits
 *     num_qunits = ntsc
 *   No local-DMA required
 * endif
 * if pkt->num_bytes != 0
 *   create desciptor for TX or RX of data based on pkt->dir
 *     iom from struct xec_mspi_cfg iom member bits[13:12]
 *     qunits = bytes
 *     num_qunits = 0
 *   config TX or RX LDMA channel 2 for TX or RX of data
 *   set TX or RX LDMA desciptor index as bit position
 * endif
 */

#endif

/* Local-DMA clearing sequence required by MEC172x */
static void qmspi_prep(mm_reg_t qb)
{
	sys_clear_bits(qb + XEC_QSPI_MODE_OFS, (BIT(XEC_QSPI_MODE_LD_RX_EN_POS) |
	                                        BIT(XEC_QSPI_MODE_LD_TX_EN_POS)));
	sys_write32(0, qb + XEC_QSPI_LDMA_RX_EN_OFS);
	sys_write32(0, qb + XEC_QSPI_LDMA_TX_EN_OFS);
#if 0 /* TODO */
	sys_write32(0, qb + XEC_QSPI_LDMA_CHX_CR_OFS(0));
	sys_write32(0, qb + XEC_QSPI_LDMA_CHX_CR_OFS(1));
	sys_write32(0, qb + XEC_QSPI_LDMA_CHX_CR_OFS(2));
	sys_write32(0, qb + XEC_QSPI_LDMA_CHX_CR_OFS(3));
	sys_write32(0, qb + XEC_QSPI_LDMA_CHX_CR_OFS(4));
	sys_write32(0, qb + XEC_QSPI_LDMA_CHX_CR_OFS(5));
#endif
	sys_write32(BIT(XEC_QSPI_CR_DESCR_EN_POS), qb + XEC_QSPI_CR_OFS);
	sys_write32(XEC_QSPI_EXE_CLR_FIFOS_POS, qb + XEC_QSPI_EXE_OFS);
	sys_write32(UINT32_MAX, qb + XEC_QSPI_SR_OFS);
}

static void qmspi_ldma_cfg(mm_reg_t qb, uint8_t ldma_chan, void *mbuf, uint32_t mbuf_len)
{
	uint32_t ofs = XEC_QSPI_LDMA_CHX_CR_OFS((uint32_t)ldma_chan);
	uint32_t msa = (uint32_t)mbuf;
	uint32_t cr = 0;

	if (IS_ALIGNED((msa | mbuf_len), 4)) {
		cr |= XEC_QSPI_LDMA_CHX_CR_SZ_SET(XEC_QSPI_LDMA_CHX_CR_SZ_4B);
	} else if (IS_ALIGNED((msa | mbuf_len), 2)) {
		cr |= XEC_QSPI_LDMA_CHX_CR_SZ_SET(XEC_QSPI_LDMA_CHX_CR_SZ_2B);
	} else {
		cr |= XEC_QSPI_LDMA_CHX_CR_SZ_SET(XEC_QSPI_LDMA_CHX_CR_SZ_1B);
	}

	cr |= (BIT(XEC_QSPI_LDMA_CHX_CR_EN_POS) | BIT(XEC_QSPI_LDMA_CHX_CR_OVRL_POS) |
	       BIT(XEC_QSPI_LDMA_CHX_CR_INCRA_POS));

	ofs = XEC_QSPI_LDMA_CHX_CR_OFS((uint32_t)ldma_chan);

	sys_write32(msa, qb + ofs + XEC_QSPI_LDMA_CH_SA_OFS);
	sys_write32((uint32_t)mbuf_len, qb + ofs + XEC_QSPI_LDMA_CH_LR_OFS);
	sys_write32(cr, qb + ofs);
}

#if 0 /* unused */
static uint8_t qmspi_pio_unit_size(uint32_t num_bytes)
{
	if ((num_bytes & 0xFU) == 0) {
		return XEC_QSPI_CR_QUNIT_16B;
	} else if ((num_bytes & 0x3U) == 0) {
		return XEC_QSPI_CR_QUNIT_4B;
	} else {
		return XEC_QSPI_CR_QUNIT_1B;
	}
}
#endif

static uint32_t qmspi_calc_descr_nunits(uint32_t num_bytes, uint32_t *rem_bytes, uint8_t *qusz)
{
	uint32_t nqu = 0, rem = 0;
	uint8_t unit_size = XEC_QSPI_CR_QUNIT_1B;

	if (num_bytes <= 0x7FFFU) {
		nqu = num_bytes;
	} else {
		unit_size = XEC_QSPI_CR_QUNIT_16B;
		nqu = num_bytes / 16U;
		rem = (num_bytes - (nqu * 16U));
	}

	if (rem_bytes != NULL) {
		*rem_bytes = rem;
	}

	if (qusz != NULL) {
		*qusz = unit_size;
	}

	return nqu;
}

static bool qmspi_txb_is_empty(mm_reg_t qb)
{
	return (sys_test_bit(qb + XEC_QSPI_SR_OFS, XEC_QSPI_SR_TXB_EMPTY_POS) != 0);
}

#if 0 /* Unused */
static uint8_t qmspi_txb_wr(mm_reg_t qb, uint8_t *pd, uint8_t nbytes)
{
	uint16_t txb_len = 0;
	uint8_t nrem = 0, nwr = 0;

	txb_len = (uint16_t)XEC_QSPI_BCNT_SR_TXB_GET(sys_read32(qb + XEC_QSPI_BCNT_SR_OFS));

	nwr = (uint8_t)(8U - txb_len);
	nrem = nbytes - nwr;

	while (nwr--) {
		sys_write8(*pd, qb + XEC_QSPI_TXB_OFS);
		pd++;
	}

	return nrem;
}
#endif

static uint32_t qmspi_tx_fifo_fill(mm_reg_t qb, const uint8_t *src, uint32_t len)
{
	mm_reg_t qtxb = qb + XEC_QSPI_TXB_OFS;
	uintptr_t addr = (uintptr_t)src;
	uint32_t tx_count = 0, avail = 0, n = 0, total = 0;

	tx_count = XEC_QSPI_BCNT_SR_TXB_GET(sys_read32(qb + XEC_QSPI_BCNT_SR_OFS));

	if (tx_count >= XEC_QSPI_TX_FIFO_DEPTH) {
		return 0;
	}

	avail = XEC_QSPI_TX_FIFO_DEPTH - tx_count;
	n = (len < avail) ? len : avail;

	if (n == 0) {
		return 0;
	}

	total = n;

	if ((addr & 1U) && (n >= 1U)) {
		sys_write8(*(const uint8_t *)addr, qtxb);
		addr += 1U;
		n -= 1U;
	}

	if ((addr & 2U) && (n >= 2U)) {
		sys_write16(*(const uint16_t *)addr, qtxb);
		addr += 2U;
		n -= 2U;
	}

	while (n >= 4U) {
		sys_write32(*(const uint32_t *)addr, qtxb);
		addr += 4U;
		n -= 4U;
	}

	if (n >= 2U) {
		sys_write16(*(const uint16_t *)addr, qtxb);
		addr += 2U;
		n -= 2U;
	}

	if (n >= 1U) {
		sys_write8(*(const uint8_t *)addr, qtxb);
	}

	return total;
}

static uint32_t qmspi_rx_fifo_drain(mm_reg_t qb, uint8_t *dst, uint32_t len)
{
	mm_reg_t qrxb = qb + XEC_QSPI_RXB_OFS;
	uintptr_t addr = (uintptr_t)dst;
	uint32_t rx_count = 0, n = 0, total = 0;

	rx_count = XEC_QSPI_BCNT_SR_RXB_GET(sys_read32(qb + XEC_QSPI_BCNT_SR_OFS));
	n = (len < rx_count) ? len : rx_count;

	if (n == 0) {
		return 0;
	}

	total = n;

	if ((addr & 1U) && (n >= 1U)) {
		*(uint8_t *)addr = sys_read8(qrxb);
		addr += 1U;
		n -= 1U;
	}

	if ((addr & 2U) && (n >= 2U)) {
		*(uint16_t *)addr = sys_read16(qrxb);
		addr += 2U;
		n -= 2U;
	}

	while (n >= 4U) {
		*(uint32_t *)addr = sys_read32(qrxb);
		addr += 4U;
		n -= 4U;
	}

	if (n >= 2U) {
		*(uint16_t *)addr = sys_read16(qrxb);
		addr += 2U;
		n -= 2U;
	};

	if (n >= 1U) {
		*(uint8_t *)addr = sys_read8(qrxb);
	}

	return total;
}

static void qmspi_rx_fifo_discard(mm_reg_t qb)
{
	mm_reg_t rxb = qb + XEC_QSPI_RXB_OFS;
	uint32_t nb = XEC_QSPI_BCNT_SR_RXB_GET(sys_read32(qb + XEC_QSPI_BCNT_SR_OFS));

	for (uint32_t n = 0; n < nb; n++) {
		sys_read8(rxb);
	}
}

#define QMSPI_XFR_IER_MSK (BIT(XEC_QSPI_IER_XFR_DONE_POS) | BIT(XEC_QSPI_IER_TXB_ERR_POS) | \
		 	   BIT(XEC_QSPI_IER_RXB_ERR_POS) | BIT(XEC_QSPI_IER_PROG_ERR_POS) |\
		 	   BIT(XEC_QSPI_IER_LDMA_RX_ERR_POS) | BIT(XEC_QSPI_IER_LDMA_TX_ERR_POS))

#define QMSPI_XFR_ERR_MSK (BIT(XEC_QSPI_SR_TXB_ERR_POS) | BIT(XEC_QSPI_SR_RXB_ERR_POS) |\
			   BIT(XEC_QSPI_SR_PROG_ERR_POS) | BIT(XEC_QSPI_SR_LDMA_RX_ERR_POS) |\
			   BIT(XEC_QSPI_SR_LDMA_TX_ERR_POS))

static int mspi_xec_qmspi_xfr(const struct device *controller)
{
	const struct mspi_xec_qmspi_drv_cfg *xcfg = controller->config;
	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;
	struct mspi_xec_dev_cfg *xdcfg = &xdat->xdev_cfg;
	const struct mspi_xfer *req = &xdat->ctx.xfer;
	/* TODO handle num_packet > 1 */
	const struct mspi_xfer_packet *pkt = req->packets;
	mm_reg_t qb = xcfg->regbase;
	int rc = 0;
	uint32_t descr = 0, nb = 0, nu = 0, rem = 0;
	uint16_t ntsc = 0, ien = 0;
	uint8_t didx = 0, ifm = 0, ldma = 0;
	uint8_t qusz = 0;
	uint8_t flags = QMSPI_DESCR_FLAG_TX_DATA;
	bool is_dma = false;

	xdat->ctx.data_ptr = NULL;
	xdat->ctx.data_len = 0;

	qmspi_prep(qb);

	if (IS_ENABLED(CONFIG_MSPI_MCHP_XEC_QMSPI_LDMA) && (req->xfer_mode == MSPI_DMA)) {
		is_dma = true;
	}

	if (req->cmd_length != 0) {
		ifm = xdcfg->iom[QMSPI_IOM_CMD_IDX];
		if (is_dma) {
			ldma = XEC_QSPI_CR_TXDMA_TLDCH0;
		}

		descr = build_descr(ifm, ldma, XEC_QSPI_CR_QUNIT_1B, (uint16_t)req->cmd_length,
		                    (didx + 1U), flags);
		sys_write32(descr, qb + XEC_QSPI_DESCR_OFS(didx));

		if (is_dma) {
			qmspi_ldma_cfg(qb, ldma, (void *)pkt->cmd, req->cmd_length);
			sys_set_bit(qb + XEC_QSPI_LDMA_TX_EN_OFS, didx);
		} else {
			qmspi_tx_fifo_fill(qb, (const uint8_t *)&pkt->cmd, req->cmd_length);
		}

		didx++;
	}

	if (req->addr_length != 0) {
		ifm = xdcfg->iom[QMSPI_IOM_ADR_IDX];
		ldma = 0;
		if (is_dma) {
			ldma = XEC_QSPI_CR_TXDMA_TLDCH1;
		}

		descr = build_descr(ifm, ldma, XEC_QSPI_CR_QUNIT_1B, (uint16_t)req->addr_length,
		                    (didx + 1U), flags);
		sys_write32(descr, qb + XEC_QSPI_DESCR_OFS(didx));

		if (is_dma) {
			qmspi_ldma_cfg(qb, ldma, (void *)pkt->address, req->addr_length);
			sys_set_bit(qb + XEC_QSPI_LDMA_TX_EN_OFS, didx);
		} else {
			qmspi_tx_fifo_fill(qb, (const uint8_t *)&pkt->address, req->addr_length);
		}

		didx++;
	}

	if (pkt->dir == MSPI_TX) {
		ntsc = req->tx_dummy;
	} else {
		ntsc = req->rx_dummy;
	}

	if (ntsc != 0) {
		ifm = xdcfg->iom[QMSPI_IOM_TSC_IDX];
		ldma = 0;
		descr = build_descr(ifm, ldma, XEC_QSPI_CR_QUNIT_BITS, ntsc, (didx + 1U), 0);
		sys_write32(descr, qb + XEC_QSPI_DESCR_OFS(didx));
		didx++;
	}

	if (pkt->num_bytes != 0) {
		flags = (pkt->dir == MSPI_TX) ? QMSPI_DESCR_FLAG_TX_DATA : QMSPI_DESCR_FLAG_RX_EN;
		ifm = xdcfg->iom[QMSPI_IOM_DAT_IDX];
		ldma = 0;

		if (is_dma) {
			ldma = XEC_QSPI_CR_TXDMA_TLDCH2;
			descr = build_descr(ifm, ldma, XEC_QSPI_CR_QUNIT_1B, 0, (didx + 1U),
			                    flags);
			sys_write32(descr, qb + XEC_QSPI_DESCR_OFS(didx));

			qmspi_ldma_cfg(qb, ldma, (void *)pkt->address, req->addr_length);

			if (pkt->dir == MSPI_TX) {
				sys_set_bit(qb + XEC_QSPI_LDMA_TX_EN_OFS, didx);
			} else {
				sys_set_bit(qb + XEC_QSPI_LDMA_RX_EN_OFS, didx);
			}
			didx++;
		} else { /* PIO */
			xdat->ctx.data_ptr = pkt->data_buf;

			/* Allocate multiple descriptors due to 15-bit num qunits field limit
			 * QMSPI does not have enought descriptors to handle 2^32 bytes.
			 * Until our SoC's have multi-megabytes of memory will we need to
			 * handle exceeding the HW limits.
			 */
			nb = pkt->num_bytes;
			while (didx < XEC_QSPI_MAX_DESCR_IDX) {
				rem = 0;
				qusz = 0;
				nu = qmspi_calc_descr_nunits(nb, &rem, &qusz);

				descr = build_descr(ifm, ldma, qusz, nu, (didx + 1U), flags);
				sys_write32(descr, qb + XEC_QSPI_DESCR_OFS(didx));

				didx++;

				if (rem == 0) {
					break;
				}

				nb = rem;
			}

			if (rem == 0) {
				if (pkt->dir == MSPI_TX) {
					if (qmspi_txb_is_empty(qb)) {
						nb = qmspi_tx_fifo_fill(qb,
							(const uint8_t *)&pkt->data_buf,
							pkt->num_bytes);
						xdat->ctx.data_ptr += nb;
						xdat->ctx.data_len += nb;
					}
					ien |= BIT(XEC_QSPI_IER_TXB_EMPTY_POS);
				} else {
					ien |= BIT(XEC_QSPI_IER_RXB_FULL_POS);
				}
			}
		}
	}

	LOG_INF("MSPI XEC xfr: Used %u descriptors", didx);

	if (rem != 0) {
		LOG_ERR("MSPI XEC xfr: num_bytes = %u", pkt->num_bytes);
		return -E2BIG;
	}

	/* TODO Logic to set last and close flags! */
	if (didx == 0) {
		LOG_ERR("MSPI XEC xfr: Do nothing!");
		return -ENODATA;
	}

	descr = BIT(XEC_QSPI_DR_LD_POS);

	if (req->hold_ce == false) {
		descr |= BIT(XEC_QSPI_CR_CLOSE_EN_POS);
	}

	--didx; /* index of last descriptor */
	sys_set_bits(qb + XEC_QSPI_DESCR_OFS(didx), descr);

	/* interrupt enable and start */
	if (is_dma) {
		descr = BIT(XEC_QSPI_MODE_LD_RX_EN_POS) | BIT(XEC_QSPI_MODE_LD_TX_EN_POS);
	}

	ien |= (BIT(XEC_QSPI_IER_XFR_DONE_POS) | BIT(XEC_QSPI_IER_TXB_ERR_POS) |
		BIT(XEC_QSPI_IER_RXB_ERR_POS) | BIT(XEC_QSPI_IER_PROG_ERR_POS) |
		BIT(XEC_QSPI_IER_LDMA_RX_ERR_POS) | BIT(XEC_QSPI_IER_LDMA_TX_ERR_POS));

	sys_write32(ien, qb + XEC_QSPI_IER_OFS);

#ifdef XEC_MSPI_QMSPI_DEBUG
	LOG_INF("packet_idx = %u", xdat->ctx.packet_idx);
	LOG_INF("packet_max = %u", xdat->ctx.packet_idx_max);
	LOG_INF("data_ptr = %p", xdat->ctx.data_ptr);
	LOG_INF("data_len = %u", xdat->ctx.data_len);
	dbg_pr_qmspi_regs(qb);
#endif
	xdat->qstatus = BIT(31); /* DEBUG bit not implemented */
	sys_write32(BIT(XEC_QSPI_EXE_START_POS), qb + XEC_QSPI_EXE_OFS);

	k_sem_take(&xdat->sync, K_FOREVER);

	LOG_INF("MSPI XEC xfr: Interrupt gave semaphore!");
	LOG_INF("  qstatus = 0x%0x", xdat->qstatus);

	if ((xdat->qstatus & QMSPI_XFR_ERR_MSK) != 0) {
		rc = -EIO;
	}

	return rc;
}

/* TODO - alernate
 * don't check each packet, instead during transfer if a packet has
 * NULL pointer for data length of 0 skip it and move to next packet.
 */
#define MAX_PIO_SIZE (13U * 0x7FFFU)

static int check_mspi_xfer(const struct mspi_xfer *req)
{
	if ((req->cmd_length > 4U) || (req->addr_length > 4U)) {
		return -EINVAL;
	}

	if ((req->num_packet == 0) || (req->packets == NULL)) {
		return -EINVAL;
	}

	for (uint32_t n = 0; n < req->num_packet; n++) {
		const struct mspi_xfer_packet *pkt = &req->packets[n];

		if ((pkt->data_buf == NULL) || (pkt->num_bytes == 0)) {
			return -EINVAL;
		}
	}

	return 0;
}

/* The Monster
 * MSPI_PIO vs MSPI_DMA modes
 *   struct mspi_xfer has enum mspi_xfer_mode = {MSPI_PIO, MSPI_DMA}
 *   MSPI_DMA depends upon CONFIG_MSPI_MCHP_XEC_QMSPI_LDMA
 * Synchronous vs Asynchronous
 *   struct mspi_xfer has bool async
 * One or more packets
 *   struct mspi_xfer has pointer, packets to struct mspi_xfer_packet
 *                        num_packet (uint32_t)
 *   struct mspi_packet has
 *     dir = MSPI_RX or MSPI_TX
 *     cb_mask (enum_mspi_bus_event_cb_mask)
 *     cmd (uint32_t)
 *     address (uint32_t)
 *     num_bytes (uint32_t)
 *     data_buf (uint8_t *)
 */
static int api_mspi_xec_qmspi_transceive(const struct device *controller,
					 const struct mspi_dev_id *dev_id,
					 const struct mspi_xfer *req)
{
	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;
	int rc = 0;

	if ((dev_id == NULL) || (req == NULL)) {
		return -EINVAL;
	}

	rc = check_mspi_xfer(req);
	if (rc != 0) {
		return rc;
	}

	k_mutex_lock(&xdat->lock, K_FOREVER);

	if (dev_id != xdat->dev_id) {
		rc = -ESTALE;
		goto transcv_exit;
	}

	xdat->ctx.xfer = *req;
	xdat->ctx.packet_idx_max = req->num_packet;
	xdat->ctx.packet_idx = 0;

#ifdef XEC_MSPI_QMSPI_DEBUG
	dbg_init_qmspi(controller);
#endif
	if (req->async) {
		rc = mspi_xec_qmspi_xfr(controller);
		goto transcv_exit;
	}

	while (xdat->ctx.packet_idx < xdat->ctx.packet_idx_max) {
		rc = mspi_xec_qmspi_xfr(controller);
		if (rc != 0) {
			break;
		}
		xdat->ctx.packet_idx++;
	}

transcv_exit:
	if ((req->async == false) || (rc != 0)) {
		k_mutex_unlock(&xdat->lock);
	}

	return rc;
}

/* QMSPI interrupt handler - Events
 * Errors
 * XfrDone
 *   MSPI_DMA - if next pkt config & start else give sync sem (all done)
 *   MSPI_PIO -
 *     if MSPI_TX then TX FIFO should be empty then if next pkt config & start else give sync sem
 *     if MSPI_RX if RX FIFO not empty copy data to pkt buffer, update length, give sync sem
 * TX FIFO Empty - MSPI_PIO only
 *   if more data, fill TX FIFO and exit ISR
 * RX FIFO Full - MSPI_PIO only
 *   drain RX FIFO storing data in pkt buffer and exit ISR
 *
 * Read JEDEC-ID
 * One interrupt expected (XfrDone)
 * TX cmd = 1 byte fits in TX FIFO. Xfr should not enable TX FIFO empty
 * RX data = 3 bytes fits in RX FIFO. Xfr should not enable RX FIFO full.
 * Status order checks in ISR issue:
 *  XfrDone checked first and exist leaving data in RX FIFO!
 */
static void mspi_xec_qmspi_isr(const struct device *controller)
{
	const struct mspi_xec_qmspi_drv_cfg *xcfg = controller->config;
	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;
	/*	struct mspi_xec_dev_cfg *xdcfg = &xdat->xdev_cfg; */
	struct xec_mspi_context *ctx = &xdat->ctx;
	const struct mspi_xfer *req = &ctx->xfer;
	/* TODO handle num_packet > 1 */
	const struct mspi_xfer_packet *pkt = &req->packets[ctx->packet_idx];
	mm_reg_t qb = xcfg->regbase;
	uint32_t qstatus = qrd32(qb, XEC_QSPI_SR_OFS);
	uint32_t qien = qrd32(qb, XEC_QSPI_IER_OFS);
	uint32_t nb = 0;

#ifdef XEC_MSPI_QMSPI_DEBUG
	xdat->isr_count++;
	dbg_qmspi_update(controller, qstatus, qien);
#endif
	xdat->qstatus = qstatus;
	xdat->qien = qien;

	/* qwr32(qb, XEC_QSPI_IER_OFS, 0); */
	qwr32(qb, XEC_QSPI_SR_OFS, qstatus);
	soc_ecia_girq_status_clear(xcfg->girq, xcfg->girq_pos);

	if ((qstatus & QMSPI_XFR_ERR_MSK) != 0) {
		sys_write32(BIT(XEC_QSPI_EXE_STOP_POS), qb + XEC_QSPI_EXE_OFS);
		qwr32(qb, XEC_QSPI_IER_OFS, 0);
		k_sem_give(&xdat->sync);
		return;
	}

	if (req->xfer_mode == MSPI_DMA) {
		/* TODO check for another packet */
		ctx->packet_idx++;
		if (ctx->packet_idx < ctx->packet_idx_max) {
			/* TODO configure next packet and start */
			LOG_INF("MSPI XEC ISR: TODO LDMA next packet");
		}
	} else { /* MSPI_PIO */
		/* TX data phase */
		if (((qstatus & qien) & BIT(XEC_QSPI_SR_TXB_EMPTY_POS)) != 0) {
			/* TX FIFO is empty and interrupt enabled */
			if (ctx->data_len < pkt->num_bytes) {
				nb = qmspi_tx_fifo_fill(qb, (const uint8_t *)ctx->data_ptr,
							(pkt->num_bytes - ctx->data_len));
				ctx->data_ptr += nb;
				ctx->data_len += nb;
			}
		}

		/* RX FIFO not empty? */
		if ((qstatus & BIT(XEC_QSPI_SR_RXB_EMPTY_POS)) == 0) {
			if (pkt->dir == MSPI_RX) {
				if (ctx->data_len < pkt->num_bytes) {
					nb = qmspi_rx_fifo_drain(qb, ctx->data_ptr,
								 (pkt->num_bytes - ctx->data_len));
					ctx->data_ptr += nb;
					ctx->data_len += nb;
				}
			} else {
				qmspi_rx_fifo_discard(qb);
			}
		}
	} /* end MSPI_PIO */

	/* set on finish descriptor with Last = 1 or Error */
	if (qstatus & BIT(XEC_QSPI_SR_XFR_DONE_POS)) {
		/* TODO if next pkt then reconfig, start, and exit without giving sync */
		qwr32(qb, XEC_QSPI_IER_OFS, 0);
		k_sem_give(&xdat->sync);
	}
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
	const struct mspi_xec_qmspi_drv_cfg *xcfg = controller->config;
	mm_reg_t qb = xcfg->regbase;
	int rc = 0;

	qmspi_reset(qb);

	rc = pinctrl_apply_state(xcfg->pcfg, PINCTRL_STATE_SLEEP);
	if (rc == -ENOENT) {
		rc = 0;
	}

	return rc;
}
#endif

static int mspi_xec_qmspi_init(const struct device *controller)
{
	const struct mspi_xec_qmspi_drv_cfg *xcfg = controller->config;
	struct mspi_xec_qmspi_drv_dat *const xdat = controller->data;
	int rc = 0;

	k_mutex_init(&xdat->lock);
	k_sem_init(&xdat->sync, 0, 1);
	k_sem_init(&xdat->ctx.lock, 0, 1);

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
