/*
 * Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MSPI driver for the Microchip MEC175x QMSPI controller.
 *
 * Supports half-duplex, controller-mode SDR transfers in single, dual,
 * and quad I/O modes (including 1-1-N and 1-N-N variants). Data phase
 * runs through either PIO (descriptors + 8-byte FIFOs) or QMSPI local
 * DMA (three dedicated channels, length-override enabled). The local
 * DMA path does not depend on CONFIG_MSPI_DMA / the Zephyr DMA driver;
 * it is gated by CONFIG_MSPI_MCHP_XEC_QMSPI_LDMA and drives QMSPI
 * local-DMA registers directly.
 */

#define DT_DRV_COMPAT microchip_xec_qmspi_controller

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zephyr/dt-bindings/clock/mchp_xec_pcr.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>


LOG_MODULE_REGISTER(mspi_mchp_xec, CONFIG_MSPI_LOG_LEVEL);

#define MSPI_MCHP_XEC_DEBUG_ISR 1
#define MSPI_MCHP_XEC_DEBUG_NO_TIMEOUT 1

#define PINCTRL_STATE_HW_CE_EN PINCTRL_STATE_PRIV_START
#define PINCTRL_STATE_HW_CE_TS (PINCTRL_STATE_PRIV_START + 1)

/* Register helpers ---------------------------------------------------------- */

#define QR8(cfg, ofs)         sys_read8((cfg)->regs + (ofs))
#define QW8(cfg, ofs, v)      sys_write8((v), (cfg)->regs + (ofs))
#define QR16(cfg, ofs)        sys_read16((cfg)->regs + (ofs))
#define QW16(cfg, ofs, v)     sys_write16((v), (cfg)->regs + (ofs))
#define QR32(cfg, ofs)        sys_read32((cfg)->regs + (ofs))
#define QW32(cfg, ofs, v)     sys_write32((v), (cfg)->regs + (ofs))
#define QRMW32(cfg, ofs, msk, v)                                                                   \
	sys_write32((sys_read32((cfg)->regs + (ofs)) & ~(msk)) | ((v) & (msk)),                    \
		    (cfg)->regs + (ofs))

/* 15-bit NQUNITS limit per descriptor */
#define QMSPI_NQUNITS_MAX 0x7FFFu

#define QMSPI_SR_ERR_MSK (BIT(XEC_QSPI_SR_TXB_ERR_POS) | BIT(XEC_QSPI_SR_RXB_ERR_POS) |\
			  BIT(XEC_QSPI_SR_PROG_ERR_POS) | BIT(XEC_QSPI_SR_LDMA_RX_ERR_POS) |\
			  BIT(XEC_QSPI_SR_LDMA_TX_ERR_POS))

/* Driver state -------------------------------------------------------------- */

enum qmspi_data_phase_ifm {
	IFM_SINGLE = XEC_QSPI_CR_IFM_FD,
	IFM_DUAL   = XEC_QSPI_CR_IFM_DUAL,
	IFM_QUAD   = XEC_QSPI_CR_IFM_QUAD,
};

struct qmspi_io_decode {
	/* IFM value for the address phase */
	uint8_t addr_ifm;
	/* IFM value for the data phase */
	uint8_t data_ifm;
};

struct qmspi_xfer_state {
	const struct mspi_xfer *xfer;
	const struct mspi_dev_id *dev_id;
	uint32_t pkt_idx;
	int err;
	/* PIO FIFO access size for the current data phase (1, 2, or 4) */
	uint8_t pio_access;
	/* PIO byte cursor */
	uint32_t pio_pos;
	/* True while a xfer is in flight (guards ISR against spurious calls) */
	bool active;
	/* Async-mode flag snapshot for this request */
	bool async;
	/* current xfer uses GPIO CE */
	bool is_gpio_ce;
};

struct mspi_mchp_xec_config {
	uint32_t regs;
	uint32_t src_clk_hz;
	uint16_t girq_enc;
	uint16_t pcr_enc;
	uint8_t cs_count;
	uint8_t num_ce_gpios;
	const struct pinctrl_dev_config *pcfg;
	struct gpio_dt_spec *ce_gpios;
	bool sw_multi_periph;
	void (*irq_cfg_func)(void);
};

struct xec_ca_buf {
	union {
		uint32_t w;
		uint16_t hw[2];
		uint8_t b[4];
	};
};

struct mspi_mchp_xec_data {
#ifdef MSPI_MCHP_XEC_DEBUG_ISR
	volatile uint32_t isr_count;
	volatile uint32_t qstatus;
	volatile uint32_t qbc_status;
	volatile uint32_t qier;
	volatile uint32_t qbc_trig;
#endif
	const struct device *dev;
	struct qmspi_xfer_state st;
	struct xec_ca_buf cmd_buf;
	struct xec_ca_buf addr_buf;
	struct qmspi_io_decode iodec;
	struct mspi_dev_cfg dev_cfg;
	const struct mspi_dev_id *active_dev;
	bool dev_cfg_valid;

#ifdef CONFIG_MULTITHREADING
	struct k_mutex lock;
	struct k_sem xfer_done;
	struct k_work async_pkt_work;
	struct k_work async_tmout_work;
	mspi_callback_handler_t cbs[MSPI_BUS_EVENT_MAX];
	struct mspi_callback_context *cb_ctxs[MSPI_BUS_EVENT_MAX];
#else
	volatile bool xfer_done;
	bool lock;
#endif
};

/* Forward declarations ------------------------------------------------------ */

static void mspi_mchp_xec_isr(const struct device *dev);
static int submit_packet(const struct device *dev, uint32_t pkt_idx);
static int finalize_packet(const struct device *dev, int rc);
static int finalize_xfer(const struct device *dev, int rc);

/* CPP → MODE mapping. Caller applies the ≥ 48 MHz CPHA_SDO inversion. */
static uint32_t cpp_to_mode_bits(enum mspi_cpp_mode cpp, uint32_t freq_hz)
{
	uint32_t bits = 0;

	switch (cpp) {
	case MSPI_CPP_MODE_0:
		bits = 0;
		break;
	case MSPI_CPP_MODE_1:
		bits = BIT(XEC_QSPI_MODE_CPHA_SDI_FE_POS) | BIT(XEC_QSPI_MODE_CPHA_SDO_SE_POS);
		break;
	case MSPI_CPP_MODE_2:
		bits = BIT(XEC_QSPI_MODE_CPOL_HI_POS);
		break;
	case MSPI_CPP_MODE_3:
		bits = BIT(XEC_QSPI_MODE_CPOL_HI_POS) | BIT(XEC_QSPI_MODE_CPHA_SDI_FE_POS) |
		       BIT(XEC_QSPI_MODE_CPHA_SDO_SE_POS);
		break;
	}

	if (freq_hz >= 48000000u) {
		bits ^= BIT(XEC_QSPI_MODE_CPHA_SDO_SE_POS);
	}
	return bits;
}

static int io_mode_decode(enum mspi_io_mode mode, struct qmspi_io_decode *d)
{
	switch (mode) {
	case MSPI_IO_MODE_SINGLE:
		d->addr_ifm = IFM_SINGLE;
		d->data_ifm = IFM_SINGLE;
		return 0;
	case MSPI_IO_MODE_DUAL:
		d->addr_ifm = IFM_DUAL;
		d->data_ifm = IFM_DUAL;
		return 0;
	case MSPI_IO_MODE_DUAL_1_1_2:
		d->addr_ifm = IFM_SINGLE;
		d->data_ifm = IFM_DUAL;
		return 0;
	case MSPI_IO_MODE_DUAL_1_2_2:
		d->addr_ifm = IFM_DUAL;
		d->data_ifm = IFM_DUAL;
		return 0;
	case MSPI_IO_MODE_QUAD:
		d->addr_ifm = IFM_QUAD;
		d->data_ifm = IFM_QUAD;
		return 0;
	case MSPI_IO_MODE_QUAD_1_1_4:
		d->addr_ifm = IFM_SINGLE;
		d->data_ifm = IFM_QUAD;
		return 0;
	case MSPI_IO_MODE_QUAD_1_4_4:
		d->addr_ifm = IFM_QUAD;
		d->data_ifm = IFM_QUAD;
		return 0;
	default:
		return -ENOTSUP;
	}
}

/* Clock divider: 1..65535, with 0 meaning /65536. */
static uint16_t calc_ckdiv(uint32_t src_hz, uint32_t dest_hz)
{
	uint32_t div;

	if (dest_hz == 0 || dest_hz >= src_hz) {
		return 1u;
	}
	div = DIV_ROUND_UP(src_hz, dest_hz);
	if (div >= 0x10000u) {
		return 0u;
	}
	return (uint16_t)div;
}

/*
 * Alignment-aware pick of the QMSPI unit size and the PIO FIFO / LDMA
 * access width for a data-phase buffer.
 *
 *   unit  -> XEC_QSPI_CR_QUNIT_* enum (for PIO descriptor)
 *   acc   -> 1, 2, or 4 (bytes per FIFO or LDMA access)
 */
static void pick_unit_and_access(const uint8_t *buf, uint32_t len, uint8_t *unit, uint8_t *acc)
{
	uintptr_t a = (uintptr_t)buf;

	if (((a | len) & 0xFu) == 0 && len >= 16u) {
		*unit = XEC_QSPI_CR_QUNIT_16B;
		*acc = 4u;
	} else if (((a | len) & 0x3u) == 0 && len >= 4u) {
		*unit = XEC_QSPI_CR_QUNIT_4B;
		*acc = 4u;
	} else if (((a | len) & 0x1u) == 0 && len >= 2u) {
		*unit = XEC_QSPI_CR_QUNIT_1B;
		*acc = 2u;
	} else {
		*unit = XEC_QSPI_CR_QUNIT_1B;
		*acc = 1u;
	}
}

/* Bytes per descriptor NQUNIT for a given QUNIT value. */
static uint32_t qunit_bytes(uint8_t qunit)
{
	switch (qunit) {
	case XEC_QSPI_CR_QUNIT_1B:
		return 1u;
	case XEC_QSPI_CR_QUNIT_4B:
		return 4u;
	case XEC_QSPI_CR_QUNIT_16B:
		return 16u;
	default:
		return 1u;
	}
}

/* Controller-level init (called once from init_fn and again on re_init) ------ */

static int qmspi_hw_reset_and_init(const struct device *dev)
{
	const struct mspi_mchp_xec_config *cfg = dev->config;
	uint32_t ckdiv = 0, mode_fdiv = 0;

	/* Ungate clocks via PCR (clock required register). Drop SLP bit so it
	 * runs while active.
	 */
	soc_xec_pcr_sleep_en_clear((uint8_t)cfg->pcr_enc);

	/* Soft reset — self-clearing. Note: clk divider set to 0 (max) */
	QW32(cfg, XEC_QSPI_MODE_OFS, BIT(XEC_QSPI_MODE_SRST_POS));
	while (QR32(cfg, XEC_QSPI_MODE_OFS) & BIT(XEC_QSPI_MODE_SRST_POS)) {
		/* spin; reset is fast */
	}

	/* 12MHz is a safe default */
	ckdiv = (uint32_t)calc_ckdiv(MHZ(96), MHZ(12));
	mode_fdiv = XEC_QSPI_MODE_CK_DIV_SET(ckdiv);
	QRMW32(cfg, XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_CK_DIV_MSK, mode_fdiv);

	/* CS timing defaults */
	QW32(cfg, XEC_QSPI_CSTM_OFS,
	     XEC_QSPI_CSTM_CSA_FCK_SET(XEC_QSPI_CSTM_CSA_FCK_DFLT) |
		     XEC_QSPI_CSTM_LCK_CSD_SET(XEC_QSPI_CSTM_LCK_CSD_DFLT) |
		     XEC_QSPI_CSTM_LDHD_SET(XEC_QSPI_CSTM_LDHD_DFLT) |
		     XEC_QSPI_CSTM_CSD_CSA_SET(XEC_QSPI_CSTM_CSD_CSA_DFLT));

	/* Disable all interrupts, clear all status */
	QW32(cfg, XEC_QSPI_IER_OFS, 0);
	QW32(cfg, XEC_QSPI_SR_OFS, 0xFFFFFFFFu);

	/* Clear LDMA enable bitmaps */
	QW32(cfg, XEC_QSPI_LDMA_RX_EN_OFS, 0);
	QW32(cfg, XEC_QSPI_LDMA_TX_EN_OFS, 0);

	/* GIRQ routing for this source */
	(void)soc_ecia_girq_status_clear(MCHP_XEC_ECIA_GIRQ(cfg->girq_enc),
					 MCHP_XEC_ECIA_GIRQ_POS(cfg->girq_enc));
	(void)soc_ecia_girq_ctrl(MCHP_XEC_ECIA_GIRQ(cfg->girq_enc),
				 MCHP_XEC_ECIA_GIRQ_POS(cfg->girq_enc), 1u);

	/* Activate the controller. Leave all other MODE bits clear until
	 * dev_config() programs CPOL/CPHA/CS-select/clock-divider.
	 */
	QW32(cfg, XEC_QSPI_MODE_OFS, BIT(XEC_QSPI_MODE_ACTV_POS));

	return 0;
}

/* API: config --------------------------------------------------------------- */

static int mspi_mchp_xec_qm_config_api(const struct mspi_dt_spec *spec)
{
	const struct device *dev = spec->bus;
	const struct mspi_mchp_xec_config *cfg = dev->config;
	struct mspi_mchp_xec_data *data = dev->data;
	int rc = 0;

	if (spec->config.op_mode != MSPI_OP_MODE_CONTROLLER) {
		return -ENOTSUP;
	}
	if (spec->config.duplex != MSPI_HALF_DUPLEX) {
		return -ENOTSUP;
	}
	if (spec->config.dqs_support) {
		return -ENOTSUP;
	}

	if (cfg->pcfg != NULL) {
		/* Configures clock, data, and HW CE pins */
		rc = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (rc < 0) {
			LOG_ERR("pinctrl_apply_state: %d", rc);
			return rc;
		}
	}

	rc = qmspi_hw_reset_and_init(dev);
	data->dev_cfg_valid = false;
	data->active_dev = NULL;
	/* make sure lock is released */
	k_mutex_unlock(&data->lock);

	return rc;
}

/* API: dev_config ----------------------------------------------------------- */

/* Callers may pass any mix of configuration.
 * We must the ones which our hardware and driver do not support specific configuration values.
 * we support all configuration values for:
 *   MSPI_DEVICE_CONFIG_CPP, MSPI_DEVICE_CONFIG_READ_CMD, MSPI_DEVICE_CONFIG_WRITE_CMD
 * Hardware and driver do not support MSPI_DEVICE_CONFIG_MEM_BOUND since the hardware
 * can DMA into any memory address. We silently ignore it.
 * Driver does not support MSPI_DEVICE_CONFIG_BREAK_TIME so we silently ignore it.
 * CE number is from mspi-hardware-ce-num DT property. GPIO CE's should not have this
 * property. The CE number for GPIO CE's is 0.
 */
static int validate_dev_cfg(enum mspi_dev_cfg_mask mask, const struct mspi_dev_cfg *cfg)
{
	if ((mask & MSPI_DEVICE_CONFIG_CE_NUM) != 0) {
		if (cfg->ce_num >= XEC_QSPI_MAX_CS) {
			return -ENOTSUP;
		}
	}

	if ((mask & MSPI_DEVICE_CONFIG_FREQUENCY) != 0) {
		if ((cfg->freq < XEC_QSPI_FREQ_MIN) || (cfg->freq > XEC_QSPI_FREQ_MAX)) {
			return -ENOTSUP;
		}
	}

	if ((mask & MSPI_DEVICE_CONFIG_IO_MODE) != 0) {
		if (cfg->io_mode >= MSPI_IO_MODE_OCTAL) {
			return -ENOTSUP;
		}
	}

	if ((mask & MSPI_DEVICE_CONFIG_DATA_RATE) != 0) {
		if (cfg->data_rate != MSPI_DATA_RATE_SINGLE) {
			return -ENOTSUP;
		}
	}

	if ((mask & MSPI_DEVICE_CONFIG_ENDIAN) != 0) {
		if (cfg->endian != MSPI_XFER_BIG_ENDIAN) {
			return -ENOTSUP;
		}
	}

	/* applies to HW CE's only. GPIO CE's use GPIO_ACTIVE_HIGH/LOW in ce-gpios */
	if ((mask & MSPI_DEVICE_CONFIG_CE_POL) != 0) {
		/* TODO: We could support active high two ways:
		 * HW CE: we need to modify CE pin GPIO Control reg and turn on function invert
		 * GPIO CE: use driver logic to invert
		 * Is is worth the effort for this feature?
		 */
		if (cfg->ce_polarity != MSPI_CE_ACTIVE_LOW) {
			return -ENOTSUP;
		}
	}

	if ((mask & MSPI_DEVICE_CONFIG_DQS) != 0) {
		if (cfg->dqs_enable == true) {
			return -ENOTSUP;
		}
	}

	if ((mask & MSPI_DEVICE_CONFIG_RX_DUMMY) != 0) {
		if (cfg->rx_dummy > 0x7fffU) {
			return -ENOTSUP;
		}
	}

	if ((mask & MSPI_DEVICE_CONFIG_TX_DUMMY) != 0) {
		if (cfg->tx_dummy > 0x7fffU) {
			return -ENOTSUP;
		}
	}

	if ((mask & MSPI_DEVICE_CONFIG_CMD_LEN) != 0) {
		if (cfg->cmd_length > 4U) {
			return -ENOTSUP;
		}
	}

	if ((mask & MSPI_DEVICE_CONFIG_ADDR_LEN) != 0) {
		if (cfg->addr_length > 4U) {
			return -ENOTSUP;
		}
	}

	return 0;
}

static int apply_dev_cfg(const struct device *dev, const struct mspi_dev_cfg *cfg)
{
	const struct mspi_mchp_xec_config *hw = dev->config;
	struct mspi_mchp_xec_data *data = dev->data;
	uint32_t mode = QR32(hw, XEC_QSPI_MODE_OFS);
	uint16_t ckdiv;
	int rc;

	rc = io_mode_decode(cfg->io_mode, &data->iodec);
	if (rc) {
		return rc;
	}

	/* Clock divider from frequency */
	ckdiv = calc_ckdiv(hw->src_clk_hz, cfg->freq);
	mode &= ~XEC_QSPI_MODE_CK_DIV_MSK;
	mode |= XEC_QSPI_MODE_CK_DIV_SET(ckdiv);

	/* CPOL / CPHA including ≥ 48 MHz CPHA_SDO inversion */
	mode &= ~XEC_QSPI_MODE_CP_MSK;
	mode |= cpp_to_mode_bits(cfg->cpp, cfg->freq);

	/* Hardware CS select. For GPIO CE, ce_num is still safe to write -
	 * the MODE CS_SEL picks among HW CS pads but driving via GPIO avoids
	 * the internal pad. When both HW and GPIO CE are configured, only
	 * one is wired at the board level.
	 */
	if (cfg->ce_num < XEC_QSPI_MAX_CS) {
		mode &= ~XEC_QSPI_MODE_CS_SEL_MSK;
		mode |= XEC_QSPI_MODE_CS_SEL_SET(cfg->ce_num);
	}

	QW32(hw, XEC_QSPI_MODE_OFS, mode);

	return 0;
}

/* Select DEFAULT or ALTERNATE pinctrl state for the active device.
 *
 * The QMSPI controller cannot disable its HW chip-enable outputs once they are
 * muxed in. If the target device uses a GPIO chip-enable (dev_id->ce.port is
 * non-NULL), both the HW CE pin and the GPIO CE would assert on the same
 * transaction. Park the HW CE pin via PINCTRL_STATE_ALTERNATE in that case so
 * only the GPIO CE drives the bus. Devices that use a HW CE run on
 * PINCTRL_STATE_DEFAULT, which keeps the HW CE pin muxed as the CE output.
 */
static int qmspi_apply_ce_pinctrl(const struct device *dev,
				  const struct mspi_dev_id *dev_id)
{
	const struct mspi_mchp_xec_config *hw = dev->config;
	gpio_flags_t gp_flags = GPIO_OUTPUT_HIGH | GPIO_INPUT;
	int rc = 0;
	uint32_t qmode = 0;
	uint8_t state_id = PINCTRL_STATE_HW_CE_EN;

	if ((dev_id != NULL) && (dev_id->ce.port != NULL)) {
		/* configure GPIO CE for use */
		rc = gpio_pin_configure_dt(&dev_id->ce, gp_flags);
		if (rc != 0) {
			LOG_ERR("CE GPIO config error (%d)", rc);
			return rc;
		}

		/* This functions sets HW CE0 to alternate pinctrl state.
		 * We need to configure QMSPI.MODE HW CS_SEL field to CE0.
		 */
		qmode = QR32(hw, XEC_QSPI_MODE_OFS);

		qmode &= (uint32_t)~(XEC_QSPI_MODE_CS_SEL_MSK);
		qmode |= XEC_QSPI_MODE_CS_SEL_SET(XEC_QSPI_MODE_CS_SEL_0);

		QW32(hw, XEC_QSPI_MODE_OFS, qmode);

		state_id = PINCTRL_STATE_HW_CE_TS;
	}

	return pinctrl_apply_state(hw->pcfg, state_id);
}

/* Apply new device config or change the current config.
 * If a new device config the routine tries to acquire the driver lock.
 *
 * return 0 success, configuration applied, and driver lock is held
 *                   driver lock will be released in transceive API or
 *                   controller configuration API.
 * return < 0 error and driver lock is released
 */
static int mspi_mchp_xec_qm_dev_config_api(const struct device *dev,
					   const struct mspi_dev_id *dev_id,
					   const enum mspi_dev_cfg_mask mask,
					   const struct mspi_dev_cfg *cfg)
{
	const struct mspi_mchp_xec_config *hw = dev->config;
	struct mspi_mchp_xec_data *data = dev->data;
	int rc = 0;

	if (data->active_dev != dev_id) { /* requesting config for new device ID? */
		/* Yes, new dev_id. Acquire the lock */
#ifdef CONFIG_MULTITHREADING
		rc = k_mutex_lock(&data->lock, K_MSEC(CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE));
#else
		if (data->lock) {
			rc = -1;
		} else {
			data->lock = true;
			rc = 0;
		}
#endif
		if (rc != 0) {
			LOG_ERR("Controller is locked");
			return -EBUSY;
		}

#ifdef CONFIG_MULTITHREADING
		memset(data->cbs, 0, sizeof(data->cbs));
#endif
	}

	if ((!hw->sw_multi_periph) && (mask == MSPI_DEVICE_CONFIG_NONE)) {
		/* Binding requires software-multiperipheral; without it the
		 * "select existing device" path is unsupported.
		 */
		rc = -ENOTSUP;
		goto dev_cfg_exit;
	}

	rc = validate_dev_cfg(mask, cfg);
	if (rc != 0) {
		goto dev_cfg_exit;
	}

	if (mask == MSPI_DEVICE_CONFIG_NONE) {
		/* Only switching the active device. Previous config stays. */
		if (!data->dev_cfg_valid) {
			rc = -EINVAL;
			goto dev_cfg_exit;
		}
	} else {
		/* Build an effective config: start from cached (if any), merge
		 * fields indicated by the mask.
		 */
		struct mspi_dev_cfg eff = data->dev_cfg_valid ? data->dev_cfg : *cfg;

		if (mask & MSPI_DEVICE_CONFIG_CE_NUM) {
			eff.ce_num = cfg->ce_num;
		}
		if (mask & MSPI_DEVICE_CONFIG_FREQUENCY) {
			eff.freq = cfg->freq;
		}
		if (mask & MSPI_DEVICE_CONFIG_IO_MODE) {
			eff.io_mode = cfg->io_mode;
		}
		if (mask & MSPI_DEVICE_CONFIG_DATA_RATE) { /* validate checked the value */
			eff.data_rate = cfg->data_rate;
		}
		if (mask & MSPI_DEVICE_CONFIG_CPP) {
			eff.cpp = cfg->cpp;
		}
		if (mask & MSPI_DEVICE_CONFIG_ENDIAN) { /* validate checked the value */
			eff.endian = cfg->endian;
		}
		if (mask & MSPI_DEVICE_CONFIG_CE_POL) { /* validate checked the value */
			eff.ce_polarity = cfg->ce_polarity;
		}
		if (mask & MSPI_DEVICE_CONFIG_RX_DUMMY) {
			eff.rx_dummy = cfg->rx_dummy;
		}
		if (mask & MSPI_DEVICE_CONFIG_TX_DUMMY) {
			eff.tx_dummy = cfg->tx_dummy;
		}
		if (mask & MSPI_DEVICE_CONFIG_READ_CMD) {
			eff.read_cmd = cfg->read_cmd;
		}
		if (mask & MSPI_DEVICE_CONFIG_WRITE_CMD) {
			eff.write_cmd = cfg->write_cmd;
		}
		if (mask & MSPI_DEVICE_CONFIG_CMD_LEN) {
			eff.cmd_length = cfg->cmd_length;
		}
		if (mask & MSPI_DEVICE_CONFIG_ADDR_LEN) {
			eff.addr_length = cfg->addr_length;
		}

		rc = apply_dev_cfg(dev, &eff);
		if (rc != 0) {
			goto dev_cfg_exit;
		}

		rc = qmspi_apply_ce_pinctrl(dev, dev_id);
		if (rc != 0) {
			goto dev_cfg_exit;
		}

		data->dev_cfg = *cfg;
		data->dev_cfg_valid = true;
	}

dev_cfg_exit:
	/* If an error occurred clear the active dev_id and release the lock */
	if (rc != 0) {
		data->active_dev = NULL;
		data->dev_cfg_valid = false;
#ifdef CONFIG_MULTITHREADING
		k_mutex_unlock(&data->lock);
#else
		data->lock = false;
#endif
	}

	return rc;
}

/* API: get_channel_status --------------------------------------------------- */

static int mspi_mchp_xec_qm_get_chan_sts_api(const struct device *dev, uint8_t ch)
{
	const struct mspi_mchp_xec_config *hw = dev->config;
	struct mspi_mchp_xec_data *data = dev->data;
	uint32_t sr;

	if (ch != 0) {
		LOG_ERR("XEC QMSPI supports one channel(0) only");
		return -EINVAL;
	}

	sr = QR32(hw, XEC_QSPI_SR_OFS);
	if ((data->st.active) || (sr & BIT(XEC_QSPI_SR_CS_ASSERTED_POS))) {
		return -EBUSY;
	}

	return 0;
}

/* API: register_callback ---------------------------------------------------- */
#ifdef CONFIG_MULTITHREADING
/* support MSPI_BUS_XFER_COMPLETE, MSPI_BUS_ERROR, MSPI_BUS_TIMEOUT */
static int mspi_mchp_xec_qm_reg_cb_api(const struct device *dev, const struct mspi_dev_id *dev_id,
				       const enum mspi_bus_event evt, mspi_callback_handler_t cb,
				       struct mspi_callback_context *ctx)
{
	struct mspi_mchp_xec_data *data = dev->data;

	if (dev_id != data->active_dev) {
		LOG_ERR("Controller not configured for this dev_id");
		return -EINVAL;
	}

	if (evt == MSPI_BUS_RESET) {
		return -ENOTSUP;
	}

	data->cbs[evt] = cb;
	data->cb_ctxs[evt] = ctx;

	return 0;
}
#endif

/* Transfer engine ---------------------------------------------------------- */

static void prep_qspi(const struct mspi_mchp_xec_config *hw)
{
	uint32_t msk = BIT(XEC_QSPI_MODE_LD_RX_EN_POS) | BIT(XEC_QSPI_MODE_LD_TX_EN_POS);

	QRMW32(hw, XEC_QSPI_MODE_OFS, msk, 0);
	QW32(hw, XEC_QSPI_CR_OFS, BIT(XEC_QSPI_CR_DESCR_EN_POS));
	QW32(hw, XEC_QSPI_LDMA_RX_EN_OFS, 0);
	QW32(hw, XEC_QSPI_LDMA_TX_EN_OFS, 0);
	QW32(hw, XEC_QSPI_DESCR_OFS(0), 0);
	QW32(hw, XEC_QSPI_DESCR_OFS(1), 0);
	QW32(hw, XEC_QSPI_DESCR_OFS(2), 0);
	QW32(hw, XEC_QSPI_DESCR_OFS(3), 0);
#if 0
	for (uint32_t i = 0; i < XEC_QSPI_LDMA_CHX_MAX; i++) {
		QW32(hw, XEC_QSPI_LDMA_CHX_CR_OFS(i), 0);
	}
#endif
}

/* Write a 32-bit descriptor at slot idx. */
static inline void desc_write(const struct mspi_mchp_xec_config *hw, uint8_t idx, uint32_t d)
{
	QW32(hw, XEC_QSPI_DESCR_OFS(idx), d);
}

/* Build a descriptor word. */
static uint32_t desc_build(uint8_t ifm, uint8_t txm, bool rx_en, uint8_t tx_dma, uint8_t rx_dma,
			   uint8_t qunit, uint16_t nqunits, uint8_t next_idx, bool close_en,
			   bool last)
{
	uint32_t d = XEC_QSPI_CR_IFM_SET(ifm) | XEC_QSPI_CR_TXM_SET(txm) |
		     XEC_QSPI_CR_TXDMA_SET(tx_dma) | XEC_QSPI_CR_RXDMA_SET(rx_dma) |
		     XEC_QSPI_CR_QUNIT_SET(qunit) | XEC_QSPI_CR_NQUNITS_SET(nqunits) |
		     XEC_QSPI_DR_ND_SET(next_idx);

	if (rx_en) {
		d |= BIT(XEC_QSPI_CR_RX_EN_POS);
	}
	if (close_en) {
		d |= BIT(XEC_QSPI_CR_CLOSE_EN_POS);
	}
	if (last) {
		d |= BIT(XEC_QSPI_DR_LD_POS);
	}
	return d;
}

/* Writes the data phase across one or more descriptors respecting NQUNITS limits.
 * Returns the index of the last descriptor written. When CONFIG_MSPI_MCHP_XEC_QMSPI_LDMA
 * is enabled and ldma is true, a single descriptor is written with the LDMA TXDMA/RXDMA
 * encoding since length-override bypasses the 15-bit limit.
 */
static uint8_t write_data_descriptors(const struct mspi_mchp_xec_config *hw, uint8_t start_idx,
				      uint8_t data_ifm, bool dir_tx, uint8_t qunit, uint32_t len,
				      bool ldma)
{
	uint8_t idx = start_idx;
	uint32_t unit_bytes = qunit_bytes(qunit);
	uint32_t nunits_total = len / unit_bytes;
	uint8_t txm = dir_tx ? XEC_QSPI_CR_TXM_DATA : XEC_QSPI_CR_TXM_DIS;
	bool rx_en = !dir_tx;
	uint8_t txdma = XEC_QSPI_CR_TXDMA_DIS;
	uint8_t rxdma = XEC_QSPI_CR_RXDMA_DIS;

	if (ldma) {
		if (dir_tx) {
			txdma = XEC_QSPI_CR_TXDMA_TLDCH2;
		} else {
			rxdma = XEC_QSPI_CR_RXDMA_RLDCH0;
		}
		/* Single descriptor; LDMA channel length-override drives byte
		 * count. NQUNITS here is a placeholder; hardware ignores it
		 * when OVRL=1 on the channel.
		 */
		desc_write(hw, idx, /* MCHP Debug: use 0 for NQunits since we are LDMA len override? */
			   desc_build(data_ifm, txm, rx_en, txdma, rxdma, qunit, 1u,
				      (idx + 1u) & 0xFu, true, true));
		return idx;
	}

	/* PIO: split into multiple descriptors when nunits > 15-bit limit */
	while (nunits_total > 0) {
		uint16_t chunk =
			(nunits_total > QMSPI_NQUNITS_MAX) ? QMSPI_NQUNITS_MAX : nunits_total;
		bool last_chunk = (chunk == nunits_total);
		uint8_t next = last_chunk ? 0u : (idx + 1u) & 0xFu;

		desc_write(hw, idx,
			   desc_build(data_ifm, txm, rx_en, txdma, rxdma, qunit, chunk, next,
				      last_chunk, last_chunk));
		nunits_total -= chunk;
		if (!last_chunk) {
			idx = (idx + 1u) & 0xFu;
		}
	}
	return idx;
}

#ifdef CONFIG_MSPI_MCHP_XEC_QMSPI_LDMA
/* Program one local-DMA channel. channel_idx uses XEC_QSPI_LDMA_{RX,TX}_CH{0..2}. */
static void ldma_program(const struct mspi_mchp_xec_config *hw, uint8_t channel_idx,
			 const void *buf, uint32_t len, uint8_t access_sz, bool incr)
{
	uint32_t cr = BIT(XEC_QSPI_LDMA_CHX_CR_EN_POS) | BIT(XEC_QSPI_LDMA_CHX_CR_OVRL_POS);
	uint8_t sz;

	switch (access_sz) {
	case 4:
		sz = XEC_QSPI_LDMA_CHX_CR_SZ_4B;
		break;
	case 2:
		sz = XEC_QSPI_LDMA_CHX_CR_SZ_2B;
		break;
	default:
		sz = XEC_QSPI_LDMA_CHX_CR_SZ_1B;
		break;
	}
	cr |= XEC_QSPI_LDMA_CHX_CR_SZ_SET(sz);
	if (incr) {
		cr |= BIT(XEC_QSPI_LDMA_CHX_CR_INCRA_POS);
	}

	QW32(hw, XEC_QSPI_LDMA_CHX_SA_OFS(channel_idx), (uint32_t)(uintptr_t)buf);
	QW32(hw, XEC_QSPI_LDMA_CHX_LR_OFS(channel_idx), len);
	QW32(hw, XEC_QSPI_LDMA_CHX_CR_OFS(channel_idx), cr);
}
#endif /* CONFIG_MSPI_MCHP_XEC_QMSPI_LDMA */

/* Set IER for the next transfer, after clearing SR.
 * NOTE: DMA Done fires when ANY DMA channel or LDMA channel finished.
 * We use multiple LDMA channels and do not need the ISR to fire on each LDMA channel done.
 * The purpose of DMA Done status is if software requires more channels for a single transfer
 * than we have. For example, using central DMA might require DMA if we want to transmit
 * command, address, and data using the same central DMA channel.
 * Hardware implements three LDMA channel per direction. We do not need DMA Done interrupt.
 */
static void prime_interrupts(const struct mspi_mchp_xec_config *hw, bool pio, bool dir_tx)
{
	uint32_t ier = 0, triglvl = 0;

	QW32(hw, XEC_QSPI_SR_OFS, 0xFFFFFFFFu);

	ier = BIT(XEC_QSPI_IER_TXB_ERR_POS) | BIT(XEC_QSPI_IER_RXB_ERR_POS) |
	      BIT(XEC_QSPI_IER_PROG_ERR_POS);

	if (pio) {
		ier |= BIT(XEC_QSPI_IER_XFR_DONE_POS);
		if (dir_tx) {
			ier |= BIT(XEC_QSPI_IER_TXB_REQ_POS);
			triglvl = XEC_QSPI_BCNT_TR_TXB_SET(4U); /* interrupt on <= this value */
		} else {
			ier |= BIT(XEC_QSPI_IER_RXB_REQ_POS);
			triglvl = XEC_QSPI_BCNT_TR_RXB_SET(4U); /* interrupt on >= this value */
		}
	} else {
#if 0
		ier |= (BIT(XEC_QSPI_IER_DMA_DONE_POS) | BIT(XEC_QSPI_IER_LDMA_RX_ERR_POS) |
			BIT(XEC_QSPI_IER_LDMA_TX_ERR_POS) | BIT(XEC_QSPI_IER_XFR_DONE_POS));
#else
		ier |= (BIT(XEC_QSPI_IER_LDMA_RX_ERR_POS) | BIT(XEC_QSPI_IER_LDMA_TX_ERR_POS) |
			BIT(XEC_QSPI_IER_XFR_DONE_POS));
#endif
	}

	QW32(hw, XEC_QSPI_BCNT_TR_OFS, triglvl);
	QW32(hw, XEC_QSPI_IER_OFS, ier);
}

/* If GPIO CE is used we must assert it here and apply delay just
 * before starting hardware.
 * struct mspi_dev_id may have member ce (struct gpio_dt_spec) or
 * struct mspi_xfer may have member ce_sw_ctrl (struct mspi_ce_control)
 * struct mspi_ce_control contains members:
 *   gpio (struct gpio_dt_spec) and delay (uint32_t microseconds)
 * flash_mspi_nor driver will fill in struct mspi_dev_id ce from DT but
 * it zeros out struct mspi_xfer.ce_sw_ctrl.
 * This routine sets the pin to its logical active or inactivate state based
 * on the GPIO pin's flag: GPIO_ACTIVE_HIGH or GPIO_ACTIVE_LOW.
 */
static int mspi_gpio_ce_ctrl(const struct device *dev, const struct mspi_xfer *xfer, bool assert)
{
	struct mspi_mchp_xec_data *data = dev->data;
	const struct gpio_dt_spec *spec = NULL;
	int rc = 0, pin_logical_state = 0;
	uint32_t delay_us = 0;

	if (assert) {
		pin_logical_state = 1;
	}

	if (xfer->ce_sw_ctrl.gpio.port != NULL) {
		spec = &xfer->ce_sw_ctrl.gpio;
		delay_us = xfer->ce_sw_ctrl.delay;
		data->st.is_gpio_ce = true;

	} else if (data->active_dev->ce.port != NULL) {
		spec = &data->active_dev->ce;
		delay_us = 1U;
		data->st.is_gpio_ce = true;
	} else {
		data->st.is_gpio_ce = false;
	}

	if (spec != NULL) {
		if (!assert) {
			k_busy_wait(delay_us);
		}
		rc = gpio_pin_set_dt(spec, pin_logical_state);
		if (assert) {
			k_busy_wait(delay_us);
		}
	}

	return rc;
}

/* Caller must not clear QSPI.SR */
static int finalize_packet(const struct device *dev, int rc)
{
	const struct mspi_mchp_xec_config *hw = dev->config;
	struct mspi_mchp_xec_data *data = dev->data;
	uint32_t qstatus = QR32(hw, XEC_QSPI_SR_OFS);
	int rc2 = 0;

	if ((qstatus & QMSPI_SR_ERR_MSK) != 0) {
		rc = -EIO;
		LOG_ERR("Xfer HW error");
	} else if (rc == -ETIMEDOUT) {
		LOG_ERR("Xfer timeout");
	}

	/* if still running force it to stop on next unit boundary */
	if ((qstatus & BIT(XEC_QSPI_SR_XFR_DONE_POS)) == 0) {
		QW32(hw, XEC_QSPI_EXE_OFS, BIT(XEC_QSPI_EXE_STOP_POS));
		if ((QR32(hw, XEC_QSPI_SR_OFS) & BIT(XEC_QSPI_SR_XFR_DONE_POS)) != 0) {
			/* clear activate to gate clocks off */
			QRMW32(hw, XEC_QSPI_MODE_OFS, BIT(XEC_QSPI_MODE_ACTV_POS), 0);
		}
	}

	if (data->st.dev_id->ce.port != NULL) {
		rc2 = gpio_pin_set_dt(&data->st.dev_id->ce, 1);
		if (rc2 < 0) {
			LOG_ERR("Error deassert GPIO CE line (%d)", rc2);
			return rc2;
		}
	}

	return rc;
}

static int finalize_xfer(const struct device *dev, int rc)
{
	struct mspi_mchp_xec_data *data = dev->data;

#ifdef CONFIG_MULTITHREADING
	k_sem_give(&data->xfer_done);
#else
	data->xfer_done = true;
#endif
	LOG_DBG("XEC finalize xfer (%d)", rc);

	return 0;
}


/* MSPI_DMA uses LDMA for cmd, addr, and data phases */
static int submit_packet(const struct device *dev, uint32_t pkt_idx)
{
	const struct mspi_mchp_xec_config *hw = dev->config;
	struct mspi_mchp_xec_data *data = dev->data;
	const struct mspi_xfer *xfer = data->st.xfer;
	const struct mspi_xfer_packet *pkt = &xfer->packets[pkt_idx];
	bool dir_tx = (pkt->dir == MSPI_TX);
	uint32_t cr = 0;
	int rc = 0;
	uint16_t dummy_cycles = dir_tx ? xfer->tx_dummy : xfer->rx_dummy;
	uint8_t cmd_len = xfer->cmd_length;
	uint8_t addr_len = xfer->addr_length;
	uint8_t qunit, access, idx;
	uint8_t desc_idx = 0;
	bool use_ldma = false;

#ifdef CONFIG_MSPI_MCHP_XEC_QMSPI_LDMA
	use_ldma = (xfer->xfer_mode == MSPI_DMA);
#else
	if (xfer->xfer_mode == MSPI_DMA) {
		LOG_ERR("DMA mode requested but CONFIG_MSPI_MCHP_XEC_QMSPI_LDMA=n");
		return -ENOTSUP;
	}
#endif

	data->cmd_buf.w = 0;
	data->addr_buf.w = 0;

	pick_unit_and_access(pkt->data_buf, pkt->num_bytes, &qunit, &access);
	data->st.pio_access = access;
	data->st.pio_pos = 0;

	/* Clear LDMA enable bitmaps from any prior transfer. */
#if 1
	prep_qspi(hw);
#else
	QW32(hw, XEC_QSPI_LDMA_RX_EN_OFS, 0);
	QW32(hw, XEC_QSPI_LDMA_TX_EN_OFS, 0);
#endif

	/* ----- Descriptor 0: command phase (TX, single lane) ----- */
	if (cmd_len > 0) {
		uint8_t txdma = XEC_QSPI_CR_TXDMA_DIS;

		/* Format cmd as MSB-first and adjust for length */
		sys_put_be32(pkt->cmd, data->cmd_buf.b);
		data->cmd_buf.w >>= ((4U - cmd_len) * 8U);

		if (use_ldma) {
			txdma = XEC_QSPI_CR_TXDMA_TLDCH0;
		} else {
			/* preload the TX FIFO */
			for (idx = 0; idx < cmd_len; idx++) {
				QW8(hw, XEC_QSPI_TXB_OFS, data->cmd_buf.b[idx]);
			}
		}

		desc_write(hw, desc_idx,
			   desc_build(IFM_SINGLE, XEC_QSPI_CR_TXM_DATA, false, txdma,
				      XEC_QSPI_CR_RXDMA_DIS, XEC_QSPI_CR_QUNIT_1B, cmd_len,
				      (desc_idx + 1u) & 0xFu, false, false));
#ifdef CONFIG_MSPI_MCHP_XEC_QMSPI_LDMA
		if (use_ldma) {
			ldma_program(hw, XEC_QSPI_LDMA_TX_CH0, &data->cmd_buf.b[0], cmd_len, 1u,
				     true);
			QW32(hw, XEC_QSPI_LDMA_TX_EN_OFS,
			     QR32(hw, XEC_QSPI_LDMA_TX_EN_OFS) | BIT(desc_idx));
		}
#else
		/* preload the TX FIFO */
		for (idx = 0; idx < cmd_len; idx++) {
			QW8(hw, XEC_QSPI_TXB_OFS, data->cmd_buf.b[idx]);
		}
#endif
		desc_idx++;
	}

	/* ----- Descriptor 1: address phase (TX, 1-1-N or 1-N-N) ----- */
	if (addr_len > 0) {
		uint8_t txdma = XEC_QSPI_CR_TXDMA_DIS;
		uint8_t ifm = data->iodec.addr_ifm;
		uint32_t a = pkt->address;

		/* Serialize address MSB-first into addr_buf. */
		if (addr_len == 4) {
			sys_put_be32(a, data->cmd_buf.b);
		} else if (addr_len == 3) {
			data->addr_buf.b[0] = (uint8_t)(a >> 16);
			data->addr_buf.b[1] = (uint8_t)(a >> 8);
			data->addr_buf.b[2] = (uint8_t)a;
		} else if (addr_len == 2) {
			sys_put_be16((uint16_t)a, &data->addr_buf.b[0]);
		} else {
			data->addr_buf.b[0] = (uint8_t)a;
		}

		if (use_ldma) {
			txdma = XEC_QSPI_CR_TXDMA_TLDCH1;
		} else {
			/* Preload the TX FIFO */
			for (idx = 0; idx < addr_len; idx++) {
				QW8(hw, XEC_QSPI_TXB_OFS, data->addr_buf.b[idx]);
			}
		}

		desc_write(hw, desc_idx,
			   desc_build(ifm, XEC_QSPI_CR_TXM_DATA, false, txdma,
				      XEC_QSPI_CR_RXDMA_DIS, XEC_QSPI_CR_QUNIT_1B, addr_len,
				      (desc_idx + 1u) & 0xFu, false, false));
#ifdef CONFIG_MSPI_MCHP_XEC_QMSPI_LDMA
		if (use_ldma) {
			ldma_program(hw, XEC_QSPI_LDMA_TX_CH1, &data->addr_buf.b[0], addr_len, 1u,
				     true);
			QW32(hw, XEC_QSPI_LDMA_TX_EN_OFS,
			     QR32(hw, XEC_QSPI_LDMA_TX_EN_OFS) | BIT(desc_idx));
		}
#endif
		desc_idx++;
	}

	/* ----- Optional dummy-clocks descriptor ----- */
	if (dummy_cycles > 0) {
		desc_write(hw, desc_idx,
			   desc_build(IFM_SINGLE, XEC_QSPI_CR_TXM_DIS, false,
				      XEC_QSPI_CR_TXDMA_DIS, XEC_QSPI_CR_RXDMA_DIS,
				      XEC_QSPI_CR_QUNIT_BITS, dummy_cycles,
				      (desc_idx + 1u) & 0xFu, false, false));
		desc_idx++;
	}

	/* ----- Data phase ----- */
	if (pkt->num_bytes > 0) {
		uint8_t last = write_data_descriptors(hw, desc_idx, data->iodec.data_ifm, dir_tx,
						      qunit, pkt->num_bytes, use_ldma);
		(void)last;
#ifdef CONFIG_MSPI_MCHP_XEC_QMSPI_LDMA
		if (use_ldma) {
			uint8_t ch = dir_tx ? XEC_QSPI_LDMA_TX_CH2 : XEC_QSPI_LDMA_RX_CH0;
			uint32_t ofs_en = dir_tx ? XEC_QSPI_LDMA_TX_EN_OFS
						 : XEC_QSPI_LDMA_RX_EN_OFS;

			ldma_program(hw, ch, pkt->data_buf, pkt->num_bytes, access, true);
			QW32(hw, ofs_en, QR32(hw, ofs_en) | BIT(desc_idx));
		}
#endif
	} else {
		/* No data phase — mark the last non-data descriptor as LD/CLOSE. */
		uint32_t d;
		uint8_t fix = (desc_idx == 0) ? 0 : (desc_idx - 1u);

		d = QR32(hw, XEC_QSPI_DESCR_OFS(fix));
		d |= BIT(XEC_QSPI_CR_CLOSE_EN_POS) | BIT(XEC_QSPI_DR_LD_POS);
		QW32(hw, XEC_QSPI_DESCR_OFS(fix), d);
	}

	/* ----- Start: enable descriptor mode and fire ----- */
	if (use_ldma) {
		cr = QR32(hw, XEC_QSPI_MODE_OFS);
		cr |= BIT(XEC_QSPI_MODE_LD_RX_EN_POS) | BIT(XEC_QSPI_MODE_LD_TX_EN_POS);
		QW32(hw, XEC_QSPI_MODE_OFS, cr);
	}

	cr = QR32(hw, XEC_QSPI_CR_OFS);
	cr &= (uint32_t)~XEC_QSPI_CR_NQUNITS_MSK; /* defensive: clear any leftover */
	cr |= BIT(XEC_QSPI_CR_DESCR_EN_POS);
	QW32(hw, XEC_QSPI_CR_OFS, cr);

	rc = mspi_gpio_ce_ctrl(dev, xfer, true);
	if (rc != 0) {
		return rc;
	}

	prime_interrupts(hw, !use_ldma, dir_tx);

	/* Set the starting descriptor index in the STATUS CDESCR field via
	 * writing it in the execute path: the QMSPI starts at descriptor 0 by
	 * default when DESCR_EN is set, so we simply START.
	 */
	QW32(hw, XEC_QSPI_EXE_OFS, BIT(XEC_QSPI_EXE_START_POS));

	return 0;
}

/* API: transceive ----------------------------------------------------------- */

/* Transceive API should only be called after a successful call to device config which
 * acquires the driver lock. Transceive will release the driver lock on error or if
 * it completes all packets in req.
 */
static int mspi_mchp_xec_qm_transceive_api(const struct device *dev,
					   const struct mspi_dev_id *dev_id,
					   const struct mspi_xfer *req)
{
	const struct mspi_mchp_xec_config *hw = dev->config;
	struct mspi_mchp_xec_data *data = dev->data;
	int rc = 0;

	if (data->st.active) { /* Driver is busy do not release lock */
		return -EBUSY;
	}

	if ((!data->dev_cfg_valid) || (data->active_dev != dev_id)) {
		LOG_ERR("dev_id mismatch");
		return -EPERM;
	}

	if ((req == NULL) || ((req->num_packet > 0) && req->packets == NULL)) {
		LOG_ERR("Bad  params in req");
		return -EINVAL;
	}

	if ((req->xfer_mode != MSPI_PIO) && (req->xfer_mode != MSPI_DMA)) {
		LOG_ERR("Invalid xfer mode %u", req->xfer_mode);
		return -ENOTSUP;
	}

#ifdef MSPI_MCHP_XEC_DEBUG_ISR
	data->isr_count = 0;
	data->qstatus = 0;
	data->qbc_status = 0;
	data->qier = 0;
	data->qbc_trig = 0;
#endif

	data->st.xfer = req;
	data->st.dev_id = dev_id;
	data->st.pkt_idx = 0;
	data->st.err = 0;
	data->st.async = req->async;
	data->st.active = true;

	if (req->num_packet == 0) {
		data->st.active = false;
		goto xfr_exit;
	}

	rc = submit_packet(dev, 0);
	if (rc != 0) {
		data->st.active = false;
		goto xfr_exit;
	}

	if (!req->async) {
#ifdef MSPI_MCHP_XEC_DEBUG_NO_TIMEOUT
		k_mutex_unlock(&data->lock);
		rc = k_sem_take(&data->xfer_done, K_FOREVER);
		k_mutex_lock(&data->lock, K_FOREVER);
#else
		k_timeout_t to = (req->timeout == 0) ? K_FOREVER : K_MSEC(req->timeout);

		k_mutex_unlock(&data->lock);
		rc = k_sem_take(&data->xfer_done, to);
		k_mutex_lock(&data->lock, K_FOREVER);
#endif
		if (req->ce_sw_ctrl.gpio.port != NULL) {
			if (!req->hold_ce) {
				k_busy_wait(req->ce_sw_ctrl.delay);

				rc = gpio_pin_set_dt(&req->ce_sw_ctrl.gpio, 1);
				if (rc != 0) {
					LOG_ERR("Failed to de-assert GPIO CE line (%d)", rc);
					rc = -EAGAIN;
				}
			}
		}

		if (rc == -EAGAIN) {
			QW32(hw, XEC_QSPI_EXE_OFS,
			     BIT(XEC_QSPI_EXE_STOP_POS) | BIT(XEC_QSPI_EXE_CLR_FIFOS_POS));
			QW32(hw, XEC_QSPI_IER_OFS, 0);
			data->st.active = false;
			rc = -ETIMEDOUT;
		} else {
			rc = data->st.err;
		}

		data->st.active = false;
	}

xfr_exit:
	k_mutex_unlock(&data->lock);
	return rc;
}

/* ISR ---------------------------------------------------------------------- */

static inline void pio_fifo_fill_tx(const struct mspi_mchp_xec_config *hw,
				    struct mspi_mchp_xec_data *data,
				    const struct mspi_xfer_packet *pkt)
{
	uint32_t sr;

	while (data->st.pio_pos < pkt->num_bytes) {
		sr = QR32(hw, XEC_QSPI_SR_OFS);
		if (sr & BIT(XEC_QSPI_SR_TXB_FULL_POS)) {
			break;
		}
		if (data->st.pio_access == 4 &&
		    (pkt->num_bytes - data->st.pio_pos) >= 4) {
			uint32_t w = sys_get_le32(&pkt->data_buf[data->st.pio_pos]);

			QW32(hw, XEC_QSPI_TXB_OFS, w);
			data->st.pio_pos += 4u;
		} else if (data->st.pio_access >= 2 &&
			   (pkt->num_bytes - data->st.pio_pos) >= 2) {
			uint16_t h = sys_get_le16(&pkt->data_buf[data->st.pio_pos]);

			QW32(hw, XEC_QSPI_TXB_OFS, (uint32_t)h);
			data->st.pio_pos += 2u;
		} else {
			QW32(hw, XEC_QSPI_TXB_OFS, (uint32_t)pkt->data_buf[data->st.pio_pos]);
			data->st.pio_pos += 1u;
		}
	}
}

/* AI generated crap
 * You cannot read more bytes from the RX FIFO than the amount in it otherwise
 * HW sets RXB Error and the RXB count wraps producing incorrect results!
 */
static inline void pio_fifo_drain_rx(const struct mspi_mchp_xec_config *hw,
				     struct mspi_mchp_xec_data *data,
				     const struct mspi_xfer_packet *pkt)
{
	uint32_t rxb_cnt = XEC_QSPI_BCNT_SR_RXB_GET(QR32(hw, XEC_QSPI_BCNT_SR_OFS));

	/* failsafe: read one byte at a time. We do not take into account buffer alignment
	 * and number of bytes in RX FIFO
	 */
	while (data->st.pio_pos < pkt->num_bytes) {
		if (rxb_cnt == 0) {
			break;
		}

		pkt->data_buf[data->st.pio_pos] = QR8(hw, XEC_QSPI_RXB_OFS);
		data->st.pio_pos++;

		rxb_cnt = XEC_QSPI_BCNT_SR_RXB_GET(QR32(hw, XEC_QSPI_BCNT_SR_OFS));
	}
}

#ifdef CONFIG_MULTITHREADING
static void fire_xfer_complete_cb(struct mspi_mchp_xec_data *data,
				  const struct device *dev,
				  const struct mspi_xfer_packet *pkt, uint32_t pkt_idx,
				  uint32_t status)
{
	mspi_callback_handler_t cb = data->cbs[MSPI_BUS_XFER_COMPLETE];
	struct mspi_callback_context *ctx = data->cb_ctxs[MSPI_BUS_XFER_COMPLETE];

	if (cb == NULL || ctx == NULL) {
		return;
	}
	if (!(pkt->cb_mask & MSPI_BUS_XFER_COMPLETE_CB)) {
		return;
	}

	ctx->mspi_evt.evt_type = MSPI_BUS_XFER_COMPLETE;
	ctx->mspi_evt.evt_data.controller = dev;
	ctx->mspi_evt.evt_data.dev_id = data->st.dev_id;
	ctx->mspi_evt.evt_data.packet = pkt;
	ctx->mspi_evt.evt_data.status = status;
	ctx->mspi_evt.evt_data.packet_idx = pkt_idx;
	cb(ctx);
}

static void mspi_mchp_xec_async_tmout_wh(struct k_work *work)
{
	struct mspi_mchp_xec_data *xdat =
		CONTAINER_OF(work, struct mspi_mchp_xec_data, async_pkt_work);
	const struct device *dev = xdat->dev;
	mspi_callback_handler_t cb = xdat->cbs[MSPI_BUS_TIMEOUT];
	struct mspi_callback_context *cbctx = xdat->cb_ctxs[MSPI_BUS_TIMEOUT];

	if (cb == NULL) {
		LOG_ERR("Bus timeout: No callback registered");
		return;
	}

	if (cbctx != NULL) {
		cbctx->mspi_evt.evt_type = MSPI_BUS_TIMEOUT;
		cbctx->mspi_evt.evt_data.controller = dev;
		cbctx->mspi_evt.evt_data.dev_id = xdat->st.dev_id;
		cbctx->mspi_evt.evt_data.packet = xdat->st.xfer->packets;
		cbctx->mspi_evt.evt_data.status = -ETIMEDOUT;
		cbctx->mspi_evt.evt_data.packet_idx = xdat->st.pkt_idx;
	}

	cb(cbctx);
}

static void mspi_mchp_xec_async_pkt_wh(struct k_work *work)
{
	struct mspi_mchp_xec_data *data =
		CONTAINER_OF(work, struct mspi_mchp_xec_data, async_pkt_work);
	const struct device *dev = data->dev;
	const struct mspi_xfer_packet *pkt = &data->st.xfer->packets[data->st.pkt_idx];\
	int rc = 0;

	/* TODO last parameter */
	fire_xfer_complete_cb(data, dev, pkt, data->st.pkt_idx, 0u);

	/* TODO next packet */
	data->st.pkt_idx++;
	if (data->st.pkt_idx < data->st.xfer->num_packet) {
		rc = submit_packet(dev, data->st.pkt_idx);
		if (rc != 0) {
			data->st.err = rc;
		}
	} else {
		finalize_xfer(dev, rc);
	}
}
#endif /* CONFIG_MULTITHREADING */

static void mspi_mchp_xec_isr(const struct device *dev)
{
	const struct mspi_mchp_xec_config *hw = dev->config;
	struct mspi_mchp_xec_data *data = dev->data;
	uint32_t sr = QR32(hw, XEC_QSPI_SR_OFS);
	const uint32_t err_mask =
		BIT(XEC_QSPI_SR_TXB_ERR_POS) | BIT(XEC_QSPI_SR_RXB_ERR_POS) |
		BIT(XEC_QSPI_SR_PROG_ERR_POS) | BIT(XEC_QSPI_SR_LDMA_RX_ERR_POS) |
		BIT(XEC_QSPI_SR_LDMA_TX_ERR_POS);

#ifdef MSPI_MCHP_XEC_DEBUG_ISR
	data->isr_count++;
	data->qstatus = sr;
	data->qbc_status = QR32(hw, XEC_QSPI_BCNT_SR_OFS);
	data->qier = QR32(hw, XEC_QSPI_IER_OFS);
	data->qbc_trig = QR32(hw, XEC_QSPI_BCNT_TR_OFS);
#endif

	if (!data->st.active) {
		QW32(hw, XEC_QSPI_SR_OFS, sr);
		(void)soc_ecia_girq_status_clear(MCHP_XEC_ECIA_GIRQ(hw->girq_enc),
						 MCHP_XEC_ECIA_GIRQ_POS(hw->girq_enc));
		return;
	}

	if (sr & err_mask) {
		LOG_ERR("QMSPI error: SR=0x%08x", sr);
		data->st.err = -EIO;
		QW32(hw, XEC_QSPI_EXE_OFS,
		     BIT(XEC_QSPI_EXE_STOP_POS) | BIT(XEC_QSPI_EXE_CLR_FIFOS_POS));
		QW32(hw, XEC_QSPI_IER_OFS, 0);
		QW32(hw, XEC_QSPI_SR_OFS, sr);
		goto complete_xfer;
	}

	/* PIO FIFO servicing */
	if (sr & (BIT(XEC_QSPI_SR_TXB_REQ_POS) | BIT(XEC_QSPI_SR_RXB_REQ_POS))) {
		const struct mspi_xfer_packet *pkt = &data->st.xfer->packets[data->st.pkt_idx];

		if (pkt->dir == MSPI_TX) {
			pio_fifo_fill_tx(hw, data, pkt);
		} else {
			pio_fifo_drain_rx(hw, data, pkt);
		}
		QW32(hw, XEC_QSPI_SR_OFS,
		     sr & (BIT(XEC_QSPI_SR_TXB_REQ_POS) | BIT(XEC_QSPI_SR_RXB_REQ_POS)));
	}

	if (sr & (BIT(XEC_QSPI_SR_XFR_DONE_POS) | BIT(XEC_QSPI_SR_DMA_DONE_POS))) {
		const struct mspi_xfer_packet *pkt = &data->st.xfer->packets[data->st.pkt_idx];

		/* Final RX drain for PIO */
		if (pkt->dir == MSPI_RX && data->st.xfer->xfer_mode == MSPI_PIO) {
			pio_fifo_drain_rx(hw, data, pkt);
		}
		QW32(hw, XEC_QSPI_SR_OFS, sr);

		/* TODO - When GPIO CE is used we can de-assert here before
		 * invoking the completion callback. ISSUE: MSPI has a delay
		 * in microseconds before deassertion of the GPIO CE. We can't
		 * delay in ISR context. Recommended solution is to submit
		 * a work function to the kernel and exit the ISR.
		 * ISSUE#2: The logic checking for another packet would have
		 * to be moved into the worker function!
		 * OR cheat and de-assert if hold_ce is not set and forget
		 * about the delay.
		 */
		if ((data->st.async) && (data->st.is_gpio_ce)) {
			k_work_submit(&data->async_pkt_work);
			goto out;
		}

		fire_xfer_complete_cb(data, dev, pkt, data->st.pkt_idx, 0u);

		data->st.pkt_idx++;
		if (data->st.pkt_idx < data->st.xfer->num_packet) {
			int rc = submit_packet(dev, data->st.pkt_idx);

			if (rc != 0) {
				data->st.err = rc;
				goto complete_xfer;
			}
			goto out;
		}
		goto complete_xfer;
	}

	/* Unhandled status bits - clear to keep IRQ from re-firing. */
	QW32(hw, XEC_QSPI_SR_OFS, sr);
	goto out;

complete_xfer:
	QW32(hw, XEC_QSPI_IER_OFS, 0);
	if (data->st.async) {
		data->st.active = false;
	}
	k_sem_give(&data->xfer_done);
out:
	(void)soc_ecia_girq_status_clear(MCHP_XEC_ECIA_GIRQ(hw->girq_enc),
					 MCHP_XEC_ECIA_GIRQ_POS(hw->girq_enc));
}

/* PM ----------------------------------------------------------------------- */
#ifdef CONFIG_PM_DEVICE
static int mspi_mchp_xec_pm_action(const struct device *ctrl, enum pm_device_action action)
{
	const struct mspi_mchp_xec_config *cfg = ctrl->config;
	int rc = 0;

	if (action == PM_DEVICE_ACTION_RESUME) {
		rc = pinctrl_appy_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	} else if (action == PM_DEVICE_ACTION_SUSPEND) {
		rc = pinctrl_appy_state(cfg->pcfg, PINCTRL_STATE_SLEEP);
	} else {
		rc = -ENOTSUP;
	}

	return rc;
}
#endif
/* Init --------------------------------------------------------------------- */

static int mspi_mchp_xec_init(const struct device *dev)
{
	const struct mspi_mchp_xec_config *cfg = dev->config;
	struct mspi_mchp_xec_data *data = dev->data;
	int rc = 0;

#ifdef CONFIG_MULTITHREADING
	k_mutex_init(&data->lock);
	k_sem_init(&data->xfer_done, 0, 1);
	k_work_init(&data->async_pkt_work, mspi_mchp_xec_async_pkt_wh);
	k_work_init(&data->async_tmout_work, mspi_mchp_xec_async_tmout_wh);
#endif
	rc = qmspi_hw_reset_and_init(dev);
	if (rc != 0) {
		return rc;
	}

	if (cfg->irq_cfg_func != NULL) {
		cfg->irq_cfg_func();
	}

	const struct mspi_dt_spec spec = {
		.bus = dev,
		.config = {0}, /* TODO */
	};

	return mspi_mchp_xec_qm_config_api(&spec);
}

static DEVICE_API(mspi, mspi_mchp_xec_api) = {
	.config = mspi_mchp_xec_qm_config_api,
	.dev_config = mspi_mchp_xec_qm_dev_config_api,
	.get_channel_status = mspi_mchp_xec_qm_get_chan_sts_api,
	.transceive = mspi_mchp_xec_qm_transceive_api,
#ifdef CONFIG_MULTITHREADING
	.register_callback = mspi_mchp_xec_qm_reg_cb_api,
#endif
};

/* Per-instance instantiation ------------------------------------------------ */

#define MSPI_MCHP_XEC_CE_GPIOS(n)                                                                  \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, ce_gpios),                                            \
		(static struct gpio_dt_spec ce_gpios_##n[] = MSPI_CE_GPIOS_DT_SPEC_INST_GET(n);),  \
		(static struct gpio_dt_spec ce_gpios_##n[1];))

#define MSPI_MCHP_XEC_CE_GPIOS_LEN(n)                                                              \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, ce_gpios), (DT_INST_PROP_LEN(n, ce_gpios)), (0))

#define MSPI_MCHP_XEC_DEFINE(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	PM_DEVICE_DT_INST_DEFINE(n, mspi_mchp_xec_pm_action);                                      \
	MSPI_MCHP_XEC_CE_GPIOS(n)                                                                  \
	static void mspi_mchp_xec_irq_cfg_##n(void)                                                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),                             \
			    mspi_mchp_xec_isr, DEVICE_DT_INST_GET(n), 0);                          \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static struct mspi_mchp_xec_data mspi_mchp_xec_data_##n;                                   \
	static const struct mspi_mchp_xec_config mspi_mchp_xec_cfg_##n = {                         \
		.regs = DT_INST_REG_ADDR(n),                                                       \
		.src_clk_hz = DT_INST_PROP(n, clock_frequency),                                    \
		.pcr_enc = (uint16_t)DT_INST_PROP(n, pcr),                                         \
		.girq_enc = (uint16_t)DT_INST_PROP(n, girqs),                                      \
		.cs_count = DT_INST_PROP_OR(n, chip_select_count, 1),                              \
		.num_ce_gpios = MSPI_MCHP_XEC_CE_GPIOS_LEN(n),                                     \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.ce_gpios = ce_gpios_##n,                                                          \
		.sw_multi_periph = DT_INST_PROP(n, software_multiperipheral),                      \
		.irq_cfg_func = mspi_mchp_xec_irq_cfg_##n,                                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, mspi_mchp_xec_init, PM_DEVICE_DT_INST_GET(n),                     \
			      &mspi_mchp_xec_data_##n, &mspi_mchp_xec_cfg_##n,                     \
			      POST_KERNEL, CONFIG_MSPI_INIT_PRIORITY, &mspi_mchp_xec_api);

DT_INST_FOREACH_STATUS_OKAY(MSPI_MCHP_XEC_DEFINE)
