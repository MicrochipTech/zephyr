/*
 * Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec_qmspi_controller

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>

/* soc.h pulls in soc_common.h which includes reg/mec_qmspi.h */
#include <soc.h>

LOG_MODULE_REGISTER(mspi_xec_qspi, CONFIG_MSPI_LOG_LEVEL);

/* --------------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------------- */

#define QSPI_FIFO_DEPTH        8u
#define QSPI_HIGH_FREQ_HZ     48000000u
#define QSPI_MAX_DESCR         16u
#define QSPI_NQUNITS_MAX      32767u   /* 15-bit field, bits[31:17] */

/* Descriptor indices for standard 4-phase transfer */
#define DESCR_IDX_CMD          0
#define DESCR_IDX_ADDR         1
#define DESCR_IDX_DUMMY        2
#define DESCR_IDX_DATA         3

/* Aggregate error mask for status register */
#define QSPI_ERR_MASK                                       \
	(BIT(XEC_QSPI_SR_TXB_ERR_POS) |                    \
	 BIT(XEC_QSPI_SR_RXB_ERR_POS) |                    \
	 BIT(XEC_QSPI_SR_PROG_ERR_POS) |                   \
	 BIT(XEC_QSPI_SR_LDMA_RX_ERR_POS) |                \
	 BIT(XEC_QSPI_SR_LDMA_TX_ERR_POS))

/* W1C bits to clear before each transfer */
#define QSPI_CLR_STATUS_MASK                                \
	(BIT(XEC_QSPI_SR_XFR_DONE_POS) |                   \
	 BIT(XEC_QSPI_SR_DMA_DONE_POS) |                   \
	 BIT(XEC_QSPI_SR_TXB_ERR_POS) |                    \
	 BIT(XEC_QSPI_SR_RXB_ERR_POS) |                    \
	 BIT(XEC_QSPI_SR_PROG_ERR_POS) |                   \
	 BIT(XEC_QSPI_SR_TXB_REQ_POS) |                    \
	 BIT(XEC_QSPI_SR_TXB_STALL_POS) |                  \
	 BIT(XEC_QSPI_SR_RXB_REQ_POS) |                    \
	 BIT(XEC_QSPI_SR_RXB_STALL_POS))

/* Interrupt enable bits for error conditions */
#define QSPI_IER_ERRORS                                     \
	(BIT(XEC_QSPI_IER_TXB_ERR_POS) |                   \
	 BIT(XEC_QSPI_IER_RXB_ERR_POS) |                   \
	 BIT(XEC_QSPI_IER_PROG_ERR_POS) |                  \
	 BIT(XEC_QSPI_IER_LDMA_RX_ERR_POS) |               \
	 BIT(XEC_QSPI_IER_LDMA_TX_ERR_POS))

/* --------------------------------------------------------------------------
 * Data structures
 * -------------------------------------------------------------------------- */

struct mspi_xec_config {
	uint32_t base;
	uint32_t clk_freq;
	uint32_t pcr;
	uint32_t girq;
	uint8_t  chip_select_count;
	bool     sw_multi_periph;
	void     (*irq_config_func)(void);
	const struct pinctrl_dev_config *pcfg;
	const struct gpio_dt_spec *ce_gpios;
	uint8_t  num_ce_gpios;
};

struct mspi_xec_data {
	struct k_sem             bus_lock;
	struct k_sem             xfer_sync;
	const struct mspi_dev_id *active_dev_id;
	struct mspi_dev_cfg      active_dev_cfg;
	volatile int             xfer_err;

	/* PIO interrupt-driven state */
	const uint8_t            *tx_buf;
	volatile uint32_t        tx_remaining;
	uint8_t                  *rx_buf;
	volatile uint32_t        rx_remaining;

	/* Callbacks indexed by enum mspi_bus_event */
	mspi_callback_handler_t         cbs[MSPI_BUS_EVENT_MAX];
	struct mspi_callback_context    *cb_ctxs[MSPI_BUS_EVENT_MAX];
};

/* --------------------------------------------------------------------------
 * Register access helpers
 * -------------------------------------------------------------------------- */

static inline uint32_t qspi_read32(uint32_t base, uint32_t off)
{
	return sys_read32(base + off);
}

static inline void qspi_write32(uint32_t base, uint32_t off, uint32_t val)
{
	sys_write32(val, base + off);
}

static inline void qspi_write8(uint32_t base, uint32_t off, uint8_t val)
{
	sys_write8(val, base + off);
}

static inline uint8_t qspi_read8(uint32_t base, uint32_t off)
{
	return sys_read8(base + off);
}

/* --------------------------------------------------------------------------
 * Clock divider calculation
 *
 * Mode register bits[31:16] is a 16-bit clock divider field:
 *   0       = divide by 65536
 *   1       = divide by 1 (no division)
 *   2..65535 = divide by 2..65535
 * -------------------------------------------------------------------------- */

static uint16_t qspi_calc_clk_div(uint32_t src_clk, uint32_t target_freq,
				   uint32_t *actual_freq)
{
	uint32_t div;

	if (target_freq == 0 || target_freq >= src_clk) {
		/* No division needed */
		*actual_freq = src_clk;
		return 1u;
	}

	/* Round up divider so we don't exceed target */
	div = (src_clk + target_freq - 1u) / target_freq;

	if (div > 65536u) {
		div = 65536u;
	}

	*actual_freq = src_clk / div;

	/* Register encoding: 0 means 65536 */
	return (uint16_t)(div & 0xFFFFu);
}

/* --------------------------------------------------------------------------
 * CPP mode -> QMSPI Mode register bits (8:10)
 *
 * Bit 8:  CPOL
 * Bit 9:  CPHA_SDI (sampling edge)
 * Bit 10: CPHA_SDO (output edge)
 *
 * When actual frequency >= 48 MHz, invert CPHA_SDO.
 * -------------------------------------------------------------------------- */

static uint32_t qspi_cpp_to_mode_bits(enum mspi_cpp_mode cpp, uint32_t actual_freq)
{
	uint32_t bits = 0;

	switch (cpp) {
	case MSPI_CPP_MODE_0: /* CPOL=0, CPHA_SDI=0, CPHA_SDO=0 */
		bits = 0;
		break;
	case MSPI_CPP_MODE_1: /* CPOL=0, CPHA_SDI=1, CPHA_SDO=1 */
		bits = BIT(XEC_QSPI_MODE_CPHA_SDI_FE_POS) |
		       BIT(XEC_QSPI_MODE_CPHA_SDO_SE_POS);
		break;
	case MSPI_CPP_MODE_2: /* CPOL=1, CPHA_SDI=0, CPHA_SDO=0 */
		bits = BIT(XEC_QSPI_MODE_CPOL_HI_POS);
		break;
	case MSPI_CPP_MODE_3: /* CPOL=1, CPHA_SDI=1, CPHA_SDO=1 */
		bits = BIT(XEC_QSPI_MODE_CPOL_HI_POS) |
		       BIT(XEC_QSPI_MODE_CPHA_SDI_FE_POS) |
		       BIT(XEC_QSPI_MODE_CPHA_SDO_SE_POS);
		break;
	default:
		bits = 0;
		break;
	}

	/* High-frequency adjustment: invert CPHA_SDO */
	if (actual_freq >= QSPI_HIGH_FREQ_HZ) {
		bits ^= BIT(XEC_QSPI_MODE_CPHA_SDO_SE_POS);
	}

	return bits;
}

/* --------------------------------------------------------------------------
 * IO mode -> per-phase IFM values
 *
 * IFM encoding in QMSPI Control/Descriptor register:
 *   0 = single (full-duplex path, but used as single for half-duplex)
 *   1 = dual
 *   2 = quad
 * -------------------------------------------------------------------------- */

static int qspi_io_mode_to_ifm(enum mspi_io_mode mode,
				uint8_t *cmd_ifm, uint8_t *addr_ifm,
				uint8_t *data_ifm)
{
	switch (mode) {
	case MSPI_IO_MODE_SINGLE:
		*cmd_ifm  = XEC_QSPI_CR_IFM_FD;
		*addr_ifm = XEC_QSPI_CR_IFM_FD;
		*data_ifm = XEC_QSPI_CR_IFM_FD;
		break;
	case MSPI_IO_MODE_DUAL: /* 2-2-2 */
		*cmd_ifm  = XEC_QSPI_CR_IFM_DUAL;
		*addr_ifm = XEC_QSPI_CR_IFM_DUAL;
		*data_ifm = XEC_QSPI_CR_IFM_DUAL;
		break;
	case MSPI_IO_MODE_DUAL_1_1_2:
		*cmd_ifm  = XEC_QSPI_CR_IFM_FD;
		*addr_ifm = XEC_QSPI_CR_IFM_FD;
		*data_ifm = XEC_QSPI_CR_IFM_DUAL;
		break;
	case MSPI_IO_MODE_DUAL_1_2_2:
		*cmd_ifm  = XEC_QSPI_CR_IFM_FD;
		*addr_ifm = XEC_QSPI_CR_IFM_DUAL;
		*data_ifm = XEC_QSPI_CR_IFM_DUAL;
		break;
	case MSPI_IO_MODE_QUAD: /* 4-4-4 */
		*cmd_ifm  = XEC_QSPI_CR_IFM_QUAD;
		*addr_ifm = XEC_QSPI_CR_IFM_QUAD;
		*data_ifm = XEC_QSPI_CR_IFM_QUAD;
		break;
	case MSPI_IO_MODE_QUAD_1_1_4:
		*cmd_ifm  = XEC_QSPI_CR_IFM_FD;
		*addr_ifm = XEC_QSPI_CR_IFM_FD;
		*data_ifm = XEC_QSPI_CR_IFM_QUAD;
		break;
	case MSPI_IO_MODE_QUAD_1_4_4:
		*cmd_ifm  = XEC_QSPI_CR_IFM_FD;
		*addr_ifm = XEC_QSPI_CR_IFM_QUAD;
		*data_ifm = XEC_QSPI_CR_IFM_QUAD;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

/* --------------------------------------------------------------------------
 * Descriptor builders
 *
 * Descriptor register layout (same as Control register for bits 0-11,16+):
 *   [1:0]   IFM        Interface mode (0=single, 1=dual, 2=quad)
 *   [3:2]   TXM        TX mode (0=dis, 1=data, 2=zeros, 3=ones)
 *   [5:4]   TXDMA      TX DMA channel select
 *   [6]     RX_EN      RX enable
 *   [8:7]   RXDMA      RX DMA channel select
 *   [9]     CLOSE_EN   De-assert CS after this descriptor
 *   [11:10] QUNIT      Unit size (0=bits, 1=1B, 2=4B, 3=16B)
 *   [15:12] ND         Next descriptor index
 *   [16]    LD         Last descriptor flag
 *   [31:17] NQUNITS    Number of units
 * -------------------------------------------------------------------------- */

static uint32_t build_cmd_descr(uint8_t ifm, uint8_t cmd_len, uint8_t next_idx)
{
	uint32_t d = 0;

	d |= XEC_QSPI_CR_IFM_SET(ifm);
	d |= XEC_QSPI_CR_TXM_SET(XEC_QSPI_CR_TXM_DATA);
	d |= XEC_QSPI_CR_QUNIT_SET(XEC_QSPI_CR_QUNIT_1B);
	d |= XEC_QSPI_CR_NQUNITS_SET(cmd_len);
	d |= XEC_QSPI_DR_ND_SET(next_idx);

	return d;
}

static uint32_t build_addr_descr(uint8_t ifm, uint8_t addr_len, uint8_t next_idx)
{
	uint32_t d = 0;

	d |= XEC_QSPI_CR_IFM_SET(ifm);
	d |= XEC_QSPI_CR_TXM_SET(XEC_QSPI_CR_TXM_DATA);
	d |= XEC_QSPI_CR_QUNIT_SET(XEC_QSPI_CR_QUNIT_1B);
	d |= XEC_QSPI_CR_NQUNITS_SET(addr_len);
	d |= XEC_QSPI_DR_ND_SET(next_idx);

	return d;
}

static uint32_t build_dummy_descr(uint16_t dummy_cycles, uint8_t next_idx)
{
	uint32_t d = 0;

	/* IFM = single, TX disabled, RX disabled */
	d |= XEC_QSPI_CR_IFM_SET(XEC_QSPI_CR_IFM_FD);
	d |= XEC_QSPI_CR_TXM_SET(XEC_QSPI_CR_TXM_DIS);
	/* RX_EN = 0 (default) */
	d |= XEC_QSPI_CR_QUNIT_SET(XEC_QSPI_CR_QUNIT_BITS);
	d |= XEC_QSPI_CR_NQUNITS_SET(dummy_cycles);
	/* CLOSE_EN = 0 (must remain 0 per spec) */
	d |= XEC_QSPI_DR_ND_SET(next_idx);

	return d;
}

/*
 * Build one or more chained data descriptors for PIO mode.
 *
 * The NQUNITS field is 15 bits (max 32767). When num_bytes exceeds what a
 * single descriptor can represent at the chosen unit size, multiple descriptors
 * are chained. Each non-final descriptor links to the next via the ND field;
 * the final descriptor is marked with LD (last descriptor).
 *
 * descr[]   - output array, caller must ensure enough room (up to
 *             QSPI_MAX_DESCR - first_idx entries)
 * first_idx - starting descriptor index (for ND chaining)
 *
 * Returns the number of data descriptors written.
 */
static uint8_t build_data_descr_pio(uint32_t *descr, uint8_t first_idx,
				    uint8_t ifm, enum mspi_xfer_direction dir,
				    uint32_t num_bytes, bool is_last)
{
	uint32_t remaining = num_bytes;
	uint8_t count = 0;
	uint8_t didx = first_idx;

	while (remaining > 0 && didx < QSPI_MAX_DESCR) {
		uint32_t d = 0;
		uint32_t qunit;
		uint32_t nunits;
		uint32_t chunk;

		d |= XEC_QSPI_CR_IFM_SET(ifm);

		if (dir == MSPI_TX) {
			d |= XEC_QSPI_CR_TXM_SET(XEC_QSPI_CR_TXM_DATA);
		} else {
			d |= XEC_QSPI_CR_TXM_SET(XEC_QSPI_CR_TXM_DIS);
			d |= BIT(XEC_QSPI_CR_RX_EN_POS);
		}

		/* Pick the largest unit size that divides the remaining
		 * bytes and fits in the 15-bit NQUNITS field.
		 */
		if ((remaining % 16u) == 0u &&
		    (remaining / 16u) <= QSPI_NQUNITS_MAX) {
			qunit = XEC_QSPI_CR_QUNIT_16B;
			nunits = remaining / 16u;
			chunk = remaining;
		} else if ((remaining % 4u) == 0u &&
			   (remaining / 4u) <= QSPI_NQUNITS_MAX) {
			qunit = XEC_QSPI_CR_QUNIT_4B;
			nunits = remaining / 4u;
			chunk = remaining;
		} else if (remaining <= QSPI_NQUNITS_MAX) {
			qunit = XEC_QSPI_CR_QUNIT_1B;
			nunits = remaining;
			chunk = remaining;
		} else {
			/* remaining > 32767 and not evenly divisible by 4 or 16.
			 * Transfer the largest 1-byte chunk that fits.
			 */
			qunit = XEC_QSPI_CR_QUNIT_1B;
			nunits = QSPI_NQUNITS_MAX;
			chunk = QSPI_NQUNITS_MAX;
		}

		d |= XEC_QSPI_CR_QUNIT_SET(qunit);
		d |= XEC_QSPI_CR_NQUNITS_SET(nunits);

		remaining -= chunk;

		if (remaining == 0) {
			/* Final data descriptor */
			if (is_last) {
				d |= BIT(XEC_QSPI_CR_CLOSE_EN_POS);
			}
			d |= BIT(XEC_QSPI_DR_LD_POS);
		} else {
			/* Chain to next descriptor */
			d |= XEC_QSPI_DR_ND_SET(didx + 1u);
		}

		descr[count] = d;
		count++;
		didx++;
	}

	return count;
}

#ifdef CONFIG_MSPI_XEC_QSPI_LDMA
static uint32_t build_data_descr_ldma(uint8_t ifm, enum mspi_xfer_direction dir,
				      bool is_last)
{
	uint32_t d = 0;

	d |= XEC_QSPI_CR_IFM_SET(ifm);

	if (dir == MSPI_TX) {
		d |= XEC_QSPI_CR_TXM_SET(XEC_QSPI_CR_TXM_DATA);
		/* TXDMA = TLDCH2 (value 3) -> TX LDMA channel 2 (hw ch 5) */
		d |= XEC_QSPI_CR_TXDMA_SET(XEC_QSPI_CR_TXDMA_TLDCH2);
	} else {
		d |= XEC_QSPI_CR_TXM_SET(XEC_QSPI_CR_TXM_DIS);
		d |= BIT(XEC_QSPI_CR_RX_EN_POS);
		/* RXDMA = RLDCH0 (value 1) -> RX LDMA channel 0 (hw ch 0) */
		d |= XEC_QSPI_CR_RXDMA_SET(XEC_QSPI_CR_RXDMA_RLDCH0);
	}

	/* LDMA handles length via size override, set NQUNITS=0 */
	d |= XEC_QSPI_CR_QUNIT_SET(XEC_QSPI_CR_QUNIT_1B);
	d |= XEC_QSPI_CR_NQUNITS_SET(0);

	if (is_last) {
		d |= BIT(XEC_QSPI_CR_CLOSE_EN_POS);
		d |= BIT(XEC_QSPI_DR_LD_POS);
	}

	return d;
}
#endif /* CONFIG_MSPI_XEC_QSPI_LDMA */

/* --------------------------------------------------------------------------
 * Chip select management
 * -------------------------------------------------------------------------- */

/* Return true if this device uses a hardware chip select */
static bool qspi_cs_is_hw(const struct mspi_xec_config *config,
			   const struct mspi_dev_cfg *cfg)
{
	return cfg->ce_num < config->chip_select_count;
}

static void qspi_cs_assert(const struct device *dev,
			    const struct mspi_dev_id *dev_id,
			    const struct mspi_dev_cfg *cfg)
{
	const struct mspi_xec_config *config = dev->config;
	uint32_t base = config->base;

	if (qspi_cs_is_hw(config, cfg)) {
		/* Hardware CS: set CS_SEL field in Mode register.
		 * The QMSPI controller will assert/deassert this CS
		 * automatically based on descriptor CLOSE_EN.
		 */
		uint32_t mode = qspi_read32(base, XEC_QSPI_MODE_OFS);

		mode &= ~XEC_QSPI_MODE_CS_SEL_MSK;
		mode |= XEC_QSPI_MODE_CS_SEL_SET(cfg->ce_num);
		qspi_write32(base, XEC_QSPI_MODE_OFS, mode);
	} else if (dev_id->ce.port != NULL) {
		/* GPIO CS: assert via GPIO.
		 * Disable CLOSE_EN-driven HW CS assertion by ensuring
		 * no descriptors set CLOSE_EN for this transfer — the
		 * caller manages is_last accordingly. The HW CS lines
		 * remain inactive because the QMSPI only drives CS
		 * when CLOSE_EN triggers de-assertion at end of the
		 * last descriptor. For GPIO CS devices we still set
		 * CLOSE_EN to signal transfer end to the controller,
		 * but the HW CS is irrelevant since it's not wired
		 * to this device.
		 */
		gpio_pin_set_dt(&dev_id->ce, 1);
	}
}

static void qspi_cs_deassert(const struct device *dev,
			      const struct mspi_dev_id *dev_id,
			      const struct mspi_dev_cfg *cfg)
{
	const struct mspi_xec_config *config = dev->config;

	if (!qspi_cs_is_hw(config, cfg) && dev_id->ce.port != NULL) {
		/* GPIO CS: deassert via GPIO */
		gpio_pin_set_dt(&dev_id->ce, 0);
	}
	/* Hardware CS is auto-deasserted by CLOSE_EN in last descriptor */
}

/* --------------------------------------------------------------------------
 * Prepare command + address buffer
 *
 * Returns total bytes written to ca_buf. Address is stored big-endian (MSB
 * first) using only the addr_length least significant bytes.
 * -------------------------------------------------------------------------- */

static uint8_t qspi_prepare_ca_buf(uint8_t *ca_buf, const struct mspi_xfer *xfer,
				    const struct mspi_xfer_packet *pkt)
{
	uint8_t pos = 0;
	uint8_t i;

	/* Command byte(s) */
	for (i = 0; i < xfer->cmd_length; i++) {
		ca_buf[pos++] = (uint8_t)(pkt->cmd >> (8u * (xfer->cmd_length - 1u - i)));
	}

	/* Address bytes, big-endian */
	for (i = 0; i < xfer->addr_length; i++) {
		ca_buf[pos++] = (uint8_t)(pkt->address >>
					  (8u * (xfer->addr_length - 1u - i)));
	}

	return pos;
}

/* --------------------------------------------------------------------------
 * ISR - Interrupt Service Routine
 *
 * Handles: errors, PIO FIFO refill/drain, transfer completion.
 * No polling anywhere — all transfer progress is driven by interrupts.
 * -------------------------------------------------------------------------- */

static void mspi_xec_qspi_isr(const struct device *dev)
{
	const struct mspi_xec_config *config = dev->config;
	struct mspi_xec_data *data = dev->data;
	uint32_t base = config->base;
	uint32_t status;
	uint32_t ier;

	status = qspi_read32(base, XEC_QSPI_SR_OFS);

	/* Clear W1C status bits */
	qspi_write32(base, XEC_QSPI_SR_OFS, status);

	/* Error check */
	if (status & QSPI_ERR_MASK) {
		LOG_ERR("QSPI error: status=0x%08x", status);
		qspi_write32(base, XEC_QSPI_IER_OFS, 0);
		data->xfer_err = -EIO;
		k_sem_give(&data->xfer_sync);
		return;
	}

	/* PIO TX FIFO refill: feed more data bytes if TX FIFO needs data */
	if (data->tx_remaining > 0 &&
	    (status & (BIT(XEC_QSPI_SR_TXB_EMPTY_POS) |
		       BIT(XEC_QSPI_SR_TXB_REQ_POS)))) {
		uint32_t bcnt = qspi_read32(base, XEC_QSPI_BCNT_SR_OFS);
		uint32_t tx_used = XEC_QSPI_BCNT_SR_TXB_GET(bcnt);
		uint32_t space = QSPI_FIFO_DEPTH - tx_used;
		uint32_t to_write = MIN(data->tx_remaining, space);

		for (uint32_t i = 0; i < to_write; i++) {
			qspi_write8(base, XEC_QSPI_TXB_OFS, *data->tx_buf);
			data->tx_buf++;
		}
		data->tx_remaining -= to_write;

		if (data->tx_remaining == 0) {
			/* All TX data queued, disable TX FIFO interrupts */
			ier = qspi_read32(base, XEC_QSPI_IER_OFS);
			ier &= ~(BIT(XEC_QSPI_IER_TXB_EMPTY_POS) |
				  BIT(XEC_QSPI_IER_TXB_REQ_POS));
			qspi_write32(base, XEC_QSPI_IER_OFS, ier);
		}
	}

	/* PIO RX FIFO drain: pull received bytes */
	if (data->rx_remaining > 0 &&
	    (status & BIT(XEC_QSPI_SR_RXB_REQ_POS))) {
		uint32_t bcnt = qspi_read32(base, XEC_QSPI_BCNT_SR_OFS);
		uint32_t rx_avail = XEC_QSPI_BCNT_SR_RXB_GET(bcnt);
		uint32_t to_read = MIN(data->rx_remaining, rx_avail);

		for (uint32_t i = 0; i < to_read; i++) {
			*data->rx_buf = qspi_read8(base, XEC_QSPI_RXB_OFS);
			data->rx_buf++;
		}
		data->rx_remaining -= to_read;

		if (data->rx_remaining == 0) {
			ier = qspi_read32(base, XEC_QSPI_IER_OFS);
			ier &= ~BIT(XEC_QSPI_IER_RXB_REQ_POS);
			qspi_write32(base, XEC_QSPI_IER_OFS, ier);
		}
	}

	/* Transfer complete (XFR_DONE or DMA_DONE) */
	if (status & (BIT(XEC_QSPI_SR_XFR_DONE_POS) |
		      BIT(XEC_QSPI_SR_DMA_DONE_POS))) {
		/* Disable all interrupts */
		qspi_write32(base, XEC_QSPI_IER_OFS, 0);

		/* For PIO RX: drain any remaining bytes left in FIFO */
		if (data->rx_remaining > 0) {
			uint32_t bcnt = qspi_read32(base, XEC_QSPI_BCNT_SR_OFS);
			uint32_t rx_avail = XEC_QSPI_BCNT_SR_RXB_GET(bcnt);
			uint32_t to_read = MIN(data->rx_remaining, rx_avail);

			for (uint32_t i = 0; i < to_read; i++) {
				*data->rx_buf = qspi_read8(base, XEC_QSPI_RXB_OFS);
				data->rx_buf++;
			}
			data->rx_remaining -= to_read;
		}

		data->xfer_err = 0;
		k_sem_give(&data->xfer_sync);
	}
}

/* --------------------------------------------------------------------------
 * Start transfer and wait for ISR completion
 * -------------------------------------------------------------------------- */

enum qspi_xfer_type {
	QSPI_XFER_DMA,
	QSPI_XFER_PIO_TX,
	QSPI_XFER_PIO_RX,
};

static int qspi_start_and_wait(const struct device *dev, uint32_t timeout_ms,
				enum qspi_xfer_type xtype)
{
	const struct mspi_xec_config *config = dev->config;
	struct mspi_xec_data *data = dev->data;
	uint32_t base = config->base;
	uint32_t ier_val;
	int ret;

	data->xfer_err = -ETIMEDOUT;

	/* Clear all status bits */
	qspi_write32(base, XEC_QSPI_SR_OFS, QSPI_CLR_STATUS_MASK);

	/* Clear FIFOs */
	qspi_write32(base, XEC_QSPI_EXE_OFS, BIT(XEC_QSPI_EXE_CLR_FIFOS_POS));

	/* Enable interrupts based on transfer type */
	ier_val = QSPI_IER_ERRORS | BIT(XEC_QSPI_IER_XFR_DONE_POS);

	switch (xtype) {
	case QSPI_XFER_DMA:
		ier_val |= BIT(XEC_QSPI_IER_DMA_DONE_POS);
		break;
	case QSPI_XFER_PIO_TX:
		ier_val |= BIT(XEC_QSPI_IER_TXB_EMPTY_POS) |
			   BIT(XEC_QSPI_IER_TXB_REQ_POS);
		break;
	case QSPI_XFER_PIO_RX:
		ier_val |= BIT(XEC_QSPI_IER_RXB_REQ_POS);
		break;
	}

	/* Reset sync semaphore (drain any previous gives) */
	k_sem_reset(&data->xfer_sync);

	/* Enable interrupts */
	qspi_write32(base, XEC_QSPI_IER_OFS, ier_val);

	/* Start transfer */
	qspi_write32(base, XEC_QSPI_EXE_OFS, BIT(XEC_QSPI_EXE_START_POS));

	/* Wait for ISR to signal completion */
	ret = k_sem_take(&data->xfer_sync,
			 K_MSEC(timeout_ms + CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE));
	if (ret < 0) {
		/* Timeout: force stop */
		qspi_write32(base, XEC_QSPI_EXE_OFS, BIT(XEC_QSPI_EXE_STOP_POS));
		qspi_write32(base, XEC_QSPI_IER_OFS, 0);
		LOG_ERR("QSPI transfer timeout");
		return -ETIMEDOUT;
	}

	return data->xfer_err;
}

/* --------------------------------------------------------------------------
 * Write descriptors and set up descriptor mode in Control register
 *
 * Programs up to 4 descriptors and configures the Control register for
 * descriptor-mode execution starting at descriptor 0.
 * -------------------------------------------------------------------------- */

static void qspi_write_descriptors(uint32_t base, const uint32_t *descr,
				    uint8_t num_descr)
{
	for (uint8_t i = 0; i < num_descr; i++) {
		qspi_write32(base, XEC_QSPI_DESCR_OFS(i), descr[i]);
	}

	/* Configure Control register: descriptor mode, start at descriptor 0 */
	uint32_t cr = BIT(XEC_QSPI_CR_DESCR_EN_POS) |
		      XEC_QSPI_CR_FD_SET(0); /* first descriptor = 0 */
	qspi_write32(base, XEC_QSPI_CR_OFS, cr);
}

/* --------------------------------------------------------------------------
 * PIO transfer for a single packet
 * -------------------------------------------------------------------------- */

static int qspi_pio_xfer_packet(const struct device *dev,
				 const struct mspi_xfer *xfer,
				 const struct mspi_xfer_packet *pkt,
				 uint8_t cmd_ifm, uint8_t addr_ifm,
				 uint8_t data_ifm, bool is_last)
{
	const struct mspi_xec_config *config = dev->config;
	struct mspi_xec_data *data = dev->data;
	uint32_t base = config->base;
	uint32_t descr[QSPI_MAX_DESCR];
	uint8_t ca_buf[8] __aligned(4);
	uint8_t ca_len;
	uint8_t num_descr = 0;
	uint8_t next_idx;
	uint16_t dummy_cycles;
	enum qspi_xfer_type xtype;

	/* Disable LDMA */
	uint32_t mode = qspi_read32(base, XEC_QSPI_MODE_OFS);

	mode &= ~(BIT(XEC_QSPI_MODE_LD_RX_EN_POS) |
		   BIT(XEC_QSPI_MODE_LD_TX_EN_POS));
	qspi_write32(base, XEC_QSPI_MODE_OFS, mode);
	qspi_write32(base, XEC_QSPI_LDMA_RX_EN_OFS, 0);
	qspi_write32(base, XEC_QSPI_LDMA_TX_EN_OFS, 0);

	/* Prepare command + address buffer */
	ca_len = qspi_prepare_ca_buf(ca_buf, xfer, pkt);

	/* Determine dummy cycles */
	dummy_cycles = (pkt->dir == MSPI_TX) ? xfer->tx_dummy : xfer->rx_dummy;

	/* Calculate descriptor chain: figure out which phases are present */
	bool has_cmd = (xfer->cmd_length > 0);
	bool has_addr = (xfer->addr_length > 0);
	bool has_dummy = (dummy_cycles > 0);
	bool has_data = (pkt->num_bytes > 0);

	/* Determine next-descriptor indices based on which phases exist.
	 * We build the chain backward to know each next_idx.
	 */
	uint8_t data_idx = 0, dummy_idx = 0, addr_idx = 0, cmd_idx = 0;
	uint8_t idx = 0;

	if (has_cmd) {
		cmd_idx = idx++;
	}
	if (has_addr) {
		addr_idx = idx++;
	}
	if (has_dummy) {
		dummy_idx = idx++;
	}
	if (has_data) {
		data_idx = idx++;
	}

	/* Build descriptors */
	num_descr = 0;

	if (has_cmd) {
		next_idx = has_addr ? addr_idx : (has_dummy ? dummy_idx :
			   (has_data ? data_idx : cmd_idx));
		descr[cmd_idx] = build_cmd_descr(cmd_ifm, xfer->cmd_length,
						  next_idx);
		/* If cmd is the only/last phase */
		if (!has_addr && !has_dummy && !has_data) {
			descr[cmd_idx] |= BIT(XEC_QSPI_DR_LD_POS);
			if (is_last) {
				descr[cmd_idx] |= BIT(XEC_QSPI_CR_CLOSE_EN_POS);
			}
		}
		num_descr++;
	}

	if (has_addr) {
		next_idx = has_dummy ? dummy_idx : (has_data ? data_idx : addr_idx);
		descr[addr_idx] = build_addr_descr(addr_ifm, xfer->addr_length,
						    next_idx);
		if (!has_dummy && !has_data) {
			descr[addr_idx] |= BIT(XEC_QSPI_DR_LD_POS);
			if (is_last) {
				descr[addr_idx] |= BIT(XEC_QSPI_CR_CLOSE_EN_POS);
			}
		}
		num_descr++;
	}

	if (has_dummy) {
		next_idx = has_data ? data_idx : dummy_idx;
		descr[dummy_idx] = build_dummy_descr(dummy_cycles, next_idx);
		if (!has_data) {
			descr[dummy_idx] |= BIT(XEC_QSPI_DR_LD_POS);
			if (is_last) {
				descr[dummy_idx] |= BIT(XEC_QSPI_CR_CLOSE_EN_POS);
			}
		}
		num_descr++;
	}

	if (has_data) {
		uint8_t data_count = build_data_descr_pio(&descr[data_idx], data_idx,
							  data_ifm, pkt->dir,
							  pkt->num_bytes, is_last);
		if (data_count == 0) {
			LOG_ERR("PIO data descriptor build failed (too many descriptors)");
			k_sem_give(&data->bus_lock);
			return -ENOMEM;
		}
		num_descr += data_count;
	}

	/* Write descriptors to hardware */
	qspi_write_descriptors(base, descr, num_descr);

	/* Set up ISR state for PIO data transfer */
	data->tx_buf = NULL;
	data->tx_remaining = 0;
	data->rx_buf = NULL;
	data->rx_remaining = 0;

	if (has_data && pkt->dir == MSPI_TX) {
		data->tx_buf = pkt->data_buf;
		data->tx_remaining = pkt->num_bytes;
		xtype = QSPI_XFER_PIO_TX;
	} else if (has_data && pkt->dir == MSPI_RX) {
		data->rx_buf = pkt->data_buf;
		data->rx_remaining = pkt->num_bytes;
		xtype = QSPI_XFER_PIO_RX;
	} else {
		/* Command-only packet, no data phase */
		xtype = QSPI_XFER_PIO_TX;
	}

	/* Pre-fill TX FIFO with command + address bytes.
	 * This is a one-time prime before the transfer starts, not polling.
	 */
	for (uint8_t i = 0; i < ca_len && i < QSPI_FIFO_DEPTH; i++) {
		qspi_write8(base, XEC_QSPI_TXB_OFS, ca_buf[i]);
	}

	/* If TX data phase, also pre-fill remaining FIFO space */
	if (has_data && pkt->dir == MSPI_TX) {
		uint32_t space = QSPI_FIFO_DEPTH - ca_len;
		uint32_t prefill = MIN(data->tx_remaining, space);

		for (uint32_t i = 0; i < prefill; i++) {
			qspi_write8(base, XEC_QSPI_TXB_OFS, *data->tx_buf);
			data->tx_buf++;
		}
		data->tx_remaining -= prefill;
	}

	return qspi_start_and_wait(dev, xfer->timeout, xtype);
}

/* --------------------------------------------------------------------------
 * LDMA transfer for a single packet
 * -------------------------------------------------------------------------- */

#ifdef CONFIG_MSPI_XEC_QSPI_LDMA

/* Compute LDMA channel control register value based on buffer alignment */
static uint32_t qspi_ldma_cr_val(const void *buf, uint32_t len)
{
	uint32_t cr = BIT(XEC_QSPI_LDMA_CHX_CR_EN_POS) |
		      BIT(XEC_QSPI_LDMA_CHX_CR_OVRL_POS) |
		      BIT(XEC_QSPI_LDMA_CHX_CR_INCRA_POS);

	if (((uintptr_t)buf % 4u) == 0u && (len % 4u) == 0u) {
		cr |= XEC_QSPI_LDMA_CHX_CR_SZ_SET(XEC_QSPI_LDMA_CHX_CR_SZ_4B);
	} else if (((uintptr_t)buf % 2u) == 0u && (len % 2u) == 0u) {
		cr |= XEC_QSPI_LDMA_CHX_CR_SZ_SET(XEC_QSPI_LDMA_CHX_CR_SZ_2B);
	} else {
		cr |= XEC_QSPI_LDMA_CHX_CR_SZ_SET(XEC_QSPI_LDMA_CHX_CR_SZ_1B);
	}

	return cr;
}

static void qspi_ldma_program_channel(uint32_t base, uint8_t hw_ch,
				       const void *buf, uint32_t len)
{
	uint32_t cr = qspi_ldma_cr_val(buf, len);

	qspi_write32(base, XEC_QSPI_LDMA_CHX_CR_OFS(hw_ch), cr);
	qspi_write32(base, XEC_QSPI_LDMA_CHX_SA_OFS(hw_ch), (uint32_t)(uintptr_t)buf);
	qspi_write32(base, XEC_QSPI_LDMA_CHX_LR_OFS(hw_ch), len);
}

static int qspi_ldma_xfer_packet(const struct device *dev,
				  const struct mspi_xfer *xfer,
				  const struct mspi_xfer_packet *pkt,
				  uint8_t cmd_ifm, uint8_t addr_ifm,
				  uint8_t data_ifm, bool is_last)
{
	const struct mspi_xec_config *config = dev->config;
	struct mspi_xec_data *data = dev->data;
	uint32_t base = config->base;
	uint32_t descr[4];
	uint8_t ca_buf[8] __aligned(4);
	uint8_t ca_len;
	uint8_t num_descr = 0;
	uint8_t next_idx;
	uint16_t dummy_cycles;
	uint32_t ldma_tx_en = 0;
	uint32_t ldma_rx_en = 0;

	/* Prepare command + address buffer */
	ca_len = qspi_prepare_ca_buf(ca_buf, xfer, pkt);

	/* Determine dummy cycles */
	dummy_cycles = (pkt->dir == MSPI_TX) ? xfer->tx_dummy : xfer->rx_dummy;

	bool has_cmd = (xfer->cmd_length > 0);
	bool has_addr = (xfer->addr_length > 0);
	bool has_dummy = (dummy_cycles > 0);
	bool has_data = (pkt->num_bytes > 0);

	/* Calculate descriptor indices */
	uint8_t data_idx = 0, dummy_idx = 0, addr_idx = 0, cmd_idx = 0;
	uint8_t idx = 0;

	if (has_cmd) {
		cmd_idx = idx++;
	}
	if (has_addr) {
		addr_idx = idx++;
	}
	if (has_dummy) {
		dummy_idx = idx++;
	}
	if (has_data) {
		data_idx = idx++;
	}

	/* Build descriptors with LDMA channel assignments */
	num_descr = 0;

	if (has_cmd) {
		next_idx = has_addr ? addr_idx : (has_dummy ? dummy_idx :
			   (has_data ? data_idx : cmd_idx));
		descr[cmd_idx] = build_cmd_descr(cmd_ifm, xfer->cmd_length,
						  next_idx);
		/* Assign TX LDMA channel 0 (hw ch 3) for command */
		descr[cmd_idx] &= ~XEC_QSPI_CR_TXDMA_MSK;
		descr[cmd_idx] |= XEC_QSPI_CR_TXDMA_SET(XEC_QSPI_CR_TXDMA_TLDCH0);
		ldma_tx_en |= BIT(cmd_idx);

		if (!has_addr && !has_dummy && !has_data) {
			descr[cmd_idx] |= BIT(XEC_QSPI_DR_LD_POS);
			if (is_last) {
				descr[cmd_idx] |= BIT(XEC_QSPI_CR_CLOSE_EN_POS);
			}
		}
		num_descr++;
	}

	if (has_addr) {
		next_idx = has_dummy ? dummy_idx : (has_data ? data_idx : addr_idx);
		descr[addr_idx] = build_addr_descr(addr_ifm, xfer->addr_length,
						    next_idx);
		/* Assign TX LDMA channel 1 (hw ch 4) for address */
		descr[addr_idx] &= ~XEC_QSPI_CR_TXDMA_MSK;
		descr[addr_idx] |= XEC_QSPI_CR_TXDMA_SET(XEC_QSPI_CR_TXDMA_TLDCH1);
		ldma_tx_en |= BIT(addr_idx);

		if (!has_dummy && !has_data) {
			descr[addr_idx] |= BIT(XEC_QSPI_DR_LD_POS);
			if (is_last) {
				descr[addr_idx] |= BIT(XEC_QSPI_CR_CLOSE_EN_POS);
			}
		}
		num_descr++;
	}

	if (has_dummy) {
		next_idx = has_data ? data_idx : dummy_idx;
		descr[dummy_idx] = build_dummy_descr(dummy_cycles, next_idx);
		/* No DMA for dummy phase */
		if (!has_data) {
			descr[dummy_idx] |= BIT(XEC_QSPI_DR_LD_POS);
			if (is_last) {
				descr[dummy_idx] |= BIT(XEC_QSPI_CR_CLOSE_EN_POS);
			}
		}
		num_descr++;
	}

	if (has_data) {
		descr[data_idx] = build_data_descr_ldma(data_ifm, pkt->dir,
							 is_last);
		if (pkt->dir == MSPI_TX) {
			ldma_tx_en |= BIT(data_idx);
		} else {
			ldma_rx_en |= BIT(data_idx);
		}
		num_descr++;
	}

	/* Enable LDMA in Mode register */
	uint32_t mode = qspi_read32(base, XEC_QSPI_MODE_OFS);

	if (ldma_rx_en) {
		mode |= BIT(XEC_QSPI_MODE_LD_RX_EN_POS);
	} else {
		mode &= ~BIT(XEC_QSPI_MODE_LD_RX_EN_POS);
	}
	if (ldma_tx_en) {
		mode |= BIT(XEC_QSPI_MODE_LD_TX_EN_POS);
	} else {
		mode &= ~BIT(XEC_QSPI_MODE_LD_TX_EN_POS);
	}
	qspi_write32(base, XEC_QSPI_MODE_OFS, mode);

	/* Set LDMA enable bitmaps */
	qspi_write32(base, XEC_QSPI_LDMA_TX_EN_OFS, ldma_tx_en);
	qspi_write32(base, XEC_QSPI_LDMA_RX_EN_OFS, ldma_rx_en);

	/* Program LDMA channels */
	if (has_cmd) {
		/* TX Ch0 (hw ch 3) = command bytes */
		qspi_ldma_program_channel(base, XEC_QSPI_LDMA_TX_CH0,
					  &ca_buf[0], xfer->cmd_length);
	}

	if (has_addr) {
		/* TX Ch1 (hw ch 4) = address bytes */
		qspi_ldma_program_channel(base, XEC_QSPI_LDMA_TX_CH1,
					  &ca_buf[xfer->cmd_length],
					  xfer->addr_length);
	}

	if (has_data) {
		if (pkt->dir == MSPI_TX) {
			/* TX Ch2 (hw ch 5) = TX data */
			qspi_ldma_program_channel(base, XEC_QSPI_LDMA_TX_CH2,
						  pkt->data_buf,
						  pkt->num_bytes);
		} else {
			/* RX Ch0 (hw ch 0) = RX data */
			qspi_ldma_program_channel(base, XEC_QSPI_LDMA_RX_CH0,
						  pkt->data_buf,
						  pkt->num_bytes);
		}
	}

	/* Write descriptors to hardware */
	qspi_write_descriptors(base, descr, num_descr);

	/* Clear PIO state — not used in LDMA mode */
	data->tx_buf = NULL;
	data->tx_remaining = 0;
	data->rx_buf = NULL;
	data->rx_remaining = 0;

	return qspi_start_and_wait(dev, xfer->timeout, QSPI_XFER_DMA);
}

#endif /* CONFIG_MSPI_XEC_QSPI_LDMA */

/* --------------------------------------------------------------------------
 * Callback helpers
 * -------------------------------------------------------------------------- */

static void call_callback(struct mspi_xec_data *data, const struct device *dev,
			   enum mspi_bus_event evt_type,
			   const struct mspi_xfer_packet *pkt,
			   uint32_t pkt_idx, int status)
{
	if (data->cbs[evt_type] == NULL || data->cb_ctxs[evt_type] == NULL) {
		return;
	}

	struct mspi_callback_context *ctx = data->cb_ctxs[evt_type];

	ctx->mspi_evt.evt_type = evt_type;
	ctx->mspi_evt.evt_data.controller = dev;
	ctx->mspi_evt.evt_data.dev_id = data->active_dev_id;
	ctx->mspi_evt.evt_data.packet = pkt;
	ctx->mspi_evt.evt_data.packet_idx = pkt_idx;
	ctx->mspi_evt.evt_data.status = status;

	data->cbs[evt_type](ctx);
}

/* --------------------------------------------------------------------------
 * MSPI API: config
 * -------------------------------------------------------------------------- */

static int mspi_xec_config(const struct mspi_dt_spec *spec)
{
	const struct device *dev = spec->bus;
	const struct mspi_xec_config *config = dev->config;
	struct mspi_xec_data *data = dev->data;
	uint32_t base = config->base;
	int ret;

	/* Validate controller configuration */
	if (spec->config.op_mode != MSPI_OP_MODE_CONTROLLER) {
		LOG_ERR("Only controller mode supported");
		return -ENOTSUP;
	}

	if (spec->config.duplex != MSPI_HALF_DUPLEX) {
		LOG_ERR("Only half-duplex supported");
		return -ENOTSUP;
	}

	if (spec->config.dqs_support) {
		LOG_ERR("DQS not supported");
		return -ENOTSUP;
	}

	/* Apply pinctrl */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to apply pinctrl: %d", ret);
		return ret;
	}

	/* Soft-reset QMSPI */
	qspi_write32(base, XEC_QSPI_MODE_OFS, BIT(XEC_QSPI_MODE_SRST_POS));

	/* Wait for reset to self-clear (a few cycles) */
	k_busy_wait(1);

	/* Activate QMSPI */
	qspi_write32(base, XEC_QSPI_MODE_OFS, BIT(XEC_QSPI_MODE_ACTV_POS));

	/* Set default CS timing */
	uint32_t cstm = XEC_QSPI_CSTM_CSA_FCK_SET(XEC_QSPI_CSTM_CSA_FCK_DFLT) |
			XEC_QSPI_CSTM_LCK_CSD_SET(XEC_QSPI_CSTM_LCK_CSD_DFLT) |
			XEC_QSPI_CSTM_LDHD_SET(XEC_QSPI_CSTM_LDHD_DFLT) |
			XEC_QSPI_CSTM_CSD_CSA_SET(XEC_QSPI_CSTM_CSD_CSA_DFLT);

	qspi_write32(base, XEC_QSPI_CSTM_OFS, cstm);

	/* Disable all interrupts initially */
	qspi_write32(base, XEC_QSPI_IER_OFS, 0);

	/* Connect and enable IRQ */
	config->irq_config_func();

	/* Configure CE GPIOs as output inactive */
	for (uint8_t i = 0; i < config->num_ce_gpios; i++) {
		if (config->ce_gpios[i].port != NULL) {
			ret = gpio_pin_configure_dt(&config->ce_gpios[i],
						    GPIO_OUTPUT_INACTIVE);
			if (ret < 0) {
				LOG_ERR("Failed to configure CE GPIO %d: %d",
					i, ret);
				return ret;
			}
		}
	}

	/* Clear any stale active device */
	data->active_dev_id = NULL;

	return 0;
}

/* --------------------------------------------------------------------------
 * MSPI API: dev_config
 * -------------------------------------------------------------------------- */

static int mspi_xec_dev_config(const struct device *dev,
			       const struct mspi_dev_id *dev_id,
			       const enum mspi_dev_cfg_mask param_mask,
			       const struct mspi_dev_cfg *cfg)
{
	const struct mspi_xec_config *config = dev->config;
	struct mspi_xec_data *data = dev->data;
	uint32_t base = config->base;
	uint32_t mode;
	uint32_t actual_freq;
	uint16_t clk_div;

	if (k_sem_take(&data->bus_lock, K_MSEC(CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE)) < 0) {
		return -EBUSY;
	}

	/* Validate unsupported features */
	if (cfg->data_rate != MSPI_DATA_RATE_SINGLE) {
		LOG_ERR("Only single data rate supported");
		k_sem_give(&data->bus_lock);
		return -ENOTSUP;
	}

	if (cfg->dqs_enable) {
		LOG_ERR("DQS not supported");
		k_sem_give(&data->bus_lock);
		return -ENOTSUP;
	}

	if (cfg->endian != MSPI_XFER_LITTLE_ENDIAN) {
		LOG_ERR("Only little-endian supported");
		k_sem_give(&data->bus_lock);
		return -ENOTSUP;
	}

	if (cfg->ce_polarity != MSPI_CE_ACTIVE_LOW) {
		LOG_ERR("Only active-low CE supported");
		k_sem_give(&data->bus_lock);
		return -ENOTSUP;
	}

	/* Validate IO mode is one we support */
	uint8_t tmp_cmd, tmp_addr, tmp_data;

	if (qspi_io_mode_to_ifm(cfg->io_mode, &tmp_cmd, &tmp_addr, &tmp_data) < 0) {
		LOG_ERR("Unsupported IO mode: %d", cfg->io_mode);
		k_sem_give(&data->bus_lock);
		return -ENOTSUP;
	}

	/* Calculate clock divider */
	clk_div = qspi_calc_clk_div(config->clk_freq, cfg->freq, &actual_freq);

	/* Read-modify-write Mode register: preserve ACTV, CS_SEL, LDMA enables */
	mode = qspi_read32(base, XEC_QSPI_MODE_OFS);
	mode &= ~(XEC_QSPI_MODE_CK_DIV_MSK | XEC_QSPI_MODE_CP_MSK);
	mode |= XEC_QSPI_MODE_CK_DIV_SET(clk_div);
	mode |= qspi_cpp_to_mode_bits(cfg->cpp, actual_freq);
	qspi_write32(base, XEC_QSPI_MODE_OFS, mode);

	/* Cache active device configuration */
	data->active_dev_cfg = *cfg;
	data->active_dev_cfg.freq = actual_freq;
	data->active_dev_id = dev_id;

	k_sem_give(&data->bus_lock);

	LOG_DBG("dev_config: freq=%u (req=%u) io_mode=%d cpp=%d ce=%d",
		actual_freq, cfg->freq, cfg->io_mode, cfg->cpp, cfg->ce_num);

	return 0;
}

/* --------------------------------------------------------------------------
 * MSPI API: transceive
 * -------------------------------------------------------------------------- */

static int mspi_xec_transceive(const struct device *dev,
			       const struct mspi_dev_id *dev_id,
			       const struct mspi_xfer *xfer)
{
	const struct mspi_xec_config *config = dev->config;
	struct mspi_xec_data *data = dev->data;
	uint8_t cmd_ifm, addr_ifm, data_ifm;
	int ret;

	if (xfer == NULL || xfer->packets == NULL || xfer->num_packet == 0) {
		return -EINVAL;
	}

	if (k_sem_take(&data->bus_lock, K_MSEC(xfer->timeout)) < 0) {
		return -EBUSY;
	}

	/* Verify device matches */
	if (config->sw_multi_periph && dev_id != data->active_dev_id) {
		LOG_ERR("Device ID mismatch");
		k_sem_give(&data->bus_lock);
		return -EINVAL;
	}

	/* Resolve IO mode to per-phase IFM values */
	ret = qspi_io_mode_to_ifm(data->active_dev_cfg.io_mode,
				   &cmd_ifm, &addr_ifm, &data_ifm);
	if (ret < 0) {
		k_sem_give(&data->bus_lock);
		return ret;
	}

	/* Assert chip select */
	qspi_cs_assert(dev, dev_id, &data->active_dev_cfg);

	/* Process each packet */
	for (uint32_t i = 0; i < xfer->num_packet; i++) {
		const struct mspi_xfer_packet *pkt = &xfer->packets[i];
		bool is_last = (i == xfer->num_packet - 1) && !xfer->hold_ce;

#ifdef CONFIG_MSPI_XEC_QSPI_LDMA
		if (xfer->xfer_mode == MSPI_DMA) {
			ret = qspi_ldma_xfer_packet(dev, xfer, pkt,
						    cmd_ifm, addr_ifm,
						    data_ifm, is_last);
		} else {
			ret = qspi_pio_xfer_packet(dev, xfer, pkt,
						   cmd_ifm, addr_ifm,
						   data_ifm, is_last);
		}
#else
		ret = qspi_pio_xfer_packet(dev, xfer, pkt,
					   cmd_ifm, addr_ifm,
					   data_ifm, is_last);
#endif

		if (ret < 0) {
			LOG_ERR("Packet %u failed: %d", i, ret);
			/* Force stop */
			qspi_write32(config->base, XEC_QSPI_EXE_OFS,
				     BIT(XEC_QSPI_EXE_STOP_POS));
			call_callback(data, dev, MSPI_BUS_ERROR, pkt, i, ret);
			break;
		}

		/* Invoke transfer complete callback if requested */
		if (pkt->cb_mask & MSPI_BUS_XFER_COMPLETE_CB) {
			call_callback(data, dev, MSPI_BUS_XFER_COMPLETE,
				      pkt, i, 0);
		}
	}

	/* De-assert chip select */
	if (!xfer->hold_ce) {
		qspi_cs_deassert(dev, dev_id, &data->active_dev_cfg);
	}

	k_sem_give(&data->bus_lock);

	return ret;
}

/* --------------------------------------------------------------------------
 * MSPI API: get_channel_status
 * -------------------------------------------------------------------------- */

static int mspi_xec_get_channel_status(const struct device *dev, uint8_t ch)
{
	const struct mspi_xec_config *config = dev->config;
	uint32_t status = qspi_read32(config->base, XEC_QSPI_SR_OFS);

	ARG_UNUSED(ch);

	if (status & BIT(XEC_QSPI_SR_CS_ASSERTED_POS)) {
		return -EBUSY;
	}

	return 0;
}

/* --------------------------------------------------------------------------
 * MSPI API: register_callback
 * -------------------------------------------------------------------------- */

static int mspi_xec_register_callback(const struct device *dev,
				       const struct mspi_dev_id *dev_id,
				       const enum mspi_bus_event evt_type,
				       mspi_callback_handler_t cb,
				       struct mspi_callback_context *ctx)
{
	struct mspi_xec_data *data = dev->data;

	if (evt_type >= MSPI_BUS_EVENT_MAX) {
		return -EINVAL;
	}

	if (dev_id != data->active_dev_id) {
		return -ESTALE;
	}

	data->cbs[evt_type] = cb;
	data->cb_ctxs[evt_type] = ctx;

	return 0;
}

/* --------------------------------------------------------------------------
 * Driver API table
 * -------------------------------------------------------------------------- */

static DEVICE_API(mspi, mspi_xec_api) = {
	.config             = mspi_xec_config,
	.dev_config         = mspi_xec_dev_config,
	.get_channel_status = mspi_xec_get_channel_status,
	.transceive         = mspi_xec_transceive,
	.register_callback  = mspi_xec_register_callback,
};

/* --------------------------------------------------------------------------
 * Init
 * -------------------------------------------------------------------------- */

static int mspi_xec_init(const struct device *dev)
{
	const struct mspi_xec_config *config = dev->config;
	struct mspi_xec_data *data = dev->data;

	k_sem_init(&data->bus_lock, 1, 1);
	k_sem_init(&data->xfer_sync, 0, 1);

	const struct mspi_dt_spec spec = {
		.bus = dev,
		.config = {
			.op_mode = MSPI_OP_MODE_CONTROLLER,
			.duplex = MSPI_HALF_DUPLEX,
			.dqs_support = false,
			.sw_multi_periph = config->sw_multi_periph,
			.num_ce_gpios = config->num_ce_gpios,
			.ce_group = (struct gpio_dt_spec *)config->ce_gpios,
			.max_freq = config->clk_freq,
		},
	};

	return mspi_xec_config(&spec);
}

/* --------------------------------------------------------------------------
 * Device instantiation
 * -------------------------------------------------------------------------- */

#define MSPI_XEC_IRQ_CONFIG(inst)                                              \
	static void mspi_xec_irq_config_##inst(void)                           \
	{                                                                      \
		IRQ_CONNECT(DT_INST_IRQN(inst),                                \
			    DT_INST_IRQ(inst, priority),                        \
			    mspi_xec_qspi_isr,                                 \
			    DEVICE_DT_INST_GET(inst), 0);                      \
		irq_enable(DT_INST_IRQN(inst));                                \
	}

#define MSPI_XEC_INST(inst)                                                    \
	PINCTRL_DT_INST_DEFINE(inst);                                          \
	MSPI_XEC_IRQ_CONFIG(inst)                                              \
	static struct gpio_dt_spec ce_gpios_##inst[] =                         \
		MSPI_CE_GPIOS_DT_SPEC_INST_GET(inst);                         \
	static struct mspi_xec_data mspi_xec_data_##inst;                      \
	static const struct mspi_xec_config mspi_xec_config_##inst = {         \
		.base = DT_INST_REG_ADDR(inst),                                \
		.clk_freq = DT_INST_PROP(inst, clock_frequency),               \
		.pcr = DT_INST_PROP_BY_IDX(inst, pcr, 0),                     \
		.girq = DT_INST_PROP_BY_IDX(inst, girqs, 0),                  \
		.chip_select_count =                                           \
			DT_INST_PROP_OR(inst, chip_select_count, 1),           \
		.sw_multi_periph =                                             \
			DT_INST_PROP(inst, software_multiperipheral),          \
		.irq_config_func = mspi_xec_irq_config_##inst,                \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                 \
		.ce_gpios = ce_gpios_##inst,                                   \
		.num_ce_gpios = ARRAY_SIZE(ce_gpios_##inst),                   \
	};                                                                     \
	DEVICE_DT_INST_DEFINE(inst, mspi_xec_init, NULL,                       \
			      &mspi_xec_data_##inst,                           \
			      &mspi_xec_config_##inst,                         \
			      POST_KERNEL, CONFIG_MSPI_INIT_PRIORITY,          \
			      &mspi_xec_api);

DT_INST_FOREACH_STATUS_OKAY(MSPI_XEC_INST)
