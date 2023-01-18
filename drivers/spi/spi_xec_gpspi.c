/*
 * Copyright (c) 2022 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_gpspi

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_xec_gpspi, CONFIG_SPI_LOG_LEVEL);

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/mchp_xec_clock_control.h>
#ifdef CONFIG_SPI_XEC_GPSPI_DMA
#include <zephyr/drivers/dma.h>
#endif
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/interrupt_controller/intc_mchp_xec_ecia.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pinmux.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <soc.h>

#include "spi_context.h"

#define XEC_GPSPI_ENABLE_REG_MSK		0x1u
#define XEC_GPSPI_ENABLE_EN_POS			0

#define XEC_GPSPI_CTRL_REG_MSK			0x7fu
#define XEC_GPSPI_CTRL_LSBF_POS			0
#define XEC_GPSPI_CTRL_BI_DIR_OUT_EN_POS	1
#define XEC_GPSPI_CTRL_SPDIN_SEL_POS		2
#define XEC_GPSPI_CTRL_SPDIN_SEL_MSK0		0x3u
#define XEC_GPSPI_CTRL_SPDIN_SEL_MSK		0xcu
#define XEC_GPSPI_CTRL_SPDIN_SEL_FULL_DUPLEX	0
#define XEC_GPSPI_CTRL_SPDIN_SEL_HALF_DUPLEX	0x4u
#define XEC_GPSPI_CTRL_SPDIN_SEL_DUAL		0x8u
#define XEC_GPSPI_CTRL_SRST_POS			4
#define XEC_GPSPI_CTRL_AUTO_READ_POS		5
#define XEC_GPSPI_CTRL_CE_POS			6

/* Status register is read-only */
#define XEC_GPSPI_STATUS_REG_MSK		0x7u
#define XEC_GPSPI_STATUS_TXBE_POS		0
#define XEC_GPSPI_STATUS_RXBF_POS		1
#define XEC_GPSPI_STATUS_ACTIVE_POS		2

#define XEC_GPSPI_STS_RXBF_TXBE			\
	(BIT(XEC_GPSPI_STATUS_RXBF_POS) | BIT(XEC_GPSPI_STATUS_TXBE_POS))

#define XEC_GPSPI_CLK_CTRL_REG_MSK		0x17u
#define XEC_GPSPI_CLK_CTRL_PH_MSK		0x07u
#define XEC_GPSPI_CLK_CTRL_TCLKPH_POS		0
#define XEC_GPSPI_CLK_CTRL_RCLKPH_POS		1
#define XEC_GPSPI_CLK_CTRL_CLKPOL_POS		2

/* default GPSPI reference clock source is 2 MHz */
#define XEC_GPSPI_CLK_CTRL_SRC_CLK_48M_POS	4

#define XEC_GPSPI_CLK_GEN_PRELOAD_POS		0
#define XEC_GPSPI_CLK_GEN_PRELOAD_MSK		0x3fu

#define XEC_GPSPI_DMA_BUF_SIZE			64

/*
 * SPI signalling mode: CPOL and CPHA
 * CPOL = 0 is clock idle state is low, 1 is clock idle state is high
 * CPHA = 0 Transmitter changes data on trailing of preceding clock cycle.
 *          Receiver samples data on leading edge of clock cyle.
 *        1 Transmitter changes data on leading edge of current clock cycle.
 *          Receiver samples data on the trailing edge of clock cycle.
 * SPI Mode and GPSPI controller nomenclature:
 * Mode CPOL CPHA  clock idle    data sampled    data shifted out
 *  0     0    0   low           rising edge     falling edge
 *  1     0    1   low           falling edge    rising edge
 *  2     1    0   high          rising edge     falling edge
 *  3     1    1   high          falling edge    rising edge
 * GPSPI clock control bits
 * Mode CLKPOL RCLKPH TCLKPH  NOTES
 *  0     0      0      0     data is valid before first rising edge
 *  1     0      1      1
 *  2     1      1      1
 *  3     1      0      0     data is valid before first falling edge
 */
#define XEC_GPIO_CLK_CTRL_SPI_MODE_0		0
#define XEC_GPIO_CLK_CTRL_SPI_MODE_1		(BIT(XEC_GPSPI_CLK_CTRL_RCLKPH_POS) \
						 | BIT(XEC_GPSPI_CLK_CTRL_TCLKPH_POS))
#define XEC_GPIO_CLK_CTRL_SPI_MODE_2		(BIT(XEC_GPSPI_CLK_CTRL_CLKPOL_POS) \
						 | BIT(XEC_GPSPI_CLK_CTRL_RCLKPH_POS) \
						 | BIT(XEC_GPSPI_CLK_CTRL_TCLKPH_POS))
#define XEC_GPIO_CLK_CTRL_SPI_MODE_3		BIT(XEC_GPSPI_CLK_CTRL_CLKPOL_POS)

struct xec_gpspi_regs {
	volatile uint8_t enable;
	uint8_t rsvd_01_03[3];
	volatile uint8_t control;
	uint8_t rsvd_05_07[3];
	volatile uint8_t status;
	uint8_t rsvd_09_0a[3];
	volatile uint8_t tx_data;
	uint8_t rsvd_0d_0f[3];
	volatile uint8_t rx_data;
	uint8_t rsvd_11_13[3];
	volatile uint8_t clock_control;
	uint8_t rsvd_15_17[3];
	volatile uint8_t clock_gen;
	uint8_t rsvd_19_1b[3];
};

struct irq_info {
	uint8_t irq_num;
	uint8_t irq_pri;
	uint8_t girq;
	uint8_t girq_pos;
};

/* Device constant configuration parameters */
struct spi_xec_gpspi_config {
	struct xec_gpspi_regs *regs;
	uint32_t freqhz;
	struct irq_info irqtx;
	struct irq_info irqrx;
	uint8_t pcr_idx;
	uint8_t pcr_pos;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_connect)(void);
#ifdef CONFIG_SPI_XEC_GPSPI_DMA
	const struct device *dma_dev;
	uint8_t tx_dma_request;
	uint8_t tx_dma_channel;
	uint8_t rx_dma_request;
	uint8_t rx_dma_channel;
#endif
};

#define XEC_QMSPI_XFR_FLAG_TX		BIT(0)
#define XEC_QMSPI_XFR_FLAG_STARTED	BIT(1)

/* 4 microsecond wait per byte */
#define XEC_GPSPI_WAIT_INTERVAL		10

/* Default number of intervales to wait */
#define XEC_GPSPI_WAIT_LOOPS		16

/* Device run time data */
struct spi_xec_gpspi_data {
	struct spi_context ctx;
	struct spi_config scfg;
	uint8_t configured;
	uint8_t isr_ctx_done;
#ifdef CONFIG_SPI_XEC_GPSPI_DMA
	const struct device *dev;
	struct k_sem dma_sem;
	uint32_t dma_len;
	int rx_cb_status;
	volatile int tx_dma_done;
	volatile int tx_dma_status;
	volatile int rx_dma_done;
	volatile int rx_dma_status;
	struct dma_config tx_dma_cfg;
	struct dma_config rx_dma_cfg;
	struct dma_block_config tx_dma_block;
	struct dma_block_config rx_dma_block;
	volatile uint8_t dma_buf[XEC_GPSPI_DMA_BUF_SIZE];
#endif
};

/* reset GPSPI controller with save/restore of timing registers. */
static void xec_gpspi_reset(struct xec_gpspi_regs *regs)
{
	uint8_t clk_ctrl, clk_gen;

	clk_ctrl = regs->clock_control;
	clk_gen = regs->clock_gen;

	/* soft reset is self clearing */
	regs->control |= BIT(XEC_GPSPI_CTRL_SRST_POS);

	regs->clock_gen = clk_gen;
	regs->clock_control = clk_ctrl;
}

/* Calculate GPSPI frequency divider register field value based upon
 * the configured GPSPI reference clock frequency: 48 or 2 MHz.
 * SPI clock frequency = (reference_clock) / (2 * preload)
 * preload 0 is a special value. When preload is 0 the hardware forces
 * SPI clock to be 48 MHz.
 */
static int gpspi_configure_spi_clock(struct xec_gpspi_regs * const regs, uint32_t spi_clk_hz)
{
	uint32_t preload, ref_clk;

	if (spi_clk_hz > MHZ(24)) {
		/* HW can only do 48 MHz. set preload = 0 */
		preload = 0u;
	} else if (spi_clk_hz < KHZ(16)) {
		regs->clock_control &= ~BIT(XEC_GPSPI_CLK_CTRL_SRC_CLK_48M_POS);
		preload = 63u;
	} else {
		regs->clock_control &= ~BIT(XEC_GPSPI_CLK_CTRL_SRC_CLK_48M_POS);
		ref_clk = MHZ(2);
		if (spi_clk_hz > ref_clk) {
			ref_clk = MHZ(48);
			regs->clock_control |= BIT(XEC_GPSPI_CLK_CTRL_SRC_CLK_48M_POS);
		}

		preload = ref_clk / (2u * spi_clk_hz);
	}

	regs->clock_gen = preload;

	return 0;
}


static const uint8_t gpspi_spi_mode_tbl[] = {
	XEC_GPIO_CLK_CTRL_SPI_MODE_0, XEC_GPIO_CLK_CTRL_SPI_MODE_1,
	XEC_GPIO_CLK_CTRL_SPI_MODE_2, XEC_GPIO_CLK_CTRL_SPI_MODE_3
};

static int gpspi_set_spi_mode(const struct device *dev, const struct spi_config *spi_conf)
{
	const struct spi_xec_gpspi_config * const cfg = dev->config;
	struct xec_gpspi_regs * const regs = cfg->regs;
	uint8_t index = 0;

	if (spi_conf->operation & SPI_MODE_CPHA) {
		index = 1u;
	}
	if (spi_conf->operation & SPI_MODE_CPOL) {
		index += 2u;
	}

	regs->clock_control &= ~(XEC_GPSPI_CLK_CTRL_PH_MSK);
	regs->clock_control |= gpspi_spi_mode_tbl[index & 0x3u];

	return 0;
}

/* NOTE: struct spi_config.operation is defined as uint32_t when
 * CONFIG_SPI_EXTENDED_MODES is enabled otherwise it is uint16_t.
 * SPI_LINES_xxx are located at bit 16.
 */
static int gpspi_check_unsupported_features(const struct spi_config *spi_conf)
{
	uint32_t lines = spi_conf->operation & SPI_LINES_MASK;

	if (spi_conf->operation & (SPI_OP_MODE_SLAVE | SPI_MODE_LOOP)) {
		LOG_ERR("Does not support SPI device or loop back");
		return -ENOTSUP;
	}

	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES)) {
		if ((lines != SPI_LINES_SINGLE) && (lines != SPI_LINES_DUAL)) {
			LOG_ERR("Supports single(full-duples) and dual only");
			return -ENOTSUP;
		}
	}

	if (spi_conf->operation & SPI_CS_ACTIVE_HIGH) {
		LOG_ERR("CS active high not supported");
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(spi_conf->operation) != 8) {
		LOG_ERR("Word size != 8 not supported");
		return -ENOTSUP;
	}

	return 0;
}

static int req_full_reconfig(const struct device *dev, const struct spi_config *spi_conf)
{
	struct spi_xec_gpspi_data * const data = dev->data;

	if (data->scfg.frequency != spi_conf->frequency) {
		return 1; /* frequency change */
	} else if ((data->scfg.operation & SPI_MODE_MASK)
		   != (spi_conf->operation & SPI_MODE_MASK)) {
		return 1; /* CPOL and/or CPHA changed */
	}

	return 0;
}

/* Configure GPSPI for full-duplex operation.
 * We support CONFIG_SPI_EXTENDED_MODES dual mode only. Controller does not support
 * quad mode.
 * If target device is a SPI flash the commands for dual/quad involve mutiple phases:
 * transmit command, transmit address, optional transmit mode byte, optional transmit
 * clocks with I/O tri-stated, read data using multiple I/O pins.
 * The SPI driver specifies the I/O mode in struct spi_config on each call to the
 * transfer API. This means struct spi_config contents can change on every call and
 * the previous call may have left the SPI open (chip select asserted). For this case
 * reconfiguring the controller should not cause any SPI signals to glitch.
 * Rules:
 * If GPSPI.control.CE is 1 then chip select is asserted
 *   Ignore frequency change. Should we return error if new frequency?
 *   Record other flags such as SPI_HOLD_ON_CS and SPI_LOCK_ON
 *   Should we allow MSBF/LSBF change?
 *   Allow I/O mode change: half-duplex, full-duplex, and dual. Reject quad.
 * Else
 *   Allowed to do full reconfigure (controller reset)
 * Endif
 *
 */
static int gpspi_configure(const struct device *dev, const struct spi_config *spi_conf)
{
	const struct spi_xec_gpspi_config * const cfg = dev->config;
	struct spi_xec_gpspi_data * const data = dev->data;
	struct xec_gpspi_regs * const regs = cfg->regs;
	struct spi_context *ctx = &data->ctx;
	int ret;
	uint8_t ctrl;

	/* Need to reconfigure? */
	ret = memcmp(&data->scfg, spi_conf, sizeof(struct spi_config));
	if (ret == 0) {
		return 0;
	}

	ret = gpspi_check_unsupported_features(spi_conf);
	if (ret) {
		return ret;
	}

	if (req_full_reconfig(dev, spi_conf)) {
		if (!(regs->control & BIT(XEC_GPSPI_CTRL_CE_POS))) {
			xec_gpspi_reset(regs);
		}
		gpspi_configure_spi_clock(regs, spi_conf->frequency);
		gpspi_set_spi_mode(dev, spi_conf);
	}

	ctrl = regs->control & ~(XEC_GPSPI_CTRL_SPDIN_SEL_MSK | BIT(XEC_GPSPI_CTRL_LSBF_POS));
	ctrl |= BIT(XEC_GPSPI_CTRL_BI_DIR_OUT_EN_POS);

	if ((spi_conf->operation & SPI_LINES_MASK) == SPI_LINES_DUAL) {
		ctrl |= XEC_GPSPI_CTRL_SPDIN_SEL_DUAL;
	} else {
		ctrl |= XEC_GPSPI_CTRL_SPDIN_SEL_FULL_DUPLEX;
	}

	if (spi_conf->operation & SPI_TRANSFER_LSB) {
		ctrl |= BIT(XEC_GPSPI_CTRL_LSBF_POS);
	}
	regs->control = ctrl;

	ctx->config = spi_conf;

	memcpy(&data->scfg, spi_conf, sizeof(struct spi_config));

	regs->enable |= BIT(XEC_GPSPI_ENABLE_EN_POS);
	if (regs->status & BIT(XEC_GPSPI_STATUS_RXBF_POS)) {
		/* clear RX buffer */
		regs->control &= ~(BIT(XEC_GPSPI_CTRL_AUTO_READ_POS));
		regs->status = regs->rx_data;
		regs->status = regs->rx_data;
	}

	data->configured = 1u;

	return 0;
}

#ifdef CONFIG_SPI_XEC_GPSPI_DMA
/* DMA driver callbacks
 * dev = DMA device
 * user_data = GPSPI device
 * channel = DMA device channel
 * status = DMA device channel status. 0=done, non-zero is error
 */
static void xec_gpspi_dma_tx_cb(const struct device *dev, void *user_data,
				uint32_t channel, int status)
{
	const struct device *gpspi_dev = (struct device *)user_data;
	struct spi_xec_gpspi_data * const data = gpspi_dev->data;

	data->tx_dma_done = 1;
	data->tx_dma_status = status;
}

static void xec_gpspi_dma_rx_cb(const struct device *dev, void *user_data,
				uint32_t channel, int status)
{
	const struct device *gpspi_dev = (struct device *)user_data;
	struct spi_xec_gpspi_data * const data = gpspi_dev->data;

	data->rx_dma_done = 1;
	data->rx_dma_status = status;
	data->rx_cb_status = 0;

	/* increments semaphore count */
	k_sem_give(&data->dma_sem);
}

static void xec_gpspi_dma_cfg(const struct device *dev, struct dma_config *dma_cfg,
			      struct dma_block_config *block_cfg, dma_callback_t cb,
			      int tx_dir)
{
	const struct spi_xec_gpspi_config *devcfg = dev->config;
	struct xec_gpspi_regs * const regs = devcfg->regs;

	dma_cfg->source_data_size = 1u;
	dma_cfg->dest_data_size = 1u;
	dma_cfg->block_count = 1u;
	dma_cfg->head_block = block_cfg;
	dma_cfg->user_data = (void *)dev;
	dma_cfg->dma_callback = cb;
	if (tx_dir) {
		dma_cfg->dma_slot = devcfg->tx_dma_request;
		dma_cfg->channel_direction = MEMORY_TO_PERIPHERAL;
		block_cfg->dest_address = (uint32_t)&regs->tx_data;
		block_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		block_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		dma_cfg->dma_slot = devcfg->rx_dma_request;
		dma_cfg->channel_direction = PERIPHERAL_TO_MEMORY;
		block_cfg->source_address = (uint32_t)&regs->rx_data;
		block_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		block_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}
}

static int xec_gpspi_xfr_dma(const struct device *dev, size_t *pdlen)
{
	const struct spi_xec_gpspi_config *devcfg = dev->config;
	struct xec_gpspi_regs * const regs = devcfg->regs;
	struct spi_xec_gpspi_data * const data = dev->data;
	struct spi_context *ctx = &data->ctx;
	struct dma_config *tx_dma_cfg = &data->tx_dma_cfg;
	struct dma_config *rx_dma_cfg = &data->rx_dma_cfg;
	struct dma_block_config *tx_dma_block = &data->tx_dma_block;
	struct dma_block_config *rx_dma_block = &data->rx_dma_block;
	size_t dma_len;
	bool use_dma_buf;
	int ret;

	use_dma_buf = false;
	dma_len = spi_context_max_continuous_chunk(ctx);

	data->dma_len = dma_len;
	data->tx_dma_done = 0;
	data->tx_dma_status = 0;
	data->rx_dma_done = 0;
	data->rx_dma_status = 0;

	/* TX DMA always required to generate SPI clocks. */
	if (spi_context_tx_buf_on(ctx)) {
		regs->control |= BIT(XEC_GPSPI_CTRL_BI_DIR_OUT_EN_POS);
		tx_dma_block->source_address = (uint32_t)ctx->tx_buf;
	} else { /* No data to TX, set output pin direction to input */
		regs->control &= ~BIT(XEC_GPSPI_CTRL_BI_DIR_OUT_EN_POS);
		tx_dma_block->source_address = (uint32_t)data->dma_buf;
		use_dma_buf = true;
	}

	if (spi_context_rx_buf_on(ctx)) {
		rx_dma_block->dest_address = (uint32_t)ctx->rx_buf;
	} else {
		rx_dma_block->dest_address = (uint32_t)data->dma_buf;
		use_dma_buf = true;
	}

	if (use_dma_buf && (dma_len > XEC_GPSPI_DMA_BUF_SIZE)) {
		dma_len = XEC_GPSPI_DMA_BUF_SIZE;
		data->dma_len = dma_len;
	}

	if (pdlen) {
		*pdlen = dma_len;
	}

	tx_dma_block->block_size = dma_len;
	rx_dma_block->block_size = dma_len;

	ret = dma_config(devcfg->dma_dev, devcfg->tx_dma_channel, tx_dma_cfg);
	if (ret) {
		return ret;
	}

	ret = dma_config(devcfg->dma_dev, devcfg->rx_dma_channel, rx_dma_cfg);
	if (ret) {
		return ret;
	}

	/* HW state: TXBE=1 TX_DMA_REQ=1, RXBF=0 RX_DMA_REQ=0
	 * Start DMA in order: RX then TX
	 */
	ret = dma_start(devcfg->dma_dev, devcfg->rx_dma_channel);
	if (ret) {
		return ret;
	}
	ret = dma_start(devcfg->dma_dev, devcfg->tx_dma_channel);
	if (ret) {
		dma_stop(devcfg->dma_dev, devcfg->rx_dma_channel);
	}

	return ret;
}

/* GPSPI synchronous transfer using DMA. GPSPI implements one byte TX and RX FIFO's
 * in the form of its TX DATA and RX DATA registers. GPSPI requires write to TX DATA
 * to generate clocks. We always configure TX and RX DMA to ensure clocks are
 * generated. We also make sure both channels are configured for the same byte length.
 * The caller may not provide both TX and RX buffers. If one is missing we use a buffer
 * in the device data structure. This local buffer is limited in size and if used we
 * break up the SPI context into multiple DMA chunks. DMA driver callbacks are used
 * to know when both DMA channels are done.
 */
static int xec_gpspi_xfr_sync(const struct device *dev, const struct spi_config *spi_conf)
{
	const struct spi_xec_gpspi_config *devcfg = dev->config;
	struct xec_gpspi_regs * const regs = devcfg->regs;
	struct spi_xec_gpspi_data * const data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret = 0;
	size_t dma_len;

	if ((regs->status & XEC_GPSPI_STS_RXBF_TXBE) != BIT(XEC_GPSPI_STATUS_TXBE_POS)) {
		return -EBUSY;
	}

	xec_gpspi_dma_cfg(dev, &data->tx_dma_cfg, &data->tx_dma_block, xec_gpspi_dma_tx_cb, 1);
	xec_gpspi_dma_cfg(dev, &data->rx_dma_cfg, &data->rx_dma_block, xec_gpspi_dma_rx_cb, 0);

	while (spi_context_tx_buf_on(ctx) || spi_context_rx_buf_on(ctx)) {
		k_sem_reset(&data->dma_sem);

		dma_len = 0;
		ret = xec_gpspi_xfr_dma(dev, &dma_len);
		if (ret) {
			return ret;
		}

		/* wait on RX DMA to finish */
		ret = k_sem_take(&data->dma_sem, K_MSEC(100));
		if (ret) {
			return -EIO;
		}

		if (spi_context_tx_buf_on(ctx)) {
			spi_context_update_tx(ctx, 1, dma_len);
		}
		if (spi_context_rx_buf_on(ctx)) {
			spi_context_update_rx(ctx, 1, dma_len);
		}
	}

	spi_context_complete(ctx, dev, 0);

	return 0;
}
#else
/* Non-DMA synchronous (blocking) transfer.
 * The configuration routine programs the controller for full-duplex or dual I/O mode.
 * GPSPI controller requires a byte write to its TX data register to generate
 * SPI clocks. The controller always samples input line(s) on the receive clock
 * edge specified by CPOL/CPHA. Once 8-bits is recevied it is stored in GPSPI
 * RX data register and RXBF status is set. We must read RX data register to
 * clear RXBF. If RX data is not read and more clocks are generated then the HW
 * will require at least to back to back reads of RX data to clear RXBF.
 * SPI buffer handling:
 * When transmit buffers are exhausted we set I/O direction to input and write a 0
 * to GPSPI TX data register to generate SPI clocks. Data sampled on input line(s)
 * is stored if a receive buffer exists or discarded otherwise.
 */

/* 8 bits @ 48 MHz     167 ns
 * 8 bits @ 1 MHz      8000 ns
 * 8 bits @ 15.9 KHz   252000 ns
 * OR we could use 10 us per byte for timeout
 */
static int xec_gpspi_spin_yield(int *counter, int max_count)
{
	*counter = *counter + 1;

	if (*counter > max_count) {
		return -ETIMEDOUT;
	}

	k_busy_wait(XEC_GPSPI_WAIT_INTERVAL);

	return 0;
}

static int xec_gpspi_xfr_sync(const struct device *dev,
			      const struct spi_config *spi_conf)
{
	const struct spi_xec_gpspi_config *cfg = dev->config;
	struct spi_xec_gpspi_data * const data = dev->data;
	struct xec_gpspi_regs * const regs = cfg->regs;
	struct spi_context *ctx = &data->ctx;
	size_t cur_xfer_len;
	int counter, ret;
	uint8_t txb, rxb;

	if ((regs->status & XEC_GPSPI_STS_RXBF_TXBE) != BIT(XEC_GPSPI_STATUS_TXBE_POS)) {
		return -EBUSY;
	}

	while (spi_context_tx_buf_on(ctx) || spi_context_rx_buf_on(ctx)) {
		cur_xfer_len = spi_context_longest_current_buf(ctx);

		for (size_t i = 0; i < cur_xfer_len; i++) {

			/* Write byte to generate SPI clocks */
			if (spi_context_tx_buf_on(ctx)) {
				regs->control |= BIT(XEC_GPSPI_CTRL_BI_DIR_OUT_EN_POS);
				txb = *ctx->tx_buf;
				regs->tx_data = txb;
				spi_context_update_tx(ctx, 1, 1);
			} else {
				regs->control &= ~BIT(XEC_GPSPI_CTRL_BI_DIR_OUT_EN_POS);
				regs->tx_data = 0;
			}

			/* Wait for rx data to fill with one data byte */
			counter = 0;
			while ((regs->status & XEC_GPSPI_STS_RXBF_TXBE)
				!= XEC_GPSPI_STS_RXBF_TXBE) {

				ret = xec_gpspi_spin_yield(&counter, XEC_GPSPI_WAIT_LOOPS);
				if (ret) {
					return ret;
				}
			}

			/* Get received byte */
			rxb = regs->rx_data;

			/* Store received byte if rx buffer is on */
			if (spi_context_rx_on(ctx)) {
				*ctx->rx_buf = rxb;
				spi_context_update_rx(ctx, 1, 1);
			}
		}
	}

	spi_context_complete(ctx, dev, 0);

	return 0;
}
#endif /* CONFIG_SPI_XEC_GPSPI_DMA */

#ifdef CONFIG_SPI_ASYNC
#ifdef CONFIG_SPI_XEC_GPSPI_DMA
/* Aync DMA uses different DMA RX callback. RX DMA callback start another
 * SPI sequency using DMA if more SPI context data is present.
 * !!! WARNING !!!
 * This function is called in DMA channel ISR context.
 */
static void xec_gpspi_dma_rx_cb_async(const struct device *dev, void *user_data,
				      uint32_t channel, int status)
{
	const struct device *gpspi_dev = (struct device *)user_data;
	const struct spi_xec_gpspi_config *devcfg = gpspi_dev->config;
	struct spi_xec_gpspi_data * const data = gpspi_dev->data;
	struct xec_gpspi_regs * const regs = devcfg->regs;
	struct spi_context *ctx = &data->ctx;
	size_t dma_len;
	int ret;

	data->rx_dma_done = 1;
	data->rx_dma_status = status;
	data->rx_cb_status = 0;

	if (spi_context_tx_buf_on(ctx)) {
		spi_context_update_tx(ctx, 1, data->dma_len);
	}
	if (spi_context_rx_buf_on(ctx)) {
		spi_context_update_rx(ctx, 1, data->dma_len);
	}

	/* more data to transfer? */
	if (spi_context_tx_buf_on(ctx) || spi_context_rx_buf_on(ctx)) {
		dma_len = 0;
		ret = xec_gpspi_xfr_dma(gpspi_dev, &dma_len);
		if (ret) {
			data->rx_cb_status = ret;
			spi_context_complete(ctx, gpspi_dev, ret);
			regs->control &= ~BIT(XEC_GPSPI_CTRL_CE_POS);
		}
	} else {
		spi_context_complete(ctx, gpspi_dev, 0);
		if (!(ctx->config->operation & SPI_LOCK_ON)) {
			regs->control &= ~BIT(XEC_GPSPI_CTRL_CE_POS);
		}
	}
}

/* DMA version */
static int xec_gpspi_xfr_async(const struct device *dev,
			       const struct spi_config *spi_conf)
{
	const struct spi_xec_gpspi_config *devcfg = dev->config;
	struct xec_gpspi_regs * const regs = devcfg->regs;
	struct spi_xec_gpspi_data * const data = dev->data;
	struct spi_context *ctx = &data->ctx;
	size_t dma_len;
	int ret = 0;

	if ((regs->status & XEC_GPSPI_STS_RXBF_TXBE) != BIT(XEC_GPSPI_STATUS_TXBE_POS)) {
		return -EBUSY;
	}

	xec_gpspi_dma_cfg(dev, &data->tx_dma_cfg, &data->tx_dma_block,
			  xec_gpspi_dma_tx_cb, 1);
	xec_gpspi_dma_cfg(dev, &data->rx_dma_cfg, &data->rx_dma_block,
			  xec_gpspi_dma_rx_cb_async, 0);

	if (spi_context_tx_buf_on(ctx) || spi_context_rx_buf_on(ctx)) {

		dma_len = 0;
		ret = xec_gpspi_xfr_dma(dev, &dma_len);
		if (ret) {
			return ret;
		}
	} else {
		spi_context_complete(ctx, dev, 0);
	}

	return 0;
}
#else /* 0 */

/* Asynchronous mode non-DMA transfer. An interrupt will be generated
 * for every byte transfer. Implement using GPSPI RXBF interrupt.
 * This code enables the RXBF interrupt and writes the GPSPI TX FIFO
 * to trigger the interrupt when HW completes clocking 8 bits of
 * data to its RX data register.
 * OR should we use both TXBE and RXBF interrupts?
 * 1. This routine enables both TXBE and RXBF interrupts.
 * 2. TXBE interrupt fires
 * 3. TXBE handler writes TX data byte register from an available data buffer
 *    or 0 if RX and clears status.
 * 4. RXBF handler reads the byte from RX data register and clears status.
 *    If RX buffer exists write data to it.
 */
static int xec_gpspi_xfr_async(const struct device *dev,
			       const struct spi_config *spi_conf)
{
	const struct spi_xec_gpspi_config *cfg = dev->config;
	struct spi_xec_gpspi_data * const data = dev->data;
	struct xec_gpspi_regs * const regs = cfg->regs;
	struct spi_context *ctx = &data->ctx;
	uint8_t txb;

	if ((regs->status & XEC_GPSPI_STS_RXBF_TXBE) != BIT(XEC_GPSPI_STATUS_TXBE_POS)) {
		return -EBUSY;
	}

	if (spi_context_tx_buf_on(ctx) || spi_context_rx_buf_on(ctx)) {
		data->isr_ctx_done = 0u;

		/* Enable GPSPI controller's RXBF interrupt.
		 * Controller has no interrupt enables. We turn it
		 * on in the GIRQ.
		 */
		mchp_xec_ecia_girq_src_en(cfg->irqrx.girq, cfg->irqrx.girq_pos);

		if (spi_context_tx_buf_on(ctx)) {
			txb = *ctx->tx_buf;
			spi_context_update_tx(ctx, 1, 1);
			regs->control |= BIT(XEC_GPSPI_CTRL_BI_DIR_OUT_EN_POS);
			regs->tx_data = txb;
		} else {
			regs->control &= ~BIT(XEC_GPSPI_CTRL_BI_DIR_OUT_EN_POS);
			regs->tx_data = 0;
		}
	} else {
		spi_context_complete(ctx, dev, 0);
	}

	return 0;
}
#endif /* CONFIG_SPI_XEC_GPSPI_DMA */
#endif /* CONFIG_SPI_ASYNC */

/* SPI model is for every clock edge where data is transmitted we must
 * read data in on the sample clock edge. The chosen SPI mode specifies
 * the transmit and input sample clock edges.
 * If no RX buffer corresponding to a TX buffer is provided the sampled
 * data is discarded. The XEC GPSPI controller requires sampled data to
 * be always read or its overrun status gets stuck requiring multiple
 * reads of the RX data register to clear. The controller also has an
 * auto-read HW feature where reads of the RX data register cause the
 * controller to generate clocks for the next byte. We will not use
 * auto-read for synchronous transfer.
 */
static int xec_gpspi_xfr(const struct device *dev,
			 const struct spi_config *spi_conf,
			 const struct spi_buf_set *tx_bufs,
			 const struct spi_buf_set *rx_bufs,
			 bool asynchronous,
			 spi_callback_t cb,
			 void *userdata)
{
	const struct spi_xec_gpspi_config *cfg = dev->config;
	struct xec_gpspi_regs * const regs = cfg->regs;
	struct spi_xec_gpspi_data * const data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret = 0;

	spi_context_lock(ctx, asynchronous, cb, userdata, spi_conf);

	ret = gpspi_configure(dev, spi_conf);
	if (ret != 0) {
		spi_context_release(ctx, ret);
		return ret;
	}


	spi_context_cs_control(&data->ctx, true);
	regs->control |= BIT(XEC_GPSPI_CTRL_CE_POS);

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

#ifdef CONFIG_SPI_ASYNC
	if (asynchronous) {
		ret = xec_gpspi_xfr_async(dev, spi_conf);
		if (ret) {
			regs->control &= ~(BIT(XEC_GPSPI_CTRL_CE_POS));
			spi_context_unlock_unconditionally(&data->ctx);
		}
		return ret;
	}
#endif
	ret = xec_gpspi_xfr_sync(dev, spi_conf);
	if (ret) {
		regs->control &= ~(BIT(XEC_GPSPI_CTRL_CE_POS));
		spi_context_unlock_unconditionally(&data->ctx);
		return ret;
	}

	if (!(spi_conf->operation & SPI_HOLD_ON_CS)) {
		spi_context_cs_control(ctx, false);
		regs->control &= ~(BIT(XEC_GPSPI_CTRL_CE_POS));
	}

	/* Attempts to take semaphore with timeout. Descriptor transfer
	 * routine completes the context giving the semaphore.
	 */
	ret = spi_context_wait_for_completion(ctx);

	/* gives semaphore */
	spi_context_release(ctx, ret);

	return ret;
}

static int xec_gpspi_transceive(const struct device *dev,
				const struct spi_config *spi_conf,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	return xec_gpspi_xfr(dev, spi_conf, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int xec_gpspi_transceive_async(const struct device *dev,
				      const struct spi_config *spi_conf,
				      const struct spi_buf_set *tx_bufs,
				      const struct spi_buf_set *rx_bufs,
				      spi_callback_t cb,
				      void *userdata)
{
	/* return -ENOTSUP; */
	return xec_gpspi_xfr(dev, spi_conf, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif

static int xec_gpspi_release(const struct device *dev,
			     const struct spi_config *spi_conf)
{
	struct spi_xec_gpspi_data * const data = dev->data;
	const struct spi_xec_gpspi_config *cfg = dev->config;
	struct xec_gpspi_regs * const regs = cfg->regs;
	struct spi_context *ctx = &data->ctx;

	regs->control &= ~(BIT(XEC_GPSPI_CTRL_CE_POS) | BIT(XEC_GPSPI_CTRL_AUTO_READ_POS));

	if (regs->status & BIT(XEC_GPSPI_STATUS_RXBF_POS)) {
		regs->status = regs->rx_data;
	}

	spi_context_unlock_unconditionally(ctx);

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int xec_gpspi_pm_action(const struct device *dev,
			       enum pm_device_action action)
{
	const struct spi_xec_gpspi_config *cfg = dev->config;
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			return ret;
		}
		break;

	case PM_DEVICE_ACTION_SUSPEND:
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_SLEEP);
		if ((ret == 0) || (ret == -ENOENT)) {
			ret = 0; /* -ENOENT indicates no sleep state defined */
		}
		break;

	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

/* Called for each GPSPI controller instance
 * Disable sleep control.
 * Disable and clear interrupt status.
 * Initialize SPI context.
 * GPSPI will be fully configured and enabled when the transceive API
 * is called.
 */
static int xec_gpspi_init(const struct device *dev)
{
	const struct spi_xec_gpspi_config *cfg = dev->config;
	struct spi_xec_gpspi_data * const data = dev->data;
	struct xec_gpspi_regs * const regs = cfg->regs;
	int ret = 0;

	data->configured = 0u;
	memset(&data->scfg, 0, sizeof(struct spi_config));

	z_mchp_xec_pcr_periph_sleep(cfg->pcr_idx, cfg->pcr_pos, 0);

#ifdef CONFIG_SPI_XEC_GPSPI_DMA
	if (!device_is_ready(cfg->dma_dev)) {
		return -ENODEV;
	}
	data->dev = dev;
	k_sem_init(&data->dma_sem, 0, 2);
#endif

	/* chip selects */
	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret) {
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("QSPI pinctrl setup failed (%d)", ret);
		return ret;
	}

	xec_gpspi_reset(regs);

	spi_context_unlock_unconditionally(&data->ctx);

#ifdef CONFIG_SPI_ASYNC
	cfg->irq_connect();
#endif

	return 0;
}

static const struct spi_driver_api spi_xec_gpspi_driver_api = {
	.transceive = xec_gpspi_transceive,
	.release = xec_gpspi_release,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = xec_gpspi_transceive_async,
#endif
};

/* GPSPI RXBF interrupt handler. Enabled when the first byte of
 * a series of buffers is begun by the asynchronous transceive
 * routine.
 * 1. Always read the data byte from RX FIFO register which
 *    clears the RXBF signal to the latched GIRQ.
 * 2. Clear the latched GIRQ RXBF bit for this controller.
 * 3. If we have a RX context buffer store the data byte
 *    and update the context buffer tracking.
 * 4. If more TX or RX context data then write TX FIFO with
 *    data from TX context else write 0 to TX FIFO. This
 *    generates SPI clocks.
 *    Else no more TX or RX context data then disable RXBF
 *    interrupt and indicate current TX/RX context is finished.
 */
static void xec_gpspi_rxbf_handler(const struct device *dev)
{
	const struct spi_xec_gpspi_config *cfg = dev->config;
	struct spi_xec_gpspi_data * const data = dev->data;
	struct xec_gpspi_regs * const regs = cfg->regs;
	struct spi_context *ctx = &data->ctx;
	bool rxbon = false, txbon = false;
	uint8_t rxb = regs->rx_data;

	mchp_xec_ecia_girq_src_clr(cfg->irqrx.girq, cfg->irqrx.girq_pos);

	rxbon = spi_context_rx_buf_on(ctx);
	if (rxbon) {
		*ctx->rx_buf = rxb;
		spi_context_update_rx(ctx, 1, 1);
		rxbon = spi_context_rx_buf_on(ctx);
	}

	txbon = spi_context_tx_buf_on(ctx);
	if (rxbon || txbon) {
		if (txbon) {
			regs->control |= BIT(XEC_GPSPI_CTRL_BI_DIR_OUT_EN_POS);
			regs->tx_data = *ctx->tx_buf;
			spi_context_update_tx(ctx, 1, 1);
		} else {
			regs->control &= ~BIT(XEC_GPSPI_CTRL_BI_DIR_OUT_EN_POS);
			regs->tx_data = 0;
		}
	} else {
		mchp_xec_ecia_girq_src_dis(cfg->irqrx.girq, cfg->irqrx.girq_pos);
		spi_context_complete(ctx, dev, 0);
		if (!(ctx->config->operation & SPI_LOCK_ON)) {
			regs->control &= ~BIT(XEC_GPSPI_CTRL_CE_POS);
		}
		data->isr_ctx_done = 1u;
	}
}

#ifdef CONFIG_SPI_XEC_GPSPI_DMA
#define XEC_GPSPI_DMA_CFG(i)						\
	.dma_dev = DEVICE_DT_GET(MCHP_XEC_DT_INST_DMA_CTLR(i, tx)),	\
	.tx_dma_request = MCHP_XEC_DT_INST_DMA_TRIGSRC(i, tx),		\
	.tx_dma_channel = MCHP_XEC_DT_INST_DMA_CHANNEL(i, tx),		\
	.rx_dma_request = MCHP_XEC_DT_INST_DMA_TRIGSRC(i, rx),		\
	.rx_dma_channel = MCHP_XEC_DT_INST_DMA_CHANNEL(i, rx),
#else
#define XEC_GPSPI_DMA_CFG(i)
#endif

/* The instance number, i is not related to block ID's rather the
 * order the DT tools process all DT files in a build.
 */
#define XEC_GPSPI_DEVICE(i)						\
									\
	PINCTRL_DT_INST_DEFINE(i);					\
									\
	static void xec_gpspi_irq_connect##i(void) {			\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, rx, irq),		\
			    DT_INST_IRQ_BY_NAME(i, rx, priority),	\
			    xec_gpspi_rxbf_handler,			\
			    DEVICE_DT_INST_GET(i), 0);			\
		irq_enable(DT_INST_IRQ_BY_NAME(i, rx, irq));		\
	}								\
									\
	static struct spi_xec_gpspi_data xec_gpspi_data_##i = {		\
		SPI_CONTEXT_INIT_LOCK(xec_gpspi_data_##i, ctx),		\
		SPI_CONTEXT_INIT_SYNC(xec_gpspi_data_##i, ctx),		\
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(i), ctx)	\
	};								\
	static const struct spi_xec_gpspi_config xec_gpspi_config_##i = { \
		.regs = (struct xec_gpspi_regs *) DT_INST_REG_ADDR(i),	\
		.freqhz = DT_INST_PROP_OR(i, clock_frequency, 0),	\
		.irqtx.irq_num = DT_INST_IRQ_BY_NAME(i, tx, irq),	\
		.irqtx.irq_pri = DT_INST_IRQ_BY_NAME(i, tx, priority),	\
		.irqtx.girq = DT_INST_PROP_BY_IDX(i, girqs, 0),		\
		.irqtx.girq_pos = DT_INST_PROP_BY_IDX(i, girqs, 1),	\
		.irqrx.irq_num = DT_INST_IRQ_BY_NAME(i, rx, irq),	\
		.irqrx.irq_pri = DT_INST_IRQ_BY_NAME(i, rx, priority),	\
		.irqrx.girq = DT_INST_PROP_BY_IDX(i, girqs, 2),		\
		.irqrx.girq_pos = DT_INST_PROP_BY_IDX(i, girqs, 3),	\
		.pcr_idx = DT_INST_PROP_BY_IDX(i, pcrs, 0),		\
		.pcr_pos = DT_INST_PROP_BY_IDX(i, pcrs, 1),		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),		\
		.irq_connect = xec_gpspi_irq_connect##i,		\
		XEC_GPSPI_DMA_CFG(i)					\
	};								\
	PM_DEVICE_DT_INST_DEFINE(i, xec_gpspi_pm_action);		\
	DEVICE_DT_INST_DEFINE(i, &xec_gpspi_init,			\
		PM_DEVICE_DT_INST_GET(i),				\
		&xec_gpspi_data_##i, &xec_gpspi_config_##i,		\
		POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,			\
		&spi_xec_gpspi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_GPSPI_DEVICE)
