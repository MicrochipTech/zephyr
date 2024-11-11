/*
 * Copyright (c) 2024 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_qspi

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_mec5, CONFIG_SPI_LOG_LEVEL);

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <soc.h>
#include <zephyr/irq.h>

#include "spi_context.h"

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_ecia_api.h>
#include <mec_espi_taf.h>
#include <mec_qspi_api.h>

#define MEC5_QSPI_DEBUG_ISR

/* microseconds for busy wait and total wait interval */
#define MEC5_QSPI_WAIT_INTERVAL  8
#define MEC5_QSPI_WAIT_COUNT     64
#define MEC5_QSPI_WAIT_FULL_FIFO 1024

/* Device constant configuration parameters */
struct mec5_qspi_config {
	struct mec_qspi_regs *regs;
	int clock_freq;
	uint32_t cs1_freq;
	uint32_t cs_timing;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
	uint8_t chip_sel;
	uint8_t width; /* 0(half), 1(single), 2(dual), 4(quad) */
	uint8_t ovrc;
	uint8_t ctrl_tap;
	uint8_t clock_tap;
};

#define MEC5_QSPI_XFR_FLAG_BUSY BIT(0)
#define MEC5_QSPI_XFR_FLAG_LDMA BIT(1)
#define MEC5_QSPI_XFR_FLAG_FAST BIT(2)

/* Device run time data */
struct mec5_qspi_data {
	struct spi_context ctx;
	const struct spi_buf *rxb;
	const struct spi_buf *txb;
	size_t rxcnt;
	size_t txcnt;
	volatile uint32_t qstatus;
	volatile uint32_t xfr_flags;
#ifdef MEC5_QSPI_DEBUG_ISR
	volatile uint32_t isr_count;
	volatile uint32_t qctrl;
	volatile uint32_t qien;
	volatile uint32_t qbuf_cnt;
	volatile uint32_t descr0;
	volatile uint32_t descr1;
#endif
	size_t total_tx_size;
	size_t total_rx_size;
	size_t chunk_size;
	uint32_t rxdb;
	uint32_t byte_time_ns;
	uint32_t freq;
	uint32_t operation;
	enum mec_qspi_io iom;
	uint8_t cs;
};

#define SPI_CFG_MODE_MSK (SPI_MODE_CPOL | SPI_MODE_CPHA)
#define SPI_CFG_MODE_0   0
#define SPI_CFG_MODE_3   (SPI_MODE_CPOL | SPI_MODE_CPHA)
#define SPI_NORM_FREQ_HZ MHZ(24)

static const enum mec_qspi_signal_mode mec5_qspi_sig_mode[4] = {
	MEC_SPI_SIGNAL_MODE_0, MEC_SPI_SIGNAL_MODE_1,
	MEC_SPI_SIGNAL_MODE_2, MEC_SPI_SIGNAL_MODE_3 };

static int spi_feature_support(const struct spi_config *config)
{
	/* NOTE: bit(11) is Half-duplex(3-wire) */
	if (config->operation & (SPI_TRANSFER_LSB | SPI_OP_MODE_SLAVE | SPI_MODE_LOOP | BIT(11))) {
		LOG_ERR("Driver does not support LSB first, slave, loop back, or half-duplex");
		return -ENOTSUP;
	}

	if (config->operation & SPI_CS_ACTIVE_HIGH) {
		LOG_ERR("CS active high not supported");
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		LOG_ERR("Word size != 8 not supported");
		return -ENOTSUP;
	}

	return 0;
}

static enum mec_qspi_io lines_to_io(uint32_t opw)
{
	enum mec_qspi_io iom = MEC_QSPI_IO_FULL_DUPLEX;

#if defined(CONFIG_SPI_EXTENDED_MODES)
	switch (opw & SPI_LINES_MASK) {
	case SPI_LINES_DUAL:
		iom = MEC_QSPI_IO_DUAL;
		break;
	case SPI_LINES_QUAD:
		iom = MEC_QSPI_IO_QUAD;
		break;
	default:
		break;
	}
#endif
	return iom;
}

static int mec5_qspi_configure(const struct device *dev, const struct spi_config *config)
{
	const struct mec5_qspi_config *devcfg = dev->config;
	struct mec_qspi_regs *regs = devcfg->regs;
	struct mec5_qspi_data *data = dev->data;
	uint8_t sgm = 0;
	int ret = 0;

	if (!config) {
		return -EINVAL;
	}

	/* chip select */
	if (config->slave >= MEC_QSPI_CS_MAX) {
		LOG_ERR("Invalid chip select [0,1]");
		return -EINVAL;
	}

	data->cs = (uint8_t)(config->slave & 0xffu);
	mec_hal_qspi_cs_select(regs, data->cs);

	data->iom = MEC_QSPI_IO_FULL_DUPLEX;

	if (config->frequency != data->freq) {
		ret = mec_hal_qspi_set_freq(regs, config->frequency);
		if (ret != MEC_RET_OK) {
			return -EINVAL;
		}
		data->freq = config->frequency;
		mec_hal_qspi_byte_time_ns(regs, &data->byte_time_ns);
	}

	if (config->operation == data->operation) {
		return 0;
	}

	data->operation = config->operation;
	ret = spi_feature_support(config);
	if (ret) {
		return ret;
	}

	data->iom = lines_to_io(data->operation);

	ret = mec_hal_qspi_io(regs, data->iom);
	if (ret != MEC_RET_OK) {
		return -EINVAL;
	}

	if (data->operation & SPI_MODE_CPHA) {
		sgm |= BIT(0);
	}
	if (data->operation & SPI_MODE_CPOL) {
		sgm |= BIT(1);
	}
	/* requires QSPI frequency to be programmed first */
	ret = mec_hal_qspi_spi_signal_mode(regs, mec5_qspi_sig_mode[sgm]);
	if (ret != MEC_RET_OK) {
		return -EINVAL;
	}

	data->ctx.config = config;

	return 0;
}

static int mec5_qspi_fd(const struct device *dev, const struct spi_buf_set *tx_bufs,
			const struct spi_buf_set *rx_bufs, bool async)
{
	const struct mec5_qspi_config *devcfg = dev->config;
	struct mec5_qspi_data *data = dev->data;
	struct mec_qspi_regs *regs = devcfg->regs;
	struct spi_context *ctx = &data->ctx;
	size_t xlen = 0, txlen = 0, rxlen = 0, total_tx = 0, total_rx = 0;
	uint32_t qflags = 0, xflags = 0;
	int ret = 0;
	bool hold_cs = (data->operation & SPI_HOLD_ON_CS) ? true : false;

	xflags = MEC_QSPI_ULDMA_FLAG_TX_OVR_VAL_SET(xflags, devcfg->ovrc);
	xflags |= MEC5_QSPI_ULDMA_FLAG_START | MEC5_QSPI_ULDMA_FLAG_IEN;

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1u);

	total_tx = spi_context_total_tx_len(ctx);
	total_rx = spi_context_total_rx_len(ctx);
	data->total_tx_size = total_tx;
	data->total_rx_size = total_rx;

	spi_context_cs_control(ctx, true);

	while (spi_context_rx_on(ctx) || spi_context_tx_on(ctx)) {
		xlen = spi_context_max_continuous_chunk(ctx);
		data->chunk_size = xlen;
		qflags = xflags;

		uint8_t const *txb = ctx->tx_buf;
		uint8_t *rxb = ctx->rx_buf;

		if (txb) {
			qflags |= MEC5_QSPI_ULDMA_FLAG_INCR_TX;
		} else {
			txb = &devcfg->ovrc;
		}

		if (rxb) {
			qflags |= MEC5_QSPI_ULDMA_FLAG_INCR_RX;
		} else {
			rxb = (uint8_t *)&data->rxdb;
		}

		txlen += xlen;
		rxlen += xlen;
		if (!hold_cs) {
			if ((txlen >= total_tx) && (rxlen >= total_rx)) {
				qflags |= MEC5_QSPI_ULDMA_FLAG_CLOSE;
			}
		}

		data->xfr_flags = MEC5_QSPI_XFR_FLAG_LDMA;
		ret = mec_hal_qspi_uldma_fd2(regs, (const uint8_t *)txb, rxb, xlen, qflags);
		if (ret != MEC_RET_OK) {
			ret = -EIO;
			break;
		}

		if (async) {
			return 0;
		}

		ret = spi_context_wait_for_completion(ctx);
		if (ret) {
			break;
		}

		spi_context_update_tx(ctx, 1u, xlen);
		spi_context_update_rx(ctx, 1u, xlen);
	}

	return ret;
}

/* MEC5 QSPI controller maximum number of lines is 4. In dual and quad modes all lines can only be
 * one direction at a time (half-duplex). Other SPI drivers which support dual and quad process
 * SPI buffers as (tx, rx) pairs. Therefore: if tx then do transmit, wait for done then if rx
 * do rx.
 */
static int mec5_qspi_fdq(const struct device *dev, const struct spi_buf_set *tx_bufs,
			 const struct spi_buf_set *rx_bufs, bool async)
{
	const struct mec5_qspi_config *devcfg = dev->config;
	struct mec_qspi_regs *regs = devcfg->regs;
	struct mec5_qspi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint8_t *rxbuf = NULL;
	uint8_t const *txbuf = NULL;
	size_t rxlen = 0, txlen = 0;
	uint32_t qflags = 0, xflags = 0;
	int ret = 0;
	bool hold_cs = (data->operation & SPI_HOLD_ON_CS) ? true : false;

	xflags = MEC_QSPI_ULDMA_FLAG_IOM_SET(data->iom);
	xflags |= MEC_QSPI_ULDMA_FLAG_TX_OVR_VAL_SET(xflags, devcfg->ovrc);
	xflags |= MEC5_QSPI_ULDMA_FLAG_START | MEC5_QSPI_ULDMA_FLAG_IEN;

	data->rxb = NULL;
	data->txb = NULL;
	data->rxcnt = 0;
	data->txcnt = 0;

	if (rx_bufs) {
		data->rxb = rx_bufs->buffers;
		data->rxcnt = rx_bufs->count;
	}

	if (tx_bufs) {
		data->txb = tx_bufs->buffers;
		data->txcnt = tx_bufs->count;
	}

	spi_context_cs_control(ctx, true);

	while ((data->txcnt != 0) || (data->rxcnt != 0)) {
		if (data->txb) {
			txbuf = data->txb->buf;
			txlen = data->txb->len;
		}

		if (data->rxb) {
			rxbuf = data->rxb->buf;
			rxlen = data->rxb->len;
		}

		qflags = xflags;
		if (!hold_cs) {
			if ((data->txcnt <= 1) && (data->rxcnt <= 1)) {
				qflags |= MEC5_QSPI_ULDMA_FLAG_CLOSE;
			}
		}

		data->xfr_flags = MEC5_QSPI_XFR_FLAG_LDMA | MEC5_QSPI_XFR_FLAG_FAST;
		ret = mec_hal_qspi_uldma(regs, txbuf, txlen, rxbuf, rxlen, qflags);
		if (ret != MEC_RET_OK) {
			return -EIO;
		}

		if (async) {
			return 0;
		}

		k_sem_take(&ctx->sync, K_FOREVER);

		if (data->rxb) {
			data->rxb++;
		}
		if (data->rxcnt) {
			data->rxcnt--;
		}
		if (data->txb) {
			data->txb++;
		}
		if (data->txcnt) {
			data->txcnt--;
		}
	}

	return ret;
}
/* #endif */

/* Returns true if the request is suitable for the fast
 * path. Specifically, the bufs are a sequence of:
 *
 * - Zero or more RX and TX buf pairs where each is the same length.
 * - Zero or more trailing RX only bufs
 * - Zero or more trailing TX only bufs
 *
 * Above is from spi_sam.c
 * Modify our test to match and handle all other conditions.
 */
static bool mec5_qspi_is_fd_fast_path(const struct spi_buf_set *txbufs,
				      const struct spi_buf_set *rxbufs)
{
	const struct spi_buf *tx = NULL;
	const struct spi_buf *rx = NULL;
	size_t tx_count = 0;
	size_t rx_count = 0;

	if (txbufs) {
		tx = txbufs->buffers;
		tx_count = txbufs->count;
	}

	if (rxbufs) {
		rx = rxbufs->buffers;
		rx_count = rxbufs->count;
	}

	if (!tx || !rx) {
		return true;
	}

	while ((tx_count != 0) && (rx_count != 0)) {
		if (tx->len != rx->len) {
			return false;
		}

		tx++;
		tx_count--;
		rx++;
		rx_count--;
	}

	return true;
}

static int mec5_qspi_do_xfr(const struct device *dev, const struct spi_config *config,
			    const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs,
			    bool async, spi_callback_t cb, void *userdata)
{
	const struct mec5_qspi_config *devcfg = dev->config;
	struct mec5_qspi_data *devdat = dev->data;
	struct spi_context *ctx = &devdat->ctx;
	int ret = 0;

	if (devdat->xfr_flags & MEC5_QSPI_XFR_FLAG_BUSY) {
		ret = -EBUSY;
	}

	if (!tx_bufs && !rx_bufs) {
		return -EINVAL;
	}

	spi_context_lock(ctx, async, cb, userdata, config);

#ifdef MEC5_QSPI_DEBUG_ISR
	devdat->isr_count = 0;
#endif

	ret = mec5_qspi_configure(dev, config);
	if (ret) {
		goto do_xfr_exit;
	}

	if (devdat->iom == MEC_QSPI_IO_FULL_DUPLEX) {
		if (mec5_qspi_is_fd_fast_path(tx_bufs, rx_bufs)) {
			ret = mec5_qspi_fdq(dev, tx_bufs, rx_bufs, async);
		} else {
			ret = mec5_qspi_fd(dev, tx_bufs, rx_bufs, async);
		}
	} else {
#ifdef CONFIG_SPI_EXTENDED_MODES
		ret = mec5_qspi_fdq(dev, tx_bufs, rx_bufs, async);
#else
		ret = -ENOTSUP;
		goto do_xfr_exit;
#endif
	}

	if (async && !ret) {
		return 0;
	}

	if (ret) {
		mec_hal_qspi_force_stop(devcfg->regs);
	}
do_xfr_exit:
	spi_context_cs_control(ctx, false);
	spi_context_release(ctx, 0);

	return ret;
}

static int mec5_qspi_xfr_check1(const struct spi_config *config)
{
	if (mec_hal_espi_taf_is_activated()) {
		return -EPERM;
	}

	if (!config) {
		return -EINVAL;
	}

	return 0;
}

static int mec5_qspi_xfr_sync(const struct device *dev, const struct spi_config *config,
			      const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	int ret = mec5_qspi_xfr_check1(config);

	if (ret) {
		return ret;
	}

	return mec5_qspi_do_xfr(dev, config, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int mec5_qspi_xfr_async(const struct device *dev, const struct spi_config *config,
			       const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs,
			       spi_callback_t cb, void *userdata)
{
	int ret = mec5_qspi_xfr_check1(config);

	if (ret) {
		return ret;
	}

	return mec5_qspi_do_xfr(dev, config, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif

static int mec5_qspi_release(const struct device *dev, const struct spi_config *config)
{
	struct mec5_qspi_data *qdata = dev->data;
	const struct mec5_qspi_config *cfg = dev->config;
	int ret = 0;

	if (mec_hal_espi_taf_is_activated()) {
		return -EPERM;
	}

	ret = mec_hal_qspi_force_stop(cfg->regs);

	/* increments lock semphare in ctx up to initial limit */
	spi_context_unlock_unconditionally(&qdata->ctx);

	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	return 0;
}

#ifdef CONFIG_SPI_ASYNC

/* Async for non-fast path */
static void mec5_qspi_async_ctx_next(const struct device *dev)
{
	const struct mec5_qspi_config *devcfg = dev->config;
	struct mec_qspi_regs *regs = devcfg->regs;
	struct mec5_qspi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	size_t xlen = 0;
	uint32_t qflags = MEC5_QSPI_ULDMA_FLAG_START | MEC5_QSPI_ULDMA_FLAG_IEN;

	spi_context_update_tx(ctx, 1u, data->chunk_size);
	spi_context_update_rx(ctx, 1u, data->chunk_size);

	if (data->total_tx_size) {
		data->total_tx_size -= data->chunk_size;
	}

	if (data->total_rx_size) {
		data->total_rx_size -= data->chunk_size;
	}

	if (spi_context_rx_on(ctx) || spi_context_tx_on(ctx)) {
		xlen = spi_context_max_continuous_chunk(ctx);
		data->chunk_size = xlen;

		uint8_t const *txb = ctx->tx_buf;
		uint8_t *rxb = ctx->rx_buf;

		if (txb) {
			qflags |= MEC5_QSPI_ULDMA_FLAG_INCR_TX;
		} else {
			txb = &devcfg->ovrc;
		}

		if (rxb) {
			qflags |= MEC5_QSPI_ULDMA_FLAG_INCR_RX;
		} else {
			rxb = (uint8_t *)&data->rxdb;
		}

		if ((data->total_tx_size <= xlen) && (data->total_rx_size <= xlen)) {
			qflags |= MEC5_QSPI_ULDMA_FLAG_CLOSE;
		}
		data->xfr_flags = MEC5_QSPI_XFR_FLAG_LDMA;
		mec_hal_qspi_uldma_fd2(regs, (const uint8_t *)txb, rxb, xlen, qflags);
	} else {
		spi_context_complete(&data->ctx, dev, 0);
	}

}

/* Async for fast path */
static void mec5_qspi_async_next(const struct device *dev)
{
	const struct mec5_qspi_config *devcfg = dev->config;
	struct mec_qspi_regs *regs = devcfg->regs;
	struct mec5_qspi_data *data = dev->data;
	uint32_t xflags = MEC5_QSPI_ULDMA_FLAG_START | MEC5_QSPI_ULDMA_FLAG_IEN;

	if (!(data->xfr_flags & MEC5_QSPI_XFR_FLAG_FAST)) {
		return mec5_qspi_async_ctx_next(dev);
	}

	if (data->rxcnt) {
		data->rxcnt--;
		data->rxb++;
	}
	if (data->txcnt) {
		data->txcnt--;
		data->txb++;
	}

	if (!data->rxcnt && !data->txcnt) {
		spi_context_complete(&data->ctx, dev, 0);
		return;
	}

	if ((data->txcnt <= 1) && (data->rxcnt <= 1)) {
		xflags |= MEC5_QSPI_ULDMA_FLAG_CLOSE;
	}

	data->xfr_flags = MEC5_QSPI_XFR_FLAG_LDMA | MEC5_QSPI_XFR_FLAG_FAST;
	mec_hal_qspi_uldma_fd(regs, data->txb->buf, data->txb->len,
			      data->rxb->buf, data->rxb->len, xflags);
}
#endif /* CONFIG_SPI_ASYNC */

static void mec5_qspi_isr(const struct device *dev)
{
	struct mec5_qspi_data *data = dev->data;
	const struct mec5_qspi_config *devcfg = dev->config;
	struct mec_qspi_regs *regs = devcfg->regs;
	uint32_t hwsts = 0u;
	int status = 0;

#ifdef MEC5_QSPI_DEBUG_ISR
	data->isr_count++;
#endif

	hwsts = mec_hal_qspi_hw_status(regs);
	data->qstatus = hwsts;
#ifdef MEC5_QSPI_DEBUG_ISR
	data->qctrl = regs->CTRL;
	data->qien = regs->INTR_CTRL;
	data->qbuf_cnt = regs->BCNT_STS;
	data->descr0 = regs->DESCR[0];
	data->descr1 = regs->DESCR[1];
	while (hwsts & BIT(4)) {
		;
	}
#endif
	status = mec_hal_qspi_done(regs);

	mec_hal_qspi_intr_ctrl(regs, 0);
	mec_hal_qspi_hw_status_clr(regs, hwsts);
	mec_hal_qspi_girq_clr(regs);

	status = (status == MEC_RET_OK) ? 0 : -EIO;

	if (status == MEC_RET_OK) {
		status = 0;
	} else {
		status = -EIO;
	}

#ifdef CONFIG_SPI_ASYNC
	if (!status && data->ctx.asynchronous) {
		mec5_qspi_async_next(dev);
		return;
	}
#endif
	spi_context_complete(&data->ctx, dev, status);
}

/*
 * Called for each QSPI controller by the kernel during driver load phase
 * specified in the device initialization structure below.
 * Initialize QSPI controller.
 * Initialize SPI context.
 * QSPI will be fully configured and enabled when the transceive API
 * is called.
 */
static int mec5_qspi_init(const struct device *dev)
{
	const struct mec5_qspi_config *devcfg = dev->config;
	struct mec_qspi_regs *regs = devcfg->regs;
	struct mec5_qspi_data *data = dev->data;
	enum mec_qspi_cs cs = MEC_QSPI_CS_0;
	enum mec_qspi_io iom = MEC_QSPI_IO_FULL_DUPLEX;
	enum mec_qspi_signal_mode spi_mode = MEC_SPI_SIGNAL_MODE_0;
	int ret = 0;

	data->cs = 0;
	if (devcfg->chip_sel) {
		data->cs = 1;
		cs = MEC_QSPI_CS_1;
	}

	ret = mec_hal_qspi_init(regs, (uint32_t)devcfg->clock_freq, spi_mode, iom, cs);
	if (ret != MEC_RET_OK) {
		LOG_ERR("QSPI init error (%d)", ret);
		return -EINVAL;
	}

	data->freq = devcfg->clock_freq;
	data->operation = SPI_WORD_SET(8) | SPI_LINES_SINGLE;
	mec_hal_qspi_byte_time_ns(regs, &data->byte_time_ns);

	if (devcfg->cs_timing) {
		ret = mec_hal_qspi_cs_timing(regs, devcfg->cs_timing);
		if (ret != MEC_RET_OK) {
			return -EINVAL;
		}
	}

	if (devcfg->clock_tap || devcfg->ctrl_tap) {
		mec_hal_qspi_tap_select(regs, devcfg->clock_tap, devcfg->ctrl_tap);
	}

	if (devcfg->cs1_freq) {
		ret = mec_hal_qspi_cs1_freq(regs, devcfg->cs1_freq);
		if (ret != MEC_RET_OK) {
			return -EINVAL;
		}
	}

	ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("QSPI pinctrl setup failed (%d)", ret);
	}

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret) {
		LOG_ERR("QSPI cs config failed (%d)", ret);
		return ret;
	}

	if (devcfg->irq_config_func) {
		devcfg->irq_config_func();
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return ret;
}

static const struct spi_driver_api mec5_qspi_driver_api = {
	.transceive = mec5_qspi_xfr_sync,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = mec5_qspi_xfr_async,
#endif
	.release = mec5_qspi_release,
};

#define MEC5_QSPI_CS_TIMING_VAL(a, b, c, d)                                                        \
	(((a) & 0xFu) | (((b) & 0xFu) << 8) | (((c) & 0xFu) << 16) | (((d) & 0xFu) << 24))

#define MEC5_QSPI_TAPS_ADJ_VAL(a, b) (((a) & 0xffu) | (((b) & 0xffu) << 8))

#define MEC5_QSPI_CS_TIMING(i)                                                                     \
	MEC5_QSPI_CS_TIMING_VAL(DT_INST_PROP_OR(i, dcsckon, 6), DT_INST_PROP_OR(i, dckcsoff, 4),   \
				DT_INST_PROP_OR(i, dldh, 6), DT_INST_PROP_OR(i, dcsda, 6))

#define MEC5_QSPI_IRQ_HANDLER_FUNC(id) .irq_config_func = mec5_qspi_irq_config_##id,

#define MEC5_QSPI_IRQ_HANDLER_CFG(id)                                                              \
	static void mec5_qspi_irq_config_##id(void)                                                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(id), DT_INST_IRQ(id, priority), mec5_qspi_isr,            \
			    DEVICE_DT_INST_GET(id), 0);                                            \
		irq_enable(DT_INST_IRQN(id));                                                      \
	}

/* The instance number, i is not related to block ID's rather the
 * order the DT tools process all DT files in a build.
 */
#define MEC5_QSPI_DEVICE(i)                                                                        \
	PINCTRL_DT_INST_DEFINE(i);                                                                 \
	MEC5_QSPI_IRQ_HANDLER_CFG(i)                                                               \
                                                                                                   \
	static struct mec5_qspi_data mec5_qspi_data_##i = {                                        \
		SPI_CONTEXT_INIT_LOCK(mec5_qspi_data_##i, ctx),                                    \
		SPI_CONTEXT_INIT_SYNC(mec5_qspi_data_##i, ctx),                                    \
	};                                                                                         \
	static const struct mec5_qspi_config mec5_qspi_config_##i = {                              \
		.regs = (struct mec_qspi_regs *)DT_INST_REG_ADDR(i),                               \
		.clock_freq = DT_INST_PROP_OR(i, clock_frequency, MHZ(12)),                        \
		.cs1_freq = DT_INST_PROP_OR(i, cs1 - freq, 0),                                     \
		.cs_timing = MEC5_QSPI_CS_TIMING(i),                                               \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),                                         \
		.chip_sel = DT_INST_PROP_OR(i, chip_select, 0),                                    \
		.width = DT_INST_PROP_OR(i, lines, 1),                                             \
		.ovrc = DT_INST_PROP_OR(i, overrun_character, 0),                                  \
		.ctrl_tap = DT_INST_PROP_OR(i, ctrl_tap, 0),                                       \
		.clock_tap = DT_INST_PROP_OR(i, clock_tap, 0),                                     \
		MEC5_QSPI_IRQ_HANDLER_FUNC(i)};                                                    \
	DEVICE_DT_INST_DEFINE(i, &mec5_qspi_init, NULL, &mec5_qspi_data_##i,                       \
			      &mec5_qspi_config_##i, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,        \
			      &mec5_qspi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MEC5_QSPI_DEVICE)
