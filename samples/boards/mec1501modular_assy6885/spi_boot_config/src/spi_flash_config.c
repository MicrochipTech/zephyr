/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/spi.h>
#include <soc.h>
#include "spi_flash_config.h"
#include <logging/log.h>
LOG_MODULE_DECLARE(saf_demo, CONFIG_ESPI_LOG_LEVEL);

/* TODO: Remove if identified that all operations are done by QMSPI or SAF
 * drivers
 */
static QMSPI_Type *regs = (QMSPI_Type *)QMSPI_BASE;

/* TODO: Remove since it's already part of saf_qmspi_init */
#define SPI_24MHZ_CLK_DIVIDE  2

#define SPI_DEV      DT_LABEL(DT_NODELABEL(spi0))

static struct device *spi_dev;
static struct spi_config spi_cfg;
static struct spi_buf rx;
const static struct spi_buf_set rx_bufs = {
	.buffers = &rx,
	.count = 1,
};

static struct spi_buf tx;
const static struct spi_buf_set tx_bufs = {
	.buffers = &tx,
	.count = 1,
};


int qspi_init(void)
{
	LOG_DBG("%s", __func__);
	spi_dev = device_get_binding(SPI_DEV);
	if (!spi_dev) {
		LOG_ERR("Failed to bind %s", SPI_DEV);
		return -1;
	};

	spi_cfg.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER |
			    SPI_LINES_QUAD;
	spi_cfg.frequency = 24000000U;

	return 0;
}

/* Porting of MEC14xx to MEC15xx access HW registers directly */

void qspi_clear_status(void)
{
	LOG_DBG("%s", __func__);
	/* TODO: Can this be done through Zephyr SPI API ? */
	regs->STS = 0xfffffffful;
	regs->MODE =  MCHP_QMSPI_EXE_CLR_FIFOS;
}

void qspi_send(uint8_t *data, uint8_t l)
{
	/* Reset module */
	LOG_DBG("QMSPI mode %x", regs->MODE);
	regs->MODE = MCHP_QMSPI_M_SRST;

	/* Wait until HW clears the bit */
	while (regs->MODE & MCHP_QMSPI_M_SRST) {
	};

	/* Configure SPI clock and SPI slave */
	regs->MODE = SPI_24MHZ_CLK_DIVIDE << MCHP_QMSPI_M_FDIV_POS|
		     0 << MCHP_QMSPI_M_CS_POS | MCHP_QMSPI_M_ACTIVATE;
	LOG_DBG("QMSPI mode %x", regs->MODE);

	regs->CTRL = MCHP_QMSPI_C_XFR_NUNITS(l) |
		     (0 << MCHP_QMSPI_C_DESCR_EN_POS) |
		     MCHP_QMSPI_C_XFR_UNITS_1 |
		     MCHP_QMSPI_C_CLOSE |
		     MCHP_QMSPI_C_RX_EN |
		     MCHP_QMSPI_C_TX_DATA;

	LOG_DBG("QMSPI CTRL %x", regs->CTRL);
	for (int i = 0; i < l; i++) {
		REG8(&regs->TX_FIFO) = data[i];
	}

	/* Finally perform a dummy? operation in quad */
	regs->EXE = 1;
	while (regs->STS & MCHP_QMSPI_STS_TXBF_RO) {
	}

	LOG_DBG("Exit %s", __func__);

}

void qspi_rcv(uint8_t *data, uint8_t len)
{
	for (int i = 0; i < len; i++) {
		data[i] = REG8(&regs->RX_FIFO);
	}
}

void qspi_exit_continuous_mode(uint8_t slave_index)
{
	/* no need to reconfigure gpios in mec15xx baseline */
	LOG_DBG("%s", __func__);

	LOG_DBG("%s CS0 pin control %x", __func__, GPIO_CTRL_REGS->CTRL_0055);
	LOG_DBG("%s CLK pin control %x", __func__, GPIO_CTRL_REGS->CTRL_0056);
	LOG_DBG("%s IO1 pin control %x", __func__, GPIO_CTRL_REGS->CTRL_0224);
	LOG_DBG("%s IO3 pin control %x", __func__, GPIO_CTRL_REGS->CTRL_0016);
	LOG_DBG("%s IO2 pin control %x", __func__, GPIO_CTRL_REGS->CTRL_0227);
	LOG_DBG("%s IO2 pin control %x", __func__, GPIO_CTRL_REGS->CTRL_0223);

	/* Reset module */
	LOG_DBG("QMSPI mode %x", regs->MODE);
	regs->MODE = MCHP_QMSPI_M_SRST;

	/* Wait until HW clears the bit */
	while (regs->MODE & MCHP_QMSPI_M_SRST) {
	};

	/* Configure SPI clock and SPI slave */
	regs->MODE = SPI_24MHZ_CLK_DIVIDE << MCHP_QMSPI_M_FDIV_POS|
		     slave_index << MCHP_QMSPI_M_CS_POS | MCHP_QMSPI_M_ACTIVATE;

	LOG_DBG("QMSPI mode %x", regs->MODE);
	/* Indicate we want to use buffer #12 */
	regs->CTRL = MCHP_QMSPI_C_DESCR_EN | (12 << MCHP_QMSPI_BUF_PTR_POS);
	LOG_DBG("QMSPI CTRL %x", regs->CTRL);

	regs->DESCR[12] = MCHP_QMSPI_C_XFR_NUNITS(1) |
			 MCHP_QMSPI_C_NEXT_DESCR(13) |
			 MCHP_QMSPI_C_XFR_UNITS_1 |
			 MCHP_QMSPI_C_NO_CLOSE |
			 MCHP_QMSPI_C_TX_ONES |
			 MCHP_QMSPI_C_IFM_4X;

	regs->DESCR[13] = MCHP_QMSPI_C_XFR_NUNITS(9) |
			 MCHP_QMSPI_C_DESCR_LAST |
			 MCHP_QMSPI_C_NEXT_DESCR(12) |
			 MCHP_QMSPI_C_XFR_UNITS_1 |
			 MCHP_QMSPI_C_CLOSE |
			 MCHP_QMSPI_C_TX_DIS |
			 MCHP_QMSPI_C_IFM_4X;
	LOG_DBG("QMSPI DESCR[12] %p=%x", &regs->DESCR[12], regs->DESCR[12]);
	LOG_DBG("QMSPI DESCR[13] %p=%x", &regs->DESCR[13], regs->DESCR[13]);
	REG32(&regs->TX_FIFO) = 0xFF;
	regs->EXE = MCHP_QMSPI_EXE_START;

	while (regs->STS & MCHP_QMSPI_STS_TXBF_RO) {
	}

	LOG_DBG("Exit %s", __func__);
}

#define ENABLE_RESET_OPCODE            0x66U
#define RESET_OPCODE                   0x99U
#define FOUR_BYTE_ADDRESS_MODE_OPCODE  0xB7U

int qspi_reset_spi_flash_device(uint8_t slave_index)
{
	int ret;
	uint8_t cmds[] = {
		ENABLE_RESET_OPCODE,
		RESET_OPCODE,
	};

	LOG_DBG("%s", __func__);
	/* TODO: How do we access the second device
	 * Need to check samples to control CS
	 */

/*
	switch (slave_index) {
	case 0:
		spi_cfg.operation = spi_cfg.operation;
		break;
	case 1:
		spi_cfg.operation = spi_cfg.operation;
		break;
	default:
		return -EINVAL;
	}

	for (int i = 0; i < sizeof(cmds); i++) {
		tx.buf = &cmds[i];
		tx.len = 1;

		LOG_DBG("Sending cmd %x", cmds[i]);
		ret = spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);
		if (ret < 0) {
			LOG_ERR("SPI transceive error: %d", ret);
			return ret;
		}
		qspi_send(cmds[i]);
	}
*/


	qspi_send(&cmds[0], 1);
	qspi_send(&cmds[1], 1);

	return 0;
}
