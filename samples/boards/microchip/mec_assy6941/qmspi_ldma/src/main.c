/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define W25Q128_JEDEC_ID				0x001840efU
#define W25Q128JV_JEDEC_ID				0x001870efU
#define SST26VF016B_JEDEC_ID				0x004126bfU
#define SPI_FLASH_JEDEC_ID_MSK				0x00ffffffu

#define SPI_FLASH_READ_JEDEC_ID_OPCODE			0x9fu

#define SPI_FLASH_READ_STATUS1_OPCODE			0x05u
#define SPI_FLASH_READ_STATUS2_OPCODE			0x35u
#define SPI_FLASH_READ_STATUS3_OPCODE			0x15u

#define SPI_FLASH_WRITE_STATUS1_OPCODE			0x01u
#define SPI_FLASH_WRITE_STATUS2_OPCODE			0x31u
#define SPI_FLASH_WRITE_STATUS3_OPCODE			0x11u

#define SPI_FLASH_WRITE_ENABLE_OPCODE			0x06u
#define SPI_FLASH_WRITE_DISABLE_OPCODE			0x04u
#define SPI_FLASH_WRITE_ENABLE_VOLATILE_STS_OPCODE	0x50u

#define SPI_FLASH_READ_SFDP_CMD				0x5au

#define SPI_FLASH_READ_SLOW_OPCODE			0x03u
#define SPI_FLASH_READ_FAST_OPCODE			0x0bu
#define SPI_FLASH_READ_DUAL_OPCODE			0x3bu
#define SPI_FLASH_READ_QUAD_OPCODE			0x6bu

#define SPI_FLASH_ERASE_SECTOR_OPCODE			0x20u
#define SPI_FLASH_ERASE_BLOCK1_OPCODE			0x52u
#define SPI_FLASH_ERASE_BLOCK2_OPCODE			0xd8u
#define SPI_FLASH_ERASE_CHIP_OPCODE			0xc7u

#define SPI_FLASH_PAGE_PROG_OPCODE			0x02u

#define SPI_FLASH_PAGE_SIZE				256
#define SPI_FLASH_SECTOR_SIZE				4096
#define SPI_FLASH_BLOCK1_SIZE				(32 * 1024)
#define SPI_FLASH_BLOCK2_SIZE				(64 * 1024)

#define SPI_FLASH_STATUS1_BUSY_POS			0
#define SPI_FLASH_STATUS1_WEL_POS			1

#define SPI_FLASH_STATUS2_SRL_POS			0
#define SPI_FLASH_STATUS2_QE_POS			1
#define SPI_FLASH_STATUS2_SUS_STS_POS			7

#define SPI_CTRL0_NODE					DT_NODELABEL(qspi0)
#define ZEPHYR_USER_NODE				DT_PATH(zephyr_user)

struct spi_address {
	uint32_t addr;
	uint8_t byte_len;
};

enum spi_flash_read_cmd {
	CMD_SPI_FLASH_READ_SLOW = 0,
	CMD_SPI_FLASH_READ_FAST,
	CMD_SPI_FLASH_READ_DUAL,
	CMD_SPI_FLASH_READ_QUAD,
	CMD_SPI_FLASH_READ_MAX
};

const struct gpio_dt_spec qspi_shd_nwp_dt =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, shd_spi_other_gpios, 0);

const struct gpio_dt_spec qspi_shd_nhold_dt =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, shd_spi_other_gpios, 1);

static const struct device *spi_ctrl_dev = DEVICE_DT_GET(SPI_CTRL0_NODE);

/* SPI endpoint devices on SPI_CTRL0_NODE */
#define SPI_CTRL_MODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8))

#define SPI_EP_M1(node_id) \
	SPI_DT_SPEC_GET(node_id, SPI_CTRL_MODE)

static const struct spi_dt_spec spi_ep_dt_specs[] = {
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(SPI_CTRL0_NODE, SPI_EP_M1, (,))
};

#define SPI_TEST_BUF1_SIZE 256

static uint8_t buf1[SPI_TEST_BUF1_SIZE];

#define SPI_TEST_ADDR1 0xc10000u

#define SPI_FILL_VAL 0x69u
#define SPI_TEST_BUFFER_SIZE (36 * 1024)

struct test_buf {
	union {
		uint32_t w[(SPI_TEST_BUFFER_SIZE) / 4];
		uint16_t h[(SPI_TEST_BUFFER_SIZE) / 2];
		uint8_t b[(SPI_TEST_BUFFER_SIZE)];
	};
};

static struct test_buf tb1;

static bool is_data_buf_filled_with(uint8_t *data, size_t datasz, uint8_t val)
{
	if (!data || !datasz) {
		return false;
	}

	for (size_t i = 0; i < datasz; i++) {
		if (data[i] != val) {
			return false;
		}
	}

	return true;
}

static int spi_flash_format_addr(uint32_t spi_addr, size_t addrsz, uint8_t *buf, size_t bufsz)
{
	if (!buf || (addrsz > 4) || (bufsz < addrsz)) {
		return -EINVAL;
	}

	for (size_t i = 0; i < addrsz; i++) {
		buf[i] = spi_addr >> ((addrsz - i - 1u) * 8);
	}

	return 0;
}

static int spi_flash_read_status(const struct spi_dt_spec *dts, uint8_t read_status_opcode,
                                 uint8_t *status)
{
	struct spi_buf spi_buffers[2];
	uint32_t cmdbuf;
	int err;

	if (!dts || !status) {
		return -EINVAL;
	}

	cmdbuf = read_status_opcode;
	spi_buffers[0].buf = &cmdbuf;
	spi_buffers[0].len = 1;
	spi_buffers[1].buf = status;
	spi_buffers[1].len = 1;

	const struct spi_buf_set tx_bufs = {
		.buffers = spi_buffers,
		.count = 2,
	};

	const struct spi_buf_set rx_bufs = {
		.buffers = spi_buffers,
		.count = 2,
	};

	err = spi_transceive_dt(dts, &tx_bufs, &rx_bufs);

	return err;
}

static int spi_poll_busy(const struct spi_dt_spec *dts, uint32_t timeout_ms)
{
	int err = 0;
	uint32_t ms = 0;
	uint8_t spi_status = 0;

	if (dts == NULL) {
		return -EINVAL;
	}

	spi_status = 0xffu;
	while (timeout_ms) {
		k_busy_wait(2);
		ms += 2u;
		spi_status = 0xffu;
		err = spi_flash_read_status(dts, SPI_FLASH_READ_STATUS1_OPCODE, &spi_status);
		if (err) {
			return err;
		}

		if ((spi_status & BIT(SPI_FLASH_STATUS1_BUSY_POS)) == 0) {
			return 0;
		}
		if (ms > 1000) {
			timeout_ms--;
		}
	}

	return -ETIMEDOUT;
}

/* SPI flash full-duplex write of command opcode, optional command parameters, and optional data */
static int spi_flash_fd_wr_cpd(const struct spi_dt_spec *dts, uint8_t cmd, uint8_t *cmdparams,
                               size_t cmdparamsz, uint8_t *data, size_t datasz)
{
	struct spi_buf tx_spi_buffers[3];
	int bufidx = 0, err = 0;
	uint8_t spicmd;

	if ((dts == NULL) || ((cmdparams == NULL) && (cmdparamsz != 0)) ||
	    ((data == NULL) && (datasz != 0))) {
		return -EINVAL;
	}

	spicmd = cmd;
	tx_spi_buffers[bufidx].buf = &spicmd;
	tx_spi_buffers[bufidx++].len = 1;
	if (cmdparams && cmdparamsz) {
		tx_spi_buffers[bufidx].buf = cmdparams;
		tx_spi_buffers[bufidx++].len = cmdparamsz;
	}

	if (data && datasz) {
		tx_spi_buffers[bufidx].buf = data;
		tx_spi_buffers[bufidx++].len = datasz;
	}

	const struct spi_buf_set tx_bufs = {
		.buffers = tx_spi_buffers,
		.count = bufidx,
	};

	err = spi_transceive_dt(dts, &tx_bufs, NULL);

	return err;
}

static int spi_flash_read_fd_sync(const struct spi_dt_spec *dts, struct spi_address *spi_addr,
				  uint8_t *data, size_t datasz, uint8_t opcode)
{
	uint8_t txdata[8] = {0};
	struct spi_buf spi_bufs[3] = {0};
	int cnt = 0, err = 0;
	size_t param_len = 0;

	if (!dts || !data || !datasz) {
		return -EINVAL;
	}

	txdata[0] = opcode;
	if (spi_addr != 0) {
		if (spi_addr->byte_len > 4) {
			return -EINVAL;
		}
		err = spi_flash_format_addr(spi_addr->addr, spi_addr->byte_len,
					    &txdata[1], sizeof(txdata)-1);
		if (err) {
			return err;
		}
		param_len += spi_addr->byte_len;
	}

	spi_bufs[cnt].buf = txdata;
	spi_bufs[cnt++].len = param_len + 1;

	if (opcode == SPI_FLASH_READ_FAST_OPCODE) {
		spi_bufs[cnt].buf = NULL; /* 8 clocks with output tri-stated */
		spi_bufs[cnt++].len = 1;
	}

	spi_bufs[cnt].buf = data;
	spi_bufs[cnt++].len = datasz;

	const struct spi_buf_set txset = {
		.buffers = &spi_bufs[0],
		.count = cnt,
	};
	const struct spi_buf_set rxset = {
		.buffers = &spi_bufs[0],
		.count = cnt,
	};

	err = spi_transceive_dt(dts, &txset, &rxset);
	if (err) {
		return err;
	}

	return 0;
}

static int spi_flash_erase_region(const struct spi_dt_spec *dts, uint8_t erase_opcode,
                                  struct spi_address *spi_addr, int timeout_ms)
{
	int err = 0;
	uint8_t cmdparams[4] = {0};

	if ((dts == NULL) || (spi_addr == NULL)) {
		return -EINVAL;
	}

	err = spi_flash_format_addr(spi_addr->addr, spi_addr->byte_len,
				    cmdparams, sizeof(cmdparams));
	if (err) {
		return err;
	}

	err = spi_flash_fd_wr_cpd(dts, erase_opcode, cmdparams,
				  spi_addr->byte_len, NULL, 0);
	if (err) {
		return err;
	}

	err = spi_poll_busy(dts, timeout_ms);

	return err;
}

static int read_jedec_id(const struct spi_dt_spec *dts, uint32_t *jid)
{
	int err = 0, ret = 0;
	uint32_t val = 0;

	if ((dts == NULL) || (jid == NULL)) {
		return -EINVAL;
	}

	val = 0;
	err = spi_flash_read_fd_sync(dts, NULL, (uint8_t *)&val, 3u,
				     SPI_FLASH_READ_JEDEC_ID_OPCODE);
	if (err != 0) {
		LOG_ERR("Read JEDEC_ID error (%d)", err);
		return err;
	}

	if (jid != NULL) {
		*jid = val;
	}

	LOG_INF("JEDEC ID = 0x%08x", val);

	if (val == W25Q128_JEDEC_ID) {
		LOG_INF("W25Q128 16Mbyte SPI flash");
	} else if (val == W25Q128JV_JEDEC_ID) {
		LOG_INF("W25Q128JV 16Mbyte SPI flash");
	} else if (val == SST26VF016B_JEDEC_ID) {
		LOG_INF("SST26VF016B 16MByte SPI flash");
	} else {
		LOG_INF("Unknown SPI flash device");
		ret = -ENODEV;
	}

	return ret;
}

int main(void)
{
	int err, num_sectors, num_pages, offset;
	uint32_t jedec_id, qe_bm;
	size_t n, spi_len;
	struct spi_address spi_addr;
	uint8_t spi_status1, spi_status2;

	memset(buf1, 0, sizeof(buf1));
	memset((void *)&tb1, 0x55, sizeof(tb1));

	if (device_is_ready(spi_ctrl_dev) == false) {
		LOG_ERR("SPI Controller driver is not ready!\n");
		return -1;
	}

	err = gpio_pin_configure_dt(&qspi_shd_nwp_dt, GPIO_OUTPUT_HIGH);
	if (err != 0) {
		LOG_ERR("Configure QSPI nWP pin GPIO output drive high error (%d)", err);
		return -2;
	}

	err = gpio_pin_configure_dt(&qspi_shd_nhold_dt, GPIO_OUTPUT_HIGH);
	if (err != 0) {
		LOG_ERR("Configure QSPI nHOLD pin GPIO output drive high error (%d)", err);
		return -3;
	}

	qe_bm = 0;
	for (n = 0; n < ARRAY_SIZE(spi_ep_dt_specs); n++) {
		const struct spi_dt_spec *pdts = &spi_ep_dt_specs[n];

		LOG_INF("Read JEDEC ID, STATUS1, and STATUS2 from SPI endpoint %u", n);

		jedec_id = 0x55555555u;
		err = read_jedec_id(pdts, &jedec_id);
		if (err != 0) {
			LOG_ERR("JEDEC ID read error (%d)", err);
		}

		spi_status1 = 0xffu;
		err = spi_flash_read_status(pdts, SPI_FLASH_READ_STATUS1_OPCODE, &spi_status1);
		if (err != 0) {
			LOG_ERR("Read SPI flash Status1 error (%d)", err);
		}

		LOG_INF("SPI Flash Status1 = 0x%02x", spi_status1);

		spi_status2 = 0xffu;
		err = spi_flash_read_status(pdts, SPI_FLASH_READ_STATUS2_OPCODE, &spi_status2);
		if (err != 0) {
			LOG_ERR("Read SPI flash STATUS2 error (%d)", err);
		}

		LOG_INF("SPI Flash Status2 = 0x%02x", spi_status2);
		if ((spi_status2 & BIT(SPI_FLASH_STATUS2_QE_POS)) != 0) {
			qe_bm |= BIT(n);
			LOG_INF("Quad-Enable bit is set. WP# and HOLD# are IO[2] and IO[3]");
		}
		if ((spi_status2 & BIT(SPI_FLASH_STATUS2_SRL_POS)) != 0) {
			LOG_INF("SPI Flash Status registers are locked!");
		}
		if ((spi_status2 & BIT(SPI_FLASH_STATUS2_SUS_STS_POS)) != 0) {
			LOG_INF("SPI Flash is in Suspend state");
		}

		spi_addr.addr = SPI_TEST_ADDR1;
		spi_addr.byte_len = 3u;
		num_sectors = 1u;

		if (SPI_TEST_BUFFER_SIZE > SPI_FLASH_SECTOR_SIZE) {
			num_sectors = SPI_TEST_BUFFER_SIZE / SPI_FLASH_SECTOR_SIZE;
		}

		LOG_INF("Exercising %u sectors", num_sectors);

		LOG_INF("Erase sectors: Send SPI flash Write-Enable and Erase commands");
		for (int i = 0; i < num_sectors; i++) {
			err = spi_flash_fd_wr_cpd(pdts, SPI_FLASH_WRITE_ENABLE_OPCODE,
						  NULL, 0, NULL, 0);
			if (err != 0) {
				LOG_ERR("ERROR: SPI flash Write-Enable: error (%d)", err);
			}

			err = spi_flash_erase_region(pdts, SPI_FLASH_ERASE_SECTOR_OPCODE,
						     &spi_addr, 1000u);
			if (err != 0) {
				LOG_ERR("SPI flash Erase sector error (%d)", err);
			}

			spi_addr.addr += SPI_FLASH_SECTOR_SIZE;
		}

		LOG_INF("Read and check erased sectors using Read 1-1-1-0");

		spi_addr.addr = SPI_TEST_ADDR1;
		spi_addr.byte_len = 3u;

		for (int i = 0; i < num_sectors; i++) {
			memset(&tb1.b[0], 0x55, SPI_FLASH_SECTOR_SIZE);

			err = spi_flash_read_fd_sync(pdts, &spi_addr, &tb1.b[0],
						     SPI_FLASH_SECTOR_SIZE,
						     SPI_FLASH_READ_SLOW_OPCODE);
			if (err != 0) {
				LOG_ERR("Read slow error (%d)", err);
			}

			if (is_data_buf_filled_with(&tb1.b[0], SPI_FLASH_SECTOR_SIZE, 0xffu)) {
				LOG_INF("Sector at 0x%08x is erased", spi_addr.addr);
			} else {
				LOG_ERR("FAIL: sector at 0x%08x is not erased", spi_addr.addr);
			}

			spi_addr.addr += SPI_FLASH_SECTOR_SIZE;
		}

		LOG_INF("Fill buffers for %d sectors", num_sectors);
		for (int i = 0; i < SPI_TEST_BUFFER_SIZE; i++) {
			tb1.b[i] = SPI_FILL_VAL;
		}

		LOG_INF("Program erased sectors");

		num_pages = SPI_TEST_BUFFER_SIZE / SPI_FLASH_PAGE_SIZE;
		offset = 0;

		spi_addr.addr = SPI_TEST_ADDR1;
		spi_addr.byte_len = 3u;

		for (int i = 0; i < num_pages; i++) {
			err = spi_flash_format_addr(spi_addr.addr, spi_addr.byte_len,
			                            buf1, sizeof(buf1));
			if (err != 0) {
				LOG_ERR("ERROR: format SPI address: (%d)", err);
			}

			err = spi_flash_fd_wr_cpd(pdts, SPI_FLASH_WRITE_ENABLE_OPCODE,
						  NULL, 0, NULL, 0);
			if (err != 0) {
				LOG_ERR("ERROR: transmit SPI flash Write-Enable: (%d)", err);
			}

			err = spi_flash_fd_wr_cpd(pdts, SPI_FLASH_PAGE_PROG_OPCODE,
						  buf1, spi_addr.byte_len, &tb1.b[offset],
						  SPI_FLASH_PAGE_SIZE);
			if (err != 0) {
				LOG_ERR("ERROR: transmit SPI Page-Program: (%d)", err);
			}

			err = spi_poll_busy(pdts, 100);
			if (err != 0) {
				LOG_ERR("ERROR: Poll SPI busy status: (%d)", err);
				return err;
			}

			spi_addr.addr += SPI_FLASH_PAGE_SIZE;
			offset += SPI_FLASH_PAGE_SIZE;
		}

		LOG_INF("Read sectors and check data");

		memset(&tb1.b[0], 0, sizeof(tb1));

		spi_addr.addr = SPI_TEST_ADDR1;
		spi_addr.byte_len = 3u;
		spi_len = num_pages * SPI_FLASH_PAGE_SIZE;

		LOG_INF("Read %u bytes from 0x%08x using SPI Read 1-1-1-0", spi_len,
		        spi_addr.addr);

		err = spi_flash_read_fd_sync(pdts, &spi_addr, &tb1.b[0], spi_len,
					     SPI_FLASH_READ_SLOW_OPCODE);
		if (err != 0) {
			LOG_ERR("Read slow error (%d)", err);
		}

		if (is_data_buf_filled_with(&tb1.b[0], spi_len, SPI_FILL_VAL)) {
			LOG_INF("PASS: Data matches");
		} else {
			LOG_ERR("FAIL: Data mismatch");
		}

		memset(&tb1.b[0], 0, sizeof(tb1));

		LOG_INF("Read %u bytes from 0x%08x using SPI Read 1-1-1-8", spi_len,
		        spi_addr.addr);

		err = spi_flash_read_fd_sync(pdts, &spi_addr, &tb1.b[0], spi_len,
					     SPI_FLASH_READ_FAST_OPCODE);
		if (err != 0) {
			LOG_ERR("Read fast error (%d)", err);
		}

		if (is_data_buf_filled_with(&tb1.b[0], spi_len, SPI_FILL_VAL)) {
			LOG_INF("PASS: Data matches");
		} else {
			LOG_ERR("FAIL: Data mismatch");
		}
	}

	LOG_INF("Program End");

	return 0;
}
