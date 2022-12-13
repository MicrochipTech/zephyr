/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

/**
 * @file Sample app using the Microchip GPSPI driver with a W25Q128 SPI flash.
 */

/* NOTE 1 Requires Zephyr SPI driver has support for CONFIG_SPI_EXTENDED_MODES
 * and SPI_LINES_DUAL/SPI_LINES_QUAD. Microchip XEC QMSPI controller supports
 * dual and quad. XEC GPSPI controller can do dual (no quad).
 */

#define W25Q128_JEDEC_ID 0x001840efU
#define W25Q128JV_JEDEC_ID 0x001870efU

#define SPI_FLASH_READ_JEDEC_ID_CMD	0x9fu

#define SPI_FLASH_READ_STATUS1_CMD	0x05u
#define SPI_FLASH_READ_STATUS2_CMD	0x35u
#define SPI_FLASH_READ_STATUS3_CMD	0x15u

#define SPI_FLASH_WRITE_STATUS1_CMD	0x01u
#define SPI_FLASH_WRITE_STATUS2_CMD	0x31u
#define SPI_FLASH_WRITE_STATUS3_CMD	0x11u

#define SPI_FLASH_WRITE_ENABLE_CMD	0x06u
#define SPI_FLASH_WRITE_DISABLE_CMD	0x04u

#define SPI_FLASH_READ_SFDP_CMD		0x5au

#define SPI_FLASH_READ_SLOW_CMD		0x03u
#define SPI_FLASH_READ_FAST_CMD		0x0bu
#define SPI_FLASH_READ_DUAL_CMD		0x3bu
#define SPI_FLASH_READ_QUAD_CMD		0x3bu

#define SPI_FLASH_ERASE_SECTOR_CMD	0x20u
#define SPI_FLASH_ERASE_BLOCK1_CMD	0x52u
#define SPI_FLASH_ERASE_BLOCK2_CMD	0xd8u
#define SPI_FLASH_ERASE_CHIP_CMD	0xc7u

#define SPI_FLASH_PAGE_PROG_CMD		0x02u

#define SPI_FLASH_PAGE_SIZE		256
#define SPI_FLASH_SECTOR_SIZE 		4096
#define SPI_FLASH_BLOCK1_SIZE 		(32 * 1024)
#define SPI_FLASH_BLOCK2_SIZE 		(64 * 1024)

#define SPI_FLASH_STATUS1_BUSY_POS	0
#define SPI_FLASH_STATUS1_WEL_POS	1

#define SPI_FLASH_STATUS2_SRL_POS	0
#define SPI_FLASH_STATUS2_QE_POS	1
#define SPI_FLASH_STATUS2_SUS_STS_POS	7

#define SPI1_NODE	DT_NODELABEL(spi1)


struct spi_flash_info {
	uint32_t jedec_id;
	uint32_t size_in_bytes;
	char *name;
};

static const struct spi_flash_info flash_table[] = {
	{

		.jedec_id = W25Q128_JEDEC_ID,
		.size_in_bytes = (16u * 1024u * 1024u),
		.name = "W25Q128FV",
	},
	{
		.jedec_id = W25Q128JV_JEDEC_ID,
		.size_in_bytes = (16u * 1024u * 1024u),
		.name = "W25Q128JV-DTR",
	}
};

struct spi_flash_info const *lookup_spi_flash(uint32_t jedec_id)
{
	for (size_t i = 0; i < ARRAY_SIZE(flash_table); i++) {
		if (flash_table[i].jedec_id == jedec_id) {
			return &flash_table[i];
		}
	}

	return NULL;
}

static int spi_flash_format_cmd_addr(uint8_t *buf, size_t bufsz, uint8_t cmd, uint32_t spi_addr)
{
	size_t bsz = bufsz;

	if (!buf || !bufsz) {
		return -EINVAL;
	}

	buf[0] = cmd;
	if (bsz > 5) {
		bsz = 5;
	}
	for (size_t i = bsz - 1u; i >= 1u; i--) {
		buf[i] = spi_addr;
		spi_addr >>= 8;
	}

	return 0;
}

/* Transmit single byte SPI flash command Read JEDEC ID.
 * SPI TX buffer set contains 2 buffers
 *	TX buffer 0: len=1 contains SPI Read JEDEC command byte
 *	TX buffer 1: NULL. Driver transmits 0's
 * SPI RX buffer set contains 2 buffers
 *	RX buffer 0: NULL, data captured during command transmit is discarded
 *	RX buffer 1: len=4, used to store JEDEC ID.
 */
static int spi_flash_read_id(const struct device *spi,
			     struct spi_config *spi_cfg,
			     uint32_t *jedec_id)
{
	int err;
	uint32_t txdata;
	struct spi_buf sb[2];

	if (!spi || !spi_cfg || !jedec_id) {
		return -EINVAL;
	}

	txdata = SPI_FLASH_READ_JEDEC_ID_CMD;
	sb[0].buf = &txdata;
	sb[0].len = 1;
	sb[1].buf = jedec_id;
	sb[1].len = 4;

	const struct spi_buf_set txs = {
		.buffers = sb,
		.count = 2,
	};

	const struct spi_buf_set rxs = {
		.buffers = sb,
		.count = 2,
	};

	err = spi_transceive(spi, spi_cfg, &txs, &rxs);

	return err;
}

static int spi_flash_read_status(const struct device *spi,
				 struct spi_config *spi_cfg,
				 uint8_t spi_rd_status_cmd,
				 uint8_t *spi_status)
{
	int err;
	uint32_t txdata;
	struct spi_buf sb[2];

	if (!spi || !spi_cfg || !spi_status) {
		return -EINVAL;
	}

	txdata = spi_rd_status_cmd;
	sb[0].buf = &txdata;
	sb[0].len = 1;
	sb[1].buf = spi_status;
	sb[1].len = 1;

	const struct spi_buf_set txs = {
		.buffers = sb,
		.count = 2,
	};

	const struct spi_buf_set rxs = {
		.buffers = sb,
		.count = 2,
	};

	err = spi_transceive(spi, spi_cfg, &txs, &rxs);

	return err;
}

static int spi_flash_send_cmd(const struct device *spi,
			      struct spi_config *spi_cfg,
			      uint8_t spi_cmd)
{
	int err;
	uint32_t txdata;
	struct spi_buf sb[2];

	if (!spi || !spi_cfg) {
		return -EINVAL;
	}

	txdata = spi_cmd;
	sb[0].buf = &txdata;
	sb[0].len = 1;
	sb[1].buf = NULL;
	sb[1].len = 0;

	const struct spi_buf_set txs = {
		.buffers = sb,
		.count = 1,
	};

	const struct spi_buf_set rxs = {
		.buffers = sb,
		.count = 2,
	};

	err = spi_transceive(spi, spi_cfg, &txs, &rxs);

	return err;
}

/* Read data from given SPI address using either SPI read slow (1-1-1-0)
 * or SPI fast read (1-1-1-8). Where (c,a,d,t) is:
 * c = command phase number of transmit pins
 * a = address phase number of pins
 * d = data phase number of pins
 * t = number of SPI clocks with output pin(s) tri-stated after address phase
 * We implement tri-state SPI clocks by adding an RX spi_buf for one byte as
 * the first RX spi_buf. The driver will switch pin(s) direction for RX and not
 * drive the output pin.
 */
static int spi_flash_read(const struct device *spi,
			  struct spi_config *spi_cfg, uint8_t cmd,
			  uint32_t spi_addr, uint8_t *data, size_t datasz)
{
	int err = 0;
	uint32_t temp = 0;
	size_t rx_count = 0;
	uint8_t txdata[4] = {0};
	struct spi_buf sb[3] = {0};

	if (!spi || !spi_cfg || !data || !datasz) {
		return -EINVAL;
	}

	err = spi_flash_format_cmd_addr(txdata, 4u, cmd, spi_addr);
	if (err) {
		return err;
	}

	if (cmd == SPI_FLASH_READ_SLOW_CMD) {
		sb[0].buf = &txdata;
		sb[0].len = 4;
		sb[1].buf = data;
		sb[1].len = datasz;
		rx_count = 2u;
	} else if (cmd == SPI_FLASH_READ_FAST_CMD) {
		sb[0].buf = &txdata;
		sb[0].len = 4;
		sb[1].buf = &temp;
		sb[1].len = 1;
		sb[2].buf = data;
		sb[2].len = datasz;
		rx_count = 3u;
	} else {
		return -EINVAL;
	}

	const struct spi_buf_set txs = {
		.buffers = sb,
		.count = rx_count
	};

	const struct spi_buf_set rxs = {
		.buffers = sb,
		.count = rx_count
	};

	err = spi_transceive(spi, spi_cfg, &txs, &rxs);

	return err;
}

static int spi_flash_read_dual(const struct device *spi, struct spi_config *spi_cfg, uint8_t cmd,
			       uint32_t spi_addr, uint8_t *data, size_t datasz)
{
	int err;
	uint32_t temp;
	uint8_t txbuf[4];
	struct spi_config fd_cfg;
	struct spi_config dual_cfg;
	struct spi_buf tb[1];
	struct spi_buf rb[2];

	if (!spi || !spi_cfg || !data || !datasz) {
		return -EINVAL;
	}

	memset(txbuf, 0, sizeof(txbuf));
	memcpy(&fd_cfg, spi_cfg, sizeof(struct spi_config));
	memcpy(&dual_cfg, spi_cfg, sizeof(struct spi_config));
	fd_cfg.operation |= SPI_HOLD_ON_CS;
	dual_cfg.operation = (dual_cfg.operation & ~SPI_LINES_MASK) | SPI_LINES_DUAL;

	/* 1. Transmit opcode || 24-bit SPI address || 1-byte(tri-state clocks)
	 *    using full-duplex mode.
	 * 2. Change config to dual to read data
	 */

	/* assuming 24-bit SPI address */
	txbuf[0] = cmd;
	txbuf[1] = (uint8_t)(spi_addr >> 16);
	txbuf[2] = (uint8_t)(spi_addr >> 8);
	txbuf[3] = (uint8_t)(spi_addr >> 0);

	/* Try. One TX buffer for command and address.
	 * Two RX buffers. First is 1 byte which is the tri-state clocks.
	 * Second is data.
	 */
	temp = 0;

	tb[0].buf = txbuf;
	tb[0].len = 4;

	const struct spi_buf_set txs = {
		.buffers = tb,
		.count = 1,
	};

	err = spi_transceive(spi, &fd_cfg, &txs, NULL);
	if (err) {
		return err;
	}

	/* GPSPI driver ignores spi_buf's with NULL pointer or data length == 0.
	 * Dual opcode 0x3b requires 8 SPI clocks with I/O tri-stated after
	 * SPI address is clocked out. Using TX for the 8 clocks results in
	 * GPSPI controller driving the MOSI pin and at some time during the
	 * 8 clocks the SPI flash will start driving the MOSI pin. We do the 8
	 * tri-state clocks using RX. In dual mode GPSPI issues 4 clocks per
	 * byte. Our first spi_buf uses a temp buffer and length = 2 bytes.
	 */
	rb[0].buf = &temp;
	rb[0].len = 2;
	rb[1].buf = data;
	rb[1].len = datasz;

	const struct spi_buf_set rxs_dual = {
		.buffers = rb,
		.count = 2,
	};

	err = spi_transceive(spi, &dual_cfg, NULL, &rxs_dual);

	return err;
}

static int spi_flash_erase(const struct device *spi, struct spi_config *spi_cfg,
			   uint8_t cmd, uint32_t spi_addr)
{
	int err;
	uint8_t txbuf[4];
	struct spi_buf sb[1];

	if (!spi || !spi_cfg) {
		return -EINVAL;
	}

	sb[0].buf = txbuf;
	sb[0].len = 4;

	switch (cmd) {
	case SPI_FLASH_ERASE_SECTOR_CMD:
		spi_addr &= ~0xffu; /* align on page boundary */
		break;
	case SPI_FLASH_ERASE_BLOCK1_CMD: /* 32kb */
		spi_addr &= ~0x7fffu;
		break;
	case SPI_FLASH_ERASE_BLOCK2_CMD: /* 64kb */
		spi_addr &= ~0xffffu;
		break;
	case SPI_FLASH_ERASE_CHIP_CMD:
		sb[0].len = 1u; /* command only */
		break;
	default:
		return -EINVAL;
	}

	txbuf[0] = cmd;
	txbuf[1] = (spi_addr >> 16);
	txbuf[2] = (spi_addr >> 8);
	txbuf[3] = spi_addr;

	const struct spi_buf_set txs = {
		.buffers = sb,
		.count = 1,
	};

	err = spi_transceive(spi, spi_cfg, &txs, NULL);

	return err;
}

static int spi_flash_page_prog(const struct device *spi, struct spi_config *spi_cfg,
			       uint32_t spi_addr, uint8_t *data, size_t datasz)
{
	int err;
	uint8_t txbuf[4];
	struct spi_buf sb[2];

	if (!spi || !spi_cfg || !data || !datasz) {
		return -EINVAL;
	}

	spi_addr &= ~0xffu; /* align on page boundary */
	if (datasz > 256U) {
		datasz = 256U;
	}

	txbuf[0] = 0x02u;
	txbuf[1] = (spi_addr >> 16);
	txbuf[2] = (spi_addr >> 8);
	txbuf[3] = spi_addr;

	sb[0].buf = txbuf;
	sb[0].len = 4;
	sb[1].buf = data;
	sb[1].len = datasz;

	const struct spi_buf_set txs = {
		.buffers = sb,
		.count = 2,
	};

	err = spi_transceive(spi, spi_cfg, &txs, NULL);

	return err;
}

static int erase_region(const struct device *spi, struct spi_config *spi_cfg,
			uint8_t cmd, uint32_t spi_addr)
{
	int err;
	uint8_t spi_status;

	err = spi_flash_send_cmd(spi, spi_cfg, 0x06U);
	if (err) {
		return err;
	}

	err = spi_flash_erase(spi, spi_cfg, cmd, spi_addr);
	if (err) {
		return err;
	}

	do {
		k_busy_wait(10);
		spi_status = 0xffu;
		err = spi_flash_read_status(spi, spi_cfg, 0x05U, &spi_status);
		if (err) {
			return err;
		}
	} while (spi_status & BIT(SPI_FLASH_STATUS1_BUSY_POS));

	return 0;
}

static int program_test_data(const struct device *spi, struct spi_config *spi_cfg,
			     uint32_t spi_addr, uint8_t *data, size_t datasz)
{
	int err;
	uint32_t start, end, end_page, len;
	uint8_t spi_status;

	if (!spi || !spi_cfg || !data || !datasz) {
		return -EINVAL;
	}

	err = 0;
	start = spi_addr;
	end = spi_addr + datasz;
	while (start < end) {
		if ((end - start) > 256U) {
			end_page = (start + 256U) & ~0xffu;
			len = end_page - start;
		} else {
			len = end - start;
		}

		/* 1. Send Write Enable */
		err = spi_flash_send_cmd(spi, spi_cfg, 0x06U);
		if (err) {
			return err;
		}

		/* 2. Send Page Program for each page */
		err = spi_flash_page_prog(spi, spi_cfg, start, data, len);
		if (err) {
			return err;
		}

		/* 3. Loop reading status1 until busy clears */
		do {
			k_busy_wait(10);
			spi_status = 0xffu;
			err = spi_flash_read_status(spi, spi_cfg, 0x05U, &spi_status);
			if (err) {
				return err;
			}
		} while (spi_status & BIT(SPI_FLASH_STATUS1_BUSY_POS));

		data += len;
		start += len;
	}

	return err;
}

static int buf_contains(uint8_t *data, size_t datasz, uint8_t val)
{
	if (!data || !datasz) {
		return 0;
	}

	for (size_t i = 0; i < datasz; i++) {
		if (data[i] != val) {
			return 0;
		}
	}

	return 1;
}

static void pr_data_buf(uint8_t *data, size_t datasz)
{
	printk("Data @ 0x%08x len = %u\n", (uint32_t)data, datasz);

	if (!data || !datasz) {
		return;
	}

	int count = 0;
	for (size_t n = 0; n < datasz; n++) {
		printk("0x%02x, ", data[n]);
		if (++count == 8) {
			count = 0;
			printk("\n");
		}
	}
}

#ifdef CONFIG_SPI_ASYNC

struct spi_user_data {
	volatile int done;
	volatile int result;
};

static volatile int spi_cb_called;

void spi_callback(const struct device *dev, int result, void *data)
{
	if (data) {
		((struct spi_user_data *)data)->done = 1;
		((struct spi_user_data *)data)->result = result;
	}

	spi_cb_called++;
}

static int spi_flash_read_async(const struct device *spi,
				struct spi_config *spi_cfg, uint8_t cmd,
				uint32_t spi_addr, uint8_t *data, size_t datasz)
{
	int err;
	uint8_t txdata1[4] = {0};
	struct spi_buf sb[2] = {0};
	struct spi_user_data ud = {0};
	uint64_t wait_count = 0;

	if (!spi || !spi_cfg || !data || !datasz) {
		return -EINVAL;
	}

	/* assuming 24-bit SPI address */
	err = spi_flash_format_cmd_addr(txdata1, 4u, cmd, spi_addr);
	if (err) {
		return err;
	}

	sb[0].buf = &txdata1;
	sb[0].len = 4;
	sb[1].buf = data;
	sb[1].len = datasz;

	const struct spi_buf_set txs = {
		.buffers = sb,
		.count = 2,
	};

	const struct spi_buf_set rxs = {
		.buffers = sb,
		.count = 2,
	};

	err = spi_transceive_cb(spi, spi_cfg, &txs, &rxs, spi_callback, (void *)&ud);
	if (err == 0) {
		/* we could spin on ud.done */
		while (!spi_cb_called) {
			wait_count++;
		}
		printk("Asynchronous read completed after %llu spin loops\n", wait_count);
		printk("Callback done = %u  result = %u\n", ud.done, ud.result);
	}

	return err;
}

#endif /* CONFIG_SPI_ASYNC */

static uint8_t testbuf1[4096];

void main(void)
{
	const struct device *spi;
	const struct spi_flash_info *flash;
	struct spi_config spi_cfg = {0};
	int err;
	uint32_t jedec_id, spi_addr;
	size_t datalen, n;
	uint8_t cmd;
	uint8_t spi_status;

	printk("Microchip XEC GPSPI example application\n");

	spi = DEVICE_DT_GET(DT_ALIAS(gpspi));
	if (!device_is_ready(spi)) {
		printk("SPI device %s is not ready\n", spi->name);
		return;
	}

	/* DT specifying GPIO pin for GPSPI CS# ? */
#if DT_NODE_HAS_PROP(SPI1_NODE, cs_gpios)
	struct spi_cs_control cs_ctrl = (struct spi_cs_control){
		.gpio = GPIO_DT_SPEC_GET(SPI1_NODE, cs_gpios),
		.delay = 0u,
	};

	spi_cfg.cs = &cs_ctrl;
#endif
	spi_cfg.operation = SPI_WORD_SET(8);
	spi_cfg.frequency = MHZ(4);

	printk("Read SPI flash JEDEC ID\n");

	jedec_id = 0x55555555U;
	err = spi_flash_read_id(spi, &spi_cfg, &jedec_id);
	if (err) {
		printk("Could not read SPI flash ID. error = %d\n", err);
		return;
	}

	printk("Read SPI device JEDEC_ID as 0x%08x\n", jedec_id);

	flash = lookup_spi_flash(jedec_id);
	if (!flash) {
		printk("Unknown SPI flash device. Test aborting.\n");
		return;
	}

	printk("SPI Flash is %s\n", flash->name);

	printk("Read SPI flash status1\n");
	spi_status = 0x55u;
	cmd = SPI_FLASH_READ_STATUS1_CMD;
	err = spi_flash_read_status(spi, &spi_cfg, cmd, &spi_status);
	if (err) {
		printk("Could not read SPI flash status register %d. error = %d\n",
			spi_status, err);
		return;
	}

	printk("SPI flash status1 = 0x%02x\n", spi_status);

	printk("Send SPI flash write enable\n");
	cmd = SPI_FLASH_WRITE_ENABLE_CMD;
	err = spi_flash_send_cmd(spi, &spi_cfg, cmd);
	if (err) {
		printk("Send SPI flash command 0x%02x failed error %d\n", cmd, err);
		return;
	}

	printk("Read SPI flash status1\n");
	spi_status = 0x55u;
	cmd = SPI_FLASH_READ_STATUS1_CMD;
	err = spi_flash_read_status(spi, &spi_cfg, cmd, &spi_status);
	if (err) {
		printk("Could not read SPI flash status register %d. error = %d\n",
			spi_status, err);
		return;
	}

	printk("SPI flash status1 = 0x%02x\n", spi_status);

	printk("Send SPI flash write disable\n");
	cmd = SPI_FLASH_WRITE_DISABLE_CMD;
	err = spi_flash_send_cmd(spi, &spi_cfg, cmd);
	if (err) {
		printk("Send SPI flash command 0x%02x failed error %d\n", cmd, err);
		return;
	}

	printk("Read SPI flash status1\n");
	spi_status = 0x55u;
	cmd = SPI_FLASH_READ_STATUS1_CMD;
	err = spi_flash_read_status(spi, &spi_cfg, cmd, &spi_status);
	if (err) {
		printk("Could not read SPI flash status register %d. error = %d\n",
			spi_status, err);
		return;
	}

	printk("SPI flash status1 = 0x%02x\n", spi_status);

	memset(testbuf1, 0xAAu, sizeof(testbuf1));
	spi_addr = 0u;
	datalen = 16U;
	cmd = SPI_FLASH_READ_SLOW_CMD;
	printk("Read %u bytes at SPI offset 0x%08x using cmd 0x%02x\n", datalen, spi_addr, cmd);

	err = spi_flash_read(spi, &spi_cfg, cmd, spi_addr, testbuf1, datalen);
	if (err) {
		printk("SPI flash read: error=%d\n", err);
		return;
	}

	pr_data_buf(testbuf1, datalen);

	for (n = 0; n < datalen; n++) {
		if (testbuf1[n] != (n + 1)) {
			break;
		}
	}

	if (n == datalen) {
		printk("Data pattern matched expected value\n");
	} else {
		printk("Data pattern does not match. Erase and program\n");
		/* erase 4KB sector */

		err = erase_region(spi, &spi_cfg, 0x20u, spi_addr);
		if (err) {
			printk("Erase 4KB sector at 0x%08x error %d\n", spi_addr, err);
			return;
		}

		printk("Read back and check if ersased\n");

		memset(testbuf1, 0xAAu, sizeof(testbuf1));

		cmd = SPI_FLASH_READ_SLOW_CMD;
		err = spi_flash_read(spi, &spi_cfg, cmd, spi_addr, testbuf1, datalen);
		if (err) {
			printk("SPI flash read: cmd=0x%02x SPIAddr=0x%08x error=%d\n",
				cmd, spi_addr, err);
			return;
		}

		pr_data_buf(testbuf1, datalen);

		if (buf_contains(testbuf1, datalen, 0xffu)) {
			printk("SPI flash region is erase\n");
		} else {
			printk("SPI flash region not erased!\n");
			return;
		}

		/* data to program */
		for (n = 0; n < datalen; n++) {
			testbuf1[n] = n + 1;
		}

		/* program page at start of 4KB sector */
		err = program_test_data(spi, &spi_cfg, spi_addr, testbuf1, datalen);
		if (err) {
			printk("Program test data at 0x%08x len = %u error %d\n",
				spi_addr, datalen, err);
			return;
		}

		memset(testbuf1, 0xAAu, sizeof(testbuf1));

		cmd = SPI_FLASH_READ_SLOW_CMD;
		err = spi_flash_read(spi, &spi_cfg, cmd, spi_addr, testbuf1, datalen);
		if (err) {
			printk("SPI flash read: cmd=0x%02x SPIAddr=0x%08x error=%d\n",
				cmd, spi_addr, err);
			return;
		}

		pr_data_buf(testbuf1, datalen);

		for (n = 0; n < datalen; n++) {
			if (testbuf1[n] != (n + 1)) {
				break;
			}
		}
		if (n == datalen) {
			printk("After programming data pattern matched expected value\n");
		} else {
			printk("After programming data mismatch\n");
			return;
		}
	}

	printk("Read data using fast read command\n");
	memset(testbuf1, 0x66u, sizeof(testbuf1));

	cmd = SPI_FLASH_READ_FAST_CMD;
	err = spi_flash_read(spi, &spi_cfg, cmd, spi_addr, testbuf1, datalen);
	if (err) {
		printk("SPI flash read: cmd=0x%02x SPIAddr=0x%08x error=%d\n",
			cmd, spi_addr, err);
		return;
	}

	pr_data_buf(testbuf1, datalen);

	for (n = 0; n < datalen; n++) {
		if (testbuf1[n] != (n + 1)) {
			break;
		}
	}
	if (n == datalen) {
		printk("SPI Fast Read: data pattern matched expected value\n");
	} else {
		printk("SPI Fast Read: data mismatch\n");
		return;
	}

#ifdef CONFIG_SPI_EXTENDED_MODES
	memset(testbuf1, 0xAAu, sizeof(testbuf1));
	cmd = SPI_FLASH_READ_DUAL_CMD;
	printk("Read %u bytes at SPI offset 0x%08x using dual (1-1-2-8)\n", datalen, spi_addr);

	err = spi_flash_read_dual(spi, &spi_cfg, cmd, spi_addr, testbuf1, datalen);
	if (err) {
		printk("SPI flash read: cmd=0x%02x SPIAddr=0x%08x error=%d\n", cmd, spi_addr, err);
		return;
	}

	pr_data_buf(testbuf1, datalen);

	for (n = 0; n < datalen; n++) {
		if (testbuf1[n] != (n + 1)) {
			break;
		}
	}
	if (n == datalen) {
		printk("SPI read dual opcode 0x3b matched expected value\n");
	} else {
		printk("ERROR: SPI read dual opcode 0x3b did not match!\n");
	}
#endif

#ifdef CONFIG_SPI_ASYNC
	printk("\nTest SPI Async\n");
	memset(testbuf1, 0x66u, sizeof(testbuf1));
	cmd = 0x03u;
	datalen = 16u;
	spi_addr = 0;
	printk("Read %u bytes at SPI offset 0x%08x using cmd 0x%02x\n", datalen, spi_addr, cmd);

	err = spi_flash_read_async(spi, &spi_cfg, cmd, spi_addr, testbuf1, datalen);
	if (err) {
		printk("SPI flash read async error %d\n", err);
	}

	pr_data_buf(testbuf1, datalen);
#endif
	printk("end main\n");
}
