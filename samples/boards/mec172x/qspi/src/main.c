/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/spi.h>
#include <sys/printk.h>
#include <soc.h>
#include <string.h>
#include "crc8.h"

#define SPI_FREQ_HZ		12000000U
#define W25Q128_JEDEC_ID	0x001840efU
#define SST25PF040C_JEDEC_ID	0x00130662U
#define SPI_DEV			DT_LABEL(DT_NODELABEL(spi0))

#define SPI_RD_JEDEC_ID_OP	0x9fU
#define SPI_RD_1110_OP		0x03U
#define SPI_RD_1118_OP		0x0bU

static const struct device *spi_dev;
static volatile uint32_t spin_val;
static volatile int ret_val;

static struct spi_config spi_cfg;

static struct spi_buf_set ibs;
static struct spi_buf_set obs;

static struct spi_buf bin[8];
static struct spi_buf bout[8];

static uint8_t data_buf1[4096];
static uint8_t data_buf2[4096];

void spin_on(uint32_t id, int rval);
void clean_bufs(void);
int pack_bytes_to_word(uint8_t *pb, size_t pbsz, uint32_t *w);


void main(void)
{
	int ret = 0;
	uint32_t d = 0U;
	uint32_t spi_addr = 0U;
	crc8_t crc8 = 0U;

	printk("Hello World! %s\n", CONFIG_BOARD);

	clean_bufs();

	spi_dev = device_get_binding(SPI_DEV);
	if (!spi_dev) {
		printk("ERROR: Get spi0 device binding failed!");
		spin_on(1, 0);
	}

	/* SPI configuration. We have not enabled CONFIG_SPI_EXTENDED_MODES
	 * If extended modes is enabled then operation becomes 32-bit and
	 * has more fields.
	*/
	spi_cfg.frequency = SPI_FREQ_HZ;
	spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |
			    SPI_WORD_SET(8) | SPI_LINES_SINGLE;
	spi_cfg.slave = 0;
	spi_cfg.cs = NULL; /* SPI controller HW handles chip select */

	printk("Read JEDEC ID from SPI flash\n");

	/* SPI input buffer for reading JEDEC ID */
	data_buf1[0] = SPI_RD_JEDEC_ID_OP;
	bin[0].buf = data_buf1;
	bin[0].len = 1;

	ibs.buffers = &bin[0];
	ibs.count = 1;

	/* SPI output buffer 4 byte result */
	bout[0].buf = data_buf2;
	bout[0].len = 4;

	obs.buffers = &bout[0];
	obs.count = 1;

	const struct spi_config *pspi_cfg = (const struct spi_config *)&spi_cfg;

	ret = spi_transceive(spi_dev, pspi_cfg, &ibs, &obs);
	if (ret) {
		printk("ERROR: SPI xfr API returned %d\n", ret);
		spin_on(2, ret);
	}

	pack_bytes_to_word(data_buf2, 4, &d);

	printk("JEDEC ID = 0x%08x\n", d);

	if (d == SST25PF040C_JEDEC_ID) {
		printk("SPI is SST25PF040C\n");
	} else if (d == W25Q128_JEDEC_ID) {
		printk("SPI is W25Q128\n");
	} else {
		printk("SPI is unkown\n");
	}

	/* Read Boot-ROM TAG from its default location at
	 * offset 0 of the SPI flash
	 */

	memset(data_buf2, 0x55, 4);
	spi_addr = 0U;

	printk("Read Boot-ROM TAG 0 SPI flash at offset 0x%08x\n", spi_addr);

	data_buf1[0] = SPI_RD_1110_OP; /* Read 1-1-1 no dummy clocks */
	data_buf1[1] = (spi_addr >> 16) & 0xffu; /* bits[23:16] */
	data_buf1[2] = (spi_addr >> 8) & 0xffu; /* bits[15:8] */
	data_buf1[3] = spi_addr & 0xffu; /* bits[7:0] */

	bin[0].buf = data_buf1;
	bin[0].len = 4;

	ibs.buffers = &bin[0];
	ibs.count = 1;

	/* SPI output buffer 4 byte result */
	bout[0].buf = data_buf2;
	bout[0].len = 4;

	obs.buffers = &bout[0];
	obs.count = 1;

	ret = spi_transceive(spi_dev, pspi_cfg, &ibs, &obs);
	if (ret) {
		printk("ERROR: SPI xfr API returned %d\n", ret);
		spin_on(3, ret);
	}

	pack_bytes_to_word(data_buf2, 4, &d);

	printk("TAG 0 = 0x%08x\n", d);

	crc8 = crc8_init();
	crc8 = crc8_update(crc8, (const unsigned char *)data_buf2, 3U);
	crc8 = crc8_finalize(crc8);

	printk("Calculated CRC-8 of first 3 TAG bytes = 0x%02x\n", crc8);
	if (crc8 == data_buf2[3]) {
		printk("TAG 0 CRC-8 OK\n");
	} else{
		printk("TAG 0 CRC-8 BAD: expected 0x%02x\n", data_buf2[3]);
	}

	spin_on(256, 0);
}

int pack_bytes_to_word(uint8_t *pb, size_t pbsz, uint32_t *w)
{
	if (!pb || !pbsz || !w) {
		return -EINVAL;
	}

	uint32_t d = 0;
	size_t nmax = (pbsz < 4) ? pbsz : 4U;

	for (size_t n = 0; n < nmax; n++) {
		d <<= 8;
		d |= *pb++;
	}

	*w = __builtin_bswap32(d);

	return 0;
}

void clean_bufs(void)
{
	memset(&ibs, 0, sizeof(ibs));
	memset(&obs, 0, sizeof(obs));
	memset(bin, 0, sizeof(bin));
	memset(bout, 0, sizeof(bout));

	memset(data_buf1, 0x55, sizeof(data_buf1));
	memset(data_buf2, 0x55, sizeof(data_buf2));
}

void spin_on(uint32_t id, int rval)
{
	spin_val = id;
	ret_val = rval;

	while (spin_val) {
		;
	}
}
