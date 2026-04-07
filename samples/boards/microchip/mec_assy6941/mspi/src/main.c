/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/mspi/devicetree.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#define W25Q128_JEDEC_ID       0x001840efU
#define W25Q128JV_JEDEC_ID     0x001870efU
#define SST26VF016B_JEDEC_ID   0x004126bfU
#define SPI_FLASH_JEDEC_ID_MSK 0x00ffffffu

#define SPI_FLASH_READ_JEDEC_ID_OPCODE 0x9fu

#define SPI_FLASH_READ_STATUS1_OPCODE 0x05u
#define SPI_FLASH_READ_STATUS2_OPCODE 0x35u
#define SPI_FLASH_READ_STATUS3_OPCODE 0x15u

#define SPI_FLASH_WRITE_STATUS1_OPCODE 0x01u
#define SPI_FLASH_WRITE_STATUS2_OPCODE 0x31u
#define SPI_FLASH_WRITE_STATUS3_OPCODE 0x11u

#define SPI_FLASH_WRITE_ENABLE_OPCODE              0x06u
#define SPI_FLASH_WRITE_DISABLE_OPCODE             0x04u
#define SPI_FLASH_WRITE_ENABLE_VOLATILE_STS_OPCODE 0x50u

#define SPI_FLASH_READ_SFDP_CMD 0x5au

#define SPI_FLASH_READ_SLOW_OPCODE 0x03u
#define SPI_FLASH_READ_FAST_OPCODE 0x0bu
#define SPI_FLASH_READ_DUAL_OPCODE 0x3bu
#define SPI_FLASH_READ_QUAD_OPCODE 0x6bu

#define SPI_FLASH_ERASE_SECTOR_OPCODE 0x20u
#define SPI_FLASH_ERASE_BLOCK1_OPCODE 0x52u
#define SPI_FLASH_ERASE_BLOCK2_OPCODE 0xd8u
#define SPI_FLASH_ERASE_CHIP_OPCODE   0xc7u

#define SPI_FLASH_PAGE_PROG_OPCODE 0x02u

#define SPI_FLASH_PAGE_SIZE   256
#define SPI_FLASH_SECTOR_SIZE 4096
#define SPI_FLASH_BLOCK1_SIZE (32 * 1024)
#define SPI_FLASH_BLOCK2_SIZE (64 * 1024)

#define SPI_FLASH_STATUS1_BUSY_POS 0
#define SPI_FLASH_STATUS1_WEL_POS  1

#define SPI_FLASH_STATUS2_SRL_POS     0
#define SPI_FLASH_STATUS2_QE_POS      1
#define SPI_FLASH_STATUS2_SUS_STS_POS 7

#define MSPI0_NODE DT_NODELABEL(qspi0)

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

#if 0 /* BUILD ERROR */
static struct gpio_dt_spec ce_gpios[] = MSPI_CE_GPIOS_DT_SPEC_GET(MSPI0_NODE);
#endif

const struct device *mspi0_dev = DEVICE_DT_GET(MSPI0_NODE);

static const struct mspi_cfg mspi_cfg0 = {
	.channel_num = 0,
	.op_mode = MSPI_OP_MODE_CONTROLLER,
	.duplex = MSPI_HALF_DUPLEX,
	.dqs_support = false,
	.ce_group = NULL, // BUILD ERROR ce_gpios,
	.num_ce_gpios = 0, // BUILD ERROR ARRAY_SIZE(ce_gpios),
	.num_periph = 4u, /* DT_CHILD_NUM(MSPI0_NODE) */
	.max_freq = DT_PROP(MSPI0_NODE, clock_frequency),
};

static const struct mspi_dt_spec mspi0_dt_spec = {
	.bus = DEVICE_DT_GET(MSPI0_NODE),
	.config = mspi_cfg0,
};

#if 0
static struct mspi_dev_cfg mspi_dev_cfg0;
static struct mspi_xfer_packet pkt0;
static struct mspi_xfer mspi_xfr0;
#endif

static uint32_t jedec_id;

#define SPI_TEST_BUF1_SIZE 256

static uint8_t buf1[SPI_TEST_BUF1_SIZE];

#define SPI_TEST_ADDR1 0xc10000u

#define SPI_FILL_VAL         0x69u
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

int main(void)
{
	struct spi_address spi_addr = {0};
	int err = 0;

	LOG_INF("MEC_Assy6941 board MSPI sample main");

	jedec_id = 0xffffffffU;

	memset(buf1, 0, sizeof(buf1));
	memset((void *)&tb1, 0x55, sizeof(tb1));

	if (!device_is_ready(mspi0_dev)) {
		LOG_ERR("MSPI 0 device not ready!\n");
		return -1;
	}

	err = mspi_config(&mspi0_dt_spec);

	LOG_INF("Program End");

	return 0;
}
