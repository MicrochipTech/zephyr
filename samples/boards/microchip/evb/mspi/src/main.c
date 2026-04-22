/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/mspi/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

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

#define SPI_FLASH_TEST_ADDR 0x2000U

#define MSPI0_NODE  DT_NODELABEL(qspi0)
#define FLASH0_NODE DT_NODELABEL(spiflash0)

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

const struct device *mspi0_dev = DEVICE_DT_GET(MSPI0_NODE);
const struct device *flash0_dev = DEVICE_DT_GET(FLASH0_NODE);

#define MSPI_OP_MODE_DT(nid) \
	(enum mspi_op_mode)DT_ENUM_IDX_OR(nid, op_mode, MSPI_OP_MODE_CONTROLLER)

#define MSPI_DUPLEX_DT(nid) \
	(enum mspi_duplex)DT_ENUM_IDX_OR(nid, duplex, MSPI_HALF_DUPLEX)

static struct gpio_dt_spec ce_gpios[] = MSPI_CE_GPIOS_DT_SPEC_GET(MSPI0_NODE);

static const struct mspi_dt_spec mspi0_dt_spec = {
	.bus = DEVICE_DT_GET(MSPI0_NODE),
	.config = {
		.channel_num = 0,
		.op_mode = MSPI_OP_MODE_DT(MSPI0_NODE),
		.duplex = MSPI_DUPLEX_DT(MSPI0_NODE),
		.dqs_support = DT_PROP_OR(MSPI0_NODE, dqs_support, 0),
		.sw_multi_periph = DT_PROP_OR(MSPI0_NODE, software_multiperipheral, 0),
		.max_freq = DT_PROP_OR(MSPI0_NODE, clock_frequency, MHZ(12)),
		.num_periph = DT_CHILD_NUM(MSPI0_NODE),
		.ce_group = ce_gpios,
		.num_ce_gpios = ARRAY_SIZE(ce_gpios),
	},
};

static struct mspi_dev_id dev_id[] = {
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(MSPI0_NODE, MSPI_DEVICE_ID_DT, (,))
};

static struct mspi_dev_cfg device_cfg[] = {
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(MSPI0_NODE, MSPI_DEVICE_CONFIG_DT, (,))
};

static uint32_t jedec_id;

#define SPI_TEST_BUF1_SIZE 256

static uint8_t buf1[SPI_TEST_BUF1_SIZE];

#define SPI_TEST_ADDR1 0xc10000u

#define SPI_FILL_VAL         0x69u
#define SPI_TEST_BUFFER_SIZE (4U * 1024U)

struct test_buf {
	union {
		uint32_t w[(SPI_TEST_BUFFER_SIZE) / 4];
		uint16_t h[(SPI_TEST_BUFFER_SIZE) / 2];
		uint8_t b[(SPI_TEST_BUFFER_SIZE)];
	};
};

static struct test_buf tb1;
static struct test_buf tb2;

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

#define BUF_FILL_INCR_VAL 0
#define BUF_FILL_RAND_VAL 1U

static int fill_buffer(uint8_t *buf, size_t bufsz, uint8_t fill_type)
{
	if ((buf == NULL) || (bufsz == 0)) {
		return -EINVAL;
	}

	if (fill_type == BUF_FILL_RAND_VAL) {
		sys_rand_get(buf, bufsz);
	} else {
		for (size_t n = 0; n < bufsz; n++) {
			buf[n] = (uint8_t)(n % 256U);
		}
	}

	return 0;
}

#if 0
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
#endif

int main(void)
{
	int rc = 0;
	off_t flash_offset = 0;
	size_t flash_len = 0;
	bool erased = false;

	LOG_INF("MEC_Assy6941 board MSPI sample main");

	jedec_id = 0xffffffffU;

	memset(buf1, 0, sizeof(buf1));
	memset((void *)&tb1, 0x55, sizeof(tb1));

	LOG_INF("Check if MSPI device is ready");
	if (!device_is_ready(mspi0_dev)) {
		LOG_ERR("MSPI driver not ready!\n");
		return -1;
	}

	if (!device_is_ready(flash0_dev)) {
		LOG_ERR("MSPI flash driver is not ready!\n");
		return -2;
	}

	rc = flash_read_jedec_id(flash0_dev, (uint8_t *)&jedec_id);
	if (rc != 0) {
		LOG_ERR("Flash driver read JEDEC-ID failed (%d)", rc);
	} else {
		LOG_INF("Flash driver read JEDEC_ID = 0x%08x", jedec_id);
	}

	LOG_INF("Configure MSPI controller using DT parameters");
	rc = mspi_config(&mspi0_dt_spec);
	LOG_INF("MSPI API config returned (%d)", rc);
	if (rc != 0) {
		goto app_end;
	}

	LOG_INF("Configure MSPI device 0 using DT parameters");

	const enum mspi_dev_cfg_mask param_msk = MSPI_DEVICE_CONFIG_ALL;

	rc = mspi_dev_config(mspi0_dt_spec.bus, &dev_id[0], param_msk, &device_cfg[0]);
	LOG_INF("MSPI API device config returned (%d)", rc);
	if (rc != 0) {
		goto app_end;
	}

#if 0  /* all MSPI drivers are clearing dev_id if get channel status indicates not busy
	* Therefore mspi_dev_config and mspi_transceive must be called back to back*/
	LOG_INF("Query channel 0 status");
	rc = mspi_get_channel_status(mspi0_dt_spec.bus, 0);
	LOG_INF("MSPI API get chan status returned (%d)", rc);
	if (rc != 0) {
		goto app_end;
	}
#endif

	LOG_INF("Read from MSPI device using DT parameters");

	const struct mspi_xfer_packet pkt = {
		.dir = MSPI_RX,
		.cb_mask = 0,
		.cmd = 0x6BU, /* Quad 1-1-4 */
		.address = 0x1240U,
		.num_bytes = 32U,
		.data_buf = tb1.b,
	};

	const struct mspi_xfer mxfr = {
		.async = false,
		.xfer_mode = MSPI_PIO,
		.tx_dummy = 0,
		.rx_dummy = 8U, /* cycles */
		.cmd_length = 1U, /* bytes */
		.addr_length = 3U, /* bytes */
		.hold_ce = false,
		.ce_sw_ctrl = {
			.gpio = {NULL, 0, 0},
			.delay = 0,
		},
		.priority = MSPI_XFER_PRIORITY_LOW,
		.packets = &pkt,
		.num_packet = 1U,
		.timeout = 0,
	};

	rc = mspi_transceive(mspi0_dt_spec.bus, &dev_id[0], &mxfr);
	LOG_INF("MSPI API transceive returned (%d)", rc);
	if (rc != 0) {
		goto app_end;
	}

	LOG_HEXDUMP_INF(tb1.b, 32U, "RX buffer contents");

	 /* all MSPI drivers are clearing dev_id if get channel status indicates not busy
	  * Therefore mspi_dev_config and mspi_transceive must be called back to back*/
	LOG_INF("Query channel 0 status");
	rc = mspi_get_channel_status(mspi0_dt_spec.bus, 0);
	LOG_INF("MSPI API get chan status returned (%d)", rc);
	if (rc != 0) {
		goto app_end;
	}

	/* SPI_FLASH_TEST_ADDR */
	memset(&tb1.w, 0x55, SPI_FLASH_SECTOR_SIZE);

	flash_offset = SPI_FLASH_TEST_ADDR;
	flash_len = SPI_FLASH_PAGE_SIZE;

	LOG_INF("Flash read %u bytes at 0x%0x", flash_len, (uint32_t)flash_offset);

	rc = flash_read(flash0_dev, flash_offset, tb1.b, flash_len);
	if (rc != 0) {
		LOG_ERR("Flash read error (%d)", rc);
	}

	LOG_HEXDUMP_INF(tb1.b, 16U, "First 16-bytes of sector");

	erased = is_data_buf_filled_with(tb1.b, flash_len, 0xffU);
	if (erased == true) {
		LOG_INF("Sector is currently erased");
	} else {
		LOG_INF("Sector is not in erased state");
	}

	if (erased == false) {
		LOG_INF("Erase Sector");
		/* flash API requries size passed to erase be valid sector size for the device */
		rc = flash_erase(flash0_dev, flash_offset, SPI_FLASH_SECTOR_SIZE);
		if (rc < 0) {
			LOG_ERR("Flash erase failed (%d)", rc);
		}

		/* check if erased */
		memset((void *)tb1.b, 0x55, SPI_FLASH_SECTOR_SIZE);
		rc = flash_read(flash0_dev, flash_offset, tb1.b, SPI_FLASH_SECTOR_SIZE);
		if (rc < 0) {
			LOG_ERR("Flash read failed (%d) at line %d", rc, __LINE__);
		}

		if (is_data_buf_filled_with(tb1.b, flash_len, 0xffU)) {
			LOG_INF("Sector read back shows it is erased");
		} else {
			LOG_ERR("Sector read back shows it is NOT erased");
		}
	}

	fill_buffer(tb2.b, flash_len, BUF_FILL_RAND_VAL);

	LOG_INF("Write to flash");

	rc = flash_write(flash0_dev, flash_offset, (const void *)tb2.b, flash_len);
	if (rc != 0) {
		LOG_ERR("Flash write error (%d)", rc);
	}

	LOG_INF("Read back data");

	rc = flash_read(flash0_dev, flash_offset, tb1.b, flash_len);
	if (rc != 0) {
		LOG_ERR("Flash read back error (%d)", rc);
	}

	rc = memcmp(tb1.b, tb2.b, flash_len);
	if (rc == 0) {
		LOG_INF("Flash read back matches data written: PASS");
	} else {
		LOG_INF("Flash read back does NOT match data written: FAIL");
	}
app_end:
	LOG_INF("Program End");

	return 0;
}
