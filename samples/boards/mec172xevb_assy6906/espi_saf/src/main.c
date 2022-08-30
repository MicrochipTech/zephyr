/*
 * Copyright (c) 2019 Intel Corporation
 * Copyright (c) 2022 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_saf.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

/* eSPI flash parameters */
#define MAX_TEST_BUF_SIZE     1024u
#define MAX_FLASH_REQUEST     64u
#define TARGET_FLASH_REGION   0x72000ul

#define ESPI_FREQ_20MHZ       20u
#define ESPI_FREQ_25MHZ       25u
#define ESPI_FREQ_66MHZ       66u

#define K_WAIT_DELAY          100u

/* eSPI event */
#define EVENT_MASK            0x0000FFFFu
#define EVENT_DETAILS_MASK    0xFFFF0000u
#define EVENT_DETAILS_POS     16u
#define EVENT_TYPE(x)         (x & EVENT_MASK)
#define EVENT_DETAILS(x)      ((x & EVENT_DETAILS_MASK) >> EVENT_DETAILS_POS)

#define PWR_SEQ_TIMEOUT    3000u

/* The devicetree node identifier for the board power rails pins. */
#define BRD_PWR_NODE DT_NODELABEL(board_power)

#if DT_NODE_HAS_STATUS(BRD_PWR_NODE, okay)
static const struct gpio_dt_spec pwrgd_gpio = GPIO_DT_SPEC_GET(BRD_PWR_NODE, pwrg_gpios);
static const struct gpio_dt_spec rsm_gpio = GPIO_DT_SPEC_GET(BRD_PWR_NODE, rsm_gpios);
#endif

static const struct device *espi_dev = DEVICE_DT_GET(DT_NODELABEL(espi0));
static struct espi_callback espi_bus_cb;
static struct espi_callback vw_rdy_cb;
static struct espi_callback vw_cb;
static struct espi_callback p80_cb;
static uint8_t espi_rst_sts;

#define TEST_FLAG_FLASH_KNOWN BIT(0)
#define TEST_FLAG_FLASH_HAS_RPMC BIT(1)

static uint32_t test_flags;

#define ESPI_NODE DT_NODELABEL(espi0)
#define ESPI_BASE_ADDR DT_REG_ADDR(ESPI_NODE)

#ifdef CONFIG_ESPI_SAF_XEC_V2
#define ESPI_SAF_NODE     DT_NODELABEL(espi_saf0)
#define SPI_NODE          DT_NODELABEL(spi0)

#define SAF_BASE_ADDR     DT_REG_ADDR(ESPI_SAF_NODE)

#define SAF_TEST_FREQ_HZ 24000000U
#define SAF_TEST_BUF_SIZE 4096U

/* SPI address of 4KB sector modified by test */
#define SAF_SPI_TEST_ADDRESS 0x1000U

#define SPI_WRITE_STATUS1 0x01U
#define SPI_WRITE_STATUS2 0x31U
#define SPI_WRITE_DISABLE 0x04U
#define SPI_READ_STATUS1 0x05U
#define SPI_WRITE_ENABLE 0x06U
#define SPI_READ_STATUS2 0x35U
#define SPI_WRITE_ENABLE_VS 0x50U
#define SPI_READ_JEDEC_ID 0x9FU

#define SPI_STATUS1_BUSY 0x80U
#define SPI_STATUS2_QE 0x02U

#define W25Q128_JEDEC_ID 0x001840efU
#define W25R128_JEDEC_ID 0x001940efU

#define SAF_RPMC_ENABLE_HMAC_UPDATE 0

enum saf_erase_size {
	SAF_ERASE_4K = 0,
	SAF_ERASE_32K = 1,
	SAF_ERASE_64K = 2,
	SAF_ERASE_MAX
};

struct saf_addr_info {
	uintptr_t saf_struct_addr;
	uintptr_t saf_exp_addr;
};

static const struct device *qspi_dev;
static const struct device *espi_saf_dev;
static uint8_t safbuf[SAF_TEST_BUF_SIZE] __aligned(4);
static uint8_t safbuf2[SAF_TEST_BUF_SIZE] __aligned(4);
static uint8_t saf_rpmc_buf[64] __aligned(4);

/*
 * W25R128 SPI flash SAF configuration.
 * Size is 16Mbytes, it requires no continuous mode prefix, or
 * other special SAF configuration.
 */
#define MCHP_W25R128_RPMC_OP1_FLAGS (MCHP_FLASH_RPMC_MIRROR_048 | \
	MCHP_FLASH_RPMC_MIRROR_848 | MCHP_FLASH_RPMC_MIRROR_NCTNR)

static const struct espi_saf_flash_cfg flash_w25r128 = {
	.version = MCHP_SAF_VER_2,
	.rd_freq_mhz = 0, /* use QMSPI driver frequency for reads */
	.freq_mhz = 0, /* use QMSPI driver frequency for all other commands */
	.flashsz = 0x1000000U,
	.opa = MCHP_SAF_OPCODE_REG_VAL(0x06U, 0x75U, 0x7aU, 0x05U),
	.opb = MCHP_SAF_OPCODE_REG_VAL(0x20U, 0x52U, 0xd8U, 0x02U),
#if DT_PROP(DT_NODELABEL(spi0), lines) == 2
	.opc = MCHP_SAF_OPCODE_REG_VAL(0xbbu, 0xffu, 0xa5u, 0x35u),
#else
	.opc = MCHP_SAF_OPCODE_REG_VAL(0xebU, 0xffU, 0xa5U, 0x35U),
#endif
	.opd = MCHP_SAF_OPCODE_REG_VAL(0xb9u, 0xabU, MCHP_FLASH_RPMC_OP2_DFLT, 0U),
	/* RPMC OP1, number of counters, host visibility flags, unused */
	.rpmc_op1 = MCHP_SAF_OPCODE_REG_VAL(MCHP_FLASH_RPMC_OP1_DFLT, 4U,
					    MCHP_W25R128_RPMC_OP1_FLAGS, 0U),
	.flags = 0U,
	.poll2_mask = MCHP_W25Q128_POLL2_MASK,
	.cont_prefix = 0U,
#if DT_PROP(DT_NODELABEL(spi0), lines) == 2
	.cs_cfg_descr_ids = MCHP_CS0_CFG_DESCR_IDX_REG_VAL_DUAL,
	.descr = {
		MCHP_W25Q128_CM_RD_DUAL_D0,
		MCHP_W25Q128_CM_RD_DUAL_D1,
		MCHP_W25Q128_CM_RD_DUAL_D2,
		MCHP_W25Q128_ENTER_CM_DUAL_D0,
		MCHP_W25Q128_ENTER_CM_DUAL_D1,
		MCHP_W25Q128_ENTER_CM_DUAL_D2,
	},
#else
	.cs_cfg_descr_ids = MCHP_CS0_CFG_DESCR_IDX_REG_VAL,
	.descr = {
		MCHP_W25Q128_CM_RD_D0,
		MCHP_W25Q128_CM_RD_D1,
		MCHP_W25Q128_CM_RD_D2,
		MCHP_W25Q128_ENTER_CM_D0,
		MCHP_W25Q128_ENTER_CM_D1,
		MCHP_W25Q128_ENTER_CM_D2
	}
#endif
};

/*
 * SAF driver configuration.
 * One SPI flash device.
 * Use QMSPI frequency, chip select timing, and signal sampling configured
 * by QMSPI driver.
 * Use SAF hardware default TAG map.
 */
static const struct espi_saf_cfg saf_cfg1 = {
	.nflash_devices = 1U,
	.hwcfg = {
		.version = MCHP_SAF_VER_2,
		.flags = 0U,
		.qmspi_cpha = 0U,
		.qmspi_cs_timing = 0U,
		.flash_pd_timeout = 0U,
		.flash_pd_min_interval = 0U,
		.generic_descr = {
#if DT_PROP(DT_NODELABEL(spi0), lines) == 2
			MCHP_SAF_EXIT_CM_DUAL_DESCR12, MCHP_SAF_EXIT_CMD_DUAL_DESCR13,
#else
			MCHP_SAF_EXIT_CM_DESCR12, MCHP_SAF_EXIT_CM_DESCR13,
#endif
			MCHP_SAF_POLL_DESCR14, MCHP_SAF_POLL_DESCR15
		},
		.tag_map = { 0U, 0U, 0U },
	},
	.flash_cfgs = (struct espi_saf_flash_cfg *)&flash_w25r128
};

/*
 * Example for SAF driver set protection regions API.
 */
static const struct espi_saf_pr w25q128_protect_regions[2] = {
	{
		.start = 0xe00000U,
		.size =  0x100000U,
		.master_bm_we = (1U << MCHP_SAF_MSTR_HOST_PCH_ME),
		.master_bm_rd = (1U << MCHP_SAF_MSTR_HOST_PCH_ME),
		.pr_num = 1U,
		.flags = MCHP_SAF_PR_FLAG_ENABLE
			 | MCHP_SAF_PR_FLAG_LOCK,
	},
	{
		.start = 0xf00000U,
		.size =  0x100000U,
		.master_bm_we = (1U << MCHP_SAF_MSTR_HOST_PCH_LAN),
		.master_bm_rd = (1U << MCHP_SAF_MSTR_HOST_PCH_LAN),
		.pr_num = 2U,
		.flags = MCHP_SAF_PR_FLAG_ENABLE
			 | MCHP_SAF_PR_FLAG_LOCK,
	},
};

static const struct espi_saf_protection saf_pr_w25q128 = {
	.nregions = 2U,
	.pregions = w25q128_protect_regions
};

#if SAF_RPMC_ENABLE_HMAC_UPDATE
/* Writing the RPMC Root Key is a one time operation and cannot
 * be erased. For testing, RPMC allows writing a root key of all 1's
 * which does not set the one time root key program lock.
 */
static const uint8_t rpmc_root_key_dflt[32] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0ff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0ff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0ff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0ff, 0xff
};

/* RPMC HMAC Key Data is a 4 byte value used to derive a 32 byte
 * HMAC-SHA-256 key.
 */
static const uint8_t rpmc_key_data[4] = {
	0xa1, 0x19, 0x1f, 0xbc
};
#endif

/*
 * Initialize the local attached SPI flash.
 * 1. Get SPI driver binding
 * 2. Read JEDEC ID and verify its a W25Q128
 * 3. Read STATUS2 and check QE bit
 * 4. If QE bit is not set
 *      Send volatile status write enable
 *      Set volatile QE bit
 *      Check STATUS1 BUSY, not expected to be set for volatile status write.
 *      Read STATUS2 and check QE
 * Returns 0 if QE was already set or this routine successfully set volatile
 * QE. Returns < 0 on SPI driver error or unexpected BUSY or STATUS values.
 * NOTE: SPI driver transceive API will configure the SPI controller to the
 * settings passed in the struct spi_config. We set the frequency to the
 * frequency we will be using for SAF.
 */
int spi_saf_init(void)
{
	struct spi_config spi_cfg;
	struct spi_buf_set tx_bufs;
	struct spi_buf_set rx_bufs;
	struct spi_buf txb;
	struct spi_buf rxb;
	uint8_t spi_status2;
	uint32_t jedec_id;
	int ret;

	/* Read JEDEC ID command and fill read buffer */
	safbuf[0] = SPI_READ_JEDEC_ID;
	memset(safbuf2, 0x55, 4U);

	spi_cfg.frequency = SAF_TEST_FREQ_HZ;
	spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB
			    | SPI_WORD_SET(8);

	/*
	 * Use SPI master mode and inform driver the SPI controller hardware
	 * controls chip select.
	 */
	jedec_id = 0U;
	spi_cfg.slave = 0;
	spi_cfg.cs = NULL;

	txb.buf = &safbuf;
	txb.len = 1U;

	tx_bufs.buffers = (const struct spi_buf *)&txb;
	tx_bufs.count = 1U;

	rxb.buf = &jedec_id;
	rxb.len = 3U;

	rx_bufs.buffers = (const struct spi_buf *)&rxb;
	rx_bufs.count = 1U;

	ret = spi_transceive(qspi_dev, (const struct spi_config *)&spi_cfg,
			     (const struct spi_buf_set *)&tx_bufs,
			     (const struct spi_buf_set *)&rx_bufs);
	if (ret) {
		LOG_ERR("Read JEDEC ID spi_transceive failure: error %d", ret);
		return ret;
	}

	LOG_INF("SPI Flash JEDEC ID is %0x", jedec_id);

	switch (jedec_id) {
	case W25Q128_JEDEC_ID:
		test_flags |= TEST_FLAG_FLASH_KNOWN;
		LOG_INF("Flash is W25Q128");
		break;
	case W25R128_JEDEC_ID:
		test_flags |= TEST_FLAG_FLASH_KNOWN | TEST_FLAG_FLASH_HAS_RPMC;
		LOG_INF("Flash is W25R128");
		break;
	default:
		LOG_ERR("JEDIC ID does not match W25Q128 %0x", safbuf2[0]);
		return -1;
	}

	/* Read STATUS2 to get quad enable bit */
	safbuf[0] = SPI_READ_STATUS2;
	memset(safbuf2, 0, 4U);

	txb.buf = &safbuf;
	txb.len = 1U;

	tx_bufs.buffers = (const struct spi_buf *)&txb;
	tx_bufs.count = 1U;

	rxb.buf = &safbuf2;
	rxb.len = 1U;

	rx_bufs.buffers = (const struct spi_buf *)&rxb;
	rx_bufs.count = 1U;

	ret = spi_transceive(qspi_dev, (const struct spi_config *)&spi_cfg,
			     (const struct spi_buf_set *)&tx_bufs,
			     (const struct spi_buf_set *)&rx_bufs);
	if (ret) {
		LOG_ERR("Read STATUS2 spi_transceive failure: error %d", ret);
		return ret;
	}

	spi_status2 = safbuf2[0];

#if DT_PROP(DT_NODELABEL(spi0), lines) == 4
	uint8_t spi_status1;

	/*
	 * If QE not set then write the volatile QE bit.
	 * SAF test requires SPI flash quad enabled so the WP#/HOLD# signals
	 * will act as IO2/IO3. We will write the volatile QE bit for less
	 * wear of the STATUS2 register
	 */
	if ((spi_status2 & SPI_STATUS2_QE) == 0U) {
		safbuf[0] = SPI_WRITE_ENABLE_VS;

		txb.buf = &safbuf;
		txb.len = 1U;

		tx_bufs.buffers = (const struct spi_buf *)&txb;
		tx_bufs.count = 1U;

		rx_bufs.buffers = NULL;
		rx_bufs.count = 0U;

		ret = spi_transceive(qspi_dev,
				     (const struct spi_config *)&spi_cfg,
				     (const struct spi_buf_set *)&tx_bufs,
				     (const struct spi_buf_set *)&rx_bufs);
		if (ret) {
			LOG_ERR("Send write enable volatile spi_transceive"
				" failure: error %d", ret);
			return ret;
		}

		safbuf[0] = SPI_WRITE_STATUS2;
		safbuf[1] = spi_status2 | SPI_STATUS2_QE;

		txb.buf = &safbuf;
		txb.len = 2U;

		tx_bufs.buffers = (const struct spi_buf *)&txb;
		tx_bufs.count = 1U;

		rx_bufs.buffers = NULL;
		rx_bufs.count = 0U;

		ret = spi_transceive(qspi_dev,
				     (const struct spi_config *)&spi_cfg,
				     (const struct spi_buf_set *)&tx_bufs,
				     (const struct spi_buf_set *)&rx_bufs);
		if (ret) {
			LOG_ERR("Write SPI STATUS2 QE=1 spi_transceive"
				" failure: error %d", ret);
			return ret;
		}

		/* Write to volatile status is fast, expect BUSY to be clear */
		safbuf[0] = SPI_READ_STATUS1;
		memset(safbuf2, 0, 4U);

		txb.buf = &safbuf;
		txb.len = 1U;

		tx_bufs.buffers = (const struct spi_buf *)&txb;
		tx_bufs.count = 1U;

		rxb.buf = &safbuf2;
		/* read 2 bytes both will be STATUS1 */
		rxb.len = 2U;

		rx_bufs.buffers = (const struct spi_buf *)&rxb;
		rx_bufs.count = 1U;

		ret = spi_transceive(qspi_dev,
				     (const struct spi_config *)&spi_cfg,
				     (const struct spi_buf_set *)&tx_bufs,
				     (const struct spi_buf_set *)&rx_bufs);
		if (ret) {
			LOG_ERR("Read SPI STATUS1 spi_transceive"
				" failure: error %d", ret);
			return ret;
		}

		spi_status1 = safbuf2[0];
		if (spi_status1 & SPI_STATUS1_BUSY) {
			LOG_ERR("SPI BUSY set after write to volatile STATUS2:"
				" STATUS1=0x%02X", spi_status1);
			return ret;
		}

		/* Read STATUS2 to make sure QE is set */
		safbuf[0] = SPI_READ_STATUS2;
		memset(safbuf2, 0, 4U);

		txb.buf = &safbuf;
		txb.len = 1U;

		tx_bufs.buffers = (const struct spi_buf *)&txb;
		tx_bufs.count = 1U;

		rxb.buf = &safbuf2;
		/* read 2 bytes both will be STATUS2 */
		rxb.len = 2U;

		rx_bufs.buffers = (const struct spi_buf *)&rxb;
		rx_bufs.count = 1U;

		ret = spi_transceive(qspi_dev,
				     (const struct spi_config *)&spi_cfg,
				     (const struct spi_buf_set *)&tx_bufs,
				     (const struct spi_buf_set *)&rx_bufs);
		if (ret) {
			LOG_ERR("Read 2 of SPI STATUS2  spi_transceive"
				" failure: error %d", ret);
			return ret;
		}

		spi_status2 = safbuf2[0];
		if (!(spi_status2 & SPI_STATUS2_QE)) {
			LOG_ERR("Read back of SPI STATUS2 after setting "
				"volatile QE bit shows QE not set: 0x%02X",
				spi_status2);
			return -1;
		}
	}
#endif
	return 0;
}

int espi_saf_init(void)
{
	int ret;

	ret = espi_saf_config(espi_saf_dev, (struct espi_saf_cfg *)&saf_cfg1);
	if (ret) {
		LOG_ERR("Failed to configure eSPI SAF error %d", ret);
	} else {
		LOG_INF("eSPI SAF configured successfully!");
	}

	ret = espi_saf_set_protection_regions(espi_saf_dev,
					      &saf_pr_w25q128);
	if (ret) {
		LOG_ERR("Failed to set SAF protection region(s) %d", ret);
	} else {
		LOG_INF("eSPI SAF protection regions(s) configured!");
	}

	return ret;
}

static int pr_check_range(struct mchp_espi_saf *regs,
			  const struct espi_saf_pr *pr)
{
	uint32_t limit;

	limit = pr->start + pr->size - 1U;

	/* registers b[19:0] = bits[31:12] (align on 4KB) */
	if (regs->SAF_PROT_RG[pr->pr_num].START != (pr->start >> 12)) {
		return -1;
	}

	if (regs->SAF_PROT_RG[pr->pr_num].LIMIT != (limit >> 12)) {
		return -1;
	}

	return 0;
}

static int pr_check_enable(struct mchp_espi_saf *regs,
			   const struct espi_saf_pr *pr)
{
	if (pr->flags & MCHP_SAF_PR_FLAG_ENABLE) {
		if (regs->SAF_PROT_RG[pr->pr_num].LIMIT >
		    regs->SAF_PROT_RG[pr->pr_num].START) {
			return 0;
		}
	} else {
		if (regs->SAF_PROT_RG[pr->pr_num].START >
		    regs->SAF_PROT_RG[pr->pr_num].LIMIT) {
			return 0;
		}
	}

	return -2;
}

static int pr_check_lock(struct mchp_espi_saf *regs,
			 const struct espi_saf_pr *pr)
{
	if (pr->flags & MCHP_SAF_PR_FLAG_LOCK) {
		if (regs->SAF_PROT_LOCK & BIT(pr->pr_num)) {
			return 0;
		}
	} else {
		if (!(regs->SAF_PROT_LOCK & BIT(pr->pr_num))) {
			return 0;
		}
	}

	return -3;
}

/* NOTE: bit[0] of bit map registers is read-only = 1 */
static int pr_check_master_bm(struct mchp_espi_saf *regs,
			      const struct espi_saf_pr *pr)
{
	if (regs->SAF_PROT_RG[pr->pr_num].WEBM !=
	    (pr->master_bm_we | BIT(0))) {
		return -4;
	}

	if (regs->SAF_PROT_RG[pr->pr_num].RDBM !=
	    (pr->master_bm_rd | BIT(0))) {
		return -4;
	}

	return 0;
}

static int espi_saf_test_pr1(const struct espi_saf_protection *spr)
{
	struct mchp_espi_saf *saf_regs;
	const struct espi_saf_pr *pr;
	int rc;

	LOG_INF("espi_saf_test_pr1");

	if (spr == NULL) {
		return 0;
	}

	saf_regs = (struct mchp_espi_saf *)(SAF_BASE_ADDR);
	pr = spr->pregions;

	for (size_t n = 0U; n < spr->nregions; n++) {
		rc = pr_check_range(saf_regs, pr);
		if (rc) {
			LOG_INF("SAF Protection region %u range fail",
				pr->pr_num);
			return rc;
		}

		rc = pr_check_enable(saf_regs, pr);
		if (rc) {
			LOG_INF("SAF Protection region %u enable fail",
				pr->pr_num);
			return rc;
		}

		rc = pr_check_lock(saf_regs, pr);
		if (rc) {
			LOG_INF("SAF Protection region %u lock check fail",
				pr->pr_num);
			return rc;
		}

		rc = pr_check_master_bm(saf_regs, pr);
		if (rc) {
			LOG_INF("SAF Protection region %u Master select fail",
				pr->pr_num);
			return rc;
		}

		pr++;
	}

	return 0;
}

/* SAF hardware limited to 1 to 64 byte read requests. */
static int saf_read(uint32_t spi_addr, uint8_t *dest, int len)
{
	int rc, chunk_len, n;
	struct espi_saf_packet saf_pkt = { 0 };

	if ((dest == NULL) || (len < 0)) {
		return -EINVAL;
	}

	saf_pkt.flash_addr = spi_addr;
	saf_pkt.buf = dest;

	n = len;
	while (n) {
		chunk_len = 64;
		if (n < 64) {
			chunk_len = n;
		}

		saf_pkt.len = chunk_len;

		rc = espi_saf_flash_read(espi_saf_dev, &saf_pkt);
		if (rc != 0) {
			LOG_INF("%s: error = %d: chunk_len = %d "
				"spi_addr = %x", __func__, rc, chunk_len,
				spi_addr);
			return rc;
		}

		saf_pkt.flash_addr += chunk_len;
		saf_pkt.buf += chunk_len;
		n -= chunk_len;
	}

	return len;
}

/* SAF hardware limited to 4KB(mandatory), 32KB, and 64KB erase sizes.
 * eSPI configuration has flags the Host can read specifying supported
 * erase sizes.
 */
static int saf_erase_block(uint32_t spi_addr, enum saf_erase_size ersz)
{
	int rc;
	struct espi_saf_packet saf_pkt = { 0 };

	switch (ersz) {
	case SAF_ERASE_4K:
		saf_pkt.len = 4096U;
		spi_addr &= ~(4096U - 1U);
		break;
	case SAF_ERASE_32K:
		saf_pkt.len = (32U * 1024U);
		spi_addr &= ~((32U * 1024U) - 1U);
		break;
	case SAF_ERASE_64K:
		saf_pkt.len = (64U * 1024U);
		spi_addr &= ~((64U * 1024U) - 1U);
		break;
	default:
		return -EINVAL;
	}

	saf_pkt.flash_addr = spi_addr;

	rc = espi_saf_flash_erase(espi_saf_dev, &saf_pkt);
	if (rc != 0) {
		LOG_INF("espi_saf_test1: erase fail = %d", rc);
		return rc;
	}

	return 0;
}

/* SAF hardware limited to 1 to 64 byte programming within a 256 byte page. */
static int saf_page_prog(uint32_t spi_addr, const uint8_t *src, int progsz)
{
	int rc, chunk_len, n;
	struct espi_saf_packet saf_pkt = { 0 };

	if ((src == NULL) || (progsz < 0) || (progsz > 256)) {
		return -EINVAL;
	}

	if (progsz == 0) {
		return 0;
	}

	saf_pkt.flash_addr = spi_addr;
	saf_pkt.buf = (uint8_t *)src;

	n = progsz;
	while (n) {
		chunk_len = 64;
		if (n < 64) {
			chunk_len = n;
		}

		saf_pkt.len = (uint32_t)chunk_len;

		rc = espi_saf_flash_write(espi_saf_dev, &saf_pkt);
		if (rc != 0) {
			LOG_INF("%s: error = %d: erase fail spi_addr = 0x%X",
				__func__, rc, spi_addr);
			return rc;
		}

		saf_pkt.flash_addr += chunk_len;
		saf_pkt.buf += chunk_len;
		n -= chunk_len;
	}

	return progsz;
}

static void pr_espi_saf_regs(void)
{
	struct espi_iom_regs * const espi_iom = (struct espi_iom_regs *)ESPI_BASE_ADDR;
	struct mchp_espi_saf * const espi_saf = (struct mchp_espi_saf *)SAF_BASE_ADDR;
	uint32_t addr, val;

	addr = (uint32_t)&espi_iom->RPMC_OP1_ODC;
	val = espi_iom->RPMC_OP1_ODC;
	LOG_INF("eSPI RPMC_OP1_ODC    @ %x = %x", addr, val);

	addr = (uint32_t)&espi_iom->RPMC_OP1_NC;
	val = espi_iom->RPMC_OP1_NC;
	LOG_INF("eSPI RPMC_OP1_NC     @ %x = %x", addr, val);

	addr = (uint32_t)&espi_iom->RPMC_OP1_NC_IMG;
	val = espi_iom->RPMC_OP1_NC_IMG;
	LOG_INF("eSPI RPMC_OP1_NC_IMG @ %x = %x", addr, val);

	addr = (uint32_t)&espi_saf->SAF_ECP_CMD;
	val = espi_saf->SAF_ECP_CMD;
	LOG_INF("eSPI SAF_ECP_CMD @ %x = %x", addr, val);

	addr = (uint32_t)&espi_saf->SAF_ECP_FLAR;
	val = espi_saf->SAF_ECP_FLAR;
	LOG_INF("eSPI SAF_ECP_FLAR @ %x = %x", addr, val);

	addr = (uint32_t)&espi_saf->SAF_ECP_BFAR;
	val = espi_saf->SAF_ECP_BFAR;
	LOG_INF("eSPI SAF_ECP_BFAR @ %x = %x", addr, val);

	addr = (uint32_t)&espi_saf->SAF_ECP_STATUS;
	val = espi_saf->SAF_ECP_STATUS;
	LOG_INF("eSPI SAF_ECP_STATUS @ %x = %x", addr, val);

	addr = (uint32_t)&espi_saf->SAF_ECP_INTEN;
	val = espi_saf->SAF_ECP_INTEN;
	LOG_INF("eSPI SAF_ECP_INTEN @ %x = %x", addr, val);

	addr = (uint32_t)&espi_saf->SAF_FL_CFG_SIZE_LIM;
	val = espi_saf->SAF_FL_CFG_SIZE_LIM;
	LOG_INF("eSPI SAF_FL_CFG_SIZE_LIM @ %x = %x", addr, val);

	addr = (uint32_t)&espi_saf->SAF_FL_CFG_THRH;
	val = espi_saf->SAF_FL_CFG_THRH;
	LOG_INF("eSPI SAF_FL_CFG_THRH @ %x = %x", addr, val);

	addr = (uint32_t)&espi_saf->SAF_FL_CFG_MISC;
	val = espi_saf->SAF_FL_CFG_MISC;
	LOG_INF("eSPI SAF_FL_CFG_MISC @ %x = %x", addr, val);

	addr = (uint32_t)&espi_saf->SAF_CFG_CS0_OPD;
	val = espi_saf->SAF_CFG_CS0_OPD;
	LOG_INF("eSPI SAF_CFG_CS0_OPD @ %x = %x", addr, val);

	addr = (uint32_t)&espi_saf->SAF_CFG_CS1_OPD;
	val = espi_saf->SAF_CFG_CS1_OPD;
	LOG_INF("eSPI SAF_CFG_CS1_OPD @ %x = %x", addr, val);

	addr = (uint32_t)&espi_saf->SAF_RPMC_OP2_ESPI_RES;
	val = espi_saf->SAF_RPMC_OP2_ESPI_RES;
	LOG_INF("eSPI SAF_RPMC_OP2_ESPI_RES @ %x = %x", addr, val);

	addr = (uint32_t)&espi_saf->SAF_RPMC_OP2_EC0_RES;
	val = espi_saf->SAF_RPMC_OP2_EC0_RES;
	LOG_INF("eSPI SAF_RPMC_OP2_EC0_RES @ %x = %x", addr, val);

	addr = (uint32_t)&espi_saf->SAF_RPMC_OP2_EC1_RES;
	val = espi_saf->SAF_RPMC_OP2_EC1_RES;
	LOG_INF("eSPI SAF_RPMC_OP2_EC1_RES @ %x = %x", addr, val);
}

int espi_saf_test1(uint32_t spi_addr)
{
	int rc, retries;
	bool erased;
	uint32_t n, saddr, progsz, chunksz;

	if (spi_addr & 0xfffU) {
		LOG_INF("%s: SPI address 0x%08x not 4KB aligned", __func__,
			spi_addr);
		spi_addr &= ~(4096U-1U);
		LOG_INF("%s: Aligned SPI address to 0x%08x", __func__,
			spi_addr);
	}

	pr_espi_saf_regs();

	memset(safbuf, 0x55, sizeof(safbuf));
	memset(safbuf2, 0, sizeof(safbuf2));

	erased = false;
	retries = 3;
	while (!erased && (retries-- > 0)) {
		/* read 4KB sector at 0 */
		rc = saf_read(spi_addr, safbuf, 4096);
		if (rc != 4096) {
			LOG_INF("%s: error=%d Read 4K sector at 0x%X failed",
				__func__, rc, spi_addr);
			return rc;
		}

		rc = 0;
		for (n = 0; n < 4096U; n++) {
			if (safbuf[n] != 0xffUL) {
				rc = -1;
				break;
			}
		}

		if (rc == 0) {
			LOG_INF("4KB sector at 0x%x is in erased state. "
				"Continue tests", spi_addr);
			erased = true;
		} else {
			LOG_INF("4KB sector at 0x%x not in erased state. "
				"Send 4K erase.", spi_addr);
			rc = saf_erase_block(spi_addr, SAF_ERASE_4K);
			if (rc != 0) {
				LOG_INF("SAF erase block at 0x%x returned "
					"error %d", spi_addr, rc);
				return rc;
			}
		}
	}

	if (!erased) {
		LOG_INF("%s: Could not erase 4KB sector at 0x%08x",
			__func__, spi_addr);
		return -1;
	}

	/* Page program test pattern every 256 bytes = 0,1,...,255 */
	for (n = 0; n < 4096U; n++) {
		safbuf[n] = n % 256U;
	}

	/* SPI flash sector erase size is 4KB, page program is 256 bytes */
	progsz = 4096U;
	chunksz = 256U;
	saddr = spi_addr;
	n = 0;
	const uint8_t *src = (const uint8_t *)safbuf;

	LOG_INF("%s: Program 4KB sector at 0x%X", __func__, saddr);

	while (n < progsz) {
		rc = saf_page_prog(saddr, (const uint8_t *)src,
				   (int)chunksz);
		if (rc != chunksz) {
			LOG_INF("saf_page_prog error=%d at 0x%X", rc,
				saddr);
			break;
		}
		saddr += chunksz;
		n += chunksz;
		src += chunksz;
	}

	/* read back and check */
	LOG_INF("%s: Read back 4K sector at 0x%X", __func__, spi_addr);

	rc = saf_read(spi_addr, safbuf2, progsz);
	if (rc == progsz) {
		rc = memcmp(safbuf, safbuf2, progsz);
		if (rc == 0) {
			LOG_INF("%s: Read back match: PASS", __func__);
		} else {
			LOG_INF("%s: Read back mismatch: FAIL", __func__);
		}
	} else {
		LOG_INF("%s: Read back 4K error=%d", __func__, rc);
		return rc;
	}

	return rc;
}

/* Send RPMC OP1 Update HMAC Key command to SAF attached flash using the
 * SAF EC Portal.
 * HMAC key does not survive flash reset/power cycle.
 * SAF packet:
 * OP1, SubCmd=0x01, CounterAddr[7:0], RSVD[7:0], KeyData[31:0], Signature[255:0]
 * Total packet length = 40 bytes
 * RPMC flash performs two HMAC-SHA-256 operations.
 * HMAC1 = HMAC-SHA-256(key=FlashRootKey[255:0], data=KeyData[31:0])
 * HMAC2 = HMAC-SHA-256(key=HMAC1,
 *                      data=OpCode[7:0] || SubCmd[7:0] || CounterAddr[7:0] ||
 *                            RSVD[7:0] || KeyData[31:0])
 */
#if SAF_RPMC_ENABLE_HMAC_UPDATE
static int espi_saf_rpmc_update_hmac_key(const uint8_t *hmac_key, uint32_t hksz)
{
	/* TODO */
	return 0;
}
#endif

/* The SAF driver only transmits/receives the raw RPMC data from the flash.
 * It is the caller's responsibity to compute check data is authentic using
 * the HMAC-SHA256 key provisioned in the RPMC flash device.
 * RPMC implements two opcodes:
 * OP1 is a write containing subcommand and parameters to the flash device
 *   subcmd 0x00 Write Root Key. Length = 64 bytes
 *     OP1 || 0x00 || CounterAddr[7:0] || RSVD[7:0] || RootKey[255:0] || TruncatedSig[223:0]
 *   subcmd 0x01 Update HMAC. Length = 40 bytes
 *     OP1 || 0x01 || CounterAddr[7:0] || RSVD[7:0] || KeyData[31:0] || Signature[255:0]
 *   subcmd 0x02 Increment Monotonic Counter. Length = 40 bytes
 *     OP1 || 0x02 || CounterAddr[7:0] || RSVD[7:0] || CounterData[31:0] || Signature[255:0]
 *   subcmd 0x03 Request Monotonic Counter. Length = 48 bytes
 *     OP1 || 0x03 || CounterAddr[7:0] || RSVD[7:0] || TAG[96:0] || Signature[255:0]
 *
 * OP2 reads status, 12-byte TAG, 4 byte counter, and 32 byte signature from the device.
 *  TX: OP2 || RSVD[7:0]
 *  RX: RPMC.Status[7:0] || TAG[96:0] || CounterData[31:0] || Signature[255:0]
 *  Length = 51
 *
 * OP1 opcode = 0x9B
 * OP2 opcode = 0x96
 *
 * For an unprovisioned RPMC Flash at power the Root Key has not been set and
 * the HMAC key is clear (power cycle/reset).
 * We expect RPMC extended status = 0x08 b[3]=1 HMAC Key register uninitialized.
 */
static int espi_saf_rpmc_test1(void)
{
	int ret = 0;
	struct espi_saf_packet pkt = { 0 };

	memset(saf_rpmc_buf, 0, sizeof(saf_rpmc_buf));

	saf_rpmc_buf[0] = 0x9bU;
	saf_rpmc_buf[1] = 0x03U;
	saf_rpmc_buf[2] = 0U; /* counter 0 */
	/* TODO - Need HMAC-SHA256 to update TAG and Signature fields */

	/* Request Monotinic Counter */
	pkt.flash_addr = ESPI_SAF_RPMC_ENCODE_PARAMS(0, MCHP_SAF_ECP_CMD_RPMC_OP1, 0x03U);
	pkt.buf = saf_rpmc_buf;
	pkt.len = 48U;

	ret = espi_saf_rpmc(espi_saf_dev, &pkt);
	if (ret) {
		LOG_INF("%s: OP1 error %d", __func__, ret);
		return ret;
	}

	/* Read Status/Data */
	memset(saf_rpmc_buf, 0x55, sizeof(saf_rpmc_buf));
	saf_rpmc_buf[0] = 0x96U;
	saf_rpmc_buf[1] = 0U;

	pkt.flash_addr = ESPI_SAF_RPMC_ENCODE_PARAMS(0, MCHP_SAF_ECP_CMD_RPMC_OP2, 0);
	pkt.buf = saf_rpmc_buf;
	pkt.len = 64U;

	ret = espi_saf_rpmc(espi_saf_dev, &pkt);
	if (ret) {
		LOG_INF("%s: OP2 error %d", __func__, ret);
		return ret;
	}

	LOG_INF("SAF RPMC test1 extended status = %x", saf_rpmc_buf[0]);

	return 0;
}
#endif /* CONFIG_ESPI_SAF */

static void host_warn_handler(uint32_t signal, uint32_t status)
{
	switch (signal) {
	case ESPI_VWIRE_SIGNAL_HOST_RST_WARN:
		LOG_INF("Host reset warning %d", status);
		if (!IS_ENABLED(CONFIG_ESPI_AUTOMATIC_WARNING_ACKNOWLEDGE)) {
			LOG_INF("HOST RST ACK %d", status);
			espi_send_vwire(espi_dev,
					ESPI_VWIRE_SIGNAL_HOST_RST_ACK,
					status);
		}
		break;
	case ESPI_VWIRE_SIGNAL_SUS_WARN:
		LOG_INF("Host suspend warning %d", status);
		if (!IS_ENABLED(CONFIG_ESPI_AUTOMATIC_WARNING_ACKNOWLEDGE)) {
			LOG_INF("SUS ACK %d", status);
			espi_send_vwire(espi_dev, ESPI_VWIRE_SIGNAL_SUS_ACK,
					status);
		}
		break;
	default:
		break;
	}
}

/* eSPI bus event handler */
static void espi_reset_handler(const struct device *dev,
			       struct espi_callback *cb,
			       struct espi_event event)
{
	if (event.evt_type == ESPI_BUS_RESET) {
		espi_rst_sts = event.evt_data;
		LOG_INF("eSPI BUS reset %d", event.evt_data);
	}
}

/* eSPI logical channels enable/disable event handler */
static void espi_ch_handler(const struct device *dev,
			    struct espi_callback *cb,
			    struct espi_event event)
{
	if (event.evt_type == ESPI_BUS_EVENT_CHANNEL_READY) {
		switch (event.evt_details) {
		case ESPI_CHANNEL_VWIRE:
			LOG_INF("VW channel event %x", event.evt_data);
			break;
		case ESPI_CHANNEL_FLASH:
			LOG_INF("Flash channel event %d", event.evt_data);
			break;
		case ESPI_CHANNEL_OOB:
			LOG_INF("OOB channel event %d", event.evt_data);
			break;
		default:
			LOG_ERR("Unknown channel event");
		}
	}
}

/* eSPI vwire received event handler */
static void vwire_handler(const struct device *dev, struct espi_callback *cb,
			  struct espi_event event)
{
	if (event.evt_type == ESPI_BUS_EVENT_VWIRE_RECEIVED) {
		switch (event.evt_details) {
		case ESPI_VWIRE_SIGNAL_PLTRST:
			LOG_INF("PLT_RST changed %d", event.evt_data);
			break;
		case ESPI_VWIRE_SIGNAL_SLP_S3:
		case ESPI_VWIRE_SIGNAL_SLP_S4:
		case ESPI_VWIRE_SIGNAL_SLP_S5:
			LOG_INF("SLP signal changed %d", event.evt_data);
			break;
		case ESPI_VWIRE_SIGNAL_SUS_WARN:
		case ESPI_VWIRE_SIGNAL_HOST_RST_WARN:
			host_warn_handler(event.evt_details,
					      event.evt_data);
			break;
		}
	}
}

/* eSPI peripheral channel notifications handler */
static void periph_handler(const struct device *dev, struct espi_callback *cb,
			   struct espi_event event)
{
	uint8_t periph_type;
	uint8_t periph_index;

	periph_type = EVENT_TYPE(event.evt_details);
	periph_index = EVENT_DETAILS(event.evt_details);

	switch (periph_type) {
	case ESPI_PERIPHERAL_DEBUG_PORT80:
		LOG_INF("Postcode %x", event.evt_data);
		break;
	case ESPI_PERIPHERAL_HOST_IO:
		LOG_INF("ACPI %x", event.evt_data);
		espi_remove_callback(espi_dev, &p80_cb);
		break;
	default:
		LOG_INF("%s periph 0x%x [%x]", __func__, periph_type,
			event.evt_data);
	}
}

int espi_init(void)
{
	int ret;
	/* Indicate to eSPI master simplest configuration: Single line,
	 * 20MHz frequency and only logical channel 0 and 1 are supported
	 */
	struct espi_cfg cfg = {
		.io_caps = ESPI_IO_MODE_SINGLE_LINE,
		.channel_caps = ESPI_CHANNEL_VWIRE | ESPI_CHANNEL_PERIPHERAL,
		.max_freq = ESPI_FREQ_20MHZ,
	};

	test_flags = 0U;

	/* If eSPI driver supports additional capabilities use them */
#ifdef CONFIG_ESPI_OOB_CHANNEL
	cfg.channel_caps |= ESPI_CHANNEL_OOB;
#endif
#ifdef CONFIG_ESPI_FLASH_CHANNEL
	cfg.channel_caps |= ESPI_CHANNEL_FLASH;
	cfg.io_caps |= ESPI_IO_MODE_QUAD_LINES;
	cfg.max_freq = ESPI_FREQ_25MHZ;
#endif

	ret = espi_config(espi_dev, &cfg);
	if (ret) {
		LOG_ERR("Failed to configure eSPI slave channels:%x err: %d",
			cfg.channel_caps, ret);
		return ret;
	}

	LOG_INF("eSPI slave configured successfully!");

	LOG_INF("eSPI test - callbacks initialization... ");
	espi_init_callback(&espi_bus_cb, espi_reset_handler, ESPI_BUS_RESET);
	espi_init_callback(&vw_rdy_cb, espi_ch_handler,
			   ESPI_BUS_EVENT_CHANNEL_READY);
	espi_init_callback(&vw_cb, vwire_handler,
			   ESPI_BUS_EVENT_VWIRE_RECEIVED);
	espi_init_callback(&p80_cb, periph_handler,
			   ESPI_BUS_PERIPHERAL_NOTIFICATION);
#ifdef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
			   espi_init_callback(&oob_cb, oob_rx_handler,
			   ESPI_BUS_EVENT_OOB_RECEIVED);
#endif
	LOG_INF("complete");

	LOG_INF("eSPI test - callbacks registration... ");
	espi_add_callback(espi_dev, &espi_bus_cb);
	espi_add_callback(espi_dev, &vw_rdy_cb);
	espi_add_callback(espi_dev, &vw_cb);
	espi_add_callback(espi_dev, &p80_cb);
#ifdef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
	espi_add_callback(espi_dev, &oob_cb);
#endif
	LOG_INF("complete");

	return ret;
}

int espi_test(void)
{
	int ret;

	/* Account for the time serial port is detected so log messages can
	 * be seen
	 */
	k_sleep(K_SECONDS(1));

	if (!device_is_ready(espi_dev)) {
		LOG_ERR("%s: device not ready.", espi_dev->name);
		return -ENODEV;
	}

#ifdef CONFIG_ESPI_SAF_XEC_V2
	qspi_dev = DEVICE_DT_GET(SPI_NODE);
	if (!device_is_ready(qspi_dev)) {
		LOG_ERR("%s: device not ready.", qspi_dev->name);
		return -ENODEV;
	}

	espi_saf_dev = DEVICE_DT_GET(ESPI_SAF_NODE);
	if (!device_is_ready(espi_saf_dev)) {
		LOG_ERR("%s: device not ready.", espi_saf_dev->name);
		return -ENODEV;
	}
#endif

	LOG_INF("Hello eSPI test %s", CONFIG_BOARD);

	espi_init();

#if defined(CONFIG_ESPI_SAF) || defined(CONFIG_ESPI_SAF_XEC_V2)
	/*
	 * eSPI SAF configuration must be after eSPI configuration.
	 * eSPI SAF EC portal flash tests before EC releases RSMRST# and
	 * Host de-asserts ESPI_RESET#.
	 */
	ret = spi_saf_init();
	if (ret) {
		LOG_ERR("Unable to configure %d:%s", ret, qspi_dev->name);
		return ret;
	}

	ret = espi_saf_init();
	if (ret) {
		LOG_ERR("Unable to configure %d:%s", ret, espi_saf_dev->name);
		return ret;
	}

	ret = espi_saf_test_pr1(&saf_pr_w25q128);
	if (ret) {
		LOG_INF("eSPI SAF test pr1 returned error %d", ret);
	}

	/* Activate SAF. QMSPI owned by SAF HW */
	ret = espi_saf_activate(espi_saf_dev);
	LOG_INF("%s: SAF activate = %d", __func__, ret);

	if (ret == 0) {
		ret = espi_saf_test1(SAF_SPI_TEST_ADDRESS);
		if (ret) {
			LOG_INF("eSPI SAF test1 returned error %d", ret);
		}

		if (test_flags & (TEST_FLAG_FLASH_KNOWN | TEST_FLAG_FLASH_HAS_RPMC)) {
			ret = espi_saf_rpmc_test1();
			if (ret) {
				LOG_INF("eSPI SAF RPMC test 1 returned error %d", ret);
			}
		}
	}
#endif

	/* Cleanup */
	k_sleep(K_SECONDS(1));
	espi_remove_callback(espi_dev, &espi_bus_cb);
	espi_remove_callback(espi_dev, &vw_rdy_cb);
	espi_remove_callback(espi_dev, &vw_cb);

	LOG_INF("eSPI sample completed err: %d", ret);

	return ret;
}

void main(void)
{
	espi_test();
}
