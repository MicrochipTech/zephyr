/*
 * Copyright (c) 2024 Microchip Technology, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <soc.h>
#include <mec_acpi_ec_api.h>
#include <mec_espi_api.h>
#include <mec_pcr_api.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_taf.h>
#include <zephyr/drivers/espi/espi_mchp_mec5.h>
#include <zephyr/drivers/espi/espi_taf_mchp_mec5.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(app);

#include "app.h"
#include "espi_taf_sample.h"

#define ESPI_TAF0_NODE DT_NODELABEL(espi_taf0)

static const struct device *espi_taf_dev = DEVICE_DT_GET(ESPI_TAF0_NODE);

struct espi_callback espi_taf_cb_s;

static volatile bool espi_taf_cb_fired;
static volatile struct espi_event espi_taf_cb_event;
static volatile uint32_t espi_taf_cb_data;

#define TAF_TEST_BUF1_LEN 256
static uint8_t taf_test_buf1[TAF_TEST_BUF1_LEN];


/* TODO we need to keep tables for both RPMC and non-RPMC flashes.
 * Take advantage of Zephyr flash driver JEDEC-ID read to determine
 * which table to use.
 */

/* W25R256JV SPI flash SAF configuration.
 * Size is 32Mbytes, it requires no continuous mode prefix, or
 * other special SAF configuration.
 * IMPORTANT: Winbond RPMC SPI flash does not support flash suspend
 * during OP1 operations. We must specify MCHP_FLASH_RPMC_NO_OP1_SUSPEND.
 */

#define W25R256JV_JEDEC_ID 0x001940efu

#define W25R256_RPMC_INFO_CS0 \
	(MCHP_FLASH_RPMC_OP1_DFLT | (4u << MCHP_FLASH_RPMC_OP1_NCTR_POS) | \
	 MCHP_FLASH_RPMC_OP1_FLAG_DISP_CS0_040 | MCHP_FLASH_RPMC_OP1_FLAG_DISP_CS0_PNP)

#define W25R256_RPMC_INFO_CS1 \
	(MCHP_FLASH_RPMC_OP1_DFLT | (4u << MCHP_FLASH_RPMC_OP1_NCTR_POS) | \
	 MCHP_FLASH_RPMC_OP1_FLAG_DISP_CS1_048 | MCHP_FLASH_RPMC_OP1_FLAG_DISP_CS1_PNP)

/* EVB Shared SPI dongle has two W25R256JV (RPMC capable) SPI flash attached to CS0 and CS1 */

struct espi_taf_flash_cfg taf_flash_rpmc_cfgs[2] = {
	{
		.version = 3u,
		.cs = 0u,
		.flags = MCHP_FLASH_FLAG_ADDR32 | MCHP_FLASH_FLAG_RPMC_SR_DIS,
		.flashsz = (1024u * 1024u * 32u),
		.rd_freq_mhz = 24u,
		.freq_mhz = 12u,
		.poll2_mask = (uint16_t)0xFF7Fu,
		.cont_prefix = 0u,
		.opa = MEC_TAF_OPCODE_A(0x06u, 0x75u, 0x7Au, 0x05u),
		.opb = MEC_TAF_OPCODE_B(0x20u, 0x52u, 0xD8u, 0x02u),
		.opc = MEC_TAF_OPCODE_C(0xEBu, 0xFFu, 0xA5u, 0x35u),
		.opd = MEC_TAF_OPCODE_D(0xB9u, 0xABu, MCHP_FLASH_RPMC_OP2_DFLT, 0),
		.rpmc_info = W25R256_RPMC_INFO_CS0,
	},
	{
		.version = 3u,
		.cs = 1u,
		.flags = MCHP_FLASH_FLAG_ADDR32 | MCHP_FLASH_FLAG_RPMC_SR_DIS,
		.flashsz = (1024u * 1024u * 32u),
		.rd_freq_mhz = 24u,
		.freq_mhz = 12u,
		.poll2_mask = (uint16_t)0xFF7Fu,
		.cont_prefix = 0u,
		.opa = MEC_TAF_OPCODE_A(0x06u, 0x75u, 0x7Au, 0x05u),
		.opb = MEC_TAF_OPCODE_B(0x20u, 0x52u, 0xD8u, 0x02u),
		.opc = MEC_TAF_OPCODE_C(0xEBu, 0xFFu, 0xA5u, 0x35u),
		.opd = MEC_TAF_OPCODE_D(0xB9u, 0xABu, MCHP_FLASH_RPMC_OP2_DFLT, 0),
		.rpmc_info = W25R256_RPMC_INFO_CS1,
	},
};

/* EVB Shared SPI dongle has two W25Q128FV SPI flash attached to CS0 and CS1 */
#define W25Q128FV_JEDEC_ID 0x001840efu

struct espi_taf_flash_cfg taf_flash_cfgs[2] = {
	{
		.version = 3u,
		.cs = 0u,
		.flags = MCHP_FLASH_FLAG_PD_EN | MCHP_FLASH_FLAG_PD_EC_WK_EN,
		.flashsz = (1024u * 1024u * 16u),
		.rd_freq_mhz = 24u,
		.freq_mhz = 12u,
		.poll2_mask = (uint16_t)0xFF7Fu,
		.cont_prefix = 0u,
		.opa = MEC_TAF_OPCODE_A(0x06u, 0x75u, 0x7Au, 0x05u),
		.opb = MEC_TAF_OPCODE_B(0x20u, 0x52u, 0xD8u, 0x02u),
		.opc = MEC_TAF_OPCODE_C(0xEBu, 0xFFu, 0xA5u, 0x35u),
		.opd = MEC_TAF_OPCODE_D(0xB9u, 0xABu, 0, 0),
		.rpmc_info = 0u,
	},
	{
		.version = 3u,
		.cs = 1u,
		.flags = MCHP_FLASH_FLAG_PD_EN | MCHP_FLASH_FLAG_PD_EC_WK_EN,
		.flashsz = (1024u * 1024u * 16u),
		.rd_freq_mhz = 24u,
		.freq_mhz = 12u,
		.poll2_mask = (uint16_t)0xFF7Fu,
		.cont_prefix = 0u,
		.opa = MEC_TAF_OPCODE_A(0x06u, 0x75u, 0x7Au, 0x05u),
		.opb = MEC_TAF_OPCODE_B(0x20u, 0x52u, 0xD8u, 0x02u),
		.opc = MEC_TAF_OPCODE_C(0xEBu, 0xFFu, 0xA5u, 0x35u),
		.opd = MEC_TAF_OPCODE_D(0xB9u, 0xABu, 0, 0),
		.rpmc_info = 0u,
	},
};

const struct espi_taf_cfg test_espi_taf_cfg_11 = {
	.nflash_devices = 2u,
	.hwcfg = {
		.version = 3u,
		.qspi_freq_mhz = 12u,
		.qspi_cpha = 0u,
		.flags = (BIT(MEC_ESPI_TAF_HW_CFG_FLAG_FREQ_POS)
			  | BIT(MEC_ESPI_TAF_HW_CFG_FLAG_FORCE_RPMC_OP1_POS)
			  | BIT(MEC_ESPI_TAF_HW_CFG_FLAG_PFEN_POS)),
		.qtaps_sel = 0u,
		.flash_pd_timeout = MEC_TAF_PD_TIMEOUT_1000MS,
		.flash_pd_min_interval = MEC_TAF_PD_MIN_INTERVAL_10US,
		.qspi_cs_timing = 0u,
		.gen_descr_ids = MEC_TAF_GEN_DESCR_IDS(12u, 14u, 14u),
		.cs0_descr_ids = MEC_TAF_FL_DCFG(0, 2, 3),
		.cs1_descr_ids = MEC_TAF_FL_DCFG(6, 8, 9),
		.tag_map = { 0u, 0u, 0u },
		.qspi_descrs = {
			MEC_TAF_CM_RD_ADDR24_QUAD(0),
			MEC_TAF_CM_ENTRY_ADDR24_QUAD(3),
			MEC_TAF_CM_RD_ADDR24_QUAD(6),
			MEC_TAF_CM_ENTRY_ADDR24_QUAD(9),
			MEC_TAF_EXIT_CM_QUAD(12),
			MEC_TAF_POLL_STATUS(14),
		},
	},
	.flash_cfgs = taf_flash_cfgs,
};

const struct espi_taf_cfg test_espi_taf_cfg_01 = {
	.nflash_devices = 1u,
	.hwcfg = {
		.version = 3u,
		.qspi_freq_mhz = 12u,
		.qspi_cpha = 0u,
		.flags = (BIT(MEC_ESPI_TAF_HW_CFG_FLAG_FREQ_POS)
			  | BIT(MEC_ESPI_TAF_HW_CFG_FLAG_FORCE_RPMC_OP1_POS)
			  | BIT(MEC_ESPI_TAF_HW_CFG_FLAG_PFEN_POS)),
		.qtaps_sel = 0u,
		.flash_pd_timeout = MEC_TAF_PD_TIMEOUT_1000MS,
		.flash_pd_min_interval = MEC_TAF_PD_MIN_INTERVAL_10US,
		.qspi_cs_timing = 0u,
		.gen_descr_ids = MEC_TAF_GEN_DESCR_IDS(12u, 14u, 14u),
		.cs0_descr_ids = MEC_TAF_FL_DCFG(0, 2, 3),
		.cs1_descr_ids = MEC_TAF_FL_DCFG(6, 8, 9),
		.tag_map = { 0u, 0u, 0u },
		.qspi_descrs = {
			MEC_TAF_CM_RD_ADDR24_QUAD(0),
			MEC_TAF_CM_ENTRY_ADDR24_QUAD(3),
			MEC_TAF_CM_RD_ADDR24_QUAD(6),
			MEC_TAF_CM_ENTRY_ADDR24_QUAD(9),
			MEC_TAF_EXIT_CM_QUAD(12),
			MEC_TAF_POLL_STATUS(14),
		},
	},
	.flash_cfgs = taf_flash_cfgs,
};

const struct espi_taf_cfg test_espi_taf_cfg_10 = {
	.nflash_devices = 1u,
	.hwcfg = {
		.version = 3u,
		.qspi_freq_mhz = 12u,
		.qspi_cpha = 0u,
		.flags = (BIT(MEC_ESPI_TAF_HW_CFG_FLAG_FREQ_POS)
			  | BIT(MEC_ESPI_TAF_HW_CFG_FLAG_FORCE_RPMC_OP1_POS)
			  | BIT(MEC_ESPI_TAF_HW_CFG_FLAG_PFEN_POS)),
		.qtaps_sel = 0u,
		.flash_pd_timeout = MEC_TAF_PD_TIMEOUT_1000MS,
		.flash_pd_min_interval = MEC_TAF_PD_MIN_INTERVAL_10US,
		.qspi_cs_timing = 0u,
		.gen_descr_ids = MEC_TAF_GEN_DESCR_IDS(12u, 14u, 14u),
		.cs0_descr_ids = MEC_TAF_FL_DCFG(0, 2, 3),
		.cs1_descr_ids = MEC_TAF_FL_DCFG(6, 8, 9),
		.tag_map = { 0u, 0u, 0u },
		.qspi_descrs = {
			MEC_TAF_CM_RD_ADDR24_QUAD(0),
			MEC_TAF_CM_ENTRY_ADDR24_QUAD(3),
			MEC_TAF_CM_RD_ADDR24_QUAD(6),
			MEC_TAF_CM_ENTRY_ADDR24_QUAD(9),
			MEC_TAF_EXIT_CM_QUAD(12),
			MEC_TAF_POLL_STATUS(14),
		},
	},
	.flash_cfgs = &taf_flash_cfgs[1],
};

const struct espi_taf_cfg test_espi_taf_cfg_rpmc_11 = {
	.nflash_devices = 2u,
	.hwcfg = {
		.version = 3u,
		.qspi_freq_mhz = 12u,
		.qspi_cpha = 0u,
		.flags = (BIT(MEC_ESPI_TAF_HW_CFG_FLAG_FREQ_POS)
			  | BIT(MEC_ESPI_TAF_HW_CFG_FLAG_FORCE_RPMC_OP1_POS)
			  | BIT(MEC_ESPI_TAF_HW_CFG_FLAG_PFEN_POS)),
		.qtaps_sel = 0u,
		.flash_pd_timeout = MEC_TAF_PD_TIMEOUT_1000MS,
		.flash_pd_min_interval = MEC_TAF_PD_MIN_INTERVAL_10US,
		.qspi_cs_timing = 0u,
		.gen_descr_ids = MEC_TAF_GEN_DESCR_IDS(12u, 14u, 14u),
		.cs0_descr_ids = MEC_TAF_FL_DCFG(0, 2, 3),
		.cs1_descr_ids = MEC_TAF_FL_DCFG(6, 8, 9),
		.tag_map = { 0u, 0u, 0u },
		.qspi_descrs = {
			MEC_TAF_CM_RD_ADDR32_QUAD(0),
			MEC_TAF_CM_ENTRY_ADDR32_QUAD(3),
			MEC_TAF_CM_RD_ADDR32_QUAD(6),
			MEC_TAF_CM_ENTRY_ADDR32_QUAD(9),
			MEC_TAF_EXIT_CM_QUAD(12),
			MEC_TAF_POLL_STATUS(14),
		},
	},
	.flash_cfgs = taf_flash_rpmc_cfgs,
};

const struct espi_taf_cfg test_espi_taf_cfg_rpmc_01 = {
	.nflash_devices = 1u,
	.hwcfg = {
		.version = 3u,
		.qspi_freq_mhz = 12u,
		.qspi_cpha = 0u,
		.flags = (BIT(MEC_ESPI_TAF_HW_CFG_FLAG_FREQ_POS)
			  | BIT(MEC_ESPI_TAF_HW_CFG_FLAG_FORCE_RPMC_OP1_POS)
			  | BIT(MEC_ESPI_TAF_HW_CFG_FLAG_PFEN_POS)),
		.qtaps_sel = 0u,
		.flash_pd_timeout = MEC_TAF_PD_TIMEOUT_1000MS,
		.flash_pd_min_interval = MEC_TAF_PD_MIN_INTERVAL_10US,
		.qspi_cs_timing = 0u,
		.gen_descr_ids = MEC_TAF_GEN_DESCR_IDS(12u, 14u, 14u),
		.cs0_descr_ids = MEC_TAF_FL_DCFG(0, 2, 3),
		.cs1_descr_ids = MEC_TAF_FL_DCFG(6, 8, 9),
		.tag_map = { 0u, 0u, 0u },
		.qspi_descrs = {
			MEC_TAF_CM_RD_ADDR32_QUAD(0),
			MEC_TAF_CM_ENTRY_ADDR32_QUAD(3),
			MEC_TAF_CM_RD_ADDR32_QUAD(6),
			MEC_TAF_CM_ENTRY_ADDR32_QUAD(9),
			MEC_TAF_EXIT_CM_QUAD(12),
			MEC_TAF_POLL_STATUS(14),
		},
	},
	.flash_cfgs = taf_flash_rpmc_cfgs,
};

const struct espi_taf_cfg test_espi_taf_cfg_rpmc_10 = {
	.nflash_devices = 1u,
	.hwcfg = {
		.version = 3u,
		.qspi_freq_mhz = 12u,
		.qspi_cpha = 0u,
		.flags = (BIT(MEC_ESPI_TAF_HW_CFG_FLAG_FREQ_POS)
			  | BIT(MEC_ESPI_TAF_HW_CFG_FLAG_FORCE_RPMC_OP1_POS)
			  | BIT(MEC_ESPI_TAF_HW_CFG_FLAG_PFEN_POS)),
		.qtaps_sel = 0u,
		.flash_pd_timeout = MEC_TAF_PD_TIMEOUT_1000MS,
		.flash_pd_min_interval = MEC_TAF_PD_MIN_INTERVAL_10US,
		.qspi_cs_timing = 0u,
		.gen_descr_ids = MEC_TAF_GEN_DESCR_IDS(12u, 14u, 14u),
		.cs0_descr_ids = MEC_TAF_FL_DCFG(0, 2, 3),
		.cs1_descr_ids = MEC_TAF_FL_DCFG(6, 8, 9),
		.tag_map = { 0u, 0u, 0u },
		.qspi_descrs = {
			MEC_TAF_CM_RD_ADDR32_QUAD(0),
			MEC_TAF_CM_ENTRY_ADDR32_QUAD(3),
			MEC_TAF_CM_RD_ADDR32_QUAD(6),
			MEC_TAF_CM_ENTRY_ADDR32_QUAD(9),
			MEC_TAF_EXIT_CM_QUAD(12),
			MEC_TAF_POLL_STATUS(14),
		},
	},
	.flash_cfgs = &taf_flash_rpmc_cfgs[1],
};

static const struct espi_taf_pr w25r128_protect_regions[2] = {
	{
		.start = 0xe00000U,
		.size =  0x100000U,
		.req_bm_we = BIT(MCHP_TAF_REQ_FROM_HOST_PCH_ME),
		.req_bm_rd = BIT(MCHP_TAF_REQ_FROM_HOST_PCH_ME),
		.pr_num = 1U,
		.flags = (MCHP_TAF_PR_FLAG_ENABLE | MCHP_TAF_PR_FLAG_LOCK),
	},
	{
		.start = 0xf00000U,
		.size =  0x100000U,
		.req_bm_we = BIT(MCHP_TAF_REQ_FROM_HOST_PCH_LAN),
		.req_bm_rd = BIT(MCHP_TAF_REQ_FROM_HOST_PCH_LAN),
		.pr_num = 2U,
		.flags = (MCHP_TAF_PR_FLAG_ENABLE | MCHP_TAF_PR_FLAG_LOCK),
	},
};

static const struct espi_taf_protection taf_pr_w25r128 = {
	.nregions = 2U,
	.pregions = w25r128_protect_regions
};

const struct device *app_get_taf_device(void)
{
	return espi_taf_dev;
}

const struct espi_taf_protection *get_taf_rpmc_flash_pr(void)
{
	return &taf_pr_w25r128;
}

static void espi_taf_callback(const struct device *dev, struct espi_callback *cb,
			      struct espi_event evt)
{
	if (evt.evt_type == ESPI_BUS_TAF_NOTIFICATION) {
		if (evt.evt_details & ESPI_TAF_ECP_READ) {
			LOG_INF("eSPI TAF read done: 0x%0x", evt.evt_data);
		}
		if (evt.evt_details & ESPI_TAF_ECP_WRITE) {
			LOG_INF("eSPI TAF write done: 0x%0x", evt.evt_data);
		}
		if (evt.evt_details & ESPI_TAF_ECP_ERASE) {
			LOG_INF("eSPI TAF erase done: 0x%0x", evt.evt_data);
		}
		if (evt.evt_details & ESPI_TAF_ECP_RPMC_OP1) {
			LOG_INF("eSPI TAF RPMC OP1 done: 0x%0x", evt.evt_data);
		}
		if (evt.evt_details & ESPI_TAF_ECP_RPMC_OP2) {
			LOG_INF("eSPI TAF RPMC OP2 done: 0x%0x", evt.evt_data);
		}
		if (evt.evt_details & ESPI_TAF_HOST_PROTOCOL_ERR) {
			LOG_ERR("eSPI TAF error from Host: 0x%0x", evt.evt_data);
		}
	}

	espi_taf_cb_event.evt_type = evt.evt_type;
	espi_taf_cb_event.evt_details = evt.evt_details;
	espi_taf_cb_event.evt_data = evt.evt_data;
	espi_taf_cb_fired = true;
}

void espi_taf_cb_data_init(void)
{
	espi_taf_cb_fired = false;
	espi_taf_cb_event.evt_type = 0;
	espi_taf_cb_event.evt_details = 0;
	espi_taf_cb_event.evt_data = 0;
}

bool espi_taf_cb_has_fired(void)
{
	return espi_taf_cb_fired;
}

void espi_taf_cb_get_event(struct espi_event *ev)
{
	ev->evt_type = espi_taf_cb_event.evt_type;
	ev->evt_details = espi_taf_cb_event.evt_details;
	ev->evt_data = espi_taf_cb_event.evt_data;
}

/* cfg_flags bit[0] = Config flash attached to nCS0
 *           bit[1] = Config flash attached to nCS1
 */
int sample_espi_taf_config(uint32_t cfg_flags)
{
	struct espi_taf_cfg const *tcfg = NULL;
	uint32_t jedec_id = 0;
	int ret = 0;
	uint8_t flash_cfg_bm = (uint8_t)(cfg_flags & 0x03u);

	if (!flash_cfg_bm) {
		LOG_ERR("Sample eSPI TAF config has no flashes specified!");
		return -EINVAL;
	}

	if (!device_is_ready(espi_taf_dev)) {
		LOG_ERR("eSPI TAF device is not ready!");
		return -EIO;
	}

	espi_taf_cb_data_init();

	jedec_id = app_flash_jedec_id(0);
	if (jedec_id == W25R256JV_JEDEC_ID) {
		if (flash_cfg_bm == 0x01u) {
			tcfg = &test_espi_taf_cfg_rpmc_01;
		} else if (flash_cfg_bm == 0x02u) {
			tcfg = &test_espi_taf_cfg_rpmc_10;
		} else {
			tcfg = &test_espi_taf_cfg_rpmc_11;
		}
	} else {
		if (flash_cfg_bm == 0x01u) {
			tcfg = &test_espi_taf_cfg_01;
		} else if (flash_cfg_bm == 0x02u) {
			tcfg = &test_espi_taf_cfg_10;
		} else {
			tcfg = &test_espi_taf_cfg_11;
		}
	}

	ret = espi_taf_config(espi_taf_dev, tcfg);
	if (ret) {
		LOG_ERR("eSPI TAF configuration returned error (%d)", ret);
		return -EIO;
	}

	ret = espi_taf_set_protection_regions(espi_taf_dev, &taf_pr_w25r128);
	if (ret) {
		LOG_ERR("eSPI TAF set PR error (%d)", ret);
		return -EIO;
	}

	espi_taf_init_callback(&espi_taf_cb_s, espi_taf_callback, ESPI_BUS_TAF_NOTIFICATION);

	ret = espi_taf_add_callback(espi_taf_dev, &espi_taf_cb_s);
	if (ret) {
		LOG_ERR("Failed to add eSPI TAF callback: %d", ret);
		return ret;
	}

	ret = espi_taf_activate(espi_taf_dev);
	if (ret) {
		LOG_ERR("eSPI TAF activate returned error (%d)", ret);
		return -EIO;
	}

	LOG_INF("eSPI TAF is Activated");

	return 0;
}


int sample_espi_taf_read_test1(uint32_t flash_addr, uint8_t *expected_data,
			       uint32_t expected_data_len)
{
	struct espi_taf_packet taf_pkt = {0};
	uint32_t flags = 0;
	int ret = 0;

	if (!expected_data || (expected_data_len > sizeof(taf_test_buf1))) {
		return -EINVAL;
	}

	memset(taf_test_buf1, 0x55, sizeof(taf_test_buf1));

	taf_pkt.flash_addr = flash_addr,
	taf_pkt.buf = taf_test_buf1,
	taf_pkt.len = expected_data_len,

	ret = espi_taf_flash_read(espi_taf_dev, &taf_pkt, flags);
	if (ret) {
		return ret;
	}

	ret = memcmp(taf_test_buf1, expected_data, expected_data_len);
	if (ret) {
		LOG_ERR("TAF flash read data mismatch!");
		LOG_HEXDUMP_ERR(expected_data, expected_data_len, "Expected data");
		LOG_HEXDUMP_ERR(taf_test_buf1, expected_data_len, "TAF read data");
		ret = -EIO;
	}

	return ret;
}

int sample_espi_taf_erase_test1(uint32_t flash_addr, uint32_t erase_size)
{
	struct espi_taf_packet taf_pkt = {0};
	uint32_t offset = 0, pktlen = 0, flags = 0;
	int ret = 0;

	if ((erase_size != 4096u) && (erase_size != (32u * 1024u))
		&& (erase_size != (64u * 1024u))) {
		return -EINVAL;
	}

	memset(taf_test_buf1, 0x55, sizeof(taf_test_buf1));

	taf_pkt.flash_addr = flash_addr;
	taf_pkt.buf = NULL;
	taf_pkt.len = erase_size;

	ret = espi_taf_flash_erase(espi_taf_dev, &taf_pkt, flags);
	if (ret) {
		return ret;
	}

	while (offset < erase_size) {

		pktlen = TAF_TEST_BUF1_LEN;
		if ((erase_size - offset) < pktlen) {
			pktlen = erase_size - offset;
		}

		taf_pkt.flash_addr = flash_addr + offset;
		taf_pkt.buf = taf_test_buf1;
		taf_pkt.len = pktlen;

		ret = espi_taf_flash_read(espi_taf_dev, &taf_pkt, flags);
		if (ret) {
			return ret;
		}

		for (uint32_t n = 0; n > pktlen; n++) {
			if (taf_test_buf1[n] != 0xffu) {
				LOG_ERR("App TAF erase failure at offset 0x%0x", offset);
				return -EIO;
			}
		}

		offset += pktlen;
	}

	return ret;
}

int sample_espi_taf_write_test1(uint32_t flash_addr, uint8_t *data, uint32_t data_len)
{
	struct espi_taf_packet taf_pkt = {0};
	uint32_t flags = 0;
	int ret = 0;

	if (!data || !data_len) {
		return -EINVAL;
	}

	taf_pkt.flash_addr = flash_addr;
	taf_pkt.buf = data;
	taf_pkt.len = data_len;

	ret = espi_taf_flash_write(espi_taf_dev, &taf_pkt, flags);
	if (ret) {
		return ret;
	}

	memset(taf_test_buf1, 0x55, data_len);

	taf_pkt.flash_addr = flash_addr;
	taf_pkt.buf = taf_test_buf1;
	taf_pkt.len = data_len;

	ret = espi_taf_flash_read(espi_taf_dev, &taf_pkt, flags);
	if (ret) {
		LOG_ERR("eSPI TAF write test 1 read back API error (%d)", ret);
		return ret;
	}

	ret = memcmp(data, taf_test_buf1, data_len);
	if (ret) {
		return -EIO;
	}

	return ret;
}
