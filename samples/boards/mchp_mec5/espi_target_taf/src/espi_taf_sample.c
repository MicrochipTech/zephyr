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

#include "espi_taf_sample.h"

#define ESPI_TAF0_NODE DT_NODELABEL(espi_taf0)

static const struct device *espi_taf_dev = DEVICE_DT_GET(ESPI_TAF0_NODE);

struct espi_taf_flash_cfg taf_flash_cfgs[2] = {
	{
		.version = 3u,
		.cs = 0u,
		.flags = MCHP_FLASH_FLAG_PD_EN | MCHP_FLASH_FLAG_PD_EC_WK_EN,
		.flashsz = (1024u * 1024u * 16u),
		.rd_freq_mhz = 24u,
		.freq_mhz = 12u,
		.opa = FL_WB128_OPA,
		.opb = FL_WB128_OPB,
		.opc = FL_WB128_OPC,
		.opd = FL_WB128_OPD,
		.rpmc_info = 0u,
		.poll2_mask = (uint16_t)FL_WB128_POLL2_MASK,
		.cont_prefix = 0u,
		.cs_cfg_descr_ids = (uint16_t)FL_WB128_CS0_CFG_DESCR_IDS,
		.descr = {
			FL_WB128_CS0_CM_RD0, FL_WB128_CS0_CM_RD1, FL_WB128_CS0_CM_RD2,
			FL_WB128_CS0_CM_ENTER0, FL_WB128_CS0_CM_ENTER1, FL_WB128_CS0_CM_ENTER2
		},
	},
	{
		.version = 3u,
		.cs = 1u,
		.flags = MCHP_FLASH_FLAG_PD_EN | MCHP_FLASH_FLAG_PD_EC_WK_EN,
		.flashsz = (1024u * 1024u * 16u),
		.rd_freq_mhz = 24u,
		.freq_mhz = 12u,
		.opa = FL_WB128_OPA,
		.opb = FL_WB128_OPB,
		.opc = FL_WB128_OPC,
		.opd = FL_WB128_OPD,
		.rpmc_info = 0u,
		.poll2_mask = (uint16_t)FL_WB128_POLL2_MASK,
		.cont_prefix = 0u,
		.cs_cfg_descr_ids = (uint16_t)FL_WB128_CS1_CFG_DESCR_IDS,
		.descr = {
			FL_WB128_CS1_CM_RD0, FL_WB128_CS1_CM_RD1, FL_WB128_CS1_CM_RD2,
			FL_WB128_CS1_CM_ENTER0, FL_WB128_CS1_CM_ENTER1, FL_WB128_CS1_CM_ENTER2
		},
	},
};

struct espi_taf_cfg test_espi_taf_cfg = {
	.nflash_devices = 1u,
	.hwcfg = {
		.version = 3u,
		.qspi_freq_mhz = 12u,
		.qspi_cpha = 0u,
		.flags = BIT(MEC_ESPI_TAF_HW_CFG_FLAG_FREQ_POS),
		.qtaps_sel = 0u,
		.flash_pd_timeout = 0u, // 0x1b8
		.flash_pd_min_interval = 0u, // 0x1cc
		.qspi_cs_timing = 0u,
		.generic_descr = {
			FL_WB128_CM_EXIT0, FL_WB128_CM_EXIT1,
			FL_WB128_POLL_STS0, FL_WB128_POLL_STS1
		},
		.tag_map = { 0u, 0u, 0u },
	},
	.flash_cfgs = taf_flash_cfgs,
};

int sample_espi_taf_config(uint32_t cfg_flags)
{
	int ret = 0;

	if (!device_is_ready(espi_taf_dev)) {
		LOG_ERR("eSPI TAF device is not ready!");
		return -EIO;
	}

	test_espi_taf_cfg.flash_cfgs = taf_flash_cfgs;
	test_espi_taf_cfg.nflash_devices = 1u;
	test_espi_taf_cfg.hwcfg.version = 3u;
	test_espi_taf_cfg.hwcfg.flags = 0; /* what flags? */
	test_espi_taf_cfg.hwcfg.qspi_freq_mhz = 12u;
	test_espi_taf_cfg.hwcfg.qspi_cpha = 0u;
	test_espi_taf_cfg.hwcfg.qtaps_sel = 0u;
	test_espi_taf_cfg.hwcfg.flash_pd_timeout = 0u;
	test_espi_taf_cfg.hwcfg.flash_pd_min_interval = 0u;
	test_espi_taf_cfg.hwcfg.qspi_cs_timing = 0u;
	test_espi_taf_cfg.hwcfg.generic_descr[0] = 0;
	test_espi_taf_cfg.hwcfg.generic_descr[1] = 0;
	test_espi_taf_cfg.hwcfg.generic_descr[2] = 0;
	test_espi_taf_cfg.hwcfg.generic_descr[3] = 0;
	test_espi_taf_cfg.hwcfg.tag_map[0] = 0;
	test_espi_taf_cfg.hwcfg.tag_map[1] = 0;
	test_espi_taf_cfg.hwcfg.tag_map[2] = 0;

	ret = espi_taf_config(espi_taf_dev, &test_espi_taf_cfg);
	if (ret) {
		LOG_ERR("eSPI TAF configuration returned error (%d)", ret);
		return -EIO;
	}

	ret = espi_taf_activate(espi_taf_dev);
	if (ret) {
		LOG_ERR("eSPI TAF activate returned error (%d)", ret);
		return -EIO;
	}

	return 0;
}
