/*
 * Copyright (c) 2024 Microchip Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_bbram

#include <zephyr/drivers/bbram.h>
#include <errno.h>
#include <soc.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bbram_mec5, CONFIG_BBRAM_LOG_LEVEL);

/* Microchip MEC5 HAL */
#include <mec_vbat_api.h>

struct bbram_mec5_devcfg {
	uint8_t *vbmem_base;
	int size;
};

static int bbram_mec5_check_invalid(const struct device *dev)
{
	uint32_t vbat_status = mec_hal_vbat_pfrs();

	if (vbat_status & BIT(MEC_VBAT_PFRS_VBAT_RST_POS)) {
		mec_hal_vbat_pfrs_clear(BIT(MEC_VBAT_PFRS_VBAT_RST_POS));
		LOG_ERR("VBAT power rail failure");
		return -EFAULT;
	}

	return 0;
}

static int bbram_mec5_get_size(const struct device *dev, size_t *size)
{
	const struct bbram_mec5_devcfg *devcfg = dev->config;

	if (!size) {
		return -EINVAL;
	}

	*size = devcfg->size;

	return 0;
}

static bool bbram_mec5_access_range_valid(size_t offset, size_t size, size_t bbramsz)
{
	if ((size < 1) || ((offset + size) > bbramsz)) {
		return false;
	}

	return true;
}

static int bbram_mec5_read(const struct device *dev, size_t offset,
			   size_t size, uint8_t *data)
{
	const struct bbram_mec5_devcfg *devcfg = dev->config;

	if (!bbram_mec5_access_range_valid(offset, size, devcfg->size) || !data) {
		LOG_ERR("Invalid params");
		return -EFAULT;
	}

	bytecpy(data, (const void *)(devcfg->vbmem_base + offset), size);

	return 0;
}

static int bbram_mec5_write(const struct device *dev, size_t offset,
			    size_t size, const uint8_t *data)
{
	const struct bbram_mec5_devcfg *devcfg = dev->config;

	if (!bbram_mec5_access_range_valid(offset, size, devcfg->size) || !data) {
		LOG_ERR("Invalid params");
		return -EFAULT;
	}

	bytecpy(devcfg->vbmem_base + offset, data, size);

	return 0;
}

static const struct bbram_driver_api bbram_mec5_driver_api = {
	.check_invalid = bbram_mec5_check_invalid,
	.get_size = bbram_mec5_get_size,
	.read = bbram_mec5_read,
	.write = bbram_mec5_write,
};

#define MEC5_BBRAM_INIT(inst)							\
	static const struct bbram_mec5_devcfg bbram_mec5_devcfg_##inst = {	\
		.vbmem_base = (uint8_t *)(DT_INST_REG_ADDR(inst)),		\
		.size = DT_INST_REG_SIZE(inst),					\
	};									\
	DEVICE_DT_INST_DEFINE(inst,						\
			      NULL,						\
			      NULL,						\
			      NULL,						\
			      &bbram_mec5_devcfg_##inst,			\
			      PRE_KERNEL_1, CONFIG_BBRAM_INIT_PRIORITY,		\
			      &bbram_mec5_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MEC5_BBRAM_INIT);
