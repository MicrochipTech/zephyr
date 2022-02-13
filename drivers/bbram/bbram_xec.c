/*
 * Copyright (c) 2021 Microchip Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_bbram

#include <drivers/bbram.h>
#include <errno.h>
#include <soc.h>
#include <sys/util.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(bbram, CONFIG_BBRAM_LOG_LEVEL);

#define XEC_VBAT_REGS_ADDR \
	(DT_REG_ADDR_BY_NAME(DT_NODELABEL(pcr), vbatr))

/** Device config */
struct bbram_xec_config {
	struct vbatr_regs *const vbr;
	/** BBRAM base address */
	uint8_t *vbm;
	/** BBRAM size (Unit:bytes) */
	int size;
};

static int bbram_xec_check_invalid(const struct device *dev)
{
	const struct bbram_xec_config * const dcfg = dev->config;
	struct vbatr_regs *const vbr = dcfg->vbr;

	if (vbr->PFRS & BIT(MCHP_VBATR_PFRS_VBAT_RST_POS)) {
		vbr->PFRS |= BIT(MCHP_VBATR_PFRS_VBAT_RST_POS);
		LOG_ERR("VBAT power rail failure");
		return -EFAULT;
	}

	return 0;
}

static int bbram_xec_get_size(const struct device *dev, size_t *size)
{
	const struct bbram_xec_config * const dcfg = dev->config;

	*size = dcfg->size;
	return 0;
}

static int bbram_xec_read(const struct device *dev, size_t offset, size_t size,
			  uint8_t *data)
{
	const struct bbram_xec_config * const dcfg = dev->config;

	if (size < 1 || offset + size > dcfg->size) {
		LOG_ERR("Invalid params");
		return -EFAULT;
	}

	bytecpy(data, (dcfg->vbm + offset), size);
	return 0;
}

static int bbram_xec_write(const struct device *dev, size_t offset, size_t size,
			       const uint8_t *data)
{
	const struct bbram_xec_config * const dcfg = dev->config;

	if (size < 1 || offset + size > dcfg->size) {
		LOG_ERR("Invalid params");
		return -EFAULT;
	}

	bytecpy((dcfg->vbm + offset), data, size);
	return 0;
}

static const struct bbram_driver_api bbram_xec_driver_api = {
	.check_invalid = bbram_xec_check_invalid,
	.get_size = bbram_xec_get_size,
	.read = bbram_xec_read,
	.write = bbram_xec_write,
};

static int bbram_xec_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static const struct bbram_xec_config bbram_cfg = {
	.vbr = (struct vbatr_regs *)(XEC_VBAT_REGS_ADDR),
	.vbm = (uint8_t *)(DT_INST_REG_ADDR(0)),
	.size = DT_INST_REG_SIZE(0),
};

DEVICE_DT_INST_DEFINE(0, bbram_xec_init, NULL, NULL,
		      &bbram_cfg,
		      PRE_KERNEL_1, CONFIG_BBRAM_INIT_PRIORITY,
		      &bbram_xec_driver_api);
