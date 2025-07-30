/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* BIOS Debug Port capture I/O 0x80 and alias
 * BDP can capture a one, two, and four byte I/O write cycles to a configurable
 * x86 I/O address range plus a one byte I/O write capture of an alias I/O
 * address.
 */
#define DT_DRV_COMPAT microchip_mec5_espi_bdp

#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/arch/common/ffs.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi/espi_mchp_mec5.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/espi/mchp-mec5-espi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include "espi_mchp_mec5_private.h"

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_retval.h>
#include <mec_bdp_api.h>
#include <mec_espi_api.h>
#include <mec_pcr_api.h>

LOG_MODULE_REGISTER(espi_bdp, CONFIG_ESPI_LOG_LEVEL);

/* #define MEC_BDP_DEBUG */

#define MEC_BDP_FIFO_SIZE 32u /* FIFO size in bytes */

struct mec5_bdp_devcfg {
	struct mec_bdp_regs *regs;
	const struct device *parent;
	uint16_t host_io_base;
	uint16_t host_io_alias;
	uint8_t alias_byte_lane;
	uint8_t ldn;
	uint8_t ldn_alias;
	uint8_t fifo_thr;
	void (*irq_config_func)(void);
};

struct mec5_bdp_data {
#ifdef MEC_BDP_DEBUG
	uint32_t isr_count;
#endif
	struct espi_ld_host_addr ha;
	struct host_io_data hiod;
	mchp_espi_pc_bdp_callback_t cb;
	void *cb_data;
	struct host_io_data cb_hiod;
};

const uint8_t fifo_thr_xlat_tbl[] = {
	1u, 4u, 8u, 16u, 20u, 24u, 28u, 30u
};

static uint32_t mec_bdp_fifo_thr_to_bytes(uint8_t fifo_thr)
{
	if (fifo_thr >= 8u) {
		return 30u;
	}

	return (uint32_t)fifo_thr_xlat_tbl[fifo_thr];
}

static int mec_bdp_get_io(const struct device *dev, struct host_io_data *hiod, uint32_t *nread)
{
	const struct mec5_bdp_devcfg *const devcfg = dev->config;
	struct mec_bdp_regs *const regs = devcfg->regs;
	uint32_t da = 0, nr = 0;
	bool io_complete = false;
	uint8_t io_width = 0, io_msk = 0;

	if (hiod == NULL) {
		return -EINVAL;
	}

	da = regs->DATRB;
	nr++;

	while ((da & BIT(MEC_BDP_DATRB_NOT_EMPTY_Pos)) != 0) {
		uint8_t lane = (da & MEC_BDP_DATRB_LANE_Msk) >> MEC_BDP_DATRB_LANE_Pos;
		uint8_t iosz = (da & MEC_BDP_DATRB_LEN_Msk) >> MEC_BDP_DATRB_LEN_Pos;

		io_msk |= BIT(lane);
		hiod->data |= ((da & 0xffu) << lane);
		hiod->flags = 0;

		if (iosz == MEC_BDP_DATRB_LEN_IO8) {
			if (io_width == 0) {
				hiod->start_byte_lane = lane;
				io_width = 1u;
				io_complete = true;
			} else if (io_width == 2u) {
				io_complete = true; /* second byte of 16-bit cycle */
			} else if ((io_width == 4u) && (lane == 3u)) {
				io_complete = true; /* fourth byte of 32-bit cycles */
			}
		} else if (iosz == MEC_BDP_DATRB_LEN_IO16B0) {
			io_width = 2u;
			hiod->start_byte_lane = lane;
		} else if (iosz == MEC_BDP_DATRB_LEN_IO32B0) {
			io_width = 4u;
			hiod->start_byte_lane = lane;
		} else { /* invalid orphan byte. discard */
			io_complete = true;
			io_width = iosz;
			hiod->start_byte_lane = lane;
			hiod->flags |= BIT(MEC5_BDP_EVENT_OVERRUN_POS);
		}

		if (io_complete == true) {
			hiod->size = io_width;
			hiod->msk = io_msk;
			break;
		}

		da = regs->DATRB;
		nr++;
	}

	if (nread != NULL) {
		*nread = nr;
	}

	return 0;
}

static int mec5_bdp_set_callback(const struct device *dev,
				 mchp_espi_pc_bdp_callback_t callback,
				 void *user_data)
{
	struct mec5_bdp_data *const data = dev->data;
	unsigned int key = irq_lock();

	data->cb = callback;
	data->cb_data = user_data;

	irq_unlock(key);

	return 0;
}

static int mec5_bdp_intr_enable(const struct device *dev, int intr_en, uint32_t flags)
{
	const struct mec5_bdp_devcfg *const devcfg = dev->config;
	struct mec_bdp_regs *const regs = devcfg->regs;
	uint8_t ien = (intr_en != 0) ? 1 : 0;

	mec_hal_bdp_intr_en(regs, ien);

	return 0;
}

static int mec5_bdp_has_data(const struct device *dev)
{
	const struct mec5_bdp_devcfg *const devcfg = dev->config;
	struct mec_bdp_regs *const regs = devcfg->regs;

	if (mec_hal_bdp_fifo_not_empty(regs)) {
		return 1;
	}

	return 0;
}

static int mec5_bdp_get_data(const struct device *dev, struct host_io_data *hiod)
{
	int ret = mec_bdp_get_io(dev, hiod, NULL);

	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	return 0;
}

/*
 * No Overrun:
 *   Read 16-byte entries from FIFO and process each one (invoke callback)
 * Overrun:
 *   Read 16-byte entries from FIFO and process each one (invoke callback)
 *   Does callback have a way to pass overrun status indicating data was lost?
 *
 * Each 16-byte entry indicates the I/O cycles size and byte lane.
 * If the size encoding is 0x3 (invalid) then we should discard the data.
 * Also can we get into a situation where the Host is writing I/O so fast
 * we can't empty the FIFO fast enough? We do not want to get stuck in this ISR.
 * The FIFO has 32 entries. We should put number of FIFO entries in DT.
 */
static void mec5_bdp_isr(const struct device *dev)
{
	const struct mec5_bdp_devcfg *const devcfg = dev->config;
	struct mec_bdp_regs *const regs = devcfg->regs;
	struct mec5_bdp_data *data = dev->data;
	struct host_io_data *hiod = &data->cb_hiod;
	int ret = 0;
	uint32_t nr = 0, fifo_thr_nb = 0, cap_nb = 0;

#ifdef MEC_BDB_DEBUG
	data->isr_count++;
	LOG_DBG("ISR: BDP: cnt=%u", data->isr_count);
#endif
	fifo_thr_nb = mec_bdp_fifo_thr_to_bytes(devcfg->fifo_thr);

	while (cap_nb < fifo_thr_nb) {
		nr = 0;
		ret = mec_bdp_get_io(dev, hiod, &nr);
		if (ret != MEC_RET_OK) {
			break;
		}

		cap_nb += nr;

		if (data->cb != NULL) {
			data->cb(dev, hiod, data->cb_data);
		}
	}

	mec_hal_bdp_girq_status_clr(regs);
}

static const struct mchp_espi_pc_bdp_driver_api mec5_bdp_drv_api = {
	.intr_enable = mec5_bdp_intr_enable,
	.has_data = mec5_bdp_has_data,
	.get_data = mec5_bdp_get_data,
	.set_callback = mec5_bdp_set_callback,
};

/* Called by Zephyr kernel during driver initialization */
static int mec5_bdp_init(const struct device *dev)
{
	const struct mec5_bdp_devcfg *const devcfg = dev->config;
	struct mec_bdp_regs *const regs = devcfg->regs;
	uint32_t cfg_flags = 0;

#ifdef MEC_BDP_DEBUG
	data->isr_count = 0;
#endif

	cfg_flags = (((uint32_t)devcfg->fifo_thr << MEC5_BDP_CFG_FIFO_THRES_POS) &
		     MEC5_BDP_CFG_FIFO_THRES_MSK);
	cfg_flags |= BIT(MEC5_BDP_CFG_THRH_IEN_POS);

	if (devcfg->host_io_alias) {
		cfg_flags |= BIT(MEC5_BDP_CFG_ALIAS_EN_POS);
	}

	int ret = mec_hal_bdp_init(regs, cfg_flags);

	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	if (devcfg->irq_config_func) {
		devcfg->irq_config_func();
		mec_hal_bdp_girq_ctrl(regs, 1u);
	}

	return 0;
}

#define MEC5_DT_NODE(inst) DT_INST(inst, DT_DRV_COMPAT)

#define MEC5_DT_BDP_HA(inst) \
	DT_PROP_BY_PHANDLE_IDX(MEC5_DT_NODE(inst), host_infos, 0, host_address)
#define MEC5_DT_BDPA_HA(inst) \
	DT_PROP_BY_PHANDLE_IDX(MEC5_DT_NODE(inst), host_infos, 1, host_address)

#define MEC5_DT_BDP_LDN(inst) DT_PROP_BY_PHANDLE_IDX(MEC5_DT_NODE(inst), host_infos, 0, ldn)
#define MEC5_DT_BDPA_LDN(inst) DT_PROP_BY_PHANDLE_IDX(MEC5_DT_NODE(inst), host_infos, 1, ldn)

#define MEC5_DT_BDPA_ABL(inst) \
	DT_PROP_BY_PHANDLE_IDX(MEC5_DT_NODE(inst), host_infos, 1, bdp_host_alias_byte_lane)

#define MEC5_BDP_DEVICE(inst)							\
										\
	static void mec5_bdp_irq_config_func_##inst(void)			\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(inst),					\
			    DT_INST_IRQ(inst, priority),			\
			    mec5_bdp_isr,					\
			    DEVICE_DT_INST_GET(inst), 0);			\
		irq_enable(DT_INST_IRQN(inst));					\
	}									\
										\
	static struct mec5_bdp_data mec5_bdp_data_##inst;			\
										\
	static const struct mec5_bdp_devcfg mec5_bdp_dcfg_##inst = {		\
		.regs = (struct mec_bdp_regs *)DT_INST_REG_ADDR(inst),		\
		.parent = DEVICE_DT_GET(DT_INST_PHANDLE(inst, espi_parent)),	\
		.host_io_base = MEC5_DT_BDP_HA(inst),				\
		.host_io_alias = MEC5_DT_BDPA_HA(inst),				\
		.alias_byte_lane = MEC5_DT_BDPA_ABL(inst),			\
		.ldn = MEC5_DT_BDP_LDN(inst),					\
		.ldn_alias = MEC5_DT_BDPA_LDN(inst),				\
		.fifo_thr = DT_INST_ENUM_IDX_OR(inst, fifo_threshold, 0),       \
		.irq_config_func = mec5_bdp_irq_config_func_##inst,		\
	};									\
	DEVICE_DT_INST_DEFINE(inst, mec5_bdp_init, NULL,			\
			&mec5_bdp_data_##inst,					\
			&mec5_bdp_dcfg_##inst,					\
			POST_KERNEL, CONFIG_ESPI_INIT_PRIORITY,			\
			&mec5_bdp_drv_api);					\
										\

DT_INST_FOREACH_STATUS_OKAY(MEC5_BDP_DEVICE)
