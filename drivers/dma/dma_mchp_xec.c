/*
 * Copyright (c) 2022 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "reg/mec_dma.h"
#define DT_DRV_COMPAT microchip_xec_dmac

#include <device.h>
#include <soc.h>
#include <drivers/clock_control/mchp_xec_clock_control.h>
#include <drivers/dma.h>
#include <drivers/interrupt_controller/intc_mchp_xec_ecia.h>
#include <dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <pm/device.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(dma_mchp_xec, CONFIG_DMA_LOG_LEVEL);

#define XEC_ABORT_WAIT_LOOPS	32

#define XEC_MAX_DMA_CHAN	DT_INST_PROP(0, dma_channels)

struct dma_xec_chan_config {
	uint8_t irqn;
	uint8_t irq_pri;
	uint8_t girq;
	uint8_t girq_pos;
};

struct dma_xec_config {
	struct dma_regs * const regs;
	uint8_t dma_channels;
	uint8_t dma_requests;
	uint8_t pcr_idx;
	uint8_t pcr_pos;
	struct dma_xec_chan_config chcfg[XEC_MAX_DMA_CHAN];
};

struct dma_xec_channel {
	uint32_t mstart;
	uint32_t mend;
	uint32_t dstart;
	uint32_t block_count;
	struct dma_block_config *block;
	dma_callback_t cb;
	void *user_data;
};

struct dma_xec_data {
	struct dma_xec_channel channels[XEC_MAX_DMA_CHAN];
};

static bool is_dma_data_size_valid(uint32_t datasz)
{
	if ((datasz == 1U) || (datasz == 2U) || (datasz == 4U)) {
		return true;
	}

	return false;
}

/* HW requires if unit size is 2 or 4 bytes the source/destination addresses
 * to be aligned >= 2 or 4 bytes.
 */
static bool is_data_aligned(uint32_t src, uint32_t dest, uint32_t unitsz)
{
	if (unitsz == 1) {
		return true;
	}

	if ((src | dest) & (unitsz - 1U)) {
		return false;
	}

	return true;
}

static void xec_dma_chan_clr(struct dma_chan_regs * const chregs,
			     const struct dma_xec_chan_config *chcfg)
{
	chregs->ACTV = 0U;
	chregs->CTRL = 0U;
	chregs->MSTART = 0U;
	chregs->MEND = 0U;
	chregs->ICTRL = 0U;
	chregs->ISTATUS = 0xffU;
	mchp_xec_ecia_girq_src_clr(chcfg->girq, chcfg->girq_pos);
}

static int dma_xec_configure(const struct device *dev, uint32_t channel,
			     struct dma_config *config)
{
	if (!dev || !config) {
		return -EINVAL;
	}

	const struct dma_xec_config * const devcfg = dev->config;
	struct dma_xec_data * const data = dev->data;
	struct dma_regs * const regs = devcfg->regs;
	uint32_t ctrl = 0U;

	if (channel >= (uint32_t)devcfg->dma_channels) {
		return -EINVAL;
	}

	const struct dma_xec_chan_config *chcfg = &devcfg->chcfg[channel];
	struct dma_chan_regs * const chregs = &regs->CHAN[channel];

	xec_dma_chan_clr(chregs, chcfg);

	if (config->block_count > 1) {
		LOG_ERR("Chained transfers not supported");
		return -ENOTSUP;
	}

	if (config->dma_slot >= (uint32_t)devcfg->dma_requests) {
		return -EINVAL;
	}

	if (config->source_data_size != config->dest_data_size) {
		LOG_ERR("HW requires source and dest data size identical");
		return -EINVAL;
	}

	if (!is_dma_data_size_valid(config->source_data_size)) {
		LOG_ERR("HW requires dest data size of 1, 2 or 4 bytes");
		return -EINVAL;
	}

	struct dma_block_config *block = config->head_block;

	/* HW does not support address decrement */
	if ((block->source_addr_adj == DMA_ADDR_ADJ_DECREMENT) ||
	    (block->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT)) {
		LOG_ERR("HW does not support address decrement");
		return -EINVAL;
	}

	/* are source and destination addresses aligned with data size */
	if (!is_data_aligned(block->source_address, block->dest_address,
			     config->source_data_size)) {
		return -EINVAL;
	}

	data->channels[channel].block_count = config->block_count;
	data->channels[channel].block = block;
	data->channels[channel].cb = NULL;
	data->channels[channel].user_data = NULL;

	if (config->complete_callback_en || config->error_callback_en) {
		data->channels[channel].cb = config->dma_callback;
		data->channels[channel].user_data = config->user_data;
	}

	/* data size */
	ctrl = MEC_DMA_CTRL_UNITS(config->source_data_size);

	switch (config->channel_direction) {
	case MEMORY_TO_MEMORY:
		ctrl |= (BIT(MEC_DMA_CTRL_MEM2DEV_POS) |
			 BIT(MEC_DMA_CTRL_DIS_HWFLC_POS));
		if (block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			ctrl |= BIT(MEC_DMA_CTRL_INCR_MEM_POS);
		}
		if (block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			ctrl |= BIT(MEC_DMA_CTRL_INCR_DEV_POS);
		}
		data->channels[channel].mstart = block->source_address;
		data->channels[channel].mend = block->source_address + block->block_size;
		data->channels[channel].dstart = block->dest_address;
		chregs->MSTART = block->source_address;
		chregs->MEND = block->source_address + block->block_size;
		chregs->DSTART = block->dest_address;
		break;
	case MEMORY_TO_PERIPHERAL:
		ctrl |= BIT(MEC_DMA_CTRL_MEM2DEV_POS);
		ctrl |= MEC_DMA_HWFLC_DEV(config->dma_slot);
		if (block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			ctrl |= BIT(MEC_DMA_CTRL_INCR_MEM_POS);
		}
		if (block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			ctrl |= BIT(MEC_DMA_CTRL_INCR_DEV_POS);
		}
		data->channels[channel].mstart = block->source_address;
		data->channels[channel].mend = block->source_address + block->block_size;
		data->channels[channel].dstart = block->dest_address;
		chregs->MSTART = block->source_address;
		chregs->MEND = block->source_address + block->block_size;
		chregs->DSTART = block->dest_address;
		break;
	case PERIPHERAL_TO_MEMORY:
		ctrl |= MEC_DMA_HWFLC_DEV(config->dma_slot);
		if (block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			ctrl |= BIT(MEC_DMA_CTRL_INCR_DEV_POS);
		}
		if (block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			ctrl |= BIT(MEC_DMA_CTRL_INCR_MEM_POS);
		}
		data->channels[channel].mstart = block->dest_address;
		data->channels[channel].mend = block->dest_address + block->block_size;
		data->channels[channel].dstart = block->source_address;
		chregs->MSTART = block->dest_address;
		chregs->MEND = block->dest_address + block->block_size;
		chregs->DSTART = block->source_address;
		break;
	default:
		LOG_ERR("Transfer direction not supported");
		return -EINVAL;
	}

	chregs->CTRL = ctrl;
	chregs->ICTRL = BIT(MEC_DMA_ISC_DONE_POS) |
			BIT(MEC_DMA_ISC_BUS_ERR_POS);
	chregs->ACTV |= BIT(MEC_DMA_ACTV_POS);

	return 0;
}

static int dma_xec_reload(const struct device *dev, uint32_t channel,
			  uint32_t src, uint32_t dst, size_t size)
{
	const struct dma_xec_config * const devcfg = dev->config;
	struct dma_xec_data * const data = dev->data;
	struct dma_regs * const regs = devcfg->regs;

	if (channel >= (uint32_t)devcfg->dma_channels) {
		return -EINVAL;
	}

	struct dma_chan_regs * const chregs = &regs->CHAN[channel];

	if (chregs->CTRL & BIT(MEC_DMA_CTRL_BUSY_POS)) {
		return -EBUSY;
	}

	uint32_t ctrl = chregs->CTRL & ~(BIT(MEC_DMA_CTRL_RUN_POS) |
					 BIT(MEC_DMA_CTRL_SWFLC_GO_POS));

	chregs->CTRL = 0U;
	chregs->ISTATUS = MEC_DMA_ISC_REG_MSK;
	chregs->MSTART = data->channels[channel].mstart;
	chregs->MEND = data->channels[channel].mend;
	chregs->DSTART = data->channels[channel].dstart;
	chregs->CTRL = ctrl;

	return 0;
}

static int dma_xec_start(const struct device *dev, uint32_t channel)
{
	const struct dma_xec_config * const devcfg = dev->config;
	struct dma_regs * const regs = devcfg->regs;
	uint32_t chan_ctrl = 0U;

	if (channel >= (uint32_t)devcfg->dma_channels) {
		return -EINVAL;
	}

	struct dma_chan_regs * const chregs = &regs->CHAN[channel];

	if (chregs->CTRL & BIT(MEC_DMA_CTRL_BUSY_POS)) {
		return -EBUSY;
	}

	chregs->ISTATUS = 0xffu;
	chan_ctrl = chregs->CTRL;

	if (chan_ctrl & BIT(MEC_DMA_CTRL_DIS_HWFLC_POS)) {
		chan_ctrl |= BIT(MEC_DMA_CTRL_SWFLC_GO_POS);
	} else {
		chan_ctrl |= BIT(MEC_DMA_CTRL_RUN_POS);
	}

	chregs->CTRL = chan_ctrl;

	return 0;
}

static int dma_xec_stop(const struct device *dev, uint32_t channel)
{
	const struct dma_xec_config * const devcfg = dev->config;
	struct dma_regs * const regs = devcfg->regs;
	int wait_loops = XEC_ABORT_WAIT_LOOPS;

	if (channel >= (uint32_t)devcfg->dma_channels) {
		return -EINVAL;
	}

	struct dma_chan_regs * const chregs = &regs->CHAN[channel];

	if (chregs->CTRL & BIT(MEC_DMA_CTRL_BUSY_POS)) {
		chregs->CTRL |= BIT(MEC_DMA_CTRL_ABORT_POS);
		/* HW stops on next unit boundary (1, 2, or 4 bytes) */

		while (chregs->CTRL & BIT(MEC_DMA_CTRL_BUSY_POS)) {
			if (!wait_loops) {
				return -ETIMEDOUT;
			}
			wait_loops--;
		}

		chregs->ICTRL = 0;
		chregs->ISTATUS = 0xffu;

		/* TODO channel data/flags in driver data structure */
	}

	return 0;
}

/* TODO - Will we have any transfer context data in struct dma_xec_data
 * to update or use in reporting status?
 * HW supports: MEMORY_TO_MEMORY, MEMORY_TO_PERIPHERAL, or
 * PERIPHERAL_TO_MEMORY
 */
static int dma_xec_get_status(const struct device *dev, uint32_t channel,
			      struct dma_status *status)
{
	const struct dma_xec_config * const devcfg = dev->config;
	struct dma_regs * const regs = devcfg->regs;
	uint32_t chan_ctrl = 0U;

	if ((channel >= (uint32_t)devcfg->dma_channels) || (!status)) {
		return -EINVAL;
	}

	struct dma_chan_regs * const chregs = &regs->CHAN[channel];

	chan_ctrl = chregs->CTRL;

	if (chan_ctrl & MEC_DMA_CTRL_BUSY) {
		status->busy = true;
		/* number of bytes remaining in channel */
		status->pending_length = chregs->MEND - chregs->MSTART;
	} else {
		status->busy = false;
		status->pending_length = 0;
	}

	if (chan_ctrl & BIT(MEC_DMA_CTRL_DIS_HWFLC_POS)) {
		status->dir = MEMORY_TO_MEMORY;
	} else if (chan_ctrl & BIT(MEC_DMA_CTRL_MEM2DEV_POS)) {
		status->dir = MEMORY_TO_PERIPHERAL;
	} else {
		status->dir = PERIPHERAL_TO_MEMORY;
	}

	return 0;
}

/* API - HW does not stupport suspend/resume */
static const struct dma_driver_api dma_xec_api = {
	.config = dma_xec_configure,
	.reload = dma_xec_reload,
	.start = dma_xec_start,
	.stop = dma_xec_stop,
	.get_status = dma_xec_get_status,
};

#ifdef CONFIG_PM_DEVICE
/* TODO - In response to SLP_EN signal channel will not clear
 * its CLK_REQ output until it finishes current transfer.
 * Can we force a pause by clearing channel Activate bit?
 */
static int dmac_xec_pm_action(const struct device *dev,
			      enum pm_device_action action)
{
	int ret = 0;
	const struct dma_xec_config * const devcfg = dev->config;
	struct dma_regs * const regs = devcfg->regs;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		regs->DM_CTRL |= BIT(MEC_DMAM_ACTV_POS);
		break;

	case PM_DEVICE_ACTION_SUSPEND:
		regs->DM_CTRL &= ~BIT(MEC_DMAM_ACTV_POS);
		break;

	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

static void dma_xec_irq_handler(const struct device *dev, uint32_t channel)
{
	const struct dma_xec_config * const devcfg = dev->config;
	const struct dma_xec_chan_config *chcfg = &devcfg->chcfg[channel];
	struct dma_xec_data * const data = dev->data;
	struct dma_regs * const regs = devcfg->regs;
	struct dma_chan_regs *chregs = &regs->CHAN[channel];
	struct dma_xec_channel *chan_data = &data->channels[channel];
	uint32_t sts = chregs->ISTATUS;
	int error_code = 0;

	chregs->ISTATUS = 0xFFu;
	mchp_xec_ecia_girq_src_clr(chcfg->girq, chcfg->girq_pos);

	if (sts & BIT(MEC_DMA_ISC_BUS_ERR_POS)) {
		error_code = (int)sts;
	}

	if (chan_data->cb) {
		chan_data->cb(dev, chan_data->user_data, channel, error_code);
	}
}

#define DECLARE_CHAN_ISR(ch, p2)					\
static void dma_xec_chan_##ch##_isr(const struct device *dev)		\
{									\
	dma_xec_irq_handler(dev, ch);					\
}									\
static void dma_xec_chan_##ch##_isr_connect(const struct device *dev)	\
{									\
	ARG_UNUSED(dev);						\
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, ch, irq),			\
		    DT_INST_IRQ_BY_IDX(0, ch, priority),		\
		    dma_xec_chan_##ch##_isr,				\
		    DEVICE_DT_INST_GET(0), 0);				\
}

UTIL_LISTIFY(XEC_MAX_DMA_CHAN, DECLARE_CHAN_ISR)

#define XEC_DMA_CHAN_CFG2(chan, inst)					\
	{								\
		.irqn = DT_INST_IRQ_BY_IDX(inst, chan, irq),		\
		.irq_pri = DT_INST_IRQ_BY_IDX(inst, chan, priority),	\
		.girq = MCHP_XEC_ECIA_GIRQ(				\
			DT_INST_PROP_BY_IDX(inst, girqs, chan)),	\
		.girq_pos = MCHP_XEC_ECIA_GIRQ_POS(			\
			DT_INST_PROP_BY_IDX(inst, girqs, chan)),	\
	},

static const struct dma_xec_config dma_devcfg0 = {
	.regs = (struct dma_regs * const)(DT_INST_REG_ADDR(0)),
	.dma_channels = DT_INST_PROP(0, dma_channels),
	.dma_requests = DT_INST_PROP(0, dma_requests),
	.pcr_idx = DT_INST_PROP_BY_IDX(0, pcrs, 0),
	.pcr_pos = DT_INST_PROP_BY_IDX(0, pcrs, 1),
	.chcfg = {
		UTIL_LISTIFY(XEC_MAX_DMA_CHAN, XEC_DMA_CHAN_CFG2, 0)
	},
};

#define IRQ_CONFIG3(idx, inst)						\
	dma_xec_chan_##idx##_isr_connect(dev);				\
	irq_enable(DT_INST_IRQ_BY_IDX(inst, idx, irq));			\
	mchp_xec_ecia_enable(dma_devcfg##inst.chcfg[idx].girq,		\
			     dma_devcfg##inst.chcfg[idx].girq_pos);

static void dma_xirq_config_func(const struct device *dev)
{
	ARG_UNUSED(dev);

	UTIL_LISTIFY(XEC_MAX_DMA_CHAN, IRQ_CONFIG3, 0);

	LOG_DBG("install irq done");
}

static int dma_xec_init(const struct device *dev)
{
	const struct dma_xec_config * const devcfg = dev->config;
	struct dma_xec_data * const data = dev->data;
	struct dma_regs * const regs = devcfg->regs;

	LOG_DBG("INIT MCHP XEC DMA");

	memset(data, 0, sizeof(struct dma_xec_data));

	z_mchp_xec_pcr_periph_sleep(devcfg->pcr_idx, devcfg->pcr_pos, 0);

	/* soft reset, self-clearing */
	regs->DM_CTRL = BIT(MEC_DMAM_SRST_POS);
	regs->DM_CTRL = BIT(MEC_DMAM_ACTV_POS);

	/* TODO more stuff such as data initialization */

	dma_xirq_config_func(dev);

	return 0;
}

struct dma_xec_data dma_data0;

#define DMAC_XEC(inst) DT_NODELABEL(DT_DRV_INST(inst))

PM_DEVICE_DT_DEFINE(DMAC_XEC(0), dmac_xec_pm_action);

DEVICE_DT_INST_DEFINE(0, &dma_xec_init, PM_DEVICE_DT_GET(DMAC_XEC(0)),
		      &dma_data0, &dma_devcfg0, PRE_KERNEL_1,
		      CONFIG_DMA_INIT_PRIORITY, &dma_xec_api);
