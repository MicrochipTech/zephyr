/*
 * Copyright (c) 2018 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <soc.h>
#include <drivers/dma.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(dma_mchp_xec, CONFIG_DMA_LOG_LEVEL);

typedef void (*dma_callback)(void *callback_arg, u32_t channel,
			     int error_code);

/* const struct spi_qmspi_config *cfg = dev->config->config_info; */
struct dma_xec_config {
	DMA_Type *regs;
};

struct dma_xec_channel {
	dma_callback cb;
	void *cb_arg;
};

struct dma_xec_data {
//	__aligned(16) DmacDescriptor descriptors[MCHP_NUM_DMA_CHANNELS];
//	__aligned(16) DmacDescriptor descriptors_wb[MCHP_NUM_DMA_CHANNELS];
	struct dma_xec_channel channels[MCHP_NUM_DMA_CHANNELS];
};

#define DEV_DATA(dev) \
	((struct dma_xec_data *const)(dev)->driver_data)


/* Handles DMA interrupts and dispatches to the individual channel */
static void dma_xec_isr(void *arg)
{
	struct device *dev = arg;
	const struct dma_xec_config *cfg = dev->config->config_info;
	DMA_Type *regs = cfg->regs;
	struct dma_xec_data *data = DEV_DATA(dev);
	struct dma_xec_channel *chdata;
	u32_t channel, result;

	result = MCHP_GIRQ_RESULT(MCHP_DMA_GIRQ_NUM);
	while (result) {
		channel = 31u - __CLZ(result);

		u8_t chan_sts = regs->CHAN[channel].ISTS;
		regs->CHAN[channel].IEN = 0u; /* disable channel interrupts */
		regs->CHAN[channel].ISTS = 0xffu;
		MCHP_GIRQ_SRC_CLR(MCHP_DMA_GIRQ_NUM, channel);

		chdata = &data->channels[channel];
		if (chdata->cb) {
			int error_code = 0;
			if (chan_sts & MCHP_DMA_STS_BUS_ERR) {
				error_code = chan_sts;
			}
			chdata->cb(chdata->cb_arg, channel, error_code);
		}

		result &= ~(1ul << channel);
	}
}

/*
 * From include/drivers/dma.h
 * Configure individual channel for DMA transfer.
 *
 * dev     Pointer to the device structure for the driver instance.
 * channel Numeric identification of the channel to configure
 * config  Data structure containing the intended configuration for the
 *         selected channel
 *
 * returns 0 if successful or negative errno code if failure.
 */

/*
 * MEMORY_TO_MEMORY
 * Control: b[22:20]=1,2,or 4, b[19]=1, b[17:16]=11b based on flags, b[15:9]=0, b[8]=1, b[0]=0
 * MStart = source addr
 * MEnd = source addr + nbytes
 * DStart = dest addr
 * MEMORY_TO_PERIPHERAL
 * Control: b[22:20]=1,2,or 4, b[19]=0, b[17:16]=11b based on flags, b[15:9]=dma_slot, b[8]=1, b[0]=1
 * MStart = source addr
 * MEnd = source addr + nbytes
 * DStart = dest addr
 * PERIPHERAL_TO_MEMORY
 * Control: b[22:20]=1,2,or 4, b[19]=0, b[17:16]=11b based on flags, b[15:9]=dma_slot, b[8]=1, b[0]=1
 * MStart = dest addr
 * MEnd = dest addr + nbytes
 * DStart = source addr
 */
static int dma_mem_dev_cfg(DMA_CHAN_Type *regs, u32_t chan,
			   struct dma_config *config,
			   struct dma_block_config *block)
{
	u32_t dctrl, mstart, mend, dstart;

	dctrl = 0u;

	switch (config->channel_direction) {
	case MEMORY_TO_MEMORY:
	case MEMORY_TO_PERIPHERAL:
		if (config->channel_direction == MEMORY_TO_MEMORY) {
			dctrl |= MCHP_DMA_C_DIS_HWFLC;
		}
		if (block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			dctrl |= MCHP_DMA_C_INCR_MEM;
		}
		if (block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			dctrl |= MCHP_DMA_C_INCR_DEV;
		}
		dctrl |= MCHP_DMA_C_MEM2DEV;
		mstart = block->source_address;
		mend = mstart + block->block_size;
		dstart = block->dest_address;
		break;
	case PERIPHERAL_TO_MEMORY:
		if (block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			dctrl |= MCHP_DMA_C_INCR_DEV;
		}
		if (block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			dctrl |= MCHP_DMA_C_INCR_MEM;
		}
		mstart = block->dest_address;
		mend = mstart + block->block_size;
		dstart = block->source_address;
		break;
	default:
		return -EINVAL;
	}

	regs->MSTART = mstart;
	regs->MEND = mend;
	regs->DSTART = dstart;
	regs->CTRL = dctrl;

	return 0;
}

static bool is_data_size_valid(u32_t data_size)
{
	switch (data_size) {
	case 1:
	case 2:
	case 4:
		return true;
	default:
		return false;
	}
}

#if 0
typedef void (*dma_callback)(void *callback_arg, u32_t channel,
			     int error_code);

/* const struct spi_qmspi_config *cfg = dev->config->config_info; */
struct dma_xec_config {
	uintptr_t dma_block_addr;
};

struct dma_xec_channel {
	dma_callback cb;
	void *cb_arg;
};

struct dma_xec_data {
//	__aligned(16) DmacDescriptor descriptors[MCHP_NUM_DMA_CHANNELS];
//	__aligned(16) DmacDescriptor descriptors_wb[MCHP_NUM_DMA_CHANNELS];
	struct dma_xec_channel channels[MCHP_NUM_DMA_CHANNELS];
};
#endif

#if 0
In struct dma_config
 *     source_data_size    [ 0 : 15 ]   - width of source data (in bytes)
 *     dest_data_size      [ 16 : 31 ]  - width of dest data (in bytes)
 *     source_burst_length [ 0 : 15 ]   - number of source data units
 *     dest_burst_length   [ 16 : 31 ]  - number of destination data units

 XEC DMA limits source/dest_data_size to 1, 2, or 4 bytes run time configurable based
 on the alignment and block transfer length in bytes.

 source/dest_burst_len = number of units. This is block transfer length / source/dest_data_size.

struct dma_block_config {
	u32_t source_address;
	u32_t source_gather_interval;
	u32_t dest_address;
	u32_t dest_scatter_interval;
	u16_t dest_scatter_count;
	u16_t source_gather_count;
	u32_t block_size;	/* number of bytes to be transferred for this block */
	struct dma_block_config *next_block;
	u16_t  source_gather_en :  1;
	u16_t  dest_scatter_en :   1;
	u16_t  source_addr_adj :   2;
	u16_t  dest_addr_adj :     2;
	u16_t  source_reload_en :  1;
	u16_t  dest_reload_en :    1;
	u16_t  fifo_mode_control : 4;
	u16_t  flow_control_mode : 1;
	u16_t  reserved :          3;
};


#endif

static int dma_xec_config(struct device *dev, u32_t channel,
			  struct dma_config *config)
{
	struct dma_xec_data *data = DEV_DATA(dev);
	const struct dma_xec_config *cfg = dev->config->config_info;
	DMA_CHAN_Type *regs = (DMA_CHAN_Type *)&cfg->regs->CHAN[channel];
	struct dma_block_config *block = config->head_block;
	struct dma_xec_channel *channel_control;
	u32_t dctrl;
	int key;

	if (channel >= MCHP_NUM_DMA_CHANNELS) {
		LOG_ERR("Unsupported channel");
		return -EINVAL;
	}

	if (config->block_count > 1) {
		LOG_ERR("Chained transfers not supported");
		return -ENOTSUP;
	}

	if (config->dma_slot >= MCHP_DMA_DEVNUM_MAX) {
		LOG_ERR("Invalid DMA slot(HW peripheral)");
		return -EINVAL;
	}

	if (config->channel_priority != 0) {
		LOG_ERR("Invalid priority: MCHP XEC DMAC does not support channel priority");
		return -EINVAL;
	}

	/* Lock and page in the channel configuration */
	key = irq_lock();

	regs->ACTV = 0u;
	regs->CTRL = 0u;
	regs->IEN = 0u;
	regs->ISTS = 0xffu;

	if ((config->source_burst_length != 1) || (config->dest_burst_length != 1)) {
		LOG_ERR("HW only supports burst length of 1");
		goto inval;
	}

	if (config->source_data_size != config->dest_data_size) {
		LOG_ERR("HW requires source and dest data size identical");
		goto inval;
	}

	if (!is_data_size_valid(config->source_data_size)) {
		LOG_ERR("HW requires dest data size of 1, 2 or 4 bytes");
		goto inval;
	}

	/* XEC DMA does not support address decrement */
	if ((block->source_addr_adj == DMA_ADDR_ADJ_DECREMENT) ||
	    (block->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT)) {
		LOG_ERR("HW does not support address decrement");
		goto inval;
	}

	/* Check if source and dest addresses are aligned with data size */
	if (((block->source_address | block->dest_address) & (config->source_data_size - 1))) {
		LOG_ERR("Source and/or dest addresses not aligned with data size");
		goto inval;
	}

	if (dma_mem_dev_cfg(regs, channel, config, block) != 0) {
		goto inval;
	}

	dctrl = ((u32_t)config->source_data_size << MCHP_DMA_C_XFRU_POS);
	dctrl |= (u32_t)config->dma_slot << MCHP_DMA_C_DEV_NUM_POS;

	regs->CTRL |= dctrl;
	regs->IEN = MCHP_DMA_STS_BUS_ERR | MCHP_DMA_STS_DONE;

	/* Set callback and its arguments for this channel */
	channel_control = &data->channels[channel];
	channel_control->cb = config->dma_callback;
	channel_control->cb_arg = config->callback_arg;

	LOG_DBG("Configured channel %d for %08X to %08X (%u)",
		channel,
		block->source_address,
		block->dest_address,
		block->block_size);

	irq_unlock(key);
	return 0;

inval:
	irq_unlock(key);
	return -EINVAL;
}

static int dma_xec_start(struct device *dev, u32_t channel)
{
	const struct dma_xec_config *cfg = dev->config->config_info;
	DMA_CHAN_Type *regs = (DMA_CHAN_Type *)&cfg->regs->CHAN[channel];
	int key = irq_lock();

	regs->ISTS = 0xffu;

	u32_t dctrl = regs->CTRL;

	if (dctrl & MCHP_DMA_C_DIS_HWFLC) { /* software control */
		dctrl |= MCHP_DMA_C_XFER_GO;
	} else { /* hardware control */
		dctrl |= MCHP_DMA_C_RUN;
	}

	regs->CTRL = dctrl;

	irq_unlock(key);

	return 0;
}

static int dma_xec_stop(struct device *dev, u32_t channel)
{
	const struct dma_xec_config *cfg = dev->config->config_info;
	DMA_CHAN_Type *regs = (DMA_CHAN_Type *)&cfg->regs->CHAN[channel];
	int key = irq_lock();

	regs->CTRL |= MCHP_DMA_C_XFER_ABORT;

	u32_t cnt = 16u;
	while (cnt--) {
		if ((regs->CTRL & MCHP_DMA_C_BUSY_STS) == 0) {
			break;
		}
	}

	irq_unlock(key);

	return 0;
}

/*
 * Reload buffer(s) for a DMA channel
 *
 * dev     Pointer to the device structure for the driver instance.
 * channel Numeric identification of the channel to configure
 *         selected channel
 * src     source address for the DMA transfer
 * dst     destination address for the DMA transfer
 * size    size of DMA transfer
 *
 * retval = 0 if successful else negative errno code
 *
 * Assuming caller is not changing alignment and size of transfer
 *
 * XEC DMA needs CTRL Go or Run bits cleared before reprogramming.
 * Determine direction from CTRL register
 * If CTRL.Mem2Dev bit set
 *	MStart = src, MEnd = src + size
 *	Dstart = dst
 * else
 *	MStart = dst, MEnd = dst + size
 *	DStart = src
 *
 */
static int dma_xec_reload(struct device *dev, u32_t channel,
			  u32_t src, u32_t dst, size_t size)
{
	const struct dma_xec_config *cfg = dev->config->config_info;
	DMA_CHAN_Type *regs = (DMA_CHAN_Type *)&cfg->regs->CHAN[channel];
	/* TODO do we need this? struct dma_xec_data *data = DEV_DATA(dev); */
	int key = irq_lock();
	u32_t dctrl;

	dctrl = regs->CTRL;
	regs->CTRL = dctrl & ~(MCHP_DMA_C_RUN | MCHP_DMA_C_XFER_GO);

	/*
	 * TODO - should we check src and dst alignment with size?
	 * dma_config check alignment but next buffer may not be properly aligned.
	 */

	if (dctrl & MCHP_DMA_C_MEM2DEV) {
		regs->MSTART = src;
		regs->MEND = src + size;
		regs->DSTART = dst;
	} else {
		regs->MSTART = dst;
		regs->MEND = dst + size;
		regs->DSTART = src;
	}

	LOG_DBG("Reloaded channel %d for %08X to %08X (%u)",
		channel, src, dst, size);

	irq_unlock(key);
	return 0;
}

static int dma_xec_get_status(struct device *dev, u32_t channel,
			      struct dma_status *stat)
{
	const struct dma_xec_config *cfg = dev->config->config_info;

	if (channel >= MCHP_NUM_DMA_CHANNELS || stat == NULL) {
		return -EINVAL;
	}

	DMA_CHAN_Type *regs = (DMA_CHAN_Type *)&cfg->regs->CHAN[channel];

	u32_t dctrl = regs->CTRL;

	if (dctrl & MCHP_DMA_C_BUSY_STS) {
		stat->busy = true;
	} else {
		stat->busy = false;
	}

	stat->pending_length = regs->MEND - regs->MSTART;

	if (dctrl & MCHP_DMA_C_DIS_HWFLC) {
		stat->dir = MEMORY_TO_MEMORY;
	} else {
		if (dctrl & MCHP_DMA_C_MEM2DEV) {
			stat->dir = MEMORY_TO_PERIPHERAL;
		} else {
			stat->dir = PERIPHERAL_TO_MEMORY;
		}
	}

	return 0;
}

DEVICE_DECLARE(dma_xec_0);

static int dma_xec_init(struct device *dev)
{
	const struct dma_xec_config *cfg = dev->config->config_info;
	DMA_Type *regs = cfg->regs;
	/* TODO struct dma_xec_data *data = DEV_DATA(dev); */

	mchp_pcr_periph_slp_ctrl(PCR_DMA, MCHP_PCR_SLEEP_DIS);

	regs->ACTRST = MCHP_DMAM_CTRL_SOFT_RESET;
	regs->ACTRST = MCHP_DMAM_CTRL_ENABLE;

	MCHP_GIRQ_ENCLR(MCHP_DMA_GIRQ_NUM) = MCHP_DMA_CHAN_BITMAP;
	MCHP_GIRQ_SRC(MCHP_DMA_GIRQ_NUM) = MCHP_DMA_CHAN_BITMAP;

#if (DT_INTMUX_XEC_ECIA_0_LEVEL2_BITMAP & BIT(MCHP_DMA_GIRQ_NUM))
	/* DMA controller interrupts are aggregated (single second level handler) */
	IRQ_CONNECT(DT_DMAC_XEC_DMAC_IRQ_0,
		    DT_DMAC_XEC_DMAC_IRQ_0_PRIORITY,
		    dma_xec_isr, DEVICE_GET(dma_xec_0), 0);
	irq_enable(DT_DMAC_XEC_DMAC_IRQ_0);

	MCHP_GIRQ_ENSET(MCHP_DMA_GIRQ_NUM) = MCHP_DMA_CHAN_BITMAP;
	/* Make sure aggregated GIRQ output is routed to NVIC */
	MCHP_GIRQ_BLK_SETEN(MCHP_DMA_GIRQ_NUM);

#else
	#error "DMA MCHP XEC driver does not support direct interrupts at this time"
#endif

	return 0;
}

static struct dma_xec_data dmac_data;

static const struct dma_driver_api dma_xec_api = {
	.config = dma_xec_config,
	.start = dma_xec_start,
	.stop = dma_xec_stop,
	.reload = dma_xec_reload,
	.get_status = dma_xec_get_status,
};

static const struct dma_xec_config dmac_config = {
	.regs = (DMA_Type *)DT_DMAC_XEC_DMAC_BASE_ADDRESS
};

DEVICE_AND_API_INIT(dma_xec_0, CONFIG_DMA_0_NAME, &dma_xec_init,
		    &dmac_data, &dmac_config, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &dma_xec_api);
