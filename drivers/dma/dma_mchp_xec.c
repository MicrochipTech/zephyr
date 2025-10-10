/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_dmac

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util_macro.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_mchp_xec, CONFIG_DMA_LOG_LEVEL);

#define XEC_DMA_CFG_FLAG_INCR_SRC_ADDR BIT(0)
#define XEC_DMA_CFG_FLAG_INCR_DST_ADDR BIT(1)
#define XEC_DMA_CFG_FLAG_SWFLC         BIT(2)
#define XEC_DMA_CFG_DIR_DEV_TO_MEM     0
#define XEC_DMA_CFG_DIR_MEM_TO_DEV     1u

struct xec_dma_chan_cfg {
	uintptr_t src_addr;
	uintptr_t dst_addr;
	size_t nbytes;
	uint8_t unitsz;
	uint8_t dir;
	uint8_t hwfc_dev;
	uint8_t flags;
};

struct xec_dmac_config {
	mem_addr_t dmac_base;
	void (*irq_config)(void);
	const uint16_t *chan_girqs;
	uint32_t chmsk;
	uint8_t dma_channels;
	uint8_t dma_requests;
	uint8_t enc_pcr_src;
};

struct xec_dmac_channel {
	struct xec_dma_chan_cfg chan_cfg;
	uint32_t isr_hw_status;
	uint32_t block_count;
	uint8_t dir;
	uint8_t flags;
	uint8_t rsvd[2];
	struct dma_block_config *head;
	struct dma_block_config *curr;
	dma_callback_t cb;
	void *user_data;
	uint32_t total_req_xfr_len;
	uint32_t total_curr_xfr_len;
};

#define DMA_XEC_CHAN_FLAGS_CB_EOB_POS     0
#define DMA_XEC_CHAN_FLAGS_CB_ERR_DIS_POS 1

struct xec_dmac_data {
	struct dma_context ctx;
	struct xec_dmac_channel *channels;
	atomic_t *channels_atomic;
#if defined(CONFIG_PM_DEVICE)
	atomic_t *pm_chan_bitmap;
#endif
};

static mem_addr_t xec_dma_chan_base(const struct device *dev, uint32_t chan)
{
	const struct xec_dmac_config *const devcfg = dev->config;
	mem_addr_t chan_base = devcfg->dmac_base;

	chan_base += (XEC_DMA_CHANNELS_OFS_FROM_BASE + (chan * XEC_DMA_CHAN_REGS_SIZE));

	return chan_base;
}

/* We have multiple DMA channels, all channels must become idle before
 * we allow deep sleep entry.
 */
#if defined(CONFIG_PM) || defined(CONFIG_PM_DEVICE)
static void dma_xec_device_busy_set(const struct device *dev, uint8_t chan)
{
	struct xec_dmac_data *const data = dev->data;

	atomic_set_bit(data->pm_chan_bitmap, chan);
	pm_device_busy_set(dev);
}

/* Clear the channel's atomic pm bit.
 * If all atomic bits are clear then clear the PM busy flag the kernel looks at.
 */
static void dma_xec_device_busy_clear(const struct device *dev, uint8_t chan)
{
	const struct xec_dmac_config *const devcfg = dev->config;
	struct xec_dmac_data *const data = dev->data;
	uint8_t chan_busy = 0;

	atomic_clear_bit(data->pm_chan_bitmap, chan);

	for (int i = 0; i < (int)devcfg->dma_channels; i++) {
		if (atomic_test_bit(data->pm_chan_bitmap, i)) {
			chan_busy++;
		}
	}

	if (chan_busy == 0) {
		pm_device_busy_clear(dev);
	}
}
#else
static void dma_xec_device_busy_set(const struct device *dev, uint8_t chan)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan);
}

static void dma_xec_device_busy_clear(const struct device *dev, uint8_t chan)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan);
}
#endif /* #if defined(CONFIG_PM) || defined(CONFIG_PM_DEVICE) */

static bool is_chan_valid(const struct device *dev, uint32_t chan)
{
	const struct xec_dmac_config *const devcfg = dev->config;

	if ((devcfg->chmsk & BIT(chan)) != 0) {
		return true;
	}

	return false;
}

static int is_dma_data_size_valid(uint32_t datasz)
{
	if ((datasz == 1U) || (datasz == 2U) || (datasz == 4U)) {
		return 1;
	}

	return 0;
}

/* HW requires if unit size is 2 or 4 bytes the source/destination addresses
 * to be aligned >= 2 or 4 bytes.
 */
static int is_data_aligned(uint32_t src, uint32_t dest, uint32_t unitsz)
{
	if (unitsz == 1) {
		return 1;
	}

	if ((src | dest) & (unitsz - 1U)) {
		return 0;
	}

	return 1;
}

static int is_dma_config_valid(const struct device *dev, struct dma_config *config)
{
	const struct xec_dmac_config *const devcfg = dev->config;

	if (config->dma_slot >= (uint32_t)devcfg->dma_requests) {
		LOG_ERR("XEC DMA config dma slot > exceeds number of request lines");
		return 0;
	}

	if (config->source_data_size != config->dest_data_size) {
		LOG_ERR("XEC DMA requires source and dest data size identical");
		return 0;
	}

	if (((config->channel_direction == MEMORY_TO_MEMORY) ||
	     (config->channel_direction == MEMORY_TO_PERIPHERAL) ||
	     (config->channel_direction == PERIPHERAL_TO_MEMORY)) == false) {
		LOG_ERR("XEC DMA only support M2M, M2P, P2M");
		return 0;
	}

	if (is_dma_data_size_valid(config->source_data_size) == false) {
		LOG_ERR("XEC DMA requires xfr unit size of 1, 2 or 4 bytes");
		return 0;
	}

	return 1;
}

static int check_blocks(struct xec_dmac_channel *chdata, struct dma_block_config *block,
			uint32_t block_count, uint32_t unit_size)
{
	struct dma_block_config *pb = block;
	uint32_t n = 0, total_len = 0;

	if ((chdata == NULL) || (block == NULL)) {
		LOG_ERR("bad pointer");
		return -EINVAL;
	}

	chdata->total_req_xfr_len = 0;

	for (n = 0; n < block_count; n++) {
		if (pb == NULL) {
			LOG_ERR("Block %u config is NULL", n);
			return -EINVAL;
		}

		if ((pb->source_addr_adj == DMA_ADDR_ADJ_DECREMENT) ||
		    (pb->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT)) {
			LOG_ERR("Block %u: HW does not support address decrement", n);
			return -EINVAL;
		}

		if (is_data_aligned(pb->source_address, pb->dest_address, unit_size) == false) {
			LOG_ERR("Block %u: violates source/dest unit size", n);
			return -EINVAL;
		}

		total_len += block->block_size;
		pb = pb->next_block;
	}

	chdata->total_req_xfr_len = total_len;

	if (total_len == 0) {
		LOG_ERR("%u blocks no data", block_count);
		return -EINVAL;
	}

	return 0;
}

/* caller's responsibility to check chan is valid */
static void xec_dma_chan_init(const struct device *dev, uint32_t chan)
{
	const struct xec_dmac_config *const devcfg = dev->config;
	mem_addr_t chan_base = devcfg->dmac_base;
	uint8_t girq = MCHP_XEC_ECIA_GIRQ(devcfg->chan_girqs[chan]);
	uint8_t girq_pos = MCHP_XEC_ECIA_GIRQ_POS(devcfg->chan_girqs[chan]);

	chan_base = xec_dma_chan_base(dev, chan);

	/* write end address to 0 first to stop HW if it was running */
	sys_write32(0, chan_base + XEC_DMA_CHAN_MEA_OFS);
	sys_write32(0, chan_base + XEC_DMA_CHAN_MSA_OFS);

	sys_write32(0, chan_base + XEC_DMA_CHAN_ACTV_OFS);
	sys_write32(0, chan_base + XEC_DMA_CHAN_CR_OFS);
	sys_write32(0, chan_base + XEC_DMA_CHAN_DEVA_OFS);
	sys_write32(0, chan_base + XEC_DMA_CHAN_IER_OFS);
	sys_write32(XEC_DMA_CHAN_IESR_MSK, chan_base + XEC_DMA_CHAN_SR_OFS);
	soc_ecia_girq_status_clear(girq, girq_pos);
}

static bool xec_dma_chan_is_busy(mem_addr_t chan_base)
{
	if (sys_test_bit(chan_base + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_BUSY_POS) != 0) {
		return true;
	}

	return false;
}

static void xec_dma_chan_reload(mem_addr_t chan_base, uintptr_t src, uintptr_t dest, size_t nbytes)
{
	uint32_t ctrl = 0, msa = 0, mea = 0, deva = 0;
	uint32_t chien = BIT(XEC_DMA_CHAN_IESR_DONE_POS) | BIT(XEC_DMA_CHAN_IESR_BERR_POS);

	sys_write32(0, chan_base + XEC_DMA_CHAN_ACTV_OFS);
	ctrl = sys_read32(chan_base + XEC_DMA_CHAN_CR_OFS);
	sys_write32(0, chan_base + XEC_DMA_CHAN_CR_OFS);
	sys_write32(0, chan_base + XEC_DMA_CHAN_MEA_OFS);
	sys_write32(0, chan_base + XEC_DMA_CHAN_MSA_OFS);
	sys_write32(0xff, chan_base + XEC_DMA_CHAN_SR_OFS);

	if ((ctrl & BIT(XEC_DMA_CHAN_CR_M2D_POS)) != 0) {
		msa = src;
		mea = src + nbytes;
		deva = dest;
	} else { /* device to memory */
		msa = dest;
		mea = dest + nbytes;
		deva = src;
	}

	if ((ctrl & BIT(XEC_DMA_CHAN_CR_DIS_HFC_POS)) == 0) {
		ctrl |= BIT(XEC_DMA_CHAN_CR_HFC_RUN_POS);
	} else {
		ctrl |= BIT(XEC_DMA_CHAN_CR_SFC_GO_POS);
	}

	sys_write32(msa, chan_base + XEC_DMA_CHAN_MSA_OFS);
	sys_write32(mea, chan_base + XEC_DMA_CHAN_MEA_OFS);
	sys_write32(deva, chan_base + XEC_DMA_CHAN_DEVA_OFS);
	sys_write32(chien, chan_base + XEC_DMA_CHAN_IER_OFS);
	sys_write32(ctrl, chan_base + XEC_DMA_CHAN_CR_OFS);
	sys_write32(1u, chan_base + XEC_DMA_CHAN_ACTV_OFS);
}

/*
 * struct dma_config flags
 * dma_slot - peripheral source/target ID. Not used for Mem2Mem
 * channel_direction - HW supports Mem2Mem, Mem2Periph, and Periph2Mem
 * complete_callback_en - if true invoke callback on completion (no error)
 * error_callback_en - if true invoke callback on error
 * source_handshake - 0=HW, 1=SW
 * dest_handshake - 0=HW, 1=SW
 * channel_priority - 4-bit field. HW implements round-robin only.
 * source_chaining_en - Chaining channel together
 * dest_chaining_en - HW does not support channel chaining.
 * linked_channel - HW does not support
 * cyclic - HW does not support cyclic buffer. Would have to emulate with SW.
 * source_data_size - unit size of source data. HW supports 1, 2, or 4 bytes
 * dest_data_size - unit size of dest data. HW requires same as source_data_size
 * source_burst_length - HW does not support
 * dest_burst_length - HW does not support
 * block_count -
 * user_data -
 * dma_callback -
 * head_block - pointer to struct dma_block_config
 *
 * struct dma_block_config
 * source_address -
 * source_gather_interval - N/A
 * dest_address -
 * dest_scatter_interval - N/A
 * dest_scatter_count - N/A
 * source_gather_count - N/A
 * block_size
 * config - flags
 *	source_gather_en - N/A
 *	dest_scatter_en - N/A
 *	source_addr_adj - 0(increment), 1(decrement), 2(no change)
 *	dest_addr_adj - 0(increment), 1(decrement), 2(no change)
 *	source_reload_en - reload source address at end of block
 *	dest_reload_en - reload destination address at end of block
 *	fifo_mode_control - N/A
 *	flow_control_mode - 0(source req service on data available) HW does this
 *			    1(source req postposed until dest req happens) N/A
 *
 *
 * DMA channel implements memory start address, memory end address,
 * and peripheral address registers. No peripheral end address.
 * Transfer ends when memory start address increments and reaches
 * memory end address.
 *
 * Memory to Memory: copy from source_address to dest_address
 *	chan direction = Mem2Dev. chan.control b[8]=1
 *	chan mem_addr = source_address
 *	chan mem_addr_end = source_address + block_size
 *	chan dev_addr = dest_address
 *
 * Memory to Peripheral: copy from source_address(memory) to dest_address(peripheral)
 *	chan direction = Mem2Dev. chan.control b[8]=1
 *	chan mem_addr = source_address
 *	chan mem_addr_end = chan mem_addr + block_size
 *	chan dev_addr = dest_address
 *
 * Peripheral to Memory:
 *	chan direction = Dev2Mem. chan.contronl b[8]=1
 *	chan mem_addr = dest_address
 *	chan mem_addr_end = chan mem_addr + block_size
 *	chan dev_addr = source_address
 */
static int xec_dmac_configure(const struct device *dev, uint32_t channel, struct dma_config *config)
{
	struct xec_dmac_data *const data = dev->data;
	struct xec_dmac_channel *chdata = NULL;
	struct xec_dma_chan_cfg *chcfg = NULL;
	struct dma_block_config *block = NULL;
	int ret = 0;

	if ((config == NULL) || (is_chan_valid(dev, channel) == false)) {
		return -EINVAL;
	}

	if (is_dma_config_valid(dev, config) == false) {
		return -EINVAL;
	}

	xec_dma_chan_init(dev, channel);

	chdata = &data->channels[channel];
	chcfg = &chdata->chan_cfg;

	chdata->total_req_xfr_len = 0;
	chdata->total_curr_xfr_len = 0;

	block = config->head_block;

	ret = check_blocks(chdata, block, config->block_count, config->source_data_size);
	if (ret) {
		return ret;
	}

	chcfg->unitsz = 1u;
	if ((config->source_data_size == 4) || (config->source_data_size == 2)) {
		chcfg->unitsz = config->source_data_size;
	}

	chcfg->flags = 0;
	chcfg->nbytes = block->block_size;
	chcfg->hwfc_dev = config->dma_slot & XEC_DMA_CHAN_CR_HFC_DEV_MSK0;
	chcfg->dir = XEC_DMA_CFG_DIR_MEM_TO_DEV;

	if (config->channel_direction == PERIPHERAL_TO_MEMORY) {
		chcfg->dir = XEC_DMA_CFG_DIR_DEV_TO_MEM;
	} else if (config->channel_direction == MEMORY_TO_MEMORY) {
		/* mem-to-mem requires software flow control */
		chcfg->flags |= XEC_DMA_CFG_FLAG_SWFLC;
	}

	chcfg->src_addr = block->source_address;
	chcfg->dst_addr = block->dest_address;
	if (block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
		chcfg->flags |= XEC_DMA_CFG_FLAG_INCR_SRC_ADDR;
	}

	if (block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
		chcfg->flags |= XEC_DMA_CFG_FLAG_INCR_DST_ADDR;
	}

	chdata->head = block;
	chdata->curr = block;
	chdata->block_count = config->block_count;
	chdata->dir = config->channel_direction;
	chdata->flags = 0;
	chdata->cb = config->dma_callback;
	chdata->user_data = config->user_data;

	/* invoke callback on completion of each block instead of all blocks? */
	if (config->complete_callback_en) {
		chdata->flags |= BIT(DMA_XEC_CHAN_FLAGS_CB_EOB_POS);
	}

	if (config->error_callback_dis) { /* disable callback on errors ? */
		chdata->flags |= BIT(DMA_XEC_CHAN_FLAGS_CB_ERR_DIS_POS);
	}

	return 0;
}

static void xec_dmac_reload_int(const struct device *dev, uint32_t channel, uint32_t src,
				uint32_t dst, size_t size)
{
	struct xec_dmac_data *const data = dev->data;
	struct xec_dmac_channel *chdata = NULL;
	struct xec_dma_chan_cfg *chcfg = NULL;
	mem_addr_t chan_base = 0;

	chan_base = xec_dma_chan_base(dev, channel);

	chdata = &data->channels[channel];
	chcfg = &chdata->chan_cfg;

	chcfg->src_addr = src;
	chcfg->dst_addr = dst;
	chcfg->nbytes = size;

	chdata->total_req_xfr_len = size;
	chdata->total_curr_xfr_len = 0;

	xec_dma_chan_reload(chan_base, (uintptr_t)src, (uintptr_t)dst, size);
}

/* Update previously configured DMA channel with new data source address,
 * data destination address, and size in bytes.
 * src = source address for DMA transfer
 * dst = destination address for DMA transfer
 * size = size of DMA transfer. Assume this is in bytes.
 * We assume the caller will pass src, dst, and size that matches
 * the unit size from the previous configure call.
 */
static int xec_dmac_reload(const struct device *dev, uint32_t channel, uint32_t src, uint32_t dst,
			   size_t size)
{
	mem_addr_t chan_base = 0;

	if (is_chan_valid(dev, channel) == false) {
		return -EINVAL;
	}

	chan_base = xec_dma_chan_base(dev, channel);

	if (xec_dma_chan_is_busy(chan_base) == true) {
		return -EBUSY;
	}

	xec_dmac_reload_int(dev, channel, src, dst, size);

	return 0;
}

static void xec_dma_chan_configure(const struct device *dev, struct xec_dma_chan_cfg *cfg,
				   uint32_t chan)
{
	mem_addr_t chan_base = 0;
	uint32_t ctrl = 0u; /* dir = Dev2Mem, IncrMem=0, IncrDev=0 */
	uint32_t usz = 1u;

	if ((cfg->unitsz == 4u) || (cfg->unitsz == 2u)) {
		usz = cfg->unitsz;
	}

	chan_base = xec_dma_chan_base(dev, chan);

	/* ensure channel is inactive and internal FSM is cleared */
	sys_write32(0, chan_base + XEC_DMA_CHAN_CR_OFS);
	sys_write32(0, chan_base + XEC_DMA_CHAN_IER_OFS);
	sys_write32(0, chan_base + XEC_DMA_CHAN_MEA_OFS);
	sys_write32(0, chan_base + XEC_DMA_CHAN_MSA_OFS);
	sys_write32(0xffu, chan_base + XEC_DMA_CHAN_SR_OFS);

	ctrl = XEC_DMA_CHAN_CR_XU_SET(usz);
	if (cfg->dir == XEC_DMA_CFG_DIR_MEM_TO_DEV) {
		ctrl |= BIT(XEC_DMA_CHAN_CR_M2D_POS);

		sys_write32(cfg->src_addr, chan_base + XEC_DMA_CHAN_MSA_OFS);
		sys_write32(cfg->src_addr + cfg->nbytes, chan_base + XEC_DMA_CHAN_MEA_OFS);
		sys_write32(cfg->dst_addr, chan_base + XEC_DMA_CHAN_DEVA_OFS);

		if ((cfg->flags & XEC_DMA_CFG_FLAG_INCR_SRC_ADDR) != 0) {
			ctrl |= BIT(XEC_DMA_CHAN_CR_INC_MEM_POS);
		}

		if ((cfg->flags & XEC_DMA_CFG_FLAG_INCR_DST_ADDR) != 0) {
			ctrl |= BIT(XEC_DMA_CHAN_CR_INC_DEV_POS);
		}
	} else { /* device(source address) to memory(destination address) */
		sys_write32(cfg->dst_addr, chan_base + XEC_DMA_CHAN_MSA_OFS);
		sys_write32(cfg->dst_addr + cfg->nbytes, chan_base + XEC_DMA_CHAN_MEA_OFS);
		sys_write32(cfg->src_addr, chan_base + XEC_DMA_CHAN_DEVA_OFS);

		if ((cfg->flags & XEC_DMA_CFG_FLAG_INCR_SRC_ADDR) != 0) {
			ctrl |= BIT(XEC_DMA_CHAN_CR_INC_DEV_POS);
		}

		if ((cfg->flags & XEC_DMA_CFG_FLAG_INCR_DST_ADDR) != 0) {
			ctrl |= BIT(XEC_DMA_CHAN_CR_INC_MEM_POS);
		}
	}

	if ((cfg->flags & XEC_DMA_CFG_FLAG_SWFLC) != 0) {
		ctrl |= BIT(XEC_DMA_CHAN_CR_DIS_HFC_POS);
	}

	ctrl |= XEC_DMA_CHAN_CR_HFC_DEV_SET((uint32_t)cfg->hwfc_dev);

	sys_write32(ctrl, chan_base + XEC_DMA_CHAN_CR_OFS);
	sys_set_bit(chan_base + XEC_DMA_CHAN_ACTV_OFS, XEC_DMA_CHAN_ACTV_EN_POS);
}

static void xec_dma_chan_go(mem_addr_t chan_base)
{
	uint32_t ctrl = sys_read32(chan_base + XEC_DMA_CHAN_CR_OFS);

	if ((ctrl & BIT(XEC_DMA_CHAN_CR_DIS_HFC_POS)) == 0) {
		ctrl &= ~BIT(XEC_DMA_CHAN_CR_SFC_GO_POS);
		ctrl |= BIT(XEC_DMA_CHAN_CR_HFC_RUN_POS);
	} else {
		ctrl &= ~BIT(XEC_DMA_CHAN_CR_HFC_RUN_POS);
		ctrl |= BIT(XEC_DMA_CHAN_CR_SFC_GO_POS);
	}

	sys_write32(ctrl, chan_base + XEC_DMA_CHAN_CR_OFS);
}

static int dma_xec_start(const struct device *dev, uint32_t channel)
{
	struct xec_dmac_data *const data = dev->data;
	struct xec_dma_chan_cfg *chcfg = NULL;
	struct xec_dmac_channel *chdata = NULL;
	mem_addr_t chan_base = 0;

	if (is_chan_valid(dev, channel) == false) {
		return -EINVAL;
	}

	chan_base = xec_dma_chan_base(dev, channel);
	chdata = &data->channels[channel];
	chcfg = &chdata->chan_cfg;

	sys_write32(0, chan_base + XEC_DMA_CHAN_IER_OFS);

	xec_dma_chan_configure(dev, chcfg, channel);

	sys_write32(XEC_DMA_CHAN_IESR_MSK, chan_base + XEC_DMA_CHAN_SR_OFS);
	sys_write32(BIT(XEC_DMA_CHAN_IESR_DONE_POS) | BIT(XEC_DMA_CHAN_IESR_BERR_POS),
		    chan_base + XEC_DMA_CHAN_IER_OFS);

	/* Block PM transition until DMA completes */
	dma_xec_device_busy_set(dev, channel);

	xec_dma_chan_go(chan_base); /* start HW */

	return 0;
}

static int xec_dma_chan_stop(mem_addr_t chan_base)
{
	uint32_t wait_cnt = 1024u;
	int ret = 0;

	if (sys_test_bit(chan_base + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_BUSY_POS) != 0) {
		sys_set_bit(chan_base + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_ABORT_POS);
		/* stop on next byte boundary */
		while (sys_test_bit(chan_base + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_BUSY_POS) !=
		       0) {
			if (wait_cnt == 0) {
				ret = -ETIMEDOUT;
				break;
			}
			wait_cnt--;
		}
	}

	/* Do we need to clear Run and SWGo bits? */
	sys_clear_bit(chan_base + XEC_DMA_CHAN_ACTV_OFS, XEC_DMA_CHAN_ACTV_EN_POS);

	return ret;
}

static int dma_xec_stop(const struct device *dev, uint32_t channel)
{
	mem_addr_t chan_base = 0;
	int ret = 0;

	if (is_chan_valid(dev, channel) == false) {
		return -EINVAL;
	}

	chan_base = xec_dma_chan_base(dev, channel);

	ret = xec_dma_chan_stop(chan_base);

	dma_xec_device_busy_clear(dev, channel);

	return ret;
}

/* Get DMA transfer status.
 * HW supports: MEMORY_TO_MEMORY, MEMORY_TO_PERIPHERAL, or
 * PERIPHERAL_TO_MEMORY
 * current DMA runtime status structure
 *
 * busy				- is current DMA transfer busy or idle
 * dir				- DMA transfer direction
 * pending_length		- data length pending to be transferred in bytes
 *					or platform dependent.
 * We don't implement a circular buffer
 * free				- free buffer space
 * write_position		- write position in a circular dma buffer
 * read_position		- read position in a circular dma buffer
 *
 */
static int dma_xec_get_status(const struct device *dev, uint32_t chan, struct dma_status *status)
{
	struct xec_dmac_data *const data = dev->data;
	struct xec_dmac_channel *chan_data = NULL;
	struct xec_dma_chan_cfg *chcfg = NULL;
	mem_addr_t chan_base = 0;
	uint32_t ctrl = 0, msa = 0, mea = 0, rembytes = 0;

	if (!is_chan_valid(dev, chan) || (status == NULL)) {
		LOG_ERR("unsupported channel");
		return -EINVAL;
	}

	chan_data = &data->channels[chan];
	chcfg = &chan_data->chan_cfg;

	chan_base = xec_dma_chan_base(dev, chan);

	msa = sys_read32(chan_base + XEC_DMA_CHAN_MSA_OFS);
	mea = sys_read32(chan_base + XEC_DMA_CHAN_MEA_OFS);
	ctrl = sys_read32(chan_base + XEC_DMA_CHAN_CR_OFS);

	if ((ctrl & BIT(XEC_DMA_CHAN_CR_BUSY_POS)) != 0) {
		if (mea > msa) {
			rembytes = mea - msa;
		}
		status->busy = true;
		status->pending_length = chan_data->total_req_xfr_len - rembytes;
	} else {
		status->busy = false;
		status->pending_length =
			(chan_data->total_req_xfr_len - chan_data->total_curr_xfr_len);
	}

	if ((chcfg->flags & XEC_DMA_CFG_FLAG_SWFLC) != 0) {
		status->dir = MEMORY_TO_MEMORY;
	} else if (chcfg->dir == XEC_DMA_CFG_DIR_MEM_TO_DEV) {
		status->dir = MEMORY_TO_PERIPHERAL;
	} else {
		status->dir = PERIPHERAL_TO_MEMORY;
	}

	status->total_copied = chan_data->total_curr_xfr_len;

	return 0;
}

static int dma_xec_get_attribute(const struct device *dev, uint32_t type, uint32_t *value)
{
	if ((type == DMA_ATTR_MAX_BLOCK_COUNT) && (value != NULL)) {
		*value = 1;
		return 0;
	}

	return -EINVAL;
}

/* If chan is not in [0, number of channels supported] return false
 * If filter_param is NULL return true, caller can use the channel.
 * If filter_param is not NULL interpret the value it points to as
 * the requested channel number. If requested channel number matches
 * the chan parameter passed return true else return false.
 */
static bool dma_xec_chan_filter(const struct device *dev, int chan, void *filter_param)
{
	const struct xec_dmac_config *const devcfg = dev->config;
	int requested_chan = 0;

	if ((chan < 0) || (chan >= devcfg->dma_channels)) {
		return false;
	}

	if (filter_param == NULL) { /* nothing to check */
		return true;
	}

	requested_chan = *(int *)filter_param;
	if (requested_chan == chan) {
		return true;
	}

	return false;
}

static void do_callback(const struct device *dev, uint32_t chan, struct xec_dmac_channel *chan_data,
			int status)
{
	if (chan_data->cb) {
		chan_data->cb(dev, chan_data->user_data, chan, status);
	}
}

static void xec_dmac_irq_handler(const struct device *dev, uint8_t chan)
{
	const struct xec_dmac_config *const devcfg = dev->config;
	struct xec_dmac_data *const data = dev->data;
	struct xec_dmac_channel *chan_data = &data->channels[chan];
	struct dma_block_config *block = chan_data->curr;
	uint8_t girq = MCHP_XEC_ECIA_GIRQ(devcfg->chan_girqs[chan]);
	uint8_t girq_pos = MCHP_XEC_ECIA_GIRQ_POS(devcfg->chan_girqs[chan]);
	uint32_t ctrl = 0, istatus = 0, msa = 0, mea = 0, num_rem_bytes = 0;
	mem_addr_t chb = 0;

	chb = xec_dma_chan_base(dev, chan);

	msa = sys_read32(chb + XEC_DMA_CHAN_MSA_OFS);
	mea = sys_read32(chb + XEC_DMA_CHAN_MEA_OFS);
	ctrl = sys_read32(chb + XEC_DMA_CHAN_CR_OFS);
	istatus = sys_read32(chb + XEC_DMA_CHAN_SR_OFS);

	num_rem_bytes = (mea >= msa) ? (mea - msa) : 0;
	chan_data->total_curr_xfr_len += (chan_data->chan_cfg.nbytes - num_rem_bytes);
	chan_data->isr_hw_status = istatus;

	sys_write32(0, chb + XEC_DMA_CHAN_IER_OFS);
	/* clear run bits, activate bit, and status */
	sys_clear_bits(chb + XEC_DMA_CHAN_CR_OFS, (BIT(XEC_DMA_CHAN_CR_HFC_RUN_POS) |
						   BIT(XEC_DMA_CHAN_CR_SFC_GO_POS)));
	sys_clear_bit(chb + XEC_DMA_CHAN_ACTV_OFS, XEC_DMA_CHAN_ACTV_EN_POS);
	sys_write32(XEC_DMA_CHAN_IESR_MSK, chb + XEC_DMA_CHAN_SR_OFS);
	soc_ecia_girq_status_clear(girq, girq_pos);

	if ((istatus & BIT(XEC_DMA_CHAN_IESR_BERR_POS)) != 0) {
		dma_xec_device_busy_clear(dev, chan);
		if ((chan_data->flags & BIT(DMA_XEC_CHAN_FLAGS_CB_ERR_DIS_POS)) == 0) {
			do_callback(dev, chan, chan_data, -EIO);
		}
		return;
	}

	chan_data->block_count--;
	if (chan_data->block_count != 0) {
		if (((chan_data->flags & BIT(DMA_XEC_CHAN_FLAGS_CB_EOB_POS)) != 0) &&
		    (chan_data->cb != NULL)) {
			do_callback(dev, chan, chan_data, DMA_STATUS_BLOCK);
		}

		block = block->next_block;
		if (block != NULL) {
			chan_data->curr = block;
			xec_dmac_reload_int(dev, chan, block->source_address, block->dest_address,
					    block->block_size);
			dma_xec_start(dev, chan);
		}
	} else {
		dma_xec_device_busy_clear(dev, chan);
		do_callback(dev, chan, chan_data, DMA_STATUS_COMPLETE);
	}
}

#if defined(CONFIG_PM_DEVICE)
/* When PM policy allows suspend or resume this function will be called
 * by the kernel PM subsystem. On suspend we clear the DMA block activate
 * bit which clock gates the block and should cause its CLK_REQ signal to
 * go inactive. On resume we set the DMA block activate ungating clocks in
 * the block. For light sleep (non-suspend) the DMA block can continue
 * operation and its interrupts will wake the CPU.
 */
static int xec_dmac_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct xec_dmac_config *const devcfg = dev->config;

	LOG_DBG("PM action: %d", (int)action);

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		sys_clear_bit(devcfg->dmac_base + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_EN_POS);
		break;

	case PM_DEVICE_ACTION_RESUME:
		sys_set_bit(devcfg->dmac_base + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_EN_POS);
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static int xec_dmac_init(const struct device *dev)
{
	struct xec_dmac_data *const data = dev->data;
	const struct xec_dmac_config *const devcfg = dev->config;
	mem_addr_t dmac_base = devcfg->dmac_base;

	LOG_DBG("driver init");

	data->ctx.magic = DMA_MAGIC;
	data->ctx.dma_channels = devcfg->dma_channels;
	data->ctx.atomic = data->channels_atomic;

	soc_xec_pcr_sleep_en_clear(devcfg->enc_pcr_src);

	sys_set_bit(dmac_base + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_SRST_POS);
	sys_write32(0, dmac_base + XEC_DMA_MAIN_DPKT_OFS); /* delay */
	sys_set_bit(dmac_base + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_EN_POS);

	if (devcfg->irq_config != NULL) {
		devcfg->irq_config();
	}

	return 0;
}

/* API - HW does not stupport suspend/resume */
static DEVICE_API(dma, xec_dmac_api) = {
	.config = xec_dmac_configure,
	.reload = xec_dmac_reload,
	.start = dma_xec_start,
	.stop = dma_xec_stop,
	.get_status = dma_xec_get_status,
	.chan_filter = dma_xec_chan_filter,
	.get_attribute = dma_xec_get_attribute,
};

/* defaults from soc.h */
#define XEC_DT_DMAC_CHAN_MAX(inst)     DT_INST_PROP_OR(inst, dma_channels, XEC_DMAC_MAX_CHAN)
#define XEC_DT_DMAC_CHAN_REQ_MAX(inst) DT_INST_PROP_OR(inst, dma_requests, XEC_DMAC_MAX_CHAN)

#define XEC_DT_DMAC_CHAN_GIRQ(inst, idx) MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, idx))

#define XEC_DT_DMAC_CHAN_GIRQ_POS(inst, idx)                                                       \
	MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, idx))

#define XEC_DMAC_GIRQ_VAL(nid, prop, idx) (uint16_t)DT_PROP_BY_IDX(nid, prop, idx)

#define XEC_DMAC_GIRQ_TBL(i)                                                                       \
	static const uint16_t xec_dmac##i##girq_tbl[] = {                                          \
		DT_INST_FOREACH_PROP_ELEM_SEP(i, girqs, XEC_DMAC_GIRQ_VAL, (, ))}

#define XEC_DMAC_CHAN_ISR_DEF(idx, inst)                                                           \
	static void xec_dmac##inst##_chan##idx##_isr(const struct device *dev)                     \
	{                                                                                          \
		xec_dmac_irq_handler(dev, idx);                                                    \
	}

#define XEC_DMAC_CHAN_DEFINE_ISRS(i) LISTIFY(XEC_DT_DMAC_CHAN_MAX(i), XEC_DMAC_CHAN_ISR_DEF, (), i);

#define XEC_DMAC_CHAN_CFG_IRQ(idx, inst)                                                           \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, idx, irq), DT_INST_IRQ_BY_IDX(inst, idx, priority),   \
		    xec_dmac##inst##_chan##idx##_isr, DEVICE_DT_INST_GET(inst), 0);                \
	irq_enable(DT_INST_IRQ_BY_IDX(inst, idx, irq));                                            \
	soc_ecia_girq_ctrl(XEC_DT_DMAC_CHAN_GIRQ(inst, idx), XEC_DT_DMAC_CHAN_GIRQ_POS(inst, idx), \
			   1u);

#define XEC_DMAC_CONFIG_IRQ_DEFINE(i)                                                              \
	static void xec_dmac##i##_config_irq(void)                                                 \
	{                                                                                          \
		LISTIFY(XEC_DT_DMAC_CHAN_MAX(i), XEC_DMAC_CHAN_CFG_IRQ, (), i);                       \
	}

/* Used by DMA request channel API */
#define XEC_DMAC_CHAN_ATOMIC_DATA(i)                                                               \
	static ATOMIC_DEFINE(xec_dmac##i##_chan_atomic, XEC_DT_DMAC_CHAN_MAX(i))

/* Driver per channel transfer data */
#define XEC_DMAC_CHAN_DATA(i)                                                                      \
	static struct xec_dmac_channel xec_dmac##i##_chans[XEC_DT_DMAC_CHAN_MAX(i)];

#ifdef CONFIG_PM_DEVICE
#define XEC_DMAC_DATA_DEFINE(i)                                                                    \
	static ATOMIC_DEFINE(xec_dmac##i##_pm_chan_bitmap, XEC_DT_DMAC_CHAN_MAX(i));               \
	static struct xec_dmac_data xec_dmac##i##_data = {                                         \
		.ctx.magic = DMA_MAGIC,                                                            \
		.ctx.dma_channels = XEC_DT_DMAC_CHAN_MAX(i),                                       \
		.ctx.atomic = xec_dmac##i##_chan_atomic,                                           \
		.channels = xec_dmac##i##_chans,                                                   \
		.channels_atomic = xec_dmac##i##_chan_atomic,                                      \
		.pm_chan_bitmap = xec_dmac##i##_pm_chan_bitmap,                                    \
	}
#else
#define XEC_DMAC_DATA_DEFINE(i)                                                                    \
	static struct xec_dmac_data xec_dmac##i##_data = {                                         \
		.ctx.magic = DMA_MAGIC,                                                            \
		.ctx.dma_channels = XEC_DT_DMAC_CHAN_MAX(i),                                       \
		.ctx.atomic = xec_dmac##i##_chan_atomic,                                           \
		.channels = xec_dmac##i##_chans,                                                   \
		.channels_atomic = xec_dmac##i##_chan_atomic,                                      \
	}
#endif

#define DMA_XEC_DEVICE(i)                                                                          \
	PM_DEVICE_DT_INST_DEFINE(i, xec_dmac_pm_action);                                           \
	XEC_DMAC_CHAN_ATOMIC_DATA(i);                                                              \
	XEC_DMAC_CHAN_DATA(i);                                                                     \
	XEC_DMAC_DATA_DEFINE(i);                                                                   \
	XEC_DMAC_GIRQ_TBL(i);                                                                      \
	XEC_DMAC_CHAN_DEFINE_ISRS(i);                                                              \
	XEC_DMAC_CONFIG_IRQ_DEFINE(i);                                                             \
	static const struct xec_dmac_config xec_dmac##i##_cfg = {                                  \
		.dmac_base = DT_INST_REG_ADDR(i),                                                  \
		.irq_config = xec_dmac##i##_config_irq,                                            \
		.chan_girqs = xec_dmac##i##girq_tbl,                                               \
		.chmsk = GENMASK(XEC_DT_DMAC_CHAN_MAX(i) - 1, 0),                                  \
		.dma_channels = XEC_DT_DMAC_CHAN_MAX(i),                                           \
		.dma_requests = XEC_DT_DMAC_CHAN_REQ_MAX(i),                                       \
		.enc_pcr_src = DT_INST_PROP(i, pcr),                                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(i, xec_dmac_init, PM_DEVICE_DT_INST_GET(i), &xec_dmac##i##_data,     \
			      &xec_dmac##i##_cfg, PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,          \
			      &xec_dmac_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_XEC_DEVICE)
