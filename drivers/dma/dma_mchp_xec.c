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
#include <zephyr/sys/util_macro.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_mchp_xec, CONFIG_DMA_LOG_LEVEL);

#define XEC_DMA_CFG_FLAG_INCR_SRC_ADDR	BIT(0)
#define XEC_DMA_CFG_FLAG_INCR_DST_ADDR	BIT(1)
#define XEC_DMA_CFG_FLAG_SWFLC		BIT(2)
#define XEC_DMA_CFG_DIR_DEV_TO_MEM	0
#define XEC_DMA_CFG_DIR_MEM_TO_DEV	1u

struct xec_dma_chan_cfg {
	uintptr_t src_addr;
	uintptr_t dst_addr;
	size_t nbytes;
	uint8_t unitsz;
	uint8_t dir;
	uint8_t hwfc_dev;
	uint8_t flags;
};

struct dma_xec_config {
	mem_addr_t dmac_base;
	void (*irq_config)(void);
	const uint16_t *chan_girqs;
	uint32_t chmsk;
	uint8_t dma_channels;
	uint8_t dma_requests;
};

struct dma_xec_channel {
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

#define DMA_XEC_CHAN_FLAGS_CB_EOB_POS		0
#define DMA_XEC_CHAN_FLAGS_CB_ERR_DIS_POS	1

struct dma_xec_data {
	struct dma_context ctx;
	struct dma_xec_channel *channels;
	atomic_t *channels_atomic;
#if defined(CONFIG_PM) || defined(CONFIG_PM_DEVICE)
	atomic_t *channels_pm_atomic;
#endif
#if defined(CONFIG_PM) || !defined(CONFIG_PM_DEVICE)
	const struct pm_state_info *pm_states;
	uint8_t num_pm_states;
#endif
};

static mem_addr_t xec_dma_chan_base(const struct device *dev, uint32_t chan)
{
	const struct dma_xec_config *const devcfg = dev->config;
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
	struct dma_xec_data *const data = dev->data;

	atomic_set_bit(data->channels_pm_atomic, chan);

#if defined(CONFIG_PM_DEVICE)
	pm_device_busy_set(dev);
#elif defined(CONFIG_PM)
	for (uint8_t n = 0; n < data->num_pm_states; n++) {
		if (data->pm_states[n].state >= PM_STATE_STANDBY) {
			pm_policy_state_lock_put(data->pm_states[n].state, PM_ALL_SUBSTATES);
		}
	}
#endif
}

/* Clear the channel's atomic pm bit.
 * If all atomic bits are clear then clear the PM busy flag the kernel looks at.
 */
static void dma_xec_device_busy_clear(const struct device *dev, uint8_t chan)
{
	const struct dma_xec_config *const devcfg = dev->config;
	struct dma_xec_data *const data = dev->data;
	uint8_t ch = 0;

	atomic_clear_bit(data->channels_pm_atomic, chan);

	for (ch = 0; ch < devcfg->dma_channels; ch++) {
		if (atomic_test_bit(data->channels_pm_atomic, ch)) {
			break;
		}
	}

	if (ch != devcfg->dma_channels) {
		return;
	}

#if defined(CONFIG_PM_DEVICE)
	pm_device_busy_clear(dev);
#elif defined(CONFIG_PM)
	for (uint8_t n = 0; n < data->num_pm_states; n++) {
		if (data->pm_states[n].state >= PM_STATE_STANDBY) {
			pm_policy_state_lock_get(data->pm_states[n].state, PM_ALL_SUBSTATES);
		}
	}
#endif
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
	const struct dma_xec_config *const devcfg = dev->config;

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
	const struct dma_xec_config *const devcfg = dev->config;

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

static int check_blocks(struct dma_xec_channel *chdata, struct dma_block_config *block,
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
	const struct dma_xec_config *const devcfg = dev->config;
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
	uint32_t msa = 0, mea = 0, deva = 0;
	uint32_t mend = sys_read32(chan_base + XEC_DMA_CHAN_MEA_OFS);

	/* ensure HW is "done" by mstart == mend */
	sys_write32(mend, chan_base + XEC_DMA_CHAN_MSA_OFS);
	/* keep mend <= mstart */
	sys_write32(0, chan_base + XEC_DMA_CHAN_MEA_OFS);

	if (sys_test_bit(chan_base + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_M2D_POS) != 0) {
		msa = src;
		mea = src + nbytes;
		deva = dest;
	} else { /* device to memory */
		msa = dest;
		mea = dest + nbytes;
		deva = src;
	}

	/* order: deva, msa, the mea */
	sys_write32(deva, chan_base + XEC_DMA_CHAN_DEVA_OFS);
	sys_write32(msa, chan_base + XEC_DMA_CHAN_MSA_OFS);
	sys_write32(mea, chan_base + XEC_DMA_CHAN_MEA_OFS);
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
static int dma_xec_configure(const struct device *dev, uint32_t channel,
			      struct dma_config *config)
{
	struct dma_xec_data *const data = dev->data;
	struct dma_xec_channel *chdata = NULL;
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

static void dma_xec_reload_int(const struct device *dev, uint32_t channel, uint32_t src,
			       uint32_t dst, size_t size)
{
	struct dma_xec_data * const data = dev->data;
	struct dma_xec_channel *chdata = NULL;
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
static int dma_xec_reload(const struct device *dev, uint32_t channel,
			  uint32_t src, uint32_t dst, size_t size)
{
	mem_addr_t chan_base = 0;

	if (is_chan_valid(dev, channel) == false) {
		return -EINVAL;
	}

	chan_base = xec_dma_chan_base(dev, channel);

	if (xec_dma_chan_is_busy(chan_base) == true) {
		return -EBUSY;
	}

	dma_xec_reload_int(dev, channel, src, dst, size);

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
	struct dma_xec_data *const data = dev->data;
	struct xec_dma_chan_cfg *chcfg = NULL;
	struct dma_xec_channel *chdata = NULL;
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
	sys_write32(BIT(XEC_DMA_CHAN_IESR_DONE_POS), chan_base + XEC_DMA_CHAN_IER_OFS);

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
		while (sys_test_bit(chan_base + XEC_DMA_CHAN_CR_OFS,
				    XEC_DMA_CHAN_CR_BUSY_POS) != 0) {
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
	struct dma_xec_data * const data = dev->data;
	struct dma_xec_channel *chan_data = NULL;
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
		status->pending_length = (chan_data->total_req_xfr_len
					  - chan_data->total_curr_xfr_len);
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

/* returns true if filter matched otherwise returns false */
static bool dma_xec_chan_filter(const struct device *dev, int ch, void *filter_param)
{
	const struct dma_xec_config *const devcfg = dev->config;
	enum dma_channel_filter *filter = (enum dma_channel_filter *)filter_param;

	if ((ch < 0) || (ch >= devcfg->dma_channels)) {
		return false;
	}

	if ((filter != NULL) && (*filter == DMA_CHANNEL_NORMAL)) {
		if ((devcfg->chmsk & BIT(ch)) != 0) {
			return true;
		}
	}

	return false;
}

/* API - HW does not stupport suspend/resume */
static const struct dma_driver_api dma_xec_api = {
	.config = dma_xec_configure,
	.reload = dma_xec_reload,
	.start = dma_xec_start,
	.stop = dma_xec_stop,
	.get_status = dma_xec_get_status,
	.chan_filter = dma_xec_chan_filter,
	.get_attribute = dma_xec_get_attribute,
};

#if defined(CONFIG_PM_DEVICE)
/* When PM policy allows suspend or resume this function will be called
 * by the kernel PM subsystem. On suspend we clear the DMA block activate
 * bit which clock gates the block and should cause its CLK_REQ signal to
 * go inactive. On resume we set the DMA block activate ungating clocks in
 * the block. For light sleep (non-suspend) the DMA block can continue
 * operation and its interrupts will wake the CPU.
 */
static int dma_xec_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct dma_xec_config *const devcfg = dev->config;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		sys_set_bit(devcfg->dmac_base + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_EN_POS);
		break;

	case PM_DEVICE_ACTION_SUSPEND:
		sys_clear_bit(devcfg->dmac_base + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_EN_POS);
		break;

	default:
		return -ENOTSUP;
	}

	return 0
}
#endif /* CONFIG_PM_DEVICE */

static void do_callback(const struct device *dev, uint32_t chan, struct dma_xec_channel *chan_data,
			int status)
{
	if (chan_data->cb) {
		chan_data->cb(dev, chan_data->user_data, chan, status);
	}
}

static void dma_xec_irq_handler(const struct device *dev, uint8_t chan)
{
	const struct dma_xec_config * const devcfg = dev->config;
	struct dma_xec_data *const data = dev->data;
	struct dma_xec_channel *chan_data = &data->channels[chan];
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
	sys_clear_bits(BIT(XEC_DMA_CHAN_CR_HFC_RUN_POS) | BIT(XEC_DMA_CHAN_CR_SFC_GO_POS),
		       chb + XEC_DMA_CHAN_CR_OFS);
	sys_clear_bit(chb + XEC_DMA_CHAN_ACTV_OFS, XEC_DMA_CHAN_ACTV_EN_POS);
	sys_write32(XEC_DMA_CHAN_IESR_MSK, chb + XEC_DMA_CHAN_SR_OFS);
	soc_ecia_girq_status_clear(girq, girq_pos);

	if (istatus & BIT(XEC_DMA_CHAN_IESR_BERR_POS)) {
		dma_xec_device_busy_clear(dev, chan);
		if ((chan_data->flags & BIT(DMA_XEC_CHAN_FLAGS_CB_ERR_DIS_POS)) == 0) {
			do_callback(dev, chan, chan_data, -EIO);
		}
		return;
	}

	chan_data->block_count--;
	if (chan_data->block_count) {
		if (((chan_data->flags & BIT(DMA_XEC_CHAN_FLAGS_CB_EOB_POS)) != 0) &&
		    (chan_data->cb != NULL)) {
			do_callback(dev, chan, chan_data, DMA_STATUS_BLOCK);
		}

		block = block->next_block;
		if (block != NULL) {
			chan_data->curr = block;
			dma_xec_reload_int(dev, chan, block->source_address, block->dest_address,
					   block->block_size);
			dma_xec_start(dev, chan);
		}
	} else {
		dma_xec_device_busy_clear(dev, chan);
		do_callback(dev, chan, chan_data, DMA_STATUS_COMPLETE);
	}
}

static int dma_xec_init(const struct device *dev)
{
	struct dma_xec_data *const data = dev->data;
	const struct dma_xec_config * const devcfg = dev->config;
	mem_addr_t dmac_base = devcfg->dmac_base;

	LOG_DBG("driver init");

	data->ctx.magic = DMA_MAGIC;
	data->ctx.dma_channels = devcfg->dma_channels;
	data->ctx.atomic = data->channels_atomic;

#if defined(CONFIG_PM) || !defined(CONFIG_PM_DEVICE)
	data->num_pm_states = pm_state_cpu_get_all(0, &data->pm_states);
#endif
	sys_set_bit(dmac_base + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_SRST_POS);
	sys_write32(0, dmac_base + XEC_DMA_MAIN_DPKT_OFS); /* delay */
	sys_set_bit(dmac_base + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_EN_POS);

	if (devcfg->irq_config != NULL) {
		devcfg->irq_config();
	}

	return 0;
}

#define DMA_XEC_NUM_CHAN(inst) DT_INST_PROP_OR(inst, dma_channels, XEC_DMAC_MAX_CHAN)

#define DMA_XEC_CHAN_GIRQ(node_id, idx) \
	MCHP_XEC_ECIA_GIRQ(DT_PROP_BY_IDX(node_id, girqs, idx))

#define DMA_XEC_CHAN_GIRQ_POS(node_id, idx) \
	MCHP_XEC_ECIA_GIRQ_POS(DT_PROP_BY_IDX(node_id, girqs, idx))

#define DMA_XEC_CHAN_MASK(node_id) DT_PROP_OR(node_id, dma_channel_mask, XEC_DMAC_CHAN_MSK)

#define XEC_DMA_CHAN_EN(node_id, chan) ((DMA_XEC_CHAN_MASK(node_id) & BIT(chan)) != 0)

/* Unused ISR's don't disappear. Compiler prodces 2-byte "bx lr" */
#define DMA_XEC_IRQ_DECLARE_FUNC(node_id, prop, idx)						\
	static void dma_xec_chan##idx##_isr(const struct device *dev)				\
	{											\
		if (XEC_DMA_CHAN_EN(node_id, idx)) {						\
			dma_xec_irq_handler(dev, (uint8_t)idx);					\
		}										\
	}

#define DMA_XEC_IRQ_DECLARE(inst)								\
	DT_INST_FOREACH_PROP_ELEM(inst, girqs, DMA_XEC_IRQ_DECLARE_FUNC)

#define DMA_XEC_IRQ_CONNECT(node_id, prop, idx)							\
	IRQ_CONNECT(DT_IRQ_BY_IDX(node_id, idx, irq), DT_IRQ_BY_IDX(node_id, idx, priority),	\
		    dma_xec_chan##idx##_isr, DEVICE_DT_GET(node_id), 0);			\
	irq_enable(DT_IRQ_BY_IDX(node_id, idx, irq));						\
	soc_ecia_girq_ctrl(DMA_XEC_CHAN_GIRQ(node_id, idx),					\
			   DMA_XEC_CHAN_GIRQ_POS(node_id, idx), 1u);

/* If channel's bit not set in channel bitmap mask then do not connect its IRQ */
#define DMA_XEC_IRQ_CONNECT_AVAIL(node_id, prop, idx)						\
	if (XEC_DMA_CHAN_EN(node_id, idx)) {							\
		DMA_XEC_IRQ_CONNECT(node_id, prop, idx)						\
	}


#define DMA_XEC_GIRQ_ELEM(nid, prop, idx) DT_PROP_BY_IDX(nid, prop, idx)

#define DMA_XEC_GIRQ_TABLE(inst)								\
	const uint16_t xec_dmac##inst_chan_girq[] = {						\
		DT_INST_FOREACH_PROP_ELEM_SEP(inst, girqs, DMA_XEC_GIRQ_ELEM, (,))		\
	}

#define DMA_XEC_DEVICE(inst)									\
	static ATOMIC_DEFINE(dma_xec_atomic##inst, DMA_XEC_NUM_CHAN(inst));			\
	static struct dma_xec_channel dma_xec_ctrl##inst##_chans[DMA_XEC_NUM_CHAN(inst)];	\
	static struct dma_xec_data dma_xec_data##inst = {					\
		.ctx.magic = DMA_MAGIC,								\
		.ctx.dma_channels = DT_INST_PROP(inst, dma_channels),				\
		.ctx.atomic = dma_xec_atomic##inst,						\
		.channels = dma_xec_ctrl##inst##_chans,						\
		.channels_atomic = dma_xec_atomic##inst,					\
	};											\
	DMA_XEC_IRQ_DECLARE(inst);								\
	static void xec_dma_irq_config##inst(void)						\
	{											\
		DT_INST_FOREACH_PROP_ELEM(inst, girqs, DMA_XEC_IRQ_CONNECT_AVAIL)		\
	}											\
	DMA_XEC_GIRQ_TABLE(inst);								\
	static const struct dma_xec_config dma_xec_cfg##inst = {				\
		.dmac_base = (mem_addr_t)DT_INST_REG_ADDR(inst),				\
		.irq_config = xec_dma_irq_config##inst,						\
		.chan_girqs = xec_dmac##inst_chan_girq,						\
		.chmsk = DT_INST_PROP_OR(inst, dma_channel_mask, XEC_DMAC_CHAN_MSK),		\
		.dma_channels = DMA_XEC_NUM_CHAN(inst),						\
		.dma_requests = DT_INST_PROP_OR(inst, dma_requests, XEC_DMAC_MAX_CHAN),		\
	};											\
	PM_DEVICE_DT_DEFINE(inst, dma_xec_pm_action);						\
	DEVICE_DT_INST_DEFINE(inst, &dma_xec_init,						\
			      PM_DEVICE_DT_INST_GET(inst),					\
			      &dma_xec_data##inst, &dma_xec_cfg##inst,				\
			      PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,				\
			      &dma_xec_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_XEC_DEVICE)
