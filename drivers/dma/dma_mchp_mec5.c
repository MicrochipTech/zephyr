/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_dmac

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/util_macro.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_mchp_mec5, CONFIG_DMA_LOG_LEVEL);

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_ecia_api.h>
#include <mec_dmac_api.h>

struct dma_mec5_config {
	struct mec_dmac_regs *regs;
	uint32_t chmsk;
	uint8_t dma_channels;
	uint8_t dma_requests;
};

struct dma_mec5_channel {
	struct mec_dma_cfg chan_cfg;
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

#define DMA_MEC5_CHAN_FLAGS_CB_EOB_POS		0
#define DMA_MEC5_CHAN_FLAGS_CB_ERR_DIS_POS	1

struct dma_mec5_data {
	struct dma_context ctx;
	struct dma_mec5_channel *channels;
	atomic_t *channels_atomic;
#if defined(CONFIG_PM) || defined(CONFIG_PM_DEVICE)
	atomic_t *channels_pm_atomic;
#endif
#if defined(CONFIG_PM) || !defined(CONFIG_PM_DEVICE)
	const struct pm_state_info *pm_states;
	uint8_t num_pm_states;
#endif
};

/* We have multiple DMA channels, all channels must become idle before
 * we allow deep sleep entry.
 */
#if defined(CONFIG_PM) || defined(CONFIG_PM_DEVICE)
static void dma_mec5_device_busy_set(const struct device *dev, uint8_t chan)
{
	struct dma_mec5_data *const data = dev->data;

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
static void dma_mec5_device_busy_clear(const struct device *dev, uint8_t chan)
{
	struct dma_mec5_data *const data = dev->data;
	uint8_t ch = MEC5_DMAC_NUM_CHANNELS;

	atomic_clear_bit(data->channels_pm_atomic, chan);

	for (ch = 0; ch < MEC5_DMAC_NUM_CHANNELS; ch++) {
		if (atomic_test_bit(data->channels_pm_atomic, ch)) {
			break;
		}
	}

	if (ch != MEC5_DMAC_NUM_CHANNELS) {
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
static void dma_mec5_device_busy_set(const struct device *dev, uint8_t chan)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan);
}

static void dma_mec5_device_busy_clear(const struct device *dev, uint8_t chan)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan);
}
#endif /* #if defined(CONFIG_PM) || defined(CONFIG_PM_DEVICE) */

static bool is_chan_valid(const struct device *dev, uint8_t chan)
{
	const struct dma_mec5_config *const devcfg = dev->config;

	if (devcfg->chmsk & BIT(chan)) {
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
	const struct dma_mec5_config *const devcfg = dev->config;

	if (config->dma_slot >= (uint32_t)devcfg->dma_requests) {
		LOG_ERR("XEC DMA config dma slot > exceeds number of request lines");
		return 0;
	}

	if (config->source_data_size != config->dest_data_size) {
		LOG_ERR("XEC DMA requires source and dest data size identical");
		return 0;
	}

	if (!((config->channel_direction == MEMORY_TO_MEMORY) ||
	      (config->channel_direction == MEMORY_TO_PERIPHERAL) ||
	      (config->channel_direction == PERIPHERAL_TO_MEMORY))) {
		LOG_ERR("XEC DMA only support M2M, M2P, P2M");
		return 0;
	}

	if (!is_dma_data_size_valid(config->source_data_size)) {
		LOG_ERR("XEC DMA requires xfr unit size of 1, 2 or 4 bytes");
		return 0;
	}

	return 1;
}

static int check_blocks(struct dma_mec5_channel *chdata, struct dma_block_config *block,
			uint32_t block_count, uint32_t unit_size)
{
	struct dma_block_config *pb = block;
	uint32_t n = 0, total_len = 0;

	if (!block || !chdata) {
		LOG_ERR("bad pointer");
		return -EINVAL;
	}

	chdata->total_req_xfr_len = 0;

	for (n = 0; n < block_count; n++) {
		if (!pb) {
			LOG_ERR("Block %u config is NULL", n);
			return -EINVAL;
		}

		if ((pb->source_addr_adj == DMA_ADDR_ADJ_DECREMENT) ||
		    (pb->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT)) {
			LOG_ERR("Block %u: HW does not support address decrement", n);
			return -EINVAL;
		}

		if (!is_data_aligned(pb->source_address, pb->dest_address, unit_size)) {
			LOG_ERR("Block %u: violates source/dest unit size", n);
			return -EINVAL;
		}

		total_len += block->block_size;
		pb = pb->next_block;
	}

	chdata->total_req_xfr_len = total_len;

	if (!total_len) {
		LOG_ERR("%u blocks no data", block_count);
		return -EINVAL;
	}

	return 0;
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
static int dma_mec5_configure(const struct device *dev, uint32_t channel,
			      struct dma_config *config)
{
	struct dma_mec5_data *const data = dev->data;
	int ret;

	if (!config || !is_chan_valid(dev, channel)) {
		return -EINVAL;
	}

	if (!is_dma_config_valid(dev, config)) {
		return -EINVAL;
	}

	ret = mec_hal_dma_chan_init((enum mec_dmac_channel)channel);
	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	struct dma_mec5_channel *chdata = &data->channels[channel];
	struct mec_dma_cfg *chcfg = &chdata->chan_cfg;

	chdata->total_req_xfr_len = 0;
	chdata->total_curr_xfr_len = 0;

	struct dma_block_config *block = config->head_block;

	ret = check_blocks(chdata, block, config->block_count, config->source_data_size);
	if (ret) {
		return ret;
	}

	if (config->source_data_size == 4) {
		chcfg->unitsz = MEC_DMAC_UNIT_SIZE_4;
	} else if (config->source_data_size == 2) {
		chcfg->unitsz = MEC_DMAC_UNIT_SIZE_2;
	} else {
		chcfg->unitsz = MEC_DMAC_UNIT_SIZE_1;
	}

	chcfg->flags = 0;
	chcfg->nbytes = block->block_size;
	chcfg->hwfc_dev = (enum mec_dmac_hwfc_dev_id)config->dma_slot;
	chcfg->dir = MEC_DMAC_DIR_MEM_TO_DEV;
	if (config->channel_direction == PERIPHERAL_TO_MEMORY) {
		chcfg->dir = MEC_DMAC_DIR_DEV_TO_MEM;
	} else if (config->channel_direction == MEMORY_TO_MEMORY) {
		/* use HW mem2dev with deviceId = None */
		chcfg->hwfc_dev = MEC_DMAC_DEV_ID_NONE;
	}

	chcfg->src_addr = block->source_address;
	chcfg->dst_addr = block->dest_address;
	if (block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
		chcfg->flags |= MEC_DMA_CFG_FLAG_INCR_SRC_ADDR;
	}
	if (block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
		chcfg->flags |= MEC_DMA_CFG_FLAG_INCR_DST_ADDR;
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
		chdata->flags |= BIT(DMA_MEC5_CHAN_FLAGS_CB_EOB_POS);
	}
	if (config->error_callback_dis) { /* disable callback on errors ? */
		chdata->flags |= BIT(DMA_MEC5_CHAN_FLAGS_CB_ERR_DIS_POS);
	}

	return 0;
}

/* Update previously configured DMA channel with new data source address,
 * data destination address, and size in bytes.
 * src = source address for DMA transfer
 * dst = destination address for DMA transfer
 * size = size of DMA transfer. Assume this is in bytes.
 * We assume the caller will pass src, dst, and size that matches
 * the unit size from the previous configure call.
 */
static int dma_mec5_reload(const struct device *dev, uint32_t channel,
			   uint32_t src, uint32_t dst, size_t size)
{
	struct dma_mec5_data * const data = dev->data;
	int ret = 0;

	if (!is_chan_valid(dev, channel)) {
		return -EINVAL;
	}

	if (mec_hal_dma_chan_is_busy((enum mec_dmac_channel)channel)) {
		return -EBUSY;
	}

	struct dma_mec5_channel *chdata = &data->channels[channel];
	struct mec_dma_cfg *chcfg = &chdata->chan_cfg;

	chcfg->src_addr = src;
	chcfg->dst_addr = dst;
	chcfg->nbytes = size;

	chdata->total_req_xfr_len = size;
	chdata->total_curr_xfr_len = 0;

	ret = mec_hal_dma_chan_reload((enum mec_dmac_channel)channel,
				      (uintptr_t)src, (uintptr_t)dst, size);
	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	return 0;
}

static int dma_mec5_start(const struct device *dev, uint32_t channel)
{

	if (!is_chan_valid(dev, channel)) {
		return -EINVAL;
	}

	struct dma_mec5_data *const data = dev->data;
	struct dma_mec5_channel *chdata = &data->channels[channel];
	struct mec_dma_cfg *chcfg = &chdata->chan_cfg;
	int ret = mec_hal_dma_chan_cfg((enum mec_dmac_channel)channel, chcfg);

	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	ret = mec_hal_dma_chan_intr_en((enum mec_dmac_channel)channel, 0);
	if (ret != MEC_RET_OK) {
		return -EINVAL;
	}

	mec_hal_dma_chan_intr_status_clr((enum mec_dmac_channel)channel);
	mec_hal_dma_chan_intr_en((enum mec_dmac_channel)channel, 1u);

	/* Block PM transition until DMA completes */
	dma_mec5_device_busy_set(dev, channel);

	ret = mec_hal_dma_chan_start((enum mec_dmac_channel)channel);
	if (ret != MEC_RET_OK) {
		/* Release PM lock */
		dma_mec5_device_busy_clear(dev, channel);
		return -EINVAL;
	}

	return 0;
}

static int dma_mec5_stop(const struct device *dev, uint32_t channel)
{
	if (!is_chan_valid(dev, channel)) {
		return -EINVAL;
	}

	int ret = mec_hal_dma_chan_stop((enum mec_dmac_channel)channel);

	if (ret == MEC_RET_OK) {
		ret = 0;
	} else if (ret == MEC_RET_ERR_TIMEOUT) {
		ret = -ETIMEDOUT;
	} else {
		ret = -EINVAL;
	}

	dma_mec5_device_busy_clear(dev, channel);

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
static int dma_mec5_get_status(const struct device *dev, uint32_t channel,
			       struct dma_status *status)
{
	struct dma_mec5_data * const data = dev->data;
	int ret = 0;
	uint32_t rembytes = 0u;
	struct mec_dma_cfg dmacfg = { 0 };

	if (!is_chan_valid(dev, channel) || (!status)) {
		LOG_ERR("unsupported channel");
		return -EINVAL;
	}

	ret =  mec_hal_dma_chan_cfg_get((enum mec_dmac_channel)channel, &dmacfg);
	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	struct dma_mec5_channel *chan_data = &data->channels[channel];

	if (mec_hal_dma_chan_is_busy((enum mec_dmac_channel)channel)) {
		mec_hal_dma_chan_rem_bytes((enum mec_dmac_channel)channel, &rembytes);
		status->busy = true;
		status->pending_length = chan_data->total_req_xfr_len - rembytes;
	} else {
		status->busy = false;
		status->pending_length = (chan_data->total_req_xfr_len
					  - chan_data->total_curr_xfr_len);
	}

	if (dmacfg.hwfc_dev == MEC_DMAC_DEV_ID_NONE) {
		status->dir = MEMORY_TO_MEMORY;
	} else if (dmacfg.dir == MEC_DMAC_DIR_MEM_TO_DEV) {
		status->dir = MEMORY_TO_PERIPHERAL;
	} else {
		status->dir = PERIPHERAL_TO_MEMORY;
	}

	status->total_copied = chan_data->total_curr_xfr_len;

	return 0;
}

static int dma_mec5_get_attribute(const struct device *dev, uint32_t type, uint32_t *value)
{
	if ((type == DMA_ATTR_MAX_BLOCK_COUNT) && value) {
		*value = 1;
		return 0;
	}

	return -EINVAL;
}

/* returns true if filter matched otherwise returns false */
static bool dma_mec5_chan_filter(const struct device *dev, int ch, void *filter_param)
{
	const struct dma_mec5_config *const devcfg = dev->config;
	enum dma_channel_filter *filter = (enum dma_channel_filter *)filter_param;
	uint32_t mask = 0u;

	if ((ch < 0) || (ch > 32) || !is_chan_valid(dev, (uint8_t)(ch & 0x7fu))) {
		return false;
	}

	if (filter && (*filter == DMA_CHANNEL_NORMAL)) {
		mask = GENMASK(devcfg->dma_channels - 1u, 0);
		if (mask & BIT(ch)) {
			return true;
		}
	}

	return false;
}

/* API - HW does not stupport suspend/resume */
static const struct dma_driver_api dma_mec5_api = {
	.config = dma_mec5_configure,
	.reload = dma_mec5_reload,
	.start = dma_mec5_start,
	.stop = dma_mec5_stop,
	.get_status = dma_mec5_get_status,
	.chan_filter = dma_mec5_chan_filter,
	.get_attribute = dma_mec5_get_attribute,
};

#if defined(CONFIG_PM_DEVICE)
/* When PM policy allows suspend or resume this function will be called
 * by the kernel PM subsystem. On suspend we clear the DMA block activate
 * bit which clock gates the block and should cause its CLK_REQ signal to
 * go inactive. On resume we set the DMA block activate ungating clocks in
 * the block. For light sleep (non-suspend) the DMA block can continue
 * operation and its interrupts will wake the CPU.
 */
static int dma_mec5_pm_action(const struct device *dev,
			      enum pm_device_action action)
{
	const struct dma_mec5_config *const devcfg = dev->config;
	struct mec_dmac_regs *const regs = devcfg->regs;
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		mec_hal_dmac_enable(1);
		break;

	case PM_DEVICE_ACTION_SUSPEND:
		mec_hal_dmac_enable(0);
		break;

	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

static void do_callback(const struct device *dev, enum mec_dmac_channel chan,
			struct dma_mec5_channel *chan_data, int status)
{
	if (chan_data->cb) {
		chan_data->cb(dev, chan_data->user_data, chan, status);
	}
}

static void dma_mec5_irq_handler(const struct device *dev, uint8_t chan)
{
	struct dma_mec5_data *const data = dev->data;
	const struct dma_mec5_config *const devcfg = dev->config;
	struct mec_dmac_regs *const regs = devcfg->regs;
	struct dma_mec5_channel *chan_data = &data->channels[chan];
	struct dma_block_config *block = chan_data->curr;
	uint32_t istatus = 0u;

	/* TODO */
	chan_data->total_curr_xfr_len += (regs->CHAN[chan].MEND - regs->CHAN[chan].MSTART);
		/* chan_data->chan_cfg.nbytes */

	mec_hal_dma_chan_intr_en((enum mec_dmac_channel)chan, 0);
	mec_hal_dma_chan_halt((enum mec_dmac_channel)chan);
	mec_hal_dma_chan_intr_status((enum mec_dmac_channel)chan, &istatus);
	mec_hal_dma_chan_intr_status_clr((enum mec_dmac_channel)chan);

	chan_data->isr_hw_status = istatus;

	if (istatus & BIT(MEC_DMA_CHAN_STS_BUS_ERR_POS)) {
		dma_mec5_device_busy_clear(dev, chan);
		if (!(chan_data->flags & BIT(DMA_MEC5_CHAN_FLAGS_CB_ERR_DIS_POS))) {
			do_callback(dev, chan, chan_data, -EIO);
		}
		return;
	}

	chan_data->block_count--;
	if (chan_data->block_count) {
		if ((chan_data->flags & BIT(DMA_MEC5_CHAN_FLAGS_CB_EOB_POS)) && chan_data->cb) {
			do_callback(dev, chan, chan_data, DMA_STATUS_BLOCK);
		}
		block = block->next_block;
		if (block) {
			chan_data->curr = block;
			dma_mec5_reload(dev, chan, block->source_address,
					block->dest_address, block->block_size);
			dma_mec5_start(dev, chan);
		}
	} else {
		dma_mec5_device_busy_clear(dev, chan);
		do_callback(dev, chan, chan_data, DMA_STATUS_COMPLETE);
	}
}

#define DMA_MEC5_NODE_ID(inst) DT_INST(inst, DT_DRV_COMPAT)

#define DMA_MEC5_CHAN_MSK_INST(inst)	\
	(DT_INST_PROP_OR(inst, dma_channel_mask, MEC_DMAC_ALL_CHAN_MASK))

#define DMA_MEC5_USE_CHAN(inst, chan)	\
	(DMA_MEC5_CHAN_MSK_INST(inst) & (BIT(chan)))

#define DMA_MEC5_IRQ_CONNECT(inst, chan)					\
	IRQ_CONNECT(DT_IRQN_BY_IDX(DMA_MEC5_NODE_ID(inst), chan),		\
		    DT_IRQ_BY_IDX(DMA_MEC5_NODE_ID(inst), chan, priority),	\
		    dma_mec5_chan_##chan##_isr,					\
		    DEVICE_DT_INST_GET(inst), 0);				\
	irq_enable(DT_IRQN_BY_IDX(DMA_MEC5_NODE_ID(inst), chan));

#if DMA_MEC5_USE_CHAN(0, 0)
static void dma_mec5_chan_0_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 0);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 1)
static void dma_mec5_chan_1_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 1);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 2)
static void dma_mec5_chan_2_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 2);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 3)
static void dma_mec5_chan_3_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 3);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 4)
static void dma_mec5_chan_4_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 4);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 5)
static void dma_mec5_chan_5_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 5);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 6)
static void dma_mec5_chan_6_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 6);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 7)
static void dma_mec5_chan_7_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 7);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 8)
static void dma_mec5_chan_8_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 8);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 9)
static void dma_mec5_chan_9_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 9);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 10)
static void dma_mec5_chan_10_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 10);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 11)
static void dma_mec5_chan_11_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 11);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 12)
static void dma_mec5_chan_12_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 12);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 13)
static void dma_mec5_chan_13_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 13);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 14)
static void dma_mec5_chan_14_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 14);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 15)
static void dma_mec5_chan_15_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 15);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 16)
static void dma_mec5_chan_16_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 16);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 17)
static void dma_mec5_chan_17_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 17);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 18)
static void dma_mec5_chan_18_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 18);
}
#endif

#if DMA_MEC5_USE_CHAN(0, 19)
static void dma_mec5_chan_19_isr(const struct device *dev)
{
	dma_mec5_irq_handler(dev, 19);
}
#endif

static int dma_mec5_init(const struct device *dev)
{
	struct dma_mec5_data *const data = dev->data;
	const struct dma_mec5_config * const devcfg = dev->config;
	int ret;

	LOG_DBG("driver init");

	data->ctx.magic = DMA_MAGIC;
	data->ctx.dma_channels = devcfg->dma_channels;
	data->ctx.atomic = data->channels_atomic;

	ret = mec_hal_dmac_init(devcfg->chmsk);

	if (ret != MEC_RET_OK) {
		return -EIO;
	}

#if defined(CONFIG_PM) || !defined(CONFIG_PM_DEVICE)
	data->num_pm_states = pm_state_cpu_get_all(0, &data->pm_states);
#endif

#if DMA_MEC5_USE_CHAN(0, 0)
	DMA_MEC5_IRQ_CONNECT(0, 0)
#endif
#if DMA_MEC5_USE_CHAN(0, 1)
	DMA_MEC5_IRQ_CONNECT(0, 1)
#endif
#if DMA_MEC5_USE_CHAN(0, 2)
	DMA_MEC5_IRQ_CONNECT(0, 2)
#endif
#if DMA_MEC5_USE_CHAN(0, 3)
	DMA_MEC5_IRQ_CONNECT(0, 3)
#endif
#if DMA_MEC5_USE_CHAN(0, 4)
	DMA_MEC5_IRQ_CONNECT(0, 4)
#endif
#if DMA_MEC5_USE_CHAN(0, 5)
	DMA_MEC5_IRQ_CONNECT(0, 5)
#endif
#if DMA_MEC5_USE_CHAN(0, 6)
	DMA_MEC5_IRQ_CONNECT(0, 6)
#endif
#if DMA_MEC5_USE_CHAN(0, 7)
	DMA_MEC5_IRQ_CONNECT(0, 7)
#endif
#if DMA_MEC5_USE_CHAN(0, 8)
	DMA_MEC5_IRQ_CONNECT(0, 8)
#endif
#if DMA_MEC5_USE_CHAN(0, 9)
	DMA_MEC5_IRQ_CONNECT(0, 9)
#endif
#if DMA_MEC5_USE_CHAN(0, 10)
	DMA_MEC5_IRQ_CONNECT(0, 10)
#endif
#if DMA_MEC5_USE_CHAN(0, 11)
	DMA_MEC5_IRQ_CONNECT(0, 11)
#endif
#if DMA_MEC5_USE_CHAN(0, 12)
	DMA_MEC5_IRQ_CONNECT(0, 12)
#endif
#if DMA_MEC5_USE_CHAN(0, 13)
	DMA_MEC5_IRQ_CONNECT(0, 13)
#endif
#if DMA_MEC5_USE_CHAN(0, 14)
	DMA_MEC5_IRQ_CONNECT(0, 14)
#endif
#if DMA_MEC5_USE_CHAN(0, 15)
	DMA_MEC5_IRQ_CONNECT(0, 15)
#endif
#if DMA_MEC5_USE_CHAN(0, 16)
	DMA_MEC5_IRQ_CONNECT(0, 16)
#endif
#if DMA_MEC5_USE_CHAN(0, 17)
	DMA_MEC5_IRQ_CONNECT(0, 17)
#endif
#if DMA_MEC5_USE_CHAN(0, 18)
	DMA_MEC5_IRQ_CONNECT(0, 18)
#endif
#if DMA_MEC5_USE_CHAN(0, 19)
	DMA_MEC5_IRQ_CONNECT(0, 19)
#endif

	return 0;
}

#define DMA_MEC5_NUM_CHAN(inst) DT_INST_PROP(inst, dma_channels)

static struct dma_mec5_channel dma_mec5_ctrl0_chans[DT_INST_PROP(0, dma_channels)];

static ATOMIC_DEFINE(dma_mec5_atomic0, DT_INST_PROP(0, dma_channels));

static struct dma_mec5_data dma_mec5_data0 = {
	.ctx.magic = DMA_MAGIC,
	.ctx.dma_channels = DT_INST_PROP(0, dma_channels),
	.ctx.atomic = dma_mec5_atomic0,
	.channels = dma_mec5_ctrl0_chans,
	.channels_atomic = dma_mec5_atomic0,
};

static const struct dma_mec5_config dma_mec5_cfg0 = {
	.regs = (struct mec_dmac_regs *)DT_INST_REG_ADDR(0),
	.chmsk = DT_INST_PROP_OR(0, dma_channel_mask, MEC_DMAC_ALL_CHAN_MASK),
	.dma_channels = DT_INST_PROP(0, dma_channels),
	.dma_requests = DT_INST_PROP(0, dma_requests),
};

PM_DEVICE_DT_DEFINE(0, dma_mec5_pm_action);

DEVICE_DT_INST_DEFINE(0, &dma_mec5_init,
		      PM_DEVICE_DT_GET(0),
		     &dma_mec5_data0, &dma_mec5_cfg0,
		     PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,
		     &dma_mec5_api);
