/*
 * Copyright (c) 2023 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_dmac

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/mchp_xec_clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util_macro.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_mchp_xec, CONFIG_DMA_LOG_LEVEL);

#define XEC_DMA_MAX_CHANS CONFIG_DMA_MCHP_XEC_DMAC_MAX_CHANNELS

/* Hardware has no alignment restriction on buffer addresses
 * other than run time based on byte count.
 */
#define XEC_DMA_BUF_ADDR_ALIGNMENT 1U
#define XEC_DMA_BUF_SIZE_ALIGNMENT 1U
#define XEC_DMA_COPY_ALIGNMENT     1U

/* The XEC central DMA hardware programs one block at a time. The driver
 * caches up to CONFIG_DMA_MCHP_XEC_MAX_BLOCKS_PER_CHAN block descriptors
 * per channel from dma_config()'s linked list and chains them in the
 * channel ISR (ABORT, rewrite MSA/MEA/DEVA + INC bits, set RUN). When
 * the Kconfig knob is 1 (the historical default) the driver behaves
 * exactly as the single-block implementation did.
 */
#define XEC_DMA_MAX_BLOCK_COUNT CONFIG_DMA_MCHP_XEC_MAX_BLOCKS_PER_CHAN

#define XEC_DMA_MAIN_REGS_SIZE			0x40
#define XEC_DMA_CHAN_REGS_SIZE			0x40

/* offset of channels from base */
#define XEC_CHAN_OFS_FROM_BASE 0x40U

#define XEC_DMA_CHAN_OFS(chan) \
	(((uint32_t)(chan) * XEC_DMA_CHAN_REGS_SIZE) + XEC_CHAN_OFS_FROM_BASE)

/* main control */
#define XEC_DMA_MAIN_CR_OFS      0
#define XEC_DMA_MAIN_CR_MSK      GENMASK(1, 0)
#define XEC_DMA_MAIN_CR_EN_POS   0
#define XEC_DMA_MAIN_CR_SRST_POS 1 /* soft-reset of all channels */
/* main data packet 32-bit read-only */
#define XEC_DMA_MAIN_DPKT_OFS    4u /* last data bytes moved by last channel operation */

/* channel activate register */
#define XEC_DMA_CHAN_ACTV_OFS    0
#define XEC_DMA_CHAN_ACTV_EN_POS 0

/* channel memmory start address register (32-bit R/W) */
#define XEC_DMA_CHAN_MSA_OFS 0x4u

/* channel memmory end address register (32-bit R/W) */
#define XEC_DMA_CHAN_MEA_OFS 0x8u

/* channel device address register (32-bit R/W) */
#define XEC_DMA_CHAN_DEVA_OFS 0xcu

/* channel control register */
#define XEC_DMA_CHAN_CR_OFS            0x10u
#define XEC_DMA_CHAN_CR_MSK            (GENMASK(2, 0) | BIT(5) | GENMASK(22, 8) | GENMASK(25, 24))
#define XEC_DMA_CHAN_CR_HFC_RUN_POS    0
#define XEC_DMA_CHAN_CR_REQ_POS        1 /* RO */
#define XEC_DMA_CHAN_CR_DONE_POS       2 /* RO valid only if HFC_RUN is set */
#define XEC_DMA_CHAN_CR_BUSY_POS       5 /* RO FSM is not idle */
#define XEC_DMA_CHAN_CR_M2D_POS        8
#define XEC_DMA_CHAN_CR_HFC_DEV_POS    9
#define XEC_DMA_CHAN_CR_HFC_DEV_MSK    GENMASK(15, 9)
#define XEC_DMA_CHAN_CR_HFC_DEV_MSK0   GENMASK(6, 0)
#define XEC_DMA_CHAN_CR_HFC_DEV_SET(d) FIELD_PREP(XEC_DMA_CHAN_CR_HFC_DEV_MSK, (d))
#define XEC_DMA_CHAN_CR_HFC_DEV_GET(r) FIELD_GET(XEC_DMA_CHAN_CR_HFC_DEV_MSK, (r))
#define XEC_DMA_CHAN_CR_INC_MEM_POS    16
#define XEC_DMA_CHAN_CR_INC_DEV_POS    17
#define XEC_DMA_CHAN_CR_LOCK_ARB_POS   18
#define XEC_DMA_CHAN_CR_DIS_HFC_POS    19
#define XEC_DMA_CHAN_CR_XU_POS         20
#define XEC_DMA_CHAN_CR_XU_MSK         GENMASK(22, 20)
#define XEC_DMA_CHAN_CR_XU_MSK0        GENMASK(2, 0)
#define XEC_DMA_CHAN_CR_XU_BYTES_1     1U
#define XEC_DMA_CHAN_CR_XU_BYTES_2     2U
#define XEC_DMA_CHAN_CR_XU_BYTES_4     4U
#define XEC_DMA_CHAN_CR_XU_SET(u)      FIELD_PREP(XEC_DMA_CHAN_CR_XU_MSK, (u))
#define XEC_DMA_CHAN_CR_XU_GET(r)      FIELD_PREP(XEC_DMA_CHAN_CR_XU_MSK, (r))
#define XEC_DMA_CHAN_CR_SFC_GO_POS     24
#define XEC_DMA_CHAN_CR_ABORT_POS      25

/* channel interrupt status and enable registers */
#define XEC_DMA_CHAN_SR_OFS             0x14u
#define XEC_DMA_CHAN_IER_OFS            0x18u
#define XEC_DMA_CHAN_IESR_MSK           GENMASK(3, 0)
#define XEC_DMA_CHAN_IESR_BERR_POS      0
#define XEC_DMA_CHAN_IESR_OVER_POS      1
#define XEC_DMA_CHAN_IESR_DONE_POS      2
#define XEC_DMA_CHAN_IESR_HFCD_TERM_POS 3

/* channel fsm (RO) */
#define XEC_DMA_CHAN_FSM_OFS            0x1cu
#define XEC_DMA_CHAN_FSM_MSK            GENMASK(15, 0)
#define XEC_DMA_CHAN_FSM_AST_POS        0
#define XEC_DMA_CHAN_FSM_AST_MSK        GENMASK(7, 0)
#define XEC_DMA_CHAN_FSM_AST_GET(fsm)   FIELD_GET(XEC_DMA_CHAN_FSM_AST_MSK, (fsm))
#define XEC_DMA_CHAN_FSM_CST_POS        8
#define XEC_DMA_CHAN_FSM_CST_MSK        GENMASK(15, 8)
#define XEC_DMA_CHAN_FSM_CST_GET(fsm)   FIELD_GET(XEC_DMA_CHAN_FSM_CST_MSK, (fsm))
#define XEC_DMA_CHAN_FSM_CST_IDLE       0
#define XEC_DMA_CHAN_FSM_CST_AREQ_POS   1u
#define XEC_DMA_CHAN_FSM_CST_RD_ACT_POS 2u
#define XEC_DMA_CHAN_FSM_CST_WR_ACT_POS 3u
#define XEC_DMA_CHAN_FSM_CST_WD_POS     4u

struct dma_xec_irq_info {
	uint8_t gid;	/* GIRQ id [8, 26] */
	uint8_t gpos;   /* bit position in GIRQ [0, 31] */
};

struct dma_xec_config {
	uintptr_t regs;
	uint8_t dma_requests;
	uint8_t dma_channels;
	uint16_t enc_pcr;
	int irq_info_size;
	const struct dma_xec_irq_info *irq_info_list;
	void (*irq_connect)(const struct device *dev);
};

/* Per-block descriptor cached by the driver at dma_config time. The
 * channel CR fields that are common to every block in a chain --
 * direction (M2D), HFC peer/disable, transfer unit -- live in
 * xec_dchan::ctrl_base; only the address-increment bits vary per block
 * (different blocks may legitimately come from different sources or go
 * to different destinations with different adjust modes).
 */
struct dma_xec_block {
	uint32_t mstart;
	uint32_t dstart;
	uint32_t nbytes;
	uint32_t inc_bits;
};

struct dma_xec_channel {
	uint32_t control;
	volatile uint32_t isr_hw_status;
	uint8_t num_blocks;
	uint8_t cur_block;
	bool cyclic; /* on last-block DONE wrap to block 0 */
	uint8_t flags;
	dma_callback_t cb;
	void *user_data;
	struct dma_xec_block blocks[XEC_DMA_MAX_BLOCK_COUNT];
};

/* DMA callback flags from struct dma_config.
 * Default behavior is to invoke the user callback once when the entire
 * transfer list completes, and once on any error along the way.
 * The caller can request changes in callback behavior:
 *  - Disable error callback (error_callback_dis).
 *  - Fire a callback at every block boundary instead of only after
 *    the last block (complete_callback_en). The block-mid-list
 *    "halfway" callback flag in dma_config is not supported -- this
 *    HW has no halfway-through-list interrupt.
 */
#define XEC_DCHAN_EACH_BLOCK_DONE_CB_POS 0
#define XEC_DCHAN_ERROR_CB_DIS_POS       1

struct dma_xec_data {
	struct dma_context ctx;
#ifdef CONFIG_PM_DEVICE
	atomic_t active_channel_count;
#endif
	struct dma_xec_channel chdata[XEC_DMA_MAX_CHANS];
};

/* Reset DMA Controller (all channels) */
static void xec_cdma_reset(const struct device *dev)
{
	const struct dma_xec_config *xcfg = dev->config;
	uintptr_t rb = xcfg->regs;

	sys_set_bit(rb + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_SRST_POS);
	/* wait for FSM to go idle */
	while (sys_test_bit(rb + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_SRST_POS) != 0) {
	}

	/* reset clears block enable, re-enable block. Channels are disabled after reset */
	sys_set_bit(rb + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_EN_POS);
}

/* Reset XEC_DMA channel */
static int dma_xec_chan_reset(const struct device *dev, uint32_t chan)
{
	const struct dma_xec_config *xcfg = dev->config;
	uintptr_t rb = xcfg->regs;

	if (chan >= XEC_DMA_MAX_CHANS) {
		return -EINVAL;
	}

	rb += XEC_DMA_CHAN_OFS(chan);

	sys_set_bit(rb + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_ABORT_POS);
	while (sys_test_bit(rb + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_BUSY_POS) != 0) {
	}

	sys_clear_bit(rb + XEC_DMA_CHAN_ACTV_OFS, XEC_DMA_CHAN_ACTV_EN_POS);
	sys_write32(0, rb + XEC_DMA_CHAN_CR_OFS);
	/* mem end address before mem start address to ensure msa >= mea */
	sys_write32(0, rb + XEC_DMA_CHAN_MEA_OFS);
	sys_write32(0, rb + XEC_DMA_CHAN_MSA_OFS);
	sys_write32(0, rb + XEC_DMA_CHAN_DEVA_OFS);
	sys_write32(0, rb + XEC_DMA_CHAN_IER_OFS);
	sys_write32(XEC_DMA_CHAN_IESR_MSK, rb + XEC_DMA_CHAN_SR_OFS);

	soc_ecia_girq_status_clear(xcfg->irq_info_list[chan].gid, xcfg->irq_info_list[chan].gpos);

	return 0;
}

static bool xec_dma_chan_is_busy(const struct device *dev, uint32_t chan)
{
	const struct dma_xec_config *xcfg = dev->config;
	uintptr_t rb = xcfg->regs;
	uint32_t cr = 0;

	if (chan >= XEC_DMA_MAX_CHANS) {
		return false;
	}

	rb += XEC_DMA_CHAN_OFS(chan);
	cr = sys_read32(rb + XEC_DMA_CHAN_CR_OFS);

	if ((cr & (BIT(XEC_DMA_CHAN_CR_HFC_RUN_POS) | BIT(XEC_DMA_CHAN_CR_SFC_GO_POS))) &&
	    (cr & BIT(XEC_DMA_CHAN_CR_BUSY_POS))) {
		return true;
	}

	return false;
}

static int validate_data_size(uint32_t data_size)
{
	if ((data_size == 1U) || (data_size == 2U) || (data_size == 4U)) {
		return 0;
	}

	return -EINVAL;
}

static int validate_chan_dir(uint32_t dir)
{
	if ((dir == MEMORY_TO_MEMORY) || (dir == MEMORY_TO_PERIPHERAL) ||
	    (dir == PERIPHERAL_TO_MEMORY)) {
		return 0;
	}

	return -EINVAL;
}

static int validate_dma_block(struct dma_block_config *block)
{
	if (block->block_size == 0) {
		return -EINVAL;
	}

	if ((block->source_addr_adj == DMA_ADDR_ADJ_DECREMENT) ||
	    (block->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT)) {
		return -EINVAL;
	}

	return 0;
}

/* Validate DMA configuration
 * Microchip XEC central DMA control hardware supports:
 * Directions: Mem-to-Periph, Periph-to-Mem, or Mem-to-Mem
 * Bus transfer unit sizes of 1, 2, or 4 bytes.
 * Optional increment of source and destination addresses (no decrement)
 *    increment size is bus transfer unit size
 * No channel suspend, channel stops on completion, error, or HW flow control termination
 *    from peripheral device
 *
 * Implementation:
 * We will use source and dest data size as the unit size: 1, 2, or 4
 *
 * Notes:
 * struct dma_config passed by the caller can be ephemeral. We must translate and process
 * all configuration into this driver's data and/or hardware registers. dma_config has a
 * pointer to a linked list of dma_block_config entries and a block_count; we walk that
 * list at config-time and cache up to CONFIG_DMA_MCHP_XEC_MAX_BLOCKS_PER_CHAN of them
 * into per-channel storage. The channel ISR advances through the cached chain by
 * ABORT/reprogram/restart on every DONE.
 */
static int validate_dma_config(const struct device *dev, struct dma_config *config)
{
	const struct dma_xec_config *xcfg = dev->config;
	struct dma_block_config *blk;
	uint32_t i;

	if (config->dma_slot >= xcfg->dma_requests) {
		return -EINVAL;
	}

	if (config->half_complete_callback_en != 0) {
		return -EINVAL;
	}

	if ((validate_data_size(config->source_data_size) != 0) ||
	    (validate_data_size(config->dest_data_size))) {
		return -EINVAL;
	}

	if (validate_chan_dir(config->channel_direction) != 0) {
		return -EINVAL;
	}

	if (config->source_handshake != config->dest_handshake) {
		return -EINVAL;
	}

	if ((config->head_block == NULL) || (config->block_count == 0U) ||
	    (config->block_count > XEC_DMA_MAX_BLOCK_COUNT)) {
		return -EINVAL;
	}

	blk = config->head_block;
	for (i = 0; i < config->block_count; i++) {
		int rc;

		if (blk == NULL) {
			return -EINVAL;
		}

		rc = validate_dma_block(blk);
		if (rc != 0) {
			return rc;
		}

		blk = blk->next_block;
	}

	/* Chain must terminate at block_count; surplus entries are a
	 * configuration error rather than silently ignored.
	 */
	if (blk != NULL) {
		return -EINVAL;
	}

	return 0;
}

/* Reset channel and load mem start address, mem end address, device address,
 * and control register from the cached block at index `idx`. Does not enable
 * or program interrupt enables. Used at dma_xec_start time for the first
 * block of a chain.
 */
static void dma_xec_load_chan(const struct device *dev, uint32_t chan, uint32_t idx)
{
	const struct dma_xec_config *xcfg = dev->config;
	struct dma_xec_data *xdat = dev->data;
	struct dma_xec_channel *chdat = &xdat->chdata[chan];
	struct dma_xec_block *blk = &chdat->blocks[idx];
	uintptr_t rb = xcfg->regs + XEC_DMA_CHAN_OFS(chan);

	(void)dma_xec_chan_reset(dev, chan);

	sys_write32(blk->mstart, rb + XEC_DMA_CHAN_MSA_OFS);
	sys_write32(blk->mstart + blk->nbytes, rb + XEC_DMA_CHAN_MEA_OFS);
	sys_write32(blk->dstart, rb + XEC_DMA_CHAN_DEVA_OFS);
	sys_write32(chdat->control | (uint32_t)blk->inc_bits, rb + XEC_DMA_CHAN_CR_OFS);
}

/* Fast-path reprogram used by the channel ISR to chain to the next block
 * without tearing down the channel state we need to keep (IER, ACTIVATE,
 * GIRQ enables). The HW design team's guidance for issue SCG_MR_22NM-131
 * is: a still-pending peripheral request can spontaneously start the new
 * transfer the instant MSA<MEA becomes true after a reprogram, so we must
 * use the channel ABORT bit to force HW IDLE before writing the new block.
 * ABORT at natural DONE settles immediately; mid-transfer it waits at most
 * one unit-size AHB transfer (~83 ns at 48 MHz worst case for a 4-byte
 * unit), which is negligible compared to any peripheral byte time we care
 * about. The DONE latch was W1C-cleared by the caller before we got here
 * and will not re-latch until the new RUN write below takes effect.
 */
static void dma_xec_chan_reprogram(const struct device *dev, uint32_t chan, uint32_t idx)
{
	const struct dma_xec_config *xcfg = dev->config;
	struct dma_xec_data *xdat = dev->data;
	struct dma_xec_channel *chdat = &xdat->chdata[chan];
	struct dma_xec_block *blk = &chdat->blocks[idx];
	uintptr_t rb = xcfg->regs + XEC_DMA_CHAN_OFS(chan);
	uint32_t cr;

	sys_set_bit(rb + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_ABORT_POS);
	while (sys_test_bit(rb + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_BUSY_POS) != 0) {
	}

	sys_write32(blk->mstart, rb + XEC_DMA_CHAN_MSA_OFS);
	sys_write32(blk->mstart + blk->nbytes, rb + XEC_DMA_CHAN_MEA_OFS);
	sys_write32(blk->dstart, rb + XEC_DMA_CHAN_DEVA_OFS);

	cr = chdat->control | (uint32_t)blk->inc_bits;
	sys_write32(cr, rb + XEC_DMA_CHAN_CR_OFS);

	if ((cr & BIT(XEC_DMA_CHAN_CR_DIS_HFC_POS)) == 0) {
		sys_set_bit(rb + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_HFC_RUN_POS);
	} else {
		sys_set_bit(rb + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_SFC_GO_POS);
	}
}

/* Translate one dma_block_config into the cached per-block descriptor.
 * The channel-wide CR bits (direction, HFC peer, unit size, DIS_HFC) are
 * already in control; only the per-block INC bits land in blk->inc_bits.
 */
static void dma_xec_translate_block(struct dma_xec_block *out,
				     const struct dma_block_config *blk, uint32_t direction)
{
	uint32_t inc = 0;

	out->nbytes = blk->block_size;

	if (direction == PERIPHERAL_TO_MEMORY) {
		out->mstart = blk->dest_address;
		out->dstart = blk->source_address;
		if (blk->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			inc |= BIT(XEC_DMA_CHAN_CR_INC_DEV_POS);
		}
		if (blk->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			inc |= BIT(XEC_DMA_CHAN_CR_INC_MEM_POS);
		}
	} else { /* MEMORY_TO_PERIPHERAL or MEMORY_TO_MEMORY */
		out->mstart = blk->source_address;
		out->dstart = blk->dest_address;
		if (blk->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			inc |= BIT(XEC_DMA_CHAN_CR_INC_MEM_POS);
		}
		if (blk->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			inc |= BIT(XEC_DMA_CHAN_CR_INC_DEV_POS);
		}
	}

	out->inc_bits = inc;
}

/* Configure specified DMA channel. Callable from ISR context to switch direction of channel */
static int dma_xec_configure(const struct device *dev, uint32_t chan, struct dma_config *config)
{
	struct dma_xec_data *xdat = dev->data;
	struct dma_xec_channel *chdat = NULL;
	struct dma_block_config *blk = NULL;
	uint32_t control = 0, unitsz = 0, i = 0;
	int rc = 0;

	if ((chan >= XEC_DMA_MAX_CHANS) || (config == NULL)) {
		return -EINVAL;
	}

	if (xec_dma_chan_is_busy(dev, chan)) {
		return -EBUSY;
	}

	/* validate params we care about: direction, etc.*/
	rc = validate_dma_config(dev, config);
	if (rc != 0) {
		return rc;
	}

	chdat = &xdat->chdata[chan];

	chdat->flags = 0;
	if (config->complete_callback_en != 0) {
		chdat->flags = BIT(XEC_DCHAN_EACH_BLOCK_DONE_CB_POS);
	}

	if (config->error_callback_dis != 0) {
		chdat->flags |= BIT(XEC_DCHAN_ERROR_CB_DIS_POS);
	}

	chdat->cb = config->dma_callback;
	chdat->user_data = config->user_data;

	control = XEC_DMA_CHAN_CR_HFC_DEV_SET(config->dma_slot);
	unitsz = MIN(config->source_data_size, config->dest_data_size);
	control |= XEC_DMA_CHAN_CR_XU_SET(unitsz);

	if (config->channel_direction == MEMORY_TO_PERIPHERAL) {
		control |= BIT(XEC_DMA_CHAN_CR_M2D_POS);
	} else if (config->channel_direction == MEMORY_TO_MEMORY) {
		control |= BIT(XEC_DMA_CHAN_CR_M2D_POS) | BIT(XEC_DMA_CHAN_CR_DIS_HFC_POS);
	}
	/* PERIPHERAL_TO_MEMORY: M2D=0, HFC enabled — both already cleared */

	chdat->control = control;
	chdat->num_blocks = (uint8_t)config->block_count;
	chdat->cur_block = 0;
	chdat->cyclic = (config->cyclic != 0U);

	blk = config->head_block;
	for (i = 0; i < config->block_count; i++) {
		dma_xec_translate_block(&chdat->blocks[i], blk, config->channel_direction);
		blk = blk->next_block;
	}

	/* Load HW registers from blocks[0]; do not start. */
	dma_xec_load_chan(dev, chan, 0);

	return 0;
}

/* Reload DMA channel and do not start. Callable from ISR context.
 * Channel configuration: direction, flow control, etc. are not changed.
 * We only reprogram the memory start, memory end, and device addresses.
 *
 * Reload collapses any multi-block chain previously configured on this
 * channel into a single block (num_blocks = 1) at cur_block = 0. The
 * caller's expectation for reload is "replace whatever block was about
 * to run next", so chains beyond block 0 are not preserved -- callers
 * needing chain-style behavior re-issue dma_config.
 */
static int dma_xec_reload(const struct device *dev, uint32_t chan, uint32_t src, uint32_t dst,
			  size_t size)
{
	const struct dma_xec_config *xcfg = dev->config;
	struct dma_xec_data *xdat = dev->data;
	struct dma_xec_channel *chdat;
	struct dma_xec_block *blk;
	uintptr_t rb = xcfg->regs;
	bool mem_source;

	if (chan >= XEC_DMA_MAX_CHANS) {
		return -EINVAL;
	}

	if (xec_dma_chan_is_busy(dev, chan)) {
		return -EBUSY;
	}

	chdat = &xdat->chdata[chan];
	blk = &chdat->blocks[0];
	rb += XEC_DMA_CHAN_OFS(chan);

	(void)dma_xec_chan_reset(dev, chan);

	mem_source = (chdat->control &
		      (BIT(XEC_DMA_CHAN_CR_M2D_POS) | BIT(XEC_DMA_CHAN_CR_DIS_HFC_POS))) != 0;

	if (mem_source) {
		/* memory to memory or memory to peripheral */
		blk->mstart = src;
		blk->dstart = dst;
	} else {
		/* peripheral to memory */
		blk->mstart = dst;
		blk->dstart = src;
	}
	blk->nbytes = (uint32_t)size;

	chdat->num_blocks = 1U;
	chdat->cur_block = 0U;
	chdat->cyclic = false;

	sys_write32(blk->mstart, rb + XEC_DMA_CHAN_MSA_OFS);
	sys_write32(blk->mstart + blk->nbytes, rb + XEC_DMA_CHAN_MEA_OFS);
	sys_write32(blk->dstart, rb + XEC_DMA_CHAN_DEVA_OFS);
	sys_write32(chdat->control | (uint32_t)blk->inc_bits, rb + XEC_DMA_CHAN_CR_OFS);

	return 0;
}

/* API - start selected channel. Callable from ISR context.
 * Clears channel status
 * Enables channel Done and Bus Error interrupts
 * Starts channel based on current channel control register HW flow control configuration.
 * If HW flow control is Disabled set SW Flow control Go bit
 * Else set HW flow control Run bit.
 */
static int dma_xec_start(const struct device *dev, uint32_t chan)
{
	const struct dma_xec_config *xcfg = dev->config;
	struct dma_xec_data *xdat = dev->data;
	struct dma_xec_channel *chdat = NULL;
	uintptr_t rb = xcfg->regs;
	uint8_t ier = (BIT(XEC_DMA_CHAN_IESR_BERR_POS) | BIT(XEC_DMA_CHAN_IESR_DONE_POS) |
		       BIT(XEC_DMA_CHAN_IESR_HFCD_TERM_POS));

	if (chan >= XEC_DMA_MAX_CHANS) {
		return -EINVAL;
	}

	if (xec_dma_chan_is_busy(dev, chan)) {
		return -EBUSY;
	}

	chdat = &xdat->chdata[chan];
	chdat->isr_hw_status = 0;
	chdat->cur_block = 0;
	/* HW registers were loaded from blocks[0] at dma_xec_configure or
	 * dma_xec_reload time; nothing to do here besides arm IER+ACTIVATE
	 * and trip the run bit.
	 */

	rb += XEC_DMA_CHAN_OFS(chan);

	sys_set_bit(rb + XEC_DMA_CHAN_ACTV_OFS, XEC_DMA_CHAN_ACTV_EN_POS);
	sys_write32(XEC_DMA_CHAN_IESR_MSK, rb + XEC_DMA_CHAN_SR_OFS);
	sys_write32((uint32_t)ier, rb + XEC_DMA_CHAN_IER_OFS);

	if (sys_test_bit(rb + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_DIS_HFC_POS) == 0) {
		sys_set_bit(rb + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_HFC_RUN_POS);
	} else {
#ifdef CONFIG_PM_DEVICE
		if (atomic_inc(&xdat->active_channel_count) == 0) { /* returns previous value */
			pm_device_busy_set(dev);
		}
#endif
		sys_set_bit(rb + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_SFC_GO_POS);
	}

	return 0;
}

#ifdef CONFIG_PM_DEVICE
/* The atomic decrement function does not check if the value is
 * already zero and will cause wrap. This routine handles this
 * corner case in a thread safe way.
 */
static void xec_cdma_check_and_clear_busy(const struct device *dev)
{
	struct dma_xec_data *xdat = dev->data;
	atomic_t prev = 0;
	bool success = false;

	do {
		prev = atomic_get(&xdat->active_channel_count);
		if (prev == 0) {
			break;
		}

		success = atomic_cas(&xdat->active_channel_count, prev, prev - 1);
	} while (!success);

	if (success && (prev == 1)) {
		pm_device_busy_clear(dev);
	}
}
#endif

/* Stopping a channel while it is running requires using the CR.ABORT bit and spinning
 * for the channel to clear read-only CR.BUSY status. Busy will clear when the current
 * unit (byte, 16-bit half-word, or 32-bit word) completes. When not busy only clear the
 * abort, HW flow control run, and SW flow control go bits. Do not clear other bits or
 * registers.
 */
static int dma_xec_stop(const struct device *dev, uint32_t chan)
{
	const struct dma_xec_config *xcfg = dev->config;
	uintptr_t rb = xcfg->regs;

	if (chan >= XEC_DMA_MAX_CHANS) {
		return -EINVAL;
	}

	rb += XEC_DMA_CHAN_OFS(chan);

	sys_set_bit(rb + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_ABORT_POS);
	while (sys_test_bit(rb + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_BUSY_POS) != 0) {
	}

	sys_clear_bits(rb + XEC_DMA_CHAN_CR_OFS,
		       (BIT(XEC_DMA_CHAN_CR_HFC_RUN_POS) | BIT(XEC_DMA_CHAN_CR_SFC_GO_POS) |
			BIT(XEC_DMA_CHAN_CR_ABORT_POS)));

#ifdef CONFIG_PM_DEVICE
	xec_cdma_check_and_clear_busy(dev);
#endif

	return 0;
}

/* Microchip XEC central DMA does not support cyclic transfers.
 * We zero free, write_position, and read_position members.
 * Hardware does not implement a transferred by count accumlator therefore
 * we can't provide a total_copied value.
 *
 * pending_length is the live MEA-MSA of the currently running block plus
 * the cached block_size of every block that has not started yet. After
 * the final block's DONE this resolves to zero.
 */
static int dma_xec_get_status(const struct device *dev, uint32_t chan, struct dma_status *status)
{
	const struct dma_xec_config *xcfg = dev->config;
	struct dma_xec_data *xdat = dev->data;
	uintptr_t rb = xcfg->regs;
	struct dma_xec_channel *chdat = NULL;
	uint32_t msa = 0, mea = 0, remaining = 0;
	int chan_status = 0;

	if ((chan >= XEC_DMA_MAX_CHANS) || (status == NULL)) {
		return -EINVAL;
	}

	status->busy = false;
	if (xec_dma_chan_is_busy(dev, chan)) {
		status->busy = true;
	}

	status->free = 0;
	status->write_position = 0;
	status->read_position = 0;
	status->total_copied = 0;

	rb += XEC_DMA_CHAN_OFS(chan);
	chdat = &xdat->chdata[chan];

	msa = sys_read32(rb + XEC_DMA_CHAN_MSA_OFS);
	mea = sys_read32(rb + XEC_DMA_CHAN_MEA_OFS);
	if (mea > msa) {
		remaining = mea - msa;
	}
	for (uint8_t i = (uint8_t)(chdat->cur_block + 1U); i < chdat->num_blocks; i++) {
		remaining += chdat->blocks[i].nbytes;
	}
	status->pending_length = remaining;

	if ((chdat->control & BIT(XEC_DMA_CHAN_CR_M2D_POS)) != 0) {
		if ((chdat->control & BIT(XEC_DMA_CHAN_CR_DIS_HFC_POS)) != 0) {
			status->dir = MEMORY_TO_MEMORY;
		} else {
			status->dir = MEMORY_TO_PERIPHERAL;
		}
	} else {
		status->dir = PERIPHERAL_TO_MEMORY;
	}

	if (chdat->isr_hw_status & BIT(XEC_DMA_CHAN_IESR_BERR_POS)) {
		chan_status = -EIO;
	}

	return chan_status;
}

static bool dma_xec_chan_filter(const struct device *dev, int chan, void *filter_param)
{
	if ((chan < 0) || (chan >= XEC_DMA_MAX_CHANS)) {
		return false; /* bad channel number */
	}

	if (filter_param == NULL) { /* allow any valid channel */
		return true;
	}

	/* Hardware only supports normal channels */
	if (*((enum dma_channel_filter *)filter_param) == DMA_CHANNEL_NORMAL) {
		return true;
	}

	return false;
}

static int xec_dma_get_attribute(const struct device *dev, uint32_t type, uint32_t *value)
{
	enum dma_attribute_type ctrl_attr = (enum dma_attribute_type)type;

	if (value == NULL) {
		return -EINVAL;
	}

	if (ctrl_attr == DMA_ATTR_BUFFER_ADDRESS_ALIGNMENT) {
		/* required alignment for buffer start address */
		*value = XEC_DMA_BUF_ADDR_ALIGNMENT;
	} else if (ctrl_attr == DMA_ATTR_BUFFER_SIZE_ALIGNMENT) {
		/* required alignment for total size of the transfer */
		*value = XEC_DMA_BUF_SIZE_ALIGNMENT;
	} else if (ctrl_attr == DMA_ATTR_COPY_ALIGNMENT) {
		/* minimum data chunk size the contrller can copy
		 * Hardware supports 1, 2, or 4 byte unit sizes.
		 * Hardware can transfer a single byte with unit size of 1 byte
		 */
		*value = DMA_ATTR_COPY_ALIGNMENT;
	} else if (ctrl_attr == DMA_ATTR_MAX_BLOCK_COUNT) {
		*value = XEC_DMA_MAX_BLOCK_COUNT;
	} else {
		return -EINVAL;
	}

	return 0;
}

static inline void dma_xec_chan_clr_girq(const struct dma_xec_config *xcfg, uint32_t chan)
{
	if (chan >= XEC_DMA_MAX_CHANS) {
		return;
	}

	soc_ecia_girq_status_clear(xcfg->irq_info_list[chan].gid, xcfg->irq_info_list[chan].gpos);
}

/* Called by channel ISR passing the driver device pointer and channel number
 * NOTE: the callback can call any DMA driver API's for this channel.
 *
 * Handle bus error, transfer terminated by flow control device, or done.
 * Bus Error:
 * Disable the channel's interrupts
 * If the caller enabled the error callback we invoke it passing -EIO
 * Exit ISR
 * Termination by flow control device:
 * The peripheral using DMA terminated the transfer.
 * Disable the channel's interrupts
 * If the caller enabled the normal callback we invoke it with DMA_STATUS_DONE because
 * the DMA driver does not have a status value for this kind of termination.
 * Exit ISR
 * 
 * Done:
 * The DMA channel completed the transfer it was programmed for. The channel's memory start
 * address register was incremented until equal to the memory end address register value.
 * If there are more blocks to transfer we reconfigure and start the channel for the next block.
 * If no more blocks we invoke the callback (if enabled) and Exit ISR
 * 
 * If the channel was configured cyclic (config->cyclic), DONE on the
 * final block wraps back to block 0 instead of taking the terminal path:
 * the per-block callback fires (if enabled), cur_block resets to 0, and
 * the channel is reprogrammed and restarted. Useful for ping-pong
 * peripheral-to-memory transfers where the application drains each
 * chunk during its per-block callback and the channel keeps cycling
 * until the peripheral signals end-of-transfer (HFC_TERM) or an
 * external dma_stop call.
 *
 * On error, or on DONE of the last block in a non-cyclic chain, the
 * channel is fully quiesced (IER cleared, status W1C, GIRQ acknowledged)
 * and the user callback fires once with the appropriate status.
 */
static void dma_xec_irq_handler(const struct device *dev, uint32_t channel)
{
	const struct dma_xec_config *xcfg = dev->config;
	struct dma_xec_data *xdat = dev->data;
	uintptr_t rb = xcfg->regs + XEC_DMA_CHAN_OFS(channel);
	struct dma_xec_channel *chdat = &xdat->chdata[channel];
	uint32_t chan_sr = sys_read32(rb + XEC_DMA_CHAN_SR_OFS);
	bool err = (chan_sr & BIT(XEC_DMA_CHAN_IESR_BERR_POS)) != 0;
	bool hw_term = (chan_sr & BIT(XEC_DMA_CHAN_IESR_HFCD_TERM_POS)) != 0;
	bool last_block = (chdat->cur_block + 1U) >= chdat->num_blocks;
	bool advance = !err && !hw_term && (!last_block || chdat->cyclic);

	/* W1C the status bits we observed (DONE, and possibly BERR). DONE
	 * will not re-latch until the next RUN write per the HW spec, so
	 * clearing it here before any reprogram is safe.
	 */
	sys_write32(chan_sr, rb + XEC_DMA_CHAN_SR_OFS);
	dma_xec_chan_clr_girq(xcfg, channel);

	/* XEC_DMA status register implements b[7:0] only */
	chdat->isr_hw_status = chan_sr;

	if (err) {
		sys_write32(0, rb + XEC_DMA_CHAN_IER_OFS);
#ifdef CONFIG_PM_DEVICE
		if (sys_test_bit(rb + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_SFC_GO_POS)) {
			xec_cdma_check_and_clear_busy(dev);
		}
#endif
		if (((chdat->flags & BIT(XEC_DCHAN_ERROR_CB_DIS_POS)) == 0) &&
		    (chdat->cb != NULL)) {
			chdat->cb(dev, chdat->user_data, channel, -EIO);
		}
		return;
	}

	if (advance) {
		uint32_t next = last_block ? 0U : (uint32_t)(chdat->cur_block + 1U);

		/* Optional per-block callback for the block that just
		 * completed. Fired BEFORE we advance so the application can
		 * inspect status, reload state, etc.
		 */
		if (((chdat->flags & BIT(XEC_DCHAN_EACH_BLOCK_DONE_CB_POS)) != 0) &&
		    (chdat->cb != NULL)) {
			chdat->cb(dev, chdat->user_data, channel, DMA_STATUS_BLOCK);
		}

		chdat->cur_block = (uint8_t)next;
		dma_xec_chan_reprogram(dev, channel, chdat->cur_block);
		return;
	}

	/* Flow control peripheral terminated or channel DONE of the final block in a
	 * non-cyclic chain. DMA driver has no completion status for a peripheral
	 * terminating the DMA early.
	 */
	sys_write32(0, rb + XEC_DMA_CHAN_IER_OFS);
#ifdef CONFIG_PM_DEVICE
	if (sys_test_bit(rb + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_SFC_GO_POS)) {
		xec_cdma_check_and_clear_busy(dev);
	}
#endif
	if (chdat->cb != NULL) {
		chdat->cb(dev, chdat->user_data, channel, DMA_STATUS_COMPLETE);
	}
}

#ifdef CONFIG_PM_DEVICE
static void dma_xec_all_girq_en(const struct device *dev, bool enable)
{
	const struct dma_xec_config *xcfg = dev->config;
	uint8_t enval = (enable) ? MCHP_MEC_ECIA_GIRQ_EN : MCHP_MEC_ECIA_GIRQ_DIS;

	for (int i = 0; i < xcfg->irq_info_size; i++) {
		soc_ecia_girq_status_clear(xcfg->irq_info_list[i].gid, xcfg->irq_info_list[i].gpos);
		soc_ecia_girq_ctrl(xcfg->irq_info_list[i].gid, xcfg->irq_info_list[i].gpos, enval);		
	}
}

static int dmac_xec_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct dma_xec_config *xcfg = dev->config;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		break; /* HW clears CLK_REQ when it is not moving data */
	case PM_DEVICE_ACTION_RESUME:
		break; /* No action required */
	case PM_DEVICE_ACTION_TURN_OFF:
		sys_clear_bit(xcfg->regs + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_EN_POS);
		dma_xec_all_girq_en(dev, false);
		break;
	case PM_DEVICE_ACTION_TURN_ON:
		xec_cdma_reset(dev);
		dma_xec_all_girq_en(dev, true);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif

static int dma_xec_init(const struct device *dev)
{
	const struct dma_xec_config *xcfg = dev->config;

	soc_xec_pcr_sleep_en_clear(xcfg->enc_pcr);

	xec_cdma_reset(dev);

#ifdef CONFIG_PM_DEVICE
	struct dma_xec_data *xdat = dev->data;

	atomic_set(&xdat->active_channel_count, 0);
#endif

	if (xcfg->irq_connect != NULL) {
		xcfg->irq_connect(dev);
	}

	return 0;
}

/* API - HW does not stupport suspend/resume */
static DEVICE_API(dma, dma_xec_api) = {
	.config = dma_xec_configure,
	.reload = dma_xec_reload,
	.start = dma_xec_start,
	.stop = dma_xec_stop,
	.get_status = dma_xec_get_status,
	.chan_filter = dma_xec_chan_filter,
	.get_attribute = xec_dma_get_attribute,
};

#define XEC_DMA_GIRQ_NUM(nid, prop, idx) MCHP_XEC_ECIA_GIRQ(DT_PROP_BY_IDX(nid, prop, idx))
#define XEC_DMA_GIRQ_POS(nid, prop, idx) MCHP_XEC_ECIA_GIRQ_POS(DT_PROP_BY_IDX(nid, prop, idx))

#define XEC_DMA_CONN_IRQ(nid, prop, idx, xargs)                                                    \
	IRQ_CONNECT(DT_IRQ_BY_IDX(nid, idx, irq), DT_IRQ_BY_IDX(nid, idx, priority),               \
		    dma_xec_chan_##idx##_isr, DEVICE_DT_GET(nid), 0);                              \
	irq_enable(DT_IRQ_BY_IDX(nid, idx, irq));                                                  \
	soc_ecia_girq_ctrl(xcfg->irq_info_list[idx].gid, xcfg->irq_info_list[idx].gpos, 1);

#define XEC_DMA_DECLARE_IRQ(nid, prop, idx)                                                        \
	static void dma_xec_chan_##idx##_isr(const struct device *dev)                             \
	{                                                                                          \
		dma_xec_irq_handler(dev, idx);                                                     \
	}

#define XEC_DMA_IRQ_CONNECT(i)                                                                     \
	DT_INST_FOREACH_PROP_ELEM(i, interrupt_names, XEC_DMA_DECLARE_IRQ)                         \
	static void dma_xec_irq_connect##i(const struct device *dev)                               \
	{                                                                                          \
		const struct dma_xec_config *xcfg = dev->config;                                   \
		DT_INST_FOREACH_PROP_ELEM_VARGS(i, interrupt_names, XEC_DMA_CONN_IRQ, xargs);      \
	}

#define XEC_DMA_GIRQ_ITEM(nid, prop, idx)                                                          \
	{.gid = XEC_DMA_GIRQ_NUM(nid, prop, idx), .gpos = XEC_DMA_GIRQ_POS(nid, prop, idx)},

#define XEC_DMA_GIRQS(i)                                                                           \
	static const struct dma_xec_irq_info dma_xec_irqi##i[] = {                                 \
		DT_INST_FOREACH_PROP_ELEM(i, girqs, XEC_DMA_GIRQ_ITEM)};

#define DMA_XEC_DEVICE(i)                                                                          \
	ATOMIC_DEFINE(dma_xec_atomic##i, DT_INST_PROP(i, dma_channels));                           \
	static struct dma_xec_data dma_xec_dat##i = {                                              \
		.ctx.magic = DMA_MAGIC,                                                            \
		.ctx.dma_channels = DT_INST_PROP(i, dma_channels),                                 \
		.ctx.atomic = dma_xec_atomic##i,                                                   \
	};                                                                                         \
	XEC_DMA_IRQ_CONNECT(i)                                                                     \
	XEC_DMA_GIRQS(i)                                                                           \
	static const struct dma_xec_config dma_xec_cfg##i = {                                      \
		.regs = (uintptr_t)DT_INST_REG_ADDR(i),                                            \
		.dma_requests = DT_INST_PROP(i, dma_requests),                                     \
		.dma_channels = DT_INST_PROP(i, dma_channels),                                     \
		.enc_pcr = DT_INST_PROP(i, pcr_scr),                                               \
		.irq_info_size = (int)ARRAY_SIZE(dma_xec_irqi##i),                                 \
		.irq_info_list = dma_xec_irqi##i,                                                  \
		.irq_connect = dma_xec_irq_connect##i,                                             \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(i, dmac_xec_pm_action);                                           \
	DEVICE_DT_INST_DEFINE(i, dma_xec_init, PM_DEVICE_DT_INST_GET(i), &dma_xec_dat##i,          \
			      &dma_xec_cfg##i, PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,             \
			      &dma_xec_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_XEC_DEVICE)
