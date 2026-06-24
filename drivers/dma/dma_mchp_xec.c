/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_dmac

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/mchp_xec_clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/interrupt_controller/intc_mchp_xec_ecia.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util_macro.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_mchp_xec_cdma, CONFIG_DMA_LOG_LEVEL);

#define XEC_DMAC_MAX_CHANS CONFIG_DMA_MCHP_XEC_DMAC_MAX_CHANNELS

/* Hardware has no alignment restriction on buffer addresses
 * other than run time based on byte count.
 */
#define XEC_CDMA_BUF_ADDR_ALIGNMENT 1U
#define XEC_CDMA_BUF_SIZE_ALIGNMENT 1U
#define XEC_CDMA_COPY_ALIGNMENT     1U

/* The XEC central DMA hardware programs one block at a time. The driver
 * caches up to CONFIG_DMA_MCHP_XEC_MAX_BLOCKS_PER_CHAN block descriptors
 * per channel from dma_config()'s linked list and chains them in the
 * channel ISR (ABORT, rewrite MSA/MEA/DEVA + INC bits, set RUN). When
 * the Kconfig knob is 1 (the historical default) the driver behaves
 * exactly as the single-block implementation did.
 */
#define XEC_CDMA_MAX_BLOCK_COUNT CONFIG_DMA_MCHP_XEC_MAX_BLOCKS_PER_CHAN

#define CDMA_MAIN_REGS_SIZE 0x40U
#define CDMA_CHAN_REGS_SIZE 0x40U

/* offset of channels from base */
#define CDMA_CHAN_OFS_FROM_BASE 0x40U

#define CDMA_CHAN_OFS(chan) (((uint32_t)(chan) * CDMA_CHAN_REGS_SIZE) + CDMA_CHAN_OFS_FROM_BASE)

#if defined(CONFIG_SOC_SERIES_MEC175X)
#define CDMA_MAX_CHANNELS  20
#define CDMA_CHAN_ALL_MASK 0xfffffu
#elif defined(CONFIG_SOC_SERIES_MEC15XX)
#define CDMA_MAX_CHANNELS  12
#define CDMA_CHAN_ALL_MASK 0xfffu
#else
#define CDMA_MAX_CHANNELS  16
#define CDMA_CHAN_ALL_MASK 0xffffu
#endif

/* main control */
#define CDMA_MAIN_CR_OFS      0
#define CDMA_MAIN_CR_MSK      GENMASK(1, 0)
#define CDMA_MAIN_CR_EN_POS   0
#define CDMA_MAIN_CR_SRST_POS 1 /* soft-reset of all channels */
/* main data packet 32-bit read-only */
#define CDMA_MAIN_DPKT_OFS    4u /* last data bytes moved by last channel operation */

/* channel activate register */
#define CDMA_CHAN_ACTV_OFS    0
#define CDMA_CHAN_ACTV_EN_POS 0

/* channel memmory start address register (32-bit R/W) */
#define CDMA_CHAN_MSA_OFS 0x4u

/* channel memmory end address register (32-bit R/W) */
#define CDMA_CHAN_MEA_OFS 0x8u

/* channel device address register (32-bit R/W) */
#define CDMA_CHAN_DEVA_OFS 0xcu

/* channel control register */
#define CDMA_CHAN_CR_OFS            0x10u
#define CDMA_CHAN_CR_MSK            (GENMASK(2, 0) | BIT(5) | GENMASK(22, 8) | GENMASK(25, 24))
#define CDMA_CHAN_CR_HFC_RUN_POS    0
#define CDMA_CHAN_CR_REQ_POS        1 /* RO */
#define CDMA_CHAN_CR_DONE_POS       2 /* RO valid only if HFC_RUN is set */
#define CDMA_CHAN_CR_BUSY_POS       5 /* RO FSM is not idle */
#define CDMA_CHAN_CR_M2D_POS        8
#define CDMA_CHAN_CR_HFC_DEV_POS    9
#define CDMA_CHAN_CR_HFC_DEV_MSK    GENMASK(15, 9)
#define CDMA_CHAN_CR_HFC_DEV_MSK0   GENMASK(6, 0)
#define CDMA_CHAN_CR_HFC_DEV_SET(d) FIELD_PREP(CDMA_CHAN_CR_HFC_DEV_MSK, (d))
#define CDMA_CHAN_CR_HFC_DEV_GET(r) FIELD_GET(CDMA_CHAN_CR_HFC_DEV_MSK, (r))
#define CDMA_CHAN_CR_INC_MEM_POS    16
#define CDMA_CHAN_CR_INC_DEV_POS    17
#define CDMA_CHAN_CR_LOCK_ARB_POS   18
#define CDMA_CHAN_CR_DIS_HFC_POS    19
#define CDMA_CHAN_CR_XU_POS         20
#define CDMA_CHAN_CR_XU_MSK         GENMASK(22, 20)
#define CDMA_CHAN_CR_XU_MSK0        GENMASK(2, 0)
#define CDMA_CHAN_CR_XU_BYTES_1     1U
#define CDMA_CHAN_CR_XU_BYTES_2     2U
#define CDMA_CHAN_CR_XU_BYTES_4     4U
#define CDMA_CHAN_CR_XU_SET(u)      FIELD_PREP(CDMA_CHAN_CR_XU_MSK, (u))
#define CDMA_CHAN_CR_XU_GET(r)      FIELD_PREP(CDMA_CHAN_CR_XU_MSK, (r))
#define CDMA_CHAN_CR_SFC_GO_POS     24
#define CDMA_CHAN_CR_ABORT_POS      25

/* channel interrupt status and enable registers */
#define CDMA_CHAN_SR_OFS             0x14u
#define CDMA_CHAN_IER_OFS            0x18u
#define CDMA_CHAN_IESR_MSK           GENMASK(3, 0)
#define CDMA_CHAN_IESR_BERR_POS      0
#define CDMA_CHAN_IESR_OVER_POS      1
#define CDMA_CHAN_IESR_DONE_POS      2
#define CDMA_CHAN_IESR_HFCD_TERM_POS 3

/* channel fsm (RO) */
#define CDMA_CHAN_FSM_OFS            0x1cu
#define CDMA_CHAN_FSM_MSK            GENMASK(15, 0)
#define CDMA_CHAN_FSM_AST_POS        0
#define CDMA_CHAN_FSM_AST_MSK        GENMASK(7, 0)
#define CDMA_CHAN_FSM_AST_GET(fsm)   FIELD_GET(CDMA_CHAN_FSM_AST_MSK, (fsm))
#define CDMA_CHAN_FSM_CST_POS        8
#define CDMA_CHAN_FSM_CST_MSK        GENMASK(15, 8)
#define CDMA_CHAN_FSM_CST_GET(fsm)   FIELD_GET(CDMA_CHAN_FSM_CST_MSK, (fsm))
#define CDMA_CHAN_FSM_CST_IDLE       0
#define CDMA_CHAN_FSM_CST_AREQ_POS   1u
#define CDMA_CHAN_FSM_CST_RD_ACT_POS 2u
#define CDMA_CHAN_FSM_CST_WR_ACT_POS 3u
#define CDMA_CHAN_FSM_CST_WD_POS     4u

struct xec_girq {
	uint8_t gnum;
	uint8_t gpos;
};

struct xec_cdma_xcfg {
	mm_reg_t regbase;
	uint16_t dma_channels;
	uint16_t dma_requests;
	uint16_t enc_pcr;
	uint8_t num_girqs;
	void (*irq_config)(const struct device *);
	const struct xec_girq *girqs;
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
#define XEC_DCHAN_EACH_BLOCK_DONE_CB_POS      0
#define XEC_DCHAN_ERROR_CB_DIS_POS            1

/* Per-block descriptor cached by the driver at dma_config time. The
 * channel CR fields that are common to every block in a chain --
 * direction (M2D), HFC peer/disable, transfer unit -- live in
 * xec_dchan::ctrl_base; only the address-increment bits vary per block
 * (different blocks may legitimately come from different sources or go
 * to different destinations with different adjust modes).
 */
struct xec_dchan_block {
	uint32_t mstart;
	uint32_t dstart;
	uint32_t nbytes;
	uint8_t  inc_bits;   /* CDMA_CHAN_CR_INC_MEM / _INC_DEV only */
};

struct xec_dchan {
	uint32_t ctrl_base;
	struct xec_dchan_block blocks[XEC_CDMA_MAX_BLOCK_COUNT];
	uint8_t num_blocks;
	uint8_t cur_block;
	dma_callback_t cb;
	void *cb_user_data;
	uint8_t flags;
	volatile uint8_t hw_status;
};

struct xec_cdma_xdata {
	struct dma_context ctx;
	struct xec_dchan chdata[XEC_DMAC_MAX_CHANS];
};

/* Reset CDMA block (all channels) */
static void xec_cdma_reset(const struct device *dev)
{
	const struct xec_cdma_xcfg *xcfg = dev->config;
	mm_reg_t rb = xcfg->regbase;

	sys_set_bit(rb + CDMA_MAIN_CR_OFS, CDMA_MAIN_CR_SRST_POS);
	/* wait for FSM to go idle */
	while (sys_test_bit(rb + CDMA_MAIN_CR_OFS, CDMA_MAIN_CR_SRST_POS) != 0) {
	}

	/* reset clears block enable, re-enable block. Channels are disabled after reset */
	sys_set_bit(rb + CDMA_MAIN_CR_OFS, CDMA_MAIN_CR_EN_POS);
}

/* Reset CDMA channel */
static int xec_cdma_chan_reset(const struct device *dev, uint32_t chan)
{
	const struct xec_cdma_xcfg *xcfg = dev->config;
	mm_reg_t rb = xcfg->regbase;

	if (chan >= XEC_DMAC_MAX_CHANS) {
		return -EINVAL;
	}

	rb += CDMA_CHAN_OFS(chan);

	sys_set_bit(rb + CDMA_CHAN_CR_OFS, CDMA_CHAN_CR_ABORT_POS);
	while (sys_test_bit(rb + CDMA_CHAN_CR_OFS, CDMA_CHAN_CR_BUSY_POS) != 0) {
	}

	sys_clear_bit(rb + CDMA_CHAN_ACTV_OFS, CDMA_CHAN_ACTV_EN_POS);
	sys_write32(0, rb + CDMA_CHAN_CR_OFS);
	/* mem end address before mem start address to ensure msa >= mea */
	sys_write32(0, rb + CDMA_CHAN_MEA_OFS);
	sys_write32(0, rb + CDMA_CHAN_MSA_OFS);
	sys_write32(0, rb + CDMA_CHAN_DEVA_OFS);
	sys_write32(0, rb + CDMA_CHAN_IER_OFS);
	sys_write32(CDMA_CHAN_IESR_MSK, rb + CDMA_CHAN_SR_OFS);

	soc_ecia_girq_status_clear(xcfg->girqs[chan].gnum, xcfg->girqs[chan].gpos);

	return 0;
}

static bool xec_cdma_chan_is_busy(const struct device *dev, uint32_t chan)
{
	const struct xec_cdma_xcfg *xcfg = dev->config;
	mm_reg_t rb = xcfg->regbase;
	uint32_t cr = 0;

	if (chan >= XEC_DMAC_MAX_CHANS) {
		return false;
	}

	rb += CDMA_CHAN_OFS(chan);
	cr = sys_read32(rb + CDMA_CHAN_CR_OFS);

	if ((cr & (BIT(CDMA_CHAN_CR_HFC_RUN_POS) | BIT(CDMA_CHAN_CR_SFC_GO_POS))) &&
	    (cr & BIT(CDMA_CHAN_CR_BUSY_POS))) {
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
	const struct xec_cdma_xcfg *xcfg = dev->config;
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
	    (config->block_count > XEC_CDMA_MAX_BLOCK_COUNT)) {
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
 * or program interrupt enables. Used at xec_cdma_start time for the first
 * block of a chain.
 */
static void xec_cdma_load_chan(const struct device *dev, uint32_t chan, uint32_t idx)
{
	const struct xec_cdma_xcfg *xcfg = dev->config;
	struct xec_cdma_xdata *xdat = dev->data;
	struct xec_dchan *chdat = &xdat->chdata[chan];
	struct xec_dchan_block *blk = &chdat->blocks[idx];
	mm_reg_t rb = xcfg->regbase + CDMA_CHAN_OFS(chan);

	(void)xec_cdma_chan_reset(dev, chan);

	sys_write32(blk->mstart, rb + CDMA_CHAN_MSA_OFS);
	sys_write32(blk->mstart + blk->nbytes, rb + CDMA_CHAN_MEA_OFS);
	sys_write32(blk->dstart, rb + CDMA_CHAN_DEVA_OFS);
	sys_write32(chdat->ctrl_base | (uint32_t)blk->inc_bits, rb + CDMA_CHAN_CR_OFS);
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
static void xec_cdma_chan_reprogram(const struct device *dev, uint32_t chan, uint32_t idx)
{
	const struct xec_cdma_xcfg *xcfg = dev->config;
	struct xec_cdma_xdata *xdat = dev->data;
	struct xec_dchan *chdat = &xdat->chdata[chan];
	struct xec_dchan_block *blk = &chdat->blocks[idx];
	mm_reg_t rb = xcfg->regbase + CDMA_CHAN_OFS(chan);
	uint32_t cr;

	sys_set_bit(rb + CDMA_CHAN_CR_OFS, CDMA_CHAN_CR_ABORT_POS);
	while (sys_test_bit(rb + CDMA_CHAN_CR_OFS, CDMA_CHAN_CR_BUSY_POS) != 0) {
	}

	sys_write32(blk->mstart, rb + CDMA_CHAN_MSA_OFS);
	sys_write32(blk->mstart + blk->nbytes, rb + CDMA_CHAN_MEA_OFS);
	sys_write32(blk->dstart, rb + CDMA_CHAN_DEVA_OFS);

	cr = chdat->ctrl_base | (uint32_t)blk->inc_bits;
	sys_write32(cr, rb + CDMA_CHAN_CR_OFS);

	if ((cr & BIT(CDMA_CHAN_CR_DIS_HFC_POS)) == 0) {
		sys_set_bit(rb + CDMA_CHAN_CR_OFS, CDMA_CHAN_CR_HFC_RUN_POS);
	} else {
		sys_set_bit(rb + CDMA_CHAN_CR_OFS, CDMA_CHAN_CR_SFC_GO_POS);
	}
}

/* Translate one dma_block_config into the cached per-block descriptor.
 * The channel-wide CR bits (direction, HFC peer, unit size, DIS_HFC) are
 * already in ctrl_base; only the per-block INC bits land in blk->inc_bits.
 */
static void xec_cdma_translate_block(struct xec_dchan_block *out,
				     const struct dma_block_config *blk,
				     uint32_t direction)
{
	uint8_t inc = 0;

	out->nbytes = blk->block_size;

	if (direction == PERIPHERAL_TO_MEMORY) {
		out->mstart = blk->dest_address;
		out->dstart = blk->source_address;
		if (blk->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			inc |= BIT(CDMA_CHAN_CR_INC_DEV_POS);
		}
		if (blk->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			inc |= BIT(CDMA_CHAN_CR_INC_MEM_POS);
		}
	} else { /* MEMORY_TO_PERIPHERAL or MEMORY_TO_MEMORY */
		out->mstart = blk->source_address;
		out->dstart = blk->dest_address;
		if (blk->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			inc |= BIT(CDMA_CHAN_CR_INC_MEM_POS);
		}
		if (blk->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			inc |= BIT(CDMA_CHAN_CR_INC_DEV_POS);
		}
	}

	out->inc_bits = inc;
}

/* Configure specified DMA channel. Callable from ISR context to switch direction of channel */
static int xec_cdma_config(const struct device *dev, uint32_t chan, struct dma_config *config)
{
	struct xec_cdma_xdata *xdat = dev->data;
	struct xec_dchan *chdat = NULL;
	struct dma_block_config *blk = NULL;
	uint32_t ctrl_base = 0, unitsz = 0;
	uint32_t i;
	int rc = 0;

	if ((chan >= XEC_DMAC_MAX_CHANS) || (config == NULL)) {
		return -EINVAL;
	}

	if (xec_cdma_chan_is_busy(dev, chan)) {
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
	chdat->cb_user_data = config->user_data;

	ctrl_base = CDMA_CHAN_CR_HFC_DEV_SET(config->dma_slot);
	unitsz = MIN(config->source_data_size, config->dest_data_size);
	ctrl_base |= CDMA_CHAN_CR_XU_SET(unitsz);

	if (config->channel_direction == MEMORY_TO_PERIPHERAL) {
		ctrl_base |= BIT(CDMA_CHAN_CR_M2D_POS);
	} else if (config->channel_direction == MEMORY_TO_MEMORY) {
		ctrl_base |= BIT(CDMA_CHAN_CR_M2D_POS) | BIT(CDMA_CHAN_CR_DIS_HFC_POS);
	}
	/* PERIPHERAL_TO_MEMORY: M2D=0, HFC enabled — both already cleared */

	chdat->ctrl_base = ctrl_base;
	chdat->num_blocks = (uint8_t)config->block_count;
	chdat->cur_block = 0;

	blk = config->head_block;
	for (i = 0; i < config->block_count; i++) {
		xec_cdma_translate_block(&chdat->blocks[i], blk, config->channel_direction);
		blk = blk->next_block;
	}

	/* Load HW registers from blocks[0]; do not start. */
	xec_cdma_load_chan(dev, chan, 0);

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
static int xec_cdma_reload(const struct device *dev, uint32_t chan, uint32_t src, uint32_t dst,
			   size_t size)
{
	const struct xec_cdma_xcfg *xcfg = dev->config;
	struct xec_cdma_xdata *xdat = dev->data;
	struct xec_dchan *chdat;
	struct xec_dchan_block *blk;
	mm_reg_t rb = xcfg->regbase;
	bool mem_source;

	if (chan >= XEC_DMAC_MAX_CHANS) {
		return -EINVAL;
	}

	if (xec_cdma_chan_is_busy(dev, chan)) {
		return -EBUSY;
	}

	chdat = &xdat->chdata[chan];
	blk = &chdat->blocks[0];
	rb += CDMA_CHAN_OFS(chan);

	(void)xec_cdma_chan_reset(dev, chan);

	mem_source = (chdat->ctrl_base &
		      (BIT(CDMA_CHAN_CR_M2D_POS) | BIT(CDMA_CHAN_CR_DIS_HFC_POS))) != 0;

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
	/* inc_bits inherited from prior config */

	chdat->num_blocks = 1U;
	chdat->cur_block = 0U;

	sys_write32(blk->mstart, rb + CDMA_CHAN_MSA_OFS);
	sys_write32(blk->mstart + blk->nbytes, rb + CDMA_CHAN_MEA_OFS);
	sys_write32(blk->dstart, rb + CDMA_CHAN_DEVA_OFS);
	sys_write32(chdat->ctrl_base | (uint32_t)blk->inc_bits, rb + CDMA_CHAN_CR_OFS);

	return 0;
}

/* API - start selected channel. Callable from ISR context.
 * Clears channel status
 * Enables channel Done and Bus Error interrupts
 * Starts channel based on current channel control register HW flow control configuration.
 * If HW flow control is Disabled set SW Flow control Go bit
 * Else set HW flow control Run bit.
 */
static int xec_cdma_start(const struct device *dev, uint32_t chan)
{
	const struct xec_cdma_xcfg *xcfg = dev->config;
	struct xec_cdma_xdata *xdat = dev->data;
	struct xec_dchan *chdat;
	mm_reg_t rb = xcfg->regbase;
	uint8_t ier = BIT(CDMA_CHAN_IESR_BERR_POS) | BIT(CDMA_CHAN_IESR_DONE_POS);

	if (chan >= XEC_DMAC_MAX_CHANS) {
		return -EINVAL;
	}

	if (xec_cdma_chan_is_busy(dev, chan)) {
		return -EBUSY;
	}

	chdat = &xdat->chdata[chan];
	chdat->hw_status = 0;
	chdat->cur_block = 0;
	/* HW registers were loaded from blocks[0] at xec_cdma_config or
	 * xec_cdma_reload time; nothing to do here besides arm IER+ACTIVATE
	 * and trip the run bit.
	 */

	rb += CDMA_CHAN_OFS(chan);

	sys_set_bit(rb + CDMA_CHAN_ACTV_OFS, CDMA_CHAN_ACTV_EN_POS);
	sys_write32(CDMA_CHAN_IESR_MSK, rb + CDMA_CHAN_SR_OFS);
	sys_write32((uint32_t)ier, rb + CDMA_CHAN_IER_OFS);

	if (sys_test_bit(rb + CDMA_CHAN_CR_OFS, CDMA_CHAN_CR_DIS_HFC_POS) == 0) {
		sys_set_bit(rb + CDMA_CHAN_CR_OFS, CDMA_CHAN_CR_HFC_RUN_POS);
	} else {
		sys_set_bit(rb + CDMA_CHAN_CR_OFS, CDMA_CHAN_CR_SFC_GO_POS);
	}

	return 0;
}

static int xec_cdma_stop(const struct device *dev, uint32_t chan)
{
	const struct xec_cdma_xcfg *xcfg = dev->config;
	mm_reg_t rb = xcfg->regbase;

	if (chan >= XEC_DMAC_MAX_CHANS) {
		return -EINVAL;
	}

	rb += CDMA_CHAN_OFS(chan);

	sys_set_bit(rb + CDMA_CHAN_CR_OFS, CDMA_CHAN_CR_ABORT_POS);
	while (sys_test_bit(rb + CDMA_CHAN_CR_OFS, CDMA_CHAN_CR_BUSY_POS) != 0) {
	}

	(void)xec_cdma_chan_reset(dev, chan);

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
static int xec_cdma_get_status(const struct device *dev, uint32_t chan, struct dma_status *status)
{
	const struct xec_cdma_xcfg *xcfg = dev->config;
	struct xec_cdma_xdata *xdat = dev->data;
	mm_reg_t rb = xcfg->regbase;
	struct xec_dchan *chdat = NULL;
	uint32_t msa = 0, mea = 0, remaining = 0;
	int chan_status = 0;

	if ((chan >= XEC_DMAC_MAX_CHANS) || (status == NULL)) {
		return -EINVAL;
	}

	status->busy = false;
	if (xec_cdma_chan_is_busy(dev, chan)) {
		status->busy = true;
	}

	status->free = 0;
	status->write_position = 0;
	status->read_position = 0;
	status->total_copied = 0;

	rb += CDMA_CHAN_OFS(chan);
	chdat = &xdat->chdata[chan];

	msa = sys_read32(rb + CDMA_CHAN_MSA_OFS);
	mea = sys_read32(rb + CDMA_CHAN_MEA_OFS);
	if (mea > msa) {
		remaining = mea - msa;
	}
	for (uint8_t i = (uint8_t)(chdat->cur_block + 1U); i < chdat->num_blocks; i++) {
		remaining += chdat->blocks[i].nbytes;
	}
	status->pending_length = remaining;

	if ((chdat->ctrl_base & BIT(CDMA_CHAN_CR_M2D_POS)) != 0) {
		if ((chdat->ctrl_base & BIT(CDMA_CHAN_CR_DIS_HFC_POS)) != 0) {
			status->dir = MEMORY_TO_MEMORY;
		} else {
			status->dir = MEMORY_TO_PERIPHERAL;
		}
	} else {
		status->dir = PERIPHERAL_TO_MEMORY;
	}

	if (chdat->hw_status & BIT(CDMA_CHAN_IESR_BERR_POS)) {
		chan_status = -EIO;
	}

	return chan_status;
}

static bool xec_cdma_chan_filter(const struct device *dev, int chan, void *filter_param)
{
	if ((chan < 0) || (chan >= XEC_DMAC_MAX_CHANS)) {
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

static int xec_cdma_get_attribute(const struct device *dev, uint32_t type, uint32_t *value)
{
	enum dma_attribute_type ctrl_attr = (enum dma_attribute_type)type;

	if (value == NULL) {
		return -EINVAL;
	}

	if (ctrl_attr == DMA_ATTR_BUFFER_ADDRESS_ALIGNMENT) {
		/* required alignment for buffer start address */
		*value = XEC_CDMA_BUF_ADDR_ALIGNMENT;
	} else if (ctrl_attr == DMA_ATTR_BUFFER_SIZE_ALIGNMENT) {
		/* required alignment for total size of the transfer */
		*value = XEC_CDMA_BUF_SIZE_ALIGNMENT;
	} else if (ctrl_attr == DMA_ATTR_COPY_ALIGNMENT) {
		/* minimum data chunk size the contrller can copy
		 * Hardware supports 1, 2, or 4 byte unit sizes.
		 * Hardware can transfer a single byte with unit size of 1 byte
		 */
		*value = DMA_ATTR_COPY_ALIGNMENT;
	} else if (ctrl_attr == DMA_ATTR_MAX_BLOCK_COUNT) {
		*value = XEC_CDMA_MAX_BLOCK_COUNT;
	} else {
		return -EINVAL;
	}

	return 0;
}

/* Called by channel ISR passing the driver device pointer and channel number
 * NOTE: the callback can call any DMA driver API's for this channel.
 *
 * On a successful DONE with more cached blocks remaining, this advances
 * the chain in-place: the channel is ABORTed back to HW IDLE, the next
 * block is programmed via xec_cdma_chan_reprogram, and RUN is re-issued.
 * IER and the GIRQ enables are deliberately left armed across the
 * transition so the next block's DONE re-enters this handler. The
 * per-block callback fires here only when complete_callback_en was set
 * on the dma_config (XEC_DCHAN_EACH_BLOCK_DONE_CB_POS).
 *
 * On error, or on DONE of the last block, the channel is fully quiesced
 * (IER cleared, status W1C, GIRQ acknowledged) and the user callback
 * fires once with the appropriate status.
 */
static void xec_cdma_chan_handler(const struct device *dev, uint32_t chan)
{
	const struct xec_cdma_xcfg *xcfg = dev->config;
	struct xec_cdma_xdata *xdat = dev->data;
	mm_reg_t rb = xcfg->regbase + CDMA_CHAN_OFS(chan);
	struct xec_dchan *chdat = &xdat->chdata[chan];
	uint32_t chan_sr = sys_read32(rb + CDMA_CHAN_SR_OFS);
	bool err = (chan_sr & BIT(CDMA_CHAN_IESR_BERR_POS)) != 0;
	bool last_block = (chdat->cur_block + 1U) >= chdat->num_blocks;

	/* W1C the status bits we observed (DONE, and possibly BERR). DONE
	 * will not re-latch until the next RUN write per the HW spec, so
	 * clearing it here before any reprogram is safe.
	 */
	sys_write32(chan_sr, rb + CDMA_CHAN_SR_OFS);
	soc_ecia_girq_status_clear(xcfg->girqs[chan].gnum, xcfg->girqs[chan].gpos);

	/* CDMA status register implements b[7:0] only */
	chdat->hw_status = (uint8_t)chan_sr;

	if (!err && !last_block) {
		/* Optional per-block callback for the block that just
		 * completed. Fired BEFORE we advance so the application can
		 * inspect status, reload state, etc.
		 */
		if (((chdat->flags & BIT(XEC_DCHAN_EACH_BLOCK_DONE_CB_POS)) != 0) &&
		    (chdat->cb != NULL)) {
			chdat->cb(dev, chdat->cb_user_data, chan, DMA_STATUS_BLOCK);
		}

		chdat->cur_block++;
		xec_cdma_chan_reprogram(dev, chan, chdat->cur_block);
		return;
	}

	/* Terminal: error, or DONE of the final block. */
	sys_write32(0, rb + CDMA_CHAN_IER_OFS);

	if (err) {
		if (((chdat->flags & BIT(XEC_DCHAN_ERROR_CB_DIS_POS)) == 0) &&
		    (chdat->cb != NULL)) {
			chdat->cb(dev, chdat->cb_user_data, chan, -EIO);
		}
	} else if (chdat->cb != NULL) {
		chdat->cb(dev, chdat->cb_user_data, chan, DMA_STATUS_COMPLETE);
	}
}

static int xec_cdma_init(const struct device *dev)
{
	const struct xec_cdma_xcfg *xcfg = dev->config;

	soc_xec_pcr_sleep_en_clear(xcfg->enc_pcr);

	xec_cdma_reset(dev);

	if (xcfg->irq_config != NULL) {
		xcfg->irq_config(dev);
	}

	return 0;
}

/* API - HW does not stupport suspend/resume */
static DEVICE_API(dma, xec_cdma_api) = {
	.config = xec_cdma_config,
	.reload = xec_cdma_reload,
	.start = xec_cdma_start,
	.stop = xec_cdma_stop,
	.get_status = xec_cdma_get_status,
	.chan_filter = xec_cdma_chan_filter,
	.get_attribute = xec_cdma_get_attribute,
};

#define XEC_CDMA_GIRQ_NUM(nid, prop, idx) MCHP_XEC_ECIA_GIRQ(DT_PROP_BY_IDX(nid, prop, idx))
#define XEC_CDMA_GIRQ_POS(nid, prop, idx) MCHP_XEC_ECIA_GIRQ_POS(DT_PROP_BY_IDX(nid, prop, idx))

#define XEC_CDMA_CONN_IRQ(nid, prop, idx, xargs)                                                   \
	IRQ_CONNECT(DT_IRQ_BY_IDX(nid, idx, irq), DT_IRQ_BY_IDX(nid, idx, priority),               \
		    xec_cdma_chan##idx##_isr, DEVICE_DT_GET(nid), 0);                              \
	irq_enable(DT_IRQ_BY_IDX(nid, idx, irq));                                                  \
	soc_ecia_girq_ctrl(xcfg->girqs[idx].gnum, xcfg->girqs[idx].gpos, 1);

#define XEC_CDMA_DECLARE_IRQ(nid, prop, idx)                                                       \
	static void xec_cdma_chan##idx##_isr(const struct device *dev)                             \
	{                                                                                          \
		xec_cdma_chan_handler(dev, idx);                                                   \
	}

#define XEC_CDMA_IRQ_CONNECT(i)                                                                    \
	DT_INST_FOREACH_PROP_ELEM(i, interrupt_names, XEC_CDMA_DECLARE_IRQ)                        \
	static void xec_cdma_irq_cfg##i(const struct device *dev)                                  \
	{                                                                                          \
		const struct xec_cdma_xcfg *xcfg = dev->config;                                    \
		DT_INST_FOREACH_PROP_ELEM_VARGS(i, interrupt_names, XEC_CDMA_CONN_IRQ, xargs);     \
	}

#define XEC_CDMA_GIRQ_ITEM(nid, prop, idx)                                                         \
	{.gnum = XEC_CDMA_GIRQ_NUM(nid, prop, idx), .gpos = XEC_CDMA_GIRQ_POS(nid, prop, idx)},

#define XEC_CDMA_GIRQS(i)                                                                          \
	static const struct xec_girq xec_cdma_girqs_##i[] = {                                      \
		DT_INST_FOREACH_PROP_ELEM(i, girqs, XEC_CDMA_GIRQ_ITEM)};

#define XEC_CDMA_DEVICE(i)                                                                         \
	ATOMIC_DEFINE(xec_cdma_atomic##i, DT_INST_PROP(i, dma_channels));                          \
	static struct xec_cdma_xdata xec_cdma_xdata##i = {                                         \
		.ctx.magic = DMA_MAGIC,                                                            \
		.ctx.dma_channels = DT_INST_PROP(i, dma_channels),                                 \
		.ctx.atomic = xec_cdma_atomic##i,                                                  \
	};                                                                                         \
	XEC_CDMA_IRQ_CONNECT(i)                                                                    \
	XEC_CDMA_GIRQS(i)                                                                          \
	static const struct xec_cdma_xcfg xec_cdma_xcfg##i = {                                     \
		.regbase = (mm_reg_t)DT_INST_REG_ADDR(i),                                          \
		.dma_channels = DT_INST_PROP(i, dma_channels),                                     \
		.dma_requests = DT_INST_PROP(i, dma_requests),                                     \
		.enc_pcr = DT_INST_PROP(i, pcr_scr),                                               \
		.num_girqs = (uint8_t)ARRAY_SIZE(xec_cdma_girqs_##i),                              \
		.irq_config = xec_cdma_irq_cfg##i,                                                 \
		.girqs = xec_cdma_girqs_##i,                                                       \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(i, xec_cdma_init, NULL, &xec_cdma_xdata##i, &xec_cdma_xcfg##i,       \
			      PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY, &xec_cdma_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_CDMA_DEVICE)
