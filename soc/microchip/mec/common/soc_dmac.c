/**
 *
 * Copyright (c) 2025 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>

#include <soc.h>
#include "soc_dmac.h"

#define XEC_DT_DMAC_NODE_ID   DT_NODELABEL(dmac)
#define XEC_DT_DMAC_BASE_ADDR DT_REG_ADDR(XEC_DT_DMAC_NODE_ID)
#define XEC_DT_DMAC_MAX_CHAN  DT_PROP(XEC_DT_DMAC_NODE_ID, dma_channels)
#define XEC_DT_DMAC_PCR_SCR   DT_PROP(XEC_DT_DMAC_NODE_ID, pcr)
#define XEC_DMAC_GIRQ         14u

#define XEC_DMAC_INTR_VAL(nid, prop, idx)                                                          \
	((uint32_t)DT_PROP_BY_IDX(nid, prop, idx) |                                                \
	 MCHP_XEC_ECIA(0, 0, 0, DT_IRQ_BY_IDX(nid, idx, irq)))

static const uint32_t xec_dmac_intr_enc[] = {
	MCHP_XEC_ECIA(14, 0, 6, 24),   MCHP_XEC_ECIA(14, 1, 6, 25),   MCHP_XEC_ECIA(14, 2, 6, 26),
	MCHP_XEC_ECIA(14, 3, 6, 27),   MCHP_XEC_ECIA(14, 4, 6, 28),   MCHP_XEC_ECIA(14, 5, 6, 29),
	MCHP_XEC_ECIA(14, 6, 6, 30),   MCHP_XEC_ECIA(14, 7, 6, 31),   MCHP_XEC_ECIA(14, 8, 6, 32),
	MCHP_XEC_ECIA(14, 9, 6, 33),   MCHP_XEC_ECIA(14, 10, 6, 34),  MCHP_XEC_ECIA(14, 11, 6, 35),
	MCHP_XEC_ECIA(14, 12, 6, 36),  MCHP_XEC_ECIA(14, 13, 6, 37),  MCHP_XEC_ECIA(14, 14, 6, 38),
	MCHP_XEC_ECIA(14, 15, 6, 39),  MCHP_XEC_ECIA(14, 16, 6, 194), MCHP_XEC_ECIA(14, 17, 6, 195),
	MCHP_XEC_ECIA(14, 18, 6, 196), MCHP_XEC_ECIA(14, 19, 6, 197),
};

static ALWAYS_INLINE mem_addr_t dmac_chan_base(uint32_t chan)
{
	mem_addr_t chan_base = (XEC_DT_DMAC_BASE_ADDR) + (XEC_DMA_CHANNELS_OFS_FROM_BASE);

	return (chan_base + (chan * XEC_DMA_CHAN_REGS_SIZE));
}

mem_addr_t soc_xec_dmac_chan_base(uint32_t chan)
{
	if (chan >= (uint32_t)XEC_DT_DMAC_MAX_CHAN) {
		return 0;
	}

	return dmac_chan_base(chan);
}

bool soc_xec_dmac_is_enabled(void)
{
	mem_addr_t regaddr = XEC_DT_DMAC_BASE_ADDR + XEC_DMA_CHAN_ACTV_OFS;

	if (sys_test_bit(regaddr, XEC_DMA_CHAN_ACTV_EN_POS) != 0) {
		return true;
	}

	return false;
}

int soc_xec_dmac_chan_state_get(uint32_t chan, struct xec_dma_chan_state *ps)
{
	mem_addr_t chb = 0;

	if (chan >= (uint32_t)XEC_DT_DMAC_MAX_CHAN) {
		return -EINVAL;
	}

	chb = dmac_chan_base(chan);

	ps->fsm = sys_read32(chb + XEC_DMA_CHAN_FSM_OFS);
	ps->msa = sys_read32(chb + XEC_DMA_CHAN_MSA_OFS);
	ps->mea = sys_read32(chb + XEC_DMA_CHAN_MEA_OFS);
	ps->deva = sys_read32(chb + XEC_DMA_CHAN_DEVA_OFS);
	ps->ctrl = sys_read32(chb + XEC_DMA_CHAN_CR_OFS);
	ps->sr = sys_read8(chb + XEC_DMA_CHAN_SR_OFS);
	ps->ier = sys_read8(chb + XEC_DMA_CHAN_IER_OFS);
	ps->actv = sys_read8(chb + XEC_DMA_CHAN_ACTV_OFS);

	return 0;
}

int soc_xec_dmac_init(uint32_t chan_girq_en_mask)
{
	mem_addr_t dmac_base = (mem_addr_t)(XEC_DT_DMAC_BASE_ADDR);
	uint32_t n = 0, girq = 0, girq_pos = 0;

	soc_xec_pcr_sleep_en_clear(XEC_DT_DMAC_PCR_SCR);

	sys_set_bit(dmac_base + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_SRST_POS);

	/* self clearing reset */
	while (sys_test_bit(dmac_base + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_SRST_POS) != 0) {
		;
	}

	sys_set_bit(dmac_base + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_EN_POS);

	for (n = 0; n < ARRAY_SIZE(xec_dmac_intr_enc); n++) {
		girq = MCHP_XEC_ECIA_GIRQ(xec_dmac_intr_enc[n]);
		girq_pos = MCHP_XEC_ECIA_GIRQ_POS(xec_dmac_intr_enc[n]);

		soc_ecia_girq_ctrl(girq, girq_pos, 0);
		soc_ecia_girq_status_clear(girq, girq_pos);

		if (chan_girq_en_mask & BIT(n)) {
			soc_ecia_girq_ctrl(girq, girq_pos, 1u);
		}
	}

	return 0;
}

/* Clear a channel. Force memory end address register <= memory start address register by
 * writing 0 before other registers. Writing 0 to activate gates clocks off for the channel.
 */
int soc_xec_dmac_chan_clear(uint32_t chan)
{
	mem_addr_t chb = 0;

	if (chan >= (uint32_t)XEC_DT_DMAC_MAX_CHAN) {
		return -EINVAL;
	}

	chb = dmac_chan_base(chan);

	sys_write32(0, chb + XEC_DMA_CHAN_MEA_OFS);
	sys_write32(0, chb + XEC_DMA_CHAN_ACTV_OFS);
	sys_write32(0, chb + XEC_DMA_CHAN_MSA_OFS);
	sys_write32(0, chb + XEC_DMA_CHAN_DEVA_OFS);
	sys_write32(0, chb + XEC_DMA_CHAN_CR_OFS);
	sys_write32(0, chb + XEC_DMA_CHAN_IER_OFS);
	sys_write32(0xffu, chb + XEC_DMA_CHAN_SR_OFS);

	return 0;
}

int soc_xec_dmac_status_get(uint32_t chan, uint32_t *status)
{
	mem_addr_t chb = 0;

	if (chan >= (uint32_t)XEC_DT_DMAC_MAX_CHAN) {
		return -EINVAL;
	}

	chb = dmac_chan_base(chan);

	if (status != NULL) {
		*status = sys_read32(chb + XEC_DMA_CHAN_SR_OFS);
	}

	return 0;
}

/* configure a channel
 * flags contain:
 *   direction = 0(dev2mem), 1(mem2dev)
 *   HW or SW flow control = 0(hw flow control), 1(sw flow control)
 *   unit size: 0(1 bytes), 1(2 bytes), 2(4 bytes)
 *   increment memory start address by unit size
 *   increment device address by unit size
 *   lock channel as highest priority in arbiter
 *   HW flow control device ID
 */
int soc_xec_dmac_chan_cfg(uint32_t chan, mem_addr_t maddr, mem_addr_t daddr, uint32_t nbytes,
			  uint32_t flags, struct xec_dma_chan_state *ps)
{
	uint32_t ctrl = 0, units = 0, hfc_dev_id = 0;
	mem_addr_t chb = 0;

	if (chan >= (uint32_t)XEC_DT_DMAC_MAX_CHAN) {
		return -EINVAL;
	}

	chb = dmac_chan_base(chan);
	soc_xec_dmac_chan_clear(chan);

	units = XEC_DMAC_CHAN_CFG_UNITS_GET(flags); /* 0, 1, 2 */
	units = (1UL << units);                     /* 1, 2, 4 */
	ctrl = XEC_DMA_CHAN_CR_XU_SET(units);

	if ((flags & XEC_DMAC_CHAN_CFG_MEM2DEV) != 0) {
		ctrl |= BIT(XEC_DMA_CHAN_CR_M2D_POS);
	}

	if ((flags & XEC_DMAC_CHAN_CFG_SFC) == 0) { /* HW flow control */
		hfc_dev_id = XEC_DMAC_CHAN_CFG_HDEVID_GET(flags);
		ctrl |= XEC_DMA_CHAN_CR_HFC_DEV_SET(hfc_dev_id);
	} else {
		ctrl |= BIT(XEC_DMA_CHAN_CR_DIS_HFC_POS);
	}

	if ((flags & BIT(XEC_DMAC_CHAN_CFG_INCRM_POS)) != 0) {
		ctrl |= BIT(XEC_DMA_CHAN_CR_INC_MEM_POS);
	}

	if ((flags & BIT(XEC_DMAC_CHAN_CFG_INCRD_POS)) != 0) {
		ctrl |= BIT(XEC_DMA_CHAN_CR_INC_DEV_POS);
	}

	if ((flags & BIT(XEC_DMAC_CHAN_CFG_LOCK_POS)) != 0) {
		ctrl |= BIT(XEC_DMA_CHAN_CR_LOCK_ARB_POS);
	}

	if (ps != NULL) {
		ps->msa_init = maddr;
		ps->deva_init = daddr;
	}

	sys_write32(maddr, chb + XEC_DMA_CHAN_MSA_OFS);
	sys_write32(maddr + nbytes, chb + XEC_DMA_CHAN_MEA_OFS);
	sys_write32(daddr, chb + XEC_DMA_CHAN_DEVA_OFS);
	sys_write32(ctrl, chb + XEC_DMA_CHAN_CR_OFS);
	sys_set_bit(chb + XEC_DMA_CHAN_ACTV_OFS, XEC_DMA_CHAN_ACTV_EN_POS);

	return 0;
}

int soc_xec_dmac_chan_start(uint32_t chan, uint32_t flags)
{
	mem_addr_t chb = 0;
	uint16_t chan_girq = 0, chan_girq_pos = 0;
	unsigned int run_bit_pos = XEC_DMA_CHAN_CR_HFC_RUN_POS;

	if (chan >= (uint32_t)XEC_DT_DMAC_MAX_CHAN) {
		return -EINVAL;
	}

	chan_girq = MCHP_XEC_ECIA_GIRQ(xec_dmac_intr_enc[chan]);
	chan_girq_pos = MCHP_XEC_ECIA_GIRQ_POS(xec_dmac_intr_enc[chan]);
	chb = dmac_chan_base(chan);

	sys_write32(UINT32_MAX, chb + XEC_DMA_CHAN_SR_OFS);
	soc_ecia_girq_status_clear(chan_girq, chan_girq_pos);

	if (flags & XEC_DMAC_START_IEN) {
		sys_write32(BIT(XEC_DMA_CHAN_IESR_DONE_POS), chb + XEC_DMA_CHAN_IER_OFS);
	}

	if (sys_test_bit(chb + XEC_DMA_CHAN_CR_OFS, XEC_DMA_CHAN_CR_DIS_HFC_POS) != 0) {
		run_bit_pos = XEC_DMA_CHAN_CR_SFC_GO_POS;
	}

	sys_set_bit(chb + XEC_DMA_CHAN_CR_OFS, run_bit_pos);

	return 0;
}
