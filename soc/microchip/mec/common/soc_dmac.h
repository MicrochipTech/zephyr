/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_MICROCHIP_MEC_COMMON_XEC_DMAC_H_
#define _SOC_MICROCHIP_MEC_COMMON_XEC_DMAC_H_

#include <zephyr/sys/sys_io.h>	/* mem_addr_t definition */
#include <zephyr/sys/util.h>	/* GENMASK and friends */

/* API */

#define XEC_DMAC_CHAN_CFG_DEV2MEM	0
#define XEC_DMAC_CHAN_CFG_MEM2DEV	BIT(0)
#define XEC_DMAC_CHAN_CFG_HFC		0
#define XEC_DMAC_CHAN_CFG_SFC		BIT(1)
#define XEC_DMAC_CHAN_CFG_UNITS_POS	2
#define XEC_DMAC_CHAN_CFG_UNITS_MSK	GENMASK(3, 2)
#define XEC_DMAC_CHAN_CFG_UNITS_1	0
#define XEC_DMAC_CHAN_CFG_UNITS_2	1
#define XEC_DMAC_CHAN_CFG_UNITS_4	2
#define XEC_DMAC_CHAN_CFG_UNITS_SET(u)	FIELD_PREP(XEC_DMAC_CHAN_CFG_UNITS_MSK, (u))
#define XEC_DMAC_CHAN_CFG_UNITS_GET(c)	FIELD_GET(XEC_DMAC_CHAN_CFG_UNITS_MSK, (c))
#define XEC_DMAC_CHAN_CFG_INCRM_POS	4
#define XEC_DMAC_CHAN_CFG_INCRD_POS	5
#define XEC_DMAC_CHAN_CFG_LOCK_POS	6
#define XEC_DMAC_CHAN_CFG_HDEVID_POS	8
#define XEC_DMAC_CHAN_CFG_HDEVID_MSK	GENMASK(14, 8)
#define XEC_DMAC_CHAN_CFG_HDEVID_SET(h)	FIELD_PREP(XEC_DMAC_CHAN_CFG_HDEVID_MSK, (h))
#define XEC_DMAC_CHAN_CFG_HDEVID_GET(c)	FIELD_GET(XEC_DMAC_CHAN_CFG_HDEVID_MSK, (c))

#define XEC_DMAC_CHAN_CFG_INCRM		BIT(XEC_DMAC_CHAN_CFG_INCRM_POS)
#define XEC_DMAC_CHAN_CFG_INCRD		BIT(XEC_DMAC_CHAN_CFG_INCRD_POS)
#define XEC_DMAC_CHAN_CFG_LOCK		BIT(XEC_DMAC_CHAN_CFG_LOCK_POS)

struct xec_dma_chan_state
{
	uint32_t msa_init;
	uint32_t deva_init;
	uint32_t msa;
	uint32_t mea;
	uint32_t deva;
	uint32_t ctrl;
	uint32_t fsm;
	uint8_t sr;
	uint8_t ier;
	uint8_t actv;
};

struct soc_xec_dma_chan_cfg {
	uint32_t daddr;
	uint32_t maddr;
	uint32_t nbytes;
	uint32_t flags;
};

/* configure a channel */
int soc_xec_dmac_chan_cfg(uint32_t chan, mem_addr_t daddr, mem_addr_t maddr, uint32_t nbytes,
			  uint32_t flags, struct xec_dma_chan_state *ps);

int soc_xec_dmac_chan_cfg2(uint32_t chan, struct soc_xec_dma_chan_cfg *cfg,
			   struct xec_dma_chan_state *ps);

#define XEC_DMAC_START_IEN		BIT(0)

int soc_xec_dmac_chan_start(uint32_t chan, uint32_t flags);

mem_addr_t soc_xec_dmac_chan_base(uint32_t chan);

/* clear a channel */
int soc_xec_dmac_chan_clear(uint32_t chan);

/* clear all channel status and corresponding GIRQ status */
int soc_xec_dmac_status_clear_all(uint32_t chan);

/* Is the central DMA enabled */
bool soc_xec_dmac_is_enabled(void);

/* reset DMA controller and all channels and clear GIRQ enables/status
 * Optionally enable selected DMA channel GIRQs after clearing.
 */
int soc_xec_dmac_init(uint32_t chan_girq_en_mask);

int soc_xec_dmac_status_get(uint32_t chan, uint32_t *status);

int soc_xec_dmac_chan_state_get(uint32_t chan, struct xec_dma_chan_state *ps);

#endif /* _SOC_MICROCHIP_MEC_COMMON_XEC_DMAC_H_ */
