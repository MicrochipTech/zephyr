/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_MICROCHIP_MEC_COMMON_XEC_DMAC_REGS_H
#define ZEPHYR_SOC_MICROCHIP_MEC_COMMON_XEC_DMAC_REGS_H

#include <zephyr/sys/util.h>

#define XEC_DMA_MAIN_REGS_SIZE         0x40U
#define XEC_DMA_CHAN_REGS_SIZE         0x40U
/* offset of channels from base */
#define XEC_DMA_CHANNELS_OFS_FROM_BASE 0x40u

#define XEC_DMA_CHAN_BASE(dma_base, chan)                                                          \
	((uintptr_t)(dma_base) + XEC_DMA_CHANNELS_OFS_FROM_BASE +                                  \
	 ((uintptr_t)(chan) * XEC_DMA_CHAN_REGS_SIZE))

#if defined(CONFIG_SOC_SERIES_MEC175X)
#define XEC_DMAC_MAX_CHANNELS  20
#define XEC_DMAC_CHAN_ALL_MASK 0xfffffu
#elif defined(CONFIG_SOC_SERIES_MEC15XX)
#define XEC_DMAC_MAX_CHANNELS  12
#define XEC_DMAC_CHAN_ALL_MASK 0xfffu
#else
#define XEC_DMAC_MAX_CHANNELS  16
#define XEC_DMAC_CHAN_ALL_MASK 0xffffu
#endif

/* main control */
#define XEC_DMA_MAIN_CR_OFS      0
#define XEC_DMA_MAIN_CR_MSK      GENMASK(1, 0)
#define XEC_DMA_MAIN_CR_EN_POS   0
#define XEC_DMA_MAIN_CR_SRST_POS 1
/* main data packet 32-bit read-only */
#define XEC_DMA_MAIN_DPKT_OFS    4u

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
#define XEC_DMA_CHAN_CR_REQ_POS        1
#define XEC_DMA_CHAN_CR_DONE_POS       2
#define XEC_DMA_CHAN_CR_BUSY_POS       5
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

#endif /* ZEPHYR_SOC_MICROCHIP_MEC_COMMON_XEC_DMAC_REGS_H */
