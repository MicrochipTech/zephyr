/**
 *
 * Copyright (c) 2022 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the Licence at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * \asf_license_stop
 *
 */

#ifndef _MEC_DMA_H
#define _MEC_DMA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef CONFIG_SOC_SERIES_MEC172X
#define MEC_DMA_CHANNELS 16
#else
#define MEC_DMA_CHANNELS 12
#endif

enum mec_dma_chan {
	MEC_DMA_CHAN0 = 0,
	MEC_DMA_CHAN1,
	MEC_DMA_CHAN2,
	MEC_DMA_CHAN3,
	MEC_DMA_CHAN4,
	MEC_DMA_CHAN5,
	MEC_DMA_CHAN6,
	MEC_DMA_CHAN7,
	MEC_DMA_CHAN8,
	MEC_DMA_CHAN9,
	MEC_DMA_CHAN10,
	MEC_DMA_CHAN11,
#ifdef CONFIG_SOC_SERIES_MEC172X
	MEC_DMA_CHAN12,
	MEC_DMA_CHAN13,
	MEC_DMA_CHAN14,
	MEC_DMA_CHAN15,
#endif
	MEC_DMA_CHAN_MAX
};

#define MEC_DMA_WCRC_CHAN_ID		MEC_DMA_CHAN0
#define MEC_DMA_WMEMFILL_CHAN_ID	MEC_DMA_CHAN1

/* HW flow control device numbers */
enum mec_dma_hw_id {
	DMA_SMB0_DEV    = 0u,
	DMA_SMB0_CTR,
	DMA_SMB1_DEV,
	DMA_SMB1_CTR,
	DMA_SMB2_DEV,
	DMA_SMB2_CTR,
	DMA_SMB3_DEV,
	DMA_SMB3_CTR,
	DMA_SMB4_DEV,
	DMA_SMB4_CTR,
	DMA_QMSPI_TX,
	DMA_QMSPI_RX,
#ifdef CONFIG_SOC_SERIES_MEC172X
	DMA_GPSPI_0_TX,
	DMA_GPSPI_0_RX,
	DMA_GPSPI_1_TX,
	DMA_GPSPI_1_RX,
#endif
	DMA_MAX_HW_DEV_ID
};

/* DMA block is composed of a main set of registers and
 * individual channel registers.
 * Main registers start at offset 0.
 * Channels start at offset 0x40 and each channel is 0x40
 * bytes.
 */

struct dma_chan_regs {
	volatile uint32_t ACTV;
	volatile uint32_t MSTART;
	volatile uint32_t MEND;
	volatile uint32_t DSTART;
	volatile uint32_t CTRL;
	volatile uint32_t ISTATUS;
	volatile uint32_t ICTRL;
	volatile uint32_t FSM;		/* read-only */
	volatile uint32_t ALU_EN;	/* ALU registers exist in channels 0 and 1 */
	volatile uint32_t ALU_DATA;
	volatile uint32_t ALU_PSTS;
	volatile uint32_t ALU_FSM;
	uint32_t chan_rsvd[4];
};

struct dma_regs {
	volatile uint32_t DM_CTRL;
	volatile uint32_t DM_DPKT;	/* Read only */
	volatile uint32_t DM_FSM;	/* Read only */
	uint32_t  rsvd1[13];
	struct dma_chan_regs CHAN[MEC_DMA_CHANNELS];
};


/* DMA Main Activate register */
#define MEC_DMAM_ACTV_REG_MSK	0x03u
#define MEC_DMAM_ACTV_POS	0
#define MEC_DMAM_SRST_POS	1
#define MEC_DMAM_ACTV		BIT(MEC_DMAM_ACTV_POS)
#define MEC_DMAM_SRST		BIT(MEC_DMAM_SRST_POS)

/* DMA channel Activate register */
#define MEC_DMA_ACTV_REG_MSK	0x01u
#define MEC_DMA_ACTV_POS	0
#define MEC_DMA_ACTV		BIT(MEC_DMA_ACTV_POS)

/* DMA channel Control register */
#define MEC_DMA_CTRL_REG_MSK		0x037fff27
#define MEC_DMA_CTRL_RUN_POS		0
#define MEC_DMA_CTRL_REQ_POS		1
#define MEC_DMA_CTRL_DONE_POS		2
#define MEC_DMA_CTRL_BUSY_POS		5
#define MEC_DMA_CTRL_MEM2DEV_POS	8
#define MEC_DMA_CTRL_HWFLC_DEV_POS	9
#define MEC_DMA_CTRL_INCR_MEM_POS	16
#define MEC_DMA_CTRL_INCR_DEV_POS	17
#define MEC_DMA_CTRL_LOCK_ARB_POS	18
#define MEC_DMA_CTRL_DIS_HWFLC_POS	19
#define MEC_DMA_CTRL_UNITS_POS		20
#define MEC_DMA_CTRL_SWFLC_GO_POS	24
#define MEC_DMA_CTRL_ABORT_POS		25

/* Start channel in HW flow control mode */
#define MEC_DMA_CTRL_HWFLC_RUN		BIT(MEC_DMA_CTRL_RUN_POS)

/* HW flow device requests channel (RO) */
#define MEC_DMA_CTRL_REQ		BIT(MEC_DMA_CTRL_REQ_POS)

/* Channel is done (RO) */
#define MEC_DMA_CTRL_DONE		BIT(MEC_DMA_CTRL_DONE_POS)

/* Channel is busy (RO) */
#define MEC_DMA_CTRL_BUSY		BIT(MEC_DMA_CTRL_BUSY_POS)

/* Channel transfer direction is read from memory write to device */
#define MEC_DMA_DIR_MEM2DEV		BIT(MEC_DMA_CTRL_MEM2DEV_POS)

/* Channel HW Flow Control Device ID Mask */
#define MEC_DMA_HWFLC_DEV_MSK0		0x7Fu
#define MEC_DMA_HWFLC_DEV_MSK		\
	((MEC_DMA_HWFLC_DEV_MSK0) << MEC_DMA_CTRL_HWFLC_DEV_POS)
#define MEC_DMA_HWFLC_DEV(n)		\
	(((uint32_t)(n) & MEC_DMA_HWFLC_DEV_MSK0) << MEC_DMA_CTRL_HWFLC_DEV_POS)

/* Increment MSTART address by unit size as each unit is transferred */
#define MEC_DMA_CTRL_INCR_MEM		BIT(MEC_DMA_CTRL_INCR_MEM_POS)

/* Increment DSTART address by unit size as each unit is transferred */
#define MEC_DMA_CTRL_INCR_DEV		BIT(MEC_DMA_CTRL_INCR_DEV_POS)

/* Lock channel as highest priority in DMA arbiter */
#define MEC_DMA_CTRL_LOCK_ARB		BIT(MEC_DMA_CTRL_LOCK_ARB_POS)

/* DMA channels transfer chunk size on AHB: 1, 2, or 4 bytes */
#define MEC_DMA_CTRL_UNITS_MSK0		0x07u
#define MEC_DMA_CTRL_UNITS_MSK		\
	((MEC_DMA_CTRL_UNITS_MSK0) << MEC_DMA_CTRL_UNITS_POS)
#define MEC_DMA_CTRL_UNITS_1BY		(1u << MEC_DMA_CTRL_UNITS_POS)
#define MEC_DMA_CTRL_UNITS_2BY		(2u << MEC_DMA_CTRL_UNITS_POS)
#define MEC_DMA_CTRL_UNITS_4BY		(4u << MEC_DMA_CTRL_UNITS_POS)
#define MEC_DMA_CTRL_UNITS(n)		\
	(((uint32_t)(n) & (MEC_DMA_CTRL_UNITS_MSK0)) << MEC_DMA_CTRL_UNITS_POS)

/* Start channel in SW flow control mode */
#define MEC_DMA_CTRL_SWFLC_GO		BIT(MEC_DMA_CTRL_SWFLC_GO_POS)

/* Abort/stop channel's current transfer on the next unit boundary */
#define MEC_DMA_CTRL_ABORT		BIT(MEC_DMA_CTRL_ABORT_POS)

/* DMA channel Interrupt Status register */
/* DMA channel Interrupt Control register */
#define MEC_DMA_ISC_REG_MSK		0x07u
#define MEC_DMA_ISC_BUS_ERR_POS		0
#define MEC_DMA_ISC_HWFLC_ERR_POS	1
#define MEC_DMA_ISC_DONE_POS		2

#define MEC_DMA_ISC_BUS_ERR		BIT(MEC_DMA_ISC_BUS_ERR_POS)
#define MEC_DMA_ISC_HWFLC_ERR		BIT(MEC_DMA_ISC_HWFLC_ERR_POS)
#define MEC_DMA_ISC_DONE		BIT(MEC_DMA_ISC_DONE_POS)

/* DMA channels 0 and 1 ALU Enable register */
#define MEC_DMA_ALU_EN_REG_MSK		0x03u
#define MEC_DMA_ALU_EN_ON_POS		0
#define MEC_DMA_ALU_EN_PXFR_POS		1
/* enable channel 0(CRC32) or channel 1(mem fill) ALU functionality */
#define MEC_DMA_ALU_EN_ON		BIT(MEC_DMA_ALU_EN_ON_POS)
/* transfer ALU output to destination when calculation done */
#define MEC_DMA_ALU_EN_PXFR		BIT(MEC_DMA_ALU_EN_PXFR_POS)

/* DMA channels 0 and 1 ALU Post Status register (RO) */
#define MEC_DMA_ALU_PSTS_REG_MSK		0x0fu
#define MEC_DMA_ALU_PSTS_CALC_DONE_POS		0
#define MEC_DMA_ALU_PSTS_XMIT_STARTED_POS	1
#define MEC_DMA_ALU_PSTS_XMIT_DONE_POS		2
#define MEC_DMA_ALU_PSTS_BUSY_POS		3

#define  MEC_DMA_ALU_PSTS_CALC_DONE		BIT(MEC_DMA_ALU_PSTS_CALC_DONE_POS)
#define  MEC_DMA_ALU_PSTS_XMIT_STARTED		BIT(MEC_DMA_ALU_PSTS_XMIT_STARTED_POS)
#define  MEC_DMA_ALU_PSTS_XMIT_DONE		BIT(MEC_DMA_ALU_PSTS_XMIT_DONE_POS)
#define  MEC_DMA_ALU_PSTS_BUSY			BIT(MEC_DMA_ALU_PSTS_BUSY_POS)

#endif /* #ifndef _MEC_DMA_H */
