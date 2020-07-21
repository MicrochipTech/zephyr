/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_espi_saf

#include <kernel.h>
#include <soc.h>
#include <errno.h>
#include <drivers/espi.h>
#include <drivers/espi_saf.h>
#include <logging/log.h>
#include "espi_saf_mchp_xec_priv.h"
#include "espi_utils.h"
LOG_MODULE_REGISTER(espi_saf, CONFIG_ESPI_LOG_LEVEL);

#define SAF_MAX_FLASH_DEVICES 2U

/*
 * SAF hardware state machine timings
 * poll timeout is in 32KHz clock periods
 * poll interval is in AHB clock(48MHz) units.
 * suspend resume interval is in 32KHz clock periods.
 * consecutive read timeout is in AHB clock periods.
 * suspend check delay is in AHB clock(48MHz) periods.
 */
#define SAF_FLASH_POLL_TIMEOUT 0x28000U
#define SAF_FLASH_POLL_INTERVAL 0U
#define SAF_FLASH_SUS_RSM_INTERVAL 8U
#define SAF_FLASH_CONSEC_READ_TIMEOUT 2U
#define SAF_FLASH_SUS_CHK_DELAY 0U

/* SAF Pre-fetch optimization mode */
#define SAF_PREFETCH_MODE MCHP_SAF_FL_CFG_MISC_PFOE_DFLT

#define SAF_CFG_MISC_PREFETCH_EXPEDITED 0x03U

/* SAF QMSPI programming */

#define SAF_QMSPI_NUM_FLASH_DESCR 6U
#define SAF_QMSPI_CS0_START_DESCR 0U
#define SAF_QMSPI_CS1_START_DESCR \
	(SAF_QMSPI_CS0_START_DESCR + SAF_QMSPI_NUM_FLASH_DESCR)

/* QMSPI descriptors 12-15 for all SPI flash devices */
/* #define SAF_QMSPI_DESCR12 0x0002D40E */
#define SAF_QMSPI_DESCR12 (MCHP_QMSPI_C_IFM_4X | MCHP_QMSPI_C_TX_ONES | \
			   MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS | \
			   MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_NO_CLOSE | \
			   MCHP_QMSPI_C_XFR_UNITS_1 | \
			   MCHP_QMSPI_C_NEXT_DESCR(13) | \
			   MCHP_QMSPI_C_XFR_NUNITS(1))

/* #define SAF_QMSPI_DESCR13 0x00130642 */
#define SAF_QMSPI_DESCR13 (MCHP_QMSPI_C_IFM_4X | MCHP_QMSPI_C_TX_DIS | \
			   MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_EN | \
			   MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_CLOSE | \
			   MCHP_QMSPI_C_XFR_UNITS_1 | \
			   MCHP_QMSPI_C_NEXT_DESCR(0) | \
			   MCHP_QMSPI_C_XFR_NUNITS(9) | \
			   MCHP_QMSPI_C_DESCR_LAST)

/* #define SAF_QMSPI_DESCR14 0x0002F404 */
#define SAF_QMSPI_DESCR14 (MCHP_QMSPI_C_IFM_1X | MCHP_QMSPI_C_TX_DATA | \
			   MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS | \
			   MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_NO_CLOSE | \
			   MCHP_QMSPI_C_XFR_UNITS_1 | \
			   MCHP_QMSPI_C_NEXT_DESCR(15) | \
			   MCHP_QMSPI_C_XFR_NUNITS(1))

/* #define SAF_QMSPI_DESCR15 0x00050640 */
#define SAF_QMSPI_DESCR15 (MCHP_QMSPI_C_IFM_1X | MCHP_QMSPI_C_TX_DIS | \
			   MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_EN | \
			   MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_CLOSE | \
			   MCHP_QMSPI_C_XFR_UNITS_1 | \
			   MCHP_QMSPI_C_NEXT_DESCR(0) | \
			   MCHP_QMSPI_C_XFR_NUNITS(2) | \
			   MCHP_QMSPI_C_DESCR_LAST)

/*
 * SAF Opcode 32-bit register value.
 * Each byte contain a SPI flash 8-bit opcode.
 * NOTE1: opcode value of 0 = flash does not support this operation
 * NOTE2:
 * SAF Opcode A
 * 	op0 = SPI flash write-enable opcode
 * 	op1 = SPI flash program/erase suspend opcode
 * 	op2 = SPI flash program/erase resume opcode
 * 	op3 = SPI flash read STATUS1 opcode
 * SAF Opcode B
 *	op0 = SPI flash erase 4KB sector opcode
 *	op1 = SPI flash erase 32KB sector opcode
 *	op2 = SPI flash erase 64KB sector opcode
 *	op3 = SPI flash page program opcode
 * SAF Opcode C
 *	op0 = SPI flash read 1-4-4 continuous mode opcode
 *	op1 = SPI flash op0 mode byte value for non-continuous mode
 *	op2 = SPI flash op0 mode byte value for continuous mode
 *	op3 = SPI flash read STATUS2 opcode
 */
#define SAF_OPCODE_REG_VAL(op0,op1,op2,op3) \
	(((uint32_t)(op0)&0xffU) | (((uint32_t)(op1)&0xffU) << 8) | \
	 (((uint32_t)(op2)&0xffU) << 16) | (((uint32_t)(op3)&0xffU) << 24))

/*
 * SAF Flash Config CS0/CS1 QMSPI descriptor indices register value
 * e = First QMSPI descriptor index for enter continuous mode chain
 * r = First QMSPI descriptor index for continuous mode read chain
 * s = Index of QMSPI descriptor in continuous mode read chain that
 *     contains the data length field.
 */
#define SAF_CS_CFG_DESCR_IDX_REG_VAL(e,r,s) (((uint32_t)(e)&0xfU) | \
	(((uint32_t)(r)&0xfU) << 8) | (((uint32_t)(s)&0xfU) << 12))

/* Flags */
#define FLASH_FLAG_ADDR32 BIT(0)

/* SPI flash device connected to QMSPI chip select 0 */
#define SAF_FLASH_CS0_SIZE (16U * 1024U * 1024U)

/*
 * Six QMSPI descriptors describe SPI flash opcode protocols.
 * W25Q128 device.
 */
/* #define CS0_DESCR0 0x00081406U */
#define CS0_DESCR0 (MCHP_QMSPI_C_IFM_4X | MCHP_QMSPI_C_TX_DATA | \
		    MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS | \
		    MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_NO_CLOSE | \
		    MCHP_QMSPI_C_XFR_UNITS_1 | MCHP_QMSPI_C_NEXT_DESCR(1) | \
		    MCHP_QMSPI_C_XFR_NUNITS(4))

/* #define CS0_DESCR1 0x00042402U */
#define CS0_DESCR1 (MCHP_QMSPI_C_IFM_4X | MCHP_QMSPI_C_TX_DIS | \
		    MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS | \
		    MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_NO_CLOSE | \
		    MCHP_QMSPI_C_XFR_UNITS_1  | MCHP_QMSPI_C_NEXT_DESCR(2) | \
		    MCHP_QMSPI_C_XFR_NUNITS(2))

/* #define CS0_DESCR2 0x000137C2U */
#define CS0_DESCR2 (MCHP_QMSPI_C_IFM_4X | MCHP_QMSPI_C_TX_DIS | \
		    MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_EN | \
		    MCHP_QMSPI_C_RX_DMA_4B | MCHP_QMSPI_C_CLOSE | \
		    MCHP_QMSPI_C_XFR_UNITS_1 | MCHP_QMSPI_C_NEXT_DESCR(3) | \
		    MCHP_QMSPI_C_DESCR_LAST | MCHP_QMSPI_C_XFR_NUNITS(0))

/* #define CS0_DESCR3 0x00024404U */
#define CS0_DESCR3 (MCHP_QMSPI_C_IFM_1X | MCHP_QMSPI_C_TX_DATA | \
		    MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS | \
		    MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_NO_CLOSE | \
		    MCHP_QMSPI_C_XFR_UNITS_1 | MCHP_QMSPI_C_NEXT_DESCR(4) | \
		    MCHP_QMSPI_C_XFR_NUNITS(1))

/* #define CS0_DESCR4 0x00085406U */
#define CS0_DESCR4 (MCHP_QMSPI_C_IFM_4X | MCHP_QMSPI_C_TX_DATA | \
		    MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS | \
		    MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_NO_CLOSE | \
		    MCHP_QMSPI_C_XFR_UNITS_1 | MCHP_QMSPI_C_NEXT_DESCR(5) | \
		    MCHP_QMSPI_C_XFR_NUNITS(4))

/* #define CS0_DESCR5 0x00076602U */
#define CS0_DESCR5 (MCHP_QMSPI_C_IFM_4X | MCHP_QMSPI_C_TX_DIS | \
		    MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS | \
		    MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_CLOSE | \
		    MCHP_QMSPI_C_XFR_UNITS_1 | MCHP_QMSPI_C_NEXT_DESCR(6) | \
		    MCHP_QMSPI_C_DESCR_LAST | MCHP_QMSPI_C_XFR_NUNITS(3))

#define CS0_OPA SAF_OPCODE_REG_VAL(0x06U,0x75U,0x7aU,0x05U)
#define CS0_OPB SAF_OPCODE_REG_VAL(0x20U,0x52U,0xd8U,0x02U)
#define CS0_OPC SAF_OPCODE_REG_VAL(0xebU,0xffU,0xa5U,0x35U)

#define CS0_POLL2_MASK 0xff7fU

/*
 * SAF Flash Continous Mode Prefix register value
 * b[7:0] = continuous mode prefix opcode
 * b[15:8] = continuous mode prefix opcode data
 * Some SPI flash devices require a prefix command before
 * they will enter continuous mode.
 * A zero value means the SPI flash does not require a prefix
 * command.
 */
#define CS0_CONT_MODE_PREFIX_VAL 0U

/* SAF Flash Config CS0 QMSPI descriptor indices */
#define CS0_CFG_DESCR_IDX_REG_VAL SAF_CS_CFG_DESCR_IDX_REG_VAL(3U,0U,2U)

#define CS0_FLAGS 0U

/* SPI flash device connected to QMSPI chip select 1 */
#define SAF_FLASH_CS1_SIZE 0U

/*
 * Six QMSPI descriptors describe SPI flash opcode protocols.
 * W25Q128 device.
 */
/* #define CS1_DESCR6 0x00087406U */
#define CS1_DESCR6 (MCHP_QMSPI_C_IFM_4X | MCHP_QMSPI_C_TX_DATA | \
		    MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS | \
		    MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_NO_CLOSE | \
		    MCHP_QMSPI_C_XFR_UNITS_1 | MCHP_QMSPI_C_NEXT_DESCR(7) | \
		    MCHP_QMSPI_C_XFR_NUNITS(4))

/* #define CS1_DESCR7 0x00048402U */
#define CS1_DESCR7 (MCHP_QMSPI_C_IFM_4X | MCHP_QMSPI_C_TX_DIS | \
		    MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS | \
		    MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_NO_CLOSE | \
		    MCHP_QMSPI_C_XFR_UNITS_1 | MCHP_QMSPI_C_NEXT_DESCR(8) | \
		    MCHP_QMSPI_C_XFR_NUNITS(2))

/* #define CS1_DESCR8 0x000197C2U */
#define CS1_DESCR8 (MCHP_QMSPI_C_IFM_4X | MCHP_QMSPI_C_TX_DIS | \
		    MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_EN | \
		    MCHP_QMSPI_C_RX_DMA_4B | MCHP_QMSPI_C_CLOSE | \
		    MCHP_QMSPI_C_XFR_UNITS_1 | MCHP_QMSPI_C_NEXT_DESCR(9) | \
		    MCHP_QMSPI_C_XFR_NUNITS(0) | MCHP_QMSPI_C_DESCR_LAST)

/* #define CS1_DESCR9 0x0002a404U */
#define CS1_DESCR9 (MCHP_QMSPI_C_IFM_1X | MCHP_QMSPI_C_TX_DATA | \
		    MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS | \
		    MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_NO_CLOSE | \
		    MCHP_QMSPI_C_XFR_UNITS_1 | \
		    MCHP_QMSPI_C_NEXT_DESCR(10) | \
		    MCHP_QMSPI_C_XFR_NUNITS(1))

/* #define CS1_DESCR10 0x0008b406U */
#define CS1_DESCR10 (MCHP_QMSPI_C_IFM_4X | MCHP_QMSPI_C_TX_DATA | \
		     MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS | \
		     MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_NO_CLOSE | \
		     MCHP_QMSPI_C_XFR_UNITS_1 | \
		     MCHP_QMSPI_C_NEXT_DESCR(11) | \
		     MCHP_QMSPI_C_XFR_NUNITS(4)) \

/* #define CS1_DESCR11 0x0007c602U */
#define CS1_DESCR11 (MCHP_QMSPI_C_IFM_4X | MCHP_QMSPI_C_TX_DIS | \
		     MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS | \
		     MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_CLOSE | \
		     MCHP_QMSPI_C_XFR_UNITS_1 | \
		     MCHP_QMSPI_C_NEXT_DESCR(12) | \
		     MCHP_QMSPI_C_XFR_NUNITS(3) | MCHP_QMSPI_C_DESCR_LAST)

#define CS1_OPA SAF_OPCODE_REG_VAL(0x06U,0x75U,0x7aU,0x05U)
#define CS1_OPB SAF_OPCODE_REG_VAL(0x20U,0x52U,0xd8U,0x02U)
#define CS1_OPC SAF_OPCODE_REG_VAL(0xebU,0xffU,0xa5U,0x35U)

#define CS1_POLL2_MASK 0xff7fU

#define CS1_CONT_MODE_PREFIX_VAL 0U

/* SAF Flash Config CS1 QMSPI descriptor indices */
#define CS1_CFG_DESCR_IDX_REG_VAL SAF_CS_CFG_DESCR_IDX_REG_VAL(9U,6U,8U)

#define CS1_FLAGS 0U


/* SAF EC Portal read/write flash access limited to 1-64 bytes */
#define MAX_SAF_ECP_BUFFER_SIZE 64ul

/* 1 second maximum for flash operations */
#define MAX_SAF_FLASH_TIMEOUT 1000ul


struct espi_isr {
	uint32_t girq_bit;
	void (*the_isr)(struct device *dev);
};

/*
 * SAF configuration from Device Tree
 * SAF controller register block base address
 * QMSPI controller register block base address
 * SAF communications register block base address
 * SAF Protection regions 0-7 mapping to eSPI master ID's.
 * SAF Protection regions 8-15 mapping to eSPI master ID's.
 * SAF Protection regions 16 mapping to eSPI masterID's.
 */
struct espi_saf_xec_config {
	uintptr_t saf_base_addr;
	uintptr_t qmspi_base_addr;
	uintptr_t saf_comm_base_addr;
	uint32_t prmap0;
	uint32_t prmap1;
	uint32_t prmap2;
};

struct espi_saf_xec_data {
	sys_slist_t callbacks;
	struct k_sem ecp_lock;
};

/*
 * SAF local flash configuration.
 * SPI flash device size in bytes
 * SPI opcodes for SAF Opcode A register
 * SPI opcodes for SAF Opcode B register
 * SPI opcodes for SAF Opcode C register
 * QMSPI descriptors describing SPI opcode transmit and
 * data read.
 * SAF controller Poll2 Mast value specific for this flash device
 * SAF continuous mode prefix register value for those flashes requireing
 * a prefix byte transmitted before the enter continuous mode command.
 * Start QMSPI descriptor numbers.
 * miscellaneous flags.
 */
struct saf_spi_flash_cfg
{
	uint32_t flashsz;
	uint32_t opa;
	uint32_t opb;
	uint32_t opc;
	uint32_t descr[SAF_QMSPI_NUM_FLASH_DESCR];
	uint16_t poll2_mask;
	uint16_t cont_prefix;
	uint16_t cs_cfg_descr_ids;
	uint16_t flags;
};

/* convenience defines */
#define DEV_CFG(dev)							\
	((const struct espi_saf_xec_config * const)			\
		(dev)->config_info)
#define DEV_DATA(dev)							\
	((struct espi_saf_xec_data * const)(dev)->driver_data)

/* EC portal local flash r/w buffer */
static uint32_t slave_mem[MAX_SAF_ECP_BUFFER_SIZE];

/*
 * @brief eSPI SAF configuration
 */

const struct saf_spi_flash_cfg flash_dev_cfg[] = {
	{ /* Winbond 25Q128 connected to SPI CS0 */
		.flashsz = SAF_FLASH_CS0_SIZE,
		.opa = CS0_OPA,
		.opb = CS0_OPB,
		.opc = CS0_OPC,
		.descr = {
			CS0_DESCR0, CS0_DESCR1, CS0_DESCR2,
			CS0_DESCR3, CS0_DESCR4, CS0_DESCR5
		},
		.poll2_mask = CS0_POLL2_MASK,
		.cont_prefix = CS0_CONT_MODE_PREFIX_VAL,
		.cs_cfg_descr_ids = CS0_CFG_DESCR_IDX_REG_VAL,
		.flags = CS0_FLAGS,
	},
/* #if SAF_FLASH_CS1_SIZE != 0 */
	{ /* Winbond 25Q128 connected to SPI CS1 */
		.flashsz = SAF_FLASH_CS1_SIZE,
		.opa = CS1_OPA,
		.opb = CS1_OPB,
		.opc = CS1_OPC,
		.descr = {
			CS1_DESCR6, CS1_DESCR7, CS1_DESCR8,
			CS1_DESCR9, CS1_DESCR10, CS1_DESCR11
		},
		.poll2_mask = CS1_POLL2_MASK,
		.cont_prefix = CS1_CONT_MODE_PREFIX_VAL,
		.cs_cfg_descr_ids = CS1_CFG_DESCR_IDX_REG_VAL,
		.flags = CS1_FLAGS,
	},
/* #endif */
};

static uint8_t saf_flash_cnt =
	sizeof(flash_dev_cfg) / sizeof(struct saf_spi_flash_cfg);



static inline void mchp_saf_cs_descr_wr(MCHP_SAF_HW_REGS *regs,
					uint8_t cs, uint32_t val)
{
	regs->SAF_CS_OP[cs].OP_DESCR = val;
}

static inline void mchp_saf_poll2_mask_wr(MCHP_SAF_HW_REGS *regs,
					  uint8_t cs, uint16_t val)
{
	LOG_DBG("%s cs: %d mask %x", __func__, cs, val);
	if (cs == 0) {
		regs->SAF_CS0_CFG_P2M = val;
	} else {
		regs->SAF_CS1_CFG_P2M = val;
	}
}

static inline void mchp_saf_cm_prefix_wr(MCHP_SAF_HW_REGS *regs,
					 uint8_t cs, uint16_t val)
{
	if (cs == 0) {
		regs->SAF_CS0_CM_PRF = val;
	} else {
		regs->SAF_CS1_CM_PRF = val;
	}
}

/*
 * MCHP SAF implements 17 flash protection regions.
 * We program at least one protection region access bit map allowing
 * all eSPI masters full access to the whole flash map.
 * PR's can overlap and the most restrictive access bit map applies.
 * Start and limit are in units of 4KB.
 */
const struct mchp_espi_saf_pr prot_regs[] = {
	{
		.START = 0,
		.LIMIT = (((SAF_FLASH_CS0_SIZE + \
			    SAF_FLASH_CS1_SIZE) / 4096) - 1U),
		.WEBM = MCHP_SAF_MSTR_ALL,
		.RDBM = MCHP_SAF_MSTR_ALL
	},
};

static uint8_t saf_pr_cnt =
	sizeof(prot_regs) / sizeof(struct mchp_espi_saf_pr);

/*
 * Intialize SAF flash protection regions.
 * SAF HW implements 16 protection regions.
 * At least one protection region must be enable to allow
 * EC access to the local flash through the EC Portal.
 * Each protection region is composed of 4 32-bit registers
 * Start bits[19:0] = start address of region in units of 4KB
 * Limit bits[19:0] = limit address of region in units of 4KB
 * Write prot b[7:0] = masters[7:0] allow write/erase. 1=allowed
 * Read prot b[7:0] = masters[7:0] allow read. 1=allowed
 *
 * POR default values:
 * Start = 0x7ffff
 * Limit = 0
 * Write Prot = 0x01 Master 0 always granted write/erase
 * Read Prot = 0x01 Master 0 always granted read
 *
 * Sample code configures PR[0]
 * Start = 0
 * Limit = 0x7ffff
 * WR = 0xFF
 * RD = 0xFF
 */
static void saf_protection_regions_init(MCHP_SAF_HW_REGS *regs)
{
	LOG_DBG("%s", __func__);
	for (size_t n = 0; n < (size_t)saf_pr_cnt; n++) {
		regs->SAF_PROT_RG[n].START = prot_regs[n].START;
		regs->SAF_PROT_RG[n].LIMIT = prot_regs[n].LIMIT;
		regs->SAF_PROT_RG[n].WEBM = prot_regs[n].WEBM;
		regs->SAF_PROT_RG[n].RDBM = prot_regs[n].RDBM;

		LOG_DBG("PROT[%d] START %x", n, regs->SAF_PROT_RG[n].START);
		LOG_DBG("PROT[%d] LIMIT %x", n, regs->SAF_PROT_RG[n].LIMIT);
		LOG_DBG("PROT[%d] WEBM %x", n, regs->SAF_PROT_RG[n].WEBM);
		LOG_DBG("PROT[%d] RDBM %x", n, regs->SAF_PROT_RG[n].RDBM);
	}
}

/*
 * Take over and re-initialize QMSPI for use by SAF HW engine.
 * Descriptors 0-11 value are specific to the properties of
 * the SPI flash devices. Descriptors 12-15 contain fixed values.
 * When SAF is enabled, QMSPI registers are controlled by SAF
 * HW engine. CPU no longer has access to QMSPI registers.
 */
static void saf_qmspi_init(const struct espi_saf_xec_config *cfg)
{
	uint32_t qmode;
	QMSPI_Type *regs = (QMSPI_Type *)cfg->qmspi_base_addr;

	LOG_DBG("%s QMSPI mode: %x", __func__, regs->MODE);
	regs->MODE = MCHP_QMSPI_M_SRST;
	regs->MODE = 0;

	MCHP_GIRQ_ENCLR(MCHP_QMSPI_GIRQ_NUM) = MCHP_QMSPI_GIRQ_VAL;
	MCHP_GIRQ_SRC(MCHP_QMSPI_GIRQ_NUM) = MCHP_QMSPI_GIRQ_VAL;

	LOG_DBG("QMSPI IFCTRL befo %x", regs->IFCTRL);
	regs->IFCTRL = (MCHP_QMSPI_IFC_WP_OUT_HI
			| MCHP_QMSPI_IFC_WP_OUT_EN
			| MCHP_QMSPI_IFC_HOLD_OUT_HI
			| MCHP_QMSPI_IFC_HOLD_OUT_EN);
	LOG_DBG("QMSPI IFCTRL after %x", regs->IFCTRL);

	regs->CSTM = SAF_QMSPI_CS_TIMING;
	LOG_DBG("CSTM %x", regs->CSTM);

	regs->DESCR[12] = SAF_QMSPI_DESCR12;
	regs->DESCR[13] = SAF_QMSPI_DESCR13;
	regs->DESCR[14] = SAF_QMSPI_DESCR14;
	regs->DESCR[15] = SAF_QMSPI_DESCR15;

	LOG_DBG("QMSPI DESC 12 %x", regs->DESCR[12]);
	LOG_DBG("QMSPI DESC 13 %x", regs->DESCR[13]);
	LOG_DBG("QMSPI DESC 14 %x", regs->DESCR[14]);
	LOG_DBG("QMSPI DESC 15 %x", regs->DESCR[15]);

	regs->IEN = MCHP_QMSPI_IEN_XFR_DONE;

	LOG_DBG("%s Before MODE reg: %x", __func__, regs->MODE);
	qmode = ((uint32_t)(SAF_QMSPI_CLK_DIV) << MCHP_QMSPI_M_FDIV_POS)
		& MCHP_QMSPI_M_FDIV_MASK;
	LOG_DBG("%s qmode val mode: %x", __func__, qmode);

	qmode |= (MCHP_QMSPI_M_SAF_DMA_MODE_EN | MCHP_QMSPI_M_SIG_MODE0_VAL
		  | MCHP_QMSPI_M_CS0 | MCHP_QMSPI_M_ACTIVATE);
	LOG_DBG("%s qmode val mode: %x", __func__, qmode);

	regs->MODE = qmode;
}

/*
 * Registers at offsets:
 * SAF Poll timeout @ 0x194.  Hard coded to 0x28000. Default value = 0.
 *	recommended value = 0x28000 32KHz clocks (5 seconds). b[17:0]
 * SAF Poll interval @ 0x198.  Hard coded to 0
 *	Default value = 0. Recommended = 0. b[15:0]
 * SAF Suspend/Resume Interval @ 0x19c.  Hard coded to 0x8
 *	Default value = 0x01. Min time erase/prog in 32KHz units. Flash specfic
 * SAF Consecutive Read Timeout @ 0x1a0. Hard coded to 0x2. b[15:0]
 *	Units of MCLK. Recommend < 20us. b[19:0]
 * SAF Suspend Check Delay @ 0x1ac. Not touched.
 *	Default = 0. Recommend = 20us. Units = MCLK. b[19:0]
 */
static void saf_flash_timing_init(MCHP_SAF_HW_REGS *regs)
{
	LOG_DBG("%s\n", __func__);
	regs->SAF_POLL_TMOUT = SAF_FLASH_POLL_TIMEOUT;
	regs->SAF_POLL_INTRVL = SAF_FLASH_POLL_INTERVAL;
	regs->SAF_SUS_RSM_INTRVL = SAF_FLASH_SUS_RSM_INTERVAL;
	regs->SAF_CONSEC_RD_TMOUT = SAF_FLASH_CONSEC_READ_TIMEOUT;
	regs->SAF_SUS_CHK_DLY = SAF_FLASH_SUS_CHK_DELAY;
	LOG_DBG("SAF_POLL_TMOUT %x\n", regs->SAF_POLL_TMOUT);
	LOG_DBG("SAF_POLL_INTRVL %x\n", regs->SAF_POLL_INTRVL);
	LOG_DBG("SAF_SUS_RSM_INTRVL %x\n", regs->SAF_SUS_RSM_INTRVL);
	LOG_DBG("SAF_CONSEC_RD_TMOUT %x\n", regs->SAF_CONSEC_RD_TMOUT);
	LOG_DBG("SAF_SUS_CHK_DLY %x\n", regs->SAF_SUS_CHK_DLY);
}

/*
 * Disable DnX bypass feature.
 */
static void saf_dnx_bypass_init(MCHP_SAF_HW_REGS *regs)
{
	regs->SAF_DNX_PROT_BYP = 0;
	regs->SAF_DNX_PROT_BYP = 0xffffffff;
}

/*
 * Bitmap of flash erase size from 1KB up to 128KB.
 * eSPI SAF specification requires 4KB erase support.
 * MCHP SAF supports 4KB, 32KB, and 64KB.
 * Only report 32KB and 64KB to Host if supported by both
 * all flash devices.
 */
static int saf_init_erase_block_size(const struct espi_saf_xec_config *cfg)
{
	uint32_t opb = flash_dev_cfg[0].opb;

	LOG_DBG("%s\n", __func__);
#if SAF_FLASH_CS1_SIZE != 0
	opb &= flash_dev_cfg[1].opb;
#endif

	if ((opb & MCHP_SAF_CS_OPB_ER0_MASK) == 0) {
		/* One or both do not support 4KB erase! */
		return -EINVAL;
	}

	uint8_t erase_bitmap = MCHP_ESPI_SERASE_SZ_4K;

	if (opb & MCHP_SAF_CS_OPB_ER1_MASK) {
		erase_bitmap |= MCHP_ESPI_SERASE_SZ_32K;
	}

	if (opb & MCHP_SAF_CS_OPB_ER2_MASK) {
		erase_bitmap |= MCHP_ESPI_SERASE_SZ_64K;
	}

	/*
	 * TODO how to pass address of eSPI capabilities registers
	 * to SAF driver?
	 */
	ESPI_CAP_REGS->FC_SERBZ = erase_bitmap;

	return 0;
}

/*
 * Set the continuous mode prefix and 4-byte address mode bits
 * based upon the flash configuration information.
 * Updates:
 * SAF Flash Config Poll2 Mask @ 0x1A4
 * SAF Flash Config Special Mode @ 0x1B0
 * SAF Flash Misc Config @ 0x38
 */
static void saf_flash_misc_cfg(MCHP_SAF_HW_REGS *regs, uint8_t cs,
			       const struct saf_spi_flash_cfg *pf)
{
	uint32_t d, v;

	LOG_DBG("%s", __func__);
	LOG_DBG("cs %d", cs);
	LOG_DBG("pf->cont_prefix %d", pf->cont_prefix);
	LOG_DBG("pf->flags %d", pf->flags);

	d = regs->SAF_FL_CFG_MISC;

	v = MCHP_SAF_FL_CFG_MISC_CS0_CPE;
	if (cs) {
		v = MCHP_SAF_FL_CFG_MISC_CS1_CPE;
	}

	/* Does this flash device require a prefix for continuous mode? */
	if (pf->cont_prefix != 0) {
		d |= v;
	} else {
		d &= ~v;
	}

	v = MCHP_SAF_FL_CFG_MISC_CS0_4BM;
	if (cs) {
		v = MCHP_SAF_FL_CFG_MISC_CS1_4BM;
	}

	/* Use 32-bit addressing for this flash device? */
	if (pf->flags & FLASH_FLAG_ADDR32) {
		d |= v;
	} else {
		d &= ~v;
	}

	LOG_DBG("%s Bef SAF_FL_CFG_MISC: %x", __func__, regs->SAF_FL_CFG_MISC);
	regs->SAF_FL_CFG_MISC = d;
	LOG_DBG("%s Aft SAF_FL_CFG_MISC: %x", __func__, regs->SAF_FL_CFG_MISC);
}

/*
 * Program flash device specific SAF and QMSPI registers.
 *
 * CS0 OpA @ 0x4c or CS1 OpA @ 0x5C
 * CS0 OpB @ 0x50 or CS1 OpB @ 0x60
 * CS0 Opc @ 0x54 or CS1 OpC @ 0x64
 * Poll 2 Mask @ 0x1a4
 * Continuous Prefix @ 0x1b0
 * CS0: QMSPI descriptors 0-5 or CS1 QMSPI descriptors 6-11
 * CS0 Descrs @ 0x58 or CS1 Descrs @ 0x68
 */
uint32_t saf_flash_cfg(struct device *dev, struct espi_saf_cfg *cfg,
		       uint8_t cs)
{
	uint32_t did;
	const struct saf_spi_flash_cfg *pf;
	const struct espi_saf_xec_config *xcfg = DEV_CFG(dev);
	MCHP_SAF_HW_REGS *regs = (MCHP_SAF_HW_REGS *)xcfg->saf_base_addr;
	QMSPI_Type *qregs = (QMSPI_Type *)xcfg->qmspi_base_addr;

	LOG_DBG("%s", __func__);
	cs = (cs < SAF_MAX_FLASH_DEVICES) ? cs : SAF_MAX_FLASH_DEVICES-1;
	pf = &flash_dev_cfg[cs];

	mchp_saf_poll2_mask_wr(regs, cs, pf->poll2_mask);
	mchp_saf_cm_prefix_wr(regs, cs, pf->cont_prefix);
	saf_flash_misc_cfg(regs, cs, pf);

	regs->SAF_CS_OP[cs].OPA = pf->opa;
	regs->SAF_CS_OP[cs].OPB = pf->opb;
	regs->SAF_CS_OP[cs].OPC = pf->opc;
	regs->SAF_CS_OP[cs].OP_DESCR = (uint32_t)pf->cs_cfg_descr_ids;

	LOG_DBG("OPA WRITE %p %x", &regs->SAF_CS_OP[cs].OPA,
		regs->SAF_CS_OP[cs].OPA);
	LOG_DBG("OPB ERASE %p %x", &regs->SAF_CS_OP[cs].OPB,
		regs->SAF_CS_OP[cs].OPB);
	LOG_DBG("OPC READ  %p %x", &regs->SAF_CS_OP[cs].OPC,
		regs->SAF_CS_OP[cs].OPC);
	did = SAF_QMSPI_CS0_START_DESCR;
	if (cs != 0) {
		did = SAF_QMSPI_CS1_START_DESCR;
	}

	LOG_DBG("CS start address cs: %d did: %d", cs, did);
	for (size_t i = 0; i < SAF_QMSPI_NUM_FLASH_DESCR; i++) {
		qregs->DESCR[did++] = pf->descr[i];
	LOG_DBG("CFG_CSX_DESCR[%d]=%x", cs, regs->SAF_CS_OP[cs].OP_DESCR);
	LOG_DBG("%s flashsz: %x\n", __func__, pf->flashsz);
	}

	return pf->flashsz;
}

static void saf_tagmap_init(MCHP_SAF_HW_REGS *regs,
			    const struct espi_saf_xec_config *xcfg)
{
	regs->SAF_TAG_MAP[0] = xcfg->prmap0;
	regs->SAF_TAG_MAP[1] = xcfg->prmap1;
	regs->SAF_TAG_MAP[2] = xcfg->prmap2;

	LOG_DBG("SAF TAG0 %x", regs->SAF_TAG_MAP[0]);
	LOG_DBG("SAF TAG1 %x", regs->SAF_TAG_MAP[1]);
	LOG_DBG("SAF TAG2 %x", regs->SAF_TAG_MAP[2]);
}

/*
 * Configure SAF and QMSPI for SAF operation based upon the
 * number and characteristics of local SPI flash devices.
 * NOTE: SAF is configured but not activated. SAF should be
 * activated only when eSPI master sends Flash Channel enable
 * message with MAF/SAF select flag.
 */
static uint32_t cap0_i;
static uint32_t cap0_f;
static uint32_t capfc_i;
static uint32_t capfc_f;

static int espi_saf_xec_configuration(struct device *dev,
				      struct espi_saf_cfg *cfg)
{
	int ret;
	uint32_t totalsz;
	const struct espi_saf_xec_config *xcfg = DEV_CFG(dev);
	MCHP_SAF_HW_REGS *regs = (MCHP_SAF_HW_REGS *)xcfg->saf_base_addr;

	LOG_DBG("%s", __func__);
	LOG_DBG("Saved value. Before saf_init cap0 %x", cap0_i);
	LOG_DBG("Saved value. After saf_init cap0 %x", cap0_f);
	LOG_DBG("Saved value. Before saf_init capfc %x", capfc_i);
	LOG_DBG("Saved value. After saf_init capfc %x", capfc_f);

	if ((saf_flash_cnt == 0U) ||
	    (saf_flash_cnt > SAF_MAX_FLASH_DEVICES)) {
		return -EINVAL;
	}

	if (saf_pr_cnt > MCHP_ESPI_SAF_PR_MAX) {
		return -EINVAL;
	}

	LOG_DBG("%s Curr SAF_FL_CFG_MISC: %x", __func__, regs->SAF_FL_CFG_MISC);
	if (regs->SAF_FL_CFG_MISC & MCHP_SAF_FL_CFG_MISC_SAF_EN) {
		/* can't configure after SAF is activated! */
		LOG_ERR("%s %x configure after activation\n", __func__,
		       regs->SAF_FL_CFG_MISC);
		return -EAGAIN;
	}

	saf_qmspi_init(xcfg);

	LOG_DBG("SAF_CS0_CFG_P2M before %x", regs->SAF_CS0_CFG_P2M);
	LOG_DBG("SAF_CS1_CFG_P2M before %x", regs->SAF_CS1_CFG_P2M);
	regs->SAF_CS0_CFG_P2M = 0;
	regs->SAF_CS1_CFG_P2M = 0;
	LOG_DBG("SAF_CS0_CFG_P2M after %x", regs->SAF_CS0_CFG_P2M);
	LOG_DBG("SAF_CS1_CFG_P2M after %x", regs->SAF_CS1_CFG_P2M);

	regs->SAF_FL_CFG_GEN_DESCR = MCHP_SAF_FL_CFG_GEN_DESCR_STD;

	/* flash device connected to CS0 required */
	totalsz = saf_flash_cfg(dev, cfg, 0);
	regs->SAF_FL_CFG_THRH = totalsz;

	LOG_DBG("SAF_CS0_CFG_P2M %x", regs->SAF_CS0_CFG_P2M);
	LOG_DBG("SAF_CS1_CFG_P2M %x", regs->SAF_CS1_CFG_P2M);

	/* optional second flash device connected to CS1 */
	totalsz += saf_flash_cfg(dev, cfg, 1);
	if (totalsz == 0) {
		return -EAGAIN;
	}

	regs->SAF_FL_CFG_SIZE_LIM = totalsz - 1;
	LOG_DBG("SAF_FL_CFG_THRH after %x", regs->SAF_FL_CFG_THRH);

	saf_tagmap_init(regs, xcfg);

	saf_protection_regions_init(regs);

	saf_dnx_bypass_init(regs);

	saf_flash_timing_init(regs);

	ret = saf_init_erase_block_size(xcfg);
	if (ret != 0) {
		LOG_ERR("SAF Config bad flash erase config");
		return ret;
	}

	/* Set pre-fetch mode */
	LOG_DBG("%s Bef SAF_FL_CFG_MISC: %x", __func__, regs->SAF_FL_CFG_MISC);
	regs->SAF_FL_CFG_MISC =
		(regs->SAF_FL_CFG_MISC & ~(MCHP_SAF_FL_CFG_MISC_PFOE_MASK))
		 | SAF_PREFETCH_MODE;
	LOG_DBG("%s Aft SAF_FL_CFG_MISC: %x", __func__, regs->SAF_FL_CFG_MISC);

	/* enable prefetch */
	LOG_DBG("%s Bef MCHP_SAF_COMM_MODE_REG: %x", __func__,
		MCHP_SAF_COMM_MODE_REG);
	MCHP_SAF_COMM_MODE_REG |= MCHP_SAF_COMM_MODE_PF_EN;
	LOG_DBG("%s Aft MCHP_SAF_COMM_MODE_REG: %x", __func__,
		MCHP_SAF_COMM_MODE_REG);

	return 0;
}

static bool espi_saf_xec_channel_ready(struct device *dev)
{
	const struct espi_saf_xec_config *cfg = DEV_CFG(dev);
	MCHP_SAF_HW_REGS *regs = (MCHP_SAF_HW_REGS *)cfg->saf_base_addr;

	if (regs->SAF_FL_CFG_MISC & MCHP_SAF_FL_CFG_MISC_SAF_EN) {
		return true;
	}

	return false;
}

/*
 * MCHP SAF hardware supports a range of flash block erase
 * sizes from 1KB to 128KB. The eSPI Host specification requires
 * 4KB must be supported. The MCHP SAF QMSPI HW interface only
 * supported three erase sizes. Most SPI flash devices chosen for
 * SAF support 4KB, 32KB, and 64KB.
 * Get flash erase sizes driver has configured from eSPI capabilities
 * registers. We assume driver flash tables have opcodes to match
 * capabilities configuration.
 * Check requested erase size is supported.
 */
struct erase_size_encoding {
	uint8_t hwbitpos;
	uint8_t encoding;
};

static const struct erase_size_encoding ersz_enc[] = {
	{ MCHP_ESPI_SERASE_SZ_4K_BITPOS, 0 },
	{ MCHP_ESPI_SERASE_SZ_32K_BITPOS, 1 },
	{ MCHP_ESPI_SERASE_SZ_64K_BITPOS, 2 }
};

#define SAF_ERASE_ENCODING_MAX_ENTRY \
	(sizeof(ersz_enc) / sizeof(struct erase_size_encoding))


static uint32_t get_erase_size_encoding(uint32_t erase_size)
{
	uint8_t supsz = ESPI_CAP_REGS->FC_SERBZ;

	LOG_DBG("%s\n", __func__);
	for (int i = 0; i < SAF_ERASE_ENCODING_MAX_ENTRY; i++) {
		uint32_t sz = MCHP_ESPI_SERASE_SZ(ersz_enc[i].hwbitpos);

		if ((sz == erase_size) &&
		    (supsz & (1 << ersz_enc[i].hwbitpos))) {
			return ersz_enc[i].encoding;
		}
	}

	return 0xffffffffU;
}

/*
 * EC access (read/erase/write) to SAF atttached flash array
 * cmd  0 = read
 *	1 = write
 *	2 = erase
 */
static int saf_ecp_access(struct device *dev,
			  struct espi_saf_packet *pckt,
			  uint8_t cmd)
{
	uint32_t err_mask, n;
	const struct espi_saf_xec_config *cfg = DEV_CFG(dev);
	MCHP_SAF_HW_REGS *regs = (MCHP_SAF_HW_REGS *)cfg->saf_base_addr;

	err_mask = MCHP_SAF_ECP_STS_ERR_MASK;

	LOG_DBG("%s", __func__);

	if (!(regs->SAF_FL_CFG_MISC & MCHP_SAF_FL_CFG_MISC_SAF_EN)) {
		LOG_ERR("SAF is disabled");
		return -EIO;
	}

	if (regs->SAF_ECP_BUSY & MCHP_SAF_ECP_BUSY) {
		LOG_ERR("SAF EC Portal is busy");
		return -EBUSY;
	}

	if (cmd >= MCHP_SAF_ECP_CMD_CTYPE_MAX0) {
		LOG_ERR("SAF EC Portal bad cmd");
		return -EAGAIN;
	}

	if (cmd == MCHP_SAF_ECP_CMD_CTYPE_ERASE0) {
		n =  get_erase_size_encoding(pckt->len);
		if (n == 0xffffffff) {
			LOG_ERR("SAF EC Portal unsupported erase size");
			return -EAGAIN;
		}
	} else {
		if ((pckt->len < MCHP_SAF_ECP_CMD_RW_LEN_MIN) ||
		    (pckt->len > MCHP_SAF_ECP_CMD_RW_LEN_MAX)) {
			LOG_ERR("SAF EC Portal size out of bounds");
			return -EAGAIN;
		}
		n = pckt->len;
	}

	LOG_DBG("%s params val done", __func__);
	regs->SAF_ECP_INTEN = 0;
	regs->SAF_ECP_STATUS = 0xffffffff;
	/* TODO
	 * Force SAF Done interrupt disabled until we have support
	 * from eSPI driver
	 */
	MCHP_GIRQ_ENCLR(MCHP_SAF_GIRQ) = MCHP_SAF_GIRQ_ECP_DONE_BIT;
	MCHP_GIRQ_SRC(MCHP_SAF_GIRQ) = MCHP_SAF_GIRQ_ECP_DONE_BIT;

	regs->SAF_ECP_FLAR = pckt->flash_addr;
	regs->SAF_ECP_BFAR = (uint32_t)&slave_mem[0];

	regs->SAF_ECP_CMD = MCHP_SAF_ECP_CMD_PUT_FLASH_NP |
			    ((uint32_t)cmd << MCHP_SAF_ECP_CMD_CTYPE_POS) |
			    ((n << MCHP_SAF_ECP_CMD_LEN_POS) &
			    MCHP_SAF_ECP_CMD_LEN_MASK);
	LOG_DBG("%x regs->SAF_ECP_CMD", regs->SAF_ECP_CMD);
	/* TODO when interrupts are available enable here */
	regs->SAF_ECP_START = MCHP_SAF_ECP_START;

	/* TODO
	 * ISR is in eSPI driver. Use polling until eSPI driver has been
	 * modified to provide callback for GIRQ19 SAF ECP Done.
	 */
	LOG_DBG("SAF ECP busy %x", regs->SAF_ECP_BUSY);
	while (regs->SAF_ECP_BUSY & MCHP_SAF_ECP_BUSY) {
		k_sleep(K_MSEC(1));
	}

	memcpy(pckt->buf, slave_mem, pckt->len);

	return 0;
}

/* Flash read using SAF EC Portal */
static int saf_xec_flash_read(struct device *dev,
			      struct espi_saf_packet *pckt)
{
	LOG_DBG("%s", __func__);
	return saf_ecp_access(dev, pckt, MCHP_SAF_ECP_CMD_CTYPE_READ0);
}

/* Flash write using SAF EC Portal */
static int saf_xec_flash_write(struct device *dev,
			       struct espi_saf_packet *pckt)
{
	return saf_ecp_access(dev, pckt, MCHP_SAF_ECP_CMD_CTYPE_WRITE0);
}

/* Flash erase using SAF EC Portal */
static int saf_xec_flash_erase(struct device *dev,
			       struct espi_saf_packet *pckt)
{
	return saf_ecp_access(dev, pckt, MCHP_SAF_ECP_CMD_CTYPE_ERASE0);
}


static int espi_saf_xec_manage_callback(struct device *dev,
					struct espi_callback *callback,
					bool set)
{
	struct espi_saf_xec_data *data = DEV_DATA(dev);

	return espi_manage_callback(&data->callbacks, callback, set);
}

static int espi_saf_xec_activate(struct device *dev)
{
	const struct espi_saf_xec_config *cfg;
	MCHP_SAF_HW_REGS *regs;

	if (dev == NULL) {
		return -EINVAL;
	}

	cfg = DEV_CFG(dev);
	regs = (MCHP_SAF_HW_REGS *)cfg->saf_base_addr;

	regs->SAF_FL_CFG_MISC |= MCHP_SAF_FL_CFG_MISC_SAF_EN;

	return 0;
}

static int espi_saf_xec_init(struct device *dev);

static const struct espi_saf_driver_api espi_saf_xec_driver_api = {
	.config = espi_saf_xec_configuration,
	.activate = espi_saf_xec_activate,
	.get_channel_status = espi_saf_xec_channel_ready,
	.flash_read = saf_xec_flash_read,
	.flash_write = saf_xec_flash_write,
	.flash_erase = saf_xec_flash_erase,
	.manage_callback = espi_saf_xec_manage_callback,
};

static struct espi_saf_xec_data espi_saf_xec_data;

static const struct espi_saf_xec_config espi_saf_xec_config = {
	.saf_base_addr = DT_INST_REG_ADDR_BY_IDX(0, 0),
	.qmspi_base_addr = DT_INST_REG_ADDR_BY_IDX(0, 1),
	.saf_comm_base_addr = DT_INST_REG_ADDR_BY_IDX(0, 2),
#if DT_INST_NODE_HAS_PROP(0, prmap)
	.prmap0 = DT_INST_PROP_BY_IDX(0, prmap, 0),
	.prmap1 = DT_INST_PROP_BY_IDX(0, prmap, 1),
	.prmap2 = DT_INST_PROP_BY_IDX(0, prmap, 2),
#else
	.prmap0 = SAF_TAG_MAP0,
	.prmap1 = SAF_TAG_MAP1,
	.prmap2 = SAF_TAG_MAP2,
#endif
};

DEVICE_AND_API_INIT(espi_saf_xec_0, DT_INST_LABEL(0),
		    &espi_saf_xec_init, &espi_saf_xec_data,
		    &espi_saf_xec_config,
		    POST_KERNEL, CONFIG_ESPI_SAF_INIT_PRIORITY,
		    &espi_saf_xec_driver_api);

static int espi_saf_xec_init(struct device *dev)
{
	struct espi_saf_xec_data *data = DEV_DATA(dev);

	/* ungate SAF clocks by disabling PCR sleep enable */
	mchp_pcr_periph_slp_ctrl(PCR_ESPI_SAF, MCHP_PCR_SLEEP_DIS);

	/* Configure the channels and its capabilities based on build config */
	cap0_i = ESPI_CAP_REGS->GLB_CAP0;
	capfc_i = ESPI_CAP_REGS->FC_CAP;
	ESPI_CAP_REGS->GLB_CAP0 |= MCHP_ESPI_GBL_CAP0_FC_SUPP;
	ESPI_CAP_REGS->FC_CAP &= ~(MCHP_ESPI_FC_CAP_SHARE_MASK);
	ESPI_CAP_REGS->FC_CAP |= MCHP_ESPI_FC_CAP_SHARE_MAF_SAF;
	cap0_f = ESPI_CAP_REGS->GLB_CAP0;
	capfc_f = ESPI_CAP_REGS->FC_CAP;

	k_sem_init(&data->ecp_lock, 0, 1);

	return 0;
}
