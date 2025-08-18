/*
 * Copyright 2025 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_I2C_REGS_H
#define _MEC_I2C_REGS_H

#include <zephyr/arch/cpu.h>

#define MEC_I2C_SMB0_BASE_ADDR 0x40004000u
#define MEC_I2C_SMB_INST_SIZE  0x400u

#define MEC_I2C_SMB_BASE(i) (MEC_I2C_SMB0_BASE_ADDR + ((uint32_t)(i) * MEC_I2C_SMB_INST_SIZE))

/* registers */
#define MEC_I2C_CR_OFS         0 /* WO */
#define MEC_I2C_CR_MSK         0xcfu
#define MEC_I2C_CR_ACK_POS     0
#define MEC_I2C_CR_STO_POS     1
#define MEC_I2C_CR_STA_POS     2
#define MEC_I2C_CR_ENI_POS     3
#define MEC_I2C_CR_ESO_POS     6
#define MEC_I2C_CR_PIN_POS     7
#define MEC_I2C_SR_OFS         0 /* RO */
#define MEC_I2C_SR_MSK         0xffu
#define MEC_I2C_SR_NBB_POS     0
#define MEC_I2C_SR_LAB_POS     1
#define MEC_I2C_SR_AAT_POS     2
#define MEC_I2C_SR_LRB_AD0_POS 3
#define MEC_I2C_SR_BER_POS     4
#define MEC_I2C_SR_STO_POS     5
#define MEC_I2C_SR_SAD_POS     6
#define MEC_I2C_SR_PIN_POS     7
#define MEC_I2C_OA_OFS         0x4u /* own address */
#define MEC_I2C_OA_1_MSK       MEC_GENMASK(6, 0)
#define MEC_I2C_OA_2_MSK       MEC_GENMASK(14, 8)
#define MEC_I2C_OA_1_SET(a)    MEC_FIELD_PREP(MEC_I2C_OA_1_MSK, (a))
#define MEC_I2C_OA_2_SET(a)    MEC_FIELD_PREP(MEC_I2C_OA_2_MSK, (a))
#define MEC_I2C_OA_1_GET(a)    MEC_FIELD_GET(MEC_I2C_OA_1_MSK, (a))
#define MEC_I2C_OA_2_GET(a)    MEC_FIELD_GET(MEC_I2C_OA_2_MSK, (a))

#define MEC_I2C_DATA_OFS      0x8u
#define MEC_I2C_HC_OFS        0x0cu /* network layer host command */
#define MEC_I2C_HC_RUN_POS    0
#define MEC_I2C_HC_PROC_POS   1
#define MEC_I2C_HC_START0_POS 8
#define MEC_I2C_HC_STARTN_POS 9
#define MEC_I2C_HC_STOP_POS   10
#define MEC_I2C_HC_PEC_POS    11
#define MEC_I2C_HC_RDM_POS    12
#define MEC_I2C_HC_RDP_POS    13
#define MEC_I2C_HC_WCL_POS    16
#define MEC_I2C_HC_WCL_MSK    MEC_GENMASK(23, 16)
#define MEC_I2C_HC_WCL_SET(n) MEC_FIELD_PREP(MEC_I2C_HC_WCL_MSK, (n))
#define MEC_I2C_HC_WCL_GET(n) MEC_FIELD_GET(MEC_I2C_HC_WCL_MSK, (n))
#define MEC_I2C_HC_RCL_POS    24
#define MEC_I2C_HC_RCL_MSK    MEC_GENMASK(31, 24)
#define MEC_I2C_HC_RCL_SET(n) MEC_FIELD_PREP(MEC_I2C_HC_RCL_MSK, (n))
#define MEC_I2C_HC_RCL_GET(n) MEC_FIELD_GET(MEC_I2C_HC_RCL_MSK, (n))

#define MEC_I2C_TC_OFS        0x10u /* network layer target command */
#define MEC_I2C_TC_RUN_POS    0
#define MEC_I2C_TC_PROC_POS   1
#define MEC_I2C_TC_PEC_POS    2
#define MEC_I2C_TC_WCL_POS    8
#define MEC_I2C_TC_WCL_MSK    MEC_GENMASK(15, 8)
#define MEC_I2C_TC_WCL_SET(n) MEC_FIELD_PREP(MEC_I2C_TC_WCL_MSK, (n))
#define MEC_I2C_TC_WCL_GET(n) MEC_FIELD_GET(MEC_I2C_TC_WCL_MSK, (n))
#define MEC_I2C_TC_RCL_POS    16
#define MEC_I2C_TC_RCL_MSK    MEC_GENMASK(23, 16)
#define MEC_I2C_TC_RCL_SET(n) MEC_FIELD_PREP(MEC_I2C_TC_RCL_MSK, (n))
#define MEC_I2C_TC_RCL_GET(n) MEC_FIELD_GET(MEC_I2C_TC_RCL_MSK, (n))
drivers/i2c/i2c_mchp_xec_v2.h
#define MEC_I2C_PEC_OFS 0x14u
#define MEC_I2C_PEC_MSK MEC_GENMASK(7, 0)

#define MEC_I2C_RSHT_OFS 0x18u /* Repeated-START hold time */
#define MEC_I2C_RSHT_MSK MEC_GENMASK(7, 0)

#define MEC_I2C_ELEN_OFS         0x1cu /* Extended length: MSB's of HC and TC read/write counts */
#define MEC_I2C_ELEN_HWRM_POS    0
#define MEC_I2C_ELEN_HWRM_MSK    MEC_GENMASK(7, 0)
#define MEC_I2C_ELEN_HWRM_SET(n) MEC_FIELD_PREP(MEC_I2C_ELEN_HWRM_MSK, (n))
#define MEC_I2C_ELEN_HWRM_GET(n) MEC_FIELD_GET(MEC_I2C_ELEN_HWRM_MSK, (n))
#define MEC_I2C_ELEN_HRDM_POS    8
#define MEC_I2C_ELEN_HRDM_MSK    MEC_GENMASK(15, 8)
#define MEC_I2C_ELEN_HRDM_SET(n) MEC_FIELD_PREP(MEC_I2C_ELEN_HRDM_MSK, (n))
#define MEC_I2C_ELEN_HRDM_GET(n) MEC_FIELD_GET(MEC_I2C_ELEN_HRDM_MSK, (n))
#define MEC_I2C_ELEN_TWRM_POS    16
#define MEC_I2C_ELEN_TWRM_MSK    MEC_GENMASK(23, 16)
#define MEC_I2C_ELEN_TWRM_SET(n) MEC_FIELD_PREP(MEC_I2C_ELEN_TWRM_MSK, (n))
#define MEC_I2C_ELEN_TWRM_GET(n) MEC_FIELD_GET(MEC_I2C_ELEN_TWRM_MSK, (n))
#define MEC_I2C_ELEN_TRDM_POS    24
#define MEC_I2C_ELEN_TRDM_MSK    MEC_GENMASK(31, 24)
#define MEC_I2C_ELEN_TRDM_SET(n) MEC_FIELD_PREP(MEC_I2C_ELEN_TRDM_MSK, (n))
#define MEC_I2C_ELEN_TRDM_GET(n) MEC_FIELD_GET(MEC_I2C_ELEN_TRDM_MSK, (n))

#define MEC_I2C_COMP_OFS 0x20u /* Completion: R/W1C status and timeout check enables */
#define MEC_I2C_COMP_MSK                                                                           \
	(MEC_GENMASK(6, 2) | MEC_GENMASK(14, 8) | MEC_GENMASK(17, 16) | MEC_GENMASK(21, 19) |      \
	 MEC_GENMASK(25, 24) | MEC_GENMASK(31, 29))
#define MEC_I2C_COMP_RW_MSK MEC_GENMASK(5, 2)
#define MEC_I2C_COMP_RO_MSK (MEC_BIT(6) | MEC_BIT(17) | MEC_BIT(25))
#define MEC_I2C_COMP_RW1C_MSK                                                                      \
	(MEC_GENMASK(14, 8) | MEC_BIT(16) | MEC_GENMASK(21, 19) | MEC_BIT(24) | MEC_GENMASK(31, 29))
#define MEC_I2C_COMP_DTEN_POS      2
#define MEC_I2C_COMP_HCEN_POS      3
#define MEC_I2C_COMP_TCEN_POS      4
#define MEC_I2C_COMP_BIDEN_POS     5
#define MEC_I2C_COMP_TMO_STS_POS   6 /* RO - any of 4 timeouts detected */
#define MEC_I2C_COMP_DTS_STS_POS   8
#define MEC_I2C_COMP_HCTO_STS_POS  9
#define MEC_I2C_COMP_TCTO_STS_POS  10
#define MEC_I2C_COMP_CHDL_STS_POS  11
#define MEC_I2C_COMP_CHDH_STS_POS  12
#define MEC_I2C_COMP_BER_STS_POS   13
#define MEC_I2C_COMP_LAB_STS_POS   14
#define MEC_I2C_COMP_TNAKR_STS_POS 16
#define MEC_I2C_COMP_TTR_POS       17 /* RO */
#define MEC_I2C_COMP_TPROT_POS     19
#define MEC_I2C_COMP_RPT_RD_POS    20
#define MEC_I2C_COMP_RPT_WR_POS    21
#define MEC_I2C_COMP_HNAKX_POS     24
#define MEC_I2C_COMP_HTR_POS       25 /* RO */
#define MEC_I2C_COMP_IDLE_POS      29
#define MEC_I2C_COMP_HDONE_POS     30
#define MEC_I2C_COMP_TDONE_POS     31

#define MEC_I2C_ISC_OFS         0x24u /* fairness idle time scaling */
#define MEC_I2C_ISC_FBI_POS     0
#define MEC_I2C_ISC_FBI_MSK     MEC_GENMASK(11, 0)
#define MEC_I2C_ISC_FBI_SET(n)  MEC_FIELD_PREP(MEC_I2C_ISC_FBI_MSK, (n))
#define MEC_I2C_ISC_FBI_GET(n)  MEC_FIELD_GET(MEC_I2C_ISC_FBI_MSK, (n))
#define MEC_I2C_ISC_FIDD_POS    16
#define MEC_I2C_ISC_FIDD_MSK    MEC_GENMASK(27, 16)
#define MEC_I2C_ISC_FIDD_SET(n) MEC_FIELD_PREP(MEC_I2C_ISC_FIDD_MSK, (n))
#define MEC_I2C_ISC_FIDD_GET(n) MEC_FIELD_GET(MEC_I2C_ISC_FIDD_MSK, (n))

#define MEC_I2C_CFG_OFS            0x28u
#define MEC_I2C_CFG_PORT_POS       0
#define MEC_I2C_CFG_PORT_MSK       MEC_GENMASK(3, 0)
#define MEC_I2C_CFG_PORT_SET(p)    MEC_FIELD_PREP(MEC_I2C_CFG_PORT_MSK, (p))
#define MEC_I2C_CFG_PORT_GET(p)    MEC_FIELD_GET(MEC_I2C_CFG_PORT_MSK, (p))
#define MEC_I2C_CFG_TCEN_POS       4
#define MEC_I2C_CFG_SLOW_CLK_POS   5
#define MEC_I2C_CFG_PECEN_POS      7
#define MEC_I2C_CFG_FEN_POS        8
#define MEC_I2C_CFG_RST_POS        9
#define MEC_I2C_CFG_ENAB_POS       10
#define MEC_I2C_CFG_DSA_POS        11
#define MEC_I2C_CFG_FAIR_POS       12
#define MEC_I2C_CFG_GC_POS         14
#define MEC_I2C_CFG_PROM_EN_POS    15
#define MEC_I2C_CFG_FTTX_POS       16 /* WO */
#define MEC_I2C_CFG_FTRX_POS       17 /* WO */
#define MEC_I2C_CFG_FHTX_POS       18 /* WO */
#define MEC_I2C_CFG_FHRX_POS       19 /* WO */
#define MEC_I2C_CFG_STD_IEN_POS    24 /* v3.8 HW only */
#define MEC_I2C_CFG_STD_NL_IEN_POS 27 /* v3.8 HW only */
#define MEC_I2C_CFG_AAT_IEN_POS    28
#define MEC_I2C_CFG_IDLE_IEN_POS   29
#define MEC_I2C_CFG_HD_IEN_POS     30
#define MEC_I2C_CFG_TD_IEN_POS     31

#define MEC_I2C_BCLK_OFS        0x2cu /* bus clock */
#define MEC_I2C_BCLK_LOP_POS    0
#define MEC_I2C_BCLK_LOP_MSK    MEC_GENMASK(7, 0)
#define MEC_I2C_BCLK_LOP_SET(n) MEC_FIELD_PREP(MEC_I2C_BCLK_LOP_MSK, (n))
#define MEC_I2C_BCLK_LOP_GET(n) MEC_FIELD_GET(MEC_I2C_BCLK_LOP_MSK, (n))
#define MEC_I2C_BCLK_HIP_POS    8
#define MEC_I2C_BCLK_HIP_MSK    MEC_GENMASK(15, 8)
#define MEC_I2C_BCLK_HIP_SET(n) MEC_FIELD_PREP(MEC_I2C_BCLK_HIP_MSK, (n))
#define MEC_I2C_BCLK_HIP_GET(n) MEC_FIELD_GET(MEC_I2C_BCLK_HIP_MSK, (n))

#define MEC_I2C_BLKID_OFS 0x30u /* RO HW block ID */
#define MEC_I2C_REV_OFS   0x34u /* RO HW revision */

#define MEC_I2C_BBCR_OFS        0x38u /* bit-bang control */
#define MEC_I2C_BBCR_EN_POS     0     /* MUX SCL/SDA pins from I2C to BB logic */
#define MEC_I2C_BBCR_CD_POS     1
#define MEC_I2C_BBCR_DD_POS     2
#define MEC_I2C_BBCR_SCL_POS    3
#define MEC_I2C_BBCR_SDA_POS    4
#define MEC_I2C_BBCR_SCL_IN_POS 5
#define MEC_I2C_BBCR_SDA_IN_POS 6
#define MEC_I2C_BBCR_CM_POS     7 /* ver3.8 only */

#define MEC_I2C_MR0_OFS        0x3cu /* MCHP reserved 0 */
#define MEC_I2C_DT_OFS         0x40u /* data timing */
#define MEC_I2C_DT_DH_POS      0
#define MEC_I2C_DT_DH_MSK      MEC_GENMASK(7, 0)
#define MEC_I2C_DT_DH_SET(n)   MEC_FIELD_PREP(MEC_I2C_DT_DH_MSK, (n))
#define MEC_I2C_DT_DH_GET(n)   MEC_FIELD_GET(MEC_I2C_DT_DH_MSK, (n))
#define MEC_I2C_DT_RSS_POS     8
#define MEC_I2C_DT_RSS_MSK     MEC_GENMASK(15, 8)
#define MEC_I2C_DT_RSS_SET(n)  MEC_FIELD_PREP(MEC_I2C_DT_RSS_MSK, (n))
#define MEC_I2C_DT_RSS_GET(n)  MEC_FIELD_GET(MEC_I2C_DT_RSS_MSK, (n))
#define MEC_I2C_DT_STPS_POS    16
#define MEC_I2C_DT_STPS_MSK    MEC_GENMASK(23, 16)
#define MEC_I2C_DT_STPS_SET(n) MEC_FIELD_PREP(MEC_I2C_DT_STPS_MSK, (n))
#define MEC_I2C_DT_STPS_GET(n) MEC_FIELD_GET(MEC_I2C_DT_STPS_MSK, (n))
#define MEC_I2C_DT_FSH_POS     24
#define MEC_I2C_DT_FSH_MSK     MEC_GENMASK(31, 24)
#define MEC_I2C_DT_FSH_SET(n)  MEC_FIELD_PREP(MEC_I2C_DT_FSH_MSK, (n))
#define MEC_I2C_DT_FSH_GET(n)  MEC_FIELD_GET(MEC_I2C_DT_FSH_MSK, (n))

#define MEC_I2C_TOSC_OFS         0x44u /* timeout scaling */
#define MEC_I2C_TOSC_CHTO_POS    0
#define MEC_I2C_TOSC_CHTO_MSK    MEC_GENMASK(7, 0)
#define MEC_I2C_TOSC_CHTO_SET(n) MEC_FIELD_PREP(MEC_I2C_TOSC_CHTO_MSK, (n))
#define MEC_I2C_TOSC_CHTO_GET(n) MEC_FIELD_GET(MEC_I2C_TOSC_CHTO_MSK, (n))
#define MEC_I2C_TOSC_TCTO_POS    8
#define MEC_I2C_TOSC_TCTO_MSK    MEC_GENMASK(15, 8)
#define MEC_I2C_TOSC_TCTO_SET(n) MEC_FIELD_PREP(MEC_I2C_TOSC_TCTO_MSK, (n))
#define MEC_I2C_TOSC_TCTO_GET(n) MEC_FIELD_GET(MEC_I2C_TOSC_TCTO_MSK, (n))
#define MEC_I2C_TOSC_HCTO_POS    16
#define MEC_I2C_TOSC_HCTO_MSK    MEC_GENMASK(23, 16)
#define MEC_I2C_TOSC_HCTO_SET(n) MEC_FIELD_PREP(MEC_I2C_TOSC_HCTO_MSK, (n))
#define MEC_I2C_TOSC_HCTO_GET(n) MEC_FIELD_GET(MEC_I2C_TOSC_HCTO_MSK, (n))
#define MEC_I2C_TOSC_BIM_POS     24
#define MEC_I2C_TOSC_BIM_MSK     MEC_GENMASK(31, 24)
#define MEC_I2C_TOSC_BIM_SET(n)  MEC_FIELD_PREP(MEC_I2C_TOSC_BIM_MSK, (n))
#define MEC_I2C_TOSC_BIM_GET(n)  MEC_FIELD_GET(MEC_I2C_TOSC_BIM_MSK, (n))

#define MEC_I2C_TTX_OFS  0x48u /* network layer mode target transmit */
#define MEC_I2C_TRX_OFS  0x4cu /* network layer mode target receive */
#define MEC_I2C_HTX_OFS  0x50u /* network layer mode host transmit */
#define MEC_I2C_HRX_OFS  0x54u /* network layer mode host receive */
#define MEC_I2C_IFSM_OFS 0x58u /* RO I2C HW FSM */
#define MEC_I2C_NFSM_OFS 0x5cu /* RO Network layer mode HW FSM */

#define MEC_I2C_WKSR_OFS    0x60u /* wake status */
#define MEC_I2C_WKSR_SB_POS 0     /* start bit detected R/W1C */

#define MEC_I2C_WKCR_OFS      0x64u /* wake control */
#define MEC_I2C_WKCR_SBEN_POS 0     /* start bit detection interrupt enable */

#define MEC_I2C_MR1_OFS 0x68u /* MCHP reserved 1 */
#define MEC_I2C_IAS_OFS 0x6cu /* RO I2C address shadow */

#define MEC_I2C_PIS_OFS     0x70u /* I2C promiscuous mode interrupt status */
#define MEC_I2C_PIS_CAP_POS 0     /* I2C has captured 8 address bits and is stretching the clock */

#define MEC_I2C_PIE_OFS     0x74u /* I2C promiscuous mode interrupt enable */
#define MEC_I2C_PIE_CAP_POS 0

#define MEC_I2C_PCR_OFS     0x78u /* I2C promiscuous mode control */
#define MEC_I2C_PCR_ACK_POS 0     /* write 1 to ACK the captured address */

#define MEC_I2C_IDS_OFS 0x7Cu /* RO I2C data shadow */

#define MEC_I2C_BUS_CLK_100K_DFLT  0x00004F4Fu
#define MEC_I2C_DATA_TM_100K_DFLT  0x0C4D5006u
#define MEC_I2C_STA_HTM_100K_DFLT  0x0000004Du
#define MEC_I2C_IDLE_SC_100K_DFLT  0x01FC01EDu
#define MEC_I2C_TMOUT_SC_100K_DFLT 0x4B9CC2C7u

#define MEC_I2C_BUS_CLK_400K_DFLT  0x00000F17u
#define MEC_I2C_DATA_TM_400K_DFLT  0x040A0A06u
#define MEC_I2C_STA_HTM_400K_DFLT  0x0000000Au
#define MEC_I2C_IDLE_SC_400K_DFLT  0x01000050u
#define MEC_I2C_TMOUT_SC_400K_DFLT 0x159CC2C7u

#define MEC_I2C_BUS_CLK_1M_DFLT  0x00000509u
#define MEC_I2C_DATA_TM_1M_DFLT  0x04060601u
#define MEC_I2C_STA_HTM_1M_DFLT  0x00000006u
#define MEC_I2C_IDLE_SC_1M_DFLT  0x10000050u
#define MEC_I2C_TMOUT_SC_1M_DFLT 0x089CC2C7u

#endif /* #ifndef _MEC_I2C_REGS_H */
