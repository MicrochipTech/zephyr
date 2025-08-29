/*
 * Copyright 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I2C_I2C_MCHP_XEC_REGS_H_
#define ZEPHYR_DRIVERS_I2C_I2C_MCHP_XEC_REGS_H_

#include <stdef.h>
#include <stdint.h>
#include <zephyr/arch/cpu.h>

#define XEC_I2C_SMB0_ID 0
#define XEC_I2C_SMB1_ID 1
#define XEC_I2C_SMB2_ID 2
#define XEC_I2C_SMB3_ID 3
#define XEC_I2C_SMB4_ID 4
#define XEC_I2C_SMB_MAX_ID 5

#define XEC_I2C_SMB_INST_SIZE  0x400u

#define XEC_I2C_SMB_BASE(base, instance)	\
	((uint32_t)(b) + ((uint32_t)(i) * (XEC_I2C_SMB_INST_SIZE)))

#define XEC_I2C_CR_OFS		0 /* WO */
	#define XEC_I2C_CR_MSK			0xcfu
	#define XEC_I2C_CR_ACK_POS		0
	#define XEC_I2C_CR_STO_POS		1
	#define XEC_I2C_CR_STA_POS		2
	#define XEC_I2C_CR_ENI_POS		3
	#define XEC_I2C_CR_ESO_POS		6
	#define XEC_I2C_CR_PIN_POS		7

#define XEC_I2C_SR_OFS		0 /* RO */
	#define XEC_I2C_SR_MSK			0xffu
	#define XEC_I2C_SR_NBB_POS		0
	#define XEC_I2C_SR_LAB_POS		1
	#define XEC_I2C_SR_AAT_POS		2
	#define XEC_I2C_SR_LRB_AD0_POS		3
	#define XEC_I2C_SR_BER_POS		4
	#define XEC_I2C_SR_STO_POS		5
	#define XEC_I2C_SR_SAD_POS		6
	#define XEC_I2C_SR_PIN_POS		7

#define XEC_I2C_OA_OFS		0x4u /* own(target) address */
	#define XEC_I2C_OA_1_MSK		GENMASK(6, 0)
	#define XEC_I2C_OA_2_MSK		GENMASK(14, 8)
	#define XEC_I2C_OA_1_SET(a)		FIELD_PREP(XEC_I2C_OA_1_MSK, (a))
	#define XEC_I2C_OA_2_SET(a)		FIELD_PREP(XEC_I2C_OA_2_MSK, (a))
	#define XEC_I2C_OA_1_GET(r)		FIELD_GET(XEC_I2C_OA_1_MSK, (r))
	#define XEC_I2C_OA_2_GET(r)		FIELD_GET(XEC_I2C_OA_2_MSK, (r))

#define XEC_I2C_DATA_OFS	0x8u
	#define XEC_I2C_DATA_MSK		GENMASK(7, 0)

#define XEC_I2C_HCMD_OFS	0x0cu /* network layer host command */
	#define XEC_I2C_HCMD_RUN_POS		0
	#define XEC_I2C_HCMD_PROC_POS		1
	#define XEC_I2C_HCMD_START0_POS		8
	#define XEC_I2C_HCMD_STARTN_POS		9
	#define XEC_I2C_HCMD_STOP_POS		10
	#define XEC_I2C_HCMD_PEC_TX_POS		11
	#define XEC_I2C_HCMD_RDM_POS		12
	#define XEC_I2C_HCMD_PEC_RD_POS		13
	#define XEC_I2C_HCMD_WCL_POS		16
	#define XEC_I2C_HCMD_WCL_MSK		GENMASK(23, 16)
	#define XEC_I2C_HCMD_WCL_SET(n)		FIELD_PREP(XEC_I2C_HCMD_WCL_MSK, (n))
	#define XEC_I2C_HCMD_WCL_GET(r)		FIELD_GET(XEC_I2C_HCMD_WCL_MSK, (r))
	#define XEC_I2C_HCMD_RCL_POS		24
	#define XEC_I2C_HCMD_RCL_MSK		GENMASK(31, 24)
	#define XEC_I2C_HCMD_RCL_SET(n)		FIELD_PREP(XEC_I2C_HCMD_RCL_MSK, (n))
	#define XEC_I2C_HCMD_RCL_GET(r)		FIELD_GET(XEC_I2C_HCMD_RCL_MSK, (r))

#define XEC_I2C_TC_OFS		0x10u /* network layer target command */
	#define XEC_I2C_TCMD_RUN_POS		0
	#define XEC_I2C_TCMD_PROC_POS		1
	#define XEC_I2C_TCMD_TX_PEC_POS		2
	#define XEC_I2C_TCMD_WCL_POS		8
	#define XEC_I2C_TCMD_WCL_MSK		GENMASK(15, 8)
	#define XEC_I2C_TCMD_WCL_SET(n)		FIELD_PREP(XEC_I2C_TCMD_WCL_MSK, (n))
	#define XEC_I2C_TCMD_WCL_GET(r)		FIELD_GET(XEC_I2C_TCMD_WCL_MSK, (r))
	#define XEC_I2C_TCMD_RCL_POS		16
	#define XEC_I2C_TCMD_RCL_MSK		GENMASK(23, 16)
	#define XEC_I2C_TCMD_RCL_SET(n)		FIELD_PREP(XEC_I2C_TCMD_RCL_MSK, (n))
	#define XEC_I2C_TCMD_RCL_GET(r)		FIELD_GET(XEC_I2C_TCMD_RCL_MSK, (r))

#define XEC_I2C_PEC_OFS		0x14u
	#define XEC_I2C_PEC_MSK			GENMASK(7, 0)

#define XEC_I2C_RSHT_OFS	0x18u /* Repeated-START hold time */
	#define XEC_I2C_RSHT_MSK		GENMASK(7, 0)

/* Network layer mode extended length. Contains bit[15:8] of write and read counts */
#define XEC_I2C_ELEN_OFS	0x1cu
	#define XEC_I2C_ELEN_HWR_POS		0
	#define XEC_I2C_ELEN_HWR_MSK		GENMASK(7, 0)
	#define XEC_I2C_ELEN_HWR_SET(n)		FIELD_PREP(XEC_I2C_ELEN_HWR_MSK, (n))
	#define XEC_I2C_ELEN_HWR_GET(r)		FIELD_GET(XEC_I2C_ELEN_HWR_MSK, (r))
	#define XEC_I2C_ELEN_HRD_POS		8
	#define XEC_I2C_ELEN_HRD_MSK		GENMASK(15, 8)
	#define XEC_I2C_ELEN_HRD_SET(n)		FIELD_PREP(XEC_I2C_ELEN_HRD_MSK, (n))
	#define XEC_I2C_ELEN_HRD_GET(r)		FIELD_GET(XEC_I2C_ELEN_HRD_MSK, (r))
	#define XEC_I2C_ELEN_TWR_POS		16
	#define XEC_I2C_ELEN_TWR_MSK		GENMASK(23, 16)
	#define XEC_I2C_ELEN_TWR_SET(n)		FIELD_PREP(XEC_I2C_ELEN_TWR_MSK, (n))
	#define XEC_I2C_ELEN_TWR_GET(r)		FIELD_GET(XEC_I2C_ELEN_TWR_MSK, (r))
	#define XEC_I2C_ELEN_TRD_POS		24
	#define XEC_I2C_ELEN_TRD_MSK		GENMASK(31, 24)
	#define XEC_I2C_ELEN_TRD_SET(n)		FIELD_PREP(XEC_I2C_ELEN_TRD_MSK, (n))
	#define XEC_I2C_ELEN_TRD_GET(r)		FIELD_GET(XEC_I2C_ELEN_TRD_MSK, (r))

#define XEC_I2C_COMP_OFS	0x20u /* Completion: R/W1C status and timeout check enables */
#define XEC_I2C_COMP_MSK	(GENMASK(6, 2) | GENMASK(14, 8) | GENMASK(17, 16) |\
				 GENMASK(21, 19) | GENMASK(25, 24) | GENMASK(31, 29))
#define XEC_I2C_COMP_RW_MSK	GENMASK(5, 2)
#define XEC_I2C_COMP_RO_MSK	(BIT(6) | BIT(17) | BIT(25))
#define XEC_I2C_COMP_RW1C_MSK	(GENMASK(14, 8) | BIT(16) | GENMASK(21, 19) |\
				 BIT(24) | GENMASK(31, 29))
	#define XEC_I2C_COMP_DTEN_POS		2
	#define XEC_I2C_COMP_HCEN_POS		3
	#define XEC_I2C_COMP_TCEN_POS		4
	#define XEC_I2C_COMP_BIDEN_POS		5
	#define XEC_I2C_COMP_TMO_STS_POS	6 /* RO - any of 4 timeouts detected */
	#define XEC_I2C_COMP_DTS_STS_POS	8
	#define XEC_I2C_COMP_HCTO_STS_POS	9
	#define XEC_I2C_COMP_TCTO_STS_POS	10
	#define XEC_I2C_COMP_CHDL_STS_POS	11
	#define XEC_I2C_COMP_CHDH_STS_POS	12
	#define XEC_I2C_COMP_BER_STS_POS	13
	#define XEC_I2C_COMP_LAB_STS_POS	14
	#define XEC_I2C_COMP_TNAKR_STS_POS	16
	#define XEC_I2C_COMP_TTR_POS		17 /* RO */
	#define XEC_I2C_COMP_TPROT_POS		19
	#define XEC_I2C_COMP_RPT_RD_POS		20
	#define XEC_I2C_COMP_RPT_WR_POS		21
	#define XEC_I2C_COMP_HNAKX_POS		24
	#define XEC_I2C_COMP_HTR_POS		25 /* RO */
	#define XEC_I2C_COMP_IDLE_POS		29
	#define XEC_I2C_COMP_HDONE_POS		30
	#define XEC_I2C_COMP_TDONE_POS		31

#define XEC_I2C_ISC_OFS		0x24u /* fairness idle time scaling */
	#define XEC_I2C_ISC_FBI_POS		0
	#define XEC_I2C_ISC_FBI_MSK		GENMASK(11, 0)
	#define XEC_I2C_ISC_FBI_SET(n)		FIELD_PREP(XEC_I2C_ISC_FBI_MSK, (n))
	#define XEC_I2C_ISC_FBI_GET(r)		FIELD_GET(XEC_I2C_ISC_FBI_MSK, (r))
	#define XEC_I2C_ISC_FIDD_POS		16
	#define XEC_I2C_ISC_FIDD_MSK		GENMASK(27, 16)
	#define XEC_I2C_ISC_FIDD_SET(n)		FIELD_PREP(XEC_I2C_ISC_FIDD_MSK, (n))
	#define XEC_I2C_ISC_FIDD_GET(r)		FIELD_GET(XEC_I2C_ISC_FIDD_MSK, (r))

#define XEC_I2C_CFG_OFS		0x28u
	#define XEC_I2C_CFG_PORT_POS		0
	#define XEC_I2C_CFG_PORT_MSK		GENMASK(3, 0)
	#define XEC_I2C_CFG_PORT_SET(p)		FIELD_PREP(XEC_I2C_CFG_PORT_MSK, (p))
	#define XEC_I2C_CFG_PORT_GET(r)		FIELD_GET(XEC_I2C_CFG_PORT_MSK, (r))
	#define XEC_I2C_CFG_TCEN_POS		4
	#define XEC_I2C_CFG_SLOW_CLK_POS	5
	#define XEC_I2C_CFG_PECEN_POS		7
	#define XEC_I2C_CFG_FEN_POS		8
	#define XEC_I2C_CFG_RST_POS		9
	#define XEC_I2C_CFG_ENAB_POS		10
	#define XEC_I2C_CFG_DSA_POS		11
	#define XEC_I2C_CFG_FAIR_POS		12
	#define XEC_I2C_CFG_GC_POS		14
	#define XEC_I2C_CFG_PROM_EN_POS		15
	#define XEC_I2C_CFG_FTTX_POS		16 /* WO */
	#define XEC_I2C_CFG_FTRX_POS		17 /* WO */
	#define XEC_I2C_CFG_FHTX_POS		18 /* WO */
	#define XEC_I2C_CFG_FHRX_POS		19 /* WO */
	#define XEC_I2C_CFG_STD_IEN_POS		24 /* v3.8 HW only */
	#define XEC_I2C_CFG_STD_NL_IEN_POS	27 /* v3.8 HW only */
	#define XEC_I2C_CFG_AAT_IEN_POS		28
	#define XEC_I2C_CFG_IDLE_IEN_POS	29
	#define XEC_I2C_CFG_HD_IEN_POS		30
	#define XEC_I2C_CFG_TD_IEN_POS		31

#define XEC_I2C_BCLK_OFS	0x2cu /* bus clock */
	#define XEC_I2C_BCLK_LOP_POS		0
	#define XEC_I2C_BCLK_LOP_MSK		GENMASK(7, 0)
	#define XEC_I2C_BCLK_LOP_SET(n)		FIELD_PREP(XEC_I2C_BCLK_LOP_MSK, (n))
	#define XEC_I2C_BCLK_LOP_GET(r)		FIELD_GET(XEC_I2C_BCLK_LOP_MSK, (r))
	#define XEC_I2C_BCLK_HIP_POS		8
	#define XEC_I2C_BCLK_HIP_MSK		GENMASK(15, 8)
	#define XEC_I2C_BCLK_HIP_SET(n)		FIELD_PREP(XEC_I2C_BCLK_HIP_MSK, (n))
	#define XEC_I2C_BCLK_HIP_GET(r)		FIELD_GET(XEC_I2C_BCLK_HIP_MSK, (r))

#define XEC_I2C_BLKID_OFS	0x30u /* RO HW block ID */
#define XEC_I2C_REV_OFS		0x34u /* RO HW revision */

#define XEC_I2C_BBCR_OFS	0x38u /* bit-bang control */
	#define XEC_I2C_BBCR_EN_POS		0 /* MUX SCL/SDA pins from I2C to BB logic */
	#define XEC_I2C_BBCR_CD_POS		1
	#define XEC_I2C_BBCR_DD_POS		2
	#define XEC_I2C_BBCR_SCL_POS		3
	#define XEC_I2C_BBCR_SDA_POS		4
	#define XEC_I2C_BBCR_SCL_IN_POS		5
	#define XEC_I2C_BBCR_SDA_IN_POS		6
	#define XEC_I2C_BBCR_CM_POS		7 /* ver3.8 only */

#define XEC_I2C_MR0_OFS		0x3cu /* MCHP reserved 0 */

#define XEC_I2C_DT_OFS		0x40u /* data timing */
	#define XEC_I2C_DT_DH_POS		0
	#define XEC_I2C_DT_DH_MSK		GENMASK(7, 0)
	#define XEC_I2C_DT_DH_SET(n)		FIELD_PREP(XEC_I2C_DT_DH_MSK, (n))
	#define XEC_I2C_DT_DH_GET(r)		FIELD_GET(XEC_I2C_DT_DH_MSK, (r))
	#define XEC_I2C_DT_RSS_POS		8
	#define XEC_I2C_DT_RSS_MSK		GENMASK(15, 8)
	#define XEC_I2C_DT_RSS_SET(n)		FIELD_PREP(XEC_I2C_DT_RSS_MSK, (n))
	#define XEC_I2C_DT_RSS_GET(r)		FIELD_GET(XEC_I2C_DT_RSS_MSK, (r))
	#define XEC_I2C_DT_STPS_POS		16
	#define XEC_I2C_DT_STPS_MSK		GENMASK(23, 16)
	#define XEC_I2C_DT_STPS_SET(n)		FIELD_PREP(XEC_I2C_DT_STPS_MSK, (n))
	#define XEC_I2C_DT_STPS_GET(r)		FIELD_GET(XEC_I2C_DT_STPS_MSK, (r))
	#define XEC_I2C_DT_FSH_POS		24
	#define XEC_I2C_DT_FSH_MSK		GENMASK(31, 24)
	#define XEC_I2C_DT_FSH_SET(n)		FIELD_PREP(XEC_I2C_DT_FSH_MSK, (n))
	#define XEC_I2C_DT_FSH_GET(r)		FIELD_GET(XEC_I2C_DT_FSH_MSK, (r))

#define XEC_I2C_TOSC_OFS	0x44u /* timeout scaling */
	#define XEC_I2C_TOSC_CHTO_POS		0
	#define XEC_I2C_TOSC_CHTO_MSK		GENMASK(7, 0)
	#define XEC_I2C_TOSC_CHTO_SET(n)	FIELD_PREP(XEC_I2C_TOSC_CHTO_MSK, (n))
	#define XEC_I2C_TOSC_CHTO_GET(r)	FIELD_GET(XEC_I2C_TOSC_CHTO_MSK, (r))
	#define XEC_I2C_TOSC_TCTO_POS		8
	#define XEC_I2C_TOSC_TCTO_MSK		GENMASK(15, 8)
	#define XEC_I2C_TOSC_TCTO_SET(n)	FIELD_PREP(XEC_I2C_TOSC_TCTO_MSK, (n))
	#define XEC_I2C_TOSC_TCTO_GET(r)	FIELD_GET(XEC_I2C_TOSC_TCTO_MSK, (r))
	#define XEC_I2C_TOSC_HCTO_POS		16
	#define XEC_I2C_TOSC_HCTO_MSK		GENMASK(23, 16)
	#define XEC_I2C_TOSC_HCTO_SET(n)	FIELD_PREP(XEC_I2C_TOSC_HCTO_MSK, (n))
	#define XEC_I2C_TOSC_HCTO_GET(r)	FIELD_GET(XEC_I2C_TOSC_HCTO_MSK, (r))
	#define XEC_I2C_TOSC_BIM_POS		24
	#define XEC_I2C_TOSC_BIM_MSK		GENMASK(31, 24)
	#define XEC_I2C_TOSC_BIM_SET(n)		FIELD_PREP(XEC_I2C_TOSC_BIM_MSK, (n))
	#define XEC_I2C_TOSC_BIM_GET(r)		FIELD_GET(XEC_I2C_TOSC_BIM_MSK, (r))

/* 8-bit data register for network layer mode */
#define XEC_I2C_TTX_OFS		0x48u /* network layer mode target transmit */
#define XEC_I2C_TRX_OFS		0x4cu /* network layer mode target receive */
#define XEC_I2C_HTX_OFS		0x50u /* network layer mode host transmit */
#define XEC_I2C_HRX_OFS		0x54u /* network layer mode host receive */

#define XEC_I2C_IFSM_OFS	0x58u /* RO I2C HW FSM */

#define XEC_I2C_NFSM_OFS	0x5cu /* RO Network layer mode HW FSM */
	#define XEC_I2C_NFSM_HC_STATE_POS	0
	#define XEC_I2C_NFSM_HC_STATE_MSK	GENMASK(7, 0)
	#define XEC_I2C_NFSM_HC_STATE_IDLE	0
	#define XEC_I2C_NFSM_HC_STATE_SOP	1u
	#define XEC_I2C_NFSM_HC_STATE_STA	2u
	#define XEC_I2C_NFSM_HC_STATE_STA_PIN	3u
	#define XEC_I2C_NFSM_HC_STATE_WDATA	4u
	#define XEC_I2C_NFSM_HC_STATE_WPEC	5u
	#define XEC_I2C_NFSM_HC_STATE_RSTA	6u
	#define XEC_I2C_NFSM_HC_STATE_RSTA_PIN	7u
	#define XEC_I2C_NFSM_HC_STATE_RDN	8u
	#define XEC_I2C_NFSM_HC_STATE_RD_PEC	9u
	#define XEC_I2C_NFSM_HC_STATE_RPEC	10u
	#define XEC_I2C_NFSM_HC_STATE_PAUSE	11u
	#define XEC_I2C_NFSM_HC_STATE_STO	12u
	#define XEC_I2C_NFSM_HC_STATE_EOP	13u
	#define XEC_I2C_NFSM_HC_STATE_GET(r)	FIELD_GET(XEC_I2C_NFSM_HC_STATE_MSK, (r))
	#define XEC_I2C_NFSM_TC_STATE_POS	8
	#define XEC_I2C_NFSM_TC_STATE_MSK	GENMASK(15, 8)
	#define XEC_I2C_NFSM_TC_STATE_IDLE	0
	#define XEC_I2C_NFSM_TC_STATE_ADDR	1u
	#define XEC_I2C_NFSM_TC_STATE_WPIN	2u
	#define XEC_I2C_NFSM_TC_STATE_RPIN	3u
	#define XEC_I2C_NFSM_TC_STATE_WDATA	4u
	#define XEC_I2C_NFSM_TC_STATE_RDATA	5u
	#define XEC_I2C_NFSM_TC_STATE_RBE	6u
	#define XEC_I2C_NFSM_TC_STATE_RPEC	7u
	#define XEC_I2C_NFSM_TC_STATE_RPECRPT	8u
	#define XEC_I2C_NFSM_TC_STATE_GET(r)	FIELD_GET(XEC_I2C_NFSM_TC_STATE_MSK, (r))
	#define XEC_I2C_NFSM_FAIR_POS		16
	#define XEC_I2C_NFSM_FAIR_MSK		GENMASK(23, 16)
	#define XEC_I2C_NFSM_FAIR_IDLE		0
	#define XEC_I2C_NFSM_FAIR_BUSY		1u
	#define XEC_I2C_NFSM_FAIR_WIN		2u
	#define XEC_I2C_NFSM_FAIR_DLY		3u
	#define XEC_I2C_NFSM_FAIR_WAIT		4u
	#define XEC_I2C_NFSM_FAIR_WAIT_DONE	5u
	#define XEC_I2C_NFSM_FAIR_ACTIVE	6u

#define XEC_I2C_WKSR_OFS	0x60u /* wake status */
	#define XEC_I2C_WKSR_SB_POS	0 /* start bit detected R/W1C */

#define XEC_I2C_WKCR_OFS		0x64u /* wake control */
	#define XEC_I2C_WKCR_SBEN_POS		0 /* start bit detection interrupt enable */

#define XEC_I2C_MR1_OFS		0x68u /* MCHP reserved 1 */
#define XEC_I2C_IAS_OFS		0x6cu /* RO I2C address shadow */

#define XEC_I2C_PIS_OFS		0x70u /* I2C promiscuous mode interrupt status */
	/* I2C has captured 8 address bits and is stretching the clock
	 * software sets HW to (n)ACK
	 */
	#define XEC_I2C_PIS_CAP_POS		0

#define XEC_I2C_PIE_OFS		0x74u /* I2C promiscuous mode interrupt enable */
	#define XEC_I2C_PIE_CAP_POS		0

#define XEC_I2C_PCR_OFS		0x78u /* I2C promiscuous mode control */
	#define XEC_I2C_PCR_ACK_POS		0 /* write 1 to ACK the captured address */

#define XEC_I2C_IDS_OFS		0x7Cu /* RO I2C data shadow */

#endif /* ZEPHYR_DRIVERS_I2C_I2C_MCHP_XEC_REGS_H_ */
