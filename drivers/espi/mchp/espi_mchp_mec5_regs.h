/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ESPI_MCHP_MEC5_REGS_ESPI_H_
#define ZEPHYR_DRIVERS_ESPI_MCHP_MEC5_REGS_ESPI_H_

#include <zephyr/sys/util.h>

#define ESPI_GIRQ		19
#define ESPI_GIRQ_IDX		11
/* bit positions in GIRQ status, set enable, clear enable, and result registers */
#define ESPI_GIRQ_PC_POS	0
#define ESPI_GIRQ_BM1_POS	1
#define ESPI_GIRQ_BM2_POS	2
#define ESPI_GIRQ_LTR_POS	3
#define ESPI_GIRQ_OOB_UP_POS	4
#define ESPI_GIRQ_OOB_DN_POS	5
#define ESPI_GIRQ_FC_POS	6
#define ESPI_GIRQ_ERST_POS	7
#define ESPI_GIRQ_VW_CHEN_POS	8
#define ESPI_GIRQ_TAF_DONE_POS	9
#define ESPI_GIRQ_TAF_ERR_POS	10

#define ESPI_GIRQ_STS_ADDR	0x4000e0dcu
#define ESPI_GIRQ_ENSET_ADDR	0x4000e0e0u
#define ESPI_GIRQ_RESULT_ADDR	0x4000e0e4u
#define ESPI_GIRQ_ENCLR_ADDR	0x4000e0e8u

#define ESPI_GIRQ_VW_BANK0	24
#define ESPI_GIRQ_VW_BANK0_IDX	15
#define ESPI_GIRQ_VW_BANK1	25
#define ESPI_GIRQ_VW_BANK1_IDX	16

#define ESPI_GIRQ_VWB0_BASE	0x4000e140u
#define ESPI_GIRQ_VWB1_BASE	0x4000e154u

#define ESPI_GIRQ_STS_OFS	0
#define ESPI_GIRQ_ENSET_OFS	0x4u
#define ESPI_GIRQ_RESULT_OFS	0x8u
#define ESPI_GIRQ_ENCLR_OFS	0xcu

/* ---- eSPI IO component ---- */

/* Global configuration  */
#define ESPI_GC				0x2e0
#define ESPI_GC_ID_POS			0
#define ESPI_GC_CAP0_POS		8
#define ESPI_GC_CAP0_MSK		GENMASK(15, 8)
#define ESPI_GC_CAP0_SUPP_MSK		GENMASK(11, 8)
#define ESPI_GC_CAP0_PC_SUPP_POS	8
#define ESPI_GC_CAP0_VW_SUPP_POS	9
#define ESPI_GC_CAP0_OOB_SUPP_POS	10
#define ESPI_GC_CAP0_FC_SUPP_POS	11
#define ESPI_GC_CAP1_POS		16
#define ESPI_GC_CAP1_MSK		GENMASK(23, 16)
#define ESPI_GC_CAP1_MAX_FREQ_POS	16
#define ESPI_GC_CAP1_MAX_FREQ_MSK	GENMASK(18, 16)
#define ESPI_GC_CAP1_MAX_FREQ_20M	FIELD_PREP(ESPI_CAP1_MAX_FREQ_MSK, 0)
#define ESPI_GC_CAP1_MAX_FREQ_25M	FIELD_PREP(ESPI_CAP1_MAX_FREQ_MSK, 1)
#define ESPI_GC_CAP1_MAX_FREQ_33M	FIELD_PREP(ESPI_CAP1_MAX_FREQ_MSK, 2)
#define ESPI_GC_CAP1_MAX_FREQ_50M	FIELD_PREP(ESPI_CAP1_MAX_FREQ_MSK, 3)
#define ESPI_GC_CAP1_MAX_FREQ_66M	FIELD_PREP(ESPI_CAP1_MAX_FREQ_MSK, 4)
#define ESPI_GC_CAP1_ALERT_MODE_PIN_POS	19
#define ESPI_GC_CAP1_IOM_POS		20
#define ESPI_GC_CAP1_IOM_MSK		GENMASK(21, 20)
#define ESPI_GC_CAP1_IOM_S		FIELD_PREP(ESPI_CAP1_IOM_MSK, 0)
#define ESPI_GC_CAP1_IOM_SD		FIELD_PREP(ESPI_CAP1_IOM_MSK, 1)
#define ESPI_GC_CAP1_IOM_SQ		FIELD_PREP(ESPI_CAP1_IOM_MSK, 2)
#define ESPI_GC_CAP1_IOM_SDQ		FIELD_PREP(ESPI_CAP1_IOM_MSK, 3)
#define ESPI_GC_CAP1_ALERT_OD_CAP_POS	22
#define ESPI_GC_CAP1_ALERT_OD_SEL_POS	23
#define ESPI_GC_PC_POS			24
#define ESPI_GC_PC_MSK			GENMASK(31, 24)
#define ESPI_GC_PC_MPLS_POS		24
#define ESPI_GC_PC_MPLS_MSK		GENMASK(26, 24)
#define ESPI_GC_PC_MPLS_64B		FIELD_PREP(ESPI_CAP_PC_MPLS_MSK, 1)
#define ESPI_GC_PC_MPLS_DFLT		ESPI_CAP_PC_MPLS_64B

#define ESPI_CAP_VOF_RDY		0x2e4
#define ESPI_CAP_VOF_VW_POS		0
#define ESPI_CAP_VOF_VW_MSK		GENMASK(7, 0)
#define ESPI_CAP_VOF_VW_MCNT_POS	0
#define ESPI_CAP_VOF_VW_MCNT_MSK	GENMASK(5, 0)
#define ESPI_CAP_VOF_VW_MCNT_8		FIELD_PREP(ESPI_CAP_VW_MCNT_MSK, 7)
#define ESPI_CAP_VOF_VW_MCNT_64		FIELD_PREP(ESPI_CAP_VW_MCNT_MSK, 0x3F)
#define ESPI_CAP_VOF_VW_MCNT_DFLT	ESPI_CAP_VW_MCNT_64
#define ESPI_CAP_VOF_VW_MCNT(nvwg)	FIELD_PREP(ESPI_CAP_VW_MCNT_MSK, (nvwg))
#define ESPI_CAP_VOF_OOB_POS		8
#define ESPI_CAP_VOF_OOB_MPLS_POS	8
#define ESPI_CAP_VOF_OOB_MPLS_MSK	GENMASK(10, 8)
#define ESPI_CAP_VOF_OOB_MPLS_73	FIELD_PREP(ESPI_CAP_OOB_MPLS_MSK, 1)
#define ESPI_CAP_VOF_OOB_MPLS_DFLT	ESPI_CAP_OOB_MPLS_73
#define ESPI_CAP_VOF_FC_POS		16
#define ESPI_CAP_VOF_FC_MSK		GENMASK(23, 16)
#define ESPI_CAP_VOF_FC_MPLS_POS	16
#define ESPI_CAP_VOF_FC_MPLS_MSK	GENMASK(18, 16)
#define ESPI_CAP_VOF_FC_MPLS_64		FIELD_PREP(ESPI_CAP_FC_MPLS_MSK, 1)
#define ESPI_CAP_VOF_FC_SM_POS		19
#define ESPI_CAP_VOF_FC_SM_MSK		GENMASK(20, 16)
#define ESPI_CAP_VOF_FC_SM_CAF		FIELD_PREP(ESPI_CAP_FC_SM_MSK, 0)
#define ESPI_CAP_VOF_FC_SM_CAF_ALT	FIELD_PREP(ESPI_CAP_FC_SM_MSK, 1)
#define ESPI_CAP_VOF_FC_SM_TAF		FIELD_PREP(ESPI_CAP_FC_SM_MSK, 2)
#define ESPI_CAP_VOF_FC_SM_CAF_TAF	FIELD_PREP(ESPI_CAP_FC_SM_MSK, 3)
#define ESPI_CAP_VOF_FC_TAF_MPLS_POS	21
#define ESPI_CAP_VOF_FC_TAF_MPLS_MSK	GENMASK(23, 21)
#define ESPI_CAP_VOF_FC_TAF_MPLS_64	FIELD_PREP(ESPI_CAP_FC_TAF_MPLS_MSK, 1)
#define ESPI_CAP_VOF_PC_RDY_POS		24

/* OOB, Flash ready, and nESPI_RESET status/interrupt enable */
#define ESPI_OFR_RST			0x2e8
#define ESPI_OFR_RST_OOB_RDY_POS	0
#define ESPI_OFR_RST_FC_RDY_POS		8
#define ESPI_OFR_RST_SR_EDGE_POS	16 /* R/W1C nESPI_RESET edge detected */
#define ESPI_OFR_RST_SR_STATE_POS	17 /* RO current nESPI_RESET pin state */
#define ESPI_OFR_RST_IE_POS		24 /* nESPI_RESET edge detect interrupt enable */

/* nESPI_RESET as an 8-bit register */
#define ESPI_ERST_SR			0x2ea
#define ESPI_ERST_STS_CHG_POS		0
#define ESPI_ESRT_STS_STATE_POS		1

/* nESPI_RESET interrupt enable as an 8-bit register */
#define ESPI_ERST_IER			0x2eb
#define ESPI_ERST_IER_CHG_EN_POS	0

#define ESPI_VWC_TAF			0x2ec
#define ESPI_PLTRST_SRC_POS		0
#define ESPI_PLTRST_SRC_VW		0
#define ESPI_PLTRST_SRC_PIN		BIT(ESPI_PLTRST_SRC_POS)
#define ESPI_VW_RDY_POS			8
#define ESPI_TAF_EBS_POS		16
#define ESPI_TAF_EBS_1K_POS		16
#define ESPI_TAF_EBS_2K_POS		17
#define ESPI_TAF_EBS_4K_POS		18
#define ESPI_TAF_EBS_8K_POS		19
#define ESPI_TAF_EBS_16K_POS		20
#define ESPI_TAF_EBS_32K_POS		21
#define ESPI_TAF_EBS_64K_POS		22
#define ESPI_TAF_EBS_128K_POS		23

#define ESPI_TAF_RPMC_OP1_CFG		0x300
#define ESPI_TAF_OP1_NC			0x304

#define ESPI_ACTV			0x330
#define ESPI_ACTV_EN_POS		0

#define ESPI_HBV_IOC			0
#define ESPI_HBV_MC			1
#define ESPI_HBV_MBOX			2
#define ESPI_HBV_KBC			3
#define ESPI_HBV_AEC0			4
#define ESPI_HBV_AEC1			5
#define ESPI_HBV_AEC2			6
#define ESPI_HBV_AEC3			7
#define ESPI_HBV_AEC4			8
#define ESPI_HBV_APM1			9
#define ESPI_HBV_P92			10
#define ESPI_HBV_UART0			11
#define ESPI_HBV_UART1			12
#define ESPI_HBV_EMI0			13
#define ESPI_HBV_EMI1			14
#define ESPI_HBV_EMI2			15
#define ESPI_HBV_BDP0			16
#define ESPI_HBV_BDP0A			17
#define ESPI_HBV_RTC0			18
#define ESPI_HBV_UART2			21
#define ESPI_HBV_GL			22
#define ESPI_HBV_UART3			23

/* host I/O base address bits[15:0] in bits[31:16] and valid flag at bit[0] */
#define ESPI_HBV_OFS(n)			(0x334u + ((n) * 4u)
#define ESPI_HBV_BASE_POS		16
#define ESPI_HBV_BASE_MSK0		GENMASK(15, 0)
#define ESPI_HBV_BASE_MSK		GENMASK(31, 16)
#define ESPI_HBV_BASE_SET(b)		FIELD_PREP(ESPI_HBV_BASE_MSK, (b))
#define ESPI_HBV_BASE_GET(h)		FIELD_GET(ESPI_HBV_BASE_MSK, (b))
#define ESPI_HBV_VALID_EN_POS		0

#define SIRQ_IDX_MBOX_WR	0
#define SIRQ_IDX_MBOX_SMI	1
#define SIRQ_IDX_KBC_KIRQ	2
#define SIRQ_IDX_KBC_MIRQ	3
#define SIRQ_IDX_AEC0_OBF	4
#define SIRQ_IDX_AEC1_OBF	5
#define SIRQ_IDX_AEC2_OBF	6
#define SIRQ_IDX_AEC3_OBF	7
#define SIRQ_IDX_AEC4_OBF	8
#define SIRQ_IDX_UART0		9
#define SIRQ_IDX_UART1		10
#define SIRQ_IDX_EMI0_HE	11
#define SIRQ_IDX_EMI0_E2H	12
#define SIRQ_IDX_EMI1_HE	13
#define SIRQ_IDX_EMI1_E2H	14
#define SIRQ_IDX_EMI2_HE	15
#define SIRQ_IDX_EMI2_E2H	16
#define SIRQ_IDX_RTC		17
#define SIRQ_IDX_EC_IRQ		18
#define SIRQ_IDX_UART2		19
#define SIRQ_IDX_UART3		21

#define ESPI_SIRQ_OFS(n)	(0x3acu + (n))

/* ---- Peripheral channel ---- */
#define ESPI_PC_LC_ADDR_LSW		0x100
#define ESPI_PC_LC_ADDR_MSW		0x104

#define ESPI_PC_LC_LTT			0x108
#define ESPI_PC_LC_LTT_LEN_POS		0
#define ESPI_PC_LC_LTT_LEN_MSK		GENMASK(11, 0)
#define ESPI_PC_LC_LTT_TYPE_POS		12
#define ESPI_PC_LC_LTT_TYPE_MSK		GENMASK(19, 12)
#define ESPI_PC_LC_LTT_TAG_POS		20
#define ESPI_PC_LC_LTT_TAG_MSK		GENMASK(23, 20)

#define ESPI_PC_ERA_LSW			0x10cu
#define ESPI_PC_ERA_MSW			0x110u

#define ESPI_PC_SR			0x114u
#define ESPI_PC_ABERR_POS		16
#define ESPI_PC_CHEN_STATE_POS		24 /* RO */
#define ESPI_PC_CHEN_CHG_POS		25
#define ESPI_PC_BMEN_STATE_POS		27 /* RO */
#define ESPI_PC_BMEN_CHG_POS		28

#define ESPI_PC_IER			0x118u
#define ESPI_PC_IER_ABERR_POS		16
#define ESPI_PC_IER_CHEN_CHG_POS	25
#define ESPI_PC_IER_BMEN_CHG_POS	28

#define ESPI_PC_BAR_INH_LSW		0x120u
#define ESPI_PC_BAR_INH_MSW		0x124u

#define ESPI_PC_BAR_INIT		0x128u
#define ESPI_PC_BAR_INIT_MSK		GENMASK(15, 0)
#define ESPI_PC_BAR_INIT_DFLT		0x2eu

#define ESPI_PC_EC_SIRQ			0x12cu
#define ESPI_PC_EC_SIRQ_GEN_POS		0

#define ESPI_PC_LTR_SR			0x220u
#define ESPI_PC_LTR_TX_DONE_POS		0
#define ESPI_PC_LTR_START_OVR_POS	3
#define ESPI_PC_LTR_HOST_DIS_POS	4
#define ESPI_PC_LTR_TX_BUSY_POS		8 /* RO */

#define ESPI_PC_LTR_IER			0x224u
#define ESPI_PC_LTR_IER_TX_DONE_POS	0

#define ESPI_PC_LTR_CR			0x228u
#define ESPI_PC_LTR_CR_START_POS	0 /* WO */
#define ESPI_PC_LTR_CR_TAG_OUT_POS	8
#define ESPI_PC_LTR_CR_TAG_OUT_MSK	GENMASK(11, 8)
#define ESPI_PC_LTR_CR_TAG_OUT(t)	FIELD_PREP(ESPI_PC_LTR_CR_TAG_OUT_MSK, (t))

#define ESPI_PC_LTR_MSG			0x22cu
#define ESPI_PC_LTR_MSG_TV_POS		0 /* time value */
#define ESPI_PC_LTR_MSG_TV_MSK          GENMASK(9, 0)
#define ESPI_PC_LTR_MSG_TV(t)		FIELD_PREP(ESPI_PC_LTR_MSG_TV_MSK, (t))
#define ESPI_PC_LTR_MSG_SC_POS		10 /* scale field: time units */
#define ESPI_PC_LTR_MSG_SC_MSK		GENMASK(12, 10)
#define ESPI_PC_LTR_MSG_SC(sc)		FIELD_PREP(ESPI_PC_LTR_MSG_SC_MSK, (sc))
#define ESPI_PC_LTR_MSG_REQ_POS		15

/* ---- OOB channel ---- */

/* eSPI specification indicates maximum packet size for OOB channel is 64 bytes
 * Actually, the spec was modified to add 9 bytes to handle MCTP payloads.
 */
#define ESPI_OOB_ADDED_SIZE		9u

#define ESPI_OOB_RX_BA			0x240 /* OOB RX EC buffer address, b[1:0]=00b(RO) */
#define ESPI_OOB_TX_BA			0x248 /* OOB TX EC buffer address, b[1:0]=00b(RO) */

#define ESPI_OOB_RXL			0x250
#define ESPI_OOB_RXL_MLEN_POS		0
#define ESPI_OOB_RXL_MLEN_MSK		GENMASK(12, 0) /* number of bytes received (RO) */
#define ESPI_OOB_RXL_BLEN_POS		16
#define ESPI_OOB_RXL_BLEN_MSK		GENMASK(28, 16) /* max RX mesg length (RW) */

#define ESPI_OOB_TX_LEN			0x254
#define ESPI_OOB_TXL_MLEN_POS		0
#define ESPI_OOB_TXL_MLEN_MSK		GENMASK(12, 0) /* number of byte to transmit (RW) */

#define ESPI_OOB_RX_CR			0x258
#define ESPI_OOB_RX_CR_SRA_POS		0
#define ESPI_OOB_RX_CR_CHEN_POS		9 /* RO */
#define EPSI_OOB_MPLD_SZ_POS		16 /* RO */
#define ESPI_OOB_MPLD_SZ_MSK		GENMASK(18, 16)

#define ESPI_OOB_RX_IER			0x25c
#define ESPI_OOB_RX_IER_DONE_POS	0

#define ESPI_OOB_RX_SR			0x260
#define ESPI_OOB_RX_SR_DONE_POS		0
#define ESPI_OOB_RX_SR_ABERR_POS	1
#define ESPI_OOB_RX_SR_OVR_POS		2
#define ESPI_OOB_RX_SR_RXBA_POS		3 /* RO */
#define ESPI_OOB_RX_TAG_IN_POS          8
#define ESPI_OOB_RX_TAG_IN_MSK		GENMASK(11, 8)
#define ESPI_OOB_RX_TAG_IN(tv)		FIELD_GET(ESPI_OOB_RX_TAG_IN_MSK, (tv))

#define ESPI_OOB_TX_CR			0x264
#define ESPI_OOB_TX_CR_START_POS	0 /* WO */
#define ESPI_OOB_TX_TAG_POS		8
#define ESPI_OOB_TX_TAG_MSK		GENMASK(11, 8)
#define ESPI_OOB_TX_TAG(t)		FIELD_PREP(ESPI_OOB_TX_TAG_MSK, (t))

#define ESPI_OOB_TX_IER			0x268
#define ESPI_OOB_TX_IER_DONE_POS	0
#define ESPI_OOB_TX_IER_CENC_POS	1 /* OOB channel enable change interrupt enable */

#define ESPI_OOB_TX_SR			0x26c
#define ESPI_OOB_TX_SR_MSK		0x30fu
#define ESPI_OOB_TX_SR_DONE_POS		0
#define ESPI_OOB_TX_SR_CENC_POS		1
#define ESPI_OOB_TX_SR_ABERR_POS	2
#define ESPI_OOB_TX_SR_OVR_POS		3
#define ESPI_OOB_TX_SR_BAD_REQ_POS	5
#define ESPI_OOB_TX_SR_BUSY_POS		8 /* RO */
#define ESPI_OOB_TX_SR_CHEN_POS		9 /* RO image of OOB channel enable */

/* Flash channel  */
#define ESPI_FC_FA			0x280 /* Flash channel flash address register */

#define ESPI_FC_BA			0x288 /* Flash channel EC buffer address register */
#define ESPI_FC_BA_MSK			GENMASK(31, 2)

#define ESPI_FC_LEN			0x290
#define ESPI_FC_LEN_ERASE_VAL		0x01u

#define ESPI_FC_CR			0x294 /* R/W unless specified */
#define ESPI_FC_CR_START_POS		0 /* WO */
#define ESPI_FC_CR_FUNC_POS		2
#define ESPI_FC_CR_FUNC_MSK		GENMASK(3, 2)
#define ESPI_FC_CR_FUNC_READ		FIELD_PREP(ESPI_FC_FUNC_MSK, 0)
#define ESPI_FC_CR_FUNC_WRITE		FIELD_PREP(ESPI_FC_FUNC_MSK, 1)
#define ESPI_FC_CR_FUNC_ERASE_SM	FIELD_PREP(ESPI_FC_FUNC_MSK, 2)
#define ESPI_FC_CR_FUNC_ERASE_LG	FIELD_PREP(ESPI_FC_FUNC_MSK, 3)
#define ESPI_FC_CR_TAG_POS		4
#define ESPI_FC_CR_TAG_MSK		GENMASK(7, 4)
#define ESPI_FC_CR_TAG(t)		FIELD_PREP(ESPI_FC_CR_TAG_MSK, (t))
#define ESPI_FC_CR_ABORT_POS		16

#define ESPI_FC_IER                     0x298
#define ESPI_FC_IER_DONE_POS		0
#define ESPI_FC_IER_CHG_EN_POS		1

#define ESPI_FC_SR			0x2a0 /* RW/1C unless specified */
#define ESPI_FC_SR_CHEN_STATE_POS	0 /* RO */
#define ESPI_FC_SR_CHEN_CHG_POS		1
#define ESPI_FC_SR_DONE_POS		2
#define ESPI_FC_SR_DIS_WB_POS		3 /* Host disabled channel while it was busy */
#define ESPI_FC_SR_ABERR_POS		4
#define ESPI_FC_SR_ABORT_POS		5
#define ESPI_FC_SR_OVR_POS		6
#define ESPI_FC_SR_INC_POS              7
#define ESPI_FC_SR_FAIL_POS		8
#define ESPI_FC_SR_START_OVFL_POS	9
#define ESPI_FC_SR_BAD_REQ_POS		11

/* ---- Virtual Wire Channel  ---- */
#define ESPI_VW_SR			0x2b0 /* read-only */
#define ESPI_VW_SR_CHEN_POS		0 /* image of VW channel enable bit */

#define ESPI_VW_ERR_SR			0x3f0
#define ESPI_VW_ERR_FATAL_POS		0 /* read-only */
#define ESPI_VW_ERR_FATAL_CLR_POS	1 /* write-only */
#define ESPI_VW_ERR_NON_FATAL_POS	4 /* read-only */
#define ESPI_VW_ERR_NON_FATAL_CLR_POS	5 /* write-only */

/* ---- RPMC configuration ---- */
#define ESPI_RPMC_CFG1			0x300u
#define ESPI_RPMC_CFG1_C0_D040_POS	0
#define ESPI_RPMC_CFG1_C0_D848_POS	1
#define ESPI_RPMC_CFG1_C1_D040_POS	2
#define ESPI_RPMC_CFG1_C1_D848_POS	3
#define ESPI_RPMC_CFG1_C0_PNP_POS	4
#define ESPI_RPMC_CFG1_C1_PNP_POS	5
#define ESPI_RPMC_CFG1_NFL_POS		6
#define ESPI_RPMC_CFG1_NFL_MSK		GENMASK(7, 6)
#define ESPI_RPMC_CFG1_NFL_1		FIELD_PREP(ESPI_RPMC_OP1_CFG_NFL_MSK, 0)
#define ESPI_RPMC_CFG1_NFL_2		FIELD_PREP(ESPI_RPMC_OP1_CFG_NFL_MSK, 1)
#define ESPI_RPMC_CFG1_CT_POS		8 /* total number of counters */
#define ESPI_RPMC_CFG1_CT_MSK		GENMASK(13, 8)
#define ESPI_RPMC_CFG1_CT(ct)		FIELD_PREP(ESPI_RPMC_OP1_CFG_CT_MSK, (ct))
#define ESPI_RPMC_CFG1_STRICT_POS	31

#define ESPI_RPMC_CFG2			0x304u /* RPMC OP1 opcodes and number of counters per flash device */
#define ESPI_RPMC_CFG2_OPC0_POS		0 /* RPMC OP1 opcode for device connected to CS0 */
#define ESPI_RPMC_CFG2_OPC0_MSK		GENMASK(7, 0)
#define ESPI_RPMC_CFG2_OPC0(opc)	FIELD_PREP(ESPI_RPMC_CFG2_OPC0_MSK, (opc))
#define ESPI_RPMC_CFG2_CNTC0_POS	8
#define ESPI_RPMC_CFG2_CNTC0_MSK	GENMASK(12, 8)
#define ESPI_RPMC_CFG2_CNTC0(nc)	FIELD_PREP(ESPI_RPMC_CFG2_CNTC0_MSK, (nc))
#define ESPI_RPMC_CFG2_OPC1_POS		16 /* RPMC OP1 opcode for device connected to CS1 */
#define ESPI_RPMC_CFG2_OPC1_MSK		GENMASK(23, 16)
#define ESPI_RPMC_CFG2_OPC1(opc)	FIELD_PREP(ESPI_RPMC_CFG2_OPC1_MSK, (opc))
#define ESPI_RPMC_CFG2_CNTC1_POS	24
#define ESPI_RPMC_CFG2_CNTC1_MSK	GENMASK(28, 24)
#define ESPI_RPMC_CFG2_CNTC1(nc)	FIELD_PREP(ESPI_RPMC_CFG2_CNTC1_MSK, (nc))

/* ---- eSPI Memory component ---- */

/* Memory BAR's are 80-bit registers packed together.
 * We treat them as an array of five 16-bit registers.
 */
#define ESPI_MC_BAR_OFS			0x130u
#define ESPI_MC_BAR_CFG_OFS		0x330u

#define ESPI_MC_BAR_DEV_OFS(id, hw)	(ESPI_MC_BAR_OFS + ((uint32_t)(id) * 10u) + ((hw) * 2u))

/* Memory BAR configuration registers are 48-bit (R/W)
 * Contents are the BAR valid bit at position 0, and
 * bits[31:0] of the Host address at position 16.
 */
#define ESPI_MC_BAR_CFG_OFS		0x330u

#define ESPI_MC_MBOX_BAR_ID		0
#define ESPI_MC_AEC0_BAR_ID		1
#define ESPI_MC_AEC1_BAR_ID		2
#define ESPI_MC_AEC2_BAR_ID		3
#define ESPI_MC_AEC3_BAR_ID		4
#define ESPI_MC_AEC4_BAR_ID		5
#define ESPI_MC_EMI0_BAR_ID		6
#define ESPI_MC_EMI1_BAR_ID		7
#define ESPI_MC_EMI2_BAR_ID		8
#define ESPI_MC_MAX_BAR_ID		9

#define ESPI_MC_MBAR_CFG_VAL		0 /* half-word 0 bit 0 is the valid bit */
#define ESPI_MC_MBAR_CFG_HA0		2 /* half-word 2 is b[15:0] of host address */
#define ESPI_MC_MBAR_CFG_HA1		3 /* half-word 3 is b[31:16] of host address */

#define ESPI_MC_MBAR_CFG_OFS		0x330
#define ESPI_MC_MBAR_CFG_HW(id, hw)	\
	(ESPI_MC_MBAR_CFG_OFS + ((uint32_t)(id) * 10u) + ((hw) * 2u))

/* SRAM BAR (register accessible by EC only)
 * b[0] = valid
 * b[2:1] = access: none, RO, WO, R/W
 * b[7:4] = size: 1 byte to 32KB in powers of 2
 * b[47:16] = b[31:0] of host address aligned by size.
 */
#define ESPI_MC_SBAR0			0x1acu
#define ESPI_MC_SRAM1			0x1b6u

/* SRAM BAR Config (register accessible by EC and Host)
 * b[48:16] = Host address b[31:0]
 * b[7:4] = RO copy of SRAM region size
 * b[2:1] = RO copy of access: none, RO, WO, or R/W
 */
#define ESPI_MC_SBAR0_CFG		0x3acu
#define ESPI_MC_SBAR1_CFG		0x3b6u

/* eSPI memory and SRAM BAR host extended address registers
 * b[31:0] = host address [47:32]
 */
#define ESPI_MC_MBAR_HA_EXT		0x3a8u
#define ESPI_MC_SBAR_HA_EXT		0x3fcu

/* -------- VWire Component -------- */

#define ESPI_VW_GRP0			0
#define ESPI_VW_GRP1			1
#define ESPI_VW_GRP2			2
#define ESPI_VW_GRP3			3
#define ESPI_VW_GRP4			4
#define ESPI_VW_GRP5			5
#define ESPI_VW_GRP6			6
#define ESPI_VW_GRP7			7
#define ESPI_VW_GRP8			8
#define ESPI_VW_GRP9			9
#define ESPI_VW_GRP10			10
#define ESPI_VW_NGRPS			11

/* Host-to-Target virtual wires are groups of 4 vwires per eSPI specification.
 * MEC5 implements 11 CTVW groups. Each group is a 96-bit register.
 * Access is as three 32-bit registers.
 * b[31:0] contain Host index, reset source, and reset state
 * b[63:32] contain interrupt select for each of the 4 vwires in the group
 * b[95:64] contain current vwire states
 */
#define ESPI_HTVW_OFS			0
#define ESPI_HTVW_GRP(n)		(ESPI_HTVW_OFS + ((uint32_t)(n) * 12u))
#define ESPI_HTVW_GRPW(n, w)		(ESPI_HTVW_GRP(n) + ((uint32_t)(w) * 4u))

/* Target-to-Host virtual wires are groups of 4 vwires.
 * MEC5 implements 11 TCVW groups. Each group is a 64-bit register accessed
 * as two 32-bit registers.
 * b[31:0] contain Host index, reset source, reset state, and RO change status
 * b[63:32] contain current vwire states
 */
#define ESPI_THVW_OFS			0x200
#define ESPI_THVW_GRP(n)		(ESPI_THVW_OFS + ((uint32_t)(n) * 8u))
#define ESPI_THVW_GRPW(n, w)		(ESPI_THVW_GRP(n) + ((uint32_t)(w) * 4u))

/* Word 0 is the same for H2T and T2H vwire registers */
#define ESPI_VW_W0_HI_POS		0
#define ESPI_VW_W0_HI_MSK		GENMASK(7, 0)

#define ESPI_VW_W0_RSRC_POS		8
#define ESPI_VW_W0_RSRC_MSK		GENMASK(9, 8)
#define ESPI_VW_W0_RSRC_ESPI_RST	FIELD_PREP(ESPI_VW_W0_RSRC_MSK, 0)
#define ESPI_VW_W0_RSRC_SYS_RST		FIELD_PREP(ESPI_VW_W0_RSRC_MSK, 1)
#define ESPI_VW_W0_RSRC_SIO_RST		FIELD_PREP(ESPI_VW_W0_RSRC_MSK, 2)
#define ESPI_VW_W0_RSRC_PLT_RST		FIELD_PREP(ESPI_VW_W0_RSRC_MSK, 3)

#define ESPI_VW_W0_RSTATE_POS		12
#define ESPI_VW_W0_RSTATE_MSK		GENMASK(15, 12)

/* T2H word 0 bits[19:16] are read-only change bits for each VWire in the group.
 * If software changes the T2H Vwire state in word 1 bits[0, 8, 16, 24] the
 * corresponding bits[16, 17, 18, 19] of word 0 become 1. When the Host reads
 * the VWire group using the eSPI GET_VW command, hardware will clear the
 * read-only bits[16:19].
 */
#define ESPI_VW_T2H_W0_CHG_POS(s)	(16 + ((s) * 8))


/* H2T word 1 contains IRQ_SEL 4-bit fields in each byte
 * s is the source bit position in the VW group [0, 3]
 */
#define ESPI_VW_H2T_W1_ISEL_POS(s)	((s) * 8)
#define ESPI_VW_H2T_W1_ISEL_MSK(s)	\
	GENMASK(ESPI_VW_H2T_W1_ISEL_POS(s) + 3, ESPI_VW_H2T_W1_ISEL_POS(s))

/* H2T word 2 and T2H word 1 contain the current value of each of the
 * four VWires in the group. The bits are located in bit[0] of each byte.
 * s is the source bit position in the VW group [0, 3]
 */
#define ESPI_VW_STATE_POS(s)		((s) * 8)

#endif /*  ZEPHYR_DRIVERS_ESPI_MCHP_MEC5_REGS_ESPI_H_ */
