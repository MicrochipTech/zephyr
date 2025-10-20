/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ESPI_MCHP_MCHP_XEC_ESPI_PCD_REGS_H_
#define ZEPHYR_DRIVERS_ESPI_MCHP_MCHP_XEC_ESPI_PCD_REGS_H_

#include <zephyr/sys/util.h>

/* Host visible logic devices configuration register offsets */
#define XEC_LD_CFG_ACTV_REG_OFS 0x330u
#define XEC_LD_CFG_ACTV_POS     0
#define XEC_LD_CFG_SEL_REG_OFS  0x3f0u

/* Mailbox 0 device */
#define XEC_MBOX0_GIRQ        15u
#define XEC_MBOX0_GIRQ_POS    20u
#define XEC_MBOX_H2EC_REG_OFS 0x100u

/* KBC0 8042 keyboard controller */
/* GIRQ15 OBE=bit[18], IBF=bit[19] */
#define XEC_KBC0_GIRQ         15u
#define XEC_KBC0_OBE_GIRQ_POS 18u
#define XEC_KBC0_IBF_GIRQ_POS 19u

#define XEC_KBC_H2EC_DCR_OFS  0x100u

#define XEC_KBC_SR_OFS        0x104u
#define XEC_KBC_SR_OBF_POS    0
#define XEC_KBC_SR_IBF_POS    1
#define XEC_KBC_SR_UD0_POS    2
#define XEC_KBC_SR_CMD_POS    3
#define XEC_KBC_SR_UD1_POS    4
#define XEC_KBC_SR_AUXOBF_POS 5
#define XEC_KBC_SR_UD2_POS    6
#define XEC_KBC_SR_UD2_MSK    GENMASK(7, 6)

#define XEC_KBC_CR_OFS         0x108u
#define XEC_KBC_CR_UD3_POS     0
#define XEC_KBC_CR_SAEN_POS    1
#define XEC_KBC_CR_PCOBFEN_POS 2
#define XEC_KBC_CR_UD4_POS     3
#define XEC_KBC_CR_UD4_MSK     GENMASK(4, 3)
#define XEC_KBC_CR_OBFEN_POS   5
#define XEC_KBC_CR_UD5_POS     6
#define XEC_KBC_CR_AUXH_POS    7

#define XEC_KBC_AUXD_OFS 0x10cu

#define XEC_KBC_PCOBF_OFS      0x114u
#define XEC_KBC_PCOBFG_POS     0

/* ACPI EC registers */
#define XEC_AEC_E2H_DATA0_OFS 0x100u /* 4-byte mode disabled: writes by EC set OBF */
#define XEC_AEC_E2H_DATA1_OFS 0x101u
#define XEC_AEC_E2H_DATA2_OFS 0x102u
#define XEC_AEC_E2H_DATA3_OFS 0x103u /* 4-byte mode enabled: writes by EC set OBF */
#define XEC_AEC_SR_OFS        0x104u
#define XEC_AEC_SR_OBF_POS    0 /* RO */
#define XEC_AEC_SR_IBF_POS    1 /* RO */
#define XEC_AEC_SR_UD1A_POS   2 /* RW */
#define XEC_AEC_SR_CMD_POS    3 /* RO */
#define XEC_AEC_SR_BURST_POS  4 /* RW */
#define XEC_AEC_SR_SCI_POS    5 /* RW */
#define XEC_AEC_SR_SMI_POS    6 /* RW */
#define XEC_AEC_SR_UD0A_POS   7 /* RW */
#define XEC_AEC_BC_OFS        0x105u
#define XEC_AEC_BC_4BEN_POS   0 /* enable 4-byte mode */
#define XEC_AEC_H2E_DATA0_OFS 0x108u /* 4-byte mode disabled: read by EC clears IBF */
#define XEC_AEC_H2E_DATA1_OFS 0x109u
#define XEC_AEC_H2E_DATA2_OFS 0x10au
#define XEC_AEC_H2E_DATA3_OFS 0x10bu /* 4-byte mode enabled: read by EC clears IBF */

/* EMI registers */
#define XEC_EMI_H2EC_OFS		0x100u
#define XEC_EMI_EC2H_OFS		0x101u
#define XEC_EMI_BA_MR0_OFS		0x104u
#define XEC_EMI_RWL_MR0_OFS		0x108u
#define XEC_EMI_BA_MR1_OFS		0x10cu
#define XEC_EMI_RWL_MR1_OFS		0x110u
#define XEC_EMI_ISET_OFS		0x114u
#define XEC_EMI_HCLR_OFS		0x116u
#define XEC_EMI_HCLR_EN_OFS		0x120u

/* Glue Logic: S0ix PM hardware support */
#define XEC_PC_GL_S0IX_DET_REG_OFS 0x04u
#define XEC_PC_GL_S0IX_DET_EN_POS  0

/* BIOS Debug Port (BDP) */
#define XEC_BDP_HDATA_OFS		0 /* 32-bit WO */

#define XEC_BDP_DA_OFS			0x100u /* data and attributes (RO) */
#define XEC_BDP_DA_DATA_POS		0
#define XEC_BDP_DA_DATA_MSK		GENMASK(7, 0)
#define XEC_BDP_DA_DATA_SET(d)		FIELD_PREP(XEC_BDP_DA_DATA_MSK)
#define XEC_BDP_DA_DATA_GET(d)		FIELD_GET(XEC_BDP_DA_DATA_MSK)
#define XEC_BDP_DA_LANE_POS		8
#define XEC_BDP_DA_LANE_MSK		GENMASK(9, 8)
#define XEC_BDP_DA_LANE_SET(lane)	FIELD_PREP(XEC_BDP_DA_LANE_MSK, (lane))
#define XEC_BDP_DA_LANE_GET(lane)	FIELD_GET(XEC_BDP_DA_LANE_MSK, (lane))
#define XEC_BDP_DA_LEN_POS		10
#define XEC_BDP_DA_LEN_MSK		GENMASK(11, 10)
#define XEC_BDP_DA_LEN_1C		0
#define XEC_BDP_DA_LEN_1_OF_2		1u
#define XEC_BDP_DA_LEN_1_OF_4		2u
#define XEC_BDP_DA_LEN_ORPHAN		3u
#define XEC_BDP_DA_LEN_SET(len)		FIELD_PREP(XEC_BDP_DA_LEN_MSK, (len))
#define XEC_BDP_DA_LEN_GET(len)		FIELD_GET(XEC_BDP_DA_LEN_MSK, (len))
#define XEC_BDP_DA_NE_POS		12u /* not empty */
#define XEC_BDP_DA_OVR_POS		13u /* overrun */
#define XEC_BDP_DA_THR_POS		18u /* FIFO data at or above threshold */

#define XEC_BDP_CFG_OFS			0x104u
#define XEC_BDP_CFG_FF_POS		0 /* fifo flush (WO) */
#define XEC_BDP_CFG_SC_POS		1 /* snapshot clear (WO) */
#define XEC_BDP_CFG_THR_POS		8
#define XEC_BDP_CFG_THR_MSK		GENMASK(10, 8)
#define XEC_BDP_CFG_THR_1		0
#define XEC_BDP_CFG_THR_4		1u
#define XEC_BDP_CFG_THR_8		2u
#define XEC_BDP_CFG_THR_16		3u
#define XEC_BDP_CFG_THR_20		4u
#define XEC_BDP_CFG_THR_24		5u
#define XEC_BDP_CFG_THR_28		6u
#define XEC_BDP_CFG_THR_30		7u
#define XEC_BDP_CFG_THR_SET(t)		FIELD_PREP(XEC_BDP_CFG_THR_MSK, (t))
#define XEC_BDP_CFG_THR_GET(t)		FIELD_GET(XEC_BDP_CFG_THR_MSK, (t))
#define XEC_BDP_CFG_SRST_POS		31 /* soft reset (WO) */

#define XEC_BDP_SR_OFS			0x108u /* 8-bit RO */
#define XEC_BDP_SR_NE_POS		0
#define XEC_BDP_SR_OVR_POS		1
#define XEC_BDP_SR_THR_POS		2

#define XEC_BDP_IER_OFS			0x109u /* 8-bit */
#define XEC_BDP_IER_THR_POS		0

#define XEC_BDP_SNAP_OFS		0x10cu /* 32-bit RO */

#define XEC_BDP_ACTV_OFS		0x330u
#define XEC_BDP_ACTV_EN_POS		0

#define XEC_BDP_ALIAS_HDATA_OFS		0x400u /* 32-bit WO */
#define XEC_BDP_ALIAS_ACTV_OFS		0x730u
#define XEC_BDP_ALIAS_BL_OFS		0x7f0u
#define XEC_BDP_ALIAS_BL_LANE_POS	0
#define XEC_BDP_ALIAS_BL_LANE_MSK	GENMASK(1, 0)
#define XEC_BDP_ALIAS_BL_LANE_0		0
#define XEC_BDP_ALIAS_BL_LANE_1		1u
#define XEC_BDP_ALIAS_BL_LANE_2		2u
#define XEC_BDP_ALIAS_BL_LANE_3		3u
#define XEC_BDP_ALIAS_BL_SET(lane)	FIELD_PREP(XEC_BDP_ALIAS_BL_LANE_MSK, (lane))
#define XEC_BDP_ALIAS_BL_GET(lane)	FIELD_GET(XEC_BDP_ALIAS_BL_LANE_MSK, (lane))

#endif /* ZEPHYR_DRIVERS_ESPI_MCHP_MCHP_XEC_ESPI_PCD_REGS_H_ */
