/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ESPI_MCHP_PCD_REGS_ESPI_H_
#define ZEPHYR_DRIVERS_ESPI_MCHP_PCD_REGS_ESPI_H_

#include <zephyr/sys/util.h>

/* Host visible logic devices configuration register offsets */
#define LD_CFG_ACTV_REG_OFS 0x330u
#define LD_CFG_ACTV_POS     0
#define LD_CFG_SEL_REG_OFS  0x3f0u

/* Mailbox 0 device */
#define MEC_MBOX0_GIRQ        15u
#define MEC_MBOX0_GIRQ_POS    20u
#define MEC_MBOX_H2EC_REG_OFS 0x100u

/* KBC0 8042 keyboard controller */
/* GIRQ15 OBE=bit[18], IBF=bit[19] */
#define MEC_KBC0_GIRQ         15u
#define MEC_KBC0_OBE_GIRQ_POS 18u
#define MEC_KBC0_IBF_GIRQ_POS 19u

#define MEC_KBC_H2EC_DCR_OFS  0x100u

#define MEC_KBC_SR_OFS        0x104u
#define MEC_KBC_SR_OBF_POS    0
#define MEC_KBC_SR_IBF_POS    1
#define MEC_KBC_SR_UD0_POS    2
#define MEC_KBC_SR_CMD_POS    3
#define MEC_KBC_SR_UD1_POS    4
#define MEC_KBC_SR_AUXOBF_POS 5
#define MEC_KBC_SR_UD2_POS    6
#define MEC_KBC_SR_UD2_MSK    GENMASK(7, 6)

#define MEC_KBC_CR_OFS         0x108u
#define MEC_KBC_CR_UD3_POS     0
#define MEC_KBC_CR_SAEN_POS    1
#define MEC_KBC_CR_PCOBFEN_POS 2
#define MEC_KBC_CR_UD4_POS     3
#define MEC_KBC_CR_UD4_MSK     GENMASK(4, 3)
#define MEC_KBC_CR_OBFEN_POS   5
#define MEC_KBC_CR_UD5_POS     6
#define MEC_KBC_CR_AUXH_POS    7

#define MEC_KBC_AUXD_OFS 0x10cu

#define MEC_KBC_PCOBF_OFS      0x114u
#define MEC_KBC_PCOBFG_POS     0

/* ACPI EC registers */
#define MEC_AEC_E2H_DATA0_OFS 0x100u /* 4-byte mode disabled: writes by EC set OBF */
#define MEC_AEC_E2H_DATA1_OFS 0x101u
#define MEC_AEC_E2H_DATA2_OFS 0x102u
#define MEC_AEC_E2H_DATA3_OFS 0x103u /* 4-byte mode enabled: writes by EC set OBF */
#define MEC_AEC_SR_OFS        0x104u
#define MEC_AEC_SR_OBF_POS    0 /* RO */
#define MEC_AEC_SR_IBF_POS    1 /* RO */
#define MEC_AEC_SR_UD1A_POS   2 /* RW */
#define MEC_AEC_SR_CMD_POS    3 /* RO */
#define MEC_AEC_SR_BURST_POS  4 /* RW */
#define MEC_AEC_SR_SCI_POS    5 /* RW */
#define MEC_AEC_SR_SMI_POS    6 /* RW */
#define MEC_AEC_SR_UD0A_POS   7 /* RW */
#define MEC_AEC_BC_OFS        0x105u
#define MEC_AEC_BC_4BEN_POS   0 /* enable 4-byte mode */
#define MEC_AEC_H2E_DATA0_OFS 0x108u /* 4-byte mode disabled: read by EC clears IBF */
#define MEC_AEC_H2E_DATA1_OFS 0x109u
#define MEC_AEC_H2E_DATA2_OFS 0x10au
#define MEC_AEC_H2E_DATA3_OFS 0x10bu /* 4-byte mode enabled: read by EC clears IBF */

/* EMI registers */
#define MEC_EMI_H2EC_OFS		0x100u
#define MEC_EMI_EC2H_OFS		0x101u
#define MEC_EMI_BA_MR0_OFS		0x104u
#define MEC_EMI_RWL_MR0_OFS		0x108u
#define MEC_EMI_BA_MR1_OFS		0x10cu
#define MEC_EMI_RWL_MR1_OFS		0x110u
#define MEC_EMI_ISET_OFS		0x114u
#define MEC_EMI_HCLR_OFS		0x116u
#define MEC_EMI_HCLR_EN_OFS		0x120u

/* Glue Logic: S0ix PM hardware support */
#define MEC_PC_GL_S0IX_DET_REG_OFS 0x04u
#define MEC_PC_GL_S0IX_DET_EN_POS  0


#endif /* ZEPHYR_DRIVERS_ESPI_MCHP_PCD_REGS_ESPI_H_ */
