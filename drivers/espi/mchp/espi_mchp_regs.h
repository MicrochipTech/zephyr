/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ESPI_MCHP_REGS_ESPI_H_
#define ZEPHYR_DRIVERS_ESPI_MCHP_REGS_ESPI_H_

#include <zephyr/sys/util.h>

#define MEC_ESPI_GIRQ              19
#define MEC_ESPI_GIRQ_IDX          11
/* bit positions in GIRQ status, set enable, clear enable, and result registers */
#define MEC_ESPI_GIRQ_PC_POS       0
#define MEC_ESPI_GIRQ_BM1_POS      1
#define MEC_ESPI_GIRQ_BM2_POS      2
#define MEC_ESPI_GIRQ_LTR_POS      3
#define MEC_ESPI_GIRQ_OOB_UP_POS   4
#define MEC_ESPI_GIRQ_OOB_DN_POS   5
#define MEC_ESPI_GIRQ_FC_POS       6
#define MEC_ESPI_GIRQ_ERST_POS     7
#define MEC_ESPI_GIRQ_VW_CHEN_POS  8
#define MEC_ESPI_GIRQ_TAF_DONE_POS 9
#define MEC_ESPI_GIRQ_TAF_ERR_POS  10

#define MEC_ESPI_GIRQ_STS_ADDR    0x4000e0dcu
#define MEC_ESPI_GIRQ_ENSET_ADDR  0x4000e0e0u
#define MEC_ESPI_GIRQ_RESULT_ADDR 0x4000e0e4u
#define MEC_ESPI_GIRQ_ENCLR_ADDR  0x4000e0e8u

#define MEC_ESPI_GIRQ_VW_BANK0     24
#define MEC_ESPI_GIRQ_VW_BANK0_IDX 15
#define MEC_ESPI_GIRQ_VW_BANK1     25
#define MEC_ESPI_GIRQ_VW_BANK1_IDX 16

#define MEC_ESPI_GIRQ_VWB0_BASE 0x4000e140u
#define MEC_ESPI_GIRQ_VWB1_BASE 0x4000e154u

#define MEC_ESPI_GIRQ_STS_OFS    0
#define MEC_ESPI_GIRQ_ENSET_OFS  0x4u
#define MEC_ESPI_GIRQ_RESULT_OFS 0x8u
#define MEC_ESPI_GIRQ_ENCLR_OFS  0xcu

/* ---- eSPI IO component ---- */

/* Global configuration  */
#define MEC_ESPI_GC_OFS                     0x2e0u
#define MEC_ESPI_GC_ID_POS                  0

#define MEC_ESPI_CAP0_OFS                   0x2e1u
#define MEC_ESPI_CAP0_SUPP_MSK              GENMASK(3, 0)
#define MEC_ESPI_CAP0_PC_SUPP_POS           0
#define MEC_ESPI_CAP0_VW_SUPP_POS           1
#define MEC_ESPI_CAP0_OOB_SUPP_POS          2
#define MEC_ESPI_CAP0_FC_SUPP_POS           3

#define MEC_ESPI_CAP1_OFS                   0x2e2u
#define MEC_ESPI_CAP1_MAX_FREQ_POS          0
#define MEC_ESPI_CAP1_MAX_FREQ_MSK          GENMASK(2, 0)
#define MEC_ESPI_CAP1_MAX_FREQ_20M          FIELD_PREP(MEC_ESPI_CAP1_MAX_FREQ_MSK, 0)
#define MEC_ESPI_CAP1_MAX_FREQ_25M          FIELD_PREP(MEC_ESPI_CAP1_MAX_FREQ_MSK, 1)
#define MEC_ESPI_CAP1_MAX_FREQ_33M          FIELD_PREP(MEC_ESPI_CAP1_MAX_FREQ_MSK, 2)
#define MEC_ESPI_CAP1_MAX_FREQ_50M          FIELD_PREP(MEC_ESPI_CAP1_MAX_FREQ_MSK, 3)
#define MEC_ESPI_CAP1_MAX_FREQ_66M          FIELD_PREP(MEC_ESPI_CAP1_MAX_FREQ_MSK, 4)
#define MEC_ESPI_CAP1_MAX_FREQ_SET(f)       FIELD_PREP(MEC_ESPI_CAP1_MAX_FREQ_MSK, (f))
#define MEC_ESPI_CAP1_MAX_FREQ_GET(f)       FIELD_GET(MEC_ESPI_CAP1_MAX_FREQ_MSK, (f))
#define MEC_ESPI_CAP1_ALERT_MODE_PIN_POS    3
#define MEC_ESPI_CAP1_IOM_POS               4
#define MEC_ESPI_CAP1_IOM_MSK               GENMASK(5, 4)
#define MEC_ESPI_CAP1_IOM_S_VAL             0
#define MEC_ESPI_CAP1_IOM_SD_VAL            1
#define MEC_ESPI_CAP1_IOM_SQ_VAL            2
#define MEC_ESPI_CAP1_IOM_SDQ_VAL           3
#define MEC_ESPI_CAP1_IOM_S                 FIELD_PREP(MEC_ESPI_CAP1_IOM_MSK, 0)
#define MEC_ESPI_CAP1_IOM_SD                FIELD_PREP(MEC_ESPI_CAP1_IOM_MSK, 1)
#define MEC_ESPI_CAP1_IOM_SQ                FIELD_PREP(MEC_ESPI_CAP1_IOM_MSK, 2)
#define MEC_ESPI_CAP1_IOM_SDQ               FIELD_PREP(MEC_ESPI_CAP1_IOM_MSK, 3)
#define MEC_ESPI_CAP1_IOM_SET(iom)          FIELD_PREP(MEC_ESPI_CAP1_IOM_MSK, (iom))
#define MEC_ESPI_CAP1_IOM_GET(iom)          FIELD_GET(MEC_ESPI_CAP1_IOM_MSK, (iom))
#define MEC_ESPI_CAP1_ALERT_OD_CAP_POS      6
#define MEC_ESPI_CAP1_ALERT_OD_SEL_POS      7

#define MEC_ESPI_CAP_PC_OFS                 0x2e3u
#define MEC_ESPI_CAP_PC_MPLS_POS            0
#define MEC_ESPI_CAP_PC_MPLS_MSK            GENMASK(2, 0)
#define MEC_ESPI_CAP_PC_MPLS_64B            FIELD_PREP(MEC_ESPI_CAP_PC_MPLS_MSK, 1)
#define MEC_ESPI_CAP_PC_MPLS_DFLT           MEC_ESPI_CAP_PC_MPLS_64B
#define MEC_ESPI_CAP_PC_MPLS_SET(n)         FIELD_PREP(MEC_ESPI_CAP_PC_MPLS_MSK, (n))
#define MEC_ESPI_CAP_PC_MPLS_GET(n)         FIELD_GET(MEC_ESPI_CAP_PC_MPLS_MSK, (n))

#define MEC_ESPI_CAP_VW_OFS                0x2e4u
#define MEC_ESPI_CAP_VW_MAX_GRPS_POS       0
#define MEC_ESPI_CAP_VW_MAX_GRPS_MSK       GENMASK(5, 0)
#define MEC_ESPI_CAP_VW_MAX_GRPS_8         FIELD_PREP(MEC_ESPI_CAP_VW_MAX_GRPS_MSK, 7)
#define MEC_ESPI_CAP_VW_MAX_GRPS_64        FIELD_PREP(MEC_ESPI_CAP_VW_MAX_GRPS_MSK, 0x3F)
#define MEC_ESPI_CAP_VW_MAX_GRPS_DFLT      MEC_ESPI_CAP_VW_MAX_GRPS_64
#define MEC_ESPI_CAP_VW_MAX_GRPS_SET(nvwg) FIELD_PREP(MEC_ESPI_CAP_VW_MAX_GRPS_MSK, (nvwg))
#define MEC_ESPI_CAP_VW_MAX_GRPS_GET(nvwg) FIELD_GET(MEC_ESPI_CAP_VW_MAX_GRPS_MSK, (nvwg))

#define MEC_ESPI_CAP_OOB_OFS            0x2e5u
#define MEC_ESPI_CAP_OOB_MPLD_POS       0
#define MEC_ESPI_CAP_OOB_MPLD_MSK       GENMASK(2, 0)
#define MEC_ESPI_CAP_OOB_MPLD_73        FIELD_PREP(MEC_ESPI_CAP_OOB_MPLS_MSK, 1)
#define MEC_ESPI_CAP_OOB_MPLD_DFLT      MEC_ESPI_CAP_OOB_MPLD_73
#define MEC_ESPI_CAP_OOB_MPLD_SET(v)    FIELD_PREP(MEC_ESPI_CAP_OOB_MPLD_MSK, (v))
#define MEC_ESPI_CAP_OOB_MPLD_GET(v)    FIELD_GET(MEC_ESPI_CAP_OOB_MPLD_MSK, (v))

#define MEC_ESPI_CAP_FC_OFS             0x2e6u
#define MEC_ESPI_CAP_FC_MPLD_POS        0
#define MEC_ESPI_CAP_FC_MPLD_MSK        GENMASK(2, 0)
#define MEC_ESPI_CAP_FC_MPLD_64         FIELD_PREP(MEC_ESPI_CAP_FC_MPLD_MSK, 1)
#define MEC_ESPI_CAP_FC_MPLD_SET(n)     FIELD_PREP(MEC_ESPI_CAP_FC_MPLD_MSK, (n))
#define MEC_ESPI_CAP_FC_MPLD_GET(n)     FIELD_GET(MEC_ESPI_CAP_FC_MPLD_MSK, (n))

#define MEC_ESPI_CAP_FC_SM_POS          3
#define MEC_ESPI_CAP_FC_SM_MSK          GENMASK(4, 3)
#define MEC_ESPI_CAP_FC_SM_CAF          FIELD_PREP(MEC_ESPI_CAP_FC_SM_MSK, 0)
#define MEC_ESPI_CAP_FC_SM_CAF_ALT      FIELD_PREP(MEC_ESPI_CAP_FC_SM_MSK, 1)
#define MEC_ESPI_CAP_FC_SM_TAF          FIELD_PREP(MEC_ESPI_CAP_FC_SM_MSK, 2)
#define MEC_ESPI_CAP_FC_SM_CAF_TAF      FIELD_PREP(MEC_ESPI_CAP_FC_SM_MSK, 3)
#define MEC_ESPI_CAP_FC_SM_SET(sm)      FIELD_PREP(MEC_ESPI_CAP_FC_SM_MSK, (sm))
#define MEC_ESPI_CAP_FC_SM_GET(sm)      FIELD_GET(MEC_ESPI_CAP_FC_SM_MSK, (sm))

/* TAF maximum read request size */
#define MEC_ESPI_CAP_FC_TAF_MRRQ_POS    5
#define MEC_ESPI_CAP_FC_TAF_MRRQ_MSK    GENMASK(7, 5)
#define MEC_ESPI_CAP_FC_TAF_MRRQ_64     FIELD_PREP(MEC_ESPI_CAP_FC_TAF_MRRQ_MSK, 1)
#define MEC_ESPI_CAP_FC_TAF_MRRQ_SET(n) FIELD_PREP(MEC_ESPI_CAP_FC_TAF_MRRQ_MSK, (n))
#define MEC_ESPI_CAP_FC_TAF_MRRQ_GET(n) FIELD_GET(MEC_ESPI_CAP_FC_TAF_MRRQ_MSK, (n))

#define MEC_ESPI_PC_RDY_OFS             0x2e7u
#define MEC_ESPI_OOB_RDY_OFS            0x2e8u
#define MEC_ESPI_FC_RDY_OFS             0x2e9u
#define MEC_ESPI_VW_RDY_OFS             0x2edu
#define MEC_ESPI_CHAN_RDY_POS           0

/* nESPI_RESET status and interrupt enable */
#define MEC_ESPI_RESET_SR_OFS          0x2eau
#define MEC_ESPI_RESET_SR_CHG_POS      0 /* RW1C */
#define MEC_ESPI_RESET_SR_STATE_POS    1 /* RO */

#define MEC_ESPI_RESET_IER_OFS         0x2ebu
#define MEC_ESPI_RESET_IER_EN_POS      0

#define MEC_ESPI_PLTRST_SRC_OFS        0x2ecu
#define MEC_ESPI_PLTRST_SRC_POS        0
#define MEC_ESPI_PLTRST_SRC_VW         0
#define MEC_ESPI_PLTRST_SRC_PIN        BIT(MEC_ESPI_PLTRST_SRC_POS)

#define MEC_ESPI_TAF_EBSZ_OFS          0x2edu
#define MEC_ESPI_TAF_EBS_1K_POS        0
#define MEC_ESPI_TAF_EBS_2K_POS        1
#define MEC_ESPI_TAF_EBS_4K_POS        2
#define MEC_ESPI_TAF_EBS_8K_POS        3
#define MEC_ESPI_TAF_EBS_16K_POS       4
#define MEC_ESPI_TAF_EBS_32K_POS       5
#define MEC_ESPI_TAF_EBS_64K_POS       6
#define MEC_ESPI_TAF_EBS_128K_POS      7

/* ---- RPMC configuration HW version 1.5 only  ---- */
#define MEC_ESPI_RPMC_CFG1_OFS         0x300u
#define MEC_ESPI_RPMC_CFG1_C0_D040_POS 0
#define MEC_ESPI_RPMC_CFG1_C0_D848_POS 1
#define MEC_ESPI_RPMC_CFG1_C1_D040_POS 2
#define MEC_ESPI_RPMC_CFG1_C1_D848_POS 3
#define MEC_ESPI_RPMC_CFG1_C0_PNP_POS  4
#define MEC_ESPI_RPMC_CFG1_C1_PNP_POS  5
#define MEC_ESPI_RPMC_CFG1_NFL_POS     6
#define MEC_ESPI_RPMC_CFG1_NFL_MSK     GENMASK(7, 6)
#define MEC_ESPI_RPMC_CFG1_NFL_1       FIELD_PREP(MEC_ESPI_RPMC_OP1_CFG_NFL_MSK, 0)
#define MEC_ESPI_RPMC_CFG1_NFL_2       FIELD_PREP(MEC_ESPI_RPMC_OP1_CFG_NFL_MSK, 1)
#define MEC_ESPI_RPMC_CFG1_CT_POS      8 /* total number of counters */
#define MEC_ESPI_RPMC_CFG1_CT_MSK      GENMASK(13, 8)
#define MEC_ESPI_RPMC_CFG1_CT(ct)      FIELD_PREP(MEC_ESPI_RPMC_OP1_CFG_CT_MSK, (ct))
#define MEC_ESPI_RPMC_CFG1_STRICT_POS  31

/* RPMC OP1 opcodes and number of counters per flash device */
#define MEC_ESPI_RPMC_CFG2_OFS       0x304u
#define MEC_ESPI_RPMC_CFG2_OPC0_POS  0      /* RPMC OP1 opcode for device connected to CS0 */
#define MEC_ESPI_RPMC_CFG2_OPC0_MSK  GENMASK(7, 0)
#define MEC_ESPI_RPMC_CFG2_OPC0(opc) FIELD_PREP(MEC_ESPI_RPMC_CFG2_OPC0_MSK, (opc))
#define MEC_ESPI_RPMC_CFG2_CNTC0_POS 8
#define MEC_ESPI_RPMC_CFG2_CNTC0_MSK GENMASK(12, 8)
#define MEC_ESPI_RPMC_CFG2_CNTC0(nc) FIELD_PREP(MEC_ESPI_RPMC_CFG2_CNTC0_MSK, (nc))
#define MEC_ESPI_RPMC_CFG2_OPC1_POS  16 /* RPMC OP1 opcode for device connected to CS1 */
#define MEC_ESPI_RPMC_CFG2_OPC1_MSK  GENMASK(23, 16)
#define MEC_ESPI_RPMC_CFG2_OPC1(opc) FIELD_PREP(MEC_ESPI_RPMC_CFG2_OPC1_MSK, (opc))
#define MEC_ESPI_RPMC_CFG2_CNTC1_POS 24
#define MEC_ESPI_RPMC_CFG2_CNTC1_MSK GENMASK(28, 24)
#define MEC_ESPI_RPMC_CFG2_CNTC1(nc) FIELD_PREP(MEC_ESPI_RPMC_CFG2_CNTC1_MSK, (nc))

#define MEC_ESPI_ACTV_OFS            0x330
#define MEC_ESPI_ACTV_EN_POS         0

/* eSPI I/O Base Address registers for x86 I/O mapped devices */
#define MEC_ESPI_IO_HBV_IOC   0
#define MEC_ESPI_IO_HBV_MC    1
#define MEC_ESPI_IO_HBV_MBOX  2
#define MEC_ESPI_IO_HBV_KBC   3
#define MEC_ESPI_IO_HBV_AEC0  4
#define MEC_ESPI_IO_HBV_AEC1  5
#define MEC_ESPI_IO_HBV_AEC2  6
#define MEC_ESPI_IO_HBV_AEC3  7
#define MEC_ESPI_IO_HBV_AEC4  8
#define MEC_ESPI_IO_HBV_APM1  9
#define MEC_ESPI_IO_HBV_FKB   10
#define MEC_ESPI_IO_HBV_UART0 11
#define MEC_ESPI_IO_HBV_UART1 12
#define MEC_ESPI_IO_HBV_EMI0  13
#define MEC_ESPI_IO_HBV_EMI1  14
#define MEC_ESPI_IO_HBV_EMI2  15
#define MEC_ESPI_IO_HBV_BDP0  16
#define MEC_ESPI_IO_HBV_BDP0A 17
#define MEC_ESPI_IO_HBV_RTC0  18
#define MEC_ESPI_IO_HBV_CVS1  19
#define MEC_ESPI_IO_HBV_TB32  20
#define MEC_ESPI_IO_HBV_UART2 21
#define MEC_ESPI_IO_HBV_GL    22
#define MEC_ESPI_IO_HBV_UART3 23
#define MEC_ESPI_IO_HBV_PP0   24
#define MEC_ESPI_IO_HBV_MAX   25

/* eSPI Memory Base Address registers for Host memory space */
#define MEC_ESPI_MEM_HBV_MBOX  0
#define MEC_ESPI_MEM_HBV_AEC0  1
#define MEC_ESPI_MEM_HBV_AEC1  2
#define MEC_ESPI_MEM_HBV_AEC2  3
#define MEC_ESPI_MEM_HBV_AEC3  4
#define MEC_ESPI_MEM_HBV_AEC4  5
#define MEC_ESPI_MEM_HBV_EMI0  6
#define MEC_ESPI_MEM_HBV_EMI1  7
#define MEC_ESPI_MEM_HBV_EMI2  8
#define MEC_ESPI_MEM_HBV_TB32  9
#define MEC_ESPI_MEM_HBV_MAX   10

/* EC-only Host I/O BAR: mask and LDN read-only */
#define MEC_ESPI_ECB_OFS(n)        (0x134u + ((uint32_t)(n) * 4u))
#define MEC_ESPI_ECB_AMSK_POS      0 /* address mask */
#define MEC_ESPI_ECB_AMSK_MSK      GENMASK(7, 0)
#define MEC_ESPI_ECB_AMSK_SET(n)   FIELD_PREP(MEC_ESPI_ECB_AMSK_MSK, (n))
#define MEC_ESPI_ECB_AMSK_GET(n)   FIELD_GET(MEC_ESPI_ECB_AMSK_MSK, (n))
#define MEC_ESPI_ECB_LD_POS        8
#define MEC_ESPI_ECB_LD_MSK        GENMASK(13, 8)
#define MEC_ESPI_ECB_LD_MSK_GET(n) FIELD_GET(MEC_ESPI_ECB_LD_MSK, (n))

/* host I/O base address bits[15:0] in bits[31:16] and valid flag at bit[0] */
#define MEC_ESPI_HBV_OFS(n)         (0x334u + ((uint32_t)(n) * 4u))
#define MEC_ESPI_HBV_VALID_EN_POS   0
#define MEC_ESPI_HBV_BASE_POS       16
#define MEC_ESPI_HBV_BASE_MSK0      GENMASK(15, 0)
#define MEC_ESPI_HBV_BASE_MSK       GENMASK(31, 16)
#define MEC_ESPI_HBV_BASE_SET(b)    FIELD_PREP(MEC_ESPI_HBV_BASE_MSK, (b))
#define MEC_ESPI_HBV_BASE_GET(h)    FIELD_GET(MEC_ESPI_HBV_BASE_MSK, (b))

/* Serial-IRQ slot index */
#define MEC_SIRQ_IDX_MBOX_E2H 0
#define MEC_SIRQ_IDX_MBOX_SMI 1u
#define MEC_SIRQ_IDX_KBC_KIRQ 2u
#define MEC_SIRQ_IDX_KBC_MIRQ 3u
#define MEC_SIRQ_IDX_AEC0_OBF 4u
#define MEC_SIRQ_IDX_AEC1_OBF 5u
#define MEC_SIRQ_IDX_AEC2_OBF 6u
#define MEC_SIRQ_IDX_AEC3_OBF 7u
#define MEC_SIRQ_IDX_AEC4_OBF 8u
#define MEC_SIRQ_IDX_UART0    9u
#define MEC_SIRQ_IDX_UART1    10u
#define MEC_SIRQ_IDX_EMI0_HE  11u
#define MEC_SIRQ_IDX_EMI0_E2H 12u
#define MEC_SIRQ_IDX_EMI1_HE  13u
#define MEC_SIRQ_IDX_EMI1_E2H 14u
#define MEC_SIRQ_IDX_EMI2_HE  15u
#define MEC_SIRQ_IDX_EMI2_E2H 16u
#define MEC_SIRQ_IDX_RTC      17u
#define MEC_SIRQ_IDX_EC_IRQ   18u
#define MEC_SIRQ_IDX_UART2    19u
#define MEC_SIRQ_IDX_PP0      20u
#define MEC_SIRQ_IDX_UART3    21u
#define MEC_SIRQ_IDX_MAX      22u

#define MEC_ESPI_SIRQ_OFS(n)  (0x3acu + (n))

/* ---- Peripheral channel ---- */
#define MEC_ESPI_PC_LC_ADDR_LSW_OFS   0x100u
#define MEC_ESPI_PC_LC_ADDR_MSW_OFS   0x104u

#define MEC_ESPI_PC_LC_LTT_OFS        0x108u
#define MEC_ESPI_PC_LC_LTT_LEN_POS    0
#define MEC_ESPI_PC_LC_LTT_LEN_MSK    GENMASK(11, 0)
#define MEC_ESPI_PC_LC_LTT_TYPE_POS   12
#define MEC_ESPI_PC_LC_LTT_TYPE_MSK   GENMASK(19, 12)
#define MEC_ESPI_PC_LC_LTT_TAG_POS    20
#define MEC_ESPI_PC_LC_LTT_TAG_MSK    GENMASK(23, 20)

#define MEC_ESPI_PC_ERA_LSW_OFS       0x10cu
#define MEC_ESPI_PC_ERA_MSW_OFS       0x110u

#define MEC_ESPI_PC_SR_OFS            0x114u
#define MEC_ESPI_PC_ABERR_POS         16
#define MEC_ESPI_PC_CHEN_STATE_POS    24 /* RO */
#define MEC_ESPI_PC_CHEN_CHG_POS      25
#define MEC_ESPI_PC_BMEN_STATE_POS    27 /* RO */
#define MEC_ESPI_PC_BMEN_CHG_POS      28

#define MEC_ESPI_PC_IER_OFS           0x118u
#define MEC_ESPI_PC_IER_ABERR_POS     16
#define MEC_ESPI_PC_IER_CHEN_CHG_POS  25
#define MEC_ESPI_PC_IER_BMEN_CHG_POS  28
#define MEC_ESPI_PC_IER_ALL_MSK       (BIT(16) | BIT(25) | BIT(28))

#define MEC_ESPI_PC_BAR_INH_LSW_OFS   0x120u
#define MEC_ESPI_PC_BAR_INH_MSW_OFS   0x124u

#define MEC_ESPI_PC_BAR_INIT_OFS      0x128u
#define MEC_ESPI_PC_BAR_INIT_MSK      GENMASK(15, 0)
#define MEC_ESPI_PC_BAR_INIT_DFLT     0x2eu

#define MEC_ESPI_PC_EC_SIRQ_OFS       0x12cu
#define MEC_ESPI_PC_EC_SIRQ_GEN_POS   0

#define MEC_ESPI_PC_LTR_SR_OFS        0x220u
#define MEC_ESPI_PC_LTR_TX_DONE_POS   0
#define MEC_ESPI_PC_LTR_START_OVR_POS 3
#define MEC_ESPI_PC_LTR_HOST_DIS_POS  4
#define MEC_ESPI_PC_LTR_TX_BUSY_POS   8 /* RO */

#define MEC_ESPI_PC_LTR_IER_OFS       0x224u
#define MEC_ESPI_PC_LTR_IER_TXD_POS   0

#define MEC_ESPI_PC_LTR_CR                0x228u
#define MEC_ESPI_PC_LTR_CR_STA_POS        0 /* WO */
#define MEC_ESPI_PC_LTR_CR_TAG_OUT_POS    8
#define MEC_ESPI_PC_LTR_CR_TAG_OUT_MSK    GENMASK(11, 8)
#define MEC_ESPI_PC_LTR_CR_TAG_OUT_SET(t) FIELD_PREP(MEC_ESPI_PC_LTR_CR_TAG_OUT_MSK, (t))

#define MEC_ESPI_PC_LTR_MSG_OFS           0x22cu
#define MEC_ESPI_PC_LTR_MSG_ALL_MSK       GENMASK(15, 0)
#define MEC_ESPI_PC_LTR_MSG_TV_POS        0 /* time value */
#define MEC_ESPI_PC_LTR_MSG_TV_MSK        GENMASK(9, 0)
#define MEC_ESPI_PC_LTR_MSG_TV_SET(t)     FIELD_PREP(MEC_ESPI_PC_LTR_MSG_TV_MSK, (t))
#define MEC_ESPI_PC_LTR_MSG_SC_POS        10 /* scale field: time units */
#define MEC_ESPI_PC_LTR_MSG_SC_MSK        GENMASK(12, 10)
#define MEC_ESPI_PC_LTR_MSG_SC_SET(sc)    FIELD_PREP(MEC_ESPI_PC_LTR_MSG_SC_MSK, (sc))
#define MEC_ESPI_PC_LTR_MSG_RTXB_POS      13
#define MEC_ESPI_PC_LTR_MSG_RTXB_MSK      GENMASK(14, 13)
#define MEC_ESPI_PC_LTR_MSG_RTXB_SET(r)   FIELD_PREP(MEC_ESPI_PC_LTR_MSG_RTXB_MSK, (r))
#define MEC_ESPI_PC_LTR_MSG_REQ_POS       15

/* ---- OOB channel ---- */

/* eSPI specification indicates maximum packet size for OOB channel is 64 bytes
 * Actually, the spec was modified to add 9 bytes to handle MCTP payloads.
 */
#define MEC_ESPI_OOB_ADDED_SIZE      9u

#define MEC_ESPI_OOB_RX_BA_OFS       0x240u /* OOB RX EC buffer address, b[1:0]=00b(RO) */
#define MEC_ESPI_OOB_TX_BA_OFS       0x248u /* OOB TX EC buffer address, b[1:0]=00b(RO) */

#define MEC_ESPI_OOB_RXL_OFS         0x250u
#define MEC_ESPI_OOB_RXL_MLEN_POS    0
#define MEC_ESPI_OOB_RXL_MLEN_MSK    GENMASK(12, 0) /* number of bytes received (RO) */
#define MEC_ESPI_OOB_RXL_MLEN_GET(n) FIELD_GET(MEC_ESPI_OOB_RXL_MLEN_MSK, (n))
#define MEC_ESPI_OOB_RXL_BLEN_POS    16
#define MEC_ESPI_OOB_RXL_BLEN_MSK    GENMASK(28, 16) /* max RX mesg length (RW) */
#define MEC_ESPI_OOB_RXL_BLEN_SET(n) FIELD_PREP(MEC_ESPI_OOB_RXL_BLEN_MSK, (n))

#define MEC_ESPI_OOB_TXL_OFS         0x254u
#define MEC_ESPI_OOB_TXL_MLEN_POS    0
#define MEC_ESPI_OOB_TXL_MLEN_MSK    GENMASK(12, 0) /* number of byte to transmit (RW) */
#define MEC_ESPI_OOB_TXL_MLEN_SET(n) FIELD_PREP(MEC_ESPI_OOB_TXL_MLEN_MSK, (n))
#define MEC_ESPI_OOB_TXL_MLEN_GET(n) FIELD_GET(MEC_ESPI_OOB_TXL_MLEN_MSK, (n))

#define MEC_ESPI_OOB_RX_CR_OFS       0x258u
#define MEC_ESPI_OOB_RX_CR_SRA_POS   0
#define MEC_ESPI_OOB_RX_CR_CHEN_POS  9  /* RO */
#define MEC_EPSI_OOB_MPLD_SZ_POS     16 /* RO */
#define MEC_ESPI_OOB_MPLD_SZ_MSK     GENMASK(18, 16)
#define MEC_ESPI_OOB_MPLD_SZ_GET(n)  FIELD_GET(MEC_ESPI_OOB_MPLD_SZ_MSK, (n))

#define MEC_ESPI_OOB_RX_IER_OFS      0x25cu
#define MEC_ESPI_OOB_RX_IER_DONE_POS 0

#define MEC_ESPI_OOB_RX_SR_OFS         0x260u
#define MEC_ESPI_OOB_RX_SR_DONE_POS    0
#define MEC_ESPI_OOB_RX_SR_ABERR_POS   1
#define MEC_ESPI_OOB_RX_SR_OVR_POS     2
#define MEC_ESPI_OOB_RX_SR_ALL_ERR_MSK GENMASK(2, 1)
#define MEC_ESPI_OOB_RX_SR_RXBA_POS    3 /* RO */
#define MEC_ESPI_OOB_RX_TAG_IN_POS     8
#define MEC_ESPI_OOB_RX_TAG_IN_MSK     GENMASK(11, 8)
#define MEC_ESPI_OOB_RX_TAG_IN_GET(tv) FIELD_GET(ESPI_OOB_RX_TAG_IN_MSK, (tv))

#define MEC_ESPI_OOB_TX_CR_OFS       0x264u
#define MEC_ESPI_OOB_TX_CR_START_POS 0 /* WO */
#define MEC_ESPI_OOB_TX_TAG_POS      8
#define MEC_ESPI_OOB_TX_TAG_MSK      GENMASK(11, 8)
#define MEC_ESPI_OOB_TX_TAG_SET(t)   FIELD_PREP(MEC_ESPI_OOB_TX_TAG_MSK, (t))
#define MEC_ESPI_OOB_TX_TAG_GET(t)   FIELD_GET(MEC_ESPI_OOB_TX_TAG_MSK, (t))

#define MEC_ESPI_OOB_TX_IER_OFS      0x268u
#define MEC_ESPI_OOB_TX_IER_DONE_POS 0
#define MEC_ESPI_OOB_TX_IER_CENC_POS 1 /* OOB channel enable change interrupt enable */

#define MEC_ESPI_OOB_TX_SR_OFS         0x26cu
#define MEC_ESPI_OOB_TX_SR_MSK         0x30fu
#define MEC_ESPI_OOB_TX_SR_DONE_POS    0
#define MEC_ESPI_OOB_TX_SR_CENC_POS    1
#define MEC_ESPI_OOB_TX_SR_ABERR_POS   2
#define MEC_ESPI_OOB_TX_SR_OVR_POS     3
#define MEC_ESPI_OOB_TX_SR_BAD_REQ_POS 5
#define MEC_ESPI_OOB_TX_SR_ALL_ERR_MSK (GENMASK(3, 2) | BIT(5))
#define MEC_ESPI_OOB_TX_SR_BUSY_POS    8 /* RO */
#define MEC_ESPI_OOB_TX_SR_CHEN_POS    9 /* RO image of OOB channel enable */

/* Flash channel  */
#define MEC_ESPI_FC_FA_OFS             0x280u /* Flash channel flash address register */

#define MEC_ESPI_FC_BA_OFS             0x288u /* Flash channel EC buffer address register */
#define MEC_ESPI_FC_BA_MSK             GENMASK(31, 2)

#define MEC_ESPI_FC_LEN_OFS            0x290u
#define MEC_ESPI_FC_LEN_ERASE_VAL      0x01u

#define MEC_ESPI_FC_CR_OFS           0x294u /* R/W unless specified */
#define MEC_ESPI_FC_CR_START_POS     0      /* WO */
#define MEC_ESPI_FC_CR_FUNC_POS      2
#define MEC_ESPI_FC_CR_FUNC_MSK      GENMASK(3, 2)
#define MEC_ESPI_FC_CR_FUNC_READ     FIELD_PREP(MEC_ESPI_FC_CR_FUNC_MSK, 0)
#define MEC_ESPI_FC_CR_FUNC_WRITE    FIELD_PREP(MEC_ESPI_FC_CR_FUNC_MSK, 1)
#define MEC_ESPI_FC_CR_FUNC_ERASE_SM FIELD_PREP(MEC_ESPI_FC_CR_FUNC_MSK, 2)
#define MEC_ESPI_FC_CR_FUNC_ERASE_LG FIELD_PREP(MEC_ESPI_FC_CR_FUNC_MSK, 3)
#define MEC_ESPI_FC_CR_FUNC_SET(f)   MEC_FILED_PREP(MEC_ESPI_FC_CR_FUNC_MSK, (f))
#define MEC_ESPI_FC_CR_FUNC_GET(f)   MEC_FILED_GET(MEC_ESPI_FC_CR_FUNC_MSK, (f))
#define MEC_ESPI_FC_CR_TAG_POS       4
#define MEC_ESPI_FC_CR_TAG_MSK       GENMASK(7, 4)
#define MEC_ESPI_FC_CR_TAG_SET(t)    FIELD_PREP(MEC_ESPI_FC_CR_TAG_MSK, (t))
#define MEC_ESPI_FC_CR_TAG_GET(t)    FIELD_GET(MEC_ESPI_FC_CR_TAG_MSK, (t))
#define MEC_ESPI_FC_CR_ABORT_POS     16

#define MEC_ESPI_FC_IER_OFS          0x298u
#define MEC_ESPI_FC_IER_DONE_POS     0
#define MEC_ESPI_FC_IER_CHG_EN_POS   1

#define MEC_ESPI_FC_CFG_OFS             0x29cu /* all RO except where specified */
#define MEC_ESPI_FC_CFG_BUSY_POS        0
#define MEC_ESPI_FC_CFG_EBSZ_POS        2
#define MEC_ESPI_FC_CFG_EBSZ_MSK        GENMASK(4, 2)
#define MEC_ESPI_FC_CFG_EBSZ_4K_VAL     1u
#define MEC_ESPI_FC_CFG_EBSZ_64K_VAL    2u
#define MEC_ESPI_FC_CFG_EBSZ_4K_64K_VAL 3u
#define MEC_ESPI_FC_CFG_EBSZ_128K_VAL   4u
#define MEC_ESPI_FC_CFG_EBSZ_256K_VAL   5u
#define MEC_ESPI_FC_CFG_EBSZ_4K         FIELD_PREP(MEC_ESPI_FC_CFG_EBSZ_MSK, MEC_ESPI_FC_CFG_EBSZ_4K_VAL)
#define MEC_ESPI_FC_CFG_EBSZ_64K        FIELD_PREP(MEC_ESPI_FC_CFG_EBSZ_MSK, MEC_ESPI_FC_CFG_EBSZ_64K_VAL)
#define MEC_ESPI_FC_CFG_EBSZ_4K_64K     FIELD_PREP(MEC_ESPI_FC_CFG_EBSZ_MSK, MEC_ESPI_FC_CFG_EBSZ_4K_64K_VAL)
#define MEC_ESPI_FC_CFG_EBSZ_128K       FIELD_PREP(MEC_ESPI_FC_CFG_EBSZ_MSK, MEC_ESPI_FC_CFG_EBSZ_128K_VAL)
#define MEC_ESPI_FC_CFG_EBSZ_256K       FIELD_PREP(MEC_ESPI_FC_CFG_EBSZ_MSK, MEC_ESPI_FC_CFG_EBSZ_256K_VAL)
#define MEC_ESPI_FC_CFG_EBSZ_GET(rv)    FIELD_GET(MEC_ESPI_FC_CFG_EBSZ_MSK, (rv))
#define MEC_ESPI_FC_CFG_MPLD_POS        8
#define MEC_ESPI_FC_CFG_MPLD_MSK        GENMASK(10, 8)
#define MEC_ESPI_FC_CFG_MPLD_64_VAL     1
#define MEC_ESPI_FC_CFG_MPLD_128_VAL    2
#define MEC_ESPI_FC_CFG_MPLD_256_VAL    3
#define MEC_ESPI_FC_CFG_MPLD_64         FIELD_PREP(MEC_ESPI_FC_CFG_MPLD_MSK, MEC_ESPI_FC_CFG_MPLD_64_VAL)
#define MEC_ESPI_FC_CFG_MPLD_128        FIELD_PREP(MEC_ESPI_FC_CFG_MPLD_MSK, MEC_ESPI_FC_CFG_MPLD_128_VAL)
#define MEC_ESPI_FC_CFG_MPLD_256        FIELD_PREP(MEC_ESPI_FC_CFG_MPLD_MSK, MEC_ESPI_FC_CFG_MPLD_256_VAL)
#define MEC_ESPI_FC_CFG_MPLD_GET(rv)    FIELD_GET(MEC_ESPI_FC_CFG_MPLD_MSK, (rv)) /* rv = register value */
#define MEC_ESPI_FC_CFG_TAF_MODE_POS    11
#define MEC_ESPI_FC_CFG_MRDR_POS        12
#define MEC_ESPI_FC_CFG_MRDR_MSK        GENMASK(14, 12)
#define MEC_ESPI_FC_CFG_MDRD_64_VAL     1
#define MEC_ESPI_FC_CFG_MDRD_128_VAL    2
#define MEC_ESPI_FC_CFG_MDRD_256_VAL    3
#define MEC_ESPI_FC_CFG_MDRD_512_VAL    4
#define MEC_ESPI_FC_CFG_MDRD_1K_VAL     5
#define MEC_ESPI_FC_CFG_MDRD_2K_VAL     6
#define MEC_ESPI_FC_CFG_MDRD_4K_VAL     7
#define MEC_ESPI_FC_CFG_MRDR_64         FIELD_PREP(MEC_ESPI_FC_CFG_MRDR_MSK, MEC_ESPI_FC_CFG_MDRD_64_VAL)
#define MEC_ESPI_FC_CFG_MRDR_128        FIELD_PREP(MEC_ESPI_FC_CFG_MRDR_MSK, MEC_ESPI_FC_CFG_MDRD_128_VAL)
#define MEC_ESPI_FC_CFG_MRDR_256        FIELD_PREP(MEC_ESPI_FC_CFG_MRDR_MSK, MEC_ESPI_FC_CFG_MDRD_256_VAL)
#define MEC_ESPI_FC_CFG_MRDR_512        FIELD_PREP(MEC_ESPI_FC_CFG_MRDR_MSK, MEC_ESPI_FC_CFG_MDRD_512_VAL)
#define MEC_ESPI_FC_CFG_MRDR_1K         FIELD_PREP(MEC_ESPI_FC_CFG_MRDR_MSK, MEC_ESPI_FC_CFG_MDRD_1K_VAL)
#define MEC_ESPI_FC_CFG_MRDR_2K         FIELD_PREP(MEC_ESPI_FC_CFG_MRDR_MSK, MEC_ESPI_FC_CFG_MDRD_2K_VAL)
#define MEC_ESPI_FC_CFG_MRDR_4K         FIELD_PREP(MEC_ESPI_FC_CFG_MRDR_MSK, MEC_ESPI_FC_CFG_MDRD_4K_VAL)
#define MEC_ESPI_FC_CFG_MRDR_GET(rv)    FIELD_GET(MEC_ESPI_FC_CFG_MRDR_MSK, (rv))
#define MEC_ESPI_FC_CFG_CTF_POS         28 /* R/W */
#define MEC_ESPI_FC_CFG_CTF_CAF         0
#define MEC_ESPI_FC_CFG_CTF_TAF         BIT(MEC_ESPI_FC_CFG_CTF_POS)
#define MEC_ESPI_FC_CFG_CTF_LOCK_POS    29

#define MEC_ESPI_FC_SR_OFS              0x2a0u /* RW/1C unless specified */
#define MEC_ESPI_FC_SR_CHEN_STATE_POS   0      /* RO */
#define MEC_ESPI_FC_SR_CHEN_CHG_POS     1
#define MEC_ESPI_FC_SR_DONE_POS         2
#define MEC_ESPI_FC_SR_DIS_WB_POS       3 /* Host disabled channel while it was busy */
#define MEC_ESPI_FC_SR_ABERR_POS        4
#define MEC_ESPI_FC_SR_ABORT_POS        5
#define MEC_ESPI_FC_SR_OVR_POS          6
#define MEC_ESPI_FC_SR_INC_POS          7
#define MEC_ESPI_FC_SR_FAIL_POS         8
#define MEC_ESPI_FC_SR_START_OVFL_POS   9
#define MEC_ESPI_FC_SR_BAD_REQ_POS      11
#define MEC_ESPI_FC_SR_ERR_ALL_MSK      GENMASK(11, 3)
#define MEC_ESPI_FC_SR_ALL_MSK          GENMASK(11, 0)

/* ---- Virtual Wire Channel  ---- */

#define MEC_ESPI_VW_SR_OFS                0x2b0u
#define MEC_ESPI_VW_SR_CHEN_POS           0 /* RO */

#define MEC_ESPI_VW_ERR_SR_OFS            0x3f0u
#define MEC_ESPI_VW_ERR_FATAL_POS         0 /* read-only */
#define MEC_ESPI_VW_ERR_FATAL_CLR_POS     1 /* write-only */
#define MEC_ESPI_VW_ERR_NON_FATAL_POS     4 /* read-only */
#define MEC_ESPI_VW_ERR_NON_FATAL_CLR_POS 5 /* write-only */

/* ---- eSPI Memory component ---- */

/* Two different eSPI memory component BARs are implemented.
 * Memory BAR internal registers contain read-only fields.
 * b[7:0] = Host address mask field (RO)
 * b[13:8] = Logical device number (RO)
 *
 * Memory BAR configuration registers are 48-bit (R/W)
 * b[0] = R/W valid bit.
 * b[47:16] = R/W host address bits[31:0]
 *
 * NOTE 1: Both internal and configuration memory BARs are 80-bits
 * and packed meaning some are located on 16-bit address boundaries.
 *
 * NOTE 2: Host address bits[47:32] are common to all memory BARs
 * and is located in the MEC_ESPI_MC_MBAR_HA_EXT register.
 */

#define MEC_ESPI_MC_BAR_OFS     0x130u /* EC access only */
#define MEC_ESPI_MC_BAR_CFG_OFS 0x330u /* Host and EC access */

#define MEC_ESPI_MC_BAR_REG_OFS(id, hw) \
    (MEC_ESPI_MC_BAR_OFS + ((uint32_t)(id) * 10u) + ((hw) * 2u))

#define MEC_ESPI_MC_BAR_CFG_REG_OFS(id, hw) \
    (MEC_ESPI_MC_BAR_CFG_OFS + ((uint32_t)(id) * 10u) + ((hw) * 2u))

/* values of hw (half-word) for register at MEC_ESPI_MC_BAR_OFS */
#define MEC_ESPI_MC_BAR_AMLD_HW 0 /* half-word 0 contains address mask and LDN */
#define MEC_ESPI_MC_BAR_AMLD_MSK_POS    0
#define MEC_ESPI_MC_BAR_AMLD_MSK_MSK    GENMASK(7, 0)
#define MEC_ESPI_MC_BAR_AMLD_MSK_SET(n) FIELD_PREP(MEC_ESPI_MC_BAR_AMLD_MSK_MSK, (n))
#define MEC_ESPI_MC_BAR_AMLD_MSK_GET(n) FIELD_GET(MEC_ESPI_MC_BAR_AMLD_MSK_MSK, (n))
#define MEC_ESPI_MC_BAR_AMLD_LD_POS     8
#define MEC_ESPI_MC_BAR_AMLD_LD_MSK     GENMASK(13, 8)
#define MEC_ESPI_MC_BAR_AMLD_LD_SET(n)  FIELD_PREP(MEC_ESPI_MC_BAR_AMLD_LD_MSK, (n))
#define MEC_ESPI_MC_BAR_AMLD_LD_GET(n)  FIELD_GET(MEC_ESPI_MC_BAR_AMLD_LD_MSK, (n))

/* values of hw (half-word) for registers at MEC_ESPI_MC_BAR_CFG_OFS */
#define MEC_ESPI_MC_BAR_CFG_VAL_HW  0 /* half-word 0 bit 0 is the valid bit */
#define MEC_ESPI_MC_BAR_CFG_VAL_POS 0

#define MEC_ESPI_MC_BAR_CFG_HA0_HW 2 /* half-word 2 is b[15:0] of host address */
#define MEC_ESPI_MC_BAR_CFG_HA1_HW 3 /* half-word 3 is b[31:16] of host address */

/* id values */
#define MEC_ESPI_MC_MBOX_BAR_ID 0
#define MEC_ESPI_MC_AEC0_BAR_ID 1
#define MEC_ESPI_MC_AEC1_BAR_ID 2
#define MEC_ESPI_MC_AEC2_BAR_ID 3
#define MEC_ESPI_MC_AEC3_BAR_ID 4
#define MEC_ESPI_MC_AEC4_BAR_ID 5
#define MEC_ESPI_MC_EMI0_BAR_ID 6
#define MEC_ESPI_MC_EMI1_BAR_ID 7
#define MEC_ESPI_MC_EMI2_BAR_ID 8
#define MEC_ESPI_MC_MAX_BAR_ID  9

/* ---- SRAM BARs ---- */
#define MEC_ESPI_MC_SRAM_BAR0_ID    0
#define MEC_ESPI_MC_SRAM_BAR1_ID    1
#define MEC_ESPI_MC_SRAM_BAR_ID_MAX 2

/* SRAM BAR (register accessible by EC only)
 * b[0] = valid
 * b[2:1] = access: none, RO, WO, R/W
 * b[7:4] = size: 1 byte to 32KB in powers of 2
 * b[47:16] = b[31:0] of host address aligned by size.
 */
#define MEC_ESPI_MC_SBAR0 0x1acu
#define MEC_ESPI_MC_SRAM1 0x1b6u

#define MEC_ESPI_MC_SBAR_OFS(id) (MEC_ESPI_MC_SBAR0 + ((uint32_t)id * 10u))

/* half-word 0 */
#define MEC_ESPI_MC_SBAR_VALID_POS  0
#define MEC_ESPI_MC_SBAR_ACC_POS    1
#define MEC_ESPI_MC_SBAR_ACC_MSK    GENMASK(2, 1)
#define MEC_ESPI_MC_SBAR_ACC_SET(n) FIELD_PREP(MEC_ESPI_MC_SBAR_ACC_MSK, (n))
#define MEC_ESPI_MC_SBAR_ACC_GET(n) FIELD_GET(MEC_ESPI_MC_SBAR_ACC_MSK, (n))

#define MEC_ESPI_MC_SBAR_SZ_POS     4
#define MEC_ESPI_MC_SBAR_SZ_MSK     GENMASK(7, 4)
#define MEC_ESPI_MC_SBAR_SZ_SET(sz) FIELD_PREP(MEC_ESPI_MC_SBAR_SZ_MSK, (sz))
#define MEC_ESPI_MC_SBAR_SZ_GET(sz) FIELD_GET(MEC_ESPI_MC_SBAR_SZ_MSK, (sz))

/* SRAM BAR Config (register accessible by EC and Host)
 * b[48:16] = Host address b[31:0]
 * b[7:4] = RO copy of SRAM region size
 * b[2:1] = RO copy of access: none, RO, WO, or R/W
 */
#define MEC_ESPI_MC_SBAR0_CFG 0x3acu
#define MEC_ESPI_MC_SBAR1_CFG 0x3b6u

#define MEC_ESPI_MC_SBAR_CFG_OFS(id) (MEC_ESPI_MC_SBAR0_CFG + ((uint32_t)id * 10u))

/* eSPI memory and SRAM BAR host extended address registers
 * b[31:0] = host address [47:32]
 */
#define MEC_ESPI_MC_MBAR_HA_EXT 0x3a8u
#define MEC_ESPI_MC_SBAR_HA_EXT 0x3fcu

/* -------- VWire Component -------- */

#define MEC_ESPI_VW_GRP0  0
#define MEC_ESPI_VW_GRP1  1
#define MEC_ESPI_VW_GRP2  2
#define MEC_ESPI_VW_GRP3  3
#define MEC_ESPI_VW_GRP4  4
#define MEC_ESPI_VW_GRP5  5
#define MEC_ESPI_VW_GRP6  6
#define MEC_ESPI_VW_GRP7  7
#define MEC_ESPI_VW_GRP8  8
#define MEC_ESPI_VW_GRP9  9
#define MEC_ESPI_VW_GRP10 10
#define MEC_ESPI_VW_NGRPS 11

/* Host-to-Target virtual wires are groups of 4 vwires per eSPI specification.
 * MEC5 implements 11 CTVW groups. Each group is a 96-bit register.
 * Access is as three 32-bit registers.
 * b[31:0] contain Host index, reset source, and reset state
 * b[63:32] contain interrupt select for each of the 4 vwires in the group
 * b[95:64] contain current vwire states
 */
#define MEC_ESPI_VW_HT_OFS        0
#define MEC_ESPI_VW_HT_GRP(n)     (MEC_ESPI_VW_HT_OFS + ((uint32_t)(n) * 12u))
#define MEC_ESPI_VW_HT_GRPW(n, w) (MEC_ESPI_VW_HT_GRP(n) + ((uint32_t)(w) * 4u))

/* Target-to-Host virtual wires are groups of 4 vwires.
 * MEC5 implements 11 TCVW groups. Each group is a 64-bit register accessed
 * as two 32-bit registers.
 * b[31:0] contain Host index, reset source, reset state, and RO change status
 * b[63:32] contain current vwire states
 */
#define MEC_ESPI_VW_TH_OFS        0x200u
#define MEC_ESPI_VW_TH_GRP(n)     (MEC_ESPI_VW_TH_OFS + ((uint32_t)(n) * 8u))
#define MEC_ESPI_VW_TH_GRPW(n, w) (MEC_ESPI_VW_TH_GRP(n) + ((uint32_t)(w) * 4u))

/* Word 0 is the same for H2T and T2H vwire registers */
#define MEC_ESPI_VW_W0_HI_POS        0 /* Host index value */
#define MEC_ESPI_VW_W0_HI_MSK        GENMASK(7, 0)
#define MEC_ESPI_VW_W0_HI_SET(hi)    FIELD_PREP(MEC_ESPI_VW_W0_HI_MSK, (hi))
#define MEC_ESPI_VW_W0_HI_GET(hi)    FIELD_GET(MEC_ESPI_VW_W0_HI_MSK, (hi))

#define MEC_ESPI_VW_W0_RSRC_POS      8
#define MEC_ESPI_VW_W0_RSRC_MSK      GENMASK(9, 8)
#define MEC_ESPI_VW_W0_RSRC_ESPI_RST FIELD_PREP(MEC_ESPI_VW_W0_RSRC_MSK, 0)
#define MEC_ESPI_VW_W0_RSRC_SYS_RST  FIELD_PREP(MEC_ESPI_VW_W0_RSRC_MSK, 1)
#define MEC_ESPI_VW_W0_RSRC_SIO_RST  FIELD_PREP(MEC_ESPI_VW_W0_RSRC_MSK, 2)
#define MEC_ESPI_VW_W0_RSRC_PLT_RST  FIELD_PREP(MEC_ESPI_VW_W0_RSRC_MSK, 3)
#define MEC_ESPI_VW_W0_RSRC_SET(r)   FIELD_PREP(MEC_ESPI_VW_W0_RSRC_MSK, (r))
#define MEC_ESPI_VW_W0_RSRC_GET(r)   FIELD_GET(MEC_ESPI_VW_W0_RSRC_MSK, (r))

#define MEC_ESPI_VW_W0_RSTATE_POS    12 /* bitmap of the 4 vwires in the group */
#define MEC_ESPI_VW_W0_RSTATE_MSK    GENMASK(15, 12)
#define MEC_ESPI_VW_W0_RSTATE_SET(r) FIELD_PREP(MEC_ESPI_VW_W0_RSTATE_MSK, (r))
#define MEC_ESPI_VW_W0_RSTATE_GET(r) FIELD_GET(MEC_ESPI_VW_W0_RSTATE_MSK, (r))

/* T2H word 0 bits[19:16] are read-only change bits for each VWire in the group.
 * If software changes the T2H Vwire state in word 1 bits[0, 8, 16, 24] the
 * corresponding bits[16, 17, 18, 19] of word 0 become 1. When the Host reads
 * the VWire group using the eSPI GET_VW command, hardware will clear the
 * read-only bits[16:19].
 */
#define MEC_ESPI_VW_T2H_W0_CHG0_POS   16
#define MEC_ESPI_VW_T2H_W0_CHG_POS(s) (MEC_ESPI_VW_T2H_W0_CHG0_POS + (uint8_t)(s))

/* H2T word 1 contains IRQ_SEL 4-bit fields in each byte
 * s is the source bit position in the VW group [0, 3]
 */
#define MEC_ESPI_VW_H2T_W1_ISEL_POS(s) ((s) * 8)
#define MEC_ESPI_VW_H2T_V1_ISEL_MSK0 0xfu
#define MEC_ESPI_VW_H2T_W1_ISEL_MSK(s)                                  \
    GENMASK(MEC_ESPI_VW_H2T_W1_ISEL_POS(s) + 3, MEC_ESPI_VW_H2T_W1_ISEL_POS(s))
#define MEC_ESPI_VW_H2T_W1_ISEL_SET(s, i) \
    FIELD_PREP(MEC_ESPI_VW_H2T_W1_ISEL_MSK(s), (i))
#define MEC_ESPI_VW_H2T_W1_ISEL_GET(s, i) \
    FIELD_GET(MEC_ESPI_VW_H2T_W1_ISEL_MSK(s), (i))

/* H2T word 2 and T2H word 1 contain the current value of each of the
 * four VWires in the group. The bits are located in bit[0] of each byte.
 * s is the source bit position in the VW group [0, 3]
 */
#define MEC_ESPI_VW_STATE_POS(s) ((s) * 8)

#endif /*  ZEPHYR_DRIVERS_ESPI_MCHP_REGS_ESPI_H_ */
