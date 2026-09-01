/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Register indices used by the io-bars, mem-bars and sirqs properties of the
 * Microchip XEC eSPI V3 peripheral (logical) channel device nodes.
 *
 * These mirror enum espi_io_bar_idx, enum espi_mem_bar_idx and
 * enum espi_io_sirq_idx in soc/microchip/mec/common/reg/mec_espi_iom_v2.h,
 * which device tree cannot see. espi_mchp_xec_v3.c asserts at build time that
 * the two stay in step.
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_ESPI_MCHP_XEC_ESPI_V3_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_ESPI_MCHP_XEC_ESPI_V3_H_

/* Host I/O BAR indices */
#define MCHP_XEC_ESPI_IOB_IOC         0
#define MCHP_XEC_ESPI_IOB_MEM         1
#define MCHP_XEC_ESPI_IOB_MBOX        2
#define MCHP_XEC_ESPI_IOB_KBC         3
#define MCHP_XEC_ESPI_IOB_ACPI_EC0    4
#define MCHP_XEC_ESPI_IOB_ACPI_EC1    5
#define MCHP_XEC_ESPI_IOB_ACPI_EC2    6
#define MCHP_XEC_ESPI_IOB_ACPI_EC3    7
#define MCHP_XEC_ESPI_IOB_ACPI_EC4    8
#define MCHP_XEC_ESPI_IOB_ACPI_PM1    9
#define MCHP_XEC_ESPI_IOB_PORT92      10
#define MCHP_XEC_ESPI_IOB_UART0       11
#define MCHP_XEC_ESPI_IOB_UART1       12
#define MCHP_XEC_ESPI_IOB_EMI0        13
#define MCHP_XEC_ESPI_IOB_EMI1        14
#define MCHP_XEC_ESPI_IOB_EMI2        15
#define MCHP_XEC_ESPI_IOB_P80BD       16
#define MCHP_XEC_ESPI_IOB_P80BD_ALIAS 17
#define MCHP_XEC_ESPI_IOB_RTC         18
#define MCHP_XEC_ESPI_IOB_T32B        20
#define MCHP_XEC_ESPI_IOB_UART2       21
#define MCHP_XEC_ESPI_IOB_GLUE        22
#define MCHP_XEC_ESPI_IOB_UART3       23

/* Host memory BAR indices */
#define MCHP_XEC_ESPI_MEMB_MBOX     0
#define MCHP_XEC_ESPI_MEMB_ACPI_EC0 1
#define MCHP_XEC_ESPI_MEMB_ACPI_EC1 2
#define MCHP_XEC_ESPI_MEMB_ACPI_EC2 3
#define MCHP_XEC_ESPI_MEMB_ACPI_EC3 4
#define MCHP_XEC_ESPI_MEMB_ACPI_EC4 5
#define MCHP_XEC_ESPI_MEMB_EMI0     6
#define MCHP_XEC_ESPI_MEMB_EMI1     7
#define MCHP_XEC_ESPI_MEMB_EMI2     8
#define MCHP_XEC_ESPI_MEMB_T32B     9

/* Serial IRQ register indices */
#define MCHP_XEC_ESPI_SIRQ_MBOX          0
#define MCHP_XEC_ESPI_SIRQ_MBOX_SMI      1
#define MCHP_XEC_ESPI_SIRQ_KBC_KIRQ      2
#define MCHP_XEC_ESPI_SIRQ_KBC_MIRQ      3
#define MCHP_XEC_ESPI_SIRQ_ACPI_EC0_OBF  4
#define MCHP_XEC_ESPI_SIRQ_ACPI_EC1_OBF  5
#define MCHP_XEC_ESPI_SIRQ_ACPI_EC2_OBF  6
#define MCHP_XEC_ESPI_SIRQ_ACPI_EC3_OBF  7
#define MCHP_XEC_ESPI_SIRQ_ACPI_EC4_OBF  8
#define MCHP_XEC_ESPI_SIRQ_UART0         9
#define MCHP_XEC_ESPI_SIRQ_UART1         10
#define MCHP_XEC_ESPI_SIRQ_EMI0_HEV      11
#define MCHP_XEC_ESPI_SIRQ_EMI0_E2H      12
#define MCHP_XEC_ESPI_SIRQ_EMI1_HEV      13
#define MCHP_XEC_ESPI_SIRQ_EMI1_E2H      14
#define MCHP_XEC_ESPI_SIRQ_EMI2_HEV      15
#define MCHP_XEC_ESPI_SIRQ_EMI2_E2H      16
#define MCHP_XEC_ESPI_SIRQ_RTC           17
#define MCHP_XEC_ESPI_SIRQ_EC            18
#define MCHP_XEC_ESPI_SIRQ_UART2         19
#define MCHP_XEC_ESPI_SIRQ_UART3         21

/* Value written to a Serial IRQ register to disable that slot */
#define MCHP_XEC_ESPI_SIRQ_SLOT_DISABLED 0xff

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_ESPI_MCHP_XEC_ESPI_V3_H_ */
