/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SOC_MICROCHIP_MEC_MEC174X_MEC174X_H
#define __SOC_MICROCHIP_MEC_MEC174X_MEC174X_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
/* ARM Cortex-M4 Specific Interrupt Numbers */
  Reset_IRQn                = -15,
  NonMaskableInt_IRQn       = -14,
  HardFault_IRQn            = -13,
  MemoryManagement_IRQn     = -12,
  BusFault_IRQn             = -11,
  UsageFault_IRQn           = -10,
  SVCall_IRQn               =  -5,
  DebugMonitor_IRQn         =  -4,
  PendSV_IRQn               =  -2,
  SysTick_IRQn              =  -1,
/* MEC174x Specific Interrupt Numbers */
  MEC_GIRQ08_IRQn           =   0,
  MEC_GIRQ09_IRQn           =   1,
  MEC_GIRQ10_IRQn           =   2,
  MEC_GIRQ11_IRQn           =   3,
  MEC_GIRQ12_IRQn           =   4,
  MEC_GIRQ13_IRQn           =   5,
  MEC_GIRQ14_IRQn           =   6,
  MEC_GIRQ15_IRQn           =   7,
  MEC_GIRQ16_IRQn           =   8,
  MEC_GIRQ17_IRQn           =   9,
  MEC_GIRQ18_IRQn           =  10,
  MEC_GIRQ19_IRQn           =  11,
  MEC_GIRQ20_IRQn           =  12,
  MEC_GIRQ21_IRQn           =  13,
  MEC_GIRQ23_IRQn           =  14,
  MEC_GIRQ24_IRQn           =  15,
  MEC_GIRQ25_IRQn           =  16,
  MEC_GIRQ26_IRQn           =  17,
  MEC_I2C_SMB0_IRQn         =  20,
  MEC_I2C_SMB1_IRQn         =  21,
  MEC_I2C_SMB2_IRQn         =  22,
  MEC_I2C_SMB3_IRQn         =  23,
  MEC_DMA_CH00_IRQn         =  24,
  MEC_DMA_CH01_IRQn         =  25,
  MEC_DMA_CH02_IRQn         =  26,
  MEC_DMA_CH03_IRQn         =  27,
  MEC_DMA_CH04_IRQn         =  28,
  MEC_DMA_CH05_IRQn         =  29,
  MEC_DMA_CH06_IRQn         =  30,
  MEC_DMA_CH07_IRQn         =  31,
  MEC_DMA_CH08_IRQn         =  32,
  MEC_DMA_CH09_IRQn         =  33,
  MEC_DMA_CH10_IRQn         =  34,
  MEC_DMA_CH11_IRQn         =  35,
  MEC_DMA_CH12_IRQn         =  36,
  MEC_DMA_CH13_IRQn         =  37,
  MEC_DMA_CH14_IRQn         =  38,
  MEC_DMA_CH15_IRQn         =  39,
  MEC_UART0_IRQn            =  40,
  MEC_UART1_IRQn            =  41,
  MEC_EMI0_IRQn             =  42,
  MEC_EMI1_IRQn             =  43,
  MEC_EMI2_IRQn             =  44,
  MEC_ACPI_EC0_IBF_IRQn     =  45,
  MEC_ACPI_EC0_OBE_IRQn     =  46,
  MEC_ACPI_EC1_IBF_IRQn     =  47,
  MEC_ACPI_EC1_OBE_IRQn     =  48,
  MEC_ACPI_EC2_IBF_IRQn     =  49,
  MEC_ACPI_EC2_OBE_IRQn     =  50,
  MEC_ACPI_EC3_IBF_IRQn     =  51,
  MEC_ACPI_EC3_OBE_IRQn     =  52,
  MEC_ACPI_EC4_IBF_IRQn     =  53,
  MEC_ACPI_EC4_OBE_IRQn     =  54,
  MEC_ACPI_PM1_CTL_IRQn     =  55,
  MEC_ACPI_PM1_EN_IRQn      =  56,
  MEC_ACPI_PM1_STS_IRQn     =  57,
  MEC_KBC0_OBE_IRQn         =  58,
  MEC_KBC0_IBF_IRQn         =  59,
  MEC_MBOX0_IRQn            =  60,
  MEC_BDP0_IRQn             =  62,
  MEC_PECI0_IRQn            =  70,
  MEC_TACH0_IRQn            =  71,
  MEC_TACH1_IRQn            =  72,
  MEC_TACH2_IRQn            =  73,
  MEC_RPMFAN0_FAIL_IRQn     =  74,
  MEC_RPMFAN0_STALL_IRQn    =  75,
  MEC_RPMFAN1_FAIL_IRQn     =  76,
  MEC_RPMFAN1_STALL_IRQn    =  77,
  MEC_ADC0_SGL_IRQn         =  78,
  MEC_ADC0_RPT_IRQn         =  79,
  MEC_RCID0_IRQn            =  80,
  MEC_RCID1_IRQn            =  81,
  MEC_RCID2_IRQn            =  82,
  MEC_BBLED0_IRQn           =  83,
  MEC_BBLED1_IRQn           =  84,
  MEC_BBLED2_IRQn           =  85,
  MEC_BBLED3_IRQn           =  86,
  MEC_PHOT_IRQn             =  87,
  MEC_QSPI0_IRQn            =  91,
  MEC_GSPI0_IRQn            =  92,
  MEC_GSPI1_IRQn            =  94,
  MEC_BCL0_ERR_IRQn         =  96,
  MEC_BCL0_BCLR_IRQn        =  97,
  MEC_PS2CTL0_ACT_IRQn      = 100,
  MEC_PS2CTL1_ACT_IRQn      = 101,
  MEC_ESPI_PC_IRQn          = 103,
  MEC_ESPI_BM1_IRQn         = 104,
  MEC_ESPI_BM2_IRQn         = 105,
  MEC_ESPI_LTR_IRQn         = 106,
  MEC_ESPI_OOB_UP_IRQn      = 107,
  MEC_ESPI_OOB_DN_IRQn      = 108,
  MEC_ESPI_FC_IRQn          = 109,
  MEC_ESPI_RST_IRQn         = 110,
  MEC_RTMR0_IRQn            = 111,
  MEC_HTMR0_IRQn            = 112,
  MEC_HTMR1_IRQn            = 113,
  MEC_WKTMR0_ALARM_IRQn     = 114,
  MEC_WKTMR0_SUBWK_IRQn     = 115,
  MEC_WKTMR0_ONESEC_IRQn    = 116,
  MEC_WKTMR0_SUBSEC_IRQn    = 117,
  MEC_WKTMR0_PWR_IRQn       = 118,
  MEC_RTC0_CLK_IRQn         = 119,
  MEC_RTC0_ALARM_IRQn       = 120,
  MEC_VCI_OVRD_IN_IRQn      = 121,
  MEC_VCI_IN0_IRQn          = 122,
  MEC_VCI_IN1_IRQn          = 123,
  MEC_VCI_IN2_IRQn          = 124,
  MEC_VCI_IN3_IRQn          = 125,
  MEC_VCI_IN4_IRQn          = 126,
  MEC_PS2CTL0_WK0A_IRQn     = 129,
  MEC_PS2CTL0_WK0B_IRQn     = 130,
  MEC_PS2CTL1_WK1B_IRQn     = 132,
  MEC_KSCAN0_INT_IRQn       = 135,
  MEC_BTMR0_IRQn            = 136,
  MEC_BTMR1_IRQn            = 137,
  MEC_BTMR2_IRQn            = 138,
  MEC_BTMR3_IRQn            = 139,
  MEC_BTMR4_IRQn            = 140,
  MEC_BTMR5_IRQn            = 141,
  MEC_CTMR0_IRQn            = 142,
  MEC_CTMR1_IRQn            = 143,
  MEC_CTMR2_IRQn            = 144,
  MEC_CTMR3_IRQn            = 145,
  MEC_CCT0_TMR_IRQn         = 146,
  MEC_CCT0_CAP0_IRQn        = 147,
  MEC_CCT0_CAP1_IRQn        = 148,
  MEC_CCT0_CAP2_IRQn        = 149,
  MEC_CCT0_CAP3_IRQn        = 150,
  MEC_CCT0_CAP4_IRQn        = 151,
  MEC_CCT0_CAP5_IRQn        = 152,
  MEC_CCT0_CMP0_IRQn        = 153,
  MEC_CCT0_CMP1_IRQn        = 154,
  MEC_ESPI_VWEN_IRQn        = 156,
  MEC_I2C_SMB4_IRQn         = 158,
  MEC_TACH3_IRQn            = 159,
  MEC_ESPI_TAF_DONE_IRQn    = 166,
  MEC_ESPI_TAF_ERR_IRQn     = 167,
  MEC_WDT0_IRQn             = 171,
  MEC_GLUE_IRQn             = 172,
  MEC_PCR_CLKMON_IRQn       = 174,
  MEC_ACPI_EC0_IRQn         = 175,
  MEC_ACPI_EC1_IRQn         = 176,
  MEC_ACPI_EC2_IRQn         = 177,
  MEC_ACPI_EC3_IRQn         = 178,
  MEC_ACPI_EC4_IRQn         = 179,
  MEC_ACPI_PM1_IRQn         = 180,
  MEC_UART2_IRQn            = 183,
  MEC_UART3_IRQn            = 184,
  MEC_BRT0_IRQn             = 193,
} IRQn_Type;

#define __CM4_REV                 0x0201U
#define __NVIC_PRIO_BITS               3
#define __Vendor_SysTickConfig         0
#define __MPU_PRESENT                  1
#define __FPU_PRESENT                  1

#include "core_cm4.h"

#define MEC_SERIES_ID           5
#define MEC5_FAM5_ID            0x29u
#define MEC175X_FAM_ID          0x00290000u

#define XEC_ESPI_HW_VER_15 15
#define XEC_I2C_SMB_HW_VER 38
#define XEC_I2C_SMB_HAS_STOP_DETECT_IRQ
#define XEC_HAS_UART_LSR2

#ifdef __cplusplus
}
#endif

#endif /* __SOC_MICROCHIP_MEC_MEC174X_MEC174X_H */
