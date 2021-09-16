/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include "mec150x_pin.h"



#ifndef __ADL_MEC172X_H__
#define __ADL_MEC172X_H__

#define KSC_PLAT_NAME                   "ADL"

#define PLATFORM_DATA(x, y)  ((x) | ((y) << 8))

/* In ADL board id is 6-bits */
#define BOARD_ID_MASK        0x003Fu
#define BOM_ID_MASK          0x01C0u
#define FAB_ID_MASK          0x0600u
#define HW_STRAP_MASK        0xF800u
#define HW_ID_MASK           (FAB_ID_MASK|BOM_ID_MASK|BOARD_ID_MASK)
#define BOARD_ID_OFFSET      0u
#define BOM_ID_OFFSET        6u
#define FAB_ID_OFFSET        9u
#define HW_STRAP_OFFSET      11u

/* Support board ids */
#define BRD_ID_ADL_S_ERB                              0x19u
#define BRD_ID_ADL_S_S01_TGP_H_SODIMM_DRR4            0x20u
#define BRD_ID_ADL_S_S01_TGP_H_SODIMM_DRR4_CRB        0x21u
#define BRD_ID_ADL_S_S01_TGP_H_SODIMM_DRR4_PPV        0x22u
#define BRD_ID_ADL_S_S02_TGP_H_SODIMM_DRR4_CRB        0x24u
#define BRD_ID_ADL_S_S03_ADP_S_UDIMM_DRR4_ERB1        0x26u
#define BRD_ID_ADL_S_S03_ADP_S_UDIMM_DRR4_CRB         0x27u
#define BRD_ID_ADL_S_S04_ADP_S_UDIMM_DRR4_CRB_EVCRB   0x28u
#define BRD_ID_ADL_S_S05_ADP_S_UDIMM_DRR4_CRB_CPV     0x29u
#define BRD_ID_ADL_S_S06_ADP_S_UDIMM_DRR4_CRB         0x2Bu
#define BRD_ID_ADL_S_S09_ADP_S_UDIMM_DRR4_CRB_PPV     0x2Cu
#define BRD_ID_ADL_S_S07_ADP_S_UDIMM_DRR4_CPV         0x2Eu
#define BRD_ID_ADL_S_S08_ADP_S_SODIMM_DRR5_CRB        0x30u

/* I2C addresses */
#define EEPROM_DRIVER_I2C_ADDR          0x50
#define IO_EXPANDER_0_I2C_ADDR          0x22

/* Signal to gpio mapping for MEC1501 card + ICL platform is described here */

/* Following blue wires are required
 * PWRBTN_EC_OUT - from MECC TP65 to MECC J50.11 (GPIO250)
 * PWRBTN_EC_IN  - from MECC TP64 to MECC J50.8  (GPIO163)
 * PM_SLP_SUS    - from MECC TP55 to MECC J50.16
 * SYS_PWROK     - from MECC J51.15 (GPIO034) to RVP
 * PM_DS3        - from MECC J50.4  (GPIO165) to RVP
 * Simulated LEDS
 * SCROLL_LOCK   - J50.12 (GPIO101)
 * NUM_LOCK      - J50.10 (GPIO102)
 * CAPS_LOCK     - J50.14 (GPIO172)
 * 48MHZ TST_CLK - JP20.16
 */

/* GPIO expander ports, offset from valid gpios */
#define EC_EXP_PORT_1			MCHP_GPIO_MAX_PORT
#define EC_EXP_PORT_2			(EC_EXP_PORT_1 + 1U)

/* Dummy gpio port */
#define EC_DUMMY_GPIO_PORT		0xFU

/* Dummy gpio default low */
#define EC_DUMMY_GPIO_LOW	EC_GPIO_PORT_PIN(EC_DUMMY_GPIO_PORT, 0x00)
/* Dummy gpio default high */
#define EC_DUMMY_GPIO_HIGH	EC_GPIO_PORT_PIN(EC_DUMMY_GPIO_PORT, 0x01)

#define PROCHOT				EC_GPIO_002


#define RSMRST_PWRGD_G3SAF_P		EC_DUMMY_GPIO_HIGH
#define RSMRST_PWRGD_MAF_P		EC_GPIO_221
#define RSMRST_PWRGD			((boot_mode_maf == 1) ? \
					 RSMRST_PWRGD_MAF_P : \
					 RSMRST_PWRGD_G3SAF_P)

#define PM_PWRBTN			EC_GPIO_016

#define VOL_UP				EC_GPIO_036
#define STD_ADP_PRSNT			EC_GPIO_043
#define WAKE_SCI			EC_GPIO_051

#define PM_RSMRST_G3SAF_P		EC_GPIO_054
#define PM_RSMRST_MAF_P			EC_GPIO_055
#define PM_RSMRST			((boot_mode_maf == 1) ? \
					 PM_RSMRST_MAF_P : PM_RSMRST_G3SAF_P)

#define ALL_SYS_PWRGD			EC_GPIO_057

/* We poll this GPIO in MAF mode in order to sense the input signal.
 * This pin was already configured in pinmux as ALT mode 1 NOT GPIO
 */
#define ESPI_RESET_MAF			EC_GPIO_061

#define PCH_PWROK			EC_GPIO_106

#define FAN_PWR_DISABLE_N		EC_GPIO_141
#define BC_ACOK				EC_GPIO_156

#define SYS_PWROK			EC_GPIO_202

/* Not used in ADL-S */
#define BATT_ID				EC_GPIO_206

#define VCCST_PWRGD			EC_GPIO_207


/* Not used in ADL-S */
#define SLATE_MODE			EC_GPIO_222
#define SMC_LID				EC_GPIO_226

#define PM_SLP_SUS			EC_GPIO_227

#define PWRBTN_EC_IN_N			EC_GPIO_246
#define VOL_DOWN			EC_GPIO_254

/* VCI_IN0 */
#define PM_SLP_S0_CS			EC_DUMMY_GPIO_HIGH

/* VCI_OUT2 */
#define PM_DS3				EC_DUMMY_GPIO_HIGH

#define PM_BATLOW			EC_DUMMY_GPIO_HIGH
#define TYPEC_ALERT_2			EC_DUMMY_GPIO_HIGH
#define TYPEC_ALERT_1			EC_DUMMY_GPIO_HIGH
#define KBC_SCROLL_LOCK			EC_DUMMY_GPIO_HIGH
#define KBC_NUM_LOCK			EC_DUMMY_GPIO_HIGH
#define KBC_CAPS_LOCK			EC_DUMMY_GPIO_HIGH
#define PM_SLP_S0_N			EC_DUMMY_GPIO_HIGH
#define EC_PG3_EXIT			EC_DUMMY_GPIO_LOW
#define EC_PWRBTN_LED			EC_DUMMY_GPIO_LOW

#define VIRTUAL_DOCK			EC_DUMMY_GPIO_LOW
#define VIRTUAL_BAT			EC_DUMMY_GPIO_LOW
#define HOME_BUTTON			EC_DUMMY_GPIO_HIGH
#define DG2_PRESENT			EC_DUMMY_GPIO_LOW
#define PEG_RTD3_COLD_MOD_SW_R		EC_DUMMY_GPIO_LOW
#define EC_PWRBTN_LED			EC_DUMMY_GPIO_LOW

/* IO expander HW strap pins */
#define SPD_PRSNT			EC_GPIO_PORT_PIN(EC_EXP_PORT_1, 0x03)
#define G3_SAF_DETECT			EC_GPIO_PORT_PIN(EC_EXP_PORT_1, 0x04)
#define THERM_STRAP			EC_GPIO_PORT_PIN(EC_EXP_PORT_1, 0x05)
#define PECI_OVER_ESPI			EC_GPIO_PORT_PIN(EC_EXP_PORT_1, 0x06)
#define TIMEOUT_DISABLE			EC_GPIO_PORT_PIN(EC_EXP_PORT_1, 0x07)

/* Device instance names */
#define I2C_BUS_0			DT_LABEL(DT_NODELABEL(i2c_smb_0))
#define I2C_BUS_1			DT_LABEL(DT_NODELABEL(i2c_smb_1))
#define PS2_KEYBOARD			DT_LABEL(DT_NODELABEL(ps2_0))
#define PS2_MOUSE			DT_LABEL(DT_NODELABEL(ps2_1))
#define ESPI_0				DT_LABEL(DT_NODELABEL(espi0))
#define ESPI_SAF_0			DT_LABEL(DT_NODELABEL(espi_saf0))
#define SPI_0				DT_LABEL(DT_NODELABEL(spi0))
#define ADC_CH_BASE			DT_LABEL(DT_NODELABEL(adc0))
#define PECI_0_INST			DT_LABEL(DT_NODELABEL(peci0))
#define WDT_0				DT_LABEL(DT_NODELABEL(wdog))
#define KSCAN_MATRIX			DT_LABEL(DT_NODELABEL(kscan0))

/* Button/Switch Initial positions */
#define PWR_BTN_INIT_POS		1
#define VOL_UP_INIT_POS			1
#define VOL_DN_INIT_POS			1
#define LID_INIT_POS			1
#define HOME_INIT_POS			1
#define VIRTUAL_BAT_INIT_POS		1
#define VIRTUAL_DOCK_INIT_POS		1

#endif /* __ADL_MEC172X_H__ */
