/*
 * Copyright (c) 2022 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_MICROCHIP_MEC_COMMON_REG_MEC_GPIO_H
#define _SOC_MICROCHIP_MEC_COMMON_REG_MEC_GPIO_H

#include <stdint.h>
#include <stddef.h>

#include <zephyr/arch/cpu.h>

#define MCHP_XEC_PINCTRL_REG_IDX(pin)		((pin >> 5) * 32 + (pin & 0x1f))

#define MCHP_XEC_GPIO_CR1_OFS_BP(bank, pos)	(((uint32_t)(bank) * 0x80u) + ((pos) * 4u))

#define MCHP_XEC_GPIO_CR1_OFS(pin)		((uint32_t)(pin) * 4u)
#define MCHP_XEC_GPIO_CR2_OFS(pin)		(MCHP_XEC_GPIO_CR1_OFS(pin) + 0x500u)
#define MCHP_XEC_GPIO_PAR_IN_OFS(pin)		((((uint32_t)pin & 0xffe0u) / 8u) + 0x300u)
#define MCHP_XEC_GPIO_PAR_OUT_OFS(pin)		((((uint32_t)pin & 0xffe0u) / 8u) + 0x380u)

/* Each GPIO pin has two 32-bit control registers */
#define MCHP_XEC_GPIO_CR1_PUD_POS		0
#define MCHP_XEC_GPIO_CR1_PUD_MSK		GENMASK(1, 0)
#define MCHP_XEC_GPIO_CR1_PUD_NONE		0
#define MCHP_XEC_GPIO_CR1_PUD_PU		1u
#define MCHP_XEC_GPIO_CR1_PUD_PD		2u
#define MCHP_XEC_GPIO_CR1_PUD_RPT		3u
#define MCHP_XEC_GPIO_CR1_PUD_SET(x)		FIELD_PREP(MCHP_XEC_GPIO_CR1_PUD_MSK, (x))
#define MCHP_XEC_GPIO_CR1_PUD_GET(x)		FIELD_GET(MCHP_XEC_GPIO_CR1_PUD_MSK, (x))
#define MCHP_XEC_GPIO_CR1_PGS_POS		2
#define MCHP_XEC_GPIO_CR1_PGS_MSK		GENMASK(3, 2)
#define MCHP_XEC_GPIO_CR1_PGS_VTR		0
#define MCHP_XEC_GPIO_CR1_PGS_VCC		1u
#define MCHP_XEC_GPIO_CR1_PGS_OFF		2u
#define MCHP_XEC_GPIO_CR1_PGS_ALWAYS		3u
#define MCHP_XEC_GPIO_CR1_PGS_SET(x)		FIELD_PREP(MCHP_XEC_GPIO_CR1_PGS_MSK, (x))
#define MCHP_XEC_GPIO_CR1_PGS_GET(x)		FIELD_GET(MCHP_XEC_GPIO_CR1_PGS_MSK, (x))
#define MCHP_XEC_GPIO_CR1_ID_POS		4
#define MCHP_XEC_GPIO_CR1_ID_MSK		GENMASK(7, 4)
#define MCHP_XEC_GPIO_CR1_ID_LL			0
#define MCHP_XEC_GPIO_CR1_ID_HL			1u
#define MCHP_XEC_GPIO_CR1_ID_DIS		4u
#define MCHP_XEC_GPIO_CR1_ID_RE			0xdu
#define MCHP_XEC_GPIO_CR1_ID_FE			0xeu
#define MCHP_XEC_GPIO_CR1_ID_BE			0xfu
#define MCHP_XEC_GPIO_CR1_ID_SET(x)		FIELD_PREP(MCHP_XEC_GPIO_CR1_ID_MSK, (x))
#define MCHP_XEC_GPIO_CR1_ID_GET(x)		FIELD_GET(MCHP_XEC_GPIO_CR1_ID_MSK, (x))
#define MCHP_XEC_GPIO_CR1_OOD_POS		8
#define MCHP_XEC_GPIO_CR1_DIR_OUT_POS		9
#define MCHP_XEC_GPIO_CR1_OCS_POS		10
#define MCHP_XEC_GPIO_CR1_POLINV_POS		11
#define MCHP_XEC_GPIO_CR1_MUX_POS		12
#define MCHP_XEC_GPIO_CR1_MUX_MSK		GENMASK(14, 12)
#define MCHP_XEC_GPIO_CR1_MUX_SET(x)		FIELD_PREP(MCHP_XEC_GPIO_CR1_MUX_MSK, (x))
#define MCHP_XEC_GPIO_CR1_MUX_GET(x)		FIELD_GET(MCHP_XEC_GPIO_CR1_MUX_MSK, (x))
#define MCHP_XEC_GPIO_CR1_IN_DIS_POS		15
#define MCHP_XEC_GPIO_CR1_OUT_DAT_POS		16
#define MCHP_XEC_GPIO_CR1_IN_PAD_POS		24

#define MCHP_XEC_GPIO_CR2_SLEW_SLOW_POS		0
#define MCHP_XEC_GPIO_CR2_DRV_STR_POS		4
#define MCHP_XEC_GPIO_CR2_DRV_STR_MSK		GENMASK(5, 4)
#define MCHP_XEC_GPIO_CR2_DRV_STR_1X		0
#define MCHP_XEC_GPIO_CR2_DRV_STR_2X		1u
#define MCHP_XEC_GPIO_CR2_DRV_STR_4X		2u
#define MCHP_XEC_GPIO_CR2_DRV_STR_6X		3u
#define MCHP_XEC_GPIO_CR2_DRV_STR_SET(x)	FIELD_PREP(MCHP_XEC_GPIO_CR2_DRV_STR_MSK, (x))
#define MCHP_XEC_GPIO_CR2_DRV_STR_GET(x)	FIELD_GET(MCHP_XEC_GPIO_CR2_DRV_STR_MSK, (x))

/* to be deprecated */
struct gpio_regs {
	volatile uint32_t  CTRL[174];
	uint32_t  RESERVED[18];
	volatile uint32_t  PARIN[6];
	uint32_t  RESERVED1[26];
	volatile uint32_t  PAROUT[6];
	uint32_t  RESERVED2[20];
	volatile uint32_t  LOCK[6];
	uint32_t  RESERVED3[64];
	volatile uint32_t  CTRL2[174];
};

#endif	/* _SOC_MICROCHIP_MEC_COMMON_REG_MEC_GPIO_H */
