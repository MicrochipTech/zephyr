/*
 * Copyright (c) 2021 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_MCHP_PCR_H_
#define _SOC_MCHP_PCR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/arch/cpu.h>

/* slp_idx = [0, 4], bitpos = [0, 31] */
#define MCHP_XEC_ENC_PCR_SCR(slp_idx, bitpos)					\
	(((slp_idx) & 0x7u) | (((bitpos) & 0x1fu) << 3))

#define MCHP_XEC_PCR_SCR_ENCODE(slp_idx, bitpos, domain)			\
	((((uint32_t)(domain) & 0xff) << 24) | (((bitpos) & 0x1f) << 3)		\
	 | ((uint32_t)(slp_idx) & 0x7))

#define MCHP_XEC_PCR_SCR_GET_IDX(e)	((e) & 0x7u)
#define MCHP_XEC_PCR_SCR_GET_BITPOS(e)	(((e) & 0xf8u) >> 3)

/* cpu clock divider */
#define MCHP_XEC_CLK_CPU_MASK		GENMASK(7, 0)
#define MCHP_XEC_CLK_CPU_CLK_DIV_1	1u
#define MCHP_XEC_CLK_CPU_CLK_DIV_2	2u
#define MCHP_XEC_CLK_CPU_CLK_DIV_4	4u
#define MCHP_XEC_CLK_CPU_CLK_DIV_8	8u
#define MCHP_XEC_CLK_CPU_CLK_DIV_16	16u
#define MCHP_XEC_CLK_CPU_CLK_DIV_48	48u

/* slow clock divider */
#define MCHP_XEC_CLK_SLOW_MASK		GENMASK(8, 0)
#define MCHP_XEC_CLK_SLOW_CLK_DIV_100K	480u

#define MCHP_XEC_CLK_SRC_POS		24
#define MCHP_XEC_CLK_SRC_MASK		GENMASK(31, 24)

#define MCHP_XEC_CLK_SRC_GET(n)		\
	(((n) & MCHP_XEC_CLK_SRC_MASK) >> MCHP_XEC_CLK_SRC_POS)

#define MCHP_XEC_CLK_SRC_SET(v, c)	(((v) & ~MCHP_XEC_CLK_SRC_MASK) |\
	(((c) << MCHP_XEC_CLK_SRC_POS) & MCHP_XEC_CLK_SRC_MASK))

/*
 * b[31:24] = clock source
 * b[23:0] = clock source specific format
 */
struct mchp_xec_pcr_clk_ctrl {
	uint32_t pcr_info;
};

/* inline routines */
#define XEC_PCR_BASE		(mem_addr_t)(DT_REG_ADDR_BY_IDX(DT_NODELABEL(pcr), 0))
#define XEC_PCR_SLP_EN_BASE	(XEC_PCR_BASE + 0x30u)
#define XEC_PCR_RST_EN_BASE	(XEC_PCR_BASE + 0x70u)


static ALWAYS_INLINE void xec_pcr_sleep_en_set(uint8_t enc_pcr_scr)
{
	mem_addr_t raddr = XEC_PCR_SLP_EN_BASE + (MCHP_XEC_PCR_SCR_GET_IDX(enc_pcr_scr) * 4u);

	sys_set_bit(raddr, MCHP_XEC_PCR_SCR_GET_BITPOS(enc_pcr_scr));
}

static ALWAYS_INLINE void xec_pcr_sleep_en_clear(uint8_t enc_pcr_scr)
{
	mem_addr_t raddr = XEC_PCR_SLP_EN_BASE + (MCHP_XEC_PCR_SCR_GET_IDX(enc_pcr_scr) * 4u);

	sys_set_bit(raddr, MCHP_XEC_PCR_SCR_GET_BITPOS(enc_pcr_scr));
}

/* Reset a peripheral block */
static ALWAYS_INLINE void xec_pcr_reset_en(uint8_t enc_pcr_scr)
{
	mem_addr_t raddr = XEC_PCR_RST_EN_BASE + (MCHP_XEC_PCR_SCR_GET_IDX(enc_pcr_scr) * 4u);

	sys_set_bit(raddr, MCHP_XEC_PCR_SCR_GET_BITPOS(enc_pcr_scr));
}

#ifdef __cplusplus
}
#endif

#endif /* _SOC_MCHP_PCR_H_ */
