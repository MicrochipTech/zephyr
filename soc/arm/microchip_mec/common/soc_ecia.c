/*
 * Copyright (c) 2021 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/__assert.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <kernel.h>
#include <arch/cpu.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>

#define ECIA_XEC_REG_BASE						\
	((struct ecia_ar_regs *)(DT_REG_ADDR(DT_NODELABEL(ecia))))

void mchp_soc_ecia_girq_src_clr(uint8_t girq_num, uint8_t src_bit_pos)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return;
	}

	struct ecia_ar_regs *regs = ECIA_XEC_REG_BASE;

	/* write 1 to clear */
	regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].SRC = BIT(src_bit_pos);
}

void mchp_soc_ecia_girq_src_en(uint8_t girq_num, uint8_t src_bit_pos)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return;
	}

	struct ecia_ar_regs *regs = ECIA_XEC_REG_BASE;

	/* write 1 to set */
	regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].EN_SET = BIT(src_bit_pos);
}

void mchp_soc_ecia_girq_src_dis(uint8_t girq_num, uint8_t src_bit_pos)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return;
	}

	struct ecia_ar_regs *regs = ECIA_XEC_REG_BASE;

	/* write 1 to clear */
	regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].EN_CLR = BIT(src_bit_pos);
}

void mchp_soc_ecia_girq_src_clr_bitmap(uint8_t girq_num, uint32_t bitmap)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return;
	}

	struct ecia_ar_regs *regs = ECIA_XEC_REG_BASE;

	/* write 1 to clear */
	regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].SRC = bitmap;
}

void mchp_soc_ecia_girq_src_en_bitmap(uint8_t girq_num, uint32_t bitmap)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return;
	}

	struct ecia_ar_regs *regs = ECIA_XEC_REG_BASE;

	/* write 1 to clear */
	regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].EN_SET = bitmap;
}

void mchp_soc_ecia_girq_src_dis_bitmap(uint8_t girq_num, uint32_t bitmap)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return;
	}

	struct ecia_ar_regs *regs = ECIA_XEC_REG_BASE;

	/* write 1 to clear */
	regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].EN_CLR = bitmap;
}

/*
 * Return read-only GIRQ result register. Result is bit-wise and of source
 * and enable registers.
 */
uint32_t mchp_soc_ecia_girq_result(uint8_t girq_num)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return 0U;
	}

	struct ecia_ar_regs *regs = ECIA_XEC_REG_BASE;

	return regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].RESULT;
}

/*
 * Enable/disable specified GIRQ's aggregated output. Aggrated output is the
 * bit-wise or of all the GIRQ's result bits.
 */
void mchp_soc_ecia_girq_aggr_en(uint8_t girq_num, uint8_t enable)
{
	struct ecia_ar_regs *regs = ECIA_XEC_REG_BASE;

	if (enable) {
		regs->BLK_EN_SET = BIT(girq_num);
	} else {
		regs->BLK_EN_CLR = BIT(girq_num);
	}
}

/* Clear NVIC pending given the external NVIC input number (zero based) */
void mchp_soc_ecia_nvic_clr_pend(uint32_t nvic_num)
{
	if (nvic_num >= MCHP_MAX_NVIC_EXT_INPUTS) {
		return;
	}

	uint32_t regofs = (nvic_num / 32u) * 4u;
	uint32_t bitpos = nvic_num % 32u;
	uint32_t addr = MCHP_NVIC_CLR_PEND_BASE + regofs;

	/* NVIC Clear Pending register: write 1 to clear bit(s) */
	sys_write32(BIT(bitpos), addr);
}
