/*
 * Copyright (c) 2021 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/interrupt_controller/intc_mchp_xec_ecia.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>

/* pinctrl Node contains the base address of the GPIO Control Registers */
#define ECIA_XEC_REG_BASE ((struct ecia_regs *)(DT_REG_ADDR(DT_NODELABEL(ecia))))

#define MCHP_XEC_GIRQ_SIZE 20u

#define MCHP_FIRST_GIRQ         MCHP_FIRST_GIRQ_NOS
#define MCHP_LAST_GIRQ          MCHP_LAST_GIRQ_NOS
#define MCHP_XEC_DIRECT_CAPABLE MCHP_ECIA_DIRECT_BITMAP

#define GIRQ_ID_TO_BITPOS(id) ((id) + 8)

int mchp_xec_ecia_enable(int girq, int src)
{
	if ((girq < MCHP_FIRST_GIRQ) || (girq > MCHP_LAST_GIRQ) || (src < 0) || (src > 31)) {
		return -EINVAL;
	}

	struct ecia_regs *regs = ECIA_XEC_REG_BASE;

	/* write 1 to set */
	regs->GIRQ[girq - MCHP_FIRST_GIRQ].EN_SET = BIT(src);

	return 0;
}

int mchp_xec_ecia_info_enable(int ecia_info)
{
	uint8_t girq = (uint8_t)MCHP_XEC_ECIA_GIRQ(ecia_info);
	uint8_t src = (uint8_t)MCHP_XEC_ECIA_GIRQ_POS(ecia_info);

	return mchp_xec_ecia_enable(girq, src);
}

int mchp_xec_ecia_disable(int girq, int src)
{
	if ((girq < MCHP_FIRST_GIRQ) || (girq > MCHP_LAST_GIRQ) || (src < 0) || (src > 31)) {
		return -EINVAL;
	}

	struct ecia_regs *regs = ECIA_XEC_REG_BASE;

	/* write 1 to clear */
	regs->GIRQ[girq - MCHP_FIRST_GIRQ].EN_CLR = BIT(src);

	return 0;
}

int mchp_xec_ecia_info_disable(int ecia_info)
{
	uint8_t girq = (uint8_t)MCHP_XEC_ECIA_GIRQ(ecia_info);
	uint8_t src = (uint8_t)MCHP_XEC_ECIA_GIRQ_POS(ecia_info);

	return mchp_xec_ecia_disable(girq, src);
}

void mchp_xec_ecia_girq_aggr_en(uint8_t girq_num, uint8_t enable)
{
	struct ecia_regs *regs = ECIA_XEC_REG_BASE;

	if (enable) {
		regs->BLK_EN_SET = BIT(girq_num);
	} else {
		regs->BLK_EN_CLR = BIT(girq_num);
	}
}

void mchp_xec_ecia_girq_src_clr(uint8_t girq_num, uint8_t src_bit_pos)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return;
	}

	struct ecia_regs *regs = ECIA_XEC_REG_BASE;

	/* write 1 to clear */
	regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].SRC = BIT(src_bit_pos);
}

void mchp_xec_ecia_girq_src_en(uint8_t girq_num, uint8_t src_bit_pos)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return;
	}

	struct ecia_regs *regs = ECIA_XEC_REG_BASE;

	/* write 1 to set */
	regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].EN_SET = BIT(src_bit_pos);
}

void mchp_xec_ecia_girq_src_dis(uint8_t girq_num, uint8_t src_bit_pos)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return;
	}

	struct ecia_regs *regs = ECIA_XEC_REG_BASE;

	/* write 1 to clear */
	regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].EN_CLR = BIT(src_bit_pos);
}

void mchp_xec_ecia_girq_src_clr_bitmap(uint8_t girq_num, uint32_t bitmap)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return;
	}

	struct ecia_regs *regs = ECIA_XEC_REG_BASE;

	/* write 1 to clear */
	regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].SRC = bitmap;
}

void mchp_xec_ecia_girq_src_en_bitmap(uint8_t girq_num, uint32_t bitmap)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return;
	}

	struct ecia_regs *regs = ECIA_XEC_REG_BASE;

	/* write 1 to clear */
	regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].EN_SET = bitmap;
}

void mchp_xec_ecia_girq_src_dis_bitmap(uint8_t girq_num, uint32_t bitmap)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return;
	}

	struct ecia_regs *regs = ECIA_XEC_REG_BASE;

	/* write 1 to clear */
	regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].EN_CLR = bitmap;
}

uint32_t mchp_xec_ecia_girq_result(uint8_t girq_num)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return 0U;
	}

	struct ecia_regs *regs = ECIA_XEC_REG_BASE;

	return regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].RESULT;
}

void mchp_xec_ecia_nvic_clr_pend(uint32_t nvic_num)
{
	if (nvic_num >= ((SCnSCB->ICTR + 1) * 32)) {
		return;
	}

	NVIC_ClearPendingIRQ(nvic_num);
}

void mchp_xec_ecia_info_girq_aggr_en(int ecia_info, uint8_t enable)
{
	uint8_t girq_num = MCHP_XEC_ECIA_GIRQ(ecia_info);

	mchp_xec_ecia_girq_aggr_en(girq_num, enable);
}

void mchp_xec_ecia_info_girq_src_clr(int ecia_info)
{
	uint8_t girq_num = MCHP_XEC_ECIA_GIRQ(ecia_info);
	uint8_t bitpos = MCHP_XEC_ECIA_GIRQ_POS(ecia_info);

	mchp_xec_ecia_girq_src_clr(girq_num, bitpos);
}

void mchp_xec_ecia_info_girq_src_en(int ecia_info)
{
	uint8_t girq_num = MCHP_XEC_ECIA_GIRQ(ecia_info);
	uint8_t bitpos = MCHP_XEC_ECIA_GIRQ_POS(ecia_info);

	mchp_xec_ecia_girq_src_en(girq_num, bitpos);
}

void mchp_xec_ecia_info_girq_src_dis(int ecia_info)
{
	uint8_t girq_num = MCHP_XEC_ECIA_GIRQ(ecia_info);
	uint8_t bitpos = MCHP_XEC_ECIA_GIRQ_POS(ecia_info);

	mchp_xec_ecia_girq_src_dis(girq_num, bitpos);
}

uint32_t mchp_xec_ecia_info_girq_result(int ecia_info)
{
	uint8_t girq_num = MCHP_XEC_ECIA_GIRQ(ecia_info);

	return mchp_xec_ecia_girq_result(girq_num);
}

void mchp_xec_ecia_info_nvic_clr_pend(int ecia_info)
{
	uint8_t nvic_num = MCHP_XEC_ECIA_NVIC_DIRECT(ecia_info);

	mchp_xec_ecia_nvic_clr_pend(nvic_num);
}
