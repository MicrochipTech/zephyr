/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/common/sys_io.h>
#include <soc.h>
#include "soc_ecia.h"

#if 0
/* ARMv7m NVIC */
#define ARM_NVIC_REG_BASE	(mem_addr_t)DT_REG_ADDR(DT_NODELABEL(nvic))
#define ARM_NVIC_ISER_OFS	0	/* set enable */
#define ARM_NVIC_ICER_OFS	0x80u	/* clear enable */
#define ARM_NVIC_ISPR_OFS	0x100u	/* set pending */
#define ARM_NVIC_ICPR_OFS	0x180u	/* clear pending */
#define ARM_NVIC_IABR_OFS	0x200u	/* read-only active status */
#define ARM_NVIC_IPR_OFS	0x300u	/* array of 8-bit priority registers */

/* interrupt number is signed and must be >= 0
 * register access surrounded by __COMPILER_BARRIER
 */
#endif

#if 0
/* EC Interrupt Aggregator. It is not real interrupt controller. */
#define MCHP_XEC_ECIA_REG_BASE	(mem_addr_t)DT_REG_ADDR(DT_NODELABEL(ecia))
#endif

#ifdef CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER

#define GIRQ_ENC(g, b)		(((uint16_t)(g) & 0xffu) | (((uint16_t)(b) & 0x1fu) << 8))
#define GIRQ_ENC_GET_NUM(e)	((e) & 0xffu)
#define GIRQ_ENC_GET_BITPOS(e)	(((e) >> 8) & 0x1fu)

/* NVIC external interrupt number (>= 0) used as index to
 * obtain encoded girq number and bit position in girq.
 */
const uint16_t girq_lookup_tbl[] = {
	[20] = GIRQ_ENC(13, 0),
	[21] = GIRQ_ENC(13, 1),
	[22] = GIRQ_ENC(13, 2),
	[23] = GIRQ_ENC(13, 3),
	[24] = GIRQ_ENC(14, 0),
	[25] = GIRQ_ENC(14, 1),
	[26] = GIRQ_ENC(14, 2),
	[27] = GIRQ_ENC(14, 3),
	[28] = GIRQ_ENC(14, 4),
	[29] = GIRQ_ENC(14, 5),
	[30] = GIRQ_ENC(14, 6),
	[31] = GIRQ_ENC(14, 7),
	[32] = GIRQ_ENC(14, 8),
	[33] = GIRQ_ENC(14, 9),
	[34] = GIRQ_ENC(14, 10),
	[35] = GIRQ_ENC(14, 11),
	[36] = GIRQ_ENC(14, 12),
	[37] = GIRQ_ENC(14, 13),
	[38] = GIRQ_ENC(14, 14),
	[39] = GIRQ_ENC(14, 15),
	[40] = GIRQ_ENC(15, 0),
	[41] = GIRQ_ENC(15, 1),
	[42] = GIRQ_ENC(15, 2),
	[43] = GIRQ_ENC(15, 3),
	[44] = GIRQ_ENC(15, 4),
	[45] = GIRQ_ENC(15, 5),
	[46] = GIRQ_ENC(15, 6),
	[47] = GIRQ_ENC(15, 7),
	[48] = GIRQ_ENC(15, 8),
	[49] = GIRQ_ENC(15, 9),
	[50] = GIRQ_ENC(15, 10),
	[51] = GIRQ_ENC(15, 11),
	[52] = GIRQ_ENC(15, 12),
	[53] = GIRQ_ENC(15, 13),
	[54] = GIRQ_ENC(15, 14),
	[55] = GIRQ_ENC(15, 15),
	[56] = GIRQ_ENC(15, 16),
	[57] = GIRQ_ENC(15, 17),
	[58] = GIRQ_ENC(15, 18),
	[59] = GIRQ_ENC(15, 19),
	[60] = GIRQ_ENC(15, 20),
	[62] = GIRQ_ENC(15, 22),
	[64] = GIRQ_ENC(15, 24),
	[65] = GIRQ_ENC(16, 0),
	[67] = GIRQ_ENC(16, 2),
	[68] = GIRQ_ENC(16, 3),
	[70] = GIRQ_ENC(17, 0),
	[71] = GIRQ_ENC(17, 1),
	[72] = GIRQ_ENC(17, 2),
	[73] = GIRQ_ENC(17, 3),
	[74] = GIRQ_ENC(17, 20),
	[75] = GIRQ_ENC(17, 21),
	[76] = GIRQ_ENC(17, 22),
	[77] = GIRQ_ENC(17, 23),
	[78] = GIRQ_ENC(17, 8),
	[79] = GIRQ_ENC(17, 9),
	[80] = GIRQ_ENC(17, 10),
	[81] = GIRQ_ENC(17, 11),
	[82] = GIRQ_ENC(17, 12),
	[83] = GIRQ_ENC(17, 13),
	[84] = GIRQ_ENC(17, 14),
	[85] = GIRQ_ENC(17, 15),
	[86] = GIRQ_ENC(17, 16),
	[87] = GIRQ_ENC(17, 17),
	[88] = GIRQ_ENC(17, 18),
	[89] = GIRQ_ENC(17, 19),
	[90] = GIRQ_ENC(18, 0),
	[91] = GIRQ_ENC(18, 1),
	[92] = GIRQ_ENC(18, 2),
	[93] = GIRQ_ENC(18, 3),
	[94] = GIRQ_ENC(18, 4),
	[95] = GIRQ_ENC(18, 5),
	[96] = GIRQ_ENC(18, 7),
	[97] = GIRQ_ENC(18, 6),
	[100] = GIRQ_ENC(18, 10),
	[101] = GIRQ_ENC(18, 11),
	[103] = GIRQ_ENC(19, 0),
	[104] = GIRQ_ENC(19, 1),
	[105] = GIRQ_ENC(19, 2),
	[106] = GIRQ_ENC(19, 3),
	[107] = GIRQ_ENC(19, 4),
	[108] = GIRQ_ENC(19, 5),
	[109] = GIRQ_ENC(19, 6),
	[110] = GIRQ_ENC(19, 7),
	[111] = GIRQ_ENC(23, 10),
	[112] = GIRQ_ENC(23, 16),
	[113] = GIRQ_ENC(23, 17),
	[114] = GIRQ_ENC(21, 3),
	[115] = GIRQ_ENC(21, 4),
	[116] = GIRQ_ENC(21, 5),
	[117] = GIRQ_ENC(21, 6),
	[118] = GIRQ_ENC(21, 7),
	[119] = GIRQ_ENC(21, 8),
	[120] = GIRQ_ENC(21, 9),
	[121] = GIRQ_ENC(21, 10),
	[122] = GIRQ_ENC(21, 11),
	[123] = GIRQ_ENC(21, 12),
	[124] = GIRQ_ENC(21, 13),
	[125] = GIRQ_ENC(21, 14),
	[126] = GIRQ_ENC(21, 15),
	[129] = GIRQ_ENC(21, 18),
	[130] = GIRQ_ENC(21, 19),
	[132] = GIRQ_ENC(21, 21),
	[134] = GIRQ_ENC(21, 24),
	[135] = GIRQ_ENC(21, 25),
	[136] = GIRQ_ENC(23, 0),
	[137] = GIRQ_ENC(23, 1),
	[138] = GIRQ_ENC(23, 2),
	[139] = GIRQ_ENC(23, 3),
	[140] = GIRQ_ENC(23, 4),
	[141] = GIRQ_ENC(23, 5),
	[142] = GIRQ_ENC(23, 6),
	[143] = GIRQ_ENC(23, 7),
	[144] = GIRQ_ENC(23, 8),
	[145] = GIRQ_ENC(23, 9),
	[146] = GIRQ_ENC(18, 20),
	[147] = GIRQ_ENC(18, 21),
	[148] = GIRQ_ENC(18, 22),
	[149] = GIRQ_ENC(18, 23),
	[150] = GIRQ_ENC(18, 24),
	[151] = GIRQ_ENC(18, 25),
	[152] = GIRQ_ENC(18, 26),
	[153] = GIRQ_ENC(18, 27),
	[154] = GIRQ_ENC(18, 28),
	[155] = GIRQ_ENC(18, 13),
	[156] = GIRQ_ENC(19, 8),
	[158] = GIRQ_ENC(13, 4),
	[159] = GIRQ_ENC(17, 4),
	[166] = GIRQ_ENC(19, 9),
	[167] = GIRQ_ENC(19, 10),
	[171] = GIRQ_ENC(21, 2),
	[172] = GIRQ_ENC(21, 26),
	[173] = GIRQ_ENC(20, 3),
	[174] = GIRQ_ENC(20, 9),
	[181] = GIRQ_ENC(13, 8),
	[182] = GIRQ_ENC(13, 9),
	[183] = GIRQ_ENC(15, 25),
	[184] = GIRQ_ENC(15, 26),
	[185] = GIRQ_ENC(17, 5),
	[186] = GIRQ_ENC(17, 6),
	[187] = GIRQ_ENC(17, 7),
	[188] = GIRQ_ENC(17, 24),
	[189] = GIRQ_ENC(17, 25),
	[190] = GIRQ_ENC(17, 26),
	[191] = GIRQ_ENC(15, 27),
	[192] = GIRQ_ENC(13, 10),
	[193] = GIRQ_ENC(21, 27),
	[194] = GIRQ_ENC(14, 16),
	[195] = GIRQ_ENC(14, 17),
	[196] = GIRQ_ENC(14, 18),
	[197] = GIRQ_ENC(14, 19),
};

static inline uint16_t lookup_girq(unsigned int irq)
{
	if (irq >= ARRAY_SIZE(girq_lookup_tbl)) {
		return 0;
	}

	return girq_lookup_tbl[irq];
}

/* ARM Cortex-M NVIC z_arm_interrupt_init is not built when CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER
 * is set. Duplicate it here and add any ECIA config.
 */
void z_soc_irq_init(void)
{
	int irq = 0;

	soc_ecia_init(MCHP_MEC_ECIA_GIRQ_AGGR_ONLY_BM, MCHP_MEC_ECIA_GIRQ_DIRECT_CAP_BM, 0);

/* CONFIG_2ND_LVL_ISR_TBL_OFFSET could be treated as total number of level1 interrupts */
#if defined(CONFIG_MULTI_LEVEL_INTERRUPTS) && defined(CONFIG_2ND_LVL_ISR_TBL_OFFSET)
	for (; irq < CONFIG_2ND_LVL_ISR_TBL_OFFSET; irq++) {
#else
	for (; irq < CONFIG_NUM_IRQS; irq++) {
#endif
		NVIC_SetPriority((IRQn_Type)irq, _IRQ_PRIO_OFFSET);
	}
}

void z_soc_irq_enable(unsigned int irq)
{
	uint16_t girq_enc = lookup_girq(irq);
	uint8_t girq = GIRQ_ENC_GET_NUM(girq_enc);
	uint8_t bitpos = GIRQ_ENC_GET_BITPOS(girq_enc);

	soc_ecia_girq_ctrl(girq, bitpos, 1u);
	NVIC_EnableIRQ((IRQn_Type)irq);
}

void z_soc_irq_disable(unsigned int irq)
{
	uint16_t girq_enc = lookup_girq(irq);
	uint8_t girq = GIRQ_ENC_GET_NUM(girq_enc);
	uint8_t bitpos = GIRQ_ENC_GET_BITPOS(girq_enc);

	soc_ecia_girq_ctrl(girq, bitpos, 0);
	NVIC_DisableIRQ((IRQn_Type)irq);
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	uint16_t girq_enc = lookup_girq(irq);
	uint8_t girq = GIRQ_ENC_GET_NUM(girq_enc);
	uint8_t bitpos = GIRQ_ENC_GET_BITPOS(girq_enc);

	if (NVIC_GetEnableIRQ((IRQn_Type)irq) == 0) {
		return 0;
	}

	if ((soc_ecia_girq_get_enable_bm(girq) & BIT(bitpos)) == 0) {
		return 0;
	}

	return 1;
}

/* MEC ECIA is not an interrupt controller. No priorities are implemented.
 * We duplicate Zephyr's cortex_m architecture code since it is compiled out
 * when CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER is enabled.
 */
void z_soc_irq_priority_set(unsigned int irq, unsigned int prio, unsigned int flags)
{
	/* The kernel may reserve some of the highest priority levels.
	 * So we offset the requested priority level with the number
	 * of priority levels reserved by the kernel.
	 */

	/* If we have zero latency interrupts, those interrupts will
	 * run at a priority level which is not masked by irq_lock().
	 * Our policy is to express priority levels with special properties
	 * via flags
	 */
	if (IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) && (flags & IRQ_ZERO_LATENCY)) {
		if (ZERO_LATENCY_LEVELS == 1) {
			prio = _EXC_ZERO_LATENCY_IRQS_PRIO;
		} else {
			/* Use caller supplied prio level as-is */
		}
	} else {
		prio += _IRQ_PRIO_OFFSET;
	}

	/* The last priority level is also used by PendSV exception, but
	 * allow other interrupts to use the same level, even if it ends up
	 * affecting performance (can still be useful on systems with a
	 * reduced set of priorities, like Cortex-M0/M0+).
	 */
	__ASSERT(prio <= (BIT(NUM_IRQ_PRIO_BITS) - 1),
		 "invalid priority %d for %d irq! values must be less than %lu\n",
		 prio - _IRQ_PRIO_OFFSET, irq, BIT(NUM_IRQ_PRIO_BITS) - (_IRQ_PRIO_OFFSET));
	NVIC_SetPriority((IRQn_Type)irq, prio);
}

unsigned int z_soc_irq_get_active(void)
{
	return (unsigned int)__get_IPSR();
}

void z_soc_irq_eoi(unsigned int irq)
{
	uint16_t girq_enc = lookup_girq(irq);
	uint8_t girq = GIRQ_ENC_GET_NUM(girq_enc);
	uint8_t bitpos = GIRQ_ENC_GET_BITPOS(girq_enc);

	soc_ecia_girq_status_clear(girq, bitpos);
}

#endif /* CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER */