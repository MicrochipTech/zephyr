/*
 * Copyright (c) 2021 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for External interrupt controller in Microchip XEC devices
 *
 * Driver is currently implemented to support MEC172x ECIA GIRQs
 */

#define DT_DRV_COMPAT microchip_xec_ecia

#include <arch/cpu.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>
#include <device.h>
#include <soc.h>
#include <sys/__assert.h>
#include <drivers/clock_control/mchp_xec_clock_control.h>
#include <drivers/interrupt_controller/intc_mchp_xec_ecia.h>
#include "intc_mchp_ecia_xec_priv.h"

#define GIRQ_ID_TO_BITPOS(id) ((id) + 8)

/*
 * MEC SoC's have one and only one instance of ECIA. GIRQ8 register are located
 * at the beginning of the ECIA block.
 */
#define ECIA_XEC_REG_BASE						\
	((struct ecia_regs *)(DT_REG_ADDR(DT_NODELABEL(ecia))))

#define ECS_XEC_REG_BASE						\
	((struct ecs_regs *)(DT_REG_ADDR(DT_NODELABEL(ecs))))

#define ECIA_XEC_PCR_INFO						\
	MCHP_XEC_PCR_SCR_ENCODE(DT_INST_CLOCKS_CELL(0, regidx),		\
				DT_INST_CLOCKS_CELL(0, bitpos))

#define GIRQ08_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq8), okay))
#define GIRQ09_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq9), okay))
#define GIRQ10_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq10), okay))
#define GIRQ11_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq11), okay))
#define GIRQ12_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq12), okay))

#define GIRQ13_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq13), okay) && \
	!(DT_IRQ_HAS_CELL(DT_NODELABEL(girq13), direct_enable)))
#define GIRQ14_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq14), okay) && \
	!(DT_IRQ_HAS_CELL(DT_NODELABEL(girq14), direct_enable)))
#define GIRQ15_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq15), okay) && \
	!(DT_IRQ_HAS_CELL(DT_NODELABEL(girq15), direct_enable)))
#define GIRQ16_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq16), okay) && \
	!(DT_IRQ_HAS_CELL(DT_NODELABEL(girq16), direct_enable)))
#define GIRQ17_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq17), okay) && \
	!(DT_IRQ_HAS_CELL(DT_NODELABEL(girq17), direct_enable)))
#define GIRQ18_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq18), okay) && \
	!(DT_IRQ_HAS_CELL(DT_NODELABEL(girq18), direct_enable)))
#define GIRQ19_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq19), okay) && \
	!(DT_IRQ_HAS_CELL(DT_NODELABEL(girq19), direct_enable)))
#define GIRQ20_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq20), okay) && \
	!(DT_IRQ_HAS_CELL(DT_NODELABEL(girq20), direct_enable)))
#define GIRQ21_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq21), okay) && \
	!(DT_IRQ_HAS_CELL(DT_NODELABEL(girq21), direct_enable)))
#define GIRQ23_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq23), okay) && \
	!(DT_IRQ_HAS_CELL(DT_NODELABEL(girq23), direct_enable)))

#define GIRQ24_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq24), okay))
#define GIRQ25_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq25), okay))
#define GIRQ26_AGGR (DT_NODE_HAS_STATUS(DT_NODELABEL(girq26), okay))

struct xec_girq_config {
	uintptr_t base;
	uint32_t src_bitmap;
	uint8_t num_srcs;
	uint8_t girq_id;
};

struct xec_ecia_config {
	uintptr_t ecia_base;
	uint32_t direct_capable;
	struct mchp_xec_pcr_clk_ctrl clk_ctrl;
};

struct xec_girq_src_data {
	mchp_xec_ecia_callback_t cb;
	void *data;
};

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

/*
 * Return read-only GIRQ result register. Result is bit-wise and of source
 * and enable registers.
 */
uint32_t mchp_xec_ecia_girq_result(uint8_t girq_num)
{
	if ((girq_num < MCHP_FIRST_GIRQ) || (girq_num > MCHP_LAST_GIRQ)) {
		return 0U;
	}

	struct ecia_regs *regs = ECIA_XEC_REG_BASE;

	return regs->GIRQ[girq_num - MCHP_FIRST_GIRQ].RESULT;
}

/*
 * Enable/disable specified GIRQ's aggregated output. Aggrated output is the
 * bit-wise or of all the GIRQ's result bits.
 */
void mchp_xec_ecia_girq_aggr_en(uint8_t girq_num, uint8_t enable)
{
	struct ecia_regs *regs = ECIA_XEC_REG_BASE;

	if (enable) {
		regs->BLK_EN_SET = BIT(girq_num);
	} else {
		regs->BLK_EN_CLR = BIT(girq_num);
	}
}

/* Clear NVIC pending given the external NVIC input number (zero based) */
void mchp_xec_ecia_nvic_clr_pend(uint32_t nvic_num)
{
	if (nvic_num >= ((SCnSCB->ICTR + 1) * 32)) {
		return;
	}

	NVIC_ClearPendingIRQ(nvic_num);
}

/* dev is a pointer to a GIRQn child device */
#if (GIRQ08_AGGR || GIRQ09_AGGR || GIRQ10_AGGR || GIRQ11_AGGR || GIRQ12_AGGR ||\
     GIRQ12_AGGR || GIRQ13_AGGR || GIRQ14_AGGR || GIRQ15_AGGR || GIRQ16_AGGR ||\
     GIRQ17_AGGR || GIRQ18_AGGR || GIRQ19_AGGR || GIRQ20_AGGR || GIRQ21_AGGR ||\
     GIRQ23_AGGR || GIRQ24_AGGR || GIRQ25_AGGR || GIRQ26_AGGR)
static void xec_girq_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
#if 0
	struct ecia_regs *const ecia = DEV_REGS(dev);
	struct girq_regs *girq = &ecia->GIRQ[girqz];
	uint32_t result = girq->RESULT;

	for (int i = 0; result && i < 32; i++) {
		if (result & BIT(i)) {
			/* invoke call back */
			girq->SRC = BIT(i);
			result &= ~BIT(i);
		}
	}
#endif
}
#endif

/*
 * NOTE: property girq_id is zero based: [0, 18] -> [GIRQ8, GIRQ26]
 */
static void xec_connect_girqs(const struct device *dev)
{
	ARG_UNUSED(dev);

#if GIRQ08_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq8), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq8)),
		    DT_IRQ(DT_NODELABEL(girq8), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq8)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq8)));
#endif
#if GIRQ09_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq9), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq9)),
		    DT_IRQ(DT_NODELABEL(girq9), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq9)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq9)));
#endif
#if GIRQ10_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq10), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq10)),
		    DT_IRQ(DT_NODELABEL(girq10), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq10)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq10)));
#endif
#if GIRQ11_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq11), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq11)),
		    DT_IRQ(DT_NODELABEL(girq11), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq11)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq11)));
#endif
#if GIRQ12_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq12), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq12)),
		    DT_IRQ(DT_NODELABEL(girq12), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq12)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq12)));
#endif
#if GIRQ13_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq13), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq13)),
		    DT_IRQ(DT_NODELABEL(girq13), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq13)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq13)));
#endif
#if GIRQ13_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq13), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq13)),
		    DT_IRQ(DT_NODELABEL(girq13), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq13)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq13)));
#endif
#if GIRQ14_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq14), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq14)),
		    DT_IRQ(DT_NODELABEL(girq14), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq14)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq14)));
#endif
#if GIRQ15_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq15), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq15)),
		    DT_IRQ(DT_NODELABEL(girq15), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq15)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq15)));
#endif
#if GIRQ16_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq16), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq16)),
		    DT_IRQ(DT_NODELABEL(girq16), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq16)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq16)));
#endif
#if GIRQ17_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq17), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq17)),
		    DT_IRQ(DT_NODELABEL(girq17), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq17)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq17)));
#endif
#if GIRQ18_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq18), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq18)),
		    DT_IRQ(DT_NODELABEL(girq18), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq18)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq18)));
#endif
#if GIRQ19_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq19), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq19)),
		    DT_IRQ(DT_NODELABEL(girq19), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq19)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq19)));
#endif
#if GIRQ20_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq20), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq20)),
		    DT_IRQ(DT_NODELABEL(girq20), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq20)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq20)));
#endif
#if GIRQ21_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq21), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq21)),
		    DT_IRQ(DT_NODELABEL(girq21), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq21)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq21)));
#endif
#if GIRQ23_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq23), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq23)),
		    DT_IRQ(DT_NODELABEL(girq23), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq23)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq23)));
#endif
#if GIRQ24_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq24), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq24)),
		    DT_IRQ(DT_NODELABEL(girq24), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq24)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq24)));
#endif
#if GIRQ25_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq25), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq25)),
		    DT_IRQ(DT_NODELABEL(girq25), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq25)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq25)));
#endif
#if GIRQ26_AGGR
	mchp_xec_ecia_girq_aggr_en(
		GIRQ_ID_TO_BITPOS(DT_PROP(DT_NODELABEL(girq26), girq_id)), 1);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(girq26)),
		    DT_IRQ(DT_NODELABEL(girq26), priority),
		    xec_girq_isr,
		    DEVICE_DT_GET(DT_NODELABEL(girq26)), 0);

	irq_enable(DT_IRQN(DT_NODELABEL(girq26)));
#endif
}

/**
 * @brief initialize XEC ECIA driver
 */
static int xec_ecia_init(const struct device *dev)
{
	const struct xec_ecia_config *cfg =
		(const struct xec_ecia_config *const) (dev->config);
	const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(pcr));
	struct ecs_regs *const ecs = ECS_XEC_REG_BASE;
	struct ecia_regs *const ecia = (struct ecia_regs *)cfg->ecia_base;
	uint32_t n = 0, nr = 0;
	int ret;

	ret = clock_control_on(clk_dev,
			       (clock_control_subsys_t *)&cfg->clk_ctrl);
	if (ret < 0) {
		return ret;
	}

	/* Enable all direct NVIC connections */
	ecs->INTR_CTRL |= BIT(0);

	/* gate off all aggregated outputs */
	ecia->BLK_EN_CLR = UINT32_MAX;
	/* gate on GIRQ's that are aggregated only */
	ecia->BLK_EN_SET = MCHP_ECIA_AGGR_BITMAP;

	/* Clear all GIRQn source enables */
	for (n = 0; n < MCHP_NUM_GIRQS; n++) {
		ecia->GIRQ[n].EN_CLR = UINT32_MAX;
	}

	/* Clear all external NVIC enables and pending status */
	nr = SCnSCB->ICTR;
	for (n = 0u; n <= nr; n++) {
		NVIC->ICER[n] = UINT32_MAX;
		NVIC->ICPR[n] = UINT32_MAX;
	}

	xec_connect_girqs(dev);

	return 0;
}

#if GIRQ08_AGGR
static struct xec_girq_src_data xec_data_girq_8[GIRQ_NBITS_8];

static const struct xec_girq_config xec_config_girq_8 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq8)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq8), source_bitmap),
	.num_srcs = GIRQ_NBITS_8,
	.girq_id = DT_PROP(DT_NODELABEL(girq8), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq8), NULL,
		 NULL, &xec_data_girq_8, &xec_config_girq_8,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ09_AGGR
static struct xec_girq_src_data xec_data_girq_9[GIRQ_NBITS_9];

static const struct xec_girq_config xec_config_girq_9 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq9)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq9), source_bitmap),
	.num_srcs = GIRQ_NBITS_9,
	.girq_id = DT_PROP(DT_NODELABEL(girq9), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq9), NULL,
		 NULL, &xec_data_girq_9, &xec_config_girq_9,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ10_AGGR
static struct xec_girq_src_data xec_data_girq_10[GIRQ_NBITS_10];

static const struct xec_girq_config xec_config_girq_10 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq10)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq10), source_bitmap),
	.num_srcs = GIRQ_NBITS_10,
	.girq_id = DT_PROP(DT_NODELABEL(girq10), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq10), NULL,
		 NULL, &xec_data_girq_10, &xec_config_girq_10,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ11_AGGR
static struct xec_girq_src_data xec_data_girq_11[GIRQ_NBITS_11];

static const struct xec_girq_config xec_config_girq_11 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq11)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq11), source_bitmap),
	.num_srcs = GIRQ_NBITS_11,
	.girq_id = DT_PROP(DT_NODELABEL(girq11), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq11), NULL,
		 NULL, &xec_data_girq_11, &xec_config_girq_11,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ12_AGGR
static struct xec_girq_src_data xec_data_girq_12[GIRQ_NBITS_12];

static const struct xec_girq_config xec_config_girq_12 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq12)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq12), source_bitmap),
	.num_srcs = GIRQ_NBITS_12,
	.girq_id = DT_PROP(DT_NODELABEL(girq12), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq12), NULL,
		 NULL, &xec_data_girq_12, &xec_config_girq_12,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ13_AGGR
static struct xec_girq_src_data xec_data_girq_13[GIRQ_NBITS_13];

static const struct xec_girq_config xec_config_girq_13 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq13)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq13), source_bitmap),
	.num_srcs = GIRQ_NBITS_13,
	.girq_id = DT_PROP(DT_NODELABEL(girq13), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq13), NULL,
		 NULL, &xec_data_girq_13, &xec_config_girq_13,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ14_AGGR
static struct xec_girq_src_data xec_data_girq_14[GIRQ_NBITS_14];

static const struct xec_girq_config xec_config_girq_14 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq14)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq14), source_bitmap),
	.num_srcs = GIRQ_NBITS_14,
	.girq_id = DT_PROP(DT_NODELABEL(girq14), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq14), NULL,
		 NULL, &xec_data_girq_14, &xec_config_girq_14,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ15_AGGR
static struct xec_girq_src_data xec_data_girq_15[GIRQ_NBITS_15];

static const struct xec_girq_config xec_config_girq_15 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq15)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq15), source_bitmap),
	.num_srcs = GIRQ_NBITS_15,
	.girq_id = DT_PROP(DT_NODELABEL(girq15), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq15), NULL,
		 NULL, &xec_data_girq_15, &xec_config_girq_15,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ16_AGGR
static struct xec_girq_src_data xec_data_girq_16[GIRQ_NBITS_16];

static const struct xec_girq_config xec_config_girq_16 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq16)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq16), source_bitmap),
	.num_srcs = GIRQ_NBITS_16,
	.girq_id = DT_PROP(DT_NODELABEL(girq16), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq17), NULL,
		 NULL, &xec_data_girq_16, &xec_config_girq_16,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ17_AGGR
static struct xec_girq_src_data xec_data_girq_17[GIRQ_NBITS_17];

static const struct xec_girq_config xec_config_girq_17 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq17)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq17), source_bitmap),
	.num_srcs = GIRQ_NBITS_17,
	.girq_id = DT_PROP(DT_NODELABEL(girq17), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq17), NULL,
		 NULL, &xec_data_girq_17 &xec_config_girq_17,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ18_AGGR
static struct xec_girq_src_data xec_data_girq_18[GIRQ_NBITS_18];

static const struct xec_girq_config xec_config_girq_18 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq18)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq18), source_bitmap),
	.num_srcs = GIRQ_NBITS_18,
	.girq_id = DT_PROP(DT_NODELABEL(girq18), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq18), NULL,
		 NULL, &xec_data_girq_18, &xec_config_girq_18,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ19_AGGR
static struct xec_girq_src_data xec_data_girq_19[GIRQ_NBITS_19];

static const struct xec_girq_config xec_config_girq_19 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq19)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq19), source_bitmap),
	.num_srcs = GIRQ_NBITS_19,
	.girq_id = DT_PROP(DT_NODELABEL(girq19), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq19), NULL,
		 NULL, &xec_data_girq_19, &xec_config_girq_19,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ20_AGGR
static struct xec_girq_src_data xec_data_girq_20[GIRQ_NBITS_20];

static const struct xec_girq_config xec_config_girq_20 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq20)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq20), source_bitmap),
	.num_srcs = GIRQ_NBITS_20,
	.girq_id = DT_PROP(DT_NODELABEL(girq20), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq20), NULL,
		 NULL, &xec_data_girq_20, &xec_config_girq_20,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ21_AGGR
static struct xec_girq_src_data xec_data_girq_21[GIRQ_NBITS_21];

static const struct xec_girq_config xec_config_girq_21 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq21)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq21), source_bitmap),
	.num_srcs = GIRQ_NBITS_21,
	.girq_id = DT_PROP(DT_NODELABEL(girq21), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq21), NULL,
		 NULL, &xec_data_girq_21, &xec_config_girq_21,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ23_AGGR
static struct xec_girq_src_data xec_data_girq_23[GIRQ_NBITS_23];

static const struct xec_girq_config xec_config_girq_23 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq23)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq23), source_bitmap),
	.num_srcs = GIRQ_NBITS_23,
	.girq_id = DT_PROP(DT_NODELABEL(girq23), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq23), NULL,
		 NULL, &xec_data_girq_23, &xec_config_girq_23,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ24_AGGR
static struct xec_girq_src_data xec_data_girq_24[GIRQ_NBITS_24];

static const struct xec_girq_config xec_config_girq_24 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq24)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq24), source_bitmap),
	.num_srcs = GIRQ_NBITS_24,
	.girq_id = DT_PROP(DT_NODELABEL(girq24), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq24), NULL,
		 NULL, &xec_data_girq_24, &xec_config_girq_24,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ25_AGGR
static struct xec_girq_src_data xec_data_girq_25[GIRQ_NBITS_25];

static const struct xec_girq_config xec_config_girq_25 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq25)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq25), source_bitmap),
	.num_srcs = GIRQ_NBITS_25,
	.girq_id = DT_PROP(DT_NODELABEL(girq25), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq25), NULL,
		 NULL, &xec_data_girq_25, &xec_config_girq_25,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif
#if GIRQ26_AGGR
static struct xec_girq_src_data xec_data_girq_26[GIRQ_NBITS_26];

static const struct xec_girq_config xec_config_girq_26 = {
	.base = DT_REG_ADDR(DT_NODELABEL(girq26)),
	.src_bitmap = DT_PROP(DT_NODELABEL(girq26), source_bitmap),
	.num_srcs = GIRQ_NBITS_26,
	.girq_id = DT_PROP(DT_NODELABEL(girq26), girq_id),
};

DEVICE_DT_DEFINE(DT_NODELABEL(girq26), NULL,
		 NULL, &xec_data_girq_26, &xec_config_girq_26,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
#endif

static const struct xec_ecia_config xec_config_ecia = {
	.ecia_base = DT_REG_ADDR(DT_NODELABEL(ecia)),
	.direct_capable = DT_PROP(DT_NODELABEL(ecia), directmap),
	.clk_ctrl = {
		.pcr_info = ECIA_XEC_PCR_INFO,
	},
};

DEVICE_DT_DEFINE(DT_NODELABEL(ecia), xec_ecia_init,
		 NULL, NULL, &xec_config_ecia,
		 PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 NULL);
