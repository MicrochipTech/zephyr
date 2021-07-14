/*
 * Copyright (c) 2019 Intel Corporation
 * Copyright (c) 2021 Microchip Technology Inc.
 *
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
#define ECS_XEC_REG_BASE						\
	((struct ecs_regs *)(DT_REG_ADDR(DT_NODELABEL(ecs))))
#define PCR_XEC_REG_BASE						\
	((struct pcr_regs *)(DT_REG_ADDR(DT_NODELABEL(pcr))))
#define VBATR_XEC_REG_BASE						\
	((struct vbatr_regs *)(DT_REG_ADDR(DT_NODELABEL(vbr))))

#define CLK32K_SIL_OSC_DELAY		256U
#define CLK32K_PLL_LOCK_WAIT		(16U * 1024U)

/*
 * In early Zephyr initialization we don't have timer services. Also, the SoC
 * may be running on its ring oscillator (+/- 50% accuracy). Configuring the
 * SoC's clock subsystem requires wait/delays. We implement a simple delay
 * by writing to a read-only hardware register in the PCR block.
 */
static uint32_t spin_delay(uint32_t cnt)
{
	struct pcr_regs *pcr = PCR_XEC_REG_BASE;
	uint32_t n;

	for (n = 0U; n < cnt; n++) {
		pcr->OSC_ID = n;
	}

	return n;
}

/*
 * Make sure PCR sleep enables are clear except for crypto
 * which do not have internal clock gating.
 */
static int soc_pcr_init(void)
{
	struct pcr_regs *pcr = PCR_XEC_REG_BASE;

	pcr->SYS_SLP_CTRL = 0U;

	for (int i = 0; i < MCHP_MAX_PCR_SCR_REGS; i++) {
		pcr->SLP_EN[i] = 0U;
	}

	pcr->SLP_EN[3] = MCHP_PCR3_CRYPTO_MASK;

	return 0;
}

/*
 * Initialize MEC172x EC Interrupt Aggregator (ECIA) and external NVIC
 * inputs.
 */
static int soc_ecia_init(void)
{
	struct ecia_ar_regs *ecia = ECIA_XEC_REG_BASE;
	struct ecs_regs *ecs = ECS_XEC_REG_BASE;
	uint32_t n;

	mchp_pcr_periph_slp_ctrl(PCR_ECIA, MCHP_PCR_SLEEP_DIS);

	ecs->INTR_CTRL |= MCHP_ECS_ICTRL_DIRECT_EN;

	/* gate off all aggregated outputs */
	ecia->BLK_EN_CLR = UINT32_MAX;
	/* gate on GIRQ's that are aggregated only */
	ecia->BLK_EN_SET = MCHP_ECIA_AGGR_BITMAP;

	/* Clear all GIRQn source enables */
	for (n = 0; n < MCHP_GIRQ_IDX_MAX; n++) {
		ecia->GIRQ[n].EN_CLR = UINT32_MAX;
	}

	/* Clear all external NVIC enables and pending status */
	for (n = 0u; n < MCHP_NUM_NVIC_REGS; n++) {
		NVIC->ICER[n] = UINT32_MAX;
		NVIC->ICPR[n] = UINT32_MAX;
	}

	return 0;
}

static bool is_sil_osc_enabled(void)
{
	struct vbatr_regs *vbr = VBATR_XEC_REG_BASE;

	if (vbr->CLK32_SRC & MCHP_VBATR_CS_SO_EN) {
		return true;
	}

	return false;
}

static void enable_sil_osc(uint32_t delay_cnt)
{
	struct vbatr_regs *vbr = VBATR_XEC_REG_BASE;

	vbr->CLK32_SRC |= MCHP_VBATR_CS_SO_EN;
	spin_delay(delay_cnt);
}

/*
 * This routine checks if the PLL is locked to its input source. Minimum lock
 * time is 3.3 ms. Lock time can be larger when the source is an external
 * crystal. Crystal cold start times may vary greatly based on many factors.
 * Crystals do not like being power cycled.
 */
static int pll_wait_lock(uint32_t wait_cnt)
{
	struct pcr_regs *regs = PCR_XEC_REG_BASE;

	while (!(regs->OSC_ID & MCHP_PCR_OSC_ID_PLL_LOCK)) {
		if (wait_cnt == 0) {
			return -ETIMEDOUT;
		}
		--wait_cnt;
	}

	return 0;
}

/* Initialize MEC172x PLL and Peripheral clock source to silicon OSC. */
static int soc_clk32_init(void)
{
	struct pcr_regs *pcr = PCR_XEC_REG_BASE;
	struct vbatr_regs *vbr = VBATR_XEC_REG_BASE;

	if (!is_sil_osc_enabled()) {
		enable_sil_osc(CLK32K_SIL_OSC_DELAY);
	}

	pcr->CLK32K_SRC_VTR = MCHP_PCR_VTR_32K_SRC_SILOSC;
	vbr->CLK32_SRC = (vbr->CLK32_SRC & ~(MCHP_VBATR_CS_PCS_MSK)) |
			 MCHP_VBATR_CS_PCS_VTR_VBAT_SO;

	int rc = pll_wait_lock(CLK32K_PLL_LOCK_WAIT);

	if (rc) {
		return rc;
	}

	return 0;
}

/*
 * MEC172x Errata document DS80000913C
 * Programming the PCR clock divider that divides the clock input to the ARM
 * Cortex-M4 may cause a clock glitch. The recommended work-around is to
 * issue four NOP instruction before and after the write to the PCR processor
 * clock control register. The final four NOP instructions are followed by
 * data and instruction barriers to flush the Cortex-M4's pipeline.
 * NOTE: Zephyr provides inline functions for Cortex-Mx NOP but not for
 * data and instruction barrier instructions. Caller's should only invoke this
 * function with interrupts locked.
 */
void soc_set_core_clock_div(uint8_t clkdiv)
{
	struct pcr_regs *regs = PCR_XEC_REG_BASE;

	arch_nop();
	arch_nop();
	arch_nop();
	arch_nop();
	regs->PROC_CLK_CTRL = (uint32_t)clkdiv;
	arch_nop();
	arch_nop();
	arch_nop();
	arch_nop();
	__DSB();
	__ISB();
}

static int soc_init(const struct device *dev)
{
	uint32_t key;
	int rc;

	ARG_UNUSED(dev);

	key = irq_lock();

	soc_pcr_init();
	soc_ecia_init();

	rc = soc_clk32_init();
	__ASSERT(rc == 0, "SoC: PLL and 32 KHz clock initialization failed");

	soc_set_core_clock_div(CONFIG_SOC_MEC172X_PROC_CLK_DIV);

	irq_unlock(key);

	return rc;
}

SYS_INIT(soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
