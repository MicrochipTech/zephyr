/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <zephyr/sys/sys_io.h>

#define XEC_PCR_REG_BASE DT_REG_ADDR(DT_NODELABEL(pcr))

#define XEC_PCR_SLP_CR_OFS          0
#define XEC_PCR_SLP_CR_ALL_POS      3
#define XEC_PCR_SLP_CR_DEEP_SLP_POS 0

#define XEC_PCR_OSC_ID_OFS          0x0CU
#define XEC_PCR_OSC_ID_PLL_LOCK_POS 8

#define XEC_BASIC_TIMER_INSTANCES 6
#define XEC_BASIC_TIMER_SPACING   0x20U
#define XEC_BASIC_TIMER0_REG_BASE DT_REG_ADDR(DT_NODELABEL(timer0))
#define XEC_BASIC_TIMER_CR_OFS    0x10U
#define XEC_BASIC_TIMER_CR_EN_POS 0

#if DT_NODE_EXISTS(DT_NODELABEL(uart3))
#define MEC165XB_UART_INSTANCES 4
#elif DT_NODE_EXISTS(DT_NODELABEL(uart2))
#define MEC165XB_UART_INSTANCES 3
#else
#define MEC165XB_UART_INSTANCES 2
#endif

#define XEC_UART0_REG_BASE DT_REG_ADDR(DT_NODELABEL(uart0))

#define XEC_PINCTRL_BASE DT_REG_ADDR(DT_NODELABEL(pinctrl))

/* #define XEC_SOC_PM_DS_MARKER */

#define XEC_SOC_PM_GPIO_DRV_LO 0x0240U
#define XEC_SOC_PM_GPIO_DRV_HI 0x10240U

#ifdef XEC_SOC_PM_DS_MARKER
#define XEC_SOC_PM_DS_MARKER_GPIO 0241
#define XEC_SOC_PM_DS_GPIO_CR1_BA MEC_GPIO_CR1_ADDR(XEC_PINCTRL_BASE, XEC_SOC_PM_DS_MARKER_GPIO)

#define XEC_SOC_PM_DS_MARKER_ENTRY() sys_write32(XEC_SOC_PM_GPIO_DRV_LO, XEC_SOC_PM_DS_GPIO_CR1_BA);
#define XEC_SOC_PM_DS_MARKER_EXIT()  sys_write32(XEC_SOC_PM_GPIO_DRV_HI, XEC_SOC_PM_DS_GPIO_CR1_BA);
#else
#define XEC_SOC_PM_DS_MARKER_ENTRY()
#define XEC_SOC_PM_DS_MARKER_EXIT()
#endif

#ifdef XEC_SOC_PM_LS_MARKER
#define XEC_SOC_PM_LS_MARKER_GPIO 0244
#define XEC_SOC_PM_LS_GPIO_CR1_BA MEC_GPIO_CR1_ADDR(XEC_PINCTRL_BASE, XEC_SOC_PM_LS_MARKER_GPIO)

#define XEC_SOC_PM_LS_MARKER_ENTRY() sys_write32(XEC_SOC_PM_GPIO_DRV_LO, XEC_SOC_PM_LS_GPIO_CR1_BA);
#define XEC_SOC_PM_LS_MARKER_EXIT()  sys_write32(XEC_SOC_PM_GPIO_DRV_HI, XEC_SOC_PM_LS_GPIO_CR1_BA);
#else
#define XEC_SOC_PM_LS_MARKER_ENTRY()
#define XEC_SOC_PM_LS_MARKER_EXIT()
#endif

/* #define XEC_SOC_PM_CLK_REQ_DEBUG */

#ifdef XEC_SOC_PM_CLK_REQ_DEBUG
#define XEC_SOC_PM_CLK_REQ_VBM_OFS 0

static void xec_soc_pm_clk_req_save_to_vbat(void)
{
	uintptr_t vbat_mem_addr = DT_REG_ADDR(DT_NODELABEL(bbram)) + XEC_SOC_PM_CLK_REQ_VBM_OFS;
	uintptr_t clk_req_addr = XEC_PCR_REG_BASE + XEC_CC_CLK_REQ_OFS(0);

	for (uint32_t n = 0; n < XEC_CC_PCR_MAX_SCR; n++) {
		uint32_t clk_req = sys_read32(clk_req_addr);

		sys_write32(clk_req, vbat_mem_addr);
		clk_req_addr += 4U;
		vbat_mem_addr += 4U;
	}
}
#define XEC_SOC_PM_CLK_REQ_DBG() xec_soc_pm_clk_req_save_to_vbat()
#else
#define XEC_SOC_PM_CLK_REQ_DBG()
#endif

static uint8_t basic_timer_cr_save[XEC_BASIC_TIMER_INSTANCES];
static uint8_t uart_actv_save[MEC165XB_UART_INSTANCES];

static void save_basic_timers(void)
{
	mm_reg_t rb = (mm_reg_t)(XEC_BASIC_TIMER0_REG_BASE);

	for (uint32_t n = 0; n < XEC_BASIC_TIMER_INSTANCES; n++) {
		uint32_t temp = sys_read32(rb + XEC_BASIC_TIMER_CR_OFS);

		sys_write32(temp & ~BIT(XEC_BASIC_TIMER_CR_EN_POS), rb + XEC_BASIC_TIMER_CR_OFS);
		basic_timer_cr_save[n] = (uint8_t)(temp & BIT(XEC_BASIC_TIMER_CR_EN_POS));
		rb += XEC_BASIC_TIMER_SPACING;
	}
}

static void restore_basic_timers(void)
{
	mm_reg_t rb = (mm_reg_t)(XEC_BASIC_TIMER0_REG_BASE);

	for (uint32_t n = 0; n < XEC_BASIC_TIMER_INSTANCES; n++) {
		if (basic_timer_cr_save[n] != 0) {
			sys_set_bit(rb + XEC_BASIC_TIMER_CR_OFS, XEC_BASIC_TIMER_CR_EN_POS);
		}
		rb += XEC_BASIC_TIMER_SPACING;
	}
}

static void save_uarts(void)
{
	mm_reg_t rb = (mm_reg_t)(XEC_UART0_REG_BASE);

	for (uint32_t n = 0; n < MEC165XB_UART_INSTANCES; n++) {
		uart_actv_save[n] = sys_read8(rb + XEC_UART_LD_ACT_OFS);
		sys_write8(0, rb + XEC_UART_LD_ACT_OFS);
		rb += XEC_UART_SPACING;
	}
}

static void restore_uarts(void)
{
	mm_reg_t rb = (mm_reg_t)(XEC_UART0_REG_BASE);

	for (uint32_t n = 0; n < MEC165XB_UART_INSTANCES; n++) {
		sys_write8(uart_actv_save[n], rb + XEC_UART_LD_ACT_OFS);
		rb += XEC_UART_SPACING;
	}
}

/* Peripheral's that don't obey PCR sleep signals and can't be disabled until just
 * before WFI: basic timers and UARTs.
 * NOTE: UARTs should be handled at the app/driver level.
 * PM=y PM_DEVICE=n app/driver should use PM notifier for UARTs
 * PM_DEVICE=y UART driver set/clear busy bit and implements PM action callback.
 */
static void soc_deep_sleep_periph_save(void)
{
	save_basic_timers();
	save_uarts();
}

static void soc_deep_sleep_periph_restore(void)
{
	restore_basic_timers();
	restore_uarts();
}

/*
 * Enable deep sleep mode in CM4 and MEC172x.
 * Enable CM4 deep sleep and sleep signals assertion on WFI.
 * Set MCHP Heavy sleep (PLL OFF when all CLK_REQ clear) and SLEEP_ALL
 * to assert SLP_EN to all peripherals on WFI.
 * Set PRIMASK = 1 so on wake the CPU will not vector to any ISR.
 * Set BASEPRI = 0 to allow any priority to wake.
 * WFI triggers MEC HW to enter deep sleep
 * On wake
 * Clear SLEEP_ALL manually since we are not vectoring to an ISR until
 * PM post ops. This de-asserts peripheral SLP_EN signals.
 */
static void z_power_soc_deep_sleep(void)
{
	mm_reg_t pcrbase = (mm_reg_t)(XEC_PCR_REG_BASE);
	uint32_t msk = BIT(XEC_PCR_SLP_CR_DEEP_SLP_POS) | BIT(XEC_PCR_SLP_CR_ALL_POS);
	uint32_t val = BIT(XEC_PCR_SLP_CR_DEEP_SLP_POS) | BIT(XEC_PCR_SLP_CR_ALL_POS);

	__disable_irq(); /* Set PRIMASK=1 */
	irq_unlock(0); /* Set BASEPRI=0 */
	__DSB();
	__ISB();

	soc_deep_sleep_periph_save();

	SCB->SCR |= BIT(SCB_SCR_SLEEPDEEP_Pos);

	soc_mmcr_mask_set(pcrbase + XEC_PCR_SLP_CR_OFS, val, msk);

	XEC_SOC_PM_CLK_REQ_DBG();

	__DSB();
	__ISB();
	__WFI();
	__NOP();
	__NOP();

	soc_mmcr_mask_set(pcrbase + XEC_PCR_SLP_CR_OFS, 0, msk);

	SCB->SCR &= ~BIT(SCB_SCR_SLEEPDEEP_Pos);

	/* Wait for PLL to lock */
	while (sys_test_bit(pcrbase + XEC_PCR_OSC_ID_OFS, XEC_PCR_OSC_ID_PLL_LOCK_POS) == 0) {
		__NOP();
	}

	soc_deep_sleep_periph_restore();
}

/* NOTE: Zephyr kernel does not block all interrupts.
 * We use compiler instrisic to disable all interrupts except unmaskable
 * and highest priority hard faults.
 * Clear Cortex-M4 deep sleep enable.
 * Set MEC PCR SLEEP_ALL bit to 1
 * Clear Cortex-M4 NIVC base priority to block immediate ISR entry on wake.
 * Issue wait for interrupt
 * On wake clear PCR SLEEP_ALL bit.
 */
static void z_power_soc_sleep(void)
{
	mm_reg_t pcrbase = (mm_reg_t)(XEC_PCR_REG_BASE);
	uint32_t msk = BIT(XEC_PCR_SLP_CR_DEEP_SLP_POS) | BIT(XEC_PCR_SLP_CR_ALL_POS);
	uint32_t val = BIT(XEC_PCR_SLP_CR_ALL_POS);

	__disable_irq();
	irq_unlock(0);
	__DSB();
	__ISB();

	SCB->SCR &= ~BIT(SCB_SCR_SLEEPDEEP_Pos);

	soc_mmcr_mask_set(pcrbase + XEC_PCR_SLP_CR_OFS, val, msk);

	__DSB();
	__ISB();
	__WFI(); /* triggers sleep hardware */
	__NOP();
	__NOP();

	soc_mmcr_mask_set(pcrbase + XEC_PCR_SLP_CR_OFS, 0, msk);
}

/*
 * CONFIG_PM=y the kernel idle thread will
 *   arch_irq_lock()
 *   tick = get next timeout expiration
 *   pm_system_suspend(ticks) in pm subsystem
 *
 * pm_system_suspend(int32_t ticks) in subsys/power.c
 * if exit latency is acceptible.
 *   lock scheduler
 *   set pm post ops flag
 *   call pm_state_set()
 *
 * For deep sleep pm_system_suspend has executed all the driver power management call backs.
 */
void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		XEC_SOC_PM_LS_MARKER_ENTRY();
		z_power_soc_sleep();
		break;
	case PM_STATE_SUSPEND_TO_RAM:
		XEC_SOC_PM_DS_MARKER_ENTRY();
		z_power_soc_deep_sleep();
		break;
	default:
		break;
	}
}

/*
 * Zephyr PM code expects us to enabled interrupt at post op exit. Zephyr used
 * arch_irq_lock() which sets BASEPRI to a non-zero value masking interrupts at
 * >= numerical priority. MCHP z_power_soc_(deep)_sleep sets PRIMASK=1 and BASEPRI=0
 * allowing wake from any enabled interrupt and prevents the CPU from entering any
 * ISR on wake except for faults. We re-enable interrupts by undoing global disable
 * and alling irq_unlock with the same value, 0 zephyr core uses.
 */
void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		XEC_SOC_PM_LS_MARKER_EXIT();
		break;
	case PM_STATE_SUSPEND_TO_RAM:
		XEC_SOC_PM_DS_MARKER_EXIT();
		break;
	default:
		break;
	}

	__DSB();
	__ISB();
	__enable_irq(); /* Clear PRIMASK */
}
