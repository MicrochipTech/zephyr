/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <zephyr/sys/sys_io.h>

#include "soc_pm_periph.h"

#define XEC_PM_DEBUG_GPIO_MARKER
#define XEC_PM_DEBUG_CLK_REQ_VBAT

#define XEC_PM_DEBUG_CLK_REQ_VBAT_LOC 0

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

#define XEC_I2C_INSTANCES 5
#define XEC_I2C0_REG_BASE DT_REG_ADDR(DT_NODELABEL(i2c_smb_0))
#define XEC_I2C_INST_OFS  0x400U

#define XEC_I2C_CONFIG_OFS   0x28U
#define XEC_I2C_WAKE_STS_OFS 0x60U
#define XEC_I2C_WAKE_EN_OFS  0x64U

#if DT_NODE_EXISTS(DT_NODELABEL(uart3))
#define XEC_UART_INSTANCES 4
#elif DT_NODE_EXISTS(DT_NODELABEL(uart2))
#define XEC_UART_INSTANCES 3
#else
#define XEC_UART_INSTANCES 2
#endif

#define XEC_UART0_REG_BASE DT_REG_ADDR(DT_NODELABEL(uart0))

/* PM debug */
#ifdef XEC_PM_DEBUG_GPIO_MARKER
#define MEC_GPIO_BASE DT_REG_ADDR(DT_NODELABEL(pinctrl))

#define XEC_PM_ENTRY_EXIT_GPIO 0241

#define XEC_PM_EE_GPIO_CR_ADDR MEC_GPIO_CR1_ADDR(MEC_GPIO_BASE, XEC_PM_ENTRY_EXIT_GPIO)

#define XEC_PM_EE_GPIO_DRV_LO 0x240U
#define XEC_PM_EE_GPIO_DRV_HI 0x10240U

#define XEC_PM_DBG_DS_ENTER() sys_write32(XEC_PM_EE_GPIO_DRV_LO, XEC_PM_EE_GPIO_CR_ADDR)
#define XEC_PM_DBG_DS_EXIT() sys_write32(XEC_PM_EE_GPIO_DRV_HI, XEC_PM_EE_GPIO_CR_ADDR)

#else
#define XEC_PM_DBG_DS_ENTER()
#define XEC_PM_DBG_DS_EXIT()
#endif

#ifdef XEC_PM_DEBUG_CLK_REQ_VBAT
static void soc_pm_dbg_clk_req_save_to_vbat(void)
{
	uintptr_t pcr_base = (uintptr_t)XEC_PCR_REG_BASE + 0x50U;
	uintptr_t vbmem_base = ((uintptr_t)DT_REG_ADDR(DT_NODELABEL(bbram)) +
				XEC_PM_DEBUG_CLK_REQ_VBAT_LOC);

	for (uint32_t i = 0; i < 20U; i += 4U) {
		uint32_t clk_req = sys_read32(pcr_base + i);

		sys_write32(clk_req, vbmem_base + i);
	}
}
#define XEC_PM_DEBUG_CLK_REQ_VBAT_SAVE() soc_pm_dbg_clk_req_save_to_vbat()
#else
#define XEC_PM_DEBUG_CLK_REQ_VBAT_SAVE()
#endif


static uint8_t basic_timer_cr_save[XEC_BASIC_TIMER_INSTANCES];
static uint8_t uart_actv_save[XEC_UART_INSTANCES];

static void save_basic_timers(void)
{
	uintptr_t rb = (uintptr_t)(XEC_BASIC_TIMER0_REG_BASE);

	for (uint32_t n = 0; n < XEC_BASIC_TIMER_INSTANCES; n++) {
		uint32_t temp = sys_read32(rb + XEC_BASIC_TIMER_CR_OFS);

		sys_write32(temp & ~BIT(XEC_BASIC_TIMER_CR_EN_POS), rb + XEC_BASIC_TIMER_CR_OFS);
		basic_timer_cr_save[n] = (uint8_t)(temp & BIT(XEC_BASIC_TIMER_CR_EN_POS));
		rb += XEC_BASIC_TIMER_SPACING;
	}
}

static void restore_basic_timers(void)
{
	uintptr_t rb = (uintptr_t)(XEC_BASIC_TIMER0_REG_BASE);

	for (uint32_t n = 0; n < XEC_BASIC_TIMER_INSTANCES; n++) {
		if (basic_timer_cr_save[n] != 0) {
			sys_set_bit(rb + XEC_BASIC_TIMER_CR_OFS, XEC_BASIC_TIMER_CR_EN_POS);
		}
		rb += XEC_BASIC_TIMER_SPACING;
	}
}

static void save_uarts(void)
{
	uintptr_t rb = (uintptr_t)(XEC_UART0_REG_BASE);

	for (uint32_t n = 0; n < XEC_UART_INSTANCES; n++) {
		uart_actv_save[n] = sys_read8(rb + XEC_UART_LD_ACT_OFS);
		sys_write8(0, rb + XEC_UART_LD_ACT_OFS);
		rb += XEC_UART_SPACING;
	}
}

static void restore_uarts(void)
{
	uintptr_t rb = (uintptr_t)(XEC_UART0_REG_BASE);

	for (uint32_t n = 0; n < XEC_UART_INSTANCES; n++) {
		sys_write8(uart_actv_save[n], rb + XEC_UART_LD_ACT_OFS);
		rb += XEC_UART_SPACING;
	}
}

/* Enable peripherals capable of waking from deep sleep when a packet is received from
 * and external source: I2C, I3C, eSPI, and SPI target.
 * GIRQ22 is used to turn the PLL on long enough to decide if the packet is meant for this
 * SoC. If it is, the peripheral will asserts its specific GIRQx interrupt which wakes the
 * SoC CPU. If the packet is not for this device, HW will turn the PLL back off not waking
 * the SoC CPU.
 */
static void prepare_wake_devices(void)
{
	soc_ecia_girq_status_clear_bm(MCHP_MEC_ECIA_GIRQ22, UINT32_MAX);
	soc_ecia_girq_ctrl_bm(MCHP_MEC_ECIA_GIRQ22, UINT32_MAX, MCHP_MEC_ECIA_GIRQ_EN);
}

/* Peripheral's that don't obey PCR sleep signals
 * Basic timers, and more ...
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
 * Enable deep sleep mode in CM4 and XEC PCR.
 * Enable CM4 deep sleep and sleep signals assertion on WFI.
 * Set MCHP Heavy sleep (PLL OFF when all CLK_REQ clear) and SLEEP_ALL
 * to assert SLP_EN to all peripherals on WFI.
 * Set PRIMASK = 1 so on wake the CPU will not vector to any ISR.
 * Set BASEPRI = 0 to allow any priority to wake.
 * WFI triggers MEC HW to enter deep sleep
 * On wake
 * Clear SLEEP_ALL manually since we are not vectoring to an ISR until
 * PM post ops. This de-asserts peripheral SLP_EN signals.
 * 
 * NOTE: Cortex-M4 NVIC pending interrupts will not wake the core if they are masked.
 * Zephyr PM subsystem masks interrupts using BASEPRI.
 * Default: CM4 priority 0(highest) is unmasked all lower priorities are masked using BASEPRI.
 * If CONFIG_ZERO_LATENCY_IRQS is enabled PM subsystem will leave priorities 0(highest) through
 * 1 + CONFIG_ZERO_LATENCY_IRQS unmasked.
 * The issue is Cortex-M4 NVIC will not wake the core if a pending interrupt's priority is masked
 * in BASEPRI.
 * Work-around recommended by ARM and use by other Zephyr Cortex-M3/M4 SoC.
 * Mask all maskable interrupts using PRIMASK bit.
 * Set BASEPRI to 0 to unmask all priority levels.
 * Any pending interrupt in the NVIC will wake from WFI. If the interrupt is pending when WFI
 * executes, WFI is treated as a NOP.
 * After WFI we don't enter any ISR because PRIMASK is 1. The XEC SoC is on ring oscillator until
 * its PLL locks. The ring oscillator clock is not stable for some peripherals (UART, etc.). We
 * spin until lock (maximum 3 ms for MEC172x,4x,5x series).
 * We restore HW and allow Zephyr PM subsystem to restore any HW (PM_DEVICE).
 * On PM exit post ops we clear PRIMASK to allow servicing of pendind interrupts. 
 */
static void z_power_soc_deep_sleep(void)
{
	uint32_t pcrbase = (uint32_t)(XEC_PCR_REG_BASE);
	uint32_t msk = BIT(XEC_PCR_SLP_CR_DEEP_SLP_POS) | BIT(XEC_PCR_SLP_CR_ALL_POS);
	uint32_t val = BIT(XEC_PCR_SLP_CR_DEEP_SLP_POS) | BIT(XEC_PCR_SLP_CR_ALL_POS);

	/* Cortex-Mx wake sequence recommended by ARM */
	__disable_irq(); /* Prevent immediate ISR entry via PRIMASK */
	irq_unlock(0); /* Set BASEPRI to 0 */
	__DSB();
	__ISB();

	XEC_PM_DBG_DS_ENTER();

	soc_deep_sleep_periph_save();
	prepare_wake_devices();

	/* Enable Cortex-M4 to assert SLEEP_DEEP signal on WFI */
	SCB->SCR |= BIT(SCB_SCR_SLEEPDEEP_Pos);

	/* Enable MEC PCR to assert all peripheral SLP_EN signals on WFI */
	soc_mmcr_mask_set(pcrbase + XEC_PCR_SLP_CR_OFS, val, msk);

	XEC_PM_DEBUG_CLK_REQ_VBAT_SAVE(); /* record PCR peripheral CLK_REQ signals */

	__DSB();
	__ISB();
	__WFI(); /* triggers sleep hardware */
	__NOP();
	__NOP();

	/* wake */
	soc_mmcr_mask_set(pcrbase + XEC_PCR_SLP_CR_OFS, 0, msk);

	SCB->SCR &= ~BIT(SCB_SCR_SLEEPDEEP_Pos);

	XEC_PM_DBG_DS_EXIT();

	/* Wait for PLL to lock. Maximum is 3ms for current MEC parts */
	while (sys_test_bit(pcrbase + XEC_PCR_OSC_ID_OFS, XEC_PCR_OSC_ID_PLL_LOCK_POS) == 0) {
		__NOP();
	}

	soc_deep_sleep_periph_restore();
}
#endif

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

	SCB->SCR &= ~BIT(SCB_SCR_SLEEPDEEP_Pos);

	soc_mmcr_mask_set(pcrbase + XEC_PCR_SLP_CR_OFS, val, msk);

	__set_BASEPRI(0);
	__DSB();
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
		z_power_soc_sleep();
		break;
	case PM_STATE_SUSPEND_TO_RAM:
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
	__DSB();
	__ISB();
	__enable_irq(); /* Clear PRIMASK */
}
