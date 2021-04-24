/*
 * Copyright (c) 2019 Microchip Technology Inc.
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/sys_io.h>
#include <sys/__assert.h>
#include <power/power.h>
#include <soc.h>
#include "device_power.h"

/*
 * Light sleep: PLL remains on. Fastest wake latency.
 */
void soc_lite_sleep_enable(void)
{
	SCB->SCR &= ~(1ul << 2);
	PCR_REGS->SYS_SLP_CTRL = MCHP_PCR_SYS_SLP_LIGHT;
}

/*
 * Deep sleep: PLL is turned off. Wake is fast. PLL requires
 * a minimum of 3ms to lock. During this time the main clock
 * will be ramping up from ~16 to 24 MHz.
 */
void soc_deep_sleep_enable(void)
{
	SCB->SCR = (1ul << 2); /* Cortex-M4 SLEEPDEEP */
	PCR_REGS->SYS_SLP_CTRL = MCHP_PCR_SYS_SLP_HEAVY;
}

void soc_deep_sleep_disable(void)
{
	SCB->SCR &= ~(1ul << 2); /* disable Cortex-M4 SLEEPDEEP */
}


void soc_deep_sleep_wait_clk_idle(void)
{
#ifdef DEEP_SLEEP_CLK_REQ_DUMP
	/* Save status to debug LPM been blocked */
	VBATM_REGS->MEM.u32[0] = PCR_REGS->CLK_REQ0;
	VBATM_REGS->MEM.u32[1] = PCR_REGS->CLK_REQ1;
	VBATM_REGS->MEM.u32[2] = PCR_REGS->CLK_REQ2;
	VBATM_REGS->MEM.u32[3] = PCR_REGS->CLK_REQ3;
	VBATM_REGS->MEM.u32[4] = PCR_REGS->CLK_REQ4;
#endif
}

/*
 * Allow peripherals connected to external masters to wake the PLL but not
 * the EC. Once the peripheral has serviced the external master the PLL
 * will be turned back off. For example, if the eSPI master requests eSPI
 * configuration information or state of virtual wires the EC doesn't need
 * to be involved. The hardware can power on the PLL long enough to service
 * the request and then turn the PLL back off.  The SMBus and I2C peripherals
 * in slave mode can also make use of this feature.
 */
void soc_deep_sleep_non_wake_en(void)
{
#ifdef CONFIG_ESPI_XEC
	GIRQ22_REGS->SRC = 0xfffffffful;
	GIRQ22_REGS->EN_SET = (1ul << 9);
#endif
}

void soc_deep_sleep_non_wake_dis(void)
{
#ifdef CONFIG_ESPI_XEC
	GIRQ22_REGS->EN_CLR = 0xfffffffful;
	GIRQ22_REGS->SRC = 0xfffffffful;
#endif
}

/* When MEC15xx drivers are power-aware this should be move there */
void soc_deep_sleep_wake_en(void)
{
#ifdef CONFIG_KSCAN
	/* Enable PLL wake via KSCAN  */
	GIRQ21_REGS->SRC = (1ul << 19);
	GIRQ21_REGS->EN_SET = (1ul << 19);
#endif
#ifdef CONFIG_PS2_XEC_0
	/* Enable PS2_0B_WK */
	GIRQ21_REGS->SRC = (1ul << 19);
	GIRQ21_REGS->EN_SET = (1ul << 19);
#endif
#ifdef CONFIG_PS2_XEC_1
	/* Enable PS2_1B_WK */
	GIRQ21_REGS->SRC = (1ul << 21);
	GIRQ21_REGS->EN_SET = (1ul << 21);
#endif
}

void soc_deep_sleep_wake_dis(void)
{
#ifdef CONFIG_PS2_XEC_0
	/* Enable PS2_0B_WK */
	GIRQ21_REGS->EN_CLR = (1ul << 19);
	GIRQ21_REGS->SRC = (1ul << 19);
#endif
#ifdef CONFIG_PS2_XEC_1
	/* Enable PS2_1B_WK */
	GIRQ21_REGS->EN_CLR = (1ul << 21);
	GIRQ21_REGS->SRC = (1ul << 21);
#endif
}


/* Variables used to save various HW state */
#ifdef DEEP_SLEEP_PERIPH_SAVE_RESTORE

const struct ds_timer_info ds_timer_tbl[] = {
	{
		(uintptr_t)&B16TMR0_REGS->CTRL, 0
	},
	{
		(uintptr_t)&B16TMR1_REGS->CTRL, 0
	},
	{
		(uintptr_t)&B32TMR0_REGS->CTRL, 0
	},
	{
		(uintptr_t)&B32TMR1_REGS->CTRL, 0
	},
	{
		(uintptr_t)&CCT_REGS->CTRL,
		(MCHP_CCT_CTRL_COMP1_SET | MCHP_CCT_CTRL_COMP0_SET),
	},
};

static struct ds_dev_info ds_ctx;

static void deep_sleep_save_ecs(void)
{
	ds_ctx.ecs[0] = ECS_REGS->ETM_CTRL;
	ds_ctx.ecs[1] = ECS_REGS->DEBUG_CTRL;
#ifdef DEEP_SLEEP_JTAG
	ECS_REGS->ETM_CTRL = 0;
	ECS_REGS->DEBUG_CTRL = 0x00;
#endif
}

static void deep_sleep_save_uarts(void)
{
	ds_ctx.uart_info[0] = UART0_REGS->ACTV;
	if (ds_ctx.uart_info[0]) {
		while ((UART0_REGS->LSR & MCHP_UART_LSR_TEMT) == 0) {
		}
	}
	UART0_REGS->ACTV = 0;
	ds_ctx.uart_info[1] = UART1_REGS->ACTV;
	if (ds_ctx.uart_info[1]) {
		while ((UART1_REGS->LSR & MCHP_UART_LSR_TEMT) == 0) {
		}
	}
	UART1_REGS->ACTV = 0;
	ds_ctx.uart_info[2] = UART2_REGS->ACTV;
	if (ds_ctx.uart_info[2]) {
		while ((UART2_REGS->LSR & MCHP_UART_LSR_TEMT) == 0) {
		}
	}
	UART2_REGS->ACTV = 0;
	UART1_REGS->ACTV = 0;
	UART0_REGS->ACTV = 0;
}

static void deep_sleep_save_timers(void)
{
	const struct ds_timer_info *p;
	uint32_t i;

	p = &ds_timer_tbl[0];
	for (i = 0; i < NUM_DS_TIMER_ENTRIES; i++) {
		ds_ctx.timers[i] = REG32(p->addr);
		REG32(p->addr) = 0;
		p++;
	}
}

static void deep_sleep_restore_ecs(void)
{
#ifdef DEEP_SLEEP_JTAG
	ECS_REGS->ETM_CTRL = ds_ctx.ecs[0];
	ECS_REGS->DEBUG_CTRL = ds_ctx.ecs[1];
#endif
}

static void deep_sleep_restore_uarts(void)
{
	UART0_REGS->ACTV = ds_ctx.uart_info[0];
	UART1_REGS->ACTV = ds_ctx.uart_info[1];
	UART2_REGS->ACTV = ds_ctx.uart_info[2];
}

static void deep_sleep_restore_timers(void)
{
	const struct ds_timer_info *p;
	uint32_t i;

	p = &ds_timer_tbl[0];
	for (i = 0; i < NUM_DS_TIMER_ENTRIES; i++) {
		REG32(p->addr) = ds_ctx.timers[i] & ~p->restore_mask;
		p++;
	}
}

void soc_deep_sleep_periph_save(void)
{
	deep_sleep_save_ecs();
	deep_sleep_save_timers();
	deep_sleep_save_uarts();
}

void soc_deep_sleep_periph_restore(void)
{
	deep_sleep_restore_ecs();
	deep_sleep_restore_uarts();
	deep_sleep_restore_timers();
}

#else

void soc_deep_sleep_periph_save(void)
{
}

void soc_deep_sleep_periph_restore(void)
{
}

#endif /* DEEP_SLEEP_PERIPH_SAVE_RESTORE */
