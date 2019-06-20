/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys_io.h>
#include <misc/__assert.h>
#include <power.h>
#include <soc.h>

#define DEBUG_DEEP_SLEEP_GPIO_TOGGLE
/* #define DEBUG_DEEP_SLEEP_SPIN_ON_CLK_REQ */
#define DEBUG_DEEP_SLEEP_MIRROR_REG
/* #define DEBUG_DEEP_SLEEP_JTAG_DIS_BEFORE */
/* #define DEBUG_DEEP_SLEEP_JTAG_EN_ON_WAKE */
/* #define DEBUG_DEEP_SLEEP_KILL_ALL_INTR */
/* #define DEBUG_DEEP_SLEEP_RTMR_DUMP */

/* Until we get device power management in drivers enable this */
#define DEEP_SLEEP_PERIPH_SAVE_RESTORE

/* Variables used to save various HW state */
#ifdef DEEP_SLEEP_PERIPH_SAVE_RESTORE
u8_t uart_activate[3];
#endif

#ifdef DEBUG_DEEP_SLEEP_MIRROR_REG
u32_t sleep_status_mirror;
#endif

#ifdef DEBUG_DEEP_SLEEP_RTMR_DUMP
u32_t rtmr_cnt, rtmr_prld, rtmr_ctrl;
#endif


#if (defined(CONFIG_SYS_POWER_DEEP_SLEEP_STATES))

/*
 * Deep Sleep
 * Pros:
 * Lower power dissipation, 48MHz PLL is off
 * Cons:
 * Longer wake latency. CPU start running on ring oscillator
 * between 16 to 25 MHz. Minimum 3ms until PLL reaches lock
 * frequency of 48MHz.
 *
 * This is working only if
 * MEC1501 EC Subsystem Debug Enable is set to 0 (disable SWD/JTAG)
 * JTAG/SWD external debugger is stopped so that it doesn't try
 * sending clocks into MEC1501. Alternatively MEC1501 TRST# pin
 * can be pulled low disabling JTAG/SWD interface (production board).
 */
static void z_power_soc_deep_sleep(void)
{
#ifdef DEBUG_DEEP_SLEEP_SPIN_ON_CLK_REQ
	u32_t clkreq, cnt;
#endif

	/* Mask all exceptions and interrupts except NMI and HardFault */
	__set_PRIMASK(1);

#ifdef DEBUG_DEEP_SLEEP_GPIO_TOGGLE
	GPIO_CTRL_REGS->CTRL_0156 = 0x0240ul;
#endif

#ifdef DEEP_SLEEP_PERIPH_SAVE_RESTORE
	uart_activate[0] = UART0_REGS->ACTV;
	UART0_REGS->ACTV = 0;
	uart_activate[1] = UART0_REGS->ACTV;
	UART0_REGS->ACTV = 0;
	uart_activate[2] = UART0_REGS->ACTV;
	UART0_REGS->ACTV = 0;
#endif

#ifdef DEBUG_DEEP_SLEEP_MIRROR_REG
	sleep_status_mirror = 0;
#endif

#ifdef DEBUG_DEEP_SLEEP_JTAG_DIS_BEFORE
	ECS_REGS->DEBUG_CTRL = 0x00;
#endif

#ifdef DEBUG_DEEP_SLEEP_KILL_ALL_INTR
	/* TEST MODE: clear all interrupt enables and pending status */
	GIRQ_Type * p = &ECIA_REGS->GIRQ08;

	for (u32_t i = 8; i < 27; i++) {
		p->EN_CLR = 0xfffffffful;
		p++;
	}
	for (u32_t i = 0; i < 6; i++) {
		NVIC->ICER[clkreq] = 0xfffffffful;
		NVIC->ICPR[clkreq] = 0xfffffffful;
	}
#endif
	SCB->SCR = (1ul << 2); /* Cortex-M4 SLEEPDEEP */
	PCR_REGS->SYS_SLP_CTRL = MCHP_PCR_SYS_SLP_HEAVY;

#ifdef DEBUG_DEEP_SLEEP_SPIN_ON_CLK_REQ
	cnt = 256;
	do {
		clkreq = PCR_REGS->CLK_REQ0 | PCR_REGS->CLK_REQ1
			 | PCR_REGS->CLK_REQ2 | PCR_REGS->CLK_REQ3
			 | PCR_REGS->CLK_REQ4;
	} while ((clkreq != 0x100ul) && (cnt-- != 0));
#endif

#ifdef DEBUG_DEEP_SLEEP_MIRROR_REG
	sleep_status_mirror = *(volatile u32_t *)0x4000fd44ul;
#endif

	/*
	 * Unmask all interrupts in BASEPRI. PRIMASK is used above to
	 * prevent entering an ISR after unmasking in BASEPRI.
	 * We clear PRIMASK in exit post ops.
	 */
	__set_BASEPRI(0);
	__DSB();
	__WFI();
	__NOP();
	__NOP();

#ifdef DEBUG_DEEP_SLEEP_JTAG_EN_ON_WAKE
	ECS_REGS->DEBUG_CTRL = 0x05;
#endif

#ifdef DEBUG_DEEP_SLEEP_GPIO_TOGGLE
	GPIO_CTRL_REGS->CTRL_0156 = 0x10240ul;
#endif

#ifdef DEBUG_DEEP_SLEEP_RTMR_DUMP
	rtmr_cnt = RTMR_REGS->CNT;
	rtmr_prld = RTMR_REGS->PRLD;
	rtmr_ctrl = RTMR_REGS->CTRL;
#endif

#ifdef DEEP_SLEEP_PERIPH_SAVE_RESTORE
	UART0_REGS->ACTV = uart_activate[0];
	UART0_REGS->ACTV = uart_activate[1];
	UART0_REGS->ACTV = uart_activate[2];
#endif

}

#endif

/*
 * Sleep
 * Pros:
 * Fast wake response:
 * Cons:
 * Higher power dissipation, 48MHz PLL remains on.
 */
static void z_power_soc_sleep(void)
{
	__set_PRIMASK(1);
	GPIO_CTRL_REGS->CTRL_0014 = 0x0240ul;

	SCB->SCR &= ~(1ul << 2);
	PCR_REGS->SYS_SLP_CTRL = MCHP_PCR_SYS_SLP_LIGHT;

	GPIO_CTRL_REGS->CTRL_0014 = 0x0240ul;

	__set_BASEPRI(0); /* Make sure wake interrupts are not masked! */
	__DSB();
	__WFI();
	__NOP();
	__NOP();

	GPIO_CTRL_REGS->CTRL_0014 = 0x10240ul;
}

/*
 * Called from _sys_suspend(s32_t ticks) in subsys/power.c
 * For deep sleep _sys_suspend has executed all the driver
 * power management call backs.
 */
void sys_set_power_state(enum power_states state)
{
	switch (state) {
#if (defined(CONFIG_SYS_POWER_SLEEP_STATES))
	case SYS_POWER_STATE_SLEEP_1:
		z_power_soc_sleep();
		break;
#endif
#if (defined(CONFIG_SYS_POWER_DEEP_SLEEP_STATES))
	case SYS_POWER_STATE_DEEP_SLEEP_1:
#if 0
		/* Hack: UART 1 CLK_REQ is on. Disable UART 1 */
		UART1_REGS->ACTV = 0; /* UART 1 CLK_REQ is not clearing!!!! */
		PCR_REGS->RST_EN_LOCK = MCHP_PCR_RSTEN_UNLOCK;
		PCR_REGS->RST_EN2 = 0x02ul;
		PCR_REGS->RST_EN_LOCK = MCHP_PCR_RSTEN_LOCK;
		ECS_REGS->DEBUG_CTRL = 0x00;
#endif
		z_power_soc_deep_sleep();
#if 0
		/* Hack: UART 1 CLK_REQ is on. Disable UART 1 */
		ECS_REGS->DEBUG_CTRL = 0x05;
		UART1_REGS->ACTV = 1;
#endif
		break;
#endif
	default:
		break;
	}
}

void _sys_pm_power_state_exit_post_ops(enum power_states state)
{
	switch (state) {
#if (defined(CONFIG_SYS_POWER_SLEEP_STATES))
	case SYS_POWER_STATE_SLEEP_1:
		__enable_irq();
		break;
#endif
#if (defined(CONFIG_SYS_POWER_DEEP_SLEEP_STATES))
	case SYS_POWER_STATE_DEEP_SLEEP_1:
		__enable_irq();
		break;
#endif
	default:
		break;
	}
}

