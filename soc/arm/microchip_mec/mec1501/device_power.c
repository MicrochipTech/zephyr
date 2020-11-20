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
 * CPU will spin up to DEEP_SLEEP_WAIT_SPIN_CLK_REQ times
 * waiting for PCR CLK_REQ bits to clear except for the
 * CPU bit itself. This is not necessary as the sleep hardware
 * will wait for all CLK_REQ to clear once WFI has executed.
 * Once all CLK_REQ signals are clear the hardware will transition
 * to the low power state.
 */
/* #define DEEP_SLEEP_WAIT_ON_CLK_REQ_ENABLE */
#define DEEP_SLEEP_WAIT_SPIN_CLK_REQ		1000


/*
 * Some peripherals if enabled always assert their CLK_REQ bits.
 * For example, any peripheral with a clock generator such as
 * timers, counters, UART, etc. We save the enables for these
 * peripherals, disable them, and restore the enabled state upon
 * wake.
 */
#define DEEP_SLEEP_PERIPH_SAVE_RESTORE

/* #define CONFIG_BLOCKS_POWER_OPTIMIZATION */

#ifdef CONFIG_BLOCKS_POWER_OPTIMIZATION
#define CONFIG_DEEP_SLEEP_ADC
#define CONFIG_DEEP_SLEEP_PECI
#define CONFIG_DEEP_SLEEP_COMP
#define CONFIG_DEEP_SLEEP_VCI
#define CONFIG_DEEP_SLEEP_WEEK
#define CONFIG_DEEP_SLEEP_SLW_CLK
#define CONFIG_DEEP_SLEEP_ESPI
#define CONFIG_DEEP_SLEEP_P80
#define CONFIG_DEEP_SLEEP_I2C
#define CONFIG_DEEP_SLEEP_DEACTIVATE_UART
#endif
/* #define CONFIG_NON_WAKE_SUPPORT */

/* #define CONFIG_GPIO_POWER_OPTIMIZATION */
/* #define CONFIG_SPI_PINS_OFF */



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

#if defined(CONFIG_SYS_POWER_DEEP_SLEEP_STATES)

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
#ifdef DEEP_SLEEP_WAIT_ON_CLK_REQ_ENABLE
	uint32_t clkreq, cnt;

	cnt = DEEP_SLEEP_WAIT_CLK_REQ;
	do {
		clkreq = PCR_REGS->CLK_REQ0 | PCR_REGS->CLK_REQ1
			 | PCR_REGS->CLK_REQ2 | PCR_REGS->CLK_REQ3
			 | PCR_REGS->CLK_REQ4;
	} while ((clkreq != (1ul << MCHP_PCR1_CPU_POS)) && (cnt-- != 0));
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
#if defined(CONFIG_NON_WAKE_SUPPORT) && defined(CONFIG_ESPI_XEC)
	GIRQ22_REGS->SRC = 0xfffffffful;
	GIRQ22_REGS->EN_SET = (1ul << 9);
x
#endif
}

void soc_deep_sleep_non_wake_dis(void)
{
#if defined(CONFIG_NON_WAKE_SUPPORT) && defined(CONFIG_ESPI_XEC)
	GIRQ22_REGS->EN_CLR = 0xfffffffful;
	GIRQ22_REGS->SRC = 0xfffffffful;
#endif
}

/* Variables used to save various HW state */
#ifdef DEEP_SLEEP_PERIPH_SAVE_RESTORE

static uint32_t ecs[2];

static void deep_sleep_save_ecs(void)
{
	ecs[0] = ECS_REGS->ETM_CTRL;
	ecs[1] = ECS_REGS->DEBUG_CTRL;
	ECS_REGS->ETM_CTRL = 0;
	ECS_REGS->DEBUG_CTRL = 0x00;
}

struct ds_timer_info {
	uintptr_t addr;
	uint32_t restore_mask;
};

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
#define NUM_DS_TIMER_ENTRIES \
	(sizeof(ds_timer_tbl) / sizeof(struct ds_timer_info))


static uint32_t timers[NUM_DS_TIMER_ENTRIES];
static uint8_t uart_activate[3];

#ifdef CONFIG_DEEP_SLEEP_ADC
struct ds_adc_info {
	uint32_t adc_ctrl;
	uint32_t adc_cfg;
};

static struct ds_adc_info adc_info;
#endif


#ifdef CONFIG_DEEP_SLEEP_PECI
static uint32_t peci_info;
#endif

static uint32_t others[15];

static void deep_sleep_save_uarts(void)
{
	printk("UART disabled!\r\n");
	uart_activate[0] = UART0_REGS->ACTV;
	if (uart_activate[0]) {
		while ((UART0_REGS->LSR & MCHP_UART_LSR_TEMT) == 0) {
		};
	}
	UART0_REGS->ACTV = 0;
	uart_activate[1] = UART1_REGS->ACTV;
	if (uart_activate[1]) {
		while ((UART1_REGS->LSR & MCHP_UART_LSR_TEMT) == 0) {
		};
	}
	UART1_REGS->ACTV = 0;
	uart_activate[2] = UART2_REGS->ACTV;
	if (uart_activate[2]) {
		while ((UART2_REGS->LSR & MCHP_UART_LSR_TEMT) == 0) {
		};
	}

	mchp_pcr_periph_slp_ctrl(PCR_UART0, MCHP_PCR_SLEEP_EN);
	mchp_pcr_periph_slp_ctrl(PCR_UART1, MCHP_PCR_SLEEP_EN);
	mchp_pcr_periph_slp_ctrl(PCR_UART2, MCHP_PCR_SLEEP_EN);

#ifdef CONFIG_DEEP_SLEEP_DEACTIVATE_UART
	UART2_REGS->ACTV = 0;
	UART1_REGS->ACTV = 0;
	UART0_REGS->ACTV = 0;
#endif
}

static void deep_sleep_save_timers(void)
{
	const struct ds_timer_info *p;
	uint32_t i;

	p = &ds_timer_tbl[0];
	for (i = 0; i < NUM_DS_TIMER_ENTRIES; i++) {
		timers[i] = REG32(p->addr);
		REG32(p->addr) = 0;
		p++;
	}
}

static void deep_sleep_restore_ecs(void)
{
	ECS_REGS->ETM_CTRL = ecs[0];
	ECS_REGS->DEBUG_CTRL = ecs[1];
}

static void deep_sleep_restore_uarts(void)
{
	if (uart_activate[0]) {
		mchp_pcr_periph_slp_ctrl(PCR_UART0, MCHP_PCR_SLEEP_DIS);
	}

	if (uart_activate[1]) {
		mchp_pcr_periph_slp_ctrl(PCR_UART1, MCHP_PCR_SLEEP_DIS);

#ifdef CONFIG_UART_GPIOS_OFF
	GPIO_CTRL_REGS->CTRL_0170 = UART_GPIO_CFG |
				    MCHP_GPIO_CTRL_MUX_F1;
	GPIO_CTRL_REGS->CTRL_0171 = UART_GPIO_CFG |
				    MCHP_GPIO_CTRL_MUX_F1;
#endif

		UART1_REGS->SCR = 0;
		UART1_REGS->CFG_SEL = (MCHP_UART_LD_CFG_INTCLK +
				       MCHP_UART_LD_CFG_RESET_SYS +
				       MCHP_UART_LD_CFG_NO_INVERT);
		UART1_REGS->IER = 0u;
		UART1_REGS->MCR = 0u;
		UART1_REGS->LCR |= MCHP_UART_LCR_DLAB_EN;
		UART1_REGS->RTXB = 0x01u;
		UART1_REGS->IER = 0u;

		UART1_REGS->LCR &= ~MCHP_UART_LCR_DLAB_EN;
		UART1_REGS->LCR |= (MCHP_UART_LCR_WORD_LEN_8 |
			MCHP_UART_LCR_STOP_BIT_1 | MCHP_UART_LCR_PARITY_NONE);
	}

	if (uart_activate[2]) {
		ECS_REGS->DEBUG_CTRL = (MCHP_ECS_DCTRL_DBG_EN |
			MCHP_ECS_DCTRL_MODE_SWD);
		mchp_pcr_periph_slp_ctrl(PCR_UART2, MCHP_PCR_SLEEP_DIS);
#ifdef CONFIG_UART_GPIOS_OFF
		GPIO_CTRL_REGS->CTRL_0145 = UART_GPIO_CFG |
					    MCHP_GPIO_CTRL_MUX_F2;
		GPIO_CTRL_REGS->CTRL_0146 = UART_GPIO_CFG |
					    MCHP_GPIO_CTRL_MUX_F2;
#endif
	}

#ifdef CONFIG_DEEP_SLEEP_DEACTIVATE_UART
	UART0_REGS->ACTV = uart_activate[0];
	UART1_REGS->ACTV = uart_activate[1];
	UART2_REGS->ACTV = uart_activate[2];
#endif
}

static void deep_sleep_restore_timers(void)
{
	const struct ds_timer_info *p;
	uint32_t i;

	p = &ds_timer_tbl[0];
	for (i = 0; i < NUM_DS_TIMER_ENTRIES; i++) {
		REG32(p->addr) = timers[i] & ~p->restore_mask;
		p++;
	}
}

#ifdef CONFIG_BLOCKS_POWER_OPTIMIZATION
static void deep_sleep_save_blocks(void)
{
	/* ADC Power saving feature is enabled */
#ifdef CONFIG_DEEP_SLEEP_ADC
	adc_info.adc_ctrl = ADC_REGS->CONTROL;
	adc_info.adc_cfg = ADC_REGS->CONFIG;
	ADC_REGS->CONTROL = 0;
	ADC_REGS->CONFIG = 0;
#endif

#ifdef CONFIG_DEEP_SLEEP_PECI
	/* Disable PECI */
	PECI_REGS->CONTROL = 0x08;
	PECI_REGS->CONTROL = 0x09;
	peci_info = ECS_REGS->PECI_DIS;
	ECS_REGS->PECI_DIS = 0x01;
#endif

#ifdef CONFIG_DEEP_SLEEP_COMP
	/* Comparators */
	others[0] = ECS_REGS->CMP_SLP_CTRL;
	ECS_REGS->CMP_SLP_CTRL = 0;
#endif

#ifdef CONFIG_DEEP_SLEEP_VCI
	/* Disable VCI_INPUT_ENABLE */
	others[1] = VCI_REGS->INPUT_EN;
	VCI_REGS->INPUT_EN = 0;
#endif

#ifdef CONFIG_DEEP_SLEEP_WEEK
	/* Disable WEEK TIMER */
	others[2] = WKTMR_REGS->BGPO_PWR;
	WKTMR_REGS->BGPO_PWR = 0;
#endif

#ifdef CONFIG_DEEP_SLEEP_SLW_CLK
	/* Set SLOW_CLOCK_DIVIDE = CLKOFF */
	others[3] = PCR_REGS->SLOW_CLK_CTRL;
	PCR_REGS->SLOW_CLK_CTRL &= (~MCHP_PCR_SLOW_CLK_CTRL_100KHZ &
				    MCHP_PCR_SLOW_CLK_CTRL_MASK);

	others[4] = TFDP_REGS->CTRL;
	TFDP_REGS->CTRL = 0;
#endif

#ifdef CONFIG_DEEP_SLEEP_ESPI
	/* eSPI */
	mchp_pcr_periph_slp_ctrl(PCR_ESPI, MCHP_PCR_SLEEP_EN);
#endif
#ifdef CONFIG_ESPI_DEACTIVATE
	others[5] = ESPI_EIO_BAR_REGS->IO_ACTV;
	ESPI_EIO_BAR_REGS->IO_ACTV = 0;
#endif

#ifdef CONFIG_DEEP_SLEEP_P80
	/* Port 80 */
	others[6] = PORT80_CAP0_REGS->ACTV;
	others[7] = PORT80_CAP1_REGS->ACTV;
	PORT80_CAP0_REGS->ACTV = 0;
	PORT80_CAP1_REGS->ACTV = 0;
#endif


#ifdef CONFIG_DEEP_SLEEP_I2C
	/* SMBUS */
	others[8] = SMB0_REGS->CTRLSTS;
	others[9] = SMB1_REGS->CTRLSTS;
	others[10] = SMB2_REGS->CTRLSTS;
	others[11] = SMB3_REGS->CTRLSTS;
	others[12] = SMB4_REGS->CTRLSTS;

	SMB0_REGS->CTRLSTS = 0;
	SMB1_REGS->CTRLSTS = 0;
	SMB2_REGS->CTRLSTS = 0;
	SMB3_REGS->CTRLSTS = 0;
	SMB4_REGS->CTRLSTS = 0;
#endif
}
#endif /* CONFIG_BLOCKS_POWER_OPTIMIZATION */

#ifdef CONFIG_BLOCKS_POWER_OPTIMIZATION
static void deep_sleep_restore_blocks(void)
{
#ifdef CONFIG_DEEP_SLEEP_ADC
	ADC_REGS->CONTROL = adc_info.adc_ctrl;
	ADC_REGS->CONFIG = adc_info.adc_cfg;
#endif

#ifdef CONFIG_DEEP_SLEEP_PECI
	/* Disable PECI */
	ECS_REGS->PECI_DIS = peci_info;
#endif

#ifdef CONFIG_DEEP_SLEEP_COMP
	/* Comparators */
	ECS_REGS->CMP_SLP_CTRL = others[0];
#endif

#ifdef CONFIG_DEEP_SLEEP_VCI
	/* Disable VCI_INPUT_ENABLE */
	VCI_REGS->INPUT_EN = others[1];
#endif

#ifdef CONFIG_DEEP_SLEEP_WEEK
	/* Disable WEEK TIMER */
	WKTMR_REGS->BGPO_PWR = others[2];
#endif

#ifdef CONFIG_DEEP_SLEEP_SLW_CLK
	/* Set SLOW_CLOCK_DIVIDE = CLKOFF */
	PCR_REGS->SLOW_CLK_CTRL = others[3];

	TFDP_REGS->CTRL = others[4];
#endif

#ifdef CONFIG_DEEP_SLEEP_ESPI
	/* eSPI */
	mchp_pcr_periph_slp_ctrl(PCR_ESPI, MCHP_PCR_SLEEP_DIS);
#endif
#ifdef CONFIG_ESPI_DEACTIVATE
	ESPI_EIO_BAR_REGS->IO_ACTV = others[5];
#endif

#ifdef CONFIG_DEEP_P80
	/* Port 80 */
	PORT80_CAP0_REGS->ACTV = others[6];
	PORT80_CAP1_REGS->ACTV = others[7];
#endif

#ifdef CONFIG_DEEP_I2C
	/* SMBUS */
	SMB0_REGS->CTRLSTS = others[8];
	SMB1_REGS->CTRLSTS = others[9];
	SMB2_REGS->CTRLSTS = others[10];
	SMB3_REGS->CTRLSTS = others[11];
	SMB4_REGS->CTRLSTS = others[12];
#endif
}
#endif /* CONFIG_BLOCKS_POWER_OPTIMIZATION */

void unpower_all_gpios(void)
{
	/* PROCHOT/ PM_SLP_SUS_N */
	GPIO_CTRL_REGS->CTRL_0000 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	/* Not used */
	GPIO_CTRL_REGS->CTRL_0002 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

#ifdef CONFIG_I2C_GPIOS_OFF
	GPIO_CTRL_REGS->CTRL_0003 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0004 = MCHP_GPIO_CTRL_PWRG_OFF;
#endif

#ifdef CONFIG_PS2_GPIOS_OFF
	GPIO_CTRL_REGS->CTRL_0007 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0010 = MCHP_GPIO_CTRL_PWRG_OFF;
#endif
	/* EC SMI */
	GPIO_CTRL_REGS->CTRL_0011 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* RSMRST_PWRGD */

	/* GPIO_CTRL_REGS->CTRL_0012 = MCHP_GPIO_CTRL_PWRG_OFF; */

	/* EC_PWR_PWRBTN_N
	 * Powering this off prevents long deep sleep
	 * GPIO_CTRL_REGS->CTRL_0013 = MCHP_GPIO_CTRL_PWRG_OFF |
	 * MCHP_GPIO_CTRL_MUX_F0;
	 */

	/* KBD_BKLT_CTRL */

	/* TRIGGER in APP side
	 * GPIO_CTRL_REGS->CTRL_0014 = MCHP_GPIO_CTRL_PWRG_OFF |
	 * MCHP_GPIO_CTRL_MUX_F0;
	 */

	/* Not used */
	GPIO_CTRL_REGS->CTRL_0015 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* SPI */
#ifdef CONFIG_SPI_PINS_OFF
	GPIO_CTRL_REGS->CTRL_0016 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	GPIO_CTRL_REGS->CTRL_0017 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0020 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0021 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
#endif
	/* Misc */
	GPIO_CTRL_REGS->CTRL_0022 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0023 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0024 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0025 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* Keyscan */
	GPIO_CTRL_REGS->CTRL_0026 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0027 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0030 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0031 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0032 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* Not used */
	GPIO_CTRL_REGS->CTRL_0033 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0034 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* Not used */
	GPIO_CTRL_REGS->CTRL_0035 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0036 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* Keyscan */
	GPIO_CTRL_REGS->CTRL_0040 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* PECI */
	GPIO_CTRL_REGS->CTRL_0042 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0043 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0044 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* Keyscan */
	GPIO_CTRL_REGS->CTRL_0045 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0046 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0047 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* TACH */
	GPIO_CTRL_REGS->CTRL_0050 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* WAKE_SCI */
	/* GPIO_CTRL_REGS->CTRL_0051 = MCHP_GPIO_CTRL_PWRG_OFF */

	GPIO_CTRL_REGS->CTRL_0052 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0053 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* PM_RSMRST */
	/* GPIO_CTRL_REGS->CTRL_0054 = MCHP_GPIO_CTRL_PWRG_OFF; */

	/* SPI */
	GPIO_CTRL_REGS->CTRL_0055 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0056 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* ALL_SYS_PWRGD */
	/* GPIO_CTRL_REGS->CTRL_0057 = MCHP_GPIO_CTRL_PWRG_OFF |
	 *			    MCHP_GPIO_CTRL_MUX_F0;
	 */

#ifndef CONFIG_SYS_PM_DEBUG
	GPIO_CTRL_REGS->CTRL_0060 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
#endif

#ifdef CONFIG_ESPI_GPIOS_OFF
	GPIO_CTRL_REGS->CTRL_0061 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0063 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0065 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0066 = MCHP_GPIO_CTRL_PWRG_OFF;

	GPIO_CTRL_REGS->CTRL_0070 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0071 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0072 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0073 = MCHP_GPIO_CTRL_PWRG_OFF;
#endif
	/* Not used */
	GPIO_CTRL_REGS->CTRL_0062 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	GPIO_CTRL_REGS->CTRL_0064 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0 |
				    MCHP_GPIO_CTRL_IDET_DISABLE;
	/* ADC VCREF */
	GPIO_CTRL_REGS->CTRL_0067 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* SLP S0 */
	GPIO_CTRL_REGS->CTRL_0100 = MCHP_GPIO_CTRL_PWRG_OFF;

	/* Misc */
	GPIO_CTRL_REGS->CTRL_0101 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0101 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	/* Not used */
	GPIO_CTRL_REGS->CTRL_0102 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0104 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0105 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	/* PCH_PWROK_EC_R */

	/* GPIO_CTRL_REGS->CTRL_0106 = MCHP_GPIO_CTRL_PWRG_OFF; */

	/* Keyscan */
	GPIO_CTRL_REGS->CTRL_0107 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0112 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0113 = MCHP_GPIO_CTRL_PWRG_OFF;

	GPIO_CTRL_REGS->CTRL_0114 = MCHP_GPIO_CTRL_PWRG_OFF;

	/* Not used */
	/* GPIO_CTRL_REGS->CTRL_0115 = MCHP_GPIO_CTRL_PWRG_OFF; */

	/* Keyscan */
	GPIO_CTRL_REGS->CTRL_0120 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0121 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0122 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0123 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0124 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0125 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0126 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0127 = MCHP_GPIO_CTRL_PWRG_OFF;

	/* USB-C I2C */
	GPIO_CTRL_REGS->CTRL_0130 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0131 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0132 = MCHP_GPIO_CTRL_PWRG_OFF;

	GPIO_CTRL_REGS->CTRL_0140 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0141 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0142 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0143 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0144 = MCHP_GPIO_CTRL_PWRG_OFF;

#ifdef CONFIG_UART_GPIOS_OFF
	GPIO_CTRL_REGS->CTRL_0145 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0146 = MCHP_GPIO_CTRL_PWRG_OFF;
#endif
	/* Not used */
	GPIO_CTRL_REGS->CTRL_0147 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0150 = MCHP_GPIO_CTRL_PWRG_OFF;

	/* Keyscan */
	GPIO_CTRL_REGS->CTRL_0151 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0152 = MCHP_GPIO_CTRL_PWRG_OFF;

	/* SMBUS Alert */
	GPIO_CTRL_REGS->CTRL_0153 = MCHP_GPIO_CTRL_PWRG_OFF;

#ifdef CONFIG_PS2_GPIOS_OFF
	GPIO_CTRL_REGS->CTRL_0154 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0155 = MCHP_GPIO_CTRL_PWRG_OFF;
#endif
	/* LEDs */
	GPIO_CTRL_REGS->CTRL_0156 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0157 = MCHP_GPIO_CTRL_PWRG_OFF;

	/* VCI */
	GPIO_CTRL_REGS->CTRL_0161 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	GPIO_CTRL_REGS->CTRL_0165 = MCHP_GPIO_CTRL_PWRG_OFF;

#ifdef CONFIG_UART_GPIOS_OFF
	GPIO_CTRL_REGS->CTRL_0170 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0171 = MCHP_GPIO_CTRL_PWRG_OFF;
#endif
	/* VCI */
	GPIO_CTRL_REGS->CTRL_0172 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* Not used */
	GPIO_CTRL_REGS->CTRL_0175 = MCHP_GPIO_CTRL_PWRG_OFF;

	/* TRACE */
	GPIO_CTRL_REGS->CTRL_0200 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0201 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0202 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0203 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* ADC */
	GPIO_CTRL_REGS->CTRL_0204 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0205 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0206 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;
	GPIO_CTRL_REGS->CTRL_0207 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_MUX_F0;

	/* Misc/Prochot */
	GPIO_CTRL_REGS->CTRL_0222 = MCHP_GPIO_CTRL_PWRG_OFF;

	/* SPI */
#ifdef CONFIG_SPI_PINS_OFF
	GPIO_CTRL_REGS->CTRL_0223 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0224 = MCHP_GPIO_CTRL_PWRG_OFF;
#endif

	/* Not used */
	GPIO_CTRL_REGS->CTRL_0226 = MCHP_GPIO_CTRL_PWRG_OFF;

	/* SPI */
#ifdef CONFIG_SPI_PINS_OFF
	GPIO_CTRL_REGS->CTRL_0227 = MCHP_GPIO_CTRL_PWRG_OFF;
#endif

	/* Not used */
	GPIO_CTRL_REGS->CTRL_0240 = MCHP_GPIO_CTRL_PWRG_OFF;

	/* Misc */
	GPIO_CTRL_REGS->CTRL_0241 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0242 = MCHP_GPIO_CTRL_PWRG_OFF;

	/* SLP S0/Misc */
	GPIO_CTRL_REGS->CTRL_0243 = MCHP_GPIO_CTRL_PWRG_OFF;

	/* Misc */
	GPIO_CTRL_REGS->CTRL_0244 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0245 = MCHP_GPIO_CTRL_PWRG_OFF;
	GPIO_CTRL_REGS->CTRL_0246 = MCHP_GPIO_CTRL_PWRG_OFF;

	/* Misc */
	GPIO_CTRL_REGS->CTRL_0250 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_IDET_DISABLE;
	GPIO_CTRL_REGS->CTRL_0253 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_IDET_DISABLE;
	GPIO_CTRL_REGS->CTRL_0254 = MCHP_GPIO_CTRL_PWRG_OFF |
				    MCHP_GPIO_CTRL_IDET_DISABLE;

}

void soc_deep_sleep_periph_save(void)
{
	deep_sleep_save_uarts();
	deep_sleep_save_ecs();
	deep_sleep_save_timers();
#ifdef CONFIG_BLOCKS_POWER_OPTIMIZATION
	deep_sleep_save_blocks();
#endif
#ifdef CONFIG_GPIO_POWER_OPTIMIZATION
	unpower_all_gpios();
#endif
}

void soc_deep_sleep_periph_restore(void)
{
	deep_sleep_restore_ecs();
	deep_sleep_restore_uarts();
	deep_sleep_restore_timers();
#ifdef CONFIG_BLOCKS_POWER_OPTIMIZATION
	deep_sleep_restore_blocks();
#endif
}

#else


#endif /* DEEP_SLEEP_PERIPH_SAVE_RESTORE */

#endif /* CONFIG_SYS_POWER_DEEP_SLEEP_STATES */
