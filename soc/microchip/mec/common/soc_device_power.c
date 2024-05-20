/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/pm/pm.h>

#include "soc_device_power.h"


#ifdef DEBUG_DEEP_SLEEP_CLK_REQ
void soc_debug_sleep_clk_req(void)
{
	/* TODO: Save PCR CLK_REQ and ECS sleep status registers to VBAT memory */
}
#endif

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
#ifdef CONFIG_ESPI
	/* TODO HAL API to enable eSPI GIRQ22 Wake */
#endif
}

void soc_deep_sleep_non_wake_dis(void)
{
#ifdef CONFIG_ESPI
	/* TODO HAL API to disable eSPI GIRQ22 Wake */
#endif
}

void soc_deep_sleep_wake_en(void)
{
	/* TODO Enable varous device wakes: currently keyscan and PS/2
	 * This handled more easily by the device driver when CONFIG_PM_DEVICE=y
	 * For CONFIG_PM_DEVICE=n we need some complicated DT or Kconfig IF-DEF logic.
	 * DT has wakeup-source property indicating this device can be a wakeup source.
	 */
}

void soc_deep_sleep_wake_dis(void)
{
	/* TODO same as soc_deep_sleep_wake_en */
}

/* Variables used to save various HW state */
#ifdef DEEP_SLEEP_PERIPH_SAVE_RESTORE

static void deep_sleep_save_ecs(void)
{
	/* TODO HAL API to pass memory buffer to save ECS stuff
	 * Need HAL define for size of buffer
	 * Currently the DEBUG and ETM control registers
	 */
}

#ifdef DEEP_SLEEP_UART_SAVE_RESTORE
static void deep_sleep_save_uarts(void)
{
	/* TODO HAL API to pass memory buffer to save UART activate regs
	 * Need HAL define for size of buffer.
	 * Currently save UART Activate register after waiting for UART TX FIFO to drain
	 * NOTE: MEC174x/175x UART has new UART register containing number of bytes in TX FIFO.
	 */
}
#endif

static void deep_sleep_save_timers(void)
{
	/* TODO HAL API to pass memory buffer to save timer control register.
	 * Need HAL define for size of buffer.
	 * Currently save basic timers. Need to expand to other timers that don't clear CLK_REQ
	 * when SLP_EN is asserted.
	 */
}

static void deep_sleep_restore_ecs(void)
{
	/* TODO restore EC subsystem registers on wake */
}

#ifdef DEEP_SLEEP_UART_SAVE_RESTORE
static void deep_sleep_restore_uarts(void)
{
	/* TODO restore UARTs on wake */
}
#endif

static void deep_sleep_restore_timers(void)
{
	/* TODO restore timers on wake */
}

#ifdef DEEP_SLEEP_PERIPH_SAVE_RESTORE_EXTENDED

static void deep_sleep_save_blocks(void)
{
	/* TODO save/disable other peripherals such as ADC, PECI, I2C, ECS analog comparator,
	 * Turn off PCR slow clock to minimize power
	 */
}

static void deep_sleep_restore_blocks(void)
{
	/* TODO restore other peripherals such as ADC, PECI, I2C, ECS analog comparator,
	 * Restore PCR slow clock to minimize power
	 */
}
#endif /* DEEP_SLEEP_PERIPH_SAVE_RESTORE_EXTENDED */

/* TODO generalize via DT?
 * Generate a build time table of peripherals?
 */
void soc_deep_sleep_periph_save(void)
{
#ifdef DEEP_SLEEP_PERIPH_SAVE_RESTORE_EXTENDED
	deep_sleep_save_blocks();
#endif
	deep_sleep_save_ecs();
	deep_sleep_save_timers();
#ifdef DEEP_SLEEP_UART_SAVE_RESTORE
	deep_sleep_save_uarts();
#endif
}

/* TODO generalize via DT? */
void soc_deep_sleep_periph_restore(void)
{
	deep_sleep_restore_ecs();
#ifdef DEEP_SLEEP_UART_SAVE_RESTORE
	deep_sleep_restore_uarts();
#endif
	deep_sleep_restore_timers();
#ifdef DEEP_SLEEP_PERIPH_SAVE_RESTORE_EXTENDED
	deep_sleep_restore_blocks();
#endif
}

#else

void soc_deep_sleep_periph_save(void) {}

void soc_deep_sleep_periph_restore(void) {}

#endif /* DEEP_SLEEP_PERIPH_SAVE_RESTORE */
