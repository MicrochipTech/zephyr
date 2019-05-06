/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <soc.h>

/* From include/irq.h */
#if 0
/**
 * @brief Initialize an interrupt handler.
 *
 * This routine initializes an interrupt handler for an IRQ. The IRQ must be
 * subsequently enabled before the interrupt handler begins servicing
 * interrupts.
 *
 * @warning
 * Although this routine is invoked at run-time, all of its arguments must be
 * computable by the compiler at build time.
 *
 * @param irq_p IRQ line number.
 * @param priority_p Interrupt priority.
 * @param isr_p Address of interrupt service routine.
 * @param isr_param_p Parameter passed to interrupt service routine.
 * @param flags_p Architecture-specific IRQ configuration flags..
 *
 * @return Interrupt vector assigned to this interrupt.
 */
#define IRQ_CONNECT(irq_p, priority_p, isr_p, isr_param_p, flags_p) \
	Z_ARCH_IRQ_CONNECT(irq_p, priority_p, isr_p, isr_param_p, flags_p)
#endif

static void girq08_aggr_isr(void *isr_args)
{
	printk("GIRQ08 Aggregated ISR\n");
}

static void girq09_aggr_isr(void *isr_args)
{
	printk("GIRQ09 Aggregated ISR\n");
}

static void girq10_aggr_isr(void *isr_args)
{
	printk("GIRQ10 Aggregated ISR\n");
}

static void girq11_aggr_isr(void *isr_args)
{
	printk("GIRQ11 Aggregated ISR\n");
}

static void girq12_aggr_isr(void *isr_args)
{
	printk("GIRQ12 Aggregated ISR\n");
}

static void girq26_aggr_isr(void *isr_args)
{
	printk("GIRQ26 Aggregated ISR\n");
}

static void girq08_bit0(void *isr_args)
{
	printk("GIRQ08 bit 0 level 2 handler\n");
}

static void girq08_bit31(void *isr_args)
{
	printk("GIRQ08 bit 31 level 2 handler\n");
}

static void girq26_bit0(void *isr_args)
{
	printk("GIRQ26 bit 0 level 2 handler\n");
}

static void girq26_bit31(void *isr_args)
{
	printk("GIRQ26 bit 31 level 2 handler\n");
}

void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);
	
	/* Aggregated GIRQ08 handler */
	IRQ_CONNECT(0x0000, 2, girq08_aggr_isr, NULL, 0);
	
	/* Aggregated GIRQ09 handler */
	IRQ_CONNECT(0x0001, 2, girq09_aggr_isr, NULL, 0);
	
	/* Aggregated GIRQ10 handler */
	IRQ_CONNECT(0x0002, 2, girq10_aggr_isr, NULL, 0);
	
	/* Aggregated GIRQ11 handler */
	IRQ_CONNECT(0x0003, 2, girq11_aggr_isr, NULL, 0);
	
	/* Aggregated GIRQ12 handler */
	IRQ_CONNECT(0x0004, 2, girq12_aggr_isr, NULL, 0);
	
	/* Aggregated GIRQ26 handler */
	IRQ_CONNECT(0x0011, 2, girq26_aggr_isr, NULL, 0);
	
	/* GIRQ08 bit[0] handler called by Aggregated GIRQ08 handler 
	 * irq_num bits[7:8] = 0 for Aggregated GIRQ08
	 * irq_num bits[15:8] = level 2 handler number + 1
	 */
	IRQ_CONNECT(0x0100, 2, girq08_bit0, NULL, 0);
	
	/* GIRQ08 bit[31] handler called by Aggregated GIRQ08 handler 
	 * irq_num bits[7:8] = 0 for Aggregated GIRQ08
	 * irq_num bits[15:8] = 0x20 level 2 handler number + 1
	 */
	IRQ_CONNECT(0x2000, 2, girq08_bit31, NULL, 0);
	
	/* GIRQ26 bit[0] handler called by Aggregated GIRQ26 handler 
	 * irq_num bits[7:8] = 17(0x11) for Aggregated GIRQ26
	 * irq_num bits[15:8] = level 2 handler number + 1
	 */
	IRQ_CONNECT(0x0111, 2, girq26_bit0, NULL, 0);
	
	/* GIRQ26 bit[31] handler called by Aggregated GIRQ26 handler 
	 * irq_num bits[7:8] = 17(0x11) for Aggregated GIRQ26
	 * irq_num bits[15:8] = 0x20 level 2 handler number + 1
	 */
	IRQ_CONNECT(0x2011, 2, girq26_bit31, NULL, 0);

}
