/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr.h>
#include <device.h>
#include <soc.h>
#include <gpio.h>
#include <logging/log.h>
LOG_MODULE_DECLARE(ecfw, CONFIG_EC_LOG_LEVEL);

#define PRINT_DURING_TEST

/* Large values in ms to test k_busy_wait / k_uptime_get */
#define DURATION 100
#define PERIOD 50
#define EXPIRE_TIMES 4

/* size of stack area used by each thread */
#define TASK_STACK_SIZE 2048 // MCHP was 1024
/* scheduling priority used by each thread */
/* MCHP get exception invalid priority (20); allowed range: 14 to -16 */
#define PRIORITY 14 /* was 20 */


enum tmr_delay {
	TMRDLY_USE_KSLEEP,
	TMRDLY_USE_KUSLEEP,
	TMRDLY_USE_KBUSYWAIT,
};

static struct device *gpio_devA;
static struct device *gpio_devB;

static struct k_thread blink_id;

K_THREAD_STACK_DEFINE(blink_stack, TASK_STACK_SIZE);


void toggle_us(u32_t delay_us)
{
	/* Minimum 6 times, maximum 100 */
	u32_t count = 100;
	if (delay_us > 1000) {
		count = 6;
	}

	LOG_DBG("Toggle pin %d times after (%d) us\n", count, delay_us);

	while(count--){
		k_busy_wait(delay_us);
		GPIO_CTRL_REGS->CTRL_0013 ^= (1ul << 16);
	}
}

void toggle_ms(u32_t delay_ms, enum tmr_delay val)
{
	/* Minimum 3 times, maximum 1s */
	u32_t count = 1*1000 / delay_ms;
	if (count < 10) {
		count = 4;
	}

	GPIO_CTRL_REGS->CTRL_0013 = 0x10240UL;

	LOG_DBG("Toggle pin %d times after (%d) ms\n", count, delay_ms);

	u8_t toggle = 0x01;
	while(count--){
		toggle ^= 0x1;

		//printk(".");

		switch(val) {
		case TMRDLY_USE_KSLEEP:
			k_sleep(delay_ms);
			break;
		case TMRDLY_USE_KUSLEEP:
			k_usleep(delay_ms*1000);
			break;
		case TMRDLY_USE_KBUSYWAIT:
			k_busy_wait(delay_ms*1000);
			break;
		default:
			printk("Unsupported!\n");
			break;
		}

		GPIO_CTRL_REGS->CTRL_0013 ^= (1ul << 16);
		//gpio_pin_write(gpio_devA, MCHP_GPIO_013, toggle);
	}
}

extern volatile u32_t dbg_zclk_set; /* DEBUG */

int ms_delay_tests(void)
{
	enum tmr_delay d = TMRDLY_USE_KSLEEP;

	gpio_pin_configure(gpio_devA, MCHP_GPIO_013, GPIO_DIR_OUT);

	toggle_ms(1, d);
	toggle_ms(5, d);
	toggle_ms(10, d);
	toggle_ms(20, d);
	//toggle_ms(50, d);
	toggle_ms(100, d);
	//toggle_ms(200, d);
	//toggle_ms(500, d);
	toggle_ms(1000, d);
	toggle_ms(2000, d);

	return 0;
}

int us_delay_tests(void)
{
	/* Need to control using direct register access
	 * to avoid latency introduced by gpio driver API
	 */
	GPIO_CTRL_REGS->CTRL_0013 = 0x10240ul;

	toggle_us(5);
	toggle_us(10);
	toggle_us(20);
	toggle_us(50);
	toggle_us(100);
	toggle_us(200);
	toggle_us(500);
	toggle_us(1*1000);
	toggle_us(2*1000);
	toggle_us(10*1000);
	toggle_us(50*1000);
	toggle_us(100*1000);
	u32_t t = DURATION + PERIOD * EXPIRE_TIMES + PERIOD / 2;
	toggle_us(t*1000);

	return 0;
}

static u8_t blink_enabled;

static void blink(void *p1, void *p2, void *p3)
{
	while(blink_enabled) {
		gpio_pin_write(gpio_devB, MCHP_GPIO_157, 0);
		k_sleep(1000);
		gpio_pin_write(gpio_devB, MCHP_GPIO_157, 1);
#ifdef PRINT_DURING_TEST
		printk("Hello Worldz from blink! %s\n", CONFIG_BOARD);
#endif
		k_sleep(1000);
	}
}

void single_toggle_in_thread(void)
{
	struct k_thread *thread = k_current_get();
	printk("Main thread: %p (%s)\n", thread,
		      k_thread_name_get(thread));

	k_thread_create(&blink_id, blink_stack, TASK_STACK_SIZE, blink,
		NULL, NULL, NULL, PRIORITY,  K_INHERIT_PERMS, K_FOREVER);

	printk("Blink thread: %p (%s)\n", &blink_id,
		      k_thread_name_get(&blink_id));

	/* gpio for turning let on/off */
	gpio_pin_configure(gpio_devB, MCHP_GPIO_157, GPIO_DIR_OUT);

	k_thread_start(&blink_id);
}

int gpio_test(void)
{
	LOG_DBG("GPIO tests! %s\n", CONFIG_BOARD);

	gpio_devA = device_get_binding(DT_GPIO_XEC_GPIO000_036_LABEL);
	gpio_devB = device_get_binding(DT_GPIO_XEC_GPIO140_176_LABEL);

	/* Test 1: Single-thread, 1 GPIO toggle using k_busy_wait */
	LOG_DBG("Test 1\n");
	LOG_DBG("PROC_CLK_CTRL %x\n", PCR_REGS->PROC_CLK_CTRL);
	us_delay_tests();
	k_sleep(1000);

	/* Test 2: Single-thread, 1 GPIO toggle using k_sleep */
	LOG_DBG("Test 2\n");
	ms_delay_tests();

	/* Test 3: Multi-thread thread, single GPIO toggle using k_sleep */
	LOG_DBG("Test 3\n");
	blink_enabled = 1;
	single_toggle_in_thread();
	k_sleep(5000);
	blink_enabled = 0;

	/* Test 4: Multi-thread thread, multiple GPIO toggle using k_sleep
	 * Achieved by restarting the previous tests
	 */
	LOG_DBG("Test 4\n");
	blink_enabled = 1;
	ms_delay_tests();
	k_sleep(5000);
	blink_enabled = 0;

	LOG_DBG("Completed!\n");
	return 0;
}
