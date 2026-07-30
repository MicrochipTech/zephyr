/*
 * (c) 2026 Microchip Technology Inc. and its subsidiaries.
 * SPDX-License-Identifier: Apache-2.0
 *
 * MTR (MEC176x) polled GPIO bring-up validation.
 *
 * Configures GPIO020 (bank gpio_000_036, Zephyr pin index 16; octal 020 = 16)
 * as an output and toggles it in a busy-loop. GPIO020 is routed to the IO-card
 * header pin J3.3 on the current FPGA bitstream, so it can be observed with a
 * logic analyzer / scope on J3.3. It can also be verified over the debug Mem-AP:
 *   - PAROUT bank0 @ 0x40081380 bit 16 (0x10000) tracks the drive (1=high, 0=low)
 *   - CR1 GPIO020   @ 0x40081040 shows DIR=output (bit9) / MUX=0 (GPIO function)
 *
 * BITSTREAM CAVEAT (current MTR proFPGA): the driver drives outputs through the
 * PAROUT parallel register (CR1.OCR=1). On this bitstream the pad output latch is
 * fed only by the CR1.ODAT path (OCR=0); the PAROUT->latch mirror is not wired, so
 * PAROUT writes update the register but do NOT reach the physical pin. External
 * (J3.3) observation is therefore blocked on the FPGA/IO-card side, not the driver.
 * The driver path is correct for real MEC silicon. See docs mtr_gpio_driver.md 5c.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

#define GPIO_NODE DT_NODELABEL(gpio_000_036)
#define TEST_PIN  16 /* GPIO020 (octal 020 = 16), routed to J3.3 */

int main(void)
{
	const struct device *gpio = DEVICE_DT_GET(GPIO_NODE);
	int ret;

	printk("*** MTR polled GPIO toggle (GPIO020 / J3.3) ***\n");

	if (!device_is_ready(gpio)) {
		printk("FAIL: gpio_000_036 device not ready\n");
		return 0;
	}

	ret = gpio_pin_configure(gpio, TEST_PIN, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		printk("FAIL: gpio_pin_configure ret=%d\n", ret);
		return 0;
	}
	printk("OK: GPIO020 output configured. Toggling; watch J3.3 (PAROUT 0x40081380 bit16).\n");

	/*
	 * Use a busy-loop delay, NOT k_msleep(): the FPGA core clock does not match
	 * CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC yet (mec1767xyz.dtsi TODO), so kernel
	 * timeouts run far slower than wall-clock. A volatile spin is timer-independent
	 * and lets a Mem-AP read of PAROUT catch bit16 in both states.
	 */
	volatile uint32_t spin;

	while (1) {
		gpio_pin_set(gpio, TEST_PIN, 1);
		for (spin = 0; spin < 300000U; spin++) {
		}
		gpio_pin_set(gpio, TEST_PIN, 0);
		for (spin = 0; spin < 300000U; spin++) {
		}
	}

	return 0;
}
