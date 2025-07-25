/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(app, CONFIG_LOG_DEFAULT_LEVEL);

#define MEC_VB_MEM_BASE_ADDR DT_REG_ADDR(DT_NODELABEL(bbram))
#define MEC_VB_MEM_SIZE 64u

int main(void)
{
	mem_addr_t maddr = MEC_VB_MEM_BASE_ADDR;
	uint16_t va1 = 0;
	uint8_t val = 0xA5u;
	uint8_t val2 = 0;

	LOG_INF("8-bit Write 0x%02x to [0x%0lx]", val, maddr);

	sys_write8(val, maddr);
	val2 = sys_read8(maddr);

	LOG_INF("8-bit Read [0x%0lx] = 0x%02x", maddr, val2);

	soc_set_bit8(maddr, BIT(1));
	val2 = sys_read8(maddr);
	if ((val2 & BIT(1)) == 0) {
		LOG_ERR("Failed to set BIT(1): result = 0x%02x", val2);
	}

	soc_clear_bit8(maddr, BIT(7));
	val2 = sys_read8(maddr);
	if ((val2 & BIT(7)) != 0) {
		LOG_ERR("Failed to clear BIT(7): result = 0x%02x", val2);
	}

	maddr += 2u;
	sys_write16(0, maddr);
	va1 = sys_read16(maddr);
	LOG_INF("Write 16-bit 0 to [%0lx]", maddr);

	for (uint32_t n = 0; n < 16; n++) {
		soc_set_bit16(maddr, n);
		if (soc_test_bit16(maddr, n) == 0) {
			LOG_ERR("16-bit set bit %u", n);
		}
	}

	printf("\nProgram End\n");

	return 0;
}
