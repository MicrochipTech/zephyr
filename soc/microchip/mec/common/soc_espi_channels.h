/*
 * Copyright (c) 2019 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_ESPI_CHANNELS_H_
#define _SOC_ESPI_CHANNELS_H_

#include <zephyr/toolchain.h>
#include <zephyr/types.h>
#include <zephyr/sys/sys_io.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Channel 0 - Peripheral Channel */

/* 8042 event data */
#define E8042_ISR_DATA_POS	8U
#define E8042_ISR_CMD_DATA_POS	0U

static ALWAYS_INLINE void sys_set_bit8(mem_addr_t addr, unsigned int bit)
{
        uint8_t temp = *(volatile uint8_t *)addr;

        *(volatile uint8_t *)addr = temp | (1U << bit);
}

static ALWAYS_INLINE void sys_clear_bit8(mem_addr_t addr, unsigned int bit)
{
        uint8_t temp = *(volatile uint8_t *)addr;

        *(volatile uint8_t *)addr = temp & ~(1U << bit);
}

static ALWAYS_INLINE int sys_test_bit8(mem_addr_t addr, unsigned int bit)
{
        uint8_t temp = *(volatile uint8_t *)addr;

        return temp & (1U << bit);
}

static ALWAYS_INLINE void sys_set_bits8(mem_addr_t addr, unsigned int mask)
{
        uint8_t temp = *(volatile uint8_t *)addr;

        *(volatile uint8_t *)addr = temp | mask;
}

static ALWAYS_INLINE void sys_clear_bits8(mem_addr_t addr, unsigned int mask)
{
        uint8_t temp = *(volatile uint8_t *)addr;

        *(volatile uint8_t *)addr = temp & ~mask;
}

#ifdef __cplusplus
}
#endif

#endif /* _SOC_ESPI_CHANNELS_H_ */
