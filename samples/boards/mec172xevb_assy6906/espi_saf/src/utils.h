/*
 * Copyright (c) 2022 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdint.h>

void pr_espi_regs(void);
void pr_espi_saf_regs(void);
void pr_byte_buffer(const uint8_t *buf, size_t bufsz);

#endif /* __UTILS_H__ */
