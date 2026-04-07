/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip MEC MCU family QSPI controller common helpers
 *
 */

#ifndef _SOC_MICROCHIP_MEC_COMMON_SOC_QSPI_H_
#define _SOC_MICROCHIP_MEC_COMMON_SOC_QSPI_H_

#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/common/sys_io.h>

int soc_xec_qspi_soft_reset(mm_reg_t qbase);

int soc_xec_qspi_calc_fdiv(uint32_t freq_hz, uint32_t *qspi_freq_div);

int soc_xec_qspi_cpol_cpha_from_freq(uint32_t freq_hz, uint8_t sig_mode, uint8_t *cpol_cpha);

#endif /* _SOC_MICROCHIP_MEC_COMMON_SOC_QSPI_H_ */
