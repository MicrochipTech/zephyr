/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/common/sys_io.h>

#include "soc_qspi.h"

#ifdef CONFIG_SOC_SERIES_MEC15XX
#define MEC_QSPI_FDIV_MSK0 GENMASK(7, 0)
#else
#define MEC_QSPI_FDIV_MSK0 GENMASK(15, 0)
#endif

int soc_xec_qspi_soft_reset(mm_reg_t qbase)
{
	uint32_t reg_save_restore[4] = {0};

	if (qbase == 0) {
		return -EINVAL;
	}

	reg_save_restore[0] = sys_read32(qbase + XEC_QSPI_CSTM_OFS);
#ifndef CONFIG_SOC_SERIES_MEC15XX
	reg_save_restore[1] = sys_read32(qbase + XEC_QSPI_TAPS_OFS);
	reg_save_restore[2] = sys_read32(qbase + XEC_QSPI_TAPS_ADJ_OFS);
	reg_save_restore[3] = sys_read32(qbase + XEC_QSPI_TAPS_CR2_OFS);
#endif

	sys_set_bit(qbase + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_SRST_POS);
	while (sys_test_bit(qbase + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_SRST_POS) != 0) {
		;
	}

	sys_write32(reg_save_restore[0], qbase + XEC_QSPI_CSTM_OFS);
#ifndef CONFIG_SOC_SERIES_MEC15XX
	sys_write32(reg_save_restore[1], qbase + XEC_QSPI_TAPS_OFS);
	sys_write32(reg_save_restore[2], qbase + XEC_QSPI_TAPS_ADJ_OFS);
	sys_write32(reg_save_restore[3], qbase + XEC_QSPI_TAPS_CR2_OFS);
#endif

	return 0;
}

/* Calculate the frequency divider field from requested frequency in hertz.
 * QSPI frequency divider uses values [0, 0xffff] where the non-zero values
 * are treated as the actual divider and 0 is a divider of 0x10000.
 */
int soc_xec_qspi_calc_fdiv(uint32_t freq_hz, uint32_t *qspi_freq_div)
{
	uint32_t qspi_core_clock = soc_core_clock_get();

	if (qspi_freq_div == NULL) {
		return -EINVAL;
	}

	/* Requested zero frequency, return maximum HW divider which is 0.
	 * QSPI frequency divider of 0 means divide by 2^8 for MEC15xx or 2^16 all others
	 */
	if (freq_hz == 0) {
		return 0;
	}

	*qspi_freq_div = (qspi_core_clock / freq_hz);

	return 0;
}

static const uint8_t qspi_signal_mode[4] = {0, 0x6U, 0x1U, 0x7U};

int soc_xec_qspi_cpol_cpha_from_freq(uint32_t freq_hz, uint8_t sig_mode, uint8_t *cpol_cpha)
{
	uint8_t val = 0;

	if ((cpol_cpha == NULL) || (sig_mode > 3U)) {
		return -EINVAL;
	}

	val = qspi_signal_mode[sig_mode];

	/* QSPI SPI frequency >= 48 MHz requires:
	 * Mode 0: CPOL=0, CPHA_SDI=0, CPHA_SDO=1
	 * Mode 3: CPOL=1, CPHA_SDI=1, CPHA_SDO=0
	 */
	if (freq_hz >= MHZ(48)) {
		val ^= BIT(2);
	}

	*cpol_cpha = val;

	return 0;
}
