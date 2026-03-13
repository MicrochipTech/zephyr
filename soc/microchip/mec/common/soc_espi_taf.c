/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/common/sys_io.h>
#include <soc.h>
#include "soc_espi_saf_v2.h"

/* QMSPI controller because a locked resource for eSPI TAF
 * When eSPI TAF is activated, hardware will hide QSPI registers causing
 * any access by the EC CPU to generate a bus error.
 * Before eSPI TAF is activated its driver calls code in this file
 * to configure QSPI for TAF operation.
 */
#define XEC_QSPI_REG_BASE DT_REG_ADDR(DT_NODELABEL(qspi0))

int soc_xec_taf_qmspi_load_descrs(const uint32_t *qmspi_descrs, uint8_t ndescrs)
{
	mm_reg_t qbase = (mm_reg_t)XEC_QSPI_REG_BASE + XEC_QSPI_DESCR_OFS(0);
	uint8_t idx_max = ndescrs;

	if (qmspi_descrs == NULL) {
		return -EINVAL;
	}

	if (idx_max > XEC_QSPI_MAX_DESCR_IDX) {
		idx_max = XEC_QSPI_MAX_DESCR_IDX;
	}

	for (uint8_t i = 0; i < idx_max; i++) {
		sys_write32(qmspi_descrs[i], qbase);
		qbase += 4U;
	}

	return 0;
}

int soc_xec_taf_qmspi_init(const uint32_t *qmspi_descrs, uint8_t ndescrs)
{
	mm_reg_t qbase = (mm_reg_t)XEC_QSPI_REG_BASE;
	uint32_t qmode = 0, qfdiv = 0, cstim = 0, n = 0;
#if 0
	struct qmspi_regs * const qregs = xcfg->qmspi_base;
	struct mchp_espi_saf * const regs = xcfg->saf_base;
	const struct espi_saf_hw_cfg *hwcfg = &cfg->hwcfg;
#endif
	qmode = qregs->MODE;
	if (!(qmode & MCHP_QMSPI_M_ACTIVATE)) {
		return -EAGAIN;
	}

	qmode = qregs->MODE & (MCHP_QMSPI_M_FDIV_MASK | MCHP_QMSPI_M_SIG_MASK);
	cstim = qregs->CSTM;
	qregs->MODE = MCHP_QMSPI_M_SRST;
	qregs->STS = MCHP_QMSPI_STS_RW1C_MASK;

	mchp_soc_ecia_girq_src_dis(XEC_QMSPI_GIRQ, XEC_QMSPI_GIRQ_POS);
	mchp_soc_ecia_girq_src_clr(XEC_QMSPI_GIRQ, XEC_QMSPI_GIRQ_POS);

	qregs->IFCTRL =
		(MCHP_QMSPI_IFC_WP_OUT_HI | MCHP_QMSPI_IFC_WP_OUT_EN |
		 MCHP_QMSPI_IFC_HOLD_OUT_HI | MCHP_QMSPI_IFC_HOLD_OUT_EN);

	for (n = 0; n < MCHP_SAF_NUM_GENERIC_DESCR; n++) {
		qregs->DESCR[MCHP_SAF_CM_EXIT_START_DESCR + n] =
			hwcfg->generic_descr[n];
	}

	/* SAF HW uses QMSPI interrupt signal */
	qregs->IEN = MCHP_QMSPI_IEN_XFR_DONE;

	qmode |= (MCHP_QMSPI_M_SAF_DMA_MODE_EN | MCHP_QMSPI_M_CS0 |
		  MCHP_QMSPI_M_ACTIVATE);

	if (hwcfg->flags & MCHP_SAF_HW_CFG_FLAG_CPHA) {
		qmode = (qmode & ~(MCHP_QMSPI_M_SIG_MASK)) |
			((hwcfg->qmspi_cpha << MCHP_QMSPI_M_SIG_POS) &
			 MCHP_QMSPI_M_SIG_MASK);
	}


	/* Copy QMSPI frequency divider into SAF CS0 and CS1 QMSPI frequency
	 * dividers. SAF HW uses CS0/CS1 divider register fields to overwrite
	 * QMSPI frequency divider in QMSPI.Mode register. Later we will update
	 * SAF CS0/CS1 SPI frequency dividers based on flash configuration.
	 */
	qfdiv = (qmode & MCHP_QMSPI_M_FDIV_MASK) >> MCHP_QMSPI_M_FDIV_POS;
	qfdiv = qfdiv | (qfdiv << 16); /* read and rest clock dividers */
	regs->SAF_CLKDIV_CS0 = qfdiv;
	regs->SAF_CLKDIV_CS1 = qfdiv;

	if (hwcfg->flags & MCHP_SAF_HW_CFG_FLAG_CSTM) {
		cstim = hwcfg->qmspi_cs_timing;
	}

	/* MEC172x SAF uses TX LDMA channel 0 in non-descriptor mode.
	 * SAF HW writes QMSPI.Control and TX LDMA channel 0 registers
	 * to transmit opcode, address, and data. We configure must
	 * configure TX LDMA channel 0 control register. We believe SAF
	 * HW will set bit[6] to 1.
	 */
	qregs->LDTX[0].CTRL = MCHP_QMSPI_LDC_EN | MCHP_QMSPI_LDC_RS_EN | MCHP_QMSPI_LDC_ASZ_4;

	qmode |= MCHP_QMSPI_M_LDMA_RX_EN | MCHP_QMSPI_M_LDMA_TX_EN;

	qregs->MODE = qmode;
	qregs->CSTM = cstim;

	return 0;
}
