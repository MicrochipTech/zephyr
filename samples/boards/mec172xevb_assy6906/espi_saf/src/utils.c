/*
 * Copyright (c) 2022 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

#include "utils.h"

void pr_byte_buffer(const uint8_t *buf, size_t bufsz)
{
	size_t n = 0;

	if (!buf) {
		return;
	}

	while (n < bufsz) {
		LOG_PRINTK("0x%02x, ", buf[n]);
		n++;
		if (!(n & 0x7)) {
			LOG_PRINTK("\n");
		}
	}
	LOG_PRINTK("\n");
}

void pr_espi_regs(void)
{
	struct espi_iom_regs * const iom =
		(struct espi_iom_regs *)DT_REG_ADDR(DT_NODELABEL(espi0));

	LOG_PRINTK("eSPI IOM registers\n");
	LOG_PRINTK("CAP0   @ 0x%x = 0x%x\n", (uint32_t)&iom->CAP0, iom->CAP0);
	LOG_PRINTK("CAP1   @ 0x%x = 0x%x\n", (uint32_t)&iom->CAP1, iom->CAP1);
	LOG_PRINTK("CAPPC  @ 0x%x = 0x%x\n", (uint32_t)&iom->CAPPC, iom->CAPPC);
	LOG_PRINTK("CAPVW  @ 0x%x = 0x%x\n", (uint32_t)&iom->CAPVW, iom->CAPVW);
	LOG_PRINTK("CAPOOB @ 0x%x = 0x%x\n", (uint32_t)&iom->CAPOOB, iom->CAPOOB);
	LOG_PRINTK("CAPFC  @ 0x%x = 0x%x\n", (uint32_t)&iom->CAPFC, iom->CAPFC);
	LOG_PRINTK("ERIS   @ 0x%x = 0x%x\n", (uint32_t)&iom->ERIS, iom->ERIS);
	LOG_PRINTK("ERIE   @ 0x%x = 0x%x\n", (uint32_t)&iom->ERIE, iom->ERIE);
	LOG_PRINTK("PLTSRC @ 0x%x = 0x%x\n", (uint32_t)&iom->PLTSRC, iom->PLTSRC);
	LOG_PRINTK("ACTV   @ 0x%x = 0x%x\n", (uint32_t)&iom->ACTV, iom->ACTV);
	LOG_PRINTK("SAFEBS @ 0x%x = 0x%x\n", (uint32_t)&iom->SAFEBS, iom->SAFEBS);
	LOG_PRINTK("RPMC_OP1_ODC    @ 0x%x = 0x%x\n",
		   (uint32_t)&iom->RPMC_OP1_ODC, iom->RPMC_OP1_ODC);
	LOG_PRINTK("RPMC_OP1_NC     @ 0x%x = 0x%x\n",
		   (uint32_t)&iom->RPMC_OP1_NC, iom->RPMC_OP1_NC);
	LOG_PRINTK("RPMC_OP1_NC_IMG @ 0x%x = 0x%x\n",
		   (uint32_t)&iom->RPMC_OP1_NC_IMG, iom->RPMC_OP1_NC_IMG);
}

void pr_espi_saf_regs(void)
{
	struct mchp_espi_saf * const saf =
		(struct mchp_espi_saf *)DT_REG_ADDR(DT_NODELABEL(espi_saf0));
	int i;

	LOG_PRINTK("eSPI SAF registers\n");
	LOG_PRINTK("SAF_ECP_STATUS @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_ECP_STATUS, saf->SAF_ECP_STATUS);
	LOG_PRINTK("SAF_ECP_INTEN  @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_ECP_INTEN, saf->SAF_ECP_INTEN);
	LOG_PRINTK("SAF_FL_CFG_SIZE_LIM @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_FL_CFG_SIZE_LIM, saf->SAF_FL_CFG_SIZE_LIM);
	LOG_PRINTK("SAF_FL_CFG_THRH @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_FL_CFG_THRH, saf->SAF_FL_CFG_THRH);
	LOG_PRINTK("SAF_FL_CFG_MISC @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_FL_CFG_MISC, saf->SAF_FL_CFG_MISC);
	LOG_PRINTK("SAF_ESPI_MON_STATUS @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_ESPI_MON_STATUS, saf->SAF_ESPI_MON_STATUS);
	LOG_PRINTK("SAF_ESPI_MON_INTEN @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_ESPI_MON_INTEN, saf->SAF_ESPI_MON_INTEN);
	LOG_PRINTK("SAF_ECP_BUSY @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_ECP_BUSY, saf->SAF_ECP_BUSY);
	for (i = 0; i < 2; i++) {
		LOG_PRINTK("SAF_CS_OPA[%d] @ 0x%x = 0x%x\n",
			   i, (uint32_t)&saf->SAF_CS_OP[i].OPA, saf->SAF_CS_OP[i].OPA);
		LOG_PRINTK("SAF_CS_OPB[%d] @ 0x%x = 0x%x\n",
			   i, (uint32_t)&saf->SAF_CS_OP[i].OPB, saf->SAF_CS_OP[i].OPB);
		LOG_PRINTK("SAF_CS_OPC[%d] @ 0x%x = 0x%x\n",
			   i, (uint32_t)&saf->SAF_CS_OP[i].OPC, saf->SAF_CS_OP[i].OPC);
		LOG_PRINTK("SAF_CS_OP_DESCR[%d] @ 0x%x = 0x%x\n",
			   i, (uint32_t)&saf->SAF_CS_OP[i].OP_DESCR, saf->SAF_CS_OP[i].OP_DESCR);
	}
	LOG_PRINTK("SAF_FL_CFG_GEN_DESCR @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_FL_CFG_GEN_DESCR, saf->SAF_FL_CFG_GEN_DESCR);
	LOG_PRINTK("SAF_PROT_LOCK @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_PROT_LOCK, saf->SAF_PROT_LOCK);
	LOG_PRINTK("SAF_PROT_DIRTY @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_PROT_DIRTY, saf->SAF_PROT_DIRTY);
	for (i = 0; i < 3; i++) {
		LOG_PRINTK("SAF_TAG_MAP[%d] @ 0x%x = 0x%x\n",
			   i, (uint32_t)&saf->SAF_TAG_MAP[i], saf->SAF_TAG_MAP[i]);
	}
	for (i = 0; i < 17; i++) {
		LOG_PRINTK("SAF_PROT_RG[%d].START @ 0x%x = 0x%x\n",
			   i, (uint32_t)&saf->SAF_PROT_RG[i].START, saf->SAF_PROT_RG[i].START);
		LOG_PRINTK("SAF_PROT_RG[%d].LIMIT @ 0x%x = 0x%x\n",
			   i, (uint32_t)&saf->SAF_PROT_RG[i].LIMIT, saf->SAF_PROT_RG[i].LIMIT);
		LOG_PRINTK("SAF_PROT_RG[%d].WEBM  @ 0x%x = 0x%x\n",
			   i, (uint32_t)&saf->SAF_PROT_RG[i].WEBM, saf->SAF_PROT_RG[i].WEBM);
		LOG_PRINTK("SAF_PROT_RG[%d].RDBM  @ 0x%x = 0x%x\n",
			   i, (uint32_t)&saf->SAF_PROT_RG[i].RDBM, saf->SAF_PROT_RG[i].RDBM);
	}
	LOG_PRINTK("SAF_POLL_TMOUT @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_POLL_TMOUT, saf->SAF_POLL_TMOUT);
	LOG_PRINTK("SAF_POLL_INTRVL @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_POLL_INTRVL, saf->SAF_POLL_INTRVL);
	LOG_PRINTK("SAF_SUS_RSM_INTRVL @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_SUS_RSM_INTRVL, saf->SAF_SUS_RSM_INTRVL);
	LOG_PRINTK("SAF_CONSEC_RD_TMOUT @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_CONSEC_RD_TMOUT, saf->SAF_CONSEC_RD_TMOUT);
	LOG_PRINTK("SAF_CS0_CFG_P2M @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_CS0_CFG_P2M, saf->SAF_CS0_CFG_P2M);
	LOG_PRINTK("SAF_CS1_CFG_P2M @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_CS1_CFG_P2M, saf->SAF_CS1_CFG_P2M);
	LOG_PRINTK("SAF_FL_CFG_SPM @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_FL_CFG_SPM, saf->SAF_FL_CFG_SPM);
	LOG_PRINTK("SAF_SUS_CHK_DLY @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_SUS_CHK_DLY, saf->SAF_SUS_CHK_DLY);
	LOG_PRINTK("SAF_CS0_CM_PRF @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_CS0_CM_PRF, saf->SAF_CS0_CM_PRF);
	LOG_PRINTK("SAF_CS1_CM_PRF @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_CS1_CM_PRF, saf->SAF_CS1_CM_PRF);
	LOG_PRINTK("SAF_DNX_PROT_BYP @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_DNX_PROT_BYP, saf->SAF_DNX_PROT_BYP);
	LOG_PRINTK("SAF_AC_RELOAD @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_AC_RELOAD, saf->SAF_AC_RELOAD);
	LOG_PRINTK("SAF_PWRDN_CTRL @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_PWRDN_CTRL, saf->SAF_PWRDN_CTRL);
	LOG_PRINTK("SAF_MEM_PWR_STS @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_MEM_PWR_STS, saf->SAF_MEM_PWR_STS);
	LOG_PRINTK("SAF_CFG_CS0_OPD @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_CFG_CS0_OPD, saf->SAF_CFG_CS0_OPD);
	LOG_PRINTK("SAF_CFG_CS1_OPD @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_CFG_CS1_OPD, saf->SAF_CFG_CS1_OPD);
	LOG_PRINTK("SAF_CLKDIV_CS0 @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_CLKDIV_CS0, saf->SAF_CLKDIV_CS0);
	LOG_PRINTK("SAF_CLKDIV_CS1 @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_CLKDIV_CS1, saf->SAF_CLKDIV_CS1);
	LOG_PRINTK("SAF_FL_PWR_TMOUT @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_FL_PWR_TMOUT, saf->SAF_FL_PWR_TMOUT);
	LOG_PRINTK("SAF_RPMC_OP2_ESPI_RES @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_RPMC_OP2_ESPI_RES, saf->SAF_RPMC_OP2_ESPI_RES);
	LOG_PRINTK("SAF_RPMC_OP2_EC0_RES @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_RPMC_OP2_EC0_RES, saf->SAF_RPMC_OP2_EC0_RES);
	LOG_PRINTK("SAF_RPMC_OP2_EC1_RES @ 0x%x = 0x%x\n",
		   (uint32_t)&saf->SAF_RPMC_OP2_EC1_RES, saf->SAF_RPMC_OP2_EC1_RES);
}
