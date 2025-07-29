/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_espi

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi/espi_mchp_mec.h>
#include <zephyr/dt-bindings/espi/mchp_mec5_espi.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <soc.h>
#include <zephyr/irq.h>

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_ecia_api.h>
#include <mec_emi_api.h>
#include <mec_espi_api.h>

/* local */
#include "../espi_utils.h"
#include "../espi_mchp_mec5.h"
#include "espi_mchp_regs.h"
#include "espi_mchp_pcd_regs.h"

/* -------- Peripheral Channel -------- */

#define MEC_ESPI_PC_MAX_SIRQ_PER_DEV	2

#define PC_MBOX0_NODE DT_NODELABEL(mbox0)
#define PC_AEC0_NODE  DT_NODELABEL(acpi_ec0)
#define PC_AEC1_NODE  DT_NODELABEL(acpi_ec1)
#define PC_AEC2_NODE  DT_NODELABEL(acpi_ec2)
#define PC_AEC3_NODE  DT_NODELABEL(acpi_ec3)
#define PC_AEC4_NODE  DT_NODELABEL(acpi_ec4)
#define PC_GLUE_NODE  DT_NODELABEL(glue)
#define PC_EMI0_NODE  DT_NODELABEL(emi0)
#define PC_EMI1_NODE  DT_NODELABEL(emi1)
#define PC_EMI2_NODE  DT_NODELABEL(emi2)
#define PC_BDP0_NODE  DT_NODELABEL(p80bd0)

#define ESPI_MEC5_NID DT_NODELABEL(espi0)
#define ESPI_MEC5_PC_DEV_PATH DT_PATH(mchp_mec5_espi_pc_host_dev)
#define ESPI_MEC5_PC_SRAM_BAR_DT_PATH DT_PATH(mchp_mec5_espi_sram_bars)

/* Generate PC device table */
#define HAS_HOST_ADDR_PROP(node_id) DT_NODE_HAS_PROP(node_id, host_address)

struct espi_mec5_pc_hi {
	uint32_t haddr_lsw;
	uint8_t ldn;
	uint8_t iob_cfg_idx;
	uint8_t memb_cfg_idx;
	uint8_t nsirqs;
	uint8_t sirq_cfgs[MEC_ESPI_PC_MAX_SIRQ_PER_DEV];
	uint8_t sirq_vals[MEC_ESPI_PC_MAX_SIRQ_PER_DEV];
};

struct espi_mec_pc_sram_bar {
	uint32_t host_addr_lsw;
	uint8_t bar_id;
	uint8_t enc_sz;
	uint8_t access;
	uint8_t rsvd;
};

#define MEC5_PC_DEV_MAX_NIRQ 3

struct espi_mec5_pc_device {
	uint32_t regbase;
	const struct device *parent;
	uint8_t num_irqs;
	uint8_t rsvd1[3];
	uint8_t irqn[MEC5_PC_DEV_MAX_NIRQ];
	uint8_t irqp[MEC5_PC_DEV_MAX_NIRQ];
	uint8_t girqs[MEC5_PC_DEV_MAX_NIRQ * 2]; /* array of (girq num, bit pos) */
};

#define MEC5_PC_DEV_NUM_IRQS(nid) \
	COND_CODE_1(DT_NODE_HAS_PROP(nid, interrupt_names), \
		   (DT_PROP_LEN(nid, interrupt_names)), (1u))

#define MEC5_PC_DEV_IRQN_ELEM(nid, prop, idx) DT_IRQ_BY_IDX(nid, idx, irq),
#define MEC5_PC_DEV_IRQP_ELEM(nid, prop, idx) DT_IRQ_BY_IDX(nid, idx, priority),
#define MEC5_PC_DEV_GIRQ_ELEM(nid, prop, idx) DT_PROP_BY_IDX(nid, prop, idx),

#define MEC5_PC_DEV_IRQN(nid)			\
	{ \
		DT_FOREACH_PROP_ELEM(nid, interrupt_names, MEC5_PC_DEV_IRQN_ELEM) \
	}

#define MEC5_PC_DEV_IRQP(nid)			\
	{ \
		DT_FOREACH_PROP_ELEM(nid, interrupt_names, MEC5_PC_DEV_IRQP_ELEM) \
	}

#define MEC5_PC_DEV_GIRQS(nid) \
	{ \
		DT_FOREACH_PROP_ELEM(nid, girqs, MEC5_PC_DEV_GIRQ_ELEM) \
	}

#define MEC5_PC_DEV_WITH_HI(nid)					\
	{                                                                                          \
		.regbase = DT_REG_ADDR(nid), \
		.parent = DEVICE_DT_GET(DT_PARENT(nid)), \
		.num_irqs = MEC5_PC_DEV_NUM_IRQS(nid), \
		.irqn = MEC5_PC_DEV_IRQN(nid),	\
		.irqp = MEC5_PC_DEV_IRQP(nid), \
		.girqs = MEC5_PC_DEV_GIRQS(nid), \
	},

#define MEC5_PC_DEV(nid) COND_CODE_1(DT_NODE_HAS_PROP(nid, hostinfos), (MEC5_PC_DEV_WITH_HI(nid)), ())

#define MEC_ESPI_PC_NSIRQS_VAL(nid) DT_PROP_LEN(nid, sirq_config_indexes)

#define MEC_ESPI_PC_NSIRQS(nid) \
	COND_CODE_1(DT_NODE_HAS_PROP(nid, sirq_config_indexes), (MEC_ESPI_PC_NSIRQS_VAL(nid)), (0))

#define MEC5_PC_SIRQ_VAL(nid, prop, i) \
	COND_CODE_1(DT_PROP_HAS_IDX(nid, prop, i), (DT_PROP_BY_IDX(nid, prop, i)), (0))

#define MEC5_PC_SIRQ(n, prop, i) \
	COND_CODE_1(DT_NODE_HAS_PROP(n, prop), (MEC5_PC_SIRQ_VAL(n, prop, i)), (0))



// DT_PROP_HAS_IDX(node_id, prop, idx)

#define ESPI_MEC5_PC_DEV_BAR_SIRQ(node_id)                                                         \
	{                                                                                          \
		.haddr_lsw = DT_PROP(node_id, host_address),                                       \
		.ldn = DT_PROP(node_id, ldn),                                                      \
		.iob_cfg_idx = DT_PROP(node_id, host_io_config_index),                             \
		.memb_cfg_idx = DT_PROP(node_id, host_mem_config_index),                           \
		.nsirqs = MEC_ESPI_PC_NSIRQS(node_id), \
		.sirq_cfgs = { MEC5_PC_SIRQ(node_id, sirq_config_indexes, 0), \
				MEC5_PC_SIRQ(node_id, sirq_config_indexes, 1), \
		}, \
		.sirq_vals = { MEC5_PC_SIRQ(node_id, sirqs, 0), MEC5_PC_SIRQ(node_id, sirqs, 1) }, \
	},

#define MEC5_PC_SRAM_BAR(nid) \
	{						\
	.host_addr_lsw = DT_PROP(nid, host_addr_lsw),\
	.bar_id = DT_PROP(nid, id), \
	.enc_sz = DT_ENUM_IDX(nid, region_size), \
	.access = DT_ENUM_IDX(nid, access), \
	},

/* PC device BAR and Serial-IRQ table */
const struct espi_mec5_pc_hi espi_mec5_pc_hitbl[] = {
	DT_FOREACH_CHILD_STATUS_OKAY(ESPI_MEC5_PC_DEV_PATH, ESPI_MEC5_PC_DEV_BAR_SIRQ)};

const struct espi_mec_pc_sram_bar espi_mec5_pc_sram_bar_tbl[] = {
	DT_FOREACH_CHILD_STATUS_OKAY(ESPI_MEC5_PC_SRAM_BAR_DT_PATH, MEC5_PC_SRAM_BAR)
};

const struct espi_mec5_pc_device espi_mec5_pc_devtbl[] = {
	DT_FOREACH_CHILD_STATUS_OKAY(ESPI_MEC5_NID, MEC5_PC_DEV)
};

/* Configure BARs (base address registers) for Periph-Channel devices.
 * KBC-8042, fast-kb(port92h), UARTs, RTC, BDP, GL(glue-logic), Logical-Device configs only have
 * I/O BARs for x86 I/O instructions. All others: mailbox, ACPI_EC, and EMI have both I/O
 * and memory BARs.
 * BAR registers are reset when nPLTRST is active (low). We can only program BARs after
 * the Host drives nPLTRST inactive (high).
 */
static int mec5_pc_config_bars(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *devcfg = dev->config;
	mm_reg_t ioregb = devcfg->ioc_base;
	mm_reg_t memregb = devcfg->memc_base;
	uint32_t ofs = 0, val = 0;

	/* Extended Host memory BAR register: host address b[47:32] */
	sys_write32(devcfg->mem_bar_msw, memregb + MEC_ESPI_MC_MBAR_HA_EXT);

	for (size_t n = 0; n < ARRAY_SIZE(espi_mec5_pc_hitbl); n++) {
		const struct espi_mec5_pc_hi *p = &espi_mec5_pc_hitbl[n];

		if (p->haddr_lsw > UINT16_MAX) { /* Program logical device memory bar */
			LOG_DBG("LDN[%u] MBAR @ 0x%02x = 0x%0x", p->ldn, p->memb_cfg_idx,
				p->haddr_lsw);

			ofs = MEC_ESPI_MEMB_HOST_OFS(p->iob_cfg_idx);
			val = MEC_ESPI_IOB_HOST_HADDR_SET(p->haddr_lsw);

			sys_write16(0, memregb + ofs); /* disable */
			sys_write16(val, memregb + ofs + 2u); /* host addr bits[15:0] */
			sys_write16((val >> 16), memregb + ofs + 4u); /* host addr bits[31:16] */
			sys_write16(BIT(MEC_ESPI_MC_BAR_CFG_VAL_POS), memregb + ofs); /* enable */

		} else { /* program logical device I/O bar */
			LOG_DBG("LDN[%u] IOBAR @ 0x%02x = 0x%0x", p->ldn, p->iob_cfg_idx,
				p->haddr_lsw);

			ofs = MEC_ESPI_IOB_HOST_OFS(p->iob_cfg_idx);
			sys_write32(0, ioregb + ofs); /* clear and disable */
			val = MEC_ESPI_IOB_HOST_HADDR_SET(p->haddr_lsw);
			val |= BIT(MEC_ESPI_IOB_HOST_VALID_POS);
			sys_write32(val, ioregb + ofs);
		}
	}

	return 0;
}

/* Many Periph-channel devices can generate interrupts to the Host.
 * These interrupts are Serial-IRQs and are delivered over dedicated
 * eSPI virtual wires. The register configuring the Serial-IRQ number
 * for each Periph-channel device are held in reset when nESPI_RESET or
 * nPLTRST are active. We must program these registers when the Host
 * de-asserts nPLTRST.
 */
static int mec5_pc_config_sirqs(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iobase = (mm_reg_t)drvcfg->ioc_base;
	uint32_t ofs = 0;

	for (size_t n = 0; n < ARRAY_SIZE(espi_mec5_pc_hitbl); n++) {
		const struct espi_mec5_pc_hi *p = &espi_mec5_pc_hitbl[n];

		for (uint8_t m = 0; m < p->nsirqs; m++) {
			LOG_DBG("LDN[%u] SIRQ[%u] idx=0x%02x val=0x%02x", p->ldn, m,
				p->sirq_cfgs[m], p->sirq_vals[m]);

			ofs = MEC_ESPI_SIRQ_OFS_FROM_HCFG(p->sirq_cfgs[m]);
			sys_write8(p->sirq_vals[m], iobase + ofs);
		}
	}

	return 0;
}

/* Host address register fields of the two SRAM BARs held in reset state by
 * nESPI_RESET, nPLTRST, or VCC_PWRGD.
 * When nPLTRST de-asserts we can program the SRAM BARs' host address fields.
 * Other fields: EC SRAM base address, size, access and valid are not affected
 * by nPLTRST or VCC_PWRGD.
 */
static int mec5_pc_config_sram_bars(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t mbase = (mm_reg_t)drvcfg->memc_base;
	uint32_t ofs = 0, ofs_cfg = 0, temp = 0;

	sys_write32(drvcfg->sram_bar_msw, mbase + MEC_ESPI_MC_SBAR_HA_EXT);

	for (size_t n = 0; n < ARRAY_SIZE(espi_mec5_pc_sram_bar_tbl); n++) {
		const struct espi_mec_pc_sram_bar *pb = &espi_mec5_pc_sram_bar_tbl[n];

		ofs = MEC_ESPI_MC_SBAR0;
		ofs_cfg = MEC_ESPI_MC_SBAR0_CFG;
		if (pb->bar_id != 0) {
			ofs = MEC_ESPI_MC_SBAR1_CFG;
			ofs_cfg = MEC_ESPI_MC_SBAR1_CFG;
		}

		soc_mmcr_clear_bit16(mbase + ofs, 0); /* disable */

		temp = pb->host_addr_lsw;
		sys_write16(temp, mbase + ofs_cfg + 2u);
		sys_write16((temp >> 16), mbase + ofs_cfg + 4u);

		soc_mmcr_set_bit16(mbase + ofs, 0);
	}

	return 0;
}

#if DT_HAS_COMPAT_STATUS_OKAY(microchip_mec5_espi_pc_kbc) != 0

#define PC_KBC0_NODE  DT_NODELABEL(kbc0)

static void mec5_pc_pltrst_kbc(const struct device *dev)
{
	mm_reg_t kbc0_base = (mm_reg_t)DT_REG_ADDR_BY_NAME(PC_KBC0_NODE, kbc);
	mm_reg_t fkbc0_base = (mm_reg_t)DT_REG_ADDR_BY_NAME(PC_KBC0_NODE, fkbc);

	sys_set_bit8(kbc0_base + LD_CFG_ACTV_REG_OFS, LD_CFG_ACTV_POS);
	sys_set_bit8(fkbc0_base + LD_CFG_ACTV_REG_OFS, LD_CFG_ACTV_POS);
}
#else
static void mec5_pc_pltrst_kbc(const struct device *dev) {}
#endif

/* BIOS Debug I/O port capture (BDP) */
#if DT_HAS_COMPAT_STATUS_OKAY(microchip_mec5_espi_pc_bdp) != 0

#endif

#if DT_HAS_COMPAT_STATUS_OKAY(microchip_mec5_espi_pc_gl) != 0

static void mec5_pc_pltrst_gl(const struct device *dev)
{
#if DT_NODE_HAS_PROP(PC_GLUE_NODE, ec_enable_s0ix_detect)
	mm_reg_t glbase = (mm_reg_t)DT_REG_ADDR(PC_GLUE_NODE);

	sys_set_bit8(glbase + MEC_PC_GL_S0IX_DET_REG_OFS, MEC_PC_GL_S0IX_DET_EN_POS);
#endif
}
#else
static void mec5_pc_pltrst_gl(const struct device *dev) {}
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(microchip_mec5_espi_pc_uart) != 0
/* One or more UARTs are mapped to Host I/O space. Host OS driver owns them.
 * TODO - DT_COMPAT_GET_ANY_STATUS_OKAY(compat)
 */
#define MEC5_PC_UART_CFG_SEL(nid) \
	(((uint8_t)DT_PROP_OR(nid, ext_clk_sel, 0) & 0x01u) |		\
	 (((uint8_t)DT_PROP_OR(nid, invert_polarity, 0) & 0x01u) << 1u))

#define MEC5_PC_PLTRST_UART(nid)                                          \
	mm_reg_t regbase = (mm_reg_t)DT_REG_ADDR(nid);                    \
	uint8_t cv = sys_read8(regbase + LD_CFG_SEL_REG_OFS) & BIT(1);    \
	cv |= MEC5_PC_UART_CFG_SEL(nid);                                  \
	sys_write8(cv, regbase + LD_CFG_SEL_REG_OFS);                     \
	sys_set_bit8(regbase + LD_CFG_ACTV_REG_OFS, LD_CFG_ACTV_REG_OFS);

static void mec5_pc_pltrst_uart(const struct device *dev)
{
	DT_FOREACH_STATUS_OKAY(microchip_mec5_espi_pc_uart, MEC5_PC_PLTRST_UART)
}
#else
static void mec5_pc_pltrst_uart(const struct device *dev) {}
#endif

/* PC devices with configuration reset by nPLTRST must be reconfigured on de-assertion */
static void mec5_pc_pltrst_misc_config(const struct device *dev)
{
	mec5_pc_pltrst_kbc(dev);
	mec5_pc_pltrst_gl(dev);
	mec5_pc_pltrst_uart(dev);
}

/* ISR for Peripheral Channel events:
 * Channel enable change by Host
 * Bus Master enable change by Host
 * PC cycle errors
 */
static void espi_mec5_pc_isr(const struct device *dev)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t iob = drvcfg->ioc_base;
	struct espi_event evt = { .evt_type = ESPI_BUS_EVENT_CHANNEL_READY,
				  .evt_details = ESPI_CHANNEL_PERIPHERAL,
				  .evt_data = 0 };
	uint32_t status = sys_read32(iob + MEC_ESPI_PC_SR_OFS);

	sys_write32(status, iob + MEC_ESPI_PC_SR_OFS);
	data->pc_status = status;

	if (status & BIT(MEC_ESPI_PC_CHEN_CHG_POS)) {
		if (status & BIT(MEC_ESPI_PC_CHEN_STATE_POS)) {
			evt.evt_data = 1u;
		}
		espi_send_callbacks(&data->callbacks, dev, evt);
	}

	if (status & BIT(MEC_ESPI_PC_BMEN_CHG_POS)) {
		evt.evt_details = ESPI_PC_EVT_BUS_MASTER_ENABLE;
		evt.evt_data = 0;
		if (status & BIT(MEC_ESPI_PC_BMEN_STATE_POS)) {
			evt.evt_data = 1u;
		}
		espi_send_callbacks(&data->callbacks, dev, evt);
	}

	if (status & BIT(MEC_ESPI_PC_ABERR_POS)) {
		/* pack bits[47:16] of address into bits[31:16] of evt_details.
	         * bits[31:0] of address in evt_data.
		 */
		evt.evt_details = sys_read32(iob + MEC_ESPI_PC_ERA_MSW_OFS);
		evt.evt_data = sys_read32(iob + MEC_ESPI_PC_ERA_LSW_OFS);
		evt.evt_details <<= 16;
		evt.evt_details |= ESPI_PC_EVT_BUS_INTERNAL_BERR;
		espi_send_callbacks(&data->callbacks, dev, evt);
	}
}

#define PC_IER_ALL                                                                                 \
	(BIT(ESPI_PC_IER_ABERR_POS) | BIT(ESPI_PC_IER_CHEN_CHG_POS) | BIT(ESPI_PC_IER_BMEN_CHG_POS))

void espi_mec5_pc_erst_config(const struct device *dev, uint8_t n_erst_state)
{
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	mm_reg_t iob = drvcfg->ioc_base;

	if (n_erst_state != 0) {
		sys_write32(MEC_ESPI_PC_IER_ALL_MSK, iob + MEC_ESPI_PC_IER_OFS);
	}
}

int espi_mec5_pc_pltrst_handler(const struct device *dev, uint8_t pltrst_state)
{
	int rc = 0;

	if (pltrst_state) {
		if (mec5_pc_config_bars(dev) != 0) {
			rc = -EIO;
		}

		if (mec5_pc_config_sirqs(dev) != 0) {
			rc = -EIO;
		}

		if (mec5_pc_config_sram_bars(dev) != 0) {
			rc = -EIO;
		}

		mec5_pc_pltrst_misc_config(dev);
	}

	return rc;
}

#ifndef CONFIG_ESPI_PERIPHERAL_ZEPHYR_MBOX

#if DT_HAS_COMPAT_STATUS_OKAY(microchip_mec5_espi_pc_mbox)
/* ISSUE:
 * MBOX generates an interrupt when the Host writes a byte to the Host-to-EC
 * mailbox register. MBOX has 32 more 8-bit registers which do not generate
 * any status to the EC if the Host accesses them.
 * If the Host writes one or more of the 32 generic mailbox registers and then
 * writes to Host-to-EC we get an interrupt but do not know how many of the
 * generic 8-bit registers were changed.
 * TODO - Investigate using Zephyr mbox driver for our MBOX HW. Zephry mbox
 * driver has concept of channels that we can map to the 32 8-bit HW mailboxes.
 * If we can use Zephyr mbox driver then we only need to handle MBOX BAR configuration
 * on nPLTRST de-assertion.
 */
static void espi_mec_pc_mbox_isr(const struct device *dev)
{
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t iob = (mm_reg_t)DT_REG_ADDR(PC_MBOX0_NODE);
	struct espi_event mbev = {
		.evt_type = ESPI_BUS_PERIPHERAL_NOTIFICATION,
		.evt_details = ESPI_PC_MCHP_MEC_MBOX0,
		.evt_data = 0,
	};
	uint8_t h2ec_cmd = sys_read8(iob + MEC_MBOX_H2EC_REG_OFS); /* clears interrupt signal */

	mbev.evt_data = h2ec_cmd;
	espi_send_callbacks(&data->callbacks, dev, mbev);
}

void mec5_pc_mbox_irq_connect(const struct device *dev)
{
	IRQ_CONNECT(DT_IRQ(PC_MBOX0_NODE, irq), DT_IRQ(PC_MBOX0_NODE, priority),
		    espi_mec_pc_mbox_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_IRQ(PC_MBOX0_NODE, irq));

	mec_hal_girq_bm_en(MEC_MBOX0_GIRQ, BIT(MEC_MBOX0_GIRQ_POS), 1u);
}
#else
void mec5_pc_mbox_irq_connect(const struct device *dev) {}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(microchip_mec5_espi_pc_mbox) */
#endif /* !CONFIG_ESPI_PERIPHERAL_ZEPHYR_MBOX */

#if DT_HAS_COMPAT_STATUS_OKAY(microchip_mec5_espi_pc_kbc)

static void espi_mec_pc_kbc0_isr(const struct device *dev)
{
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t iob = (mm_reg_t)DT_REG_ADDR(PC_KBC0_NODE);
	struct espi_event kbev = {
		.evt_type = ESPI_BUS_PERIPHERAL_NOTIFICATION,
		.evt_details = ESPI_PERIPHERAL_8042_KBC,
		.evt_data = ESPI_PERIPHERAL_NODATA,
	};
	struct espi_evt_data_kbc *kbc_evt = (struct espi_evt_data_kbc *)&kbev.evt_data;
	uint32_t girq_sts = 0;
	uint8_t status = 0, cd = 0;

	girq_sts = mec_hal_girq_result_get(MEC_KBC0_GIRQ);
	status = sys_read8(iob + MEC_KBC_SR_OFS);
	if ((status & BIT(MEC_KBC_SR_IBF_POS)) != 0) {
		cd = sys_read8(iob + MEC_KBC_H2EC_DCR_OFS); /* clears IBF */
		kbc_evt->type = HOST_KBC_TYPE_DATA;
		if ((status & BIT(MEC_KBC_SR_CMD_POS)) != 0) {
			kbc_evt->type = HOST_KBC_TYPE_CMD;
		}
		kbc_evt->data = cd;
		kbc_evt->evt = HOST_KBC_EVT_IBF;
		mec_hal_girq_bm_clr_src(MEC_KBC0_GIRQ, BIT(MEC_KBC0_IBF_GIRQ_POS));
	} else if ((girq_sts & BIT(MEC_KBC0_OBE_GIRQ_POS)) != 0) {
		kbc_evt->evt = HOST_KBC_EVT_OBE;
		mec_hal_girq_bm_en(MEC_KBC0_GIRQ, BIT(MEC_KBC0_OBE_GIRQ_POS), 0);
		mec_hal_girq_bm_clr_src(MEC_KBC0_GIRQ, BIT(MEC_KBC0_OBE_GIRQ_POS));
	}

	espi_send_callbacks(&data->callbacks, dev, kbev);
}

void mec5_pc_kbc_irq_connect(const struct device *dev)
{
	/* NOTE: OK to register same function in two different IRQ_CONNECTS */
	IRQ_CONNECT(DT_IRQ_BY_NAME(PC_KBC0_NODE, ibf, irq),
		    DT_IRQ_BY_NAME(PC_KBC0_NODE, ibf, priority),
		    espi_mec_pc_kbc0_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_IRQ(PC_KBC0_NODE, irq));

	IRQ_CONNECT(DT_IRQ_BY_NAME(PC_KBC0_NODE, obe, irq),
		    DT_IRQ_BY_NAME(PC_KBC0_NODE, obe, priority),
		    espi_mec_pc_kbc0_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_IRQ(PC_KBC0_NODE, irq));

	/* Only enable IBF. Application will call an eSPI driver API to enable OBE */
	mec_hal_girq_bm_en(MEC_KBC0_GIRQ, BIT(MEC_KBC0_IBF_GIRQ_POS), 1u);
}
#else
void mec5_pc_kbc_irq_connect(const struct device *dev) {}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(microchip_mec5_espi_pc_kbc) */

/* One ore more ACPI_EC devices enabled and use this compatible */
#if DT_HAS_COMPAT_STATUS_OKAY(microchip_mec5_espi_pc_acpi_ec)

struct mec5_espi_pc_device {
	uintptr_t pc_dev_reg_base;
	const struct device *parent;
	uint8_t girq;
	uint8_t girq_pos;
};

/* common ISR handler and read/write routines */

#endif /* #ifndef CONFIG_ESPI_PERIPHERAL_ZEPHYR_MBOX */

#define MEC5_PC_MBOX_ICONN_DEF(nid) \
	static espi_mec_pc_mbox_##?##_isr(const struct device *dev);

#define MEC5_PC_MBOX_ICONN(nid) \
	IRQ_CONNECT(DT_IRQ(nid, irq), DT_IRQ(nid, priority), \
		    espi_mec_pc_mbox_##?##_isr, DEVICE_DT_INST_GET(0), 0); \
	irq_enable(DT_IRQ(nid, irq)); \
	mec_hal_girq_bm_en(DT_PROP_BY_IDX(nid, girqs, 0), \
			   BIT(DT_PROP_BY_IDX(nid, girqs, 1)), 1u);

/* SRAM BAR 0 & 1: EC SRAM address, size, access and valid are on RESET_SYS
 * These fields can be set before ESPI_nRESET is de-asserted. Also not affected
 * by VCC_PWRGD.
 */
static int mec5_sram_bar_config(const struct device *dev, const struct mchp_mec_sram_cfg *psram,
				uint8_t num_sram_cfgs)
{
#if 0 /* TODO */
	const struct espi_mec5_drv_cfg *drvcfg = dev->config;
	struct espi_mec5_drv_data *data = dev->data;
	mm_reg_t memregb = drvcfg->memc_base;
	int rc1 = 0, rc2 = 0;
	uint8_t n = MIN(num_sram_cfgs, MCHP_MEC_NUM_SRAM_REGIONS);

	if (psram == 0) {
		if (n == 0) {
			return 0; /* nothing to do */
		}
		return -EINVAL;
	}

	while (n) {
		rc1 = mec_hal_espi_sram_bar_ec_mem_cfg(memregb, psram->sram_id, psram->mem_base,
						       psram->mem_size, psram->mem_access, 1u);
		if (rc1 == MEC_RET_OK) {
			data->sram_mem_base[psram->sram_id] = psram->mem_base;
		} else {
			rc2 = -EINVAL;
		}
		psram++;
		n--;
	}

	return rc2;
#endif /* 0 */
	return 0;
}

static int mec5_emi_mem_config(const struct device *dev, struct mchp_mec_emi_mem_cfg *pemi,
			       uint8_t num_emi_mem_cfgs)
{
#if 0 /* TODO */
	uintptr_t emi_reg_base = 0; /* TODO */
	int rc1 = 0, rc2 = 0;
	uint32_t rwsz = 0;
	uint8_t n = MIN(num_emi_mem_cfgs, (MCHP_MEC_NUM_EMIS * MCHP_MEC_NUM_EMI_MEM_REGIONS));

	if (pemi == 0) {
		if (n == 0) {
			return 0; /* nothing to do */
		}
		return -EINVAL;
	}

	while (n) {
		emi_reg_base = mec_hal_emi_get_base(pemi->emi_id);
		rwsz = MEC_EMI_MEMR_CFG_SIZES(pemi->mem_rd_sz, pemi->mem_wr_sz);

		rc1 = mec_hal_emi_mem_region_config(emi_reg_base, pemi->mem_id, pemi->mem_base, rwsz);
		if (rc1 != MEC_RET_OK) {
			rc2 = -EINVAL;
		}

		pemi++;
		n--;
	}
#endif
	return 0;
}

/* ---- Private API ---- */
void espi_mec5_pc_irq_connect(const struct device *dev)
{
	uint32_t pc_ien_msk = BIT(MEC_ESPI_GIRQ_PC_POS);

	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, pc, irq), DT_INST_IRQ_BY_NAME(0, pc, priority),
		    espi_mec5_pc_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, pc, irq));

	sys_write32(pc_ien_msk, MEC_ESPI_GIRQ_ENSET_ADDR);

	// DT_NUM_INST_STATUS_OKAY(compat)

	// DT_FOREACH_STATUS_OKAY(microchip_mec5_espi_pc_mbox, MEC5_PC_MBOX_ICONN)

	for (size_t n = 0; n < ARRAY_SIZE(espi_mec5_pc_devtbl); n++) {
		const struct espi_mec5_pc_device *pd = &espi_mec5_pc_devtbl[n];

		LOG_INF("PC dev[%u]: regbase=0x%0x parent=0x%0x",
			n, pd->regbase, (uint32_t)pd->parent);
	}

}

/* Configure PC channel and sub-devices before eSPI is enabled and nESPI_RESET is released.
 * Only certain components can be configured at this time.
 * SRAM BARs except for host address. Application provides buffers in SRAM.
 * I/O BARs except for ?
 * MEM BARs except for ?
 * EMI memory regions. Application provides buffers in SRAM.
 */
int espi_mec5_pc_config(const struct device *dev, const void *vend_ext)
{
	int rc1 = 0, rc2 = 0;

	if (vend_ext == NULL) {
		return 0; /* nothing to do */
	}

	const struct espi_cfg_mchp_mec *pcfg = (struct espi_cfg_mchp_mec *)vend_ext;

	rc1 = mec5_sram_bar_config(dev, pcfg->sram_cfgs, pcfg->num_sram_cfgs);
	rc2 = rc1;

	rc1 = mec5_emi_mem_config(dev, pcfg->emi_mem_cfgs, pcfg->num_emi_mem_cfgs);
	if (rc1 != 0) {
		rc2 = rc1;
	}

	return rc2;
}

/* ---- Driver public API ---- */
int espi_mec5_read_req_api(const struct device *dev, struct espi_request_packet *req)
{
	return -ENOTSUP;
}

int espi_mec5_write_req_api(const struct device *dev, struct espi_request_packet *req)
{
	return -ENOTSUP;
}

int espi_mec5_read_lpc_req_api(const struct device *dev, enum lpc_peripheral_opcode op,
			       uint32_t *data)
{
	return -ENOTSUP;
}

int espi_mec5_write_lpc_req_api(const struct device *dev, enum lpc_peripheral_opcode op,
				uint32_t *data)
{
	return -ENOTSUP;
}

/* TODO - Add new eSPI driver API to enable/disable PC devices interrupts */
