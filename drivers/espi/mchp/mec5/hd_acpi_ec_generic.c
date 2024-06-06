/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_espi_acpi_ec_generic

#include <zephyr/kernel.h>
#include <soc.h>
#include <errno.h>
#include <zephyr/arch/common/ffs.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi/espi_mchp_mec5.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/espi/mchp-mec5-espi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include "espi_mchp_mec5_private.h"

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_retval.h>
#include <mec_pcr_api.h>
#include <mec_espi_api.h>
#include <mec_acpi_ec_api.h>

LOG_MODULE_REGISTER(espi_acpi_ec_generic, CONFIG_ESPI_LOG_LEVEL);

#define MEC_AEC_GEN_CFG_UD_POS		0
#define MEC_AEC_GEN_CFG_UD_MSK		0x3u
#define MEC_AEC_GEN_CFG_UD_NONE		0
#define MEC_AEC_GEN_CFG_UD0_HANDSHK	0x1
#define MEC_AEC_GEN_CFG_UD1_HANDSHK	0x2
#define MEC_AEC_GEN_CFG_4BYTE_MODE	BIT(7)

struct mec5_aec_generic_devcfg {
	struct mec_acpi_ec_regs *regs;
	const struct device *parent;
	uint32_t host_addr;
	uint8_t host_mem_space;
	uint8_t ldn;
	uint8_t sirq_obf;
	uint8_t cfg_flags;
	void (*irq_config_func)(void);
};

/* Two separate callbacks for IBF and OBE due to Host being far faster than EC */
struct mec5_aec_generic_data {
	uint32_t ibf_isr_count;
	uint32_t obe_isr_count;
	void (*callback)(const struct device *dev,
			 struct mchp_espi_acpi_ec_event *ev, void *ud);
	void *userdata;
	uint8_t hwstatus;
};

static void set_clr_user_defined(const struct device *dev, uint8_t set_bit)
{
	const struct mec5_aec_generic_devcfg *devcfg = dev->config;
	struct mec_acpi_ec_regs *regs = devcfg->regs;
	uint8_t udb = (devcfg->cfg_flags & MEC_AEC_GEN_CFG_UD_MSK) >> MEC_AEC_GEN_CFG_UD_POS;
	uint8_t msk = 0, val = 0;

	if (udb == MEC_AEC_GEN_CFG_UD0_HANDSHK) {
		msk = MEC_ACPI_EC_STS_UD0A;
		if (set_bit) {
			val = MEC_ACPI_EC_STS_UD0A;
		}
	} else if (udb == MEC_AEC_GEN_CFG_UD1_HANDSHK) {
		msk = MEC_ACPI_EC_STS_UD1A;
		if (set_bit) {
			val = MEC_ACPI_EC_STS_UD1A;
		}
	}

	if (msk) {
		mec_hal_acpi_ec_status_mask(regs, val, msk);
	}
}

static void mec5_aec_generic_ibf_isr(const struct device *dev)
{
	const struct mec5_aec_generic_devcfg *cfg = dev->config;
	struct mec_acpi_ec_regs *regs = cfg->regs;
	struct mec5_aec_generic_data *data = dev->data;
	uint32_t cmd_data = 0;
	struct mchp_espi_acpi_ec_event ev = { 0 };
	uint8_t status = mec_hal_acpi_ec_status(regs);

	data->ibf_isr_count++;
	data->hwstatus = status;

	/* 32-bit read of data register insures IBF is cleared
	 * for 1-byte or 4-byte mode.
	 */
	cmd_data = mec_hal_acpi_ec_host_to_ec_data_rd32(regs);
	mec_hal_acpi_ec_girq_clr(regs, MEC_ACPI_EC_IBF_IRQ);

	LOG_DBG("ISR: ACPI_EC at 0x%0x status = 0x%0x cmd_data = 0x%0x",
		(uint32_t)regs, status, cmd_data);

	ev.cmd_data = cmd_data;
	if (status & MEC_ACPI_EC_STS_CMD) {
		ev.cmd_data &= 0xffu;
		ev.ev_type = MCHP_ESPI_AEC_EV_CMD;
	} else { /* Host wrote to data */
		ev.ev_type = MCHP_ESPI_AEC_EV_DATA;
		if (mec_hal_acpi_ec_is_4byte_mode(regs)) {
			ev.flags |= MCHP_ESPI_AEC_EV_FLAG_4BYTE;
		} else {
			ev.cmd_data &= 0xffu;
		}
	}

	set_clr_user_defined(dev, 1);
	if (data->callback) {
		data->callback(dev, &ev, data->userdata);
	}
}

/* OBE goes active when Host reads last data byte.
 * Clear OBE status and enable.
 * Build event packet and pass it via OBE callback.
 */
static void mec5_aec_generic_obe_isr(const struct device *dev)
{
	const struct mec5_aec_generic_devcfg *cfg = dev->config;
	struct mec_acpi_ec_regs *regs = cfg->regs;
	struct mec5_aec_generic_data *data = dev->data;
	struct mchp_espi_acpi_ec_event ev = { 0 };

	data->obe_isr_count++;
	data->hwstatus = mec_hal_acpi_ec_status(regs);

	mec_hal_acpi_ec_girq_dis(regs, MEC_ACPI_EC_OBE_IRQ);
	mec_hal_acpi_ec_girq_clr(regs, MEC_ACPI_EC_OBE_IRQ);

	ev.ev_type = MCHP_ESPI_AEC_EV_HOBE;

	if (data->callback) {
		data->callback(dev, &ev, data->userdata);
	}
}

/* Called by eSPI parent driver when platform reset de-asserts.
 * ACPI_EC peripheral registers reset by "RESET_SYS"
 * RESET_SYS is active if any of the following activate:
 *  RESET_VTR: VTR power rail up/down
 *  nRESET_IN pin asserted
 *  Watch Dog Timer reset generated
 *  PCR System Reset register Soft-Sys reset set by firmware
 *  Cortex-M4 SYSRESETREQ signal active
 * ACPI_EC configuration in driver init should be stable across eSPI PLTRST#
 * and VCC_RESET.
 * eSPI BARs and SerialIRQ are reset by eSPI PLTRST# active. We must reprogram
 * these eSPI registers for this device.
 */
static int mec5_aec_generic_host_access_en(const struct device *dev, uint8_t enable, uint32_t cfg)
{
	const struct mec5_aec_generic_devcfg *devcfg = dev->config;
	uint32_t barcfg = devcfg->ldn | BIT(ESPI_MEC5_BAR_CFG_EN_POS);
	uint32_t sirqcfg = devcfg->ldn;
	int ret = 0;

	if (devcfg->host_mem_space) {
		barcfg |= BIT(ESPI_MEC5_BAR_CFG_MEM_BAR_POS);
	}

	ret = espi_mec5_bar_config(devcfg->parent, devcfg->host_addr, barcfg);
	if (ret) {
		return ret;
	}

	sirqcfg |= (((uint32_t)devcfg->sirq_obf << ESPI_MEC5_SIRQ_CFG_SLOT_POS)
		    & ESPI_MEC5_SIRQ_CFG_SLOT_MSK);
	ret = espi_mec5_sirq_config(devcfg->parent, sirqcfg);

	return ret;
}

/* ISSUE
 * Supporting SIRQ enable/disable with this API is only possible if we use the
 * static DT value for SIRQ slot. We need an 8-bit value for the
 * slot. You could encode the 8-bit value in upper bits of flags as we don't
 * expect more than 4 to 6 interrupts. Some blocks have two SIRQ's which means
 * we would need b[31:16] for SIRQ value leaving bits[15:0] for flags.
 * Is this a good idea?
 * Otherwise we must add a SIRQ specific API.
 */
static int mec5_aec_generic_intr_enable(const struct device *dev, uint8_t enable, uint32_t flags)
{
	const struct mec5_aec_generic_devcfg *devcfg = dev->config;
	struct mec_acpi_ec_regs *regs = devcfg->regs;
	uint32_t iflags = 0;
	int ret = 0;
	uint8_t slot = 0xffu;

	if (flags & MCHP_ESPI_PC_AEC_IEN_FLAG_SIRQ_OBE) {
		if (enable) {
			slot = devcfg->sirq_obf;
		}
		mec_hal_espi_ld_sirq_set(MEC_ESPI_IO, devcfg->ldn, 0, slot);
	}

	if (flags & MCHP_ESPI_PC_AEC_IEN_FLAG_IBF) {
		iflags |= MEC_ACPI_EC_IBF_IRQ;
	}
	if (flags & MCHP_ESPI_PC_AEC_IEN_FLAG_OBE) {
		iflags |= MEC_ACPI_EC_OBE_IRQ;
	}

	if (!iflags) {
		return 0;
	}

	if (enable) {
		ret = mec_hal_acpi_ec_girq_en(regs, iflags);
	} else {
		ret = mec_hal_acpi_ec_girq_dis(regs, iflags);
	}

	if (ret) {
		ret = -EIO;
	}

	return ret;
}

static int mec5_aec_generic_set_cb(const struct device *dev, mchp_espi_pc_aec_callback_t cb,
				   void *userdata)
{
	struct mec5_aec_generic_data *data = dev->data;
	unsigned lock = irq_lock();

	data->callback = cb;
	data->userdata = userdata;

	irq_unlock(lock);

	return 0;
}

#if 0
enum mchp_espi_pc_acpi_ec_op {
	MCHP_ESPI_AEC_OP_GET_RXDATA = 0,
	MCHP_ESPI_AEC_OP_SEND_DATA,
	MCHP_ESPI_AEC_OP_UD_SET, /* data(0) specifies UD0A or (1) UD1A */
	MCHP_ESPI_AEC_OP_UD_CLR,
	MCHP_ESPI_AEC_OP_GEN_SCI, /* data(1) set and generate SCI else clear SCI status */
	MCHP_ESPI_AEC_OP_GEN_SMI, /* data(1) set and generate SMI else clear SMI status */
};
#endif

/* ACPI_EC generic operations requested by the application.
 * Send data - transmit data from ACPI_EC to Host. Always write 32-bit data. If 4-byte data
 * mode is not enabled HW will ignore bytes lanes 1-3.
 * UD Set/clear - data == 0 set UD0A bit else set UD1A bit in ACPI_EC status register.
 *
 */
static int mec5_aec_generic_op(const struct device *dev, enum mchp_espi_pc_acpi_ec_op op,
			       uint32_t opdata, uint32_t *rxdata)
{
	const struct mec5_aec_generic_devcfg *devcfg = dev->config;
	struct mec_acpi_ec_regs *regs = devcfg->regs;
	uint32_t cmd_data = 0;
	uint8_t msk = 0, val = 0;

	switch (op) {
	case MCHP_ESPI_AEC_OP_GET_RXDATA:
		cmd_data = mec_hal_acpi_ec_host_to_ec_data_rd32(regs);

		break;
	case MCHP_ESPI_AEC_OP_SEND_DATA:
		mec_hal_acpi_ec_e2h_to_ec_data_wr32(regs, opdata);
		break;
	case MCHP_ESPI_AEC_OP_UD_SET:
		msk = MEC_ACPI_EC_STS_UD0A;
		if (opdata) {
			msk = MEC_ACPI_EC_STS_UD1A;
		}
		val = msk;
		mec_hal_acpi_ec_status_mask(regs, val, msk);
		break;
	case MCHP_ESPI_AEC_OP_UD_CLR:
		msk = MEC_ACPI_EC_STS_UD0A;
		if (opdata) {
			msk = MEC_ACPI_EC_STS_UD0A;
		}
		mec_hal_acpi_ec_status_mask(regs, val, msk);
		break;
	case MCHP_ESPI_AEC_OP_GEN_SCI:
		msk = MEC_ACPI_EC_STS_SCI;
		if (opdata) {
			val = msk;
		}
		mec_hal_acpi_ec_status_mask(regs, val, msk);
		val = (opdata) ? 1 : 0;
		espi_send_vwire(devcfg->parent, ESPI_VWIRE_SIGNAL_SCI, val);
		break;
	case MCHP_ESPI_AEC_OP_GEN_SMI:
		msk = MEC_ACPI_EC_STS_SMI;
		if (opdata) {
			val = msk;
		}
		mec_hal_acpi_ec_status_mask(regs, val, msk);
		val = (opdata) ? 1 : 0;
		espi_send_vwire(devcfg->parent, ESPI_VWIRE_SIGNAL_SMI, val);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* API
 * First API must be host_access_enable
 * configure - Possibly not needed.
 * lpc_request - eSPI parent driver calls this passing EACPI opcodes only
 * for the ACPI_EC instance obtained via DT chosen espi,os-acpi
 * TODO -
 * API using an param for action
 *  send data to host (8-bit or 32-bit)
 *  generate SCI flag - use eSPI HAL API to actually generate SCI
 *  generate SMI flag - use eSPI HAL API to actually generate SMI
 *  set/clear UD1A bit
 *  set/clear UD0A bit
 * API's to get/set configuration: 4-byte mode, ...
 */
static const struct mchp_espi_pc_aec_driver_api mec5_aec_generic_driver_api = {
	.host_access_enable = mec5_aec_generic_host_access_en,
	.intr_enable = mec5_aec_generic_intr_enable,
	.set_callback = mec5_aec_generic_set_cb,
	.operation = mec5_aec_generic_op,
};

/* Only enable IBF interrupt. OBE interrupt is fully enabled after EC has written
 * data to the EC-to-Host register.
 */
static int mec5_aec_generic_init(const struct device *dev)
{
	const struct mec5_aec_generic_devcfg *devcfg = dev->config;
	struct mec5_aec_generic_data *data = dev->data;
	struct mec_acpi_ec_regs *regs = devcfg->regs;
	uint32_t flags = MEC_ACPI_EC_RESET;
	int ret = 0;

	data->ibf_isr_count = 0;
	data->obe_isr_count = 0;
	data->hwstatus = 0;

	if (devcfg->cfg_flags & MEC_AEC_GEN_CFG_4BYTE_MODE) {
		flags |= MEC_ACPI_EC_4BYTE_MODE;
	}

	ret = mec_hal_acpi_ec_init(regs, flags);
	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	if (devcfg->irq_config_func) {
		devcfg->irq_config_func();
		mec_hal_acpi_ec_girq_en(regs, MEC_ACPI_EC_IBF_IRQ);
	}

	return 0;
}

#define MEC5_DT_AEC_GEN_NODE(inst) DT_INST(inst, DT_DRV_COMPAT)

#define MEC5_DT_AEC_GEN_HA(inst) \
	DT_PROP_BY_PHANDLE_IDX(MEC5_DT_AEC_GEN_NODE(inst), host_infos, 0, host_address)

#define MEC5_DT_AEC_GEN_HMS(inst) \
	DT_PROP_BY_PHANDLE_IDX_OR(MEC5_DT_AEC_GEN_NODE(inst), host_infos, 0, host_mem_space, 0)

#define MEC5_DT_AEC_GEN_LDN(inst) \
	DT_PROP_BY_PHANDLE_IDX(MEC5_DT_AEC_GEN_NODE(inst), host_infos, 0, ldn)

/* return node indentifier pointed to by index, idx in phandles host_infos */
#define MEC5_DT_AEC_GEN_HI_NODE(inst) \
	DT_PHANDLE_BY_IDX(MEC5_DT_AEC_GEN_NODE(inst), host_infos, 0)

#define MEC5_DT_AEC_GEN_OBF_SIRQ(inst) \
	DT_PROP_BY_IDX(MEC5_DT_AEC_GEN_HI_NODE(inst), sirqs, 0)

#define DT_MEC5_AEC_CFG_FLAGS(inst) \
	(uint8_t)((DT_INST_ENUM_IDX_OR(inst, user_data_ibf_handshake, 0) & 0x3u) |\
		  (MEC_AEC_GEN_CFG_4BYTE_MODE * DT_INST_PROP(inst, four_byte_data_mode)))

#define MEC5_AEC_GEN_DEVICE(inst)							\
											\
	static void mec5_aec_generic_irq_cfg_func_##inst(void);				\
											\
	static struct mec5_aec_generic_data mec5_aec_generic_data_##inst;		\
											\
	static const struct mec5_aec_generic_devcfg mec5_aec_generic_dcfg_##inst = {	\
		.regs = (struct mec_acpi_ec_regs *)DT_INST_REG_ADDR(inst),		\
		.parent = DEVICE_DT_GET(DT_INST_PARENT(inst)),				\
		.host_addr = MEC5_DT_AEC_GEN_HA(inst),					\
		.host_mem_space = MEC5_DT_AEC_GEN_HMS(inst),				\
		.ldn = MEC5_DT_AEC_GEN_LDN(inst),					\
		.sirq_obf = MEC5_DT_AEC_GEN_OBF_SIRQ(inst),				\
		.cfg_flags = DT_MEC5_AEC_CFG_FLAGS(inst),				\
		.irq_config_func = mec5_aec_generic_irq_cfg_func_##inst,		\
	};										\
	DEVICE_DT_INST_DEFINE(inst, mec5_aec_generic_init, NULL,			\
			&mec5_aec_generic_data_##inst,					\
			&mec5_aec_generic_dcfg_##inst,					\
			POST_KERNEL, CONFIG_ESPI_INIT_PRIORITY,				\
			&mec5_aec_generic_driver_api);					\
											\
	static void mec5_aec_generic_irq_cfg_func_##inst(void)				\
	{										\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, ibf, irq),			\
			    DT_INST_IRQ_BY_NAME(inst, ibf, priority),			\
			    mec5_aec_generic_ibf_isr,					\
			    DEVICE_DT_INST_GET(inst), 0);				\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, ibf, irq));			\
			    IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, obe, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, obe, priority),			\
			    mec5_aec_generic_obe_isr,					\
			    DEVICE_DT_INST_GET(inst), 0);				\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, obe, irq));			\
	}

DT_INST_FOREACH_STATUS_OKAY(MEC5_AEC_GEN_DEVICE)
