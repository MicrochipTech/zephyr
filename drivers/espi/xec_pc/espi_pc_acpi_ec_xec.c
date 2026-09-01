/*
 * Copyright (c) 2021 Microchip Technology Inc.
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Microchip XEC eSPI peripheral channel ACPI Embedded Controller devices.
 *
 * MEC hardware has five identical ACPI EC blocks. The V2 eSPI driver carries
 * five hand written copies of the same code in espi_mchp_xec_host_v2.c, one per
 * block, each selected by its own CONFIG_ESPI_PERIPHERAL_* symbol and each
 * hard coding the enum espi_virtual_peripheral value it reports. This driver
 * replaces all five with one multi instance driver: the block role moves to the
 * peripheral-type device tree property, so a board describes what each ACPI EC
 * is for instead of the build configuration deciding it.
 *
 * Two of the roles also answer an opcode range of the generic
 * espi_read_lpc_request() and espi_write_lpc_request() API on behalf of the
 * controller: the "host-io" instance serves EACPI_*, and the "ec-host-cmd"
 * instance serves the EC host command opcodes of the ECUSTOM_* range that the
 * subsys/mgmt/ec_host_cmd eSPI backend calls. V2 answers both from the eSPI
 * driver itself, wired to whichever ACPI EC block its Kconfig symbols named.
 *
 * The driver owns the ACPI EC block registers and its input buffer full (IBF)
 * and output buffer empty (OBE) interrupts. It does not program its Host I/O
 * BAR, its Host memory BAR or its OBF Serial IRQ: those are described on this
 * node as io-bars, mem-bars and sirqs and are programmed by the eSPI
 * controller, which then notifies this driver that the registers are live.
 */

#define DT_DRV_COMPAT microchip_xec_espi_pc_acpi_ec

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi/mchp_xec_espi.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "../espi_mchp_xec_v3.h"
#include "espi_pc_xec.h"

LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

/* Indices into the peripheral-type enum of the binding. Two of the roles answer
 * an opcode range of the generic espi_read_lpc_request() and
 * espi_write_lpc_request() API, so at most one instance may claim either:
 * "host-io" serves the EACPI_* range, and "ec-host-cmd" serves the EC host
 * command opcodes of the ECUSTOM_* range.
 */
#define XEC_ACPI_EC_ROLE_HOST_IO     0
#define XEC_ACPI_EC_ROLE_EC_HOST_CMD 4

#define XEC_ACPI_EC_COUNT_HOST_IO(n)                                                               \
	(DT_INST_ENUM_IDX(n, peripheral_type) == XEC_ACPI_EC_ROLE_HOST_IO) +

#define XEC_ACPI_EC_COUNT_EC_HOST_CMD(n)                                                           \
	(DT_INST_ENUM_IDX(n, peripheral_type) == XEC_ACPI_EC_ROLE_EC_HOST_CMD) +

BUILD_ASSERT((DT_INST_FOREACH_STATUS_OKAY(XEC_ACPI_EC_COUNT_HOST_IO) 0) <= 1,
	     "At most one XEC eSPI ACPI EC may have peripheral-type = \"host-io\"");

BUILD_ASSERT((DT_INST_FOREACH_STATUS_OKAY(XEC_ACPI_EC_COUNT_EC_HOST_CMD) 0) <= 1,
	     "At most one XEC eSPI ACPI EC may have peripheral-type = \"ec-host-cmd\"");

/* Serving the EC host command role takes two independent Kconfig symbols: the
 * one that declares the enum espi_virtual_peripheral value the driver reports,
 * and the one that declares the opcodes themselves. The eSPI backend in
 * subsys/mgmt/ec_host_cmd depends on both, so there is no useful half of this.
 */
#if defined(CONFIG_ESPI_PERIPHERAL_EC_HOST_CMD) && defined(CONFIG_ESPI_PERIPHERAL_CUSTOM_OPCODE)
#define XEC_ACPI_EC_HOST_CMD 1
#endif

#ifndef XEC_ACPI_EC_HOST_CMD
BUILD_ASSERT((DT_INST_FOREACH_STATUS_OKAY(XEC_ACPI_EC_COUNT_EC_HOST_CMD) 0) == 0,
	     "peripheral-type = \"ec-host-cmd\" needs CONFIG_ESPI_PERIPHERAL_EC_HOST_CMD "
	     "and CONFIG_ESPI_PERIPHERAL_CUSTOM_OPCODE");
#else
/* The three EC host command opcodes are claimed as one contiguous range, which
 * deliberately starts above the deprecated ECUSTOM_HOST_SUBS_INTERRUPT_EN the
 * controller answers itself. Fail the build rather than silently swallow that
 * opcode, or miss one of the three, if the enum is ever reordered.
 */
BUILD_ASSERT((ECUSTOM_HOST_CMD_GET_PARAM_MEMORY > ECUSTOM_HOST_SUBS_INTERRUPT_EN) &&
		     (ECUSTOM_HOST_CMD_GET_PARAM_MEMORY_SIZE ==
		      (ECUSTOM_HOST_CMD_GET_PARAM_MEMORY + 1)) &&
		     (ECUSTOM_HOST_CMD_SEND_RESULT == (ECUSTOM_HOST_CMD_GET_PARAM_MEMORY + 2)),
	     "EC host command opcodes are no longer a contiguous range above 0xf0");
#endif

struct espi_pc_acpi_ec_xec_config {
	struct acpi_ec_regs *regs;
	const struct device *espi_dev;
	void (*irq_connect)(void);
	uint8_t *shm;
	uint32_t ibf_ecia_info;
	uint32_t obe_ecia_info;
	uint16_t shm_size;
	uint16_t evt_details;
	bool serves_eacpi;
	bool serves_host_cmd;
};

struct espi_pc_acpi_ec_xec_data {
	struct mchp_xec_espi_pc_cb pc_cb;
	/* Whether the application wants Host facing interrupts from this block.
	 * Enabled until espi_interrupt_config() says otherwise, so an
	 * application that never calls it behaves as it did under V2.
	 */
	bool intr_en;
};

/* Deliver an ESPI_BUS_PERIPHERAL_NOTIFICATION carrying an ACPI event. The
 * private channel event data format, struct espi_evt_data_pvt, has the same
 * layout and the same type constants as struct espi_evt_data_acpi, so one
 * encoder serves every peripheral-type value.
 */
static void acpi_ec_notify(const struct device *dev, uint8_t type, uint8_t byte, bool with_data)
{
	const struct espi_pc_acpi_ec_xec_config *cfg = dev->config;
	struct espi_event evt = {
		.evt_type = ESPI_BUS_PERIPHERAL_NOTIFICATION,
		.evt_details = cfg->evt_details,
		.evt_data = ESPI_PERIPHERAL_NODATA,
	};

	if (with_data) {
		struct espi_evt_data_acpi *acpi_evt = (struct espi_evt_data_acpi *)&evt.evt_data;

		evt.evt_data = 0U;
		acpi_evt->type = type;
		acpi_evt->data = byte;
	}

	mchp_xec_espi_v3_send_callbacks(cfg->espi_dev, evt);
}

/* Host wrote the command or the data register of our four byte window. */
static void acpi_ec_ibf_isr(const struct device *dev)
{
	const struct espi_pc_acpi_ec_xec_config *cfg = dev->config;
	uint8_t type = ESPI_EVENT_DATA_ACPI_TYPE_HOST_TO_EC_DATA;
	uint8_t byte = 0U;
	bool with_data = false;

	if (IS_ENABLED(CONFIG_ESPI_XEC_V3_PC_ACPI_EC_IBF_EVT_DATA)) {
		if ((cfg->regs->EC_STS & MCHP_ACPI_EC_STS_IBF) == 0U) {
			xec_pc_girq_clr(cfg->ibf_ecia_info);
			return;
		}

		/* Tell the Host a command is in progress. The application clears
		 * this by writing the result through EACPI_WRITE_CHAR.
		 */
		cfg->regs->EC_STS |= MCHP_ACPI_EC_STS_UD1A;

		if ((cfg->regs->EC_STS & MCHP_ACPI_EC_STS_CMD) != 0U) {
			type = ESPI_EVENT_DATA_ACPI_TYPE_HOST_TO_EC_CMD;
		}

		/* Reading the data register clears IBF in hardware. */
		byte = (uint8_t)cfg->regs->OS2EC_DATA;
		with_data = true;
	}

	/* Clear the latched interrupt status before invoking the application so
	 * an edge arriving from the Host while the callback runs is not lost.
	 */
	xec_pc_girq_clr(cfg->ibf_ecia_info);

	acpi_ec_notify(dev, type, byte, with_data);
}

/* Host read the byte we put in the EC to Host register. */
static void acpi_ec_obe_isr(const struct device *dev)
{
	const struct espi_pc_acpi_ec_xec_config *cfg = dev->config;

	/* OBE stays asserted while the output buffer is empty, so mask the
	 * source. The next EACPI_WRITE_CHAR re-arms it.
	 */
	xec_pc_girq_ctrl(cfg->obe_ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	xec_pc_girq_clr(cfg->obe_ecia_info);

	acpi_ec_notify(dev, ESPI_EVENT_DATA_ACPI_TYPE_HOST_RD_EC_TO_HOST, 0U, true);
}

static int acpi_ec_lpc_request(const struct device *dev, enum lpc_peripheral_opcode op,
			       uint32_t *data, bool write)
{
	const struct espi_pc_acpi_ec_xec_config *cfg = dev->config;

	if (data == NULL) {
		return -EINVAL;
	}

	if (write) {
		switch (op) {
		case EACPI_WRITE_CHAR:
			cfg->regs->EC2OS_DATA = (*data & 0xffU);
			break;
		case EACPI_WRITE_STS:
			cfg->regs->EC_STS = (uint8_t)(*data & 0xffU);
			break;
#ifdef XEC_ACPI_EC_HOST_CMD
		case ECUSTOM_HOST_CMD_SEND_RESULT:
			/* Writing the data register sets OBF, which is what tells
			 * the Host the result is there. Dropping the command in
			 * progress flag the IBF handler set has to come after
			 * that, so the Host never sees an idle status with no
			 * result behind it.
			 */
			cfg->regs->EC2OS_DATA = (*data & 0xffU);
			cfg->regs->EC_STS &= ~MCHP_ACPI_EC_STS_UD1A;
			break;
#endif
		default:
			return -EINVAL;
		}

		return 0;
	}

	switch (op) {
	case EACPI_OBF_HAS_CHAR:
		/* EC has written data but the Host has not read it */
		*data = cfg->regs->EC_STS & MCHP_ACPI_EC_STS_OBF;
		break;
	case EACPI_IBF_HAS_CHAR:
		/* Host has written data but the EC has not read it */
		*data = cfg->regs->EC_STS & MCHP_ACPI_EC_STS_IBF;
		break;
	case EACPI_READ_STS:
		*data = cfg->regs->EC_STS;
		break;
#ifdef CONFIG_ESPI_PERIPHERAL_ACPI_SHM_REGION
	case EACPI_GET_SHARED_MEMORY:
		if (cfg->shm == NULL) {
			return -ENOTSUP;
		}
		*data = (uint32_t)(uintptr_t)cfg->shm;
		break;
#endif
#ifdef XEC_ACPI_EC_HOST_CMD
	case ECUSTOM_HOST_CMD_GET_PARAM_MEMORY:
		/* The EC host command backend uses this region as its transmit
		 * buffer and reads the request out of it, so an instance in this
		 * role without shared-memory-size cannot serve the Host.
		 */
		if (cfg->shm == NULL) {
			return -ENOTSUP;
		}
		*data = (uint32_t)(uintptr_t)cfg->shm;
		break;
	case ECUSTOM_HOST_CMD_GET_PARAM_MEMORY_SIZE:
		if (cfg->shm == NULL) {
			return -ENOTSUP;
		}
		*data = cfg->shm_size;
		break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

/* Apply the tracked interrupt enable to the hardware. IBF is armed whenever
 * interrupts are on, because the Host writes a command whenever it likes. OBE is
 * a level source that stays asserted for as long as the output buffer is empty,
 * so it is left masked here and armed by the application once it has a byte for
 * the Host.
 */
static void acpi_ec_intr_apply(const struct device *dev)
{
	const struct espi_pc_acpi_ec_xec_config *cfg = dev->config;
	struct espi_pc_acpi_ec_xec_data *data = dev->data;

	if (data->intr_en) {
		xec_pc_girq_clr(cfg->ibf_ecia_info);
		xec_pc_girq_ctrl(cfg->ibf_ecia_info, MCHP_MEC_ECIA_GIRQ_EN);
	} else {
		xec_pc_girq_ctrl(cfg->ibf_ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
		xec_pc_girq_ctrl(cfg->obe_ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	}
}

/* Called from the controller espi_interrupt_config() implementation. */
static void acpi_ec_intr_cfg(const struct device *dev, bool enable)
{
	struct espi_pc_acpi_ec_xec_data *data = dev->data;

	data->intr_en = enable;
	acpi_ec_intr_apply(dev);
}

static void acpi_ec_espi_event(const struct device *espi_dev, const struct device *dev,
			       enum mchp_xec_espi_pc_event evt)
{
	const struct espi_pc_acpi_ec_xec_config *cfg = dev->config;

	ARG_UNUSED(espi_dev);

	if (xec_pc_evt_is_hw_usable(evt)) {
		/* The controller has programmed our BARs and OBF Serial IRQ, so
		 * a Host command arriving right after PLTRST or eSPI Reset
		 * de-assertion is seen if the application asked for interrupts.
		 */
		acpi_ec_intr_apply(dev);
	} else {
		xec_pc_girq_ctrl(cfg->ibf_ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
		xec_pc_girq_ctrl(cfg->obe_ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	}
}

static int espi_pc_acpi_ec_xec_init(const struct device *dev)
{
	const struct espi_pc_acpi_ec_xec_config *cfg = dev->config;
	struct espi_pc_acpi_ec_xec_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->espi_dev)) {
		LOG_ERR("ACPI EC: eSPI controller not ready");
		return -ENODEV;
	}

	xec_pc_girq_ctrl(cfg->ibf_ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	xec_pc_girq_ctrl(cfg->obe_ecia_info, MCHP_MEC_ECIA_GIRQ_DIS);
	xec_pc_girq_clr(cfg->ibf_ecia_info);
	xec_pc_girq_clr(cfg->obe_ecia_info);

	data->pc_cb.pc_dev = dev;
	data->pc_cb.handler = acpi_ec_espi_event;
	data->pc_cb.evt_mask = XEC_PC_EVT_MASK_ALL;

	data->pc_cb.intr_cfg = acpi_ec_intr_cfg;
	data->pc_cb.intr_flags = ESPI_PERIPHERAL_HOST_IO_EVENTS;
	data->intr_en = true;

	if (cfg->serves_eacpi) {
		data->pc_cb.lpc_request = acpi_ec_lpc_request;
		data->pc_cb.lpc_op_start = EACPI_START_OPCODE;
		data->pc_cb.lpc_op_end = EACPI_MAX_OPCODE;
#ifdef XEC_ACPI_EC_HOST_CMD
	} else if (cfg->serves_host_cmd) {
		data->pc_cb.lpc_request = acpi_ec_lpc_request;
		data->pc_cb.lpc_op_start = ECUSTOM_HOST_CMD_GET_PARAM_MEMORY;
		data->pc_cb.lpc_op_end = ECUSTOM_HOST_CMD_SEND_RESULT;
#endif
	}

	ret = mchp_xec_espi_pc_register(cfg->espi_dev, &data->pc_cb);
	if (ret != 0) {
		LOG_ERR("ACPI EC: eSPI registration failed (%d)", ret);
		return ret;
	}

	cfg->irq_connect();

	/* The Host facing decoders are held in reset until the peripheral
	 * channel is enabled, so leave both sources masked. The event handler
	 * enables IBF once the controller reports the hardware is usable.
	 */

	return 0;
}

/* A shared memory region belongs to the instance that hands its address out, and
 * the only two ways out are opcodes: the "host-io" instance answers
 * EACPI_GET_SHARED_MEMORY, which CONFIG_ESPI_PERIPHERAL_ACPI_SHM_REGION declares,
 * and the "ec-host-cmd" instance answers ECUSTOM_HOST_CMD_GET_PARAM_MEMORY.
 * Nothing else in the API exposes the address, so a region whose opcode is not
 * compiled in could not be reached by anyone and is not allocated. That is what
 * lets a board describe the region on the node it belongs to and still pay
 * nothing for it in a build that cannot use it.
 *
 * The V2 driver carves both regions out of one buffer, sized by two Kconfig
 * symbols and compiled in only with the EC host command support, so its shared
 * memory region cannot be had without that. Here each region is sized on the node
 * of the instance that owns it, and the two are independent.
 */
#ifdef CONFIG_ESPI_PERIPHERAL_ACPI_SHM_REGION
#define XEC_ACPI_EC_SHM_HOST_IO 1
#else
#define XEC_ACPI_EC_SHM_HOST_IO 0
#endif

#ifdef XEC_ACPI_EC_HOST_CMD
#define XEC_ACPI_EC_SHM_HOST_CMD 1
#else
#define XEC_ACPI_EC_SHM_HOST_CMD 0
#endif

#define XEC_ACPI_EC_HAS_SHM(n)                                                                     \
	UTIL_AND(DT_INST_NODE_HAS_PROP(n, shared_memory_size),                                     \
		 UTIL_OR(UTIL_AND(IS_EQ(DT_INST_ENUM_IDX(n, peripheral_type),                      \
					XEC_ACPI_EC_ROLE_HOST_IO),                                 \
				  XEC_ACPI_EC_SHM_HOST_IO),                                        \
			 UTIL_AND(IS_EQ(DT_INST_ENUM_IDX(n, peripheral_type),                      \
					XEC_ACPI_EC_ROLE_EC_HOST_CMD),                             \
				  XEC_ACPI_EC_SHM_HOST_CMD)))

#define XEC_ACPI_EC_SHM_DEFINE(n)                                                                  \
	IF_ENABLED(XEC_ACPI_EC_HAS_SHM(n),                                                         \
		   (static uint8_t acpi_ec_shm_##n[DT_INST_PROP(n, shared_memory_size)]            \
			    __aligned(8);))

#define XEC_ACPI_EC_SHM_INIT(n)                                                                    \
	IF_ENABLED(XEC_ACPI_EC_HAS_SHM(n),                                                         \
		   (.shm = acpi_ec_shm_##n, .shm_size = sizeof(acpi_ec_shm_##n),))

#define XEC_ACPI_EC_DEVICE(n)                                                                      \
	XEC_ACPI_EC_SHM_DEFINE(n)                                                                  \
                                                                                                   \
	static void acpi_ec_irq_connect_##n(void)                                                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, ibf, irq),                                      \
			    DT_INST_IRQ_BY_NAME(n, ibf, priority), acpi_ec_ibf_isr,                \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(n, ibf, irq));                                      \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, obe, irq),                                      \
			    DT_INST_IRQ_BY_NAME(n, obe, priority), acpi_ec_obe_isr,                \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(n, obe, irq));                                      \
	}                                                                                          \
                                                                                                   \
	static const struct espi_pc_acpi_ec_xec_config espi_pc_acpi_ec_xec_cfg_##n = {             \
		.regs = (struct acpi_ec_regs *)DT_INST_REG_ADDR(n),                                \
		.espi_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                      \
		.irq_connect = acpi_ec_irq_connect_##n,                                            \
		XEC_ACPI_EC_SHM_INIT(n).ibf_ecia_info = DT_INST_PROP_BY_IDX(n, girqs, 0),          \
		.obe_ecia_info = DT_INST_PROP_BY_IDX(n, girqs, 1),                                 \
		.evt_details =                                                                     \
			_CONCAT(ESPI_PERIPHERAL_, DT_INST_STRING_UPPER_TOKEN(n, peripheral_type)), \
		.serves_eacpi =                                                                    \
			(DT_INST_ENUM_IDX(n, peripheral_type) == XEC_ACPI_EC_ROLE_HOST_IO),        \
		.serves_host_cmd =                                                                 \
			(DT_INST_ENUM_IDX(n, peripheral_type) == XEC_ACPI_EC_ROLE_EC_HOST_CMD),    \
	};                                                                                         \
                                                                                                   \
	static struct espi_pc_acpi_ec_xec_data espi_pc_acpi_ec_xec_data_##n;                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, espi_pc_acpi_ec_xec_init, NULL, &espi_pc_acpi_ec_xec_data_##n,    \
			      &espi_pc_acpi_ec_xec_cfg_##n, POST_KERNEL,                           \
			      CONFIG_ESPI_XEC_V3_PC_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(XEC_ACPI_EC_DEVICE)
