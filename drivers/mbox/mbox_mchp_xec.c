/*
 * Copyright (c) 2025 Microchip Technology Incorporated.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/irq.h>

#define LOG_LEVEL CONFIG_MBOX_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(microchip_xec_mailbox);

#define DT_DRV_COMPAT microchip,xec-mailbox

/* XEC Mailbox peripheral has 32 8-bit mailbox registers */
#define XEC_MBOX_MAX_CHANNELS	32

/* XEC Mailbox hardware registers */
#define XEC_MBX_INDEX_OFS	0
#define XEC_MBX_DATA_OFS	1u

/* register only visible to the EC */
#define XEC_MBX_H2E_MB_OFS	0x100u
#define XEC_MBX_H2E_MB_MSK	GENMASK(7, 0)
#define XEC_MBX_H2E_MB_CLR_VAL	0xffu

#define XEC_MBX_E2H_MB_OFS	0x104u
#define XEC_MBX_E2H_MB_MSK	GENMASK(7, 0)
#define XEC_MBX_E2H_MB_CLR_VAL	0xffu

/* SMI Source: R/W for EC and R/W1C for Host */
#define XEC_MBX_SS_OFS		0x108u
/* SMI Source: R/W for EC and Host */
#define XEC_MBX_SM_OFS		0x10cu

#define XEC_MBX_SSM_EC_WR_POS	0
#define XEC_MBX_SSM_SWI_POS	1
#define XEC_MBX_SSM_SWI_MSK	GENMASK(7, 1)
#define XEC_MBX_SSM_SWI_SET(n)	FIELD_PREP(XEC_MBX_SS_SWI_MSK, (n))
#define XEC_MBX_SSM_SWI_GET(n)	FIELD_GET(XEC_MBX_SS_SWI_MSK, (n))

/* 0 <= n < 32 */
#define XEC_MBX_DATA_MB_OFS(n)	(0x110u + (uint32_t)(n))

struct xec_mbox_drv_config {
	uintptr_t regbase;
	void (*irq_config_fp)(const struct device *dev);
	uint8_t girq;
	uint8_t girq_pos;
};

struct xec_mbox_drv_data {
	k_sem lock;
	uint32_t chan_enables;
	mbox_channel_id_t cb_chan_id[XEC_MBOX_MAX_CHANNELS];
	mbox_callback_t cb[XEC_MBOX_MAX_CHANNELS];
	void *cb_user_data[XEC_MBOX_MAX_CHANNELS];
};

#if 0
typedef void (*mbox_callback_t)(const struct device *dev,
				mbox_channel_id_t channel_id, void *user_data,
				struct mbox_msg *data);
#endif

static void xec_mbox_isr(const struct device *dev)
{
	/* TODO */

	/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
	 * Store immediate overlapping exception return operation
	 * might vector to incorrect interrupt
	 */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
	barrier_dsync_fence_full();
#endif
}

static int xec_mbox_register_callback(const struct device *dev, mbox_channel_id_t chan_id,
				      mbox_callback_t cb, void *user_data)
{
	struct xec_mbox_drv_data *dd = dev->data;

	if (chan_id >= XEC_MBOX_MAX_CHANNELS) {
		return -EINVAL;
	}

	k_sem_take(&dd->lock, K_FOREVER);

	dd->cb_chan_id[chan_id] = chan_id;
	dd->cb[chan_id] = cb;
	dd->cb_user_data[chan_id] = user_data;

	k_sem_give(&dd->lock);

	return -ENOTSUP;
}

static int xec_mbox_send(const struct device *dev, mbox_channel_id_t chan_id,
			 const struct mbox_msg *msg)
{
	const struct xec_mbox_drv_config *drvcfg = dev->config;
	struct xec_mbox_drv_data *dd = dev->data;
	mem_addr_t ba = drvcfg->regbase;

	if (chan_id >= XEC_MBOX_MAX_CHANNELS) {
		return -EINVAL;
	}

	k_sem_take(&dd->lock, K_FOREVER);

	if (msg == NULL) { /* signalling mode? */
		/* write chan_id to EC-to-Host register.
		 * If Serial-IRQ has been configured by eSPI driver on VWire nPLTRST
		 * de-assertion then HW will send the MBox EC-to-Host SIRQ to the Host.
		 */
		sys_write8((uint8_t)(chan_id & 0xffu), ba + XEC_MBX_E2H_MB_OFS);
		k_sem_give(&dd->lock);
		return 0;
	}

	/* data send:
	 * Option 1:
	 *   Translate channel ID to HW mbox index.
	 *   Write data to HW mbox index ... index + len(data)
	 *   Must check we don't overrun HW mbox index range.
	 *   Write channel ID to EC-to-Host MBX HW reg to trigger interrupt to Host.
	 * Option 2:
	 *   Translate channel ID to HW mbox index.
	 *   for byte in msg->data
	 *     write byte to HW mbox index
	 *     Write channel ID to EC-to-Host MBX HW reg to trigger interrupt to Host.
	 *     Poll for Host to acknowledge (Host clears EC-to-Host MBX reg)
	 *    end for
	 * Option 3:
	 *    if msg is not NULL
	 *      write msg->data to mbox data [0..31]
	 *    endif
	 *    write channel ID to EC-to-Host MBX HW reg
	 *
	 */
	return -ENOTSUP;
}

/* Returns maximum mailbox message size in bytes.
 * A return value of 0 indicates the driver only does signalling.
 * We have 32 8-bit mailbox registers and expose them as channels.
 * The caller could write all 32 8-bit registers in one call starting
 * from channel ID 0.
 */
static int xec_mbox_mtu_get(const struct device *dev)
{
	return XEC_MBOX_MAX_CHANNELS;
}

static uint32_t xec_mbox_max_channels_get(const struct device *dev)
{
	return XEC_MBOX_MAX_CHANNELS;
}

static int xec_mbox_set_enabled(const struct device *dev, mbox_channel_id_t chan_id, bool enabled)
{
	struct xec_mbox_drv_data *dd = dev->data;

	if (chan_id >= XEC_MBOX_MAX_CHANNELS) {
		return -EINVAL;
	}

	k_sem_take(&dd->lock, K_FOREVER);

	if (enabled == true) {
		dd->chan_enables |= BIT(chan_id);
	} else {
		dd->chan_enables &= (uint32_t)~BIT(chan_id);
	}

	k_sem_give(&dd->lock);

	return 0;
}

static int xec_mbox_init(const struct device *dev)
{
	const struct xec_mbox_drv_config *drvcfg = dev->config;
	struct xec_mbox_drv_data *dd = dev->data;

	k_sem_init(&dd->lock, 1, 1);

	if (drvcfg->irq_config_fp != NULL) {
		drvcfg->irq_config_fp(dev);
	}

	return 0;
}

static DEVICE_API(mbox, xec_mbox_driver_api) = {
	.send = xec_mbox_send,
	.register_callback = xec_mbox_register_callback,
	.mtu_get = xec_mbox_mtu_get,
	.max_channels_get = xec_mbox_max_channels_get,
	.set_enabled = xec_mbox_set_enabled,
};

#define MBOX_XEC_INST_DEF(inst) \
	static struct xec_mbox_drv_data xec_mbox##inst##_data; \
	static void xec_mbox##inst##irq_config(const struct device *dev) { \
		const struct xec_mbox_drv_config *drvcfg = dev->config;
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), xec_mbox_isr, \
			    DEVICE_DT_INST_GET(inst), 0); \
		irq_enable(DT_INST_IRQN(inst)); \
		soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, 1u); \
	} \
	static const struct xec_mbox_drv_config xec_mbox##inst##_dcfg = { \
		.regbase = DT_INST_REG_ADDR(inst), \
		.irq_config_fp = xec_mbox##inst##irq_config, \
		.girq = (uint8_t)DT_INST_PROP_BY_IDX(inst, girqs, 0), \
		.girq_pos = (uint8_t)DT_INST_PROP_BY_IDX(inst, girqs, 1), \
	}; \
	DEVICE_DT_INST_DEFINE(inst, xec_mbox_init, NULL, &xec_mbox##inst##_data,      \
			      &xec_mbox##inst##_dcfg, POST_KERNEL, CONFIG_MBOX_INIT_PRIORITY, \
			      &xec_mbox_driver_api)

DT_INST_FOREACH_STATUS_OKAY(MBOX_XEC_INST_DEF)