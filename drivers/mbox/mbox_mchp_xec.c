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
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>

LOG_MODULE_REGISTER(mbox_xec, CONFIG_MBOX_LOG_LEVEL);

#define DT_DRV_COMPAT microchip,xec-mailbox

/* XEC Mailbox peripheral has 32 8-bit mailbox registers
 * Drivers support 8 channels defined as:
 * channel 0: data transport
 * channels 1 - 7: signal only
 */
#define XEC_MBOX_MAX_CHANNELS	8
#define XEC_MBOX_MAX_MSG_LEN	36
#define XEC_MBOX_MSG_DATA_OFS	4
#define XEC_MBOX_MSG_MAX_WORDS	((32 / 4) + 1)

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
	struct k_sem lock;
	mbox_channel_id_t cb_chan_id;
	mbox_callback_t cb;
	void *cb_user_data;
	struct mbox_msg msg;
	uint32_t msgbuf[XEC_MBOX_MSG_MAX_WORDS];
};

#if 0
typedef void (*mbox_callback_t)(const struct device *dev,
				mbox_channel_id_t channel_id, void *user_data,
				struct mbox_msg *data);
#endif

/* Host wrote value to Host-to-EC 8-bit register.
 * Host may have updates any of the other MBox registers before writing Host-to-EC.
 */
static void xec_mbox_isr(const struct device *dev)
{
	struct xec_mbox_drv_data *drvdat = dev->data;
	const struct xec_mbox_drv_config *drvcfg = dev->config;
	mem_addr_t ba = drvcfg->regbase;
	struct mbox_msg *msg = &drvdat->msg;
	uint32_t *mbuf = drvdat->msgbuf;
	uint32_t ofs = 0, temp = 0;

	temp = sys_read8(ba + XEC_MBX_H2E_MB_OFS);
	temp |= ((uint32_t)sys_read8(ba + XEC_MBX_E2H_MB_OFS) << 8);
	temp |= ((uint32_t)sys_read8(ba + XEC_MBX_SS_OFS) << 16);
	temp |= ((uint32_t)sys_read8(ba + XEC_MBX_SM_OFS) << 24);
	*mbuf++ = temp;

	ofs = XEC_MBX_DATA_MB_OFS(0);
	while (ofs < XEC_MBX_DATA_MB_OFS(32u)) {
		*mbuf++ = sys_read32(ba + ofs);
		ofs += 4u;
	}

	if (drvdat->cb != NULL) {
		msg->data = (const void *)drvdat->msgbuf;
		msg->size = XEC_MBOX_MAX_MSG_LEN;
		drvdat->cb(dev, cb->cb_chan_id, drvdat->cb_user_data, msg);
	}

	/* clear MBox GIRQ latched status. Spec says reading Host-to-EC clears the HW interrupt
	 * signal from MBox block.
	 */
	soc_ecia_girq_status_clear(drvcfg->girq, drvcfg->girq_pos);

	/* signal host we have received and handled message */
	sys_write8(XEC_MBX_H2E_MB_CLR_VAL, ba + XEC_MBX_H2E_MB_OFS);

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

	dd->cb_chan_id = chan_id;
	dd->cb = cb;
	dd->cb_user_data = user_data;

	k_sem_give(&dd->lock);

	return -ENOTSUP;
}

/* Send message to external Host.
 * Requires eSPI driver has configured MBox Serial-IRQ(s).
 * Channel 0: data mode only
 * Channels 1 - 7: signal or data mode.
 */
static int xec_mbox_send(const struct device *dev, mbox_channel_id_t chan_id,
			 const struct mbox_msg *msg)
{
	const struct xec_mbox_drv_config *drvcfg = dev->config;
	struct xec_mbox_drv_data *drvdat = dev->data;
	mem_addr_t ba = drvcfg->regbase;

	if (chan_id >= XEC_MBOX_MAX_CHANNELS) {
		return -EINVAL;
	}

	if ((chan_id == 0) && (msg == NULL)) {
		return -EINVAL;
	}

	k_sem_take(&drvdat->lock, K_FOREVER);

	if (msg == NULL) { /* signalling mode? */
		soc_mmcr_set_bit8(ba + XEC_MBX_SS_OFS, BIT(chan_id));
		k_sem_give(&drvdat->lock);
		return 0;
	}

	/* Copy message bytes into data mbox registers */

	/* Write value to EC-to-Host register. Write triggers Serial-IRQ(s) to Host.
	 * Issue: HW was designed for a polling mode where EC writes a non-zero value to
	 * the EC-to-Host register. EC and then poll on this register waiting for Host to
	 * clear it. (Host writes 0xFF to the register to clear it.
	 * ISSUE 1: channel ID may be 0 which doesn't work with this HW scheme.
	 * ISSUE 2: Polling is blocking. We need timeout.
	 *          HW does not generate an interrupt when Host clears this register.
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
	return XEC_MBOX_MAX_MSG_LEN;
}

static uint32_t xec_mbox_max_channels_get(const struct device *dev)
{
	return (uint32_t)XEC_MBOX_MAX_CHANNELS;
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