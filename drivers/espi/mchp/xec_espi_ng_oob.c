/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_espi_ng

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/espi/mchp_xec_espi_ng.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

/* local */
#include "../espi_utils.h"
#include "../espi_mchp_xec_ng.h"
#include "mchp_xec_espi_regs.h"

LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

static uint8_t oob_rx_buf[CONFIG_ESPI_OOB_BUFFER_SIZE] __aligned(8);
static uint8_t oob_tx_buf[CONFIG_ESPI_OOB_BUFFER_SIZE] __aligned(8);

int xec_espi_ng_oob_send_api(const struct device *dev, struct espi_oob_packet *pkt)
{
	struct xec_espi_ng_data *data = dev->data;

	if (pkt == NULL) {
		return -EINVAL;
	}

	k_sem_take(&data->oob_lock, K_FOREVER);

	/* clear tx buffer before returning */
	memset(oob_tx_buf, 0x55, sizeof(oob_tx_buf));

	k_sem_give(&data->oob_lock);

	return 0;
}

int xec_espi_ng_oob_recv_api(const struct device *dev, struct espi_oob_packet *pkt)
{
	struct xec_espi_ng_data *data = dev->data;

	if (pkt == NULL) {
		return -EINVAL;
	}

	k_sem_take(&data->oob_lock, K_FOREVER);

	memset(oob_rx_buf, 0x55, sizeof(oob_rx_buf));

	k_sem_give(&data->oob_lock);

	return 0;
}

/* OOB received packet from eSPI Host */
void xec_espi_ng_oob_dn_handler(const struct device *dev)
{
	/* TODO */
}

/* OOB transmitted packet to eSPI Host */
void xec_espi_ng_oob_up_handler(const struct device *dev)
{
	/* TODO */
}
