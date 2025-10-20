/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ESPI_ESPI_MCHP_XEC_NG_H_
#define ZEPHYR_DRIVERS_ESPI_ESPI_MCHP_XEC_NG_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/sys_io.h>

struct xec_espi_ng_drvcfg {
	mem_addr_t base;
	mem_addr_t mbase;
	mem_addr_t vwbase;
	const struct pinctrl_dev_config *pincfg;
	void (*irq_connect)(void);
	uint8_t irqn_vwb0;
	uint8_t irqn_vwb1;
};

struct xec_espi_ng_data {
	const struct device *dev;
#ifdef CONFIG_ESPI_VWIRE_CHANNEL
	struct k_work kwq_vwb0;
	struct k_work kwq_vwb1;
#endif
	struct espi_callback vwcb;
	sys_slist_t cbs;
#ifdef CONFIG_ESPI_OOB_CHANNEL
	struct k_sem oob_lock;
	struct k_sem oob_rx_sync;
	struct k_sem oob_tx_sync;
	uint8_t *oob_rxb;
	uint16_t oob_rx_len;
#endif
#ifdef CONFIG_ESPI_FLASH_CHANNEL
	struct k_sem fc_lock;
	struct k_sem fc_sync;
	volatile uint32_t fc_status;
#endif
};

#ifdef CONFIG_ESPI_PERIPHERAL_CHANNEL
int xec_espi_ng_pc_rd_api(const struct device *dev, struct espi_request_packet *req);
int xec_espi_ng_pc_wr_api(const struct device *dev, struct espi_request_packet *req);
/* TODO - data needs to change to another type or we must cast it (ugly and dangerous) */
int xec_espi_ng_pc_lpc_rd_api(const struct device *dev, enum lpc_peripheral_opcode op,
                              uint32_t *data);
int xec_espi_ng_pc_lpc_wr_api(const struct device *dev, enum lpc_peripheral_opcode op,
                              uint32_t *data);
/* Peripheral channel interrupt handlers all from core driver */
void xec_espi_ng_pc_handler(const struct device *dev);
void xec_espi_ng_bm1_handler(const struct device *dev);
void xec_espi_ng_bm2_handler(const struct device *dev);
void xec_espi_ng_ltr_handler(const struct device *dev);
#endif

#ifdef CONFIG_ESPI_VWIRE_CHANNEL
int xec_espi_ng_vw_send_api(const struct device *dev, enum espi_vwire_signal vw, uint8_t level);
int xec_espi_ng_vw_recv_api(const struct device *dev, enum espi_vwire_signal vw, uint8_t *level);
/* VWire channel interrupt handlers called from core driver */
void xec_espi_ng_vw_chen_handler(const struct device *dev);
void xec_espi_ng_vw_bank0_kworker(struct k_work *work);
void xec_espi_ng_vw_bank1_kworker(struct k_work *work);
/* VWire helpers called by core */
int xec_espi_ng_vw_init1(const struct device *dev);
#endif

#ifdef CONFIG_ESPI_OOB_CHANNEL
int xec_espi_ng_oob_send_api(const struct device *dev, struct espi_oob_packet *pkt);
int xec_espi_ng_oob_recv_api(const struct device *dev, struct espi_oob_packet *pkt);
/* OOB channel interrupt handlers called from core driver */
void xec_espi_ng_oob_dn_handler(const struct device *dev);
void xec_espi_ng_oob_up_handler(const struct device *dev);
#endif

#ifdef CONFIG_ESPI_FLASH_CHANNEL
int xec_espi_ng_fc_rd_api(const struct device *dev, struct espi_flash_packet *pkt);
int xec_espi_ng_fc_wr_api(const struct device *dev, struct espi_flash_packet *pkt);
int xec_espi_ng_fc_er_api(const struct device *dev, struct espi_flash_packet *pkt);
/* Flash channel interrupt handler called from core driver */
void xec_espi_ng_fc_handler(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_ESPI_ESPI_MCHP_XEC_NG_H_ */
