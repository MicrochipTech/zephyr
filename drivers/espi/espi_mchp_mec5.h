/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ESPI_MCHP_MEC5_ESPI_H_
#define ZEPHYR_DRIVERS_ESPI_MCHP_MEC5_ESPI_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/pinctrl.h>

struct espi_mec5_drv_cfg {
	uintptr_t ioc_base;
	uintptr_t memc_base;
	uintptr_t vwc_base;
	void (*irq_config)(const struct device *dev);
	const struct pinctrl_dev_config *pcfg;
	uint8_t freq_id;
	uint8_t chan_msk;
#if defined(CONFIG_ESPI_OOB_CHANNEL)
	uint16_t oob_rxb_size;
#endif
};

struct espi_mec5_drv_data {
	const struct device *dev;
	volatile uint32_t status;
	struct espi_callback vwcb;
	sys_slist_t callbacks;
#if defined(CONFIG_ESPI_OOB_CHANNEL)
	struct k_sem oob_tx_sync;
	struct k_sem oob_rx_sync;
	uint32_t oob_tx_status;
	uint32_t oob_rx_status;
	uint8_t *oob_rxb;
	uint16_t oob_rx_len;
#endif
};

#if defined(CONFIG_ESPI_PERIPHERAL_CHANNEL)
int espi_mec5_pc_pltrst_handler(const struct device *dev, uint8_t pltrst_state);
void espi_mec5_pc_irq_connect(const struct device *dev);
void espi_mec5_pc_erst_config(const struct device *dev, uint8_t n_erst_state);

int espi_mec5_read_req_api(const struct device *dev, struct espi_request_packet *req);

int espi_mec5_write_req_api(const struct device *dev, struct espi_request_packet *req);

int espi_mec5_read_lpc_req_api(const struct device *dev, enum lpc_peripheral_opcode op,
				uint32_t *data);

int espi_mec5_write_lpc_req_api(const struct device *dev, enum lpc_peripheral_opcode op,
				uint32_t *data);
#endif

#if defined(CONFIG_ESPI_VWIRE_CHANNEL)
int espi_mec5_init_vwires(const struct device *dev);
void espi_mec5_vw_irq_connect(const struct device *dev);
void espi_mec5_vw_erst_config(const struct device *dev, uint8_t n_erst_state);

int espi_mec5_send_vwire_api(const struct device *dev, enum espi_vwire_signal vw, uint8_t level);
int espi_mec5_recv_vwire_api(const struct device *dev, enum espi_vwire_signal vw, uint8_t *level);
#endif

#if defined(CONFIG_ESPI_OOB_CHANNEL)
void espi_mec5_oob_irq_connect(const struct device *dev);
void espi_mec5_oob_erst_config(const struct device *dev, uint8_t n_erst_state);

int espi_mec5_send_oob_api(const struct device *dev, struct espi_oob_packet *pckt);
int espi_mec5_recv_oob_api(const struct device *dev, struct espi_oob_packet *pckt);
#endif

#if defined(CONFIG_ESPI_FLASH_CHANNEL)
void espi_mec5_fc_irq_connect(const struct device *dev);
void espi_mec5_fc_erst_config(const struct device *dev, uint8_t n_erst_state);

/* eSPI driver flash channel API implementation */
int espi_mec5_flash_read_api(const struct device *dev, struct espi_flash_packet *pckt);
int espi_mec5_flash_write_api(const struct device *dev, struct espi_flash_packet *pckt);
int espi_mec5_flash_erase_api(const struct device *dev, struct espi_flash_packet *pckt);
#endif

#endif /*  ZEPHYR_DRIVERS_ESPI_MCHP_MEC5_ESPI_H_ */
