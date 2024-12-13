/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ESPI_ESPI_TAF_MCHP_MEC5_H_
#define ZEPHYR_INCLUDE_DRIVERS_ESPI_ESPI_TAF_MCHP_MEC5_H_

#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>

/**
 * @brief MCHP specific flags to TAF read, write, and erase APIs
 */
/* FLAG_CB_PER_BLOCK if set ISR will invoke the callback for every block transfered
 * otherwise the callback is invoked when all data transfer or erase operation is done.
 * FLAG_ASYNC if set API returns after starting TAF operation. Application must register
 * a callback to get notification of operation done.
 */
#define ESPI_TAF_MCHP_FLAG_CB_PER_BLOCK BIT(0)
#define ESPI_TAF_MCHP_FLAG_ASYNC BIT(4)

/**
 * @brief MCHP eSPI TAF events
 */
enum espi_taf_events {
	ESPI_TAF_ECP_READ = BIT(0),
	ESPI_TAF_ECP_WRITE = BIT(1),
	ESPI_TAF_ECP_ERASE = BIT(2),
	ESPI_TAF_ECP_RPMC_OP1 = BIT(3),
	ESPI_TAF_ECP_RPMC_OP2 = BIT(4),
	ESPI_TAF_ECP_PROTOCOL_ERR = BIT(5),
	ESPI_TAF_HOST_PROTOCOL_ERR = BIT(6),
};

/*
 * MCHP eSPI TAF protocol errors these errors may occur
 * for both EC initiated transaction via the TAF EC Portal (ECP)
 * or Host initiated TAF transactions.
 */
enum espi_taf_protocol_error {
	ESPI_TAF_PROTOCOL_ERR_TIMEOUT = BIT(0),
	ESPI_TAF_PROTOCOL_ERR_OUT_OF_RANGE = BIT(1),
	ESPI_TAF_PROTOCOL_ERR_ACCESS_VIOLATION = BIT(2),
	ESPI_TAF_PROTOCOL_ERR_4K_BOUNDARY = BIT(3),
	ESPI_TAF_PROTOCOL_ERR_ERASE_SIZE = BIT(4),
	ESPI_TAF_PROTOCOL_ERR_ECP_START_OVL = BIT(5),
	ESPI_TAF_PROTOCOL_ERR_ECP_BAD_REQ = BIT(6),
};

enum espi_taf_cmd {
	ESPI_TAF_CMD_RPMC_OP1 = 0,
	ESPI_TAF_CMD_RPMC_OP2,
};

#define ESPI_TAF_RPMC_OP1_CMD_SET_ROOT_KEY_SIZE		64u
#define ESPI_TAF_RPMC_OP1_CMD_UPDATE_HMAC_KEY_SIZE	40u
#define ESPI_TAF_RPMC_OP1_CMD_INCR_COUNTER_SIZE		40u
#define ESPI_TAF_RPMC_OP1_CMD_REQ_COUNTER_SIZE		48u

enum espi_taf_rpmc_subcmd {
	ESPI_TAF_SUBCMD_RPMC_OP1_SET_ROOT_KEY = 0,
	ESPI_TAF_SUBCMD_RPMC_OP1_UPDATE_HMAC_KEY,
	ESPI_TAF_SUBCMD_RPMC_OP1_INCR_COUNTER,
	ESPI_TAF_SUBCMD_RPMC_OP1_REQUEST_COUNTER,
	ESPI_TAF_SUBCMD_RPMC_OP1_MAX,
};

/**
 * @brief eSPI TAF RPMC transaction packet
 */
struct espi_taf_rpmc_packet {
	uint8_t *buf;
	uint32_t len;
	uint8_t cmd;
	uint8_t subcmd; /* RPMC OP1 only */
	uint8_t cs;
	uint8_t rsvd1;
};

/**
 * @brief eSPI TAF HW Monitor interrupt enable/disable
 */
int espi_taf_mchp_hwmon_ictrl(const struct device *dev, uint32_t intr_bitmap, uint8_t enable);

#define ESPI_TAF_MCHP_RPMC_OP_FLAG_ASYNC BIT(0)

/**
 * @brief eSPI TAF start RPMC operation on requested flash device with RPMC HW
 */
int espi_taf_mchp_rpmc_operation(const struct device *dev, struct espi_taf_rpmc_packet *pkt,
				 uint32_t flags);

#endif /* ZEPHYR_INCLUDE_DRIVERS_ESPI_ESPI_TAF_MCHP_MEC5_H_ */
