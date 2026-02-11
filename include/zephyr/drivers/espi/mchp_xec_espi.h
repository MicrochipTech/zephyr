/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file mchp_xec_espi.h
 * @brief Microchip eSPI Driver Header
 *
 * @details
 * This header provides conditional inclusion of Microchip eSPI controller
 * extension to types from espi.h
 */

#ifndef INCLUDE_ZEPHYR_DRIVERS_ESPI_MCHP_XEC_ESPI_H_
#define INCLUDE_ZEPHYR_DRIVERS_ESPI_MCHP_XEC_ESPI_H_

#include <zephyr/drivers/espi.h>

/** @brief Extend espi_virtual_peripheral enum */
enum mchp_xec_espi_virtual_peripheral {
	MCHP_XEC_ESPI_PERIPHERAL_MAILBOX = ESPI_PERIPHERAL_MAX,
	MCHP_XEC_ESPI_PERIPHERAL_MAX,
};

#if defined(CONFIG_ESPI_PERIPHERAL_CUSTOM_OPCODE) && defined(CONFIG_ESPI_PERIPHERAL_XEC_MAILBOX)
#define ECUSTOM_HOST_CMD_GET_MAILBOX_DATA (ECUSTOM_HOST_CMD_SEND_RESULT + 1)
#define ECUSTOM_HOST_CMD_SET_MAILBOX_DATA (ECUSTOM_HOST_CMD_SEND_RESULT + 2)
#endif

#endif /* INCLUDE_ZEPHYR_DRIVERS_ESPI_MCHP_XEC_ESPI_H_ */
