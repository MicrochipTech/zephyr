/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __TAF_RPMC_H__
#define __TAF_RPMC_H__

#include <device_mec5.h>
#include <mec_espi_taf.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_taf.h>
#include <zephyr/drivers/espi/espi_mchp_mec5.h>
#include <zephyr/drivers/espi/espi_taf_mchp_mec5.h>

#define TAF_TEST_FLAG_FLASH_KNOWN	BIT(0)
#define TAF_TEST_FLAG_FLASH_HAS_RPMC	BIT(1)

#define TAF_CB_ECP_DONE 1
#define TAF_CB_BUS_MON  2

#define RPMC_OP1_CMD_SET_ROOT_KEY		0x00
#define RPMC_OP1_CMD_UPDATE_HMAC_KEY		0x01
#define RPMC_OP1_CMD_INCR_COUNTER		0x02
#define RPMC_OP1_CMD_REQUEST_COUNTER		0x03

#define RPMC_OP1_CMD_SET_ROOT_KEY_SIZE		64u
#define RPMC_OP1_CMD_UPDATE_HMAC_KEY_SIZE	40u
#define RPMC_OP1_CMD_INCR_COUNTER_SIZE		40u
#define RPMC_OP1_CMD_REQ_COUNTER_SIZE		48u

#define RPMC_OP2_CMD_READ_STS_DATA_SIZE		49u

/* RPMC flash extended status values. Complicated, refer to spec. */
#define RPMC_EXT_STATUS_PWR_ON_STATE		0x00U
#define RPMC_EXT_STATUS_PWR_ON_STATE_MSK	0xffU

#define RPMC_EXT_STATUS_OP1_SUCCESS		BIT(7)

#define RPMC_EXT_STATUS_BUSY			BIT(0)

/* Root Key overwrite error: OP1 Set Root Key
 * Monotonic counter uninitialize: OP1 Update HMAC key
 */
#define RPMC_EXT_STATUS_RK_INIT_ERROR		BIT(1)

/* Signature mis-match, counter address out of range,
 * truncated signature mis-match, bad command type,
 * or incorrect payload size.
 */
#define RPMC_EXT_STATUS_BAD_PARAM_ERROR		BIT(2)

/* HMAC key or monotonic counter unitialized */
#define RPMC_EXT_STATUS_HK_INIT_ERROR		BIT(3)

/* Counter data mis-match error */
#define RPMC_EXT_STATUS_CNTR_DATA_ERROR		BIT(4)

#define RPMC_EXT_STATUS_FATAL_ERROR		BIT(5)

/* initialize RPMC keys used by the application. Must be called before
 * any RPMC operations.
 */
int rpmc_init_keys(void);

int taf_rpmc_check_root_key(uint8_t counter_id, uint8_t *rpmc_ext_status);

/* for all counters */
int taf_rpmc_check_root_keys(uint8_t num_rpmc_counters);

int espi_taf_rpmc_test1(void);

#endif /* __TAF_RPMC_H__ */
