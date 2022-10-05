/*
 * Copyright (c) 2022 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SAF_RPMC_H__
#define __SAF_RPMC_H__

#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_saf.h>

#define SAF_TEST_FLAG_FLASH_KNOWN	BIT(0)
#define SAF_TEST_FLAG_FLASH_HAS_RPMC	BIT(1)

#define SAF_CB_ECP_DONE 1
#define SAF_CB_BUS_MON  2

#define SAF_RPMC_OP1_CMD_SET_ROOT_KEY		0x00
#define SAF_RPMC_OP1_CMD_UPDATE_HMAC_KEY	0x01
#define SAF_RPMC_OP1_CMD_INCR_COUNTER		0x02
#define SAF_RPMC_OP1_CMD_REQUEST_COUNTER	0x03

#define SAF_RPMC_OP1_CMD_SET_ROOT_KEY_SIZE	64u
#define SAF_RPMC_OP1_CMD_UPDATE_HMAC_KEY_SIZE	40u
#define SAF_RPMC_OP1_CMD_INCR_COUNTER_SIZE	40u
#define SAF_RPMC_OP1_CMD_REQ_COUNTER_SIZE	48u

#define SAF_RPMC_OP2_CMD_READ_STS_DATA_SIZE	49u

/* RPMC flash extended status values. Complicated, refer to spec. */
#define SAF_RPMC_EXT_STATUS_PWR_ON_STATE	0x00U
#define SAF_RPMC_EXT_STATUS_PWR_ON_STATE_MSK	0xffU

#define SAF_RPMC_EXT_STATUS_OP1_SUCCESS		BIT(7)

#define SAF_RPMC_EXT_STATUS_BUSY		BIT(0)

/* Root Key overwrite error: OP1 Set Root Key
 * Monotonic counter uninitialize: OP1 Update HMAC key
 */
#define SAF_RPMC_EXT_STATUS_RK_INIT_ERROR	BIT(1)

/* Signature mis-match, counter address out of range,
 * truncated signature mis-match, bad command type,
 * or incorrect payload size.
 */
#define SAF_RPMC_EXT_STATUS_BAD_PARAM_ERROR	BIT(2)

/* HMAC key or monotonic counter unitialized */
#define SAF_RPMC_EXT_STATUS_HK_INIT_ERROR	BIT(3)

/* Counter data mis-match error */
#define SAF_RPMC_EXT_STATUS_CNTR_DATA_ERROR	BIT(4)

#define SAF_RPMC_EXT_STATUS_FATAL_ERROR		BIT(5)

/* from main.c */
extern const struct device *espi_saf_dev;
extern volatile uint32_t espi_saf_cb_sts;
extern volatile uint8_t espi_saf_cb_val;

/* initialize RPMC keys used by the application. Must be called before
 * any RPMC operations.
 */
int rpmc_init_keys(void);

int saf_rpmc_spi_check_root_key(const struct device *qspi_dev, uint8_t counter_id);

/* for all counters */
int saf_rpmc_spi_check_root_keys(const struct device *spi_dev);

struct espi_saf_cfg *get_saf_rpmc_flash_config(void);
struct espi_saf_protection *get_saf_rpmc_flash_pr(void);

int espi_saf_rpmc_test1(void);

#endif /* __SAF_RPMC_H__ */
