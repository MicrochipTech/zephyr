/* Copyright (c) 2024 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __APP_HMAC_H__
#define __APP_HMAC_H__

#include <stddef.h>
#include <stdint.h>

int app_crypto_init(void);

int app_get_rand(uint8_t *dest, size_t destsz, uint16_t req_nbytes);

int app_hmac_sha256_mesg(const uint8_t *key, size_t keysz,
			 const uint8_t *msg, size_t msgsz,
			 uint8_t *result, size_t resultsz);

int app_hmac_test(void);

#endif /* __APP_HMAC_H__ */
