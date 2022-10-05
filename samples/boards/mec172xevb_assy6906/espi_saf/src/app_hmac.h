/*
 * Copyright (c) 2022 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __TEST_HMAC_H__
#define __TEST_HMAC_H__

int app_crypto_init(void);

int app_get_rand(uint8_t *dest, size_t destsz, size_t req_nbytes);

int app_hmac_sha256_mesg(const uint8_t *key, size_t keysz,
			 const uint8_t *msg, size_t msgsz,
			 uint8_t *result, size_t resultsz);

int app_hmac_test(void);

#endif /* __TEST_HMAC_H__ */
