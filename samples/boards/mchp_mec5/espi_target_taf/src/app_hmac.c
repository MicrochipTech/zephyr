/*
 * Copyright (c) 2022 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <soc.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(app);

#include "app_hmac.h"

#if defined(CONFIG_HAS_MEC5_HAL)
#include <mec_rom_api.h>
#elif defined(CONFIG_TINYCRYPT)
#include <tinycrypt/constants.h>
#include <tinycrypt/hmac.h>
#include <tinycrypt/hmac_prng.h>
#else
#error "BUILD ERROR: No crypto library defined in project!"
#endif

static const uint8_t hmac_key4[25] = {
	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
	0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
	0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
	0x19
};
static const uint8_t hmac_msg4[50] = {
	0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd,
	0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd,
	0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd,
	0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd,
	0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd,
	0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd,
	0xcd, 0xcd
};
static const uint8_t hmac_digest4[32] = {
	0x82, 0x55, 0x8a, 0x38, 0x9a, 0x44, 0x3c, 0x0e,
	0xa4, 0xcc, 0x81, 0x98, 0x99, 0xf2, 0x08, 0x3a,
	0x85, 0xf0, 0xfa, 0xa3, 0xe5, 0x78, 0xf8, 0x07,
	0x7a, 0x2e, 0x3f, 0xf4, 0x67, 0x29, 0x66, 0x5b
};

static const uint8_t key131[] = {
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xaa, 0xaa };

static const uint8_t msg54[] = {
	0x54, 0x65, 0x73, 0x74, 0x20, 0x55, 0x73, 0x69,
	0x6e, 0x67, 0x20, 0x4c, 0x61, 0x72, 0x67, 0x65,
	0x72, 0x20, 0x54, 0x68, 0x61, 0x6e, 0x20, 0x42,
	0x6c, 0x6f, 0x63, 0x6b, 0x2d, 0x53, 0x69, 0x7a,
	0x65, 0x20, 0x4b, 0x65, 0x79, 0x20, 0x2d, 0x20,
	0x48, 0x61, 0x73, 0x68, 0x20, 0x4b, 0x65, 0x79,
	0x20, 0x46, 0x69, 0x72, 0x73, 0x74 };

static const uint8_t msg54_k131_hmac_sha256[] = {
	0x60, 0xe4, 0x31, 0x59, 0x1e, 0xe0, 0xb6, 0x7f,
	0x0d, 0x8a, 0x26, 0xaa, 0xcb, 0xf5, 0xb7, 0x7f,
	0x8e, 0x0b, 0xc6, 0x21, 0x37, 0x28, 0xc5, 0x14,
	0x05, 0x46, 0x04, 0x0f, 0x0e, 0xe3, 0x7f, 0x54 };

static int hmac_test(void)
{
	uint8_t result[32] = {0};

	int ret = app_hmac_sha256_mesg(hmac_key4, sizeof(hmac_key4),
				       hmac_msg4, sizeof(hmac_msg4),
				       result, sizeof(result));
	if (ret) {
		return ret;
	}

	ret = memcmp(result, hmac_digest4, sizeof(hmac_digest4));
	if (ret) {
		return ret;
	}

	memset(result, 0, sizeof(result));
	ret = app_hmac_sha256_mesg(key131, sizeof(key131),
				   msg54, sizeof(msg54),
				   result, sizeof(result));
	if (ret) {
		return ret;
	}

	ret = memcmp(result, msg54_k131_hmac_sha256, sizeof(msg54_k131_hmac_sha256));

	return ret;
}

#ifdef CONFIG_HAS_MEC5_HAL

K_SEM_DEFINE(sem_rom_aesh, 1, 1);

static struct mchphmac2 hmac_ctx;
static uint32_t k0[64 / 4];
static uint8_t __aligned(4) hmac_state[128];

int app_crypto_init(void)
{
	memset(&hmac_ctx, 0, sizeof(hmac_ctx));

	mec_hal_rom_crypto_enable(1);

	int ret = mec_hal_rom_rng_init();

	if (ret != 0) {
		LOG_ERR("App unable to initialize HW RNG");
		return -EIO;
	}

	mec_hal_rom_rng_enable(true);

	return 0;
}

int app_get_rand(uint8_t *dest, size_t destsz, uint16_t req_nbytes)
{
	int ret = 0;
	uint16_t nr = 0;

	if (destsz < req_nbytes) {
		return -EINVAL;
	}

	if (!mec_hal_rom_rng_is_fifo_full()) {
		return -EBUSY;
	}

	if (!mec_hal_rom_rng_is_healthy()) {
		return -EIO;
	}

	ret = mec_hal_rom_rng_read_bytes_nh(dest, req_nbytes, &nr);

	if (ret || (nr < req_nbytes)) {
		return -EIO;
	}

	return 0;
}

/* Any of the three ROM HMAC2 calls could start the HW engine.
 * We must call ROM hash wait which returns immediately if the engine was not started.
 */
int app_hmac_sha256_mesg(const uint8_t *key, size_t keysz,
			 const uint8_t *msg, size_t msgsz,
			 uint8_t *result, size_t resultsz)
{
	int ret = 0;

	k_sem_take(&sem_rom_aesh, K_FOREVER);

	ret = mec_hal_rom_hmac2_init(MCHP_HASH_ALG_SHA256, &hmac_ctx, key, keysz, k0, sizeof(k0));
	if (ret != 0) {
		ret = -EIO;
		goto app_hmac_sha256_mesg_exit;
	}

	ret = mec_hal_rom_hash_wait(&hmac_ctx.c);
	if (ret != 0) {
		LOG_ERR("Hash wait1 unexpected return code: (%d)", ret);
		ret = -ETIMEDOUT;
		goto app_hmac_sha256_mesg_exit;
	}

	ret = mec_hal_rom_hmac2_add_data_block(&hmac_ctx, hmac_state, sizeof(hmac_state),
					       k0, sizeof(k0), msg, msgsz, true);
	if (ret != 0) {
		ret = -EIO;
		goto app_hmac_sha256_mesg_exit;
	}

	ret = mec_hal_rom_hash_wait(&hmac_ctx.c);
	if (ret != 0) {
		LOG_ERR("Hash wait2 unexpected return code: (%d)", ret);
		ret = -ETIMEDOUT;
		goto app_hmac_sha256_mesg_exit;
	}

	ret = mec_hal_rom_hmac2_final(&hmac_ctx, hmac_state, sizeof(hmac_state), k0, sizeof(k0),
				      result, resultsz);
	if (ret != 0) {
		ret = -EIO;
		goto app_hmac_sha256_mesg_exit;
	}

	ret = mec_hal_rom_hash_wait(&hmac_ctx.c);
	if (ret != 0) {
		LOG_ERR("Hash wait3 unexpected return code: (%d)", ret);
		ret = -ETIMEDOUT;
	}

app_hmac_sha256_mesg_exit:
	k_sem_give(&sem_rom_aesh);

	return ret;
}

int app_hmac_test(void)
{
	return hmac_test();
}

#elif CONFIG_TINYCRYPT_SHA256_HMAC

static const char tc_hprng0_ps[] = "TC HPRNG0 PS";
/* TinyCrypt HMAC PRNG minimum seed length is 32 bytes */
static const char tc_hprng0_seed[] = "TC SEED minimum Length is 32 bytes";
static const char tc_hprng0_addin[] = "TC ADDITIONAL INPUT0";

struct tc_hmac_prng_struct tc_hprng0;

int app_crypto_init(void)
{
	int ret = tc_hmac_prng_init(&tc_hprng0, tc_hprng0_ps, sizeof(tc_hprng0_ps));

	if (ret == TC_CRYPTO_SUCCESS) {
		return 0;
	}

	return -EIO;
}

int app_get_rand(uint8_t *dest, size_t destsz, size_t req_nbytes)
{
	int ret;

	if (destsz < req_nbytes) {
		return -EINVAL;
	}

	ret = tc_hmac_prng_generate(dest, (unsigned int)req_nbytes, &tc_hprng0);
	if (ret == TC_CRYPTO_SUCCESS) {
		ret = 0;
	} else if (ret == TC_HMAC_PRNG_RESEED_REQ) {
		ret = tc_hmac_prng_reseed(&tc_hprng0, tc_hprng0_seed, sizeof(tc_hprng0_seed),
					  tc_hprng0_addin, sizeof(tc_hprng0_addin));
		if (ret == TC_CRYPTO_SUCCESS) {
			ret = tc_hmac_prng_generate(dest, (unsigned int)req_nbytes, &tc_hprng0);
			if (ret == TC_CRYPTO_SUCCESS) {
				ret = 0;
			} else {
				ret = -EIO;
			}
		} else {
			ret = -EIO;
		}
	} else {
		ret = -EIO;
	}

	return ret;
}

int app_hmac_sha256_mesg(const uint8_t *key, size_t keysz,
			 const uint8_t *msg, size_t msgsz,
			 uint8_t *result, size_t resultsz)
{
	struct tc_hmac_state_struct hctx;
	int ret = tc_hmac_set_key(&hctx, key, keysz);

	if (ret != TC_CRYPTO_SUCCESS) {
		return -EINVAL;
	}

	ret = tc_hmac_init(&hctx);
	if (ret != TC_CRYPTO_SUCCESS) {
		return -EINVAL;
	}

	ret = tc_hmac_update(&hctx, msg, msgsz);
	if (ret != TC_CRYPTO_SUCCESS) {
		return -EINVAL;
	}

	ret = tc_hmac_final(result, resultsz, &hctx);
	if (ret != TC_CRYPTO_SUCCESS) {
		return -EINVAL;
	}

	return 0;
}

int app_hmac_test(void)
{
	return hmac_test();
}

#else

int app_crypto_init(void)
{
	return -ENOSYS;
}

int app_hmac_sha256_mesg(const uint8_t *key, size_t keysz,
			 const uint8_t *msg, size_t msgsz,
			 uint8_t *result, size_t resultsz)
{
	return -ENOSYS;
}

int app_hmac_test(void)
{
	return -ENOSYS;
}
#endif
