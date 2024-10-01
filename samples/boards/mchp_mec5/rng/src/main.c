/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2022 Microchip Technology, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <soc.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/ps2.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_rom_api.h>

#define TRNG0_NODE DT_NODELABEL(trng0)

static const struct device *trng0_dev = DEVICE_DT_GET(TRNG0_NODE);

static volatile uint32_t spin_val;
static volatile int ret_val;

static void spin_on(uint32_t id, int rval);

#define RAND_BUF0_LEN 256u
static uint8_t rand_buf0[RAND_BUF0_LEN];

#define RAND_BUF1_LEN 256u
static uint32_t rand_buf1[RAND_BUF1_LEN / 4u];

int main(void)
{
	int ret = 0;
	uint8_t *p8 = NULL;
	uint32_t rand_flags = 0, loops = 0, nw = 0;
	uint16_t n = 0, nr = 0, nrand_bytes = 0, req = 0;

	LOG_INF("MEC5 RNG sample: board: %s", DT_N_P_compatible_IDX_0);

	memset(rand_buf0, 0x55, sizeof(rand_buf0));

	if (!device_is_ready(trng0_dev)) {
		LOG_ERR("TRNG0 device is not ready!");
		spin_on((uint32_t)__LINE__, -1000);
	}

	/* Based on prj.conf settings eventually calls entropy driver get_entropy API */
	nrand_bytes = 132u;
	sys_rand_get(rand_buf0, nrand_bytes);

	LOG_HEXDUMP_INF(rand_buf0, nrand_bytes, "sys_rand_get: 132 bytes: ");

	/* Test entropy driver get_entropy_isr using busy wait flag */
	LOG_INF("Test Get entropy ISR API with Busy Wait flag");
	memset(rand_buf0, 0x55, sizeof(rand_buf0));
	rand_flags = ENTROPY_BUSYWAIT;
	nrand_bytes = 164u;
	ret = entropy_get_entropy_isr(trng0_dev, rand_buf0, nrand_bytes, rand_flags);
	if (ret >= 0) {
		LOG_INF("Get entropy ISR API flags = 0x%0x: requested %u bytes,"
			" returned %d bytes", rand_flags, nrand_bytes, ret);
		LOG_HEXDUMP_INF(rand_buf0, nrand_bytes, "random bytes: ");
	} else {
		LOG_ERR("Get entropy ISR API flags = 0x%0x: error (%d)", rand_flags, ret);
	}

	/* trigger PM low power state entry */
	k_sleep(K_MSEC(8000));

	LOG_INF("Test Get entropy ISR API without Busy Wait flag");
	memset(rand_buf0, 0x55, sizeof(rand_buf0));
	p8 = rand_buf0;
	loops = 0u;
	rand_flags = 0u;
	n = 0u;
	nr = 0u;
	nrand_bytes = 164u;

	while (n < nrand_bytes) {
		loops++;
		req = nrand_bytes - n;
		ret = entropy_get_entropy_isr(trng0_dev, p8, req, rand_flags);
		if (ret >= 0) {
			nr = (uint16_t)(ret & 0xffffu);
			n += nr;
			p8 += nr;
			LOG_INF("Get entropy ISR API flags = 0x%0x: requested %u bytes,"
				" returned %u bytes", rand_flags, req, nr);

		} else {
			LOG_ERR("Get entropy ISR API flags = 0x%0x: error (%d)", rand_flags, ret);
		}
	}

	LOG_INF("Requested %u bytes. Required %u loops", nrand_bytes, loops);
	LOG_HEXDUMP_INF(rand_buf0, nrand_bytes, "random bytes: ");

	/* trigger PM low power state entry */
	k_sleep(K_MSEC(9000));

	LOG_INF("\nTest HAL RNG get number of words currently in the RNG FIFO");
	nw = mec_hal_rom_rng_fifo_level();
	LOG_INF("API reports RNG FIFO contains %u 32-bit words", nw);

	LOG_INF("Test clearing FIFO full status");
	mec_hal_rom_rng_fifo_full_status_clear();

	LOG_INF("Test HAL RNG get random words");
	memset(rand_buf1, 0x55, sizeof(rand_buf1));
	nrand_bytes = 8u * 4u;
	n = 0u;
	ret = mec_hal_rom_rng_get_rand_words(rand_buf1, nrand_bytes / 4u, &n);
	if (ret == MEC_RET_OK) {
		LOG_INF("HAL RNG get random words: 8 words returned %d", ret);
		LOG_HEXDUMP_INF(rand_buf1, nrand_bytes, "random bytes: ");
	} else {
		LOG_ERR("HAL RNG get randome words: 8 words returned error (%d)", ret);
	}

	LOG_INF("Application Done (%d)", ret);
	spin_on((uint32_t)__LINE__, 0);

	return 0;
}

static void spin_on(uint32_t id, int rval)
{
	spin_val = id;
	ret_val = rval;

	log_panic(); /* flush log buffers */

	while (spin_val) {
		;
	}
}
