/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_trng

#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <soc.h>
#include <string.h>
LOG_MODULE_REGISTER(entropy_mec5_trng, CONFIG_ENTROPY_LOG_LEVEL);

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_rom_api.h>

#define TRNG_MEC5_TIMEOUT_COUNT		200u
#define TRNG_MEC5_TIMEOUT_BUSY_WAIT_US	10u

struct trng_mec5_dev_cfg {
	uintptr_t regs;
};

/*
 * Fill the passed buffer with entropy.
 * Blocks if required in order to generate the necessary random data.
 *
 * dev = Pointer to the entropy device.
 * buffer = pointer to a buffer to fill with entropy.
 * length = Buffer length in bytes
 * return 0 on success or negative errno code.
 * Loop over length
 * Wait for FIFO to fill.
 * Check health of data in FIFO. If bad return -EIO.
 * Copy data from FIFO to buffer.
 */
static int entropy_mec5_trng_get_entropy(const struct device *dev, uint8_t *buffer,
					 uint16_t length)
{
	int ret = 0;
	uint32_t timeout = 0u;
	uint16_t nbytes = 0, nr = 0, ns = 0;

	if (!buffer) {
		return -EINVAL;
	}

	timeout = TRNG_MEC5_TIMEOUT_COUNT;
	nbytes = length;
	while (nbytes) {
		if (mec_hal_rom_rng_is_fifo_full()) {
			timeout = 200u;
			if (!mec_hal_rom_rng_is_healthy()) {
				return -EIO;
			}
			if (nbytes > MEC_RNG_FIFO_SIZE_BYTES) {
				nr = MEC_RNG_FIFO_SIZE_BYTES;
			} else {
				nr = nbytes;
			}
			ns = 0u;
			ret = mec_hal_rom_rng_read_bytes_nh(buffer, nr, &ns);
			if ((ret != MEC_RET_OK) || (ns > nr)) {
				return -EIO;
			}
			buffer += ns;
			nbytes -= ns;
			mec_hal_rom_rng_fifo_full_status_clear();
		} else {
			if (timeout-- == 0) {
				return -ETIMEDOUT;
			}
			k_busy_wait(TRNG_MEC5_TIMEOUT_BUSY_WAIT_US);
		}
	}

	return 0;
}

/* Fills a buffer with entropy in a non-blocking or busy-wait manner.
 * Callable from ISRs.
 *
 * dev = Pointer to the device structure.
 * buffer = pointer to a buffer to fill with entropy.
 * length = Buffer length in bytes
 * flags = Flags to modify the behavior of the call.
 * @retval number of bytes filled with entropy or -error.
 */
static int entropy_mec5_trng_get_entropy_isr(const struct device *dev,
					     uint8_t *buffer, uint16_t length,
					     uint32_t flags)
{
	int ret = 0;
	uint16_t nr = 0, ns = 0;
	uint16_t nbytes = length;

	if ((flags & ENTROPY_BUSYWAIT) == 0U) {
		while (nbytes) {
			if (!mec_hal_rom_rng_is_fifo_full()) {
				break;
			}
			if (!mec_hal_rom_rng_is_healthy()) {
				break;
			}
			nr = nbytes;
			if (nr > MEC_RNG_FIFO_SIZE_BYTES) {
				nr = MEC_RNG_FIFO_SIZE_BYTES;
			}
			ns = 0u;
			ret = mec_hal_rom_rng_read_bytes_nh(buffer, nr, &ns);
			if (ret != MEC_RET_OK) {
				break;
			}
			if (ns <= nbytes) {
				nbytes -= ns;
			} else {
				nbytes = 0u;
			}
			mec_hal_rom_rng_fifo_full_status_clear();
		}

		return (length - nbytes);
	} else { /* we can busy wait */
		ret = entropy_mec5_trng_get_entropy(dev, buffer, length);
		if (ret) {
			return ret;
		}

		return (int)length;
	}
}

static int entropy_mec5_trng_init(const struct device *dev)
{
	int ret = mec_hal_rom_rng_init();

	if (ret) {
		LOG_ERR("MEC5 entropy init err (%d)", ret);
	}

	mec_hal_rom_rng_enable(true);

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int entropy_mec5_trng_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		mec_hal_rom_rng_enable(false);
		break;
	case PM_DEVICE_ACTION_RESUME:
		mec_hal_rom_rng_enable(true);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct entropy_driver_api entropy_mec5_trng_api = {
	.get_entropy = entropy_mec5_trng_get_entropy,
	.get_entropy_isr = entropy_mec5_trng_get_entropy_isr
};

#define ENTROPY_MEC5_TRNG_INIT(n)					\
	static const struct trng_mec5_dev_cfg trng_mec5_cfg_##n = {	\
		.regs = (uintptr_t)DT_INST_REG_ADDR(0),			\
	};								\
									\
	PM_DEVICE_DT_INST_DEFINE(n, entropy_mec5_trng_pm_action);	\
									\
	DEVICE_DT_INST_DEFINE(n, &entropy_mec5_trng_init,		\
			 PM_DEVICE_DT_INST_GET(n),			\
			 NULL,						\
			 &trng_mec5_cfg_##n,				\
			 PRE_KERNEL_1,					\
			 CONFIG_ENTROPY_INIT_PRIORITY,			\
			 &entropy_mec5_trng_api);

DT_INST_FOREACH_STATUS_OKAY(ENTROPY_MEC5_TRNG_INIT)
