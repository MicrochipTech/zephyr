/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_MCHP_XEC_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_MCHP_XEC_H_

/**
 * @file header for Microchip XEC GPIO
 */

#include <drivers/gpio.h>

/* GPIO buses definitions */

#define MCHP_XEC_GPIO_PORT(pin) (((pin) >> 5) & 0x7)

/**
 * @brief helper for configuration of GPIO pin
 *
 * @param dev GPIO port device pointer
 * @param pin IO pin
 * @param conf GPIO mode
 * @param altf Alternate function
 *
 * @return 0 on success, negative errno code on failure
 */
int gpio_mchp_xec_configure(const struct device *dev, int pin,
			    int conf, int altf);

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_MCHP_XEC_H_ */
