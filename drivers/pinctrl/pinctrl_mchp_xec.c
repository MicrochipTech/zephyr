/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2021 Linaro Limited
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * Copyright (c) 2021 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/pinctrl.h>
#include <gpio/gpio_mchp_xec.h>
#include <soc.h>

/**
 * @brief Array containing pointers to each GPIO port.
 *
 * Entries will be NULL if the GPIO port is not enabled.
 */
static const struct device * const gpio_ports[] = {
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio_000_036)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio_040_076)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio_100_136)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio_140_176)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio_200_236)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio_240_276)),
};

/** Number of GPIO ports. */
static const size_t gpio_ports_cnt = ARRAY_SIZE(gpio_ports);


/*
 * uint32_t pin = encoded port and pin number
 * uint32_t cfg = property encodings: pulls, buffer type, slew, drive strength
 * uint32_t altf = function number, 0 = GPIO, > 0 alternate function
 */
static int mchp_xec_pin_configure(uint32_t portpin, uint32_t cfg,
				  uint32_t altf)
{
	const struct device *port_device;
	uint32_t port = MCHP_XEC_PINMUX_PORT(portpin);
	int pin = MCHP_XEC_PINMUX_PIN(portpin);

	if (port >= gpio_ports_cnt) {
		return -EINVAL;
	}

	port_device = gpio_ports[port];

	if ((port_device == NULL) || (!device_is_ready(port_device))) {
		return -ENODEV;
	}

	return gpio_mchp_xec_configure(port_device, pin, cfg, altf);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	uint32_t portpin, mux, cfg, func;
	int ret;

	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		mux = pins[i].pinmux;

		func = MCHP_XEC_PINMUX_FUNC(mux);
		if (func >= AFMAX) {
			__ASSERT_NO_MSG(func);
		}

		cfg = pins[i].pincfg;
		portpin = MEC_XEC_PINMUX_PORT_PIN(mux);

		ret = mchp_xec_pin_configure(portpin, cfg, func);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}
