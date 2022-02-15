/*
 * Copyright (c) 2021 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <init.h>
#include <kernel.h>
#include <drivers/clock_control/mchp_xec_clock_control.h>
#include <drivers/pinmux.h>

#include <soc.h>

enum gpio_ports {
	port_000_036 = 0,
	port_040_076,
	port_100_136,
	port_140_176,
	port_200_236,
	port_240_276,
	port_max,
};

struct pin_info {
	enum gpio_ports port_num;
	uint8_t pin;
	uint32_t flags;
};

struct pinmux_ports_t {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_000_036), okay)
	const struct device *porta;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_040_076), okay)
	const struct device *portb;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_100_136), okay)
	const struct device *portc;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_140_176), okay)
	const struct device *portd;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_200_236), okay)
	const struct device *porte;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_240_276), okay)
	const struct device *portf;
#endif
};

/* kscan: KSCAN KSO & KSI */
const struct pin_info kscan_pin_table[] = {
#if defined(CONFIG_KSCAN_XEC) && DT_NODE_HAS_STATUS(DT_NODELABEL(kscan0), okay)
	{ port_040_076, MCHP_GPIO_040, MCHP_GPIO_CTRL_MUX_F2 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_040_076, MCHP_GPIO_045, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_040_076, MCHP_GPIO_046, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_100_136, MCHP_GPIO_125, MCHP_GPIO_CTRL_MUX_F2 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_100_136, MCHP_GPIO_126, MCHP_GPIO_CTRL_MUX_F2 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_040_076, MCHP_GPIO_047, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_100_136, MCHP_GPIO_107, MCHP_GPIO_CTRL_MUX_F2 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_100_136, MCHP_GPIO_112, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_100_136, MCHP_GPIO_113, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_140_176, MCHP_GPIO_152, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_140_176, MCHP_GPIO_151, MCHP_GPIO_CTRL_MUX_F2 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_100_136, MCHP_GPIO_120, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_100_136, MCHP_GPIO_121, MCHP_GPIO_CTRL_MUX_F2 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_100_136, MCHP_GPIO_122, MCHP_GPIO_CTRL_MUX_F2 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_100_136, MCHP_GPIO_123, MCHP_GPIO_CTRL_MUX_F2 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_100_136, MCHP_GPIO_124, MCHP_GPIO_CTRL_MUX_F2 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_040_076, MCHP_GPIO_017, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_040_076, MCHP_GPIO_020, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_040_076, MCHP_GPIO_021, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_040_076, MCHP_GPIO_026, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_040_076, MCHP_GPIO_027, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_040_076, MCHP_GPIO_030, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_040_076, MCHP_GPIO_031, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
	{ port_040_076, MCHP_GPIO_032, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_PUD_PU },
#endif
};

const struct pin_info spi0_pin_table[] = {
#if defined(CONFIG_SPI_XEC_QMSPI_LDMA) && DT_NODE_HAS_STATUS(DT_NODELABEL(spi0), okay)
#if DT_PROP(DT_NODELABEL(spi0), port_sel) == 0
#if DT_PROP(DT_NODELABEL(spi0), chip_sel) == 0
	{ port_040_076, MCHP_GPIO_055, MCHP_GPIO_CTRL_MUX_F2 | MCHP_GPIO_CTRL_BUFT_OPENDRAIN },
#endif
#if DT_PROP(DT_NODELABEL(spi0), chip_sel) == 1
	{ port_000_036, MCHP_GPIO_002, MCHP_GPIO_CTRL_MUX_F2 | MCHP_GPIO_CTRL_BUFT_OPENDRAIN },
#endif
	{ port_040_076, MCHP_GPIO_056, MCHP_GPIO_CTRL_MUX_F2 },
	{ port_200_236, MCHP_GPIO_223, MCHP_GPIO_CTRL_MUX_F1 },
	{ port_200_236, MCHP_GPIO_224, MCHP_GPIO_CTRL_MUX_F2 },
#if DT_PROP(DT_NODELABEL(spi0), lines) == 4
	{ port_200_236, MCHP_GPIO_227, MCHP_GPIO_CTRL_MUX_F1 },
	{ port_000_036, MCHP_GPIO_016, MCHP_GPIO_CTRL_MUX_F2 },
#endif
#elif DT_PROP(DT_NODELABEL(spi0), port_sel) == 1
	{ port_100_136, MCHP_GPIO_124, MCHP_GPIO_CTRL_MUX_F1 | MCHP_GPIO_CTRL_BUFT_OPENDRAIN },
	{ port_100_136, MCHP_GPIO_125, MCHP_GPIO_CTRL_MUX_F1 },
	{ port_100_136, MCHP_GPIO_121, MCHP_GPIO_CTRL_MUX_F1 },
	{ port_100_136, MCHP_GPIO_122, MCHP_GPIO_CTRL_MUX_F1 },
#if DT_PROP(DT_NODELABEL(spi0), lines) == 4
	{ port_100_136, MCHP_GPIO_123, MCHP_GPIO_CTRL_MUX_F1 },
	{ port_100_136, MCHP_GPIO_126, MCHP_GPIO_CTRL_MUX_F1 },
#endif
#elif DT_PROP(DT_NODELABEL(spi0), port_sel) == 2
	{ port_100_136, MCHP_GPIO_116, MCHP_GPIO_CTRL_MUX_F1 },
	{ port_100_136, MCHP_GPIO_117, MCHP_GPIO_CTRL_MUX_F1 },
	{ port_040_076, MCHP_GPIO_074, MCHP_GPIO_CTRL_MUX_F1 },
	{ port_040_076, MCHP_GPIO_075, MCHP_GPIO_CTRL_MUX_F1 },
	{ port_040_076, MCHP_GPIO_076, MCHP_GPIO_CTRL_MUX_F0 },
#endif
#endif
};

static void brd_init_pinmux_ports(struct pinmux_ports_t *pp)
{
	ARG_UNUSED(pp);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_000_036), okay)
	pp->porta = DEVICE_DT_GET(DT_NODELABEL(pinmux_000_036));

	__ASSERT_NO_MSG(device_is_ready(pp->porta));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_040_076), okay)
	pp->portb = DEVICE_DT_GET(DT_NODELABEL(pinmux_040_076));

	__ASSERT_NO_MSG(device_is_ready(pp->portb));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_100_136), okay)
	pp->portc = DEVICE_DT_GET(DT_NODELABEL(pinmux_100_136));

	__ASSERT_NO_MSG(device_is_ready(pp->portc));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_140_176), okay)
	pp->portd = DEVICE_DT_GET(DT_NODELABEL(pinmux_140_176));

	__ASSERT_NO_MSG(device_is_ready(pp->portd));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_200_236), okay)
	pp->porte = DEVICE_DT_GET(DT_NODELABEL(pinmux_200_236));

	__ASSERT_NO_MSG(device_is_ready(pp->porte));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_240_276), okay)
	pp->portf = DEVICE_DT_GET(DT_NODELABEL(pinmux_240_276));

	__ASSERT_NO_MSG(device_is_ready(pp->portf));
#endif
}

const struct device *get_port_device(struct pinmux_ports_t *pp,
				     uint8_t port_num)
{
	switch (port_num) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_000_036), okay)
	case port_000_036:
		return pp->porta;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_040_076), okay)
	case port_040_076:
		return pp->portb;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_100_136), okay)
	case port_100_136:
		return pp->portc;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_140_176), okay)
	case port_140_176:
		return pp->portd;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_200_236), okay)
	case port_200_236:
		return pp->porte;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmux_240_276), okay)
	case port_240_276:
		return pp->portf;
#endif
	default:
		return NULL;
	}
}

static void brd_pin_table_init(struct pinmux_ports_t *pp,
			       const struct pin_info *table, size_t nentries)
{
	for (size_t n = 0; n < nentries; n++) {
		const struct device *dev =
			get_port_device(pp, table[n].port_num);

		if (!dev) {
			continue;
		}

		pinmux_pin_set(dev, table[n].pin, table[n].flags);
	}
}

/* caller passes dev = NULL */
static int board_pinmux_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	struct pinmux_ports_t pp;

	brd_init_pinmux_ports(&pp);
	brd_pin_table_init(&pp, kscan_pin_table, ARRAY_SIZE(kscan_pin_table));
	brd_pin_table_init(&pp, spi0_pin_table, ARRAY_SIZE(spi0_pin_table));

	return 0;
}

SYS_INIT(board_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
