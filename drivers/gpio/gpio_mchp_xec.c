/*
 * Copyright (c) 2019 Intel Corporation
 * Copyright (c) 2021 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_gpio

#include <errno.h>
#include <device.h>
#include <drivers/gpio.h>
#include <soc.h>

#include "gpio_utils.h"

#define XEC_GPIO_EDGE_DLY_COUNT		8
/* read only register for dummy writes */
#define XEC_GPIO_DLY_ADDR		0x40080150u

static const uint32_t valid_ctrl_masks[NUM_MCHP_GPIO_PORTS] = {
	(MCHP_GPIO_PORT_A_BITMAP),
	(MCHP_GPIO_PORT_B_BITMAP),
	(MCHP_GPIO_PORT_C_BITMAP),
	(MCHP_GPIO_PORT_D_BITMAP),
	(MCHP_GPIO_PORT_E_BITMAP),
	(MCHP_GPIO_PORT_F_BITMAP)
};

struct gpio_xec_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* port ISR callback routine address */
	sys_slist_t callbacks;
};

struct gpio_xec_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	uintptr_t pcr1_base;
	uintptr_t parin_addr;
	uintptr_t parout_addr;
	uint8_t girq_id;
	uint32_t port_num;
	uint32_t flags;
};

/* Each GPIO pin 32-bit control register located consecutively in memory */
static inline uintptr_t pin_ctrl_addr(const struct device *dev, gpio_pin_t pin)
{
	const struct gpio_xec_config *config = dev->config;

	return config->pcr1_base + ((uintptr_t)pin * 4u);
}

/* GPIO Parallel input is a single 32-bit register per bank of 32 pins */
static inline uintptr_t pin_parin_addr(const struct device *dev)
{
	const struct gpio_xec_config *config = dev->config;

	return config->parin_addr;
}

/* GPIO Parallel output is a single 32-bit register per bank of 32 pins */
static inline uintptr_t pin_parout_addr(const struct device *dev)
{
	const struct gpio_xec_config *config = dev->config;

	return config->parout_addr;
}

/*
 * Use Zephyr system API to implement
 * reg32(addr) = (reg32(addr) & ~mask) | (val & mask)
 */
static inline void xec_mask_write32(uintptr_t addr, uint32_t mask, uint32_t val)
{
	uint32_t r = (sys_read32(addr) & ~mask) | (val & mask);

	sys_write32(r, addr);
}

/*
 * notes: The GPIO parallel output bits are read-only until the
 * Alternate-Output-Disable (AOD) bit is set in the pin's control
 * register. To preload a parallel output value to prevent certain
 * classes of glitching for output pins we must:
 * Set GPIO control AOD=1 with the pin direction set to input.
 * Program the new pin value in the respective GPIO parallel output
 * register.
 * Program other GPIO control bits except direction.
 * Last step set the GPIO control register direction bit to output.
 */
static int gpio_xec_configure(const struct device *dev,
			      gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_xec_config *config = dev->config;
	uintptr_t pcr1_addr = pin_ctrl_addr(dev, pin);
	uintptr_t pout_addr = pin_parout_addr(dev);
	uint32_t pcr1 = 0U;
	uint32_t mask = 0U;

	/* Validate pin number range in terms of current port */
	if ((valid_ctrl_masks[config->port_num] & BIT(pin)) == 0U) {
		return -EINVAL;
	}

	/* Don't support "open source" mode */
	if (((flags & GPIO_SINGLE_ENDED) != 0U) &&
	    ((flags & GPIO_LINE_OPEN_DRAIN) == 0U)) {
		return -ENOTSUP;
	}

	/* The flags contain options that require touching registers in the
	 * PCRs for a given GPIO. There are no GPIO modules in Microchip SOCs!
	 * Keep direction as input until last.
	 * Clear input pad disable allowing input pad to operate.
	 * Clear Power gate to allow pads to operate.
	 */
	mask |= MCHP_GPIO_CTRL_DIR_MASK;
	mask |= MCHP_GPIO_CTRL_INPAD_DIS_MASK;
	mask |= MCHP_GPIO_CTRL_PWRG_MASK;
	pcr1 |= MCHP_GPIO_CTRL_DIR_INPUT;

	/* Figure out the pullup/pulldown configuration and keep it in the
	 * pcr1 variable
	 */
	mask |= MCHP_GPIO_CTRL_PUD_MASK;

	if ((flags & GPIO_PULL_UP) != 0U) {
		/* Enable the pull and select the pullup resistor. */
		pcr1 |= MCHP_GPIO_CTRL_PUD_PU;
	} else if ((flags & GPIO_PULL_DOWN) != 0U) {
		/* Enable the pull and select the pulldown resistor */
		pcr1 |= MCHP_GPIO_CTRL_PUD_PD;
	}

	/* Push-pull or open drain */
	mask |= MCHP_GPIO_CTRL_BUFT_MASK;

	if ((flags & GPIO_OPEN_DRAIN) != 0U) {
		/* Open drain */
		pcr1 |= MCHP_GPIO_CTRL_BUFT_OPENDRAIN;
	} else {
		/* Push-pull */
		pcr1 |= MCHP_GPIO_CTRL_BUFT_PUSHPULL;
	}

	/* Use GPIO output register to control pin output, instead of
	 * using the control register (=> alternate output disable).
	 */
	mask |= MCHP_GPIO_CTRL_AOD_MASK;
	pcr1 |= MCHP_GPIO_CTRL_AOD_DIS;

	/* Make sure disconnected on first control register write */
	if (flags == GPIO_DISCONNECTED) {
		pcr1 |= MCHP_GPIO_CTRL_PWRG_OFF;
	}

	/* Now write contents of pcr1 variable to the PCR1 register that
	 * corresponds to the GPIO being configured.
	 * AOD is 1 and direction is input. HW will allow use to set the
	 * GPIO parallel output bit for this pin and with the pin direction
	 * as input no glitch will occur.
	 */
	xec_mask_write32(pcr1_addr, mask, pcr1);

	if ((flags & GPIO_OUTPUT) != 0U) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			sys_set_bit(pout_addr, pin);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			sys_clear_bit(pout_addr, pin);
		}

		mask = MCHP_GPIO_CTRL_DIR_MASK;
		pcr1 = MCHP_GPIO_CTRL_DIR_OUTPUT;
		xec_mask_write32(pcr1_addr, mask, pcr1);
	}

	return 0;
}

static int gen_gpio_ctrl_icfg(enum gpio_int_mode mode, enum gpio_int_trig trig,
			      uint32_t *pin_ctr1)
{
	if (!pin_ctr1) {
		return -EINVAL;
	}

	if (mode == GPIO_INT_MODE_DISABLED) {
		*pin_ctr1 = MCHP_GPIO_CTRL_IDET_DISABLE;
	} else {
		if (mode == GPIO_INT_MODE_LEVEL) {
			if (trig == GPIO_INT_TRIG_HIGH) {
				*pin_ctr1 = MCHP_GPIO_CTRL_IDET_LVL_HI;
			} else {
				*pin_ctr1 = MCHP_GPIO_CTRL_IDET_LVL_LO;
			}
		} else {
			switch (trig) {
			case GPIO_INT_TRIG_LOW:
				*pin_ctr1 = MCHP_GPIO_CTRL_IDET_FEDGE;
				break;
			case GPIO_INT_TRIG_HIGH:
				*pin_ctr1 = MCHP_GPIO_CTRL_IDET_REDGE;
				break;
			case GPIO_INT_TRIG_BOTH:
				*pin_ctr1 = MCHP_GPIO_CTRL_IDET_BEDGE;
				break;
			default:
				*pin_ctr1 = MCHP_GPIO_CTRL_IDET_DISABLE;
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int gpio_xec_pin_interrupt_configure(const struct device *dev,
					    gpio_pin_t pin,
					    enum gpio_int_mode mode,
					    enum gpio_int_trig trig)
{
	const struct gpio_xec_config *config = dev->config;
	uintptr_t pcr1_addr = pin_ctrl_addr(dev, pin);
	uint32_t pcr1 = 0u;
	uint32_t pcr1_req = 0u;
	uint32_t mask = 0u;

	/* Validate pin number range in terms of current port */
	if ((valid_ctrl_masks[config->port_num] & BIT(pin)) == 0U) {
		return -EINVAL;
	}

	/* Check if GPIO port supports interrupts */
	if ((mode != GPIO_INT_MODE_DISABLED) &&
	    ((config->flags & GPIO_INT_ENABLE) == 0U)) {
		return -ENOTSUP;
	}

	pcr1_req = MCHP_GPIO_CTRL_IDET_DISABLE;
	if (gen_gpio_ctrl_icfg(mode, trig, &pcr1_req)) {
		return -EINVAL;
	}

	/* Disable interrupt in the EC aggregator */
	mchp_soc_ecia_girq_src_dis(config->girq_id, pin);

	/* pin configuration matches requested detection mode? */
	pcr1 = sys_read32(pcr1_addr);

	if ((pcr1 & MCHP_GPIO_CTRL_IDET_MASK) == pcr1_req) {
		goto gp_config_girq;
	}

	pcr1 &= ~MCHP_GPIO_CTRL_IDET_MASK;

	if (mode == GPIO_INT_MODE_LEVEL) {
		if (trig == GPIO_INT_TRIG_HIGH) {
			pcr1 |= MCHP_GPIO_CTRL_IDET_LVL_HI;
		} else {
			pcr1 |= MCHP_GPIO_CTRL_IDET_LVL_LO;
		}
		sys_write32(pcr1, pcr1_addr);
	} else if (mode == GPIO_INT_MODE_EDGE) {
		if (trig == GPIO_INT_TRIG_LOW) {
			pcr1 |= MCHP_GPIO_CTRL_IDET_FEDGE;
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			pcr1 |= MCHP_GPIO_CTRL_IDET_REDGE;
		} else if (trig == GPIO_INT_TRIG_BOTH) {
			pcr1 |= MCHP_GPIO_CTRL_IDET_BEDGE;
		}
		sys_write32(pcr1, pcr1_addr);
		mask = 0u;
		/* HW takes several clocks to stabilize the first time
		 * edge detect is enabled.
		 */
		for (int i = 0; i < XEC_GPIO_EDGE_DLY_COUNT; i++) {
			mask |= sys_read32(pcr1_addr);
		}
		/* trick the compiler by using mask. write to read-only */
		sys_write32(mask, XEC_GPIO_DLY_ADDR);
	} else {
		pcr1 |= MCHP_GPIO_CTRL_IDET_DISABLE;
		sys_write32(pcr1, pcr1_addr);
	}

	mchp_soc_ecia_girq_src_clr(config->girq_id, pin);

gp_config_girq:
	if (mode != GPIO_INT_MODE_DISABLED) {
		/* Enable interrupt to propagate via its GIRQ to the NVIC */
		mchp_soc_ecia_girq_src_en(config->girq_id, pin);
	}

	return 0;
}

static int gpio_xec_port_set_masked_raw(const struct device *dev,
					uint32_t mask,
					uint32_t value)
{
	uintptr_t pout_addr = pin_parout_addr(dev);

	xec_mask_write32(pout_addr, mask, value);

	return 0;
}

static int gpio_xec_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	uintptr_t pout_addr = pin_parout_addr(dev);

	sys_write32(sys_read32(pout_addr) | mask, pout_addr);

	return 0;
}

static int gpio_xec_port_clear_bits_raw(const struct device *dev,
					uint32_t mask)
{
	uintptr_t pout_addr = pin_parout_addr(dev);

	sys_write32(sys_read32(pout_addr) & ~mask, pout_addr);

	return 0;
}

static int gpio_xec_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	uintptr_t pout_addr = pin_parout_addr(dev);

	sys_write32(sys_read32(pout_addr) ^ mask, pout_addr);

	return 0;
}

static int gpio_xec_port_get_raw(const struct device *dev, uint32_t *value)
{
	uintptr_t pin_addr = pin_parin_addr(dev);

	*value = sys_read32(pin_addr);

	return 0;
}

static int gpio_xec_manage_callback(const struct device *dev,
				    struct gpio_callback *callback, bool set)
{
	struct gpio_xec_data *data = dev->data;

	gpio_manage_callback(&data->callbacks, callback, set);

	return 0;
}

static void gpio_gpio_xec_port_isr(const struct device *dev)
{
	const struct gpio_xec_config *config = dev->config;
	struct gpio_xec_data *data = dev->data;
	uint32_t girq_result;

	/* Figure out which interrupts have been triggered from the EC
	 * aggregator result register
	 */
	girq_result = mchp_soc_ecia_girq_result(config->girq_id);

	/* Clear source register in aggregator before firing callbacks */
	mchp_soc_ecia_girq_src_clr_bitmap(config->girq_id, girq_result);

	gpio_fire_callbacks(&data->callbacks, dev, girq_result);
}

static const struct gpio_driver_api gpio_xec_driver_api = {
	.pin_configure = gpio_xec_configure,
	.port_get_raw = gpio_xec_port_get_raw,
	.port_set_masked_raw = gpio_xec_port_set_masked_raw,
	.port_set_bits_raw = gpio_xec_port_set_bits_raw,
	.port_clear_bits_raw = gpio_xec_port_clear_bits_raw,
	.port_toggle_bits = gpio_xec_port_toggle_bits,
	.pin_interrupt_configure = gpio_xec_pin_interrupt_configure,
	.manage_callback = gpio_xec_manage_callback,
};

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio_000_036), okay)
static int gpio_xec_port000_036_init(const struct device *dev);

static const struct gpio_xec_config gpio_xec_port000_036_config = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_NODE(
			DT_NODELABEL(gpio_000_036)),
	},
	.pcr1_base =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_000_036), 0),
	.parin_addr =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_000_036), 1),
	.parout_addr =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_000_036), 2),
	.port_num = MCHP_GPIO_000_036,
#if DT_IRQ_HAS_CELL(DT_NODELABEL(gpio_000_036), irq)
	.girq_id = DT_PROP(DT_NODELABEL(gpio_000_036), girq_id),
	.flags = GPIO_INT_ENABLE,
#else
	.flags = 0,
#endif
};

static struct gpio_xec_data gpio_xec_port000_036_data;

DEVICE_DT_DEFINE(DT_NODELABEL(gpio_000_036),
		    gpio_xec_port000_036_init,
		    NULL,
		    &gpio_xec_port000_036_data, &gpio_xec_port000_036_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_xec_driver_api);

static int gpio_xec_port000_036_init(const struct device *dev)
{
#if DT_IRQ_HAS_CELL(DT_NODELABEL(gpio_000_036), irq)
	const struct gpio_xec_config *config = dev->config;

	/* Turn on the block enable in the EC aggregator */
	mchp_soc_ecia_girq_aggr_en(config->girq_id, 1);

	IRQ_CONNECT(DT_IRQ(DT_NODELABEL(gpio_000_036), irq),
		    DT_IRQ(DT_NODELABEL(gpio_000_036), priority),
		    gpio_gpio_xec_port_isr,
		    DEVICE_DT_GET(DT_NODELABEL(gpio_000_036)), 0U);

	irq_enable(DT_IRQ(DT_NODELABEL(gpio_000_036), irq));
#endif
	return 0;
}
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio_000_036), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio_040_076), okay)
static int gpio_xec_port040_076_init(const struct device *dev);

static const struct gpio_xec_config gpio_xec_port040_076_config = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_NODE(
			DT_NODELABEL(gpio_040_076)),
	},
	.pcr1_base =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_040_076), 0),
	.parin_addr =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_040_076), 1),
	.parout_addr =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_040_076), 2),
	.port_num = MCHP_GPIO_040_076,
#if DT_IRQ_HAS_CELL(DT_NODELABEL(gpio_040_076), irq)
	.girq_id = DT_PROP(DT_NODELABEL(gpio_040_076), girq_id),
	.flags = GPIO_INT_ENABLE,
#else
	.flags = 0,
#endif
};

static struct gpio_xec_data gpio_xec_port040_076_data;

DEVICE_DT_DEFINE(DT_NODELABEL(gpio_040_076),
		    gpio_xec_port040_076_init,
		    NULL,
		    &gpio_xec_port040_076_data, &gpio_xec_port040_076_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_xec_driver_api);

static int gpio_xec_port040_076_init(const struct device *dev)
{
#if DT_IRQ_HAS_CELL(DT_NODELABEL(gpio_040_076), irq)
	const struct gpio_xec_config *config = dev->config;

	/* Turn on the block enable in the EC aggregator */
	mchp_soc_ecia_girq_aggr_en(config->girq_id, 1);

	IRQ_CONNECT(DT_IRQ(DT_NODELABEL(gpio_040_076), irq),
		    DT_IRQ(DT_NODELABEL(gpio_040_076), priority),
		    gpio_gpio_xec_port_isr,
		    DEVICE_DT_GET(DT_NODELABEL(gpio_040_076)), 0U);

	irq_enable(DT_IRQ(DT_NODELABEL(gpio_040_076), irq));
#endif
	return 0;
}
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio_040_076), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio_100_136), okay)
static int gpio_xec_port100_136_init(const struct device *dev);

static const struct gpio_xec_config gpio_xec_port100_136_config = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_NODE(
			DT_NODELABEL(gpio_100_136)),
	},
	.pcr1_base =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_100_136), 0),
	.parin_addr =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_100_136), 1),
	.parout_addr =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_100_136), 2),
	.port_num = MCHP_GPIO_100_136,
#if DT_IRQ_HAS_CELL(DT_NODELABEL(gpio_100_136), irq)
	.girq_id = DT_PROP(DT_NODELABEL(gpio_100_136), girq_id),
	.flags = GPIO_INT_ENABLE,
#else
	.flags = 0,
#endif
};

static struct gpio_xec_data gpio_xec_port100_136_data;

DEVICE_DT_DEFINE(DT_NODELABEL(gpio_100_136),
		    gpio_xec_port100_136_init,
		    NULL,
		    &gpio_xec_port100_136_data, &gpio_xec_port100_136_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_xec_driver_api);

static int gpio_xec_port100_136_init(const struct device *dev)
{
#if DT_IRQ_HAS_CELL(DT_NODELABEL(gpio_100_136), irq)
	const struct gpio_xec_config *config = dev->config;

	/* Turn on the block enable in the EC aggregator */
	mchp_soc_ecia_girq_aggr_en(config->girq_id, 1);

	IRQ_CONNECT(DT_IRQ(DT_NODELABEL(gpio_100_136), irq),
		    DT_IRQ(DT_NODELABEL(gpio_100_136), priority),
		    gpio_gpio_xec_port_isr,
		    DEVICE_DT_GET(DT_NODELABEL(gpio_100_136)), 0U);

	irq_enable(DT_IRQ(DT_NODELABEL(gpio_100_136), irq));
#endif
	return 0;
}
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio_100_136), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio_140_176), okay)
static int gpio_xec_port140_176_init(const struct device *dev);

static const struct gpio_xec_config gpio_xec_port140_176_config = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_NODE(
			DT_NODELABEL(gpio_140_176)),
	},
	.pcr1_base =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_140_176), 0),
	.parin_addr =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_140_176), 1),
	.parout_addr =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_140_176), 2),
	.port_num = MCHP_GPIO_140_176,
#if DT_IRQ_HAS_CELL(DT_NODELABEL(gpio_140_176), irq)
	.girq_id = DT_PROP(DT_NODELABEL(gpio_140_176), girq_id),
	.flags = GPIO_INT_ENABLE,
#else
	.flags = 0,
#endif
};

static struct gpio_xec_data gpio_xec_port140_176_data;

DEVICE_DT_DEFINE(DT_NODELABEL(gpio_140_176),
		    gpio_xec_port140_176_init,
		    NULL,
		    &gpio_xec_port140_176_data, &gpio_xec_port140_176_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_xec_driver_api);

static int gpio_xec_port140_176_init(const struct device *dev)
{
#if DT_IRQ_HAS_CELL(DT_NODELABEL(gpio_140_176), irq)
	const struct gpio_xec_config *config = dev->config;

	/* Turn on the block enable in the EC aggregator */
	mchp_soc_ecia_girq_aggr_en(config->girq_id, 1);

	IRQ_CONNECT(DT_IRQ(DT_NODELABEL(gpio_140_176), irq),
		    DT_IRQ(DT_NODELABEL(gpio_140_176), priority),
		    gpio_gpio_xec_port_isr,
		    DEVICE_DT_GET(DT_NODELABEL(gpio_140_176)), 0U);

	irq_enable(DT_IRQ(DT_NODELABEL(gpio_140_176), irq));
#endif
	return 0;
}
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio_140_176), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio_200_236), okay)
static int gpio_xec_port200_236_init(const struct device *dev);

static const struct gpio_xec_config gpio_xec_port200_236_config = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_NODE(
			DT_NODELABEL(gpio_200_236)),
	},
	.pcr1_base =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_200_236), 0),
	.parin_addr =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_200_236), 1),
	.parout_addr =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_200_236), 2),
	.port_num = MCHP_GPIO_200_236,
#if DT_IRQ_HAS_CELL(DT_NODELABEL(gpio_200_236), irq)
	.girq_id = DT_PROP(DT_NODELABEL(gpio_200_236), girq_id),
	.flags = GPIO_INT_ENABLE,
#else
	.flags = 0,
#endif
};

static struct gpio_xec_data gpio_xec_port200_236_data;

DEVICE_DT_DEFINE(DT_NODELABEL(gpio_200_236),
		    gpio_xec_port200_236_init,
		    NULL,
		    &gpio_xec_port200_236_data, &gpio_xec_port200_236_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_xec_driver_api);

static int gpio_xec_port200_236_init(const struct device *dev)
{
#if DT_IRQ_HAS_CELL(DT_NODELABEL(gpio_200_236), irq)
	const struct gpio_xec_config *config = dev->config;

	/* Turn on the block enable in the EC aggregator */
	mchp_soc_ecia_girq_aggr_en(config->girq_id, 1);

	IRQ_CONNECT(DT_IRQ(DT_NODELABEL(gpio_200_236), irq),
		    DT_IRQ(DT_NODELABEL(gpio_200_236), priority),
		    gpio_gpio_xec_port_isr,
		    DEVICE_DT_GET(DT_NODELABEL(gpio_200_236)), 0U);

	irq_enable(DT_IRQ(DT_NODELABEL(gpio_200_236), irq));
#endif
	return 0;
}
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio_200_236), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio_240_276), okay)
static int gpio_xec_port240_276_init(const struct device *dev);

static const struct gpio_xec_config gpio_xec_port240_276_config = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_NODE(
			DT_NODELABEL(gpio_240_276)),
	},
	.pcr1_base =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_240_276), 0),
	.parin_addr =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_240_276), 1),
	.parout_addr =
		(uintptr_t) DT_REG_ADDR_BY_IDX(DT_NODELABEL(gpio_240_276), 2),
	.port_num = MCHP_GPIO_240_276,
#if DT_IRQ_HAS_CELL(DT_NODELABEL(gpio_240_276), irq)
	.girq_id = DT_PROP(DT_NODELABEL(gpio_240_276), girq_id),
	.flags = GPIO_INT_ENABLE,
#else
	.flags = 0,
#endif
};

static struct gpio_xec_data gpio_xec_port240_276_data;

DEVICE_DT_DEFINE(DT_NODELABEL(gpio_240_276),
		    gpio_xec_port240_276_init,
		    NULL,
		    &gpio_xec_port240_276_data, &gpio_xec_port240_276_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_xec_driver_api);

static int gpio_xec_port240_276_init(const struct device *dev)
{
#if DT_IRQ_HAS_CELL(DT_NODELABEL(gpio_240_276), irq)
	const struct gpio_xec_config *config = dev->config;

	/* Turn on the block enable in the EC aggregator */
	mchp_soc_ecia_girq_aggr_en(config->girq_id, 1);

	IRQ_CONNECT(DT_IRQ(DT_NODELABEL(gpio_240_276), irq),
		    DT_IRQ(DT_NODELABEL(gpio_240_276), priority),
		    gpio_gpio_xec_port_isr,
		    DEVICE_DT_GET(DT_NODELABEL(gpio_240_276)), 0U);

	irq_enable(DT_IRQ(DT_NODELABEL(gpio_240_276), irq));
#endif
	return 0;
}
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio_240_276), okay) */
