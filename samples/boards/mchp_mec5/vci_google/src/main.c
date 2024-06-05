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
#include <zephyr/devicetree/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);


static volatile uint32_t spin_val;
static volatile int ret_val;

static void print_vci_regs(struct mec_vci_regs *regs);
static void spin_on(uint32_t id, int rval);

struct app_vci_pin {
	struct gpio_dt_spec gpio_dt;
	uint8_t vci_info;
};

#define MEC_VCI0_NODE DT_NODELABEL(vci0)
#define MEC_VCI0_REG_ADDR DT_REG_ADDR(MEC_VCI0_NODE)

#define HIB_VCI_PINS_NODE DT_PATH(hibernate_vci_pins)

/* #if DT_NODE_EXISTS(HIB_VCI_PINS_NODE)
#error "!!! OK hibernate_vci_pins DT node exists !!!"
#endif
*/

/* bits[3:0] = VCI_IN bit position in VCI registers
 * bit[4] = 0(active low), 1(active high)
 * bit[5] = 0(do not enable latching), 1(enable latching)
 */
#define MCHP_VCI_INFO_GET_POS(v) ((v) & 0xfu)
#define MCHP_VCI_INFO_GET_POLARITY(v) (((v) >> 4) & 0x1u)
#define MCHP_VCI_INFO_GET_LATCH_EN(v) (((v) >> 5) & 0x1u)

#define MCHP_DT_VCI_INFO(nid) \
	((uint8_t)(DT_PROP(nid, vci_pos) & 0xfu) |\
	 (uint8_t)((DT_ENUM_IDX(nid, vci_polarity) & 0x1) << 4) |\
	 (uint8_t)((DT_PROP_OR(nid, vci_latch_enable, 0) & 0x1) << 5))

#define HIB_VCI_ENTRY(nid) \
	{ \
		.gpio_dt = GPIO_DT_SPEC_GET(nid, gpios), \
		.vci_info = MCHP_DT_VCI_INFO(nid), \
	},

const struct app_vci_pin app_vci_table[] = {
	DT_FOREACH_CHILD(HIB_VCI_PINS_NODE, HIB_VCI_ENTRY)
};

int main(void)
{
	struct mec_vci_regs *vci_regs = (struct mec_vci_regs *)MEC_VCI0_REG_ADDR;
	int ret = 0;

	LOG_INF("MEC5 VCI google sample: board: %s", DT_N_P_compatible_IDX_0);

	LOG_INF("Initial VCI registers");
	print_vci_regs(vci_regs);

	for (size_t n = 0; n < ARRAY_SIZE(app_vci_table); n++) {
		const struct app_vci_pin *pvci = &app_vci_table[n];
		const struct device *gpio_dev = pvci->gpio_dt.port;
		const char *gpio_dev_name = gpio_dev->name;
		uint8_t gpio_pin_pos = pvci->gpio_dt.pin;
		uint8_t vci_pos = MCHP_VCI_INFO_GET_POS(pvci->vci_info);
		uint8_t vci_polarity = MCHP_VCI_INFO_GET_POLARITY(pvci->vci_info);
		uint8_t vci_latch_en = MCHP_VCI_INFO_GET_LATCH_EN(pvci->vci_info);

		LOG_INF("VCI table[%u] GPIO name = %s pin = %u: vci_info = 0x%02x:"
			" vci_pos=%u polarity=%u latch_en=%u", n, gpio_dev_name, gpio_pin_pos,
			pvci->vci_info, vci_pos, vci_polarity, vci_latch_en);

		if (vci_polarity) {
			vci_regs->VCI_POLARITY |= BIT(vci_pos);
		} else {
			vci_regs->VCI_POLARITY &= (uint32_t)~BIT(vci_pos);
		}
		if (vci_latch_en) {
			vci_regs->LATCH_EN |= BIT(vci_pos);
		} else {
			vci_regs->LATCH_EN &= (uint32_t)~BIT(vci_pos);
		}
		vci_regs->VCI_INPUT_EN |= BIT(vci_pos);
	}

	LOG_INF("VCI registers after configuration");
	print_vci_regs(vci_regs);

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

static void print_vci_regs(struct mec_vci_regs *regs)
{
	if (!regs) {
		return;
	}

	LOG_INF("VCI.CONFIG            = 0x%08x", regs->CONFIG);
	LOG_INF("VCI.LATCH_EN          = 0x%08x", regs->LATCH_EN);
	LOG_INF("VCI.LATCH_RESET       = 0x%08x", regs->LATCH_RESET);
	LOG_INF("VCI.VCI_INPUT_EN      = 0x%08x", regs->VCI_INPUT_EN);
	LOG_INF("VCI.HOLD_OFF_CNT      = 0x%08x", regs->HOLD_OFF_CNT);
	LOG_INF("VCI.VCI_POLARITY      = 0x%08x", regs->VCI_POLARITY);
	LOG_INF("VCI.VCI_IN_POSED_STS  = 0x%08x", regs->VCI_IN_POSED_STS);
	LOG_INF("VCI.VCI_IN_NEGED_STS  = 0x%08x", regs->VCI_IN_NEGED_STS);
	LOG_INF("VCI.VCI_IN_VBAT_BUFEN = 0x%08x", regs->VCI_IN_VBAT_BUFEN);
}
