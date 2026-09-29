/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* eSPI Target sample for the in-tree Microchip XEC V2 eSPI driver.
 * Pairs with samples/boards/mchp_mec5/espi_host_emu running on a second
 * board. Handshake pins:
 *   espi-gpios[0] Target_nREADY output to Host emulator
 *   espi-gpios[1] VCC_PWRGD_ALT input driven by Host emulator
 *   espi-gpios[2] Host_nREADY input from Host emulator
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi/mchp_xec_espi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define ESPI0_NODE       DT_NODELABEL(espi0)

#define ESPI0_IOC_BASE   DT_REG_ADDR_BY_NAME(ESPI0_NODE, io)

/* EC_IRQ generates the EC Serial IRQ in the slot programmed in SIRQ[SIRQ_EC].
 * The XEC V2 driver has no API for it.
 */
#define APP_EC_SIRQ_SLOT 7u

static struct xec_espi_ioc_bar_ldm *const espi_ldm_regs =
	(struct xec_espi_ioc_bar_ldm *)(ESPI0_IOC_BASE + MCHP_ESPI_IO_HOST_BAR_OFS);
static struct xec_espi_ioc_cfg_regs *const espi_cfg_regs =
	(struct xec_espi_ioc_cfg_regs *)(ESPI0_IOC_BASE + MCHP_ESPI_IO_CFG_OFS);

PINCTRL_DT_DEFINE(ZEPHYR_USER_NODE);

static const struct pinctrl_dev_config *app_pinctrl_cfg =
	PINCTRL_DT_DEV_CONFIG_GET(ZEPHYR_USER_NODE);

static const struct gpio_dt_spec target_n_ready_out_dt =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, espi_gpios, 0);
static const struct gpio_dt_spec vcc_pwrgd_alt_in_dt =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, espi_gpios, 1);
static const struct gpio_dt_spec host_n_ready_in_dt =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, espi_gpios, 2);

static const struct device *const espi_dev = DEVICE_DT_GET(ESPI0_NODE);

static struct espi_callback espi_cb_reset;
static struct espi_callback espi_cb_chan_ready;
static struct espi_callback espi_cb_vw;
static struct espi_callback espi_cb_periph;
static struct gpio_callback gpio_cb_vcc_pwrgd;

static volatile uint32_t espi_reset_ev_cnt;
static volatile uint8_t espi_reset_ev_state;

static uint8_t *sram_bar_buf[2];
static size_t sram_bar_size[2];

static void spin_on(uint32_t id, int rval)
{
	LOG_ERR("spin id = %u ret = %d", id, rval);
	log_panic();

	while (1) {
		k_sleep(K_MSEC(1000));
	}
}

static void gpio_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	LOG_INF("GPIO CB: VCC_PWRGD_ALT = %d", gpio_pin_get_dt(&vcc_pwrgd_alt_in_dt));
}

static void espi_reset_cb(const struct device *dev, struct espi_callback *cb,
			  struct espi_event ev)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	espi_reset_ev_cnt++;
	espi_reset_ev_state = (uint8_t)(ev.evt_data & 0x7fu);
	LOG_INF("eSPI CB: eSPI_nReset = %u", espi_reset_ev_state);
}

static void espi_chan_ready_cb(const struct device *dev, struct espi_callback *cb,
			       struct espi_event ev)
{
	static const char *const chan_names[] = {"PC", "VW", "OOB", "FC"};
	const char *name = "?";

	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	for (size_t n = 0; n < ARRAY_SIZE(chan_names); n++) {
		if (ev.evt_details == BIT(n)) {
			name = chan_names[n];
		}
	}

	LOG_INF("eSPI CB: channel %s ready = %u", name, ev.evt_data);
}

static void espi_vw_received_cb(const struct device *dev, struct espi_callback *cb,
				struct espi_event ev)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	LOG_INF("eSPI CB: VW received: signal %u level %u", ev.evt_details, ev.evt_data);
}

static void espi_periph_cb(const struct device *dev, struct espi_callback *cb,
			   struct espi_event ev)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	/* Port 80: evt_details b[15:0] = peripheral, b[23:16] = byte lane,
	 * evt_data b[7:0] = value written by the Host.
	 */
	if ((ev.evt_details & 0xffffu) == ESPI_PERIPHERAL_DEBUG_PORT80) {
		LOG_INF("eSPI CB: PC Port 80 lane %u = 0x%02x", (ev.evt_details >> 16) & 0xffu,
			ev.evt_data & 0xffu);
		return;
	}

	switch (ev.evt_details) {
	case ESPI_PERIPHERAL_UART:
		LOG_INF("eSPI CB: PC Host UART");
		break;
	case ESPI_PERIPHERAL_8042_KBC:
		LOG_INF("eSPI CB: PC 8042-KBC data 0x%x", ev.evt_data);
		break;
	case ESPI_PERIPHERAL_HOST_IO: {
		struct espi_evt_data_acpi *acpi = (struct espi_evt_data_acpi *)&ev.evt_data;

		LOG_INF("eSPI CB: PC ACPI_EC0 %s 0x%02x", acpi->type ? "cmd" : "data", acpi->data);
		break;
	}
	case ESPI_PERIPHERAL_HOST_IO_PVT:
	case ESPI_PERIPHERAL_HOST_IO_PVT2:
	case ESPI_PERIPHERAL_HOST_IO_PVT3:
		LOG_INF("eSPI CB: PC Host PVT%u I/O data 0x%x",
			ev.evt_details - ESPI_PERIPHERAL_HOST_IO_PVT + 1u, ev.evt_data);
		break;
#ifdef CONFIG_ESPI_PERIPHERAL_XEC_ACPI_EC4
	case MCHP_XEC_ESPI_PERIPHERAL_ACPI_EC4:
		LOG_INF("eSPI CB: PC ACPI_EC4 I/O data 0x%x", ev.evt_data);
		break;
#endif
	default:
#ifdef CONFIG_ESPI_PERIPHERAL_XEC_EMI
		if ((ev.evt_details & ~0xffu) == MCHP_XEC_ESPI_PERIPHERAL_EMI) {
			uint8_t emi_id = ev.evt_details & 0xffu;

			LOG_INF("eSPI CB: EMI%u Host-to-EC mailbox = 0x%02x", emi_id,
				ev.evt_data);
			mchp_xec_espi_emi_mbox_ack(dev, emi_id);
			break;
		}
#endif
		LOG_INF("eSPI CB: PC details 0x%x data 0x%x", ev.evt_details, ev.evt_data);
		break;
	}
}

static int app_add_callbacks(void)
{
	int ret;

	espi_init_callback(&espi_cb_reset, espi_reset_cb, ESPI_BUS_RESET);
	espi_init_callback(&espi_cb_chan_ready, espi_chan_ready_cb, ESPI_BUS_EVENT_CHANNEL_READY);
	espi_init_callback(&espi_cb_vw, espi_vw_received_cb, ESPI_BUS_EVENT_VWIRE_RECEIVED);
	espi_init_callback(&espi_cb_periph, espi_periph_cb, ESPI_BUS_PERIPHERAL_NOTIFICATION);

	ret = espi_add_callback(espi_dev, &espi_cb_reset);
	ret = ret ? ret : espi_add_callback(espi_dev, &espi_cb_chan_ready);
	ret = ret ? ret : espi_add_callback(espi_dev, &espi_cb_vw);
	ret = ret ? ret : espi_add_callback(espi_dev, &espi_cb_periph);

	return ret;
}

static void app_sram_bars_init(void)
{
	for (uint8_t id = 0; id < ARRAY_SIZE(sram_bar_buf); id++) {
		int ret = mchp_xec_espi_sram_bar_get(espi_dev, id, &sram_bar_buf[id],
						     &sram_bar_size[id]);

		if (ret) {
			LOG_INF("SRAM BAR%u not enabled (%d)", id, ret);
			continue;
		}

		memset(sram_bar_buf[id], 0, sram_bar_size[id]);
		LOG_INF("SRAM BAR%u: EC buffer %p size %u", id, sram_bar_buf[id],
			(uint32_t)sram_bar_size[id]);
	}
}

static void app_sram_bars_dump(void)
{
	for (uint8_t id = 0; id < ARRAY_SIZE(sram_bar_buf); id++) {
		if (sram_bar_buf[id] != NULL) {
			LOG_HEXDUMP_INF(sram_bar_buf[id], 32, "SRAM BAR data written by Host:");
		}
	}
}

static void wait_chan_enabled(enum espi_channel ch, const char *name)
{
	LOG_INF("Poll driver for %s channel enable", name);
	while (!espi_get_channel_status(espi_dev, ch)) {
		k_sleep(K_MSEC(10));
	}
	LOG_INF("%s channel is enabled", name);
}

static int toggle_sci(void)
{
	uint8_t level = 0;
	int ret;

	ret = espi_receive_vwire(espi_dev, ESPI_VWIRE_SIGNAL_SCI, &level);
	if (ret) {
		return ret;
	}

	ret = espi_send_vwire(espi_dev, ESPI_VWIRE_SIGNAL_SCI, level ^ 1u);
	if (ret) {
		return ret;
	}

	k_sleep(K_MSEC(500)); /* Host eSPI emulator polls */

	return espi_send_vwire(espi_dev, ESPI_VWIRE_SIGNAL_SCI, level);
}

int main(void)
{
	struct espi_cfg ecfg = {
		.io_caps = ESPI_IO_MODE_SINGLE_LINE,
		.channel_caps = ESPI_CHANNEL_PERIPHERAL | ESPI_CHANNEL_VWIRE | ESPI_CHANNEL_OOB |
				ESPI_CHANNEL_FLASH,
		.max_freq = 20u,
	};
	int ret;

	LOG_INF("XEC eSPI Target sample for board: %s", CONFIG_BOARD);

	if (!device_is_ready(espi_dev)) {
		spin_on(__LINE__, -ENODEV);
	}

	ret = gpio_pin_configure_dt(&host_n_ready_in_dt, GPIO_INPUT);
	if (ret) {
		spin_on(__LINE__, ret);
	}

	LOG_INF("Wait for Host Emulator Ready");
	do {
		k_sleep(K_MSEC(10));
		ret = gpio_pin_get_dt(&host_n_ready_in_dt);
		if (ret < 0) {
			spin_on(__LINE__, ret);
		}
	} while (ret == 1);
	LOG_INF("Host Emulator is Ready");

	ret = pinctrl_apply_state(app_pinctrl_cfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		spin_on(__LINE__, ret);
	}

	ret = gpio_pin_configure_dt(&target_n_ready_out_dt, GPIO_OUTPUT_HIGH);
	if (ret) {
		spin_on(__LINE__, ret);
	}

	gpio_init_callback(&gpio_cb_vcc_pwrgd, gpio_cb, BIT(vcc_pwrgd_alt_in_dt.pin));
	ret = gpio_add_callback_dt(&vcc_pwrgd_alt_in_dt, &gpio_cb_vcc_pwrgd);
	ret = ret ? ret : gpio_pin_configure_dt(&vcc_pwrgd_alt_in_dt, GPIO_INPUT);
	ret = ret ? ret : gpio_pin_interrupt_configure_dt(&vcc_pwrgd_alt_in_dt,
							  GPIO_INT_EDGE_BOTH);
	if (ret) {
		spin_on(__LINE__, ret);
	}

	k_sleep(K_MSEC(10)); /* let pins settle */

	/* XEC V2 driver selects eSPI VW PLTRST# as platform reset and releases
	 * RESET_VCC during initialization.
	 */
	LOG_INF("Call eSPI driver config API");
	ret = espi_config(espi_dev, &ecfg);
	if (ret) {
		spin_on(__LINE__, ret);
	}

	ret = app_add_callbacks();
	if (ret) {
		spin_on(__LINE__, ret);
	}

	app_sram_bars_init();

	/* EC SIRQ slot used when the Target generates EC_IRQ below */
	espi_cfg_regs->SIRQ[SIRQ_EC] = APP_EC_SIRQ_SLOT;

	LOG_INF("Signal Host emulator Target is Ready");
	gpio_pin_set_dt(&target_n_ready_out_dt, 0);

	LOG_INF("Wait for Host to de-assert ESPI_nRESET");
	while ((espi_reset_ev_cnt == 0) || (espi_reset_ev_state == 0)) {
		k_sleep(K_MSEC(10));
	}

	wait_chan_enabled(ESPI_CHANNEL_VWIRE, "VW");
	wait_chan_enabled(ESPI_CHANNEL_OOB, "OOB");
	wait_chan_enabled(ESPI_CHANNEL_FLASH, "Flash");

	LOG_INF("Send VW TARGET_BOOT_LOAD_DONE/STATUS = 1");
	ret = espi_send_vwire(espi_dev, ESPI_VWIRE_SIGNAL_TARGET_BOOT_STS, 1);
	ret = ret ? ret : espi_send_vwire(espi_dev, ESPI_VWIRE_SIGNAL_TARGET_BOOT_DONE, 1);
	if (ret) {
		spin_on(__LINE__, ret);
	}

	wait_chan_enabled(ESPI_CHANNEL_PERIPHERAL, "Peripheral");

	k_sleep(K_MSEC(1000));

	LOG_INF("Send EC_IRQ=%u Serial IRQ to the Host", APP_EC_SIRQ_SLOT);
	espi_ldm_regs->PCECIRQ = MCHP_ESPI_EC_IRQ_GEN;

	k_sleep(K_MSEC(500));

	LOG_INF("Toggle nSCI VWire");
	ret = toggle_sci();
	if (ret) {
		spin_on(__LINE__, ret);
	}

	/* Host emulator runs its I/O and memory tests next. Periodically show
	 * what the Host wrote into the SRAM BARs.
	 */
	while (1) {
		k_sleep(K_SECONDS(5));
		app_sram_bars_dump();
	}

	return 0;
}
