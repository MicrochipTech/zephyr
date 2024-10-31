/*
 * Copyright (c) 2024 Microchip Technology, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <soc.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(app);

#include "espi_target.h"

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define ESPI_NODE        DT_NODELABEL(espi0)

PINCTRL_DT_DEFINE(ZEPHYR_USER_NODE);

/* eSPI Target Thread */

#define ESPI_TARGET_THREAD_STACK_SIZE 1024
#define ESPI_TARGET_THREAD_PRIORITY   6

static void espi_target_thread(void *p1, void *p2, void *p3);
static void espi_reset_cb(const struct device *dev,
			  struct espi_callback *cb,
			  struct espi_event ev);

static void espi_chan_ready_cb(const struct device *dev,
			       struct espi_callback *cb,
			       struct espi_event ev);

static void espi_vw_received_cb(const struct device *dev,
				struct espi_callback *cb,
				struct espi_event ev);

static void espi_periph_cb(const struct device *dev,
			   struct espi_callback *cb,
			   struct espi_event ev);

K_THREAD_DEFINE(espi_target_tid, ESPI_TARGET_THREAD_STACK_SIZE, espi_target_thread, NULL, NULL,
		NULL, ESPI_TARGET_THREAD_PRIORITY, 0, 0);

const struct gpio_dt_spec target_n_ready_out_dt =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, target_gpios, 0);

const struct gpio_dt_spec target_debug_out_dt =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, target_gpios, 1);

const struct pinctrl_dev_config *app_pinctrl_cfg = PINCTRL_DT_DEV_CONFIG_GET(ZEPHYR_USER_NODE);
const struct device *espi_dev = DEVICE_DT_GET(ESPI_NODE);

struct espi_cfg ecfg;
struct espi_callback espi_cb_reset;
struct espi_callback espi_cb_chan_ready;
struct espi_callback espi_cb_vw;
struct espi_callback espi_cb_periph;

#define ESPI_CHAN_READY_PC_IDX 0
#define ESPI_CHAN_READY_VW_IDX 1
#define ESPI_CHAN_READY_OOB_IDX 2
#define ESPI_CHAN_READY_FC_IDX 3
static volatile uint8_t chan_ready[4];

int espi_target_init(uint32_t flags)
{
	struct pcr_regs *pcr = (struct pcr_regs *)0x40080100u;
	int ret = 0;

	ret = pinctrl_apply_state(app_pinctrl_cfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("eSPI target init state apply: %d", ret);
		return -EIO;
	}

	if (!device_is_ready(espi_dev)) {
		LOG_ERR("eSPI target controller not ready!");
		return -EIO;
	}

	ret = gpio_pin_configure_dt(&target_n_ready_out_dt, GPIO_OUTPUT_HIGH);
	if (ret) {
		LOG_ERR("Target nReady out pin config error (%d)", ret);
		return -EIO;
	}

	ret = gpio_pin_configure_dt(&target_debug_out_dt, GPIO_OUTPUT_HIGH);
	if (ret) {
		LOG_ERR("Target debug out pin config error (%d)", ret);
		return -EIO;
	}

	/* PCR enable VCC_PWRGD input and select Platform reset signal to be eSPI
	 * PLTRST# virtual wire.
	 */
	pcr->PWR_RST_CTRL &= (uint32_t)~MCHP_PCR_PR_CTRL_MASK;

	ecfg.io_caps = ESPI_IO_MODE_SINGLE_LINE;
	ecfg.channel_caps = (ESPI_CHANNEL_PERIPHERAL | ESPI_CHANNEL_VWIRE | ESPI_CHANNEL_OOB |
			     ESPI_CHANNEL_FLASH);
	ecfg.max_freq = 20u;
	ret = espi_config(espi_dev, &ecfg);
	if (ret) {
		LOG_ERR("eSPI driver config error (%d)", ret);
		return ret;
	}

	/* register eSPI event callbacks */
	espi_cb_reset.handler = espi_reset_cb;
	espi_cb_reset.evt_type = ESPI_BUS_RESET;
	ret = espi_add_callback(espi_dev, &espi_cb_reset);
	if (ret) {
		LOG_ERR("Add callback for eSPI Reset (%d)", ret);
		return ret;
	}

	espi_cb_chan_ready.handler = espi_chan_ready_cb;
	espi_cb_chan_ready.evt_type = ESPI_BUS_EVENT_CHANNEL_READY;
	ret = espi_add_callback(espi_dev, &espi_cb_chan_ready);
	if (ret) {
		LOG_ERR("Add callback for eSPI channel ready (%d)", ret);
		return ret;
	}

	espi_cb_vw.handler = espi_vw_received_cb;
	espi_cb_vw.evt_type = ESPI_BUS_EVENT_VWIRE_RECEIVED;
	ret = espi_add_callback(espi_dev, &espi_cb_vw);
	if (ret) {
		LOG_ERR("Add callback for VW received (%d)", ret);
	}

	espi_cb_periph.handler = espi_periph_cb;
	espi_cb_periph.evt_type = ESPI_BUS_PERIPHERAL_NOTIFICATION;
	ret = espi_add_callback(espi_dev, &espi_cb_periph);
	if (ret) {
		LOG_ERR("Add callback for peripheral channel (%d)", ret);
	}

	return 0;
}

static void espi_reset_cb(const struct device *dev,
			  struct espi_callback *cb,
			  struct espi_event ev)
{
	LOG_INF("eSPI_nRESET CB: %u", ev.evt_data);
}

static void espi_chan_ready_cb(const struct device *dev,
			       struct espi_callback *cb,
			       struct espi_event ev)
{
	switch (ev.evt_details) {
	case ESPI_CHANNEL_VWIRE:
		chan_ready[ESPI_CHAN_READY_VW_IDX] = ev.evt_data & 0xffu;
		LOG_INF("eSPI Host set VW enable = %u", ev.evt_data);
		break;
	case ESPI_CHANNEL_PERIPHERAL:
		chan_ready[ESPI_CHAN_READY_PC_IDX] = ev.evt_data & 0xffu;
		LOG_INF("eSPI Host set PC enable = %u", ev.evt_data);
		break;
	case ESPI_CHANNEL_OOB:
		chan_ready[ESPI_CHAN_READY_OOB_IDX] = ev.evt_data & 0xffu;
		LOG_INF("eSPI Host set OOB enable = %u", ev.evt_data);
		break;
	case ESPI_CHANNEL_FLASH:
		chan_ready[ESPI_CHAN_READY_FC_IDX] = ev.evt_data & 0xffu;
		LOG_INF("eSPI Host set FC enable = %u", ev.evt_data);
		break;
	default:
		LOG_ERR("eSPI Chan Enable unknown: %u", ev.evt_details);
	}
}

static void espi_vw_received_cb(const struct device *dev,
				struct espi_callback *cb,
				struct espi_event ev)
{
	LOG_INF("eSPI CB: VW received: %u 0x%x 0x%x",
		ev.evt_type, ev.evt_data, ev.evt_details);
}

static void espi_periph_cb(const struct device *dev,
			   struct espi_callback *cb,
			   struct espi_event ev)
{
	LOG_INF("eSPI CB: PC: %u 0x%x 0x%x", ev.evt_type, ev.evt_data, ev.evt_details);
}

static void espi_target_thread(void *p1, void *p2, void *p3)
{
	uint32_t state = 0;
	bool vw_en = false;
	bool pc_en = false;
	bool oob_en = false;
	bool fc_en = false;

	LOG_INF("eSPI Target Thread Entry: Signal Host the Target is ready");

	gpio_pin_set_dt(&target_n_ready_out_dt, 0);
	k_sleep(K_MSEC(100));

	while (1) { /* check host events */
		switch (state) {
		case 0:
			vw_en = espi_get_channel_status(espi_dev, ESPI_CHANNEL_VWIRE);
			pc_en = espi_get_channel_status(espi_dev, ESPI_CHANNEL_PERIPHERAL);
			oob_en = espi_get_channel_status(espi_dev, ESPI_CHANNEL_OOB);
			if (!fc_en && espi_get_channel_status(espi_dev, ESPI_CHANNEL_FLASH)) {
				espi_send_vwire(espi_dev, ESPI_VWIRE_SIGNAL_TARGET_BOOT_STS, 1);
				espi_send_vwire(espi_dev, ESPI_VWIRE_SIGNAL_TARGET_BOOT_DONE, 1);
				state = 1;
				LOG_INF("Set TC VWires Boot Status and Done = 1");
			}
			k_sleep(K_MSEC(100));
			break;
		case 1:
			k_sleep(K_MSEC(1000));
			break;
		case 2:
			k_sleep(K_MSEC(1000));
			break;
		default:
			state = 0;
		}
	}
}
