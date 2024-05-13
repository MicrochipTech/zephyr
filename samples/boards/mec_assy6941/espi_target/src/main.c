/*
 * Copyright (c) 2023 Microchip Technology, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <soc.h>
#include <mec_espi_api.h>
#include <mec_pcr_api.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#include "espi_debug.h"

PINCTRL_DT_DEFINE(DT_PATH(zephyr_user));

const struct pinctrl_dev_config *app_pinctrl_cfg = PINCTRL_DT_DEV_CONFIG_GET(DT_PATH(zephyr_user));

static const struct device *espi_dev = DEVICE_DT_GET(DT_NODELABEL(espi0));

#define ESPI0_NODE DT_NODELABEL(espi0)

struct espi_io_regs *const espi_iobase = (struct espi_io_regs *)DT_REG_ADDR_BY_NAME(ESPI0_NODE, io);

static volatile uint32_t spin_val;
static volatile int ret_val;
static volatile int app_main_exit;

struct espi_cfg ecfg;
struct espi_callback espi_cb_reset;
struct espi_callback espi_cb_chan_ready;
struct espi_callback espi_cb_vw;

#define ESPI_CHAN_READY_PC_IDX 0
#define ESPI_CHAN_READY_VW_IDX 1
#define ESPI_CHAN_READY_OOB_IDX 2
#define ESPI_CHAN_READY_FC_IDX 3
volatile uint8_t chan_ready[4];

#define MAX_ESPI_EVENTS 64
static size_t espi_ev_idx;
static struct espi_event espi_evs[MAX_ESPI_EVENTS];

static void spin_on(uint32_t id, int rval);

static int init_espi_events(struct espi_event *evs, size_t nevents);

static void espi_reset_cb(const struct device *dev,
			  struct espi_callback *cb,
			  struct espi_event ev);

static void espi_chan_ready_cb(const struct device *dev,
			       struct espi_callback *cb,
			       struct espi_event ev);

static void espi_vw_received_cb(const struct device *dev,
			        struct espi_callback *cb,
				struct espi_event ev);

//static void print_espi_events(void);
//static void print_last_espi_events(uint32_t last_n_events);

#if 1
#include <mec_gpio_api.h>
#define MCHP_XEC_GPIO_BASE DT_REG_ADDR(DT_NODELABEL(gpio_000_036))
#define MCHP_XEC_GPIO_CTRL_REG_ADDR(pin) ((uint32_t)MCHP_XEC_GPIO_BASE + ((uint32_t)(pin) * 4u))
#endif

#if 1
/* PC call back */
struct espi_callback espi_cb_pc;
static void espi_pc_received_cb(const struct device *dev,
			    struct espi_callback *cb,
				struct espi_event ev);

#include "ESPI_PC_Hal.h"
#include "ESPI_KBC_Hal.h"

extern unsigned char espi_acpi_ec_space[]; 
#endif 

int main(void)
{
	int ret = 0;
//	size_t n = 0;
//	unsigned short int i;

	LOG_INF("MEC5 eSPI Target sample application for board: %s", DT_N_COMPAT_MODEL_IDX_0);

#if 0
// init space[]
	/* clear ACPI EC space */
	for(i = 0; i < 256; i++)
	{
		espi_acpi_ec_space[i] = 0x00;
	}
#endif 

	ret = pinctrl_apply_state(app_pinctrl_cfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("App pin control state apply: %d", ret);
		spin_on(1, ret);
	}

	/* let pins settle */
	k_sleep(K_MSEC(10));

	LOG_INF("Releasing SoC internal RESET_VCC. Unblocks VCC_PWRGD input");
	mec_pcr_release_reset_vcc(1);

	LOG_INF("Platform configuration: Select nPLTRST = eSPI nPLTRST VWire");
	mec_pcr_host_reset_select(MEC_PCR_PLATFORM_RST_IS_ESPI_PLTRST);

#if 0
	LOG_INF("Poll VCC_PWRGD");
	n = 1000u;
	ret = mec_pcr_is_vcc_pwrgd();
	while (!ret) {
		if (n == 0) {
			LOG_ERR("VCC_PWRGD not present!");
			spin_on(1u, -1);
			goto app_exit;
		}
		--n;
		k_sleep(K_MSEC(2));
		ret = mec_pcr_is_vcc_pwrgd();
	}
#endif

	LOG_INF("VCC_PWRGD OK");
	mec_pcr_release_reset_vcc(1);

	init_espi_events(espi_evs, MAX_ESPI_EVENTS);

	if (!device_is_ready(espi_dev)) {
		LOG_ERR("eSPI device is not ready! (%d)", -1);
		spin_on(3u, -1);
		goto app_exit;
	}

	LOG_INF("eSPI driver is ready");

	LOG_INF("Call eSPI driver config API");
	ecfg.io_caps = ESPI_IO_MODE_SINGLE_LINE;
	ecfg.channel_caps = (ESPI_CHANNEL_PERIPHERAL | ESPI_CHANNEL_VWIRE
			     | ESPI_CHANNEL_OOB | ESPI_CHANNEL_FLASH);
	ecfg.max_freq = 20u;

	ret = espi_config(espi_dev, &ecfg);
	if (ret) {
		LOG_ERR("eSPI configuration API error (%d)", ret);
		spin_on(5u, ret);
		goto app_exit;
	}

	LOG_INF("eSPI after config");
	espi_debug_print_config();
	/* espi_debug_print_io_bars(); */
	/* espi_debug_print_mem_bars(); */

	espi_cb_reset.handler = espi_reset_cb;
	espi_cb_reset.evt_type = ESPI_BUS_RESET;
	ret = espi_add_callback(espi_dev, &espi_cb_reset);
	if (ret) {
		LOG_ERR("API error add callback for eSPI Reset (%d)", ret);
		spin_on(6u, ret);
		goto app_exit;
	}

	espi_cb_chan_ready.handler = espi_chan_ready_cb;
	espi_cb_chan_ready.evt_type = ESPI_BUS_EVENT_CHANNEL_READY;
	ret = espi_add_callback(espi_dev, &espi_cb_chan_ready);
	if (ret) {
		LOG_ERR("API error add callback for VW ChanEn (%d)", ret);
		spin_on(7u, ret);
		goto app_exit;
	}

	espi_cb_vw.handler = espi_vw_received_cb;
	espi_cb_vw.evt_type = ESPI_BUS_EVENT_VWIRE_RECEIVED;
	ret = espi_add_callback(espi_dev, &espi_cb_vw);
	if (ret) {
		LOG_ERR("API error add callback for VW received (%d)", ret);
		spin_on(8u, ret);
		goto app_exit;
	}

#if 1
/* PC call back */
	espi_cb_pc.handler = espi_pc_received_cb;
	espi_cb_pc.evt_type = ESPI_BUS_PERIPHERAL_NOTIFICATION;
	ret = espi_add_callback(espi_dev, &espi_cb_pc);
	if (ret) {
		LOG_ERR("API error add callback for PC received (%d)", ret);
		spin_on(8u, ret);
		goto app_exit;
	}
#endif 
	/* Release board RSMRST# allowing Host eSPI controller to exit reset state
	 * and begin eSPI link training.
	 * TODO - we need pin uses as RSMRST#
	 */

#if 1
/* KF support */
	LOG_INF("main: eSPI init done, and cfg gpio0242 as PWRGD input##");
	uint32_t temp = 0;
	temp = 0x3041;
	sys_write32(temp, MEC_GPIO_CTRL_REG_ADDR(MEC_PIN_0242));
#endif

#if 1
	LOG_INF("main: eSPI init done, and release RSMRST# high####");
	//uint32_t temp = 0;
	temp = 0x10240;
	sys_write32(temp, MEC_GPIO_CTRL_REG_ADDR(MEC_PIN_0054));
#endif

	/* TODO - Poll driver for channel enables or use callbacks to know when channels
	 * are enabled.
	 */

//	LOG_INF("Print all eSPI events");
//	print_espi_events();

do{
	temp = sys_read32(MEC_GPIO_CTRL_REG_ADDR(MEC_PIN_0242));
}
while((temp & 0x01000000) == 0);
//k_busy_wait(5000);

#if 1
/* KF support */
	LOG_INF("main: RSMRST# high, then VR_ON high####");
	temp = 0x10240;
	sys_write32(temp, MEC_GPIO_CTRL_REG_ADDR(MEC_PIN_0106));
#endif

	LOG_INF("ADone");
#if 1
/* support kbcec cmd process */
	while(1)
	{
		;
	}
#endif 

	spin_on(256, 0);

app_exit:
	LOG_INF("App Exit");
	log_panic();
	app_main_exit = 1;

	return 0;
}

static void spin_on(uint32_t id, int rval)
{
	spin_val = id;
	ret_val = rval;

	LOG_INF("spin_on %u", id);
	log_panic(); /* flush log buffers */

	while (spin_val) {
		;
	}
}

static int init_espi_events(struct espi_event *evs, size_t nevents)
{
	if (!evs) {
		return -EINVAL;
	}

	espi_ev_idx = 0;
	memset(evs, 0, sizeof(struct espi_event) * nevents);

	return 0;
}

/* eSPI driver callbacks */
static void espi_reset_cb(const struct device *dev,
			  struct espi_callback *cb,
			  struct espi_event ev)
{
//	LOG_INF("eSPI CB: eSPI_nReset");
//	LOG_INF("eSPI RESET# ISR CallBack: eSPI_nReset#");
}

static void espi_chan_ready_cb(const struct device *dev,
			       struct espi_callback *cb,
			       struct espi_event ev)
{
//	LOG_INF("espi_chan_ready_cb: eSPI CB: channel ready: %u 0x%x 0x%x",
//		ev.evt_type, ev.evt_data, ev.evt_details);

	if (ev.evt_details == ESPI_CHANNEL_VWIRE) {
		if (ev.evt_data == 1) {
			chan_ready[ESPI_CHAN_READY_VW_IDX] = 1;
		} else {
			chan_ready[ESPI_CHAN_READY_VW_IDX] = 0;
		}
	}

	if (ev.evt_details == ESPI_CHANNEL_PERIPHERAL) {
		if (ev.evt_data == 1) {
			chan_ready[ESPI_CHAN_READY_PC_IDX] = 1;
		} else {
			chan_ready[ESPI_CHAN_READY_PC_IDX] = 0;
		}
	}

	if (ev.evt_details == ESPI_CHANNEL_OOB) {
		if (ev.evt_data == 1) {
			chan_ready[ESPI_CHAN_READY_OOB_IDX] = 1;
		} else {
			chan_ready[ESPI_CHAN_READY_OOB_IDX] = 0;
		}
	}

	if (ev.evt_details == ESPI_CHANNEL_FLASH) {
		if (ev.evt_data == 1) {
			chan_ready[ESPI_CHAN_READY_FC_IDX] = 1;
		} else {
			chan_ready[ESPI_CHAN_READY_FC_IDX] = 0;
		}
	}
}

static void espi_vw_received_cb(const struct device *dev,
			       struct espi_callback *cb,
			       struct espi_event ev)
{
//	LOG_INF("eSPI CB: VW received: type= data= detail(vw enum)=: %u 0x%x 0x%x",
//		ev.evt_type, ev.evt_data, ev.evt_details);

	if (espi_ev_idx < MAX_ESPI_EVENTS) {
		espi_evs[espi_ev_idx].evt_type = ev.evt_type;
		espi_evs[espi_ev_idx].evt_data = ev.evt_data;
		espi_evs[espi_ev_idx].evt_details = ev.evt_details;
		espi_ev_idx++;
	}

#if 1
	/* dispatch individual VW ISR */
	if(ev.evt_details == ESPI_VWIRE_SIGNAL_SUS_WARN)
	{
		//LOG_INF("eSPI Event: Host-to-Target VW ESPI_VWIRE_SIGNAL_SUS_WARN##");
		/* ACK SUS_WARN# */
		espi_send_vwire(dev, ESPI_VWIRE_SIGNAL_SUS_ACK, ev.evt_data);
	}
#endif
}

struct espi_vw_znames {
	enum espi_vwire_signal signal;
	const char *name_cstr;
};

const struct espi_vw_znames vw_names[] = {
	{ ESPI_VWIRE_SIGNAL_SLP_S3, "SLP_S3#" },
	{ ESPI_VWIRE_SIGNAL_SLP_S4, "SLP_S4#" },
	{ ESPI_VWIRE_SIGNAL_SLP_S5, "SLP_S5#" },
	{ ESPI_VWIRE_SIGNAL_OOB_RST_WARN, "OOB_RST_WARN" },
	{ ESPI_VWIRE_SIGNAL_PLTRST, "PLTRST#" },
	{ ESPI_VWIRE_SIGNAL_SUS_STAT, "SUS_STAT#" },
	{ ESPI_VWIRE_SIGNAL_NMIOUT, "NMIOUT#" },
	{ ESPI_VWIRE_SIGNAL_SMIOUT, "SMIOUT#" },
	{ ESPI_VWIRE_SIGNAL_HOST_RST_WARN, "HOST_RST_WARN" },
	{ ESPI_VWIRE_SIGNAL_SLP_A, "SLP_A#" },
	{ ESPI_VWIRE_SIGNAL_SUS_PWRDN_ACK, "SUS_PWRDN_ACK" },
	{ ESPI_VWIRE_SIGNAL_SUS_WARN, "SUS_WARN#" },
	{ ESPI_VWIRE_SIGNAL_SLP_WLAN, "SLP_WLAN#" },
	{ ESPI_VWIRE_SIGNAL_SLP_LAN, "SLP_LAN#" },
	{ ESPI_VWIRE_SIGNAL_HOST_C10, "HOST_C10" },
	{ ESPI_VWIRE_SIGNAL_DNX_WARN, "DNX_WARN" },
	{ ESPI_VWIRE_SIGNAL_PME, "PME#" },
	{ ESPI_VWIRE_SIGNAL_WAKE, "WAKE#" },
	{ ESPI_VWIRE_SIGNAL_OOB_RST_ACK, "OOB_RST_ACK" },
	{ ESPI_VWIRE_SIGNAL_TARGET_BOOT_STS, "TARGET_BOOT_STS" },
	{ ESPI_VWIRE_SIGNAL_ERR_NON_FATAL, "ERR_NON_FATAL" },
	{ ESPI_VWIRE_SIGNAL_ERR_FATAL, "ERR_FATAL" },
	{ ESPI_VWIRE_SIGNAL_TARGET_BOOT_DONE, "TARGET_BOOT_DONE" },
	{ ESPI_VWIRE_SIGNAL_HOST_RST_ACK, "HOST_RST_ACK" },
	{ ESPI_VWIRE_SIGNAL_RST_CPU_INIT, "RST_CPU_INIT" },
	{ ESPI_VWIRE_SIGNAL_SMI, "SMI#" },
	{ ESPI_VWIRE_SIGNAL_SCI, "SCI#" },
	{ ESPI_VWIRE_SIGNAL_DNX_ACK, "DNX_ACK" },
	{ ESPI_VWIRE_SIGNAL_SUS_ACK, "SUS_ACK#" },
	{ ESPI_VWIRE_SIGNAL_TARGET_GPIO_0, "TARGET_GPIO_0" },
	{ ESPI_VWIRE_SIGNAL_TARGET_GPIO_1, "TARGET_GPIO_1" },
	{ ESPI_VWIRE_SIGNAL_TARGET_GPIO_2, "TARGET_GPIO_2" },
	{ ESPI_VWIRE_SIGNAL_TARGET_GPIO_3, "TARGET_GPIO_3" },
	{ ESPI_VWIRE_SIGNAL_TARGET_GPIO_4, "TARGET_GPIO_4" },
	{ ESPI_VWIRE_SIGNAL_TARGET_GPIO_5, "TARGET_GPIO_5" },
	{ ESPI_VWIRE_SIGNAL_TARGET_GPIO_6, "TARGET_GPIO_6" },
	{ ESPI_VWIRE_SIGNAL_TARGET_GPIO_7, "TARGET_GPIO_7" },
	{ ESPI_VWIRE_SIGNAL_TARGET_GPIO_8, "TARGET_GPIO_8" },
	{ ESPI_VWIRE_SIGNAL_TARGET_GPIO_9, "TARGET_GPIO_9" },
	{ ESPI_VWIRE_SIGNAL_TARGET_GPIO_10, "TARGET_GPIO_10" },
	{ ESPI_VWIRE_SIGNAL_TARGET_GPIO_11, "TARGET_GPIO_11" },
};

const char *unkown_vwire = "Unknown VWire";

#if 0
static const char * get_vw_name(uint32_t vwire_enum_val)
{
	for (size_t n = 0; n < ARRAY_SIZE(vw_names); n++) {
		const struct espi_vw_znames *vwn = &vw_names[n];

		if (vwn->signal == (enum espi_vwire_signal)vwire_enum_val) {
			return vwn->name_cstr;
		}
	}

	return unkown_vwire;
}

static void pr_decode_espi_event(uint32_t evt_type, uint32_t evt_data, uint32_t evt_details)
{
	switch (evt_type) {
	case ESPI_BUS_RESET:
		LOG_INF("eSPI Event: Bus Reset: data=0x%0x details=0x%0x", evt_data, evt_details);
		break;
	case ESPI_BUS_EVENT_CHANNEL_READY:
		LOG_INF("eSPI Event: Channel Ready: data=0x%0x details=0x%0x",
			evt_data, evt_details);
		break;
	case ESPI_BUS_EVENT_VWIRE_RECEIVED:
		const char *vwname = get_vw_name(evt_details);
		LOG_INF("eSPI Event: Host-to-Target VW Received: data=0x%0x details=0x%0x VW=%s",
			evt_data, evt_details, vwname);
		break;
	case ESPI_BUS_EVENT_OOB_RECEIVED:
		LOG_INF("eSPI Event: OOB Received: data=0x%0x details=0x%0x",
			evt_data, evt_details);
		break;
	case ESPI_BUS_PERIPHERAL_NOTIFICATION:
		LOG_INF("eSPI Event: Peripheral Notification: data=0x%0x details=0x%0x",
			evt_data, evt_details);
		break;
	case ESPI_BUS_SAF_NOTIFICATION:
		LOG_INF("eSPI Event: SAF Notification: data=0x%0x details=0x%0x",
			evt_data, evt_details);
		break;
	default:
		LOG_INF("eSPI Event: Unknown type=%u, data=0x%0x details=0x%0x",
			evt_type, evt_data, evt_details);
		break;
	}
}

static void print_espi_events(void)
{
	if (espi_ev_idx) {
		for (size_t n = 0; n < espi_ev_idx; n++) {
			uint32_t t = espi_evs[n].evt_type;
			uint32_t dat = espi_evs[n].evt_data;
			uint32_t det = espi_evs[n].evt_details;

			LOG_INF("eSPI event number %u", n);
			pr_decode_espi_event(t, dat, det);
		}
	} else {
		LOG_INF("No eSPI Event have been received");
	}
}

static void print_last_espi_events(uint32_t last_n_events)
{
	if (last_n_events && espi_ev_idx) {
		for (size_t n = espi_ev_idx; n >= 1; n--) {
			struct espi_event *ev = &espi_evs[n-1];

			LOG_INF("eSPI event %u", n);
			pr_decode_espi_event(ev->evt_type, ev->evt_data, ev->evt_details);
			if (--last_n_events == 0) {
				break;
			}
		}
	}
}
#endif

#if 1
/* PC call back */
static void espi_pc_received_cb(const struct device *dev,
			    struct espi_callback *cb,
				struct espi_event ev)
{
	unsigned char ec_rx;
//	LOG_INF("eSPI CB: PC received: type= data= detail=: %u 0x%x 0x%x",
//		ev.evt_type, ev.evt_data, ev.evt_details);

	/* 1: process ACPI EC0 */
	if(ev.evt_type == ESPI_BUS_PERIPHERAL_NOTIFICATION)
	{	/* PC event */
		if(ev.evt_details == ESPI_PERIPHERAL_HOST_IO)
		{	/* ACPI EC0 */
			ec_rx = (unsigned char)((ev.evt_data >> 8) & 0xFF);
			if((ev.evt_data & 0xFF) == 0)
			{	/* cmd */
				LOG_INF("ACPIEC0 C=0x%02x", ec_rx);
				ESPI_ACPI_Cmd(ec_rx);
			} else if((ev.evt_data & 0xFF) == 1)
			{	/* data */
				LOG_INF("ACPIEC0 D#=0x%02x", ec_rx);
				ESPI_Srvc_Pcdat2(ec_rx);
			} else
			{	/* 4-bye mode */
				LOG_ERR("espi_pc_received_cb: ACPI 4-byte mode??? (0x%08x)", ev.evt_data);
			}
		} else if(ev.evt_details == ESPI_PERIPHERAL_8042_KBC)
		{	/* KBC 8042 */
			ec_rx = (unsigned char)((ev.evt_data >> 8) & 0xFF);
			if((ev.evt_data & 0xFF) == 1)
			{	/* cmd */
				LOG_INF("KBC C=0x%02x", ec_rx);
				ESPI_KBC_Cmd(ec_rx);
			} else if((ev.evt_data & 0xFF) == 0)
			{	/* data */
				LOG_INF("KBC D#=0x%02x", ec_rx);
				ESPI_Srvc_Kbcdat1(ec_rx);
			} else
			{	/* 4-bye mode */
				LOG_ERR("espi_pc_received_cb: ACPI 4-byte mode??? (0x%08x)", ev.evt_data);
			}
		}
	}
}
#endif 
