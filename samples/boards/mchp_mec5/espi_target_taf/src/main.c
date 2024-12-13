/*
 * Copyright (c) 2024 Microchip Technology, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <soc.h>
#include <mec_acpi_ec_api.h>
#include <mec_espi_api.h>
#include <mec_pcr_api.h>
#include <mec_rom_api.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_taf.h>
#include <zephyr/drivers/espi/espi_mchp_mec5.h>
#include <zephyr/drivers/espi/espi_taf_mchp_mec5.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#include "espi_debug.h"
#include "espi_taf_sample.h"
#include "app_hmac.h"
#include "taf_rpmc.h"

/* #define APP_ESPI_EVENT_CAPTURE */

struct gen_acpi_ec_info {
	const struct device *dev;
	uint16_t iobase;
	mchp_espi_pc_aec_callback_t cb;
};

#define ZEPHYR_USER_NODE	DT_PATH(zephyr_user)
#define ESPI0_NODE		DT_NODELABEL(espi0)
#define QSPI0_NODE		DT_NODELABEL(qspi0)

#define FLASH0_NODE		DT_NODELABEL(shd_flash0)
#define FLASH1_NODE		DT_NODELABEL(shd_flash1)

#define ACPI_EC2_NODE DT_NODELABEL(acpi_ec2)
#define ACPI_EC3_NODE DT_NODELABEL(acpi_ec3)
#define ACPI_EC4_NODE DT_NODELABEL(acpi_ec4)

#define EMI0_NODE DT_NODELABEL(emi0)
#define EMI1_NODE DT_NODELABEL(emi1)

PINCTRL_DT_DEFINE(ZEPHYR_USER_NODE);

const struct pinctrl_dev_config *app_pinctrl_cfg = PINCTRL_DT_DEV_CONFIG_GET(ZEPHYR_USER_NODE);

const struct gpio_dt_spec target_n_ready_out_dt =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, espi_gpios, 0);

const struct gpio_dt_spec vcc_pwrgd_alt_in_dt =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, espi_gpios, 1);

static const struct device *qspi_dev = DEVICE_DT_GET(QSPI0_NODE);
static const struct device *espi_dev = DEVICE_DT_GET(ESPI0_NODE);
static const struct device *spi_flash0_dev = DEVICE_DT_GET(FLASH0_NODE);
static const struct device *spi_flash1_dev = DEVICE_DT_GET(FLASH1_NODE);

struct mec_espi_io_regs *const espi_iobase =
	(struct mec_espi_io_regs *)DT_REG_ADDR_BY_NAME(ESPI0_NODE, io);

#define OS_ACPI_EC_NODE DT_CHOSEN(espi_os_acpi)
struct mec_acpi_ec_regs *const os_acpi_ec_regs =
	(struct mec_acpi_ec_regs *)DT_REG_ADDR(OS_ACPI_EC_NODE);

#if DT_NODE_HAS_STATUS(EMI0_NODE, okay)
static const struct device *emi0_dev = DEVICE_DT_GET(EMI0_NODE);
#else
static const struct device *emi0_dev = NULL;
#endif
#if DT_NODE_HAS_STATUS(EMI1_NODE, okay)
static const struct device *emi1_dev = DEVICE_DT_GET(EMI1_NODE);
#else
static const struct device *emi1_dev = NULL;
#endif

static volatile uint32_t spin_val;
static volatile int ret_val;
static volatile int app_main_exit;

struct espi_cfg ecfg;
struct espi_callback espi_cb_reset;
struct espi_callback espi_cb_chan_ready;
struct espi_callback espi_cb_vw;
struct espi_callback espi_cb_periph;
struct gpio_callback gpio_cb_vcc_pwrgd;

#define ESPI_CHAN_READY_PC_IDX 0
#define ESPI_CHAN_READY_VW_IDX 1
#define ESPI_CHAN_READY_OOB_IDX 2
#define ESPI_CHAN_READY_FC_IDX 3
volatile uint8_t chan_ready[4];

struct spi_flash_info {
	uint32_t jedec_id;
	uint8_t status1;
	uint8_t status2;
	uint8_t status3;
};

struct spi_flash_info flash0_info;
struct spi_flash_info flash1_info;

#define APP_BUF1_SIZE 256
uint8_t buf1[APP_BUF1_SIZE];
uint8_t buf2[APP_BUF1_SIZE];

/* local function prototypes */
static void spin_on(uint32_t id, int rval);

static void pr_mchphash_def(void);
static void pr_mchphmac2_def(void);
#if 0
static void pr_mchp_dmaslot_def(void);
#endif

static int app_read_spi_flash_status_reg(const struct device *spi_dev, const struct spi_config *cfg,
					 uint8_t status_id, uint8_t *status);

static bool is_buffer_filled_with(const uint8_t *buf, uint32_t bufsz, uint8_t val);

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

#define GPIO_0242_PIN_MASK BIT(2)
static void gpio_cb(const struct device *port,
		    struct gpio_callback *cb,
		    gpio_port_pins_t pins);

static void acpi_ec2_cb(const struct device *dev, struct mchp_espi_acpi_ec_event *ev,
			void *user_data);
static void acpi_ec3_cb(const struct device *dev, struct mchp_espi_acpi_ec_event *ev,
			void *user_data);
#if DT_NODE_HAS_STATUS(ACPI_EC4_NODE, okay)
static void acpi_ec4_cb(const struct device *dev, struct mchp_espi_acpi_ec_event *ev,
			void *user_data);
#endif

static void emi0_cb(const struct device *dev, uint32_t emi_mbox_data, void *user_data);
static void emi1_cb(const struct device *dev, uint32_t emi_mbox_data, void *user_data);

const struct gen_acpi_ec_info gen_acpi_ec_tbl[] = {
	{
		.dev = DEVICE_DT_GET(ACPI_EC2_NODE),
		.iobase = (uint16_t)DT_PROP(DT_PHANDLE_BY_IDX(ACPI_EC2_NODE, host_infos, 0), host_address),
		.cb = acpi_ec2_cb,
	},
#if DT_NODE_HAS_STATUS(ACPI_EC3_NODE, okay)
	{
		.dev = DEVICE_DT_GET(ACPI_EC3_NODE),
		.iobase = (uint16_t)DT_PROP(DT_PHANDLE_BY_IDX(ACPI_EC3_NODE, host_infos, 0), host_address),
		.cb = acpi_ec3_cb,
	},
#endif
#if DT_NODE_HAS_STATUS(ACPI_EC4_NODE, okay)
	{
		.dev = DEVICE_DT_GET(ACPI_EC4_NODE),
		.iobase = (uint16_t)DT_PROP(DT_PHANDLE_BY_IDX(ACPI_EC4_NODE, host_infos, 0), host_address),
		.cb = acpi_ec4_cb,
	},
#endif
};

const struct spi_config spi_cfg_fd_cs0 = {
	.frequency = MHZ(12),
	.operation = SPI_OP_MODE_MASTER | SPI_LINES_SINGLE | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
	.slave = 0,
	.cs = {
		.gpio = {
			.port = NULL,
			.pin = 0,
			.dt_flags = 0,
		},
		.delay = 0,
	},
};

const struct spi_config spi_cfg_fd_cs1 = {
	.frequency = MHZ(12),
	.operation = SPI_OP_MODE_MASTER | SPI_LINES_SINGLE | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
	.slave = 1,
	.cs = {
		.gpio = {
			.port = NULL,
			.pin = 0,
			.dt_flags = 0,
		},
		.delay = 0,
	},
};

int main(void)
{
	int ret = 0;
	uint32_t n = 0, spi_addr = 0, spi_len = 0;
	uint8_t vw_state = 0;
	uint8_t chan_en_count = 0;
	bool generic_acpi_ec_en[3] = { false, false ,false };

	LOG_INF("MEC5 eSPI Target sample application for board: %s", DT_N_COMPAT_MODEL_IDX_0);

	memset((void *)chan_ready, 0, sizeof(chan_ready));
#if 0
	pr_mchp_dmaslot_def();
#endif
	pr_mchphash_def();
	pr_mchphmac2_def();

	if (!device_is_ready(qspi_dev)) {
		LOG_ERR("QSPI device used for TAF is not ready!");
		spin_on((uint32_t)__LINE__, 1);
	}

	if (!device_is_ready(spi_flash0_dev)) {
		LOG_ERR("SPI_NOR flash device at CS0 is not ready!");
		spin_on((uint32_t)__LINE__, 1);
	}

	if (!device_is_ready(spi_flash1_dev)) {
		LOG_ERR("SPI_NOR flash device at CS1 is not ready!");
		spin_on((uint32_t)__LINE__, 1);
	}

	flash0_info.jedec_id = 0;
	ret = flash_read_jedec_id(spi_flash0_dev, (uint8_t *)&flash0_info.jedec_id);
	if (ret) {
		LOG_ERR("Flash driver failed to read JEDEC ID for CS0 (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Flash at CS0 JEDEC-ID = 0x%08x", flash0_info.jedec_id);

	flash1_info.jedec_id = 0;
	ret = flash_read_jedec_id(spi_flash1_dev, (uint8_t *)&flash1_info.jedec_id);
	if (ret) {
		LOG_ERR("Flash driver failed to read JEDEC ID for CS1 (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Flash at CS1 JEDEC-ID = 0x%08x", flash1_info.jedec_id);

	ret = app_read_spi_flash_status_reg(qspi_dev, &spi_cfg_fd_cs0, 1, &flash0_info.status1);
	if (ret) {
		LOG_ERR("SPI driver failed to read flash at CS0 STATUS1: (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}
	LOG_INF("Flash at CS0 STATUS1 = 0x%02x", flash0_info.status1);

	ret = app_read_spi_flash_status_reg(qspi_dev, &spi_cfg_fd_cs0, 2, &flash0_info.status2);
	if (ret) {
		LOG_ERR("SPI driver failed to read flash at CS0 STATUS2: (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}
	LOG_INF("Flash at CS0 STATUS2 = 0x%02x", flash0_info.status2);

	ret = app_read_spi_flash_status_reg(qspi_dev, &spi_cfg_fd_cs1, 1, &flash1_info.status1);
	if (ret) {
		LOG_ERR("SPI driver failed to read flash at CS1 STATUS1: (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}
	LOG_INF("Flash at CS1 STATUS1 = 0x%02x", flash1_info.status1);

	ret = app_read_spi_flash_status_reg(qspi_dev, &spi_cfg_fd_cs1, 2, &flash1_info.status2);
	if (ret) {
		LOG_ERR("SPI driver failed to read flash at CS1 STATUS2: (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}
	LOG_INF("Flash at CS1 STATUS2 = 0x%02x", flash1_info.status2);

	LOG_INF("Initialize crypto");
	ret = app_crypto_init();
	if (ret) {
		LOG_ERR("App crypto init failed (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Test HMAC");
	ret = app_hmac_test();
	if (ret) {
		LOG_ERR("HMAC test FAIL");
		spin_on((uint32_t)__LINE__, 1);
	} else {
		LOG_INF("HMAC test PASS");
	}

	memset(buf1, 0x55, APP_BUF1_SIZE);

	spi_addr = 0x1000u;
	spi_len = 64u;
	LOG_INF("Read %u bytes from SPI at 0x%0x", spi_len, spi_addr);

	ret = flash_read(spi_flash0_dev, spi_addr, buf1, spi_len);
	if (ret) {
		LOG_ERR("App flash read error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}
#if 0
	ret = app_spi_get_data(qspi_dev, spi_addr, spi_len, buf1, APP_BUF1_SIZE);
	if (ret) {
		LOG_ERR("App SPI data read failure: (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}
#endif
	if (is_buffer_filled_with((const uint8_t *)buf1, spi_len, 0xffu)) {
		LOG_INF("SPI region is in erased state. Write pattern");
		for (n = 0; n < 256u; n++) {
			buf1[n] = (uint8_t)n;
		}
		ret = flash_write(spi_flash0_dev, spi_addr, (const void *)buf1, 256u);
		if (ret) {
			LOG_ERR("App flash write pattern failed (%d)", ret);
			spin_on((uint32_t)__LINE__, ret);
		}
		ret = flash_read(spi_flash0_dev, spi_addr, buf2, spi_len);
		if (ret) {
			LOG_ERR("App flash read error (%d)", ret);
			spin_on((uint32_t)__LINE__, ret);
		}
		ret = memcmp(buf1, buf2, spi_len);
		if (ret) {
			LOG_ERR("App data mismatch after write-read-back of flash");
			spin_on((uint32_t)__LINE__, ret);
		}
	}

	LOG_HEXDUMP_INF(buf1, spi_len, "SPI data");

	for (size_t n = 0; n < ARRAY_SIZE(gen_acpi_ec_tbl); n++) {
		const struct gen_acpi_ec_info *info = &gen_acpi_ec_tbl[n];

		if (device_is_ready(info->dev)) {
			LOG_INF("Device %s has I/O base 0x%0u", info->dev->name, info->iobase);
			ret = mchp_espi_pc_aec_set_callback(info->dev, info->cb, NULL);
			if (ret) {
				LOG_ERR("Device %s cb set error (%d)", info->dev->name, ret);
			} else {
				generic_acpi_ec_en[n] = true;
			}
		} else {
			LOG_INF("ERROR: Device %s is not ready", info->dev->name);
		}
	}

	ret = pinctrl_apply_state(app_pinctrl_cfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("App pin control state apply: %d", ret);
		spin_on((uint32_t)__LINE__, 1);
	}

	ret = gpio_pin_configure_dt(&target_n_ready_out_dt, GPIO_OUTPUT_HIGH);
	if (ret) {
		LOG_ERR("Configure Target_nReady GPIO as output hi (%d)", ret);
		spin_on((uint32_t)__LINE__, 2);
	}

	gpio_init_callback(&gpio_cb_vcc_pwrgd, gpio_cb, GPIO_0242_PIN_MASK);
	ret = gpio_add_callback_dt(&vcc_pwrgd_alt_in_dt, &gpio_cb_vcc_pwrgd);
	if (ret) {
		LOG_ERR("GPIO add callback for VCC_PWRGD error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	ret = gpio_pin_configure_dt(&vcc_pwrgd_alt_in_dt, GPIO_INPUT);
	if (ret) {
		LOG_ERR("Configure VCC_PWRGD_ALT_IN GPIO input (%d)", ret);
		spin_on((uint32_t)__LINE__, 2);
	}

	ret =  gpio_pin_interrupt_configure_dt(&vcc_pwrgd_alt_in_dt,
					       (GPIO_INPUT | GPIO_INT_EDGE_BOTH));
	if (ret) {
		LOG_ERR("GPIO interrupt config for VCC_PWRGD_ALT_IN error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	/* let pins settle */
	k_sleep(K_MSEC(10));

	if (!device_is_ready(espi_dev)) {
		LOG_ERR("eSPI device is not ready! (%d)", -1);
		spin_on((uint32_t)__LINE__, 3);
	}

	LOG_INF("eSPI driver is ready");

	LOG_INF("Releasing SoC internal RESET_VCC. Unblocks VCC_PWRGD input");
	mec_hal_pcr_release_reset_vcc(1);

	LOG_INF("Platform configuration: Select nPLTRST = eSPI nPLTRST VWire");
	mec_hal_pcr_host_reset_select(MEC_PCR_PLATFORM_RST_IS_ESPI_PLTRST);

	LOG_INF("Call eSPI driver config API");
	ecfg.io_caps = ESPI_IO_MODE_SINGLE_LINE;
	ecfg.channel_caps = (ESPI_CHANNEL_PERIPHERAL | ESPI_CHANNEL_VWIRE
			     | ESPI_CHANNEL_OOB | ESPI_CHANNEL_FLASH);
	ecfg.max_freq = 20u;

	ret = espi_config(espi_dev, &ecfg);
	if (ret) {
		LOG_ERR("eSPI configuration API error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("eSPI Target after config");
	espi_debug_print_config();

	LOG_INF("Target eSPI: register event callbacks");
	espi_cb_reset.handler = espi_reset_cb;
	espi_cb_reset.evt_type = ESPI_BUS_RESET;
	ret = espi_add_callback(espi_dev, &espi_cb_reset);
	if (ret) {
		LOG_ERR("Add callback for eSPI Reset error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	espi_cb_chan_ready.handler = espi_chan_ready_cb;
	espi_cb_chan_ready.evt_type = ESPI_BUS_EVENT_CHANNEL_READY;
	ret = espi_add_callback(espi_dev, &espi_cb_chan_ready);
	if (ret) {
		LOG_ERR("Add callback for VW ChanEn error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	espi_cb_vw.handler = espi_vw_received_cb;
	espi_cb_vw.evt_type = ESPI_BUS_EVENT_VWIRE_RECEIVED;
	ret = espi_add_callback(espi_dev, &espi_cb_vw);
	if (ret) {
		LOG_ERR("Add callback for VW received error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	espi_cb_periph.handler = espi_periph_cb;
	espi_cb_periph.evt_type = ESPI_BUS_PERIPHERAL_NOTIFICATION;
	ret = espi_add_callback(espi_dev, &espi_cb_periph);
	if (ret) {
		LOG_ERR("Add callback for peripheral channel error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	if (device_is_ready(emi0_dev)) {
		ret = mchp_espi_pc_emi_set_callback(emi0_dev, emi0_cb, NULL);
		if (ret) {
			LOG_ERR("Add EMI0 callback error (%d)", ret);
		}
	} else {
		LOG_ERR("eSPI EMI0 PC device driver is Not Ready!");
	}

	if (device_is_ready(emi1_dev)) {
		ret = mchp_espi_pc_emi_set_callback(emi1_dev, emi1_cb, NULL);
		if (ret) {
			LOG_ERR("Add EMI1 callback error (%d)", ret);
		}
	} else {
		LOG_ERR("eSPI EMI1 PC device driver is Not Ready!");
	}

#if 0
	/* eSPI TAF */
	ret = sample_espi_taf_config(0x01u);
	if (ret) {
		spin_on((uint32_t)__LINE__, ret);
	}
#endif

	LOG_INF("Signal Host emulator Target is Ready");
	ret = gpio_pin_set_dt(&target_n_ready_out_dt, 0);

	/* poll channel enables */
	LOG_INF("Poll driver for VW, OOB, and Flash channel enables");
	while (!(chan_ready[ESPI_CHAN_READY_VW_IDX] & chan_ready[ESPI_CHAN_READY_OOB_IDX]
		& chan_ready[ESPI_CHAN_READY_FC_IDX])) {
		if (!chan_ready[ESPI_CHAN_READY_VW_IDX]) {
			if (espi_get_channel_status(espi_dev, ESPI_CHANNEL_VWIRE)) {
				chan_ready[ESPI_CHAN_READY_VW_IDX] = 1;
			}
		}
		if (!chan_ready[ESPI_CHAN_READY_OOB_IDX]) {
			if (espi_get_channel_status(espi_dev, ESPI_CHANNEL_OOB)) {
				chan_ready[ESPI_CHAN_READY_OOB_IDX] = 1;
			}
		}
		if (!chan_ready[ESPI_CHAN_READY_FC_IDX]) {
			if (espi_get_channel_status(espi_dev, ESPI_CHANNEL_FLASH)) {
				chan_ready[ESPI_CHAN_READY_FC_IDX] = 1;
			}
		}
	};
	LOG_INF("VW, OOB, and FC channels all enabled by Host and EC set Ready bits");

	LOG_INF("Call eSPI driver to send VW TARGET_BOOT_LOAD_DONE/STATUS = 1");
	ret = espi_send_vwire(espi_dev, ESPI_VWIRE_SIGNAL_TARGET_BOOT_STS, 1);
	if (ret) {
		LOG_ERR("eSPI driver send ESPI_VWIRE_SIGNAL_TARGET_BOOT_STS=1 error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}
	ret = espi_send_vwire(espi_dev, ESPI_VWIRE_SIGNAL_TARGET_BOOT_DONE, 1);
	if (ret) {
		LOG_ERR("eSPI driver send ESPI_VWIRE_SIGNAL_TARGET_BOOT_DONE=1 error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Poll driver for Peripheral channel enable");
	do {
		if (!chan_ready[ESPI_CHAN_READY_PC_IDX]) {
			if (espi_get_channel_status(espi_dev, ESPI_CHANNEL_PERIPHERAL)) {
				chan_ready[ESPI_CHAN_READY_PC_IDX] = 1;
				chan_en_count++;
			}
		}
	} while (chan_en_count < 4u);
	LOG_INF("Peripheral channel is enabled");

	k_sleep(K_MSEC(1000));

	LOG_INF("Send EC_IRQ=7 Serial IRQ(VWire) to the Host");
	mec_hal_espi_ld_sirq_set(espi_iobase, MEC_ESPI_LDN_EC, 0, 7u);
	mec_hal_espi_gen_ec_sirq(espi_iobase, 1);

	k_sleep(K_MSEC(500));

	LOG_INF("Toggle nSCI VWire");
	vw_state = 0;
	ret = espi_receive_vwire(espi_dev, ESPI_VWIRE_SIGNAL_SCI, &vw_state);
	if (ret) {
		LOG_ERR("Driver receive VWire error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	vw_state ^= BIT(0);
	ret = espi_send_vwire(espi_dev, ESPI_VWIRE_SIGNAL_SCI, vw_state);
	if (ret) {
		LOG_ERR("Driver receive VWire error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	k_sleep(K_MSEC(500)); /* delay due to Host eSPI emulator polling */

	vw_state ^= BIT(0);
	ret = espi_send_vwire(espi_dev, ESPI_VWIRE_SIGNAL_SCI, vw_state);
	if (ret) {
		LOG_ERR("Driver receive VWire error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

#if 0
	LOG_INF("TAF configuration OK. Perform read test 1");

	ret = sample_espi_taf_read_test1(spi_addr, buf1, spi_len);
	if (ret) {
		LOG_ERR("Sample TAF read test 1 failed (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Sample TAF read test 1: PASS");

	LOG_INF("Perform TAF erase 4KB test 1");

	ret = sample_espi_taf_erase_test1(spi_addr, 4096u);
	if (ret) {
		LOG_ERR("Sample TAF erase test 1 failed (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Sample TAF erase test 1: PASS");

	for (uint32_t n = 0; n < spi_len; n++) {
		buf1[n] = (uint8_t)((spi_len - n) & 0xffu);
	}

	LOG_HEXDUMP_INF(buf1, spi_len, "TAF write data");

	ret = sample_espi_taf_write_test1(spi_addr, buf1, spi_len);
	if (ret) {
		LOG_ERR("Sample TAF program test 1 failed (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Sample TAF write test 1: PASS");

	LOG_INF("Begin RPMC tests");
	ret = rpmc_init_keys();
	if (ret) {
		LOG_ERR("TAF RPMC key data init failed (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("RPMC flash has 4 counters: Update root key on each");
	ret = taf_rpmc_check_root_keys(4u);
	if (ret) {
		LOG_ERR("TAF RPMC root key update failed (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("TAF RPMC test1");
	ret = espi_taf_rpmc_test1();
	if (ret) {
		LOG_ERR("TAF RPMC test1 failed (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("TAF RPMC test1 PASS");
#endif
	LOG_INF("Application Done");
	spin_on(256, 0);
	app_main_exit = 1;

	return 0;
}

static void spin_on(uint32_t id, int rval)
{
	spin_val = id;
	ret_val = rval;

	LOG_INF("spin id = %u", id);
	log_panic(); /* flush log buffers */

	while (spin_val) {
		;
	}
}

uint32_t app_flash_jedec_id(uint8_t cs)
{
	if (cs == 0) {
		return flash0_info.jedec_id;
	} else if (cs == 1) {
		return flash1_info.jedec_id;
	}

	return 0;
}

/* The Zephyr Flash driver does not provide an API to read SPI flash status.
 * It does read/write SPI flash status internally but this functionality is not exposed!
 * We must use the SPI driver to read/write SPI flash status.
 */
static int app_read_spi_flash_status_reg(const struct device *spi_dev, const struct spi_config *cfg,
					 uint8_t status_id, uint8_t *status)
{
	uint8_t spi_data[4] = {0};
	int ret = 0;

	if (!spi_dev || !status) {
		return -EINVAL;
	}

	switch (status_id) {
	case 1:
		spi_data[0] = 0x05u;
		break;
	case 2:
		spi_data[0] = 0x35u;
		break;
	case 3:
		spi_data[0] = 0x15u;
		break;
	default:
		return -EINVAL;
	}

	const struct spi_buf txbs[2] = {
		{
			.buf = spi_data,
			.len = 1,
		},
		{
			.buf = NULL,
			.len = 1,
		},
	};

	const struct spi_buf rxbs[2] = {
		{
			.buf = NULL,
			.len = 1,
		},
		{
			.buf = &spi_data[2],
			.len = 1,
		},
	};

	const struct spi_buf_set tx = {
		.buffers = txbs,
		.count = 2,
	};
	const struct spi_buf_set rx = {
		.buffers = rxbs,
		.count = 2,
	};

	ret = spi_transceive(spi_dev, cfg, &tx, &rx);
	if (ret) {
		LOG_ERR("SPI read flash status %u error (%d)", status_id, ret);
		return ret;
	}

	*status = spi_data[2];

	return 0;
}

#if 0
static void pr_mchp_dmaslot_def(void)
{
	LOG_INF("Size of struct mchp_dmaslot is %u bytes", sizeof(struct mchp_dmaslot));
	LOG_INF("Offset of member cfg = 0x%0x", offsetof(struct mchp_dmaslot, cfg));
	LOG_INF("Offset of member extramem = 0x%0x", offsetof(struct mchp_dmaslot, extramem));
	LOG_INF("Offset of member indescs = 0x%0x", offsetof(struct mchp_dmaslot, indescs));
	LOG_INF("Offset of member outdescs = 0x%0x", offsetof(struct mchp_dmaslot, outdescs));
}
#endif

static void pr_mchphash_def(void)
{
	LOG_INF("Size of structure mchphash is %u bytes", sizeof(struct mchphash));
#if 0
	LOG_INF("Offset of member d = 0x%0x", offsetof(struct mchphash, d));
	LOG_INF("Offset of member h = 0x%0x", offsetof(struct mchphash, h));
	LOG_INF("Offset of member algo = 0x%0x", offsetof(struct mchphash, algo));
	LOG_INF("Offset of member cntindescs = 0x%0x", offsetof(struct mchphash, cntindescs));
	LOG_INF("Offset of member dma = 0x%0x", offsetof(struct mchphash, dma));
	LOG_INF("Offset of member feedsz = 0x%0x", offsetof(struct mchphash, feedsz));
#endif
}

static void pr_mchphmac2_def(void)
{
	LOG_INF("Size of structure mchphmac2 is %u bytes", sizeof(struct mchphmac2));
#if 0
	LOG_INF("Offset of member id = 0x%0x", offsetof(struct mchphmac2, id));
	LOG_INF("Offset of member hms = 0x%0x", offsetof(struct mchphmac2, hms));
	LOG_INF("Offset of member rsvd = 0x%0x", offsetof(struct mchphmac2, rsvd));
	LOG_INF("Offset of member c = 0x%0x", offsetof(struct mchphmac2, c));
	LOG_INF("Offset of member s = 0x%0x", offsetof(struct mchphmac2, s));
#endif
}

static bool is_buffer_filled_with(const uint8_t *buf, uint32_t bufsz, uint8_t val)
{
	for (uint32_t n = 0; n < bufsz; n++) {
		if (buf[n] != val) {
			return false;
		}
	}

	return true;
}

/* GPIO callbacks */
static void gpio_cb(const struct device *port,
		    struct gpio_callback *cb,
		    gpio_port_pins_t pins)
{
	if (pins & GPIO_0242_PIN_MASK) {
		int state = gpio_pin_get_dt(&vcc_pwrgd_alt_in_dt);

		LOG_INF("GPIO CB: VCC_PWRGD_ALT = %d", state);
	} else {
		LOG_ERR("GPIO CB: unknown pin 0x%08x", pins);
	}
}

/* eSPI driver callbacks */
static void espi_reset_cb(const struct device *dev,
			  struct espi_callback *cb,
			  struct espi_event ev)
{
	LOG_INF("eSPI CB: eSPI_nReset");
}

static void espi_chan_ready_cb(const struct device *dev,
			       struct espi_callback *cb,
			       struct espi_event ev)
{
	LOG_INF("eSPI CB: channel ready: %u 0x%x 0x%x",
		ev.evt_type, ev.evt_data, ev.evt_details);

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
	LOG_INF("eSPI CB: VW received: %u 0x%x 0x%x",
		ev.evt_type, ev.evt_data, ev.evt_details);
}

static void app_handle_acpi_ec(struct mec_acpi_ec_regs *os_acpi_ec_regs)
{
	uint32_t data;
	uint8_t four_byte_mode, status;

	status = mec_hal_acpi_ec_status(os_acpi_ec_regs);
	four_byte_mode = mec_hal_acpi_ec_is_4byte_mode(os_acpi_ec_regs);

	if (status & MEC_ACPI_EC_STS_CMD) {
		data = mec_hal_acpi_ec_host_to_ec_data_rd8(os_acpi_ec_regs, 0);
		LOG_INF("App ACPI_EC Clear IBF: Command received: 0x%02x", data & 0xffu);
	} else {
		if (four_byte_mode) {
			data = mec_hal_acpi_ec_host_to_ec_data_rd32(os_acpi_ec_regs);
			LOG_INF("App ACPI_EC Clear IBF: 4-byte Data: 0x%04x", data);
		} else {
			data = mec_hal_acpi_ec_host_to_ec_data_rd32(os_acpi_ec_regs);
			LOG_INF("App ACPI_EC Clear IBF: Data: 0x%02x", data & 0xffu);
		}
	}
	mec_hal_acpi_ec_girq_clr(os_acpi_ec_regs, MEC_ACPI_EC_IBF_IRQ | MEC_ACPI_EC_OBE_IRQ);
}

static void espi_periph_cb(const struct device *dev,
			   struct espi_callback *cb,
			   struct espi_event ev)
{
	uint32_t details;

	LOG_INF("eSPI CB: PC: %u 0x%x 0x%x", ev.evt_type, ev.evt_data, ev.evt_details);

	details = ev.evt_details & 0xffu;
	if (details == ESPI_PERIPHERAL_UART) {
		LOG_INF("  PC Host UART");
	} else if (details == ESPI_PERIPHERAL_8042_KBC) {
		LOG_INF("  PC 8042-KBC IBF");
	} else if (details == ESPI_PERIPHERAL_HOST_IO) {
		LOG_INF("  PC Host I/O");
#ifndef CONFIG_ESPI_PERIPHERAL_ACPI_EC_IBF_EVT_DATA
	/* if not defined the driver ISR does not clear ACPI_EC IBF status
	 * resulting in the interrupt not being cleared! Application must handle it.
	 * This means application must know which hardware ACPI_EC instance is being
	 * used. ACPI_EC definition requires reading data OS wrote to clear HW input
	 * buffer full (IBF) status.
	 */
		app_handle_acpi_ec(os_acpi_ec_regs);
#endif
	} else if (details == ESPI_PERIPHERAL_DEBUG_PORT80) {
		LOG_INF("  PC BIOS Debug I/O capture");
	} else if (details == ESPI_PERIPHERAL_HOST_IO_PVT) {
		LOG_INF("  PC Host PVT I/O");
	} else if (details == ESPI_PERIPHERAL_EC_HOST_CMD) {
		LOG_INF("  PC Host EC Cmd");
	} else if (details == ESPI_PERIPHERAL_HOST_MAILBOX) {
		LOG_INF("  PC Mailbox");
	}
}

static void acpi_ec2_cb(const struct device *dev, struct mchp_espi_acpi_ec_event *ev,
			void *user_data)
{
	LOG_INF("ACPI_EC2 CB: flags=0x%02x ev=0x%02x cmd_data=0x%08x",
		ev->flags, ev->ev_type, ev->cmd_data);

}

static void acpi_ec3_cb(const struct device *dev, struct mchp_espi_acpi_ec_event *ev,
			void *user_data)
{
	LOG_INF("ACPI_EC3 CB: flags=0x%02x ev=0x%02x cmd_data=0x%08x",
		ev->flags, ev->ev_type, ev->cmd_data);
}

#if DT_NODE_HAS_STATUS(ACPI_EC4_NODE, okay)
static void acpi_ec4_cb(const struct device *dev, struct mchp_espi_acpi_ec_event *ev,
			void *user_data)
{
	LOG_INF("ACPI_EC3 CB: flags=0x%02x ev=0x%02x cmd_data=0x%08x",
		ev->flags, ev->ev_type, ev->cmd_data);
}
#endif

static void emi0_cb(const struct device *dev, uint32_t emi_mbox_data, void *user_data)
{
	uint32_t data = 0xA5u;

	LOG_INF("EMI0 CB: Host-to-EC MBox data = 0x%0x", emi_mbox_data);
	mchp_espi_pc_emi_request(dev, MCHP_EMI_OPC_MBOX_EC_TO_HOST_WR, &data);
}

static void emi1_cb(const struct device *dev, uint32_t emi_mbox_data, void *user_data)
{
	uint32_t data = 0xB5u;

	LOG_INF("EMI1 CB: Host-to-EC MBox data = 0x%0x", emi_mbox_data);
	mchp_espi_pc_emi_request(dev, MCHP_EMI_OPC_MBOX_EC_TO_HOST_WR, &data);
}
