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
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_mec5_i2c_nl.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#include "test_i2c_target.h"

#define APP_BOARD_HAS_FRAM_ATTACHED

#define I2C_TARGET_NOT_PRESENT_ADDR 0x56
#define I2C_TARGET_MODE_ADDR0 0x40
#define I2C_TARGET_MODE_ADDR1 0x41

/* LTC2489 ADC conversion time using internal clock.
 * If a converstion is in progress LTC2489 will NACK its address.
 */
#define LTC2489_ADC_CONV_TIME_MS 150
#define LTC2489_ADC_READ_RETRIES 10

#define DMAC_NODE	DT_NODELABEL(dmac)
#define I2C0_NODE	DT_ALIAS(i2c0)
#define I2C_NL0_NODE	DT_ALIAS(i2c_nl_0)

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

const struct gpio_dt_spec test_pin1_dt =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, gpios, 0);

const struct gpio_dt_spec test_pin2_dt =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, gpios, 1);

static const struct device *i2c_dev = DEVICE_DT_GET(I2C0_NODE);
static const struct device *i2c_nl_dev = DEVICE_DT_GET(I2C_NL0_NODE);

/* Driver pads the buffer by 4 bytes */
#define I2C_NL_TM_RX_BUF_SIZE (DT_PROP(I2C_NL0_NODE, tm_rx_buf_size) + 8u)

BUILD_ASSERT(I2C_NL_TM_RX_BUF_SIZE >= 4, "I2C-NL0 tm-rx-buf-size property < 4");

/* Access devices on an I2C bus using Device Tree child nodes of the I2C controller */
static const struct i2c_dt_spec pca9555_dts = I2C_DT_SPEC_GET(DT_NODELABEL(pca9555_evb));
static const struct i2c_dt_spec ltc2489_dts = I2C_DT_SPEC_GET(DT_NODELABEL(ltc2489_evb));
#ifdef APP_BOARD_HAS_FRAM_ATTACHED
static const struct i2c_dt_spec mb85rc256v_dts = I2C_DT_SPEC_GET(DT_NODELABEL(mb85rc256v));
#endif

static volatile uint32_t spin_val;
static volatile int ret_val;

static void spin_on(uint32_t id, int rval);
static int config_i2c_device(const struct device *dev, uint32_t i2c_dev_config);

static int test_i2c_fram(const struct i2c_dt_spec *fram_i2c_spec,
			 uint16_t fram_offset, size_t datasz,
			 i2c_callback_t cb, void *userdata,
			 bool async);

static int test_i2c_fram_multi_msg(const struct i2c_dt_spec *fram_i2c_spec,
				   uint16_t fram_offset, size_t datasz,
				   i2c_callback_t cb, void *userdata,
				   bool async);

uint8_t buf1[512];
uint8_t buf2[512];
uint8_t buf3[512];

int main(void)
{
	int ret = 0;
	void (*i2c_cb_fp)(const struct device *, int, void *) = NULL;
	uint32_t i2c_dev_config = 0;
	uint32_t adc_retry_count = 0;
	uint32_t temp = 0;
	uint32_t i2c_nl_wr_len = 0;
	uint32_t i2c_nl_rd_len = 0;
	uint8_t *tm_rxb = NULL;
	uint32_t tm_rxb_sz = 0;
	uint8_t nmsgs = 0;
	uint8_t target_addr = 0;
	struct i2c_msg msgs[4];

	LOG_INF("MEC5 I2C sample: board: %s", DT_N_P_compatible_IDX_0);

	gpio_pin_configure_dt(&test_pin1_dt, GPIO_OUTPUT_HIGH);
	gpio_pin_configure_dt(&test_pin2_dt, GPIO_OUTPUT_HIGH);

	gpio_pin_set_dt(&test_pin1_dt, 1);
	gpio_pin_set_dt(&test_pin2_dt, 1);

	memset(msgs, 0, sizeof(msgs));
	memset(buf1, 0x55, sizeof(buf1));
	memset(buf2, 0x55, sizeof(buf2));

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device is not ready!");
		spin_on((uint32_t)__LINE__, -1);
	}

	i2c_dev_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;

	ret = config_i2c_device(i2c_dev, i2c_dev_config);
	if (ret) {
		LOG_ERR("I2C configuration error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	ret = i2c_get_config(i2c_dev, &temp);
	if (ret) {
		LOG_ERR("I2C get configuration error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	if (temp != i2c_dev_config) {
		LOG_ERR("I2C configuration does not match: orig(0x%08x) returned(0x%08x)\n",
			i2c_dev_config, temp);
		spin_on((uint32_t)__LINE__, ret);
	}

	/* I2C write to non-existent device: check error return and if
	 * future I2C accesses to a real device work.
	 */
	LOG_INF("Attempt I2C transaction to a non-existent address");
	target_addr = I2C_TARGET_NOT_PRESENT_ADDR;
	nmsgs = 1u;
	buf1[0] = 2u;
	buf1[1] = 0x55u;
	buf1[2] = 0xaa;
	msgs[0].buf = buf1;
	msgs[0].len = 3u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer(i2c_dev, msgs, nmsgs, target_addr);
	if (ret != 0) {
		LOG_INF("PASS: Expected API to return an error (%d)", ret);
	} else {
		LOG_ERR("FAIL: I2C API should have returned an error!");
		spin_on((uint32_t)__LINE__, ret);
	}

	for (int i = 0; i < 3; i++) {
		LOG_INF("I2C Write 3 bytes to PCA9555 target device");

		nmsgs = 1u;
		buf1[0] = 2u; /* PCA9555 cmd=2 is output port 0 */
		buf1[1] = 0x55u;
		buf1[2] = 0xaau;
		msgs[0].buf = buf1;
		msgs[0].len = 3u;
		msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

		ret = i2c_transfer_dt(&pca9555_dts, msgs, nmsgs);
		if (ret) {
			LOG_ERR("Loop %d: I2C write to PCA9555 error (%d)", i, ret);
			spin_on((uint32_t)__LINE__, ret);
		}
	}

	LOG_INF("Write 3 bytes to PCA9555 target using multiple write buffers");
	nmsgs = 3u;
	buf1[0] = 2u;
	buf1[1] = 0x33u;
	buf1[2] = 0xcc;
	msgs[0].buf = buf1;
	msgs[0].len = 1u;
	msgs[0].flags = I2C_MSG_WRITE;
	msgs[1].buf = &buf1[1];
	msgs[1].len = 1u;
	msgs[1].flags = I2C_MSG_WRITE;
	msgs[2].buf = &buf1[2];
	msgs[2].len = 1u;
	msgs[2].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&pca9555_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C write multiple buffers to PCA9555 error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Read both PCA9555 input ports");

	/* PCA9555 read protocol: START TX[wrAddr, cmd],
	 * Rpt-START TX[rdAddr],  RX[data,...], STOP
	 */
	nmsgs = 2u;
	buf1[0] = 0u; /* PCA9555 cmd=0 is input port 0 */
	buf2[0] = 0x55u; /* receive buffer */
	buf2[1] = 0x55u;

	msgs[0].buf = buf1;
	msgs[0].len = 1u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf2;
	msgs[1].len = 2u;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&pca9555_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C transfer error: write cmd byte, read 2 data bytes: (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("PCA9555 Port 0 input = 0x%02x  Port 1 input = 0x%02x", buf2[0], buf2[1]);

	LOG_INF("Read both PCA9555 input ports using different buffers for each port");
	nmsgs = 3u;

	buf1[0] = 0u; /* PCA9555 cmd=0 is input port 0 */
	buf2[0] = 0x55u; /* first receive buffer */
	buf2[1] = 0x55u;
	buf2[8] = 0x55u; /* second receive buffer */
	buf2[9] = 0x55u;

	msgs[0].buf = buf1;
	msgs[0].len = 1u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf2;
	msgs[1].len = 1u;
	msgs[1].flags = I2C_MSG_READ;

	msgs[2].buf = &buf2[8];
	msgs[2].len = 1u;
	msgs[2].flags = I2C_MSG_READ | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&pca9555_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C transfer error: 3 messages: (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("PCA9555 Port 0 input = 0x%02x  Port 1 input = 0x%02x", buf2[0], buf2[8]);

	LOG_INF("Select ADC channel 0 in LTC2489. This triggers a conversion!");
	nmsgs = 1;
	buf1[0] = 0xb0u; /* 1011_0000 selects channel 0 as single ended */
	msgs[0].buf = buf1;
	msgs[0].len = 1u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&ltc2489_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C write to set LTC2489 channel failed: (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	/* wait for LTC2489 to finish a conversion */
	k_sleep(K_MSEC(LTC2489_ADC_CONV_TIME_MS));

	/* LTC2489 ADC read protocol is I2C read: START TX[rdAddr]
	 * RX[data[7:0], data[15:8], data[23:16]] STOP
	 */
	LOG_INF("Read 24-bit ADC reading from LTC2489");
	adc_retry_count = 0;
	do {
		buf2[0] = 0x55;
		buf2[1] = 0x55;
		buf2[2] = 0x55;

		nmsgs = 1;
		msgs[0].buf = buf2;
		msgs[0].len = 3u;
		msgs[0].flags = I2C_MSG_READ | I2C_MSG_STOP;

		ret = i2c_transfer_dt(&ltc2489_dts, msgs, nmsgs);
		if (ret) {
			adc_retry_count++;
			LOG_INF("LTC2489 read error (%d)", ret);
		}
	} while ((ret != 0) && (adc_retry_count < LTC2489_ADC_READ_RETRIES));

	if (ret == 0) {
		temp = ((uint32_t)(buf2[0]) + (((uint32_t)(buf2[1])) << 8)
				+ (((uint32_t)(buf2[2])) << 16));
		LOG_INF("LTC2489 reading = 0x%08x", temp);
	}

#ifdef APP_BOARD_HAS_FRAM_ATTACHED
	uint16_t fram_addr = 0;
	uint32_t fram_datasz = 32u;

	LOG_INF("Test I2C FRAM");

	if (!device_is_ready(mb85rc256v_dts.bus)) {
		LOG_ERR("FRAM I2C bus not ready!");
		spin_on((uint32_t)__LINE__, ret);
	}

	ret = config_i2c_device(mb85rc256v_dts.bus, i2c_dev_config);
	if (ret) {
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Attempt I2C transaction to a non-existent address");
	target_addr = I2C_TARGET_NOT_PRESENT_ADDR;
	nmsgs = 1u;
	buf1[0] = 2u;
	buf1[1] = 0x55u;
	buf1[2] = 0xaa;
	msgs[0].buf = buf1;
	msgs[0].len = 3u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer(mb85rc256v_dts.bus, msgs, nmsgs, target_addr);
	if (ret) {
		LOG_INF("API returned error (%d) as expected", ret);
	} else {
		LOG_ERR("Expected an error due to NAK. API returned success");
		spin_on((uint32_t)__LINE__, ret);
	}

	i2c_mchp_nl_clr_buffers(mb85rc256v_dts.bus, 0x55);
	i2c_mchp_nl_clr_debug_data(mb85rc256v_dts.bus);

	ret = test_i2c_fram(&mb85rc256v_dts, fram_addr, fram_datasz, i2c_cb_fp, NULL, false);
	if (ret) {
		spin_on((uint32_t)__LINE__, ret);
	}

	fram_addr = 0x1234u;
	fram_datasz = 64u;
	ret = test_i2c_fram(&mb85rc256v_dts, fram_addr, fram_datasz, i2c_cb_fp, NULL, false);
	if (ret) {
		spin_on((uint32_t)__LINE__, ret);
	}

	fram_addr = 0x100u;
	fram_datasz = 128u;
	ret = test_i2c_fram(&mb85rc256v_dts, fram_addr, fram_datasz, i2c_cb_fp, NULL, false);
	if (ret) {
		spin_on((uint32_t)__LINE__, ret);
	}

	fram_addr = 0x300u;
	fram_datasz = 512u;
	ret = test_i2c_fram(&mb85rc256v_dts, fram_addr, fram_datasz, i2c_cb_fp, NULL, false);
	if (ret) {
		spin_on((uint32_t)__LINE__, ret);
	}

	ret = test_i2c_fram(&mb85rc256v_dts, fram_addr, fram_datasz, i2c_cb_fp, NULL, false);
	if (ret) {
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Test multi-message");
	fram_addr = 0x600u;
	fram_datasz = 16u;
	ret = test_i2c_fram_multi_msg(&mb85rc256v_dts, fram_addr, fram_datasz,
				      i2c_cb_fp, NULL, false);
	if (ret) {
		spin_on((uint32_t)__LINE__, ret);
	}
#endif /* APP_BOARD_HAS_FRAM_ATTACHED */

	ret = test_i2c_target_register(i2c_nl_dev, true, 0);
	if (ret) {
		LOG_ERR("Failed to register target address 0 on device %p: error(%d)",
			mb85rc256v_dts.bus, ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Target 0 registered");

	LOG_INF("I2C-NL0 TM RX buffer size is %u bytes", I2C_NL_TM_RX_BUF_SIZE);

	i2c_nl_wr_len = I2C_NL_TM_RX_BUF_SIZE - 5u;
	LOG_INF("Write %u bytes to I2C-NL0 target", i2c_nl_wr_len);

	for (int i = 0; i < i2c_nl_wr_len; i++) {
		buf1[i] = (uint8_t)(i % 256);
	}

	ret = test_i2c_target_write(i2c_dev, mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0,
				    buf1, i2c_nl_wr_len);
	if (ret) {
		LOG_ERR("Target write error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	i2c_mchp_nl_clr_buffers(mb85rc256v_dts.bus, 0x55);
	i2c_mchp_nl_clr_debug_data(mb85rc256v_dts.bus);

	i2c_nl_wr_len = I2C_NL_TM_RX_BUF_SIZE - 4u;
	LOG_INF("Write %u bytes to I2C-NL0 target", i2c_nl_wr_len);

	for (int i = 0; i < i2c_nl_wr_len; i++) {
		buf1[i] = (uint8_t)(i % 256);
	}

	ret = test_i2c_target_write(i2c_dev, mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0,
				    buf1, i2c_nl_wr_len);
	if (ret) {
		LOG_ERR("Target write error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	i2c_mchp_nl_clr_buffers(mb85rc256v_dts.bus, 0x55);
	i2c_mchp_nl_clr_debug_data(mb85rc256v_dts.bus);

	i2c_nl_wr_len = I2C_NL_TM_RX_BUF_SIZE - 3u;
	LOG_INF("Write %u bytes to I2C-NL0 target", i2c_nl_wr_len);

	for (int i = 0; i < i2c_nl_wr_len; i++) {
		buf1[i] = (uint8_t)(i % 256);
	}

	ret = test_i2c_target_write(i2c_dev, mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0,
				    buf1, i2c_nl_wr_len);
	if (ret) {
		LOG_ERR("Target write error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	i2c_mchp_nl_clr_buffers(mb85rc256v_dts.bus, 0x55);
	i2c_mchp_nl_clr_debug_data(mb85rc256v_dts.bus);

	i2c_nl_wr_len = I2C_NL_TM_RX_BUF_SIZE - 2u;
	LOG_INF("Write %u bytes to I2C-NL0 target", i2c_nl_wr_len);

	for (int i = 0; i < i2c_nl_wr_len; i++) {
		buf1[i] = (uint8_t)(i % 256);
	}

	ret = test_i2c_target_write(i2c_dev, mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0,
				    buf1, i2c_nl_wr_len);
	if (ret) {
		LOG_ERR("Target write error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	i2c_mchp_nl_clr_buffers(mb85rc256v_dts.bus, 0x55);
	i2c_mchp_nl_clr_debug_data(mb85rc256v_dts.bus);

	i2c_nl_wr_len = I2C_NL_TM_RX_BUF_SIZE - 1u;
	LOG_INF("Write %u bytes to I2C-NL0 target", i2c_nl_wr_len);

	for (int i = 0; i < i2c_nl_wr_len; i++) {
		buf1[i] = (uint8_t)(i % 256);
	}

	ret = test_i2c_target_write(i2c_dev, mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0,
				    buf1, i2c_nl_wr_len);
	if (ret) {
		LOG_ERR("Target write error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	i2c_mchp_nl_clr_buffers(mb85rc256v_dts.bus, 0x55);
	i2c_mchp_nl_clr_debug_data(mb85rc256v_dts.bus);

	i2c_nl_wr_len = I2C_NL_TM_RX_BUF_SIZE;
	LOG_INF("Write %u bytes to I2C-NL0 target", i2c_nl_wr_len);

	for (int i = 0; i < i2c_nl_wr_len; i++) {
		buf1[i] = (uint8_t)(i % 256);
	}

	ret = test_i2c_target_write(i2c_dev, mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0,
				    buf1, i2c_nl_wr_len);
	if (ret) {
		LOG_ERR("Target write error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	/* Target Read tests */
	i2c_mchp_nl_clr_buffers(mb85rc256v_dts.bus, 0x55);
	i2c_mchp_nl_clr_debug_data(mb85rc256v_dts.bus);

	i2c_nl_rd_len = 3u;
	LOG_INF("Target read %u bytes", i2c_nl_rd_len);

	memset(buf1, 0x55, sizeof(buf1));
	memset(buf2, 0xaa, sizeof(buf1));

	buf1[0] = 0x11;
	buf1[1] = 0x12;
	buf1[2] = 0x13;

	test_i2c_target_read_data_init(buf1, i2c_nl_rd_len);

	ret = test_i2c_target_read(i2c_dev, mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0,
				   buf2, i2c_nl_rd_len);
	if (ret) {
		LOG_ERR("Target read error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	while (!test_i2c_target_read_done(mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0)) {
		;
	}

	LOG_INF("Target read of %u bytes done", i2c_nl_rd_len);

	ret = memcmp(buf1, buf2, i2c_nl_rd_len);
	if (ret == 0) {
		LOG_INF("Target read %u bytes data match: PASS", i2c_nl_rd_len);
	} else {
		LOG_INF("Target read %u bytes data mismatch: FAIL", i2c_nl_rd_len);
	}

	/* read 0xf8(248) bytes */
	i2c_mchp_nl_clr_buffers(mb85rc256v_dts.bus, 0x55);
	i2c_mchp_nl_clr_debug_data(mb85rc256v_dts.bus);

	i2c_nl_rd_len = 248u;
	LOG_INF("Target read %u bytes", i2c_nl_rd_len);

	memset(buf1, 0x55, sizeof(buf1));
	memset(buf2, 0xaa, sizeof(buf1));

	for (int i = 0; i < i2c_nl_rd_len; i++) {
		buf1[i] = (uint8_t)(i % 256);
	}

	test_i2c_target_read_data_init(buf1, i2c_nl_rd_len);

	ret = test_i2c_target_read(i2c_dev, mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0,
				   buf2, i2c_nl_rd_len);
	if (ret) {
		LOG_ERR("Target read error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	while (!test_i2c_target_read_done(mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0)) {
		;
	}

	LOG_INF("Target read of %u bytes done", i2c_nl_rd_len);

	ret = memcmp(buf1, buf2, i2c_nl_rd_len);
	if (ret == 0) {
		LOG_INF("Target read %u bytes data match: PASS", i2c_nl_rd_len);
	} else {
		LOG_INF("Target read %u bytes data mismatch: FAIL", i2c_nl_rd_len);
	}

	/* read 501 bytes */
	i2c_mchp_nl_clr_buffers(mb85rc256v_dts.bus, 0x55);
	i2c_mchp_nl_clr_debug_data(mb85rc256v_dts.bus);

	i2c_nl_rd_len = 501u;
	LOG_INF("Target read %u bytes", i2c_nl_rd_len);

	memset(buf1, 0x55, sizeof(buf1));
	memset(buf2, 0xaa, sizeof(buf1));

	for (int i = 0; i < i2c_nl_rd_len; i++) {
		buf1[i] = (uint8_t)(i % 256);
	}

	test_i2c_target_read_data_init(buf1, i2c_nl_rd_len);

	ret = test_i2c_target_read(i2c_dev, mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0,
				   buf2, i2c_nl_rd_len);
	if (ret) {
		LOG_ERR("Target read error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	while (!test_i2c_target_read_done(mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0)) {
		;
	}

	LOG_INF("Target read of %u bytes done", i2c_nl_rd_len);

	ret = memcmp(buf1, buf2, i2c_nl_rd_len);
	if (ret == 0) {
		LOG_INF("Target read %u bytes data match: PASS", i2c_nl_rd_len);
	} else {
		LOG_INF("Target read %u bytes data mismatch: FAIL", i2c_nl_rd_len);
	}

	/* read 5 bytes and intentionally do not supply an application buffer: error path in driver */
	i2c_mchp_nl_clr_buffers(mb85rc256v_dts.bus, 0x55);
	i2c_mchp_nl_clr_debug_data(mb85rc256v_dts.bus);

	i2c_nl_rd_len = 5u;
	LOG_INF("Target read %u bytes with app returning error on buffer request cb", i2c_nl_rd_len);

	memset(buf1, 0x55, sizeof(buf1));
	memset(buf2, 0xaa, sizeof(buf1));

	for (int i = 0; i < i2c_nl_rd_len; i++) {
		buf1[i] = (uint8_t)(i % 256);
	}

	test_i2c_target_read_data_init(NULL, 0);

	ret = test_i2c_target_read(i2c_dev, mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0,
				   buf2, i2c_nl_rd_len);
	if (ret) {
		LOG_ERR("Target read error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	while (!test_i2c_target_read_done(mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0)) {
		;
	}

	LOG_INF("Target read of %u bytes done", i2c_nl_rd_len);

	ret = memcmp(buf1, buf2, i2c_nl_rd_len);
	if (ret == 0) {
		LOG_INF("Target read %u bytes data match: PASS", i2c_nl_rd_len);
	} else {
		LOG_INF("Target read %u bytes data mismatch: EXPECT FAIL", i2c_nl_rd_len);
	}

	i2c_mchp_nl_clr_buffers(mb85rc256v_dts.bus, 0x55);
	i2c_mchp_nl_clr_debug_data(mb85rc256v_dts.bus);

	/* Break the driver: Target START wrAddr wrData1 Rpt-START rdAddr rdData1 */
	i2c_nl_wr_len = 1u;
	i2c_nl_rd_len = 3u;
	LOG_INF("Target I2C Combined write %u bytes, read %u bytes",
		i2c_nl_wr_len, i2c_nl_rd_len);

	memset(buf1, 0x55, sizeof(buf1));
	memset(buf2, 0xaa, sizeof(buf1));
	memset(buf3, 0xbb, sizeof(buf3));

	buf1[0] = 0x11u;
	buf3[0] = 0x31u; /* data from target */
	buf3[1] = 0x32u;
	buf3[2] = 0x33u;

	test_i2c_target_read_data_init(buf3, i2c_nl_rd_len);

	ret = test_i2c_target_combined(i2c_dev, mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0,
				       buf1, i2c_nl_wr_len, buf2, i2c_nl_rd_len);
	if (ret) {
		LOG_ERR("Test I2C target combined error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	while (!test_i2c_target_read_done(mb85rc256v_dts.bus, I2C_TARGET_MODE_ADDR0)) {
		;
	}

	LOG_INF("I2C Combined write-read done");

	ret = memcmp(buf2, buf3, i2c_nl_rd_len);
	if (ret == 0) {
		LOG_INF("I2C Combined write %u bytes, read %u bytes: PASS",
			i2c_nl_wr_len, i2c_nl_rd_len);
	} else {
		LOG_INF("I2C Combined write %u bytes, read %u bytes: data mismatch FAIL",
			i2c_nl_wr_len, i2c_nl_rd_len);
	}

	i2c_mchp_nl_clr_buffers(mb85rc256v_dts.bus, 0x55);
	i2c_mchp_nl_clr_debug_data(mb85rc256v_dts.bus);

	LOG_INF("Target Combined: Write 2, Write 3");

	test_i2c_target_wr_init();

	memset(buf1, 0x55, sizeof(buf1));
	memset(buf2, 0xAA, sizeof(buf2));

	buf1[0] = 0x11;
	buf1[1] = 0x12;
	buf2[0] = 0x21;
	buf2[1] = 0x22;
	buf2[2] = 0x23;

	msgs[0].buf = buf1;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;
	msgs[1].buf = buf2;
	msgs[1].len = 3u;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_RESTART | I2C_MSG_STOP;

	/* synchronous transfer */
	ret = i2c_transfer(i2c_dev, msgs, 2, I2C_TARGET_MODE_ADDR0);
	if (ret) {
		LOG_ERR("Target I2C combined error (%d)", ret);
	}

	tm_rxb = NULL;
	tm_rxb_sz = 0;

	ret = i2c_test_get_tm_rx_buf(&tm_rxb, &tm_rxb_sz);
	if (ret) {
		LOG_ERR("Get test app TM RX buffer error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	if ((memcmp(tm_rxb, buf1, 2) == 0) && (memcmp(tm_rxb + 2, buf2, 3) == 0)) {
		LOG_INF("I2C Combined Write 2, Write 3: PASS");
	} else {
		LOG_ERR("I2C Combined Write 2, Write 3 data mismatch: FAIL");
	}

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

static int config_i2c_device(const struct device *dev, uint32_t i2c_dev_config)
{
	uint32_t temp = 0;
	int ret = 0;

	if (!dev) {
		return -EINVAL;
	}

	ret = i2c_configure(dev, i2c_dev_config);
	if (ret) {
		LOG_ERR("I2C configuration error (%d)", ret);
		return  ret;
	}

	ret = i2c_get_config(dev, &temp);
	if (ret) {
		LOG_ERR("I2C get configuration error (%d)", ret);
		return ret;
	}

	if (temp != i2c_dev_config) {
		ret = -EIO;
		LOG_ERR("I2C configuration does not match: orig(0x%08x) returned(0x%08x)\n",
			i2c_dev_config, temp);
	}

	return ret;
}

#ifdef APP_BOARD_HAS_FRAM_ATTACHED
static int test_i2c_fram(const struct i2c_dt_spec *fram_i2c_spec,
			 uint16_t fram_offset, size_t datasz,
			 i2c_callback_t cb, void *userdata,
			 bool async)
{
	struct i2c_msg msgs[4] = { 0 };
	int ret = 0;
	uint8_t nmsgs = 0;

	if (!fram_i2c_spec) {
		LOG_ERR("I2C FRAM test bad i2c_dt_spec parameter");
		return -EINVAL;
	}

	if (!datasz) {
		LOG_INF("I2C FRAM test data size is 0. Nothing to do");
		return 0;
	}

	if (((uint32_t)fram_offset + datasz) > (32u * 1024u)) {
		LOG_ERR("I2C FRAM test: offset + datasz overflows 32KB FRAM size");
		return -EMSGSIZE;
	}

	if (datasz > sizeof(buf2)) {
		LOG_ERR("I2C FRAM test requested data size exceeds "
			"test buffer size of %u bytes", sizeof(buf2));
		return -EMSGSIZE;
	}

	for (size_t n = 0; n < datasz; n++) {
		buf2[n] = (uint8_t)(n % 256u);
	}

	buf1[0] = (uint8_t)((fram_offset >> 8) & 0xffu); /* address b[15:8] */
	buf1[1] = (uint8_t)((fram_offset) & 0xffu); /* address b[7:0] */

	LOG_INF("MB85RC256V FRAM write %u bytes to offset 0x%x", datasz, fram_offset);

	msgs[0].buf = buf1;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf2;
	msgs[1].len = datasz;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;


	nmsgs = 2;
	ret = i2c_transfer_dt(fram_i2c_spec, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C API for FRAM write returned error %d", ret);
		return ret;
	}

	/* fill receive buffer with 0x55 */
	memset(buf3, 0x55, sizeof(buf3));

	LOG_INF("MB85RC256V FRAM read %u bytes from offset 0x%x", datasz, fram_offset);

	/* I2C Combined Write FRAM offset and read data */
	msgs[0].buf = buf1;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf3;
	msgs[1].len = datasz;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	nmsgs = 2;
	ret = i2c_transfer_dt(fram_i2c_spec, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C API for FRAM write-read returned error %d", ret);
		return ret;
	}

	ret = memcmp(buf2, buf3, datasz);
	if (ret == 0) {
		LOG_INF("I2C FRAM Test write data and read back at "
			"offset 0x%0x length %u: PASS", fram_offset, datasz);
	} else {
		LOG_ERR("I2C FRAM Test write data and read back FAIL");
	}

	return ret;
}

/* Trigger driver to parse messages into more than one group */
static int test_i2c_fram_multi_msg(const struct i2c_dt_spec *fram_i2c_spec,
				   uint16_t fram_offset, size_t datasz,
				   i2c_callback_t cb, void *userdata,
				   bool async)
{
	struct i2c_msg msgs[4] = { 0 };
	int ret = 0;
	uint8_t nmsgs = 0;

	if (!fram_i2c_spec) {
		LOG_ERR("I2C FRAM test bad i2c_dt_spec parameter");
		return -EINVAL;
	}

	if (!datasz) {
		LOG_INF("I2C FRAM test data size is 0. Nothing to do");
		return 0;
	}

	if (((uint32_t)fram_offset + datasz) > (32u * 1024u)) {
		LOG_ERR("I2C FRAM test: offset + datasz overflows 32KB FRAM size");
		return -EMSGSIZE;
	}

	if (datasz > sizeof(buf2)) {
		LOG_ERR("I2C FRAM test requested data size exceeds "
			"test buffer size of %u bytes", sizeof(buf2));
		return -EMSGSIZE;
	}

	/* fill receive buffer with 0x55 */
	memset(buf2, 0x55, sizeof(buf2));
	memset(buf3, 0x55, sizeof(buf3));

	for (size_t n = 0; n < datasz; n++) {
		buf2[n] = (uint8_t)(n % 256u);
	}

	buf1[0] = (uint8_t)((fram_offset >> 8) & 0xffu); /* address b[15:8] */
	buf1[1] = (uint8_t)((fram_offset) & 0xffu); /* address b[7:0] */

	LOG_INF("MB85RC256V FRAM write %u bytes to offset 0x%x and read back", datasz, fram_offset);

	msgs[0].buf = buf1;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf2;
	msgs[1].len = datasz;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	msgs[2].buf = buf1;
	msgs[2].len = 2u;
	msgs[2].flags = I2C_MSG_WRITE;

	msgs[3].buf = buf3;
	msgs[3].len = datasz;
	msgs[3].flags = I2C_MSG_READ | I2C_MSG_STOP;

	nmsgs = 4;
	ret = i2c_transfer_dt(fram_i2c_spec, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C API passed multi-messages returned error %d", ret);
		return ret;
	}

	ret = memcmp(buf2, buf3, datasz);
	if (ret == 0) {
		LOG_INF("I2C FRAM Test write data and read back at "
			"offset 0x%0x length %u: PASS", fram_offset, datasz);
	} else {
		LOG_ERR("I2C FRAM Test write data and read back FAIL");
	}

	return ret;
}
#endif /* APP_BOARD_HAS_FRAM_ATTACHED */
