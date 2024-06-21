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
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define APP_BOARD_HAS_FRAM_ATTACHED

#define I2C_TARGET_NOT_PRESENT_ADDR 0x56

/* LTC2489 ADC conversion time using internal clock.
 * If a converstion is in progress LTC2489 will NACK its address.
 */
#define LTC2489_ADC_CONV_TIME_MS 150
#define LTC2489_ADC_READ_RETRIES 10

#define I2C0_NODE	DT_ALIAS(i2c0)
#define I2C_NL0_NODE	DT_ALIAS(i2c_nl_0)

#define I2C0_REG_ADDR		DT_REG_ADDR(I2C0_NODE)
#define I2C_NL0_REG_ADDR	DT_REG_ADDR(I2C_NL0_NODE)

static const struct device *i2c_dev = DEVICE_DT_GET(I2C0_NODE);
static const struct device *i2c_nl_dev = DEVICE_DT_GET(I2C_NL0_NODE);

/* Access devices on an I2C bus using Device Tree child nodes of the I2C controller */
static const struct i2c_dt_spec pca9555_dts = I2C_DT_SPEC_GET(DT_NODELABEL(pca9555_evb));
static const struct i2c_dt_spec ltc2489_dts = I2C_DT_SPEC_GET(DT_NODELABEL(ltc2489_evb));
#ifdef APP_BOARD_HAS_FRAM_ATTACHED
#define I2C_FRAM_PARENT DT_PARENT(DT_NODELABEL(mb85rc256v))
#define I2C_FRAM_PARENT_REG_ADDR DT_REG_ADDR(I2C_FRAM_PARENT)
static const struct i2c_dt_spec mb85rc256v_dts = I2C_DT_SPEC_GET(DT_NODELABEL(mb85rc256v));
#endif

static volatile uint32_t spin_val;
static volatile int ret_val;

static void spin_on(uint32_t id, int rval);
static int config_i2c_device(const struct device *dev, uint32_t i2c_dev_config);
static int test_i2c_nl(const struct device *dev);
#ifdef CONFIG_I2C_CALLBACK
static int test_i2c_nl_cb(const struct device *dev);
#endif

#define MEC_I2C_BUF_LEN 512

uint8_t buf1[MEC_I2C_BUF_LEN];
uint8_t buf2[MEC_I2C_BUF_LEN];
uint8_t buf3[MEC_I2C_BUF_LEN];

int main(void)
{
	int ret = 0;
	uint32_t i2c_dev_config = 0;
	uint32_t adc_retry_count = 0;
	uint32_t temp = 0;
	uint8_t nmsgs = 0;
	uint8_t target_addr = 0;
	struct i2c_msg msgs[4];

	LOG_INF("MEC5 I2C sample: board: %s", DT_N_P_compatible_IDX_0);
	LOG_INF("I2C0 Controller reg address:     0x%08x", I2C0_REG_ADDR);
	LOG_INF("I2C_NL0 Controller reg address:  0x%08x", I2C_NL0_REG_ADDR);
	LOG_INF("FRAM I2C Controller reg address: 0x%08x", I2C_FRAM_PARENT_REG_ADDR);

	memset(msgs, 0, sizeof(msgs));
	memset(buf1, 0x55, sizeof(buf1));
	memset(buf2, 0x55, sizeof(buf2));
	memset(buf3, 0x55, sizeof(buf3));

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
		spin_on(10u, ret);
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
#if I2C_FRAM_PARENT_REG_ADDR == I2C0_REG_ADDR
	/* I2C FRAM tests */
	uint32_t fram_nbytes = 64u;
	uint16_t fram_mem_addr = 0x1234u;

	LOG_INF("MB85RC256V FRAM write %u bytes to offset 0x%x", fram_nbytes, fram_mem_addr);

	for (temp = 0; temp < fram_nbytes; temp++) {
		buf2[temp] = (uint8_t)((temp + 1u) & 0xffu);
	}

	nmsgs = 2;
	buf1[0] = (uint8_t)((fram_mem_addr >> 8) & 0xffu); /* address b[15:8] */
	buf1[1] = (uint8_t)((fram_mem_addr) & 0xffu); /* address b[7:0] */

	msgs[0].buf = buf1;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;
	msgs[1].buf = buf2;
	msgs[1].len = fram_nbytes;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&mb85rc256v_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C API for FRAM write returned error %d", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("MB85RC256V FRAM read %u bytes from offset 0x%x", fram_nbytes, fram_mem_addr);
	memset(buf3, 0x55, sizeof(buf3));

	nmsgs = 2;
	buf1[0] = (uint8_t)((fram_mem_addr >> 8) & 0xffu); /* address b[15:8] */
	buf1[1] = (uint8_t)((fram_mem_addr) & 0xffu); /* address b[7:0] */

	msgs[0].buf = buf1;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;
	msgs[1].buf = buf3;
	msgs[1].len = fram_nbytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&mb85rc256v_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C API for FRAM read returned error %d", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	ret = memcmp(buf2, buf3, fram_nbytes);
	if (ret == 0) {
		LOG_INF("Compare read of data written to FRAM matches: PASS");
	} else {
		LOG_ERR("Compare read of data written to FRAM has mismatch: FAIL");
	}
#endif /* I2C_FRAM_PARENT_REG_ADDR == I2C0_REG_ADDR */
#endif /* APP_BOARD_HAS_FRAM_ATTACHED */

	ret = test_i2c_nl(i2c_nl_dev);
	if (ret) {
		LOG_ERR("I2C-NL test failed");
	}

#ifdef CONFIG_I2C_CALLBACK
	ret = test_i2c_nl_cb(i2c_nl_dev);
	if (ret) {
		LOG_ERR("I2C-NL callback mode test failed");
	}
#endif

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

#if 0 /* TODO new stuff #ifdef APP_BOARD_HAS_FRAM_ATTACHED */
static int i2c_nl_test_fram_write(const struct i2c_dt_spec *spec, uint16_t fram_mem_addr,
				  const uint8_t *src, size_t srcsz, uint8_t *dest)
{
	struct i2c_msg msgs[4] = {0};
	int ret = 0;
	uint8_t nmsgs = 0;

	if (!spec || !src || !srcsz || !dest) {
		return -EINVAL;
	}

	buf1[0] = (uint8_t)((fram_mem_addr >> 8) & 0xffu); /* address b[15:8] */
	buf1[1] = (uint8_t)((fram_mem_addr) & 0xffu); /* address b[7:0] */

	msgs[0].buf = buf1;
	msgs[0].len = 2;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = (uint8_t *)src;
	msgs[1].len = srcsz;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	nmsgs = 2;
	ret = i2c_transfer_dt(&mb85rc256v_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C API for FRAM write returned error %d", ret);
		return ret;
	}

	return 0;
}
#endif

static int test_i2c_nl(const struct device *idev)
{
	uint32_t i2c_nl_dev_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
	int ret = 0;
	uint8_t nmsgs = 0;
	uint16_t target_addr = 0;
#ifdef APP_BOARD_HAS_FRAM_ATTACHED
	uint16_t fram_mem_addr = 0;
	uint32_t fram_nbytes = 0;
	uint32_t temp = 0;
#endif
	struct i2c_msg msgs[4];

	if (!idev) {
		return -EINVAL;
	}

	if (!device_is_ready(idev)) {
		LOG_ERR("I2C-NL device is not ready!");
		return -EIO;
	}

	ret = config_i2c_device(i2c_nl_dev, i2c_nl_dev_config);
	if (ret) {
		LOG_ERR("I2C-NL configuration error (%d)", ret);
		return ret;
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

	ret = i2c_transfer(idev, msgs, nmsgs, target_addr);
	if (ret) {
		LOG_INF("API returned error (%d) as expected", ret);
	} else {
		LOG_ERR("Expected an error due to NAK. API returned success");
		return -EIO;
	}

#ifdef APP_BOARD_HAS_FRAM_ATTACHED
	fram_nbytes = 32u;
	fram_mem_addr = 0x1234u;

	LOG_INF("MB85RC256V FRAM write %u bytes to offset 0x%x", fram_nbytes, fram_mem_addr);

	nmsgs = 1;
	buf1[0] = (uint8_t)((fram_mem_addr >> 8) & 0xffu); /* address b[15:8] */
	buf1[1] = (uint8_t)((fram_mem_addr) & 0xffu); /* address b[7:0] */

	for (temp = 0; temp < fram_nbytes; temp++) {
		buf1[temp + 2] = (uint8_t)((temp + 0x10) & 0xffu);
	}

	/* NOTE: Do we want to support two write messages?
	 * if len(msg1) + len(msg2) < len(xfrbuf-1) combine then in driver xfrbuf
	 * else more complicated
	 *   len(msg1) may be > len(xfrbuf-1)
	 */
	msgs[0].buf = buf1;
	msgs[0].len = fram_nbytes + 2u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&mb85rc256v_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C API for FRAM write returned error %d", ret);
		return ret;
	}

	LOG_INF("MB85RC256V FRAM read %u bytes from offset 0x%x", fram_nbytes, fram_mem_addr);

	nmsgs = 2;
	buf2[0] = (uint8_t)((fram_mem_addr >> 8) & 0xffu); /* address b[15:8] */
	buf2[1] = (uint8_t)((fram_mem_addr) & 0xffu); /* address b[7:0] */

	msgs[0].buf = buf2;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf3;
	msgs[1].len = fram_nbytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&mb85rc256v_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C API for FRAM write returned error %d", ret);
		return ret;
	}

	ret = memcmp(&buf1[2], buf3, fram_nbytes);
	if (ret == 0) {
		LOG_INF("FRAM read back of 32 bytes matches written data");
	} else {
		LOG_ERR("FRAM read back mismatch");
	}

	if (sizeof(buf1) < ((2 * CONFIG_I2C_MCHP_MEC5_NL_BUFFER_SIZE) + 2u)) {
		LOG_ERR("Test error: require buf1 and buf2 sizes > "
			"(2 * I2C-NL driver buffer size) + 2");
		return -EINVAL;
	}

	fram_nbytes = 2 * CONFIG_I2C_MCHP_MEC5_NL_BUFFER_SIZE;
	for (temp = 0; temp < fram_nbytes; temp++) {
		buf1[temp + 2] = (uint8_t)(temp & 0xffu);
	}

	/* FRAM offset 0 */
	fram_mem_addr = 0;
	buf1[0] = (uint8_t)((fram_mem_addr >> 8) & 0xffu); /* address b[15:8] */
	buf1[1] = (uint8_t)((fram_mem_addr) & 0xffu); /* address b[7:0] */

	LOG_INF("MB85RC256V FRAM write %u bytes to offset 0x%x", fram_nbytes, fram_mem_addr);

	msgs[0].buf = buf1;
	msgs[0].len = fram_nbytes + 2u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	nmsgs = 1;

	ret = i2c_transfer_dt(&mb85rc256v_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C API for FRAM write returned error %d", ret);
		return ret;
	}

	LOG_INF("MB85RC256V FRAM read %u bytes from offset 0x%x", fram_nbytes, fram_mem_addr);

	nmsgs = 2;
	buf2[0] = (uint8_t)((fram_mem_addr >> 8) & 0xffu); /* address b[15:8] */
	buf2[1] = (uint8_t)((fram_mem_addr) & 0xffu); /* address b[7:0] */

	msgs[0].buf = buf2;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf3;
	msgs[1].len = fram_nbytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&mb85rc256v_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C API for FRAM write returned error %d", ret);
		return ret;
	}

	ret = memcmp(&buf1[2], buf3, fram_nbytes);
	if (ret == 0) {
		LOG_INF("FRAM read back of %u bytes matches written data", fram_nbytes);
	} else {
		LOG_ERR("FRAM read back mismatch");
	}

	/* Size > 8-bit to test driver using EXTLEN register correctly */
	memset(buf3, 0, sizeof(buf3));

	fram_nbytes = 397u;
	for (temp = 0; temp < fram_nbytes; temp++) {
		buf1[temp + 2] = (uint8_t)(temp & 0xffu);
	}

	/* FRAM offset 0 */
	fram_mem_addr = 0x2000;
	buf1[0] = (uint8_t)((fram_mem_addr >> 8) & 0xffu); /* address b[15:8] */
	buf1[1] = (uint8_t)((fram_mem_addr) & 0xffu); /* address b[7:0] */

	LOG_INF("MB85RC256V FRAM write %u bytes to offset 0x%x", fram_nbytes, fram_mem_addr);

	msgs[0].buf = buf1;
	msgs[0].len = fram_nbytes + 2u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	nmsgs = 1;

	ret = i2c_transfer_dt(&mb85rc256v_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C API for FRAM write returned error %d", ret);
		return ret;
	}

	LOG_INF("MB85RC256V FRAM read %u bytes from offset 0x%x", fram_nbytes, fram_mem_addr);

	nmsgs = 2;
	buf2[0] = (uint8_t)((fram_mem_addr >> 8) & 0xffu); /* address b[15:8] */
	buf2[1] = (uint8_t)((fram_mem_addr) & 0xffu); /* address b[7:0] */

	msgs[0].buf = buf2;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf3;
	msgs[1].len = fram_nbytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&mb85rc256v_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C API for FRAM write returned error %d", ret);
		return ret;
	}

	ret = memcmp(&buf1[2], buf3, fram_nbytes);
	if (ret == 0) {
		LOG_INF("FRAM read back of %u bytes matches written data", fram_nbytes);
	} else {
		LOG_ERR("FRAM read back mismatch");
	}
#endif
	return 0;
}

#ifdef CONFIG_I2C_CALLBACK

atomic_t i2c_nl_cb_count;
static int i2c_nl_cb_result;
static uint32_t i2c_nl_cb_user_data[3];

static void i2c_nl_async_cb(const struct device *dev, int result, void *data)
{
	uint32_t *ud = (uint32_t *)data;

	LOG_INF("I2C NL CB result = %d", result);
	if (ud) {
		LOG_INF("target param: 0x%0x nbytes: %u bufAddr: 0x%0x", ud[0], ud[1], ud[2]);
	}
	i2c_nl_cb_result = result;
	atomic_inc(&i2c_nl_cb_count);
}

static int test_i2c_nl_cb(const struct device *idev)
{
	uint32_t i2c_nl_dev_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
	int ret = 0;
	uint8_t nmsgs = 0;
	uint16_t target_addr = 0;
	atomic_t prev_i2c_nl_count = 0;
#ifdef APP_BOARD_HAS_FRAM_ATTACHED
	uint16_t fram_mem_addr = 0;
	uint32_t fram_nbytes = 0;
	uint32_t temp = 0;
#endif
	struct i2c_msg msgs[4];

	if (!idev) {
		return -EINVAL;
	}

	if (!device_is_ready(idev)) {
		LOG_ERR("I2C-NL device is not ready!");
		return -EIO;
	}

	ret = config_i2c_device(i2c_nl_dev, i2c_nl_dev_config);
	if (ret) {
		LOG_ERR("I2C-NL configuration error (%d)", ret);
		return ret;
	}

	atomic_set(&i2c_nl_cb_count, 0);
	prev_i2c_nl_count = atomic_get(&i2c_nl_cb_count);

	LOG_INF("Attempt I2C CB transaction to a non-existent address");

	target_addr = I2C_TARGET_NOT_PRESENT_ADDR;
	nmsgs = 1u;
	buf1[0] = 2u;
	buf1[1] = 0x55u;
	buf1[2] = 0xaa;
	msgs[0].buf = buf1;
	msgs[0].len = 3u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer_cb(idev, msgs, nmsgs, target_addr,
			      i2c_nl_async_cb, i2c_nl_cb_user_data);
	if (ret) {
		LOG_INF("I2C transfer callback API returned unexpected error (%d)", ret);
		return ret;
	}

	while (atomic_get(&i2c_nl_cb_count) == prev_i2c_nl_count) {
		;
	}

	LOG_INF("I2C transfer callback fired: status = %d", i2c_nl_cb_result);

#ifdef APP_BOARD_HAS_FRAM_ATTACHED
	fram_nbytes = 193u;
	fram_mem_addr = 0x100u;

	LOG_INF("MB85RC256V FRAM write %u bytes to offset 0x%x", fram_nbytes, fram_mem_addr);

	prev_i2c_nl_count = atomic_get(&i2c_nl_cb_count);

	nmsgs = 1;
	buf1[0] = (uint8_t)((fram_mem_addr >> 8) & 0xffu); /* address b[15:8] */
	buf1[1] = (uint8_t)((fram_mem_addr) & 0xffu); /* address b[7:0] */

	for (temp = 0; temp < fram_nbytes; temp++) {
		buf1[temp + 2] = (uint8_t)((temp + 0x10) & 0xffu);
	}

	msgs[0].buf = buf1;
	msgs[0].len = fram_nbytes + 2u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	i2c_nl_cb_user_data[0] = fram_mem_addr;
	i2c_nl_cb_user_data[1] = fram_nbytes;
	i2c_nl_cb_user_data[2] = (uint32_t)buf1;

	ret = i2c_transfer_cb_dt(&mb85rc256v_dts, msgs, nmsgs,
				 i2c_nl_async_cb, i2c_nl_cb_user_data);
	if (ret) {
		LOG_ERR("I2C CB DT API for FRAM write returned error %d", ret);
		return ret;
	}

	while (atomic_get(&i2c_nl_cb_count) == prev_i2c_nl_count) {
		;
	}

	LOG_INF("I2C CB DT write transfer callback fired: status = %d", i2c_nl_cb_result);

	LOG_INF("MB85RC256V FRAM read %u bytes from offset 0x%x", fram_nbytes, fram_mem_addr);

	prev_i2c_nl_count = atomic_get(&i2c_nl_cb_count);

	nmsgs = 2;
	buf2[0] = (uint8_t)((fram_mem_addr >> 8) & 0xffu); /* address b[15:8] */
	buf2[1] = (uint8_t)((fram_mem_addr) & 0xffu); /* address b[7:0] */

	msgs[0].buf = buf2;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf3;
	msgs[1].len = fram_nbytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	i2c_nl_cb_user_data[0] = fram_mem_addr;
	i2c_nl_cb_user_data[1] = fram_nbytes;
	i2c_nl_cb_user_data[2] = (uint32_t)buf3;

	ret = i2c_transfer_cb_dt(&mb85rc256v_dts, msgs, nmsgs,
				 i2c_nl_async_cb, i2c_nl_cb_user_data);
	if (ret) {
		LOG_ERR("I2C CB API for FRAM read returned error %d", ret);
		return ret;
	}

	while (atomic_get(&i2c_nl_cb_count) == prev_i2c_nl_count) {
		;
	}

	LOG_INF("I2C CB DT read transfer callback fired: status = %d", i2c_nl_cb_result);

	ret = memcmp(&buf1[2], buf3, fram_nbytes);
	if (ret == 0) {
		LOG_INF("FRAM read back of %u bytes matches written data", fram_nbytes);
	} else {
		LOG_ERR("FRAM read back mismatch");
	}

#endif /* APP_BOARD_HAS_FRAM_ATTACHED */

	return 0;
}
#endif /* CONFIG_I2C_CALLBACK */
