/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2022 Microchip Technology, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <soc.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *dma_dev = DEVICE_DT_GET(DT_NODELABEL(dmac));
static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c_smb_0));

static const struct i2c_dt_spec pca9555_dts = I2C_DT_SPEC_GET(DT_NODELABEL(pca9555_evb));
/* i2c_write_dt(&motorctl, buf, sizeof(buf)); */
static const struct i2c_dt_spec ltc2489_dts = I2C_DT_SPEC_GET(DT_NODELABEL(ltc2489_evb));

static volatile uint32_t spin_val;
static volatile int ret_val;

static volatile void *dma_cb_arg;
static volatile uint32_t dma_cb_id;
static volatile uint32_t dma_error;

static uint8_t data_buf1[1024];
static uint8_t data_buf2[1024];

#ifdef CONFIG_I2C_CALLBACK
struct app_i2c_cb_data {
	volatile int result;
	volatile uint32_t cnt;
};

struct app_i2c_cb_data i2c_cb_data0;

/* called from I2C driver interrupt handler */
void i2c_cb_func(const struct device *dev, int result, void *user_data)
{
	if (user_data) {
		((struct app_i2c_cb_data *)user_data)->result = result;
		((struct app_i2c_cb_data *)user_data)->cnt++;
	}
}
#endif

void spin_on(uint32_t id, int rval);
void clean_bufs(void);
void fill_buf(uint8_t *b, size_t bsz, size_t fillsz, uint8_t fill_val, int flags);

static void dma_test_done(const struct device *dev, void *arg,
			  uint32_t id, int error_code)
{
	ARG_UNUSED(dev);

	dma_cb_arg = arg;
	dma_cb_id = id;
	dma_error = (uint32_t)error_code;

	if (error_code == 0) {
		LOG_INF("DMA transfer done");
	} else {
		LOG_ERR("DMA transfer error: 0x%08x", (uint32_t)error_code);
	}
}

void main(void)
{
	int ret = 0;
	uint32_t chan_id = 0U;
	uint32_t dma_wait_count = 0U;
	struct dma_config dma_cfg = { 0 };
	struct dma_block_config dma_block_cfg = { 0 };
	struct i2c_msg imsg[4] = { 0 };

	LOG_INF("MCHP DMA driver test! %s", CONFIG_BOARD);

	clean_bufs();

	if (!device_is_ready(dma_dev)) {
		LOG_ERR("ERROR: DMA device is not ready!");
		spin_on(1, ret);
	}

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("ERROR: I2C device is not ready!");
		spin_on(2, ret);
	}

	fill_buf(data_buf1, sizeof(data_buf1), 64U, 0, BIT(0));
	fill_buf(data_buf2, sizeof(data_buf2), 64U, 0x55, 0);

	dma_cfg.channel_direction = MEMORY_TO_MEMORY;
	dma_cfg.source_data_size = 1U;
	dma_cfg.dest_data_size = 1U;
	dma_cfg.dma_callback = dma_test_done;
	dma_cfg.complete_callback_en = 1U;
	dma_cfg.error_callback_en = 1U;
	dma_cfg.block_count = 1U;
	dma_cfg.head_block = &dma_block_cfg;

	dma_block_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	dma_block_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	dma_block_cfg.block_size = 64U;
	dma_block_cfg.source_address = (uint32_t)data_buf1;
	dma_block_cfg.dest_address = (uint32_t)data_buf2;

	dma_cb_arg = NULL;
	dma_cb_id = 0xA5U;
	dma_error = 0xDEADBEEFU;

	ret = dma_config(dma_dev, chan_id, &dma_cfg);
	if (ret) {
		LOG_ERR("DMA API config: error %d", ret);
		spin_on(10, ret);
	}

	ret = dma_start(dma_dev, chan_id);
	if (ret) {
		LOG_ERR("DMA API start: error %d", ret);
		spin_on(11, ret);
	}

	while (dma_cb_id == 0xA5U) {
		k_busy_wait(5);
		dma_wait_count++;
	}

	LOG_INF("DMA callback occurred");
	LOG_INF("DMA wait count = %u", dma_wait_count);
	LOG_INF("callback arg = 0x%08x", (uint32_t)dma_cb_arg);
	LOG_INF("callback id = %u", dma_cb_id);
	LOG_INF("callback error code = 0x%08x", dma_error);

	LOG_INF("I2C Network Layer driver tests. This driver makes use of DMA driver");

	/* I2C-DMA test */

	LOG_INF("One transfer using one API call and one i2c_msg to device which does not exist at 0x56");
	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = data_buf1;
	imsg[0].len = 3u;
	imsg[0].flags = (I2C_MSG_WRITE | I2C_MSG_STOP);

	ret = i2c_transfer(i2c_dev, imsg, 1u, 0x56u);
	if (ret == 0) {
		LOG_INF("I2C xfr OK");
	} else {
		LOG_ERR("I2C xfr error: %d", ret);
	}

	LOG_INF("One transfer using one API call and one i2c_msg");
	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = data_buf1;
	imsg[0].len = 3u;
	imsg[0].flags = (I2C_MSG_WRITE | I2C_MSG_STOP);

	/* ret = i2c_transfer(i2c_dev, imsg, 1u, 0x26u); */
	ret = i2c_transfer_dt(&pca9555_dts, imsg, 1u);
	if (ret == 0) {
		LOG_INF("I2C xfr OK");
	} else {
		LOG_ERR("I2C xfr error: %d", ret);
	}

	/* try one API call with two buffers */
	LOG_INF("One transfer using one API call with message broken up into two i2c_msg buffers");
	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf1[1];
	imsg[1].len = 2u;
	imsg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	/* ret = i2c_transfer(i2c_dev, imsg, 2u, 0x26u); */
	ret = i2c_transfer_dt(&pca9555_dts, imsg, 2u);
	if (ret == 0) {
		LOG_INF("I2C xfr 4.1 OK");
	} else {
		LOG_ERR("I2C xfr 4.1 error: %d", ret);
	}

	/* We need to write a first message > 64 bytes to test driver
	 * logic buffering message on initial START.
	 * The PCA9555 has two register and we will try writing more bytes.
	 * Hopefully PCA9555 will not NACK the extra bytes.
	 */
	LOG_INF("One transfer using one API call for message length > 64 bytes");

	/* fill data_buf1 with repeating pattern of 0 ... 255, */
	fill_buf(data_buf1, sizeof(data_buf1), sizeof(data_buf1), 0, BIT(0));

	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = data_buf1;
	imsg[0].len = 81u; // 64 + 17
	imsg[0].flags = (I2C_MSG_WRITE | I2C_MSG_STOP);

	ret = i2c_transfer_dt(&pca9555_dts, imsg, 1u);
	if (ret == 0) {
		LOG_INF("I2C xfr 5.1 OK");
	} else {
		LOG_ERR("I2C xfr 5.1 error: %d", ret);
	}

	/* try a read from the PCA9555. Sequence is
	 * START, write-addr, cmd-byte, Rpt-START, read-addr, data0, data1, STOP
	 */
	LOG_INF("One message broken into address write, repeated start, read, and stop");
	data_buf1[0] = 0x01u; /* input port 1 */
	data_buf2[0] = 0x55u;
	data_buf2[1] = 0x55u;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf2[0];
	imsg[1].len = 2u;
	imsg[1].flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP;

	/* ret = i2c_transfer(i2c_dev, imsg, 2u, 0x26u); */
	ret = i2c_transfer_dt(&pca9555_dts, imsg, 2u);
	if (ret == 0) {
		LOG_INF("I2C xfr 5.1 OK");
	} else {
		LOG_ERR("I2C xfr 5.1 error: %d", ret);
	}
#if 1 /* API should not start a transaction and return -EINVAL, which it does */
	/* split read into two messages
	 * This will only work with MCPH I2C-NL HW if read message length is
	 * != 1 except for the last read message with STOP flag.
	 */
	LOG_INF("Try I2C transaction with > 2 buffers. Driver should return an error");
	data_buf1[0] = 0x00u; /* input port 0 */
	data_buf2[0] = 0x55u;
	data_buf2[1] = 0x55u;
	data_buf2[2] = 0x55u;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf2[0];
	imsg[1].len = 2u; /* intermedate read message length must be > 1 */
	imsg[1].flags = I2C_MSG_READ | I2C_MSG_RESTART;
	imsg[2].buf = &data_buf2[2];
	imsg[2].len = 1u; /* last read message with STOP flag length >= 1 */
	imsg[2].flags = I2C_MSG_READ | I2C_MSG_STOP;

	/* ret = i2c_transfer(i2c_dev, imsg, 3u, 0x26u); */
	ret = i2c_transfer_dt(&pca9555_dts, imsg, 3u);
	if (ret == 0) {
		LOG_ERR("FAIL. Driver should have rejected > 2 buffers");
	} else {
		LOG_INF("PASS. Expected error for > 2 buffers: %d", ret);
	}
#endif
	/* Try transaction to non-existant I2C device */
	LOG_INF("Try transaction to non-existant device at address 0x32");
	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = data_buf1;
	imsg[0].len = 3u;
	imsg[0].flags = (I2C_MSG_WRITE | I2C_MSG_STOP);

	ret = i2c_transfer(i2c_dev, imsg, 1u, 0x32u);
	if (ret == 0) {
		LOG_INF("I2C xfr OK");
	} else {
		LOG_ERR("I2C xfr error: %d", ret);
	}

	LOG_INF("After transaction to non-existant device can we do a transfer to a real device");
	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = data_buf1;
	imsg[0].len = 3u;
	imsg[0].flags = (I2C_MSG_WRITE | I2C_MSG_STOP);

	/* ret = i2c_transfer(i2c_dev, imsg, 1u, 0x26u); */
	ret = i2c_transfer_dt(&pca9555_dts, imsg, 1u);
	if (ret == 0) {
		LOG_INF("I2C xfr OK");
	} else {
		LOG_ERR("I2C xfr error: %d", ret);
	}

	LOG_INF("I2C Write LTC2489 ADC to select channel 0");
	data_buf1[0] = 0xb0u; /* 1011_0000 select single ended channel 0 */

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&ltc2489_dts, imsg, 1u);
	if (ret == 0) {
		LOG_INF("PASS: I2C write to LTC2489 OK");
	} else {
		LOG_ERR("FAIL: I2C write to LTC2489 error: %d", ret);
	}

	/* Delay give LTC2489 time to do conversion after selecting channel.
	 * If LTC2489 is not ready it will NAK it's address.
	 */
	k_sleep(K_MSEC(2000));

	LOG_INF("I2C Read 24 bit ADC reading from LTC2489");
	data_buf2[0] = 0x55u;
	data_buf2[1] = 0x55u;
	data_buf2[2] = 0x55u;

	imsg[0].buf = &data_buf2[0];
	imsg[0].len = 3u;
	imsg[0].flags = I2C_MSG_READ | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&ltc2489_dts, imsg, 1u);
	if (ret == 0) {
		LOG_INF("I2C read from LTC2489 OK");
		/* LTC2489 data format is 24-bits
		 * b]23] = sign bit
		 * b[22:6] = reading
		 * b[5:0] = 0 always
		 */
		uint32_t adc_reading = data_buf2[0] + ((uint32_t)data_buf2[1] << 8)
			+ ((uint32_t)data_buf2[2] << 16);

		LOG_INF("LTC2489 ADC reading = 0x%08x", adc_reading);
	} else {
		LOG_ERR("I2C read from LTC2489 error: %d", ret);
	}

#ifdef CONFIG_I2C_CALLBACK
	LOG_INF("Try asycnhronous transaction. Requires CONFIG_I2C_CALLBACK=y");

	dma_wait_count = 0;

	i2c_cb_data0.result = 0x12345678;
	i2c_cb_data0.cnt = 0;

	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = data_buf1;
	imsg[0].len = 3u;
	imsg[0].flags = (I2C_MSG_WRITE | I2C_MSG_STOP);

	ret = i2c_transfer_cb_dt(&pca9555_dts, imsg, 1, i2c_cb_func, (void *)&i2c_cb_data0);
	if (ret == 0) {
		LOG_INF("I2C transfer cb OK");
		while (i2c_cb_data0.cnt == 0) {
			dma_wait_count++;
		}
		LOG_INF("I2C transfer cb done: wait count = %u", dma_wait_count);
		LOG_INF("Callback data result = %d  cnt = %u", i2c_cb_data0.result, i2c_cb_data0.cnt);
	} else {
		LOG_INF("I2C transfer cb error: %d", ret);
	}


	LOG_INF("Try asynchronous: One message broken into address write, repeated start, read, and stop");
	dma_wait_count = 0;
	i2c_cb_data0.result = 0x12345678;
	i2c_cb_data0.cnt = 0;

	data_buf1[0] = 0x01u; /* input port 1 */
	data_buf2[0] = 0x55u;
	data_buf2[1] = 0x55u;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf2[0];
	imsg[1].len = 2u;
	imsg[1].flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP;

	ret = i2c_transfer_cb_dt(&pca9555_dts, imsg, 2, i2c_cb_func, (void *)&i2c_cb_data0);
	if (ret == 0) {
		LOG_INF("I2C transfer cb OK");
		while (i2c_cb_data0.cnt == 0) {
			dma_wait_count++;
		}
		LOG_INF("I2C transfer cb done: wait count = %u", dma_wait_count);
		LOG_INF("Callback data result = %d  cnt = %u", i2c_cb_data0.result, i2c_cb_data0.cnt);
	} else {
		LOG_INF("I2C transfer cb error: %d", ret);
	}

#endif
	LOG_INF("Application Done");
	spin_on(256, 0);
}


void clean_bufs(void)
{
	memset(data_buf1, 0x55, sizeof(data_buf1));
	memset(data_buf2, 0x55, sizeof(data_buf2));
}

void fill_buf(uint8_t *b, size_t bsz, size_t fillsz, uint8_t fill_val, int flags)
{
	size_t flen, n;

	if (!b || !bsz) {
		return;
	}

	flen = fillsz;
	if (flen > bsz) {
		flen = bsz;
	}

	for (n = 0; n < flen; n++) {
		if (flags & BIT(0)) {
			b[n] = n % 255U;
		} else {
			b[n] = fill_val;
		}
	}
}

void spin_on(uint32_t id, int rval)
{
	spin_val = id;
	ret_val = rval;

	log_panic(); /* flush log buffers */

	while (spin_val) {
		;
	}
}
