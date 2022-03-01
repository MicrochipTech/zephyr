/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2022 Microchip Technology, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/dma.h>
#include <drivers/i2c.h>
#include <sys/printk.h>
#include <soc.h>
#include <string.h>
#include "mec_crypt.h"
#include "crc8.h"

#define DMA_DEV DT_LABEL(DT_NODELABEL(dmac0))
#define I2C_DEV DT_LABEL(DT_ALIAS(i2c0))

static const struct device *dma_dev;
static const struct device *i2c_dev;
static volatile uint32_t spin_val;
static volatile int ret_val;

static volatile void *dma_cb_arg;
static volatile uint32_t dma_cb_id;
static volatile uint32_t dma_error;

static uint8_t data_buf1[1024];
static uint8_t data_buf2[1024];

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
		printk("DMA transfer done\n");
	} else {
		printk("DMA transfer error: 0x%08x\n", (uint32_t)error_code);
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

	printk("MCHP DMA driver test! %s\n", CONFIG_BOARD);

	ret = mec_crypto_init(MEC_CRYPTO_INIT_AESH | MEC_CRYPTO_INIT_AESH_SRST,
			      NULL);
	if (ret) {
		printk("ERROR: MEC crypto init error: %d\n", ret);
		spin_on(1, 0);
	}

	clean_bufs();

	dma_dev = device_get_binding(DMA_DEV);
	if (!dma_dev) {
		printk("ERROR: Get dmac0 device binding failed!");
		spin_on(2, 0);
	}

	i2c_dev = device_get_binding(I2C_DEV);
	if (!i2c_dev) {
		printk("ERROR: Get i2c0 device binding failed!");
		spin_on(2, 0);
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
		printk("DMA API config: error %d\n", ret);
		spin_on(10, ret);
	}

	ret = dma_start(dma_dev, chan_id);
	if (ret) {
		printk("DMA API start: error %d\n", ret);
		spin_on(11, ret);
	}

	while (dma_cb_id == 0xA5U) {
		k_busy_wait(5);
		dma_wait_count++;
	}

	printk("DMA callback occurred\n");
	printk("DMA wait count = %u\n", dma_wait_count);
	printk("callback arg = %p\n", dma_cb_arg);
	printk("callback id = %u\n", dma_cb_id);
	printk("callback error code = 0x%08x\n", dma_error);

	/* I2C-DMA test */
	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = data_buf1;
	imsg[0].len = 3u;
	imsg[0].flags = (I2C_MSG_WRITE | I2C_MSG_STOP);

	ret = i2c_transfer(i2c_dev, imsg, 1u, 0x26u);
	if (ret == 0) {
		printk("I2C xfr OK\n");
	} else {
		printk("I2C xfr error: %d\n", ret);
	}

	/* try breaking up same transaction into multiple write buffers.
	 * Probably will not work due to I2C NL HW state machine.
	 */
	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf1[1];
	imsg[1].len = 2u;
	imsg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer(i2c_dev, imsg, 1u, 0x26u);
	if (ret == 0) {
		printk("I2C xfr 2.1 OK\n");
	} else {
		printk("I2C xfr 2.1 error: %d\n", ret);
	}

	ret = i2c_transfer(i2c_dev, &imsg[1], 1u, 0x26u);
	if (ret == 0) {
		printk("I2C xfr 2.2 OK\n");
	} else {
		printk("I2C xfr 2.2 error: %d\n", ret);
	}

	/* try again, make sequence works multiple times */
	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf1[1];
	imsg[1].len = 2u;
	imsg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer(i2c_dev, imsg, 1u, 0x26u);
	if (ret == 0) {
		printk("I2C xfr 3.1 OK\n");
	} else {
		printk("I2C xfr 3.1 error: %d\n", ret);
	}

	ret = i2c_transfer(i2c_dev, &imsg[1], 1u, 0x26u);
	if (ret == 0) {
		printk("I2C xfr 3.2 OK\n");
	} else {
		printk("I2C xfr 3.2 error: %d\n", ret);
	}

	/* try one API call with two buffers */
	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf1[1];
	imsg[1].len = 2u;
	imsg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer(i2c_dev, imsg, 2u, 0x26u);
	if (ret == 0) {
		printk("I2C xfr 4.1 OK\n");
	} else {
		printk("I2C xfr 4.1 error: %d\n", ret);
	}

	/* try a read from the PCA9555. Sequence is
	 * START, write-addr, cmd-byte, Rpt-START, read-addr, data0, data1, STOP
	 */
	data_buf1[0] = 0x01u; /* input port 1 */
	data_buf2[0] = 0x55u;
	data_buf2[1] = 0x55u;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf2[0];
	imsg[1].len = 2u;
	imsg[1].flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP;

	ret = i2c_transfer(i2c_dev, imsg, 2u, 0x26u);
	if (ret == 0) {
		printk("I2C xfr 5.1 OK\n");
	} else {
		printk("I2C xfr 5.1 error: %d\n", ret);
	}

	/* split read into two messages */
	data_buf1[0] = 0x00u; /* input port 0 */
	data_buf2[0] = 0x55u;
	data_buf2[1] = 0x55u;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf2[0];
	imsg[1].len = 1u;
	imsg[1].flags = I2C_MSG_READ | I2C_MSG_RESTART;
	imsg[2].buf = &data_buf2[1];
	imsg[2].len = 1u;
	imsg[2].flags = I2C_MSG_READ | I2C_MSG_STOP;

	ret = i2c_transfer(i2c_dev, imsg, 3u, 0x26u);
	if (ret == 0) {
		printk("I2C xfr 6.1 OK\n");
	} else {
		printk("I2C xfr 6.1 error: %d\n", ret);
	}

	spin_on(256, 0);
}


void clean_bufs(void)
{
	memset(data_buf1, 0x55, sizeof(data_buf1));
	memset(data_buf2, 0x55, sizeof(data_buf2));
}

void fill_buf(uint8_t *b, size_t bsz, size_t fillsz, uint8_t fill_val,
	      int flags)
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

	while (spin_val) {
		;
	}
}
