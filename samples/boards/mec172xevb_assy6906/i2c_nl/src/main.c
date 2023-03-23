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
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define I2C0_BASE 0x40004000
#define I2C0_CM_CMD (I2C0_BASE + 0xcu)
#define I2C_CM_CMD_MRUN_POS 0
#define I2C_CM_CMD_MPROCEED_POS 1
#define I2C_CM_CMD_START0_POS 8
#define I2C_CM_CMD_STARTN_POS 9
#define I2C_CM_CMD_STOP_POS 10
#define I2C_CM_CMD_WRCNT_POS 16
#define I2C_CM_CMD_RDCNT_POS 24
#define I2C0_COMPL (I2C0_BASE + 0x20u)
#define I2C_COMPL_STS_RW1C_MSK 0xe1397f00u
#define I2C_COMPL_CM_DONE_POS 30
#define I2C_COMPL_IDLE_POS 29
#define I2C_COMPL_CM_TX_POS 25
#define I2C_COMPL_CM_NAK_POS 24
#define I2C0_FSM_I2C (I2C0_BASE + 0x58u)
#define I2C0_FSM_NL (I2C0_BASE + 0x5cu)

#define I2C0_CM_TX_DATA (I2C0_BASE + 0x50u)
#define I2C0_CM_RX_DATA (I2C0_BASE + 0x54u)

#define DMA_CHAN1_BASE 0x40002480
#define DMA_CHAN1_ACTV (DMA_CHAN1_BASE + 0u)
#define DMA_CHAN1_MSTART (DMA_CHAN1_BASE + 4u)
#define DMA_CHAN1_MEND (DMA_CHAN1_BASE + 8u)
#define DMA_CHAN1_DEV_ADDR (DMA_CHAN1_BASE + 0xcu)
#define DMA_CHAN1_CTRL (DMA_CHAN1_BASE + 0x10u)
#define DMA_CHAN1_STS (DMA_CHAN1_BASE + 0x14u)
#define DMA_CHAN_STS_BER_POS 0
#define DMA_CHAN_STS_HWOVRFL_REQ_POS 1
#define DMA_CHAN_STS_DONE_POS 2
#define DMA_CHAN_STS_HWTERM_POS 3

#define GPIO_0012_CTRL_ADDR (0x40081000u + 0x28) /* J19-3 */
#define GPIO_0013_CTRL_ADDR (0x40081000u + 0x2c) /* J19-1 */
#define GPIO_0130_CTRL_ADDR (0x40081000u + 0x160) /* J20-3 */
#define GPIO_0131_CTRL_ADDR (0x40081000u + 0x164) /* J20-1 */
#define GPIO_0156_CTRL_ADDR (0x40081000u + 0x1b8) /* J20-1 */

#define MB85RC256V_FRAM_I2C_ADDR 0x50u

static const struct device *dma_dev = DEVICE_DT_GET(DT_NODELABEL(dmac));
static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c_smb_0));
static const struct device *i2c_dev_1 = DEVICE_DT_GET(DT_NODELABEL(i2c_smb_1));

static const struct gpio_dt_spec test_i2c_sda = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), i2c_sda_gpios);
static const struct gpio_dt_spec test_i2c_scl = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), i2c_scl_gpios);

static const struct i2c_dt_spec pca9555_dts = I2C_DT_SPEC_GET(DT_NODELABEL(pca9555_evb));
/* i2c_write_dt(&motorctl, buf, sizeof(buf)); */
static const struct i2c_dt_spec ltc2489_dts = I2C_DT_SPEC_GET(DT_NODELABEL(ltc2489_evb));
static const struct i2c_dt_spec mb85rc256v_dts = I2C_DT_SPEC_GET(DT_NODELABEL(mb85rc256v));

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

static volatile uint32_t spinval;

static void spin(uint32_t spin_val)
{
	spinval = spin_val;
	while (spinval) {
		;
	}
}

static volatile int do_exp1;
static volatile int do_exp2;
static volatile int do_exp3;
static volatile int do_exp4;
static volatile int do_i2c_configure;

static volatile int i32_test1;
static volatile uint8_t u8_test1;

static int i2c_stop_experiment1(void);
static int i2c_experiment2(void);
static int i2c_experiment3(void);
static int i2c_experiment4(void);

#if 0
static int i2c_gen_start(const struct gpio_dt_spec *sda_pin, const struct gpio_dt_spec *scl_pin);
static int i2c_gen_stop(const struct gpio_dt_spec *sda_pin, const struct gpio_dt_spec *scl_pin);
#endif

void main(void)
{
	int ret = 0;
	int i2c_test_num = 0;
	uint32_t dev_config = 0u;
	uint32_t temp = 0u;
	uint32_t chan_id = 0U;
	uint32_t dma_wait_count = 0U;
	uint32_t adc_wait_count = 0U;
	uint32_t fram_mem_addr = 0U;
	uint32_t fram_data = 0U;
	size_t xfrlen = 0u;
	struct dma_config dma_cfg = { 0 };
	struct dma_block_config dma_block_cfg = { 0 };
	struct i2c_msg imsg[4] = { 0 };

	do_exp1 = 0;
	do_exp2 = 0;
	do_exp3 = 0;
	do_exp4 = 0;
	do_i2c_configure = 0;

	u8_test1 = 0xffu;
	i32_test1 = (int)u8_test1;
	LOG_INF("u8_test1 = %u  i32_test1 = %d", u8_test1, i32_test1);

	LOG_INF("MCHP DMA driver test! %s", CONFIG_BOARD);

	clean_bufs();

	if (!device_is_ready(test_i2c_sda.port) || !device_is_ready(test_i2c_scl.port)) {
		LOG_ERR("ERROR: Test I2C pin devices not ready");
		spin_on(2, 0);
	}

	ret = gpio_pin_configure_dt(&test_i2c_sda, GPIO_OUTPUT_HIGH);
	if (ret) {
		LOG_ERR("ERROR: Could not configure test SDA GPIO: %d", ret);
		spin_on(2, ret);
	}

	ret = gpio_pin_configure_dt(&test_i2c_scl, GPIO_OUTPUT_HIGH);
	if (ret) {
		LOG_ERR("ERROR: Could not configure test SCL GPIO: %d", ret);
		spin_on(2, ret);
	}

	if (!device_is_ready(dma_dev)) {
		LOG_ERR("ERROR: DMA device is not ready!");
		spin_on(1, ret);
	}

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("ERROR: I2C device 0 is not ready!");
		spin_on(2, ret);
	}

	if (!device_is_ready(i2c_dev_1)) {
		LOG_ERR("ERROR: I2C device 1 is not ready!");
		spin_on(3, ret);
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
	LOG_INF("Kernel initialization of I2C-NL driver configured I2C-NL for controller mode and standard speed (100KHz)");

	if (do_i2c_configure) {
		dev_config = I2C_MODE_CONTROLLER | I2C_SPEED_STANDARD;
		LOG_INF("App reconfiguring I2C_NL as 0x%0x", dev_config);
		ret = i2c_configure(i2c_dev, dev_config);
		if (ret) {
			LOG_ERR("I2C-NL configuration error %d", ret);
		}
	}

	/* I2C DEBUG */
#if 0
	sys_write32(0x10240u, GPIO_0012_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(0x10240u, GPIO_0013_CTRL_ADDR); /* GPIO output drive high */
#endif
	sys_write32(0x10240u, GPIO_0130_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(0x10240u, GPIO_0131_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(0x10240u, GPIO_0156_CTRL_ADDR); /* GPIO output drive high */

	/* I2C-DMA test */
	if (do_exp1) {
		i2c_stop_experiment1();
	}

	if (do_exp2) {
		i2c_experiment2();
	}

	if (do_exp3) {
		i2c_experiment3();
	}

	if (do_exp4) {
		i2c_experiment4();
	}

	i2c_test_num++;
	LOG_INF("Test %d: One transfer using one API call and one i2c_msg to "
		"device which does not exist at 0x56", i2c_test_num);
	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = data_buf1;
	imsg[0].len = 3u;
	imsg[0].flags = (I2C_MSG_WRITE | I2C_MSG_STOP);

	ret = i2c_transfer(i2c_dev, imsg, 1u, 0x56u);
	if (ret) {
		LOG_INF("PASS: Expected API to return error -5: %d", ret);
	} else {
		LOG_ERR("FAIL: API return success when error -5 was expected");
	}

	i2c_test_num++;
	LOG_INF("Test %d: One transfer using one API call and one i2c_msg", i2c_test_num);
	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = data_buf1;
	imsg[0].len = 3u;
	imsg[0].flags = (I2C_MSG_WRITE | I2C_MSG_STOP);

	/* ret = i2c_transfer(i2c_dev, imsg, 1u, 0x26u); */
	ret = i2c_transfer_dt(&pca9555_dts, imsg, 1u);
	if (ret) {
		LOG_ERR("FAIL: %d", ret);
	}

	/* try one API call with two buffers */
	i2c_test_num++;
	LOG_INF("Test %d: One transfer using one API call with message "
		"broken up into two i2c_msg buffers", i2c_test_num);
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
	if (ret) {
		LOG_ERR("FAIL: %d", ret);
	}

	/* We need to write a first message > 64 bytes to test driver
	 * logic buffering message on initial START.
	 * The PCA9555 has two register and we will try writing more bytes.
	 * Hopefully PCA9555 will not NACK the extra bytes.
	 */
	i2c_test_num++;
	LOG_INF("Test %d: One transfer using one API call for message length > 64 bytes",
		i2c_test_num);

	/* fill data_buf1 with repeating pattern of 0 ... 255, */
	fill_buf(data_buf1, sizeof(data_buf1), sizeof(data_buf1), 0, BIT(0));

	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = data_buf1;
	imsg[0].len = 81u; // 64 + 17
	imsg[0].flags = (I2C_MSG_WRITE | I2C_MSG_STOP);

	ret = i2c_transfer_dt(&pca9555_dts, imsg, 1u);
	if (ret) {
		LOG_ERR("FAIL: %d", ret);
	}

	i2c_test_num++;
	LOG_INF("Test %d: Break up write transfer into multiple calls", i2c_test_num);

	/* fill data_buf1 with repeating pattern of 0 ... 255, */
	fill_buf(data_buf1, sizeof(data_buf1), sizeof(data_buf1), 0, BIT(0));

	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = data_buf1;
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;

	ret = i2c_transfer_dt(&pca9555_dts, imsg, 1u);
	if (ret) {
		LOG_ERR("FAIL: %d", ret);
	}

	imsg[0].buf = &data_buf1[1];
	imsg[0].len = 2u;
	imsg[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&pca9555_dts, imsg, 1u);
	if (ret) {
		LOG_ERR("FAIL: %d", ret);
	}
#if 1
	/* try a read from the PCA9555. Sequence is
	 * START, write-addr, cmd-byte, Rpt-START, read-addr, data0, data1, STOP
	 */
	i2c_test_num++;
	LOG_INF("Test %d: 1-Write, 1-Read: read with restart and stop", i2c_test_num);
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
	if (ret) {
		LOG_ERR("FAIL: %d", ret);
	}
#endif
#if 1
	i2c_test_num++;
	LOG_INF("Test %d: One call. 3 buffers: Wr-1, Rd-2, Rd-1", i2c_test_num);
	data_buf1[0] = 0x00u; /* input port 0 */
	data_buf2[0] = 0x55u;
	data_buf2[1] = 0x55u;
	data_buf2[2] = 0x55u;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf2[0];
	imsg[1].len = 2u;
	imsg[1].flags = I2C_MSG_READ | I2C_MSG_RESTART;
	imsg[2].buf = &data_buf2[2];
	imsg[2].len = 1u;
	imsg[2].flags = I2C_MSG_READ | I2C_MSG_STOP;

	/* ret = i2c_transfer(i2c_dev, imsg, 3u, 0x26u); */
	ret = i2c_transfer_dt(&pca9555_dts, imsg, 3u);
	if (ret) {
		LOG_ERR("FAIL: %d", ret);
	}
#endif
#if 1
	i2c_test_num++;
	LOG_INF("Test %d: One call. 3 buffers: WR-1, Rd-1, Rd-2", i2c_test_num);
	data_buf1[0] = 0x00u; /* input port 0 */
	data_buf2[0] = 0x55u;
	data_buf2[1] = 0x55u;
	data_buf2[2] = 0x55u;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf2[0];
	imsg[1].len = 1u;
	imsg[1].flags = I2C_MSG_READ | I2C_MSG_RESTART;
	imsg[2].buf = &data_buf2[1];
	imsg[2].len = 2u;
	imsg[2].flags = I2C_MSG_READ | I2C_MSG_STOP; /* last read message with STOP flag length */

	/* ret = i2c_transfer(i2c_dev, imsg, 3u, 0x26u); */
	ret = i2c_transfer_dt(&pca9555_dts, imsg, 3u);
	if (ret) {
		LOG_ERR("FAIL: %d", ret);
	}
#endif
#if 1
	i2c_test_num++;
	LOG_INF("Test %d: One call. 3 buffers: WR-1, Rd-2, Rd-2", i2c_test_num);
	data_buf1[0] = 0x00u; /* input port 0 */
	data_buf2[0] = 0x55u;
	data_buf2[1] = 0x55u;
	data_buf2[2] = 0x55u;
	data_buf2[3] = 0x55u;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf2[0];
	imsg[1].len = 2u;
	imsg[1].flags = I2C_MSG_READ | I2C_MSG_RESTART;
	imsg[2].buf = &data_buf2[2];
	imsg[2].len = 2u;
	imsg[2].flags = I2C_MSG_READ | I2C_MSG_STOP; /* last read message with STOP flag length */

	/* ret = i2c_transfer(i2c_dev, imsg, 3u, 0x26u); */
	ret = i2c_transfer_dt(&pca9555_dts, imsg, 3u);
	if (ret) {
		LOG_ERR("FAIL: %d", ret);
	}
#endif
#if 1
	i2c_test_num++;
	LOG_INF("Test %d: One call. 3 buffers: WR-1, Rd-2, Rd-3", i2c_test_num);
	data_buf1[0] = 0x00u; /* input port 0 */
	data_buf2[0] = 0x55u;
	data_buf2[1] = 0x55u;
	data_buf2[2] = 0x55u;
	data_buf2[3] = 0x55u;
	data_buf2[4] = 0x55u;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf2[0];
	imsg[1].len = 2u;
	imsg[1].flags = I2C_MSG_READ | I2C_MSG_RESTART;
	imsg[2].buf = &data_buf2[2];
	imsg[2].len = 3u;
	imsg[2].flags = I2C_MSG_READ | I2C_MSG_STOP; /* last read message with STOP flag length */

	/* ret = i2c_transfer(i2c_dev, imsg, 3u, 0x26u); */
	ret = i2c_transfer_dt(&pca9555_dts, imsg, 3u);
	if (ret) {
		LOG_ERR("FAIL: %d", ret);
	}
#endif
#if 1
	i2c_test_num++;
	LOG_INF("Test %d: Five calls. 1 buffer per call: WR-1, Rd-8, Rd-8, Rd-8, Rd-8-STOP",
		i2c_test_num);
	data_buf1[0] = 0x00u; /* input port 0 */
	memset(data_buf2, 0, 32);

	xfrlen = 8u;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;

	ret = i2c_transfer_dt(&pca9555_dts, imsg, 1u);
	if (ret == 0) {
		for (int i = 0; i < 4; i++) {
			imsg[0].buf = &data_buf2[i * 8];
			imsg[0].len = xfrlen;
			imsg[0].flags = I2C_MSG_READ;
			if (i == 0) {
				imsg[0].flags |= I2C_MSG_RESTART;
			}
			if (i == 3) {
				imsg[0].flags |= I2C_MSG_STOP;
			}
			ret = i2c_transfer_dt(&pca9555_dts, imsg, 1u);
			if (ret) {
				LOG_ERR("FAIL Call %d of 5", i + 1);
				break;
			}
		}
	}
#endif
#if 1
	/* Try transaction to non-existant I2C device */
	i2c_test_num++;
	LOG_INF("Test %d: Try transaction to non-existant device at address 0x32", i2c_test_num);
	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = data_buf1;
	imsg[0].len = 3u;
	imsg[0].flags = (I2C_MSG_WRITE | I2C_MSG_STOP);

	ret = i2c_transfer(i2c_dev, imsg, 1u, 0x32u);
		if (ret) {
		LOG_INF("PASS: Expected API to return error -5: %d", ret);
	} else {
		LOG_ERR("FAIL: API return success when error -5 was expected");
	}
#endif
#if 1
	i2c_test_num++;
	LOG_INF("Test %d: After transaction to non-existant device "
		"can we do a transfer to a real device", i2c_test_num);
	data_buf1[0] = 0x02u;
	data_buf1[1] = 0xa5u;
	data_buf1[2] = 0x5au;

	imsg[0].buf = data_buf1;
	imsg[0].len = 3u;
	imsg[0].flags = (I2C_MSG_WRITE | I2C_MSG_STOP);

	/* ret = i2c_transfer(i2c_dev, imsg, 1u, 0x26u); */
	ret = i2c_transfer_dt(&pca9555_dts, imsg, 1u);
	if (ret) {
		LOG_ERR("FAIL: %d", ret);
	}
#endif
#if 1
	i2c_test_num++;
	LOG_INF("Test %d: I2C Write LTC2489 ADC to select channel 0", i2c_test_num);
	data_buf1[0] = 0xb0u; /* 1011_0000 select single ended channel 0 */

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&ltc2489_dts, imsg, 1u);
	if (ret) {
		LOG_ERR("FAIL: %d", ret);
	}
#endif
#if 1
	i2c_test_num++;
	LOG_INF("Test %d: I2C Read 24 bit ADC reading from LTC2489", i2c_test_num);
	/* LTC2489 will NAK its address while a conversion is in progress. */
	adc_wait_count = 0;
	do {
		k_sleep(K_MSEC(1));
		adc_wait_count++;

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

			LOG_INF("LTC2489 ADC wait count = %u reading = 0x%08x",
				adc_wait_count, adc_reading);
			break;
		}
	} while (adc_wait_count < 3000u);

	if (adc_wait_count >= 3000u) {
		LOG_ERR("LTC2480 time out");
	}
#endif /* 0 */
#if 1
	fram_mem_addr = 0x147au;
	fram_data = 0x04030201u;
	i2c_test_num++;
	LOG_INF("Test %d: I2C FRAM write 4 bytes 0x%08x to 16-bit offset %0x",
		i2c_test_num, fram_data, fram_mem_addr);
	data_buf1[0] = (fram_mem_addr >> 8) & 0xffu;
	data_buf1[1] = fram_mem_addr & 0xffu;
	data_buf1[2] = fram_data & 0xffu;
	data_buf1[3] = (fram_data >> 8) & 0xffu;
	data_buf1[4] = (fram_data >> 16) & 0xffu;
	data_buf1[5] = (fram_data >> 24) & 0xffu;

	imsg[0].buf = data_buf1;
	imsg[0].len = 2;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf1[2];
	imsg[1].len = 4;
	imsg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	/* ret = i2c_transfer(i2c_dev_1, imsg, 2u, MB85RC256V_FRAM_I2C_ADDR); */
	ret = i2c_transfer_dt(&mb85rc256v_dts, imsg, 2u);
	if (ret) {
		LOG_ERR("I2C FRAM multi-byte write error %d", ret);
	}

	i2c_test_num++;
	LOG_INF("Test %d: I2C FRAM Read 4 bytes from 16-bit offset %0x",
		i2c_test_num, fram_mem_addr);
	data_buf1[0] = (fram_mem_addr >> 8) & 0xffu;
	data_buf1[1] = fram_mem_addr & 0xffu;

	data_buf2[0] = 0x55u;
	data_buf2[1] = 0x55u;
	data_buf2[2] = 0x55u;
	data_buf2[3] = 0x55u;

	imsg[0].buf = data_buf1;
	imsg[0].len = 2u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = data_buf2;
	imsg[1].len = 4u;
	imsg[1].flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP;

	/* ret = i2c_transfer(i2c_dev_1, imsg, 2u, MB85RC256V_FRAM_I2C_ADDR); */
	ret = i2c_transfer_dt(&mb85rc256v_dts, imsg, 2u);
	if (ret) {
		LOG_ERR("I2C FRAM read error %d", ret);
	}

	temp = data_buf2[3];
	temp <<= 8;
	temp |= data_buf2[2];
	temp <<= 8;
	temp |= data_buf2[1];
	temp <<= 8;
	temp |= data_buf2[0];

	if (temp == fram_data) {
		LOG_INF("PASS: Data read from FRAM matches data written to FRAM");
	} else {
		LOG_ERR("FAIL: Read 0x%08x from FRAM expected 0x%08x", temp, fram_data);
	}
#endif
#if 1
	i2c_test_num++;
	LOG_INF("Test %d: STOP in the middle of read buffer list", i2c_test_num);
	data_buf1[0] = 0x00u; /* input port 0 */
	data_buf1[1] = 0x02u;
	data_buf1[2] = 0x11u;
	data_buf1[3] = 0x22u;
	data_buf2[0] = 0x55u;
	data_buf2[1] = 0x55u;
	data_buf2[2] = 0x55u;
	data_buf2[3] = 0x55u;
	data_buf2[4] = 0x55u;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;
	imsg[1].buf = &data_buf2[0];
	imsg[1].len = 2u;
	imsg[1].flags = I2C_MSG_READ | I2C_MSG_RESTART;
	imsg[2].buf = &data_buf2[2];
	imsg[2].len = 3u;
	imsg[2].flags = I2C_MSG_READ;
	imsg[3].buf = &data_buf1[1];
	imsg[3].len = 3;
	imsg[3].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	/* ret = i2c_transfer(i2c_dev, imsg, 3u, 0x26u); */
	ret = i2c_transfer_dt(&pca9555_dts, imsg, 4u);
	if (ret) {
		LOG_ERR("FAIL: %d", ret);
	}
#endif
#if 1
	i2c_test_num++;
	LOG_INF("Test %d: Begin open series of reads using multiple API calls, "
		"terminate with explicit STOP call", i2c_test_num);

	data_buf1[0] = 0x00u; /* input port 0 */
	memset(data_buf2, 0, 32);

	xfrlen = 8u;

	imsg[0].buf = &data_buf1[0];
	imsg[0].len = 1u;
	imsg[0].flags = I2C_MSG_WRITE;

	ret = i2c_transfer_dt(&pca9555_dts, imsg, 1u);
	if (ret == 0) {
		for (int i = 0; i < 4; i++) {
			/* k_busy_wait(200); */
			imsg[0].buf = &data_buf2[i * 8];
			imsg[0].len = xfrlen;
			imsg[0].flags = I2C_MSG_READ;
			if (i == 0) {
				imsg[0].flags |= I2C_MSG_RESTART;
			}
			ret = i2c_transfer_dt(&pca9555_dts, imsg, 1u);
			if (ret) {
				LOG_ERR("FAIL Call %d of 5", i + 1);
				break;
			}
		}
	}

	imsg[0].buf = NULL;
	imsg[0].len = 0u;
	imsg[0].flags = I2C_MSG_STOP;
	ret = i2c_transfer_dt(&pca9555_dts, imsg, 1u);
	if (ret) {
		LOG_ERR("zero length message with I2C_MSG_STOP failed: %d", ret);
	}
#endif
#if 1
	i2c_test_num++;
	LOG_INF("Test %d: Begin open series of writes using multiple API calls, "
		"terminate with explicit STOP call", i2c_test_num);

	fram_mem_addr = 0x0210u;
	i2c_test_num++;
	LOG_INF("Test %d: I2C FRAM write 4 bytes 0x%08x to 16-bit offset %0x",
		i2c_test_num, fram_data, fram_mem_addr);
	data_buf1[0] = (fram_mem_addr >> 8) & 0xffu;
	data_buf1[1] = fram_mem_addr & 0xffu;
	for (int i = 0; i < 64u; i++) {
		data_buf1[2+i] = (uint8_t)((i + 2) & 0xffu);
	}

	xfrlen = 8u;

	imsg[0].buf = data_buf1;
	imsg[0].len = 2;
	imsg[0].flags = I2C_MSG_WRITE;

	imsg[1].buf = &data_buf1[2];
	imsg[1].len = 4;
	imsg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	/* ret = i2c_transfer(i2c_dev_1, imsg, 1u, MB85RC256V_FRAM_I2C_ADDR); */
	ret = i2c_transfer_dt(&mb85rc256v_dts, imsg, 1u);
	if (ret == 0) {
		for (int i = 2; i < (2 + (4 * 8)); i += 8u) {
			imsg[0].buf = &data_buf1[i];
			imsg[0].len = xfrlen;
			imsg[0].flags = I2C_MSG_WRITE;
			/* ret = i2c_transfer(i2c_dev_1, imsg, 1u, MB85RC256V_FRAM_I2C_ADDR); */
			ret = i2c_transfer_dt(&mb85rc256v_dts, imsg, 1u);
			if (ret) {
				LOG_ERR("FAIL Call %d of 5", i + 1);
				break;
			}
		}
	} else {
		LOG_ERR("FRAM Open write 2 error %d", ret);
		spin(i2c_test_num * 10);
	}

	imsg[0].buf = NULL;
	imsg[0].len = 0u;
	imsg[0].flags = I2C_MSG_STOP;
	/* ret = i2c_transfer(i2c_dev_1, imsg, 1u, MB85RC256V_FRAM_I2C_ADDR); */
	ret = i2c_transfer_dt(&mb85rc256v_dts, imsg, 1u);
	if (ret) {
		LOG_ERR("zero length message with I2C_MSG_STOP failed: %d", ret);
		spin((i2c_test_num * 10) + 1);
	}
#endif
#ifdef CONFIG_I2C_CALLBACK
	i2c_test_num++;
	LOG_INF("Test %d: Try asycnhronous transaction. Requires CONFIG_I2C_CALLBACK=y",
		i2c_test_num);

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

#if 0
/* I2C START is SDA high to low while SCL is high then SCL low
 * t-hd,sta is time from falling edge of SDA to falling edge of SCL = 4us (100KHz)
 */
static int i2c_gen_start(const struct gpio_dt_spec *sda_pin, const struct gpio_dt_spec *scl_pin)
{
	gpio_pin_set_dt(sda_pin, 1);
	gpio_pin_set_dt(scl_pin, 1);
	k_busy_wait(4);
	gpio_pin_set_dt(sda_pin, 0);
	k_busy_wait(4);
	gpio_pin_set_dt(scl_pin, 0);

	return 0;
}

/* I2C STOP is SDA low to high SDA while SCL is high */
static int i2c_gen_stop(const struct gpio_dt_spec *sda_pin, const struct gpio_dt_spec *scl_pin)
{
	if (gpio_pin_get_dt(scl_pin) == 0) {
		gpio_pin_set_dt(scl_pin, 1);
		k_busy_wait(4);
	}

	if (gpio_pin_get_dt(sda_pin)) {
		gpio_pin_set_dt(sda_pin, 0);
		k_busy_wait(4);
	}

	gpio_pin_set_dt(sda_pin, 1);

	return 0;
}
#endif

/* #define I2C_NL_SIGNAL_RX_DMA_HW_OVRF_REQ */
#define I2C_NL_NO_WAIT_REPROGRAM

struct i2c_fsm {
	uint32_t fsm_i2c;
	uint32_t fsm_nl;
};

static uint32_t i2c_fsm_idx;
static struct i2c_fsm i2c_fsm_dbg[16];

static uint32_t dma_sts_capture;
static uint32_t dma_sts_capture2;
static uint32_t wait_count[4];
static uint32_t i2c_compl[4];

static int i2c_stop_experiment1(void)
{
	uint32_t cm_cmd, maddr, dma_sts, dma_sts_cnt;

/*	sys_write32(0x10240u, GPIO_0012_CTRL_ADDR);  GPIO output drive high */
/*	sys_write32(0x10240u, GPIO_0013_CTRL_ADDR);  GPIO output drive high */
	sys_write32(0x10240u, GPIO_0130_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(0x10240u, GPIO_0131_CTRL_ADDR); /* GPIO output drive high */

	dma_sts_capture = dma_sts_capture2 = 0;
	i2c_fsm_idx = 0;
	memset(i2c_fsm_dbg, 0, sizeof(i2c_fsm_dbg));
	memset(wait_count, 0, sizeof(wait_count));
	memset(i2c_compl, 0, sizeof(i2c_compl));

	i2c_fsm_dbg[i2c_fsm_idx].fsm_i2c = sys_read32(I2C0_FSM_I2C);
	i2c_fsm_dbg[i2c_fsm_idx].fsm_nl = sys_read32(I2C0_FSM_NL);
	i2c_fsm_idx++;

	/* 1. Transmit START + wrAddr */
	wait_count[0] = 0;
	data_buf1[0] = 0x4cu;
	data_buf1[1] = 0u;

	sys_write32(I2C_COMPL_STS_RW1C_MSK, I2C0_COMPL);

	/* TX DMA */
	sys_write32(0, DMA_CHAN1_CTRL);
	sys_write32(0xff, DMA_CHAN1_STS);
	sys_write32(0, DMA_CHAN1_MEND);
	maddr = (uint32_t)&data_buf1[0];
	sys_write32(maddr, DMA_CHAN1_MSTART);
	sys_write32(maddr + 2u, DMA_CHAN1_MEND);
	sys_write32(I2C0_CM_TX_DATA, DMA_CHAN1_DEV_ADDR);
	sys_write32(1, DMA_CHAN1_ACTV);
	sys_write32(0x110301u, DMA_CHAN1_CTRL);

	/* Trigger I2C-NL */
	sys_write32(0x00020103u, I2C0_CM_CMD);
	sys_write32(0x00240u, GPIO_0012_CTRL_ADDR); /* GPIO output drive low */

	i2c_fsm_dbg[i2c_fsm_idx].fsm_i2c = sys_read32(I2C0_FSM_I2C);
	i2c_fsm_dbg[i2c_fsm_idx].fsm_nl = sys_read32(I2C0_FSM_NL);
	i2c_fsm_idx++;

	i2c_compl[0] = sys_read32(I2C0_COMPL);
	while (!(i2c_compl[0] & BIT(I2C_COMPL_CM_DONE_POS))) {
		wait_count[0]++;
		i2c_compl[0] = sys_read32(I2C0_COMPL);
	}

	sys_write32(0x10240u, GPIO_0012_CTRL_ADDR); /* GPIO output drive high */

	i2c_fsm_dbg[i2c_fsm_idx].fsm_i2c = sys_read32(I2C0_FSM_I2C);
	i2c_fsm_dbg[i2c_fsm_idx].fsm_nl = sys_read32(I2C0_FSM_NL);
	i2c_fsm_idx++;

	/* 2. Transmit RPT-START + rdAddr */
	wait_count[1] = 0;
	data_buf1[0] = 0x4du;

	sys_write32(I2C_COMPL_STS_RW1C_MSK, I2C0_COMPL);

	sys_write32(0, DMA_CHAN1_CTRL);
	sys_write32(0xff, DMA_CHAN1_STS);
	sys_write32(0, DMA_CHAN1_MEND);
	maddr = (uint32_t)&data_buf1[0];
	sys_write32(maddr, DMA_CHAN1_MSTART);
	sys_write32(maddr + 1u, DMA_CHAN1_MEND);
	sys_write32(I2C0_CM_TX_DATA, DMA_CHAN1_DEV_ADDR);
	sys_write32(1, DMA_CHAN1_ACTV);
	sys_write32(0x110301u, DMA_CHAN1_CTRL);

	/* Trigger I2C-NL */
	sys_write32(0xff010203u, I2C0_CM_CMD);
	sys_write32(0x00240u, GPIO_0013_CTRL_ADDR); /* GPIO output drive low */

	i2c_fsm_dbg[i2c_fsm_idx].fsm_i2c = sys_read32(I2C0_FSM_I2C);
	i2c_fsm_dbg[i2c_fsm_idx].fsm_nl = sys_read32(I2C0_FSM_NL);
	i2c_fsm_idx++;

	i2c_compl[1] = sys_read32(I2C0_COMPL);
	while (!(i2c_compl[1] & BIT(I2C_COMPL_CM_DONE_POS))) {
		wait_count[1]++;
		i2c_compl[1] = sys_read32(I2C0_COMPL);
	}

	sys_write32(0x10240u, GPIO_0013_CTRL_ADDR); /* GPIO output drive high */

	i2c_fsm_dbg[i2c_fsm_idx].fsm_i2c = sys_read32(I2C0_FSM_I2C);
	i2c_fsm_dbg[i2c_fsm_idx].fsm_nl = sys_read32(I2C0_FSM_NL);
	i2c_fsm_idx++;

	/* 3. Read 4 bytes from target device */
	wait_count[2] = 0;
	data_buf1[0] = 0x55u;
	data_buf1[1] = 0x55u;
	data_buf1[2] = 0x55u;
	data_buf1[3] = 0x55u;

	sys_write32(I2C_COMPL_STS_RW1C_MSK, I2C0_COMPL);

	dma_sts = 0;
	dma_sts_cnt = 0;
	dma_sts_capture = 0;
	sys_write32(0, DMA_CHAN1_CTRL);
	sys_write32(0xff, DMA_CHAN1_STS);
	sys_write32(0, DMA_CHAN1_MEND);
	maddr = (uint32_t)&data_buf1[0];
	sys_write32(maddr, DMA_CHAN1_MSTART);
	sys_write32(maddr + 4u, DMA_CHAN1_MEND);
	sys_write32(I2C0_CM_RX_DATA, DMA_CHAN1_DEV_ADDR);
	sys_write32(1, DMA_CHAN1_ACTV);
	sys_write32(0x110201u, DMA_CHAN1_CTRL);

	i2c_fsm_dbg[i2c_fsm_idx].fsm_i2c = sys_read32(I2C0_FSM_I2C);
	i2c_fsm_dbg[i2c_fsm_idx].fsm_nl = sys_read32(I2C0_FSM_NL);
	i2c_fsm_idx++;

	/* Set MPROCEED to 1 after DMA direction has been configured for RX */
	cm_cmd = sys_read32(I2C0_CM_CMD);
	cm_cmd |= (BIT(I2C_CM_CMD_MPROCEED_POS) | BIT(I2C_CM_CMD_MRUN_POS));
	sys_write32(cm_cmd, I2C0_CM_CMD);
	sys_write32(0x00240u, GPIO_0130_CTRL_ADDR); /* GPIO output drive low */

	i2c_fsm_dbg[i2c_fsm_idx].fsm_i2c = sys_read32(I2C0_FSM_I2C);
	i2c_fsm_dbg[i2c_fsm_idx].fsm_nl = sys_read32(I2C0_FSM_NL);
	i2c_fsm_idx++;

#ifdef I2C_NL_SIGNAL_RX_DMA_HW_OVRF_REQ
	for (;;) {
		wait_count[2]++;
		i2c_compl[2] = sys_read32(I2C0_COMPL);
		dma_sts = sys_read32(DMA_CHAN1_STS);
		if (dma_sts & BIT(DMA_CHAN_STS_HWOVRFL_REQ_POS)) {
			sys_write32(0x00240u, GPIO_0131_CTRL_ADDR); /* GPIO output drive low */
		}
		if (dma_sts & BIT(DMA_CHAN_STS_DONE_POS)) {
			sys_write32(0x10240u, GPIO_0130_CTRL_ADDR); /* GPIO output drive high */
		}
		if (dma_sts & (BIT(DMA_CHAN_STS_DONE_POS) | BIT(DMA_CHAN_STS_HWOVRFL_REQ_POS))
			== (BIT(DMA_CHAN_STS_DONE_POS) | BIT(DMA_CHAN_STS_HWOVRFL_REQ_POS))) {
			dma_sts_capture = dma_sts;
			break;
		}
	}
#else
	i2c_compl[2] = sys_read32(I2C0_COMPL);
	dma_sts = sys_read32(DMA_CHAN1_STS);
	while (!(dma_sts & BIT(DMA_CHAN_STS_DONE_POS))) {
		wait_count[2]++;
		i2c_compl[2] = sys_read32(I2C0_COMPL);
		dma_sts = sys_read32(DMA_CHAN1_STS);
	}

	sys_write32(0x10240u, GPIO_0130_CTRL_ADDR); /* GPIO output drive high */
	dma_sts_capture = dma_sts;
#endif

	i2c_fsm_dbg[i2c_fsm_idx].fsm_i2c = sys_read32(I2C0_FSM_I2C);
	i2c_fsm_dbg[i2c_fsm_idx].fsm_nl = sys_read32(I2C0_FSM_NL);
	i2c_fsm_idx++;

#ifdef I2C_NL_NO_WAIT_REPROGRAM
	/* RX DMA done. Attempt to reconfigure for another RX DMA
	 * This may work if we don't reprogram I2C-NL
	 * This assume I2C-NL rdCnt set to max and STOP bit set.
	 * Set final rdCnt and STOP.
	 */
	sys_write32(0x0A000403u, I2C0_CM_CMD);
	sys_write32(0x00240u, GPIO_0131_CTRL_ADDR); /* GPIO output drive low */
	sys_write32(0, DMA_CHAN1_CTRL);
	sys_write32(0xff, DMA_CHAN1_STS);
	sys_write32(0, DMA_CHAN1_MEND);
	maddr = (uint32_t)&data_buf1[0];
	sys_write32(maddr + 4u, DMA_CHAN1_MSTART);
	sys_write32(maddr + 14u, DMA_CHAN1_MEND);
	sys_write32(I2C0_CM_RX_DATA, DMA_CHAN1_DEV_ADDR);
	sys_write32(1, DMA_CHAN1_ACTV);
	sys_write32(0x110201u, DMA_CHAN1_CTRL);

	wait_count[3] = 0;
	i2c_compl[3] = sys_read32(I2C0_COMPL);
	dma_sts = sys_read32(DMA_CHAN1_STS);
	while (!(dma_sts & BIT(DMA_CHAN_STS_DONE_POS))) {
		wait_count[3]++;
		i2c_compl[3] = sys_read32(I2C0_COMPL);
		dma_sts = sys_read32(DMA_CHAN1_STS);
	}
	dma_sts_capture2 = dma_sts;
#else
	/* RX DMA done but I2C-NL is performing read-ahead
	 * how do we know when I2C-NL is done?
	 * !!! At this point I2C Completion register is 0. !!!
	 * We have no way of knowing when I2C-NL has finished
	 * generating read-ahead clocks.
	 * RX DMA is done reading bytes from I2C.CM_RX_Data register
	 * The result is read-ahead of two bytes to fill I2C.CM_RX_Data
	 * and low level I2C.Data. The Controller ACK's both bytes.
	 *
	 */
	wait_count[3] = 0;
	i2c_compl[3] = sys_read32(I2C0_COMPL);
	while (!(i2c_compl[3] & BIT(I2C_COMPL_CM_DONE_POS))) {
		wait_count[3]++;
		i2c_compl[3] = sys_read32(I2C0_COMPL);
	}
#endif

	i2c_fsm_dbg[i2c_fsm_idx].fsm_i2c = sys_read32(I2C0_FSM_I2C);
	i2c_fsm_dbg[i2c_fsm_idx].fsm_nl = sys_read32(I2C0_FSM_NL);
	i2c_fsm_idx++;

	LOG_INF("dma_sts = 0x%08x", dma_sts);
	LOG_INF("i2c_compl[0:3] = 0x%08x 0x%08x 0x%08x 0x%08x", i2c_compl[0], i2c_compl[1],
		i2c_compl[2], i2c_compl[3]);
	LOG_INF("wait_count[0:3] = %u %u %u %u", wait_count[0], wait_count[1],
		wait_count[2], wait_count[3]);
	LOG_INF("i2c_fsm_idx = %u", i2c_fsm_idx);
	for (uint32_t n = 0; n < i2c_fsm_idx; n++) {
		LOG_INF("i2c_fsm: i2c=0x%08x  nl=0x%08x", i2c_fsm_dbg[n].fsm_i2c, i2c_fsm_dbg[n].fsm_nl);
	}

	/* FAILURE: You can cause HW to generate an I2C STOP by writing
	 * CM_CMD to 0x0403 but HW does not clear bits[1:0] = MPROCEED:MRUN
	 * resulting in CM_CMD = 0x03.
	 * If we must force a STOP then the safest approach is a controller reset
	 * after the STOP. This cost another 35 ms to let the controller resynchronize
	 * to the I2C line states after the reset.
	 */

	return 0;
}

static int i2c_experiment2(void)
{
	uint32_t cm_cmd, maddr;
/*	uint32_t dma_sts, dma_sts_cnt; */
	uint8_t fmt_addr = ((0x34u << 1u) | 0x01u);

	sys_write32(0x10240u, GPIO_0012_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(0x10240u, GPIO_0013_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(0x10240u, GPIO_0130_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(0x10240u, GPIO_0131_CTRL_ADDR); /* GPIO output drive high */

	dma_sts_capture = dma_sts_capture2 = 0;
	i2c_fsm_idx = 0;
	memset(i2c_fsm_dbg, 0, sizeof(i2c_fsm_dbg));
	memset(wait_count, 0, sizeof(wait_count));
	memset(i2c_compl, 0, sizeof(i2c_compl));

	i2c_fsm_dbg[i2c_fsm_idx].fsm_i2c = sys_read32(I2C0_FSM_I2C);
	i2c_fsm_dbg[i2c_fsm_idx].fsm_nl = sys_read32(I2C0_FSM_NL);
	i2c_fsm_idx++;

	/* 1. Transmit START + rdAddr */
	wait_count[0] = 0;
	data_buf1[0] = 0x55u;
	data_buf1[1] = 0x55u;
	data_buf1[2] = 0x55u;
	data_buf1[3] = 0x55u;

	sys_write32(I2C_COMPL_STS_RW1C_MSK, I2C0_COMPL);

	/* RX DMA of 4 bytes */
	sys_write32(0, DMA_CHAN1_CTRL);
	sys_write32(0xff, DMA_CHAN1_STS);
	sys_write32(0, DMA_CHAN1_MEND);
	maddr = (uint32_t)&data_buf1[0];
	sys_write32(maddr, DMA_CHAN1_MSTART);
	sys_write32(maddr + 4u, DMA_CHAN1_MEND);
	sys_write32(I2C0_CM_RX_DATA, DMA_CHAN1_DEV_ADDR);
	sys_write32(1, DMA_CHAN1_ACTV);
	sys_write32(0x110201u, DMA_CHAN1_CTRL);

	/* Trigger I2C-NL */
	sys_write32(0x04010503u, I2C0_CM_CMD);
	sys_write8(fmt_addr, I2C0_CM_TX_DATA);
	sys_write32(0x00240u, GPIO_0012_CTRL_ADDR); /* GPIO output drive low */

	i2c_fsm_dbg[i2c_fsm_idx].fsm_i2c = sys_read32(I2C0_FSM_I2C);
	i2c_fsm_dbg[i2c_fsm_idx].fsm_nl = sys_read32(I2C0_FSM_NL);
	i2c_fsm_idx++;

	i2c_compl[0] = sys_read32(I2C0_COMPL);
	while (!(i2c_compl[0] & BIT(I2C_COMPL_CM_DONE_POS))) {
		wait_count[0]++;
		i2c_compl[0] = sys_read32(I2C0_COMPL);
	}

	sys_write32(0x10240u, GPIO_0012_CTRL_ADDR); /* GPIO output drive high */

	sys_write32(0x00240u, GPIO_0013_CTRL_ADDR); /* GPIO output drive low */

	sys_write32(I2C_COMPL_STS_RW1C_MSK, I2C0_COMPL);
	cm_cmd = sys_read32(I2C0_CM_CMD) | 0x03u;
	sys_write32(cm_cmd, I2C0_CM_CMD);

	i2c_compl[1] = sys_read32(I2C0_COMPL);
	while (!(i2c_compl[1] & BIT(I2C_COMPL_CM_DONE_POS))) {
		wait_count[1]++;
		i2c_compl[1] = sys_read32(I2C0_COMPL);
	}

	sys_write32(0x10240u, GPIO_0013_CTRL_ADDR); /* GPIO output drive low */

	LOG_INF("data_buf1[0] = 0x%02x", data_buf1[0]);
	LOG_INF("data_buf1[1] = 0x%02x", data_buf1[1]);
	LOG_INF("data_buf1[2] = 0x%02x", data_buf1[2]);
	LOG_INF("data_buf1[3] = 0x%02x", data_buf1[3]);
	LOG_INF("i2c_experiment2 done");

	return 0;
}

static int i2c_experiment3(void)
{
	uint32_t cm_cmd, maddr;
/*	uint32_t dma_sts, dma_sts_cnt; */
	uint8_t fmt_addr = (0x26u << 1u);

	sys_write32(0x10240u, GPIO_0012_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(0x10240u, GPIO_0013_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(0x10240u, GPIO_0130_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(0x10240u, GPIO_0131_CTRL_ADDR); /* GPIO output drive high */

	dma_sts_capture = dma_sts_capture2 = 0;
	i2c_fsm_idx = 0;
	memset(i2c_fsm_dbg, 0, sizeof(i2c_fsm_dbg));
	memset(wait_count, 0, sizeof(wait_count));
	memset(i2c_compl, 0, sizeof(i2c_compl));

	i2c_fsm_dbg[i2c_fsm_idx].fsm_i2c = sys_read32(I2C0_FSM_I2C);
	i2c_fsm_dbg[i2c_fsm_idx].fsm_nl = sys_read32(I2C0_FSM_NL);
	i2c_fsm_idx++;

	/* 1. Transmit START + wrAddr + 0x00 */
	wait_count[0] = 0;
	data_buf1[0] = fmt_addr;
	data_buf1[1] = 0;
	data_buf1[2] = 0x55u;
	data_buf1[3] = 0x55u;

	data_buf2[0] = 0x55u;
	data_buf2[1] = 0x55u;
	data_buf2[2] = 0x55u;
	data_buf2[3] = 0x55u;

	sys_write32(I2C_COMPL_STS_RW1C_MSK, I2C0_COMPL);

	/* TX DMA of 2 bytes */
	sys_write32(0, DMA_CHAN1_CTRL);
	sys_write32(0xff, DMA_CHAN1_STS);
	sys_write32(0, DMA_CHAN1_MEND);
	maddr = (uint32_t)&data_buf1[0];
	sys_write32(maddr, DMA_CHAN1_MSTART);
	sys_write32(maddr + 2u, DMA_CHAN1_MEND);
	sys_write32(I2C0_CM_TX_DATA, DMA_CHAN1_DEV_ADDR);
	sys_write32(1, DMA_CHAN1_ACTV);
	sys_write32(0x110301u, DMA_CHAN1_CTRL);

	/* Trigger I2C-NL */
	sys_write32(0x00020103u, I2C0_CM_CMD);
	sys_write32(0x00240u, GPIO_0012_CTRL_ADDR); /* GPIO output drive low */

	i2c_compl[0] = sys_read32(I2C0_COMPL);
	while (!(i2c_compl[0] & BIT(I2C_COMPL_CM_DONE_POS))) {
		wait_count[0]++;
		i2c_compl[0] = sys_read32(I2C0_COMPL);
	}

	sys_write32(0x10240u, GPIO_0012_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(I2C_COMPL_STS_RW1C_MSK, I2C0_COMPL);

	/* RPT-START + rdAddr */
	wait_count[1] = 0;
	wait_count[2] = 0;
	data_buf1[0] = fmt_addr | BIT(0);

	/* TX DMA of 1 byte */
	sys_write32(0, DMA_CHAN1_CTRL);
	sys_write32(0xff, DMA_CHAN1_STS);
	sys_write32(0, DMA_CHAN1_MEND);
	maddr = (uint32_t)&data_buf1[0];
	sys_write32(maddr, DMA_CHAN1_MSTART);
	sys_write32(maddr + 1u, DMA_CHAN1_MEND);
	sys_write32(I2C0_CM_TX_DATA, DMA_CHAN1_DEV_ADDR);
	sys_write32(1, DMA_CHAN1_ACTV);
	sys_write32(0x110301u, DMA_CHAN1_CTRL);

	sys_write32(0x00240u, GPIO_0013_CTRL_ADDR); /* GPIO output drive low */

	/* Trigger I2C-NL */
	sys_write32(0x00010203u, I2C0_CM_CMD); /* RPT-START plus transmit one byte (rdAddr) */
	sys_write32(0x00240u, GPIO_0012_CTRL_ADDR); /* GPIO output drive low */

	i2c_compl[1] = sys_read32(I2C0_COMPL);
	while (!(i2c_compl[1] & BIT(I2C_COMPL_CM_DONE_POS))) {
		wait_count[1]++;
		i2c_compl[1] = sys_read32(I2C0_COMPL);
	}

	sys_write32(0x10240u, GPIO_0012_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(I2C_COMPL_STS_RW1C_MSK, I2C0_COMPL);

	/* RX DMA of 4 bytes */
	sys_write32(0, DMA_CHAN1_CTRL);
	sys_write32(0xff, DMA_CHAN1_STS);
	sys_write32(0, DMA_CHAN1_MEND);
	maddr = (uint32_t)&data_buf2[0];
	sys_write32(maddr, DMA_CHAN1_MSTART);
	sys_write32(maddr + 4u, DMA_CHAN1_MEND);
	sys_write32(I2C0_CM_RX_DATA, DMA_CHAN1_DEV_ADDR);
	sys_write32(1, DMA_CHAN1_ACTV);
	sys_write32(0x110201u, DMA_CHAN1_CTRL);

	/* MRUN=MPROCEED=1, STOP=1, rdCnt=4 */
	cm_cmd = sys_read32(I2C0_CM_CMD) | BIT(0) | BIT(1) | BIT(10) | (4u << 24);
	sys_write32(cm_cmd, I2C0_CM_CMD);

	i2c_compl[2] = sys_read32(I2C0_COMPL);
	while (!(i2c_compl[2] & BIT(I2C_COMPL_CM_DONE_POS))) {
		wait_count[2]++;
		i2c_compl[2] = sys_read32(I2C0_COMPL);
	}

	sys_write32(0x10240u, GPIO_0013_CTRL_ADDR); /* GPIO output drive low */

	LOG_INF("data_buf2[0] = 0x%02x", data_buf2[0]);
	LOG_INF("data_buf2[1] = 0x%02x", data_buf2[1]);
	LOG_INF("data_buf2[2] = 0x%02x", data_buf2[2]);
	LOG_INF("data_buf2[3] = 0x%02x", data_buf2[3]);
	LOG_INF("i2c_experiment2 done");

	return 0;
}


#if 1
static volatile uint32_t test_dma_done;

static void dma_done_cb(const struct device *dev, void *arg, uint32_t id, int error_code)
{
	ARG_UNUSED(dev);

	test_dma_done = 1;
}

static int test_dma_cfg(const struct device *dev, uint32_t src, uint32_t dest,
			uint32_t nbytes, enum dma_channel_direction dir, dma_callback_t cb)
{
	struct dma_config dma_cfg = { 0 };				/* 7 * 4 = 28 bytes */
	struct dma_block_config dma_block_cfg = { 0 };			/* 9 * 4 = 36 bytes */
	int ret = 0;

	dma_cfg.channel_direction = dir;

	if (dir == MEMORY_TO_PERIPHERAL) {
		dma_block_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_block_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		dma_block_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_block_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	dma_block_cfg.source_address = src;
	dma_block_cfg.dest_address = dest;

	dma_cfg.dma_slot = 0x01;
	dma_cfg.source_data_size = 1U;
	dma_cfg.dest_data_size = 1U;
	dma_cfg.dma_callback = cb;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.complete_callback_en = 1U;
	dma_cfg.error_callback_en = 1U;
	dma_cfg.block_count = 1U;
	dma_cfg.head_block = &dma_block_cfg;

	dma_block_cfg.block_size = nbytes;

	ret = dma_config(dev, 0, &dma_cfg);
	if (ret) {
		return ret;
	}

	return 0;
}

/*
 * I2C address = 0x26 (PCA9555)
 * TX: START + wrAddr + 0x00.  TX DMA 2 bytes: wrAddr + cmd
 * TX: RPT-START + rdAddr:  TX DMA 1 byte: rdAddr
 * RX: 4 bytes no STOP: RX DMA 4 bytes
 * RX: 4 bytes no STOP: RX DMA 4 bytes
 * RX: 4 bytes no STOP: RX DMA 4 bytes
 * RX: 4 bytes no STOP: RX DMA 4 bytes
 * wrCnt = 3, rdCnt = 32 + 2 = 34(0x22)
 * RX: 4 bytes no STOP
 * RX: 4 bytes with STOP
 * wrCnt = 0, rdCnt = 8 bytes
 */
static int i2c_experiment4(void)
{
	uint8_t fmt_addr = (0x26u << 1u);

	sys_write32(0x10240u, GPIO_0012_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(0x10240u, GPIO_0013_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(0x10240u, GPIO_0130_CTRL_ADDR); /* GPIO output drive high */
	sys_write32(0x10240u, GPIO_0131_CTRL_ADDR); /* GPIO output drive high */

	test_dma_done = 0;
	dma_sts_capture = dma_sts_capture2 = 0;
	i2c_fsm_idx = 0;
	memset(i2c_fsm_dbg, 0, sizeof(i2c_fsm_dbg));
	memset(wait_count, 0, sizeof(wait_count));
	memset(i2c_compl, 0, sizeof(i2c_compl));
	memset(data_buf2, 0x55u, 64u);

	i2c_fsm_dbg[i2c_fsm_idx].fsm_i2c = sys_read32(I2C0_FSM_I2C);
	i2c_fsm_dbg[i2c_fsm_idx].fsm_nl = sys_read32(I2C0_FSM_NL);
	i2c_fsm_idx++;

	/* 1. Transmit START + wrAddr + 0x00 */
	wait_count[0] = 0;
	data_buf1[0] = fmt_addr;
	data_buf1[1] = 0;
	data_buf1[2] = fmt_addr | BIT(0);
	data_buf1[3] = 0x55u;

	sys_write32(I2C_COMPL_STS_RW1C_MSK, I2C0_COMPL);

	/* TX DMA of 2 bytes */
	test_dma_cfg(dma_dev, (uint32_t)data_buf1, 0x40004050u, 2u,
		     MEMORY_TO_PERIPHERAL, dma_done_cb);
	dma_start(dma_dev, 0);

	/* Trigger I2C-NL */
	sys_write32(0x34030303u, I2C0_CM_CMD); /* rdCnt = 0x34, wrCnt=0x03, START0=1, STARTN=1, MPROCEED=1, MRUN=1 */

	while (!(test_dma_done)) {
		wait_count[0]++;
	}

	test_dma_done = 0;

	/* RPT-START + rdAddr */
	wait_count[1] = 0;

	/* TX DMA of 1 byte */
	test_dma_cfg(dma_dev, (uint32_t)&data_buf1[2], 0x40004050u, 1u,
		     MEMORY_TO_PERIPHERAL, dma_done_cb);
	dma_start(dma_dev, 0);

	i2c_compl[1] = sys_read32(I2C0_COMPL);
	while (!(i2c_compl[1] & BIT(I2C_COMPL_CM_DONE_POS))) {
		wait_count[1]++;
		i2c_compl[1] = sys_read32(I2C0_COMPL);
	}


	LOG_INF("i2c_experiment4 done");

	return 0;
}
#endif
