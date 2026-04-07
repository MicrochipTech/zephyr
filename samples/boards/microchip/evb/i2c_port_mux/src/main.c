/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * Example application code demonstrating XEC I2C internal port mux usage.
 *
 * Applications use ONLY standard Zephyr I2C APIs. Port switching is
 * completely transparent - the mux driver handles it automatically.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/i2c_mchp_xec.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define PCA9555_CMD_PORT0_IN  0
#define PCA9555_CMD_PORT1_IN  1u
#define PCA9555_CMD_PORT0_OUT 2u
#define PCA9555_CMD_PORT1_OUT 3u
#define PCA9555_CMD_PORT0_POL 4u
#define PCA9555_CMD_PORT1_POL 5u
#define PCA9555_CMD_PORT0_CFG 6u
#define PCA9555_CMD_PORT1_CFG 7u

/* Get device handles for each mux channel (port).
 * Each of these is a standard Zephyr I2C bus device.
 */
static const struct device *i2c_port0 = DEVICE_DT_GET(DT_NODELABEL(mux_port0));
static const struct device *i2c_port5 = DEVICE_DT_GET(DT_NODELABEL(mux_port5));

/* Or use I2C_DT_SPEC_GET for device-tree-specified addresses */
static const struct i2c_dt_spec gpio_mux = I2C_DT_SPEC_GET(DT_NODELABEL(gpmux_evb));
static const struct i2c_dt_spec adc_evb = I2C_DT_SPEC_GET(DT_NODELABEL(ltc2489_evb));
static const struct i2c_dt_spec fram = I2C_DT_SPEC_GET(DT_NODELABEL(mb85rc256v_fram));

#define FRAM_TEST_ADDR 0x4321U

#define TX_BUF_LEN 256U
#define RX_BUF_LEN 256U

uint8_t tx_buf[TX_BUF_LEN];
uint8_t rx_buf[RX_BUF_LEN];

int main(void)
{
	int ret = 0;
	uint8_t port = 0;

	LOG_INF("I2C Port switch sample");

	memset(tx_buf, 0x55, sizeof(tx_buf));
	memset(rx_buf, 0xAA, sizeof(rx_buf));

	/* Verify all ports are ready */
	if (!device_is_ready(i2c_port0) || !device_is_ready(i2c_port5)) {
		printk("I2C mux ports not ready\n");
		return -1;
	}

	/*
	 * Write to FRAM on port 5 at 400 KHz.
	 * The mux driver transparently:
	 *   1. Acquires mutex
	 *   2. Switches to port 5 (resets controller, programs PORT_SEL=5, timing=400K)
	 *   3. Calls i2c_transfer() on parent controller
	 *   4. Releases mutex
	 */
	tx_buf[0] = (uint8_t)((FRAM_TEST_ADDR) >> 8); /* 16-bit address bits[15:8] */
	tx_buf[1] = (uint8_t)(FRAM_TEST_ADDR); /* 16-bit address bits[7:0] */
	tx_buf[2] = 0x11; /* data byte 0 */
	tx_buf[3] = 0x12; /* data byte 1 */
	tx_buf[4] = 0x13; /* data byte 2 */
	tx_buf[5] = 0x14; /* data byte 3 */

	ret = i2c_write_dt(&fram, tx_buf, sizeof(tx_buf));
	if (ret != 0) {
		LOG_ERR("FRAM write failed: %d", ret);
	}

	LOG_ERR("Get I2C port from controller FRAM is connected to");
	port = 0xffU;
	ret = i2c_xec_v2_get_port(fram.bus, &port);
	if (ret == 0) {
		LOG_ERR("Get I2C port = %u", port);
	} else {
		LOG_ERR("Get I2C port error (%d)", ret);
	}

	tx_buf[0] = (uint8_t)((FRAM_TEST_ADDR) >> 8); /* 16-bit address bits[15:8] */
	tx_buf[1] = (uint8_t)(FRAM_TEST_ADDR); /* 16-bit address bits[7:0] */

	/* Read data back and verify */
	ret = i2c_write_read_dt(&fram, tx_buf, 2, rx_buf, 4);
	if (ret != 0) {
		LOG_ERR("FRAM write-read failed: %d", ret);
	}

	ret = memcmp(&tx_buf[2], rx_buf, 4);
	if (ret != 0) {
		LOG_ERR("FRAM read data mismatch!");
	}

	log_flush();

	memset(tx_buf, 0x55, sizeof(tx_buf));
	memset(rx_buf, 0xAA, sizeof(rx_buf));

	tx_buf[0] = PCA9555_CMD_PORT0_IN; /* command */

	/*
	 * Read PCA9555 GPIO Mux on I2C Port 0 at 100KHz
	 * Port switch happens automatically (controller reset + reconfigure).
	 */
	ret = i2c_write_read_dt(&gpio_mux, tx_buf, 1, rx_buf, 2);
	if (ret != 0) {
		LOG_ERR("PCA9555 read failed: %d", ret);
	} else {
		LOG_INF("PCA9555 P0=0x%02x P1=0x%02x", rx_buf[0], rx_buf[1]);
	}

	log_flush();

#if 0
	/*
	 * Read from device on port 10 (inherits 400 KHz from parent).
	 * Another port switch happens.
	 */
	uint8_t reg_addr = 0x0F;  /* WHO_AM_I register */
	uint8_t who_am_i;

	ret = i2c_write_read(i2c_port10, 0x19, &reg_addr, 1, &who_am_i, 1);
	if (ret != 0) {
		printk("Accel read failed: %d\n", ret);
	}

	/*
	 * Back to port 0 - the mux driver caches the last selected port.
	 * If we already had port 0/400KHz active, this would skip the reset.
	 * Since we just used port 10, a reset+reconfigure occurs.
	 */
	ret = i2c_write_dt(&eeprom, tx_buf, sizeof(tx_buf));

	/*
	 * Another write to the same EEPROM - same port 0/400KHz.
	 * The mux driver detects port+speed match and SKIPS the reset.
	 * This is the fast path with zero switching overhead.
	 */
	tx_buf[0] = 0x01;
	tx_buf[1] = 0xCD;
	ret = i2c_write_dt(&eeprom, tx_buf, sizeof(tx_buf));
#endif

	LOG_INF("App done");

	return 0;
}
