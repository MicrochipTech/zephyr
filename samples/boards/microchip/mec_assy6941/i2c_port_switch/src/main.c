/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_saf.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/dt-bindings/i2c/mchp-xec-i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define BLINK_PERIOD_MS_STEP 100U
#define BLINK_PERIOD_MS_MAX  1000U

#define PCA9555_CMD_PORT0_IN  0
#define PCA9555_CMD_PORT1_IN  1u
#define PCA9555_CMD_PORT0_OUT 2u
#define PCA9555_CMD_PORT1_OUT 3u
#define PCA9555_CMD_PORT0_POL 4u
#define PCA9555_CMD_PORT1_POL 5u
#define PCA9555_CMD_PORT0_CFG 6u
#define PCA9555_CMD_PORT1_CFG 7u

/* #define APP_TEST_LTC2489 */

#define NODE_PCA9555   DT_NODELABEL(pca9555_evb)
#define NODE_LTC2489   DT_NODELABEL(ltc2489_evb)
#define NODE_FRAM      DT_NODELABEL(mb85rc256v_fram)
#define NODE_FAST_PLUS DT_NODELABEL(i2c_dev_fast_plus)

#define NODE_I2C1_P5   DT_NODELABEL(i2c1_p5)

const struct i2c_dt_spec pca9555_spec = I2C_DT_SPEC_GET(NODE_PCA9555);
#ifdef APP_TEST_LTC2489
const struct i2c_dt_spec ltc2489_spec = I2C_DT_SPEC_GET(NODE_LTC2489);
#endif
const struct i2c_dt_spec mb_fram_spec = I2C_DT_SPEC_GET(NODE_FRAM);

const struct i2c_dt_spec fast_plus_spec = I2C_DT_SPEC_GET(NODE_FAST_PLUS);

#define I2C_CTRL0_REG_BASE DT_REG_ADDR(DT_NODELABEL(i2c_smb_0))

/* depends on board jumpers */
#define PCA9555_PORT0_IN_EXPECTED 0xfffcU

#define LTC2489_ADC_CONV_TIME_MS 150
#define LTC2489_ADC_READ_RETRIES 10
#define I2C_MAX_MSGS    8
#define I2C_TX_BUF_SIZE 256
#define I2C_RX_BUF_SIZE 256

static struct i2c_msg msgs[I2C_MAX_MSGS];
static uint8_t i2c_tx_buf[I2C_TX_BUF_SIZE] __aligned(4);
static uint8_t i2c_rx_buf[I2C_RX_BUF_SIZE] __aligned(4);

static int test_read(const struct i2c_dt_spec *dts, uint32_t nread);

static int pca9555_test1(const struct i2c_dt_spec *dts, uint32_t i2c_freq_hz, uint8_t port,
			 uint16_t *port_value);

static int fram_test1(const struct i2c_dt_spec *dts, uint32_t i2c_freq_hz);

#ifdef APP_TEST_LTC2489
static int ltc2489_test1(const struct i2c_dt_spec *dts);
#endif

static void pr_i2c_ctrl_speed_regs(uintptr_t i2c_ctrl_base_addr);

int main(void)
{
	int rc = 0;
	uint16_t gpio_port0_bitmap = 0;

	LOG_INF("Zephyr Microchip MEC_ASSY6941 board I2C port switch sample");

#ifdef CONFIG_BOARD_QUALIFIERS
	LOG_INF("Board: %s/%s", CONFIG_BOARD, CONFIG_BOARD_QUALIFIERS);
#else
	LOG_INF("Board: %s", CONFIG_BOARD);
#endif

	memset((void *)msgs, 0, sizeof(msgs));
	memset((void *)i2c_tx_buf, 0x55, sizeof(i2c_tx_buf));
	memset((void *)i2c_rx_buf, 0xAA, sizeof(i2c_rx_buf));

	if (!device_is_ready(pca9555_spec.bus)) {
		LOG_ERR("I2C PCA9555 bus is not ready!");
	}

#ifdef APP_TEST_LTC2489
	if (!device_is_ready(ltc2489_spec.bus)) {
		LOG_ERR("I2C LTC2489 bus is not ready!");
	}
#endif

	if (!device_is_ready(mb_fram_spec.bus)) {
		LOG_ERR("I2C FRAM bus is not ready!");
	}

	if (!device_is_ready(fast_plus_spec.bus)) {
		LOG_ERR("I2C fast plus device controller is not ready!");
	}

	/* Read PCA9555 at default frequency for the I2C port */
	LOG_INF("Read PCA9555 input port 0 at I2C port default frequency");
	gpio_port0_bitmap = 0;
	rc = pca9555_test1(&pca9555_spec, 0, PCA9555_CMD_PORT0_IN, &gpio_port0_bitmap);
	pr_i2c_ctrl_speed_regs(I2C_CTRL0_REG_BASE);
	if (rc != 0) {
		LOG_ERR("PCA9555 read error (%d)", rc);
	}

	LOG_INF("Read FRAM at I2C port default frequency");
	rc = fram_test1(&mb_fram_spec, 0);
	pr_i2c_ctrl_speed_regs(I2C_CTRL0_REG_BASE);
	if (rc != 0) {
		LOG_ERR("FRAM read error (%d)", rc);
	}

	LOG_INF("Read FRAM at I2C Fast frequency");
	rc = fram_test1(&mb_fram_spec, I2C_BITRATE_FAST);
	pr_i2c_ctrl_speed_regs(I2C_CTRL0_REG_BASE);
	if (rc != 0) {
		LOG_ERR("FRAM read error (%d)", rc);
	}

	LOG_INF("Read one byte from FRAM at current frequency FAST");
	rc = test_read(&mb_fram_spec, 1U);
	pr_i2c_ctrl_speed_regs(I2C_CTRL0_REG_BASE);
	if (rc != 0) {
		LOG_ERR("test read 1 byte from FRAM error (%d)", rc);
	}

	LOG_INF("Read PCA9555 input port 0 at current frequency FAST");
	gpio_port0_bitmap = 0;
	rc = pca9555_test1(&pca9555_spec, 0, PCA9555_CMD_PORT0_IN, &gpio_port0_bitmap);
	pr_i2c_ctrl_speed_regs(I2C_CTRL0_REG_BASE);
	if (rc != 0) {
		LOG_ERR("PCA9555 read error (%d)", rc);
	}

	LOG_INF("Read PCA9555 input port 0 at Slow frequency");
	gpio_port0_bitmap = 0;
	rc = pca9555_test1(&pca9555_spec, I2C_BITRATE_STANDARD, PCA9555_CMD_PORT0_IN,
			   &gpio_port0_bitmap);
	pr_i2c_ctrl_speed_regs(I2C_CTRL0_REG_BASE);
	if (rc != 0) {
		LOG_ERR("PCA9555 read error (%d)", rc);
	}

	LOG_INF("Read one byte from FRAM at current frequency");
	rc = test_read(&mb_fram_spec, 1U);
	pr_i2c_ctrl_speed_regs(I2C_CTRL0_REG_BASE);
	if (rc != 0) {
		LOG_ERR("test read 1 byte from FRAM error (%d)", rc);
	}

	LOG_INF("Read FRAM at I2C Fast frequency");
	rc = fram_test1(&mb_fram_spec, I2C_BITRATE_FAST);
	pr_i2c_ctrl_speed_regs(I2C_CTRL0_REG_BASE);
	if (rc != 0) {
		LOG_ERR("FRAM read error (%d)", rc);
	}

	LOG_INF("Read fast plus device on different port");
	i2c_rx_buf[0] = 0;
	i2c_rx_buf[1] = 0;
	rc = i2c_read_dt(&fast_plus_spec, i2c_rx_buf, 2U);
	pr_i2c_ctrl_speed_regs(I2C_CTRL0_REG_BASE);
	if (rc != 0) {
		LOG_ERR("Read fast plus device error (%d)", rc);
	}

	LOG_INF("App Done");

	return 0;
}

static int pca9555_test1(const struct i2c_dt_spec *dts, uint32_t i2c_freq_hz, uint8_t port,
			 uint16_t *port_value)
{
	int rc = -ENOTSUP;
	uint32_t i2c_dev_cfg = I2C_MODE_CONTROLLER;

	if (dts == NULL) {
		LOG_ERR("PCA9555 test1 bad i2c DT spec");
		return -EINVAL;
	}

	memset(i2c_tx_buf, 0x55, sizeof(i2c_tx_buf));
	memset(i2c_rx_buf, 0xAA, sizeof(i2c_rx_buf));

	switch (i2c_freq_hz) {
	case I2C_BITRATE_STANDARD:
		LOG_INF("Use I2C standard: 100KHz");
		i2c_dev_cfg |= I2C_SPEED_SET(I2C_SPEED_STANDARD);
		break;
	case I2C_BITRATE_FAST:
		LOG_INF("Use I2C fast: 400KHz");
		i2c_dev_cfg |= I2C_SPEED_SET(I2C_SPEED_FAST);
		break;
	case I2C_BITRATE_FAST_PLUS:
		LOG_INF("Use I2C fast plus: 1MHz");
		i2c_dev_cfg |= I2C_SPEED_SET(I2C_SPEED_FAST_PLUS);
		break;
	default:
		LOG_INF("Use driver default speed");
		i2c_dev_cfg = 0;
		break;
	}

	if (i2c_dev_cfg != 0) {
		rc = i2c_configure(dts->bus, i2c_dev_cfg);
		if (rc != 0) {
			LOG_ERR("I2C config error (%d)", rc);
			return rc;
		}
	}

	i2c_tx_buf[0] = port;

	rc = i2c_write_read_dt(dts, i2c_tx_buf, 1U, i2c_rx_buf, 2U);
	if (rc != 0) {
		LOG_ERR("PCA9555 test1 I2C write(1)-read(2) error (%d)", rc);
		return rc;
	}

	if (port_value != NULL) {
		*port_value = ((uint16_t)i2c_rx_buf[1] << 8) | i2c_rx_buf[0];
	}

	return rc;
}

static int fram_test1(const struct i2c_dt_spec *dts, uint32_t i2c_freq_hz)
{
	int rc = -ENOTSUP;
	uint32_t i2c_dev_cfg = I2C_MODE_CONTROLLER;

	if (dts == NULL) {
		LOG_ERR("FRAM test1 bad i2c DT spec");
		return -EINVAL;
	}

	memset(i2c_tx_buf, 0x55, sizeof(i2c_tx_buf));
	memset(i2c_rx_buf, 0xAA, sizeof(i2c_rx_buf));

	i2c_tx_buf[0] = 0x43U;
	i2c_tx_buf[1] = 0x21U;
	i2c_tx_buf[2] = 0x01U;
	i2c_tx_buf[3] = 0x02U;
	i2c_tx_buf[4] = 0x03U;
	i2c_tx_buf[5] = 0x04U;

	switch (i2c_freq_hz) {
	case I2C_BITRATE_STANDARD:
		LOG_INF("Use I2C standard: 100KHz");
		i2c_dev_cfg |= I2C_SPEED_SET(I2C_SPEED_STANDARD);
		break;
	case I2C_BITRATE_FAST:
		LOG_INF("Use I2C fast: 400KHz");
		i2c_dev_cfg |= I2C_SPEED_SET(I2C_SPEED_FAST);
		break;
	case I2C_BITRATE_FAST_PLUS:
		LOG_INF("Use I2C fast plus: 1MHz");
		i2c_dev_cfg |= I2C_SPEED_SET(I2C_SPEED_FAST_PLUS);
		break;
	default:
		LOG_INF("Use driver default speed");
		i2c_dev_cfg = 0;
		break;
	}

	if (i2c_dev_cfg != 0) {
		rc = i2c_configure(dts->bus, i2c_dev_cfg);
		if (rc != 0) {
			LOG_ERR("I2C config error (%d)", rc);
			return rc;
		}
	}

	rc = i2c_write_dt(dts, i2c_tx_buf, 6U);
	if (rc != 0) {
		LOG_ERR("FRAM test1 write 6 bytes error: (%d)", rc);
		return rc;
	}

	i2c_tx_buf[0] = 0x43U;
	i2c_tx_buf[1] = 0x21U;

	rc = i2c_write_read_dt(dts, i2c_tx_buf, 2U, i2c_rx_buf, 4U);
	if (rc != 0) {
		LOG_ERR("FRAM test1 write(2)-read(4) error (%d)", rc);
		return rc;
	}

	rc = memcmp(&i2c_tx_buf[2], i2c_rx_buf, 4U);
	if (rc != 0) {
		LOG_ERR("FRAM data mismatch");
		rc = -EPERM;
	}

	return rc;
}

static int test_read(const struct i2c_dt_spec *dts, uint32_t nread)
{
	int rc = 0;

	if ((dts == NULL) || (nread > I2C_RX_BUF_SIZE)) {
		LOG_ERR("test read bad i2c DT spec or nread > I2C_RX_BUF_SIZE");
		return -EINVAL;
	}

	memset(i2c_rx_buf, 0xAA, sizeof(i2c_rx_buf));

	rc = i2c_read_dt(dts, i2c_rx_buf, nread);

	return rc;
}

#define XEC_I2C_BAUD_CLK MHZ(16)

static uint32_t calc_xec_i2c_bus_clk_freq(uint32_t bus_clk_regval)
{
	uint32_t low_period = (bus_clk_regval & 0xffU) + 1U;
	uint32_t high_period = ((bus_clk_regval & 0xff00U) >> 8) + 1U;

	return (XEC_I2C_BAUD_CLK / (low_period + high_period));
}

static void pr_i2c_ctrl_speed_regs(uintptr_t i2c_ctrl_base_addr)
{
	uint32_t regval = 0;
	uint32_t bus_clk_freq = 0;

	if (i2c_ctrl_base_addr == 0) {
		return;
	}

	LOG_INF("I2C Controller @ 0x%08lx", i2c_ctrl_base_addr);
	regval = sys_read32(i2c_ctrl_base_addr + 0x2cU);
	bus_clk_freq = calc_xec_i2c_bus_clk_freq(regval);
	LOG_INF("  Bus Clock = 0x%08x: Freq = %u", regval, bus_clk_freq);
	regval = sys_read32(i2c_ctrl_base_addr + 0x40U);
	LOG_INF("  Data Timing = 0x%08x", regval);
	regval = sys_read32(i2c_ctrl_base_addr + 0x18U);
	LOG_INF("  RptStart Hold time = 0x%08x", regval);
	regval = sys_read32(i2c_ctrl_base_addr + 0x24U);
	LOG_INF("  Idle Scaling = 0x%08x", regval);
	regval = sys_read32(i2c_ctrl_base_addr + 0x44U);
	LOG_INF("  Timeout Scaling = 0x%08x", regval);
	regval = sys_read32(i2c_ctrl_base_addr + 0x3cU);
	LOG_INF("  MCHP Rsvd Timing = 0x%08x", regval);
}
