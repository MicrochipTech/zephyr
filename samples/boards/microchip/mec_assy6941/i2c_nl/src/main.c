/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <soc.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_xec_i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#ifdef CONFIG_I2C_TARGET
#include "target_tests.h"
#endif

#define PCA9555_CMD_PORT0_IN  0
#define PCA9555_CMD_PORT1_IN  1u
#define PCA9555_CMD_PORT0_OUT 2u
#define PCA9555_CMD_PORT1_OUT 3u
#define PCA9555_CMD_PORT0_POL 4u
#define PCA9555_CMD_PORT1_POL 5u
#define PCA9555_CMD_PORT0_CFG 6u
#define PCA9555_CMD_PORT1_CFG 7u

#define PCA9555_I2C_NODE DT_NODELABEL(pca9555_evb)
#define PCA9555_I2C_PORT DT_PROP(DT_PARENT(PCA9555_I2C_NODE), port_sel)
#define PCA9555_I2C_ADDR DT_REG_ADDR(PCA9555_I2C_NODE)

#define LTC2489_I2C_NODE DT_NODELABEL(ltc2489_evb))
#define LTC2489_I2C_PORT DT_PROP(DT_PARENT(LTC2489_I2C_NODE), port_sel)
#define LTC2489_I2C_ADDR DT_REG_ADDR(DT_NODELABEL(ltc2489_evb))

#define FRAM_I2C_NODE    DT_NODELABEL(mb85rc256v_fram)
#define FRAM_I2C_PORT    DT_PROP(DT_PARENT(FRAM_I2C_NODE), port_sel)
#define FRAM_I2C_ADDR    DT_REG_ADDR(FRAM_I2C_NODE)

#define I2C0_CTRL_REG_ADDR DT_REG_ADDR(DT_ALIAS(i2c0))
#define I2C0_CTRL_PORT     DT_PROP(DT_ALIAS(i2c0), port_sel)
#define I2C1_CTRL_REG_ADDR DT_REG_ADDR(DT_ALIAS(i2c1))
#define I2C1_CTRL_PORT     DT_PROP(DT_ALIAS(i2c1), port_sel)
#define I2C2_CTRL_REG_ADDR DT_REG_ADDR(DT_ALIAS(i2c2))
#define I2C2_CTRL_PORT     DT_PROP(DT_ALIAS(i2c2), port_sel)

static const struct device *i2c0_dev = DEVICE_DT_GET(DT_ALIAS(i2c0));
static const struct device *i2c1_dev = DEVICE_DT_GET(DT_ALIAS(i2c1));
static const struct device *i2c2_dev = DEVICE_DT_GET(DT_ALIAS(i2c2));

#if 0
#if DT_HAS_COMPAT_STATUS_OKAY(microchip_xec_i2c_nl)
#define APP_HAS_I2C_NL
static const struct device *i2c_nl_dev = DEVICE_DT_GET(DT_ALIAS(i2c_nl));
#endif
#endif

volatile uint32_t spin_val;

uint8_t i2c_wr_buf[64];
uint8_t i2c_rd_buf[64];

#define TARGET_I2C_ADDR2_RX_BUF_SIZE 10
#define TARGET_I2C_ADDR2_TX_BUF_SIZE 10

uint8_t i2c0_tx_buf[256];
uint8_t i2c0_rx_buf[256];

struct i2c_msg msgs[4];

int app_exit_code;

#ifdef APP_HAS_I2C_NL
static int i2c_nl_test1(const struct device *i2c_dev);
#endif

static void pr_i2c_regs(mm_reg_t i2c_base);

int main(void)
{
	uint32_t i2c0_config = 0, i2c1_config = 0, i2c2_config = 0;
	uint16_t i2c_addr = 0;
	size_t nread = 0, nwrite = 0;
#if 0 /* def APP_HAS_I2C_NL */
	bool i2c_nl_ready = false;
#endif
	int rc = 0;
	uint8_t nmsgs = 0;

	app_exit_code = 0;

	memset(msgs, 0, sizeof(msgs));

	if (!device_is_ready(i2c0_dev)) {
		LOG_ERR("I2C 0 device [%s] not ready!", i2c0_dev->name);
		app_exit_code = 1;
		goto app_exit;
	}

	pr_i2c_regs(I2C0_CTRL_REG_ADDR);

	if (!device_is_ready(i2c1_dev)) {
		LOG_ERR("I2C 1 device [%s] not ready!", i2c1_dev->name);
		app_exit_code = 2;
		goto app_exit;
	}

	pr_i2c_regs(I2C1_CTRL_REG_ADDR);

	if (!device_is_ready(i2c2_dev)) {
		LOG_ERR("I2C 2 device [%s] not ready!", i2c2_dev->name);
		app_exit_code = 3;
		goto app_exit;
	}

	pr_i2c_regs(I2C2_CTRL_REG_ADDR);

	LOG_INF("Configure I2C 0 as controller at 100KHz");

	log_flush();

	i2c0_config = I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD);
#ifdef CONFIG_I2C_XEC_PORT_MUX
	i2c0_config |= I2C_XEC_PORT_SET(I2C0_CTRL_PORT);
#endif

	rc = i2c_configure(i2c0_dev, i2c0_config);
	if (rc != 0) {
		LOG_ERR("I2C 0 device config failed");
		app_exit_code = 4;
		goto app_exit;
	}

	pr_i2c_regs(I2C0_CTRL_REG_ADDR);

	LOG_INF("Configure I2C 1 as controller at 100KHz");

	i2c1_config = I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD);
#ifdef CONFIG_I2C_XEC_PORT_MUX
	i2c1_config |= I2C_XEC_PORT_SET(I2C1_CTRL_PORT);
#endif

	rc = i2c_configure(i2c1_dev, i2c1_config);
	if (rc != 0) {
		LOG_ERR("I2C 1 device config failed");
		app_exit_code = 5;
		goto app_exit;
	}

	pr_i2c_regs(I2C1_CTRL_REG_ADDR);

	LOG_INF("Configure I2C 2 as controller at 100KHz");

	i2c2_config = I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD);
#ifdef CONFIG_I2C_XEC_PORT_MUX
	i2c2_config |= I2C_XEC_PORT_SET(I2C2_CTRL_PORT);
#endif

	rc = i2c_configure(i2c2_dev, i2c2_config);
	if (rc != 0) {
		LOG_ERR("I2C 2 device config failed");
		app_exit_code = 6;
		goto app_exit;
	}

	pr_i2c_regs(I2C2_CTRL_REG_ADDR);

	LOG_INF("FRAM on I2C 2: Write data");

	i2c_addr = (uint16_t)FRAM_I2C_ADDR;
	i2c_wr_buf[0] = 0x01; /* msb of 16-bit FRAM offset */
	i2c_wr_buf[1] = 0x20; /* lsb of 16-bit FRAM offset */
	i2c_wr_buf[2] = 0x11;
	i2c_wr_buf[3] = 0x12;
	i2c_wr_buf[4] = 0x13;
	i2c_wr_buf[5] = 0x14;

	msgs[0].buf = i2c_wr_buf;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = &i2c_wr_buf[2];
	msgs[1].len = 4u;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	nmsgs = 2u;

	rc = i2c_transfer(i2c2_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("FRAM write two message rc = %d", rc);

	LOG_INF("FRAM on I2C 2: Write-Read data");

	memset((void *)i2c_rd_buf, 0xAAU, 8U);

	i2c_addr = (uint16_t)FRAM_I2C_ADDR;
	i2c_wr_buf[0] = 0x01; /* msb of 16-bit FRAM offset */
	i2c_wr_buf[1] = 0x20; /* lsb of 16-bit FRAM offset */

	msgs[0].buf = i2c_wr_buf;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = i2c_rd_buf;
	msgs[1].len = 4u;
	msgs[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	nmsgs = 2u;

	rc = i2c_transfer(i2c2_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("FRAM Write-Read two message rc = %d", rc);
	LOG_HEXDUMP_INF(i2c_rd_buf, 4U, "i2c_rd_buf");

	LOG_INF("PCA9555 on I2C 0: Read the two data registers");

	memset(i2c_rd_buf, 0x55, sizeof(i2c_rd_buf));

	i2c_addr = (uint16_t)PCA9555_I2C_ADDR;
	i2c_wr_buf[0] = PCA9555_CMD_PORT0_IN;

	nread = 2u;
	nwrite = 1u;

	rc = i2c_write_read(i2c0_dev, i2c_addr, (const void *)i2c_wr_buf, nwrite,
			    (void *)i2c_rd_buf, nread);
	if (rc != 0) {
		LOG_ERR("i2c_write_read error (%d)", rc);
		app_exit_code = 7;
		goto app_exit;
	}

	LOG_INF("PCA9555 Input Port0 = 0x%02X  Port1 = 0x%02X", i2c_rd_buf[0], i2c_rd_buf[1]);

#if 0
#ifdef APP_HAS_I2C_NL
	rc = i2c_nl_test1(i2c_nl_dev);
	LOG_INF("I2C-NL test 1 returned (%d)", rc);
#endif

#if defined(CONFIG_I2C_TARGET) && defined(APP_HAS_I2C_NL)
	rc = target_i2c_nl_prepare_tests(i2c0_dev);
	LOG_INF("Target I2C-NL prepare returned (%d)", rc);

	rc = target_i2c_nl_run_tests(i2c0_dev);
	LOG_INF("Target I2C-NL run tests returned (%d)", rc);
#endif
#endif

app_exit:
	LOG_INF("Program End");
	log_flush();

	return 0;
}

#ifdef APP_HAS_I2C_NL
#define TEST_ORDER_WR_RD
 /* HW issue exists independent of transaction order. */

static int i2c_nl_test1(const struct device *i2c_dev)
{
	struct i2c_msg msgs[4] = {0};
	int rc = 0;
	uint16_t i2c_addr = 0;
	uint8_t nmsgs = 0;

	fill_buf(i2c0_tx_buf, 16u, 0x11u, 1u);

#ifdef TEST_ORDER_WR_RD
	LOG_INF("Test I2C-NL one write message");
	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 1u;

	msgs[0].buf = i2c0_tx_buf;
	msgs[0].len = 4u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);

	LOG_INF("Test I2C-NL one read message");
	memset((void *)i2c0_rx_buf, 0xAA, 8);

	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 1u;

	msgs[0].buf = i2c0_rx_buf;
	msgs[0].len = 4u;
	msgs[0].flags = I2C_MSG_READ | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);
#else
	LOG_INF("Test I2C-NL one read message");
	memset((void *)i2c0_rx_buf, 0xAA, 8);

	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 1u;

	msgs[0].buf = i2c0_rx_buf;
	msgs[0].len = 4u;
	msgs[0].flags = I2C_MSG_READ | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);

	LOG_INF("Test I2C-NL one write message");
	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 1u;

	msgs[0].buf = i2c0_tx_buf;
	msgs[0].len = 4u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);
#endif
	LOG_INF("Test I2C-NL two write messages");
	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 2u;

	msgs[0].buf = i2c0_tx_buf;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = &i2c0_tx_buf[2];
	msgs[1].len = 4u;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);

	LOG_INF("Test I2C-NL Write-Read");
	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 2u;

	msgs[0].buf = i2c0_tx_buf;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = i2c0_rx_buf;
	msgs[1].len = 8u;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);

	LOG_INF("Test I2C-NL Write message > driver xfrbuf size");
	fill_buf(i2c0_tx_buf, 66u, 0, 1);

	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 1u;

	msgs[0].buf = i2c0_tx_buf;
	msgs[0].len = 66u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);

	/* Large Write-Write */
	LOG_INF("Test I2C-NL Write-Write 64 bytes to FRAM address 0x2000");
	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 2u;

	fill_buf(&i2c0_tx_buf[2], 64, 0, 1);

	i2c0_tx_buf[0] = 0x20u; /* address MSB */
	i2c0_tx_buf[1] = 0u; /* address LSB */

	msgs[0].buf = i2c0_tx_buf;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = &i2c0_tx_buf[2];
	msgs[1].len = 64u;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);


	/* Large Write-Read */
	LOG_INF("Test I2C-NL Write-Read 64 bytes from FRAM address 0x2000");
	i2c_addr = FRAM_I2C_ADDR;
	nmsgs = 2u;

	fill_buf(i2c0_rx_buf, 64, 0x55, 0);

	i2c0_tx_buf[0] = 0x20u; /* address MSB */
	i2c0_tx_buf[1] = 0u; /* address LSB */

	msgs[0].buf = i2c0_tx_buf;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = i2c0_rx_buf;
	msgs[1].len = 64u;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	rc = i2c_transfer(i2c_dev, msgs, nmsgs, i2c_addr);
	LOG_INF("  API returned (%d)", rc);

	rc = memcmp(&i2c0_tx_buf[2], i2c0_rx_buf, 64);
	LOG_INF("Mem compare 64 byte TX data with RX data is (%d)", rc);

	return 0;
}
#endif /* APP_HAS_I2C_NL */

static void pr_i2c_regs(mm_reg_t i2c_base)
{
	if (i2c_base == 0) {
		return;
	}

	LOG_INF("I2C controller @ 0x%08x", (uint32_t)i2c_base);
	LOG_INF("  Status(RO) = 0x%02x", sys_read8(i2c_base));
	LOG_INF("  OwnAddr    = 0x%08x", sys_read32(i2c_base + 4u));
	LOG_INF("  HCMD       = 0x%08x", sys_read32(i2c_base + 0x0cu));
	LOG_INF("  TCMD       = 0x%08x", sys_read32(i2c_base + 0x10u));
	LOG_INF("  PEC        = 0x%02x", sys_read8(i2c_base + 0x14u));
	LOG_INF("  RPSH       = 0x%08x", sys_read32(i2c_base + 18u));
	LOG_INF("  EXTLEN     = 0x%08x", sys_read32(i2c_base + 0x1cu));
	LOG_INF("  CMPL       = 0x%08x", sys_read32(i2c_base + 0x20u));
	LOG_INF("  ISCL       = 0x%08x", sys_read32(i2c_base + 0x24u));
	LOG_INF("  CONFIG     = 0x%08x", sys_read32(i2c_base + 0x28u));
	LOG_INF("  BUSCLK     = 0x%08x", sys_read32(i2c_base + 0x2cu));
	LOG_INF("  BLKID      = 0x%08x", sys_read32(i2c_base + 0x30u));
	LOG_INF("  REVID      = 0x%08x", sys_read32(i2c_base + 0x34u));
	LOG_INF("  BBCR       = 0x%08x", sys_read32(i2c_base + 0x38u));
	LOG_INF("  MRSVD1     = 0x%08x", sys_read32(i2c_base + 0x3cu));
	LOG_INF("  DATTM      = 0x%08x", sys_read32(i2c_base + 0x40u));
	LOG_INF("  TMOSC      = 0x%08x", sys_read32(i2c_base + 0x44u));
	LOG_INF("  I2CFSM     = 0x%08x", sys_read32(i2c_base + 0x58u));
	LOG_INF("  NLFSM      = 0x%08x", sys_read32(i2c_base + 0x5cu));
	LOG_INF("  WKSR       = 0x%08x", sys_read32(i2c_base + 0x60u));
	LOG_INF("  WKEN       = 0x%08x", sys_read32(i2c_base + 0x64u));
	LOG_INF("  SHAD_ADDR  = 0x%08x", sys_read32(i2c_base + 0x68u));
	LOG_INF("  PROMSR     = 0x%08x", sys_read32(i2c_base + 0x70u));
	LOG_INF("  PROMIER    = 0x%08x", sys_read32(i2c_base + 0x74u));
	LOG_INF("  PROMCR     = 0x%08x", sys_read32(i2c_base + 0x78u));
	LOG_INF("  SHAD_DATA  = 0x%08x", sys_read32(i2c_base + 0x7Cu));
	log_flush();
}
