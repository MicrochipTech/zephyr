/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/target/eeprom.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/dt-bindings/i2c/mchp-xec-i2c.h>

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

#define LTC2489_ADC_CONV_TIME_MS 150
#define LTC2489_ADC_READ_RETRIES 10

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define I2C_SMB_GET_DEV(nid) DEVICE_DT_GET(nid),

#define NODE_PCA9555 DT_NODELABEL(pca9555_evb)
#define NODE_LTC2489 DT_NODELABEL(ltc2489_evb)
#define NODE_FRAM    DT_NODELABEL(mb85rc256v_fram)

const struct gpio_dt_spec i2c_p0_pin_test =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, i2c_p0_test_gpios, 0);

const struct i2c_dt_spec pca9555_spec = I2C_DT_SPEC_GET(NODE_PCA9555);
const struct i2c_dt_spec ltc2489_spec = I2C_DT_SPEC_GET(NODE_LTC2489);
const struct i2c_dt_spec mb_fram_spec = I2C_DT_SPEC_GET(NODE_FRAM);

#ifdef CONFIG_I2C_TARGET
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
#define NODE_EP1 DT_NODELABEL(eeprom1)
const struct i2c_dt_spec ep1_spec = I2C_DT_SPEC_GET(NODE_EP1);
const struct device *ep1_dev = DEVICE_DT_GET(NODE_EP1);
#endif
#endif

static const struct device *i2c_smb_ctrls[] = {
	DT_FOREACH_STATUS_OKAY(microchip_xec_i2c_v3, I2C_SMB_GET_DEV)};

/* Ports on the controllers */
static const struct device *i2c_smb_ports[] = {
	DT_FOREACH_STATUS_OKAY(microchip_xec_i2c_v3_port, I2C_SMB_GET_DEV)};

#define I2C_MAX_MSGS    8
#define I2C_TX_BUF_SIZE 256
#define I2C_RX_BUF_SIZE 256

static struct i2c_msg msgs[I2C_MAX_MSGS];
static uint8_t i2c_tx_buf[I2C_TX_BUF_SIZE];
static uint8_t i2c_rx_buf[I2C_RX_BUF_SIZE];

#ifdef CONFIG_I2C_TARGET
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
#define EEPROM_TEST_DATA_SIZE MIN(CONFIG_I2C_TEST_DATA_MAX_SIZE, DT_PROP(NODE_EP1, size))

static uint8_t eeprom_1_data[EEPROM_TEST_DATA_SIZE];
static uint8_t target_test_buf[EEPROM_TEST_DATA_SIZE];

static void init_eeprom_test_data(void);
static int target_buffer_test1(void);
#endif
#endif

#ifdef CONFIG_I2C_CALLBACK
#ifdef CONFIG_POLL
static struct k_poll_signal xfer_signal;
static int i2c_callback_signal_test(const struct i2c_dt_spec *dts, struct k_poll_signal *signal);
#endif
#endif

static int pca9555_test1(const struct i2c_dt_spec *dts);
static int ltc2489_test1(const struct i2c_dt_spec *dts);
static int fram_test1(const struct i2c_dt_spec *dts);
static int fram_test2(const struct i2c_dt_spec *dts);

int main(void)
{
	int ctrl_ready_count = 0;
	int port_ready_count = 0;
	int rc = 0;

	memset((void *)msgs, 0, sizeof(msgs));
	memset(i2c_tx_buf, 0x55, I2C_TX_BUF_SIZE);
	memset(i2c_rx_buf, 0xAA, I2C_RX_BUF_SIZE);

	if (!gpio_is_ready_dt(&i2c_p0_pin_test)) {
		LOG_ERR("I2C Port 0 test pin is not ready!");
		goto app_exit;
	}

	/* Set pin as both input and output. Drive output high */
	gpio_pin_configure_dt(&i2c_p0_pin_test, GPIO_OUTPUT_HIGH | GPIO_INPUT);

	for (size_t i = 0; i < ARRAY_SIZE(i2c_smb_ctrls); i++) {
		const struct device *ctrl_dev = i2c_smb_ctrls[i];

		if (ctrl_dev == NULL) {
			continue;
		}

		if (device_is_ready(ctrl_dev)) {
			ctrl_ready_count++;
			LOG_INF("I2C Controller device %s is ready", ctrl_dev->name);
		} else {
			LOG_ERR("I2C Controller device %s is NOT ready", ctrl_dev->name);
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(i2c_smb_ports); i++) {
		const struct device *port_dev = i2c_smb_ports[i];

		if (port_dev == NULL) {
			continue;
		}

		if (device_is_ready(port_dev)) {
			port_ready_count++;
			LOG_INF("I2C Port device %s is ready", port_dev->name);
		} else {
			LOG_ERR("I2C Port device %s is NOT ready", port_dev->name);
		}
	}

	log_flush();

	if ((ctrl_ready_count != ARRAY_SIZE(i2c_smb_ctrls)) ||
	    (port_ready_count != ARRAY_SIZE(i2c_smb_ports))) {
		LOG_ERR("Skipping tests since some or all drivers are not ready");
		goto app_exit;
	}

	LOG_INF("Test communication with PCA9555");
	rc = pca9555_test1(&pca9555_spec);
	if (rc == 0) {
		LOG_INF("PCA9555 test 1 PASS");
	} else if (rc == -ENOTSUP) {
		LOG_INF("PCA9555 test1 not implemented");
	} else {
		LOG_ERR("PCA9555 test 1 FAIL (%d)", rc);
	}

	LOG_INF("Test communication with LTC2489");
	rc = ltc2489_test1(&ltc2489_spec);
	if (rc == 0) {
		LOG_INF("LTC2489 test 1 PASS");
	} else if (rc == -ENOTSUP) {
		LOG_INF("LTC2489 test 1 not implemented");
	} else {
		LOG_ERR("LTC2489 test 1 FAIL (%d)", rc);
	}

	LOG_INF("Test communication with FRAM");
	rc = fram_test1(&mb_fram_spec);
	if (rc == 0) {
		LOG_INF("FRAM test 1 PASS");
	} else if (rc == -ENOTSUP) {
		LOG_INF("FRAM test 1 not implemented");
	} else {
		LOG_ERR("FRAM test 1 FAIL (%d)", rc);
	}

#ifdef CONFIG_I2C_TARGET
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	if (!device_is_ready(ep1_spec.bus)) {
		LOG_ERR("Virtual EEPROM driver is not ready!");
		goto app_exit;
	}

	LOG_INF("Virtual EEPROM test data size = %u", EEPROM_TEST_DATA_SIZE);
	init_eeprom_test_data();
	rc = eeprom_target_write_data(ep1_dev, 0, eeprom_1_data, EEPROM_TEST_DATA_SIZE);
	if (rc != 0) {
		LOG_ERR("Failed to program EEPROM 1 (%d)", rc);
		goto app_exit;
	}

	LOG_INF("Register EEPROM target with I2C driver");
	rc = i2c_target_driver_register(ep1_dev);
	if (rc != 0) {
		LOG_ERR("Failed to attached EEPROM 1 as I2C target (%d)", rc);
		goto app_exit;
	}

	rc = target_buffer_test1();
	LOG_INF("Target test 1 returned (%d)", rc);
#endif
#endif /* CONFIG_I2C_TARGET */

	LOG_INF("Try recovery API on FRAM bus");

	log_flush();

	/* Do an access. Driver will be set to FRAM port */
	rc = fram_test1(&mb_fram_spec);
	if (rc != 0) {
		LOG_ERR("FRAM test 1 failure (%d)", rc);
		goto app_exit;
	}

	LOG_INF("Drive SDA low");
	gpio_pin_set_dt(&i2c_p0_pin_test, 0);

	k_busy_wait(2000U); /* 2 ms */
	/* Need an asynchronous callback to drive it back high before i2c_recover_bus
	 * finishes retries.
	 */

	gpio_pin_set_dt(&i2c_p0_pin_test, 1);

	/* We observe I2C00.STATUS 0x81 to 0x80 cleared NBB bit
	 * Recover routine reset the controller and observed a reset
	 * cleared the issue and returned success
	 */

	rc = i2c_recover_bus(mb_fram_spec.bus);
	if (rc != 0) {
		LOG_ERR("I2C recover bus API error (%d)", rc);
		/* goto app_exit; */
	}

	LOG_INF("After recovery attempt. Test FRAM access");
	rc = fram_test2(&mb_fram_spec);
	if (rc == 0) {
		LOG_INF("After recovery FRAM I2C access: PASS");
	} else {
		LOG_ERR("After recovery FRAM I2C access failed (%d)", rc);
		goto app_exit;
	}

#ifdef CONFIG_I2C_CALLBACK
#ifdef CONFIG_POLL
	rc = i2c_callback_signal_test(&mb_fram_spec, &xfer_signal);
	if (rc == 0) {
		LOG_INF("I2C callback test using a signal is PASS");
	} else {
		LOG_ERR("I2C callback test using a signal is FAIL (%d)", rc);
		goto app_exit;
	}
#endif
#endif

app_exit:
	LOG_INF("Program End");
	log_flush();

	return 0;
}

static int pca9555_test1(const struct i2c_dt_spec *dts)
{
	int rc = -ENOTSUP;

	if (dts == NULL) {
		return -EINVAL;
	}

	memset(i2c_tx_buf, 0x55, sizeof(i2c_tx_buf));
	memset(i2c_rx_buf, 0xAA, sizeof(i2c_rx_buf));

	i2c_tx_buf[0] = PCA9555_CMD_PORT0_IN;

	rc = i2c_write_read_dt(dts, i2c_tx_buf, 1U, i2c_rx_buf, 2U);
	if (rc != 0) {
		return rc;
	}

	LOG_INF("PCA9555 read ports 0x%0x 0x%0x", i2c_rx_buf[0], i2c_rx_buf[1]);

	return rc;
}

static int ltc2489_test1(const struct i2c_dt_spec *dts)
{
	int rc = -ENOTSUP;
	uint32_t adc_retry_count = 0, reading = 0;

	if (dts == NULL) {
		return -EINVAL;
	}

	/* Read ADC channel 0. Selecting the channel initiates a reading */
	i2c_tx_buf[0] = 0xb0U; /* 1011_0000 selects channel 0 as single ended */

	rc = i2c_write_dt(dts, i2c_tx_buf, 1U);
	if (rc != 0) {
		LOG_ERR("I2C write to LTC2489 channel select error (%d)", rc);
		return rc;
	}

	/* LTC2489 will NAK while it is converting */
	k_sleep(K_MSEC(LTC2489_ADC_CONV_TIME_MS));

	do {
		i2c_rx_buf[0] = 0x55U;
		i2c_rx_buf[1] = 0x55U;
		i2c_rx_buf[2] = 0x55U;

		rc = i2c_read_dt(dts, i2c_rx_buf, 3U);
		if (rc != 0) {
			adc_retry_count++;
		}

	} while ((rc != 0) && (adc_retry_count < LTC2489_ADC_READ_RETRIES));

	if (rc == 0) {
		reading = ((uint32_t)i2c_rx_buf[0] + ((uint32_t)i2c_rx_buf[1] << 8) +
			   ((uint32_t)i2c_rx_buf[2] << 16));
		LOG_INF("LTC2489 conversion done: raw reading = 0x%0x", reading);
	} else {
		LOG_ERR("LTC2489 conversion timeout: FAIL");
	}

	return rc;
}

static int fram_test1(const struct i2c_dt_spec *dts)
{
	int rc = -ENOTSUP;

	if (dts == NULL) {
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

	rc = i2c_write_dt(dts, i2c_tx_buf, 6U);
	if (rc != 0) {
		return rc;
	}

	i2c_tx_buf[0] = 0x43U;
	i2c_tx_buf[1] = 0x21U;

	rc = i2c_write_read_dt(dts, i2c_tx_buf, 2U, i2c_rx_buf, 4U);
	if (rc != 0) {
		return rc;
	}

	rc = memcmp(&i2c_tx_buf[2], i2c_rx_buf, 4U);
	if (rc != 0) {
		rc = -EPERM;
	}

	return rc;
}

static int fram_test2(const struct i2c_dt_spec *dts)
{
	int rc = -ENOTSUP;

	if (dts == NULL) {
		return -EINVAL;
	}

	memset(i2c_tx_buf, 0x55, sizeof(i2c_tx_buf));
	memset(i2c_rx_buf, 0xAA, sizeof(i2c_rx_buf));

	i2c_tx_buf[0] = 0x12U;
	i2c_tx_buf[1] = 0x30U;
	for (uint32_t i = 0; i < 32U; i++) {
		i2c_tx_buf[i + 2U] = (uint8_t)(i % 256U);
	}

	rc = i2c_write_dt(dts, i2c_tx_buf, 34U);
	if (rc != 0) {
		return rc;
	}

	i2c_tx_buf[0] = 0x12U;
	i2c_tx_buf[1] = 0x30U;

	rc = i2c_write_read_dt(dts, i2c_tx_buf, 2U, i2c_rx_buf, 32U);
	if (rc != 0) {
		return rc;
	}

	rc = memcmp(&i2c_tx_buf[2], i2c_rx_buf, 32U);
	if (rc != 0) {
		rc = -EPERM;
	}

	return rc;
}

#ifdef CONFIG_I2C_TARGET
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
static void init_eeprom_test_data(void)
{
	/*
	 * Initialize EEPROM data with printable ASCII value (range [32 126]).
	 * Make sure content differs between eeprom_0_data[] and eeprom_1_data[].
	 */
	for (size_t n = 0; n < sizeof(eeprom_1_data); n++) {
		eeprom_1_data[n] = 32 + (n % (126 - 32));
	}
}

static int target_buffer_test1(void)
{
	int rc = 0;
	uint8_t eeprom_ofs[2] = {0};

	LOG_INF("Begin target test 1");

	memset(target_test_buf, 0, sizeof(target_test_buf));

	rc = i2c_write_read(pca9555_spec.bus, ep1_spec.addr, eeprom_ofs, 2, target_test_buf,
			    EEPROM_TEST_DATA_SIZE);
	if (rc != 0) {
		LOG_ERR("Target test 1 i2c write read error (%d)", rc);
		return rc;
	}

	rc = memcmp(target_test_buf, eeprom_1_data, EEPROM_TEST_DATA_SIZE);
	if (rc == 0) {
		LOG_INF("Read back match: PASS");
	} else {
		LOG_ERR("Miscompare in expected data pattern");
		LOG_HEXDUMP_ERR(target_test_buf, EEPROM_TEST_DATA_SIZE, "read back");
		LOG_HEXDUMP_ERR(eeprom_1_data, EEPROM_TEST_DATA_SIZE, "expected ");
	}

	return rc;
}
#endif /* CONFIG_I2C_TARGET_BUFFER_MODE */
#endif /* CONFIG_I2C_TARGET */

#ifdef CONFIG_I2C_CALLBACK
#ifdef CONFIG_POLL
static int i2c_callback_signal_test(const struct i2c_dt_spec *dts, struct k_poll_signal *signal)
{
	unsigned int signaled = 0;
	int signal_result = 0;
	int rc = 0;

	LOG_INF("I2C CB KPOLL test");

	if ((dts == NULL) || (signal == NULL)) {
		return -EINVAL;
	}

	memset(i2c_tx_buf, 0x55, sizeof(i2c_tx_buf));
	memset(i2c_rx_buf, 0xAA, sizeof(i2c_rx_buf));

	k_poll_signal_init(signal);

	struct k_poll_event events[1] = {
		K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, signal),
	};

	i2c_tx_buf[0] = 0x11U;
	i2c_tx_buf[1] = 0x30U;
	for (size_t i = 0; i < 8U; i++) {
		i2c_tx_buf[i + 2] = (uint8_t)(i + 1U);
	}

	/* First message write FRAM: I2C write of 16-bit MSBF offset plus data */
	msgs[0].buf = i2c_tx_buf;
	msgs[0].len = 2U + 8U;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	/* Second and third messages: Write-Read 16-bit MSBF offset and read back data */
	msgs[1].buf = i2c_tx_buf;
	msgs[1].len = 2U;
	msgs[1].flags = I2C_MSG_WRITE;

	msgs[2].buf = &i2c_rx_buf[16];
	msgs[2].len = 8U;
	msgs[2].flags = I2C_MSG_RESTART | I2C_MSG_STOP | I2C_MSG_READ;

	rc = i2c_transfer_signal(dts->bus, msgs, 3, dts->addr, signal);
	if (rc == 0) {
		LOG_INF("I2C xfer signal return OK. Poll for completion");
	} else if (rc == -ENOSYS) {
		LOG_ERR("I2C xfer signal error (%d) -ENOSYS. Driver does not implement async", rc);
		return rc;
	} else if (rc == -EWOULDBLOCK) {
		LOG_ERR("I2C xfer signal error (%d) -EWOULDBLOCK. Driver is busy", rc);
		return rc;
	} else {
		LOG_ERR("I2C xfer signal generic error (%d)", rc);
		return rc;
	}

	/* Poll signal */
	k_poll(events, 1, K_FOREVER);

	k_poll_signal_check(signal, &signaled, &signal_result);

	LOG_INF("Poll signaled %d, signal result %u", signaled, signal_result);

	if ((signaled == 0) || (signal_result < 0)) {
		LOG_ERR("k_poll signal error");
	}

	rc = memcmp(&i2c_tx_buf[2], &i2c_rx_buf[16], 8U);
	if (rc == 0) {
		LOG_INF("I2C xfer signal data matches: PASS");
	} else {
		LOG_ERR("I2C xfer signal data mismatch: FAIL");
	}

	return rc;
}
#endif /* CONFIG_POLL */
#endif /* CONFIG_I2C_CALLBACK */
