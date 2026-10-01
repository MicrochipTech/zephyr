/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * XEC I2C-NL target mode sample.
 *
 * Two register file targets are registered on port 0 of I2C controller 0, one in each
 * own address slot. An external I2C controller on the port 0 bus accesses them:
 *   write: <target addr W> <register> <data>...      writes data starting at register
 *   read:  <target addr W> <register> <RPT-START> <target addr R> <data>...
 *          or <target addr R> <data>...               reads from the register pointer
 * The register pointer advances past each byte written. A read returns the registers
 * from the pointer to the end of the register file; the pointer then resets to 0, as
 * the number of bytes the external controller reads is not known.
 *
 * At the same time this controller reads the EVB PCA9555 on the same port once per
 * second, showing controller transfers alongside target mode.
 *
 * Target callbacks run in ISR context: they only update the register file and event
 * counters. The main thread logs the counters.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define TARGET_ADDR_1 0x62U
#define TARGET_ADDR_2 0x64U

#define REG_FILE_SIZE 256U

#define PCA9555_CMD_PORT0_IN 0U

#define TARGET_PORT_NODE DT_ALIAS(i2c_target_port)
#define PCA9555_NODE     DT_NODELABEL(pca9555_evb)

#define CONTROLLER_PORT_NODE DT_ALIAS(i2c_controller_port)
#define FRAM_NODE            DT_NODELABEL(fram_evb)

struct reg_target {
	struct i2c_target_config cfg;
	uint8_t regs[REG_FILE_SIZE];
	uint8_t ptr; /* register pointer, wraps at REG_FILE_SIZE */
	atomic_t writes;
	atomic_t reads;
	atomic_t stops;
	atomic_t errors;
	atomic_t last_error;
};

static const struct device *const target_port = DEVICE_DT_GET(TARGET_PORT_NODE);
static const struct i2c_dt_spec pca9555 = I2C_DT_SPEC_GET(PCA9555_NODE);

static const struct device *const controller_port = DEVICE_DT_GET(CONTROLLER_PORT_NODE);
static const struct i2c_dt_spec fram_spec = I2C_DT_SPEC_GET(FRAM_NODE);

static struct reg_target targets[2];

static struct reg_target *to_reg_target(struct i2c_target_config *cfg)
{
	return CONTAINER_OF(cfg, struct reg_target, cfg);
}

/* First byte is the register pointer, the rest is written from there */
static void reg_target_buf_write_received(struct i2c_target_config *cfg, uint8_t *ptr,
					  uint32_t len)
{
	struct reg_target *t = to_reg_target(cfg);

	t->ptr = ptr[0];
	for (uint32_t i = 1U; i < len; i++) {
		t->regs[t->ptr] = ptr[i];
		t->ptr++;
	}

	atomic_inc(&t->writes);
}

/* Supply the registers from the pointer to the end of the register file */
static int reg_target_buf_read_requested(struct i2c_target_config *cfg, uint8_t **ptr,
					 uint32_t *len)
{
	struct reg_target *t = to_reg_target(cfg);

	*ptr = &t->regs[t->ptr];
	*len = REG_FILE_SIZE - t->ptr;
	t->ptr = 0U;

	atomic_inc(&t->reads);

	return 0;
}

static int reg_target_stop(struct i2c_target_config *cfg)
{
	atomic_inc(&to_reg_target(cfg)->stops);

	return 0;
}

static void reg_target_error(struct i2c_target_config *cfg, enum i2c_error_reason reason)
{
	struct reg_target *t = to_reg_target(cfg);

	atomic_set(&t->last_error, (atomic_val_t)reason);
	atomic_inc(&t->errors);
}

static const struct i2c_target_callbacks reg_target_callbacks = {
	.buf_write_received = reg_target_buf_write_received,
	.buf_read_requested = reg_target_buf_read_requested,
	.stop = reg_target_stop,
	.error = reg_target_error,
};

static int reg_target_register(struct reg_target *t, uint16_t addr)
{
	for (size_t i = 0; i < ARRAY_SIZE(t->regs); i++) {
		t->regs[i] = (uint8_t)i;
	}

	t->cfg.address = addr;
	t->cfg.callbacks = &reg_target_callbacks;

	return i2c_target_register(target_port, &t->cfg);
}

static void reg_target_log(const struct reg_target *t)
{
	LOG_INF("target 0x%02x: writes %ld reads %ld stops %ld errors %ld (last %ld)",
		t->cfg.address, atomic_get(&t->writes), atomic_get(&t->reads),
		atomic_get(&t->stops), atomic_get(&t->errors), atomic_get(&t->last_error));
}

enum buf_fill_alg {
	BUF_FILL_ALG_VALUE = 0,
	BUF_FILL_ALG_INCR,
	BUF_FILL_ALG_DECR,
	BUF_FILL_ALG_MAX,
};

#define FRAM_BUF_LEN 256U
static uint8_t fram_buf[FRAM_BUF_LEN];
static uint8_t fram_buf2[FRAM_BUF_LEN];

int fill_buf(uint8_t *buf, size_t buflen, uint8_t val, enum buf_fill_alg fill_alg);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, fault_inject_gpios) &&                                      \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, fault_inject_timer)
#define FAULT_INJECT 1
#endif

#ifdef FAULT_INJECT
/* Bus fault injection. A GPIO fly wired to the SDA line of the target port is pulled
 * low part way through a long host transfer, so the controller reports a bus fault and
 * the driver's error paths run on real hardware. See the board overlay for the wiring
 * and for which pin to use.
 */

/* Offset into the FRAM and the data length of the transfer the fault lands inside. At
 * the port's 100 kHz bit rate the 2 offset bytes, 32 data bytes and the addressing take
 * about 3.2 ms, so the one-shot below has a wide window to fire in.
 */
#define FAULT_FRAM_OFFSET_MSB 0x02U
#define FAULT_FRAM_OFFSET_LSB 0x20U
#define FAULT_DATA_LEN        32U

/* Delay from arming the one-shot to the pull. Comfortably inside the transfer. */
#define FAULT_DELAY_US 1000U

/* How long SDA is held low. Longer than the 10 us SCL period at 100 kHz, so the pull
 * necessarily spans an SCL high phase. A shorter pull can fall entirely within an SCL
 * low phase, where it reads as a data bit rather than a protocol violation.
 */
#define FAULT_LOW_US 25U

static const struct gpio_dt_spec fault_pin =
	GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, fault_inject_gpios);
static const struct device *const fault_timer =
	DEVICE_DT_GET(DT_PHANDLE(ZEPHYR_USER_NODE, fault_inject_timer));

static uint8_t fault_buf[2U + FAULT_DATA_LEN];
static atomic_t fault_fired;

/* Counter callback, ISR context. The pull is released here rather than from the thread
 * after the transfer returns: the driver's bus recovery clocks SCL and checks SDA, so a
 * line still held low would turn the fault into a wedged bus and recovery would fail.
 *
 * The release is a busy wait because this basic timer reports a single alarm channel, so
 * a second alarm cannot be scheduled to do it. A few tens of microseconds in an ISR is
 * acceptable in a test, not in a driver.
 */
static void fault_inject_cb(const struct device *dev, uint8_t chan_id, uint32_t ticks,
			    void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan_id);
	ARG_UNUSED(ticks);
	ARG_UNUSED(user_data);

	(void)gpio_pin_set_dt(&fault_pin, 0);
	k_busy_wait(FAULT_LOW_US);
	(void)gpio_pin_set_dt(&fault_pin, 1);

	atomic_set(&fault_fired, 1);
}

static int fault_inject_init(void)
{
	int rc = 0;

	if (!gpio_is_ready_dt(&fault_pin)) {
		LOG_ERR("Fault injection GPIO device is not ready");
		return -ENODEV;
	}

	if (!device_is_ready(fault_timer)) {
		LOG_ERR("Fault injection counter device is not ready");
		return -ENODEV;
	}

	/* Open drain driven high is the released state: SDA is left to the bus pull-up */
	rc = gpio_pin_configure_dt(&fault_pin, GPIO_OUTPUT_HIGH);
	if (rc != 0) {
		LOG_ERR("Fault injection GPIO configure error (%d)", rc);
		return rc;
	}

	rc = counter_start(fault_timer);
	if (rc != 0) {
		LOG_ERR("Fault injection counter start error (%d)", rc);
		return rc;
	}

	LOG_INF("Fault injection armed on %s pin %u, timer %s", fault_pin.port->name,
		fault_pin.pin, fault_timer->name);

	return 0;
}

static void fault_inject_test(void)
{
	struct counter_alarm_cfg alarm = {
		.callback = fault_inject_cb,
		.user_data = NULL,
		.flags = 0U,
	};
	int rc = 0;

	fault_buf[0] = FAULT_FRAM_OFFSET_MSB;
	fault_buf[1] = FAULT_FRAM_OFFSET_LSB;
	(void)fill_buf(&fault_buf[2], FAULT_DATA_LEN, 0, BUF_FILL_ALG_INCR);

	alarm.ticks = counter_us_to_ticks(fault_timer, FAULT_DELAY_US);
	atomic_set(&fault_fired, 0);

	rc = counter_set_channel_alarm(fault_timer, 0, &alarm);
	if (rc != 0) {
		LOG_ERR("Fault injection alarm error (%d)", rc);
		return;
	}

	rc = i2c_write_dt(&fram_spec, fault_buf, sizeof(fault_buf));

	if (atomic_get(&fault_fired) == 0) {
		LOG_WRN("Fault injection did not fire inside the transfer (rc %d)", rc);
		(void)counter_cancel_channel_alarm(fault_timer, 0);
		return;
	}

	/* The driver reports an arbitration loss, a bus error and a time-out as -EIO */
	if (rc == -EIO) {
		LOG_INF("Fault injection: transfer failed with -EIO, as expected");
	} else if (rc == 0) {
		LOG_INF("Fault injection: transfer completed, no fault seen. Either the "
			"fly wire is not fitted or the pull missed an SCL high phase");
	} else {
		LOG_WRN("Fault injection: transfer failed with %d, expected -EIO", rc);
	}

	/* The aborted transfer can leave the addressed device holding SDA */
	rc = i2c_recover_bus(fram_spec.bus);
	if (rc != 0) {
		LOG_ERR("Bus recovery after fault injection error (%d)", rc);
	}
}
#else
static int fault_inject_init(void)
{
	return 0;
}

static void fault_inject_test(void)
{
}
#endif /* FAULT_INJECT */

int main(void)
{
	uint64_t loop_count = 0;
	uint8_t cmd = PCA9555_CMD_PORT0_IN;
	uint8_t port0[2] = {0};
	int rc = 0;

	LOG_INF("MEC_ASSY6941 I2C-NL target mode sample");

	memset((void *)fram_buf, 0, sizeof(fram_buf));
	memset((void *)fram_buf2, 0, sizeof(fram_buf2));

	if (!device_is_ready(controller_port)) {
		LOG_ERR("I2C controller port device is not ready");
		return 0;
	}

	if (!device_is_ready(target_port)) {
		LOG_ERR("I2C target port device is not ready");
		return 0;
	}

	rc = reg_target_register(&targets[0], TARGET_ADDR_1);
	if (rc != 0) {
		LOG_ERR("Register target 0x%02x error (%d)", TARGET_ADDR_1, rc);
		return 0;
	}

	rc = reg_target_register(&targets[1], TARGET_ADDR_2);
	if (rc != 0) {
		LOG_ERR("Register target 0x%02x error (%d)", TARGET_ADDR_2, rc);
		return 0;
	}

	LOG_INF("Targets 0x%02x and 0x%02x registered", TARGET_ADDR_1, TARGET_ADDR_2);

	rc = fault_inject_init();
	if (rc != 0) {
		LOG_WRN("Fault injection unavailable (%d), continuing without it", rc);
	}

	while (true) {
		rc = i2c_write_read_dt(&pca9555, &cmd, 1U, port0, sizeof(port0));
		if (rc != 0) {
			LOG_ERR("PCA9555 read error (%d)", rc);
		} else {
			LOG_INF("PCA9555 input port 0 = 0x%02x%02x", port0[1], port0[0]);
		}

		fram_buf[0] = 0x02U; /* MSB of FRAM memory array offset */
		fram_buf[1] = 0x10U; /* LSB of FRAM memory array offset */
		fill_buf(&fram_buf[2], 32U, 0, BUF_FILL_ALG_INCR);

		/* write 16-bit offset plus 32 bytes of data */
		rc = i2c_write_dt(&fram_spec, fram_buf, 34U);
		if (rc != 0) {
			LOG_ERR("FRAM write offset plus data error (%d)", rc);
		} else {
			LOG_INF("FRAM write offset plus data OK");
		}

		memset((void *)fram_buf2, 0, sizeof(fram_buf2));
		rc = i2c_write_read_dt(&fram_spec, fram_buf, 2U, fram_buf2, 32U);
		if (rc != 0) {
			LOG_ERR("FRAM read back of data error (%d)", rc);
		} else {
			LOG_INF("FRAM read back of data ok");
			rc = memcmp((void *)&fram_buf[2], (void *)fram_buf2, 32U);
			if (rc == 0) {
				LOG_INF("Data compare: PASS");
			} else {
				LOG_ERR("Data mismatch: FAIL");
			}
		}

		fram_buf2[0] = 0x11U;
		fram_buf2[1] = 0x22U;
		LOG_INF("Read 2 bytes from target at 0x%02x", TARGET_ADDR_1);
		rc = i2c_read(fram_spec.bus, fram_buf2, 2U, TARGET_ADDR_1);
		if (rc != 0) {
			LOG_ERR("Read from target 1 error (%d)", rc);
		} else {
			LOG_INF("Read data = 0x%02x, 0x%02x", fram_buf2[0], fram_buf2[1]);
		}

		fram_buf[0] = 0x01U;
		LOG_INF("Write 0x%02x to target at 0x%02x", fram_buf[0], TARGET_ADDR_1);
		rc = i2c_write(fram_spec.bus, (const uint8_t *)fram_buf, 1U, TARGET_ADDR_1);
		if (rc != 0) {
			LOG_ERR("Write to target 1 error (%d)", rc);
		}

		fram_buf[4] = 0x02U;
		LOG_INF("Write 0x%02x to target at 0x%02x", fram_buf[4], TARGET_ADDR_2);
		rc = i2c_write(fram_spec.bus, (const uint8_t *)&fram_buf[4], 1U, TARGET_ADDR_2);
		if (rc != 0) {
			LOG_ERR("Write to target 2 error (%d)", rc);
		}

		fram_buf[0] = 0x30;
		fram_buf[1] = 0x31;
		fram_buf[2] = 0x32;
		fram_buf[3] = 0x33;
		LOG_HEXDUMP_INF(fram_buf, 4, "Write this to TARGET_ADDR_1");
		rc = i2c_write(fram_spec.bus, (const uint8_t *)fram_buf, 4U, TARGET_ADDR_1);
		if (rc != 0) {
			LOG_ERR("Write to target 1 error (%d)", rc);
		}

		fram_buf2[0] = 0xAAU;
		fram_buf2[1] = 0xAAU;
		fram_buf2[2] = 0xAAU;
		fram_buf2[3] = 0xAAU;
		LOG_INF("Read 4 bytes from target at 0x%02x", TARGET_ADDR_1);
		rc = i2c_read(fram_spec.bus, fram_buf2, 4U, TARGET_ADDR_1);
		if (rc != 0) {
			LOG_ERR("Read from target 1 error (%d)", rc);
		} else {
			LOG_HEXDUMP_INF(fram_buf2, 4, "Read back data");
		}

		fault_inject_test();

		for (size_t i = 0; i < ARRAY_SIZE(targets); i++) {
			reg_target_log(&targets[i]);
		}

		loop_count++;
		LOG_INF("Loop count = %llu", loop_count);

		k_msleep(1000);
	}

	return 0;
}

int fill_buf(uint8_t *buf, size_t buflen, uint8_t val, enum buf_fill_alg fill_alg)
{
	if (buf == NULL) {
		return -EINVAL;
	}

	if (buflen == 0) {
		return 0;
	}

	switch (fill_alg) {
	case BUF_FILL_ALG_VALUE:
		memset((void *)buf, (int)val, buflen);
		break;
	case BUF_FILL_ALG_INCR:
		for (size_t n = 0; n < buflen; n++) {
			buf[n] = (uint8_t)(n % 256U);
		}
		break;
	case BUF_FILL_ALG_DECR:
		for (size_t n = 0; n < buflen; n++) {
			buf[n] = (uint8_t)((buflen - n) % 256U);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
