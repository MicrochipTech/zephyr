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

int main(void)
{
	uint8_t cmd = PCA9555_CMD_PORT0_IN;
	uint8_t port0[2] = {0};
	int rc = 0;

	LOG_INF("MEC_ASSY6941 I2C-NL target mode sample");

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

	while (true) {
		rc = i2c_write_read_dt(&pca9555, &cmd, 1U, port0, sizeof(port0));
		if (rc != 0) {
			LOG_ERR("PCA9555 read error (%d)", rc);
		} else {
			LOG_INF("PCA9555 input port 0 = 0x%02x%02x", port0[1], port0[0]);
		}

		for (size_t i = 0; i < ARRAY_SIZE(targets); i++) {
			reg_target_log(&targets[i]);
		}

		k_msleep(1000);
	}

	return 0;
}
