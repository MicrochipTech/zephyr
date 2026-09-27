/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C controller traffic to stress the XEC I2C-NL driver in target mode.
 *
 * The target board runs samples/boards/microchip/mec_assy6941/i2c_targ_mode: two 256
 * byte register file targets on its I2C controller 0 port 0.
 *   write: <addr W> <register> <data>...  the pointer is set to register, then advances
 *                                         past each byte written (wrapping at 256)
 *   read:  a read request supplies the registers from the pointer to the end of the
 *          file, then the pointer resets to 0
 * A write segment longer than CONFIG_APP_TARGET_BUFFER_SIZE, counting the address
 * byte, is NACKed and dropped. Each segment of a repeated START write is delivered on
 * its own.
 *
 * This board's I2C controller issues a random mix of transfers back to back and checks
 * every read against a model of both register files. The target board's controller
 * also uses the bus, so a transfer can lose arbitration. After an unexpected error the
 * target's state is unknown: the error is counted and, after CONFIG_APP_RETRY_DELAY_MS,
 * the register file is rewritten before the model is used again. A data mismatch while
 * the model is known is a verify failure.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define REG_FILE_SIZE 256U
#define RESYNC_TRIES  5
#define MAX_MISMATCH_LOGS 8U

/* Largest overflow write: the target buffer plus a margin */
#define OVERFLOW_LEN (CONFIG_APP_TARGET_BUFFER_SIZE + 8)

enum op {
	OP_WRITE,
	OP_WRITE_2MSG,
	OP_WRITE_READ,
	OP_READ,
	OP_RPT_WRITE,
	OP_FULL,
	OP_OVERFLOW,
	OP_OVER_READ,
	OP_ABSENT,
	OP_SPEED,
	OP_COUNT,
};

static const char *const op_names[OP_COUNT] = {
	"write", "write 2 msgs", "write-read", "read", "rpt-start write",
	"full 256", "overflow", "over-read", "absent addr", "speed",
};

/* Relative weight of each operation in the random mix */
static const uint8_t op_weight[OP_COUNT] = {
	[OP_WRITE] = 20, [OP_WRITE_2MSG] = 15, [OP_WRITE_READ] = 25, [OP_READ] = 15,
	[OP_RPT_WRITE] = 8, [OP_FULL] = 3, [OP_OVERFLOW] = 2, [OP_OVER_READ] = 4,
	[OP_ABSENT] = 4, [OP_SPEED] = 4,
};

struct target_model {
	uint16_t addr;
	uint8_t regs[REG_FILE_SIZE];
	uint8_t ptr;
	bool valid;
};

struct stats {
	uint32_t ops[OP_COUNT];
	uint32_t errors[OP_COUNT]; /* unexpected transfer errors */
	uint32_t verify_fail;
	uint32_t ovf_interrupted; /* overflow writes cut short by lost arbitration */
	uint32_t resyncs;
	uint32_t resync_fail;
	uint64_t bytes_wr;
	uint64_t bytes_rd;
};

static const struct device *const i2c_dev = DEVICE_DT_GET(DT_ALIAS(i2c_host));

static struct target_model targets[2];
static struct stats st;
static uint32_t rng_state;
static uint32_t cur_speed = I2C_SPEED_STANDARD;

static uint8_t wbuf[OVERFLOW_LEN];
static uint8_t rbuf[REG_FILE_SIZE];
static uint8_t expect[REG_FILE_SIZE];

/* xorshift32: reproducible with CONFIG_APP_SEED */
static uint32_t rnd(void)
{
	rng_state ^= rng_state << 13;
	rng_state ^= rng_state >> 17;
	rng_state ^= rng_state << 5;

	return rng_state;
}

static uint32_t rnd_range(uint32_t lo, uint32_t hi)
{
	return lo + (rnd() % (hi - lo + 1U));
}

static void fill_random(uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		buf[i] = (uint8_t)rnd();
	}
}

/* Model of the target's buf_write_received(): the first byte sets the pointer */
static void model_write(struct target_model *t, const uint8_t *data, size_t len)
{
	if (len == 0U) {
		return;
	}

	t->ptr = data[0];
	for (size_t i = 1; i < len; i++) {
		t->regs[t->ptr] = data[i];
		t->ptr++;
	}
}

/* Model of the target's buf_read_requested(). Returns the bytes supplied. */
static size_t model_read(struct target_model *t, uint8_t *out)
{
	size_t n = REG_FILE_SIZE - t->ptr;

	memcpy(out, &t->regs[t->ptr], n);
	t->ptr = 0U;

	return n;
}

/* A mismatch leaves the model unknown: the register file is rewritten before reuse */
static void verify(enum op op, struct target_model *t, const uint8_t *got, const uint8_t *exp,
		   size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (got[i] != exp[i]) {
			if (st.verify_fail < MAX_MISMATCH_LOGS) {
				LOG_ERR("%s 0x%02x: byte %zu is 0x%02x, expected 0x%02x",
					op_names[op], t->addr, i, got[i], exp[i]);
			}
			st.verify_fail++;
			t->valid = false;
			return;
		}
	}
}

/* Rewrite the whole register file so the model is known again */
static int resync(struct target_model *t)
{
	int rc = -EIO;

	st.resyncs++;
	wbuf[0] = 0x00U;

	for (int i = 0; (i < RESYNC_TRIES) && (rc != 0); i++) {
		fill_random(&wbuf[1], REG_FILE_SIZE);
		rc = i2c_write(i2c_dev, wbuf, 1U + REG_FILE_SIZE, t->addr);
		if (rc != 0) {
			k_msleep(CONFIG_APP_RETRY_DELAY_MS);
		}
	}

	if (rc != 0) {
		st.resync_fail++;
		t->valid = false;
		return rc;
	}

	model_write(t, wbuf, 1U + REG_FILE_SIZE);
	t->valid = true;
	st.bytes_wr += 1U + REG_FILE_SIZE;

	return 0;
}

static void op_error(enum op op, struct target_model *t, int rc)
{
	st.errors[op]++;
	LOG_DBG("%s 0x%02x error (%d)", op_names[op], t->addr, rc);
	t->valid = false;
}

static void do_write(struct target_model *t, enum op op)
{
	size_t len = rnd_range(1U, 64U);
	int rc;

	wbuf[0] = (uint8_t)rnd();
	fill_random(&wbuf[1], len);

	if (op == OP_WRITE) {
		rc = i2c_write(i2c_dev, wbuf, 1U + len, t->addr);
	} else {
		/* Register and data in separate messages: one write on the bus */
		struct i2c_msg msgs[2] = {
			{.buf = &wbuf[0], .len = 1U, .flags = I2C_MSG_WRITE},
			{.buf = &wbuf[1], .len = len, .flags = I2C_MSG_WRITE | I2C_MSG_STOP},
		};

		rc = i2c_transfer(i2c_dev, msgs, ARRAY_SIZE(msgs), t->addr);
	}

	if (rc != 0) {
		op_error(op, t, rc);
		return;
	}

	model_write(t, wbuf, 1U + len);
	st.bytes_wr += 1U + len;
}

/* <addr W> <reg> <RPT-START> <addr R> <len>. Bytes past those supplied are not checked. */
static void do_write_read(struct target_model *t, enum op op, uint8_t reg, size_t len)
{
	size_t supplied;
	int rc = i2c_write_read(i2c_dev, t->addr, &reg, 1U, rbuf, len);

	if (rc != 0) {
		op_error(op, t, rc);
		return;
	}

	st.bytes_rd += len;
	model_write(t, &reg, 1U);
	supplied = model_read(t, expect);
	verify(op, t, rbuf, expect, MIN(len, supplied));
}

static void do_read(struct target_model *t)
{
	size_t len = rnd_range(1U, 32U);
	size_t supplied;
	int rc = i2c_read(i2c_dev, rbuf, len, t->addr);

	if (rc != 0) {
		op_error(OP_READ, t, rc);
		return;
	}

	st.bytes_rd += len;
	supplied = model_read(t, expect);
	verify(OP_READ, t, rbuf, expect, MIN(len, supplied));
}

/* <addr W> <r0> <RPT-START> <addr W> <r1> <data>: each segment delivered on its own */
static void do_rpt_write(struct target_model *t)
{
	size_t len = rnd_range(1U, 32U);
	struct i2c_msg msgs[2] = {
		{.buf = &wbuf[0], .len = 1U, .flags = I2C_MSG_WRITE},
		{.buf = &wbuf[1], .len = 1U + len,
		 .flags = I2C_MSG_WRITE | I2C_MSG_RESTART | I2C_MSG_STOP},
	};
	int rc;

	fill_random(wbuf, 2U + len);
	rc = i2c_transfer(i2c_dev, msgs, ARRAY_SIZE(msgs), t->addr);
	if (rc != 0) {
		op_error(OP_RPT_WRITE, t, rc);
		return;
	}

	model_write(t, &wbuf[0], 1U);
	model_write(t, &wbuf[1], 1U + len);
	st.bytes_wr += 2U + len;
}

/* Full register file write, then read it all back */
static void do_full(struct target_model *t)
{
	int rc;

	wbuf[0] = 0x00U;
	fill_random(&wbuf[1], REG_FILE_SIZE);
	rc = i2c_write(i2c_dev, wbuf, 1U + REG_FILE_SIZE, t->addr);
	if (rc != 0) {
		op_error(OP_FULL, t, rc);
		return;
	}

	model_write(t, wbuf, 1U + REG_FILE_SIZE);
	st.bytes_wr += 1U + REG_FILE_SIZE;
	do_write_read(t, OP_FULL, 0x00U, REG_FILE_SIZE);
}

/* Longer than the target buffer: must fail, and the data must be dropped.
 * The write can also fail by losing arbitration part way, which delivers a shorter
 * prefix of it. Read back the register file to tell these apart: unchanged is a
 * dropped overflow, a prefix shorter than the target accepts is an interrupted write,
 * and the full accepted length means overflow data was delivered.
 */
static void do_overflow(struct target_model *t)
{
	static struct target_model tmp;
	/* Bytes after the address the target accepts before it NACKs */
	const size_t accepted = CONFIG_APP_TARGET_BUFFER_SIZE - 1U;
	uint8_t reg = 0x00U;
	int rc;

	wbuf[0] = reg;
	fill_random(&wbuf[1], OVERFLOW_LEN - 1U);
	rc = i2c_write(i2c_dev, wbuf, OVERFLOW_LEN, t->addr);
	if (rc == 0) {
		LOG_ERR("overflow 0x%02x: write of %u bytes succeeded", t->addr,
			(unsigned int)OVERFLOW_LEN);
		st.verify_fail++;
		t->valid = false;
		return;
	}

	rc = i2c_write_read(i2c_dev, t->addr, &reg, 1U, rbuf, REG_FILE_SIZE);
	if (rc != 0) {
		op_error(OP_OVERFLOW, t, rc);
		return;
	}
	st.bytes_rd += REG_FILE_SIZE;

	if (memcmp(rbuf, t->regs, REG_FILE_SIZE) == 0) {
		t->ptr = 0U;
		return;
	}

	for (size_t n = 1U; n <= accepted; n++) {
		tmp = *t;
		model_write(&tmp, wbuf, n);
		if (memcmp(rbuf, tmp.regs, REG_FILE_SIZE) != 0) {
			continue;
		}
		if (n == accepted) {
			LOG_ERR("overflow 0x%02x: %zu bytes delivered, expected dropped", t->addr,
				n);
			st.verify_fail++;
			t->valid = false;
			return;
		}
		st.ovf_interrupted++;
		memcpy(t->regs, rbuf, REG_FILE_SIZE);
		t->ptr = 0U;
		return;
	}

	LOG_ERR("overflow 0x%02x: register file changed", t->addr);
	st.verify_fail++;
	t->valid = false;
}

/* Read beyond the data supplied, then check the target still answers correctly */
static void do_over_read(struct target_model *t)
{
	uint8_t reg = (uint8_t)rnd_range(0xF0U, 0xFFU);

	do_write_read(t, OP_OVER_READ, reg, (REG_FILE_SIZE - reg) + 8U);
	if (t->valid) {
		do_write_read(t, OP_OVER_READ, 0x00U, 4U);
	}
}

static void do_absent(void)
{
	uint8_t b = 0;

	if (i2c_write(i2c_dev, &b, 1U, CONFIG_APP_ABSENT_ADDR) == 0) {
		LOG_ERR("absent 0x%02x: write succeeded", CONFIG_APP_ABSENT_ADDR);
		st.verify_fail++;
	}
}

static void do_speed(void)
{
	uint32_t speed = (cur_speed == I2C_SPEED_STANDARD) ? I2C_SPEED_FAST : I2C_SPEED_STANDARD;
	int rc = i2c_configure(i2c_dev, I2C_MODE_CONTROLLER | I2C_SPEED_SET(speed));

	if (rc != 0) {
		LOG_ERR("i2c_configure speed %u error (%d)", speed, rc);
		st.errors[OP_SPEED]++;
		return;
	}

	cur_speed = speed;
}

static bool op_enabled(int op)
{
	if (op == OP_RPT_WRITE) {
		return IS_ENABLED(CONFIG_APP_RPT_START_WRITE);
	}
	if (op == OP_SPEED) {
		return IS_ENABLED(CONFIG_APP_SPEED_SWITCH);
	}

	return true;
}

static enum op pick_op(void)
{
	uint32_t total = 0;
	uint32_t r;

	for (int i = 0; i < OP_COUNT; i++) {
		total += op_weight[i];
	}

	while (true) {
		r = rnd() % total;
		for (int i = 0; i < OP_COUNT; i++) {
			if (r < op_weight[i]) {
				if (!op_enabled(i)) {
					break;
				}
				return (enum op)i;
			}
			r -= op_weight[i];
		}
	}
}

static void run_op(enum op op)
{
	struct target_model *t = &targets[rnd() & 1U];

	st.ops[op]++;

	if ((op != OP_ABSENT) && (op != OP_SPEED) && !t->valid) {
		/* After an error: let the other controller's transfer finish first */
		k_msleep(CONFIG_APP_RETRY_DELAY_MS);
		if (resync(t) != 0) {
			return;
		}
	}

	switch (op) {
	case OP_WRITE:
	case OP_WRITE_2MSG:
		do_write(t, op);
		break;
	case OP_WRITE_READ: {
		uint8_t reg = (uint8_t)rnd();

		do_write_read(t, op, reg, rnd_range(1U, MIN(64U, REG_FILE_SIZE - reg)));
		break;
	}
	case OP_READ:
		do_read(t);
		break;
	case OP_RPT_WRITE:
		do_rpt_write(t);
		break;
	case OP_FULL:
		do_full(t);
		break;
	case OP_OVERFLOW:
		do_overflow(t);
		break;
	case OP_OVER_READ:
		do_over_read(t);
		break;
	case OP_ABSENT:
		do_absent();
		break;
	default:
		do_speed();
		break;
	}
}

static void report(uint32_t count, int64_t start_ms)
{
	int64_t secs = MAX((k_uptime_get() - start_ms) / 1000, 1);
	uint32_t errors = 0;

	for (int i = 0; i < OP_COUNT; i++) {
		errors += st.errors[i];
	}

	LOG_INF("%u ops, %llu bytes written, %llu read (%llu B/s), %u verify failures, "
		"%u errors, %u resyncs (%u failed), %u interrupted overflows",
		count, st.bytes_wr, st.bytes_rd, (st.bytes_wr + st.bytes_rd) / (uint64_t)secs,
		st.verify_fail, errors, st.resyncs, st.resync_fail, st.ovf_interrupted);

	for (int i = 0; i < OP_COUNT; i++) {
		if (st.errors[i] != 0U) {
			LOG_INF("  %-16s %u ops, %u errors", op_names[i], st.ops[i], st.errors[i]);
		}
	}
}

int main(void)
{
	int64_t start_ms;
	int64_t next_report;
	uint32_t count = 0;

	LOG_INF("I2C host to target stress: targets 0x%02x 0x%02x, target buffer %d",
		CONFIG_APP_TARGET_ADDR_1, CONFIG_APP_TARGET_ADDR_2, CONFIG_APP_TARGET_BUFFER_SIZE);

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C controller not ready");
		return 0;
	}

	rng_state = (CONFIG_APP_SEED != 0) ? (uint32_t)CONFIG_APP_SEED : k_cycle_get_32();
	if (rng_state == 0U) {
		rng_state = 1U;
	}
	LOG_INF("seed %u", rng_state);

	if (i2c_configure(i2c_dev, I2C_MODE_CONTROLLER | I2C_SPEED_SET(cur_speed)) != 0) {
		LOG_ERR("i2c_configure error");
		return 0;
	}

	targets[0].addr = CONFIG_APP_TARGET_ADDR_1;
	targets[1].addr = CONFIG_APP_TARGET_ADDR_2;
	for (size_t i = 0; i < ARRAY_SIZE(targets); i++) {
		if (resync(&targets[i]) != 0) {
			LOG_ERR("target 0x%02x does not respond", targets[i].addr);
			return 0;
		}
	}

	start_ms = k_uptime_get();
	next_report = start_ms + CONFIG_APP_REPORT_INTERVAL_MS;

	while ((CONFIG_APP_ITERATIONS == 0) || (count < (uint32_t)CONFIG_APP_ITERATIONS)) {
		run_op(pick_op());
		count++;

		if (k_uptime_get() >= next_report) {
			report(count, start_ms);
			next_report += CONFIG_APP_REPORT_INTERVAL_MS;
		}
	}

	report(count, start_ms);
	LOG_INF("done: %s", (st.verify_fail == 0U) ? "PASS" : "FAIL");

	return 0;
}
