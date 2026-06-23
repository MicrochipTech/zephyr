/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * I2Cv3-NL target-mode test application.
 *
 * Hardware setup required on mec_assy6941:
 *   - I2C port 0 (smb_0) and I2C port 7 (smb_1) MUST be physically
 *     wire-tied off-board (J12 <-> J22). Port 0 is configured as the
 *     I2C target (slots 0x40 and 0x41); port 7 acts as the host
 *     controller driving transactions to those slots.
 *   - Port 7 also has an mb85rc256v FRAM at 0x50 used for controller-
 *     mode smoke tests at startup.
 *
 * Each target test runs in isolation: counters are reset, the host
 * transaction is issued, and the test waits with a bounded timeout
 * for the target's stop callback. Test results are tallied at the
 * end of main().
 */

#include <soc.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_xec_i2c.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/dt-bindings/i2c/mchp-xec-i2c.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define I2C_SMB_GET_DEV(nid) DEVICE_DT_GET(nid),

/* Target nodes */
#define NODE_I2C_TARG1 DT_NODELABEL(i2c_targ1)
#define NODE_I2C_TARG2 DT_NODELABEL(i2c_targ2)

/* FRAM target node */
#define NODE_FRAM    DT_NODELABEL(mb85rc256v_fram)

/* i2c_dt_spec.bus is the port controller node */
const struct i2c_dt_spec mb_fram_spec = I2C_DT_SPEC_GET(NODE_FRAM);
const struct i2c_dt_spec targ1_spec = I2C_DT_SPEC_GET(NODE_I2C_TARG1);
const struct i2c_dt_spec targ2_spec = I2C_DT_SPEC_GET(NODE_I2C_TARG2);

const struct device *i2c_smb0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c_smb_0));
const struct device *i2c_smb1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c_smb_1));

#define I2C_MAX_MSGS    8
#define I2C_TX_BUF_SIZE 256
#define I2C_RX_BUF_SIZE 256

/* Bounded wait for a target stop callback. Driver's own per-transfer
 * timeout is ~1s; 500ms here is well below that so a hang in the
 * driver still surfaces as a test failure with a clear message rather
 * than wedging the whole program.
 */
#define TARG_STOP_TIMEOUT K_MSEC(500)

struct app_i2c_target {
	uint8_t *buf;
	size_t bufsz;
	size_t idx;
	uint32_t wr_recv_cnt;
	uint32_t rd_req_cnt;
	uint32_t stop_cnt;
	uint32_t error_cnt;
	enum i2c_error_reason err;
};

static struct k_sem app_targ1_sem;
static struct k_sem app_targ2_sem;

static struct i2c_msg msgs[I2C_MAX_MSGS];
static uint8_t i2c_tx_buf[I2C_TX_BUF_SIZE];
static uint8_t i2c_rx_buf[I2C_RX_BUF_SIZE];

#define APP_TARG1_BUF_SIZE 256
#define APP_TARG2_BUF_SIZE 64

/* Target-buffer-size from DT for smb_0. Tests that intentionally
 * exceed the HW receive budget reference this constant.
 */
#define APP_TARG_HW_RX_SIZE DT_PROP(DT_NODELABEL(i2c_smb_0), target_buffer_size)
#define APP_TARG_HW_DATA_CAPACITY (APP_TARG_HW_RX_SIZE - 1U)

static uint8_t targ1_buf[APP_TARG1_BUF_SIZE];
static uint8_t targ2_buf[APP_TARG2_BUF_SIZE];

static void targ1_buf_wr_recv_cb(struct i2c_target_config *config, uint8_t *ptr, uint32_t len);
static int targ1_buf_rd_req_cb(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len);
static int targ1_stop_cb(struct i2c_target_config *config);
static void targ1_error_cb(struct i2c_target_config *config, enum i2c_error_reason error_code);

static void targ2_buf_wr_recv_cb(struct i2c_target_config *config, uint8_t *ptr, uint32_t len);
static int targ2_buf_rd_req_cb(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len);
static int targ2_stop_cb(struct i2c_target_config *config);
static void targ2_error_cb(struct i2c_target_config *config, enum i2c_error_reason error_code);

const struct i2c_target_callbacks targ1_callbacks = {
	.buf_write_received = targ1_buf_wr_recv_cb,
	.buf_read_requested = targ1_buf_rd_req_cb,
	.stop = targ1_stop_cb,
	.error = targ1_error_cb,
};

const struct i2c_target_callbacks targ2_callbacks = {
	.buf_write_received = targ2_buf_wr_recv_cb,
	.buf_read_requested = targ2_buf_rd_req_cb,
	.stop = targ2_stop_cb,
	.error = targ2_error_cb,
};

struct app_i2c_target targ1_app_data;
struct app_i2c_target targ2_app_data;
struct i2c_target_config targ1_cfg;
struct i2c_target_config targ2_cfg;

static int fram_test1(const struct i2c_dt_spec *dts);
static int fram_test2(const struct i2c_dt_spec *dts);

static int app_i2c_target_init(struct app_i2c_target *apptrg, uint8_t *buf, size_t bufsz);

/* -------------------------------------------------------------------------
 * Test infrastructure
 * -------------------------------------------------------------------------*/

struct test_case {
	const char *name;
	int (*fn)(void);
};

/* Per-test state reset. Counters accumulate by design across multiple
 * stop callbacks within a single test, but every test starts with a
 * blank slate. The driver's state-capture buffer is also cleared so
 * each test's postmortem trace stands alone.
 */
static void reset_target_state(struct app_i2c_target *t, struct k_sem *sem)
{
	t->idx = 0;
	t->wr_recv_cnt = 0;
	t->rd_req_cnt = 0;
	t->stop_cnt = 0;
	t->error_cnt = 0;
	t->err = 0;
	k_sem_reset(sem);
}

static void reset_all_targets(void)
{
	reset_target_state(&targ1_app_data, &app_targ1_sem);
	reset_target_state(&targ2_app_data, &app_targ2_sem);
	(void)mchp_xec_i2c_nl_clear_capture(i2c_smb0_dev);
	(void)mchp_xec_i2c_nl_clear_capture(i2c_smb1_dev);
}

/* Wait for a target's stop callback with the test timeout. Returns
 * 0 on success, -ETIMEDOUT if the callback never fired (almost
 * always a driver-state bug).
 */
static int wait_for_stop(struct k_sem *sem, const char *tag)
{
	int rc = k_sem_take(sem, TARG_STOP_TIMEOUT);

	if (rc != 0) {
		LOG_ERR("%s: timed out waiting for stop callback (%d)", tag, rc);
		return -ETIMEDOUT;
	}
	return 0;
}

/* Verify that the counters in `t` exactly match the four expected
 * values. Helps tests fail loudly with a clear message instead of
 * matching by accident.
 */
struct targ_expect {
	uint32_t wr_recv_cnt;
	uint32_t rd_req_cnt;
	uint32_t stop_cnt;
	uint32_t error_cnt;
};

static int verify_counters(const char *tag, struct app_i2c_target *t,
			   const struct targ_expect *e)
{
	int rc = 0;

	if (t->wr_recv_cnt != e->wr_recv_cnt) {
		LOG_ERR("%s: wr_recv_cnt = %u, expected %u", tag, t->wr_recv_cnt,
			e->wr_recv_cnt);
		rc = -EINVAL;
	}
	if (t->rd_req_cnt != e->rd_req_cnt) {
		LOG_ERR("%s: rd_req_cnt = %u, expected %u", tag, t->rd_req_cnt,
			e->rd_req_cnt);
		rc = -EINVAL;
	}
	if (t->stop_cnt != e->stop_cnt) {
		LOG_ERR("%s: stop_cnt = %u, expected %u", tag, t->stop_cnt,
			e->stop_cnt);
		rc = -EINVAL;
	}
	if (t->error_cnt != e->error_cnt) {
		LOG_ERR("%s: error_cnt = %u (reason %u), expected %u", tag,
			t->error_cnt, t->err, e->error_cnt);
		rc = -EINVAL;
	}
	return rc;
}

static int verify_buf_eq(const char *tag, const uint8_t *got, const uint8_t *want,
			 size_t len)
{
	if (memcmp(got, want, len) != 0) {
		LOG_ERR("%s: received data does not match expected (len=%u)", tag,
			(unsigned int)len);
		LOG_HEXDUMP_ERR(got, len, "got");
		LOG_HEXDUMP_ERR(want, len, "want");
		return -EINVAL;
	}
	return 0;
}

static int run_all_tests(const struct test_case *cases, size_t count)
{
	int pass = 0;
	int fail = 0;

	for (size_t i = 0; i < count; i++) {
		LOG_INF("---- TEST %u/%u: %s ----", (unsigned int)(i + 1U),
			(unsigned int)count, cases[i].name);
		int rc = cases[i].fn();

		if (rc == 0) {
			LOG_INF("PASS: %s", cases[i].name);
			pass++;
		} else {
			LOG_ERR("FAIL: %s (rc=%d)", cases[i].name, rc);
			fail++;
		}
	}

	LOG_INF("==== TESTS: %d pass, %d fail ====", pass, fail);
	return fail == 0 ? 0 : -1;
}

/* -------------------------------------------------------------------------
 * Test cases
 * -------------------------------------------------------------------------*/

/* Baseline: host writes a small payload to targ1; expect one full
 * write-receive callback delivering the bytes, one stop, no errors.
 */
static int test_host_write_short_to_targ1(void)
{
	const uint8_t want[] = {0x01U, 0x02U, 0x03U, 0x04U};
	const struct targ_expect expect = {
		.wr_recv_cnt = 1U, .rd_req_cnt = 0U, .stop_cnt = 1U, .error_cnt = 0U,
	};
	int rc;

	reset_all_targets();

	rc = i2c_write(mb_fram_spec.bus, want, sizeof(want), targ1_spec.addr);
	if (rc != 0) {
		LOG_ERR("i2c_write returned %d", rc);
		return rc;
	}
	rc = wait_for_stop(&app_targ1_sem, "targ1");
	if (rc != 0) {
		return rc;
	}
	rc = verify_counters("targ1", &targ1_app_data, &expect);
	if (rc != 0) {
		return rc;
	}
	return verify_buf_eq("targ1", targ1_buf, want, sizeof(want));
}

/* Baseline: host reads a small payload from targ2; expect one
 * read-requested callback (the application supplies bytes), one stop
 * callback, no write-received fired (R-bit gate), no errors. The
 * host buffer should now hold the bytes the target's read callback
 * pointed at.
 */
static int test_host_read_short_from_targ2(void)
{
	uint8_t hostbuf[4] = {0U};
	uint8_t want[4];
	const struct targ_expect expect = {
		.wr_recv_cnt = 0U, .rd_req_cnt = 1U, .stop_cnt = 1U, .error_cnt = 0U,
	};
	int rc;

	reset_all_targets();

	/* targ2_buf is sourced by buf_read_requested starting at
	 * targ2_app_data.idx (== 0 after reset). Capture what should
	 * come back so the comparison is deterministic.
	 */
	memcpy(want, &targ2_buf[0], sizeof(want));

	rc = i2c_read(mb_fram_spec.bus, hostbuf, sizeof(hostbuf), targ2_spec.addr);
	if (rc != 0) {
		LOG_ERR("i2c_read returned %d", rc);
		return rc;
	}
	rc = wait_for_stop(&app_targ2_sem, "targ2");
	if (rc != 0) {
		return rc;
	}
	rc = verify_counters("targ2", &targ2_app_data, &expect);
	if (rc != 0) {
		return rc;
	}
	return verify_buf_eq("targ2", hostbuf, want, sizeof(hostbuf));
}

/* Tier 1 #1: buffer-fill. Host writes APP_TARG_HW_DATA_CAPACITY + 1
 * data bytes -- one more than the HW receive budget will accept after
 * the address byte. Expect the target to NAK the over-the-line byte
 * (host i2c_write returns an error), but our application should still
 * receive APP_TARG_HW_DATA_CAPACITY bytes via buf_write_received and
 * see exactly one stop callback (or one error+stop -- both shapes
 * accepted; the bus completed cleanly either way).
 *
 * Catches: TCMD.RCL exhaustion -> TDONE-with-RUN=0 path,
 * write-received-on-fill-then-stop dispatch, lack of stuck state
 * after a buffer-fill transaction.
 */
static int test_host_write_buffer_fill_targ1(void)
{
	uint32_t overflow_len = APP_TARG_HW_DATA_CAPACITY + 1U;
	int rc;

	if (overflow_len > sizeof(i2c_tx_buf)) {
		LOG_WRN("test buffer too small for this target-buffer-size; "
			"increase I2C_TX_BUF_SIZE");
		return -ENOSPC;
	}

	reset_all_targets();

	for (uint32_t i = 0; i < overflow_len; i++) {
		i2c_tx_buf[i] = (uint8_t)(i & 0xFFU);
	}

	rc = i2c_write(mb_fram_spec.bus, i2c_tx_buf, overflow_len, targ1_spec.addr);
	/* The host's write may complete (-EIO/-ENXIO on NAK) or report
	 * success depending on how the host driver reports a mid-byte
	 * NAK. Either outcome is acceptable here -- what we care about
	 * is the target-side observation.
	 */
	if (rc != 0) {
		LOG_INF("i2c_write returned %d (expected; host saw target NAK)", rc);
	} else {
		LOG_INF("i2c_write returned 0 (host driver did not surface the NAK)");
	}

	rc = wait_for_stop(&app_targ1_sem, "targ1");
	if (rc != 0) {
		return rc;
	}

	/* Whatever bytes did make it through should be at the start of
	 * targ1_buf; verify the first APP_TARG_HW_DATA_CAPACITY of them.
	 */
	rc = verify_buf_eq("targ1", targ1_buf, i2c_tx_buf, APP_TARG_HW_DATA_CAPACITY);
	if (rc != 0) {
		return rc;
	}

	/* Expect at least one write-received and one stop. error_cnt is
	 * not strictly required to be 0 -- some HW paths surface a NAK
	 * via the error callback, others via the host side only.
	 */
	if (targ1_app_data.stop_cnt != 1U || targ1_app_data.wr_recv_cnt < 1U ||
	    targ1_app_data.rd_req_cnt != 0U) {
		LOG_ERR("targ1 counters: wr_recv=%u rd_req=%u stop=%u error=%u "
			"(expected stop==1, wr_recv>=1, rd_req==0)",
			targ1_app_data.wr_recv_cnt, targ1_app_data.rd_req_cnt,
			targ1_app_data.stop_cnt, targ1_app_data.error_cnt);
		return -EINVAL;
	}
	return 0;
}

/* Tier 1 #7: regression test for the R-bit gate. Host reads from
 * targ1; the only callback fired must be buf_read_requested (plus
 * stop). buf_write_received MUST NOT fire. If it does, the driver
 * has regressed to the pre-fix behavior of misclassifying the
 * post-read STOP as a write completion.
 */
static int test_host_read_does_not_fire_write_cb(void)
{
	uint8_t hostbuf[8] = {0U};
	const struct targ_expect expect = {
		.wr_recv_cnt = 0U, .rd_req_cnt = 1U, .stop_cnt = 1U, .error_cnt = 0U,
	};
	int rc;

	reset_all_targets();

	rc = i2c_read(mb_fram_spec.bus, hostbuf, sizeof(hostbuf), targ1_spec.addr);
	if (rc != 0) {
		LOG_ERR("i2c_read returned %d", rc);
		return rc;
	}
	rc = wait_for_stop(&app_targ1_sem, "targ1");
	if (rc != 0) {
		return rc;
	}
	return verify_counters("targ1", &targ1_app_data, &expect);
}

/* Tier 2 #9: symmetry. Today's baseline tests only write to targ1
 * and only read from targ2. Mirror that: write to targ2, read from
 * targ1. Each target must record its own transaction and ONLY its
 * own transaction. Catches slot-lookup regressions where the wrong
 * callback set fires.
 */
static int test_symmetric_targ2_write_targ1_read(void)
{
	const uint8_t write_payload[] = {0xA0U, 0xB1U, 0xC2U, 0xD3U};
	uint8_t read_payload[4] = {0U};
	uint8_t expect_read[4];
	const struct targ_expect expect_t1 = {
		.wr_recv_cnt = 0U, .rd_req_cnt = 1U, .stop_cnt = 1U, .error_cnt = 0U,
	};
	const struct targ_expect expect_t2 = {
		.wr_recv_cnt = 1U, .rd_req_cnt = 0U, .stop_cnt = 1U, .error_cnt = 0U,
	};
	int rc;

	reset_all_targets();

	/* Host -> targ2 write. */
	rc = i2c_write(mb_fram_spec.bus, write_payload, sizeof(write_payload),
		       targ2_spec.addr);
	if (rc != 0) {
		LOG_ERR("i2c_write to targ2 returned %d", rc);
		return rc;
	}
	rc = wait_for_stop(&app_targ2_sem, "targ2-write");
	if (rc != 0) {
		return rc;
	}
	rc = verify_counters("targ2-after-write", &targ2_app_data, &expect_t2);
	if (rc != 0) {
		return rc;
	}
	/* targ1 must not have observed anything yet. */
	const struct targ_expect zero = {0};

	rc = verify_counters("targ1-after-targ2-write", &targ1_app_data, &zero);
	if (rc != 0) {
		return rc;
	}
	rc = verify_buf_eq("targ2", targ2_buf, write_payload, sizeof(write_payload));
	if (rc != 0) {
		return rc;
	}

	/* Host -> targ1 read. */
	memcpy(expect_read, &targ1_buf[0], sizeof(expect_read));

	rc = i2c_read(mb_fram_spec.bus, read_payload, sizeof(read_payload),
		      targ1_spec.addr);
	if (rc != 0) {
		LOG_ERR("i2c_read from targ1 returned %d", rc);
		return rc;
	}
	rc = wait_for_stop(&app_targ1_sem, "targ1-read");
	if (rc != 0) {
		return rc;
	}
	rc = verify_counters("targ1-after-read", &targ1_app_data, &expect_t1);
	if (rc != 0) {
		return rc;
	}
	/* targ2 still must show only the one write. */
	rc = verify_counters("targ2-after-targ1-read", &targ2_app_data, &expect_t2);
	if (rc != 0) {
		return rc;
	}
	return verify_buf_eq("targ1-read-back", read_payload, expect_read,
			     sizeof(read_payload));
}

static const struct test_case target_tests[] = {
	{"host write 4B to targ1",          test_host_write_short_to_targ1},
	{"host read 4B from targ2",         test_host_read_short_from_targ2},
	{"buffer fill targ1",               test_host_write_buffer_fill_targ1},
	{"host read targ1: no wr_recv_cb",  test_host_read_does_not_fire_write_cb},
	{"symmetric: write targ2, read targ1", test_symmetric_targ2_write_targ1_read},
};

/* -------------------------------------------------------------------------
 * Entry
 * -------------------------------------------------------------------------*/

int main(void)
{
	int rc = 0;

	memset((void *)msgs, 0, sizeof(msgs));
	memset(i2c_tx_buf, 0x55, I2C_TX_BUF_SIZE);
	memset(i2c_rx_buf, 0xAA, I2C_RX_BUF_SIZE);

	k_sem_init(&app_targ1_sem, 0, 1);
	k_sem_init(&app_targ2_sem, 0, 1);

	if (!device_is_ready(mb_fram_spec.bus)) {
		LOG_ERR("FRAM I2C port driver not ready!");
		goto app_done;
	}

	rc = fram_test1(&mb_fram_spec);
	if (rc == 0) {
		LOG_INF("FRAM test 1: PASS");
	} else {
		LOG_ERR("FRAM test 1 error (%d): FAIL", rc);
	}

	rc = fram_test2(&mb_fram_spec);
	if (rc == 0) {
		LOG_INF("FRAM test 2: PASS");
	} else {
		LOG_ERR("FRAM test 2 error (%d): FAIL", rc);
	}

	if (!device_is_ready(targ1_spec.bus)) {
		LOG_ERR("Target 1 I2C port driver not ready!");
		goto app_done;
	}

	if (!device_is_ready(targ2_spec.bus)) {
		LOG_ERR("Target 2 I2C port driver not ready!");
		goto app_done;
	}

	memset(targ1_buf, 0x55U, APP_TARG1_BUF_SIZE);
	memset(targ2_buf, 0xAAU, APP_TARG2_BUF_SIZE);

	rc = app_i2c_target_init(&targ1_app_data, targ1_buf, APP_TARG1_BUF_SIZE);
	if (rc != 0) {
		LOG_ERR("Init target 1 app structure error (%d)", rc);
		goto app_done;
	}

	rc = app_i2c_target_init(&targ2_app_data, targ2_buf, APP_TARG2_BUF_SIZE);
	if (rc != 0) {
		LOG_ERR("Init target 2 app structure error (%d)", rc);
		goto app_done;
	}

	targ1_cfg.flags = 0;
	targ1_cfg.address = targ1_spec.addr;
	targ1_cfg.callbacks = &targ1_callbacks;

	targ2_cfg.flags = 0;
	targ2_cfg.address = targ2_spec.addr;
	targ2_cfg.callbacks = &targ2_callbacks;

	LOG_INF("Register target 1 (0x%02x) on %s", targ1_spec.addr,
		targ1_spec.bus->name);
	rc = i2c_target_register(targ1_spec.bus, &targ1_cfg);
	if (rc != 0) {
		LOG_ERR("i2c_target_register targ1 failed (%d)", rc);
		goto app_done;
	}

	LOG_INF("Register target 2 (0x%02x) on %s", targ2_spec.addr,
		targ2_spec.bus->name);
	rc = i2c_target_register(targ2_spec.bus, &targ2_cfg);
	if (rc != 0) {
		LOG_ERR("i2c_target_register targ2 failed (%d)", rc);
		goto app_done;
	}

	rc = run_all_tests(target_tests, ARRAY_SIZE(target_tests));

app_done:
	LOG_INF("Program End (rc=%d)", rc);
	log_flush();

	return 0;
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

static int app_i2c_target_init(struct app_i2c_target *apptrg, uint8_t *buf, size_t bufsz)
{
	if ((apptrg == NULL) || (buf == NULL) || (bufsz == 0)) {
		return -EINVAL;
	}

	apptrg->buf = buf;
	apptrg->bufsz = bufsz;
	apptrg->idx = 0;
	apptrg->wr_recv_cnt = 0;
	apptrg->rd_req_cnt = 0;
	apptrg->stop_cnt = 0;
	apptrg->error_cnt = 0;
	apptrg->err = 0;

	return 0;
}

static void targ1_buf_wr_recv_cb(struct i2c_target_config *config, uint8_t *ptr, uint32_t len)
{
	uint32_t max_idx = targ1_app_data.idx + len;
	uint8_t *p = ptr;

	targ1_app_data.wr_recv_cnt++;

	if (p == NULL) {
		return;
	}

	if (max_idx > targ1_app_data.bufsz) {
		max_idx = targ1_app_data.bufsz;
	}
	for (uint32_t i = targ1_app_data.idx; i < max_idx; i++) {
		targ1_app_data.buf[i] = *p++;
	}
}

static int targ1_buf_rd_req_cb(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len)
{
	targ1_app_data.rd_req_cnt++;

	if ((ptr == NULL) || (len == NULL)) {
		return -EINVAL;
	}

	*ptr = &targ1_app_data.buf[targ1_app_data.idx];
	*len = targ1_app_data.bufsz - targ1_app_data.idx;

	return 0;
}

static int targ1_stop_cb(struct i2c_target_config *config)
{
	targ1_app_data.stop_cnt++;
	targ1_app_data.idx = 0;

	k_sem_give(&app_targ1_sem);

	return 0;
}

static void targ1_error_cb(struct i2c_target_config *config, enum i2c_error_reason error_code)
{
	targ1_app_data.error_cnt++;
	targ1_app_data.err = error_code;
}

static void targ2_buf_wr_recv_cb(struct i2c_target_config *config, uint8_t *ptr, uint32_t len)
{
	uint32_t max_idx = targ2_app_data.idx + len;
	uint8_t *p = ptr;

	targ2_app_data.wr_recv_cnt++;

	if (p == NULL) {
		return;
	}

	if (max_idx > targ2_app_data.bufsz) {
		max_idx = targ2_app_data.bufsz;
	}
	for (uint32_t i = targ2_app_data.idx; i < max_idx; i++) {
		targ2_app_data.buf[i] = *p++;
	}
}

static int targ2_buf_rd_req_cb(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len)
{
	targ2_app_data.rd_req_cnt++;

	if ((ptr == NULL) || (len == NULL)) {
		return -EINVAL;
	}

	*ptr = &targ2_app_data.buf[targ2_app_data.idx];
	*len = targ2_app_data.bufsz - targ2_app_data.idx;

	return 0;
}

static int targ2_stop_cb(struct i2c_target_config *config)
{
	targ2_app_data.stop_cnt++;
	targ2_app_data.idx = 0;

	k_sem_give(&app_targ2_sem);

	return 0;
}

static void targ2_error_cb(struct i2c_target_config *config, enum i2c_error_reason error_code)
{
	targ2_app_data.error_cnt++;
	targ2_app_data.err = error_code;
}
