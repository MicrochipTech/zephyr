/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i3c_target, CONFIG_I3C_TEST_TARGET_LOG_LEVEL);

/* --------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------- */
#define I3C_TEST_BUF_SIZE         1024U
#define I3C_TEST_MDB_TIR          0xAEU
#define I3C_TEST_HJ_TIMEOUT_MS    5000U

/* Event bits — must match host.c and loopback/main.c */
#define EVT_IBI_TIR_MDB_AE   BIT(0)
#define EVT_IBI_ANY          BIT(1)
#define EVT_IBI_HOTJOIN      BIT(2)
#define EVT_DATA_RECEIVED    BIT(3)
#define EVT_TARGET_READY     BIT(4)


/*
 * 48-bit MIPI PID packed into uint64_t.
 * The driver [4] unpacks it as:
 *   pid >> 16        -> SLV_MIPI_ID_VALUE (0x70) - MFG ID
 *   pid & 0xFFFFFFFF -> SLV_PID_VALUE     (0x74) - Part/Inst/DCR
 *
 * Values match i3c1-as-tgt-pid = <0xB012 0x3456789A>
 */
#define TGT_PID_HI          0xB012ULL       /* upper 16 bits of 48-bit PID */
#define TGT_PID_LO          0x3456789AULL   /* lower 32 bits of 48-bit PID */
#define TGT_PID             ((TGT_PID_HI << 32) | TGT_PID_LO)
#define TGT_STATIC_ADDR     0x08
#define TGT_MAX_READ_LEN    128
#define TGT_MAX_WRITE_LEN   128

/* --------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------- */
__aligned(4) static uint8_t tgt_rx_buf[I3C_TEST_BUF_SIZE];
static uint32_t              tgt_num_bytes_rxd;
static struct k_event       *target_events;   /* set from p2 */

static uint8_t         mctp_mdb = I3C_TEST_MDB_TIR;
static struct i3c_ibi  tir_request = {
	.ibi_type   = I3C_IBI_TARGET_INTR,
	.payload     = &mctp_mdb,
	.payload_len = sizeof(mctp_mdb),
};

static struct i3c_ibi  hj_request = {
	.ibi_type   = I3C_IBI_HOTJOIN,
	.payload     = NULL,
	.payload_len = 0,
};

/* --------------------------------------------------------------------
 * PASS/FAIL reporting helpers
 * -------------------------------------------------------------------- */
#define TEST_REPORT_INIT()          \
    bool _group_pass = true;        \
    const char *_fail_reason = "(unknown)"

#define TEST_SUB_PASS(name)         \
    LOG_DBG("[step] %s PASS", (name))

#define TEST_SUB_FAIL(name, reason_str)         \
    do {                                        \
        LOG_DBG("[step] %s FAIL: %s",           \
                (name), (reason_str));          \
        if (_group_pass) {                      \
            _fail_reason = (reason_str);        \
        }                                       \
        _group_pass = false;                    \
    } while (0)

#define TEST_GROUP_RESULT(name)                         \
    do {                                                \
        if (_group_pass) {                              \
            LOG_INF("[PASS] %s", (name));               \
        } else {                                        \
            LOG_ERR("[FAIL] %s: %s",                    \
                    (name), _fail_reason);              \
        }                                               \
    } while (0)

/* --------------------------------------------------------------------
 * Target callback declarations
 * -------------------------------------------------------------------- */
static int  tgt_write_requested_cb(struct i3c_target_config *cfg);
static int  tgt_write_received_cb(struct i3c_target_config *cfg, uint8_t val);
static int  tgt_read_requested_cb(struct i3c_target_config *cfg, uint8_t *val);
static int  tgt_read_processed_cb(struct i3c_target_config *cfg, uint8_t *val);
static int  tgt_stop_cb(struct i3c_target_config *cfg);
#ifdef CONFIG_I3C_TARGET_BUFFER_MODE
static void tgt_buf_write_received_cb(struct i3c_target_config *cfg,
				      uint8_t *ptr, uint32_t len);
static int  tgt_buf_read_requested_cb(struct i3c_target_config *cfg,
				      uint8_t **ptr, uint32_t *len,
				      uint8_t *hdr_mode);
#endif

static struct i3c_target_callbacks tgt_cbs = {
	.write_requested_cb = tgt_write_requested_cb,
	.write_received_cb  = tgt_write_received_cb,
	.read_requested_cb  = tgt_read_requested_cb,
	.read_processed_cb  = tgt_read_processed_cb,
#ifdef CONFIG_I3C_TARGET_BUFFER_MODE
	.buf_write_received_cb = tgt_buf_write_received_cb,
	.buf_read_requested_cb = tgt_buf_read_requested_cb,
#endif
	.stop_cb            = tgt_stop_cb,
};

static struct i3c_target_config tgt_cfg;

/* --------------------------------------------------------------------
 * Callback implementations
 * -------------------------------------------------------------------- */
static int tgt_write_requested_cb(struct i3c_target_config *cfg)
{
	ARG_UNUSED(cfg);
	return 0;
}

static int tgt_write_received_cb(struct i3c_target_config *cfg, uint8_t val)
{
	ARG_UNUSED(cfg);
	if (tgt_num_bytes_rxd < I3C_TEST_BUF_SIZE) {
		tgt_rx_buf[tgt_num_bytes_rxd] = val;
		tgt_num_bytes_rxd++;
	}
	return 0;
}

static int tgt_read_requested_cb(struct i3c_target_config *cfg, uint8_t *val)
{
	ARG_UNUSED(cfg);
	ARG_UNUSED(val);
	return 0;
}

static int tgt_read_processed_cb(struct i3c_target_config *cfg, uint8_t *val)
{
	ARG_UNUSED(cfg);
	ARG_UNUSED(val);
	return 0;
}

static int tgt_stop_cb(struct i3c_target_config *cfg)
{
	if (tgt_num_bytes_rxd > 0) {
		k_event_post(target_events, EVT_DATA_RECEIVED);
	}
	ARG_UNUSED(cfg);
	return 0;
}

#ifdef CONFIG_I3C_TARGET_BUFFER_MODE
static void tgt_buf_write_received_cb(struct i3c_target_config *cfg,
				      uint8_t *ptr, uint32_t len)
{
	ARG_UNUSED(cfg);
	if (len <= I3C_TEST_BUF_SIZE) {
		memset(tgt_rx_buf, 0, sizeof(tgt_rx_buf));
		memcpy(tgt_rx_buf, ptr, len);
		tgt_num_bytes_rxd = len;
		k_event_post(target_events, EVT_DATA_RECEIVED);
	}
}

static int tgt_buf_read_requested_cb(struct i3c_target_config *cfg,
				     uint8_t **ptr, uint32_t *len,
				     uint8_t *hdr_mode)
{
	ARG_UNUSED(cfg);
	ARG_UNUSED(hdr_mode);
	*ptr = tgt_rx_buf;
	*len = tgt_num_bytes_rxd;
	return 0;
}
#endif /* CONFIG_I3C_TARGET_BUFFER_MODE */

/* --------------------------------------------------------------------
 * Helper: register target and signal readiness
 * Returns 0 on success, non-zero on failure.
 * -------------------------------------------------------------------- */
static int target_register_and_signal(const struct device *dev)
{
	tgt_cfg.callbacks = &tgt_cbs;

	int ret = i3c_target_register(dev, &tgt_cfg);

	if (ret != 0) {
		LOG_ERR("i3c_target_register failed: %d", ret);
		return ret;
	}

	LOG_INF("Target[%s] registered", dev->name);
	/* Signal loopback host (no-op in standalone target app). */
	k_event_post(target_events, EVT_TARGET_READY);
	return 0;
}

/* ====================================================================
 * TC0: data_echo
 * Standalone: Introspect host writes data; this target echoes it back.
 * Loopback:   Not used (CONFIG_I3C_TARGET_ECHO_MODE=y path is used).
 * ==================================================================== */
static void run_tc0_data_echo(const struct device *dev)
{
	TEST_REPORT_INIT();
	int ret;

	/* Step 1: register */
	ret = target_register_and_signal(dev);
	if (ret != 0) {
		TEST_SUB_FAIL("register", "i3c_target_register failed");
		TEST_GROUP_RESULT("data_echo");
		return;
	}
	TEST_SUB_PASS("register");

	/* Step 2: rx_1B — wait for host to send exactly 1 byte; just log it */
	tgt_num_bytes_rxd = 0;
	(void)k_event_wait(target_events, EVT_DATA_RECEIVED, true, K_FOREVER);
	LOG_INF("rx_1B: received 0x%02x", tgt_rx_buf[0]);
	TEST_SUB_PASS("rx_1B");

	/* Step 3: rx_128B_echo — echo 128+ bytes back and raise TIR IBI */
	tgt_num_bytes_rxd = 0;
	(void)k_event_wait(target_events, EVT_DATA_RECEIVED, true, K_FOREVER);
	if (tgt_num_bytes_rxd < 128U) {
		TEST_SUB_FAIL("rx_128B_echo", "expected >= 128 bytes");
	} else {

		LOG_INF("rx_12B: received bytes %d", tgt_num_bytes_rxd);

		int written = i3c_target_tx_write(dev, tgt_rx_buf,
						  (uint16_t)tgt_num_bytes_rxd, 0);
		if (written <= 0) {
			TEST_SUB_FAIL("rx_128B_echo", "tx_write failed");
		} else {
			ret = i3c_ibi_raise(dev, &tir_request);
			if (ret != 0) {
				TEST_SUB_FAIL("rx_128B_echo", "ibi_raise failed");
			} else {
				TEST_SUB_PASS("rx_128B_echo");
			}
		}
	}

#if 0	
	/* Step 4: rx_1024B_echo — echo up to 1024 bytes back */
	tgt_num_bytes_rxd = 0;
	(void)k_event_wait(target_events, EVT_DATA_RECEIVED, true, K_FOREVER);
	int written = i3c_target_tx_write(dev, tgt_rx_buf,
					  (uint16_t)tgt_num_bytes_rxd, 0);
	if (written <= 0) {
		TEST_SUB_FAIL("rx_1024B_echo", "tx_write failed");
	} else {
		TEST_SUB_PASS("rx_1024B_echo");
	}
#endif		

	TEST_GROUP_RESULT("data_echo");
}

/* ====================================================================
 * TC1: ibi
 * Standalone: Introspect host writes data twice; target echoes and
 * raises TIR IBI each time.
 * ==================================================================== */
static void run_tc1_ibi(const struct device *dev)
{
	TEST_REPORT_INIT();
	int ret;

	/* Step 1: register */
	ret = target_register_and_signal(dev);
	if (ret != 0) {
		TEST_SUB_FAIL("register", "i3c_target_register failed");
		TEST_GROUP_RESULT("ibi");
		return;
	}
	TEST_SUB_PASS("register");

	/* Steps 2a/2b: tir_raise — two iterations */
	for (int iter = 1; iter <= 2; iter++) {
		char step_name[32];

		snprintf(step_name, sizeof(step_name), "tir_raise_%d", iter);

		tgt_num_bytes_rxd = 0;
		(void)k_event_wait(target_events, EVT_DATA_RECEIVED, true, K_FOREVER);

		/* Echo data back */
		(void)i3c_target_tx_write(dev, tgt_rx_buf,
					  (uint16_t)tgt_num_bytes_rxd, 0);

		ret = i3c_ibi_raise(dev, &tir_request);
		if (ret != 0) {
			LOG_DBG("[step] %s FAIL: ibi_raise failed (%d)",
				step_name, ret);
			if (_group_pass) {
				_fail_reason = "ibi_raise failed";
			}
			_group_pass = false;
		} else {
			LOG_DBG("[step] %s PASS", step_name);
		}
	}

	TEST_GROUP_RESULT("ibi");
}

/* ====================================================================
 * TC2: hot_join
 * Standalone: Target raises a hot-join IBI without prior registration;
 * waits for the host to assign a dynamic address.
 * ==================================================================== */
static void run_tc2_hot_join(const struct device *dev)
{
	TEST_REPORT_INIT();

	/* Step 1: hj_request — raise HJ IBI (no i3c_target_register first) */
	int ret = i3c_ibi_raise(dev, &hj_request);

	if (ret != 0) {
		TEST_SUB_FAIL("hj_request", "i3c_ibi_raise(HJ) failed");
		TEST_GROUP_RESULT("hot_join");
		return;
	}
	TEST_SUB_PASS("hj_request");

	/*
	 * Step 2: hj_addr_assigned — wait for the host to assign a DA.
	 * The driver updates tgt_cfg.address when the HJ flow completes.
	 */
	k_sleep(K_MSEC(I3C_TEST_HJ_TIMEOUT_MS));

	if (tgt_cfg.address != 0U) {
		LOG_INF("HJ: dynamic_addr=0x%02x", tgt_cfg.address);
		TEST_SUB_PASS("hj_addr_assigned");
	} else {
		TEST_SUB_FAIL("hj_addr_assigned", "no DA assigned within timeout");
	}

	TEST_GROUP_RESULT("hot_join");
}

/* ====================================================================
 * setup_i3c_target — called from main() before thread start
 * ==================================================================== */

int setup_i3c_target(const struct device *dev)
{
    if (!device_is_ready(dev)) {
        LOG_ERR("Target[%s] not ready", dev->name);
        return -ENODEV;
    }

    struct i3c_config_target cfg = {
        .pid           = TGT_PID,
        .pid_random    = false,
        .static_addr   = TGT_STATIC_ADDR,
        .max_read_len  = TGT_MAX_READ_LEN,
        .max_write_len = TGT_MAX_WRITE_LEN,
    };
    int ret = i3c_configure(dev, I3C_CONFIG_TARGET, &cfg);
    if (ret != 0) {
        LOG_ERR("i3c_configure failed: %d", ret);
        return ret;
    }
    LOG_INF("Target[%s] configured: PID=0x%012llx SA=0x%02x",
            dev->name, cfg.pid, cfg.static_addr);

    ret = i3c_target_register(dev, &tgt_cfg);
    if (ret != 0) {
        LOG_ERR("i3c_target_register failed: %d", ret);
        return ret;
    }

    k_event_post(target_events, EVT_TARGET_READY);
    return 0;
}

/* ====================================================================
 * i3c_target_xfer_task — thread entry point
 * ==================================================================== */
void i3c_target_xfer_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p3);

	const struct device *dev = (const struct device *)p1;

	target_events = (struct k_event *)p2;

#if CONFIG_I3C_TARGET_ECHO_MODE
	/*
	 * Loopback echo-server mode: register, signal the host thread that
	 * the target is ready, then echo every received write back to the
	 * host and raise a TIR IBI (errors are logged but do not halt the
	 * loop — the host controls when IBI is enabled).
	 */
	tgt_cfg.callbacks = &tgt_cbs;
	if (i3c_target_register(dev, &tgt_cfg) != 0) {
		LOG_ERR("[FAIL] loopback target register failed");
		k_sleep(K_FOREVER);
		return;
	}
	LOG_INF("Target[%s] registered (echo mode)", dev->name);
	k_event_post(target_events, EVT_TARGET_READY);

	while (true) {
		uint32_t evts = k_event_wait(target_events, EVT_DATA_RECEIVED,
					     true, K_FOREVER);

		if (evts & EVT_DATA_RECEIVED) {
			if (tgt_num_bytes_rxd > 0U) {
				int written = i3c_target_tx_write(
					dev, tgt_rx_buf,
					(uint16_t)tgt_num_bytes_rxd, 0);
				if (written <= 0) {
					LOG_WRN("echo: tx_write error %d",
						written);
				}
				int r = i3c_ibi_raise(dev, &tir_request);

				if (r != 0) {
					LOG_DBG("echo: ibi_raise: %d (IBI may not be enabled)", r);
				}
				tgt_num_bytes_rxd = 0U;
			}
		}
	}
#else
	/* Structured test-case mode (standalone target app) */
	switch (CONFIG_I3C_TEST_CASE) {
	case 0:
		run_tc0_data_echo(dev);
		break;
	case 1:
		run_tc1_ibi(dev);
		break;
	case 2:
		run_tc2_hot_join(dev);
		break;
	default:
		LOG_ERR("[FAIL] unknown test case %d", CONFIG_I3C_TEST_CASE);
		break;
	}
#endif /* CONFIG_I3C_TARGET_ECHO_MODE */

	k_sleep(K_FOREVER);
}
