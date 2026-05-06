/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/ccc.h>
#include <zephyr/logging/log.h>
#include "crc8.h"

LOG_MODULE_REGISTER(i3c_host, CONFIG_I3C_TEST_HOST_LOG_LEVEL);

/* --------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------- */
#define I3C_TEST_BUF_SIZE         1024U
#define I3C_TEST_MDB_TIR          0xAEU
#define I3C_TEST_TIMEOUT_MS       10000U
#define I3C_TEST_IBI_POLL_MS      1000U
#define I3C_TEST_HJ_TIMEOUT_MS    5000U
#define I3C_TEST_ECHO_DELAY_MS      10U
#define I3C_SCL_TYP_HZ            12500000U   /* I3C typical 12.5 MHz */

/* Event bits — must match target.c and loopback/main.c */
#define EVT_IBI_TIR_MDB_AE   BIT(0)   /* TIR IBI with MDB=0xAE */
#define EVT_IBI_ANY          BIT(1)   /* any IBI received */
#define EVT_IBI_HOTJOIN      BIT(2)   /* hot-join IBI received */
#define EVT_DATA_RECEIVED    BIT(3)   /* target data received (target side) */
#define EVT_TARGET_READY     BIT(4)   /* target registered (loopback sync) */

/* --------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------- */
__aligned(4) static uint8_t host_tx_buf[I3C_TEST_BUF_SIZE] = {
	0x58, 0x12, 0x59, 0x84, 0x5F, 0x7B, 0xC4, 0xBC, 0xD0, 0x8D, 0xF3, 0x46, 0x79, 0x18, 0x75,
	0xEC, 0xB6, 0x44, 0x43, 0x1C, 0xF4, 0xF7, 0xDA, 0xC7, 0xE8, 0x63, 0x64, 0x37, 0x62, 0xB4,
	0x13, 0xDA, 0x5F, 0x1A, 0x34, 0x98, 0x9D, 0xB4, 0x63, 0x90, 0x1E, 0xFD, 0x35, 0xD3, 0xDF,
	0x0E, 0x14, 0xD8, 0x6C, 0x49, 0xDB, 0x32, 0xF2, 0x7E, 0x7B, 0x89, 0x3C, 0xE8, 0x16, 0x1B,
	0xA0, 0x12, 0xD0, 0x53, 0xFF, 0xDC, 0xCC, 0xE7, 0xC1, 0x99, 0xDE, 0xEC, 0xD1, 0xFC, 0xFC,
	0x6D, 0x21, 0x9C, 0x98, 0x03, 0x98, 0x1D, 0xF6, 0xDB, 0xFD, 0xC2, 0x61, 0x99, 0xDB, 0x7F,
	0xA0, 0x9D, 0x75, 0xD1, 0x09, 0xE7, 0xF5, 0x8C, 0x5B, 0xE8, 0x4B, 0x91, 0x9B, 0xBD, 0x0A,
	0x43, 0x0E, 0xB3, 0x30, 0x05, 0x77, 0x18, 0xF4, 0xD0, 0x4C, 0x4E, 0xCE, 0x28, 0x3C, 0x05,
	0x6E, 0xDF, 0xE0, 0xF2, 0xBE, 0xC0, 0x50, 0x15, 0xE0, 0xCB, 0xE5, 0x5F, 0x01, 0x5F, 0x7C,
	0xA5, 0x0E, 0xB0, 0x1A, 0x17, 0x2C, 0x12, 0x6E, 0xB0, 0xED, 0xC6, 0xFF, 0x14, 0x47, 0x85,
	0x9D, 0x23, 0x7F, 0x99, 0x0D, 0x15, 0x99, 0xB5, 0xA9, 0xD4, 0x59, 0x98, 0xB1, 0xD6, 0xEE,
	0xD3, 0x4E, 0x48, 0x75, 0x63, 0xFF, 0x1B, 0x3A, 0x95, 0xE7, 0x83, 0x69, 0x74, 0xBA, 0x07,
	0x53, 0x07, 0xF5, 0x47, 0x4D, 0xE7, 0x4E, 0xFE, 0x35, 0xF9, 0x21, 0x95, 0x93, 0x8E, 0xB8,
	0xE8, 0xDD, 0xDE, 0x21, 0xE1, 0xA2, 0x6D, 0xC0, 0x55, 0xFE, 0x1F, 0xF4, 0x3B, 0xE8, 0x50,
	0x4D, 0x13, 0x8F, 0x02, 0x6B, 0xAA, 0x0C, 0x04, 0x43, 0xCD, 0xFC, 0x81, 0x8A, 0x6F, 0x75,
	0xD6, 0x97, 0xE8, 0x82, 0x6D, 0x39, 0x54, 0x84, 0x84, 0x83, 0x32, 0xA1, 0xD1, 0x22, 0x9E,
	0x6F, 0x32, 0xA8, 0xC8, 0xAB, 0x9A, 0x73, 0x1B, 0x2B, 0x85, 0xA8, 0x0B, 0x55, 0x1D, 0x17,
	0xD9, 0x2D, 0xDB, 0x9C, 0x77, 0x1E, 0x42, 0xDE, 0x94, 0x66, 0xBD, 0xF8, 0xD4, 0x0D, 0x2C,
	0x27, 0x3A, 0x1B, 0x47, 0x7E, 0x52, 0xBB, 0x2A, 0x13, 0x59, 0xC0, 0xDE, 0x1F, 0xBA, 0xFD,
	0xBC, 0x5A, 0x2F, 0x2C, 0x3D, 0xBD, 0x8D, 0x8A, 0xDD, 0x9E, 0x8B, 0x20, 0xB8, 0x80, 0xA2,
	0x1C, 0xB6, 0x55, 0xA8, 0x63, 0x6F, 0xD2, 0x39, 0x73, 0xC9, 0xD9, 0xA3, 0x27, 0x3D, 0x39,
	0x25, 0x38, 0x11, 0x94, 0xE6, 0x1D, 0xEE, 0xD2, 0x71, 0x8D, 0x90, 0x48, 0xEA, 0x96, 0x51,
	0x48, 0x3B, 0x1D, 0x58, 0xD2, 0x69, 0x28, 0x22, 0xE6, 0x87, 0x69, 0x45, 0x54, 0x16, 0x4E,
	0x56, 0xF4, 0xFC, 0x9B, 0x95, 0xFC, 0xFE, 0xA2, 0xD3, 0xC7, 0xF0, 0x91, 0x2E, 0x43, 0xE2,
	0x8D, 0xBE, 0x7F, 0x78, 0x02, 0x25, 0xF8, 0x36, 0x0B, 0x82, 0xB9, 0x74, 0xDE, 0x00, 0x99,
	0xC9, 0x03, 0x93, 0x06, 0xE1, 0x67, 0x95, 0xB6, 0x2B, 0xE3, 0x49, 0xE6, 0xC3, 0xD3, 0xCA,
	0xC0, 0x8A, 0x03, 0x50, 0x68, 0x59, 0xAF, 0x6C, 0x54, 0x0C, 0x6E, 0x73, 0xF2, 0x5D, 0x66,
	0x18, 0x35, 0xA2, 0x7D, 0xE9, 0xD4, 0xD7, 0xF7, 0xDE, 0x50, 0x06, 0x8F, 0xE3, 0x4F, 0x67,
	0xCA, 0x8F, 0xE7, 0xFD, 0x00, 0x22, 0xD5, 0xD1, 0x2D, 0x7C, 0x62, 0xC1, 0x49, 0xF2, 0x6B,
	0xE2, 0xF1, 0x1B, 0xD0, 0xF4, 0xEE, 0x3D, 0x7E, 0x25, 0x73, 0xCB, 0x35, 0xBA, 0x69, 0x2C,
	0x13, 0x71, 0x60, 0x01, 0xAA, 0xC0, 0x02, 0xDC, 0x17, 0xD1, 0xED, 0x71, 0xE9, 0x6D, 0x23,
	0x9D, 0x30, 0xD7, 0xED, 0xD5, 0xEE, 0xD7, 0xC5, 0xB7, 0x52, 0x95, 0x72, 0x8E, 0xB6, 0x5E,
	0xE3, 0x62, 0x5E, 0xA4, 0xC2, 0x54, 0xA3, 0xEC, 0x3B, 0xB9, 0x5D, 0xB9, 0x50, 0x9B, 0x7B,
	0x01, 0xB3, 0xA1, 0x7D, 0xAB, 0x36, 0xA5, 0x6F, 0xC4, 0x9E, 0xBE, 0x7B, 0xBA, 0xF1, 0x1C,
	0xAA, 0x9E, 0x20, 0x4D, 0x88, 0xD0, 0x19, 0xF2, 0xA0, 0xE9, 0xDC, 0xEF, 0x1A, 0x07, 0xDD,
	0x02, 0x80, 0x9A, 0xB9, 0x58, 0x06, 0x34, 0x9B, 0x34, 0xA1, 0x73, 0x20, 0x5B, 0x57, 0x8C,
	0x6A, 0xF5, 0x3C, 0x39, 0x45, 0x63, 0x40, 0x46, 0x42, 0x7B, 0x89, 0x05, 0x42, 0xB7, 0xAF,
	0x69, 0xF6, 0x78, 0x87, 0x29, 0xD7, 0x84, 0x90, 0x67, 0x82, 0x4B, 0xC5, 0x21, 0x3D, 0x36,
	0xBA, 0xA2, 0xEC, 0x2B, 0x4A, 0x6B, 0x43, 0x1D, 0x43, 0x49, 0xF6, 0x5E, 0x53, 0x8B, 0x96,
	0x97, 0xBE, 0xE7, 0x6C, 0x98, 0xDF, 0x2A, 0x55, 0x57, 0xE8, 0x2B, 0x23, 0x52, 0x94, 0xC5,
	0x5C, 0xE0, 0xF5, 0x5D, 0xA0, 0xE9, 0x87, 0x39, 0xBF, 0xF2, 0xC5, 0xD4, 0xB7, 0xC5, 0x9F,
	0x49, 0x86, 0x12, 0x05, 0x48, 0x9D, 0x00, 0x84, 0x68, 0xA8, 0x65, 0xE3, 0xAC, 0xE3, 0x07,
	0xAC, 0x59, 0x1F, 0x46, 0xF8, 0x8A, 0x48, 0xF4, 0xB3, 0x1F, 0x52, 0x1F, 0xD5, 0x17, 0x7A,
	0x3E, 0xAD, 0x1B, 0xF2, 0x07, 0xE3, 0x22, 0x19, 0x63, 0xEA, 0xFC, 0x62, 0xF8, 0x26, 0x86,
	0xE3, 0x5F, 0x82, 0x37, 0x73, 0x67, 0x22, 0xA5, 0x4E, 0xB4, 0xE8, 0x2D, 0xCC, 0x9B, 0xB9,
	0xAB, 0x2E, 0x71, 0x45, 0x01, 0x37, 0x4D, 0x7B, 0xC8, 0xBF, 0xD6, 0x00, 0x1C, 0x8C, 0x6A,
	0xF7, 0x0C, 0x5F, 0xC1, 0x05, 0xB9, 0x73, 0xC7, 0x33, 0x36, 0x23, 0x1E, 0x27, 0x73, 0xF7,
	0x93, 0x21, 0x27, 0x97, 0xD2, 0xD4, 0x7F, 0xCB, 0xAD, 0x27, 0x48, 0xCE, 0xCD, 0x88, 0x1E,
	0x6D, 0x6F, 0x93, 0xD8, 0x53, 0x14, 0x0E, 0x54, 0x17, 0xDA, 0xE6, 0xE5, 0x72, 0x08, 0xDD,
	0x9A, 0x70, 0x41, 0x0C, 0xC4, 0x80, 0x69, 0x17, 0xEA, 0xBC, 0xB8, 0x49, 0x2C, 0xD1, 0x3E,
	0xB2, 0xF3, 0xB2, 0x3D, 0xA8, 0x75, 0x75, 0x5F, 0x38, 0xD7, 0xC7, 0x70, 0xAD, 0x05, 0x3E,
	0xE7, 0x63, 0x47, 0x34, 0x66, 0xCD, 0xA9, 0x30, 0x05, 0x84, 0xD8, 0xA5, 0x9C, 0xF1, 0x37,
	0x97, 0x0F, 0xFA, 0xB2, 0x42, 0x52, 0x8F, 0x40, 0xF8, 0x33, 0x5D, 0xCA, 0x3B, 0x69, 0x25,
	0x94, 0x52, 0x64, 0xCE, 0x57, 0xAF, 0xEC, 0x72, 0x3E, 0xAF, 0x57, 0x14, 0xEA, 0xAD, 0xB8,
	0xB6, 0x12, 0x62, 0x6F, 0xE5, 0x5E, 0x35, 0x45, 0x17, 0xA1, 0xD0, 0x93, 0x36, 0xD9, 0x90,
	0x08, 0x0E, 0x4B, 0xE3, 0xF6, 0x0A, 0x38, 0xB2, 0x18, 0xE5, 0x2B, 0xC8, 0x74, 0xAA, 0xDE,
	0x19, 0xD7, 0x40, 0x61, 0x35, 0xAA, 0x4A, 0x6D, 0x51, 0x13, 0xE4, 0x5E, 0x72, 0x58, 0x56,
	0x1A, 0xEE, 0x9A, 0xAF, 0xE6, 0xB7, 0xBD, 0x26, 0xB9, 0x5B, 0x0A, 0x05, 0xC2, 0xDE, 0xBB,
	0xC0, 0xCD, 0x21, 0x2C, 0x13, 0x02, 0xAD, 0xF6, 0x54, 0x26, 0x12, 0xFD, 0x5F, 0x85, 0x9E,
	0x59, 0xFE, 0x37, 0x67, 0x3A, 0x10, 0x39, 0xE3, 0xE0, 0xED, 0xE2, 0xB4, 0xAD, 0x67, 0x38,
	0x05, 0x91, 0xD0, 0x6B, 0x4E, 0xFB, 0xF8, 0xB1, 0x5E, 0x25, 0xA4, 0x3C, 0xC0, 0x8B, 0x12,
	0xA1, 0x1D, 0xC5, 0xA1, 0x4F, 0x13, 0x6E, 0xF8, 0xFA, 0xF1, 0xD3, 0x95, 0x29, 0xD7, 0x44,
	0x58, 0x05, 0xE0, 0x8A, 0xE0, 0xAB, 0x81, 0x0B, 0xF5, 0x74, 0x72, 0x31, 0x9B, 0x1B, 0x34,
	0x60, 0xD3, 0xD1, 0xED, 0xA7, 0xCF, 0x34, 0x11, 0x80, 0x94, 0x19, 0x3C, 0x57, 0x95, 0xCC,
	0x07, 0x54, 0x3E, 0x0E, 0x61, 0x21, 0xB3, 0x0A, 0xD1, 0xF0, 0x23, 0x52, 0xBB, 0x37, 0xBC,
	0xBA, 0xD2, 0x5D, 0x5C, 0xA2, 0x6F, 0x8C, 0x95, 0x8F, 0x06, 0xD0, 0x71, 0x9B, 0x14, 0xC8,
	0xBC, 0x68, 0x7A, 0x2B, 0x25, 0xCE, 0xE8, 0x9E, 0xDD, 0x08, 0xE6, 0x9D, 0xB7, 0x38, 0x33,
	0x25, 0x6F, 0xE8, 0x5B, 0x3A, 0x13, 0xA1, 0xF4, 0xA1, 0x05, 0x45, 0x26, 0x68, 0x35, 0xDD,
	0xCB, 0x03, 0xC6, 0x34
};
__aligned(4) static uint8_t host_rx_buf[I3C_TEST_BUF_SIZE];

static struct k_event *host_events;   /* set from p2 in thread entry */

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
 * IBI callbacks
 * -------------------------------------------------------------------- */
static int i3c_host_ibi_cb(struct i3c_device_desc *target,
			    struct i3c_ibi_payload *payload)
{
	k_event_post(host_events, EVT_IBI_ANY);
	if (payload && payload->payload_len > 0 &&
	    payload->payload[0] == I3C_TEST_MDB_TIR) {
		k_event_post(host_events, EVT_IBI_TIR_MDB_AE);
	}
	return 0;
}

/* --------------------------------------------------------------------
 * CRC loopback verification helper
 * -------------------------------------------------------------------- */
static bool verify_loopback(const uint8_t *tx, const uint8_t *rx,
			     uint32_t len, const char *test_name)
{
#if CONFIG_I3C_TEST_CRC_CHECK
	crc8_t tx_crc = crc8_finalize(crc8_update(crc8_init(), tx, len));
	crc8_t rx_crc = crc8_finalize(crc8_update(crc8_init(), rx, len));

	if (tx_crc != rx_crc) {
		LOG_DBG("[step] %s FAIL: CRC mismatch tx=0x%02x rx=0x%02x",
			test_name, tx_crc, rx_crc);
		return false;
	}
#else
	for (uint32_t i = 0; i < len; i++) {
		if (tx[i] != rx[i]) {
			LOG_DBG("[step] %s FAIL: byte[%u] tx=0x%02x rx=0x%02x",
				test_name, i, tx[i], rx[i]);
			return false;
		}
	}
#endif
	return true;
}

/* ====================================================================
 * TC0: data_transfer
 * ==================================================================== */
static void run_tc0_data_transfer(const struct device *dev,
				  struct i3c_device_desc *target)
{
	TEST_REPORT_INIT();
	int ret;

	/* Step 1: write_1B — fire-and-forget single byte */
	host_tx_buf[0] = 0xA5U;
	ret = i3c_write(target, host_tx_buf, 1);
	if (ret != 0) {
		TEST_SUB_FAIL("write_1B", "i3c_write failed");
	} else {
		TEST_SUB_PASS("write_1B");
	}

	/* Steps 2–5: loopback at increasing lengths */
	static const struct {
		uint32_t    len;
		const char *name;
	} lb_steps[] = {
		{ 1U,    "loopback_1B"    },
		{ 32U,   "loopback_32B"   },
		{ 64U,  "loopback_64B"  },
		{ 128U,  "loopback_128B"  },
		//{ 1024U, "loopback_1024B" },
	};

	for (size_t i = 0; i < ARRAY_SIZE(lb_steps); i++) {
		uint32_t    len  = lb_steps[i].len;
		const char *name = lb_steps[i].name;

		for (uint32_t b = 0; b < len; b++) {
			host_tx_buf[b] = (uint8_t)(b & 0xFFU);
		}

		ret = i3c_write(target, host_tx_buf, len);
		if (ret != 0) {
			TEST_SUB_FAIL(name, "write failed");
			continue;
		}

		k_sleep(K_MSEC(I3C_TEST_ECHO_DELAY_MS));
		memset(host_rx_buf, 0, len);

		ret = i3c_read(target, host_rx_buf, len);
		if (ret != 0) {
			TEST_SUB_FAIL(name, "read failed");
			continue;
		}

		if (!verify_loopback(host_tx_buf, host_rx_buf, len, name)) {
			TEST_SUB_FAIL(name, "data mismatch");
		} else {
			TEST_SUB_PASS(name);
		}
	}

	/* Step 6: combined_xfer (write + repeated-START read) */
	for (uint32_t b = 0; b < 64U; b++) {
		host_tx_buf[b] = (uint8_t)(b & 0xFFU);
	}
	memset(host_rx_buf, 0, 64U);

	ret = i3c_write_read(target, host_tx_buf, 64U, host_rx_buf, 64U);
	if (ret != 0) {
		TEST_SUB_FAIL("combined_xfer", "i3c_write_read failed");
	} else if (!verify_loopback(host_tx_buf, host_rx_buf, 64U, "combined_xfer")) {
		TEST_SUB_FAIL("combined_xfer", "data mismatch");
	} else {
		TEST_SUB_PASS("combined_xfer");
	}

	TEST_GROUP_RESULT("data_transfer");
}

/* ====================================================================
 * TC1: ccc_commands
 * ==================================================================== */
static void run_tc1_ccc_commands(const struct device *dev,
				 struct i3c_device_desc *target)
{
	TEST_REPORT_INIT();
	int ret;

	/* Step 1: SETMWL / GETMWL */
	const struct i3c_ccc_mwl set_mwl = { .len = 128U };
	struct i3c_ccc_mwl get_mwl = { 0 };

	ret = i3c_ccc_do_setmwl(target, &set_mwl);
	if (ret != 0) {
		TEST_SUB_FAIL("setmwl_getmwl", "SETMWL failed");
	} else {
		ret = i3c_ccc_do_getmwl(target, &get_mwl);
		if (ret != 0) {
			TEST_SUB_FAIL("setmwl_getmwl", "GETMWL failed");
		} else if (get_mwl.len != 128U) {
			TEST_SUB_FAIL("setmwl_getmwl", "MWL not accepted");
		} else {
			TEST_SUB_PASS("setmwl_getmwl");
		}
	}

	/* Step 2: SETMRL / GETMRL */
	const struct i3c_ccc_mrl set_mrl = { .len = 128U, .ibi_len = 1U };
	struct i3c_ccc_mrl get_mrl = { 0 };

	ret = i3c_ccc_do_setmrl(target, &set_mrl);
	if (ret != 0) {
		TEST_SUB_FAIL("setmrl_getmrl", "SETMRL failed");
	} else {
		ret = i3c_ccc_do_getmrl(target, &get_mrl);
		if (ret != 0) {
			TEST_SUB_FAIL("setmrl_getmrl", "GETMRL failed");
		} else if (get_mrl.len != 128U) {
			TEST_SUB_FAIL("setmrl_getmrl", "MRL not accepted");
		} else {
			TEST_SUB_PASS("setmrl_getmrl");
		}
	}

	/* Step 3: GETPID */
	struct i3c_ccc_getpid pid_resp = { 0 };

	ret = i3c_ccc_do_getpid(target, &pid_resp);
	if (ret != 0) {
		TEST_SUB_FAIL("getpid", "GETPID failed");
	} else {
		/* Reconstruct 48-bit PID from big-endian 6 bytes */
		uint64_t received_pid = 0;

		for (int i = 0; i < 6; i++) {
			received_pid = (received_pid << 8) | pid_resp.pid[i];
		}
		uint64_t expected_pid =
			((uint64_t)CONFIG_I3C_HOST_TARGET_PID_HI << 32) |
			(uint64_t)CONFIG_I3C_HOST_TARGET_PID_LO;

		if (received_pid != expected_pid) {
			LOG_DBG("GETPID: got 0x%012llx expected 0x%012llx",
				received_pid, expected_pid);
			TEST_SUB_FAIL("getpid", "PID mismatch");
		} else {
			TEST_SUB_PASS("getpid");
		}
	}

	/* Step 4: GETBCR / GETDCR */
	struct i3c_ccc_getbcr bcr = { 0 };
	struct i3c_ccc_getdcr dcr = { 0 };
	int ret_b = i3c_ccc_do_getbcr(target, &bcr);
	int ret_d = i3c_ccc_do_getdcr(target, &dcr);

	if (ret_b != 0) {
		TEST_SUB_FAIL("getbcr_getdcr", "GETBCR failed");
	} else if (ret_d != 0) {
		TEST_SUB_FAIL("getbcr_getdcr", "GETDCR failed");
	} else {
		LOG_DBG("[step] getbcr_getdcr: BCR=0x%02x DCR=0x%02x",
			bcr.bcr, dcr.dcr);
		TEST_SUB_PASS("getbcr_getdcr");
	}

	TEST_GROUP_RESULT("ccc_commands");
}

/* ====================================================================
 * TC2: ibi
 * ==================================================================== */
static void run_tc2_ibi(const struct device *dev, struct i3c_device_desc *target)
{
	TEST_REPORT_INIT();
	int ret;
	uint32_t events;

	k_event_clear(host_events, 0xFFFFFFFFU);

	/* Step 1: ibi_tir — write 128 bytes, expect IBI with MDB=0xAE, read back */
	target->ibi_cb = i3c_host_ibi_cb;
	ret = i3c_ibi_enable(target);
	if (ret != 0) {
		TEST_SUB_FAIL("ibi_tir", "ibi_enable failed");
		goto step2_ibi_enable_disable;
	}

	for (uint32_t b = 0; b < 128U; b++) {
		host_tx_buf[b] = (uint8_t)(b & 0xFFU);
	}
	ret = i3c_write(target, host_tx_buf, 128U);
	if (ret != 0) {
		TEST_SUB_FAIL("ibi_tir", "write failed");
		goto step2_ibi_enable_disable;
	}

	events = k_event_wait(host_events, EVT_IBI_TIR_MDB_AE, true,
			      K_MSEC(I3C_TEST_TIMEOUT_MS));
	if (!(events & EVT_IBI_TIR_MDB_AE)) {
		TEST_SUB_FAIL("ibi_tir", "IBI timeout");
	} else {
		memset(host_rx_buf, 0, 128U);
		ret = i3c_read(target, host_rx_buf, 128U);
		if (ret != 0) {
			TEST_SUB_FAIL("ibi_tir", "read failed");
		} else if (!verify_loopback(host_tx_buf, host_rx_buf, 128U, "ibi_tir")) {
			TEST_SUB_FAIL("ibi_tir", "data mismatch");
		} else {
			TEST_SUB_PASS("ibi_tir");
		}
	}

	memset(host_rx_buf, 0, 128U);
	ret = i3c_read(target, host_rx_buf, 128U);
	if (ret != 0) {
		TEST_SUB_FAIL("ibi_tir", "read failed");
	} else if (!verify_loopback(host_tx_buf, host_rx_buf, 128U, "ibi_tir")) {
		TEST_SUB_FAIL("ibi_tir", "data mismatch");
	} else {
		TEST_SUB_PASS("ibi_tir");
	}

step2_ibi_enable_disable:
#if 0
	/* Step 2a: ibi_enable — verify IBI fires when enabled */
	k_event_clear(host_events, 0xFFFFFFFFU);
	(void)i3c_ibi_enable(target);

	host_tx_buf[0] = 0x01U;
	(void)i3c_write(target, host_tx_buf, 1U);

	events = k_event_wait(host_events, EVT_IBI_ANY, true,
			      K_MSEC(I3C_TEST_IBI_POLL_MS));
	if (events & EVT_IBI_ANY) {
		TEST_SUB_PASS("ibi_enable");
	} else {
		TEST_SUB_FAIL("ibi_enable", "IBI did not fire after enable");
	}

	/* Step 2b: ibi_disable — verify no IBI after disable */
	(void)i3c_ibi_disable(target);
	k_event_clear(host_events, 0xFFFFFFFFU);

	host_tx_buf[0] = 0x02U;
	(void)i3c_write(target, host_tx_buf, 1U);

	events = k_event_wait(host_events, EVT_IBI_ANY, true,
			      K_MSEC(I3C_TEST_IBI_POLL_MS));
	if (events & EVT_IBI_ANY) {
		TEST_SUB_FAIL("ibi_disable", "IBI fired after disable");
	} else {
		TEST_SUB_PASS("ibi_disable");
	}

	/* Step 3: hot_join — configure controller to ACK HJ, wait for HJ IBI */
	k_event_clear(host_events, 0xFFFFFFFFU);
	/*
	 * Enable hot-join ACK on the controller.  This tells the DW IP to
	 * acknowledge (not NACK) an incoming hot-join IBI.
	 */
	ret = i3c_ibi_hj_response(dev, true);
	if (ret != 0 && ret != -ENOSYS) {
		LOG_WRN("hot_join: i3c_ibi_hj_response not supported (%d)", ret);
	}

	events = k_event_wait(host_events, EVT_IBI_HOTJOIN, true,
			      K_MSEC(I3C_TEST_HJ_TIMEOUT_MS));
	if (events & EVT_IBI_HOTJOIN) {
		TEST_SUB_PASS("hot_join");
	} else {
		TEST_SUB_FAIL("hot_join", "HJ IBI timeout");
	}

	/* Step 4: ibi_reject — disable IBI, verify none fired */
	(void)i3c_ibi_disable(target);
	k_event_clear(host_events, 0xFFFFFFFFU);

	for (uint32_t b = 0; b < 64U; b++) {
		host_tx_buf[b] = (uint8_t)(b & 0xFFU);
	}
	(void)i3c_write(target, host_tx_buf, 64U);

	events = k_event_wait(host_events, EVT_IBI_ANY, true,
			      K_MSEC(I3C_TEST_IBI_POLL_MS));
	if (events & EVT_IBI_ANY) {
		TEST_SUB_FAIL("ibi_reject", "IBI received after disable");
	} else {
		TEST_SUB_PASS("ibi_reject");
	}
#endif
	TEST_GROUP_RESULT("ibi");
}

/* ====================================================================
 * TC3: advanced
 * ==================================================================== */
static void run_tc3_advanced(const struct device *dev,
			     struct i3c_device_desc *target)
{
	TEST_REPORT_INIT();
	int ret;

	/* Steps 1–3: frequency sweep at 1 / 6.25 / 12.5 MHz */
	static const struct {
		uint32_t    hz;
		const char *name;
	} freqs[] = {
		{ 1000000U,  "freq_1MHz"    },
		{ 6250000U,  "freq_6_25MHz" },
		{ 12500000U, "freq_12_5MHz" },
	};

	struct i3c_config_controller scl_cfg;

	for (size_t f = 0; f < ARRAY_SIZE(freqs); f++) {
		memset(&scl_cfg, 0, sizeof(scl_cfg));
		scl_cfg.scl.i3c = freqs[f].hz;

		ret = i3c_configure(dev, I3C_CONFIG_CONTROLLER, &scl_cfg);
		if (ret != 0) {
			TEST_SUB_FAIL(freqs[f].name, "i3c_configure failed");
			continue;
		}

		for (uint32_t b = 0; b < 64U; b++) {
			host_tx_buf[b] = (uint8_t)(b & 0xFFU);
		}
		memset(host_rx_buf, 0, 64U);

		int ret_w = i3c_write(target, host_tx_buf, 64U);

		k_sleep(K_MSEC(I3C_TEST_ECHO_DELAY_MS));

		int ret_r = i3c_read(target, host_rx_buf, 64U);

		if (ret_w != 0 || ret_r != 0) {
			TEST_SUB_FAIL(freqs[f].name, "transfer failed");
		} else if (!verify_loopback(host_tx_buf, host_rx_buf, 64U, freqs[f].name)) {
			TEST_SUB_FAIL(freqs[f].name, "data mismatch");
		} else {
			TEST_SUB_PASS(freqs[f].name);
		}
	}

	/* Restore SCL to 12.5 MHz regardless of sweep results */
	memset(&scl_cfg, 0, sizeof(scl_cfg));
	scl_cfg.scl.i3c = I3C_SCL_TYP_HZ;
	(void)i3c_configure(dev, I3C_CONFIG_CONTROLLER, &scl_cfg);

	/* Step 4: err_invalid_addr — write to unassigned address 0x7F */
	struct i3c_device_desc bad_desc = {
		.bus          = dev,
		.dynamic_addr = 0x7FU,
	};
	ret = i3c_write(&bad_desc, host_tx_buf, 1U);
	if (ret == 0) {
		TEST_SUB_FAIL("err_invalid_addr", "expected error, got 0");
	} else {
		TEST_SUB_PASS("err_invalid_addr");
	}

	/* Step 5: err_exceed_mwl — SETMWL(32), then try to write 128 bytes */
	const struct i3c_ccc_mwl small_mwl = { .len = 32U };

	ret = i3c_ccc_do_setmwl(target, &small_mwl);
	if (ret != 0) {
		TEST_SUB_FAIL("err_exceed_mwl", "SETMWL(32) failed");
	} else {
		ret = i3c_write(target, host_tx_buf, 128U);
		if (ret == 0) {
			TEST_SUB_FAIL("err_exceed_mwl", "expected error, got 0");
		} else {
			TEST_SUB_PASS("err_exceed_mwl");
		}
	}

	/* Always restore MWL to 128 to leave bus in a clean state */
	const struct i3c_ccc_mwl restore_mwl = { .len = 128U };

	(void)i3c_ccc_do_setmwl(target, &restore_mwl);

	TEST_GROUP_RESULT("advanced");
}

/* ====================================================================
 * setup_i3c_host — called from main() before thread start
 * ==================================================================== */
int setup_i3c_host(const struct device *dev)
{
	if (!device_is_ready(dev)) {
		int ret = device_init(dev);

		if (ret != 0) {
			LOG_ERR("Host[%s] init failed: %d", dev->name, ret);
			return ret;
		}
	}
	LOG_INF("Host[%s] ready", dev->name);
	return 0;
}

/* ====================================================================
 * i3c_host_xfer_task — thread entry point
 * ==================================================================== */
void i3c_host_xfer_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p3);

	const struct device *dev = (const struct device *)p1;

	host_events = (struct k_event *)p2;

	/*
	 * Wait for the target to complete i3c_target_register().
	 * In the standalone host app, main() posts this bit before starting
	 * the thread so the wait completes immediately.
	 * In the loopback app, the target thread posts it after register.
	 */
	(void)k_event_wait(host_events, EVT_TARGET_READY, false, K_FOREVER);

	const struct i3c_device_id devid = {
		.pid = ((uint64_t)CONFIG_I3C_HOST_TARGET_PID_HI << 32) |
		       (uint64_t)CONFIG_I3C_HOST_TARGET_PID_LO
	};

	struct i3c_device_desc *target = i3c_device_find(dev, &devid);

	if (!target || !target->dynamic_addr) {
		uint64_t pid_val = devid.pid;
		LOG_ERR("[FAIL] target 0x%012llx not found on bus (dynamic_addr=0)",
			pid_val);
		k_sleep(K_FOREVER);
		return;
	}

	LOG_INF("Target found: dynamic_addr=0x%02x", target->dynamic_addr);
	target->ibi_cb = i3c_host_ibi_cb;

	switch (CONFIG_I3C_TEST_CASE) {
	case 0:
		run_tc0_data_transfer(dev, target);
		break;
	case 1:
		run_tc1_ccc_commands(dev, target);
		break;
	case 2:
		run_tc2_ibi(dev, target);
		break;
	case 3:
		run_tc3_advanced(dev, target);
		break;
	default:
		LOG_ERR("[FAIL] unknown test case %d", CONFIG_I3C_TEST_CASE);
		break;
	}

	k_sleep(K_FOREVER);
}
