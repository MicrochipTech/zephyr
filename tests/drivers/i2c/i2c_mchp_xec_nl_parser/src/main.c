/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip XEC I2C-NL message parser tests.
 */

#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/ztest.h>

#include "i2c_mchp_xec_nl_parser.h"

#define TADDR    0x50U
#define TADDR_WR (TADDR << 1)
#define TADDR_RD ((TADDR << 1) | 1U)

#define CTRL_START_STOP (I2C_HW_CTRL_START | I2C_HW_CTRL_STOP)
#define CTRL_WR_RD      (I2C_HW_CTRL_START | I2C_HW_CTRL_RPT_START | I2C_HW_CTRL_STOP)

static uint8_t buf0[8];
static uint8_t buf1[8];
static struct i2c_xfer_desc desc;

static void set_msg(struct i2c_msg *m, uint8_t *buf, uint32_t len, uint8_t flags)
{
	m->buf = buf;
	m->len = len;
	m->flags = flags;
}

static void check_tx(size_t idx, const uint8_t *buf, uint32_t len)
{
	zassert_equal_ptr(desc.tx[idx].buf, buf, "tx[%zu].buf", idx);
	zassert_equal(desc.tx[idx].len, len, "tx[%zu].len", idx);
}

static void check_rx(size_t idx, const uint8_t *buf, uint32_t len)
{
	zassert_equal_ptr(desc.rx[idx].buf, buf, "rx[%zu].buf", idx);
	zassert_equal(desc.rx[idx].len, len, "rx[%zu].len", idx);
}

static void check_addr_bytes(void)
{
	zassert_equal(desc.addr, TADDR);
	zassert_equal(desc.addr_wr, TADDR_WR);
	zassert_equal(desc.addr_rd, TADDR_RD);
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	/* Poison the descriptor to prove the parser initializes every field */
	memset(&desc, 0xA5, sizeof(desc));
}

ZTEST_SUITE(xec_i2c_nl_parser, NULL, NULL, before, NULL, NULL);

/* Valid single message sequences */

ZTEST(xec_i2c_nl_parser, test_write)
{
	struct i2c_msg m[1];

	set_msg(&m[0], buf0, 3U, I2C_MSG_WRITE | I2C_MSG_STOP);
	zassert_ok(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc));

	check_addr_bytes();
	zassert_equal(desc.kind, I2C_XFER_WRITE);
	zassert_equal(desc.ctrl, CTRL_START_STOP);
	zassert_equal(desc.tx_nseg, 2U);
	check_tx(0, &desc.addr_wr, 1U);
	check_tx(1, buf0, 3U);
	zassert_equal(desc.rx_nseg, 0U);
	zassert_equal(desc.tx_count, 4U);
	zassert_equal(desc.rx_count, 0U);
}

ZTEST(xec_i2c_nl_parser, test_write_ping)
{
	struct i2c_msg m[1];

	/* buf NULL and non-NULL are both accepted for a zero length write */
	set_msg(&m[0], NULL, 0U, I2C_MSG_WRITE | I2C_MSG_STOP);
	zassert_ok(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc));
	zassert_equal(desc.kind, I2C_XFER_WRITE);
	zassert_equal(desc.ctrl, CTRL_START_STOP);
	zassert_equal(desc.tx_nseg, 1U);
	check_tx(0, &desc.addr_wr, 1U);
	zassert_equal(desc.rx_nseg, 0U);
	zassert_equal(desc.tx_count, 1U);
	zassert_equal(desc.rx_count, 0U);

	set_msg(&m[0], buf0, 0U, I2C_MSG_WRITE | I2C_MSG_STOP);
	zassert_ok(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc));
	zassert_equal(desc.tx_nseg, 1U);
	zassert_equal(desc.tx_count, 1U);
}

ZTEST(xec_i2c_nl_parser, test_read)
{
	struct i2c_msg m[1];

	set_msg(&m[0], buf0, 4U, I2C_MSG_READ | I2C_MSG_STOP);
	zassert_ok(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc));

	check_addr_bytes();
	zassert_equal(desc.kind, I2C_XFER_READ);
	zassert_equal(desc.ctrl, CTRL_START_STOP);
	zassert_equal(desc.tx_nseg, 1U);
	check_tx(0, &desc.addr_rd, 1U);
	zassert_equal(desc.rx_nseg, 1U);
	check_rx(0, buf0, 4U);
	zassert_equal(desc.tx_count, 1U);
	zassert_equal(desc.rx_count, 4U);
}

ZTEST(xec_i2c_nl_parser, test_read_ping)
{
	struct i2c_msg m[1];

	set_msg(&m[0], NULL, 1U, I2C_MSG_READ | I2C_MSG_STOP);
	zassert_ok(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc));

	zassert_equal(desc.kind, I2C_XFER_READ);
	zassert_equal(desc.ctrl, CTRL_START_STOP);
	zassert_equal(desc.tx_nseg, 1U);
	check_tx(0, &desc.addr_rd, 1U);
	zassert_equal(desc.rx_nseg, 1U);
	check_rx(0, &desc.rx_discard, 1U);
	zassert_equal(desc.tx_count, 1U);
	zassert_equal(desc.rx_count, 1U);
}

/* Valid two message sequences */

ZTEST(xec_i2c_nl_parser, test_write_write)
{
	struct i2c_msg m[2];

	set_msg(&m[0], buf0, 2U, I2C_MSG_WRITE);
	set_msg(&m[1], buf1, 5U, I2C_MSG_WRITE | I2C_MSG_STOP);
	zassert_ok(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc));

	check_addr_bytes();
	zassert_equal(desc.kind, I2C_XFER_WRITE_WRITE);
	zassert_equal(desc.ctrl, CTRL_START_STOP);
	zassert_equal(desc.tx_nseg, 3U);
	check_tx(0, &desc.addr_wr, 1U);
	check_tx(1, buf0, 2U);
	check_tx(2, buf1, 5U);
	zassert_equal(desc.rx_nseg, 0U);
	zassert_equal(desc.tx_count, 8U);
	zassert_equal(desc.rx_count, 0U);
}

ZTEST(xec_i2c_nl_parser, test_read_read)
{
	struct i2c_msg m[2];

	set_msg(&m[0], buf0, 3U, I2C_MSG_READ);
	set_msg(&m[1], buf1, 6U, I2C_MSG_READ | I2C_MSG_STOP);
	zassert_ok(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc));

	check_addr_bytes();
	zassert_equal(desc.kind, I2C_XFER_READ_READ);
	zassert_equal(desc.ctrl, CTRL_START_STOP);
	zassert_equal(desc.tx_nseg, 1U);
	check_tx(0, &desc.addr_rd, 1U);
	zassert_equal(desc.rx_nseg, 2U);
	check_rx(0, buf0, 3U);
	check_rx(1, buf1, 6U);
	zassert_equal(desc.tx_count, 1U);
	zassert_equal(desc.rx_count, 9U);
}

ZTEST(xec_i2c_nl_parser, test_write_read)
{
	struct i2c_msg m[2];

	set_msg(&m[0], buf0, 2U, I2C_MSG_WRITE);
	set_msg(&m[1], buf1, 4U, I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP);
	zassert_ok(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc));

	check_addr_bytes();
	zassert_equal(desc.kind, I2C_XFER_WRITE_READ);
	zassert_equal(desc.ctrl, CTRL_WR_RD);
	/* STARTN: HW sends the repeated start before the last write byte, addr_rd */
	zassert_equal(desc.tx_nseg, 3U);
	check_tx(0, &desc.addr_wr, 1U);
	check_tx(1, buf0, 2U);
	check_tx(2, &desc.addr_rd, 1U);
	zassert_equal(desc.rx_nseg, 1U);
	check_rx(0, buf1, 4U);
	zassert_equal(desc.tx_count, 4U);
	zassert_equal(desc.rx_count, 4U);
}

/* The parser only builds descriptors: buffers are not accessed. Large lengths
 * with a small buffer are fine here.
 */
ZTEST(xec_i2c_nl_parser, test_count_limits)
{
	struct i2c_msg m[2];

	/* TX count includes the address byte */
	set_msg(&m[0], buf0, I2C_HW_MAX_COUNT - 1U, I2C_MSG_WRITE | I2C_MSG_STOP);
	zassert_ok(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc));
	zassert_equal(desc.tx_count, I2C_HW_MAX_COUNT);

	set_msg(&m[0], buf0, I2C_HW_MAX_COUNT, I2C_MSG_WRITE | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc), -EINVAL);

	set_msg(&m[0], buf0, I2C_HW_MAX_COUNT, I2C_MSG_READ | I2C_MSG_STOP);
	zassert_ok(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc));
	zassert_equal(desc.rx_count, I2C_HW_MAX_COUNT);

	set_msg(&m[0], buf0, I2C_HW_MAX_COUNT + 1U, I2C_MSG_READ | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc), -EINVAL);

	/* Write-read: two address bytes in the TX count */
	set_msg(&m[0], buf0, I2C_HW_MAX_COUNT - 2U, I2C_MSG_WRITE);
	set_msg(&m[1], buf1, I2C_HW_MAX_COUNT, I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP);
	zassert_ok(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc));
	zassert_equal(desc.tx_count, I2C_HW_MAX_COUNT);
	zassert_equal(desc.rx_count, I2C_HW_MAX_COUNT);

	set_msg(&m[0], buf0, I2C_HW_MAX_COUNT - 1U, I2C_MSG_WRITE);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc), -EINVAL);

	/* Two reads whose sum exceeds the RX count */
	set_msg(&m[0], buf0, 0x8000U, I2C_MSG_READ);
	set_msg(&m[1], buf1, 0x8000U, I2C_MSG_READ | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc), -EINVAL);

	/* Lengths whose uint32_t sum wraps to a small value */
	set_msg(&m[0], buf0, UINT32_MAX, I2C_MSG_WRITE);
	set_msg(&m[1], buf1, 2U, I2C_MSG_WRITE | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc), -EINVAL);
}

ZTEST(xec_i2c_nl_parser, test_desc_reinit)
{
	struct i2c_msg m[2];

	set_msg(&m[0], buf0, 2U, I2C_MSG_WRITE);
	set_msg(&m[1], buf1, 4U, I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP);
	zassert_ok(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc));

	/* Reusing the descriptor leaves nothing from the previous request */
	set_msg(&m[0], buf0, 1U, I2C_MSG_WRITE | I2C_MSG_STOP);
	zassert_ok(xec_i2c_nl_xfer_parse(m, 1U, 0x10U, &desc));
	zassert_equal(desc.addr_wr, 0x20U);
	zassert_equal(desc.addr_rd, 0x21U);
	zassert_equal(desc.ctrl, CTRL_START_STOP);
	zassert_equal(desc.tx_nseg, 2U);
	check_tx(2, NULL, 0U);
	zassert_equal(desc.rx_nseg, 0U);
	check_rx(0, NULL, 0U);
	zassert_equal(desc.rx_count, 0U);
	zassert_equal(desc.elen, 0U);
}

/* Invalid arguments */

ZTEST(xec_i2c_nl_parser, test_bad_args)
{
	struct i2c_msg m[3];

	set_msg(&m[0], buf0, 1U, I2C_MSG_WRITE | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(NULL, 1U, TADDR, &desc), -EINVAL);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 1U, TADDR, NULL), -EINVAL);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 0U, TADDR, &desc), -EINVAL);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 1U, 0x80U, &desc), -EINVAL);
	zassert_ok(xec_i2c_nl_xfer_parse(m, 1U, 0x7FU, &desc));
	zassert_equal(desc.addr_rd, 0xFFU);

	set_msg(&m[0], buf0, 1U, I2C_MSG_WRITE);
	set_msg(&m[1], buf0, 1U, I2C_MSG_WRITE);
	set_msg(&m[2], buf0, 1U, I2C_MSG_WRITE | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 3U, TADDR, &desc), -ENOTSUP);
}

ZTEST(xec_i2c_nl_parser, test_bad_single)
{
	struct i2c_msg m[1];

	/* STOP is mandatory */
	set_msg(&m[0], buf0, 1U, I2C_MSG_WRITE);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc), -ENOTSUP);
	set_msg(&m[0], buf0, 1U, I2C_MSG_READ);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc), -ENOTSUP);

	/* NULL buffer is only allowed for a write ping or a 1 byte read ping */
	set_msg(&m[0], NULL, 2U, I2C_MSG_WRITE | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc), -EINVAL);
	set_msg(&m[0], NULL, 2U, I2C_MSG_READ | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc), -EINVAL);
	set_msg(&m[0], NULL, 0U, I2C_MSG_READ | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc), -EINVAL);

	/* Zero length read */
	set_msg(&m[0], buf0, 0U, I2C_MSG_READ | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc), -EINVAL);

	/* 10-bit addressing, including the ping forms */
	set_msg(&m[0], buf0, 1U, I2C_MSG_WRITE | I2C_MSG_STOP | I2C_MSG_ADDR_10_BITS);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc), -ENOTSUP);
	set_msg(&m[0], NULL, 0U, I2C_MSG_WRITE | I2C_MSG_STOP | I2C_MSG_ADDR_10_BITS);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc), -ENOTSUP);
	set_msg(&m[0], NULL, 1U, I2C_MSG_READ | I2C_MSG_STOP | I2C_MSG_ADDR_10_BITS);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 1U, TADDR, &desc), -ENOTSUP);
}

ZTEST(xec_i2c_nl_parser, test_bad_two)
{
	struct i2c_msg m[2];

	/* STOP on the first message */
	set_msg(&m[0], buf0, 1U, I2C_MSG_WRITE | I2C_MSG_STOP);
	set_msg(&m[1], buf1, 1U, I2C_MSG_WRITE | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc), -ENOTSUP);

	/* No STOP on the second message */
	set_msg(&m[0], buf0, 1U, I2C_MSG_WRITE);
	set_msg(&m[1], buf1, 1U, I2C_MSG_READ | I2C_MSG_RESTART);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc), -ENOTSUP);

	/* Write-read without RESTART on the read */
	set_msg(&m[1], buf1, 1U, I2C_MSG_READ | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc), -ENOTSUP);

	/* RESTART between same direction messages */
	set_msg(&m[1], buf1, 1U, I2C_MSG_WRITE | I2C_MSG_RESTART | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc), -ENOTSUP);
	set_msg(&m[0], buf0, 1U, I2C_MSG_READ);
	set_msg(&m[1], buf1, 1U, I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc), -ENOTSUP);

	/* Read then write */
	set_msg(&m[1], buf1, 1U, I2C_MSG_WRITE | I2C_MSG_RESTART | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc), -ENOTSUP);

	/* Zero length or NULL buffers are not allowed with two messages */
	set_msg(&m[0], buf0, 0U, I2C_MSG_WRITE);
	set_msg(&m[1], buf1, 1U, I2C_MSG_WRITE | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc), -EINVAL);
	set_msg(&m[0], buf0, 1U, I2C_MSG_WRITE);
	set_msg(&m[1], NULL, 1U, I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc), -EINVAL);

	/* 10-bit addressing */
	set_msg(&m[1], buf1, 1U, I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP |
		I2C_MSG_ADDR_10_BITS);
	zassert_equal(xec_i2c_nl_xfer_parse(m, 2U, TADDR, &desc), -ENOTSUP);
}
