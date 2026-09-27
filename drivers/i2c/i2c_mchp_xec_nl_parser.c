/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/i2c.h>

#include "i2c_mchp_xec_nl_parser.h"

static inline bool msg_is_read(const struct i2c_msg *m)
{
	return (m->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ;
}

/* A message longer than the HW count can never be transferred. Checking each
 * message also keeps the uint32_t count totals from wrapping.
 */
static int check_msg(const struct i2c_msg *m, bool allow_empty)
{
	if ((m->flags & I2C_MSG_ADDR_10_BITS) != 0U) {
		return -ENOTSUP;
	}
	if (m->len == 0U) {
		return allow_empty ? 0 : -EINVAL;
	}
	if (m->len > I2C_HW_MAX_COUNT) {
		return -EINVAL;
	}
	if (m->buf == NULL) {
		return -EINVAL;
	}
	return 0;
}

/* Append a segment; zero-length segments are skipped (no empty DMA blocks). */
static void push_tx(struct i2c_xfer_desc *d, uint32_t *total, uint8_t *buf, uint32_t len)
{
	if (len == 0U) {
		return;
	}
	d->tx[d->tx_nseg].buf = buf;
	d->tx[d->tx_nseg].len = len;
	d->tx_nseg++;
	*total += len;
}

static void push_rx(struct i2c_xfer_desc *d, uint32_t *total, uint8_t *buf, uint32_t len)
{
	if (len == 0U) {
		return;
	}
	d->rx[d->rx_nseg].buf = buf;
	d->rx[d->rx_nseg].len = len;
	d->rx_nseg++;
	*total += len;
}

static int parse_one(const struct i2c_msg *m0, struct i2c_xfer_desc *d, uint32_t *tx_tot,
		     uint32_t *rx_tot)
{
	int rc;

	/* Missing STOP only happens with CONFIG_I2C_ALLOW_NO_STOP_TRANSACTIONS */
	if ((m0->flags & I2C_MSG_STOP) == 0U) {
		return -ENOTSUP;
	}

	d->ctrl = I2C_HW_CTRL_START | I2C_HW_CTRL_STOP;

	if (msg_is_read(m0)) {
		d->kind = I2C_XFER_READ;
		push_tx(d, tx_tot, &d->addr_rd, 1U);

		/* Read ping: HW needs a non-zero read count, read one byte and discard it */
		if ((m0->buf == NULL) && (m0->len == 1U) &&
		    ((m0->flags & I2C_MSG_ADDR_10_BITS) == 0U)) {
			push_rx(d, rx_tot, &d->rx_discard, 1U);
			return 0;
		}

		rc = check_msg(m0, false);
		if (rc != 0) {
			return rc;
		}
		push_rx(d, rx_tot, m0->buf, m0->len);
	} else {
		/* Zero length write is a write ping: address byte only */
		rc = check_msg(m0, true);
		if (rc != 0) {
			return rc;
		}
		d->kind = I2C_XFER_WRITE;
		push_tx(d, tx_tot, &d->addr_wr, 1U);
		push_tx(d, tx_tot, m0->buf, m0->len);
	}
	return 0;
}

static int parse_two(const struct i2c_msg *m0, const struct i2c_msg *m1, struct i2c_xfer_desc *d,
		     uint32_t *tx_tot, uint32_t *rx_tot)
{
	bool r0 = msg_is_read(m0);
	bool r1 = msg_is_read(m1);
	int rc;

	/* STOP only on the second message */
	if (((m0->flags & I2C_MSG_STOP) != 0U) || ((m1->flags & I2C_MSG_STOP) == 0U)) {
		return -ENOTSUP;
	}

	rc = check_msg(m0, false);
	if (rc != 0) {
		return rc;
	}
	rc = check_msg(m1, false);
	if (rc != 0) {
		return rc;
	}

	if (!r0 && r1) {
		/* Write then read: repeated start required on the read */
		if ((m1->flags & I2C_MSG_RESTART) == 0U) {
			return -ENOTSUP;
		}
		d->kind = I2C_XFER_WRITE_READ;
		d->ctrl = I2C_HW_CTRL_START | I2C_HW_CTRL_RPT_START | I2C_HW_CTRL_STOP;
		push_tx(d, tx_tot, &d->addr_wr, 1U);
		push_tx(d, tx_tot, m0->buf, m0->len);
		push_tx(d, tx_tot, &d->addr_rd, 1U);
		push_rx(d, rx_tot, m1->buf, m1->len);
		return 0;
	}

	if (r0 == r1) {
		/* Same direction: one continuous transfer, no repeated start */
		if ((m1->flags & I2C_MSG_RESTART) != 0U) {
			return -ENOTSUP;
		}
		d->ctrl = I2C_HW_CTRL_START | I2C_HW_CTRL_STOP;
		if (r0) {
			d->kind = I2C_XFER_READ_READ;
			push_tx(d, tx_tot, &d->addr_rd, 1U);
			push_rx(d, rx_tot, m0->buf, m0->len);
			push_rx(d, rx_tot, m1->buf, m1->len);
		} else {
			d->kind = I2C_XFER_WRITE_WRITE;
			push_tx(d, tx_tot, &d->addr_wr, 1U);
			push_tx(d, tx_tot, m0->buf, m0->len);
			push_tx(d, tx_tot, m1->buf, m1->len);
		}
		return 0;
	}

	/* Read then write: second direction change, not supported */
	return -ENOTSUP;
}

/* Initialize every field explicitly: no memset() so the parser is cheap in ISR context */
static void desc_init(struct i2c_xfer_desc *desc, uint16_t addr)
{
	desc->kind = I2C_XFER_WRITE;
	desc->addr = addr;
	desc->addr_wr = (uint8_t)(addr << 1);
	desc->addr_rd = (uint8_t)((addr << 1) | 1U);
	desc->rx_discard = 0U;
	for (size_t i = 0; i < I2C_XFER_MAX_TX_SEGS; i++) {
		desc->tx[i].buf = NULL;
		desc->tx[i].len = 0U;
	}
	desc->tx_nseg = 0U;
	for (size_t i = 0; i < I2C_XFER_MAX_RX_SEGS; i++) {
		desc->rx[i].buf = NULL;
		desc->rx[i].len = 0U;
	}
	desc->rx_nseg = 0U;
	desc->tx_count = 0U;
	desc->rx_count = 0U;
	desc->ctrl = 0U;
	desc->elen = 0U;
}

int xec_i2c_nl_xfer_parse(const struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr,
			  struct i2c_xfer_desc *desc)
{
	uint32_t tx_tot = 0U;
	uint32_t rx_tot = 0U;
	int rc;

	if ((msgs == NULL) || (desc == NULL) || (num_msgs == 0U)) {
		return -EINVAL;
	}
	/* 10-bit addressing is rejected per message; addr must be 7-bit */
	if (addr > 0x7FU) {
		return -EINVAL;
	}

	desc_init(desc, addr);

	switch (num_msgs) {
	case 1:
		rc = parse_one(&msgs[0], desc, &tx_tot, &rx_tot);
		break;
	case 2:
		rc = parse_two(&msgs[0], &msgs[1], desc, &tx_tot, &rx_tot);
		break;
	default:
		rc = -ENOTSUP;
		break;
	}
	if (rc != 0) {
		return rc;
	}

	/* Totals include address bytes on TX; each must fit a 16-bit count */
	if ((tx_tot > I2C_HW_MAX_COUNT) || (rx_tot > I2C_HW_MAX_COUNT)) {
		return -EINVAL;
	}
	desc->tx_count = (uint16_t)tx_tot;
	desc->rx_count = (uint16_t)rx_tot;
	return 0;
}
