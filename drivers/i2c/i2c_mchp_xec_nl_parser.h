/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip XEC I2C-NL message parser. Converts one or two struct i2c_msg
 * into a request the I2C-NL HW FSM executes as one START to STOP transaction.
 * Kept free of SoC dependencies so it can be tested on native_sim.
 */
#ifndef ZEPHYR_DRIVERS_I2C_I2C_MCHP_XEC_NL_PARSER_H_
#define ZEPHYR_DRIVERS_I2C_I2C_MCHP_XEC_NL_PARSER_H_

#include <stdint.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>

#include "i2c_mchp_xec_regs.h"

/* Host command register START0, STARTN, and STOP bits */
#define I2C_HW_CTRL_START     BIT(XEC_I2C_HCMD_START0_POS)
#define I2C_HW_CTRL_RPT_START BIT(XEC_I2C_HCMD_STARTN_POS)
#define I2C_HW_CTRL_STOP      BIT(XEC_I2C_HCMD_STOP_POS)

/* XEC I2C-NL HW implements 16-bit write and read count register fields.
 * The driver uses XEC DMA driver block chaining to handle the maximum two
 * messages. Maximum number of TX blocks is 3 for a two msg I2C Write-Read:
 * DMA block[0] START address, len=1
 * DMA block[1] msg[0].buf, msg[0].len for data write phase
 * DMA block[2] RPT-START address, len=1
 * followed by one RX block msg[1].buf, msg[1].len for the data read phase.
 * MAX_RX_SEGS of 2 is only used for a two msg read sequence.
 */
#define I2C_HW_MAX_COUNT     UINT16_MAX
#define I2C_XFER_MAX_TX_SEGS 3
#define I2C_XFER_MAX_RX_SEGS 2

enum i2c_xfer_kind {
	I2C_XFER_WRITE,
	I2C_XFER_READ,
	I2C_XFER_WRITE_WRITE,
	I2C_XFER_READ_READ,
	I2C_XFER_WRITE_READ,
};

struct i2c_xfer_seg {
	uint8_t *buf;
	uint32_t len;
};

struct i2c_xfer_desc {
	enum i2c_xfer_kind kind;
	uint16_t addr;          /* 7-bit target address */
	uint8_t addr_wr;        /* (addr << 1) | 0 -- DMA source, keep in place */
	uint8_t addr_rd;        /* (addr << 1) | 1 -- DMA source, keep in place */
	uint8_t rx_discard;     /* DMA destination of a read ping data byte */

	struct i2c_xfer_seg tx[I2C_XFER_MAX_TX_SEGS];
	uint8_t tx_nseg;
	struct i2c_xfer_seg rx[I2C_XFER_MAX_RX_SEGS];
	uint8_t rx_nseg;

	uint16_t tx_count;      /* value for TX data count register */
	uint16_t rx_count;      /* value for RX data count register */
	uint32_t ctrl;          /* I2C_HW_CTRL_* bits to set before GO */
	uint32_t elen;          /* I2C ExtLen register before GO */
};

/*
 * Due to the I2C-NL HW FSM the driver accepts a maximum of two struct i2c_msg
 * forming one START to STOP transaction. Allowed message sequences are:
 * One msg: I2C_MSG_WRITE | I2C_MSG_STOP. Zero length is an I2C write ping
 *   (address only, buf may be NULL).
 * One msg: I2C_MSG_READ | I2C_MSG_STOP. buf == NULL with len == 1 is an I2C read
 *   ping: one byte is read and discarded.
 * Two msgs, same direction, data split across two buffers:
 *   msg0.flags = I2C_MSG_WRITE, msg1.flags = I2C_MSG_WRITE | I2C_MSG_STOP
 *   msg0.flags = I2C_MSG_READ, msg1.flags = I2C_MSG_READ | I2C_MSG_STOP
 * Two msgs, I2C combined write-read:
 *   msg0.flags = I2C_MSG_WRITE, msg1.flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP
 *
 * Returns 0 and fills desc on success, -EINVAL for invalid arguments or message
 * buffers/lengths, -ENOTSUP for a message sequence the HW cannot execute.
 * Safe to call from ISR context: no library calls.
 */
int xec_i2c_nl_xfer_parse(const struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr,
			  struct i2c_xfer_desc *desc);

#endif /* ZEPHYR_DRIVERS_I2C_I2C_MCHP_XEC_NL_PARSER_H_ */
