/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * struct i2c_msg[] -> I2C-NL HW request parsing. No SoC/register
 * dependencies: kept in its own translation unit (source-included by
 * i2c_mchp_xec_nl.c, and directly by its native_sim ztest) so the
 * parsing rules can be exercised on host without a target board.
 */

/* I2C-NL HW request flags (see xec_i2c_nl_parse_msgs()) */
#define XEC_I2C_NL_REQ_START0 BIT(0) /* issue I2C START before the first write-phase byte */
#define XEC_I2C_NL_REQ_STARTN BIT(1) /* issue RPT-START before the last write-phase byte (write->read) */
#define XEC_I2C_NL_REQ_STOP   BIT(2) /* issue I2C STOP once write and read counts reach 0 */

/* Sentinel index: request has no write (or read) messages */
#define XEC_I2C_NL_NO_MSG_IDX UINT8_MAX

/*
 * One HW transaction built out of a run of struct i2c_msg entries.
 * write_count/read_count include the one-byte target address the HW
 * moves through the Master TX data register for START0 and, if
 * present, STARTN -- both are counted against the write side even
 * when the request is read-only or ends in a read phase.
 */
struct xec_i2c_nl_request {
	uint8_t first_msg_idx;
	uint8_t last_msg_idx;
	uint8_t first_write_msg_idx;
	uint8_t first_read_msg_idx;
	uint8_t num_write_msgs;
	uint8_t num_read_msgs;
	uint16_t write_count;
	uint16_t read_count;
	uint8_t flags;
};

/*
 * Parse msgs[start_idx .. num_msgs) into the next HW request the
 * I2C-NL engine can run in a single START0/[STARTN]/[STOP]. A request
 * is a run of write messages optionally followed by a run of read
 * messages -- the only direction change this HW supports mid-request
 * -- bounded by whichever comes first:
 *   - a message with I2C_MSG_STOP (included, then the request ends),
 *   - a direction change other than the one write->read turnaround,
 *   - an I2C_MSG_RESTART this HW has no address-byte slot left to
 *     honor (anything but that same write->read turnaround message),
 *   - or a write/read byte count that would exceed UINT16_MAX.
 * The caller re-parses the messages left after *req to build the
 * request(s) that follow, until num_msgs is exhausted.
 *
 * Returns the index of the last message included in *req (same as
 * req->last_msg_idx), or a negative errno if start_idx is out of
 * range or msgs[start_idx] alone cannot be serviced by any request
 * (its length alone already exceeds what the 16-bit HW count allows).
 */
static int xec_i2c_nl_parse_msgs(struct i2c_msg *msgs, uint8_t num_msgs, uint8_t start_idx,
				  struct xec_i2c_nl_request *req)
{
	bool have_write = false;
	bool have_read = false;
	uint16_t wr_count;
	uint16_t rd_count = 0U;
	uint8_t idx;

	if (!msgs || !req || start_idx >= num_msgs) {
		return -EINVAL;
	}

	*req = (struct xec_i2c_nl_request){
		.first_msg_idx = start_idx,
		.last_msg_idx = XEC_I2C_NL_NO_MSG_IDX,
		.first_write_msg_idx = XEC_I2C_NL_NO_MSG_IDX,
		.first_read_msg_idx = XEC_I2C_NL_NO_MSG_IDX,
		.flags = XEC_I2C_NL_REQ_START0,
	};
	wr_count = 1U; /* START0 address byte */

	for (idx = start_idx; idx < num_msgs; idx++) {
		struct i2c_msg *m = &msgs[idx];
		bool is_read = (m->flags & I2C_MSG_READ) != 0U;
		bool is_dir_change = is_read && !have_read && have_write;

		/*
		 * This HW has exactly one spare address-byte slot
		 * (STARTN) beyond START0, reserved for a write->read
		 * turnaround. Any other mid-request restart request
		 * ends the request here; msgs[idx] becomes the next
		 * request's first message.
		 */
		if (idx != start_idx && (m->flags & I2C_MSG_RESTART) && !is_dir_change) {
			break;
		}

		if (!is_read && have_read) {
			break; /* read -> write is not a supported turnaround */
		}

		if (m->len > UINT16_MAX) {
			break;
		}

		if (is_dir_change) {
			if (wr_count >= UINT16_MAX) {
				break;
			}
			wr_count++; /* STARTN address byte */
			req->flags |= XEC_I2C_NL_REQ_STARTN;
		}

		if (is_read) {
			if ((uint32_t)rd_count + m->len > UINT16_MAX) {
				break;
			}
			if (!have_read) {
				have_read = true;
				req->first_read_msg_idx = idx;
			}
			rd_count += (uint16_t)m->len;
			req->num_read_msgs++;
		} else {
			if ((uint32_t)wr_count + m->len > UINT16_MAX) {
				break;
			}
			if (!have_write) {
				have_write = true;
				req->first_write_msg_idx = idx;
			}
			wr_count += (uint16_t)m->len;
			req->num_write_msgs++;
		}

		req->last_msg_idx = idx;

		if (m->flags & I2C_MSG_STOP) {
			req->flags |= XEC_I2C_NL_REQ_STOP;
			break;
		}
	}

	if (req->last_msg_idx == XEC_I2C_NL_NO_MSG_IDX) {
		return -EMSGSIZE;
	}

	req->write_count = wr_count;
	req->read_count = rd_count;

	return req->last_msg_idx;
}
