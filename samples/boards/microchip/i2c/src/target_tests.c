/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_xec_i2c.h>
#include <zephyr/logging/log.h>
#include "target_tests.h"

LOG_MODULE_DECLARE(app);

/* ISSUE:
 * CONFIG_I2C_TARGET and CONFIG_I2C_TARGET_BUFFER_MODE are global kconfig's
 *
 */
#if 0 /* unused at this time */
static const struct device *i2c1_dev = DEVICE_DT_GET(DT_ALIAS(i2c1));
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(microchip_xec_i2c_nl)
#define APP_HAS_I2C_NL
static const struct device *i2c_nl_dev = DEVICE_DT_GET(DT_ALIAS(i2c_nl));
#endif

static struct i2c_target_config i2c1_target;
static struct i2c_target_config i2c2_target;
static volatile uint32_t i2c1_target_wr_req_cb_cnt;
static volatile uint32_t i2c1_target_rd_req_cb_cnt;
static volatile uint32_t i2c1_target_wr_recv_cb_cnt;
static volatile uint32_t i2c1_target_rd_proc_cb_cnt;
static volatile uint32_t i2c1_target_stop_cb_cnt;
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
static volatile uint32_t i2c2_target_buf_wr_recv_cb_cnt;
static volatile uint32_t i2c2_target_buf_rd_req_cb_cnt;
static volatile uint32_t i2c2_target_stop_cb_cnt;
#endif

static int i2c1_target_write_requested_cb(struct i2c_target_config *config);
static int i2c1_target_read_requested_cb(struct i2c_target_config *config, uint8_t *val);
static int i2c1_target_write_received_cb(struct i2c_target_config *config, uint8_t val);
static int i2c1_target_read_processed_cb(struct i2c_target_config *config, uint8_t *val);
static int i2c1_target_stop_cb(struct i2c_target_config *config);

static int i2c2_target_stop_cb(struct i2c_target_config *config);

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE

#define I2C2_TM_HOST_WR_BUF_SIZE 128u
#define I2C2_TM_HOST_RD_BUF_SIZE 48u

uint8_t *i2c_tm_host_wr_buf_ptr;
uint8_t *i2c_tm_host_rd_buf_ptr;
uint8_t i2c_tm_host_wr_buf[I2C2_TM_HOST_WR_BUF_SIZE];
uint8_t i2c_tm_host_rd_buf[I2C2_TM_HOST_RD_BUF_SIZE];

static void i2c2_target_buf_write_received_cb(struct i2c_target_config *config, uint8_t *ptr,
					      uint32_t len);
static int i2c2_target_buf_read_requested_cb(struct i2c_target_config *config, uint8_t **ptr,
					     uint32_t *len);
#endif

const struct i2c_target_callbacks i2c_byte_mode_callbacks = {
	.write_requested = i2c1_target_write_requested_cb,
	.read_requested = i2c1_target_read_requested_cb,
	.write_received = i2c1_target_write_received_cb,
	.read_processed = i2c1_target_read_processed_cb,
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	.buf_write_received = NULL,
	.buf_read_requested = NULL,
#endif
	.stop = i2c1_target_stop_cb,
};

const struct i2c_target_callbacks i2c_buffer_mode_callbacks = {
	.write_requested = NULL,
	.read_requested = NULL,
	.write_received = NULL,
	.read_processed = NULL,
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	.buf_write_received = i2c2_target_buf_write_received_cb,
	.buf_read_requested = i2c2_target_buf_read_requested_cb,
#endif
	.stop = i2c2_target_stop_cb,
};

struct xec_i2c_info {
	const struct device *dev;
	uint32_t flags;
	uint8_t port;
};

#define XEC_I2C_V3_INFO(nid)                                                                       \
	{                                                                                          \
		.dev = DEVICE_DT_GET(nid),                                                         \
		.flags = 0,                                                                        \
		.port = DT_PROP(nid, port_sel),                                                    \
	},

#define XEC_I2C_NL_INFO(nid)                                                                       \
	{                                                                                          \
		.dev = DEVICE_DT_GET(nid),                                                         \
		.flags = BIT(0),                                                                   \
		.port = DT_PROP(nid, port_sel),                                                    \
	},

const struct xec_i2c_info i2c_xec_driver_devices[] = {
	DT_FOREACH_STATUS_OKAY(microchip_xec_i2c_v3, XEC_I2C_V3_INFO)
		DT_FOREACH_STATUS_OKAY(microchip_xec_i2c_nl, XEC_I2C_NL_INFO)};

const struct device *find_first_i2c_nl_device(void)
{
	for (size_t n = 0; n < ARRAY_SIZE(i2c_xec_driver_devices); n++) {
		const struct xec_i2c_info *p = &i2c_xec_driver_devices[n];

		if (p->flags & BIT(0)) {
			return p->dev;
		}
	}

	return NULL;
}

const struct xec_i2c_info *find_i2c_info(const struct device *i2c_dev)
{
	if (i2c_dev == NULL) {
		return NULL;
	}

	for (size_t n = 0; n < ARRAY_SIZE(i2c_xec_driver_devices); n++) {
		const struct xec_i2c_info *p = &i2c_xec_driver_devices[n];

		if (p->dev == i2c_dev) {
			return p;
		}
	}

	return NULL;
}

void clear_target_cb_counts(void)
{
	i2c1_target_wr_req_cb_cnt = 0;
	i2c1_target_rd_req_cb_cnt = 0;
	i2c1_target_wr_recv_cb_cnt = 0;
	i2c1_target_rd_proc_cb_cnt = 0;
	i2c1_target_stop_cb_cnt = 0;
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	i2c2_target_buf_wr_recv_cb_cnt = 0;
	i2c2_target_buf_rd_req_cb_cnt = 0;
	i2c2_target_stop_cb_cnt = 0;
#endif
}

void reset_target_read_buf(void)
{
	i2c_tm_host_rd_buf_ptr = i2c_tm_host_rd_buf;
}

void reset_target_write_buf(void)
{
	i2c_tm_host_wr_buf_ptr = i2c_tm_host_wr_buf;
}

int target_config(const struct device *i2c_dev, uint16_t targ_addr)
{
	struct i2c_target_config *tc = NULL;
	const struct xec_i2c_info *p = NULL;
	size_t n = 0;

	if (targ_addr > 0x7Fu) {
		LOG_ERR("MEC I2C HW does not support 10-bit I2C addressing");
		return -EINVAL;
	}

	for (n = 0; n < ARRAY_SIZE(i2c_xec_driver_devices); n++) {
		p = &i2c_xec_driver_devices[n];

		if (p->dev == i2c_dev) {
			if ((p->flags & BIT(0)) != 0) {
				tc = &i2c2_target;
				tc->callbacks = &i2c_buffer_mode_callbacks;
			} else {
				tc = &i2c1_target;
				tc->callbacks = &i2c_byte_mode_callbacks;
			}
			break;
		}
	}

	if (tc == NULL) {
		return -EINVAL;
	}

	tc->address = targ_addr;
	tc->flags = 0; /* HW does not support 10-bit I2C addresses */

	return i2c_target_register(i2c_dev, tc);
}

int i2c_nl_target_test1(const struct device *host_i2c_dev, const struct device *targ_i2c_dev,
			uint16_t targ_i2c_addr)
{
	int rc = 0;
	const uint8_t buf[16] = {0x11u, 0x12u, 0x13u, 0x14u, 0x15u, 0x16u, 0x17u, 0x18u,
				 0x19u, 0x1Au, 0x1Bu, 0x1Cu, 0x1Du, 0x1Eu, 0x1Fu, 0x20u};

	i2c_mchp_xec_nl_debug_init(targ_i2c_dev);

	rc = i2c_write(host_i2c_dev, buf, 16, targ_i2c_addr);
	LOG_INF("i2c write %u bytes to I2C target address 0x%0x returned (%d)", 16, targ_i2c_addr,
		rc);

	return rc;
}

int i2c_nl_target_test2(const struct device *host_i2c_dev, const struct device *targ_i2c_dev,
			uint16_t targ_i2c_addr)
{
	int rc = 0;
	const uint8_t wrbuf[2] = {0x20, 0x00u};
	uint8_t rdbuf[4] = {0};

	i2c_mchp_xec_nl_debug_init(targ_i2c_dev);

	rc = i2c_write_read(host_i2c_dev, targ_i2c_addr, wrbuf, 2, rdbuf, 4u);
	LOG_INF("i2c write-read at address 0x%0x nwr=2 nr=4 returned (%d)", targ_i2c_addr, rc);

	return rc;
}

int i2c_nl_target_test3(const struct device *host_i2c_dev, const struct device *targ_i2c_dev,
			uint16_t targ_i2c_addr)
{
	int rc = 0;
	uint32_t nread = 16u;
	uint8_t rdbuf[16] = {0x55u};

	i2c_mchp_xec_nl_debug_init(targ_i2c_dev);

	rc = i2c_read(host_i2c_dev, rdbuf, nread, targ_i2c_addr);
	LOG_INF("i2c read of %u bytes at address 0x%0x returned (%d)", nread, targ_i2c_addr, rc);

	rc = memcmp(rdbuf, i2c_tm_host_rd_buf, nread);
	LOG_INF("i2c read of %u bytes data match result(0 is match): (%d)", nread, rc);

	return rc;
}

int get_i2c_port(const struct device *i2c_dev, uint8_t *port)
{
#ifdef CONFIG_I2C_XEC_PORT_MUX
	uint32_t i2c_cfg = 0;

	if ((i2c_dev == NULL) || (i2c_get_config(i2c_dev, &i2c_cfg) != 0)) {
		return -EINVAL;
	}

	*port = I2C_XEC_PORT_GET(i2c_cfg);
#else
	const struct xec_i2c_info *p = find_i2c_info(i2c_dev);

	if ((p == NULL) || (port == NULL)) {
		return -EINVAL;
	}

	*port = p->port;
#endif
	return 0;
}

int target_i2c_nl_prepare_tests(const struct device *i2c_host_dev)
{
	int rc = 0;
	uint32_t host_i2c_cfg = 0, targ_i2c_cfg = 0, new_cfg = 0;
	uint32_t host_speed = 0, targ_speed = 0, desired_speed = 0;
	uint8_t host_port = 0, targ_port = 0;
	bool reconfig = false;

	if (i2c_host_dev == NULL) {
		return -EINVAL;
	}

	i2c_get_config(i2c_host_dev, &host_i2c_cfg);
	i2c_get_config(i2c_nl_dev, &targ_i2c_cfg);

	host_speed = I2C_SPEED_GET(host_i2c_cfg);
	targ_speed = I2C_SPEED_GET(targ_i2c_cfg);

	get_i2c_port(i2c_host_dev, &host_port);
	get_i2c_port(i2c_nl_dev, &targ_port);

	if (host_speed != targ_speed) {
		reconfig = true;
		desired_speed = MIN(host_speed, targ_speed);
		new_cfg |= I2C_SPEED_SET(desired_speed);
	} else {
		new_cfg |= I2C_SPEED_SET(host_speed);
	}

#ifdef CONFIG_I2C_XEC_PORT_MUX
	new_cfg |= I2C_XEC_PORT_SET((uint32_t)targ_port);
	if (host_port != targ_port) {
		reconfig = true;
	}
#else
	if (host_port != targ_port) {
		LOG_ERR("Host and target I2C ports are different and no port mux support");
		return -EIO;
	}
#endif

	if (reconfig == true) {
		LOG_INF("Reconfiguration required!");
		new_cfg |= I2C_MODE_CONTROLLER;
		rc = i2c_configure(i2c_host_dev, new_cfg);
		if (rc != 0) {
			LOG_ERR("Host I2C reconfig error (%d)", rc);
			return rc;
		}
		rc = i2c_configure(i2c_nl_dev, new_cfg);
		if (rc != 0) {
			LOG_ERR("Target I2C reconfig error (%d)", rc);
			return rc;
		}
	}

	rc = target_config(i2c_nl_dev, I2C2_TARG_ADDR1);
	if (rc != 0) {
		LOG_ERR("Register address 0x%0x with I2C-NL failed (%d)", I2C2_TARG_ADDR1, rc);
		return rc;
	}

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	i2c_tm_host_wr_buf_ptr = i2c_tm_host_wr_buf;
	i2c_tm_host_rd_buf_ptr = i2c_tm_host_rd_buf;

	memset(i2c_tm_host_wr_buf, 0x55, I2C2_TM_HOST_WR_BUF_SIZE);
	for (uint32_t n = 0; n < I2C2_TM_HOST_RD_BUF_SIZE; n++) {
		i2c_tm_host_rd_buf[n] = (uint8_t)(n & 0xffu);
	}
#endif
	return 0;
}

int target_i2c_nl_run_tests(const struct device *i2c_host_dev)
{
	int rc = 0;

	if (i2c_host_dev == NULL) {
		return -EINVAL;
	}

	clear_target_cb_counts();
	i2c_mchp_xec_nl_debug_init(i2c_nl_dev);

	rc = i2c_nl_target_test1(i2c_host_dev, i2c_nl_dev, I2C2_TARG_ADDR1);
	LOG_INF("I2C-NL target test 1 returned (%d)", rc);

	clear_target_cb_counts();
	reset_target_read_buf();
	reset_target_write_buf();
	i2c_mchp_xec_nl_debug_init(i2c_nl_dev);

	rc = i2c_nl_target_test2(i2c_host_dev, i2c_nl_dev, I2C2_TARG_ADDR1);
	LOG_INF("I2C-NL target test 2 returned (%d)", rc);

	clear_target_cb_counts();
	reset_target_read_buf();
	reset_target_write_buf();
	i2c_mchp_xec_nl_debug_init(i2c_nl_dev);

	rc = i2c_nl_target_test3(i2c_host_dev, i2c_nl_dev, I2C2_TARG_ADDR1);
	LOG_INF("I2C-NL target test 3 returned (%d)", rc);

	return rc;
}

/* Invoked by I2C driver when a START with matching target write address is received.
 * Return 0 to inform the driver to the transaction. HW has auto-ACK'd the address..
 * Return <0(error) informing the driver to not respond. The driver will clear auto-ACK
 * so future bytes transmitted by the external Host will be NACK'd. No callbacks on
 * data bytes will be invoked. Driver will invoke STOP callback.
 */
static int i2c1_target_write_requested_cb(struct i2c_target_config *config)
{
	i2c1_target_wr_req_cb_cnt++;

	return 0; /* TODO */
}

/* Invoked by I2C driver when a START with matching target read address is receivied.
 * Return 0 with val = data byte for external Host if app accepts the read request.
 * Return < 0(error code) if app wants driver to not respond. In our case HW address
 * matching has ACK'd the address. Our driver will supply 0xFF as data and not make
 * further callbacks until external Host issues STOP. Driver will invoke the STOP
 * callback.
 */
static int i2c1_target_read_requested_cb(struct i2c_target_config *config, uint8_t *val)
{
	i2c1_target_rd_req_cb_cnt++;

	return 0; /* TODO */
}

/* Invokded by I2C driver when a data byte is received and ACK'd by the driver.
 * Return 0 informs the driver to ACK the next data byte from the external Host.
 * Return <0(error) informing the driver to NACK all future data bytes. No more
 *        data callbacks will be made. Expect driver to issue STOP callback when
 *        external Host gives up and issues STOP.
 */
static int i2c1_target_write_received_cb(struct i2c_target_config *config, uint8_t val)
{
	i2c1_target_wr_recv_cb_cnt++;

	return 0; /* TODO */
}

/* Invokded by I2C driver when it needs a data byte to service external Host read.
 * Return 0 with val pointing to data for driver to provide to external Host.
 * Return <0(error) indicating no more data for external Host. Driver will supply
 *        0xFF to external Host. Driver will make this callback again. Only STOP
 *        callback will be issued.
 */
static int i2c1_target_read_processed_cb(struct i2c_target_config *config, uint8_t *val)
{
	i2c1_target_rd_proc_cb_cnt++;

	return 0; /* TODO */
}

/* Invoked by the driver when it detects STOP generated by external Host
 * Should the driver issue this callback on other conditions such as:
 * I2C bus error or I2C lost arbitration? This is the only callback informing
 * the application the current transfer has ended.
 * Driver ignores the return value per i2c.h documentation.
 */
static int i2c1_target_stop_cb(struct i2c_target_config *config)
{
	i2c1_target_stop_cb_cnt++;

	return 0;
}

static int i2c2_target_stop_cb(struct i2c_target_config *config)
{
	i2c2_target_stop_cb_cnt++;

	return 0;
}

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE

/* Invoked by the driver upon reception of data from the external Host when:
 * drivers internal buffer is full or STOP received before internal buffer is full.
 * Driver passes a pointer to its internal buffer and length of data received.
 * The app MUST copy this data or process it before returning. Upon return, the
 * driver will stop clock stretching allowing the external Host to continue the
 * transaction.
 */
static void i2c2_target_buf_write_received_cb(struct i2c_target_config *config, uint8_t *ptr,
					      uint32_t len)
{
	uint32_t ncopy = 0;

	i2c2_target_buf_wr_recv_cb_cnt++;

	ncopy = sizeof(i2c_tm_host_wr_buf) -
		((uint32_t)(i2c_tm_host_wr_buf_ptr - i2c_tm_host_wr_buf));

	if (len < ncopy) {
		ncopy = len;
	}

	memcpy(i2c_tm_host_wr_buf_ptr, ptr, ncopy);
	i2c_tm_host_wr_buf_ptr += ncopy;
}

/* Invoked by the driver when it needs data to supply an ongoing Host read.
 * Return 0 and supply pointer to app buffer and fill in length. Driver uses
 * the buffer to supply bytes to Host.
 * Return <0 if app has no more data. Driver supply dummy data to external
 * Host until the Host issues STOP.
 *
 */
static int i2c2_target_buf_read_requested_cb(struct i2c_target_config *config, uint8_t **ptr,
					     uint32_t *len)
{
	uint8_t *buf_end = i2c_tm_host_rd_buf + sizeof(i2c_tm_host_rd_buf);
	uint32_t rem_buf_len = 0;

	i2c2_target_buf_rd_req_cb_cnt++;

	if ((ptr == NULL) || (len == NULL)) {
		return -EINVAL;
	}

	if (i2c_tm_host_rd_buf_ptr < buf_end) {
		rem_buf_len = (uint32_t)(buf_end - i2c_tm_host_rd_buf_ptr);
	}

	if (rem_buf_len != 0) {
		*ptr = i2c_tm_host_rd_buf_ptr;
		*len = rem_buf_len;
		i2c_tm_host_rd_buf_ptr += rem_buf_len;
	} else {
		*ptr = NULL;
		*len = 0u;
	}

	return 0;
}
#endif /* CONFIG_I2C_TARGET_BUFFER_MODE */
