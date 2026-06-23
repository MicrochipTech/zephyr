/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
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

int test_targ1_write(const struct device *host_port_ctrl, const struct i2c_dt_spec *targ_port_dts);
int test_targ2_read(const struct device *host_port_ctrl, const struct i2c_dt_spec *targ_port_dts);

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

	LOG_INF("Register target 1 on %s", targ1_spec.bus->name);
	rc = i2c_target_register(targ1_spec.bus, &targ1_cfg);
	if (rc == 0) {
		LOG_INF("PASS");
	} else {
		LOG_ERR("FAIL");
	}

	LOG_INF("Register target 2 on %s", targ2_spec.bus->name);
	rc = i2c_target_register(targ2_spec.bus, &targ2_cfg);
	if (rc == 0) {
		LOG_INF("PASS");
	} else {
		LOG_ERR("FAIL");
	}

	LOG_INF("Write from %s to addr 0x%02x on %s", mb_fram_spec.bus->name,
		targ1_spec.addr, targ1_spec.bus->name);

	test_targ1_write(mb_fram_spec.bus, &targ1_spec);
	test_targ2_read(mb_fram_spec.bus, &targ2_spec);

app_done:
	LOG_INF("Program End");
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

static int app_i2c_target_print(struct app_i2c_target *apptrg)
{
	if (apptrg == NULL) {
		return -EINVAL;
	}

	LOG_INF("App I2C target structure");
	LOG_INF("buf = %p  bufsz = %u", (void *)apptrg->buf, apptrg->bufsz);
	LOG_INF("idx = %u, wr_recv_cnt = %u, rd_req_cnt = %u", apptrg->idx, apptrg->wr_recv_cnt,
		apptrg->rd_req_cnt);
	LOG_INF("stop_cnt = %u  error_cnt = %u  err_reason = %u", apptrg->stop_cnt,
		apptrg->error_cnt, apptrg->err);

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

	/* TODO allow wrapping? Would a real I2C do wrap? */
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

	/* TODO do we need to maintain write index and read index?
	 * Or do we wrap?
	 */
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

	/* TODO do we need to maintain write index and read index?
	 * Or do we wrap?
	 */
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

int test_targ1_write(const struct device *host_port_ctrl, const struct i2c_dt_spec *targ_port_dts)
{
	int rc = 0;
	uint8_t buf[4] = {1U, 2U, 3U, 4U};

	rc = mchp_xec_i2c_nl_clear_capture(i2c_smb0_dev);
	if (rc != 0) {
		LOG_ERR("I2C-NL clear capture buffer for %s returned (%d)", i2c_smb0_dev->name, rc);
		return rc;
	}

	rc = mchp_xec_i2c_nl_clear_capture(i2c_smb1_dev);
	if (rc != 0) {
		LOG_ERR("I2C-NL clear capture buffer for %s returned (%d)", i2c_smb1_dev->name, rc);
		return rc;
	}

	k_sem_reset(&app_targ1_sem);

	rc = i2c_write(host_port_ctrl, buf, 4U, targ_port_dts->addr);
	if (rc != 0) {
		LOG_ERR("I2C write error (%d)", rc);
		return rc;
	}

	(void)k_sem_take(&app_targ1_sem, K_FOREVER);

	LOG_INF("Target 1 STOP received (app_targ1_sem)");

	app_i2c_target_print(&targ1_app_data);

	return 0;
}

int test_targ2_read(const struct device *host_port_ctrl, const struct i2c_dt_spec *targ_port_dts)
{
	int rc = 0;
	uint8_t buf[4] = {1U, 2U, 3U, 4U};

	rc = mchp_xec_i2c_nl_clear_capture(i2c_smb0_dev);
	if (rc != 0) {
		LOG_ERR("I2C-NL clear capture buffer for %s returned (%d)", i2c_smb0_dev->name, rc);
		return rc;
	}

	rc = mchp_xec_i2c_nl_clear_capture(i2c_smb1_dev);
	if (rc != 0) {
		LOG_ERR("I2C-NL clear capture buffer for %s returned (%d)", i2c_smb1_dev->name, rc);
		return rc;
	}

	k_sem_reset(&app_targ2_sem);

	rc = i2c_read(host_port_ctrl, buf, 4U, targ_port_dts->addr);
	if (rc != 0) {
		LOG_ERR("I2C read error (%d)", rc);
		return rc;
	}

	(void)k_sem_take(&app_targ2_sem, K_FOREVER);

	LOG_INF("Target 2 STOP received (app_targ2_sem)");

	app_i2c_target_print(&targ2_app_data);

	return 0;
}
