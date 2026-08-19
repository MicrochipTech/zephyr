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
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/dt-bindings/i2c/mchp-xec-i2c.h>
#include <zephyr/dt-bindings/pinctrl/mchp-xec-pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/pm/device.h>

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define I2C_SMB_GET_DEV(nid) DEVICE_DT_GET(nid),

#define I2C_CTRL0_NODE DT_ALIAS(i2c0)
#define I2C_CTRL1_NODE DT_ALIAS(i2c1)
#define I2C_CTRL2_NODE DT_ALIAS(i2c2)

#define I2C_PORT0_NODE DT_ALIAS(i2cport0)
#define I2C_PORT3_NODE DT_ALIAS(i2cport3)
#define I2C_PORT7_NODE DT_ALIAS(i2cport7)

/* Target nodes */
#define NODE_I2C_TARG1 DT_NODELABEL(i2c_targ1)
#define NODE_I2C_TARG2 DT_NODELABEL(i2c_targ2)
#define NODE_I2C_TARG3 DT_NODELABEL(i2c_targ3)
#define NODE_I2C_TARG4 DT_NODELABEL(i2c_targ4)

/* FRAM target node */
#define NODE_FRAM DT_NODELABEL(mb85rc256v_fram)

/* ---- iteration helpers: both controller kinds and both port kinds coexist -- */

#define I2C_CTRL_FOREACH(fn)                                                    \
	DT_FOREACH_STATUS_OKAY(microchip_xec_i2c_v3_bm, fn)                     \
	DT_FOREACH_STATUS_OKAY(microchip_xec_i2c_v3_nl, fn)

#define I2C_PORT_FOREACH_VARGS(fn, ...)                                             \
	DT_FOREACH_STATUS_OKAY_VARGS(microchip_xec_i2c_v3_bm_port, fn, __VA_ARGS__) \
	DT_FOREACH_STATUS_OKAY_VARGS(microchip_xec_i2c_v3_nl_port, fn, __VA_ARGS__)

#define I2C_SMB_MAX_INSTANCES 5
#define I2C_SMB_BASE          0x40004000u
#define I2C_SMB_STRIDE        0x400u

/* HW controller index 0..4, a compile-time constant usable as an array slot.
 * Equals the second arg of girqs = <MCHP_XEC_ECIA_GIRQ_ENC(13, N)>.
 */
#define I2C_SMB_INST(ctrl) ((DT_REG_ADDR(ctrl) - I2C_SMB_BASE) / I2C_SMB_STRIDE)

struct i2c_port_info {
	const struct device *dev;   /* the I2C bus device (i2c_driver_api)     */
	const struct device *ctrl;  /* back-pointer to the owning HW block     */
	uint8_t              port;  /* physical port index 0..15               */
	bool                 is_default; /* pre-routed at boot?                */
};

struct i2c_ctrl_info {
	const struct device        *ctrl;      /* NULL => instance not present */
	uint8_t                     inst;      /* 0..4 (SMB number)            */
	bool                        wakeup_source;
	const struct device        *wakeup_dev; /* NULL unless wakeup-device set */
	const struct i2c_port_info *ports;
	size_t                      num_ports; /* excludes the sentinel        */
};

/* Mirrors the driver's guard: only compare if the ctrl actually names one. */
#define PORT_IS_DEFAULT(ctrl, port)                                            \
	COND_CODE_1(DT_NODE_HAS_PROP(ctrl, default_port),                      \
		    (DT_SAME_NODE(port, DT_PHANDLE(ctrl, default_port))), (0))

/* Emit one i2c_port_info initializer iff `port` belongs to `ctrl`.
 * `& 0x0F`: the `port` prop is sometimes raw (<0>,<7>) and sometimes
 * MCHP_XEC_I2C_CTRL_PORT(ctrl,port) with ctrl in bits[7:4] — mask as the
 * driver does.
 */
#define PORT_ENTRY_IF_ON_CTRL(port_node, ctrl_node)                             \
	COND_CODE_1(DT_SAME_NODE(DT_PHANDLE(port_node, controller), ctrl_node), \
		({                                                              \
			.dev        = DEVICE_DT_GET(port_node),                 \
			.ctrl       = DEVICE_DT_GET(ctrl_node),                 \
			.port       = DT_PROP(port_node, port) & 0x0F,          \
			.is_default = PORT_IS_DEFAULT(ctrl_node, port_node),    \
		},), ())

/* Always-present terminator: keeps the port array non-empty even for a
 * controller with zero enabled ports (no zero-length-array GCC extension),
 * and NULL-marks the end. Excluded from num_ports via -1 below.
 */
#define I2C_PORT_SENTINEL { .dev = NULL }

#define CTRL_PORTS_NAME(ctrl) _CONCAT(i2c_ctrl_ports_, DT_DEP_ORD(ctrl))

/* Today the driver has only the `wakeup-source` boolean; this resolves a
 * `wakeup-device` phandle if you add one to the controller binding later.
 */
#define CTRL_WAKEUP_DEV(ctrl)                                                  \
	COND_CODE_1(DT_NODE_HAS_PROP(ctrl, wakeup_device),                     \
		    (DEVICE_DT_GET(DT_PHANDLE(ctrl, wakeup_device))), (NULL))

/* ---- generation passes ---------------------------------------------------- */

/* Pass 1: assert each reg base lands on the SMB grid, so I2C_SMB_INST() is a
 * valid, in-range slot. Catches a base drifted off 0x40004000 / 0x400 before
 * it silently misplaces a table entry.
 */
#define GEN_CTRL_INST_ASSERT(ctrl)                                                 \
	BUILD_ASSERT(DT_REG_ADDR(ctrl) >= I2C_SMB_BASE &&                          \
		     ((DT_REG_ADDR(ctrl) - I2C_SMB_BASE) % I2C_SMB_STRIDE) == 0 && \
		     I2C_SMB_INST(ctrl) < I2C_SMB_MAX_INSTANCES,                   \
		     "I2C SMB controller reg base off the expected "               \
		     "0x40004000 / 0x400 grid — I2C_SMB_INST() slot invalid");

/* Pass 2: per-controller port array (named by DT_DEP_ORD), always >= 1 elem. */
#define GEN_PORTS_ARRAY(ctrl)                                                 \
	static const struct i2c_port_info CTRL_PORTS_NAME(ctrl)[] = {         \
		I2C_PORT_FOREACH_VARGS(PORT_ENTRY_IF_ON_CTRL, ctrl)           \
		I2C_PORT_SENTINEL,                                            \
	};

/* Pass 3: table entry at its instance slot via designated initializer, so
 * FOREACH visit order is irrelevant and disabled instances stay NULL. Real
 * port count excludes the sentinel.
 */
#define GEN_CTRL_ENTRY_INDEXED(ctrl_node)                                    \
	[I2C_SMB_INST(ctrl_node)] = {                                        \
		.ctrl          = DEVICE_DT_GET(ctrl_node),                   \
		.inst          = I2C_SMB_INST(ctrl_node),                    \
		.wakeup_source = DT_PROP_OR(ctrl_node, wakeup_source, 0),    \
		.wakeup_dev    = CTRL_WAKEUP_DEV(ctrl_node),                 \
		.ports         = CTRL_PORTS_NAME(ctrl_node),                 \
		.num_ports     = ARRAY_SIZE(CTRL_PORTS_NAME(ctrl_node)) - 1, \
	},

/* GPIO pin info */
#define XEC_GPIO_BASE 0x40081000U

#define XEC_PIN_CTRL_REG_ADDR(port, pin_in_port) \
	(XEC_GPIO_BASE + ((port) * 0x80U) + ((pin_in_port) * 4U))

#define XEC_GET_PIN_ID(pinctrl_val) (((pinctrl_val) >> 16) & 0x3FFU)

struct i2c_gpio_pins {
	uint32_t p1_cr1;
	uint32_t p2_cr1;
};

/* Build table of I2C controllers where each entry has pointer to
 * list of connected ports.
 */
I2C_CTRL_FOREACH(GEN_CTRL_INST_ASSERT)   /* assert first             */
I2C_CTRL_FOREACH(GEN_PORTS_ARRAY)        /* then the per-ctrl arrays */

static const struct i2c_ctrl_info i2c_ctrls[I2C_SMB_MAX_INSTANCES] = {
      I2C_CTRL_FOREACH(GEN_CTRL_ENTRY_INDEXED)   /* then the table */
};

static inline const struct i2c_ctrl_info *i2c_ctrl_by_inst(unsigned int n);
static const struct i2c_ctrl_info *i2c_ctrl_for_port(const struct device *port,
						     const struct i2c_port_info **port_out);
static inline int i2c_inst_for_port(const struct device *port);
static void i2c_topology_dump(void);


/* I2C pins needed to check PM suspend */
PINCTRL_DT_DEFINE(ZEPHYR_USER_NODE);

PINCTRL_DT_DEV_CONFIG_DECLARE(I2C_PORT0_NODE);
PINCTRL_DT_DEV_CONFIG_DECLARE(I2C_PORT3_NODE);
PINCTRL_DT_DEV_CONFIG_DECLARE(I2C_PORT7_NODE);

const struct pinctrl_dev_config *i2c_port0_pincfg = PINCTRL_DT_DEV_CONFIG_GET(I2C_PORT0_NODE);
const struct pinctrl_dev_config *i2c_port3_pincfg = PINCTRL_DT_DEV_CONFIG_GET(I2C_PORT3_NODE);
const struct pinctrl_dev_config *i2c_port7_pincfg = PINCTRL_DT_DEV_CONFIG_GET(I2C_PORT7_NODE);

/* i2c_dt_spec.bus is the port controller node */
const struct i2c_dt_spec mb_fram_spec = I2C_DT_SPEC_GET(NODE_FRAM);
const struct i2c_dt_spec targ1_spec = I2C_DT_SPEC_GET(NODE_I2C_TARG1);
const struct i2c_dt_spec targ2_spec = I2C_DT_SPEC_GET(NODE_I2C_TARG2);
const struct i2c_dt_spec targ3_spec = I2C_DT_SPEC_GET(NODE_I2C_TARG3);
const struct i2c_dt_spec targ4_spec = I2C_DT_SPEC_GET(NODE_I2C_TARG4);

const struct device *i2c_ctrl0_dev = DEVICE_DT_GET(I2C_CTRL0_NODE);
const struct device *i2c_ctrl1_dev = DEVICE_DT_GET(I2C_CTRL1_NODE);
const struct device *i2c_ctrl2_dev = DEVICE_DT_GET(I2C_CTRL2_NODE);

const struct device *i2c_p0_dev = DEVICE_DT_GET(I2C_PORT0_NODE);
const struct device *i2c_p3_dev = DEVICE_DT_GET(I2C_PORT3_NODE);
const struct device *i2c_p7_dev = DEVICE_DT_GET(I2C_PORT7_NODE);

const bool i2c_ctrl0_is_wakeup_source = DT_PROP(I2C_CTRL0_NODE, wakeup_source);
const bool i2c_ctrl1_is_wakeup_source = DT_PROP(I2C_CTRL1_NODE, wakeup_source);
const bool i2c_ctrl2_is_wakeup_source = DT_PROP(I2C_CTRL2_NODE, wakeup_source);

#define I2C_MAX_MSGS 8

/* Target-buffer-size from DT for smb_0 (the target controller). Tests
 * that intentionally exceed the HW receive budget reference this. The
 * buffer-mode driver stages up to this many bytes before delivering
 * them via buf_write_received; the buffer-fill test writes one more.
 */
#define APP_TARG_HW_RX_SIZE       DT_PROP(DT_NODELABEL(i2c_smb_0), target_buffer_size)
#define APP_TARG_HW_DATA_CAPACITY (APP_TARG_HW_RX_SIZE - 1U)

/* Host TX/RX scratch. The buffer-fill test drives APP_TARG_HW_DATA_CAPACITY + 1
 * (== APP_TARG_HW_RX_SIZE) bytes out of i2c_tx_buf, so the scratch must be at
 * least that large or the test skips itself with -ENOSPC. Derive the size from
 * DT with a 256-byte floor so it tracks any target-buffer-size while preserving
 * the original headroom the fixed-size tests were written against.
 */
#define I2C_TX_BUF_SIZE MAX(256, APP_TARG_HW_RX_SIZE)
#define I2C_RX_BUF_SIZE MAX(256, APP_TARG_HW_RX_SIZE)

struct app_i2c_target {
	uint8_t *buf;
	size_t bufsz;
	size_t idx;    /* read-side cursor (start of next buf_read_requested) */
	size_t wr_idx; /* write-side cursor; advances each buf_write_received
			* so streaming-mode multi-chunk delivery accumulates
			* contiguously into the application buffer. Tracked
			* separately from idx so the read-side semantics
			* (start at the beginning of the buffer after reset)
			* are unaffected by write activity in tests that mix
			* directions on the same target.
			*/
	uint32_t wr_recv_cnt;
	uint32_t rd_req_cnt;
	uint32_t stop_cnt;
	uint32_t error_cnt;
	enum i2c_error_reason err;
	size_t nack_after; /* byte-mode only: NACK once this many write bytes
			    * have been accepted (0 = never). Used by the
			    * byte-mode overflow test; unused in buffer mode.
			    */
};

static struct k_sem app_targ1_sem;
static struct k_sem app_targ2_sem;
static struct k_sem app_targ3_sem;
static struct k_sem app_targ4_sem;

static struct i2c_msg msgs[I2C_MAX_MSGS];
static uint8_t i2c_tx_buf[I2C_TX_BUF_SIZE];
static uint8_t i2c_rx_buf[I2C_RX_BUF_SIZE];

/* targ1 is the buffer-fill target: its application buffer must hold a
 * full HW-buffer delivery (up to APP_TARG_HW_RX_SIZE bytes -- the byte-
 * mode driver stages data only, so it can deliver the whole staging
 * buffer), and the buffer-fill test verifies APP_TARG_HW_DATA_CAPACITY
 * of those bytes. Track target-buffer-size with a 256-byte floor.
 */
#define APP_TARG1_BUF_SIZE MAX(256, APP_TARG_HW_RX_SIZE)
#define APP_TARG2_BUF_SIZE 64
#define APP_TARG3_BUF_SIZE 64
#define APP_TARG4_BUF_SIZE 64

static uint8_t targ1_buf[APP_TARG1_BUF_SIZE];
static uint8_t targ2_buf[APP_TARG2_BUF_SIZE];
static uint8_t targ3_buf[APP_TARG3_BUF_SIZE];
static uint8_t targ4_buf[APP_TARG4_BUF_SIZE];

/* stop/error are mode-agnostic and shared by both callback flavors. */
static int targ1_stop_cb(struct i2c_target_config *config);
static void targ1_error_cb(struct i2c_target_config *config, enum i2c_error_reason error_code);
static int targ2_stop_cb(struct i2c_target_config *config);
static void targ2_error_cb(struct i2c_target_config *config, enum i2c_error_reason error_code);
static int targ3_stop_cb(struct i2c_target_config *config);
static void targ3_error_cb(struct i2c_target_config *config, enum i2c_error_reason error_code);
static int targ4_stop_cb(struct i2c_target_config *config);
static void targ4_error_cb(struct i2c_target_config *config, enum i2c_error_reason error_code);

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
static void targ1_buf_wr_recv_cb(struct i2c_target_config *config, uint8_t *ptr, uint32_t len);
static int targ1_buf_rd_req_cb(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len);
static void targ2_buf_wr_recv_cb(struct i2c_target_config *config, uint8_t *ptr, uint32_t len);
static int targ2_buf_rd_req_cb(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len);
static void targ3_buf_wr_recv_cb(struct i2c_target_config *config, uint8_t *ptr, uint32_t len);
static int targ3_buf_rd_req_cb(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len);
static void targ4_buf_wr_recv_cb(struct i2c_target_config *config, uint8_t *ptr, uint32_t len);
static int targ4_buf_rd_req_cb(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len);
#else
static int targ1_wr_req_cb(struct i2c_target_config *config);
static int targ1_wr_recv_byte_cb(struct i2c_target_config *config, uint8_t val);
static int targ1_rd_req_byte_cb(struct i2c_target_config *config, uint8_t *val);
static int targ1_rd_proc_cb(struct i2c_target_config *config, uint8_t *val);
static int targ2_wr_req_cb(struct i2c_target_config *config);
static int targ2_wr_recv_byte_cb(struct i2c_target_config *config, uint8_t val);
static int targ2_rd_req_byte_cb(struct i2c_target_config *config, uint8_t *val);
static int targ2_rd_proc_cb(struct i2c_target_config *config, uint8_t *val);
#endif

const struct i2c_target_callbacks targ1_callbacks = {
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	.buf_write_received = targ1_buf_wr_recv_cb,
	.buf_read_requested = targ1_buf_rd_req_cb,
#else
	.write_requested = targ1_wr_req_cb,
	.write_received = targ1_wr_recv_byte_cb,
	.read_requested = targ1_rd_req_byte_cb,
	.read_processed = targ1_rd_proc_cb,
#endif
	.stop = targ1_stop_cb,
	.error = targ1_error_cb,
};

const struct i2c_target_callbacks targ2_callbacks = {
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	.buf_write_received = targ2_buf_wr_recv_cb,
	.buf_read_requested = targ2_buf_rd_req_cb,
#else
	.write_requested = targ2_wr_req_cb,
	.write_received = targ2_wr_recv_byte_cb,
	.read_requested = targ2_rd_req_byte_cb,
	.read_processed = targ2_rd_proc_cb,
#endif
	.stop = targ2_stop_cb,
	.error = targ2_error_cb,
};

const struct i2c_target_callbacks targ3_callbacks = {
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	.buf_write_received = targ3_buf_wr_recv_cb,
	.buf_read_requested = targ3_buf_rd_req_cb,
#else
	.write_requested = targ3_wr_req_cb,
	.write_received = targ3_wr_recv_byte_cb,
	.read_requested = targ3_rd_req_byte_cb,
	.read_processed = targ3_rd_proc_cb,
#endif
	.stop = targ3_stop_cb,
	.error = targ3_error_cb,
};

const struct i2c_target_callbacks targ4_callbacks = {
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	.buf_write_received = targ4_buf_wr_recv_cb,
	.buf_read_requested = targ4_buf_rd_req_cb,
#else
	.write_requested = targ4_wr_req_cb,
	.write_received = targ4_wr_recv_byte_cb,
	.read_requested = targ4_rd_req_byte_cb,
	.read_processed = targ4_rd_proc_cb,
#endif
	.stop = targ4_stop_cb,
	.error = targ4_error_cb,
};

struct app_i2c_target targ1_app_data;
struct app_i2c_target targ2_app_data;
struct app_i2c_target targ3_app_data;
struct app_i2c_target targ4_app_data;

struct i2c_target_config targ1_cfg;
struct i2c_target_config targ2_cfg;
struct i2c_target_config targ3_cfg;
struct i2c_target_config targ4_cfg;

static int fram_test1(const struct i2c_dt_spec *dts);
static int fram_test2(const struct i2c_dt_spec *dts);

static int app_i2c_target_init(struct app_i2c_target *apptrg, uint8_t *buf, size_t bufsz);

static int inspect_i2c_driver_pins(const struct pinctrl_dev_config *pincfg,
				   uint32_t *pin1_cfg, uint32_t *pin2_cfg);

static bool i2c_controllers_are_ready(void)
{
	bool ready = true;

	if (!device_is_ready(i2c_ctrl0_dev)) {
		LOG_ERR("I2C Ctrl0 driver not ready!");
		ready = false;
	}

	if (!device_is_ready(i2c_ctrl1_dev)) {
		LOG_ERR("I2C Ctrl1 driver not ready!");
		ready = false;
	}

	if (!device_is_ready(i2c_ctrl2_dev)) {
		LOG_ERR("I2C Ctrl2 driver not ready!");
		ready = false;
	}

	return ready;
}

static bool i2c_ports_are_ready(void)
{
	bool ready = true;

	if (!device_is_ready(i2c_p0_dev)) {
		LOG_ERR("I2C Port 0 driver not ready!");
		ready = false;
	}

	if (!device_is_ready(i2c_p3_dev)) {
		LOG_ERR("I2C Port 3 driver not ready!");
		ready = false;
	}

	if (!device_is_ready(i2c_p7_dev)) {
		LOG_ERR("I2C Port 7 driver not ready!");
		ready = false;
	}

	return ready;
}

struct i2c_pin_cfg {
	uint32_t pin1_init;
	uint32_t pin2_init;
	uint32_t pin1_resume;
	uint32_t pin2_resume;
	uint32_t pin1_suspend;
	uint32_t pin2_suspend;
};

struct i2c_pin_cfg i2c_p0_pin_cfg;
struct i2c_pin_cfg i2c_p3_pin_cfg;
struct i2c_pin_cfg i2c_p7_pin_cfg;

int main(void)
{
	int rc = 0;
	const struct i2c_ctrl_info *i2c_cri = NULL;
	const struct i2c_port_info *i2c_pi = NULL;

	k_sem_init(&app_targ1_sem, 0, 1);
	k_sem_init(&app_targ2_sem, 0, 1);
	k_sem_init(&app_targ3_sem, 0, 1);
	k_sem_init(&app_targ4_sem, 0, 1);

	memset((void *)msgs, 0, sizeof(msgs));
	memset(i2c_tx_buf, 0x55, I2C_TX_BUF_SIZE);
	memset(i2c_rx_buf, 0xAA, I2C_RX_BUF_SIZE);

#ifdef CONFIG_BOARD_QUALIFIERS
	LOG_INF("Microchip XEC I2Cv3 targ_mode: board: %s/%s", CONFIG_BOARD,
		CONFIG_BOARD_QUALIFIERS);
#else
	LOG_INF("Microchip XEC I2Cv3 targ_mode: board: %s", CONFIG_BOARD);
#endif
#if DT_NODE_HAS_PROP(I2C_CTRL0_NODE, compatible)
	LOG_INF("I2C Ctrl0 compatible: %s", DT_PROP_BY_IDX(I2C_CTRL0_NODE, compatible, 0));
#else
	LOG_INF("I2C Ctrl0 does not have a compatible!");
#endif
#if DT_NODE_HAS_PROP(I2C_CTRL1_NODE, compatible)
	LOG_INF("I2C Ctrl1 compatible: %s", DT_PROP_BY_IDX(I2C_CTRL1_NODE, compatible, 0));
#else
	LOG_INF("I2C Ctrl1 does not have a compatible!");
#endif
#if DT_NODE_HAS_PROP(I2C_CTRL2_NODE, compatible)
	LOG_INF("I2C Ctrl2 compatible: %s", DT_PROP_BY_IDX(I2C_CTRL2_NODE, compatible, 0));
#else
	LOG_INF("I2C Ctrl2 does not have a compatible!");
#endif

	const struct pinctrl_dev_config *zu_pcfg = PINCTRL_DT_DEV_CONFIG_GET(ZEPHYR_USER_NODE);

	rc = pinctrl_apply_state(zu_pcfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("Failed to configure GPIO for TEST_CLK_OUT (%d)", rc);
	}

	log_flush();

	if (!i2c_controllers_are_ready()) {
		goto app_done;
	}

	if (!i2c_ports_are_ready()) {
		goto app_done;
	}

	i2c_topology_dump();
	log_flush();

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

	inspect_i2c_driver_pins(i2c_port0_pincfg, &i2c_p0_pin_cfg.pin1_init,
				&i2c_p0_pin_cfg.pin2_init);
	inspect_i2c_driver_pins(i2c_port3_pincfg, &i2c_p3_pin_cfg.pin1_init,
				&i2c_p3_pin_cfg.pin2_init);
	inspect_i2c_driver_pins(i2c_port7_pincfg, &i2c_p7_pin_cfg.pin1_init,
				&i2c_p7_pin_cfg.pin2_init);
	log_flush();

	/* Put i2c port-3 and i2c port-7 into target mode by registering
	 * targets.
	 */
	memset(targ1_buf, 0x55U, APP_TARG1_BUF_SIZE);
	memset(targ2_buf, 0xAAU, APP_TARG2_BUF_SIZE);
	memset(targ3_buf, 0x55U, APP_TARG3_BUF_SIZE);
	memset(targ4_buf, 0xAAU, APP_TARG4_BUF_SIZE);

	rc = app_i2c_target_init(&targ1_app_data, targ1_buf, APP_TARG1_BUF_SIZE);
	if (rc != 0) {
		LOG_ERR("Init target 1 app structure error (%d)", rc);
		goto app_done;
	}

	rc = app_i2c_target_init(&targ3_app_data, targ3_buf, APP_TARG3_BUF_SIZE);
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

	targ3_cfg.flags = 0;
	targ3_cfg.address = targ3_spec.addr;
	targ3_cfg.callbacks = &targ3_callbacks;

	targ4_cfg.flags = 0;
	targ4_cfg.address = targ4_spec.addr;
	targ4_cfg.callbacks = &targ4_callbacks;

	LOG_INF("Register target 1 (0x%02x) on %s", targ1_spec.addr, targ1_spec.bus->name);
	rc = i2c_target_register(targ1_spec.bus, &targ1_cfg);
	if (rc != 0) {
		LOG_ERR("i2c_target_register targ1 failed (%d)", rc);
		goto app_done;
	}

	LOG_INF("Register target 3 (0x%02x) on %s", targ3_spec.addr, targ3_spec.bus->name);
	rc = i2c_target_register(targ3_spec.bus, &targ3_cfg);
	if (rc != 0) {
		LOG_ERR("i2c_target_register targ3 failed (%d)", rc);
		goto app_done;
	}

	/* Send PM_DEVICE SUSPEND message to each port */
	LOG_INF("PM_DEVICE suspend %s", i2c_p0_dev->name);
	rc = pm_device_action_run(i2c_p0_dev, PM_DEVICE_ACTION_SUSPEND);
	if (rc != 0) {
		LOG_ERR("Suspend error (%d)", rc);
	}
	inspect_i2c_driver_pins(i2c_port0_pincfg, &i2c_p0_pin_cfg.pin1_suspend,
				&i2c_p0_pin_cfg.pin2_suspend);

	LOG_INF("PM_DEVICE suspend %s", i2c_p3_dev->name);
	rc = pm_device_action_run(i2c_p3_dev, PM_DEVICE_ACTION_SUSPEND);
	if (rc != 0) {
		LOG_ERR("Suspend error (%d)", rc);
	}
	inspect_i2c_driver_pins(i2c_port3_pincfg, &i2c_p3_pin_cfg.pin1_suspend,
				&i2c_p3_pin_cfg.pin2_suspend);

	LOG_INF("PM_DEVICE suspend %s", i2c_p7_dev->name);
	rc = pm_device_action_run(i2c_p7_dev, PM_DEVICE_ACTION_SUSPEND);
	if (rc != 0) {
		LOG_ERR("Suspend error (%d)", rc);
	}
	inspect_i2c_driver_pins(i2c_port7_pincfg, &i2c_p7_pin_cfg.pin1_suspend,
				&i2c_p7_pin_cfg.pin2_suspend);
	log_flush();

	k_sleep(K_MSEC(1000));

	LOG_INF("PM_DEVICE resume %s", i2c_p0_dev->name);
	rc = pm_device_action_run(i2c_p0_dev, PM_DEVICE_ACTION_RESUME);
	if (rc != 0) {
		LOG_ERR("Suspend error (%d)", rc);
	}
	inspect_i2c_driver_pins(i2c_port0_pincfg, &i2c_p0_pin_cfg.pin1_resume,
				&i2c_p0_pin_cfg.pin2_resume);

	LOG_INF("PM_DEVICE resume %s", i2c_p3_dev->name);
	rc = pm_device_action_run(i2c_p3_dev, PM_DEVICE_ACTION_RESUME);
	if (rc != 0) {
		LOG_ERR("Suspend error (%d)", rc);
	}
	inspect_i2c_driver_pins(i2c_port3_pincfg, &i2c_p3_pin_cfg.pin1_resume,
				&i2c_p3_pin_cfg.pin2_resume);

	LOG_INF("PM_DEVICE resume %s", i2c_p7_dev->name);
	rc = pm_device_action_run(i2c_p7_dev, PM_DEVICE_ACTION_RESUME);
	if (rc != 0) {
		LOG_ERR("Suspend error (%d)", rc);
	}
	inspect_i2c_driver_pins(i2c_port7_pincfg, &i2c_p7_pin_cfg.pin1_resume,
				&i2c_p7_pin_cfg.pin2_resume);
	log_flush();

	k_sleep(K_MSEC(1000));

	i2c_pi = NULL;
	i2c_cri = i2c_ctrl_for_port(i2c_p0_dev, &i2c_pi);
	if ((i2c_cri != NULL) && (i2c_pi != NULL)) {
		if (i2c_cri->wakeup_source) {
			LOG_INF("I2C Port 0 device is a wakeup source");
			if ((i2c_p0_pin_cfg.pin1_suspend == i2c_p0_pin_cfg.pin1_init) &&
			    (i2c_p0_pin_cfg.pin2_suspend == i2c_p0_pin_cfg.pin2_init)) {
				LOG_INF("PASS: both pins enabled for I2C");
			} else {
				LOG_ERR("FAIL: one or both pins not enabled for I2C");
			}
		} else {
			LOG_INF("I2C Port 0 device is not a wakeup source");
			if ((((i2c_p0_pin_cfg.pin1_suspend & 0xCU) >> 2) == 2U) &&
			    (((i2c_p0_pin_cfg.pin2_suspend & 0xCU) >> 2) == 2U)) {
				LOG_INF("PASS: both pins in power-gate off state");
			} else {
				LOG_ERR("FAIL: one or both pins not in power-gate off state");
			}
		}
	} else {
		LOG_ERR("Unable to get info for I2C port 0 device");
	}

	i2c_pi = NULL;
	i2c_cri = i2c_ctrl_for_port(i2c_p3_dev, &i2c_pi);
	if ((i2c_cri != NULL) && (i2c_pi != NULL)) {
		if (i2c_cri->wakeup_source) {
			LOG_INF("I2C Port 3 device is a wakeup source");
			if ((i2c_p3_pin_cfg.pin1_suspend == i2c_p3_pin_cfg.pin1_init) &&
			    (i2c_p3_pin_cfg.pin2_suspend == i2c_p3_pin_cfg.pin2_init)) {
				LOG_INF("PASS: both pins enabled for I2C");
			} else {
				LOG_ERR("FAIL: one or both pins not enabled for I2C");
			}
		} else {
			LOG_INF("I2C Port 3 device is not a wakeup source");
			if ((((i2c_p3_pin_cfg.pin1_suspend & 0xCU) >> 2) == 2U) &&
			    (((i2c_p3_pin_cfg.pin2_suspend & 0xCU) >> 2) == 2U)) {
				LOG_INF("PASS: both pins in power-gate off state");
			} else {
				LOG_ERR("FAIL: one or both pins not in power-gate off state");
			}
		}
	} else {
		LOG_ERR("Unable to get info for I2C port 3 device");
	}

	i2c_pi = NULL;
	i2c_cri = i2c_ctrl_for_port(i2c_p7_dev, &i2c_pi);
	if ((i2c_cri != NULL) && (i2c_pi != NULL)) {
		if (i2c_cri->wakeup_source) {
			LOG_INF("I2C Port 7 device is a wakeup source");
			if ((i2c_p7_pin_cfg.pin1_suspend == i2c_p7_pin_cfg.pin1_init) &&
			    (i2c_p7_pin_cfg.pin2_suspend == i2c_p7_pin_cfg.pin2_init)) {
				LOG_INF("PASS: both pins enabled for I2C");
			} else {
				LOG_ERR("FAIL: one or both pins not enabled for I2C");
			}
		} else {
			LOG_INF("I2C Port 0 device is not a wakeup source");
			if ((((i2c_p7_pin_cfg.pin1_suspend & 0xCU) >> 2) == 2U) &&
			    (((i2c_p7_pin_cfg.pin2_suspend & 0xCU) >> 2) == 2U)) {
				LOG_INF("PASS: both pins in power-gate off state");
			} else {
				LOG_ERR("FAIL: one or both pins not in power-gate off state");
			}
		}
	} else {
		LOG_ERR("Unable to get info for I2C port 7 device");
	}

app_done:
	LOG_INF("Program End (rc=%d)", rc);
	log_flush();

	return 0;
}

/* O(1) by SMB instance number; NULL if out of range or that instance is off. */
static inline const struct i2c_ctrl_info *i2c_ctrl_by_inst(unsigned int n)
{
	if (n >= ARRAY_SIZE(i2c_ctrls) || i2c_ctrls[n].ctrl == NULL) {
		return NULL;
	}

	return &i2c_ctrls[n];
}

/* Reverse: port bus device -> its controller entry (+ optional port entry). */
static const struct i2c_ctrl_info *i2c_ctrl_for_port(const struct device *port,
						     const struct i2c_port_info **port_out)
{
	for (unsigned int n = 0; n < ARRAY_SIZE(i2c_ctrls); n++) {
		const struct i2c_ctrl_info *c = &i2c_ctrls[n];

		if (c->ctrl == NULL) {
			continue;
		}

		for (size_t p = 0; p < c->num_ports; p++) {   /* sentinel excluded */
			if (c->ports[p].dev == port) {
				if (port_out != NULL) {
					*port_out = &c->ports[p];
				}
				return c;
			}
		}
	}

	if (port_out != NULL) {
		*port_out = NULL;
	}

	return NULL;
}

/* Reverse convenience: port bus device -> SMB instance number, or -ENODEV. */
static inline int i2c_inst_for_port(const struct device *port)
{
	const struct i2c_ctrl_info *c = i2c_ctrl_for_port(port, NULL);

	return (c != NULL) ? (int)c->inst : -ENODEV;
}

static void i2c_topology_dump(void)
{
	for (unsigned int n = 0; n < ARRAY_SIZE(i2c_ctrls); n++) {
		const struct i2c_ctrl_info *c = &i2c_ctrls[n];

		if (c->ctrl == NULL) {
			continue;   /* instance n disabled in this overlay */
		}

		LOG_INF("I2C-SMB%u %s  wakeup_source=%d  %zu port(s)",
			c->inst, c->ctrl->name, c->wakeup_source, c->num_ports);

		for (size_t p = 0; p < c->num_ports; p++) {
			const struct i2c_port_info *pi = &c->ports[p];

			LOG_INF("   port %u -> %s%s %s", pi->port, pi->dev->name,
				pi->is_default ? " (default)" : "",
				device_is_ready(pi->dev) ? "" : "[not ready]");
		}
	}
}

static int inspect_i2c_driver_pins(const struct pinctrl_dev_config *pincfg,
				   uint32_t *pin1_cfg, uint32_t *pin2_cfg)
{
	const struct pinctrl_state *dflt_state = NULL;

	if (pincfg == NULL) {
		LOG_ERR("Bad pinctrl pointer");
		return -EINVAL;
	}

	for (uint8_t i = 0; i < pincfg->state_cnt; i++) {
		if (pincfg->states[i].id == PINCTRL_STATE_DEFAULT) {
			dflt_state = &pincfg->states[i];
		}
	}

	if (dflt_state == NULL) {
		LOG_ERR("Could not find default PINCTRL state!");
		return -EINVAL;
	}

	for (size_t n = 0; n < dflt_state->pin_cnt; n++) {
		struct mec_pinctrl pcs = (struct mec_pinctrl)dflt_state->pins[n];
		uint32_t port = MCHP_XEC_PINMUX_PORT(pcs.pinmux);
		uint32_t pin_in_port = MCHP_XEC_PINMUX_PIN(pcs.pinmux);
		uint32_t gpio_num = (port * 32U) + pin_in_port;
		uint32_t cr_addr = XEC_PIN_CTRL_REG_ADDR(port, pin_in_port);
		uint32_t crv = sys_read32(cr_addr);

		if (pin1_cfg != NULL) {
			if (n == 0) {
				*pin1_cfg = crv;
			}
		}

		if (pin2_cfg != NULL) {
			if (n == 1U) {
				*pin2_cfg = crv;
			}
		}

		LOG_INF("XEC GPIO[%03o] bank[%u] pin[%u] -> CR:0x%08x Mux:%u Pull:%u PWG:%u",
			gpio_num, port, pin_in_port, crv, ((crv >> 12) & 0x7U), (crv & 0x3U),
			((crv >> 2) & 0x3U));
	}

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

/* stop/error are identical in both modes: they observe completion and
 * count errors, independent of how payload bytes are delivered.
 */
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

static int targ3_stop_cb(struct i2c_target_config *config)
{
	targ3_app_data.stop_cnt++;
	targ3_app_data.idx = 0;

	k_sem_give(&app_targ3_sem);

	return 0;
}

static void targ3_error_cb(struct i2c_target_config *config, enum i2c_error_reason error_code)
{
	targ3_app_data.error_cnt++;
	targ3_app_data.err = error_code;
}

static int targ4_stop_cb(struct i2c_target_config *config)
{
	targ4_app_data.stop_cnt++;
	targ4_app_data.idx = 0;

	k_sem_give(&app_targ4_sem);

	return 0;
}

static void targ4_error_cb(struct i2c_target_config *config, enum i2c_error_reason error_code)
{
	targ4_app_data.error_cnt++;
	targ4_app_data.err = error_code;
}

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
static void targ1_buf_wr_recv_cb(struct i2c_target_config *config, uint8_t *ptr, uint32_t len)
{
	uint32_t max_idx = targ1_app_data.wr_idx + len;
	uint8_t *p = ptr;

	targ1_app_data.wr_recv_cnt++;

	if (p == NULL) {
		return;
	}

	if (max_idx > targ1_app_data.bufsz) {
		max_idx = targ1_app_data.bufsz;
	}
	for (uint32_t i = targ1_app_data.wr_idx; i < max_idx; i++) {
		targ1_app_data.buf[i] = *p++;
	}
	/* Advance wr_idx so streaming-mode multi-chunk delivery
	 * accumulates contiguously across successive callbacks; the
	 * read-side cursor (idx) stays put.
	 */
	targ1_app_data.wr_idx = max_idx;
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

static void targ2_buf_wr_recv_cb(struct i2c_target_config *config, uint8_t *ptr, uint32_t len)
{
	uint32_t max_idx = targ2_app_data.wr_idx + len;
	uint8_t *p = ptr;

	targ2_app_data.wr_recv_cnt++;

	if (p == NULL) {
		return;
	}

	if (max_idx > targ2_app_data.bufsz) {
		max_idx = targ2_app_data.bufsz;
	}
	for (uint32_t i = targ2_app_data.wr_idx; i < max_idx; i++) {
		targ2_app_data.buf[i] = *p++;
	}
	/* Advance wr_idx; see the targ1 cb for the rationale. */
	targ2_app_data.wr_idx = max_idx;
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

static void targ3_buf_wr_recv_cb(struct i2c_target_config *config, uint8_t *ptr, uint32_t len)
{
	uint32_t max_idx = targ3_app_data.wr_idx + len;
	uint8_t *p = ptr;

	targ3_app_data.wr_recv_cnt++;

	if (p == NULL) {
		return;
	}

	if (max_idx > targ3_app_data.bufsz) {
		max_idx = targ3_app_data.bufsz;
	}
	for (uint32_t i = targ3_app_data.wr_idx; i < max_idx; i++) {
		targ3_app_data.buf[i] = *p++;
	}
	/* Advance wr_idx; see the targ1 cb for the rationale. */
	targ3_app_data.wr_idx = max_idx;
}

static int targ3_buf_rd_req_cb(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len)
{
	targ3_app_data.rd_req_cnt++;

	if ((ptr == NULL) || (len == NULL)) {
		return -EINVAL;
	}

	*ptr = &targ3_app_data.buf[targ3_app_data.idx];
	*len = targ3_app_data.bufsz - targ3_app_data.idx;

	return 0;
}

static void targ4_buf_wr_recv_cb(struct i2c_target_config *config, uint8_t *ptr, uint32_t len)
{
	uint32_t max_idx = targ4_app_data.wr_idx + len;
	uint8_t *p = ptr;

	targ4_app_data.wr_recv_cnt++;

	if (p == NULL) {
		return;
	}

	if (max_idx > targ4_app_data.bufsz) {
		max_idx = targ4_app_data.bufsz;
	}
	for (uint32_t i = targ4_app_data.wr_idx; i < max_idx; i++) {
		targ4_app_data.buf[i] = *p++;
	}
	/* Advance wr_idx; see the targ1 cb for the rationale. */
	targ4_app_data.wr_idx = max_idx;
}

static int targ4_buf_rd_req_cb(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len)
{
	targ4_app_data.rd_req_cnt++;

	if ((ptr == NULL) || (len == NULL)) {
		return -EINVAL;
	}

	*ptr = &targ4_app_data.buf[targ4_app_data.idx];
	*len = targ4_app_data.bufsz - targ4_app_data.idx;

	return 0;
}
#else /* byte-granular target callbacks (CONFIG_I2C_TARGET_BUFFER_MODE=n) */

/* Byte returned to the host when a read runs past the stored payload;
 * matches the 0x55 initial fill the read-back tests expect.
 */
#define BM_TGT_RD_FILL 0x55U

/* The byte-mode callbacks feed the SAME app state as their buffer-mode
 * counterparts so every test's counter/data assertions hold unchanged:
 *   - write_requested fires once per write txn      -> wr_recv_cnt++
 *   - write_received appends one byte at wr_idx      (read cursor idx untouched)
 *   - read_requested fires once per read txn        -> rd_req_cnt++, first byte
 *   - read_processed supplies each subsequent byte, walking idx
 * write uses wr_idx, read walks idx (reset at stop) -- writes never move idx,
 * so a read after a write still starts at buf[0], matching buffer mode.
 */
static int targ1_wr_req_cb(struct i2c_target_config *config)
{
	targ1_app_data.wr_recv_cnt++;

	return 0;
}

static int targ1_wr_recv_byte_cb(struct i2c_target_config *config, uint8_t val)
{
	if (targ1_app_data.wr_idx < targ1_app_data.bufsz) {
		targ1_app_data.buf[targ1_app_data.wr_idx++] = val;
	}

	/* Byte-mode overflow test: NACK once nack_after bytes are accepted. */
	if ((targ1_app_data.nack_after != 0U) &&
	    (targ1_app_data.wr_idx >= targ1_app_data.nack_after)) {
		return -1;
	}

	return 0;
}

static int targ1_rd_req_byte_cb(struct i2c_target_config *config, uint8_t *val)
{
	targ1_app_data.rd_req_cnt++;

	if (val == NULL) {
		return -EINVAL;
	}

	*val = (targ1_app_data.idx < targ1_app_data.bufsz)
		       ? targ1_app_data.buf[targ1_app_data.idx++]
		       : BM_TGT_RD_FILL;

	return 0;
}

static int targ1_rd_proc_cb(struct i2c_target_config *config, uint8_t *val)
{
	if (val == NULL) {
		return -EINVAL;
	}

	*val = (targ1_app_data.idx < targ1_app_data.bufsz)
		       ? targ1_app_data.buf[targ1_app_data.idx++]
		       : BM_TGT_RD_FILL;

	return 0;
}

static int targ2_wr_req_cb(struct i2c_target_config *config)
{
	targ2_app_data.wr_recv_cnt++;

	return 0;
}

static int targ2_wr_recv_byte_cb(struct i2c_target_config *config, uint8_t val)
{
	if (targ2_app_data.wr_idx < targ2_app_data.bufsz) {
		targ2_app_data.buf[targ2_app_data.wr_idx++] = val;
	}

	if ((targ2_app_data.nack_after != 0U) &&
	    (targ2_app_data.wr_idx >= targ2_app_data.nack_after)) {
		return -1;
	}

	return 0;
}

static int targ2_rd_req_byte_cb(struct i2c_target_config *config, uint8_t *val)
{
	targ2_app_data.rd_req_cnt++;

	if (val == NULL) {
		return -EINVAL;
	}

	*val = (targ2_app_data.idx < targ2_app_data.bufsz)
		       ? targ2_app_data.buf[targ2_app_data.idx++]
		       : BM_TGT_RD_FILL;

	return 0;
}

static int targ2_rd_proc_cb(struct i2c_target_config *config, uint8_t *val)
{
	if (val == NULL) {
		return -EINVAL;
	}

	*val = (targ2_app_data.idx < targ2_app_data.bufsz)
		       ? targ2_app_data.buf[targ2_app_data.idx++]
		       : BM_TGT_RD_FILL;

	return 0;
}
#endif /* CONFIG_I2C_TARGET_BUFFER_MODE */
