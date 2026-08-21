/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * I2Cv3 deep-sleep wake-on-I2C sample.
 *
 * Demonstrates waking the MEC1753 SoC from deep sleep (suspend-to-RAM)
 * when an external I2C controller addresses a registered, wake-capable
 * target. The board overlay selects the driver:
 *   mec1753_qlj -> microchip,xec-i2c-v3-nl (network-layer / DMA)
 *   mec1753_qsz -> microchip,xec-i2c-v3-bm (byte-mode)
 *
 * Topology (both boards):
 *   SMB0 = controller on port 0 @ 100 kHz.
 *   SMB1 = plain target 0x40 on port 7 @ 100 kHz.
 *   SMB4 = wake-capable target 0x38 on port 3 @ 100 kHz (wakeup-source).
 *
 * Flow:
 *   1. Register the 0x40 and 0x38 targets.
 *   2. Pre-sleep loopback self-test: with port 0 fly-wired to port 7
 *      (J12 <-> J22), SMB0 writes then reads back the 0x40 target to
 *      prove the controller + target paths work.
 *   3. Force PM_STATE_SUSPEND_TO_RAM (deep sleep) and block waiting for
 *      the 0x38 target's stop callback.
 *   4. An external I2C controller on port 3 issues START + address 0x38,
 *      which wakes the SoC. The target callbacks capture the bytes; the
 *      app logs them (proving the wake source) and halts.
 *
 * Wake requires ALL of: CONFIG_PM_DEVICE, CONFIG_I2C_TARGET, wakeup-source
 * on the SMB4 controller node, and a registered target on it -- only then
 * does the driver's suspend hook arm the START-detect wake.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_xec_i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>

#include "xec_vci.h"

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

/* Compile-time label for whichever driver the board overlay enabled. */
#if DT_NODE_HAS_COMPAT(DT_NODELABEL(i2c_smb_0), microchip_xec_i2c_v3_nl)
#define DRV_LABEL "microchip,xec-i2c-v3-nl (network-layer/DMA)"
#else
#define DRV_LABEL "microchip,xec-i2c-v3-bm (byte-mode)"
#endif

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

PINCTRL_DT_DEFINE(ZEPHYR_USER_NODE);

/* SMB0 port-0 device: the controller bus for the loopback self-test. */
static const struct device *const smb0_port0 = DEVICE_DT_GET(DT_NODELABEL(i2c0_p0));

/* Target specs: .bus is the owning port device, .addr the 7-bit address. */
static const struct i2c_dt_spec targ040 = I2C_DT_SPEC_GET(DT_NODELABEL(i2c_targ_040));

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(i2c_targ_038))
static const struct i2c_dt_spec targ038 = I2C_DT_SPEC_GET(DT_NODELABEL(i2c_targ_038));
#endif

#define APP_TARG_BUF_SIZE 64U

/* Bounded wait for the loopback target's stop; a wiring fault then surfaces
 * as a FAIL instead of a hang. The deep-sleep wait is unbounded on purpose.
 */
#define LOOPBACK_TIMEOUT K_MSEC(500)

/* Per-target application state. The buffer-mode callbacks resolve their
 * instance from the embedded i2c_target_config via CONTAINER_OF, so one
 * callback table serves both targets.
 */
struct app_target {
	const char *name;
	uint8_t buf[APP_TARG_BUF_SIZE];
	volatile size_t rx_len;      /* bytes stored by buf_write_received */
	volatile uint32_t wr_cnt;    /* buf_write_received invocations */
	volatile uint32_t rd_cnt;    /* buf_read_requested invocations */
	volatile uint32_t stop_cnt;  /* stop invocations */
	volatile uint32_t err_cnt;   /* error invocations */
	struct k_sem stop_sem;
	struct i2c_target_config cfg;
};

static struct app_target t040 = {.name = "0x40"};
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(i2c_targ_038))
static struct app_target t038 = {.name = "0x38 (wake)"};
#endif

static const struct gpio_dt_spec pm_gpio_pin = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, test_pm_gpios);

uint8_t i2c_state_cap_buf[256] __aligned(4);

static void app_buf_wr_recv(struct i2c_target_config *config, uint8_t *ptr, uint32_t len)
{
	struct app_target *t = CONTAINER_OF(config, struct app_target, cfg);
	size_t off = t->rx_len;

	t->wr_cnt++;
	if (ptr == NULL) {
		return;
	}
	for (uint32_t i = 0; (i < len) && (off < sizeof(t->buf)); i++) {
		t->buf[off++] = ptr[i];
	}
	t->rx_len = off;
}

static int app_buf_rd_req(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len)
{
	struct app_target *t = CONTAINER_OF(config, struct app_target, cfg);

	t->rd_cnt++;
	if ((ptr == NULL) || (len == NULL)) {
		return -EINVAL;
	}
	*ptr = t->buf;
	*len = sizeof(t->buf);

	return 0;
}

static int app_stop(struct i2c_target_config *config)
{
	struct app_target *t = CONTAINER_OF(config, struct app_target, cfg);

	t->stop_cnt++;
	k_sem_give(&t->stop_sem);

	return 0;
}

static void app_error(struct i2c_target_config *config, enum i2c_error_reason reason)
{
	struct app_target *t = CONTAINER_OF(config, struct app_target, cfg);

	ARG_UNUSED(reason);
	t->err_cnt++;
}

static const struct i2c_target_callbacks app_callbacks = {
	.buf_write_received = app_buf_wr_recv,
	.buf_read_requested = app_buf_rd_req,
	.stop = app_stop,
	.error = app_error,
};

static void reset_target(struct app_target *t)
{
	t->rx_len = 0;
	t->wr_cnt = 0;
	t->rd_cnt = 0;
	t->stop_cnt = 0;
	t->err_cnt = 0;
	memset(t->buf, 0, sizeof(t->buf));
	k_sem_reset(&t->stop_sem);
}

static int register_target(const struct i2c_dt_spec *spec, struct app_target *t)
{
	int rc;

	k_sem_init(&t->stop_sem, 0, 1);
	reset_target(t);

	t->cfg.flags = 0;
	t->cfg.address = spec->addr;
	t->cfg.callbacks = &app_callbacks;

	LOG_INF("Register target %s (0x%02x) on %s", t->name, spec->addr, spec->bus->name);
	rc = i2c_target_register(spec->bus, &t->cfg);
	if (rc != 0) {
		LOG_ERR("i2c_target_register %s failed (%d)", t->name, rc);
	}

	return rc;
}

/* Pre-sleep loopback self-test: SMB0 (port 0) writes 4 bytes to the 0x40
 * target that SMB1 exposes on port 7 (fly-wired), then reads them back.
 * Returns 0 on PASS. Requires the J12<->J22 fly wire.
 */
static int loopback_self_test(void)
{
	static const uint8_t tx[4] = {0xDEU, 0xADU, 0xBEU, 0xEFU};
	uint8_t rx[4] = {0};
	int rc;

	reset_target(&t040);

	rc = i2c_write(smb0_port0, tx, sizeof(tx), targ040.addr);
	if (rc != 0) {
		LOG_ERR("loopback: i2c_write to 0x%02x failed (%d) - check J12<->J22 fly wire",
			targ040.addr, rc);
		return rc;
	}

	rc = k_sem_take(&t040.stop_sem, LOOPBACK_TIMEOUT);
	if (rc != 0) {
		LOG_ERR("loopback: no stop from 0x%02x target (%d)", targ040.addr, rc);
		return rc;
	}

	if ((t040.wr_cnt != 1U) || (t040.rx_len != sizeof(tx)) ||
	    (memcmp(t040.buf, tx, sizeof(tx)) != 0)) {
		LOG_ERR("loopback: write mismatch wr_cnt=%u rx_len=%u", t040.wr_cnt,
			(unsigned int)t040.rx_len);
		return -EIO;
	}

	/* Read the same bytes back over the wire. */
	rc = i2c_read(smb0_port0, rx, sizeof(rx), targ040.addr);
	if (rc != 0) {
		LOG_ERR("loopback: i2c_read from 0x%02x failed (%d)", targ040.addr, rc);
		return rc;
	}
	if (memcmp(rx, tx, sizeof(tx)) != 0) {
		LOG_ERR("loopback: read-back mismatch");
		return -EIO;
	}

	LOG_INF("loopback self-test PASS: wrote/read {%02x %02x %02x %02x} via 0x%02x",
		rx[0], rx[1], rx[2], rx[3], targ040.addr);

	return 0;
}

static void pr_pcr_clk_req_from_vbat(void)
{
	uintptr_t vbmem_addr = (uintptr_t)DT_REG_ADDR(DT_NODELABEL(bbram));

	for (uint32_t i = 0; i < 5U; i++) {
		LOG_INF("PCR CLK_REQ[%u] = 0x%08x", i, sys_read32(vbmem_addr));
		vbmem_addr += 4U;
	}
}

#if 0
extern uint32_t soc_pcr_clk_req[5];

static void pr_pcr_clk_req_wait(void)
{
	for (uint32_t i = 0; i < 5U; i++) {
		LOG_INF("PCR CLK_REQ_wait[%u] = 0x%08x", i, soc_pcr_clk_req[i]);
	}
}
#endif

volatile uint8_t dbg_i2c_v3_pm_susp[5];

int main(void)
{
	const struct pinctrl_dev_config *zu_pcfg = PINCTRL_DT_DEV_CONFIG_GET(ZEPHYR_USER_NODE);
	uint32_t r = 0, loop_count = 0;
	int rc;

	LOG_INF("I2Cv3 deep-sleep wake sample; driver: %s", DRV_LABEL);

#ifdef CONFIG_BOARD_QUALIFIERS
	LOG_INF("Board: %s/%s", CONFIG_BOARD, CONFIG_BOARD_QUALIFIERS);
#else
	LOG_INF("Board: %s", CONFIG_BOARD);
#endif

	memset((void *)dbg_i2c_v3_pm_susp, 0x55U, sizeof(dbg_i2c_v3_pm_susp));
	memset(i2c_state_cap_buf, 0, sizeof(i2c_state_cap_buf));

	if (!gpio_is_ready_dt(&pm_gpio_pin)) {
		LOG_ERR("PM GPIO pin is not ready!");
	}

	rc = gpio_pin_configure_dt(&pm_gpio_pin, GPIO_OUTPUT_HIGH);
	if (rc != 0) {
		LOG_ERR("PM GPIO pin configuration error (%d)", rc);
	}

	rc = pinctrl_apply_state(zu_pcfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("Failed to configure zephyr-user pins (%d)", rc);
	}

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(i2c_targ_038))
	if (!device_is_ready(smb0_port0) || !device_is_ready(targ040.bus) ||
	    !device_is_ready(targ038.bus)) {
#else
	if (!device_is_ready(smb0_port0) || !device_is_ready(targ040.bus)) {
#endif
		LOG_ERR("I2C bus device(s) not ready");
		return 0;
	}

	/* Register both targets. The 0x38 target is on the wakeup-source SMB4
	 * controller, so registering it arms the deep-sleep wake path.
	 */
	if (register_target(&targ040, &t040) != 0) {
		return 0;
	}
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(i2c_targ_038))
	if (register_target(&targ038, &t038) != 0) {
		return 0;
	}
#endif

	/* Phase 1: prove the controller + target paths before sleeping. */
	rc = loopback_self_test();
	if (rc != 0) {
		LOG_WRN("loopback self-test FAILED (%d); continuing to deep-sleep demo", rc);
	}

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(i2c_targ_038))
#if defined(CONFIG_PM) || defined(CONFIG_PM_DEVICE)
	while (1) {
		loop_count++;
		LOG_INF("Deep sleep loop %u", loop_count);

		mchp_vci_in_input_enable(BIT(MEC_VCI_IN1_POS));
		mchp_vci_in_latch_enable(BIT(MEC_VCI_IN1_POS));
		mchp_vci_edge_detect_clr_all();
		mchp_vci_in_latch_reset(BIT(MEC_VCI_IN1_POS));

		LOG_INF("Disconnect Debugger and press switch S4 when ready");
		r = mchp_vci_pedge_detect() & mchp_vci_nedge_detect();
		while ((r & BIT(MEC_VCI_IN1_POS)) == 0) {
			r = mchp_vci_pedge_detect() & mchp_vci_nedge_detect();
		}

		/* Phase 2: force deep sleep and wait for an external wake on 0x38. */
		reset_target(&t038);

		LOG_INF("Forcing deep sleep (PM_STATE_SUSPEND_TO_RAM).");
		LOG_INF("Wake it: drive START + address 0x%02x from an external I2C", targ038.addr);
		LOG_INF("controller on port 3 (SDA=GPIO007, SCL=GPIO010).");

		/* Force the deepest state on next idle. Blocking on the wake semaphore
		 * lets the idle thread run and enter suspend-to-RAM; the external write
		 * to 0x38 wakes the CPU (own-address match, GIRQ13), the driver runs the
		 * target callbacks, and app_stop() gives the semaphore below.
		 */
		if (!pm_state_force(0U, &(struct pm_state_info){PM_STATE_SUSPEND_TO_RAM, 0, 0})) {
			LOG_ERR("pm_state_force(SUSPEND_TO_RAM) rejected");
			return 0;
		}

		k_sem_take(&t038.stop_sem, K_FOREVER);

		/* Woke. */
		LOG_INF("WOKE from deep sleep via target 0x%02x: wr_cnt=%u rd_cnt=%u rx_len=%u",
			targ038.addr, t038.wr_cnt, t038.rd_cnt, (unsigned int)t038.rx_len);
		if (t038.rx_len > 0U) {
			LOG_HEXDUMP_INF(t038.buf, t038.rx_len, "bytes received from external controller:");
		}

		LOG_INF("PCR CLK_REQ captured just before WFI");
		pr_pcr_clk_req_from_vbat();
		/* pr_pcr_clk_req_wait(); */

		LOG_INF("Suspend I2C.SR   = 0x%02x", dbg_i2c_v3_pm_susp[0]);
		LOG_INF("Suspend I2C.WKSR = 0x%02x", dbg_i2c_v3_pm_susp[1]);
		LOG_INF("Suspend I2C.WKCR = 0x%02x", dbg_i2c_v3_pm_susp[2]);
		LOG_INF("Suspend GIRQ22 Source = 0x%02x", dbg_i2c_v3_pm_susp[3]);
		LOG_INF("Suspend GIRQ22 EnSet  = 0x%02x", dbg_i2c_v3_pm_susp[4]);

#if DT_NODE_HAS_COMPAT(DT_NODELABEL(i2c_smb_4), microchip_xec_i2c_v3_nl)
		rc = mchp_xec_i2c_nl_copy_capture(targ038.bus, i2c_state_cap_buf, 256U);
#else
		rc = mchp_xec_i2c_bm_copy_capture(targ038.bus, i2c_state_cap_buf, 256U);
#endif
		if (rc == 0) {
			LOG_HEXDUMP_INF(i2c_state_cap_buf, 256U, "targ038.bus states");
		} else {
			LOG_ERR("I2C BM state capture copy error (%d)", rc);
		}
	} /* end while (1) */
#endif
#endif

	LOG_INF("Deep-sleep I2C wake demo complete; halting.");
	log_flush();

	k_sleep(K_FOREVER);

	return 0;
}
