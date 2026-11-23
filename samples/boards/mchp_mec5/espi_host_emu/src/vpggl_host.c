/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Host-side `hvpggl` shell for bench-testing the Target's VPGGL (glue logic).
 *
 * The Host EVB drives the Target's VPGGL input pins from the GPIOs listed in
 * zephyr,user vpggl-gpios (boards/vpggl_bench.overlay), in this order:
 *   0 VCC_PWRGD   1 PWRGD_S0IX   2 SLP_S0#
 * and the related Controller-to-Target virtual wires over the emulated eSPI
 * link (PLTRST#, HOST_C10, or any other).
 *
 * Levels are ELECTRICAL (1 = high), including for the low-active SLP_S0#
 * and PLTRST#; `s0ix enter|exit` is the convenience form for SLP_S0#.
 *
 *   hvpggl pins                         show the levels being driven
 *   hvpggl pin <vcc|s0ix|slp> <0|1>     drive one pin
 *   hvpggl pulse <vcc|s0ix|slp> <ms>    invert a pin for <ms>, then restore
 *   hvpggl s0ix <enter|exit>            SLP_S0# low / high
 *   hvpggl pltrst <0|1>                 PLTRST# VWire (1 = de-asserted)
 *   hvpggl c10 <0|1>                    HOST_C10 VWire
 *   hvpggl vw <host_idx> <bit> <0|1>    any Controller-to-Target VWire
 *   hvpggl status                       GET_STATUS, fetch Target-to-Host VWires
 *
 * eSPI commands are refused until main() has brought the link up and
 * de-asserted PLTRST# (CONFIG_SAMPLE_STOP_AFTER_PLTRST).
 */

#ifdef CONFIG_SAMPLE_VPGGL_SHELL

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include "espi_hc_emu.h"
#include "vpggl_host.h"

LOG_MODULE_REGISTER(vpggl_host, CONFIG_LOG_DEFAULT_LEVEL);

#define USER_NODE DT_PATH(zephyr_user)

BUILD_ASSERT(DT_PROP_LEN(USER_NODE, vpggl_gpios) == 3,
	     "zephyr,user vpggl-gpios must list VCC_PWRGD, PWRGD_S0IX, SLP_S0#");

enum { P_VCC, P_S0IX, P_SLP, P_NUM };

static const struct gpio_dt_spec pins[P_NUM] = {
	GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, vpggl_gpios, 0),
	GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, vpggl_gpios, 1),
	GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, vpggl_gpios, 2),
};
static const char *const pin_key[P_NUM] = {"vcc", "s0ix", "slp"};
static const char *const pin_name[P_NUM] = {"VCC_PWRGD", "PWRGD_S0IX", "SLP_S0#"};

/* Idle: running S0, main power good, alternate path off, not in S0ix. */
static const uint8_t pin_idle[P_NUM] = {1, 0, 1};
static uint8_t pin_lvl[P_NUM];

#define VW_IDX_PLTRST   0x03u
#define VW_BIT_PLTRST   1u
#define VW_IDX_HOST_C10 0x47u
#define VW_BIT_HOST_C10 0u

static struct espi_hc_context *hc;
static atomic_t link_up;
static K_MUTEX_DEFINE(espi_lock);

int vpggl_host_pins_init(void)
{
	for (int i = 0; i < P_NUM; i++) {
		int ret;

		if (!gpio_is_ready_dt(&pins[i])) {
			LOG_ERR("VPGGL %s GPIO not ready", pin_name[i]);
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&pins[i], pin_idle[i] ? GPIO_OUTPUT_HIGH
								   : GPIO_OUTPUT_LOW);
		if (ret != 0) {
			LOG_ERR("VPGGL %s configure: %d", pin_name[i], ret);
			return ret;
		}
		pin_lvl[i] = pin_idle[i];
	}
	LOG_INF("VPGGL pins driven to idle: VCC_PWRGD=1 PWRGD_S0IX=0 SLP_S0#=1");

	return 0;
}

void vpggl_host_link_ready(struct espi_hc_context *ctx)
{
	hc = ctx;
	atomic_set(&link_up, 1);
	LOG_INF("hvpggl shell ready: 'hvpggl help'");
}

static int set_pin(const struct shell *sh, int p, uint8_t lvl)
{
	int ret = gpio_pin_set_dt(&pins[p], lvl);

	if (ret != 0) {
		shell_error(sh, "%s: %d", pin_name[p], ret);
		return ret;
	}
	pin_lvl[p] = lvl;
	return 0;
}

static int parse_pin(const struct shell *sh, const char *s)
{
	for (int i = 0; i < P_NUM; i++) {
		if (strcmp(s, pin_key[i]) == 0) {
			return i;
		}
	}
	shell_error(sh, "pin must be vcc|s0ix|slp, got '%s'", s);
	return -EINVAL;
}

static int parse_level(const struct shell *sh, const char *s, uint8_t *lvl)
{
	if (strcmp(s, "0") == 0 || strcmp(s, "1") == 0) {
		*lvl = (uint8_t)(s[0] - '0');
		return 0;
	}
	shell_error(sh, "level must be 0|1, got '%s'", s);
	return -EINVAL;
}

static int parse_ulong(const struct shell *sh, const char *s, unsigned long max,
		       unsigned long *out)
{
	char *end;
	unsigned long v = strtoul(s, &end, 0);

	if (*s == '\0' || *end != '\0' || v > max) {
		shell_error(sh, "bad number '%s' (max %lu)", s, max);
		return -EINVAL;
	}
	*out = v;
	return 0;
}

static void print_pins(const struct shell *sh)
{
	shell_print(sh, "VCC_PWRGD=%u PWRGD_S0IX=%u SLP_S0#=%u (%s)", pin_lvl[P_VCC],
		    pin_lvl[P_S0IX], pin_lvl[P_SLP],
		    pin_lvl[P_SLP] ? "SLP_S0# high: not in S0ix" : "SLP_S0# low: S0ix request");
}

static int cmd_pins(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	print_pins(sh);
	return 0;
}

static int cmd_pin(const struct shell *sh, size_t argc, char **argv)
{
	int p = parse_pin(sh, argv[1]);
	uint8_t lvl;

	ARG_UNUSED(argc);
	if (p < 0 || parse_level(sh, argv[2], &lvl) != 0) {
		return -EINVAL;
	}
	if (set_pin(sh, p, lvl) != 0) {
		return -EIO;
	}
	print_pins(sh);
	return 0;
}

static int cmd_pulse(const struct shell *sh, size_t argc, char **argv)
{
	int p = parse_pin(sh, argv[1]);
	unsigned long ms;
	uint8_t orig;

	ARG_UNUSED(argc);
	if (p < 0 || parse_ulong(sh, argv[2], 60000, &ms) != 0) {
		return -EINVAL;
	}
	orig = pin_lvl[p];
	if (set_pin(sh, p, !orig) != 0) {
		return -EIO;
	}
	k_msleep((int32_t)ms);
	if (set_pin(sh, p, orig) != 0) {
		return -EIO;
	}
	shell_print(sh, "%s pulsed %u -> %u -> %u for %lu ms", pin_name[p], orig, !orig, orig,
		    ms);
	return 0;
}

static int cmd_s0ix(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	if (strcmp(argv[1], "enter") == 0) {
		(void)set_pin(sh, P_SLP, 0);
	} else if (strcmp(argv[1], "exit") == 0) {
		(void)set_pin(sh, P_SLP, 1);
	} else {
		shell_error(sh, "expected enter|exit");
		return -EINVAL;
	}
	print_pins(sh);
	return 0;
}

static int put_vw(const struct shell *sh, uint8_t idx, uint8_t bit, uint8_t lvl)
{
	int ret;

	if (!atomic_get(&link_up)) {
		shell_error(sh, "eSPI link not up yet (wait for 'hvpggl shell ready')");
		return -EAGAIN;
	}

	k_mutex_lock(&espi_lock, K_FOREVER);
	ret = espi_hc_set_ct_vwire(hc, idx, bit, lvl);
	if (ret == 0) {
		ret = espi_hc_ctx_emu_put_host_index(hc, idx);
	}
	k_mutex_unlock(&espi_lock);

	if (ret != 0) {
		shell_error(sh, "PUT_VW index 0x%02x bit %u = %u: %d", idx, bit, lvl, ret);
		return ret;
	}
	shell_print(sh, "PUT_VW index 0x%02x bit %u = %u", idx, bit, lvl);
	return 0;
}

static int cmd_pltrst(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t lvl;

	ARG_UNUSED(argc);
	if (parse_level(sh, argv[1], &lvl) != 0) {
		return -EINVAL;
	}
	return put_vw(sh, VW_IDX_PLTRST, VW_BIT_PLTRST, lvl);
}

static int cmd_c10(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t lvl;

	ARG_UNUSED(argc);
	if (parse_level(sh, argv[1], &lvl) != 0) {
		return -EINVAL;
	}
	return put_vw(sh, VW_IDX_HOST_C10, VW_BIT_HOST_C10, lvl);
}

static int cmd_vw(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long idx, bit;
	uint8_t lvl;

	ARG_UNUSED(argc);
	if (parse_ulong(sh, argv[1], 0xff, &idx) != 0 || parse_ulong(sh, argv[2], 3, &bit) != 0 ||
	    parse_level(sh, argv[3], &lvl) != 0) {
		return -EINVAL;
	}
	return put_vw(sh, (uint8_t)idx, (uint8_t)bit, lvl);
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	static struct espi_hc_vw vws[16];
	uint8_t nrecv = 0;
	uint16_t st;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	print_pins(sh);
	if (!atomic_get(&link_up)) {
		shell_print(sh, "eSPI link not up yet");
		return 0;
	}

	k_mutex_lock(&espi_lock, K_FOREVER);
	ret = espi_hc_ctx_get_status(hc);
	st = hc->pkt_status;
	if (ret == 0 && (st & BIT(ESPI_STATUS_VW_AVAIL_POS)) != 0U) {
		ret = espi_hc_ctx_emu_get_vws(hc, vws, ARRAY_SIZE(vws), &nrecv);
		if (ret == 0) {
			ret = espi_hc_ctx_emu_update_t2c_vw(hc, vws, nrecv);
		}
	}
	k_mutex_unlock(&espi_lock);

	if (ret != 0) {
		shell_error(sh, "GET_STATUS/GET_VW: %d", ret);
		return ret;
	}
	shell_print(sh, "eSPI status=0x%04x%s", st,
		    (st & BIT(ESPI_STATUS_VW_AVAIL_POS)) ? " (VW available)" : "");
	for (uint8_t i = 0; i < nrecv; i++) {
		shell_print(sh, "  T2C VW index 0x%02x levels=0x%x valid=0x%x", vws[i].host_index,
			    vws[i].levels, vws[i].valid);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	hvpggl_cmds,
	SHELL_CMD_ARG(pins, NULL, "Show driven VPGGL pin levels", cmd_pins, 1, 0),
	SHELL_CMD_ARG(pin, NULL, "<vcc|s0ix|slp> <0|1> drive a Target VPGGL input", cmd_pin, 3,
		      0),
	SHELL_CMD_ARG(pulse, NULL, "<vcc|s0ix|slp> <ms> invert a pin briefly", cmd_pulse, 3, 0),
	SHELL_CMD_ARG(s0ix, NULL, "<enter|exit> SLP_S0# low/high", cmd_s0ix, 2, 0),
	SHELL_CMD_ARG(pltrst, NULL, "<0|1> PLTRST# VWire (1 = de-asserted)", cmd_pltrst, 2, 0),
	SHELL_CMD_ARG(c10, NULL, "<0|1> HOST_C10 VWire", cmd_c10, 2, 0),
	SHELL_CMD_ARG(vw, NULL, "<host_idx> <bit> <0|1> any C2T VWire", cmd_vw, 4, 0),
	SHELL_CMD_ARG(status, NULL, "Pins, GET_STATUS, Target-to-Host VWires", cmd_status, 1,
		      0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(hvpggl, &hvpggl_cmds, "Drive the Target VPGGL inputs (Host side)", NULL);

#endif /* CONFIG_SAMPLE_VPGGL_SHELL */
