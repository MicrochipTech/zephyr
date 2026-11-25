/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Host-side `hvpggl` commands for bench-testing the Target's VPGGL (glue logic).
 *
 * Commands are read from the console UART by polling (uart_poll_in) in a
 * low-priority thread. The Zephyr shell subsystem is deliberately not used:
 * with it enabled this board printed nothing at all, while the plain
 * polled console (no UART interrupts) is the configuration that is known to
 * work here. Output goes through printk / logging as everywhere else.
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
 *   hvpggl help
 *
 * The "hvpggl" prefix is optional.
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
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
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

/* One output line; printk does not add the newline. */
#define out(...) do { printk(__VA_ARGS__); printk("\n"); } while (0)

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
	LOG_INF("hvpggl ready: type 'hvpggl help'");
}

static int set_pin(int p, uint8_t lvl)
{
	int ret = gpio_pin_set_dt(&pins[p], lvl);

	if (ret != 0) {
		out("error: %s: %d", pin_name[p], ret);
		return ret;
	}
	pin_lvl[p] = lvl;
	return 0;
}

static int parse_pin(const char *s)
{
	for (int i = 0; i < P_NUM; i++) {
		if (strcmp(s, pin_key[i]) == 0) {
			return i;
		}
	}
	out("error: pin must be vcc|s0ix|slp, got '%s'", s);
	return -EINVAL;
}

static int parse_level(const char *s, uint8_t *lvl)
{
	if (strcmp(s, "0") == 0 || strcmp(s, "1") == 0) {
		*lvl = (uint8_t)(s[0] - '0');
		return 0;
	}
	out("error: level must be 0|1, got '%s'", s);
	return -EINVAL;
}

static int parse_ulong(const char *s, unsigned long max,
		       unsigned long *out)
{
	char *end;
	unsigned long v = strtoul(s, &end, 0);

	if (*s == '\0' || *end != '\0' || v > max) {
		out("error: bad number '%s' (max %lu)", s, max);
		return -EINVAL;
	}
	*out = v;
	return 0;
}

static void print_pins(void)
{
	out("VCC_PWRGD=%u PWRGD_S0IX=%u SLP_S0#=%u (%s)", pin_lvl[P_VCC],
		    pin_lvl[P_S0IX], pin_lvl[P_SLP],
		    pin_lvl[P_SLP] ? "SLP_S0# high: not in S0ix" : "SLP_S0# low: S0ix request");
}

static int cmd_pins(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	print_pins();
	return 0;
}

static int cmd_pin(size_t argc, char **argv)
{
	int p = parse_pin(argv[1]);
	uint8_t lvl;

	ARG_UNUSED(argc);
	if (p < 0 || parse_level(argv[2], &lvl) != 0) {
		return -EINVAL;
	}
	if (set_pin(p, lvl) != 0) {
		return -EIO;
	}
	print_pins();
	return 0;
}

static int cmd_pulse(size_t argc, char **argv)
{
	int p = parse_pin(argv[1]);
	unsigned long ms;
	uint8_t orig;

	ARG_UNUSED(argc);
	if (p < 0 || parse_ulong(argv[2], 60000, &ms) != 0) {
		return -EINVAL;
	}
	orig = pin_lvl[p];
	if (set_pin(p, !orig) != 0) {
		return -EIO;
	}
	k_msleep((int32_t)ms);
	if (set_pin(p, orig) != 0) {
		return -EIO;
	}
	out("%s pulsed %u -> %u -> %u for %lu ms", pin_name[p], orig, !orig, orig,
		    ms);
	return 0;
}

static int cmd_s0ix(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	if (strcmp(argv[1], "enter") == 0) {
		(void)set_pin(P_SLP, 0);
	} else if (strcmp(argv[1], "exit") == 0) {
		(void)set_pin(P_SLP, 1);
	} else {
		out("error: expected enter|exit");
		return -EINVAL;
	}
	print_pins();
	return 0;
}

static int put_vw(uint8_t idx, uint8_t bit, uint8_t lvl)
{
	int ret;

	if (!atomic_get(&link_up)) {
		out("error: eSPI link not up yet (wait for 'hvpggl ready')");
		return -EAGAIN;
	}

	k_mutex_lock(&espi_lock, K_FOREVER);
	ret = espi_hc_set_ct_vwire(hc, idx, bit, lvl);
	if (ret == 0) {
		ret = espi_hc_ctx_emu_put_host_index(hc, idx);
	}
	k_mutex_unlock(&espi_lock);

	if (ret != 0) {
		out("error: PUT_VW index 0x%02x bit %u = %u: %d", idx, bit, lvl, ret);
		return ret;
	}
	out("PUT_VW index 0x%02x bit %u = %u", idx, bit, lvl);
	return 0;
}

static int cmd_pltrst(size_t argc, char **argv)
{
	uint8_t lvl;

	ARG_UNUSED(argc);
	if (parse_level(argv[1], &lvl) != 0) {
		return -EINVAL;
	}
	return put_vw(VW_IDX_PLTRST, VW_BIT_PLTRST, lvl);
}

static int cmd_c10(size_t argc, char **argv)
{
	uint8_t lvl;

	ARG_UNUSED(argc);
	if (parse_level(argv[1], &lvl) != 0) {
		return -EINVAL;
	}
	return put_vw(VW_IDX_HOST_C10, VW_BIT_HOST_C10, lvl);
}

static int cmd_vw(size_t argc, char **argv)
{
	unsigned long idx, bit;
	uint8_t lvl;

	ARG_UNUSED(argc);
	if (parse_ulong(argv[1], 0xff, &idx) != 0 || parse_ulong(argv[2], 3, &bit) != 0 ||
	    parse_level(argv[3], &lvl) != 0) {
		return -EINVAL;
	}
	return put_vw((uint8_t)idx, (uint8_t)bit, lvl);
}

static int cmd_status(size_t argc, char **argv)
{
	static struct espi_hc_vw vws[16];
	uint8_t nrecv = 0;
	uint16_t st;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	print_pins();
	if (!atomic_get(&link_up)) {
		out("eSPI link not up yet");
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
		out("error: GET_STATUS/GET_VW: %d", ret);
		return ret;
	}
	out("eSPI status=0x%04x%s", st,
		    (st & BIT(ESPI_STATUS_VW_AVAIL_POS)) ? " (VW available)" : "");
	for (uint8_t i = 0; i < nrecv; i++) {
		out("  T2C VW index 0x%02x levels=0x%x valid=0x%x", vws[i].host_index,
			    vws[i].levels, vws[i].valid);
	}
	return 0;
}

struct hcmd {
	const char *name;
	int (*fn)(size_t argc, char **argv);
	uint8_t argc; /* including the command name */
	const char *help;
};

static const struct hcmd hcmds[] = {
	{"pins", cmd_pins, 1, "show the driven VPGGL pin levels"},
	{"pin", cmd_pin, 3, "<vcc|s0ix|slp> <0|1>  drive a Target VPGGL input"},
	{"pulse", cmd_pulse, 3, "<vcc|s0ix|slp> <ms>  invert a pin briefly"},
	{"s0ix", cmd_s0ix, 2, "<enter|exit>  SLP_S0# low / high"},
	{"pltrst", cmd_pltrst, 2, "<0|1>  PLTRST# VWire (1 = de-asserted)"},
	{"c10", cmd_c10, 2, "<0|1>  HOST_C10 VWire"},
	{"vw", cmd_vw, 4, "<host_idx> <bit> <0|1>  any C2T VWire"},
	{"status", cmd_status, 1, "pins, GET_STATUS, Target-to-Host VWires"},
};

static void help(void)
{
	out("hvpggl commands (levels are electrical, 1 = high):");
	for (size_t i = 0; i < ARRAY_SIZE(hcmds); i++) {
		out("  hvpggl %-7s %s", hcmds[i].name, hcmds[i].help);
	}
}

#define MAX_ARGS 5

static void run_line(char *line)
{
	char *argv[MAX_ARGS];
	size_t argc = 0;
	char *save = NULL;

	for (char *t = strtok_r(line, " \t", &save); t != NULL && argc < MAX_ARGS;
	     t = strtok_r(NULL, " \t", &save)) {
		argv[argc++] = t;
	}
	if (argc == 0) {
		return;
	}
	/* The "hvpggl" prefix is optional. */
	if (strcmp(argv[0], "hvpggl") == 0) {
		memmove(&argv[0], &argv[1], (argc - 1) * sizeof(argv[0]));
		argc--;
	}
	if (argc == 0 || strcmp(argv[0], "help") == 0) {
		help();
		return;
	}
	for (size_t i = 0; i < ARRAY_SIZE(hcmds); i++) {
		if (strcmp(argv[0], hcmds[i].name) == 0) {
			if (argc != hcmds[i].argc) {
				out("usage: hvpggl %s %s", hcmds[i].name, hcmds[i].help);
				return;
			}
			(void)hcmds[i].fn(argc, argv);
			return;
		}
	}
	out("unknown command '%s' (try 'hvpggl help')", argv[0]);
}

static void reader(void *p1, void *p2, void *p3)
{
	const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	char line[64];
	size_t n = 0;
	unsigned char c;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (!device_is_ready(uart)) {
		LOG_ERR("console UART not ready: hvpggl commands disabled");
		return;
	}

	while (true) {
		if (uart_poll_in(uart, &c) != 0) {
			k_msleep(10);
			continue;
		}
		if (c == '\r' || c == '\n') {
			if (n > 0 || c == '\r') {
				printk("\n");
			}
			line[n] = '\0';
			run_line(line);
			n = 0;
			if (c == '\r') {
				printk("hvpggl> ");
			}
		} else if (c == 0x08 || c == 0x7f) {
			if (n > 0) {
				n--;
				printk("\b \b");
			}
		} else if (c >= 0x20 && c < 0x7f && n < sizeof(line) - 1) {
			line[n++] = (char)c;
			uart_poll_out(uart, c); /* echo */
		}
	}
}

/* Below main() (priority 0): main() owns the start-up sequence; commands
 * are taken as soon as it sleeps or finishes.
 */
K_THREAD_DEFINE(hvpggl_reader, 2048, reader, NULL, NULL, NULL, 7, 0, 0);

#endif /* CONFIG_SAMPLE_VPGGL_SHELL */
