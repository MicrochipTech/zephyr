/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2022 Microchip Technology, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <soc.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/ps2.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

/* #define USE_PS2_KB_KEYMAP */

/* PS/2 commands: KBC to Host */
#define PS2_KBC_TO_HOST_ACK		0xFAu
#define PS2_KBC_TO_HOST_SELF_TEST_PASS	0xAAu
#define PS2_KBC_TO_HOST_ECHO_RESP	0xEEu
#define PS2_KBC_TO_HOST_RESEND_REQ	0xFEu
#define PS2_KBC_TO_HOST_ERROR1		0x00u
#define PS2_KBC_TO_HOST_ERROR2		0xFFu

/* PS/2 commands: Host to KBC */
#define PS2_HOST_TO_KBC_SET_STATUS_LED	0xEDu
#define PS2_HOST_TO_KBC_ECHO		0xEEu
#define PS2_HOST_TO_KBC_SET_SCAN_CODE	0xF0u
#define PS2_HOST_TO_KBC_SET_RPT_RATE	0xF3u
#define PS2_HOST_TO_KBC_KB_ENABLE	0xF4u
#define PS2_HOST_TO_KBC_KB_DISABLE	0xF5u
#define PS2_HOST_TO_KBC_RESEND		0xFEu
#define PS2_HOST_TO_KBC_RESET		0xFFu

/* PS/2 Set Status LED data pattern */
#define PS2_SET_SCROLL_LOCK_LED		BIT(0)
#define PS2_SET_NUM_LOCK_LED		BIT(1)
#define PS2_SET_CAPS_LOCK_LED		BIT(2)

/* PS/2 Mouse: uses 3-byte packets
 * byte 0 = flags
 * byte 1 = X movement
 * byte 2 = Y movement
 */
#define PS2_MS_B0_LEFT_BUTTON		BIT(0)
#define PS2_MS_B0_RIGHT_BUTTON		BIT(1)
#define PS2_MS_B0_MIDDLE_BUTTON		BIT(2)
#define PS2_MS_B0_ALWAYS_ONE		BIT(3)
#define PS2_MS_B0_X_SIGN		BIT(4)
#define PS2_MS_B0_Y_SIGN		BIT(5)
#define PS2_MS_B0_X_OVFL		BIT(6)
#define PS2_MS_B0_Y_OVFL		BIT(7)

/* PS/2 Mouse: Host to mouse commands */
#define PS2_HOST_TO_MS_RESET		0xFFu
#define PS2_HOST_TO_MS_RESET_NST	0xF6u
#define PS2_HOST_TO_MS_RESEND_LB	0xFEu
#define PS2_HOST_TO_MS_ENABLE_REPORT	0xF4u
#define PS2_HOST_TO_MS_DISABLE_REPORT	0xF5u

#ifdef USE_PS2_KB_KEYMAP
struct ps2_keycode {
	char key;
	uint8_t num_make_codes;
	uint8_t num_break_codes;
	uint8_t make_codes[2];
	uint8_t break_codes[3];
};

/* Unfinished */
static const struct ps2_keycode ps2_kb_tbl[] = {
	{ 'A', 1u, 2u, {0x1Cu}, {0xF0u, 0x1Cu} },
	{ 'B', 1u, 2u, {0x32u}, {0xF0u, 0x32u} },
	{ 'C', 1u, 2u, {0x21u}, {0xF0u, 0x21u} },
	{ 'D', 1u, 2u, {0x23u}, {0xF0u, 0x23u} },
	{ 'E', 1u, 2u, {0x24u}, {0xF0u, 0x24u} },
	{ 'F', 1u, 2u, {0x2Bu}, {0xF0u, 0x2Bu} },
	{ 'G', 1u, 2u, {0x34u}, {0xF0u, 0x34u} },
	{ 'H', 1u, 2u, {0x33u}, {0xF0u, 0x33u} },
	{ 'I', 1u, 2u, {0x43u}, {0xF0u, 0x43u} },
	{ 'J', 1u, 2u, {0x3Bu}, {0xF0u, 0x3Bu} },
	{ 'K', 1u, 2u, {0x42u}, {0xF0u, 0x42u} },
	{ 'L', 1u, 2u, {0x4Bu}, {0xF0u, 0x4Bu} },
	{ 'M', 1u, 2u, {0x3Au}, {0xF0u, 0x3Au} },
	{ 'N', 1u, 2u, {0x31u}, {0xF0u, 0x31u} },
	{ 'O', 1u, 2u, {0x44u}, {0xF0u, 0x44u} },
	{ 'P', 1u, 2u, {0x4Du}, {0xF0u, 0x4Du} },
	{ 'Q', 1u, 2u, {0x15u}, {0xF0u, 0x15u} },
	{ 'R', 1u, 2u, {0x2Du}, {0xF0u, 0x2Du} },
	{ 'S', 1u, 2u, {0x1Bu}, {0xF0u, 0x1Bu} },
	{ 'T', 1u, 2u, {0x2Cu}, {0xF0u, 0x2Cu} },
	{ 'U', 1u, 2u, {0x3Cu}, {0xF0u, 0x3Cu} },
	{ 'V', 1u, 2u, {0x2Au}, {0xF0u, 0x2Au} },
	{ 'W', 1u, 2u, {0x1Du}, {0xF0u, 0x1Du} },
	{ 'X', 1u, 2u, {0x22u}, {0xF0u, 0x22u} },
	{ 'Y', 1u, 2u, {0x35u}, {0xF0u, 0x35u} },
	{ 'Z', 1u, 2u, {0x1Au}, {0xF0u, 0x1Au} },
	{ '0', 1u, 2u, {0x45u}, {0xF0u, 0x45u} },
	{ '1', 1u, 2u, {0x16u}, {0xF0u, 0x16u} },
	{ '2', 1u, 2u, {0x1Eu}, {0xF0u, 0x1Eu} },
	{ '3', 1u, 2u, {0x26u}, {0xF0u, 0x26u} },
	{ '4', 1u, 2u, {0x25u}, {0xF0u, 0x25u} },
	{ '5', 1u, 2u, {0x2Eu}, {0xF0u, 0x2Eu} },
	{ '6', 1u, 2u, {0x36u}, {0xF0u, 0x36u} },
	{ '7', 1u, 2u, {0x3Du}, {0xF0u, 0x3Du} },
	{ '8', 1u, 2u, {0x3Eu}, {0xF0u, 0x3Eu} },
	{ '9', 1u, 2u, {0x46u}, {0xF0u, 0x46u} },
	{ '`', 1u, 2u, {0x0Eu}, {0xF0u, 0x0Eu} },
	{ '~', 1u, 2u, {0x4Eu}, {0xF0u, 0x4Eu} },
	{ '=', 1u, 2u, {0x55u}, {0xF0u, 0x55u} },
	{ '\\', 1u, 2u, {0x5Du}, {0xF0u, 0x5Du} },
	{ (char)0x08, 1u, 2u, {0x66u}, {0xF0u, 0x66u} },
	{ ' ', 1u, 2u, {0x29u}, {0xF0u, 0x29u} },
	{ (char)0x09, 1u, 2u, {0x0Du}, {0xF0u, 0x0Du} },
};
#endif /* USE_PS2_KB_KEYMAP */

#define PS2_0_NODE	DT_NODELABEL(ps2_0)
#define PS2_1_NODE	DT_NODELABEL(ps2_1)

static const struct device *ps2_0_dev = DEVICE_DT_GET(PS2_0_NODE);
static const struct device *ps2_1_dev = DEVICE_DT_GET(PS2_1_NODE);

static volatile uint32_t spin_val;
static volatile int ret_val;

static void spin_on(uint32_t id, int rval);

static void ps2_0_cb(const struct device *dev, uint8_t data);
static void ps2_1_cb(const struct device *dev, uint8_t data);

int main(void)
{
	int ret = 0;

	LOG_INF("MEC5 PS/2 sample: board: %s", DT_N_P_compatible_IDX_0);

	if (!device_is_ready(ps2_0_dev)) {
		LOG_ERR("PS/2 target 0 device is not ready!");
		spin_on((uint32_t)__LINE__, -1000);
	}

	if (!device_is_ready(ps2_1_dev)) {
		LOG_ERR("PS/2 target 1 device is not ready!");
		spin_on((uint32_t)__LINE__, -1001);
	}

	ret = ps2_config(ps2_0_dev, ps2_0_cb);
	if (ret) {
		LOG_ERR("PS/2 target 0 config error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	ret = ps2_config(ps2_1_dev, ps2_1_cb);
	if (ret) {
		LOG_ERR("PS/2 target 1 config error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	/* this enables the PS/2 target interface */
	ret = ps2_enable_callback(ps2_0_dev);
	if (ret) {
		LOG_ERR("PS/2 target 0 enable error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	ret = ps2_enable_callback(ps2_1_dev);
	if (ret) {
		LOG_ERR("PS/2 target 1 enable error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Send KB Enable to PS/2 interface 0");
	ret = ps2_write(ps2_0_dev, PS2_HOST_TO_KBC_KB_ENABLE);
	if (ret) {
		LOG_ERR("PS/2 target 0: send KB enable error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	k_sleep(K_MSEC(1000));

	LOG_INF("Send KB enable CAPS Lock to PS/2 interface 0");
	ret = ps2_write(ps2_0_dev, PS2_HOST_TO_KBC_SET_STATUS_LED);
	if (ret) {
		LOG_ERR("PS/2 target 0: send CAPS lock error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	k_sleep(K_MSEC(3000));

	ret = ps2_write(ps2_0_dev, PS2_SET_CAPS_LOCK_LED);
	if (ret) {
		LOG_ERR("PS/2 target 0: send CAPS lock error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	k_sleep(K_MSEC(3000));

	LOG_INF("PS/2 Mouse: Send Reset");
	ret = ps2_write(ps2_1_dev, PS2_HOST_TO_MS_RESET);
	if (ret) {
		LOG_ERR("PS/2 target 1: send Reset error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	k_sleep(K_MSEC(3000));

	LOG_INF("Enable PS/2 Mouse data reporting");
	ret = ps2_write(ps2_1_dev, PS2_HOST_TO_MS_ENABLE_REPORT);
	if (ret) {
		LOG_ERR("PS/2 target 1: send Enable data reporting error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	/* trigger PM residency for suspend-to-ram */
	k_sleep(K_MSEC(3000));

	LOG_INF("Application Done (%d)", ret);
	spin_on((uint32_t)__LINE__, 0);

	return 0;
}

static void spin_on(uint32_t id, int rval)
{
	spin_val = id;
	ret_val = rval;

	log_panic(); /* flush log buffers */

	while (spin_val) {
		;
	}
}

static void ps2_0_cb(const struct device *dev, uint8_t data)
{
	LOG_INF("PS/2 target 0 CB: data = 0x%02x", data);
}

static void ps2_1_cb(const struct device *dev, uint8_t data)
{
	LOG_INF("PS/2 target 1 CB: data = 0x%02x", data);
}
