/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * PM device sample application
 * UART1 is used as the Zephyr console and shell
 */

#include <soc.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/mchp-xec-pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/state.h>

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define ZEPHYR_CONSOLE_UART_NODE DT_CHOSEN(zephyr_console)

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

/* Pin(s) needed to check PM suspend */
PINCTRL_DT_DEFINE(ZEPHYR_USER_NODE);

const bool uart1_is_wakeup_source = DT_PROP_OR(ZEPHYR_CONSOLE_UART_NODE, wakeup_source, 0);

int main(void)
{
	int rc = 0;

#ifdef CONFIG_BOARD_QUALIFIERS
	LOG_INF("Microchip XEC PM device: board: %s/%s", CONFIG_BOARD, CONFIG_BOARD_QUALIFIERS);
#else
	LOG_INF("Microchip XEC I2Cv3 targ_mode: board: %s", CONFIG_BOARD);
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_CONSOLE_UART_NODE, compatible)
	LOG_INF("UART console compatible: %s",
		DT_PROP_BY_IDX(ZEPHYR_CONSOLE_UART_NODE, compatible, 0));
#else
	LOG_INF("UART console node does not have a compatible!");
#endif

	const struct pinctrl_dev_config *zu_pcfg = PINCTRL_DT_DEV_CONFIG_GET(ZEPHYR_USER_NODE);

	rc = pinctrl_apply_state(zu_pcfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("Failed to configure GPIO for TEST_CLK_OUT (%d)", rc);
	}

	while (true) {
		LOG_INF("sleep for 5000 milliseconds");

		k_sleep(K_MSEC(5000));

		LOG_INF("Wake from sleep");
		log_flush();
	}

	LOG_INF("Program End (rc=%d)", rc);
	log_flush();

	return 0;
}
