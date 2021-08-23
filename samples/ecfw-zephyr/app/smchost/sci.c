/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr.h>
#include <device.h>
#include <soc.h>
#include <logging/log.h>
#include "sci.h"
#include "smc.h"
#include "smchost.h"
#include "pwrplane.h"
#include "espi_hub.h"
#include "acpi.h"
LOG_MODULE_REGISTER(sci, CONFIG_SMCHOST_LOG_LEVEL);

struct acpi_state_flags g_acpi_state_flags;

K_MSGQ_DEFINE(sci_msgq, sizeof(uint8_t), SCIQ_SIZE, sizeof(uint8_t));

void sci_queue_init(void)
{
	g_acpi_state_flags.sci_enabled = 1;
}

/* System control interrupt are used to notify OS of ACPI events,
 * Do not send SCI when not in acpi mode or system is in Sx.
 * SCI is a pulse so need to send a eSPI virtual wire packet with zero then
 * then another eSPI VW packet with one.
 * eSPI driver should guarantee both virtual are transmitted, a delay between
 * packets should never be added here.
 */
void generate_sci(void)
{
	int ret;

	if ((!g_acpi_state_flags.sci_enabled) ||
	    (!g_acpi_state_flags.acpi_mode)) {
		return;
	}

	if (pwrseq_system_state() == SYSTEM_S0_STATE) {
#ifdef CONFIG_SMCHOST_ACPI_CRITICAL_SECTION
		uint8_t ccr;

		ccr = CPU_GET_INTERRUPT_STATE();
		CPU_DISABLE_INTERRUPTS();
#endif

#ifdef CONFIG_SMCHOST_SCI_OVER_ESPI
		ret = espihub_send_vw(ESPI_VWIRE_SIGNAL_SCI, ESPIHUB_VW_LOW);
		if (ret) {
			LOG_WRN("SCI failed");
		}

		ret = espihub_send_vw(ESPI_VWIRE_SIGNAL_SCI, ESPIHUB_VW_HIGH);
		if (ret) {
			LOG_WRN("SCI failed");
		}
#else
#warning "SCI using physical pin not supported"
#endif


#ifdef CONFIG_SMCHOST_ACPI_CRITICAL_SECTION
		CPU_SET_INTERRUPT_STATE(ccr);
#endif
	}
}

void check_sci_queue(void)
{
	if ((!g_acpi_state_flags.sci_enabled) ||
	    (!g_acpi_state_flags.acpi_mode)) {
		return;
	}

	if (k_msgq_num_used_get(&sci_msgq) == 0) {
		acpi_set_flag(ACPI_EC_0, ACPI_FLAG_SCIEVENT, 0);
	} else {
		LOG_DBG("SCI pending");
		acpi_set_flag(ACPI_EC_0, ACPI_FLAG_SCIEVENT, 1);
		generate_sci();
	}
}

void send_sci_events(void)
{
	int ret;
	uint8_t evt_byte = 0;

	if (!g_acpi_state_flags.acpi_mode) {
		return;
	}

	if (acpi_get_flag(ACPI_EC_0, ACPI_FLAG_OBF)) {
		LOG_WRN("No SCI sent. Some other transaction in progress");
		return;
	}

	ret = k_msgq_get(&sci_msgq, &evt_byte, K_NO_WAIT);
	if (ret < 0) {
		switch (ret) {
		case -ENOMSG:
			LOG_DBG("SCI msgq Empty!");
			break;
		default:
			LOG_ERR("SCI Queue Fetch failure %02x", ret);
			return;
		}
	}

	acpi_write_odr(ACPI_EC_0, evt_byte);
	if (ret == -ENOMSG) {
		acpi_set_flag(ACPI_EC_0, ACPI_FLAG_SCIEVENT, 0);
	}
	LOG_INF("Sent SCI %02x", evt_byte);
	generate_sci();
}

void enqueue_sci(uint8_t code)
{
	int ret;

	if ((!g_acpi_state_flags.sci_enabled) ||
	    (!g_acpi_state_flags.acpi_mode)) {
		return;
	}

	if (pwrseq_system_state() == SYSTEM_S0_STATE) {
		LOG_INF("enqueued SCI %02x", code);
		ret = k_msgq_put(&sci_msgq, (void *)&code, K_NO_WAIT);
		if (ret < 0) {
			LOG_WRN("SCI msgq Put failure %02x", ret);
		}
	}
}
