/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Runtime diagnostics for the generic eSPI API.
 *
 * espi_interrupt_config() is a static inline in the public header, so the
 * warning it emits has to live out of line: LOG_WRN() cannot be used from a
 * header that translation units without a registered log module include.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

/*
 * Deliberately not LOG_MODULE_REGISTER(espi): every eSPI controller driver
 * already registers that name and exactly one of them is linked at a time, so
 * a second registration would be a duplicate symbol.
 */
LOG_MODULE_REGISTER(espi_api, CONFIG_ESPI_LOG_LEVEL);

void z_espi_interrupt_config_warn(void)
{
	static atomic_t warned;

	if (atomic_set(&warned, 1) != 0) {
		return;
	}

	LOG_WRN("espi_interrupt_config(): no eSPI driver implements what this API documents");
	LOG_WRN("  its flags name device classes, not interrupt sources, so input buffer full");
	LOG_WRN("  and output buffer empty, or one ACPI EC instance of several, cannot be");
	LOG_WRN("  controlled separately");
	LOG_WRN("  it sets the whole word absolutely, with no mask and no companion getter, so");
	LOG_WRN("  a caller cannot change one source without shadowing driver owned state");
	LOG_WRN("  a driver that ignores a flag still reports success");
}
