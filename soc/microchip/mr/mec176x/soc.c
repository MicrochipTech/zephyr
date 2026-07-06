/*
 * (c) 2026 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/device.h>
#include <soc.h>

/*
 * SoC early init. Intentionally empty for the polled UART bring-up.
 *
 * It previously called soc_ecia_init() to configure the EC Interrupt Aggregator,
 * but that bus-faults on the FPGA when it writes the ECS (0x4000fc00) / ECIA
 * (0x4000e000) registers - those blocks' presence/addresses are not yet verified.
 * Polled UART does not use interrupts, so no ECIA setup is needed here. ECIA
 * initialization will be re-added when the interrupt-driven path is wired up, once
 * the ECS/ECIA register map is confirmed against the Mount Rainier address map.
 */
void soc_early_init_hook(void)
{
}
