/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Sample exercising the Microchip MEC5 I2C Network-Layer (DMA) driver
 * and its companion per-port MUX. The overlay wires two MUX child
 * ports on distinct SCL/SDA pin pairs and clock frequencies; the app
 * alternates transfers between them so the MUX has to stop+reset the
 * controller, reapply PINCTRL, and reprogram port/timing each switch.
 *
 * The sample is self-contained: it does not require a real peripheral
 * on the bus. Each transfer targets an arbitrary 7-bit address so the
 * attempt will NAK on open lines. A NAK is a valid hardware response
 * (it proves the controller and DMA completed a full START-to-STOP);
 * the log distinguishes "ACKed" / "NAKed (no device)" / "bus error".
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(mec5_mux_switch, LOG_LEVEL_INF);

#define PORT0_NODE  DT_NODELABEL(i2c_port0)
#define PORT3_NODE  DT_NODELABEL(i2c_port3)

BUILD_ASSERT(DT_NODE_HAS_STATUS(PORT0_NODE, okay),
	     "i2c_port0 node must be enabled in overlay");
BUILD_ASSERT(DT_NODE_HAS_STATUS(PORT3_NODE, okay),
	     "i2c_port3 node must be enabled in overlay");

#define DUMMY_ADDR_A 0x50  /* arbitrary unassigned 7-bit addr */
#define DUMMY_ADDR_B 0x51

static const char *describe(int rc)
{
	switch (rc) {
	case 0:       return "ACKed";
	case -ENXIO:  return "NAKed (no device)";
	case -EIO:    return "bus error";
	case -ETIMEDOUT: return "timeout";
	default:      return "unknown error";
	}
}

static void do_write(const struct device *bus, const char *tag,
		     uint16_t addr)
{
	uint8_t buf[4] = { 0xA0, 0xDE, 0xAD, 0xBE };
	int rc = i2c_write(bus, buf, sizeof(buf), addr);

	LOG_INF("%s  4B write  @0x%02x -> %s (%d)",
		tag, addr, describe(rc), rc);
}

static void do_read(const struct device *bus, const char *tag,
		    uint16_t addr)
{
	uint8_t buf[8] = {0};
	int rc = i2c_read(bus, buf, sizeof(buf), addr);

	LOG_INF("%s  8B read   @0x%02x -> %s (%d)",
		tag, addr, describe(rc), rc);
}

static void do_write_read(const struct device *bus, const char *tag,
			  uint16_t addr)
{
	uint8_t reg = 0x00;
	uint8_t rx[4] = {0};
	int rc = i2c_write_read(bus, addr, &reg, 1, rx, sizeof(rx));

	LOG_INF("%s  wr-rd     @0x%02x -> %s (%d)",
		tag, addr, describe(rc), rc);
}

static void do_big_write(const struct device *bus, const char *tag,
			 uint16_t addr)
{
	/* Larger than the 64-byte DMA buffer in the overlay, so the
	 * controller-TX chunking path fires (multiple DMA reloads
	 * across a single START-to-STOP on the wire).
	 */
	static uint8_t big[200];
	for (size_t i = 0; i < sizeof(big); i++) {
		big[i] = (uint8_t)i;
	}

	int rc = i2c_write(bus, big, sizeof(big), addr);

	LOG_INF("%s  200B write @0x%02x -> %s (%d)",
		tag, addr, describe(rc), rc);
}

int main(void)
{
	const struct device *port0 = DEVICE_DT_GET(PORT0_NODE);
	const struct device *port3 = DEVICE_DT_GET(PORT3_NODE);

	LOG_INF("mec5 mux switch sample");

	if (!device_is_ready(port0)) {
		LOG_ERR("port0 device not ready");
		return -ENODEV;
	}
	if (!device_is_ready(port3)) {
		LOG_ERR("port3 device not ready");
		return -ENODEV;
	}

	LOG_INF("port0 = %s (100 kHz)", port0->name);
	LOG_INF("port3 = %s (400 kHz)", port3->name);

	for (unsigned int round = 0; round < 3; round++) {
		LOG_INF("--- round %u ---", round);

		do_write(port0,      "port 0", DUMMY_ADDR_A);
		do_read(port3,       "port 3", DUMMY_ADDR_B);
		do_write_read(port0, "port 0", DUMMY_ADDR_A);
		do_write_read(port3, "port 3", DUMMY_ADDR_B);
		do_big_write(port0,  "port 0", DUMMY_ADDR_A);

		k_sleep(K_MSEC(50));
	}

	LOG_INF("sample done");
	return 0;
}
