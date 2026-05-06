/* Stub driver for Introspect I3C analyzer on the bus.
 *
 * Copyright (c) 2024 Microchip Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_i3c_fake_target

#include <zephyr/drivers/i3c.h>

struct i3c_fake_target_driver_api {
};

static const struct i3c_fake_target_driver_api i3c_fake_target_api = {};

#define I3C_MCHP_FAKE_TARGET(inst)                                                                 \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, NULL, NULL, POST_KERNEL,                           \
			      CONFIG_I3C_FAKE_TARGET_INIT_PRIORITY, &i3c_fake_target_api);

DT_INST_FOREACH_STATUS_OKAY(I3C_MCHP_FAKE_TARGET)
