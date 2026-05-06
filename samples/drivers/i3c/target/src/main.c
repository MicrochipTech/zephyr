/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i3c_target_main, CONFIG_I3C_TEST_TARGET_LOG_LEVEL);

#define EC_WAIT_FOREVER               (-1)
#define I3C_TARGET_TASK_PRIORITY      7u
#define I3C_TARGET_TASK_STACK_SIZE    1024U

BUILD_ASSERT(IS_POWER_OF_TWO(I3C_TARGET_TASK_STACK_SIZE),
	     "I3C target task stack size must be a power of 2");

const struct device *const target_dev = DEVICE_DT_GET(DT_NODELABEL(i3c1));

K_EVENT_DEFINE(i3c_target_wake_event);

extern void i3c_target_xfer_task(void *p1, void *p2, void *p3);
extern int  setup_i3c_target(const struct device *dev);

K_THREAD_DEFINE(i3c_target_task_id, I3C_TARGET_TASK_STACK_SIZE,
		i3c_target_xfer_task,
		(void *)target_dev, (void *)&i3c_target_wake_event, NULL,
		I3C_TARGET_TASK_PRIORITY,
		K_USER | K_INHERIT_PERMS | K_ESSENTIAL,
		EC_WAIT_FOREVER);

int main(void)
{
	if (setup_i3c_target(target_dev) != 0) {
		LOG_ERR("Target init failed — halting");
		return -1;
	}

	k_thread_start(i3c_target_task_id);
	k_sleep(K_FOREVER);
	return 0;
}
