/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i3c_host_main, CONFIG_I3C_TEST_HOST_LOG_LEVEL);

/* K_FOREVER can no longer be used with K_THREAD_DEFINE; use a user-defined
 * negative delay value to defer thread start until k_thread_start() is called.
 */
#define EC_WAIT_FOREVER              (-1)
#define I3C_HOST_TASK_PRIORITY       6u
#define I3C_HOST_TASK_STACK_SIZE     1024U

BUILD_ASSERT(IS_POWER_OF_TWO(I3C_HOST_TASK_STACK_SIZE),
	     "I3C host task stack size must be a power of 2");

const struct device *const host_dev = DEVICE_DT_GET(DT_NODELABEL(i3c0));

K_EVENT_DEFINE(i3c_host_wake_event);

extern void i3c_host_xfer_task(void *p1, void *p2, void *p3);
extern int  setup_i3c_host(const struct device *dev);

K_THREAD_DEFINE(i3c_host_task_id, I3C_HOST_TASK_STACK_SIZE,
		i3c_host_xfer_task,
		(void *)host_dev, (void *)&i3c_host_wake_event, NULL,
		I3C_HOST_TASK_PRIORITY,
		K_USER | K_INHERIT_PERMS | K_ESSENTIAL,
		EC_WAIT_FOREVER);

int main(void)
{
	if (setup_i3c_host(host_dev) != 0) {
		LOG_ERR("Host init failed — halting");
		return -1;
	}

	/*
	 * In the standalone host app there is no loopback target thread to
	 * post EVT_TARGET_READY.  Post it here so the host task can proceed
	 * immediately without waiting.  In the loopback app this bit is
	 * posted by the target thread after i3c_target_register() succeeds.
	 */
	k_event_post(&i3c_host_wake_event, BIT(4) /* EVT_TARGET_READY */);

	k_thread_start(i3c_host_task_id);
	k_sleep(K_FOREVER);
	return 0;
}
