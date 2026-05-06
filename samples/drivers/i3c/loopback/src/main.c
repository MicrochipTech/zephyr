/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i3c_loopback, CONFIG_I3C_TEST_HOST_LOG_LEVEL);

/*
 * Negative delay defers thread start until k_thread_start() is called.
 * See Zephyr K_THREAD_DEFINE documentation.
 */
#define EC_WAIT_FOREVER               (-1)
#define I3C_HOST_TASK_PRIORITY        6u
#define I3C_HOST_TASK_STACK_SIZE      1024U
#define I3C_TARGET_TASK_PRIORITY      7u
#define I3C_TARGET_TASK_STACK_SIZE    1024U

BUILD_ASSERT(IS_POWER_OF_TWO(I3C_HOST_TASK_STACK_SIZE),
	     "I3C host task stack size must be a power of 2");
BUILD_ASSERT(IS_POWER_OF_TWO(I3C_TARGET_TASK_STACK_SIZE),
	     "I3C target task stack size must be a power of 2");

const struct device *const host_dev   = DEVICE_DT_GET(DT_NODELABEL(i3c0));
const struct device *const target_dev = DEVICE_DT_GET(DT_NODELABEL(i3c1));

/*
 * Single shared event object.  Bit allocation (must match host.c and target.c):
 *   BIT(0) = EVT_IBI_TIR_MDB_AE  — host IBI callback → host TC2
 *   BIT(1) = EVT_IBI_ANY         — host IBI callback → host TC2
 *   BIT(2) = EVT_IBI_HOTJOIN     — host HJ callback  → host TC2
 *   BIT(3) = EVT_DATA_RECEIVED   — target write_cb   → target echo loop
 *   BIT(4) = EVT_TARGET_READY    — target register   → host task startup
 */
K_EVENT_DEFINE(loopback_sync_events);

extern void i3c_host_xfer_task(void *p1, void *p2, void *p3);
extern void i3c_target_xfer_task(void *p1, void *p2, void *p3);
extern int  setup_i3c_host(const struct device *dev);
extern int  setup_i3c_target(const struct device *dev);

K_THREAD_DEFINE(host_task_id, I3C_HOST_TASK_STACK_SIZE,
		i3c_host_xfer_task,
		(void *)host_dev, (void *)&loopback_sync_events, NULL,
		I3C_HOST_TASK_PRIORITY,
		K_USER | K_INHERIT_PERMS | K_ESSENTIAL,
		EC_WAIT_FOREVER);

K_THREAD_DEFINE(target_task_id, I3C_TARGET_TASK_STACK_SIZE,
		i3c_target_xfer_task,
		(void *)target_dev, (void *)&loopback_sync_events, NULL,
		I3C_TARGET_TASK_PRIORITY,
		K_USER | K_INHERIT_PERMS | K_ESSENTIAL,
		EC_WAIT_FOREVER);

int main(void)
{
	/*
	 * Start the target thread first so it can register on i3c1 and
	 * post EVT_TARGET_READY before the host thread attempts
	 * i3c_device_find().
	 */
	if (setup_i3c_target(target_dev) != 0) {
		LOG_ERR("Target init failed — halting");
		return -1;
	}
	k_thread_start(target_task_id);

	if (setup_i3c_host(host_dev) != 0) {
		LOG_ERR("Host init failed — halting");
		return -1;
	}
	k_thread_start(host_task_id);

	k_sleep(K_FOREVER);
	return 0;
}
