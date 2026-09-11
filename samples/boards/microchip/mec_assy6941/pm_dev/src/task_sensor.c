/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_REGISTER(app_sensor_task, CONFIG_LOG_DEFAULT_LEVEL);

#define APP_TASK_SENSOR_STACK_SIZE 1024
#define APP_TASK_SENSOR_PRIORITY 7

void app_task_sensor_entry(void *p1, void *p2, void *p3)
{
	LOG_INF("Task sensor init");

	while (1) {
		LOG_INF("Task sensor wake");
		k_msleep(1000);
	}
}

/* next to last is options
 *   K_ESSENTIAL if the task is vital trigger kernel panic or system reset if task crashes
 *   K_FP_REGS if the task needs the FPU
 *   K_DSP_REGS if the task needs the DSP regs
 *   
 * last is delay
 * Using K_THREAD_NO_START means the kernel will allocate the threads stack and control block
 * bit keep it in inactive state. It will not be added to the scheduler's ready queue
 * until the app calls k_thread_start().
 */
K_THREAD_DEFINE(app_task_sensor, APP_TASK_SENSOR_STACK_SIZE, app_task_sensor_entry,
		NULL, NULL, NULL, APP_TASK_SENSOR_PRIORITY, 0, -1);
