/*
 * Copyright (c) 2019 Intel Corporation
 * Copyright (c) 2022 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/gpio.h>
LOG_MODULE_DECLARE(pwm, CONFIG_PWM_LOG_LEVEL);

#define K_WAIT_DELAY          100u

/* The devicetree node identifier for the board power rails pins. */
#define BRD_PWR_NODE DT_NODELABEL(board_power)


static const struct device *pwm_inst0_dev = DEVICE_DT_GET(DT_NODELABEL(pwm0));
static const struct device *tach_inst0_dev = DEVICE_DT_GET(DT_NODELABEL(tach3));

#define PWM_INST0_NODE DT_NODELABEL(pwm0)
#define PWM_INST0_BASE_ADDR DT_REG_ADDR(PWM_INST0_NODE)

#define TACH_INST0_NODE DT_NODELABEL(tach3)
#define TACH_INST0_BASE_ADDR DT_REG_ADDR(TACH_INST0_NODE)

#define SLEEP_TIME_MS   50

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static void print_bbled_regs(mem_addr_t bbled_base)
{
	uint32_t r = 0;

	if (!bbled_base) {
		return;
	}

	LOG_INF("BBLED @ 0x%lx", bbled_base);

	r = sys_read32(bbled_base);
	LOG_INF("config = 0x%x", r);
	r = sys_read32(bbled_base + 4U);
	LOG_INF("limits = 0x%x", r);
	r = sys_read32(bbled_base + 8U);
	LOG_INF("delay = 0x%x", r);
	r = sys_read32(bbled_base + 0xcU);
	LOG_INF("update_step_size = 0x%x", r);
	r = sys_read32(bbled_base + 0x10U);
	LOG_INF("update_interval = 0x%x", r);
	r = sys_read32(bbled_base + 0x14U);
	LOG_INF("output_delay = 0x%x", r);
}

static void print_pwm_regs(mem_addr_t pwm_base)
{
	uint32_t r = 0;

	if (!pwm_base) {
		return;
	}

	LOG_INF("PWM @ 0x%lx", pwm_base);

	r = sys_read32(pwm_base);
	LOG_INF("counter_on  = 0x%x", r);
	r = sys_read32(pwm_base + 4U);
	LOG_INF("counter_off = 0x%x", r);
	r = sys_read32(pwm_base + 8U);
	LOG_INF("config = 0x%x", r);
}

int pwm_test(void)
{
	int ret = 0;
	uint64_t pwm_reported_cycles_per_sec;
	uint32_t pwm_period_cycles, pwm_period_ns;
	uint32_t pwm_pulse_width_cycles, pwm_pulse_width_ns;
	pwm_flags_t pwm_flags;
	struct sensor_value sV;
	/* Account for the time serial port is detected so log messages can
	 * be seen
	 */
	k_sleep(K_SECONDS(1));

	if (!device_is_ready(pwm_inst0_dev)) {
		LOG_ERR("%s: device not ready.", pwm_inst0_dev->name);
		return -ENODEV;
	}

	if (!device_is_ready(tach_inst0_dev)) {
		LOG_ERR("%s: device not ready.", tach_inst0_dev->name);
		return -ENODEV;
	}

	LOG_INF("Try PWM Instance 0 Get cycles per second");
	pwm_reported_cycles_per_sec = 0U;
	ret = pwm_get_cycles_per_sec(pwm_inst0_dev, 0, &pwm_reported_cycles_per_sec);
	if (ret) {
		LOG_ERR("PWM Instance 0: Get cycles API error %d", ret);
		return ret;
	}
	LOG_INF("PWM Inst 0: Get cycles per sec = %llu", pwm_reported_cycles_per_sec);
	print_pwm_regs(PWM_INST0_BASE_ADDR);

	pwm_period_cycles = (uint32_t)(pwm_reported_cycles_per_sec / 100U);
	pwm_pulse_width_cycles = pwm_period_cycles / 2U;
	pwm_flags = 0;

	LOG_INF("Choose pwm_period_cycles =      %u", pwm_period_cycles);
	LOG_INF("Choose pwm_pulse_width_cycles = %u", pwm_pulse_width_cycles);

	ret = pwm_set_cycles(pwm_inst0_dev, 0, pwm_period_cycles,
			     pwm_pulse_width_cycles, pwm_flags);
	if (ret) {
		LOG_ERR("PWM Instance 1: Set cycles API error %d", ret);
		return ret;
	}
	print_pwm_regs(PWM_INST0_BASE_ADDR);

	while(1){
		for(ret=0;ret<255;ret++){
	print_pwm_regs(PWM_INST0_BASE_ADDR);
	print_pwm_regs(PWM_INST0_BASE_ADDR);
	}
	ret = sensor_sample_fetch(tach_inst0_dev);
	if(ret==0)
	{
		ret=sensor_channel_get(tach_inst0_dev,SENSOR_CHAN_RPM,&sV);
		if(ret==0)
			LOG_INF("RPM =  %u", sV.val1);

	}
	k_sleep(K_SECONDS(3));
	}
	return 0;
}

int main(void)
{
	pwm_test();
 int ret=0,ret1=0;
 struct sensor_value sV;
        if (!device_is_ready(tach_inst0_dev)) {
                LOG_ERR("%s: device not ready.", tach_inst0_dev->name);
                return -ENODEV;
        }

  if (!gpio_is_ready_dt(&led)) {
    return 0;
  }

  ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    return 0;
  }
while(1){

  for (ret1=0;ret1<4;ret1++) {
    ret = gpio_pin_toggle_dt(&led);
    if (ret < 0) {
      return 0;
    }
    k_msleep(SLEEP_TIME_MS);
  }
 ret = sensor_sample_fetch(tach_inst0_dev);
  if(ret==0)
  {
    ret=sensor_channel_get(tach_inst0_dev,SENSOR_CHAN_RPM,&sV);
    if(ret==0)
      LOG_INF("RPM =  %u", sV.val1);

  }
}
}
