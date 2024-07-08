/*
 * Copyright (c) 2022 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_bbled

#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

/* HAL */
#include <mec_bbled_api.h>

LOG_MODULE_REGISTER(led_mec5, CONFIG_LED_LOG_LEVEL);

struct mec5_bbled_config {
	struct mec_bbled_regs * const regs;
	const struct pinctrl_dev_config *pcfg;
};

/* Delay.Lo register field */
#define XEC_BBLED_MAX_PRESCALER		4095u
/* Blink mode source frequency is 32768 Hz */
#define XEC_BBLED_BLINK_CLK_SRC_HZ	32768u
/* Fblink = 32768 / (256 * (prescaler+1))
 * prescaler is 12 bit.
 * Maximum Fblink = 128 Hz or 7.8125 ms
 * Minimum Fblink = 32.25 mHz or 32000 ms
 */
#define XEC_BBLED_BLINK_PERIOD_MAX_MS	32000u
#define XEC_BBLED_BLINK_PERIOD_MIN_MS	8u

/* Enable HW breathing of the LED.
 * limits = MSB is MAXIMUM, the highest duty cycle in breathing
 *          LSB is MINIMUM, the least duty cycle in breathing
 * high_delay = in ms, hold the highest duty cycle (MAXIMUM) for this periods
 * low_delay = in ms, hold the least duty cycle (MINIMUM) for this periods
 * stepsize_interval: 
 *          high 4-bit nibble is for step size, the adjusted duty cycles for next step
 *          low 4-bit nibble is for interval, the nubmer of PWM periods to hold each step
 * This API supports 
 * 		-- Symmetric mode
 *		-- PWM is configured as an 8-bit PWM
 *		-- Clock source is the 32.768 KHz clock
 */
static int mec5_bbled_breath(const struct device *dev, uint32_t led, uint16_t limits, uint32_t high_delay,
                uint32_t low_delay, uint8_t stepsize_interval)
{
	const struct mec5_bbled_config * const config = dev->config;
	struct mec_bbled_regs * const regs = config->regs;
	struct mec_bbled_breathe_config br_cfg;
	uint32_t stepsize, interval, i;

	if (led) {
		return -EINVAL;
	}

	/* insure period will not overflow uin32_t */
	if ((high_delay > XEC_BBLED_BLINK_PERIOD_MAX_MS)
	    || (low_delay > XEC_BBLED_BLINK_PERIOD_MAX_MS)) {
		return -EINVAL;
	}

	mec_hal_bbled_mode(regs, MEC_BBLED_CONFIG_CTRL_OFF);
	/* configure breath parameters*/
	interval = 0;
	stepsize = 0;
	for (i = 0; i < 8; i++ ){
		interval += (stepsize_interval & 0x0f) << (i * 4);
		stepsize += ((stepsize_interval >> 4) & 0x0f) << (i * 4);
	}
	br_cfg.upd_intervals = interval;
	br_cfg.upd_steps = stepsize;
	br_cfg.lo_delay = low_delay >> 3;
	br_cfg.hi_delay = high_delay >> 3;
	br_cfg.min_hold = limits & 0xff;
	br_cfg.max_hold = (limits >> 8) & 0xff;
	br_cfg.pwm_width = MEC_BBLED_PWM_WIDTH_8;
	mec_hal_bbled_breathe_config(regs, &br_cfg);

    /* config symmetic mode */
	mec_hal_bbled_asym_enable(regs, 0);
    /* config breathing behavior */
	mec_hal_bbled_mode(regs, MEC_BBLED_CONFIG_CTRL_BREATH);
    /* update to new config */
	mec_hal_bbled_enable_update(regs);

	return 0;
}

/* delay_on and delay_off are in milliseconds
 * (prescale+1) = (32768 * Tblink_ms) / (256 * 1000)
 * requires caller to limit delay_on and delay_off based
 * on BBLED 32KHz minimum/maximum values.
 */
static uint32_t calc_blink_32k_prescaler(uint32_t delay_on, uint32_t delay_off)
{
	uint32_t temp = ((delay_on + delay_off) * XEC_BBLED_BLINK_CLK_SRC_HZ) / (256U * 1000U);
	uint32_t prescaler = 0;

	if (temp) {
		temp--;
		if (temp > XEC_BBLED_MAX_PRESCALER) {
			prescaler = XEC_BBLED_MAX_PRESCALER;
		} else {
			prescaler = (uint32_t)temp;
		}
	}

	return prescaler;
}

/* return duty cycle scaled to [0, 255]
 * caller must insure delay_on and delay_off are in hardware range.
 */
static uint32_t calc_blink_duty_cycle(uint32_t delay_on, uint32_t delay_off)
{
	return (256U * delay_on) / (delay_on + delay_off);
}

/* Enable HW blinking of the LED.
 * delay_on = on time in milliseconds
 * delay_off = off time in milliseconds
 * BBLED blinking mode uses an 8-bit accumulator and an 8-bit duty cycle
 * register. The duty cycle register is programmed once and the
 * accumulator is used as an 8-bit up counter.
 * The counter uses the 32768 Hz clock and is pre-scaled by the delay
 * counter. Maximum blink rate is 128Hz to 32.25 mHz (7.8 ms to 32 seconds).
 * 8-bit duty cycle values: 0x00 = full off, 0xff = full on.
 * Fblink = 32768 / ((prescale + 1) * 256)
 * HiWidth (seconds) = (1/Fblink) * (duty_cycle / 256)
 * LoWidth (seconds) = (1/Fblink) * ((1 - duty_cycle) / 256)
 * duty_cycle in [0, 1]. Register value for duty cycle is
 * scaled to [0, 255].
 * prescale is delay register low delay field, bits[11:0]
 * duty_cycle is limits register minimum field, bits[7:0]
 */
static int mec5_bbled_blink(const struct device *dev, uint32_t led,
			    uint32_t delay_on, uint32_t delay_off)
{
	const struct mec5_bbled_config * const config = dev->config;
	struct mec_bbled_regs * const regs = config->regs;
	uint32_t period, prescaler, dcs;
	struct mec_bbled_blink_config bl_cfg;

	if (led) {
		return -EINVAL;
	}

	/* insure period will not overflow uin32_t */
	if ((delay_on > XEC_BBLED_BLINK_PERIOD_MAX_MS)
	    || (delay_off > XEC_BBLED_BLINK_PERIOD_MAX_MS)) {
		return -EINVAL;
	}

	period = delay_on + delay_off;
	if ((period < XEC_BBLED_BLINK_PERIOD_MIN_MS)
	    || (period > XEC_BBLED_BLINK_PERIOD_MAX_MS)) {
		return -EINVAL;
	}

	mec_hal_bbled_mode(regs, MEC_BBLED_CONFIG_CTRL_OFF);

	prescaler = calc_blink_32k_prescaler(delay_on, delay_off);
	dcs = calc_blink_duty_cycle(delay_on, delay_off);
	bl_cfg.pwm_clk_prescaler = (uint16_t)prescaler;
	bl_cfg.duty_cycle = (uint8_t)dcs;
	bl_cfg.flags = MEC_BBLED_BLINK_CLK_SEL_32K;
	mec_hal_bbled_blink_config(regs, &bl_cfg);

	mec_hal_bbled_mode(regs, MEC_BBLED_CONFIG_CTRL_BLINK);
	mec_hal_bbled_enable_update(regs);

	return 0;
}

static int mec5_bbled_off(const struct device *dev, uint32_t led)
{
	const struct mec5_bbled_config * const config = dev->config;
	struct mec_bbled_regs * const regs = config->regs;

	if (led) {
		return -EINVAL;
	}

	mec_hal_bbled_mode(regs, MEC_BBLED_CONFIG_CTRL_OFF);

	return 0;
}

static int mec5_bbled_on(const struct device *dev, uint32_t led)
{
	const struct mec5_bbled_config * const config = dev->config;
	struct mec_bbled_regs * const regs = config->regs;

	if (led) {
		return -EINVAL;
	}

	mec_hal_bbled_mode(regs, MEC_BBLED_CONFIG_CTRL_ON);

	return 0;
}

static int mec5_bbled_init(const struct device *dev)
{
	const struct mec5_bbled_config * const config = dev->config;
	struct mec_bbled_regs * const regs = config->regs;
	int ret;

	/* soft reset, disable BBLED WDT, set clock source to default (32KHz domain) */
	mec_hal_bbled_init(regs, 0x1400);
 
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("BBLED pinctrl setup failed (%d)", ret);
	}

	return ret;
}

static const struct led_driver_api mec5_bbled_api = {
	.on			= mec5_bbled_on,
	.off		= mec5_bbled_off,
	.blink		= mec5_bbled_blink,
	.breath		= mec5_bbled_breath,
};

#define MEC5_BBLED_PINCTRL_DEF(i) PINCTRL_DT_INST_DEFINE(i)

#define MEC5_BBLED_CONFIG(i)						\
static struct mec5_bbled_config mec5_bbled_config_##i = {			\
	.regs = (struct mec_bbled_regs * const)DT_INST_REG_ADDR(i),	\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),			\
}

#define MEC5_BBLED_DEVICE(i)						\
									\
MEC5_BBLED_PINCTRL_DEF(i);						\
									\
MEC5_BBLED_CONFIG(i);							\
									\
DEVICE_DT_INST_DEFINE(i, &mec5_bbled_init, NULL,				\
		      NULL, &mec5_bbled_config_##i,			\
		      POST_KERNEL, CONFIG_LED_INIT_PRIORITY,		\
		      &mec5_bbled_api);

DT_INST_FOREACH_STATUS_OKAY(MEC5_BBLED_DEVICE)
