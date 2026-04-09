/*
 * Copyright (c) 2024, Muhammad Waleed Badar
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>

const struct device *const rtc = DEVICE_DT_GET(DT_ALIAS(rtc));

static int set_date_time(const struct device *rtc)
{
	int ret = 0;
	struct rtc_time tm = {
		.tm_year = 2099 - 2000,
		.tm_mon = 12,
		.tm_mday = 31,
		.tm_hour = 23,
		.tm_min = 59,
		.tm_sec = 45,
		.tm_wday = 1,
	};

	ret = rtc_set_time(rtc, &tm);
	if (ret < 0) {
		printk("Cannot write date time: %d\n", ret);
		return ret;
	}
	return ret;
}

static int get_date_time(const struct device *rtc)
{
	int ret = 0;
	struct rtc_time tm;
uint16_t mask=0;
static uint8_t agt=0;
	ret = rtc_get_time(rtc, &tm);
	if (ret < 0) {
		printk("Cannot read date time: %d\n", ret);
		return ret;
	}
	printk("RTC date and time: %04d-%02d-%02d %02d:%02d:%02d %02d\n", tm.tm_year,
	       tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, tm.tm_wday);
	agt++;
	if (agt%10 == 0){
rtc_alarm_get_time(rtc,0,&mask, &tm);
	printk("a gtime: %04d-%02d-%02d %02d:%02d:%02d %02d mask:%x\n", tm.tm_year,
	       tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, tm.tm_wday,mask);
	agt=0;
	}


	return ret;
}

void cb(const struct device *dev, uint16_t id, void *user_data)
{
        struct rtc_time tm;
printk("aCB\n");
rtc_get_time(rtc, &tm);
tm.tm_min = 59;
tm.tm_sec = 40;
tm.tm_hour  = 23;
tm.tm_wday = 1;
	rtc_set_time(rtc, &tm);

}

int main(void)
{
        struct rtc_time tm = {
                .tm_year = 0,
                .tm_mon = 1,
                .tm_mday = 1,
                .tm_hour = 0,
                .tm_min = 0,
                .tm_sec = 45,
		.tm_wday = 2,
        };

	/* Check if the RTC is ready */
	if (!device_is_ready(rtc)) {
		printk("Device is not ready\n");
		return 0;
	}

	set_date_time(rtc);
	rtc_alarm_set_callback(rtc, 0, cb, 0);
	//trigger alarm every hr at 0min and 45sec
rtc_alarm_set_time(rtc, 0,RTC_ALARM_TIME_MASK_WEEKDAY|RTC_ALARM_TIME_MASK_MINUTE|RTC_ALARM_TIME_MASK_SECOND , &tm);
	/* Continuously read the current date and time from the RTC */
	while (get_date_time(rtc) == 0) {
		k_sleep(K_MSEC(1000));
	};
	return 0;
}
