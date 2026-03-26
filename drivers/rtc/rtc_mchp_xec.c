/* rtc_mchp_xec.c - Microchip XEC Real Time Clock driver */

/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <soc.h>
#include <errno.h>
#include "rtc_utils.h"

#define DT_DRV_COMPAT microchip_xec_rtc

LOG_MODULE_REGISTER(rtc_mchp_xec, CONFIG_RTC_LOG_LEVEL);

/* RTC HOURS register */
#define MCHP_RTC_HOURS_MASK       0x7F
#define MCHP_RTC_HOURS_AM_PM_MASK 0x80

/* RTC REGA register */
#define MCHP_RTC_REGA_UPDT_IN_PRGRSS_POS 7
#define MCHP_RTC_REGA_UPDT_IN_PRGRSS     BIT(MCHP_RTC_REGA_UPDT_IN_PRGRSS_POS)

/* RTC REGB register */
#define MCHP_RTC_REGB_UPDT_CYC_INHBT_POS   7
#define MCHP_RTC_REGB_UPDT_CYC_INHBT       BIT(MCHP_RTC_REGB_UPDT_CYC_INHBT_POS)
#define MCHP_RTC_REGB_PRDIC_INTR_EN_POS    6
#define MCHP_RTC_REGB_PRDIC_INTR_EN        BIT(MCHP_RTC_REGB_PRDIC_INTR_EN_POS)
#define MCHP_RTC_REGB_ALRM_INTR_EN_POS     5
#define MCHP_RTC_REGB_ALRM_INTR_EN         BIT(MCHP_RTC_REGB_ALRM_INTR_EN_POS)
#define MCHP_RTC_REGB_UPDT_END_INTR_EN_POS 4
#define MCHP_RTC_REGB_UPDT_END_INTR_EN     BIT(MCHP_RTC_REGB_UPDT_END_INTR_EN_POS)
#define MCHP_RTC_REGB_DATA_MODE_POS        2
#define MCHP_RTC_REGB_DATA_MODE            BIT(MCHP_RTC_REGB_DATA_MODE_POS)
#define MCHP_RTC_REGB_HOUR_FRMT_POS        1
#define MCHP_RTC_REGB_HOUR_FRMT            BIT(MCHP_RTC_REGB_HOUR_FRMT_POS)

/* RTC Control Register */
#define MCHP_RTC_CTL_ALRM_EN_POS 3
#define MCHP_RTC_CTL_ALRM_EN     BIT(MCHP_RTC_CTL_ALRM_EN_POS)
#define MCHP_RTC_CTL_VCI_EN_POS  2
#define MCHP_RTC_CTL_VCI_EN      BIT(MCHP_RTC_CTL_VCI_EN_POS)
#define MCHP_RTC_CTL_SFT_RST_POS 1
#define MCHP_RTC_CTL_SFT_RST_EN  BIT(MCHP_RTC_CTL_SFT_RST_POS)
#define MCHP_RTC_CTL_BLCK_EN_POS 0
#define MCHP_RTC_CTL_BLCK_EN     BIT(MCHP_RTC_CTL_BLCK_EN_POS)

/* Alarm masks */
#define MCHP_RTC_PRDIC_INTR_EN_MASK    BIT(2)
#define MCHP_RTC_ALRM_INTR_MASK        BIT(1)
#define MCHP_RTC_UPDT_END_INTR_EN_MASK BIT(0)

/**
 * Structure type to access Real Time Clock (RTC).
 */
struct rtc_regs {
	volatile uint8_t SECS;
	volatile uint8_t SECS_ALRM;
	volatile uint8_t MINS;
	volatile uint8_t MINS_ALRM;
	volatile uint8_t HRS;
	volatile uint8_t HRS_ALRM;
	volatile uint8_t DAY_WK;
	volatile uint8_t DAY_MNTH;
	volatile uint8_t MNTH;
	volatile uint8_t YEAR;
	volatile uint8_t REGA;
	volatile uint8_t REGB;
	volatile uint8_t REGC;
	volatile uint8_t REGD;
	volatile uint8_t Reserved1;
	volatile uint8_t Reserved2;
	volatile uint32_t CTL;
	volatile uint32_t WK_ALRM;
	volatile uint32_t DAY_SV_FWRD;
	volatile uint32_t DAY_SV_BCK;
	volatile uint32_t TEST_MODE;
};

struct rtc_xec_config {
	struct rtc_regs *regs;
	uint8_t girq_rtc;
	uint8_t girq_pos_rtc;
	uint8_t girq_rtc_alrm;
	uint8_t girq_pos_rtc_alrm;
	uint8_t enc_pcr;
};

struct rtc_xec_data {
	bool alrm_pending;
	bool is_bcd;
	bool is_24;
	struct k_mutex lock;
	rtc_alarm_callback cb;
};

static bool is_valid_bcd8(uint8_t v)
{
	return ((v & 0x0F) <= 9) && (((v >> 4) & 0x0F) <= 9);
}

static bool validate_rtc_alrm_time(const struct rtc_time *time_dat, struct rtc_xec_data *data)
{
	uint8_t hrs = ((uint8_t)time_dat->tm_hour) & MCHP_RTC_HOURS_MASK;
	uint8_t max_hrs = data->is_24 ? 23 : 12;

	if (data->is_bcd) {
		if (!is_valid_bcd8((uint8_t)time_dat->tm_sec) ||
		    !is_valid_bcd8((uint8_t)time_dat->tm_min) || !is_valid_bcd8(hrs)) {
			return false;
		}
	} else {
		if (((uint8_t)time_dat->tm_sec > 59) || ((uint8_t)time_dat->tm_min > 59) ||
		    (hrs > max_hrs)) {
			return false;
		}
	}

	return true;
}

static bool validate_rtc_time(const struct rtc_time *time_dat, struct rtc_xec_data *data)
{
	uint8_t hrs = ((uint8_t)time_dat->tm_hour) & MCHP_RTC_HOURS_MASK;
	uint8_t max_hrs = data->is_24 ? 23 : 12;

	if (data->is_bcd) {
		if (!is_valid_bcd8((uint8_t)time_dat->tm_sec) ||
		    !is_valid_bcd8((uint8_t)time_dat->tm_min) || !is_valid_bcd8(hrs) ||
		    !is_valid_bcd8((uint8_t)time_dat->tm_wday) ||
		    !is_valid_bcd8((uint8_t)time_dat->tm_mday) ||
		    !is_valid_bcd8((uint8_t)time_dat->tm_mon) ||
		    !is_valid_bcd8((uint8_t)time_dat->tm_year)) {
			return false;
		}
	} else {
		if (((uint8_t)time_dat->tm_sec > 59) || ((uint8_t)time_dat->tm_min > 59) ||
		    (hrs > max_hrs) || ((uint8_t)time_dat->tm_wday > 12) ||
		    ((uint8_t)time_dat->tm_mday > 31) || ((uint8_t)time_dat->tm_mon > 12) ||
		    ((uint8_t)time_dat->tm_year > 99)) {
			return false;
		}
	}

	return true;
}

static void rtc_enable(struct rtc_regs *regs, bool en)
{
	if (en) {
		regs->CTL |= MCHP_RTC_CTL_BLCK_EN;
	} else {
		regs->CTL &= ~MCHP_RTC_CTL_BLCK_EN;
	}
}

static void rtc_config_data_mode(struct rtc_regs *regs, struct rtc_xec_data *data)
{
#ifdef CONFIG_RTC_DATA_MODE_BCD
	regs->REGB &= ~MCHP_RTC_REGB_DATA_MODE;
	data->is_bcd = true;
#else
	regs->REGB |= MCHP_RTC_REGB_DATA_MODE;
	data->is_bcd = false;
#endif
}

static void rtc_enable_global_alrms(struct rtc_regs *regs, bool en)
{
	if (en) {
		regs->CTL |= MCHP_RTC_CTL_ALRM_EN;
	} else {
		regs->CTL &= ~MCHP_RTC_CTL_ALRM_EN;
	}
}

static void rtc_dis_alrm_intr(struct rtc_regs *regs, uint8_t flags)
{
	regs->REGB &= ~flags;
}

static void rtc_en_alrm_intr(struct rtc_regs *regs, uint8_t flags)
{
	regs->REGB |= flags;
}

static void rtc_enable_vci(struct rtc_regs *regs, bool en)
{
	if (en) {
		regs->CTL |= MCHP_RTC_CTL_VCI_EN;
	} else {
		regs->CTL &= ~MCHP_RTC_CTL_VCI_EN;
	}
}

static void rtc_config_hour_frmt(struct rtc_regs *regs, struct rtc_xec_data *data)
{
#ifdef CONFIG_RTC_HOUR_FORMAT_24
	regs->REGB |= MCHP_RTC_REGB_HOUR_FRMT;
	data->is_24 = true;
#else
	regs->REGB &= ~MCHP_RTC_REGB_HOUR_FRMT;
	data->is_24 = false;
#endif
}

static inline void rtc_sync_busy(struct rtc_regs *regs)
{
	while (regs->REGA & MCHP_RTC_REGA_UPDT_IN_PRGRSS) {
	}
}

static void rtc_xec_isr(const struct device *dev)
{
	struct rtc_xec_data *data = dev->data;
	struct rtc_xec_config const *cfg = dev->config;
	struct rtc_regs *regs = cfg->regs;

	sys_read8(regs->REGC);

	soc_ecia_girq_status_clear(cfg->girq_rtc, cfg->girq_pos_rtc);

	if (data->cb) {
		data->cb(dev, 0, 0);
		data->alrm_pending = false;
	}
}

static int rtc_xec_set_time(const struct device *dev, const struct rtc_time *time_dat)
{
	struct rtc_xec_data *data = dev->data;
	struct rtc_xec_config const *cfg = dev->config;
	struct rtc_regs *regs = cfg->regs;

	if (time_dat == NULL || !validate_rtc_time(time_dat, data)) {
		LOG_ERR("RTC set time failed: time_dat pointer is NULL");
		return -EINVAL;
	}

	if (!validate_rtc_time(time_dat, data)) {
		LOG_ERR("RTC time validation fail");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	regs->REGB |= MCHP_RTC_REGB_UPDT_CYC_INHBT;

	regs->SECS = (uint8_t)time_dat->tm_sec;
	regs->MINS = (uint8_t)time_dat->tm_min;
	regs->HRS = ((uint8_t)time_dat->tm_hour) & MCHP_RTC_HOURS_MASK;
	if (!data->is_24) {
		regs->HRS &= ~MCHP_RTC_HOURS_AM_PM_MASK;
		regs->HRS |= ((uint8_t)time_dat->tm_hour) & MCHP_RTC_HOURS_AM_PM_MASK;
	}
	regs->DAY_WK = (uint8_t)time_dat->tm_wday;
	regs->DAY_MNTH = (uint8_t)time_dat->tm_mday;
	regs->MNTH = (uint8_t)time_dat->tm_mon;
	regs->YEAR = (uint8_t)time_dat->tm_year;

	regs->REGB &= ~MCHP_RTC_REGB_UPDT_CYC_INHBT;

	k_mutex_unlock(&data->lock);

	return 0;
}

static int rtc_xec_get_time(const struct device *dev, struct rtc_time *time_dat)
{
	struct rtc_xec_data *data = dev->data;
	struct rtc_xec_config const *cfg = dev->config;
	struct rtc_regs *regs = cfg->regs;

	k_mutex_lock(&data->lock, K_FOREVER);

	rtc_sync_busy(cfg->regs);

	time_dat->tm_sec = regs->SECS;
	time_dat->tm_min = regs->MINS;
	time_dat->tm_hour = regs->HRS;
	time_dat->tm_wday = regs->DAY_WK;
	time_dat->tm_mday = regs->DAY_MNTH;
	time_dat->tm_mon = regs->MNTH;
	time_dat->tm_year = regs->YEAR;

	k_mutex_unlock(&data->lock);

	return 0;
}

static int rtc_xec_set_alarm_time(const struct device *dev, uint16_t alarm_id, uint16_t alarm_mask,
				  const struct rtc_time *time_dat)
{
	struct rtc_xec_data *data = dev->data;
	struct rtc_xec_config const *cfg = dev->config;
	struct rtc_regs *regs = cfg->regs;

	if (time_dat == NULL && (alarm_mask & MCHP_RTC_ALRM_INTR_MASK)) {
		LOG_ERR("RTC time_dat pointer is null");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (alarm_mask & MCHP_RTC_PRDIC_INTR_EN_MASK) {
		regs->REGA |= CONFIG_RTC_PERIODIC_ALARM_RATE_SEL;
		rtc_en_alrm_intr(cfg->regs, MCHP_RTC_REGB_PRDIC_INTR_EN);
	} else {
		rtc_dis_alrm_intr(cfg->regs, MCHP_RTC_REGB_PRDIC_INTR_EN);
	}

	if (alarm_mask & MCHP_RTC_ALRM_INTR_MASK) {
		if (!validate_rtc_alrm_time(time_dat, data)) {
			LOG_ERR("RTC time validation fail");
			return -EINVAL;
		}

		regs->SECS_ALRM = (uint8_t)time_dat->tm_sec;
		regs->MINS_ALRM = (uint8_t)time_dat->tm_min;
		regs->HRS_ALRM = ((uint8_t)time_dat->tm_hour) & MCHP_RTC_HOURS_MASK;
		if (!data->is_24) {
			regs->HRS_ALRM &= ~MCHP_RTC_HOURS_AM_PM_MASK;
			regs->HRS_ALRM |= ((uint8_t)time_dat->tm_hour) & MCHP_RTC_HOURS_AM_PM_MASK;
		}
		rtc_en_alrm_intr(cfg->regs, MCHP_RTC_REGB_ALRM_INTR_EN);
	} else {
		rtc_dis_alrm_intr(cfg->regs, MCHP_RTC_REGB_ALRM_INTR_EN);
	}

	if (alarm_mask & MCHP_RTC_UPDT_END_INTR_EN_MASK) {
		rtc_en_alrm_intr(cfg->regs, MCHP_RTC_REGB_UPDT_END_INTR_EN);
	} else {
		rtc_dis_alrm_intr(cfg->regs, MCHP_RTC_REGB_UPDT_END_INTR_EN);
	}

	if (alarm_mask) {
		data->alrm_pending = true;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

static int rtc_xec_get_alarm_time(const struct device *dev, uint16_t alarm_id, uint16_t *alarm_mask,
				  struct rtc_time *time_dat)
{
	struct rtc_xec_data *data = dev->data;
	struct rtc_xec_config const *cfg = dev->config;
	struct rtc_regs *regs = cfg->regs;

	k_mutex_lock(&data->lock, K_FOREVER);

	time_dat->tm_sec = regs->SECS;
	time_dat->tm_min = regs->MINS;
	time_dat->tm_hour = regs->HRS;

	k_mutex_unlock(&data->lock);

	return 0;
}

static int rtc_xec_get_alarm_supported_fields(const struct device *dev, uint16_t id, uint16_t *mask)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);

	*mask = (MCHP_RTC_PRDIC_INTR_EN_MASK | MCHP_RTC_ALRM_INTR_MASK |
		 MCHP_RTC_UPDT_END_INTR_EN_MASK);

	return 0;
}

static int rtc_xec_alarm_is_pending(const struct device *dev, uint16_t id)
{
	struct rtc_xec_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = data->alrm_pending ? 1 : 0;
	k_mutex_unlock(&data->lock);

	return ret;
}

static int rtc_xec_set_alarm_callback(const struct device *dev, uint16_t id,
				      rtc_alarm_callback callback, void *user_data)
{
	struct rtc_xec_data *data = dev->data;

	if (callback == NULL) {
		LOG_ERR("Callback function not assigned");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	data->cb = callback;
	k_mutex_unlock(&data->lock);

	return 0;
}

static int rtc_xec_init(const struct device *dev)
{
	struct rtc_xec_config const *cfg = dev->config;
	struct rtc_xec_data *data = dev->data;

	soc_xec_pcr_sleep_en_clear(cfg->enc_pcr);

	rtc_config_hour_frmt(cfg->regs, data);
	rtc_config_data_mode(cfg->regs, data);
	rtc_enable_global_alrms(cfg->regs, true);
	uint8_t flags = (MCHP_RTC_REGB_PRDIC_INTR_EN | MCHP_RTC_REGB_ALRM_INTR_EN |
			 MCHP_RTC_REGB_UPDT_END_INTR_EN);
	rtc_dis_alrm_intr(cfg->regs, flags);
	rtc_enable_vci(cfg->regs, true);
	rtc_enable(cfg->regs, true);

	soc_ecia_girq_ctrl(cfg->girq_rtc, cfg->girq_pos_rtc, 1);

	/* RTC Interrupt */
	IRQ_CONNECT(DT_INST_IRQN_BY_IDX(0, 0), DT_INST_IRQ_BY_IDX(0, 0, priority), rtc_xec_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN_BY_IDX(0, 0));

	return 0;
}

static DEVICE_API(rtc, rtc_xec_api) = {
	.set_time = rtc_xec_set_time,
	.get_time = rtc_xec_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = rtc_xec_get_alarm_supported_fields,
	.alarm_is_pending = rtc_xec_alarm_is_pending,
	.alarm_set_time = rtc_xec_set_alarm_time,
	.alarm_get_time = rtc_xec_get_alarm_time,
	.alarm_set_callback = rtc_xec_set_alarm_callback,
#endif /* CONFIG_RTC_ALARM */
};

#define DEV_CFG_GIRQ(inst, idx)     MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, idx))
#define DEV_CFG_GIRQ_POS(inst, idx) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, idx))

static const struct rtc_xec_config rtc_xec_config_0 = {
	.regs = (struct rtc_regs *)(DT_INST_REG_ADDR(0)),
	.girq_rtc = DEV_CFG_GIRQ(0, 0),
	.girq_pos_rtc = DEV_CFG_GIRQ_POS(0, 0),
	.girq_rtc_alrm = DEV_CFG_GIRQ(0, 1),
	.girq_pos_rtc_alrm = DEV_CFG_GIRQ_POS(0, 1),
	.enc_pcr = DT_INST_PROP(0, pcr_scr),
};

static struct rtc_xec_data rtc_xec_dev_data;

DEVICE_DT_INST_DEFINE(0, rtc_xec_init, NULL, &rtc_xec_dev_data, &rtc_xec_config_0, PRE_KERNEL_1,
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &rtc_xec_api);
