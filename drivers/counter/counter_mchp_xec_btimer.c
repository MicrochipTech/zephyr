/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_counter_btmr

/**
 * @file
 * @brief Microchip XEC Counter using a pair of basic timers.
 *
 * This is the driver for the 16/32-bit counters on the Microchip SoCs.
 *
 * Notes:
 * - First basic timer is free running in count up direction.
 * - Second basic timer is used to generate alarms (interrupts).
 * - The second timer is configured to count down.
 * - Second timer interrupt occurs when the count down reaches 0.
 */

#include <soc.h>
#include <stdbool.h>
#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(counter_mchp_xec, CONFIG_COUNTER_LOG_LEVEL);

#define XEC_BT_DEBUG_ISR

#define XEC_BT_CNT_OFS			0
#define XEC_BT_PRLD_OFS			4u
#define XEC_BT_SR_OFS			8u

#define XEC_BT_IER_OFS			0xcu
#define XEC_BT_SR_IER_EV_POS		0

#define XEC_BT_CR_OFS			0x10u
#define XEC_BT_CR_EN_POS		0
#define XEC_BT_CR_DIR_UP_POS		2
#define XEC_BT_CR_ARS_POS		3
#define XEC_BT_CR_SRST_POS		4 /* W/O self-clearing */
#define XEC_BT_CR_STA_POS		5
#define XEC_BT_CR_RLD_POS		6
#define XEC_BT_CR_HALT_POS		7
#define XEC_BT_CR_PRSC_POS		16
#define XEC_BT_CR_PRSC_MSK		GENMASK(31, 16)
#define XEC_BT_CR_PRSC_SET(n)		FIELD_PREP(XEC_BT_CR_PRSC_MSK, (n))
#define XEC_BT_CR_PRSC_GET(n)		FIELD_GET(XEC_BT_CR_PRSC_MSK, (n))

struct counter_xec_bt_config {
	struct counter_config_info info;
	void (*irq_cfg_func)(void);
	uint32_t cntr_base;
	uint32_t alarm_base;
	uint16_t prescaler;
	uint8_t cntr_girq;
	uint8_t cntr_girq_bit;
	uint8_t alarm_girq;
	uint8_t alarm_girq_bit;
	uint8_t cntr_irqn;
	uint8_t alarm_irqn;
};

struct counter_xec_bt_data {
	counter_alarm_callback_t alarm_cb;
	counter_top_callback_t top_cb;
	void *user_data;
	uint32_t top_ticks;
	uint32_t flags;
#ifdef XEC_BT_DEBUG_ISR
	volatile uint32_t counter_isr_cnt;
	volatile uint32_t alarm_isr_cnt;
#endif
};

static inline int xec_bt_cntr_is_running(uintptr_t cntr_base)
{
	uint32_t runmsk = BIT(XEC_BT_CR_EN_POS) | BIT(XEC_BT_CR_STA_POS);
	uint32_t ctrl = sys_read32(cntr_base + XEC_BT_CR_OFS);

	if ((ctrl & (runmsk | BIT(XEC_BT_CR_HALT_POS))) == runmsk) {
		return 1;
	}

	return 0;
}

static inline void xec_bt_cntr_start(uintptr_t cntr_base)
{
	uint32_t runmsk = BIT(XEC_BT_CR_EN_POS) | BIT(XEC_BT_CR_STA_POS);
	uint32_t ctrl = sys_read32(cntr_base + XEC_BT_CR_OFS);

	ctrl &= (uint32_t)~BIT(XEC_BT_CR_HALT_POS);
	ctrl |= runmsk;
	sys_write32(ctrl, cntr_base + XEC_BT_CR_OFS);
}

/* Start basic timer used as counter */
static int counter_xec_bt_start(const struct device *dev)
{
	const struct counter_xec_bt_config *drvcfg = dev->config;

	if (xec_bt_cntr_is_running(drvcfg->cntr_base) != 0) {
		return 0;
	}

	xec_bt_cntr_start(drvcfg->cntr_base);

	LOG_DBG("%p Counter started", dev);

	return 0;
}

static int counter_xec_bt_stop(const struct device *dev)
{
	const struct counter_xec_bt_config *drvcfg = dev->config;
	uint32_t rhmsk = BIT(XEC_BT_CR_EN_POS) | BIT(XEC_BT_CR_STA_POS) | BIT(XEC_BT_CR_HALT_POS);
	uint32_t cb = drvcfg->cntr_base;
	uint32_t ab = drvcfg->alarm_base;

	if (sys_test_bit(cb + XEC_BT_CR_OFS, XEC_BT_CR_EN_POS)) { /* stopped? */
		return 0;
	}

	sys_clear_bits(ab + XEC_BT_CR_OFS, rhmsk); /* stop alarm timer */
	sys_clear_bits(cb + XEC_BT_CR_OFS, rhmsk); /* stop counter timer */

	sys_write32(BIT(XEC_BT_SR_IER_EV_POS), ab + XEC_BT_SR_OFS);
	sys_write32(BIT(XEC_BT_SR_IER_EV_POS), cb + XEC_BT_SR_OFS);

	soc_ecia_girq_status_clear(drvcfg->cntr_girq, drvcfg->cntr_girq_bit);
	soc_ecia_girq_status_clear(drvcfg->alarm_girq, drvcfg->alarm_girq_bit);

	LOG_DBG("%p Counter stopped", dev);

	return 0;
}

static int counter_xec_bt_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_xec_bt_config *drvcfg = dev->config;

	if (ticks == NULL) {
		return -EINVAL;
	}

	*ticks = sys_read32(drvcfg->cntr_base + XEC_BT_CNT_OFS);

	return 0;
}

static void alarm_prog(mem_addr_t ab, uint32_t alarm_ticks)
{
	uint32_t ctrl = sys_read32(ab + XEC_BT_CR_OFS) & XEC_BT_CR_PRSC_MSK;

	sys_write32(ctrl, ab + XEC_BT_CR_OFS);
	sys_clear_bit(ab + XEC_BT_IER_OFS, XEC_BT_SR_IER_EV_POS);
	sys_write32(BIT(XEC_BT_SR_IER_EV_POS), ab + XEC_BT_SR_OFS);
	sys_write32(alarm_ticks, ab + XEC_BT_CNT_OFS);
	sys_write32(alarm_ticks, ab + XEC_BT_PRLD_OFS);
	sys_set_bit(ab + XEC_BT_IER_OFS, XEC_BT_SR_IER_EV_POS);
}

/* Wrapper in counter.h checks channel ID but does not check alarm_cfg
 * Absolute alarm: trigger alarm when counter reaches alarm_cfg->ticks.
 * Releative alarm: trigger alarm when counter is alarm_cfg->ticks beyond current counter.
 * Since we are using a separate basic timer in count down mode:
 * Absolute:
 *   The caller read the counter adjusted the value by interval it wants.
 *   We need to subtract current counter value to get the interval to program the alarm
 *   counter with.
 * Releative:
 *   Program alarm count down with alarm_cfg->ticks.
 */
#if 0
/**
 * @brief Alarm flag enabling immediate expiration when driver detects that
 *	  absolute alarm was set too late.
 *
 * Alarm callback must be called from the same context as if it was set on time.
 */
COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE

/**
 * @anchor COUNTER_GUARD_PERIOD_FLAGS
 * @name Counter guard period flags
 *
 * @brief Used by @ref counter_set_guard_period and
 *	  @ref counter_get_guard_period.
 * @{ */

/**
 * @brief Identifies guard period needed for detection of late setting of
 *	  absolute alarm (see @ref counter_set_channel_alarm).
 */
#define COUNTER_GUARD_PERIOD_LATE_TO_SET BIT(0)
#endif
static int counter_xec_bt_set_alarm(const struct device *dev, uint8_t chan_id,
				    const struct counter_alarm_cfg *alarm_cfg)
{
	const struct counter_xec_bt_config *drvcfg = dev->config;
	struct counter_xec_bt_data *data = dev->data;
	mem_addr_t cb = drvcfg->cntr_base;
 	mem_addr_t ab = drvcfg->alarm_base;
	uint32_t ticks = 0, ticks_now = 0;
/* TODO	bool dir_up = (drvcfg->info.flags & COUNTER_CONFIG_INFO_COUNT_UP) != 0; */

	if (data->alarm_cb != NULL) {
		return -EBUSY;
	}

	if ((alarm_cfg == NULL) || (alarm_cfg->callback == NULL) ||
		(alarm_cfg->ticks > data->top_ticks)) {
		return -EINVAL;
	}

	data->alarm_cb = alarm_cfg->callback;
	data->user_data = alarm_cfg->user_data;

	ticks = alarm_cfg->ticks;
	ticks_now = sys_read32(cb + XEC_BT_CNT_OFS);

	if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) != 0) { /* absolute alarm */
		ticks -= ticks_now;
		if (ticks >  drvcfg->info.max_top_value) {
			return -ETIME;
		}
	}

#ifdef XEC_BT_DEBUG_ISR
	data->alarm_isr_cnt = 0;
#endif

	alarm_prog(ab, ticks);
	xec_bt_cntr_start(ab);

	LOG_DBG("%p Counter alarm req %u ticks, set to %u ticks", dev, alarm_cfg->ticks, ticks);

	return 0;
}


static int counter_xec_bt_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	const struct counter_xec_bt_config *drvcfg = dev->config;
	struct counter_xec_bt_data *data = dev->data;
 	mem_addr_t ab = drvcfg->alarm_base;

	if (chan_id != 0) {
		LOG_ERR("Invalid channel id %u", chan_id);
		return -ENOTSUP;
	}

	sys_clear_bits(ab + XEC_BT_CR_OFS, BIT(XEC_BT_CR_EN_POS) | BIT(XEC_BT_CR_STA_POS));
	sys_clear_bit(ab + XEC_BT_IER_OFS, XEC_BT_SR_IER_EV_POS);

	data->alarm_cb = NULL;
	data->user_data = NULL;

	LOG_DBG("%p Counter alarm canceled", dev);

	return 0;
}

/* Only do alarm timer status if we don't require interrupts from freerun timer */
static uint32_t counter_xec_bt_get_pending_int(const struct device *dev)
{
	const struct counter_xec_bt_config *drvcfg = dev->config;
	mem_addr_t ab = drvcfg->alarm_base;

	if (sys_test_bit(ab + XEC_BT_SR_OFS, XEC_BT_SR_IER_EV_POS)) {
		return 1u;
	}

	return 0;
}

static uint32_t counter_xec_bt_get_top_value(const struct device *dev)
{
	struct counter_xec_bt_data *data = dev->data;

	return data->top_ticks;
}

/* Top value cannot be changed if an alarm is currently enabled.
 * Other driver allow this sequence:
 *   no alarm is current active
 *   set new top with top alarm
 *   set alarm
 *
 * Implementation:
 *  If alarm active return -EBUSY
 *  If requested top > maximum top then return -EINVAL
 *  counter timer currently not running:
 *
 * NOTE: call wrapper checks if cfg->ticks > max_top_value
 *
 */
static int counter_xec_bt_set_top_value(const struct device *dev,
					const struct counter_top_cfg *cfg)
{
	const struct counter_xec_bt_config *drvcfg = dev->config;
	struct counter_xec_bt_data *data = dev->data;
	mem_addr_t cb = drvcfg->cntr_base;
	uint32_t ticks = 0;
	int ret = 0;
	bool running = false;
	bool dir_up = (drvcfg->info.flags & COUNTER_CONFIG_INFO_COUNT_UP) != 0;

	if (data->alarm_cb != NULL) {
		return -EBUSY;
	}

	data->top_cb = cfg->callback;
	data->user_data = cfg->user_data;

	ticks = cfg->ticks;

	if (dir_up == true) {
		ticks = drvcfg->info.max_top_value - cfg->ticks;
	}

	if (xec_bt_cntr_is_running(cb) != 0) {
		running = true;
		if ((cfg->flags & COUNTER_TOP_CFG_DONT_RESET) != 0) {
			/* don't stop counter while configuring alarm */
			if (dir_up) {
				if (ticks < sys_read32(cb + XEC_BT_CNT_OFS)) {
					if ((cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) != 0) {
						running = false;
					} else {
						return -ETIME;
					}
				}
			}
		}
	}

	sys_clear_bit(cb + XEC_BT_IER_OFS, XEC_BT_SR_IER_EV_POS);

#ifdef XEC_BT_DEBUG_ISR
	data->counter_isr_cnt = 0;
#endif

	data->top_ticks = cfg->ticks;
	sys_write32(ticks, cb + XEC_BT_PRLD_OFS);

	if (running == true) {
		sys_set_bit(cb + XEC_BT_CR_OFS, XEC_BT_CR_RLD_POS);
	} else {
		sys_write32(ticks, cb + XEC_BT_CNT_OFS);
		xec_bt_cntr_start(cb);
	}

	if (data->top_cb != NULL) {
		sys_set_bit(cb + XEC_BT_CR_OFS, XEC_BT_CR_ARS_POS);
		sys_write32(XEC_BT_SR_IER_EV_POS, cb + XEC_BT_SR_OFS);
		sys_set_bit(cb + XEC_BT_IER_OFS, XEC_BT_SR_IER_EV_POS);
	}

	return ret;
}

/* ISR for basic timer used as the free run counter.
 * Interrupt is only enabled if top callback has been enabled.
 */
static void counter_xec_bt_isr(const struct device *dev)
{
	const struct counter_xec_bt_config *drvcfg = dev->config;
	struct counter_xec_bt_data *data = dev->data;
	mem_addr_t cb = drvcfg->cntr_base;

#ifdef XEC_BT_DEBUG_ISR
	data->counter_isr_cnt++;
#endif

	sys_clear_bit(cb + XEC_BT_IER_OFS, XEC_BT_SR_IER_EV_POS);
	sys_write32(BIT(XEC_BT_SR_IER_EV_POS), cb + XEC_BT_SR_OFS);
	soc_ecia_girq_status_clear(drvcfg->cntr_girq, drvcfg->cntr_girq_bit);

	LOG_DBG("%p Counter ISR", dev);

	if (data->top_cb) {
		data->top_cb(dev, data->user_data);
	}
}

/* ISR for basic timer used as alarm counter. */
static void counter_alarm_xec_bt_isr(const struct device *dev)
{
	const struct counter_xec_bt_config *drvcfg = dev->config;
	struct counter_xec_bt_data *data = dev->data;
	mem_addr_t cb = drvcfg->cntr_base;
	mem_addr_t ab = drvcfg->alarm_base;
	counter_alarm_callback_t alarm_cb = NULL;
	void *user_data = NULL;
	uint32_t ticks = 0;

#ifdef XEC_BT_DEBUG_ISR
	data->alarm_isr_cnt++;
#endif
	ticks = sys_read32(cb + XEC_BT_CNT_OFS);

	/* Alarm is one-shot, so disable interrupt and callback */
	sys_clear_bit(ab + XEC_BT_IER_OFS, XEC_BT_SR_IER_EV_POS);
	sys_write32(BIT(XEC_BT_SR_IER_EV_POS), ab + XEC_BT_SR_OFS);
	soc_ecia_girq_status_clear(drvcfg->alarm_girq, drvcfg->alarm_girq_bit);

	LOG_DBG("%p Counter ISR", dev);

	if (data->alarm_cb != NULL) {
		/* clear callback in driver data */
		alarm_cb = data->alarm_cb;
		user_data = data->user_data;

		data->alarm_cb = NULL;
		data->user_data = NULL;

		alarm_cb(dev, 0, ticks, user_data);
	}
}

static DEVICE_API(counter, counter_xec_bt_api) = {
		.start = counter_xec_bt_start,
		.stop = counter_xec_bt_stop,
		.get_value = counter_xec_bt_get_value,
		.set_alarm = counter_xec_bt_set_alarm,
		.cancel_alarm = counter_xec_bt_cancel_alarm,
		.set_top_value = counter_xec_bt_set_top_value,
		.get_pending_int = counter_xec_bt_get_pending_int,
		.get_top_value = counter_xec_bt_get_top_value,
};

static int xec_bt_is_32bit(mem_addr_t regbase)
{
	uint32_t prev = 0, val = 0;

	prev = sys_read32(regbase + XEC_BT_CNT_OFS);
	sys_write32(UINT32_MAX, regbase + XEC_BT_CNT_OFS);
	val = sys_read32(regbase + XEC_BT_CNT_OFS);
	sys_write32(prev, regbase + XEC_BT_CNT_OFS);

	if (val == UINT32_MAX) {
		return 1;
	}

	return 0;
}

static void xec_bt_config(mem_addr_t regbase, uint16_t prescale, uint32_t initial_count,
			  uint32_t flags)
{
	uint32_t ctrl = XEC_BT_CR_PRSC_SET((uint32_t)prescale);

	ctrl |= flags & 0xffu;

	if ((flags & BIT(8)) != 0) {
		sys_set_bit(regbase + XEC_BT_IER_OFS, XEC_BT_SR_IER_EV_POS);
	}

	sys_write32(initial_count, regbase + XEC_BT_CNT_OFS);
	sys_write32(initial_count, regbase + XEC_BT_PRLD_OFS);
	sys_write32(ctrl, regbase + XEC_BT_CR_OFS);
}

static int counter_xec_bt_init(const struct device *dev)
{
	const struct counter_xec_bt_config *drvcfg = dev->config;
	struct counter_xec_bt_data *data = dev->data;
	mem_addr_t cb = drvcfg->cntr_base;
	mem_addr_t ab = drvcfg->alarm_base;
	uint32_t cfg_flags = 0;
	int rc = 0;

	sys_set_bit(cb + XEC_BT_CR_OFS, XEC_BT_CR_SRST_POS);
	sys_set_bit(ab + XEC_BT_CR_OFS, XEC_BT_CR_SRST_POS);

	data->flags = 0;
	if (xec_bt_is_32bit(cb)) {
		data->top_ticks = UINT32_MAX;
		data->flags |= BIT(0);
	} else {
		data->top_ticks = UINT16_MAX;
	}

	if (xec_bt_is_32bit(ab)) {
		data->flags |= BIT(1);
	}

	cfg_flags = BIT(XEC_BT_CR_EN_POS) | BIT(XEC_BT_CR_DIR_UP_POS) | BIT(XEC_BT_CR_ARS_POS);
	xec_bt_config(cb, drvcfg->prescaler, 0, cfg_flags);

	cfg_flags = BIT(XEC_BT_CR_EN_POS);
	xec_bt_config(ab, drvcfg->prescaler, UINT32_MAX, cfg_flags);

	if (drvcfg->irq_cfg_func) {
		drvcfg->irq_cfg_func();
	        rc = soc_ecia_girq_ctrl(drvcfg->cntr_girq, drvcfg->cntr_girq_bit, 1u);
		if (rc == 0) {
			rc = soc_ecia_girq_ctrl(drvcfg->alarm_girq, drvcfg->alarm_girq_bit, 1u);
		}
	}

	return rc;
}

// DT_INST_PROP_BY_PHANDLE(inst, ph, prop)
// DT_INST_PHANDLE_BY_IDX(inst, prop, idx)

#define CNTR_DT_XEC_BT_ALARM_TIMER_NODE(inst) DT_INST_PHANDLE(inst, alarm_timer)

#define CNTR_DT_XEC_BT_ALARM_GIRQ(inst) \
	DT_PROP_BY_IDX(CNTR_DT_XEC_BT_ALARM_TIMER_NODE(inst), girqs, 0)

#define CNTR_DT_XEC_BT_ALARM_GIRQ_BIT(inst) \
	DT_PROP_BY_IDX(CNTR_DT_XEC_BT_ALARM_TIMER_NODE(inst), girqs, 1)

#define CNTR_DT_XEC_BT_ALARM_IRQN(inst) DT_IRQN(CNTR_DT_XEC_BT_ALARM_TIMER_NODE(inst))

#define CNTR_DT_XEC_BT_ALARM_IRQP(inst) DT_IRQ(CNTR_DT_XEC_BT_ALARM_TIMER_NODE(inst), priority)

#define CNTR_DT_XEC_BT_ALARM_BASE(inst) DT_REG_ADDR(CNTR_DT_XEC_BT_ALARM_TIMER_NODE(inst))

#define COUNTER_XEC_BT_INIT(inst)                                                                  \
	static void counter_xec_bt_irq_config_##inst(void)                                         \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst),                                                    \
			DT_INST_IRQ(inst, priority),                                               \
			counter_xec_bt_isr,                                                        \
			DEVICE_DT_INST_GET(inst), 0);                                              \
		irq_enable(DT_INST_IRQN(inst));                                                    \
		IRQ_CONNECT(CNTR_DT_XEC_BT_ALARM_IRQN(inst),                                       \
			CNTR_DT_XEC_BT_ALARM_IRQP(inst),                                           \
			counter_alarm_xec_bt_isr,                                                  \
			DEVICE_DT_INST_GET(inst), 0);                                              \
		irq_enable(CNTR_DT_XEC_BT_ALARM_IRQN(inst));                                       \
	}                                                                                          \
	static struct counter_xec_bt_data counter_xec_bt_dev_data_##inst;                          \
                                                                                                   \
	static const struct counter_xec_bt_config counter_xec_bt_dev_config_##inst = {             \
		.info = {                                                                          \
				.max_top_value = DT_INST_PROP(inst, max_value),                    \
				.freq = DT_INST_PROP(inst, clock_frequency) /                      \
					(DT_INST_PROP(inst, prescaler) + 1),                       \
				.flags = COUNTER_CONFIG_INFO_COUNT_UP,                             \
				.channels = 1,                                                     \
		},                                                                                 \
		.irq_cfg_func = counter_xec_bt_irq_config_##inst,                                  \
		.cntr_base = DT_INST_REG_ADDR(inst),                                               \
		.alarm_base = CNTR_DT_XEC_BT_ALARM_BASE(inst),                                     \
		.prescaler = DT_INST_PROP(inst, prescaler),                                        \
		.cntr_girq = DT_INST_PROP_BY_IDX(inst, girqs, 0),                                  \
		.cntr_girq_bit = DT_INST_PROP_BY_IDX(inst, girqs, 1),                              \
		.alarm_girq = CNTR_DT_XEC_BT_ALARM_GIRQ(inst),                                     \
		.alarm_girq_bit = CNTR_DT_XEC_BT_ALARM_GIRQ_BIT(inst),                             \
		.cntr_irqn = DT_INST_IRQN(inst),                                                   \
		.alarm_irqn = CNTR_DT_XEC_BT_ALARM_IRQN(inst),                                     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, counter_xec_bt_init, NULL, &counter_xec_bt_dev_data_##inst,    \
			      &counter_xec_bt_dev_config_##inst, POST_KERNEL,                      \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_xec_bt_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_XEC_BT_INIT)
