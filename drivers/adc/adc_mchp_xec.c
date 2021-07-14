/*
 * Copyright (c) 2019 Intel Corporation.
 * Copyright (c) 2021 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_adc

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(adc_mchp_xec);

#include <drivers/adc.h>
#include <soc.h>
#include <errno.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define XEC_ADC_VREF_ANALOG 3300

/* ADC Control Register */
#define XEC_ADC_CTRL_SINGLE_DONE_STATUS		BIT(7)
#define XEC_ADC_CTRL_REPEAT_DONE_STATUS		BIT(6)
#define XER_ADC_CTRL_SOFT_RESET			BIT(4)
#define XEC_ADC_CTRL_POWER_SAVER_DIS		BIT(3)
#define XEC_ADC_CTRL_START_REPEAT		BIT(2)
#define XEC_ADC_CTRL_START_SINGLE		BIT(1)
#define XEC_ADC_CTRL_ACTIVATE			BIT(0)

struct adc_xec_config {
	uintptr_t base;
	uint8_t irq_num;
	uint8_t irq_pri;
	uint8_t girq_sngl;
	uint8_t girq_sngl_pos;
	uint8_t girq_rpt;
	uint8_t girq_rpt_pos;
};

struct adc_xec_data {
	struct adc_context ctx;
	const struct device *adc_dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
};

struct adc_xec_regs {
	uint32_t control_reg;
	uint32_t delay_reg;
	uint32_t status_reg;
	uint32_t single_reg;
	uint32_t repeat_reg;
	uint32_t channel_read_reg[8];
	uint32_t unused[18];
	uint32_t config_reg;
	uint32_t vref_channel_reg;
	uint32_t vref_control_reg;
	uint32_t sar_control_reg;
};

#define ADC_XEC_CONFIG(_dev)				\
	(((const struct adc_xec_config * const)		\
	  _dev->config))

#define ADC_XEC_DATA(_dev)				\
	((struct adc_xec_data *)_dev->data)

#define ADC_XEC_REG_BASE(_dev)				\
	((struct adc_xec_regs *)			\
	 ((const struct adc_xec_config * const)		\
	  _dev->config)->base)

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_xec_data *data = CONTAINER_OF(ctx, struct adc_xec_data, ctx);
	struct adc_xec_regs *regs = ADC_XEC_REG_BASE(data->adc_dev);

	data->repeat_buffer = data->buffer;

	regs->single_reg = ctx->sequence.channels;
	regs->control_reg |= XEC_ADC_CTRL_START_SINGLE;
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct adc_xec_data *data = CONTAINER_OF(ctx, struct adc_xec_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int adc_xec_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	struct adc_xec_regs *regs = ADC_XEC_REG_BASE(dev);
	uint32_t r;

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		return -EINVAL;
	}

	if (channel_cfg->channel_id > MCHP_ADC_CHANNELS) {
		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		return -EINVAL;
	}

	/* Setup VREF */
	r = regs->vref_channel_reg;
	r &= ~MCHP_ADC_CH_VREF_SEL_MASK(channel_cfg->channel_id);

	if (channel_cfg->reference == ADC_REF_INTERNAL) {
		r |= MCHP_ADC_CH_VREF_SEL_PAD(channel_cfg->channel_id);
	} else if (channel_cfg->reference == ADC_REF_EXTERNAL0) {
		r |= MCHP_ADC_CH_VREF_SEL_GPIO(channel_cfg->channel_id);
	} else {
		return -EINVAL;
	}

	regs->vref_channel_reg = r;

	/* Differential mode? */
	r = regs->sar_control_reg;
	r &= ~BIT(MCHP_ADC_SAR_CTRL_SELDIFF_POS);
	if (channel_cfg->differential != 0) {
		r |= MCHP_ADC_SAR_CTRL_SELDIFF_EN;
	}
	regs->sar_control_reg = r;

	return 0;
}

static bool adc_xec_validate_buffer_size(const struct adc_sequence *sequence)
{
	int chan_count = 0;
	size_t buff_need;
	uint32_t chan_mask;

	for (chan_mask = 0x80; chan_mask != 0; chan_mask >>= 1) {
		if (chan_mask & sequence->channels) {
			chan_count++;
		}
	}

	buff_need = chan_count * sizeof(uint16_t);

	if (sequence->options) {
		buff_need *= 1 + sequence->options->extra_samplings;
	}

	if (buff_need > sequence->buffer_size) {
		return false;
	}

	return true;
}

static int adc_xec_start_read(const struct device *dev,
			      const struct adc_sequence *sequence)
{
	struct adc_xec_data *data = ADC_XEC_DATA(dev);
	struct adc_xec_regs *regs = ADC_XEC_REG_BASE(dev);
	uint32_t r;

	if (sequence->channels & ~BIT_MASK(MCHP_ADC_CHANNELS)) {
		LOG_ERR("Incorrect channels, bitmask 0x%x", sequence->channels);
		return -EINVAL;
	}

	if (sequence->channels == 0UL) {
		LOG_ERR("No channel selected");
		return -EINVAL;
	}

	if (!adc_xec_validate_buffer_size(sequence)) {
		LOG_ERR("Incorrect buffer size");
		return -ENOMEM;
	}

	/* Setup ADC resolution */
	r = regs->sar_control_reg;
	r &= ~(MCHP_ADC_SAR_CTRL_RES_MASK |
	       (1 << MCHP_ADC_SAR_CTRL_SHIFTD_POS));

	if (sequence->resolution == 12) {
		r |= MCHP_ADC_SAR_CTRL_RES_12_BITS;
	} else if (sequence->resolution == 10) {
		r |= MCHP_ADC_SAR_CTRL_RES_10_BITS;
		r |= MCHP_ADC_SAR_CTRL_SHIFTD_EN;
	} else {
		return -EINVAL;
	}

	regs->sar_control_reg = r;

	data->buffer = sequence->buffer;

	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int adc_xec_read(const struct device *dev,
			const struct adc_sequence *sequence)
{
	struct adc_xec_data *data = ADC_XEC_DATA(dev);
	int error;

	adc_context_lock(&data->ctx, false, NULL);
	error = adc_xec_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

#if defined(CONFIG_ADC_ASYNC)
static int adc_xec_read_async(const struct device *dev,
			      const struct adc_sequence *sequence,
			      struct k_poll_signal *async)
{
	struct adc_xec_data *data = ADC_XEC_DATA(dev);
	int error;

	adc_context_lock(&data->ctx, true, async);
	error = adc_xec_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif /* CONFIG_ADC_ASYNC */

static void xec_adc_get_sample(const struct device *dev)
{
	struct adc_xec_data *data = ADC_XEC_DATA(dev);
	struct adc_xec_regs *regs = ADC_XEC_REG_BASE(dev);
	uint32_t idx;
	uint32_t channels = regs->status_reg;
	uint32_t ch_status = channels;
	uint32_t bit;

	/*
	 * Using the enabled channel bit set, from
	 * lowest channel number to highest, find out
	 * which channel is enabled and copy the ADC
	 * values from hardware registers to the data
	 * buffer.
	 */
	bit = find_lsb_set(channels);
	while (bit != 0) {
		idx = bit - 1;

		*data->buffer = (uint16_t)regs->channel_read_reg[idx];
		data->buffer++;

		channels &= ~BIT(idx);
		bit = find_lsb_set(channels);
	}

	/* Clear the status register */
	regs->status_reg = ch_status;
}

static void adc_xec_isr(const struct device *dev)
{
	const struct adc_xec_config *cfg = ADC_XEC_CONFIG(dev);
	struct adc_xec_data *data = ADC_XEC_DATA(dev);
	struct adc_xec_regs *regs = ADC_XEC_REG_BASE(dev);
	uint32_t ctrl;

	/* Clear START_SINGLE bit and clear SINGLE_DONE_STATUS */
	ctrl = regs->control_reg;
	ctrl &= ~XEC_ADC_CTRL_START_SINGLE;
	ctrl |= XEC_ADC_CTRL_SINGLE_DONE_STATUS;
	regs->control_reg = ctrl;

	/* Also clear GIRQ source status bit */
	mchp_soc_ecia_girq_src_clr(cfg->girq_sngl, cfg->girq_sngl_pos);

	xec_adc_get_sample(dev);

	adc_context_on_sampling_done(&data->ctx, dev);

	LOG_DBG("ADC ISR triggered.");
}

struct adc_driver_api adc_xec_api = {
	.channel_setup = adc_xec_channel_setup,
	.read = adc_xec_read,
#if defined(CONFIG_ADC_ASYNC)
	.read_async = adc_xec_read_async,
#endif
	.ref_internal = XEC_ADC_VREF_ANALOG,
};

static int adc_xec_init(const struct device *dev)
{
	const struct adc_xec_config *cfg = ADC_XEC_CONFIG(dev);
	struct adc_xec_data *data = ADC_XEC_DATA(dev);
	struct adc_xec_regs *regs = ADC_XEC_REG_BASE(dev);

	mchp_pcr_periph_slp_ctrl(PCR_ADC_0, 0);

	data->adc_dev = dev;

	regs->control_reg = XEC_ADC_CTRL_ACTIVATE
			    | XEC_ADC_CTRL_POWER_SAVER_DIS
			    | XEC_ADC_CTRL_SINGLE_DONE_STATUS
			    | XEC_ADC_CTRL_REPEAT_DONE_STATUS;

	mchp_soc_ecia_girq_src_clr(cfg->girq_sngl, cfg->girq_sngl_pos);
	mchp_soc_ecia_girq_src_en(cfg->girq_sngl, cfg->girq_sngl_pos);

	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    adc_xec_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(cfg->irq_num);

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static struct adc_xec_data adc_xec_dev_data_0 = {
	ADC_CONTEXT_INIT_TIMER(adc_xec_dev_data_0, ctx),
	ADC_CONTEXT_INIT_LOCK(adc_xec_dev_data_0, ctx),
	ADC_CONTEXT_INIT_SYNC(adc_xec_dev_data_0, ctx),
};

static const struct adc_xec_config adc_xec_config_0 = {
	.base = DT_INST_REG_ADDR(0),
	.irq_num = DT_INST_IRQN(0),
	.irq_pri = DT_INST_IRQ(0, priority),
	.girq_sngl = DT_INST_PROP_BY_IDX(0, girqs, 0),
	.girq_sngl_pos = DT_INST_PROP_BY_IDX(0, girqs, 1),
	.girq_rpt = DT_INST_PROP_BY_IDX(0, girqs, 2),
	.girq_rpt_pos = DT_INST_PROP_BY_IDX(0, girqs, 3),
};

DEVICE_DT_INST_DEFINE(0, adc_xec_init, NULL,
		    &adc_xec_dev_data_0, &adc_xec_config_0,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &adc_xec_api);
