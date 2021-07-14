/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_ps2

#include <errno.h>
#include <device.h>
#include <drivers/ps2.h>
#include <soc.h>
#include <logging/log.h>

#define LOG_LEVEL CONFIG_PS2_LOG_LEVEL
LOG_MODULE_REGISTER(ps2_mchp_xec);

/* in 50us units */
#define PS2_TIMEOUT 10000

/* Each PS/2 has an activity interrupt and up to two port wake interrupts */
#define PS2_XEC_INTR_SRCS 3

/* index for ps2_xec_config intr[] */
enum ps2_xec_intr_src {
	actv = 0,
	p0wk,
	p1wk,
	intr_max
};

struct ps2_xec_intr_config {
	uint8_t isr_nvic;
	uint8_t isr_prio;
	uint8_t girq_id;
	uint8_t girq_bit;
};

struct ps2_xec_config {
	struct ps2_regs *base;
	struct ps2_xec_intr_config intr[PS2_XEC_INTR_SRCS];
};

struct ps2_xec_data {
	ps2_callback_t callback_isr;
	struct k_sem tx_lock;
};

static int ps2_xec_configure(const struct device *dev,
			     ps2_callback_t callback_isr)
{
	const struct ps2_xec_config *config = dev->config;
	struct ps2_xec_data *data = dev->data;
	struct ps2_regs *base = config->base;

	uint8_t  __attribute__((unused)) dummy;

	if (!callback_isr) {
		return -EINVAL;
	}

	data->callback_isr = callback_isr;

	/* In case the self test for a PS2 device already finished and
	 * set the SOURCE bit to 1 we clear it before enabling the
	 * interrupts. Instances must be allocated before the BAT or
	 * the host may time out.
	 */
	dummy = base->TRX_BUFF;
	base->STATUS = MCHP_PS2_STATUS_RW1C_MASK;
	mchp_soc_ecia_girq_src_clr(config->intr[actv].girq_id,
				   config->intr[actv].girq_bit);

	/* Enable FSM and init instance in rx mode*/
	base->CTRL = MCHP_PS2_CTRL_EN_POS;

	/* We enable the interrupts in the EC aggregator so that the
	 * result  can be forwarded to the ARM NVIC
	 */
	mchp_soc_ecia_girq_src_en(config->intr[actv].girq_id,
				  config->intr[actv].girq_bit);

	k_sem_give(&data->tx_lock);

	return 0;
}


static int ps2_xec_write(const struct device *dev, uint8_t value)
{
	const struct ps2_xec_config *config = dev->config;
	struct ps2_xec_data *data = dev->data;
	struct ps2_regs *base = config->base;
	int i = 0;

	uint8_t  __attribute__((unused)) dummy;

	if (k_sem_take(&data->tx_lock, K_NO_WAIT)) {
		return -EACCES;
	}
	/* Allow the PS2 controller to complete a RX transaction. This
	 * is because the channel may be actively receiving data.
	 * In addition, it is necessary to wait for a previous TX
	 * transaction to complete. The PS2 block has a single
	 * FSM.
	 */
	while (((base->STATUS &
		(MCHP_PS2_STATUS_RX_BUSY | MCHP_PS2_STATUS_TX_IDLE))
		!= MCHP_PS2_STATUS_TX_IDLE) && (i < PS2_TIMEOUT)) {
		k_busy_wait(50);
		i++;
	}

	if (unlikely(i == PS2_TIMEOUT)) {
		LOG_DBG("PS2 write timed out");
		return -ETIMEDOUT;
	}

	/* Inhibit ps2 controller and clear status register */
	base->CTRL = 0x00;

	/* Read to clear data ready bit in the status register*/
	dummy = base->TRX_BUFF;
	k_sleep(K_MSEC(1));
	base->STATUS = MCHP_PS2_STATUS_RW1C_MASK;

	/* Switch the interface to TX mode and enable state machine */
	base->CTRL = MCHP_PS2_CTRL_TR_TX | MCHP_PS2_CTRL_EN;

	/* Write value to TX/RX register */
	base->TRX_BUFF = value;

	k_sem_give(&data->tx_lock);

	return 0;
}

static int ps2_xec_inhibit_interface(const struct device *dev)
{
	const struct ps2_xec_config *config = dev->config;
	struct ps2_xec_data *data = dev->data;
	struct ps2_regs *base = config->base;

	if (k_sem_take(&data->tx_lock, K_MSEC(10)) != 0) {
		return -EACCES;
	}

	base->CTRL = 0x00;
	mchp_soc_ecia_girq_src_clr(config->intr[actv].girq_id,
				   config->intr[actv].girq_bit);
	NVIC_ClearPendingIRQ(config->intr[actv].isr_nvic);

	k_sem_give(&data->tx_lock);

	return 0;
}

static int ps2_xec_enable_interface(const struct device *dev)
{
	const struct ps2_xec_config *config = dev->config;
	struct ps2_xec_data *data = dev->data;
	struct ps2_regs *base = config->base;

	mchp_soc_ecia_girq_src_clr(config->intr[actv].girq_id,
				   config->intr[actv].girq_bit);
	base->CTRL = MCHP_PS2_CTRL_EN;

	k_sem_give(&data->tx_lock);

	return 0;
}

/*
 * GIRQ sources bits are latched R/W1C. Clear PS2 controller status before
 * clearing GIRQ source bit.
 */
static void ps2_xec_isr(const struct device *dev)
{
	const struct ps2_xec_config *config = dev->config;
	struct ps2_xec_data *data = dev->data;
	struct ps2_regs *base = config->base;
	uint32_t status;

	/* Read and clear status */
	status = base->STATUS;

	if (status & MCHP_PS2_STATUS_RXD_RDY) {
		base->CTRL = 0x00;
		if (data->callback_isr) {
			data->callback_isr(dev, base->TRX_BUFF);
		}
	} else if (status &
		    (MCHP_PS2_STATUS_TX_TMOUT | MCHP_PS2_STATUS_TX_ST_TMOUT)) {
		/* Clear sticky bits and go to read mode */
		base->STATUS = MCHP_PS2_STATUS_RW1C_MASK;
		LOG_ERR("TX time out: %0x", status);
	}

	mchp_soc_ecia_girq_src_clr(config->intr[actv].girq_id,
				   config->intr[actv].girq_bit);

	/* The control register reverts to RX automatically after
	 * transmiting the data
	 */
	base->CTRL = MCHP_PS2_CTRL_EN;
}

static const struct ps2_driver_api ps2_xec_driver_api = {
	.config = ps2_xec_configure,
	.read = NULL,
	.write = ps2_xec_write,
	.disable_callback = ps2_xec_inhibit_interface,
	.enable_callback = ps2_xec_enable_interface,
};

#ifdef CONFIG_PS2_XEC_0
static int ps2_xec_init_0(const struct device *dev);

static const struct ps2_xec_config ps2_xec_config_0 = {
	.base = (struct ps2_regs *) DT_INST_REG_ADDR(0),
	.intr[0] = {
		.isr_nvic = DT_INST_IRQN(0),
		.isr_prio = DT_INST_IRQ(0, priority),
		.girq_id = DT_INST_PROP_BY_IDX(0, girqs, 0),
		.girq_bit = DT_INST_PROP_BY_IDX(0, girqs, 1),
	},
	.intr[1] = {
		.isr_nvic = DT_INST_IRQ_BY_IDX(0, 1, irq),
		.isr_prio = DT_INST_IRQ_BY_IDX(0, 1, priority),
		.girq_id = DT_INST_PROP_BY_IDX(0, girqs, 2),
		.girq_bit = DT_INST_PROP_BY_IDX(0, girqs, 3),
	},
	.intr[2] = {
		.isr_nvic = DT_INST_IRQ_BY_IDX(0, 2, irq),
		.isr_prio = DT_INST_IRQ_BY_IDX(0, 2, priority),
		.girq_id = DT_INST_PROP_BY_IDX(0, girqs, 4),
		.girq_bit = DT_INST_PROP_BY_IDX(0, girqs, 5),
	},
};

static struct ps2_xec_data ps2_xec_port_data_0;

DEVICE_DT_INST_DEFINE(0,
		    &ps2_xec_init_0,
		    NULL,
		    &ps2_xec_port_data_0, &ps2_xec_config_0,
		    POST_KERNEL, CONFIG_PS2_INIT_PRIORITY,
		    &ps2_xec_driver_api);


static int ps2_xec_init_0(const struct device *dev)
{
	ARG_UNUSED(dev);

	struct ps2_xec_data *data = dev->data;

	k_sem_init(&data->tx_lock, 0, 1);

	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    ps2_xec_isr, DEVICE_DT_INST_GET(0), 0);

	irq_enable(DT_INST_IRQN(0));

	return 0;
}
#endif /* CONFIG_PS2_XEC_0 */

#ifndef CONFIG_SOC_SERIES_MEC172X
#ifdef CONFIG_PS2_XEC_1
static int ps2_xec_init_1(const struct device *dev);

static const struct ps2_xec_config ps2_xec_config_1 = {
	.base = (struct ps2_regs *) DT_INST_REG_ADDR(1),
	.intr[0] = {
		.isr_nvic = DT_INST_IRQN(1),
		.isr_prio = DT_INST_IRQ(1, priority),
		.girq_id = DT_INST_PROP_BY_IDX(1, girqs, 0),
		.girq_bit = DT_INST_PROP_BY_IDX(1, girqs, 1),
	},
	.intr[1] = {
		.isr_nvic = DT_INST_IRQ_BY_IDX(1, 1, irq),
		.isr_prio = DT_INST_IRQ_BY_IDX(1, 1, priority),
		.girq_id = DT_INST_PROP_BY_IDX(1, girqs, 2),
		.girq_bit = DT_INST_PROP_BY_IDX(1, girqs, 3),
	},
	.intr[2] = {
		.isr_nvic = DT_INST_IRQ_BY_IDX(1, 2, irq),
		.isr_prio = DT_INST_IRQ_BY_IDX(1, 2, priority),
		.girq_id = DT_INST_PROP_BY_IDX(1, girqs, 4),
		.girq_bit = DT_INST_PROP_BY_IDX(1, girqs, 5),
	},
};

static struct ps2_xec_data ps2_xec_port_data_1;

DEVICE_DT_INST_DEFINE(1,
		    &ps2_xec_init_1,
		    NULL,
		    &ps2_xec_port_data_1, &ps2_xec_config_1,
		    POST_KERNEL, CONFIG_PS2_INIT_PRIORITY,
		    &ps2_xec_driver_api);

static int ps2_xec_init_1(const struct device *dev)
{
	ARG_UNUSED(dev);

	struct ps2_xec_data *data = dev->data;

	k_sem_init(&data->tx_lock, 0, 1);

	IRQ_CONNECT(DT_INST_IRQN(1),
		    DT_INST_IRQ(1, priority),
		    ps2_xec_isr, DEVICE_DT_INST_GET(1), 0);

	irq_enable(DT_INST_IRQN(1));

	return 0;
}
#endif /* CONFIG_PS2_XEC_1 */
#endif /* CONFIG_SOC_SERIES_MEC172X */
