/*
 * Copyright 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "stdbool.h"
#include "sys/errno.h"
#define DT_DRV_COMPAT microchip_xec_i2c_nl

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_mchp_xec_nl);

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <soc.h>
#include "i2c-priv.h"

#define XEC_DMAC_BASE (mem_addr_t)(DT_REG_ADDR(DT_NODELABEL(dmac)))

#define XEC_I2C_NL_TARGET_ADDR_NONE	0xf800u

#define XEC_I2C_NL_CFG_TM		BIT(0)

#define I2C_XEC_BB_EN_SCL_DRIVE_LO	\
	(BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CD_POS) | BIT(XEC_I2C_BBCR_CM_POS))
#define I2C_XEC_BB_EN_SCL_TRI_STATE	(BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CM_POS))

#define I2C_XEC_BB_EN_SDA_DRIVE_LO	\
	(BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_DD_POS) | BIT(XEC_I2C_BBCR_CM_POS))
#define I2C_XEC_BB_EN_SDA_TRI_STATE	(BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CM_POS))

#define I2C_XEC_BB_SCL_SDA_HIGH_MSK	\
	(BIT(XEC_I2C_BBCR_SCL_IN_POS) | BIT(XEC_I2C_BBCR_SDA_IN_POS))

/* workqueue for offloading complex I2C processing from the ISR
 * https://docs.zephyrproject.org/latest/kernel/services/threads/workqueue.html
 * Instantiate one struct k_thread and one function thread_main
 * Modify xec_i2c_nl_init to call k_sem_init() and k_thread_create()
 * k_thread_create() is passed same parameters except for pointer to
 * I2C-NL instance struct device.
 * We are using the same thread handler for each I2C-NL instance.
 * Also the same thread stack is used. Must be big enough for pre-emption
 * What is k_thread_create() allocating individually for each call?
 * !!! Example sensor/bosch/bmp388/bmp388_trigger.c uses two different thread styles
 * !!! CONFIG_BMP388_TRIGGER_GLOBAL_THREAD
 * 	driver data structure has struct k_work work in it
 * 	driver data structure has const struct device *dev pointint to this device
 * 	Implements bmp388_work_handler(struct k_work *work) calls bmp388_handler_interrupts(data->dev)
 * 	driver init sets data->work.handler = bmp388_work_handler
 * 	uses GPIO interrupt callback to call k_work_submit(&data->work)
 * 	This is using the system workqueue so we don't need to define a workqueue stack
 * 	The work function should not use blocking operations. If it does or takes too much time
 * 	then the driver will need to create its own workqueue with stack, a significant overhead
 * !!! vs
 * !!! CONFIG_BMP388_TRIGGER_OWN_THREAD
 * 	driver data structure has struct k_sem sem in it
 * 	allocates thread stack one in driver file
 * 	calls k_thread_create() in driver init function
 * 	Implements bmp388_thread_main() an infinite loop with semaphore take then call to bmp388_handler_interrupts(dev)
 * 	uses GPIO interrupt callback to call k_sem_give(&data->sem)
 * Alternative:
 * drivers/adc/adc_tla202x.c uses K_THREAD_DEFINE() invoked in
 * DT_FOREACH_STATUS_OK() driver macro.
 * This method creates an individual stack for each driver instantiation.
 */
#define XEC_I2C_NL_WQ_STACK_SIZE 256
#define XEC_I2C_NL_WQ_PRIORITY   5


enum xec_i2c_nl_cm_state {
	XEC_CM_STATE_CLOSED = 0,
	XEC_CM_STATE_DATA_TX,
	XEC_CM_STATE_DATA_RX,
};

struct xec_i2c_nl_msg_group {
	struct i2c_msg *msg1;
	struct i2c_msg *msg2;
};

struct xec_i2c_nl_config {
	mem_addr_t i2c_base;
	uint32_t bitrate;
	const struct pinctrl_dev_config *pin_cfg;
	void (*irq_config)(const struct device *);
	const struct device *cm_dma_dev;
	volatile uint8_t *cm_buf;
	uint16_t cm_buf_size;
	uint8_t cm_dma_chan;
	uint8_t cm_dma_trigsrc;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t pcr;
	uint8_t port;
#ifdef CONFIG_I2C_TARGET
	const struct device *tm_dma_dev;
	volatile uint8_t *tm_rx_buf;
	uint16_t tm_rx_buf_size;
	uint8_t tm_dma_chan;
	uint8_t tm_dma_trigsrc;
#endif
};

struct xec_i2c_nl_data {
	uint32_t i2c_config;
	volatile uint32_t i2c_status;
	uint16_t i2c_addr_fmt;
	uint8_t i2c_cr;
	struct k_mutex lock_mutex;
	struct k_sem sync_sem;
	struct xec_i2c_nl_msg_group mgrp;
	struct i2c_msg *msgs;
	struct i2c_msg *mc;
	struct i2c_msg *mend;
	uint8_t num_msgs;
	uint8_t midx;
	uint8_t cm_state;
#ifdef CONFIG_I2C_XEC_NL_USE_KWORKQUEUE
	struct k_work isr_work;
#endif
#ifdef CONFIG_I2C_TARGET
	atomic_t num_targets_registered;
	struct i2c_target_config *targets[XEC_I2C_MAX_TARGETS];
#endif
};

struct xec_i2c_nl_timing {
	uint32_t freq_hz;
	uint32_t data_timing;
	uint32_t idle_scaling;
	uint32_t timeout_scaling;
	uint16_t bus_clock;
	uint8_t  rpt_sta_hold_tm;
};

#ifdef CONFIG_I2C_XEC_NL_USE_KWORKQUEUE

static void xec_i2c_nl_isr_kworker(struct k_work *work)
{
	/* TODO */
}
#endif /* CONFIG_I2C_XEC_NL_USE_KWORKQUEUE */

static const struct xec_i2c_nl_timing xec_i2c_nl_timing_tbl[] = {
	{ KHZ(100), XEC_I2C_SMB_DATA_TM_100K, XEC_I2C_SMB_IDLE_SC_100K, XEC_I2C_SMB_TMO_SC_100K,
	  XEC_I2C_SMB_BUS_CLK_100K, XEC_I2C_SMB_RSHT_100K },
	{ KHZ(400), XEC_I2C_SMB_DATA_TM_400K, XEC_I2C_SMB_IDLE_SC_400K, XEC_I2C_SMB_TMO_SC_400K,
	  XEC_I2C_SMB_BUS_CLK_400K, XEC_I2C_SMB_RSHT_400K },
	{ MHZ(1), XEC_I2C_SMB_DATA_TM_1M, XEC_I2C_SMB_IDLE_SC_1M, XEC_I2C_SMB_TMO_SC_1M,
	  XEC_I2C_SMB_BUS_CLK_1M, XEC_I2C_SMB_RSHT_1M },
};

static int xec_i2c_nl_prog_standard_timing(const struct device *dev, uint32_t freq_hz)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t i2c_base = devcfg->i2c_base;

	for (size_t n = 0; n < ARRAY_SIZE(xec_i2c_nl_timing_tbl); n++) {
		const struct xec_i2c_nl_timing *p = &xec_i2c_nl_timing_tbl[n];

		if (freq_hz == p->freq_hz) {
			sys_write32(p->data_timing, i2c_base + XEC_I2C_DT_OFS);
			sys_write32(p->idle_scaling, i2c_base + XEC_I2C_ISC_OFS);
			sys_write32(p->timeout_scaling, i2c_base + XEC_I2C_TMOUT_SC_OFS);
			sys_write16(p->bus_clock, i2c_base + XEC_I2C_BCLK_OFS);
			sys_write8(p->rpt_sta_hold_tm, i2c_base + XEC_I2C_RSHT_OFS);

			return 0;
		}
	}

	return -EINVAL;
}

static uint32_t xec_i2c_status_get(mem_addr_t i2c_base)
{
	uint32_t v = sys_read32(i2c_base + XEC_I2C_COMP_OFS);

	v &= ~GENMASK(7, 0);
	v |= sys_read8(i2c_base + XEC_I2C_SR_OFS);

	return v;
}

#ifdef CONFIG_I2C_TARGET
static bool xec_i2c_nl_is_tm(const struct device *dev)
{
	struct xec_i2c_nl_data *const data = dev->data;

	if (atomic_get(&data->num_targets_registered) != 0) {
		return true;
	}

	return false;
}

static void prog_target_addresses(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint32_t ntargets = (uint32_t)atomic_get(&data->num_targets_registered);
	uint32_t v = sys_read32(i2c_base + XEC_I2C_OA_OFS);
	uint32_t n = 0, tidx = 0;

	for (n = 0; n < ntargets; n++) { /* hardware supports two target addresses */
		struct i2c_target_config *p = data->targets[n];

		if (p == NULL) {
			continue;
		}

		v |= XEC_I2C_OA_SET(tidx, p->address);
		tidx++;
	}

	sys_write32(v, i2c_base + XEC_I2C_OA_OFS);
}

static void xec_i2c_nl_tm_cfg1(const struct device *dev)
{
	/* TODO after I2C-NL controller reset and configuration we need to
	 * configure DMA channel used by target mode.
	 */
}
#endif

static void xec_i2c_nl_cm_cfg1(const struct device *dev)
{
	/* TODO after I2C-NL controller reset and configuration we need to
	 * configure DMA channel used by controller mode.
	 */
}

static int xec_i2c_nl_hw_cfg(const struct device *dev, uint32_t bitrate, uint8_t port,
			     uint32_t flags)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint32_t v = 0;
	int rc = 0;
	uint8_t temp8 = 0;

	soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 0);

	xec_pcr_sleep_en_clear(devcfg->pcr);
	xec_pcr_reset_en(devcfg->pcr);

	soc_ecia_girq_status_clear(devcfg->girq, devcfg->girq_pos);

	/* Clear service (PIN) */
	sys_write8(BIT(XEC_I2C_CR_PIN_POS), i2c_base + XEC_I2C_CR_OFS);

#ifdef CONFIG_I2C_TARGET
	prog_target_addresses(dev);
#endif

	rc = xec_i2c_nl_prog_standard_timing(dev, bitrate);
	if (rc != 0) {
		return rc;
	}

	/* Clear service, enable output, and enable ACK generation */
	temp8 = BIT(XEC_I2C_CR_PIN_POS) | BIT(XEC_I2C_CR_ESO_POS) | BIT(XEC_I2C_CR_ACK_POS);
	data->i2c_cr = temp8;
	sys_write8(temp8, i2c_base + XEC_I2C_CR_OFS);

	/* port mux and filter enable */
	v = XEC_I2C_CFG_PORT_SET((uint32_t)port);
	v |= BIT(XEC_I2C_CFG_FEN_POS);
	sys_write32(v, i2c_base + XEC_I2C_CFG_OFS);

	/* enable: controller begins sampling SCL/SDA pins over time inverval ?
	 * Should we delay after enable?
	 */
	sys_set_bit(i2c_base + XEC_I2C_CFG_OFS, XEC_I2C_CFG_ENAB_POS);

	/* enable bit-bang continuous pin monitor. v3.8 HW only */
	sys_set_bit(i2c_base + XEC_I2C_BBCR_OFS, XEC_I2C_BBCR_CM_POS);

	xec_i2c_nl_cm_cfg1(dev);

#ifdef CONFIG_I2C_TARGET_MODE
	if (xec_i2c_nl_is_tm(dev) == true) {
		xec_i2c_nl_tm_cfg1(dev);
	}
#endif

	return 0;
}

static uint32_t i2c_bitrate_from_config(uint32_t i2c_config)
{
	uint32_t bitrate = 0;

	switch (I2C_SPEED_GET(i2c_config)) {
	case I2C_SPEED_STANDARD:
		bitrate = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		bitrate = KHZ(400);
		break;
	case I2C_SPEED_FAST_PLUS:
		bitrate = MHZ(1);
		break;
	default:
		break;
	}

	return bitrate;
}

/* Public API */
static int xec_i2c_nl_configure(const struct device *dev, uint32_t i2c_config)
{
	struct xec_i2c_nl_data *const data = dev->data;
	int rc = 0;
	uint32_t bitrate = 0, cfg_flags = 0;
	uint8_t port = 0;

	bitrate = i2c_bitrate_from_config(i2c_config);
	if (bitrate == 0) { /* unsupported I2C speed? */
		return -EINVAL;
	}

#ifdef CONFIG_I2C_PORT_MUX
	port = I2C_PORT_MUX_GET(i2c_config);
#else
	port = devcfg->port;
#endif

	k_mutex_lock(&data->lock_mutex, K_FOREVER);

	rc = xec_i2c_nl_hw_cfg(dev, bitrate, port, cfg_flags);
	if (rc == 0) {
		data->i2c_config = i2c_config;
	}

	k_mutex_unlock(&data->lock_mutex);

	return rc;
}

static int xec_i2c_nl_get_config(const struct device *dev, uint32_t *i2c_config)
{
	struct xec_i2c_nl_data *const data = dev->data;

	if (i2c_config != NULL) {
		/* TODO more */
		/* I2C_MODE_CONTROLLER */
		*i2c_config = data->i2c_config;
	}

	return 0;
}

/* TODO mutex/semaphore while changing? */
static int xec_i2c_nl_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
#ifdef CONFIG_I2C_TARGET
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint32_t v = sys_read32(i2c_base + XEC_I2C_OA_OFS);
	uint32_t ntargets = 0;
	uint16_t taddr = 0;

	if ((cfg == NULL) || (cfg->address > 0x7FU)) {
		return -EINVAL;
	}

	ntargets = (uint32_t)atomic_get(&data->num_targets_registered);
	if (ntargets >= XEC_I2C_MAX_TARGETS) {
		return -EBUSY;
	}

	for (unsigned i = 0; i < XEC_I2C_MAX_TARGETS; i++) {
		taddr = XEC_I2C_OA_GET(i, v);

		if (taddr == 0) {
			v |= XEC_I2C_OA_SET(i, cfg->address);
			sys_write32(v, i2c_base + XEC_I2C_OA_OFS);
			atomic_inc(&data->num_targets_registered);
			break;
		}
	}

	return 0;
#else
	return -ENOTSUP;
#endif
}

static int xec_i2c_nl_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
#ifdef CONFIG_I2C_TARGET
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint32_t v = sys_read32(i2c_base + XEC_I2C_OA_OFS);
	uint32_t ntargets = 0;
	uint16_t taddr = 0;

	if (cfg == NULL) {
		return -EINVAL;
	}

	ntargets = (uint32_t)atomic_get(&data->num_targets_registered);
	if (ntargets == 0) {
		return 0;
	}

	for (unsigned i = 0; i < XEC_I2C_MAX_TARGETS; i++) {
		if (data->targets[i] == cfg) {
			for (unsigned j = 0; j < XEC_I2C_MAX_TARGETS; j++) {
				taddr = XEC_I2C_OA_GET(i, v);
				if (taddr == cfg->address) {
					v &= ~XEC_I2C_OA_MSK(i);
					sys_write32(v, i2c_base + XEC_I2C_OA_OFS);
					break;
				}
			}

			atomic_dec(&data->num_targets_registered);
			break;
		}
	}

	return 0;
#else
	return -ENOTSUP;
#endif
}

/* How can we detect if this controller is driving the bus or an external
 * device is driving SCL and/or SDA low?
 * 1. Reset this controller
 * 2. Enable bit-bang in tri-state mode.
 * 3. If line go high then this controller is no longer driving the line(s)
 *    else its an external device
 * 4. External device driving line: do recovery sequence
 *    a. SDA stuck low
 *       Drive SDA high using bit-bang HW
 *       n = 0;
 *       while (n < 10) {
 *              Read SDA
 *              if SDA == 0 {
 *                  generate clock pulse on SCL (1-0-1 transition)
 *              } else {
 *                  sda_state = 1
 *                  break
 *              }
 *       }
 *       if sda_state == 1
 *          Generate STOP using bit-bang HW
 *          report success
 *
 *    OR toggle SCL (1-0-1) 20 times and then generate a STOP
 *
 *    OR issue 10 I2C clocks
 *       >5us delay, SDA low, >5us delay, SCL low (START)
 *       >5us delay, SCL high, >5us delay, SDA high (STOP)
 *
 */
static int xec_i2c_nl_recover_bus(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	mem_addr_t i2c_base = devcfg->i2c_base;
	uint8_t port = XEC_I2C_CFG_PORT_GET(sys_read32(i2c_base + XEC_I2C_CFG_OFS));
	uint8_t bbcr = 0;

	xec_i2c_nl_hw_cfg(dev, KHZ(100), port, 0); /* reset and config controller */

	k_sleep(K_MSEC(35));

	/* enable bit-bang SCL and SDA tri-state mode. SCL and SDA are disconnected from I2C
	 * controller logic and connected to bit-bang logic. We can read the pins states via
	 * bit-bang control registers. If the either pin is low then an external device is
	 * driving the pin(s).
	 */
	bbcr = (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_SCL_POS) | BIT(XEC_I2C_BBCR_SDA_POS) |
		BIT(XEC_I2C_BBCR_CM_POS));
	sys_write8(bbcr, i2c_base + XEC_I2C_BBCR_OFS);

	bbcr = sys_read8(i2c_base + XEC_I2C_BBCR_OFS);

	if ((bbcr & BIT(XEC_I2C_BBCR_SCL_IN_POS)) == 0) {
		/* SCL is being held low by somthing. Nothing we can do */
		sys_write8(BIT(XEC_I2C_BBCR_CM_POS), i2c_base + XEC_I2C_BBCR_OFS);
		return -EIO;
	}

	if ((bbcr & BIT(XEC_I2C_BBCR_SDA_IN_POS)) != 0) {
		/* SDA and SCL are both high. We are ok. */
		sys_write8(BIT(XEC_I2C_BBCR_CM_POS), i2c_base + XEC_I2C_BBCR_OFS);
		return 0;
	}

	/* generate 10 I2C clocks at 100 KHz */
	for (int i = 0; i < 10; i++) {
		bbcr = I2C_XEC_BB_EN_SCL_DRIVE_LO | I2C_XEC_BB_EN_SDA_TRI_STATE;
		sys_write8(bbcr, i2c_base + XEC_I2C_BBCR_OFS); /* drive SCL low */
		k_busy_wait(5);
		bbcr = I2C_XEC_BB_EN_SCL_TRI_STATE | I2C_XEC_BB_EN_SDA_TRI_STATE;
		sys_write8(bbcr, i2c_base + XEC_I2C_BBCR_OFS); /* release SCL */
		k_busy_wait(5);
	}

	/* generate an I2C START */
	k_busy_wait(5);
	/* drive SDA low */
	bbcr = I2C_XEC_BB_EN_SCL_TRI_STATE | I2C_XEC_BB_EN_SDA_DRIVE_LO;
	sys_write8(bbcr, i2c_base + XEC_I2C_BBCR_OFS);
	k_busy_wait(5);
	/* drive SCL low */
	bbcr = I2C_XEC_BB_EN_SCL_DRIVE_LO | I2C_XEC_BB_EN_SDA_DRIVE_LO;
	sys_write8(bbcr, i2c_base + XEC_I2C_BBCR_OFS);
	k_busy_wait(5);

	/* generate an I2C STOP */
	k_busy_wait(5);
	/* release SCL */
	bbcr = I2C_XEC_BB_EN_SCL_TRI_STATE | I2C_XEC_BB_EN_SDA_DRIVE_LO;
	sys_write8(bbcr, i2c_base + XEC_I2C_BBCR_OFS);
	k_busy_wait(5);
	/* release SCL */
	bbcr = I2C_XEC_BB_EN_SCL_TRI_STATE | I2C_XEC_BB_EN_SDA_TRI_STATE;
	sys_write8(bbcr, i2c_base + XEC_I2C_BBCR_OFS);
	k_busy_wait(5);

	bbcr = sys_read8(i2c_base + XEC_I2C_BBCR_OFS);

	/* turn off bit-bang mode. I2C SCL and SDA are re-connect to I2C controller logic */
	sys_write8(BIT(XEC_I2C_BBCR_CM_POS), i2c_base + XEC_I2C_BBCR_OFS);

	if ((bbcr & I2C_XEC_BB_SCL_SDA_HIGH_MSK) != I2C_XEC_BB_SCL_SDA_HIGH_MSK)  {
		return -EIO;
	}

	k_sleep(K_MSEC(35));

	return 0;
}

#if 0
struct i2c_msg {
	/** Data buffer in bytes */
	uint8_t		*buf;

	/** Length of buffer in bytes */
	uint32_t	len;

	/** Flags for this message */
	uint8_t		flags;
};

	data->msgs = msgs;
	data->mc = msgs;
	data->mremptr = NULL;
	data->mremlen = 0;
	data->num_msgs = num_msgs;
	data->midx = 0;
	data->state = 0;
#endif

/* Closed, I2C Write + STOP
 * 	ntx = 1 + msg->len
 * 	nrx = 0
 * 	i2c_nl_flags = START0 | STOP
 *
 * Closed, I2C Write
 * 	ntx = max
 * 	nrx = max
 * 	i2c_nl_flags = START0
 *
 * Closed, I2C Read + STOP
 * 	ntx = 1
 * 	nrx = msg->len
 * 	i2c_nl_flags = START0 | STARTN | STOP
 *
 * Open, I2C Write
 *
 * !!! How to handle one message with STOP and data length > HW max? !!!
 * Write message:
 * 	START wrAddr [A] data[0] [A] ... data[N-1] [A] STOP
 * 	                              | HW length stops here !
 *	we can't let I2C-NL wrCnt reach <= 2.
 * 	Idea 1:
 * 		Set I2C-NL wrCnt or rdCnt to 128
 * 		Limit DMA to 64 bytes
 * 		DMA done
 * 			chunk_len = 64
 * 			if not last chunk
 * 				reprogram I2C-NL wrCnt or rdCnt to 128
 * 			else
 * 				if STOP required
 * 					chunk_len = remaining length
 * 				reprogram I2C-NL wrCnt or rdCnt to remaining length
 * 				!!! DOES NOT WORK DUE TO:
 * 				!!! DMA mem2dev stops but I2C-NL is still clocking out data
 * 				!!! we can't touch I2C-NL registers in DMA ISR while DMA is
 * 				!!! still working.
 * 				!!! Same issue for I2C read. DMA dev2mem reads from I2C-NL register
 * 				!!! which triggers I2C-NL to read-ahead. DMA can stop before I2C.
 * 				and set I2C-NL STOP flag
 * 			endif
 * 			reload DMA for chunk_len
 * 			restart DMA
 *
 * Idea 2 based on 1
 * struct xec_dma_buf {
 * 	uint8_t *buf;
 * 	uint32_t len;
 * 	uint32_t hw_data_addr;
 * 	uint8_t flags;
 * 	struct xec_dma_buf *next;
 * };
 * Set HCMD.wrCnt = 128, HCMD.rdCnt = 128, and HCMD.STOP=1
 * if CLOSED
 * 	HCMD.START0 = 1
 * 	DMA.len = 1
 * 	DMA.buf = i2c_write_addr buf
 * 	DMA.dir = Mem2Dev
 * 	dma_next.buf = m->buf
 * 	dma_next.len = m->len
 * 	dma_next.
 * else
 *
 * endif
 *
 * Parse messages into struct xec_i2c_nl_msg_group
 * One I2C Write with STOP
 * OR
 * One I2C Read with STOP
 * OR
 * Two message sequence:
 * 	Write Nw bytes
 * 	Read Nr bytes
 *
 */
#if 0
struct xec_i2c_nl_msg_group {
	struct i2c_msg *msg1;
	struct i2c_msg *msg2;
};
#endif

#if 0
struct xec_i2c_nl_msg_group mgrp;
struct i2c_msg *msgs;
struct i2c_msg *mc;
struct i2c_msg *mend;
#endif
static int xfr_msg_group_sync(const struct device *dev)
{
	/* TODO */
	return 0;
}

static bool i2c_nl_skip_msg(struct i2c_msg *m)
{
	if (m == NULL) {
		return true;
	}

	if (m->buf == NULL) {
		return true;
	}

	if (m->len == 0) {
		return true;
	}

	return true;
}

static bool is_msg_write(struct i2c_msg *m)
{
	if ((m != NULL) && ((m->flags & I2C_MSG_READ) == 0)) {
		return true;
	}

	return false;
}

static bool is_msg_read(struct i2c_msg *m)
{
	if ((m != NULL) && ((m->flags & I2C_MSG_READ) != 0)) {
		return true;
	}

	return false;
}

static int add_msg_to_group(struct xec_i2c_nl_msg_group *g, struct i2c_msg *m)
{
	if (g->msg1 == NULL) {
		g->msg1 = m;
	} else if (g->msg2 == NULL) {
		g->msg2 = m;
		return 1; /* group is full */
	} else {
		return -ENOSPC;
	}

	return 0;
}

/* API wrapper in i2c.h returns on num_msgs == 0. It also accesses msgs[] without checking for
 * NULL which will result in a fault if msgs is NULL.
 * This routine will perform a complete I2C transaction from START to STOP.
 * If no START flag is present on the first message, the driver adds START.
 * If no STOP flag is present in the last message, the driver adds STOP.
 */
static int xec_i2c_nl_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			       uint16_t addr)
{
	struct xec_i2c_nl_data *const data = dev->data;
	struct xec_i2c_nl_msg_group *g = &data->mgrp;
	struct i2c_msg *m = NULL;
	struct i2c_msg *mnext = NULL;
	int rc = 0;
	bool xfr_group = false;

#ifdef CONFIG_I2C_TARGET
	if (xec_i2c_nl_is_tm(dev) == true) {
		return -EBUSY;
	}
#endif
	if (addr > XEC_I2C_TARGET_ADDR_MSK) {
		return -EINVAL; /* hardware only supports 7-bit I2C addresses */
	}

	k_mutex_lock(&data->lock_mutex, K_FOREVER);
	pm_device_busy_set(dev);

	data->msgs = msgs;
	data->mc = msgs;
	data->mend = msgs + num_msgs;
	data->num_msgs = num_msgs;
	data->midx = 0;
	data->cm_state = 0;

	g->msg1 = NULL;
	g->msg2 = NULL;

	while (data->mc < data->mend) {
		m = data->mc;
		mnext = m + 1;
		if (mnext >= data->mend) {
			mnext = NULL;
		}

		/* ISSUE: if group has one msg in it and next message is skip
		 * then we won't transmit the msg in the group!
		 */
#if 0
		if (i2c_nl_skip_msg(m) == true) {
			data->midx++;
			m++;
			continue;
		}
#endif

		if (((is_msg_write(m) == true) && (is_msg_read(mnext) == true)) {
			xfr_group = true;
		}

		if (xfr_group == true) {
			rc = xfr_msg_group_sync(dev);
			if (rc != 0) {
				break;
			}
			xfr_group = false;
			data->midx++;
			m++;
			if (g->msg2 != NULL) {
				data->midx++;
				m++;
			}
			g->msg1 = NULL;
			g->msg2 = NULL;
		} else {
			data->midx++;
			m++;
		}
	}

	pm_device_busy_clear(dev);
	k_mutex_unlock(&data->lock_mutex);

	return rc;
}

#ifdef CONFIG_I2C_CALLBACK
static int xec_i2c_nl_transfer_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				  uint16_t addr, i2c_callback_t cb, void *userdata)
{
	/* TODO */
	return 0;
}
#endif

#ifdef CONFIG_I2C_RTIO
static int xec_i2c_nl_iodev_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	/* TODO */
	return 0;
}
#endif

/* Driver initialization */
static int xec_i2c_nl_init(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *const data = dev->data;
	enum dma_channel_filter dma_filter = DMA_CHANNEL_NORMAL;
	int rc = 0;

	data->i2c_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
#ifdef CONFIG_I2C_PORT_MUX
	data->i2c_config |= I2C_PORT_MUX_SET(devcfg->port);
#endif
#ifdef CONFIG_I2C_TARGET
	for (unsigned n = 0; n < XEC_I2C_MAX_TARGETS; n++) {
		data->targets[n] = NULL;
	}
#endif

	k_mutex_init(&data->lock_mutex);
	k_sem_init(&data->sync_sem, 0, K_SEM_MAX_LIMIT);

	if (device_is_ready(devcfg->cm_dma_dev) != 0) {
		/* Check our DMA channels are not owned by DMA driver */
		if (dma_chan_filter(devcfg->cm_dma_dev, (int)devcfg->cm_dma_chan,
		    (void *)&dma_filter) == true) {
			LOG_ERR("CM DMA channel owned by DMA driver!");
			return -ENODEV;
		}

#ifdef CONFIG_I2C_TARGET
		if (dma_chan_filter(devcfg->tm_dma_dev, (int)devcfg->tm_dma_chan,
		    (void *)&dma_filter) == true) {
			LOG_ERR("TM DMA channel owned by DMA driver!");
			return -ENODEV;
		}
#endif
	} else { /* No driver for central DMA, we initialize it */
		sys_set_bit(XEC_DMAC_BASE + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_SRST_POS);
		sys_write32(0, XEC_DMAC_BASE + XEC_DMA_MAIN_DPKT_OFS);
		sys_set_bit(XEC_DMAC_BASE + XEC_DMA_MAIN_CR_OFS, XEC_DMA_MAIN_CR_EN_POS);
	}

	rc = pinctrl_apply_state(devcfg->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("Pinctrl error (%d)", rc);
		return rc;
	}

	/* configure I2C controller */
	rc = xec_i2c_nl_hw_cfg(dev, devcfg->bitrate, devcfg->port, 0);
	if (rc != 0) {
		LOG_ERR("HW config error (%d)", rc);
		return rc;
	}

	if (devcfg->irq_config != NULL) {
		devcfg->irq_config(dev);
	}

#ifdef CONFIG_I2C_XEC_NL_USE_KWORKQUEUE
	k_work_init(&data->isr_work, &xec_i2c_nl_isr_kworker);
#endif
	return 0;
}

/* I2C-NL interrupt helper Host mode */
static void xec_i2c_nl_hm_handler(const struct device *dev)
{
	/* TODO */
}

#ifdef CONFIG_I2C_TARGET
static void xec_i2c_nl_tm_handler(const struct device *dev)
{
	/* TODO */
}
#endif

/* I2C-NL interrupt handler */
static void xec_i2c_nl_isr(const struct device *dev)
{
	const struct xec_i2c_nl_config *devcfg = dev->config;
	struct xec_i2c_nl_data *data = dev->data;
	mem_addr_t i2c_base = devcfg->i2c_base;

	data->i2c_status = xec_i2c_status_get(i2c_base);

	sys_write32(XEC_I2C_COMP_RW1C_MSK, i2c_base + XEC_I2C_COMP_OFS);
	soc_ecia_girq_status_clear(devcfg->girq, devcfg->girq_pos);

	if (data->i2c_status & BIT(XEC_I2C_SR_BER_POS) | BIT(XEC_I2C_SR_LAB_POS)) {
		soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 0);
		return;
	}

#ifdef CONFIG_I2C_XEC_NL_USE_KWORKQUEUE
	k_work_submit(&data->isr_work);
#else
#ifdef CONFIG_I2C_TARGET
	if ((data->i2c_status & BIT(XEC_I2C_COMP_TDONE_POS)) != 0) {
		xec_i2c_nl_tm_handler(dev);
	} else {
		xec_i2c_nl_hm_handler(dev);
	}
#else
	xec_i2c_nl_hm_handler(dev);
#endif
#endif /* CONFIG_I2C_XEC_NL_USE_KWORKQUEUE */
}

static DEVICE_API(i2c, xec_i2c_nl_driver_api) = {
	.configure = xec_i2c_nl_configure,
	.get_config = xec_i2c_nl_get_config,
	.transfer = xec_i2c_nl_transfer,
	.target_register = xec_i2c_nl_target_register,
	.target_unregister = xec_i2c_nl_target_unregister,
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = xec_i2c_nl_transfer_cb,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit xec_i2c_nl_iodev_submit,
#endif
	.recover_bus = xec_i2c_nl_recover_bus,
};

#define XEC_I2C_GIRQ_DT(inst) MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, 0))
#define XEC_I2C_GIRQ_POS_DT(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, 0))

#define XEC_I2C_CMB_SZ(inst) DT_INST_PROP_OR(inst, cm_buffer_size, 8u)

#define XEC_I2C_NL_DMA_NODE(i, name) DT_INST_DMAS_CTLR_BY_NAME(i, name)
#define XEC_I2C_NL_DMA_DEVICE(i, name) DEVICE_DT_GET(XEC_I2C_NL_DMA_NODE(i, name))
#define XEC_I2C_NL_DMA_CHAN(i, name) DT_INST_DMAS_CELL_BY_NAME(i, name, channel)
#define XEC_I2C_NL_DMA_TRIGSRC(i, name) DT_INST_DMAS_CELL_BY_NAME(i, name, trigsrc)

#ifdef CONFIG_I2C_TARGET
#define XEC_I2C_TMRB_SZ(i) DT_INST_PROP_OR(i, tm_rx_buffer_size, 8u)

#define XEC_I2C_TMRB_DEF(i) \
	static volatile uint8_t xec_i2c_nl##i##_tm_rx_buf[XEC_I2C_TMRB_SZ(i)] __aligned(4)

#define XEC_I2C_NL_TM_DMA_INFO(i, name) \
	.tm_dma_dev = XEC_I2C_NL_DMA_DEVICE(i, name), \
	.tm_rx_buf = xec_i2c_nl##i##_tm_rx_buf, \
	.tm_rx_buf_size = XEC_I2C_TMRB_SZ(i), \
	.tm_dma_chan = XEC_I2C_NL_DMA_CHAN(i, name), \
	.tm_dma_trigsrc = XEC_I2C_NL_DMA_TRIGSRC(i, name),

#else
#define XEC_I2C_TMRB_SZ(i)
#define XEC_I2C_TMRB_DEF(i)
#define XEC_I2C_NL_TM_DMA_INFO(i, name)
#endif

#define XEC_I2C_NL_DEVICE(inst)									\
	struct xec_i2c_nl_data xec_i2c_nl##inst##_data;						\
	static volatile uint8_t xec_i2c_nl##inst##cm_buf[XEC_I2C_CMB_SZ(inst)] __aligned(4);	\
	XEC_I2C_TMRB_DEF(inst);									\
	PINCTRL_DT_INST_DEFINE(inst);								\
	static void xec_i2c_nl##inst##_irq_config(const struct device *dev) {			\
		const struct xec_i2c_nl_config *devcfg = dev->config;				\
		IRQ_CONNECT(DT_INST_IRQN(inst),							\
			    DT_INST_IRQ(inst, priority),					\
			    xec_i2c_nl_isr,							\
			    DEVICE_DT_INST_GET(inst), 0);					\
		irq_enable(DT_INST_IRQN(inst));							\
		soc_ecia_girq_ctrl(devcfg->girq, devcfg->girq_pos, 1u);				\
	}											\
	static const struct xec_i2c_nl_config xec_i2c_nl##inst##_cfg = {			\
		.i2c_base = (mem_addr_t)DT_INST_REG_ADDR(inst),					\
		.bitrate = DT_INST_PROP_OR(inst, clock_frequency, I2C_BITRATE_STANDARD),	\
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),				\
		.irq_config = xec_i2c_nl##inst##_irq_config,					\
		.cm_dma_dev = XEC_I2C_NL_DMA_DEVICE(inst, cm),					\
		.cm_buf = xec_i2c_nl##inst##cm_buf,						\
		.cm_buf_size = XEC_I2C_CMB_SZ(inst),						\
		.cm_dma_chan = XEC_I2C_NL_DMA_CHAN(inst, cm),					\
		.cm_dma_trigsrc = XEC_I2C_NL_DMA_TRIGSRC(inst, cm),				\
		.girq = XEC_I2C_GIRQ_DT(inst),							\
		.girq_pos = XEC_I2C_GIRQ_POS_DT(inst),						\
		.pcr = DT_INST_PROP(inst, pcr),							\
		.port = DT_INST_PROP(inst, port_sel),						\
		XEC_I2C_NL_TM_DMA_INFO(inst, tm)						\
	};											\
	I2C_DEVICE_DT_INST_DEFINE(inst, xec_i2c_nl_init, NULL,					\
		&xec_i2c_nl##inst##_data, &xec_i2c_nl##inst##_cfg,				\
		POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,						\
		&xec_i2c_nl_driver_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_NL_DEVICE)
