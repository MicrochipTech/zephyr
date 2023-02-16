/*
 * Copyright (c) 2023 Microchip Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_i2c_nl

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree/dma.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/interrupt_controller/intc_mchp_xec_ecia.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_mchp_nl, CONFIG_I2C_LOG_LEVEL);

#include <soc.h>

/* #define XEC_I2C_NL_DEBUG */

#define SPEED_100KHZ_BUS	0
#define SPEED_400KHZ_BUS	1
#define SPEED_1MHZ_BUS		2

#define I2C_CTRL_OWN_ADDR_1	0x7EU
#define I2C_CTRL_OWN_ADDR_2	0x7FU
#define I2C_CTRL_OWN_ADDR	((uint32_t)(I2C_CTRL_OWN_ADDR_1) |	\
				 ((uint32_t)(I2C_CTRL_OWN_ADDR_2) << 8))

#define I2C_MODE_CTRL		0
#define I2C_MODE_TARGET		1

/* I2C Read/Write bit pos */
#define I2C_READ_WRITE_POS	0

/* I2C SCL and SDA lines(signals) */
#define I2C_LINES_SCL_POS	0
#define I2C_LINES_SDA_POS	1
#define I2C_LINES_SCL_HI	BIT(I2C_LINES_SCL_POS)
#define I2C_LINES_SDA_HI	BIT(I2C_LINES_SDA_POS)
#define I2C_LINES_BOTH_HI	(I2C_LINES_SCL_HI | I2C_LINES_SDA_HI)

/* Network layer TX buffer size */
#define I2C_NWL_TX_MSG_MAX	64U
#define I2C_NWL_TX_BUF_SIZE	(I2C_NWL_TX_MSG_MAX + 4U)

#define I2C_XEC_CFG_WAIT_MS	35U
#define RESET_WAIT_US		20U
#define I2C_ENABLE_WAIT_US	160U
#define WAIT_MPROCEED_US	2U
#define WAIT_MPROCEED_LOOPS	100U

/* I2C recover SCL low retries */
#define I2C_RECOVER_SCL_LOW_RETRIES	10
/* I2C recover SDA low retries */
#define I2C_RECOVER_SDA_LOW_RETRIES	3
/* I2C recovery bit bang delay */
#define I2C_RECOVER_BB_DELAY_US		5
/* I2C recovery SCL sample delay */
#define I2C_RECOVER_SCL_DELAY_US	50

/* driver states */
#define I2C_XEC_STATE_CLOSED		0
#define I2C_XEC_STATE_OPEN		1

/* driver flags */
#define I2C_XEC_EVENT_NONE		0
#define I2C_XEC_EVENT_START		1
#define I2C_XEC_EVENT_RPT_START		2
#define I2C_XEC_EVENT_STOP		3
#define I2C_XEC_EVENT_DATA_XFR_DONE	4

#define I2C_XEC_DIR_NONE 0
#define I2C_XEC_DIR_WR	 1
#define I2C_XEC_DIR_RD	 2

#define I2C_XEC_ACTION_NONE 0
#define I2C_XEC_ACTION_START 1
#define I2C_XEC_ACTION_RPT_START 2

/* I2C control WO */
#define XEC_I2C_CTR_ACK_POS 0
#define XEC_I2C_CTR_STO_POS 1
#define XEC_I2C_CTR_STA_POS 2
#define XEC_I2C_CTR_ENI_POS 3
#define XEC_I2C_CTR_ESO_POS 6
#define XEC_I2C_CTR_PIN_POS 7
/* I2C status RO */
#define XEC_I2C_STS_NBB_POS	0
#define XEC_I2C_STS_LAB_POS	1
#define XEC_I2C_STS_AAT_POS	2
#define XEC_I2C_STS_LRB_AD0_POS	3
#define XEC_I2C_STS_BER_POS	4
#define XEC_I2C_STS_EXT_STO_POS	5
#define XEC_I2C_STS_SAD_POS	6
#define XEC_I2C_STS_PIN_POS	7
/* own address */
#define XEC_I2C_OWN_ADDR_MSK	0x7f7fu
#define XEC_I2C_OWN_ADDR0_POS	0
#define XEC_I2C_OWN_ADDR1_POS	8
/* Controller Mode Command */
#define XEC_I2C_CM_CMD_MSK	0xffff3f03u
#define XEC_I2C_CM_MRUN_POS	0
#define XEC_I2C_CM_MPROCEED_POS	1
#define XEC_I2C_CM_START0_POS	8
#define XEC_I2C_CM_STARTN_POS	9
#define XEC_I2C_CM_STOP_POS	10
#define XEC_I2C_CM_PEC_TERM_POS	11
#define XEC_I2C_CM_READM_POS	12
#define XEC_I2C_CM_READ_PEC_POS	13
#define XEC_I2C_CM_WCNT_POS	16
#define XEC_I2C_CM_RCNT_POS	24
#define XEC_I2C_CM_WCNT_MSK	0xff0000u
#define XEC_I2C_CM_RCNT_MSK	0xff000000u
#define XEC_I2C_CM_WRCNT_MSK0	0xffu
/* Extended length */
#define XEC_I2C_EXTLEN_MSK	0xffffu
#define XEC_I2C_EXT_WCNT_POS	0
#define XEC_I2C_EXT_RCNT_POS	8
#define XEC_I2C_EXT_WCNT_MSK	0xffu
#define XEC_I2C_EXT_RCNT_MSK	0xff00u
/* Completion */
#define XEC_I2C_COMPL_MSK		0xe33b7f7cu
#define XEC_I2C_COMPL_STS_RW1C_MSK	0xe1397f00u
#define XEC_I2C_COMPL_STS_RO_MSK	0x02020040u
#define XEC_I2C_COMPL_DTEN_POS		2
#define XEC_I2C_COMPL_MCEN_POS		3
#define XEC_I2C_COMPL_SCEN_POS		4
#define XEC_I2C_COMPL_BIDEN_POS		5
#define XEC_I2C_COMPL_TMOUT_STS_POS	6
#define XEC_I2C_COMPL_DTO_STS_POS	8
#define XEC_I2C_COMPL_MCTO_STS_POS	9
#define XEC_I2C_COMPL_SCTO_STS_POS	10
#define XEC_I2C_COMPL_CHDL_STS_POS	11
#define XEC_I2C_COMPL_CHDH_STS_POS	12
#define XEC_I2C_COMPL_BER_STS_POS	13
#define XEC_I2C_COMPL_LAB_STS_POS	14
#define XEC_I2C_COMPL_TMR_NAK_STS_POS	16
#define XEC_I2C_COMPL_TM_RX_RO_POS	17
#define XEC_I2C_COMPL_SPROT_STS_POS	19
#define XEC_I2C_COMPL_RPT_RD_STS_POS	20
#define XEC_I2C_COMPL_RPT_WR_STS_POS	21
#define XEC_I2C_COMPL_CMT_NAK_STS_POS	24
#define XEC_I2C_COMPL_CM_TX_RO_POS	25
#define XEC_I2C_COMPL_IDLE_STS_POS	29
#define XEC_I2C_COMPL_CMD_STS_POS	30
#define XEC_I2C_COMPL_TMD_STS_POS	31
#define XEC_I2C_COMPL_CM_ERR		(BIT(XEC_I2C_COMPL_BER_STS_POS) |\
					 BIT(XEC_I2C_COMPL_LAB_STS_POS) |\
					 BIT(XEC_I2C_COMPL_CMT_NAK_STS_POS))
/* Configuration */
#define XEC_I2C_CFG_MSK			0xf00fdfbfu
#define XEC_I2C_CFG_PORT_SEL_POS	0
#define XEC_I2C_CFG_PORT_SEL_MSK	0xfu
#define XEC_I2C_CFG_TCEN_POS		4
#define XEC_I2C_CFG_SLOW_CLK_POS	5
#define XEC_I2C_CFG_PCEN_POS		7
#define XEC_I2C_CFG_FEN_POS		8
#define XEC_I2C_CFG_SRST_POS		9
#define XEC_I2C_CFG_EN_POS		10
#define XEC_I2C_CFG_DSA_EN_POS		11
#define XEC_I2C_CFG_FAIR_EN_POS		12
#define XEC_I2C_CFG_TM_GC_EN_POS	14
#define XEC_I2C_CFG_PROM_EN_POS		15
/* bits 16:19 write-only */
#define XEC_I2C_CFG_FLUSH_TM_TXB_POS	16
#define XEC_I2C_CFG_FLUSH_TM_RXB_POS	17
#define XEC_I2C_CFG_FLUSH_CM_TXB_POS	18
#define XEC_I2C_CFG_FLUSH_CM_RXB_POS	19
#define XEC_I2C_CFG_FLUSH_ALL		0xf0000u
#define XEC_I2C_CFG_AAS_IEN_POS		28
#define XEC_I2C_CFG_IDLE_IEN_POS	29
#define XEC_I2C_CFG_CMD_IEN_POS		30
#define XEC_I2C_CFG_TMD_IEN_POS		31
/* bit bang control */
#define XEC_I2C_BBCTR_MSK		0x7fu
#define XEC_I2C_BBCTR_EN_POS		0
#define XEC_I2C_BBCTR_SCL_OUT_EN_POS	1
#define XEC_I2C_BBCTR_SDA_OUT_EN_POS	2
#define XEC_I2C_BBCTR_SCL_TRI_POS	3
#define XEC_I2C_BBCTR_SDA_TRI_POS	4
#define XEC_I2C_BBCTR_SCL_PIN_RO_POS	5
#define XEC_I2C_BBCTR_SDA_PIN_RO_POS	6

/* Bit-bang mode: enable, SCL and SDA pins are inputs */
#define XEC_BB_SCL_SDA_TRI_IN (BIT(XEC_I2C_BBCTR_EN_POS) | BIT(XEC_I2C_BBCTR_SCL_TRI_POS) \
			       | BIT(XEC_I2C_BBCTR_SDA_TRI_POS))

/* Bit-bang mode: enable, SCL output mode and tri-state(not driven), SDA input */
#define XEC_BB_SCL_OUT_HI_SDA_TRI \
	(BIT(XEC_I2C_BBCTR_EN_POS) | BIT(XEC_I2C_BBCTR_SCL_OUT_EN_POS)	\
	| BIT(XEC_I2C_BBCTR_SCL_TRI_POS) | BIT(XEC_I2C_BBCTR_SDA_TRI_POS))

/* Bit-bang mode: enable, SCL output mode driven low, SDA is input */
#define XEC_BB_SCL_OUT_DRV_LO_SDA_TRI \
	(BIT(XEC_I2C_BBCTR_EN_POS) | BIT(XEC_I2C_BBCTR_SCL_OUT_EN_POS))

/* Bit-bang mode: enable, SCL output mode driven low, SDA is input */
#define XEC_BB_SCL_TRI_SDA_OUT_DRV_LO \
	(BIT(XEC_I2C_BBCTR_EN_POS) | BIT(XEC_I2C_BBCTR_SDA_OUT_EN_POS))

#define XEC_BB_SCL_IN_HI_SDA_IN_HI \
	(BIT(XEC_I2C_BBCTR_SCL_PIN_RO_POS) | BIT(XEC_I2C_BBCTR_SDA_PIN_RO_POS))

/* wake status and wake enable registers */
#define XEC_I2C_WK_EN_STS_MSK		0x1u
#define XEC_I2C_WK_EN_STS_STA_DET_POS	0

#define XEC_I2C_NL_MAX_LEN 0xfff8u

struct xec_i2c_nl_regs {
	volatile uint32_t ctr_sts; /* ctr wo, sts ro */
	volatile uint32_t own_addr;
	volatile uint8_t i2c_data;
	uint8_t rsvd_09_0b[3];
	volatile uint32_t cm_cmd; /* NL controller mode cmd */
	volatile uint32_t tm_cmd; /* NL target mode cmd */
	volatile uint8_t pec;
	uint8_t rsvd_15_17[3];
	volatile uint32_t rsht;
	volatile uint32_t extlen;
	volatile uint32_t compl;
	volatile uint32_t idle_scaling;
	volatile uint32_t config;
	volatile uint32_t bus_clk;
	volatile uint32_t blk_id;
	volatile uint32_t blk_rev;
	volatile uint32_t bb_ctr;
	volatile uint32_t mchp_rsvd3c;
	volatile uint32_t data_timing;
	volatile uint32_t tmout_scaling;
	volatile uint8_t tm_txb;
	uint8_t rsvd_49_4b[3];
	volatile uint8_t tm_rxb;
	uint8_t rsvd_4d_4f[3];
	volatile uint8_t cm_txb;
	uint8_t rsvd_51_53[3];
	volatile uint8_t cm_rxb;
	uint8_t rsvd_55_57[3];
	volatile uint32_t fsm;
	volatile uint32_t fsm_nl;
	volatile uint32_t wake_sts;
	volatile uint32_t wake_en;
	uint8_t rsvd_68_6b[4];
	volatile uint32_t recv_tm_addr;
	volatile uint32_t prom_sts;
	volatile uint32_t prom_ien;
	volatile uint32_t prom_ctr;
	volatile uint32_t shad_data;
};

struct xec_nl_speed_cfg {
	uint32_t bus_clk;
	uint32_t data_timing;
	uint32_t start_hold_time;
	uint32_t idle_scale;
	uint32_t timeout_scale;
};

struct i2c_xec_nl_config {
	struct xec_i2c_nl_regs * const regs;
	uint8_t port_sel;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t pcr_idx;
	uint8_t pcr_bitpos;
	uint8_t ctr_dma_chan;
	uint8_t ctr_dma_id;
	uint8_t dev_dma_chan;
	uint8_t dev_dma_id;
	const struct device *dma_dev;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
	const struct gpio_dt_spec scl_gpio;
	const struct gpio_dt_spec sda_gpio;
};

#define XEC_I2C_NL_MODE_SYNC 0
#define XEC_I2C_NL_MODE_ASYNC 1

#if 0
/* i2c_ll_stm32.c  pm_device_busy_set(dev)
 * If CONFIG_PM_DEVICE_RUNTIME then pm_device_runtime_get(dev)
 */
enum i2c_xec_pm_policy_flag {
I2C_XEC_PM_POLICY_STATE_
};
#endif

struct i2c_xec_nl_data {
	struct k_mutex ctr_mutex;
	struct k_sem ctr_sem;
	uint8_t i2c_addr;
	uint8_t speed_id;
	uint8_t state;
	uint8_t mode;
	uint8_t prev_dir;
	uint8_t curr_dir;
	uint8_t cm_active;
	uint8_t rsvd1;
	volatile int dma_err;
	volatile uint32_t i2c_compl;
	volatile uint8_t i2c_status;
	volatile uint8_t dma_done;
	volatile uint8_t nl_done;
	volatile uint8_t rsvd2;
#ifdef CONFIG_I2C_CALLBACK
	struct i2c_msg *msgs;
	uint8_t num_msgs;
	uint8_t msg_idx;
	uint16_t addr;
	i2c_callback_t cb;
	void *user_data;
#endif
#ifdef CONFIG_I2C_TARGET
	struct i2c_target_config *target1_cfg;
	bool target1_attached;
	bool target1_read;
	struct i2c_target_config *target2_cfg;
	bool target2_attached;
	bool target2_read;
#endif
#ifdef XEC_I2C_NL_DEBUG
	uint32_t cm_cmd;
	uint32_t dma_mstart;
	uint32_t dma_len;
	uint32_t dma_dev_addr;
	volatile uint32_t isr_cnt;
	volatile uint32_t cm_cmd_isr;
	volatile uint32_t i2c_fsm;
	volatile uint32_t smb_fsm;
	uint8_t wait_state;
#endif
};

/* Recommended programming values based on 16MHz BAUD clock.
 * i2c_baud_clk_period/bus_clk_period - 2 = (low_period + hi_period)
 * bus_clk_reg (16MHz/100KHz -2) = 0x4F + 0x4F
 *             (16MHz/400KHz -2) = 0x0F + 0x17
 *             (16MHz/1MHz -2) = 0x05 + 0x09
 */
static const struct xec_nl_speed_cfg xec_cfg_params[] = {
	[SPEED_100KHZ_BUS] = {
		.bus_clk            = 0x00004F4F,
		.data_timing        = 0x0C4D5006,
		.start_hold_time    = 0x0000004D,
		.idle_scale         = 0x01FC01ED,
		.timeout_scale      = 0x4B9CC2C7,
	},
	[SPEED_400KHZ_BUS] = {
		.bus_clk            = 0x00000F17,
		.data_timing        = 0x040A0A06,
		.start_hold_time    = 0x0000000A,
		.idle_scale         = 0x01000050,
		.timeout_scale      = 0x159CC2C7,
	},
	[SPEED_1MHZ_BUS] = {
		.bus_clk            = 0x00000509,
		.data_timing        = 0x04060601,
		.start_hold_time    = 0x00000006,
		.idle_scale         = 0x10000050,
		.timeout_scale      = 0x089CC2C7,
	},
};

/* return state of I2C SCL and SDA lines by reading GPIO input. */
static int get_lines(const struct device *dev, uint8_t *lines)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	uint8_t  temp = 0;

	if (!lines) {
		return -EINVAL;
	}

	int ret = gpio_pin_get_raw(cfg->scl_gpio.port, cfg->scl_gpio.pin);

	if (ret < 0) {
		return ret;
	}

	if (ret > 0) {
		temp |= I2C_LINES_SCL_HI;
	}

	ret = gpio_pin_get_raw(cfg->sda_gpio.port, cfg->sda_gpio.pin);
	if (ret < 0) {
		return ret;
	}

	if (ret > 0) {
		temp |= I2C_LINES_SDA_HI;
	}

	*lines = temp;

	LOG_DBG("lines=0x%02x", temp);

	return 0;
}

static void ctrl_reset(const struct device *dev)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;

	LOG_DBG("controller reset");

	/* assert reset for >= 1 BAUD clock (16 MHz) */
	regs->config |= BIT(XEC_I2C_CFG_SRST_POS);
	regs->blk_id = 0U; /* AHB clock is 48HMz. */
	regs->blk_id = 0U; /* one AHB access is minimum 3 clocks */
	regs->blk_id = 0U;
	regs->blk_id = 0U;
	regs->config &= ~BIT(XEC_I2C_CFG_SRST_POS);
	regs->config |= XEC_I2C_CFG_FLUSH_ALL;
}

static void ctrl_set_port(struct xec_i2c_nl_regs *regs, uint8_t port)
{
	uint32_t temp = ((uint32_t)port << XEC_I2C_CFG_PORT_SEL_POS) & XEC_I2C_CFG_PORT_SEL_MSK;

	regs->config = (regs->config & ~(XEC_I2C_CFG_PORT_SEL_MSK)) | temp;
}

#ifdef CONFIG_I2C_TARGET
static void i2c_xec_nl_cfg_target_addr(const struct device *dev)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct i2c_xec_nl_data * const data = dev->data;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	uint32_t own_addr = regs->own_addr;
	uint32_t msk = 0, val = 0;

	if (data->target1_cfg) {
		msk = 0x7fu;
		val = data->target1_cfg->address & 0x7fu;
	}
	if (data->target2_cfg) {
		msk |= 0x7f00u;
		val = (data->target2_cfg->address & 0x7fu) << 8;
	}

	own_addr &= ~msk;
	own_addr |= val;
	regs->own_addr = own_addr;
}
#endif

/* Port selection, bus clock, and filter should be configured before controller enabled */
static void ctrl_config1(const struct device *dev)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct i2c_xec_nl_data * const data = dev->data;
	struct xec_i2c_nl_regs * const regs = cfg->regs;

	ctrl_set_port(regs, cfg->port_sel);

	if (data->speed_id > ARRAY_SIZE(xec_cfg_params)) {
		data->speed_id = 0U;
	}

	const struct xec_nl_speed_cfg *spdcfg = &xec_cfg_params[data->speed_id];

	regs->ctr_sts = BIT(XEC_I2C_CTR_PIN_POS);
	regs->own_addr = I2C_CTRL_OWN_ADDR;
#ifdef CONFIG_I2C_TARGET
	i2c_xec_nl_cfg_target_addr(dev);
#endif

	regs->bus_clk = spdcfg->bus_clk;
	/* filter enable */
	regs->config |= BIT(XEC_I2C_CFG_FEN_POS);

	regs->ctr_sts = BIT(XEC_I2C_CTR_PIN_POS) | BIT(XEC_I2C_CTR_ESO_POS)
		| BIT(XEC_I2C_CTR_ACK_POS);

	regs->data_timing = spdcfg->data_timing;
	regs->rsht = spdcfg->start_hold_time;
	regs->idle_scaling = spdcfg->idle_scale;
	regs->tmout_scaling = spdcfg->timeout_scale;

	/* clear sticky status */
	regs->compl = XEC_I2C_COMPL_STS_RW1C_MSK;

	/* enable controller */
	regs->config |= BIT(XEC_I2C_CFG_EN_POS);
}

/* Delay after controller is configured and enabled.
 * Controller must sample I2C lines to determine state of the bus and
 * presence of external Controllers currently using the bus.
 */
static int ctrl_config(const struct device *dev)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct i2c_xec_nl_data * const data = dev->data;
	struct xec_i2c_nl_regs * const regs = cfg->regs;

	data->state = I2C_XEC_STATE_CLOSED;

	ctrl_reset(dev);
	ctrl_config1(dev);

	/* wait SMBus timeout max */
	k_msleep(I2C_XEC_CFG_WAIT_MS);

	data->i2c_status = regs->ctr_sts;
	data->cm_active = 0;

	if (data->i2c_status & BIT(XEC_I2C_STS_BER_POS)) {
		LOG_ERR("I2C XEC NL: Bus Error after config");
		return -EIO;
	}

	return 0;
}

/* Zephyr I2C configure API implementation */
static int i2c_xec_nl_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_xec_nl_data * const data = dev->data;
	uint32_t temp = I2C_SPEED_GET(dev_config);
	int ret = 0;
	uint8_t speed_id = 0;

	if (!(dev_config & I2C_MODE_CONTROLLER)) {
		return -EINVAL;
	}

	switch (temp) {
	case I2C_SPEED_STANDARD:
		speed_id = 0U;
		break;
	case I2C_SPEED_FAST:
		speed_id = 1U;
		break;
	case I2C_SPEED_FAST_PLUS:
		speed_id = 2U;
		break;
	default:
		return -EINVAL;
	}

	k_mutex_lock(&data->ctr_mutex, K_FOREVER);
	data->speed_id = speed_id;
	ret = ctrl_config(dev);
	k_mutex_unlock(&data->ctr_mutex);

	return ret;
}

/* Zephyr I2C get configuration API implementation */
static int i2c_xec_nl_get_config(const struct device *dev, uint32_t *dev_config)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	uint32_t temp = 0;

	if (!dev || !dev_config) {
		return -EINVAL;
	}

	temp = regs->bus_clk;
	for (size_t n = 0; n < ARRAY_SIZE(xec_cfg_params); n++) {
		if (temp == xec_cfg_params[n].bus_clk) {
			*dev_config = n + I2C_SPEED_STANDARD;
			break;
		}
	}

	*dev_config |= I2C_MODE_CONTROLLER;

	return 0;
}

static int xec_nl_recover_bus(const struct device *dev)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	int i, j, ret;

	LOG_ERR("I2C attempt bus recovery");

	/* reset controller to a known state */
	regs->config = BIT(XEC_I2C_CFG_SRST_POS);
	k_busy_wait(RESET_WAIT_US);

	regs->config = BIT(XEC_I2C_CFG_FEN_POS) | (cfg->port_sel & XEC_I2C_CFG_PORT_SEL_MSK);
	regs->config |= XEC_I2C_CFG_FLUSH_ALL;
	regs->ctr_sts = BIT(XEC_I2C_CTR_PIN_POS);

	/* Enable bit-bang mode: SCL and SDA tri-stated inputs */
	regs->bb_ctr = XEC_BB_SCL_SDA_TRI_IN;

	/* SCL is low: read N times and hope external device releases it */
	if (!(regs->bb_ctr & BIT(XEC_I2C_BBCTR_SCL_PIN_RO_POS))) {
		for (i = 0;; i++) {
			if (i >= I2C_RECOVER_SCL_LOW_RETRIES) {
				ret = -EBUSY;
				goto recov_exit;
			}
			k_busy_wait(I2C_RECOVER_SCL_DELAY_US);
			if (regs->bb_ctr & BIT(XEC_I2C_BBCTR_SCL_PIN_RO_POS)) {
				break; /* SCL went High */
			}
		}
	}

	if (regs->bb_ctr & BIT(XEC_I2C_BBCTR_SDA_PIN_RO_POS)) {
		ret = 0;
		goto recov_exit;
	}

	ret = -EBUSY;
	/* SDA recovery */
	for (i = 0; i < I2C_RECOVER_SDA_LOW_RETRIES; i++) {
		/* SCL output mode and tri-stated */
		regs->bb_ctr = XEC_BB_SCL_OUT_HI_SDA_TRI;
		k_busy_wait(I2C_RECOVER_BB_DELAY_US);

		for (j = 0; j < 9; j++) {
			if (regs->bb_ctr & BIT(XEC_I2C_BBCTR_SDA_PIN_RO_POS)) {
				break;
			}
			/* drive SCL low */
			regs->bb_ctr = XEC_BB_SCL_OUT_DRV_LO_SDA_TRI;
			k_busy_wait(I2C_RECOVER_BB_DELAY_US);
			/* release SCL: pulled high by external pull-up */
			regs->bb_ctr = XEC_BB_SCL_OUT_HI_SDA_TRI;
			k_busy_wait(I2C_RECOVER_BB_DELAY_US);
		}

		/* SCL is High. Produce rising edge on SDA for STOP */
		regs->bb_ctr = XEC_BB_SCL_TRI_SDA_OUT_DRV_LO;
		k_busy_wait(I2C_RECOVER_BB_DELAY_US);
		regs->bb_ctr = XEC_BB_SCL_SDA_TRI_IN;
		k_busy_wait(I2C_RECOVER_BB_DELAY_US);

		/* check if SCL and SDA are both high */
		if ((regs->bb_ctr & XEC_BB_SCL_IN_HI_SDA_IN_HI) == XEC_BB_SCL_IN_HI_SDA_IN_HI) {
			ret = 0; /* successful recovery */
			goto recov_exit;
		}
	}

recov_exit: /* BB mode disable reconnects SCL and SDA to I2C logic. */
	regs->bb_ctr = 0;
	ctrl_config(dev); /* reset and reconfigure controller */

	return ret;
}

/* Zephyr I2C bus recovery API implementation */
static int i2c_xec_nl_recover_bus(const struct device *dev)
{
	struct i2c_xec_nl_data * const data = dev->data;
	int ret;

	k_mutex_lock(&data->ctr_mutex, K_FOREVER);
	data->state = 0;
#ifdef XEC_I2C_NL_DEBUG
	data->wait_state = 0;
#endif
	ret = xec_nl_recover_bus(dev);
	k_mutex_unlock(&data->ctr_mutex);

	return ret;
}

static int i2c_nl_check_bus(const struct device *dev)
{
	struct i2c_xec_nl_data * const data = dev->data;
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	int ret = 0;
	uint8_t lines = 0;

	k_busy_wait(35);
	if (get_lines(dev, &lines)) {
		return -EIO;
	}

	if (lines != I2C_LINES_BOTH_HI) {
		if (i2c_xec_nl_recover_bus(dev)) {
			return -EIO;
		}

		data->i2c_status = regs->ctr_sts;
		if (data->i2c_status & (BIT(XEC_I2C_STS_BER_POS) | BIT(XEC_I2C_STS_LAB_POS))) {
			ret = ctrl_config(dev);
			if (ret) {
				return ret;
			}
		}
	}

#ifdef XEC_I2C_NL_DEBUG
	data->wait_state = 0;
#endif
	return 0;
}

static void i2c_xec_nl_dma_done_cb(const struct device *dev, void *arg,
				   uint32_t id, int error_code)
{
	struct i2c_xec_nl_data * const data = ((const struct device *)arg)->data;

	ARG_UNUSED(dev);

	data->dma_err = error_code;
	data->dma_done = 1u;
}

#ifdef XEC_I2C_NL_DEBUG
static void i2c_nl_dbg_dma_cfg(const struct device *dev,
			       uint32_t src, uint32_t dest, uint32_t nbytes,
			       enum dma_channel_direction dir)
{
	struct i2c_xec_nl_data * const data = dev->data;

	data->dma_len = nbytes;
	if (dir == MEMORY_TO_PERIPHERAL) {
		data->dma_mstart = src;
		data->dma_dev_addr = dest;
	} else {
		data->dma_mstart = dest;
		data->dma_dev_addr = src;
	}
}
#endif

static int i2c_nl_dma_cfg(const struct device *dev, uint32_t src, uint32_t dest,
			  uint32_t nbytes, enum dma_channel_direction dir, dma_callback_t cb)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct dma_config dma_cfg = { 0 };
	struct dma_block_config dma_block_cfg = { 0 };
	int ret = 0;

#ifdef XEC_I2C_NL_DEBUG
	i2c_nl_dbg_dma_cfg(dev, src, dest, nbytes, dir);
#endif
	dma_cfg.channel_direction = dir;

	if (dir == MEMORY_TO_PERIPHERAL) {
		dma_block_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_block_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		dma_block_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_block_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	dma_block_cfg.source_address = src;
	dma_block_cfg.dest_address = dest;

	dma_cfg.dma_slot = cfg->ctr_dma_id;
	dma_cfg.source_data_size = 1U;
	dma_cfg.dest_data_size = 1U;
	dma_cfg.dma_callback = cb;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.complete_callback_en = 1U;
	dma_cfg.error_callback_en = 1U;
	dma_cfg.block_count = 1U;
	dma_cfg.head_block = &dma_block_cfg;

	dma_block_cfg.block_size = nbytes;

	ret = dma_config(cfg->dma_dev, cfg->ctr_dma_chan, &dma_cfg);
	if (ret) {
		LOG_ERR("XEC I2C DMA config error: (%d)", ret);
		return ret;
	}

	return 0;
}

static int do_dma_stop(const struct device *dev, uint8_t target_mode)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	int ret = 0;
	uint32_t chan = cfg->ctr_dma_chan;

	if (target_mode) {
		chan = cfg->dev_dma_chan;
	}

	ret = dma_stop(cfg->dma_dev, chan);

	return ret;
}

static void clr_isr_flags(struct i2c_xec_nl_data *data)
{
	data->dma_err = 0;
	data->i2c_compl;
	data->nl_done = 0;
	data->dma_done = 0;
#ifdef XEC_I2C_NL_DEBUG
	data->cm_cmd_isr = 0;
	data->i2c_fsm;
	data->smb_fsm;
#endif
}

static int i2c_xec_nl_msg_tx(const struct device *dev, struct i2c_msg *m, uint16_t addr, int async)
{
	struct i2c_xec_nl_data * const data = dev->data;
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	uint32_t cm_cmd, txlen, dmalen;
	uint8_t *ptxb = NULL;
	bool req_start = false;
	uint8_t fmt_addr = (addr & 0x7fu) << 1;

	ptxb = m->buf;
	txlen = m->len;
	dmalen = m->len;
	cm_cmd = BIT(XEC_I2C_CM_MRUN_POS) | BIT(XEC_I2C_CM_MPROCEED_POS);
	if (m->flags & I2C_MSG_RESTART) {
		req_start = true;
		cm_cmd |= BIT(XEC_I2C_CM_START0_POS);
		data->i2c_addr = fmt_addr;
		txlen++;
	}

	if (m->flags & I2C_MSG_STOP) {
		cm_cmd |= BIT(XEC_I2C_CM_STOP_POS);
	}

	cm_cmd |= ((txlen & 0xffu) << XEC_I2C_CM_WCNT_POS);
#ifdef XEC_I2C_NL_DEBUG
	data->cm_cmd = cm_cmd;
#endif
	regs->extlen = ((txlen >> 8) & 0xffu);
	regs->compl = XEC_I2C_COMPL_STS_RW1C_MSK;

	clr_isr_flags(data);
	i2c_nl_dma_cfg(dev, (uint32_t)ptxb, (uint32_t)&regs->cm_txb,
		       dmalen, MEMORY_TO_PERIPHERAL, i2c_xec_nl_dma_done_cb);

#ifdef XEC_I2C_NL_DEBUG
	data->wait_state = 0x10;
#endif
	regs->config |= BIT(XEC_I2C_CFG_CMD_IEN_POS);

	if (req_start) { /* !!! atomic seqeunce !!! */
		regs->cm_cmd = cm_cmd;
		regs->cm_txb = fmt_addr;
		dma_start(cfg->dma_dev, cfg->ctr_dma_chan);
	} else { /* !!! atomic sequence !!! */
		dma_start(cfg->dma_dev, cfg->ctr_dma_chan);
		regs->cm_cmd = cm_cmd;
	}

	if (!async) {
		k_sem_take(&data->ctr_sem, K_FOREVER);
#ifdef XEC_I2C_NL_DEBUG
		data->wait_state = 0;
#endif
		if (data->i2c_compl & XEC_I2C_COMPL_CM_ERR) {
			return -EIO;
		}
	}

	return 0;
}

/* I2C read with MCHP network layer is more complex due to HW idiosyncrasies.
 * 1. HW performs read-ahead unless the HW STOP bit is set. Setting HW to read
 *    N bytes without HW STOP results in HW generating clocks for up to N+2 bytes.
 *    Byte N+1 is in NL RX buffer register and byte N+2 is in low level I2C.Data
 *    register. Due to read-ahead I2C-NL will assert the DMA request signal.
 * 2. HW Rpt-START (STARTN) feature only works for transition from transmit
 *    to receive. HW wrCnt reaching 1 and STARTN=1 causes HW to generate a
 *    START before transmitting the last byte. If we want to support read to
 *    write transitions we must issue a STOP then a new START plus wrAddr.
 * 3. HW read count 2 -> 1 results in the HW NACK'ing the last byte from the
 *    target even if the HW STOP flag is not set.
 * 4. Allowing reads to be broken up into multiple I2C messages does not work
 *    with I2C-NL hardware. The I2C-NL HW read count must be set >= RX DMA length + 2.
 *    When RX DMA is finished, I2C-NL will continue reading ahead two more bytes
 *    and storing these bytes in the I2C-NL RX Data register and I2C Data registers.
 *    I2C-NL provides no indication via HW status when it completes the read-ahead.
 *    Hard coded delays will not work because I2C allows Controller and Targets
 *    to clock stretch between each clock pulse. Therefore this driver will only
 *    support one I2C read message with I2C_MSG_STOP set.
 *
 * previous direction == TX We must issue equivalent of Rpt-START.
 *   set current direction = RX
 *   Configure I2C-NL wrCnt=1, STARTN=1, rdCnt=message length, and STOP flag.
 *   CM DMA channel configured for TX one byte. Byte = target read address.
 *
 * previous direction == NONE (closed)
 *   set current direction = RX
 *   Same as Rpt-START except set START0 bit instead of STARTN bit.
 *
 * Instead of using DMA to transmit the one byte target address, we manually
 * write the address. I2C-NL HW requires a specific sequence:
 * 1. Configure RX DMA for message but do not start the DMA channel.
 * 1. Write I2C-NL Controller-Mode command register
 * 2. Write target address to I2C-NL Controller-Mode TX Data register
 * 3. Enable I2C-NL Controller-Mode Done interrupt
 * 4. ISR fires after HW gets ACK/NAK from target.
 * 5. If no error, ISR calls DMA start and sets I2C-NL MPROCEED=1
 * 6. After data read is done, ISR fires again.
 * 7. ISR sets done flag and gives semaphore.
 */
static int i2c_xec_nl_msg_rx(const struct device *dev, struct i2c_msg *m, uint16_t addr, int async)
{
	struct i2c_xec_nl_data * const data = dev->data;
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	uint32_t cm_cmd, rxlen;
	uint8_t fmt_addr = ((addr & 0x7fu) << 1) | BIT(0);

	regs->compl = XEC_I2C_COMPL_STS_RW1C_MSK;
	clr_isr_flags(data);

	rxlen = 0;
	cm_cmd = BIT(XEC_I2C_CM_MRUN_POS) | BIT(XEC_I2C_CM_MPROCEED_POS)
		| BIT(XEC_I2C_CM_STOP_POS);
	data->i2c_addr = fmt_addr;
	cm_cmd |= (1u << XEC_I2C_CM_WCNT_POS);
	if (data->prev_dir == I2C_XEC_DIR_WR) {
		cm_cmd |= BIT(XEC_I2C_CM_STARTN_POS);
	} else {
		cm_cmd |= BIT(XEC_I2C_CM_START0_POS);
	}

	/* Receive data from target and always generate STOP */
	cm_cmd |= ((m->len & 0xffu) << XEC_I2C_CM_RCNT_POS);
#ifdef XEC_I2C_NL_DEBUG
	data->cm_cmd = cm_cmd;
	data->wait_state = 0x20;
#endif
	regs->extlen = (m->len & 0xff00u);
	i2c_nl_dma_cfg(dev, (uint32_t)&regs->cm_rxb, (uint32_t)m->buf,
		       m->len, PERIPHERAL_TO_MEMORY, i2c_xec_nl_dma_done_cb);
	/* can't do dma_start(cfg->dma_dev, cfg->ctr_dma_chan) here because
	 * HW design uses one set of DMA signals for both TX and RX.
	 * HW FSM transmits read address first which can trigger RX DMA most
	 * of the time. When HW finishes transmit of read address it will
	 * decrement HW write count to 0, clear MPROCEED, and generate
	 * Controller-Mode Done interrupt. In the ISR if we observe MRUN==1 and
	 * MPROCEED==0 we arm RX DMA, set MPROCEED back to 1, and return without
	 * giving the semaphore. When HW finishes read and generates STOP, it
	 * will decrement read count to 0, clear STOP, MRUN, MPROCEED, and
	 * generate an I2C Controller-Mode interrupt. ISR will then give semaphore
	 * and k_sem_take will return.
	 */
	/* atomic sequence */
	regs->cm_cmd = cm_cmd;
	regs->cm_txb = fmt_addr; /* triggers transmit of read address to target */
	regs->config |= BIT(XEC_I2C_CFG_CMD_IEN_POS); /* enable I2C interrupt */

	if (!async) { /* wait for ISR to signal all done */
		k_sem_take(&data->ctr_sem, K_FOREVER);
#ifdef XEC_I2C_NL_DEBUG
		data->wait_state++;
#endif
		if (data->i2c_compl & XEC_I2C_COMPL_CM_ERR) {
			return -EIO;
		}
#ifdef XEC_I2C_NL_DEBUG
		data->wait_state++;
#endif
	}

	return 0;
}

static int chk_closed(const struct device *dev)
{
	struct i2c_xec_nl_data * const data = dev->data;
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	int ret;

	if ((data->state == I2C_XEC_STATE_CLOSED) || (regs->ctr_sts & BIT(XEC_I2C_STS_NBB_POS))) {
		data->state = I2C_XEC_STATE_CLOSED;
		data->prev_dir = data->curr_dir = I2C_XEC_DIR_NONE;
#ifdef XEC_I2C_NL_DEBUG
		data->isr_cnt = 0;
#endif
		ret = i2c_nl_check_bus(dev);
		if (ret) {
			k_mutex_unlock(&data->ctr_mutex);
			return ret;
		}
	}

	return 0;
}

/* I2C Network layer HW implementation is limited to:
 * Message length <= 0xffffu bytes, we reduce that to 0xfff8 because we can.
 * There can be only ONE read message. NL HW always NAK's last received byte
 * and it can't be turned off AND the read message MUST have I2C_MSG_STOP set.
 * It was designed for SMBus complete transaction START to STOP.
 * [] indicates target driving SDA.
 * num_msgs == 1
 *   a. START wrAddr [ACK] data0 [ACK] ... dataN-1 [ACK] STOP
 *   b. START rdAddr [ACK] [data0] ACK ... [dataN-1] NAK STOP
 * num_msgs == 2
 *   a. msg0: START wrAddr [ACK] msg0_0 [ACK] ... msg0_N-1 [ACK] controller drives SCL low
 *      msg1: msg1_0 [ACK] ... msg1_N-1 [ACK] STOP
 *   b. msg0: START wrAddr [ACK] msg0_0 [ACK] ... msg0_N-1 [ACK] controller drives SCL low
 *      msg1: RPT-START rdAddr [ACK] [msg1_0] ACK ... [msg1_N-1] NAK STOP
 */
static int i2c_xec_nl_chk_msgs(struct i2c_msg *msgs, uint8_t num_msgs)
{
	struct i2c_msg *curr, *next;

	if (!msgs || !num_msgs || (num_msgs > 2)) {
		return -EINVAL;
	}

	curr = &msgs[0];
	/* indicate first message needs a START. Zephyr i2c header does not
	 * have a flag for START only RESTART.
	 */
	curr->flags |= I2C_MSG_RESTART;

	for (uint8_t i = 1; i <= num_msgs; i++) {
		if (i < num_msgs) {
			next = curr + 1;

			if (curr->len > XEC_I2C_NL_MAX_LEN) {
				LOG_ERR("msg exceeds HW length");
				return -EINVAL;
			}

			uint8_t curr_dir = curr->flags & I2C_MSG_RW_MASK;
			uint8_t next_dir = next->flags & I2C_MSG_RW_MASK;

			if (curr_dir != next_dir) {
				if (!(next->flags & I2C_MSG_RESTART)) {
					LOG_ERR("msg missing restart flag");
					return -EINVAL;
				}
				if ((curr_dir == I2C_MSG_READ) && (next_dir == I2C_MSG_WRITE)) {
					LOG_ERR("HW RX to TX not supported");
					return -EINVAL;
				}
			}

			if (curr->flags & I2C_MSG_STOP) {
				LOG_ERR("Intermediate msg can't have stop");
				return -EINVAL;
			}
		} else {
			curr->flags |= I2C_MSG_STOP;
		}

		curr++;
	}

	return 0;
}

/* Check if requested target address matches one of the target addresses
 * for this controller. Controller-Mode should not address one of its
 * target addresses.
 */
static int i2c_xec_nl_chk_addr(const struct device *dev, uint16_t addr)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	uint32_t own_addr = regs->own_addr;

	for (int i = 0; i < 2; i++) {
		if ((own_addr & 0x7fu) == (addr & 0x7fu)) {
			return -EINVAL;
		}
		own_addr >>= 8;
	}

	return 0;
}

/* Zephyr I2C API i2c_transfer synchronous implemenation */
static int i2c_xec_nl_transfer(const struct device *dev, struct i2c_msg *msgs,
			       uint8_t num_msgs, uint16_t addr)
{
	struct i2c_xec_nl_data * const data = dev->data;
	struct i2c_msg *m = NULL;
	int ret = 0;

	if (!msgs || !num_msgs) {
		return -EINVAL;
	}

	ret = i2c_xec_nl_chk_addr(dev, addr);
	if (ret) {
		return ret;
	}

	ret = i2c_xec_nl_chk_msgs(msgs, num_msgs);
	if (ret) {
		return ret;
	}

	k_mutex_lock(&data->ctr_mutex, K_FOREVER);

	/* PM device runtime: driver indicates it needs the device to be active
	 * PM API will increment the usage count and resume the device if necessary.
	 */
	(void)pm_device_runtime_get(dev);

	data->cm_active = 1;
	data->state = I2C_XEC_STATE_CLOSED;
	data->mode = XEC_I2C_NL_MODE_SYNC;

	ret = chk_closed(dev);
	if (ret) {
		data->cm_active = 0;
		return ret;
	}

	for (uint8_t i = 0; i < num_msgs; i++) {
		m = &msgs[i];

		if (!m->buf || !m->len) {
			continue;
		}

		if (m->flags & I2C_MSG_READ) {
			data->curr_dir = I2C_XEC_DIR_RD;
			ret = i2c_xec_nl_msg_rx(dev, m, addr, 0);
		} else {
			data->curr_dir = I2C_XEC_DIR_WR;
			ret = i2c_xec_nl_msg_tx(dev, m, addr, 0);
		}
		if (ret) {
			/* force DMA to end */
			do_dma_stop(dev, I2C_MODE_CTRL);
			/* error: reset and reconfigure */
			ctrl_config(dev);
			break;
		}
		data->prev_dir = data->curr_dir;
		k_sem_reset(&data->ctr_sem);
	}

	data->state = I2C_XEC_STATE_CLOSED;
	data->cm_active = 0;

	/* PM device runtime: driver indicates it no longer needs the device to be active
	 * PM call decrements the device usage count and suspend the device if necessary.
	 * !!! If we clear I2C.Config.Enable bit, must we wait multiple I2C clocks to
	 * !!! resynchronize I2C Controller with SCL/SDA pin state?
	 */
	(void)pm_device_runtime_put(dev);
	/* or schedule future suspend: pm_device_runtime_put_async() */

	k_mutex_unlock(&data->ctr_mutex);

	return ret;
}

#ifdef CONFIG_I2C_CALLBACK
/* Zephyr I2C API asynchronous transfer with callback */
static int i2c_xec_nl_xfr_cb(const struct device *dev,
			     struct i2c_msg *msgs,
			     uint8_t num_msgs,
			     uint16_t addr,
			     i2c_callback_t cb,
			     void *userdata)
{
	struct i2c_xec_nl_data * const data = dev->data;
	struct i2c_msg *m = NULL;
	int ret = 0;

	if (!msgs || !num_msgs) {
		return -EINVAL;
	}

	ret = i2c_xec_nl_chk_addr(dev, addr);
	if (ret) {
		return ret;
	}

	ret = i2c_xec_nl_chk_msgs(msgs, num_msgs);
	if (ret) {
		return ret;
	}

	ret = k_mutex_lock(&data->ctr_mutex, K_NO_WAIT);
	if (ret) {
		return -EWOULDBLOCK;
	}

	data->cm_active = 1;
	data->mode = XEC_I2C_NL_MODE_ASYNC;

	ret = chk_closed(dev);
	if (ret) {
		data->cm_active = 0;
		return ret;
	}

	data->msgs = msgs;
	data->msg_idx = 0;
	data->num_msgs = num_msgs;
	data->addr = addr;
	data->cb = cb;
	data->user_data = userdata;

	m = &msgs[0];
	if (m->flags & I2C_MSG_READ) {
		data->curr_dir = I2C_XEC_DIR_RD;
		ret = i2c_xec_nl_msg_rx(dev, m, addr, 1);
	} else {
		data->curr_dir = I2C_XEC_DIR_WR;
		ret = i2c_xec_nl_msg_tx(dev, m, addr, 1);
	}

	if (ret) {
		/* force DMA to end */
		do_dma_stop(dev, I2C_MODE_CTRL);
		/* error: reset and reconfigure */
		ctrl_config(dev);
		data->cm_active = 0;
	}

	return ret;
}
#endif /* CONFIG_I2C_CALLBACK */

#ifdef CONFIG_I2C_TARGET
static int i2c_xec_nl_target_register(const struct device *dev, struct i2c_target_config *config)
{
	struct i2c_xec_nl_data * const data = dev->data;
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	int ret = 0, target_num = 0;

	if (!config) {
		return -EINVAL;
	}

	if ((data->target1_attached && data->target2_attached) || (data->cm_active)) {
		return -EBUSY;
	}

	if (!data->target1_attached) {
		data->target1_cfg = config;
		target_num = 1;
		ret = ctrl_config(dev);
		data->target1_attached = true;
	} else {
		data->target2_cfg = config;
		target_num = 2;
		data->target2_attached = true;
	}

	ret = ctrl_config(dev);
	if (ret) {
		return ret;
	}

	/* TODO Configure target DMA channel to read data from I2C-NL
	 * requires buffer byte length = n unless PEC enabled then n+2
	 */

	/* I2C-NL spec say turn this one */
	regs->config |= BIT(XEC_I2C_CFG_TMD_IEN_POS);

	/* turn on addressed as target interrupt */
	/* regs->config |= BIT(XEC_I2C_CFG_AAS_IEN_POS); */

	if (target_num == 1) {
		data->target1_attached = true;
	} else if (target_num == 2) {
		data->target2_attached = true;
	}

	return ret;
}

static int i2c_xec_nl_target_unregister(const struct device *dev, struct i2c_target_config *config)
{
	struct i2c_xec_nl_data * const data = dev->data;
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	uint32_t own_addr = regs->own_addr;

	if (!config) {
		return -EINVAL;
	}

	uint32_t cfg_addr = config->address & 0x7fu;

	if (data->target1_attached && ((own_addr & 0x7fu) == cfg_addr)) {
		data->target1_attached = false;
		data->target1_read =false;
		data->target1_cfg = NULL;
	} else if (data->target2_attached && (((own_addr >> 8) & 0x7fu) == cfg_addr)) {
		data->target2_attached = false;
		data->target2_read =false;
		data->target2_cfg = NULL;
	} else {
		return -EINVAL;
	}

	return 0;
}

/* Zephyr CONFIG_I2C_TARGET callbacks
struct i2c_target_callbacks {
	i2c_target_write_requested_cb_t write_requested;
	i2c_target_read_requested_cb_t read_requested;
	i2c_target_write_received_cb_t write_received;
	i2c_target_read_processed_cb_t read_processed;
	i2c_target_stop_cb_t stop;
};
write_requested - driver signals application a START + matching target address received and addr
b[0]=1(write target)
write_received - driver sends received data byte to application
read_requested - driver signal app a START + matching target received and addr b[0]=1(read target)
		 App fills in byte pointer with byte of data to transmit to external Controller
read_processed - driver calls this to get more data to transmit to external Controller.
stop - Called by driver when it observes I2C STOP after a START.

Due to the above being one byte oriented API's, the driver must implement a target mode buffer
to capture data from the external Controller.
Receive from external Controller.
We get START + wrAddr match and HW transfers up to len(tmbuf) bytes.
When HW transfer done, HW signals TM Done interrupt.
  ? If TM_CMD rdCnt -> 0 will HW NAK last byte ?
  ? if TM_CMD rcCnt does not reach 0, does HW set any status ?
  If external Controller sends < buf size and a STOP we should get status.
  If external Controller sends > buf then our HW will stall due to DMA done filling our rx buffer.
  If external Controller sends < buf size and stalls (clock stretches) then we can do nothing.
*/

/* I2C-NL Target Mode
 * TM DMA channel set for RX direction with buffer supplied by application.
 * Enable target mode done interrupt in I2C.Config register
 * TM_CMD = rdCnt=n, SPROCEED=SRUN=1
 * Wait for interrupt
 * If external Controller wrote to us as target, HW clears SRUN
 * If external Controller wants to read from us as target, HW clears SPROCEED, SRUN stays 1
 *   Configure TM DMA for TX direction with buffer supplied by application
 *   Enable target mode done interrupt in I2C.Config register
 *   TM_CMD = wrCnt=m, SPROCEED=1, SRUN=1
 *   wait for interrupt
 */
static void i2c_xec_nl_target_handler(const struct device *dev)
{
	/* TODO */
}
#endif /* CONFIG_I2C_TARGET */

/* I2C-NL interrupt handler
 * High level interrupt enables in Configuration register.
 *   Target-Mode Done
 *   Controller-Mode Done
 *     The two DONE interrupts mean the HW FSM has either finished the configured
 *     transaction or has paused for FW assistance (turn-around DMA).
 *     NOTE: error cause also set DONE.
 *   Idle - this I2C controller has released the bus. Errors can set IDLE if the
 *          error causes this controller to release the bus (BER, LAB, ...)
 *   AAS - Generated an interrupt in Target-Mode when received address
 *         matches one of the two addresses in the OWN_ADDR register or
 *         if enabled the SMBus General Call, SMBus Host adress, and SMBus
 *         Default address.
 *
 * The Completion register has status bits for many Controller-Mode and Target-Mode
 * events.  It also has status and enables for individual HW timeouts. The global
 * HW timeout is in the Configuration register (driver does not use HW timeouts).
 * Completion Status:
 *   SDONE (R/W1C) Target-Mode done status
 *   MDONE (R/W1C) Controller-Mode done status
 *   IDLE (R/W1C) this controller is in idle state: SCL & SDA tri-stated.
 *   MNACKX (R/W1C) Controller-Mode received from external target
 *   Various bits for Target-Mode events
 *   Various bits for HW timeouts status and enables
 *
 * Controller-Mode Done logic:
 * 1. Write to target with no errors or NAK's from the target.
 *    Expect cm_cmd register to be 0. HW decremented write count field to 0,
 *    cleared MRUN and MPROCEED, cleared other control bits (START0 and STOP).
 *    If STOP was enabled in cm_cmd the controller issued a STOP after all data
 *    was written. HW has moved to IDLE state. The IDLE status will also be set
 *    and I2C.ctr_sts register will have PIN==1 and NBB==1.
 *    If STOP was not enabled, same behaviour as above except IDLE will not
 *    be set and I2C.ctr_sts register will be 0 (PIN==0 and NBB==0). HW will
 *    be strecthing the clock by driving SCL low. SDA pin state will be based
 *    upon its state at the last clock.
 * 2. Write to target with error or NAK from the target.
 *    One or more of the error or NAK status bits will be set. HW will release
 *    the bus resulting is I2C.ctr_sts PIN==1 and NBB=1. I2C.cm_cmd write count
 *    may not be zero depending upon which byte the error/NAK occured on.
 *    The transction is over.
 * 3. RPT-START is a write of the target address. HW FSM supports direction
 *    change write to read, not the other direction. HW FSM generates RPT-START
 *    when STARTN=1 and write count reaches 1. It expects the last byte to be
 *    the target read address (bit[0]==1). Upon observing address bit[0]==1,
 *    HW FSM will read the first data byte, ACK it and store the data byte.
 *    HW FSM always performs read-ahead when direction is changed to read
 *    based on target address bit[0]==1. NOTE: read count can be 0 and HW FSM
 *    will still perform the read-ahead. Read-ahead does not occur if STOP=1
 *    in the I2C.cm_cmd register. Another idiosyncrasy is when read count
 *    becomes 1, HW FSM will always issue a NAK on the last byte read.
 * 4. If driver sets HW read count greater than the actual read size, the
 *    I2C-NL HW will not signal when it is done with read-ahead. This is
 *    a race condition between the driver reprogramming the registers for the
 *    next read chunk and I2C-NL HW performing read-ahead. I2C-NL provides no
 *    status indicating when all I2C clocks are done. Due to this HW limitation
 *    we will only support one read message with stop.
 */
static void i2c_nl_xec_isr(void *arg)
{
	const struct device *dev = arg;
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct i2c_xec_nl_data * const data = dev->data;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	uint32_t cm_cmd, compl, i2c_config;

#ifdef XEC_I2C_NL_DEBUG
	data->isr_cnt++;
	data->i2c_fsm = regs->fsm;
	data->smb_fsm = regs->fsm_nl;
#endif

	regs->config &= ~BIT(XEC_I2C_CFG_CMD_IEN_POS);

	i2c_config = regs->config;
	cm_cmd = regs->cm_cmd;
	compl = regs->compl;
	data->i2c_compl = compl;
	data->i2c_status = regs->ctr_sts;
#ifdef XEC_I2C_NL_DEBUG
	data->cm_cmd_isr = cm_cmd;
#endif
	regs->compl = compl;
	mchp_xec_ecia_girq_src_clr(cfg->girq, cfg->girq_pos);

#ifdef CONFIG_I2C_TARGET
	if ((i2c_config & BIT(XEC_I2C_CFG_AAS_IEN_POS))
	    && (data->i2c_status & BIT(XEC_I2C_STS_AAT_POS))) {
		i2c_xec_nl_target_handler(dev);
		return;
	}
#endif
	if (!(compl & XEC_I2C_COMPL_CM_ERR)) {
		/* Did HW FSM clear MPROCEED for DMA direction change from TX to RX */
		if ((cm_cmd & (BIT(XEC_I2C_CM_MRUN_POS) | BIT(XEC_I2C_CM_MPROCEED_POS)))
		    == BIT(XEC_I2C_CM_MRUN_POS)) {
			/* Arm RX DMA before setting MPROCEED back to 1 */
			dma_start(cfg->dma_dev, cfg->ctr_dma_chan);
			regs->cm_cmd |= BIT(XEC_I2C_CM_MPROCEED_POS) | BIT(XEC_I2C_CM_MRUN_POS);
			regs->config |= BIT(XEC_I2C_CFG_CMD_IEN_POS);
			return;
		}
	}

#ifdef CONFIG_I2C_CALLBACK
	if (data->mode == XEC_I2C_NL_MODE_ASYNC) {
		int result = 0;

		if (compl & XEC_I2C_COMPL_CM_ERR) {
			result = -EIO;
		}

		data->msg_idx++;
		if ((result == 0) && (data->msg_idx < data->num_msgs)) {
			struct i2c_msg *m = &data->msgs[data->msg_idx];

			if (m->flags & I2C_MSG_READ) {
				i2c_xec_nl_msg_rx(dev, m, data->addr, 1);
			} else {
				i2c_xec_nl_msg_tx(dev, m, data->addr, 1);
			}
		} else { /* all message done and controller generated STOP */
			if (data->cb) {
				data->state = I2C_XEC_STATE_CLOSED;
				data->cm_active = 0;
				data->cb(dev, result, data->user_data);
			}
		}
		return;
	}
#endif

	data->nl_done = 1u;
	k_sem_give(&data->ctr_sem); /* increment semaphore count unless its at max  */
}

static const struct i2c_driver_api i2c_xec_nl_api = {
	.configure = i2c_xec_nl_configure,
	.transfer = i2c_xec_nl_transfer,
	.get_config = i2c_xec_nl_get_config,
	.recover_bus = i2c_xec_nl_recover_bus,
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = i2c_xec_nl_xfr_cb,
#endif
#ifdef CONFIG_I2C_TARGET
	.target_register = i2c_xec_nl_target_register,
	.target_unregister = i2c_xec_nl_target_unregister,
#endif
};

#ifdef CONFIG_PM_DEVICE
static int i2c_nl_xec_pm_action(const struct device *dev,
				enum pm_device_action action)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	/*  const struct i2c_nrfx_twim_config *dev_config = dev->config; */
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		/* TODO */
		ret = pinctrl_apply_state(dev_config->pcfg,
					  PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			return ret;
		}

		/* TODO nrfx_twim_enable(&dev_config->twim); */
		break;

	case PM_DEVICE_ACTION_SUSPEND:
		/* TODO nrfx_twim_disable(&dev_config->twim); */
		/* TODO if CONFIG_I2C_TARGET is enabled do not disable pins
		 * Or do we need a DT variable indicating this is a wakeup device?
		 */
		ret = pinctrl_apply_state(dev_config->pcfg,
					  PINCTRL_STATE_SLEEP);
		if (ret < 0) {
			return ret;
		}

		break;

	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

static int i2c_nl_xec_init(const struct device *dev)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct i2c_xec_nl_data * const data = dev->data;
	int ret;

	data->state = I2C_XEC_STATE_CLOSED;

	k_mutex_init(&data->ctr_mutex);
	k_sem_init(&data->ctr_sem, 0, 2); /* initial count, limit */

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("XEC I2C pinctrl setup failed (%d)", ret);
		return ret;
	}

	/* controller mode default configuration */
	data->speed_id = SPEED_100KHZ_BUS;
	ret = ctrl_config(dev);
	if (ret) {
		return ret;
	}

#ifdef CONFIG_PM_DEVICE_RUNTIME
	pm_device_init_suspended(dev);
	pm_device_runtime_enable(dev);
#else
	/* TODO nrfx_twim_enable(&dev_config->twim); */
#endif

	cfg->irq_config_func();
	mchp_xec_ecia_girq_src_en(cfg->girq, cfg->girq_pos);

	if (!device_is_ready(cfg->dma_dev)) {
		return -ENODEV;
	}

	return 0;
}

#define I2C_NL_XEC_DEVICE(n)						\
									\
	static struct i2c_xec_nl_data i2c_xec_nl_data_##n;		\
									\
	PINCTRL_DT_INST_DEFINE(n);					\
									\
	static void i2c_nl_xec_irq_config_func_##n(void)		\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    i2c_nl_xec_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQN(n));				\
	}								\
									\
	static const struct i2c_xec_nl_config i2c_xec_nl_config_##n = {	\
		.regs =	(struct xec_i2c_nl_regs * const)DT_INST_REG_ADDR(n), \
		.port_sel = DT_INST_PROP(n, port_sel),			\
		.girq = DT_INST_PROP_BY_IDX(n, girqs, 0),		\
		.girq_pos = DT_INST_PROP_BY_IDX(n, girqs, 1),		\
		.pcr_idx = DT_INST_PROP_BY_IDX(n, pcrs, 0),		\
		.pcr_bitpos = DT_INST_PROP_BY_IDX(n, pcrs, 1),		\
		.ctr_dma_chan = MCHP_XEC_DT_INST_DMA_CHANNEL(n, ctr),	\
		.ctr_dma_id = MCHP_XEC_DT_INST_DMA_TRIGSRC(n, ctr),	\
		.irq_config_func = i2c_nl_xec_irq_config_func_##n,	\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		.dev_dma_chan = MCHP_XEC_DT_INST_DMA_CHANNEL(n, dev),	\
		.dev_dma_id = MCHP_XEC_DT_INST_DMA_TRIGSRC(n, dev),	\
		.dma_dev = DEVICE_DT_GET(MCHP_XEC_DT_INST_DMA_CTLR(n, ctr)), \
		.scl_gpio = GPIO_DT_SPEC_INST_GET(n, scl_gpios),	\
		.sda_gpio = GPIO_DT_SPEC_INST_GET(n, sda_gpios),	\
	};								\
	PM_DEVICE_DT_INST_DEFINE(n, i2c_nl_xec_pm_action);		\
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_nl_xec_init, NULL,		\
		&i2c_xec_nl_data_##n, &i2c_xec_nl_config_##n,		\
		POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,			\
		&i2c_xec_nl_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_NL_XEC_DEVICE)
