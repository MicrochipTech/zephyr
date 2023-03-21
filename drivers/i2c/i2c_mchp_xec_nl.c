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
#include <zephyr/drivers/clock_control/mchp_xec_clock_control.h>
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


#define XEC_I2C_NL_DEBUG
#define XEC_I2C_NL_DEBUG_POLL_EVENTS
/* #define XEC_I2C_NL_DEBUG_CTR_SEM */
#define XEC_I2C_NL_ALLOW_LEAVE_OPEN

#define XEC_I2C_CONTEXT
#define XEC_I2C_RESET_USE_PCR

#define SPEED_100KHZ_BUS			0
#define SPEED_400KHZ_BUS			1
#define SPEED_1MHZ_BUS				2

#define I2C_CTRL_OWN_ADDR_1			0x5EU
#define I2C_CTRL_OWN_ADDR_2			0x5FU

#define I2C_MODE_CTRL				0
#define I2C_MODE_TARGET				1

/* I2C Read/Write bit pos */
#define I2C_READ_WRITE_POS			0

/* I2C SCL and SDA lines(signals) */
#define I2C_LINES_SCL_POS			0
#define I2C_LINES_SDA_POS			1
#define I2C_LINES_SCL_HI			BIT(I2C_LINES_SCL_POS)
#define I2C_LINES_SDA_HI			BIT(I2C_LINES_SDA_POS)
#define I2C_LINES_BOTH_HI			(I2C_LINES_SCL_HI | I2C_LINES_SDA_HI)

/* Network layer TX buffer size */
#define I2C_NWL_TX_MSG_MAX			64U
#define I2C_NWL_TX_BUF_SIZE			(I2C_NWL_TX_MSG_MAX + 4U)

#define I2C_XEC_CFG_NO_WAIT_MS			0
#define I2C_XEC_CFG_WAIT_MS			35
#define RESET_WAIT_US				20
#define I2C_ENABLE_WAIT_US			160
#define WAIT_MPROCEED_US			2
#define WAIT_MPROCEED_LOOPS			100

/* I2C recover SCL low retries */
#define I2C_RECOVER_SCL_LOW_RETRIES		10
/* I2C recover SDA low retries */
#define I2C_RECOVER_SDA_LOW_RETRIES		3
/* I2C recovery bit bang delay */
#define I2C_RECOVER_BB_DELAY_US			5
/* I2C recovery SCL sample delay */
#define I2C_RECOVER_SCL_DELAY_US		50

/* driver states */
#define I2C_XEC_STATE_CLOSED			0
#define I2C_XEC_STATE_OPEN			1

/* driver flags */
#define I2C_XEC_EVENT_NONE			0
#define I2C_XEC_EVENT_START			1
#define I2C_XEC_EVENT_RPT_START			2
#define I2C_XEC_EVENT_STOP			3
#define I2C_XEC_EVENT_DATA_XFR_DONE		4

#define I2C_XEC_DIR_NONE			0
#define I2C_XEC_DIR_WR				1
#define I2C_XEC_DIR_RD				2

#define I2C_XEC_ACTION_NONE			0
#define I2C_XEC_ACTION_START			1
#define I2C_XEC_ACTION_RPT_START		2

#define I2C_XEC_RESET_HW_ANOMALY_ATTEMPTS	4

/* I2C control WO */
#define XEC_I2C_CTR_ACK_POS			0
#define XEC_I2C_CTR_STO_POS			1
#define XEC_I2C_CTR_STA_POS			2
#define XEC_I2C_CTR_ENI_POS			3
#define XEC_I2C_CTR_ESO_POS			6
#define XEC_I2C_CTR_PIN_POS			7
/* I2C status RO */
#define XEC_I2C_STS_NBB_POS			0
#define XEC_I2C_STS_LAB_POS			1
#define XEC_I2C_STS_AAT_POS			2
#define XEC_I2C_STS_LRB_AD0_POS			3
#define XEC_I2C_STS_BER_POS			4
#define XEC_I2C_STS_EXT_STO_POS			5
#define XEC_I2C_STS_SAD_POS			6
#define XEC_I2C_STS_PIN_POS			7
/* own address */
#define XEC_I2C_OWN_ADDR_MSK			0x7f7fu
#define XEC_I2C_OWN_ADDR0_POS			0
#define XEC_I2C_OWN_ADDR1_POS			8
/* Controller Mode Command */
#define XEC_I2C_CM_CMD_MSK			0xffff3f03u
#define XEC_I2C_CM_MRUN_POS			0
#define XEC_I2C_CM_MPROCEED_POS			1
#define XEC_I2C_CM_START0_POS			8
#define XEC_I2C_CM_STARTN_POS			9
#define XEC_I2C_CM_STOP_POS			10
#define XEC_I2C_CM_PEC_TERM_POS			11
#define XEC_I2C_CM_READM_POS			12
#define XEC_I2C_CM_READ_PEC_POS			13
#define XEC_I2C_CM_WCNT_POS			16
#define XEC_I2C_CM_RCNT_POS			24
#define XEC_I2C_CM_WCNT_MSK			0xff0000u
#define XEC_I2C_CM_RCNT_MSK			0xff000000u
#define XEC_I2C_CM_WRCNT_MSK0			0xffu
/* Target Mode Command */
#define XEC_I2C_TM_CMD_MSK			0x00ffff07u
#define XEC_I2C_TM_TRUN_POS			0
#define XEC_I2C_TM_TPROCEED_POS			1
#define XEC_I2C_TM_TPEC_POS			2
#define XEC_I2C_TM_WCNT_POS			8
#define XEC_I2C_TM_RCNT_POS			16
#define XEC_I2C_TM_WCNT_MSK			0xff00u
#define XEC_I2C_TM_RCNT_MSK			0xff0000u
#define XEC_I2C_TM_WRCNT_MSK0			0xffu
/* Extended length */
#define XEC_I2C_EXTLEN_MSK			0xffffu
#define XEC_I2C_EXT_WCNT_POS			0
#define XEC_I2C_EXT_RCNT_POS			8
#define XEC_I2C_EXT_WCNT_MSK			0xffu
#define XEC_I2C_EXT_RCNT_MSK			0xff00u
/* Completion */
#define XEC_I2C_COMPL_MSK			0xe33b7f7cu
#define XEC_I2C_COMPL_STS_RW1C_MSK		0xe1397f00u
#define XEC_I2C_COMPL_STS_RO_MSK		0x02020040u
#define XEC_I2C_COMPL_DTEN_POS			2
#define XEC_I2C_COMPL_MCEN_POS			3
#define XEC_I2C_COMPL_SCEN_POS			4
#define XEC_I2C_COMPL_BIDEN_POS			5
#define XEC_I2C_COMPL_TMOUT_STS_POS		6
#define XEC_I2C_COMPL_DTO_STS_POS		8
#define XEC_I2C_COMPL_MCTO_STS_POS		9
#define XEC_I2C_COMPL_SCTO_STS_POS		10
#define XEC_I2C_COMPL_CHDL_STS_POS		11
#define XEC_I2C_COMPL_CHDH_STS_POS		12
#define XEC_I2C_COMPL_BER_STS_POS		13
#define XEC_I2C_COMPL_LAB_STS_POS		14
#define XEC_I2C_COMPL_TMR_NAK_STS_POS		16
#define XEC_I2C_COMPL_TM_RX_RO_POS		17
#define XEC_I2C_COMPL_SPROT_STS_POS		19
#define XEC_I2C_COMPL_RPT_RD_STS_POS		20
#define XEC_I2C_COMPL_RPT_WR_STS_POS		21
#define XEC_I2C_COMPL_CMT_NAK_STS_POS		24
#define XEC_I2C_COMPL_CM_TX_RO_POS		25
#define XEC_I2C_COMPL_IDLE_STS_POS		29
#define XEC_I2C_COMPL_CMD_STS_POS		30
#define XEC_I2C_COMPL_TMD_STS_POS		31
#define XEC_I2C_COMPL_CM_ERR			(BIT(XEC_I2C_COMPL_BER_STS_POS) |\
						 BIT(XEC_I2C_COMPL_LAB_STS_POS) |\
						 BIT(XEC_I2C_COMPL_CMT_NAK_STS_POS))
/* Configuration */
#define XEC_I2C_CFG_MSK				0xf00fdfbfu
#define XEC_I2C_CFG_PORT_SEL_POS		0
#define XEC_I2C_CFG_PORT_SEL_MSK		0xfu
#define XEC_I2C_CFG_TCEN_POS			4
#define XEC_I2C_CFG_SLOW_CLK_POS		5
#define XEC_I2C_CFG_PCEN_POS			7
#define XEC_I2C_CFG_FEN_POS			8
#define XEC_I2C_CFG_SRST_POS			9
#define XEC_I2C_CFG_EN_POS			10
#define XEC_I2C_CFG_DSA_EN_POS			11
#define XEC_I2C_CFG_FAIR_EN_POS			12
#define XEC_I2C_CFG_TM_GC_EN_POS		14
#define XEC_I2C_CFG_PROM_EN_POS			15
/* bits 16:19 write-only */
#define XEC_I2C_CFG_FLUSH_TM_TXB_POS		16
#define XEC_I2C_CFG_FLUSH_TM_RXB_POS		17
#define XEC_I2C_CFG_FLUSH_CM_TXB_POS		18
#define XEC_I2C_CFG_FLUSH_CM_RXB_POS		19
#define XEC_I2C_CFG_FLUSH_ALL			0xf0000u
#define XEC_I2C_CFG_AAT_IEN_POS			28
#define XEC_I2C_CFG_IDLE_IEN_POS		29
#define XEC_I2C_CFG_CMD_IEN_POS			30
#define XEC_I2C_CFG_TMD_IEN_POS			31
/* bit bang control */
#define XEC_I2C_BBCTR_MSK			0x7fu
#define XEC_I2C_BBCTR_EN_POS			0
#define XEC_I2C_BBCTR_SCL_OUT_EN_POS		1
#define XEC_I2C_BBCTR_SDA_OUT_EN_POS		2
#define XEC_I2C_BBCTR_SCL_TRI_POS		3
#define XEC_I2C_BBCTR_SDA_TRI_POS		4
#define XEC_I2C_BBCTR_SCL_PIN_RO_POS		5
#define XEC_I2C_BBCTR_SDA_PIN_RO_POS		6

/* Bit-bang mode: enable, SCL and SDA pins are inputs */
#define XEC_BB_SCL_SDA_TRI_IN			(BIT(XEC_I2C_BBCTR_EN_POS)		\
						 | BIT(XEC_I2C_BBCTR_SCL_TRI_POS)	\
						 | BIT(XEC_I2C_BBCTR_SDA_TRI_POS))

/* Bit-bang mode: enable, SCL output mode and tri-state(not driven), SDA input */
#define XEC_BB_SCL_OUT_HI_SDA_TRI		(BIT(XEC_I2C_BBCTR_EN_POS)		\
						 | BIT(XEC_I2C_BBCTR_SCL_OUT_EN_POS)	\
						 | BIT(XEC_I2C_BBCTR_SCL_TRI_POS)	\
						 | BIT(XEC_I2C_BBCTR_SDA_TRI_POS))

/* Bit-bang mode: enable, SCL output mode driven low, SDA is input */
#define XEC_BB_SCL_OUT_DRV_LO_SDA_TRI		(BIT(XEC_I2C_BBCTR_EN_POS)		\
						 | BIT(XEC_I2C_BBCTR_SCL_OUT_EN_POS))

/* Bit-bang mode: enable, SCL output mode driven low, SDA is input */
#define XEC_BB_SCL_TRI_SDA_OUT_DRV_LO		(BIT(XEC_I2C_BBCTR_EN_POS)		\
						 | BIT(XEC_I2C_BBCTR_SDA_OUT_EN_POS))

#define XEC_BB_SCL_IN_HI_SDA_IN_HI		(BIT(XEC_I2C_BBCTR_SCL_PIN_RO_POS)	\
						 | BIT(XEC_I2C_BBCTR_SDA_PIN_RO_POS))

/* wake status and wake enable registers */
#define XEC_I2C_WK_EN_STS_MSK			0x1u
#define XEC_I2C_WK_EN_STS_STA_DET_POS		0

#define XEC_I2C_NL_MAX_LEN			0xfff8u

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
	struct mchp_xec_pcr_clk_ctrl clksrc;
	const struct device *clk_dev;
	const struct device *dma_dev;
	const struct pinctrl_dev_config *pcfg;
	const struct gpio_dt_spec scl_gpio;
	const struct gpio_dt_spec sda_gpio;
	void (*irq_config_func)(void);
	int32_t init_pin_wait_us;
	int32_t cfg_pin_wait_us;
	uint8_t port_sel;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t ctr_dma_chan;
	uint8_t ctr_dma_id;
	uint8_t dev_dma_chan;
	uint8_t dev_dma_id;
	uint8_t target_addr1;
	uint8_t target_addr2;
};

#define XEC_I2C_NL_MODE_SYNC 0
#define XEC_I2C_NL_MODE_ASYNC 1

#ifdef XEC_I2C_NL_DEBUG

#define I2C_NL_GPIO_0012_CTRL_ADDR (0x40081000u + 0x28) /* J19-3 */
#define I2C_NL_GPIO_0013_CTRL_ADDR (0x40081000u + 0x2c) /* J19-1 */
#define I2C_NL_GPIO_0130_CTRL_ADDR (0x40081000u + 0x160) /* J20-3 */
#define I2C_NL_GPIO_0131_CTRL_ADDR (0x40081000u + 0x164) /* J20-1 */

#if 0
sys_write32(0x00240u, I2C_NL_GPIO_0012_CTRL_ADDR); /* GPIO output drive low */
sys_write32(0x10240u, I2C_NL_GPIO_0012_CTRL_ADDR); /* GPIO output drive high */
#endif


struct xec_nl_cm_cmd {
	uint32_t dir;
	uint32_t cm_cmd;
};

#define UPDATE_DBG_CM_CMD_IDX(n) (((uint32_t)(n) + 1u) & 0xfu)

struct xec_nl_ev {
	uint32_t id;
	uint32_t evmsk;
	uint32_t ev;
};

struct xec_nl_idat {
	uint8_t id;
	uint8_t i2c_sts;
	uint8_t rsvd[2];
	uint32_t config;
	uint32_t compl;
	uint32_t cm_cmd;
};
#endif

#define I2C_XEC_NL_TXB_LEN 8

#ifdef XEC_I2C_CONTEXT
struct i2c_context {
	struct i2c_msg *curr_msg;
	int msg_count;
	uint8_t *msg_buf;
	uint32_t msg_len;
};
#endif

#define I2C_XEC_NL_EVENT_I2C_CM_DONE		BIT(0)
#define I2C_XEC_NL_EVENT_I2C_CM_ERR		BIT(1)
#define I2C_XEC_NL_EVENT_I2C_CM_STOP		BIT(2)
#define I2C_XEC_NL_EVENT_DMA_TX_DONE		BIT(8)
#define I2C_XEC_NL_EVENT_DMA_TX_ERR		BIT(9)
#define I2C_XEC_NL_EVENT_DMA_RX_DONE		BIT(10)
#define I2C_XEC_NL_EVENT_DMA_RX_ERR		BIT(11)

#define I2C_XEC_NL_EVENT_I2C_CM_ALL		(I2C_XEC_NL_EVENT_I2C_CM_DONE		\
						 | I2C_XEC_NL_EVENT_I2C_CM_ERR		\
						 | I2C_XEC_NL_EVENT_I2C_CM_STOP)

#define I2C_XEC_NL_EVENT_DMA_ALL		(I2C_XEC_NL_EVENT_DMA_TX_DONE		\
						 | I2C_XEC_NL_EVENT_DMA_TX_ERR		\
						 | I2C_XEC_NL_EVENT_DMA_RX_DONE		\
						 | I2C_XEC_NL_EVENT_DMA_RX_ERR)

struct i2c_xec_nl_data {
	struct k_mutex ctr_mutex;
	struct k_event events;
	uint8_t i2c_ctr;
	uint8_t i2c_addr;
	uint8_t speed_id;
	uint8_t state;
	uint8_t mode;
	uint8_t prev_dir;
	uint8_t curr_dir;
	uint8_t cm_active;
	uint32_t cm_cmd;
	volatile int dma_err;
	volatile uint32_t i2c_cm_cmd;
	volatile uint32_t i2c_compl;
	volatile uint32_t i2c_config;
	volatile uint8_t i2c_status;
	volatile uint8_t ctr_start_req;
	volatile uint8_t ctr_stop_req;
#ifdef XEC_I2C_CONTEXT
	struct i2c_context ctx;
#endif /* XEC_I2C_CONTEXT */
	uint8_t txblen;
	uint8_t rsvd1[3];
	uint8_t txb[I2C_XEC_NL_TXB_LEN];
#ifdef CONFIG_I2C_CALLBACK
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
	uint32_t dma_mstart;
	uint32_t dma_len;
	uint32_t dma_dev_addr;
	volatile uint32_t isr_cnt;
	volatile uint32_t i2c_fsm;
	volatile uint32_t smb_fsm;
	volatile uint32_t dma_ev;
	volatile uint32_t i2c_ev;
	uint8_t wait_state;
	uint8_t didx;
	uint8_t eidx;
	uint8_t iidx;
	struct xec_nl_cm_cmd dbg_cm_cmd[16];
	struct xec_nl_ev dbg_nl_ev[16];
	struct xec_nl_idat idat[16];
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

static int i2c_xec_nl_msg_tx(const struct device *dev, int async);
static int i2c_xec_nl_msg_rx(const struct device *dev, int async);

#ifdef XEC_I2C_CONTEXT
static inline void i2c_context_msg_setup(struct i2c_context *ctx, struct i2c_msg *msgs, uint8_t num_msgs)
{
	ctx->curr_msg = msgs;
	ctx->msg_count = (int)num_msgs;
	ctx->msg_buf = msgs ? msgs->buf : NULL;
	ctx->msg_len = msgs ? msgs->len : 0;
}

static inline bool i2c_context_on(struct i2c_context *ctx)
{
	return !!(ctx->msg_len);
}

static inline bool i2c_context_msg_on(struct i2c_context *ctx)
{
	return !!(ctx->msg_buf && ctx->msg_len);
}

static inline bool i2c_context_msg_stop(struct i2c_context *ctx)
{
	return !!((ctx->curr_msg->len == 0) && (ctx->curr_msg->flags & I2C_MSG_STOP));
}

/* How can we make this work for when we must transmit address on START or RPT-START
 * and then process messages?
 * This assumes an array of messages but transmit of address is a run-time generated
 * message we can't add to the front of the passed messages.
 * If we do address transmit, do not call this function unless we double buffer.
 * Double buffer is copy some of message after address into small buffer used
 * for address.
 */
static inline void i2c_context_update(struct i2c_context *ctx, uint32_t len)
{
	if (!ctx->msg_len) {
		return;
	}

	if (len > ctx->msg_len) {
		/* TODO ERROR condition */
		return;
	}

	ctx->msg_len -= len;
	if (!ctx->msg_len) { /* current msg done, get next */
		++ctx->curr_msg; /* increment message pointer */
		--ctx->msg_count; /* decrement message count */
		while (ctx->msg_count) {
			if (ctx->curr_msg->len) {
				ctx->msg_len = ctx->curr_msg->len;
				ctx->msg_buf = ctx->curr_msg->buf;
				return;
			}
			++ctx->curr_msg;
			--ctx->msg_count;
		}
		ctx->msg_len = 0;
		ctx->msg_buf = NULL;
	} else {
		ctx->msg_buf += len;
	}
}
#endif /* XEC_I2C_CONTEXT */

#ifdef XEC_I2C_NL_DEBUG

static uint32_t i2c_nl_dbg_tx_wait_cnt[4];

static void init_dbg(struct i2c_xec_nl_data * const data)
{
	data->didx = 0;
	data->eidx = 0;
	data->iidx = 0;
	data->isr_cnt = 0;
	data->dma_ev = 0;
	data->i2c_ev = 0;
	memset(data->dbg_cm_cmd, 0, sizeof(data->dbg_cm_cmd));
	memset(data->dbg_nl_ev, 0, sizeof(data->dbg_nl_ev));
	memset(data->idat, 0, sizeof(data->idat));
	memset(i2c_nl_dbg_tx_wait_cnt, 0, sizeof(i2c_nl_dbg_tx_wait_cnt));
}

static void update_dbg_cmd_idx(struct i2c_xec_nl_data * const data, uint8_t dir, uint32_t cm_cmd)
{
	uint32_t didx = data->didx;

	data->dbg_cm_cmd[didx].dir = dir;
	data->dbg_cm_cmd[didx].cm_cmd = cm_cmd;
	data->didx = UPDATE_DBG_CM_CMD_IDX(didx);
}

static void update_dbg_ev_idx(struct i2c_xec_nl_data * const data, uint32_t id, uint32_t evmsk, uint32_t ev)
{
	uint32_t eidx = data->eidx;

	data->dbg_nl_ev[eidx].id = id;
	data->dbg_nl_ev[eidx].evmsk = evmsk;
	data->dbg_nl_ev[eidx].ev = ev;
	data->eidx = ++eidx & 0x0fu;
}

static void update_dbg_idat_idx(struct i2c_xec_nl_data * const data, uint8_t id, uint8_t ists,
				uint32_t config, uint32_t compl, uint32_t cm_cmd)
{
	uint8_t idx = data->iidx;

	data->idat[idx].id = id;
	data->idat[idx].i2c_sts = ists;
	data->idat[idx].config = config;
	data->idat[idx].compl = compl;
	data->idat[idx].cm_cmd = cm_cmd;
	data->iidx = ++idx & 0xfu;
}
#endif

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

static void ctrl_io_delay(const struct device *dev)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;

	regs->blk_id = 0u; /* I/O delay, minimum 3 AHB (48 MHz) clocks */
	regs->blk_id = 0u;
	regs->blk_id = 0u;
	regs->blk_id = 0u;
}

#ifdef XEC_I2C_RESET_USE_PCR
/* A more in depth reset via the PCR peripheral reset feature */
static void ctrl_reset(const struct device *dev)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	const struct mchp_xec_pcr_clk_ctrl *pclksrc = &cfg->clksrc;
	uint8_t slp_idx = MCHP_XEC_PCR_SCR_GET_IDX(pclksrc->pcr_info);
	uint8_t slp_pos = MCHP_XEC_PCR_SCR_GET_BITPOS(pclksrc->pcr_info);

	z_mchp_xec_pcr_periph_reset(slp_idx, slp_pos);
}
#else
/* I2C controller reset anomaly; does not reset all parts of the controller. */
static void ctrl_reset(const struct device *dev)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;

	LOG_DBG("controller reset");

	/* assert reset for >= 1 BAUD clock (16 MHz) */
	regs->config = BIT(XEC_I2C_CFG_SRST_POS);
	ctrl_io_delay(dev);
	regs->config &= ~BIT(XEC_I2C_CFG_SRST_POS);

}
#endif /* XEC_I2C_RESET_USE_PCR */

static void ctrl_cleanup(const struct device *dev)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;

	LOG_DBG("controller cleanup");

	/* clear interrupt enables */
	regs->config &= ~(BIT(XEC_I2C_CFG_AAT_IEN_POS) | BIT(XEC_I2C_CFG_IDLE_IEN_POS)
			  | BIT(XEC_I2C_CFG_CMD_IEN_POS));
	ctrl_reset(dev);
	/* HW Anomaly */
	regs->cm_cmd |= BIT(XEC_I2C_CM_MRUN_POS);
	regs->tm_cmd |= BIT(XEC_I2C_TM_TRUN_POS);
	regs->cm_cmd &= ~BIT(XEC_I2C_CM_MRUN_POS);
	regs->tm_cmd &= ~BIT(XEC_I2C_TM_TRUN_POS);
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

/* Prerequite is controller reset.
 * Port selection, bus clock, and filter should be configured before controller enabled
 */
static void ctrl_config1(const struct device *dev)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct i2c_xec_nl_data * const data = dev->data;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	uint8_t i2c_ctr = (BIT(XEC_I2C_CTR_PIN_POS) | BIT(XEC_I2C_CTR_ESO_POS)
			   | BIT(XEC_I2C_CTR_ACK_POS));

	/* set port mux and enable digital filter */
	regs->config = (cfg->port_sel & 0x0fu) | BIT(XEC_I2C_CFG_FEN_POS);
	ctrl_io_delay(dev);

	/* clear low level status */
	regs->ctr_sts = BIT(XEC_I2C_CTR_PIN_POS);
	ctrl_io_delay(dev);
	/* clear sticky status */
	regs->compl = XEC_I2C_COMPL_STS_RW1C_MSK;
	ctrl_io_delay(dev);

	if (data->speed_id > ARRAY_SIZE(xec_cfg_params)) {
		data->speed_id = 0U;
	}

	const struct xec_nl_speed_cfg *spdcfg = &xec_cfg_params[data->speed_id];

	regs->bus_clk = spdcfg->bus_clk;
	regs->data_timing = spdcfg->data_timing;
	regs->rsht = spdcfg->start_hold_time;
	regs->idle_scaling = spdcfg->idle_scale;
	regs->tmout_scaling = spdcfg->timeout_scale;

	regs->own_addr = (cfg->target_addr1 & 0x7fu)
			 | ((uint32_t)(cfg->target_addr2 & 0x7fu) << 8);
#ifdef CONFIG_I2C_TARGET
	i2c_xec_nl_cfg_target_addr(dev);
#endif

	/* clear low level status, enable output drive, enable ACK generation */
	ctrl_io_delay(dev);
	regs->ctr_sts = i2c_ctr;
	data->i2c_ctr = i2c_ctr;
	ctrl_io_delay(dev);

	/* clear sticky status */
	regs->compl = XEC_I2C_COMPL_STS_RW1C_MSK;
	ctrl_io_delay(dev);

	/* enable controller */
	regs->config |= BIT(XEC_I2C_CFG_EN_POS);
	ctrl_io_delay(dev);
}

/* Delay after controller is configured and enabled.
 * Controller must sample I2C lines to determine state of the bus and
 * presence of external Controllers currently using the bus.
 */
static int ctrl_config(const struct device *dev, int32_t pin_wait_us)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct i2c_xec_nl_data * const data = dev->data;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	int attempts = 4u;

	data->state = I2C_XEC_STATE_CLOSED;
	data->curr_dir = I2C_XEC_DIR_NONE;
	data->prev_dir = I2C_XEC_DIR_NONE;

	/* HW anomaly on some resets */
	while (attempts--) {
		ctrl_reset(dev);
		ctrl_config1(dev);
		data->i2c_status = regs->ctr_sts;
		if (data->i2c_status == (BIT(XEC_I2C_STS_PIN_POS) | BIT(XEC_I2C_STS_NBB_POS))) {
			break;
		}
		if (data->i2c_status == BIT(XEC_I2C_STS_PIN_POS)) {
			ctrl_cleanup(dev);
		}
	}

	/* wait for controller to sample pins */
	if (pin_wait_us < 1000) {
		k_busy_wait(pin_wait_us);
	} else {
		k_msleep(pin_wait_us / 1000u);
	}

	data->i2c_status = regs->ctr_sts;
	data->cm_active = 0;

	if (data->i2c_status != (BIT(XEC_I2C_STS_PIN_POS) | BIT(XEC_I2C_STS_NBB_POS))) {
		LOG_ERR("I2C XEC NL: Error after config");
		return -EIO;
	}

	return 0;
}

static int i2c_xec_nl_config(const struct device *dev, uint32_t dev_config, int32_t pin_wait_us)
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
	ret = ctrl_config(dev, pin_wait_us);
	k_mutex_unlock(&data->ctr_mutex);

	return ret;
}

/* Zephyr I2C configure API implementation */
static int i2c_xec_nl_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;

	return i2c_xec_nl_config(dev, dev_config, cfg->cfg_pin_wait_us);
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
	ctrl_cleanup(dev);
	ctrl_config(dev, cfg->cfg_pin_wait_us); /* reset and reconfigure controller */

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
			ret = ctrl_config(dev, cfg->cfg_pin_wait_us);
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

/* We are using one DMA callback for both TX and RX cases.
 * TODO
 * Sync/Async TX: Set dma_err and dma_done in driver data structure
 * Sync RX: Same as TX
 * Async RX: various scenarios must be handled
 *   1. I2C message has I2C_MSG_STOP (last read message)
 *   2. I2C error: NAK, bus error, or lost arbitration
 *   3. No I2C STOP or error. I2C-NL read count does not reach 0
 *      RX DMA reads last byte from I2C-NL, writes to memory, and signals DMA done.
 *      The read of last byte from I2C-NL triggers I2C-NL decrement its read count
 *      and generate clocks to read-ahead. I2C-NL has two buffers so it can read
 *      ahead up to two bytes (18 clocks).
 *      Async operation requires this callback to check if another read message
 *      is available and reprogram DMA channel for the next message.
 *      NOTE: It is safe to touch DMA while I2C-NL read-ahead is in progress.
 *      Before arming RX DMA we must update I2C-NL CM_CMD read count.
 *      !!! Is touching I2C-NL CM_CMD safe while read ahead is in progress !!!
 */
static void i2c_xec_nl_dma_tx_done_cb(const struct device *dev, void *arg,
				      uint32_t id, int error_code)
{
	struct i2c_xec_nl_data * const data = ((const struct device *)arg)->data;
	uint32_t dma_events = I2C_XEC_NL_EVENT_DMA_TX_DONE;

	ARG_UNUSED(dev);

	/* sys_write32(0x00240u, I2C_NL_GPIO_0130_CTRL_ADDR); */

	data->dma_err = error_code;

	if (error_code) {
		dma_events = I2C_XEC_NL_EVENT_DMA_TX_ERR;
	}

#ifdef XEC_I2C_NL_DEBUG
	data->dma_ev |= dma_events;
#endif

	k_event_post(&data->events, dma_events);
	/* sys_write32(0x10240u, I2C_NL_GPIO_0130_CTRL_ADDR); */
}

static void i2c_xec_nl_dma_rx_done_cb(const struct device *dev, void *arg,
				      uint32_t id, int error_code)
{
	struct i2c_xec_nl_data * const data = ((const struct device *)arg)->data;
	uint32_t dma_events = I2C_XEC_NL_EVENT_DMA_RX_DONE;

	ARG_UNUSED(dev);

	sys_write32(0x00240u, I2C_NL_GPIO_0131_CTRL_ADDR);

	data->dma_err = error_code;

	if (error_code) {
		dma_events = I2C_XEC_NL_EVENT_DMA_RX_ERR;
	}

#ifdef XEC_I2C_NL_DEBUG
	data->dma_ev |= dma_events;
#endif

#ifdef CONFIG_I2C_CALLBACK
	/* TODO - rework using k_event_post */
#endif

	k_event_post(&data->events, dma_events);
	sys_write32(0x10240u, I2C_NL_GPIO_0131_CTRL_ADDR);
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
	struct dma_config dma_cfg = { 0 };				/* 7 * 4 = 28 bytes */
	struct dma_block_config dma_block_cfg = { 0 };			/* 9 * 4 = 36 bytes */
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
	k_event_clear(&data->events, I2C_XEC_NL_EVENT_I2C_CM_ALL | I2C_XEC_NL_EVENT_DMA_ALL);
	data->dma_err = 0;
	data->i2c_compl = 0;
	data->cm_cmd = 0;
	data->i2c_cm_cmd = 0;
	data->i2c_config = 0;
#ifdef XEC_I2C_NL_DEBUG
	data->i2c_fsm = 0;
	data->smb_fsm = 0;
	data->dma_ev = 0;
	data->i2c_ev = 0;
#endif
}

/* TODO 2023-02-28
 * IF data->prev_dir == I2C_XEC_DIR_NONE or ((data->prev_dir == I2C_XEC_DIR_WR) && I2C_MSG_RESTART)
 *    Must do START0.
 *    dmalen = message length
 *    wrCnt = dmalen + 1
 *    Must use dma config if (data->prev_dir == I2C_XEC_DIR_NONE) else dma_reload
 *    FW writes wrAddr to CM_TX_Data register
 *      Write cm_cmd to CM_CMD register
 *      Write wrAddr to CM_TX_Data register
 *      Start DMA
 *
 * ELSE IF data->prev_dir == I2C_XEC_DIR_RD  Caller is handling this.
 *   Must do STOP and wait for STOP completion: I2C-NL.Compl.IDLE 0 -> 1
 *   Must do START0 sequence above
 */
/* Use DMA for all data transfer to I2C-NL.CM_TX_Data register.
 * We suspect must follow DMA_REQ signal timing.
 * Using CPU to write CM_TX_Data works > 90% of the time but after
 * recompile and instruction sequences change in the overall Zephyr binary
 * we see failures.
 * Scenario 1. No (RPT-)START required.
 * 	Configure DMA for TX of (m->buf, m->len)
 *	Start transfer
 *	Wait for Done
 * Scenario 2. Require (RPT-)START
 *	Configure DMA for TX of wrAddr
 *	Start transfer
 *	Wait for Done
 *	Configure DMA for TX of (m->buf, m->len)
 *	Start transfer
 *	Wait for Done
 * Scenario 3. Require (RPT-)START and we use a driver based TX buffer
 *	If m->len < len(txb) - 1
 *		txb[0] = fmt_addr
 *		memcpy(&txb[1], m->buf, m->len)
 *		One DMA TX configure for m->len + 1 bytes
 *		Start Transfer
 *		Wait for Done
 *	else
 *		Same as Scenario 2
 *
 */
static int i2c_xec_nl_msg_tx(const struct device *dev, int async)
{
	struct i2c_xec_nl_data * const data = dev->data;
	struct i2c_context *ctx = &data->ctx;
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	uint32_t cfg_ien = 0, cm_cmd = 0, consumed = 0, txlen = 0, ev = 0, evmsk = 0;
	uint8_t fmt_addr = (data->i2c_addr & 0x7fu) << 1;

	if (!i2c_context_msg_on(ctx)) {
		return 0;
	}

#ifdef XEC_I2C_NL_DEBUG
	update_dbg_cmd_idx(data, 0x10u, regs->cm_cmd);
#endif

	clr_isr_flags(data);
	regs->compl = XEC_I2C_COMPL_STS_RW1C_MSK;

	evmsk |= (I2C_XEC_NL_EVENT_I2C_CM_ERR | I2C_XEC_NL_EVENT_DMA_TX_ERR);
	cm_cmd = BIT(XEC_I2C_CM_MRUN_POS) | BIT(XEC_I2C_CM_MPROCEED_POS);
	if ((data->prev_dir == I2C_XEC_DIR_NONE) || (ctx->curr_msg->flags & I2C_MSG_RESTART)) {
		if (data->prev_dir == I2C_XEC_DIR_NONE) {
			data->ctr_start_req = 1;
		}
		data->txb[0] = fmt_addr;
		cm_cmd |= BIT(XEC_I2C_CM_START0_POS);
		txlen = 1u;

		consumed = ctx->msg_len;
		if (consumed > (I2C_XEC_NL_TXB_LEN - 1u)) {
			consumed = (I2C_XEC_NL_TXB_LEN - 1u);
		}
		memcpy(&data->txb[1], ctx->msg_buf, consumed);
		txlen += consumed;

		if (data->prev_dir == I2C_XEC_DIR_WR) {
			dma_reload(cfg->dma_dev, cfg->ctr_dma_chan, (uint32_t)data->txb,
				   (uint32_t)&regs->cm_txb, txlen);
		} else {
			i2c_nl_dma_cfg(dev, (uint32_t)data->txb, (uint32_t)&regs->cm_txb,
				       txlen, MEMORY_TO_PERIPHERAL, i2c_xec_nl_dma_tx_done_cb);
		}
		dma_start(cfg->dma_dev, cfg->ctr_dma_chan);

		if ((consumed == ctx->msg_len) && (ctx->curr_msg->flags & I2C_MSG_STOP)) {
			cm_cmd |= BIT(XEC_I2C_CM_STOP_POS);
			data->ctr_stop_req = 1;
			evmsk |= I2C_XEC_NL_EVENT_I2C_CM_STOP;
		} else {
			evmsk |= I2C_XEC_NL_EVENT_I2C_CM_DONE;
		}

		cfg_ien |= BIT(XEC_I2C_CFG_CMD_IEN_POS);

		regs->extlen = (txlen >> 8) & 0xffu;
		cm_cmd |= ((txlen & 0xffu) << XEC_I2C_CM_WCNT_POS);
	} else { /* message only */
		data->ctr_start_req = 0;
		dma_reload(cfg->dma_dev, cfg->ctr_dma_chan, (uint32_t)ctx->msg_buf,
			   (uint32_t)&regs->cm_txb, ctx->msg_len);
		dma_start(cfg->dma_dev, cfg->ctr_dma_chan);

		consumed = ctx->msg_len;
		regs->extlen = (ctx->msg_len >> 8) & 0xffu;
		cm_cmd |= ((ctx->msg_len & 0xffu) << XEC_I2C_CM_WCNT_POS);
		cfg_ien |= BIT(XEC_I2C_CFG_CMD_IEN_POS);
		if (ctx->curr_msg->flags & I2C_MSG_STOP) {
			cm_cmd |= BIT(XEC_I2C_CM_STOP_POS);
			data->ctr_stop_req = 1;
			evmsk |= I2C_XEC_NL_EVENT_I2C_CM_STOP;
		} else {
			evmsk |= I2C_XEC_NL_EVENT_I2C_CM_DONE;
		}
	}

#ifdef XEC_I2C_NL_DEBUG
	update_dbg_cmd_idx(data, 0x11u, cm_cmd);
#endif
	/* experiment always enable IDLE and add STOP event flag */
	cfg_ien |= BIT(XEC_I2C_CFG_IDLE_IEN_POS);
	evmsk |= I2C_XEC_NL_EVENT_I2C_CM_STOP;
	/* experiment end */
	data->cm_cmd = cm_cmd;
	sys_write32(0x00240u, I2C_NL_GPIO_0131_CTRL_ADDR);
	regs->cm_cmd = cm_cmd;
	sys_write32(0x10240u, I2C_NL_GPIO_0131_CTRL_ADDR);
	regs->config |= cfg_ien;
	sys_write32(0x00240u, I2C_NL_GPIO_0131_CTRL_ADDR);

	if (!async) {
#ifdef XEC_I2C_NL_DEBUG
		update_dbg_ev_idx(data, 0x11u, evmsk, 0);
#endif
#ifdef XEC_I2C_NL_DEBUG_POLL_EVENTS
		do {
			ev = k_event_wait(&data->events, evmsk, false, K_NO_WAIT);
		} while (ev == 0);
#else
		ev = k_event_wait(&data->events, evmsk, false, K_FOREVER);
#endif
		if (ev & (I2C_XEC_NL_EVENT_I2C_CM_ERR | I2C_XEC_NL_EVENT_DMA_TX_ERR)) {
			return -EIO;
		}
		i2c_context_update(ctx, consumed);
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
 * When (Rpt)START is required we can't arm RX DMA before the address
 * is transmitted here because the HW design uses one set of DMA signals
 * for both TX and RX. HW FSM transmits read address first which can
 * trigger RX DMA. When HW finishes transmit of read address it will
 * decrement HW write count to 0, clear MPROCEED, and generate
 * Controller-Mode Done interrupt. In the ISR if we observe MRUN==1 and
 * MPROCEED==0 we arm RX DMA, set MPROCEED back to 1, and return without
 * giving the semaphore. When HW finishes read and generates STOP, it
 * will decrement read count to 0, clear STOP, MRUN, MPROCEED, and
 * generate an I2C Controller-Mode interrupt. ISR will then give semaphore
 * and k_sem_take will return.
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

/* New i2c_xec_nl_msg_rx implementation
 * Handle state open or closed. More accurate:
 * if data->prev_dir == I2C_XEC_DIR_NONE then
 *   Require START0
 * else
 *   if I2C_MSG_RESTART flag then require STARTN (RPT-START)
 * endif
 *
 * If I2C_MSG_STOP flag is present
 *   set I2C-NL rdCnt to message length
 * else
 *   set I2C-NL rdCnt to max (0xfff8)
 *   rdCnt set once if msg has START0 or STARTN
 *
 *
 */
/* Configure and start I2C Controller-Mode read of target device.
 * Scenarios:
 * 1. Previous direction is none (closed):
 *    read count = message length. If I2C_MSG_STOP NOT present increment read count by 2.
 *    Configure DMA for TX of one byte the target rdAddr.
 *    Configure I2C-NL for TX 1 byte, RX message length, START0 flag, and STOP flag
 *      if message has I2C_MSG_STOP
 *    Start TX DMA
 *    Write I2C-NL CM command register to start I2C
 *    HW generates START, transmit rdAddr, sample (n)ACK from target
 *    If not async
 *       wait for I2C-NL done
 *       if error then return error upstream
 *       Re-program DMA for RX of actual message length.
 *       Set I2C-NL MPROCEED=1
 *       If HW STOP flag set
 *           Wait for I2C-NL done
 *       else
 *           Spin checking both I2C-NL done and RX DMA done
 *           I2C-NL done will only be set for error since read
 *           count is not 0.
 *           RX DMA done only, is success.
 * 2. Previous direction is Write:
 *    Same as 1 except use STARTN HW flag
 * 3. Previous direction is Read:
 *    read count = message length. If I2C_MSG_STOP NOT present increment read count by 2.
 *    Configure DMA for RX of actual message length
 *    Configure I2C-NL for read count. If I2C_MSG_STOP present set HW STOP bit.
 *    NOTE: At this point I2C-NL has HW read count == 2 and is asserting DMA_REQ
 *          signal. We program new value of I2C-NL CM command register first to
 *          update the read count and possibly the STOP flag before starting RX DMA.
 *    Write I2C-NL CM command register.
 *    Start RX DMA.
 *    If not async
 *      if HW STOP flag set
 *        Wait for I2C-NL done
 *      else
 *        Spin checking I2C-NL done and RX DMA done.
 *        I2C-NL done will only occur for an I2C error
 *        RX DMA done only is successful completion of
 *        data transfer.
 *        In this case when RX DMA is done and there's no I2C error
 *        I2C-NL HW does not clear MPROCEED or MRUN, only decrements
 *        read count.
 *
 * Knowlege obtained by experimenting with the controller.
 * I2C-NL only fires its interrupt on write or read count reaching 0 or an error
 * condition. For the special case of transition from I2C write to read due, HW fires
 * the interrupt when transmit of the read address and (n)ACK is complete. If the
 * the target ACK'd its address, HW also clears the MPROCEED HW bit. This signals
 * FW should re-configure the DMA channel for RX of the message data. FW then
 * sets MPROCEED to 1 signalling HW to start reading data from the target and
 * signalling DMA to move the data from I2C-NL CM_RX_Data register to memory.
 * If the target device NAK'd its address then I2C-NL sets the NAK status,
 * clears MRUN/MPROCEED, generates a STOP, and released the bus. NOTE: the
 * interrupt occurs on NAK status being set which is before STOP is generated.
 * STOP generation takes 1/2 I2C clock. FW could enabled the I2C IDLE interrupt
 * on NAK or Error to know when the bus is released after an error or STOP.
 *
 * Of course, the I2C-NL HW design doesn't make coding so easy.
 * I2C-NL HW will only fire the interrupt if HW write and/or read
 * count reaches 0 or on an I2C error. You will think let the read count
 * reach 0, like we do for write count. Other HW design "features" intervene:
 * automatic NAK of the last byte read by this controller coupled with
 * automatic read-ahead. Neither can be disabled.
 * When the read count reaches 2 the HW prepares to NAK the next byte read (last byte).
 * In addition when the read count reaches 0 the HW releases the bus:
 * I2C.ctr_sts.NBB bit goes to 1 indicating this controller no longer "owns" the bus.
 * The controller releases the bus independent of the HW STOP flag. This means the
 * bus could be releases without a STOP being generated. When I2C-NL has transitioned
 * to read mode, FW cannot allow HW read count to go below 2 except on the last
 * read message and FW must set HW STOP flag on this message.
 *
 * NOTE: About HW read-ahead
 * I2C-NL hardware is a wrapper above the low level byte-by-byte I2C HW.
 * Low level I2C HW has a one byte bi-direction data register, I2C.Data.
 * The low level I2C HW knows the direction by bit[0] of the last target
 * address transmitted. When HW is in read direction, any read of the I2C.Data
 * register returns current register contents and causes HW to generate clocks
 * for the next data byte from the target. I2C-NL HW adds a pair of data registers,
 * one for transmit and one for receive. Effecitely creating a two byte FIFO. When
 * both I2C.Data is empty and I2C-NL.CM_RX_Data are empty, reading I2C.CM_RX_Data
 * will trigger the I2C HW to read-ahead up to two bytes to fill both registers.
 * Read-ahead cannot be disabled by FW. Read-ahead is not performed when the HW
 * read count goes below 2. I2C-NL HW interprets read count < 2 as the transfer
 * is ending and it expects FW to have set the HW STOP bit. If HW STOP is not
 * I2C-NL HW will release the bus without generating a STOP!
 *
 * Multiple scenarios each handled differently
 * START0 no STOP.  rdCnt = msglen + 2
 * I2C interrupt #1: START0 done (wrCnt->0) and MPROCEED->0.
 *   Configure DMA for RX of message data
 *   Set MPROCEED to 1
 *   Start DMA
 *   ISR exits without giving semaphore
 * I2C interrupt #2: Only for NAK or HW error!
 * RX DMA callback must do processing.
 *   call i2c_update_context
 *   Configure DMA for next message, this could be direction switch!
 *
 * START0 and STOP
 * I2C interrupt #1: START0 done (wrCnt->0) and MPROCEED->0.
 *   Configure DMA for RX of message data
 *   Set MPROCEED to 1
 *   Start DMA
 *   ISR exits without giving semaphore
 * I2C interrupt #2: rdCnt->0 because STOP was present
 *   If not IDLE configure I2C to interrupt on IDLE (STOP done) and exit isr
 *   Set nl_done =1 and give semaphore
 *   exit isr
 *
 * TODO overall timeout?
 * Have all race conditions with HW been analyzed?
 * RX DMA always finishes first unless an I2C error occurs before
 * RX DMA has read its last byte from I2C-NL RX Data register.
 * In the no I2C error case, its OK to exit on DMA done, and
 * come back and reprogram RX DMA for next message.
 * The danger is not reprogramming RX DMA but changing I2C-NL command
 * register while HW is generating read-ahead clocks.
 * It appears the command register is "stable", read/write counts
 * only change in response to DMA access to I2C-NL registers.
 * If this is true then changing only the count may be safe.
 * !!! Need HW design confirmation !!!
 * !!! TODO POTENTIAL RACE CONDITIONS !!!
 * I2C higher propriority than DMA?
 * I2C at 1MHz, timing is faster.
 * RX DMA reads last byte it is programmed for.
 *	RX DMA moves last byte to memory
 *	RX DMA signals DONE when memory write complete
 * Simultaneoulsy, the RX DMA read causes
 *	I2C-NL to decrement its read-count
 *	If read count > 1 no interrupt from I2C-NL and it does read ahead
 *	If read count == 0 then I2C-NL Done interrupt occurs
 *
 * For I2C read phase can we get I2C-NL interrupt before DMA interrupt?
 *	Yes, if this Controller issues a NAK on any byte.
 *	Yes, if an I2C bus error or lost arbitration occurs on any byte.
 * What about no I2C errors and last message completes?
 *	Message has I2C_MSG_STOP
 *	I2C-NL HW STOP flag set
 *	I2C-NL read count = exact message length
 *
 * ISR fires CM_DONE due to:
 * HW decrements wrCnt decremented to 0 (START or RPT-START)
 *   ISR sees MPROCEED:MRUN=01b arms DMA channel, re-enables interrupt
 *   exits without giving semaphore
 * HW decrements rdCnt decremented to 0 (last message with STOP)
 *   HW clears MPROCEED:MRUN and begin STOP generation
 *   give semaphore and exit
 * Errors: target NAK, I2C bus error, or I2C lost arbitration.
 *   HW releases bus (NBB->1, IDLE->1) but does not clear fields in cm_cmd register.
 *   give semaphore and exit
 */
static int i2c_xec_nl_msg_rx(const struct device *dev, int async)
{
	struct i2c_xec_nl_data * const data = dev->data;
	struct i2c_context *ctx = &data->ctx;
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	uint32_t cfg_ien = 0, cm_cmd = 0, rdcnt = 0, ev = 0, evmsk = 0;
	int ret = 0;
	uint8_t fmt_addr = ((data->i2c_addr & 0x7fu) << 1) | BIT(0); /* read address */

	if (!i2c_context_msg_on(ctx)) {
		return 0;
	}

#ifdef XEC_I2C_NL_DEBUG
	update_dbg_cmd_idx(data, 0x20, regs->cm_cmd);
#endif
	regs->compl = XEC_I2C_COMPL_STS_RW1C_MSK;
	clr_isr_flags(data);

	rdcnt = ctx->msg_len;
	if (!(ctx->curr_msg->flags & I2C_MSG_STOP)) {
		rdcnt += 3u; /* No stop, do not let HW read count reach < 2 */
	}

	evmsk |= (I2C_XEC_NL_EVENT_I2C_CM_ERR | I2C_XEC_NL_EVENT_DMA_TX_ERR
		  | I2C_XEC_NL_EVENT_DMA_RX_ERR);
	cm_cmd = BIT(XEC_I2C_CM_MRUN_POS) | BIT(XEC_I2C_CM_MPROCEED_POS);
	if (data->prev_dir != data->curr_dir) { /* direction change */
		if (data->prev_dir == I2C_XEC_DIR_NONE) {
			data->ctr_start_req = 1;
		}
		/* configure DMA for TX of rdAddr */
		data->txb[0] = fmt_addr;
		/* NOTE: If debug enabled this helper writes values to I2C-NL debug data */
		i2c_nl_dma_cfg(dev, (uint32_t)data->txb, (uint32_t)&regs->cm_txb,
				1u, MEMORY_TO_PERIPHERAL, i2c_xec_nl_dma_tx_done_cb);

		/* Always use START0, HW FSM will not do STARTN once HW wrCnt
		 * is decremented to 0 one time. Need stateless HW design! */
		cfg_ien |= BIT(XEC_I2C_CFG_CMD_IEN_POS);
		cm_cmd |= BIT(XEC_I2C_CM_START0_POS);
		if (ctx->curr_msg->flags & I2C_MSG_STOP) {
			data->ctr_stop_req = 1;
			cm_cmd |= BIT(XEC_I2C_CM_STOP_POS);
			evmsk |= I2C_XEC_NL_EVENT_I2C_CM_STOP;
			/* evmsk |= I2C_XEC_NL_EVENT_I2C_CM_DONE; */
		} else {
			evmsk |= I2C_XEC_NL_EVENT_DMA_RX_DONE;
		}

		regs->extlen = (uint32_t)rdcnt & 0xff00u;
		cm_cmd |= (((uint32_t)(rdcnt & 0xffu) << XEC_I2C_CM_RCNT_POS)
			   | (1u << XEC_I2C_CM_WCNT_POS));
#ifdef XEC_I2C_NL_DEBUG
		update_dbg_cmd_idx(data, 0x21, cm_cmd);
#endif
	} else {
		data->ctr_start_req = 0;
		/* previous direction is read and no RPT-START */
		dma_reload(cfg->dma_dev, cfg->ctr_dma_chan, (uint32_t)&regs->cm_rxb,
			   (uint32_t)ctx->msg_buf, ctx->msg_len);

		/* This code path is for the second and following read messages.
		 * rdCnt should be 0x0002. Therefore it is safe to change rdCnt
		 * MSB in extlen, change rdCnt LSB in cm_cmd variable, and then
		 * do a 32-bit write of cm_cmd to the CM_CMD register.
		 */
		regs->extlen = rdcnt & 0xff00u;
		cm_cmd = regs->cm_cmd & ~(0xffu << XEC_I2C_CM_RCNT_POS);
		cm_cmd |= ((rdcnt & 0xffu) << XEC_I2C_CM_RCNT_POS);
		cm_cmd |= BIT(XEC_I2C_CM_MRUN_POS) | BIT(XEC_I2C_CM_MPROCEED_POS);
		cfg_ien = BIT(XEC_I2C_CFG_CMD_IEN_POS);
		if (ctx->curr_msg->flags & I2C_MSG_STOP) {
			cm_cmd |= BIT(XEC_I2C_CM_STOP_POS);
			data->ctr_stop_req = 1;
			evmsk |= I2C_XEC_NL_EVENT_I2C_CM_STOP;
			/* evmsk |= I2C_XEC_NL_EVENT_I2C_CM_DONE; */
		} else {
			evmsk |= I2C_XEC_NL_EVENT_DMA_RX_DONE;
		}
#ifdef XEC_I2C_NL_DEBUG
		update_dbg_cmd_idx(data, 0x22, cm_cmd);
#endif
	}

	/* experiment always enable IDLE and add STOP event flag */
	cfg_ien |= BIT(XEC_I2C_CFG_IDLE_IEN_POS);
	evmsk |= I2C_XEC_NL_EVENT_I2C_CM_STOP;
	/* experiment end */

	data->cm_cmd = cm_cmd;
	regs->cm_cmd = cm_cmd;
	dma_start(cfg->dma_dev, cfg->ctr_dma_chan);
	regs->config |= cfg_ien;

#ifdef XEC_I2C_NL_DEBUG
	data->wait_state = 0x23;
#endif
	if (!async) {
#ifdef XEC_I2C_NL_DEBUG
		update_dbg_ev_idx(data, 0x23u, evmsk, 0);
#endif
#ifdef XEC_I2C_NL_DEBUG_POLL_EVENTS
		do {
			ev = k_event_wait(&data->events, evmsk, false, K_NO_WAIT);
		} while (ev == 0);
#else
		ev = k_event_wait(&data->events, evmsk, false, K_FOREVER);
#endif
#ifdef XEC_I2C_NL_DEBUG
		update_dbg_ev_idx(data, 0x24u, evmsk, ev);
#endif
		if (ev & (I2C_XEC_NL_EVENT_I2C_CM_ERR | I2C_XEC_NL_EVENT_DMA_TX_ERR
			  | I2C_XEC_NL_EVENT_DMA_RX_ERR)) {
			return -EIO;
		}
		i2c_context_update(ctx, ctx->msg_len);
	}

	return ret;
}

static int chk_bus_state(const struct device *dev)
{
	struct i2c_xec_nl_data * const data = dev->data;
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	int ret = 0;

	data->i2c_status = regs->ctr_sts;
	if (data->i2c_status & (BIT(XEC_I2C_STS_LAB_POS) | BIT(XEC_I2C_STS_BER_POS))) {
		ret = ctrl_config(dev, cfg->cfg_pin_wait_us);
	}

	if (ret || (data->state == I2C_XEC_STATE_CLOSED)
	    || (regs->ctr_sts & BIT(XEC_I2C_STS_NBB_POS))) {
		data->state = I2C_XEC_STATE_CLOSED;
		data->prev_dir = data->curr_dir = I2C_XEC_DIR_NONE;
#ifdef XEC_I2C_NL_DEBUG
		data->isr_cnt = 0;
#endif
		ret = i2c_nl_check_bus(dev);
		if (ret) {
			k_mutex_unlock(&data->ctr_mutex);
		}
	}

	return ret;
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

static int i2c_xec_nl_chk_msgs(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs)
{
	struct i2c_xec_nl_data * const data = dev->data;
	struct i2c_msg *curr = NULL;
	struct i2c_msg *prev = NULL;
	uint8_t curr_dir= 0, prev_dir = 0;

	if (!msgs || !num_msgs) {
		return -EINVAL;
	}

	prev_dir = data->curr_dir;
	curr = msgs;
	for (uint8_t i = 0; i < num_msgs; i++) {
		if (curr->len && !curr->buf) {
			LOG_ERR("msg[%d] invalid", i);
			return -EINVAL;
		}
		if (curr->len > 0xfff8u) {
			LOG_ERR("msg[%d] length > HW max", i);
			return -EINVAL;
		}
		if (curr->flags & I2C_MSG_READ) {
			curr_dir = I2C_XEC_DIR_RD;
		} else {
			curr_dir = I2C_XEC_DIR_WR;
		}

		/* HW FSM cannot switch from RX back to TX */
		if ((prev_dir == I2C_XEC_DIR_RD) && (curr_dir == I2C_XEC_DIR_WR)) {
			prev->flags |= I2C_MSG_STOP;
			curr->flags |= I2C_MSG_RESTART;
		}

		if ((prev_dir == I2C_XEC_DIR_WR) && (curr_dir == I2C_XEC_DIR_RD)) {
			curr->flags |= I2C_MSG_RESTART;
		}

		prev_dir = curr_dir;
		prev = curr;
		curr++;
	}

	return 0;
}

/* If current direction is read, the Controller will read-ahead two
 * bytes after RX DMA is finished with the previous message. This routine
 * could be called during read-ahead. The controller has NO status indicating
 * read-ahead is in progress or when it ends. Even the read-only FSM registers
 * do not provide any information about read-ahead.
 * Changing the Controller mode command register during read-ahead can results
 * in undefined behavior if the change is not a continuation of reads. For example,
 * changing the register by increasing the read count appears to work.
 * Also, we cannot allow the read count to read 2 or 1 because these values trigger
 * HW FSM to prepare for the last byte. This routine may be called by the application
 * wanting to inject an emergency I2C STOP. A fixed delays for read-ahead is simple
 * but may fail if the target clock stretches.
 * Two bytes are 2 * 9 = 18 clocks. Pad by 2.5 = 36 + 9 = 45. Make it 50 clocks
 * 100 KHz: 500 us
 * 400 KHz: 125 us
 * 1000 KHz: 50 us
 * Also, we must use RX DMA if direction is read because we do not know when the
 * HW is safe to read the I2C-NL controller-mode RX data register. The DMA channel
 * gets a request signal when data is ready.
 */
static int do_i2c_stop(const struct device *dev, int async)
{
	struct i2c_xec_nl_data * const data = dev->data;
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	uint32_t cm_cmd, ev, evmsk;
	int ret = 0;

	/* bus is idle? */
	if (regs->ctr_sts & BIT(XEC_I2C_STS_NBB_POS)) {
		return ret;
	}

	clr_isr_flags(data);

	regs->compl = XEC_I2C_COMPL_STS_RW1C_MSK;
	cm_cmd = regs->cm_cmd;
	if ((cm_cmd & XEC_I2C_CM_RCNT_MSK) == 0) {
		cm_cmd = BIT(XEC_I2C_CM_STOP_POS) | BIT(XEC_I2C_CM_MPROCEED_POS)
			 | BIT(XEC_I2C_CM_MRUN_POS);
		regs->cm_cmd = cm_cmd;

		regs->config |= BIT(XEC_I2C_CFG_CMD_IEN_POS) | BIT(XEC_I2C_CFG_CMD_IEN_POS);
		evmsk = I2C_XEC_NL_EVENT_I2C_CM_STOP | I2C_XEC_NL_EVENT_I2C_CM_ERR;
	} else {
		k_busy_wait(500u);
		i2c_nl_dma_cfg(dev, (uint32_t)&regs->cm_rxb, (uint32_t)data->txb,
			       1u, PERIPHERAL_TO_MEMORY, i2c_xec_nl_dma_rx_done_cb);

		cm_cmd = (1u << XEC_I2C_CM_RCNT_POS) | BIT(XEC_I2C_CM_STOP_POS)
			 | BIT(XEC_I2C_CM_MPROCEED_POS) | BIT(XEC_I2C_CM_MRUN_POS);

		regs->cm_cmd = cm_cmd;

		dma_start(cfg->dma_dev, cfg->ctr_dma_chan);

		evmsk = I2C_XEC_NL_EVENT_DMA_RX_DONE;
		ev = k_event_wait(&data->events, evmsk, false, K_MSEC(100));
		if (ev == 0) {
			return -ETIMEDOUT;
		}

		/* Cause HW FSM to clear STOP, MRUN and MPROCEED HW flags */
		regs->cm_rxb;

		regs->config |= BIT(XEC_I2C_CFG_CMD_IEN_POS);
		evmsk = I2C_XEC_NL_EVENT_I2C_CM_STOP;
	}

	ev = k_event_wait(&data->events, evmsk, false, K_MSEC(100));
	if (ev == 0) {
		ret = -ETIMEDOUT;
	} else if (ev & I2C_XEC_NL_EVENT_I2C_CM_ERR) {
		ret = -EIO;
	}

	return ret;
}

/* Zephyr I2C API i2c_transfer synchronous implemenation
 * Tired of fighting this brain dead hardware. It wants one read.
 * Try this:
 * Process write messages as we are currently doing. It's working.
 * Before starting the transfer:
 * Sum the total read size. This is what will programmed into
 * I2C-NL rdCnt HW 16-bit register.
 * When RX DMA Done fires, have the DMA RX callback re-program
 * the DMA channel with next message. We must store pointer to
 * first read message in I2C-NL data along with a count of read messages.
 * Similar to what we do with async.
 * The first Read will have either START or RPT-START and will generate
 * an I2C-NL interrupt to repogram the DMA channel from TX to RX.
 * Why not do the same for write messages?
 */
static volatile int i2c_xec_nl_xfr_dbg1;
static uint32_t i2c_xec_nl_xfr_dbg1_cnt;
static uint32_t i2c_xec_nl_xfr_dbg_idle_cnt;

static int i2c_xec_nl_transfer(const struct device *dev, struct i2c_msg *msgs,
			       uint8_t num_msgs, uint16_t addr)
{
	struct i2c_xec_nl_data * const data = dev->data;
	struct i2c_context *ctx = &data->ctx;
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	uint32_t ev = 0, evmsk = 0;
	int ret = 0;

#ifdef XEC_I2C_NL_DEBUG
	i2c_xec_nl_xfr_dbg1 = 1;
	init_dbg(data);
#endif

	if (!msgs || !num_msgs) {
		return -EINVAL;
	}

	ret = i2c_xec_nl_chk_addr(dev, addr);
	if (ret) {
		return ret;
	}

	ret = i2c_xec_nl_chk_msgs(dev, msgs, num_msgs);
	if (ret) {
		return ret;
	}

	k_mutex_lock(&data->ctr_mutex, K_FOREVER);
#if 0
	/* PM device runtime: driver indicates it needs the device to be active
	 * PM API will increment the usage count and resume the device if necessary.
	 */
	(void)pm_device_runtime_get(dev);
#endif

	data->cm_active = 1;
	data->mode = XEC_I2C_NL_MODE_SYNC;
	data->i2c_addr = addr;

	ret = chk_bus_state(dev);
	if (ret) {
		data->cm_active = 0;
		return ret;
	}

	i2c_context_msg_setup(ctx, msgs, num_msgs);

	data->state = I2C_XEC_STATE_OPEN;

	while (ctx->msg_count) {
		struct i2c_msg *m = ctx->curr_msg;

		if ((m->len == 0) && (m->flags & I2C_MSG_STOP)) {
			ret = do_i2c_stop(dev, 0);
			data->prev_dir = I2C_XEC_DIR_NONE;
			ctx->msg_count--;
			continue;
		}

		if (m->flags & I2C_MSG_READ) {
			data->curr_dir = I2C_XEC_DIR_RD;
			ret = i2c_xec_nl_msg_rx(dev, 0);
		} else {
			data->curr_dir = I2C_XEC_DIR_WR;
			ret = i2c_xec_nl_msg_tx(dev, 0);
		}

		if (ret) {
			break;
		}

		data->prev_dir = data->curr_dir;
	}

	if (ret) { /* reset and reconfigure I2C hardware */
		/* ISSUE: we can enter this code path if the target unexpectedly NAK'd
		 * Should we always reset on any error? Or if it a simple address NAK return
		 * more quickly?
		 */
		update_dbg_idat_idx(data, 0x70u, regs->ctr_sts, regs->config, regs->compl, regs->cm_cmd);
#if 1
		evmsk = I2C_XEC_NL_EVENT_I2C_CM_STOP;
		ev = k_event_wait(&data->events, evmsk, false, K_MSEC(100));
#endif
		do_dma_stop(dev, I2C_MODE_CTRL);
		ctrl_config(dev, 35); /* !!! milliseconds !!! */
		data->ctr_stop_req = 0;
	} else if (msgs[num_msgs - 1u].flags & I2C_MSG_STOP) {
		update_dbg_idat_idx(data, 0x71u, regs->ctr_sts, regs->config, regs->compl, regs->cm_cmd);
		evmsk = I2C_XEC_NL_EVENT_I2C_CM_ERR | I2C_XEC_NL_EVENT_I2C_CM_STOP;
		ev = k_event_wait(&data->events, evmsk, false, K_MSEC(10));
		if (ev == 0) {
			ret = -ETIMEDOUT;
		} else if (ev & I2C_XEC_NL_EVENT_I2C_CM_ERR) {
			ret = -EIO;
		}
#if 0
		i2c_xec_nl_xfr_dbg_idle_cnt = 0;
		while (!(regs->ctr_sts & BIT((XEC_I2C_STS_NBB_POS)))) {
			i2c_xec_nl_xfr_dbg_idle_cnt++;
		}
#endif
		data->ctr_stop_req = 0;
		data->curr_dir = I2C_XEC_DIR_NONE;
		data->prev_dir = I2C_XEC_DIR_NONE;
		data->state = I2C_XEC_STATE_CLOSED;
		data->cm_active = 0;
	}

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
	int ret = 0;

	if (!msgs || !num_msgs) {
		return -EINVAL;
	}

#ifdef XEC_I2C_NL_DEBUG
	init_dbg(data);
#endif

	ret = i2c_xec_nl_chk_addr(dev, addr);
	if (ret) {
		return ret;
	}

	ret = i2c_xec_nl_build_msgs(dev, msgs, num_msgs);
	if (ret) {
		return ret;
	}

	if (!data->num_wr_msgs && !data->num_rd_msgs) {
		return 0;
	}

	ret = k_mutex_lock(&data->ctr_mutex, K_NO_WAIT);
	if (ret) {
		return -EWOULDBLOCK;
	}

	ret = chk_closed(dev);
	if (ret) {
		data->cm_active = 0;
		return ret;
	}

	data->cm_active = 1;
	data->mode = XEC_I2C_NL_MODE_ASYNC;
	data->state = I2C_XEC_STATE_OPEN;
	data->addr = addr;
	data->cb = cb;
	data->user_data = userdata;

	/* HW can do TX to RX but not RX to TX. Always check TX first */
	if (data->num_wr_msgs) {
		data->curr_dir = I2C_XEC_DIR_WR;
		ret = i2c_xec_nl_msg_tx(dev, data->wr_msg, addr, 1);
	} else if (data->num_rd_msgs) {
		data->curr_dir = I2C_XEC_DIR_RD;
		ret = i2c_xec_nl_msg_rx(dev, data->rd_msg, addr, 1);
	}

	if (ret) {
		do_dma_stop(dev, I2C_MODE_CTRL); /* force DMA to end */
		/* error: reset and reconfigure */
		ctrl_config(dev);
		data->cm_active = 0;
		k_mutex_unlock(&data->ctr_mutex);
	}

	return ret;
}
#endif /* CONFIG_I2C_CALLBACK */

#ifdef CONFIG_I2C_TARGET
/* TODO - Idea for target mode processing using Network Layer that matches Zephyr I2C target
 * mode callbacks.  May not be possible.
 * TM_CMD register
 *   b[23:16] = tm_read_count: Decremented each time a byte is copied from the I2C.Data register to the
 *              TM_RX_Data register. When tm_read_count is decremented to 0, HW will NAK any additional
 *              received data.
 *   b[15:8] = tm_write_count: Set to number of bytes expected to be sent to external Controller.
 *             If > 0 when external Controller requests data and TM_TX_Data register is empty,
 *             HW will clock stretch until TM_TX_Data is not empty.
 *             Field is decremented when TM_TX_Data is not empty and HW copies byte to I2C.Data.
 *             If tm_write_count == 0 when external Controller requests data, this target HW FSM
 *             resends content of TM_TX_Data register. If this occurs HW FSM sets the SPROT bit
 *             in the completion register.
 *             I2C.Compl.SPROT=1 indicates tm_write_count reached 0 before the external Controller
 *             sent a NAK or this Controller received NAK before tm_write_count reached 0.
 * i2c_target_write_requested_cb_t - Driver invokes callback on reception of START and matching address.
 *   application returns  0 - Driver should ACK the next byte (data)
 *                      < 0 - Driver should NAK the next byte (data)
 *
 * i2c_target_write_received_cb_t - Driver invokes callback on reception of a data byte
 *   application returns  0 - Driver should ACK the next byte (data)
 *                        1 - Driver should NAK the next byte (data)
 *
 *
 */
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
	struct i2c_context *ctx = &data->ctx;
	struct xec_i2c_nl_regs * const regs = cfg->regs;
	uint32_t cfg_ien, cm_cmd, compl, i2c_config, events;

#ifdef XEC_I2C_NL_DEBUG
	sys_write32(0x00240u, I2C_NL_GPIO_0130_CTRL_ADDR);
	data->isr_cnt++;
	data->i2c_fsm = regs->fsm;
	data->smb_fsm = regs->fsm_nl;
#endif

	i2c_config = regs->config;
	data->i2c_config = i2c_config;
	regs->config &= ~(BIT(XEC_I2C_CFG_CMD_IEN_POS) | BIT(XEC_I2C_CFG_IDLE_IEN_POS));

	cm_cmd = regs->cm_cmd;
	compl = regs->compl;
	data->i2c_compl = compl;
	data->i2c_status = regs->ctr_sts;
	data->i2c_cm_cmd = cm_cmd;

	regs->compl = compl;
	mchp_xec_ecia_girq_src_clr(cfg->girq, cfg->girq_pos);

	update_dbg_idat_idx(data, data->isr_cnt + 0xA0u, data->i2c_status, i2c_config, compl, cm_cmd);

	/* EXPERIMENT: we enabled IDLE interrupt before beginning any operation
	 * This will cause IDLE interrupt before first START.
	 * We need to handle this.
	 * !!!PROBLEM!!!
	 * If target NAK's address I2C-NL FSM will not clear START0 bit. We get infinite
	 * interrupt loop.
	 */
	if (data->ctr_start_req && (i2c_config & BIT(XEC_I2C_CFG_IDLE_IEN_POS))
	    && (compl & BIT(XEC_I2C_COMPL_IDLE_STS_POS))) {
		data->ctr_start_req = 0;
		regs->config |= (BIT(XEC_I2C_CFG_CMD_IEN_POS) | BIT(XEC_I2C_CFG_IDLE_IEN_POS));
		sys_write32(0x10240u, I2C_NL_GPIO_0130_CTRL_ADDR);
		return;
	}

#ifdef CONFIG_I2C_TARGET
	if ((i2c_config & BIT(XEC_I2C_CFG_AAT_IEN_POS))
	    && (data->i2c_status & BIT(XEC_I2C_STS_AAT_POS))) {
		i2c_xec_nl_target_handler(dev);
		return;
	}
#endif

	events = 0;
	cfg_ien = 0;

	if (compl & XEC_I2C_COMPL_CM_ERR) {
		events |= I2C_XEC_NL_EVENT_I2C_CM_ERR;
		regs->config |= BIT(XEC_I2C_CFG_IDLE_IEN_POS);
#ifdef XEC_I2C_NL_DEBUG
		update_dbg_ev_idx(data, 0x81u, 0, events);
		update_dbg_idat_idx(data, 0x81u, data->i2c_status, i2c_config, compl, cm_cmd);
#endif
		goto isr_exit_post;
	}

	if (compl & BIT(XEC_I2C_COMPL_IDLE_STS_POS)) {
		data->ctr_stop_req = 0;
		events |= I2C_XEC_NL_EVENT_I2C_CM_STOP;
#ifdef XEC_I2C_NL_DEBUG
		update_dbg_ev_idx(data, 0x82u, 0, events);
		update_dbg_idat_idx(data, 0x82u, data->i2c_status, i2c_config, compl, cm_cmd);
#endif
		goto isr_exit_post;
	}

	data->i2c_ev |= events;

	/* Did HW FSM clear MPROCEED for DMA direction change from TX to RX */
	if ((cm_cmd & (BIT(XEC_I2C_CM_MRUN_POS) | BIT(XEC_I2C_CM_MPROCEED_POS)))
	    == BIT(XEC_I2C_CM_MRUN_POS)) {
#ifdef XEC_I2C_NL_DEBUG /* TODO use changed register values */
		update_dbg_ev_idx(data, 0x88u, 0, 0x1000u);
		update_dbg_idat_idx(data, 0x88u, data->i2c_status, i2c_config, compl, cm_cmd);
#endif
		/* Configure and start RX DMA before setting MPROCEED back to 1 */
		i2c_nl_dma_cfg(dev, (uint32_t)&regs->cm_rxb, (uint32_t)ctx->msg_buf,
			       ctx->msg_len, PERIPHERAL_TO_MEMORY, i2c_xec_nl_dma_rx_done_cb);
		regs->cm_cmd |= (BIT(XEC_I2C_CM_MPROCEED_POS) | BIT(XEC_I2C_CM_MRUN_POS));
		dma_start(cfg->dma_dev, cfg->ctr_dma_chan);
		regs->config |= (cfg_ien | BIT(XEC_I2C_CFG_CMD_IEN_POS));
		sys_write32(0x10240u, I2C_NL_GPIO_0130_CTRL_ADDR);
		return;
	}

	if (data->ctr_stop_req) {
		cfg_ien |= BIT(XEC_I2C_CFG_IDLE_IEN_POS);
	}

	/* CM write count or read count reached 0 without error or STOP */
	events |= I2C_XEC_NL_EVENT_I2C_CM_DONE;
	if (cfg_ien) {
		regs->config |= cfg_ien;
	}
isr_exit_post:
	data->i2c_ev |= events;
	k_event_post(&data->events, events);
#ifdef XEC_I2C_NL_DEBUG
	update_dbg_ev_idx(data, 0x8Fu, 0, events);
	update_dbg_idat_idx(data, 0x8Fu, regs->ctr_sts, regs->config, regs->compl, regs->cm_cmd);
	sys_write32(0x10240u, I2C_NL_GPIO_0130_CTRL_ADDR);
#endif
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
static int i2c_xec_nl_pm_action(const struct device *dev,
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

static int i2c_xec_nl_init(const struct device *dev)
{
	const struct i2c_xec_nl_config * const cfg = dev->config;
	struct i2c_xec_nl_data * const data = dev->data;
	int ret = 0;

	data->state = I2C_XEC_STATE_CLOSED;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("XEC I2C-NL pinctrl setup failed (%d)", ret);
		return ret;
	}

	k_mutex_init(&data->ctr_mutex);
	k_event_init(&data->events);

	if (!device_is_ready(cfg->dma_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clk_dev, (clock_control_subsys_t)&cfg->clksrc);
	if (ret) {
		LOG_ERR("XEC I2C-NL clock device failure");
		return ret;
	}

	ret = i2c_xec_nl_config(dev, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD),
				cfg->init_pin_wait_us);
	if (ret) {
		LOG_ERR("XEC I2C-NL configuration failed (%d)", ret);
		return ret;
	}

#ifdef CONFIG_PM_DEVICE_RUNTIME
	pm_device_init_suspended(dev);
	pm_device_runtime_enable(dev);
#else
	/* TODO nrfx_twim_enable(&dev_config->twim); */
#endif

	if (cfg->irq_config_func) {
		cfg->irq_config_func();
	}
	mchp_xec_ecia_girq_src_en(cfg->girq, cfg->girq_pos);

	return 0;
}

#define XEC_I2C_PCR_INFO(i)						\
	MCHP_XEC_PCR_SCR_ENCODE(DT_INST_CLOCKS_CELL(i, regidx),		\
				DT_INST_CLOCKS_CELL(i, bitpos),		\
				DT_INST_CLOCKS_CELL(0, domain))


#define I2C_XEC_NL_DT_INST_GET_BY_IDX_OR(inst, prop, idx, default_value)	\
	COND_CODE_1(DT_PROP_HAS_IDX(DT_DRV_INST(inst), prop, idx),		\
		    (DT_PROP_BY_IDX(DT_DRV_INST(inst), prop, idx)),		\
		    (default_value))

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
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),	\
		.clksrc = { .pcr_info = XEC_I2C_PCR_INFO(n), },		\
		.port_sel = DT_INST_PROP(n, port_sel),			\
		.girq = DT_INST_PROP_BY_IDX(n, girqs, 0),		\
		.girq_pos = DT_INST_PROP_BY_IDX(n, girqs, 1),		\
		.ctr_dma_chan = MCHP_XEC_DT_INST_DMA_CHANNEL(n, ctr),	\
		.ctr_dma_id = MCHP_XEC_DT_INST_DMA_TRIGSRC(n, ctr),	\
		.irq_config_func = i2c_nl_xec_irq_config_func_##n,	\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		.dev_dma_chan = MCHP_XEC_DT_INST_DMA_CHANNEL(n, dev),	\
		.dev_dma_id = MCHP_XEC_DT_INST_DMA_TRIGSRC(n, dev),	\
		.dma_dev = DEVICE_DT_GET(MCHP_XEC_DT_INST_DMA_CTLR(n, ctr)), \
		.scl_gpio = GPIO_DT_SPEC_INST_GET(n, scl_gpios),	\
		.sda_gpio = GPIO_DT_SPEC_INST_GET(n, sda_gpios),	\
		.init_pin_wait_us = DT_INST_PROP_OR(n, init_pin_wait, 100),	\
		.cfg_pin_wait_us = DT_INST_PROP_OR(n, config_pin_wait, 35000),	\
		.target_addr1 = I2C_XEC_NL_DT_INST_GET_BY_IDX_OR(n, target_addrs, 0, 0), \
		.target_addr2 = I2C_XEC_NL_DT_INST_GET_BY_IDX_OR(n, target_addrs, 1, 0), \
	};								\
	PM_DEVICE_DT_INST_DEFINE(n, i2c_xec_nl_pm_action);		\
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_xec_nl_init, NULL,		\
		&i2c_xec_nl_data_##n, &i2c_xec_nl_config_##n,		\
		POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,			\
		&i2c_xec_nl_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_NL_XEC_DEVICE)
