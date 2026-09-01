/*
 * Copyright (c) 2019 Intel Corporation
 * Copyright (c) 2021 Microchip Technology Inc.
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Microchip XEC eSPI controller driver, V3.
 *
 * Derived from espi_mchp_xec_v2.c. The difference is that the peripheral
 * channel logical devices (KBC, ACPI EC, EMI, mailbox, Port80/92, ...) are not
 * implemented here. They are separate Zephyr drivers bound to the child nodes
 * of this controller, living in drivers/espi/xec_pc/.
 *
 * This driver keeps ownership of what only it can own: the Host facing I/O
 * BARs, Memory BARs and Serial-IRQ registers, which MEC hardware places in the
 * eSPI controller register space and holds in reset while eSPI Reset or PLTRST
 * is asserted. Peripheral channel drivers subscribe to those transitions with
 * mchp_xec_espi_pc_register().
 */

#define DT_DRV_COMPAT microchip_xec_espi_v3

#include <soc.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/clock_control/mchp_xec_clock_control.h>
#include <zephyr/drivers/interrupt_controller/intc_mchp_xec_ecia.h>
#include <zephyr/dt-bindings/espi/mchp-xec-espi-v3.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/espi/mchp_xec_espi.h>
#include "espi_utils.h"
#include "espi_mchp_xec_v3.h"

LOG_MODULE_REGISTER(espi, CONFIG_ESPI_LOG_LEVEL);

/* Minimum delay before acknowledging a virtual wire */
#define ESPI_XEC_VWIRE_ACK_DELAY 10ul

/* Maximum timeout to transmit a virtual wire packet.
 * 1 ms expressed in multiples of 1us
 */
#define ESPI_XEC_VWIRE_SEND_TIMEOUT 1000ul

#define VW_MAX_GIRQS 2ul

/* 200ms */
#define MAX_OOB_TIMEOUT   200ul
/* 1s */
#define MAX_FLASH_TIMEOUT 1000ul

/* While issuing flash erase command, it should be ensured that the transfer
 * length specified is non-zero.
 */
#define ESPI_FLASH_ERASE_DUMMY 0x01ul

/* OOB maximum address configuration */
#define ESPI_XEC_OOB_ADDR_MSW 0x1ffful
#define ESPI_XEC_OOB_ADDR_LSW 0xfffful

/* OOB Rx length */
#define ESPI_XEC_OOB_RX_LEN 0x7f00ul

/* Espi peripheral has 3 uart ports */
#define ESPI_PERIPHERAL_UART_PORT0 0
#define ESPI_PERIPHERAL_UART_PORT1 1

#define UART_DEFAULT_IRQ_POS 2u
#define UART_DEFAULT_IRQ     BIT(UART_DEFAULT_IRQ_POS)

#define ESPI_XEC_SMVW_REG_OFS 0x200

/* PCR */
#define XEC_PCR_REG_BASE DT_REG_ADDR(DT_NODELABEL(pcr))

/* Microchip canonical virtual wire mapping
 * ------------------------------------------------------------------------|
 * VW Idx | VW reg | SRC_ID3      | SRC_ID2      | SRC_ID1   | SRC_ID0     |
 * ------------------------------------------------------------------------|
 * System Event Virtual Wires
 * ------------------------------------------------------------------------|
 *  2h    | MSVW00 | res          | SLP_S5#      | SLP_S4#   | SLP_S3#     |
 *  3h    | MSVW01 | res          | OOB_RST_WARN | PLTRST#   | SUS_STAT#   |
 *  4h    | SMVW00 | PME#         | WAKE#        | res       | OOB_RST_ACK |
 *  5h    | SMVW01 | SLV_BOOT_STS | ERR_NONFATAL | ERR_FATAL | SLV_BT_DONE |
 *  6h    | SMVW02 | HOST_RST_ACK | RCIN#        | SMI#      | SCI#        |
 *  7h    | MSVW02 | res          | NMIOUT#      | SMIOUT#   | HOS_RST_WARN|
 * ------------------------------------------------------------------------|
 * Platform specific virtual wires
 * ------------------------------------------------------------------------|
 *  40h   | SMVW03 | res          | res          | DNX_ACK   | SUS_ACK#    |
 *  41h   | MSVW03 | SLP_A#       | res          | SUS_PDNACK| SUS_WARN#   |
 *  42h   | MSVW04 | res          | res          | SLP_WLAN# | SLP_LAN#    |
 *  43h   | MSVW05 | generic      | generic      | generic   | generic     |
 *  44h   | MSVW06 | generic      | generic      | generic   | generic     |
 *  45h   | SMVW04 | generic      | generic      | generic   | generic     |
 *  46h   | SMVW05 | generic      | generic      | generic   | generic     |
 *  47h   | MSVW07 | res          | res          | res       | HOST_C10    |
 *  4Ah   | MSVW08 | res          | res          | DNX_WARN  | res         |
 * These are configurable by overriding device tree vw routing             |
 *  50h   | SMVW06 | ocb_3        | ocb_2        | ocb_1     | ocb_0       |
 *  51h   | SMVW07 | gpio_7       | gpio_6       | gpio_5    | gpio_4      |
 *  52h   | SMVW08 | gpio_11      | gpio_10      | gpio_9    | gpio_8      |
 */
static const struct xec_signal vw_tbl[] = {
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_SLP_S3, vw_slp_s3_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_SLP_S4, vw_slp_s4_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_SLP_S5, vw_slp_s5_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_OOB_RST_WARN, vw_oob_rst_warn),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_PLTRST, vw_pltrst_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_SUS_STAT, vw_sus_stat_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_HOST_RST_WARN, vw_host_rst_warn),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_NMIOUT, vw_nmiout_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_SMIOUT, vw_smiout_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_SLP_A, vw_slp_a_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_SUS_PWRDN_ACK, vw_sus_pwrdn_ack),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_SUS_WARN, vw_sus_warn_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_SLP_WLAN, vw_slp_wlan_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_SLP_LAN, vw_slp_lan_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_HOST_C10, vw_host_c10),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_DNX_WARN, vw_dnx_warn),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_PME, vw_pme_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_WAKE, vw_wake_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_OOB_RST_ACK, vw_oob_rst_ack),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_TARGET_BOOT_STS, vw_target_boot_status),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_ERR_NON_FATAL, vw_error_non_fatal),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_ERR_FATAL, vw_error_fatal),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_TARGET_BOOT_DONE, vw_target_boot_done),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_HOST_RST_ACK, vw_host_rst_ack),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_RST_CPU_INIT, vw_rcin_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_SMI, vw_smi_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_SCI, vw_sci_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_DNX_ACK, vw_dnx_ack),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_SUS_ACK, vw_sus_ack_n),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_TARGET_GPIO_0, vw_t2c_gpio_0),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_TARGET_GPIO_1, vw_t2c_gpio_1),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_TARGET_GPIO_2, vw_t2c_gpio_2),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_TARGET_GPIO_3, vw_t2c_gpio_3),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_TARGET_GPIO_4, vw_t2c_gpio_4),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_TARGET_GPIO_5, vw_t2c_gpio_5),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_TARGET_GPIO_6, vw_t2c_gpio_6),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_TARGET_GPIO_7, vw_t2c_gpio_7),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_TARGET_GPIO_8, vw_t2c_gpio_8),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_TARGET_GPIO_9, vw_t2c_gpio_9),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_TARGET_GPIO_10, vw_t2c_gpio_10),
	MCHP_DT_ESPI_VW_ENTRY(ESPI_VWIRE_SIGNAL_TARGET_GPIO_11, vw_t2c_gpio_11),
};

/* Buffer size are expressed in bytes */
#ifdef CONFIG_ESPI_OOB_CHANNEL
static uint32_t target_rx_mem[CONFIG_ESPI_OOB_BUFFER_SIZE >> 2];
static uint32_t target_tx_mem[CONFIG_ESPI_OOB_BUFFER_SIZE >> 2];
#endif
#ifdef CONFIG_ESPI_FLASH_CHANNEL
static uint32_t target_mem[CONFIG_ESPI_FLASH_BUFFER_SIZE >> 2];
#endif

static inline uintptr_t xec_msvw_addr(const struct device *dev, uint8_t vw_index)
{
	const struct espi_xec_config *devcfg = dev->config;
	uintptr_t vwbase = (uintptr_t)devcfg->vw_base_addr;

	return vwbase + vw_index * sizeof(struct espi_msvw_reg);
}

static inline uintptr_t xec_smvw_addr(const struct device *dev, uint8_t vw_index)
{
	const struct espi_xec_config *devcfg = dev->config;
	uintptr_t vwbase = (uintptr_t)devcfg->vw_base_addr;

	vwbase += ESPI_XEC_SMVW_REG_OFS;
	return vwbase + vw_index * sizeof(struct espi_smvw_reg);
}

static int espi_xec_configure(const struct device *dev, struct espi_cfg *cfg)
{
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_cap_regs *cap_regs =
		(struct xec_espi_cap_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_CAP_OFS);
	struct xec_espi_ioc_cfg_regs *cfg_regs =
		(struct xec_espi_ioc_cfg_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_CFG_OFS);
	uint8_t iomode = 0;
	uint8_t cap0 = cap_regs->CAP0;
	uint8_t cap1 = cap_regs->CAP1;
	uint8_t cur_iomode =
		(cap1 & MCHP_ESPI_GBL_CAP1_IO_MODE_MASK) >> MCHP_ESPI_GBL_CAP1_IO_MODE_POS;

	/* Set frequency */
	cap1 &= ~MCHP_ESPI_GBL_CAP1_MAX_FREQ_MASK;

	switch (cfg->max_freq) {
	case 20:
		cap1 |= MCHP_ESPI_GBL_CAP1_MAX_FREQ_20M;
		break;
	case 25:
		cap1 |= MCHP_ESPI_GBL_CAP1_MAX_FREQ_25M;
		break;
	case 33:
		cap1 |= MCHP_ESPI_GBL_CAP1_MAX_FREQ_33M;
		break;
	case 50:
		cap1 |= MCHP_ESPI_GBL_CAP1_MAX_FREQ_50M;
		break;
	case 66:
		cap1 |= MCHP_ESPI_GBL_CAP1_MAX_FREQ_66M;
		break;
	default:
		return -EINVAL;
	}

	/* Set IO mode */
	iomode = (cfg->io_caps >> 1);
	if (iomode > 3) {
		return -EINVAL;
	}

	if (iomode != cur_iomode) {
		cap1 &= ~(MCHP_ESPI_GBL_CAP1_IO_MODE_MASK0 << MCHP_ESPI_GBL_CAP1_IO_MODE_POS);
		cap1 |= (iomode << MCHP_ESPI_GBL_CAP1_IO_MODE_POS);
	}

	/* Validate and translate eSPI API channels to MEC capabilities */
	cap0 &= ~MCHP_ESPI_GBL_CAP0_MASK;
	if (cfg->channel_caps & ESPI_CHANNEL_PERIPHERAL) {
		if (IS_ENABLED(CONFIG_ESPI_PERIPHERAL_CHANNEL)) {
			cap0 |= MCHP_ESPI_GBL_CAP0_PC_SUPP;
		} else {
			return -EINVAL;
		}
	}

	if (cfg->channel_caps & ESPI_CHANNEL_VWIRE) {
		if (IS_ENABLED(CONFIG_ESPI_VWIRE_CHANNEL)) {
			cap0 |= MCHP_ESPI_GBL_CAP0_VW_SUPP;
		} else {
			return -EINVAL;
		}
	}

	if (cfg->channel_caps & ESPI_CHANNEL_OOB) {
		if (IS_ENABLED(CONFIG_ESPI_OOB_CHANNEL)) {
			cap0 |= MCHP_ESPI_GBL_CAP0_OOB_SUPP;
		} else {
			return -EINVAL;
		}
	}

	if (cfg->channel_caps & ESPI_CHANNEL_FLASH) {
		if (IS_ENABLED(CONFIG_ESPI_FLASH_CHANNEL)) {
			cap0 |= MCHP_ESPI_GBL_CAP0_FC_SUPP;
		} else {
			LOG_ERR("Flash channel not supported");
			return -EINVAL;
		}
	}

	cap_regs->CAP0 = cap0;
	cap_regs->CAP1 = cap1;

	/* Activate the eSPI block *.
	 * Need to guarantee that this register is configured before RSMRST#
	 * de-assertion and after pinmux
	 */
	cfg_regs->ACTV = 1;
	LOG_DBG("eSPI block activated successfully");

	return 0;
}

static bool espi_xec_channel_ready(const struct device *dev, enum espi_channel ch)
{
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_cap_regs *cap_regs =
		(struct xec_espi_cap_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_CAP_OFS);
	bool sts = false;

	switch (ch) {
	case ESPI_CHANNEL_PERIPHERAL:
		sts = cap_regs->PCRDY & MCHP_ESPI_PC_READY;
		break;
	case ESPI_CHANNEL_VWIRE:
		sts = cap_regs->VWRDY & MCHP_ESPI_VW_READY;
		break;
	case ESPI_CHANNEL_OOB:
		sts = cap_regs->OOBRDY & MCHP_ESPI_OOB_READY;
		break;
	case ESPI_CHANNEL_FLASH:
		sts = cap_regs->FCRDY & MCHP_ESPI_FC_READY;
		break;
	default:
		sts = false;
		break;
	}

	return sts;
}

/* Set state of VWire in VWire group
 * eSPI groups virtual wires into groups of 4. XEC hardware implements an 80-bit register
 * for each Controller-to-Target group and a 64-bit register for each Target-to-Controller group.
 * Updating VWire state(s) to new values results in XEC eSPI hardware actions:
 * VWire changes(s)
 *   If Target-to-Host VWire changes(s):
 *     XEC HW sets the read-only change bit in the VW group register for each changed VWire.
 *   endif
 *   XEC HW updates the VWires Available status bit in the 16-bit eSPI status word and if enabled,
 *   asserts the configured ESPI_nALERT signal (in-band or pin).
 *
 * The Host responds to ESPI_nALERT assertion by issueing GET_STATUS to read the 16-bit ESPI status
 * word. If the Host sees VWires Available status is set it issues GET_VW with the number of VWire
 * groups it has configured when eSPI was initialized. When GET_VW reads any XEC
 * Target-to-Controller group, XEC HW clears the read-only changed bit(s).
 */
static int espi_xec_send_vwire(const struct device *dev, enum espi_vwire_signal signal,
			       uint8_t level)
{
	struct xec_signal signal_info = vw_tbl[signal];
	uint8_t xec_id = signal_info.xec_reg_idx;
	uint8_t src_id = signal_info.bit;
	uint8_t dir = 0;
	uintptr_t regaddr = 0;

	if ((src_id >= ESPI_VWIRE_SRC_ID_MAX) || (xec_id >= ESPI_MSVW_IDX_MAX)) {
		return -EINVAL;
	}

	if (!(signal_info.flags & BIT(MCHP_DT_ESPI_VW_FLAG_STATUS_POS))) {
		return -EIO; /* VW not enabled */
	}

	dir = (signal_info.flags >> MCHP_DT_ESPI_VW_FLAG_DIR_POS) & BIT(0);

	if (dir == ESPI_CONTROLLER_TO_TARGET) {
		regaddr = xec_msvw_addr(dev, xec_id);
		sys_write8(level, regaddr + MSVW_BI_SRC0 + src_id);
	} else {
		regaddr = xec_smvw_addr(dev, xec_id);
		sys_write8(level, regaddr + SMVW_BI_SRC0 + src_id);

		/* Ensure eSPI virtual wire packet is transmitted
		 * There is no interrupt, so need to poll register
		 */
		uint16_t rd_cnt = ESPI_XEC_VWIRE_SEND_TIMEOUT;

		while (sys_read8(regaddr + SMVW_BI_SRC_CHG) && rd_cnt--) {
			k_busy_wait(1);
		}

		if (rd_cnt == 0) {
			LOG_ERR("VW %d send timeout", signal);
			return -ETIMEDOUT;
		}
		LOG_DBG("XEC T2C VWire %u read by Host", signal);
	}

	return 0;
}

static int espi_xec_receive_vwire(const struct device *dev, enum espi_vwire_signal signal,
				  uint8_t *level)
{
	struct xec_signal signal_info = vw_tbl[signal];
	uint8_t xec_id = signal_info.xec_reg_idx;
	uint8_t src_id = signal_info.bit;
	uint8_t dir;
	uintptr_t regaddr;

	if ((src_id >= ESPI_VWIRE_SRC_ID_MAX) || (xec_id >= ESPI_SMVW_IDX_MAX) || (level == NULL)) {
		return -EINVAL;
	}

	if (!(signal_info.flags & BIT(MCHP_DT_ESPI_VW_FLAG_STATUS_POS))) {
		return -EIO; /* VW not enabled */
	}

	dir = (signal_info.flags >> MCHP_DT_ESPI_VW_FLAG_DIR_POS) & BIT(0);

	if (dir == ESPI_CONTROLLER_TO_TARGET) {
		regaddr = xec_msvw_addr(dev, xec_id);
		*level = sys_read8(regaddr + MSVW_BI_SRC0 + src_id) & BIT(0);
	}

	if (dir == ESPI_TARGET_TO_CONTROLLER) {
		regaddr = xec_smvw_addr(dev, xec_id);
		*level = sys_read8(regaddr + SMVW_BI_SRC0 + src_id) & BIT(0);
	}

	return 0;
}

#ifdef CONFIG_ESPI_OOB_CHANNEL
static int espi_xec_send_oob(const struct device *dev, struct espi_oob_packet *pckt)
{
	struct espi_xec_data *const data = dev->data;
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_ioc_oob_regs *regs =
		(struct xec_espi_ioc_oob_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_OOB_OFS);
	uint8_t err_mask = (MCHP_ESPI_OOB_TX_STS_IBERR | MCHP_ESPI_OOB_TX_STS_OVRUN |
			    MCHP_ESPI_OOB_TX_STS_BADREQ);
	int ret = 0;

	LOG_DBG("%s", __func__);

	if (!(regs->OOBTXSTS & MCHP_ESPI_OOB_TX_STS_CHEN)) {
		LOG_ERR("OOB channel is disabled");
		return -EIO;
	}

	if (regs->OOBTXSTS & MCHP_ESPI_OOB_TX_STS_BUSY) {
		LOG_ERR("OOB channel is busy");
		return -EBUSY;
	}

	if (pckt->len > CONFIG_ESPI_OOB_BUFFER_SIZE) {
		LOG_ERR("insufficient space");
		return -EINVAL;
	}

	memcpy(target_tx_mem, pckt->buf, pckt->len);

	regs->OOBTXL = pckt->len;
	regs->OOBTXC = MCHP_ESPI_OOB_TX_CTRL_START;
	LOG_DBG("%s %d", __func__, regs->OOBTXL);

	/* Wait until ISR or timeout */
	ret = k_sem_take(&data->tx_lock, K_MSEC(MAX_OOB_TIMEOUT));
	if (ret == -EAGAIN) {
		return -ETIMEDOUT;
	}

	if (regs->OOBTXSTS & err_mask) {
		LOG_ERR("Tx failed %x", regs->OOBTXSTS);
		regs->OOBTXSTS = err_mask;
		return -EIO;
	}

	return 0;
}

static int espi_xec_receive_oob(const struct device *dev, struct espi_oob_packet *pckt)
{
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_ioc_oob_regs *regs =
		(struct xec_espi_ioc_oob_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_OOB_OFS);
	uint8_t err_mask = MCHP_ESPI_OOB_RX_STS_IBERR | MCHP_ESPI_OOB_RX_STS_OVRUN;

	if (regs->OOBRXSTS & err_mask) {
		return -EIO;
	}

#ifndef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
	int ret;
	struct espi_xec_data *data = (struct espi_xec_data *)(dev->data);

	/* Wait until ISR or timeout */
	ret = k_sem_take(&data->rx_lock, K_MSEC(MAX_OOB_TIMEOUT));
	if (ret == -EAGAIN) {
		return -ETIMEDOUT;
	}
#endif
	/* Check if buffer passed to driver can fit the received buffer */
	uint32_t rcvd_len = regs->OOBRXL & MCHP_ESPI_OOB_RX_LEN_MASK;

	if (rcvd_len > pckt->len) {
		LOG_ERR("space rcvd %d vs %d", rcvd_len, pckt->len);
		return -EIO;
	}

	pckt->len = rcvd_len;
	memcpy(pckt->buf, target_rx_mem, pckt->len);
	memset(target_rx_mem, 0, pckt->len);

	/* Only after data has been copied from SRAM, indicate channel
	 * is available for next packet
	 */
	regs->OOBRXC |= MCHP_ESPI_OOB_RX_CTRL_AVAIL;

	return 0;
}
#endif /* CONFIG_ESPI_OOB_CHANNEL */

#ifdef CONFIG_ESPI_FLASH_CHANNEL
static int espi_xec_flash_read(const struct device *dev, struct espi_flash_packet *pckt)
{
	struct espi_xec_data *data = (struct espi_xec_data *)dev->data;
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_ioc_fc_regs *regs =
		(struct xec_espi_ioc_fc_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_FC_OFS);
	uint32_t err_mask = (MCHP_ESPI_FC_STS_IBERR | MCHP_ESPI_FC_STS_FAIL |
			     MCHP_ESPI_FC_STS_OVFL | MCHP_ESPI_FC_STS_BADREQ);
	int ret = 0;

	LOG_DBG("%s", __func__);

	if (!(regs->FCSTS & MCHP_ESPI_FC_STS_CHAN_EN)) {
		LOG_ERR("Flash channel is disabled");
		return -EIO;
	}

	if (pckt->len > CONFIG_ESPI_FLASH_BUFFER_SIZE) {
		LOG_ERR("Invalid size request");
		return -EINVAL;
	}

	regs->FCFA[1] = 0;
	regs->FCFA[0] = pckt->flash_addr;
	regs->FCBA[1] = 0;
	regs->FCBA[0] = (uint32_t)&target_mem[0];
	regs->FCLEN = pckt->len;
	regs->FCCTL = MCHP_ESPI_FC_CTRL_FUNC_SET(MCHP_ESPI_FC_CTRL_RD0);
	regs->FCCTL |= MCHP_ESPI_FC_CTRL_START;

	/* Wait until ISR or timeout */
	ret = k_sem_take(&data->flash_lock, K_MSEC(MAX_FLASH_TIMEOUT));
	if (ret == -EAGAIN) {
		LOG_ERR("%s timeout", __func__);
		return -ETIMEDOUT;
	}

	if (regs->FCSTS & err_mask) {
		LOG_ERR("%s error %x", __func__, err_mask);
		regs->FCSTS = err_mask;
		return -EIO;
	}

	memcpy(pckt->buf, target_mem, pckt->len);

	return 0;
}

static int espi_xec_flash_write(const struct device *dev, struct espi_flash_packet *pckt)
{
	struct espi_xec_data *data = (struct espi_xec_data *)dev->data;
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_ioc_fc_regs *regs =
		(struct xec_espi_ioc_fc_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_FC_OFS);
	uint32_t err_mask = MCHP_ESPI_FC_STS_IBERR | MCHP_ESPI_FC_STS_OVRUN |
			    MCHP_ESPI_FC_STS_FAIL | MCHP_ESPI_FC_STS_BADREQ;
	int ret = 0;

	LOG_DBG("%s", __func__);

	if (sizeof(target_mem) < pckt->len) {
		LOG_ERR("Packet length is too big");
		return -ENOMEM;
	}

	if (!(regs->FCSTS & MCHP_ESPI_FC_STS_CHAN_EN)) {
		LOG_ERR("Flash channel is disabled");
		return -EIO;
	}

	if ((regs->FCCFG & MCHP_ESPI_FC_CFG_BUSY)) {
		LOG_ERR("Flash channel is busy");
		return -EBUSY;
	}

	memcpy(target_mem, pckt->buf, pckt->len);

	regs->FCFA[1] = 0;
	regs->FCFA[0] = pckt->flash_addr;
	regs->FCBA[1] = 0;
	regs->FCBA[0] = (uint32_t)&target_mem[0];
	regs->FCLEN = pckt->len;
	regs->FCCTL = MCHP_ESPI_FC_CTRL_FUNC_SET(MCHP_ESPI_FC_CTRL_WR0);
	regs->FCCTL |= MCHP_ESPI_FC_CTRL_START;

	/* Wait until ISR or timeout */
	ret = k_sem_take(&data->flash_lock, K_MSEC(MAX_FLASH_TIMEOUT));
	if (ret == -EAGAIN) {
		LOG_ERR("%s timeout", __func__);
		return -ETIMEDOUT;
	}

	if (regs->FCSTS & err_mask) {
		LOG_ERR("%s err: %x", __func__, err_mask);
		regs->FCSTS = err_mask;
		return -EIO;
	}

	return 0;
}

static int espi_xec_flash_erase(const struct device *dev, struct espi_flash_packet *pckt)
{
	struct espi_xec_data *data = (struct espi_xec_data *)dev->data;
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_ioc_fc_regs *regs =
		(struct xec_espi_ioc_fc_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_FC_OFS);
	uint32_t err_mask = (MCHP_ESPI_FC_STS_IBERR | MCHP_ESPI_FC_STS_OVRUN |
			     MCHP_ESPI_FC_STS_FAIL | MCHP_ESPI_FC_STS_BADREQ);
	uint32_t status = 0;
	int ret = 0;

	LOG_DBG("%s", __func__);

	if (!(regs->FCSTS & MCHP_ESPI_FC_STS_CHAN_EN)) {
		LOG_ERR("Flash channel is disabled");
		return -EIO;
	}

	if ((regs->FCCFG & MCHP_ESPI_FC_CFG_BUSY)) {
		LOG_ERR("Flash channel is busy");
		return -EBUSY;
	}

	/* Clear status register */
	status = regs->FCSTS;
	regs->FCSTS = status;

	regs->FCFA[1] = 0;
	regs->FCFA[0] = pckt->flash_addr;
	regs->FCLEN = ESPI_FLASH_ERASE_DUMMY;
	regs->FCCTL = MCHP_ESPI_FC_CTRL_FUNC_SET(MCHP_ESPI_FC_CTRL_ERS0);
	regs->FCCTL |= MCHP_ESPI_FC_CTRL_START;

	/* Wait until ISR or timeout */
	ret = k_sem_take(&data->flash_lock, K_MSEC(MAX_FLASH_TIMEOUT));
	if (ret == -EAGAIN) {
		LOG_ERR("%s timeout", __func__);
		return -ETIMEDOUT;
	}

	if (regs->FCSTS & err_mask) {
		LOG_ERR("%s err: %x", __func__, err_mask);
		regs->FCSTS = err_mask;
		return -EIO;
	}

	return 0;
}
#endif /* CONFIG_ESPI_FLASH_CHANNEL */

static int espi_xec_manage_callback(const struct device *dev, struct espi_callback *cb, bool set)
{
	struct espi_xec_data *const data = dev->data;

	return espi_manage_callback(&data->callbacks, cb, set);
}

#ifdef CONFIG_ESPI_AUTOMATIC_BOOT_DONE_ACKNOWLEDGE
static void send_slave_bootdone(const struct device *dev)
{
	int ret = 0;
	uint8_t boot_done = false;

	ret = espi_xec_receive_vwire(dev, ESPI_VWIRE_SIGNAL_TARGET_BOOT_DONE, &boot_done);
	if ((ret == 0) && (boot_done == true)) {
		/* SLAVE_BOOT_DONE & SLAVE_LOAD_STS have to be sent together */
		espi_xec_send_vwire(dev, ESPI_VWIRE_SIGNAL_TARGET_BOOT_STS, 1);
		espi_xec_send_vwire(dev, ESPI_VWIRE_SIGNAL_TARGET_BOOT_DONE, 1);
	}
}
#endif

#ifdef CONFIG_ESPI_OOB_CHANNEL
static void espi_init_oob(const struct device *dev)
{
	const struct espi_xec_config *cfg = dev->config;
	struct xec_espi_ioc_oob_regs *regs =
		(struct xec_espi_ioc_oob_regs *)(cfg->ioc_base_addr + MCHP_ESPI_IO_OOB_OFS);

	/* Enable OOB Tx/Rx interrupts */
	soc_ecia_girq_ctrl(cfg->irq_info_list[oob_up_girq_idx].gid,
			   cfg->irq_info_list[oob_up_girq_idx].gpos, 1u);
	soc_ecia_girq_ctrl(cfg->irq_info_list[oob_dn_girq_idx].gid,
			   cfg->irq_info_list[oob_dn_girq_idx].gpos, 1u);

	regs->OOBTXA[1] = 0;
	regs->OOBRXA[1] = 0;
	regs->OOBTXA[0] = (uint32_t)&target_tx_mem[0];
	regs->OOBRXA[0] = (uint32_t)&target_rx_mem[0];
	regs->OOBRXL = 0x00FF0000;

	/* Enable OOB Tx channel enable change status interrupt */
	regs->OOBTXIEN |= MCHP_ESPI_OOB_TX_IEN_CHG_EN | MCHP_ESPI_OOB_TX_IEN_DONE;

	/* Enable Rx channel to receive data any time
	 * there are case where OOB is not initiated by a previous OOB Tx
	 */
	regs->OOBRXIEN |= MCHP_ESPI_OOB_RX_IEN;
	regs->OOBRXC |= MCHP_ESPI_OOB_RX_CTRL_AVAIL;
}
#endif

#ifdef CONFIG_ESPI_FLASH_CHANNEL
static void espi_init_flash(const struct device *dev)
{
	const struct espi_xec_config *cfg = dev->config;
	struct xec_espi_ioc_fc_regs *regs =
		(struct xec_espi_ioc_fc_regs *)(cfg->ioc_base_addr + MCHP_ESPI_IO_FC_OFS);

	LOG_DBG("%s", __func__);

	/* Need to clear status done when ROM boots in MAF */
	LOG_DBG("%s ESPI_FC_REGS->CFG %X", __func__, regs->FCCFG);
	regs->FCSTS = MCHP_ESPI_FC_STS_DONE;

	/* Enable interrupts */
	soc_ecia_girq_ctrl(cfg->irq_info_list[fc_girq_idx].gid,
			   cfg->irq_info_list[fc_girq_idx].gpos, 1u);

	regs->FCIEN |= MCHP_ESPI_FC_IEN_CHG_EN;
	regs->FCIEN |= MCHP_ESPI_FC_IEN_DONE;
}
#endif

static void espi_bus_init(const struct device *dev)
{
	const struct espi_xec_config *cfg = dev->config;

	/* Enable bus interrupts */
	soc_ecia_girq_ctrl(cfg->irq_info_list[pc_girq_idx].gid,
			   cfg->irq_info_list[pc_girq_idx].gpos, 1u);
	soc_ecia_girq_ctrl(cfg->irq_info_list[rst_girq_idx].gid,
			   cfg->irq_info_list[rst_girq_idx].gpos, 1u);
	soc_ecia_girq_ctrl(cfg->irq_info_list[vw_ch_en_girq_idx].gid,
			   cfg->irq_info_list[vw_ch_en_girq_idx].gpos, 1u);
}

/* Clear specified eSPI bus GIRQ status */
static int xec_espi_bus_intr_clr(const struct device *dev, enum xec_espi_girq_idx idx)
{
	const struct espi_xec_config *cfg = dev->config;

	if (idx >= max_girq_idx) {
		return -EINVAL;
	}

	soc_ecia_girq_status_clear(cfg->irq_info_list[idx].gid, cfg->irq_info_list[idx].gpos);

	return 0;
}

/* Enable/disable specified eSPI bus GIRQ */
static int xec_espi_bus_intr_ctl(const struct device *dev, enum xec_espi_girq_idx idx, uint8_t en)
{
	const struct espi_xec_config *cfg = dev->config;

	if (idx >= max_girq_idx) {
		return -EINVAL;
	}

	soc_ecia_girq_ctrl(cfg->irq_info_list[idx].gid, cfg->irq_info_list[idx].gpos, en);

	return 0;
}

/* Host address bits[47:32] shared by every memory BAR and SRAM BAR. */
#define XEC_PC_DEV_MBAR_HOST_ADDR_HIGH DT_INST_PROP_OR(0, host_memmap_addr_high, 0)
#define XEC_PC_SRAM_BAR_HOST_ADDR_HIGH DT_INST_PROP_OR(0, sram_bar_addr_high, 0)

/* Terminator for the flattened <index, value> peripheral channel decoder
 * tables below. No real BAR or Serial-IRQ index can take this value.
 */
#define XEC_PC_TBL_END 0xffffffffU

/* The io-bars, mem-bars and sirqs device tree properties carry register indices
 * named by include/zephyr/dt-bindings/espi/mchp-xec-espi-v3.h. Device tree
 * cannot see the SoC enums those macros mirror, so tie the two together here.
 */
BUILD_ASSERT(MCHP_XEC_ESPI_IOB_MBOX == (int)IOB_MBOX, "io-bars mailbox index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_IOB_KBC == (int)IOB_KBC, "io-bars KBC index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_IOB_ACPI_EC0 == (int)IOB_ACPI_EC0, "io-bars ACPI EC0 index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_IOB_ACPI_EC4 == (int)IOB_ACPI_EC4, "io-bars ACPI EC4 index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_IOB_ACPI_PM1 == (int)IOB_ACPI_PM1, "io-bars ACPI PM1 index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_IOB_PORT92 == (int)IOB_PORT92, "io-bars Port92 index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_IOB_UART1 == (int)IOB_UART1, "io-bars UART1 index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_IOB_EMI0 == (int)IOB_EMI0, "io-bars EMI0 index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_IOB_EMI2 == (int)IOB_EMI2, "io-bars EMI2 index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_IOB_P80BD == (int)IOB_P80BD, "io-bars Port80 index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_IOB_P80BD_ALIAS == (int)IOB_P80BD_ALIAS,
	     "io-bars Port80 alias index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_IOB_GLUE == (int)IOB_GLUE, "io-bars glue logic index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_IOB_UART3 == (int)IOB_UART3, "io-bars UART3 index mismatch");

BUILD_ASSERT(MCHP_XEC_ESPI_MEMB_MBOX == (int)MEMB_MBOX, "mem-bars mailbox index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_MEMB_ACPI_EC0 == (int)MEMB_ACPI_EC0, "mem-bars ACPI EC0 mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_MEMB_ACPI_EC4 == (int)MEMB_ACPI_EC4, "mem-bars ACPI EC4 mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_MEMB_EMI0 == (int)MEMB_EMI0, "mem-bars EMI0 index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_MEMB_EMI2 == (int)MEMB_EMI2, "mem-bars EMI2 index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_MEMB_T32B == (int)MEMB_T32B, "mem-bars T32B index mismatch");

BUILD_ASSERT(MCHP_XEC_ESPI_SIRQ_MBOX == (int)SIRQ_MBOX, "sirqs mailbox index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_SIRQ_MBOX_SMI == (int)SIRQ_MBOX_SMI, "sirqs mailbox SMI mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_SIRQ_KBC_KIRQ == (int)SIRQ_KBC_KIRQ, "sirqs KBC KIRQ mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_SIRQ_KBC_MIRQ == (int)SIRQ_KBC_MIRQ, "sirqs KBC MIRQ mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_SIRQ_ACPI_EC0_OBF == (int)SIRQ_ACPI_EC0_OBF, "sirqs EC0 OBF mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_SIRQ_ACPI_EC4_OBF == (int)SIRQ_ACPI_EC4_OBF, "sirqs EC4 OBF mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_SIRQ_UART1 == (int)SIRQ_UART1, "sirqs UART1 index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_SIRQ_EMI0_HEV == (int)SIRQ_EMI0_HEV, "sirqs EMI0 HEV mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_SIRQ_EMI2_E2H == (int)SIRQ_EMI2_E2H, "sirqs EMI2 E2H mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_SIRQ_EC == (int)SIRQ_EC, "sirqs EC index mismatch");
BUILD_ASSERT(MCHP_XEC_ESPI_SIRQ_UART3 == (int)SIRQ_UART3, "sirqs UART3 index mismatch");

/* Each enabled peripheral channel logical device child node describes its own
 * Host facing decoders as flattened <index value> pairs in io-bars, mem-bars
 * and sirqs. Concatenate all of them into one table per register bank so this
 * driver can re-program every decoder without knowing what the logical devices
 * are. Children without the property, and children that are not logical
 * devices at all such as the target attached flash node, contribute nothing.
 */
#define XEC_PC_PROP_ELEMS(node_id, prop)                                                           \
	IF_ENABLED(DT_NODE_HAS_PROP(node_id, prop),                                                \
		   (DT_FOREACH_PROP_ELEM_SEP(node_id, prop, DT_PROP_BY_IDX, (,)),))

#define XEC_PC_IO_BAR_ELEMS(node_id)  XEC_PC_PROP_ELEMS(node_id, io_bars)
#define XEC_PC_MEM_BAR_ELEMS(node_id) XEC_PC_PROP_ELEMS(node_id, mem_bars)
#define XEC_PC_SIRQ_ELEMS(node_id)    XEC_PC_PROP_ELEMS(node_id, sirqs)

/* <enum espi_io_bar_idx, Host I/O base address> pairs */
static const uint32_t xec_pc_io_bars[] = {
	DT_INST_FOREACH_CHILD_STATUS_OKAY(0, XEC_PC_IO_BAR_ELEMS) XEC_PC_TBL_END, 0U};

/* <enum espi_mem_bar_idx, Host memory base address bits[31:0]> pairs */
static const uint32_t xec_pc_mem_bars[] = {
	DT_INST_FOREACH_CHILD_STATUS_OKAY(0, XEC_PC_MEM_BAR_ELEMS) XEC_PC_TBL_END, 0U};

/* <enum espi_io_sirq_idx, Host Serial-IRQ slot> pairs */
static const uint32_t xec_pc_sirqs[] = {
	DT_INST_FOREACH_CHILD_STATUS_OKAY(0, XEC_PC_SIRQ_ELEMS) XEC_PC_TBL_END, 0U};

static void xec_pc_program_io_bars(const struct espi_xec_config *devcfg)
{
	struct xec_espi_ioc_cfg_regs *cfgregs =
		(struct xec_espi_ioc_cfg_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_CFG_OFS);

	for (size_t n = 0U; (n + 1U) < ARRAY_SIZE(xec_pc_io_bars); n += 2U) {
		uint32_t idx = xec_pc_io_bars[n];

		if (idx == XEC_PC_TBL_END) {
			break;
		}

		if (idx >= ARRAY_SIZE(cfgregs->IOHBAR)) {
			LOG_ERR("eSPI I/O BAR index %u out of range", idx);
			continue;
		}

		cfgregs->IOHBAR[idx] = MCHP_ESPI_IO_BAR_HOST_ADDR_SET(xec_pc_io_bars[n + 1U]) |
				       MCHP_ESPI_IO_BAR_HOST_VALID;
	}
}

static void xec_pc_program_mem_bars(const struct espi_xec_config *devcfg)
{
	for (size_t n = 0U; (n + 1U) < ARRAY_SIZE(xec_pc_mem_bars); n += 2U) {
		uint32_t idx = xec_pc_mem_bars[n];
		uint32_t haddr = xec_pc_mem_bars[n + 1U];
		int ret = 0;

		if (idx == XEC_PC_TBL_END) {
			break;
		}

		ret = xec_espi_mbar_host_set(devcfg->mc_base_addr, (uint8_t)idx, haddr,
					     (haddr != UINT32_MAX));
		if (ret != 0) {
			LOG_ERR("eSPI memory BAR index %u rejected", idx);
		}
	}
}

static void xec_pc_program_sirqs(const struct espi_xec_config *devcfg)
{
	struct xec_espi_ioc_cfg_regs *cfgregs =
		(struct xec_espi_ioc_cfg_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_CFG_OFS);

	for (size_t n = 0U; (n + 1U) < ARRAY_SIZE(xec_pc_sirqs); n += 2U) {
		uint32_t idx = xec_pc_sirqs[n];

		if (idx == XEC_PC_TBL_END) {
			break;
		}

		if (idx >= ARRAY_SIZE(cfgregs->SIRQ)) {
			LOG_ERR("eSPI Serial-IRQ index %u out of range", idx);
			continue;
		}

		cfgregs->SIRQ[idx] = (uint8_t)xec_pc_sirqs[n + 1U];
	}
}

/*
 * Program the Host facing decoders of every enabled peripheral channel logical
 * device. Must run whenever hardware has released these registers from reset
 * and before the owning peripheral channel drivers are told their registers are
 * usable again.
 */
static void xec_pc_bar_sirq_init(const struct device *dev)
{
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_mc_cfg_regs *mcregs =
		(struct xec_espi_mc_cfg_regs *)(devcfg->mc_base_addr + MCHP_ESPI_MC_CFG_OFS);

	mcregs->HBAR_EXT = XEC_PC_DEV_MBAR_HOST_ADDR_HIGH;
	mcregs->SRAM_EXT = XEC_PC_SRAM_BAR_HOST_ADDR_HIGH;

	xec_pc_program_io_bars(devcfg);
	xec_pc_program_mem_bars(devcfg);
	xec_pc_program_sirqs(devcfg);
}

/* Deliver one event to every peripheral channel driver that asked for it. */
static void xec_pc_notify(const struct device *dev, enum mchp_xec_espi_pc_event evt)
{
	struct espi_xec_data *const data = dev->data;
	struct mchp_xec_espi_pc_cb *cb = NULL;
	struct mchp_xec_espi_pc_cb *tmp = NULL;

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&data->pc_cbs, cb, tmp, node) {
		if ((cb->handler != NULL) && ((cb->evt_mask & BIT((uint32_t)evt)) != 0U)) {
			cb->handler(dev, cb->pc_dev, evt);
		}
	}
}

void mchp_xec_espi_v3_send_callbacks(const struct device *espi_dev, struct espi_event evt)
{
	struct espi_xec_data *const data = espi_dev->data;

	espi_send_callbacks(&data->callbacks, espi_dev, evt);
}

int mchp_xec_espi_pc_register(const struct device *espi_dev, struct mchp_xec_espi_pc_cb *cb)
{
	struct espi_xec_data *data = NULL;
	sys_snode_t *prev = NULL;
	unsigned int key = 0U;

	if ((espi_dev == NULL) || (cb == NULL) || (cb->pc_dev == NULL)) {
		return -EINVAL;
	}

	data = espi_dev->data;

	key = irq_lock();

	if (sys_slist_find(&data->pc_cbs, &cb->node, &prev)) {
		irq_unlock(key);
		return -EALREADY;
	}

	sys_slist_append(&data->pc_cbs, &cb->node);

	irq_unlock(key);

	return 0;
}

int mchp_xec_espi_pc_unregister(const struct device *espi_dev, struct mchp_xec_espi_pc_cb *cb)
{
	struct espi_xec_data *data = NULL;
	unsigned int key = 0U;
	bool found = false;

	if ((espi_dev == NULL) || (cb == NULL)) {
		return -EINVAL;
	}

	data = espi_dev->data;

	key = irq_lock();
	found = sys_slist_find_and_remove(&data->pc_cbs, &cb->node);
	irq_unlock(key);

	return found ? 0 : -ENOENT;
}

/*
 * Dispatch an LPC peripheral request to whichever peripheral channel driver
 * registered the opcode range it falls in. Replaces the compile time opcode
 * table the V2 driver uses, so the generic espi_read_lpc_request() and
 * espi_write_lpc_request() API keeps working for applications such as the
 * ec_host_cmd eSPI backend.
 */
static int xec_pc_lpc_request(const struct device *dev, enum lpc_peripheral_opcode op,
			      uint32_t *data, bool write)
{
	struct espi_xec_data *const drvdata = dev->data;
	struct mchp_xec_espi_pc_cb *cb = NULL;

	SYS_SLIST_FOR_EACH_CONTAINER(&drvdata->pc_cbs, cb, node) {
		if (cb->lpc_request == NULL) {
			continue;
		}

		if (((uint16_t)op < cb->lpc_op_start) || ((uint16_t)op > cb->lpc_op_end)) {
			continue;
		}

		return cb->lpc_request(cb->pc_dev, op, data, write);
	}

	return -ENOTSUP;
}

static int espi_xec_read_lpc_request(const struct device *dev, enum lpc_peripheral_opcode op,
				     uint32_t *data)
{
	return xec_pc_lpc_request(dev, op, data, false);
}

static int espi_xec_write_lpc_request(const struct device *dev, enum lpc_peripheral_opcode op,
				      uint32_t *data)
{
	return xec_pc_lpc_request(dev, op, data, true);
}

static void setup_espi_io_config(const struct device *dev, uint16_t host_address);

static void espi_rst_isr(const struct device *dev)
{
	struct espi_xec_data *const data = dev->data;
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_cap_regs *regs =
		(struct xec_espi_cap_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_CAP_OFS);
	struct espi_event evt = {ESPI_BUS_RESET, 0, 0};
	uint8_t rst_sts = 0;

#ifdef ESPI_XEC_V3_DEBUG
	data->espi_rst_count++;
#endif

	rst_sts = regs->ERIS;

	/* eSPI reset status register is clear on write register */
	regs->ERIS = MCHP_ESPI_RST_ISTS;
	/* clear GIRQ latched status */
	xec_espi_bus_intr_clr(dev, rst_girq_idx);

	if (!(rst_sts & MCHP_ESPI_RST_ISTS)) {
		return;
	}

	if (rst_sts & MCHP_ESPI_RST_ISTS_PIN_RO_HI) {
		/* eSPI Reset de-asserted. Hardware has released the peripheral
		 * channel Host facing decoders from reset, so restore them
		 * before telling the peripheral channel drivers their registers
		 * are usable again. The V2 driver never restores them here.
		 */
		evt.evt_data = 1;
#ifdef CONFIG_ESPI_OOB_CHANNEL
		espi_init_oob(dev);
#endif
#ifdef CONFIG_ESPI_FLASH_CHANNEL
		espi_init_flash(dev);
#endif
		espi_bus_init(dev);
		setup_espi_io_config(dev, MCHP_ESPI_IOBAR_INIT_DFLT);
		espi_send_callbacks(&data->callbacks, dev, evt);
		xec_pc_notify(dev, MCHP_XEC_ESPI_PC_EVT_ESPI_RESET_DEASSERT);
	} else {
		/* eSPI Reset asserted. Notify first so the peripheral channel
		 * drivers can quiesce pending Host operations before hardware
		 * holds their decoders in reset.
		 */
		evt.evt_data = 0;
		xec_pc_notify(dev, MCHP_XEC_ESPI_PC_EVT_ESPI_RESET_ASSERT);
		espi_send_callbacks(&data->callbacks, dev, evt);
#ifdef CONFIG_ESPI_OOB_CHANNEL
		espi_init_oob(dev);
#endif
#ifdef CONFIG_ESPI_FLASH_CHANNEL
		espi_init_flash(dev);
#endif
		espi_bus_init(dev);
	}
}

static void setup_espi_io_config(const struct device *dev, uint16_t host_address)
{
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_ioc_pc_regs *pcregs =
		(struct xec_espi_ioc_pc_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_PC_OFS);
	struct xec_espi_ioc_cfg_regs *cfgregs =
		(struct xec_espi_ioc_cfg_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_CFG_OFS);
	struct xec_espi_cap_regs *capregs =
		(struct xec_espi_cap_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_CAP_OFS);

	cfgregs->IOHBAR[IOB_IOC] = (host_address << 16) | MCHP_ESPI_IO_BAR_HOST_VALID;

	/* Program the Host facing decoders of every peripheral channel logical
	 * device child node. The peripheral channel drivers never touch these.
	 */
	xec_pc_bar_sirq_init(dev);

	pcregs->PCSTS = MCHP_ESPI_PC_STS_EN_CHG | MCHP_ESPI_PC_STS_BM_EN_CHG_POS;
	pcregs->PCIEN |= MCHP_ESPI_PC_IEN_EN_CHG;
	capregs->PCRDY = 1;

	LOG_DBG("PC Ready set to 1");
}

/*
 * Write the interrupt select field of the specified MSVW source.
 * Each MSVW controls 4 virtual wires.
 */
static int xec_espi_vw_intr_ctrl(const struct device *dev, uint8_t msvw_idx, uint8_t src_id,
				 uint8_t intr_mode)
{
	const struct espi_xec_config *devcfg = dev->config;
	struct espi_msvw_ar_regs *regs = (struct espi_msvw_ar_regs *)devcfg->vw_base_addr;
	uintptr_t msvw_addr = 0;

	if ((msvw_idx >= ESPI_NUM_MSVW) || (src_id > 3)) {
		return -EINVAL;
	}

	msvw_addr = (uintptr_t)&regs->MSVW[msvw_idx];

	sys_write8(intr_mode, msvw_addr + MSVW_BI_IRQ_SEL0 + src_id);

	return 0;
}

static void espi_pc_isr(const struct device *dev)
{
	struct espi_xec_data *const data = dev->data;
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_ioc_pc_regs *regs =
		(struct xec_espi_ioc_pc_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_PC_OFS);
	struct espi_event evt = {.evt_type = ESPI_BUS_EVENT_CHANNEL_READY,
				 .evt_details = ESPI_CHANNEL_PERIPHERAL,
				 .evt_data = 0};
	uint32_t status = regs->PCSTS;

	LOG_DBG("%s %x", __func__, status);
	if (status & MCHP_ESPI_PC_STS_BUS_ERR) {
		LOG_ERR("%s bus error", __func__);
		regs->PCSTS = MCHP_ESPI_PC_STS_BUS_ERR;
	}

	if (status & MCHP_ESPI_PC_STS_EN_CHG) {
		LOG_DBG("PC enable change");
		if (status & MCHP_ESPI_PC_STS_EN) {
			LOG_DBG("PC dis to en");
			setup_espi_io_config(dev, MCHP_ESPI_IOBAR_INIT_DFLT);
			xec_pc_notify(dev, MCHP_XEC_ESPI_PC_EVT_CHAN_ENABLED);
		} else {
			LOG_DBG("PC en to dis");
			xec_pc_notify(dev, MCHP_XEC_ESPI_PC_EVT_CHAN_DISABLED);
		}

		regs->PCSTS = MCHP_ESPI_PC_STS_EN_CHG;
	}

	if (status & MCHP_ESPI_PC_STS_BM_EN_CHG) {
		if (status & MCHP_ESPI_PC_STS_BM_EN) {
			evt.evt_data = ESPI_PC_EVT_BUS_MASTER_ENABLE;
			LOG_WRN("%s BM change %x", __func__, status);
			espi_send_callbacks(&data->callbacks, dev, evt);
		}

		regs->PCSTS = MCHP_ESPI_PC_STS_BM_EN_CHG;
	}

	xec_espi_bus_intr_clr(dev, pc_girq_idx);
}

static void espi_vw_chan_en_isr(const struct device *dev)
{
	struct espi_xec_data *const data = dev->data;
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_cap_regs *regs =
		(struct xec_espi_cap_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_CAP_OFS);
	struct espi_event evt = {.evt_type = ESPI_BUS_EVENT_CHANNEL_READY,
				 .evt_details = ESPI_CHANNEL_VWIRE,
				 .evt_data = 0};
	uint32_t status = regs->VW_CHEN_SR;

	if (status & MCHP_ESPI_VW_EN_STS_RO) {
		regs->VWRDY = 1;
		evt.evt_data = 1;
		/* VW channel interrupt can disabled at this point */
		xec_espi_bus_intr_ctl(dev, vw_ch_en_girq_idx, 0);

#ifdef CONFIG_ESPI_AUTOMATIC_BOOT_DONE_ACKNOWLEDGE
		send_slave_bootdone(dev);
#endif
	}

	espi_send_callbacks(&data->callbacks, dev, evt);

	xec_espi_bus_intr_clr(dev, vw_ch_en_girq_idx);
}

#ifdef CONFIG_ESPI_OOB_CHANNEL
static void espi_oob_down_isr(const struct device *dev)
{
	struct espi_xec_data *const data = dev->data;
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_ioc_oob_regs *regs =
		(struct xec_espi_ioc_oob_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_OOB_OFS);
#ifdef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
	struct espi_event evt = {
		.evt_type = ESPI_BUS_EVENT_OOB_RECEIVED, .evt_details = 0, .evt_data = 0};
#endif
	uint32_t status = regs->OOBRXSTS;

	LOG_DBG("%s %x", __func__, status);
	if (status & MCHP_ESPI_OOB_RX_STS_DONE) {
		/* Register is write-on-clear, ensure only 1 bit is affected */
		regs->OOBRXSTS = MCHP_ESPI_OOB_RX_STS_DONE;

#ifndef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
		k_sem_give(&data->rx_lock);
#else
		evt.evt_details = regs->OOBRXL & MCHP_ESPI_OOB_RX_LEN_MASK;
		espi_send_callbacks(&data->callbacks, dev, evt);
#endif
	}

	xec_espi_bus_intr_clr(dev, oob_dn_girq_idx);
}

static void espi_oob_up_isr(const struct device *dev)
{
	struct espi_xec_data *const data = dev->data;
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_ioc_oob_regs *regs =
		(struct xec_espi_ioc_oob_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_OOB_OFS);
	struct xec_espi_cap_regs *capregs =
		(struct xec_espi_cap_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_CAP_OFS);
	struct espi_event evt = {.evt_type = ESPI_BUS_EVENT_CHANNEL_READY,
				 .evt_details = ESPI_CHANNEL_OOB,
				 .evt_data = 0};
	uint32_t status = regs->OOBTXSTS;

	LOG_DBG("%s sts:%x", __func__, status);

	if (status & MCHP_ESPI_OOB_TX_STS_DONE) {
		/* Register is write-on-clear, ensure only 1 bit is affected */
		status = regs->OOBTXSTS = MCHP_ESPI_OOB_TX_STS_DONE;
		k_sem_give(&data->tx_lock);
	}

	if (status & MCHP_ESPI_OOB_TX_STS_CHG_EN) {
		if (status & MCHP_ESPI_OOB_TX_STS_CHEN) {
			espi_init_oob(dev);
			/* Indicate OOB channel is ready to eSPI host */
			capregs->OOBRDY = 1;
			evt.evt_data = 1;
		}

		status = regs->OOBTXSTS = MCHP_ESPI_OOB_TX_STS_CHG_EN;
		espi_send_callbacks(&data->callbacks, dev, evt);
	}

	xec_espi_bus_intr_clr(dev, oob_up_girq_idx);
}
#endif

#ifdef CONFIG_ESPI_FLASH_CHANNEL
static void espi_flash_isr(const struct device *dev)
{
	struct espi_xec_data *const data = dev->data;
	const struct espi_xec_config *devcfg = dev->config;
	struct xec_espi_ioc_fc_regs *regs =
		(struct xec_espi_ioc_fc_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_FC_OFS);
	struct xec_espi_cap_regs *capregs =
		(struct xec_espi_cap_regs *)(devcfg->ioc_base_addr + MCHP_ESPI_IO_CAP_OFS);
	struct espi_event evt = {
		.evt_type = ESPI_BUS_EVENT_CHANNEL_READY,
		.evt_details = ESPI_CHANNEL_FLASH,
		.evt_data = 0,
	};
	uint32_t status = regs->FCSTS;

	LOG_DBG("%s %x", __func__, status);

	if (status & MCHP_ESPI_FC_STS_DONE) {
		/* Ensure to clear only relevant bit */
		regs->FCSTS = MCHP_ESPI_FC_STS_DONE;

		k_sem_give(&data->flash_lock);
	}

	if (status & MCHP_ESPI_FC_STS_CHAN_EN_CHG) {
		/* Ensure to clear only relevant bit */
		regs->FCSTS = MCHP_ESPI_FC_STS_CHAN_EN_CHG;

		if (status & MCHP_ESPI_FC_STS_CHAN_EN) {
			espi_init_flash(dev);
			/* Indicate flash channel is ready to eSPI master */
			capregs->FCRDY = MCHP_ESPI_FC_READY;
			evt.evt_data = 1;
		}

		espi_send_callbacks(&data->callbacks, dev, evt);
	}

	xec_espi_bus_intr_clr(dev, fc_girq_idx);
}
#endif

/* Send callbacks if enabled and track eSPI host system state */
static void notify_system_state(const struct device *dev, enum espi_vwire_signal signal)
{
	struct espi_xec_data *const data = dev->data;
	struct espi_event evt = {ESPI_BUS_EVENT_VWIRE_RECEIVED, 0, 0};
	uint8_t status = 0;

	espi_xec_receive_vwire(dev, signal, &status);
	evt.evt_details = signal;
	evt.evt_data = status;
	espi_send_callbacks(&data->callbacks, dev, evt);
}

static void notify_host_warning(const struct device *dev, enum espi_vwire_signal signal)
{
	uint8_t status = 0;

	espi_xec_receive_vwire(dev, signal, &status);

	if (!IS_ENABLED(CONFIG_ESPI_AUTOMATIC_WARNING_ACKNOWLEDGE)) {
		struct espi_xec_data *const data = dev->data;
		struct espi_event evt = {ESPI_BUS_EVENT_VWIRE_RECEIVED, 0, 0};

		evt.evt_details = signal;
		evt.evt_data = status;
		espi_send_callbacks(&data->callbacks, dev, evt);
	} else {
		k_busy_wait(ESPI_XEC_VWIRE_ACK_DELAY);
		/* Some flows are dependent on awareness of client's driver
		 * about these warnings in such cases these automatic response
		 * should not be enabled.
		 */
		switch (signal) {
		case ESPI_VWIRE_SIGNAL_HOST_RST_WARN:
			espi_xec_send_vwire(dev, ESPI_VWIRE_SIGNAL_HOST_RST_ACK, status);
			break;
		case ESPI_VWIRE_SIGNAL_SUS_WARN:
			espi_xec_send_vwire(dev, ESPI_VWIRE_SIGNAL_SUS_ACK, status);
			break;
		case ESPI_VWIRE_SIGNAL_OOB_RST_WARN:
			espi_xec_send_vwire(dev, ESPI_VWIRE_SIGNAL_OOB_RST_ACK, status);
			break;
		case ESPI_VWIRE_SIGNAL_DNX_WARN:
			espi_xec_send_vwire(dev, ESPI_VWIRE_SIGNAL_DNX_ACK, status);
			break;
		default:
			break;
		}
	}
}

static void notify_vw_status(const struct device *dev, enum espi_vwire_signal signal)
{
	struct espi_xec_data *const data = dev->data;
	struct espi_event evt = {ESPI_BUS_EVENT_VWIRE_RECEIVED, 0, 0};
	uint8_t status = 0;

	espi_xec_receive_vwire(dev, signal, &status);
	evt.evt_details = signal;
	evt.evt_data = status;
	espi_send_callbacks(&data->callbacks, dev, evt);
}

/*
 * VW handlers must have signature
 * typedef void (*mchp_xec_ecia_callback_t) (int girq_id, int src, void *user)
 * where parameter user is a pointer to const struct device
 * These handlers are registered to their respective GIRQ child device of the
 * ECIA driver.
 */

static void vw_slp3_handler(const struct device *dev)
{
	notify_system_state(dev, ESPI_VWIRE_SIGNAL_SLP_S3);
}

static void vw_slp4_handler(const struct device *dev)
{
	notify_system_state(dev, ESPI_VWIRE_SIGNAL_SLP_S4);
}

static void vw_slp5_handler(const struct device *dev)
{
	notify_system_state(dev, ESPI_VWIRE_SIGNAL_SLP_S5);
}

static void vw_host_rst_warn_handler(const struct device *dev)
{
	notify_host_warning(dev, ESPI_VWIRE_SIGNAL_HOST_RST_WARN);
}

static void vw_sus_warn_handler(const struct device *dev)
{
	notify_host_warning(dev, ESPI_VWIRE_SIGNAL_SUS_WARN);
}

static void vw_oob_rst_handler(const struct device *dev)
{
	notify_host_warning(dev, ESPI_VWIRE_SIGNAL_OOB_RST_WARN);
}

static void vw_sus_pwrdn_ack_handler(const struct device *dev)
{
	notify_vw_status(dev, ESPI_VWIRE_SIGNAL_SUS_PWRDN_ACK);
}

static void vw_slp_a_handler(const struct device *dev)
{
	notify_vw_status(dev, ESPI_VWIRE_SIGNAL_SLP_A);
}

static void vw_dnx_warn_handler(const struct device *dev)
{
	notify_host_warning(dev, ESPI_VWIRE_SIGNAL_DNX_WARN);
}

static void vw_pltrst_handler(const struct device *dev)
{
	struct espi_xec_data *const data = dev->data;
	struct espi_event evt = {ESPI_BUS_EVENT_VWIRE_RECEIVED, ESPI_VWIRE_SIGNAL_PLTRST, 0};
	uint8_t status = 0;

	espi_xec_receive_vwire(dev, ESPI_VWIRE_SIGNAL_PLTRST, &status);
	evt.evt_data = status;

	if (status != 0) {
		LOG_DBG("VW nPLTRST de-asserted");
		/* Decoders are out of reset: restore them, then notify. */
		setup_espi_io_config(dev, MCHP_ESPI_IOBAR_INIT_DFLT);
		espi_send_callbacks(&data->callbacks, dev, evt);
		xec_pc_notify(dev, MCHP_XEC_ESPI_PC_EVT_PLTRST_DEASSERT);
	} else {
		LOG_DBG("VW nPLTRST asserted");
		/* Notify before hardware holds the decoders in reset. */
		xec_pc_notify(dev, MCHP_XEC_ESPI_PC_EVT_PLTRST_ASSERT);
		espi_send_callbacks(&data->callbacks, dev, evt);
	}
}

static void vw_sus_stat_handler(const struct device *dev)
{
	notify_host_warning(dev, ESPI_VWIRE_SIGNAL_SUS_STAT);
}

static void vw_slp_wlan_handler(const struct device *dev)
{
	notify_vw_status(dev, ESPI_VWIRE_SIGNAL_SLP_WLAN);
}

static void vw_slp_lan_handler(const struct device *dev)
{
	notify_vw_status(dev, ESPI_VWIRE_SIGNAL_SLP_LAN);
}

static void vw_host_c10_handler(const struct device *dev)
{
	notify_vw_status(dev, ESPI_VWIRE_SIGNAL_HOST_C10);
}

static void vw_nmiout_handler(const struct device *dev)
{
	notify_vw_status(dev, ESPI_VWIRE_SIGNAL_NMIOUT);
}

static void vw_smiout_handler(const struct device *dev)
{
	notify_vw_status(dev, ESPI_VWIRE_SIGNAL_SMIOUT);
}

struct espi_ht_sig_handler {
	void (*vw_handler)(const struct device *dev);
};

const struct espi_ht_sig_handler espi_ht_vw_bank0_sh_tbl[] = {
	{vw_slp3_handler}, /* MSVW00 b[0] */
	{vw_slp4_handler},
	{vw_slp5_handler},
	{NULL},
	{vw_sus_stat_handler}, /* MSVW01 b[0] */
	{vw_pltrst_handler},
	{vw_oob_rst_handler},
	{NULL},
	{vw_host_rst_warn_handler}, /* MSVW02 b[0] */
	{vw_nmiout_handler},
	{vw_smiout_handler},
	{NULL},
	{vw_sus_warn_handler}, /* MSVW03 b[0] */
	{vw_sus_pwrdn_ack_handler},
	{NULL},
	{vw_slp_a_handler},
	{vw_slp_lan_handler}, /* MSVW04 b[0] */
	{vw_slp_wlan_handler},
	{NULL},
	{NULL},
	{NULL}, /* MSVW05 b[0] */
	{NULL},
	{NULL},
	{NULL},
	{NULL}, /* MSVW06 b[0] */
	{NULL},
	{NULL},
	{NULL},
};

const struct espi_ht_sig_handler espi_ht_vw_bank1_sh_tbl[] = {
	{vw_host_c10_handler}, /* MSVW07 b[0] */
	{NULL},
	{NULL},
	{NULL},
	{NULL}, /* MSVW08 b[0] */
	{vw_dnx_warn_handler},
	{NULL},
	{NULL},
	{NULL}, /* MSVW09 b[0] */
	{NULL},
	{NULL},
	{NULL},
	{NULL}, /* MSVW10 b[0] */
	{NULL},
	{NULL},
	{NULL},
};

static void espi_ht_vw_bank0_isr(const struct device *dev)
{
	const struct espi_xec_config *drvcfg = dev->config;
	uint32_t result = 0, bitpos = 0;
	uint8_t girq = drvcfg->irq_info_list[ht_vw_bank0_idx].gid;

	soc_ecia_girq_result(girq, &result);

	/* returns 0 if result is 0 else (bit position + 1) */
	bitpos = (uint32_t)find_lsb_set(result);
	while (bitpos != 0) {
		bitpos -= 1U; /* make zero based */
		result &= ~BIT(bitpos);

		soc_ecia_girq_status_clear(girq, bitpos);

		if (espi_ht_vw_bank0_sh_tbl[bitpos].vw_handler != NULL) {
			espi_ht_vw_bank0_sh_tbl[bitpos].vw_handler(dev);
		}

		bitpos = (uint32_t)find_lsb_set(result);
	}
}

static void espi_ht_vw_bank1_isr(const struct device *dev)
{
	const struct espi_xec_config *drvcfg = dev->config;
	uint32_t result = 0, bitpos = 0;
	uint8_t girq = drvcfg->irq_info_list[ht_vw_bank1_idx].gid;

	soc_ecia_girq_result(girq, &result);

	/* returns 0 if result is 0 else (bit position + 1) */
	bitpos = (uint32_t)find_lsb_set(result);
	while (bitpos != 0) {
		result &= ~BIT(--bitpos);

		soc_ecia_girq_status_clear(girq, bitpos);

		if (espi_ht_vw_bank1_sh_tbl[bitpos].vw_handler != NULL) {
			espi_ht_vw_bank1_sh_tbl[bitpos].vw_handler(dev);
		}

		bitpos = (uint32_t)find_lsb_set(result);
	}
}

static int espi_xec_init(const struct device *dev);

static DEVICE_API(espi, espi_xec_driver_api) = {
	.config = espi_xec_configure,
	.get_channel_status = espi_xec_channel_ready,
	.send_vwire = espi_xec_send_vwire,
	.receive_vwire = espi_xec_receive_vwire,
#ifdef CONFIG_ESPI_OOB_CHANNEL
	.send_oob = espi_xec_send_oob,
	.receive_oob = espi_xec_receive_oob,
#endif
#ifdef CONFIG_ESPI_FLASH_CHANNEL
	.flash_read = espi_xec_flash_read,
	.flash_write = espi_xec_flash_write,
	.flash_erase = espi_xec_flash_erase,
#endif
	.manage_callback = espi_xec_manage_callback,
	.read_lpc_request = espi_xec_read_lpc_request,
	.write_lpc_request = espi_xec_write_lpc_request,
};

static struct espi_xec_data espi_xec_data_var;

/* n = node-id, p = property, i = index */
#define XEC_IRQ_INFO(n, p, i)                                                                      \
	{                                                                                          \
		.gid = MCHP_XEC_ECIA_GIRQ(DT_PROP_BY_IDX(n, p, i)),                                \
		.gpos = MCHP_XEC_ECIA_GIRQ_POS(DT_PROP_BY_IDX(n, p, i)),                           \
		.anid = MCHP_XEC_ECIA_NVIC_AGGR(DT_PROP_BY_IDX(n, p, i)),                          \
		.dnid = MCHP_XEC_ECIA_NVIC_DIRECT(DT_PROP_BY_IDX(n, p, i)),                        \
	},

static const struct espi_xec_irq_info espi_xec_irq_info_0[] = {
	DT_FOREACH_PROP_ELEM(DT_DRV_INST(0), girqs, XEC_IRQ_INFO)};

/* pin control structure(s) */
PINCTRL_DT_INST_DEFINE(0);

static const struct espi_xec_config espi_xec_config = {
	.ioc_base_addr = DT_INST_REG_ADDR_BY_NAME(0, io),
	.mc_base_addr = DT_INST_REG_ADDR_BY_NAME(0, mem),
	.vw_base_addr = DT_INST_REG_ADDR_BY_NAME(0, vw),
	.pcr_scr = DT_INST_PROP(0, pcr_scr),
	.irq_info_size = ARRAY_SIZE(espi_xec_irq_info_0),
	.irq_info_list = espi_xec_irq_info_0,
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
};

DEVICE_DT_INST_DEFINE(0, &espi_xec_init, NULL, &espi_xec_data_var, &espi_xec_config, PRE_KERNEL_2,
		      CONFIG_ESPI_INIT_PRIORITY, &espi_xec_driver_api);

#define XEC_GIRQ24_NODE DT_NODELABEL(girq24)
#define XEC_GIRQ25_NODE DT_NODELABEL(girq25)

#define XEC_GIRQ_EN(girq_node_id) DT_NODE_HAS_STATUS_OKAY(node_id)

#define XEC_HAS_IDRV_GIRQ_24_25                                                                    \
	((DT_HAS_COMPAT_STATUS_OKAY(microchip_xec_ecia) != 0) &&                                   \
	 ((XEC_GIRQ_EN(XEC_GIRQ24_NODE) != 0) || (XEC_GIRQ_EN(XEC_GIRQ25_NODE) != 0)))

BUILD_ASSERT(XEC_HAS_IDRV_GIRQ_24_25 == 0, "GIRQ24/25 cannot be owned by another driver!");

/*
 * Connect ESPI bus interrupt handlers: ESPI_RESET and channels.
 * MEC172x hardware fixed SAF interrupt routing bug. SAF driver
 * will connect its direct mode interrupt handler(s) on this GIRQ.
 */
static void espi_xec_connect_irqs(const struct device *dev)
{
	const struct espi_xec_config *drvcfg = dev->config;

	/* eSPI Reset */
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, erst, irq), DT_INST_IRQ_BY_NAME(0, erst, priority),
		    espi_rst_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, erst, irq));
	soc_ecia_girq_ctrl(drvcfg->irq_info_list[rst_girq_idx].gid,
			   drvcfg->irq_info_list[rst_girq_idx].gpos, 1u);

	/* eSPI Virtual wire channel enable change ISR */
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, vw_chan_en, irq),
		    DT_INST_IRQ_BY_NAME(0, vw_chan_en, priority), espi_vw_chan_en_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, vw_chan_en, irq));
	soc_ecia_girq_ctrl(drvcfg->irq_info_list[vw_ch_en_girq_idx].gid,
			   drvcfg->irq_info_list[vw_ch_en_girq_idx].gpos, 1u);

	/* aggregated GIRQ for VWire bank 0 */
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, ht_vw_bank0, irq),
		    DT_INST_IRQ_BY_NAME(0, ht_vw_bank0, priority), espi_ht_vw_bank0_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, ht_vw_bank0, irq));
	soc_ecia_girq_ctrl(drvcfg->irq_info_list[ht_vw_bank0_idx].gid,
			   drvcfg->irq_info_list[ht_vw_bank0_idx].gpos, 1u);

	/* aggregated GIRQ for VWire bank 1 */
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, ht_vw_bank1, irq),
		    DT_INST_IRQ_BY_NAME(0, ht_vw_bank1, priority), espi_ht_vw_bank1_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, ht_vw_bank1, irq));
	soc_ecia_girq_ctrl(drvcfg->irq_info_list[ht_vw_bank1_idx].gid,
			   drvcfg->irq_info_list[ht_vw_bank1_idx].gpos, 1u);

	/* eSPI Peripheral Channel */
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, 0, irq), DT_INST_IRQ_BY_IDX(0, 0, priority), espi_pc_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_IDX(0, 0, irq));
	soc_ecia_girq_ctrl(drvcfg->irq_info_list[pc_girq_idx].gid,
			   drvcfg->irq_info_list[pc_girq_idx].gpos, 1u);

#ifdef CONFIG_ESPI_OOB_CHANNEL
	/* eSPI OOB Upstream direction */
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, 4, irq), DT_INST_IRQ_BY_IDX(0, 4, priority),
		    espi_oob_up_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_IDX(0, 4, irq));

	/* eSPI OOB Channel Downstream direction */
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, 5, irq), DT_INST_IRQ_BY_IDX(0, 5, priority),
		    espi_oob_down_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_IDX(0, 5, irq));

	soc_ecia_girq_ctrl(drvcfg->irq_info_list[oob_up_girq_idx].gid,
			   drvcfg->irq_info_list[oob_up_girq_idx].gpos, 1u);
	soc_ecia_girq_ctrl(drvcfg->irq_info_list[oob_dn_girq_idx].gid,
			   drvcfg->irq_info_list[oob_dn_girq_idx].gpos, 1u);
#endif

#ifdef CONFIG_ESPI_FLASH_CHANNEL
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, 6, irq), DT_INST_IRQ_BY_IDX(0, 6, priority),
		    espi_flash_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_IDX(0, 6, irq));
	soc_ecia_girq_ctrl(drvcfg->irq_info_list[fc_girq_idx].gid,
			   drvcfg->irq_info_list[fc_girq_idx].gpos, 1u);
#endif
}

/* MSVW is a 96-bit register and SMVW is a 64-bit register.
 * Each MSVW/SMVW controls a group of 4 eSPI virtual wires.
 * Host index located in b[7:0]
 * Reset source located in b[9:8]
 * Reset VW values SRC[3:0] located in b[15:12].
 * MSVW current VW state values located in bits[64, 72, 80, 88]
 * SMVW current VW state values located in bits[32, 40, 48, 56]
 */
static void xec_vw_cfg_properties(const struct xec_signal *p, uint32_t regaddr, uint8_t dir)
{
	uint32_t src_ofs = 4u;
	uint8_t src_pos = (8u * p->bit);
	uint8_t rst_state = (p->flags >> MCHP_DT_ESPI_VW_FLAG_RST_STATE_POS) &
			    MCHP_DT_ESPI_VW_FLAG_RST_STATE_MSK0;
	uint8_t rst_src =
		(p->flags >> MCHP_DT_ESPI_VW_FLAG_RST_SRC_POS) & MCHP_DT_ESPI_VW_FLAG_RST_SRC_MSK0;

	if (dir) {
		src_ofs = 8u;
	}

	if (rst_state || rst_src) {     /* change reset source or state ? */
		sys_write8(0, regaddr); /* disable register */

		uint8_t temp = sys_read8(regaddr + 1u);

		if (rst_state) { /* change reset state and default value of this vwire? */
			rst_state--;
			if (rst_state) {
				temp |= BIT(p->bit + 4u);
				sys_set_bit(regaddr + src_ofs, src_pos);
			} else {
				temp &= ~BIT(p->bit + 4u);
				sys_clear_bit(regaddr + src_ofs, src_pos);
			}
		}

		if (rst_src) { /* change reset source of all vwires in this group? */
			rst_src--;
			temp = (temp & ~0x3u) | (rst_src & 0x3u);
		}

		sys_write8(temp, regaddr + 1u);
	}

	if (sys_read8(regaddr) != p->host_idx) {
		sys_write8(p->host_idx, regaddr);
	}
}

/* Check each VW register set host index is present.
 * Some VW's power up with the host index and others do not.
 * NOTE: Virtual wires are in groups of 4. Disabling one wire in a group
 * will disable all wires in the group. We do not implement disabling.
 */
static void xec_vw_config(const struct device *dev)
{
	for (int i = ESPI_VWIRE_SIGNAL_TARGET_GPIO_0; i < ARRAY_SIZE(vw_tbl); i++) {
		const struct xec_signal *p = &vw_tbl[i];
		uint32_t regaddr = xec_smvw_addr(dev, p->xec_reg_idx);
		uint8_t dir = (p->flags >> MCHP_DT_ESPI_VW_FLAG_DIR_POS) & BIT(0);
		uint8_t en = (p->flags & BIT(MCHP_DT_ESPI_VW_FLAG_STATUS_POS));

		if (dir) {
			regaddr = xec_msvw_addr(dev, p->xec_reg_idx);
		}

		if (en) {
			xec_vw_cfg_properties(p, regaddr, dir);
		}
	}
}

/* The tables are by bit position in the VW GIRQ registers.
 * first table entry is bit[0] of GIRQ registers, etc.
 * Each four consecutive table entries are source 0, 1, 2, 3 of
 * the Host-to-Target VW 96-bit register.
 * The 96-bit VWire register index is table_idx / 4
 * Source position is table_idx % 4
 */
static void xec_vw_bank_isel_cfg(const struct device *dev, const struct espi_ht_sig_handler *tbl,
				 size_t max_tbl_entries, uint8_t bank_num)
{
	const struct espi_xec_config *drvcfg = dev->config;
	uint32_t girq_bm = 0;
	uint8_t girq = 0, vw_reg_idx = 0, vw_src_id = 0;

	if (bank_num == 0) {
		girq = drvcfg->irq_info_list[ht_vw_bank0_idx].gid;
	} else {
		girq = drvcfg->irq_info_list[ht_vw_bank1_idx].gid;
	}

	for (size_t n = 0; n < max_tbl_entries; n++) {
		if (tbl[n].vw_handler == NULL) {
			continue;
		}

		girq_bm |= BIT(n);

		vw_reg_idx = n / 4u;
		vw_src_id = n % 4u;

		xec_espi_vw_intr_ctrl(dev, vw_reg_idx, vw_src_id, MSVW_IRQ_SEL_EDGE_BOTH);
	}

	soc_ecia_girq_ctrl_bm(girq, girq_bm, 1u);
}

static void xec_register_vw_handlers(const struct device *dev)
{
	xec_vw_bank_isel_cfg(dev, espi_ht_vw_bank0_sh_tbl, ARRAY_SIZE(espi_ht_vw_bank0_sh_tbl), 0);
	xec_vw_bank_isel_cfg(dev, espi_ht_vw_bank1_sh_tbl, ARRAY_SIZE(espi_ht_vw_bank1_sh_tbl), 1);
}

/*
 * Initialize eSPI hardware and associated peripherals blocks using eSPI
 * as their host interface.
 * We change VW capabilities reported to match the number of VWires the
 * driver is supporting.
 * A VW packet on the bus contains VW count followed by the VW groups.
 * The VW count is a zero based 6-bit value: (0 - 63) specifying the number of
 * groups in the packet.
 * A VW group consists of two bytes: VW host index and VW data. Each group
 * contains the state of 4 virtual wires.
 * The total supported virtual wires is 64 * 4 = 256.
 * MEC172x supports 11 MSVW groups and 11 SMVW groups.
 * NOTE: While ESPI_nRESET is active most of the eSPI hardware is held
 * in reset state.
 */
static int espi_xec_init(const struct device *dev)
{
	struct espi_xec_data *const data = dev->data;
	const struct espi_xec_config *cfg = dev->config;
	struct xec_espi_ioc_pc_regs *pcregs =
		(struct xec_espi_ioc_pc_regs *)(cfg->ioc_base_addr + MCHP_ESPI_IO_PC_OFS);
	struct xec_espi_cap_regs *capregs =
		(struct xec_espi_cap_regs *)(cfg->ioc_base_addr + MCHP_ESPI_IO_CAP_OFS);
	mm_reg_t pcr_base = XEC_PCR_REG_BASE;
	int ret = 0;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("XEC eSPI V2 pinctrl setup failed (%d)", ret);
		return ret;
	}

#ifdef ESPI_XEC_V3_DEBUG
	data->espi_rst_count = 0;
#endif
	/* clear eSPI PCR sleep enable */
	soc_xec_pcr_sleep_en_clear(cfg->pcr_scr);

	/* Configure eSPI_PLTRST# to cause nSIO_RESET reset
	 * NOTE: this is also clearing bit 0(PWR_INV) causing the internal
	 * RESET_VCC to de-assert. Host facing peripherals will no longer
	 * be held in reset.
	 */
	sys_set_bit(pcr_base + XEC_CC_PRC_OFS, XEC_CC_PRC_HRST_PIN_POS);
	sys_clear_bit(pcr_base + XEC_CC_PRC_OFS, XEC_CC_PRC_PWR_INV_POS);

	capregs->PLTSRC = MCHP_ESPI_PLTRST_SRC_IS_VW;

	/* Configure the channels and its capabilities based on build config */
	capregs->CAP0 |= MCHP_ESPI_GBL_CAP0_VW_SUPP | MCHP_ESPI_GBL_CAP0_PC_SUPP;

	capregs->CAPVW = MAX(ESPI_NUM_MSVW, ESPI_NUM_SMVW);
	capregs->CAPPC |= MCHP_ESPI_PC_CAP_MAX_PLD_SZ_64;

#ifdef CONFIG_ESPI_OOB_CHANNEL
	capregs->CAP0 |= MCHP_ESPI_GBL_CAP0_OOB_SUPP;
	capregs->CAPOOB |= MCHP_ESPI_OOB_CAP_MAX_PLD_SZ_73;

	k_sem_init(&data->tx_lock, 0, 1);
#ifndef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
	k_sem_init(&data->rx_lock, 0, 1);
#endif /* CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC */
#else
	capregs->CAP0 &= ~MCHP_ESPI_GBL_CAP0_OOB_SUPP;
#endif

#ifdef CONFIG_ESPI_FLASH_CHANNEL
	capregs->CAP0 |= MCHP_ESPI_GBL_CAP0_FC_SUPP | MCHP_ESPI_FC_CAP_MAX_PLD_SZ_64;
	capregs->CAPFC |= MCHP_ESPI_FC_CAP_SHARE_CAF_TAF | MCHP_ESPI_FC_CAP_MAX_RD_SZ_64;

	k_sem_init(&data->flash_lock, 0, 1);
#else
	capregs->CAP0 &= ~MCHP_ESPI_GBL_CAP0_FC_SUPP;
#endif

	/* Clear reset interrupt status and enable interrupts */
	capregs->ERIS = MCHP_ESPI_RST_ISTS;
	capregs->ERIE |= MCHP_ESPI_RST_IEN;

	pcregs->PCSTS = MCHP_ESPI_PC_STS_EN_CHG;
	pcregs->PCIEN |= MCHP_ESPI_PC_IEN_EN_CHG;

	xec_vw_config(dev);

	/* register VWire handlers with their aggregated GIRQs
	 * in the ECIA driver
	 */
	xec_register_vw_handlers(dev);

#ifdef CONFIG_ESPI_OOB_CHANNEL
	espi_init_oob(dev);
#endif
#ifdef CONFIG_ESPI_FLASH_CHANNEL
	espi_init_flash(dev);
#endif

	espi_xec_connect_irqs(dev);

	/* Peripheral channel logical devices are separate drivers. Each one
	 * connects its own interrupts from its own node and registers with this
	 * controller from its own init function.
	 */

	return ret;
}
