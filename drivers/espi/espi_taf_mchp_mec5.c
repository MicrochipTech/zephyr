/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_espi_taf

#include <zephyr/kernel.h>
#include <soc.h>
#include <errno.h>
#include <mec_retval.h> /* MEC5 HAL headers */
#include <mec_pcr_api.h>
#include <mec_qspi_api.h>
#include <mec_espi_taf.h>
#include <zephyr/arch/common/ffs.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_taf.h>
#include <zephyr/drivers/espi/espi_mchp_mec5.h>
#include <zephyr/drivers/espi/espi_taf_mchp_mec5.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/espi/mchp-mec5-espi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include "espi_utils.h"

LOG_MODULE_REGISTER(espi_taf_mec5, CONFIG_ESPI_LOG_LEVEL);

#define MEC5_ESPI_TAF_DEBUG_ISR

#define MEC5_ESPI_TAF_ECP_TMOUT_US CONFIG_ESPI_TAF_MEC5_EC_PORTAL_XFR_TIMEOUT_US
/* eSPI TAF specification maximum allowed data transfer size */
#define MEC5_ESPI_TAF_XFRBUF_SIZE  64u

#define MEC5_ESPI_TAF_XFR_FLAG_ASYNC BIT(0)

#define MEC5_ESPI_TAF_FLASH_CFG_CS0 BIT(0)
#define MEC5_ESPI_TAF_FLASH_CFG_CS1 BIT(1)
#define MEC5_ESPI_TAF_FLASH_CFG_RPMC_CS0 BIT(4)
#define MEC5_ESPI_TAF_FLASH_CFG_RPMC_CS1 BIT(5)

#define MEC5_ESPI_TAF_RPMC_OP2_BUF_LEN 64

struct mec5_espi_taf_devcfg {
	struct mec_espi_taf_regs *regs;
	void (*irq_cfg_func)(const struct device *);
	uint32_t poll_timeout;
	uint32_t poll_interval;
	uint32_t consec_rd_timeout;
	uint32_t sus_chk_delay;
	uint32_t sus_rsm_interval;
};

struct mec5_espi_taf_data {
	uint8_t *xfrbuf_ptr;
	uint32_t hwmon_status;
	uint32_t ecp_status;
	struct k_sem ecp_lock;
	struct k_sem ecp_done;
	const struct espi_taf_cfg *taf_cfg;
	sys_slist_t callbacks;
	uint16_t xflags;
	uint16_t flash_cfg;
	struct mec_taf_ecp_cmd_pkt cmd_pkt;
#ifdef CONFIG_ESPI_TAF_MEC5_RPMC_SUPPORT
	uint8_t *rpmc_op_buf_ptr;
#endif
#ifdef MEC5_ESPI_TAF_DEBUG_ISR
	uint32_t isr_done_count;
	uint32_t isr_err_count;
#endif
} __aligned(4);

static void record_taf_chip_selects(const struct device *dev, const struct espi_taf_cfg *taf_cfg)
{
	struct mec5_espi_taf_data *const data = dev->data;

	for (uint8_t n = 0; n < taf_cfg->nflash_devices; n++) {
		if (taf_cfg->flash_cfgs[n].cs == 0) {
			data->flash_cfg |= MEC5_ESPI_TAF_FLASH_CFG_CS0;
#ifdef CONFIG_ESPI_TAF_MEC5_RPMC_SUPPORT
			if (taf_cfg->flash_cfgs[n].rpmc_info) {
				data->flash_cfg |= MEC5_ESPI_TAF_FLASH_CFG_RPMC_CS0;
			}
#endif
		} else if (taf_cfg->flash_cfgs[n].cs == 1) {
			data->flash_cfg |= MEC5_ESPI_TAF_FLASH_CFG_CS1;
#ifdef CONFIG_ESPI_TAF_MEC5_RPMC_SUPPORT
			if (taf_cfg->flash_cfgs[n].rpmc_info) {
				data->flash_cfg |= MEC5_ESPI_TAF_FLASH_CFG_RPMC_CS1;
			}
#endif
		}
	}
}

/* -------- Public APIs -------- */

/* Configure TAF and the QSPI controller it makes use of.
 * NOTE: This routine does not activate TAF as there is an activate API.
 * Once activated any access to the QSPI register will be blocked and
 * generate a fault to the processor.
 */
static int mec5_espi_taf_config(const struct device *dev, const struct espi_taf_cfg *taf_cfg)
{
	const struct mec5_espi_taf_devcfg *const devcfg = dev->config;
	struct mec_espi_taf_regs *const tregs = devcfg->regs;
	struct mec5_espi_taf_data *const data = dev->data;
	int ret = 0;

	if (!taf_cfg) {
		return -EINVAL;
	}

	if (mec_hal_espi_taf_is_activated()) {
		return -EAGAIN;
	}

	data->taf_cfg = taf_cfg;
	data->xflags = 0u;
	data->flash_cfg = 0u;

	for (uint8_t n = 0; n < taf_cfg->nflash_devices; n++) {
		if (taf_cfg->flash_cfgs[n].cs == 0) {
			data->flash_cfg |= MEC5_ESPI_TAF_FLASH_CFG_CS0;
		} else if (taf_cfg->flash_cfgs[n].cs == 1) {
			data->flash_cfg |= MEC5_ESPI_TAF_FLASH_CFG_CS1;
		}
	}

	const struct espi_taf_hw_cfg *hwcfg = &taf_cfg->hwcfg;

	ret = mec_hal_espi_taf_config(tregs, hwcfg, taf_cfg->flash_cfgs, taf_cfg->nflash_devices);
	if (ret != MEC_RET_OK) {
		LOG_ERR("MEC5 TAF config error");
		return -EIO;
	}

#ifdef CONFIG_ESPI_TAF_MEC5_RPMC_SUPPORT
	ret = mec_hal_espi_taf_flash_rpmc_op1_config(tregs, hwcfg, taf_cfg->flash_cfgs,
						     taf_cfg->nflash_devices);
	if (ret != MEC_RET_OK) {
		LOG_ERR("MEC5 TAF config HW RPMC error");
		return -EIO;
	}
#endif

	mec_hal_espi_taf_poll_timing(tregs, devcfg->poll_timeout, devcfg->poll_interval);
	mec_hal_espi_taf_read_timeout(tregs, devcfg->consec_rd_timeout);
	mec_hal_espi_taf_sus_timeout(tregs, devcfg->sus_rsm_interval, devcfg->sus_chk_delay);

	record_taf_chip_selects(dev, taf_cfg);

	return 0;
}

static int mec5_espi_taf_set_pr(const struct device *dev, const struct espi_taf_protection *pr)
{
	const struct mec5_espi_taf_devcfg *const devcfg = dev->config;
	struct mec_espi_taf_regs *const tregs = devcfg->regs;
	int ret = 0;

	if (!pr || (pr->nregions >= MEC_ESPI_TAF_PROT_REG_MAX)) {
		return -EINVAL;
	}

	if (mec_hal_espi_taf_is_activated()) {
		return -EAGAIN;
	}

	for (size_t n = 0; n < pr->nregions; n++) {
		const struct espi_taf_pr *pregion = &pr->pregions[n];

		if (mec_hal_espi_taf_pr_set(tregs, pregion) != MEC_RET_OK) {
			LOG_ERR("MEC5 TAF set PR[%u] error", n);
			ret = -EIO;
		}
	}

	return ret;
}

static int mec5_espi_taf_activate(const struct device *dev)
{
	const struct mec5_espi_taf_devcfg *const devcfg = dev->config;
	struct mec_espi_taf_regs *const tregs = devcfg->regs;
	struct mec5_espi_taf_data *const data = dev->data;
	uint32_t iflags = (BIT(MEC_ESPI_TAF_INTR_ECP_DONE_POS)
			   | BIT(MEC_ESPI_TAF_INTR_HWMON_ERR_POS));
	int ret = 0;

	if (mec_hal_espi_taf_is_activated()) {
		return -EAGAIN;
	}

	if (!data->taf_cfg) {
		return -EINVAL;
	}

	const struct espi_taf_cfg *tcfg = data->taf_cfg;
	const struct espi_taf_hw_cfg *hwcfg = &tcfg->hwcfg;
	const struct espi_taf_flash_cfg *fcfgs = tcfg->flash_cfgs;

	ret = mec_hal_espi_taf_qspi_init(tregs, hwcfg, fcfgs, tcfg->nflash_devices);
	if (ret != MEC_RET_OK) {
		LOG_ERR("MEC5 TAF QSPI init error");
		return -EIO;
	}

	mec_hal_espi_taf_ecp_istatus_clr(tregs, MEC_TAF_ECP_STS_ALL);
	mec_hal_espi_taf_mon_istatus_clr(tregs, MEC_TAF_HMON_STS_ALL);
	mec_hal_espi_taf_girq_status_clr(iflags);
	mec_hal_espi_taf_mon_ictrl(tregs, MEC_TAF_HMON_STS_ALL, 1u);

	mec_hal_espi_taf_activate(1u);
	mec_hal_espi_taf_girq_ctrl(1u, (BIT(MEC_ESPI_TAF_INTR_ECP_DONE_POS)
					| BIT(MEC_ESPI_TAF_INTR_HWMON_ERR_POS)));

	return 0;
}

static bool mec5_espi_taf_get_chan_sts(const struct device *dev)
{
	ARG_UNUSED(dev);

	if (mec_hal_espi_taf_is_activated()) {
		return true;
	}

	return false;
}

#if 0 /* TODO implement flags */
CONFIG_ESPI_TAF_MEC5_ASYNC
#define ESPI_TAF_MCHP_FLAG_CB_PER_BLOCK BIT(0)
#define ESPI_TAF_MCHP_FLAG_ASYNC BIT(4)
#endif
static int mec5_espi_taf_ecp_rw_cmd(const struct device *dev, struct espi_taf_packet *pckt,
				    uint32_t flags, uint8_t cmd)
{
	const struct mec5_espi_taf_devcfg *const devcfg = dev->config;
	struct mec_espi_taf_regs *const regs = devcfg->regs;
	struct mec5_espi_taf_data *const data = dev->data;
	uint32_t cflags = BIT(MEC_ESPI_TAF_ECP_CMD_FLAG_IEN_POS);
	uint32_t faddr = 0, nbytes = 0, nxfr = 0;
	uint8_t *pktbuf = NULL;
	int ret = 0;

	k_sem_take(&data->ecp_lock, K_FOREVER);

	if (!pckt) {
		ret = -EINVAL;
		goto ecp_cmd_rw_exit;
	}

	if (!mec_hal_espi_taf_is_activated()) {
		ret = -EIO;
		goto ecp_cmd_rw_exit;
	}

	if (mec_hal_espi_taf_ecp_busy(regs)) {
		ret = -EBUSY;
		goto ecp_cmd_rw_exit;
	}

	pktbuf = pckt->buf;
	faddr = pckt->flash_addr;
	nbytes = pckt->len;
	while (nbytes) {
		k_sem_reset(&data->ecp_done);

		nxfr = nbytes;
		if (nxfr > MEC5_ESPI_TAF_XFRBUF_SIZE) {
			nxfr = MEC5_ESPI_TAF_XFRBUF_SIZE;
		}

		if (cmd == MEC_TAF_ECP_CMD_WRITE) {
			memcpy(data->xfrbuf_ptr, pktbuf, nxfr);
		}

		data->cmd_pkt.dataptr = data->xfrbuf_ptr;
		data->cmd_pkt.dlen = nxfr;
		data->cmd_pkt.flash_addr = faddr;
		data->cmd_pkt.misc = cmd;

		ret = mec_hal_espi_taf_ecp_cmd_start(regs, cmd, &data->cmd_pkt, cflags);
		if (ret != MEC_RET_OK) {
			LOG_ERR("MEC5 TAF ECP cmd (%u) error (%d)", cmd, ret);
			ret = -EIO;
			break;
		}

		ret = k_sem_take(&data->ecp_done, K_USEC(MEC5_ESPI_TAF_ECP_TMOUT_US));
		if (ret == -EAGAIN) {
			LOG_ERR("MEC5 TAF ECP cmd (%u) timeout", cmd);
			ret = -ETIMEDOUT;
			break;
		}

		if (cmd == MEC_TAF_ECP_CMD_READ) {
			memcpy(pktbuf, data->xfrbuf_ptr, nxfr);
		}

		pktbuf += nxfr;
		faddr += nxfr;
		nbytes -= nxfr;
	}

ecp_cmd_rw_exit:
	memset(data->xfrbuf_ptr, 0, MEC5_ESPI_TAF_XFRBUF_SIZE);
	k_sem_give(&data->ecp_lock);

	return ret;
}

#if 0 /* TODO implement flags */
CONFIG_ESPI_TAF_MEC5_ASYNC
#define ESPI_TAF_MCHP_FLAG_CB_PER_BLOCK BIT(0)
#define ESPI_TAF_MCHP_FLAG_ASYNC BIT(4)
#endif
static int mec5_espi_taf_ecp_erase_cmd(const struct device *dev, struct espi_taf_packet *pckt,
				       uint32_t flags)
{
	const struct mec5_espi_taf_devcfg *const devcfg = dev->config;
	struct mec_espi_taf_regs *const regs = devcfg->regs;
	struct mec5_espi_taf_data *const data = dev->data;
	uint32_t cflags = BIT(MEC_ESPI_TAF_ECP_CMD_FLAG_IEN_POS);
	int ret = 0;

	k_sem_take(&data->ecp_lock, K_FOREVER);

	if (!pckt) {
		ret = -EINVAL;
		goto ecp_cmd_erase_exit;
	}

	if (!mec_hal_espi_taf_is_activated()) {
		ret = -EIO;
		goto ecp_cmd_erase_exit;
	}

	if (mec_hal_espi_taf_ecp_busy(regs)) {
		ret = -EBUSY;
		goto ecp_cmd_erase_exit;
	}

	data->cmd_pkt.dataptr = NULL;
	data->cmd_pkt.dlen = pckt->len;
	data->cmd_pkt.flash_addr = pckt->flash_addr;
	data->cmd_pkt.misc = MEC_TAF_ECP_CMD_ERASE;

	ret = mec_hal_espi_taf_ecp_cmd_start(regs, MEC_TAF_ECP_CMD_ERASE, &data->cmd_pkt, cflags);
	if (ret != MEC_RET_OK) {
		LOG_ERR("MEC5 TAF ECP erase error (%d)", ret);
		ret = -EIO;
		goto ecp_cmd_erase_exit;
	}

	ret = k_sem_take(&data->ecp_done, K_USEC(MEC5_ESPI_TAF_ECP_TMOUT_US));
	if (ret == -EAGAIN) {
		LOG_ERR("MEC5 TAF ECP erase timeout");
		ret = -ETIMEDOUT;
	}

 ecp_cmd_erase_exit:
	k_sem_give(&data->ecp_lock);

	return ret;
}

static int mec5_espi_taf_read(const struct device *dev, struct espi_taf_packet *pckt,
			      uint32_t flags)
{
	return mec5_espi_taf_ecp_rw_cmd(dev, pckt, flags, MEC_TAF_ECP_CMD_READ);
}

static int mec5_espi_taf_write(const struct device *dev, struct espi_taf_packet *pckt,
			       uint32_t flags)
{
	return mec5_espi_taf_ecp_rw_cmd(dev, pckt, flags, MEC_TAF_ECP_CMD_WRITE);
}

static int mec5_espi_taf_erase(const struct device *dev, struct espi_taf_packet *pckt,
			       uint32_t flags)
{
	return mec5_espi_taf_ecp_erase_cmd(dev, pckt, flags);
}

/* A transaction was started using pckt (read, write, or erase)
 * Get the status of the transaction. Busy poll?
 * return -ENOTSUP eSPI flash logical channel transactions not supported.
 * retrun -EBUSY eSPI flash channel is not ready or disabled by controller.
 * return -EIO General input / output error, failed request to controller.
 */
static int mec5_espi_taf_op_status(const struct device *dev, struct espi_taf_packet *pckt)
{
	const struct mec5_espi_taf_devcfg *const devcfg = dev->config;
	struct mec_espi_taf_regs *const tregs = devcfg->regs;

	if (!pckt) {
		return -EINVAL;
	}

	if (mec_hal_espi_taf_ecp_busy(tregs)) {
		return -EBUSY;
	}

	return 0;
}

static int mec5_espi_taf_man_cb(const struct device *dev, struct espi_callback *callback, bool set)
{
	struct mec5_espi_taf_data *const data = dev->data;

	if (!callback) {
		return -EINVAL;
	}

	return espi_manage_callback(&data->callbacks, callback, set);
}

/* Side-band API's */
int espi_taf_mchp_hwmon_ictrl(const struct device *dev, uint32_t intr_bitmap, uint8_t enable)
{
	if (!dev) {
		return -EINVAL;
	}

	const struct mec5_espi_taf_devcfg *const devcfg = dev->config;
	struct mec_espi_taf_regs *const tregs = devcfg->regs;
	int ret = mec_hal_espi_taf_mon_ictrl(tregs, intr_bitmap, enable);

	if (ret) {
		return -EIO;
	}

	return 0;
}

#ifdef CONFIG_ESPI_TAF_MEC5_RPMC_SUPPORT

static bool mec5_taf_flash_supports_rpmc(const struct device *dev, uint8_t cs)
{
	struct mec5_espi_taf_data *const data = dev->data;

	switch (cs) {
	case 0:
		if (data->flash_cfg & MEC5_ESPI_TAF_FLASH_CFG_RPMC_CS0) {
			return true;
		}
		break;
	case 1:
		if (data->flash_cfg & MEC5_ESPI_TAF_FLASH_CFG_RPMC_CS1) {
			return true;
		}
		break;
	default:
		break;
	}

	return false;
}

static bool mec5_taf_is_pkt_buf_valid(void *buf, uint32_t reqlen)
{
	if (!buf) {
		return false;
	}

	if ((uint32_t)buf & 0x03u) { /* not aligned >= 4 bytes */
		return false;
	}

	if ((reqlen < 1u) || (reqlen > 64u)) {
		return false;
	}

	return true;
}

#if 0 /* TODO implement flags */
CONFIG_ESPI_TAF_MEC5_ASYNC
#define ESPI_TAF_MCHP_FLAG_CB_PER_BLOCK BIT(0)
#define ESPI_TAF_MCHP_FLAG_ASYNC BIT(4)
#endif
int espi_taf_mchp_rpmc_operation(const struct device *dev, struct espi_taf_rpmc_packet *pkt,
				 uint32_t flags)
{
	if (!dev || !pkt) {
		return -EINVAL;
	}

	if (!mec_hal_espi_taf_is_activated()) {
		return -EIO;
	}

	const struct mec5_espi_taf_devcfg *const devcfg = dev->config;
	struct mec_espi_taf_regs *const tregs = devcfg->regs;
	struct mec5_espi_taf_data *const data = dev->data;
	enum mec_taf_ecp_command cmd = MEC_TAF_ECP_CMD_RPMC_OP1_CS0;
	uint32_t hal_flags = BIT(MEC_ESPI_TAF_ECP_CMD_FLAG_IEN_POS);
	int ret = 0;

	if (mec_hal_espi_taf_ecp_busy(tregs)) {
		return -EBUSY;
	}

	if (!mec5_taf_flash_supports_rpmc(dev, pkt->cs)) {
		return -ENOTSUP;
	}

	if (!mec5_taf_is_pkt_buf_valid(pkt->buf, pkt->len)) {
		return -ENOMSG;
	}

	if (pkt->cmd == ESPI_TAF_CMD_RPMC_OP1) {
		if (pkt->cs) {
			cmd = MEC_TAF_ECP_CMD_RPMC_OP1_CS1;
		}
		memcpy(data->xfrbuf_ptr, pkt->buf, pkt->len);
		data->cmd_pkt.dataptr = data->xfrbuf_ptr;
	} else if (pkt->cmd == ESPI_TAF_CMD_RPMC_OP2) {
		cmd = MEC_TAF_ECP_CMD_RPMC_OP2_CS0;
		if (pkt->cs) {
			cmd = MEC_TAF_ECP_CMD_RPMC_OP2_CS1;
		}
		data->cmd_pkt.dataptr = pkt->buf;
	} else {
		return -EPROTOTYPE;
	}

	k_sem_take(&data->ecp_lock, K_FOREVER);

	if (flags & ESPI_TAF_MCHP_RPMC_OP_FLAG_ASYNC) {
		data->xflags = MEC5_ESPI_TAF_XFR_FLAG_ASYNC;
	} else {
		data->xflags &= (uint16_t)~MEC5_ESPI_TAF_XFR_FLAG_ASYNC;
	}

	data->cmd_pkt.dlen = pkt->len;
	data->cmd_pkt.flash_addr = 0;
	data->cmd_pkt.misc = cmd; /* store cmd in misc for use by ISR */

	ret = mec_hal_espi_taf_ecp_cmd_start(tregs, cmd, &data->cmd_pkt, hal_flags);
	if (ret || (flags & ESPI_TAF_MCHP_RPMC_OP_FLAG_ASYNC)) {
		return ret;
	}

	ret = k_sem_take(&data->ecp_done, K_USEC(MEC5_ESPI_TAF_ECP_TMOUT_US)); /* decrement */
	if (ret == -EAGAIN) {
		LOG_ERR("MEC5 TAF ECP RPMC timeout");
		ret = -ETIMEDOUT;
	}

	return 0;
}
#else
int espi_taf_mchp_rpmc_operation(const struct device *dev, struct espi_taf_rpmc_packet *pkt,
				 uint32_t flags)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pkt);

	return -ENOTSUP;
}
#endif /* CONFIG_ESPI_TAF_MEC5_RPMC_SUPPORT */

/* -------- driver ISR -------- */

static uint32_t taf_hmon_event_data(uint32_t hwmon_status)
{
	uint32_t evd = 0;

	if (hwmon_status & BIT(MEC_TAF_HMON_STS_TIMEOUT_POS)) {
		evd |= ESPI_TAF_PROTOCOL_ERR_TIMEOUT;
	}
	if (hwmon_status & BIT(MEC_TAF_HMON_STS_OOR_POS)) {
		evd |= ESPI_TAF_PROTOCOL_ERR_TIMEOUT;
	}
	if (hwmon_status & BIT(MEC_TAF_HMON_STS_AVL_POS)) {
		evd |= ESPI_TAF_PROTOCOL_ERR_ACCESS_VIOLATION;
	}
	if (hwmon_status & BIT(MEC_TAF_HMON_STS_CROSS_4KB_POS)) {
		evd |= ESPI_TAF_PROTOCOL_ERR_4K_BOUNDARY;
	}
	if (hwmon_status & BIT(MEC_TAF_HMON_STS_INVAL_ERSZ_POS)) {
		evd |= ESPI_TAF_PROTOCOL_ERR_ERASE_SIZE;
	}

	return evd;
}

static uint32_t taf_ecp_event_data(uint32_t ecp_status)
{
	uint32_t evd = 0;

	if (ecp_status & BIT(MEC_TAF_ECP_STS_TIMEOUT_POS)) {
		evd |= ESPI_TAF_PROTOCOL_ERR_TIMEOUT;
	}
	if (ecp_status & BIT(MEC_TAF_ECP_STS_OOR_POS)) {
		evd |= ESPI_TAF_PROTOCOL_ERR_OUT_OF_RANGE;
	}
	if (ecp_status & BIT(MEC_TAF_ECP_STS_AVL_POS)) {
		evd |= ESPI_TAF_PROTOCOL_ERR_ACCESS_VIOLATION;
	}
	if (ecp_status & BIT(MEC_TAF_ECP_STS_CROSS_4KB_POS)) {
		evd |= ESPI_TAF_PROTOCOL_ERR_4K_BOUNDARY;
	}
	if (ecp_status & BIT(MEC_TAF_ECP_STS_INVAL_ERSZ_POS)) {
		evd |= ESPI_TAF_PROTOCOL_ERR_ERASE_SIZE;
	}
	if (ecp_status & BIT(MEC_TAF_ECP_STS_START_OVFL_POS)) {
		evd |= ESPI_TAF_PROTOCOL_ERR_ECP_START_OVL;
	}
	if (ecp_status & BIT(MEC_TAF_ECP_STS_BAD_REQ_POS)) {
		evd |= ESPI_TAF_PROTOCOL_ERR_ECP_BAD_REQ;
	}

	return evd;
}

static void update_taf_ecp_event(const struct device *dev, struct espi_event *pevt)
{
	struct mec5_espi_taf_data *const data = dev->data;

	switch (data->cmd_pkt.misc) {
	case MEC_TAF_ECP_CMD_READ:
		pevt->evt_details = ESPI_TAF_ECP_READ;
		break;
	case  MEC_TAF_ECP_CMD_WRITE:
		pevt->evt_details = ESPI_TAF_ECP_WRITE;
		break;
	case MEC_TAF_ECP_CMD_ERASE:
		pevt->evt_details = ESPI_TAF_ECP_ERASE;
		break;
	case MEC_TAF_ECP_CMD_RPMC_OP1_CS0:
		__fallthrough;
	case MEC_TAF_ECP_CMD_RPMC_OP1_CS1:
		pevt->evt_details = ESPI_TAF_ECP_RPMC_OP1;
		break;
	case MEC_TAF_ECP_CMD_RPMC_OP2_CS0:
		__fallthrough;
	case MEC_TAF_ECP_CMD_RPMC_OP2_CS1:
		pevt->evt_details = ESPI_TAF_ECP_RPMC_OP2;
		break;
	default:
		pevt->evt_details = 0;
	}

	if (data->ecp_status & MEC_TAF_ECP_STS_ERR_ALL) {
		pevt->evt_details |= ESPI_TAF_ECP_PROTOCOL_ERR;
	}

	pevt->evt_data = taf_ecp_event_data(data->ecp_status);
}

#if 0 /* TODO implement flags */
CONFIG_ESPI_TAF_MEC5_ASYNC
#define ESPI_TAF_MCHP_FLAG_CB_PER_BLOCK BIT(0)
#define ESPI_TAF_MCHP_FLAG_ASYNC BIT(4)
#endif
static void mec5_espi_taf_done_isr(const struct device *dev)
{
	const struct mec5_espi_taf_devcfg *const devcfg = dev->config;
	struct mec_espi_taf_regs *const tregs = devcfg->regs;
	struct mec5_espi_taf_data *const data = dev->data;
	struct espi_event evt = {
		.evt_type = ESPI_BUS_TAF_NOTIFICATION,
		.evt_details = 0,
		.evt_data = 0 };

#ifdef MEC5_ESPI_TAF_DEBUG_ISR
	data->isr_done_count++;
#endif
	mec_hal_espi_taf_ecp_ictrl(tregs, 0);
	mec_hal_espi_taf_ecp_istatus(tregs, &data->ecp_status);
	mec_hal_espi_taf_ecp_istatus_clr(tregs, MEC_TAF_ECP_STS_ALL);
	mec_hal_espi_taf_girq_status_clr(BIT(MEC_ESPI_TAF_INTR_ECP_DONE_POS));

	update_taf_ecp_event(dev, &evt);

	k_sem_give(&data->ecp_done);
	if (data->xflags & MEC5_ESPI_TAF_XFR_FLAG_ASYNC) {
		data->xflags &= (uint16_t)~MEC5_ESPI_TAF_XFR_FLAG_ASYNC;
		k_sem_give(&data->ecp_lock);
	}

	espi_send_callbacks(&data->callbacks, dev, evt);
}

static void mec5_espi_taf_err_isr(const struct device *dev)
{
	const struct mec5_espi_taf_devcfg *const devcfg = dev->config;
	struct mec_espi_taf_regs *const tregs = devcfg->regs;
	struct mec5_espi_taf_data *const data = dev->data;
	struct espi_event evt = {
		.evt_type = ESPI_BUS_TAF_NOTIFICATION,
		.evt_details = ESPI_TAF_HOST_PROTOCOL_ERR,
		.evt_data = 0 };

#ifdef MEC5_ESPI_TAF_DEBUG_ISR
	data->isr_err_count++;
#endif
	mec_hal_espi_taf_mon_istatus(tregs, &data->hwmon_status);
	mec_hal_espi_taf_mon_istatus_clr(tregs, MEC_TAF_HMON_STS_ALL);
	mec_hal_espi_taf_girq_status_clr(BIT(MEC_ESPI_TAF_INTR_HWMON_ERR_POS));

	evt.evt_data = taf_hmon_event_data(data->hwmon_status);

	espi_send_callbacks(&data->callbacks, dev, evt);
}

/* -------- driver intialization -------- */

/* Initialize eSPI TAF device
 * SoC Boot-ROM may have been configured to activate the eSPI interface and TAF
 * for system chipset to fetch Host CPU system BIOS.
 */
static int mec5_espi_taf_init(const struct device *dev)
{
	const struct mec5_espi_taf_devcfg *const devcfg = dev->config;
	struct mec_espi_taf_regs *const regs = devcfg->regs;
	struct mec5_espi_taf_data *data = dev->data;
	uint32_t tflags = BIT(MEC_ESPI_TAF_CAF_SHARE_POS);
	int ret = 0;

	k_sem_init(&data->ecp_lock, 1, 1);
	k_sem_init(&data->ecp_done, 1, 1);

#ifdef CONFIG_ESPI_TAF_MEC5_RPMC_SUPPORT
	data->rpmc_op_buf_ptr = NULL; /* TODO */
#endif

#ifdef CONFIG_ESPI_TAF_MEC5_RESET_ON_INIT
	tflags |= BIT(MEC_ESPI_TAF_INIT_RESET_POS);
#endif
	ret = mec_hal_espi_taf_init(regs, tflags);
	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	if (devcfg->irq_cfg_func) {
		devcfg->irq_cfg_func(dev);
	}

	return 0;
}

/* public API structure */
const struct espi_taf_driver_api mec5_espi_taf_api = {
	.config = mec5_espi_taf_config,
	.set_protection_regions = mec5_espi_taf_set_pr,
	.activate = mec5_espi_taf_activate,
	.get_channel_status = mec5_espi_taf_get_chan_sts,
	.flash_read = mec5_espi_taf_read,
	.flash_write = mec5_espi_taf_write,
	.flash_erase = mec5_espi_taf_erase,
	.flash_unsuccess = mec5_espi_taf_op_status,
	.manage_callback = mec5_espi_taf_man_cb,
};


#define MEC5_ESPI_TAF_XFR_BUF_DEF(inst) static uint8_t __aligned(4) \
	espi_taf_mec5_xfrbuf##inst[MEC5_ESPI_TAF_XFRBUF_SIZE]

#define MEC5_ESPI_TAF_DATA_INIT(inst) .xfrbuf_ptr = espi_taf_mec5_xfrbuf##inst

#define MEC5_ESPI_TAF_DEVICE(inst)                                                                 \
	MEC5_ESPI_TAF_XFR_BUF_DEF(inst);                                                           \
                                                                                                   \
	static struct mec5_espi_taf_data mec5_etaf_data_##inst = {                                 \
		MEC5_ESPI_TAF_DATA_INIT(inst),                                                     \
	};                                                                                         \
                                                                                                   \
	static void mec5_etaf_irq_cfg_func_##inst(const struct device *dev);                       \
                                                                                                   \
	static const struct mec5_espi_taf_devcfg mec5_etaf_dcfg_##inst = {                         \
		.regs = (struct mec_espi_taf_regs *)DT_INST_REG_ADDR(inst),                        \
		.irq_cfg_func = mec5_etaf_irq_cfg_func_##inst,                                     \
		.poll_timeout = DT_INST_PROP_OR(inst, poll_timeout, 0x28000),                      \
		.poll_interval = DT_INST_PROP_OR(inst, poll_interval, 0),                          \
		.consec_rd_timeout = DT_INST_PROP_OR(inst, consec_rd_timeout, 0),                  \
		.sus_chk_delay = DT_INST_PROP_OR(inst, sus_chk_delay, 20000),                      \
		.sus_rsm_interval = DT_INST_PROP_OR(inst, sus_rsm_interval, 32),                   \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, mec5_espi_taf_init, NULL, &mec5_etaf_data_##inst,              \
			      &mec5_etaf_dcfg_##inst, POST_KERNEL, CONFIG_ESPI_TAF_INIT_PRIORITY,  \
			      &mec5_espi_taf_api);                                                 \
                                                                                                   \
	static void mec5_etaf_irq_cfg_func_##inst(const struct device *dev)                        \
	{                                                                                          \
		uint32_t iflags = (BIT(MEC_ESPI_TAF_INTR_ECP_DONE_POS) |                           \
				   BIT(MEC_ESPI_TAF_INTR_HWMON_ERR_POS));                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, done, irq),                                  \
			    DT_INST_IRQ_BY_NAME(inst, done, priority), mec5_espi_taf_done_isr,     \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, done, irq));                                  \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, err, irq),                                   \
			    DT_INST_IRQ_BY_NAME(inst, err, priority), mec5_espi_taf_err_isr,       \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, err, irq));                                   \
		mec_hal_espi_taf_girq_ctrl(1u, iflags);                                            \
	}

DT_INST_FOREACH_STATUS_OKAY(MEC5_ESPI_TAF_DEVICE)
