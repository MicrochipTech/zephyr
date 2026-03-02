/*
 * Copyright (c) 2026 Microchip Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT microchip_xec_i3c

#include <soc.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i3c_mchp_xec, CONFIG_I3C_LOG_LEVEL);

#include "i3c_mchp_xec_priv.h"

#define DRV_RESP_WAIT_MS 1000U

#define DRV_EVENT_BIT_HANDLE_IBI         (0x01U << 1U)
#define DRV_EVENT_BIT_HANDLE_TGT_RX      (0x01U << 2U)
#define DRV_EVENT_BIT_HANDLE_TGT_TX_DONE (0x01U << 3U)

enum target_states {
	TGT_STATE_NOT_PRESENT,
	TGT_STATE_ADDR_ASSIGNED,
	TGT_STATE_NEEDS_DAA,
	TGT_STATE_DAA_IN_PROGRESS,
	TGT_STATE_MAX
};

enum ibi_node_states {
	IBI_NODE_STATE_FREE,
	IBI_NODE_STATE_IN_USE,
	IBI_NODE_ISR_UPDATED,
};

enum tgt_pvt_receive_node_states {
	TGT_RX_NODE_STATE_FREE,
	TGT_RX_NODE_STATE_IN_USE,
	TGT_RX_NODE_STATE_IN_USE_DMA,
	TGT_RX_NODE_ISR_UPDATED,
	TGT_RX_NODE_ISR_UPDATED_THR,
};

enum pending_xfer_type {
	XFER_TYPE_INVALID,
	XFER_TYPE_CCC,
	XFER_TYPE_ENTDAA,
	XFER_TYPE_PVT_RW,
	XFER_TYPE_TGT_RAISE_IBI,
	XFER_TYPE_TGT_RAISE_IBI_MR,
	XFER_TYPE_TGT_PVT_RD,
};

#define THRESHOLD_SIZE 32

/* Pending Transfer Context data */
struct i3c_pending_xfer pending_xfer_ctxt;

/* Tier1 change: round2: Static inline helper functions to avoid code duplication for role checks */
/* Check if controller is in master mode */
static inline bool xec_i3c_is_current_role_master(mm_reg_t regbase)
{
	uint32_t dev_ext_cr = sys_read32(regbase + XEC_I3C_DEV_EXT_CR_OFS);
	uint32_t opmode = XEC_I3C_DEV_EXT_CR_OPM_GET(dev_ext_cr);
	uint32_t hwcap = sys_read32(regbase + XEC_I3C_HW_CAP_OFS);
	uint32_t role = XEC_I3C_HW_CAP_ROLE_GET(hwcap);

	if (((role == XEC_I3C_HW_CAP_ROLE_SC) && (opmode != XEC_I3C_DEV_EXT_CR_OPM_HC)) ||
	    (role == XEC_I3C_HW_CAP_ROLE_TGT)) {
		return false;
	}
	return true;
}

/* Check if controller is current bus master */
static inline bool xec_i3c_is_current_role_bus_master(mm_reg_t regbase)
{
	uint32_t hwcap = sys_read32(regbase + XEC_I3C_HW_CAP_OFS);
	uint32_t role = XEC_I3C_HW_CAP_ROLE_GET(hwcap);

	if ((role == XEC_I3C_HW_CAP_ROLE_SC) &&
	    (sys_test_bit(regbase + XEC_I3C_PRES_ST_OFS, XEC_I3C_PRES_ST_CH_POS) == 0)) {
		return false;
	}
	return true;
}

/* Check if controller is configured as primary */
static inline bool xec_i3c_is_current_role_primary(mm_reg_t regbase)
{
	uint32_t hwcap = sys_read32(regbase + XEC_I3C_HW_CAP_OFS);
	uint32_t role = XEC_I3C_HW_CAP_ROLE_GET(hwcap);

	return (role == XEC_I3C_HW_CAP_ROLE_HC);
}

#ifdef CONFIG_I3C_USE_IBI
static struct ibi_node *_drv_i3c_free_ibi_node_get_isr(struct xec_i3c_data *xec_data)
{
	uint8_t idx;
	struct ibi_node *ret = NULL;

	for (idx = 0; idx < XEC_I3C_MAX_IBI_LIST_COUNT; idx++) {
		if (xec_data->ibis[idx].state == IBI_NODE_STATE_FREE) {
			xec_data->ibis[idx].state = IBI_NODE_STATE_IN_USE;
			ret = &xec_data->ibis[idx];
			break;
		}
	}
	return ret;
}
#endif

static struct i3c_tgt_pvt_receive_node *
_drv_i3c_free_tgt_rx_node_get_isr(struct xec_i3c_data *xec_data, bool dma_flag)
{
	uint8_t idx;
	struct i3c_tgt_pvt_receive_node *ret = NULL;

	for (idx = 0; idx < XEC_I3C_MAX_TGT_RX_LIST_COUNT; idx++) {
		if (xec_data->tgt_pvt_rx[idx].state == TGT_RX_NODE_STATE_FREE) {
			if (dma_flag) {
				xec_data->tgt_pvt_rx[idx].state = TGT_RX_NODE_STATE_IN_USE_DMA;
			} else {
				xec_data->tgt_pvt_rx[idx].state = TGT_RX_NODE_STATE_IN_USE;
			}
			ret = &xec_data->tgt_pvt_rx[idx];
			break;
		}
	}
	return ret;
}

static int _drv_i3c_targets_free_pos_get(const struct xec_i3c_data *xec_data, uint8_t *free_posn)
{
	uint8_t idx;
	int ret = -1;

	for (idx = 0; idx < XEC_I3C_MAX_TARGETS; idx++) {
		if (xec_data->targets[idx].state == TGT_STATE_NOT_PRESENT) {
			*free_posn = idx;
			ret = 0;
			break;
		}
	}
	return ret;
}

static int _drv_i3c_targets_next_DAA_get(struct xec_i3c_data *xec_data,
					 struct targets_on_bus **tgt_DAA)
{
	uint8_t idx;
	int ret = -1;

	for (idx = 0; idx < XEC_I3C_MAX_TARGETS; idx++) {
		if (xec_data->targets[idx].state == TGT_STATE_NEEDS_DAA) {
			*tgt_DAA = &xec_data->targets[idx];
			ret = 0;
			break;
		}
	}
	return ret;
}

/* Updates actual address assigned during DAA */
static void _drv_i3c_targets_DAA_addr_update(struct xec_i3c_data *data, uint64_t pid,
					     uint8_t new_addr, uint8_t new_dat_idx)
{
	uint8_t idx;

	for (idx = 0; idx < XEC_I3C_MAX_TARGETS; idx++) {
		if (data->targets[idx].state == TGT_STATE_DAA_IN_PROGRESS) {
			if (pid == data->targets[idx].pid) {
				data->targets[idx].address = new_addr;
				data->targets[idx].dat_idx = new_dat_idx;
				break;
			}
		}
	}
}

static void _drv_i3c_targets_DAA_done(const struct device *dev, bool DAA_success,
				      uint16_t dat_success_idx)
{
	struct xec_i3c_data *data = dev->data;
	const struct xec_i3c_config *config = dev->config;
	mm_reg_t regbase = config->regbase;
	uint8_t idx = 0, dat_idx = 0;

	for (idx = 0; idx < XEC_I3C_MAX_TARGETS; idx++) {
		if (data->targets[idx].state == TGT_STATE_DAA_IN_PROGRESS) {
			dat_idx = data->targets[idx].dat_idx;
			if (DAA_success && dat_idx <= dat_success_idx) {
				/* Mark state as address assigned */
				data->targets[idx].state = TGT_STATE_ADDR_ASSIGNED;
			} else {
				/* Tier1 change: round2: XEC_I3C_DAT_DynamicAddr_write - Write
				 * dynamic address to DAT entry */
				uint32_t val = DEV_ADDR_TABLE1_LOC1_DYNAMIC_ADDR(0);
				uintptr_t entry_addr = (uintptr_t)regbase +
						       (uintptr_t)data->DAT_start_addr +
						       ((uintptr_t)dat_idx * 4u);

				sys_write32(val, (mm_reg_t)entry_addr);

				/* Mark the DAT position as free */
				data->DAT_free_positions |= (1U << dat_idx);
				/* Mark state as needs DAA */
				data->targets[idx].state = TGT_STATE_NEEDS_DAA;
			}
		}
	}
}

static int _drv_i3c_DAT_free_pos_get(const struct xec_i3c_data *xec_data, uint16_t *free_posn)
{
	uint16_t max_positions_bitmask = 0U;
	uint16_t free_positions_bitmask = 0U;
	uint16_t posn = 0U;
	int ret = 0;

	max_positions_bitmask = GENMASK(xec_data->DAT_depth - 1, 0);
	if (xec_data->DAT_free_positions & max_positions_bitmask) {
		/* Get Leftmost Set bit in DAT_free_positions */
		free_positions_bitmask = xec_data->DAT_free_positions;
		while (!(free_positions_bitmask & (0x01U << posn))) {
			posn++;
		}
		*free_posn = posn;
	} else {
		ret = -1;
	}
	return ret;
}

static int _drv_i3c_DAT_idx_get(const struct xec_i3c_data *xec_data, const uint8_t tgt_addr,
				uint8_t *tgt_posn)
{
	uint8_t idx;
	int ret = -1;

	for (idx = 0; idx < xec_data->DAT_depth; idx++) {
		if (TGT_STATE_ADDR_ASSIGNED == xec_data->targets[idx].state) {
			if (tgt_addr == xec_data->targets[idx].address) {
				*tgt_posn = xec_data->targets[idx].dat_idx;
				ret = 0;
				break;
			}
		}
	}
	return ret;
}

static int i3c_xec_attach_device(const struct device *dev, struct i3c_device_desc *desc)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *data = dev->data;
	mm_reg_t regbase = config->regbase;
	uint16_t free_posn_DAT = 0;
	uint8_t target_info_idx = 0;
	uint8_t addr = desc->static_addr; /* TODO API removed addr parameter. Use static? */
	bool program_dyn_addr = false;
	int ret = -1;

	if (_drv_i3c_DAT_free_pos_get(data, &free_posn_DAT)) {
		/* Unable to find a free location in DAT */
		LOG_ERR("%s: no space in DAT for i3c device: %s", dev->name, desc->dev->name);
		return ret;
	}

	if (_drv_i3c_targets_free_pos_get(data, &target_info_idx)) {
		/* Unable to find a free location in targets list */
		LOG_ERR("%s: no space in targets list for i3c device: %s", dev->name,
			desc->dev->name);
		return ret;
	}

	/* Initialize the target info node */
	data->targets[target_info_idx].state = TGT_STATE_ADDR_ASSIGNED;
	data->targets[target_info_idx].address = addr;
	data->targets[target_info_idx].pid = desc->pid;
	desc->controller_priv = &data->targets[target_info_idx];

	/* Check if address is a dynamic address (set by primary controller) */
	if (desc->dynamic_addr != 0) {
		program_dyn_addr = true;
	}

	/* Check if dynamic address will be assigned by SETDASA */
	if ((desc->dynamic_addr == 0) && (desc->static_addr != 0)) {
		program_dyn_addr = true;
	}

	if (program_dyn_addr) {
		/* Tier1 change: round2: XEC_I3C_DAT_DynamicAddr_write - Write dynamic address to
		 * DAT entry */
		uint32_t val = DEV_ADDR_TABLE1_LOC1_DYNAMIC_ADDR(addr);
		uintptr_t entry_addr = (uintptr_t)regbase + (uintptr_t)data->DAT_start_addr +
				       ((uintptr_t)free_posn_DAT * 4u);

		sys_write32(val, (mm_reg_t)entry_addr);

		data->targets[target_info_idx].dat_idx = free_posn_DAT;
		/* Mark the free position as used */
		data->DAT_free_positions &= ~(1U << free_posn_DAT);
	} else {
		data->targets[target_info_idx].state = TGT_STATE_NEEDS_DAA;
	}

	return 0;
}

static int i3c_xec_detach_device(const struct device *dev, struct i3c_device_desc *desc)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *data = dev->data;
	mm_reg_t regbase = config->regbase;
	struct targets_on_bus *target_info = desc->controller_priv;

	if (NULL == target_info) {
		LOG_ERR("%s: %s: device not attached", dev->name, desc->dev->name);
		return -EINVAL;
	}

	/* Tier1 change: round2: XEC_I3C_DAT_DynamicAddr_write - Invalidate DAT entry by writing
	 * address 0 */
	uint32_t val = DEV_ADDR_TABLE1_LOC1_DYNAMIC_ADDR(0);
	uintptr_t entry_addr = (uintptr_t)regbase + (uintptr_t)data->DAT_start_addr +
			       ((uintptr_t)target_info->dat_idx * 4u);

	sys_write32(val, (mm_reg_t)entry_addr);

	/* Mark the DAT position as free */
	data->DAT_free_positions |= (1U << target_info->dat_idx);

	/* Reclaim the target info node */
	target_info->state = TGT_STATE_NOT_PRESENT;
	target_info->address = 0;
	target_info->dat_idx = 0;

	/* Clear the target info */
	desc->controller_priv = NULL;

	return 0;
}

static int i3c_xec_reattach_device(const struct device *dev, struct i3c_device_desc *desc,
				   uint8_t old_dyn_addr)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *data = dev->data;
	mm_reg_t regbase = config->regbase;
	struct targets_on_bus *target_info = desc->controller_priv;

	if (NULL == target_info) {
		LOG_ERR("%s: %s: device not attached", dev->name, desc->dev->name);
		return -EINVAL;
	}

	if (target_info->address != old_dyn_addr) {
		LOG_ERR("Old dynamic address doesn't match the one in DAT");
		return -EINVAL;
	}

	/* Tier1 change: round2: XEC_I3C_DAT_DynamicAddr_write - Update DAT entry with new dynamic
	 * address */
	uint32_t val = DEV_ADDR_TABLE1_LOC1_DYNAMIC_ADDR(desc->dynamic_addr);
	uintptr_t entry_addr = (uintptr_t)regbase + (uintptr_t)data->DAT_start_addr +
			       ((uintptr_t)target_info->dat_idx * 4u);
	sys_write32(val, (mm_reg_t)entry_addr);

	/* Update the target info node with new address */
	target_info->address = desc->dynamic_addr;

	return 0;
}

static void _drv_pending_xfer_ctxt_init(void)
{
	int i = 0;

	pending_xfer_ctxt.xfer_type = 0;
	for (i = 0; i < XEC_I3C_MAX_MSGS; i++) {
		pending_xfer_ctxt.node[i].data_buf = NULL;
		pending_xfer_ctxt.node[i].read = false;
		pending_xfer_ctxt.node[i].error_status = 0;
		pending_xfer_ctxt.node[i].tid = 0;
		pending_xfer_ctxt.node[i].ret_data_len = 0;
	}
}

static void _drv_dct_info_init(struct mec_i3c_DCT_info *info)
{
	info->bcr = 0x0;
	info->dcr = 0x0;
	info->dynamic_addr = 0x0;
	info->pid = 0x0;
}

static int _drv_i3c_CCC(const struct device *dev, struct i3c_ccc_payload *payload,
			uint8_t *response)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *xec_data = dev->data;
	struct mec_i3c_ctx *hwctx = &xec_data->ctx;
	struct mec_i3c_DO_CCC do_ccc_instance;
	struct i3c_ccc_target_payload *target;
	int ret = 0;
	int n;
	uint8_t num_targets;
	uint8_t DAT_idx; /* Index of the device in Device Address Table */

	hwctx->base = (mm_reg_t)config->regbase;
	*response = 0;

	memset(&do_ccc_instance, 0x00, sizeof(do_ccc_instance));

	/* Handle Broadcast Write CCC */
	if (payload->ccc.id <= I3C_CCC_BROADCAST_MAX_ID) {
		do_ccc_instance.read = false; /* No Broadcast Read */
		do_ccc_instance.defining_byte_valid = false;
		do_ccc_instance.ccc_id = payload->ccc.id;

		if (payload->ccc.data_len) {
			/* Set the first byte as the optional defining byte */
			do_ccc_instance.defining_byte = payload->ccc.data[0];
			do_ccc_instance.defining_byte_valid = true;

			/* Handle optional write data*/
			if (1 < payload->ccc.data_len) {
				do_ccc_instance.data_buf = &payload->ccc.data[1];
				do_ccc_instance.data_len = payload->ccc.data_len - 1;
			}
		}

		_drv_pending_xfer_ctxt_init();
		pending_xfer_ctxt.xfer_type = XFER_TYPE_CCC;
		pending_xfer_ctxt.xfer_sem = &xec_data->xfer_sem;

		XEC_I3C_DO_CCC(dev, &do_ccc_instance, &pending_xfer_ctxt.node[0].tid);

		if (k_sem_take(&xec_data->xfer_sem, K_MSEC(DRV_RESP_WAIT_MS))) {
			XEC_I3C_Xfer_Reset(dev);
			ret = -EBUSY;
		} else if (pending_xfer_ctxt.xfer_status) {
			*response = pending_xfer_ctxt.xfer_status;
			ret = -EIO;
		}
	} else { /* Handle Directed CCC */
		num_targets = payload->targets.num_targets;

		/* Ensure num_targets is valid */
		if ((0 == num_targets) || (XEC_I3C_MAX_TARGETS < num_targets)) {
			return -EINVAL;
		}

		for (n = 0; n < num_targets; n++) {
			do_ccc_instance.defining_byte_valid = false;
			do_ccc_instance.ccc_id = payload->ccc.id;

			_drv_pending_xfer_ctxt_init();
			pending_xfer_ctxt.xfer_type = XFER_TYPE_CCC;
			pending_xfer_ctxt.xfer_sem = &xec_data->xfer_sem;

			if (payload->ccc.data_len) {
				/* Take only the defining byte from the ccc data, if any other
				 * data then we are ignoring since for directed CCC there is
				 * only defining byte before the target slave address
				 */
				do_ccc_instance.defining_byte = payload->ccc.data[0];
				do_ccc_instance.defining_byte_valid = true;
			}

			target = &payload->targets.payloads[n];
			DAT_idx = 0;
			if (_drv_i3c_DAT_idx_get(xec_data, target->addr, &DAT_idx)) {
				/* Unable to locate target in target list */
				ret = -EINVAL;
				break;
			}

			do_ccc_instance.tgt_idx = DAT_idx;
			do_ccc_instance.data_buf = target->data;
			do_ccc_instance.data_len = target->data_len;

			if (target->rnw) {
				do_ccc_instance.read = true;
				pending_xfer_ctxt.node[0].data_buf = do_ccc_instance.data_buf;
				pending_xfer_ctxt.node[0].read = true;
			}

			XEC_I3C_DO_CCC(dev, &do_ccc_instance, &pending_xfer_ctxt.node[0].tid);

			if (k_sem_take(&xec_data->xfer_sem, K_MSEC(DRV_RESP_WAIT_MS))) {
				XEC_I3C_Xfer_Reset(dev);
				ret = -EBUSY;
				break;
			} else if (pending_xfer_ctxt.xfer_status) {
				*response = pending_xfer_ctxt.xfer_status;
				ret = -EIO;
				break;
			}
		} /* end for */
	}

	return ret;
}

/**
 * @brief Send Common Command Code (CCC).
 *
 * @see i3c_do_ccc
 *
 * @param dev Pointer to controller device driver instance.
 * @param payload Pointer to CCC payload.
 *
 * @return @see i3c_do_ccc
 */
static int i3c_xec_do_ccc(const struct device *dev, struct i3c_ccc_payload *payload)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *data = dev->data;
	mm_reg_t regbase = config->regbase;
	uint8_t response = 0;
	int ret = 0;

	/* Tier1 change: round2: Use inline helper - Check if controller is in master and bus master
	 * mode */
	if (false == xec_i3c_is_current_role_master(regbase) &&
	    xec_i3c_is_current_role_bus_master(regbase)) {
		ret = -EACCES;
		goto exit_ccc;
	}

	k_mutex_lock(&data->xfer_lock, K_FOREVER);

	LOG_DBG("[%s] - Sending CCC = 0x%02X", __func__, payload->ccc.id);

	ret = _drv_i3c_CCC(dev, payload, &response);

	k_mutex_unlock(&data->xfer_lock);

	if ((!ret) && response) { /* Error in Response */
		LOG_ERR("!!Error - 0x%08x - %d!!", response, ret);
	}

exit_ccc:
	return ret;
}

/**
 * @brief Perform Dynamic Address Assignment.
 *
 * @see i3c_do_daa
 *
 * @param dev Pointer to controller device driver instance.
 *
 * @return @see i3c_do_daa
 */
static int i3c_xec_do_daa(const struct device *dev)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *data = dev->data;
	mm_reg_t regbase = config->regbase;
	struct targets_on_bus *target_needs_DAA = NULL;
	uint64_t pid = 0;
	uint16_t DAA_entries_count = 0;
	uint16_t DAA_success_count = 0;
	uint16_t DAT_first_free_posn = 0;
	struct mec_i3c_DCT_info dct_info;
	int ret = 0, idx;

	/* Tier1 change: round2: Use inline helper - Check if controller is in master and bus master
	 * mode */
	if (false == xec_i3c_is_current_role_master(regbase) &&
	    xec_i3c_is_current_role_bus_master(regbase)) {
		ret = -EACCES;
		goto exit_da;
	}

	if (_drv_i3c_DAT_free_pos_get(data, &DAT_first_free_posn)) {
		/* No free location in DAT */
		LOG_ERR("%s: no space in DAT", dev->name);
		ret = -ENOMEM;
		goto exit_da;
	}

	k_mutex_lock(&data->xfer_lock, K_FOREVER);

	for (idx = DAT_first_free_posn; idx < data->DAT_depth; idx++) {
		/* Ensure DAT position is free */
		if (!(data->DAT_free_positions & (1U << idx))) {
			/* DAT position is not available; can occur during Hot Join
			 * Go for the next DAT position
			 */
			continue;
		}

		if (_drv_i3c_targets_next_DAA_get(data, &target_needs_DAA)) {
			break;
			/* IF DISCOVERY: Add logic to fill the remaining entries in DAT with
			 * possible dynamic address so that new devices on the bus can be
			 * discovered
			 */
		} else if (NULL != target_needs_DAA) {
			target_needs_DAA->dat_idx = idx;

			/* Tier1 change: round2: XEC_I3C_DAT_DynamicAddrAssign_write - Write dynamic
			 * address with parity for DAA */
			uint32_t val = DEV_ADDR_TABLE1_LOC1_DYNAMIC_ADDR(target_needs_DAA->address);
			if (!__builtin_parity(target_needs_DAA->address)) {
				val |= DEV_ADDR_TABLE1_LOC1_PARITY;
			}
			uintptr_t entry_addr = (uintptr_t)regbase +
					       (uintptr_t)data->DAT_start_addr +
					       ((uintptr_t)idx * 4u);

			sys_write32(val, (mm_reg_t)entry_addr);

			/* Mark the free position as used */
			data->DAT_free_positions &= ~(1U << idx);
			DAA_entries_count++;
			target_needs_DAA->state = TGT_STATE_DAA_IN_PROGRESS;
			/* Note: PID will be 0 for hot join device */
			LOG_DBG("ENTDAA in progress for 0x%04x%08x",
				(uint16_t)(target_needs_DAA->pid >> 32U),
				(uint32_t)(target_needs_DAA->pid & 0xFFFFFFFFU));
		}
	}

	if (DAA_entries_count) {
		_drv_pending_xfer_ctxt_init();
		pending_xfer_ctxt.xfer_type = XFER_TYPE_ENTDAA;
		pending_xfer_ctxt.xfer_sem = &data->xfer_sem;

		/* Start the DAA process */
		XEC_I3C_DO_DAA(dev, DAT_first_free_posn, DAA_entries_count,
			       &pending_xfer_ctxt.node[0].tid);

		if (k_sem_take(&data->xfer_sem, K_MSEC(DRV_RESP_WAIT_MS))) {
			XEC_I3C_Xfer_Reset(dev);
			ret = -EBUSY;
		} else if (pending_xfer_ctxt.xfer_status) {
			LOG_ERR("DAA status error - 0x%x", pending_xfer_ctxt.xfer_status);
			if (pending_xfer_ctxt.node[0].ret_data_len) {
				LOG_ERR("DAA remaining devices count - %d",
					pending_xfer_ctxt.node[0].ret_data_len);
				/* Not all devices in the static list (meant for DAA) are
				 * assigned addresses. This is an error condition?
				 */
			}
			ret = -EIO;
		}

		DAA_success_count = DAA_entries_count - pending_xfer_ctxt.node[0].ret_data_len;

		if (ret != -EBUSY) {
			/* DAA is successful (maybe partial), but devices may have different
			 * intended dynamic addresses due to arbitration.
			 * Need to update accordingly
			 */
			for (idx = 0; idx < DAA_success_count; idx++) {
				_drv_dct_info_init(&dct_info);
				XEC_I3C_DCT_read(dev, data->DCT_start_addr, idx, &dct_info);
				pid = dct_info.pid;

				const struct i3c_device_id i3c_id = I3C_DEVICE_ID(pid);
				const uint16_t vendor_id = (uint16_t)(pid >> 32U);
				const uint32_t part_no = (uint32_t)(pid & 0xFFFFFFFFU);
				struct i3c_device_desc *target = i3c_device_find(dev, &i3c_id);

				if (target == NULL) {
					LOG_DBG("%s: PID 0x%04x%08x is not in registered device "
						"list, given DA 0x%02x",
						dev->name, vendor_id, part_no,
						dct_info.dynamic_addr);
					/* This is probably an error condition ??
					 * what should we do?
					 */
					i3c_addr_slots_mark_i3c(
						&data->common.attached_dev.addr_slots,
						dct_info.dynamic_addr);
				} else {
					target->dynamic_addr = dct_info.dynamic_addr;
					target->bcr = dct_info.bcr;
					target->dcr = dct_info.dcr;
					_drv_i3c_targets_DAA_addr_update(data, pid,
									 dct_info.dynamic_addr,
									 DAT_first_free_posn + idx);
					LOG_DBG("%s: PID 0x%04x%08x assigned dynamic address "
						"0x%02x",
						dev->name, vendor_id, part_no,
						dct_info.dynamic_addr);
				}
			}
		}

		/* Need Review - should be (ret != -EBUSY)?? */
		_drv_i3c_targets_DAA_done(dev, ret != EBUSY,
					  DAT_first_free_posn + DAA_success_count - 1);
	}

exit_da:
	k_mutex_unlock(&data->xfer_lock);
	return ret;
}

/**
 * @brief Transfer messages in I3C mode
 *
 * @see _drv_i3c_xfers
 *
 * @param dev Pointer to controller device driver instance.
 * @param msgs Pointer to I3C messages.
 * @param response Pointer to xfer response
 * @return @see _drv_i3c_xfers
 */
static int _drv_i3c_xfers(const struct device *dev, struct i3c_msg *msgs, uint8_t num_msgs,
			  uint8_t tgt_addr, uint8_t *response)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *xec_data = dev->data;
	struct mec_i3c_ctx *hwctx = &xec_data->ctx;
	struct mec_i3c_XFER do_xfer_instance;
	uint8_t i = 0;
	int ret = 0;
	uint8_t DAT_idx; /* Index of the device in Device Address Table */

	hwctx->base = (mm_reg_t)config->regbase;
	*response = 0;

	memset(&do_xfer_instance, 0x00, sizeof(do_xfer_instance));

	_drv_pending_xfer_ctxt_init();
	pending_xfer_ctxt.xfer_type = XFER_TYPE_PVT_RW;
	pending_xfer_ctxt.xfer_sem = &xec_data->xfer_sem;

	for (i = 0; i < num_msgs; i++) {
		if (I3C_MSG_READ == (msgs[i].flags & I3C_MSG_RW_MASK)) {
			LOG_DBG("Read [%d] bytes from target [0x%02x]", msgs[i].len, tgt_addr);
			do_xfer_instance.cmds[i].read = true;
		} else {
			LOG_DBG("Send [%d] bytes to target [0x%02x]", msgs[i].len, tgt_addr);
			do_xfer_instance.cmds[i].read = false;
		}

		if (I3C_MSG_STOP == (msgs[i].flags & I3C_MSG_STOP)) {
			do_xfer_instance.cmds[i].stop = true;
		} else {
			do_xfer_instance.cmds[i].stop = false;
		}

		if (I3C_MSG_HDR == (msgs[i].flags & I3C_MSG_HDR)) {
			/* Only DDR supported */
			do_xfer_instance.cmds[i].xfer_speed = MEC_XFER_SPEED_HDR_DDR;
		} else {
			/* Use SDR0 for fast xfer */
			do_xfer_instance.cmds[i].xfer_speed = MEC_XFER_SPEED_SDR0;
		}

		do_xfer_instance.cmds[i].pec_en = false;

		DAT_idx = 0;
		if (_drv_i3c_DAT_idx_get(xec_data, tgt_addr, &DAT_idx)) {
			/* Unable to locate target in target list */
			ret = -EINVAL;
			break;
		}

		do_xfer_instance.cmds[i].tgt_idx = DAT_idx;
		do_xfer_instance.cmds[i].data_buf = msgs[i].buf;
		do_xfer_instance.cmds[i].data_len = msgs[i].len;

		pending_xfer_ctxt.node[i].read = do_xfer_instance.cmds[i].read;
		pending_xfer_ctxt.node[i].data_buf = do_xfer_instance.cmds[i].data_buf;

		XEC_I3C_DO_Xfer_Prep(dev, &do_xfer_instance.cmds[i],
				     &pending_xfer_ctxt.node[i].tid);
	}

	/* Tier1 change: round2: XEC_I3C_Thresholds_Response_buf_set - Set response buffer threshold
	 * (fixed underflow) */
	if (num_msgs > 0U) {
		uint8_t threshold = (uint8_t)(num_msgs - 1U);
		if (threshold < XEC_I3C_RESPONSE_BUF_DEPTH) {
			uint32_t msk = XEC_I3C_QT_CR_RBT_MSK;
			uint32_t val = XEC_I3C_QT_CR_RBT_SET((uint32_t)threshold);
			soc_mmcr_mask_set(config->regbase + XEC_I3C_QT_CR_OFS, val, msk);
		}
	}

	for (i = 0; i < num_msgs; i++) {
		XEC_I3C_DO_Xfer(dev, &do_xfer_instance.cmds[i]);
	}

	if (k_sem_take(&xec_data->xfer_sem, K_MSEC(DRV_RESP_WAIT_MS))) {
		ret = -EBUSY;
		XEC_I3C_Xfer_Reset(dev);
	} else {
		if (pending_xfer_ctxt.xfer_status) {
			*response = pending_xfer_ctxt.xfer_status;
			ret = -EIO;
		}
	}

	return ret;
}

/**
 * @brief Transfer messages in I3C mode
 *
 * @see i3c_xec_xfers
 *
 * @param dev Pointer to controller device driver instance.
 * @param msgs Pointer to I3C messages.
 *
 * @return @see i3c_xec_xfers
 */
static int i3c_xec_xfers(const struct device *dev, struct i3c_device_desc *target,
			 struct i3c_msg *msgs, uint8_t num_msgs)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *data = dev->data;
	mm_reg_t regbase = config->regbase;
	uint32_t nrxwords = 0, ntxwords = 0;
	uint8_t response = 0;
	int ret = 0, i = 0;

	/* Tier1 change: round2: Use inline helper - Check if controller is in master mode */
	if (false == xec_i3c_is_current_role_master(regbase)) {
		ret = -EACCES;
		goto exit_xfer;
	}

	if (num_msgs == 0) {
		ret = 0;
		goto exit_xfer;
	}

	if (0U == target->dynamic_addr) {
		ret = -EINVAL;
		goto exit_xfer;
	}

	if (num_msgs > data->fifo_depths.cmd_fifo_depth) {
		ret = -ENOTSUP;
		goto exit_xfer;
	}

	for (i = 0; i < num_msgs; i++) {
		if (I3C_MSG_READ == (msgs[i].flags & I3C_MSG_RW_MASK)) {
			nrxwords += DIV_ROUND_UP(msgs[i].len, 4);
		} else {
			ntxwords += DIV_ROUND_UP(msgs[i].len, 4);
		}
	}

	if (ntxwords > data->fifo_depths.tx_fifo_depth ||
	    nrxwords > data->fifo_depths.rx_fifo_depth) {
		ret = -ENOTSUP;
		goto exit_xfer;
	}

	k_mutex_lock(&data->xfer_lock, K_FOREVER);

	ret = _drv_i3c_xfers(dev, msgs, num_msgs, target->dynamic_addr, &response);

	k_mutex_unlock(&data->xfer_lock);

	if ((!ret) && response) { /* Error in Response */
		LOG_ERR("!!Error - 0x%08x - %d!!", response, ret);
	}

exit_xfer:
	return ret;
}

#ifdef CONFIG_I3C_USE_IBI
static int i3c_xec_ibi_enable(const struct device *dev, struct i3c_device_desc *target)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *data = dev->data;
	mm_reg_t regbase = config->regbase;
	struct i3c_ccc_events i3c_events;
	struct mec_i3c_IBI_SIR enable_ibi_instance;
	uint8_t DAT_idx = 0; /* Index of the device in Device Address Table */
	int ret = 0;

	/* Tier1 change: round2: Use inline helper - Check if controller is in master mode */
	if (false == xec_i3c_is_current_role_master(regbase)) {
		return -EACCES;
	}

	if (0U == target->dynamic_addr) {
		return -EINVAL;
	}

	if (!i3c_device_is_ibi_capable(target)) {
		return -EINVAL;
	}

	DAT_idx = 0;
	if (_drv_i3c_DAT_idx_get(data, target->dynamic_addr, &DAT_idx)) {
		/* Unable to locate target in target list */
		return -EINVAL;
	}

	LOG_DBG("%s: IBI enabling for 0x%02x (BCR 0x%02x)", dev->name, target->dynamic_addr,
		target->bcr);

	/* Tell target to enable IBI */
	i3c_events.events = I3C_CCC_EVT_INTR;
	ret = i3c_ccc_do_events_set(target, true, &i3c_events);
	if (ret != 0) {
		LOG_ERR("%s: Error sending IBI ENEC for 0x%02x (%d)", dev->name,
			target->dynamic_addr, ret);
		return ret;
	}

	enable_ibi_instance.DAT_start = data->DAT_start_addr;
	enable_ibi_instance.tgt_dat_idx = DAT_idx;
	enable_ibi_instance.ibi_has_payload = i3c_ibi_has_payload(target);

	XEC_I3C_IBI_SIR_Enable(dev, &enable_ibi_instance, !data->ibi_intr_enabled_init);

	return 0;
}

static int i3c_xec_ibi_disable(const struct device *dev, struct i3c_device_desc *target)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *data = dev->data;
	mm_reg_t regbase = config->regbase;
	struct i3c_ccc_events i3c_events;
	struct mec_i3c_IBI_SIR disable_ibi_instance;
	uint8_t DAT_idx = 0; /* Index of the device in Device Address Table */
	int ret = 0;

	/* Tier1 change: round2: Use inline helper - Check if controller is in master mode */
	if (false == xec_i3c_is_current_role_master(regbase)) {
		return -EACCES;
	}

	if (0U == target->dynamic_addr) {
		return -EINVAL;
	}

	if (!i3c_device_is_ibi_capable(target)) {
		return -EINVAL;
	}

	DAT_idx = 0;
	if (_drv_i3c_DAT_idx_get(data, target->dynamic_addr, &DAT_idx)) {
		/* Unable to locate target in target list */
		return -EINVAL;
	}

	LOG_DBG("%s: IBI disabling for 0x%02x (BCR 0x%02x)", dev->name, target->dynamic_addr,
		target->bcr);

	/* Tell target to enable IBI */
	i3c_events.events = I3C_CCC_EVT_INTR;
	ret = i3c_ccc_do_events_set(target, false, &i3c_events);
	if (ret != 0) {
		LOG_ERR("%s: Error sending IBI DISEC for 0x%02x (%d)", dev->name,
			target->dynamic_addr, ret);
		return ret;
	}

	disable_ibi_instance.DAT_start = data->DCT_start_addr;
	disable_ibi_instance.tgt_dat_idx = DAT_idx;
	disable_ibi_instance.ibi_has_payload = i3c_ibi_has_payload(target);

	XEC_I3C_IBI_SIR_Disable(dev, &disable_ibi_instance, !data->ibi_intr_enabled_init);

	return 0;
}

static int i3c_xec_target_ibi_raise(const struct device *dev, struct i3c_ibi *request)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *xec_data = dev->data;
	struct mec_i3c_ctx *hwctx = &xec_data->ctx;
	struct mec_i3c_raise_IBI_SIR ibi_sir_request;
	int ret = 0;

	hwctx->base = config->regbase;

	if (request == NULL) {
		return -EINVAL;
	}

	switch (request->ibi_type) {
	case I3C_IBI_TARGET_INTR:
		if ((0 == request->payload_len) || (request->payload_len > 5)) {
			LOG_ERR("%s: Invalid IBI SIR payload len (%d)", dev->name,
				request->payload_len);
			return -EINVAL;
		}

		k_mutex_lock(&xec_data->xfer_lock, K_FOREVER);

		ibi_sir_request.mdb = request->payload[0];
		ibi_sir_request.data_buf = &request->payload[1];
		ibi_sir_request.data_len = (request->payload_len - 1U);

		_drv_pending_xfer_ctxt_init();
		pending_xfer_ctxt.xfer_type = XFER_TYPE_TGT_RAISE_IBI;
		pending_xfer_ctxt.xfer_sem = &xec_data->xfer_sem;

		LOG_DBG("[%s] - Raise IBI SIR", __func__);

		XEC_I3C_TGT_IBI_SIR_Raise(dev, &ibi_sir_request);

		k_mutex_unlock(&xec_data->xfer_lock);

		if (k_sem_take(&xec_data->xfer_sem, K_MSEC(DRV_RESP_WAIT_MS))) {
			ret = -EBUSY;
			break;
		} else if (pending_xfer_ctxt.xfer_status) {
			LOG_ERR("!!TGT Raise IBI SIR Error - 0x%08x !!",
				pending_xfer_ctxt.xfer_status);
			ret = -EIO;
			break;
		}
		break;

	case I3C_IBI_CONTROLLER_ROLE_REQUEST:
		/* We need to wait to process all outstanding responses/data from the
		 * Response Queue / Rx-FIFO
		 */
		k_mutex_lock(&xec_data->xfer_lock, K_FOREVER);

		_drv_pending_xfer_ctxt_init();
		pending_xfer_ctxt.xfer_type = XFER_TYPE_TGT_RAISE_IBI_MR;
		pending_xfer_ctxt.xfer_sem = &xec_data->xfer_sem;

		LOG_DBG("[%s] - Raise IBI MR", __func__);

		XEC_I3C_TGT_IBI_MR_Raise(dev);

		k_mutex_unlock(&xec_data->xfer_lock);

		if (k_sem_take(&xec_data->xfer_sem, K_MSEC(DRV_RESP_WAIT_MS))) {
			ret = -EBUSY;
			break;
		} else if (pending_xfer_ctxt.xfer_status) {
			LOG_ERR("!!TGT Raise IBI MR Error - 0x%08x !!",
				pending_xfer_ctxt.xfer_status);
			ret = -EIO;
			break;
		}
		break;

	case I3C_IBI_HOTJOIN:
		return -ENOTSUP;
		/* return _drv_i3c_target_ibi_raise_hj(dev); */

	default:
		return -EINVAL;
	}

	return 0;
}

static int _drv_i3c_initiate_hotjoin(const struct device *dev)
{
	struct xec_i3c_data *data = dev->data;
	uint8_t target_info_idx = 0;
	uint8_t free_addr = 0;
	int ret = -1;

	free_addr = i3c_addr_slots_next_free_find(&data->common.attached_dev.addr_slots, 0);
	if (!free_addr) {
		LOG_ERR("%s: no free address available for hot join", dev->name);
		return ret;
	}

	if (_drv_i3c_targets_free_pos_get(data, &target_info_idx)) {
		/* Unable to find a free location in targets list */
		LOG_ERR("%s: no space in targets list for i3c device (hot join)", dev->name);
		return ret;
	}

	/* Initialize the target info node */
	data->targets[target_info_idx].state = TGT_STATE_NEEDS_DAA;
	data->targets[target_info_idx].address = free_addr;
	data->targets[target_info_idx].pid = 0;

	/* Now that we have created the target info node, proceed to DAA */
	if (i3c_xec_do_daa(dev)) {
		/* Unable to retrieve target PID */
		LOG_ERR("%s: DAA for hot join: fail", dev->name);
		return ret;
	}

	return 0;
}

/**
 * @brief IBI Work Queue Callback
 *
 * @param work pointer to k_work item
 */
static void i3c_xec_ibi_work_cb(struct k_work *work)
{
	struct i3c_ibi_work *i3c_ibi_work = CONTAINER_OF(work, struct i3c_ibi_work, work);
	const struct device *dev = i3c_ibi_work->controller;
	struct xec_i3c_data *xec_data = dev->data;
	struct i3c_device_desc *target = NULL;
	uint8_t idx = 0, ibi_addr = 0;

	for (idx = 0; idx < XEC_I3C_MAX_IBI_LIST_COUNT; idx++) {
		if (IBI_NODE_ISR_UPDATED == xec_data->ibis[idx].state) {
			if (I3C_IBI_TARGET_INTR == xec_data->ibis[idx].ibi_type) {
				ibi_addr = xec_data->ibis[idx].addr;
				target = i3c_dev_list_i3c_addr_find(dev, ibi_addr);
				if (target != NULL) {
					/* Inform the application with IBI Payload */
					(void)target->ibi_cb(target, &xec_data->ibis[idx].payload);
					/* Note: we are ignoring the return value from this
					 * callback because the hardware will automatically ACK the
					 * target which is expected to send an IBI
					 */
				} else {
					LOG_ERR("IBI SIR from unknown device %x", ibi_addr);
				}
			} else if (I3C_IBI_HOTJOIN == xec_data->ibis[idx].ibi_type) {
				LOG_DBG("Received HJ request");
				if (_drv_i3c_initiate_hotjoin(dev)) {
					LOG_ERR("unable to complete DAA for HJ request "
						"device 0x%x",
						ibi_addr);
				}
			} else {
				LOG_DBG("MR from device %x", ibi_addr);
			}
			xec_data->ibis[idx].state = IBI_NODE_STATE_FREE;
		}
	}
}

static void _drv_tgt_rx_handler(const struct device *dev)
{
	struct xec_i3c_data *xec_data = (struct xec_i3c_data *)dev->data;
	const struct i3c_target_callbacks *target_cbks;
	struct i3c_tgt_pvt_receive_node *tgt_rx_node;
	uint16_t i;
	uint8_t idx = 0;

	target_cbks = xec_data->target_config->callbacks;

	/* Tier1 change: round2: XEC_I3C_TGT_dyn_addr_get - Get target dynamic address from register
	 */
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	uint32_t da = sys_read32(regbase + XEC_I3C_DEV_ADDR_OFS);
	xec_data->target_config->address = (uint8_t)XEC_I3C_DEV_ADDR_DYA_GET(da);

	for (idx = 0; idx < XEC_I3C_MAX_TGT_RX_LIST_COUNT; idx++) {
		tgt_rx_node = &xec_data->tgt_pvt_rx[idx];
		if ((TGT_RX_NODE_ISR_UPDATED == tgt_rx_node->state) ||
		    (TGT_RX_NODE_ISR_UPDATED_THR == tgt_rx_node->state)) {
			if (!tgt_rx_node->error_status) {
				/* Inform the application of the received data */
				for (i = 0; i < tgt_rx_node->data_len; i++) {
					/* Note we are using only the write_received_cb to send all
					 * the data byte by byte as expected by the Zephyr Model.
					 * write_requested_cb which is used when write is initiated
					 * is not used as we are not supporting ACK/NACK based on
					 * application's decision.
					 */
					target_cbks->write_received_cb(xec_data->target_config,
								       tgt_rx_node->data_buf[i]);
				}
				if (TGT_RX_NODE_ISR_UPDATED == tgt_rx_node->state) {
					/* Inform the end of transaction */
					target_cbks->stop_cb(xec_data->target_config);
				}
			} else {
				LOG_ERR("Error status for Target Private Receive 0x%x",
					tgt_rx_node->error_status);
			}
			tgt_rx_node->state = TGT_RX_NODE_STATE_FREE;
		}
	}
}

static void _drv_tgt_tx_done_handler(const struct device *dev)
{
	struct xec_i3c_data *xec_data = (struct xec_i3c_data *)dev->data;

	xec_data->tgt_pvt_tx_sts = 0x0;
	xec_data->tgt_pvt_tx_rem_data_len = 0x0;

	/* Clear the tx queued flag to allow application to start another
	 * target tx
	 */
	xec_data->tgt_tx_queued = false;

	/* Keeping this function for possible enhancements later */
}

static bool _drv_i3c_ibi_isr(mm_reg_t regbase, struct xec_i3c_data *data)
{
	uint8_t num_ibis = 0;
	uint8_t i = 0;
	uint32_t ibi_sts = 0;
	uint8_t ibi_addr = 0;
	uint8_t ibi_datalen = 0;
	struct ibi_node *ibi_node_ptr = NULL;
	bool ibi_error = false;

	/* Tier1 change: round2: xec_i3c_ibi_status_count_get - Get IBI status count from queue
	 * status register */
	uint32_t qlvl = sys_read32(regbase + XEC_I3C_QL_SR_OFS);
	num_ibis = (uint8_t)XEC_I3C_QL_SR_IBC_GET(qlvl);

	for (i = 0; i < num_ibis; i++) {
		/* Tier1 change: round2: xec_i3c_ibi_queue_status_get - Get IBI queue status
		 * register value */
		ibi_sts = sys_read32(regbase + XEC_I3C_IBI_QUE_SR_OFS);

		ibi_datalen = IBI_QUEUE_STATUS_DATA_LEN(ibi_sts);
		ibi_addr = IBI_QUEUE_IBI_ADDR(ibi_sts);

		LOG_DBG("[%s] - ibi_sts = 0x%08x, ibi_addr = 0x%02x ibi_datalen = %d", __func__,
			ibi_sts, ibi_addr, ibi_datalen);

		ibi_node_ptr = _drv_i3c_free_ibi_node_get_isr(data);
		if (ibi_node_ptr) {
			if (ibi_datalen) {
				if (ibi_datalen <= CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE) {
					ibi_node_ptr->payload.payload_len = ibi_datalen;
					xec_i3c_ibi_data_read(regbase,
							      &ibi_node_ptr->payload.payload[0],
							      ibi_datalen);
				} else {
					LOG_ERR("IBI DataLen > MAX_IBI_PAYLOAD_LEN");
					ibi_error = true;
				}
			} else {
				ibi_node_ptr->payload.payload_len = 0;
				LOG_ERR("IBI DataLen 0");
			}

			if (IBI_TYPE_SIRQ(ibi_sts)) {
				LOG_DBG("SIRQ IBI received");
				ibi_node_ptr->ibi_type = I3C_IBI_TARGET_INTR;
			}
			if (IBI_TYPE_HJ(ibi_sts)) {
				LOG_DBG("HOT Join IBI received");
				ibi_node_ptr->ibi_type = I3C_IBI_HOTJOIN;
			}
			if (IBI_TYPE_MR(ibi_sts)) {
				LOG_DBG("MR IBI received");
				ibi_node_ptr->ibi_type = I3C_IBI_CONTROLLER_ROLE_REQUEST;
			}

			ibi_node_ptr->state = IBI_NODE_ISR_UPDATED;
			ibi_node_ptr->addr = ibi_addr;
			LOG_DBG("Node updated");
		} else {
			LOG_ERR("No free IBI nodes");
			ibi_error = true;
		}
	}

	if (ibi_error) {
		/* Drain the IBI Queue for this IBI */
		xec_i3c_ibi_data_read(regbase, NULL, IBI_QUEUE_STATUS_DATA_LEN(ibi_sts));
	}

	return ibi_error;
}
#endif

/**
 * @brief Find a registered I3C target device.
 *
 * This returns the I3C device descriptor of the I3C device
 * matching the incoming @p id.
 *
 * @param dev Pointer to controller device driver instance.
 * @param id Pointer to I3C device ID.
 *
 * @return @see i3c_device_find.
 */
static struct i3c_device_desc *i3c_xec_device_find(const struct device *dev,
						   const struct i3c_device_id *id)
{
	const struct xec_i3c_config *config = dev->config;

	return i3c_dev_list_find(&config->common.dev_list, id);
}

/**
 * @brief Writes to the Target's TX FIFO
 *
 * @param dev Pointer to the device structure for an I3C controller
 *            driver configured in target mode.
 * @param buf Pointer to the buffer
 * @param len Length of the buffer
 *
 * @retval Number of bytes written
 */
/* TODO API change. Added hdr_mode parameter */
static int i3c_xec_target_tx_write(const struct device *dev, uint8_t *buf, uint16_t len,
				   uint8_t hdr_mode)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *xec_data = dev->data;
	struct mec_i3c_ctx *hwctx = &xec_data->ctx;

	hwctx->base = config->regbase;

	if (xec_data->tgt_tx_queued) {
		LOG_DBG("Target TX is in progress");
		return -EBUSY;
	}

	xec_data->tgt_tx_queued = true;

	if (len > xec_data->i3c_cfg_as_tgt.max_write_len) {
		LOG_DBG("[%s] - Target write data len %d greater than SLV MAX WR LEN %d", __func__,
			len, xec_data->i3c_cfg_as_tgt.max_write_len);
		len = xec_data->i3c_cfg_as_tgt.max_write_len;
	}

	if (len > xec_data->fifo_depths.tx_fifo_depth) {
		len = 0;
		goto exit_tgt_xfer;
	}

	k_mutex_lock(&xec_data->xfer_lock, K_FOREVER);

	XEC_I3C_DO_TGT_Xfer(dev, buf, len);

	k_mutex_unlock(&xec_data->xfer_lock);

exit_tgt_xfer:
	return len;
}

/**
 * @brief Register itself as target (to the I3C Controller)
 *
 * This tells the controller to act as a target device
 * on the I3C bus.
 *
 * @param dev Pointer to target device driver instance.
 * @param cfg Config struct with functions and parameters used by the I3C driver
 * to send bus events
 *
 * @return @see i3c_device_find.
 */
static int i3c_xec_target_register(const struct device *dev, struct i3c_target_config *cfg)
{
	struct xec_i3c_data *data = dev->data;

	data->target_config = cfg;

	return 0;
}

/**
 * @brief Unregisters the provided config as Target device
 *
 * This tells the controller to stop acting as a target device
 * on the I3C bus.
 *
 * @param dev Pointer to target device driver instance.
 * @param cfg I3C target device configuration
 *
 */
static int i3c_xec_target_unregister(const struct device *dev, struct i3c_target_config *cfg)
{
	struct xec_i3c_data *data = dev->data;

	if (cfg == data->target_config) {
		data->target_config = NULL;
	}

	return 0;
}

/**
 * @brief Get I3C Configuration
 *
 * Retrieve current configuration of I3C controller
 *
 *
 * @param dev Pointer to controller device driver instance.
 * @param type Type of configuration parameters expected
 * @param config Pointer to the configuration parameters.
 *
 * @retval 0 If successful.
 * @retval -EIO General Input/Output errors.
 * @retval -ENOSYS If not implemented.
 */
static int i3c_xec_config_get(const struct device *dev, enum i3c_config_type type, void *config)
{
	struct xec_i3c_data *xec_data = dev->data;
	int ret = 0;

	if ((type != I3C_CONFIG_CONTROLLER) || (config == NULL)) {
		return -EINVAL;
	}

	(void)memcpy(config, &xec_data->common.ctrl_config, sizeof(xec_data->common.ctrl_config));

	return ret;
}

/**
 * @brief Configure I3C hardware.
 *
 * @param dev Pointer to controller device driver instance.
 * @param type Type of configuration parameters being passed
 *             in @p config.
 * @param config Pointer to the configuration parameters.
 *
 * @retval 0 If successful.
 * @retval -EINVAL Invalid parameters.
 */
static int i3c_xec_configure(const struct device *dev, enum i3c_config_type type, void *config)
{
	const struct xec_i3c_config *xec_config = dev->config;
	struct xec_i3c_data *xec_data = dev->data;
	mm_reg_t regbase = xec_config->regbase;
	uint32_t core_clock = xec_config->clock;
	struct i3c_config_controller *ctrl_cfg;
	struct i3c_config_target *tgt_cfg;

	if (type == I3C_CONFIG_TARGET) {
		/* Tier1 change: round2: Use inline helper - Check if controller is in master mode
		 */
		if (true == xec_i3c_is_current_role_master(regbase)) {
			return -EINVAL;
		}

		tgt_cfg = (struct i3c_config_target *)config;

		XEC_I3C_TGT_PID_set(dev, tgt_cfg->pid, tgt_cfg->pid_random);
	} else if (type == I3C_CONFIG_CONTROLLER) {
		/* Tier1 change: round2: Use inline helper - Check if controller is in master mode
		 */
		if (false == xec_i3c_is_current_role_master(regbase)) {
			return -EINVAL;
		}

		ctrl_cfg = (struct i3c_config_controller *)config;

		if ((ctrl_cfg->scl.i2c == 0U) || (ctrl_cfg->scl.i3c == 0U)) {
			return -EINVAL;
		}

		/* Save the config */
		(void)memcpy(&xec_data->common.ctrl_config, ctrl_cfg, sizeof(*ctrl_cfg));

		XEC_I3C_Controller_Clk_Cfg(dev, core_clock, xec_data->common.ctrl_config.scl.i3c);
	}

	return 0;
}

static void _drv_i3c_isr_xfers(const struct device *dev, uint16_t num_responses)
{
	const struct xec_i3c_config *config = dev->config;
	mm_reg_t rb = config->regbase;
	uint16_t data_len = 0;
	uint8_t resp_sts = 0;
	uint8_t tid = 0;

	/* Note: We are handling multiple responses only for chained private xfers */
	for (int i = 0; i < num_responses; i++) {
		/* Tier1 change: round2: xec_i3c_response_sts_get - Get response status and extract
		 * data length and TID */
		uint32_t response = sys_read32(rb + XEC_I3C_RESP_OFS);
		data_len = (uint16_t)(response & 0xffffu);
		tid = (uint8_t)((response & RESPONSE_TID_BITMASK) >> RESPONSE_TID_BITPOS);
		resp_sts =
			(uint8_t)((response & RESPONSE_ERR_STS_BITMASK) >> RESPONSE_ERR_STS_BITPOS);

		pending_xfer_ctxt.node[i].error_status = resp_sts;
		pending_xfer_ctxt.node[i].ret_data_len = data_len;

		LOG_DBG("[%s] - tid = %d, resp_sts = 0x%08x data_len = %d", __func__, tid, resp_sts,
			data_len);

		/* Ensure TID of response match pending transfer */
		if (tid == pending_xfer_ctxt.node[i].tid) {
			if ((!resp_sts) && data_len) { /* Read response bytes from Fifo */
				if (pending_xfer_ctxt.node[i].read) {
					LOG_DBG("[%s] - Reading [%d] bytes into [0x%08x]", __func__,
						data_len,
						(uint32_t)pending_xfer_ctxt.node[i].data_buf);
					xec_i3c_fifo_read(rb, pending_xfer_ctxt.node[i].data_buf,
							  data_len);
				} else {
					LOG_ERR("Read data encountered with no matching "
						"read request");
				}
			}
		} else {
			LOG_ERR("TID match error - need to investigate");
		}
	}

	pending_xfer_ctxt.xfer_status = 0;
	for (int i = 0; i < num_responses; i++) {
		switch (pending_xfer_ctxt.node[i].error_status) {
		case RESPONSE_ERROR_PARITY:
			LOG_ERR("RESPONSE_ERROR_PARITY");
			break;
		case RESPONSE_ERROR_IBA_NACK:
			LOG_ERR("RESPONSE_ERROR_IBA_NACK");
			break;
		case RESPONSE_ERROR_TRANSF_ABORT:
			LOG_ERR("RESPONSE_ERROR_TRANSF_ABORT");
			break;
		case RESPONSE_ERROR_CRC:
			LOG_ERR("RESPONSE_ERROR_CRC");
			break;
		case RESPONSE_ERROR_FRAME:
			LOG_ERR("RESPONSE_ERROR_FRAME");
			break;
		case RESPONSE_ERROR_OVER_UNDER_FLOW:
			LOG_ERR("RESPONSE_ERROR_OVER_UNDER_FLOW");
			break;
		case RESPONSE_ERROR_I2C_W_NACK_ERR:
			LOG_ERR("RESPONSE_ERROR_I2C_W_NACK_ERR");
			break;
		case RESPONSE_ERROR_ADDRESS_NACK:
			LOG_ERR("RESPONSE_ERROR_ADDRESS_NACK");
			break;
		case RESPONSE_NO_ERROR:
			__fallthrough;
		default:
			break;
		}

		if (pending_xfer_ctxt.node[i].error_status) {
			/* Mark as Transaction error */
			pending_xfer_ctxt.xfer_status = pending_xfer_ctxt.node[i].error_status;
			break;
		}
	} /* end for */

	if (pending_xfer_ctxt.xfer_status) { /* Error Handling */
		XEC_I3C_Xfer_Error_Resume(dev);
	}

	k_sem_give(pending_xfer_ctxt.xfer_sem);
}

static bool _drv_i3c_isr_target_xfers(const struct device *dev, uint16_t num_responses)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *data = dev->data;
	mm_reg_t rb = config->regbase;
	struct i3c_tgt_pvt_receive_node *tgt_rx_node;
	uint16_t data_len = 0;
	uint8_t resp_sts = 0;
	uint8_t tid = 0;
	bool notify_app = false;

	/*Note: We are expecting only one response in the ISR */
	for (int i = 0; i < num_responses; i++) {
		/* Tier1 change: round2: xec_i3c_tgt_response_sts_get - Get target response status
		 * with RX indication (fixed loop bug) */
		bool tgt_receive = false; /* Reset for each iteration */
		uint32_t response = sys_read32(rb + XEC_I3C_RESP_OFS);
		data_len = (uint16_t)(response & 0xFFFFu);
		tid = (uint8_t)((response & RESPONSE_TID_TGT_BITMASK) >> RESPONSE_TID_BITPOS);
		if (((response & RESPONSE_RX_RESP_BITMASK) >> RESPONSE_RX_RESP_BITPOS) != 0) {
			tgt_receive = true;
		}
		resp_sts =
			(uint8_t)((response & RESPONSE_ERR_STS_BITMASK) >> RESPONSE_ERR_STS_BITPOS);

		LOG_DBG("[%s] - tid = %d, resp_sts = 0x%08x data_len = %d", __func__, tid, resp_sts,
			data_len);

		if (tgt_receive) {
			if (tid == RESPONSE_TID_DEFTGTS) { /* Response for DEFSLVS */
				if (data_len <= data->DAT_depth) {
					LOG_DBG("[%s] - DEFSLVS response: no of targets %d",
						__func__, data_len);
					XEC_I3C_TGT_DEFTGTS_DAT_write(dev, data->DCT_start_addr,
								      data->DAT_start_addr,
								      data_len);
				} else {
					LOG_DBG("[%s] - DEFSLVS response: no of targets "
						"%d > DAT Depth %d",
						__func__, data_len, data->DAT_depth);
				}
			} else { /* Private Receive Transfer - Controller Write */
				tgt_rx_node = _drv_i3c_free_tgt_rx_node_get_isr(data, false);
				if (tgt_rx_node) {
					tgt_rx_node->error_status = resp_sts;
					tgt_rx_node->data_len = data_len;

					if (data_len > data->i3c_cfg_as_tgt.max_read_len) {
						LOG_DBG("[%s] - Received data len %d greater than "
							"SLV MAX RD LEN %d",
							__func__, data_len,
							data->i3c_cfg_as_tgt.max_read_len);
					}

					/* Read response bytes from Fifo */
					if ((!resp_sts) && data_len) {
						LOG_DBG("[%s] - Reading [%d] bytes into [0x%08x]",
							__func__, data_len,
							(uint32_t)tgt_rx_node->data_buf);
						xec_i3c_fifo_read(rb, tgt_rx_node->data_buf,
								  data_len);
					}

					tgt_rx_node->state = TGT_RX_NODE_ISR_UPDATED;
					notify_app = true;
					LOG_DBG("Node updated");

					if (resp_sts) {
						XEC_I3C_TGT_Error_Recovery(dev, resp_sts);
						/* Controller is expected to issue GETSTATUS CCC
						 * to clear error status from CCC_DEVICE_STATUS
						 * register.
						 */
						break; /* break out of the for loop */
					}
				} else {
					LOG_ERR("Target RX Node Unavailable");
				}
			}
		} else { /* Private Write Transfer - Controller Read */
			data->tgt_pvt_tx_rem_data_len = data_len;
			data->tgt_pvt_tx_sts = resp_sts;

			/* Prepare for next Target TX */
			_drv_tgt_tx_done_handler(dev);

			if ((resp_sts) || (data_len != 0)) {
				XEC_I3C_TGT_Error_Recovery(dev, resp_sts);
				/* Controller is expected to issue GETSTATUS CCC to clear
				 * error status from CCC_DEVICE_STATUS register
				 */
				break; /* break out of the for loop */
			}
		}
	}

	return notify_app;
}

static bool _drv_i3c_isr_target(const struct device *dev, uint32_t intr_sts)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *data = dev->data;
	mm_reg_t rb = config->regbase;
	uint8_t ibi_sir_rem_datalen = 0;
	uint16_t num_responses = 0;
	bool notify_app = false;

	/* Tier1 change: round2: xec_i3c_resp_buf_level_get - Get response buffer level from queue
	 * status register */
	uint32_t qlvl = sys_read32(rb + XEC_I3C_QL_SR_OFS);
	num_responses = (uint8_t)XEC_I3C_QL_SR_RBL_GET(qlvl);

	LOG_DBG("[%s] - num_responses = %d", __func__, num_responses);

	if (num_responses) {
		notify_app = _drv_i3c_isr_target_xfers(dev, num_responses);
	}

	if (intr_sts & BIT(XEC_I3C_ISR_IBI_UPD_POS)) {
		LOG_DBG("[%s] IBI updated status", __func__);
		/* Ensure there is corresponding pending context */
		if ((XFER_TYPE_TGT_RAISE_IBI == pending_xfer_ctxt.xfer_type) ||
		    (XFER_TYPE_TGT_RAISE_IBI_MR == pending_xfer_ctxt.xfer_type)) {
			pending_xfer_ctxt.xfer_status = 1;

			/* Tier1 change: round2: xec_i3c_tgt_ibi_resp_get - Get target IBI response
			 * status */
			uint32_t resp = sys_read32(rb + XEC_I3C_SC_TIBI_RESP_OFS);
			ibi_sir_rem_datalen = XEC_I3C_SC_TIBI_RESP_LEN_GET(resp);
			if (XEC_I3C_SC_TIBI_RESP_STS_GET(resp) == XEC_I3C_SC_TIBI_RESP_STS_ACK) {
				pending_xfer_ctxt.xfer_status = 0;
			} else {
				LOG_DBG("[%s] Target Raise IBI SIR error, ibi_sir_rem_dlen = %d",
					__func__, ibi_sir_rem_datalen);
			}

			/* Error Handling */
			if (pending_xfer_ctxt.xfer_status && ibi_sir_rem_datalen) {
				LOG_DBG("[%s] Handle Target Raise IBI SIR Residual data", __func__);
				XEC_I3C_TGT_IBI_SIR_Residual_handle(dev);
			}

			if (XFER_TYPE_TGT_RAISE_IBI == pending_xfer_ctxt.xfer_type) {
				k_sem_give(pending_xfer_ctxt.xfer_sem);
			} else if ((XFER_TYPE_TGT_RAISE_IBI_MR == pending_xfer_ctxt.xfer_type) &&
				   pending_xfer_ctxt.xfer_status) {
				k_sem_give(pending_xfer_ctxt.xfer_sem);
			}
		} else {
			LOG_DBG("[%s] IBI Updated Sts without raising IBI ??", __func__);
		}
	}

	if (intr_sts & BIT(XEC_I3C_ISR_CCC_UPD_POS)) {
		LOG_DBG("[%s] CCC updated by master", __func__);
		/* Check and update MRL, MWL */
		XEC_I3C_Target_MRL_MWL_update(dev, &data->i3c_cfg_as_tgt.max_read_len,
					      &data->i3c_cfg_as_tgt.max_write_len);
	}

	if (intr_sts & BIT(XEC_I3C_ISR_DYNA_POS)) {
		/* Tier1 change: round2: XEC_I3C_TGT_is_dyn_addr_valid - Check if target dynamic
		 * address is valid */
		if (sys_test_bit(rb + XEC_I3C_DEV_ADDR_OFS, XEC_I2C_DEV_ADDR_DYAV_POS) != 0) {
			LOG_DBG("[%s] DA assigned by master", __func__);
		} else {
			LOG_DBG("[%s] DA reset by master", __func__);
		}
	}

	if (intr_sts & BIT(XEC_I3C_ISR_DEFTR_POS)) {
		LOG_DBG("[%s] DEFSLV CCC sent by master", __func__);
	}

	if (intr_sts & BIT(XEC_I3C_ISR_RRR_POS)) {
		LOG_DBG("[%s] READ_REQ_RECV_STS No valid command in command Q", __func__);
	}

	if (intr_sts & BIT(XEC_I3C_ISR_BUS_OUPD_POS)) {
		LOG_DBG("[%s] TGT: Bus owner was changed", __func__);
		/* Bus Owner has changed; flush all fifos and queues and program resume bit */
		XEC_I3C_TGT_RoleSwitch_Resume(dev);

		/* Ensure there is corresponding pending context to inform the raise IBI API */
		if ((XFER_TYPE_TGT_RAISE_IBI_MR == pending_xfer_ctxt.xfer_type) &&
		    (!pending_xfer_ctxt.xfer_status)) {
			k_sem_give(pending_xfer_ctxt.xfer_sem);
		}
	}

	return notify_app;
}

static void _drv_i3c_isr_controller(const struct device *dev, uint32_t intr_sts)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *data = dev->data;
	mm_reg_t rb = config->regbase;
	uint16_t num_responses = 0;

	/* Tier1 change: round2: xec_i3c_resp_buf_level_get - Get response buffer level from queue
	 * status register */
	uint32_t qlvl = sys_read32(rb + XEC_I3C_QL_SR_OFS);
	num_responses = (uint8_t)XEC_I3C_QL_SR_RBL_GET(qlvl);

	/* LOG_DBG("[%s] - num_responses = %d", __func__, num_responses); */

	if (num_responses) {
		_drv_i3c_isr_xfers(dev, num_responses);
	}

#ifdef CONFIG_I3C_USE_IBI
	if (intr_sts & BIT(XEC_I3C_ISR_IBI_THLD_POS)) {
		if (false != _drv_i3c_ibi_isr(rb, data)) {
			LOG_ERR("[%s] - Error handling IBI", __func__);
		} else {
			LOG_DBG("[%s] - Schedule IBI Task", __func__);
			i3c_ibi_work_enqueue_cb(dev, i3c_xec_ibi_work_cb);
		}
	}
#endif

	if (intr_sts & BIT(XEC_I3C_ISR_BUS_OUPD_POS)) {
		LOG_DBG("[%s] CNTRLR: Bus owner was changed", __func__);
	}
}

/**
 * @brief Interrupt Service Routine
 *
 * @see i3c_xec_isr
 *
 * @param dev Pointer to controller device driver instance.
 * @param payload Pointer to CCC payload.
 *
 * @return @see i3c_do_ccc
 */
static void i3c_xec_isr(const struct device *dev)
{
	const struct xec_i3c_config *config = dev->config;
	mm_reg_t rb = config->regbase;
	uint32_t intr_sts = 0;
	bool notify_app = false;

	intr_sts = sys_read32(rb + XEC_I3C_INTR_SR_OFS);

	/* Tier1 change: round2: Use inline helper - Check if controller is in master mode */
	if (false == xec_i3c_is_current_role_master(rb)) {
		notify_app = _drv_i3c_isr_target(dev, intr_sts);
		if (notify_app) {
			_drv_tgt_rx_handler(dev);
		}
	} else {
		_drv_i3c_isr_controller(dev, intr_sts);
	}

	sys_write32(intr_sts, rb + XEC_I3C_INTR_SR_OFS);

	/* Tier1 change: round2: XEC_I3C_GIRQ_Status_Clr - Clear GIRQ interrupt status */
	soc_ecia_girq_status_clear(config->hwctx.girq, config->hwctx.girq_pos);
}

/**
 * @brief Initialize the hardware.
 *
 * @param dev Pointer to controller device driver instance.
 */
static int i3c_xec_init(const struct device *dev)
{
	const struct xec_i3c_config *config = dev->config;
	struct xec_i3c_data *data = dev->data;
	mm_reg_t regbase = config->regbase;
	struct i3c_config_controller *ctrl_config = &data->common.ctrl_config;
	int i3c_bus_mode = I3C_BUS_MODE_PURE;
	uint32_t core_clock = config->clock;
	int i = 0, ret = 0;
	uint8_t enable_config = 0;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("XEC I3C pinctrl init failed (%d)", ret);
		return ret;
	}

	/* Soft reset before configuration */
	XEC_I3C_Soft_Reset(dev);

	ctrl_config->is_secondary = false;

	/* Tier1 change: round2: Use inline helper - Check if controller is configured as primary */
	if (!xec_i3c_is_current_role_primary(regbase)) {
		ctrl_config->is_secondary = true;
	}

	if (ctrl_config->is_secondary) {
		XEC_I3C_Target_Init(dev, core_clock, &data->i3c_cfg_as_tgt.max_read_len,
				    &data->i3c_cfg_as_tgt.max_write_len);
		data->tgt_tx_queued = false;
	} else {
		switch (i3c_bus_mode) {
		case I3C_BUS_MODE_MIXED_FAST:
		case I3C_BUS_MODE_MIXED_LIMITED:
			XEC_I3C_Controller_Clk_I2C_Init(dev, core_clock);
			__fallthrough;
		case I3C_BUS_MODE_PURE:
			XEC_I3C_Controller_Clk_Init(dev, core_clock,
						    data->common.ctrl_config.scl.i3c);
			break;
		default:
			return -EINVAL;
		}
	}

	/* Create Semaphore for synchronization with ISR -
	 * Initial count is set to 0 so that we block when we first Take
	 */
	if (k_sem_init(&data->xfer_sem, 0, 1)) {
		/* Semaphore creation error */
		return -EIO;
	}

	/* Create mutex for thread synchronization */
	if (k_mutex_init(&data->xfer_lock)) {
		/* Mutex creation error */
		return -EIO;
	}

	if (ctrl_config->is_secondary) {
		XEC_I3C_Sec_Host_Config(dev);
	} else {
		XEC_I3C_Host_Config(dev);
	}

	/* Initialize the Queues and FIFO thresholds */
	XEC_I3C_Thresholds_Init(dev);

	if (ctrl_config->is_secondary) {
		/* Enable the i3c target interrupts */
		XEC_I3C_Target_Interrupts_Init(dev);
	} else {
		/* Enable the i3c controller interrupts */
		XEC_I3C_Controller_Interrupts_Init(dev);
	}

	if (config->irq_config_func) {
		config->irq_config_func();
	}

	enable_config = sbit_CONFG_ENABLE;
	if (ctrl_config->is_secondary) {
		enable_config |= sbit_MODE_TARGET;
	}

#ifndef CONFIG_I3C_USE_IBI
	enable_config |= sbit_HOTJOIN_DISABLE;
#endif

	XEC_I3C_Enable(dev, config->address, enable_config);

#ifdef CONFIG_I3C_USE_IBI
	data->ibi_intr_enabled_init = false;
	if (!(ctrl_config->is_secondary)) {
		data->ibi_intr_enabled_init = true;
	}
#endif

	XEC_I3C_queue_depths_get(dev, &data->fifo_depths);

	data->DAT_start_addr = 0;
	data->DAT_depth = 0;

	/* Tier1 change: round2: XEC_I3C_DAT_info_get - Get Device Address Table information from
	 * DAT pointer register */
	uint32_t dat_val = sys_read32(regbase + XEC_I3C_DAT_PTR_OFS);
	data->DAT_start_addr = XEC_I3C_DAT_PTR_STA_GET(dat_val);
	data->DAT_depth = XEC_I3C_DAT_PTR_DEPTH_GET(dat_val);

	data->DCT_start_addr = 0;
	data->DCT_depth = 0;

	/* Tier1 change: round2: XEC_I3C_DCT_info_get - Get Device Characteristic Table information
	 * from DCT pointer register */
	uint32_t dct_val = sys_read32(regbase + XEC_I3C_DCT_PTR_OFS);
	data->DCT_start_addr = XEC_I3C_DCT_PTR_STA_GET(dct_val);
	data->DCT_depth = XEC_I3C_DCT_PTR_DEPTH_GET(dct_val);

	for (i = 0; i < XEC_I3C_MAX_TARGETS; i++) {
		data->targets[i].state = TGT_STATE_NOT_PRESENT;
	}

#ifdef CONFIG_I3C_USE_IBI
	for (i = 0; i < XEC_I3C_MAX_IBI_LIST_COUNT; i++) {
		data->ibis[i].state = IBI_NODE_STATE_FREE;
	}
#endif

	/* Create bitmask of available positions in DAT */
	data->DAT_free_positions = GENMASK(data->DAT_depth - 1, 0);

	if (ctrl_config->is_secondary) {
		/* Call only for Target mode */
		i3c_xec_configure(dev, I3C_CONFIG_TARGET, &data->i3c_cfg_as_tgt);
	} else {
		ret = i3c_addr_slots_init(dev);
		if (ret != 0) {
			return ret;
		}

		/* Perform bus initialization */
		ret = i3c_bus_init(dev, &config->common.dev_list);
	}

	return ret;
}

static const struct i3c_driver_api i3c_xec_driver_api = {
	.configure = i3c_xec_configure,
	.config_get = i3c_xec_config_get,
	.attach_i3c_device = i3c_xec_attach_device,
	.reattach_i3c_device = i3c_xec_reattach_device,
	.detach_i3c_device = i3c_xec_detach_device,
	.do_daa = i3c_xec_do_daa,
	.do_ccc = i3c_xec_do_ccc,
	.i3c_device_find = i3c_xec_device_find,
	.i3c_xfers = i3c_xec_xfers,
	.target_tx_write = i3c_xec_target_tx_write,
	.target_register = i3c_xec_target_register,
	.target_unregister = i3c_xec_target_unregister,
#ifdef CONFIG_I3C_USE_IBI
	.ibi_enable = i3c_xec_ibi_enable,
	.ibi_disable = i3c_xec_ibi_disable,
	.ibi_raise = i3c_xec_target_ibi_raise,
#endif
};

#define XEC_I3C_GIRQ_DT(inst, idx)     MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, idx))
#define XEC_I3C_GIRQ_POS_DT(inst, idx) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, idx))

#define XEC_DT_I3C_HWCTX(id)                                                                       \
	{                                                                                          \
		.base = (mm_reg_t)DT_INST_REG_ADDR(id),                                            \
		.pcr_scr = (uint16_t)DT_INST_PROP(id, pcr_scr),                                    \
		.girq = (uint8_t)XEC_I3C_GIRQ_DT(id, 0),                                           \
		.girq_pos = (uint8_t)XEC_I3C_GIRQ_POS_DT(id, 0),                                   \
		.girq_wk_only = (uint8_t)XEC_I3C_GIRQ_DT(id, 1),                                   \
		.girq_pos_wk_only = (uint8_t)XEC_I3C_GIRQ_POS_DT(id, 1),                           \
	}

#define READ_PID_FROM_DTS(id)                                                                      \
	(((uint64_t)DT_PROP_BY_IDX(id, i3c1_as_tgt_pid, 1) << 32) |                                \
	 DT_PROP_BY_IDX(id, i3c1_as_tgt_pid, 2))

#define I3C_MCHP_XEC_DEVICE(id)                                                                    \
	static void i3c_xec_irq_config_func_##id(void)                                             \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(id), DT_INST_IRQ(id, priority), i3c_xec_isr,              \
			    DEVICE_DT_INST_GET(id), 0);                                            \
		irq_enable(DT_INST_IRQN(id));                                                      \
	}                                                                                          \
	PINCTRL_DT_INST_DEFINE(id);                                                                \
	static struct i3c_device_desc xec_i3c_device_list_##id[] = I3C_DEVICE_ARRAY_DT_INST(id);   \
                                                                                                   \
	static void i3c_xec_irq_config_func_##id(void);                                            \
                                                                                                   \
	static struct i3c_i2c_device_desc xec_i3c_i2c_device_list_##id[] =                         \
		I3C_I2C_DEVICE_ARRAY_DT_INST(id);                                                  \
                                                                                                   \
	static const struct xec_i3c_config xec_i3c_config_##id = {                                 \
		.regbase = (mm_reg_t)DT_INST_REG_ADDR(id),                                         \
		.clock = DT_INST_PROP(id, input_clock_frequency),                                  \
		.common.dev_list.i3c = xec_i3c_device_list_##id,                                   \
		.common.dev_list.num_i3c = ARRAY_SIZE(xec_i3c_device_list_##id),                   \
		.common.dev_list.i2c = xec_i3c_i2c_device_list_##id,                               \
		.common.dev_list.num_i2c = ARRAY_SIZE(xec_i3c_i2c_device_list_##id),               \
		.irq_config_func = i3c_xec_irq_config_func_##id,                                   \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(id),                                        \
		.hwctx = XEC_DT_I3C_HWCTX(id),                                                     \
	};                                                                                         \
	static struct xec_i3c_data i3c_data_##id = {                                               \
		.common.ctrl_config.scl.i3c = DT_INST_PROP_OR(id, i3c_scl_hz, 0),                  \
		.common.ctrl_config.scl.i2c = DT_INST_PROP_OR(id, i2c_scl_hz, 0),                  \
		.i3c_cfg_as_tgt.static_addr = DT_INST_PROP_OR(id, i3c1_as_tgt_static_addr, 0),     \
		.i3c_cfg_as_tgt.max_read_len = DT_INST_PROP_OR(id, i3c1_as_tgt_mrl, 8),            \
		.i3c_cfg_as_tgt.max_write_len = DT_INST_PROP_OR(id, i3c1_as_tgt_mwl, 8),           \
		.i3c_cfg_as_tgt.pid_random = DT_INST_PROP_OR(id, i3c1_as_tgt_pid_random, 0),       \
		.i3c_cfg_as_tgt.pid = COND_CODE_1(DT_PROP(id, i3c1_as_tgt_pid),                    \
                          READ_PID_FROM_DTS(id),                           \
                          (0xB0123456789B)) };             \
	DEVICE_DT_INST_DEFINE(id, i3c_xec_init, NULL, &i3c_data_##id, &xec_i3c_config_##id,        \
			      POST_KERNEL, CONFIG_I3C_CONTROLLER_INIT_PRIORITY,                    \
			      &i3c_xec_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I3C_MCHP_XEC_DEVICE)
