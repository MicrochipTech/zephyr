/*
 * Copyright 2026 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/arch/cpu.h>
#include "i3c_mchp_xec_priv.h"

#define MEC_DIV_ROUND_UP(n, d) (((uint32_t)(n) + ((uint32_t)(d) / 2)) / (uint32_t)(d))

/* MEC I3C low level register routines */
uint32_t xec_i3c_intr_sts_get(mm_reg_t regbase)
{
	return sys_read32(regbase + XEC_I3C_INTR_SR_OFS);
}

static void xec_i3c_intr_sts_clear(mm_reg_t regbase, uint32_t mask)
{
	sys_write32(mask, regbase + XEC_I3C_INTR_SR_OFS);
}

static void xec_i3c_intr_sts_enable(mm_reg_t regbase, uint32_t mask)
{
	sys_write32(mask, regbase + XEC_I3C_INTR_EN_OFS);
}

static void xec_i3c_intr_IBI_enable(mm_reg_t regbase)
{
	sys_set_bit(regbase + XEC_I3C_INTR_EN_OFS, XEC_I3C_ISR_IBI_THLD_POS);
	sys_set_bit(regbase + XEC_I3C_INTR_SIG_EN_OFS, XEC_I3C_ISR_IBI_THLD_POS);
}

static void xec_i3c_intr_IBI_disable(mm_reg_t regbase)
{
	sys_clear_bit(regbase + XEC_I3C_INTR_EN_OFS, XEC_I3C_ISR_IBI_THLD_POS);
	sys_clear_bit(regbase + XEC_I3C_INTR_SIG_EN_OFS, XEC_I3C_ISR_IBI_THLD_POS);
}

void xec_i3c_intr_thresholds_tx_enable(mm_reg_t regbase)
{
	sys_set_bit(regbase + XEC_I3C_INTR_EN_OFS, XEC_I3C_ISR_TX_THLD_POS);
	sys_set_bit(regbase + XEC_I3C_INTR_SIG_EN_OFS, XEC_I3C_ISR_TX_THLD_POS);
}

#if 0 /* currently UNUSED */
static void xec_i3c_intr_thresholds_tx_disable(mm_reg_t regbase)
{
	sys_clear_bit(regbase + XEC_I3C_INTR_EN_OFS, XEC_I3C_ISR_TX_THLD_POS);
	sys_clear_bit(regbase + XEC_I3C_INTR_SIG_EN_OFS, XEC_I3C_ISR_TX_THLD_POS);
}

static void xec_i3c_intr_thresholds_rx_enable(mm_reg_t regbase)
{
	sys_set_bit(regbase + XEC_I3C_INTR_EN_OFS, XEC_I3C_ISR_RX_THLD_POS);
	sys_set_bit(regbase + XEC_I3C_INTR_SIG_EN_OFS, XEC_I3C_ISR_RX_THLD_POS);
}

static void xec_i3c_intr_thresholds_rx_disable(mm_reg_t regbase)
{
	sys_clear_bit(regbase + XEC_I3C_INTR_EN_OFS, XEC_I3C_ISR_RX_THLD_POS);
	sys_clear_bit(regbase + XEC_I3C_INTR_SIG_EN_OFS, XEC_I3C_ISR_RX_THLD_POS);
}
#endif

static void xec_i3c_intr_sgnl_enable(mm_reg_t regbase, uint32_t mask)
{
	sys_write32(mask, regbase + XEC_I3C_INTR_SIG_EN_OFS);
}

static void xec_i3c_resp_queue_threshold_set(mm_reg_t regbase, uint8_t threshold)
{
	if (threshold < XEC_I3C_RESPONSE_BUF_DEPTH) {
		uint32_t msk = XEC_I3C_QT_CR_RBT_MSK;
		uint32_t val = XEC_I3C_QT_CR_RBT_SET((uint32_t)threshold);

		soc_mmcr_mask_set(regbase + XEC_I3C_QT_CR_OFS, val, msk);
	}
}

static void xec_i3c_cmd_queue_threshold_set(mm_reg_t regbase, uint32_t val)
{
	uint32_t mask = XEC_I3C_QT_CR_CEBT_MSK;
	uint32_t rval = XEC_I3C_QT_CR_CEBT_SET(val);

	soc_mmcr_mask_set(regbase + XEC_I3C_QT_CR_OFS, rval, mask);
}

static void xec_i3c_ibi_data_threshold_set(mm_reg_t regbase, uint32_t val)
{
	uint32_t mask = XEC_I3C_QT_CR_IDT_MSK;
	uint32_t rval = XEC_I3C_QT_CR_IDT_SET(val);

	soc_mmcr_mask_set(regbase + XEC_I3C_QT_CR_OFS, rval, mask);
}

static void xec_i3c_ibi_status_threshold_set(mm_reg_t regbase, uint32_t val)
{
	uint32_t mask = XEC_I3C_QT_CR_IST_MSK;
	uint32_t rval = XEC_I3C_QT_CR_IST_SET(val);

	soc_mmcr_mask_set(regbase + XEC_I3C_QT_CR_OFS, rval, mask);
}

static void xec_i3c_tx_buf_threshold_set(mm_reg_t regbase, uint32_t val)
{
	uint32_t mask = XEC_I3C_DBT_CR_TXEB_MSK;
	uint32_t rval = XEC_I3C_DBT_CR_TXEB_SET(val);

	soc_mmcr_mask_set(regbase + XEC_I3C_DBT_CR_OFS, rval, mask);
}

static void xec_i3c_rx_buf_threshold_set(mm_reg_t regbase, uint32_t val)
{
	uint32_t mask = XEC_I3C_DBT_CR_RXB_MSK;
	uint32_t rval = XEC_I3C_DBT_CR_RXB_SET(val);

	soc_mmcr_mask_set(regbase + XEC_I3C_DBT_CR_OFS, rval, mask);
}

static void xec_i3c_tx_start_threshold_set(mm_reg_t regbase, uint32_t val)
{
	uint32_t mask = XEC_I3C_DBT_CR_TXST_MSK;
	uint32_t rval = XEC_I3C_DBT_CR_TXST_SET(val);

	soc_mmcr_mask_set(regbase + XEC_I3C_DBT_CR_OFS, rval, mask);
}

static void xec_i3c_rx_start_threshold_set(mm_reg_t regbase, uint32_t val)
{
	uint32_t mask = XEC_I3C_DBT_CR_RXST_MSK;
	uint32_t rval = XEC_I3C_DBT_CR_RXST_SET(val);

	soc_mmcr_mask_set(regbase + XEC_I3C_DBT_CR_OFS, rval, mask);
}

/* TODO bool is a bad type to use as a value because it assumes the compiler will use
 * 0(false) or 1(true). True is actually ANY non-zero integer value.
 * We changed the code to check bool true/false and set/clear the bit.
 */
static void xec_i3c_notify_sir_reject(mm_reg_t regbase, bool opt)
{
	if (opt == true) {
		sys_set_bit(regbase + XEC_I3C_IBI_QUE_CR_OFS, XEC_I3C_IBI_QUE_CR_NR_TIRC_POS);
	} else {
		sys_clear_bit(regbase + XEC_I3C_IBI_QUE_CR_OFS, XEC_I3C_IBI_QUE_CR_NR_TIRC_POS);
	}
}

static void xec_i3c_notify_mr_reject(mm_reg_t regbase, bool opt)
{
	if (opt == true) {
		sys_set_bit(regbase + XEC_I3C_IBI_QUE_CR_OFS, XEC_I3C_IBI_QUE_CR_NR_HRC_POS);
	} else {
		sys_clear_bit(regbase + XEC_I3C_IBI_QUE_CR_OFS, XEC_I3C_IBI_QUE_CR_NR_HRC_POS);
	}
}

static void xec_i3c_notify_hj_reject(mm_reg_t regbase, bool opt)
{
	if (opt == true) {
		sys_set_bit(regbase + XEC_I3C_IBI_QUE_CR_OFS, XEC_I3C_IBI_QUE_CR_NR_HJ_POS);
	} else {
		sys_clear_bit(regbase + XEC_I3C_IBI_QUE_CR_OFS, XEC_I3C_IBI_QUE_CR_NR_HJ_POS);
	}
}

static void xec_i3c_dynamic_addr_set(mm_reg_t regbase, uint8_t address)
{
	uint32_t mask = XEC_I3C_DEV_ADDR_DYA_MSK | BIT(XEC_I2C_DEV_ADDR_DYAV_POS);
	uint32_t rval =
		(XEC_I3C_DEV_ADDR_DYA_SET((uint32_t)address) | BIT(XEC_I2C_DEV_ADDR_DYAV_POS));

	soc_mmcr_mask_set(regbase + XEC_I3C_DEV_ADDR_OFS, rval, mask);
}

static void xec_i3c_static_addr_set(mm_reg_t regbase, uint8_t address)
{
	uint32_t mask = XEC_I3C_DEV_ADDR_STA_MSK | BIT(XEC_I3C_DEV_ADDR_STAV_POS);
	uint32_t rval =
		(XEC_I3C_DEV_ADDR_STA_SET((uint32_t)address) | BIT(XEC_I3C_DEV_ADDR_STAV_POS));

	soc_mmcr_mask_set(regbase + XEC_I3C_DEV_ADDR_OFS, rval, mask);
}

/* Set the operation mode of the controller: 0=Controller, 1=Target */
void xec_i3c_operation_mode_set(mm_reg_t regbase, uint8_t mode)
{
	uint32_t mask = XEC_I3C_DEV_EXT_CR_OPM_MSK;
	uint32_t val = XEC_I3C_DEV_EXT_CR_OPM_HC;

	if (mode != XEC_I3C_OP_MODE_CTL) { /* Target controller mode? */
		val = XEC_I3C_DEV_EXT_CR_OPM_SC;
	}

	val = XEC_I3C_DEV_EXT_CR_OPM_SET(val);

	soc_mmcr_mask_set(regbase + XEC_I3C_DEV_EXT_CR_OFS, val, mask);
}

/* Enable Controller(0) or Target(1) mode */
void xec_i3c_enable(mm_reg_t regbase, uint8_t mode, bool enable_dma)
{
	uint32_t mask = (BIT(XEC_I3C_DEV_CR_EN_POS) | BIT(XEC_I3C_DEV_CR_IBAI_POS) |
			 BIT(XEC_I3C_DEV_CR_DMA_EN_POS));
	uint32_t rval = sys_read32(regbase + XEC_I3C_DEV_CR_OFS);

	rval |= BIT(XEC_I3C_DEV_CR_EN_POS);

	if (XEC_I3C_DEV_EXT_CR_OPM_HC == mode) {
		/* I3C Broadcast Address is included for private transfers */
		rval |= BIT(XEC_I3C_DEV_CR_IBAI_POS);
	}

	if (enable_dma == true) {
		rval |= BIT(XEC_I3C_DEV_CR_DMA_EN_POS);
	}

	soc_mmcr_mask_set(regbase + XEC_I3C_DEV_CR_OFS, rval, mask);
}

static void xec_i3c_disable(mm_reg_t regbase)
{
	sys_clear_bit(regbase + XEC_I3C_DEV_CR_OFS, XEC_I3C_DEV_CR_EN_POS);
}

static void xec_i3c_resume(mm_reg_t regbase)
{
	sys_set_bit(regbase + XEC_I3C_DEV_CR_OFS, XEC_I3C_DEV_CR_RESUME_POS);
}

static void xec_i3c_xfer_err_sts_clr(mm_reg_t regbase)
{
	if (sys_test_bit(regbase + XEC_I3C_INTR_SR_OFS, XEC_I3C_ISR_XERR_POS) != 0) {
		sys_write32(BIT(XEC_I3C_ISR_XERR_POS), regbase + XEC_I3C_INTR_SR_OFS);
	}
}

/* Target controller status register contains R/W hot join interrupt enable
 * control bit and two R/W1C bits. All other bits are read-only.
 * We must only clear Host Join interrupt enable and not cause clearing of the
 * R/W1C status bits.
 * A controller in Target Mode uses the bit in Target Event Status register to
 * enable/disable Hot Join?
 */
static void xec_i3c_tgt_hot_join_disable(mm_reg_t regbase)
{
	uint32_t targ_ev_status = sys_read32(regbase + XEC_I3C_SC_TEVT_SR_OFS);

	targ_ev_status &= (uint32_t)~(BIT(XEC_I3C_SC_TEVT_SR_HJ_EN_POS) |
				      BIT(XEC_I3C_SC_TEVT_SR_MRL_UPD_POS) |
				      BIT(XEC_I3C_SC_TEVT_SR_MWL_UPD_POS));

	/* clear hot join bit and write to device control register */
	sys_write32(targ_ev_status, regbase + XEC_I3C_SC_TEVT_SR_OFS);
}

#if 0 /* currently UNUSED */
/* Host Controller mode has a bit in Dev Control register to ACK/NAK Hot Join
 * requests it receives. Clear the bit to ACK HJ requests.
 */
static void xec_i3c_hot_join_enable(mm_reg_t regbase)
{
	sys_clear_bit(regbase + XEC_I3C_DEV_CR_OFS, XEC_I3C_DEV_CR_HJND_POS);
}
#endif

/* Set Hot Join bit in the control register for NACK and send broadcash CCC */
static void xec_i3c_hot_join_disable(mm_reg_t regbase)
{
	sys_set_bit(regbase + XEC_I3C_DEV_CR_OFS, XEC_I3C_DEV_CR_HJND_POS);
}

/**
 * @brief Program I2C Fast Mode Timing Register
 *
 * @param regs Pointer to controller registers
 * @param core_clk_freq_ns Core clock frequency in nanoseconds
 */
void xec_i2c_fm_timing_set(mm_reg_t hrb, uint32_t core_clk_freq_ns)
{
	uint32_t low_count = 0, high_count = 0, timing_val = 0, mask = 0, val = 0;

	high_count = MEC_DIV_ROUND_UP(XEC_I2C_FM_SCL_MIN_HIGH_PER_NS, core_clk_freq_ns);

	if (high_count < XEC_I3C_SCL_TIMING_COUNT_MIN) {
		high_count = XEC_I3C_SCL_TIMING_COUNT_MIN;
	}

	low_count = MEC_DIV_ROUND_UP(XEC_I2C_FM_SCL_MIN_LOW_PER_NS, core_clk_freq_ns);
	if (low_count < XEC_I3C_SCL_TIMING_COUNT_MIN) {
		low_count = XEC_I3C_SCL_TIMING_COUNT_MIN;
	}

	/* Program the I3C Push Pull Timing Register */
	timing_val = (XEC_I2C_SCL_I2C_FM_TM_LCNT_SET(low_count) |
		      XEC_I2C_SCL_I2C_FM_TM_HCNT_SET(high_count));
	sys_write32(timing_val, hrb + XEC_I3C_SCL_I2C_FM_TM_OFS);

	/* This is a Mixed Bus system
	 * Hence program the Bus Free Time (Master Mode) to tLOW of I2C Timing
	 */
	mask = XEC_I3C_BUS_FREE_TM_FT_MSK;
	val = XEC_I3C_BUS_FREE_TM_FT_SET((uint32_t)low_count);
	soc_mmcr_mask_set(hrb + XEC_I3C_BUS_FREE_TM_OFS, val, mask);
}

/**
 * @brief Program the I3C Bus Free Timing Register
 *
 * @param regs Pointer to controller registers
 * @param core_clk_freq_ns Core clock frequency in nanoseconds
 */
void xec_i3c_bus_free_timing_set(mm_reg_t srb, uint32_t core_clk_freq_ns)
{
	uint32_t bus_free_timing_count = 0;
	uint32_t val = 0;

	/* To review */
	bus_free_timing_count =
		MEC_DIV_ROUND_UP(XEC_I3C_TGT_BUS_FREE_DURATION_NS, core_clk_freq_ns);

	if (bus_free_timing_count < XEC_I3C_SCL_TIMING_COUNT_MIN) {
		bus_free_timing_count = XEC_I3C_SCL_TIMING_COUNT_MIN;
	}

	val = XEC_I3C_BUS_FREE_TM_FT_SET(bus_free_timing_count);

	soc_mmcr_mask_set(srb + XEC_I3C_BUS_FREE_TM_OFS, val, XEC_I3C_BUS_FREE_TM_FT_MSK);
}

/**
 * @brief Program the I3C Bus Free Timing Register
 *
 * @param regs Pointer to controller registers
 * @param core_clk_freq_ns Core clock frequency in nanoseconds
 */
void xec_i3c_bus_available_timing_set(mm_reg_t srb, uint32_t core_clk_freq_ns)
{
	uint32_t bus_avail_timing_count = 0, val = 0;

	bus_avail_timing_count = MEC_DIV_ROUND_UP(XEC_I3C_TGT_BUS_AVAIL_COND_NS, core_clk_freq_ns);

	if (bus_avail_timing_count < XEC_I3C_SCL_TIMING_COUNT_MIN) {
		bus_avail_timing_count = XEC_I3C_SCL_TIMING_COUNT_MIN;
	}

	val = XEC_I3C_BUS_FREE_TM_AV_SET(bus_avail_timing_count);

	soc_mmcr_mask_set(srb + XEC_I3C_BUS_FREE_TM_OFS, val, XEC_I3C_BUS_FREE_TM_AV_MSK);
}

/**
 * @brief Program the I3C Bus Free Timing Register
 *
 * @param regs Pointer to controller registers
 * @param core_clk_freq_ns Core clock frequency in nanoseconds
 */
void xec_i3c_bus_idle_timing_set(mm_reg_t srb, uint32_t core_clk_freq_ns)
{
	uint32_t idle_count = 0, val = 0;

	idle_count = MEC_DIV_ROUND_UP(XEC_I3C_TGT_BUS_IDLE_COND_NS, core_clk_freq_ns);

	if (idle_count < XEC_I3C_SCL_TIMING_COUNT_MIN) {
		idle_count = XEC_I3C_SCL_TIMING_COUNT_MIN;
	}

	val = XEC_I3C_BUS_FREE_TM_FT_SET(idle_count);

	soc_mmcr_mask_set(srb + XEC_I3C_BUS_FREE_TM_OFS, val, XEC_I3C_BUS_FREE_TM_FT_MSK);
}

/**
 * @brief Program the I3C SDA Hold Switch Timing Register
 *
 * @param regs Pointer to controller registers
 * @param core_clk_freq_ns Core clock frequency in nanoseconds
 */
void xec_i3c_sda_hld_switch_delay_timing_set(mm_reg_t srb, uint8_t sda_od_pp_switch_dly,
					     uint8_t sda_pp_od_switch_dly, uint8_t sda_tx_hold)
{
	uint32_t msk = (XEC_I3C_SDA_HMSD_TM_SDA_OP_SD_MSK | XEC_I3C_SDA_HMSD_TM_SDA_PO_SD_MSK |
			XEC_I3C_SDA_HMSD_TM_SDA_TXH_MSK);
	uint32_t val = XEC_I3C_SDA_HMSD_TM_SDA_OP_SD_SET((uint32_t)sda_od_pp_switch_dly);

	val |= XEC_I3C_SDA_HMSD_TM_SDA_PO_SD_SET((uint32_t)sda_pp_od_switch_dly);
	val |= XEC_I3C_SDA_HMSD_TM_SDA_TXH_SET((uint32_t)sda_tx_hold);

	soc_mmcr_mask_set(srb + XEC_I3C_SDA_HMSD_TM_OFS, val, msk);
}

/**
 * @brief Program the I3C SDA Hold time control value
 *
 * @param regs Pointer to controller registers
 * @param sda_tx_hold SDA TX Hold time control
 */
void xec_i3c_sda_hld_timing_set(mm_reg_t rb, uint8_t sda_tx_hold)
{
	uint32_t val = XEC_I3C_SDA_HMSD_TM_SDA_TXH_SET((uint32_t)sda_tx_hold);

	soc_mmcr_mask_set(rb + XEC_I3C_SDA_HMSD_TM_OFS, val, XEC_I3C_SDA_HMSD_TM_SDA_TXH_MSK);
}

/**
 * @brief Program the I3C read termination bit low count
 *
 * @param regs Pointer to controller registers
 * @param sda_tx_hold SDA TX Hold time control
 */
void xec_i3c_read_term_bit_low_count_set(mm_reg_t rb, uint8_t read_term_low_count)
{
	uint32_t val = XEC_I3C_SCL_TBLC_ETLC_SET((uint32_t)read_term_low_count);

	soc_mmcr_mask_set(rb + XEC_I3C_SCL_TBLC_OFS, val, XEC_I3C_SCL_TBLC_ETLC_MSK);
}

/**
 * @brief Program the I3C SCL Low Master Extended Timeout register
 *
 * @param regs Pointer to controller registers
 * @param core_clk_freq_ns Core clock frequency in nanoseconds
 */
void xec_i3c_scl_low_mst_tout_set(mm_reg_t rb, uint32_t tout_val)
{
	sys_write32(tout_val, rb + XEC_I3C_SCL_LMST_TM_OFS);
}

/**
 * @brief Inform the core that I2C targets are present on the bus
 *
 * @param regs Pointer to controller registers
 */
void xec_i2c_target_present_set(mm_reg_t rb)
{
	sys_set_bit(rb + XEC_I3C_DEV_CR_OFS, XEC_I3C_DEV_CR_I2C_PRES_POS);
}

/**
 * @brief Inform the core that I2C targets are not present on the bus
 *
 * @param regs Pointer to controller registers
 */
void xec_i2c_target_present_reset(mm_reg_t rb)
{
	sys_clear_bit(rb + XEC_I3C_DEV_CR_OFS, XEC_I3C_DEV_CR_I2C_PRES_POS);
}

/**
 * @brief Program I2C Fast Mode Plus Timing Register
 *
 * @param regs Pointer to controller registers
 * @param core_clk_freq_ns Core clock frequency in nanoseconds
 */
void xec_i2c_fmp_timing_set(mm_reg_t hrb, uint32_t core_clk_freq_ns)
{
	uint32_t low_count = 0, high_count = 0, val = 0, msk = 0;

	high_count = MEC_DIV_ROUND_UP(XEC_I2C_FMP_SCL_MIN_HIGH_PER_NS, core_clk_freq_ns);

	if (high_count < XEC_I3C_SCL_TIMING_COUNT_MIN) {
		high_count = XEC_I3C_SCL_TIMING_COUNT_MIN;
	}

	low_count = MEC_DIV_ROUND_UP(XEC_I2C_FMP_SCL_MIN_LOW_PER_NS, core_clk_freq_ns);

	if (low_count < XEC_I3C_SCL_TIMING_COUNT_MIN) {
		low_count = XEC_I3C_SCL_TIMING_COUNT_MIN;
	}

	val = XEC_I3C_SCL_I2C_FMP_TM_LCNT_SET(low_count) |
	      XEC_I3C_SCL_I2C_FMP_TM_HCNT_SET(high_count);
	msk = XEC_I3C_SCL_I2C_FMP_TM_LCNT_MSK | XEC_I3C_SCL_I2C_FMP_TM_HCNT_MSK;

	soc_mmcr_mask_set(hrb + XEC_I3C_SCL_I2C_FMP_TM_OFS, val, msk);
}

/**
 * @brief Program I3C Push Pull Timing Register
 *
 * @param regs Pointer to controller registers
 * @param core_clk_freq_ns Core clock frequency in nanoseconds
 */
void xec_i3c_push_pull_timing_set(mm_reg_t hrb, uint32_t core_clk_freq_ns, uint32_t i3c_freq_ns)
{
	uint32_t low_count = 0, high_count = 0, base_count = 0;
	uint32_t sdr_elcnt = 0, val = 0, msk = 0;

	base_count = MEC_DIV_ROUND_UP(XEC_I3C_PUSH_PULL_SCL_MIN_HIGH_PER_NS, core_clk_freq_ns);

	if (base_count < XEC_I3C_SCL_TIMING_COUNT_MIN) {
		base_count = XEC_I3C_SCL_TIMING_COUNT_MIN;
	}

	high_count = MEC_DIV_ROUND_UP(base_count * i3c_freq_ns, XEC_I3C_SCL_12_5MHZ_PER_NS);

	if (high_count < XEC_I3C_SCL_TIMING_COUNT_MIN) {
		high_count = XEC_I3C_SCL_TIMING_COUNT_MIN;
	}

	low_count = high_count;

	msk = XEC_I3C_SCL_PP_TM_LCNT_MSK | XEC_I3C_SCL_PP_TM_HCNT_MSK;
	val = XEC_I3C_SCL_PP_TM_LCNT_SET(low_count) | XEC_I3C_SCL_PP_TM_HCNT_SET(high_count);

	soc_mmcr_mask_set(hrb + XEC_I3C_SCL_PP_TM_OFS, val, msk);

	/* If this is a PURE I3C bus
	 * Program the Bus Free Time (Master Mode) to tCAS parameter
	 */
	if (sys_test_bit(hrb + XEC_I3C_DEV_CR_OFS, XEC_I3C_DEV_CR_I2C_PRES_POS) == 0) {
		val = XEC_I3C_BUS_FREE_TM_FT_SET(low_count);
		soc_mmcr_mask_set(hrb + XEC_I3C_BUS_FREE_TM_OFS, val, XEC_I3C_BUS_FREE_TM_FT_MSK);
	}

	sdr_elcnt = MEC_DIV_ROUND_UP(XEC_I3C_BUS_SDR1_SCL_PER_NS, core_clk_freq_ns) - high_count;
	val = XEC_I3C_SCL_ELC_TM_CNT1_SET(sdr_elcnt);

	sdr_elcnt = MEC_DIV_ROUND_UP(XEC_I3C_BUS_SDR2_SCL_PER_NS, core_clk_freq_ns) - high_count;
	val |= XEC_I3C_SCL_ELC_TM_CNT2_SET(sdr_elcnt);

	sdr_elcnt = MEC_DIV_ROUND_UP(XEC_I3C_BUS_SDR3_SCL_PER_NS, core_clk_freq_ns) - high_count;
	val |= XEC_I3C_SCL_ELC_TM_CNT3_SET(sdr_elcnt);

	sdr_elcnt = MEC_DIV_ROUND_UP(XEC_I3C_BUS_SDR4_SCL_PER_NS, core_clk_freq_ns) - high_count;
	val |= XEC_I3C_SCL_ELC_TM_CNT4_SET(sdr_elcnt);

	msk = (XEC_I3C_SCL_ELC_TM_CNT1_MSK | XEC_I3C_SCL_ELC_TM_CNT2_MSK |
	       XEC_I3C_SCL_ELC_TM_CNT3_MSK | XEC_I3C_SCL_ELC_TM_CNT4_MSK);

	soc_mmcr_mask_set(hrb + XEC_I3C_SCL_ELC_TM_OFS, val, msk);
}

/**
 * @brief Program I3C Open Drain Timing Register
 *
 * @param regs Pointer to controller registers
 * @param core_clk_freq_ns Core clock frequency in nanoseconds
 */
void xec_i3c_open_drain_timing_set(mm_reg_t hrb, uint32_t core_clk_freq_ns, uint32_t i3c_freq_ns)
{
	uint32_t low_count = 0, high_count = 0, val = 0, msk = 0;

	high_count = MEC_DIV_ROUND_UP(XEC_I3C_OPEN_DRAIN_SCL_MIN_HIGH_PER_NS, core_clk_freq_ns);

	high_count = MEC_DIV_ROUND_UP(high_count * i3c_freq_ns, XEC_I3C_SCL_12_5MHZ_PER_NS);

	if (high_count < XEC_I3C_SCL_TIMING_COUNT_MIN) {
		high_count = XEC_I3C_SCL_TIMING_COUNT_MIN;
	}

	low_count = MEC_DIV_ROUND_UP(XEC_I3C_OPEN_DRAIN_SCL_MIN_LOW_PER_NS, core_clk_freq_ns);

	low_count = MEC_DIV_ROUND_UP(low_count * i3c_freq_ns, XEC_I3C_SCL_12_5MHZ_PER_NS);

	if (low_count < XEC_I3C_SCL_TIMING_COUNT_MIN) {
		low_count = XEC_I3C_SCL_TIMING_COUNT_MIN;
	}

	val = XEC_I3C_SCL_OD_TM_LCNT_SET(low_count) | XEC_I3C_SCL_OD_TM_HCNT_SET(high_count);
	msk = XEC_I3C_SCL_OD_TM_LCNT_MSK | XEC_I3C_SCL_OD_TM_HCNT_MSK;

	soc_mmcr_mask_set(hrb + XEC_I3C_SCL_OD_TM_OFS, val, msk);
}

/* Read from IBI Queue
 * rb : I3C controller register base address
 * buffer : buffer to copy data
 * len : Length of data to read
 * !!! Handle caller passing an 8-bit or 16-bit aligned buffer !!!
 */
void xec_i3c_ibi_data_read(mm_reg_t hrb, uint8_t *buffer, uint16_t len)
{
	uint32_t *dword_ptr = NULL;
	uint8_t *bptr = NULL;
	uint32_t last_dword = 0, ibi_data = 0;
	uint16_t i = 0, remaining_bytes = 0;
	volatile uint32_t drain_dword = 0;
	bool drain_flag = false;
	bool aligned_buffer = IS_ALIGNED(buffer, 4);

	if (NULL == buffer) {
		drain_flag = true;
	}

	if (aligned_buffer == true) {
		dword_ptr = (uint32_t *)buffer;
	} else {
		bptr = buffer;
	}

	if (len >= 4) {
		if (drain_flag == true) {
			for (i = 0; i < len / 4; i++) {
				drain_dword |= sys_read32(hrb + XEC_I3C_IBI_QUE_SR_OFS);
			}
		} else {
			for (i = 0; i < len / 4; i++) {
				ibi_data = sys_read32(hrb + XEC_I3C_IBI_QUE_SR_OFS);
				if (aligned_buffer == true) {
					dword_ptr[i] = ibi_data;
				} else {
					*bptr++ = ibi_data;
					*bptr++ = (uint8_t)(ibi_data >> 8);
					*bptr++ = (uint8_t)(ibi_data >> 16);
					*bptr++ = (uint8_t)(ibi_data >> 24);
				}
			}
		}
	}

	remaining_bytes = len % 4;

	if (remaining_bytes != 0) {
		last_dword = sys_read32(hrb + XEC_I3C_IBI_QUE_SR_OFS);

		if (drain_flag == false) {
			memcpy(buffer + (len & ~0x3), &last_dword, remaining_bytes);
		}
	}
}

/**
 * @brief Retrieve target max read length
 */
void xec_i3c_tgt_MRL_get(mm_reg_t srb, uint16_t *max_rd_len)
{
	uint32_t rval = sys_read32(srb + XEC_I3C_SC_MAX_RW_LEN_OFS);

	*max_rd_len = (uint16_t)XEC_I3C_SC_MAX_RW_LEN_RL_GET(rval);
}

/**
 * @brief Retrieve target max write length
 */
void xec_i3c_tgt_MWL_get(mm_reg_t srb, uint16_t *max_wr_len)
{
	uint32_t rval = sys_read32(srb + XEC_I3C_SC_MAX_RW_LEN_OFS);

	*max_wr_len = (uint16_t)XEC_I3C_SC_MAX_RW_LEN_WL_GET(rval);
}

/* returns 1 if bit was set and it clears the bit else returns 0 */
int xec_i3c_tgt_MRL_updated(mm_reg_t srb)
{
	/* MRL bit is read and write 1-to-clear */
	return sys_test_and_set_bit(srb + XEC_I3C_SC_TEVT_SR_OFS, XEC_I3C_SC_TEVT_SR_MRL_UPD_POS);
}

/* returns 1 if bit was set and it clears the bit else returns 0 */
int xec_i3c_tgt_MWL_updated(mm_reg_t srb)
{
	/* MRL bit is read and write 1-to-clear */
	return sys_test_and_set_bit(srb + XEC_I3C_SC_TEVT_SR_OFS, XEC_I3C_SC_TEVT_SR_MWL_UPD_POS);
}

void xec_i3c_tgt_max_speed_update(mm_reg_t srb, uint8_t max_rd_speed, uint8_t max_wr_speed)
{
	uint32_t msk = 0, val = 0;

	val = XEC_I3C_SC_MXDS_MWS_SET((uint32_t)max_wr_speed) |
	      XEC_I3C_SC_MXDS_MRS_SET((uint32_t)max_rd_speed);
	msk = XEC_I3C_SC_MXDS_MWS_MSK | XEC_I3C_SC_MXDS_MRS_MSK;

	soc_mmcr_mask_set(srb + XEC_I3C_SC_MXDS_OFS, val, msk);
}

void xec_i3c_tgt_clk_to_data_turn_update(mm_reg_t srb, uint8_t clk_data_turn_time)
{
	uint32_t val = XEC_I3C_SC_MXDS_CDT_SET((uint32_t)clk_data_turn_time);

	soc_mmcr_mask_set(srb + XEC_I3C_SC_MXDS_OFS, val, XEC_I3C_SC_MXDS_CDT_MSK);
}

void xec_i3c_tgt_MRL_MWL_set(mm_reg_t srb, uint16_t max_rd_len, uint16_t max_wr_len)
{
	uint32_t val = XEC_I3C_SC_MAX_RW_LEN_WL_SET((uint32_t)max_wr_len) |
		       XEC_I3C_SC_MAX_RW_LEN_RL_SET((uint32_t)max_rd_len);

	sys_write32(val, srb + XEC_I3C_SC_MAX_RW_LEN_OFS);
}

void xec_i3c_host_dma_tx_burst_length_set(mm_reg_t hrb, uint32_t val)
{
	uint32_t rv = XEC_I3C_HC_CFG_DMA_TXBT_SET(val);

	soc_mmcr_mask_set(hrb + XEC_I3C_HC_CFG_OFS, rv, XEC_I3C_HC_CFG_DMA_TXBT_MSK);
}

void xec_i3c_host_dma_rx_burst_length_set(mm_reg_t hrb, uint32_t val)
{
	uint32_t rv = XEC_I3C_HC_CFG_DMA_RXBT_SET(val);

	soc_mmcr_mask_set(hrb + XEC_I3C_HC_CFG_OFS, rv, XEC_I3C_HC_CFG_DMA_RXBT_MSK);
}

void xec_i3c_host_port_set(mm_reg_t hrb, uint32_t port_sel)
{
	uint32_t rv = XEC_I3C_HC_CFG_PORT_SET(port_sel);

	soc_mmcr_mask_set(hrb + XEC_I3C_HC_CFG_OFS, rv, XEC_I3C_HC_CFG_PORT_MSK);
}

void xec_i3c_sec_host_port_set(mm_reg_t srb, uint32_t port_sel)
{
	uint32_t rv = XEC_I3C_SC_CFG_PORT_SET(port_sel);

	soc_mmcr_mask_set(srb + XEC_I3C_SC_CFG_OFS, rv, XEC_I3C_SC_CFG_PORT_MSK);
}

void xec_i3c_host_stuck_sda_config(mm_reg_t hrb, uint32_t en, uint32_t tout_val)
{
	uint32_t rv = 0;

	if (en != HOST_CFG_STUCK_SDA_DISABLE) {
		rv = XEC_I3C_HC_STK_SDA_TMOUT_VAL_SET(tout_val);
		sys_set_bit(hrb + XEC_I3C_HC_CFG_OFS, XEC_I3C_HC_CFG_SSDA_EN_POS);
	} else {
		sys_clear_bit(hrb + XEC_I3C_HC_CFG_OFS, XEC_I3C_HC_CFG_SSDA_EN_POS);
	}

	soc_mmcr_mask_set(hrb + XEC_I3C_HC_STK_SDA_TMOUT_OFS, rv, XEC_I3C_HC_STK_SDA_TMOUT_VAL_MSK);
}

void xec_i3c_host_tx_dma_tout_config(mm_reg_t hrb, uint32_t en, uint32_t tout_val)
{
	uint32_t field_val = 0;

	if (en != XEC_I3C_CFG_DMA_TMOUT_DIS) {
		field_val = XEC_I3C_HC_DMA_TMOUT_SET(tout_val);
		sys_set_bit(hrb + XEC_I3C_HC_CFG_OFS, XEC_I3C_HC_CFG_DMA_TTMO_EN_POS);
	} else {
		sys_clear_bit(hrb + XEC_I3C_HC_CFG_OFS, XEC_I3C_HC_CFG_DMA_TTMO_EN_POS);
	}

	soc_mmcr_mask_set(hrb + XEC_I3C_HC_DMA_TX_TMOUT_OFS, field_val, XEC_I3C_HC_DMA_TMOUT_MSK);
}

void xec_i3c_host_rx_dma_tout_config(mm_reg_t hrb, uint32_t en, uint32_t tout_val)
{
	uint32_t field_val = 0;

	if (en != XEC_I3C_CFG_DMA_TMOUT_DIS) {
		field_val = XEC_I3C_HC_DMA_TMOUT_SET(tout_val);
		sys_set_bit(hrb + XEC_I3C_HC_CFG_OFS, XEC_I3C_HC_CFG_DMA_RTMO_EN_POS);
	} else {
		sys_clear_bit(hrb + XEC_I3C_HC_CFG_OFS, XEC_I3C_HC_CFG_DMA_RTMO_EN_POS);
	}

	soc_mmcr_mask_set(hrb + XEC_I3C_HC_DMA_RX_TMOUT_OFS, field_val, XEC_I3C_HC_DMA_TMOUT_MSK);
}

void xec_i3c_sec_host_tx_dma_tout_config(mm_reg_t srb, uint32_t en, uint32_t tout_val)
{
	uint32_t field_val = 0;

	if (en != XEC_I3C_CFG_DMA_TMOUT_DIS) {
		field_val = XEC_I3C_SC_DMA_TMOUT_SET(tout_val);
	}

	soc_mmcr_mask_set(srb + XEC_I3C_SC_DMA_TX_TMOUT_OFS, field_val, XEC_I3C_SC_DMA_TMOUT_MSK);
}

void xec_i3c_sec_host_rx_dma_tout_config(mm_reg_t srb, uint32_t en, uint32_t tout_val)
{
	uint32_t field_val = 0;

	if (en != XEC_I3C_CFG_DMA_TMOUT_DIS) {
		field_val = XEC_I3C_SC_DMA_TMOUT_SET(tout_val);
	}

	soc_mmcr_mask_set(srb + XEC_I3C_SC_DMA_RX_TMOUT_OFS, field_val, XEC_I3C_SC_DMA_TMOUT_MSK);
}

void xec_i3c_sec_host_dma_tx_burst_length_set(mm_reg_t srb, uint32_t val)
{
	uint32_t rv = XEC_I3C_SC_CFG_DMA_TXBT_SET(val);

	soc_mmcr_mask_set(srb + XEC_I3C_SC_CFG_OFS, rv, XEC_I3C_SC_CFG_DMA_TXBT_MSK);
}

void xec_i3c_sec_host_dma_rx_burst_length_set(mm_reg_t srb, uint32_t val)
{
	uint32_t rv = XEC_I3C_SC_CFG_DMA_RXBT_SET(val);

	soc_mmcr_mask_set(srb + XEC_I3C_SC_CFG_OFS, rv, XEC_I3C_SC_CFG_DMA_RXBT_MSK);
}

void xec_i3c_sec_host_stuck_sda_scl_config(mm_reg_t srb, uint32_t en, uint32_t sda_tout_val,
					   uint32_t scl_tout_val)
{
	uint32_t msk = XEC_I3C_SC_TMO_SSDA_MSK | XEC_I3C_SC_TMO_SCL_LO_MSK;
	uint32_t val = (XEC_I3C_SC_TMO_SSDA_SET(XEC_I3C_SC_TMO_SDDA_DIS) |
			XEC_I3C_SC_TMO_SCL_LO_SET(XEC_I3C_SC_TMO_SCL_LO_DIS));

	if (en != SEC_HOST_STK_SDA_SCL_DIS) {
		val = (XEC_I3C_SC_TMO_SSDA_SET(sda_tout_val) |
		       XEC_I3C_SC_TMO_SCL_LO_SET(scl_tout_val));
		sys_set_bit(srb + XEC_I3C_SC_CFG_OFS, XEC_I3C_SC_CFG_SSDA_EN_POS);
	} else {
		sys_clear_bit(srb + XEC_I3C_SC_CFG_OFS, XEC_I3C_SC_CFG_SSDA_EN_POS);
	}

	soc_mmcr_mask_set(srb + XEC_I3C_SC_TMO_CR_OFS, val, msk);
}

/* Get device pointer from Host or Secondary Host controller */
void xec_i3c_dev_addr_table_ptr_get(mm_reg_t regbase, uint16_t *start_addr, uint16_t *depth)
{
	uint32_t val = sys_read32(regbase + XEC_I3C_DAT_PTR_OFS);

	*start_addr = XEC_I3C_DAT_PTR_STA_GET(val);
	*depth = XEC_I3C_DAT_PTR_DEPTH_GET(val);
}

/* Get device characteristics pointer from Host or Secondary Host controller */
void xec_i3c_dev_char_table_ptr_get(mm_reg_t regbase, uint16_t *start_addr, uint16_t *depth)
{
	uint32_t val = sys_read32(regbase + XEC_I3C_DCT_PTR_OFS);

	*start_addr = XEC_I3C_DCT_PTR_STA_GET(val);
	*depth = XEC_I3C_DCT_PTR_DEPTH_GET(val);
}

/* Get TX FIFO depth from Host or Secondary Host controller
 * fv                2<<fv   bytes
 * 0 = 2 dwords      2       8
 * 1 = 4 dwords      4       16
 * 2 = 8 dwords      8       32
 * ...
 * 5 = 64 dwords     64      256
 */
uint32_t xec_i3c_tx_fifo_depth_get(mm_reg_t regbase)
{
	uint32_t r = sys_read32(regbase + XEC_I3C_QSZ_CAP_OFS);

	return ((2 << XEC_I3C_QSZ_CAP_TX_GET(r)) * 4u);
}

uint32_t xec_i3c_rx_fifo_depth_get(mm_reg_t regbase)
{
	uint32_t r = sys_read32(regbase + XEC_I3C_QSZ_CAP_OFS);

	return ((2 << XEC_I3C_QSZ_CAP_RX_GET(r)) * 4u);
}

uint32_t xec_i3c_cmd_fifo_depth_get(mm_reg_t regbase)
{
	uint32_t r = sys_read32(regbase + XEC_I3C_QSZ_CAP_OFS);

	return ((2 << XEC_I3C_QSZ_CAP_CMD_GET(r)) * 4u);
}

uint32_t xec_i3c_resp_fifo_depth_get(mm_reg_t regbase)
{
	uint32_t r = sys_read32(regbase + XEC_I3C_QSZ_CAP_OFS);

	return ((2 << XEC_I3C_QSZ_CAP_RESP_GET(r)) * 4u);
}

uint32_t xec_i3c_ibi_fifo_depth_get(mm_reg_t regbase)
{
	uint32_t r = sys_read32(regbase + XEC_I3C_QSZ_CAP_OFS);

	return ((2 << XEC_I3C_QSZ_CAP_IBI_GET(r)) * 4u);
}

/**
 * @brief Write one Device Address Table (DAT) entry in external RAM.
 *
 * The Device Address Table (DAT) is used only in Controller mode to store addressing
 * information for each target device on the I3C bus. Command descriptors (e.g. Address
 * Assignment and Transfer commands) reference DAT entries through a device index
 * (DEV_INDEX) to obtain the target addressing/protocol attributes (I3C dynamic/static
 * address, legacy I2C select, IBI capability bits, etc.).
 *
 * Per the specification, DAT registers are implemented in the controller’s external RAM
 * and their offsets vary by configuration. Software must obtain the DAT start address and
 * depth from the DEVICE_ADDR_TABLE_POINTER register and then program one 32-bit DAT word
 * per device at:
 *
 *   entry_addr = regbase + DAT_start + (DAT_idx * 4)
 *
 * where DAT_start is the start offset/address of the DAT external RAM window and DAT_idx
 * is the device index used by command descriptors.
 *
 * This helper performs the raw 32-bit write. The caller is responsible for:
 * - ensuring DAT_start/DAT_idx are valid for the current configuration (within DAT depth)
 * - formatting @p val according to the selected DAT structure
 *   (I3C w/ static addr, I3C w/ dynamic addr, or legacy I2C)
 *
 * @param regbase   I3C controller register base address.
 * @param DAT_start Start offset/address of the DAT in external RAM (from DAT pointer register).
 * @param DAT_idx   Device index within the DAT.
 * @param val       Encoded 32-bit DAT entry value to program.
 */
void xec_i3c_DAT_write(mm_reg_t regbase, uint16_t DAT_start, uint8_t DAT_idx, uint32_t val)
{
	uint32_t *entry_addr = (uint32_t *)((uintptr_t)regbase + (uintptr_t)DAT_start +
					    ((uintptr_t)DAT_idx * sizeof(uint32_t)));

	*entry_addr = val;
}

/* Handle pointer of any alignment and length of any size */
void xec_i3c_fifo_write(mm_reg_t regbase, uint8_t *buffer, uint16_t len)
{
	uint32_t i = 0, data32 = 0, rem_bytes = 0;
	uint8_t *remptr = NULL;

	if ((buffer == NULL) || (len == 0)) {
		return;
	}

	if (IS_ALIGNED(buffer, 4)) {
		uint32_t *aligned_ptr = (uint32_t *)buffer;

		if (len >= 4U) {
			for (i = 0; i < len / 4U; i++) {
				sys_write32(*aligned_ptr, regbase + XEC_I3C_TX_DATA_OFS);
				aligned_ptr++;
			}
		}

		remptr = (uint8_t *)aligned_ptr;

	} else if (IS_ALIGNED(buffer, 2)) {
		uint16_t *aligned_ptr16 = (uint16_t *)buffer;

		if (len >= 4U) {
			for (i = 0; i < len / 4U; i++) {
				data32 = *aligned_ptr16++;
				data32 |= ((uint32_t)*aligned_ptr16 << 16);
				sys_write32(data32, regbase + XEC_I3C_TX_DATA_OFS);
				aligned_ptr16++;
			}
		}

		remptr = (uint8_t *)aligned_ptr16;

	} else {
		if (len >= 4U) {
			for (i = 0; i < len / 4U; i++) {
				data32 = *(buffer + 3U);
				data32 <<= 8;
				data32 |= *(buffer + 2U);
				data32 <<= 8;
				data32 |= *(buffer + 1U);
				data32 <<= 8;
				data32 |= *buffer;
				sys_write32(data32, regbase + XEC_I3C_TX_DATA_OFS);
				buffer += 4u;
			}
		}

		remptr = buffer;
	}

	rem_bytes = len % 4U;

	if (rem_bytes != 0) {
		data32 = 0;
		memcpy((void *)&data32, (const void *)remptr, rem_bytes);
		sys_write32(data32, regbase + XEC_I3C_TX_DATA_OFS);
	}
}

/* Handle pointer of any alignment and length of any size */
void xec_i3c_fifo_read(mm_reg_t hrb, uint8_t *buffer, uint16_t len)
{
	uint32_t data32 = 0, n = 0, num_rem = 0;
	uint8_t *remptr = NULL;

	if (IS_PTR_ALIGNED(buffer, uint32_t) == true) {
		uint32_t *aligned_ptr = (uint32_t *)buffer;

		if (len >= 4U) {
			for (n = 0; n < (len / 4U); n++) {
				*aligned_ptr++ = sys_read32(hrb + XEC_I2C_RX_DATA_OFS);
			}
		}

		remptr = (uint8_t *)aligned_ptr;
	} else if (IS_PTR_ALIGNED(buffer, uint16_t) == true) {
		uint16_t *aligned_ptr16 = (uint16_t *)buffer;

		if (len >= 4U) {
			for (n = 0; n < (len / 4U); n++) {
				data32 = sys_read32(hrb + XEC_I2C_RX_DATA_OFS);
				*aligned_ptr16 = (uint16_t)data32;
				aligned_ptr16++;
				*aligned_ptr16 = (uint16_t)(data32 >> 16);
				aligned_ptr16++;
			}
		}

		remptr = (uint8_t *)aligned_ptr16;
	} else {
		uint8_t *ptr = buffer;

		if (len >= 4U) {
			for (n = 0; n < (len / 4U); n++) {
				data32 = sys_read32(hrb + XEC_I2C_RX_DATA_OFS);
				*ptr++ = (uint8_t)data32;
				*ptr++ = (uint8_t)(data32 >> 8);
				*ptr++ = (uint8_t)(data32 >> 16);
				*ptr++ = (uint8_t)(data32 >> 24);
			}
		}

		remptr = ptr;
	}

	num_rem = len % 4U;

	if (num_rem) {
		data32 = sys_read32(hrb + XEC_I2C_RX_DATA_OFS);
		memcpy(remptr, (const void *)&data32, num_rem);
	}
}

/**
 * @brief Read one Device Address Table (DAT) entry from external RAM.
 *
 * The Device Address Table (DAT) is stored in the controller’s external RAM (Controller mode)
 * and contains one 32-bit entry per device indexed by DEV_INDEX. DAT base (start offset/address)
 * and depth are provided by the DEVICE_ADDR_TABLE_POINTER register.
 *
 * This helper reads the raw 32-bit DAT word from:
 *
 *   entry_addr = regbase + DAT_start + (DAT_idx * 4)
 *
 * The caller is responsible for:
 * - ensuring DAT_start/DAT_idx are valid for the current configuration (within DAT depth)
 * - decoding the returned value according to the DAT structure in use
 *   (I3C w/ static addr, I3C w/ dynamic addr, or legacy I2C)
 *
 * @param regbase   I3C controller register base address.
 * @param DAT_start Start offset/address of the DAT in external RAM (from DAT pointer register).
 * @param DAT_idx   Device index within the DAT.
 *
 * @return Raw 32-bit DAT entry value.
 */
uint32_t xec_i3c_DAT_read(mm_reg_t regbase, uint16_t DAT_start, uint8_t DAT_idx)
{
	uint32_t *entry_addr = (uint32_t *)((uintptr_t)regbase + (uintptr_t)DAT_start +
					    ((uintptr_t)DAT_idx * sizeof(uint32_t)));

	return *entry_addr;
}

/* TODO - original code was setting Target Provision ID type in bit[0] to 0 */
void xec_i3c_tgt_pid_set(mm_reg_t srb, uint16_t tgt_mipi_mfg_id, bool is_random_prov_id,
			 uint16_t tgt_part_id, uint8_t tgt_inst_id, uint16_t tgt_pid_dcr)
{
	uint32_t val = XEC_I3C_SC_MID_MFG_ID_SET((uint32_t)tgt_mipi_mfg_id);
	uint32_t msk = XEC_I3C_SC_MID_MFG_ID_MSK | BIT(XEC_I3C_SC_MID_TYPE_POS);

	soc_mmcr_mask_set(srb + XEC_I3C_SC_MID_OFS, val, XEC_I3C_SC_MID_MFG_ID_MSK);

	if (is_random_prov_id == false) {
		msk = XEC_I3C_SC_PROV_ID_PID_DCR_MSK | XEC_I3C_SC_PROV_ID_INST_MSK |
		      XEC_I3C_SC_PROV_ID_PART_MSK;
		val = XEC_I3C_SC_PROV_ID_PID_DCR_SET((uint32_t)tgt_pid_dcr) |
		      XEC_I3C_SC_PROV_ID_INST_SET((uint32_t)tgt_inst_id) |
		      XEC_I3C_SC_PROV_ID_PART_SET((uint32_t)tgt_part_id);
	}
}

void xec_i3c_tgt_raise_ibi_SIR(mm_reg_t srb, uint8_t *sir_data, uint8_t sir_datalen, uint8_t mdb)
{
	uint32_t sir_data_dword = 0;
	uint32_t val = 0, msk = 0;

	msk = XEC_I3C_SC_TIREQ_MDB_MSK | XEC_I3C_SC_TIREQ_TDL_MSK;
	val = XEC_I3C_SC_TIREQ_MDB_SET((uint32_t)mdb);
	val |= XEC_I3C_SC_TIREQ_TDL_SET((uint32_t)sir_datalen);

	soc_mmcr_mask_set(srb + XEC_I3C_SC_TIREQ_OFS, val, msk);

	if (sir_datalen != 0) {
		for (int i = 0; i < sir_datalen; i++) {
			sir_data_dword <<= 8;
			sir_data_dword |= sir_data[i];
		}

		sys_write32(sir_data_dword, srb + XEC_I3C_SC_TIREQ_DAT_OFS);
	}

	sys_set_bit(srb + XEC_I3C_SC_TIREQ_OFS, XEC_I3C_SC_TIREQ_TIR_POS);
}

/*------------------------ High level helpers ---------------------------------*/

/* Intialize timing for i2c transfers */
void XEC_I3C_Controller_Clk_I2C_Init(const struct device *dev, uint32_t core_clk_rate_mhz)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t rb = drvcfg->regbase;
	uint32_t core_clk_freq_ns = 0;

	soc_xec_pcr_sleep_en_clear(drvcfg->hwctx.pcr_scr);
	soc_xec_pcr_reset_en(drvcfg->hwctx.pcr_scr);

	core_clk_freq_ns = MEC_DIV_ROUND_UP(MHZ(1000), core_clk_rate_mhz);

	xec_i2c_fmp_timing_set(rb, core_clk_freq_ns);

	xec_i2c_fm_timing_set(rb, core_clk_freq_ns);

	xec_i2c_target_present_set(rb);
}

/**
 * @brief Initialize timing for i3c transfers
 *
 * @param ctx Context structure containing Pointer to controller registers
 * @param core_clk_rate_mhz Core Clock speed
 * @param i3c_freq I3C Frequency
 */
void XEC_I3C_Controller_Clk_Init(const struct device *dev, uint32_t core_clk_rate_mhz,
				 uint32_t i3c_freq)
{
	const struct xec_i3c_config *drvcfg = dev->config;

	soc_xec_pcr_sleep_en_clear(drvcfg->hwctx.pcr_scr);
	soc_xec_pcr_reset_en(drvcfg->hwctx.pcr_scr);

	XEC_I3C_Controller_Clk_Cfg(dev, core_clk_rate_mhz, i3c_freq);
}

/**
 * @brief configure timing for i3c transfers
 *
 * @param ctx Context structure containing Pointer to controller registers
 * @param core_clk_rate_mhz Core Clock speed
 * @param i3c_freq I3C Frequency
 */
void XEC_I3C_Controller_Clk_Cfg(const struct device *dev, uint32_t core_clk_rate_mhz,
				uint32_t i3c_freq)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	uint32_t core_clk_freq_ns = 0, i3c_freq_ns = 0;

	core_clk_freq_ns = MEC_DIV_ROUND_UP(MHZ(1000), core_clk_rate_mhz);

	i3c_freq_ns = MEC_DIV_ROUND_UP(MHZ(1000), i3c_freq);

	/* Program the I3C Push Pull Timing Register */
	xec_i3c_push_pull_timing_set(regbase, core_clk_freq_ns, i3c_freq_ns);

	/* Program the I3C Open Drain Timing Register */
	xec_i3c_open_drain_timing_set(regbase, core_clk_freq_ns, i3c_freq_ns);

	xec_i3c_sda_hld_timing_set(regbase, XEC_I3C_SDA_HMSD_TM_SDA_TXH_4);
	xec_i3c_read_term_bit_low_count_set(regbase, XEC_I3C_SCL_TBLC_ETLC_4);
}

/**
 * @brief Initializes the target related registers
 *
 * @param regs Pointer to controller registers
 */
void XEC_I3C_Target_Init(const struct device *dev, uint32_t core_clk_rate_mhz, uint16_t *max_rd_len,
			 uint16_t *max_wr_len)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	uint32_t core_clk_freq_ns = 0;

	soc_xec_pcr_sleep_en_clear(drvcfg->hwctx.pcr_scr);
	soc_xec_pcr_reset_en(drvcfg->hwctx.pcr_scr);

	core_clk_freq_ns = MEC_DIV_ROUND_UP(MHZ(1000), core_clk_rate_mhz);

	/* Program the I3C Bus Free Avail Timing Register */
	xec_i3c_bus_available_timing_set(regbase, core_clk_freq_ns);

	/* Program the I3C Bus Idle Timing Register */
	xec_i3c_bus_idle_timing_set(regbase, core_clk_freq_ns);

	xec_i3c_bus_free_timing_set(regbase, core_clk_freq_ns);

	/* Get default max read and write length */
	xec_i3c_tgt_MRL_get(regbase, max_rd_len);
	xec_i3c_tgt_MWL_get(regbase, max_wr_len);

	xec_i3c_sda_hld_switch_delay_timing_set(regbase, SDA_OD_PP_SWITCH_DLY_0,
						SDA_PP_OD_SWITCH_DLY_0, SDA_TX_HOLD_1);

	xec_i3c_scl_low_mst_tout_set(regbase, XEC_I3C_SCL_LMST_TM_DFLT);

	xec_i3c_tgt_max_speed_update(regbase, TGT_MAX_RD_DATA_SPEED, TGT_MAX_WR_DATA_SPEED);

	xec_i3c_tgt_clk_to_data_turn_update(regbase, TGT_CLK_TO_DATA_TURN);
	/* Going with default values for TGT_TSX_SYMBL_TIMING register
	 * (params MXDS_CLK_DATA_TURN and MXDS_CLK_DATA_TURN)
	 * See section 8.1.4 in user guide; revisit later if we need to program
	 * these registers */
}

/**
 * @brief Updates the Max read and write lengths if controller updates using CCC
 *
 * @param regs Pointer to controller registers
 */
void XEC_I3C_Target_MRL_MWL_update(const struct device *dev, uint16_t *max_rd_len,
				   uint16_t *max_wr_len)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;

	if (xec_i3c_tgt_MRL_updated(regbase) != 0) {
		xec_i3c_tgt_MRL_get(regbase, max_rd_len);
	}

	if (xec_i3c_tgt_MWL_updated(regbase) != 0) {
		xec_i3c_tgt_MWL_get(regbase, max_wr_len);
	}
}

/**
 * @brief Sets the Max read and write lengths
 *
 * @param regs Pointer to controller registers
 */
void XEC_I3C_Target_MRL_MWL_set(const struct device *dev, uint16_t max_rd_len, uint16_t max_wr_len)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;

	xec_i3c_tgt_MRL_MWL_set(regbase, max_rd_len, max_wr_len);
}

/**
 * @brief Enable target controller interrupts
 *
 * @param regs Pointer to controller registers
 */
void XEC_I3C_Target_Interrupts_Init(const struct device *dev)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	const struct mec_i3c_ctx *ctx = &drvcfg->hwctx;
	uint32_t mask = UINT32_MAX;

	soc_ecia_girq_ctrl(ctx->girq, ctx->girq_pos, 0);
	soc_xec_pcr_sleep_en_clear(ctx->pcr_scr);

	/* Clear all interrupt status */
	xec_i3c_intr_sts_clear(regbase, mask);

	/* Enable only necessary interrupts */
	mask = (BIT(XEC_I3C_ISR_RRDY_POS) | BIT(XEC_I3C_ISR_CCC_UPD_POS) |
		BIT(XEC_I3C_ISR_DYNA_POS) | BIT(XEC_I3C_ISR_DEFTR_POS) | BIT(XEC_I3C_ISR_RRR_POS) |
		BIT(XEC_I3C_ISR_IBI_UPD_POS) | BIT(XEC_I3C_ISR_BUS_OUPD_POS));

	/* Enable required interrupt status */
	xec_i3c_intr_sts_enable(regbase, mask);

	/* Enable required interrupt signals */
	xec_i3c_intr_sgnl_enable(regbase, mask);

	/* Enable GIRQ */
	soc_ecia_girq_status_clear(ctx->girq, ctx->girq_pos);
	soc_ecia_girq_ctrl(ctx->girq, ctx->girq_pos, 1);
}

/**
 * @brief Enable required controller interrupts
 *
 * @param regs Pointer to controller registers
 */
void XEC_I3C_Controller_Interrupts_Init(const struct device *dev)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	const struct mec_i3c_ctx *ctx = &drvcfg->hwctx;
	uint32_t mask = UINT32_MAX;

	soc_ecia_girq_ctrl(ctx->girq, ctx->girq_pos, 0);
	soc_xec_pcr_sleep_en_clear(ctx->pcr_scr);

	/* Clear all interrupt status */
	xec_i3c_intr_sts_clear(regbase, mask);

	/* Enable only necessary interrupts */
	mask = (BIT(XEC_I3C_ISR_RRDY_POS) | BIT(XEC_I3C_ISR_XFR_ABRT_POS) |
		BIT(XEC_I3C_ISR_XERR_POS) | BIT(XEC_I3C_ISR_DEFTR_POS) |
		BIT(XEC_I3C_ISR_BUS_OUPD_POS) | BIT(XEC_I3C_ISR_BUS_RD_POS));

	/* Enable required interrupt status */
	xec_i3c_intr_sts_enable(regbase, mask);

	/* Enable required interrupt signals */
	xec_i3c_intr_sgnl_enable(regbase, mask);

	/* Enable GIRQ */
	soc_ecia_girq_status_clear(ctx->girq, ctx->girq_pos);
	soc_ecia_girq_ctrl(ctx->girq, ctx->girq_pos, 1);
}

/**
 * @brief Initialize the Queue and Data buffer thresholds
 *
 * @param regs Pointer to controller registers
 */
void XEC_I3C_Thresholds_Init(const struct device *dev)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;

	// Command Buffer Empty Threshold Value.
	xec_i3c_cmd_queue_threshold_set(regbase, 0x00);

	// Response Buffer Threshold Value.
	xec_i3c_resp_queue_threshold_set(regbase, 0x00);

	// IBI Data Threshold Value
	xec_i3c_ibi_data_threshold_set(regbase, 10);

	// In-Band Interrupt Status Threshold Value.
	xec_i3c_ibi_status_threshold_set(regbase, 0x00);

	// Transmit Buffer Threshold Value
	xec_i3c_tx_buf_threshold_set(regbase, DATA_BUF_THLD_FIFO_1);

	// Receive Buffer Threshold Value
	xec_i3c_rx_buf_threshold_set(regbase, DATA_BUF_THLD_FIFO_1);

	// Transfer Start Threshold Value
	xec_i3c_tx_start_threshold_set(regbase, DATA_BUF_THLD_FIFO_1);

	// Receive Start Threshold Value
	xec_i3c_rx_start_threshold_set(regbase, DATA_BUF_THLD_FIFO_1);

	// Notify Rejected Target Interrupt Request Control.
	xec_i3c_notify_sir_reject(regbase, false);

	// Notify Rejected Master Request Control.
	xec_i3c_notify_mr_reject(regbase, false);

	// Notify Rejected Hot-Join Control.
	xec_i3c_notify_hj_reject(regbase, false);
}

/**
 * @brief
 *
 * @param regs Pointer to controller registers
 */
void XEC_I3C_Host_Config(const struct device *dev)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;

	xec_i3c_host_dma_tx_burst_length_set(regbase, XEC_I3C_HC_CFG_DMA_BEAT_DW_4);

	xec_i3c_host_dma_rx_burst_length_set(regbase, XEC_I3C_HC_CFG_DMA_BEAT_DW_4);

	xec_i3c_host_port_set(regbase, XEC_I3C_HC_PORT_I3C1);

	xec_i3c_host_stuck_sda_config(regbase, HOST_CFG_STUCK_SDA_DISABLE, 0);

	xec_i3c_host_tx_dma_tout_config(regbase, XEC_I3C_CFG_DMA_TMOUT_DIS, 0);

	xec_i3c_host_rx_dma_tout_config(regbase, XEC_I3C_CFG_DMA_TMOUT_DIS, 0);
}

void XEC_I3C_Sec_Host_Config(const struct device *sec_dev)
{
	const struct xec_i3c_config *drvcfg = sec_dev->config;
	mm_reg_t regbase = drvcfg->regbase;

	xec_i3c_sec_host_dma_tx_burst_length_set(regbase, XEC_I3C_SC_CFG_DMA_BEAT_DW_4);

	xec_i3c_sec_host_dma_rx_burst_length_set(regbase, XEC_I3C_SC_CFG_DMA_BEAT_DW_4);

	xec_i3c_sec_host_port_set(regbase, XEC_I3C_SC_PORT_I3C0);

	xec_i3c_sec_host_stuck_sda_scl_config(regbase, SEC_HOST_STK_SDA_SCL_DIS, 0, 0);

	xec_i3c_sec_host_tx_dma_tout_config(regbase, XEC_I3C_CFG_DMA_TMOUT_DIS, 0);

	xec_i3c_sec_host_rx_dma_tout_config(regbase, XEC_I3C_CFG_DMA_TMOUT_DIS, 0);
}

/**
 * @brief configure timing for i3c transfers
 *
 * @param regs Pointer to controller registers
 */
void XEC_I3C_Soft_Reset(const struct device *host_dev)
{
	const struct xec_i3c_config *drvcfg = host_dev->config;
	mm_reg_t hrb = drvcfg->regbase;

	sys_set_bit(hrb + XEC_I3C_RST_CR_OFS, XEC_I3C_RST_CR_SRST_POS);

	/* wait for hardware to clear the bit */
	while (sys_test_bit(hrb + XEC_I3C_RST_CR_SRST_POS, XEC_I3C_RST_CR_SRST_POS) != 0) {
	}
}

/**
 * @brief Retrieve the depth of all queues
 *
 * @param regs Pointer to controller registers
 */
void XEC_I3C_queue_depths_get(const struct device *dev, struct queue_depths *fd)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;

	if (fd == NULL) {
		return;
	}

	fd->tx_fifo_depth = xec_i3c_tx_fifo_depth_get(regbase);
	fd->rx_fifo_depth = xec_i3c_rx_fifo_depth_get(regbase);
	fd->cmd_fifo_depth = xec_i3c_cmd_fifo_depth_get(regbase);
	fd->resp_fifo_depth = xec_i3c_resp_fifo_depth_get(regbase);
	fd->ibi_fifo_depth = xec_i3c_ibi_fifo_depth_get(regbase);
}

/**
 * @brief Enables the I3C Controller
 *
 * @param regs Pointer to controller registers
 * @param address 7-bit dynamic address for controller or
 *                7-bit static address for target
 * @param config configuration flags
 */
void XEC_I3C_Enable(const struct device *dev, uint8_t address, uint8_t config)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	uint8_t mode = 0;
	bool enable_dma = false;

	mode = XEC_I3C_OP_MODE_CTL;

	if (sbit_MODE_TARGET & config) {
		mode = XEC_I3C_OP_MODE_TGT;
		xec_i3c_static_addr_set(regbase, address);
	} else {
		xec_i3c_dynamic_addr_set(regbase, address);
	}

	if (sbit_HOTJOIN_DISABLE & config) {
		if (sbit_MODE_TARGET & config) {
			xec_i3c_tgt_hot_join_disable(regbase);
		} else {
			/* Disable Hot-Join */
			xec_i3c_hot_join_disable(regbase);
		}
	} else {
		if (!(sbit_MODE_TARGET & config)) {
			/* Enable IBI Interrupt */
			xec_i3c_intr_IBI_enable(regbase);
		}
	}

	if (sbit_CONFG_ENABLE & config) {
		/* Set Operation Mode */
		xec_i3c_operation_mode_set(regbase, mode);

		if (sbit_DMA_MODE & config) {
			enable_dma = true;
		}

		/* Enable the Controller */
		xec_i3c_enable(regbase, mode, enable_dma);

		/* For the target mode of operation, we are not programming
		 * the IDLE_CNT_MULTIPLIER and ADAPTIVE_I2C_I3C. Mostly likely
		 * the default values should be fine; Revisit later if we need to update
		 * them. Reference section 8.1.4 in user guide
		 */
	} else {
		/* Disable the Controller */
		xec_i3c_disable(regbase);
	}
}

/**
 * @brief Read one Device Characteristics Table (DCT) entry (ENTDAA format).
 *
 * The Device Characteristics Table (DCT) is stored in the controller's external RAM
 * and is populated by the controller during ENTDAA (Active Controller mode).
 * Each device consumes 4 x 32-bit words (16 bytes) in the DCT.
 *
 * The DCT base address (start offset in the controller external RAM window) is
 * provided by the DEVICE_CHAR_TABLE_POINTER register; the caller supplies that
 * start offset via @p DCT_start along with the device index @p DCT_idx.
 *
 * Layout parsed here matches the ENTDAA-captured structure:
 * - PID (48-bit, assembled from word0 and low 16 bits of word1)
 * - DCR (8-bit) and BCR (8-bit) from word2
 * - Dynamic address (7-bit) from word3
 *
 * @param dev       I3C controller device.
 * @param DCT_start Start offset/address of the DCT in external RAM (from DCT pointer register).
 * @param DCT_idx   Device index within the DCT.
 * @param info      Output decoded DCT information.
 */
void XEC_I3C_DCT_read(const struct device *dev, uint16_t DCT_start, uint16_t DCT_idx,
		      struct mec_i3c_DCT_info *info)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	uint32_t *entry_addr;
	uint64_t prov_id;

	/* Each DCT entry is 4 x 32-bit words (16 bytes) */
	entry_addr = (uint32_t *)((uintptr_t)regbase + (uintptr_t)DCT_start +
				  ((uintptr_t)DCT_idx * 4u * sizeof(uint32_t)));

	/* Word0 + low 16 bits of word1 form the 48-bit PID per existing format */
	prov_id = (uint64_t)entry_addr[0];
	info->pid = (prov_id << 16) | ((uint64_t)entry_addr[1] & 0xFFFFu);

	/* Word2: DCR in bits[7:0], BCR in bits[15:8] */
	info->dcr = (uint8_t)(entry_addr[2] & 0xFFu);
	info->bcr = (uint8_t)((entry_addr[2] >> 8) & 0xFFu);

	/* Word3: Dynamic address in bits[6:0] */
	info->dynamic_addr = (uint8_t)(entry_addr[3] & 0x7Fu);
}

/**
 * @brief Read the entries from SDCT and program DAT
 *
 * @param regs Pointer to controller registers
 * @param DCT_start Start address of DCT
 * @param targets_count Number of entries in the SDCT
 */
void XEC_I3C_TGT_DEFTGTS_DAT_write(const struct device *dev, uint16_t DCT_start, uint16_t DAT_start,
				   uint8_t targets_count)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	struct mec_i3c_SDCT_info sdct_info = {0};
	uint32_t val = 0;
	uint8_t i = 0;

	for (i = 0; i < targets_count; i++) {
		XEC_I3C_SDCT_read(dev, DCT_start, i, &sdct_info);

		val = DEV_ADDR_TABLE1_LOC1_DYNAMIC_ADDR(sdct_info.dynamic_addr);

		if (sdct_info.static_addr) {
			val |= DEV_ADDR_TABLE1_LOC1_STATIC_ADDR(sdct_info.static_addr);
		}

		xec_i3c_DAT_write(regbase, DAT_start, i, val);
	}
}

/**
 * @brief Read one Secondary Device Characteristics Table (SDCT) entry (DEFSLVS format).
 *
 * In Secondary Controller mode, the controller captures device characteristics received
 * during the DEFSLVS CCC into the Secondary Device Characteristics Table (SDCT), also
 * referred to in the spec as SEC_DEV_CHAR_TABLEx. These SDCT registers are implemented
 * in the controller’s external RAM window and share the same external RAM region/offsets
 * as the DEV_CHAR_TABLEx_LOCy registers (used in Active Controller mode).
 *
 * Each SDCT entry is a single 32-bit word with the following byte layout (SEC_DEV_CHAR_TABLE1):
 *   [31:24] STATIC_ADDR  - static address (if applicable; 0 if not present)
 *   [23:16] BCR_TYPE     - Bus Characteristics Register (BCR)
 *   [15:8]  DCR_TYPE     - Device Characteristics Register (DCR)
 *   [7:0]   DYNAMIC_ADDR - dynamic address assigned to the device
 *
 * The SDCT base address (start offset in external RAM) is provided by the device
 * characteristic table pointer register; the caller supplies that start offset via
 * @p DCT_start, and selects an entry via @p idx. Each entry is 4 bytes, so the entry
 * address is computed as: regbase + DCT_start + idx * 4.
 *
 * @param dev       I3C controller device (secondary controller instance).
 * @param DCT_start Start offset/address of the (S)DCT in external RAM.
 * @param idx       Device index within the SDCT.
 * @param info      Output decoded SDCT information.
 */
void XEC_I3C_SDCT_read(const struct device *dev, uint16_t DCT_start, uint16_t idx,
		       struct mec_i3c_SDCT_info *info)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	uint32_t sdct_val;
	uint32_t *entry_addr;

	/* One SDCT entry is one 32-bit word (4 bytes) per device */
	entry_addr = (uint32_t *)((uintptr_t)regbase + (uintptr_t)DCT_start +
				  ((uintptr_t)idx * sizeof(uint32_t)));

	sdct_val = *entry_addr;

	info->dynamic_addr = (uint8_t)(sdct_val & 0xFFu);
	info->dcr = (uint8_t)((sdct_val >> 8) & 0xFFu);
	info->bcr = (uint8_t)((sdct_val >> 16) & 0xFFu);
	info->static_addr = (uint8_t)((sdct_val >> 24) & 0xFFu);
}

/**
 * @brief Starts the DAA process using ENTDAA
 *
 * @param regs Pointer to controller registers
 * @param tgt_idx Device index of the first device in DAT that needs DAA
 * @param tgts_count Number of devices (from tgt_idx) that needs DAA process
 * @param tid_xfer Tid used for the transfer
 */
void XEC_I3C_DO_DAA(const struct device *dev, uint8_t tgt_idx, uint8_t tgts_count,
		    uint8_t *tid_xfer)
{
	struct xec_i3c_data *data = dev->data;
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	uint32_t command = 0;

	data->tid++;
	if (data->tid > 0) {
		data->tid = 0;
	}

	*tid_xfer = data->tid;

	command = (data->tid << COMMAND_AA_TID_BITPOS) | (tgt_idx << COMMAND_AA_DEV_IDX_BITPOS) |
		  (MEC_I3C_CCC_ENTDAA << COMMAND_AA_CMD_BITPOS) |
		  (tgts_count << COMMAND_AA_DEV_CNT_BITPOS) | COMMAND_STOP_ON_COMPLETION |
		  COMMAND_RESPONSE_ON_COMPLETION | COMMAND_ATTR_ADDR_ASSGN_CMD;

	/* Set Response Buffer Threshold as 1 entry */
	xec_i3c_resp_queue_threshold_set(regbase, 0);

	/* All required interrupts are already enabled? */

	/* Write the Command */
	sys_write32(command, regbase + XEC_I3C_CMD_OFS);
}

/**
 * @brief Sends the CCC on the I3C bus
 *
 * @param regs Pointer to controller registers
 * @param target Target that need to be sent CCC
 * @param tid_xfer Tid used for the transfer
 */
void XEC_I3C_DO_CCC(const struct device *dev, struct mec_i3c_DO_CCC *tgt, uint8_t *tid_xfer)
{
	struct xec_i3c_data *data = dev->data;
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	uint32_t command = 0, argument = 0;

	argument = COMMAND_ATTR_XFER_ARG;
	argument |= (tgt->data_len << COMMAND_XFER_ARG_DATA_LEN_BITPOS);
	if (tgt->defining_byte_valid) {
		argument |= (tgt->defining_byte << COMMAND_XFER_DEF_BYTE_BITPOS);
	}

	data->tid++;
	if (data->tid > 0) {
		data->tid = 0;
	}

	*tid_xfer = data->tid;

	command = (data->tid << COMMAND_TID_BITPOS) | (tgt->tgt_idx << COMMAND_DEV_IDX_BITPOS) |
		  (tgt->ccc_id << COMMAND_CMD_BITPOS) | COMMAND_STOP_ON_COMPLETION |
		  COMMAND_RESPONSE_ON_COMPLETION | COMMAND_CMD_PRESENT | COMMAND_ATTR_XFER_CMD;
	/* Note:
	 * Response Buffer Threshold is already set to 1 (in initialization)
	 */

	if (tgt->defining_byte_valid) {
		command |= COMMAND_DEF_BYTE_PRESENT;
	}

	if (tgt->read != 0) { // CCC Get
		command |= COMMAND_READ_XFER;
	} else { // CCC Set
		if (tgt->data_len != 0) {
			/* fill the TX FIFO with the data
			 * Note: We are not using Short Data Argument
			 */
			xec_i3c_fifo_write(regbase, tgt->data_buf, tgt->data_len);
		}
	}

	/* Set Response Buffer Threshold as 1 entry */
	xec_i3c_resp_queue_threshold_set(regbase, 0);

	if (tgt->data_len || tgt->defining_byte_valid) {
		/* Write the transfer argument */
		sys_write32(argument, regbase + XEC_I3C_CMD_OFS);
	}

	/* All required interrupts are already enabled? */
	/* Write the Command */
	sys_write32(command, regbase + XEC_I3C_CMD_OFS);
}

/**
 * @brief Prepare for private transfer
 * This function will form the command and argument DS
 * and write data to the tx FIFO in case of private
 * write
 *
 * @param regs Pointer to controller registers
 * @param target Target that need to be sent CCC
 * @param tid_xfer Tid used for the transfer
 */
void XEC_I3C_DO_Xfer_Prep(const struct device *dev, struct mec_i3c_dw_cmd *cmd, uint8_t *tid_xfer)
{
	struct xec_i3c_data *data = dev->data;
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	uint32_t command = 0, argument = 0;

	argument = (cmd->data_len << COMMAND_XFER_ARG_DATA_LEN_BITPOS) | COMMAND_ATTR_XFER_ARG;

	data->tid++;
	if (data->tid > 0) {
		data->tid = 0;
	}
	*tid_xfer = data->tid;

	command = (data->tid << COMMAND_TID_BITPOS) | (cmd->tgt_idx << COMMAND_DEV_IDX_BITPOS) |
		  (cmd->xfer_speed << COMMAND_SPEED_BITPOS) | COMMAND_RESPONSE_ON_COMPLETION |
		  COMMAND_ATTR_XFER_CMD;

	if (true == cmd->stop) {
		/* Send STOP */
		command |= COMMAND_STOP_ON_COMPLETION;
	} else {
		// Send REPEATED START
	}

	if (true == cmd->pec_en) {
		/* Calculate PEC */
		command |= COMMAND_PACKET_ERROR_CHECK;
	} else {
		/* Do not calculate PEC */
	}
	/* Note:
	 * Response Buffer Threshold is already set to 1 (in initialization)
	 */

	if (cmd->read != 0) { // Read data
		command |= COMMAND_READ_XFER;
		if (MEC_XFER_SPEED_HDR_DDR == cmd->xfer_speed) {
			command |= (COMMAND_CMD_PRESENT | COMMAND_HDR_DDR_READ);
		}
	} else { // Write data
		/* fill the TX FIFO with the data
		 * Note: We are not using Short Data Argument
		 */
		if (MEC_XFER_SPEED_HDR_DDR == cmd->xfer_speed) {
			command |= (COMMAND_CMD_PRESENT | COMMAND_HDR_DDR_WRITE);
		}

		xec_i3c_fifo_write(regbase, cmd->data_buf, cmd->data_len);
	}

	cmd->cmd = command;
	cmd->arg = argument;
}

void XEC_I3C_DO_Xfer(const struct device *dev, struct mec_i3c_dw_cmd *tgt)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;

	/* Write the transfer argument */
	sys_write32(tgt->arg, regbase + XEC_I3C_CMD_OFS);

	sys_write32(tgt->cmd, regbase + XEC_I3C_CMD_OFS);
}

void XEC_I3C_DO_TGT_Xfer(const struct device *dev, uint8_t *data_buf, uint16_t data_len)
{
	struct xec_i3c_data *data = dev->data;
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	uint32_t command = 0;

	data->tid++;
	if (data->tid > 0) {
		data->tid = 0;
	}

	/* fill the TX FIFO with the data */
	xec_i3c_fifo_write(regbase, data_buf, data_len);

	command = ((data_len << COMMAND_XFER_ARG_DATA_LEN_BITPOS) |
		   (data->tid << COMMAND_TID_BITPOS));
	/*Note: Command Attribute is 0 - Transmit Command without IBI */

	/* Write the transfer command */
	sys_write32(command, regbase + XEC_I3C_CMD_OFS);
}

/**
 * @brief Enables the IBI SIR for the target
 *
 * @param regs Pointer to controller registers
 * @param ibi_sir_info Information required to enable IBI SIR on a target
 * @param ibi_sir_info Flag to indicate if IBI interrupt needs to be enabled
 */
void XEC_I3C_IBI_SIR_Enable(const struct device *dev, struct mec_i3c_IBI_SIR *ibi_sir_info,
			    bool enable_ibi_interrupt)
{
	struct xec_i3c_data *data = dev->data;
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	uint32_t dat_value = 0;

	/* Following sequence is for Controller only configuration
	 * For secondary controller need to program 32-bit vector control
	 * register (IBI_SIR_REQ_REJECT).
	 * See section 2.6.3.3.1 1 in Databook
	 */

	/* Get the Dat entry */
	dat_value = xec_i3c_DAT_read(regbase, ibi_sir_info->DAT_start, ibi_sir_info->tgt_dat_idx);

	/* Enable IBI SIR */
	dat_value &= ~DEV_ADDR_TABLE1_LOC1_SIR_REJECT;

	if (ibi_sir_info->ibi_has_payload) {
		/* IBI with one or more mandatory bytes */
		dat_value |= DEV_ADDR_TABLE1_LOC1_IBI_WITH_DATA;
	}

	xec_i3c_DAT_write(regbase, ibi_sir_info->DAT_start, ibi_sir_info->tgt_dat_idx, dat_value);

	if ((0 == data->targets_ibi_enable_sts) && (enable_ibi_interrupt == true)) {
		/* IBI Data and Status thresholds are already set in initialization */
		xec_i3c_intr_IBI_enable(regbase);
	}

	data->targets_ibi_enable_sts |= (1 << ibi_sir_info->tgt_dat_idx);
}

/**
 * @brief Enables the IBI SIR for the target
 *
 * @param regs Pointer to controller registers
 * @param ibi_sir_info Information required to disable IBI SIR on a target
 * @param disable_ibi_interrupt Flag to indicate if IBI interrupt  can be disabled
 */
void XEC_I3C_IBI_SIR_Disable(const struct device *dev, struct mec_i3c_IBI_SIR *ibi_sir_info,
			     bool disable_ibi_interrupt)
{
	struct xec_i3c_data *data = dev->data;
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	uint32_t dat_value = 0;

	/* Get the Dat entry */
	dat_value = xec_i3c_DAT_read(regbase, ibi_sir_info->DAT_start, ibi_sir_info->tgt_dat_idx);

	/* Disable IBI SIR */
	dat_value |= DEV_ADDR_TABLE1_LOC1_SIR_REJECT;

	xec_i3c_DAT_write(regbase, ibi_sir_info->DAT_start, ibi_sir_info->tgt_dat_idx, dat_value);

	data->targets_ibi_enable_sts &= (uint32_t)~(1 << ibi_sir_info->tgt_dat_idx);

	if ((0 == data->targets_ibi_enable_sts) && (disable_ibi_interrupt == true)) {
		xec_i3c_intr_IBI_disable(regbase);
	}
}

/**
 * @brief Set the MIPI PID value for target
 *
 * @param regs Pointer to controller registers
 * @param pid 48-bit PID value
 */
void XEC_I3C_TGT_PID_set(const struct device *dev, uint64_t pid, bool pid_random)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;

	xec_i3c_tgt_pid_set(regbase, TGT_MIPI_MFG_ID(pid), pid_random, TGT_PART_ID(pid),
			    TGT_INST_ID(pid), TGT_PID_DCR(pid));
}

/**
 * @brief Set the MWL value for target
 *
 * @param regs Pointer to controller registers
 * @param uint8_t wr_speed maximum write speed, refer enum mec_mxds_max_wr_speed
 * @param uint8_t rd_speed maximum read speed, refer enum mec_mxds_max_rd_speed
 * @param uint8_t tsco clock to data turnaround time, refer enum mec_mxds_tsco
 */
void XEC_I3C_TGT_MXDS_set(const struct device *dev, uint8_t wr_speed, uint8_t rd_speed,
			  uint8_t tsco, uint32_t rd_trnd_us)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t regbase = drvcfg->regbase;
	uint32_t msk = XEC_I3C_SC_MXDS_MWS_MSK | XEC_I3C_SC_MXDS_MRS_MSK | XEC_I3C_SC_MXDS_CDT_MSK;
	uint32_t val = XEC_I3C_SC_MXDS_MWS_SET((uint32_t)wr_speed) |
		       XEC_I3C_SC_MXDS_MRS_SET((uint32_t)rd_speed) |
		       XEC_I3C_SC_MXDS_CDT_SET((uint32_t)tsco);
	uint32_t val2 = XEC_I3C_SC_MXRT_MRT_SET(rd_trnd_us);

	soc_mmcr_mask_set(regbase + XEC_I3C_SC_MXDS_OFS, val, msk);

	soc_mmcr_mask_set(regbase + XEC_I3C_SC_MXRT_OFS, val2, XEC_I3C_SC_MXRT_MRT_MSK);
}

/**
 * @brief Issues the IBI SIR for the target
 *
 * @param regs Pointer to controller registers
 * @param ibi_sir_info Information required to enable IBI SIR on a target
 */
int XEC_I3C_TGT_IBI_SIR_Raise(const struct device *dev,
			      struct mec_i3c_raise_IBI_SIR *ibi_sir_request)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t srb = drvcfg->regbase;
	int ret = 0;

	/* Ensure Controller has enabled TIR for the target (us) */
	if (sys_test_bit(srb + XEC_I3C_SC_TEVT_SR_OFS, XEC_I3C_SC_TEVT_SR_TIR_EN_POS) != 0) {
		/* Raise IBI TIR */
		xec_i3c_tgt_raise_ibi_SIR(srb, ibi_sir_request->data_buf, ibi_sir_request->data_len,
					  ibi_sir_request->mdb);
	} else {
		ret = 1;
	}

	return ret;
}

/**
 * @brief Issues the IBI Master Request for the target
 *
 * @param regs Pointer to controller registers
 */
int XEC_I3C_TGT_IBI_MR_Raise(const struct device *dev)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t srb = drvcfg->regbase;
	uint8_t ret = 0;

	/* Ensure Controller has enabled MR for the target (us) */
	if (sys_test_bit(srb + XEC_I3C_SC_TEVT_SR_OFS, XEC_I3C_SC_TEVT_SR_HR_EN_POS) != 0) {
		/* Raise IBI SIR*/
		sys_set_bit(srb + XEC_I3C_SC_TIREQ_OFS, XEC_I3C_SC_TIREQ_HR_POS);

		/* Enable ACK for GETACCMST CCC from Host */
		sys_clear_bit(srb + XEC_I3C_DEV_EXT_CR_OFS, XEC_I3C_DEV_EXT_CR_TM_NAK_CCC_POS);

	} else {
		ret = 1;
	}

	return ret;
}

/**
 * @brief Handles Target Raise IBI SIR Residual data
 *
 * @param regs Pointer to controller registers
 */
void XEC_I3C_TGT_IBI_SIR_Residual_handle(const struct device *dev)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t hrb = drvcfg->regbase;

	/* Clear SIR residual data by resettig TX Fifo */
	sys_set_bit(hrb + XEC_I3C_RST_CR_OFS, XEC_I3C_RST_CR_TX_FIFO_POS);

	/* TODO - above bit is self-clearing. Does one need to wait before resume? */

	/* Hit Resume */
	xec_i3c_resume(hrb);
}

/**
 * @brief Handles Target Raise IBI SIR Residual data
 *
 * @param regs Pointer to controller registers
 */
void XEC_I3C_TGT_Error_Recovery(const struct device *dev, uint8_t err_sts)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t hrb = drvcfg->regbase;

	if ((err_sts == TARGET_RESP_ERR_CRC) || (err_sts == TARGET_RESP_ERR_PARITY) ||
	    (err_sts == TARGET_RESP_ERR_UNDERFLOW_OVERFLOW)) {
		/* Reset RX Fifo */
		sys_set_bit(hrb + XEC_I3C_RST_CR_OFS, XEC_I3C_RST_CR_RX_FIFO_POS);
	} else {
		/* Reset TX FIFO and Command Queue */
		sys_set_bits(hrb + XEC_I3C_RST_CR_OFS,
			     BIT(XEC_I3C_RST_CR_TX_FIFO_POS) | BIT(XEC_I3C_RST_CR_CMDQ_POS));
	}

	/* TODO the above bits are self-clearing. Does one need to wait for them to clear
	 * before resuming?
	 */

	/* Hit Resume */
	xec_i3c_resume(hrb);
}

/**
 * @brief Handles switching of target to controller for role switch IBI
 *
 * @param regs Pointer to controller registers
 */
void XEC_I3C_TGT_RoleSwitch_Resume(const struct device *dev)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t hrb = drvcfg->regbase;
	uint32_t val = BIT(XEC_I3C_RST_CR_TX_FIFO_POS) | BIT(XEC_I3C_RST_CR_RX_FIFO_POS) |
		       BIT(XEC_I3C_RST_CR_CMDQ_POS);

	sys_set_bits(hrb + XEC_I3C_RST_CR_OFS, val);

	/* TODO Do we need to wait for HW to clearing resets before resume?
	 * EC keeps getting faster and our AHB may also be faster than previous chips.
	 */

	/* Hit Resume */
	xec_i3c_resume(hrb);
}

/**
 * @brief Resumes the controller and clears transfer error
 *
 * @param regs Pointer to controller registers
 */
void XEC_I3C_Xfer_Error_Resume(const struct device *dev)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t hrb = drvcfg->regbase;

	/* Hit Resume */
	xec_i3c_resume(hrb);

	/* Clear transfer error status */
	xec_i3c_xfer_err_sts_clr(hrb);
}

/**
 * @brief Resets the transfer fifos and queues
 *
 * @param regs Pointer to controller registers
 */
void XEC_I3C_Xfer_Reset(const struct device *dev)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	mm_reg_t hrb = drvcfg->regbase;
	uint32_t rstval = (BIT(XEC_I3C_RST_CR_CMDQ_POS) | BIT(XEC_I3C_RST_CR_RESPQ_POS) |
			   BIT(XEC_I3C_RST_CR_TX_FIFO_POS) | BIT(XEC_I3C_RST_CR_RX_FIFO_POS));
	uint32_t rv = 0;

	/* Reset the TX/RX Fifos & Cmd/Res Queues */
	sys_write32(rstval, hrb + XEC_I3C_RST_CR_OFS);

	/* wait for HW to clear all the reset bits we set */
	do {
		rv = sys_read32(hrb + XEC_I3C_RST_CR_OFS) & rstval;
	} while (rv != 0);
}

/* Interrupt functions */
/*--------------------------------------------------------*/

/* Enable/disable I23 controller interrupt signal from propagating to NVIC */
void XEC_I3C_GIRQ_CTRL(const struct device *dev, int flags)
{
	const struct xec_i3c_config *drvcfg = dev->config;

	if (flags & MEC_I3C_GIRQ_DIS) {
		soc_ecia_girq_ctrl(drvcfg->hwctx.girq, drvcfg->hwctx.girq_pos, 0);
	}

	if (flags & MEC_I3C_GIRQ_CLR_STS) {
		soc_ecia_girq_status_clear(drvcfg->hwctx.girq, drvcfg->hwctx.girq_pos);
	}

	if (flags & MEC_I3C_GIRQ_EN) {
		soc_ecia_girq_ctrl(drvcfg->hwctx.girq, drvcfg->hwctx.girq_pos, 1);
	}
}

int XEC_I3C_GIRQ_Status(const struct device *dev)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	uint32_t status = 0;

	soc_ecia_girq_status(drvcfg->hwctx.girq, &status);

	return (int)status;
}

int XEC_I3C_GIRQ_Result(const struct device *dev)
{
	const struct xec_i3c_config *drvcfg = dev->config;
	uint32_t result = 0;

	soc_ecia_girq_result(drvcfg->hwctx.girq, &result);

	return (int)result;
}

/* End of file */
