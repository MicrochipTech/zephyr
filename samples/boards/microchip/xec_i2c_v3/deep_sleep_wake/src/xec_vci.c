/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdint.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/pinctrl.h>

#include "xec_vci.h"

#define XEC_VCI_NODE DT_NODELABEL(vci0)

#define XEC_VCI_REG_BASE (mm_reg_t)(DT_REG_ADDR(XEC_VCI_NODE))

#define XEC_VCI_CR_OFS            0
#define XEC_VCI_LATCH_EN_OFS      4U
#define XEC_VCI_LATCH_RST_OFS     8U
#define XEC_VCI_INPUT_EN_OFS      0xCU
#define XEC_VCI_HOLD_OFF_CNT_OFS  0x10U
#define XEC_VCI_IN_POLARITY_OFS   0x14U
#define XEC_VCI_IN_POSEDG_STS_OFS 0x18U
#define XEC_VCI_IN_NEGEDG_STS_OFS 0x1CU
#define XEC_VCI_IN_VBAT_BUFEN_OFS 0x20U

/* Control register */
#define XEC_VCI_CR_LIN_POS          0 /* R/O latched input states */
#define XEC_VCI_CR_LIN_MSK          GENMASK(6, 0)
#define XEC_VCI_CR_LIN_GET(r)       FIELD_GET(XEC_VCI_CR_LIN_MSK, (r))
#define XEC_VCI_CR_OVRD_IN_POS      8 /* R/O current VCI_OVRD_IN pin state*/
#define XEC_VCI_CR_OUTV_POS         9 /* R/O current VCI_OUT pin state */
#define XEC_VCI_CR_FW_OUTV_POS      10 /* R/W VCI_OUT pin state under FW control */
#define XEC_VCI_CR_FW_EN_POS        11 /* R/W enable FW control of VCI_OUT pin state */
#define XEC_VCI_CR_FILT_BYPASS_POS  12 /* R/W bypass(disable) input filters */
#define XEC_VCI_CR_WK_ALRM_STS_POS  16 /* R/O Week alarm asserted WEEK_ALRM_LS signal */
#define XEC_VCI_CR_RTC_ALRM_STS_POS 17 /* R/O RTC asserted its alarm signal */
#define XEC_VCI_CR_SEL_SYSPP_POS    18 /* R/W 1=VCI_IN3 becomes SYSPWR_PRES */

#define XEC_VCI_CR_ALL_IN_STATE_MSK \
	(XEC_VCI_CR_LIN_MSK | BIT(XEC_VCI_CR_OVRD_IN_POS) | BIT(XEC_VCI_CR_OUTV_POS) |\
	 BIT(XEC_VCI_CR_WK_ALRM_STS_POS) | BIT(XEC_VCI_CR_RTC_ALRM_STS_POS))

/* Latch enable and Latch reset registers */
#define XEC_VCI_LER_IN_POS       0 /* Latch enable/reset for VCI_IN pins */
#define XEC_VCI_LER_IN_MSK       GENMASK(6, 0)
#define XEC_VCI_LER_SET(v)       FIELD_PREP(XEC_VCI_LER_IN_MSK, (v))
#define XEC_VCI_LER_GET(r)       FIELD_GET(XEC_VCI_LER_IN_MSK, (r))
#define XEC_VCI_LER_WK_ALRM_POS  16
#define XEC_VCI_LER_RTC_ALRM_POS 17

#define XEC_VCI_LER_ALL_MSK \
	(XEC_VCI_LER_IN_MSK | BIT(XEC_VCI_LER_WK_ALRM_POS) | BIT(XEC_VCI_LER_RTC_ALRM_POS))

/* Input enable register */
#define XEC_VCI_INPUT_EN_VIN_POS    0
#define XEC_VCI_INPUT_EN_VIN_MSK    GENMASK(6, 0)
#define XEC_VCI_INPUT_EN_VIN_SET(v) FIELD_PREP(XEC_VCI_INPUT_EN_VIN_MSK, (v))
#define XEC_VCI_INPUT_EN_VIN_GET(r) FIELD_GET(XEC_VCI_INPUT_EN_VIN_MSK, (r))

/* Hold off count register. Units of 125 ms */
#define XEC_VCI_HOLD_OFF_CNT_TM_POS    0
#define XEC_VCI_HOLD_OFF_CNT_TM_MSK    GENMASK(7, 0)
#define XEC_VCI_HOLD_OFF_CNT_TM_SET(v) FIELD_PREP(XEC_VCI_HOLD_OFF_CNT_TM_MSK, (v))
#define XEC_VCI_HOLD_OFF_CNT_TM_GET(r) FIELD_GET(XEC_VCI_HOLD_OFF_CNT_TM_MSK, (r))

/* Input Polarity register */
#define XEC_VCI_IN_POLARITY_POS    0
#define XEC_VCI_IN_POLARITY_MSK    GENMASK(6, 0)
#define XEC_VCI_IN_POLARITY_SET(v) FIELD_PREP(XEC_VCI_IN_POLARITY_MSK, (v))
#define XEC_VCI_IN_POLARITY_GET(r) FIELD_GET(XEC_VCI_IN_POLARITY_MSK, (r))

/* Positive and Negative Edge detect registers */
#define XEC_VCI_EDG_DET_POS    0
#define XEC_VCI_EDG_DET_MSK    GENMASK(6, 0)
#define XEC_VCI_EDG_DET_SET(v) FIELD_PREP(XEC_VCI_EDG_DET_MSK, (v))
#define XEC_VCI_EDG_DET_GET(r) FIELD_GET(XEC_VCI_EDG_DET_MSK, (r))

/* Input VBAT buffer enable register */
#define XEC_VCI_IN_VB_EN_POS    0
#define XEC_VCI_IN_VB_EN_MSK    GENMASK(6, 0)
#define XEC_VCI_IN_VB_EN_SET(v) FIELD_PREP(XEC_VCI_IN_VB_EN_MSK, (v))
#define XEC_VCI_IN_VB_EN_GET(r) FIELD_GET(XEC_VCI_IN_VB_EN_MSK, (r))

/* VCI_IN[0:6] are always bits[0:5] in registers referencing them */
#define XEC_VCI_IN_0_6_MASK 0x3fu

/* -------- VBAT Powered Control Interface (VCI) API -------- */

int mchp_xec_vci_pin_disable(uint8_t vci_id)
{
	mm_reg_t base = XEC_VCI_REG_BASE;

	if (vci_id >= MCHP_XEC_VCI_IN_ID_MAX) {
		return -EINVAL;
	}

	sys_clear_bit(base + XEC_VCI_INPUT_EN_OFS, vci_id);
	sys_clear_bit(base + XEC_VCI_IN_VBAT_BUFEN_OFS, vci_id);

	return 0;
}

/* VCI_IN pins filter enable/disable */
void mchp_vci_in_filter_enable(uint8_t enable)
{
	mm_reg_t base = XEC_VCI_REG_BASE;

	if (enable != 0) {
		sys_clear_bit(base + XEC_VCI_CR_OFS, XEC_VCI_CR_FILT_BYPASS_POS);
	} else {
		sys_set_bit(base + XEC_VCI_CR_OFS, XEC_VCI_CR_FILT_BYPASS_POS);
	}
}

/* Return current state of VCI pin inputs. If latching is enabled
 * the current state is the latched state otherwise the state is
 * live pin after filtering and polarity are applied.
 * b[6:0] = VCI_IN[6:0]
 * b[8] = current VCI_OVRD_IN state
 * b[9] = current VCI_OUT state
 * b[16] = Week Alarm state
 * b[17] = RTC Alaram state
 */
uint32_t mchp_vci_in_pin_states(void)
{
	mm_reg_t base = XEC_VCI_REG_BASE;

	return sys_read32(base + XEC_VCI_CR_OFS) & XEC_VCI_CR_ALL_IN_STATE_MSK;
}

uint8_t mchp_vci_out_get(void)
{
	mm_reg_t base = XEC_VCI_REG_BASE;

	if ((sys_test_bit(base + XEC_VCI_CR_OFS, XEC_VCI_CR_OUTV_POS)) != 0) {
		return 1U;
	}

	return 0;
}

uint8_t mchp_vci_ovrd_in_get(void)
{
	mm_reg_t base = XEC_VCI_REG_BASE;

	if ((sys_test_bit(base + XEC_VCI_CR_OFS, XEC_VCI_CR_OVRD_IN_POS)) != 0) {
		return 1U;
	}

	return 0;
}

/* Enable software control of VCI_OUT pin state */
void mchp_vci_sw_vci_out_enable(uint8_t enable)
{
	mm_reg_t base = XEC_VCI_REG_BASE;

	if (enable != 0) {
		sys_set_bit(base + XEC_VCI_CR_OFS, XEC_VCI_CR_FW_EN_POS);
	} else {
		sys_clear_bit(base + XEC_VCI_CR_OFS, XEC_VCI_CR_FW_EN_POS);
	}
}

/* set the state of software controlled VCI_OUT pin state
 * This value has no effect on VCI_OUT pin unless the FW_EXT bit is 1.
 */
void mchp_vci_sw_vci_out_set(uint8_t pin_state)
{
	mm_reg_t base = XEC_VCI_REG_BASE;

	if (pin_state != 0) {
		sys_set_bit(base + XEC_VCI_CR_OFS, XEC_VCI_CR_FW_OUTV_POS);
	} else {
		sys_clear_bit(base + XEC_VCI_CR_OFS, XEC_VCI_CR_FW_OUTV_POS);
	}
}

uint32_t mchp_vci_in_latched_get(void)
{
	mm_reg_t base = XEC_VCI_REG_BASE;
	uint32_t r = sys_read32(base + XEC_VCI_CR_OFS);

	return XEC_VCI_CR_LIN_GET(r);
}

void mchp_vci_in_latch_enable(uint32_t latch_bitmap)
{
	mm_reg_t base = XEC_VCI_REG_BASE;
	uint32_t v = XEC_VCI_LER_SET(latch_bitmap);

	sys_set_bits(base + XEC_VCI_LATCH_EN_OFS, v);
}

void mchp_vci_in_latch_disable(uint32_t latch_bitmap)
{
	mm_reg_t base = XEC_VCI_REG_BASE;
	uint32_t v = XEC_VCI_LER_SET(latch_bitmap);

	sys_clear_bits(base + XEC_VCI_LATCH_EN_OFS, v);
}

uint32_t mchp_vci_in_latch_enable_get(void)
{
	mm_reg_t base = XEC_VCI_REG_BASE;
	uint32_t r = sys_read32(base + XEC_VCI_LATCH_EN_OFS);

	return XEC_VCI_LER_GET(r);
}

void mchp_vci_in_latch_reset(uint32_t latch_bitmap)
{
	mm_reg_t base = XEC_VCI_REG_BASE;

	sys_write32(latch_bitmap & XEC_VCI_LER_ALL_MSK, base + XEC_VCI_LATCH_RST_OFS);
}

void mchp_vci_in_input_enable(uint32_t latch_bitmap)
{
	mm_reg_t base = XEC_VCI_REG_BASE;

	sys_write32(XEC_VCI_INPUT_EN_VIN_SET(latch_bitmap), base + XEC_VCI_INPUT_EN_OFS);
	sys_write32(XEC_VCI_LER_SET(latch_bitmap), base + XEC_VCI_LATCH_RST_OFS);
}

uint32_t mchp_vci_in_input_enable_get(void)
{
	mm_reg_t base = XEC_VCI_REG_BASE;
	uint32_t r = sys_read32(base + XEC_VCI_INPUT_EN_OFS);

	return XEC_VCI_INPUT_EN_VIN_GET(r);
}

int mchp_vci_out_power_on_delay(uint32_t delay_ms)
{
	mm_reg_t base = XEC_VCI_REG_BASE;
	uint32_t delay_cnt = 0, v = 0;

	if (delay_ms == 0) { /* disable */
		v = 0;
	} else if ((delay_ms >= 125u) || (delay_ms <= (32u * 1000u))) {
		delay_cnt = delay_ms / 125u;
		if (((delay_cnt % 125u) > 62u) && (delay_cnt < 0xffu)) {
			delay_cnt++;
		}
	} else {
		return -ERANGE;
	}

	v = XEC_VCI_HOLD_OFF_CNT_TM_SET(v);
	soc_mmcr_mask_set(base + XEC_VCI_HOLD_OFF_CNT_OFS, v, XEC_VCI_HOLD_OFF_CNT_TM_MSK);

	return 0;
}

/* Set the polarity of selected VCI_IN[n] pins.
 * Polarity = 1 Active High
 *          = 0 Active Low
 */
void mchp_vci_in_polarity(uint32_t vci_in_bitmap, uint32_t polarity_bitmap)
{
	mm_reg_t base = XEC_VCI_REG_BASE;
	uint32_t v = sys_read32(base + XEC_VCI_IN_POLARITY_OFS);

	v &= (uint32_t)~(vci_in_bitmap);
	v |= (polarity_bitmap & vci_in_bitmap);
	sys_write32(v, base + XEC_VCI_IN_POLARITY_OFS);
}

uint32_t mchp_vci_pedge_detect(void)
{
	mm_reg_t base = XEC_VCI_REG_BASE;

	return XEC_VCI_EDG_DET_GET(sys_read32(base + XEC_VCI_IN_POSEDG_STS_OFS));
}

uint32_t mchp_vci_nedge_detect(void)
{
	mm_reg_t base = XEC_VCI_REG_BASE;

	return XEC_VCI_EDG_DET_GET(sys_read32(base + XEC_VCI_IN_NEGEDG_STS_OFS));
}

void mchp_vci_pedge_detect_clr(uint32_t bitmap)
{
	mm_reg_t base = XEC_VCI_REG_BASE;
	uint32_t v = XEC_VCI_EDG_DET_SET(bitmap);

	sys_write32(v, base + XEC_VCI_IN_POSEDG_STS_OFS);
}

void mchp_vci_nedge_detect_clr(uint32_t bitmap)
{
	mm_reg_t base = XEC_VCI_REG_BASE;
	uint32_t v = XEC_VCI_EDG_DET_SET(bitmap);

	sys_write32(v, base + XEC_VCI_IN_NEGEDG_STS_OFS);
}

void mchp_vci_edge_detect_clr_all(void)
{
	mm_reg_t base = XEC_VCI_REG_BASE;

	sys_write32(XEC_VCI_EDG_DET_MSK, base + XEC_VCI_IN_POSEDG_STS_OFS);
	sys_write32(XEC_VCI_EDG_DET_MSK, base + XEC_VCI_IN_NEGEDG_STS_OFS);
}

/* Select which VCI_IN[] pin edge detectors are enabled when the chip
 * is powered only by the VBAT power rail (VTR Core is off).
 * When the chip is on (VTR Core ON) this register has no effect on
 * the edge detector enables.
 */
uint32_t mchp_vci_vbat_edge_detect_get(void)
{
	mm_reg_t base = XEC_VCI_REG_BASE;
	uint32_t r = sys_read32(base + XEC_VCI_IN_VBAT_BUFEN_OFS);

	return XEC_VCI_IN_VB_EN_GET(r);
}

void mchp_vci_vbat_edge_detect_enable(uint32_t en_bitmap, uint32_t en_mask)
{
	mm_reg_t base = XEC_VCI_REG_BASE;
	uint32_t en = XEC_VCI_IN_VB_EN_SET(en_bitmap);
	uint32_t msk = XEC_VCI_IN_VB_EN_SET(en_mask);

	soc_mmcr_mask_set(base + XEC_VCI_IN_VBAT_BUFEN_OFS, en, msk);
}

/* end xec_vci.c */
