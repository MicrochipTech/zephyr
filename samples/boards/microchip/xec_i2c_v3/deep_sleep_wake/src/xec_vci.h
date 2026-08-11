/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _XEC_VCI_H
#define _XEC_VCI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Interfaces to any C modules */
#ifdef __cplusplus
extern "C" {
#endif

#define MCHP_XEC_VCI_IN0_ID 0
#define MCHP_XEC_VCI_IN1_ID 1U
#define MCHP_XEC_VCI_IN2_ID 2U
#define MCHP_XEC_VCI_IN3_ID 3U
#define MCHP_XEC_VCI_IN4_ID 4U
#define MCHP_XEC_VCI_IN5_ID 5U
#define MCHP_XEC_VCI_IN_ID_MAX 6U

enum mec_vci_sel {
	MEC_VCI_IN0_POS = 0,
	MEC_VCI_IN1_POS,
	MEC_VCI_IN2_POS,
	MEC_VCI_IN3_POS,
	MEC_VCI_IN4_POS,
	MEC_VCI_IN5_POS,
	MEC_VCI_IN6_POS,
	MEC_VCI_OVRD_IN_POS = 8,
	MEC_VCI_OUT_POS,
	MEC_VCI_IN_WEEK_ALARM_POS = 16,
	MEC_VCI_IN_RTC_ALARM_POS,
};

int mchp_xec_vci_pin_disable(uint8_t vci_id);

/* Return current state of VCI pin inputs. If latching is enabled
 * the current state is the latched state otherwise the state is
 * live pin after filtering and polarity are applied.
 * b[6:0] = VCI_IN[6:0]
 * b[8] = current VCI_OVRD_IN state
 * b[9] = current VCI_OUT state
 * b[16] = Week Alarm state
 * b[17] = RTC Alaram state
 */
uint32_t mchp_vci_in_pin_states(void);

/* VCI_IN pins filter enable/disable */
void mchp_vci_in_filter_enable(uint8_t enable);

uint8_t mchp_vci_out_get(void);
uint8_t mchp_vci_ovrd_in_get(void);

/* Enable software control of VCI_OUT pin state */
void mchp_vci_sw_vci_out_enable(uint8_t enable);

/* set the state of software controlled VCI_OUT pin state
 * This value has no effect on VCI_OUT pin unless the FW_EXT bit is 1.
 */
void mchp_vci_sw_vci_out_set(uint8_t pin_state);

/* Return bitmap of VCI_IN[6:0] input latched state */
uint32_t mchp_vci_in_latched_get(void);
void mchp_vci_in_latch_enable(uint32_t latch_bitmap);
void mchp_vci_in_latch_disable(uint32_t latch_bitmap);
uint32_t mchp_vci_in_latch_enable_get(void);

/* clear latched state of selected VCI inputs */
void mchp_vci_in_latch_reset(uint32_t latch_bitmap);

/* Set/get VCI_IN[] input enables */
void mchp_vci_in_input_enable(uint32_t latch_bitmap);
uint32_t mchp_vci_in_input_enable_get(void);

/* Program the delay after nSYS_SHDN asserts before VCI logic re-asserts
 * VCI_OUT. A value of zero disables the delay. Non-zero values should
 * be in the range [125, 32000] milliseconds.
 */
int mchp_vci_out_power_on_delay(uint32_t delay_ms);

/* Set the polarity of selected VCI_IN[n] pins.
 * Polarity = 1 Active High
 *          = 0 Active Low
 */
void mchp_vci_in_polarity(uint32_t vci_in_bitmap, uint32_t polarity_bitmap);

/* Return bitmap of detected positive edges on VCI_IN[] pins */
uint32_t mchp_vci_pedge_detect(void);

/* Return bitmap of detected negative edges on VCI_IN[] pins */
uint32_t mchp_vci_nedge_detect(void);

/* Clear edge detection logic */
void mchp_vci_pedge_detect_clr(uint32_t bitmap);
void mchp_vci_nedge_detect_clr(uint32_t bitmap);
void mchp_vci_edge_detect_clr_all(void);

/* Select which VCI_IN[] pin edge detector are enabled when the chip
 * is powered only by the VBAT power rail (VTR Core is off).
 * When the chip is on (VTR Core ON) this register has no effect on
 * the edge detector enables.
 */
uint32_t mchp_vci_vbat_edge_detect_get(void);
void mchp_vci_vbat_edge_detect_enable(uint32_t en_bitmap, uint32_t en_mask);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _XEC_VCI_H */
