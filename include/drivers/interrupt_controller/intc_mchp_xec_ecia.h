/*
 * Copyright (c) 2021 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for External interrupt controller in Microchip XEC devices
 *
 * Based on reference manuals:
 *   Reference Manuals for MEC152x and MEC172x ARM(r) 32-bit MCUs
 *
 * Chapter: EC Interrupt Aggregator (ECIA)
 *
 */

#ifndef ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_MCHP_XEC_ECIA_H_
#define ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_MCHP_XEC_ECIA_H_

#include <zephyr/types.h>

/**
 * @brief enable GIRQn interrupt for specific source
 *
 * @param girq is the GIRQ number (8 - 26)
 * @param src is the interrupt source in the GIRQ (0 - 31)
 */
int mchp_xec_ecia_enable(int girq, int src);

/**
 * @brief disable EXTI interrupt for specific line
 *
 * @param girq is the GIRQ number (8 - 26)
 * @param src is the interrupt source in the GIRQ (0 - 31)
 */
int mchp_xec_ecia_disable(int girq, int src);


/* callback for ECIA GIRQ interrupt source */
typedef void (*mchp_xec_ecia_callback_t) (int girq, int src, void *user);

/**
 * @brief set GIRQn interrupt source callback
 *
 * @param girq is the GIRQ number (8 - 26)
 * @param src is the interrupt source in the GIRQ (0 - 31)
 * @param cb user callback
 * @param data user data
 */
int mchp_xec_ecia_set_callback(int girq, int src, mchp_xec_ecia_callback_t cb,
			       void *data);

/**
 * @brief unset GIRQn interrupt source callback
 *
 * @param girq is the GIRQ number (8 - 26)
 * @param src is the interrupt source in the GIRQ (0 - 31)
 */
int mchp_ecia_unset_callback(int girq, int src);

/* platform specific */
void mchp_xec_ecia_girq_aggr_en(uint8_t girq, uint8_t enable);
void mchp_xec_ecia_girq_src_clr(uint8_t girq, uint8_t src_bit_pos);
void mchp_xec_ecia_girq_src_en(uint8_t girq, uint8_t src_bit_pos);
void mchp_xec_ecia_girq_src_dis(uint8_t girq, uint8_t src_bit_pos);
void mchp_xec_ecia_girq_src_clr_bitmap(uint8_t girq, uint32_t bitmap);
void mchp_xec_ecia_girq_src_en_bitmap(uint8_t girq, uint32_t bitmap);
void mchp_xec_ecia_girq_src_dis_bitmap(uint8_t girq, uint32_t bitmap);
uint32_t mchp_xec_ecia_girq_result(uint8_t girq);
void mchp_xec_ecia_nvic_clr_pend(uint32_t nvic_num);

#endif /* ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_MCHP_XEC_ECIA_H_ */
