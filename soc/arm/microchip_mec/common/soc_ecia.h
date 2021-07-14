/*
 * Copyright (c) 2021 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip XEC MCU family EC Interrupt Aggregator (ECIA) defines.
 *
 */

#ifndef _MICROCHIP_MEC_SOC_ECIA_H_
#define _MICROCHIP_MEC_SOC_ECIA_H_

#define MCHP_XEC_ECIA_GIRQ_FIRST	8u
#define MCHP_XEC_ECIA_GIRQ_LAST		26u

void mchp_soc_ecia_girq_src_clr(uint8_t girq_num, uint8_t src_bit_pos);
void mchp_soc_ecia_girq_src_en(uint8_t girq_num, uint8_t src_bit_pos);
void mchp_soc_ecia_girq_src_dis(uint8_t girq_num, uint8_t src_bit_pos);

void mchp_soc_ecia_girq_src_clr_bitmap(uint8_t girq_num, uint32_t bitmap);
void mchp_soc_ecia_girq_src_en_bitmap(uint8_t girq_num, uint32_t bitmap);
void mchp_soc_ecia_girq_src_dis_bitmap(uint8_t girq_num, uint32_t bitmap);

uint32_t mchp_soc_ecia_girq_result(uint8_t girq_num);

/* enable or disable GIRQ aggregated output */
void mchp_soc_ecia_girq_aggr_en(uint8_t girq_num, uint8_t enable);

/* Clear NVIC external input pending bit */
void mchp_soc_ecia_nvic_clr_pend(uint32_t nvic_num);

#endif /* _MICROCHIP_MEC_SOC_ECIA_H_ */
