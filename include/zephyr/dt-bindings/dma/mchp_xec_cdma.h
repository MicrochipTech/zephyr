/*
 * Copyright (c) 2026 Microchip Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_DMA_MCHP_XEC_CDMA_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_DMA_MCHP_XEC_CDMA_H_

/**
 * @name custom DMA flags for channel configuration
 * @{
 */
/** DMA peripher device source IDs */
#define MCHP_XEC_DMA_I2C0_TARG  0
#define MCHP_XEC_DMA_I2C0_HOST  1
#define MCHP_XEC_DMA_I2C1_TARG  2
#define MCHP_XEC_DMA_I2C1_HOST  3
#define MCHP_XEC_DMA_I2C2_TARG  4
#define MCHP_XEC_DMA_I2C2_HOST  5
#define MCHP_XEC_DMA_I2C3_TARG  6
#define MCHP_XEC_DMA_I2C3_HOST  7
#define MCHP_XEC_DMA_I2C4_TARG  8
#define MCHP_XEC_DMA_I2C4_HOST  9
#define MCHP_XEC_DMA_QSPI0_TX   10
#define MCHP_XEC_DMA_QSPI0_RX   11
#define MCHP_XEC_DMA_GP_SPI0_TX 12
#define MCHP_XEC_DMA_GP_SPI0_RX 13
#define MCHP_XEC_DMA_GP_SPI1_TX 14
#define MCHP_XEC_DMA_GP_SPI1_RX 15
/* MEC175x/MEC165xB */
#define MCHP_XEC_I3C_HC0_TX     16
#define MCHP_XEC_I3C_HC0_RX     17
#define MCHP_XEC_I3C_SC0_TX     18
#define MCHP_XEC_I3C_SC0_RX     19

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_DMA_MCHP_XEC_CDMA_H_ */
