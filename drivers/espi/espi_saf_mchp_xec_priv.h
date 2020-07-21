/*
 * Copyright (c) 2020 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Header with private definitions for MCHP eSPI SAF
 */

#ifndef ZEPHYR_DRIVERS_ESPI_SAF_MCHP_XEC_PRIV_H_
#define ZEPHYR_DRIVERS_ESPI_SAF_MCHP_XEC_PRIV_H_

#include <stdint.h>
#include <sys/util.h>

/* Default SAF Map of eSPI TAG numbers to master numbers */
#define SAF_TAG_MAP0 0x23221100
#define SAF_TAG_MAP1 0x77677767
#define SAF_TAG_MAP2 0x00000005

/* QMSPI master clock is 48MHz AHB clock. Specify divider */
#define SAF_QMSPI_CLK_DIV 2U

#define SAF_QMSPI_CS_TIMING 0x03000101U

#endif /* ZEPHYR_DRIVERS_ESPI_SAF_MCHP_XEC_PRIV_H_ */
