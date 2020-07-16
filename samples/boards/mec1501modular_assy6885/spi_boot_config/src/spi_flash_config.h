/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SPI_FLASH_CONFIG_H__
#define __SPI_FLASH_CONFIG_H__

/* Need to move these values to MCHP HAL */
#define MCHP_QMSPI_BUF_PTR_POS  12u

int qspi_init(void);
int qspi_reset_spi_flash_device(uint8_t slave_index);

void qspi_clear_status(void);
void qspi_exit_continuous_mode(uint8_t slave_index);

#endif /* __SPI_FLASH_CONFIG_H__ */
