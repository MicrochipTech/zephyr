/* Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ESPI_MCHP_MEC_H__
#define ZEPHYR_INCLUDE_DRIVERS_ESPI_MCHP_MEC_H__

#include <zephyr/drivers/espi.h>

#define MCHP_MEC_NUM_SRAM_REGIONS    2
#define MCHP_MEC_NUM_EMIS            3
#define MCHP_MEC_NUM_EMI_MEM_REGIONS 2

enum espi_pc_event_mchp_mec {
	ESPI_PC_EVT_BUS_INTERNAL_BERR = ESPI_PC_EVT_BUS_PRIVATE_START,
};

enum espi_virtual_peripheral_mchp_mec {
	ESPI_PC_MCHP_MEC_MBOX0 = ESPI_PERIPHERAL_PRIV_START,
	ESPI_PC_MCHP_MEC_ACPI_EC0,
	ESPI_PC_MCHP_MEC_ACPI_EC1,
	ESPI_PC_MCHP_MEC_ACPI_EC2,
	ESPI_PC_MCHP_MEC_ACPI_EC3,
	ESPI_PC_MCHP_MEC_ACPI_EC4,
	ESPI_PC_MCHP_MEC_EMI0,
	ESPI_PC_MCHP_MEC_EMI1,
	ESPI_PC_MCHP_MEC_EMI2,
	ESPI_PC_MCHP_MEC_BDP0,
	ESPI_PC_MCHP_MEC_RTC0,
	ESPI_PC_MCHP_MEC_GL,
};

struct mchp_mec_emi_mem_cfg {
	uint32_t mem_base;
	uint16_t mem_rd_sz;
	uint16_t mem_wr_sz;
	uint8_t emi_id;
	uint8_t mem_id;
};

struct mchp_mec_sram_cfg {
	uint32_t mem_base;
	uint16_t mem_size;
	uint8_t mem_access;
	uint8_t sram_id;
};

struct espi_cfg_mchp_mec {
	struct mchp_mec_sram_cfg *sram_cfgs;
	struct mchp_mec_emi_mem_cfg *emi_mem_cfgs;
	uint8_t num_sram_cfgs;
	uint8_t num_emi_mem_cfgs;
};

enum espi_mec_io_capture_event_pos {
	ESPI_IO_CAP_EVENT_INCOMPLETE_POS = 0,
	ESPI_IO_CAP_EVENT_OVERRUN_POS,
	ESPI_IO_CAP_EVENT_ERROR_POS
};

struct espi_mec_host_io_data {
	uint32_t data;
	uint8_t start_byte_lane;
	uint8_t size;
	uint8_t msk;
	uint8_t flags;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_ESPI_MCHP_MEC_H__ */
