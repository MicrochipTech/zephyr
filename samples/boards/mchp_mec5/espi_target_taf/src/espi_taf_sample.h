/*
 * Copyright (c) 2024 Microchip Technology, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SAMPLES_BOARDS_MEC5_ESPI_TAF_SAMPLE_H_
#define __SAMPLES_BOARDS_MEC5_ESPI_TAF_SAMPLE_H_

#include <stdbool.h>
#include <stdint.h>

#include <device_mec5.h>
#include <zephyr/drivers/espi.h>

/* QSPI descriptors 12-13 are exit continuous mode */
#define MCHP_TAF_EXIT_CM_QUAD_D0(next)						\
	(QD_IO_QUAD | QD_TX_EN_ONES | QD_UNITS_1B | QD_NU(1) | QD_NEXT_DESCR(next))

#define MCHP_TAF_EXIT_CM_QUAD_D1(next)						\
	(QD_IO_QUAD | QD_RX_EN | QD_UNITS_1B | QD_NU(9) | QD_NEXT_DESCR(next) | \
	 QD_CLOSE_EN | QD_LAST_EN)

#define MCHP_TAF_EXIT_CM_DUAL_D0(didx)						\
		((MEC_QSPI_DESCR_IFM_DUAL << MEC_QSPI_DESCR_IFM_Pos) |		\
		 (MEC_QSPI_DESCR_TXEN_EN1 << MEC_QSPI_DESCR_TXEN_Pos) |		\
		 (MEC_QSPI_DESCR_QUNITS_1B << MEC_QSPI_DESCR_QUNITS_Pos) |	\
		 (13u << MEC_QSPI_DESCR_NEXT_Pos) |				\
		 (1u << MEC_QSPI_DESCR_QNUNITS_Pos))

#define MCHP_TAF_EXIT_CM_DUAL_DESCR13						\
		((MEC_QSPI_DESCR_IFM_DUAL << MEC_QSPI_DESCR_IFM_Pos) |		\
		 BIT(MEC_QSPI_DESCR_RXEN_Pos) |					\
		 (MEC_QSPI_DESCR_QUNITS_1B << MEC_QSPI_DESCR_QUNITS_Pos) |	\
		 (5u << MEC_QSPI_DESCR_QNUNITS_Pos) |				\
		 BIT(MEC_QSPI_DESCR_LAST_Pos) |					\
		 BIT(MEC_QSPI_DESCR_CLOSE_Pos))

/*
 * QSPI descriptors 14-15 are for polling 16-bit flash status
 * Transmit one byte opcode at 1X (no DMA).
 * Receive two bytes at 1X (no DMA).
 */
#define MCHP_TAF_POLL_DESCR14							\
		((MEC_QSPI_DESCR_IFM_FD << MEC_QSPI_DESCR_IFM_Pos) |		\
		 (MEC_QSPI_DESCR_TXEN_EN << MEC_QSPI_DESCR_TXEN_Pos) |		\
		 (MEC_QSPI_DESCR_QUNITS_1B << MEC_QSPI_DESCR_QUNITS_Pos) |	\
		 (15u << MEC_QSPI_DESCR_NEXT_Pos) |				\
		 (1u << MEC_QSPI_DESCR_QNUNITS_Pos))

#define MCHP_TAF_POLL_DESCR15							\
		((MEC_QSPI_DESCR_IFM_FD << MEC_QSPI_DESCR_IFM_Pos) |		\
		 BIT(MEC_QSPI_DESCR_RXEN_Pos) |					\
		 (MEC_QSPI_DESCR_QUNITS_1B << MEC_QSPI_DESCR_QUNITS_Pos) |	\
		 (2u << MEC_QSPI_DESCR_QNUNITS_Pos) |				\
		 BIT(MEC_QSPI_DESCR_LAST_Pos) |					\
		 BIT(MEC_QSPI_DESCR_CLOSE_Pos))

/*
 * Six QMSPI descriptors describe SPI flash opcode protocols.
 * Example: W25Q128 Continuous 24-bit address data read using quad I/O
 */
/* Continuous mode read: transmit-quad 24-bit address and mode byte */
#define MCHP_TAF_CM_RD_QUAD_D0							\
		((MEC_QSPI_DESCR_IFM_QUAD << MEC_QSPI_DESCR_IFM_Pos) |		\
		 (MEC_QSPI_DESCR_TXEN_EN << MEC_QSPI_DESCR_TXEN_Pos) |		\
		 (MEC_QSPI_DESCR_QUNITS_1B << MEC_QSPI_DESCR_QUNITS_Pos) |	\
		 (4u << MEC_QSPI_DESCR_QNUNITS_Pos))

/* Continuous mode read: transmit-quad 4 dummy clocks with I/O tri-stated */
#define MCHP_TAF_CM_RD_QUAD_D1							\
		((MEC_QSPI_DESCR_IFM_QUAD << MEC_QSPI_DESCR_IFM_Pos) |		\
		 (MEC_QSPI_DESCR_QUNITS_1B << MEC_QSPI_DESCR_QUNITS_Pos) |	\
		 (2u << MEC_QSPI_DESCR_QNUNITS_Pos))

/* Continuous mode read: read N bytes */
#define MCHP_TAF_CM_RD_QUAD_D2							\
		((MEC_QSPI_DESCR_IFM_QUAD << MEC_QSPI_DESCR_IFM_Pos) |		\
		 (MEC_QSPI_DESCR_QUNITS_1B << MEC_QSPI_DESCR_QUNITS_Pos) |	\
		 BIT(MEC_QSPI_DESCR_RXEN_Pos) |					\
		 BIT(MEC_QSPI_DESCR_LAST_Pos) |					\
		 BIT(MEC_QSPI_DESCR_CLOSE_Pos))

/* Continuous Mode: 24-bit address plus mode byte */
#define MCHP_TAF_CM_RD_DUAL_D0					\
		((MEC_QSPI_DESCR_IFM_DUAL << MEC_QSPI_DESCR_IFM_Pos) |		\
		 (MEC_QSPI_DESCR_TXEN_EN << MEC_QSPI_DESCR_TXEN_Pos) |		\
		 (MEC_QSPI_DESCR_QUNITS_1B << MEC_QSPI_DESCR_QUNITS_Pos) |	\
		 (4u << MEC_QSPI_DESCR_QNUNITS_Pos))

/* Continuous mode read: read N bytes */
#define MCHP_TAF_CM_RD_DUAL_D1					\
		((MEC_QSPI_DESCR_IFM_DUAL << MEC_QSPI_DESCR_IFM_Pos) |		\
		 (MEC_QSPI_DESCR_QUNITS_1B << MEC_QSPI_DESCR_QUNITS_Pos) |	\
		 BIT(MEC_QSPI_DESCR_RXEN_Pos) |					\
		 BIT(MEC_QSPI_DESCR_LAST_Pos) |					\
		 BIT(MEC_QSPI_DESCR_CLOSE_Pos))

/* Continuous mode Dual D2. Not used */
#define MCHP_TAF_CM_RD_DUAL_D2	0

/* Enter Continuous mode: transmit-single CM quad read opcode */
#define MCHP_W25Q128_ENTER_CM_D0					\
		(MCHP_QMSPI_C_IFM_1X | MCHP_QMSPI_C_TX_DATA |		\
		 MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS |	\
		 MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_NO_CLOSE |	\
		 MCHP_QMSPI_C_XFR_UNITS_1 | MCHP_QMSPI_C_XFR_NUNITS(1))

/* Enter Continuous mode: transmit-quad 24-bit address and mode byte  */
#define MCHP_W25Q128_ENTER_CM_D1					\
		(MCHP_QMSPI_C_IFM_4X | MCHP_QMSPI_C_TX_DATA |		\
		 MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS |	\
		 MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_NO_CLOSE |	\
		 MCHP_QMSPI_C_XFR_UNITS_1 | MCHP_QMSPI_C_XFR_NUNITS(4))

/* Enter Continuous mode: read-quad 3 bytes */
#define MCHP_W25Q128_ENTER_CM_D2					\
		(MCHP_QMSPI_C_IFM_4X | MCHP_QMSPI_C_TX_DIS |		\
		 MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS |	\
		 MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_CLOSE |		\
		 MCHP_QMSPI_C_XFR_UNITS_1 |				\
		 MCHP_QMSPI_C_XFR_NUNITS(3) | MCHP_QMSPI_C_DESCR_LAST)

/* Enter Continuous mode: transmit-single CM dual read opcode */
#define MCHP_W25Q128_ENTER_CM_DUAL_D0					\
		(MCHP_QMSPI_C_IFM_1X | MCHP_QMSPI_C_TX_DATA |		\
		 MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS |	\
		 MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_NO_CLOSE |	\
		 MCHP_QMSPI_C_XFR_UNITS_1 | MCHP_QMSPI_C_XFR_NUNITS(1))

/* Enter Continuous mode: transmit-dual 24-bit address and mode byte  */
#define MCHP_W25Q128_ENTER_CM_DUAL_D1					\
		(MCHP_QMSPI_C_IFM_2X | MCHP_QMSPI_C_TX_DATA |		\
		 MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS |	\
		 MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_NO_CLOSE |	\
		 MCHP_QMSPI_C_XFR_UNITS_1 | MCHP_QMSPI_C_XFR_NUNITS(4))

/* Enter Continuous mode: read-dual 3 bytes */
#define MCHP_W25Q128_ENTER_CM_DUAL_D2					\
		(MCHP_QMSPI_C_IFM_2X | MCHP_QMSPI_C_TX_DIS |		\
		 MCHP_QMSPI_C_TX_DMA_DIS | MCHP_QMSPI_C_RX_DIS |	\
		 MCHP_QMSPI_C_RX_DMA_DIS | MCHP_QMSPI_C_CLOSE |		\
		 MCHP_QMSPI_C_XFR_UNITS_1 |				\
		 MCHP_QMSPI_C_XFR_NUNITS(3) | MCHP_QMSPI_C_DESCR_LAST)



#define FL_WB128_CS0_CM_RD0 0x81406u
#define FL_WB128_CS0_CM_RD1 0x42402u
#define FL_WB128_CS0_CM_RD2 0x1C7C2u

#define FL_WB128_CS0_CM_ENTER0 0x24404u
#define FL_WB128_CS0_CM_ENTER1 0x85406u
#define FL_WB128_CS0_CM_ENTER2 0x7C602u

#define FL_WB128_CS1_CM_RD0 0x87406u
#define FL_WB128_CS1_CM_RD1 0x48402u
#define FL_WB128_CS1_CM_RD2 0x1C7C2u

#define FL_WB128_CS1_CM_ENTER0 0x2A404u
#define FL_WB128_CS1_CM_ENTER1 0x8B406u
#define FL_WB128_CS1_CM_ENTER2 0x7C602u

#define FL_WB128_CM_EXIT0 0xAD406u
#define FL_WB128_CM_EXIT1 0xBC602u

#define FL_WB128_POLL_STS0 0x2F404u
#define FL_WB128_POLL_STS1 0x50640u

#define FL_WB128_OPA 0x057A7506u
#define FL_WB128_OPB 0x02D85220u
#define FL_WB128_OPC 0x35A5FFEBu
#define FL_WB128_OPD 0x0000ABB9u

#define FL_WB128_CS0_CFG_DESCR_IDS 0x5300u
#define FL_WB128_CS1_CFG_DESCR_IDS 0xB900u

#define FL_WB128_POLL2_MASK 0xFF7Fu

const struct device *app_get_taf_device(void);

const struct espi_taf_protection *get_taf_rpmc_flash_pr(void);

void espi_taf_cb_data_init(void);
bool espi_taf_cb_has_fired(void);
void espi_taf_cb_get_event(struct espi_event *ev);

int sample_espi_taf_config(uint32_t cfg_flags);

int sample_espi_taf_read_test1(uint32_t flash_addr, uint8_t *expected_data,
			       uint32_t expected_data_len);

int sample_espi_taf_erase_test1(uint32_t flash_addr, uint32_t erase_size);

int sample_espi_taf_write_test1(uint32_t flash_addr, uint8_t *data, uint32_t data_len);

#endif /* __SAMPLES_BOARDS_MEC5_ESPI_TAF_SAMPLE_H_ */
