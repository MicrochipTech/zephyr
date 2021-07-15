/*
 * Copyright (c) 2021 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_MCHP_XEC_ECIA_PRIV_H_
#define ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_MCHP_XEC_ECIA_PRIV_H_

#include <zephyr/types.h>
#include <soc.h>

/*
 * The MEC XEC family ECIA is composed of a set of GIRQ's numbered 8 to 26 for
 * historical reasons. Each GIRQ is 20 bytes in size composed of 5 32-bit
 * registers. GIRQn located at ECIA base + n * 0x14
 * Offset 0x0: Source R/W1C each bit is an interrupt line from a peripheral
 * Offset 0x4: Enable Set: Write 1 to a bit to enable, writes of 0 no effect.
 * Offset 0x8: Result: Read-only = Source AND Enable-Set.
 * Offset 0xC: Enable Clear: Write 1 to a bit to clear the enable.
 * Offset 0x10: reserved, read 0, writes ignored.
 * Aggregated output is bit-wise OR of all result bits.
 * Aggregated output of GIRQ's 8 - 21, 23 - 26 are connected to NVIC external
 * inputs 0 - 17.
 * Direct capable GIRQ's 13 - 21, and 23 connect their individual Result
 * register bits to NVIC external input 20 and above. Refer to MECxxxx
 * documentation.
 * ECIA has three other registers at offsets 0x200, 0x204, and 0x208 each
 * a bit map where GIRQ08 = bit[8], ..., GIRQ26 = bit[26].
 * 0x200: GIRQ aggregated output enable set. Write 1 to enable a GIRQ's
 * aggregated output connection to the NVIC.
 * 0x204: GIRQ aggregated output disable set. Write 1 to disable a GIRQ's
 * aggregated output connection to the NVIC.
 * 0x208: Read only, each bit indicates a GIRQ has at least one active source
 * (result) bit active.
 *
 * For direct capable GIRQ's software must not enable both aggregated and
 * direct connections. Direct capable GIRQ rules:
 * 1. If a direct capable GIRQ will be used in direct mode clears it aggregated
 *    output enable bit in ECIA register at 0x200. Alternatively, one can
 *    set the aggregated enable and clear the aggregated GIRQn NVIC enable.
 * 2. If a direct capable GIRQ will be used in aggregated mode its aggregated
 *    output enable must be set and make sure all direct NVIC for this GIRQ
 *    inputs are disabled in the NVIC.
 *
 * Each GIRQ may have up to 31 sources (bit[31] in the GIRQ is not used due to
 * historical reasons). Most GIRQ's do not implement all sources. The number
 * and identity of the interrupt sources is SoC specific.
 */

#define MCHP_FIRST_GIRQ		8
#define MCHP_LAST_GIRQ		26
#define MCHP_NUM_GIRQS		(MCHP_LAST_GIRQ - MCHP_FIRST_GIRQ + 1)

/*
 * GIRQ22 used to wake PLL from deep sleep for data transfers from external
 * devices which do not need immediate service from the SoC CPU. GIRQ22 is
 * not connected to the NVIC. If SoC CPU service is required the data transfer
 * completion will trigger another interrupt source in a GIRQ connected to
 * the NVIC.
 */
#define MCHP_ECIA_GIRQ_NO_NVIC	22u

/*
 * Bit map of GIRQ's only usable in aggregated mode by HW design.
 * GPIO's, eSPI Host-to-Device virtual wires, etc.
 */
#define MCHP_ECIA_AGGR_BITMAP	(BIT(8) | BIT(9) | BIT(10) | BIT(11) | \
				 BIT(12) | BIT(22) | BIT(24) | BIT(25) | \
				 BIT(26))

/*
 * Bit map of GIRQ's usable as aggregated or direct configurable by software.
 * All sources is a GIRQ are aggregated or all direct. In direct mode all
 * result bits in a GIRQ have individual NVIC connections.
 */
#define MCHP_ECIA_DIRECT_BITMAP	(BIT(13) | BIT(14) | BIT(15) | BIT(16) | \
				 BIT(17) | BIT(18) | BIT(19) | BIT(20) | \
				 BIT(21) | BIT(23))

#define MCHP_ECIA_ALL_BITMAP	GENMASK(26, 8)

#define GIRQ08_BITMAP	GENMASK(30, 0)
#define GIRQ09_BITMAP	GENMASK(30, 0)
#define GIRQ10_BITMAP	GENMASK(30, 0)
#define GIRQ11_BITMAP	GENMASK(30, 0)
#define GIRQ12_BITMAP	GENMASK(30, 0)
#define GIRQ24_BITMAP	GENMASK(27, 0)
#define GIRQ25_BITMAP	GENMASK(15, 0)

#if defined(CONFIG_SOC_SERIES_MEC1501X)
#define GIRQ13_BITMAP	GENMASK(7, 0)
#define GIRQ14_BITMAP	GENMASK(11, 0)
#define GIRQ15_BITMAP	(GENMASK(12, 0) | GENMASK(20, 15) | GENMASK(23, 22))
#define GIRQ16_BITMAP	GENMASK(4, 0)
#define GIRQ17_BITMAP	\
	(GENMASK(5, 0) | GENMASK(9, 8) | GENMASK(15, 13) | BIT(17))
#define GIRQ18_BITMAP	\
	(GENMASK(1, 0) | GENMASK(11, 10) | BIT(13) | GENMASK(28, 20))
#define GIRQ19_BITMAP	GENMASK(10, 0)
#define GIRQ20_BITMAP	BIT(3)
#define GIRQ21_BITMAP	(GENMASK(14, 2) | GENMASK(19, 18) | BIT(21) | BIT(25))
#define GIRQ22_BITMAP	GENMASK(9, 0)
#define GIRQ23_BITMAP	\
	(GENMASK(1, 0) | GENMASK(5, 4) | GENMASK(14, 10) | GENMASK(17, 16))
#define GIRQ26_BITMAP	(GENMASK(6, 0) | GENMASK(13, 11))
#elif defined(CONFIG_SOC_SERIES_MEC172X)
#define GIRQ13_BITMAP	GENMASK(4, 0)
#define GIRQ14_BITMAP	GENMASK(15, 0)
#define GIRQ15_BITMAP	(GENMASK(20, 0) | BIT(22))
#define GIRQ16_BITMAP	(BIT(0) | BIT(2) | BIT(3))
#define GIRQ17_BITMAP	(GENMASK(4, 0) | GENMASK(17, 8) | GENMASK(23, 20))
#define GIRQ18_BITMAP	(GENMASK(7, 0) | BIT(10) | BIT(13) | GENMASK(28, 20))
#define GIRQ19_BITMAP	GENMASK(11, 0)
#define GIRQ20_BITMAP	(GENMASK(3, 0) | GENMASK(9, 8))
#define GIRQ21_BITMAP	(GENMASK(15, 2) | GENMASK(19, 18) | GENMASK(26, 25))
#define GIRQ22_BITMAP	(GENMASK(5, 0) | BIT(9) | BIT(15))
#define GIRQ23_BITMAP	(GENMASK(14, 0) | GENMASK(17, 16))
#define GIRQ26_BITMAP	(GENMASK(6, 0) | GENMASK(13, 12))
#endif

#define BCP1(n)		\
	(((0xaaaaaaaau & (uint32_t)n) >> 1) + (0x55555555u & (uint32_t)(n)))
#define BCP2(n)		\
	(((0xccccccccu & (uint32_t)n) >> 2) + (0x33333333u & (uint32_t)(n)))
#define BCP3(n)		\
	(((0xf0f0f0f0u & (uint32_t)n) >> 4) + (0x0f0f0f0fu & (uint32_t)(n)))
#define BCP4(n)		\
	(((0xff00ff00u & (uint32_t)n) >> 8) + (0x00ff00ffu & (uint32_t)(n)))
#define BCP5(n)		\
	(((0xffff0000u & (uint32_t)n) >> 16) + (0x0000ffffu & (uint32_t)(n)))

#define BC32(n) BCP5(BCP4(BCP3(BCP2(BCP1(n)))))

#define GIRQ_NBITS_8	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq8), source_bitmap)))
#define GIRQ_NBITS_9	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq9), source_bitmap)))
#define GIRQ_NBITS_10	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq10), source_bitmap)))
#define GIRQ_NBITS_11	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq11), source_bitmap)))
#define GIRQ_NBITS_12	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq12), source_bitmap)))
#define GIRQ_NBITS_13	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq13), source_bitmap)))
#define GIRQ_NBITS_14	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq14), source_bitmap)))
#define GIRQ_NBITS_15	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq15), source_bitmap)))
#define GIRQ_NBITS_16	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq16), source_bitmap)))
#define GIRQ_NBITS_17	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq17), source_bitmap)))
#define GIRQ_NBITS_18	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq18), source_bitmap)))
#define GIRQ_NBITS_19	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq19), source_bitmap)))
#define GIRQ_NBITS_20	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq20), source_bitmap)))
#define GIRQ_NBITS_21	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq21), source_bitmap)))
#define GIRQ_NBITS_22	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq22), source_bitmap)))
#define GIRQ_NBITS_23	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq23), source_bitmap)))
#define GIRQ_NBITS_24	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq24), source_bitmap)))
#define GIRQ_NBITS_25	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq25), source_bitmap)))
#define GIRQ_NBITS_26	\
	BC32((uint32_t)(DT_PROP(DT_NODELABEL(girq26), source_bitmap)))

/*
 * Placeholder for DT flag to enable aggregated GIRQ.
 * If an aggregated GIRQ will not be used we don't want to waste data memory.
 */
#define XEC_ECIA_GIRQ08_AGGR_EN
#define XEC_ECIA_GIRQ09_AGGR_EN
#define XEC_ECIA_GIRQ10_AGGR_EN
#define XEC_ECIA_GIRQ11_AGGR_EN
#define XEC_ECIA_GIRQ12_AGGR_EN
/* #define XEC_ECIA_GIRQ13_AGGR_EN */
/* #define XEC_ECIA_GIRQ14_AGGR_EN */
/* #define XEC_ECIA_GIRQ15_AGGR_EN */
/* #define XEC_ECIA_GIRQ16_AGGR_EN */
/* #define XEC_ECIA_GIRQ17_AGGR_EN */
/* #define XEC_ECIA_GIRQ18_AGGR_EN */
#define XEC_ECIA_GIRQ19_AGGR_EN
/* #define XEC_ECIA_GIRQ20_AGGR_EN */
/* #define XEC_ECIA_GIRQ21_AGGR_EN */
#define XEC_ECIA_GIRQ22_AGGR_EN
/* #define XEC_ECIA_GIRQ23_AGGR_EN */
#define XEC_ECIA_GIRQ24_AGGR_EN
#define XEC_ECIA_GIRQ25_AGGR_EN
#define XEC_ECIA_GIRQ26_AGGR_EN

#endif /* ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_MCHP_XEC_ECIA_PRIV_H_ */
