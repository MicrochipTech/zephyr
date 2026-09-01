/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Helpers shared by the Microchip XEC eSPI V3 peripheral (logical) channel
 * device drivers. Each logical device is an independent Zephyr driver bound to
 * its own device tree child node of the eSPI controller. The controller owns
 * the Host I/O BAR, Memory BAR and Serial-IRQ registers, so nothing in here
 * touches them; a peripheral driver only ever programs the registers of its
 * own logical device block and its own interrupt sources.
 */

#ifndef ZEPHYR_DRIVERS_ESPI_XEC_PC_ESPI_PC_XEC_H_
#define ZEPHYR_DRIVERS_ESPI_XEC_PC_ESPI_PC_XEC_H_

#include <stdint.h>
#include <soc.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>

/* Bit mask of the events a peripheral channel driver must observe to keep its
 * logical device block armed. The controller re-programs the BARs and Serial
 * IRQ registers before delivering any of these, so the driver can (re)apply
 * its own block configuration from the handler.
 */
#define XEC_PC_EVT_MASK_HW_USABLE                                                                  \
	(BIT(MCHP_XEC_ESPI_PC_EVT_ESPI_RESET_DEASSERT) |                                           \
	 BIT(MCHP_XEC_ESPI_PC_EVT_PLTRST_DEASSERT) | BIT(MCHP_XEC_ESPI_PC_EVT_CHAN_ENABLED))

/* Bit mask of the events telling a peripheral channel driver its Host facing
 * hardware is about to be held in reset.
 */
#define XEC_PC_EVT_MASK_HW_GONE                                                                    \
	(BIT(MCHP_XEC_ESPI_PC_EVT_ESPI_RESET_ASSERT) |                                             \
	 BIT(MCHP_XEC_ESPI_PC_EVT_PLTRST_ASSERT) | BIT(MCHP_XEC_ESPI_PC_EVT_CHAN_DISABLED))

#define XEC_PC_EVT_MASK_ALL (XEC_PC_EVT_MASK_HW_USABLE | XEC_PC_EVT_MASK_HW_GONE)

/* True for the events delivered once the Host facing decoders are valid. */
static inline bool xec_pc_evt_is_hw_usable(enum mchp_xec_espi_pc_event evt)
{
	return (XEC_PC_EVT_MASK_HW_USABLE & BIT((uint32_t)evt)) != 0U;
}

/* Enable or disable one interrupt source described by a device tree girqs
 * entry, which encodes the aggregator instance and the bit position in it.
 */
static inline void xec_pc_girq_ctrl(uint32_t ecia_info, uint8_t enable)
{
	soc_ecia_girq_ctrl(MCHP_XEC_ECIA_GIRQ(ecia_info), MCHP_XEC_ECIA_GIRQ_POS(ecia_info),
			   enable);
}

/* Clear the latched status of one interrupt source. */
static inline void xec_pc_girq_clr(uint32_t ecia_info)
{
	soc_ecia_girq_status_clear(MCHP_XEC_ECIA_GIRQ(ecia_info),
				   MCHP_XEC_ECIA_GIRQ_POS(ecia_info));
}

#endif /* ZEPHYR_DRIVERS_ESPI_XEC_PC_ESPI_PC_XEC_H_ */
