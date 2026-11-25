/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ESPI_HOST_EMU_VPGGL_HOST_H_
#define ESPI_HOST_EMU_VPGGL_HOST_H_

#include "espi_hc_emu.h"

#ifdef CONFIG_SAMPLE_VPGGL_SHELL
/* Configure the VPGGL drive GPIOs as outputs at their idle levels. Call
 * before the eSPI link is brought up.
 */
int vpggl_host_pins_init(void);

/* The link is up and PLTRST# de-asserted: enable the hvpggl eSPI commands. */
void vpggl_host_link_ready(struct espi_hc_context *ctx);
#else
static inline int vpggl_host_pins_init(void)
{
	return 0;
}

static inline void vpggl_host_link_ready(struct espi_hc_context *ctx)
{
	(void)ctx;
}
#endif

#endif /* ESPI_HOST_EMU_VPGGL_HOST_H_ */
