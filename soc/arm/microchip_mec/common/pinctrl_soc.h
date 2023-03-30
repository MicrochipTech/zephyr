/*
 * Copyright (c) 2020 Linaro Ltd.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * Copyright (c) 2021 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * Microchip XEC SoC specific helpers for pinctrl driver
 */

#ifndef ZEPHYR_SOC_ARM_MICROCHIP_XEC_COMMON_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_MICROCHIP_XEC_COMMON_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#include <zephyr/dt-bindings/pinctrl/mchp-xec-pinctrl.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

/* Type for MCHP XEC pin. */
typedef struct pinctrl_soc_pin {
	uint32_t pinmux;
} pinctrl_soc_pin_t;

/* initialize pinmux member fields of pinctrl_pin_t */
#define Z_PINCTRL_MCHP_XEC_PINMUX_INIT(node_id) DT_PROP(node_id, pinmux)

#define MCHP_XEC_BIAS_DIS_VAL(nid) (1u * DT_PROP_OR(nid, bias_disable, 0))
#define MCHP_XEC_BIAS_PD_VAL(nid) (1u * DT_PROP_OR(nid, bias_pull_down, 0))
#define MCHP_XEC_BIAS_PU_VAL(nid) (1u * DT_PROP_OR(nid, bias_pull_up, 0))
#define MCHP_XEC_DRV_PP_VAL(nid) (1u * DT_PROP_OR(nid, drive_push_pull, 0))
#define MCHP_XEC_DRV_OD_VAL(nid) (1u * DT_PROP_OR(nid, drive_open_drain, 0))
#define MCHP_XEC_OUT_DIS_VAL(nid) (1u * DT_PROP_OR(nid, output_disable, 0))
#define MCHP_XEC_OUT_EN_VAL(nid) (1u * DT_PROP_OR(nid, output_enable, 0))
#define MCHP_XEC_OUT_DRV_HI(nid) (1u * DT_PROP_OR(nid, output_high, 0))
#define MCHP_XEC_OUT_DRV_LO(nid) (1u * DT_PROP_OR(nid, output_low, 0))
#define MCHP_XEC_PAD_INPUT_DIS_VAL(nid)	(1u * DT_PROP_OR(nid, input_disable, 0))
#define MCHP_XEC_SLEW_VAL(nid) (DT_ENUM_IDX_OR(nid, slew_rate, 0))
#define MCHP_XEC_DRVSTR_VAL(nid) (DT_ENUM_IDX_OR(nid, drive_strength, 0))
#define MCHP_XEC_LOW_POWER_EN(nid) (1u * DT_PROP_OR(nid, low_power_enable, 0))
#define MCHP_XEC_FUNC_INVERT_VAL(nid) (1u * DT_PROP_OR(nid, microchip_output_func_invert, 0))

#define Z_PINCTRL_STATE_PIN_INIT(node_id, state_prop, idx)					\
	{ .pinmux = (Z_PINCTRL_MCHP_XEC_PINMUX_INIT(DT_PROP_BY_IDX(node_id, state_prop, idx))	\
		     | (MCHP_XEC_BIAS_DIS_VAL(node_id) << MCHP_XEC_NO_PUD_POS)			\
		     | (MCHP_XEC_BIAS_PD_VAL(node_id) << MCHP_XEC_PD_POS)			\
		     | (MCHP_XEC_BIAS_PU_VAL(node_id) << MCHP_XEC_PU_POS)			\
		     | (MCHP_XEC_DRV_PP_VAL(node_id) << MCHP_XEC_PUSH_PULL_POS)			\
		     | (MCHP_XEC_DRV_OD_VAL(node_id) << MCHP_XEC_OPEN_DRAIN_POS)		\
		     | (MCHP_XEC_OUT_DIS_VAL(node_id) << MCHP_XEC_OUT_DIS_POS)			\
		     | (MCHP_XEC_OUT_EN_VAL(node_id) << MCHP_XEC_OUT_EN_POS)			\
		     | (MCHP_XEC_OUT_DRV_HI(node_id) << MCHP_XEC_OUT_HI_POS)			\
		     | (MCHP_XEC_OUT_DRV_LO(node_id) << MCHP_XEC_OUT_LO_POS)			\
		     | (MCHP_XEC_PAD_INPUT_DIS_VAL(node_id) << MCHP_XEC_INPUT_PAD_DIS_POS)	\
		     | (MCHP_XEC_SLEW_VAL(node_id) << MCHP_XEC_SLEW_RATE_POS)			\
		     | (MCHP_XEC_DRVSTR_VAL(node_id) << MCHP_XEC_DRV_STR_POS)			\
		     | (MCHP_XEC_LOW_POWER_EN(node_id) << MCHP_XEC_PIN_LOW_POWER_POS)		\
		     | (MCHP_XEC_FUNC_INVERT_VAL(node_id) << MCHP_XEC_FUNC_INV_POS) ) },

/* Use DT FOREACH macro to initialize each used pin */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)					\
	{DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_MICROCHIP_XEC_COMMON_PINCTRL_SOC_H_ */
