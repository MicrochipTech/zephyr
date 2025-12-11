/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/ztest.h>

/* Pin configuration for test device */
#define TEST_DEVICE DT_NODELABEL(test_device)
PINCTRL_DT_DEV_CONFIG_DECLARE(TEST_DEVICE);
static const struct pinctrl_dev_config *pcfg = PINCTRL_DT_DEV_CONFIG_GET(TEST_DEVICE);

#ifdef CONFIG_TEST_PINCTRL_MCHP_MEC

ZTEST(pinctrl_mchp_mec, test_slew_rate)
{
	const struct pinctrl_state *scfg;
	uint32_t val = 0;

	scfg = &pcfg->states[0];

	val = (scfg->pins[1] >> MCHP_XEC_SLEW_RATE_POS) & MCHP_XEC_SLEW_RATE_MSK0;
	zassert_equal(val, MCHP_XEC_SLEW_RATE_SLOW0);
}

ZTEST(pinctrl_mchp_mec, test_drive_strength)
{
	const struct pinctrl_state *scfg;
	uint32_t val = 0;

	scfg = &pcfg->states[0];

	val = (scfg->pins[2] >> MCHP_XEC_DRV_STR_POS) & MCHP_XEC_DRV_STR_MSK0;
	zassert_equal(val, MCHP_XEC_DRV_STR0_4X);
}

ZTEST(pinctrl_mchp_mec, test_pullup)
{
	const struct pinctrl_state *scfg;

	scfg = &pcfg->states[0];

	zassert_equal(((scfg->pins[3] >> MCHP_XEC_PU_POS) & 1u), 1);
}

ZTEST(pinctrl_mchp_mec, test_pulldown)
{
	const struct pinctrl_state *scfg;

	scfg = &pcfg->states[0];

	zassert_equal(((scfg->pins[4] >> MCHP_XEC_PD_POS) & 1u), 1);
}

ZTEST(pinctrl_mchp_mec, test_apply_state)
{
	int ret;

	ret = pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT);
	zassert_equal(ret, 0);
}

ZTEST_SUITE(pinctrl_mchp_mec, NULL, NULL, NULL, NULL, NULL);

#else
#define MCHP_PINCTRL_FLAG_GET(pincfg, pos) (((pincfg.pinflag) >> pos) & MCHP_PINCTRL_FLAG_MASK)

ZTEST(pinctrl_mchp, test_pullup_pulldown_none)
{
	const struct pinctrl_state *scfg;

	scfg = &pcfg->states[0];

	zassert_equal(MCHP_PINCTRL_FLAG_GET(scfg->pins[0], MCHP_PINCTRL_PULLUP_POS), 0);
	zassert_equal(MCHP_PINCTRL_FLAG_GET(scfg->pins[0], MCHP_PINCTRL_PULLDOWN_POS), 0);
	zassert_equal(MCHP_PINCTRL_FLAG_GET(scfg->pins[1], MCHP_PINCTRL_PULLUP_POS), 0);
	zassert_equal(MCHP_PINCTRL_FLAG_GET(scfg->pins[1], MCHP_PINCTRL_PULLDOWN_POS), 0);
}

ZTEST(pinctrl_mchp, test_pullup)
{
	const struct pinctrl_state *scfg;

	scfg = &pcfg->states[0];

	zassert_equal(MCHP_PINCTRL_FLAG_GET(scfg->pins[2], MCHP_PINCTRL_PULLUP_POS), 1);
}

ZTEST(pinctrl_mchp, test_pulldown)
{
	const struct pinctrl_state *scfg;

	scfg = &pcfg->states[0];

	zassert_equal(MCHP_PINCTRL_FLAG_GET(scfg->pins[3], MCHP_PINCTRL_PULLDOWN_POS), 1);
}

ZTEST(pinctrl_mchp, test_input_output_enable)
{
	const struct pinctrl_state *scfg;

	scfg = &pcfg->states[0];

	zassert_equal(MCHP_PINCTRL_FLAG_GET(scfg->pins[4], MCHP_PINCTRL_INPUTENABLE_POS), 1);
	zassert_equal(MCHP_PINCTRL_FLAG_GET(scfg->pins[4], MCHP_PINCTRL_OUTPUTENABLE_POS), 1);
}

#if defined(CONFIG_TEST_PINCTRL_MCHP_SAM)
ZTEST(pinctrl_mchp, test_drive_strength)
{
	const struct pinctrl_state *scfg;

	scfg = &pcfg->states[0];

	zassert_equal(MCHP_PINCTRL_FLAG_GET(scfg->pins[5], MCHP_PINCTRL_DRIVESTRENGTH_POS), 1);
}
#endif

ZTEST(pinctrl_mchp, test_apply_state)
{
	int ret;

	ret = pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT);
	zassert_equal(ret, 0);
}

ZTEST_SUITE(pinctrl_mchp, NULL, NULL, NULL, NULL, NULL);
#endif /* CONFIG_TEST_PINCTRL_MCHP_MEC */