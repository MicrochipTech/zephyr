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

static void pr_xec_pinctrl(const pinctrl_soc_pin_t *p)
{
	uint32_t d = *(const uint32_t *)p;

	TC_PRINT("d = 0x%08x\n", d);
	TC_PRINT(".pinmux    = 0x%0x\n", p->pinmux);
	TC_PRINT(".no_pud    = %u\n", p->no_pud);
	TC_PRINT(".pd        = %u\n", p->pd);
	TC_PRINT(".pu        = %u\n", p->pu);
	TC_PRINT(".obuf_pp   = %u\n", p->obuf_pp);
	TC_PRINT(".obuf_od   = %u\n", p->obuf_od);
	TC_PRINT(".out_dis   = %u\n", p->out_dis);
	TC_PRINT(".out_en    = %u\n", p->out_en);
	TC_PRINT(".out_hi    = %u\n", p->out_hi);
	TC_PRINT(".out_lo    = %u\n", p->out_lo);
	TC_PRINT(".slew_rate = %u\n", p->slew_rate);
	TC_PRINT(".drive_str = %u\n", p->drive_str);
	TC_PRINT(".finv      = %u\n", p->finv);
	TC_PRINT(".lp        = %u\n", p->lp);
}

ZTEST(pinctrl_mchp_mec, test_slew_rate)
{
	const struct pinctrl_state *scfg = &pcfg->states[0];
	const pinctrl_soc_pin_t *p = &scfg->pins[1];

	TC_PRINT("pins[1]\n");
	pr_xec_pinctrl(p);

	zassert_equal(p->slew_rate, MCHP_XEC_SLEW_RATE_SLOW);
}

ZTEST(pinctrl_mchp_mec, test_drive_strength)
{
	const struct pinctrl_state *scfg = &pcfg->states[0];
	const pinctrl_soc_pin_t *p = &scfg->pins[2];

	TC_PRINT("pins[2]\n");
	pr_xec_pinctrl(p);

	zassert_equal(p->drive_str, MCHP_XEC_DRV_STR_4X);
}

ZTEST(pinctrl_mchp_mec, test_pullup)
{
	const struct pinctrl_state *scfg = &pcfg->states[0];
	const pinctrl_soc_pin_t *p = &scfg->pins[3];

	TC_PRINT("pins[3]\n");
	pr_xec_pinctrl(p);

	zassert_equal(p->pu, 1);
}

ZTEST(pinctrl_mchp_mec, test_pulldown)
{
	const struct pinctrl_state *scfg = &pcfg->states[0];
	const pinctrl_soc_pin_t *p = &scfg->pins[4];

	zassert_equal(p->pd, 1);
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