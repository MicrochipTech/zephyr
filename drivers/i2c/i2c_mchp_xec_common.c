/*
 * Copyright (c) 2026 Microchip Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include "i2c_mchp_xec_common.h"
#include "i2c_mchp_xec_regs.h"

/* I2C recover SCL low retries */
#define I2C_RECOVER_SCL_LOW_RETRIES 10
/* I2C recover SDA low retries */
#define I2C_RECOVER_SDA_LOW_RETRIES 3
/* I2C recovery bit bang delay */
#define I2C_RECOVER_BB_DELAY_US     5
/* I2C recovery SCL sample delay */
#define I2C_RECOVER_SCL_DELAY_US    50

struct xec_i2c_timing {
	uint32_t freq_hz;
	uint32_t data_tm;    /* data timing  */
	uint32_t idle_sc;    /* idle scaling */
	uint32_t timeout_sc; /* timeout scaling */
	uint32_t bus_clock;  /* bus clock hi/lo pulse widths */
	uint8_t rpt_sta_htm; /* repeated start hold time */
};

static const struct xec_i2c_timing xec_i2c_nl_timing_tbl[] = {
	{KHZ(100), XEC_I2C_SMB_DATA_TM_100K, XEC_I2C_SMB_IDLE_SC_100K, XEC_I2C_SMB_TMO_SC_100K,
	 XEC_I2C_SMB_BUS_CLK_100K, XEC_I2C_SMB_RSHT_100K},
	{KHZ(400), XEC_I2C_SMB_DATA_TM_400K, XEC_I2C_SMB_IDLE_SC_400K, XEC_I2C_SMB_TMO_SC_400K,
	 XEC_I2C_SMB_BUS_CLK_400K, XEC_I2C_SMB_RSHT_400K},
	{MHZ(1), XEC_I2C_SMB_DATA_TM_1M, XEC_I2C_SMB_IDLE_SC_1M, XEC_I2C_SMB_TMO_SC_1M,
	 XEC_I2C_SMB_BUS_CLK_1M, XEC_I2C_SMB_RSHT_1M},
};

void i2c_xec_ctrl_reset(mm_reg_t regbase, uint8_t port, uint16_t enc_pcr_scr)
{
#if 1
	uint32_t flush_msk = (BIT(XEC_I2C_CFG_FTTX_POS) | BIT(XEC_I2C_CFG_FTRX_POS) |
	                      BIT(XEC_I2C_CFG_FHTX_POS) | BIT(XEC_I2C_CFG_FHRX_POS));
	uint32_t port_fe_val = XEC_I2C_CFG_PORT_SET(port) | BIT(XEC_I2C_CFG_FEN_POS);

	sys_set_bit(regbase + XEC_I2C_CFG_OFS, XEC_I2C_CFG_RST_POS);
	k_busy_wait(10U);
	sys_write32(port_fe_val, regbase + XEC_I2C_CFG_OFS);
	k_busy_wait(10U);

	sys_set_bits(regbase + XEC_I2C_CFG_OFS, flush_msk);
	sys_write32(BIT(XEC_I2C_WKSR_SB_POS), regbase + XEC_I2C_WKSR_OFS);
#else
	soc_xec_pcr_reset_en(enc_pcr_scr);
#endif
}

int i2c_xec_prog_timing(mm_reg_t regbase, uint32_t freq_hz)
{
	const struct xec_i2c_timing *ptm = NULL;

	for (size_t i = 0; i < ARRAY_SIZE(xec_i2c_nl_timing_tbl); i++) {
		if (freq_hz == xec_i2c_nl_timing_tbl[i].freq_hz) {
			ptm = &xec_i2c_nl_timing_tbl[i];
			break;
		}
	}

	if (ptm == NULL) {
		return -EINVAL;
	}

	sys_write32(ptm->data_tm, regbase + XEC_I2C_DT_OFS);
	sys_write32(ptm->idle_sc, regbase + XEC_I2C_IDS_OFS);
	sys_write32(ptm->timeout_sc, regbase + XEC_I2C_TMOUT_SC_OFS);
	sys_write32(ptm->bus_clock, regbase + XEC_I2C_BCLK_OFS);
	sys_write8(ptm->rpt_sta_htm, regbase + XEC_I2C_RSHT_OFS);

	return 0;
}

uint32_t i2c_xec_freq_from_dev_config(uint32_t dev_config)
{
	uint32_t speed = I2C_SPEED_GET(dev_config);
	uint32_t fhz = 0;

	switch (speed) {
	case I2C_SPEED_STANDARD: /* 100 KHz */
		return (uint32_t)KHZ(100);
	case I2C_SPEED_FAST: /* 400 KHz */
		return (uint32_t)KHZ(400);
	case I2C_SPEED_FAST_PLUS: /* 1 MHz */
		return (uint32_t)MHZ(1);
	default:
		fhz = 0;
	}

	return fhz;
}

#if defined(CONFIG_SOC_SERIES_MEC15XX) || defined(CONFIG_SOC_SERIES_MEC172X)
uint8_t i2c_xec_get_lines(mm_reg_t regbase, const struct gpio_dt_spec *scl_gpio,
                          const struct gpio_dt_spec *sda_gpio)
{
	gpio_port_value_t sda = 0, scl = 0;
	uint8_t lines = 0;

	gpio_port_get_raw(scl_gpio->port, &scl);

	if (scl_gpio->port == sda_gpio->port) { /* both pins in same GPIO port? */
		sda = scl;
	} else {
		gpio_port_get_raw(sda_gpio->port, &sda);
	}

	if ((scl & BIT(scl_gpio->pin)) != 0) {
		lines |= BIT(XEC_I2C_SCL_LINE_POS);
	}

	if ((scl & BIT(sda_gpio->pin)) != 0) {
		lines |= BIT(XEC_I2C_SDA_LINE_POS);
	}

	return lines;
}
#else
uint8_t i2c_xec_get_lines(mm_reg_t regbase, const struct gpio_dt_spec *scl_gpio,
                          const struct gpio_dt_spec *sda_gpio)
{
	uint8_t bbcr = 0, lines = 0;

	sys_write8(BIT(XEC_I2C_BBCR_CM_POS), regbase + XEC_I2C_BBCR_OFS);
	bbcr = sys_read8(regbase + XEC_I2C_BBCR_OFS);

	if ((bbcr & BIT(XEC_I2C_BBCR_SCL_IN_POS)) != 0) {
		lines |= BIT(XEC_I2C_SCL_LINE_POS);
	}

	if ((bbcr & BIT(XEC_I2C_BBCR_SDA_IN_POS)) != 0) {
		lines |= BIT(XEC_I2C_SDA_LINE_POS);
	}

	return lines;
}
#endif

/*
 * If SCL is low sample I2C_RECOVER_SCL_LOW_RETRIES times with a 5 us delay
 * between samples. If SCL remains low then return -EBUSY
 * If SCL is High and SDA is low then loop up to I2C_RECOVER_SDA_LOW_RETRIES
 * times driving the pins:
 * Drive SCL high
 * delay I2C_RECOVER_BB_DELAY_US
 * Generate 9 clock pulses on SCL checking SDA before each falling edge of SCL
 *   If SDA goes high exit clock loop else to all 9 clocks
 * Drive SDA low, delay 5 us, release SDA, delay 5 us
 * Both lines are high then exit SDA recovery loop
 * Both lines should not be driven
 * Check both lines: if any bad return error else return success
 * NOTE 1: Bit-bang mode uses a HW MUX to switch the lines away from the I2C
 * controller logic to BB logic.
 * NOTE 2: Bit-bang mode requires HW timeouts to be disabled.
 * NOTE 3: Bit-bang mode requires the controller's configuration enable bit
 * to be set.
 * NOTE 4: The controller must be reset after using bit-bang mode.
 */
int i2c_xec_bus_recovery(mm_reg_t rb)
{
	int i = 0, j = 0, ret = 0;
	uint8_t bbcr = 0;

	bbcr = BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_SCL_POS) | BIT(XEC_I2C_BBCR_SDA_POS);
	sys_write8(bbcr, rb + XEC_I2C_BBCR_OFS);

	if (soc_test_bit8(rb + XEC_I2C_BBCR_OFS, XEC_I2C_BBCR_SCL_IN_POS) == 0) {
		for (i = 0;; i++) {
			if (i >= I2C_RECOVER_SCL_LOW_RETRIES) {
				ret = -EBUSY;
				goto recov_exit;
			}

			k_busy_wait(I2C_RECOVER_SCL_DELAY_US);

			if (soc_test_bit8(rb + XEC_I2C_BBCR_OFS, XEC_I2C_BBCR_SCL_IN_POS) != 0) {
				break; /* SCL went High */
			}
		}
	}

	if (soc_test_bit8(rb + XEC_I2C_BBCR_OFS, XEC_I2C_BBCR_SDA_IN_POS) != 0) {
		ret = 0;
		goto recov_exit;
	}

	ret = -EBUSY;
	/* SDA recovery */
	for (i = 0; i < I2C_RECOVER_SDA_LOW_RETRIES; i++) {
		/* SCL output mode and tri-stated */
		bbcr = (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CD_POS) |
			BIT(XEC_I2C_BBCR_SCL_POS) | BIT(XEC_I2C_BBCR_SDA_POS));
		sys_write8(bbcr, rb + XEC_I2C_BBCR_OFS);
		k_busy_wait(I2C_RECOVER_BB_DELAY_US);

		for (j = 0; j < 9; j++) {
			if (soc_test_bit8(rb + XEC_I2C_BBCR_OFS, XEC_I2C_BBCR_SDA_IN_POS) != 0) {
				break;
			}

			/* drive SCL low */
			bbcr = (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CD_POS) |
				BIT(XEC_I2C_BBCR_SDA_POS));
			sys_write8(bbcr, rb + XEC_I2C_BBCR_OFS);
			k_busy_wait(I2C_RECOVER_BB_DELAY_US);

			/* release SCL: pulled high by external pull-up */
			bbcr = (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_CD_POS) |
				BIT(XEC_I2C_BBCR_SCL_POS) | BIT(XEC_I2C_BBCR_SDA_POS));
			sys_write8(bbcr, rb + XEC_I2C_BBCR_OFS);
			k_busy_wait(I2C_RECOVER_BB_DELAY_US);
		}

		/* SCL is High. Produce rising edge on SCL for STOP */
		bbcr = (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_SCL_POS) |
			BIT(XEC_I2C_BBCR_DD_POS));
		sys_write8(bbcr, rb + XEC_I2C_BBCR_OFS);
		k_busy_wait(I2C_RECOVER_BB_DELAY_US);

		bbcr = (BIT(XEC_I2C_BBCR_EN_POS) | BIT(XEC_I2C_BBCR_SCL_POS) |
			BIT(XEC_I2C_BBCR_SDA_POS));
		sys_write8(bbcr, rb + XEC_I2C_BBCR_OFS);
		k_busy_wait(I2C_RECOVER_BB_DELAY_US);

		/* check if SCL and SDA are both high */
		bbcr = sys_read8(rb + XEC_I2C_BBCR_OFS) &
		       (BIT(XEC_I2C_BBCR_SCL_IN_POS) | BIT(XEC_I2C_BBCR_SDA_IN_POS));

		if (bbcr == (BIT(XEC_I2C_BBCR_SCL_IN_POS) | BIT(XEC_I2C_BBCR_SDA_IN_POS))) {
			ret = 0; /* successful recovery */
			goto recov_exit;
		}
	}

recov_exit:
	/* BB mode disable reconnects SCL and SDA to I2C logic. */
	sys_write8(BIT(XEC_I2C_BBCR_CM_POS), rb + XEC_I2C_BBCR_OFS);

	return ret;
}
