/*
 * Copyright (c) 2022 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(pm_device, CONFIG_SPI_LOG_LEVEL);

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

void main(void)
{
	struct vci_regs * const vci = (struct vci_regs *)DT_REG_ADDR(DT_NODELABEL(vci0));
	const struct device *spi_dev;
	int err;
	int16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	LOG_INF("MEC172x EVB: Test Device Power Management");

	spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi0));
	if (!device_is_ready(spi_dev)) {
		LOG_ERR("SPI controller device not ready!");
		return;
	}

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!device_is_ready(adc_channels[i].dev)) {
			LOG_ERR("ADC controller device not ready\n");
			return;
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			LOG_ERR("Could not setup channel #%d (%d)\n", i, err);
			return;
		}
	}

	vci->CONFIG = MCHP_VCI_FW_EXT_SEL;
	vci->LATCH_EN = BIT(1); /* VCI_IN1 (GPIO_0162) connect to EVB switch S4 */
	vci->INPUT_EN = BIT(1);
	vci->LATCH_RST = BIT(1);
	vci->PEDGE_DET = BIT(1); /* clear R/WC status bit */
	vci->NEDGE_DET = BIT(1); /* clear R/WC status bit */

	LOG_INF("Pause: Shut down debugger and press EVB button S4");

	/* did we get a pulse (press and release) of button S4 ? */
	while (!((vci->PEDGE_DET & BIT(1)) && (vci->NEDGE_DET & BIT(1)))) {
		;
	}

#ifdef APP_DIS_JTAG
	struct ecs_regs * const ecs = (struct ecs_regs *)DT_REG_ADDR(DT_NODELABEL(ecs));

	ecs->DEBUG_CTRL = 0; /* disable JTAG/SWD */
#endif

	while (1) {
		LOG_INF("ADC reading:\n");
		for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
			int32_t val_mv;

			LOG_INF("- %s, channel %d: ",
				adc_channels[i].dev->name,
				adc_channels[i].channel_id);

			(void)adc_sequence_init_dt(&adc_channels[i], &sequence);

			err = adc_read(adc_channels[i].dev, &sequence);
			if (err < 0) {
				LOG_ERR("Could not read (%d)\n", err);
				continue;
			} else {
				LOG_INF("%"PRId16, buf);
			}

			/* conversion to mV may not be supported, skip if not */
			val_mv = buf;
			err = adc_raw_to_millivolts_dt(&adc_channels[i],
						       &val_mv);
			if (err < 0) {
				LOG_ERR(" (value in mV not available)\n");
			} else {
				LOG_INF(" = %"PRId32" mV\n", val_mv);
			}
		}

		/* deep sleep residency is 2000 ms in board DTS.
		 * Instruct kernel to sleep for larger value to trigger
		 * entry to suspend state.
		 */
		k_sleep(K_MSEC(3000));
	}
}
