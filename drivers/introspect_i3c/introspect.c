/* Introspect driver to test I3C
 *
 * Copyright (c) 2024 Microchip Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT microchip_introspect

#include <zephyr/drivers/introspect_i3c.h>
#include <zephyr/drivers/i3c.h>

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "Introspect driver enabled without any devices"
#undef DT_DRV_COMPAT
#endif

struct introspect_config {
	const struct device *bus;
	const struct i3c_device_id dev_id;
};

struct introspect_data {
	struct i3c_device_desc *i3c_dev;
};

static int i3c_data_get(const struct device *dev,
			       uint8_t *val, uint32_t len)
{
	struct introspect_data *data = dev->data;
int ret;
  ret = i3c_read(data->i3c_dev, val, len);

	return ret;
}

static int i3c_data_set(const struct device *dev,
			    uint8_t *val, uint32_t len)
{
		struct introspect_data *data = dev->data;
int ret;

  ret = i3c_write(data->i3c_dev, val, len);
	return ret;
}

static int i3c_data_set_get(const struct device *dev,
          uint8_t *set_val, uint32_t slen,
          uint8_t *get_val, uint32_t glen)
{
    struct introspect_data *data = dev->data;
  int ret;

  ret = i3c_write_read(data->i3c_dev, set_val, slen, get_val, glen);
  return ret;
}

void i3c_test_ibi(const struct device *dev, tgt_cb_t tgt_cb)
{
    struct i3c_device_desc *target = NULL;
		struct introspect_data *data = dev->data;

        target = data->i3c_dev;

        target->ibi_cb = tgt_cb;

        printk("i3c_test_ibi: Enable IBI on target\n");

        i3c_ibi_enable(target);
}

static const struct introspect_driver_api intro_driver_api = {
#ifdef DT_DRV_COMPAT
	/* Introspect as Target */
	.set_data = i3c_data_set,
 	.get_data = i3c_data_get,
 	.set_get_data = i3c_data_set_get,
	.test_ibi = i3c_test_ibi,
#else
	/* Introspect as Master */
#endif
};

static int introspect_init_chip(const struct device *dev)
{
	const struct introspect_config * const cfg = dev->config;
	struct introspect_data *data = dev->data;

	if (cfg->bus != NULL) {
		/*
		 * Need to grab the pointer to the I3C device descriptor
		 * before we can talk to the sensor.
		 */
		data->i3c_dev = i3c_device_find(cfg->bus, &cfg->dev_id);
		if (data->i3c_dev == NULL) {
			printk("Cannot find I3C device descriptor");
			return -ENODEV;
		}
	}

	return 0;
}

static int introspect_init(const struct device *dev)
{
	printf("Introspect driver called\n");
#ifdef DT_DRV_COMPAT
	if (introspect_init_chip(dev) < 0) {
		printk("Failed to initialize");
		return -EIO;
	}
#endif
	return 0;
}

#ifdef DT_DRV_COMPAT
#warning "Introspect as Target"

#define XEC_intro_CONFIG(inst)                                                    \
        static const struct introspect_config introspect_config_##inst = {            \
                .bus = DEVICE_DT_GET(DT_INST_BUS(inst)),                    \
                .dev_id = I3C_DEVICE_ID_DT_INST(inst),      \
        }

#define INTROSPECT_DEFINE(inst)                                                 \
        static struct introspect_data introspect_data_##inst;                           \
        XEC_intro_CONFIG(inst);                                              \
        SENSOR_DEVICE_DT_INST_DEFINE(inst, introspect_init, NULL, &introspect_data_##inst,      \
                              &introspect_config_##inst, POST_KERNEL,           \
                              CONFIG_INTROSPECT_INIT_PRIORITY, &intro_driver_api);
#else

#warning "Introspect as Master"
#define INTROSPECT_DEFINE(inst)                                                 \
        SENSOR_DEVICE_DT_INST_DEFINE(inst, introspect_init, NULL, NULL,      \
                              NULL, POST_KERNEL,           \
                              CONFIG_INTROSPECT_INIT_PRIORITY, &intro_driver_api);
#endif

DT_INST_FOREACH_STATUS_OKAY(INTROSPECT_DEFINE)
