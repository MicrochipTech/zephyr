/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/introspect_i3c.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

#if DT_NODE_EXISTS(DT_NODELABEL(i3c1))
 #if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c1), okay)
  #if DT_PROP(DT_NODELABEL(i3c1), enable_i3c1_as_host)
    #warning "i3c1 boot as Tgt, rqt to Host"
  #else
    #warning "i3c1 boot as Target"
    #define I3C1_AS_TARGET
  #endif
 #endif
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(i3c0))
 #if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c0), okay)
    #warning "i3c0 Selected"
    #define I3C0
 #endif
#endif

#ifdef I3C1_AS_TARGET
#include "zephyr/drivers/i3c.h"
static int tgt0_write_requested_cb(struct i3c_target_config *config);
static int tgt0_write_received_cb(struct i3c_target_config *config, uint8_t val);
static int tgt0_read_requested_cb(struct i3c_target_config *config, uint8_t *val);
static int tgt0_read_processed_cb(struct i3c_target_config *config, uint8_t *val);
static int tgt0_stop_cb(struct i3c_target_config *config);

struct i3c_target_callbacks tgt0_cbs= {
    .write_requested_cb   = tgt0_write_requested_cb,
    .write_received_cb    = tgt0_write_received_cb,
    .read_requested_cb    = tgt0_read_requested_cb,
    .read_processed_cb    = tgt0_read_processed_cb,
    .stop_cb              = tgt0_stop_cb,
};

struct i3c_target_config tgt0_cfg;
uint8_t tgt_tx_buff[10] = {0x4b, 0x7f, 0x73, 0x66, 0x7d, 0x13, 0x69, 0x25, 0xb4, 0xbf};

static int tgt0_write_requested_cb(struct i3c_target_config *config)
{
    printf("[%s]\n", __FUNCTION__);
    return 0;
}

static int tgt0_write_received_cb(struct i3c_target_config *config, uint8_t val)
{
    printf("[%s] Data => 0x%02x\n", __FUNCTION__, val);
    return 0;
}

static int tgt0_read_requested_cb(struct i3c_target_config *config, uint8_t *val)
{
    printf("[%s]\n", __FUNCTION__);
    return 0;
}

static int tgt0_read_processed_cb(struct i3c_target_config *config, uint8_t *val)
{
    printf("[%s]\n", __FUNCTION__);
    return 0;
}

static int tgt0_stop_cb(struct i3c_target_config *config)
{
    printf("[%s]\n", __FUNCTION__);
    return 0;
}
#endif

#if defined(I3C0) || defined(I3C1)
static void transfer_data(const struct device *dev)
{
    uint8_t txData[10] = {0,1,2,3,4,5,6,7,8,9};

    printf("Tx:\n ");
    for(uint8_t i=0; i < 10; i++)
        printf("%x ", txData[i]);
    printf("\n");

    if (i3c_set_data(dev, &txData[0], 10) < 0) {
	printf("Cannot write\n");
	return;
    }
}

static void receive_data(const struct device *dev)
{
    uint8_t rxData[10];
    if (i3c_get_data(dev, &rxData[0], 10) < 0) {
	printf("Cannot read\n");
	return;
    }
    printf("Rx:\n ");
    for(uint8_t i=0; i < 10; i++)
        printf("%x ", rxData[i]);
    printf("\n");
}

static void transceive_data(const struct device *dev)
{
    uint8_t txData[10] = {10,11,12,13,14,15,16,17,18,19};
    uint8_t rxData[10];

    printf("TxRx\n ");
    printf("tx ");
    for(uint8_t i=0; i < 10; i++)
        printf("%x ", txData[i]);
    printf("\n");

    if (i3c_set_get_data(dev, &txData[0], 10, &rxData[0], 10) < 0) {
        printf("Cannot write read\n");
        return;
    }

    printf(" rx ");
    for(uint8_t i=0; i < 10; i++)
        printf("%x ", rxData[i]);
    printf("\n");
}

#endif

int main(void)
{
#ifdef I3C1_AS_TARGET
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(i3c1));

        if (!device_is_ready(dev)) {
                printk("I3C1_AS_TARGET not ready.\n");
                return 0;
        }

tgt0_cfg.callbacks = &tgt0_cbs;
int ret = i3c_target_register(dev, &tgt0_cfg);
ret = i3c_target_tx_write(dev, &tgt_tx_buff[0], 10);
#endif

#ifdef I3C0
        const struct device *const dev_i3c_h = DEVICE_DT_GET(DT_NODELABEL(i3c0_dev0));

        if (!device_is_ready(dev_i3c_h)) {
                printk("sensor: device not ready.\n");
                return 0;
        }

    transfer_data(dev_i3c_h);
    receive_data(dev_i3c_h);
    transceive_data(dev_i3c_h);
#endif

	k_sleep(K_FOREVER);
	return 0;
}
