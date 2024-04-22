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
uint8_t tgt_tx_buff[88] = {
                           0x1,  0x2,  0x3,  0x4,  0x5,  0x6,  0x7,  0x8,
                           0x9,  0xa,  0xb,  0xc,  0xd,  0xe,  0xf,  0x10,
                           0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
                           0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20,
                           0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
                           0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30,
                           0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
                           0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40,
                           0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8,
                           0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8,
                           0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8};

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
    uint8_t txData[32];

    for (int i = 0; i<32; i++) {
        txData[i] = i;
    }

    if (i3c_set_data(dev, &txData[0], 32) < 0) {
	printf("Cannot write\n");
	return;
    }
}

static void receive_data(const struct device *dev)
{
    uint8_t rxData[32];
    if (i3c_get_data(dev, &rxData[0], 32) < 0) {
	printf("Cannot read\n");
	return;
    }
    printf("rxData: ");
    for(uint8_t i=0; i < 32; i++)
    printf("%x ", rxData[i]);
}
#endif

void print_buf(uint8_t *buf, uint32_t len)
{
    uint32_t j, k;

    printk("0x00000000: ");
    for (j=0;j<len;j++)
    {
        printk("%02x ", buf[j]);
        k = j+1;

        if ((k<len) && !(k % 16)) printk("\r\n0x%08x: ", k);
    }
    printk("\r\n");
}

#if defined(I3C0) || defined(I3C1)
static int target_ibi_cb(struct i3c_device_desc *target, struct i3c_ibi_payload *payload)
{
    if(payload == NULL)
    {
	return 1;
    }

    printk("Enter [%s] - RxD %d bytes of payload\n", __FUNCTION__, payload->payload_len);
    if(payload->payload_len) {
        print_buf(&payload->payload[0], payload->payload_len);
    }

    return 0;
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
ret = i3c_target_tx_write(dev, &tgt_tx_buff[0], 88);
#endif

#ifdef I3C0
        const struct device *const dev_i3c_h = DEVICE_DT_GET(DT_NODELABEL(i3c0_dev0));

        if (!device_is_ready(dev_i3c_h)) {
                printk("sensor: device not ready.\n");
                return 0;
        }

    //i3c_ibi(dev_i3c_h, &target_ibi_cb);
    transfer_data(dev_i3c_h);
    receive_data(dev_i3c_h);

    //i3c_ibi(dev_i3c_h, &target_ibi_cb);

#endif

	k_sleep(K_FOREVER);
	return 0;
}
