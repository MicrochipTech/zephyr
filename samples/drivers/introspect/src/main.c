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
//uint8_t tgt_tx_buff[10] = {0x4b, 0x7f, 0x73, 0x66, 0x7d, 0x13, 0x69, 0x25, 0xb4, 0xbf};
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

//I3C1 as Target
//struct k_timer timer0;
//void tgt_test_sir_ibi_cb(struct k_timer *timer)
void tgt_test_sir_ibi(const struct device *dev)
{
	uint8_t sir_ibi_payload[3] = {0x2E, 0x01, 0x02};
	struct i3c_ibi sir_ibi;

//	struct device *dev = (struct device *)k_timer_user_data_get(timer);

	//SIR IBI with MDB only
	sir_ibi.ibi_type = I3C_IBI_TARGET_INTR;
	sir_ibi.payload = &sir_ibi_payload[0];
	sir_ibi.payload_len = 1;
	if (0 != i3c_ibi_raise(dev, &sir_ibi))
	{
		printf("SIR IBI with MDB only failed!!\n");
	}

        //SIR IBI with MDB + 1 byte
        sir_ibi.ibi_type = I3C_IBI_TARGET_INTR;
        sir_ibi.payload = &sir_ibi_payload[0];
        sir_ibi.payload_len = 2; 
	if (0 != i3c_ibi_raise(dev, &sir_ibi))
	{
		printf("SIR IBI with MDB + 1 byte of payload failed!!\n");
	}

        //SIR IBI with MDB + 2 byte
        sir_ibi.ibi_type = I3C_IBI_TARGET_INTR;
        sir_ibi.payload = &sir_ibi_payload[0];
        sir_ibi.payload_len = 3;
	if (0 != i3c_ibi_raise(dev, &sir_ibi))
	{
		printf("SIR IBI with MDB + 2 bytes of payload failed!!\n");
	}
}

void tgt_test_mr_ibi(const struct device *dev)
{
	struct i3c_ibi mr_ibi = {
	    .ibi_type = I3C_IBI_CONTROLLER_ROLE_REQUEST,
	    .payload = NULL,
	};

	if (0 != i3c_ibi_raise(dev, &mr_ibi))
	{
		printf("MR IBI failed!!\n");
	}
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
	//struct device *dev = DEVICE_DT_GET(DT_NODELABEL(i3c1));

        if (!device_is_ready(dev)) {
                printk("I3C1_AS_TARGET not ready.\n");
                return 0;
        }
//For private transfer
//tgt0_cfg.callbacks = &tgt0_cbs;
//int ret = i3c_target_register(dev, &tgt0_cfg);
//ret = i3c_target_tx_write(dev, &tgt_tx_buff[0], 64);

k_sleep(K_MSEC(10000));
tgt_test_sir_ibi(dev);
//tgt_test_mr_ibi(dev);
#endif

#ifdef I3C0
        const struct device *const dev_i3c_h = DEVICE_DT_GET(DT_NODELABEL(i3c0_dev0));

        if (!device_is_ready(dev_i3c_h)) {
                printk("sensor: device not ready.\n");
                return 0;
        }

 //   transfer_data(dev_i3c_h);
   // receive_data(dev_i3c_h);
   // transceive_data(dev_i3c_h);
i3c_ibi(dev_i3c_h, &target_ibi_cb);
#endif

	k_sleep(K_FOREVER);
	return 0;
}
