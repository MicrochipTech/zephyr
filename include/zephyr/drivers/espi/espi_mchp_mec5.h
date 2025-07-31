/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ESPI_ESPI_MCHP_MEC5_H_
#define ZEPHYR_INCLUDE_DRIVERS_ESPI_ESPI_MCHP_MEC5_H_

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/sys/slist.h>

enum mec5_espi_sram_bar_id {
	MEC5_ESPI_SRAM_BAR_0 = 0,
	MEC5_ESPI_SRAM_BAR_1,
	MEC5_ESPI_SRAM_BAR_ID_MAX
};

enum mec5_espi_sram_region_size {
	MEC5_ESPI_SRAM_REGION_SIZE_1B = 0,
	MEC5_ESPI_SRAM_REGION_SIZE_2B,
	MEC5_ESPI_SRAM_REGION_SIZE_4B,
	MEC5_ESPI_SRAM_REGION_SIZE_8B,
	MEC5_ESPI_SRAM_REGION_SIZE_16B,
	MEC5_ESPI_SRAM_REGION_SIZE_32B,
	MEC5_ESPI_SRAM_REGION_SIZE_64B,
	MEC5_ESPI_SRAM_REGION_SIZE_128B,
	MEC5_ESPI_SRAM_REGION_SIZE_256B,
	MEC5_ESPI_SRAM_REGION_SIZE_512B,
	MEC5_ESPI_SRAM_REGION_SIZE_1KB,
	MEC5_ESPI_SRAM_REGION_SIZE_2KB,
	MEC5_ESPI_SRAM_REGION_SIZE_4KB,
	MEC5_ESPI_SRAM_REGION_SIZE_8KB,
	MEC5_ESPI_SRAM_REGION_SIZE_16KB,
	MEC5_ESPI_SRAM_REGION_SIZE_32KB,
};

enum mec5_espi_sram_access {
	MEC5_ESPI_SRAM_ACCESS_NONE = 0,
	MEC5_ESPI_SRAM_ACCESS_RO,
	MEC5_ESPI_SRAM_ACCESS_WO,
	MEC5_ESPI_SRAM_ACCESS_RW,
};

struct mec5_espi_sram_bar_cfg {
	void *buf;
	uint8_t szp2;
	uint8_t access;
};

enum mchp_espi_pc_aec_ien_flags {
	MCHP_ESPI_PC_AEC_IEN_FLAG_IBF = BIT(0),
	MCHP_ESPI_PC_AEC_IEN_FLAG_OBE = BIT(1),
	MCHP_ESPI_PC_AEC_IEN_FLAG_SIRQ_OBE = BIT(2),
};

enum mchp_espi_pc_kbc_ien_flags {
	MCHP_ESPI_PC_KBC_IEN_FLAG_IBF = BIT(0),
	MCHP_ESPI_PC_KBC_IEN_FLAG_OBE = BIT(1),
	MCHP_ESPI_PC_KBC_IEN_FLAG_SIRQ_K = BIT(2),
	MCHP_ESPI_PC_KBC_IEN_FLAG_SIRQ_M = BIT(3),
};

enum mchp_espi_pc_emi_ien_flags {
	MCHP_ESPI_PC_KBC_IEN_FLAG_H2EC = BIT(0),
	MCHP_ESPI_PC_KBC_IEN_FLAG_SIRQ_SWI = BIT(1),
	MCHP_ESPI_PC_KBC_IEN_FLAG_SIRQ_EC2H = BIT(2),
};

enum mchp_espi_pc_bdp_ien_flags {
	MCHP_ESPI_PC_BDP_IEN_FLAG_FIFO_THRH = BIT(0),
};

enum mchp_espi_pc_mbox_ien_flags {
	MCHP_ESPI_PC_MBOX_IEN_FLAG_H2EC = BIT(0),
	MCHP_ESPI_PC_MBOX_IEN_FLAG_SIRQ_EC_WR = BIT(1),
	MCHP_ESPI_PC_MBOX_IEN_FLAG_SIRQ_SMI = BIT(2),
};

enum mchp_espi_pc_host_uart_ien_flags {
	MCHP_ESPI_EC_HUART_IEN_EC = BIT(0),
	MCHP_ESPI_PC_HUART_IEN_FLAG_SIRQ = BIT(1),
};

/* ---- SRAM0 and SRAM1 BAR Memory Access ---- */
int mchp_espi_sram_bar_get_size(const struct device *dev, uint8_t sram_bar_id,
				size_t *size_in_bytes);

int mchp_espi_sram_bar_get_access(const struct device *dev, uint8_t sram_bar_id, int *access);

int mchp_espi_sram_bar_mem_rd(const struct device *dev, uint8_t sram_bar_id, uint16_t offset,
			      uint8_t *dest, size_t destsz, size_t *rdsz);

int mchp_espi_sram_bar_mem_wr(const struct device *dev, uint8_t sram_bar_id, uint16_t offset,
			      const uint8_t *src, size_t srcsz, size_t *wrsz);

int mchp_espi_sram_bar_mem_fill(const struct device *dev, uint8_t sram_bar_id,
				uint16_t offset, size_t fillsz, uint8_t val);

/* ---- Generic Host Device API ---- */
#define ESPI_HAE_CFG_LDN_POS	0
#define ESPI_HAE_CFG_LDN_MSK0	0xffu
#define ESPI_HAE_CFG_LDN_MSK	((ESPI_HAE_CFG_LDN_MSK0) << ESPI_HAE_CFG_LDN_POS)

#define ESPI_HAE_CFG_GET_LDN(h)	\
	(((uint32_t)(h) >> ESPI_HAE_CFG_LDN_POS) & ESPI_HAE_CFG_LDN_MSK0)

#define ESPI_HAE_CFG_SET_LDN(ldn)	\
	(((uint32_t)(ldn) & ESPI_HAE_CFG_LDN_MSK0) << ESPI_HAE_CFG_LDN_POS)

/* eSPI peripheral channel host facing devices have different API's with
 * the requirement the first API is "host_access_enable". We can cast
 * each driver's API struct to this one allowing the eSPI driver to invoke
 * host_access_enable after the Host eSPI controller de-asserts PLTRST#.
 */
struct espi_pc_driver_api {
	int (*host_access_enable)(const struct device *dev, uint8_t enable, uint32_t cfg);
	int (*intr_enable)(const struct device *dev, uint8_t enable, uint32_t flags);
};

static inline int espi_pc_host_access(const struct device *dev, uint8_t en, uint32_t cfg)
{
	const struct espi_pc_driver_api *api = (const struct espi_pc_driver_api *)dev->api;

	if (!api->host_access_enable) {
		return -ENOTSUP;
	}

	return api->host_access_enable(dev, en, cfg);
}

static inline int espi_pc_intr_enable(const struct device *dev, uint8_t en, uint32_t flags)
{
	const struct espi_pc_driver_api *api = (const struct espi_pc_driver_api *)dev->api;

	if (!api->intr_enable) {
		return -ENOTSUP;
	}

	return api->intr_enable(dev, en, flags);
}

/* ---- Host Device BDP ---- */
enum mec5_bdp_event {
	MEC5_BDP_EVENT_NONE = 0,
	MEC5_BDP_EVENT_IO,
	MEC5_BDP_EVENT_OVERRUN,
};

struct host_io_data {
	uint32_t data;
	uint8_t start_byte_lane;
	uint8_t size;
};

typedef void (*mchp_espi_pc_bdp_callback_t)(const struct device *dev,
					    struct host_io_data *hiod,
					    void *user_data);

struct mchp_espi_pc_bdp_driver_api {
	int (*host_access_enable)(const struct device *dev, uint8_t enable, uint32_t cfg);
	int (*intr_enable)(const struct device *dev, int intr_en, uint32_t flags);
	int (*has_data)(const struct device *dev);
	int (*get_data)(const struct device *dev, struct host_io_data *data);
#ifdef CONFIG_ESPI_MEC5_BDP_CALLBACK
	int (*set_callback)(const struct device *dev, mchp_espi_pc_bdp_callback_t cb,
			    void *user_data);
#endif
};

static inline int mchp_espi_pc_bdp_intr_enable(const struct device *dev, int intr_en,
					       uint32_t flags)
{
	const struct mchp_espi_pc_bdp_driver_api *api =
		(const struct mchp_espi_pc_bdp_driver_api *)dev->api;

	if (!api->intr_enable) {
		return -ENOTSUP;
	}

	return api->intr_enable(dev, intr_en, flags);
}

static inline int mchp_espi_pc_bdp_has_data(const struct device *dev)
{
	const struct mchp_espi_pc_bdp_driver_api *api =
		(const struct mchp_espi_pc_bdp_driver_api *)dev->api;

	if (!api->has_data) {
		return -ENOTSUP;
	}

	return api->has_data(dev);
}

static inline int mchp_espi_pc_bdp_get_data(const struct device *dev, struct host_io_data *hiod)
{
	const struct mchp_espi_pc_bdp_driver_api *api =
		(const struct mchp_espi_pc_bdp_driver_api *)dev->api;

	if (!api->get_data) {
		return -ENOTSUP;
	}

	return api->get_data(dev, hiod);
}

#ifdef CONFIG_ESPI_MEC5_BDP_CALLBACK
static inline int mchp_espi_pc_bdp_set_callback(const struct device *dev,
						mchp_espi_pc_bdp_callback_t callback,
						void *user_data)
{
	const struct mchp_espi_pc_bdp_driver_api *api =
		(const struct mchp_espi_pc_bdp_driver_api *)dev->api;

	if (!api->set_callback) {
		return -ENOTSUP;
	}

	return api->set_callback(dev, callback, user_data);
}
#endif

/* Legacy 8042 Keyboard controller on eSPI peripheral channel */

#define ESPI_MCHP_LPC_REQ_FLAG_DIR_POS 0
#define ESPI_MCHP_LPC_REQ_FLAG_RD 0
#define ESPI_MCHP_LPC_REQ_FLAG_WR BIT(ESPI_MCHP_LPC_REQ_FLAG_DIR_POS)

#ifdef CONFIG_ESPI_MEC5_KBC_CALLBACK
#define ESPI_MCHP_KBC_CALLBACK_IBF 0
#define ESPI_MCHP_KBC_CALLBACK_OBE 1

typedef void (*mchp_espi_pc_kbc_callback_t)(const struct device *dev,
					    struct host_io_data *hiod,
					    void *user_data);
#endif

struct mchp_espi_pc_kbc_driver_api {
	int (*host_access_enable)(const struct device *dev, uint8_t enable, uint32_t cfg);
	int (*intr_enable)(const struct device *dev, uint8_t enable, uint32_t flags);
	int (*lpc_request)(const struct device *dev, enum lpc_peripheral_opcode op,
			   uint32_t *data, uint32_t flags);
#ifdef CONFIG_ESPI_MEC5_KBC_CALLBACK
	int (*set_callback)(const struct device *dev, mchp_espi_pc_kbc_callback_t cb,
			    void *user_data, uint8_t cb_event);
#endif
};

static inline int mchp_espi_pc_kbc_intr_enable(const struct device *dev,
					       uint8_t enable, uint32_t flags)
{
	const struct mchp_espi_pc_kbc_driver_api *api =
		(const struct mchp_espi_pc_kbc_driver_api *)dev->api;

	if (!api->intr_enable) {
		return -ENOTSUP;
	}

	return api->intr_enable(dev, enable, flags);
}

static inline int mchp_espi_pc_kbc_lpc_request(const struct device *dev,
					       enum lpc_peripheral_opcode op,
					       uint32_t *data, uint32_t flags)
{
	const struct mchp_espi_pc_kbc_driver_api *api =
		(const struct mchp_espi_pc_kbc_driver_api *)dev->api;

	if (!api->lpc_request) {
		return -ENOTSUP;
	}

	return api->lpc_request(dev, op, data, flags);
}

#ifdef CONFIG_ESPI_MEC5_KBC_CALLBACK
static inline int mchp_espi_pc_kbc_set_callback(const struct device *dev,
						mchp_espi_pc_kbc_callback_t cb,
						void *user_data, uint8_t cb_event)
{
	const struct mchp_espi_pc_kbc_driver_api *api =
		(const struct mchp_espi_pc_kbc_driver_api *)dev->api;

	if (!api->set_callback) {
		return -ENOTSUP;
	}

	return api->set_callback(dev, cb, user_data, cb_event);
}
#endif

/* -------- ACPI EC -------- */

#define MCHP_ESPI_AEC_EV_FLAG_4BYTE	BIT(0)

#define MCHP_ESPI_AEC_EV_NONE		0
#define MCHP_ESPI_AEC_EV_CMD		1
#define MCHP_ESPI_AEC_EV_DATA		2
#define MCHP_ESPI_AEC_EV_HOBE		3

enum mchp_espi_pc_acpi_ec_op {
	MCHP_ESPI_AEC_OP_GET_RXDATA = 0,
	MCHP_ESPI_AEC_OP_SEND_DATA,
	MCHP_ESPI_AEC_OP_UD_SET, /* data(0) specifies UD0A or (1) UD1A */
	MCHP_ESPI_AEC_OP_UD_CLR,
	MCHP_ESPI_AEC_OP_GEN_SCI, /* data(1) set and generate SCI else clear SCI status */
	MCHP_ESPI_AEC_OP_GEN_SMI, /* data(1) set and generate SMI else clear SMI status */
};

struct mchp_espi_acpi_ec_event {
	uint32_t cmd_data;
	uint8_t ev_type;
	uint8_t flags;
};

typedef void (*mchp_espi_pc_aec_callback_t)(const struct device *dev,
					    struct mchp_espi_acpi_ec_event *ev,
					    void *user_data);

struct mchp_espi_pc_aec_driver_api {
	int (*host_access_enable)(const struct device *dev, uint8_t enable, uint32_t cfg);
	int (*intr_enable)(const struct device *dev, uint8_t enable, uint32_t flags);
	int (*lpc_request)(const struct device *dev, enum lpc_peripheral_opcode op,
			   uint32_t *data, uint32_t flags);
	int (*set_callback)(const struct device *dev, mchp_espi_pc_aec_callback_t,
			    void *userdata);
	int (*operation)(const struct device *dev, enum mchp_espi_pc_acpi_ec_op op,
			 uint32_t opdata, uint32_t *rxdata);
};

static inline int mchp_espi_pc_aec_intr_enable(const struct device *dev,
					       uint8_t enable, uint32_t flags)
{
	const struct mchp_espi_pc_aec_driver_api *api =
		(const struct mchp_espi_pc_aec_driver_api *)dev->api;

	if (!api->intr_enable) {
		return -ENOTSUP;
	}

	return api->intr_enable(dev, enable, flags);
}

static inline int mchp_espi_pc_aec_lpc_request(const struct device *dev,
					       enum lpc_peripheral_opcode op,
					       uint32_t *data, uint32_t flags)
{
	const struct mchp_espi_pc_aec_driver_api *api =
		(const struct mchp_espi_pc_aec_driver_api *)dev->api;

	if (!api->lpc_request) {
		return -ENOTSUP;
	}

	return api->lpc_request(dev, op, data, flags);
}

static inline int mchp_espi_pc_aec_set_callback(const struct device *dev,
						mchp_espi_pc_aec_callback_t cb,
						void *userdata)
{
	const struct mchp_espi_pc_aec_driver_api *api =
		(const struct mchp_espi_pc_aec_driver_api *)dev->api;

	if (!api->set_callback) {
		return -ENOTSUP;
	}

	return api->set_callback(dev, cb, userdata);
}

static inline int mchp_espi_pc_aec_operation(const struct device *dev,
					     enum mchp_espi_pc_acpi_ec_op op,
					     uint32_t opdata, uint32_t *rxdata)
{
	const struct mchp_espi_pc_aec_driver_api *api =
		(const struct mchp_espi_pc_aec_driver_api *)dev->api;

	if (!api->operation) {
		return -ENOTSUP;
	}

	return api->operation(dev, op, opdata, rxdata);
}

/* -------- EMI -------- */
enum mchp_emi_region_id {
	MCHP_EMI_MR_0 = 0,
	MCHP_EMI_MR_1,
};

enum mchp_emi_opcode {
	MCHP_EMI_OPC_MR_DIS = 0,	/* disable a previously configured memory window */
	MCHP_EMI_OPC_MR_EN,		/* re-enable a previously configured memory window */
	MCHP_EMI_OPC_MBOX_EC_IRQ_DIS,	/* disable EMI Host-to-EC mailbox interrupt */
	MCHP_EMI_OPC_MBOX_EC_IRQ_EN,	/* enable EMI Host-to-EC mailbox interrupt */
	MCHP_EMI_OPC_MBOX_EC_TO_HOST_WR,
};

/* @brief MCHP EMI memory region description. Address 4-byte aligned. Sizes multiple of 4 bytes */
struct mchp_emi_mem_region {
	void *memptr;
	uint16_t rdsz;
	uint16_t wrsz;
};

typedef void (*mchp_espi_pc_emi_callback_t)(const struct device *dev,
					    uint32_t emi_mbox_data, void *data);

struct mchp_espi_pc_emi_driver_api {
	int (*host_access_enable)(const struct device *dev, uint8_t enable, uint32_t cfg);
	int (*intr_enable)(const struct device *dev, uint8_t enable, uint32_t flags);
	int (*configure_mem_region)(const struct device *dev, struct mchp_emi_mem_region *mr,
				    uint8_t region_id);
	int (*set_callback)(const struct device *dev, mchp_espi_pc_emi_callback_t callback,
			    void *user_data);
	int (*request)(const struct device *dev, enum mchp_emi_opcode op, uint32_t *data);
};

static inline int mchp_espi_pc_emi_intr_enable(const struct device *dev,
					       uint8_t enable, uint32_t flags)
{
	const struct mchp_espi_pc_aec_driver_api *api =
		(const struct mchp_espi_pc_aec_driver_api *)dev->api;

	if (!api->intr_enable) {
		return -ENOTSUP;
	}

	return api->intr_enable(dev, enable, flags);
}

static inline int mchp_espi_pc_emi_config_mem_region(const struct device *dev,
						     struct mchp_emi_mem_region *mr,
						     int region_id)
{
	const struct mchp_espi_pc_emi_driver_api *api =
		(const struct mchp_espi_pc_emi_driver_api *)dev->api;

	if (!api->configure_mem_region) {
		return -ENOTSUP;
	}

	return api->configure_mem_region(dev, mr, region_id);
}

static inline int mchp_espi_pc_emi_set_callback(const struct device *dev,
						mchp_espi_pc_emi_callback_t callback,
						void *user_data)
{
	const struct mchp_espi_pc_emi_driver_api *api =
		(const struct mchp_espi_pc_emi_driver_api *)dev->api;

	if (!api->set_callback) {
		return -ENOTSUP;
	}

	return api->set_callback(dev, callback, user_data);
}

static inline int mchp_espi_pc_emi_request(const struct device *dev,
					   enum mchp_emi_opcode op, uint32_t *data)
{
	const struct mchp_espi_pc_emi_driver_api *api =
		(const struct mchp_espi_pc_emi_driver_api *)dev->api;

	if (!api->request) {
		return -ENOTSUP;
	}

	return api->request(dev, op, data);
}

/* -------- Mailbox -------- */

struct mchp_espi_pc_mbox_driver_api {
	int (*host_access_enable)(const struct device *dev, uint8_t enable, uint32_t cfg);
	int (*intr_enable)(const struct device *dev, uint8_t enable, uint32_t flags);
#ifdef ESPI_MEC5_MAILBOX_CALLBACK
	int (*set_callback)(const struct device *dev, mchp_espi_pc_mbox_callback_t callback,
			    void *user_data)
#endif
};

static inline int mchp_espi_pc_mbox_intr_enable(const struct device *dev,
						uint8_t enable, uint32_t flags)
{
	const struct mchp_espi_pc_mbox_driver_api *api =
		(const struct mchp_espi_pc_mbox_driver_api *)dev->api;

	if (!api->intr_enable) {
		return -ENOTSUP;
	}

	return api->intr_enable(dev, enable, flags);
}

#ifdef ESPI_MEC5_MAILBOX_CALLBACK

struct mchp_mbox_data {
	uint8_t id;
	uint8_t data;
};

typedef void (*mchp_espi_pc_mbox_callback_t)(const struct device *dev,
					     uint32_t mbox_info, void *data);

static inline int mchp_espi_pc_mbox_set_callback(const struct device *dev,
						 mchp_espi_pc_mbox_callback_t callback,
						 void *user_data)
{
	const struct mchp_espi_pc_mbox_driver_api *api =
		(const struct mchp_espi_pc_mbox_driver_api *)dev->api;

	if (!api->set_callback) {
		return -ENOTSUP;
	}

	return api->set_callback(dev, callback, user_data);
}
#endif

/* -------- Host visible UART -------- */
struct mec5_host_uart_driver_api {
	int (*host_access_enable)(const struct device *dev, uint8_t enable, uint32_t cfg);
	int (*intr_enable)(const struct device *dev, uint8_t enable, uint32_t flags);
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_ESPI_ESPI_MCHP_MEC5_H_ */
