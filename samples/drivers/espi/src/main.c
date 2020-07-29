/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr.h>
#include <device.h>
#include <soc.h>
#include <drivers/gpio.h>
#include <drivers/espi.h>
#include <drivers/espi_saf.h>
#include <logging/log_ctrl.h>
#include <logging/log.h>
LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

/* eSPI host entity address  */
#define DEST_SLV_ADDR         0x02u
#define SRC_SLV_ADDR          0x21u

/* Temperature command opcode */
#define OOB_CMDCODE           0x01u
#define OOB_RESPONSE_LEN      0x05u
#define OOB_RESPONSE_INDEX    0x03u

/* Maximum bytes for OOB transactions */
#define MAX_RESP_SIZE         20u

/* eSPI flash parameters */
#define MAX_TEST_BUF_SIZE     1024u
#define MAX_FLASH_REQUEST     64u
#define TARGET_FLASH_REGION   0x72000ul

/* 20 MHz */
#define MIN_ESPI_FREQ         20u

#define K_WAIT_DELAY          100u

/* eSPI event */
#define EVENT_MASK            0x0000FFFFu
#define EVENT_DETAILS_MASK    0xFFFF0000u
#define EVENT_DETAILS_POS     16u
#define EVENT_TYPE(x)         (x & EVENT_MASK)
#define EVENT_DETAILS(x)      ((x & EVENT_DETAILS_MASK) >> EVENT_DETAILS_POS)

/* SAF local flash frequency */
#define ESPI_SAF_FLASH_FREQ_48M 1
#define ESPI_SAF_FLASH_FREQ_24M 2
#define ESPI_SAF_FLASH_FREQ_16M 3

struct oob_header {
	uint8_t dest_slave_addr;
	uint8_t oob_cmd_code;
	uint8_t byte_cnt;
	uint8_t src_slave_addr;
};

#ifdef CONFIG_ESPI_GPIO_DEV_NEEDED
static struct device *gpio_dev0;
static struct device *gpio_dev1;
#define PWR_SEQ_TIMEOUT    3000u
#endif

static struct device *espi_dev;
static struct espi_callback espi_bus_cb;
static struct espi_callback vw_rdy_cb;
static struct espi_callback vw_cb;
static struct espi_callback p80_cb;

static uint8_t espi_rst_sts;

#ifdef CONFIG_ESPI_FLASH_CHANNEL
static uint8_t flash_write_buf[MAX_TEST_BUF_SIZE];
static uint8_t flash_read_buf[MAX_TEST_BUF_SIZE];
#endif

#ifdef CONFIG_ESPI_SAF
enum saf_erase_size {
	SAF_ERASE_4K = 0,
	SAF_ERASE_32K = 1,
	SAF_ERASE_64K = 2,
	SAF_ERASE_MAX
};

#define SAF_TEST_BUF_SIZE 4096U
struct saf_addr_info {
	uintptr_t saf_struct_addr;
	uintptr_t saf_exp_addr;
};
static struct device *espi_saf_dev;
static uint32_t safbuf[SAF_TEST_BUF_SIZE/4U];
static uint32_t safbuf2[SAF_TEST_BUF_SIZE/4U];

static struct saf_addr_info saf_addr_check[] = {
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_ECP_CMD, 		0x40008018 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_ECP_FLAR, 		0x4000801C },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_ECP_START, 		0x40008020 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_ECP_BFAR, 		0x40008024 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_ECP_STATUS, 		0x40008028 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_ECP_INTEN, 		0x4000802C },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_FL_CFG_SIZE_LIM, 	0x40008030 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_FL_CFG_THRH, 		0x40008034 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_FL_CFG_MISC, 		0x40008038 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_ESPI_MON_STATUS, 	0x4000803C },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_ESPI_MON_INTEN, 	0x40008040 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_ECP_BUSY, 		0x40008044 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_CS_OP[0], 		0x4000804C },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_CS_OP[MCHP_ESPI_SAF_CS_MAX-1], 0x4000805C },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_CS_OP[MCHP_ESPI_SAF_CS_MAX-1].OP_DESCR, 0x40008068 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_FL_CFG_GEN_DESCR, 	0x4000806C },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_PROT_LOCK, 		0x40008070 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_PROT_DIRTY, 		0x40008074 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_TAG_MAP[0], 		0x40008078 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_TAG_MAP[MCHP_ESPI_SAF_TAGMAP_MAX-1], 0x40008080 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_PROT_RG[0], 		0x40008084 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_PROT_RG[MCHP_ESPI_SAF_PR_MAX-1], 0x40008184 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_PROT_RG[MCHP_ESPI_SAF_PR_MAX-1].RDBM, 0x40008190 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_POLL_TMOUT, 		0x40008194 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_POLL_INTRVL, 		0x40008198 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_SUS_RSM_INTRVL,	0x4000819C },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_CONSEC_RD_TMOUT, 	0x400081A0 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_CS0_CFG_P2M, 		0x400081A4 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_CS1_CFG_P2M, 		0x400081A6 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_FL_CFG_SPM, 		0x400081A8 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_SUS_CHK_DLY, 		0x400081AC },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_CS0_CM_PRF, 		0x400081B0 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_CS1_CM_PRF, 		0x400081B2 },
	{ (uintptr_t)&MCHP_SAF_REGS->SAF_DNX_PROT_BYP, 		0x400081B4 },
};
#define DEBUG_NUM_SAF_REG_ADDR (sizeof(saf_addr_check) / sizeof(struct saf_addr_info))
#endif

static void host_warn_handler(uint32_t signal, uint32_t status)
{
	switch (signal) {
	case ESPI_VWIRE_SIGNAL_HOST_RST_WARN:
		LOG_INF("Host reset warning %d", status);
		if (!IS_ENABLED(CONFIG_ESPI_AUTOMATIC_WARNING_ACKNOWLEDGE)) {
			LOG_INF("HOST RST ACK %d", status);
			espi_send_vwire(espi_dev,
					ESPI_VWIRE_SIGNAL_HOST_RST_ACK,
					status);
		}
		break;
	case ESPI_VWIRE_SIGNAL_SUS_WARN:
		LOG_INF("Host suspend warning %d", status);
		if (!IS_ENABLED(CONFIG_ESPI_AUTOMATIC_WARNING_ACKNOWLEDGE)) {
			LOG_INF("SUS ACK %d", status);
			espi_send_vwire(espi_dev, ESPI_VWIRE_SIGNAL_SUS_ACK,
					status);
		}
		break;
	default:
		break;
	}
}

/* eSPI bus event handler */
static void espi_reset_handler(struct device *dev,
			       struct espi_callback *cb,
			       struct espi_event event)
{
	if (event.evt_type == ESPI_BUS_RESET) {
		espi_rst_sts = event.evt_data;
		LOG_INF("eSPI BUS reset %d", event.evt_data);
	}
}

/* eSPI logical channels enable/disable event handler */
static void espi_ch_handler(struct device *dev, struct espi_callback *cb,
			    struct espi_event event)
{
	if (event.evt_type == ESPI_BUS_EVENT_CHANNEL_READY) {
		switch (event.evt_details) {
		case ESPI_CHANNEL_VWIRE:
			LOG_INF("VW channel event %x", event.evt_data);
			break;
		case ESPI_CHANNEL_FLASH:
			LOG_INF("Flash channel event %d", event.evt_data);
			break;
		case ESPI_CHANNEL_OOB:
			LOG_INF("OOB channel event %d", event.evt_data);
			break;
		default:
			LOG_ERR("Unknown channel event");
		}
	}
}

/* eSPI vwire received event handler */
static void vwire_handler(struct device *dev, struct espi_callback *cb,
			  struct espi_event event)
{
	if (event.evt_type == ESPI_BUS_EVENT_VWIRE_RECEIVED) {
		switch (event.evt_details) {
		case ESPI_VWIRE_SIGNAL_PLTRST:
			LOG_INF("PLT_RST changed %d", event.evt_data);
			break;
		case ESPI_VWIRE_SIGNAL_SLP_S3:
		case ESPI_VWIRE_SIGNAL_SLP_S4:
		case ESPI_VWIRE_SIGNAL_SLP_S5:
			LOG_INF("SLP signal changed %d", event.evt_data);
			break;
		case ESPI_VWIRE_SIGNAL_SUS_WARN:
		case ESPI_VWIRE_SIGNAL_HOST_RST_WARN:
			host_warn_handler(event.evt_details,
					      event.evt_data);
			break;
		}
	}
}

/* eSPI peripheral channel notifications handler */
static void periph_handler(struct device *dev, struct espi_callback *cb,
			   struct espi_event event)
{
	uint8_t periph_type;
	uint8_t periph_index;

	periph_type = EVENT_TYPE(event.evt_details);
	periph_index = EVENT_DETAILS(event.evt_details);

	switch (periph_type) {
	case ESPI_PERIPHERAL_DEBUG_PORT80:
		LOG_INF("Postcode %x", event.evt_data);
		break;
	case ESPI_PERIPHERAL_HOST_IO:
		LOG_INF("ACPI %x", event.evt_data);
		espi_remove_callback(espi_dev, &p80_cb);
		break;
	default:
		LOG_INF("%s periph 0x%x [%x]", __func__, periph_type,
			event.evt_data);
	}
}

int espi_init(void)
{
	int ret;
	/* Indicate to eSPI master simplest configuration: Single line,
	 * 20MHz frequency and only logical channel 0 and 1 are supported
	 */
	struct espi_cfg cfg = {
		ESPI_IO_MODE_SINGLE_LINE,
		ESPI_CHANNEL_VWIRE | ESPI_CHANNEL_PERIPHERAL,
		MIN_ESPI_FREQ,
	};

	/* If eSPI driver supports additional capabilities use them */
#ifdef CONFIG_ESPI_OOB_CHANNEL
	cfg.channel_caps |= ESPI_CHANNEL_OOB;
#endif
#ifdef CONFIG_ESPI_FLASH_CHANNEL
	cfg.channel_caps |= ESPI_CHANNEL_FLASH;
#endif

	ret = espi_config(espi_dev, &cfg);
	if (ret) {
		LOG_ERR("Failed to configure eSPI slave channels:%x err: %d",
			cfg.channel_caps, ret);
		return ret;
	} else {
		LOG_INF("eSPI slave configured successfully!");
	}

	LOG_INF("eSPI test - callbacks initialization... ");
	espi_init_callback(&espi_bus_cb, espi_reset_handler, ESPI_BUS_RESET);
	espi_init_callback(&vw_rdy_cb, espi_ch_handler,
			   ESPI_BUS_EVENT_CHANNEL_READY);
	espi_init_callback(&vw_cb, vwire_handler,
			   ESPI_BUS_EVENT_VWIRE_RECEIVED);
	espi_init_callback(&p80_cb, periph_handler,
			   ESPI_BUS_PERIPHERAL_NOTIFICATION);
	LOG_INF("complete");

	LOG_INF("eSPI test - callbacks registration... ");
	espi_add_callback(espi_dev, &espi_bus_cb);
	espi_add_callback(espi_dev, &vw_rdy_cb);
	espi_add_callback(espi_dev, &vw_cb);
	espi_add_callback(espi_dev, &p80_cb);
	LOG_INF("complete");

	return ret;
}

#ifdef CONFIG_ESPI_SAF

int espi_saf_init(void)
{
	int ret;
	struct espi_saf_cfg cfg;

	cfg.spi_freq = ESPI_SAF_FLASH_FREQ_24M;

	ret = espi_saf_config(espi_saf_dev, &cfg);
	if (ret) {
		LOG_ERR("Failed to configure eSPI SAF SPI freq:%x err: %d",
			cfg.spi_freq, ret);
		return ret;
	} else {
		LOG_INF("eSPI SAF configured successfully!");
	}

#if 0
	LOG_INF("eSPI test - callbacks initialization... ");
	espi_init_callback(&espi_bus_cb, espi_reset_handler, ESPI_BUS_RESET);
	espi_init_callback(&vw_rdy_cb, espi_ch_handler,
			   ESPI_BUS_EVENT_CHANNEL_READY);
	espi_init_callback(&vw_cb, vwire_handler,
			   ESPI_BUS_EVENT_VWIRE_RECEIVED);
	espi_init_callback(&p80_cb, periph_handler,
			   ESPI_BUS_PERIPHERAL_NOTIFICATION);
	LOG_INF("complete");

	LOG_INF("eSPI test - callbacks registration... ");
	espi_add_callback(espi_dev, &espi_bus_cb);
	espi_add_callback(espi_dev, &vw_rdy_cb);
	espi_add_callback(espi_dev, &vw_cb);
	espi_add_callback(espi_dev, &p80_cb);
	LOG_INF("complete");
#endif

	return ret;
}

/*
 * SAF hardware limited to 1 to 64 byte read requests.
 */
static int saf_read(uint32_t spi_addr, uint8_t *dest, int len)
{
	int rc, chunk_len, n;
	struct espi_saf_packet saf_pkt = { 0 };

	if ((dest == NULL) || (len < 0)) {
		return -EINVAL;
	}

	saf_pkt.flash_addr = spi_addr;
	saf_pkt.buf = dest;

	n = len;
	while (n) {
		chunk_len = 64;
		if (n < 64) {
			chunk_len = n;
		}

		saf_pkt.len = chunk_len;

		rc = espi_saf_flash_read(espi_saf_dev, &saf_pkt);
		if (rc != 0) {
			LOG_INF("saf_read: error = %d: chunk_len=%d spi_addr=%x",
				rc, chunk_len, spi_addr);
			return rc;
		}

		saf_pkt.flash_addr += chunk_len;
		saf_pkt.buf += chunk_len;
		n -= chunk_len;
	}

	return len;
}

/*
 * SAF hardware limited to 4KB(mandatory), 32KB, and 64KB erase sizes.
 * eSPI configuration has flags the Host can read specifying supported
 * erase sizes.
 */
static int saf_erase_block(uint32_t spi_addr, enum saf_erase_size ersz)
{
	int rc;
	struct espi_saf_packet saf_pkt = { 0 };

	switch (ersz) {
	case SAF_ERASE_4K:
		saf_pkt.len = 4096U;
		spi_addr &= ~(4096U);
		break;
	case SAF_ERASE_32K:
		saf_pkt.len = (32U * 1024U);
		spi_addr &= ~(32U * 1024U);
		break;
	case SAF_ERASE_64K:
		saf_pkt.len = (64U * 1024U);
		spi_addr &= ~(64U * 1024U);
		break;
	default:
		return -EINVAL;
	}

	saf_pkt.flash_addr = spi_addr;

	rc = espi_saf_flash_erase(espi_saf_dev, &saf_pkt);
	if (rc != 0) {
		LOG_INF("espi_saf_test1: erase fail = %d", rc);
		return rc;
	}

	return 0;
}

/*
 * SAF hardware limited to 1 to 64 byte programming within a 256 byte page.
 */
static int saf_page_prog(uint32_t spi_addr, const uint8_t *src, int progsz)
{
	int rc, chunk_len, n;
	struct espi_saf_packet saf_pkt = { 0 };

	if ((src == NULL) || (progsz < 0) || (progsz > 256)) {
		return -EINVAL;
	}

	if (progsz == 0) {
		return 0;
	}

	saf_pkt.flash_addr = spi_addr;
	saf_pkt.buf = (uint8_t *)src;

	n = progsz;
	while (n) {
		chunk_len = 64;
		if (n < 64) {
			chunk_len = n;
		}

		saf_pkt.len = (uint32_t)chunk_len;

		rc = espi_saf_flash_write(espi_saf_dev, &saf_pkt);
		if (rc != 0) {
			LOG_INF("saf_page_prog: error=%d: erase fail spi_addr=0x%X",
				rc, spi_addr);
			return rc;
		}

		saf_pkt.flash_addr += chunk_len;
		saf_pkt.buf += chunk_len;
		n -= chunk_len;
	}

	return progsz;
}


int espi_saf_test1(void)
{
	int rc;
	uint32_t n, spi_addr, progsz, chunksz;

	/*
	 * Activating SAF triggers the SAF engine into sending enter
	 * continuous mode sequences to both chip selects.
	 * Continuous mode entry sequence is 24 SPI clocks plus
	 * chip select de-assertion time. At 24MHz SPI clock each
	 * command sequence is about 1.12 us. The engine also has
	 * a delay between each chip select. The scope shows about
	 * 2.82 us overall.
	 * Application must delay about 3 us after SAF activate
	 * before it calls any API for SAF flash access.
	 */
	rc = espi_saf_activate(espi_saf_dev);
	k_busy_wait(3);
	LOG_INF("espi_saf_test1: activate = %d", rc);

	memset(safbuf, 0x55, sizeof(safbuf));
	memset(safbuf2, 0, sizeof(safbuf2));

	while (true) {
		/* read 4KB sector at 0 */
		spi_addr = 0U;
		rc = saf_read(spi_addr, (uint8_t *)safbuf, 4096);
		if (rc != 4096) {
			LOG_INF("espi_saf_test1: error=%d Read 4K sector at 0x%X failed", rc, spi_addr);
			return rc;
		}

		rc = 0;
		for (n = 0; n < 4096U/4U; n++) {
			if (safbuf[n] != 0xffffffffUL) {
				rc = -1;
				break;
			}
		}

		if (rc == 0) {
			LOG_INF("4KB sector at 0x%x is in erased state. Continue tests", spi_addr);
			break;
		} else {
			LOG_INF("4KB sector at 0x%x not in erased state. Send 4K erase.", spi_addr);
			rc = saf_erase_block(spi_addr, SAF_ERASE_4K);
			if (rc != 0) {
				LOG_INF("SAF erase block at 0x%x returned error %d", spi_addr, rc);
				return rc;
			}
		}
	}

	/*
	 * Program 4KB page using SAF Write limited by SAF protocol to
	 * 64 bytes chunks.
	 */
	for (n = 0; n < 4096U/4U; n++) {
		uint32_t m = (n * 4U) % 256U;
		uint8_t *p8 = (uint8_t *)&safbuf[n];
		*p8++ = (uint8_t)m;
		*p8++ = (uint8_t)(m + 1);
		*p8++ = (uint8_t)(m + 2);
		*p8++ = (uint8_t)(m + 3);
	}


	progsz = 4096U;
	chunksz = 256U;
	spi_addr = 0U;
	n = 0;
	const uint8_t *src = (const uint8_t *)safbuf;

	LOG_INF("espi_saf_test1: Program 4KB sector at 0x%X", spi_addr);

	while (n < progsz) {
		rc = saf_page_prog(spi_addr, (const uint8_t *)src, (int)chunksz);
		if (rc != chunksz) {
			LOG_INF("saf_page_prog error=%d at 0x%X", rc, spi_addr);
			break;
		}
		spi_addr += chunksz;
		n += chunksz;
		src += chunksz;
	}

	/* read back and check */
	spi_addr = 0U;
	LOG_INF("espi_saf_test1: Read back 4K sector at 0x%X", spi_addr);

	rc = saf_read(spi_addr, (uint8_t *)safbuf2, progsz);
	if (rc == progsz) {
		rc = memcmp(safbuf, safbuf2, progsz);
		if (rc == 0) {
			LOG_INF("espi_saf_test1: Read back match: PASS");
		} else {
			LOG_INF("espi_saf_test1: Read back mismatch: FAIL");
		}
	} else {
		LOG_INF("espi_saf_test1: Read back 4K error=%d", rc);
		return rc;
	}

	return rc;
}
#endif

static int wait_for_pin(struct device *dev, uint8_t pin, uint16_t timeout,
			int exp_level)
{
	uint16_t loop_cnt = timeout;
	int level;

	do {
		level = gpio_pin_get(dev, pin);
		if (level < 0) {
			LOG_ERR("Failed to read %x %d", pin, level);
			return -EIO;
		}

		if (exp_level == level) {
			LOG_DBG("PIN %x = %x", pin, exp_level);
			break;
		}

		k_usleep(K_WAIT_DELAY);
		loop_cnt--;
	} while (loop_cnt > 0);

	if (loop_cnt == 0) {
		LOG_ERR("Timeout for %x %x", pin, level);
		return -ETIMEDOUT;
	}

	return 0;
}

static int wait_for_vwire(struct device *espi_dev,
			  enum espi_vwire_signal signal,
			  uint16_t timeout, uint8_t exp_level)
{
	int ret;
	uint8_t level;
	uint16_t loop_cnt = timeout;

	do {
		ret = espi_receive_vwire(espi_dev, signal, &level);
		if (ret) {
			LOG_ERR("Failed to read %x %d", signal, ret);
			return -EIO;
		}

		if (exp_level == level) {
			break;
		}

		k_usleep(K_WAIT_DELAY);
		loop_cnt--;
	} while (loop_cnt > 0);

	if (loop_cnt == 0) {
		LOG_ERR("VWIRE %d is %x", signal, level);
		return -ETIMEDOUT;
	}

	return 0;
}

static int wait_for_espi_reset(uint8_t exp_sts)
{
	uint16_t loop_cnt = CONFIG_ESPI_VIRTUAL_WIRE_TIMEOUT;

	do {
		if (exp_sts == espi_rst_sts) {
			break;
		}
		k_usleep(K_WAIT_DELAY);
		loop_cnt--;
	} while (loop_cnt > 0);

	if (loop_cnt == 0) {
		return -ETIMEDOUT;
	}

	return 0;
}

int espi_handshake(void)
{
	int ret;

	LOG_INF("eSPI test - Handshake with eSPI master...");
	ret = wait_for_vwire(espi_dev, ESPI_VWIRE_SIGNAL_SUS_WARN,
			     CONFIG_ESPI_VIRTUAL_WIRE_TIMEOUT, 1);
	if (ret) {
		LOG_ERR("SUS_WARN Timeout");
		return ret;
	}

	LOG_INF("1st phase completed");
	ret = wait_for_vwire(espi_dev, ESPI_VWIRE_SIGNAL_SLP_S5,
			     CONFIG_ESPI_VIRTUAL_WIRE_TIMEOUT, 1);
	if (ret) {
		LOG_ERR("SLP_S5 Timeout");
		return ret;
	}

	ret = wait_for_vwire(espi_dev, ESPI_VWIRE_SIGNAL_SLP_S4,
			     CONFIG_ESPI_VIRTUAL_WIRE_TIMEOUT, 1);
	if (ret) {
		LOG_ERR("SLP_S4 Timeout");
		return ret;
	}

	ret = wait_for_vwire(espi_dev, ESPI_VWIRE_SIGNAL_SLP_S3,
			     CONFIG_ESPI_VIRTUAL_WIRE_TIMEOUT, 1);
	if (ret) {
		LOG_ERR("SLP_S3 Timeout");
		return ret;
	}

	LOG_INF("2nd phase completed");

	ret = wait_for_vwire(espi_dev, ESPI_VWIRE_SIGNAL_PLTRST,
			     CONFIG_ESPI_VIRTUAL_WIRE_TIMEOUT, 1);
	if (ret) {
		LOG_ERR("PLT_RST Timeout");
		return ret;
	}

	LOG_INF("3rd phase completed");

	return 0;
}

#ifdef CONFIG_ESPI_FLASH_CHANNEL
int read_test_block(uint8_t *buf, uint32_t start_flash_adr, uint16_t block_len)
{
	uint8_t i = 0;
	uint32_t flash_addr = start_flash_adr;
	uint16_t transactions = block_len/MAX_FLASH_REQUEST;
	int ret = 0;
	struct espi_flash_packet pckt;

	for (i = 0; i < transactions; i++) {
		pckt.buf = buf;
		pckt.flash_addr = flash_addr;
		pckt.len = MAX_FLASH_REQUEST;

		ret = espi_read_flash(espi_dev, &pckt);
		if (ret) {
			LOG_ERR("espi_read_flash failed: %d", ret);
			return ret;
		}

		buf += MAX_FLASH_REQUEST;
		flash_addr += MAX_FLASH_REQUEST;
	}

	LOG_INF("%d read flash transactions completed", transactions);
	return 0;
}

int write_test_block(uint8_t *buf, uint32_t start_flash_adr, uint16_t block_len)
{
	uint8_t i = 0;
	uint32_t flash_addr = start_flash_adr;
	uint16_t transactions = block_len/MAX_FLASH_REQUEST;
	int ret = 0;
	struct espi_flash_packet pckt;

	/* Split operation in multiple MAX_FLASH_REQ transactions */
	for (i = 0; i < transactions; i++) {
		pckt.buf = buf;
		pckt.flash_addr = flash_addr;
		pckt.len = MAX_FLASH_REQUEST;

		ret = espi_write_flash(espi_dev, &pckt);
		if (ret) {
			LOG_ERR("espi_write_flash failed: %d", ret);
			return ret;
		}

		buf += MAX_FLASH_REQUEST;
		flash_addr += MAX_FLASH_REQUEST;
	}

	LOG_INF("%d write flash transactions completed", transactions);
	return 0;
}

static int espi_flash_test(uint32_t start_flash_addr, uint8_t blocks)
{
	uint8_t i;
	uint8_t pattern;
	uint32_t flash_addr;
	int ret = 0;

	LOG_INF("Test eSPI write flash");
	flash_addr = start_flash_addr;
	pattern = 0x99;
	for (i = 0; i <= blocks; i++) {
		memset(flash_write_buf, pattern++, sizeof(flash_write_buf));
		ret = write_test_block(flash_write_buf, flash_addr,
				       sizeof(flash_write_buf));
		if (ret) {
			LOG_ERR("Failed to write to eSPI");
			return ret;
		}

		flash_addr += sizeof(flash_write_buf);
	}

	LOG_INF("Test eSPI read flash");
	flash_addr = start_flash_addr;
	pattern = 0x99;
	for (i = 0; i <= blocks; i++) {
		/* Set expected content */
		memset(flash_write_buf, pattern, sizeof(flash_write_buf));
		/* Clear last read content */
		memset(flash_read_buf, 0, sizeof(flash_read_buf));
		ret = read_test_block(flash_read_buf, flash_addr,
				      sizeof(flash_read_buf));
		if (ret) {
			LOG_ERR("Failed to read from eSPI");
			return ret;
		}

		/* Compare buffers  */
		int cmp = memcmp(flash_write_buf, flash_read_buf,
				 sizeof(flash_write_buf));

		if (cmp != 0) {
			LOG_ERR("eSPI read mismmatch at %d expected %x",
				cmp, pattern);
		}

		flash_addr += sizeof(flash_read_buf);
		pattern++;
	}

	return 0;
}
#endif /* CONFIG_ESPI_FLASH_CHANNEL */

int get_pch_temp(struct device *dev, int *temp)
{
	struct espi_oob_packet req_pckt;
	struct espi_oob_packet resp_pckt;
	struct oob_header oob_hdr;
	uint8_t buf[MAX_RESP_SIZE];
	int ret;

	LOG_INF("%s", __func__);

	oob_hdr.dest_slave_addr = DEST_SLV_ADDR;
	oob_hdr.oob_cmd_code = OOB_CMDCODE;
	oob_hdr.byte_cnt = 1;
	oob_hdr.src_slave_addr = SRC_SLV_ADDR;

	/* Packetize OOB request */
	req_pckt.buf = (uint8_t *)&oob_hdr;
	req_pckt.len = sizeof(struct oob_header);
	resp_pckt.buf = (uint8_t *)&buf;
	resp_pckt.len = MAX_RESP_SIZE;

	ret = espi_send_oob(dev, &req_pckt);
	if (ret) {
		LOG_ERR("OOB Tx failed %d", ret);
		return ret;
	}

	ret = espi_receive_oob(dev, &resp_pckt);
	if (ret) {
		LOG_ERR("OOB Rx failed %d", ret);
		return ret;
	}

	LOG_INF("OOB transaction completed rcvd: %d bytes", resp_pckt.len);
	for (int i = 0; i < resp_pckt.len; i++) {
		LOG_INF("%x ", buf[i]);
	}

	if (resp_pckt.len == OOB_RESPONSE_LEN) {
		*temp = buf[OOB_RESPONSE_INDEX];
	} else {
		LOG_ERR("Incorrect size response");
	}

	return 0;
}

int espi_test(void)
{
	int ret;

	/* Account for the time serial port is detected so log messages can
	 * be seen
	 */
	k_sleep(K_SECONDS(1));

#ifdef CONFIG_ESPI_GPIO_DEV_NEEDED
	gpio_dev0 = device_get_binding(CONFIG_ESPI_GPIO_DEV0);
	if (!gpio_dev0) {
		LOG_WRN("Fail to find: %s", CONFIG_ESPI_GPIO_DEV0);
		return -1;
	}

	gpio_dev1 = device_get_binding(CONFIG_ESPI_GPIO_DEV1);
	if (!gpio_dev1) {
		LOG_WRN("Fail to find: %s", CONFIG_ESPI_GPIO_DEV1);
		return -1;
	}

#endif
	espi_dev = device_get_binding(CONFIG_ESPI_DEV);
	if (!espi_dev) {
		LOG_WRN("Fail to find %s", CONFIG_ESPI_DEV);
		return -1;
	}

#ifdef CONFIG_ESPI_SAF
	espi_saf_dev = device_get_binding(CONFIG_ESPI_SAF_DEV);
	if (!espi_saf_dev) {
		LOG_WRN("Fail to find %s", CONFIG_ESPI_SAF_DEV);
		return -1;
	}
#endif

	LOG_INF("Hello eSPI test %s", CONFIG_BOARD);

#ifdef CONFIG_ESPI_GPIO_DEV_NEEDED
	ret = gpio_pin_configure(gpio_dev0, CONFIG_PWRGD_PIN,
				 GPIO_INPUT | GPIO_ACTIVE_HIGH);
	if (ret) {
		LOG_ERR("Unable to configure %d:%d", CONFIG_PWRGD_PIN, ret);
		return ret;
	}

	ret = gpio_pin_configure(gpio_dev1, CONFIG_ESPI_INIT_PIN,
				 GPIO_OUTPUT | GPIO_ACTIVE_HIGH);
	if (ret) {
		LOG_ERR("Unable to config %d: %d", CONFIG_ESPI_INIT_PIN, ret);
		return ret;
	}

	ret = gpio_pin_set(gpio_dev1, CONFIG_ESPI_INIT_PIN, 0);
	if (ret) {
		LOG_ERR("Unable to initialize %d", CONFIG_ESPI_INIT_PIN);
		return -1;
	}
#endif

	espi_init();

#ifdef CONFIG_ESPI_SAF
	espi_saf_init();

	espi_saf_test1();
#endif

#ifdef CONFIG_ESPI_GPIO_DEV_NEEDED
	ret = wait_for_pin(gpio_dev0, CONFIG_PWRGD_PIN, PWR_SEQ_TIMEOUT, 1);
	if (ret) {
		LOG_ERR("RSMRST_PWRGD timeout");
		return ret;
	}

	ret = gpio_pin_set(gpio_dev1, CONFIG_ESPI_INIT_PIN, 1);
	if (ret) {
		LOG_ERR("Failed to write %x %d", CONFIG_ESPI_INIT_PIN, ret);
		return ret;
	}
#endif
	ret = wait_for_espi_reset(1);
	if (ret) {
		LOG_INF("ESPI_RESET timeout");
		return ret;
	}

#ifdef CONFIG_ESPI_FLASH_CHANNEL
	/* Flash operation need to be perform before VW handshake or
	 * after eSPI host completes full initialization.
	 * This sample code can't assume a full initialized eSPI host
	 * so flash operations are perform here.
	 */
	bool flash_sts;

	do {
		flash_sts = espi_get_channel_status(espi_dev,
						    ESPI_CHANNEL_FLASH);
		k_busy_wait(100);
	} while (!flash_sts);

	/* eSPI flash test can fail and rest of operation can continue */
	ret = espi_flash_test(TARGET_FLASH_REGION, 1);
	if (ret) {
		LOG_INF("eSPI flash test failed %d", ret);
	}
#endif

	/* Showcase VW channel by exchanging virtual wires with eSPI host */
	ret = espi_handshake();
	if (ret) {
		LOG_ERR("eSPI VW handshake failed %d", ret);
		return ret;
	}

	/*  Attempt to use OOB channel to read temperature, regardless of
	 * if is enabled or not.
	 */
	for (int i = 0; i < 5; i++) {
		int temp;

		ret = get_pch_temp(espi_dev, &temp);
		if (ret)  {
			LOG_ERR("eSPI OOB transaction failed %d", ret);
		} else {
			LOG_INF("Temp: %d ", temp);
		}
	}

	/* Cleanup */
	k_sleep(K_SECONDS(1));
	espi_remove_callback(espi_dev, &espi_bus_cb);
	espi_remove_callback(espi_dev, &vw_rdy_cb);
	espi_remove_callback(espi_dev, &vw_cb);

	LOG_INF("eSPI sample completed err: %d", ret);

	return ret;
}

#ifdef CONFIG_ESPI_SAF
bool check_espi_saf_struct(void)
{
	size_t n;
	bool pass = true;
	struct saf_addr_info *p = &saf_addr_check[0];

	for (n = 0; n < DEBUG_NUM_SAF_REG_ADDR; n++) {
		if (p->saf_struct_addr != p->saf_exp_addr) {
			pass = false;
			printk("SAF register struct check index %d failed!\n", n);
		}
		p++;
	}

	return pass;
}
#endif

void main(void)
{
#ifdef CONFIG_ESPI_SAF
	if (check_espi_saf_struct()) {
		LOG_INF("eSPI SAF reg check PASS");
	} else {
		LOG_INF("eSPI SAF reg check FAIL");
	}
#endif

	espi_test();
}
