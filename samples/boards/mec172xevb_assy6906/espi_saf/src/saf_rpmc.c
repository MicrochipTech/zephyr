/*
 * Copyright (c) 2022 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_saf.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(espi, CONFIG_ESPI_LOG_LEVEL);

#include "saf_rpmc.h"
#include "app_hmac.h"
#include "utils.h"

#define ESPI_SAF_RPMC_TEST_INCR_COUNTER

#define RPMC_MAX_COUNTERS	4

struct rpmc_key_info {
	uint32_t hmac_key_data;
	uint8_t root_key[32];
	uint8_t hmac_key[32];
	uint8_t tag[12];
};

static uint8_t saf_rpmc_buf[64] __aligned(4);
static uint8_t saf_rpmc_buf2[64] __aligned(4);

/* RPMC HMAC Key Data is a 4 byte value used to derive a 32 byte
 * HMAC-SHA-256 key.
 */
static const uint32_t rpmc_key_data[RPMC_MAX_COUNTERS] = {
	0xbc1f19a1U, 0x9e7c7aa5U, 0x58f8b611U, 0x39b51dc8U
};

static struct rpmc_key_info rpmc_keys[RPMC_MAX_COUNTERS];

/* W25R128 SPI flash SAF configuration.
 * Size is 16Mbytes, it requires no continuous mode prefix, or
 * other special SAF configuration.
 * IMPORTANT: Winbond RPMC SPI flash does not support flash suspend
 * during OP1 operations. We must specify MCHP_FLASH_RPMC_NO_OP1_SUSPEND.
 */
#define MCHP_W25R128_RPMC_OP1_FLAGS (MCHP_FLASH_RPMC_MIRROR_048 | \
	MCHP_FLASH_RPMC_MIRROR_848 | MCHP_FLASH_RPMC_MIRROR_NCTNR | \
	MCHP_FLASH_RPMC_NO_OP1_SUSPEND)

static const struct espi_saf_flash_cfg flash_w25r128 = {
	.version = MCHP_SAF_VER_2,
	.rd_freq_mhz = 0, /* use QMSPI driver frequency for reads */
	.freq_mhz = 0, /* use QMSPI driver frequency for all other commands */
	.flashsz = 0x1000000U,
	.opa = MCHP_SAF_OPCODE_REG_VAL(0x06U, 0x75U, 0x7aU, 0x05U),
	.opb = MCHP_SAF_OPCODE_REG_VAL(0x20U, 0x52U, 0xd8U, 0x02U),
#if DT_PROP(DT_NODELABEL(spi0), lines) == 2
	.opc = MCHP_SAF_OPCODE_REG_VAL(0xbbu, 0xffu, 0xa5u, 0x35u),
#else
	.opc = MCHP_SAF_OPCODE_REG_VAL(0xebU, 0xffU, 0xa5U, 0x35U),
#endif
	.opd = MCHP_SAF_OPCODE_REG_VAL(0xb9u, 0xabU, MCHP_FLASH_RPMC_OP2_DFLT, 0U),
	/* RPMC OP1, number of counters, host visibility flags, unused */
	.rpmc_op1 = MCHP_SAF_OPCODE_REG_VAL(MCHP_FLASH_RPMC_OP1_DFLT, 4U,
					    MCHP_W25R128_RPMC_OP1_FLAGS, 0U),
	.flags = 0U,
	.poll2_mask = MCHP_W25Q128_POLL2_MASK,
	.cont_prefix = 0U,
#if DT_PROP(DT_NODELABEL(spi0), lines) == 2
	.cs_cfg_descr_ids = MCHP_CS0_CFG_DESCR_IDX_REG_VAL_DUAL,
	.descr = {
		MCHP_W25Q128_CM_RD_DUAL_D0,
		MCHP_W25Q128_CM_RD_DUAL_D1,
		MCHP_W25Q128_CM_RD_DUAL_D2,
		MCHP_W25Q128_ENTER_CM_DUAL_D0,
		MCHP_W25Q128_ENTER_CM_DUAL_D1,
		MCHP_W25Q128_ENTER_CM_DUAL_D2,
	},
#else
	.cs_cfg_descr_ids = MCHP_CS0_CFG_DESCR_IDX_REG_VAL,
	.descr = {
		MCHP_W25Q128_CM_RD_D0,
		MCHP_W25Q128_CM_RD_D1,
		MCHP_W25Q128_CM_RD_D2,
		MCHP_W25Q128_ENTER_CM_D0,
		MCHP_W25Q128_ENTER_CM_D1,
		MCHP_W25Q128_ENTER_CM_D2
	}
#endif
};

static const struct espi_saf_cfg saf_cfg_rpmc_w25r128 = {
	.nflash_devices = 1U,
	.hwcfg = {
		.version = MCHP_SAF_VER_2,
		.flags = 0U,
		/* all OP1 commands reported to the host as successful */
		.misc_flags = BIT(MCHP_SAF_HW_CFG_MFLAG_RPMC_OP1_FRSC_POS),
		.qmspi_cpha = 0U,
		.qmspi_cs_timing = 0U,
		.flash_pd_timeout = 0U,
		.flash_pd_min_interval = 0U,
		.generic_descr = {
#if DT_PROP(DT_NODELABEL(spi0), lines) == 2
			MCHP_SAF_EXIT_CM_DUAL_DESCR12, MCHP_SAF_EXIT_CMD_DUAL_DESCR13,
#else
			MCHP_SAF_EXIT_CM_DESCR12, MCHP_SAF_EXIT_CM_DESCR13,
#endif
			MCHP_SAF_POLL_DESCR14, MCHP_SAF_POLL_DESCR15
		},
		.tag_map = { 0U, 0U, 0U },
	},
	.flash_cfgs = (struct espi_saf_flash_cfg *)&flash_w25r128
};

static const struct espi_saf_pr w25r128_protect_regions[2] = {
	{
		.start = 0xe00000U,
		.size =  0x100000U,
		.master_bm_we = (1U << MCHP_SAF_MSTR_HOST_PCH_ME),
		.master_bm_rd = (1U << MCHP_SAF_MSTR_HOST_PCH_ME),
		.pr_num = 1U,
		.flags = MCHP_SAF_PR_FLAG_ENABLE
			 | MCHP_SAF_PR_FLAG_LOCK,
	},
	{
		.start = 0xf00000U,
		.size =  0x100000U,
		.master_bm_we = (1U << MCHP_SAF_MSTR_HOST_PCH_LAN),
		.master_bm_rd = (1U << MCHP_SAF_MSTR_HOST_PCH_LAN),
		.pr_num = 2U,
		.flags = MCHP_SAF_PR_FLAG_ENABLE
			 | MCHP_SAF_PR_FLAG_LOCK,
	},
};

static const struct espi_saf_protection saf_pr_w25r128 = {
	.nregions = 2U,
	.pregions = w25r128_protect_regions
};

/* Writing the RPMC Root Key is a one time operation and cannot
 * be erased. For testing, RPMC allows writing a root key of all 1's
 * which does not set the one time root key program lock.
 */
static const uint8_t rpmc_root_key_all_ones[32] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

struct espi_saf_cfg *get_saf_rpmc_flash_config(void)
{
	return (struct espi_saf_cfg *)&saf_cfg_rpmc_w25r128;
}


struct espi_saf_protection *get_saf_rpmc_flash_pr(void)
{
	return (struct espi_saf_protection *)&saf_pr_w25r128;
}

/* Send RPMC OP1 Update HMAC Key command to SAF attached flash using the
 * SAF EC Portal.
 * HMAC key does not survive flash reset/power cycle.
 * SAF packet:
 * OP1, SubCmd=0x01, CounterAddr[7:0], RSVD[7:0], KeyData[31:0], Signature[255:0]
 * Total packet length = 40 bytes
 * RPMC flash performs two HMAC-SHA-256 operations.
 * HMAC1 = HMAC-SHA-256(key=FlashRootKey[255:0], data=KeyData[31:0])
 * HMAC2 = HMAC-SHA-256(key=HMAC1,
 *                      data=OpCode[7:0] || SubCmd[7:0] || CounterAddr[7:0] ||
 *                            RSVD[7:0] || KeyData[31:0])
 */


/* RPMC Set Root Key message is 64 bytes
 * b[0] = OP1 opcode
 * b[1] = CmdType = 0x00
 * b[2] = CounterAddr[7:0] = 0, 1, 2, or 3
 * b[3] = reserved = 0x00
 * b[4 through 35] = 256-bit (32 byte) root key
 * b[36 through 63] = truncated signature: least 224 bits (28 bytes) of
 *    HMAC-SHA256 output where key = root key and
 *    message = OP1 opcode || CmdType || CounterAddr[7:0] || ReservedByte[3]
 */
static int rpmc_set_root_key_msg(uint8_t counter_id, uint8_t *msgbuf, size_t mbsz)
{
	uint32_t hmacd[32/4];
	int ret;

	if ((counter_id > 3) || !msgbuf || (mbsz < 64U)) {
		return -EINVAL;
	}

	memset(msgbuf, 0, 64);
	memset(hmacd, 0, sizeof(hmacd));

	struct rpmc_key_info *pki = &rpmc_keys[counter_id];

	msgbuf[0] = MCHP_FLASH_RPMC_OP1_DFLT;
	msgbuf[1] = SAF_RPMC_OP1_CMD_SET_ROOT_KEY;
	msgbuf[2] = counter_id;
	msgbuf[3] = 0u;
	memcpy(&msgbuf[4], pki->root_key, 32);

	ret = app_hmac_sha256_mesg(pki->root_key, 32U, msgbuf, 4U, (uint8_t *)hmacd, 32U);
	if (ret) {
		LOG_ERR("RPMC root key msg: HMAC-SHA256 error %d", ret);
		return ret;
	}

	/* RPMC v0.7 specification states root key uses 224 least significant bits
	 * of HMAC-SHA256(OP1 || RootKeyCmdID || CntrID || 0x00).
	 * FIPS HMAC specification describes numbers as left most is most signficant.
	 * HMAC library code stores "left-most" in lower memory address since Cortex-M4
	 * is a little-endian processor.
	 * Verified with W25R128 RPMC flash by sending Write Root Key.
	 * OP2 response status = 0x80 (successful completion) and returned signature matched
	 * full 32 byte HMAC-SHA256 calculated here.
	 */
	memcpy(&msgbuf[36], &hmacd[1], 28U); /* most significant at beginning of HMAC digest */

	return 0;
}

/* Assemble RPMC Update HMAC Key command packet in caller supplied buffer.
 * message length = 40 bytes
 * b[0] = OP1 opcode
 * b[1] = CmdType = 0x01
 * b[2] = CounterAddr = (0, 1, 2, or 3) based on how many counters RPMC flash implements
 * b[3] = reserved = 0
 * b[4 through 7] = 32 bit KeyData
 * b[8 through 39] = 256 bit signature
 */
static int rpmc_update_hmac_key_msg(uint8_t counter_id, uint8_t *msgbuf, size_t mbsz)
{
	int ret = 0;
	uint32_t hmac1[32/4];

	if ((counter_id > 3) || !msgbuf || (mbsz < 40)) {
		return -EINVAL;
	}

	memset(hmac1, 0, sizeof(hmac1));
	memset(msgbuf, 0, 40);

	struct rpmc_key_info *pki = &rpmc_keys[counter_id];

	msgbuf[0] = MCHP_FLASH_RPMC_OP1_DFLT;
	msgbuf[1] = SAF_RPMC_OP1_CMD_UPDATE_HMAC_KEY;
	msgbuf[2] = counter_id;
	msgbuf[3] = 0; /* reserved 0 value */

	memcpy(&msgbuf[4], &pki->hmac_key_data, 4);

	/* Generate signature:
	 * sig = HMAC-SHA256 where key = HMAC_Key[counter_id] and
	 * message = OP1 || CmdType || Counter || 0x00 || HMACKeyData[0-3]
	 */
	ret = app_hmac_sha256_mesg((const uint8_t *)pki->hmac_key, 32U,
				   (const uint8_t *)msgbuf, 8U,
				   &msgbuf[8], 32U);
	if (ret) {
		LOG_ERR("RPMC HMAC2 generation: mhmac error %d", ret);
	}

	return 0;
}

#ifdef ESPI_SAF_RPMC_TEST_INCR_COUNTER
/* Assemble RPMC OP1 Increment Monotonic Counter message in destination buffer.
 * 0 <= counter_id < MAX_NUMBER_OF_COUNTERS_IN_RPMC_FLASH
 * counter_value = previous counter value
 * dest = pointer to buffer
 * destsz = byte length of buffer, must be >= 40 bytes.
 * Total message length = 40 bytes
 *    b[0] = OP1 Opcode
 *    b[1] = CmdType = 0x02 (Increment Monotonic Counter)
 *    b[2] = CounterAddress (0, 1, 2, or 3. Depends upon how many counters implemented in the flash)
 *    b[3] = 0x00 (reserved)
 *    b[4 thru 7] = 32-bit counter data stored MSBF
 *    b[8 through 39] = 256-bit Signature
 * Signature = output of HMAC-SHA256 where key = HMAC_Key[counter_id] and
 * message = OP1 opcode || CmdType || CounterID || 0x00 || CounterData
 */
static int rpmc_incr_counter_msg(uint8_t counter_id, uint32_t counter_value,
				 uint8_t *dest, size_t destsz)
{
	if ((counter_id > 3) || !dest || (destsz < 40)) {
		return -EINVAL;
	}

	struct rpmc_key_info *pki = &rpmc_keys[counter_id];

	dest[0] = MCHP_FLASH_RPMC_OP1_DFLT;
	dest[1] = SAF_RPMC_OP1_CMD_INCR_COUNTER;
	dest[2] = counter_id;
	dest[3] = 0;
	dest[4] = (counter_value >> 24);
	dest[5] = (counter_value >> 16);
	dest[6] = (counter_value >> 8);
	dest[7] = counter_value;

	int ret = app_hmac_sha256_mesg((const uint8_t *)pki->hmac_key, 32,
				   (const uint8_t *)dest, 8U, &dest[8], 32U);

	if (ret) {
		LOG_ERR("RPMC ReqCounterMsg signature: HMAC-SHA256 error %d", ret);
	}

	return ret;
}
#endif /* ESPI_SAF_RPMC_TEST_INCR_COUNTER */

/* Assemble RPMC OP1 Request Monotonic Counter message in destination buffer.
 * Total message length = 48 bytes
 *    b[0] = OP1 Opcode
 *    b[1] = CmdType = 0x03 (Request Monotonic Counter)
 *    b[2] = CounterAddress (0, 1, 2, or 3. Depends upon how many counters implemented in the flash)
 *    b[3] = 0x00 (reserved)
 *    b[4 thru 15] = 96-bit TAG.
 *    b[16 thru 47] = 256-bit Signature
 * Signature = HMAC-SHA256 where key = HMAC_Key[counter_id] and
 * message = OP1 opcode || CmdType || CounterID || 0x00 || TAG
 */
static int rpmc_req_counter_msg(uint8_t counter_id, uint8_t *dest, size_t destsz)
{
	if ((counter_id > 3) || !dest || (destsz < 48)) {
		return -EINVAL;
	}

	struct rpmc_key_info *pki = &rpmc_keys[counter_id];

	dest[0] = MCHP_FLASH_RPMC_OP1_DFLT;
	dest[1] = SAF_RPMC_OP1_CMD_REQUEST_COUNTER;
	dest[2] = counter_id;
	dest[3] = 0;

	memcpy(&dest[4], pki->tag, 12);

	int ret = app_hmac_sha256_mesg((const uint8_t *)pki->hmac_key, 32,
				   (const uint8_t *)dest, 16U, &dest[16], 32U);

	if (ret) {
		LOG_ERR("RPMC ReqCounterMsg signature: HMAC-SHA256 error %d", ret);
		return ret;
	}

	return 0;
}

/* The SAF driver only transmits/receives the raw RPMC data from the flash.
 * It is the caller's responsibity to compute check data is authentic using
 * the HMAC-SHA256 key provisioned in the RPMC flash device.
 * RPMC implements two opcodes:
 * OP1 is a write containing subcommand and parameters to the flash device
 *   subcmd 0x00 Write Root Key. Length = 64 bytes
 *     OP1 || 0x00 || CounterAddr[7:0] || RSVD[7:0] || RootKey[255:0] || TruncatedSig[223:0]
 *   subcmd 0x01 Update HMAC. Length = 40 bytes
 *     OP1 || 0x01 || CounterAddr[7:0] || RSVD[7:0] || KeyData[31:0] || Signature[255:0]
 *   subcmd 0x02 Increment Monotonic Counter. Length = 40 bytes
 *     OP1 || 0x02 || CounterAddr[7:0] || RSVD[7:0] || CounterData[31:0] || Signature[255:0]
 *   subcmd 0x03 Request Monotonic Counter. Length = 48 bytes
 *     OP1 || 0x03 || CounterAddr[7:0] || RSVD[7:0] || TAG[96:0] || Signature[255:0]
 *
 * OP2 reads status, 12-byte TAG, 4 byte counter, and 32 byte signature from the device.
 *  TX: OP2 || RSVD[7:0]
 *  RX: RPMC.Status[7:0] || TAG[96:0] || CounterData[31:0] || Signature[255:0]
 *  Length = 51
 *
 * OP1 opcode = 0x9B
 * OP2 opcode = 0x96
 *
 * For an unprovisioned RPMC Flash at power the Root Key has not been set and
 * the HMAC key is clear (power cycle/reset).
 * We expect RPMC extended status = 0x08 b[3]=1 HMAC Key register uninitialized.
 */

/* Transmit RPMC OP1 Update HMAC Key packet followed by RPMC OP2 Read Status command.
 * Update HMAC Packet size = 40 bytes.
 * Software:
 *    Build 40 byte RPMC OP1 Update HMAC Key packet which include HMAC-SHA256 signature.
 *    Call SAF driver ECP API with 40 byte RPMC OP1 Update HMAC Key packet.
 * SAF Controller will:
 *    Replace byte[0] of packet with OP1 opcode from SAF RPMC OP1 register
 *    Transmit 40 byte packet and de-assert SPI chip select
 *    do
 *      Transmit RPMC OP2 Read Status command to SPI flash. Assert SPI chip select
 *      Read 64 bytes from SPI flash (byte[0] = RPMC Status). De-assert SPI chip select after read.
 *    while (RPMC Status BUSY set and not SAF ECP HW timeout)
 *    If exit without timeout set SAF ECP HW Status = 0x03 and fire interrupt (if enabled)
 * Software:
 *   Build RPMC OP2 Read Status packet
 *   Call SAF driver ECP API to send OP2 Read command and read status/data. SAF HW will return last
 *   cached data from the above HW polling loop.
 * Check returned RPMC Extended Status byte (first byte read).
 * If Extended Status == 0x80 then command was successfully completed
 *
 * NOTE 1: In RPMC OP2 Read Status only the first byte (RPMC Extended Status) is valid
 * when the OP1 command is Update HMAC Key. SAF HW reads a full 64 bytes. Software can
 * request reading 1 to 64 of these bytes that were cached in HW by SAF controller
 * during the above HW OP2 polling loop.
 *
 * NOTE 2: Algorithm used to generate a full 256-bit HMAC key from the 32-bit HMAC Key Data
 * sent in this OP1 command.
 * RPMC HMAC Key = HMAC-SHA256(key, message)
 * key = RootKey[counterID] 32-bytes
 * message = HMAC Key Data (4 bytes = 32 bits)
 * The 32-byte RPMC HMAC Key is stored in the RPMC flash volatile HMAC_Storage area.
 * Application firmware given the Root-Key for the specified counterID and 32-bit HMAC Key Data
 * can generate the same HMAC key as the flash for authentication/validation purposes.
 */
static int espi_saf_rpmc_update_hmac_key(uint8_t counter_id)
{
	int ret;
	struct espi_saf_packet pkt = { 0 };

	if (counter_id > 3) {
		return -EINVAL;
	}

	ret = rpmc_update_hmac_key_msg(counter_id, saf_rpmc_buf, sizeof(saf_rpmc_buf));
	if (ret) {
		LOG_ERR("SAF RPMC generate Update HMAC Key message error %d", ret);
		return ret;
	}

	LOG_INF("RPMC HMAC Update Key packet (%d bytes):", SAF_RPMC_OP1_CMD_UPDATE_HMAC_KEY_SIZE);
	pr_byte_buffer((const uint8_t *)saf_rpmc_buf, SAF_RPMC_OP1_CMD_UPDATE_HMAC_KEY_SIZE);

	espi_saf_cb_val = 0x5au;
	espi_saf_cb_sts = 0x5a5a5a5au;

	/* encode: CS=0, SAF EC Portal OP1 cmd, RPMC command = 1(update HMAC key) */
	pkt.flash_addr = ESPI_SAF_RPMC_ENCODE_PARAMS(0, MCHP_SAF_ECP_CMD_RPMC_OP1,
						     SAF_RPMC_OP1_CMD_UPDATE_HMAC_KEY);
	pkt.buf = saf_rpmc_buf;
	pkt.len = SAF_RPMC_OP1_CMD_UPDATE_HMAC_KEY_SIZE;

	LOG_INF("Use SAF EC Portal to send RPMC HMAC Key");
	ret = espi_saf_rpmc(espi_saf_dev, &pkt);
	if (ret) {
		LOG_INF("%s: OP1 CmdType %d error %d",
			__func__, SAF_RPMC_OP1_CMD_UPDATE_HMAC_KEY, ret);
		return ret;
	}

	/* Spin for eSPI SAF callback to update global variable. We don't expect to spin
	 * because the SAF driver is using interrupts and invokes the callback from the ISR.
	 */
	while (espi_saf_cb_val == 0x5au)
		;

	LOG_INF("After OP1 HMAC Update Key: SAF EC Portal callback value: %u status: 0x%x",
		espi_saf_cb_val, espi_saf_cb_sts);
	LOG_INF("saf_rpmc_buf:");
	pr_byte_buffer((const uint8_t *)saf_rpmc_buf, sizeof(saf_rpmc_buf));

	ret = -EIO;
	if (espi_saf_cb_val == SAF_CB_ECP_DONE) {
		ret = 0;
		LOG_INF("SAF RPMC OP1 Update HMAC Key: ECP Done: status=0x%x\n",
			espi_saf_cb_sts);
	} else if (espi_saf_cb_val == SAF_CB_BUS_MON) {
		LOG_ERR("SAF RPMC OP1 Update HMAC Key: Bus Monitor: status=0x%x\n",
			espi_saf_cb_sts);
	} else {
		LOG_ERR("Unexpected SAF RPMC OP1 callback value: %u\n", espi_saf_cb_val);
	}

	if (ret) {
		return ret;
	}

	/* Send RPMC OP2 Read Status/Data
	 * encode: CS=0, SAF EC Portal OP2 cmd, RPMC command = 0 (reserved)
	 * NOTE: RPMC spec. states Update HMAC Key response is only the status
	 * byte. The TAG, CounterReadData, and Signature fields can be read but
	 * are not valid for Update HMAC Key.
	 * NOTE2: SAF HW automatically issues RPMC OP2 Read Status after any RPMC OP1 command.
	 * HW will keep issuing RPMC OP2 Read Status until either the EC Portal HW timeout or
	 * returned status BUSY bit is clear. HW always reads 64 bytes for OP2 Read Status and
	 * caches the last read.  When FW uses the SAF EC Portal to do RPMC OP2 Read Status
	 * the HW acually copies the caches value into the caller's buffer.
	 */
	memset(saf_rpmc_buf2, 0, sizeof(saf_rpmc_buf2));
	pkt.flash_addr = ESPI_SAF_RPMC_ENCODE_PARAMS(0, MCHP_SAF_ECP_CMD_RPMC_OP2, 0);
	pkt.buf = saf_rpmc_buf2;
	/* TX: OP2 || 0x00, RD: ExtStatusByte || TAG(12-bytes) || CounterReadData(4 bytes)
	 *     || Signature(32 bytes)
	 */
	pkt.len = 64U;

	espi_saf_cb_val = 0x5au;
	espi_saf_cb_sts = 0x5a5a5a5au;

	ret = espi_saf_rpmc(espi_saf_dev, &pkt);
	if (ret) {
		LOG_INF("%s: OP2 error %d", __func__, ret);
		return ret;
	}

	while (espi_saf_cb_val == 0x5au)
		;

	LOG_INF("After OP2 Read Status: SAF EC Portal callback value: %u status: 0x%x",
		espi_saf_cb_val, espi_saf_cb_sts);
	LOG_INF("saf_rpmc_buf2:");
	pr_byte_buffer((const uint8_t *)saf_rpmc_buf2, sizeof(saf_rpmc_buf2));

	ret = -EIO;
	if (espi_saf_cb_val == SAF_CB_ECP_DONE) {
		ret = 0;
		LOG_INF("SAF RPMC OP2 callback value: ECP Done: status=0x%x\n", espi_saf_cb_sts);
	} else if (espi_saf_cb_val == SAF_CB_BUS_MON) {
		LOG_ERR("SAF RPMC OP2 callback value: Bus Monitor: status=0x%x\n", espi_saf_cb_sts);
	} else {
		LOG_ERR("Unexpected SAF RPMC OP2 callback value: %u\n", espi_saf_cb_val);
	}

	return ret;
}

/* Transmit RPMC OP1 Request Monotonic Counter packet followed by RPMC OP2 Read Status command.
 * Software assembles packet: Total packet length = 48 bytes
 *    b[0] = OP1 Opcode
 *    b[1] = CmdType = 0x03 (Request Monotonic Counter)
 *    b[2] = CounterAddress (0, 1, 2, or 3. Depends upon how many counters implemented in the flash)
 *    b[3] = 0x00 (reserved)
 *    b[4 thru 15] = 96-bit TAG.
 *    b[16 thru 47] = 256-bit Signature = HMAC-SHA256 where key = HMAC_key[counter_id] and
 *    message = OP1 Opcode || CmdType || CounterAddress || 0x00 || TAG
 * Call SAF driver ECP API with packet size = 48 bytes.
 * SAF Controller will:
 *    Replace byte[0] of buffer with OP1 opcode from eSPI RPMC OP1 opcode register
 *    Assert SPI CS#, transmit 48 byte packet, de-assert SPI CS#
 *    do
 *      Assert SPI CS#, transmit RPMC OP2 Read Status command to SPI flash.
 *      Read 64 bytes from SPI flash (byte[0] = RPMC Status).
 *      De-assert SPI chip select.
 *    while (RPMC Status BUSY set and not SAF ECP HW timeout)
 *    If exit without timeout set SAF ECP HW Status = 0x03 and fire interrupt (if enabled)
 * Software:
 *   Build RPMC OP2 Read Status packet
 *   Call SAF driver ECP API to send OP2 Read command and read status/data. SAF will return
 *   last cached OP2 read from above HW polling loop.
 * Check returned RPMC Extended Status byte (first byte read).
 * If Extended Status == 0x80 then command was successfully completed
 * NOTE: For Request Monotonic Counter the RPMC OP2 Read Status also returns additonal data:
 * b[0] = RPMC Extended Status
 * b[1 to 12]  = 96 bit(12 byte) TAG
 * b[13 to 16] = 32 bit(4 byte) Counter Data
 * b[17 to 48] = 256 bit (32 byte) Signature
 * SAF HW reads a full 64 bytes. Software can request reading 1 to 64 of these bytes that
 * were cached in HW by SAF controller during the above HW OP2 polling loop.
 */
static int espi_saf_rpmc_req_counter(uint8_t counter_id)
{
	int ret;
	struct rpmc_key_info *pki;
	struct espi_saf_packet pkt = { 0 };

	if (counter_id >= RPMC_MAX_COUNTERS) {
		return -EINVAL;
	}

	pki = &rpmc_keys[counter_id];

	ret = rpmc_req_counter_msg(counter_id, saf_rpmc_buf, sizeof(saf_rpmc_buf));
	if (ret) {
		LOG_ERR("SAF RPMC generate Request Counter message error %d", ret);
		return ret;
	}

	LOG_INF("RPMC Request Monotonic Counter packet (%d bytes):",
		SAF_RPMC_OP1_CMD_REQ_COUNTER_SIZE);
	pr_byte_buffer((const uint8_t *)saf_rpmc_buf, SAF_RPMC_OP1_CMD_REQ_COUNTER_SIZE);

	espi_saf_cb_val = 0x5au;
	espi_saf_cb_sts = 0x5a5a5a5au;

	/* encode: CS=0, SAF EC Portal OP1 cmd, RPMC command = 3(request counter) */
	pkt.flash_addr = ESPI_SAF_RPMC_ENCODE_PARAMS(0, MCHP_SAF_ECP_CMD_RPMC_OP1,
						     SAF_RPMC_OP1_CMD_REQUEST_COUNTER);
	pkt.buf = saf_rpmc_buf;
	pkt.len = SAF_RPMC_OP1_CMD_REQ_COUNTER_SIZE;

	LOG_INF("Use SAF EC Portal to send RPMC Request Monotonic Counter");
	ret = espi_saf_rpmc(espi_saf_dev, &pkt);
	if (ret) {
		LOG_INF("%s: OP1 CmdType %d error %d",
			__func__, SAF_RPMC_OP1_CMD_REQUEST_COUNTER, ret);
		return ret;
	}

	while (espi_saf_cb_val == 0x5au)
		;

	LOG_INF("After OP1 Request Monotonic Counter: SAF EC Portal callback value:"
		" %u status: 0x%x", espi_saf_cb_val, espi_saf_cb_sts);
	LOG_INF("saf_rpmc_buf:");
	pr_byte_buffer((const uint8_t *)saf_rpmc_buf, sizeof(saf_rpmc_buf));

	ret = -EIO;
	if (espi_saf_cb_val == SAF_CB_ECP_DONE) {
		ret = 0;
		LOG_INF("SAF RPMC OP1 Request Monotonic Counter: ECP Done: status=0x%x\n",
			espi_saf_cb_sts);
	} else if (espi_saf_cb_val == SAF_CB_BUS_MON) {
		LOG_ERR("SAF RPMC OP1 Request Monotonic Counter: Bus Monitor: status=0x%x\n",
			espi_saf_cb_sts);
	} else {
		LOG_ERR("Unexpected SAF RPMC OP1 callback value: %u\n", espi_saf_cb_val);
	}

	if (ret) {
		return ret;
	}

	/* Send RPMC OP2 Read Status/Data */
	/* encode: CS=0, SAF EC Portal OP2 cmd, RPMC command = 0 (reserved) */
	memset(saf_rpmc_buf2, 0, sizeof(saf_rpmc_buf2));
	pkt.flash_addr = ESPI_SAF_RPMC_ENCODE_PARAMS(0, MCHP_SAF_ECP_CMD_RPMC_OP2, 0);
	pkt.buf = saf_rpmc_buf2;
	/* TX: OP2 || 0x00, RD: ExtStatusByte || TAG(12-bytes) || CounterReadData(4 bytes)
	 *     || Signature(32 bytes)
	 */
	pkt.len = 64U;

	espi_saf_cb_val = 0x5au;
	espi_saf_cb_sts = 0x5a5a5a5au;

	ret = espi_saf_rpmc(espi_saf_dev, &pkt);
	if (ret) {
		LOG_INF("%s: OP2 error %d", __func__, ret);
		return ret;
	}

	while (espi_saf_cb_val == 0x5au)
		;

	LOG_INF("After OP2 Read Status: SAF EC Portal callback value: %u status: 0x%x",
		espi_saf_cb_val, espi_saf_cb_sts);
	LOG_INF("saf_rpmc_buf2:");
	pr_byte_buffer((const uint8_t *)saf_rpmc_buf2, sizeof(saf_rpmc_buf2));

	ret = -EIO;
	if (espi_saf_cb_val == SAF_CB_ECP_DONE) {
		ret = 0;
		LOG_INF("SAF RPMC OP2 callback value: ECP Done: status=0x%x\n", espi_saf_cb_sts);
	} else if (espi_saf_cb_val == SAF_CB_BUS_MON) {
		LOG_ERR("SAF RPMC OP2 callback value: Bus Monitor: status=0x%x\n", espi_saf_cb_sts);
	} else {
		LOG_ERR("Unexpected SAF RPMC OP2 callback value: %u\n", espi_saf_cb_val);
	}

	return ret;
}

#ifdef ESPI_SAF_RPMC_TEST_INCR_COUNTER
/* Transmit RPMC OP1 Increment Monotonic Counter packet followed by RPMC OP2 Read Status command.
 * Software assembles packet: Total packet length = 40 bytes
 *    b[0] = OP1 Opcode
 *    b[1] = CmdType = 0x02 (Increment Monotonic Counter)
 *    b[2] = CounterAddress (0, 1, 2, or 3. Depends upon how many counters implemented in the flash)
 *    b[3] = 0x00 (reserved)
 *    b[4 thru 7] = Previous counter value
 *    b[8 thru 39] = 256-bit Signature = HMAC-SHA256 where key = HMAC_key[counter_id] and
 *    message = OP1 Opcode || CmdType || CounterAddress || 0x00 || PrevCounterData
 * Call SAF driver ECP API with packet size = 40 bytes
 * SAF Controller will:
 *    Replace byte[0] of buffer with OP1 opcode from eSPI RPMC OP1 opcode register
 *    Assert SPI CS#, transmit 48 byte packet, de-assert SPI CS#
 *    do
 *      Assert SPI CS#, transmit RPMC OP2 Read Status command to SPI flash.
 *      Read 64 bytes from SPI flash (byte[0] = RPMC Status).
 *      De-assert SPI chip select.
 *    while (RPMC Status BUSY set and not SAF ECP HW timeout)
 *    If exit without timeout set SAF ECP HW Status = 0x03 and fire interrupt (if enabled)
 * Software:
 *   Build RPMC OP2 Read Status packet
 *   Call SAF driver ECP API to send OP2 Read command and read status/data. SAF will return
 *   last cached OP2 read from above HW polling loop.
 * Check returned RPMC Extended Status byte (first byte read).
 * If Extended Status == 0x80 then command was successfully completed. Other fields in
 * OP2 Read Status are not valid for OP1 Increment Monotonic Counter.
 * SAF HW reads a full 64 bytes. Software can request reading 1 to 64 of these bytes that
 * were cached in HW by SAF controller during the above HW OP2 polling loop.
 */
static int espi_saf_rpmc_incr_counter(uint8_t counter_id, uint32_t prev_counter_value)
{
	int ret;
	struct rpmc_key_info *pki;
	struct espi_saf_packet pkt = { 0 };

	if (counter_id >= RPMC_MAX_COUNTERS) {
		return -EINVAL;
	}

	pki = &rpmc_keys[counter_id];

	ret = rpmc_incr_counter_msg(counter_id, prev_counter_value,
				    saf_rpmc_buf, sizeof(saf_rpmc_buf));
	if (ret) {
		LOG_ERR("SAF RPMC generate RPMC Increment Monotonic Counter message error %d",
			ret);
		return ret;
	}

	LOG_INF("RPMC Increment Monotonic Counter packet (%d bytes):",
		SAF_RPMC_OP1_CMD_INCR_COUNTER_SIZE);
	pr_byte_buffer((const uint8_t *)saf_rpmc_buf, SAF_RPMC_OP1_CMD_INCR_COUNTER_SIZE);

	espi_saf_cb_val = 0x5au;
	espi_saf_cb_sts = 0x5a5a5a5au;

	/* encode: CS=0, SAF EC Portal OP1 cmd, RPMC command = 2(increment counter) */
	pkt.flash_addr = ESPI_SAF_RPMC_ENCODE_PARAMS(0, MCHP_SAF_ECP_CMD_RPMC_OP1,
						     SAF_RPMC_OP1_CMD_INCR_COUNTER);
	pkt.buf = saf_rpmc_buf;
	pkt.len = SAF_RPMC_OP1_CMD_INCR_COUNTER_SIZE;

	LOG_INF("Use SAF EC Portal to send RPMC Increment Monotonic Counter packet");
	ret = espi_saf_rpmc(espi_saf_dev, &pkt);
	if (ret) {
		LOG_INF("%s: OP1 CmdType %d error %d",
			__func__, SAF_RPMC_OP1_CMD_INCR_COUNTER, ret);
		return ret;
	}

	while (espi_saf_cb_val == 0x5au)
		;

	LOG_INF("After OP1 Increment Monotonic Counter: SAF EC Portal callback"
		" value: %u status: 0x%x", espi_saf_cb_val, espi_saf_cb_sts);
	LOG_INF("saf_rpmc_buf:");
	pr_byte_buffer((const uint8_t *)saf_rpmc_buf, sizeof(saf_rpmc_buf));

	ret = -EIO;
	if (espi_saf_cb_val == SAF_CB_ECP_DONE) {
		ret = 0;
		LOG_INF("SAF RPMC OP1 Increment Monotonic Counter: ECP Done: status=0x%x\n",
			espi_saf_cb_sts);
	} else if (espi_saf_cb_val == SAF_CB_BUS_MON) {
		LOG_ERR("SAF RPMC OP1 Increment Monotonic Counter: Bus Monitor: status=0x%x\n",
			espi_saf_cb_sts);
	} else {
		LOG_ERR("Unexpected SAF RPMC OP1 callback value: %u\n", espi_saf_cb_val);
	}

	if (ret) {
		return ret;
	}

	/* Send RPMC OP2 Read Status/Data */
	/* encode: CS=0, SAF EC Portal OP2 cmd, RPMC command = 0 (reserved) */
	memset(saf_rpmc_buf2, 0, sizeof(saf_rpmc_buf2));
	pkt.flash_addr = ESPI_SAF_RPMC_ENCODE_PARAMS(0, MCHP_SAF_ECP_CMD_RPMC_OP2, 0);
	pkt.buf = saf_rpmc_buf2;
	/* TX: OP2 || 0x00, RD: ExtStatusByte || TAG(12-bytes) || CounterReadData(4 bytes)
	 *     || Signature(32 bytes)
	 */
	pkt.len = SAF_RPMC_OP2_CMD_READ_STS_DATA_SIZE + 2U;

	espi_saf_cb_val = 0x5au;
	espi_saf_cb_sts = 0x5a5a5a5au;

	ret = espi_saf_rpmc(espi_saf_dev, &pkt);
	if (ret) {
		LOG_INF("%s: OP2 error %d", __func__, ret);
		return ret;
	}

	while (espi_saf_cb_val == 0x5au)
		;

	LOG_INF("After OP2 Read Status: SAF EC Portal callback value: %u status: 0x%x",
		espi_saf_cb_val, espi_saf_cb_sts);
	LOG_INF("saf_rpmc_buf2:");
	pr_byte_buffer((const uint8_t *)saf_rpmc_buf2, sizeof(saf_rpmc_buf2));

	ret = -EIO;
	if (espi_saf_cb_val == SAF_CB_ECP_DONE) {
		ret = 0;
		LOG_INF("SAF RPMC OP2 callback value: ECP Done: status=0x%x\n", espi_saf_cb_sts);
	} else if (espi_saf_cb_val == SAF_CB_BUS_MON) {
		LOG_ERR("SAF RPMC OP2 callback value: Bus Monitor: status=0x%x\n", espi_saf_cb_sts);
	} else {
		LOG_ERR("Unexpected SAF RPMC OP2 callback value: %u\n", espi_saf_cb_val);
	}

	return ret;
}
#endif /* ESPI_SAF_RPMC_TEST_INCR_COUNTER */

int rpmc_init_keys(void)
{
	for (size_t n = 0; n < ARRAY_SIZE(rpmc_keys); n++) {
		rpmc_keys[n].hmac_key_data = rpmc_key_data[n];
		memcpy(rpmc_keys[n].root_key, rpmc_root_key_all_ones, 32);

		int ret = app_hmac_sha256_mesg(rpmc_keys[n].root_key, 32,
					       (uint8_t *)&rpmc_keys[n].hmac_key_data, 4,
					       (uint8_t *)rpmc_keys[n].hmac_key, 32U);

		if (ret) {
			LOG_ERR("RPMC init keys: HMAC-SHA256 error %d", ret);
			return ret;
		}

		/* fill tag with random data */
		ret = app_get_rand(rpmc_keys[n].tag, 12U, 12U);
		if (ret) {
			LOG_ERR("RPMC init keys: get rand error %d", ret);
			return ret;
		}
	}

	return 0;
}

/* Write test RPMC Root Key (all one's) to RPMC SPI flash using Zephyr SPI driver.
 * Requires rpmc_init_keys to have been called.
 * 1. Build Write Root Key message (64 bytes)
 * 2. Transmit message via SPI driver
 * 3. Begin polling loop
 *      Send RPMC OP2 Read Status command and read response via Zephyr SPI driver
 *    Loop until RPMC Extended Status indicates Done.
 * 4. Check Extended Status is Successful Completion
 */
int saf_rpmc_spi_check_root_key(const struct device *spi_dev, uint8_t rpmc_counter_id)
{
	struct spi_config spi_cfg;
	struct spi_buf_set tx_bufs;
	struct spi_buf_set rx_bufs;
	struct spi_buf txb;
	struct spi_buf rxb;
	uint32_t d;
	int op2_count, ret;

	if (!spi_dev || (rpmc_counter_id > 3)) {
		return -EINVAL;
	}

	spi_cfg.frequency = MHZ(12);
	spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB
			    | SPI_WORD_SET(8);
	spi_cfg.slave = 0;
	spi_cfg.cs = NULL;

	memset(saf_rpmc_buf, 0, sizeof(saf_rpmc_buf));
	memset(saf_rpmc_buf2, 0, sizeof(saf_rpmc_buf2));

	ret = rpmc_set_root_key_msg(rpmc_counter_id, saf_rpmc_buf, 64U);
	if (ret) {
		LOG_ERR("SPI SAF RPMC generate root key message error %d", ret);
		return ret;
	}

	txb.buf = saf_rpmc_buf;
	txb.len = 64U;

	tx_bufs.buffers = (const struct spi_buf *)&txb;
	tx_bufs.count = 1U;

	ret = spi_transceive(spi_dev, (const struct spi_config *)&spi_cfg,
			     (const struct spi_buf_set *)&tx_bufs, NULL);
	if (ret) {
		LOG_ERR("SAF SPI transmit Write Root Key error %d", ret);
	}

	/* Issue RPMC OP2 Read Status/Data */
	op2_count = 0;
	do {
		d = 0x96U; /* b[7:0] = OP2 opcode, b[15:8] = 0x00 reserved byte */
		txb.buf = &d;
		txb.len = 2U;

		tx_bufs.buffers = (const struct spi_buf *)&txb;
		tx_bufs.count = 1U;

		rxb.buf = saf_rpmc_buf2;
		rxb.len = 49U; /* status(1) || tag(12) || counter_data(4) || signature(32) */

		rx_bufs.buffers = (const struct spi_buf *)&rxb;
		rx_bufs.count = 1U;

		ret = spi_transceive(spi_dev, (const struct spi_config *)&spi_cfg,
				     (const struct spi_buf_set *)&tx_bufs,
				     (const struct spi_buf_set *)&rx_bufs);
		if (ret) {
			LOG_ERR("SAF SPI transmit RPMC OP2 Read Status error %d", ret);
			return ret;
		}

		op2_count++;
	} while (saf_rpmc_buf2[0] & SAF_RPMC_EXT_STATUS_BUSY);

	LOG_INF("SPI RPMC: Sent OP1 Write Root Key followed by %d OP2 Read Status/Data", op2_count);
	LOG_INF("RPMC Status: 0x%02x", saf_rpmc_buf2[0]);
	LOG_INF("TAG len=12:");
	pr_byte_buffer((const uint8_t *)&saf_rpmc_buf2[1], 12);
	LOG_INF("Counter data len=4:");
	pr_byte_buffer((const uint8_t *)&saf_rpmc_buf2[13], 4);
	LOG_INF("Signature:");
	pr_byte_buffer((const uint8_t *)&saf_rpmc_buf2[17], 32);

	if (saf_rpmc_buf2[0] != SAF_RPMC_EXT_STATUS_OP1_SUCCESS) {
		return -EACCES;
	}

	return 0;
}

int saf_rpmc_spi_check_root_keys(const struct device *spi_dev)
{
	for (uint8_t counter_id = 0; counter_id < RPMC_MAX_COUNTERS; counter_id++) {
		int ret = saf_rpmc_spi_check_root_key(spi_dev, counter_id);

		if (ret) {
			LOG_ERR("SPI RPMC check root key for counter %u error %d",
				counter_id, ret);
		}
	}

	return 0;
}

/* Extract 32-bit monotonic counter value from RPMC request monotonic 48 byte packet
 * received from flash. Counter is stored MSBF in offsets 15 - 18 by the specication.
 * Our received buffer does not contain the first two bytes (opcode | reserved),
 * therefore we use offsets 13 - 16.
 */
static int extract_req_counter(const uint8_t *rpmc_mesg, size_t rpmc_mesg_size, uint32_t *counter)
{
	uint32_t temp = 0U;

	if (!counter || !rpmc_mesg || rpmc_mesg_size != SAF_RPMC_OP1_CMD_REQ_COUNTER_SIZE) {
		return -EINVAL;
	}

	for (int i = 13; i < 17; i++) {
		temp <<= 8;
		temp |= rpmc_mesg[i];
	}

	*counter = temp;

	return 0;
}

/* SAF RPMC test1
 * Initial conditions:
 * SPI flash implements RPMC HW.
 * SPI flash has been power cycled:
 *    RPMC Root Key may be present. This app tries to program RPMC test Root Key
 *    of all 1's.
 *    HMAC keys are ephemeral and are not present.
 *
 * If RPMC SPI flash has Root Key programmed, the application must send the HMAC Key Update
 * command. If HMAC Key Update successful, the application can use the Request Monotonic Counter,
 * and Increment Monotonic Counter OP1 commands.
 * NOTE: Each OP1 command must be followed with an OP2 Read Status/Data command.
 */
int espi_saf_rpmc_test1(void)
{
	int ret = 0;
	uint8_t counter_id = 0u;
	uint32_t initial_counter = 0u;
	uint32_t current_counter = 0u;
	uint32_t ext_status = 0u;
	struct rpmc_key_info *pki = NULL;
	uint8_t hmacd[32] = {0};

	if (counter_id >= RPMC_MAX_COUNTERS) {
		return -EINVAL;
	}

	pki = &rpmc_keys[counter_id];

	ret = espi_saf_rpmc_update_hmac_key(counter_id);
	if (ret) {
		LOG_ERR("eSPI SAF RPMC test1: Update HMAC key error %d", ret);
		return ret;
	}

	ret = espi_saf_rpmc_req_counter(counter_id);
	if (ret) {
		LOG_ERR("eSPI SAF RPMC test1: Request Monotinic Counter error %d", ret);
		return ret;
	}

	/* saf_rpmc_buf2 contains status, TAG[12], counterData[4], and Signature[32]
	 * We should authenticate the message before using the new counterData.
	 * Compute HMAC-SHA256 where key = HMAC_Key[counter_id] and
	 * message = TAG || counterData
	 */
	ext_status = saf_rpmc_buf2[0];
	if (ext_status != SAF_RPMC_EXT_STATUS_OP1_SUCCESS) {
		LOG_ERR("eSPI SAF RPMC Request Counter OP1 status error 0x%02x", ext_status);
		return -EIO;
	}

	initial_counter = 0xffffffffu;
	ret = extract_req_counter((const uint8_t *)saf_rpmc_buf2,
				  SAF_RPMC_OP1_CMD_REQ_COUNTER_SIZE, &initial_counter);
	if (ret) {
		LOG_ERR("eSPI SAF RPMC test1: extract counter from RPMC packet error %d", ret);
		return ret;
	}

	LOG_INF("eSPI SAF RPMC Request Monotonic Counter %d: Status=0x%x", counter_id, ext_status);
	LOG_INF("Counter value: 0x%04x", initial_counter);
	LOG_INF("TAG:");
	pr_byte_buffer((const uint8_t *)&saf_rpmc_buf2[1], 12);
	LOG_INF("Signature:");
	pr_byte_buffer((const uint8_t *)&saf_rpmc_buf2[17], 32);

	ret = app_hmac_sha256_mesg((const uint8_t *)pki->hmac_key, 32,
				   &saf_rpmc_buf2[1], 16, hmacd, 32U);
	if (ret) {
		LOG_ERR("HMAC-SHA256 error %d", ret);
		return ret;
	}

	LOG_INF("Calculated signature using HMAC Key[%d]", counter_id);
	pr_byte_buffer((const uint8_t *)hmacd, 32);

	ret = memcmp(&saf_rpmc_buf2[17], hmacd, 32);
	if (ret) {
		LOG_INF("SAF RPMC Request Monotonic Counter packet authenticated: FAIL\n");
		return ret;
	} else {
		LOG_INF("SAF RPMC Request Monotonic Counter packet authenticated: PASS\n");
	}

	LOG_INF("Current Monotonic Counter = 0x%04x", initial_counter);
	LOG_INF("Send RPMC Increment Monotonic Counter command packet");

	ret = espi_saf_rpmc_incr_counter(counter_id, initial_counter);
	if (ret) {
		LOG_ERR("eSPI SAF RPMC test1: Increment Monotinic Counter error %d", ret);
		return ret;
	}

	ext_status = saf_rpmc_buf2[0];
	if (ext_status != SAF_RPMC_EXT_STATUS_OP1_SUCCESS) {
		LOG_ERR("eSPI SAF RPMC Increment Counter OP1 status error 0x%02x", ext_status);
		return -EIO;
	}

	LOG_INF("After increment read monotonic counter from RPMC flash");

	ret = espi_saf_rpmc_req_counter(counter_id);
	if (ret) {
		LOG_ERR("eSPI SAF RPMC test1: Request Monotinic Counter error %d", ret);
		return ret;
	}

	ext_status = saf_rpmc_buf2[0];
	if (ext_status != SAF_RPMC_EXT_STATUS_OP1_SUCCESS) {
		LOG_ERR("eSPI SAF RPMC Request Counter OP1 status error 0x%02x", ext_status);
		return -EIO;
	}

	current_counter = 0xffffffffu;
	ret = extract_req_counter((const uint8_t *)saf_rpmc_buf2,
				  SAF_RPMC_OP1_CMD_REQ_COUNTER_SIZE, &current_counter);
	if (ret) {
		LOG_ERR("eSPI SAF RPMC test1: extract counter from RPMC packet error %d", ret);
		return ret;
	}

	LOG_INF("eSPI SAF RPMC Request Monotonic Counter %d: Status=0x%x", counter_id, ext_status);
	LOG_INF("Counter value: 0x%04x", current_counter);
	LOG_INF("TAG:");
	pr_byte_buffer((const uint8_t *)&saf_rpmc_buf2[1], 12);
	LOG_INF("Signature:");
	pr_byte_buffer((const uint8_t *)&saf_rpmc_buf2[17], 32);

	ret = app_hmac_sha256_mesg((const uint8_t *)pki->hmac_key, 32,
				   &saf_rpmc_buf2[1], 16, hmacd, 32U);
	if (ret) {
		LOG_ERR("HMAC-SHA256 error %d", ret);
		return ret;
	}

	LOG_INF("Calculated signature using HMAC Key[%d]", counter_id);
	pr_byte_buffer((const uint8_t *)hmacd, 32);

	ret = memcmp(&saf_rpmc_buf2[17], hmacd, 32);
	if (ret) {
		LOG_INF("SAF RPMC Request Monotonic Counter packet authenticated: FAIL");
		return ret;
	} else {
		LOG_INF("SAF RPMC Request Monotonic Counter packet authenticated: PASS");
	}

	return ret;
}
