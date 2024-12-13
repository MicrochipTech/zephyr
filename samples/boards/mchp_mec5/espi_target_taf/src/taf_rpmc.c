/*
 * Copyright (c) 2022 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <mec_espi_taf.h>

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_taf.h>
#include <zephyr/drivers/espi/espi_mchp_mec5.h>
#include <zephyr/drivers/espi/espi_taf_mchp_mec5.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(app);

#include "taf_rpmc.h"
#include "app_hmac.h"
#include "espi_taf_sample.h"

#define ESPI_TAF_RPMC_TEST_INCR_COUNTER

#define RPMC_MAX_COUNTERS	4

struct rpmc_key_info {
	uint32_t hmac_key_data;
	uint8_t root_key[32];
	uint8_t hmac_key[32];
	uint8_t tag[12];
};

static uint8_t taf_rpmc_buf[64] __aligned(4);
static uint8_t taf_rpmc_buf2[64] __aligned(4);

/* RPMC HMAC Key Data is a 4 byte value used to derive a 32 byte
 * HMAC-SHA-256 key.
 */
static const uint32_t rpmc_key_data[RPMC_MAX_COUNTERS] = {
	0xbc1f19a1U, 0x9e7c7aa5U, 0x58f8b611U, 0x39b51dc8U
};

static struct rpmc_key_info rpmc_keys[RPMC_MAX_COUNTERS];


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

/* Send RPMC OP1 Update HMAC Key command to TAF attached flash using the
 * SAF EC Portal.
 * HMAC key does not survive flash reset/power cycle.
 * TAF packet:
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
	msgbuf[1] = RPMC_OP1_CMD_SET_ROOT_KEY;
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
	msgbuf[1] = RPMC_OP1_CMD_UPDATE_HMAC_KEY;
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

#ifdef ESPI_TAF_RPMC_TEST_INCR_COUNTER
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
	dest[1] = RPMC_OP1_CMD_INCR_COUNTER;
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
#endif /* ESPI_TAF_RPMC_TEST_INCR_COUNTER */

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
	dest[1] = RPMC_OP1_CMD_REQUEST_COUNTER;
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

int taf_send_rpmc_op1(uint8_t *buf, uint8_t len, uint8_t op1_subcmd)
{
	const struct device *taf_dev = app_get_taf_device();
	struct espi_taf_rpmc_packet pkt = { 0 };
	struct espi_event ev = {0};
	int ret = 0;

	if (!buf || (len > 64u)) {
		return -EINVAL;
	}

	LOG_INF("Use TAF EC Portal to send RPMC OP1 subcmd=0x%02x len=%u", op1_subcmd, len);

	espi_taf_cb_data_init();

	pkt.buf = buf;
	pkt.len = len;
	pkt.cmd = ESPI_TAF_CMD_RPMC_OP1;
	pkt.subcmd = op1_subcmd;
	pkt.cs = 0;

	ret = espi_taf_mchp_rpmc_operation(taf_dev, &pkt, ESPI_TAF_MCHP_RPMC_OP_FLAG_ASYNC);
	if (ret) {
		LOG_INF("%s: OP1 CmdType %d error %d",
			__func__, RPMC_OP1_CMD_UPDATE_HMAC_KEY, ret);
		return ret;
	}

	/* Spin for eSPI TAF callback to update a flag. The TAF driver uses interrupts and
	 * invokes the callback from the ISR.
	 */
	while (!espi_taf_cb_has_fired())
		;

	espi_taf_cb_get_event(&ev);
	LOG_INF("eSPI TAF callback info: %u : 0x%0x : 0x%0x", ev.evt_type, ev.evt_details,
		ev.evt_data);

	ret = -EIO;
	if (ev.evt_details & ESPI_TAF_ECP_RPMC_OP1) {
		ret = 0;
		LOG_INF("TAF RPMC OP1: ECP Done: status=0x%x\n", ev.evt_data);
	} else if (ev.evt_details & ESPI_TAF_ECP_PROTOCOL_ERR) {
		LOG_ERR("TAF RPMC OP1: Bus Monitor: status=0x%x\n", ev.evt_data);
	} else {
		LOG_ERR("Unexpected TAF RPMC OP1 callback data\n");
	}

	return ret;
}

/* Send RPMC OP2 command to flash to get extended status.
 * NOTE: MCHP TAF HW automatically issues RPMC OP2 after sending any RPMC OP1 message.
 * HW caches the recevied RPMC OP2 packet. This operation usually returns the last
 * cached RPMC OP2 packet.
 */
int taf_send_rpmc_op2(uint8_t *rpmc_ext_status)
{
	const struct device *taf_dev = app_get_taf_device();
	struct espi_taf_rpmc_packet pkt = { 0 };
	struct espi_event ev = {0};
	int ret = 0;

	LOG_INF("Use TAF EC Portal to send RPMC OP2 cmd packet");

	memset(taf_rpmc_buf2, 0, sizeof(taf_rpmc_buf2));

	/* Send RPMC OP2 Read Status/Data
	 * encode: CS=0, TAF EC Portal OP2 cmd, RPMC command = 0 (reserved)
	 * NOTE: RPMC spec. states Update HMAC Key response is only the status
	 * byte. The TAG, CounterReadData, and Signature fields can be read but
	 * are not valid for Update HMAC Key.
	 * NOTE2: TAF HW automatically issues RPMC OP2 Read Status after any RPMC OP1 command.
	 * HW will keep issuing RPMC OP2 Read Status until either the EC Portal HW timeout or
	 * returned status BUSY bit is clear. HW always reads 64 bytes for OP2 Read Status and
	 * caches the last read.  When FW uses the TAF EC Portal to do RPMC OP2 Read Status
	 * the HW acually copies the caches value into the caller's buffer.
	 */
	pkt.buf = taf_rpmc_buf2;
	pkt.len = 64u;
	pkt.cmd = ESPI_TAF_CMD_RPMC_OP2;
	pkt.subcmd = 0;
	pkt.cs = 0;

	/* TX: OP2 || 0x00,
	 * RX: ExtStatusByte || TAG(12-bytes) || CounterReadData(4 bytes) || Signature(32 bytes)
	 */

	espi_taf_cb_data_init();

	ret = espi_taf_mchp_rpmc_operation(taf_dev, &pkt, ESPI_TAF_MCHP_RPMC_OP_FLAG_ASYNC);
	if (ret) {
		LOG_INF("%s: Send RPMC OP2 error %d", __func__, ret);
		return ret;
	}

	while (!espi_taf_cb_has_fired())
		;

	espi_taf_cb_get_event(&ev);
	LOG_INF("Send RPMC OP2 eSPI TAF callback info: %u : 0x%0x : 0x%0x", ev.evt_type,
		ev.evt_details, ev.evt_data);

	LOG_HEXDUMP_INF((const uint8_t *)taf_rpmc_buf2, sizeof(taf_rpmc_buf2),
			"RPMC OP2 response data buffer");

	ret = -EIO;
	if (ev.evt_details & ESPI_TAF_ECP_RPMC_OP2) {
		if (rpmc_ext_status) {
			*rpmc_ext_status = taf_rpmc_buf2[0];
		}
		ret = 0;
		LOG_INF("TAF RPMC OP2: ECP Done: status=0x%x\n", ev.evt_data);
	} else if (ev.evt_details & ESPI_TAF_ECP_PROTOCOL_ERR) {
		ret = -EIO;
		LOG_ERR("TAF RPMC OP2: Bus Monitor: status=0x%x\n", ev.evt_data);
	} else {
		ret = -ENOSYS;
		LOG_ERR("Unexpected TAF RPMC OP2 callback data\n");
	}

	return ret;
}

/* The TAF driver only transmits/receives the raw RPMC data from the flash.
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
 *    Call TAF driver ECP API with 40 byte RPMC OP1 Update HMAC Key packet.
 * TAF Controller will:
 *    Replace byte[0] of packet with OP1 opcode from TAF RPMC OP1 register
 *    Transmit 40 byte packet and de-assert SPI chip select
 *    do
 *      Transmit RPMC OP2 Read Status command to SPI flash. Assert SPI chip select
 *      Read 64 bytes from SPI flash (byte[0] = RPMC Status). De-assert SPI chip select after read.
 *    while (RPMC Status BUSY set and not TAF ECP HW timeout)
 *    If exit without timeout set TAF ECP HW Status = 0x03 and fire interrupt (if enabled)
 * Software:
 *   Build RPMC OP2 Read Status packet
 *   Call TAF driver ECP API to send OP2 Read command and read status/data. TAF HW will return last
 *   cached data from the above HW polling loop.
 * Check returned RPMC Extended Status byte (first byte read).
 * If Extended Status == 0x80 then command was successfully completed
 *
 * NOTE 1: In RPMC OP2 Read Status only the first byte (RPMC Extended Status) is valid
 * when the OP1 command is Update HMAC Key. TAF HW reads a full 64 bytes. Software can
 * request reading 1 to 64 of these bytes that were cached in HW by TAF controller
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
#if 0 /* previous version */
static int espi_taf_rpmc_update_hmac_key(uint8_t counter_id)
{
	const struct device *taf_dev = app_get_taf_device();
	struct espi_taf_rpmc_packet pkt = { 0 };
	struct espi_event ev = {0};
	int ret = 0;

	if (counter_id > 3) {
		return -EINVAL;
	}

	ret = rpmc_update_hmac_key_msg(counter_id, taf_rpmc_buf, sizeof(taf_rpmc_buf));
	if (ret) {
		LOG_ERR("TAF RPMC generate Update HMAC Key message error %d", ret);
		return ret;
	}

	LOG_HEXDUMP_INF((const uint8_t *)taf_rpmc_buf, RPMC_OP1_CMD_UPDATE_HMAC_KEY_SIZE,
			"RPMC HMAC Update Key packet");

	espi_taf_cb_data_init();

	pkt.buf = taf_rpmc_buf;
	pkt.len = ESPI_TAF_RPMC_OP1_CMD_UPDATE_HMAC_KEY_SIZE;
	pkt.cmd = ESPI_TAF_CMD_RPMC_OP1;
	pkt.subcmd = ESPI_TAF_SUBCMD_RPMC_OP1_UPDATE_HMAC_KEY;
	pkt.cs = 0;

	LOG_INF("Use TAF EC Portal to send RPMC HMAC Key");

	ret = espi_taf_mchp_rpmc_operation(taf_dev, &pkt, true);
	if (ret) {
		LOG_INF("%s: OP1 CmdType %d error %d",
			__func__, RPMC_OP1_CMD_UPDATE_HMAC_KEY, ret);
		return ret;
	}

	/* Spin for eSPI TAF callback to update a flag. The TAF driver uses interrupts and
	 * invokes the callback from the ISR.
	 */
	while (!espi_taf_cb_has_fired())
		;

	LOG_HEXDUMP_INF((const uint8_t *)taf_rpmc_buf, sizeof(taf_rpmc_buf),
			"App TAF RPMC buffer");

	espi_taf_cb_get_event(&ev);
	LOG_INF("eSPI TAF callback info: %u : 0x%0x : 0x%0x", ev.evt_type, ev.evt_details,
		ev.evt_data);

	ret = -EIO;
	if (ev.evt_details & ESPI_TAF_ECP_RPMC_OP1) {
		ret = 0;
		LOG_INF("TAF RPMC OP1 Update HMAC Key: ECP Done: status=0x%x\n", ev.evt_data);
	} else if (ev.evt_details & ESPI_TAF_ECP_PROTOCOL_ERR) {
		LOG_ERR("TAF RPMC OP1 Update HMAC Key: Bus Monitor: status=0x%x\n", ev.evt_data);
	} else {
		LOG_ERR("Unexpected TAF RPMC OP1 callback data\n");
	}

	if (ret) {
		return ret;
	}

	/* Send RPMC OP2 Read Status/Data
	 * encode: CS=0, TAF EC Portal OP2 cmd, RPMC command = 0 (reserved)
	 * NOTE: RPMC spec. states Update HMAC Key response is only the status
	 * byte. The TAG, CounterReadData, and Signature fields can be read but
	 * are not valid for Update HMAC Key.
	 * NOTE2: TAF HW automatically issues RPMC OP2 Read Status after any RPMC OP1 command.
	 * HW will keep issuing RPMC OP2 Read Status until either the EC Portal HW timeout or
	 * returned status BUSY bit is clear. HW always reads 64 bytes for OP2 Read Status and
	 * caches the last read.  When FW uses the TAF EC Portal to do RPMC OP2 Read Status
	 * the HW acually copies the caches value into the caller's buffer.
	 */
	memset(taf_rpmc_buf2, 0, sizeof(taf_rpmc_buf2));

	pkt.buf = taf_rpmc_buf2;
	pkt.len = 64u;
	pkt.cmd = ESPI_TAF_CMD_RPMC_OP2;
	pkt.subcmd = 0;
	pkt.cs = 0;

	/* TX: OP2 || 0x00, RD: ExtStatusByte || TAG(12-bytes) || CounterReadData(4 bytes)
	 *     || Signature(32 bytes)
	 */

	espi_taf_cb_data_init();

	ret = espi_taf_mchp_rpmc_operation(taf_dev, &pkt, true);
	if (ret) {
		LOG_INF("%s: Send RPMC OP2 error %d", __func__, ret);
		return ret;
	}

	while (!espi_taf_cb_has_fired())
		;

	espi_taf_cb_get_event(&ev);
	LOG_INF("eSPI TAF callback info: %u : 0x%0x : 0x%0x", ev.evt_type, ev.evt_details,
		ev.evt_data);

	LOG_HEXDUMP_INF((const uint8_t *)taf_rpmc_buf2, sizeof(taf_rpmc_buf2), "App TAF RPMC Buf2");

	ret = -EIO;
	if (ev.evt_details & ESPI_TAF_ECP_RPMC_OP1) {
		ret = 0;
		LOG_INF("TAF RPMC OP2: ECP Done: status=0x%x\n", ev.evt_data);
	} else if (ev.evt_details & ESPI_TAF_ECP_PROTOCOL_ERR) {
		ret = -EIO;
		LOG_ERR("TAF RPMC OP2: Bus Monitor: status=0x%x\n", ev.evt_data);
	} else {
		ret = -ENOSYS;
		LOG_ERR("Unexpected TAF RPMC OP2 callback data\n");
	}

	return ret;
}
#else

static void pr_rpmc_op2_ext_status(uint8_t rpmc_ext_status)
{
	if (rpmc_ext_status == 0u) {
		LOG_INF("RPMC ExtStatus=0x00: Power On State");
	} else if (rpmc_ext_status == 0x80u) {
		LOG_INF("RPMC ExtStatus=0x80: OP1 Successful Completion");
	} else if ((rpmc_ext_status & 0x81u) == 0x01u) {
		LOG_INF("RPMC ExtStatus=0x01: Busy");
	} else if ((rpmc_ext_status & 0x82u) == 0x02u) {
		LOG_ERR("RPMC ExtStatus=0x%02x: Payload error", rpmc_ext_status);
	} else {
		LOG_ERR("RPMC ExtStatus=0x%02x: Unkown", rpmc_ext_status);
	}
}

static int espi_taf_rpmc_update_hmac_key(uint8_t counter_id, uint8_t *rpmc_ext_status)
{
	int ret = 0;

	if ((counter_id > 3) || !rpmc_ext_status) {
		return -EINVAL;
	}

	ret = rpmc_update_hmac_key_msg(counter_id, taf_rpmc_buf, sizeof(taf_rpmc_buf));
	if (ret) {
		LOG_ERR("TAF RPMC generate Update HMAC Key message error %d", ret);
		return ret;
	}

	LOG_HEXDUMP_INF((const uint8_t *)taf_rpmc_buf, RPMC_OP1_CMD_UPDATE_HMAC_KEY_SIZE,
			"RPMC HMAC Update Key packet");

	ret = taf_send_rpmc_op1(taf_rpmc_buf, ESPI_TAF_RPMC_OP1_CMD_UPDATE_HMAC_KEY_SIZE,
				ESPI_TAF_SUBCMD_RPMC_OP1_UPDATE_HMAC_KEY);
	if (ret) {
		LOG_ERR("TAF send RPMC OP1 error (%d)", ret);
		return ret;
	}

	ret = taf_send_rpmc_op2(rpmc_ext_status);
	if (ret) {
		LOG_ERR("TAF send RPMC OP2 error (%d)", ret);
	}

	pr_rpmc_op2_ext_status(*rpmc_ext_status);

	return ret;
}
#endif

/* Transmit RPMC OP1 Request Monotonic Counter packet followed by RPMC OP2 Read Status command.
 * Software assembles packet: Total packet length = 48 bytes
 *    b[0] = OP1 Opcode
 *    b[1] = CmdType = 0x03 (Request Monotonic Counter)
 *    b[2] = CounterAddress (0, 1, 2, or 3. Depends upon how many counters implemented in the flash)
 *    b[3] = 0x00 (reserved)
 *    b[4 thru 15] = 96-bit TAG.
 *    b[16 thru 47] = 256-bit Signature = HMAC-SHA256 where key = HMAC_key[counter_id] and
 *    message = OP1 Opcode || CmdType || CounterAddress || 0x00 || TAG
 * Call TAF driver ECP API with packet size = 48 bytes.
 * TAF Controller will:
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
 *   Call TAF driver ECP API to send OP2 Read command and read status/data. TAF will return
 *   last cached OP2 read from above HW polling loop.
 * Check returned RPMC Extended Status byte (first byte read).
 * If Extended Status == 0x80 then command was successfully completed
 * NOTE: For Request Monotonic Counter the RPMC OP2 Read Status also returns additonal data:
 * b[0] = RPMC Extended Status
 * b[1 to 12]  = 96 bit(12 byte) TAG
 * b[13 to 16] = 32 bit(4 byte) Counter Data
 * b[17 to 48] = 256 bit (32 byte) Signature
 * TAF HW reads a full 64 bytes. Software can request reading 1 to 64 of these bytes that
 * were cached in HW by SAF controller during the above HW OP2 polling loop.
 */
#if 0
static int espi_taf_rpmc_req_counter(uint8_t counter_id)
{
	const struct device *taf_dev = app_get_taf_device();
	struct rpmc_key_info *pki = NULL;
	struct espi_taf_rpmc_packet pkt = { 0 };
	struct espi_event ev = {0};
	int ret = 0;

	if (counter_id >= RPMC_MAX_COUNTERS) {
		return -EINVAL;
	}

	pki = &rpmc_keys[counter_id];

	ret = rpmc_req_counter_msg(counter_id, taf_rpmc_buf, sizeof(taf_rpmc_buf));
	if (ret) {
		LOG_ERR("SAF RPMC generate Request Counter message error %d", ret);
		return ret;
	}

	LOG_HEXDUMP_INF((const uint8_t *)taf_rpmc_buf, RPMC_OP1_CMD_REQ_COUNTER_SIZE,
			"RPMC Request Monotonic Counter packet");

	espi_taf_cb_data_init();

	pkt.buf = taf_rpmc_buf;
	pkt.len = ESPI_TAF_RPMC_OP1_CMD_REQ_COUNTER_SIZE;
	pkt.cmd = ESPI_TAF_CMD_RPMC_OP1;
	pkt.subcmd = ESPI_TAF_SUBCMD_RPMC_OP1_REQUEST_COUNTER;
	pkt.cs = 0;

	LOG_INF("Use TAF EC Portal to send RPMC Request Monotonic Counter");


	ret = espi_taf_mchp_rpmc_operation(taf_dev, &pkt, true);
	if (ret) {
		LOG_INF("%s: OP1 CmdType %d error %d",
			__func__, ESPI_TAF_SUBCMD_RPMC_OP1_REQUEST_COUNTER, ret);
		return ret;
	}

	while (!espi_taf_cb_has_fired())
		;

	espi_taf_cb_get_event(&ev);
	LOG_INF("eSPI TAF callback info: %u : 0x%0x : 0x%0x", ev.evt_type, ev.evt_details,
		ev.evt_data);

	LOG_HEXDUMP_INF((const uint8_t *)taf_rpmc_buf, sizeof(taf_rpmc_buf), "taf_rpmc_buf");

	ret = -EIO;
	if (ev.evt_details & ESPI_TAF_ECP_RPMC_OP1) {
		ret = 0;
		LOG_INF("TAF RPMC OP1 Update HMAC Key: ECP Done: status=0x%x\n", ev.evt_data);
	} else if (ev.evt_details & ESPI_TAF_ECP_PROTOCOL_ERR) {
		LOG_ERR("TAF RPMC OP1 Update HMAC Key: Bus Monitor: status=0x%x\n", ev.evt_data);
	} else {
		LOG_ERR("Unexpected TAF RPMC OP1 callback data\n");
	}

	if (ret) {
		return ret;
	}

	/* Send RPMC OP2 Read Status/Data */
	/* encode: CS=0, SAF EC Portal OP2 cmd, RPMC command = 0 (reserved) */
	memset(taf_rpmc_buf2, 0, sizeof(taf_rpmc_buf2));

	espi_taf_cb_data_init();

	pkt.buf = taf_rpmc_buf2;
	pkt.len = 64u;
	pkt.cmd = ESPI_TAF_CMD_RPMC_OP2;
	pkt.subcmd = 0;
	pkt.cs = 0;

	/* TX: OP2 || 0x00, RD: ExtStatusByte || TAG(12-bytes) || CounterReadData(4 bytes)
	 *     || Signature(32 bytes)
	 */

	ret = espi_taf_mchp_rpmc_operation(taf_dev, &pkt, true);
	if (ret) {
		LOG_INF("%s: OP2 error %d", __func__, ret);
		return ret;
	}

	while (!espi_taf_cb_has_fired())
		;

	espi_taf_cb_get_event(&ev);
	LOG_INF("eSPI TAF callback info: %u : 0x%0x : 0x%0x", ev.evt_type, ev.evt_details,
		ev.evt_data);

	LOG_HEXDUMP_INF((const uint8_t *)taf_rpmc_buf2, sizeof(taf_rpmc_buf2), "taf_rpmc_buf2");

	ret = -EIO;
	if (ev.evt_details & ESPI_TAF_ECP_RPMC_OP1) {
		ret = 0;
		LOG_INF("TAF RPMC OP2: ECP Done: status=0x%x\n", ev.evt_data);
	} else if (ev.evt_details & ESPI_TAF_ECP_PROTOCOL_ERR) {
		ret = -EIO;
		LOG_ERR("TAF RPMC OP2: Bus Monitor: status=0x%x\n", ev.evt_data);
	} else {
		ret = -ENOSYS;
		LOG_ERR("Unexpected TAF RPMC OP2 callback data\n");
	}

	return ret;
}
#else
static int espi_taf_rpmc_req_counter(uint8_t counter_id, uint8_t *rpmc_ext_status)
{
	int ret = 0;

	if ((counter_id > RPMC_MAX_COUNTERS) || !rpmc_ext_status) {
		return -EINVAL;
	}

	ret = rpmc_req_counter_msg(counter_id, taf_rpmc_buf, sizeof(taf_rpmc_buf));
	if (ret) {
		LOG_ERR("SAF RPMC generate Request Counter message error %d", ret);
		return ret;
	}

	LOG_HEXDUMP_INF((const uint8_t *)taf_rpmc_buf, RPMC_OP1_CMD_REQ_COUNTER_SIZE,
			"RPMC Request Monotonic Counter packet");

	ret = taf_send_rpmc_op1(taf_rpmc_buf, RPMC_OP1_CMD_REQ_COUNTER_SIZE,
				ESPI_TAF_SUBCMD_RPMC_OP1_REQUEST_COUNTER);
	if (ret) {
		LOG_ERR("TAF send RPMC OP1 error (%d)", ret);
		return ret;
	}

	ret = taf_send_rpmc_op2(rpmc_ext_status);
	if (ret) {
		LOG_ERR("TAF send RPMC OP2 error (%d)", ret);
	}

	pr_rpmc_op2_ext_status(*rpmc_ext_status);

	return ret;
}
#endif

#ifdef ESPI_TAF_RPMC_TEST_INCR_COUNTER
/* Transmit RPMC OP1 Increment Monotonic Counter packet followed by RPMC OP2 Read Status command.
 * Software assembles packet: Total packet length = 40 bytes
 *    b[0] = OP1 Opcode
 *    b[1] = CmdType = 0x02 (Increment Monotonic Counter)
 *    b[2] = CounterAddress (0, 1, 2, or 3. Depends upon how many counters implemented in the flash)
 *    b[3] = 0x00 (reserved)
 *    b[4 thru 7] = Previous counter value
 *    b[8 thru 39] = 256-bit Signature = HMAC-SHA256 where key = HMAC_key[counter_id] and
 *    message = OP1 Opcode || CmdType || CounterAddress || 0x00 || PrevCounterData
 * Call TAF driver ECP API with packet size = 40 bytes
 * TAF Controller will:
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
 *   Call TAF driver ECP API to send OP2 Read command and read status/data. TAF will return
 *   last cached OP2 read from above HW polling loop.
 * Check returned RPMC Extended Status byte (first byte read).
 * If Extended Status == 0x80 then command was successfully completed. Other fields in
 * OP2 Read Status are not valid for OP1 Increment Monotonic Counter.
 * SAF HW reads a full 64 bytes. Software can request reading 1 to 64 of these bytes that
 * were cached in HW by SAF controller during the above HW OP2 polling loop.
 */

#define RPMC_OP2_CMD_READ_STS_DATA_SIZE 49u

#if 0
static int espi_taf_rpmc_incr_counter(uint8_t counter_id, uint32_t prev_counter_value)
{
	const struct device *taf_dev = app_get_taf_device();
	struct rpmc_key_info *pki = NULL;
	struct espi_taf_rpmc_packet pkt = { 0 };
	struct espi_event ev = {0};
	int ret = 0;

	if (counter_id >= RPMC_MAX_COUNTERS) {
		return -EINVAL;
	}

	pki = &rpmc_keys[counter_id];

	ret = rpmc_incr_counter_msg(counter_id, prev_counter_value,
				    taf_rpmc_buf, sizeof(taf_rpmc_buf));
	if (ret) {
		LOG_ERR("TAF RPMC generate RPMC Increment Monotonic Counter message error %d",
			ret);
		return ret;
	}

	LOG_HEXDUMP_INF((const uint8_t *)taf_rpmc_buf, RPMC_OP1_CMD_INCR_COUNTER_SIZE,
			"RPMC Increment Monotonic Counter packet");

	espi_taf_cb_data_init();

	pkt.buf = taf_rpmc_buf;
	pkt.len = ESPI_TAF_RPMC_OP1_CMD_INCR_COUNTER_SIZE;
	pkt.cmd = ESPI_TAF_CMD_RPMC_OP1;
	pkt.subcmd = ESPI_TAF_SUBCMD_RPMC_OP1_INCR_COUNTER;
	pkt.cs = 0;

	LOG_INF("Use TAF EC Portal to send RPMC Increment Monotonic Counter packet");

	ret = espi_taf_mchp_rpmc_operation(taf_dev, &pkt, true);
	if (ret) {
		LOG_INF("%s: OP1 CmdType %d error %d",
			__func__, ESPI_TAF_SUBCMD_RPMC_OP1_INCR_COUNTER, ret);
		return ret;
	}

	while (!espi_taf_cb_has_fired())
		;

	espi_taf_cb_get_event(&ev);
	LOG_INF("eSPI TAF callback info: %u : 0x%0x : 0x%0x", ev.evt_type, ev.evt_details,
		ev.evt_data);

	LOG_HEXDUMP_INF((const uint8_t *)taf_rpmc_buf, sizeof(taf_rpmc_buf), "taf_rpmc_buf");

	ret = -EIO;
	if (ev.evt_details & ESPI_TAF_ECP_RPMC_OP1) {
		ret = 0;
		LOG_INF("TAF RPMC OP1 Update HMAC Key: ECP Done: status=0x%x\n", ev.evt_data);
	} else if (ev.evt_details & ESPI_TAF_ECP_PROTOCOL_ERR) {
		LOG_ERR("TAF RPMC OP1 Update HMAC Key: Bus Monitor: status=0x%x\n", ev.evt_data);
	} else {
		LOG_ERR("Unexpected TAF RPMC OP1 callback data\n");
	}

	if (ret) {
		return ret;
	}

	/* Send RPMC OP2 Read Status/Data */
	/* encode: CS=0, SAF EC Portal OP2 cmd, RPMC command = 0 (reserved) */
	memset(taf_rpmc_buf2, 0, sizeof(taf_rpmc_buf2));

	espi_taf_cb_data_init();

	/* TX: OP2 || 0x00, RD: ExtStatusByte || TAG(12-bytes) || CounterReadData(4 bytes)
	 *     || Signature(32 bytes)
	 */
	pkt.buf = taf_rpmc_buf2;
	pkt.len = RPMC_OP2_CMD_READ_STS_DATA_SIZE + 2U;
	pkt.cmd = ESPI_TAF_CMD_RPMC_OP2;
	pkt.subcmd = 0;
	pkt.cs = 0;

	ret = espi_taf_mchp_rpmc_operation(taf_dev, &pkt, true);
	if (ret) {
		LOG_INF("%s: OP2 error %d", __func__, ret);
		return ret;
	}

	while (!espi_taf_cb_has_fired())
		;

	espi_taf_cb_get_event(&ev);
	LOG_INF("eSPI TAF callback info: %u : 0x%0x : 0x%0x", ev.evt_type, ev.evt_details,
		ev.evt_data);

	LOG_HEXDUMP_INF((const uint8_t *)taf_rpmc_buf2, sizeof(taf_rpmc_buf2), "taf_rpmc_buf2");

	ret = -EIO;
	if (ev.evt_details & ESPI_TAF_ECP_RPMC_OP1) {
		ret = 0;
		LOG_INF("TAF RPMC OP2: ECP Done: status=0x%x\n", ev.evt_data);
	} else if (ev.evt_details & ESPI_TAF_ECP_PROTOCOL_ERR) {
		ret = -EIO;
		LOG_ERR("TAF RPMC OP2: Bus Monitor: status=0x%x\n", ev.evt_data);
	} else {
		ret = -ENOSYS;
		LOG_ERR("Unexpected TAF RPMC OP2 callback data\n");
	}

	return ret;
}
#else
static int espi_taf_rpmc_incr_counter(uint8_t counter_id, uint32_t prev_counter_value,
				      uint8_t *rpmc_ext_status)
{
	int ret = 0;

	if ((counter_id >= RPMC_MAX_COUNTERS) || !rpmc_ext_status) {
		return -EINVAL;
	}

	ret = rpmc_incr_counter_msg(counter_id, prev_counter_value,
				    taf_rpmc_buf, sizeof(taf_rpmc_buf));
	if (ret) {
		LOG_ERR("TAF RPMC generate RPMC Increment Monotonic Counter msg error %d", ret);
		return ret;
	}

	LOG_HEXDUMP_INF((const uint8_t *)taf_rpmc_buf, RPMC_OP1_CMD_INCR_COUNTER_SIZE,
			"RPMC Increment Monotonic Counter packet");

	ret = taf_send_rpmc_op1(taf_rpmc_buf, RPMC_OP1_CMD_INCR_COUNTER_SIZE,
				ESPI_TAF_SUBCMD_RPMC_OP1_INCR_COUNTER);
	if (ret) {
		LOG_ERR("TAF send RPMC OP1 Incr Counter error (%d)", ret);
		return ret;
	}

	ret = taf_send_rpmc_op2(rpmc_ext_status);
	if (ret) {
		LOG_ERR("TAF send RPMC OP2 error (%d)", ret);
	}

	pr_rpmc_op2_ext_status(*rpmc_ext_status);

	return ret;
}
#endif
#endif /* ESPI_TAF_RPMC_TEST_INCR_COUNTER */

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

/* Write test RPMC Root Key (all one's) to RPMC SPI flash using TAF driver
 * Requires rpmc_init_keys to have been called.
 * 1. Build Write Root Key message (64 bytes)
 * 2. Transmit message via SPI driver
 * 3. Begin polling loop
 *      Send RPMC OP2 Read Status command and read response via Zephyr SPI driver
 *    Loop until RPMC Extended Status indicates Done.
 * 4. Check Extended Status is Successful Completion
 */
/* Use eSPI TAF to check keys */
int taf_rpmc_check_root_key(uint8_t rpmc_counter_id, uint8_t *rpmc_ext_status)
{
	int ret = 0;

	if ((rpmc_counter_id > 3) || !rpmc_ext_status) {
		return -EINVAL;
	}

	memset(taf_rpmc_buf, 0, sizeof(taf_rpmc_buf));

	ret = rpmc_set_root_key_msg(rpmc_counter_id, taf_rpmc_buf, 64U);
	if (ret) {
		LOG_ERR("RPMC generate root key message error %d", ret);
		return ret;
	}

	LOG_HEXDUMP_INF((const uint8_t *)taf_rpmc_buf, RPMC_OP1_CMD_SET_ROOT_KEY_SIZE,
			"RPMC HMAC Update Key packet");

	ret = taf_send_rpmc_op1(taf_rpmc_buf, RPMC_OP1_CMD_SET_ROOT_KEY_SIZE,
				ESPI_TAF_SUBCMD_RPMC_OP1_SET_ROOT_KEY);
	if (ret) {
		LOG_ERR("TAF Send RPMC OP1 returned error (%d)", ret);
		return ret;
	}

	ret = taf_send_rpmc_op2(rpmc_ext_status);

	pr_rpmc_op2_ext_status(*rpmc_ext_status);

	return ret;
}

int taf_rpmc_check_root_keys(uint8_t num_rpmc_counters)
{
	uint8_t fail_count = 0;

	for (uint8_t counter_id = 0; counter_id < num_rpmc_counters; counter_id++) {
		uint8_t rpmc_ext_status = 0;
		int ret = taf_rpmc_check_root_key(counter_id, &rpmc_ext_status);

		if (ret) {
			LOG_ERR("SPI RPMC check root key for counter %u error %d",
				counter_id, ret);
		}

		if (rpmc_ext_status == 0x80u) {
			LOG_INF("Check Root Key %u: PASS", counter_id);
		} else {
			fail_count++;
			LOG_INF("Check Root Key %u: FAIL", counter_id);
		}
	}

	if (fail_count) {
		return -EIO;
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

	if (!counter || !rpmc_mesg || rpmc_mesg_size != ESPI_TAF_RPMC_OP1_CMD_REQ_COUNTER_SIZE) {
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
int espi_taf_rpmc_test1(void)
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

	ret = espi_taf_rpmc_update_hmac_key(counter_id, (uint8_t *)&ext_status);
	if (ret) {
		LOG_ERR("eSPI TAF RPMC test1: Update HMAC key error %d", ret);
		return ret;
	}

	ret = espi_taf_rpmc_req_counter(counter_id, (uint8_t *)&ext_status);
	if (ret) {
		LOG_ERR("eSPI TAF RPMC test1: Request Monotinic Counter error %d", ret);
		return ret;
	}

	/* taf_rpmc_buf2 contains status, TAG[12], counterData[4], and Signature[32]
	 * We should authenticate the message before using the new counterData.
	 * Compute HMAC-SHA256 where key = HMAC_Key[counter_id] and
	 * message = TAG || counterData
	 */
	if (ext_status != RPMC_EXT_STATUS_OP1_SUCCESS) {
		LOG_ERR("eSPI TAF RPMC Request Counter OP1 status error 0x%02x", ext_status);
		return -EIO;
	}

	initial_counter = 0xffffffffu;
	ret = extract_req_counter((const uint8_t *)taf_rpmc_buf2,
				  RPMC_OP1_CMD_REQ_COUNTER_SIZE, &initial_counter);
	if (ret) {
		LOG_ERR("eSPI TAF RPMC test1: extract counter from RPMC packet error %d", ret);
		return ret;
	}

	LOG_INF("eSPI TAF RPMC Request Monotonic Counter %d: Status=0x%x", counter_id, ext_status);
	LOG_INF("Counter value: 0x%04x", initial_counter);
	LOG_HEXDUMP_INF((const uint8_t *)&taf_rpmc_buf2[1], 12, "TAG");
	LOG_HEXDUMP_INF((const uint8_t *)&taf_rpmc_buf2[17], 32, "Signature");

	ret = app_hmac_sha256_mesg((const uint8_t *)pki->hmac_key, 32,
				   &taf_rpmc_buf2[1], 16, hmacd, 32U);
	if (ret) {
		LOG_ERR("HMAC-SHA256 error %d", ret);
		return ret;
	}

	LOG_INF("Key ID: %u", counter_id);
	LOG_HEXDUMP_INF((const uint8_t *)hmacd, 32, "Calculated signature using HMAC key");

	ret = memcmp(&taf_rpmc_buf2[17], hmacd, 32);
	if (ret) {
		LOG_INF("TAF RPMC Request Monotonic Counter packet authenticated: FAIL\n");
		return ret;
	} else {
		LOG_INF("TAF RPMC Request Monotonic Counter packet authenticated: PASS\n");
	}

	LOG_INF("Current Monotonic Counter = 0x%04x", initial_counter);
	LOG_INF("Send RPMC Increment Monotonic Counter command packet");

	ext_status = 0u;
	ret = espi_taf_rpmc_incr_counter(counter_id, initial_counter, (uint8_t *)&ext_status);
	if (ret) {
		LOG_ERR("eSPI TAF RPMC test1: Increment Monotinic Counter error %d", ret);
		return ret;
	}

	if (ext_status != RPMC_EXT_STATUS_OP1_SUCCESS) {
		LOG_ERR("eSPI TAF RPMC Increment Counter OP1 status error 0x%02x", ext_status);
		return -EIO;
	}

	LOG_INF("After increment read monotonic counter from RPMC flash");

	ext_status = 0u;
	ret = espi_taf_rpmc_req_counter(counter_id, (uint8_t *)&ext_status);
	if (ret) {
		LOG_ERR("eSPI TAF RPMC test1: Request Monotinic Counter error %d", ret);
		return ret;
	}

	if (ext_status != RPMC_EXT_STATUS_OP1_SUCCESS) {
		LOG_ERR("eSPI TAF RPMC Request Counter OP1 status error 0x%02x", ext_status);
		return -EIO;
	}

	current_counter = 0xffffffffu;
	ret = extract_req_counter((const uint8_t *)taf_rpmc_buf2,
				  RPMC_OP1_CMD_REQ_COUNTER_SIZE, &current_counter);
	if (ret) {
		LOG_ERR("eSPI TAF RPMC test1: extract counter from RPMC packet error %d", ret);
		return ret;
	}

	LOG_INF("eSPI TAF RPMC Request Monotonic Counter %d: Status=0x%x", counter_id, ext_status);
	LOG_INF("Counter value: 0x%04x", current_counter);
	LOG_HEXDUMP_INF((const uint8_t *)&taf_rpmc_buf2[1], 12, "TAG");
	LOG_HEXDUMP_INF((const uint8_t *)&taf_rpmc_buf2[17], 32, "Signature");

	ret = app_hmac_sha256_mesg((const uint8_t *)pki->hmac_key, 32,
				   &taf_rpmc_buf2[1], 16, hmacd, 32U);
	if (ret) {
		LOG_ERR("HMAC-SHA256 error %d", ret);
		return ret;
	}

	LOG_INF("Calculated signature using HMAC Key[%d]", counter_id);
	LOG_HEXDUMP_INF((const uint8_t *)hmacd, 32, "Signature");

	ret = memcmp(&taf_rpmc_buf2[17], hmacd, 32);
	if (ret) {
		LOG_INF("TAF RPMC Request Monotonic Counter packet authenticated: FAIL");
		return ret;
	} else {
		LOG_INF("TAF RPMC Request Monotonic Counter packet authenticated: PASS");
	}

	return ret;
}
