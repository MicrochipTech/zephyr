/*
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODEM_UBX_KEYS_
#define ZEPHYR_MODEM_UBX_KEYS_

enum ubx_keys_msg_out {
	UBX_KEY_MSG_OUT_NMEA_GGA_UART1 = 0x209100bb,
	UBX_KEY_MSG_OUT_NMEA_RMC_UART1 = 0x209100ac,
	UBX_KEY_MSG_OUT_NMEA_GSV_UART1 = 0x209100c5,
	UBX_KEY_MSG_OUT_NMEA_DTM_UART1 = 0x209100a7,
	UBX_KEY_MSG_OUT_NMEA_GBS_UART1 = 0x209100de,
	UBX_KEY_MSG_OUT_NMEA_GLL_UART1 = 0x209100ca,
	UBX_KEY_MSG_OUT_NMEA_GNS_UART1 = 0x209100b6,
	UBX_KEY_MSG_OUT_NMEA_GRS_UART1 = 0x209100cf,
	UBX_KEY_MSG_OUT_NMEA_GSA_UART1 = 0x209100c0,
	UBX_KEY_MSG_OUT_NMEA_GST_UART1 = 0x209100d4,
	UBX_KEY_MSG_OUT_NMEA_VTG_UART1 = 0x209100b1,
	UBX_KEY_MSG_OUT_NMEA_VLW_UART1 = 0x209100e8,
	UBX_KEY_MSG_OUT_NMEA_ZDA_UART1 = 0x209100d9,
	UBX_KEY_MSG_OUT_UBX_NAV_PVT_UART1 = 0x20910007,
	UBX_KEY_MSG_OUT_UBX_NAV_SAT_UART1 = 0x20910016,
};

enum ubx_keys_rate {
	UBX_KEY_RATE_MEAS = 0x30210001,
	UBX_KEY_RATE_NAV = 0x30210002,
};

enum ubx_keys_nav_cfg {
	UBX_KEY_NAV_CFG_FIX_MODE = 0x20110011,
	UBX_KEY_NAV_CFG_DYN_MODEL = 0x20110021,
};

#endif /* ZEPHYR_MODEM_UBX_KEYS_ */
