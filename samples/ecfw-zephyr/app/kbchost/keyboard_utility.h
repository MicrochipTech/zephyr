/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Utility APIs for transating keys and hotkeys for
 * the keyscan and ps2 keyboards.
 */


#ifndef __KEYBOARD_UTILITY_H__
#define __KEYBOARD_UTILITY_H__

/* This table is used for translating scan code set 2 to set 1 */
static const uint8_t kb_translation_table[] = {
	0xff, 0xfe, 0xfd, 0xfc, 0xfb, 0xfa, 0xf9, 0xf8,		/* 0 - 7 */
	0xf7, 0xf6, 0xf5, 0xf4, 0xf3, 0xf2, 0xf1, 0xf0,		/* 8 - 15 */
	0xef, 0xee, 0xed, 0xec, 0xeb, 0xea, 0xe9, 0xe8,		/* 16 - 23 */
	0xe7, 0xe6, 0xe5, 0xe4, 0xe3, 0xe2, 0xe1, 0xe0,		/* 24 - 31 */
	0xdf, 0xde, 0xdd, 0xdc, 0xdb, 0xda, 0xd9, 0xd8,		/* 32 - 39 */
	0xd7, 0xd6, 0xd5, 0xd4, 0xd3, 0xd2, 0xd1, 0xd0,		/* 40 - 47 */
	0xcf, 0xce, 0xcd, 0xcc, 0xcb, 0xca, 0xc9, 0xc8,		/* 48 - 55 */
	0xc7, 0xc6, 0xc5, 0xc4, 0xc3, 0xc2, 0xc1, 0xc0,		/* 56 - 63 */
	0xbf, 0xbe, 0xbd, 0xbc, 0xbb, 0xba, 0xb9, 0xb8,		/* 64 - 71 */
	0xb7, 0xb6, 0xb5, 0xb4, 0xb3, 0xb2, 0xb1, 0xb0,		/* 72 - 79 */
	0xaf, 0xae, 0xad, 0xac, 0xab, 0xaa, 0xa9, 0xa8,		/* 80 - 87 */
	0xa7, 0xa6, 0xa5, 0xa4, 0xa3, 0xa2, 0xa1, 0xa0,		/* 88 - 95 */
	0x9f, 0x9e, 0x9d, 0x9c, 0x9b, 0x9a, 0x99, 0x98,		/* 96 - 103 */
	0x97, 0x96, 0x95, 0x94, 0x93, 0x92, 0x91, 0x90,		/* 104 - 111 */
	0x8f, 0x8e, 0x8d, 0x8c, 0x8b, 0x8a, 0x89, 0x88,		/* 112 - 119 */
	0x87, 0x86, 0x85, 0x54, 0x41, 0x82, 0x81, 0x80,		/* 120 - 127 */
	0x54, 0x46, 0x49, 0x37, 0x4a, 0x51, 0x4e, 0x57,		/* 128 - 135 */
	0x45, 0x01, 0x48, 0x4d, 0x4c, 0x50, 0x53, 0x52,		/* 136 - 143 */
	0x6f, 0x7f, 0x7e, 0x47, 0x4b, 0x7d, 0x4f, 0x7c,		/* 144 - 151 */
	0x7b, 0x0e, 0x7a, 0x79, 0x78, 0x77, 0x56, 0x55,		/* 152 - 159 */
	0x76, 0x63, 0x2b, 0x75, 0x1b, 0x1c, 0x36, 0x3a,		/* 160 - 167 */
	0x6e, 0x62, 0x0d, 0x1a, 0x74, 0x28, 0x73, 0x6d,		/* 168 - 175 */
	0x61, 0x0c, 0x19, 0x27, 0x26, 0x35, 0x34, 0x6c,		/* 176 - 183 */
	0x60, 0x0a, 0x0b, 0x18, 0x17, 0x25, 0x33, 0x6b,		/* 184 - 191 */
	0x5f, 0x09, 0x08, 0x16, 0x24, 0x32, 0x72, 0x6a,		/* 192 - 199 */
	0x5e, 0x07, 0x15, 0x22, 0x23, 0x30, 0x31, 0x69,		/* 200 - 207 */
	0x5d, 0x06, 0x13, 0x14, 0x21, 0x2f, 0x39, 0x68,		/* 208 - 215 */
	0x5c, 0x04, 0x05, 0x12, 0x20, 0x2d, 0x2e, 0x67,		/* 216 - 223 */
	0x5b, 0x03, 0x11, 0x1e, 0x1f, 0x2c, 0x71, 0x66,		/* 224 - 231 */
	0x5a, 0x02, 0x10, 0x1d, 0x70, 0x2a, 0x38, 0x65,		/* 232 - 239 */
	0x59, 0x29, 0x0f, 0x3e, 0x40, 0x42, 0x44, 0x64,		/* 240 - 247 */
	0x58, 0x3c, 0x3b, 0x3d, 0x3f, 0x41, 0x43, 0xff		/* 248 - 255 */
};

enum scan_code_set {
	GET_SET_SCANCODE = 0,
	SCAN_CODE_SET1,
	SCAN_CODE_SET2,
	SCAN_CODE_SET3 /* Not supported */
};

int translate_key(enum scan_code_set scan_code, uint8_t *data);

#endif /* __KEYBOARD_UTILITY_H__ */

