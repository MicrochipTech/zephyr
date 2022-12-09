/*
 * Copyright (c) 2022 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_MICROCHIP_XEC_GPIO_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_MICROCHIP_XEC_GPIO_H_

/**
 * @brief Microchip XEC GPIO bank and bit position convenience defines
 *
 * Microchip XEC documentation uses octal GPIO pin
 * numbering. These macros do not require the user to do octal arithmetic
 * to derive the pin's bit postion.
 *
 * Example DT usage: gpios = <&MCHP_GPIO176 GPIO_ACTIVE_HIGH>;
 *
 * @{
 */

/* bank A */
#define MCHP_GPIO000 gpio_000_036 0
#define MCHP_GPIO001 gpio_000_036 1
#define MCHP_GPIO002 gpio_000_036 2
#define MCHP_GPIO003 gpio_000_036 3
#define MCHP_GPIO004 gpio_000_036 4
#define MCHP_GPIO005 gpio_000_036 5
#define MCHP_GPIO006 gpio_000_036 6
#define MCHP_GPIO007 gpio_000_036 7
#define MCHP_GPIO010 gpio_000_036 8
#define MCHP_GPIO011 gpio_000_036 9
#define MCHP_GPIO012 gpio_000_036 10
#define MCHP_GPIO013 gpio_000_036 11
#define MCHP_GPIO014 gpio_000_036 12
#define MCHP_GPIO015 gpio_000_036 13
#define MCHP_GPIO016 gpio_000_036 14
#define MCHP_GPIO017 gpio_000_036 15
#define MCHP_GPIO020 gpio_000_036 16
#define MCHP_GPIO021 gpio_000_036 17
#define MCHP_GPIO022 gpio_000_036 18
#define MCHP_GPIO023 gpio_000_036 19
#define MCHP_GPIO024 gpio_000_036 20
#define MCHP_GPIO025 gpio_000_036 21
#define MCHP_GPIO026 gpio_000_036 22
#define MCHP_GPIO027 gpio_000_036 23
#define MCHP_GPIO030 gpio_000_036 24
#define MCHP_GPIO031 gpio_000_036 25
#define MCHP_GPIO032 gpio_000_036 26
#define MCHP_GPIO033 gpio_000_036 27
#define MCHP_GPIO034 gpio_000_036 28
#define MCHP_GPIO035 gpio_000_036 29
#define MCHP_GPIO036 gpio_000_036 30

/* bank B */
#define MCHP_GPIO040 gpio_040_076 0
#define MCHP_GPIO041 gpio_040_076 1
#define MCHP_GPIO042 gpio_040_076 2
#define MCHP_GPIO043 gpio_040_076 3
#define MCHP_GPIO044 gpio_040_076 4
#define MCHP_GPIO045 gpio_040_076 5
#define MCHP_GPIO046 gpio_040_076 6
#define MCHP_GPIO047 gpio_040_076 7
#define MCHP_GPIO050 gpio_040_076 8
#define MCHP_GPIO051 gpio_040_076 9
#define MCHP_GPIO052 gpio_040_076 10
#define MCHP_GPIO053 gpio_040_076 11
#define MCHP_GPIO054 gpio_040_076 12
#define MCHP_GPIO055 gpio_040_076 13
#define MCHP_GPIO056 gpio_040_076 14
#define MCHP_GPIO057 gpio_040_076 15
#define MCHP_GPIO060 gpio_040_076 16
#define MCHP_GPIO061 gpio_040_076 17
#define MCHP_GPIO062 gpio_040_076 18
#define MCHP_GPIO063 gpio_040_076 19
#define MCHP_GPIO064 gpio_040_076 20
#define MCHP_GPIO065 gpio_040_076 21
#define MCHP_GPIO066 gpio_040_076 22
#define MCHP_GPIO067 gpio_040_076 23
#define MCHP_GPIO070 gpio_040_076 24
#define MCHP_GPIO071 gpio_040_076 25
#define MCHP_GPIO072 gpio_040_076 26
#define MCHP_GPIO073 gpio_040_076 27
#define MCHP_GPIO074 gpio_040_076 28
#define MCHP_GPIO075 gpio_040_076 29
#define MCHP_GPIO076 gpio_040_076 30

/* bank C */
#define MCHP_GPIO100 gpio_100_136 0
#define MCHP_GPIO101 gpio_100_136 1
#define MCHP_GPIO102 gpio_100_136 2
#define MCHP_GPIO103 gpio_100_136 3
#define MCHP_GPIO104 gpio_100_136 4
#define MCHP_GPIO105 gpio_100_136 5
#define MCHP_GPIO106 gpio_100_136 6
#define MCHP_GPIO107 gpio_100_136 7
#define MCHP_GPIO110 gpio_100_136 8
#define MCHP_GPIO111 gpio_100_136 9
#define MCHP_GPIO112 gpio_100_136 10
#define MCHP_GPIO113 gpio_100_136 11
#define MCHP_GPIO114 gpio_100_136 12
#define MCHP_GPIO115 gpio_100_136 13
#define MCHP_GPIO116 gpio_100_136 14
#define MCHP_GPIO117 gpio_100_136 15
#define MCHP_GPIO120 gpio_100_136 16
#define MCHP_GPIO121 gpio_100_136 17
#define MCHP_GPIO122 gpio_100_136 18
#define MCHP_GPIO123 gpio_100_136 19
#define MCHP_GPIO124 gpio_100_136 20
#define MCHP_GPIO125 gpio_100_136 21
#define MCHP_GPIO126 gpio_100_136 22
#define MCHP_GPIO127 gpio_100_136 23
#define MCHP_GPIO130 gpio_100_136 24
#define MCHP_GPIO131 gpio_100_136 25
#define MCHP_GPIO132 gpio_100_136 26
#define MCHP_GPIO133 gpio_100_136 27
#define MCHP_GPIO134 gpio_100_136 28
#define MCHP_GPIO135 gpio_100_136 29
#define MCHP_GPIO136 gpio_100_136 30

/* bank D */
#define MCHP_GPIO140 gpio_140_176 0
#define MCHP_GPIO141 gpio_140_176 1
#define MCHP_GPIO142 gpio_140_176 2
#define MCHP_GPIO143 gpio_140_176 3
#define MCHP_GPIO144 gpio_140_176 4
#define MCHP_GPIO145 gpio_140_176 5
#define MCHP_GPIO146 gpio_140_176 6
#define MCHP_GPIO147 gpio_140_176 7
#define MCHP_GPIO150 gpio_140_176 8
#define MCHP_GPIO151 gpio_140_176 9
#define MCHP_GPIO152 gpio_140_176 10
#define MCHP_GPIO153 gpio_140_176 11
#define MCHP_GPIO154 gpio_140_176 12
#define MCHP_GPIO155 gpio_140_176 13
#define MCHP_GPIO156 gpio_140_176 14
#define MCHP_GPIO157 gpio_140_176 15
#define MCHP_GPIO160 gpio_140_176 16
#define MCHP_GPIO161 gpio_140_176 17
#define MCHP_GPIO162 gpio_140_176 18
#define MCHP_GPIO163 gpio_140_176 19
#define MCHP_GPIO164 gpio_140_176 20
#define MCHP_GPIO165 gpio_140_176 21
#define MCHP_GPIO166 gpio_140_176 22
#define MCHP_GPIO167 gpio_140_176 23
#define MCHP_GPIO170 gpio_140_176 24
#define MCHP_GPIO171 gpio_140_176 25
#define MCHP_GPIO172 gpio_140_176 26
#define MCHP_GPIO173 gpio_140_176 27
#define MCHP_GPIO174 gpio_140_176 28
#define MCHP_GPIO175 gpio_140_176 29
#define MCHP_GPIO176 gpio_140_176 30

/* bank E */
#define MCHP_GPIO200 gpio_200_236 0
#define MCHP_GPIO201 gpio_200_236 1
#define MCHP_GPIO202 gpio_200_236 2
#define MCHP_GPIO203 gpio_200_236 3
#define MCHP_GPIO204 gpio_200_236 4
#define MCHP_GPIO205 gpio_200_236 5
#define MCHP_GPIO206 gpio_200_236 6
#define MCHP_GPIO207 gpio_200_236 7
#define MCHP_GPIO210 gpio_200_236 8
#define MCHP_GPIO211 gpio_200_236 9
#define MCHP_GPIO212 gpio_200_236 10
#define MCHP_GPIO213 gpio_200_236 11
#define MCHP_GPIO214 gpio_200_236 12
#define MCHP_GPIO215 gpio_200_236 13
#define MCHP_GPIO216 gpio_200_236 14
#define MCHP_GPIO217 gpio_200_236 15
#define MCHP_GPIO220 gpio_200_236 16
#define MCHP_GPIO221 gpio_200_236 17
#define MCHP_GPIO222 gpio_200_236 18
#define MCHP_GPIO223 gpio_200_236 19
#define MCHP_GPIO224 gpio_200_236 20
#define MCHP_GPIO225 gpio_200_236 21
#define MCHP_GPIO226 gpio_200_236 22
#define MCHP_GPIO227 gpio_200_236 23
#define MCHP_GPIO230 gpio_200_236 24
#define MCHP_GPIO231 gpio_200_236 25
#define MCHP_GPIO232 gpio_200_236 26
#define MCHP_GPIO233 gpio_200_236 27
#define MCHP_GPIO234 gpio_200_236 28
#define MCHP_GPIO235 gpio_200_236 29
#define MCHP_GPIO236 gpio_200_236 30

/* bank F */
#define MCHP_GPIO240 gpio_240_276 0
#define MCHP_GPIO241 gpio_240_276 1
#define MCHP_GPIO242 gpio_240_276 2
#define MCHP_GPIO243 gpio_240_276 3
#define MCHP_GPIO244 gpio_240_276 4
#define MCHP_GPIO245 gpio_240_276 5
#define MCHP_GPIO246 gpio_240_276 6
#define MCHP_GPIO247 gpio_240_276 7
#define MCHP_GPIO250 gpio_240_276 8
#define MCHP_GPIO251 gpio_240_276 9
#define MCHP_GPIO252 gpio_240_276 10
#define MCHP_GPIO253 gpio_240_276 11
#define MCHP_GPIO254 gpio_240_276 12
#define MCHP_GPIO255 gpio_240_276 13
#define MCHP_GPIO256 gpio_240_276 14
#define MCHP_GPIO257 gpio_240_276 15
#define MCHP_GPIO260 gpio_240_276 16
#define MCHP_GPIO261 gpio_240_276 17
#define MCHP_GPIO262 gpio_240_276 18
#define MCHP_GPIO263 gpio_240_276 19
#define MCHP_GPIO264 gpio_240_276 20
#define MCHP_GPIO265 gpio_240_276 21
#define MCHP_GPIO266 gpio_240_276 22
#define MCHP_GPIO267 gpio_240_276 23
#define MCHP_GPIO270 gpio_240_276 24
#define MCHP_GPIO271 gpio_240_276 25
#define MCHP_GPIO272 gpio_240_276 26
#define MCHP_GPIO273 gpio_240_276 27
#define MCHP_GPIO274 gpio_240_276 28
#define MCHP_GPIO275 gpio_240_276 29
#define MCHP_GPIO276 gpio_240_276 30

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_MICROCHIP_XEC_GPIO_H_ */
