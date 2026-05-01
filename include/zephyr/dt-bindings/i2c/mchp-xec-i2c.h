/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_I2C_MCHP_XEC_I2C_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_I2C_MCHP_XEC_I2C_H_

#define MCHP_XEC_I2C_CTRL_PORT(ctrl, port) (((ctrl & 0xf) << 4) | (port & 0xf))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_I2C_MCHP_XEC_I2C_H_ */
