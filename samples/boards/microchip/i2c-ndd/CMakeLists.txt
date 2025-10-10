# Copyright (c) 2025 Microchip Technology Inc.
#
# SPDX-License-Identifier: Apache-2.0
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mec_app_i2c_ndd)

target_sources(app PRIVATE src/main.c)
target_sources_ifdef(CONFIG_I2C_TARGET app PRIVATE src/target_tests.c)
