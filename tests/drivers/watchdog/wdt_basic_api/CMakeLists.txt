# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wdt_basic_api)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

zephyr_linker_sources_ifdef(CONFIG_DT_HAS_MICROCHIP_MEC5_WATCHDOG_ENABLED
	SECTIONS microchip_mec5_noinit.ld)
