#
# Copyright (c) 2019, Microchip Technology Inc.
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_sources(soc.c)
zephyr_include_directories(.)

zephyr_sources_ifdef(CONFIG_PM
  device_power.c
  power.c
  )

# This method allows forcing use of SoC timing functions by setting
# CONFIG_MCHP_MEC_SOC_HAS_TIMING_FUNCTIONS=y. Both arch and SoC timing functions will be
# built but the timing subsys module will only call the SoC functions.
# CONFIG_MCHP_MEC_SOC_HAS_TIMING_FUNCTIONS defaults to "n" resulting in the arch Cortex-M4 DWT
# timing functions being built if CONFIG_TIMING_FUNCTIONS=y.
if(CONFIG_TIMING_FUNCTIONS)
  if(CONFIG_SOC_HAS_TIMING_FUNCTIONS AND NOT CONFIG_BOARD_HAS_TIMING_FUNCTIONS)
    zephyr_library_sources(timing.c)
  endif()
endif()

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
