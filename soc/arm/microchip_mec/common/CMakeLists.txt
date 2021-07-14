# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_SERIES_MEC1501X OR CONFIG_SOC_SERIES_MEC172X)
  zephyr_include_directories(.)
  zephyr_library_sources(
    soc_ecia.c
  )
endif()
