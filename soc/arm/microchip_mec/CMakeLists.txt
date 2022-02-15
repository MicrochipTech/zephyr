# SPDX-License-Identifier: Apache-2.0

add_subdirectory(${SOC_SERIES})
zephyr_include_directories_ifdef(CONFIG_SOC_SERIES_MEC172X common)

