# SPDX-License-Identifier: Apache-2.0

# Zephyr SDK >= 1.0 moved the GNU toolchain cmake under a gnu/ subdirectory.
# Prefer it when present; fall back to the flat layout for SDK < 1.0 (e.g. 0.17.4).
if(EXISTS ${ZEPHYR_SDK_INSTALL_DIR}/cmake/zephyr/gnu/generic.cmake)
  include(${ZEPHYR_SDK_INSTALL_DIR}/cmake/zephyr/gnu/generic.cmake)
else()
  include(${ZEPHYR_SDK_INSTALL_DIR}/cmake/zephyr/generic.cmake)
endif()

set(TOOLCHAIN_KCONFIG_DIR ${ZEPHYR_SDK_INSTALL_DIR}/cmake/zephyr)

message(STATUS "Found toolchain: zephyr ${SDK_VERSION} (${ZEPHYR_SDK_INSTALL_DIR})")
