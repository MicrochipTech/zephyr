#
# Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
#
# SPDX-License-Identifier: Apache-2.0
#

set(MEC176X_BOARD_SELECTED True)

# Point to the TF-M platform target in the upstream TF-M module
add_subdirectory(${Trusted\ Firmware\ M_SOURCE_DIR}/platform/ext/target/microchip/mec176x mec176x)

install(FILES       ${CMAKE_CURRENT_LIST_DIR}/ns/cpuarch_ns.cmake
        DESTINATION ${INSTALL_PLATFORM_NS_DIR}
        RENAME      cpuarch.cmake)

install(FILES       config.cmake
        DESTINATION ${INSTALL_PLATFORM_NS_DIR})
