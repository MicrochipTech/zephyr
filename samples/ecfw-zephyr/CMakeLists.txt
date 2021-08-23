# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})


## set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
## include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)

project(ecfw)

include(${CMAKE_CURRENT_LIST_DIR}/app/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/boards/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/drivers/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/misc/CMakeLists.txt)

#Note : The target named "app" is the target name expected by Zephyr RTOS
include_directories(ecfw_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

zephyr_compile_options(-Werror)
zephyr_linker_sources(SECTIONS linker/custom-sections.ld)
