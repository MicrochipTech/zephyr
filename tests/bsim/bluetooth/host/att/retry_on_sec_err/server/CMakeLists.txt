# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(app)

add_subdirectory(${ZEPHYR_BASE}/tests/bsim/babblekit babblekit)
add_subdirectory(${ZEPHYR_BASE}/tests/bluetooth/common/testlib testlib)

target_sources(app PRIVATE
  main.c
)

zephyr_include_directories(
  ${BSIM_COMPONENTS_PATH}/libPhyComv1/src/
  ${BSIM_COMPONENTS_PATH}/libUtilv1/src/
)

target_link_libraries(app PRIVATE
  testlib
  babblekit
)
