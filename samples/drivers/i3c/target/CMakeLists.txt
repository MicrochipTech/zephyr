cmake_minimum_required(VERSION 3.20.0)

# DTS_ROOT must be set before find_package(Zephyr) for binding discovery.
list(APPEND DTS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../common)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(i3c_target)

target_sources(app PRIVATE
    src/main.c
    src/target.c
)

target_include_directories(app PRIVATE ../common/src)
