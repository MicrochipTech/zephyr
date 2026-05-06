cmake_minimum_required(VERSION 3.20.0)

# DTS_ROOT must be set before find_package(Zephyr) for binding discovery.
list(APPEND DTS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../common)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(i3c_loopback)

# Reuse host.c and target.c from the standalone apps; no duplication.
target_sources(app PRIVATE
    src/main.c
    ../host/src/host.c
    ../target/src/target.c
    ../common/src/crc8.c
    ../common/src/i3c_fake_target.c
)

target_include_directories(app PRIVATE ../common/src)
