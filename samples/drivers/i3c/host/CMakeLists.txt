cmake_minimum_required(VERSION 3.20.0)

# DTS_ROOT must be set before find_package(Zephyr) so the build system can
# locate the microchip,i3c-fake-target binding in common/dts/bindings/.
list(APPEND DTS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../common)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(i3c_host)

target_sources(app PRIVATE
    src/main.c
    src/host.c
    ../common/src/crc8.c
    ../common/src/i3c_fake_target.c
)

target_include_directories(app PRIVATE ../common/src)
