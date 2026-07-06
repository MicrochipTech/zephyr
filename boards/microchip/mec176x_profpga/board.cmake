# (c) 2026 Microchip Technology Inc. and its subsidiaries.
# SPDX-License-Identifier: Apache-2.0
#
# Mount Rainier is a flashless / RAM-execution FPGA target: there is no flash bank.
# Programming = loading the image into code SRAM (@0x00080000) over JTAG and setting
# VTOR/PC/SP via GDB. The ready-made OpenOCD config lives at the workspace root:
#     openocd -f .openocd/profpga_mec1767.cfg
# `west flash` is therefore not the primary path here; this include keeps the
# openocd runner available for debug attach.

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
