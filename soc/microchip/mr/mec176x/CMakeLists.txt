# (c) 2026 Microchip Technology Inc. and its subsidiaries.
# SPDX-License-Identifier: Apache-2.0
#
# Mount Rainier is an XEC/MEC-family derivative, so we REUSE the upstream MEC
# common SoC helpers (ECIA interrupt aggregator + PCR clock-gate) by reference
# instead of forking them. They are self-contained: topology constants live in
# their own headers and register bases come from DT nodelabels (pcr/ecia/ecs).
# Upstream zephyr/ is never edited.

set(MEC_COMMON ${ZEPHYR_BASE}/soc/microchip/mec/common)

zephyr_include_directories(.)
zephyr_include_directories(${MEC_COMMON})

zephyr_library_sources(soc.c)
zephyr_library_sources(${MEC_COMMON}/soc_ecia.c)
zephyr_library_sources(${MEC_COMMON}/soc_pcr.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
