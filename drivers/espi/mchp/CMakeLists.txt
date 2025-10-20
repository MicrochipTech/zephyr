# Copyright (c) 2025 Microchip Technology Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_ESPI_FLASH_CHANNEL xec_espi_ng_fc.c)
zephyr_library_sources_ifdef(CONFIG_ESPI_OOB_CHANNEL xec_espi_ng_oob.c)
zephyr_library_sources_ifdef(CONFIG_ESPI_PERIPHERAL_CHANNEL xec_espi_ng_pc.c)
zephyr_library_sources_ifdef(CONFIG_ESPI_VWIRE_CHANNEL xec_espi_ng_vw.c)
