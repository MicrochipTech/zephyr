# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_ESPI_VWIRE_CHANNEL		espi_vw_chan_mec5.c)
zephyr_library_sources_ifdef(CONFIG_ESPI_OOB_CHANNEL		espi_oob_chan_mec5.c)
zephyr_library_sources_ifdef(CONFIG_ESPI_FLASH_CHANNEL		espi_fc_chan_mec5.c)

if(CONFIG_ESPI_PERIPHERAL_CHANNEL)
	zephyr_library_sources(espi_pc_chan_mec5.c)
	zephyr_library_sources_ifdef(CONFIG_ESPI_MEC5_ACPI_EC	espi_pc_acpi_ec_mec5.c)
	zephyr_library_sources_ifdef(CONFIG_ESPI_MEC5_EMI	espi_pc_emi_mec5.c)
endif()
