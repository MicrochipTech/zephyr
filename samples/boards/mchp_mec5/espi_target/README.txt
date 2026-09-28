eSPI Target sample using the in-tree Microchip XEC V2 eSPI driver.

Runs on mec_assy6941/mec1753_qsz as the eSPI Target for
samples/boards/mchp_mec5/espi_host_emu on a second board.

Host emulator tests not covered by the XEC V2 driver (expected to fail):
- ACPI EC4 I/O 0x340 and memory 0x10002000 - 0x10002005
