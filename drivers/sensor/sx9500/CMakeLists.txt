# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(sx9500.c)
zephyr_library_sources_ifdef(CONFIG_SX9500_TRIGGER sx9500_trigger.c)
