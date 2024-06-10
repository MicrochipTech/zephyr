# SPDX-License-Identifier: Apache-2.0

#zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/introspect_i3c.h)

#zephyr_library()

zephyr_library_sources_ifdef(CONFIG_INTROSPECT	introspect.c)
