#
# Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
#
# SPDX-License-Identifier: Apache-2.0
#

# MEC176x Cortex-M55 architecture settings for non-secure build

set(TFM_SYSTEM_PROCESSOR cortex-m55)
set(TFM_SYSTEM_ARCHITECTURE armv8.1-m.main+mve.fp+fp.dp)
set(CONFIG_TFM_ENABLE_FP ON)
set(CONFIG_TFM_ENABLE_MVE ON)
set(CONFIG_TFM_ENABLE_MVE_FP ON)
