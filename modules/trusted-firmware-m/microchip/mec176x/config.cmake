#
# Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
#
# SPDX-License-Identifier: Apache-2.0
#

# MEC176x TF-M build configuration

set(TFM_SYSTEM_PROCESSOR cortex-m55)
set(TFM_SYSTEM_ARCHITECTURE armv8.1-m.main)
set(CONFIG_TFM_FLOAT_ABI hard)

# Use standard C library for TF-M secure side
# The toolchain_GNUARM.cmake has been patched to automatically detect
# picolibc availability and fall back to nosys.specs for gnuarmemb
set(CONFIG_TFM_INCLUDE_STDLIBC ON CACHE BOOL "Use standard libc")

# TrustZone configuration - enable NS TrustZone agent partition
# This must be set before tfm_ipc_config_default.cmake is included
set(CONFIG_TFM_USE_TRUSTZONE ON)
set(TFM_MULTI_CORE_TOPOLOGY OFF)

# Use RAM-FS for storage services (no flash on this platform)
set(PS_RAM_FS ON CACHE BOOL "Use RAM FS for Protected Storage")
set(ITS_RAM_FS ON CACHE BOOL "Use RAM FS for Internal Trusted Storage")

# Disable BL2 (MCUboot) for initial bring-up - direct S->NS boot
set(BL2 OFF CACHE BOOL "Disable BL2 bootloader")

# Platform-specific settings
set(PLATFORM_DEFAULT_PROVISIONING OFF CACHE BOOL "Use default provisioning")
set(TFM_DUMMY_PROVISIONING ON CACHE BOOL "Use dummy provisioning for development")

# Enable RAM emulation for OTP/NV counters (no real flash on this platform)
# This uses a RAM buffer instead of calling the Flash driver
set(OTP_NV_COUNTERS_RAM_EMULATION ON CACHE BOOL "Emulate OTP/NV counters in RAM")

# Minimal partition config for 128KB Code SRAM constraint
# Disable all partitions except NS_AGENT_TZ for initial bring-up
set(TFM_PARTITION_CRYPTO OFF CACHE BOOL "Disable crypto partition" FORCE)
set(TFM_PARTITION_INTERNAL_TRUSTED_STORAGE OFF CACHE BOOL "Disable ITS partition" FORCE)
set(TFM_PARTITION_PROTECTED_STORAGE OFF CACHE BOOL "Disable PS partition" FORCE)
set(TFM_PARTITION_PLATFORM OFF CACHE BOOL "Disable platform partition" FORCE)
set(TFM_PARTITION_INITIAL_ATTESTATION OFF CACHE BOOL "Disable attestation partition" FORCE)

# DEV/TEST ONLY - MPU enforcement test App-RoT partition (see docs/mpu_enforcement_tests.md).
# Proves L2 MPU actually isolates App-RoT from PSA-RoT. MUST be OFF for normal/production builds.
set(TFM_PARTITION_MPU_TEST ON CACHE BOOL "Enable MPU enforcement test partition (dev only)" FORCE)

# DEV/TEST ONLY - halt (spin) instead of rebooting on a secure core panic. Needed
# for the destructive MPU tests (TC6-8): the unpriv->PSA-RoT access raises a
# MemManage that TF-M's own handler claims and routes to tfm_core_panic; with the
# default (reboot) the RAM-resident image re-runs and re-faults in a tight reset
# loop that also resets the debug domain (kills JTAG). Halting spins in the secure
# panic path so the fault is catchable at a stable halt. MUST be OFF for production.
set(CONFIG_TFM_HALT_ON_CORE_PANIC ON CACHE BOOL "Halt (don't reboot) on secure panic (dev only)" FORCE)
