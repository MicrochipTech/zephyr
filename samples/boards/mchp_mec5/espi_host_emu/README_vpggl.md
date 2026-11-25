# espi_host_emu: VPGGL (glue logic) bench support

This Host EVB plays the role of the platform (PCH and power rails) for
bench-testing the VPGGL glue logic on a second EVB, the **Target**. Both
EVBs are ASSY6941 with a MEC1757B-B0 CPU card.

The Target application, the VPGGL driver and the full test description
live here:

| | |
|---|---|
| Repository | https://github.com/MicrochipTech/mec5_zapps_z4_4 |
| Branch | `c21974_1110_app_mec1757b_vpggl` |
| Path | `app_vpggl` (see `app_vpggl/README.md` for the block, use case and test cases), `drivers/vpggl` |
| Chip tested | MEC1757B-B0 (notebook / mobile glue logic, datasheet chapter 50) |

## What the Host does

1. Drives the Target's VPGGL input pins to idle (VCC_PWRGD=1, PWRGD_S0IX=0,
   SLP_S0#=1).
2. Brings up the emulated eSPI link and de-asserts PLTRST# (VWire). The
   Target needs that before it can enable S0ix detection.
3. Stops the scripted sequence and takes `hvpggl` commands on its console,
   so the pins and VWires can be changed by hand.

## Wiring (Host EVB -> Target EVB)

| Host | Target | Signal |
|---|---|---|
| GPIO057 | GPIO057 | VCC_PWRGD |
| GPIO202 | GPIO226 | PWRGD_S0IX |
| GPIO025 | GPIO064 | SLP_S0# |
| GND | GND | |
| eSPI (QSPI emulation) | eSPI | PLTRST#, HOST_C10 as VWires |

All signals are 3.3 V (VTR1). The Host_nREADY / TARGET_nREADY handshake is
not used; those Host pins are reused for VPGGL.

## Build

The Host chip is a MEC1757B, but Zephyr has no MEC1757 SoC support, so the
Host is built for the `mec_assy6941/mec1753_qsz` board. This works because
`boards/mec_assy6941_mec1753_qsz.overlay` puts the MEC1757 internal-flash
pins in low-power state, so QSPI uses the shared port for the eSPI
emulation.

```
west build -d /tmp/build_hostemu -b mec_assy6941/mec1753_qsz \
    samples/boards/mchp_mec5/espi_host_emu -- \
    -DEXTRA_CONF_FILE=boards/vpggl_bench.conf \
    -DEXTRA_DTC_OVERLAY_FILE=boards/vpggl_bench.overlay
```

Files for this support: `boards/vpggl_bench.conf`,
`boards/vpggl_bench.overlay`, `src/vpggl_host.c`, `src/vpggl_host.h`.
Without the two bench files the sample builds and behaves as before.

## Start-up order

1. Reset the **Target** first (it waits up to 60 s for PLTRST#).
2. Reset this **Host**. Expect `VPGGL pins driven to idle ...`, the eSPI
   bring-up steps, then `hvpggl ready`.

## Commands

Levels are electrical (1 = high). The `hvpggl` prefix is optional.

| Command | Action |
|---|---|
| `hvpggl pins` | Show the levels being driven |
| `hvpggl pin <vcc\|s0ix\|slp> <0\|1>` | Drive a Target VPGGL input |
| `hvpggl s0ix enter\|exit` | Same as `pin slp 0` / `pin slp 1` |
| `hvpggl pulse <vcc\|s0ix\|slp> <ms>` | Invert a pin for `<ms>`, then restore |
| `hvpggl pltrst <0\|1>` | PLTRST# VWire (1 = de-asserted) |
| `hvpggl c10 <0\|1>` | HOST_C10 VWire |
| `hvpggl vw <idx> <bit> <0\|1>` | Any Controller-to-Target VWire |
| `hvpggl status` | Pins, eSPI GET_STATUS, Target-to-Host VWires |
| `hvpggl help` | List the commands |

Test sequences and the expected Target logs are in `app_vpggl/README.md`
(section "Test cases").

## Note

`hvpggl` reads the console by polling the UART in a low-priority thread. It
deliberately does not use the Zephyr shell: with `CONFIG_SHELL` enabled this
Host EVB printed nothing at all.
