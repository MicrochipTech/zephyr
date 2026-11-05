# MEC175x MECC card: I3C loopback, failure with the bcr property

- Board: `mec_assy6941/mec1753_qsz`
- Source log: `teraterm_loopback_v4.log`
- Build: `bcr = <0x06>` on `&i3c1`, driver writes `DEV_OPERATION_MODE` before enable,
  waits for the soft reset to complete, 4 KB thread stacks
- Not in this build: clearing `ENABLE` before the mode write, `SEC_CFG` (0x400)
  port-select offset
- I3C0 = `i3c@40010800` (HOST0, controller-only), port-sel 1
- I3C1 = `i3c@40010000` (SEC0, dual-role), port-sel 0

```mermaid
sequenceDiagram
    autonumber
    participant APP as Test app
    participant DRV as Zephyr I3C driver
    participant I3C0 as I3C0 controller-only
    participant I3C1 as I3C1 dual-role

    Note over DRV,I3C1: POST_KERNEL device init, before the app starts
    DRV->>I3C1: dw_i3c_init: soft reset, wait for RESET_CTRL to clear
    I3C1-->>DRV: Reset complete (no timeout)
    DRV->>I3C1: Port-sel 0 written at 0x300 (HOST_CFG offset, SEC_CFG is at 0x400)
    DRV->>I3C1: BCR 0x06, device role target: DEVICE_CTRL_EXTENDED.DEV_OPERATION_MODE = 1
    I3C1-->>DRV: Readback DEV_OPERATION_MODE = 0
    Note over DRV: 0.013 i3c@40010000: DEV_OPERATION_MODE stays 0x0, requested 0x1
    DRV --x DRV: dw_i3c_init(i3c1) returns -EIO
    Note right of I3C1: Never enabled, no IRQs<br/>Device not ready

    DRV->>I3C0: dw_i3c_init: soft reset, port-sel 1, enable
    Note over DRV: 0.022 i3c@40010800: role=1 DEV_CTRL_EXT=0x00000000<br/>PRESENT_STATE=0x10000007 is_secondary=0

    Note over DRV,I3C1: i3c_bus_init on I3C0. Single controller, no target answering
    loop RSTACT (whole target), RSTACT (peripheral), RSTDAA, DISEC
        DRV->>I3C0: Broadcast CCC
        I3C0->>I3C1: 0x7E + CCC
        Note right of I3C1: Disabled, no ACK
        I3C0 --x DRV: Transfer error, -EIO
    end
    Note over DRV: 0.050 RSTACT (whole target) NACK<br/>0.085 RSTACT (peripheral) NACK<br/>0.119 RSTDAA error (-5)<br/>0.159 DISEC NACK
    Note over DRV,I3C1: No MR from 0x00 reports: the second controller is gone

    DRV->>DRV: 0.166 attach dev0 with SA 0x08 for SETDASA
    DRV->>I3C0: Direct CCC SETDASA to 0x08, DA 0x08
    I3C0->>I3C1: 0x7E + SETDASA
    I3C0 --x DRV: Transfer error, -EIO
    Note over DRV: 0.212 SETDASA error (-5)
    DRV->>DRV: 0.220 detach dev0

    DRV->>I3C0: Broadcast CCC ENEC
    I3C0->>I3C1: 0x7E + ENEC
    I3C0 --x DRV: Transfer error, -EIO
    Note over DRV: 0.262 Broadcast ENEC was NACK

    Note over APP,I3C1: 0.262 Zephyr boots, 4 KB host and target thread stacks
    APP->>DRV: setup_i3c_target(i3c1): device_is_ready(i3c1)
    DRV --x APP: false
    Note over APP: 0.302 Target[i3c@40010000] not ready<br/>0.309 Target init failed, halting
    Note over APP: Host thread never started
```
