# MEC175x MECC card: I3C loopback, original failure

- Board: `mec_assy6941/mec1753_qsz`
- Source log: `teraterm_loopback.log` (no `bcr` property, 1 KB thread stacks)
- I3C0 = `i3c@40010800` (HOST0, controller-only), port-sel 1, GPIO012/GPIO013
- I3C1 = `i3c@40010000` (SEC0, dual-role), port-sel 0, GPIO132/GPIO140
- I3C0 and I3C1 pads are blue-wired together (SDA-SDA, SCL-SCL)
- The I3C1 role and `DEV_OPERATION_MODE` come from the debug readout in `teraterm_loopback_v3.log`

```mermaid
sequenceDiagram
    autonumber
    participant APP as Test app
    participant DRV as Zephyr I3C driver
    participant I3C0 as I3C0 controller-only
    participant I3C1 as I3C1 dual-role

    Note over DRV,I3C1: POST_KERNEL device init, before the app starts
    DRV->>I3C1: dw_i3c_init: soft reset, port-sel 0, enable
    Note right of I3C1: role=3, DEV_OPERATION_MODE=0 (reset value)<br/>CURRENT_MASTER=1, so it runs as a controller<br/>Controller and target IRQs both enabled
    DRV->>I3C0: dw_i3c_init: soft reset, port-sel 1, enable
    Note right of I3C0: role=1, primary controller<br/>dev list has dev0 PID 0xB0123456789A, SA 0x08

    Note over DRV,I3C1: i3c_bus_init on I3C0. Both controllers share SDA and SCL
    DRV->>I3C0: Broadcast CCC RSTACT (whole target)
    I3C0->>I3C1: START + 0x7E on the shared bus
    I3C1-->>DRV: IRQ, IBI with address 0x00 seen as a mastership request
    DRV->>I3C1: NAK for MR from 0x00 (0.031, 0.039)
    I3C0-->>DRV: IRQ, IBI with address 0x00 seen as a mastership request
    DRV->>I3C0: NAK for MR from 0x00 (0.023)
    I3C0 --x DRV: Transfer error, -EIO
    Note over DRV: 0.064 Broadcast RSTACT (whole target) was NACK

    loop Same pattern for RSTACT (peripheral), RSTDAA, DISEC
        DRV->>I3C0: Broadcast CCC
        I3C0->>I3C1: START + 0x7E
        I3C1-->>DRV: MR from 0x00, NAK (x2)
        I3C0-->>DRV: MR from 0x00, NAK
        I3C0 --x DRV: Transfer error, -EIO
    end
    Note over DRV: 0.123 RSTACT (peripheral) NACK<br/>0.182 RSTDAA error (-5)<br/>0.246 DISEC NACK

    DRV->>DRV: 0.253 attach dev0 with SA 0x08 for SETDASA
    DRV->>I3C0: Direct CCC SETDASA to 0x08, assign DA 0x08
    I3C0->>I3C1: START + 0x7E
    I3C1-->>DRV: MR from 0x00, NAK (x2)
    I3C0-->>DRV: MR from 0x00, NAK
    I3C0 --x DRV: Transfer error, -EIO
    Note over DRV: 0.324 SETDASA error (-5)
    DRV->>DRV: 0.332 detach dev0

    DRV->>I3C0: Broadcast CCC ENEC
    I3C0->>I3C1: START + 0x7E
    I3C1-->>DRV: MR from 0x00, NAK (0.357, 0.373)
    I3C0-->>DRV: 0.365 MR from unknown addr 0x00
    I3C0 --x DRV: Transfer error, -EIO
    Note over DRV: 0.397 Broadcast ENEC was NACK<br/>bus_init ends, dev0 has no dynamic address

    Note over APP,I3C1: 0.410 Zephyr boots and main() runs
    APP->>DRV: setup_i3c_target(i3c1): i3c_configure(TARGET)
    DRV->>I3C1: Write SLV_MAX_LEN, DEVICE_ADDR SA 0x08, PID
    Note right of I3C1: Mode is not changed. Still a controller
    DRV-->>APP: 0.438 Target configured, PID 0xB0123456789A, SA 0x08
    APP->>DRV: setup_i3c_host(i3c0)
    DRV-->>APP: 0.447 Host ready
    APP->>DRV: i3c_target_register(i3c1)
    DRV-->>APP: 0.453 Target registered (echo mode)
    APP->>DRV: Host thread: i3c_device_find(PID 0xB0123456789A)
    DRV --x APP: 0.461 FAIL, target not found on bus, dynamic_addr=0
    Note over APP: 1 KB thread stacks overflow later (usage fault)
```
