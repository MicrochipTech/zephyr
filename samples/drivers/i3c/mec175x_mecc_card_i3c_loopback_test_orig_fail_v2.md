# MEC175x MECC card: I3C loopback, original failure with role readout (teraterm_loopback_v2.log)

- Board: `mec_assy6941/mec1753_qsz`
- Source log: `teraterm_loopback_v2.log` only. Same build as `teraterm_loopback.log` plus a
  debug readout of the role registers in `dw_i3c_init()`. No `bcr`, 1 KB thread stacks.
- I3C0 = `i3c@40010800` (HOST0, controller-only), regs 0x40010800-0x40010FFF, port-sel 1
  at `HOST_CFG` 0x40010B00, GPIO012/GPIO013
- I3C1 = `i3c@40010000` (SEC0, dual-role), regs 0x40010000-0x400107FF, port-sel 0 written at
  0x40010300; its `SEC_CFG` at 0x40010400 is not written by this build. GPIO132/GPIO140
- I3C0 and I3C1 pads are blue-wired together (SDA-SDA, SCL-SCL)
- Red block: the point where the dual-role I3C1 is initialized as a controller (0.013).
- Orange blocks: bus exchange per CCC, inferred from the log (confirm with the analyzer).
  Red blocks inside: I3C1 driving the bus. I3C1 queues no transfer of its own (empty device
  list, so no bus init), but as an enabled controller it answers every I3C0 START by driving
  NACK and controller clocking. I3C0 sees that as a request from 0x00 and refuses it, and I3C1
  answers again: two I3C1 drive attempts and one I3C0 refusal per CCC.
- Log lines follow the bus exchange in ISR order. With immediate logging each line takes
  about 8 ms, while the bus exchange lasts microseconds, so log order is not bus order.

```mermaid
%%{init: {"theme": "base", "themeVariables": {"background": "#ffffff", "fontSize": "14px", "primaryColor": "#ffffff", "primaryTextColor": "#000000", "primaryBorderColor": "#000000", "textColor": "#000000", "lineColor": "#000000", "actorBkg": "#ffffff", "actorBorder": "#000000", "actorTextColor": "#000000", "actorLineColor": "#000000", "signalColor": "#000000", "signalTextColor": "#000000", "labelBoxBkgColor": "#ffffff", "labelBoxBorderColor": "#000000", "labelTextColor": "#000000", "loopTextColor": "#000000", "noteBkgColor": "#fff5ad", "noteBorderColor": "#000000", "noteTextColor": "#000000", "sequenceNumberColor": "#ffffff"}}}%%
sequenceDiagram
    autonumber
    participant APP as Test app
    participant DRV as Zephyr I3C driver
    participant I3C0 as I3C0 controller-only
    participant I3C1 as I3C1 dual-role
    rect rgb(255, 255, 255)

        %% I3C0: i3c@40010800 (HOST0, controller-only), regs 0x40010800-0x40010FFF
        %%       port-sel 1 written to HOST_CFG 0x40010B00 (base + 0x300), pads GPIO012/GPIO013
        %% I3C1: i3c@40010000 (SEC0, dual-role), regs 0x40010000-0x400107FF
        %%       port-sel 0 written to 0x40010300 (base + 0x300, a DCT entry on SEC0)
        %%       SEC_CFG at 0x40010400 (base + 0x400) is not written by this build, pads GPIO132/GPIO140
        %% I3C0 and I3C1 pads blue-wired together (SDA-SDA, SCL-SCL)

        Note over I3C0: Regs 0x40010800-0x40010FFF<br/>port-sel 1 at HOST_CFG 0x40010B00
        Note over I3C1: Regs 0x40010000-0x400107FF<br/>port-sel 0 at 0x40010300<br/>SEC_CFG 0x40010400 not written

        Note over DRV,I3C1: POST_KERNEL device init, before the app starts
        rect rgb(255, 210, 210)
            DRV->>I3C1: dw_i3c_init: soft reset, port-sel 0, no mode write, enable
            Note right of I3C1: 0.013 role=3 DEV_CTRL_EXT=0x00000000<br/>PRESENT_STATE=0x10000007 is_secondary=0<br/>DEV_OPERATION_MODE=0 reset value, CURRENT_MASTER=1<br/>I3C1 dual-role initialized as CONTROLLER<br/>controller and target IRQs enabled
        end
        Note right of I3C1: No bus init on I3C1 (empty device list)
        DRV->>I3C0: dw_i3c_init: soft reset, port-sel 1, enable
        Note right of I3C0: 0.024 role=1 DEV_CTRL_EXT=0x00000000<br/>PRESENT_STATE=0x10000007 is_secondary=0<br/>dev list has dev0 PID 0xB0123456789A, SA 0x08

        Note over DRV,I3C1: i3c_bus_init on I3C0. Both instances act as controllers on SDA and SCL
        DRV->>I3C0: Broadcast CCC RSTACT (whole target)
        rect rgb(255, 230, 190)
            Note over I3C0,I3C1: Bus exchange, microseconds (inferred)
            I3C0->>I3C1: START + 0x7E header
            rect rgb(255, 190, 190)
                I3C1->>I3C0: I3C1 attempt 1: takes the START as an IBI, drives NACK and clocking
            end
            I3C0->>I3C1: I3C0 sees I3C1 driving as a request from 0x00, drives NACK
            rect rgb(255, 190, 190)
                I3C1->>I3C0: I3C1 attempt 2: sees that as another request, drives NACK and clocking
            end
        end
        Note over DRV,I3C1: Logged in ISR order
        I3C0-->>DRV: 0.045 i3c@40010800: NAK for MR from 0x00
        I3C1-->>DRV: 0.053 i3c@40010000: NAK for MR from 0x00
        I3C1-->>DRV: 0.061 i3c@40010000: NAK for MR from 0x00
        I3C0 --x DRV: Transfer error, -EIO
        Note over DRV: 0.086 Broadcast RSTACT (whole target) was NACK

        DRV->>I3C0: Broadcast CCC RSTACT (peripheral)
        rect rgb(255, 230, 190)
            Note over I3C0,I3C1: Bus exchange, microseconds (inferred)
            I3C0->>I3C1: START + 0x7E header
            rect rgb(255, 190, 190)
                I3C1->>I3C0: I3C1 attempt 1: takes the START as an IBI, drives NACK and clocking
            end
            I3C0->>I3C1: I3C0 sees I3C1 driving as a request from 0x00, drives NACK
            rect rgb(255, 190, 190)
                I3C1->>I3C0: I3C1 attempt 2: sees that as another request, drives NACK and clocking
            end
        end
        Note over DRV,I3C1: Logged in ISR order
        I3C1-->>DRV: 0.104 i3c@40010000: NAK for MR from 0x00
        I3C0-->>DRV: 0.112 i3c@40010800: NAK for MR from 0x00
        I3C1-->>DRV: 0.120 i3c@40010000: NAK for MR from 0x00
        I3C0 --x DRV: Transfer error, -EIO
        Note over DRV: 0.145 Broadcast RSTACT (peripheral) was NACK

        DRV->>I3C0: Broadcast CCC RSTDAA
        rect rgb(255, 230, 190)
            Note over I3C0,I3C1: Bus exchange, microseconds (inferred)
            I3C0->>I3C1: START + 0x7E header
            rect rgb(255, 190, 190)
                I3C1->>I3C0: I3C1 attempt 1: takes the START as an IBI, drives NACK and clocking
            end
            I3C0->>I3C1: I3C0 sees I3C1 driving as a request from 0x00, drives NACK
            rect rgb(255, 190, 190)
                I3C1->>I3C0: I3C1 attempt 2: sees that as another request, drives NACK and clocking
            end
        end
        Note over DRV,I3C1: Logged in ISR order
        I3C1-->>DRV: 0.163 i3c@40010000: NAK for MR from 0x00
        I3C0-->>DRV: 0.171 i3c@40010800: NAK for MR from 0x00
        I3C1-->>DRV: 0.179 i3c@40010000: NAK for MR from 0x00
        I3C0 --x DRV: Transfer error, -EIO
        Note over DRV: 0.204 RSTDAA error (-5)<br/>0.210 Broadcast RSTDAA was NACK

        DRV->>I3C0: Broadcast CCC DISEC
        rect rgb(255, 230, 190)
            Note over I3C0,I3C1: Bus exchange, microseconds (inferred)
            I3C0->>I3C1: START + 0x7E header
            rect rgb(255, 190, 190)
                I3C1->>I3C0: I3C1 attempt 1: takes the START as an IBI, drives NACK and clocking
            end
            I3C0->>I3C1: I3C0 sees I3C1 driving as a request from 0x00, drives NACK
            rect rgb(255, 190, 190)
                I3C1->>I3C0: I3C1 attempt 2: sees that as another request, drives NACK and clocking
            end
        end
        Note over DRV,I3C1: Logged in ISR order
        I3C1-->>DRV: 0.227 i3c@40010000: NAK for MR from 0x00
        I3C0-->>DRV: 0.235 i3c@40010800: NAK for MR from 0x00
        I3C1-->>DRV: 0.243 i3c@40010000: NAK for MR from 0x00
        I3C0 --x DRV: Transfer error, -EIO
        Note over DRV: 0.268 Broadcast DISEC was NACK

        DRV->>DRV: 0.275 attach dev0 with SA 0x08 for SETDASA
        DRV->>I3C0: Direct CCC SETDASA to SA 0x08, DA 0x08
        rect rgb(255, 230, 190)
            Note over I3C0,I3C1: Bus exchange, microseconds (inferred)
            I3C0->>I3C1: START + 0x7E header
            rect rgb(255, 190, 190)
                I3C1->>I3C0: I3C1 attempt 1: takes the START as an IBI, drives NACK and clocking
            end
            I3C0->>I3C1: I3C0 sees I3C1 driving as a request from 0x00, drives NACK
            rect rgb(255, 190, 190)
                I3C1->>I3C0: I3C1 attempt 2: sees that as another request, drives NACK and clocking
            end
        end
        Note over DRV,I3C1: Logged in ISR order
        I3C1-->>DRV: 0.304 i3c@40010000: NAK for MR from 0x00
        I3C0-->>DRV: 0.312 i3c@40010800: NAK for MR from 0x00
        I3C1-->>DRV: 0.321 i3c@40010000: NAK for MR from 0x00
        I3C0 --x DRV: Transfer error, -EIO
        Note over DRV: 0.346 SETDASA error (-5)<br/>0.363 SETDASA error on address 0x8 (-5)
        DRV->>DRV: 0.354 detach dev0

        DRV->>I3C0: Broadcast CCC ENEC
        rect rgb(255, 230, 190)
            Note over I3C0,I3C1: Bus exchange, microseconds (inferred)
            I3C0->>I3C1: START + 0x7E header
            rect rgb(255, 190, 190)
                I3C1->>I3C0: I3C1 attempt 1: takes the START as an IBI, drives NACK and clocking
            end
            I3C0->>I3C1: I3C0 sees I3C1 driving as a request from 0x00, drives NACK
            rect rgb(255, 190, 190)
                I3C1->>I3C0: I3C1 attempt 2: sees that as another request, drives NACK and clocking
            end
        end
        Note over DRV,I3C1: Logged in ISR order
        I3C1-->>DRV: 0.379 i3c@40010000: NAK for MR from 0x00
        I3C0-->>DRV: 0.387 i3c@40010800: MR from unknown addr 0x00
        I3C1-->>DRV: 0.394 i3c@40010000: NAK for MR from 0x00
        I3C0 --x DRV: Transfer error, -EIO
        Note over DRV: 0.419 Broadcast ENEC was NACK
        Note over DRV: bus_init ends, dev0 has no dynamic address

        Note over APP,I3C1: Zephyr boots and main() runs
        APP->>DRV: setup_i3c_target(i3c1): i3c_configure(TARGET)
        DRV->>I3C1: SLV_MAX_LEN, DEVICE_ADDR SA 0x08, PID. Mode not changed
        DRV-->>APP: 0.460 Target configured, PID 0xB0123456789A, SA 0x08
        APP->>DRV: setup_i3c_host(i3c0)
        DRV-->>APP: 0.469 Host ready
        APP->>DRV: i3c_target_register(i3c1)
        DRV-->>APP: 0.475 Target registered (echo mode)
        APP->>DRV: Host thread: i3c_device_find(PID 0xB0123456789A)
        DRV --x APP: 0.483 FAIL, target not found on bus, dynamic_addr=0
        Note over APP: 0.492 USAGE FAULT, illegal use of the EPSR<br/>pc 0x000b1b88, thread 0x118b70<br/>1 KB thread stacks overflowed<br/>0.548 Halting system
    end
```
