# MEC175x MECC card: I3C loopback, expected sequence

- Board: `mec_assy6941/mec1753_qsz`
- I3C0 = `i3c@40010800` (HOST0, controller-only), port-sel 1 (`HOST_CFG` at 0x300)
- I3C1 = `i3c@40010000` (SEC0, dual-role), port-sel 0 (`SEC_CFG` at 0x400), `bcr = <0x06>`
- Test case `CONFIG_I3C_TEST_CASE=0` (data transfer), target in echo mode

Assumptions:

- `DEV_OPERATION_MODE` on I3C1 accepts the target mode (not achieved yet, see the
  failure after the `bcr` property)
- `&i3c0` carries `zephyr,deferred-init`, so bus init (SETDASA 0x08) runs from
  `setup_i3c_host()` after `main()` has given I3C1 its static address. The current
  loopback overlay does not set it.

```mermaid
sequenceDiagram
    autonumber
    participant APP as Test app
    participant DRV as Zephyr I3C driver
    participant I3C0 as I3C0 controller-only
    participant I3C1 as I3C1 dual-role

    Note over DRV,I3C1: POST_KERNEL device init. I3C0 is deferred
    DRV->>I3C1: dw_i3c_init: soft reset, wait for RESET_CTRL to clear
    DRV->>I3C1: SEC_CFG (0x400) = port-sel 0
    DRV->>I3C1: BCR 0x06, device role target: clear ENABLE, wait
    DRV->>I3C1: DEVICE_CTRL_EXTENDED.DEV_OPERATION_MODE = 1
    I3C1-->>DRV: Readback DEV_OPERATION_MODE = 1
    DRV->>I3C1: SLV_CHAR_CTRL.BCR = 0x06
    DRV->>I3C1: Enable, target IRQ mask only
    Note right of I3C1: is_secondary=1, CURRENT_MASTER=0<br/>No bus init, never drives START

    Note over APP,I3C1: Zephyr boots and main() runs
    APP->>DRV: setup_i3c_target(i3c1): i3c_configure(TARGET)
    DRV->>I3C1: SLV_MAX_LEN 128/128, DEVICE_ADDR SA 0x08 valid, PID 0xB0123456789A
    APP->>DRV: i3c_target_register(i3c1)
    APP->>APP: Post EVT_TARGET_READY, start target thread

    APP->>DRV: setup_i3c_host(i3c0): device_init(i3c0)
    DRV->>I3C0: dw_i3c_init: soft reset, HOST_CFG (0x300) = port-sel 1, enable
    Note right of I3C0: role=1, primary controller, controller IRQ mask

    Note over DRV,I3C1: i3c_bus_init on I3C0. I3C1 answers as a target
    DRV->>I3C0: Broadcast RSTACT, RSTDAA, DISEC
    I3C0->>I3C1: 0x7E + CCC
    I3C1-->>I3C0: ACK
    I3C0-->>DRV: Response OK
    DRV->>I3C0: Direct SETDASA to SA 0x08, DA 0x08
    I3C0->>I3C1: 0x7E, SETDASA, 0x08 + DA 0x08
    I3C1-->>I3C0: ACK
    I3C1-->>DRV: IRQ DYN_ADDR_ASSGN, DA 0x08
    I3C0-->>DRV: Response OK, dev0 dynamic_addr = 0x08
    DRV->>I3C0: Broadcast ENEC
    I3C0->>I3C1: 0x7E + ENEC
    I3C1-->>I3C0: ACK
    DRV-->>APP: Host ready

    APP->>APP: Start host thread, EVT_TARGET_READY already set
    APP->>DRV: i3c_device_find(PID 0xB0123456789A)
    DRV-->>APP: Target found, dynamic_addr = 0x08

    Note over APP,I3C1: TC0 data transfer
    APP->>DRV: write_1B: i3c_write(0xA5)
    DRV->>I3C0: Private write to DA 0x08
    I3C0->>I3C1: 0x08 W + 0xA5
    I3C1-->>DRV: RX data, stop_cb, EVT_DATA_RECEIVED
    DRV-->>APP: Target thread: echo with i3c_target_tx_write, raise TIR
    Note right of I3C1: The echoed byte stays in the TX queue<br/>until the host reads it

    loop loopback_1B, 32B, 64B, 128B
        APP->>DRV: Host: i3c_write(N bytes)
        DRV->>I3C0: Private write to DA 0x08
        I3C0->>I3C1: 0x08 W + N bytes
        I3C1-->>DRV: RX data, EVT_DATA_RECEIVED
        DRV->>I3C1: Target thread: i3c_target_tx_write(N bytes) to TX queue
        APP->>APP: Host: sleep I3C_TEST_ECHO_DELAY_MS
        APP->>DRV: Host: i3c_read(N bytes)
        DRV->>I3C0: Private read from DA 0x08
        I3C0->>I3C1: 0x08 R
        I3C1-->>I3C0: N bytes from TX queue
        I3C0-->>DRV: Response OK
        DRV-->>APP: Compare TX and RX with CRC8, PASS
    end

    APP->>DRV: combined_xfer: i3c_write_read(64 bytes, 64 bytes)
    DRV->>I3C0: Write + repeated START + read to DA 0x08
    I3C0->>I3C1: 0x08 W + 64 bytes, Sr, 0x08 R
    I3C1-->>I3C0: 64 bytes from TX queue
    DRV-->>APP: Compare, PASS
    Note over APP: TC0 group result PASS
```
