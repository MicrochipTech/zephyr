.. zephyr:code-sample:: mec5-mux-switch
   :name: Microchip MEC5 I2C Network-Layer + MUX port switch

   Exercise the Microchip MEC5 I2C Network-Layer (DMA) driver and the
   companion per-port MUX driver, switching between two pin pairs and
   two clock frequencies on a single underlying controller.

Overview
********

The MEC5 I2C_SMB controller has a 16-entry pin-pair mux selected by
``CONFIG[3:0]``. Switching pin pairs requires a controller reset plus
PINCTRL re-apply before the next transfer. Rather than bake that into
the controller driver, the port-switching policy lives in a companion
MUX driver (``microchip,mec5-i2c-mux``) that re-exports the Zephyr I2C
API and serializes reconfiguration behind a mutex.

This sample enables one controller (``i2c_smb_0``) with two MUX child
ports on different SCL/SDA pins and different clock rates (100 kHz
and 400 kHz). ``main()`` alternates transfers between the two child
bus devices so the MUX is forced to stop-reset-reapply-reprogram the
controller between calls.

The sample does not require a physical peripheral on the bus. Each
transfer targets an arbitrary 7-bit address; a NAK is a valid
hardware response (it proves the Network-Layer engine drove a full
START-to-STOP transaction through DMA) and the log labels each
outcome.

One transfer per round is a 200-byte write on a 64-byte DMA buffer,
exercising the controller-TX chunking path (multiple DMA reloads
across a single START-to-STOP).

Building and flashing
*********************

.. code-block:: console

   west build -b mec_assy6941/mec1753/qlj samples/drivers/i2c/mec5_mux_switch
   west flash

Expected console output (abbreviated):

.. code-block:: console

   mec5 mux switch sample
   port0 = i2c_port0@... (100 kHz)
   port3 = i2c_port3@... (400 kHz)
   --- round 0 ---
   port 0  4B write  @0x50 -> NAKed (no device) (-6)
   port 3  8B read   @0x51 -> NAKed (no device) (-6)
   ...

Wiring for a real target
************************

To ACK instead of NAK, wire an I2C peripheral to the pin pair of
either port in the overlay:

* port 0: ``GPIO003`` (SDA) / ``GPIO004`` (SCL)
* port 3: ``GPIO007`` (SDA) / ``GPIO010`` (SCL)

and use its real 7-bit address in ``DUMMY_ADDR_A`` / ``DUMMY_ADDR_B``
in ``src/main.c``.
