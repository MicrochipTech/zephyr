.. _flash-debug-host-tools:

Flash & Debug Host Tools
########################

This guide describes the software tools you can run on your host workstation to
flash and debug Zephyr applications.

Zephyr's west tool has built-in support for all of these in its ``flash``,
``debug``, ``debugserver``, and ``attach`` commands, provided your board
hardware supports them and your Zephyr board directory's :file:`board.cmake`
file declares that support properly. See :ref:`west-build-flash-debug` for
more information on these commands.

.. _runner_blackmagicprobe:

Black Magic Probe
*****************

Black Magic Probe (BMP) is an open-source debugging hardware incorporating GDB debug
server functionality into the firmware.
There is no need for a GDB server program, so there is no program equivalent
to host-tool.

For more details, including usage instructions and supported targets,
see :ref:`black-magic-probe`.

.. _atmel_sam_ba_bootloader:
.. _runner_bossac:

SAM Boot Assistant (SAM-BA)
***************************

Atmel SAM Boot Assistant (Atmel SAM-BA) allows In-System Programming (ISP)
from USB or UART host without any external programming interface.  Zephyr
allows users to develop and program boards with SAM-BA support using
:ref:`west <west-flashing>`.  Zephyr supports devices with/without ROM
bootloader and both extensions from Arduino and Adafruit. Full support was
introduced in Zephyr SDK 0.12.0.

The typical command to flash the board is:

.. code-block:: console

	west flash [ -r bossac ] [ -p /dev/ttyX ] [ --erase ]

.. note::

    By default, flashing with bossac will only erase the flash pages containing
    the flashed application, leaving other pages untouched. Should you wish to
    erase the entire flash of the target when flashing, pass the ``--erase``
    parameter when flashing.

Flash configuration for devices:

.. tabs::

    .. tab:: With ROM bootloader

        These devices don't need any special configuration.  After building your
        application, just run ``west flash`` to flash the board.

    .. tab:: Without ROM bootloader

        For these devices, the user should:

        1. Define flash partitions required to accommodate the bootloader and
           application image; see :ref:`flash_map_api` for details.
        2. Have board :file:`.defconfig` file with the
           :kconfig:option:`CONFIG_USE_DT_CODE_PARTITION` Kconfig option set to ``y`` to
           instruct the build system to use these partitions for code relocation.
           This option can also be set in ``prj.conf`` or any other Kconfig fragment.
        3. Build and flash the SAM-BA bootloader on the device.

    .. tab:: With compatible SAM-BA bootloader

        For these devices, the user should:

        1. Define flash partitions required to accommodate the bootloader and
           application image; see :ref:`flash_map_api` for details.
        2. Have board :file:`.defconfig` file with the
           :kconfig:option:`CONFIG_BOOTLOADER_BOSSA` Kconfig option set to ``y``.  This will
           automatically select the :kconfig:option:`CONFIG_USE_DT_CODE_PARTITION` Kconfig
           option which instruct the build system to use these partitions for code
           relocation.  The board :file:`.defconfig` file should have
           :kconfig:option:`CONFIG_BOOTLOADER_BOSSA_ARDUINO` ,
           :kconfig:option:`CONFIG_BOOTLOADER_BOSSA_ADAFRUIT_UF2` or the
           :kconfig:option:`CONFIG_BOOTLOADER_BOSSA_LEGACY` Kconfig option set to ``y``
           to select the right compatible SAM-BA bootloader mode.
           These options can also be set in ``prj.conf`` or any other Kconfig fragment.
        3. Build and flash the SAM-BA bootloader on the device.

.. note::

    The :kconfig:option:`CONFIG_BOOTLOADER_BOSSA_LEGACY` Kconfig option should be used
    as last resource.  Try configure first with Devices without ROM bootloader.


Typical flash layout and configuration
--------------------------------------

For bootloaders that reside on flash, the devicetree partition layout is
mandatory.  For devices that have a ROM bootloader, they are mandatory when
the application uses a storage or other non-application partition.  In this
special case, the boot partition should be omitted and code_partition should
start from offset 0.  It is necessary to define the partitions with sizes that
avoid overlaps, always.

A typical flash layout for devices without a ROM bootloader is:

.. code-block:: devicetree

	/ {
		chosen {
			zephyr,code-partition = &code_partition;
		};
	};

	&flash0 {
		partitions {
			compatible = "fixed-partitions";
			#address-cells = <1>;
			#size-cells = <1>;

			boot_partition: partition@0 {
				label = "sam-ba";
				reg = <0x00000000 0x2000>;
				read-only;
			};

			code_partition: partition@2000 {
				label = "code";
				reg = <0x2000 0x3a000>;
				read-only;
			};

			/*
			* The final 16 KiB is reserved for the application.
			* Storage partition will be used by FCB/LittleFS/NVS
			* if enabled.
			*/
			storage_partition: partition@3c000 {
				label = "storage";
				reg = <0x0003c000 0x00004000>;
			};
		};
	};

A typical flash layout for devices with a ROM bootloader and storage
partition is:

.. code-block:: devicetree

	/ {
		chosen {
			zephyr,code-partition = &code_partition;
		};
	};

	&flash0 {
		partitions {
			compatible = "fixed-partitions";
			#address-cells = <1>;
			#size-cells = <1>;

			code_partition: partition@0 {
				label = "code";
				reg = <0x0 0xF0000>;
				read-only;
			};

			/*
			* The final 64 KiB is reserved for the application.
			* Storage partition will be used by FCB/LittleFS/NVS
			* if enabled.
			*/
			storage_partition: partition@F0000 {
				label = "storage";
				reg = <0x000F0000 0x00100000>;
			};
		};
	};


Enabling SAM-BA runner
----------------------

In order to instruct Zephyr west tool to use the SAM-BA bootloader the
:file:`board.cmake` file must have
``include(${ZEPHYR_BASE}/boards/common/bossac.board.cmake)`` entry.  Note that
Zephyr tool accept more entries to define multiple runners.  By default, the
first one will be selected when using ``west flash`` command.  The remaining
options are available passing the runner option, for instance
``west flash -r bossac``.


More implementation details can be found in the :ref:`boards` documentation.
As a quick reference, see these three board documentation pages:

  - :zephyr:board:`sam4e_xpro` (ROM bootloader)
  - :zephyr:board:`adafruit_feather_m0_basic_proto` (Adafruit UF2 bootloader)
  - :zephyr:board:`arduino_nano_33_iot` (Arduino bootloader)
  - :zephyr:board:`arduino_nano_33_ble` (Arduino legacy bootloader)

Enabling BOSSAC on Windows Native [Experimental]
------------------------------------------------

Zephyr SDK´s bossac is currently supported on Linux and macOS only. Windows support
can be achieved by using the bossac version from `BOSSA official releases`_.
After installing using default options, the :file:`bossac.exe` must be added to
Windows PATH. A specific bossac executable can be used by passing the
``--bossac`` option, as follows:

.. code-block:: console

    west flash -r bossac --bossac="C:\Program Files (x86)\BOSSA\bossac.exe" --bossac-port="COMx"

.. note::

   WSL is not currently supported.


.. _linkserver-debug-host-tools:
.. _runner_linkserver:

LinkServer Debug  Host Tools
****************************

Linkserver is a utility for launching and managing GDB servers for NXP debug probes,
which also provides a command-line target flash programming capabilities.
Linkserver can be used with the `NXP MCUXpresso for Visual Studio Code`_ implementation,
with custom debug configurations based on GNU tools or as part of a headless solution
for continuous integration and test. LinkServer can be used with MCU-Link, LPC-Link2,
LPC11U35-based and OpenSDA based standalone or on-board debug probes from NXP.

NXP recommends installing LinkServer by using NXP's `MCUXpresso Installer`_.
This method will also install the tools supporting the debug probes below,
including NXP's MCU-Link and LPCScrypt tools.

LinkServer is compatible with the following debug probes:

- :ref:`lpclink2-cmsis-onboard-debug-probe`
- :ref:`mcu-link-cmsis-onboard-debug-probe`
- :ref:`opensda-daplink-onboard-debug-probe`

To use LinkServer with West commands, the install folder should be added to the
:envvar:`PATH` :ref:`environment variable <env_vars>`.  The default installation
path to add is:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

         /usr/local/LinkServer

   .. group-tab:: Windows

      .. code-block:: console

         c:\nxp\LinkServer_<version>

Supported west commands:

1. flash
#. debug
#. debugserver
#. attach

Notes:


1. Probes can be listed with LinkServer:

.. code-block:: console

   LinkServer probes

2. With multiple debug probes attached to the host, use the
LinkServer west runner   ``--probe`` option to pass the probe index.

.. code-block:: console

   west flash --runner=linkserver --probe=3

3. Device-specific settings can be overridden with the west runner for LinkServer with
   the option '--override'. May be used multiple times. The format is dictated
   by LinkServer, e.g.:

.. code-block:: console

   west flash --runner=linkserver --override /device/memory/5/flash-driver=MIMXRT500_SFDP_MXIC_OSPI_S.cfx

4. LinkServer does not install an implicit breakpoint at the reset handler. If
   you would like to single step from the start of their application, you
   will need to add a breakpoint at ``main`` or the reset handler manually.

.. _jlink-debug-host-tools:
.. _runner_jlink:

J-Link Debug Host Tools
***********************

Segger provides a suite of debug host tools for Linux, macOS, and Windows
operating systems:

- J-Link GDB Server: GDB remote debugging
- J-Link Commander: Command-line control and flash programming
- RTT Viewer: RTT terminal input and output
- SystemView: Real-time event visualization and recording

These debug host tools are compatible with the following debug probes:

- :ref:`lpclink2-jlink-onboard-debug-probe`
- :ref:`opensda-jlink-onboard-debug-probe`
- :ref:`mcu-link-jlink-onboard-debug-probe`
- :ref:`jlink-external-debug-probe`
- :ref:`stlink-v21-onboard-debug-probe`

Check if your SoC is listed in `J-Link Supported Devices`_.

Download and install the `J-Link Software and Documentation Pack`_ to get the
J-Link GDB Server and Commander, and to install the associated USB device
drivers. RTT Viewer and SystemView can be downloaded separately, but are not
required.

Note that the J-Link GDB server does not yet support Zephyr RTOS-awareness.

.. _openocd-debug-host-tools:
.. _runner_openocd:

OpenOCD Debug Host Tools
************************

OpenOCD is a community open source project that provides GDB remote debugging
and flash programming support for a wide range of SoCs. A fork that adds Zephyr
RTOS-awareness is included in the Zephyr SDK; otherwise see `Getting OpenOCD`_
for options to download OpenOCD from official repositories.

These debug host tools are compatible with the following debug probes:

- :ref:`opensda-daplink-onboard-debug-probe`
- :ref:`jlink-external-debug-probe`
- :ref:`stlink-v21-onboard-debug-probe`

Check if your SoC is listed in `OpenOCD Supported Devices`_.

.. note:: On Linux, openocd is available though the `Zephyr SDK
   <https://github.com/zephyrproject-rtos/sdk-ng/releases>`_.
   Windows users should use the following steps to install
   openocd:

   - Download openocd for Windows from here: `OpenOCD Windows`_
   - Copy bin and share dirs to ``C:\Program Files\OpenOCD\``
   - Add ``C:\Program Files\OpenOCD\bin`` to 'PATH' environment variable

.. _pyocd-debug-host-tools:
.. _runner_pyocd:

pyOCD Debug Host Tools
**********************

pyOCD is an open source project from Arm that provides GDB remote debugging and
flash programming support for Arm Cortex-M SoCs. It is distributed on PyPi and
installed when you complete the :ref:`gs_python_deps` step in the Getting
Started Guide. pyOCD includes support for Zephyr RTOS-awareness.

These debug host tools are compatible with the following debug probes:

- :ref:`lpclink2-cmsis-onboard-debug-probe`
- :ref:`mcu-link-cmsis-onboard-debug-probe`
- :ref:`opensda-daplink-onboard-debug-probe`
- :ref:`stlink-v21-onboard-debug-probe`

Check if your SoC is listed in `pyOCD Supported Devices`_.

.. _lauterbach-trace32-debug-host-tools:
.. _runner_trace32:

Lauterbach TRACE32 Debug Host Tools
***********************************

`Lauterbach TRACE32`_ is a product line of microprocessor development tools,
debuggers and real-time tracer with support for JTAG, SWD, NEXUS or ETM over
multiple core architectures, including Arm Cortex-A/-R/-M, RISC-V, Xtensa, etc.
Zephyr allows users to develop and program boards with Lauterbach TRACE32
support using :ref:`west <west-flashing>`.

The runner consists of a wrapper around TRACE32 software, and allows a Zephyr
board to execute a custom start-up script (Practice Script) for the different
commands supported, including the ability to pass extra arguments from CMake.
Is up to the board using this runner to define the actions performed on each
command.

Install Lauterbach TRACE32 Software
-----------------------------------

Download Lauterbach TRACE32 software from the `Lauterbach TRACE32 download website`_
(registration required) and follow the installation steps described in
`Lauterbach TRACE32 Installation Guide`_.

Flashing and Debugging
----------------------

Set the :ref:`environment variable <env_vars>` :envvar:`T32_DIR` to the TRACE32
system directory. Then execute ``west flash`` or ``west debug`` commands to
flash or debug the Zephyr application as detailed in :ref:`west-build-flash-debug`.
The ``debug`` command launches TRACE32 GUI to allow debug the Zephyr
application, while the ``flash`` command hides the GUI and perform all
operations in the background.

By default, the ``t32`` runner will launch TRACE32 using the default
configuration file named ``config.t32`` located in the TRACE32 system
directory. To use a different configuration file, supply the argument
``--config CONFIG`` to the runner, for example:

.. code-block:: console

	west flash --config myconfig.t32

For more options, run ``west flash --context -r t32`` to print the usage.

Zephyr RTOS Awareness
---------------------

To enable Zephyr RTOS awareness follow the steps described in
`Lauterbach TRACE32 Zephyr OS Awareness Manual`_.

.. _nxp-s32-debug-host-tools:
.. _runner_nxp_s32dbg:

NXP S32 Debug Probe Host Tools
******************************

:ref:`nxp-s32-debug-probe` is designed to work in conjunction with
`NXP S32 Design Studio for S32 Platform`_.

Download (registration required) NXP S32 Design Studio for S32 Platform and
follow the `S32 Design Studio for S32 Platform Installation User Guide`_ to get
the necessary debug host tools and associated USB device drivers.

Note that Zephyr RTOS-awareness support for the NXP S32 GDB server depends on
the target device. Consult the product release notes for more information.

Supported west commands:

1. debug
#. debugserver
#. attach

Basic usage
-----------

Before starting, add NXP S32 Design Studio installation directory to the system
:ref:`PATH environment variable <env_vars>`. Alternatively, it can be passed to
the runner on each invocation via ``--s32ds-path`` as shown below:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

         west debug --s32ds-path=/opt/NXP/S32DS.3.5

   .. group-tab:: Windows

      .. code-block:: console

         west debug --s32ds-path=C:\NXP\S32DS.3.5

If multiple S32 debug probes are connected to the host via USB, the runner will
ask the user to select one via command line prompt before continuing. The
connection string for the probe can be also specified when invoking the runner
via ``--dev-id=<connection-string>``. Consult NXP S32 debug probe user manual
for details on how to construct the connection string. For example, if using a
probe with serial ID ``00:04:9f:00:ca:fe``:

.. code-block:: console

   west debug --dev-id='s32dbg:00:04:9f:00:ca:fe'

It is possible to pass extra options to the debug host tools via ``--tool-opt``.
When executing ``debug`` or ``attach`` commands, the tool options will be passed
to the GDB client only. When executing ``debugserver``, the tool options will be
passed to the GDB server. For example, to load a Zephyr application to SRAM and
afterwards detach the debug session:

.. code-block:: console

   west debug --tool-opt='--batch'

.. _runner_probe_rs:

probe-rs Debug Host Tools
*************************

probe-rs is an open-source embedded toolkit written in Rust. It provides
out-of-the-box support for a variety of debug probes, including CMSIS-DAP,
ST-Link, SEGGER J-Link, FTDI and built-in USB-JTAG interface on ESP32 devices.

Check `probe-rs Installation`_ for more setup details.

Check if your SoC is listed in `probe-rs Supported Devices`_.

.. _runner_rfp:

Renesas Flash Programmer (RFP) Host Tools
*****************************************

Renesas provides `Renesas Flash Programmer`_ as an official programming tool for Renesas boards
using the Renesas standard boot firmware. It is available as a GUI and CLI.

For boards configured with the ``rfp`` west runner, the RFP CLI can be easily used to flash Zephyr.

Supported west commands:

1. flash

Once downloaded, if ``rfp-cli`` is not placed somewhere in your system PATH, you can pass the location
to ``rfp-cli`` when flashing:

.. code-block:: console

   west flash --rfp-cli ~/Downloads/RFP_CLI_Linux_V31800_x64/linux-x64/rfp-cli

.. _stm32cubeclt-host-tools:
.. _runner_stlink_gdbserver:

STM32CubeCLT Flash & Debug Host Tools
*************************************

STMicroelectronics provides `STM32CubeCLT`_ as an official all-in-one toolset compatible with
Linux |reg|, macOS |reg| and Windows |reg|, allowing the use of STMicroelectronics proprietary
tools within third-party development environments.

It notably provides a GDB debugging server (the *ST-LINK GDB Server*) that can be used to debug
applications on STM32 boards thanks to on-board or external ST-LINK debug probes.

It is compatible with the following debug probes:

- :ref:`stlink-v21-onboard-debug-probe`
- Standalone `ST-LINK-V2`_, `ST-LINK-V3`_, and `STLINK-V3PWR`_ probes

Install STM32CubeCLT
--------------------

The easiest way to get the ST-LINK GDB Server is to install `STM32CubeCLT`_ from STMicroelectronics' website.
A valid email address is needed to receive the downloading link.

Basic usage
-----------

The ST-Link GDB Server can be used through the ``west attach``, ``west debug`` or ``west debugserver`` commands
to debug Zephyr applications.

.. code-block:: console

   west debug --runner stlink_gdbserver

.. note::

   The `STM32CubeProgrammer`_ version contained in the `STM32CubeCLT`_ installation can also be used to flash
   applications. To do so, the dedicated :ref:`STM32CubeProgrammer runner <runner_stm32cubeprogrammer>` should
   be used instead of ``stlink_gdbserver``, as done in the following example:

   .. code-block:: console

      west flash --runner stm32cubeprogrammer

.. _stm32cubeprog-flash-host-tools:
.. _runner_stm32cubeprogrammer:

STM32CubeProgrammer Flash Host Tools
************************************

STMicroelectronics provides `STM32CubeProgrammer`_ (STM32CubeProg) as an official programming tool
for STM32 boards on Linux |reg|, macOS |reg|, and Windows |reg| operating systems.

It provides an easy-to-use and efficient environment for reading, writing, and verifying device memory
through both the debug interface (JTAG and SWD) and the bootloader interface (UART and USB DFU, I2C, SPI, and CAN).

It offers a wide range of features to program STM32 internal memories (such as flash, RAM, and OTP)
as well as external memories.

It also allows option programming and upload, programming content verification, and programming automation
through scripting.

It is delivered in GUI (graphical user interface) and CLI (command-line interface) versions.

It is compatible with the following debug probes:

- :ref:`stlink-v21-onboard-debug-probe`
- :ref:`jlink-external-debug-probe`
- Standalone `ST-LINK-V2`_, `ST-LINK-V3`_, and `STLINK-V3PWR`_ probes

Install STM32CubeProgrammer
---------------------------

The easiest way to get `STM32CubeProgrammer`_ is to download it from STMicroelectronics website.
A valid email address is needed to receive the downloading link.

Alternatively, it can be installed as part of `STM32CubeCLT`_ all-in-one multi-OS command-line toolset
which also includes GDB debugger client and server.

If you have STM32CubeIDE installed on your system, then STM32CubeProg is already present.

Basic usage
-----------

`STM32CubeProgrammer`_ is setup as the default west runner for all active STM32 boards supported by Zephyr.
It can be used through the ``west flash`` command to flash Zephyr applications.

.. code-block:: console

   west flash --runner stm32cubeprogrammer

For advanced usage via the GUI or CLI, check out the `STM32CubeProgrammer User Manual`_.

.. _runner_uf2:

UF2 Uploader
************

The uf2 runner supports flashing some boards using the UF2 (USB Flashing Format).
UF2 is a user-friendly file format designed for drag-and-drop programming via a USB mass storage device.

It relies on the target device entering a special bootloader mode where it appears to the host
as a USB mass storage device.
Once in this mode, the application image can be uploaded by copying a ``.uf2`` file to the
mounted volume.

.. code-block:: console

   west flash --runner uf2

If the UF2 volume is not automatically detected, you may need to manually specify the mount point
using the ``--device`` option:

For more about the UF2 format and its tooling, see `USB Flashing Format (UF2)`_.

.. _J-Link Software and Documentation Pack:
   https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack

.. _J-Link Supported Devices:
   https://www.segger.com/downloads/supported-devices.php

.. _Getting OpenOCD:
   https://openocd.org/pages/getting-openocd.html

.. _OpenOCD Supported Devices:
   https://github.com/zephyrproject-rtos/openocd/tree/latest/tcl/target

.. _pyOCD Supported Devices:
   https://github.com/pyocd/pyOCD/tree/main/pyocd/target/builtin

.. _OpenOCD Windows:
    https://gnutoolchains.com/arm-eabi/openocd/

.. _Lauterbach TRACE32:
    https://www.lauterbach.com/

.. _Lauterbach TRACE32 download website:
   https://www.lauterbach.com/download_trace32.html

.. _Lauterbach TRACE32 Installation Guide:
   https://www2.lauterbach.com/pdf/installation.pdf

.. _Lauterbach TRACE32 Zephyr OS Awareness Manual:
	https://www2.lauterbach.com/pdf/rtos_zephyr.pdf

.. _BOSSA official releases:
	https://github.com/shumatech/BOSSA/releases

.. _NXP MCUXpresso for Visual Studio Code:
	https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-for-visual-studio-code:MCUXPRESSO-VSC

.. _MCUXpresso Installer:
	https://github.com/nxp-mcuxpresso/vscode-for-mcux/wiki/Dependency-Installation

.. _NXP S32 Design Studio for S32 Platform:
   https://www.nxp.com/design/software/development-software/s32-design-studio-ide/s32-design-studio-for-s32-platform:S32DS-S32PLATFORM

.. _Renesas Flash Programmer:
   https://www.renesas.com/en/software-tool/renesas-flash-programmer-programming-gui

.. _S32 Design Studio for S32 Platform Installation User Guide:
   https://www.nxp.com/webapp/Download?colCode=S32DSIG

.. _probe-rs Installation:
   https://probe.rs/docs/getting-started/installation/

.. _probe-rs Supported Devices:
   https://probe.rs/targets/

.. _STM32CubeCLT:
   https://www.st.com/en/development-tools/stm32cubeclt.html

.. _STM32CubeProgrammer:
   https://www.st.com/en/development-tools/stm32cubeprog.html

.. _STM32CubeProgrammer User Manual:
   https://www.st.com/resource/en/user_manual/um2237-stm32cubeprogrammer-software-description-stmicroelectronics.pdf

.. _ST-LINK-V2:
   https://www.st.com/en/development-tools/st-link-v2.html

.. _ST-LINK-V3:
   https://www.st.com/en/development-tools/stlink-v3set.html

.. _STLINK-V3PWR:
   https://www.st.com/en/development-tools/stlink-v3pwr.html

.. _USB Flashing Format (UF2):
   https://github.com/microsoft/uf2
