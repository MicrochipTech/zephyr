.. zephyr:code-sample:: xec_cct_kt
   :name: Microchip XEC CCT kernel timer

   Use the MEC capture-compare timer as the Zephyr system timer with a 32 KHz
   RTOS-timer low-power companion.

Overview
********

This sample runs the Microchip MEC capture-compare timer (CCT) as the Zephyr
system timer on the MEC1753-QLJ. The CCT is a free-running 32-bit up-counter
clocked from the SoC PLL (48 MHz / 4 = 12 MHz) whose Compare0 match generates
the kernel tick.

Because the PLL is powered down in deep low-power states, the CCT cannot keep
time there. The driver delegates timekeeping across those states to the
always-on 32 KHz RTOS timer through the system-timer low-power companion
*hooks* interface (``CONFIG_SYSTEM_TIMER_LPM_COMPANION_HOOKS``): the CCT driver
implements :c:func:`z_sys_clock_lpm_enter` / :c:func:`z_sys_clock_lpm_exit`
against the RTOS-timer registers and reconciles the elapsed time on wake.

The application sleeps in a loop long enough for the PM policy to enter a
low-power state, then prints the requested sleep time alongside the measured
uptime delta and free-running cycle-counter delta. Correct timekeeping shows
all three agreeing.

Device tree
***********

All board-specific device-tree values live in the sample overlay
:file:`boards/mec_assy6941_mec1753_qlj.overlay`:

* ``cctmr0`` is given the ``microchip,xec-cct-ktimer`` compatible, the
  Compare0/Compare1 interrupts and GIRQ encodings, the PCR sleep-enable, and
  the 48 MHz input clock, and is enabled.
* ``rtimer`` (the LPM companion) is disabled so the CCT kernel-timer driver is
  selected; the CCT driver still reaches the RTOS-timer registers by node
  label for the companion hooks.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/boards/microchip/xec_cct_kt
   :board: mec_assy6941/mec1753_qlj
   :goals: build flash
   :compact:

Sample Output
*************

.. code-block:: console

   XEC CCT kernel timer sample
   system timer HW cycles/sec: 12000000
   ticks/sec: 10000
   iter 0: requested 3000 ms, uptime +3000 ms, cycles +36000000 (3000 ms by cycle count)
   ...
   Sample complete
