.. zephyr:code-sample:: mec_assy6941-tach-pwm-loopback
   :name: Microchip XEC tachometer PWM loopback
   :relevant-api: sensor_interface pwm_interface

   Check the XEC TACH RPM conversion against a PWM generated pulse train.

Overview
********

This sample turns one XEC PWM channel into a fan emulator and fly-wires it to
all four tachometer inputs, so that the RPM the
:dtcompatible:`microchip,xec-tach` driver reports can be compared against the
frequency that was programmed rather than merely printed.

The PWM block and the tachometer share a 100 kHz clock, which is what makes an
exact reference possible: the sample programs a period that is a whole number of
100 kHz clocks, so the count the tachometer latches is exact and the RPM the
driver must report is a known number.

It sweeps six emulated fan frequencies, measures every enabled tachometer at
each one, prints the expected and measured RPM side by side, and finishes with a
single machine-checkable line.

Wiring
******

Connect the PWM0 output to all four TACH inputs and share the board ground:

.. list-table::
   :header-rows: 1

   * - Signal
     - Pin
     - Direction
   * - PWM0
     - GPIO053
     - out
   * - TACH0
     - GPIO050
     - in
   * - TACH1
     - GPIO051
     - in
   * - TACH2
     - GPIO052
     - in
   * - TACH3
     - GPIO033
     - in

.. caution::

   Check the Assy 6941 schematic before jumpering at a fan header. Those pins
   may carry pull-ups, series resistors or level shifters intended for a real
   fan, none of which the tachometer inputs need here, and one of which may load
   the PWM output enough to matter.

What is measured
****************

``app.overlay`` fixes ``tach-edges`` at 2, 3, 5 and 9 on ``tach0`` through
``tach3``, so a single build exercises all four measurement window widths at
once. Only ``pulses-per-round`` varies between builds, through four overlays
carrying a Latin square:

.. list-table::
   :header-rows: 1

   * - Overlay
     - ``tach0`` (2 edges)
     - ``tach1`` (3 edges)
     - ``tach2`` (5 edges)
     - ``tach3`` (9 edges)
   * - ``ppr-a.overlay``
     - 1
     - 2
     - 3
     - 4
   * - ``ppr-b.overlay``
     - 2
     - 1
     - 4
     - 3
   * - ``ppr-c.overlay``
     - 3
     - 4
     - 1
     - 2
   * - ``ppr-d.overlay``
     - 4
     - 3
     - 2
     - 1

The four builds together cover all 16 combinations, and any single build already
covers every edge setting and every divisor.

The reference does not assume the mapping under test. With a period of ``k``
clocks and a window of ``whp`` half TACH periods the latched count is
``whp * k / 2``, so the RPM the driver must report is
``60 * 100000 / (pulses_per_round * k)`` - the window width cancels. A driver
that mis-maps ``tach-edges`` to a window width therefore shows up as a factor of
two or four, which is far outside the 0.8 % worst-case tolerance, rather than
cancelling out of both sides.

The sweep runs at 200, 125, 100, 62.5, 25 and 12.5 Hz, which is 6000 down to
375 RPM at two TACH periods per revolution. It ends by disabling the PWM output
and requiring every instance to report exactly 0 RPM, which is the only coverage
anywhere of the saturated-counter path.

Building and Running
********************

Without an extra overlay every instance takes the ``pulses-per-round`` default
of 2:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/microchip/mec_assy6941/tach_pwm_loopback
   :board: mec_assy6941/mec1753_qsz
   :goals: build flash
   :compact:

Select a row of the matrix with ``EXTRA_DTC_OVERLAY_FILE``, which adds to
``app.overlay`` rather than replacing it:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/microchip/mec_assy6941/tach_pwm_loopback
   :board: mec_assy6941/mec1753_qsz
   :gen-args: -DEXTRA_DTC_OVERLAY_FILE=ppr-b.overlay
   :goals: build flash
   :compact:

Sample output
=============

.. code-block:: console

   Microchip XEC tachometer PWM loopback self test

   Fly-wire pwm@40005800 (PWM0, GPIO053) to the TACH inputs GPIO050, GPIO051,
   GPIO052 and GPIO033, sharing the board ground.

   instance       edges  half periods  pulses/rev
   tach@40006000      2             1           1
   tach@40006010      3             2           2
   tach@40006020      5             4           3
   tach@40006030      9             8           4

   --- 200.0 Hz, period 500 clocks of 100 kHz ---
   instance         expected RPM   measured RPM       error  result
   tach@40006000   12000.000000   12000.000000          0ppm  PASS
   tach@40006010    6000.000000    6000.000000          0ppm  PASS
   tach@40006020    4000.000000    4000.000000          0ppm  PASS
   tach@40006030    3000.000000    3000.000000          0ppm  PASS

   ...

   --- PWM output disabled: every instance must report 0 RPM ---
   tach@40006000       0.000000 RPM  PASS
   tach@40006010       0.000000 RPM  PASS
   tach@40006020       0.000000 RPM  PASS
   tach@40006030       0.000000 RPM  PASS

   76 of 76 checks passed
   TACH XEC PWM loopback: PASS

Running under twister
=====================

The fly-wire is declared as a fixture, so twister builds all five scenarios in
CI and only executes them when the wiring is present:

.. code-block:: console

   west twister -T samples/boards/microchip/mec_assy6941/tach_pwm_loopback \
     --device-testing --fixture tach_xec_pwm_loopback \
     -p mec_assy6941/mec1753_qsz --device-serial /dev/ttyUSB0

Limitations
***********

The 2-edge setting measures one *half* TACH period, so the reading it produces
depends on the duty cycle of the tachometer output. It is exact here only
because the emulated waveform is a 50 % square wave. A real fan whose tach
output is asymmetric will read differently at ``tach-edges = 2`` than at 3, 5 or
9, and that is a property of the hardware measurement window, not a driver
defect. The 3, 5 and 9 settings span whole TACH periods and are duty cycle
independent.

The driver enables the tachometer input filter, which discards pulses narrower
than two 100 kHz periods. The shortest half period this sample emits is 2.5 ms,
two orders of magnitude clear of that, so the filter has no effect here. A
faster emulated fan would eventually run into it.
