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

A second mode, ``CONFIG_APP_RAW_COUNT_MODE``, replaces the sweep with a direct
measurement of the count the block latches, driven from an external generator
instead of the on-chip PWM. It answers a question the loopback structurally
cannot - whether the latched count really is the number of clocks the measurement
window took, which it is not - and is how the ``count + 1`` correction in the
driver was established. See `Measuring the latched count offset`_.

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

Clock reference
***************

``app.overlay`` points the 48 MHz PLL at the 32.768 kHz crystal rather than at
the internal silicon oscillator, and ``prj.conf`` enables
:kconfig:option:`CONFIG_CLOCK_CONTROL` so that the PCR driver applies it. On an
Assembly 6949 CPU daughter card this needs jumpers on JP1 1-2 and JP2 1-2.

The tachometer counts the 48 MHz clock divided by 480, so the accuracy of that
reference is the accuracy of every RPM the driver reports. Referenced to the
silicon oscillator the PLL is specified only to 46.56 to 49.44 MHz, and the part
this sample was run on measured 99042 Hz on the divided clock, 0.96 % low;
referenced to a crystal it is specified to 47.5 to 48.5 MHz, and the same part
measured 100100 Hz. The driver takes the ``clock-frequency`` property of the node
as exact, and that property defaults to the nominal 100 kHz, so whatever the
reference is off by appears directly in the reading until a board that has
measured its own clock sets it.

Setting it changes nothing here. This sample derives the RPM it expects from the
same property, so a calibrated instance still passes: the number the sample
checks is the conversion, not the clock. The configuration table it prints on
startup shows the value each instance is using, which is worth a glance if the
readings look uniformly shifted.

.. note::

   If the crystal does not start - no daughter card, missing jumpers, unpopulated
   parts - the PCR driver falls back to the silicon oscillator and carries on
   without reporting anything, so the only symptom is a reading that is off by up
   to a few percent. The raw count mode's implied input frequency, compared
   against what the generator is set to, is the check for that.

The loopback sweep is blind to this, because PWM0 is divided from the same clock:
a slow clock stretches the emulated fan period and the measurement window by the
same factor and the comparison still passes. It is the raw count mode, and any
comparison against a real fan or an external generator, that sees it.

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
clocks and a window of ``whp`` half TACH periods the window spans ``whp * k / 2``
clocks, so the RPM the driver must report is
``60 * clock_frequency / (pulses_per_round * k)`` - the window width cancels. A
driver that mis-maps ``tach-edges`` to a window width therefore shows up as a
factor of two or four, which is far outside the 0.4 % worst-case tolerance,
rather than cancelling out of both sides.

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

   instance       edges  half periods  pulses/rev  clock Hz
   tach@40006000      2             1           1    100000
   tach@40006010      3             2           2    100000
   tach@40006020      5             4           3    100000
   tach@40006030      9             8           4    100000

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

Measuring the latched count offset
**********************************

The RPM the driver reports is only as accurate as the assumption behind it: that
the latched count is the number of 100 kHz clocks the measurement window took.
It is not - the block latches one less, so the driver converts ``count + 1``.
This mode is the measurement that established that, and is the way to confirm it
still holds on a part or a series it has not run on.

The loopback cannot establish it. PWM0 is derived from the same 100 kHz clock the
tachometer counts, so every emulated edge arrives on a counter clock edge, and an
off-by-one at that coincidence is indistinguishable from an off-by-one at any
phase - which is why the loopback saw the deficit in all 360 of its readings
without being able to attribute it. ``CONFIG_APP_RAW_COUNT_MODE`` replaces the
sweep with a direct measurement of the residual offset ``c`` in

.. code-block:: none

   count as converted by the driver = whp * M / 2 + c

driven from a source that is not locked to the SoC clock, where ``M`` is the
input period in 100 kHz clocks and ``whp`` the window width in half TACH
periods. The count is recovered from the RPM the driver reported, so ``c`` covers
the whole path and a correct driver reports **0**. Two instances whose windows
are in a 1:2 ratio give

.. code-block:: none

   c = 2 * mean(count at whp) - mean(count at 2 * whp)

in which ``M`` cancels. The accuracy of the generator therefore never enters the
result and its frequency does not even have to be known - which is what makes
this measurable on the bench rather than only against a calibrated reference.
``app.overlay`` sets ``tach-edges`` to 2, 3, 5 and 9, so the windows are 1, 2, 4
and 8 half periods and three such pairs are available. ``pulses-per-round``
cancels when the count is recovered from the reported RPM, so the ``ppr-*``
overlays make no difference here.

Averaging is what recovers the fraction: the counter truncates, so a single
reading is only ever within one count of the true window width, and the mean is
the true width only if the phase between the input and the 100 kHz clock is
spread across the readings. Hence the generator, and hence a frequency that is
**not** a round number of 100 kHz clocks.

Set-up
======

Remove the PWM0 fly-wire, then drive all four tachometer inputs from one
generator, sharing the board ground:

.. list-table::
   :header-rows: 1

   * - Setting
     - Value
   * - Waveform
     - square, 50 % duty
   * - Level
     - 0 to 3.3 V, i.e. 3.3 Vpp with a 1.65 V offset
   * - Frequency
     - 120 Hz, or anything else whose period is not a whole number of 10 µs

.. caution::

   Set the amplitude and offset before connecting anything. A generator left at
   its default drives about ±5 V into a high impedance load, and of these four
   pins only GPIO050 and GPIO051 are over-voltage protected; GPIO052 sits on the
   VTR2 bank, which may be powered at 1.8 V. A 100 Ω series resistor costs
   nothing and limits the damage if the amplitude is wrong.

120 Hz is 833.33 clocks, which leaves every one of the four windows a third of a
clock from an integer, so every instance's counter alternates between adjacent
values - the case that distinguishes the two answers. At 200 Hz exactly, every
window would end on the same clock edge and the measurement would say nothing
the loopback has not already said. Prefer a frequency whose fractional part is
robust rather than one that has to be exact: the counting clock is nominally
100 kHz but is specified only to a few percent, so a dial computed against
100000 is already off by more than the fraction it was chosen for.

Keep the half period above 20 µs, which the driver's input filter discards, and
the widest window - eight half periods, four input periods - below the 655 ms
counter saturation: between about 7 Hz and 25 kHz.

Both hardware runs used ``CONFIG_APP_RAW_COUNT_READINGS=200``. At the default of
50 the pair estimates can straddle the quarter-count agreement window and the
result comes back inconclusive.

.. important::

   Too fast an input does not fail loudly. Driving one input with a 91 kHz square
   wave, whose 5.5 µs half period is well under the filter's 20 µs floor, latched
   counts of 13 and 14 at 3 edges rather than the 1 the period calls for: the
   filter emits a subharmonic, one window per 12 or 13 input periods, and the
   driver reports a plausible RPM derived from it. That also makes the offset
   unmeasurable there, because at 91 kHz one input period is 1.1 clocks, so an
   unknown divide ratio shifts the count by the same one count the measurement is
   looking for. The frequency the instances imply, printed below, is the check:
   it must agree with the dial.

.. zephyr-app-commands::
   :zephyr-app: samples/boards/microchip/mec_assy6941/tach_pwm_loopback
   :board: mec_assy6941/mec1753_qsz
   :gen-args: -DEXTRA_CONF_FILE=raw-count.conf -DEXTRA_DTC_OVERLAY_FILE=raw-count.overlay
   :goals: build flash
   :compact:

``raw-count.overlay`` disables PWM0. A disabled PWM still drives its pin, so
leaving it enabled would put the generator into an output driver if the fly-wire
were still fitted.

Reading the result
==================

.. code-block:: console

   --- latched counts, 200 readings per instance ---
   instance       edges  half periods  readings    min    max  mean count
   tach@40006000      2             1       200    416    417     416.245
   tach@40006010      3             2       200    832    833     832.520
   tach@40006020      5             4       200   1664   1666    1665.020
   tach@40006030      9             8       200   3329   3331    3330.225

   --- offset implied by each pair of windows in a 1:2 ratio ---
   instance a     instance b      whp a  whp b   offset c
   tach@40006000  tach@40006010       1      2   -0.030
   tach@40006010  tach@40006020       2      4    0.020
   tach@40006020  tach@40006030       4      8   -0.185

   --- input implied by each instance at that offset ---
   instance      period clocks  clock Hz  frequency Hz
   tach@40006000       832.620    100000       120.103
   tach@40006010       832.585    100000       120.108
   tach@40006020       832.543    100000       120.114
   tach@40006030       832.573    100000       120.110

   offset c = -0.065 counts from 3 pairs, spread 0.205

   The count the driver converts is the number of 100 kHz clocks the
   measurement window took, so the conversion is correct as it stands.
   TACH XEC latched count offset: 0.000 counts

An offset of 0 is the expected result: the count the driver converts is the
elapsed window. A result of -1 means the ``count + 1`` correction in
``tach_xec_channel_get()`` has been lost, or that the block on the part under
test does not apply the offset the correction assumes. Anything else, or pair
estimates that disagree by more than a quarter of a count, is a measurement
problem rather than an answer.

Two results are refusals rather than answers. If no instance ever latched two
different counts the input is locked to the tachometer clock, which is the
ambiguity this mode exists to break, and the offset is reported as inconclusive.
The implied period table is a wiring check: all four instances must agree, and
must agree with the generator, or one of them is not connected to it.

What the offset was measured to be
==================================

Two runs on an Assy 6941 ``mec1753_qlj``, 200 readings on each of the four
instances, with the 48 MHz PLL referenced first to the internal silicon
oscillator and then to a 32.768 kHz crystal - a 1 % change in the counting clock
between them - put the raw latched count at ``whp * M / 2 - 1.0``, which is what
the driver now corrects. The strongest form of the evidence is not the pair
estimate but the consistency of the four windows: taking the counts at face value
makes the input period implied by the four instances disagree by 1.8 clocks in a
monotonic ramp whose deviations are ``2 / whp`` to three digits, while adding one
count collapses them to a spread of 80 ppm across windows spanning 8:1. No
multiplicative error - clock rate, generator calibration, a mis-mapped
``tach-edges`` - can produce that shape.

The datasheet does not document the offset. §29.9.1 says only that the internal
counter is reset to zero after being copied into the register, which is
consistent with the latch and the reset landing in the same clock and that clock
not incrementing the counter.

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
